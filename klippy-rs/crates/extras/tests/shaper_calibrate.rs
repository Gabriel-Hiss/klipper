//! Parity tests for `klippy/extras/shaper_calibrate.py`.
//!
//! Every expected value comes from `pyoracle` (live CPython 3.8 with frozen
//! `klippy/` and NumPy 1.16.4) and floats compare bit for bit. Inputs are the
//! recorded KE accelerometer batch of `support/accel_batch.rs`. Tests skip
//! with a message when `pyoracle::Oracle::start()` returns `None` (no
//! `$KLIPPY_PYTHON`).
//!
//! The port has no G-code commands, config options or status fields
//! (inventory contracts are empty); the exercised surface is the numeric
//! pipeline (`calc_freq_response`, `_psd`, `fit_shaper`, ...), CSV saving
//! and `save_params` / `apply_params`.
use extras::shaper_calibrate::{
    CalibrationData, CalibrationResult, ShaperCalibrate, ShaperFreqs, Sample, Shaper, MIN_FREQ,
    MAX_FREQ, WINDOW_T_SEC, MAX_SHAPER_FREQ, TEST_DAMPING_RATIOS, AUTOTUNE_SHAPERS,
};
use extras::input_shaper::InputShaperApi;
use extras::shaper_defs::{init_shaper, DEFAULT_DAMPING_RATIO, SHAPER_VIBRATION_REDUCTION};
use klippy_api::config::{Config, DeprecateGcodeOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd,
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts, OutputCallback,
    RegisterCommandOpts,
};
use klippy_api::mcu::Mcu;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, PrinterApi, Printer, ModuleFactory};
use indexmap::IndexMap;
use pycompat::PyValue;
use recording::Value;
use reactor::{Reactor, ReactorMutex};
#[path = "support/accel_batch.rs"]
mod accel_batch;
use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Fakes: Printer, ConfigFile, GCode, GCmd, InputShaper.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
}

impl Default for FakePrinter {
    fn default() -> Self {
        FakePrinter { objects: Mutex::new(IndexMap::new()) }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { ("".to_string(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let map = self.objects.lock();
        match map.get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::ReturnNone => Ok(None),
                _ => Err(KlippyError::Config(format!("Unknown object {name}"))),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _cb: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

fn fake_printer() -> Printer { Arc::new(FakePrinter::default()) }

struct FakeConfigFile {
    sets: Mutex<Vec<(String, String, String)>>,
    printer: Printer,
}
impl FakeConfigFile {
    fn new() -> Self { FakeConfigFile { sets: Mutex::new(Vec::new()), printer: fake_printer() } }
}
impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn read_config(&self, _f: &str) -> KResult<Config> { Err(KlippyError::Config("unused".to_string())) }
    fn read_main_config(&self) -> KResult<Config> { Err(KlippyError::Config("unused".to_string())) }
    fn log_config(&self, _c: &Config) {}
    fn check_unused_options(&self, _c: &Config) -> KResult<()> { Ok(()) }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(&self, _s: &str, _o: &str, _v: Option<PyValue>, _m: Option<String>) {}
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::None }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.sets.lock().push((section.to_string(), option.to_string(), value.to_string()));
        Ok(())
    }
    fn remove_section(&self, _section: &str) {}
}

#[derive(Debug)]
struct FakeGcmd {
    params: IndexMap<String, String>,
}
impl GCodeCommandApi for FakeGcmd {
    fn get(&self, _n: &str, _o: GetOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn get_int(&self, _n: &str, _o: GGetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn get_float(&self, _n: &str, _o: GGetFloatOpts) -> KResult<Option<f64>> { Ok(None) }
    fn get_command(&self) -> String { "SET_INPUT_SHAPER".to_string() }
    fn get_commandline(&self) -> String { "".to_string() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.clone() }
    fn get_raw_command_parameters(&self) -> String { "".to_string() }
    fn respond_info(&self, _m: &str, _log: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool { false }
}

struct FakeGcode {
    last_params: Mutex<IndexMap<String, String>>,
}
impl FakeGcode {
    fn new() -> Self { FakeGcode { last_params: Mutex::new(IndexMap::new()) } }
}
impl GCodeApi for FakeGcode {
    fn register_command(&self, _c: &str, _f: Option<GCodeCommandHandler>, _o: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> { Ok(None) }
    fn register_mux_command(&self, _c: &str, _k: &str, _v: Option<&str>, _f: GCodeCommandHandler, _d: Option<&str>) -> KResult<()> { Ok(()) }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _t: f64) -> PyValue { PyValue::None }
    fn register_output_handler(&self, _cb: OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> ReactorMutex { Reactor::new(false).mutex(true) }
    fn create_gcode_command(&self, _c: &str, _l: &str, params: &IndexMap<String, String>) -> GCmd {
        *self.last_params.lock() = params.clone();
        Arc::new(FakeGcmd { params: params.clone() })
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool { false }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

struct FakeInputShaper {
    calls: Mutex<Vec<IndexMap<String, String>>>,
}
impl FakeInputShaper {
    fn new() -> Self { FakeInputShaper { calls: Mutex::new(Vec::new()) } }
}
impl InputShaperApi for FakeInputShaper {
    fn disable_shaping(&self) -> KResult<()> {
        unreachable!("apply_params only sends SET_INPUT_SHAPER")
    }
    fn enable_shaping(&self) -> KResult<()> {
        unreachable!("apply_params only sends SET_INPUT_SHAPER")
    }
    fn cmd_set_input_shaper(&self, gcmd: &GCmd) -> KResult<()> {
        self.calls.lock().push(gcmd.get_command_parameters());
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn as_items(v: &Value) -> Vec<&Value> {
    match v {
        Value::List(items) | Value::Tuple(items) => items.iter().collect(),
        other => panic!("expected list/tuple, got {other:?}"),
    }
}

fn float_of(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().unwrap(),
        Value::Int(i) => *i as f64,
        other => panic!("expected float, got {other:?}"),
    }
}

fn py_floats(v: &Value) -> Vec<f64> {
    as_items(v).iter().map(|it| float_of(it)).collect()
}

fn py_nested_floats(v: &Value) -> Vec<f64> {
    let mut out = Vec::new();
    fn walk(v: &Value, out: &mut Vec<f64>) {
        match v {
            Value::List(items) | Value::Tuple(items) => items.iter().for_each(|it| walk(it, out)),
            other => out.push(float_of(other)),
        }
    }
    walk(v, &mut out);
    out
}

fn same(a: f64, b: f64) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

fn assert_bits(what: &str, actual: f64, expected: f64) {
    assert!(same(actual, expected), "{what}: {actual:?} vs Python {expected:?}");
}

fn assert_all_bits(what: &str, actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len(), "{what}: length");
    for (i, (&a, &e)) in actual.iter().zip(expected).enumerate() {
        assert!(same(a, e), "{what}[{i}]: {a:?} vs Python {e:?}");
    }
}

/// A Python expression that parses to the same bits.
fn py_float(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "float('inf')" } else { "float('-inf')" }.to_string()
    } else {
        format!("{v:?}")
    }
}

fn lit(vs: &[f64]) -> String {
    format!("[{}]", vs.iter().map(|&v| py_float(v)).collect::<Vec<_>>().join(","))
}

fn py_shaper(shaper: &Shaper) -> String {
    format!("({}, {})", lit(&shaper.0), lit(&shaper.1))
}

fn py_opt(v: Option<f64>) -> String {
    v.map_or_else(|| "None".to_string(), py_float)
}

fn py_freqs(freqs: Option<&ShaperFreqs>) -> String {
    match freqs {
        None => "None".to_string(),
        Some(ShaperFreqs::Range(a, b, c)) => format!("({}, {}, {})", py_opt(*a), py_opt(*b), py_opt(*c)),
        Some(ShaperFreqs::List(v)) => lit(v),
    }
}

fn eval(py: &mut pyoracle::Oracle, expr: &str) -> Value {
    py.eval(expr).unwrap_or_else(|e| panic!("{expr}: {}: {}", e.class_name, e.message))
}

fn eval_float(py: &mut pyoracle::Oracle, expr: &str) -> f64 {
    float_of(&eval(py, expr))
}

fn eval_floats(py: &mut pyoracle::Oracle, expr: &str) -> Vec<f64> {
    py_floats(&eval(py, expr))
}

fn eval_strings(py: &mut pyoracle::Oracle, expr: &str) -> Vec<String> {
    as_items(&eval(py, expr))
        .iter()
        .map(|v| match v {
            Value::Str(s) => s.clone(),
            other => panic!("{expr}: expected str, got {other:?}"),
        })
        .collect()
}

/// The Rust call and the Python statement must raise the same exception.
fn check_err<T: std::fmt::Debug>(what: &str, actual: KResult<T>, expected: pyoracle::OracleResult) {
    let expected = expected.expect_err(&format!("{what}: Python did not raise"));
    let actual = actual.expect_err(&format!("{what}: port did not raise"));
    assert_eq!(
        (actual.class_name().to_string(), actual.message().to_string()),
        (expected.class_name, expected.message),
        "{what}"
    );
}

fn start_oracle() -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: KLIPPY_PYTHON not set (no oracle)");
        return None;
    };
    let setup = "import numpy as np\nimport extras.shaper_calibrate as shaper_calibrate\nfrom extras.shaper_calibrate import ShaperCalibrate\nimport extras.shaper_defs as shaper_defs\nsc = ShaperCalibrate(None)\n";
    py.exec(setup).unwrap();
    Some(py)
}

fn samples_of(rows: &[(f64, f64, f64, f64)]) -> Vec<Sample> {
    rows.iter().map(|&(t, x, y, z)| [t, x, y, z]).collect()
}

/// The recorded batch (first `count` rows) through `calc_freq_response` on
/// both sides: the Rust result, and `var` in the oracle namespace.
fn batch_data(py: &mut pyoracle::Oracle, var: &str, count: usize) -> CalibrationData {
    let rows = accel_batch::ke_batch();
    let rows = &rows[..count];
    py.exec(&format!(
        "{var} = sc.calc_freq_response('ke{count}', np.array({}))\n{var}.set_numpy(np)",
        accel_batch::py_rows(rows)
    ))
    .unwrap();
    ShaperCalibrate::new(None)
        .calc_freq_response(&format!("ke{count}"), Some(&samples_of(rows)))
        .unwrap()
        .expect("batch data")
}

fn check_calibration_data(py: &mut pyoracle::Oracle, what: &str, var: &str, data: &CalibrationData) {
    for (field, actual) in [
        ("freq_bins", &data.freq_bins),
        ("psd_sum", &data.psd_sum),
        ("psd_x", &data.psd_x),
        ("psd_y", &data.psd_y),
        ("psd_z", &data.psd_z),
    ] {
        let expected = eval_floats(py, &format!("{var}.{field}.tolist()"));
        assert_all_bits(&format!("{what}.{field}"), actual, &expected);
    }
}

/// A scalar field of `CalibrationResult` by name.
type ResultField = (&'static str, fn(&CalibrationResult) -> f64);

/// Compares `CalibrationResult`s with the Python list `expr`, every field
/// bit for bit.
fn check_results(py: &mut pyoracle::Oracle, what: &str, expr: &str, actual: &[CalibrationResult]) {
    let names = eval_strings(py, &format!("[r.name for r in {expr}]"));
    let actual_names: Vec<String> = actual.iter().map(|r| r.name.clone()).collect();
    assert_eq!(actual_names, names, "{what}: names");
    let fields: [ResultField; 5] = [
        ("freq", |r| r.freq),
        ("vibrs", |r| r.vibrs),
        ("smoothing", |r| r.smoothing),
        ("score", |r| r.score),
        ("max_accel", |r| r.max_accel),
    ];
    for (field, get) in fields {
        let expected = eval_floats(py, &format!("[float(r.{field}) for r in {expr}]"));
        let got: Vec<f64> = actual.iter().map(get).collect();
        assert_all_bits(&format!("{what}.{field}"), &got, &expected);
    }
    for field in ["freq_bins", "vals"] {
        let expected = eval_floats(py, &format!("[v for r in {expr} for v in r.{field}.tolist()]"));
        let got: Vec<f64> = actual
            .iter()
            .flat_map(|r| if field == "vals" { r.vals.clone() } else { r.freq_bins.clone() })
            .collect();
        assert_all_bits(&format!("{what}.{field}"), &got, &expected);
    }
}

// ---------------------------------------------------------------------------
// ModuleHeader.
// ---------------------------------------------------------------------------

#[test]
fn module_header_constants() {
    assert_eq!(MIN_FREQ.to_bits(), 5.0f64.to_bits());
    assert_eq!(MAX_FREQ.to_bits(), 200.0f64.to_bits());
    assert_eq!(WINDOW_T_SEC.to_bits(), 0.5f64.to_bits());
    assert_eq!(MAX_SHAPER_FREQ.to_bits(), 150.0f64.to_bits());
    assert_eq!(DEFAULT_DAMPING_RATIO.to_bits(), 0.1f64.to_bits());
    assert_eq!(SHAPER_VIBRATION_REDUCTION.to_bits(), 20.0f64.to_bits());
    assert_eq!(TEST_DAMPING_RATIOS, [0.075, 0.1, 0.15]);
    assert_eq!(AUTOTUNE_SHAPERS, ["zv", "mzv", "ei", "2hump_ei", "3hump_ei"]);
    let f: ModuleFactory = extras::shaper_calibrate::module_factory();
    assert!(f.load_config.is_some());
}

// ---------------------------------------------------------------------------
// SplitIntoWindows.
// ---------------------------------------------------------------------------

#[test]
fn split_into_windows_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let x = (0..200).map(|i| i as f64 * 0.01 - 1.0).collect::<Vec<_>>();
    let rust = ShaperCalibrate::new(None);
    let ws = rust._split_into_windows(&x, 64, 32);
    let expr = format!(r#"sc._split_into_windows(np.array({}), 64, 32).tolist()"#, lit(&x));
    let expected = py_nested_floats(&eval(&mut py, &expr));
    // Compare per-window: flatten in order.
    let flat: Vec<f64> = ws.into_iter().flatten().collect();
    assert_all_bits("windows", &flat, &expected);
    let expected = py_nested_floats(&eval(&mut py, "sc._split_into_windows(np.array([1.0, 2.0, 3.0, 4.0, 5.0]), 3, 1).tolist()"));
    let flat: Vec<f64> = rust._split_into_windows(&[1.0, 2.0, 3.0, 4.0, 5.0], 3, 1).into_iter().flatten().collect();
    assert_all_bits("overlap edge", &flat, &expected);
}

// ---------------------------------------------------------------------------
// _psd / calc_freq_response.
// ---------------------------------------------------------------------------

#[test]
fn psd_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let rows = accel_batch::ke_batch();
    let fs = rows.len() as f64 / (rows[rows.len() - 1].0 - rows[0].0);
    let x: Vec<f64> = rows.iter().map(|r| r.1).collect();
    py.exec(&format!("x = np.array({})", lit(&x))).unwrap();
    let rust = ShaperCalibrate::new(None);
    // Powers of two as `calc_freq_response` picks, plus composite, prime-
    // factor and degenerate window sizes.
    for nfft in [2048usize, 1024, 64, 360, 1001, 2, 1] {
        let (freqs, psd) = rust._psd(&x, fs, nfft).unwrap();
        py.exec(&format!("f, p = sc._psd(x, {fs:?}, {nfft})")).unwrap();
        assert_all_bits(&format!("_psd freqs nfft={nfft}"), &freqs, &eval_floats(&mut py, "f.tolist()"));
        assert_all_bits(&format!("_psd nfft={nfft}"), &psd, &eval_floats(&mut py, "p.tolist()"));
    }
    check_err(
        "_psd nfft=0",
        rust._psd(&[1.0, 2.0], 100.0, 0),
        py.exec("sc._psd(np.array([1.0, 2.0]), 100.0, 0)"),
    );
    check_err(
        "_psd shorter than the overlap",
        rust._psd(&[1.0, 2.0], 100.0, 16),
        py.exec("sc._psd(np.array([1.0, 2.0]), 100.0, 16)"),
    );
}

#[test]
fn calc_freq_response_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let data = batch_data(&mut py, "d", accel_batch::SAMPLES);
    check_calibration_data(&mut py, "calc_freq_response", "d", &data);
    // A shorter capture (another window count).
    let data = batch_data(&mut py, "d2", 5000);
    check_calibration_data(&mut py, "calc_freq_response 5000", "d2", &data);
    let rust = ShaperCalibrate::new(None);
    // N <= M returns None.
    let tiny: Vec<Sample> = (0..10).map(|i| [i as f64 / 3200.0, 0.0, 0.0, 0.0]).collect();
    let tiny_rows: Vec<String> = tiny.iter().map(|s| lit(s)).collect();
    let expected = eval(&mut py, &format!("sc.calc_freq_response('t', np.array([{}])) is None", tiny_rows.join(",")));
    assert_eq!(Value::Bool(rust.calc_freq_response("t", Some(&tiny)).unwrap().is_none()), expected);
    assert!(rust.calc_freq_response("none", None).unwrap().is_none());
    assert!(rust.calc_freq_response("empty", Some(&[])).unwrap().is_none());
    // A single sample spans no time: `int(inf)` raises.
    check_err(
        "single sample",
        rust.calc_freq_response("one", Some(&[[0.0, 1.0, 2.0, 3.0]])).map(|d| d.is_some()),
        py.exec("sc.calc_freq_response('one', np.array([[0.0, 1.0, 2.0, 3.0]]))"),
    );
}

#[test]
fn calibration_data_normalize_and_datasets() {
    let Some(mut py) = start_oracle() else { return };
    let mut data = batch_data(&mut py, "d", accel_batch::SAMPLES);
    let other = batch_data(&mut py, "d2", 6000);
    data.add_data(other);
    py.exec("d.add_data(d2)").unwrap();
    data.normalize_to_frequencies();
    // Idempotent per `_normalized` flag.
    data.normalize_to_frequencies();
    py.exec("d.normalize_to_frequencies(); d.normalize_to_frequencies()").unwrap();
    check_calibration_data(&mut py, "normalized", "d", &data);
    check_calibration_data(&mut py, "normalized child", "d.get_datasets()[1]", data.get_datasets()[1]);
    assert_eq!(data.get_psd("x").unwrap(), &data.psd_x);
    assert_eq!(data.get_psd("all").unwrap(), &data.psd_sum);
    assert!(data.get_psd("bogus").is_none());
    let expected = eval(&mut py, "len(d.get_datasets())");
    assert_eq!(Value::Int(data.get_datasets().len() as i64), expected);
}

// ---------------------------------------------------------------------------
// Estimate remaining vibrations / smoothing / bisect.
// ---------------------------------------------------------------------------

#[test]
fn estimate_vibrations_and_smoothing_match_python() {
    let Some(mut py) = start_oracle() else { return };
    let mut data = batch_data(&mut py, "d", accel_batch::SAMPLES);
    data.normalize_to_frequencies();
    py.exec("d.normalize_to_frequencies()\nbins = d.freq_bins[d.freq_bins <= 90.]\npsd = d.psd_sum[d.freq_bins <= 90.]").unwrap();
    let (bins, psd): (Vec<f64>, Vec<f64>) =
        data.freq_bins.iter().zip(&data.psd_sum).filter(|(f, _)| **f <= 90.0).map(|(f, p)| (*f, *p)).unzip();
    let rust = ShaperCalibrate::new(None);
    for name in ["zv", "mzv", "zvd", "ei", "2hump_ei", "3hump_ei"] {
        for freq in [21.7, 38.5, 64.2] {
            let shaper = init_shaper(name, freq, DEFAULT_DAMPING_RATIO, None).unwrap().unwrap();
            let expected = py_nested_floats(&eval(&mut py, &format!("shaper_defs.init_shaper('{name}', {freq:?}, 0.1)")));
            let flat: Vec<f64> = shaper.0.iter().chain(&shaper.1).copied().collect();
            assert_all_bits(&format!("init_shaper {name} {freq}"), &flat, &expected);
            let s = py_shaper(&shaper);
            for dr in TEST_DAMPING_RATIOS {
                let what = format!("{name} {freq} dr={dr}");
                let est = rust._estimate_shaper(&shaper, dr, &bins).unwrap();
                let expected = eval_floats(&mut py, &format!("sc._estimate_shaper({s}, {dr:?}, bins).tolist()"));
                assert_all_bits(&format!("_estimate_shaper {what}"), &est, &expected);
                let (vib, vals) = rust._estimate_remaining_vibrations(&shaper, dr, &bins, &psd).unwrap();
                py.exec(&format!("v, vals = sc._estimate_remaining_vibrations({s}, {dr:?}, bins, psd)")).unwrap();
                assert_bits(&format!("vibrations {what}"), vib, eval_float(&mut py, "float(v)"));
                assert_all_bits(&format!("vals {what}"), &vals, &eval_floats(&mut py, "vals.tolist()"));
            }
            for (accel, scv) in [(5000.0, 5.0), (3000.0, 7.5), (1e-9, 5.0)] {
                let expr = format!("sc._get_shaper_smoothing({s}, {accel:?}, {scv:?})");
                assert_bits(&expr, rust._get_shaper_smoothing(&shaper, accel, scv), eval_float(&mut py, &expr));
            }
        }
    }
    check_err(
        "empty psd",
        rust._estimate_remaining_vibrations(&init_shaper("zv", 40.0, 0.1, None).unwrap().unwrap(), 0.1, &[], &[]),
        py.exec("sc._estimate_remaining_vibrations(shaper_defs.init_shaper('zv', 40.0, 0.1), 0.1, np.array([]), np.array([]))"),
    );
}

#[test]
fn bisect_and_max_accel_match_python() {
    let Some(mut py) = start_oracle() else { return };
    let rust = ShaperCalibrate::new(None);
    assert_bits("bisect x > 0.5", rust._bisect(|x: f64| x > 0.5), eval_float(&mut py, "float(sc._bisect(lambda x: x > 0.5))"));
    assert_bits("bisect x <= 3.7", rust._bisect(|x: f64| x <= 3.7), eval_float(&mut py, "sc._bisect(lambda x: x <= 3.7)"));
    assert_bits("bisect x <= 0.3", rust._bisect(|x: f64| x <= 0.3), eval_float(&mut py, "sc._bisect(lambda x: x <= 0.3)"));
    for (name, freq, scv) in [("zv", 40.0, 7.0), ("mzv", 38.5, 5.0), ("ei", 32.0, 5.0), ("3hump_ei", 55.0, 2.5)] {
        let shaper = init_shaper(name, freq, DEFAULT_DAMPING_RATIO, None).unwrap().unwrap();
        let expr = format!("sc.find_shaper_max_accel({}, {scv:?})", py_shaper(&shaper));
        assert_bits(&expr, rust.find_shaper_max_accel(&shaper, scv), eval_float(&mut py, &expr));
    }
}

// ---------------------------------------------------------------------------
// fit_shaper / find_best_shaper.
// ---------------------------------------------------------------------------

type FitCase<'a> = (
    &'a str,
    Option<ShaperFreqs>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<f64>,
    Option<&'a [f64]>,
    Option<f64>,
);

fn py_fit_args(case: &FitCase<'_>) -> String {
    let (name, freqs, dr, scv, ms, mv, tdr, mf) = case;
    format!(
        "'{name}', d, {}, {}, {}, {}, {}, {}, {}",
        py_freqs(freqs.as_ref()),
        py_opt(*dr),
        py_opt(*scv),
        py_opt(*ms),
        py_opt(*mv),
        tdr.map_or_else(|| "None".to_string(), lit),
        py_opt(*mf)
    )
}

fn rust_fit(rust: &ShaperCalibrate, data: &CalibrationData, case: &FitCase<'_>) -> KResult<Vec<CalibrationResult>> {
    let (name, freqs, dr, scv, ms, mv, tdr, mf) = case;
    rust.fit_shaper(name, data, freqs.as_ref(), *dr, *scv, *ms, *mv, *tdr, *mf)
}

#[test]
fn fit_shaper_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let mut data = batch_data(&mut py, "d", accel_batch::SAMPLES);
    data.normalize_to_frequencies();
    py.exec("d.normalize_to_frequencies()").unwrap();
    let rust = ShaperCalibrate::new(None);
    let drs: &[f64] = &[0.05, 0.2];
    let cases: Vec<FitCase<'_>> = vec![
        ("mzv", Some(ShaperFreqs::Range(Some(30.0), Some(45.0), Some(0.5))), None, Some(5.0), None, None, None, Some(90.0)),
        ("ei", Some(ShaperFreqs::List(vec![25.0, 33.3, 38.5, 47.1])), Some(0.12), Some(3.5), None, None, Some(drs), Some(90.0)),
        // `max_smoothing` stops the scan early.
        ("zv", Some(ShaperFreqs::Range(None, Some(80.0), Some(1.0))), None, Some(5.0), Some(0.05), None, None, None),
        // `max_vibrations` above the best result selects by vibrations.
        ("2hump_ei", Some(ShaperFreqs::Range(Some(40.0), Some(60.0), Some(2.0))), None, Some(5.0), None, Some(0.0001), None, Some(90.0)),
        // `(start, end)` collapsing to a single frequency, and a step that
        // does not divide the range.
        ("3hump_ei", Some(ShaperFreqs::Range(Some(70.0), Some(60.0), Some(0.2))), None, Some(5.0), None, None, None, Some(90.0)),
        ("zvd", Some(ShaperFreqs::Range(Some(25.0), Some(35.0), Some(0.7))), Some(0.05), Some(5.0), None, None, None, None),
    ];
    for case in &cases {
        let what = format!("fit_shaper({})", py_fit_args(case));
        py.exec(&format!("r = sc.fit_shaper({})", py_fit_args(case))).unwrap_or_else(|e| panic!("{what}: {e:?}"));
        let out = rust_fit(&rust, &data, case).unwrap();
        check_results(&mut py, &what, "r", &out);
    }
    // Error paths.
    let errors: Vec<FitCase<'_>> = vec![
        // Unknown shaper with an explicit start: `init_shaper` gives None.
        ("bogus", Some(ShaperFreqs::Range(Some(20.0), Some(30.0), Some(5.0))), None, Some(5.0), None, None, None, None),
        // Unknown shaper without a start: `get_shaper_cfg(...).min_freq`.
        ("bogus", None, None, Some(5.0), None, None, None, None),
        // An empty range: `test_freqs.max()` of nothing.
        ("zv", Some(ShaperFreqs::Range(Some(30.0), Some(40.0), Some(-1.0))), None, Some(5.0), None, None, None, None),
        // `scv=None` reaches `_get_shaper_smoothing`.
        ("zv", Some(ShaperFreqs::List(vec![30.0, 31.0])), None, None, None, None, None, None),
    ];
    for case in &errors {
        let what = format!("fit_shaper({})", py_fit_args(case));
        check_err(&what, rust_fit(&rust, &data, case), py.exec(&format!("sc.fit_shaper({})", py_fit_args(case))));
    }
}

#[test]
fn find_best_shaper_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let mut data = batch_data(&mut py, "d", accel_batch::SAMPLES);
    data.normalize_to_frequencies();
    py.exec("d.normalize_to_frequencies()").unwrap();
    let rust = ShaperCalibrate::new(None);
    let logs = Mutex::new(Vec::new());
    let logger = |s: &str| logs.lock().push(s.to_string());
    let freqs = ShaperFreqs::Range(Some(30.0), Some(50.0), Some(0.5));
    let (best, all) = rust
        .find_best_shaper(&data, Some(&["zv", "bogus", "ei", "mzv"]), None, Some(5.0), Some(&freqs), Some(0.1), Some(0.02), None, Some(90.0), Some(&logger))
        .unwrap();
    py.exec(
        "lines = []\n\
         best, allsh = sc.find_best_shaper(d, ['zv', 'bogus', 'ei', 'mzv'], None, 5.0, (30.0, 50.0, 0.5), 0.1, 0.02, None, 90.0, lines.append)",
    )
    .unwrap();
    check_results(&mut py, "best", "[best]", std::slice::from_ref(&best));
    check_results(&mut py, "all_shapers", "allsh", &all);
    assert_eq!(*logs.lock(), eval_strings(&mut py, "lines"), "logger lines");
    // No usable shaper: `best_shaper.name` of None.
    check_err(
        "only unknown shapers",
        rust.find_best_shaper(&data, Some(&["bogus"]), None, Some(5.0), Some(&freqs), None, None, None, None, None),
        py.exec("sc.find_best_shaper(d, ['bogus'], None, 5.0, (30.0, 50.0, 0.5))"),
    );
}

/// The KE `SHAPER_CALIBRATE` computation end to end on the recorded batch:
/// `process_accelerometer_data`, `normalize_to_frequencies`,
/// `find_best_shaper` with the KE `max_freq: 90` and `square_corner_velocity`
/// 5, the recommendation line, `save_params` and the calibration CSV.
#[test]
fn recorded_batch_calibration_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let rows = accel_batch::ke_batch();
    let rust = ShaperCalibrate::new(None);
    let mut data = rust.process_accelerometer_data("x", Some(&samples_of(&rows))).unwrap();
    data.normalize_to_frequencies();
    let logs = Mutex::new(Vec::new());
    let logger = |s: &str| logs.lock().push(s.to_string());
    let (best, all) =
        rust.find_best_shaper(&data, None, None, Some(5.0), None, None, None, None, Some(90.0), Some(&logger)).unwrap();
    logs.lock().push(
        pycompat::text::percent_format(
            "Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz",
            &[
                pycompat::text::PctValue::Str("x".to_string()),
                pycompat::text::PctValue::Str(best.name.clone()),
                pycompat::text::PctValue::Str("x".to_string()),
                pycompat::text::PctValue::Float(best.freq),
            ],
        )
        .unwrap(),
    );
    let cfg = FakeConfigFile::new();
    rust.save_params(&cfg, "x", &best.name, best.freq);
    let rs_csv = std::env::temp_dir().join(format!("rs_calibration_{}.csv", std::process::id()));
    let py_csv = std::env::temp_dir().join(format!("py_calibration_{}.csv", std::process::id()));
    rust.save_calibration_data(&rs_csv.to_string_lossy(), &data, Some(&all), Some(90.0)).unwrap();

    py.exec(&format!(
        "d = sc.process_accelerometer_data('x', np.array({rows}))\n\
         d.normalize_to_frequencies()\n\
         lines = []\n\
         best, allsh = sc.find_best_shaper(d, max_smoothing=None, scv=5.0, max_freq=90.0, logger=lines.append)\n\
         lines.append('Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz' % ('x', best.name, 'x', best.freq))\n\
         class Cfg:\n    def __init__(self):\n        self.sets = []\n    def set(self, s, o, v):\n        self.sets.append((s, o, v))\n\
         cfg = Cfg()\n\
         sc.save_params(cfg, 'x', best.name, best.freq)\n\
         sc.save_calibration_data({csv:?}, d, allsh, max_freq=90.0)",
        rows = accel_batch::py_rows(&rows),
        csv = py_csv.to_string_lossy()
    ))
    .unwrap();
    check_calibration_data(&mut py, "batch", "d", &data);
    check_results(&mut py, "best", "[best]", std::slice::from_ref(&best));
    check_results(&mut py, "all_shapers", "allsh", &all);
    assert_eq!(*logs.lock(), eval_strings(&mut py, "lines"), "printed lines");
    let expected_sets = py_nested_strings(&eval(&mut py, "[list(s) for s in cfg.sets]"));
    let sets: Vec<Vec<String>> =
        cfg.sets.lock().iter().map(|(s, o, v)| vec![s.clone(), o.clone(), v.clone()]).collect();
    assert_eq!(sets, expected_sets, "save_params");
    let rs_text = std::fs::read_to_string(&rs_csv).unwrap();
    let py_text = std::fs::read_to_string(&py_csv).unwrap();
    let _ = std::fs::remove_file(&rs_csv);
    let _ = std::fs::remove_file(&py_csv);
    assert_eq!(rs_text, py_text, "calibration CSV");
    // `MEASURE_AXES_NOISE` prints the PSD means of the same data.
    let fresh = rust.process_accelerometer_data("x", Some(&samples_of(&rows))).unwrap();
    py.exec(&format!("dn = sc.process_accelerometer_data(None, np.array({}))", accel_batch::py_rows(&rows))).unwrap();
    for axis in ["x", "y", "z"] {
        let psd = fresh.get_psd(axis).unwrap();
        let expr = format!("float(dn.psd_{axis}.mean())");
        assert_bits(&expr, pycompat::numpy::mean(psd), eval_float(&mut py, &expr));
    }
}

fn py_nested_strings(v: &Value) -> Vec<Vec<String>> {
    as_items(v)
        .iter()
        .map(|row| {
            as_items(row)
                .iter()
                .map(|s| match s {
                    Value::Str(s) => s.clone(),
                    other => panic!("expected str, got {other:?}"),
                })
                .collect()
        })
        .collect()
}
// ---------------------------------------------------------------------------
// SaveParams / ApplyParams / SaveCalibrationData.
// ---------------------------------------------------------------------------

#[test]
fn save_params_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let cfg = FakeConfigFile::new();
    let rust = ShaperCalibrate::new(None);
    rust.save_params(&cfg, "x", "zv", 42.0).expect("save_params");
    let sets = cfg.sets.lock();
    assert_eq!(sets.len(), 2);
    assert_eq!(sets[0], ("input_shaper".to_string(), "shaper_type_x".to_string(), "zv".to_string()));
    assert_eq!(sets[1], ("input_shaper".to_string(), "shaper_freq_x".to_string(), "42.0".to_string()));
    py.exec(r#"
class FakeCfg:
    def __init__(self):
        self.sets = []
    def set(self, section, option, value):
        self.sets.append((section, option, value))
cfg = FakeCfg()
sc.save_params(cfg, 'x', 'zv', 42.0)
py_sets = cfg.sets
    "#).unwrap();
    let expected = py.eval("py_sets").unwrap();
    let items = as_items(&expected);
    assert_eq!(items.len(), 2);
    let t0 = as_items(items[0]);
    let Value::Str(s0) = &t0[0] else { panic!("str expected") };
    assert_eq!(s0, "input_shaper");
}

#[test]
fn apply_params_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(r#"
class FakeGcmd:
    def __init__(self, params):
        self._params = params
    def get_command_params(self):
        return self._params
class FakeGcode:
    def __init__(self):
        self.calls = []
    def create_gcode_command(self, cmd, line, params):
        self.calls.append((cmd, line, dict(params)))
        return FakeGcmd(params)
class FakeInputShaper:
    def __init__(self):
        self.calls = []
    def cmd_SET_INPUT_SHAPER(self, gcmd):
        self.calls.append(dict(gcmd.get_command_params()))
class FakePrinter:
    command_error = Exception
    def __init__(self):
        self.gcode = FakeGcode()
    def lookup_object(self, name):
        assert name == 'gcode'
        return self.gcode
fprinter = FakePrinter()
sc2 = ShaperCalibrate(fprinter)
fake_ish = FakeInputShaper()
sc2.apply_params(fake_ish, 'x', 'zv', 42.0)
sc2.apply_params(fake_ish, 'xy', 'ei', 30.0)
    "#).unwrap();
    let printer = fake_printer();
    printer.add_object("gcode", Arc::new(FakeGcode::new()) as ObjectRef).unwrap();
    let rust = ShaperCalibrate::new(Some(printer));
    let shaper = FakeInputShaper::new();
    rust.apply_params(&shaper, "x", "zv", 42.0).unwrap();
    rust.apply_params(&shaper, "xy", "ei", 30.0).unwrap();
    // x (1) + xy expanded to x, y (2) = 3 calls, same order on both sides.
    let rust_calls = shaper.calls.lock();
    assert_eq!(rust_calls.len(), 3);
    let keys = [
        ("SHAPER_TYPE_X", "SHAPER_FREQ_X"),
        ("SHAPER_TYPE_X", "SHAPER_FREQ_X"),
        ("SHAPER_TYPE_Y", "SHAPER_FREQ_Y"),
    ];
    for (i, (tk, fk)) in keys.iter().enumerate() {
        let t = py.eval(&format!("fake_ish.calls[{i}]['{tk}']")).unwrap();
        let f = py.eval(&format!("fake_ish.calls[{i}]['{fk}']")).unwrap();
        let Value::Str(pt) = t else { panic!("str expected") };
        let Value::Float(pf) = f else { panic!("float expected") };
        assert_eq!(&rust_calls[i][*tk], &pt);
        assert_eq!(
            rust_calls[i][*fk].parse::<f64>().unwrap().to_bits(),
            pf.to_f64().unwrap().to_bits()
        );
    }
    // `ShaperCalibrate(None).apply_params` -> AttributeError on lookup_object.
    let err = ShaperCalibrate::new(None)
        .apply_params(&shaper, "x", "zv", 42.0)
        .unwrap_err();
    assert_eq!(err.class_name(), "AttributeError");
}

#[test]
fn escape_csv_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let rust = ShaperCalibrate::new(None);
    for name in ["plain", "a,b", "a\"b", "x,y\"z"] {
        let esc = rust._escape_for_csv(name);
        let lit = name.replace('\\', "\\\\").replace('"', "\\\"");
        let expected = py.eval(&format!("sc._escape_for_csv(\"{lit}\")")).unwrap();
        let Value::Str(s) = expected else { panic!("str expected") };
        assert_eq!(esc, s);
    }
}

/// Writes the calibration CSV from both sides and compares the text.
#[allow(clippy::too_many_arguments)]
fn compare_csv(
    py: &mut pyoracle::Oracle,
    rust: &ShaperCalibrate,
    what: &str,
    data: &CalibrationData,
    py_data: &str,
    shapers: Option<&[CalibrationResult]>,
    py_shapers: &str,
    max_freq: Option<f64>,
) {
    let pid = std::process::id();
    let rs_path = std::env::temp_dir().join(format!("rs_cal_{pid}.csv"));
    let py_path = std::env::temp_dir().join(format!("py_cal_{pid}.csv"));
    rust.save_calibration_data(&rs_path.to_string_lossy(), data, shapers, max_freq).unwrap();
    py.exec(&format!(
        "sc.save_calibration_data({:?}, {py_data}, {py_shapers}, {})",
        py_path.to_string_lossy(),
        py_opt(max_freq)
    ))
    .unwrap();
    let rs_text = std::fs::read_to_string(&rs_path).unwrap();
    let py_text = std::fs::read_to_string(&py_path).unwrap();
    let _ = std::fs::remove_file(&rs_path);
    let _ = std::fs::remove_file(&py_path);
    assert_eq!(rs_text, py_text, "{what}");
}

#[test]
fn save_calibration_data_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let mut data = batch_data(&mut py, "d", accel_batch::SAMPLES);
    let mut other = batch_data(&mut py, "d2", 6000);
    data.normalize_to_frequencies();
    other.normalize_to_frequencies();
    py.exec("d.normalize_to_frequencies(); d2.normalize_to_frequencies()").unwrap();
    let rust = ShaperCalibrate::new(None);
    let freqs = ShaperFreqs::List(vec![35.0, 38.5, 42.0]);
    let shapers = rust.fit_shaper("mzv", &data, Some(&freqs), None, Some(5.0), None, None, None, Some(90.0)).unwrap();
    py.exec("shapers = sc.fit_shaper('mzv', d, [35.0, 38.5, 42.0], None, 5.0, None, None, None, 90.0)").unwrap();
    let py = &mut py;
    compare_csv(py, &rust, "single dataset", &data, "d", None, "None", None);
    compare_csv(py, &rust, "single dataset with shapers", &data, "d", Some(&shapers), "shapers", Some(90.0));
    data.add_data(other);
    py.exec("d.add_data(d2)").unwrap();
    compare_csv(py, &rust, "two datasets", &data, "d", None, "None", Some(90.0));
    compare_csv(py, &rust, "two datasets with shapers", &data, "d", Some(&shapers), "shapers", Some(90.0));
    // An unwritable path raises `self.error` (`Exception` without a printer).
    let actual = rust.save_calibration_data("/nonexistent-dir-xyz/test.csv", &data, None, None).unwrap_err();
    let expected = py
        .exec("sc.save_calibration_data('/nonexistent-dir-xyz/test.csv', d, None, None)")
        .expect_err("Python did not raise");
    assert_eq!(actual.class_name().to_string(), expected.class_name);
}

#[test]
fn background_process_exec_wraps_errors() {
    let rust = ShaperCalibrate::new(None);
    // No printer: direct passthrough of errors.
    let err = rust.background_process_exec(|| -> KResult<()> { Err(KlippyError::command("boom".to_string())) }).unwrap_err();
    assert_eq!(err.message(), "boom");
}

#[test]
fn process_accelerometer_data_error() {
    let rust = ShaperCalibrate::new(None);
    let err = rust.process_accelerometer_data("x", None).unwrap_err();
    assert!(err.message().contains("Internal error processing accelerometer data"));
}
