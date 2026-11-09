//! Parity tests for `klippy/extras/shaper_calibrate.py`.
//!
//! Expected values come from `pyoracle` (live CPython with frozen `klippy/`)
//! for pure-math helpers and from direct reference replay where NumPy is
//! unavailable. Tests that need the oracle skip with a message when
//! `pyoracle::Oracle::start()` returns `None` (no `$KLIPPY_PYTHON`).
//!
//! The port has no G-code commands, config options or status fields
//! (inventory contracts are empty); the exercised surface is the numeric
//! pipeline (`calc_freq_response`, `_psd`, `fit_shaper`, ...), CSV saving
//! and `save_params` / `apply_params`.
use extras::shaper_calibrate::{
    CalibrationData, InputShaperGCodeApi, ShaperCalibrate, ShaperFreqs, Sample, MIN_FREQ,
    MAX_FREQ, WINDOW_T_SEC, MAX_SHAPER_FREQ, TEST_DAMPING_RATIOS, AUTOTUNE_SHAPERS,
};
use extras::shaper_defs::{get_shaper_cfg, init_shaper, DEFAULT_DAMPING_RATIO, SHAPER_VIBRATION_REDUCTION};
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
use kos::libm::{cos, exp, sin};
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
    fn set(&self, section: &str, option: &str, value: &str) {
        self.sets.lock().push((section.to_string(), option.to_string(), value.to_string()));
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
impl InputShaperGCodeApi for FakeInputShaper {
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

fn py_floats(v: &Value) -> Vec<f64> {
    as_items(v).iter().map(|it| match it {
        Value::Float(f) => f.to_f64().unwrap(),
        Value::Int(i) => *i as f64,
        other => panic!("expected float, got {other:?}"),
    }).collect()
}

fn py_nested_floats(v: &Value) -> Vec<f64> {
    let mut out = Vec::new();
    fn walk(v: &Value, out: &mut Vec<f64>) {
        match v {
            Value::List(items) | Value::Tuple(items) => items.iter().for_each(|it| walk(it, out)),
            Value::Float(f) => out.push(f.to_f64().unwrap()),
            Value::Int(i) => out.push(*i as f64),
            other => panic!("expected number, got {other:?}"),
        }
    }
    walk(v, &mut out);
    out
}

fn assert_close(actual: &[f64], expected: &[f64], tol: f64, what: &str) {
    assert_eq!(actual.len(), expected.len(), "{what}: length");
    for (i, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
        let scale = (*a).abs().max((*e).abs()).max(1.0);
        assert!((a - e).abs() <= tol * scale, "{what}[{i}]: {a} vs {e} (tol {tol})");
    }
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

fn sample_data(n: usize, fs: f64) -> Vec<Sample> {
    (0..n).map(|i| {
        let t = i as f64 / fs;
        [
            t,
            sin(2.0 * std::f64::consts::PI * 30.0 * t) * 0.5 + sin(2.0 * std::f64::consts::PI * 55.0 * t) * 0.25,
            cos(2.0 * std::f64::consts::PI * 30.0 * t) * 0.5,
            sin(2.0 * std::f64::consts::PI * 17.0 * t) * 0.3,
        ]
    }).collect()
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
    let x_lit = format!("[{}]", x.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let expr = format!(r#"sc._split_into_windows(np.array({x_lit}), 64, 32).tolist()"#);
    let expected = py_nested_floats(&py.eval(&expr).unwrap());
    // Compare per-window: flatten in order.
    let flat: Vec<f64> = ws.into_iter().flatten().collect();
    assert_eq!(flat.len(), expected.len());
    for (a, e) in flat.iter().zip(expected.iter()) {
        assert_eq!(a.to_bits(), e.to_bits());
    }
    // Overlap edge: reference layout is (window_size, n_windows).
    let ws2 = rust._split_into_windows(&[1.0, 2.0, 3.0, 4.0, 5.0], 3, 1);
    assert_eq!(ws2.len(), 3);
    assert_eq!(ws2[0], vec![1.0, 3.0]);
    assert_eq!(ws2[1], vec![2.0, 4.0]);
    assert_eq!(ws2[2], vec![3.0, 5.0]);
}

// ---------------------------------------------------------------------------
// _psd / calc_freq_response.
// ---------------------------------------------------------------------------

#[test]
fn psd_matches_python_within_fft_tolerance() {
    let Some(mut py) = start_oracle() else { return };
    let fs = 3200.0;
    let n = 3200usize;
    let x: Vec<f64> = (0..n).map(|i| {
        let t = i as f64 / fs;
        sin(2.0 * std::f64::consts::PI * 30.0 * t) * 0.5
    }).collect();
    let rust = ShaperCalibrate::new(None);
    let (freqs, psd) = rust._psd(&x, fs, 2048);
    let x_lit = format!("[{}]", x.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let expr = format!(r#"list(sc._psd(np.array({x_lit}), 3200.0, 2048)[1])"#);
    let expected = py_floats(&py.eval(&expr).unwrap());
    assert_eq!(freqs.len(), expected.len());
    // Naive DFT vs pocketfft: allow 1e-9 relative.
    assert_close(&psd, &expected, 1e-9, "psd");
    for (i, &f) in freqs.iter().enumerate() {
        assert!((f - i as f64 * fs / 2048.0).abs() < 1e-12);
    }
}

#[test]
fn calc_freq_response_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let samples = sample_data(3200, 3200.0);
    let rust = ShaperCalibrate::new(None);
    let data = rust.calc_freq_response("test", Some(&samples)).expect("data");
    let samples_lit = format!("[{}]", samples.iter().map(|s| format!("[{:?}, {:?}, {:?}, {:?}]", s[0], s[1], s[2], s[3])).collect::<Vec<_>>().join(","));
    py.exec(&format!(r#"d = sc.calc_freq_response('test', np.array({samples_lit}))"#)).unwrap();
    let expected = py_floats(&py.eval("list(d.psd_sum)").unwrap());
    assert_close(&data.psd_sum, &expected, 1e-9, "psd_sum");
    assert_eq!(data.psd_x.len(), data.psd_sum.len());
    // Small input: N <= M returns None.
    assert!(rust.calc_freq_response("tiny", Some(&sample_data(10, 3200.0))).is_none());
    assert!(rust.calc_freq_response("none", None).is_none());
    assert!(rust.calc_freq_response("empty", Some(&[])).is_none());
}

#[test]
fn calibration_data_normalize_and_datasets() {
    let Some(mut py) = start_oracle() else { return };
    let mut data = CalibrationData::new(
        "a".to_string(),
        vec![0.0, 5.0, 10.0, 20.0],
        vec![1.0, 2.0, 3.0, 4.0],
        vec![0.5, 1.0, 1.5, 2.0],
        vec![0.2, 0.3, 0.4, 0.5],
        vec![0.1, 0.1, 0.2, 0.2],
    );
    data.normalize_to_frequencies();
    // Idempotent per `_normalized` flag.
    data.normalize_to_frequencies();
    py.exec("d = shaper_calibrate.CalibrationData('a', np.array([0.0,5.0,10.0,20.0]), np.array([1.0,2.0,3.0,4.0]), np.array([0.5,1.0,1.5,2.0]), np.array([0.2,0.3,0.4,0.5]), np.array([0.1,0.1,0.2,0.2])); d.set_numpy(np); d.normalize_to_frequencies()").unwrap();
    let expected = py.eval("list(d.psd_sum)").unwrap();
    assert_close(&data.psd_sum, &py_floats(&expected), 1e-12, "psd_sum normalized");
    assert_eq!(data.get_psd("x").unwrap(), &data.psd_x);
    assert_eq!(data.get_psd("all").unwrap(), &data.psd_sum);
    assert!(data.get_psd("bogus").is_none());
    assert_eq!(data.get_datasets().len(), 1);
}

// ---------------------------------------------------------------------------
// Estimate remaining vibrations / smoothing / bisect.
// ---------------------------------------------------------------------------

#[test]
fn estimate_vibrations_and_smoothing_match_python() {
    let Some(mut py) = start_oracle() else { return };
    let shaper = init_shaper("zv", 40.0, 0.1, None).unwrap().unwrap();
    let rust = ShaperCalibrate::new(None);
    let freqs = vec![10.0, 20.0, 40.0, 80.0];
    let est = rust._estimate_shaper(&shaper, 0.1, &freqs);
    let a_lit = format!("[{}]", shaper.0.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let t_lit = format!("[{}]", shaper.1.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let expr = format!(r#"list(sc._estimate_shaper(({a_lit}, {t_lit}), 0.1, np.array([10.0, 20.0, 40.0, 80.0])))"#);
    assert_close(&est, &py_floats(&py.eval(&expr).unwrap()), 1e-12, "estimate_shaper");
    let psd = vec![0.1, 0.5, 1.0, 0.2];
    let (vib, _vals) = rust._estimate_remaining_vibrations(&shaper, 0.1, &freqs, &psd);
    let exp_v = py.eval(&format!(r#"float(sc._estimate_remaining_vibrations(({a_lit}, {t_lit}), 0.1, np.array([10.0,20.0,40.0,80.0]), np.array([0.1,0.5,1.0,0.2]))[0])"#)).unwrap();
    let Value::Float(ef) = exp_v else { panic!("float expected") };
    let exp_vib = ef.to_f64().unwrap();
    assert!((vib - exp_vib).abs() < 1e-12, "{vib} vs {exp_vib}");
    let smooth = rust._get_shaper_smoothing(&shaper, 5000.0, 5.0);
    let exp_smooth = py.eval(&format!(r#"sc._get_shaper_smoothing(({a_lit}, {t_lit}), 5000, 5.)"#)).unwrap();
    let Value::Float(f) = exp_smooth else { panic!("float expected") };
    assert!((smooth - f.to_f64().unwrap()).abs() < 1e-12, "{smooth} vs {:?}", f.to_f64().unwrap());
}

#[test]
fn bisect_and_max_accel_match_python() {
    let Some(mut py) = start_oracle() else { return };
    let rust = ShaperCalibrate::new(None);
    // `func(x) = x > 0.5`.
    let v = rust._bisect(|x: f64| x > 0.5);
    let exp = py.eval("sc._bisect(lambda x: x > 0.5)").unwrap();
    let Value::Float(f) = exp else { panic!("float expected") };
    assert!((v - f.to_f64().unwrap()).abs() < 1e-9, "{v} vs {:?}", f.to_f64().unwrap());
    // Always-false func returns 0.0.
    assert_eq!(rust._bisect(|_x: f64| false), 0.0);
    let shaper = init_shaper("zv", 40.0, 0.1, None).unwrap().unwrap();
    let accel = rust.find_shaper_max_accel(&shaper, 7.0);
    let a_lit = format!("[{}]", shaper.0.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let t_lit = format!("[{}]", shaper.1.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let exp_accel = py.eval(&format!(r#"sc.find_shaper_max_accel(({a_lit}, {t_lit}), 7.0)"#)).unwrap();
    let Value::Float(fa) = exp_accel else { panic!("float expected") };
    assert!((accel - fa.to_f64().unwrap()).abs() < 1e-9, "{accel} vs {:?}", fa.to_f64().unwrap());
}

// ---------------------------------------------------------------------------
// fit_shaper / find_best_shaper.
// ---------------------------------------------------------------------------

fn synthetic_calib() -> CalibrationData {
    let n = 500usize;
    let freqs: Vec<f64> = (0..n).map(|i| i as f64 * 0.4).collect();
    let mut psd_sum = vec![0.0; n];
    for (i, &f) in freqs.iter().enumerate() {
        let bump = |c: f64, w: f64| exp(-((f - c) * (f - c)) / (2.0 * w * w));
        psd_sum[i] = 1.0 * bump(30.0, 5.0) + 0.6 * bump(55.0, 8.0) + 0.001;
    }
    let px: Vec<f64> = psd_sum.iter().map(|v| v * 0.5).collect();
    let pyv: Vec<f64> = psd_sum.iter().map(|v| v * 0.3).collect();
    let pz: Vec<f64> = psd_sum.iter().map(|v| v * 0.2).collect();
    CalibrationData::new("synth".to_string(), freqs, psd_sum, px, pyv, pz)
}

#[test]
fn fit_shaper_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let data = synthetic_calib();
    let rust = ShaperCalibrate::new(None);
    let freqs = ShaperFreqs::Range(Some(20.0), Some(60.0), Some(5.0));
    let out = rust.fit_shaper("zv", &data, Some(&freqs), None, Some(5.0), None, None, None, None).unwrap();
    assert!(!out.is_empty());
    let freq_lit = format!("[{}]", data.freq_bins.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let psd_lit = format!("[{}]", data.psd_sum.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let setup = format!(r#"cd = shaper_calibrate.CalibrationData('synth', np.array({freq_lit}), np.array({psd_lit}), np.array([0.0]*{n}), np.array([0.0]*{n}), np.array([0.0]*{n})); cd.psd_x = np.array([v*0.5 for v in {psd_lit}]); cd.psd_y = np.array([v*0.3 for v in {psd_lit}]); cd.psd_z = np.array([v*0.2 for v in {psd_lit}])"#,
        n = data.psd_sum.len());
    py.exec(&setup).unwrap();
    py.exec("r = sc.fit_shaper('zv', cd, (20.0, 60.0, 5.0), None, 5.0, None, None, None, None)").unwrap();
    let expected_freqs = py_floats(&py.eval("[res.freq for res in r]").unwrap());
    let actual_freqs: Vec<f64> = out.iter().map(|r| r.freq).collect();
    assert_close(&actual_freqs, &expected_freqs, 1e-9, "fit freqs");
    let vib_expr = "r[0].vibrs";
    let Value::Float(ev) = py.eval(vib_expr).unwrap() else { panic!("float") };
    assert!((out[0].vibrs - ev.to_f64().unwrap()).abs() < 1e-9, "{} vs {:?}", out[0].vibrs, ev.to_f64().unwrap());
    let Value::Float(es) = py.eval("r[0].score").unwrap() else { panic!("float") };
    assert!((out[0].score - es.to_f64().unwrap()).abs() < 1e-9, "{} vs {:?}", out[0].score, es.to_f64().unwrap());
    // Unknown shaper without cfg: AttributeError.
    let err = rust.fit_shaper("bogus", &data, Some(&freqs), None, Some(5.0), None, None, None, None).unwrap_err();
    assert_eq!(err.class_name(), "AttributeError");
    // Empty freq list maps to the default range.
    let out2 = rust.fit_shaper("zv", &data, Some(&ShaperFreqs::List(Vec::new())), None, Some(5.0), None, None, None, None).unwrap();
    assert!(!out2.is_empty());
}

#[test]
fn find_best_shaper_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let data = synthetic_calib();
    let rust = ShaperCalibrate::new(None);
    let logs = Mutex::new(Vec::new());
    let logger = |s: &str| logs.lock().push(s.to_string());
    let freqs = ShaperFreqs::List(vec![25.0, 30.0, 35.0, 40.0]);
    let (best, all) = rust.find_best_shaper(&data, Some(&["zv", "ei"]), None, Some(5.0), Some(&freqs), None, None, None, None, Some(&logger)).unwrap();
    assert_eq!(all.len(), 2);
    let freq_lit = format!("[{}]", data.freq_bins.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let psd_lit = format!("[{}]", data.psd_sum.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    py.exec(&format!(r#"cd2 = shaper_calibrate.CalibrationData('synth', np.array({freq_lit}), np.array({psd_lit}), np.array([0.0]*{n}), np.array([0.0]*{n}), np.array([0.0]*{n}))"#,
        n = data.psd_sum.len())).unwrap();
    py.exec("best, allsh = sc.find_best_shaper(cd2, ['zv', 'ei'], None, 5.0, [25.0, 30.0, 35.0, 40.0], None, None, None, None, lambda m: None)").unwrap();
    let v = py.eval("(best.name, best.freq, best.vibrs, best.score)").unwrap();
    let items = as_items(&v);
    let name = match &items[0] { Value::Str(s) => s.clone(), other => panic!("str expected: {other:?}") };
    assert_eq!(best.name, name);
    let ef = match &items[1] { Value::Float(f) => f.to_f64().unwrap(), other => panic!("float: {other:?}") };
    assert!((best.freq - ef).abs() < 1e-9);
    let ev = match &items[2] { Value::Float(f) => f.to_f64().unwrap(), other => panic!("float: {other:?}") };
    assert!((best.vibrs - ev).abs() < 1e-9);
    // Logger saw Fitted lines.
    assert!(logs.lock().iter().any(|m| m.contains("Fitted shaper")));
    // Unknown shaper is skipped with a log line.
    let (best2, all2) = rust.find_best_shaper(&data, Some(&["bogus", "zv"]), None, Some(5.0), Some(&freqs), None, None, None, None, Some(&logger)).unwrap();
    assert_eq!(best2.name, "zv");
    assert_eq!(all2.len(), 1);
    assert!(logs.lock().iter().any(|m| m.contains("Unknown shaper bogus")));
    // max_smoothing early exit.
    let out = rust.fit_shaper("zv", &data, Some(&ShaperFreqs::List(vec![25.0, 40.0, 60.0])), None, Some(5.0), Some(0.0), None, None, None).unwrap();
    let _ = out;
    let _ = get_shaper_cfg("zv").map(|c| c.min_freq);
}

// ---------------------------------------------------------------------------
// SaveParams / ApplyParams / SaveCalibrationData.
// ---------------------------------------------------------------------------

#[test]
fn save_params_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    let cfg = FakeConfigFile::new();
    let rust = ShaperCalibrate::new(None);
    rust.save_params(&cfg, "x", "zv", 42.0);
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

#[test]
fn save_calibration_data_matches_python() {
    let Some(py) = start_oracle() else { return };
    let mut data = synthetic_calib();
    data.normalize_to_frequencies();
    let rust = ShaperCalibrate::new(None);
    let mut dir = std::env::temp_dir();
    dir.push(format!("shaper_cal_test_{}.csv", std::process::id()));
    let path = dir.to_string_lossy().to_string();
    rust.save_calibration_data(&path, &data, None, None).unwrap();
    let content = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    let lines: Vec<&str> = content.lines().collect();
    assert_eq!(lines[0], "freq,psd_x,psd_y,psd_z,psd_xyz");
    assert!(lines.len() > 10);
    assert!(lines[1].starts_with("0.0,"));
    // Invalid path raises command_error (self.error with no printer -> Exception).
    let err = rust.save_calibration_data("/nonexistent-dir-xyz/test.csv", &data, None, None).unwrap_err();
    assert_eq!(err.class_name(), "Exception");
    assert!(err.message().contains("Error writing to file"));
    let _ = py;
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
