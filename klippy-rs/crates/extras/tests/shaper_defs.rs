//! Parity tests for `klippy/extras/shaper_defs.py`.
//!
//! Expected values come live from the frozen reference through
//! `pyoracle` (set `KLIPPY_PYTHON` to the CPython 3.8.2 binary; the
//! test skips when it is unset). Each case builds the same call on
//! both sides and compares returned `(A, T)` float lists bit for
//! bit (hex + repr), exception class/message, and
//! `get_shaper_cfg` / `init_shaper` name-resolution behaviour.

use extras::shaper_defs;
use klippy_api::error::{KResult, KlippyError};
use recording::{Exception, Value};

fn skip_no_oracle(py: Option<pyoracle::Oracle>) -> Option<pyoracle::Oracle> {
    if let Some(p) = py {
        Some(p)
    } else {
        println!("skipping shaper_defs oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        None
    }
}

fn pair_value(a: &[f64], t: &[f64]) -> Value {
    let la: Vec<Value> = a.iter().map(|x| Value::float(*x)).collect();
    let lt: Vec<Value> = t.iter().map(|x| Value::float(*x)).collect();
    Value::Tuple(vec![Value::List(la), Value::List(lt)])
}
fn assert_bit_equal(label: &str, want: &Value, got: &Value) {
    // Compare floats by bits: the oracle encodes hex floats from CPython's
    // zero-padded `float.hex()`, while `Value::float` trims trailing zeros.
    // `recording::assert_equal` already compares floats bit for bit.
    let rec = recording::Record {
        v: 1,
        unit: "shaper_defs".to_string(),
        scenario: "live".to_string(),
        seq: 1,
        function: label.to_string(),
        args: vec![],
        ret: None,
        exc: None,
        side_effects: recording::SideEffects::default(),
        object_id: None,
        kind: recording::RecordKind::Python,
    };
    if let Err(d) = recording::assert_equal(&rec, "ret", want, got) {
        panic!("{label}: {d}");
    }
}

fn check_pair(label: &str, expected: pyoracle::OracleResult, actual: KResult<(Vec<f64>, Vec<f64>)>) {
    match (expected, actual) {
        (Ok(want), Ok((a, t))) => {
            let got = pair_value(&a, &t);
            assert_bit_equal(label, &want, &got);
        }
        (Ok(want), Err(e)) => {
            panic!("{label}: oracle returned {want:?}, port raised {}: {}", e.class_name(), e.message())
        }
        (Err(exc), Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception type");
            assert_eq!(e.message(), exc.message, "{label} exception message");
        }
        (Err(exc), Ok((a, t))) => {
            panic!("{label}: oracle raised {}: {}, port returned A={a:?} T={t:?}", exc.class_name, exc.message)
        }
    }
}

fn check_opt_pair(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<Option<(Vec<f64>, Vec<f64>)>>,
) {
    match (expected, actual) {
        (Ok(Value::Null), Ok(None)) => {}
        (Ok(want), Ok(Some((a, t)))) => {
            let got = pair_value(&a, &t);
            assert_bit_equal(label, &want, &got);
        }
        (Ok(want), Ok(None)) => panic!("{label}: oracle returned {want:?}, port returned None"),
        (Ok(want), Err(e)) => panic!("{label}: oracle returned {want:?}, port raised {}: {}", e.class_name(), e.message()),
        (Err(exc), Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception type");
            assert_eq!(e.message(), exc.message, "{label} exception message");
        }
        (Err(exc), Ok(Some((a, t)))) => panic!("{label}: oracle raised {}: {}, port returned A={a:?} T={t:?}", exc.class_name, exc.message),
        (Err(exc), Ok(None)) => panic!("{label}: oracle raised {}: {}, port returned None", exc.class_name, exc.message),
    }
}

fn check_value(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    match expected {
        Ok(want) => assert_bit_equal(label, &want, &actual),
        Err(exc) => panic!("{label}: oracle raised {}: {}", exc.class_name, exc.message),
    }
}

#[test]
fn constants_and_cfg_table() {
    let Some(mut py) = skip_no_oracle(pyoracle::Oracle::start()) else { return };
    py.exec("import extras.shaper_defs").expect("import shaper_defs");
    // Constants.
    let e = py.eval("extras.shaper_defs.SHAPER_VIBRATION_REDUCTION").expect("eval");
    check_value("SHAPER_VIBRATION_REDUCTION", Ok(e), Value::float(shaper_defs::SHAPER_VIBRATION_REDUCTION));
    let e = py.eval("extras.shaper_defs.DEFAULT_DAMPING_RATIO").expect("eval");
    check_value("DEFAULT_DAMPING_RATIO", Ok(e), Value::float(shaper_defs::DEFAULT_DAMPING_RATIO));
    // Config table: compare name/min_freq/max_damping_ratio per entry.
    for (i, cfg) in shaper_defs::INPUT_SHAPERS.iter().enumerate() {
        let e = py.eval(&format!("extras.shaper_defs.INPUT_SHAPERS[{i}].name")).expect("eval");
        check_value("INPUT_SHAPERS name", Ok(e), Value::str(cfg.name));
        let e = py.eval(&format!("extras.shaper_defs.INPUT_SHAPERS[{i}].min_freq")).expect("eval");
        check_value("INPUT_SHAPERS min_freq", Ok(e), Value::float(cfg.min_freq));
        let e = py.eval(&format!("extras.shaper_defs.INPUT_SHAPERS[{i}].max_damping_ratio")).expect("eval");
        check_value("INPUT_SHAPERS max_damping", Ok(e), Value::float(cfg.max_damping_ratio));
        // init_func callable identity via constructor-derived behaviour is
        // covered by init_shaper; here just check the cfg lookup.
    }
    // get_shaper_cfg name resolution (function object itself is opaque).
    for name in ["zv", "mzv", "zvd", "ei", "2hump_ei", "3hump_ei", "bogus", "Mzv", "zvx", ""] {
        let stripped_py = py
            .eval(&format!("__import__('extras.shaper_defs', fromlist=['get_shaper_cfg']).get_shaper_cfg({name:?}) is None"))
            .expect("eval");
        let stripped_rs = shaper_defs::get_shaper_cfg(name).is_none();
        check_value(&format!("get_shaper_cfg({name:?}) is None"), Ok(stripped_py), Value::Bool(stripped_rs));
        if let Some(cfg) = shaper_defs::get_shaper_cfg(name) {
            let e = py.eval(&format!("__import__('extras.shaper_defs', fromlist=['get_shaper_cfg']).get_shaper_cfg({name:?}).name")).expect("eval");
            check_value(&format!("get_shaper_cfg({name:?}).name"), Ok(e), Value::str(cfg.name));
        }
    }
    // Parenthesised names strip to the head token.
    for name in ["mzv(5,0.6)", "ei(v_tol=0.05)", "zv(1,2)", "mzv (5)", "2hump_ei( )"] {
        let e = py
            .eval(&format!("__import__('extras.shaper_defs', fromlist=['get_shaper_cfg']).get_shaper_cfg({name:?}) is None"))
            .expect("eval");
        let rs = shaper_defs::get_shaper_cfg(name).is_none();
        check_value(&format!("get_shaper_cfg({name:?}) is None"), Ok(e), Value::Bool(rs));
    }
    // Unbalanced / non-matching strings do not strip.
    for name in ["mzv(", "mzv)", "(5)", "mzv(5", "mzv(5))extra"] {
        let e = py
            .eval(&format!("__import__('extras.shaper_defs', fromlist=['get_shaper_cfg']).get_shaper_cfg({name:?}) is None"))
            .expect("eval");
        let rs = shaper_defs::get_shaper_cfg(name).is_none();
        check_value(&format!("get_shaper_cfg({name:?}) is None"), Ok(e), Value::Bool(rs));
    }
}

#[test]
fn direct_shapers_dense_grid() {
    let Some(mut py) = skip_no_oracle(pyoracle::Oracle::start()) else { return };
    py.exec("import extras.shaper_defs").expect("import shaper_defs");
    let freqs = [5.0, 20.0, 21.0, 23.0, 25.0, 30.0, 38.5, 50.0, 100.0, 0.0, 1.0, 200.0];
    let dampings = [0.0, 0.1, 0.05, 0.2, 0.4, 0.99, 1.0, 1.5, -0.1];
    for &freq in &freqs {
        for &damp in &dampings {
            let e = py.eval("extras.shaper_defs.get_none_shaper()");
            let (a, t) = shaper_defs::get_none_shaper();
            check_pair("get_none_shaper()", e, Ok((a, t)));
            let e = py.eval(&format!("extras.shaper_defs.get_zv_shaper({freq:?}, {damp:?})"));
            check_pair(&format!("zv({freq},{damp})"), e, shaper_defs::get_zv_shaper(freq, damp));
            let e = py.eval(&format!("extras.shaper_defs.get_zvd_shaper({freq:?}, {damp:?})"));
            check_pair(&format!("zvd({freq},{damp})"), e, shaper_defs::get_zvd_shaper(freq, damp));
            let e = py.eval(&format!("extras.shaper_defs.get_ei_shaper({freq:?}, {damp:?})"));
            check_pair(&format!("ei({freq},{damp})"), e, shaper_defs::get_ei_shaper(freq, damp));
            let e = py.eval(&format!("extras.shaper_defs.get_2hump_ei_shaper({freq:?}, {damp:?})"));
            check_pair(&format!("2hump({freq},{damp})"), e, shaper_defs::get_2hump_ei_shaper(freq, damp));
            let e = py.eval(&format!("extras.shaper_defs.get_3hump_ei_shaper({freq:?}, {damp:?})"));
            check_pair(&format!("3hump({freq},{damp})"), e, shaper_defs::get_3hump_ei_shaper(freq, damp));
            let e = py.eval(&format!("extras.shaper_defs.get_mzv_shaper({freq:?}, {damp:?})"));
            check_pair(&format!("mzv({freq},{damp})"), e, shaper_defs::get_mzv_shaper(freq, damp));
        }
    }
    // MZV explicit parameters: n, t, tau grid including error branches.
    let ns = [2i64, 3, 4, 5, 6, 8, 0, 1, 10];
    let ts = [0.0f64, 0.75, 0.5, 0.6, 1.0, 2.0, 0.2, -1.0];
    let taus = [0.0f64, 0.5, 1.0, 2.0, -0.5];
    for &n in &ns {
        for &t in &ts {
            for &freq in &[30.0f64, 38.5] {
                let e = py.eval(&format!("extras.shaper_defs.get_mzv_coeffs({n:?}, {t:?})"));
                check_pair(&format!("mzv_coeffs({n},{t})"), e, shaper_defs::get_mzv_coeffs(n, t));
                let e = py.eval(&format!("extras.shaper_defs.get_mzv_shaper({freq:?}, 0.1, {n:?}, {t:?})"));
                check_pair(&format!("mzv_f({freq},{n},{t})"), e, shaper_defs::get_mzv_shaper_full(freq, 0.1, n, t, 0.0));
                let _ = freq;
            }
        }
    }
    for &n in &[3i64, 4, 5] {
        for &tau in &taus {
            let e = py.eval(&format!("extras.shaper_defs.get_mzv_shaper(38.5, 0.1, {n:?}, 0.0, {tau:?})"));
            check_pair(&format!("mzv_tau({n},{tau})"), e, shaper_defs::get_mzv_shaper_full(38.5, 0.1, n, 0.0, tau));
        }
    }
    // EI v_tol keyword via init_shaper is covered below; here direct
    // expansion helper is exercised through 2/3hump paths.
}

#[test]
fn init_shaper_dispatch_and_errors() {
    let Some(mut py) = skip_no_oracle(pyoracle::Oracle::start()) else { return };
    py.exec("import extras.shaper_defs").expect("import shaper_defs");
    let factory = |m: String| KlippyError::Config(m);
    let fact: &dyn Fn(String) -> KlippyError = &factory;
    let names = [
        "zv",
        "mzv",
        "zvd",
        "ei",
        "2hump_ei",
        "3hump_ei",
        "bogus",
        "mzv(5,0.6)",
        "mzv(4)",
        "mzv(3,0.5)",
        "mzv(n=5, t=0.6)",
        "mzv(n=5)",
        "mzv(tau=0.8)",
        "mzv(n=4, tau=0.9)",
        "mzv(5,0.6,0.9)",
        "mzv(n=5, tau=0.5)",
        "ei(v_tol=0.05)",
        "ei(0.07)",
        "ei()",
        "zv(1)",
        "zv(1,2)",
        "zvd(1)",
        "2hump_ei(1)",
        "ei(v_tol=0.05, v_tol=0.07)",
        "mzv(5, n=4)",
        "zv(v_tol=0.05)",
        "mzv(x=1)",
        "mzv(2,0.6)",
        "mzv(-1)",
        "mzv(5.,0.6)",
        "mzv(abc)",
        "zog(1)",
        "mzv(5, t=0.6)",
        "ei(0.05, v_tol=0.07)",
        "mzv(5,0.6,0.9,1.0)",
        "ei(0.05, 0.06)",
        "mzv(3.0,0.6)",
        "mzv(3,0)",
        "mzv()",
        "zv()",
        "mzv(1..2)",
        "mzv(.)",
        "mzv(1.2.3)",
    ];
    for name in names {
        for &(freq, damp) in &[(38.5, 0.1), (25.0, 0.0), (50.0, 0.2)] {
            let e = py.eval(&format!(
                "extras.shaper_defs.init_shaper({name:?}, {freq:?}, {damp:?})"
            ));
            check_opt_pair(&format!("init_shaper {name}"), e, shaper_defs::init_shaper(name, freq, damp, None));
            // With an error factory, ShaperError wraps but other errors
            // (TypeError/ValueError/math) propagate unchanged.
            let e = py.eval(&format!(
                "(lambda: extras.shaper_defs.init_shaper({name:?}, {freq:?}, {damp:?}, error=Exception))()"
            ));
            // Oracle raises generic Exception wrapper; compare only the
            // wrapped ShaperError cases by class+message shape below.
            let actual = shaper_defs::init_shaper(name, freq, damp, Some(fact));
            match (e, actual) {
                (Err(exc), Err(err)) => {
                    if exc.class_name == "Exception" {
                        assert_eq!(err.message(), exc.message, "{name} wrapped message");
                    } else {
                        assert_eq!(err.class_name(), exc.class_name, "{name} exception type");
                        assert_eq!(err.message(), exc.message, "{name} exception message");
                    }
                }
                (Err(exc), Ok(_)) => panic!("{name}: oracle raised {:?}, port succeeded", exc),
                (Ok(_), Ok(_)) => {}
                (Ok(want), Err(err)) => panic!("{name}: oracle returned {want:?}, port raised {}: {}", err.class_name(), err.message()),
            }
        }
    }
    // error=None ShaperError text is checked via check_opt_pair above;
    // additionally verify a Config-wrapped mixing error verbatim.
    let e = py.eval("extras.shaper_defs.init_shaper('mzv(5, t=0.6)', 38.5, 0.1, error=Exception)")
        .expect_err("oracle should raise wrapped ShaperError");
    match shaper_defs::init_shaper("mzv(5, t=0.6)", 38.5, 0.1, Some(fact)) {
        Err(err) => {
            assert_eq!(err.class_name(), "Error", "config-wrapped class");
            assert_eq!(err.message(), e.message);
            let _ = Exception { class_name: err.class_name().to_string(), message: err.message().to_string() };
        }
        Ok(_) => panic!("expected wrapped ShaperError"),
    }
}

#[test]
fn cfg_helpers_do_not_panic_on_zero_freq() {
    let Some(mut py) = skip_no_oracle(pyoracle::Oracle::start()) else { return };
    py.exec("import extras.shaper_defs").expect("import");
    // Zero damping_ratio hits df = 1.0 but K = 1.0; zero freq divides by zero.
    let e = py.eval("extras.shaper_defs.get_zv_shaper(0.0, 0.1)");
    check_pair("zv zero freq", e, shaper_defs::get_zv_shaper(0.0, 0.1));
    let e = py.eval("extras.shaper_defs.get_ei_shaper(0.0, 0.1)");
    check_pair("ei zero freq", e, shaper_defs::get_ei_shaper(0.0, 0.1));
    let e = py.eval("extras.shaper_defs.get_3hump_ei_shaper(0.0, 0.1)");
    check_pair("3hump zero freq", e, shaper_defs::get_3hump_ei_shaper(0.0, 0.1));
    // Damping above 1: sqrt of a negative number.
    let e = py.eval("extras.shaper_defs.get_zv_shaper(30.0, 1.5)");
    check_pair("zv damp>1", e, shaper_defs::get_zv_shaper(30.0, 1.5));
    // Large damping for exp overflow paths.
    let e = py.eval("extras.shaper_defs.get_zv_shaper(30.0, -200.0)");
    check_pair("zv damp -200", e, shaper_defs::get_zv_shaper(30.0, -200.0));
}
