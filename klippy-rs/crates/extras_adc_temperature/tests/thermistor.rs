//! Parity tests for the `Thermistor` math of `klippy/extras/thermistor.py`.
//!
//! 1. **Recording replay.** Every record in
//!    `tests/fixtures/recordings/thermistor/*.jsonl` is replayed through the
//!    port and compared exactly: return value (floats by hex + repr spelling),
//!    exception type/message when present, and emitted log lines. Recorded
//!    calls are methods on `Thermistor` objects; the objects are rebuilt the
//!    same way the Python built them: `PrinterThermistor(config, params)`
//!    creates `Thermistor(pullup, inline_resistor)` with `pullup_resistor`
//!    defaulting to 4700 and `inline_resistor` defaulting to 0, and none of
//!    the recorded scenarios overrides those options, so each fresh record
//!    object id starts as `Thermistor::new(4700.0, 0.0)` and is then driven
//!    through its recorded `setup_coefficients` / `setup_coefficients_beta`
//!    calls before any `calc_adc` call on it.
//!
//! 2. **Dense-grid oracle.** Every sensor in
//!    `klippy/extras/temperature_sensors.cfg` is evaluated over a dense grid
//!    of ADC values (for `calc_temp`) and temperatures (for `calc_adc`) and
//!    compared by float bits plus repr spelling, and exception type/message
//!    for the calls that raise. Expected values come live from the frozen
//!    reference through the `pyoracle` crate (set `KLIPPY_PYTHON` to the
//!    CPython 3.8.2 binary; the test skips when it is unset): the oracle
//!    reads the section parameters with the real `CustomThermistor` over the
//!    real `configfile`, builds a `Thermistor(4700.0, 0.0)` and drives it
//!    through `setup_coefficients` / `setup_coefficients_beta` with them,
//!    exactly as `PrinterThermistor` with default `pullup_resistor` /
//!    `inline_resistor` would. The config layer (`CustomThermistor`,
//!    `PrinterThermistor`) is ported in the `extras` crate and tested there.

use std::collections::BTreeMap;
use std::path::PathBuf;

use extras_adc_temperature::thermistor::Thermistor;
use pycompat::text::{float_repr, PyError};
use recording::hexfloat::{f64_from_hex, f64_to_hex};
use recording::reader::load_unit;
use recording::replay::assert_equal;
use recording::value::{FloatVal, Value};

/// Repository root, from the crate manifest directory (three levels up).
fn repo_root() -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    dir.ancestors()
        .nth(3)
        .expect("crate lives three levels below the repo root")
        .to_path_buf()
}

fn float_arg(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().expect("recorded float must decode"),
        other => panic!("expected a recorded float, got {other}"),
    }
}

fn str_arg(v: &Value) -> String {
    match v {
        Value::Str(s) => s.clone(),
        other => panic!("expected a recorded string, got {other}"),
    }
}

fn assert_float(label: &str, expected_hex: &str, expected_repr: &str, actual: f64) {
    // Compare by decoded bits plus repr spelling, not by raw hex string:
    // CPython `float.hex()` never trims trailing zero nibbles (13 hex
    // digits always), while the port's formatter trims them, so the same
    // value spells differently (e.g. `...9f870p+6` vs `...9f87p+6`).
    // `recording::assert_equal` compares float bits the same way.
    let bits_match = match expected_hex {
        "nan" => actual.is_nan(),
        "inf" => actual == f64::INFINITY,
        "-inf" => actual == f64::NEG_INFINITY,
        _ => f64_from_hex(expected_hex).map(f64::to_bits) == Some(actual.to_bits()),
    };
    let rep = float_repr(actual);
    assert!(
        bits_match && rep == expected_repr,
        "{label}: expected {expected_hex} / {expected_repr}, got {} / {rep}",
        f64_to_hex(actual)
    );
}

/// Replay every record in `tests/fixtures/recordings/thermistor/`.
#[test]
fn replays_recordings() {
    let root = repo_root().join("tests/fixtures/recordings");
    let scenarios = load_unit(&root, "thermistor").expect("load thermistor recordings");
    assert!(
        !scenarios.is_empty(),
        "no thermistor recordings found under {root:?}"
    );
    let mut total = 0usize;
    for scenario in scenarios {
        // Object state rebuilt per scenario: each `Thermistor#<n>` starts as
        // Thermistor::new(4700.0, 0.0) (PrinterThermistor defaults).
        let mut objects: BTreeMap<String, Thermistor> = BTreeMap::new();
        for rec in &scenario {
            total += 1;
            let oid = rec.object_id.as_deref().unwrap_or("?");
            match rec.function.as_str() {
                "Thermistor.setup_coefficients" => {
                    let t = objects.entry(oid.to_string()).or_insert_with(|| Thermistor::new(4700.0, 0.0));
                    // args[0] is the receiver, args[1..7] the floats, args[7] the name.
                    let log_lines = t
                        .setup_coefficients(
                            float_arg(&rec.args[1]),
                            float_arg(&rec.args[2]),
                            float_arg(&rec.args[3]),
                            float_arg(&rec.args[4]),
                            float_arg(&rec.args[5]),
                            float_arg(&rec.args[6]),
                            &str_arg(&rec.args[7]),
                        )
                        .unwrap_or_else(|e| panic!("{oid}: setup_coefficients raised {}", e.display()));
                    assert_eq!(
                        log_lines,
                        rec.side_effects.log_lines,
                        "{oid}: log lines diverged on seq {} ({}::{})",
                        rec.seq,
                        rec.scenario,
                        rec.function,
                    );
                    check_null(rec);
                }
                "Thermistor.setup_coefficients_beta" => {
                    let t = objects.entry(oid.to_string()).or_insert_with(|| Thermistor::new(4700.0, 0.0));
                    t.setup_coefficients_beta(
                        float_arg(&rec.args[1]),
                        float_arg(&rec.args[2]),
                        float_arg(&rec.args[3]),
                    )
                    .unwrap_or_else(|e| panic!("{oid}: setup_coefficients_beta raised {}", e.display()));
                    assert_eq!(
                        Vec::<String>::new(),
                        rec.side_effects.log_lines,
                        "{oid}: beta setup must not log (seq {})",
                        rec.seq,
                    );
                    check_null(rec);
                }
                "Thermistor.calc_adc" => {
                    let t = objects.get(oid).unwrap_or_else(|| panic!("{oid}: calc_adc before setup"));
                    let actual = t
                        .calc_adc(float_arg(&rec.args[1]))
                        .unwrap_or_else(|e| panic!("{oid}: calc_adc raised {}", e.display()));
                    assert_equal(
                        rec,
                        "ret",
                        rec.ret.as_ref().expect("calc_adc record has no ret"),
                        &Value::Float(FloatVal::new(actual)),
                    )
                    .unwrap_or_else(|d| panic!("divergence replaying calc_adc: {d}"));
                    assert!(rec.side_effects.log_lines.is_empty(), "{oid}: calc_adc logs nothing");
                }
                "Thermistor.calc_temp" => {
                    let t = objects.get(oid).unwrap_or_else(|| panic!("{oid}: calc_temp before setup"));
                    let actual = t
                        .calc_temp(float_arg(&rec.args[1]))
                        .unwrap_or_else(|e| panic!("{oid}: calc_temp raised {}", e.display()));
                    assert_equal(
                        rec,
                        "ret",
                        rec.ret.as_ref().expect("calc_temp record has no ret"),
                        &Value::Float(FloatVal::new(actual)),
                    )
                    .unwrap_or_else(|d| panic!("divergence replaying calc_temp: {d}"));
                    assert!(rec.side_effects.log_lines.is_empty(), "{oid}: calc_temp logs nothing");
                }
                other => panic!("unexpected recorded function {other} at seq {}", rec.seq),
            }
        }
    }
    assert!(total > 0, "thermistor recordings replayed zero records");
}

fn check_null(rec: &recording::Record) {
    assert_equal(
        rec,
        "ret",
        rec.ret.as_ref().expect("setup record has no ret"),
        &Value::Null,
    )
    .unwrap_or_else(|d| panic!("divergence replaying setup: {d}"));
}

/// Dense ADC grid for `calc_temp`, including the clamp boundaries and cases
/// outside them.
fn adc_grid() -> Vec<f64> {
    let mut v = vec![0.0, 1.0, -0.5, 1.5, 0.000009, 0.999991, 0.00001, 0.99999];
    // 1000 evenly spaced readings across the ADC range.
    for i in 0..=999u32 {
        v.push(0.00001 + (0.99999 - 0.00001) * (i as f64) / 999.0);
    }
    v
}

/// Dense temperature grid for `calc_adc`, including the Kelvin boundary and
/// cases outside it.
fn temp_grid() -> Vec<f64> {
    let mut v = vec![-1000.0, -300.0, -273.15, -273.0, 1000.0];
    let mut t = -270.0;
    while t <= 400.0 {
        v.push(t);
        t += 5.0;
    }
    v
}

/// Compare one pyoracle result with the port's result for the same call.
fn check_oracle(label: &str, expected: pyoracle::OracleResult, actual: Result<f64, PyError>) {
    match (expected, actual) {
        (Ok(Value::Float(f)), Ok(a)) => assert_float(label, &f.hex, &f.repr, a),
        (Ok(v), Ok(_)) => panic!("{label}: oracle returned {v:?}, expected a float"),
        (Ok(v), Err(e)) => panic!("{label}: oracle returned {v:?}, port raised {}: {}", e.kind, e.msg),
        (Err(exc), Err(e)) => {
            assert_eq!(e.kind, exc.class_name, "{label} exception type");
            assert_eq!(e.msg, exc.message, "{label} exception message");
        }
        (Err(exc), Ok(a)) => panic!(
            "{label}: oracle raised {}: {}, port returned {}",
            exc.class_name, exc.message, a
        ),
    }
}

/// Python: one `Thermistor(4700., 0.)` per `[thermistor ...]` section of
/// `temperature_sensors.cfg`, parameterized as `PrinterThermistor` does from
/// the real `CustomThermistor` (real `configfile` parse of the same text).
const ORACLE_SETUP: &str = r#"
import configfile as _cf
import extras.thermistor as _thermistor

def build_defaults(text):
    fileconfig = _cf.ConfigFileReader().build_fileconfig(text, 'temperature_sensors.cfg')
    config = _cf.ConfigWrapper(None, fileconfig, {}, 'printer')
    sensors = []
    for section in config.get_prefix_sections('thermistor '):
        custom = _thermistor.CustomThermistor(section)
        params = custom.params
        th = _thermistor.Thermistor(4700., 0.)
        if 'beta' in params:
            th.setup_coefficients_beta(params['t1'], params['r1'], params['beta'])
        else:
            th.setup_coefficients(params['t1'], params['r1'], params['t2'], params['r2'],
                                  params['t3'], params['r3'], name=custom.name)
        sensors.append((custom.name, params, th))
    return sensors
"#;

/// Dense-grid oracle comparison against the reference Python implementation.
#[test]
fn dense_grid_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!(
            "skipping dense-grid oracle: set KLIPPY_PYTHON to a CPython 3.8.2 \
             binary and run with the repo checkout available"
        );
        return;
    };

    let cfg_path = repo_root().join("klippy/extras/temperature_sensors.cfg");
    let cfg_text = std::fs::read_to_string(&cfg_path).expect("temperature_sensors.cfg must be readable");
    py.exec(ORACLE_SETUP).expect("oracle setup");
    py.exec(&format!("SENSORS = build_defaults({cfg_text:?})"))
        .expect("oracle default thermistors");
    let count = match py.eval("len(SENSORS)") {
        Ok(Value::Int(n)) => n,
        other => panic!("len(SENSORS) -> {other:?}"),
    };
    assert_eq!(count, 8, "temperature_sensors.cfg defines 8 thermistors");

    for i in 0..count {
        let name = match py.eval(&format!("SENSORS[{i}][0]")) {
            Ok(Value::Str(s)) => s,
            other => panic!("sensor name -> {other:?}"),
        };
        // The recorded parameters, as the oracle's `CustomThermistor` read
        // them, drive the Rust converter.
        let params = match py.eval(&format!("list(SENSORS[{i}][1].items())")) {
            Ok(Value::List(items)) => items
                .into_iter()
                .map(|item| match item {
                    Value::Tuple(mut kv) if kv.len() == 2 => {
                        let value = float_arg(&kv.pop().expect("value"));
                        (str_arg(&kv.pop().expect("key")), value)
                    }
                    other => panic!("params item {other:?}"),
                })
                .collect::<BTreeMap<String, f64>>(),
            other => panic!("params -> {other:?}"),
        };
        let mut t = Thermistor::new(4700.0, 0.0);
        match params.get("beta") {
            Some(&beta) => t.setup_coefficients_beta(params["t1"], params["r1"], beta).expect("beta setup"),
            None => {
                t.setup_coefficients(
                    params["t1"],
                    params["r1"],
                    params["t2"],
                    params["r2"],
                    params["t3"],
                    params["r3"],
                    &name,
                )
                .expect("sh setup");
            }
        };
        let lit = float_repr;
        for input in adc_grid() {
            let expected = py.eval(&format!("SENSORS[{i}][2].calc_temp({})", lit(input)));
            check_oracle(&format!("{name}.calc_temp({input})"), expected, t.calc_temp(input));
        }
        for input in temp_grid() {
            let expected = py.eval(&format!("SENSORS[{i}][2].calc_adc({})", lit(input)));
            check_oracle(&format!("{name}.calc_adc({input})"), expected, t.calc_adc(input));
        }
    }
}
