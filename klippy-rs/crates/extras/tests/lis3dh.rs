//! Parity tests for `extras::lis3dh` (port of `klippy/extras/lis3dh.py`).
//!
//! The module has no G-code commands, config options, status fields,
//! endpoints, MCU commands or log lines of its own: both entry points
//! only forward to `lis2dw.LIS2DW(config, lis2dw.LIS3DH_TYPE)`. The
//! forwarding is proven against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): the reference `lis3dh`
//! module is imported with `lis2dw.LIS2DW` replaced by a recording
//! fake, so `load_config` / `load_config_prefix` run with a fake
//! config and the constructed `(config, lis_type)` pair, the returned
//! instance and constructor-exception propagation are compared exactly
//! (returned values by recording value, exceptions by class and
//! message). The Rust side exercises what it owns: the factory shape
//! (both entry points exposed, no `add_*` entry points) and the value
//! of the forwarded `LIS3DH_TYPE` constant against the reference's.
//!
//! Full `LIS2DW` construction parity (config reads inside the sensor
//! constructor, MCU commands, status) belongs to the `lis2dw` unit.

use extras::lis3dh::{load_config, load_config_prefix, module_factory};
use extras::lis2dw::LIS3DH_TYPE;
use recording::replay::assert_equal;
use recording::value::{Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.lis3dh as lis3dh
import extras.lis2dw as lis2dw

class FakeConfig:
    def __init__(self, name):
        self._name = name
    def get_name(self):
        return self._name
    def get_printer(self):
        return None

constructed = []
class FakeLIS2DW:
    def __init__(self, config, lis_type):
        self.config = config
        self.lis_type = lis_type
        constructed.append(self)

class Boom(Exception):
    pass

class RaisingLIS2DW:
    def __init__(self, config, lis_type):
        raise Boom("boom: constructor failed")

real_lis2dw_class = lis2dw.LIS2DW
lis2dw.LIS2DW = FakeLIS2DW
cfg = FakeConfig("lis3dh")
other_cfg = FakeConfig("lis3dh second")
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("lis3dh fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping lis3dh oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.lis3dh.ModuleHeader".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

/// The observed construction: the fake recorded exactly one instance,
/// built from the given config and the port's forwarded `LIS3DH_TYPE`.
/// The type string flows from the Rust constant and is cross-checked
/// against the reference value by `forwarded_lis_type_matches_python`.
fn construction_value() -> Value {
    Value::Tuple(vec![
        Value::Bool(true),
        Value::str(LIS3DH_TYPE),
        Value::Bool(true),
        Value::int(1),
    ])
}

// ---------------------------------------------------------------------------
// Rust-side factory shape.
// ---------------------------------------------------------------------------

#[test]
fn factory_exposes_exactly_the_python_entry_points() {
    let factory = module_factory();
    type LoadFn = fn(&klippy_api::config::Config) -> klippy_api::error::KResult<Option<klippy_api::ObjectRef>>;
    // `assert_eq!` on the fn items needs `Debug`, which fn items lack, and
    // the expected side would not coerce to the factory's fn-pointer type.
    assert!(matches!(factory.load_config, Some(f) if std::ptr::fn_addr_eq(f, load_config as LoadFn)));
    assert!(matches!(factory.load_config_prefix, Some(f) if std::ptr::fn_addr_eq(f, load_config_prefix as LoadFn)));
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

// ---------------------------------------------------------------------------
// Oracle tests.
// ---------------------------------------------------------------------------

/// The reference module exposes exactly `load_config` and
/// `load_config_prefix` (matching the Rust factory shape above).
#[test]
fn python_entry_points_match_factory_shape() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval(
        "(hasattr(lis3dh, 'load_config'), hasattr(lis3dh, 'load_config_prefix'), \
         hasattr(lis3dh, 'add_early_printer_objects'), hasattr(lis3dh, 'add_printer_objects'))",
    );
    let actual = Value::Tuple(vec![
        Value::Bool(true),
        Value::Bool(true),
        Value::Bool(false),
        Value::Bool(false),
    ]);
    check("entry points", expected, actual);
}

/// The `LIS3DH_TYPE` the port forwards matches the reference constant
/// exactly, and the reference module forwards that same constant.
#[test]
fn forwarded_lis_type_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("constructed.clear()").expect("clear");
    py.exec("obj = lis3dh.load_config(cfg)").expect("load_config");
    let expected = py.eval("(obj.lis_type, obj.lis_type == lis2dw.LIS3DH_TYPE)");
    let actual = Value::Tuple(vec![Value::str(LIS3DH_TYPE), Value::Bool(true)]);
    check("forwarded lis_type", expected, actual);
    let expected_ref = py.eval("lis2dw.LIS3DH_TYPE");
    check("reference constant", expected_ref, Value::str(LIS3DH_TYPE));
}

/// `load_config(config)` on the reference constructs `LIS2DW(config,
/// LIS3DH_TYPE)` with the identical config object and returns the
/// constructed instance, exactly once.
#[test]
fn load_config_constructs_lis2dw_with_same_config() {
    let Some(mut py) = oracle() else { return };
    py.exec("constructed.clear()").expect("clear");
    py.exec("obj = lis3dh.load_config(cfg)").expect("load_config");
    let expected =
        py.eval("(isinstance(obj, FakeLIS2DW), obj.lis_type, obj.config is cfg, len(constructed))");
    check("load_config", expected, construction_value());
}

/// `load_config_prefix(config)` behaves identically to `load_config`.
#[test]
fn load_config_prefix_constructs_lis2dw_with_same_config() {
    let Some(mut py) = oracle() else { return };
    py.exec("constructed.clear()").expect("clear");
    py.exec("obj = lis3dh.load_config_prefix(other_cfg)").expect("load_config_prefix");
    let expected =
        py.eval("(isinstance(obj, FakeLIS2DW), obj.lis_type, obj.config is other_cfg, len(constructed))");
    check("load_config_prefix", expected, construction_value());
}

/// A constructor exception propagates unchanged out of both entry
/// points; the Rust side mirrors this with `?` on `LIS2DW::new`.
#[test]
fn constructor_exceptions_propagate_from_both_entry_points() {
    let Some(mut py) = oracle() else { return };
    py.exec("lis2dw.LIS2DW = RaisingLIS2DW").expect("patch");
    for (label, call) in [
        ("load_config", "lis3dh.load_config(cfg)"),
        ("load_config_prefix", "lis3dh.load_config_prefix(cfg)"),
    ] {
        let exc = match py.eval(call) {
            Err(e) => e,
            Ok(_) => panic!("{label}: expected Boom, got a return value"),
        };
        assert_eq!(exc.class_name, "Boom", "{label}: exception class");
        assert_eq!(exc.message, "boom: constructor failed", "{label}: exception message");
    }
    py.exec("lis2dw.LIS2DW = real_lis2dw_class").expect("restore");
}
