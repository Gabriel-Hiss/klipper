use recording::Value;

#[test]
fn evaluates_against_frozen_klippy() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec("import msgproto").unwrap();
    let crc = py.eval("msgproto.crc16_ccitt(b'abc')").unwrap();
    let Value::List(items) = crc else { panic!("expected a list, got {crc:?}") };
    assert_eq!(items.len(), 2);
    let third = py.eval("1/3").unwrap();
    let Value::Float(f) = third else { panic!("expected a float") };
    assert_eq!(f.to_f64().unwrap().to_bits(), (1.0f64 / 3.0).to_bits());
    let err = py.eval("int('x')").unwrap_err();
    assert_eq!(err.class_name, "ValueError");
    assert_eq!(err.message, "invalid literal for int() with base 10: 'x'");
}
