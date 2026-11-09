//! Parity tests for `klippy/extras/display/font8x14.py`.
//!
//! The Python module is pure data: one `VGA_FONT` table of 256 16-byte
//! glyph bitmaps, with no classes, functions, config options, G-code
//! commands or printer objects. Every comparison below is against the
//! live reference through `pyoracle` (CPython 3.8.2 + the frozen
//! `klippy/`): the reference file is loaded directly (it has no
//! imports, so no printer/config/MCU fakes are needed), and the whole
//! table, each glyph by index, the lengths and the bytearray copies
//! the display drivers make (`st7920.CharGlyphs`, `uc1701` swizzling)
//! are compared byte for byte as `recording::Value`s.
//!
//! All oracle comparisons run in a single sequential session: the x86
//! oracle container is memory constrained, and one CPython child at a
//! time is the only footprint the parity checks need (running one
//! oracle per `#[test]` would spawn several interpreters concurrently
//! under cargo's parallel harness).
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so expected values come from the oracle alone. When
//! `$KLIPPY_PYTHON` is unset the oracle test skips.

use extras::display::font8x14::{module_factory, VGA_FONT};
use recording::replay::assert_equal;
use recording::value::{Record, RecordKind, SideEffects, Value};
use std::path::PathBuf;

/// Repository root, from the crate manifest directory (three levels up).
fn repo_root() -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    dir.ancestors()
        .nth(3)
        .expect("crate lives three levels below the repo root")
        .to_path_buf()
}

/// Starts the oracle and loads the reference module from its file.
/// Returns `None` outside the oracle image (test then skips).
fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    let path = repo_root().join("klippy/extras/display/font8x14.py");
    let code = format!(
        "import importlib.util\n\
         _spec = importlib.util.spec_from_file_location(\"font8x14_ref\", {path:?})\n\
         FONT = importlib.util.module_from_spec(_spec)\n\
         _spec.loader.exec_module(FONT)\n"
    );
    py.exec(&code).expect("load reference font8x14.py");
    Some(py)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.display.font8x14".to_string(),
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

/// The ported table as a recording value: one `Bytes` per glyph.
fn table_value() -> Value {
    Value::List(VGA_FONT.iter().map(|g| Value::bytes(g)).collect())
}

/// Table dimensions, and the factory against the reference module's entry points.
#[test]
fn table_shape_and_factory() {
    assert_eq!(VGA_FONT.len(), 256);
    for (i, glyph) in VGA_FONT.iter().enumerate() {
        assert_eq!(glyph.len(), 16, "glyph {i} must be 16 bytes");
    }
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.display.font8x14"), "entry points of extras.display.font8x14");
}

/// Whole-table and per-glyph parity, driver uses and module surface,
/// all in one sequential oracle session.
#[test]
fn vga_font_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };

    // The whole table, and its shape.
    check("VGA_FONT", py.eval("FONT.VGA_FONT"), table_value());
    check("len(VGA_FONT)", py.eval("len(FONT.VGA_FONT)"), Value::int(256));
    check(
        "glyph lengths",
        py.eval("[len(g) for g in FONT.VGA_FONT]"),
        Value::List(vec![Value::int(16); 256]),
    );
    let flat: Vec<u8> = VGA_FONT.iter().flat_map(|g| *g).collect();
    check(
        "joined bytes",
        py.eval("b''.join(FONT.VGA_FONT)"),
        Value::bytes(&flat),
    );

    // Every glyph by index (spot divisor: the whole-table check above
    // already pins each byte; this localises a divergence to an index).
    for (i, glyph) in VGA_FONT.iter().enumerate() {
        check(
            &format!("VGA_FONT[{i}]"),
            py.eval(&format!("FONT.VGA_FONT[{i}]")),
            Value::bytes(glyph),
        );
    }

    // st7920.py:17: CharGlyphs = {'degrees': bytearray(font8x14.VGA_FONT[0xf8])}
    check(
        "degrees glyph as bytearray",
        py.eval("bytearray(FONT.VGA_FONT[0xf8])"),
        Value::bytes(&VGA_FONT[0xf8]),
    );
    // uc1701.py:25-26: [self._swizzle_bits(bytearray(c)) for c in font8x14.VGA_FONT]
    // The port copies every glyph through bytearray the same way.
    for i in [0x00usize, 0x20, 0x41, 0x61, 0xf8, 0xff] {
        check(
            &format!("bytearray(VGA_FONT[{i:#04x}])"),
            py.eval(&format!("bytearray(FONT.VGA_FONT[{i}])")),
            Value::bytes(&VGA_FONT[i]),
        );
    }

    // The reference module exports exactly one public name, so the
    // Rust `module_factory()` must be empty (asserted in
    // `table_shape_and_factory`).
    check(
        "public names",
        py.eval("sorted(n for n in dir(FONT) if not n.startswith('__'))"),
        Value::List(vec![Value::str("VGA_FONT")]),
    );
    check(
        "no load_config",
        py.eval("(hasattr(FONT, 'load_config'), hasattr(FONT, 'load_config_prefix'), \
                 hasattr(FONT, 'add_printer_objects'), hasattr(FONT, 'add_early_printer_objects'))"),
        Value::Tuple(vec![
            Value::Bool(false),
            Value::Bool(false),
            Value::Bool(false),
            Value::Bool(false),
        ]),
    );
}
