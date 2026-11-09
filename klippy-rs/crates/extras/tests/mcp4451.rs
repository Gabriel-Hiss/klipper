//! Parity tests for `extras::mcp4451` (port of `klippy/extras/mcp4451.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): the reference module is built
//! against a fake I2C device and the real `configfile.ConfigWrapper`
//! (option reads, range checks and error messages come from the reference
//! implementation), and the port is built against fakes of the same shape.
//! Compared for every scenario: the status (object or exception class and
//! message), the exact I2C write sequence passed to the device, and the
//! exact sequence of config option reads (option name, default and bounds).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::mcp4451::{module_factory, Mcp4451, WIPER_REGISTERS};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::float_str;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.mcp4451 as mcp4451
from configfile import ConfigWrapper, sentinel

class FakePrinter:
    config_error = configparser.Error

class FakeI2C:
    def __init__(self, address=44):
        self.address = address
        self.writes = []
    def get_i2c_address(self):
        return self.address
    def i2c_write(self, data):
        self.writes.append(list(bytearray(data)))

_fake_i2c = FakeI2C(44)
def _fake_i2c_from_config(config, default_addr=None, default_speed=100000, async_write_only=False):
    return _fake_i2c
mcp4451.bus.MCU_I2C_from_config = _fake_i2c_from_config

class LoggingConfig(ConfigWrapper):
    def __init__(self, *args):
        ConfigWrapper.__init__(self, *args)
        self.reads = []
    def getfloat(self, option, default=sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        def fmt(v):
            if v is sentinel:
                return "sentinel"
            return repr(v)
        self.reads.append("|".join([repr(option), fmt(default), repr(minval), repr(maxval), repr(above), repr(below)]))
        return ConfigWrapper.getfloat(self, option, default, minval=minval, maxval=maxval, above=above, below=below, note_valid=note_valid)

def _make_config(options):
    parser = configparser.ConfigParser()
    lines = "[mcp4451 test]\n"
    for k, v in options:
        lines += "%s = %s\n" % (k, v)
    parser.read_string(lines)
    return LoggingConfig(FakePrinter(), parser, {}, "mcp4451 test")

def run_case(options, address=44):
    global _fake_i2c
    _fake_i2c = FakeI2C(address)
    config = _make_config(options)
    try:
        mcp4451.load_config_prefix(config)
        return ("ok", _fake_i2c.writes, config.reads)
    except Exception as e:
        return ("err", type(e).__name__, str(e), _fake_i2c.writes, config.reads)

def run_set_register(reg, value, address=44):
    global _fake_i2c
    _fake_i2c = FakeI2C(address)
    config = _make_config([])
    try:
        obj = mcp4451.load_config_prefix(config)
        obj.set_register(reg, value)
        return ("ok", _fake_i2c.writes)
    except Exception as e:
        return ("err", type(e).__name__, str(e), _fake_i2c.writes)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter;

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        _name: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeI2c {
    address: i64,
    writes: GCell<Vec<Vec<i64>>>,
}

impl FakeI2c {
    fn new(address: i64) -> Arc<FakeI2c> {
        Arc::new(FakeI2c {
            address,
            writes: GCell::new(Vec::new()),
        })
    }
}

impl extras::bus::McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 { panic!("get_oid unused") }
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { panic!("get_mcu unused") }
    fn get_i2c_address(&self) -> i64 {
        self.address
    }
    fn get_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        panic!("get_command_queue unused")
    }
    fn build_config(&self) -> KResult<()> { Ok(()) }
    fn i2c_write_noack(&self, _data: &[u8], _opts: klippy_api::mcu::SendOpts) -> KResult<()> {
        panic!("i2c_write_noack unused")
    }
    fn i2c_write(&self, data: &[u8], _opts: klippy_api::mcu::QuerySendOpts) -> KResult<()> {
        self.writes
            .borrow_mut()
            .push(data.iter().map(|&b| b as i64).collect());
        Ok(())
    }
    fn i2c_read(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("i2c_read unused")
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("i2c_transfer unused")
    }
}

/// Faithful `ConfigWrapper._get_wrapper` clone for the `getfloat` reads
/// `mcp4451` performs (`configfile.py:29`): missing-option default
/// handling, `float()` parsing, then `minval`/`maxval`/`above`/`below`
/// checks in that order with the exact CPython messages.
struct FakeConfig {
    printer: Printer,
    section: String,
    options: Vec<(String, String)>,
    reads: GCell<Vec<String>>,
}

impl FakeConfig {
    fn new(options: &[(&str, &str)], section: &str) -> Arc<FakeConfig> {
        let printer: Printer = Arc::new(FakePrinter);
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            options: options
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
            reads: GCell::new(Vec::new()),
        })
    }

    fn fmt_bound(v: Option<f64>) -> String {
        match v {
            None => "None".to_string(),
            Some(v) => float_str(v),
        }
    }

    fn fmt_default(v: Option<Option<f64>>) -> String {
        match v {
            None => "sentinel".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => float_str(v),
        }
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads.borrow_mut().push(format!(
            "'{option}'|{}|{}|{}|{}|{}",
            Self::fmt_default(opts.default),
            Self::fmt_bound(opts.minval),
            Self::fmt_bound(opts.maxval),
            Self::fmt_bound(opts.above),
            Self::fmt_bound(opts.below),
        ));
        let raw = match self.options.iter().find(|(k, _)| k == option) {
            None => {
                return match opts.default {
                    None => Err(klippy_api::error::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                    Some(d) => Ok(d),
                };
            }
            Some((_, v)) => v.clone(),
        };
        // `_get_wrapper` maps any non-configparser error (here the
        // `ValueError` of `float()`) to the "Unable to parse" message.
        let v = match pycompat::text::float_from_string(&raw) {
            Ok(v) => v,
            Err(_) => {
                return Err(klippy_api::error::KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.section
                )))
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(klippy_api::error::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(klippy_api::error::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(klippy_api::error::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(klippy_api::error::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.section,
                    float_str(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by mcp4451")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("mcp4451 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping mcp4451 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

enum Expected {
    Ok {
        writes: Vec<Vec<i64>>,
        reads: Vec<String>,
    },
    Err {
        class_name: String,
        message: String,
        writes: Vec<Vec<i64>>,
        reads: Vec<String>,
    },
}

fn as_int(v: &Value) -> i64 {
    match v {
        Value::Int(n) => *n,
        _ => panic!("expected Int, got {v:?}"),
    }
}

fn as_str(v: &Value) -> String {
    match v {
        Value::Str(s) => s.clone(),
        _ => panic!("expected Str, got {v:?}"),
    }
}

fn as_writes(v: &Value) -> Vec<Vec<i64>> {
    match v {
        Value::List(items) => items
            .iter()
            .map(|row| match row {
                Value::List(nums) => nums.iter().map(as_int).collect(),
                _ => panic!("expected List of writes, got {row:?}"),
            })
            .collect(),
        _ => panic!("expected writes List, got {v:?}"),
    }
}

fn as_reads(v: &Value) -> Vec<String> {
    match v {
        Value::List(items) => items.iter().map(as_str).collect(),
        _ => panic!("expected reads List, got {v:?}"),
    }
}

fn run_py_case(
    py: &mut pyoracle::Oracle,
    options: &[(&str, &str)],
    address: i64,
) -> Expected {
    let mut opts = String::from("[");
    for (i, (k, v)) in options.iter().enumerate() {
        if i > 0 {
            opts.push_str(", ");
        }
        opts.push_str(&format!("({k:?}, {v:?})"));
    }
    opts.push(']');
    let expr = format!("run_case({opts}, {address})");
    let res = py.eval(&expr).expect("py run_case");
    let Value::Tuple(items) = res else {
        panic!("py run_case did not return a tuple");
    };
    match &items[0] {
        Value::Str(status) if status == "ok" => Expected::Ok {
            writes: as_writes(&items[1]),
            reads: as_reads(&items[2]),
        },
        Value::Str(status) if status == "err" => Expected::Err {
            class_name: as_str(&items[1]),
            message: as_str(&items[2]),
            writes: as_writes(&items[3]),
            reads: as_reads(&items[4]),
        },
        other => panic!("unexpected run_case status: {other:?}"),
    }
}

fn writes_value(writes: &[Vec<i64>]) -> Value {
    Value::List(
        writes
            .iter()
            .map(|row| {
                Value::List(row.iter().map(|n| Value::int(*n)).collect())
            })
            .collect(),
    )
}

fn reads_value(reads: &[String]) -> Value {
    Value::List(reads.iter().map(|s| Value::Str(s.clone())).collect())
}

fn check_equal(label: &str, expected: Value, actual: Value) {
    let record = Record {
        v: 1,
        unit: "mcp4451".to_string(),
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

/// Builds the Rust object exactly as the reference was built and compares
/// status, writes and reads with the oracle's answer.
fn check_case(label: &str, py: &mut pyoracle::Oracle, options: &[(&str, &str)], address: i64) {
    let expected = run_py_case(py, options, address);
    let fake = FakeConfig::new(options, "mcp4451 test");
    let config: Config = fake.clone();
    let i2c = FakeI2c::new(address);
    let result = Mcp4451::new(&config, i2c.clone());
    let writes = i2c.writes.borrow().clone();
    let reads = fake.reads.borrow().clone();
    match expected {
        Expected::Ok {
            writes: exp_writes,
            reads: exp_reads,
        } => {
            let rust = match result {
                Ok(r) => r,
                Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
            };
            drop(rust);
            check_equal(&format!("{label} writes"), writes_value(&exp_writes), writes_value(&writes));
            check_equal(&format!("{label} reads"), reads_value(&exp_reads), reads_value(&reads));
        }
        Expected::Err {
            class_name,
            message,
            writes: exp_writes,
            reads: exp_reads,
        } => {
            let e = match result {
                Ok(_) => panic!("{label}: rust returned an object but the call must fail"),
                Err(e) => e,
            };
            assert_eq!(
                e.class_name(),
                class_name.as_str(),
                "{label}: exception class"
            );
            assert_eq!(e.message(), message.as_str(), "{label}: exception message");
            check_equal(&format!("{label} writes"), writes_value(&exp_writes), writes_value(&writes));
            check_equal(&format!("{label} reads"), reads_value(&exp_reads), reads_value(&reads));
        }
    }
}

/// Drives one `set_register` call on both sides and compares the writes
/// (and the raised error, when the register values are outside the byte
/// range the Python bus layer packs with `bytearray`).
fn check_set_register(py: &mut pyoracle::Oracle, reg: i64, value: i64) {
    let label = format!("set_register {reg:#x} {value} writes");
    let expected = py
        .eval(&format!("run_set_register({reg}, {value})"))
        .expect("py run_set_register");
    let Value::Tuple(items) = expected else {
        panic!("py run_set_register did not return a tuple");
    };
    let fake = FakeConfig::new(&[], "mcp4451 test");
    let config: Config = fake.clone();
    let i2c = FakeI2c::new(44);
    let obj = Mcp4451::new(&config, i2c.clone()).expect("rust new");
    let result = obj.set_register(reg, value);
    let writes = i2c.writes.borrow().clone();
    match &items[0] {
        Value::Str(status) if status == "ok" => {
            result.expect("rust set_register");
            check_equal(&label, writes_value(&as_writes(&items[1])), writes_value(&writes));
        }
        Value::Str(status) if status == "err" => {
            let e = result.expect_err("rust set_register must fail");
            assert_eq!(e.class_name(), as_str(&items[1]).as_str(), "{label}: exception class");
            assert_eq!(e.message(), as_str(&items[2]).as_str(), "{label}: exception message");
            check_equal(&label, writes_value(&as_writes(&items[3])), writes_value(&writes));
        }
        other => panic!("unexpected run_set_register status: {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn wiper_registers_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = match py.eval("mcp4451.WiperRegisters").expect("eval WiperRegisters") {
        Value::List(items) => items.iter().map(as_int).collect::<Vec<_>>(),
        other => panic!("WiperRegisters is not a list: {other:?}"),
    };
    let actual = WIPER_REGISTERS.to_vec();
    check_equal(
        "wiper_registers_match_python",
        writes_value(&[expected]),
        writes_value(&[actual]),
    );
}

#[test]
fn no_options_configures_control_registers_only() {
    let Some(mut py) = oracle() else { return };
    check_case("no_options", &mut py, &[], 44);
}

#[test]
fn all_wipers_at_scale() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "all_wipers_at_scale",
        &mut py,
        &[
            ("scale", "2.0"),
            ("wiper_0", "0.0"),
            ("wiper_1", "2.0"),
            ("wiper_2", "1.0"),
            ("wiper_3", "0.5"),
        ],
        44,
    );
}

#[test]
fn single_wiper() {
    let Some(mut py) = oracle() else { return };
    check_case("single_wiper", &mut py, &[("wiper_2", "0.75")], 47);
}

#[test]
fn scale_defaults_to_one() {
    let Some(mut py) = oracle() else { return };
    check_case("scale_defaults_to_one", &mut py, &[("wiper_0", "0.5")], 44);
}

#[test]
fn scaled_rounding() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "scaled_rounding",
        &mut py,
        &[
            ("scale", "510"),
            ("wiper_0", "3"),
            ("wiper_1", "1"),
            ("wiper_3", "255"),
        ],
        44,
    );
}

#[test]
fn underscore_float_scale() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "underscore_float_scale",
        &mut py,
        &[("scale", "1_0"), ("wiper_0", "5")],
        44,
    );
}

#[test]
fn infinite_scale() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "infinite_scale",
        &mut py,
        &[("scale", "inf"), ("wiper_0", "2")],
        44,
    );
}

#[test]
fn address_boundaries() {
    let Some(mut py) = oracle() else { return };
    check_case("address_44", &mut py, &[], 44);
    check_case("address_47", &mut py, &[], 47);
}

#[test]
fn address_out_of_range() {
    let Some(mut py) = oracle() else { return };
    check_case("address_43", &mut py, &[], 43);
    check_case("address_48", &mut py, &[], 48);
    check_case("address_0", &mut py, &[], 0);
}

#[test]
fn scale_zero_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("scale_zero_fails", &mut py, &[("scale", "0")], 44);
}

#[test]
fn scale_negative_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("scale_negative_fails", &mut py, &[("scale", "-1")], 44);
}

#[test]
fn scale_unparseable_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("scale_unparseable_fails", &mut py, &[("scale", "abc")], 44);
}

#[test]
fn scale_empty_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("scale_empty_fails", &mut py, &[("scale", "")], 44);
}

#[test]
fn nan_scale_fails_on_wiper() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "nan_scale_fails_on_wiper",
        &mut py,
        &[("scale", "nan"), ("wiper_0", "1")],
        44,
    );
}

#[test]
fn wiper_below_minimum_fails() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "wiper_below_minimum_fails",
        &mut py,
        &[("wiper_0", "1"), ("wiper_1", "-0.1")],
        44,
    );
}

#[test]
fn wiper_above_maximum_fails() {
    let Some(mut py) = oracle() else { return };
    check_case(
        "wiper_above_maximum_fails",
        &mut py,
        &[("scale", "3"), ("wiper_2", "3.5")],
        44,
    );
}

#[test]
fn wiper_unparseable_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("wiper_unparseable_fails", &mut py, &[("wiper_0", "xyz")], 44);
}

#[test]
fn wiper_empty_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("wiper_empty_fails", &mut py, &[("wiper_0", "")], 44);
}

#[test]
fn wiper_nan_fails() {
    let Some(mut py) = oracle() else { return };
    check_case("wiper_nan_fails", &mut py, &[("wiper_0", "nan")], 44);
}

#[test]
fn wiper_inf_fails_above_maximum() {
    let Some(mut py) = oracle() else { return };
    check_case("wiper_inf_fails_above_maximum", &mut py, &[("wiper_0", "inf")], 44);
}

#[test]
fn set_register_bytes() {
    let Some(mut py) = oracle() else { return };
    check_set_register(&mut py, 0x04, 0xff);
    check_set_register(&mut py, 0x00, 0x00);
    check_set_register(&mut py, 0x07, 0xff);
    check_set_register(&mut py, 0x06, 511);
    check_set_register(&mut py, 0x01, 0x100);
    check_set_register(&mut py, 0x0a, 0x2ff);
}

#[test]
fn factory_shape() {
    let Some(_py) = oracle() else { return };
    let factory = module_factory();
    assert!(
        factory.load_config.is_none(),
        "mcp4451 has no load_config"
    );
    assert!(
        factory.load_config_prefix.is_some(),
        "mcp4451 exposes load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "mcp4451 has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "mcp4451 has no add_printer_objects"
    );
}
