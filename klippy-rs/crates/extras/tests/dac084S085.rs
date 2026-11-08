//! Parity tests for `extras::dac084S085` (port of `klippy/extras/dac084S085.py`).
//!
//! `dac084S085` has no G-code, status or endpoints; its only
//! observable behaviour is the config options it reads and the
//! SPI bytes it sends. Every expected value comes from the live
//! reference through `pyoracle`: a fresh `dac084S085` is built
//! on both sides with minimal Python fakes for the printer, the
//! config wrapper and the SPI bus, then fed identical `scale` /
//! `channel_A..D` configs and the resulting `spi_send` byte
//! lists and config errors are compared exactly (class and
//! message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply; expected values come
//! from the oracle alone.

use extras::bus::McuSpiApi;
use extras::dac084S085::Dac084S085;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::mcu::{CommandQueue, Mcu, QuerySendOpts, SendOpts};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::ObjectRef;
use klippy_api::EventArg;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.bus
import extras.dac084S085 as dac084S085

class _FakeSpi:
    def __init__(self):
        self.sent = []
        self.minclocks = []
        self.reqclocks = []
    def spi_send(self, data, minclock=0, reqclock=0):
        self.sent.append(list(data))
        self.minclocks.append(minclock)
        self.reqclocks.append(reqclock)

_fake_spis = []
def _fake_mcu_spi_from_config(config, mode, pin_option="cs_pin",
                              default_speed=100000, share_type=None,
                              cs_active_high=False):
    spi = _FakeSpi()
    _fake_spis.append((mode, pin_option, default_speed, spi))
    return spi
extras.bus.MCU_SPI_from_config = _fake_mcu_spi_from_config

class _Sentinel:
    pass

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, values):
        self._printer = printer
        self.section = section
        self.values = dict(values)
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def getfloat(self, option, default=_Sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        if option not in self.values:
            if default is _Sentinel:
                raise configparser.Error(
                    "Option '%s' in section '%s' must be specified"
                    % (option, self.section))
            return default
        raw = self.values[option]
        try:
            v = float(raw)
        except Exception:
            raise configparser.Error(
                "Unable to parse option '%s' in section '%s'"
                % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have minimum of %s"
                % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self.section, maxval))
        if above is not None and v <= above:
            raise configparser.Error(
                "Option '%s' in section '%s' must be above %s"
                % (option, self.section, above))
        if below is not None and v >= below:
            raise configparser.Error(
                "Option '%s' in section '%s' must be below %s"
                % (option, self.section, below))
        return v

class FakePrinter:
    config_error = configparser.Error

def make_dac(values):
    del _fake_spis[:]
    p = FakePrinter()
    c = FakeConfig(p, "dac084S085 test", values)
    obj = dac084S085.load_config_prefix(c)
    return obj
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

#[derive(Clone, Debug)]
enum FakeValue {
    Float(f64),
    Bad(String),
}

pub struct FakeSpi {
    sent: Mutex<Vec<(Vec<u8>, i64, i64)>>,
}

impl FakeSpi {
    fn new() -> Arc<FakeSpi> {
        Arc::new(FakeSpi {
            sent: Mutex::new(Vec::new()),
        })
    }
    fn sent(&self) -> Vec<(Vec<u8>, i64, i64)> {
        self.sent.lock().unwrap().clone()
    }
}

impl McuSpiApi for FakeSpi {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by dac084S085")
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        panic!("build_config is not used by dac084S085")
    }
    fn setup_shutdown_msg(&self, _shutdown_seq: &[u8]) -> KResult<()> {
        panic!("setup_shutdown_msg is not used by dac084S085")
    }
    fn spi_send(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.sent
            .lock()
            .unwrap()
            .push((data.to_vec(), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn spi_transfer(&self, _data: &[u8], _opts: QuerySendOpts) -> KResult<PyValue> {
        panic!("spi_transfer is not used by dac084S085")
    }
    fn spi_transfer_with_preface(
        &self,
        _preface_data: &[u8],
        _data: &[u8],
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("spi_transfer_with_preface is not used by dac084S085")
    }
}

struct FakeConfig {
    printer: Printer,
    values: indexmap::IndexMap<String, FakeValue>,
}

impl FakeConfig {
    fn new(values: &[(&str, FakeValue)]) -> Config {
        let printer: Printer = Arc::new(FakePrinter);
        let mut values_map = indexmap::IndexMap::new();
        for (k, v) in values {
            values_map.insert(k.to_string(), v.clone());
        }
        Arc::new(FakeConfig {
            printer,
            values: values_map,
        }) as Config
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "dac084S085 test".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let section = self.get_name();
        if !self.values.contains_key(option) {
            return match opts.default {
                None => Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be specified"
                ))),
                Some(d) => Ok(d),
            };
        }
        let bad = match self.values.get(option) {
            Some(FakeValue::Bad(_)) => true,
            _ => false,
        };
        if bad {
            return Err(klippy_api::KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{section}'"
            )));
        }
        let v = match self.values.get(option) {
            Some(FakeValue::Float(f)) => *f,
            _ => unreachable!(),
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must have minimum of {}",
                    pycompat::text::float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must have maximum of {}",
                    pycompat::text::float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be above {}",
                    pycompat::text::float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be below {}",
                    pycompat::text::float_str(below)
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
        panic!("getsection is not used by dac084S085")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("dac084S085 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping dac084S085 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "dac084S085".to_string(),
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

fn check_err<T>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<T>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned ok but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class",
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message",
    );
    assert_eq!(
        rust_err.class_name(),
        "Error",
        "{label}: dac084S085 errors are config errors"
    );
}

fn sent_value(spi: &FakeSpi) -> Value {
    Value::List(
        spi.sent()
            .iter()
            .map(|(data, _, _)| {
                Value::List(
                    data.iter()
                        .map(|b| Value::int(*b as i64))
                        .collect(),
                )
            })
            .collect(),
    )
}

/// Build a `(dac, spi, config)` pair on both sides with the same
/// `values`; Python side is `obj`, Rust side carries `sent`.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    values: &[(&str, FakeValue)],
) -> (Arc<FakeSpi>, Config) {
    let py_values = values
        .iter()
        .map(|(k, v)| match v {
            FakeValue::Float(f) => format!("('{k}', {f:?})"),
            FakeValue::Bad(s) => format!("('{k}', {s:?})"),
        })
        .collect::<Vec<_>>()
        .join(", ");
    py.exec(&format!("obj = make_dac([{py_values}])"))
        .expect("py make_dac");
    let spi = FakeSpi::new();
    let config = FakeConfig::new(values);
    let dac = Dac084S085::from_spi(&config, spi.clone())
        .expect("rust Dac084S085::from_spi");
    // Keep the Python object referenced under a name so readback
    // helpers below can see the fake SPI.
    let _ = dac;
    (spi, config)
}

fn py_sent(py: &mut pyoracle::Oracle) -> pyoracle::OracleResult {
    py.eval("_fake_spis[-1][3].sent")
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn bus_uses_mode1_enable_pin_10mhz() {
    let Some(mut py) = oracle() else { return };
    let _ = fresh_pair(&mut py, &[]);
    let expected = py.eval("_fake_spis[-1][0]");
    check(
        "bus mode",
        expected,
        Value::int(1),
    );
    let expected = py.eval("_fake_spis[-1][1]");
    check("bus pin option", expected, Value::str("enable_pin"));
    let expected = py.eval("_fake_spis[-1][2]");
    check("bus speed", expected, Value::int(10_000_000));
    let expected = py.eval("len(_fake_spis)");
    check("bus count", expected, Value::int(1));
}

#[test]
fn no_channels_sends_nothing() {
    let Some(mut py) = oracle() else { return };
    let (spi, _config) = fresh_pair(&mut py, &[]);
    let expected = py_sent(&mut py);
    check("no_channels sends", expected, sent_value(&spi));
}

#[test]
fn channel_value_bytes_with_default_scale() {
    let Some(mut py) = oracle() else { return };
    for val in [0.0f64, 0.5, 1.0, 0.1, 0.25, 255.0 / 1000.0] {
        let (spi, _config) = fresh_pair(
            &mut py,
            &[("channel_A", FakeValue::Float(val))],
        );
        let expected = py_sent(&mut py);
        check(
            &format!("channel_A default scale {val}"),
            expected,
            sent_value(&spi),
        );
    }
}

#[test]
fn channel_value_bytes_with_custom_scale() {
    let Some(mut py) = oracle() else { return };
    for (scale, val) in [
        (2.0f64, 1.0f64),
        (3.3, 1.0),
        (10.0, 0.5),
        (0.5, 0.25),
        (1.5, 1.5),
        (5.0, 0.0),
    ] {
        let (spi, _config) = fresh_pair(
            &mut py,
            &[
                ("scale", FakeValue::Float(scale)),
                ("channel_B", FakeValue::Float(val)),
            ],
        );
        let expected = py_sent(&mut py);
        check(
            &format!("custom scale {scale} channel_B {val}"),
            expected,
            sent_value(&spi),
        );
    }
}

#[test]
fn all_four_channels_in_order() {
    let Some(mut py) = oracle() else { return };
    let (spi, _config) = fresh_pair(
        &mut py,
        &[
            ("channel_A", FakeValue::Float(1.0)),
            ("channel_B", FakeValue::Float(0.5)),
            ("channel_C", FakeValue::Float(0.0)),
            ("channel_D", FakeValue::Float(0.25)),
        ],
    );
    let expected = py_sent(&mut py);
    check("four channels", expected, sent_value(&spi));
    // Clock arguments are the `spi_send` defaults (0, 0).
    for (_, minclock, reqclock) in spi.sent() {
        assert_eq!(minclock, 0);
        assert_eq!(reqclock, 0);
    }
}

#[test]
fn sparse_channels_skip_absent() {
    let Some(mut py) = oracle() else { return };
    let (spi, _config) = fresh_pair(
        &mut py,
        &[
            ("channel_B", FakeValue::Float(1.0)),
            ("channel_D", FakeValue::Float(0.5)),
        ],
    );
    let expected = py_sent(&mut py);
    check("sparse channels", expected, sent_value(&spi));
}

#[test]
fn set_register_packs_bytes() {
    let Some(mut py) = oracle() else { return };
    let (spi, config) = fresh_pair(&mut py, &[]);
    // Build a Rust dac for direct calls (same SPI, same config).
    let dac = Dac084S085::from_spi(&config, spi.clone()).expect("rust dac");
    // Python direct calls need a Python object: rebuild one whose
    // SPI is the fake (fresh pair already created one `obj`).
    for (chan, value) in [
        (0i64, 0i64),
        (0, 255),
        (1, 1),
        (2, 16),
        (3, 240),
        (0, 0x12),
        (1, 0xAB),
        (2, 0x34),
        (3, 255),
        (0, 15),
        (0, 16),
        (0, 31),
    ] {
        let before = spi.sent().len();
        let expected = py.eval(&format!("obj.set_register({chan}, {value})"))
            .map(|_| ())
            .and_then(|_| py_sent(&mut py));
        // Python set_register returns None; compare the new send.
        let py_list = match py.eval("_fake_spis[-1][3].sent[-1]") {
            Ok(v) => v,
            Err(e) => panic!("oracle set_register raised {}: {}", e.class_name, e.message),
        };
        dac.set_register(chan, value).expect("rust set_register");
        let rust_list = match spi.sent().get(before) {
            Some((data, _, _)) => Value::List(
                data.iter().map(|b| Value::int(*b as i64)).collect(),
            ),
            None => panic!("rust set_register sent nothing"),
        };
        check(
            &format!("set_register {chan} {value}"),
            Ok(py_list),
            rust_list,
        );
        let _ = expected;
    }
}

#[test]
fn scale_must_be_above_zero() {
    let Some(mut py) = oracle() else { return };
    for scale in [0.0f64, -1.0, -0.001] {
        let py_values = format!("('scale', {scale:?})");
        let expected = py.eval(&format!("make_dac([{py_values}])"));
        let config = FakeConfig::new(&[("scale", FakeValue::Float(scale))]);
        let spi = FakeSpi::new();
        let actual = Dac084S085::from_spi(&config, spi);
        check_err(&format!("scale above {scale}"), expected, actual);
    }
}

#[test]
fn channel_below_minimum_fails() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("make_dac([('channel_C', -0.5)])");
    let config = FakeConfig::new(&[("channel_C", FakeValue::Float(-0.5))]);
    let spi = FakeSpi::new();
    let actual = Dac084S085::from_spi(&config, spi);
    check_err("channel below min", expected, actual);
}

#[test]
fn channel_above_scale_fails() {
    let Some(mut py) = oracle() else { return };
    // Default scale 1.0.
    let expected = py.eval("make_dac([('channel_A', 1.5)])");
    let config = FakeConfig::new(&[("channel_A", FakeValue::Float(1.5))]);
    let spi = FakeSpi::new();
    let actual = Dac084S085::from_spi(&config, spi);
    check_err("channel above default scale", expected, actual);
    // Custom scale: channel past a smaller scale.
    let expected = py.eval("make_dac([('scale', 1.0), ('channel_B', 2.0)])");
    let config = FakeConfig::new(&[
        ("scale", FakeValue::Float(1.0)),
        ("channel_B", FakeValue::Float(2.0)),
    ]);
    let spi = FakeSpi::new();
    let actual = Dac084S085::from_spi(&config, spi);
    check_err("channel above custom scale", expected, actual);
}

#[test]
fn channel_at_scale_maximum_ok() {
    let Some(mut py) = oracle() else { return };
    for (scale, val) in [(1.0f64, 1.0f64), (2.5, 2.5), (0.5, 0.5)] {
        let (spi, _config) = fresh_pair(
            &mut py,
            &[
                ("scale", FakeValue::Float(scale)),
                ("channel_A", FakeValue::Float(val)),
            ],
        );
        let expected = py_sent(&mut py);
        check(
            &format!("channel at max {scale} {val}"),
            expected,
            sent_value(&spi),
        );
    }
}

#[test]
fn unparsable_float_fails() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("make_dac([('channel_A', 'not-a-float')])");
    let config = FakeConfig::new(&[(
        "channel_A",
        FakeValue::Bad("not-a-float".to_string()),
    )]);
    let spi = FakeSpi::new();
    let actual = Dac084S085::from_spi(&config, spi);
    check_err("unparsable channel", expected, actual);
    let expected = py.eval("make_dac([('scale', 'x')])");
    let config = FakeConfig::new(&[("scale", FakeValue::Bad("x".to_string()))]);
    let spi = FakeSpi::new();
    let actual = Dac084S085::from_spi(&config, spi);
    check_err("unparsable scale", expected, actual);
}

#[test]
fn factory_shape_matches_python() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::dac084S085::module_factory();
    assert!(
        factory.load_config.is_none(),
        "dac084S085 has no load_config"
    );
    let load_config_prefix = factory
        .load_config_prefix
        .expect("dac084S085 exposes load_config_prefix");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "dac084S085 has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "dac084S085 has no add_printer_objects"
    );
    // Python side has load_config_prefix and no load_config.
    py.exec("obj = make_dac([])").expect("py make_dac for class name");
    let expected = py.eval("hasattr(dac084S085, 'load_config_prefix')");
    check("py has prefix", expected, Value::Bool(true));
    let expected = py.eval("hasattr(dac084S085, 'load_config')");
    check("py no load_config", expected, Value::Bool(false));
    // The Python object type name matches the Rust type name.
    let expected = py.eval("type(obj).__name__");
    check(
        "class name",
        expected,
        Value::str("dac084S085"),
    );
    let _ = load_config_prefix;
}
