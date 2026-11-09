//! Parity tests for `extras::ad5206` (`klippy/extras/ad5206.py`).
//!
//! Expected values come from the live reference through `pyoracle`:
//! the frozen `klippy/extras/ad5206.py` is imported with a fake
//! `klippy.extras.bus` module (recording SPI handle) and a minimal
//! `ConfigWrapper`-compatible fake, then constructed with the same
//! option dict as the Rust port. SPI sends (`spi_send([reg, value])`
//! data), raised errors (class and message), the config reads
//! with their defaults and bounds, and direct `set_register` calls are
//! compared bit-for-bit through the recording value format.
//!
//! The module has no G-code commands, `get_status`, endpoints or log
//! lines; the config contract is `scale` plus `channel_1..channel_6`.

use extras::ad5206::Ad5206;
use extras::bus::McuSpiApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetIntListOpts,
    GetListsOpts, GetListOpts, GetFloatListOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper,
    CommandWrapper, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams,
    QuerySendOpts, SendOpts,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, StartArgs,
};
use pycompat::PyValue;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------
//
// `ad5206.py` does `from . import bus`, so a fake `klippy.extras.bus`
// module is installed in `sys.modules` before the real module is
// imported. `FakeConfig` reproduces `ConfigWrapper.getfloat`
// (`klippy/configfile.py:68`) including its error texts; errors use a
// local `Error` class matching `configparser.Error`'s name so the
// oracle reports class `Error` like `KlippyError::Config`.

const SETUP: &str = r#"
import sys, types

class Error(Exception):
    pass

class FakeSpi:
    def __init__(self):
        self.sends = []
    def spi_send(self, data, minclock=0, reqclock=0):
        assert len(data) == 2, data
        self.sends.append([int(data[0]), int(data[1])])

_bus_calls = []
_fake_spi = [None]

def _fake_spi_from_config(config, mode, pin_option='cs_pin', default_speed=100000, share_type=None, cs_active_high=False):
    _bus_calls.append((mode, pin_option, default_speed, share_type, cs_active_high))
    _fake_spi[0] = FakeSpi()
    return _fake_spi[0]

_fake_bus = types.ModuleType('extras.bus')
_fake_bus.MCU_SPI_from_config = _fake_spi_from_config
sys.modules['extras.bus'] = _fake_bus

import extras.ad5206 as _ad5206
ad5206_mod = _ad5206

_sentinel = object()

class FakeConfig:
    def __init__(self, section, values):
        self._section = section
        self._values = dict(values)
        self.reads = []
    def get_name(self):
        return self._section
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        self.reads.append((option, default, minval, maxval, above, below, note_valid))
        if option not in self._values:
            if default is not _sentinel:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._section))
        v = float(self._values[option])
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (option, self._section, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (option, self._section, below))
        return v

def new_obj(values):
    del _bus_calls[:]
    cfg = FakeConfig('ad5206', values)
    obj = ad5206_mod.ad5206(cfg)
    spi = _fake_spi[0]
    return (spi.sends, cfg.reads, list(_bus_calls))

def new_pair(values):
    del _bus_calls[:]
    cfg = FakeConfig('ad5206', values)
    obj = ad5206_mod.ad5206(cfg)
    return (obj, _fake_spi[0].sends)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakeMcu {
    name: String,
    printer: std::sync::Mutex<Option<Printer>>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String { self.name.clone() }
    fn get_printer(&self) -> Printer { self.printer.lock().unwrap().clone().expect("printer set") }
    fn is_fileoutput(&self) -> bool { false }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin not used".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> { Ok(1) }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> { Ok(()) }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> { Ok(()) }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, time: f64) -> i64 { (time * 1000000.0) as i64 }
    fn min_schedule_time(&self) -> f64 { 0.1 }
    fn max_nominal_duration(&self) -> f64 { 3.0 }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Err(KlippyError::Mcu("lookup_command not used".to_string()))
    }
    fn lookup_query_command(&self, _msgformat: &str, _respformat: &str, _opts: LookupQueryCommandOpts) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu("lookup_query_command not used".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> { None }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(7) }
    fn register_serial_response(&self, _cb: klippy_api::mcu::SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> AsyncResponseWrapper {
        struct FakeAsync;
        impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
            fn unregister(&self) {}
        }
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool { true }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { IndexMap::new() }
    fn get_constants(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> { Ok(0.0) }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 { 0 }
    fn clock_to_print_time(&self, clock: f64) -> f64 { clock }
    fn estimated_print_time(&self, eventtime: f64) -> f64 { eventtime }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { clock32 }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> { Ok((print_time, 0.0)) }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::None }
    fn stats(&self, _eventtime: f64) -> (bool, String) { (false, String::new()) }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
}

#[derive(Clone)]
struct FakeSpi {
    sends: Arc<Mutex<Vec<(i64, i64)>>>,
    oid: i64,
    mcu: Mcu,
}

impl FakeSpi {
    fn new(printer: Printer) -> (Self, Arc<Mutex<Vec<(i64, i64)>>>) {
        let sends = Arc::new(Mutex::new(Vec::new()));
        let mcu: Mcu = Arc::new(FakeMcu { name: "mcu".to_string(), printer: std::sync::Mutex::new(Some(printer)) });
        (FakeSpi { sends: sends.clone(), oid: 1, mcu }, sends)
    }
}

impl McuSpiApi for FakeSpi {
    fn get_oid(&self) -> i64 { self.oid }
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn get_command_queue(&self) -> CommandQueue { CommandQueue(7) }
    fn build_config(&self) -> KResult<()> { Ok(()) }
    fn setup_shutdown_msg(&self, _shutdown_seq: &[u8]) -> KResult<()> { Ok(()) }
    fn spi_send(&self, data: &[u8], _opts: SendOpts) -> KResult<()> {
        assert_eq!(data.len(), 2, "spi data {data:?}");
        self.sends.lock().unwrap().push((data[0] as i64, data[1] as i64));
        Ok(())
    }
    fn spi_transfer(&self, _data: &[u8], _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn spi_transfer_with_preface(&self, _preface_data: &[u8], _data: &[u8], _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

/// One recorded `getfloat` call: `(option, default, minval, maxval,
/// above, below, note_valid)` as the Python fake records it.
#[derive(Debug, Clone, PartialEq)]
struct ReadRec {
    option: String,
    default: Option<Option<f64>>,
    minval: Option<f64>,
    maxval: Option<f64>,
    above: Option<f64>,
    below: Option<f64>,
    note_valid: bool,
}

struct FakeConfig {
    section: String,
    printer: Printer,
    values: IndexMap<String, f64>,
    reads: Mutex<Vec<ReadRec>>,
}

impl FakeConfig {
    fn new(section: &str, values: IndexMap<String, f64>) -> Arc<Self> {
        Arc::new(FakeConfig {
            section: section.to_string(),
            printer: Arc::new(FakePrinter),
            values,
            reads: Mutex::new(Vec::new()),
        })
    }
    fn reads(&self) -> Vec<ReadRec> {
        self.reads.lock().unwrap().clone()
    }
}

fn float_opt(v: Option<f64>) -> Value {
    match v {
        Some(f) => Value::float(f),
        None => Value::Null,
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
        self.reads.lock().unwrap().push(ReadRec {
            option: option.to_string(),
            default: opts.default,
            minval: opts.minval,
            maxval: opts.maxval,
            above: opts.above,
            below: opts.below,
            note_valid: opts.note_valid,
        });
        match self.values.get(option) {
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
                Some(inner) => Ok(inner),
            },
            Some(&v) => {
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            pycompat::text::float_repr(minval)
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            pycompat::text::float_repr(maxval)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            pycompat::text::float_repr(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            pycompat::text::float_repr(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("unused in ad5206")
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

/// Minimal printer stub: ad5206 only needs one to satisfy `ConfigApi`.
struct FakePrinter;

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> reactor::Reactor {
        reactor::Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, _name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match default {
            LookupDefault::Raise => Err(KlippyError::config("missing object")),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
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
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

fn _unused(_: StartArgs) {}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("ad5206 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping ad5206 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.ad5206".to_string(),
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

fn check_exc(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?}, rust raised {}: {}", actual.class_name(), actual.message()),
        Err(e) => {
            assert_eq!(actual.class_name(), e.class_name, "{label}: class");
            assert_eq!(actual.message(), e.message, "{label}: message");
        }
    }
}

fn sends_value(sends: &[(i64, i64)]) -> Value {
    Value::List(
        sends
            .iter()
            .map(|(r, v)| Value::List(vec![Value::int(*r), Value::int(*v)]))
            .collect(),
    )
}

fn reads_value(reads: &[ReadRec]) -> Value {
    Value::List(
        reads
            .iter()
            .map(|r| {
                let default = match r.default {
                    None => Value::Str("sentinel".to_string()),
                    Some(inner) => match inner {
                        Some(f) => Value::float(f),
                        None => Value::Null,
                    },
                };
                Value::Tuple(vec![
                    Value::str(&r.option),
                    default,
                    float_opt(r.minval),
                    float_opt(r.maxval),
                    float_opt(r.above),
                    float_opt(r.below),
                    Value::Bool(r.note_valid),
                ])
            })
            .collect(),
    )
}

fn py_float_lit(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "float('inf')" } else { "float('-inf')" }.to_string()
    } else {
        format!("{v:?}")
    }
}

fn py_values_expr(values: &IndexMap<String, f64>) -> String {
    let mut parts = Vec::new();
    for (k, v) in values {
        parts.push(format!("{k:?}: {}", py_float_lit(*v)));
    }
    format!("{{{}}}", parts.join(", "))
}

fn rust_new(
    values: &IndexMap<String, f64>,
) -> (KResult<Arc<Ad5206>>, Arc<Mutex<Vec<(i64, i64)>>>, Vec<ReadRec>) {
    let printer: Printer = Arc::new(FakePrinter);
    let (spi, sends) = FakeSpi::new(printer.clone());
    let fake_cfg = FakeConfig::new("ad5206", values.clone());
    let cfg: Config = fake_cfg.clone();
    let reads_snapshot;
    let result = Ad5206::new(&cfg, Arc::new(spi));
    reads_snapshot = fake_cfg.reads();
    (result, sends, reads_snapshot)
}

fn py_new(
    py: &mut pyoracle::Oracle,
    values: &IndexMap<String, f64>,
) -> pyoracle::OracleResult {
    py.eval(&format!("new_obj({})", py_values_expr(values)))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

fn case(label: &str, values: IndexMap<String, f64>) {
    let Some(mut py) = oracle() else { return };
    let (result, sends, reads) = rust_new(&values);
    match result {
        Ok(_) => {
            let expected = py_new(&mut py, &values);
            let expected = match expected {
                Ok(v) => v,
                Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
            };
            // new_obj returns (sends, reads, bus_calls); compare each part.
            let (exp_sends, exp_reads, exp_bus) = match expected {
                Value::Tuple(t) if t.len() == 3 => (t[0].clone(), t[1].clone(), t[2].clone()),
                other => panic!("{label}: unexpected oracle shape {other:?}"),
            };
            check(label, Ok(exp_sends), sends_value(&sends.lock().unwrap()));
            check(&format!("{label}: reads"), Ok(exp_reads), reads_value(&reads));
            // SPI construction args: mode 0, enable_pin, 25 MHz, no share/csh.
            check(
                &format!("{label}: bus"),
                Ok(exp_bus),
                Value::List(vec![Value::Tuple(vec![
                    Value::int(0),
                    Value::str("enable_pin"),
                    Value::int(25000000),
                    Value::Null,
                    Value::Bool(false),
                ])]),
            );
        }
        Err(e) => {
            let expected = py_new(&mut py, &values);
            check_exc(label, expected, &e);
        }
    }
}

#[test]
fn defaults_sends_nothing() {
    case("defaults", IndexMap::new());
}

#[test]
fn single_channel_default_scale() {
    let mut v = IndexMap::new();
    v.insert("channel_1".to_string(), 0.5);
    case("single", v);
}

#[test]
fn all_channels() {
    let mut v = IndexMap::new();
    v.insert("scale".to_string(), 10.0);
    // 9.9 keeps the computed value byte-representable (below the 256
    // overflow threshold); the 256 formula itself is asserted below
    // via channel_value without touching the SPI byte boundary.
    for (i, val) in [0.0, 1.0, 2.5, 5.0, 7.25, 9.9].into_iter().enumerate() {
        v.insert(format!("channel_{}", i + 1), val);
    }
    case("all_channels", v);
}

#[test]
fn boundary_values() {
    let mut v = IndexMap::new();
    v.insert("scale".to_string(), 2.5);
    v.insert("channel_1".to_string(), 0.0);
    v.insert("channel_2".to_string(), 1.25);
    v.insert("channel_3".to_string(), 2.4);
    case("boundary", v);
}

#[test]
fn rounding_half() {
    // int(0.502 * 256 + 0.5) exercises the +.5-then-truncate pattern;
    // values are compared against the oracle, not hand-computed here.
    let mut v = IndexMap::new();
    for (i, val) in [0.001, 0.003, 0.007, 0.25, 0.75, 0.99].into_iter().enumerate() {
        v.insert(format!("channel_{}", i + 1), val);
    }
    case("rounding", v);
}

#[test]
fn scale_below_or_equal_zero_errors() {
    let mut v = IndexMap::new();
    v.insert("scale".to_string(), 0.0);
    case("scale_zero", v);
}

#[test]
fn scale_negative_errors() {
    let mut v = IndexMap::new();
    v.insert("scale".to_string(), -1.0);
    case("scale_negative", v);
}

#[test]
fn channel_above_scale_errors() {
    let mut v = IndexMap::new();
    v.insert("scale".to_string(), 1.0);
    v.insert("channel_2".to_string(), 1.5);
    case("channel_above", v);
}

#[test]
fn channel_below_zero_errors() {
    let mut v = IndexMap::new();
    v.insert("channel_1".to_string(), -0.5);
    case("channel_below", v);
}

#[test]
fn nan_channel_errors() {
    let mut v = IndexMap::new();
    v.insert("channel_1".to_string(), f64::NAN);
    // nan passes minval/maxval checks (comparisons false) and reaches int().
    case("nan_channel", v);
}

#[test]
fn set_register_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("pair = new_pair({'channel_1': 0.5})").expect("py seed");
    let mut v = IndexMap::new();
    v.insert("channel_1".to_string(), 0.5);
    let (result, sends, _) = rust_new(&v);
    let obj = result.expect("rust seed");
    obj.set_register(3, 100).expect("rust set_register");
    let expected = py.eval("(pair[0].set_register(3, 100), pair[1])[1]");
    check("set_register", expected, sends_value(&sends.lock().unwrap()));
}

#[test]
fn channel_value_formula_matches_python() {
    let Some(mut py) = oracle() else { return };
    for (val, scale) in [(0.0, 1.0), (0.5, 1.0), (1.0, 1.0), (2.5, 10.0), (1.0, 3.0), (7.0, 7.0)] {
        let expected = py.eval(&format!("int(({val:?} * 256. / {scale:?} + .5))"));
        let actual = Ad5206::channel_value(val, scale).expect("formula");
        check(&format!("channel_value({val}, {scale})"), expected, Value::int(actual));
    }
}

#[test]
fn module_factory_shape() {
    let f = extras::ad5206::module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_some());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("hasattr(ad5206_mod, 'load_config') , hasattr(ad5206_mod, 'load_config_prefix')");
    check("factory_shape", expected, Value::Tuple(vec![Value::Bool(false), Value::Bool(true)]));
}
