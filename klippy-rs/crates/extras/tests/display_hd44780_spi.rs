//! Parity tests for `extras::display::hd44780_spi` (port of
//! `klippy/extras/display/hd44780_spi.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a Python `hd44780_spi` is built
//! with minimal fakes for the printer, reactor, config and the `bus` SPI
//! helper (both sides fake the bus: on the Rust side through the
//! `crate::bus::McuSpiApi` trait that `Hd44780Spi::from_config` takes;
//! the faked `spi_send` calls are compared byte-for-byte and in order),
//! and a Rust `Hd44780Spi` is built with the same fakes in Rust. Driving
//! both through `init` / `flush` / `write_text` / `write_glyph` /
//! `set_glyphs` / `clear` / `write_graphics` and comparing the recorded
//! SPI bytes and minclocks covers the whole module surface: the config
//! options (`hd44780_protocol_init`, `line_length`), the 4-bit send
//! sequence, the framebuffer diffing and batching in `flush`, glyph
//! handling (`set_glyphs`, icon slots, `right_arrow`) and the config
//! error for an invalid `line_length`.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::bus::McuSpiApi;
use extras::display::hd44780_spi::{
    GlyphIcon, Hd44780Spi, Icon5x8, LINE_LENGTH_DEFAULT, LINE_LENGTH_OPTIONS,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper,
    LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, QuerySendOpts, SendOpts, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::ObjectRef;
use klippy_api::EventArg;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import sys
import types
# The Python side fakes the `bus` module too: `hd44780_spi` only needs
# `bus.MCU_SPI_from_config`.
extras_bus = types.ModuleType('extras.bus')
sys.modules['extras.bus'] = extras_bus
import extras.display.hd44780_spi as hd44780_spi

SPI_CALLS = []
OPTIONS = {}

class FakeReactor:
    def monotonic(self):
        return 100.0

class FakePrinter:
    config_error = configparser.Error
    def get_reactor(self):
        return FakeReactor()

class FakeMcu:
    def estimated_print_time(self, eventtime):
        return 100.0
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000)

class FakeSpi:
    def __init__(self, printer):
        self._mcu = FakeMcu()
    def spi_send(self, data, minclock=0):
        SPI_CALLS.append((bytes(data), minclock))
    def get_mcu(self):
        return self._mcu

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer=None):
        self._printer = printer if printer is not None else FakePrinter()
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'display'
    def get(self, option, default=None, note_valid=True):
        return OPTIONS.get(option, default)
    def getint(self, option, default=None, note_valid=True):
        if option not in OPTIONS:
            return default
        return int(OPTIONS[option])
    def getboolean(self, option, default=None, note_valid=True):
        if option not in OPTIONS:
            return default
        return OPTIONS[option] not in ('false', '0', 'no', 'off')
    def getchoice(self, option, choices, default=None, note_valid=True):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default, note_valid=note_valid)
        else:
            c = self.get(option, default, note_valid=note_valid)
        if c not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self.get_name()))
        return choices[c]

extras_bus.MCU_SPI_from_config = lambda config, mode, pin_option='cs_pin': FakeSpi(config.get_printer())

def set_options(**kwargs):
    OPTIONS.clear()
    OPTIONS.update(kwargs)

def make_lcd():
    SPI_CALLS.clear()
    return hd44780_spi.hd44780_spi(FakeConfig(FakePrinter()))

def spi_hex():
    def fmt(data, minclock):
        return ','.join('%02x' % b for b in data) + ':' + str(minclock)
    return '|'.join(fmt(d, m) for d, m in SPI_CALLS)

def clear_spi():
    SPI_CALLS.clear()
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
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<&str> {
        self.options.get(option).map(String::as_str)
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "display".to_string()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(raw) = self.raw(option) {
            return Ok(Some(raw.to_string()));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section 'display' is required but not set"
            ))),
            Some(inner) => Ok(inner),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        if let Some(raw) = self.raw(option) {
            let raw = raw.trim();
            return match raw.parse::<i64>() {
                Ok(n) => Ok(Some(n)),
                Err(_) => Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section 'display' cannot be parsed as int"
                ))),
            };
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section 'display' is required but not set"
            ))),
            Some(inner) => Ok(inner),
        }
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by hd44780_spi")
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        if let Some(raw) = self.raw(option) {
            return Ok(Some(!matches!(
                raw.to_lowercase().as_str(),
                "false" | "0" | "no" | "off"
            )));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section 'display' is required but not set"
            ))),
            Some(inner) => Ok(inner),
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        // Mirror `configfile.py:getchoice`: a list of ints becomes
        // `{i: i}` and is resolved in int mode; the absent option falls
        // back to `default`.
        let int_mode = match choices {
            PyValue::List(items) => items.iter().all(|v| matches!(v, PyValue::Int(_))),
            _ => false,
        };
        let candidates: Vec<PyValue> = match choices {
            PyValue::List(items) => items.clone(),
            _ => Vec::new(),
        };
        let key: PyValue = if let Some(raw) = self.raw(option) {
            if int_mode {
                match raw.trim().parse::<i64>() {
                    Ok(n) => PyValue::Int(PyInt::from_i64(n)),
                    Err(_) => {
                        return Err(klippy_api::KlippyError::config(format!(
                            "Option '{option}' in section 'display' cannot be parsed as int"
                        )))
                    }
                }
            } else {
                PyValue::Str(raw.to_string())
            }
        } else {
            match opts.default {
                None => {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section 'display' is required but not set"
                    )))
                }
                Some(Some(v)) => v,
                Some(None) => PyValue::None,
            }
        };
        let c = match &key {
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Str(s) => s.clone(),
            PyValue::None => "None".to_string(),
            other => other.str(),
        };
        for candidate in &candidates {
            if pycompat::value::py_equal(candidate, &key) {
                return Ok(candidate.clone());
            }
        }
        Err(klippy_api::KlippyError::config(format!(
            "Choice '{c}' for option '{option}' in section 'display' is not a valid choice"
        )))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by hd44780_spi")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by hd44780_spi")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by hd44780_spi")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by hd44780_spi")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by hd44780_spi")
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

struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "fake-mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by hd44780_spi")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by hd44780_spi")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by hd44780_spi")
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by hd44780_spi")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by hd44780_spi")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by hd44780_spi")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by hd44780_spi")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by hd44780_spi")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by hd44780_spi")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by hd44780_spi")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by hd44780_spi")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by hd44780_spi")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by hd44780_spi")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by hd44780_spi")
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by hd44780_spi")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by hd44780_spi")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("get_enumerations is not used by hd44780_spi")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("get_constants is not used by hd44780_spi")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by hd44780_spi")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        // `int(print_time * 1000)`, matching the Python fake.
        pycompat::text::trunc_float_to_bigint(print_time * 1000.0)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap()
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by hd44780_spi")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        100.0
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by hd44780_spi")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by hd44780_spi")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by hd44780_spi")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by hd44780_spi")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by hd44780_spi")
    }
}

struct FakeSpi {
    mcu: Mcu,
    calls: klippy_api::GCell<Vec<(Vec<u8>, i64)>>,
}

impl FakeSpi {
    fn new() -> FakeSpi {
        FakeSpi {
            mcu: Arc::new(FakeMcu),
            calls: klippy_api::GCell::new(Vec::new()),
        }
    }
    /// The same `spi_hex()` rendering as the Python fake.
    fn spi_hex(&self) -> String {
        let calls = self.calls.borrow();
        if calls.is_empty() {
            return String::new();
        }
        calls
            .iter()
            .map(|(data, minclock)| {
                let hex = data
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<Vec<_>>()
                    .join(",");
                format!("{hex}:{minclock}")
            })
            .collect::<Vec<_>>()
            .join("|")
    }
}

impl McuSpiApi for FakeSpi {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn setup_shutdown_msg(&self, _shutdown_seq: &[u8]) -> KResult<()> {
        Ok(())
    }
    fn spi_send(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.calls.borrow_mut().push((data.to_vec(), opts.minclock));
        Ok(())
    }
    fn spi_transfer(&self, _data: &[u8], _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn spi_transfer_with_preface(
        &self,
        _preface_data: &[u8],
        _data: &[u8],
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("hd44780_spi fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping hd44780_spi oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_options(options: &[(&str, &str)]) -> String {
    let parts: Vec<String> = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("set_options(**{{{}}})", parts.join(", "))
}

/// Builds the same `hd44780_spi` pair on both sides with the given raw
/// config options, and clears both SPI logs.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    options: &[(&str, &str)],
) -> (Hd44780Spi, Arc<FakeSpi>, Config) {
    py.exec(&py_options(options)).expect("py set_options");
    py.exec("lcd = make_lcd()").expect("py construction");
    let printer: Printer = Arc::new(FakePrinter);
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: options
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
    });
    let spi = Arc::new(FakeSpi::new());
    let spi_trait: Arc<dyn McuSpiApi> = spi.clone();
    let rust = Hd44780Spi::from_config(&config, spi_trait).expect("rust from_config");
    (rust, spi, config)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "hd44780_spi".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust returned success but call must fail"),
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
}

/// Compares the recorded SPI bytes + minclocks on both sides.
fn check_spi(label: &str, py: &mut pyoracle::Oracle, spi: &FakeSpi) {
    let expected = py.eval("spi_hex()");
    check(label, expected, Value::str(&spi.spi_hex()));
}

fn check_int(label: &str, expected: pyoracle::OracleResult, actual: KResult<i64>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, Ok(expected), Value::int(actual));
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check(
        "LINE_LENGTH_DEFAULT",
        py.eval("hd44780_spi.LINE_LENGTH_DEFAULT"),
        Value::int(LINE_LENGTH_DEFAULT),
    );
    check(
        "LINE_LENGTH_OPTIONS",
        py.eval("str(hd44780_spi.LINE_LENGTH_OPTIONS)"),
        Value::str(&format!("{:?}", LINE_LENGTH_OPTIONS)),
    );
    check(
        "TextGlyphs right_arrow",
        py.eval("hd44780_spi.TextGlyphs['right_arrow'][0]"),
        Value::int(b"\x7e"[0] as i64),
    );
}

#[test]
fn default_construction() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    check(
        "line_length",
        py.eval("lcd.line_length"),
        Value::int(rust.line_length()),
    );
    check(
        "hd44780_protocol_init",
        py.eval("lcd.hd44780_protocol_init"),
        Value::Bool(rust.hd44780_protocol_init()),
    );
    check(
        "get_dimensions",
        py.eval("str(lcd.get_dimensions())"),
        Value::str(&format!("{:?}", rust.get_dimensions())),
    );
    // Construction sends nothing.
    check_spi("no spi during construction", &mut py, &spi);
}

#[test]
fn line_length_16() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[("line_length", "16")]);
    check(
        "line_length",
        py.eval("lcd.line_length"),
        Value::int(rust.line_length()),
    );
    check(
        "get_dimensions",
        py.eval("str(lcd.get_dimensions())"),
        Value::str(&format!("{:?}", rust.get_dimensions())),
    );
    // The initial framebuffers are 2*16 bytes; the init sequence differs
    // in size from the 20-wide default.
    py.exec("lcd.init()").expect("py init");
    rust.init().expect("rust init");
    check_spi("init with 16-wide display", &mut py, &spi);
}

#[test]
fn invalid_line_length_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_options(line_length='17')")
        .expect("py set_options");
    let expected = py.exec("make_lcd()");
    let printer: Printer = Arc::new(FakePrinter);
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: IndexMap::from([("line_length".to_string(), "17".to_string())]),
    });
    let spi = Arc::new(FakeSpi::new());
    let spi_trait: Arc<dyn McuSpiApi> = spi.clone();
    check_err(
        "line_length=17",
        expected,
        Hd44780Spi::from_config(&config, spi_trait).map(|_| ()),
    );
}

#[test]
fn init_with_protocol() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    py.exec("lcd.init()").expect("py init");
    rust.init().expect("rust init");
    check_spi("init with protocol init", &mut py, &spi);
}

#[test]
fn init_without_protocol() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) =
        fresh_pair(&mut py, &[("hd44780_protocol_init", "false")]);
    check(
        "protocol_init read as false",
        py.eval("lcd.hd44780_protocol_init"),
        Value::Bool(rust.hd44780_protocol_init()),
    );
    py.exec("lcd.init()").expect("py init");
    rust.init().expect("rust init");
    check_spi("init without protocol init", &mut py, &spi);
}

#[test]
fn write_text_and_flush() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    py.exec("lcd.write_text(0, 0, b'HELLO')").expect("py write_text");
    rust.write_text(0, 0, b"HELLO").expect("rust write_text");
    py.exec("lcd.flush()").expect("py flush");
    rust.flush().expect("rust flush");
    check_spi("write_text + flush", &mut py, &spi);

    // Truncation past the row end and the (y & 2) row offset.
    py.exec("clear_spi()").expect("py clear_spi");
    spi.calls.borrow_mut().clear();
    py.exec("lcd.write_text(18, 0, b'XYZ')").expect("py write_text trunc");
    rust.write_text(18, 0, b"XYZ").expect("rust write_text trunc");
    py.exec("lcd.write_text(2, 2, b'ROW2')").expect("py write_text row2");
    rust.write_text(2, 2, b"ROW2").expect("rust write_text row2");
    py.exec("lcd.flush()").expect("py flush2");
    rust.flush().expect("rust flush2");
    check_spi("truncation + row offset", &mut py, &spi);
}

#[test]
fn flush_batching() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    // Diff gaps of 3 (merged), 4 (merged) and 15 (not merged) expose the
    // batching loop's merge window.
    py.exec(
        "lcd.init(); clear_spi(); lcd.write_text(0, 0, b'A'); \
         lcd.write_text(4, 0, b'B'); lcd.write_text(9, 0, b'C'); \
         lcd.write_text(0, 1, b'D'); lcd.flush()",
    )
    .expect("py batched");
    rust.init().expect("rust init");
    spi.calls.borrow_mut().clear();
    rust.write_text(0, 0, b"A").expect("rust A");
    rust.write_text(4, 0, b"B").expect("rust B");
    rust.write_text(9, 0, b"C").expect("rust C");
    rust.write_text(0, 1, b"D").expect("rust D");
    rust.flush().expect("rust flush");
    check_spi("batched flush", &mut py, &spi);

    // A second flush re-sends nothing.
    py.exec("clear_spi(); lcd.flush()").expect("py second flush");
    spi.calls.borrow_mut().clear();
    rust.flush().expect("rust second flush");
    check_spi("second flush is empty", &mut py, &spi);
}

#[test]
fn write_glyph_icon_and_text() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    py.exec(
        "lcd.set_glyphs({'my_icon': {'icon5x8': (3, bytes([0x1f, 0x11, 0x11, 0x1f, 0x00, 0x00, 0x00, 0x00]))}, \
         'no_icon': {'icon16x16': (bytes([0x00] * 16), bytes([0x00] * 16))}}); \
         lcd.write_text(0, 0, b'')",
    )
    .expect("py set_glyphs");
    let mut glyphs = IndexMap::new();
    glyphs.insert(
        "my_icon".to_string(),
        GlyphIcon {
            icon5x8: Some(Icon5x8 {
                slot: 3,
                bits: vec![0x1f, 0x11, 0x11, 0x1f, 0x00, 0x00, 0x00, 0x00],
            }),
        },
    );
    glyphs.insert("no_icon".to_string(), GlyphIcon { icon5x8: None });
    rust.set_glyphs(&glyphs);

    check_int(
        "write_glyph icon",
        py.eval("lcd.write_glyph(0, 0, 'my_icon')"),
        rust.write_glyph(0, 0, "my_icon"),
    );
    check_int(
        "write_glyph right_arrow",
        py.eval("lcd.write_glyph(1, 0, 'right_arrow')"),
        rust.write_glyph(1, 0, "right_arrow"),
    );
    check_int(
        "write_glyph unknown",
        py.eval("lcd.write_glyph(2, 0, 'missing')"),
        rust.write_glyph(2, 0, "missing"),
    );
    py.exec("lcd.flush()").expect("py flush after glyphs");
    rust.flush().expect("rust flush after glyphs");
    check_spi("glyph flush", &mut py, &spi);
}

#[test]
fn clear_resets_text_buffers() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    py.exec("lcd.write_text(0, 0, b'ABC'); lcd.clear(); lcd.flush()")
        .expect("py clear");
    rust.write_text(0, 0, b"ABC").expect("rust write_text");
    rust.clear();
    rust.flush().expect("rust flush");
    check_spi("clear + flush", &mut py, &spi);
}

#[test]
fn write_graphics_is_noop() {
    let Some(mut py) = oracle() else { return };
    let (rust, spi, _config) = fresh_pair(&mut py, &[]);
    py.exec("lcd.write_graphics(0, 0, [1, 2, 3]); lcd.flush()")
        .expect("py write_graphics");
    rust.write_graphics(0, 0, &[1, 2, 3]);
    rust.flush().expect("rust flush");
    check_spi("write_graphics sends nothing", &mut py, &spi);
}

#[test]
fn module_factory_has_no_entry_points() {
    let factory = extras::display::hd44780_spi::module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.display.hd44780_spi"), "entry points of extras.display.hd44780_spi");
}
