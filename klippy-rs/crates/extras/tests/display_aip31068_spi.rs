//! Parity tests for `extras::display::aip31068_spi` (port of
//! `klippy/extras/display/aip31068_spi.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `aip31068_spi` chip is
//! built on both sides with minimal Python fakes for the printer, config
//! and SPI bus, then fed identical `encode` / `send` / `init` / `flush` /
//! `write_text` / `set_glyphs` / `write_glyph` / `write_graphics` / `clear`
//! / `get_dimensions` calls and the SPI byte streams (plus error class and
//! message for `encode`) are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

use extras::bus::McuSpiApi;
use extras::display::aip31068_spi::{
    Aip31068Spi, Aip31068SpiApi, DISPLAY_INIT_CMNDS, CMND, FLG_CGRAM, FLG_DDRAM,
    FLG_DISPLAY, FLG_ENTERY_MODE, FLG_FUNCTION, FLG_SHIFT, FLG_WRITE_RAM,
    LINE_LENGTH_DEFAULT, LINE_LENGTH_OPTIONS,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandWrapper,
    CommandWrapperApi, CommandQueryWrapper, CommandQueryWrapperApi, ConfigCallback,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi,
    QuerySendOpts, SendOpts, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, configparser
fake_bus = types.ModuleType("extras.bus")
class FakeReactor:
    def monotonic(self):
        return 10.0
class FakePrinter:
    def get_reactor(self):
        return FakeReactor()
class FakeMcu:
    def estimated_print_time(self, ct):
        return 20.0
    def print_time_to_clock(self, pt):
        return int(pt * 1000)
class FakeSpi:
    def __init__(self, mcu):
        self._mcu = mcu
        self.log = []
    def spi_send(self, data, minclock=0, reqclock=0):
        self.log.append((list(data), minclock))
    def get_mcu(self):
        return self._mcu
mcu = FakeMcu()
spi = FakeSpi(mcu)
def _fake_from_config(config, mode, pin_option="cs_pin", default_speed=100000, share_type=None, cs_active_high=False):
    return spi
fake_bus.MCU_SPI_from_config = _fake_from_config
sys.modules["extras.bus"] = fake_bus
import extras.display.aip31068_spi as aip
class FakeConfig:
    def __init__(self, line_length=None):
        self._printer = FakePrinter()
        self._options = {}
        if line_length is not None:
            self._options["line_length"] = line_length
    def get_printer(self):
        return self._printer
    def get_name(self):
        return "display"
    def getchoice(self, option, choices, default=None):
        if option in self._options:
            c = self._options[option]
        else:
            if default is None:
                raise configparser.Error("Option '%s' in section 'display' must be specified" % (option,))
            c = default
        if c not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self.get_name()))
        return choices[c]
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    pins: GCell<Option<Arc<FakePins>>>,
    configfile: GCell<Option<Arc<FakeConfigFile>>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter { pins: GCell::new(None), configfile: GCell::new(None) }
    }
}

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
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found: Option<ObjectRef> = match name {
            "pins" => self
                .pins
                .borrow()
                .clone()
                .map(|p| p.clone() as ObjectRef),
            "configfile" => self
                .configfile
                .borrow()
                .clone()
                .map(|p| p.clone() as ObjectRef),
            _ => None,
        };
        match found {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::ReturnNone => Ok(None),
                LookupDefault::Raise => Err(klippy_api::error::KlippyError::Config(format!(
                    "Unable to lookup object '{name}'"
                ))),
            },
        }
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

struct FakeConfigFile;

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config not used")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("read_main_config not used")
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {}
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) {}
    fn remove_section(&self, _section: &str) {}
}

impl PrinterObject for FakeConfigFile {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
}

struct FakePins {
    mcu: Mcu,
}

impl FakePins {
    fn pin_params(&self, pin_desc: &str, opts: &LookupPinOpts) -> PinParams {
        PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type.clone(),
        }
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        let _ = opts;
        Ok(self.pin_params(pin_desc, &LookupPinOpts::default()))
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        Ok(self.pin_params(pin_desc, &opts))
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin not used")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(
        &self,
        _chip_name: &str,
    ) -> KResult<klippy_api::mcu::PinResolver> {
        Ok(Arc::new(FakeResolver))
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

impl PrinterObject for FakePins {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PinsApi);
}

struct FakeResolver;

impl klippy_api::mcu::PinResolverApi for FakeResolver {
    fn reserve_pin(&self, _pin: &str, _reserve_name: &str) -> KResult<()> {
        Ok(())
    }
    fn alias_pin(&self, _alias: &str, _pin: &str) -> KResult<()> {
        Ok(())
    }
    fn update_command(&self, cmd: &str) -> KResult<String> {
        Ok(cmd.to_string())
    }
}

struct FakeCmd {
    tag: i64,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        self.tag
    }
}

struct FakeQueryCmd;

impl CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, _data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

struct FakeAsync;

impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
    next_oid: GCell<i64>,
    next_cq: GCell<i64>,
}

impl FakeMcu {
    fn new(printer: Printer) -> Self {
        FakeMcu {
            printer,
            next_oid: GCell::new(1),
            next_cq: GCell::new(1),
        }
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin not used")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut n = self.next_oid.borrow_mut();
        let v = *n;
        *n += 1;
        Ok(v)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        kos::cconv::f64_to_i64(time * 1000.0)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCmd { tag: 0 }))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Ok(Arc::new(FakeQueryCmd))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        Some(Arc::new(FakeCmd { tag: 1 }))
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        let mut n = self.next_cq.borrow_mut();
        let v = *n;
        *n += 1;
        CommandQueue(v)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float not used")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        kos::cconv::f64_to_i64(print_time * 1000.0)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        20.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, eventtime))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch not used")
    }
}

struct FakeSpi {
    mcu: Mcu,
    log: GCell<Vec<(Vec<u8>, i64)>>,
}

impl FakeSpi {
    fn new(mcu: Mcu) -> Self {
        FakeSpi { mcu, log: GCell::new(Vec::new()) }
    }
    fn log_value(&self) -> Value {
        let entries: Vec<Value> = self
            .log
            .borrow()
            .iter()
            .map(|(data, mc)| {
                Value::Tuple(vec![
                    Value::List(data.iter().map(|&b| Value::Int(b as i64)).collect()),
                    Value::Int(*mc),
                ])
            })
            .collect();
        Value::List(entries)
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
        self.log.borrow_mut().push((data.to_vec(), opts.minclock));
        Ok(())
    }
    fn spi_transfer(&self, _data: &[u8], _opts: QuerySendOpts) -> KResult<PyValue> {
        panic!("spi_transfer not used")
    }
    fn spi_transfer_with_preface(
        &self,
        _preface_data: &[u8],
        _data: &[u8],
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("spi_transfer_with_preface not used")
    }
}

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
    line_length: Option<i64>,
}

impl FakeConfig {
    fn new(printer: Printer, line_length: Option<i64>) -> Self {
        let mut options = IndexMap::new();
        options.insert("latch_pin".to_string(), "PB0".to_string());
        FakeConfig { printer, options, line_length }
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
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::Config(format!(
                "Option '{option}' in section 'display' must be specified"
            ))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getint(&self, _option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match opts.default {
            None => Err(klippy_api::error::KlippyError::Config(
                "Option must be specified".to_string(),
            )),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        _option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        // Mirror `configfile.py:76` for the int-keyed `{16:16,20:20}` map.
        let selected: i64 = match self.line_length {
            Some(v) => v,
            None => match opts.default {
                None => {
                    return Err(klippy_api::error::KlippyError::Config(format!(
                        "Option 'line_length' in section 'display' must be specified"
                    )));
                }
                Some(None) => {
                    return Err(klippy_api::error::KlippyError::Config(
                        "Choice 'None' for option 'line_length' in section 'display' is not a valid choice"
                            .to_string(),
                    ));
                }
                Some(Some(v)) => match v {
                    PyValue::Int(n) => n.inner().to_string().parse::<i64>().unwrap_or(0),
                    _ => 0,
                },
            },
        };
        if let PyValue::Dict(map) = choices {
            for (k, v) in map.iter() {
                if let PyValue::Int(kn) = k {
                    if kn.inner().to_string().parse::<i64>().unwrap_or(i64::MAX) == selected {
                        return Ok(v.clone());
                    }
                }
            }
        }
        Err(klippy_api::error::KlippyError::Config(format!(
            "Choice '{selected}' for option 'line_length' in section 'display' is not a valid choice"
        )))
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
        panic!("getsection not used")
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
            py.exec(SETUP).expect("aip31068_spi fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping aip31068_spi oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

/// Builds a fresh chip on both sides with the given `line_length`
/// (`None` = default 20). Returns the Rust chip, its fake SPI (for log
/// comparison) and the Rust config.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    line_length: Option<i64>,
) -> (Aip31068Spi, Arc<FakeSpi>, Config) {
    let ll_expr = match line_length {
        Some(v) => format!("line_length={v}"),
        None => "line_length=None".to_string(),
    };
    py.exec(&format!(
        "spi = FakeSpi(mcu); config = FakeConfig({ll_expr}); obj = aip.aip31068_spi(config)"
    ))
    .expect("py construct aip31068_spi");

    let printer: Printer = Arc::new(FakePrinter::new());
    let mcu: Mcu = Arc::new(FakeMcu::new(printer.clone()));
    let fake_spi = Arc::new(FakeSpi::new(mcu.clone()));
    let spi: Arc<dyn McuSpiApi> = fake_spi.clone();
    let config: Config = Arc::new(FakeConfig::new(printer.clone(), line_length));
    let rust =
        Aip31068Spi::from_config_and_spi(&config, spi).expect("rust construct aip31068_spi");
    (rust, fake_spi, config)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "aip31068_spi".to_string(),
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
        Ok(_) => panic!("{label}: rust returned Ok but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

fn check_log(label: &str, py: &mut pyoracle::Oracle, fake_spi: &Arc<FakeSpi>) {
    let expected = py.eval("spi.log");
    check(label, expected, fake_spi.log_value());
}

fn clear_log(py: &mut pyoracle::Oracle, fake_spi: &Arc<FakeSpi>) {
    py.exec("spi.log.clear()").expect("clear py log");
    fake_spi.log.borrow_mut().clear();
}

fn bytes_list_py(data: &[u8]) -> String {
    let parts: Vec<String> = data.iter().map(|b| b.to_string()).collect();
    format!("[{}]", parts.join(", "))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check(
        "cmnd_clr",
        py.eval("aip.CMND.CLR"),
        Value::Int(CMND::CLR),
    );
    check(
        "display_init",
        py.eval("aip.DISPLAY_INIT_CMNDS"),
        Value::List(DISPLAY_INIT_CMNDS.iter().map(|&v| Value::Int(v)).collect()),
    );
    check(
        "line_length_default",
        py.eval("aip.LINE_LENGTH_DEFAULT"),
        Value::Int(LINE_LENGTH_DEFAULT),
    );
    check(
        "line_length_options",
        py.eval("aip.LINE_LENGTH_OPTIONS"),
        Value::Dict(vec![
            (
                Value::Int(LINE_LENGTH_OPTIONS[0].0),
                Value::Int(LINE_LENGTH_OPTIONS[0].1),
            ),
            (
                Value::Int(LINE_LENGTH_OPTIONS[1].0),
                Value::Int(LINE_LENGTH_OPTIONS[1].1),
            ),
        ]),
    );
    // Flag values spot-checked through one init command each.
    check(
        "flags",
        py.eval("[aip.flg_ENTERY_MODE.INC, aip.flg_DISPLAY.ON, aip.flg_SHIFT.RIGHT, aip.flg_FUNCTION.TWO_LINES, aip.flg_CGRAM.MASK, aip.flg_DDRAM.MASK, aip.flg_WRITE_RAM.MASK]"),
        Value::List(vec![
            Value::Int(FLG_ENTERY_MODE::INC),
            Value::Int(FLG_DISPLAY::ON),
            Value::Int(FLG_SHIFT::RIGHT),
            Value::Int(FLG_FUNCTION::TWO_LINES),
            Value::Int(FLG_CGRAM::MASK),
            Value::Int(FLG_DDRAM::MASK),
            Value::Int(FLG_WRITE_RAM::MASK),
        ]),
    );
}

#[test]
fn encode_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<Vec<i64>> = vec![
        vec![],
        vec![0],
        vec![1],
        vec![255],
        vec![256],
        vec![511],
        vec![0, 1, 511, 256, 128],
        vec![6, 6, 6, 6, 6, 6, 6, 6],
        vec![2, 6, 12, 20, 40, 1, 2, 3, 4, 5],
    ];
    for data in cases {
        let label = format!("encode {data:?}");
        let expr = format!("aip.aip31068_spi.encode({})", {
            let parts: Vec<String> = data.iter().map(|v| v.to_string()).collect();
            format!("[{}]", parts.join(", "))
        });
        let expected = py.eval(&expr);
        let actual = Aip31068Spi::encode(&data);
        match (expected, actual) {
            (Ok(expected_val), Ok(encoded)) => {
                check(&label, Ok(expected_val), Value::List(
                    encoded.iter().map(|&b| Value::Int(b as i64)).collect()
                ));
            }
            (Err(e), Err(rust)) => {
                assert_eq!(rust.class_name(), e.class_name.as_str(), "{label} class");
                assert_eq!(rust.message(), e.message.as_str(), "{label} message");
            }
            _ => panic!("{label}: oracle/rust outcome mismatch"),
        }
    }
    // Custom widths. `(vec![1, 2, 3, 4], 2)` overflows on both sides
    // (4 needs 3 bits); expect the matching ValueError.
    for (data, width) in [
        (vec![7, 7, 7], 4i64),
        (vec![255, 0, 127], 8i64),
        (vec![1, 2, 3, 4], 2i64),
        (vec![0b1010_1010], 8i64),
    ] {
        let label = format!("encode width {width} {data:?}");
        let expr = format!("aip.aip31068_spi.encode({}, {})", {
            let parts: Vec<String> = data.iter().map(|v| v.to_string()).collect();
            format!("[{}]", parts.join(", "))
        }, width);
        let expected = py.eval(&expr);
        let actual = Aip31068Spi::encode_in_width(&data, width);
        match (expected, actual) {
            (Ok(expected_val), Ok(encoded)) => {
                check(
                    &label,
                    Ok(expected_val),
                    Value::List(encoded.iter().map(|&b| Value::Int(b as i64)).collect()),
                );
            }
            (Err(e), Err(rust)) => {
                assert_eq!(rust.class_name(), e.class_name.as_str(), "{label} class");
                assert_eq!(rust.message(), e.message.as_str(), "{label} message");
            }
            _ => panic!("{label}: oracle/rust outcome mismatch"),
        }
    }
}

#[test]
fn encode_overflow_raises_value_error() {
    let Some(mut py) = oracle() else { return };
    for (data, width) in [
        (vec![512], 9i64),
        (vec![256], 8i64),
        (vec![1000], 9i64),
        (vec![767], 8i64),
        (vec![0, 0, 512], 9i64),
    ] {
        let label = format!("encode overflow {data:?} width {width}");
        let expr = format!("aip.aip31068_spi.encode({}, {})", {
            let parts: Vec<String> = data.iter().map(|v| v.to_string()).collect();
            format!("[{}]", parts.join(", "))
        }, width);
        let expected = py.eval(&expr);
        let actual = Aip31068Spi::encode_in_width(&data, width);
        check_err(&label, expected, actual.map(|v| v));
    }
}

#[test]
fn send_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, fake_spi, _cfg) = fresh_pair(&mut py, Some(20));
    for (label, data, minclock) in [
        ("single", vec![2], 0),
        ("eight", vec![2, 6, 12, 20, 40, 1, 2, 3], 0),
        ("nine", vec![0i64; 8].into_iter().chain([511i64].iter().copied()).collect::<Vec<i64>>(), 0),
        ("empty", vec![], 0),
        ("minclock", vec![6, 6], 12345),
        ("two_groups", vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 7),
    ] {
        clear_log(&mut py, &fake_spi);
        py.exec(&format!(
            "obj.send({}, {})",
            {
                let parts: Vec<String> = data.iter().map(|v| v.to_string()).collect();
                format!("[{}]", parts.join(", "))
            },
            minclock
        ))
        .expect("py send");
        rust.send(&data, minclock).expect("rust send");
        check_log(&format!("send {label}"), &mut py, &fake_spi);
    }
    // Overflow inside send propagates the ValueError and sends nothing
    // for the failing group.
    clear_log(&mut py, &fake_spi);
    let expected = py.eval("obj.send([512], 0)");
    let actual = rust.send(&[512], 0);
    check_err("send overflow", expected, actual);
    check_log("send overflow log", &mut py, &fake_spi);
}

#[test]
fn init_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, fake_spi, _cfg) = fresh_pair(&mut py, Some(20));
    py.exec("obj.init()").expect("py init");
    rust.init().expect("rust init");
    check_log("init", &mut py, &fake_spi);
}

#[test]
fn flush_initial_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, fake_spi, _cfg) = fresh_pair(&mut py, Some(20));
    py.exec("obj.flush()").expect("py flush");
    rust.flush().expect("rust flush");
    check_log("flush initial", &mut py, &fake_spi);
    // Second flush sends nothing (old buffers updated).
    clear_log(&mut py, &fake_spi);
    py.exec("obj.flush()").expect("py flush2");
    rust.flush().expect("rust flush2");
    check_log("flush no-diff", &mut py, &fake_spi);
}

#[test]
fn write_text_and_flush_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, fake_spi, _cfg) = fresh_pair(&mut py, Some(20));
    // Settle initial state.
    py.exec("obj.flush()").expect("py flush0");
    rust.flush().expect("rust flush0");
    clear_log(&mut py, &fake_spi);

    let writes: Vec<(i64, i64, Vec<u8>)> = vec![
        (0, 0, b"Hello".to_vec()),
        (5, 1, b"World".to_vec()),
        (0, 2, b"Second row".to_vec()),
        (10, 3, b"XYZ".to_vec()),
        (18, 0, b"TooLong".to_vec()),
        (25, 0, b"Out".to_vec()),
        (0, 0, vec![]),
        (19, 0, b"AB".to_vec()),
        (0, 0, b" ".to_vec()),
    ];
    for (x, y, data) in &writes {
        py.exec(&format!(
            "obj.write_text({}, {}, {})",
            x,
            y,
            bytes_list_py(data)
        ))
        .expect("py write_text");
        rust.write_text(*x, *y, data);
    }
    py.exec("obj.flush()").expect("py flush");
    rust.flush().expect("rust flush");
    check_log("write_text flush", &mut py, &fake_spi);
    clear_log(&mut py, &fake_spi);
    py.exec("obj.flush()").expect("py flush again");
    rust.flush().expect("rust flush again");
    check_log("write_text no-diff", &mut py, &fake_spi);
}

#[test]
fn set_glyphs_and_write_glyph_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, fake_spi, _cfg) = fresh_pair(&mut py, Some(20));
    py.exec("obj.flush()").expect("py flush0");
    rust.flush().expect("rust flush0");
    clear_log(&mut py, &fake_spi);

    // icon5x8 entry plus a glyph with no icon5x8 key (must be ignored).
    py.exec(
        "obj.set_glyphs({'myicon': {'icon5x8': (3, [1, 2, 3, 4, 5, 6, 7, 8])}, \
         'other': {'icon16x16': ([0], [0])}})",
    )
    .expect("py set_glyphs");
    let mut glyphs: IndexMap<String, IndexMap<String, (i64, Vec<u8>)>> = IndexMap::new();
    let mut inner: IndexMap<String, (i64, Vec<u8>)> = IndexMap::new();
    inner.insert("icon5x8".to_string(), (3, vec![1, 2, 3, 4, 5, 6, 7, 8]));
    glyphs.insert("myicon".to_string(), inner);
    let mut inner2: IndexMap<String, (i64, Vec<u8>)> = IndexMap::new();
    inner2.insert("icon16x16".to_string(), (0, vec![0]));
    glyphs.insert("other".to_string(), inner2);
    rust.set_glyphs(&glyphs);

    for (label, name) in [
        ("icon", "myicon"),
        ("text-right-arrow", "right_arrow"),
        ("missing", "nope"),
        ("wrong-kind", "other"),
    ] {
        let expected = py.eval(&format!("obj.write_glyph(0, 0, '{name}')"));
        let actual = rust.write_glyph(0, 0, name);
        match (expected, actual) {
            (Ok(expected_val), Ok(v)) => {
                check(&format!("write_glyph {label}"), Ok(expected_val), Value::Int(v));
            }
            (Err(e), Err(err)) => {
                assert_eq!(err.class_name(), e.class_name.as_str());
                assert_eq!(err.message(), e.message.as_str());
            }
            _ => panic!("write_glyph {label} outcome mismatch"),
        }
    }
    py.exec("obj.flush()").expect("py flush glyphs");
    rust.flush().expect("rust flush glyphs");
    check_log("write_glyph flush", &mut py, &fake_spi);
}

#[test]
fn write_graphics_and_clear_match_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, fake_spi, _cfg) = fresh_pair(&mut py, Some(20));
    py.exec("obj.flush()").expect("py flush0");
    rust.flush().expect("rust flush0");
    clear_log(&mut py, &fake_spi);

    // write_graphics is a no-op.
    py.exec("obj.write_graphics(0, 0, [1, 2, 3])").expect("py write_graphics");
    rust.write_graphics(0, 0, &[1, 2, 3]);
    py.exec("obj.flush()").expect("py flush gfx");
    rust.flush().expect("rust flush gfx");
    check_log("write_graphics no-op flush", &mut py, &fake_spi);

    // Write something, then clear and flush.
    py.exec("obj.write_text(0, 0, b'ABC')").expect("py write ABC");
    rust.write_text(0, 0, b"ABC");
    clear_log(&mut py, &fake_spi);
    py.exec("obj.clear()").expect("py clear");
    rust.clear();
    py.exec("obj.flush()").expect("py flush clear");
    rust.flush().expect("rust flush clear");
    check_log("clear flush", &mut py, &fake_spi);

    // Clear on already-blank then flush: nothing to send.
    clear_log(&mut py, &fake_spi);
    py.exec("obj.clear()").expect("py clear2");
    rust.clear();
    py.exec("obj.flush()").expect("py flush clear2");
    rust.flush().expect("rust flush clear2");
    check_log("clear empty flush", &mut py, &fake_spi);
}

#[test]
fn dimensions_and_line_lengths_match_python() {
    let Some(mut py) = oracle() else { return };
    for ll in [None, Some(16), Some(20)] {
        let (rust, fake_spi, _cfg) = fresh_pair(&mut py, ll);
        let expected = py.eval("obj.get_dimensions()");
        check(
            &format!("get_dimensions ll={ll:?}"),
            expected,
            Value::Tuple(vec![
                Value::Int(rust.get_dimensions().0),
                Value::Int(rust.get_dimensions().1),
            ]),
        );
        // Trait view agrees with the inherent method.
        let (a, b) = <Aip31068Spi as Aip31068SpiApi>::get_dimensions(&rust);
        assert_eq!((a, b), (rust.line_length(), 4));
        clear_log(&mut py, &fake_spi);
        py.exec("obj.flush()").expect("py flush dims");
        rust.flush().expect("rust flush dims");
        check_log(&format!("flush ll={ll:?}"), &mut py, &fake_spi);
    }
    // Invalid choice raises config error on both sides.
    let err = py.eval("aip.aip31068_spi(FakeConfig(line_length=17))");
    assert!(err.is_err(), "oracle must reject line_length 17");
    let printer: Printer = Arc::new(FakePrinter::new());
    let mcu: Mcu = Arc::new(FakeMcu::new(printer.clone()));
    let fake_spi: Arc<dyn McuSpiApi> = Arc::new(FakeSpi::new(mcu));
    let config: Config = Arc::new(FakeConfig::new(printer, Some(17)));
    let actual = Aip31068Spi::from_config_and_spi(&config, fake_spi);
    check_err("invalid line_length", err, actual.map(|_| ()));
}

#[test]
fn module_factory_builds_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::display::aip31068_spi::module_factory();
    assert!(factory.load_config.is_some(), "aip31068_spi exposes load_config");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());

    // Build through `from_config` with a registered pins object so the
    // local SPI helper can look up the latch pin. The SPI side effects
    // (oid + config commands) are expected to succeed.
    let printer = Arc::new(FakePrinter::new());
    let mcu: Mcu = Arc::new(FakeMcu::new(printer.clone()));
    let pins: Arc<FakePins> = Arc::new(FakePins { mcu: mcu.clone() });
    *printer.pins.borrow_mut() = Some(pins.clone());
    let configfile: Arc<FakeConfigFile> = Arc::new(FakeConfigFile);
    *printer.configfile.borrow_mut() = Some(configfile);
    let config: Config = Arc::new(FakeConfig::new(printer, Some(20)));
    let obj: ObjectRef = factory
        .load_config
        .expect("load_config")( &config)
        .expect("factory load_config")
        .expect("returns object");
    let rust: Arc<Aip31068Spi> = downcast_arc(&obj).expect("downcast to Aip31068Spi");
    assert_eq!(rust.get_dimensions(), (20, 4));

    // Python counterpart for the same config section shape.
    py.exec("spi = FakeSpi(mcu); config2 = FakeConfig(line_length=20); obj2 = aip.aip31068_spi(config2)")
        .expect("py construct");
    let expected = py.eval("obj2.get_dimensions()");
    check(
        "factory dimensions",
        expected,
        Value::Tuple(vec![Value::Int(20), Value::Int(4)]),
    );
}
