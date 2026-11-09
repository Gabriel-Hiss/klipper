//! Parity tests for `extras::static_digital_output` (port of `klippy/extras/static_digital_output.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterStaticDigitalOut`
//! is built on both sides with minimal Python / Rust fakes for the printer,
//! the config wrapper, the `pins` object and the MCU, then fed identical
//! pin tables and pin lists. The recorded MCU `add_config_cmd` strings
//! (format strings and argument values), the config options read, and the
//! raised errors (class and message) are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so expected values come from the oracle alone.

use extras::static_digital_output::PrinterStaticDigitalOut;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, Pin,
    PinParams, PinsApi, ParsePinOpts, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.static_digital_output as sdo_mod

class error(Exception):
    pass

_sentinel = object()

class FakeMcu:
    def __init__(self, name):
        self.name = name
        self.cmds = []
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.cmds.append((cmd, is_init, on_restart))

class FakePins:
    def __init__(self, table, mcus):
        self.table = table
        self.mcus = mcus
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        if pin_desc not in self.table:
            raise error("Unknown pin '%s'" % (pin_desc,))
        chip_name, pin, invert = self.table[pin_desc]
        return {'chip': self.mcus[chip_name], 'chip_name': chip_name, 'pin': pin, 'invert': invert}

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, objects):
        self.objects = objects
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, pins_list, section):
        self._printer = printer
        self._pins_list = pins_list
        self._section = section
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._section
    def getlist(self, option, default=_sentinel, sep=',', count=None, note_valid=True):
        if option != 'pins':
            raise AssertionError('unexpected getlist option %r' % (option,))
        if self._pins_list is None:
            raise self.error("Option '%s' in section '%s' must be specified" % (option, self._section))
        return list(self._pins_list)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter {
    pins: Option<ObjectRef>,
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
        let found = match name {
            "pins" => self.pins.clone(),
            _ => None,
        };
        match found {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
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
        panic!("load_object is not used by static_digital_output")
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

struct FakeMcuData {
    cmds: Mutex<Vec<(String, bool, bool)>>,
}

struct FakeMcu {
    data: Arc<FakeMcuData>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by static_digital_output test MCU")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by static_digital_output test MCU")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by static_digital_output test MCU")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by static_digital_output test MCU")
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.data
            .cmds
            .lock()
            .unwrap()
            .push((cmd.to_string(), opts.is_init, opts.on_restart));
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by test MCU")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by test MCU")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by test MCU")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by test MCU")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by test MCU")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by test MCU")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by test MCU")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by test MCU")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by test MCU")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by test MCU")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by test MCU")
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        panic!("get_enumerations is not used by test MCU")
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        panic!("get_constants is not used by test MCU")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by test MCU")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by test MCU")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by test MCU")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by test MCU")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by test MCU")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by test MCU")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by test MCU")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by test MCU")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by test MCU")
    }
}

struct FakePins {
    table: Vec<(String, PinParams)>,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by static_digital_output test pins")
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        for (key, params) in &self.table {
            if key == pin_desc {
                return Ok(params.clone());
            }
        }
        Err(KlippyError::Pins(format!("Unknown pin '{pin_desc}'")))
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by test pins")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { panic!("reset_pin_sharing is not used by test pins") }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by test pins")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used by test pins")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by test pins")
    }
}

struct FakeConfig {
    printer: Printer,
    pins: Option<Vec<String>>,
    section: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("get is not used by static_digital_output")
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by static_digital_output")
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by static_digital_output")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by static_digital_output")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("getchoice is not used by static_digital_output")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by static_digital_output")
    }
    fn getlist(&self, option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        assert_eq!(option, "pins", "static_digital_output reads only 'pins'");
        match &self.pins {
            Some(list) => Ok(Some(list.clone())),
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
        }
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by static_digital_output")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by static_digital_output")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by static_digital_output")
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
            py.exec(SETUP).expect("static_digital_output fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping static_digital_output oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

/// One pin-table row: pin descriptor as written in the config, chip name,
/// pin name and invert flag, mirroring `pins.lookup_pin` output.
struct Row {
    desc: &'static str,
    chip: &'static str,
    pin: &'static str,
    invert: i64,
}

fn cmds_list_value(cmds: &[(String, bool, bool)]) -> Value {
    Value::List(
        cmds.iter()
            .map(|(cmd, is_init, on_restart)| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    Value::Bool(*is_init),
                    Value::Bool(*on_restart),
                ])
            })
            .collect(),
    )
}


/// Builds the same case on the Python side: fresh MCUs, pins table, printer,
/// config; attempts `load_config_prefix` and returns the per-chip command
/// lists (or the oracle exception) plus the Rust-equivalent MCUs' command
/// buffers are returned separately by the caller building the Rust case.
fn build_py_case(
    py: &mut pyoracle::Oracle,
    chip_names: &[&str],
    rows: &[Row],
    pins_list: Option<&[&str]>,
    section: &str,
    has_pins_object: bool,
) -> pyoracle::OracleResult {
    let mut table_entries = String::new();
    for r in rows {
        table_entries.push_str(&format!(
            "\n    {:?}: ({:?}, {:?}, {}),",
            r.desc, r.chip, r.pin, r.invert
        ));
    }
    let chips = chip_names
        .iter()
        .map(|c| format!("{:?}: FakeMcu({:?})", c, c))
        .collect::<Vec<_>>()
        .join(", ");
    let pins_list_py = match pins_list {
        Some(list) => format!(
            "[{}]",
            list.iter().map(|p| format!("{:?}", p)).collect::<Vec<_>>().join(", ")
        ),
        None => "None".to_string(),
    };
    py.exec(&format!(
        "mcus = {{{chips}}}\n\
         table = {{{table_entries}\n}}\n\
         pins_obj = FakePins(table, mcus)\n\
         objects = {{}}\n\
         if {has}: objects['pins'] = pins_obj\n\
         printer = FakePrinter(objects)\n\
         cfg = FakeConfig(printer, {pins_py}, {section:?})\n",
        chips = chips,
        table_entries = table_entries,
        has = if has_pins_object { "True" } else { "False" },
        pins_py = pins_list_py,
        section = section
    ))
    .expect("py case setup");
    let res = py.eval("sdo_mod.load_config_prefix(cfg)");
    // Side effects are read back through `mcus` even when construction failed.
    match res {
        Err(e) => Err(e),
        Ok(_) => {
            // Single-chip cases compare in config order (the reference iterates
            // the pin list in order). Multi-chip cases compare the sorted set,
            // since inter-chip ordering follows dict iteration.
            if chip_names.len() == 1 {
                py.eval("[c for m in mcus.values() for c in m.cmds]")
            } else {
                py.eval("sorted([c for m in mcus.values() for c in m.cmds], key=lambda t: t[0])")
            }
        }
    }
}

/// Builds the same case on the Rust side and returns the per-chip commands
/// in config order (order across chips is the config-list order, matching
/// the Python iteration order).
fn build_rust_case(
    chip_names: &[&str],
    rows: &[Row],
    pins_list: Option<&[&str]>,
    section: &str,
    has_pins_object: bool,
) -> (KResult<()>, Vec<(String, bool, bool)>) {
    let mut datas: Vec<(String, Arc<FakeMcuData>)> = Vec::new();
    let mut mcus: Vec<(String, Mcu)> = Vec::new();
    for chip in chip_names {
        let data = Arc::new(FakeMcuData {
            cmds: Mutex::new(Vec::new()),
        });
        let mcu: Mcu = Arc::new(FakeMcu { data: data.clone() });
        datas.push(((*chip).to_string(), data));
        mcus.push(((*chip).to_string(), mcu));
    }
    let table: Vec<(String, PinParams)> = rows
        .iter()
        .map(|r| {
            let chip = mcus.iter().find(|(n, _)| n == r.chip).map(|(_, m)| m.clone()).expect("chip name in test rows");
            (
                r.desc.to_string(),
                PinParams {
                    chip,
                    chip_name: r.chip.to_string(),
                    pin: r.pin.to_string(),
                    invert: r.invert,
                    pullup: 0,
                    share_type: None,
                },
            )
        })
        .collect();
    let pins_obj: ObjectRef = Arc::new(FakePins { table });
    let printer: Printer = Arc::new(FakePrinter {
        pins: if has_pins_object { Some(pins_obj) } else { None },
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        pins: pins_list.map(|list| list.iter().map(|s| s.to_string()).collect()),
        section: section.to_string(),
    });
    let res = PrinterStaticDigitalOut::new(&config).map(|_| ());
    let all: Vec<(String, bool, bool)> = datas
        .iter()
        .flat_map(|(_, d)| d.cmds.lock().unwrap().clone())
        .collect();
    (res, all)
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust returned Ok but call must fail"),
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

fn sorted_cmds(cmds: &[(String, bool, bool)]) -> Vec<(String, bool, bool)> {
    let mut v = cmds.to_vec();
    v.sort_by(|a, b| a.0.cmp(&b.0));
    v
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn single_normal_pin() {
    let Some(mut py) = oracle() else { return };
    let rows = [Row { desc: "mcu:PE3", chip: "mcu", pin: "PE3", invert: 0 }];
    let list = ["mcu:PE3"];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", true);

    let (actual, cmds) = build_rust_case(&["mcu"], &rows, Some(&list), "static_digital_output main", true);
    let actual_cmds = sorted_cmds(&cmds);
    assert_cmds_equal("single_normal_pin", expected, actual, actual_cmds);
}

#[test]
fn single_inverted_pin() {
    let Some(mut py) = oracle() else { return };
    // lookup_pin resolves "!mcu:PE4" to pin PE4 with invert=1; the module
    // writes value = not invert = 0.
    let rows = [Row { desc: "!mcu:PE4", chip: "mcu", pin: "PE4", invert: 1 }];
    let list = ["!mcu:PE4"];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", true);
    let (actual, cmds) = build_rust_case(&["mcu"], &rows, Some(&list), "static_digital_output main", true);
    assert_cmds_equal("single_inverted_pin", expected, actual, cmds);
}

#[test]
fn many_pins_in_config_order() {
    let Some(mut py) = oracle() else { return };
    let rows = [
        Row { desc: "mcu:PE3", chip: "mcu", pin: "PE3", invert: 0 },
        Row { desc: "!mcu:PE4", chip: "mcu", pin: "PE4", invert: 1 },
        Row { desc: "mcu:PC1", chip: "mcu", pin: "PC1", invert: 0 },
        Row { desc: "mcu:PD0", chip: "mcu", pin: "PD0", invert: 0 },
    ];
    let list = ["mcu:PE3", "!mcu:PE4", "mcu:PC1", "mcu:PD0"];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", true);

    let (actual, cmds) = build_rust_case(&["mcu"], &rows, Some(&list), "static_digital_output main", true);
    assert_cmds_equal("many_pins_in_config_order", expected, actual, cmds);
}

#[test]
fn two_mcUs() {
    let Some(mut py) = oracle() else { return };
    let rows = [
        Row { desc: "mcu:PE3", chip: "mcu", pin: "PE3", invert: 0 },
        Row { desc: "aux:PE3", chip: "aux", pin: "PE3", invert: 1 },
    ];
    let list = ["mcu:PE3", "aux:PE3"];
    let expected = build_py_case(&mut py, &["mcu", "aux"], &rows, Some(&list), "static_digital_output main", true);
    let (actual, cmds) = build_rust_case(&["mcu", "aux"], &rows, Some(&list), "static_digital_output main", true);
    let actual_cmds = sorted_cmds(&cmds);
    assert_cmds_equal("two_mcUs", expected, actual, actual_cmds);
}

#[test]
fn empty_pin_list() {
    let Some(mut py) = oracle() else { return };
    let rows: [Row; 0] = [];
    let list: [&str; 0] = [];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", true);
    let (actual, cmds) = build_rust_case(&["mcu"], &rows, Some(&list), "static_digital_output main", true);
    assert_cmds_equal("empty_pin_list", expected, actual, cmds);
}

#[test]
fn missing_pins_option_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let rows: [Row; 0] = [];
    let expected = build_py_case(&mut py, &["mcu"], &rows, None, "static_digital_output main", true);
    let (actual, _cmds) = build_rust_case(&["mcu"], &rows, None, "static_digital_output main", true);
    check_err("missing_pins_option_raises_config_error", expected, actual);
}

#[test]
fn missing_pins_object_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let rows: [Row; 0] = [];
    let list = ["mcu:PE3"];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", false);
    let (actual, _cmds) = build_rust_case(&["mcu"], &rows, Some(&list), "static_digital_output main", false);
    check_err("missing_pins_object_raises_config_error", expected, actual);
}

#[test]
fn unknown_pin_propagates_pins_error() {
    let Some(mut py) = oracle() else { return };
    let rows = [Row { desc: "mcu:PE3", chip: "mcu", pin: "PE3", invert: 0 }];
    let list = ["mcu:PE3", "mcu:ZZ9"];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", true);
    let (actual, cmds) = build_rust_case(&["mcu"], &rows, Some(&list), "static_digital_output main", true);
    check_err("unknown_pin_propagates_pins_error", expected, actual);
    // The first pin's command was already queued before the failure, on both sides.
    assert_eq!(cmds, vec![("set_digital_out pin=PE3 value=1".to_string(), false, false)]);
}

#[test]
fn load_config_prefix_factory_builds_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::static_digital_output::module_factory();
    assert!(
        factory.load_config.is_none(),
        "static_digital_output has no load_config"
    );
    assert!(
        factory.load_config_prefix.is_some(),
        "static_digital_output exposes load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "static_digital_output has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "static_digital_output has no add_printer_objects"
    );
    let load_config_prefix = factory.load_config_prefix.expect("load_config_prefix");
    let rows = [
        Row { desc: "mcu:PE3", chip: "mcu", pin: "PE3", invert: 0 },
        Row { desc: "!mcu:PE4", chip: "mcu", pin: "PE4", invert: 1 },
    ];
    let list = ["mcu:PE3", "!mcu:PE4"];
    let expected = build_py_case(&mut py, &["mcu"], &rows, Some(&list), "static_digital_output main", true);
    // Rebuild a fresh Rust case for the factory.
    let mut datas: Vec<(String, Arc<FakeMcuData>)> = Vec::new();
    let data = Arc::new(FakeMcuData { cmds: Mutex::new(Vec::new()) });
    let mcu: Mcu = Arc::new(FakeMcu { data: data.clone() });
    datas.push(("mcu".to_string(), data));
    let table: Vec<(String, PinParams)> = rows
        .iter()
        .map(|r| {
            (
                r.desc.to_string(),
                PinParams {
                    chip: mcu.clone(),
                    chip_name: r.chip.to_string(),
                    pin: r.pin.to_string(),
                    invert: r.invert,
                    pullup: 0,
                    share_type: None,
                },
            )
        })
        .collect();
    let pins_obj: ObjectRef = Arc::new(FakePins { table });
    let printer: Printer = Arc::new(FakePrinter { pins: Some(pins_obj) });
    let config: Config = Arc::new(FakeConfig {
        printer,
        pins: Some(list.iter().map(|s| s.to_string()).collect()),
        section: "static_digital_output main".to_string(),
    });
    let obj = load_config_prefix(&config).expect("factory load_config_prefix");
    let obj = obj.expect("factory returns an object");
    let _rust: Arc<PrinterStaticDigitalOut> = downcast_arc(&obj).expect("downcast to PrinterStaticDigitalOut");
    let cmds = datas[0].1.cmds.lock().unwrap().clone();
    // Single-chip case: build_py_case compares in config order, matching
    // the reference iteration order, so the factory result proves parity too.
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("factory: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = cmds_list_value(&cmds);
    let record = Record {
        v: 1,
        unit: "static_digital_output".to_string(),
        scenario: "factory".to_string(),
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
        panic!("factory: {d}");
    }
}


fn assert_cmds_equal(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<()>,
    actual_cmds_in_order: Vec<(String, bool, bool)>,
) {
    if let Err(e) = actual {
        panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
    }
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = cmds_list_value(&actual_cmds_in_order);
    let record = Record {
        v: 1,
        unit: "static_digital_output".to_string(),
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
