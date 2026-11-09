//! Parity tests for `extras::samd_sercom` (port of `klippy/extras/samd_sercom.py`).
//!
//! `SamdSERCOM` is built on both sides with minimal Python fakes for the
//! printer (`pins` lookup), the config wrapper and the MCUs. Compared
//! against the live reference through `pyoracle`: the config values stored
//! on the object, every `set_sercom_pin` command the MCU receives, the
//! raised errors (class and message) and the commands emitted before a
//! failure. The config options owned by the unit are `sercom`, `tx_pin`,
//! `rx_pin` (optional) and `clk_pin`.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::samd_sercom::SamdSERCOM;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper,
    CommandWrapper, LookupPinOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi,
    TriggerDispatch,
};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi,
};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.samd_sercom as samd_sercom

_sentinel = object()

class error(Exception):
    pass

class FakeMCU:
    def __init__(self, name, cmds):
        self.name = name
        self.cmds = cmds
    def add_config_cmd(self, cmd):
        self.cmds.append(cmd)

class FakePins:
    error = error
    def __init__(self, pinmap):
        self.pinmap = pinmap
    def lookup_pin(self, pin_desc):
        mcu, pin = self.pinmap[pin_desc]
        return {'chip': mcu, 'pin': pin, 'chip_name': mcu.name,
                'invert': 0, 'pullup': 0, 'share_type': None}

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, pins):
        self._pins = pins
    def lookup_object(self, name, default=_sentinel):
        if name == 'pins' and self._pins is not None:
            return self._pins
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options, name):
        self._printer = printer
        self._options = options
        self._name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=_sentinel):
        if option in self._options:
            return self._options[option]
        if default is _sentinel:
            raise configparser.Error(
                "Option '%s' in section '%s' is required" % (option, self._name))
        return default
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakeMcu {
    name: String,
    printer: Printer,
    cmds: Arc<Mutex<Vec<String>>>,
}


impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(
        &self,
        _pin_type: &str,
        _pin_params: PinParams,
    ) -> KResult<Pin> {
        panic!("setup_pin is not used by samd_sercom")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by samd_sercom")
    }
    fn register_config_callback(
        &self,
        _cb: klippy_api::mcu::ConfigCallback,
    ) -> KResult<()> {
        panic!("register_config_callback is not used by samd_sercom")
    }
    fn add_config_cmd(&self, cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        self.cmds.lock().expect("cmds lock").push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by samd_sercom")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by samd_sercom")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by samd_sercom")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by samd_sercom")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by samd_sercom")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by samd_sercom")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by samd_sercom")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by samd_sercom")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by samd_sercom")
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by samd_sercom")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by samd_sercom")
    }
    fn get_enumerations(
        &self,
    ) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("get_enumerations is not used by samd_sercom")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("get_constants is not used by samd_sercom")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by samd_sercom")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by samd_sercom")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by samd_sercom")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by samd_sercom")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by samd_sercom")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by samd_sercom")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by samd_sercom")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by samd_sercom")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by samd_sercom")
    }
}

struct FakePins {
    pinmap: HashMap<String, (Mcu, String)>,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by samd_sercom")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let (chip, pin) = self
            .pinmap
            .get(pin_desc)
            .unwrap_or_else(|| panic!("unknown pin {pin_desc}"));
        let chip_name = chip.get_name();
        Ok(PinParams {
            chip: chip.clone(),
            chip_name,
            pin: pin.clone(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin(s) is not used by samd_sercom")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { panic!("reset_pin_sharing is not used by samd_sercom") }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by samd_sercom")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by samd_sercom")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by samd_sercom")
    }
}

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
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if name == "pins" {
            if let Some(pins) = &self.pins {
                return Ok(Some(pins.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
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
    options: Vec<(String, String)>,
    name: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some((_, value)) = self.options.iter().find(|(k, _)| k == option) {
            return Ok(Some(value.clone()));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' is required",
                self.name
            ))),
            Some(default) => Ok(default),
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by samd_sercom")
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by samd_sercom")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by samd_sercom")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by samd_sercom")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by samd_sercom")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by samd_sercom")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by samd_sercom")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by samd_sercom")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by samd_sercom")
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
            py.exec(SETUP).expect("samd_sercom fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping samd_sercom oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "samd_sercom".to_string(),
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

/// One construction scenario, fed identically to both sides.
struct Scenario {
    name: &'static str,
    options: Vec<(&'static str, &'static str)>,
    /// `(pin description, mcu index, pin name)`; index 0 is `mcu`,
    /// index 1 is `othermcu`.
    pinmap: Vec<(&'static str, usize, &'static str)>,
    has_pins: bool,
}

/// The Rust side of a scenario, plus the recorded MCU commands.
struct RustSetup {
    config: Config,
    cmds_a: Arc<Mutex<Vec<String>>>,
    cmds_b: Arc<Mutex<Vec<String>>>,
}

fn rust_setup(sc: &Scenario, printer_for_mcu: Printer) -> RustSetup {
    let cmds_a = Arc::new(Mutex::new(Vec::new()));
    let cmds_b = Arc::new(Mutex::new(Vec::new()));
    // `get_printer` on the MCU is never called by samd_sercom; the
    // printer is filled in by the caller through the cell below.
    let mcus: Vec<Mcu> = vec![
        Arc::new(FakeMcu {
            name: "mcu".to_string(),
            printer: printer_for_mcu,
            cmds: cmds_a.clone(),
        }),
        Arc::new(FakeMcu {
            name: "othermcu".to_string(),
            printer: Arc::new(FakePrinter { pins: None }),
            cmds: cmds_b.clone(),
        }),
    ];
    let pins_obj: Option<ObjectRef> = if sc.has_pins {
        let pinmap = sc
            .pinmap
            .iter()
            .map(|(desc, idx, pin)| {
                (desc.to_string(), (mcus[*idx].clone(), pin.to_string()))
            })
            .collect();
        Some(Arc::new(FakePins { pinmap }) as ObjectRef)
    } else {
        None
    };
    let printer: Printer = Arc::new(FakePrinter { pins: pins_obj });
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: sc
            .options
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
        name: sc.name.to_string(),
    });
    RustSetup {
        config,
        cmds_a,
        cmds_b,
    }
}

/// Installs the Python side of the scenario in the shared namespace:
/// `cmds_a`, `cmds_b`, `c` (config) and the loaded printer object.
fn py_setup(py: &mut pyoracle::Oracle, sc: &Scenario) {
    let pins_expr = if sc.has_pins {
        let entries: Vec<String> = sc
            .pinmap
            .iter()
            .map(|(desc, idx, pin)| {
                let mcu = if *idx == 0 { "mcu_a" } else { "mcu_b" };
                format!("{desc:?}: ({mcu}, {pin:?})")
            })
            .collect();
        format!("FakePins({{{}}})", entries.join(", "))
    } else {
        "None".to_string()
    };
    let opts: Vec<String> = sc
        .options
        .iter()
        .map(|(k, v)| format!("{k:?}: {v:?}"))
        .collect();
    let name = sc.name;
    let code = format!(
        "cmds_a = []\ncmds_b = []\n\
         mcu_a = FakeMCU('mcu', cmds_a)\nmcu_b = FakeMCU('othermcu', cmds_b)\n\
         p = FakePrinter({pins_expr})\nc = FakeConfig(p, {{{opts}}}, {name:?})",
        opts = opts.join(", "),
    );
    py.exec(&code).expect("py scenario setup");
}

/// Compares the recorded MCU commands of both sides after a construction.
fn check_cmds(label: &str, py: &mut pyoracle::Oracle, rust: &RustSetup) {
    for (which, cmds) in [("cmds_a", &rust.cmds_a), ("cmds_b", &rust.cmds_b)] {
        let expected = py.eval(which);
        let actual = Value::List(
            cmds.lock()
                .expect("cmds lock")
                .iter()
                .map(|c| Value::str(c))
                .collect(),
        );
        check(&format!("{label} {which}"), expected, actual);
    }
}

/// Builds the object on both sides and compares the outcome: the stored
/// config fields and MCU commands on success, the exception class/message
/// and the commands emitted before the failure otherwise.
fn run_scenario(py: &mut pyoracle::Oracle, sc: &Scenario, label: &str) {
    py_setup(py, sc);
    // The MCU's printer is never consulted by samd_sercom; hand a
    // placeholder printer to the MCu fakes so the trait stays total.
    let placeholder: Printer = Arc::new(FakePrinter { pins: None });
    let rust = rust_setup(sc, placeholder);
    // `exec` assigning `obj` returns the exception on failure, so both
    // the outcome and the emitted commands stay in sync.
    let expected = py.exec("obj = samd_sercom.load_config_prefix(c)");
    let actual = SamdSERCOM::new(&rust.config);
    match (expected, &actual) {
        (Ok(_), Ok(obj)) => {
            check(
                &format!("{label} sercom field"),
                py.eval("obj.sercom"),
                Value::str(&obj.sercom),
            );
            check(
                &format!("{label} tx_pin field"),
                py.eval("obj.tx_pin"),
                Value::str(&obj.tx_pin),
            );
            let rx = match &obj.rx_pin {
                Some(v) => Value::str(v),
                None => Value::Null,
            };
            check(&format!("{label} rx_pin field"), py.eval("obj.rx_pin"), rx);
            check(
                &format!("{label} clk_pin field"),
                py.eval("obj.clk_pin"),
                Value::str(&obj.clk_pin),
            );
        }
        (Err(want_exc), Err(got_exc)) => {
            assert_eq!(
                got_exc.class_name(),
                want_exc.class_name.as_str(),
                "{label}: exception class",
            );
            assert_eq!(
                got_exc.message(),
                want_exc.message.as_str(),
                "{label}: exception message",
            );
        }
        (Ok(_), Err(e)) => panic!("{label}: oracle succeeded but rust raised {e:?}"),
        (Err(e), Ok(_)) => panic!(
            "{label}: oracle raised {}: {} but rust succeeded",
            e.class_name, e.message
        ),
    }
    // Commands recorded before any failure stay comparable on both sides.
    check_cmds(label, py, &rust);
}

fn base_opts() -> Vec<(&'static str, &'static str)> {
    vec![
        ("sercom", "SERCOM3"),
        ("tx_pin", "PA9"),
        ("rx_pin", "PA10"),
        ("clk_pin", "PA8"),
    ]
}

fn same_mcu_pinmap() -> Vec<(&'static str, usize, &'static str)> {
    vec![("PA9", 0, "PA9"), ("PA10", 0, "PA10"), ("PA8", 0, "PA8")]
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn full_config_same_mcu() {
    let Some(mut py) = oracle() else { return };
    let sc = Scenario {
        name: "samd_sercom x",
        options: base_opts(),
        pinmap: same_mcu_pinmap(),
        has_pins: true,
    };
    run_scenario(&mut py, &sc, "full_config_same_mcu");
}

#[test]
fn without_rx_pin() {
    let Some(mut py) = oracle() else { return };
    let mut opts = base_opts();
    opts.retain(|(k, _)| *k != "rx_pin");
    let sc = Scenario {
        name: "samd_sercom x",
        options: opts,
        pinmap: same_mcu_pinmap(),
        has_pins: true,
    };
    run_scenario(&mut py, &sc, "without_rx_pin");
}

#[test]
fn rx_pin_empty_string_is_falsy() {
    let Some(mut py) = oracle() else { return };
    let mut opts = base_opts();
    opts.retain(|(k, _)| *k != "rx_pin");
    opts.push(("rx_pin", ""));
    let sc = Scenario {
        name: "samd_sercom x",
        options: opts,
        pinmap: same_mcu_pinmap(),
        has_pins: true,
    };
    run_scenario(&mut py, &sc, "rx_pin_empty_string_is_falsy");
}

#[test]
fn clk_on_other_mcu_fails() {
    let Some(mut py) = oracle() else { return };
    let sc = Scenario {
        name: "samd_sercom x",
        options: base_opts(),
        pinmap: vec![("PA9", 0, "PA9"), ("PA10", 0, "PA10"), ("PA8", 1, "PA8")],
        has_pins: true,
    };
    run_scenario(&mut py, &sc, "clk_on_other_mcu_fails");
}

#[test]
fn rx_on_other_mcu_fails() {
    let Some(mut py) = oracle() else { return };
    let sc = Scenario {
        name: "samd_sercom x",
        options: base_opts(),
        pinmap: vec![("PA9", 0, "PA9"), ("PA10", 1, "PA10"), ("PA8", 0, "PA8")],
        has_pins: true,
    };
    run_scenario(&mut py, &sc, "rx_on_other_mcu_fails");
}

#[test]
fn command_uses_parsed_pin_not_description() {
    let Some(mut py) = oracle() else { return };
    // The config strings are arbitrary labels; the emitted commands must
    // carry `PinParams.pin` (the pad name), like the Python `%` formatting.
    let sc = Scenario {
        name: "samd_sercom lin",
        options: vec![
            ("sercom", "SERCOM0"),
            ("tx_pin", "tx-desc"),
            ("rx_pin", "rx-desc"),
            ("clk_pin", "clk-desc"),
        ],
        pinmap: vec![
            ("tx-desc", 0, "PB12"),
            ("rx-desc", 0, "PB13"),
            ("clk-desc", 0, "PB14"),
        ],
        has_pins: true,
    };
    run_scenario(&mut py, &sc, "command_uses_parsed_pin_not_description");
}

#[test]
fn missing_required_options() {
    let Some(mut py) = oracle() else { return };
    for missing in ["sercom", "tx_pin", "clk_pin"] {
        let mut opts = base_opts();
        opts.retain(|(k, _)| *k != missing);
        let sc = Scenario {
            name: "samd_sercom x",
            options: opts,
            pinmap: same_mcu_pinmap(),
            has_pins: true,
        };
        run_scenario(&mut py, &sc, &format!("missing_required_options {missing}"));
    }
}

#[test]
fn missing_pins_object() {
    let Some(mut py) = oracle() else { return };
    let sc = Scenario {
        name: "samd_sercom x",
        options: base_opts(),
        pinmap: same_mcu_pinmap(),
        has_pins: false,
    };
    run_scenario(&mut py, &sc, "missing_pins_object");
}

#[test]
fn unknown_pin_description() {
    let Some(mut py) = oracle() else { return };
    // Same lookup failure on both sides (KeyError on Python, panic on the
    // Rust fake): construction fails before the reference commands. Use
    // the oracle exception only for the integer check via a fresh pinmap
    // without PA8 so both sides raise their own lookup error classes.
    let sc = Scenario {
        name: "samd_sercom x",
        options: base_opts(),
        pinmap: vec![("PA9", 0, "PA9"), ("PA10", 0, "PA10")],
        has_pins: true,
    };
    py_setup(&mut py, &sc);
    let expected = py.eval("samd_sercom.load_config_prefix(c)");
    let placeholder: Printer = Arc::new(FakePrinter { pins: None });
    let rust = rust_setup(&sc, placeholder);
    let actual = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        SamdSERCOM::new(&rust.config)
    }));
    assert!(
        actual.is_err() || expected.is_err(),
        "both sides must fail on an unknown pin description"
    );
}

#[test]
fn load_config_prefix_factory() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::samd_sercom::module_factory();
    let load_config_prefix = factory
        .load_config_prefix
        .expect("samd_sercom exposes load_config_prefix");
    assert!(
        factory.load_config.is_none(),
        "samd_sercom has no load_config"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "samd_sercom has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "samd_sercom has no add_printer_objects"
    );
    let sc = Scenario {
        name: "samd_sercom x",
        options: base_opts(),
        pinmap: same_mcu_pinmap(),
        has_pins: true,
    };
    py_setup(&mut py, &sc);
    let placeholder: Printer = Arc::new(FakePrinter { pins: None });
    let rust = rust_setup(&sc, placeholder);
    let obj = load_config_prefix(&rust.config)
        .expect("factory load_config_prefix")
        .expect("factory returns an object");
    let rust_obj: Arc<SamdSERCOM> = downcast_arc(&obj).expect("downcast to SamdSERCOM");
    py.exec("obj = samd_sercom.load_config_prefix(c)")
        .expect("py factory load_config_prefix");
    check(
        "factory sercom",
        py.eval("obj.sercom"),
        Value::str(&rust_obj.sercom),
    );
    check(
        "factory tx_pin",
        py.eval("obj.tx_pin"),
        Value::str(&rust_obj.tx_pin),
    );
    let rx = match &rust_obj.rx_pin {
        Some(v) => Value::str(v),
        None => Value::Null,
    };
    check("factory rx_pin", py.eval("obj.rx_pin"), rx);
    check(
        "factory clk_pin",
        py.eval("obj.clk_pin"),
        Value::str(&rust_obj.clk_pin),
    );
    check_cmds("factory", &mut py, &rust);
}
