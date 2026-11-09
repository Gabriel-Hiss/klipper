//! Parity tests for `extras::board_pins` (port of `klippy/extras/board_pins.py`).
//!
//! Every case builds the reference object with minimal Python fakes for the
//! printer, config and pins through `pyoracle` (CPython 3.8.2 + frozen
//! `klippy/`), runs the same inputs on the Rust port through fakes of the
//! contract traits, and compares the raised errors (class and message) and
//! the exact pin-resolver call sequence (order, method and arguments).
//! The Python side uses the real `configfile.ConfigWrapper` and
//! `pins.PinResolver`, so config parsing and resolver validation match the
//! reference; the Rust fakes replicate that parsing and validation.

use extras::board_pins::{module_factory, PrinterBoardAliases};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{Chip, Pin, PinParams, PinsApi};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import configfile
import pins
import extras.board_pins as board_pins

class FakePinResolver:
    def __init__(self, chip, calls):
        self.chip = chip
        self.calls = calls
        self.inner = pins.PinResolver()
    def reserve_pin(self, pin, reserve_name):
        self.calls.append((self.chip, 'reserve_pin', pin, reserve_name))
        self.inner.reserve_pin(pin, reserve_name)
    def alias_pin(self, alias, pin):
        self.calls.append((self.chip, 'alias_pin', alias, pin))
        self.inner.alias_pin(alias, pin)
    def update_command(self, cmd):
        return self.inner.update_command(cmd)

class FakePins:
    def __init__(self, chips, calls):
        self.calls = calls
        self.resolvers = {c: FakePinResolver(c, calls) for c in chips}
    def get_pin_resolver(self, chip_name):
        self.calls.append(('pins', 'get_pin_resolver', chip_name))
        if chip_name not in self.resolvers:
            raise pins.error("Unknown chip name '%s'" % (chip_name,))
        return self.resolvers[chip_name]

class FakePrinter:
    def __init__(self, pins_obj):
        self.objects = {}
        if pins_obj is not None:
            self.objects['pins'] = pins_obj
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        self.objects[name] = obj

def make_printer(pins_obj):
    return FakePrinter(pins_obj)

def make_config(printer, section, options):
    cp = configparser.RawConfigParser(strict=False)
    cp.add_section(section)
    for k, v in options.items():
        cp.set(section, k, v)
    tracking = {}
    return configfile.ConfigWrapper(printer, cp, tracking, section)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Calls = Arc<Mutex<Vec<Vec<String>>>>;

fn pins_error(msg: String) -> KlippyError {
    KlippyError::Pins(msg)
}

struct ResolverState {
    reserved: IndexMap<String, String>,
    aliases: IndexMap<String, String>,
}

struct FakePinResolver {
    chip: String,
    calls: Calls,
    state: Mutex<ResolverState>,
}

impl FakePinResolver {
    fn new(chip: &str, calls: Calls) -> Self {
        FakePinResolver {
            chip: chip.to_string(),
            calls,
            state: Mutex::new(ResolverState {
                reserved: IndexMap::new(),
                aliases: IndexMap::new(),
            }),
        }
    }
}

impl klippy_api::mcu::PinResolverApi for FakePinResolver {
    fn reserve_pin(&self, pin: &str, reserve_name: &str) -> KResult<()> {
        self.calls.lock().push(vec![
            self.chip.clone(),
            "reserve_pin".to_string(),
            pin.to_string(),
            reserve_name.to_string(),
        ]);
        let mut state = self.state.lock();
        if let Some(existing) = state.reserved.get(pin) {
            if existing != reserve_name {
                return Err(pins_error(format!(
                    "Pin {pin} reserved for {existing} - can't reserve for {reserve_name}"
                )));
            }
        }
        state.reserved.insert(pin.to_string(), reserve_name.to_string());
        Ok(())
    }

    fn alias_pin(&self, alias: &str, pin: &str) -> KResult<()> {
        self.calls.lock().push(vec![
            self.chip.clone(),
            "alias_pin".to_string(),
            alias.to_string(),
            pin.to_string(),
        ]);
        let mut state = self.state.lock();
        if let Some(existing) = state.aliases.get(alias) {
            if existing != pin {
                return Err(pins_error(format!(
                    "Alias {alias} mapped to {existing} - can't alias to {pin}"
                )));
            }
        }
        if pin.chars().any(|c| matches!(c, '^' | '~' | '!' | ':'))
            || pin.split_whitespace().collect::<String>() != pin
        {
            return Err(pins_error(format!("Invalid pin alias '{pin}'\n")));
        }
        let resolved = state
            .aliases
            .get(pin)
            .cloned()
            .unwrap_or_else(|| pin.to_string());
        state.aliases.insert(alias.to_string(), resolved.clone());
        let targets: Vec<String> = state
            .aliases
            .iter()
            .filter(|(_, v)| *v == alias)
            .map(|(k, _)| k.clone())
            .collect();
        for key in targets {
            state.aliases.insert(key, resolved.clone());
        }
        Ok(())
    }

    fn update_command(&self, _cmd: &str) -> KResult<String> {
        panic!("update_command is not used by board_pins")
    }
}

struct FakePins {
    resolvers: IndexMap<String, Arc<FakePinResolver>>,
    calls: Calls,
}

impl FakePins {
    fn new(chips: &[&str], calls: Calls) -> Self {
        let mut resolvers = IndexMap::new();
        for chip in chips {
            resolvers.insert(
                chip.to_string(),
                Arc::new(FakePinResolver::new(chip, calls.clone())),
            );
        }
        FakePins { resolvers, calls }
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by board_pins")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used by board_pins")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by board_pins")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by board_pins")
    }
    fn get_pin_resolver(&self, chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        self.calls.lock().push(vec![
            "pins".to_string(),
            "get_pin_resolver".to_string(),
            chip_name.to_string(),
        ]);
        match self.resolvers.get(chip_name) {
            Some(resolver) => Ok(resolver.clone() as klippy_api::mcu::PinResolver),
            None => Err(pins_error(format!("Unknown chip name '{chip_name}'"))),
        }
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used by board_pins")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by board_pins")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

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

struct FakeConfig {
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
}

fn missing_option(section: &str, option: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

fn count_error(section: &str, option: &str, count: usize) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section '{section}' must have {count} elements"
    ))
}

fn lparser(
    value: &str,
    pos: usize,
    seps: &[String],
    count: Option<usize>,
    option: &str,
    section: &str,
) -> KResult<PyValue> {
    let parts: Vec<String> = if value.trim().is_empty() {
        Vec::new()
    } else {
        value
            .split(&seps[pos])
            .map(|p| p.trim().to_string())
            .collect()
    };
    if pos > 0 {
        let mut out = Vec::new();
        for part in &parts {
            if part.is_empty() {
                continue;
            }
            out.push(lparser(part, pos - 1, seps, count, option, section)?);
        }
        Ok(PyValue::Tuple(out))
    } else {
        if let Some(want) = count {
            if parts.len() != want {
                return Err(count_error(section, option, want));
            }
        }
        Ok(PyValue::Tuple(
            parts.into_iter().map(PyValue::Str).collect(),
        ))
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
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
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
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        match self.options.get(option) {
            None => match opts.default {
                Some(Some(value)) => Ok(value),
                Some(None) => Ok(PyValue::None),
                None => Err(missing_option(&self.section, option)),
            },
            Some(raw) => lparser(
                raw,
                opts.seps.len() - 1,
                &opts.seps,
                opts.count,
                option,
                &self.section,
            ),
        }
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match self.options.get(option) {
            None => match opts.default {
                Some(Some(value)) => Ok(Some(value)),
                Some(None) => Ok(None),
                None => Err(missing_option(&self.section, option)),
            },
            Some(raw) => {
                let parts: Vec<String> = if raw.trim().is_empty() {
                    Vec::new()
                } else {
                    raw.split(&opts.sep)
                        .map(|p| p.trim().to_string())
                        .collect()
                };
                if let Some(want) = opts.count {
                    if parts.len() != want {
                        return Err(count_error(&self.section, option, want));
                    }
                }
                Ok(Some(parts))
            }
        }
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
        panic!("getsection is not used by board_pins")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, prefix: &str) -> Vec<String> {
        self.options
            .keys()
            .filter(|key| key.starts_with(prefix))
            .cloned()
            .collect()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("board_pins fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping board_pins oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_str(text: &str) -> String {
    format!("'{text}'")
}

fn py_str_list(items: &[&str]) -> String {
    let inner: Vec<String> = items.iter().map(|item| py_str(item)).collect();
    format!("[{}]", inner.join(", "))
}

fn py_options(options: &[(&str, &str)]) -> String {
    let inner: Vec<String> = options
        .iter()
        .map(|(key, value)| format!("{}: {}", py_str(key), py_str(value)))
        .collect();
    format!("{{{}}}", inner.join(", "))
}

struct Scenario {
    chips: Vec<String>,
    options: Vec<(&'static str, &'static str)>,
    section: &'static str,
    with_pins: bool,
}

fn run_one(py: &mut pyoracle::Oracle, label: &str, scenario: &Scenario) {
    let chip_refs: Vec<&str> = scenario.chips.iter().map(String::as_str).collect();
    py.exec("py_calls = []").expect("py calls list");
    if scenario.with_pins {
        let chips_expr = py_str_list(&chip_refs);
        py.exec(&format!(
            "py_pins = FakePins({chips_expr}, py_calls)"
        ))
        .expect("py pins");
    } else {
        py.exec("py_pins = None").expect("py no pins");
    }
    py.exec("py_printer = make_printer(py_pins)").expect("py printer");
    let options_expr = py_options(&scenario.options);
    py.exec(&format!(
        "py_config = make_config(py_printer, {}, {options_expr})",
        py_str(scenario.section)
    ))
    .expect("py config");
    let py_result = py.eval("board_pins.load_config(py_config)");
    let py_calls = py.eval("py_calls").expect("py calls value");

    let calls: Calls = Arc::new(Mutex::new(Vec::new()));
    let pins_obj: Option<ObjectRef> = if scenario.with_pins {
        Some(Arc::new(FakePins::new(&chip_refs, calls.clone())) as ObjectRef)
    } else {
        None
    };
    let printer: Printer = Arc::new(FakePrinter { pins: pins_obj });
    let mut options = IndexMap::new();
    for (key, value) in &scenario.options {
        options.insert(key.to_string(), value.to_string());
    }
    let config: Config = Arc::new(FakeConfig {
        printer,
        section: scenario.section.to_string(),
        options,
    });
    let rust_result = PrinterBoardAliases::new(&config);
    let rust_calls: Vec<Vec<String>> = calls.lock().clone();
    let actual = Value::List(
        rust_calls
            .iter()
            .map(|call| {
                Value::Tuple(call.iter().map(|arg| Value::str(arg)).collect())
            })
            .collect(),
    );

    match (py_result, rust_result) {
        (Ok(_), Ok(_)) => {
            let record = Record {
                v: 1,
                unit: "board_pins".to_string(),
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
            if let Err(divergence) = assert_equal(&record, "calls", &py_calls, &actual) {
                panic!("{label}: {divergence}");
            }
        }
        (Err(py_exc), Err(rust_exc)) => {
            assert_eq!(
                rust_exc.class_name(),
                py_exc.class_name.as_str(),
                "{label}: exception class"
            );
            assert_eq!(
                rust_exc.message(),
                py_exc.message.as_str(),
                "{label}: exception message"
            );
            let record = Record {
                v: 1,
                unit: "board_pins".to_string(),
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
            if let Err(divergence) = assert_equal(&record, "calls", &py_calls, &actual) {
                panic!("{label}: {divergence}");
            }
        }
        (Ok(_), Err(rust_exc)) => {
            panic!("{label}: python succeeded but rust raised {}: {}", rust_exc.class_name(), rust_exc.message());
        }
        (Err(py_exc), Ok(_)) => {
            panic!(
                "{label}: python raised {}: {} but rust succeeded",
                py_exc.class_name, py_exc.message
            );
        }
    }
}


// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn basic_alias_default_mcu() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "basic_alias_default_mcu",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=PA0,BB=PB1")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn missing_aliases_option_raises() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "missing_aliases_option_raises",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn empty_aliases_value() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "empty_aliases_value",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn reserve_angle_bracket_values() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "reserve_angle_bracket_values",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=<RES1>,BB=<>,CC=<")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn trailing_angle_value_aliases() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "trailing_angle_value_aliases",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=>,BB=PA0>,CC=<PA0")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn multiple_mcus() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "multiple_mcus",
        &Scenario {
            chips: vec!["mcu".to_string(), "mcu2".to_string()],
            options: vec![
                ("mcu", "mcu, mcu2"),
                ("aliases", "AA=PA0,BB=<RES1>"),
            ],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn unknown_chip_raises() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "unknown_chip_raises",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("mcu", "missing"), ("aliases", "AA=PA0")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn prefix_options_order() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "prefix_options_order",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![
                ("aliases_foo", "AA=PA0"),
                ("aliases", "BB=PB1"),
                ("aliases_bar", "CC=PC2"),
            ],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn count_error_missing_equals() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "count_error_missing_equals",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn count_error_extra_equals() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "count_error_extra_equals",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=PA0=extra")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn whitespace_stripping() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "whitespace_stripping",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", " AA = PA0 , BB = <X> ")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn invalid_pin_alias_raises() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "invalid_pin_alias_raises",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=PA 0")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn duplicate_alias_conflict() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "duplicate_alias_conflict",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=PA0,AA=PA1")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn reserve_conflict() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "reserve_conflict",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=<R1>,AA=<R2>")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn missing_pins_object_raises() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "missing_pins_object_raises",
        &Scenario {
            chips: vec![],
            options: vec![("aliases", "AA=PA0")],
            section: "board_pins",
            with_pins: false,
        },
    );
}

#[test]
fn empty_mcu_list_noop() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "empty_mcu_list_noop",
        &Scenario {
            chips: vec![],
            options: vec![("mcu", ""), ("aliases", "AA=PA 0")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn alias_chain() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "alias_chain",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=BB,BB=PA0")],
            section: "board_pins",
            with_pins: true,
        },
    );
}

#[test]
fn named_section_prefix() {
    let Some(mut py) = oracle() else { return };
    run_one(
        &mut py,
        "named_section_prefix",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![],
            section: "board_pins myname",
            with_pins: true,
        },
    );
}

#[test]
fn factory_load_config_prefix_builds_object() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory
        .load_config
        .expect("board_pins exposes load_config");
    let load_config_prefix = factory
        .load_config_prefix
        .expect("board_pins exposes load_config_prefix");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "board_pins has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "board_pins has no add_printer_objects"
    );
    run_one(
        &mut py,
        "factory_load_config_prefix_builds_object py",
        &Scenario {
            chips: vec!["mcu".to_string()],
            options: vec![("aliases", "AA=PA0")],
            section: "board_pins myname",
            with_pins: true,
        },
    );
    let calls: Calls = Arc::new(Mutex::new(Vec::new()));
    let pins_obj: ObjectRef = Arc::new(FakePins::new(&["mcu"], calls.clone())) as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter { pins: Some(pins_obj) });
    let mut options = IndexMap::new();
    options.insert("aliases".to_string(), "AA=PA0".to_string());
    let config: Config = Arc::new(FakeConfig {
        printer,
        section: "board_pins myname".to_string(),
        options,
    });
    let obj: ObjectRef = load_config_prefix(&config)
        .expect("factory load_config_prefix")
        .expect("factory load_config_prefix returns an object");
    let _rust: Arc<PrinterBoardAliases> = downcast_arc(&obj).expect("downcast");
    let obj2: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let _rust2: Arc<PrinterBoardAliases> = downcast_arc(&obj2).expect("downcast");
    let locked: Vec<Vec<String>> = calls.lock().clone();
    assert_eq!(locked.len(), 4, "two builds resolve and alias twice");
}
