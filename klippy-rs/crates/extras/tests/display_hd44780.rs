//! Parity tests for `extras::display::hd44780`.
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `HD44780` is built
//! on both sides with minimal Python fakes for the printer, pins, config
//! and MCU, then fed identical calls and the resulting command streams,
//! framebuffers, return values and errors are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::display::hd44780::{HD44780, Hd44780Api, module_factory, BACKGROUND_PRIORITY_CLOCK, HD44780_DELAY, LINE_LENGTH_DEFAULT};
use indexmap::IndexMap;
use klippy_api::config::{Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{AddConfigCmdOpts, CommandQueue, CommandWrapper, CommandWrapperApi, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi, SendOpts, Chip, ChipApi};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::ObjectRef;
use klippy_api::{downcast_arc, EventArg};
use pycompat::{PyValue, PyInt};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.display.hd44780 as hd44780_mod

_SENTINEL = object()

class error(Exception):
    pass

class FakeCmd:
    def __init__(self, mcu, msgformat):
        self._mcu = mcu
        self._msgformat = msgformat
    def send(self, data=(), minclock=0, reqclock=0):
        self._mcu.sends.append((self._msgformat, list(data), minclock, reqclock))

class FakeMcu:
    def __init__(self):
        self._oid = 0
        self.config_cmds = []
        self.callbacks = []
        self.sends = []
    def create_oid(self):
        self._oid += 1
        return self._oid - 1
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append(cmd)
    def alloc_command_queue(self):
        return 0
    def lookup_command(self, msgformat, cq=None):
        return FakeCmd(self, msgformat)
    def seconds_to_clock(self, time):
        return int(time * 1000000)
    def estimated_print_time(self, eventtime):
        return 10.0
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000)

class FakePins:
    error = error
    def __init__(self, mcu, mapping=None):
        self._mcu = mcu
        self.mapping = mapping or {}
    def lookup_pin(self, pin_desc):
        chip = self.mapping.get(pin_desc, self._mcu)
        if ':' not in pin_desc:
            chip_name, pin = 'mcu', pin_desc
        else:
            chip_name, pin = [s.strip() for s in pin_desc.split(':', 1)]
        return {'chip': chip, 'chip_name': chip_name, 'pin': pin, 'invert': 0, 'pullup': 0}

class FakeReactor:
    def monotonic(self):
        return 123.0

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, pins):
        self._pins = pins
        self._reactor = FakeReactor()
    def get_reactor(self):
        return self._reactor
    def lookup_object(self, name, default=_SENTINEL):
        if name == 'pins':
            return self._pins
        if default is _SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, opts=None):
        self._printer = printer
        self.opts = opts or {}
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'display'
    def get(self, option, default=_SENTINEL):
        if option not in self.opts:
            if default is not _SENTINEL:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, 'display'))
        return self.opts[option]
    def getboolean(self, option, default=_SENTINEL):
        if option not in self.opts:
            if default is not _SENTINEL:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, 'display'))
        return self.opts[option]
    def getchoice(self, option, choices, default=_SENTINEL):
        if option not in self.opts:
            if default is not _SENTINEL:
                c = default
            else:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, 'display'))
        else:
            c = self.opts[option]
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if c not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, 'display'))
        return choices[c]
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Sent {
    msgformat: String,
    oid: i64,
    cmds: Vec<u8>,
    minclock: i64,
    reqclock: i64,
}

struct FakeMcuState {
    oid_counter: Mutex<i64>,
    config_cmds: Mutex<Vec<String>>,
    callbacks: Mutex<Vec<Box<dyn FnMut() -> KResult<()> + Send>>>,
    sends: Mutex<Vec<Sent>>,
}

struct FakeMcu {
    state: Arc<FakeMcuState>,
}

struct FakeDummyPrinter;

impl PrinterApi for FakeDummyPrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> { Ok(()) }
    fn lookup_object(&self, _name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> { Ok(None) }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> { Vec::new() }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> { Ok(None) }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakeDummyPrinter);

struct FakeCommandWrapper {
    msgformat: String,
    state: Arc<FakeMcuState>,
}

impl CommandWrapperApi for FakeCommandWrapper {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let (oid, cmds) = match (&data[0], &data[1]) {
            (PyValue::Int(n), PyValue::Bytes(b)) => (n.0.to_string().parse::<i64>().unwrap(), b.clone()),
            _ => (0, Vec::new()),
        };
        self.state.sends.lock().unwrap().push(Sent {
            msgformat: self.msgformat.clone(),
            oid,
            cmds,
            minclock: opts.minclock,
            reqclock: opts.reqclock,
        });
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> { Ok(()) }
    fn get_command_tag(&self) -> i64 { 0 }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String { "mcu".to_string() }
    fn get_printer(&self) -> Printer { Arc::new(FakeDummyPrinter) }
    fn is_fileoutput(&self) -> bool { false }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut counter = self.state.oid_counter.lock().unwrap();
        let oid = *counter;
        *counter += 1;
        Ok(oid)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.state.callbacks.lock().unwrap().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        self.state.config_cmds.lock().unwrap().push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, time: f64) -> i64 { (time * 1000000.0) as i64 }
    fn min_schedule_time(&self) -> f64 { 0.0 }
    fn max_nominal_duration(&self) -> f64 { 9999.0 }
    fn lookup_command(&self, msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCommandWrapper { msgformat: msgformat.to_string(), state: Arc::clone(&self.state) }))
    }
    fn lookup_query_command(&self, _msgformat: &str, _respformat: &str, _opts: LookupQueryCommandOpts) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> { None }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(0) }
    fn register_serial_response(&self, _cb: klippy_api::mcu::SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("unused")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool { true }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { IndexMap::new() }
    fn get_constants(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> { Err(KlippyError::Mcu("unused".to_string())) }
    fn print_time_to_clock(&self, print_time: f64) -> i64 { (print_time * 1000.0) as i64 }
    fn clock_to_print_time(&self, clock: f64) -> f64 { clock / 1000.0 }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 { 10.0 }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { clock32 }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> { Ok((print_time, 0.0)) }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::None }
    fn stats(&self, _eventtime: f64) -> (bool, String) { (false, String::new()) }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
}

struct FakePins {
    mcu: Mcu,
    mapping: Vec<(String, Mcu)>,
}

impl FakePins {
    fn chip_for(&self, pin_desc: &str) -> Mcu {
        for (desc, chip) in &self.mapping {
            if desc == pin_desc {
                return chip.clone();
            }
        }
        self.mcu.clone()
    }
}

fn pin_params_for(chip: &Mcu, pin_desc: &str) -> PinParams {
    let (chip_name, pin) = if let Some((c, p)) = pin_desc.split_once(':') {
        (c.trim().to_string(), p.trim().to_string())
    } else {
        ("mcu".to_string(), pin_desc.to_string())
    };
    PinParams { chip: chip.clone(), chip_name, pin, invert: 0, pullup: 0, share_type: None }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Ok(pin_params_for(&self.chip_for(pin_desc), pin_desc))
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        Ok(pin_params_for(&self.chip_for(pin_desc), pin_desc))
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::Other { class_name: "error".to_string(), message: "unused".to_string() })
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        Err(KlippyError::Other { class_name: "error".to_string(), message: "unused".to_string() })
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Err(KlippyError::Other { class_name: "error".to_string(), message: "unused".to_string() })
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl ChipApi for FakePins {
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Other { class_name: "error".to_string(), message: "unused".to_string() })
    }
    fn get_mcu(&self) -> klippy_api::KResult<Mcu> { Ok(self.mcu.clone()) }
}

struct FakePrinter {
    pins_obj: ObjectRef,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> { Ok(()) }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if name == "pins" {
            return Ok(Some(self.pins_obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> { Vec::new() }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _result: &str) {}
}

#[derive(Clone, Debug)]
enum FakeOpt {
    Str(String),
    Bool(bool),
    Int(i64),
}

struct FakeConfig {
    printer: Printer,
    opts: Vec<(String, FakeOpt)>,
}

impl FakeConfig {
    fn lookup(&self, option: &str) -> Option<&FakeOpt> {
        self.opts.iter().find(|(k, _)| k == option).map(|(_, v)| v)
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { "display".to_string() }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.lookup(option) {
            Some(FakeOpt::Str(v)) => Ok(Some(v.clone())),
            _ => match opts.default {
                None => Err(KlippyError::config(format!("Option '{option}' in section 'display' must be specified"))),
                Some(default) => Ok(default),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> { Ok(None) }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.lookup(option) {
            Some(FakeOpt::Bool(v)) => Ok(Some(*v)),
            _ => match opts.default {
                None => Err(KlippyError::config(format!("Option '{option}' in section 'display' must be specified"))),
                Some(default) => Ok(default),
            },
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let value = match self.lookup(option) {
            Some(FakeOpt::Int(v)) => *v,
            _ => match &opts.default {
                None => return Err(KlippyError::config(format!("Option '{option}' in section 'display' must be specified"))),
                Some(Some(PyValue::Int(n))) => n.0.to_string().parse::<i64>().unwrap_or(0),
                _ => 0,
            },
        };
        let valid = match choices {
            PyValue::List(items) => items.iter().any(|item| matches!(item, PyValue::Int(n) if n.0.to_string().parse::<i64>().unwrap_or(i64::MIN) == value)),
            _ => false,
        };
        if !valid {
            return Err(KlippyError::config(format!("Choice '{value}' for option '{option}' in section 'display' is not a valid choice")));
        }
        Ok(PyValue::Int(PyInt::from_i64(value)))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, _section: &str) -> Config { panic!("getsection is not used by hd44780") }
    fn has_section(&self, _section: &str) -> bool { false }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> { Ok(Vec::new()) }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}
const PY_SENDS_EXPR: &str = "[(f, d[0], bytes(bytearray(d[1])), mc, rc) for (f, d, mc, rc) in hd.mcu.sends]";

// ---------------------------------------------------------------------------
// Harness: build the same HD44780 on both sides.
// ---------------------------------------------------------------------------

struct Harness {
    rust: Arc<HD44780>,
    state: Arc<FakeMcuState>,
}

fn py_opts_expr(extra: &[(String, String)]) -> String {
    let mut s = "{'rs_pin': 'PA0', 'e_pin': 'PA1', 'd4_pin': 'PA2', 'd5_pin': 'PA3', 'd6_pin': 'PA4', 'd7_pin': 'PA5'".to_string();
    for (k, v) in extra {
        s.push_str(&format!(", {k:?}: {v}"));
    }
    s.push('}');
    s
}

fn fresh_state() -> Arc<FakeMcuState> {
    Arc::new(FakeMcuState {
        oid_counter: Mutex::new(0),
        config_cmds: Mutex::new(Vec::new()),
        callbacks: Mutex::new(Vec::new()),
        sends: Mutex::new(Vec::new()),
    })
}

fn build_rust(state: Arc<FakeMcuState>, extra_fake: Vec<(String, FakeOpt)>, mapping: Vec<(String, Mcu)>) -> KResult<Arc<HD44780>> {
    let mcu: Mcu = Arc::new(FakeMcu { state: Arc::clone(&state) });
    let pins: ObjectRef = Arc::new(FakePins { mcu: mcu.clone(), mapping });
    let printer: Printer = Arc::new(FakePrinter { pins_obj: pins });
    let mut opts = vec![
        ("rs_pin".to_string(), FakeOpt::Str("PA0".to_string())),
        ("e_pin".to_string(), FakeOpt::Str("PA1".to_string())),
        ("d4_pin".to_string(), FakeOpt::Str("PA2".to_string())),
        ("d5_pin".to_string(), FakeOpt::Str("PA3".to_string())),
        ("d6_pin".to_string(), FakeOpt::Str("PA4".to_string())),
        ("d7_pin".to_string(), FakeOpt::Str("PA5".to_string())),
    ];
    opts.extend(extra_fake);
    let config: Config = Arc::new(FakeConfig { printer, opts });
    HD44780::new(&config)
}

fn fresh(
    py: &mut pyoracle::Oracle,
    py_extra: Vec<(String, String)>,
    rust_extra: Vec<(String, FakeOpt)>,
) -> Harness {
    let opts = py_opts_expr(&py_extra);
    py.exec(&format!("hd = (lambda: (lambda mcu: (lambda pins: (lambda printer: (lambda cfg: hd44780_mod.HD44780(cfg))(FakeConfig(printer, {opts})))(FakePrinter(pins)))(FakePins(mcu)))(FakeMcu()))()"))
        .expect("py build");
    let state = fresh_state();
    let rust = build_rust(Arc::clone(&state), rust_extra, Vec::new()).expect("rust build");
    Harness { rust, state }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("hd44780 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping hd44780 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.display.hd44780".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(actual.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(actual.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

fn sends_value(state: &Arc<FakeMcuState>) -> Value {
    let sends = state.sends.lock().unwrap();
    Value::List(sends.iter().map(|s| {
        Value::Tuple(vec![
            Value::Str(s.msgformat.clone()),
            Value::Int(s.oid),
            Value::Bytes(s.cmds.clone()),
            Value::Int(s.minclock),
            Value::Int(s.reqclock),
        ])
    }).collect())
}


// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check("BACKGROUND", py.eval("hd44780_mod.BACKGROUND_PRIORITY_CLOCK"), Value::Int(BACKGROUND_PRIORITY_CLOCK));
    check("LINE_LENGTH_DEFAULT", py.eval("hd44780_mod.LINE_LENGTH_DEFAULT"), Value::Int(LINE_LENGTH_DEFAULT));
    check("HD44780_DELAY", py.eval("hd44780_mod.HD44780_DELAY"), Value::float(HD44780_DELAY));
    check("BACKGROUND_TYPE", py.eval("type(hd44780_mod.BACKGROUND_PRIORITY_CLOCK).__name__"), Value::str("int"));
}

#[test]
fn module_factory_has_no_load_hooks() {
    let factory = module_factory();
    assert!(factory.load_config.is_none(), "hd44780 has no load_config");
    assert!(factory.load_config_prefix.is_none(), "hd44780 has no load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "hd44780 has no add_early_printer_objects");
    assert!(factory.add_printer_objects.is_none(), "hd44780 has no add_printer_objects");
}

#[test]
fn default_construction_reads_defaults() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    check("oid", py.eval("hd.oid"), Value::Int(h.rust.oid()));
    check("line_length", py.eval("hd.line_length"), Value::Int(h.rust.line_length()));
    check("protocol", py.eval("hd.hd44780_protocol_init"), Value::Bool(h.rust.hd44780_protocol_init()));
    check("dims", py.eval("hd.get_dimensions()"), Value::Tuple(vec![Value::Int(h.rust.get_dimensions().0), Value::Int(h.rust.get_dimensions().1)]));
    check("text0", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
    check("text1", py.eval("bytes(hd.text_framebuffers[1])"), Value::Bytes(h.rust.text_framebuffers()[1].clone()));
    check("glyph", py.eval("bytes(hd.glyph_framebuffer)"), Value::Bytes(h.rust.glyph_framebuffer()));
    check("old0", py.eval("bytes(hd.all_framebuffers[0][1])"), Value::Bytes(h.rust.old_framebuffers().0[0].clone()));
    check("old1", py.eval("bytes(hd.all_framebuffers[1][1])"), Value::Bytes(h.rust.old_framebuffers().0[1].clone()));
    check("oldg", py.eval("bytes(hd.all_framebuffers[2][1])"), Value::Bytes(h.rust.old_framebuffers().1.clone()));
    let obj: ObjectRef = h.rust.clone() as ObjectRef;
    assert!(downcast_arc::<HD44780>(&obj).is_some(), "HD44780 is a printer object");
}

#[test]
fn line_length_16() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![("line_length".to_string(), "16".to_string())], vec![("line_length".to_string(), FakeOpt::Int(16))]);
    check("line_length_16", py.eval("hd.line_length"), Value::Int(h.rust.line_length()));
    check("dims_16", py.eval("hd.get_dimensions()"), Value::Tuple(vec![Value::Int(16), Value::Int(4)]));
    check("fb_len_16", py.eval("len(hd.text_framebuffers[0])"), Value::Int(h.rust.text_framebuffers()[0].len() as i64));
}

#[test]
fn invalid_line_length_raises_choice_error() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("hd44780_mod.HD44780(FakeConfig(FakePrinter(FakePins(FakeMcu())), {'rs_pin': 'PA0', 'e_pin': 'PA1', 'd4_pin': 'PA2', 'd5_pin': 'PA3', 'd6_pin': 'PA4', 'd7_pin': 'PA5', 'line_length': 24}))");
    let state = fresh_state();
    let rust_err = match build_rust(state, vec![("line_length".to_string(), FakeOpt::Int(24))], Vec::new()) {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("invalid line_length", expected, &rust_err);
}

#[test]
fn missing_pin_option_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("hd44780_mod.HD44780(FakeConfig(FakePrinter(FakePins(FakeMcu())), {'e_pin': 'PA1', 'd4_pin': 'PA2'}))");
    let state = fresh_state();
    let mcu: Mcu = Arc::new(FakeMcu { state: Arc::clone(&state) });
    let pins: ObjectRef = Arc::new(FakePins { mcu: mcu.clone(), mapping: Vec::new() });
    let printer: Printer = Arc::new(FakePrinter { pins_obj: pins });
    let config: Config = Arc::new(FakeConfig { printer, opts: vec![("e_pin".to_string(), FakeOpt::Str("PA1".to_string()))] });
    let rust_err = match HD44780::new(&config) {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("missing pin", expected, &rust_err);
}

#[test]
fn pins_on_different_mcus_raise() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("(lambda: (lambda mcu1, mcu2: hd44780_mod.HD44780(FakeConfig(FakePrinter(FakePins(mcu1, {'PA0': mcu1, 'PA1': mcu1, 'PA2': mcu1, 'PA3': mcu2, 'PA4': mcu1, 'PA5': mcu1})), {'rs_pin': 'PA0', 'e_pin': 'PA1', 'd4_pin': 'PA2', 'd5_pin': 'PA3', 'd6_pin': 'PA4', 'd7_pin': 'PA5'})))(FakeMcu(), FakeMcu()))()");
    let state = fresh_state();
    let _mcu1: Mcu = Arc::new(FakeMcu { state: Arc::clone(&state) });
    let mcu2: Mcu = Arc::new(FakeMcu { state: fresh_state() });
    let mapping = vec![("PA3".to_string(), mcu2)];
    let rust_err = match build_rust(Arc::clone(&state), vec![], mapping) {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("different mcu", expected, &rust_err);
}

#[test]
fn protocol_init_false_changes_init_stream() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![("hd44780_protocol_init".to_string(), "False".to_string())], vec![("hd44780_protocol_init".to_string(), FakeOpt::Bool(false))]);
    check("protocol_false", py.eval("hd.hd44780_protocol_init"), Value::Bool(h.rust.hd44780_protocol_init()));
}

#[test]
fn build_config_emits_config_cmd() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.build_config().expect("rust build_config");
    py.exec("hd.build_config()").expect("py build_config");
    check("config_cmds", py.eval("hd.mcu.config_cmds"), Value::List(h.state.config_cmds.lock().unwrap().iter().cloned().map(Value::Str).collect()));
    check("sends_after_build", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn config_callback_builds_commands() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    {
        let mut cbs = h.state.callbacks.lock().unwrap();
        assert_eq!(cbs.len(), 1, "one config callback");
        cbs[0]().expect("rust callback");
    }
    py.exec("hd.mcu.callbacks[0]()").expect("py callback");
    check("config_cmds_cb", py.eval("hd.mcu.config_cmds"), Value::List(h.state.config_cmds.lock().unwrap().iter().cloned().map(Value::Str).collect()));
    check("sends_cb", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn send_before_build_config_raises() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    let expected = py.eval("hd.send([0x80])");
    let rust_err = match h.rust.send(&[0x80], false) {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("send before build", expected, &rust_err);
    let expected = py.eval("hd.send(b'\\x20', is_data=True)");
    let rust_err = match h.rust.send(&[0x20], true) {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("send data before build", expected, &rust_err);
}

#[test]
fn init_before_build_config_raises() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    let expected = py.eval("hd.init()");
    let rust_err = match h.rust.init() {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("init before build", expected, &rust_err);
}

#[test]
fn flush_initial_writes_spaces_and_zeros() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.flush().expect("rust flush");
    py.exec("hd.flush()").expect("py flush");
    check("sends_initial_flush", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
    check("old0_after", py.eval("bytes(hd.all_framebuffers[0][1])"), Value::Bytes(h.rust.old_framebuffers().0[0].clone()));
    check("oldg_after", py.eval("bytes(hd.all_framebuffers[2][1])"), Value::Bytes(h.rust.old_framebuffers().1.clone()));
}

#[test]
fn init_full_streams_match() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.init().expect("rust init");
    py.exec("hd.init()").expect("py init");
    check("sends_init", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn init_without_protocol_streams_match() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![("hd44780_protocol_init".to_string(), "False".to_string())], vec![("hd44780_protocol_init".to_string(), FakeOpt::Bool(false))]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.init().expect("rust init");
    py.exec("hd.init()").expect("py init");
    check("sends_init_noproto", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn flush_after_init_is_quiet() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.init().expect("rust init");
    py.exec("hd.init()").expect("py init");
    h.rust.flush().expect("rust flush2");
    py.exec("hd.flush()").expect("py flush2");
    check("sends_quiet", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn write_text_rows_and_truncation() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    let cases = [
        (0i64, 0i64, b"Hello".to_vec()),
        (5, 0, b" World".to_vec()),
        (0, 1, b"second".to_vec()),
        (0, 2, b"third".to_vec()),
        (3, 3, b"fourth".to_vec()),
        (17, 0, b"OVERFLOW".to_vec()),
        (20, 1, b"AT-END".to_vec()),
        (25, 2, b"BEYOND".to_vec()),
        (0, 0, b"".to_vec()),
    ];
    for (x, y, data) in &cases {
        let rust_data: Vec<u8> = data.clone();
        h.rust.write_text(*x, *y, &rust_data).expect("rust write_text");
        let hex: String = rust_data.iter().map(|b| format!("{b:02x}")).collect();
        py.exec(&format!("hd.write_text({x}, {y}, bytes.fromhex('{hex}'))")).expect("py write_text");
    }
    check("text0_wt", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
    check("text1_wt", py.eval("bytes(hd.text_framebuffers[1])"), Value::Bytes(h.rust.text_framebuffers()[1].clone()));
}

#[test]
fn write_text_exact_width() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.write_text(0, 0, b"01234567890123456789").expect("rust");
    py.exec("hd.write_text(0, 0, b'01234567890123456789')").expect("py");
    h.rust.write_text(0, 1, b"01234567890123456789EXTRA").expect("rust");
    py.exec("hd.write_text(0, 1, b'01234567890123456789EXTRA')").expect("py");
    check("exact", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
    check("exact1", py.eval("bytes(hd.text_framebuffers[1])"), Value::Bytes(h.rust.text_framebuffers()[1].clone()));
}

#[test]
fn clear_restores_spaces() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.write_text(0, 0, b"dirty").expect("rust dirty");
    py.exec("hd.write_text(0, 0, b'dirty')").expect("py dirty");
    h.rust.clear();
    py.exec("hd.clear()").expect("py clear");
    check("cleared0", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
    check("cleared1", py.eval("bytes(hd.text_framebuffers[1])"), Value::Bytes(h.rust.text_framebuffers()[1].clone()));
    check("old_untouched", py.eval("bytes(hd.all_framebuffers[0][1])"), Value::Bytes(h.rust.old_framebuffers().0[0].clone()));
}

#[test]
fn flush_batches_scattered_changes() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.init().expect("rust init");
    py.exec("hd.init()").expect("py init");
    // Change pos 0 and pos 10 (gap 10 > 4, must stay split).
    h.rust.write_text(0, 0, b"X").expect("rust x");
    py.exec("hd.write_text(0, 0, b'X')").expect("py x");
    h.rust.write_text(10, 0, b"Y").expect("rust y");
    py.exec("hd.write_text(10, 0, b'Y')").expect("py y");
    h.rust.flush().expect("rust flush");
    py.exec("hd.flush()").expect("py flush");
    check("sends_sparse", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn flush_gap_of_four_merges() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.init().expect("rust init");
    py.exec("hd.init()").expect("py init");
    h.rust.write_text(0, 0, b"A").expect("rust a");
    py.exec("hd.write_text(0, 0, b'A')").expect("py a");
    h.rust.write_text(4, 0, b"B").expect("rust b");
    py.exec("hd.write_text(4, 0, b'B')").expect("py b");
    h.rust.flush().expect("rust flush");
    py.exec("hd.flush()").expect("py flush");
    check("sends_gap4", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn set_and_write_glyph_icons() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    py.exec("hd.set_glyphs({'glyph1': {'icon5x8': (3, [1, 2, 3, 4, 5, 6, 7, 8])}})").expect("py set_glyphs");
    let mut inner = IndexMap::new();
    inner.insert("icon5x8".to_string(), (3i64, vec![1, 2, 3, 4, 5, 6, 7, 8]));
    let mut glyphs = IndexMap::new();
    glyphs.insert("glyph1".to_string(), inner);
    h.rust.set_glyphs(&glyphs);
    check("icons", py.eval("list(hd.icons.items())"), Value::List(vec![Value::Tuple(vec![
        Value::Str("glyph1".to_string()),
        Value::Tuple(vec![Value::Int(3), Value::List(vec![1, 2, 3, 4, 5, 6, 7, 8].iter().map(|v| Value::Int(*v)).collect())]),
    ])]));
    h.rust.write_glyph(1, 0, "glyph1").expect("rust write_glyph");
    let py_ret = py.eval("hd.write_glyph(1, 0, 'glyph1')");
    check("glyph_ret", py_ret, Value::Int(1));
    check("glyph_fb", py.eval("bytes(hd.glyph_framebuffer)"), Value::Bytes(h.rust.glyph_framebuffer()));
    check("glyph_text", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
}

#[test]
fn write_glyph_right_arrow_and_unknown() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    let py_ret = py.eval("hd.write_glyph(2, 0, 'right_arrow')");
    let rust_ret = h.rust.write_glyph(2, 0, "right_arrow").expect("rust arrow");
    check("arrow_ret", py_ret, Value::Int(rust_ret));
    check("arrow_text", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
    let py_ret = py.eval("hd.write_glyph(5, 0, 'nope')");
    let rust_ret = h.rust.write_glyph(5, 0, "nope").expect("rust nope");
    check("unknown_ret", py_ret, Value::Int(rust_ret));
    check("unknown_text", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
}

#[test]
fn set_glyphs_ignores_non_icon5x8() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    py.exec("hd.set_glyphs({'a': {'icon16x16': (1, [1, 1])}, 'b': {}, 'c': {'icon5x8': (0, [0, 0, 0, 0, 0, 0, 0, 0])}})").expect("py set");
    let mut glyphs = IndexMap::new();
    glyphs.insert("a".to_string(), IndexMap::new());
    glyphs.insert("b".to_string(), IndexMap::new());
    let mut inner = IndexMap::new();
    inner.insert("icon5x8".to_string(), (0i64, vec![0, 0, 0, 0, 0, 0, 0, 0]));
    glyphs.insert("c".to_string(), inner);
    h.rust.set_glyphs(&glyphs);
    check("icons_filtered", py.eval("list(hd.icons.items())"), Value::List(vec![Value::Tuple(vec![
        Value::Str("c".to_string()),
        Value::Tuple(vec![Value::Int(0), Value::List(vec![Value::Int(0); 8])]),
    ])]));
}

#[test]
fn write_graphics_is_noop() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    h.rust.write_graphics(0, 0, &[1, 2, 3]).expect("rust gfx");
    py.exec("hd.write_graphics(0, 0, [1, 2, 3])").expect("py gfx");
    check("gfx_text", py.eval("bytes(hd.text_framebuffers[0])"), Value::Bytes(h.rust.text_framebuffers()[0].clone()));
    check("gfx_glyph", py.eval("bytes(hd.glyph_framebuffer)"), Value::Bytes(h.rust.glyph_framebuffer()));
    check("gfx_sends", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}

#[test]
fn glyph_bits_out_of_range_raise() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    py.exec("hd.set_glyphs({'bad': {'icon5x8': (1, [0, 0, 300, 0, 0, 0, 0, 0])}})").expect("py set");
    let mut inner = IndexMap::new();
    inner.insert("icon5x8".to_string(), (1i64, vec![0, 0, 300, 0, 0, 0, 0, 0]));
    let mut glyphs = IndexMap::new();
    glyphs.insert("bad".to_string(), inner);
    h.rust.set_glyphs(&glyphs);
    let expected = py.eval("hd.write_glyph(0, 0, 'bad')");
    let rust_err = match h.rust.write_glyph(0, 0, "bad") {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("bits out of range", expected, &rust_err);
}

#[test]
fn glyph_slot_out_of_range_raise() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    py.exec("hd.set_glyphs({'bad': {'icon5x8': (300, [0, 0, 0, 0, 0, 0, 0, 0])}})").expect("py set");
    let mut inner = IndexMap::new();
    inner.insert("icon5x8".to_string(), (300i64, vec![0, 0, 0, 0, 0, 0, 0, 0]));
    let mut glyphs = IndexMap::new();
    glyphs.insert("bad".to_string(), inner);
    h.rust.set_glyphs(&glyphs);
    let expected = py.eval("hd.write_glyph(0, 0, 'bad')");
    let rust_err = match h.rust.write_glyph(0, 0, "bad") {
        Ok(_) => panic!("rust must fail"),
        Err(e) => e,
    };
    check_err("slot out of range", expected, &rust_err);
}

#[test]
fn glyph_bits_shorter_than_eight_resizes() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![], vec![]);
    py.exec("hd.set_glyphs({'s': {'icon5x8': (0, [9, 9, 9, 9])}})").expect("py set");
    let mut inner = IndexMap::new();
    inner.insert("icon5x8".to_string(), (0i64, vec![9, 9, 9, 9]));
    let mut glyphs = IndexMap::new();
    glyphs.insert("s".to_string(), inner);
    h.rust.set_glyphs(&glyphs);
    h.rust.write_glyph(0, 0, "s").expect("rust");
    py.exec("hd.write_glyph(0, 0, 's')").expect("py");
    check("short_fb", py.eval("bytes(hd.glyph_framebuffer)"), Value::Bytes(h.rust.glyph_framebuffer()));
    check("short_len", py.eval("len(hd.glyph_framebuffer)"), Value::Int(h.rust.glyph_framebuffer().len() as i64));
}

#[test]
fn line_length_16_flush_streams() {
    let Some(mut py) = oracle() else { return };
    let h = fresh(&mut py, vec![("line_length".to_string(), "16".to_string())], vec![("line_length".to_string(), FakeOpt::Int(16))]);
    h.rust.build_config().expect("rust build");
    py.exec("hd.build_config()").expect("py build");
    h.rust.init().expect("rust init");
    py.exec("hd.init()").expect("py init");
    check("sends_16", py.eval(PY_SENDS_EXPR), sends_value(&h.state));
}
