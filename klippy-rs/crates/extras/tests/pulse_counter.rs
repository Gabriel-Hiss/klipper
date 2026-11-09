//! Parity tests for `extras::pulse_counter` (port of `klippy/extras/pulse_counter.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): both sides are driven with minimal
//! Python fakes for the printer, pins and MCU, given identical constructor
//! arguments and identical `counter_state` params, and the following are
//! compared exactly (class and message for errors, bit-exact floats via
//! `Value`):
//!
//! - constructor call order and MCU commands (`config_counter` /
//!   `query_counter` strings, `register_config_callback`,
//!   `register_serial_response` message and oid),
//! - `MCU_counter` callback tuples `(time, count, count_time)`,
//! - `FrequencyCounter` frequencies and external `(time, frequency)` callbacks,
//! - error paths: missing `pins`, invalid pin descriptions, MCU oid failure,
//!   malformed `counter_state` params (missing key),
//! - the factory exposes no entry points, like the Python module.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so expected values come from the oracle alone.

use extras::pulse_counter::{CounterCallback, FreqCallback, FrequencyCounter, MCU_counter};
use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, Chip, CommandQueue,
    CommandQueryWrapper, CommandWrapper, ConfigCallback, LookupPinOpts,
    LookupQueryCommandOpts, McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi,
    SerialResponseCallback, TriggerDispatch, Mcu,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::float_repr;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.pulse_counter as pc_mod
import configparser

class error(Exception):
    pass

FREQ = 1000000.0

class FakeMCU:
    name = 'mcu'
    def __init__(self):
        self.calls = []
        self.oid_count = 0
        self.config_callbacks = []
        self.config_cmds = []
        self.serial_cb = None
        self.serial = None
        self.last32 = 0
        self.clock64 = 0
        self.fail_create_oid = False
    def create_oid(self):
        self.calls.append('create_oid')
        if self.fail_create_oid:
            raise error('oid allocation failed')
        oid = self.oid_count
        self.oid_count += 1
        return oid
    def register_config_callback(self, cb):
        self.calls.append('register_config_callback')
        self.config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.calls.append('add_config_cmd is_init=%r on_restart=%r cmd=%s' % (is_init, on_restart, cmd))
        self.config_cmds.append((cmd, is_init, on_restart))
    def get_query_slot(self, oid):
        self.calls.append('get_query_slot oid=%d' % oid)
        return 5000 + oid * 100
    def seconds_to_clock(self, time):
        self.calls.append('seconds_to_clock time=%r' % time)
        return int(time * FREQ)
    def clock32_to_clock64(self, clock32):
        self.calls.append('clock32_to_clock64 clock32=%d' % clock32)
        self.clock64 += (clock32 - self.last32) & 0xffffffff
        self.last32 = clock32
        return self.clock64
    def clock_to_print_time(self, clock):
        self.calls.append('clock_to_print_time clock=%r' % float(clock))
        return clock / FREQ
    def register_serial_response(self, cb, msg, oid=None):
        self.calls.append('register_serial_response msg=%r oid=%r' % (msg, oid))
        self.serial_cb = cb
        self.serial = (msg, oid)
        return object()

class FakePins:
    def __init__(self, mcu):
        self.mcu = mcu
        self.calls = []
    def parse_pin(self, pin_desc, can_invert=False, can_pullup=False):
        desc = pin_desc.strip()
        pullup = invert = 0
        if can_pullup and (desc.startswith('^') or desc.startswith('~')):
            pullup = 1
            if desc.startswith('~'):
                pullup = -1
            desc = desc[1:].strip()
        if can_invert and desc.startswith('!'):
            invert = 1
            desc = desc[1:].strip()
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        if chip_name != 'mcu':
            raise error("Unknown pin chip name '%s'" % (chip_name,))
        if [c for c in '^~!:' if c in pin] or ''.join(pin.split()) != pin:
            fmt = ''
            if can_pullup:
                fmt += '[^~] '
            if can_invert:
                fmt += '[!] '
            raise error("Invalid pin description '%s'\nFormat is: %s[chip_name:] pin_name" % (pin_desc, fmt))
        return {'chip': self.mcu, 'chip_name': chip_name, 'pin': pin, 'invert': invert, 'pullup': pullup}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        self.calls.append('lookup_pin %r can_invert=%r can_pullup=%r share_type=%r' % (pin_desc, can_invert, can_pullup, share_type))
        return self.parse_pin(pin_desc, can_invert, can_pullup)

class FakePrinter:
    def __init__(self):
        self.mcu = FakeMCU()
        self.pins = FakePins(self.mcu)
    def lookup_object(self, name):
        if name == 'pins':
            return self.pins
        raise configparser.Error("Unknown config object '%s'" % (name,))

class NoPinsPrinter:
    def lookup_object(self, name):
        raise configparser.Error("Unknown config object '%s'" % (name,))

def counter_params(next_clock, count, count_clock):
    return {'next_clock': next_clock, 'count': count, 'count_clock': count_clock}
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct McuState {
    pub calls: Vec<String>,
    pub oid_next: i64,
    pub config_callbacks: Vec<ConfigCallback>,
    pub config_cmds: Vec<(String, bool, bool)>,
    pub serial_cb: Option<SerialResponseCallback>,
    pub serial: Option<(String, Option<i64>)>,
    pub last32: i64,
    pub clock64: i64,
    pub fail_create_oid: bool,
}

pub struct FakeMcu {
    pub printer: GCell<Option<Printer>>,
    pub state: GCell<McuState>,
}

impl FakeMcu {
    fn new() -> FakeMcu {
        FakeMcu { printer: GCell::new(None), state: GCell::new(McuState::default()) }
    }

    fn bind(&self, printer: Printer) {
        self.printer.with_mut(|p| *p = Some(printer));
    }

    fn calls(&self) -> Vec<String> {
        self.state.borrow().calls.clone()
    }

    fn log(&self, entry: String) {
        self.state.borrow_mut().calls.push(entry);
    }

    fn run_config_callbacks(&self) -> KResult<()> {
        let mut callbacks = std::mem::take(&mut self.state.borrow_mut().config_callbacks);
        for cb in callbacks.iter_mut() {
            cb()?;
        }
        self.state.borrow_mut().config_callbacks = callbacks;
        Ok(())
    }

    fn serial_callback(&self) -> Option<SerialResponseCallback> {
        self.state.borrow().serial_cb.clone()
    }

    fn serial_msg(&self) -> Option<(String, Option<i64>)> {
        self.state.borrow().serial.clone()
    }
}

struct FakeAsyncResponse;

impl AsyncResponseWrapperApi for FakeAsyncResponse {
    fn unregister(&self) {}
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.cloned().expect("FakeMcu printer bound")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin is not used by pulse_counter".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        self.log("create_oid".to_string());
        let mut state = self.state.borrow_mut();
        if state.fail_create_oid {
            return Err(KlippyError::Mcu("oid allocation failed".to_string()));
        }
        let oid = state.oid_next;
        state.oid_next += 1;
        Ok(oid)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.log("register_config_callback".to_string());
        self.state.borrow_mut().config_callbacks.push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.log(format!(
            "add_config_cmd is_init={} on_restart={} cmd={cmd}",
            py_bool(opts.is_init),
            py_bool(opts.on_restart)
        ));
        self.state
            .borrow_mut()
            .config_cmds
            .push((cmd.to_string(), opts.is_init, opts.on_restart));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, oid: i64) -> i64 {
        self.log(format!("get_query_slot oid={oid}"));
        5000 + oid * 100
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        self.log(format!("seconds_to_clock time={}", float_repr(time)));
        // Fake clock frequency is 1e6 Hz; Python `int(time * FREQ)` truncates.
        pycompat::text::trunc_float_to_bigint(time * 1_000_000.0)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        1.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Err(KlippyError::Mcu("lookup_command is not used by pulse_counter".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu(
            "lookup_query_command is not used by pulse_counter".to_string(),
        ))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.log(format!(
            "register_serial_response msg={} oid={}",
            py_str(msg),
            py_oid(oid)
        ));
        let mut state = self.state.borrow_mut();
        state.serial_cb = Some(cb);
        state.serial = Some((msg.to_string(), oid));
        Arc::new(FakeAsyncResponse)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Ok(1_000_000.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        pycompat::text::trunc_float_to_bigint(print_time * 1_000_000.0)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        self.log(format!("clock_to_print_time clock={}", float_repr(clock)));
        clock / 1_000_000.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        self.log(format!("clock32_to_clock64 clock32={clock32}"));
        let mut state = self.state.borrow_mut();
        state.clock64 += (clock32 - state.last32) & 0xffff_ffff;
        state.last32 = clock32;
        state.clock64
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 1.0))
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
        Err(KlippyError::Mcu("not used by pulse_counter".to_string()))
    }
}

fn py_bool(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

fn py_str(s: &str) -> String {
    format!("'{s}'")
}

fn py_oid(oid: Option<i64>) -> String {
    match oid {
        Some(v) => v.to_string(),
        None => "None".to_string(),
    }
}

pub struct FakePins {
    pub mcu: Mcu,
    pub calls: GCell<Vec<String>>,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl FakePins {
    fn parse(&self, pin_desc: &str, can_invert: bool, can_pullup: bool) -> KResult<PinParams> {
        self.calls.borrow_mut().push(format!(
            "lookup_pin {} can_invert={} can_pullup={} share_type=None",
            py_str(pin_desc),
            py_bool(can_invert),
            py_bool(can_pullup)
        ));
        // Mirrors `klippy/pins.py:67` `parse_pin` with `can_invert`/`can_pullup`.
        let mut desc = pin_desc.trim().to_string();
        let mut pullup: i64 = 0;
        let mut invert: i64 = 0;
        if can_pullup && (desc.starts_with('^') || desc.starts_with('~')) {
            pullup = 1;
            if desc.starts_with('~') {
                pullup = -1;
            }
            desc = desc[1..].trim().to_string();
        }
        if can_invert && desc.starts_with('!') {
            invert = 1;
            desc = desc[1..].trim().to_string();
        }
        let (chip_name, pin) = match desc.find(':') {
            None => ("mcu".to_string(), desc),
            Some(idx) => (
                desc[..idx].trim().to_string(),
                desc[idx + 1..].trim().to_string(),
            ),
        };
        if chip_name != "mcu" {
            return Err(KlippyError::Other {
                class_name: "error".to_string(),
                message: format!("Unknown pin chip name '{chip_name}'"),
            });
        }
        if pin.chars().any(|c| c == '^' || c == '~' || c == '!' || c == ':')
            || pin.chars().any(char::is_whitespace)
        {
            let mut fmt = String::new();
            if can_pullup {
                fmt.push_str("[^~] ");
            }
            if can_invert {
                fmt.push_str("[!] ");
            }
            return Err(KlippyError::Other {
                class_name: "error".to_string(),
                message: format!(
                    "Invalid pin description '{pin_desc}'\nFormat is: {fmt}[chip_name:] pin_name"
                ),
            });
        }
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name,
            pin,
            invert,
            pullup,
            share_type: None,
        })
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        self.parse(pin_desc, opts.can_invert, opts.can_pullup)
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        self.parse(pin_desc, opts.can_invert, opts.can_pullup)
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::Mcu("not used by pulse_counter".to_string()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::Mcu("not used by pulse_counter".to_string()))
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

struct FakePrinter {
    pins: Arc<FakePins>,
    with_pins: bool,
}

klippy_api::printer_object!(FakePrinter);

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
        if name == "pins" && self.with_pins {
            let pins: ObjectRef = self.pins.clone();
            return Ok(Some(pins));
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

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("pulse_counter fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping pulse_counter oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

struct Pair {
    printer: Printer,
    mcu: Arc<FakeMcu>,
    pins: Arc<FakePins>,
}

fn rust_pair() -> Pair {
    let mcu = Arc::new(FakeMcu::new());
    let mcu_ref: Mcu = mcu.clone();
    let pins = Arc::new(FakePins {
        mcu: mcu_ref,
        calls: GCell::new(Vec::new()),
    });
    let printer: Printer = Arc::new(FakePrinter {
        pins: Arc::clone(&pins),
        with_pins: true,
    });
    mcu.bind(printer.clone());
    Pair { printer, mcu, pins }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "pulse_counter".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        actual.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        actual.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

fn check_calls(label: &str, py: &mut pyoracle::Oracle, rust: &[String]) {
    let expected = py.eval("pr.mcu.calls").expect("py mcu calls");
    let actual = Value::List(rust.iter().map(|s| Value::str(s)).collect());
    check(label, Ok(expected), actual);
}

fn counter_value(next_clock: i64, count: i64, count_clock: i64) -> PyValue {
    let mut entries = IndexMap::new();
    entries.insert(PyValue::Str("next_clock".to_string()), PyValue::Int(PyInt::from_i64(next_clock)));
    entries.insert(PyValue::Str("count".to_string()), PyValue::Int(PyInt::from_i64(count)));
    entries.insert(PyValue::Str("count_clock".to_string()), PyValue::Int(PyInt::from_i64(count_clock)));
    PyValue::Dict(entries)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constructor_pins_populated() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.0015)")
        .expect("py construct");
    let pair = rust_pair();
    let rust = MCU_counter::new(&pair.printer, "PA0", 1.0, 0.0015)
        .expect("rust construct");
    check("oid", Ok(py.eval("mc._oid").expect("oid")), Value::int(rust.oid()));
    check("pin", Ok(py.eval("mc._pin").expect("pin")), Value::str(rust.pin()));
    check("pullup", Ok(py.eval("mc._pullup").expect("pullup")), Value::int(rust.pullup()));
    check("poll_ticks", Ok(py.eval("mc._poll_ticks").expect("pt")), Value::int(rust.poll_ticks()));
    check(
        "last_count",
        Ok(py.eval("mc._last_count").expect("lc")),
        Value::int(rust.last_count()),
    );
    check_calls("constructor calls", &mut py, &pair.mcu.calls());
    let py_pins = py.eval("pr.pins.calls").expect("py pins calls");
    let rust_pins = Value::List(
        pair.pins.calls.borrow().iter().map(|s| Value::str(s)).collect(),
    );
    check("pins calls", Ok(py_pins), rust_pins);
    assert_eq!(pair.mcu.serial_msg(), None);
}

#[test]
fn constructor_pullup_prefixes() {
    let Some(mut py) = oracle() else { return };
    for (pin, expected_pullup) in [("^PA1", 1i64), ("~PA2", -1i64), ("mcu:PA3", 0i64)] {
        py.exec(&format!(
            "pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, '{pin}', 0.5, 0.001)"
        ))
        .expect("py construct");
        let pair = rust_pair();
        let rust = MCU_counter::new(&pair.printer, pin, 0.5, 0.001)
            .expect("rust construct");
        check(
            &format!("pullup {pin}"),
            Ok(py.eval("mc._pullup").expect("pullup")),
            Value::int(rust.pullup()),
        );
        assert_eq!(rust.pullup(), expected_pullup);
        check_calls(&format!("pullup calls {pin}"), &mut py, &pair.mcu.calls());
    }
}

#[test]
fn build_config_commands() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, '^PA9', 2.0, 0.002)")
        .expect("py construct");
    let pair = rust_pair();
    let rust = MCU_counter::new(&pair.printer, "^PA9", 2.0, 0.002)
        .expect("rust construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    pair.mcu.run_config_callbacks().expect("rust build_config");
    check_calls("build_config calls", &mut py, &pair.mcu.calls());
    check(
        "poll_ticks after build",
        Ok(py.eval("mc._poll_ticks").expect("pt")),
        Value::int(rust.poll_ticks()),
    );
    check(
        "serial msg",
        Ok(py.eval("pr.mcu.serial[0]").expect("serial msg")),
        Value::str(&pair.mcu.serial_msg().expect("serial msg").0),
    );
    check(
        "serial oid",
        Ok(py.eval("pr.mcu.serial[1]").expect("serial oid")),
        Value::int(pair.mcu.serial_msg().expect("serial").1.expect("oid")),
    );
    let py_cmds = py.eval("pr.mcu.config_cmds").expect("py config cmds");
    let rust_cmds = Value::List(
        pair.mcu
            .state
            .borrow()
            .config_cmds
            .iter()
            .map(|(cmd, is_init, on_restart)| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    Value::Bool(*is_init),
                    Value::Bool(*on_restart),
                ])
            })
            .collect(),
    );
    check("config cmds", Ok(py_cmds), rust_cmds);
}

#[test]
fn handle_counter_state_reports_counts() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.001); calls = []; mc.setup_callback(lambda t, c, ct: calls.append((t, c, ct)))")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()")
        .expect("py build_config");
    let pair = rust_pair();
    let rust = MCU_counter::new(&pair.printer, "PA0", 1.0, 0.001)
        .expect("rust construct");
    let captured: Arc<GCell<Vec<(f64, i64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let cap = captured.clone();
    rust.setup_callback(Arc::new(move |t, c, ct| cap.borrow_mut().push((t, c, ct))) as CounterCallback);
    pair.mcu.run_config_callbacks().expect("rust build_config");
    let samples = [(1000, 100, 900), (2000, 250, 1900)];
    for (next_clock, count, count_clock) in samples {
        py.exec(&format!(
            "pr.mcu.serial_cb(counter_params({next_clock}, {count}, {count_clock}))"
        ))
        .expect("py handle");
        let cb = pair.mcu.serial_callback().expect("serial cb");
        cb(counter_value(next_clock, count, count_clock));
    }
    let expected = py.eval("calls").expect("py calls");
    let actual = Value::List(
        captured
            .borrow()
            .iter()
            .map(|(t, c, ct)| {
                Value::Tuple(vec![Value::float(*t), Value::int(*c), Value::float(*ct)])
            })
            .collect(),
    );
    check("counter callbacks", Ok(expected), actual);
    check_calls("handle calls", &mut py, &pair.mcu.calls());
}

#[test]
fn handle_before_build_config_no_poll_offset() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.005); calls = []; mc.setup_callback(lambda t, c, ct: calls.append((t, c, ct)))")
        .expect("py construct");
    let pair = rust_pair();
    let rust = MCU_counter::new(&pair.printer, "PA0", 1.0, 0.005)
        .expect("rust construct");
    let captured: Arc<GCell<Vec<(f64, i64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let cap = captured.clone();
    rust.setup_callback(Arc::new(move |t, c, ct| cap.borrow_mut().push((t, c, ct))) as CounterCallback);
    py.exec("mc._handle_counter_state(counter_params(3000, 7, 2500))")
        .expect("py handle");
    rust.handle_counter_state(&counter_value(3000, 7, 2500))
        .expect("rust handle");
    let expected = py.eval("calls").expect("py calls");
    let actual = Value::List(
        captured
            .borrow()
            .iter()
            .map(|(t, c, ct)| {
                Value::Tuple(vec![Value::float(*t), Value::int(*c), Value::float(*ct)])
            })
            .collect(),
    );
    check("pre-config callback", Ok(expected), actual);
    check_calls("pre-config calls", &mut py, &pair.mcu.calls());
}

#[test]
fn counter_overflow_continues_monotonic() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.001)")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    let pair = rust_pair();
    let rust = MCU_counter::new(&pair.printer, "PA0", 1.0, 0.001)
        .expect("rust construct");
    pair.mcu.run_config_callbacks().expect("rust build_config");
    // Firmware count near the 32-bit top, then wrapped to a small value.
    for count in [4294967290i64, 10, 20] {
        py.exec(&format!("mc._handle_counter_state(counter_params(1000, {count}, 900))"))
            .expect("py handle");
        rust.handle_counter_state(&counter_value(1000, count, 900))
            .expect("rust handle");
    }
    check(
        "wrapped last_count",
        Ok(py.eval("mc._last_count").expect("lc")),
        Value::int(rust.last_count()),
    );
    check_calls("overflow calls", &mut py, &pair.mcu.calls());
}

#[test]
fn handle_missing_key_raises_key_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); mc = pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.001)")
        .expect("py construct");
    let pair = rust_pair();
    let rust = MCU_counter::new(&pair.printer, "PA0", 1.0, 0.001)
        .expect("rust construct");
    let expected = py.eval("mc._handle_counter_state({'next_clock': 1000, 'count_clock': 900})");
    let mut entries = IndexMap::new();
    entries.insert(PyValue::Str("next_clock".to_string()), PyValue::Int(PyInt::from_i64(1000)));
    entries.insert(PyValue::Str("count_clock".to_string()), PyValue::Int(PyInt::from_i64(900)));
    let err = rust
        .handle_counter_state(&PyValue::Dict(entries))
        .expect_err("missing count must fail");
    check_err("missing count key", expected, err);
}

#[test]
fn invalid_pin_raises_pins_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter()").expect("py printer");
    let expected = py.eval("pc_mod.MCU_counter(pr, 'unknown:PA0', 1.0, 0.001)");
    let pair = rust_pair();
    let err = MCU_counter::new(&pair.printer, "unknown:PA0", 1.0, 0.001)
        .err()
        .expect("unknown chip must fail");
    check_err("unknown chip", expected, err);

    py.exec("pr = FakePrinter()").expect("py printer");
    let expected = py.eval("pc_mod.MCU_counter(pr, 'bad pin', 1.0, 0.001)");
    let pair = rust_pair();
    let err = MCU_counter::new(&pair.printer, "bad pin", 1.0, 0.001)
        .err()
        .expect("invalid pin must fail");
    check_err("invalid pin", expected, err);
}

#[test]
fn missing_pins_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = NoPinsPrinter()").expect("py printer");
    let expected = py.eval("pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.001)");
    let mcu = Arc::new(FakeMcu::new());
    let mcu_ref: Mcu = mcu.clone();
    let pins = Arc::new(FakePins { mcu: mcu_ref, calls: GCell::new(Vec::new()) });
    let printer: Printer = Arc::new(FakePrinter { pins, with_pins: false });
    mcu.bind(printer.clone());
    let err = MCU_counter::new(&printer, "PA0", 1.0, 0.001)
        .err()
        .expect("missing pins must fail");
    check_err("missing pins", expected, err);
}

#[test]
fn oid_failure_propagates() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); pr.mcu.fail_create_oid = True")
        .expect("py printer");
    let expected = py.eval("pc_mod.MCU_counter(pr, 'PA0', 1.0, 0.001)");
    let pair = rust_pair();
    pair.mcu.state.borrow_mut().fail_create_oid = true;
    let err = MCU_counter::new(&pair.printer, "PA0", 1.0, 0.001)
        .err()
        .expect("oid failure must fail");
    check_err("oid failure", expected, err);
}

#[test]
fn frequency_chain_first_sample_silent() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); fc = pc_mod.FrequencyCounter(pr, 'PA0', 1.0, 0.0015)")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    let pair = rust_pair();
    let rust = FrequencyCounter::new(&pair.printer, "PA0", 1.0, 0.0015)
        .expect("rust construct");
    pair.mcu.run_config_callbacks().expect("rust build_config");
    check(
        "initial frequency",
        Ok(py.eval("fc.get_frequency()").expect("freq")),
        Value::float(rust.get_frequency()),
    );
    py.exec("pr.mcu.serial_cb(counter_params(10000, 100, 8000))")
        .expect("py handle");
    pair.mcu.serial_callback().expect("serial cb")(
        counter_value(10000, 100, 8000),
    );
    check(
        "frequency after first sample",
        Ok(py.eval("fc.get_frequency()").expect("freq")),
        Value::float(rust.get_frequency()),
    );
    check_calls("first sample calls", &mut py, &pair.mcu.calls());
}

#[test]
fn frequency_from_two_samples() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); fc = pc_mod.FrequencyCounter(pr, 'PA0', 1.0, 0.001)")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    let pair = rust_pair();
    let rust = FrequencyCounter::new(&pair.printer, "PA0", 1.0, 0.001)
        .expect("rust construct");
    pair.mcu.run_config_callbacks().expect("rust build_config");
    for (next_clock, count, count_clock) in [(1000, 100, 900), (2000, 250, 1900), (3000, 400, 2900)] {
        py.exec(&format!(
            "pr.mcu.serial_cb(counter_params({next_clock}, {count}, {count_clock}))"
        ))
        .expect("py handle");
        pair.mcu.serial_callback().expect("serial cb")(
            counter_value(next_clock, count, count_clock),
        );
        check(
            &format!("frequency after count {count}"),
            Ok(py.eval("fc.get_frequency()").expect("freq")),
            Value::float(rust.get_frequency()),
        );
    }
    check_calls("frequency calls", &mut py, &pair.mcu.calls());
}

#[test]
fn frequency_zero_delta_time() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); fc = pc_mod.FrequencyCounter(pr, 'PA0', 1.0, 0.0)")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    let pair = rust_pair();
    let rust = FrequencyCounter::new(&pair.printer, "PA0", 1.0, 0.0)
        .expect("rust construct");
    pair.mcu.run_config_callbacks().expect("rust build_config");
    py.exec("pr.mcu.serial_cb(counter_params(1000, 100, 1000))")
        .expect("py handle 1");
    pair.mcu.serial_callback().expect("serial cb")(
        counter_value(1000, 100, 1000),
    );
    py.exec("pr.mcu.serial_cb(counter_params(2000, 100, 1000))")
        .expect("py handle 2");
    pair.mcu.serial_callback().expect("serial cb")(
        counter_value(2000, 100, 1000),
    );
    check(
        "frequency zero delta",
        Ok(py.eval("fc.get_frequency()").expect("freq")),
        Value::float(rust.get_frequency()),
    );
}

#[test]
fn frequency_external_callback() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); fc = pc_mod.FrequencyCounter(pr, 'PA0', 1.0, 0.001); freq_calls = []; fc._callback = lambda t, f: freq_calls.append((t, f))")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    let pair = rust_pair();
    let rust = FrequencyCounter::new(&pair.printer, "PA0", 1.0, 0.001)
        .expect("rust construct");
    let captured: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let cap = captured.clone();
    rust.setup_callback(Arc::new(move |t, f| cap.borrow_mut().push((t, f))) as FreqCallback);
    pair.mcu.run_config_callbacks().expect("rust build_config");
    for (next_clock, count, count_clock) in [(1000, 100, 900), (2000, 250, 1900)] {
        py.exec(&format!(
            "pr.mcu.serial_cb(counter_params({next_clock}, {count}, {count_clock}))"
        ))
        .expect("py handle");
        pair.mcu.serial_callback().expect("serial cb")(
            counter_value(next_clock, count, count_clock),
        );
    }
    let expected = py.eval("freq_calls").expect("py freq calls");
    let actual = Value::List(
        captured
            .borrow()
            .iter()
            .map(|(t, f)| Value::Tuple(vec![Value::float(*t), Value::float(*f)]))
            .collect(),
    );
    check("frequency callbacks", Ok(expected), actual);
}

#[test]
fn serial_callback_drives_frequency() {
    let Some(mut py) = oracle() else { return };
    py.exec("pr = FakePrinter(); fc = pc_mod.FrequencyCounter(pr, 'PA0', 1.0, 0.001)")
        .expect("py construct");
    py.exec("pr.mcu.config_callbacks[0]()").expect("py build_config");
    let pair = rust_pair();
    let rust = FrequencyCounter::new(&pair.printer, "PA0", 1.0, 0.001)
        .expect("rust construct");
    pair.mcu.run_config_callbacks().expect("rust build_config");
    // Drive both sides through the stored serial callback only.
    py.exec("pr.mcu.serial_cb(counter_params(1000, 50, 900)); pr.mcu.serial_cb(counter_params(2000, 150, 1900))")
        .expect("py serial cb");
    let cb = pair.mcu.serial_callback().expect("serial cb");
    cb(counter_value(1000, 50, 900));
    cb(counter_value(2000, 150, 1900));
    check(
        "frequency via serial cb",
        Ok(py.eval("fc.get_frequency()").expect("freq")),
        Value::float(rust.get_frequency()),
    );
}

#[test]
fn factory_has_no_entry_points() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("(hasattr(pc_mod, 'load_config'), hasattr(pc_mod, 'load_config_prefix'))")
        .expect("py factory attrs");
    check(
        "factory attrs",
        Ok(expected),
        Value::Tuple(vec![Value::Bool(false), Value::Bool(false)]),
    );
    let factory = extras::pulse_counter::module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}
