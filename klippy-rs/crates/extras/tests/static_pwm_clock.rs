//! Parity tests for `extras::static_pwm_clock` (port of
//! `klippy/extras/static_pwm_clock.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): both sides are built with
//! mirrored minimal Python/Rust fakes for the printer, config, pins and
//! MCU, fed identical config options and `seconds_to_clock` values, and
//! the recorded construction calls, `_build_config` outcomes and log
//! lines are compared. When `$KLIPPY_PYTHON` is unset the tests skip.
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply.

use extras::static_pwm_clock::{load_config_prefix, PrinterClockOutputPin};
use klippy_api::config::{Config, ConfigApi, GetFloatOpts, GetStrOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{AsyncResponseWrapper, Chip, CommandQueue, CommandWrapper, CommandQueryWrapper, ConfigCallback, Mcu, McuApi, Pin, PinParams, PinsApi, PwmPinApi, TriggerDispatch};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::BTreeMap as Map;
use std::sync::{Arc, LazyLock, Once};

// ---------------------------------------------------------------------------
// Python-side fakes (mirrored in Rust below).
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.static_pwm_clock as spc
import logging as _logging

class Error(Exception):
    pass

class FakePwmPin:
    def __init__(self, mcu):
        self._mcu = mcu
        self.calls = []
    def get_mcu(self):
        self.calls.append(('get_mcu',))
        return self._mcu
    def setup_cycle_time(self, cycle_time, hardware_pwm):
        self.calls.append(('setup_cycle_time', cycle_time, hardware_pwm))
    def setup_max_duration(self, max_duration):
        self.calls.append(('setup_max_duration', max_duration))
    def setup_start_value(self, start_value, shutdown_value):
        self.calls.append(('setup_start_value', start_value, shutdown_value))

class FakeMcu:
    def __init__(self, freq):
        self.freq = int(freq)
        self.callbacks = []
        self.seconds_calls = []
    def seconds_to_clock(self, t):
        self.seconds_calls.append(t)
        assert t == 1.0
        return self.freq
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def get_name(self):
        return 'mcu'

class FakePins:
    def __init__(self, mcu):
        self.mcu = mcu
        self.calls = []
        self.pin = FakePwmPin(mcu)
    def setup_pin(self, pin_type, pin_desc):
        self.calls.append(('setup_pin', pin_type, pin_desc))
        assert pin_type == 'pwm'
        return self.pin

class FakePrinter:
    def __init__(self, pins_obj=None):
        self.objects = {}
        if pins_obj is not None:
            self.objects['pins'] = pins_obj
        self.config_error = Error
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        raise Error("Unknown config object '%s'" % (name,))

class FakeConfig:
    def __init__(self, printer, name, opts):
        self._printer = printer
        self._name = name
        self._opts = dict(opts)
        self.reads = []
    def get_name(self):
        return self._name
    def get_printer(self):
        return self._printer
    def get(self, option, default=None, note_valid=True):
        self.reads.append(('get', option))
        if option not in self._opts:
            raise Error(
                "Option '%s' in section '%s' must be specified" % (option, self._name))
        return self._opts[option]
    def getfloat(self, option, default=100, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        self.reads.append(('getfloat', option, default, above, maxval))
        if option not in self._opts:
            return default
        v = float(self._opts[option])
        if maxval is not None and v > maxval:
            raise Error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self._name, maxval))
        if above is not None and v <= above:
            raise Error(
                "Option '%s' in section '%s' must be above %s"
                % (option, self._name, above))
        return v

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.handlers = [_Capture()]
_root.setLevel(0)

def _make(opts, mcu_freq, section='static_pwm_clock clk'):
    mcu = FakeMcu(mcu_freq)
    pins = FakePins(mcu)
    printer = FakePrinter(pins)
    config = FakeConfig(printer, section, opts)
    return printer, config, pins, mcu

def _construct(opts, mcu_freq, section='static_pwm_clock clk'):
    printer, config, pins, mcu = _make(opts, mcu_freq, section)
    obj = spc.load_config_prefix(config)
    return (obj.name, float(obj.frequency), list(pins.calls), list(pins.pin.calls),
            [(r[0], r[1]) for r in config.reads], len(mcu.callbacks))

def _run_build(opts, mcu_freq, section='static_pwm_clock clk'):
    printer, config, pins, mcu = _make(opts, mcu_freq, section)
    obj = spc.load_config_prefix(config)
    log_lines.clear()
    obj._build_config()
    return list(log_lines)

def _construct_no_pins():
    printer, config, pins, mcu = _make({'pin': 'PA0'}, 12000000)
    printer.objects.pop('pins')
    return spc.load_config_prefix(config)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Map<String, ObjectRef>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> Self {
        Self { objects: Map::new(), reactor: Reactor::new(false) }
    }
    fn with_pins(pins: Arc<FakePins>) -> Self {
        let mut p = Self::new();
        let as_obj: ObjectRef = pins;
        p.objects.insert("pins".to_string(), as_obj);
        p
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
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
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.objects.get(name) {
            return Ok(Some(Arc::clone(o)));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
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
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    opts: Map<String, String>,
    reads: Mutex<Vec<(String, String)>>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, opts: &[(&str, &str)]) -> Self {
        let mut m = Map::new();
        for (k, v) in opts {
            m.insert(k.to_string(), v.to_string());
        }
        Self {
            printer,
            name: name.to_string(),
            opts: m,
            reads: Mutex::new(Vec::new()),
        }
    }
    fn reads(&self) -> Vec<(String, String)> {
        self.reads.lock().clone()
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.reads.lock().push(("get".to_string(), option.to_string()));
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Err(KlippyError::Config("unexpected getint".to_string()))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads
            .lock()
            .push(("getfloat".to_string(), option.to_string()));
        let raw = match self.opts.get(option) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    Some(d) => Ok(d),
                    None => Err(KlippyError::Config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    ))),
                }
            }
        };
        let v = pycompat::text::float_from_string(&raw)
            .map_err(|_| KlippyError::Config(format!("Option '{option}' in section '{}' must be specified", self.name)))?;
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    pycompat::text::trunc_float_to_bigint(max).0
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    pycompat::text::float_repr(above)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(KlippyError::Config("unexpected getboolean".to_string()))
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::Config("unexpected getchoice".to_string()))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::Config("unexpected getlists".to_string()))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::Config("unexpected getlist".to_string()))
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::Config("unexpected getintlist".to_string()))
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::Config("unexpected getfloatlist".to_string()))
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig::new(self.printer.clone(), section, &[]))
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

#[derive(Clone, Debug, PartialEq)]
enum PwmCall {
    GetMcu,
    SetupCycleTime(f64, Option<bool>),
    SetupMaxDuration(f64),
    SetupStartValue(f64, f64),
}

struct FakePwmPin {
    mcu: Mcu,
    calls: Mutex<Vec<PwmCall>>,
}

impl FakePwmPin {
    fn new(mcu: Mcu) -> Self {
        Self { mcu, calls: Mutex::new(Vec::new()) }
    }
    fn calls(&self) -> Vec<PwmCall> {
        self.calls.lock().clone()
    }
}

impl PwmPinApi for FakePwmPin {
    fn get_mcu(&self) -> Mcu {
        self.calls.lock().push(PwmCall::GetMcu);
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.calls.lock().push(PwmCall::SetupMaxDuration(max_duration));
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        self.calls.lock().push(PwmCall::SetupCycleTime(cycle_time, hardware_pwm));
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        self.calls.lock().push(PwmCall::SetupStartValue(start_value, shutdown_value));
    }
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, _print_time: f64, _value: f64) {}
}

struct FakeMcu {
    freq: i64,
    callbacks: Mutex<Vec<ConfigCallback>>,
    seconds_calls: Mutex<Vec<f64>>,
    name: String,
}

impl FakeMcu {
    fn new(freq: i64) -> Self {
        Self {
            freq,
            callbacks: Mutex::new(Vec::new()),
            seconds_calls: Mutex::new(Vec::new()),
            name: "mcu".to_string(),
        }
    }
    fn fire(&self) {
        let mut cbs = self.callbacks.lock();
        for cb in cbs.iter_mut() {
            let _ = cb();
        }
    }
    fn callback_count(&self) -> usize {
        self.callbacks.lock().len()
    }
    fn seconds_calls(&self) -> Vec<f64> {
        self.seconds_calls.lock().clone()
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        Arc::new(FakePrinter::new())
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Config("unexpected mcu setup_pin".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        self.seconds_calls.lock().push(time);
        assert_eq!(time, 1.0);
        self.freq
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
        Err(KlippyError::Config("unexpected lookup_command".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Config("unexpected lookup_query_command".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        struct Nop;
        impl klippy_api::mcu::AsyncResponseWrapperApi for Nop {
            fn unregister(&self) {}
        }
        Arc::new(Nop)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        indexmap::IndexMap::new()
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Ok(0.0)
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        0.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Config("unexpected create_trigger_dispatch".to_string()))
    }
}

struct FakePins {
    pin: Arc<FakePwmPin>,
    calls: Mutex<Vec<(String, String)>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        Err(KlippyError::Config("unexpected parse_pin".to_string()))
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        Err(KlippyError::Config("unexpected lookup_pin".to_string()))
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.calls.lock().push((pin_type.to_string(), pin_desc.to_string()));
        assert_eq!(pin_type, "pwm");
        Ok(Pin::Pwm(self.pin.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        Err(KlippyError::Config("unexpected get_pin_resolver".to_string()))
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("static_pwm_clock fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping static_pwm_clock oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.static_pwm_clock".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_error(label: &str, expected: pyoracle::OracleResult, actual: Result<(), KlippyError>) {
    match (expected, actual) {
        (Err(e), Err(a)) => {
            assert_eq!(e.class_name, a.class_name(), "{label} class");
            assert_eq!(e.message, a.message(), "{label} message");
        }
        (Ok(v), Err(a)) => panic!("{label}: oracle returned {v:?}, rust raised {}: {}", a.class_name(), a.message()),
        (Err(e), Ok(())) => panic!("{label}: oracle raised {}: {}, rust succeeded", e.class_name, e.message),
        (Ok(_), Ok(())) => panic!("{label}: both succeeded"),
    }
}

static LOG_LINES: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static LOG_ONCE: Once = Once::new();
static LOG_GUARD: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

fn log_lines() -> &'static Mutex<Vec<String>> {
    &LOG_LINES
}

fn install_sink() {
    LOG_ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            log_lines().lock().push(rec.message.to_string());
        }));
    });
}

fn take_logs() -> Vec<String> {
    install_sink();
    log_lines().lock().clone()
}

fn clear_logs() {
    install_sink();
    log_lines().lock().clear();
}
struct Rig {
    obj: Arc<PrinterClockOutputPin>,
    fake_config: Arc<FakeConfig>,
    fake_mcu: Arc<FakeMcu>,
    fake_pins: Arc<FakePins>,
    fake_pin: Arc<FakePwmPin>,
}

fn build_rig(opts: &[(&str, &str)], mcu_freq: i64, section: &str) -> KResult<Rig> {
    let fake_mcu = Arc::new(FakeMcu::new(mcu_freq));
    let mcu: Mcu = fake_mcu.clone();
    let fake_pin = Arc::new(FakePwmPin::new(mcu));
    let fake_pins = Arc::new(FakePins { pin: fake_pin.clone(), calls: Mutex::new(Vec::new()) });
    let printer: Printer = Arc::new(FakePrinter::with_pins(fake_pins.clone()));
    let fake_config = Arc::new(FakeConfig::new(printer, section, opts));
    let config: Config = fake_config.clone();
    let obj = PrinterClockOutputPin::new(&config)?;
    Ok(Rig { obj, fake_config, fake_mcu, fake_pins, fake_pin })
}

fn pwm_calls_value(calls: &[PwmCall]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|c| match c {
                PwmCall::GetMcu => Value::Tuple(vec![Value::str("get_mcu")]),
                PwmCall::SetupCycleTime(t, hw) => Value::Tuple(vec![
                    Value::str("setup_cycle_time"),
                    Value::float(*t),
                    Value::Bool(hw.unwrap_or(false)),
                ]),
                PwmCall::SetupMaxDuration(t) => {
                    Value::Tuple(vec![Value::str("setup_max_duration"), Value::float(*t)])
                }
                PwmCall::SetupStartValue(a, b) => Value::Tuple(vec![
                    Value::str("setup_start_value"),
                    Value::float(*a),
                    Value::float(*b),
                ]),
            })
            .collect(),
    )
}

fn construct_value(rig: &Rig) -> Value {
    Value::Tuple(vec![
        Value::str(rig.obj.name()),
        Value::float(rig.obj.frequency()),
        Value::List(
            rig.fake_pins
                .calls
                .lock()
                .iter()
                .map(|(t, d)| Value::Tuple(vec![Value::str("setup_pin"), Value::str(t), Value::str(d)]))
                .collect(),
        ),
        pwm_calls_value(&rig.fake_pin.calls()),
        Value::List(
            rig.fake_config.reads().iter().map(|(a, b)| Value::Tuple(vec![Value::str(a), Value::str(b)])).collect(),
        ),
        Value::int(rig.fake_mcu.callback_count() as i64),
    ])
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn construct_default_frequency() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct({'pin': 'PA0'}, 12000000)");
    let rig = build_rig(&[("pin", "PA0")], 12_000_000, "static_pwm_clock clk")
        .expect("rust build");
    check("construct_default", expected, construct_value(&rig));
}

#[test]
fn construct_explicit_frequency() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct({'pin': 'PB1', 'frequency': '1000'}, 12000000)");
    let rig = build_rig(&[("pin", "PB1"), ("frequency", "1000")], 12_000_000, "static_pwm_clock clk")
        .expect("rust build");
    check("construct_explicit", expected, construct_value(&rig));
}

#[test]
fn construct_missing_pin() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct({}, 12000000)");
    let rig = build_rig(&[], 12_000_000, "static_pwm_clock clk");
    match rig {
        Ok(_) => panic!("expected missing pin to fail"),
        Err(e) => check_error("construct_missing_pin", expected, Err(e)),
    }
}

#[test]
fn construct_frequency_below_above() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct({'pin': 'PA0', 'frequency': '3.0'}, 12000000)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "3.0")], 12_000_000, "static_pwm_clock clk");
    match rig {
        Ok(_) => panic!("expected below-above frequency to fail"),
        Err(e) => check_error("construct_frequency_below_above", expected, Err(e)),
    }
}

#[test]
fn construct_frequency_above_max() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct({'pin': 'PA0', 'frequency': '600000000'}, 12000000)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "600000000")], 12_000_000, "static_pwm_clock clk");
    match rig {
        Ok(_) => panic!("expected above-max frequency to fail"),
        Err(e) => check_error("construct_frequency_above_max", expected, Err(e)),
    }
}

#[test]
fn construct_missing_pins_object() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct_no_pins()");
    let printer: Printer = Arc::new(FakePrinter::new());
    let fake_config = Arc::new(FakeConfig::new(printer, "static_pwm_clock clk", &[("pin", "PA0")]));
    let config: Config = fake_config;
    let result = PrinterClockOutputPin::new(&config).map(|_| ());
    check_error("construct_missing_pins", expected, result);
}

#[test]
fn build_config_exact() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_run_build({'pin': 'PA0', 'frequency': '1000'}, 12000000)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "1000")], 12_000_000, "static_pwm_clock clk")
        .expect("rust build");
    let _log_guard = LOG_GUARD.lock();
    clear_logs();
    rig.obj.build_config().expect("build_config");
    let logs = take_logs();
    check("build_exact", expected, Value::List(logs.iter().map(|s| Value::str(s)).collect()));
    assert_eq!(rig.fake_mcu.seconds_calls(), vec![1.0]);
    assert!(rig.obj.build_error().is_none());
    clear_logs();
    rig.fake_mcu.fire();
    assert!(rig.obj.build_error().is_none());
}

#[test]
fn build_config_remainder() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_run_build({'pin': 'PA0', 'frequency': '123456'}, 12000000)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "123456")], 12_000_000, "static_pwm_clock clk")
        .expect("rust build");
    let result = rig.obj.build_config().map(|_| ());
    check_error("build_remainder", expected, result);
}
#[test]
fn build_config_pulse_width_low() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_run_build({'pin': 'PA0', 'frequency': '34'}, 102)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "34")], 102, "static_pwm_clock clk")
        .expect("rust build");
    let _log_guard = LOG_GUARD.lock();
    clear_logs();
    rig.obj.build_config().expect("build_config");
    let logs = take_logs();
    check("build_low", expected, Value::List(logs.iter().map(|s| Value::str(s)).collect()));
}

#[test]
fn build_config_zero_clock_zero_division() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_run_build({'pin': 'PA0', 'frequency': '100'}, 0)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "100")], 0, "static_pwm_clock clk")
        .expect("rust build");
    let result = rig.obj.build_config().map(|_| ());
    check_error("build_zero", expected, result);
}

#[test]
fn build_config_nan_frequency() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_run_build({'pin': 'PA0', 'frequency': 'nan'}, 12000000)");
    let rig = build_rig(&[("pin", "PA0"), ("frequency", "nan")], 12_000_000, "static_pwm_clock clk")
        .expect("rust build");
    let result = rig.obj.build_config().map(|_| ());
    check_error("build_nan", expected, result);
}

#[test]
fn load_config_prefix_returns_object() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_construct({'pin': 'PA0'}, 12000000)");
    let fake_mcu = Arc::new(FakeMcu::new(12_000_000));
    let mcu: Mcu = fake_mcu.clone();
    let fake_pin = Arc::new(FakePwmPin::new(mcu));
    let fake_pins = Arc::new(FakePins { pin: fake_pin.clone(), calls: Mutex::new(Vec::new()) });
    let printer: Printer = Arc::new(FakePrinter::with_pins(fake_pins.clone()));
    let fake_config = Arc::new(FakeConfig::new(printer, "static_pwm_clock clk", &[("pin", "PA0")]));
    let config: Config = fake_config;
    let obj = load_config_prefix(&config).expect("load").expect("object");
    let pin = klippy_api::downcast_arc::<PrinterClockOutputPin>(&obj).expect("downcast");
    let expected_parts = match expected {
        Ok(Value::Tuple(parts)) => parts,
        other => panic!("load_prefix: unexpected oracle result {other:?}"),
    };
    let actual = Value::Tuple(vec![
        Value::str(pin.name()),
        Value::float(pin.frequency()),
        Value::int(fake_mcu.callback_count() as i64),
    ]);
    let expected = Value::Tuple(vec![
        expected_parts[0].clone(),
        expected_parts[1].clone(),
        expected_parts[5].clone(),
    ]);
    check("load_prefix", Ok(expected), actual);
}
