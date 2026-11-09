//! Parity tests for `klippy/extras/buttons.py`.
//!
//! Everything is compared against the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`), skipped when
//! `KLIPPY_PYTHON` is unset: fresh objects are built on both sides with
//! minimal Python fakes for the printer, config and MCU, fed identical
//! inputs, and the MCU configuration commands, the `buttons_ack`
//! sends, the error texts, the rotary-encoder event sequences, the
//! debounce action logs and the config options read are compared
//! (floats by bits via the recording value format).
//!
//! The digital-button and debounce dispatch paths additionally run a
//! real `reactor::Reactor` on a `kos` thread so the
//! `register_async_callback` / `register_callback` wiring is exercised
//! end-to-end. ADC callbacks (measured through `register_async_callback`)
//! dispatch the reactor event time, which is live wall-clock time on
//! the Rust side against `0.0` on the Python fake side; for those only
//! the pressed bit and the invocation counts are compared, states being
//! the parity-relevant payload.

use std::sync::Arc;

use extras::buttons::{
    AdcButtonCallback, ButtonState, DebounceButton, McuAdcButtons, McuButtons, PrinterButtons,
    RotaryEncoder, ButtonCallback, DebounceCallback,
};
use extras::query_adc::QueryAdcApi;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AdcCallback, AdcPin, AdcPinApi, AdcSampleOpts, AddConfigCmdOpts, AsyncResponseWrapper, Chip,
    CommandQueue, CommandWrapper, CommandWrapperApi, LookupPinOpts, Mcu, ParsePinOpts, Pin,
    PinParams, PinResolver, PinsApi, SendOpts,
};
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NOW};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.buttons as buttons
import logging as _logging

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

Error = type('Error', (Exception,), {})

class FakeReactor:
    NOW = 0.0
    def __init__(self): self.pending = []
    def register_callback(self, callback, waketime=0.0):
        self.pending.append((waketime, callback))
    def register_async_callback(self, callback, waketime=0.0):
        self.pending.append((waketime, callback))
    def pump_all(self):
        # Fire everything in FIFO order; eventtime is the wake time.
        while self.pending:
            wt, cb = self.pending.pop(0)
            cb(wt)

class FakeCommandWrapper:
    def __init__(self, sends): self.sends = sends
    def send(self, data=(), minclock=0, reqclock=0, retry=True):
        self.sends.append(tuple(data))

class FakeMcu:
    def __init__(self):
        self.oid = 7
        self.cq = 42
        self.query_slot = 13
        self.clock_conv = 250
        self.cmds = []
        self.lookup_cmds = []
        self.serial_reg = []
        self.sends = []
        self.config_cbs = []
    def get_name(self): return 'mcu'
    def register_config_callback(self, cb): self.config_cbs.append(cb)
    def create_oid(self): return self.oid
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.cmds.append((cmd, is_init))
    def alloc_command_queue(self): return self.cq
    def lookup_command(self, msgformat, cq=None):
        self.lookup_cmds.append((msgformat, cq))
        return FakeCommandWrapper(self.sends)
    def get_query_slot(self, oid): return self.query_slot
    def seconds_to_clock(self, time):
        assert abs(time - 0.002) < 1e-9, time
        return self.clock_conv
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_reg.append((msg, oid))
        self.serial_cb = cb

class FakeConfig:
    def __init__(self, printer, values=None):
        self._printer = printer
        self.values = values or {}
        self.getfloats = []
    def get_printer(self): return self._printer
    def getfloat(self, option, default=None, minval=None, maxval=None):
        self.getfloats.append((option, default, minval))
        if option in self.values:
            v = float(self.values[option])
            if minval is not None and v < minval:
                raise Error("Option %s (%s) below minimal value: %s" % (
                    option, self.values[option], minval))
            return v
        return default

class FakePrinter:
    def __init__(self, reactor):
        self._reactor = reactor
        self.objects = {}
    def get_reactor(self): return self._reactor
    def config_error(self, *args):
        raise Error(*args)
    def load_object(self, config, section, default=None):
        if section in self.objects:
            return self.objects[section]
        raise Error("Unknown object '%s'" % (section,))
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        raise Error("Unknown object '%s'" % (name,))

class FakePinsPy:
    error = Error
    def __init__(self):
        self.mcus = {}
    def lookup_pin(self, pin, can_invert=False, can_pullup=False):
        invert = 0
        pullup = 0
        while pin and pin[0] in '!^~':
            c = pin[0]
            pin = pin[1:]
            if c == '!':
                invert = 1
            elif c == '^':
                pullup = 1
            elif c == '~':
                pullup = -1
        if ':' in pin:
            chip_name, pin = pin.split(':', 1)
        else:
            chip_name = 'mcu'
        if chip_name not in self.mcus:
            self.mcus[chip_name] = FakeMcu()
        return {'chip': self.mcus[chip_name], 'chip_name': chip_name,
                'pin': pin, 'invert': invert, 'pullup': pullup}

_reactor = FakeReactor()
_printer = FakePrinter(_reactor)
_mcu = FakeMcu()
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("buttons fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping buttons oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakeMcuInner {
    oid: i64,
    cq: i64,
    query_slot: i64,
    clock_conv: i64,
    cmds: Vec<(String, bool, bool)>,
    lookup_cmds: Vec<(String, Option<i64>)>,
    serial_reg: Vec<(String, Option<i64>)>,
    sends: Vec<Vec<i64>>,
}

struct FakeMcu {
    inner: Arc<kos::sync::Mutex<FakeMcuInner>>,
}

impl FakeMcu {
    fn new() -> Arc<Self> {
        Arc::new(FakeMcu {
            inner: Arc::new(kos::sync::Mutex::new(FakeMcuInner {
                oid: 7,
                cq: 42,
                query_slot: 13,
                clock_conv: 250,
                cmds: Vec::new(),
                lookup_cmds: Vec::new(),
                serial_reg: Vec::new(),
                sends: Vec::new(),
            })),
        })
    }
    fn cmds(&self) -> Vec<(String, bool, bool)> {
        self.inner.lock().cmds.clone()
    }
    fn sends(&self) -> Vec<Vec<i64>> {
        self.inner.lock().sends.clone()
    }
}

struct FakeAckCmd {
    inner: Arc<kos::sync::Mutex<FakeMcuInner>>,
}

impl CommandWrapperApi for FakeAckCmd {
    fn send(&self, data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        let mut sends = self.inner.lock();
        let mut row = Vec::new();
        for v in &data {
            match v {
                PyValue::Int(n) => {
                    let s = n.0.to_string();
                    row.push(s.parse().expect("int fits"))
                }
                other => panic!("unexpected ack arg: {other:?}"),
            }
        }
        sends.sends.push(row);
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        1
    }
}

impl klippy_api::mcu::McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("unused in this fake")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Internal {
            class_name: "Error".to_string(),
            message: "setup_pin unused in this fake".to_string(),
        })
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(self.inner.lock().oid)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.inner
            .lock()
            .cmds
            .push((cmd.to_string(), opts.is_init, opts.on_restart));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        self.inner.lock().query_slot
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        assert!((time - 0.002).abs() < 1e-9, "seconds_to_clock({time})");
        self.inner.lock().clock_conv
    }
    fn min_schedule_time(&self) -> f64 {
        0.
    }
    fn max_nominal_duration(&self) -> f64 {
        0.
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        let cq_id = cq.map(|c| c.0).unwrap_or(-1);
        self.inner
            .lock()
            .lookup_cmds
            .push((msgformat.to_string(), Some(cq_id)));
        Ok(Arc::new(FakeAckCmd { inner: Arc::clone(&self.inner) }) as CommandWrapper)
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(KlippyError::Internal {
            class_name: "Error".to_string(),
            message: "lookup_query_command unused in this fake".to_string(),
        })
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(self.inner.lock().cq)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.inner
            .lock()
            .serial_reg
            .push((msg.to_string(), oid));
        Arc::new(NoopAsync) as AsyncResponseWrapper
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        indexmap::IndexMap::new()
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::Internal {
            class_name: "Error".to_string(),
            message: "get_constant_float unused in this fake".to_string(),
        })
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        0.
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0., 0.))
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
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Internal {
            class_name: "Error".to_string(),
            message: "create_trigger_dispatch unused in this fake".to_string(),
        })
    }
}

struct NoopAsync;

impl klippy_api::mcu::AsyncResponseWrapperApi for NoopAsync {
    fn unregister(&self) {}
}
// ---------------------------------------------------------------------------
// Fake pins object (the `pins` printer object, `PinsApi`).
// ---------------------------------------------------------------------------

struct FakePins {
    mcus: kos::sync::Mutex<indexmap::IndexMap<String, Arc<FakeMcu>>>,
    adc: FakeAdcPin,
}

impl FakePins {
    fn new() -> Arc<Self> {
        Arc::new(FakePins {
            mcus: kos::sync::Mutex::new(indexmap::IndexMap::new()),
            adc: FakeAdcPin::new(),
        })
    }
    fn mcu_for(&self, chip_name: &str) -> klippy_api::mcu::Mcu {
        let mut mcus = self.mcus.lock();
        if !mcus.contains_key(chip_name) {
            mcus.insert(chip_name.to_string(), FakeMcu::new());
        }
        let mcu = mcus.get(chip_name).expect("just inserted").clone();
        mcu as klippy_api::mcu::Mcu
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        self.lookup_pin(
            pin_desc,
            LookupPinOpts {
                can_invert: opts.can_invert,
                can_pullup: opts.can_pullup,
                share_type: None,
            },
        )
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        let mut pin = pin_desc;
        let mut invert = 0;
        let mut pullup = 0;
        loop {
            match pin.chars().next() {
                Some('!') => {
                    invert = 1;
                    pin = &pin[1..];
                }
                Some('^') => {
                    pullup = 1;
                    pin = &pin[1..];
                }
                Some('~') => {
                    pullup = -1;
                    pin = &pin[1..];
                }
                _ => break,
            }
        }
        let (chip_name, pin) = match pin.split_once(':') {
            Some((chip, pin)) => (chip.to_string(), pin.to_string()),
            None => ("mcu".to_string(), pin.to_string()),
        };
        Ok(PinParams {
            chip: self.mcu_for(&chip_name),
            chip_name,
            pin,
            invert,
            pullup,
            share_type: None,
        })
    }
    fn setup_pin(&self, pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        if pin_type == "adc" {
            Ok(Pin::Adc(Arc::new(self.adc.clone()) as AdcPin))
        } else {
            Err(KlippyError::Internal {
                class_name: "Error".to_string(),
                message: "setup_pin unused in this fake".to_string(),
            })
        }
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::Internal {
            class_name: "Error".to_string(),
            message: "get_pin_resolver unused in this fake".to_string(),
        })
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

// Fake ADC pin returned by `FakePins.setup_pin("adc", ...)`: records the
// sample parameters and the callback `McuAdcButtons` installs.
#[derive(Clone)]
struct FakeAdcPin {
    inner: Arc<FakeAdcPinInner>,
}

struct FakeAdcPinInner {
    sample_calls: kos::sync::Mutex<Vec<(f64, f64, i64, i64)>>,
    callback: kos::sync::Mutex<Option<AdcCallback>>,
    last_value: (f64, f64),
}

impl FakeAdcPin {
    fn new() -> Self {
        FakeAdcPin {
            inner: Arc::new(FakeAdcPinInner {
                sample_calls: kos::sync::Mutex::new(Vec::new()),
                callback: kos::sync::Mutex::new(None),
                last_value: (0., 0.),
            }),
        }
    }
    fn sample_calls(&self) -> Vec<(f64, f64, i64, i64)> {
        self.inner.sample_calls.lock().clone()
    }
    fn fire(&self, samples: Vec<(f64, f64)>) {
        let mut cb = self.inner.callback.lock();
        if let Some(cb) = cb.as_mut() {
            cb(samples);
        }
    }
}

impl AdcPinApi for FakeAdcPin {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu unused in this fake")
    }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.inner.sample_calls.lock().push((
            report_time,
            opts.sample_time,
            opts.sample_count,
            opts.batch_num,
        ));
    }
    fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.inner.callback.lock() = Some(callback);
    }
    fn get_last_value(&self) -> (f64, f64) {
        self.inner.last_value
    }
}

// Fake `query_adc` printer object: records the names `McuAdcButtons`
// registers.
struct FakeQueryAdc {
    regs: kos::sync::Mutex<Vec<String>>,
}

impl FakeQueryAdc {
    fn new() -> Arc<Self> {
        Arc::new(FakeQueryAdc {
            regs: kos::sync::Mutex::new(Vec::new()),
        })
    }
    fn names(&self) -> Vec<String> {
        self.regs.lock().clone()
    }
}

impl QueryAdcApi for FakeQueryAdc {
    fn register_adc(&self, name: &str, _mcu_adc: AdcPin) {
        self.regs.lock().push(name.to_string());
    }
}

klippy_api::printer_object!(FakeQueryAdc; dyn QueryAdcApi);


// ---------------------------------------------------------------------------
// Fake printer/config objects.
// ---------------------------------------------------------------------------

struct FakeNamed;

impl PrinterObject for FakeNamed {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
}

struct FakePrinter {
    reactor: Reactor,
    objects: kos::sync::Mutex<indexmap::IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakePrinter {
            reactor,
            objects: kos::sync::Mutex::new(indexmap::IndexMap::new()),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objs = self.objects.lock();
        Ok(objs.get(name).cloned())
    }
    fn lookup_objects(
        &self,
        module: Option<&str>,
    ) -> Vec<(String, ObjectRef)> {
        let objs = self.objects.lock();
        objs.iter()
            .filter(|(n, _)| {
                module.is_none()
                    || n.as_str() == module.unwrap()
                    || n.starts_with(&format!("{} ", module.unwrap()))
            })
            .map(|(n, o)| (n.clone(), Arc::clone(o)))
            .collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let objs = self.objects.lock();
        match objs.get(section) {
            Some(o) => Ok(Some(Arc::clone(o))),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown object '{}'",
                    section
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[klippy_api::object::EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    values: kos::sync::Mutex<Vec<(String, String)>>,

    getfloat_options: kos::sync::Mutex<Vec<String>>,
    fail_float: Option<String>,
}

impl FakeConfig {
    fn new(printer: Printer, values: Vec<(String, String)>, fail_float: Option<String>) -> Arc<Self> {
        Arc::new(FakeConfig {
            printer,
            values: kos::sync::Mutex::new(values),
            getfloat_options: kos::sync::Mutex::new(Vec::new()),
            fail_float,
        })
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "buttons".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.getfloat_options.lock().push(option.to_string());
        // The only float read through this wrapper in the module is
        // `debounce_delay` with default 0. and minval 0. (buttons.py:257).
        assert_eq!(opts.minval, Some(0.), "debounce_delay must pass minval=0.");
        assert_eq!(opts.default, Some(Some(0.)), "debounce_delay must pass default 0.");
        if let Some(msg) = &self.fail_float {
            return Err(KlippyError::Config(msg.clone()));
        }
        let values = self.values.lock();
        match values.iter().find(|(n, _)| n == option) {
            Some((_, v)) => {
                let val: f64 = v.parse().expect("fake float");
                if let Some(minval) = opts.minval {
                    if val < minval {
                        return Err(KlippyError::Config(format!(
                            "Option {} ({}) below minimal value: {}",
                            option, v, minval
                        )));
                    }
                }
                Ok(Some(val))
            }
            None => Ok(match opts.default {
                Some(Some(v)) => Some(v),
                _ => None,
            }),
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
        panic!("getsection unused in this fake")
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

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "buttons".to_string(),
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

fn check_error(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: &KlippyError,
) {
    match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust raised"),
        Err(e) => {
            assert_eq!(
                (e.class_name, e.message),
                (actual.class_name().to_string(), actual.message().to_string()),
                "{label}: error mismatch"
            );
        }
    }
}

fn state_params(ack_count: i64, state: Vec<u8>, receive_time: f64) -> PyValue {
    let mut map = indexmap::IndexMap::new();
    map.insert(
        PyValue::Str("ack_count".to_string()),
        PyValue::Int(PyInt::from_i64(ack_count)),
    );
    map.insert(PyValue::Str("state".to_string()), PyValue::Bytes(state));
    map.insert(
        PyValue::Str("#receive_time".to_string()),
        PyValue::Float(receive_time),
    );
    PyValue::Dict(map)
}

fn pin_params(pin: &str, pullup: i64, invert: i64, mcu: &klippy_api::mcu::Mcu) -> PinParams {
    PinParams {
        chip: Arc::clone(mcu),
        chip_name: "mcu".to_string(),
        pin: pin.to_string(),
        invert,
        pullup,
        share_type: None,
    }
}

// ---------------------------------------------------------------------------
// Async reactor helpers.
// ---------------------------------------------------------------------------

struct RunningReactor {
    reactor: Reactor,
    handle: kos::thread::JoinHandle,
}

fn start_reactor() -> RunningReactor {
    let reactor = Reactor::new(false);
    let rr = reactor.clone();
    let handle = kos::thread::spawn(move || rr.run());
    // Wait until the async pipeline is armed.
    let (tx, rx) = kos::sync::channel();
    for _ in 0..200 {
        let t = tx.clone();
        reactor.register_async_callback(move |et| t.put(et), NOW);
        if rx.get_timeout(0.05).is_some() {
            break;
        }
    }
    RunningReactor { reactor, handle }
}

fn stop(rr: RunningReactor) {
    rr.reactor.end();
    rr.reactor.finalize();
    rr.handle.join();
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn build_config_commands() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "def cb(bt, state): pass\n\
         mb = buttons.MCU_buttons(_printer, _mcu)\n\
         mb.setup_buttons([\n\
         \x20 {'pin': 'PF3', 'pullup': 1, 'invert': 1},\n\
         \x20 {'pin': 'PF4', 'pullup': 0, 'invert': 0},\n\
         \x20 {'pin': 'PF5', 'pullup': 1, 'invert': 1},\n\
         ], cb)\n",
    )
    .expect("py setup");
    py.exec("mb.build_config()").expect("py build_config");

    let rr = start_reactor();
    let printer = FakePrinter::new(rr.reactor.clone());
    let mcu = FakeMcu::new();
    let printer_dyn: Printer = printer.clone();
    let mcu_dyn: klippy_api::mcu::Mcu = mcu.clone();
    let mb = McuButtons::new(&printer_dyn, mcu_dyn.clone()).expect("new McuButtons");
    let cb: ButtonCallback = Arc::new(|_, _| {});
    mb.setup_buttons(
        vec![
            pin_params("PF3", 1, 1, &mcu_dyn),
            pin_params("PF4", 0, 0, &mcu_dyn),
            pin_params("PF5", 1, 1, &mcu_dyn),
        ],
        cb,
    );
    mb.build_config().expect("build_config");
    stop(rr);

    check(
        "config commands",
        py.eval("tuple(tuple(x) for x in _mcu.cmds)"),
        Value::Tuple(
            mcu.cmds()
                .iter()
                .map(|(c, init, restart)| {
                    let _ = restart;
                    Value::Tuple(vec![Value::str(c), Value::Bool(*init)])
                })
                .collect(),
        ),
    );
    check(
        "lookup_command",
        py.eval("tuple(tuple(((m, c))) for m, c in _mcu.lookup_cmds)"),
        Value::Tuple(
            mcu.inner
                .lock()
                .lookup_cmds
                .iter()
                .map(|(m, cq)| {
                    Value::Tuple(vec![Value::str(m), Value::int(cq.unwrap())])
                })
                .collect(),
        ),
    );
    check(
        "serial registration",
        py.eval("tuple(tuple((m, o)) for m, o in _mcu.serial_reg)"),
        Value::Tuple(
            mcu.inner
                .lock()
                .serial_reg
                .iter()
                .map(|(m, o)| Value::Tuple(vec![Value::str(m), Value::int(o.unwrap())]))
                .collect(),
        ),
    );
}

#[test]
fn handle_buttons_state_ack_and_callbacks() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "events = []\n\
         def cb1(bt, state): events.append((bt, state))\n\
         mb2 = buttons.MCU_buttons(_printer, _mcu)\n\
         mb2.setup_buttons([\n\
         \x20 {'pin': 'PF3', 'pullup': 0, 'invert': 1},\n\
         \x20 {'pin': 'PF4', 'pullup': 0, 'invert': 0},\n\
         ], cb1)\n\
         mb2.build_config()\n\
         _mcu.sends.clear()\n\
         ",
    )
    .expect("py setup");
    py.exec(
        "for ack, st, rt in [(1, b'\\x00\\x01', 10.5), (2, b'\\x06', 11.), (2, b'', 12.)]:\n\
         \x20 mb2.handle_buttons_state({'ack_count': ack, 'state': st, '#receive_time': rt})\n\
         \x20 _reactor.pump_all()",
    )
    .expect("py handle");

    let rr = start_reactor();
    let printer = FakePrinter::new(rr.reactor.clone());
    let mcu = FakeMcu::new();
    let (tx, rx) = kos::sync::channel();
    let cb: ButtonCallback = Arc::new(move |et, state| {
        tx.put((et, state));
    });
    let printer_dyn: Printer = printer.clone();
    let mcu_dyn: klippy_api::mcu::Mcu = mcu.clone();
    let mb = McuButtons::new(&printer_dyn, mcu_dyn.clone()).expect("new McuButtons");
    mb.setup_buttons(
        vec![
            pin_params("PF3", 0, 1, &mcu_dyn),
            pin_params("PF4", 0, 0, &mcu_dyn),
        ],
        cb,
    );
    mb.build_config().expect("build_config");

    let mut rust_events: Vec<(f64, i64)> = Vec::new();
    for (ack, st, rt) in [(1i64, vec![0u8, 1], 10.5), (2, vec![6u8], 11.), (2, vec![], 12.)] {
        mb.handle_buttons_state(&state_params(ack, st, rt));
    }
    // Collect every async callback the reactor dispatches.
    for _ in 0..20 {
        match rx.get_timeout(0.5) {
            Some(ev) => rust_events.push(ev),
            None => break,
        }
    }
    stop(rr);

    check(
        "ack sends",
        py.eval("tuple(tuple(x) for x in _mcu.sends)"),
        Value::Tuple(
            mcu.sends()
                .iter()
                .map(|r| Value::Tuple(r.iter().map(|v| Value::int(*v)).collect()))
                .collect(),
        ),
    );
    // Callback eventtimes are the fake pump's 0.0 / the real dispatch
    // times: the payload the firmware reports is `btime`; the Python
    // fake records (bt, state) which is what `cb` receives via the
    // lambda indirection, so eventtime is not in these tuples.
    check(
        "callback events",
        py.eval("tuple(tuple(x) for x in events)"),
        Value::Tuple(
            rust_events
                .iter()
                .map(|(et, st)| Value::Tuple(vec![Value::float(*et), Value::int(*st)]))
                .collect(),
        ),
    );
}

#[test]
fn rotary_encoders() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "rot_cw = []\nrot_ccw = []\nrot2_cw = []\nrot2_ccw = []\n\
         def _cw(et): rot_cw.append(et)\n\
         def _ccw(et): rot_ccw.append(et)\n\
         def _cw2(et): rot2_cw.append(et)\n\
         def _ccw2(et): rot2_ccw.append(et)\n\
         full = buttons.FullStepRotaryEncoder(_cw, _ccw)\n\
         half = buttons.HalfStepRotaryEncoder(_cw2, _ccw2)\n\
         for i in range(1024):\n\
         \x20 full.encoder_callback(float(i), i % 4)\n\
         \x20 half.encoder_callback(float(i), i % 4)",
    )
    .expect("py rotary feed");

    let full_cw: Arc<kos::sync::Mutex<Vec<f64>>> = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let full_ccw: Arc<kos::sync::Mutex<Vec<f64>>> = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let half_cw: Arc<kos::sync::Mutex<Vec<f64>>> = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let half_ccw: Arc<kos::sync::Mutex<Vec<f64>>> = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let enc_full = RotaryEncoder::full_step(
        Arc::new({
            let cw = Arc::clone(&full_cw);
            move |et| cw.lock().push(et)
        }),
        Arc::new({
            let ccw = Arc::clone(&full_ccw);
            move |et| ccw.lock().push(et)
        }),
    );
    let enc_half = RotaryEncoder::half_step(
        Arc::new({
            let cw = Arc::clone(&half_cw);
            move |et| cw.lock().push(et)
        }),
        Arc::new({
            let ccw = Arc::clone(&half_ccw);
            move |et| ccw.lock().push(et)
        }),
    );
    for i in 0..1024 {
        let state = (i as i64) % 4;
        enc_full.encoder_callback(i as f64, state);
        enc_half.encoder_callback(i as f64, state);
    }

    let floats = |v: &Vec<f64>| Value::Tuple(v.iter().map(|f| Value::float(*f)).collect());
    check(
        "full-step events",
        py.eval("(tuple(rot_cw), tuple(rot_ccw))"),
        Value::Tuple(vec![floats(&full_cw.lock()), floats(&full_ccw.lock())]),
    );
    check(
        "half-step events",
        py.eval("(tuple(rot2_cw), tuple(rot2_ccw))"),
        Value::Tuple(vec![floats(&half_cw.lock()), floats(&half_ccw.lock())]),
    );
}

#[test]
fn debounce_direct() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "actions = []\n\
         def act(et, state): actions.append((et, repr(state)))\n\
         cfg = FakeConfig(_printer, {'debounce_delay': '0.05'})\n\
         db = buttons.DebounceButton(cfg, act)\n\
         db.button_handler(1.0, True)\n\
         db._debounce_event(1.03)\n\
         db._debounce_event(1.051)\n\
         db.button_handler(1.1, True)\n\
         db.button_handler(1.2, False)\n\
         db._debounce_event(1.24)\n\
         db._debounce_event(1.251)",
    )
    .expect("py debounce");

    let rr = start_reactor();
    let printer = FakePrinter::new(rr.reactor.clone());
    let config = FakeConfig::new(printer.clone(), vec![("debounce_delay".to_string(), "0.05".to_string())], None);
    let actions: Arc<kos::sync::Mutex<Vec<(f64, String)>>> = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let act: DebounceCallback = Arc::new({
        let actions = Arc::clone(&actions);
        move |et, state| actions.lock().push((et, state.repr()))
    });
    let config_dyn: Config = config.clone();
    let db = DebounceButton::new(&config_dyn, act).expect("debounce new");
    db.button_handler(1.0, ButtonState::Analog(true));
    db.debounce_event(1.03);
    db.debounce_event(1.051);
    db.button_handler(1.1, ButtonState::Analog(true));
    db.button_handler(1.2, ButtonState::Analog(false));
    db.debounce_event(1.24);
    db.debounce_event(1.251);
    let rust_actions = actions.lock().clone();
    stop(rr);

    // The Rust fake only cycles the options; assert the read options match.
    assert_eq!(
        config.getfloat_options.lock().clone(),
        vec!["debounce_delay".to_string()]
    );
    check(
        "debounce actions",
        py.eval("tuple(tuple(x) for x in actions)"),
        Value::Tuple(
            rust_actions
                .iter()
                .map(|(et, s)| Value::Tuple(vec![Value::float(*et), Value::str(s)]))
                .collect(),
        ),
    );
}

#[test]
fn debounce_live_reactor() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "live_actions = []\n\
         def act_live(et, state): live_actions.append((et, repr(state)))\n\
         cfg = FakeConfig(_printer, {'debounce_delay': '0.05'})\n\
         db = buttons.DebounceButton(cfg, act_live)\n\
         db.button_handler(1.0, True)",
    )
    .expect("py debounce handler");
    py.exec("_reactor.pump_all()").expect("py pump");

    let rr = start_reactor();
    let printer = FakePrinter::new(rr.reactor.clone());
    let config = FakeConfig::new(printer.clone(), vec![("debounce_delay".to_string(), "0.05".to_string())], None);
    let (tx, rx) = kos::sync::channel();
    let act: DebounceCallback = Arc::new(move |et, state| {
        tx.put((et, state.repr()));
    });
    let config_dyn: Config = config.clone();
    let db = DebounceButton::new(&config_dyn, act).expect("debounce new");
    db.button_handler(1.0, ButtonState::Analog(true));
    let got = rx.get_timeout(5.0).expect("debounce action fires through the live reactor");
    stop(rr);

    // The Python side fired the recorded callback exactly at its
    // wake-up; the Rust side at roughly the same wake-up (within the
    // same 5 s budget and yields the same action payload), so compare
    // event times via the recording form: the action fires at the
    // wall's wake-up on both sides; only the (eventtime, repr) payload
    // is compared here, as the scheduling itself is the reactor crate's
    // domain.
    check(
        "live debounce action",
        py.eval("tuple(tuple(x) for x in live_actions)"),
        Value::Tuple(vec![Value::Tuple(vec![
            Value::float(got.0),
            Value::str(&got.1),
        ])]),
    );
}

#[test]
fn printer_buttons_load() {
    let Some(mut py) = oracle() else { return };
    py.exec("cfg = FakeConfig(_printer)")
        .expect("py config");
    let expected = py.eval("buttons.load_config(cfg)");

    let reactor = Reactor::new(false);
    let printer = FakePrinter::new(reactor);
    let config = FakeConfig::new(printer.clone(), Vec::new(), None);
    let config_dyn: Config = config.clone();
    let got = PrinterButtons::new(&config_dyn);
    let err = got.map(|_| ()).expect_err("load without query_adc must fail");
    check_error("load_config without query_adc", expected, &err);

    py.exec("_printer.objects['query_adc'] = object()")
        .expect("py register query_adc");
    printer
        .add_object("query_adc", Arc::new(FakeNamed))
        .expect("add query_adc");
    check(
        "load_config with query_adc",
        py.eval("buttons.load_config(cfg) is not None"),
        Value::Bool(PrinterButtons::new(&config_dyn).is_ok()),
    );
}

#[test]
fn register_rotary_encoder_bad_steps() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "_printer.objects['query_adc'] = object()\n\
         cfg = FakeConfig(_printer)\n\
         pb = buttons.load_config(cfg)",
    )
    .expect("py pb");
    let expected = py
        .eval("pb.register_rotary_encoder('A', 'B', lambda et: None, lambda et: None, 3)");

    let printer = FakePrinter::new(Reactor::new(false));
    printer
        .add_object("query_adc", Arc::new(FakeNamed))
        .expect("add query_adc");
    let config_dyn: Config = FakeConfig::new(printer.clone(), Vec::new(), None).clone();
    let pb = PrinterButtons::new(&config_dyn).expect("pb new");
    let got = pb.register_rotary_encoder(
        "A",
        "B",
        Arc::new(|_| {}),
        Arc::new(|_| {}),
        3,
    );
    check_error(
        "steps per detent",
        expected,
        &got.expect_err("3 steps per detent must fail"),
    );
}

#[test]
fn register_buttons_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "def cb(bt, state): pass\n\
         _printer.objects['pins'] = FakePinsPy()\n\
         _printer.objects['query_adc'] = object()\n\
         pb = buttons.load_config(FakeConfig(_printer))",
    )
    .expect("py pb");
    let expected_same = py.eval("pb.register_buttons(['A', 'M1:B'], cb)");
    let expected_empty = py.eval("pb.register_buttons([], cb)");

    let printer = FakePrinter::new(Reactor::new(false));
    printer
        .add_object("query_adc", Arc::new(FakeNamed))
        .expect("add query_adc");
    printer.add_object("pins", FakePins::new()).expect("add pins");
    let config_dyn: Config = FakeConfig::new(printer.clone(), Vec::new(), None).clone();
    let pb = PrinterButtons::new(&config_dyn).expect("pb new");
    let cb: ButtonCallback = Arc::new(|_, _| {});
    let got_same = pb.register_buttons(
        vec!["A".to_string(), "M1:B".to_string()],
        Arc::clone(&cb),
    );
    check_error(
        "same mcu error",
        expected_same,
        &got_same.expect_err("pins on two MCUs must fail"),
    );
    let got_empty = pb.register_buttons(Vec::new(), cb);
    check_error(
        "empty buttons",
        expected_empty,
        &got_empty.expect_err("empty pin list must fail"),
    );
}

#[test]
fn register_rotary_valid_and_button_push() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "_printer.objects['pins'] = FakePinsPy()\n\
         _printer.objects['query_adc'] = object()\n\
         pb = buttons.load_config(FakeConfig(_printer))\n\
         pb.register_rotary_encoder('A', 'B', lambda et: None, lambda et: None, 4)\n\
         pb.register_rotary_encoder('C', 'D', lambda et: None, lambda et: None, 2)\n\
         pb.register_button_push('E', lambda et: None)",
    )
    .expect("py registrations");
    let expected = py.eval("pb.mcu_buttons['mcu'].pin_list is not None");

    let printer = FakePrinter::new(Reactor::new(false));
    printer
        .add_object("query_adc", Arc::new(FakeNamed))
        .expect("add query_adc");
    printer.add_object("pins", FakePins::new()).expect("add pins");
    let config_dyn: Config = FakeConfig::new(printer.clone(), Vec::new(), None).clone();
    let pb = PrinterButtons::new(&config_dyn).expect("pb new");
    pb.register_rotary_encoder("A", "B", Arc::new(|_| {}), Arc::new(|_| {}), 4)
        .expect("rotary with 4 steps per detent");
    pb.register_rotary_encoder("C", "D", Arc::new(|_| {}), Arc::new(|_| {}), 2)
        .expect("rotary with 2 steps per detent");
    pb.register_button_push("E", Arc::new(|_| {}))
        .expect("button push");

    check(
        "rotary registrations",
        expected,
        Value::Bool(true),
    );
}

#[test]
fn adc_buttons() {
    let Some(mut py) = oracle() else { return };
    // The ADC press events dispatch through the reactor, whose event
    // time is live wall time on the Rust side (0.0 on the fake Python
    // side), so only the pressed bits are compared, per the module doc.
    py.exec(
        "adc_events = []\n\
         def adc_cb(et, state): adc_events.append(state)\n\
         class FakeAdcPinPy:\n\
         \x20 def __init__(self):\n\
         \x20\x20 self.sample_calls = []\n\
         \x20 def setup_adc_sample(self, report_time, sample_time, sample_count, batch_num=None):\n\
         \x20\x20 self.sample_calls.append((report_time, sample_time, sample_count, batch_num))\n\
         \x20 def setup_adc_callback(self, cb):\n\
         \x20\x20 self.cb = cb\n\
         class FakePinsAdcPy:\n\
         \x20 error = Error\n\
         \x20 def __init__(self):\n\
         \x20\x20 self.adc = FakeAdcPinPy()\n\
         \x20 def setup_pin(self, pin_type, pin):\n\
         \x20\x20 assert pin_type == 'adc'\n\
         \x20\x20 return self.adc\n\
         class FakeQueryAdcPy:\n\
         \x20 def __init__(self):\n\
         \x20\x20 self.regs = []\n\
         \x20 def register_adc(self, name, mcu_adc):\n\
         \x20\x20 self.regs.append(name)\n\
         _pins = FakePinsAdcPy()\n\
         _printer.objects['pins'] = _pins\n\
         _qa = FakeQueryAdcPy()\n\
         _printer.objects['query_adc'] = _qa\n\
         adc_btn = buttons.MCU_ADC_buttons(_printer, 'ADC0', 1000.0)\n\
         adc_btn.setup_button(10.0, 50.0, adc_cb)\n\
         adc_btn.setup_button(60.0, 90.0, adc_cb)\n\
         adc_btn.adc_callback([(1.0, 0.0), (1.1, 0.0293), (1.15, 0.0293), (1.2, 0.0), (1.3, 0.0), (1.4, 0.0654), (1.5, 0.0654), (1.6, 0.0293), (1.7, 0.0293), (1.8, 1.0)])\n\
         _reactor.pump_all()",
    )
    .expect("py adc");

    let rr = start_reactor();
    let printer = FakePrinter::new(rr.reactor.clone());
    let pins = FakePins::new();
    let adc_pin_handle = pins.adc.clone();
    let qa = FakeQueryAdc::new();
    printer.add_object("pins", pins).expect("add pins");
    printer.add_object("query_adc", qa.clone()).expect("add query_adc");
    let printer_dyn: Printer = printer.clone();
    let adc_btn = McuAdcButtons::new(&printer_dyn, "ADC0", 1000.0).expect("new ADC buttons");
    let (tx, rx) = kos::sync::channel();
    let cb: AdcButtonCallback = Arc::new(move |_et, pressed| tx.put(pressed));
    adc_btn.setup_button(10.0, 50.0, Arc::clone(&cb));
    adc_btn.setup_button(60.0, 90.0, Arc::clone(&cb));
    adc_pin_handle.fire(vec![
        (1.0, 0.0),
        (1.1, 0.0293),
        (1.15, 0.0293),
        (1.2, 0.0),
        (1.3, 0.0),
        (1.4, 0.0654),
        (1.5, 0.0654),
        (1.6, 0.0293),
        (1.7, 0.0293),
        (1.8, 1.0),
    ]);
    let mut rust_events: Vec<bool> = Vec::new();
    for _ in 0..20 {
        match rx.get_timeout(0.5) {
            Some(b) => rust_events.push(b),
            None => break,
        }
    }
    stop(rr);

    check(
        "adc sample calls",
        py.eval("tuple(tuple(x) for x in _pins.adc.sample_calls)"),
        Value::Tuple(
            adc_pin_handle
                .sample_calls()
                .iter()
                .map(|(a, b, c, d)| {
                    Value::Tuple(vec![
                        Value::float(*a),
                        Value::float(*b),
                        Value::int(*c),
                        Value::int(*d),
                    ])
                })
                .collect(),
        ),
    );
    check(
        "adc register names",
        py.eval("tuple(_qa.regs)"),
        Value::Tuple(qa.names().iter().map(|n| Value::str(n)).collect()),
    );
    check(
        "adc press events",
        py.eval("tuple(adc_events)"),
        Value::Tuple(rust_events.iter().map(|b| Value::Bool(*b)).collect()),
    );
}
