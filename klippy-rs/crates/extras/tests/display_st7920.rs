//! Parity tests for `extras::display::st7920` (port of
//! `klippy/extras/display/st7920.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). `DisplayBase` is driven with a
//! Python-installed `send` recorder on both sides with identical writes
//! and glyphs; `ST7920` and `EmulatedST7920` are built with minimal
//! Python fakes for the printer, config, pins and MCU fed identical
//! config, then the MCU traffic (config commands, command sends, pin
//! updates) and the framebuffer state are compared bit-for-bit through
//! the recording value format.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::display::st7920::{DisplayBase, DisplayChip, EmulatedSt7920, St7920};
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatOpts,
    GetFloatListOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueue, CommandWrapper,
    CommandWrapperApi, LookupPinOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver,
    PinResolverApi, PinsApi, QuerySendOpts, SendOpts,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use pycompat::{PyInt, PyValue};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use reactor::Reactor;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.display.st7920 as st7920
import logging as _logging

class Error(Exception):
    pass

class error(Exception):
    pass

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

_MISSING = object()

class FakeReactor:
    def monotonic(self):
        return 100.0

class FakePins:
    error = error
    def __init__(self, mcu, mcu2):
        self.chips = {'mcu': mcu, 'other': mcu2}
        self._active = {}
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
        if chip_name not in self.chips:
            raise error("Unknown pin chip name '%s'" % (chip_name,))
        if [c for c in '^~!:' if c in pin] or ''.join(pin.split()) != pin:
            fmt = ''
            if can_pullup:
                fmt += '[^~] '
            if can_invert:
                fmt += '[!] '
            raise error("Invalid pin description '%s'\nFormat is: %s[chip_name:] pin_name" % (
                pin_desc, fmt))
        return {'chip': self.chips[chip_name], 'chip_name': chip_name,
                'pin': pin, 'invert': invert, 'pullup': pullup}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        pin_params = self.parse_pin(pin_desc, can_invert, can_pullup)
        pin = pin_params['pin']
        share_name = "%s:%s" % (pin_params['chip_name'], pin)
        if share_name in self._active:
            share_params = self._active[share_name]
            if share_type is None or share_type != share_params['share_type']:
                raise error("pin %s used multiple times in config" % (pin,))
            elif (pin_params['invert'] != share_params['invert']
                    or pin_params['pullup'] != share_params['pullup']):
                raise error("Shared pin %s must have same polarity" % (pin,))
            return share_params
        pin_params['share_type'] = share_type
        self._active[share_name] = pin_params
        return pin_params

class FakeCmd:
    def __init__(self, mcu, fmt, cq):
        self._mcu = mcu
        self._fmt = fmt
        self._cq = cq
    def send(self, data=(), minclock=0, reqclock=0):
        oid, payload = data
        self._mcu.sends.append((self._fmt, oid, bytes(payload), minclock, reqclock))

class FakeQueryCmd:
    def __init__(self, mcu, fmt, resp, oid, cq):
        self._mcu = mcu
        self._fmt = fmt
        self._resp = resp
        self._oid = oid
        self._cq = cq
    def send(self, data=(), minclock=0, reqclock=0):
        return None
    def send_with_preface(self, preface_cmd, preface_data=(), data=(), minclock=0, reqclock=0):
        return None

class FakeMCU:
    KNOWN = {"spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u"}
    def __init__(self, name='mcu', freq=12000000.0):
        self._name = name
        self.freq = freq
        self._oid = 0
        self._cq = 0
        self.config_cmds = []
        self.cbs = []
        self.sends = []
        self._printer = None
    def get_name(self):
        return self._name
    def get_printer(self):
        return self._printer
    def create_oid(self):
        self._oid += 1
        return self._oid
    def register_config_callback(self, cb):
        self.cbs.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, bool(is_init), bool(on_restart)))
    def alloc_command_queue(self):
        self._cq += 1
        return ('cq', self._cq)
    def lookup_command(self, msg, cq=None):
        return FakeCmd(self, msg, cq)
    def lookup_query_command(self, msg, resp, oid=None, cq=None, is_async=False):
        return FakeQueryCmd(self, msg, resp, oid, cq)
    def try_lookup_command(self, msg):
        if msg in self.KNOWN:
            return FakeCmd(self, msg, None)
        return None
    def seconds_to_clock(self, t):
        return int(t * self.freq)
    def estimated_print_time(self, eventtime):
        return 102.0
    def print_time_to_clock(self, print_time):
        return int(print_time * self.freq)

class FakePrinter:
    def __init__(self, mcu, pins, configfile=None):
        self._reactor = FakeReactor()
        self.objects = {'pins': pins}
        if configfile is not None:
            self.objects['configfile'] = configfile
        mcu._printer = self
    def get_reactor(self):
        return self._reactor
    def lookup_object(self, name, default=_MISSING):
        if name in self.objects:
            return self.objects[name]
        if default is _MISSING:
            raise Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfigFile:
    def __init__(self):
        self.deprecations = []
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        self.deprecations.append((feature, msg))

class FakeConfig:
    def __init__(self, printer, section, opts):
        self._printer = printer
        self._section = section
        self._opts = opts
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._section
    def get(self, option, default=_MISSING):
        if option in self._opts:
            return self._opts[option]
        if default is _MISSING:
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._section))
        return default
    def getint(self, option, default=_MISSING, minval=None, maxval=None):
        if option not in self._opts:
            if default is _MISSING:
                raise Error("Option '%s' in section '%s' must be specified" % (option, self._section))
            return default
        v = int(self._opts[option])
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        return v

def new_driver(kind, opts):
    mcu = FakeMCU()
    mcu2 = FakeMCU(name='other')
    pins = FakePins(mcu, mcu2)
    cfgfile = FakeConfigFile()
    printer = FakePrinter(mcu, pins, cfgfile)
    config = FakeConfig(printer, 'display', opts)
    if kind == 'st7920':
        d = st7920.ST7920(config)
    else:
        d = st7920.EmulatedST7920(config)
    return (mcu, printer, config, d)

def run_cbs(mcu):
    cbs = list(mcu.cbs)
    del mcu.cbs[:]
    for cb in cbs:
        cb()
"#;

// ---------------------------------------------------------------------------
// Comparison plumbing + fakes.
// ---------------------------------------------------------------------------

fn trunc_i64(f: f64) -> i64 {
    pycompat::text::trunc_float_to_bigint(f).0.to_string().parse::<i64>().expect("fits")
}

fn fresh_reactor() -> Reactor {
    Reactor::new(false)
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("st7920 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping st7920 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: Value, actual: Value) {
    let record = Record {
        v: 1,
        unit: "extras.display.st7920".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    let exc = match expected {
        Err(e) => e,
        Ok(v) => panic!("{label}: oracle returned {v:?}, expected error"),
    };
    let err = match actual {
        Err(e) => e,
        Ok(v) => panic!("{label}: rust returned {v:?}, expected error"),
    };
    assert_eq!(
        (err.class_name(), err.message()),
        (exc.class_name.as_str(), exc.message.as_str()),
        "{label}"
    );
}

// -- MCU ---------------------------------------------------------------

type SendRecord = (String, i64, Vec<u8>, i64, i64);

struct FakeCmd {
    mcu: Arc<FakeMcu>,
    fmt: String,
    #[allow(dead_code)]
    cq: Option<CommandQueue>,
}

fn normalize_payload(v: &PyValue) -> Vec<u8> {
    // Mirror Python `bytes(payload)` on the recorded MCU traffic.
    match v {
        PyValue::Bytes(b) => b.clone(),
        PyValue::List(items) => items
            .iter()
            .map(|e| match e {
                PyValue::Int(n) => n.inner().to_string().parse::<u8>().expect("byte fits"),
                other => panic!("list item must be int byte, got {}", other.str()),
            })
            .collect(),
        PyValue::Bool(b) => vec![0u8; *b as usize],
        other => panic!("payload must be bytes/list, got {}", other.str()),
    }
}

impl FakeCmd {
    fn record(&self, data: &[PyValue], minclock: i64, reqclock: i64) {
        let oid = match &data[0] {
            PyValue::Int(n) => n.inner().to_string().parse::<i64>().expect("oid fits"),
            other => panic!("oid must be int, got {}", other.str()),
        };
        let payload = normalize_payload(&data[1]);
        self.mcu
            .sends
            .lock()
            .unwrap()
            .push((self.fmt.clone(), oid, payload, minclock, reqclock));
    }
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.record(&data, opts.minclock, opts.reqclock);
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.record(&data, opts.minclock, opts.reqclock);
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeMcu {
    name: String,
    freq: f64,
    oids: Mutex<i64>,
    cqs: Mutex<i64>,
    config_cmds: Mutex<Vec<(String, bool, bool)>>,
    callbacks: Mutex<Vec<klippy_api::mcu::ConfigCallback>>,
    sends: Mutex<Vec<SendRecord>>,
    printer: Mutex<Option<Printer>>,
    me: Mutex<Option<Arc<FakeMcu>>>,
}

impl FakeMcu {
    fn new(name: &str) -> FakeMcu {
        FakeMcu {
            name: name.to_string(),
            freq: 12_000_000.0,
            oids: Mutex::new(0),
            cqs: Mutex::new(0),
            config_cmds: Mutex::new(Vec::new()),
            callbacks: Mutex::new(Vec::new()),
            sends: Mutex::new(Vec::new()),
            printer: Mutex::new(None),
            me: Mutex::new(None),
        }
    }
    fn config_cmds(&self) -> Vec<(String, bool, bool)> {
        self.config_cmds.lock().unwrap().clone()
    }
    fn sends(&self) -> Vec<SendRecord> {
        self.sends.lock().unwrap().clone()
    }
    fn run_callbacks(&self) {
        let cbs: Vec<_> = self.callbacks.lock().unwrap().drain(..).collect();
        for mut cb in cbs {
            cb().expect("config callback");
        }
    }
    fn clone_mcu(&self) -> Arc<FakeMcu> {
        self.me.lock().unwrap().clone().expect("mcu self arc set")
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.lock().unwrap().clone().expect("printer set")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        match pin_type {
            "digital_out" => Ok(Pin::DigitalOut(Arc::new(FakeDigitalOut {
                mcu: self.clone_mcu() as Mcu,
            }))),
            other => Err(KlippyError::Other {
                class_name: "error".to_string(),
                message: format!("Unknown pin type '{other}'"),
            }),
        }
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut o = self.oids.lock().unwrap();
        *o += 1;
        Ok(*o)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.callbacks.lock().unwrap().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        self.config_cmds
            .lock()
            .unwrap()
            .push((cmd.to_string(), opts.is_init, opts.on_restart));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        trunc_i64(time * self.freq)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCmd { mcu: self.clone_mcu(), fmt: msgformat.to_string(), cq }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Ok(Arc::new(FakeQueryCmd { fmt: msgformat.to_string() }))
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        if msgformat
            == "spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u"
        {
            Some(Arc::new(FakeCmd { mcu: self.clone_mcu(), fmt: msgformat.to_string(), cq: None }))
        } else {
            None
        }
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        let mut c = self.cqs.lock().unwrap();
        *c += 1;
        CommandQueue(*c)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsyncResp)
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
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        if name == "CLOCK_FREQ" {
            Ok(self.freq)
        } else {
            Err(KlippyError::Other {
                class_name: "KeyError".to_string(),
                message: format!("'{name}'"),
            })
        }
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        trunc_i64(print_time * self.freq)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / self.freq
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        102.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, self.freq))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::config("create_trigger_dispatch unused in st7920 tests"))
    }
}

struct FakeAsyncResp;
impl AsyncResponseWrapperApi for FakeAsyncResp {
    fn unregister(&self) {}
}

struct FakeQueryCmd {
    #[allow(dead_code)]
    fmt: String,
}
impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryCmd {
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

struct FakeDigitalOut {
    mcu: Mcu,
}
impl klippy_api::mcu::DigitalOutPinApi for FakeDigitalOut {
    fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.mcu)
    }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
    fn set_digital(&self, _print_time: f64, _value: bool) {}
}

// -- Printer -----------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<indexmap::IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter { reactor: fresh_reactor(), objects: Mutex::new(indexmap::IndexMap::new()) }
    }
    fn register(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("READY".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock().unwrap();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{name}' already created")));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock().unwrap();
        match objects.get(name) {
            Some(obj) => Ok(Some(Arc::clone(obj))),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects = self.objects.lock().unwrap();
        objects
            .iter()
            .filter(|(n, _)| match module {
                None => true,
                Some(m) => n.as_str() == m || n.starts_with(&format!("{m} ")),
            })
            .map(|(n, o)| (n.clone(), Arc::clone(o)))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(&self, _config: &Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

// -- Config ------------------------------------------------------------

struct FakeConfig {
    printer: Printer,
    section: String,
    opts: Arc<indexmap::IndexMap<String, String>>,
}

fn missing_option(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

fn parse_option_err(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(missing_option(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.opts.get(option) {
            Some(v) => {
                let parsed = pycompat::text::int_from_string(v, 10)
                    .map_err(|_| parse_option_err(option, &self.section))?;
                let parsed = parsed.0.to_string().parse::<i64>().expect("fits");
                if let Some(min) = opts.minval {
                    if parsed < min {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {min}",
                            self.section
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if parsed > max {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {max}",
                            self.section
                        )));
                    }
                }
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(missing_option(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.opts.get(option) {
            Some(v) => pycompat::text::float_from_string(v)
                .map(Some)
                .map_err(|_| parse_option_err(option, &self.section)),
            None => match opts.default {
                None => Err(missing_option(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        fn parse(v: &str) -> Option<bool> {
            match v.trim().to_ascii_lowercase().as_str() {
                "1" | "yes" | "true" | "on" => Some(true),
                "0" | "no" | "false" | "off" => Some(false),
                _ => None,
            }
        }
        match self.opts.get(option) {
            Some(v) => parse(v)
                .map(Some)
                .ok_or_else(|| parse_option_err(option, &self.section)),
            None => match opts.default {
                None => Err(missing_option(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Err(KlippyError::config("getchoice unused in st7920 tests"))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::config("getlists unused in st7920 tests"))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::config("getlist unused in st7920 tests"))
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::config("getintlist unused in st7920 tests"))
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::config("getfloatlist unused in st7920 tests"))
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: Arc::clone(&self.printer),
            section: section.to_string(),
            opts: Arc::clone(&self.opts),
        })
    }
    fn has_section(&self, _section: &str) -> bool {
        true
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// -- Pins --------------------------------------------------------------

struct FakePins {
    chips: indexmap::IndexMap<String, Mcu>,
    active: Mutex<indexmap::IndexMap<String, PinParams>>,
}

impl FakePins {
    fn parse(&self, pin_desc: &str, can_invert: bool, can_pullup: bool) -> KResult<PinParams> {
        let mut desc = pin_desc.trim().to_string();
        let mut pullup = 0;
        let mut invert = 0;
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
            None => ("mcu".to_string(), desc.clone()),
            Some(i) => (desc[..i].trim().to_string(), desc[i + 1..].trim().to_string()),
        };
        let Some(chip) = self.chips.get(&chip_name).cloned() else {
            return Err(KlippyError::Other {
                class_name: "error".to_string(),
                message: format!("Unknown pin chip name '{chip_name}'"),
            });
        };
        let invalid = pin.chars().any(|c| "^~!:".contains(c))
            || pin.split_whitespace().collect::<Vec<_>>().join("") != pin;
        if invalid {
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
        Ok(PinParams { chip, chip_name, pin, invert, pullup, share_type: None })
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        self.parse(pin_desc, opts.can_invert, opts.can_pullup)
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let mut pin_params = self.parse(pin_desc, opts.can_invert, opts.can_pullup)?;
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        let mut active = self.active.lock().unwrap();
        if let Some(share_params) = active.get(&share_name).cloned() {
            if opts.share_type.is_none() || opts.share_type != share_params.share_type {
                return Err(KlippyError::Other {
                    class_name: "error".to_string(),
                    message: format!("pin {} used multiple times in config", pin_params.pin),
                });
            }
            if pin_params.invert != share_params.invert || pin_params.pullup != share_params.pullup {
                return Err(KlippyError::Other {
                    class_name: "error".to_string(),
                    message: format!("Shared pin {} must have same polarity", pin_params.pin),
                });
            }
            return Ok(share_params);
        }
        pin_params.share_type = opts.share_type;
        active.insert(share_name, pin_params.clone());
        Ok(pin_params)
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        let can_invert = pin_type == "endstop" || pin_type == "digital_out" || pin_type == "pwm";
        let can_pullup = pin_type == "endstop";
        let pin_params = self.parse(pin_desc, can_invert, can_pullup)?;
        let chip = pin_params.chip.clone();
        chip.setup_pin(pin_type, pin_params)
    }
    fn reset_pin_sharing(&self, pin_params: PinParams) -> klippy_api::KResult<()> {
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        self.active.lock().unwrap().shift_remove(&share_name);
        Ok(())
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Ok(Arc::new(FakePinResolver))
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

struct FakePinResolver;
impl PinResolverApi for FakePinResolver {
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

// -- Configfile ---------------------------------------------------------

struct FakeConfigFileService {
    deprecations: Mutex<Vec<(String, Option<String>)>>,
}

impl PrinterConfigApi for FakeConfigFileService {
    fn get_printer(&self) -> Printer {
        panic!("get_printer unused in st7920 tests")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Err(KlippyError::config("read_config unused in st7920 tests"))
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::config("read_main_config unused in st7920 tests"))
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {}
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, feature: &str, msg: Option<String>) {
        self.deprecations.lock().unwrap().push((feature.to_string(), msg));
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> { Ok(()) }
    fn remove_section(&self, _section: &str) {}
}

impl PrinterObject for FakeConfigFileService {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
}

// -- Driver fixture ------------------------------------------------------

struct Fixture {
    mcu: Arc<FakeMcu>,
    _mcu2: Arc<FakeMcu>,
    _printer: Arc<FakePrinter>,
    config: Config,
    _pins: Arc<FakePins>,
    _configfile: Arc<FakeConfigFileService>,
}

fn fixture(opts: Vec<(&str, &str)>) -> Fixture {
    let mcu = Arc::new(FakeMcu::new("mcu"));
    let mcu2 = Arc::new(FakeMcu::new("other"));
    let printer = Arc::new(FakePrinter::new());
    let printer_dyn = Arc::clone(&printer) as Printer;
    mcu.printer.lock().unwrap().replace(Arc::clone(&printer_dyn));
    mcu2.printer.lock().unwrap().replace(Arc::clone(&printer_dyn));
    mcu.me.lock().unwrap().replace(Arc::clone(&mcu));
    mcu2.me.lock().unwrap().replace(Arc::clone(&mcu2));
    let mut chips = indexmap::IndexMap::new();
    chips.insert("mcu".to_string(), Arc::clone(&mcu) as Mcu);
    chips.insert("other".to_string(), Arc::clone(&mcu2) as Mcu);
    let pins = Arc::new(FakePins { chips, active: Mutex::new(indexmap::IndexMap::new()) });
    let configfile =
        Arc::new(FakeConfigFileService { deprecations: Mutex::new(Vec::new()) });
    printer.register("pins", pins.clone());
    printer.register("configfile", configfile.clone());
    let opts_map: indexmap::IndexMap<String, String> =
        opts.into_iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
    let config = Arc::new(FakeConfig {
        printer: printer_dyn,
        section: "display".to_string(),
        opts: Arc::new(opts_map),
    }) as Config;
    Fixture { mcu, _mcu2: mcu2, _printer: printer, config, _pins: pins, _configfile: configfile }
}

fn sends_value(sends: &[SendRecord]) -> Value {
    Value::List(
        sends
            .iter()
            .map(|(fmt, oid, payload, minclock, reqclock)| {
                Value::Tuple(vec![
                    Value::str(fmt),
                    Value::int(*oid),
                    Value::bytes(payload),
                    Value::int(*minclock),
                    Value::int(*reqclock),
                ])
            })
            .collect(),
    )
}

fn config_cmds_value(cmds: &[(String, bool, bool)]) -> Value {
    Value::List(
        cmds
            .iter()
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

// ---------------------------------------------------------------------------
// DisplayBase helpers (shared direct comparisons).
// ---------------------------------------------------------------------------

struct SendLog {
    sends: Vec<(Vec<u8>, bool, bool)>,
}

impl SendLog {
    fn value(&self) -> Value {
        Value::List(
            self.sends
                .iter()
                .map(|(cmds, is_data, is_extended)| {
                    Value::Tuple(vec![
                        Value::bytes(cmds),
                        Value::Bool(*is_data),
                        Value::Bool(*is_extended),
                    ])
                })
                .collect(),
        )
    }
}

fn base_value(base: &DisplayBase, sent: &SendLog) -> Value {
    Value::Tuple(vec![
        Value::bytes(&base.text_framebuffer()),
        Value::bytes(&base.glyph_framebuffer()),
        Value::List(
            base.graphics_framebuffers().iter().map(|row| Value::bytes(row)).collect(),
        ),
        Value::Dict(
            base.cached_glyphs()
                .iter()
                .map(|(k, (b, (a, c)))| {
                    (
                        Value::str(k),
                        Value::Tuple(vec![
                            Value::str(b),
                            Value::Tuple(vec![Value::int(*a), Value::int(*c)]),
                        ]),
                    )
                })
                .collect(),
        ),
        sent.value(),
    ])
}

fn fresh_base(py: &mut pyoracle::Oracle) -> (DisplayBase, SendLog) {
    py.exec(
        "disp = st7920.DisplayBase()\n\
         _sends = []\n\
         def _send_rec(cmds, is_data=False, is_extended=False):\n\
         \x20   _sends.append((bytes(cmds), bool(is_data), bool(is_extended)))\n\
         disp.send = _send_rec\n",
    )
    .expect("py DisplayBase");
    (DisplayBase::new(), SendLog { sends: Vec::new() })
}

fn base_state_expr() -> &'static str {
    "(bytes(disp.text_framebuffer), bytes(disp.glyph_framebuffer), \
     [bytes(x) for x in disp.graphics_framebuffers], \
     {k: (v[0], tuple(v[1])) for k, v in disp.cached_glyphs.items()}, _sends)"
}

fn rust_glyphs_fixture() -> PyValue {
    let icon = |lo: u8| {
        PyValue::Tuple(vec![
            PyValue::List((0..16).map(|i| PyValue::Int(PyInt::from_i64(lo as i64 + i))).collect()),
            PyValue::List((16..32).map(|i| PyValue::Int(PyInt::from_i64(lo as i64 + i))).collect()),
        ])
    };
    let glyph = |icon_val: PyValue| {
        let mut fields = indexmap::IndexMap::new();
        fields.insert(PyValue::Str("icon16x16".to_string()), icon_val);
        PyValue::Dict(fields)
    };
    let mut glyphs = indexmap::IndexMap::new();
    glyphs.insert(PyValue::Str("fan1".to_string()), glyph(icon(0)));
    glyphs.insert(PyValue::Str("fan2".to_string()), glyph(icon(32)));
    glyphs.insert(PyValue::Str("bed_heat1".to_string()), glyph(icon(64)));
    glyphs.insert(PyValue::Str("bed_heat2".to_string()), glyph(icon(96)));
    glyphs.insert(PyValue::Str("right_arrow".to_string()), glyph(icon(128)));
    // Present but without icon16x16: skipped by set_glyphs.
    let mut skip_fields = indexmap::IndexMap::new();
    skip_fields.insert(PyValue::Int(PyInt::from_i64(3)), PyValue::Int(PyInt::from_i64(1)));
    glyphs.insert(PyValue::Str("note".to_string()), PyValue::Dict(skip_fields));
    PyValue::Dict(glyphs)
}

const PY_GLYPH_FIXTURE: &str = r#"
glyphs = {'fan1': {'icon16x16': (list(range(0,16)), list(range(16,32)))},
          'fan2': {'icon16x16': (list(range(32,48)), list(range(48,64)))},
          'bed_heat1': {'icon16x16': (list(range(64,80)), list(range(80,96)))},
          'bed_heat2': {'icon16x16': (list(range(96,112)), list(range(112,128)))},
          'right_arrow': {'icon16x16': (list(range(128,144)), list(range(144,160)))},
          'note': {'icon5x8': (3, [1])}}
disp.set_glyphs(glyphs)
"#;

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_is_empty() {
    let factory = extras::display::st7920::module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn base_initial_state_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, sent) = fresh_base(&mut py);
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base initial", expected, base_value(&base, &sent));
}

#[test]
fn base_dimensions_match_python() {
    let Some(mut py) = oracle() else { return };
    let _base = fresh_base(&mut py);
    let expected = py.eval("disp.get_dimensions()").expect("py dims");
    let (w, h) = DisplayBase::new().get_dimensions();
    check(
        "dims",
        expected,
        Value::Tuple(vec![Value::int(w as i64), Value::int(h as i64)]),
    );
}

#[test]
fn base_init_and_flush_match_python() {
    let Some(mut py) = oracle() else { return };
    let (base, mut sent) = fresh_base(&mut py);
    base.init(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust init");
    py.exec("disp.init()").expect("py init");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base init+flush", expected, base_value(&base, &sent));
}

#[test]
fn base_write_text_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, sent) = fresh_base(&mut py);
    // Interior writes, truncation at x=15, fully-clipped x=16, row underlaps.
    base.write_text(3, 2, b"Hello").expect("rust write");
    base.write_text(15, 0, b"abcdefghij").expect("rust write");
    base.write_text(100, 3, b"ZZZZ").expect("rust write");
    base.write_text(0, 0, b"").expect("rust write");
    base.write_text(2, 0, b"a").expect("rust write");
    py.exec(
        "disp.write_text(3, 2, b'Hello')\n\
         disp.write_text(15, 0, b'abcdefghij')\n\
         disp.write_text(100, 3, b'ZZZZ')\n\
         disp.write_text(0, 0, b'')\n\
         disp.write_text(2, 0, b'a')\n\
         disp.flush()\n",
    )
    .expect("py writes");
    let mut sent = sent;
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base write_text", expected, base_value(&base, &sent));
}

#[test]
fn base_write_text_wraps_negative_y_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, sent) = fresh_base(&mut py);
    base.write_text(4, -1, b"wrap").expect("rust write");
    base.write_text(1, -4, b"first").expect("rust write");
    py.exec("disp.write_text(4, -1, b'wrap')\ndisp.write_text(1, -4, b'first')")
        .expect("py writes");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base wrap y", expected, base_value(&base, &sent));
}

#[test]
fn base_write_text_y_out_of_range_is_index_error() {
    let Some(mut py) = oracle() else { return };
    let (base, _sent) = fresh_base(&mut py);
    let rust_result = base.write_text(0, 4, b"x").map(|_| Value::Null);
    let expected = py.eval("disp.write_text(0, 4, b'x')");
    check_err("base y>=4", expected, rust_result);
    let rust_result = base.write_text(0, -5, b"x").map(|_| Value::Null);
    let expected = py.eval("disp.write_text(0, -5, b'x')");
    check_err("base y<-4", expected, rust_result);
}

#[test]
fn base_write_graphics_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, mut sent) = fresh_base(&mut py);
    let row: Vec<u8> = (0..16).map(|i| i as u8).collect();
    let row2: Vec<u8> = (0..16).map(|i| 0xffu8 - i as u8).collect();
    base.write_graphics(5, 1, &row).expect("rust gfx");
    base.write_graphics(3, 3, &row2).expect("rust gfx");
    // Rejected by the bounds guard: x >= 16, y >= 4, wrong length.
    base.write_graphics(16, 1, &row).expect("rust gfx rejected");
    base.write_graphics(2, 4, &row).expect("rust gfx rejected");
    base.write_graphics(2, 1, &row[..15]).expect("rust gfx rejected");
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush");
    py.exec(
        "disp.write_graphics(5, 1, list(range(16)))\n\
         disp.write_graphics(3, 3, [0xff - i for i in range(16)])\n\
         disp.write_graphics(16, 1, list(range(16)))\n\
         disp.write_graphics(2, 4, list(range(16)))\n\
         disp.write_graphics(2, 1, list(range(15)))\n\
         disp.flush()\n",
    )
    .expect("py gfx");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base write_graphics", expected, base_value(&base, &sent));
}

#[test]
fn base_write_graphics_gfx_wrap_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, mut sent) = fresh_base(&mut py);
    // y=2 maps to framebuffers starting at 32 -> gfx_fb wraps to 0, x += 16.
    let row: Vec<u8> = (0..16).map(|i| 0xa0u8 + i as u8).collect();
    base.write_graphics(2, 2, &row).expect("rust gfx wrap");
    py.exec("disp.write_graphics(2, 2, [0xa0 + i for i in range(16)])").expect("py gfx wrap");
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush");
    py.exec("disp.flush()").expect("py flush");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base gfx wrap", expected, base_value(&base, &sent));
}

#[test]
fn base_write_graphics_negative_index_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    let (base, _sent) = fresh_base(&mut py);
    let row = [0u8; 16];
    let rust_result = base.write_graphics(-40, 0, &row).map(|_| Value::Null);
    let expected = py.eval("disp.write_graphics(-40, 0, [0]*16)");
    check_err("base gfx col", expected, rust_result);
    let rust_result = base.write_graphics(0, -3, &row).map(|_| Value::Null);
    let expected = py.eval("disp.write_graphics(0, -3, [0]*16)");
    check_err("base gfx fb", expected, rust_result);
}

#[test]
fn base_set_glyphs_and_cached_flush_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, mut sent) = fresh_base(&mut py);
    base.set_glyphs(&rust_glyphs_fixture());
    py.exec(PY_GLYPH_FIXTURE).expect("py glyphs");
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush");
    py.exec("disp.flush()").expect("py flush");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base set_glyphs", expected, base_value(&base, &sent));
}

#[test]
fn base_write_glyph_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, mut sent) = fresh_base(&mut py);
    base.set_glyphs(&rust_glyphs_fixture());
    py.exec(PY_GLYPH_FIXTURE).expect("py glyphs");
    let cases = [
        (0, 0, "fan2"),
        (2, 0, "fan2"),
        (3, 0, "fan2"),
        (4, 1, "bed_heat2"),
        (1, 1, "bed_heat2"),
        (6, 2, "right_arrow"),
        (7, 2, "degrees"),
        (8, 3, "note"),
        (9, 3, "unknown"),
    ];
    let mut rust_returns = Vec::new();
    for (x, y, name) in cases {
        rust_returns.push(base.write_glyph(x, y, name).expect("rust glyph"));
    }
    py.exec(
        "_rets = [\n\
        \x20 disp.write_glyph(0, 0, 'fan2'),\n\
        \x20 disp.write_glyph(2, 0, 'fan2'),\n\
        \x20 disp.write_glyph(3, 0, 'fan2'),\n\
        \x20 disp.write_glyph(4, 1, 'bed_heat2'),\n\
        \x20 disp.write_glyph(1, 1, 'bed_heat2'),\n\
        \x20 disp.write_glyph(6, 2, 'right_arrow'),\n\
        \x20 disp.write_glyph(7, 2, 'degrees'),\n\
        \x20 disp.write_glyph(8, 3, 'note'),\n\
        \x20 disp.write_glyph(9, 3, 'unknown'),\n\
        \x20]\n",
    )
    .expect("py glyphs");
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush");
    py.exec("disp.flush()").expect("py flush");
    let expected_rets = py.eval("_rets").expect("py rets");
    check(
        "base glyph rets",
        expected_rets,
        Value::List(rust_returns.iter().map(|r| Value::int(*r)).collect()),
    );
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base write_glyph", expected, base_value(&base, &sent));
}

#[test]
fn base_clear_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (base, mut sent) = fresh_base(&mut py);
    base.set_glyphs(&rust_glyphs_fixture());
    py.exec(PY_GLYPH_FIXTURE).expect("py glyphs");
    base.write_text(0, 0, b"DATA").expect("rust write");
    base.write_graphics(0, 0, &[0x55u8; 16]).expect("rust gfx");
    py.exec(
        "disp.write_text(0, 0, b'DATA')\n\
         disp.write_graphics(0, 0, [0x55]*16)\n",
    )
    .expect("py writes");
    // First flush syncs; clear + flush sends only the cleared areas.
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush 1");
    py.exec("disp.flush()").expect("py flush 1");
    base.clear();
    base.flush(&mut |cmds, is_data, is_extended| {
        sent.sends.push((cmds.to_vec(), is_data, is_extended));
        Ok(())
    })
    .expect("rust flush 2");
    py.exec("disp.clear()\ndisp.flush()").expect("py clear");
    let expected = py.eval(base_state_expr()).expect("py state");
    check("base clear", expected, base_value(&base, &sent));
}

// ---------------------------------------------------------------------------
// ST7920 driver tests.
// ---------------------------------------------------------------------------

fn st7920_opts() -> Vec<(&'static str, &'static str)> {
    vec![("cs_pin", "mcu:PA4"), ("sclk_pin", "mcu:PA5"), ("sid_pin", "mcu:PA6")]
}

fn emu_opts() -> Vec<(&'static str, &'static str)> {
    vec![
        ("spi_software_miso_pin", "mcu:PA1"),
        ("spi_software_mosi_pin", "mcu:PA2"),
        ("spi_software_sclk_pin", "mcu:PA3"),
        ("spi_speed", "1000000"),
        ("en_pin", "mcu:PA7"),
    ]
}

#[test]
fn st7920_build_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(st7920_opts());
    let st = St7920::new(&fx.config).expect("rust St7920");
    fx.mcu.run_callbacks();
    py.exec(
        "mcu, printer, config, d = new_driver('st7920', {'cs_pin': 'mcu:PA4', 'sclk_pin': 'mcu:PA5', 'sid_pin': 'mcu:PA6'})\n\
         run_cbs(mcu)\n",
    )
    .expect("py build_config");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py build");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("st7920 build_config", expected, actual);
    let _ = st;
}

#[test]
fn st7920_init_and_flush_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(st7920_opts());
    let st = St7920::new(&fx.config).expect("rust St7920");
    fx.mcu.run_callbacks();
    st.init().expect("rust init");
    py.exec(
        "mcu, printer, config, d = new_driver('st7920', {'cs_pin': 'mcu:PA4', 'sclk_pin': 'mcu:PA5', 'sid_pin': 'mcu:PA6'})\n\
         run_cbs(mcu)\n\
         d.init()\n",
    )
    .expect("py init");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py init");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("st7920 init", expected, actual);
    // Second flush after a write: only the diff goes out, through data cmd.
    st.write_text(0, 0, b"Hi").expect("rust write");
    st.flush().expect("rust flush");
    py.exec("d.write_text(0, 0, b'Hi')\nd.flush()").expect("py write+flush");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py write");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("st7920 flush diff", expected, actual);
    // write_glyph / clear / flush through the chip trait.
    st.write_glyph(2, 0, "degrees").expect("rust glyph");
    st.clear();
    st.flush().expect("rust flush");
    py.exec("d.write_glyph(2, 0, 'degrees')\nd.clear()\nd.flush()").expect("py glyph+clear");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py glyph+clear");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("st7920 glyph+clear", expected, actual);
}

#[test]
fn st7920_send_before_build_config_is_attribute_error() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(st7920_opts());
    let st = St7920::new(&fx.config).expect("rust St7920");
    let rust_result = st.send(&[0x24], false, false).map(|_| Value::Null);
    py.exec(
        "mcu, printer, config, d = new_driver('st7920', {'cs_pin': 'mcu:PA4', 'sclk_pin': 'mcu:PA5', 'sid_pin': 'mcu:PA6'})\n",
    )
    .expect("py ctor");
    let expected = py.eval("d.send([0x24])");
    check_err("st7920 send before build", expected, rust_result);
}

#[test]
fn st7920_missing_pin_option_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(vec![("cs_pin", "mcu:PA4"), ("sclk_pin", "mcu:PA5")]);
    let rust_result = St7920::new(&fx.config).map(|_| Value::Null);
    let expected = py.eval(
        "new_driver('st7920', {'cs_pin': 'mcu:PA4', 'sclk_pin': 'mcu:PA5'})[3].send([0x24])",
    );
    check_err("st7920 missing pin opt", expected, rust_result);
}

#[test]
fn st7920_pin_mismatch_is_pins_error() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(vec![
        ("cs_pin", "mcu:PA4"),
        ("sclk_pin", "other:PA5"),
        ("sid_pin", "mcu:PA6"),
    ]);
    let rust_result = St7920::new(&fx.config).map(|_| Value::Null);
    let expected = py.eval(
        "new_driver('st7920', {'cs_pin': 'mcu:PA4', 'sclk_pin': 'other:PA5', 'sid_pin': 'mcu:PA6'})",
    );
    check_err("st7920 pin mismatch", expected, rust_result);
}

#[test]
fn st7920_missing_pins_object_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let mcu = Arc::new(FakeMcu::new("mcu"));
    let printer = Arc::new(FakePrinter::new());
    let printer_dyn = Arc::clone(&printer) as Printer;
    mcu.printer.lock().unwrap().replace(printer_dyn.clone());
    mcu.me.lock().unwrap().replace(Arc::clone(&mcu));
    let config = Arc::new(FakeConfig {
        printer: printer_dyn,
        section: "display".to_string(),
        opts: Arc::new(
            st7920_opts().into_iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        ),
    }) as Config;
    let rust_result = St7920::new(&config).map(|_| Value::Null);
    py.exec(
        "mcu = FakeMCU()\n\
         printer = FakePrinter.__new__(FakePrinter)\n\
         printer._reactor = FakeReactor()\n\
         printer.objects = {}\n\
         mcu._printer = printer\n\
         config = FakeConfig(printer, 'display', {'cs_pin': 'mcu:PA4', 'sclk_pin': 'mcu:PA5', 'sid_pin': 'mcu:PA6'})\n",
    )
    .expect("py setup no pins");
    let expected = py.eval("st7920.ST7920(config)");
    check_err("st7920 missing pins", expected, rust_result);
}

// ---------------------------------------------------------------------------
// Emulated ST7920 tests.
// ---------------------------------------------------------------------------

#[test]
fn emulated_build_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(emu_opts());
    EmulatedSt7920::new(&fx.config).expect("rust EmulatedSt7920");
    fx.mcu.run_callbacks();
    py.exec(
        "mcu, printer, config, d = new_driver('emulated_st7920', {'spi_software_miso_pin': 'mcu:PA1', 'spi_software_mosi_pin': 'mcu:PA2', 'spi_software_sclk_pin': 'mcu:PA3', 'spi_speed': '1000000', 'en_pin': 'mcu:PA7'})\n\
         run_cbs(mcu)\n",
    )
    .expect("py build_config");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py build");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("emulated build_config", expected, actual);
}

#[test]
fn emulated_init_and_flush_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(emu_opts());
    let d = EmulatedSt7920::new(&fx.config).expect("rust EmulatedSt7920");
    fx.mcu.run_callbacks();
    d.init().expect("rust init");
    py.exec(
        "mcu, printer, config, d = new_driver('emulated_st7920', {'spi_software_miso_pin': 'mcu:PA1', 'spi_software_mosi_pin': 'mcu:PA2', 'spi_software_sclk_pin': 'mcu:PA3', 'spi_speed': '1000000', 'en_pin': 'mcu:PA7'})\n\
         run_cbs(mcu)\n\
         d.init()\n",
    )
    .expect("py init");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py init");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("emulated init", expected, actual);
    // A data + write + flush round through the emulator.
    d.write_text(0, 0, b"Hi").expect("rust write");
    d.flush().expect("rust flush");
    py.exec("d.write_text(0, 0, b'Hi')\nd.flush()").expect("py write+flush");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py write");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("emulated flush diff", expected, actual);
}

#[test]
fn emulated_send_before_build_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(emu_opts());
    let d = EmulatedSt7920::new(&fx.config).expect("rust EmulatedSt7920");
    // No build_config callbacks run yet: en update and spi send become
    // init config commands.
    d.send(&[0x24, 0x40], false, false).expect("rust pre-build send");
    py.exec(
        "mcu, printer, config, d = new_driver('emulated_st7920', {'spi_software_miso_pin': 'mcu:PA1', 'spi_software_mosi_pin': 'mcu:PA2', 'spi_software_sclk_pin': 'mcu:PA3', 'spi_speed': '1000000', 'en_pin': 'mcu:PA7'})\n\
         d.send([0x24, 0x40])\n",
    )
    .expect("py pre-build send");
    let expected = py.eval("(mcu.config_cmds, mcu.sends)").expect("py pre-build");
    let actual = Value::Tuple(vec![
        config_cmds_value(&fx.mcu.config_cmds()),
        sends_value(&fx.mcu.sends()),
    ]);
    check("emulated pre-build", expected, actual);
}

#[test]
fn emulated_spi_speed_below_min_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(vec![
        ("spi_software_miso_pin", "mcu:PA1"),
        ("spi_software_mosi_pin", "mcu:PA2"),
        ("spi_software_sclk_pin", "mcu:PA3"),
        ("spi_speed", "50"),
        ("en_pin", "mcu:PA7"),
    ]);
    let rust_result = EmulatedSt7920::new(&fx.config).map(|_| Value::Null);
    let expected = py.eval(
        "new_driver('emulated_st7920', {'spi_software_miso_pin': 'mcu:PA1', 'spi_software_mosi_pin': 'mcu:PA2', 'spi_software_sclk_pin': 'mcu:PA3', 'spi_speed': '50', 'en_pin': 'mcu:PA7'})",
    );
    check_err("emulated spi_speed min", expected, rust_result);
}

#[test]
fn emulated_pin_mismatch_is_pins_error() {
    let Some(mut py) = oracle() else { return };
    let fx = fixture(vec![
        ("spi_software_miso_pin", "mcu:PA1"),
        ("spi_software_mosi_pin", "other:PA2"),
        ("spi_software_sclk_pin", "mcu:PA3"),
        ("spi_speed", "1000000"),
        ("en_pin", "mcu:PA7"),
    ]);
    let rust_result = EmulatedSt7920::new(&fx.config).map(|_| Value::Null);
    let expected = py.eval(
        "new_driver('emulated_st7920', {'spi_software_miso_pin': 'mcu:PA1', 'spi_software_mosi_pin': 'other:PA2', 'spi_software_sclk_pin': 'mcu:PA3', 'spi_speed': '1000000', 'en_pin': 'mcu:PA7'})",
    );
    check_err("emulated pin mismatch", expected, rust_result);
}
