// Parity tests for `klippy/extras/aht10.py` (port: `extras::aht10`).
//
// Expected values come from the live `pyoracle` (frozen CPython plus the
// frozen `klippy/`), driven through Python fakes that mirror the Rust
// fakes below: a scripted I2C bus, a config wrapper returning a fixed
// `aht10_report_time`, a printer recording objects/handlers/shutdowns and
// a reactor whose clock is fixed at 1000.0. The Rust side forces the
// same scripts and compares status dicts, log lines (level and formatted
// message; traceback bodies are normalised by the parity table), I2C
// writes, read requests, config getter calls, shutdown messages, the
// sampling callback temperature and the measurement results bit for bit.
//
// Clock-derived values cannot be oracle-compared: the Rust reactor reads
// the real monotonic clock while the Python fake returns 1000.0. Those
// values (the next-waketime of a successful sample, the callback
// `print_time`) are asserted against the call window instead. The
// `NEVER` return of a failed sample is a frozen constant and is
// oracle-compared.
//
// Every test wants the oracle interpreter (`$KLIPPY_PYTHON`); outside
// the oracle image the test reports the skip and passes.

use extras::aht10::{aht1x, aht2x, aht3x, module_factory};
use extras::bus::McuI2cApi;
use extras::heaters::{Heater, PrinterHeatersApi};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListsOpts,
    GetListOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueue,
    CommandQueryWrapper, CommandWrapper, ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi,
    PinParams, QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

thread_local! {
    /// Lines logged on this test's thread; parallel tests cannot clear or
    /// add to each other's capture.
    static LOG_CAPTURE: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static LOG_ONCE: std::sync::Once = std::sync::Once::new();
    LOG_ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_CAPTURE.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn clear_logs() {
    install_log_sink();
    LOG_CAPTURE.with(|logs| logs.borrow_mut().clear());
}

fn logs() -> Vec<(String, String)> {
    LOG_CAPTURE.with(|logs| logs.borrow().clone())
}

// ---------------------------------------------------------------------------
// Fakes of the contract traits.
// ---------------------------------------------------------------------------

#[derive(Default)]
struct PrinterInner {
    objects: Mutex<indexmap::IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<String>>,
    shutdown: Mutex<Vec<String>>,
}

struct FakePrinter {
    inner: Arc<PrinterInner>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> (Arc<FakePrinter>, Reactor) {
        let reactor = Reactor::new(false);
        let printer = Arc::new(FakePrinter {
            inner: Arc::new(PrinterInner::default()),
            reactor: reactor.clone(),
        });
        (printer, reactor)
    }

    fn object_names(&self) -> Vec<String> {
        self.inner.objects.lock().unwrap().keys().cloned().collect()
    }

    fn handler_count(&self, event: &str) -> usize {
        self.inner
            .handlers
            .lock()
            .unwrap()
            .iter()
            .filter(|name| name.as_str() == event)
            .count()
    }

    fn shutdown_messages(&self) -> Vec<String> {
        self.inner.shutdown.lock().unwrap().clone()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::default()
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
        let mut objects = self.inner.objects.lock().unwrap();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!("Duplicate printer object '{name}'")));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.inner.objects.lock().unwrap();
        match objects.get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::config(format!("Unable to lookup object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.inner.objects.lock().unwrap();
        objects.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
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
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.inner.shutdown.lock().unwrap().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, details: PyValue) {
        self.invoke_shutdown(msg, details)
    }
    fn register_event_handler(&self, event: &str, _callback: klippy_api::printer::EventCallback) {
        self.inner.handlers.lock().unwrap().push(event.to_string());
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfigInner {
    printer: Printer,
    name: String,
    report_time: i64,
    report_time_error: Option<KlippyError>,
    getint_calls: Mutex<Vec<(String, Option<Option<i64>>, Option<i64>, Option<i64>)>>,
}

impl ConfigApi for FakeConfigInner {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, _option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(opts.default.clone().unwrap_or(None))
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.getint_calls.lock().unwrap().push((
            option.to_string(),
            opts.default.clone(),
            opts.minval,
            opts.maxval,
        ));
        if let Some(err) = &self.report_time_error {
            return Err(err.clone());
        }
        Ok(Some(self.report_time))
    }
    fn getfloat(&self, _option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(opts.default.flatten())
    }
    fn getboolean(&self, _option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(opts.default.unwrap_or(None))
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::default())
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::List(Vec::new()))
    }
    fn getlist(&self, _option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(opts.default.flatten())
    }
    fn getintlist(
        &self,
        _option: &str,
        opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(opts.default.flatten())
    }
    fn getfloatlist(
        &self,
        _option: &str,
        opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(opts.default.flatten())
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfigInner {
            printer: self.printer.clone(),
            name: section.to_string(),
            report_time: self.report_time,
            report_time_error: None,
            getint_calls: Mutex::new(Vec::new()),
        })
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

fn fake_config(
    printer: &Printer,
    name: &str,
    report_time: i64,
    report_time_error: Option<KlippyError>,
) -> (Config, Arc<FakeConfigInner>) {
    let inner = Arc::new(FakeConfigInner {
        printer: printer.clone(),
        name: name.to_string(),
        report_time,
        report_time_error,
        getint_calls: Mutex::new(Vec::new()),
    });
    let config: Config = inner.clone();
    (config, inner)
}

struct FakeAsyncWrapper;

impl AsyncResponseWrapperApi for FakeAsyncWrapper {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
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
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Mcu("FakeMcu: setup_pin not scripted".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
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
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.
    }
    fn max_nominal_duration(&self) -> f64 {
        0.
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Err(KlippyError::Mcu("FakeMcu: lookup_command not scripted".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu("FakeMcu: lookup_query_command not scripted".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsyncWrapper)
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
        Err(KlippyError::Mcu("FakeMcu: get_constant_float not scripted".to_string()))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0., 0.))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(
        &self,
    ) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Mcu("FakeMcu: create_trigger_dispatch not scripted".to_string()))
    }
}

enum ReadStep {
    Data(Vec<u8>),
    None,
    Error(KlippyError),
}

struct FakeI2cInner {
    writes: Mutex<Vec<Vec<u8>>>,
    reads: Mutex<VecDeque<ReadStep>>,
    read_calls: Mutex<Vec<(Vec<u8>, usize)>>,
}

struct FakeI2c {
    inner: Arc<FakeI2cInner>,
    mcu: Mcu,
}

impl FakeI2c {
    fn new(mcu: Mcu, reads: Vec<ReadStep>) -> Arc<FakeI2c> {
        Arc::new(FakeI2c {
            inner: Arc::new(FakeI2cInner {
                writes: Mutex::new(Vec::new()),
                reads: Mutex::new(reads.into()),
                read_calls: Mutex::new(Vec::new()),
            }),
            mcu,
        })
    }

    fn dyn_i2c(raw: Arc<FakeI2c>) -> Arc<dyn McuI2cApi> {
        raw
    }
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_i2c_address(&self) -> i64 {
        0x38
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write_noack(&self, _data: &[u8], _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn i2c_write(&self, data: &[u8], _opts: QuerySendOpts) -> KResult<()> {
        self.inner.writes.lock().unwrap().push(data.to_vec());
        Ok(())
    }
    fn i2c_read(
        &self,
        write: &[u8],
        read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        self.inner
            .read_calls
            .lock()
            .unwrap()
            .push((write.to_vec(), read_len as usize));
        match self.inner.reads.lock().unwrap().pop_front() {
            Some(ReadStep::Data(bytes)) => {
                let mut map = indexmap::IndexMap::new();
                map.insert(
                    PyValue::Str("response".to_string()),
                    PyValue::Bytes(bytes),
                );
                Ok(PyValue::Dict(map))
            }
            Some(ReadStep::None) => Ok(PyValue::None),
            Some(ReadStep::Error(err)) => Err(err),
            None => Err(KlippyError::Mcu("FakeI2c: no scripted read left".to_string())),
        }
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::Mcu("FakeI2c: i2c_transfer not scripted".to_string()))
    }
}

fn make_sensor(
    reads: Vec<ReadStep>,
) -> (Arc<FakePrinter>, Arc<FakeI2c>, Config, Arc<FakeConfigInner>) {
    let (printer_raw, _reactor) = FakePrinter::new();
    let printer: Printer = printer_raw.clone();
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer.clone() });
    let i2c = FakeI2c::new(mcu, reads);
    let (config, config_inner) = fake_config(&printer, "temperature_sensor bench", 30, None);
    (printer_raw, i2c, config, config_inner)
}

// ---------------------------------------------------------------------------
// Oracle helpers (Python side).
// ---------------------------------------------------------------------------

const PY_BASE: &str = r"
import logging, sys, types
LOGS = []
class _Cap(logging.Handler):
    def emit(self, record):
        LOGS.append((record.levelname, record.getMessage()))
logging.getLogger().addHandler(_Cap())
logging.getLogger().setLevel(0)

WRITES = []
READS = []
READ_CALLS = []
class FakeMcu:
    def estimated_print_time(self, eventtime): return eventtime
class FakeI2C:
    def i2c_write(self, data): WRITES.append(list(data))
    def i2c_read(self, write, read_len, retry=True):
        READ_CALLS.append((list(write), read_len))
        r = READS.pop(0)
        if isinstance(r, Exception): raise r
        if r is None: return None
        return {'response': bytes(r)}
    def get_mcu(self): return FAKE_MCU
FAKE_MCU = FakeMcu()
_i2c = FakeI2C()
class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self): self.timers = []
    def monotonic(self): return 1000.0
    def pause(self, eventtime): return eventtime
    def register_timer(self, callback, waketime=NEVER):
        self.timers.append(callback)
        return len(self.timers)
    def update_timer(self, timer, waketime): pass
class FakePrinter:
    def __init__(self, reactor):
        self.reactor = reactor
        self.objects = {}
        self.handlers = {}
        self.shutdown = []
    def get_reactor(self): return self.reactor
    def add_object(self, name, obj): self.objects[name] = obj
    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)
    def invoke_shutdown(self, msg, details=None): self.shutdown.append(msg)
    _MISSING = object()
    def lookup_object(self, name, default=_MISSING):
        if name in self.objects: return self.objects[name]
        if default is FakePrinter._MISSING:
            raise Exception('Unable to lookup object \'%s\'' % (name,))
        return default
GETINT_CALLS = []
REPORT_TIME = [30]
class FakeConfig:
    def __init__(self, printer, name):
        self.printer = printer
        self.name = name
    def get_printer(self): return self.printer
    def get_name(self): return self.name
    def getint(self, option, default=None, minval=None, maxval=None, note_valid=True):
        GETINT_CALLS.append((option, default, minval, maxval))
        return REPORT_TIME[0]
_reactor = FakeReactor()
_printer = FakePrinter(_reactor)
_bus = types.ModuleType('extras.bus')
def _mcu_i2c_from_config(config, default_addr=None, default_speed=100000, async_write_only=False):
    return _i2c
_bus.MCU_I2C_from_config = _mcu_i2c_from_config
sys.modules['extras.bus'] = _bus
import extras.aht10 as aht10
";

fn py_new(py: &mut pyoracle::Oracle, section: &str, model: &str, reads: &str) {
    py.exec(PY_BASE).unwrap();
    py.exec(&format!(
        "READS.extend({reads})\n_config = FakeConfig(_printer, {section:?})\n_obj = aht10.{model}(_config)"
    ))
    .unwrap();
}

fn record(scenario: &str, function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.aht10".to_string(),
        scenario: scenario.to_string(),
        seq: 1,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.inner().to_string().parse().unwrap()),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(to_value).collect()),
        PyValue::Dict(items) => Value::Dict(
            items
                .iter()
                .map(|(k, v)| (to_value(k), to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "object".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn bytes_list(bytes: &[u8]) -> Value {
    Value::List(bytes.iter().map(|b| Value::Int(i64::from(*b))).collect())
}

fn writes_value(writes: &[Vec<u8>]) -> Value {
    Value::List(writes.iter().map(|w| bytes_list(w)).collect())
}

fn logs_value(logs: &[(String, String)]) -> Value {
    Value::List(
        logs.iter()
            .map(|(level, message)| {
                Value::Tuple(vec![Value::Str(level.clone()), Value::Str(message.clone())])
            })
            .collect(),
    )
}

fn getint_calls_value(
    calls: &[(String, Option<Option<i64>>, Option<i64>, Option<i64>)],
) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(option, default, minval, maxval)| {
                let opt = |value: &Option<Option<i64>>| match value {
                    Some(Some(n)) => Value::Int(*n),
                    Some(None) => Value::Null,
                    None => Value::Null,
                };
                let int_opt = |value: &Option<i64>| match value {
                    Some(n) => Value::Int(*n),
                    None => Value::Null,
                };
                Value::Tuple(vec![
                    Value::Str(option.clone()),
                    opt(default),
                    int_opt(minval),
                    int_opt(maxval),
                ])
            })
            .collect(),
    )
}

fn check_equal(scenario: &str, function: &str, path: &str, expected: &Value, actual: &Value) {
    assert_equal(&record(scenario, function), path, expected, actual).unwrap();
}

const INIT_FRAME: [u8; 6] = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00];

// ---------------------------------------------------------------------------
// Scenarios.
// ---------------------------------------------------------------------------

/// `AHT1x.__init__` plus the init sequence: one calibrated frame.
/// Compares the config reads, the object registration, the status dict,
/// the init I2C writes and the info log (`InitGroup`).
#[test]
fn init_success_aht1x() {
    clear_logs();
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    py_new(&mut py, "temperature_sensor bench", "AHT1x", "[[0x08, 0x64, 0x00, 0x10, 0x00, 0x00]]");
    py.exec("_obj._init_sensor()").unwrap();

    let (printer_raw, i2c, config, config_inner) = make_sensor(vec![
        ReadStep::Data(vec![0x08, 0x64, 0x00, 0x10, 0x00, 0x00]),
    ]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    obj.init_sensor().unwrap();

    let expected_status = py.eval("_obj.get_status(0)").unwrap();
    check_equal(
        "init",
        "get_status",
        "ret",
        &expected_status,
        &to_value(&obj.status()),
    );
    let expected_writes = py.eval("WRITES").unwrap();
    check_equal(
        "init",
        "writes",
        " ret",
        &expected_writes,
        &writes_value(&i2c.inner.writes.lock().unwrap()),
    );
    let expected_logs = py.eval("LOGS").unwrap();
    check_equal("init", "logs", " ret", &expected_logs, &logs_value(&logs()));
    let expected_calls = py.eval("GETINT_CALLS").unwrap();
    check_equal(
        "init",
        "getint calls",
        " ret",
        &expected_calls,
        &getint_calls_value(&config_inner.getint_calls.lock().unwrap()),
    );
    assert_eq!(printer_raw.object_names(), vec!["aht10 bench".to_string()]);
    assert_eq!(printer_raw.handler_count(EV_KLIPPY_CONNECT), 1);
    assert_eq!(obj.get_report_time_delta(), 30);
}

/// The same init flow for `AHT2x` (`CMD_INIT_AHT2X`) and `AHT3x` (pause
/// only, no init write).
#[test]
fn init_writes_per_model() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    for (ctor_py, ctor_rs) in [("AHT2x", "aht2x"), ("AHT3x", "aht3x")] {
        clear_logs();
        py_new(&mut py, "temperature_sensor bench", ctor_py, "[[0x08, 0, 0, 0, 0, 0]]");
        py.exec("_obj._init_sensor()").unwrap();

        let (_printer_raw, i2c, config, _config_inner) =
            make_sensor(vec![ReadStep::Data(INIT_FRAME.to_vec())]);
        let obj = match ctor_rs {
            "aht2x" => aht2x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap(),
            _ => aht3x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap(),
        };
        obj.init_sensor().unwrap();

        let expected_writes = py.eval("WRITES").unwrap();
        check_equal(
            "init per model",
            "writes",
            " ret",
            &expected_writes,
            &writes_value(&i2c.inner.writes.lock().unwrap()),
        );
        let expected_status = py.eval("_obj.get_status(0)").unwrap();
        check_equal(
            "init per model",
            "get_status",
            "ret",
            &expected_status,
            &to_value(&obj.status()),
        );
    }
}

/// Transient read problems: a `None` reply and a short frame are logged
/// and retried; a busy frame loops. The calibrated frame eventually
/// lands (`MakeMeasurementGroup`).
#[test]
fn measurement_retries() {
    clear_logs();
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    py_new(
        &mut py,
        "temperature_sensor bench",
        "AHT1x",
        "[[0x08, 0, 0, 0, 0, 0], None, [0x08, 0x01], [0x88, 0, 0, 0, 0, 0], [0x08, 0x7B, 0x00, 0x70, 0x00, 0x00]]",
    );
    py.exec("_obj._init_sensor()\n_result = _obj._make_measurement()").unwrap();

    let (_printer_raw, i2c, config, _config_inner) = make_sensor(vec![
        ReadStep::Data(INIT_FRAME.to_vec()),
        ReadStep::None,
        ReadStep::Data(vec![0x08, 0x01]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x08, 0x7B, 0x00, 0x70, 0x00, 0x00]),
    ]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    obj.init_sensor().unwrap();
    let result = obj.make_measurement();

    let expected_result = py.eval("_result").unwrap();
    check_equal(
        "retries",
        "make_measurement result",
        "ret",
        &expected_result,
        &Value::Bool(result),
    );
    let expected_status = py.eval("_obj.get_status(0)").unwrap();
    check_equal(
        "retries",
        "get_status",
        "ret",
        &expected_status,
        &to_value(&obj.status()),
    );
    let expected_logs = py.eval("LOGS").unwrap();
    check_equal("retries", "logs", " ret", &expected_logs, &logs_value(&logs()));
    let expected_read_calls = py.eval("READ_CALLS").unwrap();
    let read_calls = i2c.inner.read_calls.lock().unwrap();
    check_equal(
        "retries",
        "read calls",
        " ret",
        &expected_read_calls,
        &Value::List(
            read_calls
                .iter()
                .map(|(write, len)| Value::Tuple(vec![bytes_list(write), Value::Int(*len as i64)]))
                .collect(),
        ),
    );
}

/// Persistent busy: after `MAX_BUSY_CYCLES` the driver soft-resets and
/// the measurement fails (`MakeMeasurementGroup`).
#[test]
fn measurement_busy_reset() {
    clear_logs();
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    py_new(
        &mut py,
        "temperature_sensor bench",
        "AHT1x",
        "[[0x08, 0, 0, 0, 0, 0]] + [[0x88, 0, 0, 0, 0, 0]] * 10",
    );
    py.exec("_obj._init_sensor()\n_result = _obj._make_measurement()").unwrap();

    let (_printer_raw, i2c, config, _config_inner) = make_sensor(vec![
        ReadStep::Data(INIT_FRAME.to_vec()),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
        ReadStep::Data(vec![0x88, 0, 0, 0, 0, 0]),
    ]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    obj.init_sensor().unwrap();
    let result = obj.make_measurement();

    let expected_result = py.eval("_result").unwrap();
    check_equal(
        "busy reset",
        "make_measurement result",
        "ret",
        &expected_result,
        &Value::Bool(result),
    );
    let expected_writes = py.eval("WRITES").unwrap();
    check_equal(
        "busy reset",
        "writes",
        " ret",
        &expected_writes,
        &writes_value(&i2c.inner.writes.lock().unwrap()),
    );
    let expected_logs = py.eval("LOGS").unwrap();
    check_equal(
        "busy reset",
        "logs",
        " ret",
        &expected_logs,
        &logs_value(&logs()),
    );
}

/// A read error is caught inside the measurement: it is logged via
/// `logging.exception` and the measurement fails
/// (`MakeMeasurementGroup`).
#[test]
fn measurement_exception() {
    clear_logs();
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    py_new(
        &mut py,
        "temperature_sensor bench",
        "AHT1x",
        "[[0x08, 0, 0, 0, 0, 0]]",
    );
    py.exec("_obj._init_sensor()\nREADS.append(IOError('FakeI2C read failure'))\n_result = _obj._make_measurement()").unwrap();

    let (_printer_raw, i2c, config, _config_inner) = make_sensor(vec![
        ReadStep::Data(INIT_FRAME.to_vec()),
        ReadStep::Error(KlippyError::Mcu("FakeI2C read failure".to_string())),
    ]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    obj.init_sensor().unwrap();
    let result = obj.make_measurement();

    let expected_result = py.eval("_result").unwrap();
    check_equal(
        "exception",
        "make_measurement result",
        "ret",
        &expected_result,
        &Value::Bool(result),
    );
    // Only (level, formatted message) compare: the traceback body is
    // normalised by the parity table, and `klippy_api::log::exception`
    // formats the message at the call site like Python does.
    let expected_logs = py.eval("LOGS").unwrap();
    check_equal(
        "exception",
        "logs",
        " ret",
        &expected_logs,
        &logs_value(&logs()),
    );
}

/// The not-calibrated warning of `InitGroup`.
#[test]
fn init_not_calibrated() {
    clear_logs();
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    py_new(&mut py, "temperature_sensor bench", "AHT1x", "[[0x00, 0, 0, 0, 0, 0]]");
    py.exec("_obj._init_sensor()").unwrap();

    let (_printer_raw, i2c, config, _config_inner) =
        make_sensor(vec![ReadStep::Data(vec![0x00, 0, 0, 0, 0, 0])]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    obj.init_sensor().unwrap();

    let expected_logs = py.eval("LOGS").unwrap();
    check_equal(
        "not calibrated",
        "logs",
        " ret",
        &expected_logs,
        &logs_value(&logs()),
    );
}

/// Periodic sampling (`SampleAhtGroup`): success reports through the
/// callback, failure returns `NEVER` with the reading zeroed, and an
/// out-of-range temperature triggers the shutdown message.
#[test]
fn sampling_paths() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };

    // Successful sample.
    clear_logs();
    py_new(
        &mut py,
        "temperature_sensor bench",
        "AHT1x",
        "[[0x08, 0, 0, 0, 0, 0], [0x08, 0x64, 0x00, 0x10, 0x00, 0x00]]",
    );
    py.exec(
        "CB = []\n_obj.setup_callback(lambda pt, t: CB.append((pt, t)))\n_obj.setup_minmax(-100., 200.)\n_obj._init_sensor()\n_next = _obj._sample_aht(0.)",
    )
    .unwrap();

    let (printer_raw, i2c, config, _config_inner) = make_sensor(vec![
        ReadStep::Data(INIT_FRAME.to_vec()),
        ReadStep::Data(vec![0x08, 0x64, 0x00, 0x10, 0x00, 0x00]),
    ]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    let t0 = reactor::monotonic();
    obj.setup_minmax(-100., 200.);
    let seen = Arc::new(Mutex::new(Vec::new()));
    let seen_clone = seen.clone();
    obj.setup_callback(Box::new(move |print_time, temp| {
        seen_clone.lock().unwrap().push((print_time, temp));
    }));
    obj.init_sensor().unwrap();
    let next = obj.sample_aht(0.);
    let t1 = reactor::monotonic();

    // Callback temperature and print time check: the print time is
    // clock-derived on both sides (1000.0 in Python, the real clock
    // here), so only the temperature is oracle-compared.
    let expected_temps = py.eval("[t for _pt, t in CB]").unwrap();
    let seen = seen.lock().unwrap();
    assert_eq!(seen.len(), 1);
    check_equal(
        "sample",
        "callback temps",
        "ret",
        &expected_temps,
        &Value::List(vec![Value::float(seen[0].1)]),
    );
    assert!(
        seen[0].0 >= t0 && seen[0].0 <= t1,
        "print_time {:?} outside [{t0}, {t1}]",
        seen[0].0
    );
    assert_eq!(printer_raw.shutdown_messages(), Vec::<String>::new());
    // measured_time + report_time: clock-derived, checked against the
    // call window (Python returns 1000.0 + 30.0).
    let expected_next = py.eval("_next").unwrap();
    check_equal(
        "sample",
        "python next wake",
        "ret",
        &expected_next,
        &Value::float(1030.0),
    );
    assert!(
        next >= t0 + 30.0 && next <= t1 + 30.0,
        "next wake {next} outside window"
    );

    // Failed sample returns NEVER and zeroes the reading.
    clear_logs();
    py_new(&mut py, "temperature_sensor bench", "AHT1x", "[]");
    py.exec("_next = _obj._sample_aht(0.)").unwrap();

    let (_printer_raw, i2c, config, _config_inner) = make_sensor(Vec::new());
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    let next = obj.sample_aht(0.);
    assert_eq!(next, reactor::NEVER);
    let expected_next = py.eval("_next").unwrap();
    check_equal(
        "sample",
        "never wake",
        "ret",
        &expected_next,
        &Value::float(reactor::NEVER),
    );
    let expected_status = py.eval("_obj.get_status(0)").unwrap();
    check_equal(
        "sample",
        "zeroed status",
        "ret",
        &expected_status,
        &to_value(&obj.status()),
    );

    // Out-of-range temperature invokes shutdown.
    clear_logs();
    py_new(
        &mut py,
        "temperature_sensor bench",
        "AHT1x",
        "[[0x08, 0, 0, 0, 0, 0], [0x08, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF]]",
    );
    // The callback must be installed before the out-of-range sample:
    // `heaters` always calls `setup_callback` first, and `_sample_aht`
    // unconditionally invokes it (`aht10.py:154`), so a missing callback
    // raises `TypeError: 'NoneType' object is not callable`.
    py.exec(
        "CB = []\n_obj.setup_callback(lambda pt, t: CB.append((pt, t)))\n_obj.setup_minmax(0., 10.)\n_obj._init_sensor()\n_next = _obj._sample_aht(0.)",
    )
    .unwrap();

    let (printer_raw, i2c, config, _config_inner) = make_sensor(vec![
        ReadStep::Data(INIT_FRAME.to_vec()),
        ReadStep::Data(vec![0x08, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF]),
    ]);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    obj.setup_minmax(0., 10.);
    obj.setup_callback(Box::new(|_, _| {}));
    obj.init_sensor().unwrap();
    let _next = obj.sample_aht(0.);

    let expected_shutdown = py.eval("_printer.shutdown").unwrap();
    let shutdown = printer_raw.shutdown_messages();
    check_equal(
        "sample",
        "shutdown message",
        " ret",
        &expected_shutdown,
        &Value::List(shutdown.iter().map(|m| Value::Str(m.clone())).collect()),
    );
}

/// `load_config` (`aht10.py:194`): registers the four sensor factories
/// with the `heaters` object; with `heaters` missing it raises the lookup
/// error. Factory names and the missing-object message are oracle
/// compared against the Python `load_config`.
#[test]
fn load_config_entry_point() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: pyoracle unavailable outside the oracle image");
        return;
    };
    let factory = module_factory();
    let load_config = factory.load_config.expect("load_config entry point");

    struct FakeHeatersInner {
        factories: Mutex<Vec<String>>,
    }
    struct FakeHeaters(Arc<FakeHeatersInner>);
    impl PrinterHeatersApi for FakeHeaters {
        fn add_sensor_factory(
            &self,
            sensor_type: &str,
            _factory: extras::heaters::SensorFactory,
        ) {
            self.0.factories.lock().unwrap().push(sensor_type.to_string());
        }
        fn setup_heater(
            &self,
            _config: &Config,
            _gcode_id: Option<String>,
        ) -> KResult<Arc<Heater>> {
            Err(KlippyError::config("FakeHeaters: setup_heater not scripted"))
        }
        fn get_all_heaters(&self) -> Vec<String> {
            Vec::new()
        }
        fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
            Err(KlippyError::config("FakeHeaters: lookup_heater not scripted"))
        }
        fn setup_sensor(
            &self,
            _config: &Config,
        ) -> KResult<Arc<dyn extras::heaters::TemperatureSensorApi>> {
            Err(KlippyError::config("FakeHeaters: setup_sensor not scripted"))
        }
        fn register_sensor(
            &self,
            _config: &Config,
            _psensor: Arc<dyn extras::heaters::SensorApi>,
            _gcode_id: Option<String>,
        ) -> KResult<()> {
            Ok(())
        }
        fn register_monitor(&self, _config: &Config) {}
        fn turn_off_all_heaters(&self, _print_time: f64) {}
        fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
            Ok(())
        }
    }
    impl PrinterObject for FakeHeaters {
        klippy_api::printer_object_base!();
        klippy_api::printer_interfaces!(dyn PrinterHeatersApi);
    }

    // heaters present.
    py.exec(PY_BASE).unwrap();
    py.exec(
        "class _FakeHeaters:\n   def add_sensor_factory(self, sensor_type, factory): FACTORY_NAMES.append(sensor_type)\nFACTORY_NAMES = []\n_printer.add_object('heaters', _FakeHeaters())\naht10.load_config(FakeConfig(_printer, 'aht10'))",
    )
    .unwrap();

    let (_printer_raw, _reactor) = FakePrinter::new();
    let printer: Printer = _printer_raw.clone();
    let heaters_inner = Arc::new(FakeHeatersInner {
        factories: Mutex::new(Vec::new()),
    });
    let heaters = Arc::new(FakeHeaters(heaters_inner.clone())) as ObjectRef;

    printer.add_object("heaters", heaters).unwrap();
    let (config, _) = fake_config(&printer, "aht10", 30, None);
    let result = load_config(&config).unwrap();
    assert!(result.is_none());
    let expected_factories = py.eval("FACTORY_NAMES").unwrap();
    let actual_factories = heaters_inner.factories.lock().unwrap().clone();
    check_equal(
        "load_config",
        "factory names",
        "ret",
        &expected_factories,
        &Value::List(
            actual_factories
                .iter()
                .map(|name| Value::Str(name.clone()))
                .collect(),
        ),
    );

    // heaters missing.
    py.exec(PY_BASE).unwrap();
    py.exec(
        "try:\n   aht10.load_config(FakeConfig(_printer, 'aht10'))\n   _err = None\nexcept Exception as e:\n   _err = str(e)",
    )
    .unwrap();
    let (printer_raw, _reactor) = FakePrinter::new();
    let printer: Printer = printer_raw.clone();
    let (config, _) = fake_config(&printer, "aht10", 30, None);
    let err = match load_config(&config) {
        Ok(_) => panic!("expected lookup error"),
        Err(err) => err,
    };
    assert_eq!(err.class_name(), "Error");
    let expected_err = py.eval("_err").unwrap();
    check_equal(
        "load_config",
        "missing heaters message",
        "ret",
        &expected_err,
        &Value::Str(err.message().to_string()),
    );
}

/// Construction reads the configured report time; a `getint` failure
/// aborts construction exactly as Python propagates the error.
#[test]
fn constructor_config_paths() {
    let (printer_raw, _reactor) = FakePrinter::new();
    let printer: Printer = printer_raw.clone();
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer.clone() });
    let i2c = FakeI2c::new(mcu, Vec::new());
    let (config, _) = fake_config(&printer, "temperature_sensor bench", 45, None);
    let obj = aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())).unwrap();
    assert_eq!(obj.get_report_time_delta(), 45);
    let (printer_raw, _reactor) = FakePrinter::new();
    let printer: Printer = printer_raw.clone();
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer.clone() });
    let i2c = FakeI2c::new(mcu, Vec::new());
    let (config, _) = fake_config(
        &printer,
        "temperature_sensor bench",
        30,
        Some(KlippyError::config("Invalid value")),
    );
    let err = match aht1x(&config, FakeI2c::dyn_i2c(i2c.clone())) {
        Ok(_) => panic!("expected construction error"),
        Err(err) => err,
    };
    assert_eq!(err.class_name(), "Error");
    assert_eq!(err.message(), "Invalid value");
}
