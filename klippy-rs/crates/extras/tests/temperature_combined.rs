//! Parity tests for `extras::temperature_combined` (port of `klippy/extras/temperature_combined.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). Tests skip with a message when
//! `KLIPPY_PYTHON` is unset (outside the oracle image), following the wave
//! contract. There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so oracle comparisons are the sole source of truth.
//!
//! What is compared against the oracle, per unit in `parity/inventory/units.json`:
//! - `ModuleHeader`: `REPORT_TIME` (0.300) and module importability.
//! - `InitGroup` (lines 11-68): construction reads `sensor_list` (required list),
//!   `maximum_deviation` (required float, `above=0.`), `combination_method`
//!   (required choice `min`/`max`/`mean`); `self.name` is the last token of the
//!   section name; `self.sensor = self`; defaults `last_temp/min_temp/max_temp=0.0`,
//!   `humidity/pressure/gas=None`; registration of `temperature_combined <name>`
//!   object, the `NEVER`-armed reactor timer and the `klippy:connect` /
//!   `klippy:ready` handlers; `_handle_connect` error paths (no get_status,
//!   no temperature, temperature is None) and the happy path; `_handle_ready`
//!   arming the timer at `monotonic()+1.`.
//! - `SetupMinmaxGroup` (lines 69-99): `setup_minmax`, `setup_callback`,
//!   `get_report_time_delta` (=0.300), `update_temp` (deviation shutdown message
//!   with `%0.1f` formatting, `if temp:` truthiness with 0.0, `min`/`max`/`mean`
//!   selection) and `get_temp` (always `(last_temp, 0.)`).
//! - `UpdateAdditionalGroup` (lines 100-145): `update_additional` for
//!   humidity/pressure/gas (None-skip, empty-skip, `if x:` truthiness, same
//!   combination method) and `get_status` (temperature rounded with
//!   `round(x, 2)`, optional fields present only when set, insertion order
//!   temperature/humidity/pressure/gas).
//! - `TemperatureUpdateEventGroup` (lines 146-173): `_temperature_update_event`
//!   calls `update_temp` then `update_additional`, shuts down below `min_temp`
//!   and above `max_temp` with the exact `%0.1f` messages, invokes
//!   `temperature_callback(mcu.estimated_print_time(monotonic()), last_temp)`
//!   and returns `monotonic() + REPORT_TIME`.
//! - `Mean` (lines 174-182): `mean([])` is `None`, `mean([a,b])` is
//!   `sum/len` with plain left-to-right float summation.
//!
//! Rust fakes for `PrinterApi`, `ConfigApi` and `McuApi` live in this file and
//! are driven with the same inputs as the Python fakes built in `SETUP`.

use extras::heaters::{Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi};
use extras::temperature_combined::{mean, CombinedSensor, REPORT_TIME};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts,
    GetIntOpts, GetListOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::McuApi;
use klippy_api::object::{interface, ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, PrinterApi};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.temperature_combined as _mod
PrinterSensorCombined = _mod.PrinterSensorCombined
load_config = _mod.load_config
mean = _mod.mean
REPORT_TIME = _mod.REPORT_TIME

class Error(Exception):
    pass

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self): self.timers = []; self.updates = []; self.t = 1000.0
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb); return ('timer', len(self.timers)-1)
    def update_timer(self, timer, waketime): self.updates.append(waketime)
    def monotonic(self): return self.t

class FakePrinter:
    def __init__(self, reactor, objects=None):
        self._reactor = reactor
        self.objects = dict(objects or {})
        self.handlers = []
        self.shutdowns = []
    def get_reactor(self): return self._reactor
    def config_error(self, msg): return Error(msg)
    def add_object(self, name, obj): self.objects[name] = obj
    def lookup_object(self, name, default=None):
        if name in self.objects: return self.objects[name]
        raise Error("Object '%s' not found" % (name,))
    def load_object(self, config, section):
        return self.objects["heaters"]
    def invoke_shutdown(self, msg, details=None): self.shutdowns.append(msg)
    def register_event_handler(self, event, cb): self.handlers.append((event, cb))

class FakeMcu:
    def estimated_print_time(self, eventtime): return eventtime + 5.0

class FakeSensor:
    def __init__(self, status): self._status = status
    def get_status(self, eventtime=None): return self._status

class NoStatusSensor:
    pass

class FakeHeaters:
    def __init__(self): self.factories = {}
    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.factories[sensor_type] = sensor_factory

class FakeConfig:
    def __init__(self, name, printer, opts):
        self._name = name; self._printer = printer; self._opts = opts
    def get_printer(self): return self._printer
    def get_name(self): return self._name
    def getlist(self, opt):
        if opt not in self._opts.d:
            raise Error("Option '%s' in section '%s' must be specified" % (opt, self._name))
        return self._opts.getlist(opt)
    def getfloat(self, opt, above=None, **kw):
        if opt not in self._opts.d:
            raise Error("Option '%s' in section '%s' must be specified" % (opt, self._name))
        v = self._opts.getfloat(opt)
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (opt, self._name, above))
        return v
    def getchoice(self, opt, choices):
        if opt not in self._opts.d:
            raise Error("Option '%s' in section '%s' must be specified" % (opt, self._name))
        v = self._opts.getchoice(opt)
        if v not in choices:
            raise Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (v, opt, self._name))
        return choices[v]

class Opts:
    def __init__(self, d): self.d = d
    def getlist(self, opt):
        if opt not in self.d: raise Error("Option '%s' is required" % opt)
        return self.d[opt]
    def getfloat(self, opt):
        if opt not in self.d: raise Error("Option '%s' is required" % opt)
        return self.d[opt]
    def getchoice(self, opt):
        if opt not in self.d: raise Error("Option '%s' is required" % opt)
        return self.d[opt]
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("temperature_combined fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping temperature_combined oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn record(seq: u64, function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.temperature_combined".to_string(),
        scenario: "oracle".to_string(),
        seq,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

// Value helpers: floats compare by bits+repr via recording::Value::float.
fn f(x: f64) -> Value {
    Value::float(x)
}
fn s(x: &str) -> Value {
    Value::str(x)
}
fn tuple2(a: Value, b: Value) -> Value {
    Value::Tuple(vec![a, b])
}
fn assert_value_eq(seq: u64, function: &str, expected: &Value, actual: &Value) {
    let rec = record(seq, function);
    assert_equal(&rec, "ret", expected, actual).expect(function);
}

// ---------------------------------------------------------------------------
// Rust-side fakes for the contract traits.
// ---------------------------------------------------------------------------

#[derive(Clone, Default)]
struct FakePrinterInner {
    objects: Vec<(String, ObjectRef)>,
    shutdowns: Vec<String>,
    reactor: Option<Reactor>,
}

struct FakePrinter {
    inner: Mutex<FakePrinterInner>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            inner: Mutex::new(FakePrinterInner { reactor: Some(reactor), ..Default::default() }),
            handlers: Mutex::new(Vec::new()),
        })
    }
    fn objects(&self) -> Vec<(String, ObjectRef)> {
        self.inner.lock().unwrap().objects.clone()
    }
    fn shutdowns(&self) -> Vec<String> {
        self.inner.lock().unwrap().shutdowns.clone()
    }
    fn handler_events(&self) -> Vec<String> {
        self.handlers.lock().unwrap().iter().map(|(e, _)| e.clone()).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.inner.lock().unwrap().reactor.clone().expect("reactor")
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut inner = self.inner.lock().unwrap();
        if inner.objects.iter().any(|(n, _)| n == name) {
            return Err(KlippyError::config(format!("Object '{}' already added", name)));
        }
        inner.objects.push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let inner = self.inner.lock().unwrap();
        match inner.objects.iter().find(|(n, _)| n == name) {
            Some((_, obj)) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!("Object '{}' not found", name))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let inner = self.inner.lock().unwrap();
        match module {
            None => inner.objects.clone(),
            Some(m) => inner
                .objects
                .iter()
                .filter(|(n, _)| n == m || n.starts_with(&format!("{} ", m)))
                .cloned()
                .collect(),
        }
    }
    fn load_object(&self, _config: &Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.inner.lock().unwrap().shutdowns.push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.invoke_shutdown(msg, PyValue::None);
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().unwrap().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, _params: &[klippy_api::object::EventArg]) -> KResult<Vec<PyValue>> {
        let mut hs = self.handlers.lock().unwrap();
        let mut out = Vec::new();
        let mut i = 0;
        while i < hs.len() {
            if hs[i].0 == event {
                out.push((hs[i].1)(&[])?);
            }
            i += 1;
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfigWrapper {
    name: String,
    printer: Arc<FakePrinter>,
    /// section options keyed by name
    sensor_list: Option<Vec<String>>,
    maximum_deviation: Option<f64>,
    combination_method: Option<String>,
}

impl FakeConfigWrapper {
    fn new(
        name: &str,
        printer: Arc<FakePrinter>,
        sensor_list: Option<Vec<String>>,
        maximum_deviation: Option<f64>,
        combination_method: Option<String>,
    ) -> Config {
        Arc::new(FakeConfigWrapper {
            name: name.to_string(),
            printer,
            sensor_list,
            maximum_deviation,
            combination_method,
        }) as Config
    }
}

impl ConfigApi for FakeConfigWrapper {
    fn get_printer(&self) -> Arc<dyn PrinterApi> {
        Arc::clone(&self.printer) as Arc<dyn PrinterApi>
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Err(KlippyError::config("unexpected get"))
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Err(KlippyError::config("unexpected getint"))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        if option != "maximum_deviation" {
            return Err(KlippyError::config(format!("unexpected getfloat '{}'", option)));
        }
        let v = self.maximum_deviation;
        match v {
            None => {
                match opts.default {
                    None => Err(KlippyError::config(format!("Option '{}' in section '{}' must be specified", option, self.name))),
                    Some(d) => Ok(d),
                }
            }
            Some(v) => {
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option,
                            self.name,
                            pycompat::text::float_repr(above)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(KlippyError::config("unexpected getboolean"))
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        if option != "combination_method" {
            return Err(KlippyError::config(format!("unexpected getchoice '{}'", option)));
        }
        let keys: Vec<PyValue> = match choices {
            PyValue::Dict(d) => d.keys().cloned().collect(),
            _ => Vec::new(),
        };
        let key_strs: Vec<String> = keys
            .iter()
            .filter_map(|k| match k {
                PyValue::Str(s) => Some(s.clone()),
                _ => None,
            })
            .collect();
        match &self.combination_method {
            None => match opts.default {
                None => Err(KlippyError::config(format!("Option '{}' in section '{}' must be specified", option, self.name))),
                Some(_) => Ok(PyValue::None),
            },
            Some(v) => {
                if key_strs.contains(v) {
                    Ok(PyValue::Str(v.clone()))
                } else {
                    Err(KlippyError::config(format!(
                        "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                        v,
                        option,
                        self.name
                    )))
                }
            }
        }
    }
    fn getlists(&self, _option: &str, _opts: klippy_api::config::GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::config("unexpected getlists"))
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        if option != "sensor_list" {
            return Err(KlippyError::config(format!("unexpected getlist '{}'", option)));
        }
        match &self.sensor_list {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::config(format!("Option '{}' in section '{}' must be specified", option, self.name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::config("unexpected getintlist"))
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::config("unexpected getfloatlist"))
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("unused in fake")
    }
    fn has_section(&self, _section: &str) -> bool {
        true
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

// A fake sensor whose runtime status can change after connect.
struct FakeSensor {
    status: klippy_api::GCell<PyValue>,
}

impl FakeSensor {
    fn dict(pairs: Vec<(&str, PyValue)>) -> Arc<FakeSensor> {
        let mut m = IndexMap::new();
        for (k, v) in pairs {
            m.insert(PyValue::Str(k.to_string()), v);
        }
        Arc::new(FakeSensor { status: klippy_api::GCell::new(PyValue::Dict(m)) })
    }
}

impl PrinterObject for FakeSensor {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.status.cloned())
    }
}

// A sensor with no get_status (exercises the `hasattr` false branch).
struct NoStatusSensor;

impl PrinterObject for NoStatusSensor {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
}

// A fake MCU.
struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Arc<dyn PrinterApi> {
        panic!("unused in fake")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: klippy_api::mcu::PinParams) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Mcu("unused".into()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Err(KlippyError::Mcu("unused".into()))
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
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
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<klippy_api::mcu::CommandQueue>) -> KResult<klippy_api::mcu::CommandWrapper> {
        Err(KlippyError::Mcu("unused".into()))
    }
    fn lookup_query_command(&self, _msgformat: &str, _respformat: &str, _opts: klippy_api::mcu::LookupQueryCommandOpts) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(KlippyError::Mcu("unused".into()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        klippy_api::mcu::CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("unused in fake")
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
        Err(KlippyError::Mcu("unused".into()))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        // Test double: print time = eventtime + 5 (matches the Python fake).
        eventtime + 5.0
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        0
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Err(KlippyError::Mcu("unused".into()))
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
        Err(KlippyError::Mcu("unused".into()))
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

// A fake `heaters` object: records the sensor factories registered
// with it (`PrinterHeatersApi::add_sensor_factory`); every other
// method is unused by this module's `load_config`.
struct FakeHeatersObj {
    factories: Mutex<Vec<(String, SensorFactory)>>,
}

impl PrinterHeatersApi for FakeHeatersObj {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories.lock().unwrap().push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        Err(KlippyError::config("unused"))
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        Err(KlippyError::config("unused"))
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Err(KlippyError::config("unused"))
    }
    fn register_sensor(&self, _config: &Config, _psensor: Arc<dyn SensorApi>, _gcode_id: Option<String>) -> KResult<()> {
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeHeatersObj; dyn PrinterHeatersApi);
fn rx(v: pyoracle::OracleResult) -> Value {
    match v {
        Ok(val) => val,
        Err(e) => Value::Str(format!("{}: {}", e.class_name, e.message)),
    }
}


// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn mean_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<Vec<f64>> = vec![
        vec![],
        vec![10.0],
        vec![10.0, 20.0],
        vec![1.5, 2.5, 3.0],
        vec![-5.0, 5.0],
    ];
    for (i, vals) in cases.iter().enumerate() {
        let list = vals.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(",");
        let expected = rx(py.eval(&format!("mean([{}])", list)));
        let actual = match mean(vals) {
            Some(m) => f(m),
            None => Value::Null,
        };
        assert_value_eq(i as u64, "mean", &expected, &actual);
    }
}

#[test]
fn report_time_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let expected = rx(py.eval("REPORT_TIME"));
    assert_value_eq(0, "REPORT_TIME", &expected, &f(REPORT_TIME));
}

#[test]
fn construction_and_registration() {
    let Some(mut py) = oracle() else { return };
    py.exec("r = FakeReactor(); p = FakePrinter(r); p.objects['temp_sensor'] = FakeSensor({'temperature': 25.0}); c = FakeConfig('temperature_combined hot', p, Opts({'sensor_list': ['temp_sensor'], 'maximum_deviation': 2.0, 'combination_method': 'mean'})); s = PrinterSensorCombined(c)").expect("py construction");
    let py_name = rx(py.eval("s.name"));
    let py_sensor_is_self = rx(py.eval("s.sensor is s"));
    let py_last_temp = rx(py.eval("s.last_temp"));
    let py_status = rx(py.eval("s.get_status(0.0)"));
    let py_objs = rx(py.eval("list(p.objects.keys())"));
    let py_handlers = rx(py.eval("[e for e,_ in p.handlers]"));
    let py_delta = rx(py.eval("s.get_report_time_delta()"));
    let py_temp = rx(py.eval("s.get_temp(0.0)"));

    let reactor = Reactor::new(false);
    let printer = FakePrinter::new(reactor.clone());
    let sensor = FakeSensor::dict(vec![("temperature", PyValue::Float(25.0))]);
    printer.add_object("temp_sensor", sensor as ObjectRef).unwrap();
    let config = FakeConfigWrapper::new(
        "temperature_combined hot",
        Arc::clone(&printer),
        Some(vec!["temp_sensor".to_string()]),
        Some(2.0),
        Some("mean".to_string()),
    );
    let combined = CombinedSensor::new(&config).unwrap();
    let combined = Arc::new(combined);
    combined.add_object().unwrap();
    combined.init().unwrap();
    assert_value_eq(1, "name", &py_name, &s(combined.name()));
    assert_value_eq(2, "sensor_is_self", &py_sensor_is_self, &Value::Bool(combined.sensor_is_self()));
    assert_value_eq(3, "last_temp", &py_last_temp, &f(combined.get_temp(0.0).0));
    assert_value_eq(4, "status_defaults", &py_status, &pyvalue_to_recording(&combined.status_dict()));
    let expected_objs = Value::List(printer.objects().iter().map(|(n, _)| Value::str(n)).collect());
    assert_value_eq(5, "objects", &py_objs, &expected_objs);
    let expected_handlers = Value::List(printer.handler_events().iter().map(|e| Value::str(e)).collect());
    assert_value_eq(6, "handlers", &py_handlers, &expected_handlers);
    assert_value_eq(7, "report_delta", &py_delta, &f(combined.get_report_time_delta()));
    let (lt, zt) = combined.get_temp(0.0);
    assert_value_eq(8, "get_temp", &py_temp, &tuple2(f(lt), f(zt)));

    // min/max combination through the oracle with two sensors.
    let _ = py.exec("p2 = FakePrinter(FakeReactor()); p2.objects['s1'] = FakeSensor({'temperature': 10.0}); p2.objects['s2'] = FakeSensor({'temperature': 20.0}); cmin = PrinterSensorCombined(FakeConfig('temperature_combined x', p2, Opts({'sensor_list': ['s1','s2'], 'maximum_deviation': 100.0, 'combination_method': 'min'}))); cmin._handle_connect(); cmin.update_temp(0.0); exp_min = cmin.get_temp(0.0); cmax = PrinterSensorCombined(FakeConfig('temperature_combined x', p2, Opts({'sensor_list': ['s1','s2'], 'maximum_deviation': 100.0, 'combination_method': 'max'}))); cmax._handle_connect(); cmax.update_temp(0.0); exp_max = cmax.get_temp(0.0)").expect("py minmax");
    let exp_min = rx(py.eval("exp_min"));
    let exp_max = rx(py.eval("exp_max"));
    let (s1, s2) = (FakeSensor::dict(vec![("temperature", PyValue::Float(10.0))]), FakeSensor::dict(vec![("temperature", PyValue::Float(20.0))]));
    let printer2 = FakePrinter::new(Reactor::new(false));
    printer2.add_object("s1", s1 as ObjectRef).unwrap();
    printer2.add_object("s2", s2 as ObjectRef).unwrap();
    let cfg_min = FakeConfigWrapper::new("temperature_combined x", Arc::clone(&printer2), Some(vec!["s1".into(), "s2".into()]), Some(100.0), Some("min".into()));
    let cmin = Arc::new(CombinedSensor::new(&cfg_min).unwrap());
    cmin.handle_connect().unwrap();
    cmin.update_temp(0.0);
    let (tmin, _) = cmin.get_temp(0.0);
    assert_value_eq(9, "min_temp", &exp_min, &tuple2(f(tmin), f(0.0)));
    let cfg_max = FakeConfigWrapper::new("temperature_combined x", Arc::clone(&printer2), Some(vec!["s1".into(), "s2".into()]), Some(100.0), Some("max".into()));
    let cmax = Arc::new(CombinedSensor::new(&cfg_max).unwrap());
    cmax.handle_connect().unwrap();
    cmax.update_temp(0.0);
    let (tmax, _) = cmax.get_temp(0.0);
    assert_value_eq(10, "max_temp", &exp_max, &tuple2(f(tmax), f(0.0)));
}

#[test]
fn update_additional_and_status() {
    let Some(mut py) = oracle() else { return };
    py.exec("r2 = FakeReactor(); p2 = FakePrinter(r2); p2.objects['a'] = FakeSensor({'temperature': 21.0, 'humidity': 40.0, 'pressure': 1013.0, 'gas': 5.0}); p2.objects['b'] = FakeSensor({'temperature': 23.0, 'humidity': 60.0, 'pressure': None, 'gas': 15.0}); c2 = PrinterSensorCombined(FakeConfig('temperature_combined y', p2, Opts({'sensor_list': ['a','b'], 'maximum_deviation': 10.0, 'combination_method': 'mean'}))); c2._handle_connect(); c2.update_temp(1.0); c2.update_additional(1.0); exp_status = c2.get_status(1.0); exp_temp = c2.get_temp(999.0)").expect("py additional");
    let exp_status = rx(py.eval("exp_status"));
    let exp_temp = rx(py.eval("exp_temp"));

    let reactor = Reactor::new(false);
    let printer = FakePrinter::new(reactor);
    let a = FakeSensor::dict(vec![
        ("temperature", PyValue::Float(21.0)),
        ("humidity", PyValue::Float(40.0)),
        ("pressure", PyValue::Float(1013.0)),
        ("gas", PyValue::Float(5.0)),
    ]);
    let b = FakeSensor::dict(vec![
        ("temperature", PyValue::Float(23.0)),
        ("humidity", PyValue::Float(60.0)),
        ("pressure", PyValue::None),
        ("gas", PyValue::Float(15.0)),
    ]);
    printer.add_object("a", a as ObjectRef).unwrap();
    printer.add_object("b", b as ObjectRef).unwrap();
    let cfg = FakeConfigWrapper::new("temperature_combined y", Arc::clone(&printer), Some(vec!["a".into(), "b".into()]), Some(10.0), Some("mean".into()));
    let c = Arc::new(CombinedSensor::new(&cfg).unwrap());
    c.handle_connect().unwrap();
    c.update_temp(1.0);
    c.update_additional(1.0);
    let st = c.status_dict();
    assert_value_eq(2, "status_additional", &exp_status, &pyvalue_to_recording(&st));
    let (t, z) = c.get_temp(999.0);
    assert_value_eq(3, "get_temp_additional", &exp_temp, &tuple2(f(t), f(z)));
}

#[test]
fn update_event_shutdowns_and_callback() {
    let Some(mut py) = oracle() else { return };
    py.exec("r3 = FakeReactor(); p3 = FakePrinter(r3); p3.objects['mcu'] = FakeMcu(); p3.objects['a'] = FakeSensor({'temperature': 100.0}); e1 = PrinterSensorCombined(FakeConfig('temperature_combined z', p3, Opts({'sensor_list': ['a'], 'maximum_deviation': 10.0, 'combination_method': 'mean'}))); e1._handle_connect(); e1.setup_minmax(0.0, 50.0); calls = []; e1.setup_callback(lambda pt, t: calls.append((pt, t))); wake1 = e1._temperature_update_event(10.0); exp_shutdowns1 = list(p3.shutdowns); exp_wake1 = wake1; exp_call_pt_delta = calls[0][0] - r3.monotonic(); exp_call_temp = calls[0][1]").expect("py event above");
    let exp_shutdowns1 = rx(py.eval("exp_shutdowns1"));
    let exp_wake1 = rx(py.eval("exp_wake1"));
    let exp_call_pt_delta = rx(py.eval("exp_call_pt_delta"));
    let exp_call_temp = rx(py.eval("exp_call_temp"));

    let reactor = Reactor::new(false);
    let printer = FakePrinter::new(reactor.clone());
    let mcu = Arc::new(FakeMcu);
    printer.add_object("mcu", mcu as ObjectRef).unwrap();
    let a = FakeSensor::dict(vec![("temperature", PyValue::Float(100.0))]);
    printer.add_object("a", a as ObjectRef).unwrap();
    let cfg = FakeConfigWrapper::new("temperature_combined z", Arc::clone(&printer), Some(vec!["a".into()]), Some(10.0), Some("mean".into()));
    let c = Arc::new(CombinedSensor::new(&cfg).unwrap());
    c.handle_connect().unwrap();
    c.setup_minmax(0.0, 50.0);
    let cb_calls: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let cc = Arc::clone(&cb_calls);
    c.setup_callback(Box::new(move |pt, t| cc.lock().unwrap().push((pt, t))));
    let before = reactor.monotonic();
    let wake = c.temperature_update_event(10.0);
    let after = reactor.monotonic();
    let rust_shutdowns1 = Value::List(printer.shutdowns().iter().map(|m| Value::str(m)).collect());
    assert_value_eq(10, "shutdown_above", &exp_shutdowns1, &rust_shutdowns1);
    // `_temperature_update_event` returns `monotonic() + REPORT_TIME`
    // (the oracle confirms `exp_wake1` equals that formula on its fake
    // clock). On the Rust side the clock is the real reactor, so the
    // absolute value cannot be compared; the offset from the tick
    // before the call and the oracle-verified REPORT_TIME are.
    assert!(
        (wake - before - REPORT_TIME).abs() < 0.05,
        "wake {wake} not monotonic()+REPORT_TIME of {before}+{REPORT_TIME}"
    );
    assert!((after - before).abs() < 1.0, "reactor clock moved implausibly");
    let calls = cb_calls.lock().unwrap();
    assert_eq!(calls.len(), 1);
    assert_value_eq(12, "callback_temp", &exp_call_temp, &f(calls[0].1));
    // The oracle callback print time was `estimated_print_time` of its
    // monotonic reading: `calls[0][0] - r3.monotonic()` there; on the
    // Rust side the same argument is the same formula on the real
    // clock near `wake - REPORT_TIME`.
    // Oracle-side sanity: on its fake clock the event returned
    // `monotonic() + REPORT_TIME` and the mcu's estimated-print-time
    // delta was its `eventtime + 5.0` fake.
    assert_value_eq(13, "oracle_wake_formula", &rx(py.eval("r3.monotonic() + REPORT_TIME")), &exp_wake1);
    assert_value_eq(14, "oracle_mcu_delta", &f(5.0), &exp_call_pt_delta);
    assert!(
        (calls[0].0 - (wake - REPORT_TIME) - 5.0).abs() < 0.05,
        "callback print time {} not wake + 5.0",
        calls[0].0
    );
    let exp_above_temp = rx(py.eval("e1.get_temp(0.0)"));
    let (at, az) = c.get_temp(0.0);
    assert_value_eq(16, "above_temp", &exp_above_temp, &tuple2(f(at), f(az)));

    // Below minimum.
    py.exec("p4 = FakePrinter(FakeReactor()); p4.objects['mcu'] = FakeMcu(); p4.objects['b'] = FakeSensor({'temperature': -10.0}); e2 = PrinterSensorCombined(FakeConfig('temperature_combined z', p4, Opts({'sensor_list': ['b'], 'maximum_deviation': 10.0, 'combination_method': 'mean'}))); e2._handle_connect(); e2.setup_minmax(10.0, 50.0); e2.setup_callback(lambda pt, t: None); e2._temperature_update_event(11.0); exp_shutdowns2 = list(p4.shutdowns)").expect("py below");
    let exp_shutdowns2 = rx(py.eval("exp_shutdowns2"));
    let b = FakeSensor::dict(vec![("temperature", PyValue::Float(-10.0))]);
    printer.add_object("b", b as ObjectRef).unwrap();
    let cfg2 = FakeConfigWrapper::new("temperature_combined z", Arc::clone(&printer), Some(vec!["b".into()]), Some(10.0), Some("mean".into()));
    let c2 = Arc::new(CombinedSensor::new(&cfg2).unwrap());
    c2.handle_connect().unwrap();
    c2.setup_minmax(10.0, 50.0);
    c2.setup_callback(Box::new(|_, _| {}));
    c2.temperature_update_event(11.0);
    let rust_shutdowns2 = Value::List(printer.shutdowns().iter().filter(|m| m.contains("below")).map(|m| Value::str(m)).collect());
    // Compare only the below-minimum entry count and text against the oracle's identical entry.
    let exp_below = Value::List(match &exp_shutdowns2 {
        Value::List(items) => items.iter().filter(|v| matches!(v, Value::Str(sx) if sx.contains("below"))).cloned().collect(),
        other => vec![other.clone()],
    });
    assert_value_eq(13, "shutdown_below", &exp_below, &rust_shutdowns2);

    // Deviation exceeded; compare shutdown list and resulting temp.
    py.exec("p5 = FakePrinter(FakeReactor()); p5.objects['a'] = FakeSensor({'temperature': 10.0}); p5.objects['b'] = FakeSensor({'temperature': 30.0}); e3 = PrinterSensorCombined(FakeConfig('temperature_combined z', p5, Opts({'sensor_list': ['a','b'], 'maximum_deviation': 5.0, 'combination_method': 'mean'}))); e3._handle_connect(); e3.update_temp(0.0); exp_shutdowns3 = list(p5.shutdowns); exp_dev_temp = e3.get_temp(0.0)").expect("py deviation");
    let exp_shutdowns3 = rx(py.eval("exp_shutdowns3"));
    let exp_dev_temp = rx(py.eval("exp_dev_temp"));
    let p2 = FakePrinter::new(Reactor::new(false));
    p2.add_object("a", FakeSensor::dict(vec![("temperature", PyValue::Float(10.0))]) as ObjectRef).unwrap();
    p2.add_object("b", FakeSensor::dict(vec![("temperature", PyValue::Float(30.0))]) as ObjectRef).unwrap();
    let cfg3 = FakeConfigWrapper::new("temperature_combined z", Arc::clone(&p2), Some(vec!["a".into(), "b".into()]), Some(5.0), Some("mean".into()));
    let c3 = Arc::new(CombinedSensor::new(&cfg3).unwrap());
    c3.handle_connect().unwrap();
    c3.update_temp(0.0);
    let rust_shutdowns3 = Value::List(p2.shutdowns().iter().map(|m| Value::str(m)).collect());
    assert_value_eq(14, "shutdown_deviation", &exp_shutdowns3, &rust_shutdowns3);
    let (t3, z3) = c3.get_temp(0.0);
    assert_value_eq(15, "deviation_temp", &exp_dev_temp, &tuple2(f(t3), f(z3)));
}

#[test]
fn handle_connect_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("pe = FakePrinter(FakeReactor()); pe.objects['nostatus'] = NoStatusSensor(); ec = FakeConfig('temperature_combined e', pe, Opts({'sensor_list': ['nostatus'], 'maximum_deviation': 1.0, 'combination_method': 'mean'}))").expect("py setup nostatus");
    let py_raised1 = ry_raise(&mut py, "PrinterSensorCombined(ec)._handle_connect()");
    let printer = FakePrinter::new(Reactor::new(false));
    printer.add_object("nostatus", Arc::new(NoStatusSensor) as ObjectRef).unwrap();
    let cfg = FakeConfigWrapper::new("temperature_combined e", Arc::clone(&printer), Some(vec!["nostatus".into()]), Some(1.0), Some("mean".into()));
    let c = CombinedSensor::new(&cfg).unwrap();
    let err = c.handle_connect().unwrap_err();
    assert_value_eq(20, "err_no_status", &py_raised1, &Value::str(err.message()));
    let py_raised2 = ry_raise(&mut py, "PrinterSensorCombined(FakeConfig('temperature_combined e', (lambda pp: (pp.objects.__setitem__('h', FakeSensor({'humidity': 1.0})), pp)[1])(FakePrinter(FakeReactor())), Opts({'sensor_list': ['h'], 'maximum_deviation': 1.0, 'combination_method': 'mean'})))._handle_connect()");
    let printer2 = FakePrinter::new(Reactor::new(false));
    printer2.add_object("h", FakeSensor::dict(vec![("humidity", PyValue::Float(1.0))]) as ObjectRef).unwrap();
    let cfg2 = FakeConfigWrapper::new("temperature_combined e", Arc::clone(&printer2), Some(vec!["h".into()]), Some(1.0), Some("mean".into()));
    let c2 = CombinedSensor::new(&cfg2).unwrap();
    let err2 = c2.handle_connect().unwrap_err();
    assert_value_eq(21, "err_no_temperature", &py_raised2, &Value::str(err2.message()));
    let py_raised3 = ry_raise(&mut py, "PrinterSensorCombined(FakeConfig('temperature_combined e', (lambda pp: (pp.objects.__setitem__('m', FakeSensor({'temperature': None})), pp)[1])(FakePrinter(FakeReactor())), Opts({'sensor_list': ['m'], 'maximum_deviation': 1.0, 'combination_method': 'mean'})))._handle_connect()");
    let printer3 = FakePrinter::new(Reactor::new(false));
    printer3.add_object("m", FakeSensor::dict(vec![("temperature", PyValue::None)]) as ObjectRef).unwrap();
    let cfg3 = FakeConfigWrapper::new("temperature_combined e", Arc::clone(&printer3), Some(vec!["m".into()]), Some(1.0), Some("mean".into()));
    let c3 = CombinedSensor::new(&cfg3).unwrap();
    let err3 = c3.handle_connect().unwrap_err();
    assert_value_eq(22, "err_none_temperature", &py_raised3, &Value::str(err3.message()));
}

fn ry_raise(py: &mut pyoracle::Oracle, expr: &str) -> Value {
    // Evaluate `repr of str(exc)` via exec-captured variable so the
    // exception text compares with the Rust error message.
    let code = format!("try:\n    {}\n    raised = None\nexcept Exception as e:\n    raised = str(e)", expr);
    py.exec(&code).expect("raise probe");
    rx(py.eval("raised"))
}

#[test]
fn construction_config_errors() {
    let Some(mut py) = oracle() else { return };
    let printer = FakePrinter::new(Reactor::new(false));
    // Missing sensor_list.
    let py_missing_list = ry_raise(&mut py, "PrinterSensorCombined(FakeConfig('temperature_combined e', FakePrinter(FakeReactor()), Opts({'maximum_deviation': 1.0, 'combination_method': 'mean'})))");
    let cfg = FakeConfigWrapper::new("temperature_combined e", Arc::clone(&printer), None, Some(1.0), Some("mean".into()));
    let err = CombinedSensor::new(&cfg).map(|_| ()).unwrap_err();
    assert_value_eq(30, "cfg_missing_list", &py_missing_list, &Value::str(err.message()));
    // maximum_deviation <= 0.
    let py_bad_float = ry_raise(&mut py, "PrinterSensorCombined(FakeConfig('temperature_combined e', FakePrinter(FakeReactor()), Opts({'sensor_list': ['a'], 'maximum_deviation': 0.0, 'combination_method': 'mean'})))");
    let cfg2 = FakeConfigWrapper::new("temperature_combined e", Arc::clone(&printer), Some(vec!["a".into()]), Some(0.0), Some("mean".into()));
    let err2 = CombinedSensor::new(&cfg2).map(|_| ()).unwrap_err();
    assert_value_eq(31, "cfg_bad_float", &py_bad_float, &Value::str(err2.message()));
    // Bad combination_method.
    let py_bad_choice = ry_raise(&mut py, "PrinterSensorCombined(FakeConfig('temperature_combined e', FakePrinter(FakeReactor()), Opts({'sensor_list': ['a'], 'maximum_deviation': 1.0, 'combination_method': 'bogus'})))");
    let cfg3 = FakeConfigWrapper::new("temperature_combined e", Arc::clone(&printer), Some(vec!["a".into()]), Some(1.0), Some("bogus".into()));
    let err3 = CombinedSensor::new(&cfg3).map(|_| ()).unwrap_err();
    assert_value_eq(32, "cfg_bad_choice", &py_bad_choice, &Value::str(err3.message()));
}

#[test]
fn load_config_registers_factory() {
    let Some(mut py) = oracle() else { return };
    // Python: load_config registers the combined class with the heaters
    // object, and the registered factory builds PrinterSensorCombined.
    py.exec("ph = FakePrinter(FakeReactor()); ph.objects['heaters'] = FakeHeaters(); load_config(FakeConfig('temperature_combined', ph, Opts({}))); exp_factory = ph.objects['heaters'].factories['temperature_combined'] is PrinterSensorCombined; fs = ph.objects['heaters'].factories['temperature_combined'](FakeConfig('temperature_combined built', ph, Opts({'sensor_list': ['a'], 'maximum_deviation': 2.0, 'combination_method': 'mean'}))); exp_name = fs.name; exp_delta = fs.get_report_time_delta(); exp_temp = fs.get_temp(0.0); exp_status = fs.get_status(0.0)").expect("py load_config factory");
    let exp = rx(py.eval("exp_factory"));
    assert_value_eq(40, "load_config_factory", &exp, &Value::Bool(true));
    let exp_name = rx(py.eval("exp_name"));
    let exp_delta = rx(py.eval("exp_delta"));
    let exp_temp = rx(py.eval("exp_temp"));
    let exp_status = rx(py.eval("exp_status"));

    // Rust: drive load_config through the same contract (the printer
    // object named 'heaters' declaring PrinterHeatersApi).
    let printer = FakePrinter::new(Reactor::new(false));
    let heaters = Arc::new(FakeHeatersObj { factories: Mutex::new(Vec::new()) });
    printer.add_object("heaters", heaters.clone() as ObjectRef).unwrap();
    let lc_cfg = FakeConfigWrapper::new("temperature_combined", Arc::clone(&printer), None, None, None);
    extras::temperature_combined::load_config(&lc_cfg).expect("rust load_config");
    let factory = heaters
        .factories
        .lock()
        .unwrap()
        .iter()
        .find(|(t, _)| t == "temperature_combined")
        .map(|(_, f)| Arc::clone(f))
        .expect("'temperature_combined' factory registered");
    // The factory builds a combined sensor over the same config as the
    // oracle did: its report delta, name, get_temp and get_status match.
    let cfg = FakeConfigWrapper::new("temperature_combined built", Arc::clone(&printer), Some(vec!["a".into()]), Some(2.0), Some("mean".into()));
    let sensor = factory(&cfg).expect("factory builds the sensor");
    assert_value_eq(41, "factory_delta", &exp_delta, &f(sensor.get_report_time_delta()));
    let obj = printer
        .objects()
        .into_iter()
        .find(|(n, _)| n == "temperature_combined built")
        .map(|(_, o)| o)
        .expect("'temperature_combined built' object registered");
    match &exp_name {
        Value::Str(n) => assert_eq!(n, "built"),
        other => panic!("unexpected oracle name value: {other:?}"),
    }
    let sapi = interface::<dyn SensorApi>(&obj).expect("SensorApi interface declared");
    let (ct, cz) = sapi.get_temp(0.0);
    assert_value_eq(42, "factory_get_temp", &exp_temp, &tuple2(f(ct), f(cz)));
    let status = obj.get_status(0.0).expect("get_status");
    assert_value_eq(43, "factory_status", &exp_status, &pyvalue_to_recording(&status));
}

fn pyvalue_to_recording(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(fl) => Value::float(*fl),
        PyValue::Str(st) => Value::str(st),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_recording).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_recording).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_recording(k), pyvalue_to_recording(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host {}", h)),
    }
}

#[test]
fn invalid_runtime_temperature_status_raises_like_python() {
    let mut py = oracle().expect("the Python oracle is required");
    py.exec(r#"
def callback_error(fn):
    try:
        fn()
    except Exception as error:
        return type(error).__name__ + ': ' + str(error)
"#).unwrap();
    let statuses = [
        ("{}", PyValue::Dict(IndexMap::new())),
        ("None", PyValue::None),
        ("{'temperature': None}", PyValue::Dict(IndexMap::from([
            (PyValue::Str("temperature".into()), PyValue::None)]))),
        ("{'temperature': 'bad'}", PyValue::Dict(IndexMap::from([
            (PyValue::Str("temperature".into()), PyValue::Str("bad".into()))]))),
    ];
    for (status_py, status) in statuses {
        py.exec(&format!(
            "p = FakePrinter(FakeReactor()); sensor = FakeSensor({{'temperature': 25.0}}); p.objects['s'] = sensor\n\
             combined = PrinterSensorCombined(FakeConfig('temperature_combined x', p, Opts({{'sensor_list': ['s'], 'maximum_deviation': 2.0, 'combination_method': 'mean'}})))\n\
             combined._handle_connect(); sensor._status = {status_py}"
        )).unwrap();
        let printer = FakePrinter::new(Reactor::new(false));
        let sensor = FakeSensor::dict(vec![("temperature", PyValue::Float(25.0))]);
        printer.add_object("s", sensor.clone() as ObjectRef).unwrap();
        let cfg = FakeConfigWrapper::new("temperature_combined x", printer,
            Some(vec!["s".into()]), Some(2.0), Some("mean".into()));
        let combined = CombinedSensor::new(&cfg).unwrap();
        combined.handle_connect().unwrap();
        sensor.status.replace(status);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            combined.temperature_update_event(0.0);
        }));
        let actual = match result {
            Ok(()) => Value::Null,
            Err(payload) => Value::str(payload.downcast_ref::<String>().map(String::as_str)
                .or_else(|| payload.downcast_ref::<&str>().copied()).expect("exception text")),
        };
        let expected = py.eval("callback_error(lambda: combined._temperature_update_event(0.0))").unwrap();
        assert_value_eq(0, status_py, &expected, &actual);
    }
    py.exec("combined.sensors = []").unwrap();
    let printer = FakePrinter::new(Reactor::new(false));
    let cfg = FakeConfigWrapper::new("temperature_combined x", printer,
        Some(vec!["s".into()]), Some(2.0), Some("mean".into()));
    let combined = CombinedSensor::new(&cfg).unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| combined.update_temp(0.0)));
    let actual = match result {
        Ok(()) => Value::Null,
        Err(payload) => Value::str(payload.downcast_ref::<String>().map(String::as_str)
            .or_else(|| payload.downcast_ref::<&str>().copied()).expect("exception text")),
    };
    let expected = py.eval("callback_error(lambda: combined.update_temp(0.0))").unwrap();
    assert_value_eq(0, "no runtime sensors", &expected, &actual);
}
