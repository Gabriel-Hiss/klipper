//! Parity tests for the `load_cell` port (`klippy/extras/load_cell.py`).
//!
//! Every test drives the Rust port through fakes of the contract traits and the
//! live reference module through pyoracle with mirrored fakes, then compares the
//! observations: returned values (floats by bits), raised errors (class and
//! message), G-code response lines, `get_status` dicts, configfile writes,
//! registered commands and help, webhooks endpoints and printer events.
//!
//! The oracle imports the real `extras.load_cell` (and thus the real
//! `bulk_sensor`, `hx71x`, `ads1220` and `ads131m0x`), so expected values always
//! come from the reference implementation.

#![allow(clippy::too_many_arguments)]

use std::sync::Arc;

use extras::bulk_sensor::ClientCallback;
use extras::load_cell::{
    avg as rust_avg, select_column as rust_select_column, sensor_type_names, ApiClientHelper,
    BulkSensorAdc, LoadCell, LoadCellSampleCollector,
};
use indexmap::IndexMap;
use klippy_api::config::{
    ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts as GCmdGetFloatOpts,
    GetIntOpts as GCmdGetIntOpts, GetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper,
    ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{
    ClientConnection, ClientConnectionApi, WebHooksApi, WebHooksCallback, WebRequest, WebRequestApi,
};
use pycompat::{PyInt, PyValue};
use recording::{assert_equal, Exception, Record, RecordKind, Value as Rv};
use reactor::Reactor;

// ---------------------------------------------------------------------------
// Recording helpers
// ---------------------------------------------------------------------------

fn dummy_record(fn_name: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.load_cell".to_string(),
        scenario: "live".to_string(),
        seq: 0,
        function: fn_name.to_string(),
        args: vec![],
        ret: None,
        exc: None,
        side_effects: Default::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check_value(name: &str, expected: &Rv, actual: &Rv) {
    let record = dummy_record(name);
    if let Err(divergence) = assert_equal(&record, "ret", expected, actual) {
        panic!("{name}: {divergence}");
    }
}

fn check_error(name: &str, actual: &KlippyError, expected: &Exception) {
    assert_eq!(
        actual.class_name(),
        expected.class_name,
        "{name}: error class"
    );
    assert_eq!(actual.message(), expected.message, "{name}: error message");
}

/// A `PyValue` as a recording value (floats compare by bits).
fn rv_of(value: &PyValue) -> Rv {
    match value {
        PyValue::None => Rv::Null,
        PyValue::Bool(b) => Rv::Bool(*b),
        PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
            Ok(i) => Rv::Int(i),
            Err(_) => Rv::BigInt(n.inner().to_string()),
        },
        PyValue::Float(f) => Rv::float(*f),
        PyValue::Str(s) => Rv::Str(s.clone()),
        PyValue::Bytes(b) => Rv::bytes(b),
        PyValue::List(items) => Rv::List(items.iter().map(rv_of).collect()),
        PyValue::Tuple(items) => Rv::Tuple(items.iter().map(rv_of).collect()),
        PyValue::Dict(pairs) => {
            Rv::Dict(pairs.iter().map(|(k, v)| (rv_of(k), rv_of(v))).collect())
        }
        PyValue::Host(_) => panic!("opaque host value in a compared result"),
    }
}

fn rv_of_list(items: &[PyValue]) -> Rv {
    Rv::List(items.iter().map(rv_of).collect())
}

fn rv_strs(items: &[String]) -> Rv {
    Rv::List(items.iter().map(|s| Rv::Str(s.clone())).collect())
}

fn py_str_dict(map: &IndexMap<String, String>) -> PyValue {
    PyValue::Dict(
        map.iter()
            .map(|(k, v)| (PyValue::Str(k.clone()), PyValue::Str(v.clone())))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Fakes: printer
// ---------------------------------------------------------------------------

struct SharedLog {
    msgs: GCell<Vec<String>>,
}

impl SharedLog {
    fn new() -> Arc<Self> {
        Arc::new(SharedLog { msgs: GCell::new(Vec::new()) })
    }
    fn push(&self, msg: String) {
        self.msgs.with_mut(|v| v.push(msg));
    }
    fn get(&self) -> Vec<String> {
        self.msgs.cloned()
    }
}

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<IndexMap<String, Vec<EventCallback>>>,
    reactor: Reactor,
    events: GCell<Vec<String>>,
}

impl FakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(IndexMap::new()),
            reactor: Reactor::new(false),
            events: GCell::new(Vec::new()),
        })
    }
    fn printer_arc(self: &Arc<Self>) -> Printer {
        Arc::clone(self) as Printer
    }
    fn event_names(&self) -> Vec<String> {
        self.events.cloned()
    }
    fn handler_names(&self) -> Vec<String> {
        self.handlers.borrow().keys().cloned().collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        if self.objects.borrow().contains_key(name) {
            return Err(KlippyError::Config(format!("Object {name} already exists")));
        }
        self.objects.with_mut(|m| {
            m.insert(name.to_string(), obj);
        });
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.borrow().get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!("variables: {name}"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.borrow();
        match module {
            None => objects.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(prefix) => objects
                .iter()
                .filter(|(k, _)| *k == prefix || k.starts_with(&format!("{prefix} ")))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
    }
    fn load_object(
        &self,
        _config: &klippy_api::config::Config,
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
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.with_mut(|m| {
            m.entry(event.to_string()).or_default().push(callback);
        });
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.events.with_mut(|v| v.push(event.to_string()));
        let mut cbs = self
            .handlers
            .with_mut(|m| m.get_mut(event).map(std::mem::take))
            .unwrap_or_default();
        let mut results = Vec::new();
        let mut survivors: Vec<EventCallback> = Vec::new();
        for mut cb in cbs.drain(..) {
            results.push(cb(params)?);
            survivors.push(cb);
        }
        self.handlers.with_mut(|m| {
            let mut existing = m.shift_remove(event).unwrap_or_default();
            existing.splice(0..0, survivors);
            m.insert(event.to_string(), existing);
        });
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

// ---------------------------------------------------------------------------
// Fakes: config wrapper (mirrors `configfile.ConfigWrapper` messages)
// ---------------------------------------------------------------------------

struct FakeConfig {
    printer: Printer,
    name: String,
    values: GCell<IndexMap<String, String>>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, values: &[(&str, &str)]) -> Arc<Self> {
        let mut map = IndexMap::new();
        for (key, value) in values {
            map.insert(key.to_lowercase(), (*value).to_string());
        }
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            values: GCell::new(map),
        })
    }
}

fn cfg_missing(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

fn cfg_parse(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        let raw = self.values.borrow().get(&option.to_lowercase()).cloned();
        match raw {
            None => match opts.default {
                None => Err(cfg_missing(&self.name, option)),
                Some(default) => Ok(default),
            },
            Some(value) => Ok(Some(value)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let raw = self.values.borrow().get(&option.to_lowercase()).cloned();
        let raw = match raw {
            None => match opts.default {
                None => return Err(cfg_missing(&self.name, option)),
                Some(default) => return Ok(default),
            },
            Some(value) => value,
        };
        let value = match pycompat::text::int_from_string(&raw, 10) {
            Ok(n) => n.inner().to_string().parse::<i64>().unwrap_or(0),
            Err(_) => return Err(cfg_parse(&self.name, option)),
        };
        if let Some(minval) = opts.minval {
            if value < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {minval}",
                    self.name
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if value > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {maxval}",
                    self.name
                )));
            }
        }
        Ok(Some(value))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = self.values.borrow().get(&option.to_lowercase()).cloned();
        let raw = match raw {
            None => match opts.default {
                None => return Err(cfg_missing(&self.name, option)),
                Some(default) => return Ok(default),
            },
            Some(value) => value,
        };
        let value = match pycompat::text::float_from_string(&raw) {
            Ok(f) => f,
            Err(_) => return Err(cfg_parse(&self.name, option)),
        };
        if let Some(minval) = opts.minval {
            if value < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    pycompat::text::float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if value > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    pycompat::text::float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if value <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    pycompat::text::float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if value >= below {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    pycompat::text::float_str(below)
                )));
            }
        }
        Ok(Some(value))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let raw = self.values.borrow().get(&option.to_lowercase()).cloned();
        match raw {
            None => match opts.default {
                None => Err(cfg_missing(&self.name, option)),
                Some(default) => Ok(default),
            },
            Some(value) => {
                let lowered = value.to_lowercase();
                if ["1", "true", "yes", "on"].contains(&lowered.as_str()) {
                    Ok(Some(true))
                } else if ["0", "false", "no", "off", ""].contains(&lowered.as_str()) {
                    Ok(Some(false))
                } else {
                    Err(cfg_parse(&self.name, option))
                }
            }
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let raw = self.values.borrow().get(&option.to_lowercase()).cloned();
        let mapping = match choices {
            PyValue::Dict(map) => map,
            _ => panic!("getchoice requires a dict"),
        };
        let key = match raw {
            None => match &opts.default {
                None => return Err(cfg_missing(&self.name, option)),
                Some(None) => PyValue::None,
                Some(Some(value)) => value.clone(),
            },
            Some(value) => PyValue::Str(value),
        };
        match mapping.get(&key) {
            Some(value) => Ok(value.clone()),
            None => Err(KlippyError::Config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                key.str(),
                self.name
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::Tuple(vec![]))
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        let raw = self.values.borrow().get(&option.to_lowercase()).cloned();
        match raw {
            None => match opts.default {
                None => Err(cfg_missing(&self.name, option)),
                Some(default) => Ok(default),
            },
            Some(value) => Ok(Some(
                value.split(&opts.sep).map(str::to_string).collect(),
            )),
        }
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(Some(vec![]))
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(Some(vec![]))
    }
    fn getsection(&self, section: &str) -> klippy_api::config::Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            values: GCell::new(IndexMap::new()),
        })
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<klippy_api::config::Config> {
        vec![]
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(vec![])
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
// Fakes: configfile printer object
// ---------------------------------------------------------------------------

struct FakeConfigFile {
    sets: GCell<Vec<(String, String, String)>>,
}

impl FakeConfigFile {
    fn new() -> Arc<Self> {
        Arc::new(FakeConfigFile { sets: GCell::new(Vec::new()) })
    }
}

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("configfile.get_printer is not used by load_cell")
    }
    fn read_config(&self, _filename: &str) -> KResult<klippy_api::config::Config> {
        Err(KlippyError::Config("unexpected read_config".to_string()))
    }
    fn read_main_config(&self) -> KResult<klippy_api::config::Config> {
        Err(KlippyError::Config("unexpected read_main_config".to_string()))
    }
    fn log_config(&self, _config: &klippy_api::config::Config) {}
    fn check_unused_options(&self, _config: &klippy_api::config::Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(
        &self,
        _section: &str,
        _option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.sets
            .with_mut(|v| v.push((section.to_string(), option.to_string(), value.to_string())));
        Ok(())
    }
    fn remove_section(&self, _section: &str) {}
}

// ---------------------------------------------------------------------------
// Fakes: MCU and sensor
// ---------------------------------------------------------------------------

/// The MCU fake. `is_fileoutput()` is the point in `_collect_until` where a
/// running reactor would deliver sensor batches, so the fake hands the queued
/// batches to the sensor there; both languages do it at the same place.
struct FakeMcu {
    fileoutput: bool,
    estimated: f64,
    flush_hook: GCell<Option<Arc<FakeSensor>>>,
}

impl FakeMcu {
    fn new(fileoutput: bool, estimated: f64) -> Arc<Self> {
        Arc::new(FakeMcu {
            fileoutput,
            estimated,
            flush_hook: GCell::new(None),
        })
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("mcu.get_printer is not used by load_cell")
    }
    fn is_fileoutput(&self) -> bool {
        if let Some(sensor) = self.flush_hook.borrow().clone() {
            let _ = sensor.flush();
        }
        self.fileoutput
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin is not used by load_cell".to_string()))
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
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Err(KlippyError::Mcu("lookup_command is not used by load_cell".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu(
            "lookup_query_command is not used by load_cell".to_string(),
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
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by load_cell")
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
        Err(KlippyError::Mcu(
            "get_constant_float is not used by load_cell".to_string(),
        ))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        self.estimated
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        0
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Mcu(
            "create_trigger_dispatch is not used by load_cell".to_string(),
        ))
    }
}

struct SensorClient {
    id: u64,
    cb: ClientCallback,
}

/// The sensor fake: batches are queued by the test and delivered by `flush`.
struct FakeSensor {
    mcu: Mcu,
    sps: i64,
    range: (i64, i64),
    errors: i64,
    overflows: i64,
    clients: GCell<Vec<SensorClient>>,
    next_client_id: GCell<u64>,
    batches: GCell<Vec<PyValue>>,
}

impl FakeSensor {
    fn new(mcu: Mcu, sps: i64, range: (i64, i64), errors: i64, overflows: i64) -> Arc<Self> {
        Arc::new(FakeSensor {
            mcu,
            sps,
            range,
            errors,
            overflows,
            clients: GCell::new(Vec::new()),
            next_client_id: GCell::new(0),
            batches: GCell::new(Vec::new()),
        })
    }
    fn queue_batch(&self, msg: PyValue) {
        self.batches.with_mut(|v| v.push(msg));
    }
    fn flush(&self) -> KResult<()> {
        loop {
            let next = {
                let mut batches = self.batches.borrow_mut();
                if batches.is_empty() {
                    None
                } else {
                    Some(batches.remove(0))
                }
            };
            let Some(msg) = next else { break };
            let ids: Vec<u64> = self.clients.borrow().iter().map(|c| c.id).collect();
            for id in ids {
                let Some(idx) = self.clients.borrow().iter().position(|c| c.id == id) else {
                    continue;
                };
                let mut cb = std::mem::replace(
                    &mut self.clients.borrow_mut()[idx].cb,
                    Box::new(|_| Ok(false)) as ClientCallback,
                );
                let keep = cb(msg.clone())?;
                let pos = self.clients.borrow().iter().position(|c| c.id == id);
                match pos {
                    Some(pos) if keep => self.clients.borrow_mut()[pos].cb = cb,
                    Some(pos) => {
                        self.clients.borrow_mut().remove(pos);
                    }
                    None => {}
                }
            }
        }
        Ok(())
    }
}

impl BulkSensorAdc for FakeSensor {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_samples_per_second(&self) -> f64 {
        self.sps as f64
    }
    fn get_range(&self) -> (i64, i64) {
        self.range
    }
    fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        let id = {
            let mut next = self.next_client_id.borrow_mut();
            let id = *next;
            *next += 1;
            id
        };
        self.clients.borrow_mut().push(SensorClient { id, cb });
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("errors".to_string()),
            PyValue::Int(PyInt::from_i64(self.errors)),
        );
        status.insert(
            PyValue::Str("overflows".to_string()),
            PyValue::Int(PyInt::from_i64(self.overflows)),
        );
        status.insert(
            PyValue::Str("sample_rate".to_string()),
            PyValue::Int(PyInt::from_i64(self.sps)),
        );
        PyValue::Dict(status)
    }
}

// ---------------------------------------------------------------------------
// Fakes: gcode dispatch and command
// ---------------------------------------------------------------------------

struct FakeGCode {
    log: Arc<SharedLog>,
    plain: GCell<IndexMap<String, Option<GCodeCommandHandler>>>,
    mux: GCell<IndexMap<String, IndexMap<Option<String>, GCodeCommandHandler>>>,
    mux_key: GCell<IndexMap<String, String>>,
    help: GCell<IndexMap<String, String>>,
    registrations: GCell<Vec<String>>,
}

fn dummy_handler() -> GCodeCommandHandler {
    Box::new(|_: &GCmd| Ok(()))
}

impl FakeGCode {
    fn new(log: Arc<SharedLog>) -> Arc<Self> {
        Arc::new(FakeGCode {
            log,
            plain: GCell::new(IndexMap::new()),
            mux: GCell::new(IndexMap::new()),
            mux_key: GCell::new(IndexMap::new()),
            help: GCell::new(IndexMap::new()),
            registrations: GCell::new(Vec::new()),
        })
    }
    fn command(&self, line: &str, params: &[(&str, &str)]) -> GCmd {
        let mut map = IndexMap::new();
        for (key, value) in params {
            map.insert((*key).to_string(), (*value).to_string());
        }
        self.create_gcode_command(line, line, &map)
    }
    fn dispatch_plain(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handler = {
            let mut map = self.plain.borrow_mut();
            match map.get_mut(cmd) {
                Some(slot) => slot.take().expect("handler is registered"),
                None => panic!("command {cmd} is not registered"),
            }
        };
        let result = handler(gcmd);
        self.plain.with_mut(|map| {
            if let Some(slot) = map.get_mut(cmd) {
                *slot = Some(handler);
            }
        });
        result
    }
    fn dispatch_mux(&self, cmd: &str, value: Option<&str>, gcmd: &GCmd) -> KResult<()> {
        let key = value.map(str::to_string);
        let mut handler = {
            let mut map = self.mux.borrow_mut();
            let values = map
                .get_mut(cmd)
                .unwrap_or_else(|| panic!("mux command {cmd} is not registered"));
            let slot = values
                .get_mut(&key)
                .unwrap_or_else(|| panic!("mux value {key:?} of {cmd} is not registered"));
            std::mem::replace(slot, dummy_handler())
        };
        let result = handler(gcmd);
        self.mux.with_mut(|map| {
            if let Some(values) = map.get_mut(cmd) {
                if let Some(slot) = values.get_mut(&key) {
                    *slot = handler;
                }
            }
        });
        result
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        match func {
            None => {
                let old = self.plain.borrow_mut().shift_remove(cmd).flatten();
                self.registrations
                    .with_mut(|v| v.push(format!("unregister {cmd}")));
                Ok(old)
            }
            Some(handler) => {
                if self.plain.borrow().contains_key(cmd) {
                    return Err(KlippyError::Config(format!(
                        "gcode command {cmd} already registered"
                    )));
                }
                self.plain.with_mut(|m| {
                    m.insert(cmd.to_string(), Some(handler));
                });
                if let Some(desc) = opts.desc {
                    self.help.with_mut(|h| {
                        h.insert(cmd.to_string(), desc);
                    });
                }
                self.registrations
                    .with_mut(|v| v.push(format!("register {cmd}")));
                Ok(None)
            }
        }
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        let first = !self.mux_key.borrow().contains_key(cmd);
        if first {
            self.mux_key.with_mut(|m| {
                m.insert(cmd.to_string(), key.to_string());
            });
            // Python registers the base command first.
            self.register_command(
                cmd,
                Some(dummy_handler()),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: desc.map(str::to_string),
                },
            )?;
            self.registrations
                .with_mut(|v| v.push(format!("mux-base {cmd}")));
        } else if self.mux_key.borrow().get(cmd).map(String::as_str) != Some(key) {
            return Err(KlippyError::Config(format!(
                "mux command {cmd} {key} {} may have only one key",
                value.unwrap_or("None")
            )));
        }
        let value_key = value.map(str::to_string);
        if self
            .mux
            .borrow()
            .get(cmd)
            .map(|values| values.contains_key(&value_key))
            .unwrap_or(false)
        {
            return Err(KlippyError::Config(format!(
                "mux command {cmd} {key} {} already registered",
                value.unwrap_or("None")
            )));
        }
        self.mux.with_mut(|m| {
            m.entry(cmd.to_string())
                .or_default()
                .insert(value_key, func);
        });
        if first {
            if let Some(desc) = desc {
                self.help.with_mut(|h| {
                    h.insert(cmd.to_string(), desc.to_string());
                });
            }
        }
        self.registrations.with_mut(|v| {
            v.push(format!("mux {cmd} {key}={}", value.unwrap_or("None")))
        });
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        self.help.cloned()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            log: Arc::clone(&self.log),
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        })
    }
    fn respond_raw(&self, msg: &str) {
        self.log.push(format!("raw:{msg}"));
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

struct FakeGCmd {
    log: Arc<SharedLog>,
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(default) => Ok(default),
            },
            Some(value) => Ok(PyValue::Str(value.clone())),
        }
    }
    fn get_int(&self, name: &str, opts: GCmdGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(default) => Ok(default),
            },
            Some(value) => match pycompat::text::int_from_string(value, 10) {
                Ok(n) => Ok(Some(n.inner().to_string().parse::<i64>().unwrap_or(0))),
                Err(_) => Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {value}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_float(&self, name: &str, opts: GCmdGetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            None => match opts.default {
                None => {
                    return Err(KlippyError::Command(format!(
                        "Error on '{}': missing {name}",
                        self.commandline
                    )))
                }
                Some(default) => return Ok(default),
            },
            Some(value) => value,
        };
        let value = match pycompat::text::float_from_string(raw) {
            Ok(f) => f,
            Err(_) => {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                )))
            }
        };
        if let Some(minval) = opts.minval {
            if value < minval {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {name} must have minimum of {}",
                    self.commandline,
                    pycompat::text::float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if value > maxval {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {name} must have maximum of {}",
                    self.commandline,
                    pycompat::text::float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if value <= above {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {name} must be above {}",
                    self.commandline,
                    pycompat::text::float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if value >= below {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {name} must be below {}",
                    self.commandline,
                    pycompat::text::float_str(below)
                )));
            }
        }
        Ok(Some(value))
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.commandline.clone()
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.push(msg.to_string());
    }
    fn respond_raw(&self, msg: &str) {
        self.log.push(format!("raw:{msg}"));
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Fakes: webhooks
// ---------------------------------------------------------------------------

struct FakeWebHooks {
    mux: GCell<Vec<(String, String, String)>>,
    callbacks: GCell<IndexMap<String, WebHooksCallback>>,
}

impl FakeWebHooks {
    fn new() -> Arc<Self> {
        Arc::new(FakeWebHooks {
            mux: GCell::new(Vec::new()),
            callbacks: GCell::new(IndexMap::new()),
        })
    }
}

klippy_api::printer_object!(FakeWebHooks; dyn WebHooksApi);

impl WebHooksApi for FakeWebHooks {
    fn register_endpoint(&self, path: &str, _callback: WebHooksCallback) -> KResult<()> {
        self.mux
            .with_mut(|v| v.push((path.to_string(), "endpoint".to_string(), String::new())));
        Ok(())
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()> {
        self.mux
            .with_mut(|v| v.push((path.to_string(), key.to_string(), value.to_string())));
        self.callbacks.with_mut(|m| {
            m.insert(format!("{path}\0{key}\0{value}"), callback);
        });
        Ok(())
    }
    fn call_remote_method(&self, _method: &str, _params: &IndexMap<String, PyValue>) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
}

struct FakeClientConn {
    sent: GCell<Vec<PyValue>>,
}

impl FakeClientConn {
    fn new() -> Arc<Self> {
        Arc::new(FakeClientConn { sent: GCell::new(Vec::new()) })
    }
}

impl ClientConnectionApi for FakeClientConn {
    fn uid(&self) -> u64 {
        1
    }
    fn dump_request_log(&self) {}
    fn set_client_info(&self, _client_info: PyValue, _state_msg: Option<String>) {}
    fn close(&self) {}
    fn is_closed(&self) -> bool {
        false
    }
    fn send(&self, data: PyValue) {
        self.sent.with_mut(|v| v.push(data));
    }
}

struct FakeWebRequest {
    conn: Arc<FakeClientConn>,
    sent: GCell<Vec<PyValue>>,
}

impl FakeWebRequest {
    fn new(conn: Arc<FakeClientConn>) -> Arc<Self> {
        Arc::new(FakeWebRequest {
            conn,
            sent: GCell::new(Vec::new()),
        })
    }
}

impl WebRequestApi for FakeWebRequest {
    fn client_conn(&self) -> ClientConnection {
        self.conn.clone()
    }
    fn id(&self) -> Option<PyValue> {
        None
    }
    fn method(&self) -> String {
        String::new()
    }
    fn params(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn response(&self) -> Option<PyValue> {
        None
    }
    fn is_error(&self) -> bool {
        false
    }
    fn get_client_connection(&self) -> ClientConnection {
        self.conn.clone()
    }
    fn get(&self, _item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        match default {
            Some(default) => Ok(default),
            None => Err(KlippyError::Command("missing".to_string())),
        }
    }
    fn get_str(&self, _item: &str, default: Option<String>) -> KResult<String> {
        Ok(default.unwrap_or_default())
    }
    fn get_int(&self, _item: &str, default: Option<i64>) -> KResult<i64> {
        Ok(default.unwrap_or(0))
    }
    fn get_float(&self, _item: &str, default: Option<f64>) -> KResult<f64> {
        Ok(default.unwrap_or(0.0))
    }
    fn get_dict(&self, _item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        Ok(default.unwrap_or(PyValue::Dict(IndexMap::new())))
    }
    fn get_method(&self) -> String {
        String::new()
    }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        self.sent.with_mut(|v| v.push(data));
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        None
    }
}

// ---------------------------------------------------------------------------
// World builder
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    log: Arc<SharedLog>,
    gcode: Arc<FakeGCode>,
    webhooks: Arc<FakeWebHooks>,
    configfile: Arc<FakeConfigFile>,
    sensor: Arc<FakeSensor>,
    config: klippy_api::config::Config,
}

impl World {
    fn cell(&self) -> Arc<LoadCell> {
        let cell = LoadCell::new(&self.config, self.sensor.clone()).expect("LoadCell::new");
        self.printer
            .add_object(&self.config.get_name(), cell.clone() as ObjectRef)
            .expect("add load_cell object");
        cell
    }
}

struct WorldSpec<'a> {
    section: &'a str,
    values: &'a [(&'a str, &'a str)],
    sps: i64,
    range: (i64, i64),
    sensor_errors: i64,
    sensor_overflows: i64,
    fileoutput: bool,
    estimated: f64,
    flush_hook: bool,
}

impl<'a> WorldSpec<'a> {
    fn new(section: &'a str, values: &'a [(&'a str, &'a str)], sps: i64) -> WorldSpec<'a> {
        WorldSpec {
            section,
            values,
            sps,
            range: (-0x800000, 0x7FFFFF),
            sensor_errors: 0,
            sensor_overflows: 0,
            fileoutput: false,
            estimated: 100.0,
            flush_hook: false,
        }
    }
}

fn make_world(spec: &WorldSpec) -> World {
    let printer = FakePrinter::new();
    let printer_api = printer.printer_arc();
    let log = SharedLog::new();
    let gcode = FakeGCode::new(Arc::clone(&log));
    let webhooks = FakeWebHooks::new();
    let configfile = FakeConfigFile::new();
    printer
        .add_object("gcode", Arc::clone(&gcode) as ObjectRef)
        .expect("register gcode");
    printer
        .add_object("webhooks", Arc::clone(&webhooks) as ObjectRef)
        .expect("register webhooks");
    printer
        .add_object("configfile", Arc::clone(&configfile) as ObjectRef)
        .expect("register configfile");
    let mcu = FakeMcu::new(spec.fileoutput, spec.estimated);
    let mcu_api: Mcu = Arc::clone(&mcu) as Mcu;
    let sensor = FakeSensor::new(
        mcu_api,
        spec.sps,
        spec.range,
        spec.sensor_errors,
        spec.sensor_overflows,
    );
    if spec.flush_hook {
        mcu.flush_hook.replace(Some(Arc::clone(&sensor)));
    }
    let config = FakeConfig::new(printer_api, spec.section, spec.values);
    World {
        printer,
        log,
        gcode,
        webhooks,
        configfile,
        sensor,
        config,
    }
}

/// Build the mirrored Python world: `_p`, `_gcode`, `_wh`, `_cf`, `_mcu`, `_s`,
/// `_cfg` and the load cell instance `_lc`. Returns the construction error when
/// the reference raises (so error paths compare the same world).
fn py_world(py: &mut pyoracle::Oracle, spec: &WorldSpec) -> Result<(), Exception> {
    let mut items = String::new();
    for (key, value) in spec.values {
        items.push_str(&format!("{key:?}: {value:?},"));
    }
    let fileoutput = if spec.fileoutput { "True" } else { "False" };
    let flush_hook = if spec.flush_hook { "True" } else { "False" };
    let code = format!(
        r#"
from extras.load_cell import LoadCell
_p = FakePrinter()
_log = []
_gcode = FakeGCode(_log)
_wh = FakeWebHooks()
_cf = FakeConfigFile()
_mcu = FakeMcu(fileoutput={fileoutput}, estimated={estimated:?})
_s = FakeSensor(_mcu, {sps}, {rng:?}, errors={errors}, overflows={overflows})
if {flush_hook}:
    _mcu.sensor = _s
_cfg = FakeConfig(_p, {section:?}, {{{items}}})
_p.objects['gcode'] = _gcode
_p.objects['webhooks'] = _wh
_p.objects['configfile'] = _cf
_lc = LoadCell(_cfg, _s)
_p.add_object({section:?}, _lc)
"#,
        fileoutput = fileoutput,
        estimated = spec.estimated,
        sps = spec.sps,
        rng = spec.range,
        errors = spec.sensor_errors,
        overflows = spec.sensor_overflows,
        flush_hook = flush_hook,
        section = spec.section,
        items = items,
    );
    py.exec(&code).map(|_| ())
}

fn py_queue_batch(
    py: &mut pyoracle::Oracle,
    samples: &[(f64, i64, f64)],
    errors: i64,
    overflows: i64,
) {
    let rows: Vec<String> = samples
        .iter()
        .map(|(time, counts, value)| format!("({time:?}, {counts}, {value:?})"))
        .collect();
    let code = format!(
        "_s.batches.append({{'data': [{}], 'errors': {errors}, 'overflows': {overflows}}})",
        rows.join(", ")
    );
    py.exec(&code).expect("oracle queue batch");
}

fn batch_msg(samples: &[(f64, i64, f64)], errors: i64, overflows: i64) -> PyValue {
    let mut msg = IndexMap::new();
    msg.insert(
        PyValue::Str("data".to_string()),
        PyValue::List(
            samples
                .iter()
                .map(|(time, counts, value)| {
                    PyValue::Tuple(vec![
                        PyValue::Float(*time),
                        PyValue::Int(PyInt::from_i64(*counts)),
                        PyValue::Float(*value),
                    ])
                })
                .collect(),
        ),
    );
    msg.insert(
        PyValue::Str("errors".to_string()),
        PyValue::Int(PyInt::from_i64(errors)),
    );
    msg.insert(
        PyValue::Str("overflows".to_string()),
        PyValue::Int(PyInt::from_i64(overflows)),
    );
    PyValue::Dict(msg)
}

/// `n` samples with the same counts, 0.1 s apart.
fn uniform_batch(counts: i64, n: usize) -> Vec<(f64, i64, f64)> {
    (0..n)
        .map(|i| (1000.0 + i as f64 * 0.1, counts, counts as f64 * 0.5))
        .collect()
}

/// `(samples, errors)` from Python: only the sample list is returned.
fn samples_of(value: &Rv) -> Rv {
    match value {
        Rv::Tuple(items) if items.len() == 2 => items[0].clone(),
        other => panic!("expected (samples, errors), got {}", other.render()),
    }
}

fn errors_of(value: &Rv) -> Rv {
    match value {
        Rv::Tuple(items) if items.len() == 2 => items[1].clone(),
        other => panic!("expected (samples, errors), got {}", other.render()),
    }
}

fn rv_errors(errors: Option<(i64, i64)>) -> Rv {
    match errors {
        None => Rv::int(0),
        Some((errors, overflows)) => Rv::Tuple(vec![Rv::int(errors), Rv::int(overflows)]),
    }
}

// ---------------------------------------------------------------------------
// Oracle scaffolding
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
from extras import load_cell as _lcm

class Error(Exception):
    pass

class CommandError(Exception):
    pass

class FakeReactor:
    def __init__(self):
        self.time = 100.0
        self.sensor = None
    def monotonic(self):
        return self.time
    def pause(self, wake):
        if self.sensor is not None:
            self.sensor.flush()
        return wake
    def register_callback(self, cb, wake=0.0):
        cb(self.time)
        return None

class FakePrinter:
    def __init__(self):
        self.config_error = Error
        self.command_error = CommandError
        self.reactor = FakeReactor()
        self.objects = {}
        self.handlers = {}
        self.events = []
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            raise Error('variables: %s' % (name,))
        return default
    def add_object(self, name, obj):
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        self.events.append(event)
        results = []
        for cb in list(self.handlers.get(event, [])):
            results.append(cb(*params))
        return results

_SENTINEL = object()

class FakeConfig:
    def __init__(self, printer, name, values):
        self.printer = printer
        self.name = name
        self.values = {}
        for key, value in values.items():
            self.values[key.lower()] = value
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def _missing(self, option):
        raise Error("Option '%s' in section '%s' must be specified"
                    % (option, self.name))
    def _parse(self, option):
        raise Error("Unable to parse option '%s' in section '%s'"
                    % (option, self.name))
    def _min(self, option, minval):
        raise Error("Option '%s' in section '%s' must have minimum of %s"
                    % (option, self.name, minval))
    def _max(self, option, maxval):
        raise Error("Option '%s' in section '%s' must have maximum of %s"
                    % (option, self.name, maxval))
    def get(self, option, default=_SENTINEL, note_valid=True):
        if option.lower() not in self.values:
            if default is _SENTINEL:
                self._missing(option)
            return default
        return self.values[option.lower()]
    def getint(self, option, default=_SENTINEL, minval=None, maxval=None,
               note_valid=True):
        if option.lower() not in self.values:
            if default is _SENTINEL:
                self._missing(option)
            return default
        try:
            value = int(self.values[option.lower()])
        except:
            self._parse(option)
        if minval is not None and value < minval:
            self._min(option, minval)
        if maxval is not None and value > maxval:
            self._max(option, maxval)
        return value
    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        if option.lower() not in self.values:
            if default is _SENTINEL:
                self._missing(option)
            return default
        try:
            value = float(self.values[option.lower()])
        except:
            self._parse(option)
        if minval is not None and value < minval:
            self._min(option, minval)
        if maxval is not None and value > maxval:
            self._max(option, maxval)
        if above is not None and value <= above:
            raise Error("Option '%s' in section '%s' must be above %s"
                        % (option, self.name, above))
        if below is not None and value >= below:
            raise Error("Option '%s' in section '%s' must be below %s"
                        % (option, self.name, below))
        return value
    def getboolean(self, option, default=_SENTINEL, note_valid=True):
        if option.lower() not in self.values:
            if default is _SENTINEL:
                self._missing(option)
            return default
        value = self.values[option.lower()].lower()
        if value in ('1', 'true', 'yes', 'on'):
            return True
        if value in ('0', 'false', 'no', 'off', ''):
            return False
        self._parse(option)
    def getchoice(self, option, choices, default=_SENTINEL, note_valid=True):
        if option.lower() not in self.values:
            if default is _SENTINEL:
                self._missing(option)
            value = default
        else:
            value = self.values[option.lower()]
        if value not in choices:
            raise Error("Choice '%s' for option '%s' in section '%s'"
                        " is not a valid choice"
                        % (value, option, self.name))
        return choices[value]

class FakeConfigFile:
    def __init__(self):
        self.sets = []
    def set(self, section, option, value):
        self.sets.append((section, option, value))

class FakeMcu:
    def __init__(self, fileoutput=False, estimated=100.0):
        self.fileoutput = fileoutput
        self.estimated = estimated
        self.sensor = None
    def is_fileoutput(self):
        if self.sensor is not None:
            self.sensor.flush()
        return self.fileoutput
    def estimated_print_time(self, eventtime):
        return self.estimated

class FakeSensor:
    def __init__(self, mcu, sps, rng, errors=0, overflows=0):
        self.mcu = mcu
        self.sps = sps
        self.rng = rng
        self.errors = errors
        self.overflows = overflows
        self.clients = []
        self.batches = []
    def get_mcu(self):
        return self.mcu
    def get_samples_per_second(self):
        return self.sps
    def get_range(self):
        return self.rng
    def add_client(self, callback):
        self.clients.append(callback)
    def flush(self):
        while self.batches:
            msg = self.batches.pop(0)
            for cb in list(self.clients):
                keep = cb(msg)
                if not keep:
                    self.clients.remove(cb)
    def get_status(self, eventtime):
        return {'errors': self.errors, 'overflows': self.overflows,
                'sample_rate': self.sps}

class FakeGcmd:
    error = CommandError
    def __init__(self, log, commandline, params):
        self.log = log
        self.commandline = commandline
        self.params = params
    def get_command(self):
        return self.commandline.split()[0]
    def get_commandline(self):
        return self.commandline
    def respond_info(self, msg, log=True):
        self.log.append(msg)
    def respond_raw(self, msg):
        self.log.append('raw:' + msg)
    def ack(self, msg=None):
        return False
    def get_float(self, name, default=None, minval=None, maxval=None,
                  above=None, below=None):
        if name not in self.params:
            if default is None:
                raise CommandError("Error on '%s': missing %s"
                                   % (self.commandline, name))
            return default
        try:
            value = float(self.params[name])
        except:
            raise CommandError("Error on '%s': unable to parse %s"
                               % (self.commandline, self.params[name]))
        if minval is not None and value < minval:
            raise CommandError("Error on '%s': %s must have minimum of %s"
                               % (self.commandline, name, minval))
        if maxval is not None and value > maxval:
            raise CommandError("Error on '%s': %s must have maximum of %s"
                               % (self.commandline, name, maxval))
        return value

class FakeGCode:
    error = CommandError
    def __init__(self, log):
        self.log = log
        self.plain = {}
        self.mux = {}
        self.mux_key = {}
        self.help = {}
        self.registrations = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            old = self.plain.pop(cmd, None)
            self.registrations.append('unregister %s' % (cmd,))
            return old
        if cmd in self.plain:
            raise Error('gcode command %s already registered' % (cmd,))
        self.plain[cmd] = func
        if desc is not None:
            self.help[cmd] = desc
        self.registrations.append('register %s' % (cmd,))
    def register_mux_command(self, cmd, key, value, func, desc=None):
        if cmd not in self.mux_key:
            self.mux_key[cmd] = key
            self.plain[cmd] = lambda gcmd: None
            if desc is not None:
                self.help[cmd] = desc
            self.registrations.append('register %s' % (cmd,))
            self.registrations.append('mux-base %s' % (cmd,))
        elif self.mux_key[cmd] != key:
            raise Error('mux command %s %s %s may have only one key (%s)'
                        % (cmd, key, value, self.mux_key[cmd]))
        if (cmd, value) in self.mux:
            raise Error('mux command %s %s %s already registered'
                        % (cmd, key, value))
        self.mux[(cmd, value)] = func
        self.registrations.append('mux %s %s=%s' % (cmd, key, value))
    def respond_info(self, msg, log=True):
        self.log.append(msg)
    def respond_raw(self, msg):
        self.log.append('raw:' + msg)
    def dispatch_plain(self, cmd, commandline, params=None):
        return self.plain[cmd](FakeGcmd(self.log, commandline, params or {}))
    def dispatch_mux(self, cmd, value, commandline, params=None):
        return self.mux[(cmd, value)](FakeGcmd(self.log, commandline, params or {}))

class FakeClientConn:
    def __init__(self):
        self.sent = []
    def is_closed(self):
        return False
    def send(self, data):
        self.sent.append(data)

class FakeWebRequest:
    def __init__(self):
        self.sent = []
        self.conn = FakeClientConn()
    def get_client_connection(self):
        return self.conn
    def get_dict(self, item, default=None):
        return default if default is not None else {}
    def send(self, data):
        self.sent.append(data)

class FakeWebHooks:
    def __init__(self):
        self.mux = []
        self.callbacks = {}
    def register_mux_endpoint(self, path, key, value, callback):
        self.mux.append((path, key, value))
        self.callbacks[(path, key, value)] = callback
"#;

fn start_oracle() -> Option<pyoracle::Oracle> {
    pyoracle::Oracle::start().map(|mut py| {
        py.exec(PY_SETUP).expect("oracle setup");
        py
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn select_column_and_avg_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    py.exec(
        "DATA = [[1.5, 100, 0.25], [2.5, -50, -0.125], [3.5, 7, 1.0]]\n\
         IDATA = [[1, 100, 0.25], [2, 50, 0.5]]",
    )
    .expect("oracle data");
    let rows = vec![
        PyValue::List(vec![
            PyValue::Float(1.5),
            PyValue::Int(PyInt::from_i64(100)),
            PyValue::Float(0.25),
        ]),
        PyValue::List(vec![
            PyValue::Float(2.5),
            PyValue::Int(PyInt::from_i64(-50)),
            PyValue::Float(-0.125),
        ]),
        PyValue::List(vec![
            PyValue::Float(3.5),
            PyValue::Int(PyInt::from_i64(7)),
            PyValue::Float(1.0),
        ]),
    ];
    // `zip(*data)[idx]` returns a tuple whose values keep their Python types.
    fn floats_of(column: &[PyValue]) -> Vec<f64> {
        column
            .iter()
            .map(|v| match v {
                PyValue::Float(f) => *f,
                PyValue::Int(n) => n.inner().to_string().parse::<f64>().expect("int as float"),
                other => panic!("not a number: {other:?}"),
            })
            .collect()
    }
    for column in [0usize, 1, 2] {
        let got = rust_select_column(&rows, column).expect("select_column");
        let expected = py
            .eval(&format!("_lcm.select_column(DATA, {column})"))
            .expect("oracle column");
        check_value(
            "select_column",
            &expected,
            &Rv::Tuple(got.iter().map(rv_of).collect()),
        );
    }
    let column = floats_of(&rust_select_column(&rows, 2).expect("column 2"));
    let mean = rust_avg(&column).expect("avg");
    let expected = py
        .eval("_lcm.avg(_lcm.select_column(DATA, 2))")
        .expect("oracle avg");
    check_value("avg floats", &expected, &Rv::float(mean));

    // Integer rows: `sum(ints) / len` is a float division.
    let int_rows = vec![
        PyValue::List(vec![
            PyValue::Int(PyInt::from_i64(1)),
            PyValue::Int(PyInt::from_i64(100)),
            PyValue::Float(0.25),
        ]),
        PyValue::List(vec![
            PyValue::Int(PyInt::from_i64(2)),
            PyValue::Int(PyInt::from_i64(50)),
            PyValue::Float(0.5),
        ]),
    ];
    let column = rust_select_column(&int_rows, 1).expect("column 1");
    let expected = py
        .eval("_lcm.select_column(IDATA, 1)")
        .expect("oracle int column");
    check_value(
        "select_column ints",
        &expected,
        &Rv::Tuple(column.iter().map(rv_of).collect()),
    );
    let mean = rust_avg(&floats_of(&column)).expect("avg");
    let expected = py
        .eval("_lcm.avg(_lcm.select_column(IDATA, 1))")
        .expect("oracle avg ints");
    check_value("avg ints", &expected, &Rv::float(mean));

    // A column past the end of a row is an IndexError, like an empty row list;
    // an empty average is `sum([]) / 0`.
    let err = rust_select_column(&rows, 3).expect_err("column past the row");
    let expected = py
        .eval("_lcm.select_column(DATA, 3)")
        .expect_err("oracle column past the row");
    check_error("select_column short row", &err, &expected);
    let err = rust_select_column(&[], 2).expect_err("empty column");
    let expected = py
        .eval("_lcm.select_column([], 2)")
        .expect_err("oracle empty column");
    check_error("select_column empty", &err, &expected);
    let err = rust_avg(&[]).expect_err("empty average");
    py.exec("EMPTY = []").expect("oracle empty list");
    let expected = py.eval("_lcm.avg(EMPTY)").expect_err("oracle empty avg");
    check_error("avg empty", &err, &expected);
}

#[test]
fn config_options_and_status_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    // Defaults: uncalibrated, untared, `sensor_orientation` normal.
    let values: &[(&str, &str)] = &[];
    let spec = WorldSpec::new("load_cell mycell", values, 80);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    let expected = py.eval("_lc.get_status(100.0)").expect("oracle status");
    check_value(
        "get_status defaults",
        &expected,
        &rv_of(&cell.get_status_value(100.0)),
    );
    let expected = py.eval("_lc.is_calibrated()").expect("oracle calibrated");
    check_value(
        "is_calibrated default",
        &expected,
        &Rv::Bool(cell.is_calibrated()),
    );
    let expected = py.eval("_lc.is_tared()").expect("oracle tared");
    check_value("is_tared default", &expected, &Rv::Bool(cell.is_tared()));
    let expected = py.eval("_lc.get_counts_per_gram()").expect("oracle cpg");
    check_value("get_counts_per_gram default", &expected, &Rv::Null);
    let expected = py.eval("_lc.get_reference_tare_counts()").expect("oracle ref");
    check_value("get_reference_tare_counts default", &expected, &Rv::Null);
    let expected = py.eval("_lc.get_tare_counts()").expect("oracle tare");
    check_value("get_tare_counts default", &expected, &Rv::Null);

    // Configured and inverted.
    let values: &[(&str, &str)] = &[
        ("reference_tare_counts", "1000"),
        ("counts_per_gram", "2.5"),
        ("sensor_orientation", "inverted"),
    ];
    let spec = WorldSpec::new("load_cell mycell", values, 80);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world 2");
    let world = make_world(&spec);
    let cell = world.cell();
    let expected = py.eval("_lc.get_status(100.0)").expect("oracle status 2");
    check_value(
        "get_status calibrated",
        &expected,
        &rv_of(&cell.get_status_value(100.0)),
    );
    for sample in [1000i64, 1100, 900, 0] {
        let got = cell.counts_to_grams(sample as f64).expect("grams");
        let expected = py
            .eval(&format!("_lc.counts_to_grams({sample})"))
            .expect("oracle grams");
        check_value("counts_to_grams inverted", &expected, &Rv::float(got));
    }
    for counts in [0.0, 1000.0, -8388608.0, 8388607.0] {
        let got = cell.counts_to_percent(counts).expect("percent");
        let expected = py
            .eval(&format!("_lc.counts_to_percent({counts:?})"))
            .expect("oracle percent");
        check_value("counts_to_percent", &expected, &Rv::float(got));
    }
    let expected = py.eval("_lc.saturation_range()").expect("oracle range");
    let (min, max) = cell.saturation_range();
    check_value(
        "saturation_range",
        &expected,
        &Rv::Tuple(vec![Rv::int(min), Rv::int(max)]),
    );

    // `counts_per_gram` below the minimum.
    let values: &[(&str, &str)] = &[("counts_per_gram", "0.5")];
    let spec = WorldSpec::new("load_cell lowcell", values, 80);
    let world = make_world(&spec);
    let err = LoadCell::new(&world.config, world.sensor.clone())
        .err()
        .expect("counts_per_gram minimum");
    let expected = py_world(&mut py, &spec).expect_err("oracle counts_per_gram minimum");
    check_error("counts_per_gram minimum", &err, &expected);

    // Invalid orientation choice.
    let values: &[(&str, &str)] = &[("sensor_orientation", "sideways")];
    let spec = WorldSpec::new("load_cell badcell", values, 80);
    let world = make_world(&spec);
    let err = LoadCell::new(&world.config, world.sensor.clone())
        .err()
        .expect("orientation choice");
    let expected = py_world(&mut py, &spec).expect_err("oracle orientation choice");
    check_error("sensor_orientation choice", &err, &expected);

    // Unparsable integer option.
    let values: &[(&str, &str)] = &[("reference_tare_counts", "abc")];
    let spec = WorldSpec::new("load_cell badcell", values, 80);
    let world = make_world(&spec);
    let err = LoadCell::new(&world.config, world.sensor.clone())
        .err()
        .expect("tare counts parse");
    let expected = py_world(&mut py, &spec).expect_err("oracle tare counts parse");
    check_error("reference_tare_counts parse", &err, &expected);
}

#[test]
fn force_buffer_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    // `collections.deque(maxlen=int(sps / 2))`: 7 samples keep the last 5.
    let values: &[(&str, &str)] = &[
        ("reference_tare_counts", "1000"),
        ("counts_per_gram", "2.0"),
    ];
    let spec = WorldSpec::new("load_cell mycell", values, 10);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    cell.handle_do_ready().expect("rust ready");
    py.exec("_lc._handle_do_ready(100.0)").expect("oracle ready");
    let samples: Vec<(f64, i64, f64)> = (0..7)
        .map(|i| (1000.0 + i as f64 * 0.1, 1100 + i * 100, 0.0))
        .collect();
    let msg = batch_msg(&samples, 0, 0);
    cell.sensor_data_event(&msg).expect("rust sensor data");
    py_queue_batch(&mut py, &samples, 0, 0);
    py.exec("_lc._sensor_data_event(_s.batches.pop(0))")
        .expect("oracle sensor data");
    let expected = py.eval("_lc.get_status(100.0)").expect("oracle status");
    check_value(
        "get_status force buffer",
        &expected,
        &rv_of(&cell.get_status_value(100.0)),
    );
}

#[test]
fn tare_and_read_command_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[];
    let mut spec = WorldSpec::new("load_cell mycell", values, 10);
    spec.flush_hook = true;
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    // `klippy:ready` registers `_handle_ready`; the reactor callback it queues
    // attaches the sensor client and announces the calibration state. The
    // reactor runs until a timer shortly after the callback ends it.
    let expected = py.eval("list(_p.handlers.keys())").expect("oracle handlers");
    check_value(
        "ready handler registered",
        &expected,
        &rv_strs(&world.printer.handler_names()),
    );
    world
        .printer
        .send_event("klippy:ready", &[])
        .expect("rust klippy:ready");
    let reactor = world.printer.get_reactor();
    let ender = reactor.clone();
    reactor.register_timer(
        move |_| {
            ender.end();
            reactor::NEVER
        },
        reactor.monotonic() + 0.1,
    );
    reactor.run();
    py.exec("_p.send_event('klippy:ready')").expect("oracle ready");

    // Registrations: four mux commands keyed by the section's last word.
    let expected = py.eval("_gcode.registrations").expect("oracle registrations");
    check_value(
        "mux registrations",
        &expected,
        &rv_strs(&world.gcode.registrations.cloned()),
    );
    let expected = py.eval("_gcode.help").expect("oracle help");
    check_value(
        "command help",
        &expected,
        &rv_of(&py_str_dict(&world.gcode.help.cloned())),
    );
    let expected = py.eval("[list(x) for x in _wh.mux]").expect("oracle webhooks");
    let mux = world.webhooks.mux.cloned();
    check_value(
        "webhooks mux",
        &expected,
        &Rv::List(
            mux.iter()
                .map(|(path, key, value)| {
                    Rv::List(vec![Rv::str(path), Rv::str(key), Rv::str(value)])
                })
                .collect(),
        ),
    );

    // LOAD_CELL_TARE: ten samples of 1000 counts average to 1000.
    let samples = uniform_batch(1000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("LOAD_CELL_TARE", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_TARE", Some("mycell"), &gcmd)
        .expect("rust tare");
    py.eval("_gcode.dispatch_mux('LOAD_CELL_TARE', 'mycell', 'LOAD_CELL_TARE')")
        .expect("oracle tare");
    let expected = py.eval("_log").expect("oracle tare log");
    check_value("LOAD_CELL_TARE respond", &expected, &rv_strs(&world.log.get()));
    let expected = py.eval("_lc.get_tare_counts()").expect("oracle tare value");
    check_value(
        "tare_counts after tare",
        &expected,
        &Rv::int(cell.get_tare_counts().expect("tared")),
    );
    let expected = py.eval("_p.events").expect("oracle events");
    check_value("tare event", &expected, &rv_strs(&world.printer.event_names()));

    // LOAD_CELL_READ before calibration reports no force.
    let samples = uniform_batch(1000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("LOAD_CELL_READ", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_READ", Some("mycell"), &gcmd)
        .expect("rust read");
    py.eval("_gcode.dispatch_mux('LOAD_CELL_READ', 'mycell', 'LOAD_CELL_READ')")
        .expect("oracle read");
    let expected = py.eval("_log").expect("oracle read log");
    check_value(
        "LOAD_CELL_READ uncalibrated",
        &expected,
        &rv_strs(&world.log.get()),
    );

    // Calibrate: the config file writes and the event are compared.
    cell.set_calibration(Some(2.0), Some(1000.0))
        .expect("rust set_calibration");
    py.exec("_lc.set_calibration(2.0, 1000.0)").expect("oracle set_calibration");
    let expected = py.eval("_cf.sets").expect("oracle configfile sets");
    let sets = world.configfile.sets.cloned();
    check_value(
        "configfile sets",
        &expected,
        &Rv::List(
            sets.iter()
                .map(|(section, option, value)| {
                    Rv::Tuple(vec![Rv::str(section), Rv::str(option), Rv::str(value)])
                })
                .collect(),
        ),
    );
    let expected = py.eval("_p.events").expect("oracle events 2");
    check_value(
        "calibrate event",
        &expected,
        &rv_strs(&world.printer.event_names()),
    );
    let samples = uniform_batch(1020, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("LOAD_CELL_READ", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_READ", Some("mycell"), &gcmd)
        .expect("rust read 2");
    py.eval("_gcode.dispatch_mux('LOAD_CELL_READ', 'mycell', 'LOAD_CELL_READ')")
        .expect("oracle read 2");
    let expected = py.eval("_log").expect("oracle read log 2");
    check_value(
        "LOAD_CELL_READ calibrated",
        &expected,
        &rv_strs(&world.log.get()),
    );
}

#[test]
fn avg_counts_checks_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[];
    let mut spec = WorldSpec::new("load_cell mycell", values, 10);
    spec.flush_hook = true;
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    cell.handle_do_ready().expect("rust ready");
    py.exec("_lc._handle_do_ready(100.0)").expect("oracle ready");

    // Saturated sample: the counts column reaches the sensor maximum.
    let mut samples = uniform_batch(1000, 10);
    samples[4].1 = 0x7FFFFF;
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let err = cell.avg_counts(None).expect_err("rust saturated");
    let expected = py.eval("_lc.avg_counts()").expect_err("oracle saturated");
    check_error("avg_counts saturated", &err, &expected);

    // Sensor errors: `_start_errors` is 0, the batch reports 3 errors and 2
    // overflows, and the message reports their sum.
    let samples = uniform_batch(1000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 3, 2));
    py_queue_batch(&mut py, &samples, 3, 2);
    let err = cell.avg_counts(None).expect_err("rust errors");
    let expected = py.eval("_lc.avg_counts()").expect_err("oracle errors");
    check_error("avg_counts sensor errors", &err, &expected);

    // A clean average over an explicit sample count.
    let samples = vec![(1000.0, 1000, 0.5), (1000.1, 1002, 0.5), (1000.2, 1004, 0.5)];
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let got = cell.avg_counts(Some(3.0)).expect("rust average");
    let expected = py.eval("_lc.avg_counts(3)").expect("oracle average");
    check_value("avg_counts value", &expected, &Rv::float(got));
}

#[test]
fn collector_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[
        ("reference_tare_counts", "1000"),
        ("counts_per_gram", "2.0"),
    ];
    let mut spec = WorldSpec::new("load_cell mycell", values, 10);
    spec.flush_hook = true;
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    cell.handle_do_ready().expect("rust ready");
    py.exec("_lc._handle_do_ready(100.0)").expect("oracle ready");

    // collect_min collects until the sample count is reached; the rows are the
    // rewritten `[time, grams, counts, tare_counts]` lists.
    let samples: Vec<(f64, i64, f64)> = vec![
        (1000.0, 1100, 0.0),
        (1000.1, 1200, 0.0),
        (1000.2, 1300, 0.0),
    ];
    world.sensor.queue_batch(batch_msg(&samples, 2, 1));
    py_queue_batch(&mut py, &samples, 2, 1);
    let collector: Arc<LoadCellSampleCollector> = cell.get_collector();
    let got = collector.collect_min(3.0).expect("rust collect_min");
    let expected = py
        .eval("(lambda c: c.collect_min(3))(_lc.get_collector())")
        .expect("oracle collect_min");
    check_value(
        "collect_min samples",
        &samples_of(&expected),
        &rv_of_list(&got.samples),
    );
    check_value(
        "collect_min errors",
        &errors_of(&expected),
        &rv_errors(got.errors),
    );

    // collect_until returns as soon as a buffered sample is at/after the time.
    let samples: Vec<(f64, i64, f64)> = vec![(1001.0, 1400, 0.0), (1001.5, 1500, 0.0)];
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let collector = cell.get_collector();
    collector.start_collecting(None);
    world.sensor.flush().expect("rust flush");
    py.exec("_c2 = _lc.get_collector(); _c2.start_collecting(); _s.flush()")
        .expect("oracle start");
    let got = collector.collect_until(1001.2).expect("rust collect_until");
    let expected = py.eval("_c2.collect_until(1001.2)").expect("oracle collect_until");
    check_value(
        "collect_until samples",
        &samples_of(&expected),
        &rv_of_list(&got.samples),
    );
    check_value(
        "collect_until errors",
        &errors_of(&expected),
        &rv_errors(got.errors),
    );

    // Fileoutput substitutes a single `(0., 0., 0.)` sample.
    let mut fileoutput_spec = WorldSpec::new("load_cell mycell", values, 10);
    fileoutput_spec.fileoutput = true;
    assert!(py_world(&mut py, &fileoutput_spec).is_ok(), "oracle fileoutput world");
    let fileoutput_world = make_world(&fileoutput_spec);
    let fileoutput_cell = fileoutput_world.cell();
    let got = fileoutput_cell
        .get_collector()
        .collect_min(5.0)
        .expect("rust fileoutput collect");
    let expected = py
        .eval("(lambda c: c.collect_min(5))(_lc.get_collector())")
        .expect("oracle fileoutput collect");
    check_value(
        "collect_min fileoutput samples",
        &samples_of(&expected),
        &rv_of_list(&got.samples),
    );
    check_value(
        "collect_min fileoutput errors",
        &errors_of(&expected),
        &rv_errors(got.errors),
    );

    // Mini-timeout: the estimated print time is already past the deadline.
    let mut timeout_spec = WorldSpec::new("load_cell mycell", values, 10);
    timeout_spec.estimated = 100000.0;
    assert!(py_world(&mut py, &timeout_spec).is_ok(), "oracle timeout world");
    let timeout_world = make_world(&timeout_spec);
    let timeout_cell = timeout_world.cell();
    let collector = timeout_cell.get_collector();
    let err = collector
        .collect_until(1.0)
        .err()
        .expect("rust timeout");
    let expected = py
        .eval("_lc.get_collector().collect_until(1.0)")
        .expect_err("oracle timeout");
    check_error("collector timeout", &err, &expected);
}

#[test]
fn diagnostic_command_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    // Samples with sensor errors and one saturated reading. The reference
    // delivers them inside the 10 second reactor pause; the port's pause is a
    // real sleep, so a helper thread hands them over the same way.
    let values: &[(&str, &str)] = &[];
    let spec = WorldSpec::new("load_cell mycell", values, 20);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    cell.handle_do_ready().expect("rust ready");
    py.exec("_lc._handle_do_ready(100.0)\n_p.reactor.sensor = _s")
        .expect("oracle ready and pause hook");
    let samples: Vec<(f64, i64, f64)> = vec![
        (100.0, 10, 0.0),
        (100.1, 20, 0.0),
        (100.2, 0x7FFFFF, 0.0),
        (100.3, 30, 0.0),
        (100.4, 20, 0.0),
    ];
    world.sensor.queue_batch(batch_msg(&samples, 1, 2));
    py_queue_batch(&mut py, &samples, 1, 2);
    let sensor = Arc::clone(&world.sensor);
    let flusher = kos::thread::spawn(move || {
        kos::time::sleep(1.0);
        let _ = sensor.flush();
    });
    let gcmd = world.gcode.command("LOAD_CELL_DIAGNOSTIC", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_DIAGNOSTIC", Some("mycell"), &gcmd)
        .expect("rust diagnostic");
    flusher.join();
    py.eval("_gcode.dispatch_mux('LOAD_CELL_DIAGNOSTIC', 'mycell', 'LOAD_CELL_DIAGNOSTIC')")
        .expect("oracle diagnostic");
    let expected = py.eval("_log").expect("oracle diagnostic log");
    check_value("diagnostic samples", &expected, &rv_strs(&world.log.get()));

    // No samples: the command raises before reporting the counts.
    let spec = WorldSpec::new("load_cell mycell", values, 20);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world 2");
    let world = make_world(&spec);
    world.cell();
    let gcmd = world.gcode.command("LOAD_CELL_DIAGNOSTIC", &[]);
    let err = world
        .gcode
        .dispatch_mux("LOAD_CELL_DIAGNOSTIC", Some("mycell"), &gcmd)
        .expect_err("rust diagnostic empty");
    let expected = py
        .eval("_gcode.dispatch_mux('LOAD_CELL_DIAGNOSTIC', 'mycell', 'LOAD_CELL_DIAGNOSTIC')")
        .expect_err("oracle diagnostic empty");
    check_error("diagnostic no samples", &err, &expected);
    let expected = py.eval("_log").expect("oracle diagnostic log 2");
    check_value(
        "diagnostic no samples log",
        &expected,
        &rv_strs(&world.log.get()),
    );
}

#[test]
fn guided_calibration_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[];
    let mut spec = WorldSpec::new("load_cell mycell", values, 10);
    spec.flush_hook = true;
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    cell.handle_do_ready().expect("rust ready");
    py.exec("_lc._handle_do_ready(100.0)").expect("oracle ready");

    let gcmd = world.gcode.command("LOAD_CELL_CALIBRATE", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_CALIBRATE", Some("mycell"), &gcmd)
        .expect("rust calibrate start");
    py.eval("_gcode.dispatch_mux('LOAD_CELL_CALIBRATE', 'mycell', 'LOAD_CELL_CALIBRATE')")
        .expect("oracle calibrate start");
    let expected = py.eval("_log").expect("oracle start log");
    check_value("calibrate start respond", &expected, &rv_strs(&world.log.get()));
    let expected = py.eval("_gcode.registrations").expect("oracle registrations");
    check_value(
        "guided registrations",
        &expected,
        &rv_strs(&world.gcode.registrations.cloned()),
    );
    let expected = py.eval("_gcode.help").expect("oracle help");
    check_value(
        "guided help",
        &expected,
        &rv_of(&py_str_dict(&world.gcode.help.cloned())),
    );

    // TARE: ten samples of 1000 counts.
    let samples = uniform_batch(1000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("TARE", &[]);
    world.gcode.dispatch_plain("TARE", &gcmd).expect("rust guided tare");
    py.eval("_gcode.dispatch_plain('TARE', 'TARE')").expect("oracle guided tare");
    let expected = py.eval("_log").expect("oracle tare log");
    check_value("guided tare respond", &expected, &rv_strs(&world.log.get()));

    // 500000 counts over 1000 g: 500 counts/gram, 16.7 kg capacity.
    let samples = uniform_batch(501000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("CALIBRATE GRAMS=1000", &[("GRAMS", "1000")]);
    world.gcode.dispatch_plain("CALIBRATE", &gcmd).expect("rust calibrate 1");
    py.eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=1000', {'GRAMS': '1000'})")
        .expect("oracle calibrate 1");
    let expected = py.eval("_log").expect("oracle calibrate log 1");
    check_value("calibrate respond 1", &expected, &rv_strs(&world.log.get()));

    // The same delta over 50 g: 10000 counts/gram, capacity below 1 kg.
    let samples = uniform_batch(501000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("CALIBRATE GRAMS=50", &[("GRAMS", "50")]);
    world.gcode.dispatch_plain("CALIBRATE", &gcmd).expect("rust calibrate 2");
    py.eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=50', {'GRAMS': '50'})")
        .expect("oracle calibrate 2");
    let expected = py.eval("_log").expect("oracle calibrate log 2");
    check_value("calibrate respond 2", &expected, &rv_strs(&world.log.get()));

    // 100000 counts over 1000 g: 100 counts/gram, capacity above 25 kg.
    let samples = uniform_batch(101000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("CALIBRATE GRAMS=1000", &[("GRAMS", "1000")]);
    world.gcode.dispatch_plain("CALIBRATE", &gcmd).expect("rust calibrate 3");
    py.eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=1000', {'GRAMS': '1000'})")
        .expect("oracle calibrate 3");
    let expected = py.eval("_log").expect("oracle calibrate log 3");
    check_value("calibrate respond 3", &expected, &rv_strs(&world.log.get()));

    // Back to a valid 500 counts/gram, then ACCEPT stores it.
    let samples = uniform_batch(501000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("CALIBRATE GRAMS=1000", &[("GRAMS", "1000")]);
    world.gcode.dispatch_plain("CALIBRATE", &gcmd).expect("rust calibrate 4");
    py.eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=1000', {'GRAMS': '1000'})")
        .expect("oracle calibrate 4");
    let expected = py.eval("_log").expect("oracle calibrate log 4");
    check_value("calibrate respond 4", &expected, &rv_strs(&world.log.get()));

    let gcmd = world.gcode.command("ACCEPT", &[]);
    world.gcode.dispatch_plain("ACCEPT", &gcmd).expect("rust accept");
    py.eval("_gcode.dispatch_plain('ACCEPT', 'ACCEPT')").expect("oracle accept");
    let expected = py.eval("_log").expect("oracle accept log");
    check_value("accept respond", &expected, &rv_strs(&world.log.get()));
    let expected = py.eval("_cf.sets").expect("oracle configfile sets");
    let sets = world.configfile.sets.cloned();
    check_value(
        "accept configfile sets",
        &expected,
        &Rv::List(
            sets.iter()
                .map(|(section, option, value)| {
                    Rv::Tuple(vec![Rv::str(section), Rv::str(option), Rv::str(value)])
                })
                .collect(),
        ),
    );
    let expected = py.eval("_p.events").expect("oracle events");
    check_value(
        "accept events",
        &expected,
        &rv_strs(&world.printer.event_names()),
    );
    let expected = py.eval("_lc.get_status(100.0)").expect("oracle status");
    check_value(
        "status after accept",
        &expected,
        &rv_of(&cell.get_status_value(100.0)),
    );
    let expected = py.eval("_gcode.registrations").expect("oracle registrations 2");
    check_value(
        "accept unregisters",
        &expected,
        &rv_strs(&world.gcode.registrations.cloned()),
    );
}

#[test]
fn guided_calibration_error_paths() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[];
    let mut spec = WorldSpec::new("load_cell mycell", values, 10);
    spec.flush_hook = true;
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    cell.handle_do_ready().expect("rust ready");
    py.exec("_lc._handle_do_ready(100.0)").expect("oracle ready");
    let gcmd = world.gcode.command("LOAD_CELL_CALIBRATE", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_CALIBRATE", Some("mycell"), &gcmd)
        .expect("rust calibrate start");
    py.eval("_gcode.dispatch_mux('LOAD_CELL_CALIBRATE', 'mycell', 'LOAD_CELL_CALIBRATE')")
        .expect("oracle calibrate start");

    // CALIBRATE before TARE only responds.
    let gcmd = world.gcode.command("CALIBRATE GRAMS=1000", &[("GRAMS", "1000")]);
    world.gcode.dispatch_plain("CALIBRATE", &gcmd).expect("rust no tare");
    py.eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=1000', {'GRAMS': '1000'})")
        .expect("oracle no tare");
    let expected = py.eval("_log").expect("oracle no tare log");
    check_value(
        "calibrate without tare",
        &expected,
        &rv_strs(&world.log.get()),
    );

    // A second calibration tool cannot start while one is active.
    let gcmd = world.gcode.command("LOAD_CELL_CALIBRATE", &[]);
    let err = world
        .gcode
        .dispatch_mux("LOAD_CELL_CALIBRATE", Some("mycell"), &gcmd)
        .expect_err("rust double calibrate");
    let expected = py
        .eval("_gcode.dispatch_mux('LOAD_CELL_CALIBRATE', 'mycell', 'LOAD_CELL_CALIBRATE')")
        .expect_err("oracle double calibrate");
    check_error("already calibrating", &err, &expected);

    // TARE, then a force below the 50 g minimum is rejected by the command.
    let samples = uniform_batch(1000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("TARE", &[]);
    world.gcode.dispatch_plain("TARE", &gcmd).expect("rust tare");
    py.eval("_gcode.dispatch_plain('TARE', 'TARE')").expect("oracle tare");
    let gcmd = world.gcode.command("CALIBRATE GRAMS=10", &[("GRAMS", "10")]);
    let err = world
        .gcode
        .dispatch_plain("CALIBRATE", &gcmd)
        .expect_err("rust grams minimum");
    let expected = py
        .eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=10', {'GRAMS': '10'})")
        .expect_err("oracle grams minimum");
    check_error("grams minimum", &err, &expected);

    // Identical tare and calibration readings: the counts delta is zero, so
    // the counts-per-gram division raises before the "same readings" check.
    let samples = uniform_batch(1000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("CALIBRATE GRAMS=1000", &[("GRAMS", "1000")]);
    let err = world
        .gcode
        .dispatch_plain("CALIBRATE", &gcmd)
        .expect_err("rust zero counts per gram");
    let expected = py
        .eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=1000', {'GRAMS': '1000'})")
        .expect_err("oracle zero counts per gram");
    check_error("zero counts per gram", &err, &expected);
    let expected = py.eval("_log").expect("oracle zero cpg log");
    check_value("zero cpg respond", &expected, &rv_strs(&world.log.get()));

    // Readings less than 1% apart: different counts, tiny percentage delta.
    let samples = uniform_batch(50000, 10);
    world.sensor.queue_batch(batch_msg(&samples, 0, 0));
    py_queue_batch(&mut py, &samples, 0, 0);
    let gcmd = world.gcode.command("CALIBRATE GRAMS=1000", &[("GRAMS", "1000")]);
    let err = world
        .gcode
        .dispatch_plain("CALIBRATE", &gcmd)
        .expect_err("rust under 1 percent");
    let expected = py
        .eval("_gcode.dispatch_plain('CALIBRATE', 'CALIBRATE GRAMS=1000', {'GRAMS': '1000'})")
        .expect_err("oracle under 1 percent");
    check_error("under 1 percent", &err, &expected);
    let expected = py.eval("_log").expect("oracle under 1 percent log");
    check_value(
        "under 1 percent respond",
        &expected,
        &rv_strs(&world.log.get()),
    );

    // ACCEPT with an incomplete calibration reports and cannot store it.
    let gcmd = world.gcode.command("ACCEPT", &[]);
    let err = world
        .gcode
        .dispatch_plain("ACCEPT", &gcmd)
        .expect_err("rust incomplete accept");
    let expected = py
        .eval("_gcode.dispatch_plain('ACCEPT', 'ACCEPT')")
        .expect_err("oracle incomplete accept");
    check_error("incomplete accept", &err, &expected);
    let expected = py.eval("_log").expect("oracle incomplete log");
    check_value(
        "incomplete accept respond",
        &expected,
        &rv_strs(&world.log.get()),
    );

    // The failed ACCEPT already unregistered the interactive commands, so a
    // new tool can start; ABORT then unregisters it again and responds.
    let gcmd = world.gcode.command("LOAD_CELL_CALIBRATE", &[]);
    world
        .gcode
        .dispatch_mux("LOAD_CELL_CALIBRATE", Some("mycell"), &gcmd)
        .expect("rust calibrate restart");
    py.eval("_gcode.dispatch_mux('LOAD_CELL_CALIBRATE', 'mycell', 'LOAD_CELL_CALIBRATE')")
        .expect("oracle calibrate restart");
    let gcmd = world.gcode.command("ABORT", &[]);
    world.gcode.dispatch_plain("ABORT", &gcmd).expect("rust abort");
    py.eval("_gcode.dispatch_plain('ABORT', 'ABORT')").expect("oracle abort");
    let expected = py.eval("_log").expect("oracle abort log");
    check_value("abort respond", &expected, &rv_strs(&world.log.get()));
    let expected = py.eval("_gcode.registrations").expect("oracle abort registrations");
    check_value(
        "abort unregisters",
        &expected,
        &rv_strs(&world.gcode.registrations.cloned()),
    );
    assert!(!cell.is_calibrated());
}

#[test]
fn api_client_helper_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    // Dispatch order, removal of falsy clients, and a client added by a
    // callback (visited from the next send on, like `list(self.client_cbs)`).
    let printer = FakePrinter::new();
    let helper = ApiClientHelper::new(printer.printer_arc());
    let seen = Arc::new(GCell::new(Vec::<String>::new()));
    let helper_slot: Arc<GCell<Option<Arc<ApiClientHelper>>>> = Arc::new(GCell::new(None));
    let added = Arc::new(GCell::new(false));
    let seen_a = Arc::clone(&seen);
    let seen_d = Arc::clone(&seen);
    let slot_a = Arc::clone(&helper_slot);
    helper.add_client(Box::new(move |_| {
        seen_a.with_mut(|v| v.push("A".to_string()));
        let first = !added.replace(true);
        if first {
            if let Some(helper) = slot_a.borrow().as_ref() {
                let seen_d = Arc::clone(&seen_d);
                helper.add_client(Box::new(move |_| {
                    seen_d.with_mut(|v| v.push("D".to_string()));
                    Ok(true)
                }));
            }
        }
        Ok(true)
    }));
    let seen_b = Arc::clone(&seen);
    helper.add_client(Box::new(move |_| {
        seen_b.with_mut(|v| v.push("B".to_string()));
        Ok(false)
    }));
    let seen_c = Arc::clone(&seen);
    helper.add_client(Box::new(move |_| {
        seen_c.with_mut(|v| v.push("C".to_string()));
        Ok(true)
    }));
    helper_slot.replace(Some(Arc::clone(&helper)));
    py.exec(
        "_hlp = _lcm.ApiClientHelper(FakePrinter())\n\
         _seen = []\n\
         _added = [False]\n\
         def _cb_a(msg):\n\
         \x20   _seen.append('A')\n\
         \x20   if not _added[0]:\n\
         \x20       _added[0] = True\n\
         \x20       _hlp.add_client(lambda m: _seen.append('D') or True)\n\
         \x20   return True\n\
         def _cb_b(msg):\n\
         \x20   _seen.append('B')\n\
         \x20   return False\n\
         def _cb_c(msg):\n\
         \x20   _seen.append('C')\n\
         \x20   return True\n\
         _hlp.add_client(_cb_a)\n\
         _hlp.add_client(_cb_b)\n\
         _hlp.add_client(_cb_c)",
    )
    .expect("oracle helper");
    let msg = PyValue::Dict(IndexMap::new());
    helper.send(&msg).expect("rust send 1");
    helper.send(&msg).expect("rust send 2");
    py.exec("_hlp.send({}); _hlp.send({})").expect("oracle sends");
    let expected = py.eval("_seen").expect("oracle seen");
    let actual = seen.cloned();
    check_value("client dispatch order", &expected, &rv_strs(&actual));

    // Webhooks client: the static header on subscribe, then batch forwarding
    // through the real `bulk_sensor.BatchWebhooksClient`.
    let printer = FakePrinter::new();
    let webhooks = FakeWebHooks::new();
    printer
        .add_object("webhooks", Arc::clone(&webhooks) as ObjectRef)
        .expect("register webhooks");
    let helper = ApiClientHelper::new(printer.printer_arc());
    let mut header = IndexMap::new();
    header.insert(
        PyValue::Str("header".to_string()),
        PyValue::List(vec![
            PyValue::Str("time".to_string()),
            PyValue::Str("force (g)".to_string()),
            PyValue::Str("counts".to_string()),
            PyValue::Str("tare_counts".to_string()),
        ]),
    );
    helper
        .add_mux_endpoint(
            "load_cell/dump_force",
            "load_cell",
            "mycell",
            PyValue::Dict(header),
        )
        .expect("rust mux endpoint");
    let callback = webhooks
        .callbacks
        .borrow()
        .get("load_cell/dump_force\0load_cell\0mycell")
        .cloned()
        .expect("mux callback");
    let conn = FakeClientConn::new();
    let request = FakeWebRequest::new(Arc::clone(&conn));
    callback(Arc::clone(&request) as WebRequest).expect("rust subscribe");
    helper
        .send(&batch_msg(&[(1.0, 2, 3.0)], 0, 0))
        .expect("rust batch");
    py.exec(
        "_p3 = FakePrinter()\n\
         _wh3 = FakeWebHooks()\n\
         _p3.objects['webhooks'] = _wh3\n\
         _hlp3 = _lcm.ApiClientHelper(_p3)\n\
         _hlp3.add_mux_endpoint('load_cell/dump_force', 'load_cell', 'mycell',\
         {'header': ['time', 'force (g)', 'counts', 'tare_counts']})\n\
         _wr3 = FakeWebRequest()\n\
         _wh3.callbacks[('load_cell/dump_force', 'load_cell', 'mycell')](_wr3)\n\
         _hlp3.send({'data': [(1.0, 2, 3.0)], 'errors': 0, 'overflows': 0})",
    )
    .expect("oracle webhooks client");
    let expected = py.eval("_wr3.sent").expect("oracle header");
    let sent = request.sent.cloned();
    check_value("webhooks header", &expected, &rv_of_list(&sent));
    let expected = py.eval("_wr3.conn.sent").expect("oracle batch");
    let sent = conn.sent.cloned();
    check_value("webhooks batch", &expected, &rv_of_list(&sent));
}

#[test]
fn sensor_data_event_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[];
    let spec = WorldSpec::new("load_cell mycell", values, 10);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");
    let world = make_world(&spec);
    let cell = world.cell();
    let samples = [(100.0, 1100, 0.5), (100.1, 1200, 1.0)];

    // Uncalibrated: grams are None and the tare entry is None.
    let msg = batch_msg(&samples, 1, 2);
    let keep = cell.sensor_data_event(&msg).expect("rust sensor event");
    py_queue_batch(&mut py, &samples, 1, 2);
    let expected = py
        .eval("_lc._sensor_data_event(_s.batches.pop(0))")
        .expect("oracle sensor event");
    check_value("sensor_data_event keep", &expected, &Rv::Bool(keep));

    // Clients receive the rewritten rows.
    let received = Arc::new(GCell::new(Vec::<PyValue>::new()));
    let received_client = Arc::clone(&received);
    cell.add_client(Box::new(move |msg| {
        received_client.with_mut(|v| v.push(msg.clone()));
        Ok(true)
    }));
    py.exec("_got = []\n_lc.add_client(lambda m: _got.append(m) or True)")
        .expect("oracle client");
    cell.sensor_data_event(&msg).expect("rust sensor event 2");
    py_queue_batch(&mut py, &samples, 1, 2);
    py.exec("_lc._sensor_data_event(_s.batches.pop(0))")
        .expect("oracle sensor event 2");
    let expected = py.eval("_got").expect("oracle received");
    let actual = received.cloned();
    check_value("client message", &expected, &rv_of_list(&actual));

    // Calibrated and tared: grams are computed from the counts.
    cell.set_calibration(Some(2.0), Some(1000.0))
        .expect("rust calibrate");
    py.exec("_lc.set_calibration(2.0, 1000.0)").expect("oracle calibrate");
    cell.tare(1000.0).expect("rust tare");
    py.exec("_lc.tare(1000.0)").expect("oracle tare");
    let calibrated = [(100.0, 1100, 0.0), (100.1, 1250, 0.0), (100.2, 1400, 0.0)];
    let msg = batch_msg(&calibrated, 0, 0);
    cell.sensor_data_event(&msg).expect("rust calibrated event");
    py_queue_batch(&mut py, &calibrated, 0, 0);
    py.exec("_lc._sensor_data_event(_s.batches.pop(0))")
        .expect("oracle calibrated event");
    let expected = py.eval("_got").expect("oracle received 2");
    let actual = received.cloned();
    check_value(
        "client message calibrated",
        &expected,
        &rv_of_list(&actual),
    );
}

#[test]
fn load_config_dispatch_parity() {
    let Some(mut py) = start_oracle() else {
        eprintln!("skipping: KLIPPY_PYTHON is not set");
        return;
    };
    let values: &[(&str, &str)] = &[];
    let spec = WorldSpec::new("load_cell mycell", values, 80);
    assert!(py_world(&mut py, &spec).is_ok(), "oracle world");

    // The sensor type names and their order come from the reference modules.
    let expected = py
        .eval(
            "list(_lcm.hx71x.HX71X_SENSOR_TYPES) \
             + list(_lcm.ads1220.ADS1220_SENSOR_TYPE) \
             + list(_lcm.ads131m0x.ADS131M0X_SENSOR_TYPES)",
        )
        .expect("oracle sensor types");
    let names: Vec<String> = sensor_type_names()
        .iter()
        .map(|name| (*name).to_string())
        .collect();
    check_value("sensor_type names", &expected, &rv_strs(&names));

    // An unknown sensor type is rejected by the choice lookup.
    let values: &[(&str, &str)] = &[("sensor_type", "nope")];
    let spec = WorldSpec::new("load_cell mycell", values, 80);
    let world = make_world(&spec);
    let err = extras::load_cell::load_config(&world.config)
        .err()
        .expect("sensor type choice");
    let expected = py
        .eval("_lcm.load_config(FakeConfig(_p, 'load_cell mycell', {'sensor_type': 'nope'}))")
        .expect_err("oracle sensor type choice");
    check_error("sensor_type invalid", &err, &expected);

    // A known sensor type reaches the driver's own configuration validation
    // (the fake config has none of the driver's options), not the choice error.
    for name in ["hx711", "hx717", "ads1220", "ads131m02", "ads131m04"] {
        let values: &[(&str, &str)] = &[("sensor_type", name)];
        let spec = WorldSpec::new("load_cell mycell", values, 80);
        let world = make_world(&spec);
        let err = extras::load_cell::load_config(&world.config)
            .err()
            .expect("driver configuration validation");
        assert!(
            !err.message().contains("is not a valid choice"),
            "{name}: dispatch must reach the driver, got {:?}",
            err.message()
        );
        let expected = py
            .eval(&format!(
                "_lcm.load_config(FakeConfig(_p, 'load_cell mycell', {{'sensor_type': '{name}'}}))"
            ))
            .expect_err("oracle driver configuration validation");
        assert!(
            !expected.message.contains("is not a valid choice"),
            "{name}: the reference must reach the driver, got {:?}",
            expected.message
        );
        assert_eq!(err.class_name(), expected.class_name, "{name}: error class");
    }
}
