//! Parity tests for `klippy/extras/pca9632.py`.
//!
//! Every expected value comes from the live `pyoracle` (CPython 3.8.2
//! with the frozen `klippy/`): the reference `PCA9632` is built with
//! minimal Python fakes for the printer, config, `MCU_I2C` and
//! `LEDHelper`, driven with the same inputs as the Rust port.
//!
//! The Rust side drives the port through a fake `McuI2cApi` (the
//! owner's trait, recorded with the same `i2c_write` shapes the
//! Python fake records) and a real `LEDHelper` built by
//! `PCA9632::from_i2c` itself, seeded from the `initial_*` config
//! options the fixture sets to the same colour the Python fake
//! carries. The fixture's `FakePrinter` therefore also registers a
//! fake `gcode` object (`register_mux_command`) and a fake
//! `gcode_macro` object, which is all that constructing the real
//! `LEDHelper` (and its real template evaluator) needs.
//! Register writes, cache behaviour, clock conversion, status
//! dicts and every error path are compared value-by-value.
//! Without the oracle (`KLIPPY_PYTHON` unset) each test skips.

use kos::sync::Mutex;
use std::sync::Arc;

use extras::bus::McuI2cApi;
use extras::display::display::{DisplayGroup, DisplayTemplateApi, GlyphTable, PrinterDisplayTemplateApi};
use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras::pca9632::{module_factory, PCA9632};
use extras_macros_templates::gcode_macro::Value as MacroValue;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandHandler, GCmd, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{
    self, CommandQueue, Mcu, McuApi, QuerySendOpts, SendOpts, TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT};
use pycompat::text::trunc_float_to_bigint;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Fakes of the contract traits (shared with the Python fakes below).
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct I2cCall {
    data: Vec<u8>,
    minclock: i64,
    reqclock: u64,
    retry: bool,
}

struct FakeI2c {
    mcu: Mcu,
    calls: klippy_api::GCell<Vec<I2cCall>>,
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_i2c_address(&self) -> i64 {
        98
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.calls.borrow_mut().push(I2cCall {
            data: data.to_vec(),
            minclock: opts.minclock,
            reqclock: opts.reqclock as u64,
            retry: false,
        });
        Ok(())
    }
    fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        self.calls.borrow_mut().push(I2cCall {
            data: data.to_vec(),
            minclock: opts.minclock,
            reqclock: opts.reqclock as u64,
            retry: opts.retry,
        });
        Ok(())
    }
    fn i2c_read(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        Err(KlippyError::Config("unused read".to_string()))
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::Config("unused transfer".to_string()))
    }
}

struct FakeGCode;

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _c: &str,
        _f: Option<GCodeCommandHandler>,
        _o: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _cb: OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _c: &str,
        _cl: &str,
        _p: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("unused")
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeMacro;

impl PrinterGCodeMacroApi for FakeMacro {
    fn load_template(&self, _config: &Config, _option: &str, _default: Option<&str>) -> KResult<TemplateWrapper> {
        unreachable!("pca9632 never loads templates")
    }
    fn create_template_context(&self, _eventtime: Option<f64>) -> IndexMap<String, MacroValue> {
        IndexMap::new()
    }
}

klippy_api::printer_object!(FakeMacro; dyn PrinterGCodeMacroApi);

struct FakeEmptyDisplayTemplates;

impl PrinterDisplayTemplateApi for FakeEmptyDisplayTemplates {
    fn get_display_templates(&self) -> IndexMap<String, Arc<dyn DisplayTemplateApi>> {
        IndexMap::new()
    }
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        IndexMap::new()
    }
    fn get_display_glyphs(&self) -> GlyphTable {
        IndexMap::new()
    }
}

klippy_api::printer_object!(FakeEmptyDisplayTemplates; dyn PrinterDisplayTemplateApi);

struct FakeAsyncWrapper;
impl mcu::AsyncResponseWrapperApi for FakeAsyncWrapper {
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
    fn setup_pin(&self, _pin_type: &str, _pin_params: mcu::PinParams) -> KResult<mcu::Pin> {
        Err(KlippyError::Config("unused pin".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, _cb: mcu::ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: mcu::AddConfigCmdOpts) -> KResult<()> {
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
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<mcu::CommandQueue>,
    ) -> KResult<mcu::CommandWrapper> {
        Err(KlippyError::Config("unused command".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: mcu::LookupQueryCommandOpts,
    ) -> KResult<mcu::CommandQueryWrapper> {
        Err(KlippyError::Config("unused query command".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> mcu::CommandQueue {
        mcu::CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> mcu::AsyncResponseWrapper {
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
        Ok(0.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        // Matches the Python fake: `int(print_time * 1000000)`.
        trunc_float_to_bigint(print_time * 1_000_000.0)
            .inner()
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, eventtime))
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
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Config("unused trigger".to_string()))
    }
}

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: klippy_api::GCell<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: klippy_api::GCell::new(Vec::new()),
        }
    }

    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }

    fn handler_events(&self) -> Vec<String> {
        self.handlers.borrow().iter().map(|(e, _)| e.clone()).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "Ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.objects.lock().get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{}'",
                name
            ))),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.objects.lock().get(section) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unable to load module '{}'",
                section
            ))),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let n = self.handlers.borrow().len();
        for i in 0..n {
            let mut handlers = self.handlers.borrow_mut();
            if handlers[i].0 == event {
                let cb = &mut handlers[i].1;
                results.push(cb(params)?);
            }
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    options: klippy_api::GCell<indexmap::IndexMap<String, String>>,
}

impl FakeConfig {
    fn new(printer: Printer, options: &[(&str, &str)]) -> FakeConfig {
        FakeConfig {
            printer,
            name: "pca9632 p6led".to_string(),
            options: klippy_api::GCell::new(
                options
                    .iter()
                    .map(|(k, v)| (k.to_string(), v.to_string()))
                    .collect(),
            ),
        }
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
        if let Some(v) = self.options.borrow().get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        if let Some(v) = self.options.borrow().get(option) {
            return v.parse::<i64>().map(Some).map_err(|_| KlippyError::Config(
                format!("Option '{}' in section '{}' must be an integer", option, self.name),
            ));
        }
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        if let Some(v) = self.options.borrow().get(option) {
            return v.parse::<f64>().map(Some).map_err(|_| KlippyError::Config(
                format!("Option '{}' in section '{}' must be a float", option, self.name),
            ));
        }
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getchoice(
        &self,
        option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::Config(format!(
            "Option '{}' in section '{}' is not a valid choice",
            option, self.name
        )))
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        match opts.default {
            Some(inner) => Ok(inner.unwrap_or(PyValue::None)),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getintlist(&self, option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getfloatlist(
        &self,
        option: &str,
        opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        match opts.default {
            Some(inner) => Ok(inner),
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' must be specified",
                option, self.name
            ))),
        }
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig::new(self.printer.clone(), &[("section", section)]))
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

/// Build the Rust-side fixture mirroring the Python fakes.
struct Fixture {
    printer: Arc<FakePrinter>,
    i2c: Arc<FakeI2c>,
    obj: Arc<PCA9632>,
}

fn fixture(color_order: Option<&str>, color: Vec<[f64; 4]>) -> Fixture {
    let printer: Arc<FakePrinter> = Arc::new(FakePrinter::new());
    printer.add("gcode", Arc::new(FakeGCode) as ObjectRef);
    printer.add("gcode_macro", Arc::new(FakeMacro) as ObjectRef);
    printer.add("display_template", Arc::new(FakeEmptyDisplayTemplates) as ObjectRef);
    let printer_api: Printer = printer.clone();
    // The LED helper seeds its colour from `initial_RED` etc.; the
    // Python fake seeds its own colour from `make_pca`'s argument.
    // Both start at `color[0]` (the one row `led_count=1` holds).
    let mut opts: Vec<(String, String)> = Vec::new();
    if let Some(o) = color_order {
        opts.push(("color_order".to_string(), o.to_string()));
    }
    for (key, old) in [("initial_RED", 0), ("initial_GREEN", 1), ("initial_BLUE", 2), ("initial_WHITE", 3)] {
        let v = color.first().map(|row| row[old]).unwrap_or(0.0);
        opts.push((key.to_string(), format!("{v:?}")));
    }
    let opts: Vec<(&str, &str)> = opts
        .iter()
        .map(|(k, v)| (k.as_str(), v.as_str()))
        .collect();
    let config: Config = Arc::new(FakeConfig::new(printer_api.clone(), &opts));
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer_api });
    let i2c = Arc::new(FakeI2c { mcu, calls: klippy_api::GCell::new(Vec::new()) });
    let obj = PCA9632::from_i2c(&config, i2c.clone()).expect("build PCA9632");
    Fixture { printer, i2c, obj }
}

// ---------------------------------------------------------------------------
// pyoracle side: same fakes in Python, same inputs.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r"
import sys
import types

Error = type('Error', (Exception,), {})

I2C_CALLS = []
HANDLERS = {}

class FakeMCU:
    def print_time_to_clock(self, t):
        return int(t * 1000000)
MCU = FakeMCU()

class FakeI2C:
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        I2C_CALLS.append((bytes(data), minclock, reqclock, retry))
    def get_mcu(self):
        return MCU

CURRENT_COLOR = [(0.0, 0.0, 0.0, 0.0)]

class FakeLEDHelper:
    def __init__(self, config, update_func, led_count=1):
        self.update_func = update_func
        self.color = CURRENT_COLOR
    def get_status(self, eventtime=None):
        return {'color_data': list(self.color)}

class FakePrinter:
    def register_event_handler(self, event, cb):
        HANDLERS.setdefault(event, []).append(cb)
PRINTER = FakePrinter()

class FakeConfig:
    def __init__(self, color_order):
        self.color_order = color_order
    def get_printer(self):
        return PRINTER
    def get(self, option, default=None):
        if option == 'color_order' and self.color_order is not None:
            return self.color_order
        return default
    def error(self, msg):
        raise Error(msg)

def mcu_i2c_from_config(config, default_addr=None):
    return FakeI2C()

bus_fake = types.ModuleType('extras.bus')
bus_fake.MCU_I2C_from_config = mcu_i2c_from_config
sys.modules['extras.bus'] = bus_fake
led_fake = types.ModuleType('extras.led')
led_fake.LEDHelper = FakeLEDHelper
sys.modules['extras.led'] = led_fake
import extras.pca9632

def make_pca(order, color):
    global CURRENT_COLOR
    CURRENT_COLOR = color
    return extras.pca9632.PCA9632(FakeConfig(order))

def connect():
    for cb in HANDLERS.get('klippy:connect', []):
        cb()
";

fn rec(function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.pca9632".to_string(),
        scenario: "live".to_string(),
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

fn i64_value(v: i64) -> Value {
    Value::Int(v)
}

fn u64_value(v: u64) -> Value {
    if v <= i64::MAX as u64 {
        Value::Int(v as i64)
    } else {
        Value::BigInt(v.to_string())
    }
}

fn calls_value(calls: &[I2cCall]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|c| {
                Value::Tuple(vec![
                    Value::Bytes(c.data.clone()),
                    i64_value(c.minclock),
                    u64_value(c.reqclock),
                    Value::Bool(c.retry),
                ])
            })
            .collect(),
    )
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.inner().to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(xs) => Value::List(xs.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(xs) => Value::Tuple(xs.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        // Fakes never produce host handles.
        PyValue::Host(_) => Value::Null,
    }
}

/// Compare an oracle exception with a Rust error (class + message).
fn assert_exc(py: &recording::Exception, err: &KlippyError) {
    assert_eq!(py.class_name, err.class_name(), "exception class");
    assert_eq!(py.message, err.message(), "exception message");
}

#[test]
fn module_factory_exposes_load_config_prefix() {
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.pca9632"), "entry points of extras.pca9632");
}

#[test]
fn connect_writes_mode_and_channel_registers() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 1.0, 0.5, 0.0)])").expect("construct");

    // Both sides registered exactly one klippy:connect handler.
    let py_events = py.eval("sorted(HANDLERS.keys())").expect("python events");
    let fix = fixture(None, vec![[0.0, 1.0, 0.5, 0.0]]);
    assert_equal(
        &rec("handlers"),
        "ret",
        &py_events,
        &Value::List(fix.printer.handler_events().iter().map(|e| Value::str(e)).collect()),
    )
    .expect("handler events");

    // Fire klippy:connect on both sides.
    py.exec("connect()").expect("python connect");
    fix.printer
        .send_event(EV_KLIPPY_CONNECT, &[])
        .expect("rust send_event");

    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("connect writes");
}

#[test]
fn default_color_order_matches_rgbw() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca(None, [(1.0, 1.0, 1.0, 1.0)])").expect("construct");
    py.exec("connect()").expect("python connect");

    // Unknown config option value (None) reads the "RGBW" default.
    let fix = fixture(None, vec![[1.0, 1.0, 1.0, 1.0]]);
    fix.printer
        .send_event(EV_KLIPPY_CONNECT, &[])
        .expect("rust send_event");
    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("default order writes");
}

#[test]
fn color_order_permutation_maps_channels() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('WGRB', [(1.0, 0.0, 0.5, 0.0)])").expect("construct");
    py.exec("connect()").expect("python connect");

    let fix = fixture(Some("WGRB"), vec![[1.0, 0.0, 0.5, 0.0]]);
    fix.printer
        .send_event(EV_KLIPPY_CONNECT, &[])
        .expect("rust send_event");
    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("permuted writes");
}

#[test]
fn update_leds_syncs_minclock_to_print_time() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 0.0, 0.0, 0.0)])").expect("construct");
    py.exec("connect()").expect("python connect");
    py.exec("I2C_CALLS.clear()").expect("clear");
    py.exec("pca.update_leds([(0.0, 0.0, 0.0, 1.0)], 1.25)")
        .expect("python update");

    let fix = fixture(None, vec![[0.0, 0.0, 0.0, 0.0]]);
    fix.printer
        .send_event(EV_KLIPPY_CONNECT, &[])
        .expect("rust send_event");
    fix.i2c.calls.borrow_mut().clear();
    fix.obj
        .update_leds(&[vec![0.0, 0.0, 0.0, 1.0]], Some(1.25))
        .expect("rust update");

    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("print-time writes");
}

#[test]
fn unchanged_registers_are_not_rewritten() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.25, 0.5, 0.75, 1.0)])").expect("construct");
    py.exec("connect()").expect("python connect");
    py.exec("I2C_CALLS.clear()").expect("clear");
    // Repeat the same colour: the cache suppresses every write.
    py.exec("pca.update_leds([(0.25, 0.5, 0.75, 1.0)], None)")
        .expect("python update");

    let fix = fixture(None, vec![[0.25, 0.5, 0.75, 1.0]]);
    fix.printer
        .send_event(EV_KLIPPY_CONNECT, &[])
        .expect("rust send_event");
    fix.i2c.calls.borrow_mut().clear();
    fix.obj
        .update_leds(&[vec![0.25, 0.5, 0.75, 1.0]], None)
        .expect("rust update");

    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("cached writes");
}

#[test]
fn reg_write_writes_once_per_value() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 0.0, 0.0, 0.0)])").expect("construct");
    py.exec("pca.reg_write(0x10, 7)").expect("python first write");
    py.exec("pca.reg_write(0x10, 7)").expect("python repeat");
    py.exec("pca.reg_write(0x10, 9)").expect("python changed");

    let fix = fixture(None, vec![[0.0, 0.0, 0.0, 0.0]]);
    fix.obj.reg_write(0x10, PyInt::from_i64(7), 0).expect("rust first");
    fix.obj.reg_write(0x10, PyInt::from_i64(7), 0).expect("rust repeat");
    fix.obj.reg_write(0x10, PyInt::from_i64(9), 0).expect("rust changed");

    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("reg_write calls");
}

#[test]
fn get_status_delegates_to_led_helper() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.1, 0.2, 0.3, 0.4)])").expect("construct");
    let py_status = py.eval("pca.get_status(0.0)").expect("python status");

    let fix = fixture(None, vec![[0.1, 0.2, 0.3, 0.4]]);
    let status = PrinterObject::get_status(&*fix.obj, 0.0).unwrap().expect("status present");
    assert_equal(
        &rec("get_status"),
        "ret",
        &py_status,
        &pyvalue_to_value(&status),
    )
    .expect("status dict");
}

#[test]
fn invalid_color_order_is_a_config_error() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");

    for order in ["RGB", "rgbw", "RRGB", "RGBWX", ""] {
        let expr = format!("make_pca({order:?}, [(0.0, 0.0, 0.0, 0.0)])");
        // OracleResult is Err(Exception) here; restructure to compare.
        let py_result = py.eval(&expr);
        let printer: Arc<FakePrinter> = Arc::new(FakePrinter::new());
        printer.add("gcode", Arc::new(FakeGCode) as ObjectRef);
        printer.add("gcode_macro", Arc::new(FakeMacro) as ObjectRef);
        printer.add("display_template", Arc::new(FakeEmptyDisplayTemplates) as ObjectRef);
        let printer_api: Printer = printer.clone();
        let config: Config = Arc::new(FakeConfig::new(
            printer_api.clone(),
            &[("color_order", order)],
        ));
        let mcu: Mcu = Arc::new(FakeMcu { printer: printer_api });
        let i2c: Arc<FakeI2c> =
            Arc::new(FakeI2c { mcu, calls: klippy_api::GCell::new(Vec::new()) });
        let rust = PCA9632::from_i2c(&config, i2c);
        match (py_result, rust) {
            (Err(exc), Err(err)) => assert_exc(&exc, &err),
            (Err(exc), Ok(_)) => panic!("rust accepted order {order:?}; python raised {exc:?}"),
            (Ok(v), Err(err)) => panic!("rust rejected order {order:?} ({err:?}); python returned {v:?}"),
            (Ok(v), Ok(_)) => panic!("both accepted order {order:?}; python returned {v:?}"),
        }
    }
}

#[test]
fn update_leds_empty_state_is_an_index_error() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 0.0, 0.0, 0.0)])").expect("construct");

    let py_result = py.eval("pca.update_leds([], None)");
    let fix = fixture(None, vec![[0.0, 0.0, 0.0, 0.0]]);
    let rust = fix.obj.update_leds(&[], None);
    match (py_result, rust) {
        (Err(exc), Err(err)) => assert_exc(&exc, &err),
        other => panic!("expected IndexError on both sides; got {other:?} vs {other:?}"),
    }
}

#[test]
fn short_color_row_is_an_index_error() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 0.0, 0.0, 0.0)])").expect("construct");

    let py_result = py.eval("pca.update_leds([(0.5, 0.5)], None)");
    let fix = fixture(None, vec![[0.0, 0.0, 0.0, 0.0]]);
    let rust = fix.obj.update_leds(&[vec![0.5, 0.5]], None);
    match (py_result, rust) {
        (Err(exc), Err(err)) => assert_exc(&exc, &err),
        other => panic!("expected IndexError on both sides; got {other:?} vs {other:?}"),
    }
}

#[test]
fn out_of_byte_range_pwm_is_a_value_error() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 0.0, 0.0, 0.0)])").expect("construct");

    // 2.0 does not clamp: int(2.0 * 255. + .5) == 510, which
    // bytearray([reg, val]) rejects inside i2c_write.
    let py_result = py.eval("pca.update_leds([(2.0, -1.0, 0.0, 0.0)], None)");
    let fix = fixture(None, vec![[0.0, 0.0, 0.0, 0.0]]);
    let rust = fix.obj.update_leds(&[vec![2.0, -1.0, 0.0, 0.0]], None);
    match (py_result, rust) {
        (Err(exc), Err(err)) => assert_exc(&exc, &err),
        other => panic!("expected ValueError on both sides; got {other:?} vs {other:?}"),
    }
    let py_calls = py.eval("I2C_CALLS").expect("python calls");
    assert_equal(
        &rec("i2c_calls"),
        "ret",
        &py_calls,
        &calls_value(&fix.i2c.calls.borrow()),
    )
    .expect("no writes before value error");
}

#[test]
fn infinite_and_nan_components_raise_like_int() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("pyoracle unavailable (KLIPPY_PYTHON unset); skipping");
        return;
    };
    py.exec(PY_SETUP).expect("fake setup");
    py.exec("I2C_CALLS.clear(); HANDLERS.clear()").expect("reset");
    py.exec("pca = make_pca('RGBW', [(0.0, 0.0, 0.0, 0.0)])").expect("construct");

    let cases = [
        ("float('inf')", f64::INFINITY),
        ("float('nan')", f64::NAN),
    ];
    for (expr_lit, value) in cases {
        let py_result = py
            .eval(&format!("pca.update_leds([({expr_lit}, 0.0, 0.0, 0.0)], None)"));
        let fix = fixture(None, vec![[0.0, 0.0, 0.0, 0.0]]);
        let rust = fix.obj.update_leds(&[vec![value, 0.0, 0.0, 0.0]], None);
        match (py_result, rust) {
            (Err(exc), Err(err)) => assert_exc(&exc, &err),
            _ => panic!("expected non-finite conversion error on both sides"),
        }
    }
}
