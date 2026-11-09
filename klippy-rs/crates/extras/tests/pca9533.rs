//! Parity tests for `klippy/extras/pca9533.py`.
//!
//! The frozen Python `extras.pca9533.PCA9533` is the reference. Expected
//! values come live from `pyoracle` (CPython 3.8.2 + the frozen `klippy/`
//! checkout): a reference object is built with minimal Python fakes for the
//! I2C device, the MCU and the `LEDHelper`, then driven through
//! `update_leds`, `handle_connect` and `get_status`. The Rust port is
//! driven through fakes of the same contracts with identical inputs and the
//! results are compared: the I2C writes (register byte, value, `minclock`,
//! `reqclock`), the `get_status` dict, and every raised error (Python class
//! name and `str(exc)` text, including `ValueError`/`OverflowError` for
//! `int(v * 4.0)` on NaN/inf, `IndexError` for the `rmap[...]` lookup with
//! Python negative wrap, and the unpack `ValueError`s). No recordings exist
//! for this unit, so oracle comparison is the proof.
//!
//! The Rust port is built through `PCA9533::from_parts` with a fake of
//! `bus.MCU_I2C` (same `i2c_write` calls with `QuerySendOpts`) and a real
//! `led.LEDHelper` (same setup as the `led` module tests: a fake printer
//! serving `gcode`, `gcode_macro` and `display_template` objects), then
//! driven through `update_leds`, `handle_connect` and `get_status`. The
//! results are compared: the I2C writes (register byte, value, `minclock`,
//! `reqclock`), the `get_status` dict, and every raised error (Python class
//! name and `str(exc)` text, including `ValueError`/`OverflowError` for
//! `int(v * 4.0)` on NaN/inf, `IndexError` for the `rmap[...]` lookup with
//! Python negative wrap, and the unpack `ValueError`s). No recordings exist
//! for this unit, so oracle comparison is the proof.
//!
//! `PCA9533::from_parts` registers a `klippy:connect` handler through
//! `PrinterApi::register_event_handler`; the test delivers that event
//! through the fake printer and checks the three writes
//! (`PWM0/85`, `PWM1/170`, `PLS0/ls0`). `load_config_prefix` builds the
//! real object through `bus::mcu_i2c_from_config(config, Some(98), 100000, false)`.

use extras::bus::McuI2cApi;
use extras::display::display::{DisplayGroup, DisplayTemplateApi, GlyphTable, PrinterDisplayTemplateApi};
use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras::pca9533::{
    PCA9533, PCA9533_PLS0, PCA9533_PWM0, PCA9533_PWM1, BACKGROUND_PRIORITY_CLOCK,
};
use extras_macros_templates::gcode_macro::Value;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListsOpts,
    GetListOpts, GetIntListOpts, GetFloatListOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, Chip, CommandQueue, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, SendOpts, QuerySendOpts,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use kos::sync::Mutex;
use pycompat::value::PyValue;
use pycompat::PyInt;
use reactor::Reactor;
use recording::value::py_float_repr;
use recording::Value as RValue;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[allow(clippy::type_complexity)]
struct FakePrinter {
    handlers: Mutex<Vec<(String, Box<dyn FnMut(&[EventArg]) -> KResult<PyValue> + Send>)>>,
    objs: Mutex<IndexMap<String, ObjectRef>>,
}

#[allow(clippy::box_collection)]
impl FakePrinter {
    fn new() -> Arc<Self> {
        let printer = Arc::new(FakePrinter {
            handlers: Mutex::new(Vec::new()),
            objs: Mutex::new(IndexMap::new()),
        });
        let ptr = Arc::clone(&printer);
        let mcu = Arc::new(FakeMcu { printer: ptr.clone() });
        let pins = FakePins::new();
        let gcode = Arc::new(FakeGCode {
            mux: Mutex::new(Vec::new()),
            handlers: Mutex::new(Vec::new()),
        });
        let macro_obj = Arc::new(FakeMacro { contexts: Mutex::new(Vec::new()) });
        let templates = Arc::new(FakeDisplayTemplates);
        let mut objs = printer.objs.lock();
        objs.insert("mcu".to_string(), mcu as ObjectRef);
        objs.insert("pins".to_string(), pins as ObjectRef);
        objs.insert("gcode".to_string(), gcode as ObjectRef);
        objs.insert("gcode_macro".to_string(), macro_obj as ObjectRef);
        objs.insert("display_template".to_string(), templates as ObjectRef);
        drop(objs);
        printer
    }
    /// The object a `lookup_object` call resolves to, if any.
    fn find(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objs.lock().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(klippy_api::error::KlippyError::Config(format!(
                    "Unknown config object '{}'",
                    name
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }

    fn send_event_inner(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut guard = self.handlers.lock();
        for (name, cb) in guard.iter_mut() {
            if name == event {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objs.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.find(name, default)
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.find(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(
        &self,
        event: &str,
        callback: klippy_api::printer::EventCallback,
    ) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.send_event_inner(event, params)
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfig {
    printer: Printer,
    initial: [f64; 4],
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        "pca9533 test".to_string()
    }
    fn get(&self, _option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(opts.default.and_then(|d| d))
    }
    fn getint(&self, _option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(opts.default.and_then(|d| d))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let value = match option {
            "initial_RED" => Some(self.initial[0]),
            "initial_GREEN" => Some(self.initial[1]),
            "initial_BLUE" => Some(self.initial[2]),
            "initial_WHITE" => Some(self.initial[3]),
            _ => None,
        };
        Ok(value.or(opts.default.and_then(|d| d)))
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
        Arc::new(FakeConfig { printer: self.get_printer(), initial: self.initial })
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

struct FakeMcu {
    printer: Printer,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<klippy_api::mcu::Pin> {
        Err(klippy_api::error::KlippyError::Config("setup_pin is unused by the pca9533 fakes".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        let v = time * 1000.0;
        if v.is_nan() {
            return 0;
        }
        if v.is_infinite() {
            return if v < 0.0 { i64::MIN } else { i64::MAX };
        }
        pycompat::text::trunc_float_to_bigint(v).0.to_string().parse::<i64>().unwrap_or(if v < 0.0 { i64::MIN } else { i64::MAX })
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
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        Err(klippy_api::error::KlippyError::Config("lookup_command is unused by the pca9533 fakes".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(klippy_api::error::KlippyError::Config("lookup_query_command is unused by the pca9533 fakes".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("register_serial_response is unused by the pca9533 fakes")
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
        Err(klippy_api::error::KlippyError::Config("get_constant_float is unused by the pca9533 fakes".to_string()))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        // int(print_time * 1000.0), matching the Python fake.
        let v = print_time * 1000.0;
        if v.is_nan() {
            return 0;
        }
        if v.is_infinite() {
            return if v < 0.0 { i64::MIN } else { i64::MAX };
        }
        pycompat::text::trunc_float_to_bigint(v).0.to_string().parse::<i64>().unwrap_or(if v < 0.0 { i64::MIN } else { i64::MAX })
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        0.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((1.0, 0.0))
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
        Err(klippy_api::error::KlippyError::Config(
            "create_trigger_dispatch is unused by the pca9533 fakes".to_string(),
        ))
    }
}

type Writes = Vec<(Vec<i64>, i64, i64)>;

struct FakeI2c {
    mcu: klippy_api::mcu::Mcu,
    writes: Mutex<Writes>,
    fail_with: Option<String>,
}

impl FakeI2c {
    fn new(mcu: klippy_api::mcu::Mcu) -> Arc<Self> {
        Arc::new(Self { mcu, writes: Mutex::new(Vec::new()), fail_with: None })
    }
    fn failing(mcu: klippy_api::mcu::Mcu) -> Arc<Self> {
        Arc::new(Self { mcu, writes: Mutex::new(Vec::new()), fail_with: Some("fake i2c nack".to_string()) })
    }
    fn snapshot(&self) -> Writes {
        self.writes.lock().clone()
    }
}

impl FakeI2c {
    fn record(&self, data: &[u8], minclock: i64, reqclock: i64) {
        self.writes.lock().push((
            data.iter().map(|v| *v as i64).collect(),
            minclock,
            reqclock,
        ));
    }
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 {
        1
    }
    fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.mcu)
    }
    fn get_i2c_address(&self) -> i64 {
        98
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.record(data, opts.minclock, opts.reqclock);
        Ok(())
    }
    fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        if let Some(msg) = &self.fail_with {
            return Err(klippy_api::error::KlippyError::Mcu(msg.clone()));
        }
        self.record(data, opts.minclock, opts.reqclock);
        Ok(())
    }
    fn i2c_read(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        Err(klippy_api::error::KlippyError::Config("i2c_read is unused by the pca9533 fakes".to_string()))
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Err(klippy_api::error::KlippyError::Config("i2c_transfer is unused by the pca9533 fakes".to_string()))
    }
}

struct FakeGCode {
    mux: Mutex<Vec<RValue>>,
    handlers: Mutex<Vec<(String, GCodeCommandHandler)>>,
}

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
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().push(RValue::Tuple(vec![
            RValue::str(cmd),
            RValue::str(key),
            match value {
                Some(v) => RValue::str(v),
                None => RValue::Null,
            },
            match desc {
                Some(v) => RValue::str(v),
                None => RValue::Null,
            },
        ]));
        self.handlers.lock().push((cmd.to_string(), func));
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
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("unused")
    }
    fn create_gcode_command(
        &self,
        _c: &str,
        _cl: &str,
        _p: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        panic!("unused")
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _g: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

struct FakeMacro {
    contexts: Mutex<Vec<f64>>,
}

impl PrinterGCodeMacroApi for FakeMacro {
    fn load_template(&self, _config: &Config, _option: &str, _default: Option<&str>) -> KResult<TemplateWrapper> {
        unreachable!("pca9533 never loads templates")
    }
    fn create_template_context(&self, eventtime: Option<f64>) -> IndexMap<String, Value> {
        let eventtime = eventtime.expect("eventtime");
        self.contexts.lock().push(eventtime);
        let mut out = IndexMap::new();
        out.insert("eventtime".to_string(), Value::py(PyValue::Float(eventtime)));
        out
    }
}

struct FakeDisplayTemplates;

impl PrinterDisplayTemplateApi for FakeDisplayTemplates {
    fn get_display_templates(
        &self,
    ) -> IndexMap<String, Arc<dyn DisplayTemplateApi>> {
        IndexMap::new()
    }
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        IndexMap::new()
    }
    fn get_display_glyphs(&self) -> GlyphTable {
        IndexMap::new()
    }
}

struct FakePins;

impl FakePins {
    fn new() -> Arc<FakePins> {
        Arc::new(FakePins)
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _d: &str, _o: ParsePinOpts) -> KResult<PinParams> {
        panic!("unused")
    }
    fn lookup_pin(&self, _d: &str, _o: LookupPinOpts) -> KResult<PinParams> {
        panic!("unused")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("unused")
    }
    fn reset_pin_sharing(&self, _p: PinParams) {}
    fn get_pin_resolver(&self, _c: &str) -> KResult<PinResolver> {
        panic!("unused")
    }
    fn register_chip(&self, _c: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _d: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);
klippy_api::printer_object!(FakePins; dyn PinsApi);
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakeMacro; dyn PrinterGCodeMacroApi);
klippy_api::printer_object!(FakeDisplayTemplates; dyn PrinterDisplayTemplateApi);

fn led_state(r: f64, g: f64, b: f64, w: f64) -> PyValue {
    PyValue::List(vec![PyValue::Tuple(vec![
        PyValue::Float(r),
        PyValue::Float(g),
        PyValue::Float(b),
        PyValue::Float(w),
    ])])
}

fn make_rs_pair(initial: [f64; 4]) -> (Arc<PCA9533>, Arc<FakeI2c>, Printer) {
    let printer: Printer = FakePrinter::new();
    let mcu: Mcu = Arc::new(FakeMcu { printer: Arc::clone(&printer) });
    let i2c = FakeI2c::new(mcu);
    let config: Config = Arc::new(FakeConfig { printer: Arc::clone(&printer), initial });
    let p = PCA9533::from_parts(&config, i2c.clone() as Arc<dyn McuI2cApi>).expect("PCA9533::from_parts");
    (p, i2c, printer)
}

// ---------------------------------------------------------------------------
// Python-side reference object.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.pca9533 as _pca_mod

class error(Exception):
    pass

class FakeMcu:
    def print_time_to_clock(self, t):
        return int(float(t) * 1000.0)

class FakeI2C:
    def __init__(self, mcu):
        self.mcu = mcu
        self.writes = []
        self.fail = False
    def get_mcu(self):
        return self.mcu
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        if self.fail:
            raise error("fake i2c nack")
        self.writes.append((list(data), minclock, reqclock))

class FakeLed:
    def __init__(self, color_data):
        self.color_data = color_data
    def get_status(self, eventtime=None):
        return {'color_data': self.color_data}

class FakePrinter:
    def register_event_handler(self, event, cb):
        pass

_mcu = FakeMcu()
_i2c = FakeI2C(_mcu)
_led = FakeLed([(0.0, 0.0, 0.0, 0.0)])
p = object.__new__(_pca_mod.PCA9533)
p.printer = FakePrinter()
p.i2c = _i2c
p.led_helper = _led
BACKGROUND = 0x7fffffff00000000
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    let r = py.exec(SETUP);
    match r {
        Ok(_) => Some(py),
        Err(e) => panic!("oracle setup failed: {}: {}", e.class_name, e.message),
    }
}

fn set_color_and_clear(py: &mut pyoracle::Oracle, expr: &str) {
    py.exec(&format!("p.i2c.writes.clear(); p.led_helper.color_data = {expr}"))
        .expect("set color data");
}

fn writes_value(writes: &Writes) -> RValue {
    RValue::List(
        writes
            .iter()
            .map(|(data, mc, rc)| {
                RValue::Tuple(vec![
                    RValue::List(data.iter().map(|v| RValue::Int(*v)).collect()),
                    RValue::Int(*mc),
                    RValue::Int(*rc),
                ])
            })
            .collect(),
    )
}


/// Compare an oracle `update_leds` call (return value + recorded writes)
/// with the port's outcome.
#[allow(clippy::too_many_arguments)]
fn check_update(
    py: &mut pyoracle::Oracle,
    label: &str,
    led_expr: &str,
    print_time: Option<f64>,
    rs_state: &PyValue,
    rs_print_time: Option<f64>,
    rs_i2c: &Arc<FakeI2c>,
    rs_p: &Arc<PCA9533>,
) {
    set_color_and_clear(py, led_expr);
    let pt_arg = print_time.map(py_float_repr).unwrap_or("None".to_string());
    let led_call = format!("p.update_leds({led_expr}, {pt_arg})");
    let expected_ret = py.eval(&led_call);
    let expected_writes = py.eval("p.i2c.writes");
    let rs_result = rs_p.update_leds(rs_state, rs_print_time);
    match (expected_ret, expected_writes, rs_result) {
        (Ok(RValue::Null), Ok(RValue::List(expected_items)), Ok(())) => {
            let record = recording::Record {
                v: 1,
                unit: "pca9533".to_string(),
                scenario: "live".to_string(),
                seq: 0,
                function: label.to_string(),
                args: Vec::new(),
                ret: None,
                exc: None,
                side_effects: recording::SideEffects::default(),
                object_id: None,
                kind: recording::RecordKind::Python,
            };
            recording::assert_equal(
                &record,
                "writes",
                &RValue::List(expected_items),
                &writes_value(&rs_i2c.snapshot()),
            )
            .unwrap_or_else(|d| panic!("{label}: divergence {d}"));
        }
        (Err(exc), Err(_), Ok(())) => panic!(
            "{label}: oracle raised {}: {}, port succeeded with writes {:?}",
            exc.class_name, exc.message, rs_i2c.snapshot()
        ),
        (Err(exc), _, Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception class");
            assert_eq!(e.message(), exc.message, "{label} exception message");
            assert!(
                rs_i2c.snapshot().is_empty(),
                "{label}: port emitted writes despite raising: {:?}",
                rs_i2c.snapshot()
            );
        }
        (Ok(r), _, Ok(())) => panic!("{label}: oracle returned {r:?}, expected null"),
        (Ok(r), _, _) => panic!("{label}: oracle returned {r:?} but port raised"),
        (Err(exc), _, _) => panic!("{label}: oracle raised {}: {}", exc.class_name, exc.message),
    }
}

#[test]
fn module_factory_shape() {
    let f = extras::pca9533::module_factory();
    assert!(f.load_config.is_none(), "pca9533 has no load_config");
    assert!(f.load_config_prefix.is_some(), "pca9533 must expose load_config_prefix");
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
    assert_eq!(BACKGROUND_PRIORITY_CLOCK, 0x7fffffff00000000);
    assert_eq!(PCA9533_PWM0, 2);
    assert_eq!(PCA9533_PWM1, 4);
    assert_eq!(PCA9533_PLS0, 5);
}

#[test]
fn load_config_prefix_builds_pca9533() {
    // The Python factory runs `bus.MCU_I2C_from_config(config, default_addr=98)`
    // and `led.LEDHelper(config, callback, 1)`; the Rust factory builds the
    // same real object over the fakes above.
    let printer: Printer = FakePrinter::new();
    let config: Config = Arc::new(FakeConfig { printer: Arc::clone(&printer), initial: [1.0, 0.0, 0.0, 0.0] });
    let res = extras::pca9533::load_config_prefix(&config);
    let obj = match res {
        Ok(Some(obj)) => obj,
        Ok(None) => panic!("load_config_prefix must build a PCA9533"),
        Err(e) => panic!("load_config_prefix must build a PCA9533: {}: {}", e.class_name(), e.message()),
    };
    assert!(klippy_api::downcast_arc::<PCA9533>(&obj).is_some());
    let factory = extras::pca9533::module_factory();
    let via_factory = factory.load_config_prefix.expect("factory exposes load_config_prefix")(&config);
    assert!(matches!(via_factory, Ok(Some(_))));
}

#[test]
fn update_black_matches_python() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 update oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    let (p, i2c, _printer) = make_rs_pair([0.0, 0.0, 0.0, 0.0]);
    let state = led_state(0.0, 0.0, 0.0, 0.0);
    let result = p.update_leds(&state, None);
    assert!(result.is_ok());
    assert_eq!(i2c.snapshot(), vec![(vec![PCA9533_PLS0, 0], 0, BACKGROUND_PRIORITY_CLOCK)]);
    // Cross-check against the oracle with the same input.
    set_color_and_clear(&mut py, "[(0.0, 0.0, 0.0, 0.0)]");
    let ret = py.eval("p.update_leds([(0.0, 0.0, 0.0, 0.0)], None)");
    assert!(matches!(ret, Ok(RValue::Null)));
    let writes = py.eval("p.i2c.writes");
    match writes {
        Ok(RValue::List(items)) => {
            assert_eq!(items.len(), 1);
            assert_eq!(items[0], RValue::Tuple(vec![RValue::List(vec![RValue::Int(PCA9533_PLS0), RValue::Int(0)]), RValue::Int(0), RValue::Int(BACKGROUND_PRIORITY_CLOCK)]));
        }
        other => panic!("oracle writes for black: {other:?}"),
    }
}

#[test]
fn update_grid_matches_python() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 grid oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    let cases: Vec<(&str, PyValue, Option<f64>)> = vec![
        ("white", led_state(1.0, 1.0, 1.0, 1.0), None),
        ("full-perceived", led_state(0.99, 0.01, 0.5, 0.25), Some(2.5)),
        ("halves", led_state(0.5, 0.25, 0.75, 1.0), None),
        ("near-boundaries", led_state(0.999, 0.001, 0.2, 0.8), Some(0.0)),
        ("negative-wrap-m1", led_state(-0.3, 0.0, 0.0, 0.0), None),
        ("negative-wrap-m2", led_state(-0.6, 0.0, 0.0, 0.0), None),
        ("negative-wrap-m4", led_state(-1.1, 0.0, 0.0, 0.0), None),
        ("negative-wrap-m5", led_state(-1.3, 0.0, 0.0, 0.0), None),
        ("negative-zero", led_state(-0.0, -0.0, 0.0, 0.0), None),
        ("exact-one", led_state(1.0, 0.0, 0.0, 0.0), None),
        ("trunc-quarter", led_state(0.25, 0.0, 0.0, 0.0), None),
        ("trunc-quarter-minus-eps", led_state(0.2499999, 0.0, 0.0, 0.0), None),
    ];
    for (label, state, pt) in cases {
        let expr = match &state {
            PyValue::List(l) => match &l[0] {
                PyValue::Tuple(t) => format!(
                    "[({}, {}, {}, {})]",
                    py_float_repr(t[0].to_f64().unwrap()),
                    py_float_repr(t[1].to_f64().unwrap()),
                    py_float_repr(t[2].to_f64().unwrap()),
                    py_float_repr(t[3].to_f64().unwrap()),
                ),
                _ => panic!("case {label}: bad state"),
            },
            _ => panic!("case {label}: bad state"),
        };
        let (p, i2c, _printer) = make_rs_pair([0.0, 0.0, 0.0, 0.0]);
        check_update(&mut py, label, &expr, pt, &state, pt, &i2c, &p);
    }
}

#[test]
fn update_error_paths_match_python() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 error oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    // (label, python led_state expr, rust state)
    let cases: Vec<(&str, &str, PyValue)> = vec![
        ("index-high", "[(1.25, 0.0, 0.0, 0.0)]", led_state(1.25, 0.0, 0.0, 0.0)),
        ("index-low", "[(-1.6, 0.0, 0.0, 0.0)]", led_state(-1.6, 0.0, 0.0, 0.0)),
        ("nan", "[(float('nan'), 0.0, 0.0, 0.0)]", led_state(f64::NAN, 0.0, 0.0, 0.0)),
        ("inf", "[(float('inf'), 0.0, 0.0, 0.0)]", led_state(f64::INFINITY, 0.0, 0.0, 0.0)),
        ("neg-inf", "[(float('-inf'), 0.0, 0.0, 0.0)]", led_state(f64::NEG_INFINITY, 0.0, 0.0, 0.0)),
        ("empty", "[]", PyValue::List(vec![])),
        ("short", "[(0.0, 0.0)]", PyValue::List(vec![PyValue::Tuple(vec![PyValue::Float(0.0), PyValue::Float(0.0)])])),
        ("long", "[(0.0, 0.0, 0.0, 0.0, 0.0)]", PyValue::List(vec![PyValue::Tuple(vec![PyValue::Float(0.0), PyValue::Float(0.0), PyValue::Float(0.0), PyValue::Float(0.0), PyValue::Float(0.0)])])),
        ("string-elem", "[('x', 0.0, 0.0, 0.0)]", PyValue::List(vec![PyValue::Tuple(vec![PyValue::Str("x".to_string()), PyValue::Float(0.0), PyValue::Float(0.0), PyValue::Float(0.0)])])),
    ];
    for (label, expr, state) in cases {
        let (p, i2c, _printer) = make_rs_pair([0.0, 0.0, 0.0, 0.0]);
        let res = p.update_leds(&state, None);
        set_color_and_clear(&mut py, expr);
        let expected = py.eval(&format!("p.update_leds({expr}, None)"));
        match (expected, res) {
            (Err(exc), Err(e)) => {
                assert_eq!(e.class_name(), exc.class_name, "{label} exception class");
                assert_eq!(e.message(), exc.message, "{label} exception message");
                assert!(i2c.snapshot().is_empty(), "{label}: port must not emit writes on error");
            }
            (Ok(_), Err(e)) => panic!("{label}: oracle succeeded but port raised {}: {}", e.class_name(), e.message()),
            (Err(exc), Ok(())) => panic!("{label}: oracle raised {}: {} but port succeeded", exc.class_name, exc.message),
            (Ok(r), _) => panic!("{label}: unexpected oracle result {r:?}"),
        }
    }
}

#[test]
fn int_channels_match_python() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 int-channel oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    let state = PyValue::List(vec![PyValue::Tuple(vec![
        PyValue::Int(PyInt::from_i64(1)),
        PyValue::Int(PyInt::from_i64(0)),
        PyValue::Int(PyInt::from_i64(0)),
        PyValue::Int(PyInt::from_i64(0)),
    ])]);
    let (p, i2c, _printer) = make_rs_pair([0.0, 0.0, 0.0, 0.0]);
    p.update_leds(&state, None).expect("int channels must succeed");
    // int(1 * 4.0) = 4 -> rmap[4] = 1, so red = 1, others 0.
    assert_eq!(i2c.snapshot(), vec![(vec![PCA9533_PLS0, 1], 0, BACKGROUND_PRIORITY_CLOCK)]);
    set_color_and_clear(&mut py, "[(1, 0, 0, 0)]");
    let ret = py.eval("p.update_leds([(1, 0, 0, 0)], None)");
    assert!(matches!(ret, Ok(RValue::Null)));
    let w = py.eval("p.i2c.writes");
    assert_eq!(
        w,
        Ok(RValue::List(vec![RValue::Tuple(vec![
            RValue::List(vec![RValue::Int(PCA9533_PLS0), RValue::Int(1)]),
            RValue::Int(0),
            RValue::Int(BACKGROUND_PRIORITY_CLOCK),
        ])]))
    );
}

#[test]
fn handle_connect_matches_python() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 connect oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    let cases: Vec<(&str, f64, f64, f64, f64)> = vec![
        ("connect-black", 0.0, 0.0, 0.0, 0.0),
        ("connect-color", 0.5, 0.25, 0.75, 1.0),
        ("connect-white", 1.0, 1.0, 1.0, 1.0),
    ];
    for (label, r, g, b, w) in cases {
        let (p, i2c, _printer) = make_rs_pair([r, g, b, w]);
        p.handle_connect().expect("handle_connect must succeed");
        let writes = i2c.snapshot();
        assert_eq!(writes.len(), 3, "{label}: handle_connect must emit exactly 3 writes");
        assert_eq!(writes[0], (vec![PCA9533_PWM0, 85], 0, 0), "{label} PWM0");
        assert_eq!(writes[1], (vec![PCA9533_PWM1, 170], 0, 0), "{label} PWM1");
        assert_eq!(writes[2].0[0], PCA9533_PLS0, "{label} PLS0 register");
        assert_eq!(writes[2].1, 0, "{label} PLS0 minclock");
        assert_eq!(writes[2].2, BACKGROUND_PRIORITY_CLOCK, "{label} PLS0 reqclock");
        // Oracle cross-check.
        py.exec(&format!(
            "p.i2c.writes.clear(); p.led_helper.color_data = [({}, {}, {}, {})]",
            py_float_repr(r), py_float_repr(g), py_float_repr(b), py_float_repr(w),
        ))
        .expect("set oracle color");
        let ret = py.eval("p.handle_connect()");
        assert!(matches!(ret, Ok(RValue::Null)), "{label}: {ret:?}");
        let ow = py.eval("p.i2c.writes");
        match ow {
            Ok(RValue::List(items)) => {
                assert_eq!(items.len(), 3, "{label} oracle write count");
            }
            other => panic!("{label}: oracle writes {other:?}"),
        }
    }
}

#[test]
fn get_status_delegates_to_led_helper() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 status oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    let (p, _i2c, _printer) = make_rs_pair([0.5, 0.25, 0.75, 1.0]);
    let status = p.get_status(123.5);
    // Rust returns the led helper dict verbatim.
    let mut d = indexmap::IndexMap::new();
    d.insert(PyValue::Str("color_data".to_string()), led_state(0.5, 0.25, 0.75, 1.0));
    assert_eq!(status, PyValue::Dict(d));
    py.exec("p.led_helper.color_data = [(0.5, 0.25, 0.75, 1.0)]").expect("set oracle color");
    let expected = py.eval("p.get_status(123.5)");
    match expected {
        Ok(RValue::Dict(pairs)) => {
            assert_eq!(pairs.len(), 1);
            assert_eq!(pairs[0].0, RValue::Str("color_data".to_string()));
        }
        other => panic!("get_status oracle: {other:?}"),
    }
}

#[test]
fn connect_event_triggers_pwm_writes() {
    let (p, i2c, printer) = make_rs_pair([0.0, 0.0, 0.0, 0.0]);
    printer.send_event("klippy:connect", &[]).expect("connect event must succeed");
    let writes = i2c.snapshot();
    assert_eq!(writes.len(), 3);
    assert_eq!(writes[0], (vec![PCA9533_PWM0, 85], 0, 0));
    assert_eq!(writes[1], (vec![PCA9533_PWM1, 170], 0, 0));
    assert_eq!(writes[2], (vec![PCA9533_PLS0, 0], 0, BACKGROUND_PRIORITY_CLOCK));
    drop(p);
}

#[test]
fn i2c_errors_propagate_from_python_and_port() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 i2c-error oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("p.i2c.fail = True").expect("enable oracle i2c failure");
    let expected = py.eval("p.update_leds([(0.0, 0.0, 0.0, 0.0)], None)");
    py.exec("p.i2c.fail = False").expect("disable oracle i2c failure");
    match expected {
        Err(e) => {
            assert_eq!(e.class_name, "error");
            assert_eq!(e.message, "fake i2c nack");
        }
        Ok(r) => panic!("oracle must raise the injected i2c error, got {r:?}"),
    }
    let printer: Printer = FakePrinter::new();
    let mcu: Mcu = Arc::new(FakeMcu { printer: Arc::clone(&printer) });
    let i2c = FakeI2c::failing(mcu);
    let config: Config = Arc::new(FakeConfig { printer: Arc::clone(&printer), initial: [0.0, 0.0, 0.0, 0.0] });
    let p = PCA9533::from_parts(&config, i2c.clone() as Arc<dyn McuI2cApi>).expect("PCA9533::from_parts");
    let res = p.update_leds(&led_state(0.0, 0.0, 0.0, 0.0), None);
    match res {
        Err(e) => {
            assert_eq!(e.class_name(), "error");
            assert_eq!(e.message(), "fake i2c nack");
            assert!(i2c.snapshot().is_empty());
        }
        Ok(()) => panic!("port must propagate the injected i2c error"),
    }
    // handle_connect fails on the first write without issuing the rest.
    let printer2: Printer = FakePrinter::new();
    let mcu2: Mcu = Arc::new(FakeMcu { printer: Arc::clone(&printer2) });
    let i2c2 = FakeI2c::failing(mcu2);
    let config2: Config = Arc::new(FakeConfig { printer: Arc::clone(&printer2), initial: [0.0, 0.0, 0.0, 0.0] });
    let p2 = PCA9533::from_parts(&config2, i2c2.clone() as Arc<dyn McuI2cApi>).expect("PCA9533::from_parts");
    let res2 = p2.handle_connect();
    assert!(res2.is_err());
    assert!(i2c2.snapshot().is_empty());
}

#[test]
fn print_time_maps_to_minclock() {
    let Some(mut py) = oracle() else {
        println!("skipping pca9533 minclock oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    set_color_and_clear(&mut py, "[(0.0, 0.0, 0.0, 0.0)]");
    let ret = py.eval("p.update_leds([(0.0, 0.0, 0.0, 0.0)], 2.5)");
    assert!(matches!(ret, Ok(RValue::Null)));
    let w = py.eval("p.i2c.writes");
    match w {
        Ok(RValue::List(items)) => {
            assert_eq!(items.len(), 1);
            // int(2.5 * 1000.0) = 2500.
            assert_eq!(
                items[0],
                RValue::Tuple(vec![
                    RValue::List(vec![RValue::Int(PCA9533_PLS0), RValue::Int(0)]),
                    RValue::Int(2500),
                    RValue::Int(BACKGROUND_PRIORITY_CLOCK),
                ])
            );
        }
        other => panic!("minclock oracle: {other:?}"),
    }
    let (p, i2c, _printer) = make_rs_pair([0.0, 0.0, 0.0, 0.0]);
    p.update_leds(&led_state(0.0, 0.0, 0.0, 0.0), Some(2.5)).expect("must succeed");
    assert_eq!(i2c.snapshot(), vec![(vec![PCA9533_PLS0, 0], 2500, BACKGROUND_PRIORITY_CLOCK)]);
}
