//! Parity tests for `extras::dotstar` (port of `klippy/extras/dotstar.py`).
//!
//! Every expected value comes live from the frozen reference through the
//! `pyoracle` crate (CPython 3.8.2 + the frozen `klippy/`); the tests
//! skip when `$KLIPPY_PYTHON` is unset. The oracle builds the real
//! `PrinterDotstar` with minimal Python fakes for the printer, config
//! and MCUs and runs the real `McuSpi` / `led.LEDHelper`; the port is
//! driven through fakes of the contract traits (`PrinterApi`,
//! `ConfigApi`, `McuApi`) plus the real `klippy_core::pins::PrinterPins`,
//! `klippy_core::gcode::GCodeDispatch` and the real `bus` / `led` ports.
//! MCU config commands, SPI sends, event handlers and `get_status`
//! output are compared end to end. The template evaluator the LED
//! helper pulls in is pre-registered in both fakes the way production
//! code pre-registers printer objects.

#![allow(dead_code, unused_imports, clippy::disallowed_types, clippy::new_ret_no_self)]

use std::collections::BTreeMap;
use std::sync::Arc;

use klippy_core::gcode::GCodeDispatch;
use klippy_core::pins::PrinterPins;
use extras::dotstar::{LedColor, PrinterDotstar};
use extras::output_pin::{lookup_template_eval, DisplayTemplate, DisplayTemplatesApi, GCodeMacroContextApi};
use extras_macros_templates::gcode_macro::Value as TemplateValue;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts, GetIntOpts,
    GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, Chip, ChipApi, CommandQueue, CommandQueryWrapper, CommandQueryWrapperApi,
    CommandWrapper, CommandWrapperApi, ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin,
    PinParams, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use kos::sync::Mutex;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use klippy_api::gcode::GCodeApi;
use indexmap::IndexMap;

// ---------------------------------------------------------------------------
// Python-side fakes (executed once per oracle process)
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import extras.dotstar as ds_mod
import extras.bus as bus_mod
import extras.led as led_mod
import extras.output_pin as op_mod
import pins as pins_mod
import configfile as cf_mod
from configparser import RawConfigParser

SENTINEL = object()

class FakeReactor:
    def mutex(self):
        import threading
        return threading.RLock()

class FakeCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append((self.fmt, bytes(data[1]).hex(), minclock, reqclock))

class FakeQueryCmd:
    def send(self, data, minclock=0, reqclock=0, retry=True):
        return None
    def send_with_preface(self, pre, pre_data, data, minclock=0, reqclock=0):
        return None

class FakeMCU:
    def __init__(self, name):
        self.name = name
        self.oid_count = 0
        self.config_cmds = []
        self.callbacks = []
        self.sends = []
        self.queries = []
        self.freq = 12000000.0
    def get_printer(self):
        return PRINTER
    def create_oid(self):
        self.oid_count += 1
        return self.oid_count - 1
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, is_init, on_restart))
    def alloc_command_queue(self):
        return ('cq', len(self.config_cmds))
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def lookup_command(self, fmt, cq=None):
        return FakeCmd(self, fmt)
    def lookup_query_command(self, fmt, respfmt, oid=None, cq=None, is_async=False):
        self.queries.append((fmt, respfmt, oid))
        return FakeQueryCmd()
    def try_lookup_command(self, fmt):
        return FakeCmd(self, fmt)
    def seconds_to_clock(self, t):
        return int(t * self.freq)
    def print_time_to_clock(self, pt):
        return int(pt * self.freq)
    def get_name(self):
        return self.name

class FakeGCode:
    def __init__(self):
        self.mux = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))

class FakeTemplateEval:
    def __init__(self):
        self.assigned = []
    def set_template(self, gcmd, cb, flush=None):
        self.assigned.append((gcmd.get_command(), cb, flush))

class FakePrinter:
    def __init__(self):
        self.objects = {}
        self.handlers = {}
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise Exception("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        if name in self.objects:
            raise Exception("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def get_reactor(self):
        return FakeReactor()
    def get_start_args(self):
        return {}
    def load_object(self, config, section, default=SENTINEL):
        if section in self.objects:
            return self.objects[section]
        raise Exception("load_object of '%s' is out of scope for this harness" % (section,))

globals()['PRINTER'] = None

def _make_config(ini):
    parser = RawConfigParser()
    parser.read_string(ini)
    tracking = {}
    return cf_mod.ConfigWrapper(PRINTER, parser, tracking, 'dotstar dled')

def build(ini):
    global PRINTER, MCU, D
    PRINTER = FakePrinter()
    gcode = FakeGCode()
    PRINTER.add_object('gcode', gcode)
    PRINTER.add_object('template_evaluator', FakeTemplateEval())
    ppins = pins_mod.PrinterPins()
    MCU = FakeMCU('mcu')
    ppins.chips['mcu'] = MCU
    ppins.chips['aux'] = FakeMCU('aux')
    PRINTER.add_object('pins', ppins)
    # Silence the template-evaluator construction path entirely.
    op_mod.lookup_template_eval = lambda config: PRINTER.lookup_object('template_evaluator')
    try:
        D = ds_mod.PrinterDotstar(_make_config(ini))
    except Exception as e:
        return ('raise', type(e).__name__, str(e))
    return ('ok', D.chain_count, sorted(PRINTER.handlers.keys()),
            list(MCU.config_cmds), list(MCU.queries), list(gcode.mux),
            D.get_status(1.0))

def fire():
    cbs = list(MCU.callbacks)
    del MCU.callbacks[:]
    for cb in cbs:
        cb()
    return (list(MCU.config_cmds), list(MCU.queries))

def sends():
    return list(MCU.sends)

def update(colors, pt):
    D.update_leds(colors, pt)

def connect():
    D.handle_connect()

def status():
    return D.get_status(1.0)

def update_ret(colors, pt):
    D.update_leds(colors, pt)
    return sends()

def connect_ret():
    D.handle_connect()
    return sends()
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(PY_SETUP).expect("dotstar fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping dotstar oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_str(s: &str) -> String {
    format!("{:?}", s)
}

fn ini_text(pairs: &[(&str, &str)]) -> String {
    let mut s = String::from("[dotstar dled]\n");
    for (k, v) in pairs {
        s.push_str(&format!("{} = {}\n", k, v));
    }
    s
}

fn ini_map(pairs: &[(&str, &str)]) -> BTreeMap<String, String> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

// ---------------------------------------------------------------------------
// Rust fakes
// ---------------------------------------------------------------------------

struct FakePrinterState {
    objects: IndexMap<String, ObjectRef>,
    handlers: BTreeMap<String, Vec<EventCallback>>,
    reactor: Reactor,
}

struct FakePrinter {
    state: Mutex<FakePrinterState>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            state: Mutex::new(FakePrinterState {
                objects: IndexMap::new(),
                handlers: BTreeMap::new(),
                reactor: Reactor::new(false),
            }),
        }
    }
    fn handler_names(&self) -> Vec<String> {
        self.state.lock().handlers.keys().cloned().collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.state.lock().reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut state = self.state.lock();
        if state.objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{}' already created",
                name
            )));
        }
        state.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let state = self.state.lock();
        if let Some(obj) = state.objects.get(name) {
            return Ok(Some(Arc::clone(obj)));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{}'",
                name
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let state = self.state.lock();
        match module {
            None => state
                .objects
                .iter()
                .map(|(k, v)| (k.clone(), Arc::clone(v)))
                .collect(),
            Some(module) => {
                let prefix = format!("{} ", module);
                let mut out = Vec::new();
                if let Some(obj) = state.objects.get(module) {
                    out.push((module.to_string(), Arc::clone(obj)));
                }
                for (k, v) in state.objects.iter() {
                    if k.starts_with(&prefix) {
                        out.push((k.clone(), Arc::clone(v)));
                    }
                }
                out
            }
        }
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let state = self.state.lock();
        if let Some(obj) = state.objects.get(section) {
            return Ok(Some(Arc::clone(obj)));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unable to load module '{}'",
                section
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.state
            .lock()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut state = self.state.lock();
        if let Some(cbs) = state.handlers.get_mut(event) {
            let mut out = Vec::new();
            for cb in cbs {
                out.push(cb(params)?);
            }
            return Ok(out);
        }
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeDisplayTemplates;

impl DisplayTemplatesApi for FakeDisplayTemplates {
    fn get_display_templates(&self) -> IndexMap<String, DisplayTemplate> {
        IndexMap::new()
    }
}

klippy_api::printer_object!(FakeDisplayTemplates; dyn DisplayTemplatesApi);

struct FakeMacro;

impl GCodeMacroContextApi for FakeMacro {
    fn create_template_context(&self, _eventtime: f64) -> IndexMap<String, TemplateValue> {
        IndexMap::new()
    }
}

klippy_api::printer_object!(FakeMacro; dyn GCodeMacroContextApi);

/// Fake config mirroring `configfile.ConfigWrapper` semantics and
/// messages (`klippy/configfile.py:29-60`).
struct FakeConfig {
    printer: Printer,
    name: String,
    options: BTreeMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, options: BTreeMap<String, String>) -> FakeConfig {
        FakeConfig {
            printer,
            name: name.to_string(),
            options,
        }
    }
}

fn missing(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{}' in section '{}' must be specified",
        option, section
    ))
}

fn unparseable(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Unable to parse option '{}' in section '{}'",
        option, section
    ))
}

fn check_bounds(
    option: &str,
    section: &str,
    min: Option<f64>,
    max: Option<f64>,
    above: Option<f64>,
    below: Option<f64>,
    value: f64,
) -> KResult<()> {
    if let Some(minval) = min {
        if value < minval {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must have minimum of {}",
                option, section, minval
            )));
        }
    }
    if let Some(maxval) = max {
        if value > maxval {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must have maximum of {}",
                option, section, maxval
            )));
        }
    }
    if let Some(above) = above {
        if value <= above {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must be above {}",
                option, section, above
            )));
        }
    }
    if let Some(below) = below {
        if value >= below {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must be below {}",
                option, section, below
            )));
        }
    }
    Ok(())
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(missing(option, &self.name)),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let value = match self.options.get(option) {
            Some(raw) => Some(raw.parse::<i64>().map_err(|_| unparseable(option, &self.name))?),
            None => None,
        };
        if let Some(v) = value {
            check_bounds(
                option,
                &self.name,
                opts.minval.map(|m| m as f64),
                opts.maxval.map(|m| m as f64),
                None,
                None,
                v as f64,
            )?;
        }
        if let Some(v) = value {
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(missing(option, &self.name)),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let value = match self.options.get(option) {
            Some(raw) => Some(raw.parse::<f64>().map_err(|_| unparseable(option, &self.name))?),
            None => None,
        };
        if let Some(v) = value {
            check_bounds(
                option, &self.name, opts.minval, opts.maxval, opts.above, opts.below, v,
            )?;
        }
        if let Some(v) = value {
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(missing(option, &self.name)),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let value = match self.options.get(option) {
            Some(raw) => Some(match raw.to_lowercase().as_str() {
                "yes" | "on" | "true" | "1" => true,
                "no" | "off" | "false" | "0" => false,
                _ => return Err(unparseable(option, &self.name)),
            }),
            None => None,
        };
        if let Some(v) = value {
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(missing(option, &self.name)),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: klippy_api::config::GetChoiceOpts,
    ) -> KResult<PyValue> {
        unimplemented!("getchoice is not used by dotstar/led/bus construction")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unimplemented!("getlists is not used by dotstar/led/bus construction")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unimplemented!("getlist is not used by dotstar/led/bus construction")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unimplemented!("getintlist is not used by dotstar/led/bus construction")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        unimplemented!("getfloatlist is not used by dotstar/led/bus construction")
    }
    fn getsection(&self, section: &str) -> Config {
        let config = FakeConfig {
            printer: Arc::clone(&self.printer),
            name: section.to_string(),
            options: BTreeMap::new(),
        };
        Arc::new(config)
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

// ---------------------------------------------------------------------------
// MCU fakes
// ---------------------------------------------------------------------------

#[derive(Default, Clone)]
struct McuLog {
    config_cmds: Vec<(String, bool, bool)>,
    sends: Vec<(String, Vec<u8>, i64, i64)>,
    queries: Vec<(String, String, Option<i64>)>,
}

struct FakeMcu {
    printer: Printer,
    name: String,
    freq: f64,
    oid: std::sync::Mutex<i64>,
    log: Arc<Mutex<McuLog>>,
    callbacks: Mutex<Vec<ConfigCallback>>,
}

fn trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = pycompat::text::trunc_float_to_bigint(f).0.to_string();
    digits.parse::<i64>().unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

impl FakeMcu {
    fn new(printer: Printer, name: &str) -> FakeMcu {
        FakeMcu {
            printer,
            name: name.to_string(),
            freq: 12_000_000.0,
            oid: std::sync::Mutex::new(0),
            log: Arc::new(Mutex::new(McuLog::default())),
            callbacks: Mutex::new(Vec::new()),
        }
    }
    fn log(&self) -> McuLog {
        self.log.lock().clone()
    }
    fn fire_callbacks(&self) {
        let callbacks = std::mem::take(&mut *self.callbacks.lock());
        for mut cb in callbacks {
            cb().expect("config callback");
        }
    }
}

struct FakeCmd {
    log: Arc<Mutex<McuLog>>,
    msgformat: String,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let bytes: Vec<u8> = match &data[1] {
            PyValue::Bytes(b) => b.clone(),
            PyValue::List(items) => items
                .iter()
                .map(|v| trunc_i64(v.to_f64().expect("byte value")) as u8)
                .collect(),
            other => panic!("unexpected spi data {other:?}"),
        };
        self.log
            .lock()
            .sends
            .push((self.msgformat.clone(), bytes, opts.minclock, opts.reqclock));
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeQueryCmd {
    msgformat: String,
    respformat: String,
}

impl CommandQueryWrapperApi for FakeQueryCmd {
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

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        unimplemented!("the fakes never set up pins")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut oid = self.oid.lock().unwrap();
        let out = *oid;
        *oid += 1;
        Ok(out)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.log
            .lock()
            .config_cmds
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
        0.1
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCmd {
            log: Arc::clone(&self.log),
            msgformat: msgformat.to_string(),
        }) as CommandWrapper)
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.log.lock().queries.push((
            msgformat.to_string(),
            respformat.to_string(),
            opts.oid,
        ));
        Ok(Arc::new(FakeQueryCmd {
            msgformat: msgformat.to_string(),
            respformat: respformat.to_string(),
        }) as CommandQueryWrapper)
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        Some(Arc::new(FakeCmd {
            log: Arc::clone(&self.log),
            msgformat: "".to_string(),
        }) as CommandWrapper)
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        unimplemented!("the fakes never register serial responses")
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
        unimplemented!("the fakes expose no constants")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        trunc_i64(print_time * self.freq)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / self.freq
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        unimplemented!("the fakes capture no clocks")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        unimplemented!("the fakes never create trigger dispatches")
    }
}

struct FakeChip {
    mcu: Mcu,
}

impl ChipApi for FakeChip {
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        unimplemented!("the fakes never set up pins")
    }
    fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.mcu)
    }
}

// ---------------------------------------------------------------------------
// World builder and comparison helpers
// ---------------------------------------------------------------------------

struct World {
    printer: Printer,
    printer_inner: Arc<FakePrinter>,
    gcode: Arc<GCodeDispatch>,
    mcu: Arc<FakeMcu>,
}

fn build_world(options: &BTreeMap<String, String>) -> World {
    let printer_inner = Arc::new(FakePrinter::new());
    let printer: Printer = printer_inner.clone();
    let mcu: Arc<FakeMcu> = Arc::new(FakeMcu::new(printer.clone(), "mcu"));
    let mcu2: Arc<FakeMcu> = Arc::new(FakeMcu::new(printer.clone(), "aux"));
    let pins = Arc::new(PrinterPins::new());
    pins
        .register_chip("mcu", Arc::new(FakeChip { mcu: mcu.clone() }) as Chip)
        .expect("register mcu chip");
    pins
        .register_chip("aux", Arc::new(FakeChip { mcu: mcu2.clone() }) as Chip)
        .expect("register aux chip");
    printer_inner
        .add_object("pins", pins as ObjectRef)
        .expect("register pins");
    let gcode = Arc::new(GCodeDispatch::new(printer.clone()));
    printer_inner
        .add_object("gcode", gcode.clone() as ObjectRef)
        .expect("register gcode");
    printer_inner
        .add_object("gcode_macro", Arc::new(FakeMacro) as ObjectRef)
        .expect("register gcode_macro");
    printer_inner
        .add_object("display_template", Arc::new(FakeDisplayTemplates) as ObjectRef)
        .expect("register display_template");
    let eval_config: Config = Arc::new(FakeConfig::new(
        printer.clone(),
        "template_evaluator",
        BTreeMap::new(),
    ));
    lookup_template_eval(&eval_config).expect("register template_evaluator");
    let _ = options;
    World {
        printer,
        printer_inner,
        gcode,
        mcu,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "dotstar".to_string(),
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

fn check_exception(label: &str, expected: pyoracle::OracleResult, err: &KlippyError) {
    let (class_name, message) = match expected {
        Ok(v) => panic!("{label}: expected exception, oracle returned {v}"),
        Err(e) => (e.class_name, e.message),
    };
    assert!(
        class_name == err.class_name() && message == err.message(),
        "{label}: expected {class_name}: {message}, got {}: {}",
        err.class_name(),
        err.message()
    );
}

fn sends_value(mcu: &McuLog) -> Value {
    let sends: Vec<Value> = mcu
        .sends
        .iter()
        .map(|(fmt, data, minclock, reqclock)| {
            let hex: String = data.iter().map(|b| format!("{:02x}", b)).collect();
            Value::Tuple(vec![
                Value::Str(fmt.clone()),
                Value::Str(hex),
                Value::Int(*minclock),
                Value::Int(*reqclock),
            ])
        })
        .collect();
    Value::List(sends)
}

fn config_cmds_value(mcu: &McuLog) -> Value {
    Value::List(
        mcu.config_cmds
            .iter()
            .map(|(cmd, is_init, on_restart)| {
                Value::Tuple(vec![
                    Value::Str(cmd.clone()),
                    Value::Bool(*is_init),
                    Value::Bool(*on_restart),
                ])
            })
            .collect(),
    )
}

fn queries_value(mcu: &McuLog) -> Value {
    Value::List(
        mcu.queries
            .iter()
            .map(|(fmt, respfmt, oid)| {
                Value::Tuple(vec![
                    Value::Str(fmt.clone()),
                    Value::Str(respfmt.clone()),
                    match oid {
                        Some(oid) => Value::Int(*oid),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

fn py_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(py_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(py_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (py_to_value(k), py_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "host".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn py_float(f: f64) -> String {
    format!("{f:?}")
}

fn leds_literal(colors: &[LedColor]) -> String {
    let items: Vec<String> = colors
        .iter()
        .map(|(r, g, b, w)| format!("({}, {}, {}, {})", py_float(*r), py_float(*g), py_float(*b), py_float(*w)))
        .collect();
    format!("[{}]", items.join(", "))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Construction: config reads, `chain_count`, event handler
/// registration, MCU config traffic and `get_status`, compared
/// against the oracle's build tuple.
#[test]
fn construction_parity() {
    let Some(mut py) = oracle() else { return };
    let cases: &[(&str, &[(&str, &str)])] = &[
        ("default", &[("data_pin", "PB5"), ("clock_pin", "PB6")]),
        (
            "chain4",
            &[
                ("data_pin", "PB5"),
                ("clock_pin", "PB6"),
                ("chain_count", "4"),
                ("initial_RED", "1.0"),
                ("initial_GREEN", "0.5"),
                ("initial_BLUE", "0.25"),
            ],
        ),
    ];
    for (label, pairs) in cases {
        let expected = py.eval(&format!("build({})", py_str(&ini_text(pairs))));
        let expected = match expected {
            Ok(Value::Tuple(fields)) => fields,
            other => panic!("{label}: oracle build shape: {other:?}"),
        };
        assert_eq!(expected[0], Value::Str("ok".to_string()));
        let world = build_world(&ini_map(pairs));
        let handlers_before = world.printer_inner.handler_names();
        let config: Config = Arc::new(FakeConfig::new(
            world.printer.clone(),
            "dotstar dled",
            ini_map(pairs),
        ));
        let obj = PrinterDotstar::new(&config).expect("rust construction");
        assert_eq!(expected[1], Value::Int(obj.chain_count));
        // Event handlers: dotstar registers only klippy:connect; the
        // harness GCodeDispatch registers ready/shutdown/disconnect, so
        // compare what construction added against the oracle build tuple.
        let added: Vec<String> = world
            .printer_inner
            .handler_names()
            .into_iter()
            .filter(|name| !handlers_before.contains(name))
            .collect();
        check(
            &format!("{label}/handlers"),
            Ok(expected[2].clone()),
            Value::List(added.into_iter().map(Value::Str).collect()),
        );
        // MCU traffic at construction: build tuple fields [3] config
        // commands and [4] query lookups, recorded by both sides.
        let mcu = world.mcu.log();
        check(
            &format!("{label}/config-cmds"),
            Ok(expected[3].clone()),
            config_cmds_value(&mcu),
        );
        check(
            &format!("{label}/queries"),
            Ok(expected[4].clone()),
            queries_value(&mcu),
        );
        // LED helper status: {'color_data': [(r, g, b, w), ...]}.
        let status = PrinterObject::get_status(&*obj, 1.0).expect("status present");
        check(
            &format!("{label}/status"),
            Ok(expected[6].clone()),
            py_to_value(&status),
        );
        // SET_LED / SET_LED_TEMPLATE mux commands are now registered
        // on the gcode dispatch; the LED commands are exercised by the
        // `led` module tests, so here only their presence matters.
        let help: Vec<String> = world
            .gcode
            .get_command_help()
            .iter()
            .map(|(cmd, _)| cmd.clone())
            .collect::<Vec<_>>();
        assert!(
            help.iter().any(|c| c.starts_with("SET_LED")),
            "{label}: SET_LED must be registered, got {help:?}"
        );
    }
}

/// Construction error paths: identical exception class and message.
#[test]
fn construction_errors() {
    let Some(mut py) = oracle() else { return };
    let cases: &[(&str, &[(&str, &str)])] = &[
        ("missing-data-pin", &[("clock_pin", "PB6")]),
        ("missing-clock-pin", &[("data_pin", "PB5")]),
        (
            "chain-count-min",
            &[("data_pin", "PB5"), ("clock_pin", "PB6"), ("chain_count", "0")],
        ),
        (
            "mcu-mismatch",
            &[("data_pin", "mcu:PB5"), ("clock_pin", "aux:PB6")],
        ),
    ];
    for (label, pairs) in cases {
        let expected = py.eval(&format!("build({})", py_str(&ini_text(pairs))));
        let world = build_world(&ini_map(pairs));
        let config: Config = Arc::new(FakeConfig::new(
            world.printer.clone(),
            "dotstar dled",
            ini_map(pairs),
        ));
        let err = PrinterDotstar::new(&config)
            .err()
            .unwrap_or_else(|| panic!("{label}: expected a construction error"));
        let expected = match expected {
            Ok(Value::Tuple(fields)) => fields,
            other => panic!("{label}: oracle build shape: {other:?}"),
        };
        let tag = match &expected[0] {
            Value::Str(tag) => tag.clone(),
            other => panic!("{label}: unexpected oracle shape {other:?}"),
        };
        assert_eq!(tag, "raise", "{label}: oracle must reach a construction error");
        let class = match &expected[1] {
            Value::Str(class) => class.clone(),
            other => panic!("{label}: unexpected oracle shape {other:?}"),
        };
        let message = match &expected[2] {
            Value::Str(message) => message.clone(),
            other => panic!("{label}: unexpected oracle shape {other:?}"),
        };
        assert!(
            err.class_name() == class.as_str() && err.message() == message.as_str(),
            "{label}: expected {class}: {message}, got {}: {}",
            err.class_name(),
            err.message()
        );
    }
}

/// Frame parity: `update_leds` for a spread of LED states, byte
/// for byte including framing, byte values, 20-byte chunking and
/// clocks. Runs after the MCU config callbacks ("finalize") have
/// been fired on both sides, so sends take the `lookup_command`
/// path with `minclock` / `reqclock`.
#[test]
fn update_leds_parity() {
    let Some(mut py) = oracle() else { return };
    let inis = ini_text(&[("data_pin", "PB5"), ("clock_pin", "PB6")]);
    let cases: &[(&str, &[LedColor], Option<f64>)] = &[
        ("one-black", &[(0.0, 0.0, 0.0, 0.0)], None),
        ("one-white", &[(1.0, 1.0, 1.0, 1.0)], None),
        (
            "two-mixed",
            &[(0.5, 0.25, 0.75, 0.1), (1.0, 0.0, 0.0, 0.0)],
            None,
        ),
        (
            "three-mixed-time",
            &[
                (1.0, 0.5, 0.25, 0.125),
                (0.0, 1.0, 0.5, 0.75),
                (0.3, 0.7, 0.2, 0.9),
            ],
            Some(123.5),
        ),
        (
            "five-chunked",
            &[
                (1.0, 0.5, 0.25, 0.125),
                (0.0, 1.0, 0.5, 0.75),
                (0.3, 0.7, 0.2, 0.9),
                (0.5, 0.5, 0.5, 0.5),
                (1.0, 1.0, 1.0, 1.0),
            ],
            Some(0.0),
        ),
    ];
    for (label, colors, print_time) in cases {
        let build = py.eval(&format!("build({})", py_str(&inis)));
        let built_ok = matches!(&build, Ok(Value::Tuple(f)) if f.first() == Some(&Value::Str("ok".to_string())));
        assert!(built_ok, "{label}: oracle build failed: {build:?}");
        let world = build_world(&ini_map(&[("data_pin", "PB5"), ("clock_pin", "PB6")]));
        let config: Config = Arc::new(FakeConfig::new(
            world.printer.clone(),
            "dotstar dled",
            ini_map(&[("data_pin", "PB5"), ("clock_pin", "PB6")]),
        ));
        let obj = PrinterDotstar::new(&config).expect("rust construction");
        py.eval("fire()").expect("oracle fire");
        world.mcu.fire_callbacks();
        // Config commands now match; query/command lookups are recorded.
        check(
            &format!("{label}/fire-config-cmds"),
            py.eval("MCU.config_cmds"),
            config_cmds_value(&world.mcu.log()),
        );
        let expected = py.eval(&format!(
            "update_ret({}, {})",
            leds_literal(colors),
            print_time.map(py_float).unwrap_or_else(|| "None".to_string())
        ));
        obj.update_leds(colors, *print_time).expect("rust update");
        check(
            &format!("{label}/sends"),
            expected,
            sends_value(&world.mcu.log()),
        );
    }
}

/// Dedup (identical state sends nothing), white-only changes
/// re-send, and `handle_connect` / status parity including the
/// `klippy:connect` event handler path.
#[test]
fn dedup_connect_status() {
    let Some(mut py) = oracle() else { return };
    let inis = ini_text(&[("data_pin", "PB5"), ("clock_pin", "PB6")]);
    py.eval(&format!("build({})", py_str(&inis))).expect("oracle build");
    let world = build_world(&ini_map(&[("data_pin", "PB5"), ("clock_pin", "PB6")]));
    let config: Config = Arc::new(FakeConfig::new(
        world.printer.clone(),
        "dotstar dled",
        ini_map(&[("data_pin", "PB5"), ("clock_pin", "PB6")]),
    ));
    let obj = PrinterDotstar::new(&config).expect("rust construction");
    py.eval("fire()").expect("oracle fire");
    world.mcu.fire_callbacks();

    let red = [(1.0, 0.0, 0.0, 0.0)];
    let expected = py
        .eval(&format!("update_ret({}, None)", leds_literal(&red)))
        .expect("oracle update once");
    obj.update_leds(&red, None).expect("rust update once");
    check("dedup/first", Ok(expected), sends_value(&world.mcu.log()));

    // Identical state: no further send on either side.
    let expected_count = py.eval("len(sends())").expect("oracle sends");
    py.eval(&format!("update({}, None)", leds_literal(&red)))
        .expect("oracle update twice");
    obj.update_leds(&red, None).expect("rust update twice");
    check(
        "dedup/count",
        Ok(expected_count),
        Value::Int(world.mcu.log().sends.len() as i64),
    );

    // White-only change: an unequal tuple re-sends identical bytes.
    let red_white = [(1.0, 0.0, 0.0, 1.0)];
    let expected = py
        .eval(&format!("update_ret({}, None)", leds_literal(&red_white)))
        .expect("oracle update white");
    obj.update_leds(&red_white, None).expect("rust update white");
    check(
        "white-change/sends",
        Ok(expected),
        sends_value(&world.mcu.log()),
    );

    // handle_connect sends the LED helper's current colors (113
    // state differs from the helper's initial state here).
    let expected = py.eval("connect_ret()").expect("oracle connect");
    obj.handle_connect().expect("rust connect");
    check("connect/sends", Ok(expected), sends_value(&world.mcu.log()));

    // The klippy:connect event handler runs the same code path.
    let expected = py.eval("PRINTER.handlers['klippy:connect'][0]()").expect("oracle event");
    let _ = expected;
    world
        .printer
        .send_event("klippy:connect", &[])
        .expect("rust event");
    check(
        "event/sends",
        py.eval("sends()"),
        sends_value(&world.mcu.log()),
    );
    check(
        "event/status",
        py.eval("status()"),
        py_to_value(&PrinterObject::get_status(&*obj, 1.0).expect("status present")),
    );
}

/// Non-finite colors raise `int()`'s exceptions verbatim.
#[test]
fn non_finite_colors_raise() {
    let Some(mut py) = oracle() else { return };
    let inis = ini_text(&[("data_pin", "PB5"), ("clock_pin", "PB6")]);
    py.eval(&format!("build({})", py_str(&inis))).expect("oracle build");
    py.eval("fire()").expect("oracle fire");
    let world = build_world(&ini_map(&[("data_pin", "PB5"), ("clock_pin", "PB6")]));
    let config: Config = Arc::new(FakeConfig::new(
        world.printer.clone(),
        "dotstar dled",
        ini_map(&[("data_pin", "PB5"), ("clock_pin", "PB6")]),
    ));
    let obj = PrinterDotstar::new(&config).expect("rust construction");
    world.mcu.fire_callbacks();
    for (label, literal, value) in [
        ("inf", "float('inf')", f64::INFINITY),
        ("nan", "float('nan')", f64::NAN),
    ] {
        let colors = [(value, 0.0, 0.0, 0.0)];
        let expected = py.eval(&format!(
            "update_ret([({}, 0.0, 0.0, 0.0)], None)",
            literal
        ));
        let err = obj
            .update_leds(&colors, None)
            .err()
            .unwrap_or_else(|| panic!("{label}: expected an update error"));
        check_exception(&format!("{label}/error"), expected, &err);
    }
}
