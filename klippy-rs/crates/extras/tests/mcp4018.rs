//! Parity tests for `extras::mcp4018` (port of `klippy/extras/mcp4018.py`).
//!
//! Expected values come live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`; the tests skip
//! with a message when `$KLIPPY_PYTHON` is unset). The Python side
//! builds a real `mcp4018` object on minimal fakes: `FakeBus`
//! stands in for `bus.MCU_I2C_from_config` (records `i2c_write`
//! data), a real `ConfigWrapper` over a real config text reads the
//! config (so scale/wiper defaults and error texts come from the
//! real `configfile.py`, not a fake), and `FakePrinter` /
//! `FakeGCode` record registrations; `GCodeCommand` is the real
//! one (so `get_float` failure texts come from the real
//! `gcode.py`). The Rust side drives the port through fakes of the contract traits (`ConfigApi`, `PrinterApi`, `GCodeApi`, `GCodeCommandApi`) and the `bus::McuI2cApi` fake with identical
//! inputs, and the traced effects are compared exactly: the config
//! results and raised errors, the registered event handler and mux
//! command (command, key, value, description), the I2C write data,
//! the `handle_connect` byte, and every `respond_info` line plus
//! the `get_float('WIPER', ...)` failure paths.
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply.

use extras::bus::McuI2cApi;
use extras::mcp4018::{Mcp4018, CMD_SET_DIGIPOT_HELP};
use klippy_api::mcu::{CommandQueue, Mcu, QuerySendOpts, SendOpts};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts as ConfigGetFloatOpts,
    GetIntOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GcmdGetFloatOpts,
    GetIntOpts as GcmdGetIntOpts, GetOpts, RegisterCommandOpts, GCmd, GCode,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, PrinterApi};
use klippy_api::GCell;
use pycompat::text::percent_format;
use pycompat::PctValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

fn oracle() -> Option<pyoracle::Oracle> {
    let Some(py) = pyoracle::Oracle::start() else {
        eprintln!("SKIP: $KLIPPY_PYTHON is unset; pyoracle unavailable");
        return None;
    };
    Some(py)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.mcp4018".to_string(),
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

/// Compare an oracle result with a Rust `KResult`: values go through
/// `assert_equal`, errors through exception class + message.
fn check_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    match (expected, actual) {
        (Ok(expected), Ok(actual)) => check(label, Ok(expected), actual),
        (Ok(v), Err(e)) => panic!(
            "{label}: oracle returned {}, port raised {}: {}",
            v.render(),
            e.class_name(),
            e.message()
        ),
        (Err(py_exc), Err(e)) => {
            assert_eq!(e.class_name(), py_exc.class_name, "{label}: exception class");
            assert_eq!(e.message(), py_exc.message, "{label}: exception message");
        }
        (Err(py_exc), Ok(actual)) => panic!(
            "{label}: oracle raised {}: {}, port returned {}",
            py_exc.class_name,
            py_exc.message,
            actual.render()
        ),
    }
}

fn pct_str(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant mcp4018 format")
}

// ---------------------------------------------------------------------------
// Rust-side fakes of the contract traits.
// ---------------------------------------------------------------------------

struct FakeConfig {
    section: String,
    printer: klippy_api::printer::Printer,
    values: GCell<IndexMap<String, String>>,
}

impl FakeConfig {
    fn new(
        printer: klippy_api::printer::Printer,
        section: &str,
        values: &[(&str, &str)],
    ) -> Arc<FakeConfig> {
        let mut map = IndexMap::new();
        for (k, v) in values {
            map.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeConfig {
            section: section.to_string(),
            printer,
            values: GCell::new(map),
        })
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> klippy_api::printer::Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Err(KlippyError::Config(
            "FakeConfig.get not exercised".to_string(),
        ))
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Err(KlippyError::Config(
            "FakeConfig.getint not exercised".to_string(),
        ))
    }
    /// Faithful copy of `ConfigWrapper.getfloat` /
    /// `ConfigWrapper._get_wrapper` (`klippy/configfile.py:29-72`):
    /// missing + required raises, parsed values are checked against
    /// minval/maxval/above/below with the exact configfile texts.
    fn getfloat(&self, option: &str, opts: ConfigGetFloatOpts) -> KResult<Option<f64>> {
        let section = self.section.clone();
        let raw = {
            let map = self.values.borrow();
            map.get(option).cloned()
        };
        let Some(raw) = raw else {
            return match &opts.default {
                None => Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must be specified",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                    ],
                ))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(*v)),
            };
        };
        let v = raw.trim().parse::<f64>().map_err(|_| {
            KlippyError::Config(pct_str(
                "Unable to parse option '%s' in section '%s'",
                &[
                    PctValue::Str(option.to_string()),
                    PctValue::Str(section.clone()),
                ],
            ))
        })?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must have minimum of %s",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                        PctValue::Float(minval),
                    ],
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must have maximum of %s",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                        PctValue::Float(maxval),
                    ],
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must be above %s",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                        PctValue::Float(above),
                    ],
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must be below %s",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                        PctValue::Float(below),
                    ],
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(KlippyError::Config(
            "FakeConfig.getboolean not exercised".to_string(),
        ))
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &pycompat::PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<pycompat::PyValue> {
        Err(KlippyError::Config(
            "FakeConfig.getchoice not exercised".to_string(),
        ))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<pycompat::PyValue> {
        Err(KlippyError::Config(
            "FakeConfig.getlists not exercised".to_string(),
        ))
    }
    fn getlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::Config(
            "FakeConfig.getlist not exercised".to_string(),
        ))
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::Config(
            "FakeConfig.getintlist not exercised".to_string(),
        ))
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::Config(
            "FakeConfig.getfloatlist not exercised".to_string(),
        ))
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("FakeConfig.getsection not exercised")
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
    fn deprecate(&self, _option: &str, _value: Option<pycompat::PyValue>) -> KResult<()> { Ok(()) }
}

struct FakePrinter {
    reactor: Reactor,
    handlers: GCell<Vec<(String, Box<dyn FnMut(&[EventArg]) -> KResult<pycompat::PyValue> + Send>)>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            handlers: GCell::new(Vec::new()),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> pycompat::PyValue {
        pycompat::PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Err(KlippyError::Config(format!("Unknown config object '{}'", name)))
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
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
    fn invoke_shutdown(&self, _msg: &str, _details: pycompat::PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: pycompat::PyValue) {}
    fn register_event_handler(
        &self,
        event: &str,
        callback: klippy_api::printer::EventCallback,
    ) {
        self.handlers
            .borrow_mut()
            .push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<pycompat::PyValue>> {
        let mut returns = Vec::new();
        let mut handlers = self.handlers.borrow_mut();
        for (ev, cb) in handlers.iter_mut() {
            if ev == event {
                returns.push(cb(params)?);
            }
        }
        Ok(returns)
    }
    fn request_exit(&self, _result: &str) {}
}

#[derive(Clone, Debug, PartialEq)]
struct MuxRecord {
    cmd: String,
    key: String,
    value: Option<String>,
    desc: Option<String>,
}

struct FakeGCode {
    mux: GCell<Option<(MuxRecord, GCodeCommandHandler)>>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new() -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            mux: GCell::new(None),
            reactor: Reactor::new(false),
        })
    }
}

impl klippy_api::gcode::GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
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
        *self.mux.borrow_mut() = Some((
            MuxRecord {
                cmd: cmd.to_string(),
                key: key.to_string(),
                value: value.map(str::to_string),
                desc: desc.map(str::to_string),
            },
            func,
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> pycompat::PyValue {
        pycompat::PyValue::None
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
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd::new(command, commandline, params.clone()))
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
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
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    responses: GCell<Vec<(String, bool)>>,
}

impl FakeGCmd {
    fn new(command: &str, commandline: &str, params: IndexMap<String, String>) -> FakeGCmd {
        FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params,
            responses: GCell::new(Vec::new()),
        }
    }

    fn with_params(pairs: &[(&str, &str)]) -> FakeGCmd {
        let mut params = IndexMap::new();
        for (k, v) in pairs {
            params.insert(k.to_string(), v.to_string());
        }
        FakeGCmd::new(
            "SET_DIGIPOT",
            "SET_DIGIPOT DIGIPOT=D0 WIPER=0",
            params,
        )
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GetOpts) -> KResult<pycompat::PyValue> {
        Err(KlippyError::Command(
            "FakeGCmd.get not exercised".to_string(),
        ))
    }
    fn get_int(&self, _name: &str, _opts: GcmdGetIntOpts) -> KResult<Option<i64>> {
        Err(KlippyError::Command(
            "FakeGCmd.get_int not exercised".to_string(),
        ))
    }
    /// Faithful copy of `GCodeCommand.get` / `get_float`
    /// (`klippy/gcode.py:64-96`) for the float parser.
    fn get_float(&self, name: &str, opts: GcmdGetFloatOpts) -> KResult<Option<f64>> {
        let raw = self.params.get(name).cloned();
        let Some(raw) = raw else {
            return match opts.default {
                None => Err(KlippyError::Command(pct_str(
                    "Error on '%s': missing %s",
                    &[
                        PctValue::Str(self.commandline.clone()),
                        PctValue::Str(name.to_string()),
                    ],
                ))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            };
        };
        let v = raw.trim().parse::<f64>().map_err(|_| {
            KlippyError::Command(pct_str(
                "Error on '%s': unable to parse %s",
                &[
                    PctValue::Str(self.commandline.clone()),
                    PctValue::Str(raw.clone()),
                ],
            ))
        })?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Command(pct_str(
                    "Error on '%s': %s must have minimum of %s",
                    &[
                        PctValue::Str(self.commandline.clone()),
                        PctValue::Str(name.to_string()),
                        PctValue::Float(minval),
                    ],
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Command(pct_str(
                    "Error on '%s': %s must have maximum of %s",
                    &[
                        PctValue::Str(self.commandline.clone()),
                        PctValue::Str(name.to_string()),
                        PctValue::Float(maxval),
                    ],
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Command(pct_str(
                    "Error on '%s': %s must be above %s",
                    &[
                        PctValue::Str(self.commandline.clone()),
                        PctValue::Str(name.to_string()),
                        PctValue::Float(above),
                    ],
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::Command(pct_str(
                    "Error on '%s': %s must be below %s",
                    &[
                        PctValue::Str(self.commandline.clone()),
                        PctValue::Str(name.to_string()),
                        PctValue::Float(below),
                    ],
                )));
            }
        }
        Ok(Some(v))
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
        String::new()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.responses.borrow_mut().push((msg.to_string(), log));
    }
    fn respond_raw(&self, msg: &str) {
        self.responses.borrow_mut().push((msg.to_string(), false));
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeI2c {
    writes: GCell<Vec<Vec<u8>>>,
    fail: GCell<Option<KlippyError>>,
}

impl FakeI2c {
    fn new() -> Arc<FakeI2c> {
        Arc::new(FakeI2c {
            writes: GCell::new(Vec::new()),
            fail: GCell::new(None),
        })
    }
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 { 1 }
    fn get_mcu(&self) -> Mcu { panic!("FakeI2c.get_mcu not exercised") }
    fn get_i2c_address(&self) -> i64 { 0x2f }
    fn get_command_queue(&self) -> CommandQueue { CommandQueue(0) }
    fn build_config(&self) -> KResult<()> { Ok(()) }
    fn i2c_write_noack(&self, _data: &[u8], _opts: SendOpts) -> KResult<()> {
        panic!("FakeI2c.i2c_write_noack not exercised")
    }
    fn i2c_write(&self, data: &[u8], _opts: QuerySendOpts) -> KResult<()> {
        if let Some(e) = self.fail.borrow().clone() {
            return Err(e);
        }
        self.writes.borrow_mut().push(data.to_vec());
        Ok(())
    }
    fn i2c_read(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<pycompat::PyValue> {
        panic!("FakeI2c.i2c_read not exercised")
    }
    fn i2c_transfer(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<pycompat::PyValue> {
        panic!("FakeI2c.i2c_transfer not exercised")
    }
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.mcp4018 as mcp4018
import configfile
import gcode as gcode_mod
from configparser import RawConfigParser

calls = []
fake_bus_i2c = None

class FakeI2C:
    def __init__(self):
        self.writes = []
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        self.writes.append(list(data))
        return None

class FakeGCode:
    def __init__(self): self.mux = None
    def register_mux_command(self, cmd, key, value, func, desc=None):
        calls.append(('mux', cmd, key, value, desc))
        self.mux = (cmd, key, value, func, desc)
class FakePrinter:
    def __init__(self, gcode): self._gcode = gcode; self._handlers = {}
    def lookup_object(self, name, default=None):
        if name == 'gcode':
            return self._gcode
        if default is None:
            raise Exception("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, cb):
        calls.append(('event', event))
        self._handlers.setdefault(event, []).append(cb)

def _fake_i2c_from_config(config, default_addr=None, **kwargs):
    return fake_bus_i2c

def build(config_text, section):
    global fake_bus_i2c, obj
    del calls[:]
    fake_bus_i2c = FakeI2C()
    fc = RawConfigParser()
    fc.optionxform = str.lower
    fc.read_string(config_text)
    gcode = FakeGCode()
    printer = FakePrinter(gcode)
    access = {}
    wrapper = configfile.ConfigWrapper(printer, fc, access, section)
    orig = mcp4018.bus.MCU_I2C_from_config
    mcp4018.bus.MCU_I2C_from_config = _fake_i2c_from_config
    try:
        obj = mcp4018.mcp4018(wrapper)
    finally:
        mcp4018.bus.MCU_I2C_from_config = orig
    return obj

class FakeGCodeDispatch:
    def __init__(self): self.messages = []
    def respond_info(self, msg, log=True): self.messages.append((msg, log))
    def respond_raw(self, msg): self.messages.append((repr(msg), 'raw'))

class _Sentinel: pass
SENT = _Sentinel()

class FakeGcmd:
    def __init__(self, command, commandline, params, need_ack=False):
        self.disp = FakeGCodeDispatch()
        self.inner = gcode_mod.GCodeCommand(self.disp, command, commandline, params, need_ack)
        self.messages = self.disp.messages
    def get_float(self, name, default=SENT, minval=None, maxval=None, above=None, below=None):
        if default is SENT:
            return self.inner.get_float(name, minval=minval, maxval=maxval, above=above, below=below)
        return self.inner.get_float(name, default, minval=minval, maxval=maxval, above=above, below=below)
    def get_commandline(self): return self.inner.get_commandline()
    def get_command(self): return self.inner.get_command()
    def respond_info(self, msg, log=True): return self.inner.respond_info(msg)
"#;

fn py_float(f: f64) -> String {
    if f.is_nan() {
        return "float('nan')".to_string();
    }
    if f.is_infinite() {
        return if f > 0.0 {
            "float('inf')".to_string()
        } else {
            "float('-inf')".to_string()
        };
    }
    let s = f.to_string();
    if s.contains('.') || s.contains('e') || s.contains('E') {
        s
    } else {
        format!("{s}.0")
    }
}

fn parse_kv(text: &str) -> Vec<(&str, &str)> {
    let mut out = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('[') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            out.push((k.trim(), v.trim()));
        }
    }
    out
}

struct Built {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    i2c: Arc<FakeI2c>,
    obj: Arc<Mcp4018>,
}

fn build_rust(name: &str, scale: f64, start_value: f64) -> Built {
    let printer: Arc<FakePrinter> = FakePrinter::new();
    let gcode: Arc<FakeGCode> = FakeGCode::new();
    let i2c: Arc<FakeI2c> = FakeI2c::new();
    let printer_api: klippy_api::printer::Printer = printer.clone();
    let gcode_api: GCode = gcode.clone();
    let obj = Mcp4018::new(
        &printer_api,
        &gcode_api,
        i2c.clone(),
        name.to_string(),
        scale,
        start_value,
    )
    .expect("Mcp4018::new");
    Built { printer, gcode, i2c, obj }
}

fn writes_value(writes: &[&[u8]]) -> Value {
    Value::List(
        writes
            .iter()
            .map(|w| Value::List(w.iter().map(|b| Value::int(*b as i64)).collect()))
            .collect(),
    )
}

fn pairs_map(pairs: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut params = IndexMap::new();
    for (k, v) in pairs {
        params.insert(k.to_string(), v.to_string());
    }
    params
}

fn pairs_line(pairs: &[(&str, &str)]) -> String {
    let text = pairs
        .iter()
        .map(|(k, v)| format!("{}={}", k, v))
        .collect::<Vec<_>>()
        .join(" ");
    format!("SET_DIGIPOT {}", text)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn config_read_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    // (config text, section) pairs covering: defaults, explicit
    // scale, small and fractional scales, boundary values, missing
    // required wiper, scale above-0 breach, wiper min/max breaches,
    // unparseable values, and name extraction from a wide-spaced
    // section.
    let cases: &[(&str, &str)] = &[
        ("[mcp4018 D0]\nwiper = 0.5\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 2.0\nwiper = 2.0\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 0.25\nwiper = 0\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 1.0\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 0\nwiper = 0\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = -1\nwiper = 0\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 2.0\nwiper = -0.1\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 2.0\nwiper = 2.1\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = abc\nwiper = 0\n", "mcp4018 D0"),
        ("[mcp4018 D0]\nscale = 2.0\nwiper = xyz\n", "mcp4018 D0"),
        (
            "[mcp4018  D1]\nscale = 3.5\nwiper = 1.25\n",
            "mcp4018  D1",
        ),
    ];
    for (i, (text, section)) in cases.iter().enumerate() {
        let label = format!("config[{i}]");
        let py_result = py.exec(&format!("obj = build({:?}, {:?})", text, section));
        let printer: klippy_api::printer::Printer = FakePrinter::new();
        let config: Config = FakeConfig::new(printer, section, &parse_kv(text));
        let rust_result: KResult<Value> = match Mcp4018::read_config(&config) {
            Ok(p) => Ok(Value::Tuple(vec![
                Value::float(p.scale),
                Value::float(p.wiper),
                Value::str(&p.name),
            ])),
            Err(e) => Err(e),
        };
        match (py_result, rust_result) {
            (Ok(_), Ok(actual)) => {
                let expected = py.eval("(obj.scale, obj.start_value, obj.name)").expect("py params");
                check(&label, Ok(expected), actual);
            }
            (Ok(_), Err(e)) => {
                panic!("{label}: port raised {}: {}", e.class_name(), e.message())
            }
            (Err(py_exc), Err(e)) => {
                assert_eq!(e.class_name(), py_exc.class_name, "{label}: error class");
                assert_eq!(e.message(), py_exc.message, "{label}: error message");
            }
            (Err(py_exc), Ok(actual)) => panic!(
                "{label}: oracle raised {}: {}, port returned {}",
                py_exc.class_name,
                py_exc.message,
                actual.render()
            ),
        }
    }
}

#[test]
fn registration_and_connect_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let text = "[mcp4018 D0]\nscale = 2.0\nwiper = 0.5\n";
    let expected = py.exec(&format!("obj = build({:?}, \"mcp4018 D0\")", text));
    assert!(expected.is_ok(), "py build must succeed: {:?}", expected);
    let py_trace = py
        .eval("({'calls': calls, 'desc': mcp4018.mcp4018.cmd_SET_DIGIPOT_help})")
        .expect("py trace");
    let built = build_rust("D0", 2.0, 0.5);
    let mux = built.gcode.mux.borrow().as_ref().expect("mux registered").0.clone();
    // The registry records the event handler and the mux command;
    // compare the command key, value and description plus the help
    // text against the oracle's record.
    let py_mux = py.eval("calls").expect("py calls");
    let rust_mux = Value::List(vec![
        Value::Tuple(vec![Value::str("event"), Value::str("klippy:connect")]),
        Value::Tuple(vec![
            Value::str("mux"),
            Value::str(&mux.cmd),
            Value::str(&mux.key),
            match &mux.value {
                Some(v) => Value::str(v),
                None => Value::Null,
            },
            match &mux.desc {
                Some(v) => Value::str(v),
                None => Value::Null,
            },
        ]),
    ]);
    check("registration calls", Ok(py_mux), rust_mux);
    assert_eq!(mux.desc.as_deref(), Some(CMD_SET_DIGIPOT_HELP));
    let py_desc = py
        .eval("mcp4018.mcp4018.cmd_SET_DIGIPOT_help")
        .expect("py desc");
    check("registration desc", Ok(py_desc), Value::str(CMD_SET_DIGIPOT_HELP));
    let _ = py_trace;
    // Firing klippy:connect writes the start value byte.
    built
        .printer
        .send_event("klippy:connect", &[])
        .expect("send event");
    let writes = built.i2c.writes.borrow().clone();
    assert_eq!(writes, vec![vec![32u8]], "connect byte for 0.5/2.0");
    py.exec("obj.handle_connect()").expect("py connect call");
    let py_connect = py.eval("obj.i2c.writes").expect("py connect");
    check(
        "connect writes",
        Ok(py_connect),
        writes_value(&writes.iter().map(|w| w.as_slice()).collect::<Vec<_>>()),
    );
}

#[test]
fn dac_byte_grid_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let scales = [0.25f64, 0.5, 1.0, 2.0, 3.5, 10.0, 100.0];
    let values = [
        0.0f64, 0.01, 0.1, 0.25, 0.3333, 0.5, 0.75, 0.999, 1.0, 1.5, 2.0, 3.5, 5.0, 7.77, 10.0,
        63.5, 100.0,
    ];
    for &scale in &scales {
        let built = build_rust("D0", scale, 0.0);
        py.exec(&format!(
            "obj = build(\"[mcp4018 D0]\\nscale = {}\\nwiper = 0\\n\", \"mcp4018 D0\")",
            py_float(scale)
        ))
        .expect("py build");
        for &v in &values {
            if v > scale {
                continue;
            }
            let label = format!("dac v={v} scale={scale}");
            py.exec(&format!("obj.set_dac({})", py_float(v)))
                .expect("py set_dac");
            let expected = py.eval("obj.i2c.writes[-1]");
            let actual = match built.obj.set_dac(v) {
                Ok(()) => {
                    let w = built.i2c.writes.borrow().clone();
                    Value::List(
                        w.last()
                            .unwrap()
                            .iter()
                            .map(|b| Value::int(*b as i64))
                            .collect(),
                    )
                }
                Err(e) => {
                    panic!("{label}: port raised {}: {}", e.class_name(), e.message())
                }
            };
            check(&label, expected, actual);
        }
        py.exec("obj.set_dac(obj.scale)").expect("py set_dac");
        let expected = py.eval("obj.i2c.writes[-1]");
        built.obj.set_dac(scale).expect("top");
        let w = built.i2c.writes.borrow().clone();
        check(
            &format!("dac top scale={scale}"),
            expected,
            Value::List(
                w.last()
                    .unwrap()
                    .iter()
                    .map(|b| Value::int(*b as i64))
                    .collect(),
            ),
        );
    }
}

#[test]
fn dac_byte_special_values_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let cases: &[(f64, f64)] = &[
        (0.5, 1.0),
        (127.0, 254.0),
        (1e-9, 1.0),
        (0.9999999, 1.0),
        (5.0, 3.0),
        (0.0, 1.0),
        (1.0, 3.0),
    ];
    for (i, (v, scale)) in cases.iter().enumerate() {
        let built = build_rust("D0", *scale, 0.0);
        py.exec(&format!(
            "obj = build(\"[mcp4018 D0]\\nscale = {}\\nwiper = 0\\n\", \"mcp4018 D0\")",
            py_float(*scale)
        ))
        .expect("py build");
        let label = format!("dac-special[{i}] v={v} scale={scale}");
        py.exec(&format!("obj.set_dac({})", py_float(*v)))
            .expect("py set_dac");
        let expected = py.eval("obj.i2c.writes[-1]");
        let actual = match built.obj.set_dac(*v) {
            Ok(()) => {
                let w = built.i2c.writes.borrow().clone();
                Value::List(
                    w.last()
                        .unwrap()
                        .iter()
                        .map(|b| Value::int(*b as i64))
                        .collect(),
                )
            }
            Err(e) => {
                panic!("{label}: port raised {}: {}", e.class_name(), e.message())
            }
        };
        check(&label, expected, actual);
    }
    // A "nan" wiper string parses as a float, so set_dac raises
    // the Python int(nan) ValueError on both sides.
    let built = build_rust("D0", 1.0, 0.0);
    py.exec("obj = build(\"[mcp4018 D0]\\nscale = 1.0\\nwiper = 0\\n\", \"mcp4018 D0\")")
        .expect("py build");
    let gcmd = FakeGCmd::with_params(&[("WIPER", "nan")]);
    let py_cmd = py.exec(
        "gcmd = FakeGcmd('SET_DIGIPOT', 'SET_DIGIPOT DIGIPOT=D0 WIPER=nan', {'WIPER': 'nan'})\nobj.cmd_SET_DIGIPOT(gcmd)",
    );
    let cmd: Arc<dyn GCodeCommandApi> = Arc::new(gcmd);
    let rust_cmd = built.obj.cmd_set_digipot(&cmd);
    check_result("dac nan", py_cmd, rust_cmd.map(|_| Value::Null));
    // An "inf" wiper is rejected by the get_float bound first
    // (inf > maxval), so no write and no response.
    let gcmd = FakeGCmd::new("SET_DIGIPOT", "SET_DIGIPOT DIGIPOT=D0 WIPER=inf", pairs_map(&[("WIPER", "inf")]));
    let py_cmd = py.exec(
        "gcmd = FakeGcmd('SET_DIGIPOT', 'SET_DIGIPOT DIGIPOT=D0 WIPER=inf', {'WIPER': 'inf'})\nobj.cmd_SET_DIGIPOT(gcmd)",
    );
    let cmd: Arc<dyn GCodeCommandApi> = Arc::new(gcmd);
    let rust_cmd = built.obj.cmd_set_digipot(&cmd);
    check_result("dac inf bounds", py_cmd, rust_cmd.map(|_| Value::Null));
    assert!(built.i2c.writes.borrow().is_empty());
}

#[test]
fn set_digipot_command_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let text = "[mcp4018 D0]\nscale = 2.0\nwiper = 0.5\n";
    let expected = py.exec(&format!("obj = build({:?}, \"mcp4018 D0\")", text));
    assert!(expected.is_ok(), "py build must succeed: {:?}", expected);
    let built = build_rust("D0", 2.0, 0.5);
    for wiper in ["0", "0.1", "0.5", "1", "1.25", "2"] {
        let py_trace = py.exec(&format!(
            "gcmd = FakeGcmd('SET_DIGIPOT', 'SET_DIGIPOT DIGIPOT=D0 WIPER={}', {{'WIPER': '{}'}})\nobj.cmd_SET_DIGIPOT(gcmd)",
            wiper, wiper
        ));
        let gcmd = Arc::new(FakeGCmd::with_params(&[
            ("DIGIPOT", "D0"),
            ("WIPER", wiper),
        ]));
        let cmd: Arc<dyn GCodeCommandApi> = gcmd.clone();
        let rust_err = built
            .obj
            .cmd_set_digipot(&cmd)
            .err()
            .map(|e| (e.class_name().to_string(), e.message().to_string()));
        match py_trace {
            Ok(_) => {
                assert!(rust_err.is_none(), "wiper {wiper}: port raised {rust_err:?}");
                let py_responses = py.eval("obj.i2c.writes").expect("py writes");
                let rust_writes = built.i2c.writes.borrow().clone();
                let rust_value = writes_value(
                    &rust_writes
                        .iter()
                        .map(|w| w.as_slice())
                        .collect::<Vec<_>>(),
                );
                check(
                    &format!("SET_DIGIPOT writes wiper={wiper}"),
                    Ok(py_responses),
                    rust_value,
                );
                let responses = gcmd.responses.borrow().clone();
                let expected_msg = pct_str(
                    "New value for DIGIPOT = %s, wiper = %.2f",
                    &[
                        PctValue::Str("D0".to_string()),
                        PctValue::Float(wiper.parse::<f64>().unwrap()),
                    ],
                );
                assert_eq!(
                    responses,
                    vec![(expected_msg, true)],
                    "wiper {wiper}: respond_info"
                );
            }
            Err(py_exc) => panic!(
                "wiper {wiper}: oracle raised {}: {}",
                py_exc.class_name, py_exc.message
            ),
        }
    }
    // The exact response text (including the `%.2f` spelling)
    // against the oracle.
    let wiper = "1.25";
    py.exec(&format!(
        "gcmd = FakeGcmd('SET_DIGIPOT', 'SET_DIGIPOT DIGIPOT=D0 WIPER={}', {{'WIPER': '{}'}})\nobj.cmd_SET_DIGIPOT(gcmd)",
        wiper, wiper
    ))
    .expect("py cmd");
    let py_msg = py.eval("gcmd.messages[-1][0]").expect("py msg");
    let gcmd = Arc::new(FakeGCmd::with_params(&[("WIPER", wiper)]));
    let cmd: Arc<dyn GCodeCommandApi> = gcmd.clone();
    built.obj.cmd_set_digipot(&cmd).expect("port cmd");
    let responses = gcmd.responses.borrow().clone();
    check("respond wiper=1.25", Ok(py_msg), Value::str(&responses[0].0));
}

#[test]
fn set_digipot_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let text = "[mcp4018 D0]\nscale = 2.0\nwiper = 0.5\n";
    let expected = py.exec(&format!("obj = build({:?}, \"mcp4018 D0\")", text));
    assert!(expected.is_ok(), "py build must succeed: {:?}", expected);
    let built = build_rust("D0", 2.0, 0.5);
    let cases: &[(&str, &[(&str, &str)])] = &[
        ("missing WIPER", &[("DIGIPOT", "D0")]),
        ("unparseable WIPER", &[("DIGIPOT", "D0"), ("WIPER", "xyz")]),
        ("below minimum", &[("DIGIPOT", "D0"), ("WIPER", "-0.1")]),
        ("above maximum", &[("DIGIPOT", "D0"), ("WIPER", "2.1")]),
    ];
    for (label, pairs) in cases {
        fn py_params(pairs: &[(&str, &str)]) -> String {
            pairs
                .iter()
                .map(|(k, v)| format!("'{k}': '{v}'"))
                .collect::<Vec<_>>()
                .join(", ")
        }
        let py_cmd = py.exec(&format!(
            "gcmd = FakeGcmd('SET_DIGIPOT', '{}', {{{}}})\nobj.cmd_SET_DIGIPOT(gcmd)",
            pairs_line(pairs),
            py_params(pairs)
        ));
        let gcmd = Arc::new(FakeGCmd::new("SET_DIGIPOT", &pairs_line(pairs), pairs_map(pairs)));
        let cmd: Arc<dyn GCodeCommandApi> = gcmd.clone();
        let rust_cmd = built.obj.cmd_set_digipot(&cmd).map(|_| Value::Null);
        check_result(label, py_cmd, rust_cmd);
        assert!(
            built.i2c.writes.borrow().is_empty(),
            "{label}: no write expected"
        );
    }
}

#[test]
fn i2c_failure_propagates() {
    // A failing `i2c_write` escapes `set_dac` unchanged on both sides.
    let msg = "MCU 'mcu' I2C request to addr 47 reports error BUSY";
    let mut py = pyoracle::Oracle::start().expect("oracle");
    py.exec(&format!(
        "import mcu, extras.mcp4018 as mcp4018\n\
         class _FailI2C:\n    def i2c_write(self, data, minclock=0, reqclock=0): raise mcu.error({msg:?})\n\
         _obj = object.__new__(mcp4018.mcp4018)\n_obj.scale = 1.0\n_obj.i2c = _FailI2C()"
    ))
    .expect("python mcp4018");
    let expected = py.eval("_obj.set_dac(0.5)").expect_err("python set_dac must raise");
    let built = build_rust("D0", 1.0, 0.0);
    built.i2c.fail.borrow_mut().replace(KlippyError::Mcu(msg.to_string()));
    let err = built.obj.set_dac(0.5).expect_err("set_dac must propagate");
    assert_eq!((err.class_name(), err.message()), (expected.class_name.as_str(), expected.message.as_str()));
}

#[test]
fn module_factory_has_prefix_entry() {
    let factory = extras::mcp4018::module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.mcp4018"), "entry points of extras.mcp4018");
}
