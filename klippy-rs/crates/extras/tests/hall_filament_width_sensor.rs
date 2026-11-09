//! Parity tests for `extras::hall_filament_width_sensor` (port of
//! `klippy/extras/hall_filament_width_sensor.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh
//! `HallFilamentWidthSensor` is built on both sides with minimal
//! Python fakes (printer, reactor, config, pins, adc pins, gcode,
//! toolhead), then driven through init, ADC callbacks, all seven
//! g-code commands, the extrude-factor timer, pending
//! disable/clear handling, `get_status`, and config error paths.
//! The Python side constructs the real `filament_switch_sensor`
//! `RunoutHelper`; the Rust side attaches the real helper port
//! (`extras::filament_switch_sensor::RunoutHelper`) built from the
//! same config. Message and error classes compare exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply; expected values come
//! from the oracle alone.

use extras::filament_switch_sensor::RunoutHelper;
use extras::gcode_macro::PrinterGCodeMacro;
use extras::hall_filament_width_sensor::{
    HallFilamentWidthSensor, ADC_REPORT_TIME, ADC_SAMPLE_COUNT, ADC_SAMPLE_TIME,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetIntOpts as GGetIntOpts, GCodeApi, GCodeCommandApi, GetOpts as GGetOpts, GCmd, OutputCallback,
    RegisterCommandOpts,
};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{
    AdcCallback, AdcPinApi, AdcSampleOpts, AsyncResponseWrapper, AddConfigCmdOpts, Chip,
    CommandQueue, CommandQueryWrapper, CommandWrapper, ConfigCallback, LookupQueryCommandOpts,
    Mcu, McuApi, Pin, PinParams, PinsApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::object::EventArg;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{Kinematics, PrinterExtruder, ToolHeadApi, ExtraAxis};
use klippy_api::ObjectRef;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;
use reactor::Completion;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.hall_filament_width_sensor as hall
import gcode as gcode_mod

class FakeReactor:
    NEVER = 9999999999999999.
    NOW = 0.
    def __init__(self):
        self.timers = []
        self.updates = []
        self.monotonic_time = 1000.0
    def register_timer(self, cb, waketime=NEVER):
        timer = {"cb": cb, "waketime": waketime}
        self.timers.append(timer)
        return timer
    def update_timer(self, timer, waketime):
        timer["waketime"] = waketime
        self.updates.append(waketime)
    def monotonic(self):
        return self.monotonic_time
    def register_callback(self, cb):
        pass

class FakePauseResume:
    def send_pause_command(self):
        pass

class FakeTemplate:
    def __init__(self, text):
        self._text = text
    def render(self, ctx):
        return self._text

class FakeGcodeMacro:
    def load_template(self, config, name, default=""):
        return FakeTemplate("")

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {}
        self.handlers = []
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        raise configparser.Error("Unknown config object '%s'" % (name,))
    def load_object(self, config, section, default=None):
        if section == "gcode_macro":
            return FakeGcodeMacro()
        if section == "pause_resume":
            return FakePauseResume()
        raise configparser.Error("Unable to load object '%s'" % (section,))
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def send_event(self, event, *params):
        for (name, cb) in self.handlers:
            if name == event:
                cb(*params)

class FakeConfig:
    error = configparser.Error
    SENTINEL = object()
    def __init__(self, printer, options, section="hall_filament_width_sensor"):
        self._printer = printer
        self._options = dict(options)
        self.section = section
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def _get(self, option, default, parser, minval=None, maxval=None, above=None, below=None):
        if option not in self._options:
            if default is FakeConfig.SENTINEL:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            return default
        try:
            v = parser(self._options[option])
        except:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise configparser.Error("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v
    def get(self, option, default=SENTINEL):
        return self._get(option, default, lambda v: v)
    def getint(self, option, default=SENTINEL, minval=None, maxval=None):
        return self._get(option, default, int, minval, maxval)
    def getfloat(self, option, default=SENTINEL, minval=None, maxval=None, above=None, below=None):
        return self._get(option, default, float, minval, maxval, above, below)
    def getboolean(self, option, default=SENTINEL):
        states = {'1': True, 'yes': True, 'true': True, 'on': True,
                  '0': False, 'no': False, 'false': False, 'off': False}
        def parser(v):
            lv = v.strip().lower()
            if lv not in states:
                raise ValueError("not a bool")
            return states[lv]
        return self._get(option, default, parser)

class FakeAdcPin:
    def __init__(self):
        self.sample_args = None
        self.callback = None
    def setup_adc_sample(self, report_time, sample_time=0., sample_count=1,
                         batch_num=1, minval=0., maxval=1., range_check_count=0):
        self.sample_args = (report_time, sample_time, sample_count, batch_num, minval, maxval, range_check_count)
    def setup_adc_callback(self, callback):
        self.callback = callback
    def get_last_value(self):
        return (0., 0.)

class FakePins:
    def __init__(self):
        self.adc_pins = []
        self.setup_calls = []
    def setup_pin(self, pin_type, pin_desc):
        self.setup_calls.append((pin_type, pin_desc))
        pin = FakeAdcPin()
        self.adc_pins.append(pin)
        return pin

class FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.registrations = []
        self.mux = []
        self.responses = []
        self.raw = []
        self.scripts = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registrations.append((cmd, desc, when_not_ready))
        self.handlers[cmd] = func
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))
    def respond_info(self, msg, log=True):
        self.responses.append((msg, log))
    def respond_raw(self, msg):
        self.raw.append(msg)
    def run_script(self, script):
        self.scripts.append(("run", script))
    def run_script_from_command(self, script):
        self.scripts.append(("fromcommand", script))
    def create_gcode_command(self, command, commandline, params):
        return gcode_mod.GCodeCommand(self, command, commandline, params, False)

class FakeToolhead:
    def __init__(self):
        self.pos = [0., 0., 0., 0.]
    def get_position(self):
        return list(self.pos)
"#;

// ---------------------------------------------------------------------------
// Shared configuration fixture.
// ---------------------------------------------------------------------------

/// The default printer.cfg section both fakes share.
fn default_options() -> Vec<(&'static str, &'static str)> {
    vec![
        ("adc1", "pin1"),
        ("adc2", "pin2"),
        ("Cal_dia1", "1.5"),
        ("Cal_dia2", "2.0"),
        ("Raw_dia1", "9500"),
        ("Raw_dia2", "10500"),
        ("measurement_interval", "10"),
        ("default_nominal_filament_diameter", "1.75"),
        ("measurement_delay", "2.0"),
        ("max_difference", "0.2"),
        ("enable", "true"),
        ("logging", "false"),
        ("enable_flow_compensation", "true"),
        ("use_current_dia_while_delay", "false"),
        ("pause_on_runout", "false"),
    ]
}

fn py_options_dict(rows: &[(&str, &str)]) -> String {
    let mut out = String::from("{");
    for (i, (k, v)) in rows.iter().enumerate() {
        if i > 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("'{k}': '{v}'"));
    }
    out.push('}');
    out
}

// ---------------------------------------------------------------------------
// Rust-side fakes of the contract traits.
// ---------------------------------------------------------------------------

/// `configfile` error messages, byte-identical to the Python fake.
fn cfg_err(msg: String) -> KlippyError {
    KlippyError::config(msg)
}

fn fmt_above(v: f64) -> String {
    // The Python call passes `above=1` (int) for the nominal diameter and
    // `above=0.` (float) for the delay; `%s` renders them "1" and "0.0".
    if v == 0.0 {
        return "0.0".to_string();
    }
    if v.is_finite() && v == v.trunc() && v.abs() < 1e16 {
        return format!("{}", v as i64);
    }
    pycompat::text::float_repr(v)
}

fn fmt_num_i64(v: i64) -> String {
    v.to_string()
}

fn fmt_num_f64(v: f64) -> String {
    pycompat::text::float_repr(v)
}

/// Python `int(str)` subset: strips whitespace, optional sign, digits with
/// single underscores between digits (CPython 3.6+ allows `1_0`).
fn py_int(text: &str) -> Option<i64> {
    let t = text.trim();
    let (sign, rest) = if let Some(s) = t.strip_prefix('-') {
        (-1i128, s)
    } else if let Some(s) = t.strip_prefix('+') {
        (1i128, s)
    } else {
        (1i128, t)
    };
    if rest.is_empty() {
        return None;
    }
    let mut value: i128 = 0;
    let mut prev_digit = false;
    for c in rest.chars() {
        if c == '_' {
            if !prev_digit {
                return None;
            }
            prev_digit = false;
        } else if c.is_ascii_digit() {
            value = value.checked_mul(10)?.checked_add((c as u8 - b'0') as i128)?;
            prev_digit = true;
        } else {
            return None;
        }
    }
    if !prev_digit {
        return None;
    }
    let v = value * sign;
    if v < i64::MIN as i128 || v > i64::MAX as i128 {
        return None;
    }
    Some(v as i64)
}

/// Python `float(str)` subset enough for config values: strips
/// whitespace, accepts inf/nan spellings and digit underscores.
fn py_float(text: &str) -> Option<f64> {
    let t = text.trim();
    let lower = t.to_ascii_lowercase();
    let (sign, rest) = if let Some(s) = lower.strip_prefix('-') {
        (-1.0f64, s)
    } else if let Some(s) = lower.strip_prefix('+') {
        (1.0, s)
    } else {
        (1.0, lower.as_str())
    };
    if rest == "inf" || rest == "infinity" {
        return Some(sign * f64::INFINITY);
    }
    if rest == "nan" {
        return Some(f64::NAN);
    }
    if rest.is_empty() {
        return None;
    }
    let mut prev_digit = false;
    let mut has_digit = false;
    let mut cleaned = String::new();
    for c in rest.chars() {
        if c == '_' {
            if !prev_digit {
                return None;
            }
            prev_digit = false;
        } else {
            if c.is_ascii_digit() {
                prev_digit = true;
                has_digit = true;
            }
            if c == '.' || c == 'e' || c == '+' || c == '-' {
                prev_digit = false;
            }
            cleaned.push(c);
        }
    }
    if !has_digit {
        return None;
    }
    let cleaned = format!("{}{}", if sign < 0.0 { "-" } else { "" }, cleaned);
    cleaned.parse::<f64>().ok()
}

fn py_bool(text: &str) -> Option<bool> {
    match text.trim().to_ascii_lowercase().as_str() {
        "1" | "yes" | "true" | "on" => Some(true),
        "0" | "no" | "false" | "off" => Some(false),
        _ => None,
    }
}

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
    section: String,
}

impl FakeConfig {
    fn opt(&self, option: &str) -> Option<&str> {
        self.options.get(option).map(|s| s.as_str())
    }

    fn missing(&self, option: &str) -> KlippyError {
        cfg_err(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
    }

    fn unparseable(&self, option: &str) -> KlippyError {
        cfg_err(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.section
        ))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opt(option) {
            Some(v) => Ok(Some(v.to_string())),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.opt(option) {
            Some(v) => {
                let parsed = py_int(v).ok_or_else(|| self.unparseable(option))?;
                if let Some(min) = opts.minval {
                    if parsed < min {
                        return Err(cfg_err(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            fmt_num_i64(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if parsed > max {
                        return Err(cfg_err(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            fmt_num_i64(max)
                        )));
                    }
                }
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.opt(option) {
            Some(v) => {
                let parsed = py_float(v).ok_or_else(|| self.unparseable(option))?;
                if let Some(min) = opts.minval {
                    if parsed < min {
                        return Err(cfg_err(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            fmt_num_f64(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if parsed > max {
                        return Err(cfg_err(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            fmt_num_f64(max)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if parsed <= above {
                        return Err(cfg_err(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            fmt_above(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if parsed >= below {
                        return Err(cfg_err(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            fmt_num_f64(below)
                        )));
                    }
                }
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.opt(option) {
            Some(v) => {
                let parsed = py_bool(v).ok_or_else(|| self.unparseable(option))?;
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("getchoice is not used by hall_filament_width_sensor")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by hall_filament_width_sensor")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by hall_filament_width_sensor")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by hall_filament_width_sensor")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by hall_filament_width_sensor")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by hall_filament_width_sensor")
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


/// One `register_mux_command` recording: (command, key, value, desc).
type MuxEntry = (String, String, String, Option<String>);
/// Shared state for the fake gcode dispatch and the fake commands it
/// creates (Python `GCodeCommand.respond_info` is bound to the
/// dispatch's `respond_info`, so both sides share one sink).
struct FakeGCodeState {
    handlers: GCell<IndexMap<String, klippy_api::gcode::GCodeCommandHandler>>,
    registrations: GCell<Vec<(String, Option<String>, bool)>>,
    mux: GCell<Vec<MuxEntry>>,
    responses: GCell<Vec<(String, bool)>>,
    raw: GCell<Vec<String>>,
    scripts: GCell<Vec<(String, String)>>,
}

struct FakeGCode {
    state: Arc<FakeGCodeState>,
}

impl FakeGCode {
    fn new() -> Self {
        FakeGCode {
            state: Arc::new(FakeGCodeState {
                handlers: GCell::new(IndexMap::new()),
                registrations: GCell::new(Vec::new()),
                mux: GCell::new(Vec::new()),
                responses: GCell::new(Vec::new()),
                raw: GCell::new(Vec::new()),
                scripts: GCell::new(Vec::new()),
            }),
        }
    }

    fn call_command(
        &self,
        cmd: &str,
        commandline: &str,
        params: &[(&str, &str)],
    ) -> KResult<()> {
        let handler = {
            let mut map = self.state.handlers.borrow_mut();
            map.shift_remove(cmd).expect("command registered")
        };
        let mut handler = handler;
        let mut indexmap = IndexMap::new();
        for (k, v) in params {
            indexmap.insert(k.to_string(), v.to_string());
        }
        let gcmd: Arc<dyn GCodeCommandApi> = Arc::new(FakeGCmd {
            state: self.state.clone(),
            command: cmd.to_string(),
            commandline: commandline.to_string(),
            params: indexmap,
        });
        let result = handler(&gcmd);
        self.state.handlers.borrow_mut().insert(cmd.to_string(), handler);
        result
    }
}

struct FakeGCmd {
    state: Arc<FakeGCodeState>,
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GGetOpts) -> KResult<PyValue> {
        panic!("get is not used by hall_filament_width_sensor")
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let value = self.params.get(name);
        match value {
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(v) => Ok(v),
            },
            Some(v) => {
                let parsed = py_int(v).ok_or_else(|| {
                    KlippyError::command(format!(
                        "Error on '{}': unable to parse {}",
                        self.commandline, v
                    ))
                })?;
                if let Some(min) = opts.minval {
                    if parsed < min {
                        return Err(KlippyError::command(format!(
                            "Error on '{}': {} must have minimum of {}",
                            self.commandline, name, min
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if parsed > max {
                        return Err(KlippyError::command(format!(
                            "Error on '{}': {} must have maximum of {}",
                            self.commandline, name, max
                        )));
                    }
                }
                Ok(Some(parsed))
            }
        }
    }
    fn get_float(&self, _name: &str, _opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        panic!("get_float is not used by hall_filament_width_sensor")
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
        panic!("get_raw_command_parameters is not used by hall_filament_width_sensor")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.state.responses.borrow_mut().push((msg.to_string(), log));
    }
    fn respond_raw(&self, msg: &str) {
        self.state.raw.borrow_mut().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        self.state
            .registrations
            .borrow_mut()
            .push((cmd.to_string(), opts.desc, opts.when_not_ready));
        if let Some(f) = func {
            self.state.handlers.borrow_mut().insert(cmd.to_string(), f);
        }
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: klippy_api::gcode::GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        let _ = func;
        self.state.mux.borrow_mut().push((
            cmd.to_string(),
            key.to_string(),
            value.map(|v| v.to_string()).unwrap_or_default(),
            desc.map(|d| d.to_string()),
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        self.state
            .registrations
            .borrow()
            .iter()
            .filter_map(|(cmd, desc, _)| desc.clone().map(|d| (cmd.clone(), d)))
            .collect()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("gcode get_status is not used by hall_filament_width_sensor")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("register_output_handler is not used by hall_filament_width_sensor")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by hall_filament_width_sensor")
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.state
            .scripts
            .borrow_mut()
            .push(("run".to_string(), script.to_string()));
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.state
            .scripts
            .borrow_mut()
            .push(("fromcommand".to_string(), script.to_string()));
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used by hall_filament_width_sensor")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            state: self.state.clone(),
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        }) as GCmd
    }
    fn respond_raw(&self, msg: &str) {
        self.state.raw.borrow_mut().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.state.responses.borrow_mut().push((msg.to_string(), log));
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        let upper = cmd.to_ascii_uppercase();
        let mut chars = upper.split_whitespace();
        if let Some(first) = chars.next() {
            if let Some(rest) = first.get(1..) {
                return first.chars().next().is_some_and(|c| c.is_ascii_uppercase())
                    && rest.chars().next().is_some_and(|c| c.is_ascii_digit())
                    && rest.chars().all(|c| c.is_ascii_digit());
            }
        }
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by hall_filament_width_sensor")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by hall_filament_width_sensor")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "fake".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by hall_filament_width_sensor")
    }
    fn is_fileoutput(&self) -> bool {
        panic!("is_fileoutput is not used by hall_filament_width_sensor")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by hall_filament_width_sensor")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by hall_filament_width_sensor")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by hall_filament_width_sensor")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by hall_filament_width_sensor")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by hall_filament_width_sensor")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by hall_filament_width_sensor")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by hall_filament_width_sensor")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by hall_filament_width_sensor")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by hall_filament_width_sensor")
    }
    fn lookup_command(&self, _msg: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by hall_filament_width_sensor")
    }
    fn lookup_query_command(
        &self,
        _msg: &str,
        _resp: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by hall_filament_width_sensor")
    }
    fn try_lookup_command(&self, _msg: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by hall_filament_width_sensor")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by hall_filament_width_sensor")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by hall_filament_width_sensor")
    }
    fn check_valid_response(&self, _msg: &str) -> bool {
        panic!("check_valid_response is not used by hall_filament_width_sensor")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("get_enumerations is not used by hall_filament_width_sensor")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("get_constants is not used by hall_filament_width_sensor")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by hall_filament_width_sensor")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by hall_filament_width_sensor")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by hall_filament_width_sensor")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by hall_filament_width_sensor")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by hall_filament_width_sensor")
    }
    fn calibrate_clock(&self, _pt: f64, _et: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by hall_filament_width_sensor")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("mcu get_status is not used by hall_filament_width_sensor")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("mcu stats is not used by hall_filament_width_sensor")
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by hall_filament_width_sensor")
    }
}

struct FakeAdcPin {
    samples: GCell<Vec<(f64, AdcSampleOpts)>>,
    callbacks: GCell<Vec<AdcCallback>>,
    last_value: (f64, f64),
}

impl AdcPinApi for FakeAdcPin {
    fn get_mcu(&self) -> Mcu {
        Arc::new(FakeMcu) as Arc<dyn McuApi>
    }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.samples.borrow_mut().push((report_time, opts));
    }
    fn setup_adc_callback(&self, callback: AdcCallback) {
        self.callbacks.borrow_mut().push(callback);
    }
    fn get_last_value(&self) -> (f64, f64) {
        self.last_value
    }
}

struct FakePins {
    setup_calls: GCell<Vec<(String, String)>>,
    pins: GCell<Vec<Arc<FakeAdcPin>>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by hall_filament_width_sensor")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used by hall_filament_width_sensor")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setup_calls
            .borrow_mut()
            .push((pin_type.to_string(), pin_desc.to_string()));
        let pin = Arc::new(FakeAdcPin {
            samples: GCell::new(Vec::new()),
            callbacks: GCell::new(Vec::new()),
            last_value: (0.0, 0.0),
        });
        self.pins.borrow_mut().push(pin);
        let pin: Arc<FakeAdcPin> = self.pins.borrow().last().cloned().unwrap();
        Ok(Pin::Adc(pin as Arc<dyn AdcPinApi>))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by hall_filament_width_sensor")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by hall_filament_width_sensor")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used by hall_filament_width_sensor")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by hall_filament_width_sensor")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakePrinter {
    objects: IndexMap<String, ObjectRef>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new(pins: Arc<FakePins>, gcode: Arc<FakeGCode>, toolhead: Arc<FakeToolhead>) -> Self {
        let mut objects = IndexMap::new();
        objects.insert("pins".to_string(), pins as ObjectRef);
        objects.insert("gcode".to_string(), gcode as ObjectRef);
        objects.insert("toolhead".to_string(), toolhead as ObjectRef);
        FakePrinter {
            objects,
            handlers: GCell::new(Vec::new()),
            reactor: Reactor::new(false),
        }
    }

    fn fire_event(&self, event: &str, args: &[EventArg]) {
        let mut handlers = Vec::new();
        std::mem::swap(&mut *self.handlers.borrow_mut(), &mut handlers);
        let mut any = false;
        for (name, mut cb) in handlers {
            if name == event {
                any = true;
                let _ = cb(args).expect("event handler");
            }
            self.handlers.borrow_mut().push((name, cb));
        }
        if !any {
            panic!("no handler registered for {event}");
        }
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
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, _obj: ObjectRef) -> KResult<()> {
        let _ = name;
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(
        &self,
        config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match section {
            "gcode_macro" => Ok(Some(
                Arc::new(PrinterGCodeMacro::new(config)) as ObjectRef,
            )),
            _ => panic!("load_object('{section}') is not used by hall_filament_width_sensor"),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers
            .borrow_mut()
            .push((event.to_string(), callback));
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

// ---------------------------------------------------------------------------
// Fixture contexts (one on each side, built from the same options).
// ---------------------------------------------------------------------------

struct RustCtx {
    sensor: Arc<HallFilamentWidthSensor>,
    gcode: Arc<FakeGCode>,
    pins: Arc<FakePins>,
    toolhead: Arc<FakeToolhead>,
    helper: Arc<RunoutHelper>,
    printer: Arc<FakePrinter>,
}

fn rust_ctx(options: &[(&str, &str)]) -> RustCtx {
    let pins = Arc::new(FakePins {
        setup_calls: GCell::new(Vec::new()),
        pins: GCell::new(Vec::new()),
    });
    let gcode = Arc::new(FakeGCode::new());
    let toolhead = Arc::new(FakeToolhead {
        position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
    });
    let printer = Arc::new(FakePrinter::new(
        pins.clone(),
        gcode.clone(),
        toolhead.clone(),
    ));
    let mut map = IndexMap::new();
    for (k, v) in options {
        map.insert(k.to_string(), v.to_string());
    }
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        options: map,
        section: "hall_filament_width_sensor".to_string(),
    });
    let sensor = HallFilamentWidthSensor::new(&config).expect("rust init");
    let helper = Arc::new(RunoutHelper::new(&config).expect("rust helper"));
    sensor.set_runout_helper(helper.clone());
    RustCtx {
        sensor,
        gcode,
        pins,
        toolhead,
        helper,
        printer,
    }
}

fn py_ctx(py: &mut pyoracle::Oracle, options: &[(&str, &str)]) {
    let setup = format!(
        "p = FakePrinter()\ng = FakeGCode()\npins = FakePins()\nth = FakeToolhead()\np.objects = {{'gcode': g, 'pins': pins, 'toolhead': th}}\nc = FakeConfig(p, {})\ncb = hall.load_config(c)",
        py_options_dict(options)
    );
    py.exec(&setup).expect("py init");
}

/// The fake gcode command call (Rust side).
fn run_cmd(ctx: &RustCtx, cmd: &str, commandline: &str, params: &[(&str, &str)]) -> KResult<()> {
    ctx.gcode.call_command(cmd, commandline, params)
}

/// The fake gcode command call (Python side): constructs the real
/// `GCodeCommand`, then dispatches through the recorded handler.
fn py_cmd(py: &mut pyoracle::Oracle, cmd: &str, commandline: &str, params: &[(&str, &str)]) {
    let dict = py_options_dict(params);
    let expr = format!(
        "g.handlers['{cmd}'](g.create_gcode_command('{cmd}', '{commandline}', {dict}))",
        cmd = cmd,
        commandline = commandline,
        dict = dict
    );
    let _ = py.eval(&expr);
}

/// ADC callback invocation (Rust side: takes the callback the pins fake
/// stored, calls it with the samples, puts it back).
fn adc(ctx: &RustCtx, which: usize, samples: Vec<(f64, f64)>) {
    let pins = ctx.pins.pins.borrow();
    let pin: Arc<FakeAdcPin> = pins[which].clone();
    drop(pins);
    let mut cb = pin.callbacks.borrow_mut().remove(0);
    cb(samples);
    pin.callbacks.borrow_mut().insert(0, cb);
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("hall fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping hall_filament_width_sensor oracle tests: set KLIPPY_PYTHON \
                 to a CPython 3.8.2 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "hall_filament_width_sensor".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: Result<(), KlippyError>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust returned Ok but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

/// Python accumulated `responses` (list of `(msg, log)` tuples) vs the
/// Rust fake gcode response sink.
fn check_responses(label: &str, py: &mut pyoracle::Oracle, ctx: &RustCtx) {
    let expected = py.eval("g.responses");
    let actual: Vec<Value> = ctx
        .gcode
        .state
        .responses
        .borrow()
        .iter()
        .map(|(msg, log)| Value::Tuple(vec![Value::str(msg), Value::Bool(*log)]))
        .collect();
    check(label, expected, Value::List(actual));
}

/// Python accumulated `scripts` vs the Rust fake gcode script sink.
fn check_scripts(label: &str, py: &mut pyoracle::Oracle, ctx: &RustCtx) {
    let expected = py.eval("g.scripts");
    let actual: Vec<Value> = ctx
        .gcode
        .state
        .scripts
        .borrow()
        .iter()
        .map(|(kind, script)| Value::Tuple(vec![Value::str(kind), Value::str(script)]))
        .collect();
    check(label, expected, Value::List(actual));
}

fn clear_records(ctx: &RustCtx) {
    ctx.gcode.state.responses.borrow_mut().clear();
    ctx.gcode.state.scripts.borrow_mut().clear();
}

fn py_clear(py: &mut pyoracle::Oracle) {
    py.exec("g.responses = []\ng.scripts = []").expect("clear");
}

/// `PyValue` (from `get_status`) to the recording `Value` encoding.
fn pyv_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let text = n.inner().to_string();
            match text.parse::<i64>() {
                Ok(i) => Value::int(i),
                Err(_) => Value::BigInt(text),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyv_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyv_to_value).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter()
                .map(|(k, val)| (pyv_to_value(k), pyv_to_value(val)))
                .collect(),
        ),
        PyValue::Host(_) => panic!("host value in get_status"),
    }
}

fn check_status(label: &str, py: &mut pyoracle::Oracle, ctx: &RustCtx, eventtime: f64) {
    let expected = py.eval(&format!("cb.get_status({eventtime:?})"));
    let actual = pyv_to_value(&ctx.sensor.get_status(eventtime));
    check(label, expected, actual);
}

fn check_float(label: &str, expected: pyoracle::OracleResult, actual: f64) {
    check(label, expected, Value::float(actual));
}

fn check_never_or_time(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<f64>,
) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, Ok(expected), Value::float(actual));
}

const NOW_VAL: f64 = 0.0;

#[test]
fn constants_match() {
    let Some(mut py) = oracle() else { return };
    // `ADC_SAMPLE_COUNT` is an int in the reference, the other two are
    // floats; compare each with its own type.
    for (name, rust) in [("ADC_REPORT_TIME", ADC_REPORT_TIME), ("ADC_SAMPLE_TIME", ADC_SAMPLE_TIME)] {
        let expected = py.eval(&format!("hall.{name}"));
        check_float(&format!("constants_match {name}"), expected, rust);
    }
    let expected = py.eval("hall.ADC_SAMPLE_COUNT");
    check("constants_match ADC_SAMPLE_COUNT", expected, Value::int(ADC_SAMPLE_COUNT));
}

struct FakeToolhead {
    position: GCell<Vec<f64>>,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.borrow().clone()
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by hall_filament_width_sensor")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by hall_filament_width_sensor")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by hall_filament_width_sensor")
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used by hall_filament_width_sensor")
    }
    fn dwell(&self, _delay: f64) {
        panic!("dwell is not used by hall_filament_width_sensor")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by hall_filament_width_sensor")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by hall_filament_width_sensor")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by hall_filament_width_sensor")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by hall_filament_width_sensor")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("toolhead get_status is not used by hall_filament_width_sensor")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by hall_filament_width_sensor")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by hall_filament_width_sensor")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by hall_filament_width_sensor")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback is not used by hall_filament_width_sensor")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by hall_filament_width_sensor")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by hall_filament_width_sensor")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by hall_filament_width_sensor")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by hall_filament_width_sensor")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by hall_filament_width_sensor")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used by hall_filament_width_sensor")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        vec![None, None, None]
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("toolhead stats is not used by hall_filament_width_sensor")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by hall_filament_width_sensor")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

#[test]
fn init_registers_commands_pins_and_adcs() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    // Registration records (cmd, desc, when_not_ready).
    let expected = py.eval("g.registrations");
    let actual: Vec<Value> = ctx
        .gcode
        .state
        .registrations
        .borrow()
        .iter()
        .map(|(cmd, desc, wnr)| {
            Value::Tuple(vec![
                Value::str(cmd),
                match desc {
                    Some(d) => Value::str(d),
                    None => Value::Null,
                },
                Value::Bool(*wnr),
            ])
        })
        .collect();
    check("init registrations", expected, Value::List(actual));
    // Pin setup calls.
    let expected = py.eval("pins.setup_calls");
    let actual: Vec<Value> = ctx
        .pins
        .setup_calls
        .borrow()
        .iter()
        .map(|(t, d)| Value::Tuple(vec![Value::str(t), Value::str(d)]))
        .collect();
    check("init pin setup", expected, Value::List(actual));
    // ADC sample parameters for pin 1.
    let expected = py.eval("pins.adc_pins[0].sample_args");
    let actual = {
        let pins = ctx.pins.pins.borrow();
        let pin: Arc<FakeAdcPin> = pins[0].clone();
        let samples = pin.samples.borrow();
        let (report_time, opts) = &samples[0];
        Value::Tuple(vec![
            Value::float(*report_time),
            Value::float(opts.sample_time),
            Value::int(opts.sample_count),
            Value::int(opts.batch_num),
            Value::float(opts.minval),
            Value::float(opts.maxval),
            Value::int(opts.range_check_count),
        ])
    };
    check("init adc1 sample", expected, actual);
    // ADC sample parameters for pin 2.
    let expected = py.eval("pins.adc_pins[1].sample_args");
    let actual = {
        let pins = ctx.pins.pins.borrow();
        let pin: Arc<FakeAdcPin> = pins[1].clone();
        let samples = pin.samples.borrow();
        let (report_time, opts) = &samples[0];
        Value::Tuple(vec![
            Value::float(*report_time),
            Value::float(opts.sample_time),
            Value::int(opts.sample_count),
            Value::int(opts.batch_num),
            Value::float(opts.minval),
            Value::float(opts.maxval),
            Value::int(opts.range_check_count),
        ])
    };
    check("init adc2 sample", expected, actual);
    // A timer was registered on both reactors (Python side records it;
    // the Rust side uses the real reactor, so only existence is
    // checked there by a successful fire below).
    let expected = py.eval("p.reactor.timers[0]['waketime']");
    check_float("init timer waketime", expected, 9999999999999999.0);
    // Helper enabled from `enable = true` on both sides.
    let expected = py.eval("cb.runout_helper.sensor_enabled");
    let actual = Value::Bool(ctx.helper.sensor_enabled());
    check("init helper enabled", expected, actual);
}

#[test]
fn init_config_errors() {
    let cases: Vec<(&'static str, Vec<(&'static str, &'static str)>)> = vec![
        (
            "raw_dia_equal",
            default_options()
                .into_iter()
                .map(|(k, v)| if k == "Raw_dia2" { (k, "9500") } else { (k, v) })
                .collect(),
        ),
        (
            "max_difference_too_big",
            default_options()
                .into_iter()
                .map(|(k, v)| {
                    if k == "default_nominal_filament_diameter" {
                        (k, "1.5")
                    } else if k == "max_difference" {
                        (k, "1.6")
                    } else {
                        (k, v)
                    }
                })
                .collect(),
        ),
        (
            "missing_adc1",
            default_options()
                .into_iter()
                .filter(|(k, _)| *k != "adc1")
                .collect(),
        ),
        (
            "missing_nominal",
            default_options()
                .into_iter()
                .filter(|(k, _)| *k != "default_nominal_filament_diameter")
                .collect(),
        ),
        (
            "nominal_not_above_1",
            default_options()
                .into_iter()
                .map(|(k, v)| {
                    if k == "default_nominal_filament_diameter" {
                        (k, "1.0")
                    } else {
                        (k, v)
                    }
                })
                .collect(),
        ),
        (
            "delay_not_above_0",
            default_options()
                .into_iter()
                .map(|(k, v)| if k == "measurement_delay" { (k, "0.0") } else { (k, v) })
                .collect(),
        ),
        (
            "unparseable_int",
            default_options()
                .into_iter()
                .map(|(k, v)| if k == "Raw_dia1" { (k, "abc") } else { (k, v) })
                .collect(),
        ),
        (
            "unparseable_float",
            default_options()
                .into_iter()
                .map(|(k, v)| if k == "Cal_dia1" { (k, "xyz") } else { (k, v) })
                .collect(),
        ),
        (
            "unparseable_bool",
            default_options()
                .into_iter()
                .map(|(k, v)| if k == "enable" { (k, "maybe") } else { (k, v) })
                .collect(),
        ),
    ];
    for (label, rows) in cases {
        let Some(mut py) = oracle() else { return };
        let setup = format!(
            "p = FakePrinter()\ng = FakeGCode()\npins = FakePins()\nth = FakeToolhead()\np.objects = {{'gcode': g, 'pins': pins, 'toolhead': th}}\nc = FakeConfig(p, {})\ncb = hall.load_config(c)",
            py_options_dict(&rows)
        );
        let expected = py.exec(&setup);
        // Rust side: build the config directly.
        let printer = Arc::new(FakePrinter::new(
            Arc::new(FakePins {
                setup_calls: GCell::new(Vec::new()),
                pins: GCell::new(Vec::new()),
            }),
            Arc::new(FakeGCode::new()),
            Arc::new(FakeToolhead {
                position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            }),
        ));
        let mut map = IndexMap::new();
        for (k, v) in &rows {
            map.insert(k.to_string(), v.to_string());
        }
        let config: Config = Arc::new(FakeConfig {
            printer: printer.clone() as Printer,
            options: map,
            section: "hall_filament_width_sensor".to_string(),
        });
        let actual = HallFilamentWidthSensor::new(&config).map(|_| ());
        check_err(&format!("init_config_errors {label}"), expected, actual);
    }
}

/// Drives 20 readings of `value` into both ADC pins on both sides.
fn pump(py: &mut pyoracle::Oracle, ctx: &RustCtx, value: f64, count: usize) {
    for _ in 0..count {
        py.exec(&format!("pins.adc_pins[0].callback([(1.0, {value:?})])"))
            .expect("py adc1");
        py.exec(&format!("pins.adc_pins[1].callback([(1.0, {value:?})])"))
            .expect("py adc2");
        adc(ctx, 0, vec![(1.0, value)]);
        adc(ctx, 1, vec![(1.0, value)]);
    }
}

#[test]
fn adc_callbacks_and_diameter() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    // adc1: last sample only, round-half-even readings.
    let value = 0.485;
    py.exec("pins.adc_pins[0].callback([(1.0, 0.1), (2.0, 0.485)])")
        .expect("py adc1 pair");
    adc(&ctx, 0, vec![(1.0, 0.1), (2.0, 0.485)]);
    // Raw readings after the first pair.
    let expected = py.eval("cb.lastFilamentWidthReading");
    check("adc1 reading", expected, Value::int(4850));
    let expected = py.eval("cb.lastFilamentWidthReading2");
    check("adc2 reading before callback", expected, Value::int(0));
    // adc2: diameter moves toward diameter_new with 5/6 smoothing.
    for _ in 0..30 {
        py.exec(&format!("pins.adc_pins[1].callback([(1.0, {value:?})])"))
            .expect("py adc2");
        adc(&ctx, 1, vec![(1.0, value)]);
    }
    let expected = py.eval("cb.get_status(0.0)['Diameter']");
    check_float("diameter after smoothing", expected, diameter_of(&ctx));
    // Half-even rounding: x.5 rounds to even.
    py.exec("pins.adc_pins[0].callback([(1.0, 0.00025)])").expect("py");
    adc(&ctx, 0, vec![(1.0, 0.00025)]);
    let expected = py.eval("cb.lastFilamentWidthReading");
    check("round 2.5 to even", expected, Value::int(2));
    py.exec("pins.adc_pins[0].callback([(1.0, 0.00035)])").expect("py");
    adc(&ctx, 0, vec![(1.0, 0.00035)]);
    let expected = py.eval("cb.lastFilamentWidthReading");
    check("round 3.5 to even", expected, Value::int(4));
    py.exec("pins.adc_pins[0].callback([(1.0, 0.4855)])").expect("py");
    adc(&ctx, 0, vec![(1.0, 0.4855)]);
    let expected = py.eval("cb.lastFilamentWidthReading");
    check("round 4855.0", expected, Value::int(4855));
}

#[test]
fn queries_and_get_status() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    // Status shape (keys, order, values) before any movement.
    check_status("status initial", &mut py, &ctx, 0.0);
    // QUERY_FILAMENT_WIDTH, sensor on, filament not present.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]);
    run_cmd(&ctx, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]).expect("rust query");
    check_responses("query width off", &mut py, &ctx);
    // Note filament present, then query again.
    py.exec("cb.runout_helper.note_filament_present(1.0, True)").expect("py note");
    ctx.helper.note_filament_present(1.0, true).expect("rust note");
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]);
    run_cmd(&ctx, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]).expect("rust query");
    check_responses("query width present", &mut py, &ctx);
    check_status("status present", &mut py, &ctx, 0.0);
    // QUERY_RAW_FILAMENT_WIDTH: readings feed it.
    pump(&mut py, &ctx, 0.485, 2);
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "QUERY_RAW_FILAMENT_WIDTH", "QUERY_RAW_FILAMENT_WIDTH", &[]);
    run_cmd(&ctx, "QUERY_RAW_FILAMENT_WIDTH", "QUERY_RAW_FILAMENT_WIDTH", &[]).expect("rust raw");
    check_responses("query raw", &mut py, &ctx);
    check_status("status readings", &mut py, &ctx, 0.0);
    // Logging enable/disable.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "ENABLE_FILAMENT_WIDTH_LOG", "ENABLE_FILAMENT_WIDTH_LOG", &[]);
    run_cmd(&ctx, "ENABLE_FILAMENT_WIDTH_LOG", "ENABLE_FILAMENT_WIDTH_LOG", &[]).expect("log on");
    check_responses("log enable", &mut py, &ctx);
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "DISABLE_FILAMENT_WIDTH_LOG", "DISABLE_FILAMENT_WIDTH_LOG", &[]);
    run_cmd(&ctx, "DISABLE_FILAMENT_WIDTH_LOG", "DISABLE_FILAMENT_WIDTH_LOG", &[]).expect("log off");
    check_responses("log disable", &mut py, &ctx);
}

#[test]
fn disabled_queries() {
    let Some(mut py) = oracle() else { return };
    let opts: Vec<(&'static str, &'static str)> = default_options()
        .into_iter()
        .map(|(k, v)| if k == "enable" { (k, "false") } else { (k, v) })
        .collect();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    check_status("status disabled", &mut py, &ctx, 0.0);
    py_cmd(&mut py, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]);
    run_cmd(&ctx, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]).expect("query");
    check_responses("query disabled", &mut py, &ctx);
    // adc2 samples while disabled: diameter resets to nominal.
    pump(&mut py, &ctx, 0.9, 3);
    let expected = py.eval("cb.get_status(0.0)['Diameter']");
    check_float("diameter disabled", expected, diameter_of(&ctx));
}

fn diameter_of(ctx: &RustCtx) -> f64 {
    match ctx.sensor.get_status(0.0) {
        PyValue::Dict(map) => match map.get(&PyValue::Str("Diameter".to_string())) {
            Some(PyValue::Float(f)) => *f,
            _ => panic!("Diameter missing from status"),
        },
        _ => panic!("status is not a dict"),
    }
}

#[test]
fn reset_sensor_active_and_inactive() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    fire_ready(&mut py, &ctx);
    // Fill the array on both sides through one extrude event.
    py.exec("th.pos[3] = 5.0").expect("py pos");
    ctx.toolhead.position.borrow_mut()[3] = 5.0;
    let expected = py.eval("cb.extrude_factor_update_event(1.0)");
    check_never_or_time("event time armed", expected, ctx.sensor.extrude_factor_update_event(1.0));
    // RESET while active: deferred (array_clear_pending), no script yet.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "RESET_FILAMENT_WIDTH_SENSOR", "RESET_FILAMENT_WIDTH_SENSOR", &[]);
    run_cmd(&ctx, "RESET_FILAMENT_WIDTH_SENSOR", "RESET_FILAMENT_WIDTH_SENSOR", &[]).expect("reset");
    check_responses("reset active text", &mut py, &ctx);
    check_scripts("reset active no script", &mut py, &ctx);
    // The next event clears: M221 S100 first, then the pending clear.
    py.exec("th.pos[3] = 6.0").expect("py pos");
    ctx.toolhead.position.borrow_mut()[3] = 6.0;
    py_clear(&mut py);
    clear_records(&ctx);
    let expected = py.eval("cb.extrude_factor_update_event(1.0)");
    let actual = ctx.sensor.extrude_factor_update_event(1.0);
    check_never_or_time("event time after clear", expected, actual);
    check_scripts("clear script", &mut py, &ctx);
    check_status("status after clear", &mut py, &ctx, 0.0);
}

#[test]
fn enable_sensor_variants() {
    let Some(mut py) = oracle() else { return };
    let opts: Vec<(&'static str, &'static str)> = default_options()
        .into_iter()
        .map(|(k, v)| if k == "enable" { (k, "false") } else { (k, v) })
        .collect();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    // Enable without parameters: flow stays on, sensor on.
    py_cmd(&mut py, "ENABLE_FILAMENT_WIDTH_SENSOR", "ENABLE_FILAMENT_WIDTH_SENSOR", &[]);
    run_cmd(&ctx, "ENABLE_FILAMENT_WIDTH_SENSOR", "ENABLE_FILAMENT_WIDTH_SENSOR", &[]).expect("enable");
    check_responses("enable plain", &mut py, &ctx);
    check_status("status enabled", &mut py, &ctx, 0.0);
    // Disable flow compensation (int 0).
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "ENABLE_FILAMENT_WIDTH_SENSOR", "ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=0", &[("FLOW_COMPENSATION", "0")]);
    run_cmd(&ctx, "ENABLE_FILAMENT_WIDTH_SENSOR", "ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=0", &[("FLOW_COMPENSATION", "0")]).expect("flow 0");
    check_responses("enable flow 0", &mut py, &ctx);
    check_status("status flow 0", &mut py, &ctx, 0.0);
    // FLOW_COMPENSATION=2: parameter out of range (max 1).
    let expected = py.eval("g.handlers['ENABLE_FILAMENT_WIDTH_SENSOR'](g.create_gcode_command('ENABLE_FILAMENT_WIDTH_SENSOR', 'ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=2', {'FLOW_COMPENSATION': '2'}))");
    let actual = run_cmd(&ctx, "ENABLE_FILAMENT_WIDTH_SENSOR", "ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=2", &[("FLOW_COMPENSATION", "2")]);
    check_err("flow out of range", expected, actual);
    // FLOW_COMPENSATION=abc: unable to parse.
    let expected = py.eval("g.handlers['ENABLE_FILAMENT_WIDTH_SENSOR'](g.create_gcode_command('ENABLE_FILAMENT_WIDTH_SENSOR', 'ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=abc', {'FLOW_COMPENSATION': 'abc'}))");
    let actual = run_cmd(&ctx, "ENABLE_FILAMENT_WIDTH_SENSOR", "ENABLE_FILAMENT_WIDTH_SENSOR FLOW_COMPENSATION=abc", &[("FLOW_COMPENSATION", "abc")]);
    check_err("flow unparseable", expected, actual);
}

#[test]
fn disable_sensor_pending_and_immediate() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    // Disable while active: only pending, text immediate.
    py_cmd(&mut py, "DISABLE_FILAMENT_WIDTH_SENSOR", "DISABLE_FILAMENT_WIDTH_SENSOR", &[]);
    run_cmd(&ctx, "DISABLE_FILAMENT_WIDTH_SENSOR", "DISABLE_FILAMENT_WIDTH_SENSOR", &[]).expect("disable");
    check_responses("disable pending text", &mut py, &ctx);
    check_scripts("disable pending no script", &mut py, &ctx);
    // Query still shows the sensor on until the timer runs.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]);
    run_cmd(&ctx, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]).expect("query");
    check_responses("query pre-timer", &mut py, &ctx);
    // The event applies the disable: M221 S100, then NEVER.
    py_clear(&mut py);
    clear_records(&ctx);
    let expected = py.eval("cb.extrude_factor_update_event(1.0)");
    let actual = ctx.sensor.extrude_factor_update_event(1.0);
    check_never_or_time("disable event time", expected, actual);
    check_scripts("disable script", &mut py, &ctx);
    check_status("status disabled after", &mut py, &ctx, 0.0);
    py.exec("th.pos[3] = 5.0").expect("py pos");
    ctx.toolhead.position.borrow_mut()[3] = 5.0;
    // Disable again while inactive: immediate M221 from the command.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "DISABLE_FILAMENT_WIDTH_SENSOR", "DISABLE_FILAMENT_WIDTH_SENSOR", &[]);
    run_cmd(&ctx, "DISABLE_FILAMENT_WIDTH_SENSOR", "DISABLE_FILAMENT_WIDTH_SENSOR", &[]).expect("disable again");
    check_responses("disable again text", &mut py, &ctx);
    check_scripts("disable again script", &mut py, &ctx);
    // RESET while inactive: immediate M221 from the command.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "RESET_FILAMENT_WIDTH_SENSOR", "RESET_FILAMENT_WIDTH_SENSOR", &[]);
    run_cmd(&ctx, "RESET_FILAMENT_WIDTH_SENSOR", "RESET_FILAMENT_WIDTH_SENSOR", &[]).expect("reset inactive");
    check_responses("reset inactive text", &mut py, &ctx);
    check_scripts("reset inactive script", &mut py, &ctx);
}

fn set_e(py: &mut pyoracle::Oracle, ctx: &RustCtx, e: f64) {
    py.exec(&format!("th.pos[3] = {e:?}")).expect("py pos");
    ctx.toolhead.position.borrow_mut()[3] = e;
}

fn fire_event(py: &mut pyoracle::Oracle, ctx: &RustCtx, eventtime: f64) -> f64 {
    let expected = py.eval(&format!("cb.extrude_factor_update_event({eventtime:?})"));
    match ctx.sensor.extrude_factor_update_event(eventtime) {
        Ok(t) => {
            check("fire event time", expected, Value::float(t));
            t
        }
        Err(e) => panic!("rust event raised {}: {}", e.class_name(), e.message()),
    }
}

/// Fire `klippy:ready` on both sides so `handle_ready` looks up the
/// toolhead before any extrude-factor timer event runs.
fn fire_ready(py: &mut pyoracle::Oracle, ctx: &RustCtx) {
    py.exec("p.send_event('klippy:ready')").expect("py ready");
    ctx.printer.fire_event("klippy:ready", &[]);
}

#[test]
fn extrude_flow_compensation() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    fire_ready(&mut py, &ctx);
    // Stabilize diameter at 1.60 mm (M221 S120 with a 1.75 nominal).
    pump(&mut py, &ctx, 0.485, 30);
    check_float("diameter pre-move", py.eval("cb.get_status(0.0)['Diameter']"), diameter_of(&ctx));
    // Move E forward; every interval crossing appends and pops, the
    // compensation from the popped width drives M221.
    for e in [5.0, 7.0, 12.0, 17.0, 22.0, 27.0, 32.0] {
        set_e(&mut py, &ctx, e);
        let t = fire_event(&mut py, &ctx, 1.0);
        assert_eq!(t, 2.0, "armed timer returns eventtime + 1");
        check_scripts(&format!("flow scripts e={e}"), &mut py, &ctx);
    }
    check_status("status after moves", &mut py, &ctx, 0.0);
    // Query shows the queued reading.
    py_clear(&mut py);
    clear_records(&ctx);
    py_cmd(&mut py, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]);
    run_cmd(&ctx, "QUERY_FILAMENT_WIDTH", "QUERY_FILAMENT_WIDTH", &[]).expect("query");
    check_responses("query after moves", &mut py, &ctx);
}

#[test]
fn extrude_below_min_and_flow_off() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    fire_ready(&mut py, &ctx);
    // diameter_new = 0.0005*( (0.05+0.05)*10000 - 9500 ) + 1.5 < 1.0.
    pump(&mut py, &ctx, 0.05, 30);
    set_e(&mut py, &ctx, 5.0);
    fire_event(&mut py, &ctx, 1.0);
    check_scripts("below min scripts", &mut py, &ctx);
    check_status("below min status", &mut py, &ctx, 0.0);
    // Runout gating is exercised through status below.
}

#[test]
fn extrude_while_delay_and_logging() {
    let Some(mut py) = oracle() else { return };
    let opts: Vec<(&'static str, &'static str)> = default_options()
        .into_iter()
        .map(|(k, v)| {
            if k == "use_current_dia_while_delay" || k == "logging" {
                (k, "true")
            } else {
                (k, v)
            }
        })
        .collect();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    fire_ready(&mut py, &ctx);
    set_e(&mut py, &ctx, 0.0);
    fire_event(&mut py, &ctx, 1.0);
    // Pending first entry is at delay + 0.0 = 2.0, ahead of the
    // current E position, so the current diameter is used; the log
    // line is just the previous (empty) array state so nothing is
    // logged yet.
    check_scripts("delay scripts 1", &mut py, &ctx);
    check_responses("delay responses 1", &mut py, &ctx);
    set_e(&mut py, &ctx, 20.0);
    fire_event(&mut py, &ctx, 1.0);
    check_scripts("delay scripts 2", &mut py, &ctx);
    check_responses("delay responses 2", &mut py, &ctx);
}

#[test]
fn handle_ready_and_event_before_ready() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    py_ctx(&mut py, &opts);
    let ctx = rust_ctx(&opts);
    // Event before klippy:ready: no toolhead yet (AttributeError).
    let expected = py.eval("cb.extrude_factor_update_event(1.0)");
    let actual = ctx.sensor.extrude_factor_update_event(1.0);
    let expected_err = match expected {
        Ok(v) => panic!("oracle returned {v:?} before ready"),
        Err(e) => e,
    };
    let actual_err = match actual {
        Ok(_) => panic!("rust did not raise before ready"),
        Err(e) => e,
    };
    assert_eq!(actual_err.class_name(), expected_err.class_name.as_str());
    assert_eq!(actual_err.message(), expected_err.message.as_str());
    // Fire klippy:ready on both sides.
    py.exec("p.send_event('klippy:ready')").expect("py ready");
    ctx.printer.fire_event("klippy:ready", &[]);
    // Python: the timer is now armed (waketime NOW recorded).
    let expected = py.eval("p.reactor.updates");
    check("ready timer updates", expected, Value::List(vec![Value::float(NOW_VAL)]));
    // The event now runs.
    set_e(&mut py, &ctx, 5.0);
    fire_event(&mut py, &ctx, 1.0);
    check_scripts("ready scripts", &mut py, &ctx);
    check_status("ready status", &mut py, &ctx, 0.0);
}

#[test]
fn load_config_factory_builds_object() {
    let Some(mut py) = oracle() else { return };
    let opts = default_options();
    // Python side: load_config builds the sensor and its real helper.
    py_ctx(&mut py, &opts);
    let factory = extras::hall_filament_width_sensor::module_factory();
    let load_config = factory
        .load_config
        .expect("hall_filament_width_sensor exposes load_config");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    // Rust side: the factory builds the real object (sensor + helper).
    let printer = Arc::new(FakePrinter::new(
        Arc::new(FakePins {
            setup_calls: GCell::new(Vec::new()),
            pins: GCell::new(Vec::new()),
        }),
        Arc::new(FakeGCode::new()),
        Arc::new(FakeToolhead {
            position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
        }),
    ));
    let mut map = IndexMap::new();
    for (k, v) in &opts {
        map.insert(k.to_string(), v.to_string());
    }
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        options: map,
        section: "hall_filament_width_sensor".to_string(),
    });
    let obj = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns the object");
    // The factory-built object's status matches the Python object.
    let expected = py.eval("cb.get_status(0.0)");
    let actual = pyv_to_value(&obj.get_status(0.0).expect("status"));
    check("factory load_config status", expected, actual);
}

