//! Parity tests for `extras::pwm_cycle_time` (port of
//! `klippy/extras/pwm_cycle_time.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterOutputPWMCycle`
//! is built on both sides with minimal Python fakes for the printer, the
//! config wrapper, the MCU, pins, gcode and toolhead, then fed identical
//! config options, `_build_config` / `set_pwm_cycle` / `SET_PIN` calls and
//! `_set_pin` steps. Compared exactly: returned values (floats by bits),
//! raised errors (class and message), the MCU command strings and
//! `CommandWrapper.send` traffic (format string, data, `minclock` /
//! `reqclock`), the `SET_PIN` mux registration, the lookahead callback
//! registration and the `get_status` dict.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::pwm_cycle_time::PrinterOutputPWMCycle;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler,
    GetFloatOpts as GcmdGetFloatOpts, GetIntOpts as GcmdGetIntOpts,
    GetOpts as GcmdGetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueryWrapper, CommandQueue,
    CommandWrapper, CommandWrapperApi, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinsApi, SendOpts, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, LookaheadCallback, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.pwm_cycle_time as pwm

class CommandError(Exception):
    pass

class Sentinel:
    pass

class FakeReactor:
    def monotonic(self):
        return 100.0

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
    def get_reactor(self):
        return FakeReactor()
    def lookup_object(self, name, default=Sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            return None
        raise self.config_error("Unknown config object '%s'" % (name,))
    def add(self, name, obj):
        self.objects[name] = obj

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, opts):
        self._printer = printer
        self.section = section
        self.opts = dict(opts)
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def _get_opt(self, option, default, minval, maxval, above, below):
        if option not in self.opts:
            if default is not Sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        raw = self.opts[option]
        try:
            v = float(raw)
        except Exception:
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
    def getfloat(self, option, default=Sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        return self._get_opt(option, default, minval, maxval, above, below)
    def get(self, option, default=Sentinel, note_valid=True):
        if option not in self.opts:
            if default is not Sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return self.opts[option]

class FakeCommand:
    def __init__(self, mcu, msgformat, cq):
        self.mcu = mcu
        self.msgformat = msgformat
        self.cq = cq
    def send(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append((self.msgformat, tuple(data), minclock, reqclock))

class FakeMCU:
    def __init__(self, printer, max_duration):
        self._printer = printer
        self.max_duration = float(max_duration)
        self.config_cmds = []
        self.callbacks = []
        self.oid = 3
        self.sends = []
        self.commands = []
        self.requested_slot = 0
    def get_printer(self):
        return self._printer
    def alloc_command_queue(self):
        return 7
    def estimated_print_time(self, curtime):
        return 10.0
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000.0)
    def seconds_to_clock(self, time):
        return int(time * 1000.0)
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def create_oid(self):
        return self.oid
    def request_move_queue_slot(self):
        self.requested_slot += 1
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, is_init))
    def lookup_command(self, msgformat, cq=None):
        self.commands.append((msgformat, cq))
        return FakeCommand(self, msgformat, cq)
    def min_schedule_time(self):
        return 0.001
    def max_nominal_duration(self):
        return self.max_duration

class FakePins:
    def __init__(self, pin_params):
        self.pin_params = pin_params
        self.last_desc = None
        self.last_opts = None
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        self.last_desc = pin_desc
        self.last_opts = (can_invert, can_pullup, share_type)
        return self.pin_params

class FakeGCode:
    def __init__(self):
        self.mux = []
        self.handler = None
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))
        self.handler = func

class FakeToolhead:
    def __init__(self):
        self.callbacks = []
    def register_lookahead_callback(self, cb):
        self.callbacks.append(cb)

class FakeGCmd:
    error = CommandError
    def __init__(self, commandline, params):
        self._commandline = commandline
        self._params = dict(params)
    def get_commandline(self):
        return self._commandline
    def get_command(self):
        return 'SET_PIN'
    def get_float(self, name, default=Sentinel, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is Sentinel:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = float(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s" % (self._commandline, name, below))
        return value

def make_world(pin_name, invert, max_duration, opts):
    global printer, mcu, pins, gcode, toolhead, config, obj
    printer = FakePrinter()
    mcu = FakeMCU(printer, max_duration)
    pins = FakePins({'chip': mcu, 'pin': 'PA0', 'invert': invert, 'pullup': 0, 'chip_name': 'mcu', 'share_type': None})
    gcode = FakeGCode()
    toolhead = FakeToolhead()
    printer.add('pins', pins)
    printer.add('gcode', gcode)
    printer.add('toolhead', toolhead)
    config = FakeConfig(printer, 'pwm_cycle_time ' + pin_name, opts)
    obj = pwm.PrinterOutputPWMCycle(config)
    return obj

def make_mcu_pwm(pin_name, invert, max_duration, cycle_time, start_value, shutdown_value):
    global printer, mcu
    printer = FakePrinter()
    mcu = FakeMCU(printer, max_duration)
    pin_params = {'chip': mcu, 'pin': 'PA0', 'invert': invert, 'pullup': 0, 'chip_name': 'mcu', 'share_type': None}
    mp = pwm.MCU_pwm_cycle(pin_params, cycle_time, start_value, shutdown_value)
    return mp
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

const FREQ: f64 = 1000.0;

fn trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = pycompat::text::trunc_float_to_bigint(f).0.to_string();
    digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

fn secs_to_clock(t: f64) -> i64 {
    trunc_i64(t * FREQ)
}

fn float_repr(f: f64) -> String {
    recording::value::py_float_repr(f)
}

struct FakePrinter {
    objects: std::sync::Mutex<Vec<(String, ObjectRef)>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter { objects: std::sync::Mutex::new(Vec::new()) })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().push((name.to_string(), obj));
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
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock().unwrap();
        for (n, o) in objects.iter() {
            if n == name {
                return Ok(Some(o.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
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
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    section: String,
    opts: std::sync::Mutex<Vec<(String, String)>>,
}

fn missing_opt(section: &str, option: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

fn parse_error(section: &str, option: &str) -> KlippyError {
    KlippyError::config(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

fn bound_err(
    section: &str,
    option: &str,
    kind: &str,
    bound: f64,
    _param: Option<f64>,
) -> KlippyError {
    // G-code and config formats differ; the config ones end with
    // "must have minimum of X" / "must have maximum of X" / "must be above X".
    match kind {
        "minimum" => KlippyError::config(format!(
            "Option '{option}' in section '{section}' must have minimum of {}",
            float_repr(bound)
        )),
        "maximum" => KlippyError::config(format!(
            "Option '{option}' in section '{section}' must have maximum of {}",
            float_repr(bound)
        )),
        "above" => KlippyError::config(format!(
            "Option '{option}' in section '{section}' must be above {}",
            float_repr(bound)
        )),
        _ => KlippyError::config(format!(
            "Option '{option}' in section '{section}' must be below {}",
            float_repr(bound)
        )),
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
        let opts_map = self.opts.lock().unwrap();
        let raw = opts_map
            .iter()
            .find(|(k, _)| k == option)
            .map(|(_, v)| v.clone());
        drop(opts_map);
        match raw {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(missing_opt(&self.section, option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by pwm_cycle_time")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = {
            let opts_map = self.opts.lock().unwrap();
            opts_map
                .iter()
                .find(|(k, _)| k == option)
                .map(|(_, v)| v.clone())
        };
        let Some(raw) = raw else {
            return match opts.default {
                None => Err(missing_opt(&self.section, option)),
                Some(d) => Ok(d),
            };
        };
        let v = pycompat::text::float_from_string(&raw)
            .map_err(|_| parse_error(&self.section, option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(bound_err(&self.section, option, "minimum", min, None));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(bound_err(&self.section, option, "maximum", max, None));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(bound_err(&self.section, option, "above", above, None));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(bound_err(&self.section, option, "below", below, None));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by pwm_cycle_time")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by pwm_cycle_time")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by pwm_cycle_time")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by pwm_cycle_time")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by pwm_cycle_time")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by pwm_cycle_time")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by pwm_cycle_time")
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

/// Shared state of the fake MCU so the command wrappers and the "chip"
/// pins fake can inspect it.
#[derive(Clone, Default)]
struct FakeMcuShared {
    config_cmds: std::sync::Arc<std::sync::Mutex<Vec<(String, AddConfigCmdOpts)>>>,
    oid: i64,
    sends: std::sync::Arc<std::sync::Mutex<Vec<(String, Vec<i64>, i64, i64)>>>,
    commands: std::sync::Arc<std::sync::Mutex<Vec<(String, Option<CommandQueue>)>>>,
    requested_slot: std::sync::Arc<std::sync::Mutex<i64>>,
    max_duration: f64,
}

struct FakeCommand {
    shared: FakeMcuShared,
    format: String,
}

impl CommandWrapperApi for FakeCommand {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let ints: Vec<i64> = data
            .iter()
            .map(|v| match v {
                PyValue::Int(n) => n.0.to_string().parse::<i64>().unwrap(),
                _ => panic!("send data must be ints"),
            })
            .collect();
        self.shared
            .sends
            .lock()
            .unwrap()
            .push((self.format.clone(), ints, opts.minclock, opts.reqclock));
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Err(KlippyError::Mcu("send_wait_ack is not used by pwm_cycle_time".to_string()))
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeMcu {
    printer: Printer,
    shared: FakeMcuShared,
    callbacks: std::sync::Arc<std::sync::Mutex<Vec<klippy_api::mcu::ConfigCallback>>>,
}

fn err_unused_mcu() -> KlippyError {
    KlippyError::Mcu("not used by pwm_cycle_time".to_string())
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
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(err_unused_mcu())
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(self.shared.oid)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.callbacks.lock().unwrap().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.shared.config_cmds.lock().unwrap().push((cmd.to_string(), opts));
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        *self.shared.requested_slot.lock().unwrap() += 1;
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        secs_to_clock(time)
    }
    fn min_schedule_time(&self) -> f64 {
        0.001
    }
    fn max_nominal_duration(&self) -> f64 {
        self.shared.max_duration
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        self.shared
            .commands
            .lock()
            .unwrap()
            .push((msgformat.to_string(), cq));
        Ok(Arc::new(FakeCommand {
            shared: self.shared.clone(),
            format: msgformat.to_string(),
        }))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(err_unused_mcu())
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        struct FakeAsync;
        impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
            fn unregister(&self) {}
        }
        Arc::new(FakeAsync)
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
        Err(err_unused_mcu())
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        secs_to_clock(print_time)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / FREQ
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        10.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Err(err_unused_mcu())
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(err_unused_mcu())
    }
}

struct FakePins {
    mcu: Mcu,
    invert: i64,
    last_desc: std::sync::Mutex<Option<String>>,
    last_can_invert: std::sync::Mutex<Option<bool>>,
}

fn err_unused_pins() -> KlippyError {
    KlippyError::Other {
        class_name: "error".to_string(),
        message: "not used by pwm_cycle_time".to_string(),
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Err(err_unused_pins())
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        *self.last_desc.lock().unwrap() = Some(pin_desc.to_string());
        *self.last_can_invert.lock().unwrap() = Some(opts.can_invert);
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: "PA0".to_string(),
            invert: self.invert,
            pullup: 0,
            share_type: None,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(err_unused_pins())
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        Err(err_unused_pins())
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Err(err_unused_pins())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Err(err_unused_pins())
    }
}

struct FakeGCode {
    mux: std::sync::Mutex<Vec<(String, String, Option<String>, Option<String>)>>,
    handler: std::sync::Mutex<Option<GCodeCommandHandler>>,
}

fn err_unused_gcode() -> KlippyError {
    KlippyError::Mcu("not used by pwm_cycle_time".to_string())
}

impl GCodeApi for FakeGCode {
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
        self.mux.lock().unwrap().push((
            cmd.to_string(),
            key.to_string(),
            value.map(|s| s.to_string()),
            desc.map(|s| s.to_string()),
        ));
        *self.handler.lock().unwrap() = Some(func);
        Ok(())
    }
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Err(err_unused_gcode())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Err(err_unused_gcode())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Err(err_unused_gcode())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by pwm_cycle_time")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Err(err_unused_gcode())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Err(err_unused_gcode())
    }
}

struct FakeToolhead {
    callbacks: std::sync::Mutex<Vec<LookaheadCallback>>,
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        Vec::new()
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Err(err_unused_gcode())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by pwm_cycle_time")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Err(err_unused_gcode())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Err(err_unused_gcode())
    }
}

fn err_unused_toolhead() -> KlippyError {
    KlippyError::Mcu("not used by pwm_cycle_time".to_string())
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Err(err_unused_toolhead())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Err(err_unused_toolhead())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by pwm_cycle_time")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, callback: LookaheadCallback) -> KResult<()> {
        self.callbacks.lock().unwrap().push(callback);
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by pwm_cycle_time")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by pwm_cycle_time")
    }
}

struct FakeGCmd {
    commandline: String,
    params: std::sync::Mutex<Vec<(String, String)>>,
}

fn gcmd_err(msg: String) -> KlippyError {
    KlippyError::command(msg)
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GcmdGetOpts) -> KResult<PyValue> {
        panic!("get is not used by pwm_cycle_time")
    }
    fn get_int(&self, _name: &str, _opts: GcmdGetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int is not used by pwm_cycle_time")
    }
    fn get_float(&self, name: &str, opts: GcmdGetFloatOpts) -> KResult<Option<f64>> {
        let raw = {
            let params = self.params.lock().unwrap();
            params.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone())
        };
        let Some(raw) = raw else {
            return match opts.default {
                None => Err(gcmd_err(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(d) => Ok(d),
            };
        };
        let v = pycompat::text::float_from_string(&raw).map_err(|_| {
            gcmd_err(format!(
                "Error on '{}': unable to parse {}",
                self.commandline, raw
            ))
        })?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(gcmd_err(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    float_repr(min)
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(gcmd_err(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    float_repr(max)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(gcmd_err(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(gcmd_err(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    float_repr(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn get_command(&self) -> String {
        "SET_PIN".to_string()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> indexmap::IndexMap<String, String> {
        let params = self.params.lock().unwrap();
        params
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect::<indexmap::IndexMap<_, _>>()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("pwm_cycle_time fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping pwm_cycle_time oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
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
        unit: "pwm_cycle_time".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        actual.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        actual.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

fn check_ok(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    if let Err(e) = actual {
        panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
    }
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    assert_eq!(expected, Value::Null, "{label}: expected null");
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().unwrap()),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "object".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn config_cmds_value(shared: &FakeMcuShared) -> Value {
    let cmds = shared.config_cmds.lock().unwrap();
    Value::List(
        cmds.iter()
            .map(|(s, o)| {
                Value::Tuple(vec![
                    Value::Str(s.clone()),
                    Value::Bool(o.is_init),
                ])
            })
            .collect(),
    )
}

fn sends_value(shared: &FakeMcuShared) -> Value {
    let sends = shared.sends.lock().unwrap();
    Value::List(
        sends.iter()
            .map(|(f, data, minclock, reqclock)| {
                Value::Tuple(vec![
                    Value::Str(f.clone()),
                    Value::Tuple(data.iter().map(|d| Value::int(*d)).collect()),
                    Value::int(*minclock),
                    Value::int(*reqclock),
                ])
            })
            .collect(),
    )
}

fn commands_value(shared: &FakeMcuShared) -> Value {
    let cmds = shared.commands.lock().unwrap();
    Value::List(
        cmds.iter()
            .map(|(f, cq)| {
                Value::Tuple(vec![
                    Value::Str(f.clone()),
                    match cq {
                        Some(CommandQueue(n)) => Value::int(*n),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

/// Builds the Rust world for `PrinterOutputPWMCycle` with the same
/// parameters as the Python `make_world` call.
#[allow(dead_code)]
struct World {
    printer: Arc<FakePrinter>,
    mcu: Mcu,
    shared: FakeMcuShared,
    callbacks: std::sync::Arc<std::sync::Mutex<Vec<klippy_api::mcu::ConfigCallback>>>,
    pins: Arc<FakePins>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    config: Config,
    obj: Arc<PrinterOutputPWMCycle>,
}

fn make_world(
    pin_name: &str,
    invert: bool,
    max_duration: f64,
    opts: &[(String, String)],
) -> World {
    let printer = FakePrinter::new();
    let shared = FakeMcuShared {
        oid: 3,
        max_duration,
        ..FakeMcuShared::default()
    };
    let callbacks = Arc::new(std::sync::Mutex::new(Vec::new()));
    let mcu: Mcu = Arc::new(FakeMcu {
        printer: printer.clone() as Printer,
        shared: shared.clone(),
        callbacks: callbacks.clone(),
    });
    let pins: Arc<FakePins> = Arc::new(FakePins {
        mcu: mcu.clone(),
        invert: if invert { 1 } else { 0 },
        last_desc: std::sync::Mutex::new(None),
        last_can_invert: std::sync::Mutex::new(None),
    });
    let gcode: Arc<FakeGCode> = Arc::new(FakeGCode {
        mux: std::sync::Mutex::new(Vec::new()),
        handler: std::sync::Mutex::new(None),
    });
    let toolhead: Arc<FakeToolhead> = Arc::new(FakeToolhead {
        callbacks: std::sync::Mutex::new(Vec::new()),
    });
    printer.add("pins", pins.clone() as ObjectRef);
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("toolhead", toolhead.clone() as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        section: format!("pwm_cycle_time {pin_name}"),
        opts: std::sync::Mutex::new(opts.to_vec()),
    });
    let obj = PrinterOutputPWMCycle::new(&config).expect("PrinterOutputPWMCycle::new");
    World { printer: printer.clone(), mcu, shared, callbacks, pins, gcode, toolhead, config, obj }
}

fn opt_list(opts: &[(&str, &str)]) -> Vec<(String, String)> {
    opts.iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

fn py_opts_expr(opts: &[(String, String)]) -> String {
    let items: Vec<String> = opts
        .iter()
        .map(|(k, v)| format!("('{k}', '{v}')"))
        .collect();
    format!("[{}]", items.join(", "))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn init_default_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0")]);
    let expected_make = py.exec(&format!(
        "make_world('my_pin', 0, 2.0, {})",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py make_world: {expected_make:?}");
    let world = make_world("my_pin", false, 2.0, &opts);

    for (label, py_expr) in [
        ("scale", "obj.scale"),
        ("last_value", "obj.last_value"),
        ("shutdown_value", "obj.shutdown_value"),
        ("default_cycle_time", "obj.default_cycle_time"),
        ("last_print_time", "obj.last_print_time"),
        ("last_cycle_time", "obj.last_cycle_time"),
    ] {
        let label = format!("init_default {label}");
        let expected = py.eval(py_expr);
        let actual = match label.as_str() {
            "init_default scale" => Value::float(world.obj.scale()),
            "init_default last_value" => Value::float(world.obj.last_value()),
            "init_default shutdown_value" => Value::float(world.obj.shutdown_value()),
            "init_default default_cycle_time" => Value::float(world.obj.default_cycle_time()),
            "init_default last_print_time" => Value::float(world.obj.last_print_time()),
            "init_default last_cycle_time" => Value::float(world.obj.last_cycle_time()),
            _ => unreachable!(),
        };
        check(&label, expected, actual);
    }

    let label = "init_default mux_registration";
    let expected = py.eval("gcode.mux");
    let mux = world.gcode.mux.lock().unwrap();
    let actual = Value::List(
        mux.iter()
            .map(|(c, k, v, d)| {
                Value::Tuple(vec![
                    Value::Str(c.clone()),
                    Value::Str(k.clone()),
                    match v {
                        Some(s) => Value::Str(s.clone()),
                        None => Value::Null,
                    },
                    match d {
                        Some(s) => Value::Str(s.clone()),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    );
    check(label, expected, actual);

    let label = "init_default lookup_pin";
    let expected = py.eval("(pins.last_desc, pins.last_opts)");
    let actual = Value::Tuple(vec![
        Value::Str(world.pins.last_desc.lock().unwrap().clone().unwrap()),
        Value::Tuple(vec![
            Value::Bool(world.pins.last_can_invert.lock().unwrap().unwrap()),
            Value::Bool(false),
            Value::Null,
        ]),
    ]);
    check(label, expected, actual);

    let label = "init_default callbacks_registered";
    let expected = py.eval("len(mcu.callbacks)");
    check(label, expected, Value::int(world.callbacks.lock().unwrap().len() as i64));

    let label = "init_default get_status";
    let expected = py.eval("obj.get_status(0.0)");
    check(label, expected, pyvalue_to_value(&world.obj.status_value()));
}

#[test]
fn init_custom_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[
        ("pin", "mcu:PA0"),
        ("scale", "2.0"),
        ("value", "1.0"),
        ("shutdown_value", "2.0"),
        ("cycle_time", "0.05"),
    ]);
    let expected_make = py.exec(&format!(
        "make_world('fan_pin', 0, 2.0, {})",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py make_world: {expected_make:?}");
    let world = make_world("fan_pin", false, 2.0, &opts);

    let expected = py.eval("obj.last_value");
    check("init_custom last_value", expected, Value::float(world.obj.last_value()));
    let expected = py.eval("obj.scale");
    check("init_custom scale", expected, Value::float(world.obj.scale()));
    let expected = py.eval("obj.default_cycle_time");
    check(
        "init_custom default_cycle_time",
        expected,
        Value::float(world.obj.default_cycle_time()),
    );
}

#[test]
fn build_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[
        ("pin", "mcu:PA0"),
        ("scale", "2.0"),
        ("value", "1.0"),
        ("shutdown_value", "2.0"),
        ("cycle_time", "0.05"),
    ]);
    let expected_make = py.exec(&format!(
        "make_world('fan_pin', 0, 2.0, {})\nmcu.callbacks[0]()",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py make_world+build: {expected_make:?}");
    let world = make_world("fan_pin", false, 2.0, &opts);
    {
        let mut cbs = world.callbacks.lock().unwrap();
        let r = cbs[0]();
        drop(cbs);
        check_ok("build_config callback", Ok(Value::Null), r);
    }

    let expected = py.eval("mcu.config_cmds");
    check("build_config config_cmds", expected, config_cmds_value(&world.shared));
    let expected = py.eval("mcu.commands");
    check("build_config lookup_command", expected, commands_value(&world.shared));
    let expected = py.eval("mcu.requested_slot");
    check(
        "build_config requested_slot",
        expected,
        Value::int(*world.shared.requested_slot.lock().unwrap()),
    );
    let expected = py.eval("obj.mcu_pin._oid");
    check(
        "build_config oid",
        expected,
        Value::int(world.obj.mcu_pin().oid()),
    );
    let expected = py.eval("obj.mcu_pin._last_clock");
    check(
        "build_config last_clock",
        expected,
        Value::int(world.obj.mcu_pin().last_clock()),
    );
    let expected = py.eval("obj.mcu_pin._cycle_ticks");
    check(
        "build_config cycle_ticks",
        expected,
        Value::int(world.obj.mcu_pin().cycle_ticks()),
    );
}

#[test]
fn build_config_invert_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[
        ("pin", "mcu:PA0"),
        ("value", "0.0"),
        ("shutdown_value", "0.0"),
    ]);
    let expected_make = py.exec(&format!(
        "make_world('led_pin', 1, 2.0, {})\nmcu.callbacks[0]()",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py make_world+build: {expected_make:?}");
    let world = make_world("led_pin", true, 2.0, &opts);
    {
        let mut cbs = world.callbacks.lock().unwrap();
        let r = cbs[0]();
        drop(cbs);
        check_ok("build_config invert callback", Ok(Value::Null), r);
    }
    let expected = py.eval("mcu.config_cmds");
    check(
        "build_config invert config_cmds",
        expected,
        config_cmds_value(&world.shared),
    );
}

#[test]
fn build_config_shutdown_error_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0"), ("shutdown_value", "0.5")]);
    let expected_make = py.exec(&format!(
        "make_world('bad_pin', 0, 2.0, {})",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py make_world: {expected_make:?}");
    let world = make_world("bad_pin", false, 2.0, &opts);
    let expected = py.eval("mcu.callbacks[0]()");
    let actual = {
        let mut cbs = world.callbacks.lock().unwrap();
        cbs[0]()
    };
    let err = actual.expect_err("build_config must fail for shutdown 0.5");
    check_err("build_config shutdown_error", expected, &err);
    // The failure happens before any command was queued on both sides.
    let expected = py.eval("mcu.config_cmds");
    check(
        "build_config shutdown_error config_cmds",
        expected,
        config_cmds_value(&world.shared),
    );
}

#[test]
fn build_config_cycle_too_large_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0"), ("cycle_time", "3000000.0")]);
    let expected_make = py.exec(&format!(
        "make_world('big_pin', 0, 10000000.0, {})",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py make_world: {expected_make:?}");
    let world = make_world("big_pin", false, 10000000.0, &opts);
    let expected = py.eval("mcu.callbacks[0]()");
    let actual = {
        let mut cbs = world.callbacks.lock().unwrap();
        cbs[0]()
    };
    let err = actual.expect_err("build_config must fail for large cycle_time");
    check_err("build_config cycle_too_large", expected, &err);
}

#[test]
fn set_pwm_cycle_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0"), ("cycle_time", "0.1")]);
    let expected_make = py.exec(&format!(
        "make_world('out_pin', 0, 2.0, {})\nmcu.callbacks[0]()\nmp = obj.mcu_pin",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py setup: {expected_make:?}");
    let world = make_world("out_pin", false, 2.0, &opts);
    {
        let mut cbs = world.callbacks.lock().unwrap();
        let r = cbs[0]();
        drop(cbs);
        check_ok("set_pwm_cycle setup build", Ok(Value::Null), r);
    }

    // (print_time, value, cycle_time) rows exercising on/off/scale and a
    // cycle-time change; final row exceeds the tick limit.
    let rows: [(f64, f64, f64); 4] = [
        (11.0, 0.0, 0.1),
        (12.0, 1.0, 0.1),
        (13.0, 0.5, 0.2),
        (14.0, 0.5, 3000000.0),
    ];
    for (i, (print_time, value, cycle_time)) in rows.iter().enumerate() {
        let label = format!("set_pwm_cycle[{i}] value={value} cycle={cycle_time}");
        let expected = py.eval(&format!(
            "mp.set_pwm_cycle({print_time:?}, {value:?}, {cycle_time:?})"
        ));
        let actual = world
            .obj
            .mcu_pin()
            .set_pwm_cycle(*print_time, *value, *cycle_time);
        match actual {
            Ok(()) => check(&label, expected, Value::Null),
            Err(e) => check_err(&label, expected, &e),
        }
        let expected_sends = py.eval("mcu.sends");
        check(&format!("{label} sends"), expected_sends, sends_value(&world.shared));
    }
}

#[test]
fn cmd_set_pin_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0"), ("scale", "2.0")]);
    let expected_make = py.exec(&format!(
        "make_world('out_pin', 0, 2.0, {})\nmcu.callbacks[0]()",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py setup: {expected_make:?}");
    let world = make_world("out_pin", false, 2.0, &opts);
    {
        let mut cbs = world.callbacks.lock().unwrap();
        let r = cbs[0]();
        drop(cbs);
        check_ok("cmd_set_pin setup build", Ok(Value::Null), r);
    }

    let label = "cmd_set_pin VALUE=1.5";
    let params = vec![
        ("VALUE".to_string(), "1.5".to_string()),
    ];
    let expected = py.eval("obj.cmd_SET_PIN(FakeGCmd('SET_PIN PIN=out_pin VALUE=1.5', [('VALUE', '1.5')]))");
    let gcmd = Arc::new(FakeGCmd {
        commandline: "SET_PIN PIN=out_pin VALUE=1.5".to_string(),
        params: std::sync::Mutex::new(params),
    }) as GCmd;
    let actual = world.obj.cmd_set_pin(&gcmd);
    match actual {
        Ok(()) => check(label, expected, Value::Null),
        Err(e) => check_err(label, expected, &e),
    }

    let label = "cmd_set_pin lookahead registered";
    let expected = py.eval("len(toolhead.callbacks)");
    check(
        label,
        expected,
        Value::int(world.toolhead.callbacks.lock().unwrap().len() as i64),
    );

    // Run the registered lookahead callback at print_time 11.0 on both sides.
    let label = "cmd_set_pin lookahead run";
    let expected = py.eval("toolhead.callbacks[0](11.0)");
    let actual = {
        let mut cbs = world.toolhead.callbacks.lock().unwrap();
        cbs[0](11.0)
    };
    match actual {
        Ok(()) => check(label, expected, Value::Null),
        Err(e) => check_err(label, expected, &e),
    }
    let expected_sends = py.eval("mcu.sends");
    check("cmd_set_pin sends", expected_sends, sends_value(&world.shared));
    let expected_status = py.eval("obj.get_status(0.0)");
    check(
        "cmd_set_pin get_status",
        expected_status,
        pyvalue_to_value(&world.obj.status_value()),
    );
}

#[test]
fn cmd_set_pin_error_paths_match_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0"), ("scale", "2.0")]);
    let cases: [(&str, Vec<(&str, &str)>); 4] = [
        ("missing VALUE", vec![]),
        ("VALUE too large", vec![("VALUE", "3.0")]),
        ("CYCLE_TIME zero", vec![("VALUE", "1.0"), ("CYCLE_TIME", "0.0")]),
        (
            "CYCLE_TIME too large",
            vec![("VALUE", "1.0"), ("CYCLE_TIME", "3.0")],
        ),
    ];
    for (name, params) in cases {
        let label = format!("cmd_set_pin_error {name}");
        let py_params: Vec<String> = params
            .iter()
            .map(|(k, v)| format!("('{k}', '{v}')"))
            .collect();
        let py_expr = format!(
            "make_world('err_pin', 0, 2.0, {}) and obj.cmd_SET_PIN(FakeGCmd('SET_PIN {}', [{}]))",
            py_opts_expr(&opts),
            params
                .iter()
                .map(|(k, v)| format!("{k}={v}"))
                .collect::<Vec<_>>()
                .join(" "),
            py_params.join(", "),
        );
        let expected = py.eval(&py_expr);
        let world = make_world("err_pin", false, 2.0, &opts);
        let params_owned: Vec<(String, String)> = params
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        let gcmd = Arc::new(FakeGCmd {
            commandline: format!(
                "SET_PIN {}",
                params
                    .iter()
                    .map(|(k, v)| format!("{k}={v}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            ),
            params: std::sync::Mutex::new(params_owned),
        }) as GCmd;
        let actual = world.obj.cmd_set_pin(&gcmd);
        match actual {
            Ok(()) => check(&label, expected, Value::Null),
            Err(e) => check_err(&label, expected, &e),
        }
    }
}

#[test]
fn config_error_paths_match_python() {
    let Some(mut py) = oracle() else { return };
    let cases: [(&str, Vec<(&str, &str)>, f64); 4] = [
        ("scale zero", vec![("pin", "mcu:PA0"), ("scale", "0.0")], 2.0),
        ("missing pin", vec![("scale", "1.0")], 2.0),
        (
            "value above scale",
            vec![("pin", "mcu:PA0"), ("scale", "2.0"), ("value", "3.0")],
            2.0,
        ),
        (
            "cycle_time above max",
            vec![("pin", "mcu:PA0"), ("cycle_time", "3.0")],
            2.0,
        ),
    ];
    for (name, opts_pairs, max_duration) in cases {
        let label = format!("config_error {name}");
        let opts = opt_list(&opts_pairs);
        let py_expr = format!(
            "make_world('my_pin', 0, {max_duration:?}, {})",
            py_opts_expr(&opts)
        );
        let expected = py.eval(&py_expr);
        let printer = FakePrinter::new();
        let shared = FakeMcuShared {
            oid: 3,
            max_duration,
            ..FakeMcuShared::default()
        };
        let callbacks = Arc::new(std::sync::Mutex::new(Vec::new()));
        let mcu: Mcu = Arc::new(FakeMcu {
            printer: printer.clone() as Printer,
            shared,
            callbacks,
        });
        let pins: Arc<FakePins> = Arc::new(FakePins {
            mcu: mcu.clone(),
            invert: 0,
            last_desc: std::sync::Mutex::new(None),
            last_can_invert: std::sync::Mutex::new(None),
        });
        let gcode: Arc<FakeGCode> = Arc::new(FakeGCode {
            mux: std::sync::Mutex::new(Vec::new()),
            handler: std::sync::Mutex::new(None),
        });
        let toolhead: Arc<FakeToolhead> = Arc::new(FakeToolhead {
            callbacks: std::sync::Mutex::new(Vec::new()),
        });
        printer.add("pins", pins.clone() as ObjectRef);
        printer.add("gcode", gcode.clone() as ObjectRef);
        printer.add("toolhead", toolhead.clone() as ObjectRef);
        let config: Config = Arc::new(FakeConfig {
            printer: printer.clone() as Printer,
            section: "pwm_cycle_time my_pin".to_string(),
            opts: std::sync::Mutex::new(opts),
        });
        let actual = PrinterOutputPWMCycle::new(&config);
        match actual {
            Ok(_) => check(&label, expected, Value::Null),
            Err(e) => check_err(&label, expected, &e),
        }
    }
}

#[test]
fn set_pin_dedup_and_clamp_matches_python() {
    let Some(mut py) = oracle() else { return };
    let opts = opt_list(&[("pin", "mcu:PA0"), ("cycle_time", "0.1")]);
    let expected_make = py.exec(&format!(
        "make_world('out_pin', 0, 2.0, {})\nmcu.callbacks[0]()",
        py_opts_expr(&opts)
    ));
    assert!(expected_make.is_ok(), "py setup: {expected_make:?}");
    let world = make_world("out_pin", false, 2.0, &opts);
    {
        let mut cbs = world.callbacks.lock().unwrap();
        let r = cbs[0]();
        drop(cbs);
        check_ok("set_pin setup build", Ok(Value::Null), r);
    }

    // (print_time, value, cycle_time): a no-op (same value/cycle as the
    // initial state), an ordinary update, an update whose print_time is
    // clamped to last_print_time + min_schedule_time, and a second no-op.
    let rows: [(f64, f64, f64); 4] = [
        (11.0, 0.0, 0.1),
        (12.0, 0.5, 0.1),
        (12.0, 0.5, 0.2),
        (12.0, 0.5, 0.2),
    ];
    for (i, (print_time, value, cycle_time)) in rows.iter().enumerate() {
        let label = format!("set_pin[{i}] print_time={print_time} value={value} cycle={cycle_time}");
        let expected = py.eval(&format!(
            "obj._set_pin({print_time:?}, {value:?}, {cycle_time:?})"
        ));
        let actual = world.obj.set_pin(*print_time, *value, *cycle_time);
        match actual {
            Ok(()) => check(&label, expected, Value::Null),
            Err(e) => check_err(&label, expected, &e),
        }
        let expected_sends = py.eval("mcu.sends");
        check(&format!("{label} sends"), expected_sends, sends_value(&world.shared));
        let expected_state =
            py.eval("(obj.last_value, obj.last_cycle_time, obj.last_print_time)");
        let state = Value::Tuple(vec![
            Value::float(world.obj.last_value()),
            Value::float(world.obj.last_cycle_time()),
            Value::float(world.obj.last_print_time()),
        ]);
        check(&format!("{label} state"), expected_state, state);
    }
}
