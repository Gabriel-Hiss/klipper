//! Parity tests for `extras::angle` (port of `klippy/extras/angle.py`).
//!
//! Expected values come live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + `klippy/`). Both sides build the object
//! the same way: the real `bus.MCU_SPI` / `bus.MCU_SPI_from_config` /
//! `bulk_sensor.BulkDataQueue` / `bulk_sensor.BatchBulkHelper` on the
//! Python side and `crate::bus` / `crate::bulk_sensor` here, driven by
//! fakes of the `klippy_api` contract traits plus a fake `force_move`
//! and TMC driver implementing the owner traits. Compared: MCU config
//! commands, query/command sends, SPI transfers, G-code
//! `respond_info` output, `configfile` writes, `get_status` dicts,
//! returned values (floats bit-for-bit) and exception class plus
//! message.
//!
//! `AngleCalibration.load_calibration` needs `numpy.linalg.lstsq` on
//! the Python side; those tests skip with a message when the oracle
//! interpreter has no numpy. There are no recordings under
//! `tests/fixtures/recordings/` for this unit.

use std::any::Any;
use std::collections::VecDeque;
use kos::sync::Mutex;
use std::sync::Arc;

use extras::angle::calibration::{lstsq_min_norm, AngleCalibration};
use extras::angle::helpers::{
    HelperA1333, HelperAS5047D, HelperMT6816, HelperMT6826S, HelperTLE5012B,
};
use extras::angle::{
    Angle, AngleApi, SensorHelperApi, SensorKind, ANGLE_BITS, BATCH_UPDATES, BYTES_PER_SAMPLE,
    CALIBRATION_BITS, MIN_MSG_TIME, SAMPLE_PERIOD, SAMPLES_PER_BLOCK, TCODE_ERROR,
};
use extras::bus::mcu_spi_from_config;
use extras::force_move::ForceMoveApi;
use extras::tmc::TMCCommandHelperApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetIntOpts as GGetIntOpts, GetOpts,
    GetParser, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueryWrapperApi,
    CommandWrapper, CommandWrapperApi, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, QuerySendOpts, SendOpts,
    SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{McuStepperApi, Stepper, StepperKinematics, Trapq};
use klippy_api::toolhead::{
    HomingTargetApi, Kinematics, KinematicsApi, MoveApi, ToolHeadApi,
};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.angle as angle
import extras.bus as bus

class CommandError(Exception):
    pass

_sentinel = object()

def capture(fn):
    try:
        return ['ok', fn()]
    except Exception as e:
        return ['error', type(e).__name__, str(e)]

class FakeReactor:
    NEVER = 9999999999999999.0
    def __init__(self):
        self.now = 100.0
        self.timers = []
    def monotonic(self):
        return self.now
    def register_timer(self, cb, waketime):
        self.timers.append((cb, waketime))
        return len(self.timers) - 1
    def unregister_timer(self, timer):
        pass

class FakePrinter:
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self._handlers = {}
        self.sections = {}
        self.settings = {}
        self.reactor = FakeReactor()
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        if name in self.objects:
            raise configparser.Error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self._handlers.setdefault(event, []).append(cb)
    def get_reactor(self):
        return self.reactor
    def send_event(self, event, *params):
        return [cb(*params) for cb in self._handlers.get(event, [])]
    def section_options(self, section):
        return self.sections.get(section, {})
    def set_section(self, section, options):
        self.sections[section] = options
    def get_status_settings(self):
        return self.settings

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, options):
        self._printer = printer
        self._section = section
        self._options = options or {}
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._section
    def _missing(self, option):
        raise configparser.Error(
            "Option '%s' in section '%s' must be specified" % (option, self._section))
    def _parse_err(self, option):
        raise configparser.Error(
            "Unable to parse option '%s' in section '%s'" % (option, self._section))
    def get(self, option, default=_sentinel, **kw):
        if option not in self._options:
            if default is not _sentinel:
                return default
            self._missing(option)
        return self._options[option]
    def getint(self, option, default=_sentinel, minval=None, maxval=None, note_valid=True, **kw):
        if option not in self._options:
            if default is not _sentinel:
                return default
            self._missing(option)
        try:
            v = int(self._options[option], 10)
        except Exception:
            self._parse_err(option)
        if minval is not None and v < minval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        return v
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True, **kw):
        if option not in self._options:
            if default is not _sentinel:
                return default
            self._missing(option)
        try:
            v = float(self._options[option])
        except Exception:
            self._parse_err(option)
        if minval is not None and v < minval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        if above is not None and v <= above:
            raise configparser.Error(
                "Option '%s' in section '%s' must be above %s" % (option, self._section, above))
        if below is not None and v >= below:
            raise configparser.Error(
                "Option '%s' in section '%s' must be below %s" % (option, self._section, below))
        return v
    def getchoice(self, option, choices, default=_sentinel, **kw):
        c = self.get(option, default)
        if c not in choices:
            raise configparser.Error(
                "Choice '%s' for option '%s' in section '%s' is not a valid choice"
                % (c, option, self._section))
        return choices[c]
    def getsection(self, section):
        return FakeConfig(self._printer, section, self._printer.section_options(section))
    def has_section(self, section):
        return section in self._printer.sections

def norm_args(data):
    out = []
    for v in data:
        if isinstance(v, (bytes, bytearray, list, tuple)):
            out.append([int(b) for b in v])
        else:
            out.append(int(v))
    return out

class FakeCommandWrapper:
    def __init__(self, fmt):
        self.fmt = fmt
        self.sends = []
    def send(self, data, minclock=0, reqclock=0):
        self.sends.append(norm_args(data))
    def send_wait_ack(self, data, minclock=0, reqclock=0):
        self.sends.append(norm_args(data))

class FakeQueryWrapper:
    def __init__(self, fmt, respfmt):
        self.fmt = fmt
        self.respfmt = respfmt
        self.script = []
        self.calls = []
    def send(self, data, minclock=0, reqclock=0, retry=True):
        self.calls.append(norm_args(data))
        if self.script:
            return self.script.pop(0)
        return {'response': b'\x00' * 6, 'clock': 0}

class FakeMcu:
    def __init__(self, freq=25000000.0):
        self.freq = freq
        self.printer = None
        self.oid_next = 6
        self.config_cmds = []
        self.config_cbs = []
        self.commands = []
        self.queries = []
        self.serial_cbs = []
    def create_oid(self):
        self.oid_next += 1
        return self.oid_next
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append([cmd, is_init, on_restart])
    def register_config_callback(self, cb):
        self.config_cbs.append(cb)
    def alloc_command_queue(self):
        return 1
    def seconds_to_clock(self, time):
        return int(time * self.freq)
    def estimated_print_time(self, systime):
        return 101.0
    def print_time_to_clock(self, print_time):
        return int(print_time * self.freq)
    def clock_to_print_time(self, clock):
        return clock / self.freq
    def clock32_to_clock64(self, clock32):
        return clock32
    def get_enumerations(self):
        return {}
    def get_constants(self):
        return {}
    def get_name(self):
        return 'mcu'
    def get_printer(self):
        return self.printer
    def lookup_command(self, msgformat, cq=None):
        w = FakeCommandWrapper(msgformat)
        self.commands.append(w)
        return w
    def try_lookup_command(self, msgformat):
        return True
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None):
        w = FakeQueryWrapper(msgformat, respformat)
        self.queries.append(w)
        return w
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_cbs.append(cb)
        return object()

class FakePins:
    error = configparser.Error
    def __init__(self, mcu):
        self.mcu = mcu
        self.lookups = []
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        self.lookups.append([pin_desc, can_invert, can_pullup, share_type])
        return {'chip': self.mcu, 'chip_name': 'mcu', 'pin': pin_desc,
                'invert': 0, 'pullup': 0, 'share_type': share_type}
    def reset_pin_sharing(self, params):
        pass

class FakeWebhooks:
    def __init__(self):
        self.endpoints = []
    def register_mux_endpoint(self, path, key, value, callback):
        self.endpoints.append([path, key, value])

class FakeStepper:
    def __init__(self, name='stepper_x', step_dist=0.01, past=100, commanded=5.0):
        self._name = name
        self._step_dist = step_dist
        self._past = past
        self._commanded = commanded
    def get_name(self, short=False):
        return self._name
    def get_step_dist(self):
        return self._step_dist
    def get_past_mcu_position(self, print_time):
        return self._past
    def get_mcu_position(self, cmd_pos=None):
        if cmd_pos is None:
            cmd_pos = self._commanded
        mcu_pos = cmd_pos / self._step_dist
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist

class FakeToolHead:
    def __init__(self):
        self.now = 100.0
        self.dwells = []
    def get_last_move_time(self):
        return self.now
    def dwell(self, delay):
        self.dwells.append(delay)
        self.now += delay
    def wait_moves(self):
        pass

class FakeForceMove:
    def __init__(self, printer, stepper=None):
        self.printer = printer
        self.stepper = stepper or FakeStepper()
        self.moves = []
        self.hook = None
    def lookup_stepper(self, name):
        if name is None or name != self.stepper.get_name():
            raise configparser.Error("Unknown stepper %s" % (name,))
        return self.stepper
    def manual_move(self, stepper, dist, speed, accel=0.):
        self.moves.append([dist, speed, accel])
        if self.hook is not None:
            self.hook(len(self.moves))

class FakeTmc:
    def __init__(self, phase_offset=10, phases=256):
        self.phase_offset = phase_offset
        self.phases = phases
    def get_phase_offset(self):
        return self.phase_offset, self.phases

class FakeConfigFile:
    def __init__(self, printer):
        self.printer = printer
        self.set_calls = []
        self.removed = []
    def get_status(self, eventtime):
        return {'settings': self.printer.get_status_settings()}
    def set(self, section, option, value):
        self.set_calls.append([section, option, str(value)])
    def remove_section(self, section):
        self.removed.append(section)

class FakeGCode:
    def __init__(self):
        self.mux = []
        self.responses = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append([cmd, key, value, desc or ''])
    def respond_info(self, msg, log=True):
        self.responses.append(msg)

class FakeGCommand:
    def __init__(self, gcode, cmdline, params):
        self.gcode = gcode
        self.cmdline = cmdline
        self.params = params
    sentinel = _sentinel
    def get(self, name, default=sentinel, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self.params.get(name)
        if value is None:
            if default is self.sentinel:
                raise Exception("Error on '%s': missing %s" % (self.cmdline, name))
            return default
        try:
            value = parser(value)
        except Exception:
            raise Exception("Error on '%s': unable to parse %s" % (self.cmdline, value))
        if minval is not None and value < minval:
            raise Exception("Error on '%s': %s must have minimum of %s" % (self.cmdline, name, minval))
        if maxval is not None and value > maxval:
            raise Exception("Error on '%s': %s must have maximum of %s" % (self.cmdline, name, maxval))
        if above is not None and value <= above:
            raise Exception("Error on '%s': %s must be above %s" % (self.cmdline, name, above))
        if below is not None and value >= below:
            raise Exception("Error on '%s': %s must be below %s" % (self.cmdline, name, below))
        return value
    def respond_info(self, msg, log=True):
        self.gcode.responses.append(msg)
    def get_command(self):
        return self.cmdline.split()[0]

class FakeAngleClient:
    def __init__(self):
        self.clients = []
    def add_client(self, cb):
        self.clients.append(cb)
    def deliver(self, msg):
        for cb in self.clients:
            cb(msg)

def mk_env(section, options, freq=25000000.0):
    p = FakePrinter()
    mcu = FakeMcu(freq)
    mcu.printer = p
    p.objects['mcu'] = mcu
    p.objects['pins'] = FakePins(mcu)
    p.objects['webhooks'] = FakeWebhooks()
    p.objects['gcode'] = FakeGCode()
    cfg = FakeConfig(p, section, options)
    return p, mcu, cfg

def run_cbs(mcu):
    for cb in mcu.config_cbs:
        cb()

def script_query(mcu, prefix, script):
    for w in mcu.queries:
        if w.fmt.startswith(prefix):
            w.script.extend(script)
            return
    raise Exception('no query wrapper for ' + prefix)

def query_calls(mcu, prefix):
    out = []
    for w in mcu.queries:
        if w.fmt.startswith(prefix):
            out.extend(w.calls)
    return out

def command_calls(mcu, prefix):
    out = []
    for w in mcu.commands:
        if w.fmt.startswith(prefix):
            out.extend(w.sends)
    return out

def angle_cmds(mcu):
    return [c for c in mcu.config_cmds
            if c[0].startswith('config_spi_angle') or c[0].startswith('query_spi_angle')]

def feed(mcu, raw):
    for cb in mcu.serial_cbs:
        for params in raw:
            cb(params)

def tle_responses(entries):
    # entries: (tag, msg bytes, r2, r3, r4, clock). `_send_spi` compares
    # resp[-1] with the CRC of msg[:2] + resp[2:4], so the CRC byte comes
    # from the reference implementation.
    names = {'t': 'spi_transfer', 'a': 'spi_angle_transfer'}
    out = {'spi_transfer': [], 'spi_angle_transfer': []}
    for tag, msg, r2, r3, r4, clock in entries:
        resp = bytearray([0, 0, r2, r3, r4, 0])
        resp[5] = angle.HelperTLE5012B._calc_crc(None, bytes(msg[:2]) + bytes([r2, r3]))
        out[names[tag]].append({'response': bytes(resp), 'clock': clock})
    return out
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------
type SectionList = Vec<(String, Vec<(String, String)>)>;
type MoveHook = Arc<dyn Fn(usize) + Send + Sync>;

#[derive(Clone)]
struct FakePrinter {
    reactor: Reactor,
    objects: Arc<Mutex<Vec<(String, ObjectRef)>>>,
    handlers: Arc<Mutex<Vec<(String, EventCallback)>>>,
    sections: Arc<Mutex<SectionList>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            reactor: Reactor::new(false),
            objects: Arc::new(Mutex::new(Vec::new())),
            handlers: Arc::new(Mutex::new(Vec::new())),
            sections: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn set_section(&self, section: &str, options: Vec<(String, String)>) {
        self.sections
            .lock()
            .push((section.to_string(), options));
    }

    fn add_object(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().push((name.to_string(), obj));
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock();
        if objects.iter().any(|(n, _)| n == name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        for (n, obj) in self.objects.lock().iter() {
            if n == name {
                return Ok(Some(obj.clone()));
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
        self.objects.lock().clone()
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
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers
            .lock()
            .push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let mut handlers = self.handlers.lock();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                results.push(cb(params)?);
            }
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

#[derive(Clone)]
struct FakeConfig {
    printer: Printer,
    section: String,
    options: Vec<(String, String)>,
    sections: Arc<Mutex<SectionList>>,
}

impl FakeConfig {
    fn get_opt(&self, option: &str) -> Option<&String> {
        self.options
            .iter()
            .find(|(n, _)| n == option)
            .map(|(_, v)| v)
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
        match self.get_opt(option).cloned() {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.get_opt(option).cloned() {
            Some(s) => {
                let v: i64 = s.parse().map_err(|_| {
                    KlippyError::config(format!(
                        "Unable to parse option '{option}' in section '{}'",
                        self.section
                    ))
                })?;
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.section
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.section
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.get_opt(option).cloned() {
            Some(s) => {
                let v: f64 = s.parse().map_err(|_| {
                    KlippyError::config(format!(
                        "Unable to parse option '{option}' in section '{}'",
                        self.section
                    ))
                })?;
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            pycompat::text::float_repr(minval)
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            pycompat::text::float_repr(maxval)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            pycompat::text::float_repr(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            pycompat::text::float_repr(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            },
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let raw: Option<String> = match (self.get_opt(option).cloned(), opts.default) {
            (Some(v), _) => Some(v),
            (None, Some(d)) => d.as_ref().map(|d| d.str()),
            (None, None) => {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                )));
            }
        };
        let c = raw.unwrap_or_default();
        if let PyValue::Dict(map) = choices {
            if let Some(mapped) = map.get(&PyValue::Str(c.clone())) {
                return Ok(mapped.clone());
            }
        }
        Err(KlippyError::config(format!(
            "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
            self.section
        )))
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
    fn getsection(&self, section: &str) -> Config {
        let mut found = Vec::new();
        for (sec, opts) in self.sections.lock().iter() {
            if sec == section {
                found = opts.clone();
            }
        }
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            options: found,
            sections: Arc::clone(&self.sections),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.lock().iter().any(|(sec, _)| sec == section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

#[derive(Clone)]
struct FakeCommand {
    sends: Arc<Mutex<Vec<Vec<PyValue>>>>,
    fmt: String,
}

impl CommandWrapperApi for FakeCommand {
    fn send(&self, data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        self.sends.lock().push(data);
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        self.sends.lock().push(data);
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        1
    }
}

#[derive(Clone)]
struct FakeQuery {
    script: Arc<Mutex<VecDeque<PyValue>>>,
    calls: Arc<Mutex<Vec<Vec<PyValue>>>>,
    fmt: String,
}

impl CommandQueryWrapperApi for FakeQuery {
    fn send(&self, data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        self.calls.lock().push(data);
        let mut script = self.script.lock();
        if let Some(v) = script.pop_front() {
            Ok(v)
        } else {
            let mut map = IndexMap::new();
            map.insert(
                PyValue::Str("response".to_string()),
                PyValue::Bytes(vec![0, 0, 0, 0, 0, 0]),
            );
            map.insert(
                PyValue::Str("clock".to_string()),
                PyValue::Int(PyInt::from_i64(0)),
            );
            Ok(PyValue::Dict(map))
        }
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        self.send(data, opts)
    }
}

#[derive(Clone)]
struct FakeMcu {
    printer: Printer,
    freq: f64,
    oid_next: Arc<Mutex<i64>>,
    config_cmds: Arc<Mutex<Vec<(String, bool, bool)>>>,
    config_cbs: Arc<Mutex<Vec<klippy_api::mcu::ConfigCallback>>>,
    commands: Arc<Mutex<Vec<FakeCommand>>>,
    queries: Arc<Mutex<Vec<FakeQuery>>>,
    serial_cbs: Arc<Mutex<Vec<SerialResponseCallback>>>,
}

impl FakeMcu {
    fn new(printer: Printer) -> FakeMcu {
        FakeMcu {
            printer,
            freq: 25_000_000.0,
            oid_next: Arc::new(Mutex::new(6)),
            config_cmds: Arc::new(Mutex::new(Vec::new())),
            config_cbs: Arc::new(Mutex::new(Vec::new())),
            commands: Arc::new(Mutex::new(Vec::new())),
            queries: Arc::new(Mutex::new(Vec::new())),
            serial_cbs: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn run_config_callbacks(&self) -> KResult<()> {
        let mut cbs = self.config_cbs.lock();
        for cb in cbs.iter_mut() {
            cb()?;
        }
        Ok(())
    }

    fn script(&self, prefix: &str, responses: Vec<PyValue>) {
        for query in self.queries.lock().iter() {
            if query.fmt.starts_with(prefix) {
                query.script.lock().extend(responses);
                return;
            }
        }
        panic!("no query wrapper for {prefix}");
    }

    fn query_calls(&self, prefix: &str) -> Vec<Vec<PyValue>> {
        let mut out = Vec::new();
        for query in self.queries.lock().iter() {
            if query.fmt.starts_with(prefix) {
                out.extend(query.calls.lock().iter().cloned());
            }
        }
        out
    }

    fn command_calls(&self, prefix: &str) -> Vec<Vec<PyValue>> {
        let mut out = Vec::new();
        for cmd in self.commands.lock().iter() {
            if cmd.fmt.starts_with(prefix) {
                out.extend(cmd.sends.lock().iter().cloned());
            }
        }
        out
    }

    fn angle_cmds(&self) -> Vec<(String, bool, bool)> {
        self.config_cmds
            .lock()
            .iter()
            .filter(|(cmd, _, _)| {
                cmd.starts_with("config_spi_angle") || cmd.starts_with("query_spi_angle")
            })
            .cloned()
            .collect()
    }

    fn feed(&self, raw: &[PyValue]) {
        for cb in self.serial_cbs.lock().iter() {
            for params in raw {
                cb(params.clone());
            }
        }
    }
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
        Err(KlippyError::config("setup_pin unsupported in fake"))
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut next = self.oid_next.lock();
        *next += 1;
        Ok(*next)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.config_cbs.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.config_cmds
            .lock()
            .push((cmd.to_string(), opts.is_init, opts.on_restart));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * self.freq) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        _cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<CommandWrapper> {
        let cmd = FakeCommand {
            sends: Arc::new(Mutex::new(Vec::new())),
            fmt: msgformat.to_string(),
        };
        self.commands.lock().push(cmd.clone());
        Ok(Arc::new(cmd))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        let query = FakeQuery {
            script: Arc::new(Mutex::new(VecDeque::new())),
            calls: Arc::new(Mutex::new(Vec::new())),
            fmt: msgformat.to_string(),
        };
        self.queries.lock().push(query.clone());
        Ok(Arc::new(query))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        klippy_api::mcu::CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.serial_cbs.lock().push(cb);
        struct Nop;
        impl klippy_api::mcu::AsyncResponseWrapperApi for Nop {
            fn unregister(&self) {}
        }
        Arc::new(Nop)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Ok(0.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * self.freq) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / self.freq
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        101.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::config("unsupported in fake"))
    }
}

#[derive(Clone)]
struct FakePins {
    mcu: Mcu,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("parse_pin unsupported in fake"))
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::config("setup_pin unsupported in fake"))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::config("get_pin_resolver unsupported in fake"))
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

#[derive(Clone)]
struct FakeWebhooks {
    endpoints: Arc<Mutex<Vec<(String, String, String)>>>,
}

impl Default for FakeWebhooks {
    fn default() -> FakeWebhooks {
        FakeWebhooks { endpoints: Arc::new(Mutex::new(Vec::new())) }
    }
}

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        Ok(())
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        self.endpoints.lock().push((
            path.to_string(),
            key.to_string(),
            value.to_string(),
        ));
        Ok(())
    }
    fn call_remote_method(
        &self,
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

#[derive(Clone)]
struct FakeStepper {
    name: String,
    step_dist: f64,
    past: i64,
    commanded: f64,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu unsupported in fake stepper")
    }
    fn get_name(&self, short: Option<bool>) -> String {
        match short {
            Some(true) => self
                .name
                .strip_prefix("stepper")
                .unwrap_or(&self.name)
                .to_string(),
            _ => self.name.clone(),
        }
    }
    fn get_oid(&self) -> i64 {
        1
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 {
        self.step_dist
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _coord: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        self.commanded
    }
    fn get_mcu_position(&self, cmd_pos: Option<f64>) -> i64 {
        let cmd = cmd_pos.unwrap_or(self.commanded);
        let mcu_pos = cmd / self.step_dist;
        if mcu_pos >= 0.0 {
            (mcu_pos + 0.5) as i64
        } else {
            (mcu_pos - 0.5) as i64
        }
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        self.past
    }
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        (mcu_pos as f64) * self.step_dist
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

klippy_api::printer_object!(FakeStepper; dyn McuStepperApi);

/// The `force_move` printer object (`crate::force_move::ForceMoveApi`).
struct FakeForceMove {
    stepper: Stepper,
    moves: Mutex<Vec<(f64, f64, f64)>>,
    hook: Mutex<Option<MoveHook>>,
}

impl ForceMoveApi for FakeForceMove {
    fn lookup_stepper(&self, name: Option<&str>) -> KResult<Stepper> {
        match name {
            Some(n) if n == self.stepper.get_name(None) => Ok(self.stepper.clone()),
            Some(n) => Err(KlippyError::config(format!("Unknown stepper {n}"))),
            None => Err(KlippyError::config("Unknown stepper None".to_string())),
        }
    }
    fn manual_move(
        &self,
        _stepper: &Stepper,
        dist: f64,
        speed: f64,
        accel: f64,
    ) -> KResult<()> {
        let count = {
            let mut moves = self.moves.lock();
            moves.push((dist, speed, accel));
            moves.len()
        };
        let hook = self.hook.lock().clone();
        if let Some(hook) = hook {
            hook(count);
        }
        Ok(())
    }
}

klippy_api::printer_object!(FakeForceMove; dyn ForceMoveApi);

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

struct FakeTmcObject {
    phase_offset: Option<i64>,
    phases: i64,
}

impl PrinterObject for FakeTmcObject {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);
}

impl TMCCommandHelperApi for FakeTmcObject {
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        (self.phase_offset, self.phases)
    }
}

#[derive(Clone)]
struct FakeConfigFile {
    set_calls: Arc<Mutex<Vec<(String, String, String)>>>,
    removed: Arc<Mutex<Vec<String>>>,
    settings: PyValue,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("fake configfile get_printer unused")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Err(KlippyError::config("unused"))
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::config("unused"))
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
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
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("settings".to_string()), self.settings.clone());
        PyValue::Dict(map)
    }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.set_calls.lock().push((
            section.to_string(),
            option.to_string(),
            value.to_string(),
        ));
    }
    fn remove_section(&self, section: &str) {
        self.removed.lock().push(section.to_string());
    }
}

type MuxLog = Vec<(String, String, Option<String>, String)>;

struct FakeGCode {
    mux: Arc<Mutex<MuxLog>>,
    responses: Arc<Mutex<Vec<String>>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            mux: Arc::new(Mutex::new(Vec::new())),
            responses: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

impl Clone for FakeGCode {
    fn clone(&self) -> FakeGCode {
        FakeGCode {
            mux: Arc::clone(&self.mux),
            responses: Arc::clone(&self.responses),
        }
    }
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
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.map(str::to_string),
            desc.unwrap_or("").to_string(),
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
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
    ) -> klippy_api::gcode::GCmd {
        Arc::new(FakeGCmd {
            gcode: self.clone(),
            commandline: commandline.to_string(),
            params: params.clone(),
            command: command.to_string(),
        })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, _log: bool) {
        self.responses.lock().push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

#[derive(Clone)]
struct FakeGCmd {
    gcode: FakeGCode,
    commandline: String,
    params: IndexMap<String, String>,
    command: String,
}

fn parse_int_auto_base(s: &str) -> Option<i64> {
    let t = s.trim();
    let (neg, digits) = match t.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, t.strip_prefix('+').unwrap_or(t)),
    };
    let v: i64 = if let Some(hex) = digits.strip_prefix("0x").or_else(|| digits.strip_prefix("0X")) {
        i64::from_str_radix(hex, 16).ok()?
    } else if let Some(oct) = digits.strip_prefix("0o").or_else(|| digits.strip_prefix("0O")) {
        i64::from_str_radix(oct, 8).ok()?
    } else if let Some(bin) = digits.strip_prefix("0b").or_else(|| digits.strip_prefix("0B")) {
        i64::from_str_radix(bin, 2).ok()?
    } else if digits.len() > 1 && digits.starts_with('0') {
        i64::from_str_radix(digits, 8).ok()?
    } else {
        digits.parse().ok()?
    };
    Some(if neg { -v } else { v })
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let value = self.params.get(name).cloned();
        let Some(value) = value else {
            return match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
            };
        };
        let parsed: PyValue = match opts.parser {
            GetParser::Str => PyValue::Str(value.clone()),
            GetParser::Int => match value.parse::<i64>() {
                Ok(v) => pyint(v),
                Err(_) => {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': unable to parse {value}",
                        self.commandline
                    )));
                }
            },
            GetParser::IntAutoBase => match parse_int_auto_base(&value) {
                Some(v) => pyint(v),
                None => {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': unable to parse {value}",
                        self.commandline
                    )));
                }
            },
            GetParser::Float => match value.parse::<f64>() {
                Ok(v) => PyValue::Float(v),
                Err(_) => {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': unable to parse {value}",
                        self.commandline
                    )));
                }
            },
        };
        let fv = extras::angle::pyvalue_to_f64(&parsed).unwrap_or(0.0);
        if let Some(minval) = &opts.minval {
            if fv < extras::angle::pyvalue_to_f64(minval).unwrap_or(0.0) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have minimum of {}",
                    self.commandline,
                    minval.str()
                )));
            }
        }
        if let Some(maxval) = &opts.maxval {
            if fv > extras::angle::pyvalue_to_f64(maxval).unwrap_or(0.0) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have maximum of {}",
                    self.commandline,
                    maxval.str()
                )));
            }
        }
        if let Some(above) = &opts.above {
            if fv <= extras::angle::pyvalue_to_f64(above).unwrap_or(0.0) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must be above {}",
                    self.commandline,
                    above.str()
                )));
            }
        }
        if let Some(below) = &opts.below {
            if fv >= extras::angle::pyvalue_to_f64(below).unwrap_or(0.0) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must be below {}",
                    self.commandline,
                    below.str()
                )));
            }
        }
        Ok(parsed)
    }
    fn get_int(&self, name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        let v = self.get(
            name,
            GetOpts {
                default: None,
                parser: GetParser::Int,
                minval: None,
                maxval: None,
                above: None,
                below: None,
            },
        )?;
        Ok(extras::angle::pyvalue_to_i64(&v))
    }
    fn get_float(&self, _name: &str, _opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
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
    fn respond_info(&self, msg: &str, _log: bool) {
        self.gcode.responses.lock().push(msg.to_string());
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

struct FakeTrapq;
impl klippy_api::stepper::TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct FakeKin;

impl KinematicsApi for FakeKin {
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Err(KlippyError::config("unused"))
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _clear_axes: &str) {}
}

#[derive(Clone)]
struct FakeToolHead {
    now: Arc<Mutex<f64>>,
    dwells: Arc<Mutex<Vec<f64>>>,
}

impl FakeToolHead {
    fn new() -> FakeToolHead {
        FakeToolHead {
            now: Arc::new(Mutex::new(100.0)),
            dwells: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKin)
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        *self.now.lock()
    }
    fn dwell(&self, delay: f64) {
        self.dwells.lock().push(delay);
        *self.now.lock() += delay;
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<f64>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: Option<f64>,
        min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (
            max_velocity.unwrap_or(0.0),
            max_accel.unwrap_or(0.0),
            square_corner_velocity.unwrap_or(0.0),
            min_cruise_ratio.unwrap_or(0.0),
        )
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder unused in fake toolhead")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) {}
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory unused in fake toolhead")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

/// The `Angle` printer object as `AngleCalibration` reaches it.
#[derive(Clone)]
struct FakeAngleClient {
    clients: Arc<Mutex<Vec<extras::bulk_sensor::ClientCallback>>>,
}

impl Default for FakeAngleClient {
    fn default() -> FakeAngleClient {
        FakeAngleClient { clients: Arc::new(Mutex::new(Vec::new())) }
    }
}

impl FakeAngleClient {
    fn deliver(&self, msg: PyValue) {
        for cb in self.clients.lock().iter_mut() {
            let _ = cb(msg.clone());
        }
    }
}

struct FakeAngleObject(FakeAngleClient);

impl PrinterObject for FakeAngleObject {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn AngleApi);
}

impl AngleApi for FakeAngleObject {
    fn add_client(&self, client_cb: extras::bulk_sensor::ClientCallback) -> KResult<()> {
        self.0.clients.lock().push(client_cb);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("angle fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping angle oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "angle".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

/// Compares a Rust result against `capture(lambda: ...)` from the oracle.
fn check_call(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let Value::List(items) = &expected else {
        panic!("{label}: capture() did not return a list");
    };
    let tag = match items.first() {
        Some(Value::Str(tag)) => tag.clone(),
        _ => panic!("{label}: capture() returned no tag"),
    };
    if tag == "ok" {
        let value = items.get(1).cloned().unwrap_or(Value::Null);
        match actual {
            Ok(actual) => {
                if let Err(d) = assert_equal(&record(label), "ret", &value, &actual) {
                    panic!("{label}: {d}");
                }
            }
            Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
        }
        return;
    }
    let class = match items.get(1) {
        Some(Value::Str(s)) => s.clone(),
        _ => panic!("{label}: capture() returned no exception class"),
    };
    let message = match items.get(2) {
        Some(Value::Str(s)) => s.clone(),
        _ => String::new(),
    };
    match actual {
        Ok(v) => panic!("{label}: rust returned {v:?}, oracle raised {class}: {message}"),
        Err(e) => {
            assert_eq!(e.class_name(), class, "{label}: exception class");
            assert_eq!(e.message(), message, "{label}: exception message");
        }
    }
}

fn pyint(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn py_dict(pairs: &[(&str, PyValue)]) -> PyValue {
    let mut map = IndexMap::new();
    for (k, v) in pairs {
        map.insert(PyValue::Str(k.to_string()), v.clone());
    }
    PyValue::Dict(map)
}

fn bytes_value(bytes: &[u8]) -> PyValue {
    PyValue::Bytes(bytes.to_vec())
}

fn ints_of(v: &PyValue) -> Vec<i64> {
    match v {
        PyValue::Bytes(b) => b.iter().map(|x| *x as i64).collect(),
        PyValue::List(items) => items
            .iter()
            .map(|x| match x {
                PyValue::Int(n) => n.0.to_string().parse::<i64>().unwrap_or(0),
                PyValue::Bool(b) => i64::from(*b),
                _ => 0,
            })
            .collect(),
        PyValue::Int(n) => vec![n.0.to_string().parse::<i64>().unwrap_or(0)],
        _ => Vec::new(),
    }
}

/// One recorded send with every byte-string payload rendered as a list of
/// ints, matching the Python `norm_args` fake.
fn calls_value(calls: &[Vec<PyValue>]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|call| {
                Value::List(
                    call.iter()
                        .map(|v| match v {
                            PyValue::Bytes(_) | PyValue::List(_) => Value::List(
                                ints_of(v).into_iter().map(Value::int).collect(),
                            ),
                            other => Value::int(
                                extras::angle::pyvalue_to_i64(other).unwrap_or(0),
                            ),
                        })
                        .collect(),
                )
            })
            .collect(),
    )
}

fn value_to_py(v: &Value) -> Option<PyValue> {
    Some(match v {
        Value::Null => PyValue::None,
        Value::Bool(b) => PyValue::Bool(*b),
        Value::Int(i) => pyint(*i),
        Value::Float(f) => PyValue::Float(f.to_f64()?),
        Value::Bytes(b) => PyValue::Bytes(b.clone()),
        Value::Str(s) => PyValue::Str(s.clone()),
        Value::List(items) => {
            let conv: Option<Vec<PyValue>> = items.iter().map(value_to_py).collect();
            PyValue::List(conv?)
        }
        Value::Dict(pairs) => {
            let mut map = IndexMap::new();
            for (k, v) in pairs {
                map.insert(value_to_py(k)?, value_to_py(v)?);
            }
            PyValue::Dict(map)
        }
        _ => return None,
    })
}

fn scripts_from(value: &Value, tag: &str) -> Vec<PyValue> {
    let Value::Dict(pairs) = value else {
        panic!("tle_responses must return a dict");
    };
    for (k, v) in pairs {
        if let Value::Str(name) = k {
            if name == tag {
                let Value::List(items) = v else { panic!("script list") };
                return items.iter().filter_map(value_to_py).collect();
            }
        }
    }
    panic!("missing script tag {tag}");
}

fn pyvalue_scalar(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_scalar).collect()),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_scalar).collect()),
        PyValue::Dict(m) => {
            let entries: Vec<(Value, Value)> = m
                .iter()
                .map(|(k, v)| (pyvalue_scalar(k), pyvalue_scalar(v)))
                .collect();
            Value::Dict(entries)
        }
        PyValue::Host(h) => Value::ObjectId(format!("host{h}")),
    }
}

fn strings_value(items: &[String]) -> Value {
    Value::List(items.iter().map(|s| Value::str(s)).collect())
}

fn moves_value(moves: &[(f64, f64, f64)]) -> Value {
    Value::List(
        moves
            .iter()
            .map(|(d, s, a)| Value::List(vec![Value::float(*d), Value::float(*s), Value::float(*a)]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Shared environment builders.
// ---------------------------------------------------------------------------

struct RustEnv {
    fp: FakePrinter,
    printer: Printer,
    mcu: FakeMcu,
    gcode: FakeGCode,
    webhooks: FakeWebhooks,
}

fn rust_env(section: &str, options: &[(&str, &str)]) -> (RustEnv, Config) {
    let fp = FakePrinter::new();
    let printer: Printer = Arc::new(fp.clone());
    let gcode = FakeGCode::new();
    fp.add_object("gcode", Arc::new(gcode.clone()) as ObjectRef);
    let mcu = FakeMcu::new(printer.clone());
    let mcu_handle: Mcu = Arc::new(mcu.clone());
    let pins = FakePins { mcu: mcu_handle };
    fp.add_object("pins", Arc::new(pins) as ObjectRef);
    let webhooks = FakeWebhooks::default();
    fp.add_object("webhooks", Arc::new(webhooks.clone()) as ObjectRef);
    let config = Arc::new(FakeConfig {
        printer: printer.clone(),
        section: section.to_string(),
        options: options
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
        sections: Arc::clone(&fp.sections),
    }) as Config;
    (
        RustEnv { fp, printer, mcu, gcode, webhooks },
        config,
    )
}

fn rust_config(
    fp: &FakePrinter,
    printer: &Printer,
    section: &str,
    options: &[(&str, &str)],
) -> Config {
    Arc::new(FakeConfig {
        printer: printer.clone(),
        section: section.to_string(),
        options: options
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
        sections: Arc::clone(&fp.sections),
    })
}

fn py_env(section: &str, options: &str) -> String {
    format!("p, mcu, cfg = mk_env('{section}', {options})")
}

fn response_dict(bytes: &[u8], clock: i64) -> PyValue {
    let mut map = IndexMap::new();
    map.insert(
        PyValue::Str("response".to_string()),
        PyValue::Bytes(bytes.to_vec()),
    );
    map.insert(PyValue::Str("clock".to_string()), pyint(clock));
    PyValue::Dict(map)
}

fn fake_gcmd(
    gcode: &FakeGCode,
    commandline: &str,
    params: &[(&str, &str)],
) -> klippy_api::gcode::GCmd {
    let mut map = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    let command = commandline.split_whitespace().next().unwrap_or("").to_string();
    Arc::new(FakeGCmd {
        gcode: gcode.clone(),
        commandline: commandline.to_string(),
        params: map,
        command,
    })
}

// ---------------------------------------------------------------------------
// Constants and sensor tables.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    for (name, rust) in [
        ("MIN_MSG_TIME", Value::float(MIN_MSG_TIME)),
        ("TCODE_ERROR", Value::int(TCODE_ERROR)),
        ("CALIBRATION_BITS", Value::int(CALIBRATION_BITS)),
        ("ANGLE_BITS", Value::int(ANGLE_BITS)),
        ("BYTES_PER_SAMPLE", Value::int(BYTES_PER_SAMPLE)),
        ("SAMPLES_PER_BLOCK", Value::int(SAMPLES_PER_BLOCK)),
        ("SAMPLE_PERIOD", Value::float(SAMPLE_PERIOD)),
        ("BATCH_UPDATES", Value::float(BATCH_UPDATES)),
    ] {
        let expected = py.eval(&format!("angle.{name}")).expect("eval const");
        check(&format!("constants {name}"), Ok(expected), rust);
    }
}

#[test]
fn spi_mode_speed_match_python() {
    let Some(mut py) = oracle() else { return };
    for (cls, kind) in [
        ("HelperA1333", SensorKind::A1333),
        ("HelperAS5047D", SensorKind::AS5047D),
        ("HelperTLE5012B", SensorKind::TLE5012B),
        ("HelperMT6816", SensorKind::MT6816),
        ("HelperMT6826S", SensorKind::MT6826S),
    ] {
        let expected_mode = py.eval(&format!("angle.{cls}.SPI_MODE")).expect("eval mode");
        check(
            &format!("spi_mode {cls}"),
            Ok(expected_mode),
            Value::int(kind.spi_mode()),
        );
        let expected_speed = py.eval(&format!("angle.{cls}.SPI_SPEED")).expect("eval speed");
        check(
            &format!("spi_speed {cls}"),
            Ok(expected_speed),
            Value::int(kind.spi_speed()),
        );
    }
}

// ---------------------------------------------------------------------------
// Sensor helpers.
// ---------------------------------------------------------------------------

#[test]
fn a1333_and_as5047d_start_transfers() {
    let Some(mut py) = oracle() else { return };
    for (kind, class, mode, speed) in [
        ("a1333", "HelperA1333", 3i64, 10_000_000i64),
        ("as5047d", "HelperAS5047D", 1, 2_857_142),
    ] {
        py.exec(&format!(
            "{}\nspi = bus.MCU_SPI_from_config(cfg, {mode}, default_speed={speed})\n\
             helper = angle.{class}(cfg, spi, 7)\nrun_cbs(mcu)\nhelper.start()\n\
             res = query_calls(mcu, 'spi_transfer')",
            py_env(
                "angle testchip",
                &format!("{{'sensor_type': '{kind}', 'cs_pin': 'PA5'}}")
            )
        ))
        .expect("py start");
        let expected = py.eval("res").expect("transfers");

        let (env, config) = rust_env(
            "angle testchip",
            &[("sensor_type", kind), ("cs_pin", "PA5")],
        );
        let spi = mcu_spi_from_config(&config, mode, "cs_pin", speed, None, false)
            .expect("rust spi");
        match kind {
            "a1333" => {
                let helper = HelperA1333::new(&config, spi, 7);
                env.mcu.run_config_callbacks().expect("config cbs");
                helper.start().expect("start");
            }
            _ => {
                let helper = HelperAS5047D::new(&config, spi, 7);
                env.mcu.run_config_callbacks().expect("config cbs");
                helper.start().expect("start");
            }
        }
        let actual = env.mcu.query_calls("spi_transfer");
        check(&format!("{kind}_start"), Ok(expected), calls_value(&actual));
    }
}

#[test]
fn mt6816_start_is_noop_and_debug_read() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\nspi = bus.MCU_SPI_from_config(cfg, 3, default_speed=10000000)\n\
         helper = angle.HelperMT6816(cfg, spi, 7)\nrun_cbs(mcu)\nhelper.start()\n\
         start_calls = query_calls(mcu, 'spi_transfer')\n\
         script_query(mcu, 'spi_transfer', [{{'response': bytes([0, 0x34, 0x12]), 'clock': 0}}])\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_DEBUG_READ', {{}})\n\
         helper.cmd_ANGLE_DEBUG_READ(g)",
        py_env(
            "angle testchip",
            "{'sensor_type': 'mt6816', 'cs_pin': 'PA5'}"
        )
    ))
    .expect("py helper");
    let expected_calls = py.eval("start_calls").expect("start calls");
    let expected_resp = py.eval("p.objects['gcode'].responses").expect("responses");

    let (env, config) = rust_env(
        "angle testchip",
        &[("sensor_type", "mt6816"), ("cs_pin", "PA5")],
    );
    let spi = mcu_spi_from_config(&config, 3, "cs_pin", 10_000_000, None, false).expect("spi");
    let helper = HelperMT6816::new(&config, spi, 7).expect("helper");
    env.mcu.run_config_callbacks().expect("config cbs");
    helper.start().expect("start");
    let actual_start = env.mcu.query_calls("spi_transfer");
    env.mcu
        .script("spi_transfer", vec![response_dict(&[0, 0x34, 0x12], 0)]);
    let gcmd = fake_gcmd(&env.gcode, "ANGLE_DEBUG_READ", &[]);
    helper.cmd_angle_debug_read(&gcmd).expect("debug read");
    let responses = env.gcode.responses.lock().clone();
    check("mt6816_start", Ok(expected_calls), calls_value(&actual_start));
    check("mt6816_debug_read", Ok(expected_resp), strings_value(&responses));
}

#[test]
fn mt6826s_start_and_debug_reads() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\nspi = bus.MCU_SPI_from_config(cfg, 3, default_speed=10000000)\n\
         helper = angle.HelperMT6826S(cfg, spi, 7)\nrun_cbs(mcu)",
        py_env(
            "angle testchip",
            "{'sensor_type': 'mt6826s', 'cs_pin': 'PA5'}"
        )
    ))
    .expect("py helper");

    py.exec(
        "script_query(mcu, 'spi_transfer', [\
           {'response': bytes([0, 0, 0x21, 0, 0, 0]), 'clock': 0},\
           {'response': bytes([0, 0, 0, 0, 0, 0]), 'clock': 0}])\n\
         helper.start()\nstart_calls = query_calls(mcu, 'spi_transfer')",
    )
    .expect("py start");
    let expected_start = py.eval("start_calls").expect("calls");

    let (env, config) = rust_env(
        "angle testchip",
        &[("sensor_type", "mt6826s"), ("cs_pin", "PA5")],
    );
    let spi = mcu_spi_from_config(&config, 3, "cs_pin", 10_000_000, None, false).expect("spi");
    let helper = HelperMT6826S::new(&config, spi, 7).expect("helper");
    env.mcu.run_config_callbacks().expect("config cbs");
    env.mcu.script(
        "spi_transfer",
        vec![
            response_dict(&[0, 0, 0x21, 0, 0, 0], 0),
            response_dict(&[0, 0, 0, 0, 0, 0], 0),
        ],
    );
    helper.start().expect("start");
    let actual_start = env.mcu.query_calls("spi_transfer");
    check("mt6826s_start", Ok(expected_start), calls_value(&actual_start));

    for (reg, resp) in [
        ("0x003", [0u8, 0, 0x21, 0x42, 0x0a, 0x00]),
        ("0x00e", [0, 0, 0xa5, 0, 0, 0]),
        ("0x113", [0, 0, 0xc0, 0, 0, 0]),
        ("0x00d", [0, 0, 0x77, 0, 0, 0]),
    ] {
        let list = resp
            .iter()
            .map(|b| b.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!(
            "p.objects['gcode'].responses = []\n\
             script_query(mcu, 'spi_transfer', [{{'response': bytes([{list}]), 'clock': 0}}])\n\
             g = FakeGCommand(p.objects['gcode'], 'ANGLE_DEBUG_READ REG={reg}', {{'REG': '{reg}'}})\n\
             helper.cmd_ANGLE_DEBUG_READ(g)\nres = p.objects['gcode'].responses"
        ))
        .expect("py read");
        let expected = py.eval("res").expect("responses");
        env.mcu.script("spi_transfer", vec![response_dict(&resp, 0)]);
        env.gcode.responses.lock().clear();
        let gcmd = fake_gcmd(&env.gcode, "ANGLE_DEBUG_READ", &[("REG", reg)]);
        helper.cmd_angle_debug_read(&gcmd).expect("debug read");
        let responses = env.gcode.responses.lock().clone();
        check(
            &format!("mt6826s_debug_read {reg}"),
            Ok(expected),
            strings_value(&responses),
        );
    }
}

#[test]
fn mt6826s_chip_calibrate() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '16'}})\n\
         p.settings = {{'stepper_x': {{'microsteps': 16, 'full_steps_per_rotation': 8}}}}\n\
         p.objects['configfile'] = FakeConfigFile(p)\n\
         p.objects['toolhead'] = FakeToolHead()\n\
         fm = FakeForceMove(p)\np.objects['force_move'] = fm\n\
         spi = bus.MCU_SPI_from_config(cfg, 3, default_speed=10000000)\n\
         helper = angle.HelperMT6826S(cfg, spi, 7)\nrun_cbs(mcu)\n\
         script_query(mcu, 'spi_transfer', [\
           {{'response': bytes([0, 0, 0x20, 0, 0, 0]), 'clock': 0}},\
           {{'response': bytes([0, 0, 0, 0, 0, 0]), 'clock': 0}},\
           {{'response': bytes([0, 0, 0xc0, 0, 0, 0]), 'clock': 0}}])\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_CHIP_CALIBRATE', {{}})\n\
         helper.cmd_ANGLE_CHIP_CALIBRATE(g)\n\
         res = {{'moves': fm.moves, 'responses': p.objects['gcode'].responses}}",
        py_env(
            "angle testchip",
            "{'sensor_type': 'mt6826s', 'cs_pin': 'PA5', 'stepper': 'stepper_x'}"
        )
    ))
    .expect("py calibrate");
    let expected_moves = py.eval("res['moves']").expect("moves");
    let expected_resp = py.eval("res['responses']").expect("responses");

    let (env, config) = rust_env(
        "angle testchip",
        &[
            ("sensor_type", "mt6826s"),
            ("cs_pin", "PA5"),
            ("stepper", "stepper_x"),
        ],
    );
    env.fp
        .set_section("stepper_x", vec![("microsteps".to_string(), "16".to_string())]);
    let configfile = Arc::new(FakeConfigFile {
        set_calls: Arc::new(Mutex::new(Vec::new())),
        removed: Arc::new(Mutex::new(Vec::new())),
        settings: py_dict(&[(
            "stepper_x",
            py_dict(&[
                ("microsteps", pyint(16)),
                ("full_steps_per_rotation", pyint(8)),
            ]),
        )]),
    });
    env.fp.add_object("configfile", configfile as ObjectRef);
    env.fp
        .add_object("toolhead", Arc::new(FakeToolHead::new()) as ObjectRef);
    let stepper = Arc::new(FakeStepper {
        name: "stepper_x".to_string(),
        step_dist: 0.01,
        past: 100,
        commanded: 5.0,
    }) as Stepper;
    let force_move = Arc::new(FakeForceMove {
        stepper,
        moves: Mutex::new(Vec::new()),
        hook: Mutex::new(None),
    });
    env.fp
        .add_object("force_move", force_move.clone() as ObjectRef);
    let spi = mcu_spi_from_config(&config, 3, "cs_pin", 10_000_000, None, false).expect("spi");
    let helper = HelperMT6826S::new(&config, spi, 7).expect("helper");
    env.mcu.run_config_callbacks().expect("config cbs");
    env.mcu.script(
        "spi_transfer",
        vec![
            response_dict(&[0, 0, 0x20, 0, 0, 0], 0),
            response_dict(&[0, 0, 0, 0, 0, 0], 0),
            response_dict(&[0, 0, 0xc0, 0, 0, 0], 0),
        ],
    );
    let gcmd = fake_gcmd(&env.gcode, "ANGLE_CHIP_CALIBRATE", &[]);
    helper.cmd_angle_chip_calibrate(&gcmd).expect("chip calibrate");
    let moves: Vec<(f64, f64, f64)> = force_move.moves.lock().clone();
    check(
        "mt6826s_chip_calibrate moves",
        Ok(expected_moves),
        moves_value(&moves),
    );
    let responses = env.gcode.responses.lock().clone();
    check(
        "mt6826s_chip_calibrate responses",
        Ok(expected_resp),
        strings_value(&responses),
    );
}

// ---------------------------------------------------------------------------
// TLE5012B.
// ---------------------------------------------------------------------------

/// TLE5012B responses whose CRC byte comes from the reference `_calc_crc`:
/// the entry list is rendered once as a Python literal and evaluated on
/// both sides.
type TleEntry = (char, Vec<u8>, u8, u8, u8, i64);

fn tle_literal(entries: &[TleEntry]) -> String {
    entries
        .iter()
        .map(|(tag, msg, r2, r3, r4, clock)| {
            let msg_list = msg
                .iter()
                .map(|b| b.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            format!("('{tag}', [{msg_list}], {r2}, {r3}, {r4}, {clock})")
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn tle_scripts(py: &mut pyoracle::Oracle, entries: &[TleEntry]) -> (Vec<PyValue>, Vec<PyValue>) {
    let literal = tle_literal(entries);
    let value = py
        .eval(&format!("tle_responses([{literal}])"))
        .expect("tle responses");
    (
        scripts_from(&value, "spi_transfer"),
        scripts_from(&value, "spi_angle_transfer"),
    )
}

fn tle_helper_env(py: &mut pyoracle::Oracle) -> (RustEnv, Config, Arc<HelperTLE5012B>) {
    py.exec(&format!(
        "{}\nspi = bus.MCU_SPI_from_config(cfg, 1, default_speed=4000000)\n\
         helper = angle.HelperTLE5012B(cfg, spi, 7)\nrun_cbs(mcu)",
        py_env(
            "angle testchip",
            "{'sensor_type': 'tle5012b', 'cs_pin': 'PA5'}"
        )
    ))
    .expect("py helper");
    let (env, config) = rust_env(
        "angle testchip",
        &[("sensor_type", "tle5012b"), ("cs_pin", "PA5")],
    );
    let spi = mcu_spi_from_config(&config, 1, "cs_pin", 4_000_000, None, false).expect("spi");
    let helper = HelperTLE5012B::new(&config, spi, 7).expect("helper");
    env.mcu.run_config_callbacks().expect("config cbs");
    (env, config, helper)
}

#[test]
fn tle5012b_calc_crc() {
    let Some(mut py) = oracle() else { return };
    let (env, _config, helper) = tle_helper_env(&mut py);
    let _ = &env;
    let tables: Vec<Vec<u8>> = vec![
        vec![],
        vec![0x12],
        vec![0x84, 0x42, 0x00, 0x11],
        vec![0xff, 0xfc, 0x00, 0x01, 0xde, 0xad, 0xbe, 0xef],
        (0..64u8).collect(),
    ];
    for data in tables {
        let list = data
            .iter()
            .map(|b| b.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py
            .eval(&format!("helper._calc_crc(bytes([{list}]))"))
            .expect("eval crc");
        check(
            &format!("tle5012b_calc_crc len={}", data.len()),
            Ok(expected),
            Value::int(helper.calc_crc(&data)),
        );
    }
}

#[test]
fn tle5012b_start_clock_and_temperature() {
    let Some(mut py) = oracle() else { return };
    let (env, _config, helper) = tle_helper_env(&mut py);
    let entries: Vec<TleEntry> = vec![
        ('t', vec![0x80, 0x01], 0x00, 0x00, 0x00, 0),
        // _mask_reg(0x06, 0xc003, 0x4000)
        ('t', vec![0xd0, 0x61], 0x0f, 0xff, 0x00, 0),
        ('t', vec![0x50, 0x61], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0x61], 0x4f, 0xfc, 0x00, 0),
        // _mask_reg(0x08, 0x0007, 0x0001)
        ('t', vec![0xd0, 0x81], 0x00, 0x00, 0x00, 0),
        ('t', vec![0x50, 0x81], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0x81], 0x00, 0x01, 0x00, 0),
        // _mask_reg(0x0e, 0x0003, 0x0000)
        ('t', vec![0xd0, 0xe1], 0x00, 0x00, 0x00, 0),
        ('t', vec![0x50, 0xe1], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0xe1], 0x00, 0x00, 0x00, 0),
        // _query_clock inside start() and inside update_clock()
        ('a', vec![0x84, 0x42], 0x12, 0x00, 0x34, 1000),
        ('a', vec![0x84, 0x42], 0x12, 0x00, 0x34, 2000),
    ];
    let literal = tle_literal(&entries);
    py.exec(&format!(
        "PY = tle_responses([{literal}])\n\
         script_query(mcu, 'spi_transfer', PY['spi_transfer'])\n\
         script_query(mcu, 'spi_angle_transfer', PY['spi_angle_transfer'])\n\
         helper.start()\n\
         res = {{'t': query_calls(mcu, 'spi_transfer'), 'a': query_calls(mcu, 'spi_angle_transfer'), \
                'temp': helper.last_temperature, 'tcode': list(helper.get_tcode_params())}}"
    ))
    .expect("py start");
    let expected_t = py.eval("res['t']").expect("t calls");
    let expected_a = py.eval("res['a']").expect("a calls");
    let expected_temp = py.eval("res['temp']").expect("temp");
    let expected_tcode = py.eval("res['tcode']").expect("tcode");

    let (t_script, a_script) = tle_scripts(&mut py, &entries);
    env.mcu.script("spi_transfer", t_script);
    env.mcu.script("spi_angle_transfer", a_script);
    helper.start().expect("start");
    let actual_t = env.mcu.query_calls("spi_transfer");
    let actual_a = env.mcu.query_calls("spi_angle_transfer");
    check("tle5012b_start transfers", Ok(expected_t), calls_value(&actual_t));
    check(
        "tle5012b_start clock queries",
        Ok(expected_a),
        calls_value(&actual_a),
    );
    let temp = helper.last_temperature().expect("temperature");
    check("tle5012b_start temperature", Ok(expected_temp), Value::float(temp));
    let (a, b, f) = helper.get_tcode_params().expect("tcode params");
    check(
        "tle5012b_start tcode params",
        Ok(expected_tcode),
        Value::List(vec![Value::int(a), Value::int(b), Value::float(f)]),
    );
}

#[test]
fn tle5012b_debug_read_write() {
    let Some(mut py) = oracle() else { return };
    let (env, _config, helper) = tle_helper_env(&mut py);
    // ANGLE_DEBUG_WRITE 0x06 0x4ffc: one write plus the verifying read.
    let entries: Vec<TleEntry> = vec![
        ('t', vec![0x50, 0x61], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0x61], 0x4f, 0xfc, 0x00, 0),
    ];
    let literal = tle_literal(&entries);
    py.exec(&format!(
        "PY = tle_responses([{literal}])\n\
         script_query(mcu, 'spi_transfer', PY['spi_transfer'])\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_DEBUG_WRITE REG=0x06 VAL=0x4ffc', \
                          {{'REG': '0x06', 'VAL': '0x4ffc'}})\n\
         r = capture(lambda: helper.cmd_ANGLE_DEBUG_WRITE(g))\n\
         res = {{'calls': query_calls(mcu, 'spi_transfer'), 'r': r}}"
    ))
    .expect("py write");
    let expected_write = py.eval("res['calls']").expect("calls");
    let expected_result = py.eval("res['r']").expect("result");

    let (t_script, _) = tle_scripts(&mut py, &entries);
    env.mcu.script("spi_transfer", t_script);
    let gcmd = fake_gcmd(
        &env.gcode,
        "ANGLE_DEBUG_WRITE REG=0x06 VAL=0x4ffc",
        &[("REG", "0x06"), ("VAL", "0x4ffc")],
    );
    let result = helper.cmd_angle_debug_write(&gcmd).map(|_| Value::Null);
    let actual_write = env.mcu.query_calls("spi_transfer");
    check(
        "tle5012b_debug_write transfers",
        Ok(expected_write),
        calls_value(&actual_write),
    );
    check_call("tle5012b_debug_write result", Ok(expected_result), result);

    // ANGLE_DEBUG_READ 0x06 reads the two register bytes.
    let entries: Vec<TleEntry> = vec![('t', vec![0xd0, 0x61], 0x12, 0x34, 0x00, 0)];
    let literal = tle_literal(&entries);
    py.exec(&format!(
        "PY = tle_responses([{literal}])\n\
         script_query(mcu, 'spi_transfer', PY['spi_transfer'])\n\
         p.objects['gcode'].responses = []\n\
         sent = len(query_calls(mcu, 'spi_transfer'))\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_DEBUG_READ REG=0x06', {{'REG': '0x06'}})\n\
         helper.cmd_ANGLE_DEBUG_READ(g)\n\
         res = {{'calls': query_calls(mcu, 'spi_transfer')[sent:], \
                'resp': p.objects['gcode'].responses}}"
    ))
    .expect("py read");
    let expected_calls = py.eval("res['calls']").expect("calls");
    let expected_resp = py.eval("res['resp']").expect("responses");
    let before = env.mcu.query_calls("spi_transfer").len();
    let (t_script, _) = tle_scripts(&mut py, &entries);
    env.mcu.script("spi_transfer", t_script);
    env.gcode.responses.lock().clear();
    let gcmd = fake_gcmd(&env.gcode, "ANGLE_DEBUG_READ", &[("REG", "0x06")]);
    helper.cmd_angle_debug_read(&gcmd).expect("read");
    let all_calls = env.mcu.query_calls("spi_transfer");
    let read_calls = &all_calls[before..];
    check(
        "tle5012b_debug_read transfers",
        Ok(expected_calls),
        calls_value(read_calls),
    );
    let responses = env.gcode.responses.lock().clone();
    check(
        "tle5012b_debug_read responses",
        Ok(expected_resp),
        strings_value(&responses),
    );
}

#[test]
fn tle5012b_write_failure() {
    let Some(mut py) = oracle() else { return };
    let (env, _config, helper) = tle_helper_env(&mut py);
    let mut entries: Vec<TleEntry> = Vec::new();
    for _ in 0..5 {
        entries.push(('t', vec![0x50, 0x61], 0x00, 0x00, 0x00, 0));
        entries.push(('t', vec![0xd0, 0x61], 0x00, 0x00, 0x00, 0));
    }
    let literal = tle_literal(&entries);
    py.exec(&format!(
        "PY = tle_responses([{literal}])\n\
         script_query(mcu, 'spi_transfer', PY['spi_transfer'])\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_DEBUG_WRITE REG=0x06 VAL=0x4ffc', \
                          {{'REG': '0x06', 'VAL': '0x4ffc'}})\n\
         res = capture(lambda: helper.cmd_ANGLE_DEBUG_WRITE(g))"
    ))
    .expect("py write");
    let expected = py.eval("res").expect("result");
    let (t_script, _) = tle_scripts(&mut py, &entries);
    env.mcu.script("spi_transfer", t_script);
    let gcmd = fake_gcmd(
        &env.gcode,
        "ANGLE_DEBUG_WRITE REG=0x06 VAL=0x4ffc",
        &[("REG", "0x06"), ("VAL", "0x4ffc")],
    );
    let result = helper.cmd_angle_debug_write(&gcmd).map(|_| Value::Null);
    check_call("tle5012b_write_failure", Ok(expected), result);
}

// ---------------------------------------------------------------------------
// AngleCalibration.
// ---------------------------------------------------------------------------

/// `AngleCalibration.__init__` imports numpy, so the reference cannot run
/// without it.
fn require_numpy(py: &mut pyoracle::Oracle) {
    py.exec("import numpy")
        .expect("the oracle interpreter needs numpy for AngleCalibration");
}

fn calibration_angles() -> Vec<f64> {
    (0..8).map(|i| (i as f64) * 8000.0 + 100.0).collect()
}

fn angles_csv(angles: &[f64]) -> String {
    angles.iter().map(|a| a.to_string()).collect::<Vec<_>>().join(",")
}

#[test]
fn calibration_load_table() {
    let Some(mut py) = oracle() else { return };
    require_numpy(&mut py);
    for reversed in [false, true] {
        let mut angles = calibration_angles();
        if reversed {
            angles.reverse();
        }
        let csv = angles_csv(&angles);
        py.exec(&format!(
            "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
             c = angle.AngleCalibration(FakeConfig(p, 'angle c', \
                 {{'stepper': 'stepper_x', 'calibrate': '{csv}'}}))\n\
             res = {{'table': c.calibration, 'phase': c.angle_phase_offset, \
                    'rev': c.calibration_reversed}}",
            py_env("angle c", "{}")
        ))
        .expect("py calibration");
        let expected_table = py.eval("res['table']").expect("table");
        let expected_phase = py.eval("res['phase']").expect("phase");
        let expected_rev = py.eval("res['rev']").expect("rev");

        let (env, _config) = rust_env("angle c", &[]);
        env.fp.set_section(
            "stepper_x",
            vec![("microsteps".to_string(), "256".to_string())],
        );
        let config = rust_config(
            &env.fp,
            &env.printer,
            "angle c",
            &[("stepper", "stepper_x"), ("calibrate", &csv)],
        );
        let cal = AngleCalibration::new(&config).expect("rust calibration");
        let table: Vec<Value> = cal
            .calibration_table()
            .iter()
            .map(|v| Value::int(*v))
            .collect();
        let label = if reversed {
            "calibration_load_reversed"
        } else {
            "calibration_load_table"
        };
        check(label, Ok(expected_table), Value::List(table));
        check(
            &format!("{label} phase"),
            Ok(expected_phase),
            Value::float(cal.angle_phase_offset()),
        );
        check(
            &format!("{label} reversed flag"),
            Ok(expected_rev),
            Value::Bool(cal.calibration_reversed()),
        );
    }
}

struct CalibrationEnv {
    env: RustEnv,
    config: Config,
    force_move: Arc<FakeForceMove>,
    configfile: Arc<FakeConfigFile>,
}

fn calibration_env(
    section: &str,
    csv: Option<&str>,
    microsteps: i64,
    full_steps: i64,
    tmc: Option<(Option<i64>, i64)>,
) -> CalibrationEnv {
    let mut opts: Vec<(&str, &str)> = vec![("stepper", "stepper_x")];
    if let Some(csv) = csv {
        opts.push(("calibrate", csv));
    }
    let (env, config) = rust_env(section, &opts);
    env.fp.set_section(
        "stepper_x",
        vec![("microsteps".to_string(), microsteps.to_string())],
    );
    let configfile = Arc::new(FakeConfigFile {
        set_calls: Arc::new(Mutex::new(Vec::new())),
        removed: Arc::new(Mutex::new(Vec::new())),
        settings: py_dict(&[(
            "stepper_x",
            py_dict(&[
                ("microsteps", pyint(microsteps)),
                ("full_steps_per_rotation", pyint(full_steps)),
            ]),
        )]),
    });
    env.fp
        .add_object("configfile", configfile.clone() as ObjectRef);
    let stepper = Arc::new(FakeStepper {
        name: "stepper_x".to_string(),
        step_dist: 0.01,
        past: 100,
        commanded: 5.0,
    }) as Stepper;
    let force_move = Arc::new(FakeForceMove {
        stepper,
        moves: Mutex::new(Vec::new()),
        hook: Mutex::new(None),
    });
    env.fp
        .add_object("force_move", force_move.clone() as ObjectRef);
    if let Some((offset, phases)) = tmc {
        env.fp.add_object(
            "tmc2209 stepper_x",
            Arc::new(FakeTmcObject { phase_offset: offset, phases }) as ObjectRef,
        );
    }
    env.fp
        .add_object("toolhead", Arc::new(FakeToolHead::new()) as ObjectRef);
    CalibrationEnv { env, config, force_move, configfile }
}

#[test]
fn calibration_apply_with_offset() {
    let Some(mut py) = oracle() else { return };
    require_numpy(&mut py);
    let csv = angles_csv(&calibration_angles());
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
         p.settings = {{'stepper_x': {{'microsteps': 256, 'full_steps_per_rotation': 256}}}}\n\
         p.objects['configfile'] = FakeConfigFile(p)\n\
         p.objects['force_move'] = FakeForceMove(p)\n\
         p.objects['tmc2209 stepper_x'] = FakeTmc(10, 256)\n\
         c = angle.AngleCalibration(FakeConfig(p, 'angle c', \
             {{'stepper': 'stepper_x', 'calibrate': '{csv}'}}))\n\
         c.connect()\n\
         samples = [(1.5, 100), (2.5, 5000), (3.5, 20000)]\n\
         res = c.apply_calibration(samples)\n\
         out = {{'samples': samples, 'res': res, 'offset': c.mcu_pos_offset}}\n\
         p.send_event('stepper:sync_mcu_position', p.objects['force_move'].stepper)\n\
         out['offset_after'] = c.mcu_pos_offset",
        py_env("angle c", "{}")
    ))
    .expect("py apply");
    let expected_samples = py.eval("out['samples']").expect("samples");
    let expected_res = py.eval("out['res']").expect("res");
    let expected_offset = py.eval("out['offset']").expect("offset");
    let expected_offset_after = py.eval("out['offset_after']").expect("offset after");

    let env = calibration_env("angle c", Some(&csv), 256, 256, Some((Some(10), 256)));
    let cal = AngleCalibration::new(&env.config).expect("rust calibration");
    cal.connect().expect("connect");
    let mut samples = vec![(1.5, 100), (2.5, 5000), (3.5, 20000)];
    let res = cal.apply_calibration(&mut samples).expect("apply");
    let samples_value = Value::List(
        samples
            .iter()
            .map(|(t, a)| Value::Tuple(vec![Value::float(*t), Value::int(*a)]))
            .collect(),
    );
    check("calibration_apply samples", Ok(expected_samples), samples_value);
    check(
        "calibration_apply result",
        Ok(expected_res),
        res.map(Value::float).unwrap_or(Value::Null),
    );
    check(
        "calibration_apply offset",
        Ok(expected_offset),
        cal.mcu_pos_offset().map(Value::float).unwrap_or(Value::Null),
    );
    env.env
        .printer
        .send_event(
            klippy_api::printer::EV_STEPPER_SYNC_MCU_POSITION,
            // `stepper.py` sends the stepper printer object itself.
            &[EventArg::Object(Arc::new(FakeStepper {
                name: "stepper_x".to_string(),
                step_dist: 0.01,
                past: 100,
                commanded: 5.0,
            }) as ObjectRef)],
        )
        .expect("sync mcu position event");
    check(
        "calibration_sync_mcu_position clears the offset",
        Ok(expected_offset_after),
        cal.mcu_pos_offset().map(Value::float).unwrap_or(Value::Null),
    );
}

#[test]
fn calibration_lookup_tmc_and_stepper_phase() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
         p.settings = {{'stepper_x': {{'microsteps': 256, 'full_steps_per_rotation': 256}}}}\n\
         p.objects['configfile'] = FakeConfigFile(p)\n\
         p.objects['force_move'] = FakeForceMove(p)\n\
         c = angle.AngleCalibration(FakeConfig(p, 'angle c', {{'stepper': 'stepper_x'}}))\n\
         missing = capture(lambda: c.lookup_tmc())\n\
         p.objects['tmc2209 stepper_x'] = FakeTmc(10, 256)\n\
         c.connect()\n\
         phase = c.get_stepper_phase()\n\
         p.objects['tmc2209 stepper_x'] = FakeTmc(None, 256)\n\
         c.connect()\n\
         unknown = capture(lambda: c.get_stepper_phase())",
        py_env("angle c", "{}")
    ))
    .expect("py phase");
    let expected_missing = py.eval("missing").expect("missing");
    let expected_phase = py.eval("phase").expect("phase");
    let expected_unknown = py.eval("unknown").expect("unknown");

    let env = calibration_env("angle c", None, 256, 256, None);
    let cal = AngleCalibration::new(&env.config).expect("calibration");
    check_call(
        "calibration_lookup_tmc_missing",
        Ok(expected_missing),
        cal.lookup_tmc().map(|_| Value::Null),
    );
    env.env.fp.add_object(
        "tmc2209 stepper_x",
        Arc::new(FakeTmcObject { phase_offset: Some(10), phases: 256 }) as ObjectRef,
    );
    cal.connect().expect("connect");
    check(
        "calibration_get_stepper_phase",
        Ok(expected_phase),
        Value::int(cal.get_stepper_phase().expect("phase")),
    );
    env.env.fp.add_object(
        "tmc2209 stepper_x",
        Arc::new(FakeTmcObject { phase_offset: None, phases: 256 }) as ObjectRef,
    );
    // The first registration stays; `lookup_object` returns the first match,
    // so replace the entry in place to model the reference reassignment.
    {
        let mut objects = env.env.fp.objects.lock();
        for (name, obj) in objects.iter_mut() {
            if name == "tmc2209 stepper_x" {
                *obj = Arc::new(FakeTmcObject { phase_offset: None, phases: 256 })
                    as ObjectRef;
            }
        }
    }
    cal.connect().expect("connect");
    check_call(
        "calibration_stepper_phase_unknown",
        Ok(expected_unknown),
        cal.get_stepper_phase().map(Value::int),
    );
}

#[test]
fn calibration_cmd_angle_calibrate() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '16'}})\n\
         p.settings = {{'stepper_x': {{'microsteps': 16, 'full_steps_per_rotation': 8}}}}\n\
         cf = FakeConfigFile(p)\np.objects['configfile'] = cf\n\
         th = FakeToolHead()\np.objects['toolhead'] = th\n\
         fm = FakeForceMove(p)\np.objects['force_move'] = fm\n\
         p.objects['tmc2209 stepper_x'] = FakeTmc(10, 256)\n\
         client = FakeAngleClient()\np.objects['angle c'] = client\n\
         c = angle.AngleCalibration(FakeConfig(p, 'angle c', {{'stepper': 'stepper_x'}}))\n\
         c.connect()\n\
         fm.hook = lambda n: client.deliver({{'data': [(th.now + 0.07, n)]}}) if n > 4 else None\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_CALIBRATE', {{}})\n\
         res = capture(lambda: c.cmd_ANGLE_CALIBRATE(g))\n\
         out = {{'moves': fm.moves, 'responses': p.objects['gcode'].responses, \
                'set': cf.set_calls, 'removed': cf.removed, 'table': c.calibration}}",
        py_env("angle c", "{}")
    ))
    .expect("py calibrate");
    let expected_result = py.eval("res").expect("result");
    let expected_moves = py.eval("out['moves']").expect("moves");
    let expected_resp = py.eval("out['responses']").expect("responses");
    let expected_set = py.eval("out['set']").expect("set");
    let expected_removed = py.eval("out['removed']").expect("removed");
    let expected_table = py.eval("out['table']").expect("table");

    let env = calibration_env("angle c", None, 16, 8, Some((Some(10), 256)));
    let client = FakeAngleClient::default();
    env.env.fp.add_object(
        "angle c",
        Arc::new(FakeAngleObject(client.clone())) as ObjectRef,
    );
    let toolhead_obj = env
        .env
        .fp
        .lookup_object("toolhead", LookupDefault::Raise)
        .expect("toolhead lookup")
        .expect("toolhead object");
    let toolhead =
        klippy_api::require_interface::<dyn ToolHeadApi>(&toolhead_obj, "toolhead")
            .expect("toolhead interface");
    {
        let client = client.clone();
        *env.force_move.hook.lock() = Some(Arc::new(move |n: usize| {
            if n > 4 {
                let now = toolhead.get_last_move_time();
                client.deliver(py_dict(&[(
                    "data",
                    PyValue::List(vec![PyValue::Tuple(vec![
                        PyValue::Float(now + 0.07),
                        pyint(n as i64),
                    ])]),
                )]));
            }
        }));
    }
    let cal = AngleCalibration::new(&env.config).expect("calibration");
    cal.connect().expect("connect");
    let gcmd = fake_gcmd(&env.env.gcode, "ANGLE_CALIBRATE", &[]);
    let result = cal.cmd_angle_calibrate(&gcmd).map(|_| Value::Null);
    check_call("calibration_cmd_angle_calibrate", Ok(expected_result), result);
    let moves: Vec<(f64, f64, f64)> = env.force_move.moves.lock().clone();
    check(
        "calibration_cmd_angle_calibrate moves",
        Ok(expected_moves),
        moves_value(&moves),
    );
    let responses = env.env.gcode.responses.lock().clone();
    check(
        "calibration_cmd_angle_calibrate responses",
        Ok(expected_resp),
        strings_value(&responses),
    );
    let set_calls = env.configfile.set_calls.lock().clone();
    check(
        "calibration_cmd_angle_calibrate set",
        Ok(expected_set),
        Value::List(
            set_calls
                .iter()
                .map(|(s, o, v)| Value::List(vec![Value::str(s), Value::str(o), Value::str(v)]))
                .collect(),
        ),
    );
    let removed = env.configfile.removed.lock().clone();
    check(
        "calibration_cmd_angle_calibrate removed",
        Ok(expected_removed),
        strings_value(&removed),
    );
    check(
        "calibration_cmd_angle_calibrate table",
        Ok(expected_table),
        Value::List(
            cal.calibration_table()
                .iter()
                .map(|v| Value::int(*v))
                .collect(),
        ),
    );
}

#[test]
fn calibration_incomplete_sensor_data() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '16'}})\n\
         p.settings = {{'stepper_x': {{'microsteps': 16, 'full_steps_per_rotation': 8}}}}\n\
         p.objects['configfile'] = FakeConfigFile(p)\n\
         p.objects['toolhead'] = FakeToolHead()\n\
         p.objects['force_move'] = FakeForceMove(p)\n\
         p.objects['tmc2209 stepper_x'] = FakeTmc(10, 256)\n\
         p.objects['angle c'] = FakeAngleClient()\n\
         c = angle.AngleCalibration(FakeConfig(p, 'angle c', {{'stepper': 'stepper_x'}}))\n\
         c.connect()\n\
         g = FakeGCommand(p.objects['gcode'], 'ANGLE_CALIBRATE', {{}})\n\
         res = capture(lambda: c.cmd_ANGLE_CALIBRATE(g))",
        py_env("angle c", "{}")
    ))
    .expect("py setup");
    let expected = py.eval("res").expect("result");

    let env = calibration_env("angle c", None, 16, 8, Some((Some(10), 256)));
    let client = FakeAngleClient::default();
    env.env.fp.add_object(
        "angle c",
        Arc::new(FakeAngleObject(client)) as ObjectRef,
    );
    let cal = AngleCalibration::new(&env.config).expect("calibration");
    cal.connect().expect("connect");
    let gcmd = fake_gcmd(&env.env.gcode, "ANGLE_CALIBRATE", &[]);
    let result = cal.cmd_angle_calibrate(&gcmd).map(|_| Value::Null);
    check_call("calibration_incomplete_sensor_data", Ok(expected), result);
}

#[test]
fn lstsq_matches_numpy_rounded() {
    let Some(mut py) = oracle() else { return };
    require_numpy(&mut py);
    py.exec(
        "import numpy as np\n\
         A = [[1.0 if (i % 64) == (j % 64) and j < 64 else (0.5 if j == (i + 1) % 64 else 0.0) for j in range(64)] for i in range(8)]\n\
         b = [float(i * 8192) for i in range(8)]\n\
         sol = np.linalg.lstsq(A, b, rcond=None)[0].tolist()\n\
         rounded = [int(s + .5) for s in sol]",
    )
    .expect("py lstsq");
    let expected = py.eval("rounded").expect("eval");
    let mut a = vec![vec![0.0; 64]; 8];
    for (i, row) in a.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            if i % 64 == j % 64 && j < 64 {
                *cell = 1.0;
            } else if j == (i + 1) % 64 {
                *cell = 0.5;
            }
        }
    }
    let b: Vec<f64> = (0..8).map(|i| (i as f64) * 8192.0).collect();
    let sol = lstsq_min_norm(&a, &b);
    let rounded: Vec<Value> = sol
        .iter()
        .map(|s| Value::int(extras::angle::py_trunc_i64(s + 0.5)))
        .collect();
    check("lstsq_matches_numpy_rounded", Ok(expected), Value::List(rounded));
}

// ---------------------------------------------------------------------------
// Angle: construction, sampling and batch processing.
// ---------------------------------------------------------------------------

fn make_raw_samples() -> Vec<PyValue> {
    vec![
        py_dict(&[
            ("sequence", pyint(1)),
            (
                "data",
                bytes_value(&[0x02, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xaa, 0xbb]),
            ),
        ]),
        py_dict(&[
            ("sequence", pyint(2)),
            ("data", bytes_value(&[0x04, 0x00, 0x01, 0x05, 0x00, 0x02])),
        ]),
    ]
}

fn raw_samples_py_expr(raw: &[PyValue]) -> String {
    let parts: Vec<String> = raw
        .iter()
        .map(|v| {
            let PyValue::Dict(m) = v else { return "None".to_string() };
            let seq = m
                .get(&PyValue::Str("sequence".to_string()))
                .cloned()
                .unwrap_or(pyint(0));
            let data = m
                .get(&PyValue::Str("data".to_string()))
                .cloned()
                .unwrap_or(bytes_value(&[]));
            let (seq_i, bytes) = match (seq, data) {
                (PyValue::Int(n), PyValue::Bytes(b)) => {
                    (n.0.to_string().parse::<i64>().unwrap_or(0), b)
                }
                _ => (0, Vec::new()),
            };
            let list = bytes
                .iter()
                .map(|b| b.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            format!("{{'sequence': {seq_i}, 'data': bytes([{list}])}}")
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

fn rust_angle(env: &RustEnv, config: &Config) -> Arc<Angle> {
    let angle = Angle::new(config).expect("rust angle");
    env.mcu.run_config_callbacks().expect("config callbacks");
    angle
}

#[test]
fn angle_init_gcode_and_mcu_commands() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
         a = angle.Angle(cfg)\np.objects['angle testangle'] = a\nrun_cbs(mcu)\n\
         res = {{'cmds': angle_cmds(mcu), 'mux': p.objects['gcode'].mux, \
                'webhooks': p.objects['webhooks'].endpoints}}",
        py_env(
            "angle testangle",
            "{'sensor_type': 'mt6816', 'cs_pin': 'PA5', 'stepper': 'stepper_x'}"
        )
    ))
    .expect("py angle");
    let expected_cmds = py.eval("res['cmds']").expect("cmds");
    let expected_mux = py.eval("res['mux']").expect("mux");
    let expected_webhooks = py.eval("res['webhooks']").expect("webhooks");

    let (env, config) = rust_env(
        "angle testangle",
        &[
            ("sensor_type", "mt6816"),
            ("cs_pin", "PA5"),
            ("stepper", "stepper_x"),
        ],
    );
    env.fp.set_section(
        "stepper_x",
        vec![("microsteps".to_string(), "256".to_string())],
    );
    let _angle = rust_angle(&env, &config);
    let cmds = env.mcu.angle_cmds();
    check(
        "angle_init_config_commands",
        Ok(expected_cmds),
        Value::List(
            cmds.iter()
                .map(|(c, is_init, on_restart)| {
                    Value::List(vec![
                        Value::str(c),
                        Value::Bool(*is_init),
                        Value::Bool(*on_restart),
                    ])
                })
                .collect(),
        ),
    );
    let mux = env.gcode.mux.lock().clone();
    check(
        "angle_init_gcode_commands",
        Ok(expected_mux),
        Value::List(
            mux.iter()
                .map(|(cmd, key, value, desc)| {
                    Value::List(vec![
                        Value::str(cmd),
                        Value::str(key),
                        value.as_deref().map(Value::str).unwrap_or(Value::Null),
                        Value::str(desc),
                    ])
                })
                .collect(),
        ),
    );
    let endpoints = env.webhooks.endpoints.lock().clone();
    check(
        "angle_init_webhooks",
        Ok(expected_webhooks),
        Value::List(
            endpoints
                .iter()
                .map(|(p, k, v)| Value::List(vec![Value::str(p), Value::str(k), Value::str(v)]))
                .collect(),
        ),
    );
}

#[test]
fn angle_extract_samples_batches() {
    let Some(mut py) = oracle() else { return };
    let raw = make_raw_samples();
    let raw_expr = raw_samples_py_expr(&raw);
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
         a = angle.Angle(cfg)\np.objects['angle testangle'] = a\nrun_cbs(mcu)\n\
         a._start_measurements()\nfeed(mcu, {raw_expr})\nbatch = a._process_batch(0.0)",
        py_env(
            "angle testangle",
            "{'sensor_type': 'mt6816', 'cs_pin': 'PA5', 'stepper': 'stepper_x'}"
        )
    ))
    .expect("py angle");
    let expected_batch = py.eval("batch").expect("batch");
    let expected_cmd = py.eval("command_calls(mcu, 'query_spi_angle')").expect("cmds");

    let (env, config) = rust_env(
        "angle testangle",
        &[
            ("sensor_type", "mt6816"),
            ("cs_pin", "PA5"),
            ("stepper", "stepper_x"),
        ],
    );
    env.fp.set_section(
        "stepper_x",
        vec![("microsteps".to_string(), "256".to_string())],
    );
    let angle = rust_angle(&env, &config);
    angle.start_measurements().expect("start");
    assert!(angle.is_measuring().expect("measuring"));
    env.mcu.feed(&raw);
    let batch = angle.process_batch(0.0).expect("batch");
    check(
        "angle_extract_samples_batches",
        Ok(expected_batch),
        pyvalue_scalar(&batch),
    );
    check(
        "angle_start_measurements command",
        Ok(expected_cmd),
        calls_value(&env.mcu.command_calls("query_spi_angle")),
    );
}

#[test]
fn angle_process_batch_no_stepper_raises() {
    let Some(mut py) = oracle() else { return };
    let raw = make_raw_samples();
    let raw_expr = raw_samples_py_expr(&raw);
    py.exec(&format!(
        "{}\na = angle.Angle(cfg)\np.objects['angle testangle'] = a\nrun_cbs(mcu)\n\
         a._start_measurements()\nfeed(mcu, {raw_expr})\n\
         res = capture(lambda: a._process_batch(0.0))",
        py_env(
            "angle testangle",
            "{'sensor_type': 'mt6816', 'cs_pin': 'PA5'}"
        )
    ))
    .expect("py angle");
    let expected = py.eval("res").expect("result");

    let (env, config) = rust_env(
        "angle testangle",
        &[("sensor_type", "mt6816"), ("cs_pin", "PA5")],
    );
    let angle = rust_angle(&env, &config);
    angle.start_measurements().expect("start");
    env.mcu.feed(&raw);
    let result = angle.process_batch(0.0).map(|v| pyvalue_scalar(&v));
    check_call("angle_process_batch_no_stepper", Ok(expected), result);
}

#[test]
fn angle_add_client_batch_flow() {
    let Some(mut py) = oracle() else { return };
    let raw = make_raw_samples();
    let raw_expr = raw_samples_py_expr(&raw);
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
         a = angle.Angle(cfg)\np.objects['angle testangle'] = a\nrun_cbs(mcu)\n\
         msgs = []\n\
         client = lambda msg: (msgs.append(msg), True)[1]\n\
         a.add_client(client)\n\
         idle = a.batch_bulk._proc_batch(0.0)\n\
         feed(mcu, {raw_expr})\n\
         a.batch_bulk._proc_batch(0.1)\n\
         res = {{'msgs': msgs, 'idle': idle, 'measuring': a._is_measuring(), \
                'status': a.get_status()}}\n\
         a._finish_measurements()\n\
         res['stopped'] = a._is_measuring()\n\
         res['status_after'] = a.get_status()",
        py_env(
            "angle testangle",
            "{'sensor_type': 'mt6816', 'cs_pin': 'PA5', 'stepper': 'stepper_x'}"
        )
    ))
    .expect("py flow");
    let expected_msgs = py.eval("res['msgs']").expect("msgs");
    let expected_idle = py.eval("res['idle']").expect("idle");
    let expected_measuring = py.eval("res['measuring']").expect("measuring");
    let expected_status = py.eval("res['status']").expect("status");
    let expected_stopped = py.eval("res['stopped']").expect("stopped");
    let expected_status_after = py.eval("res['status_after']").expect("status after");

    let (env, config) = rust_env(
        "angle testangle",
        &[
            ("sensor_type", "mt6816"),
            ("cs_pin", "PA5"),
            ("stepper", "stepper_x"),
        ],
    );
    env.fp.set_section(
        "stepper_x",
        vec![("microsteps".to_string(), "256".to_string())],
    );
    let angle = rust_angle(&env, &config);
    let msgs: Arc<Mutex<Vec<PyValue>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let msgs = Arc::clone(&msgs);
        angle
            .add_client(Box::new(move |msg: PyValue| {
                msgs.lock().push(msg);
                Ok(true)
            }))
            .expect("add client");
    }
    let idle = angle
        .batch_bulk()
        .expect("batch bulk")
        ._proc_batch(0.0)
        .expect("idle");
    env.mcu.feed(&raw);
    angle
        .batch_bulk()
        .expect("batch bulk")
        ._proc_batch(0.1)
        .expect("batch");
    let delivered = msgs.lock().clone();
    check(
        "angle_add_client_batch_flow messages",
        Ok(expected_msgs),
        Value::List(delivered.iter().map(pyvalue_scalar).collect()),
    );
    check(
        "angle_add_client_batch_flow idle",
        Ok(expected_idle),
        Value::float(idle),
    );
    check(
        "angle_add_client_batch_flow measuring",
        Ok(expected_measuring),
        Value::Bool(angle.is_measuring().expect("measuring")),
    );
    check(
        "angle_add_client_batch_flow status",
        Ok(expected_status),
        pyvalue_scalar(&angle.temperature_status()),
    );
    angle.finish_measurements().expect("finish");
    check(
        "angle_add_client_batch_flow stopped",
        Ok(expected_stopped),
        Value::Bool(angle.is_measuring().expect("measuring")),
    );
    check(
        "angle_add_client_batch_flow status after",
        Ok(expected_status_after),
        pyvalue_scalar(&angle.temperature_status()),
    );
}

#[test]
fn angle_extract_samples_absolute() {
    let Some(mut py) = oracle() else { return };
    let raw = make_raw_samples();
    let raw_expr = raw_samples_py_expr(&raw);
    // `start()` writes/reads the sensor and queries the clock twice; the
    // batch then runs `update_clock()` once more.
    let entries: Vec<TleEntry> = vec![
        ('t', vec![0x80, 0x01], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0x61], 0x0f, 0xff, 0x00, 0),
        ('t', vec![0x50, 0x61], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0x61], 0x4f, 0xfc, 0x00, 0),
        ('t', vec![0xd0, 0x81], 0x00, 0x00, 0x00, 0),
        ('t', vec![0x50, 0x81], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0x81], 0x00, 0x01, 0x00, 0),
        ('t', vec![0xd0, 0xe1], 0x00, 0x00, 0x00, 0),
        ('t', vec![0x50, 0xe1], 0x00, 0x00, 0x00, 0),
        ('t', vec![0xd0, 0xe1], 0x00, 0x00, 0x00, 0),
        // The chip frame counter must advance between the three clock
        // queries (992 chip ticks per 1000 mcu clocks); equal counters make
        // `update_clock` compute `chip_freq = 0.` and `1. / chip_freq`
        // raise ZeroDivisionError in `_extract_samples`.
        ('a', vec![0x84, 0x42], 0x12, 0x00, 0x34, 1000),
        ('a', vec![0x84, 0x42], 0x14, 0x00, 0x32, 2000),
        ('a', vec![0x84, 0x42], 0x16, 0x00, 0x30, 3000),
    ];
    let literal = tle_literal(&entries);
    py.exec(&format!(
        "{}\np.set_section('stepper_x', {{'microsteps': '256'}})\n\
         a = angle.Angle(cfg)\np.objects['angle testangle'] = a\nrun_cbs(mcu)\n\
         PY = tle_responses([{literal}])\n\
         script_query(mcu, 'spi_transfer', PY['spi_transfer'])\n\
         script_query(mcu, 'spi_angle_transfer', PY['spi_angle_transfer'])\n\
         a._start_measurements()\nfeed(mcu, {raw_expr})\nbatch = a._process_batch(0.0)",
        py_env(
            "angle testangle",
            "{'sensor_type': 'tle5012b', 'cs_pin': 'PA5', 'stepper': 'stepper_x'}"
        )
    ))
    .expect("py angle");
    let expected_batch = py.eval("batch").expect("batch");

    let (env, config) = rust_env(
        "angle testangle",
        &[
            ("sensor_type", "tle5012b"),
            ("cs_pin", "PA5"),
            ("stepper", "stepper_x"),
        ],
    );
    env.fp.set_section(
        "stepper_x",
        vec![("microsteps".to_string(), "256".to_string())],
    );
    let angle = Angle::new(&config).expect("rust angle");
    env.mcu.run_config_callbacks().expect("config cbs");
    let (t_script, a_script) = tle_scripts(&mut py, &entries);
    env.mcu.script("spi_transfer", t_script);
    env.mcu.script("spi_angle_transfer", a_script);
    angle.start_measurements().expect("start");
    env.mcu.feed(&raw);
    let batch = angle.process_batch(0.0).expect("batch");
    check(
        "angle_extract_samples_absolute",
        Ok(expected_batch),
        pyvalue_scalar(&batch),
    );
}

#[test]
fn angle_get_status_temperature() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "{}\na = angle.Angle(cfg)\np.objects['angle testangle'] = a\nrun_cbs(mcu)\nres = a.get_status()",
        py_env(
            "angle testangle",
            "{'sensor_type': 'mt6816', 'cs_pin': 'PA5'}"
        )
    ))
    .expect("py angle");
    let expected = py.eval("res").expect("status");
    let (env, config) = rust_env(
        "angle testangle",
        &[("sensor_type", "mt6816"), ("cs_pin", "PA5")],
    );
    let angle = rust_angle(&env, &config);
    check(
        "angle_get_status_temperature",
        Ok(expected),
        pyvalue_scalar(&angle.temperature_status()),
    );
}

#[test]
fn angle_bad_sensor_type() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval(
        "capture(lambda: angle.Angle(FakeConfig(FakePrinter(), 'angle a', \
         {'sensor_type': 'nosuch', 'cs_pin': 'PA5'})))",
    );
    let (env, config) = rust_env(
        "angle a",
        &[("sensor_type", "nosuch"), ("cs_pin", "PA5")],
    );
    let _ = &env;
    let result = Angle::new(&config).map(|_| Value::Null);
    check_call("angle_bad_sensor_type", expected, result);
}

#[test]
fn angle_bad_sample_period() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval(
        "capture(lambda: angle.Angle(FakeConfig(FakePrinter(), 'angle a', \
         {'sensor_type': 'mt6816', 'sample_period': '0', 'cs_pin': 'PA5'})))",
    );
    let (env, config) = rust_env(
        "angle a",
        &[
            ("sensor_type", "mt6816"),
            ("sample_period", "0"),
            ("cs_pin", "PA5"),
        ],
    );
    let _ = &env;
    let result = Angle::new(&config).map(|_| Value::Null);
    check_call("angle_bad_sample_period", expected, result);
}

#[test]
fn module_factory_shape() {
    let factory = extras::angle::module_factory();
    assert!(factory.load_config.is_none(), "angle has no load_config");
    assert!(
        factory.load_config_prefix.is_some(),
        "angle exposes load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "angle has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "angle has no add_printer_objects"
    );
}
