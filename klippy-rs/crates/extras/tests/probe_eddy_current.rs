//! Parity tests for `extras::probe_eddy_current`
//! (port of `klippy/extras/probe_eddy_current.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). The Python side builds the real
//! `extras.probe_eddy_current` classes on real `extras.probe`,
//! `extras.manual_probe`, `gcode` and `configfile` objects, with fakes only
//! for the hardware collaborators (`extras.ldc1612` sensor and
//! `extras.trigger_analog` filters, which need a real MCU/I2C bus) and for
//! the printer/toolhead/kinematics. The Rust side drives the port with fakes
//! of the same contract traits, feeding identical inputs.
//!
//! The Rust side uses the real `klippy_core::gcode::GCodeDispatch` and the
//! real `crate::trigger_analog` objects, so G-code parameters, response text
//! and error text are produced by the same code both sides.
//!
//! Compared: the module constants, calibration table parsing/sorting,
//! `apply_calibration` interpolation (in range and both out-of-range ends),
//! `freq_to_height`/`height_to_freq`, the drift-compensation hook order and
//! values, the saved `calibrate`/`tap_z_offset` strings and every
//! `respond_info` line (captured through the real dispatch on both sides),
//! `calc_freqs`/`_median`/`validate_calibration_data` (including the noisy and
//! empty cases), the 101-step descending calibration move table,
//! `TapBestFit` coefficients and both matrix builders, the gathered-sample
//! window/deferral/file-output/outage paths, `probe_results_from_avg`, the
//! probe offsets, `get_probe_params` for `automatic`/`scan`/`tap`, the tap
//! sample-time and lift checks, and one full descend session (homing call,
//! gathered samples, averaged result) plus the `scan` session with the
//! `probe:update_results` handler rewriting the result, and `rapid_scan`
//! lookahead registration.
//!
//! Calibration commands also run through the real manual-probe helper,
//! including abort and acceptance. The missing `ldc1612` driver prevents a
//! full `PrinterEddyProbe` construction; that dependency is separate from
//! the calibration and session comparisons.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so expected values come from the oracle alone. When `$KLIPPY_PYTHON` is
//! unset the oracle test skips.

use extras::bulk_sensor::ClientCallback;
use extras::ldc1612::LDC1612Api;
use extras::probe_eddy_current::{
    module_factory, probe_results_from_avg, EddyCalibration, EddyCalibrationApi,
    EddyCalibrationTool, EddyDescend, EddyGatherSamples, EddyParameterHelper, PrinterEddyProbeApi,
    EddyProbeOffsets, EddyScanningProbe, EddyTap, TapBestFit,
};
use extras::manual_probe::ProbeResult;
use extras::probe::{PrinterProbeApi, ProbeParams, ProbeSessionApi};
use extras::trigger_analog::{MCU_trigger_analog, TriggerAnalogSensor};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeMoveApi, GCmd};
use klippy_api::mcu::{
    AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper, Mcu, McuApi,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{McuStepperApi, PullHistoryStep, Stepper, StepperKinematics, Trapq};
use klippy_api::toolhead::{
    HomingStateHandle, HomingTarget, KinematicsApi, MoveApi, PrinterHomingApi, ToolHeadApi,
};
use klippy_api::{EventArg, GCell, ObjectRef};
use klippy_core::gcode::GCodeDispatch;
use pycompat::text::float_from_string;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python side: real reference classes, fakes for the hardware collaborators.
// ---------------------------------------------------------------------------
const SETUP: &str = r#"
import sys, types, configparser, math
import gcode as gcode_mod
import configfile as configfile_mod
import extras.probe as probe_mod
import extras.manual_probe as manual_probe

LOG = []

class CommandError(Exception):
    pass

class FakeMutex:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        return False

class FakeReactor:
    def __init__(self):
        self.time = 100.0
    def monotonic(self):
        return self.time
    def pause(self, waketime):
        return waketime
    def mutex(self):
        return FakeMutex()

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.reactor = FakeReactor()
        self.shutdowns = []
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=configfile_mod.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile_mod.sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=configfile_mod.sentinel):
        return self.lookup_object(section, default)
    def get_reactor(self):
        return self.reactor
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        for cb in list(self.handlers.get(event, [])):
            cb(*params)
        return []
    def get_start_args(self):
        return {}
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)

class FakeConfigFile:
    def __init__(self, settings=None):
        self.writes = []
        self.deprecations = []
        self.settings = settings or {}
    def set(self, section, option, value):
        self.writes.append((section, option, value))
    def deprecate(self, section, option, value=None, msg=None):
        self.deprecations.append((section, option))
    def get_status(self, eventtime=None):
        return {'settings': self.settings}

class FakeGCodeMove:
    def __init__(self, offset):
        self.offset = offset
    def get_status(self, eventtime=None):
        return {'homing_origin': gcode_mod.Coord((0.0, 0.0, self.offset, 0.0))}

class FakeMcu:
    def __init__(self):
        self.is_fileout = False
        self.est = 1e9
    def is_fileoutput(self):
        return self.is_fileout
    def estimated_print_time(self, systime):
        return self.est

class FakeSensor:
    def __init__(self, mcu):
        self.clients = []
        self.mcu = mcu
        self.sps = 400.0
        self.conv = 1.0
        self.auto = None
    def add_client(self, cb):
        self.clients.append(cb)
    def deliver(self, msg):
        keep = []
        for cb in self.clients:
            if cb(msg):
                keep.append(cb)
        self.clients = keep
    def get_mcu(self):
        return self.mcu
    def get_samples_per_second(self):
        return self.sps
    def convert_raw_to_frequency(self, raw):
        return raw * self.conv
    def convert_frequency_to_raw(self, freq):
        return int(freq / self.conv + 0.5)

class FakeStepper:
    def __init__(self, name, toolhead=None, past_fn=None):
        self._name = name
        self._toolhead = toolhead
        self._past_fn = past_fn
    def get_name(self, short=False):
        return self._name
    def get_commanded_position(self):
        return self._toolhead.get_position()[2]
    def get_past_mcu_position(self, print_time):
        fn = self._past_fn or getattr(self._toolhead, 'past_fn', None)
        if fn is None:
            return 0
        return int(round(fn(print_time) * 1e7))
    def mcu_to_commanded_position(self, m):
        return m / 1e7

class FakeKinematics:
    def __init__(self, toolhead):
        self.toolhead = toolhead
    def get_steppers(self):
        th = self.toolhead
        return [FakeStepper('stepper_x', th), FakeStepper('stepper_y', th),
                FakeStepper('stepper_z', th)]
    def calc_position(self, spos):
        return [spos['stepper_x'], spos['stepper_y'], spos['stepper_z']]

class FakeToolhead:
    def __init__(self, position, sensor=None):
        self.position = list(position)
        self.kin = FakeKinematics(self)
        self.moves = []
        self.dwells = []
        self.flushes = 0
        self.waits = 0
        self.print_time = 0.0
        self.sensor = sensor
        self.lookahead = []
        self.past_fn = None
    def get_position(self):
        return list(self.position)
    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v
        if self.sensor is not None and self.sensor.auto is not None:
            data = self.sensor.auto(self.print_time + 0.1, len(self.moves))
            self.sensor.deliver({'data': data, 'errors': 0, 'overflows': 0})
    def dwell(self, delay):
        self.dwells.append(delay)
        self.print_time += delay
    def flush_step_generation(self):
        self.flushes += 1
    def wait_moves(self):
        self.waits += 1
    def get_last_move_time(self):
        return self.print_time
    def get_kinematics(self):
        return self.kin
    def register_lookahead_callback(self, cb):
        self.lookahead.append(cb)

class FakeHoming:
    def __init__(self, toolhead=None):
        self.calls = []
        self.toolhead = toolhead
    def probing_move(self, mcu_probe, pos, speed, check_movement=True):
        self.calls.append((list(pos), speed, check_movement))
        if self.toolhead is not None:
            self.toolhead.position = list(pos)
        return list(pos)
    def check_probe_first_home(self, gcmd):
        return False

class FakePins:
    def __init__(self):
        self.chips = {}
    def register_chip(self, name, chip):
        self.chips[name] = chip

class FakeProbeSession:
    def __init__(self, results):
        self.results = results
        self.ran = False
        self.ended = False
    def run_probe(self, gcmd):
        self.ran = True
    def pull_probed_results(self):
        return self.results
    def end_probe_session(self):
        self.ended = True

class FakeProbeHost:
    def __init__(self):
        self.results = []
        self.sessions = []
        self.offsets = (3.0, 4.0, 5.0)
        self.seen_params = []
    def get_probe_params(self, gcmd=None):
        return {'probe_speed': 5.0, 'lift_speed': 4.0, 'samples': 1,
                'sample_retract_dist': 0.0, 'samples_tolerance': 0.1,
                'samples_tolerance_retries': 0, 'samples_result': 'average'}
    def get_offsets(self, gcmd=None):
        return self.offsets
    def start_probe_session(self, gcmd):
        self.seen_params.append(dict(gcmd.get_command_parameters()))
        session = FakeProbeSession(self.results)
        self.sessions.append(session)
        return session

# ---- fake extras.ldc1612 (needs an I2C bus) ----
ldc_mod = types.ModuleType('extras.ldc1612')
class FakeLDC1612:
    def __init__(self, config, calibration=None):
        self.printer = config.get_printer()
        self.calibration = calibration
        self.mcu = self.printer.lookup_object('mcu')
        self.data_rate = 400.0
        self.freq_conv = 1.0
    def setup_trigger_analog(self, oid):
        pass
    def get_mcu(self):
        return self.mcu
    def get_samples_per_second(self):
        return self.data_rate
    def add_client(self, cb):
        self.printer.lookup_object('probe_eddy_current eddy').add_client(cb)
    def lookup_sensor_error(self, error):
        return "Unknown ldc1612 error"
    def convert_raw_to_frequency(self, raw):
        return raw * self.freq_conv
    def convert_frequency_to_raw(self, freq):
        return int(freq / self.freq_conv + 0.5)
ldc_mod.LDC1612 = FakeLDC1612
sys.modules['extras.ldc1612'] = ldc_mod

# ---- fake extras.trigger_analog (needs a real MCU) ----
ta_mod = types.ModuleType('extras.trigger_analog')
class FakeSosFilter:
    def __init__(self):
        self.design = None
        self.offset_scale = None
    def set_filter_design(self, design):
        self.design = design
    def set_offset_scale(self, offset, scale, auto_offset=False):
        self.offset_scale = (offset, scale, auto_offset)
    def reset_filter(self):
        pass
class FakeDigitalFilter:
    GeneratedSOS = None
    def __init__(self, sps, cfg_error):
        self.sections = 0
    def add_lowpass(self, freq, order):
        self.sections += 2
    def add_derivative(self):
        self.sections += 1
    def get_size(self):
        return self.sections
class FakeDispatch:
    def get_command_queue(self):
        return 1
    def add_stepper(self, stepper):
        pass
class FakeMcuSosFilter:
    def __init__(self, mcu, cmd_queue, max_sections):
        self.max_sections = max_sections
    def set_filter_design(self, design):
        pass
    def set_offset_scale(self, offset, scale, auto_offset=False):
        pass
    def reset_filter(self):
        pass
class FakeMcuTrigger:
    def __init__(self, sensor):
        self.sensor = sensor
        self.sos = FakeSosFilter()
        self.triggers = []
        self.ranges = []
        self.last_trigger_time = 0.0
    def get_sos_filter(self):
        return self.sos
    def setup_sos_filter(self, sos):
        self.sos = sos
    def set_raw_range(self, lo, hi):
        self.ranges.append((lo, hi))
    def set_trigger(self, typ, value):
        self.triggers.append((typ, value))
    def get_last_trigger_time(self):
        return self.last_trigger_time
    def get_dispatch(self):
        return FakeDispatch()
    def get_mcu(self):
        return self.sensor.get_mcu()
ta_mod.DigitalFilter = FakeDigitalFilter
ta_mod.MCU_SosFilter = FakeMcuSosFilter
ta_mod.MCU_trigger_analog = FakeMcuTrigger
sys.modules['extras.trigger_analog'] = ta_mod

import extras.probe_eddy_current as pec

def build(cal=None, extra=None, offset=0.0, position=(10.0, 10.0, 2.0, 0.0),
          settings=None):
    del LOG[:]
    P = FakePrinter()
    GC = gcode_mod.GCodeDispatch(P)
    GC.register_output_handler(lambda msg: LOG.append(msg))
    MCU = FakeMcu()
    SENSOR = FakeSensor(MCU)
    TH = FakeToolhead(list(position), SENSOR)
    CF = FakeConfigFile(settings)
    HOM = FakeHoming(TH)
    PINS = FakePins()
    HOST = FakeProbeHost()
    fc = configparser.RawConfigParser()
    # `manual_probe.ManualProbe` reads `[printer] kinematics` unconditionally.
    secs = {'printer': {'kinematics': 'cartesian'},
            'stepper_z': {'position_min': '0.0', 'position_endstop': '5.0'}}
    if cal is not None:
        opts = {'calibrate': cal, 'sensor_type': 'ldc1612', 'descend_z': '1.0',
                'x_offset': '2.0', 'y_offset': '1.0',
                'tap_z_offset': '0.0', 'tap_threshold': '1000.0'}
        if extra:
            for key, value in extra.items():
                if value == '':
                    opts.pop(key, None)
                else:
                    opts[key] = value
        secs['probe_eddy_current eddy'] = opts
    for name, opts in secs.items():
        fc.add_section(name)
        for k, v in opts.items():
            fc.set(name, k, v)
    C = configfile_mod.ConfigWrapper(P, fc, {}, 'probe_eddy_current eddy')
    P.objects.update({'gcode': GC, 'toolhead': TH, 'configfile': CF,
                      'homing': HOM, 'pins': PINS, 'mcu': MCU, 'probe': HOST,
                      'gcode_move': FakeGCodeMove(offset),
                      'probe_eddy_current eddy': SENSOR})
    return {'p': P, 'g': GC, 'th': TH, 'cf': CF, 'c': C, 'sensor': SENSOR,
            'homing': HOM, 'mcu': MCU, 'pins': PINS, 'host': HOST}

def gcmd(w, params, command='PROBE'):
    return w['g'].create_gcode_command(command, '', params)

def params_tuple(p):
    return (p['probe_speed'], p['lift_speed'], p['samples'],
            p['sample_retract_dist'], p['samples_tolerance'],
            p['samples_tolerance_retries'], p['samples_result'])

def msg(data):
    return {'data': data, 'errors': 0, 'overflows': 0}
"#;

// ---------------------------------------------------------------------------
// Rust fakes.
// ---------------------------------------------------------------------------

type Trace = Arc<Mutex<Vec<String>>>;
type Clients = Arc<Mutex<Vec<ClientCallback>>>;
type Sections = Arc<Mutex<IndexMap<String, IndexMap<String, String>>>>;

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
    handlers: Arc<Mutex<Vec<(String, EventCallback)>>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: Arc::new(Mutex::new(IndexMap::new())),
            handlers: Arc::new(Mutex::new(Vec::new())),
            reactor: Reactor::new(false),
        })
    }

    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
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
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        if self.objects.lock().contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name) {
            return Ok(Some(Arc::clone(obj)));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects
            .lock()
            .iter()
            .filter(|(name, _)| module.is_none_or(|m| name.as_str() == m || name.starts_with(&format!("{m} "))))
            .map(|(name, obj)| (name.clone(), Arc::clone(obj)))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
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
        let mut out = Vec::new();
        let mut handlers = self.handlers.lock();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    sections: Sections,
    deprecations: Arc<Mutex<Vec<(String, String)>>>,
}

impl FakeConfig {
    fn make(printer: &Arc<FakePrinter>, name: &str, sections: Sections) -> Config {
        Arc::new(FakeConfig {
            printer: Arc::clone(printer) as Printer,
            name: name.to_string(),
            sections,
            deprecations: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn raw(&self, option: &str) -> Option<String> {
        self.sections
            .lock()
            .get(&self.name)
            .and_then(|opts| opts.get(option))
            .cloned()
    }

    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.name
        ))
    }

    fn unparseable(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.name
        ))
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
        match self.raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.missing(option)),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.raw(option) {
            Some(v) => {
                let parsed = pycompat::text::int_from_string(&v, 10)
                    .map_err(|_| self.unparseable(option))?;
                let n: i64 = parsed
                    .inner()
                    .to_string()
                    .parse()
                    .map_err(|_| self.unparseable(option))?;
                if let Some(minval) = opts.minval {
                    if n < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.name
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if n > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.name
                        )));
                    }
                }
                Ok(Some(n))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.missing(option)),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.raw(option) {
            Some(v) => {
                let value = float_from_string(&v).map_err(|_| self.unparseable(option))?;
                if let Some(minval) = opts.minval {
                    if value < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.name,
                            pycompat::text::float_str(minval)
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if value > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.name,
                            pycompat::text::float_str(maxval)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if value <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.name,
                            pycompat::text::float_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if value >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.name,
                            pycompat::text::float_str(below)
                        )));
                    }
                }
                Ok(Some(value))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.missing(option)),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.raw(option) {
            Some(v) => match v.trim().to_lowercase().as_str() {
                "1" | "yes" | "true" | "on" => Ok(Some(true)),
                "0" | "no" | "false" | "off" => Ok(Some(false)),
                _ => Err(self.unparseable(option)),
            },
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.missing(option)),
            },
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let (keys, map): (Vec<PyValue>, IndexMap<String, PyValue>) = match choices {
            PyValue::List(items) => (items.clone(), IndexMap::new()),
            PyValue::Tuple(items) => (items.clone(), IndexMap::new()),
            PyValue::Dict(entries) => (
                entries.keys().cloned().collect(),
                entries
                    .iter()
                    .map(|(k, v)| (k.str(), v.clone()))
                    .collect::<IndexMap<String, PyValue>>(),
            ),
            other => (Vec::new(), {
                let _ = other;
                IndexMap::new()
            }),
        };
        let value = match self.get(option, GetStrOpts { default: opts.default.map(|d| d.map(|v| v.str())), note_valid: opts.note_valid })? {
            Some(text) => PyValue::Str(text),
            None => PyValue::None,
        };
        let found = keys.iter().find(|k| match (k, &value) {
            (PyValue::Str(a), PyValue::Str(b)) => a == b,
            (PyValue::Int(a), PyValue::Str(b)) => a.inner().to_string() == *b,
            _ => k.str() == value.str(),
        });
        let Some(key) = found else {
            return Err(KlippyError::config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                value.str(),
                self.name
            )));
        };
        if map.is_empty() {
            Ok(key.clone())
        } else {
            Ok(map.get(&key.str()).cloned().unwrap_or(PyValue::None))
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by probe_eddy_current")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by probe_eddy_current")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by probe_eddy_current")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by probe_eddy_current")
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            sections: Arc::clone(&self.sections),
            deprecations: Arc::clone(&self.deprecations),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.lock().contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        let names: Vec<String> = self
            .sections
            .lock()
            .keys()
            .filter(|name| name.starts_with(prefix))
            .cloned()
            .collect();
        names.iter().map(|name| self.getsection(name)).collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>> {
        Ok(self.sections
            .lock()
            .get(&self.name)
            .map(|opts| {
                opts.keys()
                    .filter(|k| k.starts_with(prefix))
                    .cloned()
                    .collect()
            })
            .unwrap_or_default())
    }
    fn deprecate(&self, option: &str, _value: Option<PyValue>) -> KResult<()> {
        if self.raw(option).is_some() {
            self.deprecations
                .lock()
                .push((self.name.clone(), option.to_string()));
        }
        Ok(())
    }
}

struct FakeConfigFile {
    writes: Arc<Mutex<Vec<(String, String, String)>>>,
    deprecations: Arc<Mutex<Vec<(String, String)>>>,
    settings: IndexMap<String, IndexMap<String, PyValue>>,
}

impl FakeConfigFile {
    fn new(settings: &[(&str, &[(&str, PyValue)])]) -> Arc<Self> {
        let mut map = IndexMap::new();
        for (section, opts) in settings {
            let mut inner = IndexMap::new();
            for (name, value) in *opts {
                inner.insert(name.to_string(), value.clone());
            }
            map.insert(section.to_string(), inner);
        }
        Arc::new(FakeConfigFile {
            writes: Arc::new(Mutex::new(Vec::new())),
            deprecations: Arc::new(Mutex::new(Vec::new())),
            settings: map,
        })
    }
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by probe_eddy_current")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config is not used by probe_eddy_current")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("read_main_config is not used by probe_eddy_current")
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(
        &self,
        section: &str,
        option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
        self.deprecations
            .lock()
            .push((section.to_string(), option.to_string()));
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut sections = IndexMap::new();
        for (name, opts) in &self.settings {
            sections.insert(
                PyValue::Str(name.clone()),
                PyValue::Dict(
                    opts.iter()
                        .map(|(k, v)| (PyValue::Str(k.clone()), v.clone()))
                        .collect(),
                ),
            );
        }
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("settings".to_string()),
            PyValue::Dict(sections),
        );
        PyValue::Dict(status)
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.writes.lock().push((
            section.to_string(),
            option.to_string(),
            value.to_string(),
        ));
        Ok(())
    }
    fn remove_section(&self, _section: &str) {}
}

klippy_api::printer_object!(
    FakeConfigFile;
    dyn PrinterConfigApi
);

struct FakeGCodeMove {
    offset: f64,
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("homing_origin".to_string()),
            PyValue::Tuple(vec![
                PyValue::Float(0.0),
                PyValue::Float(0.0),
                PyValue::Float(self.offset),
                PyValue::Float(0.0),
            ]),
        );
        PyValue::Dict(status)
    }
    fn set_move_transform(
        &self,
        _transform: Option<klippy_api::gcode::MoveTransform>,
        _force: bool,
    ) -> KResult<Option<klippy_api::gcode::MoveTransform>> {
        panic!("set_move_transform is not used by probe_eddy_current")
    }
    fn reset_last_position(&self) -> KResult<()> { Ok(()) }
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> {
        panic!("cmd_g1 is not used by probe_eddy_current")
    }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

struct FakeStepper {
    name: String,
    origin: Arc<GCell<Vec<f64>>>,
    past: Arc<dyn Fn(f64) -> f64 + Send + Sync>,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by probe_eddy_current")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        0
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _duration: f64, _both: bool) {}
    fn setup_itersolve(&self, _func: &str, _params: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 {
        0.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _distance: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _invert: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _coord: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        self.origin.borrow()[2]
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, print_time: f64) -> i64 {
        // Python's `int(round(pos * 1e7))`: half-even, not half-away-from-zero.
        let scaled = (self.past)(print_time) * 1e7;
        pycompat::text::round_py(scaled)
            .ok()
            .and_then(|n| n.inner().to_string().parse::<i64>().ok())
            .unwrap_or(0)
    }
    fn mcu_to_commanded_position(&self, mcu_pos: f64) -> f64 {
        mcu_pos / 1e7
    }
    fn dump_steps(&self, _count: i64, _start: i64, _end: i64) -> (Vec<PullHistoryStep>, i64) {
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
    fn set_trapq(&self, _trapq: Option<Trapq>) -> Option<Trapq> {
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeKinematics {
    steppers: Vec<Stepper>,
}

impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn calc_position(&self, spos: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(vec![
            Some(spos["stepper_x"]),
            Some(spos["stepper_y"]),
            Some(spos["stepper_z"]),
        ])
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::Dict(IndexMap::new()))
    }
    fn clear_homing_state(&self, _clear_axes: &str) -> klippy_api::KResult<()> { Ok(()) }
}

struct FakeToolhead {
    position: Arc<GCell<Vec<f64>>>,
    kin: Arc<FakeKinematics>,
    moves: Arc<Mutex<Vec<(Vec<Option<f64>>, f64)>>>,
    dwells: Arc<Mutex<Vec<f64>>>,
    flushes: Arc<Mutex<i64>>,
    waits: Arc<Mutex<i64>>,
    print_time: GCell<f64>,
    lookahead: Arc<Mutex<Vec<klippy_api::toolhead::LookaheadCallback>>>,
    move_hook: Arc<Mutex<Option<Arc<dyn Fn(f64) + Send + Sync>>>>,
}

impl FakeToolhead {
    fn new(
        position: Vec<f64>,
        past: Arc<dyn Fn(f64) -> f64 + Send + Sync>,
    ) -> Arc<Self> {
        let origin = Arc::new(GCell::new(position));
        let past = past;
        let steppers = ["stepper_x", "stepper_y", "stepper_z"]
            .iter()
            .map(|name| {
                Arc::new(FakeStepper {
                    name: name.to_string(),
                    origin: Arc::clone(&origin),
                    past: Arc::clone(&past),
                }) as Stepper
            })
            .collect();
        Arc::new(FakeToolhead {
            position: Arc::clone(&origin),
            kin: Arc::new(FakeKinematics { steppers }),
            moves: Arc::new(Mutex::new(Vec::new())),
            dwells: Arc::new(Mutex::new(Vec::new())),
            flushes: Arc::new(Mutex::new(0)),
            waits: Arc::new(Mutex::new(0)),
            print_time: GCell::new(0.0),
            lookahead: Arc::new(Mutex::new(Vec::new())),
            move_hook: Arc::new(Mutex::new(None)),
        })
    }

    /// `sensor.auto`: one hook call after every `manual_move`.
    fn set_move_hook(&self, hook: Arc<dyn Fn(f64) + Send + Sync>) {
        *self.move_hook.lock() = Some(hook);
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.borrow().clone()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        *self.position.borrow_mut() = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        self.kin.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        *self.flushes.lock() += 1;
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(self.print_time.get())
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        self.dwells.lock().push(delay);
        self.print_time.set(self.print_time.get() + delay);
        Ok(())
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _cheb: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by probe_eddy_current")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        self.moves.lock().push((coord.clone(), speed));
        {
            let mut position = self.position.borrow_mut();
            for (i, value) in coord.iter().enumerate() {
                if let Some(value) = value {
                    position[i] = *value;
                }
            }
        }
        let hook = self.move_hook.lock().clone();
        if let Some(hook) = hook {
            hook(self.print_time.get());
        }
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> {
        *self.waits.lock() += 1;
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::Dict(IndexMap::new()))
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by probe_eddy_current")
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
    fn register_lookahead_callback(
        &self,
        callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        self.lookahead.lock().push(callback);
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(
        &self,
        _extruder: klippy_api::toolhead::PrinterExtruder,
        _extrude_pos: f64,
    ) {
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder is not used by probe_eddy_current")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by probe_eddy_current")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct FakeHoming {
    calls: Arc<Mutex<Vec<(Vec<Option<f64>>, f64, bool)>>>,
    position: Arc<GCell<Vec<f64>>>,
}

impl PrinterHomingApi for FakeHoming {
    fn manual_home(
        &self,
        _toolhead: HomingTarget,
        _endstops: Vec<(klippy_api::mcu::Endstop, String)>,
        _pos: Vec<Option<f64>>,
        _speed: f64,
        _probe_pos: bool,
        _triggered: bool,
        _check_triggered: bool,
    ) -> KResult<Vec<f64>> {
        panic!("manual_home is not used by probe_eddy_current")
    }
    fn probing_move(
        &self,
        _mcu_probe: klippy_api::mcu::Endstop,
        pos: Vec<Option<f64>>,
        speed: f64,
        check_movement: bool,
    ) -> KResult<Vec<f64>> {
        self.calls
            .lock()
            .push((pos.clone(), speed, check_movement));
        *self.position.borrow_mut() = pos.iter().map(|v| v.unwrap_or(0.0)).collect();
        Ok(pos.iter().map(|v| v.unwrap_or(0.0)).collect())
    }
    fn check_probe_first_home(&self, _gcmd: &GCmd) -> KResult<bool> {
        Ok(false)
    }
}

klippy_api::printer_object!(FakeHoming; dyn PrinterHomingApi);

struct FakePins {
    chips: Arc<Mutex<Vec<String>>>,
}

impl klippy_api::mcu::PinsApi for FakePins {
    fn parse_pin(
        &self,
        _pin_desc: &str,
        _opts: klippy_api::mcu::ParsePinOpts,
    ) -> KResult<klippy_api::mcu::PinParams> {
        panic!("parse_pin is not used by probe_eddy_current")
    }
    fn lookup_pin(
        &self,
        _pin_desc: &str,
        _opts: klippy_api::mcu::LookupPinOpts,
    ) -> KResult<klippy_api::mcu::PinParams> {
        panic!("lookup_pin is not used by probe_eddy_current")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<klippy_api::mcu::Pin> {
        panic!("setup_pin is not used by probe_eddy_current")
    }
    fn reset_pin_sharing(&self, _pin_params: klippy_api::mcu::PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by probe_eddy_current")
    }
    fn register_chip(&self, chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        self.chips.lock().push(chip_name.to_string());
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn klippy_api::mcu::PinsApi);

struct FakeTriggerDispatch {
    steppers: Mutex<Vec<Stepper>>,
}

impl klippy_api::mcu::TriggerDispatchApi for FakeTriggerDispatch {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn add_stepper(&self, stepper: Stepper) -> KResult<()> {
        self.steppers.lock().push(stepper);
        Ok(())
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.lock().clone()
    }
    fn start(&self, _print_time: f64) -> reactor::Completion<bool> {
        let completion = reactor::Completion::new(Reactor::new(false));
        completion.complete(true);
        completion
    }
    fn wait_end(&self, _end_time: f64) {}
    fn stop(&self) -> KResult<i64> {
        Ok(1)
    }
}

#[derive(Clone)]
struct FakeCommandWrapper {
    fmt: String,
    sent: Arc<Mutex<Vec<(String, Vec<PyValue>)>>>,
}

impl klippy_api::mcu::CommandWrapperApi for FakeCommandWrapper {
    fn send(&self, data: Vec<PyValue>, _opts: klippy_api::mcu::SendOpts) -> KResult<()> {
        self.sent.lock().push((self.fmt.clone(), data));
        Ok(())
    }
    fn send_wait_ack(
        &self,
        data: Vec<PyValue>,
        _opts: klippy_api::mcu::SendOpts,
    ) -> KResult<()> {
        self.sent.lock().push((self.fmt.clone(), data));
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

#[derive(Clone)]
struct FakeCommandQueryWrapper {
    fmt: String,
    sent: Arc<Mutex<Vec<(String, Vec<PyValue>)>>>,
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeCommandQueryWrapper {
    fn send(
        &self,
        data: Vec<PyValue>,
        _opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        self.sent.lock().push((self.fmt.clone(), data));
        Ok(PyValue::Dict(IndexMap::new()))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        self.send(data, opts)
    }
}

struct FakeAsyncResponseWrapper;

impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsyncResponseWrapper {
    fn unregister(&self) {}
}

#[derive(Clone)]
struct FakeMcu {
    oid: Arc<Mutex<i64>>,
    fileoutput: Arc<Mutex<bool>>,
    est: Arc<Mutex<f64>>,
    config_cmds: Arc<Mutex<Vec<String>>>,
    config_callbacks: Arc<Mutex<Vec<klippy_api::mcu::ConfigCallback>>>,
    sent: Arc<Mutex<Vec<(String, Vec<PyValue>)>>>,
}

impl FakeMcu {
    fn new() -> Arc<Self> {
        Arc::new(FakeMcu {
            oid: Arc::new(Mutex::new(0)),
            fileoutput: Arc::new(Mutex::new(false)),
            est: Arc::new(Mutex::new(1e9)),
            config_cmds: Arc::new(Mutex::new(Vec::new())),
            config_callbacks: Arc::new(Mutex::new(Vec::new())),
            sent: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn set_fileoutput(&self, value: bool) {
        *self.fileoutput.lock() = value;
    }

    fn set_est(&self, value: f64) {
        *self.est.lock() = value;
    }

    /// Runs the `_build_config` callbacks, like `klippy:mcu_identify` does.
    fn run_config_callbacks(&self) -> KResult<()> {
        let mut callbacks: Vec<klippy_api::mcu::ConfigCallback> =
            self.config_callbacks.lock().drain(..).collect();
        for cb in callbacks.iter_mut() {
            cb()?;
        }
        *self.config_callbacks.lock() = callbacks;
        Ok(())
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by probe_eddy_current")
    }
    fn is_fileoutput(&self) -> bool {
        *self.fileoutput.lock()
    }
    fn setup_pin(
        &self,
        _pin_type: &str,
        _pin_params: klippy_api::mcu::PinParams,
    ) -> KResult<klippy_api::mcu::Pin> {
        panic!("setup_pin is not used by probe_eddy_current")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut oid = self.oid.lock();
        *oid += 1;
        Ok(*oid)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.config_callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(
        &self,
        cmd: &str,
        _opts: klippy_api::mcu::AddConfigCmdOpts,
    ) -> KResult<()> {
        self.config_cmds.lock().push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 1e6) as i64
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
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCommandWrapper {
            fmt: msgformat.to_string(),
            sent: Arc::clone(&self.sent),
        }) as CommandWrapper)
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Ok(Arc::new(FakeCommandQueryWrapper {
            fmt: msgformat.to_string(),
            sent: Arc::clone(&self.sent),
        }) as CommandQueryWrapper)
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
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
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsyncResponseWrapper) as AsyncResponseWrapper
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
        (print_time * 1e6) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1e6
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        *self.est.lock()
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
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Ok(Arc::new(FakeTriggerDispatch {
            steppers: Mutex::new(Vec::new()),
        }))
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakeSensor {
    clients: Clients,
    mcu: Arc<FakeMcu>,
    sps: f64,
}

impl FakeSensor {
    fn new(mcu: Arc<FakeMcu>) -> Arc<Self> {
        Arc::new(FakeSensor {
            clients: Arc::new(Mutex::new(Vec::new())),
            mcu,
            sps: 400.0,
        })
    }

    fn deliver(&self, msg: PyValue) {
        let mut clients = self.clients.lock();
        let mut keep = Vec::new();
        for mut cb in clients.drain(..) {
            match cb(msg.clone()) {
                Ok(true) => keep.push(cb),
                Ok(false) => {}
                Err(_) => {}
            }
        }
        *clients = keep;
    }
}

impl TriggerAnalogSensor for FakeSensor {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_trigger_analog(&self, _oid: i64) -> KResult<()> {
        Ok(())
    }
    fn get_samples_per_second(&self) -> f64 {
        self.sps
    }
    fn lookup_sensor_error(&self, _error_code: i64) -> KResult<String> {
        Ok("Unknown ldc1612 error".to_string())
    }
}

impl LDC1612Api for FakeSensor {
    fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        self.clients.lock().push(cb);
        Ok(())
    }
    fn convert_raw_to_frequency(&self, raw_value: f64) -> f64 {
        raw_value
    }
    fn convert_frequency_to_raw(&self, freq: f64) -> KResult<i64> {
        Ok((freq + 0.5).trunc() as i64)
    }
}

struct FakeProbeSession {
    results: Vec<ProbeResult>,
    ran: Arc<Mutex<bool>>,
    ended: Arc<Mutex<bool>>,
}

impl ProbeSessionApi for FakeProbeSession {
    fn run_probe(&self, _gcmd: &GCmd) -> KResult<()> {
        *self.ran.lock() = true;
        Ok(())
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(self.results.clone())
    }
    fn end_probe_session(&self) -> KResult<()> {
        *self.ended.lock() = true;
        Ok(())
    }
}

struct FakeProbeHost {
    sessions: Arc<Mutex<Vec<(Arc<FakeProbeSession>, Vec<IndexMap<String, String>>)>>>,
    offsets: (f64, f64, f64),
    results: Vec<ProbeResult>,
}

impl PrinterProbeApi for FakeProbeHost {
    fn get_probe_params(&self, _gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        Ok(ProbeParams {
            probe_speed: 5.0,
            lift_speed: 4.0,
            samples: 1,
            sample_retract_dist: 0.0,
            samples_tolerance: 0.1,
            samples_tolerance_retries: 0,
            samples_result: "average".to_string(),
        })
    }
    fn get_offsets(&self, _gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        Ok(self.offsets)
    }
    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        let session = Arc::new(FakeProbeSession {
            results: self.results.clone(),
            ran: Arc::new(Mutex::new(false)),
            ended: Arc::new(Mutex::new(false)),
        });
        self.sessions
            .lock()
            .push((Arc::clone(&session), vec![gcmd.get_command_parameters()]));
        Ok(session as Arc<dyn ProbeSessionApi>)
    }
}

klippy_api::printer_object!(FakeProbeHost; dyn PrinterProbeApi);

struct FakeProbeClient {
    clients: Clients,
}

impl FakeProbeClient {
    fn deliver(&self, msg: PyValue) {
        let mut clients = self.clients.lock();
        let mut keep = Vec::new();
        for mut cb in clients.drain(..) {
            match cb(msg.clone()) {
                Ok(true) => keep.push(cb),
                Ok(false) => {}
                Err(_) => {}
            }
        }
        *clients = keep;
    }
}

impl PrinterEddyProbeApi for FakeProbeClient {
    fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        self.clients.lock().push(cb);
        Ok(())
    }
    fn register_drift_compensation(
        &self,
        _comp: Arc<dyn extras::probe_eddy_current::DriftCompensationApi>,
    ) {
    }
}

klippy_api::printer_object!(FakeProbeClient; dyn PrinterEddyProbeApi);

// ---------------------------------------------------------------------------
// World builders.
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    gcode: Arc<GCodeDispatch>,
    toolhead: Arc<FakeToolhead>,
    homing: Arc<FakeHoming>,
    configfile: Arc<FakeConfigFile>,
    config: Config,
    sensor: Arc<FakeSensor>,
    mcu: Arc<FakeMcu>,
    probe_client: Arc<FakeProbeClient>,
    trace: Trace,
}

/// `trigger_analog.MCU_trigger_analog(sensor)`: its `_build_config` (the
/// `klippy:mcu_identify` pass) creates the default SOS filter the probe
/// sessions expect to find.
fn trigger_analog(world: &World) -> Arc<MCU_trigger_analog> {
    let trigger =
        MCU_trigger_analog::new(world.sensor.clone() as Arc<dyn TriggerAnalogSensor>)
            .expect("trigger analog");
    world.mcu.run_config_callbacks().expect("mcu config callbacks");
    trigger
}

fn cal_string() -> String {
    (0..20)
        .map(|i| {
            format!(
                "{:.3}:{:.0}",
                i as f64 * 0.1,
                10_000_000.0 - i as f64 * 1000.0
            )
        })
        .collect::<Vec<_>>()
        .join(",")
}

fn edge_sections(cal: Option<&str>, extra: &[(&str, &str)]) -> Sections {
    let mut sections: IndexMap<String, IndexMap<String, String>> = IndexMap::new();
    // `manual_probe::ManualProbe` reads `[printer] kinematics` unconditionally.
    let mut printer = IndexMap::new();
    printer.insert("kinematics".to_string(), "cartesian".to_string());
    sections.insert("printer".to_string(), printer);
    let mut stepper_z = IndexMap::new();
    stepper_z.insert("position_min".to_string(), "0.0".to_string());
    stepper_z.insert("position_endstop".to_string(), "5.0".to_string());
    sections.insert("stepper_z".to_string(), stepper_z);
    if let Some(cal) = cal {
        let mut opts = IndexMap::new();
        for (name, value) in [
            ("calibrate", cal.to_string()),
            ("sensor_type", "ldc1612".to_string()),
            ("descend_z", "1.0".to_string()),
            ("x_offset", "2.0".to_string()),
            ("y_offset", "1.0".to_string()),
            ("tap_z_offset", "0.0".to_string()),
            ("tap_threshold", "1000.0".to_string()),
        ] {
            opts.insert(name.to_string(), value);
        }
        for (name, value) in extra {
            if value.is_empty() {
                opts.shift_remove(*name);
            } else {
                opts.insert(name.to_string(), value.to_string());
            }
        }
        sections.insert("probe_eddy_current eddy".to_string(), opts);
    }
    Arc::new(Mutex::new(sections))
}

fn make_msg(data: &[(f64, f64, f64)]) -> PyValue {
    let items: Vec<PyValue> = data
        .iter()
        .map(|(t, f, z)| {
            PyValue::Tuple(vec![
                PyValue::Float(*t),
                PyValue::Float(*f),
                PyValue::Float(*z),
            ])
        })
        .collect();
    let mut map = IndexMap::new();
    map.insert(PyValue::Str("data".to_string()), PyValue::List(items));
    map.insert(
        PyValue::Str("errors".to_string()),
        PyValue::Int(PyInt::from_i64(0)),
    );
    map.insert(
        PyValue::Str("overflows".to_string()),
        PyValue::Int(PyInt::from_i64(0)),
    );
    PyValue::Dict(map)
}

fn rust_world(
    cal: Option<&str>,
    extra: &[(&str, &str)],
    offset: f64,
    position: Vec<f64>,
    settings: &[(&str, &[(&str, PyValue)])],
) -> World {
    let printer = FakePrinter::new();
    let gcode = Arc::new(GCodeDispatch::new(Arc::clone(&printer) as Printer));
    let trace: Trace = Arc::new(Mutex::new(Vec::new()));
    let trace_cb = Arc::clone(&trace);
    gcode.register_output_handler(Box::new(move |msg| {
        trace_cb.lock().push(msg);
    }));
    let mcu = FakeMcu::new();
    let sensor = FakeSensor::new(mcu.clone());
    let toolhead = FakeToolhead::new(position, Arc::new(|_| 0.0));
    let homing = Arc::new(FakeHoming {
        calls: Arc::new(Mutex::new(Vec::new())),
        position: Arc::clone(&toolhead.position),
    });
    let configfile = FakeConfigFile::new(settings);
    let gcode_move = Arc::new(FakeGCodeMove { offset });
    let pins = Arc::new(FakePins {
        chips: Arc::new(Mutex::new(Vec::new())),
    });
    let probe_host = Arc::new(FakeProbeHost {
        sessions: Arc::new(Mutex::new(Vec::new())),
        offsets: (3.0, 4.0, 5.0),
        results: Vec::new(),
    });
    let probe_client = Arc::new(FakeProbeClient {
        clients: Arc::new(Mutex::new(Vec::new())),
    });
    printer.add("gcode", Arc::clone(&gcode) as ObjectRef);
    printer.add("toolhead", Arc::clone(&toolhead) as ObjectRef);
    printer.add("gcode_move", gcode_move as ObjectRef);
    printer.add("configfile", Arc::clone(&configfile) as ObjectRef);
    printer.add("homing", Arc::clone(&homing) as ObjectRef);
    printer.add("pins", pins as ObjectRef);
    printer.add("mcu", Arc::clone(&mcu) as ObjectRef);
    printer.add("probe", probe_host as ObjectRef);
    printer.add(
        "probe_eddy_current eddy",
        Arc::clone(&probe_client) as ObjectRef,
    );
    let config = FakeConfig::make(&printer, "probe_eddy_current eddy", edge_sections(cal, extra));
    World {
        printer,
        gcode,
        toolhead,
        homing,
        configfile,
        config,
        sensor,
        mcu,
        probe_client,
        trace,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(SETUP).expect("probe_eddy_current python fakes");
    Some(py)
}

fn rec(label: &str) -> Record {
    Record {
        v: 1,
        unit: "probe_eddy_current".to_string(),
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

// Floats compare by decoded bits: CPython's `float.hex()` always prints 13 hex digits while
// the port's formatter trims trailing zero nibbles, so the raw `FloatVal` spellings differ.
fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    match expected {
        Ok(expected) => {
            if let Err(d) = assert_equal(&rec(label), "ret", &expected, &actual) {
                panic!(
                    "{label}: {d}; expected {} got {}",
                    expected.render(),
                    actual.render()
                );
            }
        }
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    }
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match expected {
        Ok(value) => panic!("{label}: oracle returned {} but the port must fail", value.render()),
        Err(expected) => match actual {
            Ok(()) => panic!(
                "{label}: the port returned Ok but the oracle raised {}: {}",
                expected.class_name, expected.message
            ),
            Err(actual) => {
                assert_eq!(actual.class_name(), expected.class_name, "{label}: exception class");
                assert_eq!(actual.message(), expected.message.as_str(), "{label}: exception message");
            }
        },
    }
}

fn floats(values: &[f64]) -> Value {
    Value::List(values.iter().map(|v| Value::float(*v)).collect())
}

fn floats_list(values: &[Vec<f64>]) -> Value {
    Value::List(values.iter().map(|v| floats(v)).collect())
}

fn strings(values: &[String]) -> Value {
    Value::List(values.iter().map(|v| Value::str(v)).collect())
}

fn writes_value(writes: &[(String, String, String)]) -> Value {
    Value::List(
        writes
            .iter()
            .map(|(a, b, c)| Value::Tuple(vec![Value::str(a), Value::str(b), Value::str(c)]))
            .collect(),
    )
}

fn probe_result_value(result: &ProbeResult) -> Value {
    Value::Tuple(vec![
        Value::float(result.bed_x),
        Value::float(result.bed_y),
        Value::float(result.bed_z),
        Value::float(result.test_x),
        Value::float(result.test_y),
        Value::float(result.test_z),
    ])
}

fn probe_results_value(results: &[ProbeResult]) -> Value {
    Value::List(results.iter().map(probe_result_value).collect())
}

fn log_value(trace: &Trace) -> Value {
    strings(&trace.lock().clone())
}

/// `[(pos, (freq_avg, mad_hz, count))]`, the dict `calc_freqs` returns.
fn positions_value(
    positions: &extras::probe_eddy_current::CalibrationPositions,
) -> Value {
    Value::List(
        positions
            .iter()
            .map(|(pos, (avg, mad, count))| {
                Value::Tuple(vec![
                    Value::float(pos),
                    Value::Tuple(vec![
                        Value::float(*avg),
                        Value::float(*mad),
                        Value::int(*count),
                    ]),
                ])
            })
            .collect(),
    )
}

fn filtered_value(filtered: &[(f64, f64, f64, f64)]) -> Value {
    Value::List(
        filtered
            .iter()
            .map(|(pos, freq, mad_hz, mad_mm)| {
                Value::Tuple(vec![
                    Value::float(*pos),
                    Value::float(*freq),
                    Value::float(*mad_hz),
                    Value::float(*mad_mm),
                ])
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

/// Constants and the module surface need no oracle.
#[test]
fn surface_and_factory() {
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.probe_eddy_current"), "entry points of extras.probe_eddy_current");
}

#[test]
fn parity() {
    let Some(mut py) = oracle() else {
        eprintln!("probe_eddy_current parity: KLIPPY_PYTHON is not set, no oracle available");
        return;
    };
    let cal = cal_string();

    // Module constants.
    check(
        "OUT_OF_RANGE",
        py.eval("pec.OUT_OF_RANGE"),
        Value::float(extras::probe_eddy_current::OUT_OF_RANGE),
    );
    check(
        "MAX_VALID_RAW_VALUE",
        py.eval("pec.MAX_VALID_RAW_VALUE"),
        Value::int(extras::probe_eddy_current::MAX_VALID_RAW_VALUE),
    );

    // -----------------------------------------------------------------
    // EddyCalibration: the `calibrate` table, interpolation, drift comp.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         cal0 = pec.EddyCalibration(build()['c'])"
    ))
    .expect("python calibration");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("calibration");
    let empty_config = rust_world(None, &[], 0.0, vec![0.0, 0.0, 0.0, 0.0], &[]);
    let empty_calibration = EddyCalibration::new(&empty_config.config).expect("empty calibration");
    let (freqs, zpos) = calibration.get_calibration();
    check("calibration freqs", py.eval("list(cal.cal_freqs)"), floats(&freqs));
    check("calibration zpos", py.eval("list(cal.cal_zpos)"), floats(&zpos));
    let (empty_freqs, empty_zpos) = empty_calibration.get_calibration();
    check("empty freqs", py.eval("list(cal0.cal_freqs)"), floats(&empty_freqs));
    check("empty zpos", py.eval("list(cal0.cal_zpos)"), floats(&empty_zpos));
    check_err(
        "verify uncalibrated",
        py.eval("cal0.verify_calibrated()"),
        empty_calibration.verify_calibrated(),
    );
    check(
        "verify calibrated",
        py.eval("cal.verify_calibrated()"),
        Value::Null,
    );
    calibration.verify_calibrated().expect("calibrated");

    // freq_to_height: below the first entry, between entries, above the last.
    for (i, freq) in [0.0f64, 9_999_500.0, 9_995_500.0, 5.0e8]
        .iter()
        .enumerate()
    {
        check(
            &format!("freq_to_height {i}"),
            py.eval(&format!("cal.freq_to_height({freq:?})")),
            Value::float(calibration.freq_to_height(*freq).expect("freq_to_height")),
        );
    }
    // height_to_freq: valid heights and the two out-of-table errors.
    for (i, height) in [0.05f64, 0.55, 1.85].iter().enumerate() {
        check(
            &format!("height_to_freq {i}"),
            py.eval(&format!("cal.height_to_freq({height:?})")),
            Value::float(calibration.height_to_freq(*height).expect("height_to_freq")),
        );
    }
    for (label, height) in [("height_to_freq low", -0.1f64), ("height_to_freq high", 5.0)] {
        check_err(
            label,
            py.eval(&format!("cal.height_to_freq({height:?})")),
            calibration.height_to_freq(height).map(|_| ()),
        );
    }

    // apply_calibration replaces each entry in place.
    let samples: Vec<(f64, f64, f64)> = vec![
        (0.0, 9_999_000.0, 0.0),
        (0.1, 9_995_000.0, 0.0),
        (0.2, 5.0e8, 0.0),
        (0.3, 1.0, 0.0),
    ];
    let mut rust_samples = samples.clone();
    calibration
        .apply_calibration(&mut rust_samples)
        .expect("apply_calibration");
    py.exec("samples = [(0.0, 9999000.0, 0.0), (0.1, 9995000.0, 0.0), \
             (0.2, 5.0e8, 0.0), (0.3, 1.0, 0.0)]\ncal.apply_calibration(samples)")
        .expect("python apply_calibration");
    check(
        "apply_calibration",
        py.eval("samples"),
        Value::List(
            rust_samples
                .iter()
                .map(|(t, f, z)| {
                    Value::Tuple(vec![
                        Value::float(*t),
                        Value::float(*f),
                        Value::float(*z),
                    ])
                })
                .collect(),
        ),
    );

    // Drift compensation: every hook the calibration calls.
    py.exec(
        "class Comp:\n\
        \x20   def __init__(self):\n\
        \x20       self.calls = []\n\
        \x20   def get_temperature(self):\n\
        \x20       self.calls.append('temp')\n\
        \x20       return 30.0\n\
        \x20   def note_z_calibration_start(self):\n\
        \x20       self.calls.append('start')\n\
        \x20   def note_z_calibration_finish(self):\n\
        \x20       self.calls.append('finish')\n\
        \x20   def adjust_freq(self, freq, temp=None):\n\
        \x20       self.calls.append('adjust')\n\
        \x20       return freq + 500.0\n\
        \x20   def unadjust_freq(self, freq, temp=None):\n\
        \x20       self.calls.append('unadjust')\n\
        \x20       return freq - 500.0\n\
         comp = Comp()\n\
         cal.register_drift_compensation(comp)\n\
         cal.note_z_calibration_start()\n\
         cal.note_z_calibration_finish()\n\
         comp_samples = [(0.0, 9996000.0, 0.0)]\n\
         cal.apply_calibration(comp_samples)\n\
         comp_height = cal.freq_to_height(9996000.0)\n\
         comp_freq = cal.height_to_freq(0.5)",
    )
    .expect("python drift comp");
    let calls = Arc::new(Mutex::new(Vec::<String>::new()));
    struct RecordingComp {
        calls: Arc<Mutex<Vec<String>>>,
    }
    impl extras::probe_eddy_current::DriftCompensationApi for RecordingComp {
        fn get_temperature(&self) -> KResult<f64> {
            self.calls.lock().push("temp".to_string());
            Ok(30.0)
        }
        fn note_z_calibration_start(&self) -> KResult<()> {
            self.calls.lock().push("start".to_string());
            Ok(())
        }
        fn note_z_calibration_finish(&self) -> KResult<()> {
            self.calls.lock().push("finish".to_string());
            Ok(())
        }
        fn adjust_freq(&self, freq: f64, _temp: Option<f64>) -> KResult<f64> {
            self.calls.lock().push("adjust".to_string());
            Ok(freq + 500.0)
        }
        fn unadjust_freq(&self, freq: f64, _temp: Option<f64>) -> KResult<f64> {
            self.calls.lock().push("unadjust".to_string());
            Ok(freq - 500.0)
        }
    }
    calibration.register_drift_compensation(Arc::new(RecordingComp {
        calls: Arc::clone(&calls),
    }));
    calibration.note_z_calibration_start().expect("note start");
    calibration
        .note_z_calibration_finish()
        .expect("note finish");
    let mut comp_samples = vec![(0.0, 9_996_000.0, 0.0)];
    calibration
        .apply_calibration(&mut comp_samples)
        .expect("apply with drift");
    let comp_height = calibration.freq_to_height(9_996_000.0).expect("comp height");
    let comp_freq = calibration.height_to_freq(0.5).expect("comp freq");
    check(
        "drift sample",
        py.eval("comp_samples"),
        Value::List(
            comp_samples
                .iter()
                .map(|(t, f, z)| {
                    Value::Tuple(vec![
                        Value::float(*t),
                        Value::float(*f),
                        Value::float(*z),
                    ])
                })
                .collect(),
        ),
    );
    check("drift freq_to_height", py.eval("comp_height"), Value::float(comp_height));
    check("drift height_to_freq", py.eval("comp_freq"), Value::float(comp_freq));
    check(
        "drift calls",
        py.eval("comp.calls"),
        strings(&calls.lock().clone()),
    );

    // -----------------------------------------------------------------
    // EddyCalibrationTool: saving, calc_freqs/median, validation, commands.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?})\n\
         tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))"
    ))
    .expect("python tool");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let tool = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("cal") as Arc<dyn EddyCalibrationApi>,
    )
    .expect("tool");
    // Registering the tool registered both commands on the real dispatch.
    check(
        "tool command help",
        py.eval("sorted(k for k in w['g'].gcode_help if k.startswith(('PROBE_EDDY_CURRENT', 'Z_OFFSET')))"),
        strings(
            &w.gcode
                .get_command_help()
                .keys()
                .filter(|k| k.starts_with("PROBE_EDDY_CURRENT") || k.starts_with("Z_OFFSET"))
                .cloned()
                .collect::<Vec<String>>(),
        ),
    );

    // _save_calibration: the config string and the response text.
    let pairs: Vec<(f64, f64)> = vec![
        (0.0, 9_999_000.0),
        (0.1, 9_998_000.0),
        (0.2, 9_997_000.0),
        (0.3, 9_996_000.0),
    ];
    tool.save_calibration(&pairs).expect("save calibration");
    py.exec(&format!(
        "tool._save_calibration([{}])",
        pairs
            .iter()
            .map(|(a, b)| format!("({a:?}, {b:?})"))
            .collect::<Vec<_>>()
            .join(", ")
    ))
    .expect("python save calibration");
    check(
        "save writes",
        py.eval("[tuple(x) for x in w['cf'].writes]"),
        writes_value(&w.configfile.writes.lock().clone()),
    );
    check("save log", py.eval("LOG"), log_value(&w.trace));
    check_err(
        "save empty",
        py.eval("tool._save_calibration([])"),
        tool.save_calibration(&[]).map(|_| ()),
    );

    // _median and calc_freqs.
    check("median odd", py.eval("tool._median([3.0, 1.0, 2.0])"), Value::float(tool.median(&[3.0, 1.0, 2.0]).expect("median odd")));
    check(
        "median even",
        py.eval("tool._median([4.0, 1.0, 3.0, 2.0])"),
        Value::float(tool.median(&[4.0, 1.0, 3.0, 2.0]).expect("median even")),
    );
    check_err("median empty", py.eval("tool._median([])"), tool.median(&[]).map(|_| ()));
    py.exec(
        "meas = {0.0: [10000000.0, 10000020.0, 10000010.0],\n\
        \x20       0.1: [9999000.0, 9998980.0]}\n\
         positions = tool.calc_freqs(meas)",
    )
    .expect("python calc_freqs");
    let mut rust_meas = extras::probe_eddy_current::F64Map::new();
    for (pos, freqs) in [
        (0.0, vec![10_000_000.0, 10_000_020.0, 10_000_010.0]),
        (0.1, vec![9_999_000.0, 9_998_980.0]),
    ] {
        for freq in freqs {
            rust_meas.push(pos, freq);
        }
    }
    let positions = tool.calc_freqs(&rust_meas).expect("calc_freqs");
    check(
        "calc_freqs",
        py.eval("[(k, v) for k, v in positions.items()]"),
        positions_value(&positions),
    );

    // validate_calibration_data: the messages and the filtered table.
    py.exec(
        "positions2 = {0.0: (10000000.0, 10.0, 3), 0.1: (9999000.0, 12.0, 3),\n\
        \x20              0.2: (9997000.0, 20.0, 3), 0.3: (9994000.0, 40.0, 3)}\n\
         filtered = tool.validate_calibration_data(positions2)",
    )
    .expect("python validate");
    let mut positions2 = extras::probe_eddy_current::CalibrationPositions::new();
    positions2.insert(0.0, (10_000_000.0, 10.0, 3));
    positions2.insert(0.1, (9_999_000.0, 12.0, 3));
    positions2.insert(0.2, (9_997_000.0, 20.0, 3));
    positions2.insert(0.3, (9_994_000.0, 40.0, 3));
    let filtered = tool
        .validate_calibration_data(&positions2)
        .expect("validate");
    check("validate filtered", py.eval("filtered"), filtered_value(&filtered));
    check("validate log", py.eval("LOG"), log_value(&w.trace));

    // A frequency that stops decreasing stops the loop early.
    w.trace.lock().clear();
    py.exec(
        "del LOG[:]\n\
         noisy = {0.0: (10000000.0, 10.0, 3), 0.1: (10000001.0, 12.0, 3)}\n\
         noisy_out = tool.validate_calibration_data(noisy)",
    )
    .expect("python noisy");
    let mut noisy = extras::probe_eddy_current::CalibrationPositions::new();
    noisy.insert(0.0, (10_000_000.0, 10.0, 3));
    noisy.insert(0.1, (10_000_001.0, 12.0, 3));
    let noisy_out = tool.validate_calibration_data(&noisy).expect("noisy");
    check("validate noisy", py.eval("noisy_out"), filtered_value(&noisy_out));
    check("validate noisy log", py.eval("LOG"), log_value(&w.trace));
    // No usable entry at all: the averages divide by zero.
    check_err(
        "validate empty",
        py.eval("tool.validate_calibration_data({})"),
        tool.validate_calibration_data(&extras::probe_eddy_current::CalibrationPositions::new())
            .map(|_| ()),
    );

    // Z_OFFSET_APPLY_PROBE: zero offset, table shift, and METHOD=tap.
    let w = rust_world(
        Some(&cal),
        &[],
        0.25,
        vec![10.0, 10.0, 2.0, 0.0],
        &[(
            "probe_eddy_current eddy",
            &[("tap_z_offset", PyValue::Float(0.0))],
        )],
    );
    py.exec(&format!(
        "w = build({cal:?}, offset=0.25, settings={{'probe_eddy_current eddy': {{'tap_z_offset': 0.0}}}})\n\
         tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))"
    ))
    .expect("python z offset");
    let tool = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("cal") as Arc<dyn EddyCalibrationApi>,
    )
    .expect("tool");
    let gcode_command = w
        .gcode
        .create_gcode_command("Z_OFFSET_APPLY_PROBE", "", &IndexMap::new());
    tool.cmd_z_offset_apply_probe(&gcode_command)
        .expect("z offset apply");
    py.exec("tool.cmd_Z_OFFSET_APPLY_PROBE(w['g'].create_gcode_command('Z_OFFSET_APPLY_PROBE', '', {}))")
        .expect("python z offset apply");
    check(
        "z offset writes",
        py.eval("[tuple(x) for x in w['cf'].writes]"),
        writes_value(&w.configfile.writes.lock().clone()),
    );
    check("z offset log", py.eval("LOG"), log_value(&w.trace));

    // METHOD=tap writes `tap_z_offset`.
    let mut tap_params = IndexMap::new();
    tap_params.insert("METHOD".to_string(), "TAP".to_string());
    let gcode_command = w
        .gcode
        .create_gcode_command("Z_OFFSET_APPLY_PROBE", "", &tap_params);
    tool.cmd_z_offset_apply_probe(&gcode_command)
        .expect("z offset tap");
    py.exec("tool.cmd_Z_OFFSET_APPLY_PROBE(w['g'].create_gcode_command('Z_OFFSET_APPLY_PROBE', '', {'METHOD': 'TAP'}))")
        .expect("python z offset tap");
    check(
        "z offset tap writes",
        py.eval("[tuple(x) for x in w['cf'].writes]"),
        writes_value(&w.configfile.writes.lock().clone()),
    );
    check("z offset tap log", py.eval("LOG"), log_value(&w.trace));

    // A zero homing offset does nothing.
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    py.exec(&format!(
        "w = build({cal:?}, offset=0.0)\n\
         tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))"
    ))
    .expect("python zero offset");
    let tool = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("cal") as Arc<dyn EddyCalibrationApi>,
    )
    .expect("tool");
    let gcode_command = w
        .gcode
        .create_gcode_command("Z_OFFSET_APPLY_PROBE", "", &IndexMap::new());
    tool.cmd_z_offset_apply_probe(&gcode_command)
        .expect("zero offset");
    py.exec("tool.cmd_Z_OFFSET_APPLY_PROBE(w['g'].create_gcode_command('Z_OFFSET_APPLY_PROBE', '', {}))")
        .expect("python zero offset");
    check(
        "zero offset writes",
        py.eval("[tuple(x) for x in w['cf'].writes]"),
        writes_value(&w.configfile.writes.lock().clone()),
    );
    check("zero offset log", py.eval("LOG"), log_value(&w.trace));

    // do_calibration_moves: the 101 descending hops, the query windows and
    // the correlated frequency table.
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let tool_moves = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("cal") as Arc<dyn EddyCalibrationApi>,
    )
    .expect("tool");
    {
        let client = Arc::clone(&w.probe_client);
        w.toolhead.set_move_hook(Arc::new(move |print_time: f64| {
            client.deliver(make_msg(&[(print_time + 0.1, 9_999_000.0, 999.9)]));
        }));
    }
    let cal_map = tool_moves
        .do_calibration_moves(5.0)
        .expect("do_calibration_moves");
    py.exec(&format!(
        "w = build({cal:?})\n\
         tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))\n\
         w['th'].sensor.auto = lambda t, n: [(t, 9999000.0, 999.9)]\n\
         cal_map = tool.do_calibration_moves(5.0)"
    ))
    .expect("python calibration moves");
    check(
        "calibration moves",
        py.eval("[(k, v) for k, v in cal_map.items()]"),
        Value::List(
            cal_map
                .iter()
                .map(|(pos, freqs)| Value::Tuple(vec![Value::float(pos), floats(freqs)]))
                .collect(),
        ),
    );
    // Without sensor data the correlation cannot cover every step.
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let tool_moves = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("cal") as Arc<dyn EddyCalibrationApi>,
    )
    .expect("tool");
    py.exec(&format!(
        "w = build({cal:?})\n\
         tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))"
    ))
    .expect("python calibration moves incomplete");
    check_err(
        "calibration moves incomplete",
        py.eval("tool.do_calibration_moves(5.0)"),
        tool_moves.do_calibration_moves(5.0).map(|_| ()),
    );

    // PROBE_EDDY_CURRENT_CALIBRATE refuses PROBE_SPEED=0 before it starts
    // the manual probe (the success path needs `manual_probe` plumbing).
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    py.exec(&format!(
        "w = build({cal:?})\n\
         tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))\n\
         tool_calibrate = w['g'].create_gcode_command('PROBE_EDDY_CURRENT_CALIBRATE', '', {{'PROBE_SPEED': '0'}})"
    ))
    .expect("python eddy calibrate");
    let tool_arc = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("cal") as Arc<dyn EddyCalibrationApi>,
    )
    .expect("tool arc");
    let mut params = IndexMap::new();
    params.insert("PROBE_SPEED".to_string(), "0".to_string());
    let gcmd_zero = w
        .gcode
        .create_gcode_command("PROBE_EDDY_CURRENT_CALIBRATE", "", &params);
    check_err(
        "eddy calibrate speed 0",
        py.eval("tool.cmd_EDDY_CALIBRATE(tool_calibrate)"),
        EddyCalibrationTool::cmd_eddy_calibrate(&tool_arc, &gcmd_zero).map(|_| ()),
    );

    // -----------------------------------------------------------------
    // TapBestFit: least squares, matrix shapes, and the flat-data failure.
    // -----------------------------------------------------------------
    py.exec(
        "fit = pec.TapBestFit()\n\
         data = [(9990000.0 + 3000.0 * i - 200.0 * i * i, (i * 0.05, i * 0.05, 0.30 + i * 0.02)) for i in range(24)]\n\
         coeffs = fit.find_best_fit(data)\n\
         samples = [(d[1][2] - 0.5, d[0] - 9990000.0) for d in data]\n\
         matrix_full = fit._build_ls_matrix(samples, 0.4)\n\
         matrix_opt = fit._build_ls_matrix_opt(samples, 0.4)",
    )
    .expect("python tap fit");
    let fit = TapBestFit::new();
    let data: Vec<(f64, Vec<f64>)> = (0..24)
        .map(|i| {
            let i = i as f64;
            (
                9_990_000.0 + 3000.0 * i - 200.0 * i * i,
                vec![i * 0.05, i * 0.05, 0.30 + i * 0.02],
            )
        })
        .collect();
    let coeffs = fit.find_best_fit(&data).expect("find_best_fit");
    check(
        "tap coeffs",
        py.eval("coeffs"),
        Value::Tuple(coeffs.iter().map(|v| Value::float(*v)).collect()),
    );
    let samples: Vec<(f64, f64)> = data
        .iter()
        .map(|(freq, pos)| (pos[2] - 0.5, freq - 9_990_000.0))
        .collect();
    let (full_eqs, full_ans) = fit.build_ls_matrix(&samples, 0.4).expect("build_ls_matrix");
    let (opt_eqs, opt_ans) = fit
        .build_ls_matrix_opt(&samples, 0.4)
        .expect("build_ls_matrix_opt");
    let matrix_value = |eqs: &Vec<Vec<f64>>, ans: &Vec<Vec<f64>>| {
        Value::Tuple(vec![floats_list(eqs), floats_list(ans)])
    };
    check(
        "tap matrix full",
        py.eval("matrix_full"),
        matrix_value(&full_eqs, &full_ans),
    );
    // The reference stores `num_samples` (a Python int) at `eqst_eqs[0][0]`
    // (`probe_eddy_current.py:626`). That cell only feeds float arithmetic in
    // `mathutil.gaussian_solve`, where `int * float` equals `float(int) * float`
    // exactly, so the port's `f64` matrix carries the same value. The oracle
    // matrix is compared after `float()` on that one cell, and the cell's
    // integer value is compared on its own.
    check(
        "tap matrix opt",
        py.eval(
            "([[float(v) if (i, j) == (0, 0) else v for j, v in enumerate(row)] \
             for i, row in enumerate(matrix_opt[0])], matrix_opt[1])",
        ),
        matrix_value(&opt_eqs, &opt_ans),
    );
    check(
        "tap matrix opt sample count",
        py.eval("matrix_opt[0][0][0]"),
        Value::int(opt_eqs[0][0] as i64),
    );
    // All samples at the same z: the search loop never runs and the flat
    // `[0., 0., 0., 0.]` placeholder raises TypeError in the reference.
    check_err(
        "tap flat data",
        py.eval("fit.find_best_fit([(1.0, (0.0, 0.0, 0.5)), (2.0, (0.0, 0.0, 0.5))])"),
        TapBestFit::new()
            .find_best_fit(&[(1.0, vec![0.0, 0.0, 0.5]), (2.0, vec![0.0, 0.0, 0.5])])
            .map(|_| ()),
    );
    check_err(
        "tap empty data",
        py.eval("fit.find_best_fit([])"),
        TapBestFit::new().find_best_fit(&[]).map(|_| ()),
    );

    // -----------------------------------------------------------------
    // EddyGatherSamples: windows, deferred errors, outage, file output.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?})\n\
         gather = pec.EddyGatherSamples(w['p'], w['sensor'])"
    ))
    .expect("python gather");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let gather = EddyGatherSamples::new(w.printer.clone() as Printer, w.sensor.clone() as Arc<dyn LDC1612Api>)
        .expect("gather");
    let messages: Vec<(f64, f64, f64)> = vec![
        (0.00, 1.0, 0.0),
        (0.02, 2.0, 0.0),
        (0.05, 3.0, 0.0),
        (0.09, 4.0, 0.0),
        (0.20, 5.0, 0.0),
    ];
    let messages_py: Vec<String> = messages
        .iter()
        .map(|(t, f, z)| format!("({t:?}, {f:?}, {z:?})"))
        .collect();
    py.exec(&format!(
        "w['sensor'].deliver(msg([{}]))",
        messages_py.join(", ")
    ))
    .expect("python gather deliver");
    w.sensor.deliver(make_msg(&messages));
    check(
        "pull_measurements",
        py.eval("gather._pull_measurements(0.01, 0.10)"),
        Value::List(
            gather
                .pull_measurements(0.01, 0.10)
                .expect("pull_measurements")
                .iter()
                .map(|(t, f, z)| {
                    Value::Tuple(vec![
                        Value::float(*t),
                        Value::float(*f),
                        Value::float(*z),
                    ])
                })
                .collect(),
        ),
    );

    // A `command_error` inside the analysis callback is deferred to
    // `pull_probed()`, other exceptions propagate immediately.
    py.exec(&format!(
        "w = build({cal:?})\n\
         gather = pec.EddyGatherSamples(w['p'], w['sensor'])\n\
         def bad_cb(measures):\n\
        \x20   raise w['p'].command_error('bad analysis')\n\
         gather.add_probe_request(bad_cb, 0.0, 0.05)\n\
         w['sensor'].deliver(msg([(0.0, 1.0, 0.0), (0.06, 1.0, 0.0)]))",
    ))
    .expect("python deferred error");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let gather = EddyGatherSamples::new(w.printer.clone() as Printer, w.sensor.clone() as Arc<dyn LDC1612Api>)
        .expect("gather");
    let bad: extras::probe_eddy_current::ProbeRequestCallback =
        Arc::new(|_measures| Err(KlippyError::command("bad analysis")));
    gather.add_probe_request(bad, 0.0, 0.05).expect("add request");
    w.sensor
        .deliver(make_msg(&[(0.0, 1.0, 0.0), (0.06, 1.0, 0.0)]));
    check_err(
        "deferred error",
        py.eval("gather.pull_probed()"),
        gather.pull_probed().map(|_| ()),
    );

    // File output produces a dummy result immediately.
    py.exec(&format!(
        "w = build({cal:?})\n\
         w['mcu'].is_fileout = True\n\
         w['mcu'].est = 0.0\n\
         gather = pec.EddyGatherSamples(w['p'], w['sensor'])\n\
         gather.add_probe_request(lambda measures: 1, 0.0, 0.05)\n\
         file_results = gather.pull_probed()"
    ))
    .expect("python fileoutput");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    w.mcu.set_fileoutput(true);
    w.mcu.set_est(0.0);
    let gather = EddyGatherSamples::new(w.printer.clone() as Printer, w.sensor.clone() as Arc<dyn LDC1612Api>)
        .expect("gather");
    let never: extras::probe_eddy_current::ProbeRequestCallback =
        Arc::new(|_measures| Err(KlippyError::command("never called")));
    gather.add_probe_request(never, 0.0, 0.05).expect("add request");
    let file_results = gather.pull_probed().expect("file output pull");
    check(
        "fileoutput results",
        py.eval("file_results"),
        probe_results_value(&file_results),
    );

    // Sensor outage: the estimated print time runs past the window.
    py.exec(&format!(
        "w = build({cal:?})\n\
         w['mcu'].est = 100.0\n\
         gather = pec.EddyGatherSamples(w['p'], w['sensor'])\n\
         gather.add_probe_request(lambda measures: 1, 0.0, 0.05)"
    ))
    .expect("python outage");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    w.mcu.set_est(100.0);
    let gather = EddyGatherSamples::new(w.printer.clone() as Printer, w.sensor.clone() as Arc<dyn LDC1612Api>)
        .expect("gather");
    let never: extras::probe_eddy_current::ProbeRequestCallback =
        Arc::new(|_measures| Err(KlippyError::command("never called")));
    gather.add_probe_request(never, 0.0, 0.05).expect("add request");
    check_err(
        "sensor outage",
        py.eval("gather.pull_probed()"),
        gather.pull_probed().map(|_| ()),
    );

    // -----------------------------------------------------------------
    // probe_results_from_avg and EddyProbeOffsets.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         offsets = pec.EddyProbeOffsets(w['c'])\n\
         avg_result = pec.probe_results_from_avg([(0.0, 9996000.0, 0.0), (0.1, 9998000.0, 0.0)],\n\
        \x20                                       (10.0, 20.0, 2.0), cal, (1.0, 2.0, 0.0))",
    ))
    .expect("python avg");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("cal");
    let offsets = EddyProbeOffsets::new(&w.config).expect("offsets");
    let avg_result = probe_results_from_avg(
        &vec![(0.0, 9_996_000.0, 0.0), (0.1, 9_998_000.0, 0.0)],
        &[10.0, 20.0, 2.0],
        &*calibration,
        (1.0, 2.0, 0.0),
    )
    .expect("probe_results_from_avg");
    check("avg result", py.eval("avg_result"), probe_result_value(&avg_result));
    let (x_offset, y_offset, z_offset) = offsets.get_offsets(None);
    check(
        "offsets",
        py.eval("offsets.get_offsets()"),
        Value::Tuple(vec![
            Value::float(x_offset),
            Value::float(y_offset),
            Value::float(z_offset),
        ]),
    );
    check_err(
        "avg empty",
        py.eval("pec.probe_results_from_avg([], (0.0, 0.0, 0.0), cal, (0.0, 0.0, 0.0))"),
        probe_results_from_avg(&Vec::new(), &[0.0, 0.0, 0.0], &*calibration, (0.0, 0.0, 0.0))
            .map(|_| ()),
    );
    check_err(
        "avg out of range",
        py.eval("pec.probe_results_from_avg([(0.0, 5.0e8, 0.0)], (0.0, 0.0, 0.0), cal, (0.0, 0.0, 0.0))"),
        probe_results_from_avg(&vec![(0.0, 5.0e8, 0.0)], &[0.0, 0.0, 0.0], &*calibration, (0.0, 0.0, 0.0))
            .map(|_| ()),
    );

    // -----------------------------------------------------------------
    // EddyParameterHelper: scan/tap parameters and the config delegation.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?}, extra={{'samples': '3', 'samples_result': 'median'}})\n\
         ph = pec.EddyParameterHelper(w['c'])\n\
         params_none = ph.get_probe_params()\n\
         params_auto = ph.get_probe_params(w['g'].create_gcode_command('PROBE', '', {{'METHOD': 'automatic'}}))\n\
         params_scan = ph.get_probe_params(w['g'].create_gcode_command('PROBE', '', {{'METHOD': 'SCAN', 'PROBE_SPEED': '7.5', 'SAMPLES': '2'}}))\n\
         params_tap = ph.get_probe_params(w['g'].create_gcode_command('PROBE', '', {{'METHOD': 'tap'}}))",
    ))
    .expect("python parameter helper");
    let w = rust_world(
        Some(&cal),
        &[("samples", "3"), ("samples_result", "median")],
        0.0,
        vec![10.0, 10.0, 2.0, 0.0],
        &[],
    );
    let param_helper = EddyParameterHelper::new(&w.config).expect("param helper");
    let params_value = |params: &ProbeParams| {
        Value::Tuple(vec![
            Value::float(params.probe_speed),
            Value::float(params.lift_speed),
            Value::int(params.samples),
            Value::float(params.sample_retract_dist),
            Value::float(params.samples_tolerance),
            Value::int(params.samples_tolerance_retries),
            Value::str(&params.samples_result),
        ])
    };
    let params_none = param_helper.get_probe_params(None).expect("params none");
    check(
        "params none",
        py.eval("params_tuple(params_none)"),
        params_value(&params_none),
    );
    let mut auto_params = IndexMap::new();
    auto_params.insert("METHOD".to_string(), "automatic".to_string());
    let auto_command = w
        .gcode
        .create_gcode_command("PROBE", "", &auto_params);
    let params_auto = param_helper
        .get_probe_params(Some(&auto_command))
        .expect("params automatic");
    check(
        "params automatic",
        py.eval("params_tuple(params_auto)"),
        params_value(&params_auto),
    );
    let mut scan_params = IndexMap::new();
    scan_params.insert("METHOD".to_string(), "SCAN".to_string());
    scan_params.insert("PROBE_SPEED".to_string(), "7.5".to_string());
    scan_params.insert("SAMPLES".to_string(), "2".to_string());
    let scan_command = w.gcode.create_gcode_command("PROBE", "", &scan_params);
    let params_scan = param_helper
        .get_probe_params(Some(&scan_command))
        .expect("params scan");
    check(
        "params scan",
        py.eval("params_tuple(params_scan)"),
        params_value(&params_scan),
    );
    let mut tap_params = IndexMap::new();
    tap_params.insert("METHOD".to_string(), "tap".to_string());
    let tap_command = w.gcode.create_gcode_command("PROBE", "", &tap_params);
    let params_tap = param_helper
        .get_probe_params(Some(&tap_command))
        .expect("params tap");
    check(
        "params tap",
        py.eval("params_tuple(params_tap)"),
        params_value(&params_tap),
    );
    // Scan parameters validate their bounds.
    let mut bad_params = IndexMap::new();
    bad_params.insert("METHOD".to_string(), "scan".to_string());
    bad_params.insert("SAMPLES".to_string(), "0".to_string());
    let bad_command = w.gcode.create_gcode_command("PROBE", "", &bad_params);
    py.exec("params_bad = w['g'].create_gcode_command('PROBE', '', {'METHOD': 'scan', 'SAMPLES': '0'})")
        .expect("python bad params");
    check_err(
        "params scan samples 0",
        py.eval("ph.get_probe_params(params_bad)"),
        param_helper.get_probe_params(Some(&bad_command)).map(|_| ()),
    );

    // -----------------------------------------------------------------
    // EddyTap: configuration, sample-time checks and a full tap analysis.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?}, extra={{'tap_threshold': ''}})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         trig = FakeMcuTrigger(w['sensor'])\n\
         ph = pec.EddyParameterHelper(w['c'])\n\
         tap = pec.EddyTap(w['c'], w['sensor'], ph, trig)"
    ))
    .expect("python tap");
    let w = rust_world(
        Some(&cal),
        &[("tap_threshold", "")],
        0.0,
        vec![10.0, 10.0, 2.0, 0.0],
        &[],
    );
    let param_helper = EddyParameterHelper::new(&w.config).expect("param helper");
    let trig_analog = trigger_analog(&w);
    let tap = EddyTap::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        param_helper,
        trig_analog,
    )
    .expect("tap");
    // `TAP_THRESHOLD` is absent, so the config default (0.0) is used and the
    // tap mode refuses to run.
    let zero_threshold = w
        .gcode
        .create_gcode_command("PROBE", "", &IndexMap::new());
    py.exec("tap_prep = w['g'].create_gcode_command('PROBE', '', {})")
        .expect("python tap prep");
    check_err(
        "tap not configured",
        py.eval("tap._prep_trigger_analog_tap(tap_prep)"),
        tap.prep_trigger_analog_tap(&zero_threshold).map(|_| ()),
    );

    // `_validate_samples_time` messages.
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         trig = FakeMcuTrigger(w['sensor'])\n\
         ph = pec.EddyParameterHelper(w['c'])\n\
         tap = pec.EddyTap(w['c'], w['sensor'], ph, trig)"
    ))
    .expect("python tap validate");
    let param_helper = EddyParameterHelper::new(&w.config).expect("param helper");
    let trig_analog = trigger_analog(&w);
    let tap = EddyTap::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        param_helper,
        trig_analog,
    )
    .expect("tap");
    check_err(
        "tap too close",
        py.eval("tap._validate_samples_time([(0.0, 1.0, 0.0), (0.01, 1.0, 0.0)], 0.0, 0.05)"),
        tap.validate_samples_time(&vec![(0.0, 1.0, 0.0), (0.01, 1.0, 0.0)], 0.0, 0.05),
    );
    check_err(
        "tap too few samples",
        py.eval("tap._validate_samples_time([(0.0, 1.0, 0.0)], 0.0, 0.2)"),
        tap.validate_samples_time(&vec![(0.0, 1.0, 0.0)], 0.0, 0.2),
    );
    let gap = vec![(0.0, 1.0, 0.0), (0.02, 1.0, 0.0), (0.04, 1.0, 0.0)];
    let gap_py: Vec<String> = gap
        .iter()
        .map(|(t, f, z)| format!("({t:?}, {f:?}, {z:?})"))
        .collect();
    check_err(
        "tap gap",
        py.eval(&format!(
            "tap._validate_samples_time([{}], 0.0, 0.1)",
            gap_py.join(", ")
        )),
        tap.validate_samples_time(&gap, 0.0, 0.1),
    );
    let mut dup_times: Vec<f64> = (1..100).map(|k| k as f64 * 0.001).collect();
    dup_times.insert(2, 0.002);
    let dup: Vec<(f64, f64, f64)> = dup_times.iter().map(|t| (*t, 1.0, 0.0)).collect();
    let dup_py: Vec<String> = dup
        .iter()
        .map(|(t, f, z)| format!("({t:?}, {f:?}, {z:?})"))
        .collect();
    check_err(
        "tap clock too low",
        py.eval(&format!(
            "tap._validate_samples_time([{}], 0.0, 0.1)",
            dup_py.join(", ")
        )),
        tap.validate_samples_time(&dup, 0.0, 0.1),
    );

    // Full tap analysis: a modeled pullback with the exact piecewise slope.
    let (zc, fc, dslope, slope, slope2) =
        (0.45f64, 10_000_000.0f64, -30_000.0f64, -60_000.0f64, 200_000.0f64);
    let z_at = |t: f64| 0.3 + (t - 200.0) * 2.4;
    let samples: Vec<(f64, f64, f64)> = (0..66)
        .map(|k| {
            let t = 199.991 + k as f64 * 0.0025;
            let z = z_at(t);
            let dz = z - zc;
            let freq = if z <= zc {
                fc + dslope * dz
            } else {
                fc + slope * dz + slope2 * dz * dz
            };
            (t, freq, 999.9)
        })
        .collect();
    let samples_py: Vec<String> = samples
        .iter()
        .map(|(t, f, z)| format!("({t:?}, {f:?}, {z:?})"))
        .collect();
    py.exec(&format!(
        "w['th'].past_fn = lambda t: 0.3 + (t - 200.0) * 2.4\n\
         tap_start = 199.99\n\
         tap_end = 200.15\n\
         tap_samples = [{}]",
        samples_py.join(", ")
    ))
    .expect("python tap samples");
    let toolhead = FakeToolhead::new(
        vec![10.0, 10.0, 2.0, 0.0],
        Arc::new(|t| 0.3 + (t - 200.0) * 2.4),
    );
    w.printer.add("toolhead", Arc::clone(&toolhead) as ObjectRef);
    // The synthetic pullback lands on whichever branch the least-squares fit
    // reaches; the port has to land on the same one with the same numbers.
    py.exec(
        "try:\n\
        \x20   tap_result = tap._analyze_pullback(tap_samples, tap_start, tap_end)\n\
        \x20   tap_exc = None\n\
         except Exception as e:\n\
        \x20   tap_result = None\n\
        \x20   tap_exc = (type(e).__name__, str(e))",
    )
    .expect("python analyze pullback");
    let rust_result = tap.analyze_pullback(samples.clone(), 199.99, 200.15);
    match py.eval("tap_exc") {
        Ok(Value::Null) => check(
            "tap result",
            py.eval("tuple(tap_result)"),
            probe_result_value(&rust_result.expect("tap analysis succeeded in the reference")),
        ),
        Ok(Value::Tuple(items)) => {
            let class = &items[0];
            let message = &items[1];
            check(
                "tap exception",
                Ok(Value::Tuple(vec![class.clone(), message.clone()])),
                match &rust_result {
                    Ok(result) => panic!(
                        "tap analysis: the port returned {} but the reference raised {}",
                        probe_result_value(result).render(),
                        message.render()
                    ),
                    Err(e) => Value::Tuple(vec![
                        Value::str(e.class_name()),
                        Value::str(e.message()),
                    ]),
                },
            );
        }
        other => panic!("tap_exc: unexpected oracle value {:?}", other),
    }
    match tap.get_last_tap_info() {
        Some(last) => {
            check("tap last status", py.eval("tap._last_tap[0]"), Value::str(&last.0));
            check(
                "tap last depress",
                py.eval("tap._last_tap[1]"),
                Value::float(last.1),
            );
            check(
                "tap last coeffs",
                py.eval("tap._last_tap[2]"),
                Value::Tuple(last.2.iter().map(|v| Value::float(*v)).collect()),
            );
        }
        None => check(
            "tap last info",
            py.eval("tap._last_tap is None"),
            Value::Bool(true),
        ),
    }

    // `_analyze_pullback` refuses a pullback with too little lift.
    let flat_samples: Vec<(f64, f64, f64)> = (0..64)
        .map(|k| (199.991 + k as f64 * 0.0025, fc, 999.9))
        .collect();
    let flat_py: Vec<String> = flat_samples
        .iter()
        .map(|(t, f, z)| format!("({t:?}, {f:?}, {z:?})"))
        .collect();
    let flat_toolhead = FakeToolhead::new(vec![10.0, 10.0, 2.0, 0.0], Arc::new(|_| 0.3));
    w.printer.add("toolhead", Arc::clone(&flat_toolhead) as ObjectRef);
    py.exec(&format!(
        "w['th'].past_fn = lambda t: 0.3\n\
         flat_samples = [{}]",
        flat_py.join(", ")
    ))
    .expect("python flat tap");
    check_err(
        "tap insufficient lift",
        py.eval("tap._analyze_pullback(flat_samples, 199.99, 200.15)"),
        tap.analyze_pullback(flat_samples.clone(), 199.99, 200.15)
            .map(|_| ()),
    );

    // -----------------------------------------------------------------
    // EddyDescend: `descend_z` / deprecated `z_offset`, prep, run, session.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         trig = FakeMcuTrigger(w['sensor'])\n\
         offsets = pec.EddyProbeOffsets(w['c'])\n\
         ph = pec.EddyParameterHelper(w['c'])\n\
         descend = pec.EddyDescend(w['c'], w['sensor'], cal, offsets, ph, trig)\n\
         descend_descend_z = descend._descend_z"
    ))
    .expect("python descend");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("cal");
    let offsets = EddyProbeOffsets::new(&w.config).expect("offsets");
    let param_helper = EddyParameterHelper::new(&w.config).expect("param helper");
    let trig_analog = trigger_analog(&w);
    let descend = EddyDescend::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        calibration.clone() as Arc<dyn EddyCalibrationApi>,
        offsets,
        param_helper,
        trig_analog,
    )
    .expect("descend");
    check(
        "descend_z",
        py.eval("descend_descend_z"),
        Value::float(descend.descend_z()),
    );
    // The descending probe refuses to start before calibration.
    let short_cal = "0.000:10000000,0.100:9999000";
    let empty_world = rust_world(Some(short_cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let empty_cal = EddyCalibration::new(&empty_world.config).expect("cal");
    let empty_offsets = EddyProbeOffsets::new(&empty_world.config).expect("offsets");
    let empty_param = EddyParameterHelper::new(&empty_world.config).expect("param");
    let empty_trig = trigger_analog(&empty_world);
    let empty_descend = EddyDescend::new(
        &empty_world.config,
        empty_world.sensor.clone() as Arc<dyn LDC1612Api>,
        empty_cal as Arc<dyn EddyCalibrationApi>,
        empty_offsets,
        empty_param,
        empty_trig,
    )
    .expect("descend");
    let probe_command = w
        .gcode
        .create_gcode_command("PROBE", "", &IndexMap::new());
    py.exec(&format!(
        "w2 = build({short_cal:?})\n\
         cal2 = pec.EddyCalibration(w2['c'])\n\
         trig2 = FakeMcuTrigger(w2['sensor'])\n\
         offsets2 = pec.EddyProbeOffsets(w2['c'])\n\
         ph2 = pec.EddyParameterHelper(w2['c'])\n\
         descend2 = pec.EddyDescend(w2['c'], w2['sensor'], cal2, offsets2, ph2, trig2)\n\
         probe_gcmd = w2['g'].create_gcode_command('PROBE', '', {{}})"
    ))
    .expect("python empty descend");
    check_err(
        "descend not calibrated",
        py.eval("descend2.start_probe_session(probe_gcmd)"),
        empty_descend.start_probe_session(&probe_command).map(|_| ()),
    );

    // The full descending session: homing call, gathered samples, result.
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         trig = FakeMcuTrigger(w['sensor'])\n\
         offsets = pec.EddyProbeOffsets(w['c'])\n\
         ph = pec.EddyParameterHelper(w['c'])\n\
         descend = pec.EddyDescend(w['c'], w['sensor'], cal, offsets, ph, trig)\n\
         descend.start_probe_session(w['g'].create_gcode_command('PROBE', '', {{}}))\n\
         descend.run_probe(w['g'].create_gcode_command('PROBE', '', {{}}))\n\
         w['sensor'].deliver(msg([(0.1, 9995000.0, 999.9), (0.16, 9995000.0, 999.9)]))\n\
         descend_results = descend.pull_probed_results()"
    ))
    .expect("python descend session");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("cal");
    let offsets = EddyProbeOffsets::new(&w.config).expect("offsets");
    let param_helper = EddyParameterHelper::new(&w.config).expect("param helper");
    let trig_analog = trigger_analog(&w);
    let descend = EddyDescend::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        calibration as Arc<dyn EddyCalibrationApi>,
        offsets,
        param_helper,
        trig_analog,
    )
    .expect("descend");
    let probe_gcmd = w
        .gcode
        .create_gcode_command("PROBE", "", &IndexMap::new());
    descend
        .start_probe_session(&probe_gcmd)
        .expect("start descend");
    descend.run_probe(&probe_gcmd).expect("run descend");
    w.sensor
        .deliver(make_msg(&[(0.1, 9_995_000.0, 999.9), (0.16, 9_995_000.0, 999.9)]));
    let descend_results = descend.pull_probed_results().expect("descend results");
    check(
        "descend results",
        py.eval("descend_results"),
        probe_results_value(&descend_results),
    );
    let calls: Vec<Value> = w
        .homing
        .calls
        .lock()
        .iter()
        .map(|(pos, speed, check)| {
            Value::Tuple(vec![
                Value::List(
                    pos.iter()
                        .map(|v| match v {
                            Some(f) => Value::float(*f),
                            None => Value::Null,
                        })
                        .collect(),
                ),
                Value::float(*speed),
                Value::Bool(*check),
            ])
        })
        .collect();
    let expected_calls =
        py.eval("[(list(pos), speed, check) for pos, speed, check in w['homing'].calls]");
    check("descend homing", expected_calls, Value::List(calls));
    descend.end_probe_session().expect("end descend");

    // -----------------------------------------------------------------
    // EddyScanningProbe: SAMPLE_TIME, scan run, rapid lookahead, event.
    // -----------------------------------------------------------------
    py.exec(&format!(
        "w = build({cal:?})\n\
         w['p'].register_event_handler('probe:update_results', lambda results: results.__setitem__(0, results[0]._replace(bed_z=1.25)))\n\
         cal = pec.EddyCalibration(w['c'])\n\
         offsets = pec.EddyProbeOffsets(w['c'])\n\
         scan = pec.EddyScanningProbe(w['c'], w['sensor'], cal, offsets)\n\
         scan.start_probe_session(w['g'].create_gcode_command('PROBE', '', {{'METHOD': 'scan', 'SAMPLE_TIME': '0.100'}}))\n\
         scan.run_probe(w['g'].create_gcode_command('PROBE', '', {{}}))\n\
         scan_samples = [(0.06, 9995000.0, 999.9), (0.10, 9995000.0, 999.9), (0.16, 9995000.0, 999.9)]\n\
         w['sensor'].deliver(msg(scan_samples))\n\
         scan_results = scan.pull_probed_results()"
    ))
    .expect("python scan");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    {
        let printer = w.printer.clone() as Printer;
        printer.register_event_handler(
            klippy_api::printer::EV_PROBE_UPDATE_RESULTS,
            Box::new(|args| {
                if let Some(list) = args.first().and_then(|arg| arg.as_handle::<extras::probe::ProbeResultList>()) {
                    let mut results = list.borrow_mut();
                    results[0].bed_z = 1.25;
                }
                Ok(PyValue::None)
            }),
        );
    }
    let calibration = EddyCalibration::new(&w.config).expect("cal");
    let offsets = EddyProbeOffsets::new(&w.config).expect("offsets");
    let scan = EddyScanningProbe::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        calibration as Arc<dyn EddyCalibrationApi>,
        &offsets,
    )
    .expect("scan");
    let mut scan_params = IndexMap::new();
    scan_params.insert("METHOD".to_string(), "scan".to_string());
    scan_params.insert("SAMPLE_TIME".to_string(), "0.100".to_string());
    let scan_gcmd = w.gcode.create_gcode_command("PROBE", "", &scan_params);
    scan.start_probe_session(&scan_gcmd).expect("scan start");
    let run_gcmd = w
        .gcode
        .create_gcode_command("PROBE", "", &IndexMap::new());
    scan.run_probe(&run_gcmd).expect("scan run");
    w.sensor.deliver(make_msg(&[
        (0.06, 9_995_000.0, 999.9),
        (0.10, 9_995_000.0, 999.9),
        (0.16, 9_995_000.0, 999.9),
    ]));
    let scan_results = scan.pull_probed_results().expect("scan results");
    check(
        "scan results",
        py.eval("scan_results"),
        probe_results_value(&scan_results),
    );
    scan.end_probe_session().expect("scan end");

    // SAMPLE_TIME must be above zero.
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         offsets = pec.EddyProbeOffsets(w['c'])\n\
         scan = pec.EddyScanningProbe(w['c'], w['sensor'], cal, offsets)\n\
         scan_bad = w['g'].create_gcode_command('PROBE', '', {{'METHOD': 'scan', 'SAMPLE_TIME': '0'}})"
    ))
    .expect("python scan bad");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("cal");
    let offsets = EddyProbeOffsets::new(&w.config).expect("offsets");
    let scan = EddyScanningProbe::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        calibration as Arc<dyn EddyCalibrationApi>,
        &offsets,
    )
    .expect("scan");
    let mut bad_params = IndexMap::new();
    bad_params.insert("METHOD".to_string(), "scan".to_string());
    bad_params.insert("SAMPLE_TIME".to_string(), "0".to_string());
    let bad_gcmd = w.gcode.create_gcode_command("PROBE", "", &bad_params);
    check_err(
        "scan sample time 0",
        py.eval("scan.start_probe_session(scan_bad)"),
        scan.start_probe_session(&bad_gcmd).map(|_| ()),
    );

    // rapid_scan registers a lookahead callback instead of probing at once.
    py.exec(&format!(
        "w = build({cal:?})\n\
         cal = pec.EddyCalibration(w['c'])\n\
         offsets = pec.EddyProbeOffsets(w['c'])\n\
         scan = pec.EddyScanningProbe(w['c'], w['sensor'], cal, offsets)\n\
         scan.start_probe_session(w['g'].create_gcode_command('PROBE', '', {{'METHOD': 'rapid_scan', 'SAMPLE_TIME': '0.050'}}))\n\
         scan.run_probe(w['g'].create_gcode_command('PROBE', '', {{}}))\n\
         scan_lookahead = len(w['th'].lookahead)"
    ))
    .expect("python rapid scan");
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("cal");
    let offsets = EddyProbeOffsets::new(&w.config).expect("offsets");
    let scan = EddyScanningProbe::new(
        &w.config,
        w.sensor.clone() as Arc<dyn LDC1612Api>,
        calibration as Arc<dyn EddyCalibrationApi>,
        &offsets,
    )
    .expect("scan");
    let mut rapid_params = IndexMap::new();
    rapid_params.insert("METHOD".to_string(), "rapid_scan".to_string());
    rapid_params.insert("SAMPLE_TIME".to_string(), "0.050".to_string());
    let rapid_gcmd = w.gcode.create_gcode_command("PROBE", "", &rapid_params);
    scan.start_probe_session(&rapid_gcmd).expect("rapid start");
    let run_gcmd = w
        .gcode
        .create_gcode_command("PROBE", "", &IndexMap::new());
    scan.run_probe(&run_gcmd).expect("rapid run");
    let lookahead = w.toolhead.lookahead.lock().len();
    check(
        "rapid lookahead",
        py.eval("scan_lookahead"),
        Value::int(lookahead as i64),
    );
}

#[test]
fn calibration_commands_parity() {
    let Some(mut py) = oracle() else {
        eprintln!("probe_eddy_current commands: KLIPPY_PYTHON is not set, no oracle available");
        return;
    };
    let cal = cal_string();
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let manual = extras::manual_probe::ManualProbe::new(&w.config).expect("manual probe");
    w.printer.add("manual_probe", manual as ObjectRef);
    let tool = EddyCalibrationTool::new(
        &w.config,
        EddyCalibration::new(&w.config).expect("calibration"),
    ).expect("calibration tool");
    py.exec(&format!(r#"
w = build({cal:?})
w['p'].add_object('manual_probe', manual_probe.ManualProbe(w['c']))
tool = pec.EddyCalibrationTool(w['c'], pec.EddyCalibration(w['c']))
w['p'].send_event('klippy:ready')
"#)).expect("python manual command setup");
    w.printer.send_event("klippy:ready", &[]).expect("ready");
    let cmd = w.gcode.create_gcode_command(
        "PROBE_EDDY_CURRENT_CALIBRATE", "", &IndexMap::new(),
    );
    EddyCalibrationTool::cmd_eddy_calibrate(&tool, &cmd).expect("start manual calibration");
    py.exec("tool.cmd_EDDY_CALIBRATE(gcmd(w, {}, 'PROBE_EDDY_CURRENT_CALIBRATE'))")
        .expect("python manual calibration");
    check("manual calibration starts", py.eval("LOG"), log_value(&w.trace));
    w.gcode.run_script_from_command("ABORT").expect("abort manual calibration");
    py.exec("w['g'].run_script_from_command('ABORT')").expect("python abort calibration");
    check("manual calibration abort", py.eval("LOG"), log_value(&w.trace));
    check("abort config writes", py.eval("w['cf'].writes"), writes_value(&w.configfile.writes.lock()));
    check("post manual abort", py.eval("tool.post_manual_probe(None)"), {
        tool.post_manual_probe(None).expect("post manual abort");
        Value::Null
    });

    let client = Arc::clone(&w.probe_client);
    let position = Arc::clone(&w.toolhead.position);
    let move_count = GCell::new(0);
    w.toolhead.set_move_hook(Arc::new(move |time| {
        move_count.set(move_count.get() + 1);
        if move_count.get() <= 3 || move_count.get() % 2 == 0 {
            return;
        }
        let freq = 10_000_000.0 - position.borrow()[2] * 1000.0;
        client.deliver(make_msg(&[(time + 0.1, freq, 999.9)]));
    }));
    py.exec(r#"
calibration_moves = 0
def calibration_batch(t, n):
    global calibration_moves
    calibration_moves += 1
    if calibration_moves <= 3 or calibration_moves % 2 == 0:
        return []
    return [(t, 10000000.0 - w['th'].position[2] * 1000.0, 999.9)]
w['th'].sensor.auto = calibration_batch
"#).expect("python calibration sensor");
    EddyCalibrationTool::cmd_eddy_calibrate(&tool, &cmd).expect("restart calibration");
    py.exec("tool.cmd_EDDY_CALIBRATE(gcmd(w, {}, 'PROBE_EDDY_CURRENT_CALIBRATE'))")
        .expect("python restart calibration");
    klippy_api::toolhead::HomingTargetApi::set_position(
        &*w.toolhead, vec![10.0, 10.0, 1.5, 0.0], "",
    ).expect("lower nozzle");
    py.exec("w['th'].position[2] = 1.5").expect("python lower nozzle");
    w.gcode.run_script_from_command("ACCEPT").expect("accept manual calibration");
    py.exec("w['g'].run_script_from_command('ACCEPT')").expect("python accept calibration");
    check("manual calibration accepted", py.eval("LOG"), log_value(&w.trace));
    check("manual calibration config", py.eval("w['cf'].writes"), writes_value(&w.configfile.writes.lock()));
}

#[test]
fn tap_calibration_commands_parity() {
    let Some(mut py) = oracle() else {
        eprintln!("probe_eddy_current tap commands: KLIPPY_PYTHON is not set, no oracle available");
        return;
    };
    let cal = cal_string();
    let w = rust_world(Some(&cal), &[], 0.0, vec![10.0, 10.0, 2.0, 0.0], &[]);
    let calibration = EddyCalibration::new(&w.config).expect("calibration");
    let tap = EddyTap::new(
        &w.config, w.sensor.clone(), EddyParameterHelper::new(&w.config).expect("params"),
        trigger_analog(&w),
    ).expect("tap");
    let tool = extras::probe_eddy_current::EddyTapCalibration::new(
        &w.config, calibration.clone(), tap.clone(),
    ).expect("tap calibration");
    let result = extras::manual_probe::create_probe_result(
        &[1.0, 2.0, 3.0], [0.0, 0.0, 0.0],
    );
    let host = Arc::new(FakeProbeHost {
        sessions: Arc::new(Mutex::new(Vec::new())),
        offsets: (3.0, 4.0, 5.0),
        results: vec![result],
    });
    w.printer.add("probe", host.clone() as ObjectRef);
    py.exec(&format!(r#"
w = build({cal:?})
cal = pec.EddyCalibration(w['c'])
tap = pec.EddyTap(w['c'], w['sensor'], pec.EddyParameterHelper(w['c']), FakeMcuTrigger(w['sensor']))
tool = pec.EddyTapCalibration(w['c'], cal, tap)
w['host'].results = [manual_probe.create_probe_result((1.0, 2.0, 3.0))]
"#)).expect("python tap calibration");
    let command = |params: &[(&str, &str)]| {
        let params = params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
        w.gcode.create_gcode_command("PROBE_EDDY_CURRENT_TAP_CALIBRATE", "", &params)
    };
    let cmd = command(&[]);
    tool.cmd_tap_calibrate(&cmd).expect("tap description");
    py.exec("tool.cmd_TAP_CALIBRATE(gcmd(w, {}, 'PROBE_EDDY_CURRENT_TAP_CALIBRATE'))")
        .expect("python tap description");
    check("tap calibration description", py.eval("LOG"), log_value(&w.trace));
    for value in ["invalid", "refine", "verify"] {
        check_err(
            &format!("tap calibration {value}"),
            py.eval(&format!("tool.cmd_TAP_CALIBRATE(gcmd(w, {{'TAP': {value:?}}}, 'PROBE_EDDY_CURRENT_TAP_CALIBRATE'))")),
            tool.cmd_tap_calibrate(&command(&[("TAP", value)])),
        );
    }
    tool.cmd_tap_calibrate(&command(&[("TAP", "guess")])).expect("tap guess");
    py.exec("tool.cmd_TAP_CALIBRATE(gcmd(w, {'TAP': 'guess'}, 'PROBE_EDDY_CURRENT_TAP_CALIBRATE'))")
        .expect("python tap guess");
    check("tap guess output", py.eval("LOG"), log_value(&w.trace));
    check("tap guess params", py.eval("w['host'].seen_params[0]"), {
        let sessions = host.sessions.lock();
        Value::Dict(sessions[0].1[0].iter().map(|(k, v)| (Value::str(k), Value::str(v))).collect())
    });
    check("tap guess session", py.eval("(w['host'].sessions[0].ran, w['host'].sessions[0].ended)"), {
        let sessions = host.sessions.lock();
        let ran = *sessions[0].0.ran.lock();
        let ended = *sessions[0].0.ended.lock();
        Value::Tuple(vec![Value::Bool(ran), Value::Bool(ended)])
    });
    tool.try_tap(&cmd, 1234.5, 5).expect("tap repeated");
    py.exec("tool._try_tap(gcmd(w, {}), 1234.5, 5)").expect("python tap repeated");
    tool.save_tap_threshold(&cmd, 1234.5).expect("save tap threshold");
    py.exec("tool._save_tap_threshold(gcmd(w, {}), 1234.5)").expect("python save tap threshold");
    check("tap saved writes", py.eval("w['cf'].writes"), writes_value(&w.configfile.writes.lock()));
    check("tap saved output", py.eval("LOG"), log_value(&w.trace));
    for status in ["success", "fail"] {
        let last = Some((status.to_string(), 0.125, [0.5, 1000.0, -200.0, -500.0, 20.0]));
        check(
            "tap last description",
            py.eval(&format!("tool._describe_last_tap(({status:?}, 0.125, (0.5, 1000.0, -200.0, -500.0, 20.0)))")),
            strings(&tool.describe_last_tap(&last).expect("last description")),
        );
    }
    let toolhead = FakeToolhead::new(vec![0.0, 0.0, 0.0, 0.0], Arc::new(|t| t * 3.0));
    w.printer.add("toolhead", toolhead as ObjectRef);
    let measures: Vec<_> = (0..64).map(|i| {
        let t = 0.001 + i as f64 * 0.0025;
        let z = t * 3.0;
        let dz = z - 0.125;
        let freq = if z <= 0.125 {
            10_000_000.0 + -30_000.0 * dz
        } else {
            10_000_000.0 + -60_000.0 * dz + 20_000.0 * dz * dz
        };
        (t, freq, 0.0)
    }).collect();
    let measures_py = measures.iter()
        .map(|(t, f, z)| format!("({t:?}, {f:?}, {z:?})"))
        .collect::<Vec<_>>().join(", ");
    py.exec(&format!("w['th'].past_fn = lambda t: t * 3.0\npullback = [{measures_py}]"))
        .expect("python valid pullback");
    let result = tap.analyze_pullback(measures, 0.0, 0.16).expect("valid pullback");
    check(
        "tap refine pullback",
        py.eval("tuple(tap._analyze_pullback(pullback, 0.0, 0.16))"),
        probe_result_value(&result),
    );
    for value in ["refine", "verify"] {
        tool.cmd_tap_calibrate(&command(&[("TAP", value)])).expect("tap refinement");
        py.exec(&format!("tool.cmd_TAP_CALIBRATE(gcmd(w, {{'TAP': {value:?}}}, 'PROBE_EDDY_CURRENT_TAP_CALIBRATE'))"))
            .expect("python tap refinement");
        check("tap refinement output", py.eval("LOG"), log_value(&w.trace));
        check("tap refinement config", py.eval("w['cf'].writes"), writes_value(&w.configfile.writes.lock()));
    }
    calibration.load_calibration(&[]);
    py.exec("cal._load_calibration([])").expect("python clear calibration");
    check_err(
        "tap guess needs calibration",
        py.eval("tool.cmd_TAP_CALIBRATE(gcmd(w, {'TAP': 'guess'}, 'PROBE_EDDY_CURRENT_TAP_CALIBRATE'))"),
        tool.cmd_tap_calibrate(&command(&[("TAP", "guess")])),
    );
    check(
        "tap no main calibration",
        py.eval("tool._describe_main_calibration(None)"),
        strings(&tool.describe_main_calibration(&None).expect("no main calibration")),
    );
    check_err("tap session not started", py.eval("tap.end_probe_session()"), tap.end_probe_session());
}

