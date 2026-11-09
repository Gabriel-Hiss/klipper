//! Parity tests for `extras::bed_tilt` (port of `klippy/extras/bed_tilt.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `BedTilt` is built on both
//! sides with minimal Python/Rust fakes for the printer, config, toolhead,
//! gcode, gcode_move and configfile, then fed the same adjustments,
//! toolhead positions, moves, updates and probed positions. Returned values
//! (floats by bits), raised errors (class and message), gcode registration,
//! respond/rollover text, config writes, log lines and the config options
//! read are compared through `recording::assert_equal`.
//!
//! Both sides drive the real `ProbePointsHelper` (`extras::probe`): the
//! Python side uses the frozen `klippy/extras/probe.py` helper and the Rust
//! side uses `crate::probe::ProbePointsHelper`, so construction reads
//! (`points`, `horizontal_move_z`, `speed`), `minimum_points(3)` failures
//!
//! Test fakes are plain synchronised bags of state, not printer objects.
#![allow(clippy::disallowed_types)]

use extras::bed_tilt::{probe_positions_repr, BedTilt, CMD_BED_TILT_CALIBRATE_HELP};
use extras::manual_probe::ProbeResult;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, MoveTransform, MoveTransformApi, RegisterCommandOpts, GCmd,
};
use klippy_api::log;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    Rail, RailOpts, Stepper, StepperFactory, StepperFactoryApi, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    ExtraAxis, ExtraAxisApi, HomingTargetApi, KinematicsApi, LookaheadCallback, MoveApi,
    PrinterExtruder, PrinterExtruderApi, ToolHeadApi,
};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use pycompat::text::float_repr;
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::any::Any;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging
import collections

import extras.bed_tilt as bed_tilt

log_messages = []
class ListHandler(logging.Handler):
    def emit(self, record):
        log_messages.append(record.getMessage())
logging.getLogger().addHandler(ListHandler())
logging.getLogger().setLevel(logging.INFO)

ProbeResult = collections.namedtuple('probe_result',
    ['bed_x', 'bed_y', 'bed_z', 'test_x', 'test_y', 'test_z'])

class CommandError(Exception):
    pass

class FakeGCodeMove:
    def __init__(self):
        self.transform = None
        self.reset_calls = 0
    def set_move_transform(self, transform, force=False):
        old = self.transform
        self.transform = transform
        return old
    def reset_last_position(self):
        self.reset_calls += 1

class FakeKinematics:
    def __init__(self, toolhead):
        self.toolhead = toolhead
    def get_steppers(self):
        return []
    def calc_position(self, stepper_positions):
        return list(self.toolhead.get_position())

class FakeToolhead:
    def __init__(self, position):
        self._position = list(position)
        self.moves = []
        self.manual_moves = []
        self._kinematics = FakeKinematics(self)
    def get_position(self):
        return list(self._position)
    def move(self, newpos, speed):
        self.moves.append((list(newpos), speed))
        self._position = list(newpos)
    def manual_move(self, coord, speed):
        self.manual_moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self._position[i] = v
    def get_kinematics(self):
        return self._kinematics
    def flush_step_generation(self):
        pass
    def get_last_move_time(self):
        return 0.0

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.registered = {}
        self.responded = []
    def register_command(self, cmd, func=None, desc=None, when_not_ready=False):
        if func is None:
            # Like `gcode.py:133`: unregistering a missing command raises.
            if cmd not in self.registered:
                raise configparser.Error("Unknown command '%s'" % (cmd,))
            del self.registered[cmd]
            return None
        self.registered[cmd] = (func, desc)
        return func
    def create_gcode_command(self, command, commandline, params):
        return FakeGcmd(command, params)
    def respond_info(self, msg, log=True):
        if log:
            logging.info(msg)
        self.responded.append(msg)
    def respond_raw(self, msg):
        pass

class FakeGcmd:
    error = CommandError
    def __init__(self, command, params=None):
        self._command = command
        self._params = params or {}
    def get(self, name, default=None, parser=str, minval=None, maxval=None,
            above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is None:
                raise CommandError(
                    "Error on '%s': missing %s" % (self._command, name))
            return default
        try:
            value = parser(value)
        except Exception:
            raise CommandError(
                "Error on '%s': unable to parse %s" % (self._command, value))
        if minval is not None and value < minval:
            raise CommandError(
                "Error on '%s': %s must have minimum of %s"
                % (self._command, name, minval))
        if maxval is not None and value > maxval:
            raise CommandError(
                "Error on '%s': %s must have maximum of %s"
                % (self._command, name, maxval))
        if above is not None and value <= above:
            raise CommandError(
                "Error on '%s': %s must be above %s"
                % (self._command, name, above))
        if below is not None and value >= below:
            raise CommandError(
                "Error on '%s': %s must be below %s"
                % (self._command, name, below))
        return value
    def get_int(self, name, default=None, minval=None, maxval=None):
        return self.get(name, default, parser=int,
                        minval=minval, maxval=maxval)
    def get_float(self, name, default=None, minval=None, maxval=None,
                  above=None, below=None):
        return self.get(name, default, parser=float, minval=minval,
                        maxval=maxval, above=above, below=below)
    def get_command(self):
        return self._command
    def get_command_parameters(self):
        return self._params
    def get_commandline(self):
        return self._command
    def get_raw_command_parameters(self):
        return ''
    def respond_info(self, msg, log=True):
        pass
    def respond_raw(self, msg):
        pass
    def ack(self, msg=None):
        return True

class FakeConfigFile:
    def __init__(self):
        self.settings = {}
    def set(self, section, option, value):
        self.settings[(section, option)] = value

class FakeManualProbe:
    def __init__(self):
        self.status = {'is_active': False, 'z_position': None,
                       'z_position_lower': None, 'z_position_upper': None}
    def reset_status(self):
        self.status = {'is_active': False, 'z_position': None,
                       'z_position_lower': None, 'z_position_upper': None}
_sentinel = object()

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.rollover = []
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if default is _sentinel:
            raise configparser.Error("Unable to load module '%s'" % (section,))
        return default
    def set_rollover_info(self, name, info, log=True):
        if log:
            logging.info(info)
        self.rollover.append((name, info, log))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options):
        self._printer = printer
        self._options = options
        self.reads = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'bed_tilt'
    def get(self, option, default=_sentinel, note_valid=True):
        self.reads.append(('get', option))
        if option in self._options:
            return self._options[option]
        if default is not _sentinel:
            return default
        raise configparser.Error(
            "Option '%s' in section '%s' must be specified"
            % (option, self.get_name()))
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        self.reads.append(('getfloat', option))
        if option not in self._options:
            if default is not _sentinel:
                return default
            raise configparser.Error(
                "Option '%s' in section '%s' must be specified"
                % (option, self.get_name()))
        try:
            value = float(self._options[option])
        except ValueError:
            raise configparser.Error(
                "Unable to parse option '%s' in section '%s'"
                % (option, self.get_name()))
        if minval is not None and value < minval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have minimum of %s"
                % (option, self.get_name(), minval))
        if maxval is not None and value > maxval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self.get_name(), maxval))
        if above is not None and value <= above:
            raise configparser.Error(
                "Option '%s' in section '%s' must be above %s"
                % (option, self.get_name(), above))
        if below is not None and value >= below:
            raise configparser.Error(
                "Option '%s' in section '%s' must be below %s"
                % (option, self.get_name(), below))
        return value
    def getlists(self, option, default=_sentinel, seps=(',',), count=None,
                 parser=str, note_valid=True):
        self.reads.append(('getlists', option))
        if option not in self._options:
            if default is not _sentinel:
                return default
            raise configparser.Error(
                "Option '%s' in section '%s' must be specified"
                % (option, self.get_name()))
        def lparser(value, pos):
            if len(value.strip()) == 0:
                parts = []
            else:
                parts = [p.strip() for p in value.split(seps[pos])]
            if pos:
                return tuple([lparser(p, pos - 1) for p in parts if p])
            res = [parser(p) for p in parts]
            if count is not None and len(res) != count:
                raise configparser.Error(
                    "Option '%s' in section '%s' must have %d elements"
                    % (option, self.get_name(), count))
            return tuple(res)
        return lparser(self._options[option], len(seps) - 1)

p = None
c = None
bt = None
gcode_fake = None
gcode_move_fake = None
toolhead_fake = None
configfile_fake = None

def make_shell(options, toolhead_pos):
    global p, c, gcode_fake, gcode_move_fake, toolhead_fake, configfile_fake
    log_messages.clear()
    gcode_fake = FakeGCode()
    gcode_move_fake = FakeGCodeMove()
    toolhead_fake = FakeToolhead(toolhead_pos)
    configfile_fake = FakeConfigFile()
    p = FakePrinter()
    p.objects = {'gcode': gcode_fake, 'gcode_move': gcode_move_fake,
                 'toolhead': toolhead_fake, 'configfile': configfile_fake,
                 'manual_probe': FakeManualProbe()}
    c = FakeConfig(p, options)
def build_bedtilt():
    global bt
    bt = bed_tilt.load_config(c)
"#;

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG_LINES: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        let _ = log::set_sink(Box::new(|rec| {
            LOG_LINES.with(|lines| lines.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

/// Calls `f`, clearing captured lines first, and returns the captured lines.
fn with_log_capture<R>(f: impl FnOnce() -> R) -> (R, Vec<String>) {
    install_log_sink();
    LOG_LINES.with(|lines| lines.borrow_mut().clear());
    let result = f();
    let lines = LOG_LINES.with(|lines| lines.borrow().clone());
    (result, lines)
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default)]
struct SetupConfig {
    x_adjust: Option<&'static str>,
    y_adjust: Option<&'static str>,
    z_adjust: Option<&'static str>,
    points: Option<&'static str>,
}

// --- toolhead fake ---

struct FakeTrapq;
impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct FakeStepperFactory;
impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(
        &self,
        _config: &Config,
        _units_in_radians: bool,
    ) -> KResult<Stepper> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "printer_stepper is not used by bed_tilt".to_string(),
        })
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "lookup_rail is not used by bed_tilt".to_string(),
        })
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "lookup_multi_rail is not used by bed_tilt".to_string(),
        })
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "generic_rail is not used by bed_tilt".to_string(),
        })
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "parse_step_distance is not used by bed_tilt".to_string(),
        })
    }
}

struct FakeKinematics {
    position: Arc<GCell<Vec<f64>>>,
}
impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(self.position.borrow().iter().map(|v| Some(*v)).collect())
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
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn clear_homing_state(&self, _clear_axes: &str) -> klippy_api::KResult<()> { Ok(()) }
}

struct FakeExtruder;
impl ExtraAxisApi for FakeExtruder {
    fn get_name(&self) -> String {
        String::new()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        None
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn check_move(&self, _mv: &dyn MoveApi, _ea_index: usize) -> KResult<()> {
        Ok(())
    }
    fn calc_junction(
        &self,
        _prev_move: &dyn MoveApi,
        _mv: &dyn MoveApi,
        _ea_index: usize,
    ) -> klippy_api::KResult<f64> {
        Ok(0.0)
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}
impl PrinterExtruderApi for FakeExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::command("Extruder not configured"))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

struct FakeToolhead {
    position: Arc<GCell<Vec<f64>>>,
    moves: GCell<Vec<(Vec<f64>, f64)>>,
    manual_moves: GCell<Vec<(Vec<Option<f64>>, f64)>>,
    kinematics: klippy_api::toolhead::Kinematics,
}

impl FakeToolhead {
    fn new(position: Vec<f64>) -> Arc<Self> {
        let shared = Arc::new(GCell::new(position));
        let kinematics = Arc::new(FakeKinematics {
            position: Arc::clone(&shared),
        }) as klippy_api::toolhead::Kinematics;
        Arc::new(FakeToolhead {
            position: shared,
            moves: GCell::new(Vec::new()),
            manual_moves: GCell::new(Vec::new()),
            kinematics,
        })
    }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.borrow().clone()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        *self.position.borrow_mut() = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        self.kinematics.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
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
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.moves.borrow_mut().push((newpos.clone(), speed));
        *self.position.borrow_mut() = newpos;
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        self.manual_moves.borrow_mut().push((coord.clone(), speed));
        let mut pos = self.position.borrow().clone();
        for (slot, axis) in pos.iter_mut().zip(coord.iter()) {
            if let Some(v) = axis {
                *slot = *v;
            }
        }
        *self.position.borrow_mut() = pos;
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
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
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        Arc::new(FakeExtruder)
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        Arc::new(FakeStepperFactory)
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

// --- gcode fake ---

struct FakeGCode {
    reactor: Reactor,
    registered: GCell<Vec<(String, Option<String>)>>,
    handlers: GCell<Vec<(String, GCodeCommandHandler)>>,
    responded: GCell<Vec<String>>,
}

impl FakeGCode {
    fn take_handler(&self, cmd: &str) -> Option<GCodeCommandHandler> {
        let mut handlers = self.handlers.borrow_mut();
        handlers
            .iter()
            .position(|(name, _)| name == cmd)
            .map(|i| handlers.remove(i).1)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        match func {
            Some(handler) => {
                self.registered
                    .borrow_mut()
                    .push((cmd.to_string(), opts.desc));
                self.handlers
                    .borrow_mut()
                    .push((cmd.to_string(), handler));
                Ok(None)
            }
            None => {
                let mut handlers = self.handlers.borrow_mut();
                match handlers.iter().position(|(name, _)| name == cmd) {
                    Some(i) => {
                        let (_, old) = handlers.remove(i);
                        let mut registered = self.registered.borrow_mut();
                        if let Some(j) = registered.iter().position(|(name, _)| name == cmd) {
                            registered.remove(j);
                        }
                        Ok(Some(old))
                    }
                    None => Err(KlippyError::config(format!(
                        "Unknown command '{cmd}'"
                    ))),
                }
            }
        }
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
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: GCell::new(params.clone()),
        })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, log: bool) {
        if log {
            klippy_api::log::info(msg);
        }
        self.responded.borrow_mut().push(msg.to_string());
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

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGcmd {
    command: String,
    commandline: String,
    params: GCell<IndexMap<String, String>>,
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let raw = match self.params.borrow().get(name) {
            Some(v) => v.clone(),
            None => match opts.default {
                None => {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': missing {name}",
                        self.commandline
                    )))
                }
                Some(d) => return Ok(d),
            },
        };
        let commandline = self.commandline.clone();
        let unable = || {
            KlippyError::command(format!(
                "Error on '{commandline}': unable to parse {raw}"
            ))
        };
        let parsed = match opts.parser {
            klippy_api::gcode::GetParser::Str => PyValue::Str(raw.clone()),
            klippy_api::gcode::GetParser::Int => raw
                .parse::<i64>()
                .map(|n| PyValue::Int(pycompat::PyInt::from_i64(n)))
                .map_err(|_| unable())?,
            klippy_api::gcode::GetParser::IntAutoBase => raw
                .parse::<i64>()
                .map(|n| PyValue::Int(pycompat::PyInt::from_i64(n)))
                .map_err(|_| unable())?,
            klippy_api::gcode::GetParser::Float => raw
                .parse::<f64>()
                .map(PyValue::Float)
                .map_err(|_| unable())?,
        };
        let as_float = |v: &PyValue| match v {
            PyValue::Int(n) => n.0.to_string().parse::<f64>().unwrap_or(0.0),
            PyValue::Float(f) => *f,
            PyValue::Bool(b) => {
                if *b {
                    1.0
                } else {
                    0.0
                }
            }
            _ => 0.0,
        };
        let bound_str = |b: &PyValue| match b {
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Float(f) => pycompat::text::float_repr(*f),
            PyValue::Bool(b) => b.to_string(),
            _ => format!("{b:?}"),
        };
        let v = as_float(&parsed);
        let check =
            |bound: Option<&PyValue>, op: fn(f64, f64) -> bool, word: &str| {
                if let Some(b) = bound {
                    if op(v, as_float(b)) {
                        return Err(KlippyError::command(format!(
                            "Error on '{commandline}': {name} must {word} {}",
                            bound_str(b)
                        )));
                    }
                }
                Ok(())
            };
        check(opts.minval.as_ref(), |a, b| a < b, "have minimum of")?;
        check(opts.maxval.as_ref(), |a, b| a > b, "have maximum of")?;
        check(opts.above.as_ref(), |a, b| a <= b, "be above")?;
        check(opts.below.as_ref(), |a, b| a >= b, "be below")?;
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.borrow().get(name).cloned() {
            Some(raw) => raw
                .parse::<i64>()
                .map(Some)
                .map_err(|_| {
                    KlippyError::command(format!(
                        "Error on '{}': unable to parse {raw}",
                        self.commandline
                    ))
                })
                .and_then(|v| {
                    if let Some(min) = opts.minval {
                        if v.unwrap_or(0) < min {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {name} must have minimum of {min}",
                                self.commandline
                            )));
                        }
                    }
                    if let Some(max) = opts.maxval {
                        if v.unwrap_or(0) > max {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {name} must have maximum of {max}",
                                self.commandline
                            )));
                        }
                    }
                    Ok(v)
                }),
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.borrow().get(name).cloned() {
            Some(raw) => raw
                .parse::<f64>()
                .map(Some)
                .map_err(|_| {
                    KlippyError::command(format!(
                        "Error on '{}': unable to parse {raw}",
                        self.commandline
                    ))
                })
                .and_then(|v| {
                    if let Some(min) = opts.minval {
                        if v.unwrap_or(0.0) < min {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {name} must have minimum of {min}",
                                self.commandline
                            )));
                        }
                    }
                    if let Some(max) = opts.maxval {
                        if v.unwrap_or(0.0) > max {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {name} must have maximum of {max}",
                                self.commandline
                            )));
                        }
                    }
                    if let Some(above) = opts.above {
                        if v.unwrap_or(0.0) <= above {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {name} must be above {above}",
                                self.commandline
                            )));
                        }
                    }
                    if let Some(below) = opts.below {
                        if v.unwrap_or(0.0) >= below {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {name} must be below {below}",
                                self.commandline
                            )));
                        }
                    }
                    Ok(v)
                }),
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.borrow().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

// --- gcode_move fake ---

struct FakeGCodeMove {
    transform: GCell<Option<MoveTransform>>,
    reset_calls: GCell<usize>,
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set_move_transform(
        &self,
        transform: Option<MoveTransform>,
        _force: bool,
    ) -> KResult<Option<MoveTransform>> {
        Ok(self.transform.replace(transform))
    }
    fn reset_last_position(&self) -> KResult<()> {
        *self.reset_calls.borrow_mut() += 1;
        Ok(())
    }
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

// --- configfile fake ---

struct FakeConfigFile {
    printer: Printer,
    settings: GCell<Vec<(String, String, String)>>,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "read_config is not used by bed_tilt".to_string(),
        })
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "read_main_config is not used by bed_tilt".to_string(),
        })
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
    fn deprecate_mcu_code(
        &self,
        _mcu: &klippy_api::mcu::Mcu,
        _feature: &str,
        _msg: Option<String>,
    ) {
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.settings.borrow_mut().push((
            section.to_string(),
            option.to_string(),
            value.to_string(),
        ));
        Ok(())
    }
    fn remove_section(&self, _section: &str) {}
}
klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

// --- manual_probe fake: `ManualProbeHelper::new` looks it up and calls
// `SPEED` on the gcmd, `get_position`/`get_kinematics` on the toolhead, then
// registers ACCEPT/NEXT/ABORT/TESTZ on the gcode fake.

struct FakeManualProbe;

impl extras::manual_probe::ManualProbeApi for FakeManualProbe {
    fn reset_status(&self) {}
    fn set_status(
        &self,
        _is_active: bool,
        _z_position: Option<f64>,
        _z_position_lower: Option<f64>,
        _z_position_upper: Option<f64>,
    ) {
    }
}

klippy_api::printer_object!(FakeManualProbe; dyn extras::manual_probe::ManualProbeApi);

// --- printer fake ---

struct FakePrinter {
    toolhead: ObjectRef,
    gcode: ObjectRef,
    gcode_move: ObjectRef,
    configfile: ObjectRef,
    manual_probe: ObjectRef,
    events: GCell<Vec<String>>,
    rollover: GCell<Vec<(String, Option<String>, bool)>>,
    reactor: Reactor,
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let found = match name {
            "toolhead" => Some(self.toolhead.clone()),
            "gcode" => Some(self.gcode.clone()),
            "gcode_move" => Some(self.gcode_move.clone()),
            "configfile" => Some(self.configfile.clone()),
            "manual_probe" => Some(self.manual_probe.clone()),
            _ => None,
        };
        match found {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
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
        if section == "gcode_move" {
            return Ok(Some(self.gcode_move.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unable to load module '{section}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool) {
        if log {
            if let Some(text) = info {
                klippy_api::log::info(text);
            }
        }
        self.rollover
            .borrow_mut()
            .push((name.to_string(), info.map(str::to_string), log));
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.events.borrow_mut().push(event.to_string());
    }
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

// --- config fake ---

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
    reads: GCell<Vec<(String, String)>>,
}

fn required_error(option: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section 'bed_tilt' must be specified"
    ))
}

fn parse_float_error(option: &str) -> KlippyError {
    KlippyError::config(format!(
        "Unable to parse option '{option}' in section 'bed_tilt'"
    ))
}

fn min_error(option: &str, bound: f64) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section 'bed_tilt' must have minimum of {bound}"
    ))
}

fn max_error(option: &str, bound: f64) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section 'bed_tilt' must have maximum of {bound}"
    ))
}

fn above_error(option: &str, bound: f64) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section 'bed_tilt' must be above {bound}"
    ))
}

fn below_error(option: &str, bound: f64) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section 'bed_tilt' must be below {bound}"
    ))
}

fn count_error(option: &str, count: usize) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section 'bed_tilt' must have {count} elements"
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "bed_tilt".to_string()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.reads.borrow_mut().push(("get".into(), option.into()));
        match self.options.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(required_error(option)),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads.borrow_mut().push(("getfloat".into(), option.into()));
        let value = match self.options.get(option) {
            Some(v) => match v.parse::<f64>() {
                Ok(f) => f,
                Err(_) => return Err(parse_float_error(option)),
            },
            None => match opts.default {
                None => return Err(required_error(option)),
                Some(inner) => return Ok(inner),
            },
        };
        if let Some(min) = opts.minval {
            if value < min {
                return Err(min_error(option, min));
            }
        }
        if let Some(max) = opts.maxval {
            if value > max {
                return Err(max_error(option, max));
            }
        }
        if let Some(above) = opts.above {
            if value <= above {
                return Err(above_error(option, above));
            }
        }
        if let Some(below) = opts.below {
            if value >= below {
                return Err(below_error(option, below));
            }
        }
        Ok(Some(value))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        self.reads
            .borrow_mut()
            .push(("getlists".into(), option.into()));
        let raw = match self.options.get(option) {
            Some(v) => v.clone(),
            None => match opts.default {
                None => return Err(required_error(option)),
                Some(inner) => return Ok(inner.unwrap_or(PyValue::None)),
            },
        };
        fn lparser(
            value: &str,
            seps: &[String],
            pos: usize,
            count: Option<usize>,
            option: &str,
            parse_float: bool,
        ) -> KResult<PyValue> {
            let parts: Vec<String> = if value.trim().is_empty() {
                Vec::new()
            } else {
                value.split(&seps[pos]).map(|p| p.trim().to_string()).collect()
            };
            if pos > 0 {
                let mut out = Vec::new();
                for p in &parts {
                    if p.is_empty() {
                        continue;
                    }
                    out.push(lparser(p, seps, pos - 1, count, option, parse_float)?);
                }
                return Ok(PyValue::Tuple(out));
            }
            let mut res = Vec::new();
            for p in &parts {
                if parse_float {
                    match p.parse::<f64>() {
                        Ok(f) => res.push(PyValue::Float(f)),
                        Err(_) => return Err(parse_float_error(option)),
                    }
                } else {
                    res.push(PyValue::Str(p.clone()));
                }
            }
            if let Some(count) = count {
                if res.len() != count {
                    return Err(count_error(option, count));
                }
            }
            Ok(PyValue::Tuple(res))
        }
        let seps = if opts.seps.is_empty() {
            vec![",".to_string()]
        } else {
            opts.seps.clone()
        };
        let parse_float =
            opts.parser == klippy_api::config::ListParser::Float;
        lparser(&raw, &seps, seps.len() - 1, opts.count, option, parse_float)
    }
    fn getlist(
        &self,
        _option: &str,
        _opts: GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by bed_tilt")
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

// ---------------------------------------------------------------------------
// Pair construction (both sides use the real `ProbePointsHelper`).
// ---------------------------------------------------------------------------

struct RustPair {
    bedtilt: Arc<BedTilt>,
    toolhead: Arc<FakeToolhead>,
    gcode: Arc<FakeGCode>,
    gcode_move: Arc<FakeGCodeMove>,
    configfile: Arc<FakeConfigFile>,
    printer: Arc<FakePrinter>,
    config_fake: Arc<FakeConfig>,
}

struct RustShell {
    printer: Arc<FakePrinter>,
    config: Config,
    toolhead: Arc<FakeToolhead>,
    gcode: Arc<FakeGCode>,
    gcode_move: Arc<FakeGCodeMove>,
    configfile: Arc<FakeConfigFile>,
    config_fake: Arc<FakeConfig>,
}

fn make_shell(cfg: SetupConfig, toolhead_pos: Vec<f64>) -> RustShell {
    let reactor = Reactor::new(false);
    let toolhead = FakeToolhead::new(toolhead_pos);
    let gcode = Arc::new(FakeGCode {
        reactor: reactor.clone(),
        registered: GCell::new(Vec::new()),
        handlers: GCell::new(Vec::new()),
        responded: GCell::new(Vec::new()),
    });
    let gcode_move = Arc::new(FakeGCodeMove {
        transform: GCell::new(None),
        reset_calls: GCell::new(0),
    });
    let toolhead_obj: ObjectRef = Arc::clone(&toolhead) as ObjectRef;
    let gcode_obj: ObjectRef = Arc::clone(&gcode) as ObjectRef;
    let gcode_move_obj: ObjectRef = Arc::clone(&gcode_move) as ObjectRef;
    // The configfile fake only needs a printer handle for trait
    // compliance; `bed_tilt` never calls it.
    let printer_tmp: Printer = Arc::new(FakePrinterEmpty);
    let configfile = Arc::new(FakeConfigFile {
        printer: printer_tmp,
        settings: GCell::new(Vec::new()),
    });
    let configfile_obj: ObjectRef = Arc::clone(&configfile) as ObjectRef;
    let manual_probe: ObjectRef = Arc::new(FakeManualProbe) as ObjectRef;
    let printer_arc = Arc::new(FakePrinter {
        toolhead: toolhead_obj,
        gcode: gcode_obj,
        gcode_move: gcode_move_obj,
        configfile: configfile_obj,
        manual_probe,
        events: GCell::new(Vec::new()),
        rollover: GCell::new(Vec::new()),
        reactor: reactor.clone(),
    });
    let printer: Printer = printer_arc.clone() as Printer;
    let mut options = IndexMap::new();
    if let Some(v) = cfg.x_adjust {
        options.insert("x_adjust".into(), v.into());
    }
    if let Some(v) = cfg.y_adjust {
        options.insert("y_adjust".into(), v.into());
    }
    if let Some(v) = cfg.z_adjust {
        options.insert("z_adjust".into(), v.into());
    }
    if let Some(v) = cfg.points {
        options.insert("points".into(), v.into());
    }
    let config_fake = Arc::new(FakeConfig {
        printer: printer.clone(),
        options,
        reads: GCell::new(Vec::new()),
    });
    let config: Config = config_fake.clone() as Config;
    RustShell {
        printer: printer_arc,
        config,
        toolhead,
        gcode,
        gcode_move,
        configfile,
        config_fake,
    }
}

fn build_pair(shell: RustShell) -> KResult<RustPair> {
    let bedtilt = BedTilt::new(&shell.config)?;
    Ok(RustPair {
        bedtilt,
        toolhead: shell.toolhead,
        gcode: shell.gcode,
        gcode_move: shell.gcode_move,
        configfile: shell.configfile,
        printer: shell.printer,
        config_fake: shell.config_fake,
    })
}

// Temporary printer used only to satisfy FakeConfigFile construction; the
// configfile fake never delegates to it in any bed_tilt path.
struct FakePrinterEmpty;
impl PrinterApi for FakePrinterEmpty {
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
        _name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match default {
            LookupDefault::Raise => Err(KlippyError::config(
                "Unknown config object".to_string(),
            )),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match default {
            LookupDefault::Raise => Err(KlippyError::config(
                "Unable to load module".to_string(),
            )),
            LookupDefault::ReturnNone => Ok(None),
        }
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

fn py_str_literal(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            _ => out.push(c),
        }
    }
    out.push('\'');
    out
}

fn py_options_literal(cfg: SetupConfig) -> String {
    let mut parts: Vec<String> = Vec::new();
    if let Some(v) = cfg.x_adjust {
        parts.push(format!("'x_adjust': {}", py_str_literal(v)));
    }
    if let Some(v) = cfg.y_adjust {
        parts.push(format!("'y_adjust': {}", py_str_literal(v)));
    }
    if let Some(v) = cfg.z_adjust {
        parts.push(format!("'z_adjust': {}", py_str_literal(v)));
    }
    if let Some(v) = cfg.points {
        parts.push(format!("'points': {}", py_str_literal(v)));
    }
    format!("{{{}}}", parts.join(", "))
}

fn py_float_list(values: &[f64]) -> String {
    let inner: Vec<String> = values.iter().map(|v| float_repr(*v)).collect();
    format!("[{}]", inner.join(", "))
}

fn py_shell_setup(py: &mut pyoracle::Oracle, cfg: SetupConfig, toolhead_pos: &[f64]) {
    let expr = format!(
        "make_shell({}, {})",
        py_options_literal(cfg),
        py_float_list(toolhead_pos)
    );
    py.exec(&expr).expect("py make_shell");
}

fn fresh_pair(
    py: &mut pyoracle::Oracle,
    cfg: SetupConfig,
    toolhead_pos: Vec<f64>,
) -> RustPair {
    py_shell_setup(py, cfg, &toolhead_pos);
    py.exec("build_bedtilt()").expect("py build_bedtilt");
    let shell = make_shell(cfg, toolhead_pos);
    build_pair(shell).expect("rust BedTilt construction")
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("bed_tilt fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping bed_tilt oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.bed_tilt".to_string(),
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

fn check_unit<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    match actual {
        Ok(_) => check(label, Ok(expected), Value::Null),
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    }
}

fn check_floats(label: &str, expected: pyoracle::OracleResult, actual: Vec<f64>) {
    let value = Value::List(actual.iter().map(|v| Value::float(*v)).collect());
    check(label, expected, value);
}

fn check_strs(label: &str, expected: pyoracle::OracleResult, actual: &[String]) {
    let value = Value::List(actual.iter().map(|s| Value::str(s)).collect());
    check(label, expected, value);
}

fn check_err<T>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<T>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class",
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message",
    );
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn config_defaults_match_python() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig::default(),
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval("[bt.x_adjust, bt.y_adjust, bt.z_adjust]");
    check_floats(
        "config_defaults_match_python",
        expected,
        vec![pair.bedtilt.x_adjust(), pair.bedtilt.y_adjust(), pair.bedtilt.z_adjust()],
    );
}

#[test]
fn config_options_are_read_with_defaults() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            x_adjust: Some("0.01"),
            y_adjust: Some("-0.02"),
            z_adjust: Some("0.003"),
            points: Some("10,10\n10,20\n20,10"),
        },
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval("c.reads");
    let actual = Value::List(
        pair.config_fake
            .reads
            .borrow()
            .iter()
            .map(|(kind, option)| Value::Tuple(vec![Value::str(kind), Value::str(option)]))
            .collect(),
    );
    check("config_options_are_read_with_defaults", expected, actual);
}

#[test]
fn get_position_applies_adjust() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            x_adjust: Some("0.001"),
            y_adjust: Some("-0.002"),
            z_adjust: Some("0.5"),
            points: None,
        },
        vec![100.0, 200.0, 10.0, 0.0],
    );
    pair.bedtilt.handle_connect().expect("rust handle_connect");
    py.exec("bt.handle_connect()").expect("py handle_connect");
    let expected = py.eval("bt.get_position()");
    let actual = pair.bedtilt.get_position();
    check_floats("get_position_applies_adjust", expected, actual.unwrap());
}

#[test]
fn get_position_keeps_extra_axes() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            x_adjust: Some("0.01"),
            y_adjust: Some("0.01"),
            z_adjust: Some("1.0"),
            points: None,
        },
        vec![10.0, 20.0, 5.0, 7.5],
    );
    pair.bedtilt.handle_connect().expect("rust handle_connect");
    py.exec("bt.handle_connect()").expect("py handle_connect");
    let expected = py.eval("bt.get_position()");
    check_floats(
        "get_position_keeps_extra_axes",
        expected,
        pair.bedtilt.get_position().unwrap(),
    );
}

#[test]
fn move_applies_adjust_before_toolhead() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            x_adjust: Some("0.001"),
            y_adjust: Some("0.002"),
            z_adjust: Some("0.25"),
            points: None,
        },
        vec![0.0, 0.0, 0.0, 0.0],
    );
    pair.bedtilt.handle_connect().expect("rust handle_connect");
    py.exec("bt.handle_connect()").expect("py handle_connect");
    let newpos = vec![50.0, 60.0, 5.0, 0.0];
    let expected = py.eval(&format!(
        "bt.move({}, 250.0)",
        py_float_list(&newpos)
    ));
    check_unit(
        "move_applies_adjust_before_toolhead ret",
        expected,
        pair.bedtilt.r#move(newpos, 250.0),
    );
    let expected = py.eval("[[list(pos), speed] for pos, speed in toolhead_fake.moves]");
    let actual = Value::List(
        pair.toolhead
            .moves
            .borrow()
            .iter()
            .map(|(pos, speed)| {
                Value::List(vec![
                    Value::List(pos.iter().map(|v| Value::float(*v)).collect()),
                    Value::float(*speed),
                ])
            })
            .collect(),
    );
    check("move_applies_adjust_before_toolhead moves", expected, actual);
}

#[test]
fn move_and_get_position_before_connect_raise_attribute_error() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig::default(),
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval("bt.get_position()");
    check_err(
        "move_and_get_position_before_connect get_position",
        expected,
        pair.bedtilt.get_position(),
    );
    let expected = py.eval("bt.move([1.0, 2.0, 3.0, 0.0], 100.0)");
    check_err(
        "move_and_get_position_before_connect move",
        expected,
        pair.bedtilt.r#move(vec![1.0, 2.0, 3.0, 0.0], 100.0),
    );
}

#[test]
fn connect_event_is_registered() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig::default(),
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval("sorted(p.handlers.keys())");
    check_strs(
        "connect_event_is_registered",
        expected,
        &pair.printer.events.borrow().clone(),
    );
}

#[test]
fn update_adjust_writes_config_and_resets() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig::default(),
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval("bt.update_adjust(0.000123, -0.000456, 0.000789)");
    check_unit(
        "update_adjust_writes_config_and_resets ret",
        expected,
        pair.bedtilt.update_adjust(0.000123, -0.000456, 0.000789),
    );
    let expected = py
        .eval("[[sec, opt, val] for (sec, opt), val in configfile_fake.settings.items()]");
    let actual = Value::List(
        pair.configfile
            .settings
            .borrow()
            .iter()
            .map(|(sec, opt, val)| {
                Value::List(vec![
                    Value::str(sec),
                    Value::str(opt),
                    Value::str(val),
                ])
            })
            .collect(),
    );
    check("update_adjust_writes_config_and_resets settings", expected, actual);
    let expected = py.eval("gcode_move_fake.reset_calls");
    check(
        "update_adjust_writes_config_and_resets reset",
        expected,
        Value::int(pair.gcode_move.reset_calls.get() as i64),
    );
    let expected = py.eval("[bt.x_adjust, bt.y_adjust, bt.z_adjust]");
    check_floats(
        "update_adjust_writes_config_and_resets adjustments",
        expected,
        vec![
            pair.bedtilt.x_adjust(),
            pair.bedtilt.y_adjust(),
            pair.bedtilt.z_adjust(),
        ],
    );
}

#[test]
fn calibrate_registers_bed_tilt_calibrate_command() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            points: Some("10,10\n10,20\n20,10"),
            ..Default::default()
        },
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval(
        "[[cmd, desc] for cmd, (func, desc) in sorted(gcode_fake.registered.items())]",
    );
    let actual = Value::List(
        pair.gcode
            .registered
            .borrow()
            .iter()
            .map(|(cmd, desc)| {
                Value::List(vec![
                    Value::str(cmd),
                    match desc {
                        Some(d) => Value::str(d),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    );
    check("calibrate_registers_bed_tilt_calibrate_command", expected, actual);
    let expected = py.eval("bed_tilt.BedTiltCalibrate.cmd_BED_TILT_CALIBRATE_help");
    check(
        "calibrate_registers_bed_tilt_calibrate_command help",
        expected,
        Value::str(CMD_BED_TILT_CALIBRATE_HELP),
    );
    assert!(
        pair.bedtilt.calibrate().is_some(),
        "bedtilt with points owns a calibrator"
    );
}

#[test]
fn calibrate_without_points_registers_nothing() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig::default(),
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let expected = py.eval("sorted(gcode_fake.registered.keys())");
    check_strs(
        "calibrate_without_points_registers_nothing",
        expected,
        &pair
            .gcode
            .registered
            .borrow()
            .iter()
            .map(|(cmd, _)| cmd.clone())
            .collect::<Vec<_>>(),
    );
    assert!(
        pair.bedtilt.calibrate().is_none(),
        "bedtilt without points has no calibrator"
    );
}

#[test]
fn calibrate_cmd_delegates_to_probe_helper() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            points: Some("10,10\n10,20\n20,10"),
            ..Default::default()
        },
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let calibrate = pair.bedtilt.calibrate().expect("calibrator");
    let gcmd: GCmd = Arc::new(FakeGcmd {
        command: "BED_TILT_CALIBRATE".to_string(),
        commandline: "BED_TILT_CALIBRATE".to_string(),
        params: GCell::new(IndexMap::new()),
    });
    // With no `probe` object both sides take the manual-probe path: the
    // helper raises inside `toolhead.manual_move` (the fake toolhead has no
    // kinematics), so compare the raised error and the recorded manual move.
    let expected = py.eval(
        "gcode_fake.registered['BED_TILT_CALIBRATE'][0](FakeGcmd('BED_TILT_CALIBRATE'))",
    );
    let mut handler = pair
        .gcode
        .take_handler("BED_TILT_CALIBRATE")
        .expect("registered handler");
    let actual_result = handler(&gcmd);
    match expected {
        Ok(_) => check_unit(
            "calibrate_cmd_delegates_to_probe_helper ret",
            Ok(Value::Null),
            actual_result,
        ),
        Err(e) => check_err(
            "calibrate_cmd_delegates_to_probe_helper ret",
            Err(e),
            actual_result.map(|_| ()),
        ),
    }
    let expected = py.eval("toolhead_fake.manual_moves");
    let actual = Value::List(
        pair.toolhead
            .manual_moves
            .borrow()
            .iter()
            .map(|(coord, speed)| {
                // `manual_moves.append((coord, speed))` on the Python side.
                Value::Tuple(vec![
                    Value::List(
                        coord
                            .iter()
                            .map(|v| match v {
                                Some(f) => Value::float(*f),
                                None => Value::Null,
                            })
                            .collect(),
                    ),
                    Value::float(*speed),
                ])
            })
            .collect(),
    );
    check("calibrate_cmd_delegates_to_probe_helper moves", expected, actual);
    let _ = calibrate;
}

#[test]
fn probe_finalize_computes_adjustments() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            points: Some("10,10\n10,20\n20,10"),
            ..Default::default()
        },
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let calibrate = pair.bedtilt.calibrate().expect("calibrator");
    // Plane z = 0.5 + 0.001*x + 0.002*y sampled at the three corners.
    let positions = vec![
        ProbeResult {
            bed_x: 10.0,
            bed_y: 10.0,
            bed_z: 0.53,
            test_x: 10.0,
            test_y: 10.0,
            test_z: 0.53,
        },
        ProbeResult {
            bed_x: 10.0,
            bed_y: 20.0,
            bed_z: 0.55,
            test_x: 10.0,
            test_y: 20.0,
            test_z: 0.55,
        },
        ProbeResult {
            bed_x: 20.0,
            bed_y: 10.0,
            bed_z: 0.54,
            test_x: 20.0,
            test_y: 10.0,
            test_z: 0.54,
        },
    ];
    py.exec("log_messages.clear()")
        .expect("py clear logs");
    let expected = py.eval(
        "gcode_fake.registered['BED_TILT_CALIBRATE'][0].__self__.probe_helper.finalize_callback([\
        ProbeResult(10.0, 10.0, 0.53, 10.0, 10.0, 0.53),\
        ProbeResult(10.0, 20.0, 0.55, 10.0, 20.0, 0.55),\
        ProbeResult(20.0, 10.0, 0.54, 20.0, 10.0, 0.54)])",
    );
    let (actual_result, rust_logs) = with_log_capture(|| calibrate.probe_finalize(&positions));
    check_unit(
        "probe_finalize_computes_adjustments ret",
        expected,
        actual_result,
    );
    let expected = py.eval("[bt.x_adjust, bt.y_adjust, bt.z_adjust]");
    check_floats(
        "probe_finalize_computes_adjustments adjustments",
        expected,
        vec![
            pair.bedtilt.x_adjust(),
            pair.bedtilt.y_adjust(),
            pair.bedtilt.z_adjust(),
        ],
    );
    let expected = py.eval("gcode_fake.responded");
    check_strs(
        "probe_finalize_computes_adjustments responded",
        expected,
        &pair.gcode.responded.borrow().clone(),
    );
    let expected = py.eval("[[name, info, log] for name, info, log in p.rollover]");
    let actual = Value::List(
        pair.printer
            .rollover
            .borrow()
            .iter()
            .map(|(name, info, is_log)| {
                Value::List(vec![
                    Value::str(name),
                    match info {
                        Some(i) => Value::str(i),
                        None => Value::Null,
                    },
                    Value::Bool(*is_log),
                ])
            })
            .collect(),
    );
    check("probe_finalize_computes_adjustments rollover", expected, actual);
    let expected = py.eval("list(log_messages)");
    check_strs("probe_finalize_computes_adjustments logs", expected, &rust_logs);
}

#[test]
fn probe_finalize_singular_raises_unable_to_calculate() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        SetupConfig {
            points: Some("10,10\n10,20\n20,10"),
            ..Default::default()
        },
        vec![0.0, 0.0, 0.0, 0.0],
    );
    let calibrate = pair.bedtilt.calibrate().expect("calibrator");
    // All points share one xy: the fit is singular.
    let positions = vec![
        ProbeResult {
            bed_x: 10.0,
            bed_y: 10.0,
            bed_z: 1.0,
            test_x: 10.0,
            test_y: 10.0,
            test_z: 1.0,
        },
        ProbeResult {
            bed_x: 10.0,
            bed_y: 10.0,
            bed_z: 2.0,
            test_x: 10.0,
            test_y: 10.0,
            test_z: 2.0,
        },
        ProbeResult {
            bed_x: 10.0,
            bed_y: 10.0,
            bed_z: 3.0,
            test_x: 10.0,
            test_y: 10.0,
            test_z: 3.0,
        },
    ];
    let expected = py.eval(
        "gcode_fake.registered['BED_TILT_CALIBRATE'][0].__self__.probe_helper.finalize_callback([\
        ProbeResult(10.0, 10.0, 1.0, 10.0, 10.0, 1.0),\
        ProbeResult(10.0, 10.0, 2.0, 10.0, 10.0, 2.0),\
        ProbeResult(10.0, 10.0, 3.0, 10.0, 10.0, 3.0)])",
    );
    let (actual, _logs) = with_log_capture(|| calibrate.probe_finalize(&positions));
    let actual = actual.map(|_| ());
    check_err(
        "probe_finalize_singular_raises_unable_to_calculate",
        expected,
        actual,
    );
    let expected = py.eval("[bt.x_adjust, bt.y_adjust, bt.z_adjust]");
    check_floats(
        "probe_finalize_singular_raises_unable_to_calculate adjustments untouched",
        expected,
        vec![
            pair.bedtilt.x_adjust(),
            pair.bedtilt.y_adjust(),
            pair.bedtilt.z_adjust(),
        ],
    );
}

#[test]
fn probe_positions_repr_matches_python() {
    let Some(mut py) = oracle() else { return };
    let positions = vec![
        ProbeResult {
            bed_x: 10.0,
            bed_y: 10.5,
            bed_z: 0.53,
            test_x: 10.0,
            test_y: 10.5,
            test_z: 0.53,
        },
        ProbeResult {
            bed_x: 20.0,
            bed_y: 30.0,
            bed_z: 0.6,
            test_x: 20.0,
            test_y: 30.0,
            test_z: 0.6,
        },
    ];
    let expected = py.eval(
        "str([ProbeResult(10.0, 10.5, 0.53, 10.0, 10.5, 0.53), \
        ProbeResult(20.0, 30.0, 0.6, 20.0, 30.0, 0.6)])",
    );
    check(
        "probe_positions_repr_matches_python",
        expected,
        Value::str(&probe_positions_repr(&positions)),
    );
}

#[test]
fn few_points_raise_config_error() {
    let Some(mut py) = oracle() else { return };
    let cfg = SetupConfig {
        points: Some("10,10\n10,20"),
        ..Default::default()
    };
    poly_shell_setup(&mut py, cfg, &[0.0, 0.0, 0.0, 0.0]);
    let expected = py.eval("bed_tilt.load_config(c)");
    let shell = make_shell(cfg, vec![0.0, 0.0, 0.0, 0.0]);
    let actual = BedTilt::new(&shell.config).map(|_| ());
    check_err("few_points_raise_config_error", expected, actual);
}

fn poly_shell_setup(
    py: &mut pyoracle::Oracle,
    cfg: SetupConfig,
    toolhead_pos: &[f64],
) {
    py_shell_setup(py, cfg, toolhead_pos);
}

#[test]
fn invalid_adjust_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let cfg = SetupConfig {
        x_adjust: Some("not-a-float"),
        ..Default::default()
    };
    py_shell_setup(&mut py, cfg, &[0.0, 0.0, 0.0, 0.0]);
    let expected = py.eval("bed_tilt.load_config(c)");
    let shell = make_shell(cfg, vec![0.0, 0.0, 0.0, 0.0]);
    let actual = BedTilt::new(&shell.config).map(|_| ());
    check_err("invalid_adjust_raises_config_error", expected, actual);
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::bed_tilt::module_factory();
    let load_config = factory
        .load_config
        .expect("bed_tilt exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "no add_early_printer_objects"
    );
    assert!(factory.add_printer_objects.is_none(), "no add_printer_objects");
    let cfg = SetupConfig {
        x_adjust: Some("0.005"),
        points: Some("10,10\n10,20\n20,10"),
        ..Default::default()
    };
    py_shell_setup(&mut py, cfg, &[0.0, 0.0, 0.0, 0.0]);
    py.exec("bt_factory = bed_tilt.load_config(c)")
        .expect("py factory load_config");
    let shell = make_shell(cfg, vec![0.0, 0.0, 0.0, 0.0]);
    let obj: ObjectRef = load_config(&shell.config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let rust: Arc<BedTilt> = downcast_arc(&obj).expect("downcast to BedTilt");
    let expected = py.eval("[bt_factory.x_adjust, bt_factory.y_adjust, bt_factory.z_adjust]");
    check_floats(
        "load_config_factory_builds_working_object",
        expected,
        vec![rust.x_adjust(), rust.y_adjust(), rust.z_adjust()],
    );
    let expected = py.eval("sorted(p.handlers.keys())");
    let events = shell.printer.events.borrow().clone();
    check_strs(
        "load_config_factory_builds_working_object events",
        expected,
        &events,
    );
}
