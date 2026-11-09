//! Parity tests for `extras::gcode_move` (port of `klippy/extras/gcode_move.py`).

use extras::gcode_move::{module_factory, GCodeMove};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GetIntOpts as GGetIntOpts, GCodeApi, GCodeCommandApi, GCodeMoveApi, GCmd, MoveTransform,
    RegisterCommandOpts, GetFloatOpts as GGetFloatOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{ExtraAxisApi, KinematicsApi, ToolHeadApi, HomingTargetApi};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use pycompat::text::{float_from_string, int_from_string, float_repr};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// Python fakes
const SETUP: &str = r#"
import configparser
import extras.gcode_move as gcode_move
from gcode import Coord as _Coord
import gcode as _gcode_mod

class CommandError(Exception):
    pass

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.handlers = {}
    def lookup_object(self, name, default=None):
        # Mirror real Printer.lookup_object: sentinel means raise.
        import configfile
        if name in self.objects:
            return self.objects[name]
        if default is None and _no_default_marker[0]:
            raise self.config_error("Unknown config object '%s'" % (name,))
        # We distinguish: caller passed no default vs explicit None by marker.
        return default
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

_no_default_marker = [False]
# Wrap lookup to handle two-arg vs one-arg calls faithfully:
_orig_lookup = FakePrinter.lookup_object
def _lookup(self, name, default=None, *a):
    # If called with only one arg after self, raise on missing; if two args, return default.
    import inspect
    return _orig_lookup(self, name, default)
# Simpler: redefine lookup_object to use a sentinel.
class _Sentinel: pass
_S = _Sentinel()
def _fake_lookup(self, name, default=_S):
    if name in self.objects:
        return self.objects[name]
    if default is _S:
        raise self.config_error("Unknown config object '%s'" % (name,))
    return default
FakePrinter.lookup_object = _fake_lookup

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
    def get_name(self):
        return "gcode_move"

class FakeGCode:
    error = CommandError
    Coord = _Coord
    def __init__(self):
        self.regs = []
        self.outputs = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.regs.append([cmd, when_not_ready, desc])
    def respond_raw(self, msg):
        self.outputs.append(["raw", msg])
    def respond_info(self, msg, log=True):
        self.outputs.append(["info", msg, log])
        # Mirror real respond_info forwarding to respond_raw with // prefix is not
        # needed for parity of the message handed to respond_info; tests compare
        # the message argument itself via gcmd outputs.
    def create_gcode_command(self, command, commandline, params):
        return FakeGCodeCommand(self, command, commandline, params)

class _Sent2: pass
_SENT = _Sent2()

class FakeGCodeCommand:
    error = CommandError
    def __init__(self, gcode, command, commandline, params):
        self._gcode = gcode
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
        self.raw_out = []
        self.info_out = []
    def get_command(self):
        return self._command
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return dict(self._params)
    def respond_raw(self, msg):
        self.raw_out.append(msg)
        self._gcode.respond_raw(msg)
    def respond_info(self, msg, log=True):
        self.info_out.append([msg, log])
        self._gcode.respond_info(msg, log)
    def get(self, name, default=_SENT, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _SENT:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
        except:
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
    def get_int(self, name, default=_SENT, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)
    def get_float(self, name, default=_SENT, minval=None, maxval=None, above=None, below=None):
        return self.get(name, default, parser=float, minval=minval, maxval=maxval, above=above, below=below)

class FakeStepper:
    def __init__(self, name, cmd_pos, mcu_pos):
        self._name = name
        self._cmd = cmd_pos
        self._mcu = mcu_pos
    def get_name(self, short=False):
        return self._name
    def get_commanded_position(self):
        return self._cmd
    def get_mcu_position(self, cmd_pos=None):
        return self._mcu

class FakeKinematics:
    def __init__(self, steppers):
        self._steppers = steppers
    def get_steppers(self):
        return list(self._steppers)
    def calc_position(self, d):
        return [d.get("stepper_x", 0.0), d.get("stepper_y", 0.0), d.get("stepper_z", 0.0)]

class FakeExtraAxis:
    def __init__(self, gid):
        self._gid = gid
    def get_axis_gcode_id(self):
        return self._gid

class FakeToolHead:
    def __init__(self, position=None, extra_ids=None):
        self.position = list(position) if position is not None else [0.0, 0.0, 0.0, 0.0]
        self.moves = []
        self.kin = FakeKinematics([FakeStepper("stepper_x", 10.0, 100), FakeStepper("stepper_y", 20.0, 200), FakeStepper("stepper_z", 30.0, 300)])
        self.extras = []
        if extra_ids:
            for gid in extra_ids:
                self.extras.append(FakeExtraAxis(gid))
    def move(self, pos, speed):
        self.moves.append([list(pos), speed])
        self.position = list(pos)
    def get_position(self):
        return list(self.position)
    def get_kinematics(self):
        return self.kin
    def get_extra_axes(self):
        return [None, None, None] + list(self.extras)

def make_gcmd(gcode, cmd, cmdline, params):
    return FakeGCodeCommand(gcode, cmd, cmdline, params)
"#;

// Rust fakes

struct FakePrinter {
    objects: klippy_api::GCell<IndexMap<String, ObjectRef>>,
    handlers: klippy_api::GCell<IndexMap<String, Vec<EventCallback>>>,
}

impl FakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: klippy_api::GCell::new(IndexMap::new()),
            handlers: klippy_api::GCell::new(IndexMap::new()),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objs = self.objects.borrow_mut();
        if objs.contains_key(name) {
            return Err(klippy_api::KlippyError::config(format!("Printer object '{}' already created", name)));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objs = self.objects.borrow();
        if let Some(o) = objs.get(name) { return Ok(Some(o.clone())); }
        match default {
            LookupDefault::Raise => Err(klippy_api::KlippyError::config(format!("Unknown config object '{}'", name))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, Option<ObjectRef>)> { Vec::new() }
    fn load_object(&self, _c: &Config, _s: &str, _d: LookupDefault) -> KResult<Option<ObjectRef>> { Ok(None) }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().entry(event.to_string()).or_default().push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let mut handlers = self.handlers.borrow_mut();
        if let Some(cbs) = handlers.get_mut(event) {
            for cb in cbs.iter_mut() {
                results.push(cb(params)?);
            }
        }
        Ok(results)
    }
    fn request_exit(&self, _r: &str) {}
}


struct FakeConfig { printer: Printer }

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { "gcode_move".to_string() }
    fn get(&self, _o: &str, _x: GetStrOpts) -> KResult<Option<String>> { Ok(None) }
    fn getint(&self, _o: &str, _x: GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn getfloat(&self, _o: &str, _x: GetFloatOpts) -> KResult<Option<f64>> { Ok(None) }
    fn getboolean(&self, _o: &str, _x: GetBoolOpts) -> KResult<Option<bool>> { Ok(None) }
    fn getchoice(&self, _o: &str, _c: &PyValue, _x: GetChoiceOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlists(&self, _o: &str, _x: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _o: &str, _x: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _o: &str, _x: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _o: &str, _x: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, _s: &str) -> Config { panic!("getsection unused") }
    fn has_section(&self, _s: &str) -> bool { false }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> { Ok(Vec::new()) }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// Fake GCode
struct FakeGCode {
    regs: klippy_api::GCell<Vec<(String, bool, Option<String>)>>,
    outputs: klippy_api::GCell<Vec<String>>,
}
impl FakeGCode {
    fn new() -> Arc<Self> {
        Arc::new(FakeGCode { regs: klippy_api::GCell::new(Vec::new()), outputs: klippy_api::GCell::new(Vec::new()) })
    }
}
impl GCodeApi for FakeGCode {
    fn register_command(&self, cmd: &str, _func: Option<Box<dyn FnMut(&GCmd) -> KResult<()> + Send>>, opts: RegisterCommandOpts) -> KResult<Option<Box<dyn FnMut(&GCmd) -> KResult<()> + Send>>> {
        self.regs.borrow_mut().push((cmd.to_string(), opts.when_not_ready, opts.desc.clone()));
        Ok(None)
    }
    fn register_mux_command(&self, _c: &str, _k: &str, _v: Option<&str>, _f: Box<dyn FnMut(&GCmd) -> KResult<()> + Send>, _d: Option<&str>) -> KResult<()> { Ok(()) }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _e: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn register_output_handler(&self, _cb: Box<dyn FnMut(String) + Send>) {}
    fn request_restart(&self, _r: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> ReactorMutex { Reactor::new(false).mutex(false) }
    fn create_gcode_command(&self, command: &str, commandline: &str, params: &IndexMap<String, String>) -> GCmd {
        let g: GCmd = Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
            raw_out: klippy_api::GCell::new(Vec::new()),
            info_out: klippy_api::GCell::new(Vec::new()),
        });
        g
    }
    fn respond_raw(&self, msg: &str) { self.outputs.borrow_mut().push(msg.to_string()); }
    fn respond_info(&self, msg: &str, _log: bool) { self.outputs.borrow_mut().push(msg.to_string()); }
    fn is_traditional_gcode(&self, _c: &str) -> bool { false }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> { Ok(()) }
}
impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

// Fake GCmd
struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    raw_out: klippy_api::GCell<Vec<String>>,
    info_out: klippy_api::GCell<Vec<(String, bool)>>,
}
fn make_gcmd(command: &str, commandline: &str, params: Vec<(&str, &str)>) -> GCmd {
    let mut m = IndexMap::new();
    for (k, v) in params { m.insert(k.to_string(), v.to_string()); }
    let g: GCmd = Arc::new(FakeGcmd {
        command: command.to_string(),
        commandline: commandline.to_string(),
        params: m,
        raw_out: klippy_api::GCell::new(Vec::new()),
        info_out: klippy_api::GCell::new(Vec::new()),
    });
    g
}
impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        let value = self.params.get(name);
        let value = match value {
            None => {
                match opts.default {
                    None => return Err(klippy_api::KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                    Some(d) => return Ok(d),
                }
            }
            Some(v) => v.clone(),
        };
        let parsed: PyValue = match opts.parser {
            klippy_api::gcode::GetParser::Str => PyValue::Str(value.clone()),
            klippy_api::gcode::GetParser::Int => {
                let n = int_from_string(&value, 10).map_err(|_| {
                    klippy_api::KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, value))
                })?;
                PyValue::Int(n)
            }
            klippy_api::gcode::GetParser::IntAutoBase => {
                let n = int_from_string(&value, 0).map_err(|_| {
                    klippy_api::KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, value))
                })?;
                PyValue::Int(n)
            }
            klippy_api::gcode::GetParser::Float => {
                let f = float_from_string(&value).map_err(|_| {
                    klippy_api::KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, value))
                })?;
                PyValue::Float(f)
            }
        };
        // bounds checks mirror gcode.py order
        if let Some(minv) = opts.minval {
            let fv = parsed.to_f64();
            let mv = minv.to_f64();
            if let (Some(a), Some(b)) = (fv, mv) {
                if a < b {
                    return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must have minimum of {}", self.commandline, name, minv.str())));
                }
            }
        }
        if let Some(maxv) = opts.maxval {
            let fv = parsed.to_f64();
            let mv = maxv.to_f64();
            if let (Some(a), Some(b)) = (fv, mv) {
                if a > b {
                    return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must have maximum of {}", self.commandline, name, maxv.str())));
                }
            }
        }
        if let Some(ab) = opts.above {
            let fv = parsed.to_f64();
            let mv = ab.to_f64();
            if let (Some(a), Some(b)) = (fv, mv) {
                if a <= b {
                    return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must be above {}", self.commandline, name, ab.str())));
                }
            }
        }
        if let Some(be) = opts.below {
            let fv = parsed.to_f64();
            let mv = be.to_f64();
            if let (Some(a), Some(b)) = (fv, mv) {
                if a >= b {
                    return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must be below {}", self.commandline, name, be.str())));
                }
            }
        }
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let value = match self.params.get(name) {
            None => {
                match opts.default {
                    None => return Err(klippy_api::KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                    Some(d) => return Ok(d),
                }
            }
            Some(v) => v.clone(),
        };
        let parsed = int_from_string(&value, 10).map_err(|_| {
            klippy_api::KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, value))
        })?;
        // bounds: pycompat PyInt to i64 approx via to_string parse for comparison
        let n: i64 = parsed.inner().to_string().parse().unwrap_or(0);
        if let Some(mn) = opts.minval {
            if n < mn {
                return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must have minimum of {}", self.commandline, name, mn)));
            }
        }
        if let Some(mx) = opts.maxval {
            if n > mx {
                return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must have maximum of {}", self.commandline, name, mx)));
            }
        }
        Ok(Some(n))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let value = match self.params.get(name) {
            None => {
                match opts.default {
                    None => return Err(klippy_api::KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                    Some(d) => return Ok(d),
                }
            }
            Some(v) => v.clone(),
        };
        let v = float_from_string(&value).map_err(|_| {
            klippy_api::KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, value))
        })?;
        if let Some(mn) = opts.minval {
            if v < mn {
                return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must have minimum of {}", self.commandline, name, float_repr(mn))));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must have maximum of {}", self.commandline, name, float_repr(mx))));
            }
        }
        if let Some(ab) = opts.above {
            if v <= ab {
                return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must be above {}", self.commandline, name, float_repr(ab))));
            }
        }
        if let Some(be) = opts.below {
            if v >= be {
                return Err(klippy_api::KlippyError::command(format!("Error on '{}': {} must be below {}", self.commandline, name, float_repr(be))));
            }
        }
        Ok(Some(v))
    }
    fn get_command(&self) -> String { self.command.clone() }
    fn get_commandline(&self) -> String { self.commandline.clone() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.clone() }
    fn get_raw_command_parameters(&self) -> String { String::new() }
    fn respond_info(&self, msg: &str, log: bool) { self.info_out.borrow_mut().push((msg.to_string(), log)); }
    fn respond_raw(&self, msg: &str) { self.raw_out.borrow_mut().push(msg.to_string()); }
    fn ack(&self, _m: Option<&str>) -> bool { false }
}

// Fake ToolHead / Kinematics / Stepper / ExtraAxis
struct FakeStepper { name: String, cmd: f64, mcu: i64 }
impl klippy_api::stepper::McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { panic!("get_mcu unused") }
    fn get_name(&self, _s: Option<bool>) -> String { self.name.clone() }
    fn get_oid(&self) -> i64 { 0 }
    fn units_in_radians(&self) -> bool { false }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) { (None, false) }
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {}
    fn setup_itersolve(&self, _a: &str, _p: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 { 0.0 }
    fn get_rotation_distance(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) { (false, false) }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 { 0.0 }
    fn set_position(&self, _c: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 { self.cmd }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 { self.mcu }
    fn get_past_mcu_position(&self, _t: f64) -> i64 { 0 }
    fn mcu_to_commanded_position(&self, _m: f64) -> f64 { 0.0 }
    fn dump_steps(&self, _c: i64, _s: i64, _e: i64) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) { (Vec::new(), 0) }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> { None }
    fn set_stepper_kinematics(&self, _s: Option<klippy_api::stepper::StepperKinematics>) -> Option<klippy_api::stepper::StepperKinematics> { None }
    fn note_homing_end(&self) -> KResult<()> { Ok(()) }
    fn get_trapq(&self) -> Option<klippy_api::stepper::Trapq> { None }
    fn set_trapq(&self, _t: Option<klippy_api::stepper::Trapq>) -> Option<klippy_api::stepper::Trapq> { None }
    fn add_active_callback(&self, _c: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _a: char) -> bool { false }
}
struct FakeKinematics { steppers: Vec<klippy_api::stepper::Stepper> }
impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> { self.steppers.clone() }
    fn calc_position(&self, d: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        let x = *d.get("stepper_x").unwrap_or(&0.0);
        let y = *d.get("stepper_y").unwrap_or(&0.0);
        let z = *d.get("stepper_z").unwrap_or(&0.0);
        Ok(vec![Some(x), Some(y), Some(z)])
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> { Ok(()) }
    fn check_move(&self, _m: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> { Ok(()) }
    fn home(&self, _h: klippy_api::toolhead::HomingStateHandle) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> { Ok(PyValue::Dict(IndexMap::new())) }
    fn clear_homing_state(&self, _c: &str) -> klippy_api::KResult<()> { Ok(()) }
}
struct FakeExtraAxis { gid: Option<String> }
impl ExtraAxisApi for FakeExtraAxis {
    fn get_name(&self) -> String { self.gid.clone().unwrap_or_default() }
    fn get_axis_gcode_id(&self) -> Option<String> { self.gid.clone() }
    fn get_trapq(&self) -> Option<klippy_api::stepper::Trapq> { None }
    fn check_move(&self, _m: &dyn klippy_api::toolhead::MoveApi, _i: usize) -> KResult<()> { Ok(()) }
    fn calc_junction(&self, _p: &dyn klippy_api::toolhead::MoveApi, _m: &dyn klippy_api::toolhead::MoveApi, _i: usize) -> klippy_api::KResult<f64> { Ok(0.0) }
    fn process_move(&self, _t: f64, _m: &dyn klippy_api::toolhead::MoveApi, _i: usize) {}
}
struct FakeToolHead {
    position: klippy_api::GCell<Vec<f64>>,
    moves: klippy_api::GCell<Vec<(Vec<f64>, f64)>>,
    kin: klippy_api::toolhead::Kinematics,
    extras: klippy_api::GCell<Vec<Option<klippy_api::toolhead::ExtraAxis>>>,
}
impl FakeToolHead {
    fn new(position: Vec<f64>, extra_ids: Vec<Option<String>>) -> Arc<Self> {
        let steppers: Vec<klippy_api::stepper::Stepper> = vec![
            Arc::new(FakeStepper { name: "stepper_x".to_string(), cmd: 10.0, mcu: 100 }),
            Arc::new(FakeStepper { name: "stepper_y".to_string(), cmd: 20.0, mcu: 200 }),
            Arc::new(FakeStepper { name: "stepper_z".to_string(), cmd: 30.0, mcu: 300 }),
        ];
        let kin: klippy_api::toolhead::Kinematics = Arc::new(FakeKinematics { steppers });
        let extras: Vec<Option<klippy_api::toolhead::ExtraAxis>> =
            extra_ids.into_iter().map(|gid| gid.map(|g| {
                let a: klippy_api::toolhead::ExtraAxis = Arc::new(FakeExtraAxis { gid: Some(g) });
                a
            })).collect();
        Arc::new(FakeToolHead {
            position: klippy_api::GCell::new(position),
            moves: klippy_api::GCell::new(Vec::new()),
            kin,
            extras: klippy_api::GCell::new(extras),
        })
    }
}
impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> { self.position.cloned() }
    fn set_position(&self, newpos: Vec<f64>, _h: &str) -> KResult<()> { *self.position.borrow_mut() = newpos; Ok(()) }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics { self.kin.clone() }
    fn flush_step_generation(&self) -> KResult<()> { Ok(()) }
    fn get_last_move_time(&self) -> KResult<f64> { Ok(0.0) }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &Completion<bool>) -> KResult<()> { Ok(()) }
}
impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.moves.borrow_mut().push((newpos.clone(), speed));
        *self.position.borrow_mut() = newpos;
        Ok(())
    }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> { Ok(()) }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> { Ok(PyValue::Dict(IndexMap::new())) }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq { panic!("trapq unused") }
    fn get_max_velocity(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_max_velocities(&self, _a: Option<f64>, _b: Option<f64>, _c: Option<f64>, _d: Option<f64>) -> (f64, f64, f64, f64) { (0.0, 0.0, 0.0, 0.0) }
    fn register_lookahead_callback(&self, _c: klippy_api::toolhead::LookaheadCallback) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) { (0.0, 0.0, false) }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: klippy_api::toolhead::PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder { panic!("extruder unused") }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        let mut v: Vec<Option<klippy_api::toolhead::ExtraAxis>> = vec![None, None, None];
        v.extend(self.extras.borrow().iter().cloned());
        v
    }
    fn stats(&self, _e: f64) -> (bool, String) { (false, String::new()) }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory { panic!("factory unused") }
}
impl PrinterObject for FakeToolHead {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ToolHeadApi);
}

// Helpers: conversion PyValue -> recording Value
fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.inner().to_string();
            if let Ok(i) = s.parse::<i64>() { Value::Int(i) } else { Value::BigInt(s) }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect()),
        PyValue::Host(h) => Value::Opaque { type_name: "host".to_string(), addr: format!("{:#x}", h) },
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => { py.exec(SETUP).expect("setup"); Some(py) }
        None => { println!("skipping gcode_move oracle tests"); None }
    }
}

struct Pair {
    rust: Arc<GCodeMove>,
    printer_rust: Arc<FakePrinter>,
    gcode_rust: Arc<FakeGCode>,
    toolhead_rust: Arc<FakeToolHead>,
    printer: Printer,
    config: Config,
}

fn fresh_pair(py: &mut pyoracle::Oracle, toolhead_pos: Vec<f64>, extra_ids: Vec<Option<String>>) -> Pair {
    // Python side
    let pos_repr = toolhead_pos.iter().map(|v| float_repr(*v)).collect::<Vec<_>>().join(", ");
    let extra_repr = extra_ids.iter().map(|g| match g { None => "None".to_string(), Some(s) => format!("'{s}'") }).collect::<Vec<_>>().join(", ");
    py.exec(&format!(
        "p = FakePrinter(); g = FakeGCode(); p.add_object('gcode', g); th = FakeToolHead(position=[{pos_repr} ], extra_ids=[{extra_repr}]); p.add_object('toolhead', th); c = FakeConfig(p); gm = gcode_move.load_config(c); p.add_object('gcode_move', gm)"
    )).expect("py load_config");
    // Rust side
    let printer_rust = FakePrinter::new();
    let printer: Printer = printer_rust.clone();
    let gcode_rust = FakeGCode::new();
    printer_rust.add_object("gcode", gcode_rust.clone() as ObjectRef).unwrap();
    let toolhead_rust = FakeToolHead::new(toolhead_pos, extra_ids);
    printer_rust.add_object("toolhead", toolhead_rust.clone() as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let rust = GCodeMove::new(&config).unwrap();
    printer_rust.add_object("gcode_move", rust.clone() as ObjectRef).unwrap();
    Pair { rust, printer_rust, gcode_rust, toolhead_rust, printer, config }
}

fn fire_ready(py: &mut pyoracle::Oracle, pair: &Pair) {
    py.exec("p.send_event('klippy:ready')").expect("py ready");
    pair.printer.send_event("klippy:ready", &[]).unwrap();
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record { v: 1, unit: "gcode_move".to_string(), scenario: label.to_string(), seq: 0, function: "test".to_string(), args: Vec::new(), ret: None, exc: None, side_effects: SideEffects::default(), object_id: None, kind: RecordKind::Python };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}
fn check_status(label: &str, expected: pyoracle::OracleResult, actual: &PyValue) {
    check(label, expected, pyvalue_to_value(actual));
}
fn check_cmd(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match (expected, actual) {
        (Ok(v), Ok(())) => {
            // Python methods return None on success
            assert_eq!(v, Value::Null, "{label}: expected None");
        }
        (Ok(v), Err(e)) => panic!("{label}: oracle returned {v:?} but rust raised {}: {}", e.class_name(), e.message()),
        (Err(exc), Ok(())) => panic!("{label}: oracle raised {}: {} but rust succeeded", exc.class_name, exc.message),
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "{label}: class");
            assert_eq!(err.message(), exc.message.as_str(), "{label}: message");
        }
    }
}
fn check_moves(label: &str, py: &mut pyoracle::Oracle, toolhead: &Arc<FakeToolHead>) {
    let expected = py.eval("th.moves").expect("eval moves");
    let moves = toolhead.moves.cloned();
    let actual = Value::List(moves.iter().map(|(pos, spd)| {
        Value::List(vec![
            Value::List(pos.iter().map(|v| Value::float(*v)).collect()),
            Value::float(*spd),
        ])
    }).collect());
    check(label, Ok(expected), actual);
}
fn check_status_both(label: &str, py: &mut pyoracle::Oracle, rust_obj: &Arc<GCodeMove>) {
    let expected = py.eval("gm.get_status()").expect("eval status");
    let actual = <GCodeMove as GCodeMoveApi>::get_status(&*rust_obj, None);
    check_status(label, Ok(expected), &actual);
}
fn py_repr_str(s: &str) -> String {
    // quote for python code
    let mut out = String::from("'");
    for c in s.chars() {
        match c { '\'' => out.push_str("\\'"), '\\' => out.push_str("\\\\"), c => out.push(c) }
    }
    out.push('\'');
    out
}
fn apply_cmd(py: &mut pyoracle::Oracle, rust_obj: &Arc<GCodeMove>, cmd: &str, cmdline: &str, params: Vec<(&str, &str)>, method: &str) -> (pyoracle::OracleResult, KResult<()>, Vec<String>, Vec<String>, Vec<String>, Vec<String>) {
    // Python: create gcmd then call
    let params_repr = params.iter().map(|(k, v)| format!("{}: {}", py_repr_str(k), py_repr_str(v))).collect::<Vec<_>>().join(", ");
    // Call via eval of method; need to capture result and outputs
    // We exec creation first then eval call
    let create = format!("gc = make_gcmd(g, '{}', '{}', {{{}}})", cmd, cmdline, params_repr);
    py.exec(&create).expect("py gcmd");
    let expected = py.eval(&format!("gm.{}(gc)", method));
    let py_raw = py.eval("gc.raw_out").unwrap_or(Value::Null);
    let py_info = py.eval("[x[0] for x in gc.info_out]").unwrap_or(Value::Null);
    // Rust
    let gcmd_concrete: Arc<FakeGcmd> = {
        let mut m = IndexMap::new();
        for (k, v) in params.iter() { m.insert(k.to_string(), v.to_string()); }
        Arc::new(FakeGcmd {
            command: cmd.to_string(),
            commandline: cmdline.to_string(),
            params: m,
            raw_out: klippy_api::GCell::new(Vec::new()),
            info_out: klippy_api::GCell::new(Vec::new()),
        })
    };
    let gcmd_trait: GCmd = gcmd_concrete.clone();
    let result = match method {
        "cmd_G1" => rust_obj.cmd_G1(&gcmd_trait),
        "cmd_G20" => rust_obj.cmd_G20(&gcmd_trait),
        "cmd_G21" => rust_obj.cmd_G21(&gcmd_trait),
        "cmd_M82" => rust_obj.cmd_M82(&gcmd_trait),
        "cmd_M83" => rust_obj.cmd_M83(&gcmd_trait),
        "cmd_G90" => rust_obj.cmd_G90(&gcmd_trait),
        "cmd_G91" => rust_obj.cmd_G91(&gcmd_trait),
        "cmd_G92" => rust_obj.cmd_G92(&gcmd_trait),
        "cmd_M114" => rust_obj.cmd_M114(&gcmd_trait),
        "cmd_M220" => rust_obj.cmd_M220(&gcmd_trait),
        "cmd_M221" => rust_obj.cmd_M221(&gcmd_trait),
        "cmd_SET_GCODE_OFFSET" => rust_obj.cmd_SET_GCODE_OFFSET(&gcmd_trait),
        "cmd_SAVE_GCODE_STATE" => rust_obj.cmd_SAVE_GCODE_STATE(&gcmd_trait),
        "cmd_RESTORE_GCODE_STATE" => rust_obj.cmd_RESTORE_GCODE_STATE(&gcmd_trait),
        "cmd_GET_POSITION" => rust_obj.cmd_GET_POSITION(&gcmd_trait),
        _ => panic!("unknown method {method}"),
    };
    let rust_raw = gcmd_concrete.raw_out.cloned();
    let rust_info = gcmd_concrete.info_out.cloned().into_iter().map(|(s, _)| s).collect::<Vec<_>>();
    // Convert python outputs to Vec<String> via render for comparison
    let render_list = |v: Value| -> Vec<String> {
        match v {
            Value::List(items) => items.iter().map(|x| match x { Value::Str(s) => s.clone(), other => other.render() }).collect(),
            _ => Vec::new(),
        }
    };
    let py_raw_v = render_list(py_raw.clone());
    let py_info_v = render_list(py_info.clone());
    (expected, result, py_raw_v, rust_raw, py_info_v, rust_info)
}

#[test]
fn registrations_match_python() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], Vec::new());
    let expected = py.eval("g.regs").expect("py regs");
    let regs = pair.gcode_rust.regs.cloned();
    let actual = Value::List(regs.iter().map(|(c, w, d)| {
        Value::List(vec![
            Value::Str(c.clone()),
            Value::Bool(*w),
            match d { None => Value::Null, Some(s) => Value::Str(s.clone()) },
        ])
    }).collect());
    check("registrations_match_python", Ok(expected), actual);
}

#[test]
fn initial_status_after_ready() {
    let Some(mut py) = oracle() else { return };
    let mut pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], Vec::new());
    fire_ready(&mut py, &pair);
    check_status_both("initial_status_after_ready", &mut py, &pair.rust);
    check_moves("initial_status_after_ready moves", &mut py, &pair.toolhead_rust);
    pair.rust = pair.rust.clone();
}

#[test]
fn g1_absolute_relative_and_speed() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let cases: Vec<(&str, &str, Vec<(&str, &str)>, &str)> = vec![
        ("G1", "G1 X10 Y20 Z5", vec![("X", "10"), ("Y", "20"), ("Z", "5")], "cmd_G1"),
        ("G1", "G1 X10 Y20 Z5 F120", vec![("X", "10"), ("Y", "20"), ("Z", "5"), ("F", "120")], "cmd_G1"),
        ("G91", "G91", vec![], "cmd_G91"),
        ("G1", "G1 X1 Y1 E2", vec![("X", "1"), ("Y", "1"), ("E", "2")], "cmd_G1"),
        ("G90", "G90", vec![], "cmd_G90"),
        ("G1", "G1 E5", vec![("E", "5")], "cmd_G1"),
        ("M83", "M83", vec![], "cmd_M83"),
        ("G1", "G1 E1.5", vec![("E", "1.5")], "cmd_G1"),
        ("M82", "M82", vec![], "cmd_M82"),
        ("G1", "G1 E-1", vec![("E", "-1")], "cmd_G1"),
        ("G1", "G1 F0", vec![("F", "0")], "cmd_G1"),
        ("G1", "G1 F-5", vec![("F", "-5")], "cmd_G1"),
    ];
    for (cmd, line, params, meth) in cases {
        let label = format!("g1 {} {}", cmd, line);
        let (expected, actual, py_raw, rust_raw, py_info, rust_info) = apply_cmd(&mut py, &pair.rust, cmd, line, params, meth);
        check_cmd(&label, expected, actual);
        assert_eq!(py_raw, rust_raw, "{label} raw");
        assert_eq!(py_info, rust_info, "{label} info");
        check_status_both(&label, &mut py, &pair.rust);
        check_moves(&label, &mut py, &pair.toolhead_rust);
    }
}

#[test]
fn g1_parse_errors_partial_update() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (expected, actual, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 Xabc Y5", vec![("X", "abc"), ("Y", "5")], "cmd_G1");
    check_cmd("g1 parse error", expected, actual);
    check_status_both("g1 parse error status", &mut py, &pair.rust);
    check_moves("g1 parse error moves", &mut py, &pair.toolhead_rust);
    // Empty F
    let (expected, actual, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X5 F", vec![("X", "5"), ("F", "")], "cmd_G1");
    check_cmd("g1 empty F", expected, actual);
    check_status_both("g1 empty F status", &mut py, &pair.rust);
    check_moves("g1 empty F moves", &mut py, &pair.toolhead_rust);
}

#[test]
fn g0_alias() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (expected, actual, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G0", "G0 X7", vec![("X", "7")], "cmd_G1");
    check_cmd("g0 alias", expected, actual);
    check_status_both("g0 alias status", &mut py, &pair.rust);
    check_moves("g0 alias moves", &mut py, &pair.toolhead_rust);
}

#[test]
fn g20_g21() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (expected, actual, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G20", "G20", vec![], "cmd_G20");
    check_cmd("g20", expected, actual);
    let (expected, actual, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G21", "G21", vec![], "cmd_G21");
    check_cmd("g21", expected, actual);
    check_status_both("g20/g21 status", &mut py, &pair.rust);
}

#[test]
fn g92_cases() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let setup = [
        ("G1", "G1 X10 Y10 Z10 E10", vec![("X", "10"), ("Y", "10"), ("Z", "10"), ("E", "10")], "cmd_G1"),
    ];
    for (c, l, p, m) in setup {
        let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, c, l, p, m);
        check_cmd("g92 setup", e, a);
    }
    let cases: Vec<(&str, &str, Vec<(&str, &str)>)> = vec![
        ("G92", "G92 X5", vec![("X", "5")]),
        ("G92", "G92 E3", vec![("E", "3")]),
        ("G92", "G92", vec![]),
        ("G92", "G92 X1 Y2 Z3 E4", vec![("X", "1"), ("Y", "2"), ("Z", "3"), ("E", "4")]),
        ("G92", "G92 Xabc", vec![("X", "abc")]),
    ];
    for (c, l, p) in cases {
        let label = format!("g92 {l}");
        let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, c, l, p, "cmd_G92");
        check_cmd(&label, e, a);
        check_status_both(&label, &mut py, &pair.rust);
        check_moves(&label, &mut py, &pair.toolhead_rust);
    }
}

#[test]
fn m114_output() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X1.5 Y2.25 Z-3 E4.5", vec![("X", "1.5"), ("Y", "2.25"), ("Z", "-3"), ("E", "4.5")], "cmd_G1");
    check_cmd("m114 setup", e, a);
    let (e, a, py_raw, rust_raw, _, _) = apply_cmd(&mut py, &pair.rust, "M114", "M114", vec![], "cmd_M114");
    check_cmd("m114", e, a);
    assert_eq!(py_raw, rust_raw, "m114 raw");
    check_status_both("m114 status", &mut py, &pair.rust);
}

#[test]
fn m220_m221() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X10 E10 F100", vec![("X", "10"), ("E", "10"), ("F", "100")], "cmd_G1");
    check_cmd("m220 setup", e, a);
    let cases: Vec<(&str, &str, Vec<(&str, &str)>, &str)> = vec![
        ("M220", "M220 S150", vec![("S", "150")], "cmd_M220"),
        ("M220", "M220", vec![], "cmd_M220"),
        ("M220", "M220 S0", vec![("S", "0")], "cmd_M220"),
        ("M220", "M220 S-10", vec![("S", "-10")], "cmd_M220"),
        ("M220", "M220 Sabc", vec![("S", "abc")], "cmd_M220"),
        ("M221", "M221 S120", vec![("S", "120")], "cmd_M221"),
        ("M221", "M221", vec![], "cmd_M221"),
        ("M221", "M221 S0", vec![("S", "0")], "cmd_M221"),
        ("M221", "M221 S-5", vec![("S", "-5")], "cmd_M221"),
    ];
    for (c, l, p, m) in cases {
        let label = format!("{c} {l}");
        let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, c, l, p, m);
        check_cmd(&label, e, a);
        check_status_both(&label, &mut py, &pair.rust);
        check_moves(&label, &mut py, &pair.toolhead_rust);
    }
}

#[test]
fn set_gcode_offset_cases() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X10 Y10", vec![("X", "10"), ("Y", "10")], "cmd_G1");
    check_cmd("offset setup", e, a);
    let cases: Vec<Vec<(&str, &str)>> = vec![
        vec![("X", "5")],
        vec![("X_ADJUST", "2")],
        vec![("X", "5"), ("Y", "6"), ("Z", "7"), ("E", "8")],
        vec![("X", "5"), ("MOVE", "1")],
        vec![("X", "1"), ("MOVE", "1"), ("MOVE_SPEED", "30")],
        vec![("X", "abc")],
        vec![],
        vec![("MOVE", "1")],
    ];
    for (idx, p) in cases.iter().enumerate() {
        let label = format!("set_offset case {idx}");
        let l = format!("SET_GCODE_OFFSET {}", p.iter().map(|(k, v)| format!("{k}{v}")).collect::<Vec<_>>().join(" "));
        let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "SET_GCODE_OFFSET", &l, p.clone(), "cmd_SET_GCODE_OFFSET");
        check_cmd(&label, e, a);
        check_status_both(&label, &mut py, &pair.rust);
        check_moves(&label, &mut py, &pair.toolhead_rust);
    }
}

#[test]
fn save_restore_states() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X10 Y10 E10", vec![("X", "10"), ("Y", "10"), ("E", "10")], "cmd_G1");
    check_cmd("save setup", e, a);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "SAVE_GCODE_STATE", "SAVE_GCODE_STATE", vec![], "cmd_SAVE_GCODE_STATE");
    check_cmd("save default", e, a);
    // Move away and change modes
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X20 E20", vec![("X", "20"), ("E", "20")], "cmd_G1");
    check_cmd("save move", e, a);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "M83", "M83", vec![], "cmd_M83");
    check_cmd("save m83", e, a);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G91", "G91", vec![], "cmd_G91");
    check_cmd("save g91", e, a);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "SAVE_GCODE_STATE", "SAVE_GCODE_STATE NAME=foo", vec![("NAME", "foo")], "cmd_SAVE_GCODE_STATE");
    check_cmd("save foo", e, a);
    check_status_both("after saves", &mut py, &pair.rust);
    // Restore default
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "RESTORE_GCODE_STATE", "RESTORE_GCODE_STATE", vec![], "cmd_RESTORE_GCODE_STATE");
    check_cmd("restore default", e, a);
    check_status_both("after restore default", &mut py, &pair.rust);
    check_moves("after restore default moves", &mut py, &pair.toolhead_rust);
    // Restore foo with move
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X0 Y0", vec![("X", "0"), ("Y", "0")], "cmd_G1");
    check_cmd("restore move prep", e, a);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "RESTORE_GCODE_STATE", "RESTORE_GCODE_STATE NAME=foo MOVE=1 MOVE_SPEED=50", vec![("NAME", "foo"), ("MOVE", "1"), ("MOVE_SPEED", "50")], "cmd_RESTORE_GCODE_STATE");
    check_cmd("restore foo move", e, a);
    check_status_both("after restore foo", &mut py, &pair.rust);
    check_moves("after restore foo moves", &mut py, &pair.toolhead_rust);
    // Unknown
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "RESTORE_GCODE_STATE", "RESTORE_GCODE_STATE NAME=nope", vec![("NAME", "nope")], "cmd_RESTORE_GCODE_STATE");
    check_cmd("restore unknown", e, a);
    check_status_both("after unknown", &mut py, &pair.rust);
}

#[test]
fn get_position_cases() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X5 Y6 Z7 E8", vec![("X", "5"), ("Y", "6"), ("Z", "7"), ("E", "8")], "cmd_G1");
    check_cmd("getpos setup", e, a);
    let (e, a, py_raw, rust_raw, py_info, rust_info) = apply_cmd(&mut py, &pair.rust, "GET_POSITION", "GET_POSITION", vec![], "cmd_GET_POSITION");
    check_cmd("getpos", e, a);
    assert_eq!(py_raw, rust_raw, "getpos raw");
    assert_eq!(py_info, rust_info, "getpos info");
}

#[test]
fn get_position_no_toolhead() {
    let Some(mut py) = oracle() else { return };
    py.exec("p2 = FakePrinter(); g2 = FakeGCode(); p2.add_object('gcode', g2); c2 = FakeConfig(p2); gm2 = gcode_move.load_config(c2)").expect("py no toolhead");
    let printer_rust = FakePrinter::new();
    let printer: Printer = printer_rust.clone();
    let gcode_rust = FakeGCode::new();
    printer_rust.add_object("gcode", gcode_rust.clone() as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let rust = GCodeMove::new(&config).unwrap();
    // Python: call GET_POSITION with no toolhead
    let expected = py.eval("gm2.cmd_GET_POSITION(make_gcmd(g2, 'GET_POSITION', 'GET_POSITION', {}))");
    let gcmd = make_gcmd("GET_POSITION", "GET_POSITION", vec![]);
    let actual = rust.cmd_GET_POSITION(&gcmd);
    check_cmd("getpos no toolhead", expected, actual);
}

#[test]
fn set_move_transform_cases() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], Vec::new());
    fire_ready(&mut py, &pair);
    // First transform: use toolhead as transform (allowed)
    let toolhead_handle: klippy_api::toolhead::ToolHead = pair.toolhead_rust.clone();
    let old1 = pair.rust.set_move_transform(Some(MoveTransform::ToolHead(toolhead_handle.clone())), false).expect("first transform");
    // Python equivalent: set transform to toolhead object
    py.exec("gm.set_move_transform(th)").expect("py set transform");
    // Second without force must raise Config error on both sides
    let py_again = py.eval("gm.set_move_transform(th)");
    let second = pair.rust.set_move_transform(Some(MoveTransform::ToolHead(toolhead_handle.clone())), false);
    match (py_again, second) {
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "transform class");
            assert_eq!(err.message(), exc.message.as_str(), "transform message");
        }
        _ => panic!("expected transform error on both sides"),
    }
    // With force succeeds on both
    let py_force = py.eval("gm.set_move_transform(th, force=True)");
    assert!(py_force.is_ok(), "py force should succeed: {py_force:?}");
    let force = pair.rust.set_move_transform(Some(MoveTransform::ToolHead(toolhead_handle.clone())), true);
    assert!(force.is_ok(), "rust force should succeed");
    assert!(old1.is_some(), "old transform should be toolhead");
    check_status_both("after transforms", &mut py, &pair.rust);
}

/// `set_move_transform(None, force=True)` stores `None`, then raises
/// `AttributeError` binding `None.move`; moves keep going through the
/// previous `move_with_transform`, and a later non-forced set succeeds.
#[test]
fn set_move_transform_none_cases() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], Vec::new());
    fire_ready(&mut py, &pair);
    let toolhead_handle: klippy_api::toolhead::ToolHead = pair.toolhead_rust.clone();
    py.exec("gm.set_move_transform(th)").expect("py set transform");
    pair.rust
        .set_move_transform(Some(MoveTransform::ToolHead(toolhead_handle.clone())), false)
        .expect("rust set transform");
    let expected = py.eval("gm.set_move_transform(None, force=True)").expect_err("py None raises");
    let Err(actual) = pair.rust.set_move_transform(None, true) else { panic!("rust None raises") };
    assert_eq!(actual.class_name(), expected.class_name.as_str(), "None transform class");
    assert_eq!(actual.message(), expected.message.as_str(), "None transform message");
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X5", vec![("X", "5")], "cmd_G1");
    check_cmd("move after None transform", e, a);
    check_moves("moves after None transform", &mut py, &pair.toolhead_rust);
    let py_again = py.eval("gm.set_move_transform(th) is None");
    let again = pair.rust
        .set_move_transform(Some(MoveTransform::ToolHead(toolhead_handle)), false)
        .expect("rust set after None");
    // `old_transform` falls back to the toolhead object when the stored one is `None`.
    assert_eq!(py_again.expect("py set after None"), Value::Bool(false), "py old transform");
    assert!(again.is_some(), "rust old transform");
    check_status_both("after None transform", &mut py, &pair.rust);
}

#[test]
fn update_extra_axes_cases() {
    let Some(mut py) = oracle() else { return };
    // Extra axes: I (valid), F/N (reserved), lowercase (invalid), duplicate X, multi-char, None
    let extras: Vec<Option<String>> = vec![
        Some("I".to_string()),
        Some("F".to_string()),
        Some("a".to_string()),
        Some("X".to_string()),
        Some("IJ".to_string()),
        None,
        Some("J".to_string()),
    ];
    let pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], extras.clone());
    fire_ready(&mut py, &pair);
    // Trigger update: send toolhead:update_extra_axes
    py.exec("p.send_event('toolhead:update_extra_axes')").expect("py update");
    pair.printer.send_event("toolhead:update_extra_axes", &[]).unwrap();
    check_status_both("update_extra_axes status", &mut py, &pair.rust);
    // Move via extra axis I should now work
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 I2.5", vec![("I", "2.5")], "cmd_G1");
    check_cmd("move I", e, a);
    check_status_both("after I move", &mut py, &pair.rust);
    check_moves("after I move moves", &mut py, &pair.toolhead_rust);
}

#[test]
fn reset_position_after_command_error_and_activate() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X10 E10", vec![("X", "10"), ("E", "10")], "cmd_G1");
    check_cmd("reset setup", e, a);
    // Change extrude factor then activate extruder resets it
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "M221", "M221 S150", vec![("S", "150")], "cmd_M221");
    check_cmd("reset m221", e, a);
    // Move toolhead externally
    py.exec("th.position = [1.0, 1.0, 1.0, 7.0]").expect("py move th");
    *pair.toolhead_rust.position.borrow_mut() = vec![1.0, 1.0, 1.0, 7.0];
    py.exec("p.send_event('extruder:activate_extruder')").expect("py activate");
    pair.printer.send_event("extruder:activate_extruder", &[]).unwrap();
    check_status_both("after activate", &mut py, &pair.rust);
    // gcode:command_error resets last position to toolhead position
    py.exec("th.position = [2.0, 2.0, 2.0, 8.0]").expect("py move th2");
    *pair.toolhead_rust.position.borrow_mut() = vec![2.0, 2.0, 2.0, 8.0];
    py.exec("p.send_event('gcode:command_error')").expect("py cmd error event");
    pair.printer.send_event("gcode:command_error", &[]).unwrap();
    check_status_both("after command_error", &mut py, &pair.rust);
    // shutdown blocks reset
    py.exec("p.send_event('klippy:shutdown')").expect("py shutdown");
    pair.printer.send_event("klippy:shutdown", &[]).unwrap();
    py.exec("th.position = [9.0, 9.0, 9.0, 9.0]").expect("py move th3");
    *pair.toolhead_rust.position.borrow_mut() = vec![9.0, 9.0, 9.0, 9.0];
    py.exec("p.send_event('toolhead:set_position')").expect("py set pos");
    pair.printer.send_event("toolhead:set_position", &[]).unwrap();
    check_status_both("after shutdown set_position", &mut py, &pair.rust);
}

#[test]
fn home_rails_end_updates_base() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    fire_ready(&mut py, &pair);
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "G1", "G1 X10 Y10 Z10", vec![("X", "10"), ("Y", "10"), ("Z", "10")], "cmd_G1");
    check_cmd("homing setup", e, a);
    // homing_position is [0,0,0,0] initially; set via SET_GCODE_OFFSET to known values
    let (e, a, _, _, _, _) = apply_cmd(&mut py, &pair.rust, "SET_GCODE_OFFSET", "SET_GCODE_OFFSET X1 Y2 Z3", vec![("X", "1"), ("Y", "2"), ("Z", "3")], "cmd_SET_GCODE_OFFSET");
    check_cmd("homing offset", e, a);
    check_status_both("after homing offset", &mut py, &pair.rust);
    // Fire home_rails_end with axes [0,1] : base X/Y copy homing X/Y
    // Python: need a homing_state object with get_axes
    py.exec("class _H: \n    def get_axes(self): return [0, 1]\nhom = _H()").expect("py hom");
    py.exec("p.send_event('homing:home_rails_end', hom, [])").expect("py home rails");
    // Rust: need HomingStateApi handle
    struct FakeHoming { axes: Vec<i64> }
    impl klippy_api::toolhead::HomingStateApi for FakeHoming {
        fn set_axes(&self, _a: Vec<i64>) {}
        fn get_axes(&self) -> Vec<i64> { self.axes.clone() }
        fn get_trigger_position(&self, _s: &str) -> KResult<i64> { Ok(0) }
        fn set_stepper_adjustment(&self, _s: &str, _a: f64) {}
        fn set_homed_position(&self, _p: Vec<Option<f64>>) -> KResult<()> { Ok(()) }
        fn home_rails(&self, _r: Vec<klippy_api::stepper::Rail>, _f: Vec<Option<f64>>, _m: Vec<Option<f64>>) -> KResult<()> { Ok(()) }
    }
    let handle: klippy_api::toolhead::HomingStateHandle = Arc::new(FakeHoming { axes: vec![0, 1] });
    let arg = EventArg::handle(handle);
    pair.printer.send_event("homing:home_rails_end", &[arg.clone(), EventArg::List(Vec::new())]).unwrap();
    check_status_both("after home rails end", &mut py, &pair.rust);
}

#[test]
fn module_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory.load_config.expect("load_config");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let printer_rust = FakePrinter::new();
    let printer: Printer = printer_rust.clone();
    let gcode_rust = FakeGCode::new();
    printer_rust.add_object("gcode", gcode_rust.clone() as ObjectRef).unwrap();
    let toolhead_rust = FakeToolHead::new(vec![0.0, 0.0, 0.0, 0.0], Vec::new());
    printer_rust.add_object("toolhead", toolhead_rust.clone() as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let obj: ObjectRef = load_config(&config).expect("factory").expect("obj");
    let rust: Arc<GCodeMove> = klippy_api::downcast_arc(&obj).expect("downcast");
    printer_rust.add_object("gcode_move", obj.clone()).unwrap();
    py.exec("p = FakePrinter(); g = FakeGCode(); p.add_object('gcode', g); th = FakeToolHead(position=[0.0, 0.0, 0.0, 0.0]); p.add_object('toolhead', th); c = FakeConfig(p); gm = gcode_move.load_config(c); p.add_object('gcode_move', gm); p.send_event('klippy:ready')").expect("py factory");
    printer_rust.send_event("klippy:ready", &[]).unwrap();
    check_status_both("factory status", &mut py, &rust);
    let _ = py.eval("gm.cmd_G1(make_gcmd(g, 'G1', 'G1 X3', {'X': '3'}))");
    let concrete: Arc<FakeGcmd> = {
        let mut m = IndexMap::new();
        m.insert("X".to_string(), "3".to_string());
        Arc::new(FakeGcmd { command: "G1".to_string(), commandline: "G1 X3".to_string(), params: m, raw_out: klippy_api::GCell::new(Vec::new()), info_out: klippy_api::GCell::new(Vec::new()) })
    };
    let trait_arc: GCmd = concrete.clone();
    let actual = rust.cmd_G1(&trait_arc);
    assert!(actual.is_ok(), "factory move should succeed");
    check_moves("factory moves", &mut py, &toolhead_rust);
}


#[test]
fn reset_position_propagates_transform_errors() {
    struct FailingPosition;
    impl klippy_api::gcode::MoveTransformApi for FailingPosition {
        fn r#move(&self, _: Vec<f64>, _: f64) -> KResult<()> {
            panic!("the position-reset test never moves");
        }
        fn get_position(&self) -> KResult<Vec<f64>> {
            Err(klippy_api::KlippyError::Internal {
                class_name: "ValueError".into(),
                message: "transform position failed".into(),
            })
        }
    }
    let mut py = oracle().expect("the Python oracle is required");
    let pair = fresh_pair(&mut py, vec![1.0, 2.0, 3.0, 4.0], Vec::new());
    fire_ready(&mut py, &pair);
    py.exec(r#"
class FailingPosition:
    def move(self, pos, speed):
        raise AssertionError('the position-reset test never moves')
    def get_position(self):
        raise ValueError('transform position failed')
gm.set_move_transform(FailingPosition())
"#).unwrap();
    pair.rust.set_move_transform(Some(MoveTransform::Custom(Arc::new(FailingPosition))), false).unwrap();
    for event in ["toolhead:set_position", "toolhead:manual_move", "gcode:command_error"] {
        let expected = py.eval(&format!("p.send_event({event:?})"));
        let actual = pair.printer.send_event(event, &[]).map(|_| ());
        check_cmd(event, expected, actual);
        check_status_both(event, &mut py, &pair.rust);
    }
}
