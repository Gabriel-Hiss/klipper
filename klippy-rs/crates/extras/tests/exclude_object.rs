//! Parity tests for `extras::exclude_object` (port of `klippy/extras/exclude_object.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `ExcludeObject` is built
//! on both sides with minimal Python fakes for the printer, gcode, gcode_move
//! and toolhead, then fed identical G-code commands and moves. Returned
//! values, raised errors (class and message), G-code responses, `get_status`
//! dicts and toolhead moves are compared exactly (floats by bits).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::exclude_object::ExcludeObject;
use extras::tuning_tower::TuningTowerApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GCmd, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, MoveTransform, MoveTransformApi, RegisterCommandOpts,
};
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{
    ExtraAxis, Kinematics, LookaheadCallback, MoveApi, PrinterExtruder, ToolHead, ToolHeadApi,
};
use klippy_api::stepper::StepperFactory;
use klippy_api::{EventArg, GCell};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.exclude_object as eo_mod

class CommandError(Exception):
    pass

SENTINEL = object()

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = {}
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=SENTINEL):
        if section in self.objects:
            return self.objects[section]
        if default is SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        for cb in self.handlers.get(event, []):
            cb(*params)
    def add_object(self, name, obj):
        self.objects[name] = obj

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.commands = {}
        self.responded = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.commands[cmd] = (func, desc)
    def respond_info(self, msg, log=True):
        self.responded.append(msg)
    def respond_raw(self, msg):
        self.responded.append(msg)

class FakeGCmd:
    error = CommandError
    def __init__(self, gcode, command, commandline, params):
        self._gcode = gcode
        self._command = command
        self._commandline = commandline
        self._params = params
    def get_command(self):
        return self._command
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return self._params
    def respond_info(self, msg, log=True):
        self._gcode.responded.append(msg)
    def respond_raw(self, msg):
        self._gcode.responded.append(msg)
    def get(self, name, default=SENTINEL, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is SENTINEL:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
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

class FakeExtruder:
    def __init__(self, name="extruder"):
        self._name = name
    def get_name(self):
        return self._name

class FakeToolhead:
    def __init__(self, pos=(0.0, 0.0, 0.0, 0.0), extruder_name="extruder"):
        self._pos = list(pos)
        self._extruder = FakeExtruder(extruder_name)
        self.moves = []
    def get_position(self):
        return list(self._pos)
    def move(self, newpos, speed):
        self.moves.append((list(newpos), speed))
        self._pos = list(newpos)
    def get_extruder(self):
        return self._extruder

class FakeGCodeMove:
    def __init__(self, toolhead):
        self.reset_count = 0
        self.transform_calls = []
        self._toolhead = toolhead
        self._current = None
    def set_move_transform(self, transform, force=False):
        old = self._current if self._current is not None else self._toolhead
        self._current = transform
        self.transform_calls.append(transform)
        return old
    def reset_last_position(self):
        self.reset_count += 1

class FakeTuningTower:
    def __init__(self, active=False):
        self._active = active
    def is_active(self):
        return self._active

def make_pair(tuning=None, pos=(0.0, 0.0, 0.0, 0.0), extruder_name="extruder"):
    p = FakePrinter()
    g = FakeGCode()
    th = FakeToolhead(pos, extruder_name)
    gm = FakeGCodeMove(th)
    p.objects['gcode'] = g
    p.objects['gcode_move'] = gm
    if tuning != "absent":
        p.objects['tuning_tower'] = FakeTuningTower(bool(tuning))
    p.objects['toolhead'] = th
    c = FakeConfig(p)
    eo = eo_mod.load_config(c)
    p.send_event('klippy:connect')
    return p, c, g, th, gm, eo
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone, Default)]
struct FakePrinter {
    inner: Arc<PrinterInner>,
}

#[derive(Default)]
struct PrinterInner {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<IndexMap<String, Vec<EventCallback>>>,
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) {
        self.inner.objects.borrow_mut().insert(name.to_string(), obj);
    }
    fn send(&self, event: &str) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let all: Vec<(String, Vec<EventCallback>)> = {
            let taken = self.inner.handlers.replace(IndexMap::new());
            taken.into_iter().collect()
        };
        let mut rest = IndexMap::new();
        for (ev, mut cbs) in all {
            if ev == event {
                for cb in cbs.iter_mut() {
                    results.push(cb(&[])?);
                }
                rest.entry(ev.clone()).or_insert_with(Vec::new).extend(cbs);
            } else {
                rest.insert(ev, cbs);
            }
        }
        *self.inner.handlers.borrow_mut() = rest;
        Ok(results)
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.inner.objects.borrow_mut();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.inner.objects.borrow().get(name).cloned() {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.inner
            .objects
            .borrow()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
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
        self.inner
            .handlers
            .borrow_mut()
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(
        &self,
        event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        self.send(event)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "exclude_object".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
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
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
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
        panic!("getsection is not used by exclude_object")
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

#[derive(Clone, Default)]
struct FakeGCode {
    inner: Arc<GCodeInner>,
}

#[derive(Default)]
struct GCodeInner {
    commands: GCell<IndexMap<String, Option<GCodeCommandHandler>>>,
    responded: GCell<Vec<String>>,
    descs: GCell<IndexMap<String, Option<String>>>,
}

impl FakeGCode {
    fn responded(&self) -> Vec<String> {
        self.inner.responded.cloned()
    }
    fn command_names(&self) -> Vec<String> {
        self.inner.commands.borrow().keys().cloned().collect()
    }
    fn descs(&self) -> IndexMap<String, Option<String>> {
        self.inner.descs.cloned()
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.inner
            .descs
            .borrow_mut()
            .insert(cmd.to_string(), opts.desc);
        let old = self
            .inner
            .commands
            .borrow_mut()
            .insert(cmd.to_string(), func);
        Ok(old.flatten())
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
    fn get_mutex(&self) -> ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            gcode: self.clone(),
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        })
    }
    fn respond_raw(&self, msg: &str) {
        self.inner.responded.borrow_mut().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.inner.responded.borrow_mut().push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

#[derive(Clone)]
struct FakeGCmd {
    gcode: FakeGCode,
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn missing(&self, name: &str) -> KlippyError {
        KlippyError::Command(format!(
            "Error on '{}': missing {}",
            self.commandline, name
        ))
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.missing(name)),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(raw) => match pycompat::text::int_from_string(raw, 10) {
                Ok(n) => Ok(Some(n.0.to_string().parse::<i64>().unwrap_or(0))),
                Err(_) => Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, raw
                ))),
            },
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(raw) => match pycompat::text::float_from_string(raw) {
                Ok(f) => Ok(Some(f)),
                Err(_) => Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, raw
                ))),
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
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.gcode.respond_info(msg, log);
    }
    fn respond_raw(&self, msg: &str) {
        self.gcode.respond_raw(msg);
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

#[derive(Clone)]
struct FakeExtruder {
    name: String,
}

impl klippy_api::toolhead::ExtraAxisApi for FakeExtruder {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        None
    }
    fn get_trapq(&self) -> Option<klippy_api::stepper::Trapq> {
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
    ) -> f64 {
        0.0
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl klippy_api::toolhead::PrinterExtruderApi for FakeExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::Command("Extruder not configured".to_string()))
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

#[derive(Clone)]
struct FakeToolhead {
    inner: Arc<ToolheadInner>,
}

struct ToolheadInner {
    position: GCell<Vec<f64>>,
    moves: GCell<Vec<(Vec<f64>, f64)>>,
    extruder_name: GCell<String>,
}

impl FakeToolhead {
    fn new(position: Vec<f64>, extruder_name: &str) -> Self {
        FakeToolhead {
            inner: Arc::new(ToolheadInner {
                position: GCell::new(position),
                moves: GCell::new(Vec::new()),
                extruder_name: GCell::new(extruder_name.to_string()),
            }),
        }
    }
    fn moves(&self) -> Vec<(Vec<f64>, f64)> {
        self.inner.moves.cloned()
    }
    fn set_position(&self, pos: Vec<f64>) {
        *self.inner.position.borrow_mut() = pos;
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.inner.position.cloned()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        *self.inner.position.borrow_mut() = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics)
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
        newpos: Vec<f64>,
        speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        self.inner
            .moves
            .borrow_mut()
            .push((newpos.clone(), speed));
        *self.inner.position.borrow_mut() = newpos;
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.inner
            .moves
            .borrow_mut()
            .push((newpos.clone(), speed));
        *self.inner.position.borrow_mut() = newpos;
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq is not used by exclude_object")
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
        Arc::new(FakeExtruder {
            name: self.inner.extruder_name.cloned(),
        })
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
        panic!("stepper_factory is not used by exclude_object")
    }
}

struct FakeKinematics;

impl klippy_api::toolhead::KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(Vec::new())
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
struct FakeGCodeMove {
    inner: Arc<GCodeMoveInner>,
}

struct GCodeMoveInner {
    current: GCell<Option<MoveTransform>>,
    resets: GCell<u64>,
    toolhead: ToolHead,
}

impl FakeGCodeMove {
    fn new(toolhead: FakeToolhead) -> Self {
        FakeGCodeMove {
            inner: Arc::new(GCodeMoveInner {
                current: GCell::new(None),
                resets: GCell::new(0),
                toolhead: Arc::new(toolhead),
            }),
        }
    }
    fn resets(&self) -> u64 {
        self.inner.resets.get()
    }
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set_move_transform(
        &self,
        transform: MoveTransform,
        _force: bool,
    ) -> KResult<Option<MoveTransform>> {
        match self.inner.current.replace(Some(transform)) {
            Some(old) => Ok(Some(old)),
            None => Ok(Some(MoveTransform::ToolHead(self.inner.toolhead.clone()))),
        }
    }
    fn reset_last_position(&self) -> KResult<()> {
        let n = self.inner.resets.get() + 1;
        self.inner.resets.set(n);
        Ok(())
    }
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> {
        Ok(())
    }
}

struct FakeTuningTower {
    active: bool,
}

impl TuningTowerApi for FakeTuningTower {
    fn is_active(&self) -> bool {
        self.active
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi, dyn klippy_api::toolhead::HomingTargetApi);
klippy_api::printer_object!(FakeTuningTower; dyn TuningTowerApi);

/// One paired scenario: Rust object plus the shared fake handles.
struct Pair {
    obj: Arc<ExcludeObject>,
    printer: FakePrinter,
    gcode: FakeGCode,
    toolhead: FakeToolhead,
    gcode_move: FakeGCodeMove,
}

fn make_pair(tuning: Option<bool>) -> Pair {
    let printer = FakePrinter::default();
    let toolhead = FakeToolhead::new(vec![0.0, 0.0, 0.0, 0.0], "extruder");
    let gcode = FakeGCode::default();
    let gcode_move = FakeGCodeMove::new(toolhead.clone());
    printer.add("gcode", Arc::new(gcode.clone()) as ObjectRef);
    printer.add("gcode_move", Arc::new(gcode_move.clone()) as ObjectRef);
    printer.add("toolhead", Arc::new(toolhead.clone()) as ObjectRef);
    let active = tuning.unwrap_or(false);
    printer.add(
        "tuning_tower",
        Arc::new(FakeTuningTower { active }) as ObjectRef,
    );
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig {
        printer: printer_arc.clone(),
    });
    let obj = ExcludeObject::create(&config).expect("create exclude_object");
    printer.send("klippy:connect").expect("connect");
    Pair {
        obj,
        printer,
        gcode,
        toolhead,
        gcode_move,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("exclude_object fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping exclude_object oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "exclude_object".to_string(),
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
        Ok(_) => panic!("{label}: rust returned Ok but call must fail"),
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

fn py_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let digits = n.0.to_string();
            match digits.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(digits),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(py_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(py_to_value).collect()),
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

fn responded_value(gcode: &FakeGCode) -> Value {
    Value::List(
        gcode
            .responded()
            .iter()
            .map(|s| Value::str(s))
            .collect(),
    )
}

fn moves_value(th: &FakeToolhead) -> Value {
    Value::List(
        th.moves()
            .iter()
            .map(|(pos, speed)| {
                Value::Tuple(vec![
                    Value::List(pos.iter().map(|f| Value::float(*f)).collect()),
                    Value::float(*speed),
                ])
            })
            .collect(),
    )
}

fn status_value(obj: &ExcludeObject) -> Value {
    py_to_value(&PrinterObject::get_status(obj, 0.0).expect("get_status"))
}

/// `CMD K=V K2=V2` commandline shared by both sides.
fn commandline(cmd: &str, params: &[(&str, &str)]) -> String {
    let mut line = cmd.to_string();
    for (k, v) in params {
        line.push(' ');
        line.push_str(k);
        line.push('=');
        line.push_str(v);
    }
    line
}

fn params_expr(params: &[(&str, &str)]) -> String {
    let inner: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", inner.join(", "))
}

/// Runs a G-code command on the Python side.
fn run_py(
    py: &mut pyoracle::Oracle,
    cmd: &str,
    params: &[(&str, &str)],
) -> pyoracle::OracleResult {
    let line = commandline(cmd, params);
    let expr = format!(
        "g.commands['{cmd}'][0](FakeGCmd(g, '{cmd}', '{line}', {params}))",
        cmd = cmd,
        line = line,
        params = params_expr(params)
    );
    py.eval(&expr)
}

/// Runs a G-code command on the Rust side.
fn run_rs(pair: &Pair, cmd: &str, params: &[(&str, &str)]) -> KResult<()> {
    let line = commandline(cmd, params);
    let mut map: IndexMap<String, String> = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    let gcmd: GCmd =
        GCodeApi::create_gcode_command(&pair.gcode, cmd, &line, &map);
    let mut handler = pair
        .gcode
        .inner
        .commands
        .borrow_mut()
        .swap_remove(cmd)
        .expect("command registered")
        .expect("command has a handler");
    let result = handler(&gcmd);
    pair.gcode
        .inner
        .commands
        .borrow_mut()
        .insert(cmd.to_string(), Some(handler));
    result
}

/// Runs one command on both sides and compares results, responses and status.
fn run_both(
    py: &mut pyoracle::Oracle,
    pair: &Pair,
    cmd: &str,
    params: &[(&str, &str)],
    label: &str,
) {
    let expected = run_py(py, cmd, params);
    let actual = run_rs(pair, cmd, params);
    match expected {
        Ok(_) => match actual {
            Ok(()) => {}
            Err(e) => panic!(
                "{label}: rust raised {}: {}",
                e.class_name(),
                e.message()
            ),
        },
        Err(e) => match actual {
            Ok(()) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
            Err(rust_e) => {
                assert_eq!(rust_e.class_name(), e.class_name.as_str(), "{label}: class");
                assert_eq!(rust_e.message(), e.message.as_str(), "{label}: message");
            }
        },
    }
    let expected_resp = py.eval("g.responded").expect("py responded");
    check(
        &format!("{label} responded"),
        Ok(expected_resp),
        responded_value(&pair.gcode),
    );
    let expected_status = py.eval("eo.get_status()").expect("py status");
    check(
        &format!("{label} status"),
        Ok(expected_status),
        status_value(&pair.obj),
    );
}

/// Runs a move on both sides and compares results and toolhead moves.
fn move_both(
    py: &mut pyoracle::Oracle,
    pair: &Pair,
    pos: &[f64],
    speed: f64,
    label: &str,
) {
    let pos_expr: Vec<String> = pos.iter().map(|f| format!("{f:?}")).collect();
    let expr = format!("eo.move([{pos}], {speed:?})", pos = pos_expr.join(", "));
    let expected = py.eval(&expr);
    let actual = pair.obj.r#move(pos.to_vec(), speed);
    match expected {
        Ok(_) => match actual {
            Ok(()) => {}
            Err(e) => panic!(
                "{label}: rust raised {}: {}",
                e.class_name(),
                e.message()
            ),
        },
        Err(e) => match actual {
            Ok(()) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
            Err(rust_e) => {
                assert_eq!(rust_e.class_name(), e.class_name.as_str(), "{label}: class");
                assert_eq!(rust_e.message(), e.message.as_str(), "{label}: message");
            }
        },
    }
    let expected_moves = py.eval("th.moves").expect("py moves");
    check(&format!("{label} moves"), Ok(expected_moves), moves_value(&pair.toolhead));
}

// ---------------------------------------------------------------------------
// Tests.


#[test]
fn start_adds_object_and_sets_current() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "outer")],
        "start outer",
    );
}

#[test]
fn start_upcases_and_dedupes_objects() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "B")],
        "start B",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "a")],
        "start a",
    );
    // Duplicate start (different case): definition is not duplicated.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "A")],
        "start A again",
    );
}

#[test]
fn start_missing_name_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(&mut py, &pair, "EXCLUDE_OBJECT_START", &[], "start missing");
}

#[test]
fn end_paths() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    // END with no current and no transform: silent.
    run_both(&mut py, &pair, "EXCLUDE_OBJECT_END", &[], "end bare");
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "obj1")],
        "start obj1",
    );
    // END with matching name: silent.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_END",
        &[("NAME", "obj1")],
        "end matching",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "obj2")],
        "start obj2",
    );
    // END with mismatching name: message, current cleared anyway.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_END",
        &[("NAME", "other")],
        "end mismatch",
    );
    // END with name but no current and no transform: mismatch message with None.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_END",
        &[("NAME", "other")],
        "end name no current",
    );
}

#[test]
fn end_without_object_but_transform_warns() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "obj1")],
        "start obj1",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "obj1")],
        "exclude obj1",
    );
    run_both(&mut py, &pair, "EXCLUDE_OBJECT_END", &[], "end obj1");
    // No current object now, but the transform is installed: END warns.
    run_both(&mut py, &pair, "EXCLUDE_OBJECT_END", &[], "end again warns");
}

#[test]
fn exclude_and_list() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(&mut py, &pair, "EXCLUDE_OBJECT", &[], "list empty");
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "foo")],
        "exclude foo",
    );
    // Duplicate exclusion: no second message.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "FOO")],
        "exclude FOO again",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "bar")],
        "exclude bar",
    );
    run_both(&mut py, &pair, "EXCLUDE_OBJECT", &[], "list two");
}

#[test]
fn exclude_current_paths() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    // CURRENT without a current object: CommandError.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("CURRENT", "1")],
        "current without object",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "cur")],
        "start cur",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("CURRENT", "1")],
        "current excludes cur",
    );
}

#[test]
fn exclude_reset_paths() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "a")],
        "exclude a",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "b")],
        "exclude b",
    );
    // RESET=1 NAME=A unexcludes only a.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("RESET", "1"), ("NAME", "a")],
        "reset a",
    );
    // RESET=1 alone clears the rest, without a message.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("RESET", "1")],
        "reset all",
    );
    // RESET with an unknown name still responds.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("RESET", "1"), ("NAME", "c")],
        "reset unknown",
    );
}

#[test]
fn define_and_list_objects() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("NAME", "beta")],
        "define beta",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("NAME", "alpha"), ("CENTER", "10.5,20.5")],
        "define alpha with center",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("NAME", "gamma"), ("POLYGON", "[[0,0],[1,0],[1,1]]"), ("LINKED", "1")],
        "define gamma with polygon",
    );
    run_both(&mut py, &pair, "EXCLUDE_OBJECT_DEFINE", &[], "list plain");
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("JSON", "1")],
        "list JSON",
    );
    // JSON= present-but-empty still selects JSON (`is not None`).
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("JSON", "")],
        "list JSON empty",
    );
}

#[test]
fn define_bad_center_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("NAME", "bad"), ("CENTER", "abc")],
        "define bad center",
    );
}

#[test]
fn define_bad_polygon_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_DEFINE",
        &[("NAME", "bad"), ("POLYGON", "oops")],
        "define bad polygon",
    );
}

#[test]
fn commands_registered_with_help() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    let expected = py.eval("sorted(g.commands.keys())").expect("py commands");
    let actual = Value::List(
        {
            let mut names = pair.gcode.command_names();
            names.sort();
            names.iter().map(|s| Value::str(s)).collect::<Vec<_>>()
        },
    );
    check("registered commands", Ok(expected), actual);
    let expected_descs: Vec<(String, String)> = py.eval("sorted((k, v[1]) for k, v in g.commands.items())")
        .map(|v| match v {
            Value::List(items) => items
                .into_iter()
                .map(|i| match i {
                    Value::Tuple(p) => match (&p[0], &p[1]) {
                        (Value::Str(k), Value::Str(d)) => (k.clone(), d.clone()),
                        _ => panic!("unexpected help entry"),
                    },
                    _ => panic!("unexpected help entry"),
                })
                .collect(),
            _ => panic!("unexpected command list"),
        })
        .unwrap_or_else(|_| Vec::new());
    let mut descs: Vec<(String, String)> = pair
        .gcode
        .descs()
        .into_iter()
        .map(|(k, v)| (k, v.unwrap_or_default()))
        .collect();
    descs.sort();
    assert_eq!(descs, expected_descs, "command help texts");
}

#[test]
fn moves_inside_excluded_object_are_ignored() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "first")],
        "start first",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "first")],
        "exclude first",
    );
    // Five E-moves burn off the initial extrusion pass.
    for i in 1..=5 {
        let e = i as f64;
        move_both(&mut py, &pair, &[10.0, 10.0, 0.2, e], 100.0, "burn move");
    }
    // Now the move enters the excluded region: swallowed, nothing forwarded.
    move_both(&mut py, &pair, &[40.0, 40.0, 0.2, 6.0], 100.0, "into excluded");
    move_both(&mut py, &pair, &[45.0, 40.0, 0.2, 7.0], 100.0, "inside excluded");
    // Leaving forwards again.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_END",
        &[("NAME", "first")],
        "end first",
    );
    move_both(&mut py, &pair, &[50.0, 50.0, 0.4, 8.0], 120.0, "leave region");
}

#[test]
fn normal_move_offsets_after_exclusion() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p, c, g, th, gm, eo = make_pair(None, pos=(10.0, 10.0, 0.2, 0.0))",
    )
    .expect("py make_pair");
    let pair = make_pair(None);
    pair.toolhead.set_position(vec![10.0, 10.0, 0.2, 0.0]);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "skip")],
        "start skip",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "skip")],
        "exclude skip",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "keep")],
        "start keep",
    );
    for i in 1..=5 {
        move_both(
            &mut py,
            &pair,
            &[10.0 + i as f64, 10.0, 0.2, i as f64],
            80.0,
            "burn move",
        );
    }
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "skip")],
        "start skip again",
    );
    move_both(&mut py, &pair, &[30.0, 30.0, 0.2, 6.0], 80.0, "into excluded");
    move_both(&mut py, &pair, &[32.0, 30.0, 0.2, 9.0], 80.0, "inside excluded");
    run_both(&mut py, &pair, "EXCLUDE_OBJECT_END", &[], "end skip");
    move_both(
        &mut py,
        &pair,
        &[15.0, 15.0, 0.4, 11.0],
        90.0,
        "back to printed",
    );
    move_both(&mut py, &pair, &[16.0, 15.0, 0.4, 12.0], 90.0, "continue printed");
}

#[test]
fn move_before_connect_errors() {
    let Some(mut py) = oracle() else { return };
    // Python: a fresh object has no `toolhead` attribute yet.
    py.exec(
        "p = FakePrinter(); g = FakeGCode(); th = FakeToolhead(); gm = FakeGCodeMove(th); \
         p.objects['gcode'] = g; p.objects['gcode_move'] = gm; p.objects['toolhead'] = th; \
         p.objects['tuning_tower'] = FakeTuningTower(False); \
         c = FakeConfig(p); eo = eo_mod.load_config(c)",
    )
    .expect("py fresh object");
    let printer = FakePrinter::default();
    let toolhead = FakeToolhead::new(vec![0.0, 0.0, 0.0, 0.0], "extruder");
    let gcode = FakeGCode::default();
    let gcode_move = FakeGCodeMove::new(toolhead.clone());
    printer.add("gcode", Arc::new(gcode.clone()) as ObjectRef);
    printer.add("gcode_move", Arc::new(gcode_move.clone()) as ObjectRef);
    printer.add("toolhead", Arc::new(toolhead.clone()) as ObjectRef);
    printer.add(
        "tuning_tower",
        Arc::new(FakeTuningTower { active: false }) as ObjectRef,
    );
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::new(printer.clone()),
    });
    let obj = ExcludeObject::create(&config).expect("create");
    let expected = py.eval("eo.move([0.0, 0.0, 0.0, 1.0], 1.0)");
    check_err(
        "move before connect",
        expected,
        obj.r#move(vec![0.0, 0.0, 0.0, 1.0], 1.0),
    );
}

#[test]
fn get_position_without_transform_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    let expected = py.eval("eo.get_position()");
    check_err(
        "get_position no transform",
        expected,
        ExcludeObject::get_position(&pair.obj),
    );
}

#[test]
fn tuning_tower_active_blocks_transform() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(True)")
        .expect("py make_pair");
    let pair = make_pair(Some(true));
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "obj")],
        "start obj",
    );
    // Exclusion is recorded and answered even though the transform is blocked.
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "obj")],
        "exclude obj",
    );
}

#[test]
fn missing_tuning_tower_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    // No `tuning_tower` object: `_register_transform` raises a config error.
    py.exec("p, c, g, th, gm, eo = make_pair(\"absent\")")
        .expect("py make_pair without tuning tower");
    // Make the rust side match: rebuild a pair without the tuning tower.
    let printer = FakePrinter::default();
    let toolhead = FakeToolhead::new(vec![0.0, 0.0, 0.0, 0.0], "extruder");
    let gcode = FakeGCode::default();
    let gcode_move = FakeGCodeMove::new(toolhead.clone());
    printer.add("gcode", Arc::new(gcode.clone()) as ObjectRef);
    printer.add("gcode_move", Arc::new(gcode_move.clone()) as ObjectRef);
    printer.add("toolhead", Arc::new(toolhead.clone()) as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::new(printer.clone()),
    });
    let obj = ExcludeObject::create(&config).expect("create");
    printer.send("klippy:connect").expect("connect");
    let pair = Pair {
        obj,
        printer,
        gcode,
        toolhead,
        gcode_move,
    };

    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "obj")],
        "start obj",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "obj")],
        "exclude raises",
    );
}

#[test]
fn reset_file_handler_resets_state() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, c, g, th, gm, eo = make_pair(None)")
        .expect("py make_pair");
    let pair = make_pair(None);
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT_START",
        &[("NAME", "obj")],
        "start obj",
    );
    run_both(
        &mut py,
        &pair,
        "EXCLUDE_OBJECT",
        &[("NAME", "obj")],
        "exclude obj",
    );
    let expected = py.eval("p.send_event('virtual_sdcard:reset_file')");
    let actual = pair.printer.send("virtual_sdcard:reset_file");
    match expected {
        Ok(_) => {
            assert!(actual.is_ok(), "rust reset_file failed");
        }
        Err(e) => panic!("oracle raised {}: {}", e.class_name, e.message),
    }
    let expected_status = py.eval("eo.get_status()").expect("py status");
    check(
        "reset status",
        Ok(expected_status),
        status_value(&pair.obj),
    );
    let expected_resets = py.eval("gm.reset_count").expect("py resets");
    check("reset count", Ok(expected_resets), Value::int(pair.gcode_move.resets() as i64));
}

#[test]
fn missing_gcode_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p = FakePrinter(); c = FakeConfig(p)",
    )
    .expect("py bare printer");
    let printer = FakePrinter::default();
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::new(printer),
    });
    let expected = py.eval("eo_mod.load_config(c)");
    let actual = ExcludeObject::create(&config);
    check_err("missing gcode", expected, actual);
}

#[test]
fn missing_toolhead_at_connect_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p = FakePrinter(); g = FakeGCode(); th = FakeToolhead(); gm = FakeGCodeMove(th); \
         p.objects['gcode'] = g; p.objects['gcode_move'] = gm; p.objects['tuning_tower'] = FakeTuningTower(False); \
         c = FakeConfig(p); eo = eo_mod.load_config(c)",
    )
    .expect("py fresh object");
    let expected = py.eval("p.send_event('klippy:connect')");
    let printer = FakePrinter::default();
    let toolhead = FakeToolhead::new(vec![0.0, 0.0, 0.0, 0.0], "extruder");
    let gcode = FakeGCode::default();
    let gcode_move = FakeGCodeMove::new(toolhead.clone());
    printer.add("gcode", Arc::new(gcode.clone()) as ObjectRef);
    printer.add("gcode_move", Arc::new(gcode_move.clone()) as ObjectRef);
    printer.add(
        "tuning_tower",
        Arc::new(FakeTuningTower { active: false }) as ObjectRef,
    );
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::new(printer.clone()),
    });
    let _ = ExcludeObject::create(&config).expect("create");
    let actual = printer.send("klippy:connect");
    check_err("missing toolhead at connect", expected, actual);
}

#[test]
fn factory_load_config_builds_working_object() {
    let factory = extras::exclude_object::module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let load_config = factory.load_config.expect("load_config");
    let pair = make_pair(None);
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::new(pair.printer.clone()),
    });
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory returns an object");
    let status = PrinterObject::get_status(&*obj, 0.0).expect("status");
    assert!(matches!(status, PyValue::Dict(_)));
}
