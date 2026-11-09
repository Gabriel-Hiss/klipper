//! Parity tests for `extras::stepper_enable` (port of `klippy/extras/stepper_enable.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a `PrinterStepperEnable` is
//! built on both sides with minimal Python fakes for the printer, gcode,
//! toolhead, pins, chip and steppers, then driven through identical
//! scenarios (registering steppers, firing stepper active callbacks,
//! running `M18` / `M84` / `SET_STEPPER_ENABLE`, `motor_off`, restart
//! events) and the recorded calls, status dicts, return values, log
//! lines and raised errors are compared exactly (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::stepper_enable::{PrinterStepperEnable, CMD_SET_STEPPER_ENABLE_HELP, DISABLE_STALL_TIME};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetIntOpts as GGetIntOpts, GetOpts, GCmd, GCodeApi, GCodeCommandHandler, GetParser,
    RegisterCommandOpts,
};
use klippy_api::mcu::{DigitalOutPinApi, LookupPinOpts, McuApi, Pin, PinParams, PinsApi};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_GCODE_REQUEST_RESTART,
};
use klippy_api::stepper::McuStepperApi;
use klippy_api::toolhead::{HomingTargetApi, KinematicsApi, ToolHeadApi};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::text::float_str;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging
import extras.stepper_enable as stepper_enable

class CommandError(Exception):
    pass

PY_LOGS = []
class _LogCapture(logging.Handler):
    def emit(self, record):
        PY_LOGS.append("%s:%s" % (logging.getLevelName(record.levelno), record.getMessage()))
logging.getLogger().addHandler(_LogCapture())
logging.getLogger().setLevel(logging.INFO)

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.events = []
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        raise self.config_error("Unknown config object '%s'" % (name,))
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event):
        self.events.append(event)
        return []

class FakeGCode:
    def __init__(self):
        self.commands = {}
        self.mux = {}
        self.registered = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if cmd in self.commands:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.commands[cmd] = func
        self.registered.append(('command', cmd, when_not_ready, desc))
    def register_mux_command(self, cmd, key, value, func, desc=None):
        if (cmd, key, value) in self.mux:
            raise configparser.Error("mux command %s %s %s already registered (%s)" % (cmd, key, value, self.mux[(cmd, key, value)]))
        self.mux[(cmd, key, value)] = func
        self.registered.append(('mux', cmd, key, value, desc))

class FakeKinematics:
    def __init__(self, calls):
        self.calls = calls
    def clear_homing_state(self, axes):
        self.calls.append('kin.clear_homing_state(%s)' % (axes,))

class FakeToolHead:
    def __init__(self, calls, last_move_time):
        self.calls = calls
        self.last_move_time = last_move_time
        self.kinematics = FakeKinematics(calls)
    def flush_step_generation(self):
        self.calls.append('toolhead.flush_step_generation')
    def get_last_move_time(self):
        self.calls.append('toolhead.get_last_move_time')
        return self.last_move_time
    def dwell(self, delay):
        self.calls.append('toolhead.dwell(%r)' % (delay,))
    def get_kinematics(self):
        return self.kinematics

class FakePin:
    def __init__(self, calls, label):
        self.calls = calls
        self.label = label
    def setup_max_duration(self, duration):
        self.calls.append('%s.setup_max_duration(%r)' % (self.label, duration))
    def set_digital(self, print_time, value):
        self.calls.append('%s.set_digital(%r, %r)' % (self.label, print_time, value))

class FakeChip:
    def __init__(self, calls):
        self.calls = calls
        self.count = 0
    def setup_pin(self, pin_type, pin_params):
        self.count += 1
        label = 'pin%d' % (self.count,)
        self.calls.append('chip.setup_pin(%r, %s:%s)' % (pin_type, pin_params['chip_name'], pin_params['pin']))
        return FakePin(self.calls, label)

class FakePins:
    def __init__(self, chip):
        self.chip = chip
        self.active = {}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        desc = pin_desc.strip()
        invert = 0
        if can_invert and desc.startswith('!'):
            invert = 1
            desc = desc[1:]
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        share_name = '%s:%s' % (chip_name, pin)
        if share_name in self.active:
            share_params = self.active[share_name]
            if share_type is None or share_type != share_params['share_type']:
                raise configparser.Error("pin %s used multiple times in config" % (pin,))
            return share_params
        params = {'chip': self.chip, 'chip_name': chip_name, 'pin': pin,
                  'invert': invert, 'pullup': 0, 'share_type': share_type}
        self.active[share_name] = params
        return params

class FakeStepper:
    def __init__(self, name):
        self._name = name
        self.active_callbacks = []
    def get_name(self, short=False):
        if short and self._name.startswith('stepper'):
            return self._name[8:]
        return self._name
    def add_active_callback(self, cb):
        self.active_callbacks.append(cb)
    def fire_active(self, print_time):
        cbs = self.active_callbacks
        self.active_callbacks = []
        for cb in cbs:
            cb(print_time)
        return len(cbs)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options):
        self._printer = printer
        self._options = options
    def get_printer(self):
        return self._printer
    def get(self, option, default=None):
        return self._options.get(option, default)

class FakeGcmd:
    class sentinel:
        pass
    def __init__(self, commandline, params):
        self._commandline = commandline
        self._params = params
    def get(self, name, default=sentinel, parser=str):
        value = self._params.get(name)
        if value is None:
            if default is self.sentinel:
                raise CommandError("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            return parser(value)
        except Exception:
            raise CommandError("Error on '%s': unable to parse %s" % (self._commandline, value))
    def get_int(self, name, default=sentinel):
        return self.get(name, default, parser=int)

def build_world(steppers, last_move_time):
    global printer, gcode, toolhead, pins, chip, pin_calls, th_calls, se, py_steppers
    printer = FakePrinter()
    gcode = FakeGCode()
    th_calls = []
    toolhead = FakeToolHead(th_calls, last_move_time)
    pin_calls = []
    chip = FakeChip(pin_calls)
    pins = FakePins(chip)
    printer.add_object('gcode', gcode)
    printer.add_object('toolhead', toolhead)
    printer.add_object('pins', pins)
    cfg = FakeConfig(printer, {})
    se = stepper_enable.PrinterStepperEnable(cfg)
    printer.add_object('stepper_enable', se)
    py_steppers = {}
    for name, pin in steppers.items():
        opts = {} if pin is None else {'enable_pin': pin}
        scfg = FakeConfig(printer, opts)
        st = FakeStepper(name)
        py_steppers[name] = st
        se.register_stepper(scfg, st)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

// Shared `McuStepperApi::add_active_callback` callback value.
type ActiveCallback = Box<dyn FnMut(f64) + Send>;

// Shared `GCodeApi::register_mux_command` mux key `(cmd, key, value)`.
type MuxKey = (String, String, Option<String>);

// Process-wide captured log records (`INFO:message`), mirroring the Python
// `PY_LOGS` capture in SETUP. Only the log test reads it.
static RUST_LOGS: std::sync::LazyLock<Calls> =
    std::sync::LazyLock::new(|| Arc::new(GCell::new(Vec::new())));

fn install_log_sink() {
    let _ = klippy_api::log::set_sink(Box::new(|rec| {
        RUST_LOGS
            .borrow_mut()
            .push(format!("{}:{}", rec.level.name(), rec.message));
    }));
}

// Shared call recorder, mirrors the Python fakes' `pin_calls` / `th_calls`.
type Calls = Arc<GCell<Vec<String>>>;

fn new_calls() -> Calls {
    Arc::new(GCell::new(Vec::new()))
}

fn join(calls: &Calls) -> String {
    calls.borrow().join(";")
}

fn py_str(s: &str) -> String {
    format!("'{s}'")
}

fn py_bool(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

fn py_desc(desc: &Option<String>) -> String {
    match desc {
        Some(d) => format!("'{d}'"),
        None => "None".to_string(),
    }
}

struct PrinterState {
    objects: IndexMap<String, ObjectRef>,
    handlers: IndexMap<String, Vec<EventCallback>>,
    events: Vec<String>,
}

struct FakePrinter {
    state: GCell<PrinterState>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            state: GCell::new(PrinterState {
                objects: IndexMap::new(),
                handlers: IndexMap::new(),
                events: Vec::new(),
            }),
        }
    }

    fn events(&self) -> Vec<String> {
        self.state.borrow().events.clone()
    }

    fn fire_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        // Take the callbacks out before invoking them: a handler (`motor_off`)
        // looks the printer up again, so the borrow must not be held across it.
        let mut callbacks = self
            .state
            .borrow_mut()
            .handlers
            .shift_remove(event)
            .unwrap_or_default();
        let mut result: KResult<()> = Ok(());
        for cb in callbacks.iter_mut() {
            if let Err(e) = cb(params) {
                result = Err(e);
                break;
            }
        }
        {
            let mut state = self.state.borrow_mut();
            let entry = state.handlers.entry(event.to_string()).or_default();
            entry.splice(0..0, callbacks.drain(..));
        }
        result?;
        Ok(Vec::new())
    }

    fn handler_names(&self) -> Vec<String> {
        self.state.borrow().handlers.keys().cloned().collect()
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
        let mut state = self.state.borrow_mut();
        if state.objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        state.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.state.borrow().objects.get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
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
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.state
            .borrow_mut()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.state.borrow_mut().events.push(event.to_string());
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeGCode {
    commands: GCell<IndexMap<String, GCodeCommandHandler>>,
    mux: GCell<IndexMap<MuxKey, GCodeCommandHandler>>,
    records: GCell<Vec<String>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            commands: GCell::new(IndexMap::new()),
            mux: GCell::new(IndexMap::new()),
            records: GCell::new(Vec::new()),
        }
    }

    fn records(&self) -> String {
        self.records.borrow().join(";")
    }

    fn call(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        // Take the handler out before invoking it: it may reach back into
        // this object, so the borrow must not be held across the call.
        let mut handler = self.commands.borrow_mut().shift_remove(cmd).expect("command registered");
        let result = handler(gcmd);
        self.commands.borrow_mut().insert(cmd.to_string(), handler);
        result
    }

    fn call_mux(&self, cmd: &str, key: &str, value: Option<&str>, gcmd: &GCmd) -> KResult<()> {
        let mux_key = (cmd.to_string(), key.to_string(), value.map(|v| v.to_string()));
        let mut entry = self.mux.borrow_mut().shift_remove(&mux_key).expect("mux command registered");
        let result = entry(gcmd);
        self.mux.borrow_mut().insert(mux_key, entry);
        result
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
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
                self.records.borrow_mut().push(format!(
                    "('command', {}, {}, {})",
                    py_str(cmd),
                    py_bool(opts.when_not_ready),
                    py_desc(&opts.desc),
                ));
                self.commands.borrow_mut().insert(cmd.to_string(), handler);
                Ok(None)
            }
            None => Ok(None),
        }
    }

    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.records.borrow_mut().push(format!(
            "('mux', {}, {}, {}, {})",
            py_str(cmd),
            py_str(key),
            match value {
                Some(v) => py_str(v),
                None => "None".to_string(),
            },
            py_desc(&desc.map(|d| d.to_string())),
        ));
        self.mux.borrow_mut().insert(
            (cmd.to_string(), key.to_string(), value.map(|v| v.to_string())),
            func,
        );
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
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by stepper_enable")
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

struct FakeKinematics {
    calls: Calls,
}

impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        panic!("get_steppers is not used by stepper_enable")
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        panic!("calc_position is not used by stepper_enable")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by stepper_enable")
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        panic!("check_move is not used by stepper_enable")
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        panic!("home is not used by stepper_enable")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by stepper_enable")
    }
    fn clear_homing_state(&self, clear_axes: &str) {
        self.calls
            .borrow_mut()
            .push(format!("kin.clear_homing_state({clear_axes})"))
    }
}

struct FakeToolHead {
    calls: Calls,
    kinematics: Arc<FakeKinematics>,
    last_move_time: f64,
}

impl PrinterObject for FakeToolHead {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ToolHeadApi);
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by stepper_enable")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by stepper_enable")
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        self.kinematics.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        self.calls.borrow_mut().push("toolhead.flush_step_generation".to_string());
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        self.calls.borrow_mut().push("toolhead.get_last_move_time".to_string());
        self.last_move_time
    }
    fn dwell(&self, delay: f64) {
        self.calls
            .borrow_mut()
            .push(format!("toolhead.dwell({})", float_str(delay)))
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by stepper_enable")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by stepper_enable")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by stepper_enable")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by stepper_enable")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by stepper_enable")
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq is not used by stepper_enable")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by stepper_enable")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by stepper_enable")
    }
    fn register_lookahead_callback(&self, _callback: klippy_api::toolhead::LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by stepper_enable")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by stepper_enable")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by stepper_enable")
    }
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by stepper_enable")
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder is not used by stepper_enable")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by stepper_enable")
    }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) {
        panic!("remove_extra_axis is not used by stepper_enable")
    }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        panic!("get_extra_axes is not used by stepper_enable")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by stepper_enable")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by stepper_enable")
    }
}

struct FakePin {
    calls: Calls,
    label: String,
}

impl DigitalOutPinApi for FakePin {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("get_mcu is not used by stepper_enable")
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.calls
            .borrow_mut()
            .push(format!("{}.setup_max_duration({})", self.label, float_str(max_duration)))
    }
    fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {
        panic!("setup_start_value is not used by stepper_enable")
    }
    fn set_digital(&self, print_time: f64, value: bool) {
        self.calls.borrow_mut().push(format!(
            "{}.set_digital({}, {})",
            self.label,
            float_str(print_time),
            if value { "1" } else { "0" }
        ))
    }
}

struct FakeChip {
    calls: Calls,
    count: GCell<usize>,
}

impl McuApi for FakeChip {
    fn get_name(&self) -> String {
        panic!("get_name is not used by stepper_enable")
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by stepper_enable")
    }
    fn is_fileoutput(&self) -> bool {
        panic!("is_fileoutput is not used by stepper_enable")
    }
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        self.calls.borrow_mut().push(format!(
            "chip.setup_pin({}, {}:{})",
            py_str(pin_type),
            pin_params.chip_name,
            pin_params.pin
        ));
        let count = self.count.get() + 1;
        self.count.set(count);
        Ok(Pin::DigitalOut(Arc::new(FakePin {
            calls: self.calls.clone(),
            label: format!("pin{count}"),
        })))
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by stepper_enable")
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by stepper_enable")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by stepper_enable")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by stepper_enable")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by stepper_enable")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by stepper_enable")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by stepper_enable")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by stepper_enable")
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<klippy_api::mcu::CommandQueue>) -> KResult<klippy_api::mcu::CommandWrapper> {
        panic!("lookup_command is not used by stepper_enable")
    }
    fn lookup_query_command(&self, _msgformat: &str, _respformat: &str, _opts: klippy_api::mcu::LookupQueryCommandOpts) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        panic!("lookup_query_command is not used by stepper_enable")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        panic!("try_lookup_command is not used by stepper_enable")
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        panic!("alloc_command_queue is not used by stepper_enable")
    }
    fn register_serial_response(&self, _cb: klippy_api::mcu::SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("register_serial_response is not used by stepper_enable")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by stepper_enable")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("get_enumerations is not used by stepper_enable")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("get_constants is not used by stepper_enable")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by stepper_enable")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by stepper_enable")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by stepper_enable")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by stepper_enable")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by stepper_enable")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by stepper_enable")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by stepper_enable")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by stepper_enable")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by stepper_enable")
    }
}

struct FakePins {
    chip: Arc<FakeChip>,
}

impl PrinterObject for FakePins {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PinsApi);
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by stepper_enable")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let mut desc = pin_desc.trim().to_string();
        let mut invert = 0;
        if opts.can_invert && desc.starts_with('!') {
            invert = 1;
            desc = desc[1..].to_string();
        }
        let (chip_name, pin) = match desc.find(':') {
            Some(i) => (desc[..i].trim().to_string(), desc[i + 1..].trim().to_string()),
            None => ("mcu".to_string(), desc),
        };
        Ok(PinParams {
            chip: self.chip.clone(),
            chip_name,
            pin,
            invert,
            pullup: 0,
            share_type: opts.share_type,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by stepper_enable")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by stepper_enable")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by stepper_enable")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by stepper_enable")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by stepper_enable")
    }
}

struct FakeStepper {
    name: String,
    active_callbacks: GCell<Vec<ActiveCallback>>,
}

impl FakeStepper {
    fn new(name: &str) -> FakeStepper {
        FakeStepper {
            name: name.to_string(),
            active_callbacks: GCell::new(Vec::new()),
        }
    }

    fn fire_active(&self, print_time: f64) -> usize {
        let callbacks = {
            let mut guard = self.active_callbacks.borrow_mut();
            std::mem::take(&mut *guard)
        };
        let count = callbacks.len();
        for mut cb in callbacks {
            cb(print_time);
        }
        count
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("get_mcu is not used by stepper_enable")
    }
    fn get_name(&self, short: Option<bool>) -> String {
        match short {
            Some(true) if self.name.starts_with("stepper") => self.name[8..].to_string(),
            _ => self.name.clone(),
        }
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid is not used by stepper_enable")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians is not used by stepper_enable")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration is not used by stepper_enable")
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {
        panic!("setup_default_pulse_duration is not used by stepper_enable")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by stepper_enable")
    }
    fn get_step_dist(&self) -> f64 {
        panic!("get_step_dist is not used by stepper_enable")
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance is not used by stepper_enable")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        panic!("set_rotation_distance is not used by stepper_enable")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted is not used by stepper_enable")
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        panic!("set_dir_inverted is not used by stepper_enable")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by stepper_enable")
    }
    fn set_position(&self, _coord: [f64; 3]) {
        panic!("set_position is not used by stepper_enable")
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by stepper_enable")
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        panic!("get_mcu_position is not used by stepper_enable")
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        panic!("get_past_mcu_position is not used by stepper_enable")
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: i64) -> f64 {
        panic!("mcu_to_commanded_position is not used by stepper_enable")
    }
    fn dump_steps(&self, _count: i64, _start_clock: i64, _end_clock: i64) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        panic!("dump_steps is not used by stepper_enable")
    }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> {
        panic!("get_stepper_kinematics is not used by stepper_enable")
    }
    fn set_stepper_kinematics(&self, _sk: Option<klippy_api::stepper::StepperKinematics>) -> Option<klippy_api::stepper::StepperKinematics> {
        panic!("set_stepper_kinematics is not used by stepper_enable")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end is not used by stepper_enable")
    }
    fn get_trapq(&self) -> Option<klippy_api::stepper::Trapq> {
        panic!("get_trapq is not used by stepper_enable")
    }
    fn set_trapq(&self, _tq: Option<klippy_api::stepper::Trapq>) -> Option<klippy_api::stepper::Trapq> {
        panic!("set_trapq is not used by stepper_enable")
    }
    fn add_active_callback(&self, cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        self.active_callbacks.borrow_mut().push(cb);
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis is not used by stepper_enable")
    }
}

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "stepper_enable".to_string()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.options.get(option) {
            Some(value) => Ok(Some(value.clone())),
            None => match opts.default {
                None => Err(KlippyError::Config(format!("missing '{option}'"))),
                Some(default) => Ok(default),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by stepper_enable")
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by stepper_enable")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by stepper_enable")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by stepper_enable")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by stepper_enable")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by stepper_enable")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by stepper_enable")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by stepper_enable")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by stepper_enable")
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

struct FakeGcmd {
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGcmd {
    fn new(commandline: &str, params: &[(&str, &str)]) -> FakeGcmd {
        FakeGcmd {
            commandline: commandline.to_string(),
            params: params
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        }
    }
}

impl klippy_api::gcode::GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(value) => Ok(value),
            },
            Some(value) => match opts.parser {
                GetParser::Str => Ok(PyValue::Str(value.clone())),
                _ => panic!("only the str parser is used by stepper_enable"),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(default) => Ok(default),
            },
            Some(value) => match value.parse::<i64>() {
                Ok(parsed) => Ok(Some(parsed)),
                Err(_) => Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {value}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_float(&self, _name: &str, _opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        panic!("get_float is not used by stepper_enable")
    }
    fn get_command(&self) -> String {
        self.commandline.split_whitespace().next().unwrap_or("").to_string()
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
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// Rust-side world assembled from the same fixtures as `build_world`.
#[allow(dead_code)]
struct RustWorld {
    printer: Printer,
    printer_obj: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    se: Arc<PrinterStepperEnable>,
    steppers: IndexMap<String, Arc<FakeStepper>>,
    pin_calls: Calls,
    th_calls: Calls,
}

fn build_rust_world(steppers: &[(&str, Option<&str>)], last_move_time: f64) -> RustWorld {
    let printer_obj = Arc::new(FakePrinter::new());
    let printer: Printer = printer_obj.clone();
    let pin_calls = new_calls();
    let chip = Arc::new(FakeChip {
        calls: pin_calls.clone(),
        count: GCell::new(0),
    });
    let pins = Arc::new(FakePins { chip });
    let th_calls = new_calls();
    let kinematics = Arc::new(FakeKinematics {
        calls: th_calls.clone(),
    });
    let toolhead = Arc::new(FakeToolHead {
        calls: th_calls.clone(),
        kinematics,
        last_move_time,
    });
    let gcode = Arc::new(FakeGCode::new());
    printer.add_object("gcode", gcode.clone() as ObjectRef).unwrap();
    printer.add_object("toolhead", toolhead.clone() as ObjectRef).unwrap();
    printer.add_object("pins", pins as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        options: IndexMap::new(),
    });
    let se = PrinterStepperEnable::new(&config).unwrap();
    printer.add_object("stepper_enable", se.clone() as ObjectRef).unwrap();
    let mut stepper_map = IndexMap::new();
    for (name, pin) in steppers {
        let mut options = IndexMap::new();
        if let Some(pin) = pin {
            options.insert("enable_pin".to_string(), pin.to_string());
        }
        let stepper_config: Config = Arc::new(FakeConfig {
            printer: printer.clone(),
            options,
        });
        let stepper = Arc::new(FakeStepper::new(name));
        let stepper_dyn: klippy_api::stepper::Stepper = stepper.clone();
        se.register_stepper(&stepper_config, &stepper_dyn).unwrap();
        stepper_map.insert(name.to_string(), stepper);
    }
    RustWorld {
        printer,
        printer_obj,
        gcode,
        toolhead,
        se,
        steppers: stepper_map,
        pin_calls,
        th_calls,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("stepper_enable fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping stepper_enable oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "stepper_enable".to_string(),
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

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
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
        "{label}: exception class"
    );
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

fn str_list(values: Vec<String>) -> Value {
    Value::List(values.iter().map(|v| Value::str(v)).collect())
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn disable_stall_time_matches_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("stepper_enable.DISABLE_STALL_TIME").expect("eval DISABLE_STALL_TIME");
    check(
        "disable_stall_time_matches_python",
        Ok(expected),
        Value::float(DISABLE_STALL_TIME),
    );
}

#[test]
fn register_commands_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    let expected = py.eval("';'.join(str(r) for r in gcode.registered)").expect("eval registered");
    check("register_commands_match_python", Ok(expected), Value::str(&world.gcode.records()));
    let expected = py.eval("sorted(printer.handlers.keys())").expect("eval handlers");
    check(
        "register_commands_match_python restart handler",
        Ok(expected),
        str_list(world.printer_obj.handler_names()),
    );
}

#[test]
fn register_stepper_pin_setup_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1', 'stepper_y': 'mcu:PA2'}, 5.0)")
        .expect("py build_world");
    let world = build_rust_world(
        &[("stepper_x", Some("mcu:PA1")), ("stepper_y", Some("mcu:PA2"))],
        5.0,
    );
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check(
        "register_stepper_pin_setup_matches_python",
        Ok(expected),
        Value::str(&join(&world.pin_calls)),
    );
    let expected = py.eval("se.get_steppers()").expect("eval get_steppers");
    let actual = str_list(world.se.get_steppers());
    check("register_stepper get_steppers", Ok(expected), actual);
    let expected = py.eval("se.get_status(0)").expect("eval get_status");
    check(
        "register_stepper get_status",
        Ok(expected),
        Value::Dict(vec![(
            Value::str("steppers"),
            Value::Dict(vec![
                (Value::str("stepper_x"), Value::Bool(false)),
                (Value::str("stepper_y"), Value::Bool(false)),
            ]),
        )]),
    );
}

#[test]
fn shared_enable_pin_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1', 'stepper_y': 'mcu:PA1'}, 5.0)")
        .expect("py build_world");
    let world = build_rust_world(
        &[("stepper_x", Some("mcu:PA1")), ("stepper_y", Some("mcu:PA1"))],
        5.0,
    );
    // One `setup_pin` for the shared pin, and both trackers report a shared line.
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check("shared_enable_pin pin_calls", Ok(expected), Value::str(&join(&world.pin_calls)));
    let expected = py.eval("se.lookup_enable('stepper_x').has_dedicated_enable()")
        .expect("eval has_dedicated_enable");
    check(
        "shared_enable_pin has_dedicated_enable x",
        Ok(expected),
        Value::Bool(world.se.lookup_enable("stepper_x").unwrap().has_dedicated_enable()),
    );
    let expected = py.eval("se.lookup_enable('stepper_y').has_dedicated_enable()")
        .expect("eval has_dedicated_enable");
    check(
        "shared_enable_pin has_dedicated_enable y",
        Ok(expected),
        Value::Bool(world.se.lookup_enable("stepper_y").unwrap().has_dedicated_enable()),
    );
    // Enable both: refcount 2. Disable one: no digital transition. Disable second: transition.
    py.exec("py_steppers['stepper_x'].fire_active(5.0); py_steppers['stepper_y'].fire_active(5.0)")
        .expect("py fire");
    world.steppers["stepper_x"].fire_active(5.0);
    world.steppers["stepper_y"].fire_active(5.0);
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls enabled");
    check("shared_enable_pin enabled", Ok(expected), Value::str(&join(&world.pin_calls)));
    let expected = py.eval("se.set_motors_enable(['stepper_x'], False)");
    let actual = world.se.set_motors_enable(&["stepper_x".to_string()], false);
    let actual_bool = match actual {
        Ok(b) => b,
        Err(e) => panic!("rust: {e:?}"),
    };
    check("shared_enable_pin disable x did_change", expected, Value::Bool(actual_bool));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls after disable x");
    check("shared_enable_pin after disable x", Ok(expected), Value::str(&join(&world.pin_calls)));
    let expected = py.eval("se.set_motors_enable(['stepper_y'], False)");
    let actual = world.se.set_motors_enable(&["stepper_y".to_string()], false);
    let actual_bool = match actual {
        Ok(b) => b,
        Err(e) => panic!("rust: {e:?}"),
    };
    check("shared_enable_pin disable y did_change", expected, Value::Bool(actual_bool));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls after disable y");
    check("shared_enable_pin after disable y", Ok(expected), Value::str(&join(&world.pin_calls)));
}

#[test]
fn no_enable_pin_stepper_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': None}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", None)], 5.0);
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check("no_enable_pin pin_calls", Ok(expected), Value::str(&join(&world.pin_calls)));
    let expected = py.eval("se.lookup_enable('stepper_x').has_dedicated_enable()")
        .expect("eval has_dedicated_enable");
    check(
        "no_enable_pin has_dedicated_enable",
        Ok(expected),
        Value::Bool(world.se.lookup_enable("stepper_x").unwrap().has_dedicated_enable()),
    );
    py.exec("py_steppers['stepper_x'].fire_active(5.0)").expect("py fire");
    world.steppers["stepper_x"].fire_active(5.0);
    let expected = py.eval("se.get_status(0)").expect("eval status enabled");
    check(
        "no_enable_pin status enabled",
        Ok(expected),
        Value::Dict(vec![(
            Value::str("steppers"),
            Value::Dict(vec![(Value::str("stepper_x"), Value::Bool(true))]),
        )]),
    );
    let expected = py.eval("se.set_motors_enable(['stepper_x'], False)");
    let actual = world.se.set_motors_enable(&["stepper_x".to_string()], false);
    let actual_bool = match actual {
        Ok(b) => b,
        Err(e) => panic!("rust: {e:?}"),
    };
    check("no_enable_pin disable did_change", expected, Value::Bool(actual_bool));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls after disable");
    check("no_enable_pin after disable", Ok(expected), Value::str(&join(&world.pin_calls)));
}

#[test]
fn inverted_enable_pin_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': '!PA3'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("!PA3"))], 5.0);
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check("inverted_enable_pin pin_calls", Ok(expected), Value::str(&join(&world.pin_calls)));
}

#[test]
fn fire_active_enables_with_state_callbacks() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "build_world({'stepper_x': 'mcu:PA1'}, 5.0); se.lookup_enable('stepper_x').register_state_callback(lambda t, e: pin_calls.append('cb(%r, %r)' % (t, e)))",
    )
    .expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    let record_calls = world.pin_calls.clone();
    world.se.lookup_enable("stepper_x").unwrap().register_state_callback(Box::new(
        move |print_time: f64, is_enable: bool| {
            record_calls.borrow_mut().push(format!(
                "cb({}, {})",
                float_str(print_time),
                if is_enable { "True" } else { "False" }
            ));
            let _ = is_enable;
        },
    ));
    let expected = py.eval("py_steppers['stepper_x'].fire_active(5.0)").expect("py fire");
    let fired = world.steppers["stepper_x"].fire_active(5.0);
    check("fire_active_enables_with_state_callbacks fired count", Ok(expected), Value::int(fired as i64));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check(
        "fire_active_enables_with_state_callbacks",
        Ok(expected),
        Value::str(&join(&world.pin_calls)),
    );
    let expected = py.eval("se.lookup_enable('stepper_x').is_motor_enabled()")
        .expect("eval is_motor_enabled");
    check(
        "fire_active_enables_with_state_callbacks is_motor_enabled",
        Ok(expected),
        Value::Bool(world.se.lookup_enable("stepper_x").unwrap().is_motor_enabled()),
    );
    // Second fire is a no-op: the callback list was drained by the stepper.
    py.exec("py_steppers['stepper_x'].fire_active(5.0)").expect("py fire again");
    world.steppers["stepper_x"].fire_active(5.0);
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls after second fire");
    check(
        "fire_active_enables_with_state_callbacks second fire",
        Ok(expected),
        Value::str(&join(&world.pin_calls)),
    );
}

#[test]
fn state_callback_registered_mid_pass_runs_in_same_pass() {
    let Some(mut py) = oracle() else { return };
    py.exec(concat!(
        "build_world({'stepper_x': 'mcu:PA1'}, 5.0)\n",
        "et = se.lookup_enable('stepper_x')\n",
        "def cb_b(t, e):\n",
        "    pin_calls.append('b(%r, %r)' % (t, e))\n",
        "def cb_a(t, e):\n",
        "    pin_calls.append('a(%r, %r)' % (t, e))\n",
        "    et.register_state_callback(cb_b)\n",
        "et.register_state_callback(cb_a)\n",
        "et.motor_enable(7.0)\n",
        "et.motor_disable(8.0)\n",
    ))
    .expect("py mid-pass registration");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    let et = world.se.lookup_enable("stepper_x").unwrap();
    let calls = world.pin_calls.clone();
    let et_for_a = et.clone();
    et.register_state_callback(Box::new(move |t: f64, e: bool| {
        calls.borrow_mut().push(format!("a({}, {})", float_str(t), py_bool(e)));
        let calls_b = calls.clone();
        et_for_a.register_state_callback(Box::new(move |t: f64, e: bool| {
            calls_b.borrow_mut().push(format!("b({}, {})", float_str(t), py_bool(e)));
        }));
    }));
    et.motor_enable(7.0);
    et.motor_disable(8.0).expect("rust motor_disable");
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check(
        "state_callback_registered_mid_pass_runs_in_same_pass",
        Ok(expected),
        Value::str(&join(&world.pin_calls)),
    );
}

#[test]
fn disable_reregisters_active_callback() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    py.exec("py_steppers['stepper_x'].fire_active(5.0)").expect("py fire");
    world.steppers["stepper_x"].fire_active(5.0);
    py.exec("se.set_motors_enable(['stepper_x'], False)").expect("py disable");
    world.se.set_motors_enable(&["stepper_x".to_string()], false).unwrap();
    // After a manual disable the stepper sees a new active callback...
    let expected = py.eval("py_steppers['stepper_x'].fire_active(5.0)").expect("py re-fire");
    let fired = world.steppers["stepper_x"].fire_active(5.0);
    check("disable_reregisters_active_callback fired count", Ok(expected), Value::int(fired as i64));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check(
        "disable_reregisters_active_callback",
        Ok(expected),
        Value::str(&join(&world.pin_calls)),
    );
    let expected = py.eval("se.get_status(0)").expect("eval get_status");
    check(
        "disable_reregisters_active_callback status",
        Ok(expected),
        Value::Dict(vec![(
            Value::str("steppers"),
            Value::Dict(vec![(Value::str("stepper_x"), Value::Bool(true))]),
        )]),
    );
}

#[test]
fn multi_stepper_enable_shares_print_time() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1', 'stepper_y': 'mcu:PA2'}, 5.0)")
        .expect("py build_world");
    let world = build_rust_world(
        &[("stepper_x", Some("mcu:PA1")), ("stepper_y", Some("mcu:PA2"))],
        5.0,
    );
    let expected = py.eval("se.set_motors_enable(['stepper_x', 'stepper_y'], True)")
        .expect("py enable both");
    let actual = world
        .se
        .set_motors_enable(&["stepper_x".to_string(), "stepper_y".to_string()], true);
    let actual_bool = match actual {
        Ok(b) => b,
        Err(e) => panic!("rust: {e:?}"),
    };
    check("multi_stepper_enable_shares_print_time", Ok(expected), Value::Bool(actual_bool));
    let expected = py.eval("';'.join(th_calls)").expect("eval th_calls");
    check("multi_stepper_enable_shares_print_time th", Ok(expected), Value::str(&join(&world.th_calls)));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check("multi_stepper_enable_shares_print_time pin", Ok(expected), Value::str(&join(&world.pin_calls)));
    // Idempotent second call: flush only.
    py.exec("se.set_motors_enable(['stepper_x', 'stepper_y'], True)").expect("py enable again");
    world
        .se
        .set_motors_enable(&["stepper_x".to_string(), "stepper_y".to_string()], true)
        .unwrap();
    let expected = py.eval("';'.join(th_calls)").expect("eval th_calls after repeat");
    check(
        "multi_stepper_enable_shares_print_time th repeat",
        Ok(expected),
        Value::str(&join(&world.th_calls)),
    );
}

#[test]
fn motor_off_via_m18_and_m84() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1', 'stepper_y': 'mcu:PA2'}, 5.0)")
        .expect("py build_world");
    let world = build_rust_world(
        &[("stepper_x", Some("mcu:PA1")), ("stepper_y", Some("mcu:PA2"))],
        5.0,
    );
    py.exec(
        "py_steppers['stepper_x'].fire_active(5.0); py_steppers['stepper_y'].fire_active(5.0); gcode.commands['M18'](None)",
    )
    .expect("py M18");
    world.steppers["stepper_x"].fire_active(5.0);
    world.steppers["stepper_y"].fire_active(5.0);
    let cmd = Arc::new(FakeGcmd::new("M18", &[]));
    let cmd: GCmd = cmd.clone() as GCmd;
    world.gcode.call("M18", &cmd).unwrap();
    let expected = py.eval("';'.join(th_calls)").expect("eval th_calls");
    check("motor_off_via_m18_and_m84 th", Ok(expected), Value::str(&join(&world.th_calls)));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check("motor_off_via_m18_and_m84 pin", Ok(expected), Value::str(&join(&world.pin_calls)));
    let expected = py.eval("';'.join(printer.events)").expect("eval events");
    check("motor_off_via_m18_and_m84 events", Ok(expected), Value::str(&world.printer_obj.events().join(";")));
    // M84 behaves the same (motors already off: flush + event, no dwell).
    py.exec("gcode.commands['M84'](None)").expect("py M84");
    world.gcode.call("M84", &cmd).unwrap();
    let expected = py.eval("';'.join(printer.events)").expect("eval events after M84");
    check("motor_off_via_m18_and_m84 M84 events", Ok(expected), Value::str(&world.printer_obj.events().join(";")));
}

#[test]
fn restart_handler_motor_off_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    py.exec("py_steppers['stepper_x'].fire_active(5.0)").expect("py fire");
    world.steppers["stepper_x"].fire_active(5.0);
    py.exec("printer.handlers['gcode:request_restart'][0](5.0)").expect("py restart");
    world
        .printer_obj
        .fire_event(EV_GCODE_REQUEST_RESTART, &[EventArg::Value(PyValue::Float(5.0))])
        .unwrap();
    let expected = py.eval("';'.join(th_calls)").expect("eval th_calls");
    check("restart_handler_motor_off_matches_python th", Ok(expected), Value::str(&join(&world.th_calls)));
    let expected = py.eval("';'.join(printer.events)").expect("eval events");
    check("restart_handler_motor_off_matches_python events", Ok(expected), Value::str(&world.printer_obj.events().join(";")));
}

#[test]
fn set_stepper_enable_command_matches_python() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    RUST_LOGS.borrow_mut().clear();
    py.exec("build_world({'stepper_cmd': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_cmd", Some("mcu:PA1"))], 5.0);
    let cmd = Arc::new(FakeGcmd::new(
        "SET_STEPPER_ENABLE STEPPER=stepper_cmd ENABLE=1",
        &[("STEPPER", "stepper_cmd"), ("ENABLE", "1")],
    ));
    let cmd: GCmd = cmd.clone() as GCmd;
    let expected = py.eval(
        "gcode.mux[('SET_STEPPER_ENABLE', 'STEPPER', 'stepper_cmd')](FakeGcmd('SET_STEPPER_ENABLE STEPPER=stepper_cmd ENABLE=1', {'STEPPER': 'stepper_cmd', 'ENABLE': '1'}))",
    );
    let actual = world.gcode.call_mux("SET_STEPPER_ENABLE", "STEPPER", Some("stepper_cmd"), &cmd);
    match (expected, actual) {
        (Ok(_), Ok(())) => {}
        (e, a) => panic!("mismatch: {e:?} vs {a:?}"),
    }
    let expected = py.eval("';'.join(th_calls)").expect("eval th_calls");
    check("set_stepper_enable_command_matches_python th", Ok(expected), Value::str(&join(&world.th_calls)));
    // Disable again through the same mux entry, then check the manual log lines.
    let cmd = Arc::new(FakeGcmd::new(
        "SET_STEPPER_ENABLE STEPPER=stepper_cmd ENABLE=0",
        &[("STEPPER", "stepper_cmd"), ("ENABLE", "0")],
    ));
    let cmd: GCmd = cmd.clone() as GCmd;
    py.eval(
        "gcode.mux[('SET_STEPPER_ENABLE', 'STEPPER', 'stepper_cmd')](FakeGcmd('SET_STEPPER_ENABLE STEPPER=stepper_cmd ENABLE=0', {'STEPPER': 'stepper_cmd', 'ENABLE': '0'}))",
    )
    .expect("py SET_STEPPER_ENABLE off");
    world.gcode.call_mux("SET_STEPPER_ENABLE", "STEPPER", Some("stepper_cmd"), &cmd).unwrap();
    let expected = py.eval("';'.join(th_calls)").expect("eval th_calls after disable");
    check("set_stepper_enable_command_matches_python th disable", Ok(expected), Value::str(&join(&world.th_calls)));
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls");
    check("set_stepper_enable_command_matches_python pin", Ok(expected), Value::str(&join(&world.pin_calls)));
    let expected = py.eval("';'.join(PY_LOGS)").expect("eval logs");
    let filtered: Vec<String> = RUST_LOGS
        .borrow()
        .iter()
        .filter(|m| m.contains("stepper_cmd"))
        .cloned()
        .collect();
    check(
        "set_stepper_enable_command_matches_python logs",
        Ok(expected),
        Value::str(&filtered.join(";")),
    );
}

#[test]
fn set_stepper_enable_defaults_and_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    // ENABLE omitted: defaults to 1 (enabled).
    let cmd = FakeGcmd::new("SET_STEPPER_ENABLE STEPPER=stepper_x", &[("STEPPER", "stepper_x")]);
    let cmd: GCmd = Arc::new(cmd);
    py.eval(
        "se.cmd_SET_STEPPER_ENABLE(FakeGcmd('SET_STEPPER_ENABLE STEPPER=stepper_x', {'STEPPER': 'stepper_x'}))",
    )
    .expect("py default ENABLE");
    world.se.cmd_set_stepper_enable(&cmd).unwrap();
    let expected = py.eval("';'.join(pin_calls)").expect("eval pin_calls default");
    check("set_stepper_enable_defaults_and_errors default", Ok(expected), Value::str(&join(&world.pin_calls)));
    // Unknown STEPPER: KeyError on the tracking lookup.
    let cmd = FakeGcmd::new(
        "SET_STEPPER_ENABLE STEPPER=stepper_z ENABLE=1",
        &[("STEPPER", "stepper_z"), ("ENABLE", "1")],
    );
    let cmd: GCmd = Arc::new(cmd);
    let expected = py.eval(
        "se.cmd_SET_STEPPER_ENABLE(FakeGcmd('SET_STEPPER_ENABLE STEPPER=stepper_z ENABLE=1', {'STEPPER': 'stepper_z', 'ENABLE': '1'}))",
    );
    check_err("set_stepper_enable_defaults_and_errors unknown", expected, world.se.cmd_set_stepper_enable(&cmd));
    // Missing STEPPER: CommandError from gcmd.get.
    let cmd = FakeGcmd::new("SET_STEPPER_ENABLE ENABLE=1", &[("ENABLE", "1")]);
    let cmd: GCmd = Arc::new(cmd);
    let expected = py.eval(
        "se.cmd_SET_STEPPER_ENABLE(FakeGcmd('SET_STEPPER_ENABLE ENABLE=1', {'ENABLE': '1'}))",
    );
    check_err("set_stepper_enable_defaults_and_errors missing", expected, world.se.cmd_set_stepper_enable(&cmd));
    // Unparseable ENABLE: CommandError from gcmd.get_int.
    let cmd = FakeGcmd::new(
        "SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=abc",
        &[("STEPPER", "stepper_x"), ("ENABLE", "abc")],
    );
    let cmd: GCmd = Arc::new(cmd);
    let expected = py.eval(
        "se.cmd_SET_STEPPER_ENABLE(FakeGcmd('SET_STEPPER_ENABLE STEPPER=stepper_x ENABLE=abc', {'STEPPER': 'stepper_x', 'ENABLE': 'abc'}))",
    );
    check_err("set_stepper_enable_defaults_and_errors unparseable", expected, world.se.cmd_set_stepper_enable(&cmd));
}

#[test]
fn lookup_enable_unknown_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    let expected = py.eval("se.lookup_enable('stepper_z')");
    let actual = world.se.lookup_enable("stepper_z");
    let oracle_exc = match expected {
        Ok(_) => panic!("expected oracle raise"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("expected rust raise"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "exception class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "exception message");
}

#[test]
fn set_motors_enable_unknown_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("build_world({'stepper_x': 'mcu:PA1'}, 5.0)").expect("py build_world");
    let world = build_rust_world(&[("stepper_x", Some("mcu:PA1"))], 5.0);
    let expected = py.eval("se.set_motors_enable(['stepper_z'], False)");
    let actual = world.se.set_motors_enable(&["stepper_z".to_string()], false);
    let oracle_exc = match expected {
        Ok(_) => panic!("expected oracle raise"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("expected rust raise"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "exception class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "exception message");
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::stepper_enable::module_factory();
    let load_config = factory
        .load_config
        .expect("stepper_enable exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "no add_early_printer_objects");
    assert!(factory.add_printer_objects.is_none(), "no add_printer_objects");
    // A fresh fixture world on each side; stepper_enable built only through
    // the factory entry point.
    py.exec(
        "p2 = FakePrinter(); g2 = FakeGCode(); p2.add_object('gcode', g2); c2 = FakeConfig(p2, {}); se2 = stepper_enable.load_config(c2)",
    )
    .expect("py factory load_config");
    let printer2: Printer = Arc::new(FakePrinter::new());
    let gcode2 = Arc::new(FakeGCode::new());
    printer2.add_object("gcode", gcode2.clone() as ObjectRef).unwrap();
    let config2: Config = Arc::new(FakeConfig {
        printer: printer2.clone(),
        options: IndexMap::new(),
    });
    let obj: ObjectRef = load_config(&config2)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let built: Arc<PrinterStepperEnable> = downcast_arc::<PrinterStepperEnable>(&obj)
        .expect("downcast to PrinterStepperEnable");
    let expected = py.eval("se2.get_steppers()").expect("eval get_steppers");
    check(
        "factory load_config get_steppers",
        Ok(expected),
        str_list(built.get_steppers()),
    );
    let expected = py.eval("';'.join(str(r) for r in g2.registered)").expect("eval registered");
    check("factory load_config registered", Ok(expected), Value::str(&gcode2.records()));
    let expected = py.eval("se2.get_status(0)").expect("eval status");
    check(
        "factory load_config status",
        Ok(expected),
        Value::Dict(vec![(Value::str("steppers"), Value::Dict(Vec::new()))]),
    );
    assert_eq!(CMD_SET_STEPPER_ENABLE_HELP, "Enable/disable individual stepper by name");
}
