//! Parity tests for `extras::filament_motion_sensor` (port of
//! `klippy/extras/filament_motion_sensor.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `EncoderSensor` is
//! built on both sides with minimal Python fakes for the printer, the
//! config wrapper, the reactor, the buttons object, the extruder, the
//! MCU and the `RunoutHelper` companions (`gcode`, `gcode_macro`,
//! `idle_timeout`). The real Python `RunoutHelper` runs unchanged on
//! the Python side; the Rust side runs the port against fakes of the
//! same contract traits. Construction reads, handler registration,
//! ready/printing events, encoder events, the periodic runout check
//! and `get_status` are all driven with identical inputs and compared
//! exactly (values, exception class and message).
//!
//! Reactor timer mechanics (register/update) are exercised through the
//! real `reactor` crate on the Rust side and through a recording fake
//! on the Python side; the timer firing protocol itself is verified by
//! the `reactor` crate's own tests.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use std::sync::Arc;

use extras::buttons::{
    AdcButtonCallback, ButtonCallback, DebounceCallback, PrinterButtonsApi, PushCallback,
};
use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras::filament_motion_sensor::{EncoderSensor, CHECK_RUNOUT_TIMEOUT};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandHandler, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper, CommandWrapper,
    ConfigCallback, LookupQueryCommandOpts, McuApi, Pin, PinParams, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::Trapq;
use klippy_api::toolhead::{
    ExtruderStepperApi, MoveApi, PrinterExtruderApi,
};
use klippy_api::{EventArg, GCell, PrinterObject};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.filament_motion_sensor as fms

# Sentinel mirroring configfile.sentinel.
SENTINEL = object()

class FakeReactor:
    NEVER = 9999999999999999.
    NOW = 0.
    def __init__(self):
        self.now = 10000.
        self.timers = []
        self.updates = []
        self.callbacks = []
    def monotonic(self):
        return self.now
    def register_timer(self, cb, waketime=None):
        self.timers.append(cb)
        return len(self.timers) - 1
    def update_timer(self, timer, waketime):
        self.updates.append((timer, waketime))
    def register_callback(self, cb):
        self.callbacks.append(cb)

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.reactor = FakeReactor()
        self.buttons = FakeButtons()
        self.extruder = FakeExtruder(None)
        self.mcu = FakeMcu()
        self.mcu_present = True
        self.gcode = FakeGcode()
        self.idle_timeout = FakeIdleTimeout()
        self.gcode_macro = FakeGCodeMacro()
        self.handlers = []
        self.objects = {}
        self.load_calls = []
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None and name == 'buttons':
            return self.buttons
        if default is None and name == 'gcode':
            return self.gcode
        if default is None and name == 'idle_timeout':
            return self.idle_timeout
        if name == 'mcu':
            if self.mcu_present:
                return self.mcu
            raise configparser.Error("Unknown config object 'mcu'")
        if name == 'extruder1' or name == 'stepper1':
            return self.extruder
        if default is SENTINEL or default is None:
            raise configparser.Error(
                "Unknown config object '%s'" % (name,))
        return None
    def load_object(self, config, section, default=None):
        self.load_calls.append(section)
        if section == 'buttons':
            return self.buttons
        if section == 'gcode_macro':
            return self.gcode_macro
        if default is SENTINEL or default is None:
            raise configparser.Error(
                "Unknown config object '%s'" % (section,))
        return None
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, opts=None):
        self._printer = printer
        self.opts = dict(opts or {})
        self.floats = {}
        self.bools = {}
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'filament_motion_sensor my_sensor'
    def get(self, option, default=SENTINEL):
        if option in self.opts:
            return self.opts[option]
        if default is SENTINEL:
            raise configparser.Error(
                "Option '%s' in section '%s' must be specified" % (
                    option, self.get_name()))
        return default
    def getfloat(self, option, default=None, **kwargs):
        if option in self.floats:
            v = self.floats[option]
        else:
            v = default
        if v is None:
            raise configparser.Error(
                "Option '%s' in section '%s' must be specified" % (
                    option, self.get_name()))
        above = kwargs.get('above', None)
        if above is not None and v <= above:
            raise configparser.Error(
                "Option '%s' in section '%s' must be above %s" % (
                    option, self.get_name(), above))
        return v
    def getboolean(self, option, default=None):
        if option in self.bools:
            return self.bools[option]
        return default

class FakeButtons:
    def __init__(self):
        self.regs = []
    def register_buttons(self, pins, cb):
        self.regs.append((list(pins), cb))

class FakeExtruder:
    def __init__(self, pos):
        self.pos = pos
    def find_past_position(self, print_time):
        return self.pos

class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime + 2.

class FakeIdleTimeout:
    def __init__(self):
        self.state = "Ready"
    def get_status(self, eventtime):
        return {"state": self.state}

class FakeGcode:
    def __init__(self):
        self.commands = []
    def register_mux_command(self, cmd, typ, name, func, desc=None):
        self.commands.append((cmd, typ, name))

class FakeTemplate:
    def render(self):
        return ""

class FakeGCodeMacro:
    def load_template(self, config, option, default=None):
        return FakeTemplate()
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinterState {
    handlers: Vec<(String, EventCallback)>,
     objects: Vec<(String, ObjectRef)>,
    load_calls: Vec<String>,
    buttons: Arc<FakeButtons>,
    gcode_macro: Option<Arc<FakeGCodeMacro>>,
}

struct FakePrinter {
    state: GCell<FakePrinterState>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> (Arc<Self>, Arc<FakeButtons>) {
        let buttons_obj = Arc::new(FakeButtons::default());
        let printer_ref = Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            state: GCell::new(FakePrinterState {
                handlers: Vec::new(),
                objects: Vec::new(),
                load_calls: Vec::new(),
                buttons: buttons_obj.clone(),
                gcode_macro: None,
            }),
        });
        printer_ref.state.borrow_mut().gcode_macro = Some(Arc::new(FakeGCodeMacro {
            printer: Arc::clone(&printer_ref) as Printer,
        }));
        (printer_ref, buttons_obj)
    }

    fn add_object(&self, name: &str, obj: ObjectRef) {
        self.state.borrow_mut().objects.push((name.to_string(), obj));
    }

    fn lookup(&self, name: &str) -> KResult<Option<ObjectRef>> {
        Ok(self
            .state
            .borrow()
            .objects
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, o)| o.clone()))
    }

    fn fire(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        // Detach the handler list before invoking: handlers call back
        // into printer lookups, and a GCell borrow would conflict.
        let mut handlers = std::mem::take(&mut self.state.borrow_mut().handlers);
        let mut rets = Vec::new();
        for (ev, cb) in handlers.iter_mut() {
            if ev == event {
                rets.push(cb(params)?);
            }
        }
        self.state.borrow_mut().handlers = handlers;
        Ok(rets)
    }

    fn handler_names(&self) -> Vec<String> {
        self.state
            .borrow()
            .handlers
            .iter()
            .map(|(n, _)| n.clone())
            .collect()
    }

    fn load_calls(&self) -> Vec<String> {
        self.state.borrow().load_calls.clone()
    }

    fn buttons_regs(&self) -> Vec<Vec<String>> {
        self.state.borrow().buttons.regs.borrow().clone()
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.lookup(name)? {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.state.borrow_mut().load_calls.push(section.to_string());
        match section {
            "buttons" => Ok(Some(self.state.borrow().buttons.clone() as ObjectRef)),
            "gcode_macro" => Ok(Some(
                self.state.borrow().gcode_macro.clone().expect("macro fake") as ObjectRef,
            )),
            "pause_resume" => Ok(Some(
                self.state.borrow().gcode_macro.clone().expect("macro fake") as ObjectRef,
            )),
            _ => Err(KlippyError::Config(format!(
                "Unknown config object '{section}'"
            ))),
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
            .borrow_mut()
            .handlers
            .push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        self.fire(event, params)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeGCodeMacro {
    printer: Printer,
}

klippy_api::printer_object!(FakeGCodeMacro; dyn PrinterGCodeMacroApi);

impl PrinterGCodeMacroApi for FakeGCodeMacro {
    fn load_template(
        &self,
        config: &Config,
        option: &str,
        default: Option<&str>,
    ) -> KResult<TemplateWrapper> {
        let script = config
            .get(
                option,
                GetStrOpts {
                    default: default.map(|d| Some(d.to_string())),
                    ..GetStrOpts::default()
                },
            )?
            .expect("script text");
        TemplateWrapper::new(
            &self.printer,
            &format!("{}:{}", config.get_name(), option),
            &script,
        )
    }
    fn create_template_context(
        &self,
        _eventtime: Option<f64>,
    ) -> indexmap::IndexMap<String, extras_macros_templates::gcode_macro::Value> {
        indexmap::IndexMap::new()
    }
}

struct FakeConfig {
    printer: Printer,
    opts: indexmap::IndexMap<String, String>,
    floats: indexmap::IndexMap<String, f64>,
    bools: indexmap::IndexMap<String, bool>,
}

impl FakeConfig {
    fn get_name(&self) -> String {
        "filament_motion_sensor my_sensor".to_string()
    }

    fn must_specify(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.get_name()
        ))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.get_name()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(self.must_specify(option)),
                Some(inner) => Ok(inner.clone()),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let v = self.floats.get(option).copied().or(opts.default.unwrap_or_default());
        let Some(v) = v else {
            return Err(self.must_specify(option));
        };
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.get_name(),
                    pycompat::text::float_repr(above),
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.bools.get(option).copied() {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Ok(None),
                Some(inner) => Ok(inner),
            },
        }
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
        panic!("getsection is not used by filament_motion_sensor")
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

// Object `gcode` machine: registers mux commands like the Python fake.
struct FakeGcode {
    commands: GCell<Vec<(String, String, String)>>,
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

impl GCodeApi for FakeGcode {
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
        _desc: Option<&str>,
    ) -> KResult<()> {
        self.commands
            .borrow_mut()
            .push((
                cmd.to_string(),
                key.to_string(),
                value.unwrap_or("").to_string(),
            ));
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
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("get_mutex is not used by filament_motion_sensor")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        panic!("create_gcode_command is not used by filament_motion_sensor")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by filament_motion_sensor")
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by filament_motion_sensor")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by filament_motion_sensor")
    }
}

// `idle_timeout` object whose `get_status` RunoutHelper can read.
struct FakeIdleTimeout {
    state: GCell<String>,
}

impl FakeIdleTimeout {
    fn new(state: &str) -> Self {
        FakeIdleTimeout {
            state: GCell::new(state.to_string()),
        }
    }
}

impl PrinterObject for FakeIdleTimeout {
    klippy_api::printer_object_base!();

    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut d = indexmap::IndexMap::new();
        d.insert(
            PyValue::Str("state".to_string()),
            PyValue::Str(self.state.borrow().clone()),
        );
        Some(PyValue::Dict(d))
    }
}

// The buttons object: records the registered pins so tests can match
// the Python recording of the same registration.
#[derive(Default)]
struct FakeButtons {
    regs: GCell<Vec<Vec<String>>>,
}

klippy_api::printer_object!(FakeButtons; dyn PrinterButtonsApi);

impl PrinterButtonsApi for FakeButtons {
    fn register_adc_button(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: AdcButtonCallback,
    ) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn register_debounce_button(
        &self,
        _pin: &str,
        _callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn register_debounce_adc_button(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn register_adc_button_push(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: PushCallback,
    ) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn register_buttons(&self, pins: Vec<String>, _callback: ButtonCallback) -> KResult<()> {
        self.regs.borrow_mut().push(pins);
        Ok(())
    }
    fn register_rotary_encoder(
        &self,
        _pin1: &str,
        _pin2: &str,
        _cw_callback: PushCallback,
        _ccw_callback: PushCallback,
        _steps_per_detent: i64,
    ) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn register_button_push(&self, _pin: &str, _callback: PushCallback) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
}

// Extruder with a controllable past position.
struct FakeExtruder {
    pos: GCell<f64>,
}

impl FakeExtruder {
    fn new(pos: f64) -> Self {
        FakeExtruder {
            pos: GCell::new(pos),
        }
    }

    fn unreachable(&self) -> ! {
        panic!("not used by filament_motion_sensor")
    }
}

klippy_api::printer_object!(FakeExtruder; dyn PrinterExtruderApi);

impl klippy_api::toolhead::ExtraAxisApi for FakeExtruder {
    fn get_name(&self) -> String {
        self.unreachable()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        self.unreachable()
    }
    fn get_trapq(&self) -> Option<Trapq> {
        self.unreachable()
    }
    fn check_move(&self, _mv: &dyn MoveApi, _ea_index: usize) -> KResult<()> {
        self.unreachable()
    }
    fn calc_junction(
        &self,
        _prev_move: &dyn MoveApi,
        _mv: &dyn MoveApi,
        _ea_index: usize,
    ) -> f64 {
        self.unreachable()
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {
        self.unreachable()
    }
}

impl PrinterExtruderApi for FakeExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        self.unreachable()
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(self.pos.get())
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

// An extruder-stepper object with the same position protocol.
struct FakeStepperExtruder {
    pos: GCell<f64>,
}

klippy_api::printer_object!(FakeStepperExtruder; dyn ExtruderStepperApi);

impl ExtruderStepperApi for FakeStepperExtruder {
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(self.pos.get())
    }
    fn sync_to_extruder(&self, _extruder_name: Option<&str>) -> KResult<()> {
        Ok(())
    }
    fn stepper(&self) -> klippy_api::stepper::Stepper {
        panic!("stepper is not used by filament_motion_sensor")
    }
    fn cmd_set_pressure_advance(&self, _gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        Ok(())
    }
}

// A plain object with no extruder shape (AttributeError path).
struct PlainObject;

klippy_api::printer_object!(PlainObject);

// The `mcu` object.
struct FakeMcu;

klippy_api::printer_object!(FakeMcu; dyn McuApi);

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime + 2.0
    }
    fn get_printer(&self) -> Printer {
        panic!("not used by filament_motion_sensor")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("not used by filament_motion_sensor")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("not used by filament_motion_sensor")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("not used by filament_motion_sensor")
    }
    fn request_move_queue_slot(&self) {
        panic!("not used by filament_motion_sensor")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("not used by filament_motion_sensor")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("not used by filament_motion_sensor")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("not used by filament_motion_sensor")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("not used by filament_motion_sensor")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("not used by filament_motion_sensor")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("not used by filament_motion_sensor")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("not used by filament_motion_sensor")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("not used by filament_motion_sensor")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("not used by filament_motion_sensor")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("not used by filament_motion_sensor")
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        panic!("not used by filament_motion_sensor")
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        panic!("not used by filament_motion_sensor")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("not used by filament_motion_sensor")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("not used by filament_motion_sensor")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("not used by filament_motion_sensor")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("not used by filament_motion_sensor")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("not used by filament_motion_sensor")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("not used by filament_motion_sensor")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("not used by filament_motion_sensor")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("not used by filament_motion_sensor")
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("filament_motion_sensor fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping filament_motion_sensor oracle tests: set KLIPPY_PYTHON to a \
                 CPython 3.8.2 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_value(value: &PyValue) -> Value {
    match value {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::BigInt(n.0.to_string()),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(py_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(py_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (py_value(k), py_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Int(*h as i64),
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.filament_motion_sensor".to_string(),
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

fn compare(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_ok(label: &str, expected: pyoracle::OracleResult) {
    if let Err(e) = expected {
        panic!("{label}: oracle raised {}: {}", e.class_name, e.message);
    }
}

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<PyValue>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v:?} but call must fail"),
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

fn check_err_class(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<PyValue>,
) {
    // Compare the exception class only; the message differs because the
    // Python duck-typing AttributeError names the fake's Python class
    // while the Rust contract's missing_interface names the trait.
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v:?} but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class (message differs by design)",
    );
}

// Builds one sensor on both sides with matching configurations.
// Python globals: `s` (sensor), `port_printer`, `port_config`.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    opts: &[(&str, &str)],
    detection_length: Option<f64>,
    extruder_pos: f64,
) -> (Arc<EncoderSensor>, Arc<FakePrinter>, Arc<FakeExtruder>) {
    let mut effective: Vec<(&str, &str)> =
        vec![("switch_pin", "sensor_pin"), ("extruder", "extruder1")];
    for (k, v) in opts {
        effective.retain(|(ek, _)| ek != k);
        effective.push((*k, *v));
    }
    let opts_literal: String = effective
        .iter()
        .map(|(k, v)| format!("{k:?}: {v:?}"))
        .collect::<Vec<_>>()
        .join(", ");
    let dl_stmt = match detection_length {
        Some(v) => format!("port_config.floats['detection_length'] = {v:?}; "),
        None => String::new(),
    };
    py.exec(&format!(
        "port_printer = FakePrinter(); \
         port_config = FakeConfig(port_printer, {{{opts}}}); \
         port_config.bools['pause_on_runout'] = False; {dl} \
         port_printer.objects['mcu'] = port_printer.mcu; \
         port_printer.objects['extruder1'] = port_printer.extruder; \
         port_printer.objects['stepper1'] = port_printer.extruder; \
         port_printer.extruder.pos = {extruder_pos:?}; \
         s = fms.load_config_prefix(port_config)",
        opts = opts_literal,
        dl = dl_stmt,
    ))
    .expect("py load_config_prefix");

    // Rust side: mirror the same objects.
    let (printer, _btn) = FakePrinter::new();
    let extruder = Arc::new(FakeExtruder::new(extruder_pos));
    let extruder_obj: ObjectRef = extruder.clone() as ObjectRef;
    printer.add_object("mcu", Arc::new(FakeMcu) as ObjectRef);
    printer.add_object("extruder1", extruder_obj.clone());
    printer.add_object(
        "stepper1",
        Arc::new(FakeStepperExtruder {
            pos: GCell::new(extruder_pos),
        }) as ObjectRef,
    );
    printer.add_object(
        "gcode",
        Arc::new(FakeGcode {
            commands: GCell::new(Vec::new()),
        }) as ObjectRef,
    );
    printer.add_object(
        "idle_timeout",
        Arc::new(FakeIdleTimeout::new("Ready")) as ObjectRef,
    );
    let mut opts_map = indexmap::IndexMap::new();

    for (k, v) in &effective {
        opts_map.insert(k.to_string(), v.to_string());
    }
    let mut floats = indexmap::IndexMap::new();
    if let Some(v) = detection_length {
        floats.insert("detection_length".to_string(), v);
    }
    let mut bools = indexmap::IndexMap::new();
    bools.insert("pause_on_runout".to_string(), false);
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::clone(&printer) as Printer,
        opts: opts_map,
        floats,
        bools,
    });
    let sensor = EncoderSensor::new(&config).expect("load_config_prefix");
    (sensor, printer, extruder)
}

// Rust-only construction (for error-path tests where the Python side is
// driven through separate exec strings).
fn rust_sensor(
    printer: &Arc<FakePrinter>,
    opts: &[(&str, &str)],
    detection_length: Option<f64>,
) -> Config {
    let mut opts_map = indexmap::IndexMap::new();
    for (k, v) in opts {
        opts_map.insert(k.to_string(), v.to_string());
    }
    let mut floats = indexmap::IndexMap::new();
    if let Some(v) = detection_length {
        floats.insert("detection_length".to_string(), v);
    }
    let mut bools = indexmap::IndexMap::new();
    bools.insert("pause_on_runout".to_string(), false);
    Arc::new(FakeConfig {
        printer: Arc::clone(printer) as Printer,
        opts: opts_map,
        floats,
        bools,
    })
}

#[test]
fn check_runout_timeout_value() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("fms.CHECK_RUNOUT_TIMEOUT")
        .expect("eval CHECK_RUNOUT_TIMEOUT");
    let actual = Value::float(CHECK_RUNOUT_TIMEOUT);
    compare("check_runout_timeout_value", Ok(expected), actual);
}

#[test]
fn factory_has_load_config_prefix_only() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("(hasattr(fms, 'load_config'), hasattr(fms, 'load_config_prefix'))")
        .expect("eval hasattr");
    let actual = Value::Tuple(vec![Value::Bool(false), Value::Bool(true)]);
    compare("factory_has_load_config_prefix_only", Ok(expected), actual);
    let factory = extras::filament_motion_sensor::module_factory();
    assert!(
        factory.load_config.is_none(),
        "python has no load_config"
    );
    assert!(
        factory.load_config_prefix.is_some(),
        "module root entry is load_config_prefix"
    );
}

#[test]
fn construction_reads_config_options_in_order() {
    let Some(mut py) = oracle() else { return };

    // Missing switch_pin: raised before `extruder` is read.
    let expected = py.exec(
        "p2 = FakePrinter(); p2.objects['mcu'] = p2.mcu; \
         p2.objects['extruder1'] = p2.extruder; \
         c3 = FakeConfig(p2, {}); c3.bools['pause_on_runout'] = False; \
         fms.load_config_prefix(c3)",
    );
    let (printer, _btn) = FakePrinter::new();
    let config = rust_sensor(&printer, &[], None);
    let actual = EncoderSensor::new(&config).map(|_| PyValue::None);
    check_err("construction_reads_config_options_in_order missing switch_pin", expected, actual);

    // Missing extruder (switch_pin present).
    let expected = py.exec(
        "p2 = FakePrinter(); p2.objects['mcu'] = p2.mcu; \
         p2.objects['extruder1'] = p2.extruder; \
         c3 = FakeConfig(p2, {'switch_pin': 'sensor_pin'}); c3.bools['pause_on_runout'] = False; \
         fms.load_config_prefix(c3)",
    );
    let (printer, _btn) = FakePrinter::new();
    let config = rust_sensor(&printer, &[("switch_pin", "sensor_pin")], None);
    let actual = EncoderSensor::new(&config).map(|_| PyValue::None);
    check_err("construction_reads_config_options_in_order missing extruder", expected, actual);

    // detection_length = 0 (above=0).
    let expected = py.exec(
        "p4 = FakePrinter(); p4.objects['mcu'] = p4.mcu; p4.objects['extruder1'] = p4.extruder; \
         c4 = FakeConfig(p4, {'switch_pin': 'sensor_pin', 'extruder': 'extruder1'}); \
         c4.floats['detection_length'] = 0.; c4.bools['pause_on_runout'] = False; \
         fms.load_config_prefix(c4)",
    );
    let (printer, _btn) = FakePrinter::new();
    let config =
        rust_sensor(&printer, &[("switch_pin", "sensor_pin"), ("extruder", "extruder1")], Some(0.0));
    let actual = EncoderSensor::new(&config).map(|_| PyValue::None);
    check_err(
        "construction_reads_config_options_in_order detection_length 0",
        expected,
        actual,
    );
}

#[test]
fn buttons_and_handler_registration() {
    let Some(mut py) = oracle() else { return };
    let (sensor, printer, _extruder) = fresh_pair(&mut py, &[], None, 10.0);

    // Registration order of printer event handlers.
    let expected = py
        .eval("[e for e, _ in port_printer.handlers]")
        .expect("eval handlers");
    let actual = Value::List(
        printer
            .handler_names()
            .iter()
            .map(|n| Value::str(n))
            .collect(),
    );
    compare("buttons_and_handler_registration handlers", Ok(expected), actual);

    // Buttons registered with exactly [switch_pin].
    let expected = py
        .eval("[pins for pins, _ in port_printer.buttons.regs]")
        .expect("eval regs");
    let actual = Value::List(
        printer
            .buttons_regs()
            .iter()
            .map(|pins| Value::List(pins.iter().map(|p| Value::str(p)).collect()))
            .collect(),
    );
    compare(
        "buttons_and_handler_registration regs",
        Ok(expected),
        actual,
    );

    // Buttons load is the only load_object call for this unit.
    let expected = py.eval("port_printer.load_calls").expect("eval loads");
    let actual = Value::List(
        printer
            .load_calls()
            .iter()
            .map(|n| Value::str(n))
            .collect(),
    );
    compare("buttons_and_handler_registration loads", Ok(expected), actual);

    let _ = sensor;
}

#[test]
fn ready_and_motion_flow() {
    let Some(mut py) = oracle() else { return };
    let (sensor, printer, _extruder) = fresh_pair(&mut py, &[], None, 10.0);

    // Initial status before klippy:ready.
    let expected = py.eval("s.get_status(100.)").expect("status initial");
    let actual = sensor.get_status(100.).expect("rust initial status");
    compare(
        "ready_and_motion_flow initial",
        Ok(expected),
        py_value(&actual),
    );

    // Encoder event before klippy:ready is a no-op in both.
    check_ok("ready_and_motion_flow py encoder pre-ready", py.eval("s.encoder_event(100., True)"));
    assert!(sensor.encoder_event(100., true).is_ok(), "rust encoder pre-ready");
    let expected = py.eval("s.get_status(100.)").expect("status pre");
    let actual = sensor.get_status(100.).expect("rust status pre");
    compare(
        "ready_and_motion_flow pre-ready",
        Ok(expected),
        py_value(&actual),
    );

    // Fire klippy:ready on both (Rust via the registered handler).
    check_ok(
        "ready_and_motion_flow py ready",
        py.eval("[cb() for e, cb in port_printer.handlers if e == 'klippy:ready']"),
    );
    let rets = printer.fire("klippy:ready", &[]).expect("rust ready fire");
    assert_eq!(rets.len(), 2, "runout and sensor ready handlers");

    // One timer tick: extruder pos 10 < runout_pos 17 -> filament present.
    let expected = py
        .eval("s._extruder_pos_update_event(200.)")
        .expect("timer tick");
    let actual = Value::float(sensor.extruder_pos_update_event(200.));
    compare("ready_and_motion_flow timer tick return", Ok(expected), actual);
    let expected = py.eval("s.get_status(200.)").expect("status after tick");
    let actual = sensor.get_status(200.).expect("rust status after tick");
    compare(
        "ready_and_motion_flow after tick",
        Ok(expected),
        py_value(&actual),
    );

    // idle_timeout:printing / ready / idle handlers succeed after ready.
    check_ok("ready_and_motion_flow py printing", py.eval("s._handle_printing(0.)"));
    assert!(sensor.handle_printing().is_ok(), "rust handle_printing");
    check_ok("ready_and_motion_flow py not-printing", py.eval("s._handle_not_printing(0.)"));
    assert!(sensor.handle_not_printing().is_ok(), "rust handle_not_printing");
}

#[test]
fn runout_after_motion_stops() {
    let Some(mut py) = oracle() else { return };
    let (sensor, printer, extruder) = fresh_pair(&mut py, &[], None, 10.0);
    check_ok(
        "runout py ready",
        py.eval("[cb() for e, cb in port_printer.handlers if e == 'klippy:ready']"),
    );
    printer.fire("klippy:ready", &[]).expect("rust ready");

    // Motion continues: pos 10 -> tick marks filament present.
    let _ = py.eval("s._extruder_pos_update_event(200.)");
    let _ = sensor.extruder_pos_update_event(200.);

    // Extruder passes the runout position 17 -> runout.
    py.exec("port_printer.extruder.pos = 25.").expect("move");
    extruder.pos.set(25.0);
    let expected = py
        .eval("s._extruder_pos_update_event(210.)")
        .expect("tick past");
    let actual = Value::float(sensor.extruder_pos_update_event(210.));
    compare("runout_after_motion_stops tick return", Ok(expected), actual);
    let expected = py.eval("s.get_status(210.)").expect("status");
    let actual = sensor.get_status(210.).expect("rust status");
    compare(
        "runout_after_motion_stops status",
        Ok(expected),
        py_value(&actual),
    );

    // Encoder event refreshes runout_pos to 32 -> present again.
    check_ok("runout py encoder", py.eval("s.encoder_event(220., True)"));
    assert!(sensor.encoder_event(220., true).is_ok(), "rust encoder");
    let expected = py.eval("s.get_status(220.)").expect("status after encoder");
    let actual = sensor.get_status(220.).expect("rust status after encoder");
    compare(
        "runout_after_motion_stops after encoder",
        Ok(expected),
        py_value(&actual),
    );
}

#[test]
fn custom_detection_length() {
    let Some(mut py) = oracle() else { return };
    let (sensor, printer, extruder) = fresh_pair(&mut py, &[], Some(3.0), 10.0);
    check_ok(
        "custom_dl py ready",
        py.eval("[cb() for e, cb in port_printer.handlers if e == 'klippy:ready']"),
    );
    printer.fire("klippy:ready", &[]).expect("rust ready");
    // runout_pos = 13.
    let expected = py
        .eval("s._extruder_pos_update_event(200.)")
        .expect("tick");
    let actual = Value::float(sensor.extruder_pos_update_event(200.));
    compare("custom_detection_length tick return", Ok(expected), actual);

    // Boundary: pos == runout_pos -> not present (strict <).
    py.exec("port_printer.extruder.pos = 13.").expect("pos 13");
    extruder.pos.set(13.0);
    let _ = py.eval("s._extruder_pos_update_event(210.)");
    let _ = sensor.extruder_pos_update_event(210.);
    let expected = py.eval("s.get_status(210.)").expect("status");
    let actual = sensor.get_status(210.).expect("rust status");
    compare(
        "custom_detection_length status",
        Ok(expected),
        py_value(&actual),
    );
}

#[test]
fn stepper_shaped_extruder() {
    let Some(mut py) = oracle() else { return };
    let (sensor, printer, _extruder) =
        fresh_pair(&mut py, &[("extruder", "stepper1")], None, 10.0);
    check_ok(
        "stepper py ready",
        py.eval("[cb() for e, cb in port_printer.handlers if e == 'klippy:ready']"),
    );
    printer.fire("klippy:ready", &[]).expect("rust ready");
    let expected = py
        .eval("s._extruder_pos_update_event(200.)")
        .expect("tick");
    let actual = Value::float(sensor.extruder_pos_update_event(200.));
    compare("stepper_shaped_extruder tick return", Ok(expected), actual);
}

#[test]
fn pre_ready_and_missing_object_errors() {
    let Some(mut py) = oracle() else { return };
    let (sensor, _printer, _extruder) = fresh_pair(&mut py, &[], None, 10.0);

    // get_extruder_pos before ready: estimated_print_time is None.
    let expected = py.eval("s._get_extruder_pos(100.)");
    let actual = sensor.get_extruder_pos(Some(100.)).map(PyValue::Float);
    check_err("pre_ready get_extruder_pos", expected, actual);

    // printing before ready: timer does not exist yet.
    let expected = py.eval("s._handle_printing(0.)");
    let actual = sensor.handle_printing().map(|_| PyValue::None);
    check_err("pre_ready handle_printing", expected, actual);

    // not_printing before ready: same missing timer.
    let expected = py.eval("s._handle_not_printing(0.)");
    let actual = sensor.handle_not_printing().map(|_| PyValue::None);
    check_err("pre_ready handle_not_printing", expected, actual);
}

#[test]
fn ready_missing_mcu_and_extruder_interface() {
    let Some(mut py) = oracle() else { return };

    // Missing mcu: lookup_object('mcu') raises config_error.
    let expected = py.exec(
        "p6 = FakePrinter(); p6.objects['extruder1'] = p6.extruder; \
         p6.mcu_present = False; \
         c6 = FakeConfig(p6, {'switch_pin': 'sensor_pin', 'extruder': 'extruder1'}); \
         c6.bools['pause_on_runout'] = False; \
         fms.load_config_prefix(c6)._handle_ready()",
    );
    let (printer, _btn) = FakePrinter::new();
    printer.add_object("extruder1", Arc::new(FakeExtruder::new(10.0)) as ObjectRef);
    printer.add_object(
        "gcode",
        Arc::new(FakeGcode {
            commands: GCell::new(Vec::new()),
        }) as ObjectRef,
    );
    let config = rust_sensor(&printer, &[("switch_pin", "sensor_pin"), ("extruder", "extruder1")], None);
    let sensor = EncoderSensor::new(&config).expect("construct without mcu");
    let actual = sensor.handle_ready().map(|_| PyValue::None);
    check_err("ready_missing_mcu", expected, actual);

    // Object exists but has no find_past_position: AttributeError.
    let expected = py.exec(
        "p7 = FakePrinter(); p7.objects['mcu'] = p7.mcu; p7.objects['extruder1'] = object(); \
         c7 = FakeConfig(p7, {'switch_pin': 'sensor_pin', 'extruder': 'extruder1'}); \
         c7.bools['pause_on_runout'] = False; \
         fms.load_config_prefix(c7)._handle_ready()",
    );
    let (printer, _btn) = FakePrinter::new();
    printer.add_object("mcu", Arc::new(FakeMcu) as ObjectRef);
    printer.add_object("extruder1", Arc::new(PlainObject) as ObjectRef);
    printer.add_object(
        "gcode",
        Arc::new(FakeGcode {
            commands: GCell::new(Vec::new()),
        }) as ObjectRef,
    );
    let config = rust_sensor(&printer, &[("switch_pin", "sensor_pin"), ("extruder", "extruder1")], None);
    let sensor = EncoderSensor::new(&config).expect("construct plain extruder");
    let actual = sensor.handle_ready().map(|_| PyValue::None);
    check_err_class("ready_missing_extruder_interface", expected, actual);
}
