//! Parity tests for `extras::bed_screws` (port of `klippy/extras/bed_screws.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `BedScrews` is built on
//! both sides with minimal Python / Rust fakes for the printer, the config
//! wrapper, the `gcode` object, a `gcmd` and the `toolhead`, then fed
//! identical config sections and G-code calls. Toolhead moves, G-code
//! messages, command registrations, `get_status` dicts and error classes and
//! messages are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::bed_screws::{module_factory, BedScrews};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, Kinematics, LookaheadCallback, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.bed_screws as bed_screws

class CommandError(Exception):
    pass

class sentinel:
    pass

class FakeToolhead:
    def __init__(self, fail_at=0):
        self.moves = []
        self.fail_at = fail_at
    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        if self.fail_at and len(self.moves) == self.fail_at:
            raise CommandError("toolhead manual_move failure")

class FakeGCode:
    def __init__(self):
        self.commands = {}
        self.registrations = []
        self.messages = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registrations.append((cmd, func is not None, desc))
        if func is None:
            old = self.commands.pop(cmd, None)
            return old[0] if old is not None else None
        self.commands[cmd] = (func, desc)
        return None
    def respond_info(self, msg, log=True):
        self.messages.append((msg, log))
    def create_gcmd(self):
        return FakeGCmd(self)

class FakeGCmd:
    error = CommandError
    def __init__(self, gcode):
        self.respond_info = gcode.respond_info

class FakePrinter:
    command_error = CommandError
    def __init__(self, gcode=None, toolhead=None):
        self.objects = {}
        if gcode is not None:
            self.objects['gcode'] = gcode
        if toolhead is not None:
            self.objects['toolhead'] = toolhead
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options, section='bed_screws'):
        self._printer = printer
        self.options = dict(options)
        self.section = section
    def get_printer(self):
        return self._printer
    def get(self, option, default=sentinel, note_valid=True):
        if option in self.options:
            return self.options[option]
        if default is not sentinel:
            return default
        raise configparser.Error("Option '%s' in section '%s' must be specified"
                                 % (option, self.section))
    def getfloat(self, option, default=sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        if option not in self.options:
            if default is not sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified"
                                     % (option, self.section))
        try:
            v = float(self.options[option])
        except:
            raise configparser.Error("Unable to parse option '%s' in section '%s'"
                                     % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s"
                                     % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s"
                                     % (option, self.section, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s"
                                     % (option, self.section, above))
        if below is not None and v >= below:
            raise configparser.Error("Option '%s' in section '%s' must be below %s"
                                     % (option, self.section, below))
        return v
    def getfloatlist(self, option, default=sentinel, sep=',', count=None,
                     note_valid=True):
        if option not in self.options:
            if default is not sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified"
                                     % (option, self.section))
        value = self.options[option]
        if len(value.strip()) == 0:
            parts = []
        else:
            parts = [p.strip() for p in value.split(sep)]
        try:
            res = [float(p) for p in parts]
        except:
            raise configparser.Error("Unable to parse option '%s' in section '%s'"
                                     % (option, self.section))
        if count is not None and len(res) != count:
            raise configparser.Error("Option '%s' in section '%s' must have %d elements"
                                     % (option, self.section, count))
        return tuple(res)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct FailAt(Option<i64>);

struct FakeToolHead {
    moves: GCell<Vec<(Vec<Option<f64>>, f64)>>,
    fail_at: FailAt,
}

impl FakeToolHead {
    fn new(fail_at: i64) -> FakeToolHead {
        FakeToolHead {
            moves: GCell::new(Vec::new()),
            fail_at: FailAt(if fail_at > 0 { Some(fail_at) } else { None }),
        }
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by bed_screws")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by bed_screws")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by bed_screws")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by bed_screws")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        panic!("get_last_move_time is not used by bed_screws")
    }
    fn dwell(&self, _delay: f64) -> KResult<()> {
        panic!("dwell is not used by bed_screws")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by bed_screws")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by bed_screws")
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        let mut moves = self.moves.borrow_mut();
        moves.push((coord, speed));
        if self.fail_at.0 == Some(moves.len() as i64) {
            return Err(KlippyError::command("toolhead manual_move failure"));
        }
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> {
        panic!("wait_moves is not used by bed_screws")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by bed_screws")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by bed_screws")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by bed_screws")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by bed_screws")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by bed_screws")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by bed_screws")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by bed_screws")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by bed_screws")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by bed_screws")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis is not used by bed_screws")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis is not used by bed_screws")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by bed_screws")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by bed_screws")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by bed_screws")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct Logs {
    registrations: GCell<Vec<(String, bool, Option<String>)>>,
    messages: GCell<Vec<(String, bool)>>,
}

struct FakeGCode {
    logs: Arc<Logs>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            logs: Arc::new(Logs {
                registrations: GCell::new(Vec::new()),
                messages: GCell::new(Vec::new()),
            }),
        }
    }
    fn registrations(&self) -> Vec<(String, bool, Option<String>)> {
        self.logs.registrations.cloned()
    }
    fn messages(&self) -> Vec<(String, bool)> {
        self.logs.messages.cloned()
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.logs
            .registrations
            .borrow_mut()
            .push((cmd.to_string(), func.is_some(), opts.desc.clone()));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        panic!("register_mux_command is not used by bed_screws")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used by bed_screws")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by bed_screws")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("register_output_handler is not used by bed_screws")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by bed_screws")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by bed_screws")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by bed_screws")
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("get_mutex is not used by bed_screws")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd { logs: Arc::clone(&self.logs) })
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by bed_screws")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.logs.messages.borrow_mut().push((msg.to_string(), log));
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by bed_screws")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by bed_screws")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by bed_screws")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCmd {
    logs: Arc<Logs>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        panic!("gcmd.get is not used by bed_screws")
    }
    fn get_int(&self, _name: &str, _opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        panic!("gcmd.get_int is not used by bed_screws")
    }
    fn get_float(
        &self,
        _name: &str,
        _opts: klippy_api::gcode::GetFloatOpts,
    ) -> KResult<Option<f64>> {
        panic!("gcmd.get_float is not used by bed_screws")
    }
    fn get_command(&self) -> String {
        panic!("gcmd.get_command is not used by bed_screws")
    }
    fn get_commandline(&self) -> String {
        panic!("gcmd.get_commandline is not used by bed_screws")
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        panic!("gcmd.get_command_parameters is not used by bed_screws")
    }
    fn get_raw_command_parameters(&self) -> String {
        panic!("gcmd.get_raw_command_parameters is not used by bed_screws")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.logs.messages.borrow_mut().push((msg.to_string(), log));
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("gcmd.respond_raw is not used by bed_screws")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        panic!("gcmd.ack is not used by bed_screws")
    }
}

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter { objects: GCell::new(IndexMap::new()) }
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
        let mut objects = self.objects.borrow_mut();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.borrow();
        if let Some(obj) = objects.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.borrow();
        objects
            .iter()
            .filter(|(name, _)| module.is_none_or(|m| name.as_str() == m || name.starts_with(&format!("{m} "))))
            .map(|(name, obj)| (name.clone(), obj.clone()))
            .collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by bed_screws")
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfig {
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
}

fn float_repr(v: f64) -> String {
    pycompat::text::float_repr(v)
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(value) = self.options.get(option) {
            return Ok(Some(value.clone()));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
            Some(value) => Ok(value),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let raw = match self.options.get(option) {
            Some(value) => value.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                    Some(value) => Ok(value),
                };
            }
        };
        raw.parse::<i64>().map(Some).map_err(|_| {
            KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'",
                self.section
            ))
        })
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.options.get(option) {
            Some(value) => value.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                    Some(value) => Ok(value),
                };
            }
        };
        let value: f64 = raw.trim().parse().map_err(|_| {
            KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'",
                self.section
            ))
        })?;
        if let Some(minval) = opts.minval {
            if value < minval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    float_repr(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if value > maxval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    float_repr(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if value <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if value >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.section,
                    float_repr(below)
                )));
            }
        }
        Ok(Some(value))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by bed_screws")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by bed_screws")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by bed_screws")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by bed_screws")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by bed_screws")
    }
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let raw = match self.options.get(option) {
            Some(value) => value.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                    Some(value) => Ok(value),
                };
            }
        };
        let parts: Vec<&str> = if raw.trim().is_empty() {
            Vec::new()
        } else {
            raw.split(&opts.sep).map(str::trim).collect()
        };
        let values: Vec<f64> = parts
            .iter()
            .map(|p| {
                p.parse::<f64>().map_err(|_| {
                    KlippyError::config(format!(
                        "Unable to parse option '{option}' in section '{}'",
                        self.section
                    ))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        if let Some(count) = opts.count {
            if values.len() != count {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.section
                )));
            }
        }
        Ok(Some(values))
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by bed_screws")
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

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("bed_screws fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping bed_screws oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

struct RustWorld {
    bed_screws: KResult<Arc<BedScrews>>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    config: Config,
    gcmd: GCmd,
}

fn build_rust(
    options: &[(&str, &str)],
    fail_at: i64,
    with_gcode: bool,
    with_toolhead: bool,
) -> RustWorld {
    let gcode = Arc::new(FakeGCode::new());
    let toolhead = Arc::new(FakeToolHead::new(fail_at));
    let printer: Printer = Arc::new(FakePrinter::new());
    if with_gcode {
        printer
            .add_object("gcode", gcode.clone() as ObjectRef)
            .expect("add gcode object");
    }
    if with_toolhead {
        printer
            .add_object("toolhead", toolhead.clone() as ObjectRef)
            .expect("add toolhead object");
    }
    let map: IndexMap<String, String> = options
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        section: "bed_screws".to_string(),
        options: map,
    });
    let gcmd = gcode.create_gcode_command("BED_SCREWS_ADJUST", "BED_SCREWS_ADJUST", &IndexMap::new());
    let bed_screws = BedScrews::new(&config);
    RustWorld { bed_screws, gcode, toolhead, config, gcmd }
}

fn dict_literal(options: &[(&str, &str)]) -> String {
    let pairs: Vec<String> = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", pairs.join(", "))
}

fn build_py(
    py: &mut pyoracle::Oracle,
    options: &[(&str, &str)],
    fail_at: i64,
    with_gcode: bool,
    with_toolhead: bool,
) -> pyoracle::OracleResult {
    let gcode_expr = if with_gcode { "g" } else { "None" };
    let toolhead_expr = if with_toolhead { "t" } else { "None" };
    let code = format!(
        "g = FakeGCode()\nt = FakeToolhead(fail_at={fail_at})\np = FakePrinter({gcode_expr}, {toolhead_expr})\
         \nc = FakeConfig(p, {})\nbs = bed_screws.load_config(c)\ngc = g.create_gcmd()",
        dict_literal(options)
    );
    py.exec(&code)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "bed_screws".to_string(),
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

fn check_call(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match expected {
        Ok(v) => {
            if let Err(e) = actual {
                panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
            }
            check(label, Ok(v), Value::Null);
        }
        Err(e) => {
            let rust_err = match actual {
                Ok(()) => panic!("{label}: rust returned Ok but call must fail"),
                Err(e) => e,
            };
            assert_eq!(
                rust_err.class_name(),
                e.class_name.as_str(),
                "{label}: exception class"
            );
            assert_eq!(rust_err.message(), e.message.as_str(), "{label}: exception message");
        }
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect(),
        ),
        PyValue::Host(_) => Value::Null,
    }
}

fn moves_value(moves: &[(Vec<Option<f64>>, f64)]) -> Value {
    Value::List(
        moves
            .iter()
            .map(|(coord, speed)| {
                Value::Tuple(vec![
                    Value::List(
                        coord
                            .iter()
                            .map(|c| match c {
                                Some(f) => Value::float(*f),
                                None => Value::Null,
                            })
                            .collect(),
                    ),
                    Value::float(*speed),
                ])
            })
            .collect(),
    )
}

fn messages_value(messages: &[(String, bool)]) -> Value {
    Value::List(
        messages
            .iter()
            .map(|(msg, log)| Value::Tuple(vec![Value::str(msg), Value::Bool(*log)]))
            .collect(),
    )
}

fn registrations_value(registrations: &[(String, bool, Option<String>)]) -> Value {
    Value::List(
        registrations
            .iter()
            .map(|(cmd, registered, desc)| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    Value::Bool(*registered),
                    match desc {
                        Some(d) => Value::str(d),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

struct Harness {
    rust: Arc<BedScrews>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    gcmd: GCmd,
}

fn harness(
    py: &mut pyoracle::Oracle,
    options: &[(&str, &str)],
    fail_at: i64,
) -> Harness {
    let built = build_py(py, options, fail_at, true, true);
    let world = build_rust(options, fail_at, true, true);
    match (built, world.bed_screws) {
        (Ok(_), Ok(rust)) => Harness {
            rust,
            gcode: world.gcode,
            toolhead: world.toolhead,
            gcmd: world.gcmd,
        },
        (Err(e), Err(rust_err)) => {
            panic!(
                "unexpected build failure: oracle {}: {}, rust {}: {}",
                e.class_name,
                e.message,
                rust_err.class_name(),
                rust_err.message()
            )
        }
        (Ok(_), Err(e)) => {
            panic!("rust build failed but oracle built: {}: {}", e.class_name(), e.message())
        }
        (Err(e), Ok(_)) => panic!("oracle build failed but rust built: {}: {}", e.class_name, e.message),
    }
}

fn check_status(py: &mut pyoracle::Oracle, harness: &Harness, label: &str) {
    let expected = py.eval("bs.get_status(0)");
    check(label, expected, pyvalue_to_value(&harness.rust.get_status(0.0)));
}

fn check_moves(py: &mut pyoracle::Oracle, harness: &Harness, label: &str) {
    let expected = py.eval("t.moves");
    check(label, expected, moves_value(&harness.toolhead.moves.borrow()));
}

fn check_messages(py: &mut pyoracle::Oracle, harness: &Harness, label: &str) {
    let expected = py.eval("g.messages");
    check(label, expected, messages_value(&harness.gcode.messages()));
}

fn check_registrations(py: &mut pyoracle::Oracle, harness: &Harness, label: &str) {
    let expected = py.eval("g.registrations");
    check(label, expected, registrations_value(&harness.gcode.registrations()));
}

fn check_side_effects(py: &mut pyoracle::Oracle, harness: &Harness, label: &str) {
    check_status(py, harness, &format!("{label} status"));
    check_moves(py, harness, &format!("{label} moves"));
    check_messages(py, harness, &format!("{label} messages"));
    check_registrations(py, harness, &format!("{label} registrations"));
}

const BASE: &[(&str, &str)] = &[
    ("screw1", "30,30"),
    ("screw2", "200,30"),
    ("screw3", "30,200"),
];

const CUSTOM: &[(&str, &str)] = &[
    ("screw1", "10.1234,20.9999"),
    ("screw1_name", "Front left"),
    ("screw2", "200.5,30.25"),
    ("screw3", "30,200"),
    ("screw3_name", "Back left"),
    ("speed", "80"),
    ("probe_speed", "7"),
    ("horizontal_move_z", "12"),
    ("probe_height", "1.5"),
];

const FINE: &[(&str, &str)] = &[
    ("screw1", "30,30"),
    ("screw1_fine_adjust", "31,31"),
    ("screw2", "200,30"),
    ("screw2_fine_adjust", "199,29"),
    ("screw3", "30,200"),
    ("screw3_fine_adjust", "29,199"),
];

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn initial_status_and_registration() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    check_side_effects(&mut py, &h, "initial");
}

#[test]
fn adjust_moves_to_first_screw_with_defaults() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    let actual = h.rust.cmd_bed_screws_adjust(&h.gcmd);
    check_call("adjust", expected, actual);
    check_side_effects(&mut py, &h, "after adjust");
}

#[test]
fn accept_advances_through_screws_and_completes() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    for n in 1..=3 {
        let expected = py.eval("bs.cmd_ACCEPT(gc)");
        let actual = h.rust.cmd_accept(&h.gcmd);
        check_call(&format!("accept {n}"), expected, actual);
        check_side_effects(&mut py, &h, &format!("after accept {n}"));
    }
}

#[test]
fn accept_on_last_screw_completes_tool() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    for n in 1..3 {
        let expected = py.eval("bs.cmd_ACCEPT(gc)");
        check_call(&format!("accept {n}"), expected, h.rust.cmd_accept(&h.gcmd));
    }
    let expected = py.eval("bs.cmd_ACCEPT(gc)");
    check_call("accept last", expected, h.rust.cmd_accept(&h.gcmd));
    check_side_effects(&mut py, &h, "completed");
    let expected = py.eval("bs.get_status(0)");
    check(
        "completed status is_active",
        expected,
        pyvalue_to_value(&h.rust.get_status(0.0)),
    );
}

#[test]
fn abort_resets_and_adjust_can_restart() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    let expected = py.eval("bs.cmd_ACCEPT(gc)");
    check_call("accept", expected, h.rust.cmd_accept(&h.gcmd));
    let expected = py.eval("bs.cmd_ABORT(gc)");
    let actual = h.rust.cmd_abort(&h.gcmd);
    check_call("abort", expected, actual);
    check_side_effects(&mut py, &h, "after abort");
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust again", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    check_side_effects(&mut py, &h, "after re-adjust");
}

#[test]
fn adjusted_does_not_count_current_screw() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    let expected = py.eval("bs.cmd_ADJUSTED(gc)");
    let actual = h.rust.cmd_adjusted(&h.gcmd);
    check_call("adjusted", expected, actual);
    check_side_effects(&mut py, &h, "after adjusted");
}

#[test]
fn adjusted_on_last_screw_retries_coarse() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    for n in 1..3 {
        let expected = py.eval("bs.cmd_ACCEPT(gc)");
        check_call(&format!("accept {n}"), expected, h.rust.cmd_accept(&h.gcmd));
    }
    let expected = py.eval("bs.cmd_ADJUSTED(gc)");
    check_call("adjusted last", expected, h.rust.cmd_adjusted(&h.gcmd));
    check_side_effects(&mut py, &h, "after adjusted last");
    for n in 1..=3 {
        let expected = py.eval("bs.cmd_ACCEPT(gc)");
        check_call(&format!("accept after retry {n}"), expected, h.rust.cmd_accept(&h.gcmd));
    }
    check_side_effects(&mut py, &h, "completed after retry");
}

#[test]
fn already_active_raises_command_error() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, BASE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    let actual = h.rust.cmd_bed_screws_adjust(&h.gcmd);
    check_call("adjust while active", expected, actual);
    check_side_effects(&mut py, &h, "after failed re-adjust");
}

#[test]
fn move_failure_unregisters_resets_and_reraises() {
    let Some(mut py) = oracle() else { return };
    // Initial adjust does 4 moves (1 up + 3 in move_to_screw); the next
    // accept move_to_screw does 3 more. Fail on the 6th move overall: the
    // "over" move of the second screw.
    let h = harness(&mut py, BASE, 6);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    let expected = py.eval("bs.cmd_ACCEPT(gc)");
    let actual = h.rust.cmd_accept(&h.gcmd);
    check_call("accept with failing move", expected, actual);
    check_side_effects(&mut py, &h, "after failure");
}

#[test]
fn custom_config_values_and_names_drive_moves() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, CUSTOM, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    check_side_effects(&mut py, &h, "after adjust custom");
    let expected = py.eval("bs.cmd_ACCEPT(gc)");
    check_call("accept custom", expected, h.rust.cmd_accept(&h.gcmd));
    check_side_effects(&mut py, &h, "after accept custom");
}

#[test]
fn fine_adjust_pass_runs_after_coarse() {
    let Some(mut py) = oracle() else { return };
    let h = harness(&mut py, FINE, 0);
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust", expected, h.rust.cmd_bed_screws_adjust(&h.gcmd));
    for n in 1..=3 {
        let expected = py.eval("bs.cmd_ACCEPT(gc)");
        check_call(&format!("coarse accept {n}"), expected, h.rust.cmd_accept(&h.gcmd));
    }
    check_side_effects(&mut py, &h, "after coarse pass");
    for n in 1..=3 {
        let expected = py.eval("bs.cmd_ACCEPT(gc)");
        check_call(&format!("fine accept {n}"), expected, h.rust.cmd_accept(&h.gcmd));
    }
    check_side_effects(&mut py, &h, "after fine pass");
}

#[test]
fn fewer_than_three_screws_is_a_config_error() {
    let Some(mut py) = oracle() else { return };
    let options = [("screw1", "30,30"), ("screw2", "200,30")];
    let py_result = build_py(&mut py, &options, 0, true, true);
    let world = build_rust(&options, 0, true, true);
    match (py_result, world.bed_screws) {
        (Err(e), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), e.class_name.as_str(), "exception class");
            assert_eq!(rust_err.message(), e.message.as_str(), "exception message");
        }
        (Err(e), Ok(_)) => panic!("rust built but oracle failed: {}: {}", e.class_name, e.message),
        (Ok(_), Err(rust_err)) => {
            panic!("oracle built but rust failed: {}: {}", rust_err.class_name(), rust_err.message())
        }
        (Ok(_), Ok(_)) => panic!("both built but two screws must fail"),
    }
}

#[test]
fn speed_must_be_above_zero() {
    let Some(mut py) = oracle() else { return };
    let options = [
        ("screw1", "30,30"),
        ("screw2", "200,30"),
        ("screw3", "30,200"),
        ("speed", "0"),
    ];
    let py_result = build_py(&mut py, &options, 0, true, true);
    let world = build_rust(&options, 0, true, true);
    match (py_result, world.bed_screws) {
        (Err(e), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), e.class_name.as_str(), "exception class");
            assert_eq!(rust_err.message(), e.message.as_str(), "exception message");
        }
        (Err(e), Ok(_)) => panic!("rust built but oracle failed: {}: {}", e.class_name, e.message),
        (Ok(_), Err(rust_err)) => {
            panic!("oracle built but rust failed: {}: {}", rust_err.class_name(), rust_err.message())
        }
        (Ok(_), Ok(_)) => panic!("both built but speed=0 must fail"),
    }
}

#[test]
fn screw_coordinate_count_is_enforced() {
    let Some(mut py) = oracle() else { return };
    let options = [
        ("screw1", "30,30"),
        ("screw2", "200"),
        ("screw3", "30,200"),
    ];
    let py_result = build_py(&mut py, &options, 0, true, true);
    let world = build_rust(&options, 0, true, true);
    match (py_result, world.bed_screws) {
        (Err(e), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), e.class_name.as_str(), "exception class");
            assert_eq!(rust_err.message(), e.message.as_str(), "exception message");
        }
        (Err(e), Ok(_)) => panic!("rust built but oracle failed: {}: {}", e.class_name, e.message),
        (Ok(_), Err(rust_err)) => {
            panic!("oracle built but rust failed: {}: {}", rust_err.class_name(), rust_err.message())
        }
        (Ok(_), Ok(_)) => panic!("both built but a one-element screw must fail"),
    }
}

#[test]
fn unparseable_screw_coordinate_is_a_config_error() {
    let Some(mut py) = oracle() else { return };
    let options = [
        ("screw1", "30,30"),
        ("screw2", "west,30"),
        ("screw3", "30,200"),
    ];
    let py_result = build_py(&mut py, &options, 0, true, true);
    let world = build_rust(&options, 0, true, true);
    match (py_result, world.bed_screws) {
        (Err(e), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), e.class_name.as_str(), "exception class");
            assert_eq!(rust_err.message(), e.message.as_str(), "exception message");
        }
        (Err(e), Ok(_)) => panic!("rust built but oracle failed: {}: {}", e.class_name, e.message),
        (Ok(_), Err(rust_err)) => {
            panic!("oracle built but rust failed: {}: {}", rust_err.class_name(), rust_err.message())
        }
        (Ok(_), Ok(_)) => panic!("both built but a non-float coordinate must fail"),
    }
}

#[test]
fn missing_gcode_object_is_a_config_error() {
    let Some(mut py) = oracle() else { return };
    let py_result = build_py(&mut py, BASE, 0, false, true);
    let world = build_rust(BASE, 0, false, true);
    match (py_result, world.bed_screws) {
        (Err(e), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), e.class_name.as_str(), "exception class");
            assert_eq!(rust_err.message(), e.message.as_str(), "exception message");
        }
        (Err(e), Ok(_)) => panic!("rust built but oracle failed: {}: {}", e.class_name, e.message),
        (Ok(_), Err(rust_err)) => {
            panic!("oracle built but rust failed: {}: {}", rust_err.class_name(), rust_err.message())
        }
        (Ok(_), Ok(_)) => panic!("both built but missing gcode must fail"),
    }
}

#[test]
fn missing_toolhead_object_fails_first_move() {
    let Some(mut py) = oracle() else { return };
    let py_result = build_py(&mut py, BASE, 0, true, false);
    let world = build_rust(BASE, 0, true, false);
    let rust = match (py_result, world.bed_screws) {
        (Ok(_), Ok(rust)) => rust,
        (Err(e), _) => panic!("oracle build failed: {}: {}", e.class_name, e.message),
        (_, Err(e)) => panic!("rust build failed: {}: {}", e.class_name(), e.message()),
    };
    let expected = py.eval("bs.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("adjust without toolhead", expected, rust.cmd_bed_screws_adjust(&world.gcmd));
    let expected = py.eval("bs.get_status(0)");
    check(
        "status after toolhead lookup failure",
        expected,
        pyvalue_to_value(&rust.get_status(0.0)),
    );
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory.load_config.expect("bed_screws exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "bed_screws has no load_config_prefix");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "bed_screws has no add_early_printer_objects"
    );
    assert!(factory.add_printer_objects.is_none(), "bed_screws has no add_printer_objects");
    let world = build_rust(BASE, 0, true, true);
    let obj: ObjectRef = load_config(&world.config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let py_built = build_py(&mut py, BASE, 0, true, true);
    match py_built {
        Ok(_) => {}
        Err(e) => panic!("py build for factory test: {}: {}", e.class_name, e.message),
    }
    let rust: Arc<BedScrews> = downcast_arc(&obj).expect("downcast to BedScrews");
    py.exec("c2 = FakeConfig(p, {'screw1': '30,30', 'screw2': '200,30', 'screw3': '30,200'})\nbs2 = bed_screws.load_config(c2)")
        .expect("py factory load_config");
    let expected = py.eval("bs2.get_status(0)");
    check("factory status", expected, pyvalue_to_value(&rust.get_status(0.0)));
    let expected = py.eval("bs2.cmd_BED_SCREWS_ADJUST(gc)");
    check_call("factory adjust", expected, rust.cmd_bed_screws_adjust(&world.gcmd));
    let expected = py.eval("bs2.get_status(0)");
    check("factory status active", expected, pyvalue_to_value(&rust.get_status(0.0)));
}
