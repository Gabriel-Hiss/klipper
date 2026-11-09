//! Parity tests for `extras::homing_override` (port of
//! `klippy/extras/homing_override.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a Python `HomingOverride` is
//! built on inert fakes for the printer, config wrapper, `gcode_macro`,
//! `toolhead` and `gcode`, and driven with the same `G28` parameter sets
//! as the Rust port through the same fakes in Rust. Compared exactly:
//! returned values (floats by bits), raised errors (class and message),
//! `toolhead.set_position` arguments, template-run contexts (`params`,
//! `rawparams`), and the `G28` handler registration chain.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::gcode_macro::PrinterGCodeMacro;
use extras::homing_override::HomingOverride;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListOpts,
    GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCmd, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::Trapq;
use klippy_api::toolhead::{ExtraAxis, Kinematics, PrinterExtruder, ToolHeadApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value as RecordingValue};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.homing_override as homing_override
import extras.gcode_macro as gcode_macro

class CommandError(Exception):
    pass

# `printer.command_error` is `gcode.CommandError` (`klippy/klippy.py:27`).
CommandError.__module__ = "gcode"

_sentinel = object()

DEFAULT_SCRIPT = 'G28 X=0 Z=1\nM400{rawparams}'

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.loaded = []
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_sentinel):
        self.loaded.append(section)
        if section in self.objects:
            return self.objects[section]
        raise self.config_error("Unable to load module '%s'" % (section,))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, opts=None, name='homing_override'):
        self._printer = printer
        self._opts = opts if opts is not None else {}
        self._name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=_sentinel):
        if option in self._opts:
            return self._opts[option]
        if default is not _sentinel:
            return default
        raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
    def getfloat(self, option, default=_sentinel):
        if option in self._opts:
            return float(self._opts[option])
        if default is not _sentinel:
            return default
        raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))

class FakeGcmd:
    def __init__(self, params, rawparams):
        self._params = dict(params)
        self._raw = rawparams
    def get(self, name, default=None):
        return self._params.get(name, default)
    def get_command_parameters(self):
        return dict(self._params)
    def get_raw_command_parameters(self):
        return self._raw

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.handlers = {}
        self.calls = []
        self.scripts = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.calls.append((cmd, func is None))
        if func is None:
            return self.handlers.pop(cmd, None)
        if cmd in self.handlers:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = func
        return None
    def run_script_from_command(self, script):
        self.scripts.append(script)
        for line in script.split('\n'):
            if line != 'G28' and not line.startswith('G28 '):
                continue
            params = {}
            for tok in line.split()[1:]:
                k, _, v = tok.partition('=')
                params[k] = v
            self.handlers['G28'](FakeGcmd(params, line[3:]))

class FakeToolhead:
    def __init__(self, pos):
        self.pos = list(pos)
        self.set_calls = []
    def get_position(self):
        return list(self.pos)
    def set_position(self, newpos, homing_axes=""):
        self.set_calls.append((list(newpos), homing_axes))
        self.pos = list(newpos)

class FakeHoming:
    pass

def build(opts, pos, prev, omit):
    printer = FakePrinter()
    gcode = FakeGCode()
    toolhead = FakeToolhead(pos)
    prev_calls = []
    if prev:
        def prev_g28(gcmd):
            params = gcmd.get_command_parameters()
            prev_calls.append(dict(params))
            if 'FAIL' in params:
                raise CommandError('prev failed')
        gcode.handlers['G28'] = prev_g28
    if 'gcode' not in omit:
        printer.objects['gcode'] = gcode
    if 'gcode_macro' not in omit:
        printer.objects['gcode_macro'] = gcode_macro.load_config(
            FakeConfig(printer, {}, 'gcode_macro'))
    if 'homing' not in omit:
        printer.objects['homing'] = FakeHoming()
    if 'toolhead' not in omit:
        printer.objects['toolhead'] = toolhead
    options = {'gcode': DEFAULT_SCRIPT}
    options.update(opts)
    return printer, gcode, toolhead, prev_calls, FakeConfig(printer, options)

def err_of(e):
    return (type(e).__name__, str(e))

def run(opts, calls, pos, prev, omit):
    printer, gcode, toolhead, prev_calls, config = build(opts, pos, prev, omit)
    try:
        ho = homing_override.load_config(config)
    except BaseException as e:
        return {'build_err': err_of(e), 'loaded': list(printer.loaded)}
    errs = []
    for params, raw in calls:
        try:
            ho.cmd_G28(FakeGcmd(params, raw))
            errs.append(None)
        except BaseException as e:
            errs.append(err_of(e))
    return {
        'build_err': None,
        'errs': errs,
        'set_calls': [([float(x) for x in p], h) for p, h in toolhead.set_calls],
        'scripts': list(gcode.scripts),
        'prev': prev_calls,
        'reg_calls': list(gcode.calls),
        'loaded': list(printer.loaded),
        'start_pos': list(ho.start_pos),
        'axes': ho.axes,
    }

def via_handler(opts, params, raw):
    printer, gcode, toolhead, prev_calls, config = build(opts, [1.0, 2.0, 3.0, 4.0], True, ())
    homing_override.load_config(config)
    try:
        gcode.handlers['G28'](FakeGcmd(params, raw))
        err = None
    except BaseException as e:
        err = err_of(e)
    return {
        'err': err,
        'set_calls': [([float(x) for x in p], h) for p, h in toolhead.set_calls],
        'scripts': list(gcode.scripts),
        'prev': prev_calls,
        'reg_calls': list(gcode.calls),
        'loaded': list(printer.loaded),
    }
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    loaded: GCell<Vec<String>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: GCell::new(IndexMap::new()),
            loaded: GCell::new(Vec::new()),
        }
    }
    fn register(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }
    fn loaded_sections(&self) -> Vec<String> {
        self.loaded.borrow().clone()
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
        self.register(name, obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.borrow().get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(klippy_api::KlippyError::Config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
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
        self.loaded.borrow_mut().push(section.to_string());
        if let Some(obj) = self.objects.borrow().get(section).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(klippy_api::KlippyError::Config(format!(
                "Unable to load module '{section}'"
            ))),
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

struct FakeConfig {
    printer: Printer,
    opts: IndexMap<String, String>,
    name: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.opts.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::Config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        if let Some(v) = self.opts.get(option) {
            return Ok(Some(v.parse().map_err(|_| {
                klippy_api::KlippyError::Config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                ))
            })?));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::Config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by homing_override")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by homing_override")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by homing_override")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by homing_override")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by homing_override")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by homing_override")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by homing_override")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by homing_override")
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

struct FakeGCode {
    handlers: GCell<IndexMap<String, GCodeCommandHandler>>,
    calls: GCell<Vec<(String, bool)>>,
    scripts: GCell<Vec<String>>,
    route: GCell<Option<Arc<HomingOverride>>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            handlers: GCell::new(IndexMap::new()),
            calls: GCell::new(Vec::new()),
            scripts: GCell::new(Vec::new()),
            route: GCell::new(None),
        }
    }
    /// Seeds a handler without recording a registration call.
    fn install(&self, cmd: &str, handler: GCodeCommandHandler) {
        self.handlers.borrow_mut().insert(cmd.to_string(), handler);
    }
    fn take_handler(&self, cmd: &str) -> Option<GCodeCommandHandler> {
        self.handlers.borrow_mut().shift_remove(cmd)
    }
    fn set_route(&self, ho: Arc<HomingOverride>) {
        self.route.replace(Some(ho));
    }
    fn calls_snapshot(&self) -> Vec<(String, bool)> {
        self.calls.borrow().clone()
    }
    fn scripts_snapshot(&self) -> Vec<String> {
        self.scripts.borrow().clone()
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.calls.borrow_mut().push((cmd.to_string(), func.is_none()));
        match func {
            None => Ok(self.handlers.borrow_mut().shift_remove(cmd)),
            Some(handler) => {
                if self.handlers.borrow().contains_key(cmd) {
                    return Err(klippy_api::KlippyError::Config(format!(
                        "gcode command {cmd} already registered"
                    )));
                }
                self.handlers.borrow_mut().insert(cmd.to_string(), handler);
                Ok(None)
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
        panic!("register_mux_command is not used by homing_override")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by homing_override")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by homing_override")
    }
    /// Records the script and dispatches each `G28` line to the
    /// override, which is the handler the registration made current. The
    /// route stands in for looking the handler up again, because the
    /// handler of a running dispatch cannot be re-entered through its
    /// `FnMut` box.
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.scripts.borrow_mut().push(script.to_string());
        for line in script.split('\n') {
            if line != "G28" && !line.starts_with("G28 ") {
                continue;
            }
            let mut params = IndexMap::new();
            for tok in line.split_whitespace().skip(1) {
                match tok.split_once('=') {
                    Some((k, v)) => params.insert(k.to_string(), v.to_string()),
                    None => params.insert(tok.to_string(), String::new()),
                };
            }
            let ho = self.route.cloned().expect("a G28 route is installed");
            let gcmd: GCmd = Arc::new(FakeGcmd {
                params,
                rawparams: line[3..].to_string(),
                commandline: line.to_string(),
            });
            ho.cmd_g28(&gcmd)?;
        }
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used by homing_override")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by homing_override")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by homing_override")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by homing_override")
    }
}

struct FakeToolhead {
    pos: GCell<Vec<f64>>,
    set_calls: GCell<Vec<(Vec<f64>, String)>>,
}

impl FakeToolhead {
    fn new(pos: Vec<f64>) -> FakeToolhead {
        FakeToolhead {
            pos: GCell::new(pos),
            set_calls: GCell::new(Vec::new()),
        }
    }
    fn set_calls_snapshot(&self) -> Vec<(Vec<f64>, String)> {
        self.set_calls.borrow().clone()
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.pos.borrow().clone()
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.set_calls
            .borrow_mut()
            .push((newpos.clone(), homing_axes.to_string()));
        *self.pos.borrow_mut() = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by homing_override")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        0.0
    }
    fn dwell(&self, _delay: f64) {}
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<f64>,
    ) -> KResult<()> {
        panic!("drip_move is not used by homing_override")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by homing_override")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by homing_override")
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing_override")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by homing_override")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by homing_override")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by homing_override")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback is not used by homing_override")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by homing_override")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by homing_override")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by homing_override")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by homing_override")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used by homing_override")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by homing_override")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by homing_override")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by homing_override")
    }
}

struct FakeHoming;

klippy_api::printer_object!(FakeHoming);

struct FakeGcmd {
    params: IndexMap<String, String>,
    rawparams: String,
    commandline: String,
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        if let Some(v) = self.params.get(name) {
            return Ok(PyValue::Str(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::Command(format!(
                "Error on '{}': missing {name}",
                self.commandline
            ))),
            Some(d) => Ok(d),
        }
    }
    fn get_int(&self, _name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int is not used by homing_override")
    }
    fn get_float(&self, _name: &str, _opts: GGetFloatOpts) -> KResult<Option<f64>> {
        panic!("get_float is not used by homing_override")
    }
    fn get_command(&self) -> String {
        "G28".to_string()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.rawparams.clone()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("homing_override fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping homing_override oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

const DEFAULT_SCRIPT: &str = "G28 X=0 Z=1\nM400{rawparams}";
const POS: [f64; 4] = [1.0, 2.0, 3.0, 4.0];

type Opts<'a> = &'a [(&'a str, &'a str)];
type Call<'a> = (&'a [(&'a str, &'a str)], &'a str);

struct World {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    prev_calls: Arc<GCell<Vec<IndexMap<String, String>>>>,
    config: Config,
}

/// The Rust twin of the Python `build` function: the real `gcode_macro`
/// object, fakes for `gcode`, `homing` and `toolhead`, and a config whose
/// `gcode` option defaults to `DEFAULT_SCRIPT`.
fn world(opts: Opts, pos: &[f64], with_prev: bool, omit: &[&str]) -> World {
    let printer = Arc::new(FakePrinter::new());
    let gcode = Arc::new(FakeGCode::new());
    let toolhead = Arc::new(FakeToolhead::new(pos.to_vec()));
    let prev_calls: Arc<GCell<Vec<IndexMap<String, String>>>> = Arc::new(GCell::new(Vec::new()));
    if with_prev {
        let calls = prev_calls.clone();
        gcode.install(
            "G28",
            Box::new(move |gcmd: &GCmd| {
                let params = gcmd.get_command_parameters();
                let fail = params.contains_key("FAIL");
                calls.borrow_mut().push(params);
                if fail {
                    return Err(klippy_api::KlippyError::Command("prev failed".to_string()));
                }
                Ok(())
            }),
        );
    }
    let printer_handle: Printer = printer.clone();
    if !omit.contains(&"gcode") {
        printer.register("gcode", gcode.clone() as ObjectRef);
    }
    if !omit.contains(&"gcode_macro") {
        let macro_config: Config = Arc::new(FakeConfig {
            printer: printer_handle.clone(),
            opts: IndexMap::new(),
            name: "gcode_macro".to_string(),
        });
        printer.register(
            "gcode_macro",
            Arc::new(PrinterGCodeMacro::new(&macro_config)) as ObjectRef,
        );
    }
    if !omit.contains(&"homing") {
        printer.register("homing", Arc::new(FakeHoming) as ObjectRef);
    }
    if !omit.contains(&"toolhead") {
        printer.register("toolhead", toolhead.clone() as ObjectRef);
    }
    let mut options = IndexMap::new();
    options.insert("gcode".to_string(), DEFAULT_SCRIPT.to_string());
    for (k, v) in opts {
        options.insert(k.to_string(), v.to_string());
    }
    let config: Config = Arc::new(FakeConfig {
        printer: printer_handle,
        opts: options,
        name: "homing_override".to_string(),
    });
    World { printer, gcode, toolhead, prev_calls, config }
}

fn err_value(e: &klippy_api::KlippyError) -> RecordingValue {
    RecordingValue::Tuple(vec![nval(e.class_name()), nval(e.message())])
}

fn strs_value(items: Vec<String>) -> RecordingValue {
    RecordingValue::List(items.iter().map(|s| nval(s)).collect())
}

fn reg_calls_value(gcode: &FakeGCode) -> RecordingValue {
    RecordingValue::List(
        gcode
            .calls_snapshot()
            .iter()
            .map(|(cmd, unreg)| RecordingValue::Tuple(vec![nval(cmd), RecordingValue::Bool(*unreg)]))
            .collect(),
    )
}

fn set_calls_value(toolhead: &FakeToolhead) -> RecordingValue {
    RecordingValue::List(
        toolhead
            .set_calls_snapshot()
            .iter()
            .map(|(pos, axes)| {
                RecordingValue::Tuple(vec![
                    RecordingValue::List(pos.iter().map(|f| RecordingValue::float(*f)).collect()),
                    nval(axes),
                ])
            })
            .collect(),
    )
}

fn prev_value(prev: &GCell<Vec<IndexMap<String, String>>>) -> RecordingValue {
    RecordingValue::List(
        prev.borrow()
            .iter()
            .map(|p| RecordingValue::Dict(p.iter().map(|(k, v)| (nval(k), nval(v))).collect()))
            .collect(),
    )
}

fn gcmd(params: &[(&str, &str)], raw: &str) -> GCmd {
    let mut map = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeGcmd { params: map, rawparams: raw.to_string(), commandline: "G28".to_string() })
}

/// Rust twin of the Python `run`: build the override, issue each call
/// through `cmd_g28`, and describe everything observable.
fn run_rust(opts: Opts, calls: &[Call], pos: &[f64], with_prev: bool, omit: &[&str]) -> RecordingValue {
    let w = world(opts, pos, with_prev, omit);
    let ho = match HomingOverride::new(&w.config) {
        Ok(ho) => ho,
        Err(e) => {
            return RecordingValue::Dict(vec![
                (nval("build_err"), err_value(&e)),
                (nval("loaded"), strs_value(w.printer.loaded_sections())),
            ]);
        }
    };
    w.gcode.set_route(ho.clone());
    let mut errs = Vec::new();
    for (params, raw) in calls {
        errs.push(match ho.cmd_g28(&gcmd(params, raw)) {
            Ok(()) => RecordingValue::Null,
            Err(e) => err_value(&e),
        });
    }
    RecordingValue::Dict(vec![
        (nval("build_err"), RecordingValue::Null),
        (nval("errs"), RecordingValue::List(errs)),
        (nval("set_calls"), set_calls_value(&w.toolhead)),
        (nval("scripts"), strs_value(w.gcode.scripts_snapshot())),
        (nval("prev"), prev_value(&w.prev_calls)),
        (nval("reg_calls"), reg_calls_value(&w.gcode)),
        (nval("loaded"), strs_value(w.printer.loaded_sections())),
        (nval("start_pos"), start_pos_value(&ho)),
        (nval("axes"), nval(ho.axes())),
    ])
}

fn py_str(s: &str) -> String {
    format!("{s:?}")
}

fn py_pairs(pairs: &[(&str, &str)]) -> String {
    let items: Vec<String> = pairs.iter().map(|(k, v)| format!("{}: {}", py_str(k), py_str(v))).collect();
    format!("{{{}}}", items.join(", "))
}

fn py_calls(calls: &[Call]) -> String {
    let items: Vec<String> = calls
        .iter()
        .map(|(params, raw)| format!("({}, {})", py_pairs(params), py_str(raw)))
        .collect();
    format!("[{}]", items.join(", "))
}

fn py_floats(pos: &[f64]) -> String {
    let items: Vec<String> = pos.iter().map(|f| format!("{f:?}")).collect();
    format!("[{}]", items.join(", "))
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_omit(omit: &[&str]) -> String {
    let items: Vec<String> = omit.iter().map(|s| format!("{},", py_str(s))).collect();
    format!("({})", items.join(" "))
}

fn py_run(opts: Opts, calls: &[Call], pos: &[f64], with_prev: bool, omit: &[&str]) -> String {
    format!(
        "run({}, {}, {}, {}, {})",
        py_pairs(opts),
        py_calls(calls),
        py_floats(pos),
        py_bool(with_prev),
        py_omit(omit)
    )
}

fn compare_run(py: &mut pyoracle::Oracle, label: &str, opts: Opts, calls: &[Call], pos: &[f64], with_prev: bool, omit: &[&str]) {
    let expected = py.eval(&py_run(opts, calls, pos, with_prev, omit));
    check(label, expected, run_rust(opts, calls, pos, with_prev, omit));
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: RecordingValue) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "homing_override".to_string(),
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

fn nval(s: &str) -> RecordingValue {
    RecordingValue::str(s)
}

fn start_pos_value(ho: &HomingOverride) -> RecordingValue {
    RecordingValue::List(
        ho.start_pos()
            .iter()
            .map(|o| match o {
                Some(v) => RecordingValue::float(*v),
                None => RecordingValue::Null,
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn no_axis_g28_runs_template_and_recursion_reaches_prev() {
    let Some(mut py) = oracle() else { return };
    compare_run(&mut py, "no-axis", &[], &[(&[], "")], &POS, true, &[]);
    // The nested `G28 X=0 Z=1` of the template must reach the previous
    // handler, not the template again.
    let w = world(&[], &POS, true, &[]);
    let ho = HomingOverride::new(&w.config).expect("build");
    w.gcode.set_route(ho.clone());
    ho.cmd_g28(&gcmd(&[], "")).expect("no-axis G28");
    // One script (the override template); its nested G28 reaches `prev`.
    assert_eq!(w.gcode.scripts_snapshot().len(), 1);
    assert_eq!(w.prev_calls.borrow().len(), 1);
}

#[test]
fn config_options_and_registration_chain() {
    let Some(mut py) = oracle() else { return };
    compare_run(&mut py, "options default", &[], &[], &POS, true, &[]);
    compare_run(
        &mut py,
        "options set",
        &[("axes", "xyz"), ("set_position_x", "10.5"), ("set_position_y", "-0.0"), ("set_position_z", "-1.25")],
        &[],
        &POS,
        true,
        &[],
    );
    compare_run(&mut py, "options registration without prev", &[], &[], &POS, false, &[]);
}

#[test]
fn forced_position_applied_before_template() {
    let Some(mut py) = oracle() else { return };
    let opts: Opts = &[("set_position_x", "10.5"), ("set_position_z", "-1.25")];
    compare_run(&mut py, "forced position", opts, &[(&[], "")], &POS, true, &[]);
    let w = world(opts, &POS, true, &[]);
    let ho = HomingOverride::new(&w.config).expect("build");
    w.gcode.set_route(ho.clone());
    ho.cmd_g28(&gcmd(&[], "")).expect("forced G28");
    let calls = w.toolhead.set_calls_snapshot();
    assert_eq!(calls, vec![(vec![10.5, 2.0, -1.25, 4.0], "xz".to_string())]);
    // A second forced homing starts from the position the first one left.
    compare_run(&mut py, "forced position twice", opts, &[(&[], ""), (&[], "")], &POS, true, &[]);
}

#[test]
fn params_and_rawparams_reach_template_context() {
    let Some(mut py) = oracle() else { return };
    let script = "M117 X{params.X} Z{params.Z}{rawparams}";
    compare_run(
        &mut py,
        "params context",
        &[("gcode", script)],
        &[(&[("X", "1"), ("Z", "-2")], " X1 Z-2")],
        &POS,
        true,
        &[],
    );
    compare_run(
        &mut py,
        "rawparams only",
        &[("gcode", "{rawparams}")],
        &[(&[], ""), (&[("Y", "5")], " Y5")],
        &POS,
        true,
        &[],
    );
}

#[test]
fn axes_option_selects_override_or_delegation() {
    let Some(mut py) = oracle() else { return };
    let script = "M117 {rawparams}";
    // Y is listed in the uppercased `axes`: override fires with a forced y.
    compare_run(
        &mut py,
        "axis listed",
        &[("gcode", script), ("axes", "xyz"), ("set_position_y", "0")],
        &[(&[("Y", "0")], " Y0")],
        &POS,
        true,
        &[],
    );
    // Z is not in axes=X: delegated; a call that also has X is overridden;
    // a call without axes is always overridden.
    compare_run(
        &mut py,
        "axis not listed",
        &[("gcode", script), ("axes", "X")],
        &[
            (&[("Z", "0")], " Z0"),
            (&[("X", "1"), ("Z", "0")], " X1 Z0"),
            (&[], ""),
            (&[("E", "1")], " E1"),
        ],
        &POS,
        true,
        &[],
    );
    compare_run(
        &mut py,
        "axes lowercase",
        &[("gcode", script), ("axes", "z")],
        &[(&[("Z", "0")], " Z0"), (&[("X", "0")], " X0")],
        &POS,
        true,
        &[],
    );
}

#[test]
fn short_position_raises_index_error() {
    let Some(mut py) = oracle() else { return };
    compare_run(
        &mut py,
        "short position",
        &[("set_position_z", "9")],
        &[(&[], "")],
        &[1.0],
        true,
        &[],
    );
}

#[test]
fn missing_prev_handler_raises_type_error() {
    let Some(mut py) = oracle() else { return };
    // Delegation, then the nested `G28` of the default script, twice: the
    // second override call proves `in_script` was reset after the error.
    compare_run(
        &mut py,
        "no prev handler",
        &[("axes", "X")],
        &[(&[("Z", "0")], " Z0"), (&[], ""), (&[], "")],
        &POS,
        false,
        &[],
    );
}

#[test]
fn in_script_is_reset_when_the_script_fails() {
    let Some(mut py) = oracle() else { return };
    compare_run(
        &mut py,
        "failing script",
        &[("gcode", "G28 FAIL=1\nM400")],
        &[(&[], ""), (&[], ""), (&[("X", "1")], " X1")],
        &POS,
        true,
        &[],
    );
}

#[test]
fn missing_objects_raise_like_the_reference() {
    let Some(mut py) = oracle() else { return };
    compare_run(&mut py, "missing gcode_macro", &[], &[], &POS, true, &["gcode_macro"]);
    compare_run(&mut py, "missing homing", &[], &[], &POS, true, &["homing"]);
    compare_run(&mut py, "missing gcode", &[], &[], &POS, true, &["gcode"]);
    // The toolhead is only looked up when an override fires.
    compare_run(&mut py, "missing toolhead", &[], &[(&[], ""), (&[("Z", "0")], " Z0")], &POS, true, &["toolhead"]);
}

#[test]
fn template_syntax_error_fails_the_load() {
    let Some(mut py) = oracle() else { return };
    compare_run(&mut py, "template syntax error", &[("gcode", "{% if x %}G1")], &[], &POS, true, &[]);
}

#[test]
fn load_config_registers_a_working_g28_handler() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::homing_override::module_factory();
    let load_config = factory.load_config.expect("homing_override exposes load_config");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());

    // `via_handler` builds with `homing_override.load_config` and invokes
    // the handler registered for G28, as the dispatcher does.
    let script = "M117{rawparams}";
    let expected = py.eval(&format!(
        "via_handler({}, {{'X': '1'}}, ' X1')",
        py_pairs(&[("gcode", script)])
    ));
    let w = world(&[("gcode", script)], &POS, true, &[]);
    let obj = load_config(&w.config).expect("factory load_config");
    assert!(obj.is_some(), "load_config builds the object");
    let mut handler = w.gcode.take_handler("G28").expect("G28 registered");
    let err = match handler(&gcmd(&[("X", "1")], " X1")) {
        Ok(()) => RecordingValue::Null,
        Err(e) => err_value(&e),
    };
    check(
        "registered G28 handler",
        expected,
        RecordingValue::Dict(vec![
            (nval("err"), err),
            (nval("set_calls"), set_calls_value(&w.toolhead)),
            (nval("scripts"), strs_value(w.gcode.scripts_snapshot())),
            (nval("prev"), prev_value(&w.prev_calls)),
            (nval("reg_calls"), reg_calls_value(&w.gcode)),
            (nval("loaded"), strs_value(w.printer.loaded_sections())),
        ]),
    );

    // A delegated G28 goes through the same handler to the previous one.
    let expected = py.eval(&format!(
        "via_handler({}, {{'Z': '0'}}, ' Z0')",
        py_pairs(&[("axes", "X")])
    ));
    let w2 = world(&[("axes", "X")], &POS, true, &[]);
    load_config(&w2.config).expect("factory load_config");
    let mut handler = w2.gcode.take_handler("G28").expect("G28 registered");
    let err = match handler(&gcmd(&[("Z", "0")], " Z0")) {
        Ok(()) => RecordingValue::Null,
        Err(e) => err_value(&e),
    };
    check(
        "registered G28 handler delegates",
        expected,
        RecordingValue::Dict(vec![
            (nval("err"), err),
            (nval("set_calls"), set_calls_value(&w2.toolhead)),
            (nval("scripts"), strs_value(w2.gcode.scripts_snapshot())),
            (nval("prev"), prev_value(&w2.prev_calls)),
            (nval("reg_calls"), reg_calls_value(&w2.gcode)),
            (nval("loaded"), strs_value(w2.printer.loaded_sections())),
        ]),
    );
}
