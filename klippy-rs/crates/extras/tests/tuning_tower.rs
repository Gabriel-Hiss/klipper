//! Parity tests for `extras::tuning_tower` (port of `klippy/extras/tuning_tower.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `TuningTower` is built
//! on both sides with minimal Python fakes for the printer, `gcode`,
//! `gcode_move` and the move transform, then fed identical
//! `TUNING_TOWER` commands and `move` calls. Returned values (floats by
//! bits), raised errors (class and message), `respond_info` text,
//! `run_script_from_command` scripts and forwarded moves are compared
//! exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::tuning_tower::{
    TuningTower, CANCEL_Z_DELTA, CMD_TUNING_TOWER_HELP, TUNING_TOWER_CMD,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandHandler,
    GetFloatOpts as GcmdFloatOpts, GetIntOpts as GcmdIntOpts,
    GetOpts as GcmdGetOpts, GetParser, MoveTransform, MoveTransformApi,
    OutputCallback, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.tuning_tower as tuning_tower
from gcode import GCodeCommand, Coord

class FakeTransform:
    def __init__(self, pos):
        self.pos = list(pos)
        self.moves = []
    def get_position(self):
        return list(self.pos)
    def move(self, newpos, speed):
        self.moves.append((list(newpos), speed))
        self.pos = list(newpos)

class FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.helps = {}
        self.infos = []
        self.raws = []
        self.scripts = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            old = self.handlers.get(cmd)
            self.handlers.pop(cmd, None)
            self.helps.pop(cmd, None)
            return old
        if cmd in self.handlers:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = func
        if desc is not None:
            self.helps[cmd] = desc
        return None
    def is_traditional_gcode(self, cmd):
        try:
            c = cmd.upper().split()[0]
            float(c[1:])
            return c[0].isupper() and c[1].isdigit()
        except:
            return False
    def respond_info(self, msg, log=True):
        self.infos.append((msg, log))
    def respond_raw(self, msg):
        self.raws.append(msg)
    def run_script_from_command(self, script):
        self.scripts.append(script)

class FakeGCodeMove:
    def __init__(self):
        self.normal = FakeTransform([0., 0., 0., 0.])
        self.move_transform = self.normal
        self.gcode_position = [0., 0., 0., 0.]
    def set_move_transform(self, transform, force=False):
        if self.move_transform is not None and not force:
            raise configparser.Error("G-Code move transform already specified")
        old = self.move_transform
        self.move_transform = transform
        return old
    def get_status(self, eventtime=None):
        return {'gcode_position': Coord(self.gcode_position)}

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.gcode = FakeGCode()
        self.gcode_move = FakeGCodeMove()
    def lookup_object(self, name, default=None):
        if name == 'gcode':
            return self.gcode
        if name == 'gcode_move':
            return self.gcode_move
        raise configparser.Error("Unknown config object '%s'" % (name,))
    def load_object(self, config, section, default=None):
        if section == 'gcode_move':
            return self.gcode_move
        raise configparser.Error("Unable to load module '%s'" % (section,))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn is_traditional_gcode(cmd: &str) -> bool {
    // Port of `GCodeDispatch.is_traditional_gcode` (`klippy/gcode.py:125`).
    let upper = cmd.to_uppercase();
    let Some(first) = upper.split_whitespace().next() else {
        return false;
    };
    let rest = &first[first.len().min(1)..];
    if rest.parse::<f64>().is_err() {
        return false;
    }
    let first_char = first.chars().next().unwrap_or('a');
    let second_char = first.chars().nth(1).unwrap_or('a');
    first_char.is_uppercase() && second_char.is_ascii_digit()
}

fn missing_msg(commandline: &str, name: &str) -> String {
    format!("Error on '{commandline}': missing {name}")
}

fn unable_msg(commandline: &str, value: &str) -> String {
    format!("Error on '{commandline}': unable to parse {value}")
}

fn bound_msg(commandline: &str, name: &str, kind: &str, bound: f64) -> String {
    let detail = match kind {
        "above" => format!("must be above {}", pycompat::text::float_str(bound)),
        "below" => format!("must be below {}", pycompat::text::float_str(bound)),
        other => format!("must have {other} of {}", pycompat::text::float_str(bound)),
    };
    format!("Error on '{commandline}': {name} {detail}")
}

#[derive(Default, Clone)]
struct GCodeShared {
    handlers: indexmap::IndexMap<String, Option<String>>,
    helps: indexmap::IndexMap<String, String>,
    infos: Vec<(String, bool)>,
    raws: Vec<String>,
    scripts: Vec<String>,
}

struct FakeGCode {
    shared: Mutex<GCodeShared>,
}

impl FakeGCode {
    fn new() -> Self {
        FakeGCode {
            shared: Mutex::new(GCodeShared::default()),
        }
    }
    fn infos(&self) -> Vec<(String, bool)> {
        self.shared.lock().unwrap().infos.clone()
    }
    fn scripts(&self) -> Vec<String> {
        self.shared.lock().unwrap().scripts.clone()
    }
    fn help(&self, cmd: &str) -> Option<String> {
        self.shared.lock().unwrap().helps.get(cmd).cloned()
    }
}

impl klippy_api::gcode::GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut shared = self.shared.lock().unwrap();
        if func.is_none() {
            shared.handlers.shift_remove(cmd);
            shared.helps.shift_remove(cmd);
            return Ok(None);
        }
        if shared.handlers.contains_key(cmd) {
            return Err(KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        shared.handlers.insert(cmd.to_string(), opts.desc.clone());
        if let Some(desc) = opts.desc {
            shared.helps.insert(cmd.to_string(), desc);
        }
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
        panic!("register_mux_command is not used by tuning_tower");
    }
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        self.shared.lock().unwrap().helps.clone()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn register_output_handler(&self, _callback: OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.shared
            .lock()
            .unwrap()
            .scripts
            .push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.shared
            .lock()
            .unwrap()
            .scripts
            .push(script.to_string());
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by tuning_tower");
    }
    fn respond_raw(&self, msg: &str) {
        self.shared
            .lock()
            .unwrap()
            .raws
            .push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.shared
            .lock()
            .unwrap()
            .infos
            .push((msg.to_string(), log));
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        is_traditional_gcode(cmd)
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by tuning_tower");
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by tuning_tower");
    }
}

klippy_api::printer_object!(FakeGCode; dyn klippy_api::gcode::GCodeApi);

struct FakeTransform {
    pos: Mutex<Vec<f64>>,
    moves: Mutex<Vec<(Vec<f64>, f64)>>,
}

impl FakeTransform {
    fn new(pos: Vec<f64>) -> Self {
        FakeTransform {
            pos: Mutex::new(pos),
            moves: Mutex::new(Vec::new()),
        }
    }
    fn set_pos(&self, pos: Vec<f64>) {
        *self.pos.lock().unwrap() = pos;
    }
    fn moves(&self) -> Vec<(Vec<f64>, f64)> {
        self.moves.lock().unwrap().clone()
    }
}

impl MoveTransformApi for FakeTransform {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.moves.lock().unwrap().push((newpos.clone(), speed));
        *self.pos.lock().unwrap() = newpos;
        Ok(())
    }
    fn get_position(&self) -> KResult<Vec<f64>> {
        Ok(self.pos.lock().unwrap().clone())
    }
}

struct FakeGCodeMove {
    current: Mutex<Option<MoveTransform>>,
    gcode_position: Mutex<Vec<f64>>,
}

impl FakeGCodeMove {
    fn new(normal: Arc<FakeTransform>) -> Self {
        let handle: klippy_api::gcode::MoveTransformHandle = normal;
        FakeGCodeMove {
            current: Mutex::new(Some(MoveTransform::Custom(handle))),
            gcode_position: Mutex::new(vec![0.0, 0.0, 0.0, 0.0]),
        }
    }
    fn set_gcode_position(&self, pos: Vec<f64>) {
        *self.gcode_position.lock().unwrap() = pos;
    }
}

impl klippy_api::gcode::GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let pos = self.gcode_position.lock().unwrap().clone();
        let mut entries = indexmap::IndexMap::new();
        entries.insert(
            PyValue::Str("gcode_position".to_string()),
            PyValue::Tuple(pos.iter().map(|f| PyValue::Float(*f)).collect()),
        );
        PyValue::Dict(entries)
    }
    fn set_move_transform(
        &self,
        transform: MoveTransform,
        force: bool,
    ) -> KResult<Option<MoveTransform>> {
        let mut current = self.current.lock().unwrap();
        if current.is_some() && !force {
            return Err(KlippyError::config(
                "G-Code move transform already specified",
            ));
        }
        Ok(current.replace(transform))
    }
    fn reset_last_position(&self) {}
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> {
        panic!("cmd_g1 is not used by tuning_tower");
    }
}

klippy_api::printer_object!(FakeGCodeMove; dyn klippy_api::gcode::GCodeMoveApi);

struct FakeGCodeCommand {
    command: String,
    commandline: String,
    params: indexmap::IndexMap<String, String>,
    gcode: Arc<FakeGCode>,
}

impl klippy_api::gcode::GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: GcmdGetOpts) -> KResult<PyValue> {
        let value = match self.params.get(name) {
            None => match opts.default {
                None => {
                    return Err(KlippyError::command(missing_msg(
                        &self.commandline, name,
                    )))
                }
                Some(default) => return Ok(default),
            },
            Some(v) => v.clone(),
        };
        match opts.parser {
            GetParser::Str => Ok(PyValue::Str(value)),
            GetParser::Int => {
                let parsed = value
                    .parse::<i64>()
                    .map_err(|_| KlippyError::command(unable_msg(&self.commandline, &value)))?;
                Ok(PyValue::Int(parsed.into()))
            }
            GetParser::IntAutoBase => {
                let parsed = value
                    .parse::<i64>()
                    .map_err(|_| KlippyError::command(unable_msg(&self.commandline, &value)))?;
                Ok(PyValue::Int(parsed.into()))
            }
            GetParser::Float => {
                let parsed = pycompat::text::float_from_string(&value).map_err(|_| {
                    KlippyError::command(unable_msg(&self.commandline, &value))
                })?;
                Ok(PyValue::Float(parsed))
            }
        }
    }
    fn get_int(&self, name: &str, opts: GcmdIntOpts) -> KResult<Option<i64>> {
        let raw = match self.params.get(name) {
            None => match opts.default {
                None => {
                    return Err(KlippyError::command(missing_msg(
                        &self.commandline, name,
                    )))
                }
                Some(default) => return Ok(default),
            },
            Some(v) => v.clone(),
        };
        let parsed = raw
            .parse::<i64>()
            .map_err(|_| KlippyError::command(unable_msg(&self.commandline, &raw)))?;
        if let Some(min) = opts.minval {
            if parsed < min {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have minimum of {min}",
                    self.commandline
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if parsed > max {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have maximum of {max}",
                    self.commandline
                )));
            }
        }
        Ok(Some(parsed))
    }
    fn get_float(&self, name: &str, opts: GcmdFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            None => match opts.default {
                None => {
                    return Err(KlippyError::command(missing_msg(
                        &self.commandline, name,
                    )))
                }
                Some(default) => return Ok(default),
            },
            Some(v) => v.clone(),
        };
        let parsed = pycompat::text::float_from_string(&raw)
            .map_err(|_| KlippyError::command(unable_msg(&self.commandline, &raw)))?;
        if let Some(min) = opts.minval {
            if parsed < min {
                return Err(KlippyError::command(bound_msg(
                    &self.commandline, name, "minimum", min,
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if parsed > max {
                return Err(KlippyError::command(bound_msg(
                    &self.commandline, name, "maximum", max,
                )));
            }
        }
        if let Some(above) = opts.above {
            if parsed <= above {
                return Err(KlippyError::command(bound_msg(
                    &self.commandline, name, "above", above,
                )));
            }
        }
        if let Some(below) = opts.below {
            if parsed >= below {
                return Err(KlippyError::command(bound_msg(
                    &self.commandline, name, "below", below,
                )));
            }
        }
        Ok(Some(parsed))
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> indexmap::IndexMap<String, String> {
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
        false
    }
}

#[derive(Clone)]
struct FakePrinter {
    gcode: ObjectRef,
    gcode_move: ObjectRef,
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
        let obj = if name == "gcode" {
            Some(self.gcode.clone())
        } else if name == "gcode_move" {
            Some(self.gcode_move.clone())
        } else {
            None
        };
        match obj {
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
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "tuning_tower".to_string()
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
        panic!("getsection is not used by tuning_tower");
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
// Pair construction.
// ---------------------------------------------------------------------------

struct Pair {
    tower: Arc<TuningTower>,
    gcode: Arc<FakeGCode>,
    gcode_move: Arc<FakeGCodeMove>,
    normal: Arc<FakeTransform>,
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("tuning_tower fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping tuning_tower oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn fresh_pair(py: &mut pyoracle::Oracle) -> Pair {
    py.exec("p = FakePrinter(); c = FakeConfig(p); tt = tuning_tower.load_config(c)")
        .expect("py load_config");
    let gcode = Arc::new(FakeGCode::new());
    let normal = Arc::new(FakeTransform::new(vec![0.0, 0.0, 0.0, 0.0]));
    let gcode_move = Arc::new(FakeGCodeMove::new(normal.clone()));
    let gcode_obj: ObjectRef = gcode.clone();
    let gcode_move_obj: ObjectRef = gcode_move.clone();
    let printer: Printer = Arc::new(FakePrinter {
        gcode: gcode_obj,
        gcode_move: gcode_move_obj,
    });
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
    });
    let tower = TuningTower::new(&config).expect("rust load_config");
    Pair {
        tower,
        gcode,
        gcode_move,
        normal,
    }
}

fn make_gcmd(
    gcode: &Arc<FakeGCode>,
    commandline: &str,
    params: &[(&str, &str)],
) -> GCmd {
    let mut map = indexmap::IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    let fake = FakeGCodeCommand {
        command: TUNING_TOWER_CMD.to_string(),
        commandline: commandline.to_string(),
        params: map,
        gcode: gcode.clone(),
    };
    Arc::new(fake)
}

fn py_make_gcmd(py: &mut pyoracle::Oracle, commandline: &str, params: &[(&str, &str)]) {
    let mut dict = String::from("{");
    for (i, (k, v)) in params.iter().enumerate() {
        if i > 0 {
            dict.push_str(", ");
        }
        dict.push_str(&format!("'{k}': '{v}'"));
    }
    dict.push('}');
    py.exec(&format!(
        "gcmd = GCodeCommand(p.gcode, 'TUNING_TOWER', '{commandline}', {dict}, False)"
    ))
    .expect("py make gcmd");
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "tuning_tower".to_string(),
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

fn check_float(label: &str, expected: pyoracle::OracleResult, actual: f64) {
    check(label, expected, Value::float(actual));
}

fn check_float_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<f64>) {
    match (expected, actual) {
        (Ok(v), Ok(f)) => check(label, Ok(v), Value::float(f)),
        (Err(e), Err(err)) => {
            assert_eq!(
                err.class_name(),
                e.class_name.as_str(),
                "{label}: exception class",
            );
            assert_eq!(
                err.message(),
                e.message.as_str(),
                "{label}: exception message",
            );
        }
        (Ok(v), Err(err)) => {
            panic!("{label}: oracle returned {v:?}, rust raised {}: {}", err.class_name(), err.message())
        }
        (Err(e), Ok(f)) => {
            panic!("{label}: oracle raised {}: {}, rust returned {f}", e.class_name, e.message)
        }
    }
}

fn check_unit(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match (expected, actual) {
        (Ok(v), Ok(())) => {
            if v != Value::Null {
                panic!("{label}: oracle returned {v:?}, rust returned ()");
            }
        }
        (Err(e), Err(err)) => {
            assert_eq!(
                err.class_name(),
                e.class_name.as_str(),
                "{label}: exception class",
            );
            assert_eq!(
                err.message(),
                e.message.as_str(),
                "{label}: exception message",
            );
        }
        (Ok(v), Err(err)) => {
            panic!("{label}: oracle returned {v:?}, rust raised {}: {}", err.class_name(), err.message())
        }
        (Err(e), Ok(())) => {
            panic!("{label}: oracle raised {}: {}, rust returned ()", e.class_name, e.message)
        }
    }
}

fn check_vec(label: &str, expected: pyoracle::OracleResult, actual: KResult<Vec<f64>>) {
    match (expected, actual) {
        (Ok(v), Ok(actual)) => check(
            label,
            Ok(v),
            Value::List(actual.iter().map(|f| Value::float(*f)).collect()),
        ),
        (Err(e), Err(err)) => {
            assert_eq!(
                err.class_name(),
                e.class_name.as_str(),
                "{label}: exception class",
            );
            assert_eq!(
                err.message(),
                e.message.as_str(),
                "{label}: exception message",
            );
        }
        (Ok(v), Err(err)) => {
            panic!("{label}: oracle returned {v:?}, rust raised {}: {}", err.class_name(), err.message())
        }
        (Err(e), Ok(actual)) => {
            panic!("{label}: oracle raised {}: {}, rust returned {actual:?}", e.class_name, e.message)
        }
    }
}

fn check_infos(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("p.gcode.infos");
    let infos = pair.gcode.infos();
    check(
        &format!("{label} infos"),
        expected,
        Value::List(
            infos
                .iter()
                .map(|(msg, log)| {
                    Value::Tuple(vec![Value::str(msg), Value::Bool(*log)])
                })
                .collect(),
        ),
    );
}

fn check_scripts(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("p.gcode.scripts");
    check(
        &format!("{label} scripts"),
        expected,
        Value::List(
            pair.gcode
                .scripts()
                .iter()
                .map(|s| Value::str(s))
                .collect(),
        ),
    );
}

fn check_moves(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("p.gcode_move.normal.moves");
    let moves = pair.normal.moves();
    check(
        &format!("{label} moves"),
        expected,
        Value::List(
            moves
                .iter()
                .map(|(pos, speed)| {
                    Value::Tuple(vec![
                        Value::List(pos.iter().map(|f| Value::float(*f)).collect()),
                        Value::float(*speed),
                    ])
                })
                .collect(),
        ),
    );
}

fn check_active(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("tt.is_active()");
    check(label, expected, Value::Bool(pair.tower.is_active()));
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("tuning_tower.CANCEL_Z_DELTA");
    check_float("cancel_z_delta", expected, CANCEL_Z_DELTA);
    let expected = py.eval("tuning_tower.TuningTower.cmd_TUNING_TOWER_help");
    check("help", expected, Value::str(CMD_TUNING_TOWER_HELP));
    let expected = py.eval("'TUNING_TOWER'");
    check("cmd_name", expected, Value::str(TUNING_TOWER_CMD));
}

#[test]
fn registers_tuning_tower_with_help() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    let expected = py.eval("sorted(p.gcode.handlers.keys())");
    check(
        "registered handlers",
        expected,
        Value::List(vec![Value::str(TUNING_TOWER_CMD)]),
    );
    let expected = py.eval("p.gcode.helps['TUNING_TOWER']");
    check("registered help", expected, Value::str(CMD_TUNING_TOWER_HELP));
    let help = pair.gcode.help(TUNING_TOWER_CMD);
    assert_eq!(help.as_deref(), Some(CMD_TUNING_TOWER_HELP));
}

#[test]
fn calc_value_before_start_raises_skip_attribute() {
    // `step_delta`/`step_height`/`skip` only exist after the first
    // `TUNING_TOWER` command (`tuning_tower.py:33-35`), so `calc_value`
    // raises `AttributeError` for `skip` before any command.
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    for z in [0.0, 1.0, -2.5, 10.0] {
        let expected = py.eval(&format!("tt.calc_value({z:?})"));
        check_float_result(
            &format!("calc_value before start z={z}"),
            expected,
            pair.tower.calc_value(z),
        );
    }
}

fn start_and_calc(
    py: &mut pyoracle::Oracle,
    pair: &Pair,
    label: &str,
    commandline: &str,
    params: &[(&str, &str)],
    zs: &[f64],
) {
    py_make_gcmd(py, commandline, params);
    let expected = py.eval("tt.cmd_TUNING_TOWER(gcmd)");
    let gcmd = make_gcmd(&pair.gcode, commandline, params);
    let actual = pair.tower.cmd_tuning_tower(&gcmd);
    check_unit(label, expected, actual);
    for z in zs {
        let expected = py.eval(&format!("tt.calc_value({z:?})"));
        check_float_result(
            &format!("{label} calc_value z={z}"),
            expected,
            pair.tower.calc_value(*z),
        );
    }
}

#[test]
fn calc_value_factor_linear() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "factor",
        "TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0.1 FACTOR=0.005",
        &[
            ("COMMAND", "SET_PRESSURE_ADVANCE"),
            ("PARAMETER", "ADVANCE"),
            ("START", "0.1"),
            ("FACTOR", "0.005"),
        ],
        &[0.0, 0.2, 1.0, 10.0, 33.75],
    );
    check_infos("factor", &mut py, &pair);
}

#[test]
fn calc_value_factor_with_band() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "factor band",
        "TUNING_TOWER COMMAND=SET_INPUT_SHAPER PARAMETER=SHAPER_FREQ_X START=20 FACTOR=1.5 BAND=5",
        &[
            ("COMMAND", "SET_INPUT_SHAPER"),
            ("PARAMETER", "SHAPER_FREQ_X"),
            ("START", "20"),
            ("FACTOR", "1.5"),
            ("BAND", "5"),
        ],
        &[0.0, 2.4, 2.5, 2.6, 5.0, 7.5, 12.3],
    );
    check_infos("factor band", &mut py, &pair);
}

#[test]
fn calc_value_step_mode_uses_step_not_band() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    // BAND is present but STEP_HEIGHT takes precedence in calc_value.
    start_and_calc(
        &mut py,
        &pair,
        "step",
        "TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1500 STEP_DELTA=500 STEP_HEIGHT=5 BAND=3",
        &[
            ("COMMAND", "SET_VELOCITY_LIMIT"),
            ("PARAMETER", "ACCEL"),
            ("START", "1500"),
            ("STEP_DELTA", "500"),
            ("STEP_HEIGHT", "5"),
            ("BAND", "3"),
        ],
        &[0.0, 4.0, 4.999, 5.0, 9.99, 10.0, 15.0],
    );
    check_infos("step", &mut py, &pair);
}

#[test]
fn calc_value_skip_clamps_at_zero() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "skip",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=100 FACTOR=2 SKIP=1",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "100"),
            ("FACTOR", "2"),
            ("SKIP", "1"),
        ],
        &[0.0, 0.5, 1.0, 1.5, 3.0],
    );
    check_infos("skip", &mut py, &pair);
}

#[test]
fn calc_value_skip_with_steps() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "skip step",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=10 STEP_DELTA=4 STEP_HEIGHT=2 SKIP=3",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "10"),
            ("STEP_DELTA", "4"),
            ("STEP_HEIGHT", "2"),
            ("SKIP", "3"),
        ],
        &[0.0, 3.0, 4.9, 5.0, 9.0],
    );
    check_infos("skip step", &mut py, &pair);
}

#[test]
fn command_fmt_uses_traditional_and_extended_forms() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    // Traditional: no `=` in the emitted command.
    start_and_calc(
        &mut py,
        &pair,
        "traditional fmt",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    let expected = py.eval("tt.command_fmt");
    check("traditional fmt", expected, Value::str(&pair.tower.command_fmt()));
    // Restart with an extended name: `=` form.
    start_and_calc(
        &mut py,
        &pair,
        "extended fmt",
        "TUNING_TOWER COMMAND=SET_FAN_SPEED PARAMETER=speed START=0 FACTOR=1",
        &[
            ("COMMAND", "SET_FAN_SPEED"),
            ("PARAMETER", "speed"),
            ("START", "0"),
            ("FACTOR", "1"),
        ],
        &[],
    );
    let expected = py.eval("tt.command_fmt");
    check("extended fmt", expected, Value::str(&pair.tower.command_fmt()));
    // Restarting while active ends the first test first.
    check_infos("restart", &mut py, &pair);
}

#[test]
fn starting_twice_ends_previous_test() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "first start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    check_active("active after first start", &mut py, &pair);
    start_and_calc(
        &mut py,
        &pair,
        "second start",
        "TUNING_TOWER COMMAND=M105 PARAMETER=T START=100 FACTOR=0.5 BAND=2 SKIP=1",
        &[
            ("COMMAND", "M105"),
            ("PARAMETER", "T"),
            ("START", "100"),
            ("FACTOR", "0.5"),
            ("BAND", "2"),
            ("SKIP", "1"),
        ],
        &[],
    );
    check_active("active after second start", &mut py, &pair);
    check_infos("double start", &mut py, &pair);
    let expected = py.eval("[tt.start, tt.factor, tt.band, tt.skip, tt.step_delta, tt.step_height]");
    check(
        "second start params",
        expected,
        Value::List(vec![
            Value::float(pair.tower.start()),
            Value::float(pair.tower.factor()),
            Value::float(pair.tower.band()),
            Value::float(pair.tower.skip().expect("skip set by start")),
            Value::float(pair.tower.step_delta().expect("step_delta set by start")),
            Value::float(pair.tower.step_height().expect("step_height set by start")),
        ]),
    );
}

#[test]
fn end_test_restores_transform() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    check_active("active", &mut py, &pair);
    let expected = py.eval("tt.end_test()");
    let actual = pair.tower.end_test();
    check_unit("end_test", expected, actual);
    check_active("inactive after end_test", &mut py, &pair);
    check_infos("end_test", &mut py, &pair);
}

#[test]
fn get_position_and_move_when_inactive_raise() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    let expected = py.eval("tt.get_position()");
    let actual = pair.tower.get_position();
    check_vec("get_position inactive", expected, actual);
    let expected = py.eval("tt.move([1.0, 0.0, 0.2, 1.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![1.0, 0.0, 0.2, 1.0], 20.0);
    check_unit("move inactive", expected, actual);
}

#[test]
fn move_emits_script_and_forwards() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0 FACTOR=0.005",
        &[
            ("COMMAND", "SET_PRESSURE_ADVANCE"),
            ("PARAMETER", "ADVANCE"),
            ("START", "0"),
            ("FACTOR", "0.005"),
        ],
        &[],
    );
    // gcode_z drives the emitted value; newpos z is the layer height.
    py.exec("p.gcode_move.gcode_position = [0., 0., 10.0, 0.]")
        .expect("set gcode z");
    pair.gcode_move
        .set_gcode_position(vec![0.0, 0.0, 10.0, 0.0]);
    let expected = py.eval("tt.move([1.0, 0.0, 0.2, 1.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![1.0, 0.0, 0.2, 1.0], 20.0);
    check_unit("extruding move", expected, actual);
    check_scripts("extruding move", &mut py, &pair);
    check_moves("extruding move", &mut py, &pair);
    let expected = py.eval("tt.last_z");
    check_float("last_z", expected, pair.tower.last_z());
    let expected = py.eval("tt.last_position");
    check_vec("last_position", expected, Ok(pair.tower.last_position()));
}

#[test]
fn move_traditional_script_has_no_equals() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    py.exec("p.gcode_move.gcode_position = [0., 0., 2.0, 0.]")
        .expect("set gcode z");
    pair.gcode_move
        .set_gcode_position(vec![0.0, 0.0, 2.0, 0.0]);
    let expected = py.eval("tt.move([1.0, 0.0, 0.2, 1.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![1.0, 0.0, 0.2, 1.0], 20.0);
    check_unit("traditional move", expected, actual);
    check_scripts("traditional move", &mut py, &pair);
    check_moves("traditional move", &mut py, &pair);
}

#[test]
fn move_filters_non_extruding_moves() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    // No extrusion (e equal).
    py.exec("p.gcode_move.gcode_position = [0., 0., 2.0, 0.]")
        .expect("set gcode z");
    pair.gcode_move
        .set_gcode_position(vec![0.0, 0.0, 2.0, 0.0]);
    let expected = py.eval("tt.move([1.0, 0.0, 0.2, 0.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![1.0, 0.0, 0.2, 0.0], 20.0);
    check_unit("no extrusion", expected, actual);
    // Extruding with unchanged XYZ does not trigger.
    let expected = py.eval("tt.move([1.0, 0.0, 0.2, 0.5], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![1.0, 0.0, 0.2, 0.5], 20.0);
    check_unit("same XYZ", expected, actual);
    // Extruding with changed XY and Z triggers once.
    let expected = py.eval("tt.move([2.0, 0.0, 0.2, 1.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![2.0, 0.0, 0.2, 1.0], 20.0);
    check_unit("triggering move", expected, actual);
    // Extruding with changed XY but unchanged Z does not trigger.
    let expected = py.eval("tt.move([3.0, 0.0, 0.2, 2.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![3.0, 0.0, 0.2, 2.0], 20.0);
    check_unit("same Z", expected, actual);
    // Only the triggering move emitted a script; all moves were forwarded.
    check_scripts("filtered moves", &mut py, &pair);
    check_moves("filtered moves", &mut py, &pair);
    check_active("still active", &mut py, &pair);
}

#[test]
fn move_repeats_only_when_value_changes() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=0 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "0"), ("FACTOR", "1")],
        &[],
    );
    for (i, (gcode_z, newpos)) in [
        (2.0, vec![1.0, 0.0, 0.2, 1.0]),
        (2.0, vec![2.0, 0.0, 0.4, 2.0]),
        (3.0, vec![3.0, 0.0, 0.6, 3.0]),
    ]
    .into_iter()
    .enumerate()
    {
        py.exec(&format!(
            "p.gcode_move.gcode_position = [0., 0., {gcode_z:?}, 0.]"
        ))
        .expect("set gcode z");
        pair.gcode_move
            .set_gcode_position(vec![0.0, 0.0, gcode_z, 0.0]);
        let expr = format!(
            "tt.move({:?}, 20.0)",
            newpos
        );
        let expected = py.eval(&expr);
        let actual = pair.tower.move_pos(newpos, 20.0);
        check_unit(&format!("move {i}"), expected, actual);
    }
    // Middle move repeats value 2.0 and emits nothing; two scripts total.
    check_scripts("repeat filter", &mut py, &pair);
    check_moves("repeat filter", &mut py, &pair);
}

#[test]
fn move_z_drop_ends_test() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    py.exec("p.gcode_move.gcode_position = [0., 0., 2.0, 0.]")
        .expect("set gcode z");
    pair.gcode_move
        .set_gcode_position(vec![0.0, 0.0, 2.0, 0.0]);
    let expected = py.eval("tt.move([1.0, 0.0, 5.0, 1.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![1.0, 0.0, 5.0, 1.0], 20.0);
    check_unit("move to z=5", expected, actual);
    // Drop of exactly CANCEL_Z_DELTA does not end the test (strict <).
    let expected = py.eval("tt.move([2.0, 0.0, 3.0, 2.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![2.0, 0.0, 3.0, 2.0], 20.0);
    check_unit("drop exactly 2.0", expected, actual);
    check_active("active after exact drop", &mut py, &pair);
    // Drop just over 2.0 ends the test, still forwards the move.
    let expected = py.eval("tt.move([3.0, 0.0, 0.9, 3.0], 20.0)");
    let actual = pair
        .tower
        .move_pos(vec![3.0, 0.0, 0.9, 3.0], 20.0);
    check_unit("drop over 2.0", expected, actual);
    check_active("inactive after over drop", &mut py, &pair);
    check_infos("z drop", &mut py, &pair);
    check_moves("z drop", &mut py, &pair);
    let expected = py.eval("tt.last_position");
    check_vec("last_position after cancel", expected, Ok(pair.tower.last_position()));
}

fn cmd_error_case(
    label: &str,
    commandline: &str,
    params: &[(&str, &str)],
) {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    py_make_gcmd(&mut py, commandline, params);
    let expected = py.eval("tt.cmd_TUNING_TOWER(gcmd)");
    let gcmd = make_gcmd(&pair.gcode, commandline, params);
    let actual = pair.tower.cmd_tuning_tower(&gcmd);
    check_unit(label, expected, actual);
    // A failed start leaves the tower inactive and without a starting message.
    check_active(&format!("{label} active"), &mut py, &pair);
    check_infos(label, &mut py, &pair);
}

#[test]
fn cmd_missing_command_and_parameter() {
    cmd_error_case(
        "missing command",
        "TUNING_TOWER PARAMETER=S START=200 FACTOR=1",
        &[("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
    );
    cmd_error_case(
        "missing parameter",
        "TUNING_TOWER COMMAND=M104 START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("START", "200"), ("FACTOR", "1")],
    );
}

#[test]
fn cmd_factor_with_steps_rejected() {
    cmd_error_case(
        "factor with step_delta",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1 STEP_DELTA=5",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("FACTOR", "1"),
            ("STEP_DELTA", "5"),
        ],
    );
    cmd_error_case(
        "factor with step_height",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1 STEP_HEIGHT=2",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("FACTOR", "1"),
            ("STEP_HEIGHT", "2"),
        ],
    );
}

#[test]
fn cmd_step_mode_requires_both_half() {
    cmd_error_case(
        "step_delta without height",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 STEP_DELTA=5",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("STEP_DELTA", "5"),
        ],
    );
    cmd_error_case(
        "step_height without delta",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 STEP_HEIGHT=2",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("STEP_HEIGHT", "2"),
        ],
    );
}

#[test]
fn cmd_numeric_parsing_errors() {
    cmd_error_case(
        "start not a float",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=abc FACTOR=1",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "abc"),
            ("FACTOR", "1"),
        ],
    );
    cmd_error_case(
        "negative band",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1 BAND=-1",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("FACTOR", "1"),
            ("BAND", "-1"),
        ],
    );
    cmd_error_case(
        "negative step_height",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 STEP_DELTA=5 STEP_HEIGHT=-2",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("STEP_DELTA", "5"),
            ("STEP_HEIGHT", "-2"),
        ],
    );
    cmd_error_case(
        "negative skip",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1 SKIP=-0.5",
        &[
            ("COMMAND", "M104"),
            ("PARAMETER", "S"),
            ("START", "200"),
            ("FACTOR", "1"),
            ("SKIP", "-0.5"),
        ],
    );
}

#[test]
fn get_position_after_start_matches_normal() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    py.exec("p.gcode_move.normal.pos = [5.0, 6.0, 7.0, 8.0]")
        .expect("set normal pos");
    pair.normal.set_pos(vec![5.0, 6.0, 7.0, 8.0]);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    let expected = py.eval("tt.get_position()");
    let actual = pair.tower.get_position();
    check_vec("get_position after start", expected, actual);
    let expected = py.eval("tt.last_position");
    check_vec("last_position after start", expected, Ok(pair.tower.last_position()));
}

#[test]
fn move_transform_interface_round_trip() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py);
    start_and_calc(
        &mut py,
        &pair,
        "start",
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
        &[],
    );
    py.exec("p.gcode_move.gcode_position = [0., 0., 2.0, 0.]")
        .expect("set gcode z");
    pair.gcode_move
        .set_gcode_position(vec![0.0, 0.0, 2.0, 0.0]);
    // Call through the `MoveTransformApi` trait object, as `gcode_move` does.
    let expected = py.eval("tt.move([1.0, 0.0, 0.2, 1.0], 20.0)");
    let actual = MoveTransformApi::r#move(
        &*pair.tower,
        vec![1.0, 0.0, 0.2, 1.0],
        20.0,
    );
    check_unit("trait move", expected, actual);
    let expected = py.eval("tt.get_position()");
    let actual = MoveTransformApi::get_position(&*pair.tower);
    check_vec("trait get_position", expected, actual);
    check_scripts("trait move", &mut py, &pair);
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::tuning_tower::module_factory();
    let load_config = factory
        .load_config
        .expect("tuning_tower exposes load_config");
    assert!(
        factory.load_config_prefix.is_none(),
        "tuning_tower has no load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "tuning_tower has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "tuning_tower has no add_printer_objects"
    );
    let gcode = Arc::new(FakeGCode::new());
    let normal = Arc::new(FakeTransform::new(vec![0.0, 0.0, 0.0, 0.0]));
    let gcode_move = Arc::new(FakeGCodeMove::new(normal.clone()));
    let gcode_obj: ObjectRef = gcode.clone();
    let gcode_move_obj: ObjectRef = gcode_move.clone();
    let printer: Printer = Arc::new(FakePrinter {
        gcode: gcode_obj,
        gcode_move: gcode_move_obj,
    });
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
    });
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let rust: Arc<TuningTower> = downcast_arc(&obj).expect("downcast to TuningTower");
    // The factory object registered the command with the help text.
    assert_eq!(
        gcode.help(TUNING_TOWER_CMD).as_deref(),
        Some(CMD_TUNING_TOWER_HELP)
    );
    py.exec("p2 = FakePrinter(); c2 = FakeConfig(p2); tt2 = tuning_tower.load_config(c2)")
        .expect("py factory load_config");
    py.exec("p = p2; tt = tt2").expect("swap py objects");
    py_make_gcmd(
        &mut py,
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
    );
    let expected = py.eval("tt.cmd_TUNING_TOWER(gcmd)");
    let gcmd = make_gcmd(
        &gcode,
        "TUNING_TOWER COMMAND=M104 PARAMETER=S START=200 FACTOR=1",
        &[("COMMAND", "M104"), ("PARAMETER", "S"), ("START", "200"), ("FACTOR", "1")],
    );
    let actual = rust.cmd_tuning_tower(&gcmd);
    check_unit("factory cmd", expected, actual);
    let expected = py.eval("tt.is_active()");
    check("factory active", expected, Value::Bool(rust.is_active()));
}

