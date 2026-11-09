//! Parity tests for `extras::input_shaper` (port of
//! `klippy/extras/input_shaper.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): an `InputShaperParams` /
//! `AxisInputShaper` is built on both sides with matched fakes for the
//! printer, config and gcode command, then driven through config
//! parsing, per-type shaper computation, `update`, `report`,
//! `disable/enable_shaping` and the C `init_shaper` validation that
//! `set_shaper_kinematics` relies on. Results are compared exactly
//! (floats by bit value, errors by class and message).

use extras::input_shaper::{
    AxisInputShaper, InputShaper, InputShaperParams, CMD_SET_INPUT_SHAPER_HELP,
    input_shaper_alloc, input_shaper_pulses, input_shaper_set_shaper_params,
    input_shaper_set_sk, input_shaper_update_sk,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts,
    GetFloatListOpts, GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd,
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, GetParser,
    OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{CommandQueue, Mcu};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{Stepper, StepperKinematicsApi, StepperFactory, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, FlushCallback, HomingStateHandle, HomingTargetApi, Kinematics, KinematicsApi,
    LookaheadCallback, MotionQueuingApi, MoveApi, PrinterExtruder, SyncEmitterHandle, ToolHeadApi,
    TrapqAppend,
};
use klippy_api::{EventArg, GCell, ObjectRef};
use klippy_core::motion_handles::alloc_stepper_kinematics;
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{Record, RecordKind, SideEffects, Value, assert_equal};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
from extras import input_shaper, shaper_defs
import gcode as gcode_mod

class CmdError(Exception):
    pass

class FakeConfig:
    error = configparser.Error
    def __init__(self, values=None, section='input_shaper', printer=None):
        self.values = dict(values) if values else {}
        self.section = section
        self.reads = []
        self.printer = printer if printer is not None else FakePrinter()
    def get_printer(self):
        return self.printer
    def get(self, option, default=None):
        self.reads.append(option)
        if option in self.values:
            return self.values[option]
        if default is not None:
            return default
        raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
    def getfloat(self, option, default=None, minval=None, maxval=None):
        self.reads.append(option)
        if option not in self.values:
            if default is not None:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        v = float(self.values[option])
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v

class FakePrinter:
    config_error = configparser.Error
    command_error = CmdError
    def __init__(self, objects=None):
        self.objects = dict(objects) if objects else {}
        self.events = []
    def register_event_handler(self, event, handler):
        self.events.append(event)
    def lookup_object(self, name, default='__sentinel__'):
        if name in self.objects:
            return self.objects[name]
        if default == '__sentinel__':
            raise configparser.Error("Failed to lookup " + name)
        return default

class FakeKinematics:
    def __init__(self, steppers=None):
        self.steppers = list(steppers) if steppers else []
    def get_steppers(self):
        return self.steppers

class FakeToolHead:
    def __init__(self, kinematics=None):
        self.kinematics = kinematics if kinematics is not None else FakeKinematics()
        self.flushes = 0
    def flush_step_generation(self):
        self.flushes += 1
    def get_kinematics(self):
        return self.kinematics

class FakeMotionQueuing:
    def __init__(self):
        self.checks = 0
    def check_step_generation_scan_windows(self):
        self.checks += 1

def make_input_shaper(values=None, objects=None):
    g = FakeGcode()
    objs = {'gcode': g}
    if objects is not None:
        objs.update(objects)
    printer = FakePrinter(objs)
    cfg = FakeConfig(values if values else {}, printer=printer)
    obj = input_shaper.InputShaper(cfg)
    return obj, g, printer, cfg

class FakeGcode:
    error = CmdError
    def __init__(self):
        self.responses = []
        self.commands = []
    def register_command(self, cmd, func, desc=None):
        self.commands.append([cmd, desc])
    def respond_info(self, msg, log=True):
        self.responses.append(msg)
    def respond_raw(self, msg):
        self.responses.append(msg)

def make_gcmd(params):
    cmdline = 'SET_INPUT_SHAPER'
    for k, v in params.items():
        cmdline += ' %s=%s' % (k, v)
    g = FakeGcode()
    gcmd = gcode_mod.GCodeCommand(g, 'SET_INPUT_SHAPER', cmdline, dict(params), False)
    return gcmd, g
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn bound_fmt(v: f64) -> String {
    if v.is_finite() && v == v.trunc() && v.abs() < 1e16 {
        format!("{:.1}", v)
    } else {
        format!("{v}")
    }
}

#[derive(Clone, Default)]
struct FakePrinter {
    objects: Vec<(String, ObjectRef)>,
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
    fn update_error_msg(&self, _: &str, _: &str) {}
    fn add_object(&self, _: &str, _: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some((_, obj)) = self.objects.iter().find(|(n, _)| n == name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Failed to lookup {name}"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.clone()
    }
    fn load_object(&self, _: &Config, _: &str, _: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _: &str, _: Option<&str>, _: bool) {}
    fn invoke_shutdown(&self, _: &str, _: PyValue) {}
    fn invoke_async_shutdown(&self, _: &str, _: PyValue) {}
    fn register_event_handler(&self, _: &str, _: EventCallback) {}
    fn send_event(&self, _: &str, _: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _: &str) {}
}

struct FakeConfig {
    printer: Printer,
    values: Vec<(String, String)>,
    get_reads: Arc<GCell<Vec<String>>>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "input_shaper".to_string()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.get_reads.borrow_mut().push(option.to_string());
        match self.values.iter().find(|(k, _)| k == option) {
            Some((_, v)) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section 'input_shaper' must be specified"
                ))),
            },
        }
    }
    fn getint(&self, _: &str, _: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.get_reads.borrow_mut().push(option.to_string());
        match self.values.iter().find(|(k, _)| k == option) {
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section 'input_shaper' must be specified"
                ))),
            },
            Some((_, raw)) => {
                let v: f64 = raw.parse().unwrap_or(f64::NAN);
                if let Some(mn) = opts.minval {
                    if v < mn {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section 'input_shaper' must have minimum of {}",
                            bound_fmt(mn)
                        )));
                    }
                }
                if let Some(mx) = opts.maxval {
                    if v > mx {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section 'input_shaper' must have maximum of {}",
                            bound_fmt(mx)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, _: &str, _: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _: &str, _: &PyValue, _: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _: &str, _: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _: &str, _: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _: &str, _: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _: &str, _: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _: &str) -> Config {
        panic!("getsection is not used by input_shaper")
    }
    fn has_section(&self, _: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _: &str, _: Option<PyValue>) {}
}

fn make_config(values: &[(&str, &str)]) -> (Config, Arc<GCell<Vec<String>>>) {
    let printer: Printer = Arc::new(FakePrinter::default());
    let reads = Arc::new(GCell::new(Vec::new()));
    let config: Config = Arc::new(FakeConfig {
        printer,
        values: values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        get_reads: reads.clone(),
    });
    (config, reads)
}

/// Minimal `GCodeCommand` fake mirroring `klippy/gcode.py:24` errors.
struct FakeGcmd {
    params: indexmap::IndexMap<String, String>,
    responses: GCell<Vec<String>>,
    commandline: String,
}

impl FakeGcmd {
    fn new(params: &[(&str, &str)]) -> Self {
        let mut map = indexmap::IndexMap::new();
        let mut commandline = "SET_INPUT_SHAPER".to_string();
        for (k, v) in params {
            map.insert(k.to_string(), v.to_string());
            commandline.push_str(&format!(" {k}={v}"));
        }
        FakeGcmd {
            params: map,
            responses: GCell::new(Vec::new()),
            commandline,
        }
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
            },
            Some(v) => match opts.parser {
                GetParser::Str => Ok(PyValue::Str(v.clone())),
                GetParser::Int => Err(KlippyError::command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, v
                ))),
                GetParser::Float => match v.parse::<f64>() {
                    Ok(f) => Ok(PyValue::Float(f)),
                    Err(_) => Err(KlippyError::command(format!(
                        "Error on '{}': unable to parse {}",
                        self.commandline, v
                    ))),
                },
                GetParser::IntAutoBase => Err(KlippyError::command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, v
                ))),
            },
        }
    }
    fn get_int(&self, _: &str, _: GGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
            },
            Some(v) => match v.parse::<f64>() {
                Ok(parsed) => {
                    if let Some(mn) = opts.minval {
                        if parsed < mn {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {} must have minimum of {}",
                                self.commandline,
                                name,
                                bound_fmt(mn)
                            )));
                        }
                    }
                    if let Some(mx) = opts.maxval {
                        if parsed > mx {
                            return Err(KlippyError::command(format!(
                                "Error on '{}': {} must have maximum of {}",
                                self.commandline,
                                name,
                                bound_fmt(mx)
                            )));
                        }
                    }
                    Ok(Some(parsed))
                }
                Err(_) => Err(KlippyError::command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, v
                ))),
            },
        }
    }
    fn get_command(&self) -> String {
        "SET_INPUT_SHAPER".to_string()
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
    fn respond_info(&self, msg: &str, _log: bool) {
        self.responses.borrow_mut().push(msg.to_string());
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}
// ---------------------------------------------------------------------------
// Printer-object fakes for the `InputShaper` object tests.
// ---------------------------------------------------------------------------

/// The `gcode` object: records `register_command(cmd, desc)`.
struct FakeGcodeDispatch {
    commands: GCell<Vec<(String, Option<String>)>>,
}

impl GCodeApi for FakeGcodeDispatch {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.commands
            .borrow_mut()
            .push((cmd.to_string(), opts.desc.clone()));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _: &str,
        _: &str,
        _: Option<&str>,
        _: GCodeCommandHandler,
        _: Option<&str>,
    ) -> KResult<()> {
        panic!("register_mux_command is not used by input_shaper")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _: f64) -> PyValue {
        panic!("gcode get_status is not used by input_shaper")
    }
    fn register_output_handler(&self, _: OutputCallback) {
        panic!("register_output_handler is not used by input_shaper")
    }
    fn request_restart(&self, _: &str) -> KResult<()> {
        panic!("request_restart is not used by input_shaper")
    }
    fn run_script(&self, _: &str) -> KResult<()> {
        panic!("run_script is not used by input_shaper")
    }
    fn run_script_from_command(&self, _: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by input_shaper")
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("get_mutex is not used by input_shaper")
    }
    fn create_gcode_command(&self, _: &str, _: &str, _: &IndexMap<String, String>) -> GCmd {
        panic!("create_gcode_command is not used by input_shaper")
    }
    fn respond_raw(&self, _: &str) {
        panic!("respond_raw is not used by input_shaper")
    }
    fn respond_info(&self, _: &str, _: bool) {
        panic!("respond_info is not used by input_shaper")
    }
    fn is_traditional_gcode(&self, _: &str) -> bool {
        panic!("is_traditional_gcode is not used by input_shaper")
    }
    fn cmd_m112(&self, _: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_M112 is not used by input_shaper")
    }
    fn process_commands(&self, _: &[String], _: bool) -> KResult<()> {
        panic!("process_commands is not used by input_shaper")
    }
}

klippy_api::printer_object!(FakeGcodeDispatch; dyn GCodeApi);

/// Kinematics with no steppers (stepper-carrying flows need the chelper,
/// which the oracle cannot run).
struct FakeKinematics;

impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn calc_position(&self, _: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        panic!("calc_position is not used by input_shaper")
    }
    fn set_position(&self, _: Vec<f64>, _: &str) -> KResult<()> {
        panic!("kinematics set_position is not used by input_shaper")
    }
    fn check_move(&self, _: &dyn MoveApi) -> KResult<()> {
        panic!("check_move is not used by input_shaper")
    }
    fn home(&self, _: HomingStateHandle) -> KResult<()> {
        panic!("home is not used by input_shaper")
    }
    fn get_status(&self, _: f64) -> PyValue {
        panic!("kinematics get_status is not used by input_shaper")
    }
    fn clear_homing_state(&self, _: &str) {
        panic!("clear_homing_state is not used by input_shaper")
    }
}

/// The `toolhead` object: counts `flush_step_generation`.
struct FakeToolhead {
    flushes: GCell<usize>,
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by input_shaper")
    }
    fn set_position(&self, _: Vec<f64>, _: &str) -> KResult<()> {
        panic!("toolhead set_position is not used by input_shaper")
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics)
    }
    fn flush_step_generation(&self) -> KResult<()> {
        *self.flushes.borrow_mut() += 1;
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used by input_shaper")
    }
    fn dwell(&self, _: f64) {
        panic!("dwell is not used by input_shaper")
    }
    fn drip_move(&self, _: Vec<f64>, _: f64, _: &Completion<f64>) -> KResult<()> {
        panic!("drip_move is not used by input_shaper")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _: Vec<f64>, _: f64) -> KResult<()> {
        panic!("move is not used by input_shaper")
    }
    fn manual_move(&self, _: Vec<Option<f64>>, _: f64) -> KResult<()> {
        panic!("manual_move is not used by input_shaper")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by input_shaper")
    }
    fn get_status(&self, _: f64) -> PyValue {
        panic!("toolhead get_status is not used by input_shaper")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by input_shaper")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by input_shaper")
    }
    fn set_max_velocities(
        &self,
        _: Option<f64>,
        _: Option<f64>,
        _: Option<f64>,
        _: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by input_shaper")
    }
    fn register_lookahead_callback(&self, _: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by input_shaper")
    }
    fn check_busy(&self, _: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by input_shaper")
    }
    fn limit_next_junction_speed(&self, _: f64) {
        panic!("limit_next_junction_speed is not used by input_shaper")
    }
    fn set_extruder(&self, _: PrinterExtruder, _: f64) {
        panic!("set_extruder is not used by input_shaper")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by input_shaper")
    }
    fn add_extra_axis(&self, _: ExtraAxis, _: f64) {
        panic!("add_extra_axis is not used by input_shaper")
    }
    fn remove_extra_axis(&self, _: &ExtraAxis) {
        panic!("remove_extra_axis is not used by input_shaper")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by input_shaper")
    }
    fn stats(&self, _: f64) -> (bool, String) {
        panic!("toolhead stats is not used by input_shaper")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by input_shaper")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

/// The `motion_queuing` object: counts `check_step_generation_scan_windows`.
struct FakeMotionQueuing {
    checks: GCell<usize>,
}

impl MotionQueuingApi for FakeMotionQueuing {
    fn allocate_trapq(&self) -> Trapq {
        panic!("allocate_trapq is not used by input_shaper")
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        panic!("lookup_trapq_append is not used by input_shaper")
    }
    fn wipe_trapq(&self, _: &Trapq) {
        panic!("wipe_trapq is not used by input_shaper")
    }
    fn allocate_syncemitter(&self, _: Mcu, _: &str, _: bool) -> KResult<SyncEmitterHandle> {
        panic!("allocate_syncemitter is not used by input_shaper")
    }
    fn queue_syncemitter_msg(&self, _: &SyncEmitterHandle, _: u64, _: &[u32]) {
        panic!("queue_syncemitter_msg is not used by input_shaper")
    }
    fn setup_mcu_movequeue(&self, _: Mcu, _: CommandQueue, _: i64) {
        panic!("setup_mcu_movequeue is not used by input_shaper")
    }
    fn register_flush_callback(&self, _: FlushCallback, _: bool) {
        panic!("register_flush_callback is not used by input_shaper")
    }
    fn unregister_flush_callback(&self, _: &FlushCallback) {
        panic!("unregister_flush_callback is not used by input_shaper")
    }
    fn get_kin_flush_delay(&self) -> f64 {
        panic!("get_kin_flush_delay is not used by input_shaper")
    }
    fn check_step_generation_scan_windows(&self) {
        *self.checks.borrow_mut() += 1;
    }
    fn flush_all_steps(&self) -> KResult<()> {
        panic!("flush_all_steps is not used by input_shaper")
    }
    fn calc_step_gen_restart(&self, _: f64) -> f64 {
        panic!("calc_step_gen_restart is not used by input_shaper")
    }
    fn note_mcu_movequeue_activity(&self, _: f64, _: bool) {
        panic!("note_mcu_movequeue_activity is not used by input_shaper")
    }
    fn drip_update_time(&self, _: f64, _: f64, _: &Completion<f64>) -> KResult<()> {
        panic!("drip_update_time is not used by input_shaper")
    }
    fn check_drip_timing(&self) -> Option<f64> {
        panic!("check_drip_timing is not used by input_shaper")
    }
}

klippy_api::printer_object!(FakeMotionQueuing; dyn MotionQueuingApi);

/// Stand-in for an unrelated printer object (e.g. `dual_carriage`).
struct TestObject;

klippy_api::printer_object!(TestObject);

fn make_config_with_objects(
    values: &[(&str, &str)],
    objects: Vec<(String, ObjectRef)>,
) -> (Config, Arc<GCell<Vec<String>>>) {
    let printer: Printer = Arc::new(FakePrinter { objects });
    let reads = Arc::new(GCell::new(Vec::new()));
    let config: Config = Arc::new(FakeConfig {
        printer,
        values: values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        get_reads: reads.clone(),
    });
    (config, reads)
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("input_shaper fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping input_shaper oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "input_shaper".to_string(),
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

fn triple_value(triple: (i64, Vec<f64>, Vec<f64>)) -> Value {
    let (n, a, t) = triple;
    Value::Tuple(vec![
        Value::int(n),
        Value::List(a.into_iter().map(Value::float).collect()),
        Value::List(t.into_iter().map(Value::float).collect()),
    ])
}

fn check_triple(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<(i64, Vec<f64>, Vec<f64>)>,
) {
    let actual = match actual {
        Ok(t) => t,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, expected, triple_value(actual));
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust must fail"),
        Err(e) => e,
    };
    assert_eq!(
        actual.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        actual.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

fn status_value(status: indexmap::IndexMap<String, String>) -> Value {
    Value::List(
        status
            .iter()
            .map(|(k, v)| {
                Value::Tuple(vec![Value::str(k), Value::str(v)])
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn params_defaults_and_status() {
    let Some(mut py) = oracle() else { return };
    py.exec("pcfg = FakeConfig(); pp = input_shaper.InputShaperParams('x', pcfg)")
        .expect("py params");
    let expected = py.eval("list(pp.get_status().items())");
    let (config, _) = make_config(&[]);
    let params = InputShaperParams::new("x", &config).expect("rust params");
    check("params status", expected, status_value(params.get_status()));
    check(
        "params fields",
        py.eval("[pp.shaper_type, pp.shaper_freq, pp.damping_ratio]"),
        Value::List(vec![
            Value::str(params.shaper_type()),
            Value::float(params.shaper_freq()),
            Value::float(params.damping_ratio()),
        ]),
    );
}

#[test]
fn params_config_values_and_reads() {
    let Some(mut py) = oracle() else { return };
    let cases: &[(&str, &str, &[(&str, &str)])] = &[
        (
            "zv config",
            "x",
            &[("shaper_type", "zv"), ("shaper_freq_x", "31.5"), ("damping_ratio_x", "0.2")],
        ),
        (
            "per-axis type",
            "y",
            &[("shaper_type", "mzv"), ("shaper_type_y", "2hump_ei"), ("shaper_freq_y", "41")],
        ),
        ("freq only", "z", &[("shaper_freq_z", "28.25")]),
    ];
    for (idx, (label, axis, values)) in cases.iter().enumerate() {
        let py_values = values
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!(
            "pcfg{idx} = FakeConfig({{{py_values}}}); pp{idx} = input_shaper.InputShaperParams('{axis}', pcfg{idx})"
        ))
        .expect("py params");
        let expected = py.eval(&format!(
            "list(pp{idx}.get_status().items()) + [pp{idx}.shaper_type, pp{idx}.shaper_freq, pp{idx}.damping_ratio]"
        ));
        let (config, _) = make_config(values);
        let params = InputShaperParams::new(axis, &config).expect("rust params");
        let got: Vec<Value> = params
            .get_status()
            .iter()
            .map(|(k, v)| Value::Tuple(vec![Value::str(k), Value::str(v)]))
            .chain([
                Value::str(params.shaper_type()),
                Value::float(params.shaper_freq()),
                Value::float(params.damping_ratio()),
            ])
            .collect();
        check(label, expected, Value::List(got));
    }
}

#[test]
fn config_read_order_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "pcfg = FakeConfig({'shaper_type': 'zv', 'shaper_freq_x': '30'}); \
         pp = input_shaper.InputShaperParams('x', pcfg)",
    )
    .expect("py params");
    let expected = py.eval("pcfg.reads");
    let (config, reads) = make_config(&[("shaper_type", "zv"), ("shaper_freq_x", "30")]);
    let _ = InputShaperParams::new("x", &config).expect("rust params");
    let got: Vec<Value> = reads.borrow().iter().map(|s| Value::str(s)).collect();
    // The ConfigApi get/getfloat order matches the reference `get`/`getfloat`
    // order, so the read sequences must be identical.
    check("read order", expected, Value::List(got));
}

#[test]
fn unsupported_type_and_validation_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("pcfg = FakeConfig({'shaper_type': 'bogus'})").expect("py cfg");
    let expected = py.eval("input_shaper.InputShaperParams('x', pcfg)");
    let (config, _) = make_config(&[("shaper_type", "bogus")]);
    let err = InputShaperParams::new("x", &config).expect_err("rust must fail");
    check_err("unsupported type", expected, err);

    py.exec("pcfg2 = FakeConfig({'shaper_type': 'mzv(2,0.6)', 'shaper_freq_x': '30'})")
        .expect("py cfg 2");
    let expected = py.eval("input_shaper.InputShaperParams('x', pcfg2)");
    let (config, _) = make_config(&[("shaper_type", "mzv(2,0.6)"), ("shaper_freq_x", "30")]);
    let err = InputShaperParams::new("x", &config).expect_err("rust must fail");
    check_err("init validation", expected, err);

    // Damping above the shaper's max: config bounds error.
    py.exec("pcfg3 = FakeConfig({'shaper_type': 'ei', 'shaper_freq_x': '30', 'damping_ratio_x': '0.5'})")
        .expect("py cfg 3");
    let expected = py.eval("input_shaper.InputShaperParams('x', pcfg3)");
    let (config, _) = make_config(&[
        ("shaper_type", "ei"),
        ("shaper_freq_x", "30"),
        ("damping_ratio_x", "0.5"),
    ]);
    let err = InputShaperParams::new("x", &config).expect_err("rust must fail");
    check_err("damping bounds", expected, err);
}

#[test]
fn shaper_value_table() {
    let Some(mut py) = oracle() else { return };
    let cases: &[(&str, &str, &str)] = &[
        ("zv", "30", "0.1"),
        ("zvd", "30", "0.15"),
        ("mzv", "38.5", "0.1"),
        ("mzv(5,0.6)", "22.2", "0.1"),
        ("mzv(n=5,tau=0.05)", "30", "0.1"),
        ("ei", "33.3", "0.11"),
        ("ei(v_tol=0.05)", "40", "0.2"),
        ("2hump_ei", "41", "0.1"),
        ("3hump_ei", "50", "0.1"),
    ];
    for (idx, (name, freq, damping)) in cases.iter().enumerate() {
        // Some combinations are invalid in the reference (e.g. a
        // negative-valued shaper): construction raises there, so record
        // the exception instead of derailing the table and compare the
        // error with the port's.
        py.exec(&format!(
            "try:\n    pp{idx} = input_shaper.InputShaperParams('x', FakeConfig({{'shaper_type': '{name}', 'shaper_freq_x': '{freq}', 'damping_ratio_x': '{damping}'}}))\n    perr{idx} = None\nexcept Exception as e:\n    perr{idx} = (type(e).__name__, str(e))"
        ))
        .expect("py params");
        let how = py
            .eval(&format!("'err' if perr{idx} is not None else 'ok'"))
            .expect("py validity");
        let (config, _) = make_config(&[
            ("shaper_type", name),
            ("shaper_freq_x", freq),
            ("damping_ratio_x", damping),
        ]);
        if how == recording::Value::str("err") {
            let (class_name, message) = match py
                .eval(&format!("perr{idx}"))
                .expect("py error tuple")
            {
                recording::Value::Tuple(items) => match (&items[0], &items[1]) {
                    (recording::Value::Str(c), recording::Value::Str(m)) => {
                        (c.clone(), m.clone())
                    }
                    other => panic!("unexpected py error tuple: {other:?}"),
                },
                other => panic!("unexpected py error tuple: {other:?}"),
            };
            let err = InputShaperParams::new("x", &config).expect_err("rust must fail");
            assert_eq!(err.class_name(), class_name.as_str(), "shaper {name}: class");
            assert_eq!(err.message(), message.as_str(), "shaper {name}: message");
            continue;
        }
        let expected = py.eval(&format!("pp{idx}.get_shaper()"));
        let params = InputShaperParams::new("x", &config).expect("rust params");
        let label = format!("shaper value {name}");
        check_triple(&label, expected, params.get_shaper(None, None, None, None));
    }
}

#[test]
fn update_paths() {
    let Some(mut py) = oracle() else { return };
    py.exec("pcfg = FakeConfig(); pp = input_shaper.InputShaperParams('x', pcfg)")
        .expect("params");
    py.exec(
        "gc, g = make_gcmd({'SHAPER_FREQ_X': '22.2', 'DAMPING_RATIO_X': '0.1', 'SHAPER_TYPE_X': 'mzv(5,0.6)'}); \
         pp.update(gc)",
    )
    .expect("py update");
    let expected = py.eval("list(pp.get_shaper()) + list(pp.get_status().values())");
    let (config, _) = make_config(&[]);
    let mut params = InputShaperParams::new("x", &config).expect("rust params");
    let cmd: GCmd = Arc::new(FakeGcmd::new(&[
        ("SHAPER_FREQ_X", "22.2"),
        ("DAMPING_RATIO_X", "0.1"),
        ("SHAPER_TYPE_X", "mzv(5,0.6)"),
    ]));
    params.update(&cmd).expect("rust update");
    let (n, a, t) = params.get_shaper(None, None, None, None).expect("rust shaper");
    let mut got = vec![
        Value::int(n),
        Value::List(a.into_iter().map(Value::float).collect()),
        Value::List(t.into_iter().map(Value::float).collect()),
    ];
    for v in params.get_status().values() {
        got.push(Value::str(v));
    }
    check("update SET_INPUT_SHAPER", expected, Value::List(got));

    // Global SHAPER_TYPE (no per-axis suffix).
    py.exec(
        "gc, g = make_gcmd({'SHAPER_TYPE': 'zv', 'SHAPER_FREQ_X': '25', 'DAMPING_RATIO_X': '0.2'}); \
         pp.update(gc)",
    )
    .expect("py update global");
    let expected = py.eval("list(pp.get_shaper()) + list(pp.get_status().values())");
    let cmd: GCmd = Arc::new(FakeGcmd::new(&[
        ("SHAPER_TYPE", "zv"),
        ("SHAPER_FREQ_X", "25"),
        ("DAMPING_RATIO_X", "0.2"),
    ]));
    params.update(&cmd).expect("rust update global");
    let (n, a, t) = params.get_shaper(None, None, None, None).expect("rust shaper");
    let mut got = vec![
        Value::int(n),
        Value::List(a.into_iter().map(Value::float).collect()),
        Value::List(t.into_iter().map(Value::float).collect()),
    ];
    for v in params.get_status().values() {
        got.push(Value::str(v));
    }
    check("update global type", expected, Value::List(got));
}

#[test]
fn update_error_paths() {
    let Some(mut py) = oracle() else { return };
    py.exec("pcfg = FakeConfig(); pp = input_shaper.InputShaperParams('x', pcfg)")
        .expect("params");
    py.exec("gc, g = make_gcmd({'SHAPER_TYPE': 'Bogus'})").expect("gc");
    let expected = py.eval("pp.update(gc)");
    let (config, _) = make_config(&[]);
    let mut params = InputShaperParams::new("x", &config).expect("rust params");
    let cmd: GCmd = Arc::new(FakeGcmd::new(&[("SHAPER_TYPE", "Bogus")]));
    check_err("unsupported update type", expected, params.update(&cmd).expect_err("must fail"));

    py.exec("gc, g = make_gcmd({'SHAPER_TYPE': 'ei', 'DAMPING_RATIO_X': '0.5'})")
        .expect("gc");
    let expected = py.eval("pp.update(gc)");
    let cmd: GCmd = Arc::new(FakeGcmd::new(&[
        ("SHAPER_TYPE", "ei"),
        ("DAMPING_RATIO_X", "0.5"),
    ]));
    check_err("too high damping", expected, params.update(&cmd).expect_err("must fail"));

    // Bad shaper arguments: the init error is wrapped as a CommandError.
    // The configured frequency must be nonzero here: `get_shaper` returns
    // the empty shaper without validating when `self.shaper_freq` is
    // zero (`input_shaper.py:51`), so both sides succeed then.
    py.exec(
        "pcfg2 = FakeConfig({'shaper_freq_x': '30'}); pp2 = input_shaper.InputShaperParams('x', pcfg2); \
         gc, g = make_gcmd({'SHAPER_TYPE': 'mzv(2,0.6)', 'SHAPER_FREQ_X': '30'})",
    )
    .expect("gc");
    let expected = py.eval("pp2.update(gc)");
    let (config, _) = make_config(&[("shaper_freq_x", "30")]);
    let mut params2 = InputShaperParams::new("x", &config).expect("rust params2");
    let cmd: GCmd = Arc::new(FakeGcmd::new(&[
        ("SHAPER_TYPE", "mzv(2,0.6)"),
        ("SHAPER_FREQ_X", "30"),
    ]));
    check_err("bad args on update", expected, params2.update(&cmd).expect_err("must fail"));
}

#[test]
fn axis_state_and_report() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "pcfg = FakeConfig({'shaper_freq_x': '30'}); sc = input_shaper.AxisInputShaper('x', pcfg)",
    )
    .expect("py axis");
    let expected = py.eval("[sc.get_shaper(), sc.is_enabled(), sc.get_name()]");
    let (config, _) = make_config(&[("shaper_freq_x", "30")]);
    let axis = AxisInputShaper::new("x", &config).expect("rust axis");
    check(
        "axis initial",
        expected,
        Value::List(vec![
            triple_value(axis.get_shaper()),
            Value::Bool(axis.is_enabled()),
            Value::str(&axis.get_name()),
        ]),
    );

    // disable then enable restores.
    py.exec("sc.disable_shaping(); sc.enable_shaping()").expect("py toggle");
    let expected = py.eval("[sc.get_shaper(), sc.is_enabled()]");
    axis.disable_shaping();
    axis.enable_shaping();
    check(
        "axis disable/enable",
        expected,
        Value::List(vec![
            triple_value(axis.get_shaper()),
            Value::Bool(axis.is_enabled()),
        ]),
    );

    // report: response text comes from params status.
    py.exec("gc, g = make_gcmd({}); sc.report(gc)").expect("py report");
    let expected = py.eval("g.responses[0]");
    let cmd = Arc::new(FakeGcmd::new(&[]));
    let cmd: GCmd = cmd;
    axis.report(&cmd);
    let report_str = axis.get_status().iter().map(|(k, v)| format!("{k}_x:{v}")).collect::<Vec<_>>().join(" ");
    check("axis report", expected, Value::str(&report_str));
}

#[test]
fn c_validation_and_update() {
    let sk = input_shaper_alloc().expect("alloc wrapper");
    let orig = alloc_stepper_kinematics("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'x'])])
        .expect("alloc original");
    assert_eq!(input_shaper_set_sk(&sk, Some(&orig)).expect("set sk"), 0);
    assert_eq!(
        input_shaper_set_shaper_params(&sk, b'x', 2, &[0.5, 0.5], &[0., 0.01]).expect("set params"),
        0
    );
    assert_eq!(input_shaper_pulses(&sk).unwrap()[0].num_pulses, 2);
    // Non-increasing times leave the previous pulses untouched.
    assert_eq!(
        input_shaper_set_shaper_params(&sk, b'x', 2, &[0.5, 0.5], &[0.01, 0.]).expect("bad times"),
        -1
    );
    assert_eq!(input_shaper_pulses(&sk).unwrap()[0].num_pulses, 2);
    // Too many pulses clear the axis, exactly like the C.
    let big_a = vec![0.1; 11];
    let big_t: Vec<f64> = (0..11).map(|i| i as f64 * 0.001).collect();
    assert_eq!(
        input_shaper_set_shaper_params(&sk, b'x', 11, &big_a, &big_t).expect("too many"),
        -1
    );
    assert_eq!(input_shaper_pulses(&sk).unwrap()[0].num_pulses, 0);
    // An axis inactive on the wrapped kinematics is ignored with status 0.
    assert_eq!(
        input_shaper_set_shaper_params(&sk, b'y', 2, &[0.5, 0.5], &[0., 0.01]).expect("inactive axis"),
        0
    );
    assert_eq!(input_shaper_pulses(&sk).unwrap()[1].num_pulses, 0);
    assert_eq!(
        input_shaper_set_shaper_params(&sk, b'w', 0, &[], &[]).expect("unknown axis"),
        -1
    );
    // An axis letter below 'x' is rejected as well.
    assert_eq!(
        input_shaper_set_shaper_params(&sk, b'v', 0, &[], &[]).expect("axis below x"),
        -1
    );
    input_shaper_update_sk(&sk).expect("update sk");
    // A missing original fails the wrap, and an original with no active
    // axis is rejected by the C like `input_shaper_set_sk` rejects it.
    let sk2 = input_shaper_alloc().expect("alloc wrapper 2");
    assert_eq!(input_shaper_set_sk(&sk2, None).expect("missing original"), -1);
    let flat = alloc_stepper_kinematics("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'w'])])
        .expect("alloc flat original");
    let sk3 = input_shaper_alloc().expect("alloc wrapper 3");
    assert_eq!(input_shaper_set_sk(&sk3, Some(&flat)).expect("flat original"), -1);
    let _: &dyn StepperKinematicsApi = &*sk;
}

#[test]
fn help_text_matches() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("input_shaper.InputShaper.cmd_SET_INPUT_SHAPER_help");
    check(
        "help text",
        expected,
        Value::str(CMD_SET_INPUT_SHAPER_HELP),
    );
    let factory = extras::input_shaper::module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
}

#[test]
fn input_shaper_object_paths() {
    let Some(mut py) = oracle() else { return };
    // SET_INPUT_SHAPER with no parameters: registers the command and
    // reports the (disabled) x and y shapers.
    py.exec("obj, greg, printer, cfg = make_input_shaper()").expect("py object");
    py.exec("gc, g = make_gcmd({}); obj.cmd_SET_INPUT_SHAPER(gc)").expect("py cmd");
    let expected_responses = py.eval("g.responses");
    let expected_commands = py.eval("greg.commands");
    let gcode = Arc::new(FakeGcodeDispatch { commands: GCell::new(Vec::new()) });
    let (config, _) = make_config_with_objects(
        &[],
        vec![("gcode".to_string(), gcode.clone() as ObjectRef)],
    );
    let obj = InputShaper::new(&config).expect("rust object");
    let cmd = Arc::new(FakeGcmd::new(&[]));
    let gcmd: GCmd = cmd.clone();
    obj.cmd_set_input_shaper(&gcmd).expect("rust cmd");
    let rust_responses: Vec<Value> = cmd
        .responses
        .borrow()
        .iter()
        .map(|s| Value::str(s))
        .collect();
    check("empty cmd responses", expected_responses, Value::List(rust_responses));
    let rust_commands: Vec<Value> = gcode
        .commands
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
        .collect();
    check("command registration", expected_commands, Value::List(rust_commands));

    // SET_INPUT_SHAPER with parameters before `connect()` raises the
    // unbound-toolhead AttributeError on both sides.
    py.exec("gc2, g2 = make_gcmd({'SHAPER_FREQ_X': '25'})").expect("py gc2");
    let expected = py.eval("obj.cmd_SET_INPUT_SHAPER(gc2)");
    let cmd: GCmd = Arc::new(FakeGcmd::new(&[("SHAPER_FREQ_X", "25")]));
    let err = obj.cmd_set_input_shaper(&cmd).expect_err("rust must fail");
    check_err("cmd before connect", expected, err);

    // `connect()` with stepper-free kinematics flushes and checks the
    // step-generation scan windows.
    py.exec(
        "th = FakeToolHead(); mq = FakeMotionQueuing(); \
         obj2, greg2, printer2, cfg2 = make_input_shaper({}, {'toolhead': th, 'motion_queuing': mq}); \
         obj2.connect()",
    )
    .expect("py connect");
    let expected = py.eval("[th.flushes, mq.checks]");
    let toolhead = Arc::new(FakeToolhead { flushes: GCell::new(0) });
    let motion = Arc::new(FakeMotionQueuing { checks: GCell::new(0) });
    let (config, _) = make_config_with_objects(
        &[],
        vec![
            ("gcode".to_string(), gcode.clone() as ObjectRef),
            ("toolhead".to_string(), toolhead.clone() as ObjectRef),
            ("motion_queuing".to_string(), motion.clone() as ObjectRef),
        ],
    );
    let obj = InputShaper::new(&config).expect("rust object 2");
    obj.connect().expect("rust connect");
    check(
        "connect flush and checks",
        expected,
        Value::List(vec![
            Value::int(*toolhead.flushes.borrow() as i64),
            Value::int(*motion.checks.borrow() as i64),
        ]),
    );

    // `connect()` with `dual_carriage` present and an enabled shaper fails.
    py.exec(
        "dual = object(); \
         obj3, greg3, printer3, cfg3 = make_input_shaper({'shaper_freq_x': '30'}, {'dual_carriage': dual})",
    )
    .expect("py object dual");
    let expected = py.eval("obj3.connect()");
    let dual: ObjectRef = Arc::new(TestObject) as ObjectRef;
    let (config, _) = make_config_with_objects(
        &[("shaper_freq_x", "30")],
        vec![
            ("gcode".to_string(), gcode.clone() as ObjectRef),
            ("dual_carriage".to_string(), dual),
        ],
    );
    let obj = InputShaper::new(&config).expect("rust object dual");
    let err = obj.connect().expect_err("rust must fail");
    check_err("connect with dual carriage", expected, err);
}
