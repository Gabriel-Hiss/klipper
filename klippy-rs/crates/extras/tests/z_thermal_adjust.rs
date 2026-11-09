use std::sync::Arc;

use extras::heaters::{Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback, TemperatureSensorApi};
use extras::z_thermal_adjust::{module_factory, ZThermalAdjuster};
use indexmap::IndexMap;
use klippy_api::config::*;
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GetFloatOpts as GFloat, GetIntOpts as GInt, GetOpts, MoveTransform, MoveTransformApi, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::*;
use klippy_api::toolhead::*;
use klippy_api::{downcast_arc, EventArg, GCell, KResult, KlippyError, ObjectRef};
use pycompat::text::{float_from_string, int_from_string};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

const PYTHON: &str = r#"
import configparser
import configfile
import gcode
import extras.z_thermal_adjust as module
trace = []

def capture(fn):
    try:
        return ('ok', fn())
    except Exception as exc:
        return ('err', type(exc).__name__, str(exc))

class Config(configfile.ConfigWrapper):
    def getfloat(self, option, default=configfile.sentinel, minval=None,
                 maxval=None, above=None, below=None, note_valid=True):
        d = 'required' if default is configfile.sentinel else default
        trace.append(('config', option, d, minval, maxval, above, below,
                      note_valid))
        return super().getfloat(option, default, minval, maxval, above, below,
                                note_valid)

class Stepper:
    def __init__(self, name): self.name = name
    def get_name(self):
        trace.append(('stepper_name', self.name))
        return self.name
    def get_step_dist(self):
        trace.append(('step_dist', 0.0125))
        return 0.0125

class Kinematics:
    def __init__(self, names): self.steppers = [Stepper(n) for n in names]
    def get_steppers(self):
        trace.append(('steppers',))
        return self.steppers

class Toolhead:
    def __init__(self, names):
        self.kin = Kinematics(names)
        self.position = [0., 0., 0., 0.]
        self.fail = False
    def get_kinematics(self):
        trace.append(('kinematics',))
        return self.kin
    def get_position(self):
        trace.append(('position',))
        return list(self.position)
    def move(self, pos, speed):
        trace.append(('move', list(pos), speed))
        if self.fail: raise gcode.CommandError('move rejected')
        self.position = list(pos)

class GcodeMove:
    def __init__(self, toolhead):
        self.next = toolhead
        self.transform = None
        self.resets = 0
    def set_move_transform(self, transform, force=False):
        trace.append(('transform', force))
        self.transform = transform
        return self.next
    def reset_last_position(self):
        trace.append(('reset',))
        self.resets += 1
        # Exercise re-entry into the transform, as real gcode_move does.
        if self.transform is not None:
            self.transform.get_position()

class Sensor:
    def setup_minmax(self, low, high): trace.append(('minmax', low, high))
    def setup_callback(self, cb):
        trace.append(('callback',))
        self.callback = cb

class Heaters:
    def setup_sensor(self, config):
        trace.append(('sensor', config.get_name()))
        self.sensor = Sensor()
        return self.sensor
    def register_sensor(self, config, sensor, gcode_id=None):
        trace.append(('register_sensor', config.get_name(), gcode_id))
        self.registered = sensor

class Gcode:
    def __init__(self): self.commands = {}
    def register_command(self, name, callback, desc=None, when_not_ready=False):
        trace.append(('command', name, desc, when_not_ready))
        self.commands[name] = callback
    def respond_info(self, msg, log=True): trace.append(('info', msg, log))
    def respond_raw(self, msg): trace.append(('raw', msg))

class Printer:
    def __init__(self, names):
        self.handlers = {}
        t = Toolhead(names)
        self.objects = {'gcode': Gcode(), 'toolhead': t,
                        'gcode_move': GcodeMove(t), 'heaters': Heaters()}
    def lookup_object(self, name):
        trace.append(('lookup', name))
        if name not in self.objects:
            raise configparser.Error("Unknown config object '%s'" % name)
        return self.objects[name]
    def load_object(self, config, name):
        trace.append(('load', name))
        return self.objects[name]
    def register_event_handler(self, event, callback):
        trace.append(('event', event))
        self.handlers.setdefault(event, []).append(callback)
    def send(self, event, *args):
        for cb in self.handlers[event]: cb(*args)

class Homing:
    def __init__(self, axes): self.axes = axes
    def get_axes(self): return self.axes

def build(options=None, names=None):
    global trace, p, cfg, obj
    trace = []
    p = Printer(names if names is not None else ['stepper_x', 'stepper_z', 'stepper_z1'])
    parser = configparser.RawConfigParser()
    parser.add_section('z_thermal_adjust')
    for k, v in (options or {}).items(): parser.set('z_thermal_adjust', k, v)
    cfg = Config(p, parser, {}, 'z_thermal_adjust')
    obj = module.load_config(cfg)

def command(params):
    c = gcode.GCodeCommand(p.objects['gcode'], 'SET_Z_THERMAL_ADJUST',
                          'SET_Z_THERMAL_ADJUST', params, False)
    return p.objects['gcode'].commands['SET_Z_THERMAL_ADJUST'](c)

def snapshot():
    return (obj.get_status(123.), obj.get_temp(123.), obj.stats(123.))
"#;

fn tuple(v: Vec<Value>) -> Value { Value::Tuple(v) }
fn text(s: &str) -> Value { Value::str(s) }
fn floats(v: &[f64]) -> Value { Value::List(v.iter().copied().map(Value::float).collect()) }
fn value(v: PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(v) => Value::Bool(v),
        PyValue::Float(v) => Value::float(v),
        PyValue::Int(v) => {
            let s = v.inner().to_string();
            s.parse::<i64>().map(Value::Int).unwrap_or(Value::BigInt(s))
        }
        PyValue::Str(v) => Value::Str(v),
        PyValue::List(v) => Value::List(v.into_iter().map(value).collect()),
        PyValue::Tuple(v) => tuple(v.into_iter().map(value).collect()),
        PyValue::Dict(v) => Value::Dict(v.into_iter().map(|(k,v)| (value(k),value(v))).collect()),
        _ => panic!("unexpected status type"),
    }
}
fn outcome(result: KResult<Value>) -> Value {
    match result {
        Ok(v) => tuple(vec![text("ok"), v]),
        Err(e) => tuple(vec![text("err"), text(e.class_name()), text(e.message())]),
    }
}
fn compare(py: &mut pyoracle::Oracle, expr: &str, actual: Value) {
    let expected = py.eval(expr).unwrap_or_else(|e| panic!("oracle setup at {expr}: {e:?}"));
    let record = Record { v: 1, unit: "extras.z_thermal_adjust".into(), scenario: expr.into(),
        seq: 0, function: expr.into(), args: vec![], ret: None, exc: None,
        side_effects: SideEffects::default(), object_id: None, kind: RecordKind::Python };
    assert_equal(&record, "ret", &expected, &actual).unwrap();
}
fn oracle() -> Option<pyoracle::Oracle> {
    let Some(mut p) = pyoracle::Oracle::start() else {
        eprintln!("skipping z_thermal_adjust parity: KLIPPY_PYTHON is unset");
        return None;
    };
    p.exec(PYTHON).unwrap();
    Some(p)
}

type Trace = Arc<GCell<Vec<Value>>>;
fn push(trace: &Trace, v: Vec<Value>) { trace.borrow_mut().push(tuple(v)); }
macro_rules! unused {
    ($(fn $name:ident(&self $(, $arg:ident : $ty:ty)* $(,)?) $(-> $out:ty)?;)*) => {
        $(fn $name(&self $(, $arg: $ty)*) $(-> $out)? {
            $(let _ = &$arg;)*
            panic!(concat!("unexpected fake call: ", stringify!($name)))
        })*
    }
}

struct FakeSensor { trace: Trace, callback: GCell<Option<TemperatureCallback>> }
impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, min: f64, max: f64) -> KResult<()> {
        push(&self.trace, vec![text("minmax"), Value::float(min), Value::float(max)]);
        Ok(())
    }
    fn setup_callback(&self, callback: TemperatureCallback) {
        push(&self.trace, vec![text("callback")]);
        self.callback.replace(Some(callback));
    }
    fn get_report_time_delta(&self) -> f64 { panic!("unexpected report time query") }
}
struct FakeHeaters { trace: Trace, sensor: Arc<FakeSensor>, registered: GCell<Option<Arc<dyn SensorApi>>> }
klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);
impl PrinterHeatersApi for FakeHeaters {
    fn setup_sensor(&self, config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        push(&self.trace, vec![text("sensor"), text(&config.get_name())]);
        Ok(self.sensor.clone())
    }
    fn register_sensor(&self, config: &Config, sensor: Arc<dyn SensorApi>, id: Option<String>) -> KResult<()> {
        push(&self.trace, vec![text("register_sensor"), text(&config.get_name()), id.map(Value::Str).unwrap_or(Value::Null)]);
        self.registered.replace(Some(sensor));
        Ok(())
    }
    unused! {
        fn add_sensor_factory(&self, name: &str, factory: SensorFactory);
        fn setup_heater(&self, config: &Config, id: Option<String>) -> KResult<Arc<Heater>>;
        fn get_all_heaters(&self) -> Vec<String>;
        fn lookup_heater(&self, name: &str) -> KResult<Arc<Heater>>;
        fn register_monitor(&self, config: &Config);
        fn turn_off_all_heaters(&self, time: f64);
        fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()>;
    }
}
struct FakeGcode { trace: Trace, commands: GCell<IndexMap<String, GCodeCommandHandler>> }
klippy_api::printer_object!(FakeGcode; dyn GCodeApi);
impl GCodeApi for FakeGcode {
    fn register_command(&self, cmd: &str, func: Option<GCodeCommandHandler>, opts: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> {
        push(&self.trace, vec![text("command"), text(cmd), opts.desc.map(Value::Str).unwrap_or(Value::Null), Value::Bool(opts.when_not_ready)]);
        Ok(self.commands.borrow_mut().insert(cmd.into(), func.unwrap()))
    }
    unused! {
        fn register_mux_command(&self, cmd: &str, key: &str, val: Option<&str>, func: GCodeCommandHandler, desc: Option<&str>) -> KResult<()>;
        fn get_command_help(&self) -> IndexMap<String, String>;
        fn get_status(&self, time: f64) -> PyValue;
        fn register_output_handler(&self, callback: OutputCallback);
        fn request_restart(&self, result: &str) -> KResult<()>;
        fn run_script(&self, script: &str) -> KResult<()>;
        fn run_script_from_command(&self, script: &str) -> KResult<()>;
        fn get_mutex(&self) -> ReactorMutex;
        fn create_gcode_command(&self, cmd: &str, line: &str, params: &IndexMap<String,String>) -> GCmd;
        fn respond_raw(&self, msg: &str);
        fn respond_info(&self, msg: &str, log: bool);
        fn is_traditional_gcode(&self, cmd: &str) -> bool;
        fn cmd_m112(&self, cmd: Option<&GCmd>) -> KResult<()>;
        fn process_commands(&self, cmds: &[String], ack: bool) -> KResult<()>;
    }
}
struct FakeCommand { trace: Trace, params: IndexMap<String,String> }
impl FakeCommand {
    fn bound(&self, name: &str, v: f64, o: &GFloat) -> KResult<()> {
        let checks = [
            (o.minval, IntArgs::MINVAL, o.minval.is_some_and(|l| v < l), "minimum"),
            (o.maxval, IntArgs::MAXVAL, o.maxval.is_some_and(|h| v > h), "maximum"),
        ];
        for (bound, arg, failed, kind) in checks {
            if failed {
                let number = o.ints.value(arg, bound.unwrap()).str();
                return Err(KlippyError::command(format!("Error on 'SET_Z_THERMAL_ADJUST': {name} must have {kind} of {number}")));
            }
        }
        Ok(())
    }
    fn parse_error(&self, raw: &str) -> KlippyError {
        KlippyError::command(format!("Error on 'SET_Z_THERMAL_ADJUST': unable to parse {raw}"))
    }
}
impl GCodeCommandApi for FakeCommand {
    fn get_int(&self, name: &str, o: GInt) -> KResult<Option<i64>> {
        let Some(raw) = self.params.get(name) else { return Ok(o.default.unwrap()); };
        let integer = int_from_string(raw, 10).map_err(|_| self.parse_error(raw))?;
        for (bound, minimum) in [(o.minval, true), (o.maxval, false)] {
            if let Some(bound) = bound {
                let b = pycompat::PyInt::from_i64(bound);
                let fails = if minimum { integer.inner() < b.inner() } else { integer.inner() > b.inner() };
                if fails {
                    let kind = if minimum { "minimum" } else { "maximum" };
                    return Err(KlippyError::command(format!("Error on 'SET_Z_THERMAL_ADJUST': {name} must have {kind} of {bound}")));
                }
            }
        }
        Ok(Some(integer.inner().to_string().parse::<i64>().unwrap()))
    }
    fn get_float(&self, name: &str, o: GFloat) -> KResult<Option<f64>> {
        let Some(raw) = self.params.get(name) else { return Ok(o.default.unwrap()); };
        let n = float_from_string(raw).map_err(|_| self.parse_error(raw))?;
        self.bound(name, n, &o)?;
        Ok(Some(n))
    }
    fn respond_info(&self, msg: &str, log: bool) {
        push(&self.trace, vec![text("info"), text(msg), Value::Bool(log)]);
    }
    unused! {
        fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue>;
        fn get_command(&self) -> String;
        fn get_commandline(&self) -> String;
        fn get_command_parameters(&self) -> IndexMap<String,String>;
        fn get_raw_command_parameters(&self) -> String;
        fn respond_raw(&self, msg: &str);
        fn ack(&self, msg: Option<&str>) -> bool;
    }
}

struct FakeStepper { trace: Trace, name: String }
impl McuStepperApi for FakeStepper {
    fn get_name(&self, _short: Option<bool>) -> String {
        push(&self.trace, vec![text("stepper_name"), text(&self.name)]); self.name.clone()
    }
    fn get_step_dist(&self) -> f64 {
        push(&self.trace, vec![text("step_dist"), Value::float(0.0125)]); 0.0125
    }
    unused! {
        fn get_mcu(&self) -> Mcu;
        fn get_oid(&self) -> i64;
        fn units_in_radians(&self) -> bool;
        fn get_pulse_duration(&self) -> (Option<f64>,bool);
        fn setup_default_pulse_duration(&self, duration: f64, both: bool);
        fn setup_itersolve(&self, func: &str, params: &[PyValue]);
        fn get_rotation_distance(&self) -> (f64,f64);
        fn set_rotation_distance(&self, distance: f64);
        fn get_dir_inverted(&self) -> (bool,bool);
        fn set_dir_inverted(&self, invert: bool) -> KResult<()>;
        fn calc_position_from_coord(&self, coord: [f64;3]) -> f64;
        fn set_position(&self, coord: [f64;3]);
        fn get_commanded_position(&self) -> f64;
        fn get_mcu_position(&self, cmd: Option<f64>) -> i64;
        fn get_past_mcu_position(&self, time: f64) -> i64;
        fn mcu_to_commanded_position(&self, pos: i64) -> f64;
        fn dump_steps(&self, count: i64, start: i64, end: i64) -> (Vec<PullHistoryStep>,i64);
        fn get_stepper_kinematics(&self) -> Option<StepperKinematics>;
        fn set_stepper_kinematics(&self, kin: Option<StepperKinematics>) -> Option<StepperKinematics>;
        fn note_homing_end(&self) -> KResult<()>;
        fn get_trapq(&self) -> Option<Trapq>;
        fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq>;
        fn add_active_callback(&self, cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()>;
        fn is_active_axis(&self, axis: char) -> bool;
    }
}
struct FakeKinematics { trace: Trace, steppers: Vec<Stepper> }
impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        push(&self.trace, vec![text("steppers")]); self.steppers.clone()
    }
    unused! {
        fn calc_position(&self, pos: &IndexMap<String,f64>) -> KResult<Vec<Option<f64>>>;
        fn set_position(&self, pos: Vec<f64>, axes: &str) -> KResult<()>;
        fn check_move(&self, mv: &dyn MoveApi) -> KResult<()>;
        fn home(&self, state: HomingStateHandle) -> KResult<()>;
        fn get_status(&self, time: f64) -> KResult<PyValue>;
        fn clear_homing_state(&self, axes: &str) -> KResult<()>;
    }
}
struct FakeToolhead { trace: Trace, kin: Kinematics, position: GCell<Vec<f64>>, fail: GCell<bool> }
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);
impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        push(&self.trace, vec![text("position")]); self.position.cloned()
    }
    fn get_kinematics(&self) -> Kinematics {
        push(&self.trace, vec![text("kinematics")]); self.kin.clone()
    }
    unused! {
        fn set_position(&self, pos: Vec<f64>, axes: &str) -> KResult<()>;
        fn flush_step_generation(&self) -> KResult<()>;
        fn get_last_move_time(&self) -> KResult<f64>;
        fn dwell(&self, delay: f64) -> KResult<()>;
        fn drip_move(&self, pos: Vec<f64>, speed: f64, completion: &Completion<bool>) -> KResult<()>;
    }
}
impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, pos: Vec<f64>, speed: f64) -> KResult<()> {
        push(&self.trace, vec![text("move"), floats(&pos), Value::float(speed)]);
        if self.fail.get() { return Err(KlippyError::command("move rejected")); }
        self.position.replace(pos);
        Ok(())
    }
    unused! {
        fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()>;
        fn wait_moves(&self) -> KResult<()>;
        fn get_status(&self, time: f64) -> KResult<PyValue>;
        fn get_trapq(&self) -> Trapq;
        fn get_max_velocity(&self) -> (f64,f64);
        fn set_max_velocities(&self, v: Option<f64>, a: Option<f64>, scv: Option<f64>, ratio: Option<f64>) -> (f64,f64,f64,f64);
        fn register_lookahead_callback(&self, cb: LookaheadCallback) -> KResult<()>;
        fn check_busy(&self, time: f64) -> (f64,f64,bool);
        fn limit_next_junction_speed(&self, speed: f64);
        fn set_extruder(&self, extruder: PrinterExtruder, pos: f64);
        fn get_extruder(&self) -> PrinterExtruder;
        fn add_extra_axis(&self, axis: ExtraAxis, pos: f64) -> KResult<()>;
        fn remove_extra_axis(&self, axis: &ExtraAxis) -> KResult<()>;
        fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>>;
        fn stats(&self, time: f64) -> (bool,String);
        fn stepper_factory(&self) -> StepperFactory;
    }
}
impl MoveTransformApi for FakeToolhead {
    fn get_position(&self) -> KResult<Vec<f64>> { Ok(HomingTargetApi::get_position(self)) }
    fn r#move(&self, pos: Vec<f64>, speed: f64) -> KResult<()> { ToolHeadApi::r#move(self, pos, speed) }
}
struct FakeMove { trace: Trace, next: GCell<Option<MoveTransform>>, transform: GCell<Option<MoveTransform>> }
klippy_api::printer_object!(FakeMove; dyn GCodeMoveApi);
impl GCodeMoveApi for FakeMove {
    fn set_move_transform(&self, transform: Option<MoveTransform>, force: bool) -> KResult<Option<MoveTransform>> {
        push(&self.trace, vec![text("transform"), Value::Bool(force)]);
        self.transform.replace(transform);
        Ok(self.next.cloned())
    }
    fn reset_last_position(&self) -> KResult<()> {
        push(&self.trace, vec![text("reset")]);
        if let Some(t) = self.transform.cloned() {
            match t {
                MoveTransform::Custom(t) => { t.get_position()?; }
                MoveTransform::ToolHead(t) => { t.get_position(); }
            }
        }
        Ok(())
    }
    unused! {
        fn get_status(&self, time: Option<f64>) -> PyValue;
        fn cmd_g1(&self, cmd: &GCmd) -> KResult<()>;
    }
}
struct FakeHoming(Vec<i64>);
impl HomingStateApi for FakeHoming {
    fn get_axes(&self) -> Vec<i64> { self.0.clone() }
    unused! {
        fn set_axes(&self, axes: Vec<i64>);
        fn get_trigger_position(&self, name: &str) -> KResult<i64>;
        fn set_stepper_adjustment(&self, name: &str, adjustment: f64);
        fn set_homed_position(&self, pos: Vec<Option<f64>>) -> KResult<()>;
        fn home_rails(&self, rails: Vec<Rail>, force: Vec<Option<f64>>, pos: Vec<Option<f64>>) -> KResult<()>;
    }
}

struct FakePrinter { trace: Trace, objects: GCell<IndexMap<String,ObjectRef>>, events: GCell<IndexMap<String, Vec<EventCallback>>> }
impl PrinterApi for FakePrinter {
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        push(&self.trace, vec![text("lookup"), text(name)]);
        match self.objects.borrow().get(name).cloned() {
            Some(v) => Ok(Some(v)),
            None if default == LookupDefault::ReturnNone => Ok(None),
            None => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
        }
    }
    fn load_object(&self, _c: &Config, name: &str, _d: LookupDefault) -> KResult<Option<ObjectRef>> {
        push(&self.trace, vec![text("load"), text(name)]);
        Ok(self.objects.borrow().get(name).cloned())
    }
    fn register_event_handler(&self, name: &str, callback: EventCallback) {
        push(&self.trace, vec![text("event"), text(name)]);
        self.events.borrow_mut().entry(name.into()).or_default().push(callback);
    }
    fn send_event(&self, event: &str, args: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut handlers = self.events.borrow_mut().shift_remove(event).unwrap();
        let result = handlers.iter_mut().map(|h| h(args)).collect();
        self.events.borrow_mut().insert(event.into(), handlers);
        result
    }
    unused! {
        fn get_start_args(&self) -> PyValue;
        fn get_reactor(&self) -> Reactor;
        fn get_state_message(&self) -> (String,String);
        fn is_shutdown(&self) -> bool;
        fn update_error_msg(&self, old: &str, new: &str);
        fn add_object(&self, name: &str, object: ObjectRef) -> KResult<()>;
        fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)>;
        fn run(&self) -> Option<String>;
        fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool);
        fn invoke_shutdown(&self, message: &str, details: PyValue);
        fn invoke_async_shutdown(&self, message: &str, details: PyValue);
        fn request_exit(&self, result: &str);
    }
}
struct FakeConfig { printer: Printer, options: IndexMap<String,String>, trace: Trace }
impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { "z_thermal_adjust".into() }
    fn getfloat(&self, name: &str, o: GetFloatOpts) -> KResult<Option<f64>> {
        let num = |arg: IntArgs, v: Option<f64>| v.map_or(Value::Null, |v| value(o.ints.value(arg, v)));
        push(&self.trace, vec![text("config"), text(name),
            o.default.map(|d| num(IntArgs::DEFAULT, d)).unwrap_or(text("required")),
            num(IntArgs::MINVAL, o.minval), num(IntArgs::MAXVAL, o.maxval), num(IntArgs::ABOVE, o.above),
            num(IntArgs::BELOW, o.below), Value::Bool(o.note_valid)]);
        let Some(raw) = self.options.get(name) else {
            return o.default.ok_or_else(|| KlippyError::config(format!("Option '{name}' in section 'z_thermal_adjust' must be specified")));
        };
        let n = float_from_string(raw).map_err(|_| KlippyError::config(format!("Unable to parse option '{name}' in section 'z_thermal_adjust'")))?;
        for (bound, arg, failed, kind) in [
            (o.minval, IntArgs::MINVAL, o.minval.is_some_and(|b| n < b), "have minimum of"),
            (o.maxval, IntArgs::MAXVAL, o.maxval.is_some_and(|b| n > b), "have maximum of"),
            (o.above, IntArgs::ABOVE, o.above.is_some_and(|b| n <= b), "be above"),
            (o.below, IntArgs::BELOW, o.below.is_some_and(|b| n >= b), "be below"),
        ] {
            if failed {
                let b = o.ints.value(arg, bound.unwrap()).str();
                return Err(KlippyError::config(format!("Option '{name}' in section 'z_thermal_adjust' must {kind} {b}")));
            }
        }
        Ok(Some(n))
    }
    unused! {
        fn get(&self, name: &str, opts: GetStrOpts) -> KResult<Option<String>>;
        fn getint(&self, name: &str, opts: GetIntOpts) -> KResult<Option<i64>>;
        fn getboolean(&self, name: &str, opts: GetBoolOpts) -> KResult<Option<bool>>;
        fn getchoice(&self, name: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue>;
        fn getlists(&self, name: &str, opts: GetListsOpts) -> KResult<PyValue>;
        fn getlist(&self, name: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>>;
        fn getintlist(&self, name: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>>;
        fn getfloatlist(&self, name: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>>;
        fn getsection(&self, name: &str) -> Config;
        fn has_section(&self, name: &str) -> bool;
        fn get_prefix_sections(&self, prefix: &str) -> Vec<Config>;
        fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>>;
        fn deprecate(&self, name: &str, val: Option<PyValue>) -> KResult<()>;
    }
}

struct Harness {
    trace: Trace, printer: Arc<FakePrinter>, config: Config, sensor: Arc<FakeSensor>,
    toolhead: Arc<FakeToolhead>, gcode: Arc<FakeGcode>, gmove: Arc<FakeMove>,
}
impl Harness {
    fn new(options: &[(&str,&str)], names: &[&str], custom: bool) -> Self {
        let trace = Arc::new(GCell::new(vec![]));
        let kin = Arc::new(FakeKinematics { trace: trace.clone(), steppers: names.iter().map(|n| {
            Arc::new(FakeStepper { trace: trace.clone(), name: (*n).into() }) as Stepper
        }).collect() });
        let toolhead = Arc::new(FakeToolhead { trace: trace.clone(), kin, position: GCell::new(vec![0.;4]), fail: GCell::new(false) });
        let next = if custom { MoveTransform::Custom(toolhead.clone()) } else { MoveTransform::ToolHead(toolhead.clone()) };
        let gmove = Arc::new(FakeMove { trace: trace.clone(), next: GCell::new(Some(next)), transform: GCell::new(None) });
        let gcode = Arc::new(FakeGcode { trace: trace.clone(), commands: GCell::new(IndexMap::new()) });
        let sensor = Arc::new(FakeSensor { trace: trace.clone(), callback: GCell::new(None) });
        let heaters = Arc::new(FakeHeaters { trace: trace.clone(), sensor: sensor.clone(), registered: GCell::new(None) });
        let objects: IndexMap<String,ObjectRef> = [
            ("gcode".into(), gcode.clone() as ObjectRef),
            ("toolhead".into(), toolhead.clone() as ObjectRef),
            ("gcode_move".into(), gmove.clone() as ObjectRef),
            ("heaters".into(), heaters as ObjectRef),
        ].into_iter().collect();
        let printer = Arc::new(FakePrinter { trace: trace.clone(), objects: GCell::new(objects), events: GCell::new(IndexMap::new()) });
        let config = Arc::new(FakeConfig { printer: printer.clone(), options: options.iter().map(|(k,v)| ((*k).into(),(*v).into())).collect(), trace: trace.clone() });
        Self { trace, printer, config, sensor, toolhead, gcode, gmove }
    }
    fn load(&self) -> KResult<Arc<ZThermalAdjuster>> {
        let obj = module_factory().load_config.unwrap()(&self.config)?.unwrap();
        Ok(downcast_arc(&obj).unwrap())
    }
    fn trace(&self, py: &mut pyoracle::Oracle) {
        compare(py, "trace", Value::List(self.trace.cloned()));
        self.trace.borrow_mut().clear();
        py.exec("trace.clear()").unwrap();
    }
    fn connect(&self) -> KResult<Value> {
        self.printer.send_event("klippy:connect", &[]).map(|_| Value::Null)
    }
    fn sample(&self, time: f64, temp: f64) {
        (self.sensor.callback.borrow_mut().as_mut().unwrap())(time,temp);
    }
    fn command(&self, params: &[(&str,&str)]) -> KResult<Value> {
        let gcmd: GCmd = Arc::new(FakeCommand { trace: self.trace.clone(), params: params.iter().map(|(k,v)| ((*k).into(),(*v).into())).collect() });
        self.gcode.commands.borrow_mut().get_mut("SET_Z_THERMAL_ADJUST").unwrap()(&gcmd).map(|_| Value::Null)
    }
    fn homing(&self, axes: Vec<i64>) {
        let h: Arc<dyn HomingStateApi> = Arc::new(FakeHoming(axes));
        self.printer.send_event("homing:home_rails_end", &[EventArg::handle(h), EventArg::List(vec![])]).unwrap();
    }
}
fn snapshot(py: &mut pyoracle::Oracle, obj: &ZThermalAdjuster) {
    let (current,target) = obj.get_temp(123.);
    let (active,stats) = obj.stats(123.);
    compare(py, "snapshot()", tuple(vec![value(obj.get_status(123.)), tuple(vec![Value::float(current), Value::float(target)]), tuple(vec![Value::Bool(active), text(&stats)])]));
}
fn mapping(options: &[(&str,&str)]) -> String {
    format!("{{{}}}", options.iter().map(|(k,v)| format!("{k:?}: {v:?}")).collect::<Vec<_>>().join(","))
}
fn build(py: &mut pyoracle::Oracle, options: &[(&str,&str)], custom: bool) -> (Harness, Arc<ZThermalAdjuster>) {
    let h = Harness::new(options, &["stepper_x","stepper_z","stepper_z1"], custom);
    let obj = h.load().unwrap();
    compare(py, &format!("capture(lambda: build({}))", mapping(options)), outcome(Ok(Value::Null)));
    h.trace(py);
    snapshot(py, &obj);
    (h,obj)
}

#[test]
fn configuration_and_initialization_parity() {
    let Some(mut py) = oracle() else { return; };
    let cases = [
        vec![], vec![("min_temp","0")], vec![("min_temp","-273.16"),("max_temp","100")],
        vec![("min_temp","0"),("max_temp","0")], vec![("min_temp","0"),("max_temp","-1")],
        vec![("min_temp","bad"),("max_temp","100")], vec![("min_temp","0"),("max_temp","bad")],
        vec![("temp_coeff","-1.1")], vec![("temp_coeff","1.1")], vec![("temp_coeff","bad")],
        vec![("smooth_time","0")], vec![("smooth_time","-0.1")], vec![("smooth_time","bad")],
        vec![("z_adjust_off_above","bad")], vec![("max_z_adjustment","bad")],
        vec![("min_temp","-273.15"),("max_temp","500")],
        vec![("temp_coeff","-1"),("smooth_time","0.01"),("z_adjust_off_above","-10"),("max_z_adjustment","-0.2"),("min_temp","0"),("max_temp","1")],
        vec![("temp_coeff","1"),("min_temp","0"),("max_temp","100")],
    ];
    for options in cases {
        let h = Harness::new(&options, &["stepper_z"], false);
        let result = h.load().map(|_| Value::Null);
        compare(&mut py, &format!("capture(lambda: build({}, ['stepper_z']))", mapping(&options)), outcome(result));
        h.trace(&mut py);
    }
}

#[test]
fn temperature_homing_commands_and_transform_parity() {
    let Some(mut py) = oracle() else { return; };
    for custom in [false,true] {
        let (h,obj) = build(&mut py, &[("min_temp","-100"),("max_temp","200"),("temp_coeff","0.02"),("z_adjust_off_above","5"),("max_z_adjustment","0.25")], custom);
        compare(&mut py, "capture(lambda: p.send('klippy:connect'))", outcome(h.connect()));
        h.trace(&mut py);
        for (time,temp) in [(0.5,20.), (1.,30.), (3.,25.555), (3.,100.), (2.,-20.), (4.,-15.125)] {
            h.sample(time,temp);
            py.exec(&format!("p.objects['heaters'].sensor.callback({time:?}, {temp:?})")).unwrap();
            snapshot(&mut py,&obj);
        }
        for axes in [vec![0,1],vec![2]] {
            h.homing(axes.clone());
            py.exec(&format!("p.send('homing:home_rails_end', Homing({axes:?}), [])")).unwrap();
            snapshot(&mut py,&obj);
        }
        let steps: Vec<(&str,Vec<(&str,&str)>)> = vec![
            ("query",vec![]), ("manual",vec![("REF_TEMP","20"),("TEMP_COEFF","0.01")]),
            ("disable",vec![("ENABLE","0")]), ("disable again",vec![("ENABLE","0")]),
            ("enable",vec![("ENABLE","1")]),
            ("lower bounds",vec![("REF_TEMP","-273.15"),("TEMP_COEFF","-1")]),
            ("upper coefficient",vec![("TEMP_COEFF","1")]),
        ];
        for (_,params) in steps {
            compare(&mut py,&format!("capture(lambda: command({}))",mapping(&params)),outcome(h.command(&params)));
            h.trace(&mut py);
            snapshot(&mut py,&obj);
        }
        h.homing(vec![2]);
        py.exec("p.send('homing:home_rails_end', Homing([2]), [])").unwrap();
        compare(&mut py,"capture(lambda: command({'TEMP_COEFF':'0.02'}))",outcome(h.command(&[("TEMP_COEFF","0.02")])));
        h.sample(6.,15.);
        py.exec("p.objects['heaters'].sensor.callback(6.,15.)").unwrap();
        let moves = [
            vec![0.,0.,1.,3.,99.], // XY unchanged truncates extra axes.
            vec![1.,0.,1.,4.,98.], // XY move applies and clamps adjustment.
            vec![1.,0.,2.,5.,97.], // Z-only preserves previous adjustment.
            vec![2.,0.,5.,6.,96.], // Exact off_above threshold freezes adjustment.
            vec![3.,0.,6.,7.,95.],
            vec![4.,0.,4.,8.,94.],
        ];
        for pos in moves {
            compare(&mut py,&format!("capture(lambda: obj.move({pos:?}, 20.))"),outcome(obj.r#move(pos,20.).map(|_| Value::Null)));
            h.trace(&mut py);
            snapshot(&mut py,&obj);
            compare(&mut py,"capture(lambda: obj.get_position())",outcome(obj.get_position().map(|v| floats(&v))));
            h.trace(&mut py);
            snapshot(&mut py,&obj);
        }
        compare(&mut py,"capture(lambda: command({'ENABLE':'0'}))",outcome(h.command(&[("ENABLE","0")])));
        h.trace(&mut py);
        h.sample(8.,-30.);
        py.exec("p.objects['heaters'].sensor.callback(8.,-30.)").unwrap();
        compare(&mut py,"capture(lambda: obj.move([10.,1.,1.,0.,88.], 2.))",outcome(obj.r#move(vec![10.,1.,1.,0.,88.],2.).map(|_| Value::Null)));
        h.trace(&mut py);
        snapshot(&mut py,&obj);
        // get_position calculates adjustments even while disabled.
        compare(&mut py,"capture(lambda: obj.get_position())",outcome(obj.get_position().map(|v| floats(&v))));
        h.trace(&mut py);
        snapshot(&mut py,&obj);
        h.toolhead.fail.set(true);
        py.exec("p.objects['toolhead'].fail = True").unwrap();
        compare(&mut py,"capture(lambda: obj.move([11.,1.,1.,0.], 2.))",outcome(obj.r#move(vec![11.,1.,1.,0.],2.).map(|_| Value::Null)));
        h.trace(&mut py);
        snapshot(&mut py,&obj);
    }
}

#[test]
fn command_errors_and_partial_state_parity() {
    let Some(mut py) = oracle() else { return; };
    let (h,obj) = build(&mut py,&[("min_temp","0"),("max_temp","100")],false);
    compare(&mut py,"capture(lambda: p.send('klippy:connect'))",outcome(h.connect()));
    h.trace(&mut py);
    let cases = [
        vec![("ENABLE","-1")],vec![("ENABLE","2")],vec![("ENABLE","0.0")],vec![("ENABLE","bad")],
        vec![("TEMP_COEFF","-1.01")],vec![("TEMP_COEFF","1.01")],vec![("TEMP_COEFF","bad")],
        vec![("REF_TEMP","-273.16")],vec![("REF_TEMP","bad")],
        vec![("ENABLE","0"),("TEMP_COEFF","0.5"),("REF_TEMP","bad")],
        vec![("ENABLE","1"),("TEMP_COEFF","0.5"),("REF_TEMP","22")],
    ];
    for params in cases {
        compare(&mut py,&format!("capture(lambda: command({}))",mapping(&params)),outcome(h.command(&params)));
        h.trace(&mut py);
        snapshot(&mut py,&obj);
    }
    h.printer.objects.borrow_mut().shift_remove("gcode_move");
    py.exec("del p.objects['gcode_move']").unwrap();
    compare(&mut py,"capture(lambda: command({'ENABLE':'0','REF_TEMP':'30'}))",outcome(h.command(&[("ENABLE","0"),("REF_TEMP","30")])));
    h.trace(&mut py);
    snapshot(&mut py,&obj);
}

#[test]
fn missing_transform_stepper_and_short_positions_parity() {
    let Some(mut py) = oracle() else { return; };
    let (h,obj) = build(&mut py,&[("min_temp","0"),("max_temp","100")],false);
    compare(&mut py,"capture(lambda: obj.get_position())",outcome(obj.get_position().map(|p| floats(&p))));
    compare(&mut py,"capture(lambda: obj.move([0.,0.,0.,0.], 1.))",outcome(obj.r#move(vec![0.;4],1.).map(|_| Value::Null)));
    compare(&mut py,"capture(lambda: obj.calc_adjust([1.,0.,0.,0.]))",outcome(obj.calc_adjust(vec![1.,0.,0.,0.]).map(|p| floats(&p))));
    h.trace(&mut py);
    compare(&mut py,"capture(lambda: p.send('klippy:connect'))",outcome(h.connect()));
    h.trace(&mut py);
    for pos in [vec![],vec![1.],vec![1.,2.],vec![1.,2.,3.]] {
        compare(&mut py,&format!("capture(lambda: obj.calc_adjust({pos:?}))"),outcome(obj.calc_adjust(pos.clone()).map(|p| floats(&p))));
        compare(&mut py,&format!("capture(lambda: obj.calc_unadjust({pos:?}))"),outcome(obj.calc_unadjust(pos.clone()).map(|p| floats(&p))));
        compare(&mut py,&format!("capture(lambda: obj.move({pos:?}, 1.))"),outcome(obj.r#move(pos,1.).map(|_| Value::Null)));
    }
    h.trace(&mut py);
    let options = [("min_temp","0"),("max_temp","100")];
    let h = Harness::new(&options,&["stepper_x","stepper_z1"],false);
    let _obj = h.load().unwrap();
    compare(&mut py,&format!("capture(lambda: build({}, ['stepper_x','stepper_z1']))",mapping(&options)),outcome(Ok(Value::Null)));
    h.trace(&mut py);
    compare(&mut py,"capture(lambda: p.send('klippy:connect'))",outcome(h.connect()));
    h.trace(&mut py);
    for missing in ["toolhead", "gcode_move"] {
        let (h, _obj) = build(&mut py, &options, false);
        h.printer.objects.borrow_mut().shift_remove(missing);
        py.exec(&format!("del p.objects[{missing:?}]")).unwrap();
        compare(&mut py, "capture(lambda: p.send('klippy:connect'))", outcome(h.connect()));
        h.trace(&mut py);
    }
    let (h, obj) = build(&mut py, &options, false);
    h.gmove.next.replace(None);
    py.exec("p.objects['gcode_move'].next = None").unwrap();
    compare(&mut py, "capture(lambda: p.send('klippy:connect'))", outcome(h.connect()));
    h.trace(&mut py);
    compare(&mut py, "capture(lambda: obj.get_position())", outcome(obj.get_position().map(|p| floats(&p))));
    compare(&mut py, "capture(lambda: obj.move([1.,0.,1.,0.],1.))", outcome(obj.r#move(vec![1.,0.,1.,0.],1.).map(|_| Value::Null)));
}

#[test]
fn step_threshold_sign_limits_and_nonfinite_samples_parity() {
    let Some(mut py) = oracle() else { return; };
    for limit in ["0.25","-0.25","0"] {
        let (h,obj) = build(&mut py,&[("min_temp","-273.15"),("max_temp","500"),("temp_coeff","0.0125"),("max_z_adjustment",limit)],false);
        compare(&mut py,"capture(lambda: p.send('klippy:connect'))",outcome(h.connect()));
        h.trace(&mut py);
        for (time,temp) in [(2.,1.),(4.,1.0000001),(6.,-1.),(8.,-40.),(10.,0.)] {
            h.sample(time,temp);
            py.exec(&format!("obj.temperature_callback({time:?},{temp:?})")).unwrap();
            for pos in [vec![1.,2.,3.,4.,5.],vec![1.,2.,99999999.,4.,5.]] {
                compare(&mut py,&format!("capture(lambda: obj.calc_adjust({pos:?}))"),outcome(obj.calc_adjust(pos.clone()).map(|p| floats(&p))));
                compare(&mut py,&format!("capture(lambda: obj.calc_unadjust({pos:?}))"),outcome(obj.calc_unadjust(pos).map(|p| floats(&p))));
                snapshot(&mut py,&obj);
            }
        }
        for (time,temp,expr) in [(12.,f64::INFINITY,"float('inf')"),(14.,f64::NAN,"float('nan')")] {
            h.sample(time,temp);
            py.exec(&format!("obj.temperature_callback({time:?},{expr})")).unwrap();
            snapshot(&mut py,&obj);
            compare(&mut py,"capture(lambda: obj.calc_adjust([2.,3.,0.,1.]))",outcome(obj.calc_adjust(vec![2.,3.,0.,1.]).map(|p| floats(&p))));
        }
    }
}

#[test]
fn homing_retains_previous_move_offset_and_failed_moves_keep_state_parity() {
    let Some(mut py) = oracle() else { return; };
    let (h,obj) = build(&mut py, &[("min_temp","0"),("max_temp","100"),("temp_coeff","0.01")], true);
    compare(&mut py,"capture(lambda: p.send('klippy:connect'))",outcome(h.connect()));
    h.trace(&mut py);
    h.sample(2.,20.);
    py.exec("obj.temperature_callback(2.,20.)").unwrap();
    compare(&mut py,"capture(lambda: obj.move([1.,0.,1.,0.],2.))",outcome(obj.r#move(vec![1.,0.,1.,0.],2.).map(|_| Value::Null)));
    h.trace(&mut py);
    h.homing(vec![2]);
    py.exec("p.send('homing:home_rails_end',Homing([2]),[])").unwrap();
    snapshot(&mut py,&obj);
    compare(&mut py,"capture(lambda: obj.move([1.,0.,1.,3.],2.))",outcome(obj.r#move(vec![1.,0.,1.,3.],2.).map(|_| Value::Null)));
    h.trace(&mut py);
    h.sample(4.,40.);
    h.toolhead.fail.set(true);
    py.exec("obj.temperature_callback(4.,40.); p.objects['toolhead'].fail=True").unwrap();
    compare(&mut py,"capture(lambda: obj.move([2.,0.,1.,3.],2.))",outcome(obj.r#move(vec![2.,0.,1.,3.],2.).map(|_| Value::Null)));
    h.trace(&mut py);
    snapshot(&mut py,&obj);
    h.sample(6.,50.);
    h.toolhead.fail.set(false);
    py.exec("obj.temperature_callback(6.,50.); p.objects['toolhead'].fail=False").unwrap();
    compare(&mut py,"capture(lambda: obj.move([2.,0.,1.,4.],2.))",outcome(obj.r#move(vec![2.,0.,1.,4.],2.).map(|_| Value::Null)));
    h.trace(&mut py);
    snapshot(&mut py,&obj);
}
