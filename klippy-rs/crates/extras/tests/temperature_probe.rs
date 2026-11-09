//! Parity tests for `klippy/extras/temperature_probe.py`.
//!
//! Every expected value comes from `pyoracle` (live CPython 3.8.2 + frozen
//! `klippy/`). When `$KLIPPY_PYTHON` is unset the oracle is unavailable and
//! each test skips with a message.
//!
//! The pyoracle runs the real `klippy/extras/temperature_probe.py` (and the
//! real `extras.manual_probe` it drives) with minimal fakes for the printer
//! objects it looks up, then the test drives the Rust port through
//! contract-trait fakes with the same inputs and compares via
//! `recording::assert_equal`. Both sides read their options through the real
//! config wrapper (`configfile.ConfigWrapper` / `klippy_core::configfile`)
//! over the same config file text, and every `gcmd` comes from the real
//! dispatcher (`gcode.GCodeDispatch` / `klippy_core::gcode::GCodeDispatch`),
//! so option and parameter errors carry the real messages. Covered:
//!
//! - all config options owned by the units (defaults, bounds, required),
//! - all G-code commands owned (CALIBRATE, NEXT, COMPLETE, ABORT, ENABLE)
//!   plus errors (not homed, unlinked probe, already in calibration, too
//!   few samples, duplicate registration, missing probe/helper, already in
//!   manual probe),
//! - `get_status` dicts, `get_temp`/`stats`,
//! - G-code side effects (respond_info strings, run_script strings,
//!   registered/unregistered commands),
//! - `_temp_callback` smoothing + min/max tracking + async kick,
//! - `Polynomial2d` call/str/repr/fit,
//! - `EddyDriftCompensation` init (drift rows, min_freq min over 0..120,
//!   enable rules, cal_temp gating), set_enabled, adjust/unadjust freq,
//!   check_calibration, collect_sample (bulk-window + averages), and
//!   finish_calibration (polynomial logs, configfile sets).
//!
//! The async `_check_kick_next` hook runs inside `reactor.run()` for the port
//! and through the Python fake reactor's `run_async` for the reference.

#![allow(clippy::disallowed_types)]

use extras::bulk_sensor::ClientCallback;
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use extras::manual_probe::{ManualProbeApi, ProbeResult};
use extras::probe::{PrinterProbeApi, ProbeParams, ProbeSessionApi};
use extras::probe_eddy_current::{DriftCompensationApi, PrinterEddyProbeApi};
use extras::temperature_probe::{EddyDriftCompensation, Polynomial2d, TemperatureProbe};
use indexmap::IndexMap;
use klippy_api::config::{Config, DeprecateGcodeOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, GCmd, RegisterCommandOpts};
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::gcell::GCell;
use klippy_api::stepper::{Stepper, Trapq};
use klippy_api::toolhead::{
    DeltaCalibration, ExtraAxis, ExtraAxisApi, Kinematics, KinematicsApi, MoveApi, PrinterExtruder, PrinterExtruderApi, ToolHeadApi,
};
use klippy_core::configfile::{ConfigFileReader, ConfigWrapper};
use klippy_core::gcode::GCodeDispatch;
use pycompat::value::PyValue;
use recording::{Exception, Record, Value};
use reactor::{Reactor, NEVER, NOW};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, Once};

// ---------------------------------------------------------------------------
// Recording helpers
// ---------------------------------------------------------------------------

fn dummy_record(function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.temperature_probe".to_string(),
        scenario: "oracle".to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: Default::default(),
        object_id: None,
        kind: recording::RecordKind::Python,
    }
}

fn check_oracle(fn_name: &str, expected: &Value, actual: &Value) {
    let rec = dummy_record(fn_name);
    if let Err(d) = recording::assert_equal(&rec, "ret", expected, actual) {
        panic!("parity divergence in {fn_name} at {path}: expected {expected}, actual {actual}",
            path = d.path, expected = d.expected, actual = d.actual);
    }
}

fn pyvalue_to_recording(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_recording).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_recording).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_recording(k), pyvalue_to_recording(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host#{h}")),
    }
}

fn check_exc(fn_name: &str, expected: &Exception, actual: &KlippyError) {
    assert_eq!(
        expected.class_name,
        actual.class_name(),
        "{fn_name}: exception class",
    );
    assert_eq!(expected.message, actual.message(), "{fn_name}: exception message");
}

// ---------------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            reactor: Reactor::new(false),
        })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects
            .lock()
            .unwrap()
            .insert(name.to_string(), obj);
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
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock().unwrap();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
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
        let objects = self.objects.lock().unwrap();
        match objects.get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .unwrap()
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
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[klippy_api::object::EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}


/// The module's `gcode` object. Handlers, mux registrations, responses and
/// scripts are recorded; `gcmd` objects come from the real
/// `klippy_core::gcode::GCodeDispatch` (the reference uses the real
/// `gcode.GCodeDispatch`), whose output handler records what a `gcmd`
/// responds.
struct FakeGCode {
    handlers: Mutex<IndexMap<String, Option<GCodeCommandHandler>>>,
    mux: Mutex<Vec<(String, String, Option<String>)>>,
    infos: Mutex<Vec<String>>,
    scripts: Mutex<Vec<String>>,
    scripts_from: Mutex<Vec<String>>,
    pre_registered: Mutex<Vec<String>>,
    reactor: Reactor,
    dispatch: GCodeDispatch,
    gcmd_out: Arc<Mutex<Vec<String>>>,
}

impl FakeGCode {
    fn new(printer: Printer) -> Arc<FakeGCode> {
        let reactor = printer.get_reactor();
        let dispatch = GCodeDispatch::new(printer);
        let gcmd_out = Arc::new(Mutex::new(Vec::new()));
        let out = Arc::clone(&gcmd_out);
        dispatch.register_output_handler(Box::new(move |msg| out.lock().unwrap().push(msg)));
        Arc::new(FakeGCode {
            handlers: Mutex::new(IndexMap::new()),
            mux: Mutex::new(Vec::new()),
            infos: Mutex::new(Vec::new()),
            scripts: Mutex::new(Vec::new()),
            scripts_from: Mutex::new(Vec::new()),
            pre_registered: Mutex::new(Vec::new()),
            reactor,
            dispatch,
            gcmd_out,
        })
    }
    fn pre_register(&self, cmd: &str) {
        self.pre_registered
            .lock()
            .unwrap()
            .push(cmd.to_string());
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut h = self.handlers.lock().unwrap();
        if func.is_none() {
            // Python: `return self.handlers.pop(cmd, None)`.
            return Ok(h.shift_remove(cmd).flatten());
        }
        if h.contains_key(cmd) {
            // Python raises config_error on duplicate registration.
            return Err(KlippyError::Config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        if self.pre_registered.lock().unwrap().contains(&cmd.to_string()) {
            return Err(KlippyError::Config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        Ok(h.insert(cmd.to_string(), func).flatten())
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        self.mux
            .lock()
            .unwrap()
            .push((cmd.to_string(), key.to_string(), value.map(|s| s.to_string())));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.lock().unwrap().push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.scripts_from.lock().unwrap().push(script.to_string());
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
        self.dispatch.create_gcode_command(command, commandline, params)
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, _log: bool) {
        self.infos.lock().unwrap().push(msg.to_string());
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


struct FakeToolHead {
    position: Mutex<Vec<f64>>,
    homed_axes: Mutex<String>,
    moves: Mutex<Vec<(Vec<Option<f64>>, f64)>>,
    last_move_time: Mutex<f64>,
    dwell_count: Mutex<usize>,
    wait_count: Mutex<usize>,
    bulk: Mutex<Option<Arc<Mutex<Option<ClientCallback>>>>>,
    bulk_schedule: Mutex<VecDeque<Vec<(f64, f64, f64)>>>,
    fail_move_at: Mutex<Option<usize>>,
}

impl FakeToolHead {
    fn new(pos: Vec<f64>, homed: &str) -> Arc<FakeToolHead> {
        Arc::new(FakeToolHead {
            position: Mutex::new(pos),
            homed_axes: Mutex::new(homed.to_string()),
            moves: Mutex::new(Vec::new()),
            last_move_time: Mutex::new(10.0),
            dwell_count: Mutex::new(0),
            wait_count: Mutex::new(0),
            bulk: Mutex::new(None),
            bulk_schedule: Mutex::new(VecDeque::new()),
            fail_move_at: Mutex::new(None),
        })
    }
}

struct FakeTrapq;
impl klippy_api::stepper::TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any { self }
}
struct FakeExtruder;
impl ExtraAxisApi for FakeExtruder {
    fn get_name(&self) -> String { "extruder".to_string() }
    fn get_axis_gcode_id(&self) -> Option<String> { None }
    fn get_trapq(&self) -> Option<klippy_api::stepper::Trapq> { None }
    fn check_move(&self, _mv: &dyn MoveApi, _i: usize) -> KResult<()> { Ok(()) }
    fn calc_junction(&self, _p: &dyn MoveApi, _m: &dyn MoveApi, _i: usize) -> f64 { 0.0 }
    fn process_move(&self, _t: f64, _m: &dyn MoveApi, _i: usize) {}
}
impl PrinterExtruderApi for FakeExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> { panic!("unused") }
    fn find_past_position(&self, _t: f64) -> KResult<f64> { Ok(0.0) }
    fn stats(&self, _e: f64) -> Option<(bool, String)> { None }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> { None }
}
impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        self.position.lock().unwrap().clone()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        *self.position.lock().unwrap() = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics {
            pos: self.position.lock().unwrap().clone(),
        })
    }
    fn flush_step_generation(&self) -> KResult<()> { Ok(()) }
    fn get_last_move_time(&self) -> f64 {
        let mut t = self.last_move_time.lock().unwrap();
        *t += 1.0;
        *t
    }
    fn dwell(&self, _delay: f64) {
        *self.dwell_count.lock().unwrap() += 1;
        let bulk_opt = self.bulk.lock().unwrap().clone();
        if let Some(bulk) = bulk_opt {
            let batch: Vec<(f64, f64, f64)> =
                self.bulk_schedule.lock().unwrap().pop_front().unwrap_or_default();
            if !batch.is_empty() {
                let mut d = IndexMap::new();
                d.insert(
                    PyValue::Str("data".to_string()),
                    PyValue::List(
                        batch
                            .iter()
                            .map(|(a, b, c)| {
                                PyValue::List(vec![
                                    PyValue::Float(*a),
                                    PyValue::Float(*b),
                                    PyValue::Float(*c),
                                ])
                            })
                            .collect(),
                    ),
                );
                let msg = PyValue::Dict(d);
                if let Some(cb) = bulk.lock().unwrap().as_mut() {
                    let _ = cb(msg);
                }
            }
        }
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}
impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.moves
            .lock()
            .unwrap()
            .push((newpos.iter().map(|&x| Some(x)).collect(), speed));
        *self.position.lock().unwrap() = newpos;
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        if let Some(n) = *self.fail_move_at.lock().unwrap() {
            if self.moves.lock().unwrap().len() == n {
                return Err(KlippyError::Command("Move out of range".to_string()));
            }
        }
        self.moves.lock().unwrap().push((coord.clone(), speed));
        let mut pos = self.position.lock().unwrap().clone();
        for (i, c) in coord.iter().enumerate() {
            if let Some(v) = c {
                if i < pos.len() {
                    pos[i] = *v;
                }
            }
        }
        *self.position.lock().unwrap() = pos;
        Ok(())
    }
    fn wait_moves(&self) {
        *self.wait_count.lock().unwrap() += 1;
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(self.homed_axes.lock().unwrap().clone()),
        );
        PyValue::Dict(d)
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (100.0, 100.0)
    }
    fn set_max_velocities(
        &self,
        _v: Option<f64>,
        _a: Option<f64>,
        _s: Option<f64>,
        _c: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _cb: klippy_api::toolhead::LookaheadCallback) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        Arc::new(FakeExtruder)
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _p: f64) {}
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory unused")
    }
}

struct FakeKinematics {
    pos: Vec<f64>,
}

impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        vec![Arc::new(FakeStepper {
            name: "stepper_z".to_string(),
            pos: self.pos[2],
        })]
    }
    fn calc_position(&self, _sp: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(self.pos.iter().map(|&x| Some(x)).collect())
    }
    fn set_position(&self, _p: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _h: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _h: &str) {}
    fn home_axis(
        &self,
        _h: klippy_api::toolhead::HomingStateHandle,
        _axis: i64,
        _rail: klippy_api::stepper::Rail,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_calibration(&self) -> KResult<DeltaCalibration> {
        Err(klippy_api::toolhead::no_kinematics_attr("get_calibration"))
    }
}

struct FakeStepper {
    name: String,
    pos: f64,
}

impl klippy_api::stepper::McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("unused")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        0
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {}
    fn setup_itersolve(&self, _f: &str, _p: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 {
        0.025
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (8.0, 200.0)
    }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        self.pos
    }
    fn set_position(&self, _c: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        self.pos
    }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _t: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, _m: i64) -> f64 {
        self.pos
    }
    fn dump_steps(
        &self,
        _c: i64,
        _s: i64,
        _e: i64,
    ) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<klippy_api::stepper::StepperKinematics>,
    ) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<klippy_api::stepper::Trapq> {
        None
    }
    fn set_trapq(&self, _t: Option<klippy_api::stepper::Trapq>) -> Option<klippy_api::stepper::Trapq> {
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _axis: char) -> bool {
        true
    }
}

struct FakePrinterConfig {
    sets: Mutex<Vec<(String, String, String)>>,
}

impl PrinterConfigApi for FakePrinterConfig {
    fn get_printer(&self) -> Printer {
        panic!("unused")
    }
    fn read_config(&self, _f: &str) -> KResult<Config> {
        panic!("unused")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("unused")
    }
    fn log_config(&self, _c: &Config) {}
    fn check_unused_options(&self, _c: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _m: &str) {}
    fn deprecate(
        &self,
        _s: &str,
        _o: &str,
        _v: Option<PyValue>,
        _m: Option<String>,
    ) {
    }
    fn deprecate_gcode(&self, _cmd: &str, _o: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(
        &self,
        _m: &klippy_api::mcu::Mcu,
        _f: &str,
        _m2: Option<String>,
    ) {
    }
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.sets
            .lock()
            .unwrap()
            .push((section.to_string(), option.to_string(), value.to_string()));
    }
    fn remove_section(&self, _s: &str) {}
}

struct FakeSensor {
    minmax: Mutex<Option<(f64, f64)>>,
    callback: Mutex<Option<TemperatureCallback>>,
    report_delta: f64,
}

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        *self.minmax.lock().unwrap() = Some((min_temp, max_temp));
        Ok(())
    }
    fn setup_callback(&self, cb: TemperatureCallback) {
        *self.callback.lock().unwrap() = Some(cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.report_delta
    }
}

struct FakeHeaters {
    sensor: Arc<FakeSensor>,
    registered: Mutex<Vec<String>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {
        panic!("add_sensor_factory unused")
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater unused")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("lookup_heater unused")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Ok(Arc::clone(&self.sensor) as Arc<dyn TemperatureSensorApi>)
    }
    fn register_sensor(
        &self,
        config: &Config,
        _psensor: Arc<dyn extras::heaters::SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        self.registered.lock().unwrap().push(config.get_name());
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature unused")
    }
}

struct FakeProbe {
    offsets: (f64, f64, f64),
    params: (f64, f64),
    name: Mutex<String>,
    results: Arc<Mutex<Vec<ProbeResult>>>,
    run_probe: Arc<Mutex<bool>>,
}

impl PrinterProbeApi for FakeProbe {
    fn get_probe_params(&self, _gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        Ok(ProbeParams {
            probe_speed: self.params.0,
            lift_speed: self.params.1,
            samples: 1,
            sample_retract_dist: 2.0,
            samples_tolerance: 0.1,
            samples_tolerance_retries: 0,
            samples_result: "average".to_string(),
        })
    }
    fn get_offsets(&self, _gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        Ok(self.offsets)
    }
    fn start_probe_session(&self, _gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        Ok(Arc::new(FakeSession {
            results: self.results.clone(),
            ran: self.run_probe.clone(),
        }))
    }
}

impl PrinterObject for FakeProbe {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterProbeApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("name".to_string()),
            PyValue::Str(self.name.lock().unwrap().clone()),
        );
        Some(PyValue::Dict(d))
    }
}

struct FakeEddyProbe {
    client: Arc<Mutex<Option<ClientCallback>>>,
    drift_comp: Mutex<Option<Arc<dyn DriftCompensationApi>>>,
}

impl PrinterEddyProbeApi for FakeEddyProbe {
    fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        *self.client.lock().unwrap() = Some(cb);
        Ok(())
    }
    fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>) {
        *self.drift_comp.lock().unwrap() = Some(comp);
    }
}

type ManualStatus = (bool, Option<f64>, Option<f64>, Option<f64>);

struct FakeManualProbe {
    status: Mutex<Option<ManualStatus>>,
    resets: Mutex<usize>,
}

impl ManualProbeApi for FakeManualProbe {
    fn reset_status(&self) {
        *self.resets.lock().unwrap() += 1;
        *self.status.lock().unwrap() = None;
    }
    fn set_status(
        &self,
        is_active: bool,
        z_position: Option<f64>,
        z_position_lower: Option<f64>,
        z_position_upper: Option<f64>,
    ) {
        *self.status.lock().unwrap() =
            Some((is_active, z_position, z_position_lower, z_position_upper));
    }
}
klippy_api::printer_object!(FakeManualProbe; dyn ManualProbeApi);
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakePrinterConfig; dyn PrinterConfigApi);
klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);
klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);
klippy_api::printer_object!(FakeEddyProbe; dyn PrinterEddyProbeApi);

struct FakeSession {
    results: Arc<Mutex<Vec<ProbeResult>>>,
    ran: Arc<Mutex<bool>>,
}

impl ProbeSessionApi for FakeSession {
    fn run_probe(&self, _gcmd: &GCmd) -> KResult<()> {
        *self.ran.lock().unwrap() = true;
        Ok(())
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(self.results.lock().unwrap().clone())
    }
    fn end_probe_session(&self) -> KResult<()> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// World builder
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    config: Config,
    /// The config file text both sides parse with their real config reader.
    config_text: String,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    configfile: Arc<FakePrinterConfig>,
    heaters: Arc<FakeHeaters>,
    sensor: Arc<FakeSensor>,
    probe: Arc<FakeProbe>,
}

impl World {
    /// A `gcmd` from the real dispatcher; the reference builds the same one
    /// with `make_gcmd` (see [`py_gcmd`]).
    fn gcmd(&self, name: &str, params: &[(&str, &str)]) -> GCmd {
        let map: IndexMap<String, String> = params
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        self.gcode
            .dispatch
            .create_gcode_command(name, &commandline(name, params), &map)
    }
}

/// The command line both sides give the dispatcher: `NAME K=V ...`, in the
/// order the parameters are listed (`make_gcmd` in the reference prelude).
fn commandline(name: &str, params: &[(&str, &str)]) -> String {
    let mut line = name.to_string();
    for (k, v) in params {
        line.push_str(&format!(" {k}={v}"));
    }
    line
}

/// The reference's `make_gcmd(name, {params})` call for [`World::gcmd`].
fn py_gcmd(name: &str, params: &[(&str, &str)]) -> String {
    format!("make_gcmd({name:?}, {})", py_params(params))
}

const PROBE_NAME: &str = "temperature_probe";
const EDDY_NAME: &str = "probe_eddy_current temperature_probe";

/// The drift calibration most tests configure.
const DRIFT_ROWS: [[f64; 3]; 3] = [[3000.0, -2.0, 0.0], [2000.0, -2.0, 0.0], [1000.0, -2.0, 0.0]];

/// Config options set on top of the defaults.
#[derive(Clone, Default)]
struct Extra {
    floats: Vec<(&'static str, f64)>,
    floatlists: Vec<(&'static str, Vec<f64>)>,
}

/// The printer config file of a world: the `[temperature_probe]` section and,
/// when present, an empty `[probe_eddy_current temperature_probe]` section.
/// Floats are written with their `repr`, which both readers parse back to
/// the same value.
fn config_text(
    drift_rows: Option<&[Vec<f64>]>,
    cal_temp: Option<f64>,
    with_eddy_section: bool,
    extra: &Extra,
) -> String {
    let join = |values: &[f64]| {
        values.iter().map(|x| format!("{x:?}")).collect::<Vec<_>>().join(", ")
    };
    let mut text = format!("[{PROBE_NAME}]\n");
    if let Some(t) = cal_temp {
        text.push_str(&format!("calibration_temp: {t:?}\n"));
    }
    for (name, value) in &extra.floats {
        text.push_str(&format!("{name}: {value:?}\n"));
    }
    for (name, values) in &extra.floatlists {
        text.push_str(&format!("{name}: {}\n", join(values)));
    }
    if let Some(rows) = drift_rows {
        text.push_str("drift_calibration:\n");
        for row in rows {
            text.push_str(&format!("  {}\n", join(row)));
        }
    }
    if with_eddy_section {
        text.push_str(&format!("\n[{EDDY_NAME}]\n"));
    }
    text
}

/// Python statement that replaces the reference's `config` with a wrapper
/// over `text` (the Rust `Debug` form of an ASCII string is a valid Python
/// string literal).
fn py_config(text: &str) -> String {
    format!("config = make_config({text:?})")
}

/// The real `ConfigWrapper` over `text`, as `PrinterConfig` hands it to
/// `load_config_prefix`.
fn make_config(printer: Printer, text: &str) -> Config {
    let fileconfig =
        ConfigFileReader::build_fileconfig(text, "printer.cfg").expect("parse config text");
    Arc::new(ConfigWrapper::new(
        printer,
        Arc::new(fileconfig),
        Arc::new(GCell::new(IndexMap::new())),
        PROBE_NAME.to_string(),
    ))
}

// ---------------------------------------------------------------------------
// Log capture: the port logs through `klippy_api::log`, the reference through
// the root logger; both end up as message lists that the tests compare.
// ---------------------------------------------------------------------------

thread_local! {
    static LOGS: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}
static LOG_SINK: Once = Once::new();

fn install_log_sink() {
    LOG_SINK.call_once(|| {
        let installed = klippy_api::log::set_sink(Box::new(|rec| {
            LOGS.with(|logs| logs.borrow_mut().push(rec.message.to_string()));
        }));
        assert!(installed, "another log sink was installed first");
    });
}

fn take_logs() -> Vec<String> {
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

fn build_world(
    drift_rows: Option<Vec<[f64; 3]>>,
    cal_temp: Option<f64>,
    with_eddy_section: bool,
) -> World {
    let rows: Option<Vec<Vec<f64>>> =
        drift_rows.map(|rows| rows.iter().map(|r| r.to_vec()).collect());
    build_world_cfg(rows, cal_temp, with_eddy_section, &Extra::default())
}

fn build_world_cfg(
    drift_rows: Option<Vec<Vec<f64>>>,
    cal_temp: Option<f64>,
    with_eddy_section: bool,
    extra: &Extra,
) -> World {
    install_log_sink();
    let printer = FakePrinter::new();
    let gcode = FakeGCode::new(printer.clone() as Printer);
    let toolhead = FakeToolHead::new(vec![10.0, 10.0, 10.0, 0.0], "xyz");
    let configfile = Arc::new(FakePrinterConfig {
        sets: Mutex::new(Vec::new()),
    });
    let sensor = Arc::new(FakeSensor {
        minmax: Mutex::new(None),
        callback: Mutex::new(None),
        report_delta: 0.1,
    });
    let heaters = Arc::new(FakeHeaters {
        sensor: sensor.clone(),
        registered: Mutex::new(Vec::new()),
    });
    let probe = Arc::new(FakeProbe {
        offsets: (1.0, 2.0, 0.5),
        params: (5.0, 10.0),
        name: Mutex::new(format!("probe {}", PROBE_NAME)),
        results: Arc::new(Mutex::new(vec![ProbeResult {
            bed_x: 11.0,
            bed_y: 12.0,
            bed_z: 2.5,
            test_x: 10.0,
            test_y: 10.0,
            test_z: 10.0,
        }])),
        run_probe: Arc::new(Mutex::new(false)),
    });
    let bulk_cell: Arc<Mutex<Option<ClientCallback>>> = Arc::new(Mutex::new(None));
    let eddy = Arc::new(FakeEddyProbe {
        client: Arc::clone(&bulk_cell),
        drift_comp: Mutex::new(None),
    });
    *toolhead.bulk.lock().unwrap() = Some(bulk_cell);
    let manual = Arc::new(FakeManualProbe {
        status: Mutex::new(None),
        resets: Mutex::new(0),
    });
    let text = config_text(drift_rows.as_deref(), cal_temp, with_eddy_section, extra);
    let config = make_config(printer.clone() as Printer, &text);
    take_logs();
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("toolhead", toolhead.clone() as ObjectRef);
    printer.add("configfile", configfile.clone() as ObjectRef);
    printer.add("heaters", heaters.clone() as ObjectRef);
    printer.add("probe", probe.clone() as ObjectRef);
    printer.add(EDDY_NAME, eddy.clone() as ObjectRef);
    printer.add("manual_probe", manual.clone() as ObjectRef);
    World {
        printer,
        config,
        config_text: text,
        gcode,
        toolhead,
        configfile,
        heaters,
        sensor,
        probe,
    }
}

// ---------------------------------------------------------------------------
// pyoracle helpers: Python fakes mirroring the Rust fakes, plus the real
// `configfile.ConfigWrapper` and `gcode.GCodeDispatch`/`GCodeCommand`
// ---------------------------------------------------------------------------

const PY_PRELUDE: &str = r#"
import logging
PY_LOGS = []
class _LogCapture(logging.Handler):
    def emit(self, record):
        PY_LOGS.append(record.getMessage())
logging.getLogger().addHandler(_LogCapture())
logging.getLogger().setLevel(logging.DEBUG)
import configfile as _cf
import gcode as _g
class _Sentinel: pass
sentinel = _Sentinel()
config_error = _cf.error
CommandError = _g.CommandError
_gcode_module = _g
class FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.mux = []
        self.infos = []
        self.scripts = []
        self.scripts_from = []
        self.error = CommandError
        self.preregistered = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            return self.handlers.pop(cmd, None)
        if cmd in self.handlers or cmd in self.preregistered:
            raise config_error("gcode command %s already registered" % cmd)
        self.handlers[cmd] = func
        return None
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value))
    def respond_info(self, msg, log=True):
        self.infos.append(msg)
    def respond_raw(self, msg):
        pass
    def run_script(self, script):
        self.scripts.append(script)
    def run_script_from_command(self, script):
        self.scripts_from.append(script)
    def create_gcode_command(self, command, commandline, params):
        return cmd_dispatch.create_gcode_command(command, commandline, params)
    def is_traditional_gcode(self, cmd):
        return False
    def get_mutex(self):
        return None

def make_gcmd(command, params):
    # Same command line as the Rust `commandline` helper.
    commandline = ' '.join([command] + ['%s=%s' % (k, v) for k, v in params.items()])
    return cmd_dispatch.create_gcode_command(command, commandline, dict(params))

def make_config(text):
    fileconfig = _cf.ConfigFileReader().build_fileconfig(text, 'printer.cfg')
    return _cf.ConfigWrapper(printer, fileconfig, {}, 'temperature_probe')

def build_probe(config):
    global probe_obj
    probe_obj = tpmod.TemperatureProbe(config)

class FakeToolHead:
    def __init__(self, pos, homed):
        self.pos = list(pos)
        self.homed = homed
        self.moves = []
        self.last_move_time = 10.0
        self.bulk_schedule = []
        self.eddy_ref = None
        self.fail_at = None
    def get_position(self):
        return list(self.pos)
    def manual_move(self, coord, speed):
        if self.fail_at is not None and len(self.moves) == self.fail_at:
            raise CommandError("Move out of range")
        self.moves.append((list(coord), speed))
        for i, c in enumerate(coord):
            if c is not None:
                self.pos[i] = c
    def dwell(self, delay):
        batch = self.bulk_schedule.pop(0) if self.bulk_schedule else None
        if batch and self.eddy_ref is not None and self.eddy_ref.client is not None:
            self.eddy_ref.client({'data': [list(x) for x in batch]})
    def wait_moves(self):
        pass
    def get_last_move_time(self):
        self.last_move_time += 1.0
        return self.last_move_time
    def flush_step_generation(self):
        pass
    def get_kinematics(self):
        return FakeKinematics(self.pos)
    def get_status(self, eventtime):
        return {'homed_axes': self.homed}
    def get_extruder(self):
        return _extruder

class FakeKinematics:
    def __init__(self, pos):
        self.pos = pos
    def get_steppers(self):
        return [FakeStepper(self.pos[2])]
    def calc_position(self, spos):
        return list(self.pos)

class FakeStepper:
    name = 'stepper_z'
    def __init__(self, pos):
        self.pos = pos
    def get_name(self):
        return self.name
    def get_commanded_position(self):
        return self.pos

class _Extruder:
    def get_name(self):
        return 'extruder'
_extruder = _Extruder()

class FakePrinter:
    # `Printer.config_error`/`command_error` (klippy/klippy.py:26-27).
    config_error = config_error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise config_error("Unknown config object '%s'" % name)
        return default
    def load_object(self, config, section, default=sentinel):
        return self.lookup_object(section, default)
    def get_reactor(self):
        return _reactor
    def lookup_objects(self, module=None):
        return list(self.objects.items())
    def get_start_args(self):
        return {}
    def register_event_handler(self, event, callback):
        pass

class _Reactor:
    def __init__(self):
        self.async_cbs = []
    def monotonic(self):
        return 100.0
    def pause(self, wake):
        return wake
    def register_async_callback(self, cb, waketime=0.0):
        self.async_cbs.append(cb)
    def run_async(self):
        for cb in self.async_cbs:
            cb(self.monotonic())
        self.async_cbs = []
    def mutex(self, is_locked=False):
        return None
_reactor = _Reactor()

class FakeSensor:
    def __init__(self):
        self.minmax = None
        self.cb = None
    def setup_minmax(self, lo, hi):
        self.minmax = (lo, hi)
    def setup_callback(self, cb):
        self.cb = cb
    def get_report_time_delta(self):
        return 0.1

class FakeHeaters:
    def __init__(self, sensor):
        self.sensor = sensor
        self.registered = []
    def setup_sensor(self, config):
        return self.sensor
    def register_sensor(self, config, psensor, gcode_id=None):
        self.registered.append(config.get_name())
class FakeProbeResult:
    def __init__(self, bed_x, bed_y, bed_z):
        self.bed_x = bed_x
        self.bed_y = bed_y
        self.bed_z = bed_z

class FakeProbe:
    def __init__(self):
        self.offsets = (1.0, 2.0, 0.5)
        self.params = {'probe_speed': 5.0, 'lift_speed': 10.0}
        self.name = 'probe temperature_probe'
        self.results = []
        self.session_ran = False
    def get_offsets(self, gcmd=None):
        return self.offsets
    def get_probe_params(self, gcmd=None):
        return dict(self.params)
    def get_status(self, eventtime=None):
        return {'name': self.name}
    def start_probe_session(self, gcmd):
        return FakeSession(self)

class FakeSession:
    def __init__(self, probe):
        self.probe = probe
    def run_probe(self, gcmd):
        self.probe.session_ran = True
    def pull_probed_results(self):
        return list(self.probe.results)
    def end_probe_session(self):
        pass

class FakeEddyProbe:
    def __init__(self):
        self.client = None
        self.comp = None
    def add_client(self, cb):
        self.client = cb
    def register_drift_compensation(self, comp):
        self.comp = comp

class FakeManualProbe:
    def __init__(self):
        self.status = None
        self.resets = 0
    def reset_status(self):
        self.resets += 1
        self.status = None

"#;

/// The reference world; `CONFIG_TEXT` is replaced with the Rust world's
/// config file text.
const PY_WORLD: &str = r#"
printer = FakePrinter()
cmd_dispatch = _g.GCodeDispatch(printer)
GCMD_OUT = []
cmd_dispatch.register_output_handler(GCMD_OUT.append)
gcode = FakeGCode()
toolhead = FakeToolHead([10.0, 10.0, 10.0, 0.0], 'xyz')
configfile = FakeConfigFile()
sensor = FakeSensor()
heaters = FakeHeaters(sensor)
probe = FakeProbe()
eddy = FakeEddyProbe()
manual = FakeManualProbe()
printer.add_object('gcode', gcode)
printer.add_object('toolhead', toolhead)
printer.add_object('configfile', configfile)
printer.add_object('heaters', heaters)
printer.add_object('probe', probe)
printer.add_object('probe_eddy_current temperature_probe', eddy)
printer.add_object('manual_probe', manual)
toolhead.eddy_ref = eddy
config = make_config(CONFIG_TEXT)
import extras.temperature_probe as tpmod
del PY_LOGS[:]
probe_obj = tpmod.TemperatureProbe(config)
"#;

const PY_CFGFILE: &str = r#"
class FakeConfigFile:
    def __init__(self):
        self.sets = []
    def set(self, section, option, value):
        # `PrinterConfig.set` stores `str(value)` (configfile.py:311-314).
        self.sets.append((section, option, str(value)))
"#;

/// Starts the reference world over `config_text` (the Rust world's file).
fn oracle_start(config_text: &str) -> Option<pyoracle::Oracle> {
    oracle_start_setup(PY_WORLD, config_text)
}

fn oracle_start_setup(py_setup: &str, config_text: &str) -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: oracle unavailable ($KLIPPY_PYTHON unset)");
        return None;
    };
    py.exec(PY_PRELUDE).expect("oracle prelude");
    py.exec(PY_CFGFILE).expect("oracle cfgfile fake");
    let code = py_setup.replace("CONFIG_TEXT", &format!("{config_text:?}"));
    if let Err(e) = py.exec(&code) {
        panic!("oracle setup failed: {e:?}");
    }
    Some(py)
}

/// The setup script without the final `TemperatureProbe(config)`, for tests
/// that build the object themselves (and expect it to fail).
fn oracle_start_nobuild(config_text: &str) -> Option<pyoracle::Oracle> {
    let setup = PY_WORLD.replace("probe_obj = tpmod.TemperatureProbe(config)", "probe_obj = None");
    oracle_start_setup(&setup, config_text)
}


// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn poly_call_str_repr() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: oracle unavailable ($KLIPPY_PYTHON unset)");
        return;
    };
    py.exec("import extras.temperature_probe as tpmod")
        .expect("oracle import");
    let cases: [(f64, f64, f64, Vec<f64>); 4] = [
        (1.0, 2.0, 0.5, vec![0.0, 1.5, -3.0]),
        (0.0, 0.0, 0.0, vec![0.0]),
        (1000000.5, -2.25, 1e-9, vec![10.0]),
        (5.0, 0.0, 0.0, vec![2.0, -1.0, 100.0]),
    ];
    for (a, b, c, xs) in cases {
        let poly = Polynomial2d::new(a, b, c);
        for &x in &xs {
            let expected = py
                .eval(&format!("tpmod.Polynomial2d({a:?}, {b:?}, {c:?})({x:?})"))
                .expect("oracle poly call");
            check_oracle("poly_call", &expected, &Value::float(poly.call(x)));
        }
        let exp_str = py
            .eval(&format!("str(tpmod.Polynomial2d({a:?}, {b:?}, {c:?}))"))
            .expect("oracle poly str");
        check_oracle("poly_str", &exp_str, &Value::Str(poly.py_str()));
        let exp_repr = py
            .eval(&format!("repr(tpmod.Polynomial2d({a:?}, {b:?}, {c:?}))"))
            .expect("oracle poly repr");
        check_oracle("poly_repr", &exp_repr, &Value::Str(poly.py_repr().expect("rust repr")));
        let exp_coefs = py
            .eval(&format!("tpmod.Polynomial2d({a:?}, {b:?}, {c:?}).get_coefs()"))
            .expect("oracle poly coefs");
        let (x, y, z) = poly.get_coefs();
        check_oracle(
            "poly_coefs",
            &exp_coefs,
            &Value::Tuple(vec![Value::float(x), Value::float(y), Value::float(z)]),
        );
    }
}

#[test]
fn poly_fit_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: oracle unavailable ($KLIPPY_PYTHON unset)");
        return;
    };
    py.exec("import extras.temperature_probe as tpmod")
        .expect("oracle import");
    let coords: [(f64, f64); 5] = [
        (0.0, 3000.0),
        (2.0, 2990.0),
        (4.0, 2980.0),
        (6.0, 2970.0),
        (8.0, 2960.0),
    ];
    let coord_list = coords
        .iter()
        .map(|(x, y)| format!("({x:?}, {y:?})"))
        .collect::<Vec<_>>()
        .join(", ");
    let expected = py
        .eval(&format!("tpmod.Polynomial2d.fit([{coord_list}]).get_coefs()"))
        .expect("oracle poly fit");
    let poly = Polynomial2d::fit(&coords).expect("rust fit");
    let (a, b, c) = poly.get_coefs();
    check_oracle(
        "poly_fit",
        &expected,
        &Value::Tuple(vec![Value::float(a), Value::float(b), Value::float(c)]),
    );
    let exp_repr = py
        .eval(&format!("repr(tpmod.Polynomial2d.fit([{coord_list}]))"))
        .expect("oracle poly fit repr");
    check_oracle(
        "poly_fit_repr",
        &exp_repr,
        &Value::Str(poly.py_repr().expect("rust repr")),
    );
}

#[test]
fn poly_edge_cases_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: oracle unavailable ($KLIPPY_PYTHON unset)");
        return;
    };
    py.exec("import extras.temperature_probe as tpmod")
        .expect("oracle import");
    let repr_cases: [(f64, f64, f64); 6] = [
        (f64::NAN, 1.0, 1.0),
        (1.0, f64::INFINITY, 0.0),
        (1.0, 2.0, f64::NEG_INFINITY),
        (-0.5, 1e-11, 3.0),
        (123456789.12345679, -2.00000000001, 4e-9),
        (0.0, -0.0, 0.0),
    ];
    for (a, b, c) in repr_cases {
        let expected = py.eval(&format!(
            "repr(tpmod.Polynomial2d({}, {}, {}))",
            py_float(a),
            py_float(b),
            py_float(c)
        ));
        let actual = Polynomial2d::new(a, b, c).py_repr();
        match (expected, actual) {
            (Ok(e), Ok(s)) => check_oracle("repr", &e, &Value::Str(s)),
            (Err(x), Err(e)) => check_exc("repr", &x, &e),
            (Ok(_), Err(e)) => panic!("repr({a}, {b}, {c}): port raised {}", e.class_name()),
            (Err(x), Ok(s)) => panic!("repr({a}, {b}, {c}): reference raised {}, port gave {s}", x.class_name),
        }
    }
    let fit_cases: Vec<Vec<(f64, f64)>> = vec![
        Vec::new(),
        vec![(0.0, 1.0), (1.0, 2.0)],
        vec![(1.0, 2.0), (1.0, 3.0), (1.0, 4.0)],
        vec![(0.0, 1.0), (1.0, 3.0), (2.0, 7.0), (3.0, 13.0)],
    ];
    for coords in fit_cases {
        let list = coords
            .iter()
            .map(|(x, y)| format!("({x:?}, {y:?})"))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!("tpmod.Polynomial2d.fit([{list}]).get_coefs()"));
        let actual = Polynomial2d::fit(&coords).map(|p| p.get_coefs());
        match (expected, actual) {
            (Ok(e), Ok((a, b, c))) => check_oracle(
                "fit",
                &e,
                &Value::Tuple(vec![Value::float(a), Value::float(b), Value::float(c)]),
            ),
            (Err(x), Err(e)) => check_exc("fit", &x, &e),
            (Ok(_), Err(e)) => panic!("fit {coords:?}: port raised {}", e.class_name()),
            (Err(x), Ok(_)) => panic!("fit {coords:?}: reference raised {}", x.class_name),
        }
    }
}

fn load_rust_probe(world: &World) -> Arc<TemperatureProbe> {
    TemperatureProbe::load_config_prefix(&world.config).expect("rust load_config_prefix")
}

const CALIBRATE: &str = "TEMPERATURE_PROBE_CALIBRATE";
const ENABLE: &str = "TEMPERATURE_PROBE_ENABLE";

/// Runs one command handler with the same command on both sides.
fn run_both(
    world: &World,
    py: &mut pyoracle::Oracle,
    rust: impl FnOnce(&GCmd) -> KResult<()>,
    py_method: &str,
    name: &str,
    params: &[(&str, &str)],
) -> (KResult<()>, Result<Value, Exception>) {
    let rust = rust(&world.gcmd(name, params));
    let expected = py.eval(&format!("probe_obj.{py_method}({})", py_gcmd(name, params)));
    (rust, expected)
}

fn calibrate_both(
    world: &World,
    probe: &Arc<TemperatureProbe>,
    py: &mut pyoracle::Oracle,
    params: &[(&str, &str)],
) -> (KResult<()>, Result<Value, Exception>) {
    run_both(
        world,
        py,
        |gcmd| probe.cmd_calibrate(gcmd),
        "cmd_TEMPERATURE_PROBE_CALIBRATE",
        CALIBRATE,
        params,
    )
}

fn enable_both(
    world: &World,
    probe: &Arc<TemperatureProbe>,
    py: &mut pyoracle::Oracle,
    params: &[(&str, &str)],
) -> (KResult<()>, Result<Value, Exception>) {
    run_both(
        world,
        py,
        |gcmd| probe.cmd_enable(gcmd),
        "cmd_TEMPERATURE_PROBE_ENABLE",
        ENABLE,
        params,
    )
}

/// Both sides raised, with the same class and message.
fn expect_both_raise(label: &str, outcome: (KResult<()>, Result<Value, Exception>)) {
    match outcome {
        (Err(e), Err(x)) => check_exc(label, &x, &e),
        (Ok(()), Err(x)) => panic!("{label}: port succeeded, reference raised {}", x.class_name),
        (Err(e), Ok(_)) => panic!("{label}: port raised {}, reference succeeded", e.class_name()),
        (Ok(()), Ok(_)) => panic!("{label}: both sides succeeded"),
    }
}

#[test]
fn init_defaults_and_status() {
    let world = build_world(None, Some(25.0), true);
    let probe = load_rust_probe(&world);
    let rust_logs = take_logs();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    let exp_logs = py.eval("PY_LOGS").expect("oracle logs");
    check_oracle("init_logs", &exp_logs, &strings_value(&rust_logs));
    let exp_minmax = py.eval("sensor.minmax").expect("oracle minmax");
    let actual_minmax = world.sensor.minmax.lock().unwrap().unwrap();
    check_oracle(
        "init_minmax",
        &exp_minmax,
        &Value::Tuple(vec![Value::float(actual_minmax.0), Value::float(actual_minmax.1)]),
    );
    let exp_reg = py.eval("heaters.registered").expect("oracle registered");
    check_oracle(
        "init_registered",
        &exp_reg,
        &strings_value(&world.heaters.registered.lock().unwrap()),
    );
    let exp_mux = py.eval("gcode.mux").expect("oracle mux");
    let actual_mux: Vec<Value> = world
        .gcode
        .mux
        .lock()
        .unwrap()
        .iter()
        .map(|m| {
            Value::Tuple(vec![
                Value::Str(m.0.clone()),
                Value::Str(m.1.clone()),
                m.2.clone().map(Value::Str).unwrap_or(Value::Null),
            ])
        })
        .collect();
    check_oracle("init_mux", &exp_mux, &Value::List(actual_mux));
    let exp_status = py
        .eval("probe_obj.get_status(0.0)")
        .expect("oracle status");
    check_oracle(
        "init_status",
        &exp_status,
        &pyvalue_to_recording(&probe.status_dict()),
    );
    let exp_temp = py.eval("probe_obj.get_temp()").expect("oracle get_temp");
    let (t, target) = probe.get_temp();
    check_oracle(
        "init_get_temp",
        &exp_temp,
        &Value::Tuple(vec![Value::float(t), pyvalue_to_recording(&target)]),
    );
    let exp_stats = py.eval("probe_obj.stats(0.0)").expect("oracle stats");
    let (_, stat) = probe.stats(0.0);
    check_oracle(
        "init_stats",
        &exp_stats,
        &Value::Tuple(vec![Value::Bool(false), Value::Str(stat)]),
    );
}

#[test]
fn temp_callback_smooths_and_tracks() {
    let world = build_world(None, Some(25.0), true);
    let probe = load_rust_probe(&world);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    let steps: Vec<(f64, f64)> = vec![(1.0, 30.0), (2.5, 31.0), (3.0, 10.0)];
    for (rt, temp) in &steps {
        probe.temp_callback(*rt, *temp);
        py.eval(&format!("sensor.cb({rt:?}, {temp:?})"))
            .expect("oracle temp_callback");
    }
    let exp = py.eval("probe_obj.get_status(0.0)").expect("oracle status");
    check_oracle(
        "cb_status",
        &exp,
        &pyvalue_to_recording(&probe.status_dict()),
    );
}

#[test]
fn calibrate_no_helper() {
    // No `[probe_eddy_current temperature_probe]` section: no helper.
    let world = build_world(None, Some(25.0), false);
    let probe = load_rust_probe(&world);
    let rust_logs = take_logs();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    let exp_logs = py.eval("PY_LOGS").expect("oracle logs");
    check_oracle("no_helper_logs", &exp_logs, &strings_value(&rust_logs));
    let outcome = calibrate_both(&world, &probe, &mut py, &[("TARGET", "40.0")]);
    expect_both_raise("no_helper", outcome);
}

#[test]
fn calibrate_not_homed() {
    let world = build_world(None, Some(25.0), true);
    *world.toolhead.homed_axes.lock().unwrap() = "xy".to_string();
    let probe = load_rust_probe(&world);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec("toolhead.homed = 'xy'").expect("oracle setup");
    let outcome = calibrate_both(&world, &probe, &mut py, &[("TARGET", "40.0")]);
    expect_both_raise("not_homed", outcome);
}

#[test]
fn calibrate_expected_count_error() {
    let world = build_world(None, Some(25.0), true);
    let probe = load_rust_probe(&world);
    probe.temp_callback(10.0, 25.0);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.eval("sensor.cb(10.0, 25.0)").expect("oracle prime");
    let outcome = calibrate_both(&world, &probe, &mut py, &[("TARGET", "26.0"), ("STEP", "2.0")]);
    expect_both_raise("expected_count", outcome);
}

#[test]
fn calibrate_wrong_probe_link() {
    let world = build_world(None, Some(25.0), true);
    *world.probe.name.lock().unwrap() = "probe other".to_string();
    let probe = load_rust_probe(&world);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec("probe.name = 'probe other'").expect("oracle setup");
    let outcome = calibrate_both(&world, &probe, &mut py, &[("TARGET", "40.0")]);
    expect_both_raise("wrong_link", outcome);
}

#[test]
fn calibrate_aux_commands_already_registered() {
    let world = build_world(None, Some(25.0), true);
    world.gcode.pre_register("TEMPERATURE_PROBE_NEXT");
    let probe = load_rust_probe(&world);
    probe.temp_callback(10.0, 25.0);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.eval("sensor.cb(10.0, 25.0)").expect("oracle prime");
    py.exec("gcode.preregistered.append('TEMPERATURE_PROBE_NEXT')")
        .expect("oracle preregister");
    let outcome = calibrate_both(&world, &probe, &mut py, &[("TARGET", "28.0"), ("STEP", "1.0")]);
    expect_both_raise("aux_already_registered", outcome);
}

#[test]
fn calibrate_manual_flow_and_abort() {
    let world = build_world(None, Some(25.0), true);
    let probe = load_rust_probe(&world);
    probe.temp_callback(10.0, 25.0);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.eval("sensor.cb(10.0, 25.0)").expect("oracle prime");
    let outcome = calibrate_both(&world, &probe, &mut py, &[("TARGET", "28.0"), ("STEP", "1.0")]);
    compare_result("calibrate", outcome.0, outcome.1);
    let exp_status = py
        .eval("probe_obj.get_status(0.0)")
        .expect("oracle status");
    check_oracle(
        "calibrating_status",
        &exp_status,
        &pyvalue_to_recording(&probe.status_dict()),
    );
    let outcome = run_both(
        &world,
        &mut py,
        |gcmd| probe.cmd_abort(gcmd),
        "cmd_TEMPERATURE_PROBE_ABORT",
        "ABORT",
        &[],
    );
    compare_result("abort", outcome.0, outcome.1);
    let exp_status = py
        .eval("probe_obj.get_status(0.0)")
        .expect("oracle status");
    check_oracle(
        "aborted_status",
        &exp_status,
        &pyvalue_to_recording(&probe.status_dict()),
    );
    let exp_infos = py.eval("gcode.infos").expect("oracle infos");
    check_oracle(
        "abort_infos",
        &exp_infos,
        &strings_value(&world.gcode.infos.lock().unwrap()),
    );
}

#[test]
fn enable_cmd_rules() {
    let (world, probe, helper) = eddy_with_drift();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    for enable in ["1", "0"] {
        let outcome = enable_both(&world, &probe, &mut py, &[("ENABLE", enable)]);
        compare_result(&format!("enable {enable}"), outcome.0, outcome.1);
        let exp = py.eval("probe_obj.cal_helper.is_enabled()").expect("oracle enabled");
        check_oracle(
            &format!("enabled after {enable}"),
            &exp,
            &pyvalue_to_recording(&helper.is_enabled()),
        );
    }
}

#[test]
fn enable_errors() {
    let world = build_world(None, Some(25.0), true);
    let probe = load_rust_probe(&world);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    let outcome = enable_both(&world, &probe, &mut py, &[("ENABLE", "1")]);
    expect_both_raise("enable_no_dc", outcome);
    let outcome = enable_both(&world, &probe, &mut py, &[("ENABLE", "on")]);
    expect_both_raise("enable_unparsable", outcome);
}

#[test]
fn next_and_complete_after_calibrate() {
    let world = build_world(None, Some(25.0), true);
    let probe = load_rust_probe(&world);
    probe.temp_callback(10.0, 25.0);
    let sched = bulk_schedule();
    world.toolhead.bulk_schedule.lock().unwrap().extend(sched.clone());
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.eval("sensor.cb(10.0, 25.0)").expect("oracle prime");
    py.exec(&format!("toolhead.bulk_schedule = {}", bulk_schedule_py(&sched))).expect("oracle schedule");
    py.exec("probe.results = [FakeProbeResult(11.0, 12.0, 2.5)]").expect("oracle results");
    // METHOD=tap: no manual probe is started, so COMPLETE passes
    // verify_no_manual_probe on both sides.
    let outcome = calibrate_both(
        &world,
        &probe,
        &mut py,
        &[("TARGET", "28.0"), ("STEP", "1.0"), ("METHOD", "tap")],
    );
    compare_result("calibrate", outcome.0, outcome.1);
    // Sample count is 1, so COMPLETE aborts (sample_count < 3).
    let outcome = run_both(
        &world,
        &mut py,
        |gcmd| probe.cmd_complete(gcmd),
        "cmd_TEMPERATURE_PROBE_COMPLETE",
        "TEMPERATURE_PROBE_COMPLETE",
        &[],
    );
    compare_result("complete", outcome.0, outcome.1);
    let exp_status = py.eval("probe_obj.get_status(0.0)").expect("oracle status");
    check_oracle(
        "complete_status",
        &exp_status,
        &pyvalue_to_recording(&probe.status_dict()),
    );
    let exp_infos = py.eval("gcode.infos").expect("oracle infos");
    check_oracle(
        "complete_infos",
        &exp_infos,
        &strings_value(&world.gcode.infos.lock().unwrap()),
    );
}

/// Bulk schedule used by collect_sample-based tests: 9 batches whose
/// ptimes match the fake toolhead's move-time sequence. `base` is the
/// fake toolhead's `last_move_time` before the call (10.0 initially, then
/// 19.0, 28.0, ... after each `collect_sample`).
fn bulk_schedule_at(base: f64, freq_offset: f64) -> Vec<Vec<(f64, f64, f64)>> {
    (0..9)
        .map(|i| {
            let start = base + 1.0 + i as f64 + 0.05;
            let end = start + 0.1;
            vec![
                (start - 1.0, 99999.0, 0.0),
                (start + 0.04, 3000.0 - 400.0 * i as f64 - freq_offset, 0.1 * i as f64),
                (end + 1.0, 99999.0, 9.9),
            ]
        })
        .collect()
}

fn bulk_schedule() -> Vec<Vec<(f64, f64, f64)>> {
    bulk_schedule_at(10.0, 0.0)
}

/// Render a bulk schedule as a Python list-of-lists-of-tuples literal.
fn bulk_schedule_py(sched: &[Vec<(f64, f64, f64)>]) -> String {
    let inner: Vec<String> = sched
        .iter()
        .map(|batch| {
            format!(
                "[{}]",
                batch
                    .iter()
                    .map(|(a, b, c)| format!("({a:?}, {b:?}, {c:?})"))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        })
        .collect();
    format!("[{}]", inner.join(", "))
}

/// A world whose config carries [`DRIFT_ROWS`], so the probe owns a drift
/// helper; the reference's `probe_obj.cal_helper` is the same object
/// ([`PY_HELPER`]).
fn eddy_with_drift() -> (World, Arc<TemperatureProbe>, Arc<EddyDriftCompensation>) {
    let world = build_world(Some(DRIFT_ROWS.to_vec()), Some(25.0), true);
    let probe = load_rust_probe(&world);
    let helper = probe.cal_helper().expect("cal helper");
    (world, probe, helper)
}

const PY_HELPER: &str = "_h = probe_obj.cal_helper";

#[test]
fn drift_adjust_and_unadjust() {
    let (world, probe, helper) = eddy_with_drift();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec(PY_HELPER).expect("oracle helper");
    let outcome = enable_both(&world, &probe, &mut py, &[("ENABLE", "1")]);
    compare_result("enable", outcome.0, outcome.1);
    for &freq in &[3000.0, 2500.0, 1500.0, 1200.0, 900.0] {
        let exp_adj = py
            .eval(&format!("_h.adjust_freq({freq:?}, 25.0)"))
            .expect("oracle adjust");
        check_oracle(
            "adjust_freq",
            &exp_adj,
            &Value::float(helper.adjust_freq(freq, Some(25.0)).expect("rust adjust")),
        );
        let exp_unadj = py
            .eval(&format!("_h.unadjust_freq({freq:?}, 30.0)"))
            .expect("oracle unadjust");
        check_oracle(
            "unadjust_freq",
            &exp_unadj,
            &Value::float(helper.unadjust_freq(freq, Some(30.0)).expect("rust unadjust")),
        );
    }
    // Without an explicit temperature both sides read the probe's smoothed
    // temperature.
    probe.temp_callback(10.0, 40.0);
    py.eval("sensor.cb(10.0, 40.0)").expect("oracle temp");
    for &freq in &[2900.0, 1900.0, 500.0] {
        let exp_adj = py.eval(&format!("_h.adjust_freq({freq:?})")).expect("oracle adjust");
        check_oracle(
            "adjust_freq current",
            &exp_adj,
            &Value::float(helper.adjust_freq(freq, None).expect("rust adjust")),
        );
        let exp_unadj = py.eval(&format!("_h.unadjust_freq({freq:?})")).expect("oracle unadjust");
        check_oracle(
            "unadjust_freq current",
            &exp_unadj,
            &Value::float(helper.unadjust_freq(freq, None).expect("rust unadjust")),
        );
    }
}

/// Rust side of `compare_result`: the port's outcome against the reference's.
fn compare_result(label: &str, rust: KResult<()>, py: Result<Value, Exception>) {
    match (rust, py) {
        (Ok(()), Ok(_)) => {}
        (Err(e), Err(x)) => check_exc(label, &x, &e),
        (Ok(()), Err(x)) => panic!(
            "{label}: port succeeded, reference raised {}: {}",
            x.class_name, x.message
        ),
        (Err(e), Ok(_)) => panic!(
            "{label}: port raised {}: {}, reference succeeded",
            e.class_name(),
            e.message()
        ),
    }
}

fn strings_value(v: &[String]) -> Value {
    Value::List(v.iter().map(|s| Value::Str(s.clone())).collect())
}

fn floats_value(v: &[f64]) -> Value {
    Value::List(v.iter().map(|&x| Value::float(x)).collect())
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
                            .map(|c| c.map(Value::float).unwrap_or(Value::Null))
                            .collect(),
                    ),
                    Value::float(*speed),
                ])
            })
            .collect(),
    )
}

/// Python dict literal for a command's parameters.
fn py_params(params: &[(&str, &str)]) -> String {
    let items: Vec<String> = params.iter().map(|(k, v)| format!("{k:?}: {v:?}")).collect();
    format!("{{{}}}", items.join(", "))
}

fn py_poly_list(polys: &[(f64, f64, f64)]) -> String {
    polys
        .iter()
        .map(|(a, b, c)| format!("tpmod.Polynomial2d({a:?}, {b:?}, {c:?})"))
        .collect::<Vec<_>>()
        .join(", ")
}

#[test]
fn drift_check_calibration_matches() {
    let (world, _probe, helper) = eddy_with_drift();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec(PY_HELPER).expect("oracle helper");
    type Poly = (f64, f64, f64);
    let down: Vec<Poly> = vec![(3000.0, -2.0, 0.0), (2000.0, -2.0, 0.0)];
    let cases: Vec<(Vec<Poly>, f64, f64)> = vec![
        (vec![(1000.0, 2.0, 0.0), (2000.0, -2.0, 0.0)], 0.0, 10.0),
        (down.clone(), 0.0, 10.0),
        (
            vec![(3000.0, -2.0, 0.0), (2000.0, -2.0, 0.0), (2500.0, -2.0, 0.0)],
            0.5,
            3.7,
        ),
        (down.clone(), 5.0, 1.0),
        (down.clone(), -3.9, 2.2),
        (Vec::new(), 0.0, 10.0),
        (vec![(3000.0, -2.0, 0.0), (3000.0, -2.0, 0.0)], 0.0, 0.0),
        (down.clone(), f64::NAN, 10.0),
        (down, 0.0, f64::INFINITY),
    ];
    for (idx, (polys, start, end)) in cases.iter().enumerate() {
        let rust_polys: Vec<Polynomial2d> = polys
            .iter()
            .map(|&(a, b, c)| Polynomial2d::new(a, b, c))
            .collect();
        let rust = helper.check_calibration(&rust_polys, *start, *end);
        let expected = py.eval(&format!(
            "_h._check_calibration([{}], {}, {})",
            py_poly_list(polys),
            py_float(*start),
            py_float(*end),
        ));
        compare_result(&format!("check_calibration[{idx}]"), rust, expected);
    }
}

/// Python float expression for any finite or non-finite value.
fn py_float(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x.is_infinite() {
        if x > 0.0 { "float('inf')".to_string() } else { "float('-inf')".to_string() }
    } else {
        format!("{x:?}")
    }
}

/// Builds the probe on both sides from the Rust world's config text and
/// compares the outcome and the construction logs. The reference's object
/// becomes `probe_obj` when it builds.
fn construct_both(
    py: &mut pyoracle::Oracle,
    label: &str,
    world: &World,
) -> KResult<Arc<TemperatureProbe>> {
    take_logs();
    let rust = TemperatureProbe::load_config_prefix(&world.config);
    let rust_logs = take_logs();
    py.exec(&py_config(&world.config_text)).expect("oracle config");
    py.exec("del PY_LOGS[:]").expect("oracle clear logs");
    let expected = py.eval("build_probe(config)");
    match (&rust, &expected) {
        (Ok(_), Ok(_)) => {}
        (Err(e), Err(x)) => check_exc(label, x, e),
        (Ok(_), Err(x)) => panic!(
            "{label}: port built the probe, reference raised {}: {}",
            x.class_name, x.message
        ),
        (Err(e), Ok(_)) => panic!(
            "{label}: port raised {}: {}, reference built the probe",
            e.class_name(),
            e.message()
        ),
    }
    let exp_logs = py.eval("PY_LOGS").expect("oracle logs");
    check_oracle(&format!("{label}:construct_logs"), &exp_logs, &strings_value(&rust_logs));
    rust
}

#[test]
fn drift_init_variants_match() {
    let Some(mut py) = oracle_start_nobuild(&config_text(None, Some(25.0), true, &Extra::default()))
    else {
        return;
    };
    let rows3: Vec<Vec<f64>> = DRIFT_ROWS.iter().map(|r| r.to_vec()).collect();
    type Case = (&'static str, Option<Vec<Vec<f64>>>, Option<f64>);
    let cases: Vec<Case> = vec![
        ("no_rows", None, Some(25.0)),
        ("rows_cal_temp", Some(rows3.clone()), Some(25.0)),
        ("rows_no_cal_temp", Some(rows3.clone()), None),
        ("rows_tiny_cal_temp", Some(rows3.clone()), Some(5e-7)),
        (
            "rows_repr_rounding",
            Some(vec![vec![3000.0, -2.0, 1e-9], vec![2000.5, -2.0, 0.0], vec![1000.0, -2.0, 0.0]]),
            Some(25.0),
        ),
    ];
    for (label, rows, cal_temp) in cases {
        let world = build_world_cfg(rows, cal_temp, true, &Extra::default());
        let probe = construct_both(&mut py, label, &world).expect("rust construct");
        let exp_status = py.eval("probe_obj.get_status(0.0)").expect("oracle status");
        check_oracle(
            &format!("{label}:init_status"),
            &exp_status,
            &pyvalue_to_recording(&probe.status_dict()),
        );
        for enable in ["1", "0", "2"] {
            let outcome = enable_both(&world, &probe, &mut py, &[("ENABLE", enable)]);
            compare_result(&format!("{label}:enable {enable}"), outcome.0, outcome.1);
            let exp_status = py.eval("probe_obj.get_status(0.0)").expect("oracle status");
            check_oracle(
                &format!("{label}:status after enable {enable}"),
                &exp_status,
                &pyvalue_to_recording(&probe.status_dict()),
            );
        }
        // ENABLE is required by the command.
        let outcome = enable_both(&world, &probe, &mut py, &[]);
        compare_result(&format!("{label}:enable missing"), outcome.0, outcome.1);
    }
}

#[test]
fn drift_init_errors_match() {
    let Some(mut py) = oracle_start_nobuild(&config_text(None, Some(25.0), true, &Extra::default()))
    else {
        return;
    };
    type Case = (&'static str, Vec<Vec<f64>>);
    let cases: Vec<Case> = vec![
        ("overlap", vec![vec![1000.0, 2.0, 0.0], vec![2000.0, -2.0, 0.0]]),
        ("short_row", vec![vec![1.0, 2.0]]),
        ("long_row", vec![vec![1.0, 2.0, 3.0, 4.0]]),
        ("empty", Vec::new()),
    ];
    for (label, rows) in cases {
        let world = build_world_cfg(Some(rows), Some(25.0), true, &Extra::default());
        let rust = construct_both(&mut py, label, &world);
        assert!(rust.is_err(), "{label}: the port accepted the calibration");
    }
}

#[test]
fn config_option_bounds_match() {
    let Some(mut py) = oracle_start_nobuild(&config_text(None, Some(25.0), true, &Extra::default()))
    else {
        return;
    };
    let cases: Vec<(&'static str, f64)> = vec![
        ("speed", 0.0),
        ("horizontal_move_z", 0.0),
        ("resting_z", -1.0),
        ("calibration_bed_temp", 50.0),
        ("calibration_extruder_temp", 49.5),
        ("extruder_heating_z", 0.0),
        ("smooth_time", 0.0),
        ("min_temp", -300.0),
        ("max_temp", -273.15),
    ];
    for (name, value) in cases {
        let extra = Extra {
            floats: vec![(name, value)],
            floatlists: Vec::new(),
        };
        let world = build_world_cfg(None, Some(25.0), true, &extra);
        let rust = construct_both(&mut py, &format!("bound[{name}]"), &world);
        assert!(rust.is_err(), "bound[{name}]: the port accepted {value}");
    }
    let extra = Extra {
        floats: Vec::new(),
        floatlists: vec![("calibration_position", vec![1.0, 2.0])],
    };
    let world = build_world_cfg(None, Some(25.0), true, &extra);
    let rust = construct_both(&mut py, "bound[calibration_position]", &world);
    assert!(rust.is_err(), "calibration_position: the port accepted two elements");
}

#[test]
fn config_options_reach_sensor_and_smoothing() {
    let extra = Extra {
        floats: vec![("smooth_time", 4.0), ("min_temp", 10.0), ("max_temp", 120.0)],
        floatlists: Vec::new(),
    };
    let world = build_world_cfg(None, Some(25.0), true, &extra);
    let probe = load_rust_probe(&world);
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    let exp_minmax = py.eval("sensor.minmax").expect("oracle minmax");
    let actual = world.sensor.minmax.lock().unwrap().unwrap();
    check_oracle(
        "minmax",
        &exp_minmax,
        &Value::Tuple(vec![Value::float(actual.0), Value::float(actual.1)]),
    );
    for (rt, temp) in [(1.0, 30.0), (2.0, 31.0), (2.5, 29.0), (8.0, 60.0), (7.0, 20.0)] {
        probe.temp_callback(rt, temp);
        py.eval(&format!("sensor.cb({rt:?}, {temp:?})")).expect("oracle callback");
        let exp = py.eval("probe_obj.get_status(0.0)").expect("oracle status");
        check_oracle(
            "smoothing",
            &exp,
            &pyvalue_to_recording(&probe.status_dict()),
        );
        let exp_stats = py.eval("probe_obj.stats(0.0)").expect("oracle stats");
        let (_, stat) = probe.stats(0.0);
        check_oracle(
            "smoothing stats",
            &exp_stats,
            &Value::Tuple(vec![Value::Bool(false), Value::Str(stat)]),
        );
    }
}

fn collect_case(label: &str, missing_batch: Option<usize>) {
    let (world, probe, helper) = eddy_with_drift();
    probe.temp_callback(10.0, 25.0);
    let mut sched = bulk_schedule();
    if let Some(batch) = missing_batch {
        sched[batch].remove(1);
    }
    world.toolhead.bulk_schedule.lock().unwrap().extend(sched.clone());
    let mpresult = ProbeResult {
        bed_x: 11.0,
        bed_y: 12.0,
        bed_z: 2.5,
        test_x: 10.0,
        test_y: 10.0,
        test_z: 10.0,
    };
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec(PY_HELPER).expect("oracle helper");
    py.eval("sensor.cb(10.0, 25.0)").expect("oracle prime");
    py.exec(&format!("toolhead.bulk_schedule = {}", bulk_schedule_py(&sched)))
        .expect("oracle schedule");
    py.exec("del PY_LOGS[:]").expect("oracle clear logs");
    take_logs();
    let rust = helper.collect_sample(&mpresult, 10.0, (10.0, 5.0, 10.0));
    let expected = py.eval("_h.collect_sample(FakeProbeResult(11.0, 12.0, 2.5), 10.0, (10.0, 5.0, 10.0))");
    match (rust, expected) {
        (Ok(v), Ok(e)) => check_oracle(&format!("{label}:ret"), &e, &Value::float(v)),
        (Err(e), Err(x)) => check_exc(label, &x, &e),
        (Ok(v), Err(x)) => panic!("{label}: port returned {v}, reference raised {}", x.class_name),
        (Err(e), Ok(_)) => panic!("{label}: port raised {}, reference returned", e.class_name()),
    }
    let exp_moves = py.eval("toolhead.moves").expect("oracle moves");
    check_oracle(
        &format!("{label}:moves"),
        &exp_moves,
        &moves_value(&world.toolhead.moves.lock().unwrap()),
    );
    let exp_logs = py.eval("PY_LOGS").expect("oracle logs");
    check_oracle(&format!("{label}:logs"), &exp_logs, &strings_value(&take_logs()));
}

#[test]
fn collect_sample_matches() {
    collect_case("collect", None);
}

#[test]
fn collect_sample_missing_batch_matches() {
    collect_case("collect_missing", Some(3));
}

#[test]
fn finish_calibration_error_paths() {
    let (world, _probe, helper) = eddy_with_drift();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec(PY_HELPER).expect("oracle helper");
    // Fresh helper (samples None): Python hits `len(None)` -> TypeError.
    let exp_none = py.eval("_h.finish_calibration(True)").expect_err("oracle finish none");
    let err_none = helper.finish_calibration(true).unwrap_err();
    check_exc("finish_type_error", &exp_none, &err_none);
    // After start_calibration the 9 sample rows are empty; finish(True)
    // hits `cal_samples[0][0]` -> IndexError in Python.
    py.exec("_h.start_calibration()").expect("oracle start");
    helper.start_calibration();
    let exp_idx = py.eval("_h.finish_calibration(True)").expect_err("oracle finish idx");
    let err_idx = helper.finish_calibration(true).unwrap_err();
    check_exc("finish_index_error", &exp_idx, &err_idx);
    // Abort succeeds silently with samples cleared.
    helper.finish_calibration(false).expect("abort finish");
    py.eval("_h.finish_calibration(False)").expect("oracle abort finish");
}

/// `configfile.set` records `str(value)` (`klippy/configfile.py:311-314`), so the
/// fakes compare the same text the port writes.
fn config_sets_value(sets: &[(String, String, String)]) -> Value {
    Value::List(
        sets.iter()
            .map(|(s, o, v)| {
                Value::Tuple(vec![
                    Value::Str(s.clone()),
                    Value::Str(o.clone()),
                    Value::Str(v.clone()),
                ])
            })
            .collect(),
    )
}

#[test]
fn note_z_calibration_writes_temperature() {
    let (world, probe, helper) = eddy_with_drift();
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec(PY_HELPER).expect("oracle helper");
    probe.temp_callback(10.0, 26.5);
    py.eval("sensor.cb(10.0, 26.5)").expect("oracle prime");
    helper.note_z_calibration_start().expect("rust note start");
    py.eval("_h.note_z_calibration_start()").expect("oracle note start");
    probe.temp_callback(20.0, 27.5);
    py.eval("sensor.cb(20.0, 27.5)").expect("oracle prime 2");
    helper.note_z_calibration_finish().expect("rust note finish");
    py.eval("_h.note_z_calibration_finish()").expect("oracle note finish");
    let exp_sets = py.eval("configfile.sets").expect("oracle sets");
    let actual_sets: Vec<(String, String, String)> =
        world.configfile.sets.lock().unwrap().clone();
    check_oracle("note_sets", &exp_sets, &config_sets_value(&actual_sets));
    let exp_infos = py.eval("gcode.infos").expect("oracle infos");
    check_oracle(
        "note_infos",
        &exp_infos,
        &strings_value(&world.gcode.infos.lock().unwrap()),
    );
    let exp_temp = py.eval("_h.get_temperature()").expect("oracle temperature");
    check_oracle(
        "note_temperature",
        &exp_temp,
        &Value::float(helper.get_temperature().expect("rust temperature")),
    );
}

#[test]
fn finish_calibration_writes_polynomials() {
    let (world, probe, helper) = eddy_with_drift();
    helper.start_calibration();
    let mut sched = Vec::new();
    for (call, offset) in [0.0_f64, 100.0, 200.0].into_iter().enumerate() {
        sched.extend(bulk_schedule_at(10.0 + 9.0 * call as f64, offset));
    }
    world
        .toolhead
        .bulk_schedule
        .lock()
        .unwrap()
        .extend(sched.clone());
    let mpresult = ProbeResult {
        bed_x: 11.0,
        bed_y: 12.0,
        bed_z: 2.5,
        test_x: 10.0,
        test_y: 10.0,
        test_z: 10.0,
    };
    let Some(mut py) = oracle_start(&world.config_text) else { return };
    py.exec(PY_HELPER).expect("oracle helper");
    py.exec(&format!("toolhead.bulk_schedule = {}", bulk_schedule_py(&sched)))
        .expect("oracle schedule");
    py.eval("_h.start_calibration()").expect("oracle start");
    py.exec("del PY_LOGS[:]").expect("oracle clear logs");
    take_logs();
    for (read_time, temp) in [(10.0_f64, 25.0_f64), (20.0, 27.0), (30.0, 29.0)] {
        probe.temp_callback(read_time, temp);
        py.eval(&format!("sensor.cb({read_time:?}, {temp:?})"))
            .expect("oracle prime");
        helper
            .collect_sample(&mpresult, 10.0, (10.0, 5.0, 10.0))
            .expect("rust collect");
        py.eval("_h.collect_sample(FakeProbeResult(11.0, 12.0, 2.5), 10.0, (10.0, 5.0, 10.0))")
            .expect("oracle collect");
    }
    helper.finish_calibration(true).expect("rust finish");
    py.eval("_h.finish_calibration(True)").expect("oracle finish");
    let exp_sets = py.eval("configfile.sets").expect("oracle sets");
    let actual_sets: Vec<(String, String, String)> =
        world.configfile.sets.lock().unwrap().clone();
    check_oracle("finish_sets", &exp_sets, &config_sets_value(&actual_sets));
    let exp_infos = py.eval("gcode.infos").expect("oracle infos");
    check_oracle(
        "finish_infos",
        &exp_infos,
        &strings_value(&world.gcode.infos.lock().unwrap()),
    );
    let exp_logs = py.eval("PY_LOGS").expect("oracle logs");
    check_oracle("finish_logs", &exp_logs, &strings_value(&take_logs()));
}

// ---------------------------------------------------------------------------
// Whole calibration runs: the port and the reference are driven step by step
// and every observable is compared after each step.
// ---------------------------------------------------------------------------

struct Flow {
    world: World,
    probe: Arc<TemperatureProbe>,
    py: pyoracle::Oracle,
    rust_logs: Vec<String>,
}

fn flow_extra() -> Extra {
    Extra {
        floats: vec![
            ("speed", 40.0),
            ("horizontal_move_z", 3.0),
            ("resting_z", 0.5),
            ("calibration_bed_temp", 60.0),
            ("calibration_extruder_temp", 200.0),
            ("extruder_heating_z", 30.0),
            ("smooth_time", 1.0),
        ],
        floatlists: vec![("calibration_position", vec![50.0, 60.0, 5.0])],
    }
}

/// Bulk batches for `samples` consecutive `collect_sample` calls; sample `k`
/// reads `100 * k` Hz lower than sample 0.
fn flow_schedule(samples: usize) -> Vec<Vec<(f64, f64, f64)>> {
    let mut sched = Vec::new();
    for k in 0..samples {
        sched.extend(bulk_schedule_at(10.0 + 9.0 * k as f64, 100.0 * k as f64));
    }
    sched
}

impl Flow {
    fn start(extra: &Extra) -> Option<Flow> {
        let world = build_world_cfg(None, Some(25.0), true, extra);
        let probe = load_rust_probe(&world);
        let py = oracle_start(&world.config_text)?;
        let mut flow = Flow {
            world,
            probe,
            py,
            rust_logs: Vec::new(),
        };
        flow.compare("init");
        Some(flow)
    }

    fn temp(&mut self, read_time: f64, temp: f64) {
        self.probe.temp_callback(read_time, temp);
        self.py
            .eval(&format!("sensor.cb({read_time:?}, {temp:?})"))
            .expect("oracle temp callback");
    }

    fn bulk(&mut self, sched: &[Vec<(f64, f64, f64)>]) {
        self.world
            .toolhead
            .bulk_schedule
            .lock()
            .unwrap()
            .extend(sched.iter().cloned());
        self.py
            .exec(&format!("toolhead.bulk_schedule.extend({})", bulk_schedule_py(sched)))
            .expect("oracle schedule");
    }

    fn set_z(&mut self, z: f64) {
        self.world.toolhead.position.lock().unwrap()[2] = z;
        self.py.exec(&format!("toolhead.pos[2] = {z:?}")).expect("oracle set z");
    }

    fn set_bed_z(&mut self, z: f64) {
        self.world.probe.results.lock().unwrap()[0].bed_z = z;
        self.py
            .exec(&format!("probe.results = [FakeProbeResult(11.0, 12.0, {z:?})]"))
            .expect("oracle probe results");
    }

    fn fail_move_at(&mut self, n: usize) {
        *self.world.toolhead.fail_move_at.lock().unwrap() = Some(n);
        self.py.exec(&format!("toolhead.fail_at = {n}")).expect("oracle fail at");
    }

    fn remove_object(&mut self, name: &str) {
        self.world.printer.objects.lock().unwrap().shift_remove(name);
        self.py
            .exec(&format!("printer.objects.pop({name:?})"))
            .expect("oracle remove object");
    }

    fn calibrate(&mut self, label: &str, params: &[(&str, &str)]) {
        let (rust, expected) = calibrate_both(&self.world, &self.probe, &mut self.py, params);
        compare_result(label, rust, expected);
        self.compare(label);
    }

    /// Runs a command the port registered with the dispatcher (`NEXT`,
    /// `COMPLETE`, `ABORT`, `ACCEPT`, `TESTZ`) and the same handler in the
    /// reference.
    fn run_handler(&mut self, label: &str, name: &str, params: &[(&str, &str)]) {
        let gcmd = self.world.gcmd(name, params);
        let taken = {
            let mut handlers = self.world.gcode.handlers.lock().unwrap();
            handlers.get_mut(name).and_then(|slot| slot.take())
        };
        let mut handler = taken.unwrap_or_else(|| panic!("{label}: handler {name} not registered"));
        let rust = handler(&gcmd);
        {
            // The slot stays registered during the call, like Python's dict.
            let mut handlers = self.world.gcode.handlers.lock().unwrap();
            if let Some(slot) = handlers.get_mut(name) {
                if slot.is_none() {
                    *slot = Some(handler);
                }
            }
        }
        let expected = self
            .py
            .eval(&format!("gcode.handlers[{name:?}]({})", py_gcmd(name, params)));
        compare_result(label, rust, expected);
        self.compare(label);
    }

    fn compare(&mut self, label: &str) {
        self.rust_logs.extend(take_logs());
        let handler_keys: Vec<String> = self
            .world
            .gcode
            .handlers
            .lock()
            .unwrap()
            .keys()
            .cloned()
            .collect();
        let (temp, target) = self.probe.get_temp();
        let checks: Vec<(&str, &str, Value)> = vec![
            ("infos", "gcode.infos", strings_value(&self.world.gcode.infos.lock().unwrap())),
            ("scripts", "gcode.scripts", strings_value(&self.world.gcode.scripts.lock().unwrap())),
            (
                "scripts_from",
                "gcode.scripts_from",
                strings_value(&self.world.gcode.scripts_from.lock().unwrap()),
            ),
            ("moves", "toolhead.moves", moves_value(&self.world.toolhead.moves.lock().unwrap())),
            ("position", "toolhead.pos", floats_value(&self.world.toolhead.position.lock().unwrap())),
            ("handlers", "list(gcode.handlers.keys())", strings_value(&handler_keys)),
            ("status", "probe_obj.get_status(0.0)", pyvalue_to_recording(&self.probe.status_dict())),
            (
                "get_temp",
                "probe_obj.get_temp()",
                Value::Tuple(vec![Value::float(temp), pyvalue_to_recording(&target)]),
            ),
            ("logs", "PY_LOGS", strings_value(&self.rust_logs)),
            (
                "gcmd_responses",
                "GCMD_OUT",
                strings_value(&self.world.gcode.gcmd_out.lock().unwrap()),
            ),
            (
                "config_sets",
                "configfile.sets",
                config_sets_value(&self.world.configfile.sets.lock().unwrap()),
            ),
        ];
        for (name, expr, actual) in checks {
            let expected = self.py.eval(expr).expect("oracle observation");
            check_oracle(&format!("{label}:{name}"), &expected, &actual);
        }
    }
}

#[test]
fn flow_tap_runs_to_target() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.bulk(&flow_schedule(3));
    f.set_bed_z(2.5);
    f.calibrate(
        "calibrate",
        &[("TARGET", "28.0"), ("STEP", "1.0"), ("METHOD", "Tap")],
    );
    f.calibrate(
        "calibrate again",
        &[("TARGET", "28.0"), ("STEP", "1.0"), ("METHOD", "tap")],
    );
    f.temp(20.0, 26.0);
    f.set_bed_z(2.4);
    f.run_handler("next 1", "TEMPERATURE_PROBE_NEXT", &[]);
    f.temp(30.0, 28.5);
    f.set_bed_z(2.45);
    f.run_handler("next 2", "TEMPERATURE_PROBE_NEXT", &[]);
}

#[test]
fn flow_manual_completes_with_three_samples() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.bulk(&flow_schedule(3));
    f.calibrate("calibrate", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    f.calibrate("calibrate during manual probe", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    f.run_handler("testz", "TESTZ", &[("Z", "-1.0")]);
    f.run_handler("accept 1", "ACCEPT", &[]);
    f.temp(20.0, 26.0);
    f.run_handler("next 1", "TEMPERATURE_PROBE_NEXT", &[]);
    f.set_z(2.0);
    f.run_handler("accept 2", "ACCEPT", &[]);
    f.temp(30.0, 27.0);
    f.run_handler("next 2", "TEMPERATURE_PROBE_NEXT", &[]);
    f.set_z(1.5);
    f.run_handler("accept 3", "ACCEPT", &[]);
    f.run_handler("complete", "TEMPERATURE_PROBE_COMPLETE", &[]);
}

#[test]
fn flow_manual_abort_and_early_complete() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.bulk(&flow_schedule(2));
    f.calibrate("calibrate", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    // ACCEPT without lowering the nozzle fails and aborts the calibration.
    f.run_handler("accept too high", "ACCEPT", &[]);
    f.calibrate("calibrate again", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    f.set_z(3.0);
    f.run_handler("accept 1", "ACCEPT", &[]);
    f.run_handler("complete early", "TEMPERATURE_PROBE_COMPLETE", &[]);
    f.calibrate("calibrate third", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    f.set_z(3.0);
    f.run_handler("accept 2", "ACCEPT", &[]);
    f.run_handler("abort", "ABORT", &[]);
}

#[test]
fn flow_manual_helper_abort() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.calibrate("calibrate", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    f.run_handler("helper abort", "ABORT", &[]);
}

#[test]
fn flow_degenerate_fit_matches() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.bulk(&flow_schedule(3));
    f.calibrate("calibrate", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    f.run_handler("testz", "TESTZ", &[("Z", "-1.0")]);
    f.run_handler("accept 1", "ACCEPT", &[]);
    // The temperature never changes, so the three samples share one x value
    // and the least squares system is singular.
    f.temp(20.0, 25.0);
    f.run_handler("next 1", "TEMPERATURE_PROBE_NEXT", &[]);
    f.set_z(2.0);
    f.run_handler("accept 2", "ACCEPT", &[]);
    f.run_handler("next 2", "TEMPERATURE_PROBE_NEXT", &[]);
    f.set_z(1.5);
    f.run_handler("accept 3", "ACCEPT", &[]);
    f.run_handler("complete", "TEMPERATURE_PROBE_COMPLETE", &[]);
}

#[test]
fn flow_failed_initial_move_finalizes() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.fail_move_at(0);
    f.calibrate("calibrate", &[("TARGET", "30.0"), ("STEP", "1.0")]);
    // The failed run released its commands, so the next run fails the same way.
    f.calibrate("calibrate after failure", &[("TARGET", "30.0"), ("STEP", "1.0")]);
}

#[test]
fn flow_without_probe_object_raises() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.remove_object("probe");
    f.calibrate("calibrate", &[("TARGET", "30.0"), ("STEP", "1.0")]);
}

#[test]
fn flow_target_and_step_validation() {
    let Some(mut f) = Flow::start(&Extra::default()) else { return };
    f.temp(10.0, 25.0);
    f.calibrate("missing target", &[]);
    f.calibrate("target not above", &[("TARGET", "25.0")]);
    f.calibrate("step below minimum", &[("TARGET", "40.0"), ("STEP", "0.5")]);
    f.calibrate("too few samples", &[("TARGET", "26.0"), ("STEP", "2.0")]);
    f.calibrate("target as number text", &[("TARGET", "40"), ("STEP", "2")]);
}

#[test]
fn flow_not_homed_and_unlinked_probe() {
    let Some(mut f) = Flow::start(&Extra::default()) else { return };
    f.temp(10.0, 25.0);
    *f.world.toolhead.homed_axes.lock().unwrap() = "xz".to_string();
    f.py.exec("toolhead.homed = 'xz'").expect("oracle homed");
    f.calibrate("not homed", &[("TARGET", "40.0")]);
    *f.world.toolhead.homed_axes.lock().unwrap() = "xyz".to_string();
    f.py.exec("toolhead.homed = 'xyz'").expect("oracle homed");
    *f.world.probe.name.lock().unwrap() = "probe other".to_string();
    f.py.exec("probe.name = 'probe other'").expect("oracle probe name");
    f.calibrate("unlinked probe", &[("TARGET", "40.0")]);
}

#[test]
fn async_kick_runs_next_command() {
    let Some(mut f) = Flow::start(&flow_extra()) else { return };
    f.temp(10.0, 25.0);
    f.bulk(&flow_schedule(1));
    f.set_bed_z(2.5);
    f.calibrate(
        "calibrate",
        &[("TARGET", "40.0"), ("STEP", "1.0"), ("METHOD", "tap")],
    );
    // The first sample scheduled the next one at 26C. A reading above that
    // asks the reactor to run TEMPERATURE_PROBE_NEXT.
    let reactor = f.world.printer.get_reactor();
    let probe = Arc::clone(&f.probe);
    let r_end = reactor.clone();
    reactor.register_timer(
        move |_| {
            probe.temp_callback(20.0, 26.5);
            let r_stop = r_end.clone();
            r_end.register_timer(
                move |_| {
                    r_stop.end();
                    NEVER
                },
                r_end.monotonic() + 0.3,
            );
            NEVER
        },
        NOW,
    );
    let done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let timed_out = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let (d2, t2, r_watch) = (done.clone(), timed_out.clone(), reactor.clone());
    let watchdog = kos::thread::spawn(move || {
        for _ in 0..200 {
            if d2.load(std::sync::atomic::Ordering::SeqCst) {
                return;
            }
            kos::time::sleep(0.05);
        }
        t2.store(true, std::sync::atomic::Ordering::SeqCst);
        r_watch.finalize();
    });
    reactor.run();
    done.store(true, std::sync::atomic::Ordering::SeqCst);
    watchdog.join();
    assert!(
        !timed_out.load(std::sync::atomic::Ordering::SeqCst),
        "reactor run timed out"
    );
    f.py.eval("sensor.cb(20.0, 26.5)").expect("oracle temp callback");
    f.py.exec("_reactor.run_async()").expect("oracle async callbacks");
    f.compare("kick");
    // The kick reset next_auto_temp, so running the hook again does nothing.
    f.probe.check_kick_next().expect("rust second kick");
    f.py.exec("probe_obj._check_kick_next(0.0)").expect("oracle second kick");
    f.compare("second kick");
}
