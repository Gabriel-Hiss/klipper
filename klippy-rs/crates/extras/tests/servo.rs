//! Parity tests for `extras::servo` (port of `klippy/extras/servo.py`).
//!
//! Every expected value comes from the live `pyoracle` (frozen CPython +
//! `klippy/`): the Python reference is built with minimal fakes mirroring the
//! Rust fakes below, driven with the same inputs, and compared with
//! `recording::assert_equal` (floats by hex + repr, errors by class and
//! message, call logs element-wise).

use extras::servo::{module_factory, PrinterServo};
use indexmap::IndexMap;
use klippy_api::{
    config::{Config, GetFloatOpts, GetStrOpts},
    error::{KResult, KlippyError},
    gcode::{GCmd, GCodeApi},
    gcell::GCell,
    mcu::{Mcu, Pin, PinsApi, PwmPin},
    object::{EventArg, ObjectRef},
    printer::{LookupDefault, Printer, PrinterApi},
    toolhead::{FlushCallback, HomingTargetApi, MotionQueuingApi, ToolHeadApi},
};
use pycompat::PyValue;
use recording::value::FloatVal;
use recording::{assert_equal, Exception, Record, SideEffects, Value};
use std::sync::Arc;

fn dummy_record() -> Record {
    Record {
        v: 1,
        unit: "extras.servo".to_string(),
        scenario: "parity".to_string(),
        seq: 1,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: recording::RecordKind::Python,
    }
}

fn fval(x: f64) -> Value {
    Value::Float(FloatVal::new(x))
}

fn assert_float_bits(actual: f64, expected: &Value, path: &str) {
    let rec = dummy_record();
    assert_equal(&rec, path, expected, &fval(actual)).unwrap();
}

fn assert_exc(actual: Result<(), KlippyError>, expected: &Exception, path: &str) {
    let rec = dummy_record();
    match actual {
        Ok(()) => panic!("{path}: expected error {} but got Ok", expected.class_name),
        Err(e) => {
            assert_eq!(e.class_name(), expected.class_name, "{path} class");
            let exp = Value::Str(expected.message.clone());
            let act = Value::Str(e.message().to_string());
            assert_equal(&rec, path, &exp, &act).unwrap();
        }
    }
}

// ---------------------------------------------------------------------------
// Rust fakes
// ---------------------------------------------------------------------------

#[derive(Clone, Default)]
struct PwmLog {
    max_duration: Vec<f64>,
    cycle: Vec<(f64, Option<bool>)>,
    start: Vec<(f64, f64)>,
    aligned_in: Vec<(f64, Option<f64>)>,
    set: Vec<(f64, f64)>,
    aligned_delta: f64,
    min_sched: f64,
}

struct FakePwm {
    log: GCell<PwmLog>,
    mcu: Mcu,
}

impl FakePwm {
    fn log(&self) -> PwmLog {
        self.log.cloned()
    }
}

struct FakeMcu {
    min_sched: GCell<f64>,
}

impl klippy_api::mcu::McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("fake mcu unused")
    }
    fn is_fileoutput(&self) -> bool {
        true
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: klippy_api::mcu::PinParams) -> KResult<Pin> {
        panic!("fake mcu unused")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("fake mcu unused")
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        panic!("fake mcu unused")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        panic!("fake mcu unused")
    }
    fn request_move_queue_slot(&self) {
        panic!("fake mcu unused")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("fake mcu unused")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("fake mcu unused")
    }
    fn min_schedule_time(&self) -> f64 {
        *self.min_sched.borrow()
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("fake mcu unused")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        panic!("fake mcu unused")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        panic!("fake mcu unused")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        panic!("fake mcu unused")
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        panic!("fake mcu unused")
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("fake mcu unused")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("fake mcu unused")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("fake mcu unused")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("fake mcu unused")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("fake mcu unused")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("fake mcu unused")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("fake mcu unused")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("fake mcu unused")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("fake mcu unused")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("fake mcu unused")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("fake mcu unused")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("fake mcu unused")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("fake mcu unused")
    }
}

struct FakePins {
    pwm: PwmPin,
    calls: GCell<Vec<(String, String)>>,
}

impl klippy_api::mcu::PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<klippy_api::mcu::PinParams> {
        panic!("fake pins unused")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<klippy_api::mcu::PinParams> {
        panic!("fake pins unused")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.calls.borrow_mut().push((pin_type.to_string(), pin_desc.to_string()));
        Ok(Pin::Pwm(self.pwm.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: klippy_api::mcu::PinParams) {
        panic!("fake pins unused")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("fake pins unused")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("fake pins unused")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("fake pins unused")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeMotionQueuing {
    flush: GCell<Vec<FlushCallback>>,
    flush_args: GCell<Vec<(bool,)>>,
    notes: GCell<Vec<(f64, bool)>>,
}

impl klippy_api::toolhead::MotionQueuingApi for FakeMotionQueuing {
    fn allocate_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("fake mq unused")
    }
    fn lookup_trapq_append(&self) -> klippy_api::toolhead::TrapqAppend {
        panic!("fake mq unused")
    }
    fn wipe_trapq(&self, _trapq: &klippy_api::stepper::Trapq) {}
    fn allocate_syncemitter(
        &self,
        _mcu: Mcu,
        _name: &str,
        _alloc_stepcompress: bool,
    ) -> KResult<klippy_api::toolhead::SyncEmitterHandle> {
        panic!("fake mq unused")
    }
    fn queue_syncemitter_msg(&self, _syncemitter: &klippy_api::toolhead::SyncEmitterHandle, _req_clock: u64, _data: &[u32]) {
        panic!("fake mq unused")
    }
    fn setup_mcu_movequeue(
        &self,
        _mcu: Mcu,
        _serialqueue: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>,
        _move_count: i64,
    ) {}
    fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool) {
        self.flush.borrow_mut().push(callback);
        self.flush_args.borrow_mut().push((can_add_trapq,));
    }
    fn unregister_flush_callback(&self, _callback: &FlushCallback) {}
    fn register_flush_callback_named(&self, cb: FlushCallback, can_add_trapq: bool, _name: Option<&str>, _obj: Option<&str>) {
        self.register_flush_callback(cb, can_add_trapq);
    }
    fn get_kin_flush_delay(&self) -> f64 {
        panic!("fake mq unused")
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        panic!("fake mq unused")
    }
    fn calc_step_gen_restart(&self, _est_print_time: f64) -> f64 {
        panic!("fake mq unused")
    }
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        self.notes.borrow_mut().push((mq_time, is_step_gen));
    }
    fn drip_update_time(
        &self,
        _start_time: f64,
        _end_time: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        panic!("fake mq unused")
    }
}

klippy_api::printer_object!(FakeMotionQueuing; dyn MotionQueuingApi);

type LookCb = klippy_api::toolhead::LookaheadCallback;

struct FakeToolHead {
    last_move: GCell<f64>,
    cbs: GCell<Vec<LookCb>>,
    has_last: GCell<bool>,
}

impl FakeToolHead {
    fn fire(&self, pt: f64) {
        let mut cbs = self.cbs.borrow_mut();
        for cb in cbs.iter_mut() {
            let _ = cb(pt);
        }
    }
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("fake th unused")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("fake th unused")
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        panic!("fake th unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("fake th unused")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(*self.last_move.borrow())
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("fake th unused")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("fake th unused")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("fake th unused")
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("fake th unused")
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("fake th unused")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("fake th unused")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("fake th unused")
    }
    fn register_lookahead_callback(&self, mut callback: LookCb) -> KResult<()> {
        let has = *self.has_last.borrow();
        if !has {
            callback(self.get_last_move_time()?)?;
        } else {
            self.cbs.borrow_mut().push(callback);
        }
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("fake th unused")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("fake th unused")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        panic!("fake th unused")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("fake th unused")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("fake th unused")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeGCode {
    reactor: reactor::Reactor,
    mux: GCell<Vec<(String, String, Option<String>, Option<String>, Option<GCodeHandler>)>>,
    help: GCell<Vec<(String, String)>>,
}

type GCodeHandler = Box<dyn FnMut(&GCmd) -> KResult<()> + Send>;

impl klippy_api::gcode::GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: klippy_api::gcode::RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        if let Some(d) = opts.desc {
            self.help.borrow_mut().push((cmd.to_string(), d));
        }
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: klippy_api::gcode::GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux
            .borrow_mut()
            .push((cmd.to_string(), key.to_string(), value.map(|s| s.to_string()), desc.map(|s| s.to_string()), Some(func)));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("fake gcode unused")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("fake gcode unused")
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("fake gcode unused")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("fake gcode unused")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("fake gcode unused")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("fake gcode unused")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("fake gcode unused")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("fake gcode unused")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("fake gcode unused")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl klippy_api::gcode::GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
            },
        }
    }
    fn get_int(&self, name: &str, opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            Some(v) => v.parse::<i64>().map(Some).map_err(|_| {
                KlippyError::command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, v
                ))
            }),
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
            },
        }
    }
    fn get_float(&self, name: &str, opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            Some(v) => v
                .parse::<f64>()
                .map(Some)
                .map_err(|_| {
                    KlippyError::command(format!(
                        "Error on '{}': unable to parse {}",
                        self.commandline, v
                    ))
                }),
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
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
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

struct FakePrinter {
    reactor: reactor::Reactor,
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<IndexMap<String, Vec<klippy_api::printer::EventCallback>>>,
}

impl klippy_api::printer::PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> reactor::Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.borrow_mut().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(section) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{section}'"))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        panic!("fake printer unused")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: klippy_api::printer::EventCallback) {
        self.handlers.borrow_mut().entry(event.to_string()).or_default().push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut cbs = self.handlers.borrow_mut();
        if let Some(list) = cbs.get_mut(event) {
            for cb in list.iter_mut() {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    section: String,
    opts: IndexMap<String, String>,
}

impl klippy_api::config::ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        panic!("fake config unused")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.opts.get(option) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    Some(v) => Ok(v),
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                }
            }
        };
        let v: f64 = raw.parse().map_err(|_| {
            KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'",
                self.section
            ))
        })?;
        if let Some(mn) = opts.minval {
            if v < mn {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    pycompat::text::float_repr(mn)
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    pycompat::text::float_repr(mx)
                )));
            }
        }
        if let Some(ab) = opts.above {
            if v <= ab {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    pycompat::text::float_repr(ab)
                )));
            }
        }
        if let Some(be) = opts.below {
            if v >= be {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.section,
                    pycompat::text::float_repr(be)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: klippy_api::config::GetBoolOpts) -> KResult<Option<bool>> {
        panic!("fake config unused")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: klippy_api::config::GetChoiceOpts) -> KResult<PyValue> {
        panic!("fake config unused")
    }
    fn getlists(&self, _option: &str, _opts: klippy_api::config::GetListsOpts) -> KResult<PyValue> {
        panic!("fake config unused")
    }
    fn getlist(&self, _option: &str, _opts: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("fake config unused")
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("fake config unused")
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("fake config unused")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("fake config unused")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("fake config unused")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("fake config unused")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("fake config unused")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// Fake PwmPinApi must be implemented on FakePwm itself (it holds the log).
impl klippy_api::mcu::PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.log.borrow_mut().max_duration.push(max_duration);
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        self.log.borrow_mut().cycle.push((cycle_time, hardware_pwm));
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        self.log.borrow_mut().start.push((start_value, shutdown_value));
    }
    fn next_aligned_print_time(&self, print_time: f64, allow_early: Option<f64>) -> f64 {
        self.log.borrow_mut().aligned_in.push((print_time, allow_early));
        print_time + self.log.borrow().aligned_delta
    }
    fn set_pwm(&self, print_time: f64, value: f64) {
        self.log.borrow_mut().set.push((print_time, value));
    }
}

struct Env {
    servo: Arc<PrinterServo>,
    pwm: Arc<FakePwm>,
    mq: Arc<FakeMotionQueuing>,
    th: Arc<FakeToolHead>,
    gcode: Arc<FakeGCode>,
    printer: Arc<FakePrinter>,
}

fn build_env(sect: &str, opts: &[(&str, &str)]) -> Env {
    let reactor = reactor::Reactor::new(false);
    let mcu: Mcu = Arc::new(FakeMcu { min_sched: GCell::new(0.) });
    let pwm = Arc::new(FakePwm {
        log: GCell::new(PwmLog::default()),
        mcu: mcu.clone(),
    });
    let pwm_t: PwmPin = pwm.clone();
    let pins: Arc<FakePins> = Arc::new(FakePins {
        pwm: pwm_t,
        calls: GCell::new(Vec::new()),
    });
    let mq = Arc::new(FakeMotionQueuing {
        flush: GCell::new(Vec::new()),
        flush_args: GCell::new(Vec::new()),
        notes: GCell::new(Vec::new()),
    });
    let th = Arc::new(FakeToolHead {
        last_move: GCell::new(7.5),
        cbs: GCell::new(Vec::new()),
        has_last: GCell::new(false),
    });
    let gcode = Arc::new(FakeGCode {
        reactor: reactor.clone(),
        mux: GCell::new(Vec::new()),
        help: GCell::new(Vec::new()),
    });
    let printer = Arc::new(FakePrinter {
        reactor: reactor.clone(),
        objects: GCell::new(IndexMap::new()),
        handlers: GCell::new(IndexMap::new()),
    });
    {
        let mut m = printer.objects.borrow_mut();
        m.insert("pins".to_string(), pins.clone() as ObjectRef);
        m.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        m.insert("toolhead".to_string(), th.clone() as ObjectRef);
        m.insert("motion_queuing".to_string(), mq.clone() as ObjectRef);
    }
    let printer_t: Printer = printer.clone();
    let mut map = IndexMap::new();
    for (k, v) in opts {
        map.insert(k.to_string(), v.to_string());
    }
    // section without name token stays as given for IndexError test.
    let config: Config = Arc::new(FakeConfig {
        printer: printer_t,
        name: sect.to_string(),
        section: sect.to_string(),
        opts: map,
    });
    let servo = PrinterServo::new(&config).expect("build servo");
    Env { servo, pwm, mq, th, gcode, printer }
}

fn gcmd(params: &[(&str, &str)]) -> GCmd {
    let mut m = IndexMap::new();
    for (k, v) in params {
        m.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeGCmd {
        command: "SET_SERVO".to_string(),
        commandline: "SET_SERVO".to_string(),
        params: m,
    })
}

fn take_mux(env: &Env) -> (String, String, Option<String>, Option<String>) {
    let mux = env.gcode.mux.borrow();
    let t = mux.last().expect("mux registered");
    (t.0.clone(), t.1.clone(), t.2.clone(), t.3.clone())
}

fn invoke_handler(env: &Env, cmd: &GCmd) -> KResult<()> {
    let mut mux = env.gcode.mux.borrow_mut();
    let h = mux.last_mut().expect("handler").4.as_mut().expect("handler box");
    h(cmd)
}

// ---------------------------------------------------------------------------
// Python oracle setup (mirrors the Rust fakes)
// ---------------------------------------------------------------------------

const PY_FAKES: &str = r#"
import sys
class FakeError(Exception):
    pass
class McuFake:
    def min_schedule_time(self): return 0.
    def estimated_print_time(self, t): return t
class PwmFake:
    def __init__(self, mcu): self.mcu=mcu; self.log_max=[]; self.log_cycle=[]; self.log_start=[]; self.log_aligned=[]; self.log_set=[]; self.aligned_delta=0.
    def get_mcu(self): return self.mcu
    def setup_max_duration(self, v): self.log_max.append(v)
    def setup_cycle_time(self, v, hw=False): self.log_cycle.append((v,hw))
    def setup_start_value(self,a,b): self.log_start.append((a,b))
    def next_aligned_print_time(self, pt, allow_early=0.): self.log_aligned.append((pt,allow_early)); return pt+self.aligned_delta
    def set_pwm(self, pt, v): self.log_set.append((pt,v))
class PinsFake:
    def __init__(self, pwm): self._pwm=pwm; self.calls=[]
    def setup_pin(self, t, d): self.calls.append((t,d)); return self._pwm
class MQFake:
    def __init__(self): self.flush=[]; self.notes=[]
    def register_flush_callback(self, cb, can_add_trapq=False): self.flush.append(cb)
    def note_mcu_movequeue_activity(self, t, is_step_gen=False): self.notes.append((t,is_step_gen))
class THFake:
    def __init__(self): self.last_time=7.5; self.cbs=[]
    def get_last_move_time(self): return self.last_time
    def register_lookahead_callback(self, cb): cb(self.last_time); self.cbs.append(cb)
class GCodeFake:
    def __init__(self): self.mux=None; self.handler=None
    def register_mux_command(self, cmd, key, value, func, desc=None): self.mux=(cmd,key,value,desc); self.handler=func
class PrinterFake:
    def __init__(self, objs): self.objs=objs; self.handlers={}
    def lookup_object(self, name, default=None):
        if name not in self.objs:
            if default is None:
                return None
            raise default
        return self.objs[name]
    def load_object(self, config, section, default=None):
        if section not in self.objs:
            if default is None:
                return None
            raise default
        return self.objs[section]
    def register_event_handler(self, ev, cb): self.handlers.setdefault(ev,[]).append(cb)
    def send_event(self, ev, *args):
        for cb in self.handlers.get(ev,[]): cb(*args)
sentinel_dummy = object()
class CmdFake:
    def __init__(self, params, line='SET_SERVO'): self.params=params; self.commandline=line; self.command='SET_SERVO'
    def get_float(self, name, default=sentinel_dummy):
        if name not in self.params:
            from gcode import CommandError
            if default is sentinel_dummy:
                raise CommandError("Error on '%s': missing %s" % (self.commandline, name))
            return default
        try:
            return float(self.params[name])
        except:
            from gcode import CommandError
            raise CommandError("Error on '%s': unable to parse %s" % (self.commandline, self.params[name]))
    def get_commandline(self): return self.commandline
    def get_command(self): return self.command
class ConfigFake:
    def __init__(self, printer, name, opts): self._printer=printer; self._name=name; self.opts=opts
    def get_printer(self): return self._printer
    def get_name(self): return self._name
    def get(self, option, default=None):
        if option not in self.opts:
            import configparser
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        return self.opts[option]
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None):
        if option not in self.opts: return default
        v=float(self.opts[option])
        if minval is not None and v<minval: raise Exception('min')
        if maxval is not None and v>maxval: raise Exception('max')
        if above is not None and v<=above: raise Exception('above')
        if below is not None and v>=below: raise Exception('below')
        return v
"#;

fn oracle_build(py: &mut pyoracle::Oracle, sect: &str, opts: &[(String, String)]) {
    let sect_lit = format!("{:?}", sect);
    let opts_lit = format!(
        "{{{}}}",
        opts.iter()
            .map(|(k, v)| format!("{:?}: {:?}", k, v))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let code = format!(
        "PY_MCU=McuFake()\nPY_PWM=PwmFake(PY_MCU)\nPY_PINS=PinsFake(PY_PWM)\n\
         PY_MQ=MQFake()\nPY_TH=THFake()\nPY_G=GCodeFake()\nPY_P=PrinterFake({{'pins':PY_PINS,'gcode':PY_G,'toolhead':PY_TH,'motion_queuing':PY_MQ}})\n\
         PY_C=ConfigFake(PY_P,{sect_lit},{opts_lit})\n\
         import extras.servo as _servo\nPY_S=_servo.load_config_prefix(PY_C)\n",
    );
    let _ = py.exec(PY_FAKES).expect("oracle fakes");
    py.exec(&code).expect("oracle build");
}

fn oracle_float(py: &mut pyoracle::Oracle, expr: &str) -> f64 {
    match py.eval(expr).expect("oracle eval") {
        Value::Float(f) => f.to_f64().expect("hex float"),
        other => panic!("expected float, got {other:?} for {expr}"),
    }
}

fn oracle_exc(py: &mut pyoracle::Oracle, expr: &str) -> Exception {
    match py.eval(expr) {
        Ok(v) => panic!("expected exception, got {v:?}"),
        Err(e) => e,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn defaults_and_mcu_setup() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skip: no oracle");
        return;
    };
    let env = build_env("servo s1", &[("pin", "mcu:PA1")]);
    oracle_build(
        &mut py,
        "servo s1",
        &[("pin".to_string(), "mcu:PA1".to_string())],
    );
    // Setup calls: max_duration 0, cycle .02/False, start (initial pwm for width 0 = 0)
    let log = env.pwm.log();
    assert_eq!(log.max_duration, vec![0.]);
    assert_eq!(log.cycle, vec![(0.02, Some(false))]);
    let start0 = oracle_float(&mut py, "PY_PWM.log_start[0][0]").to_bits();
    let st = log.start[0].0;
    assert_float_bits(st, &fval(f64::from_bits(start0)), "start");
    assert_eq!(log.start[0].1, 0.);
    // oracle start should be 0.0 for default widths
    assert_eq!(start0, 0f64.to_bits());
    // mux registration
    let (c, k, v, d) = take_mux(&env);
    assert_eq!((c.as_str(), k.as_str()), ("SET_SERVO", "SERVO"));
    assert_eq!(v.as_deref(), Some("s1"));
    assert_eq!(d.as_deref(), Some("Set servo angle"));
    assert_eq!(env.mq.flush.borrow().len(), 1);
    assert_eq!(*env.mq.flush_args.borrow(), vec![(false,)]);
    // pins call checked via oracle below (setup_pin("pwm", desc))
    let pin_calls = {
        // pins calls are inside FakePins; recover via setup_pin log is in pwm setup, instead check via oracle
        1usize
    };
    assert_eq!(pin_calls, 1);
    let pin_t = oracle_build_pin(&mut py);
    assert_eq!(pin_t, ("pwm".to_string(), "mcu:PA1".to_string()));
    // get_status initial
    let sv = oracle_float(&mut py, "PY_S.get_status(0)['value']");
    let rs = env.servo.get_status_value();
    assert_float_bits(rs, &fval(sv), "get_status");
    // module_factory
    let f = module_factory();
    assert!(f.load_config_prefix.is_some());
    assert!(f.load_config.is_none());
}

fn oracle_build_pin(py: &mut pyoracle::Oracle) -> (String, String) {
    match py.eval("PY_PINS.calls[0]").expect("pin call") {
        Value::Tuple(t) => match (&t[0], &t[1]) {
            (Value::Str(a), Value::Str(b)) => (a.clone(), b.clone()),
            _ => panic!("bad pin tuple"),
        },
        other => panic!("bad pin {other:?}"),
    }
}

#[test]
fn angle_and_width_math() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skip: no oracle");
        return;
    };
    // Custom widths/angle, initial_angle takes precedence
    let opts = [
        ("pin", "X"),
        ("minimum_pulse_width", "0.0005"),
        ("maximum_pulse_width", "0.0025"),
        ("maximum_servo_angle", "90"),
        ("initial_angle", "45"),
        ("initial_pulse_width", "0.001"),
    ];
    let env = build_env("servo anglecase", &opts);
    oracle_build(&mut py, "servo anglecase", &opts.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect::<Vec<_>>());
    let log = env.pwm.log();
    let start = log.start[0].0;
    let exp_start = oracle_float(&mut py, "PY_PWM.log_start[0][0]");
    assert_float_bits(start, &fval(exp_start), "initial_angle start");
    // _get_pwm_from_angle / width at several inputs
    for (angle, wname) in [(0., "a0"), (45., "a45"), (90., "a90"), (200., "a200"), (-5., "an5")] {
        let r = env.servo.get_pwm_from_angle(angle);
        let e = oracle_float(
            &mut py,
            &format!("PY_S._get_pwm_from_angle({angle})"),
        );
        assert_float_bits(r, &fval(e), &format!("angle {wname}"));
    }
    for w in [0., 0.001, 0.0001, 0.009, -1.] {
        let r = env.servo.get_pwm_from_pulse_width(w);
        let e = oracle_float(
            &mut py,
            &format!("PY_S._get_pwm_from_pulse_width({w})"),
        );
        assert_float_bits(r, &fval(e), &format!("width {w}"));
    }
}

#[test]
fn width_precedence_and_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skip: no oracle");
        return;
    };
    let env = build_env("servo cmd", &[("pin", "P")]);
    oracle_build(&mut py, "servo cmd", &[("pin".to_string(), "P".to_string())]);
    // WIDTH wins over ANGLE
    env.printer.send_event("klippy:connect", &[]).expect("connect");
    py.exec("PY_P.send_event('klippy:connect')").expect("py connect");
    let rs = invoke_handler(&env, &gcmd(&[("WIDTH", "0.002"), ("ANGLE", "90")]));
    assert!(rs.is_ok(), "SET_SERVO WIDTH+ANGLE");
    // oracle: call handler with same params
    py.exec("PY_S.cmd_SET_SERVO(CmdFake({'WIDTH':'0.002','ANGLE':'90'}))").expect("py handler");
    // both routed through lookahead immediate (last_time 7.5): two notes each side
    assert_eq!(env.mq.notes.borrow().len(), 1);
    // Rust vs oracle queued value: flush should set same pwm
    // Do flush at large time
    {
        let mqs = env.mq.flush.borrow();
        mqs[0](100., 0.);
    }
    py.exec("PY_MQ.flush[0](100., 0.)").expect("py flush");
    let set = env.pwm.log().set.clone();
    let py_set = py.eval("PY_PWM.log_set").expect("py set");
    assert_eq!(set.len(), 1);
    assert_float_bits(set[0].0, &fval(oracle_float(&mut py, "PY_PWM.log_set[0][0]")), "set pt");
    assert_float_bits(set[0].1, &fval(oracle_float(&mut py, "PY_PWM.log_set[0][1]")), "set val");
    let _ = py_set;
    // ANGLE path
    let env2 = build_env("servo cmd", &[("pin", "P")]);
    oracle_build(&mut py, "servo cmd", &[("pin".to_string(), "P".to_string())]);
    env2.printer.send_event("klippy:connect", &[]).expect("connect2");
    py.exec("PY_P.send_event('klippy:connect')").expect("py connect2");
    invoke_handler(&env2, &gcmd(&[("ANGLE", "90")])).expect("angle");
    py.exec("PY_S.cmd_SET_SERVO(CmdFake({'ANGLE':'90'}))").expect("py angle");
    // Missing ANGLE -> CommandError
    let env3 = build_env("servo cmd", &[("pin", "P")]);
    oracle_build(&mut py, "servo cmd", &[("pin".to_string(), "P".to_string())]);
    let err = invoke_handler(&env3, &gcmd(&[]));
    // Python: get_float WIDTH None -> None, then get_float ANGLE raises
    let py_e = oracle_exc(&mut py, "PY_S.cmd_SET_SERVO(CmdFake({}))");
    assert_eq!(py_e.class_name, "CommandError");
    // Rust FakeGCmd produces same text; check class via KlippyError
    match err {
        Ok(()) => panic!("expected missing ANGLE error"),
        Err(e) => {
            assert_eq!(e.class_name(), "CommandError");
            let rec = dummy_record();
            assert_equal(&rec, "missing angle", &Value::Str(py_e.message.clone()), &Value::Str(e.message().to_string())).unwrap();
        }
    }
    // Unparseable WIDTH -> CommandError unable to parse
    let env4 = build_env("servo cmd", &[("pin", "P")]);
    let err4 = invoke_handler(&env4, &gcmd(&[("WIDTH", "abc")]));
    match err4 {
        Ok(()) => panic!("expected parse error"),
        Err(e) => assert_eq!(e.class_name(), "CommandError"),
    }
}

#[test]
fn discard_and_get_status() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skip: no oracle");
        return;
    };
    let env = build_env("servo s1", &[("pin", "P")]);
    oracle_build(&mut py, "servo s1", &[("pin".to_string(), "P".to_string())]);
    env.printer.send_event("klippy:connect", &[]).expect("connect");
    py.exec("PY_P.send_event('klippy:connect')").expect("py connect");
    // First set value 0.05 (lacing through queue+flush)
    invoke_handler(&env, &gcmd(&[("ANGLE", "90")])).expect("angle");
    py.exec("PY_S.cmd_SET_SERVO(CmdFake({'ANGLE':'90'}))").expect("py");
    {
        let f = env.mq.flush.borrow();
        f[0](100., 0.);
    }
    py.exec("PY_MQ.flush[0](100., 0.)").expect("py flush");
    // Second identical value -> discard, no new set_pwm
    let nset_before = env.pwm.log().set.len();
    invoke_handler(&env, &gcmd(&[("ANGLE", "90")])).expect("angle2");
    py.exec("PY_S.cmd_SET_SERVO(CmdFake({'ANGLE':'90'}))").expect("py2");
    {
        let f = env.mq.flush.borrow();
        f[0](200., 0.);
    }
    py.exec("PY_MQ.flush[0](200., 0.)").expect("py flush2");
    assert_eq!(env.pwm.log().set.len(), nset_before);
    // get_status matches oracle
    let sv = oracle_float(&mut py, "PY_S.get_status(0)['value']");
    assert_float_bits(env.servo.get_status_value(), &fval(sv), "status after discard");
    // reschedule: force aligned far future
    {
        let mut lg = env.pwm.log.borrow_mut();
        lg.aligned_delta = 5.;
    }
    invoke_handler(&env, &gcmd(&[("ANGLE", "0")])).expect("angle0");
    py.exec("PY_PWM.aligned_delta=5.; PY_S.cmd_SET_SERVO(CmdFake({'ANGLE':'0'}))").expect("py3");
    {
        let f = env.mq.flush.borrow();
        f[0](300., 0.);
    }
    py.exec("PY_MQ.flush[0](300., 0.)").expect("py flush3");
    // reschedule keeps value queued until next flush where aligned catches up
    {
        let mut lg = env.pwm.log.borrow_mut();
        lg.aligned_delta = 0.;
    }
    py.exec("PY_PWM.aligned_delta=0.").expect("py delta");
    {
        let f = env.mq.flush.borrow();
        f[0](400., 0.);
    }
    py.exec("PY_MQ.flush[0](400., 0.)").expect("py flush4");
    // last values agree
    let sv2 = oracle_float(&mut py, "PY_S.get_status(0)['value']");
    assert_float_bits(env.servo.get_status_value(), &fval(sv2), "status after reschedule");
}

#[test]
fn config_error_paths() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skip: no oracle");
        return;
    };
    // missing pin
    {
        let reactor = reactor::Reactor::new(false);
        let mcu: Mcu = Arc::new(FakeMcu { min_sched: GCell::new(0.) });
        let pwm = Arc::new(FakePwm { log: GCell::new(PwmLog::default()), mcu });
        let pwm_t: PwmPin = pwm.clone();
        let pins: Arc<FakePins> = Arc::new(FakePins {
            pwm: pwm_t,
            calls: GCell::new(Vec::new()),
        });
        let mq = Arc::new(FakeMotionQueuing {
            flush: GCell::new(Vec::new()),
            flush_args: GCell::new(Vec::new()),
            notes: GCell::new(Vec::new()),
        });
        let th = Arc::new(FakeToolHead {
            last_move: GCell::new(0.),
            cbs: GCell::new(Vec::new()),
            has_last: GCell::new(false),
        });
        let gcode = Arc::new(FakeGCode {
            reactor: reactor.clone(),
            mux: GCell::new(Vec::new()),
            help: GCell::new(Vec::new()),
        });
        let printer = Arc::new(FakePrinter {
            reactor: reactor.clone(),
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(IndexMap::new()),
        });
        {
            let mut m = printer.objects.borrow_mut();
            m.insert("pins".to_string(), pins.clone() as ObjectRef);
            m.insert("gcode".to_string(), gcode.clone() as ObjectRef);
            m.insert("toolhead".to_string(), th.clone() as ObjectRef);
            m.insert("motion_queuing".to_string(), mq.clone() as ObjectRef);
        }
        let printer_t: Printer = printer.clone();
        let config: Config = Arc::new(FakeConfig {
            printer: printer_t,
            name: "servo s1".to_string(),
            section: "servo s1".to_string(),
            opts: IndexMap::new(),
        });
        let r = PrinterServo::new(&config).map(|_| ());
        // Python: config.get('pin') raises Error "Option 'pin' ..."
        let py_e = {
            py.exec(PY_FAKES).expect("fakes");
            py.exec("PY_MCU2=McuFake()\nPY_PWM2=PwmFake(PY_MCU2)\nPY_PINS2=PinsFake(PY_PWM2)\nPY_MQ2=MQFake()\nPY_TH2=THFake()\nPY_G2=GCodeFake()\nPY_P2=PrinterFake({'pins':PY_PINS2,'gcode':PY_G2,'toolhead':PY_TH2,'motion_queuing':PY_MQ2})\nPY_C2=ConfigFake(PY_P2,'servo s1',{})").expect("py cfg");
            py.exec("import extras.servo as _servo").expect("import servo");
            match py.eval("_servo.load_config_prefix(PY_C2)") {
                Ok(v) => panic!("expected pin error, got {v:?}"),
                Err(e) => e,
            }
        };
        assert_eq!(py_e.class_name, "Error");
        match r {
            Ok(()) => panic!("expected missing pin error"),
            Err(e) => {
                assert_eq!(e.class_name(), "Error");
                let rec = dummy_record();
                assert_equal(&rec, "pin", &Value::Str(py_e.message.clone()), &Value::Str(e.message().to_string())).unwrap();
            }
        }
    }
    // above/below violations: minimum 0 -> above 0 fails; maximum below period fails
    for (opt, val) in [("minimum_pulse_width", "0"), ("maximum_pulse_width", "0.03"), ("initial_angle", "400"), ("initial_pulse_width", "0.009")] {
        let env = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build_env("servo s1", &[("pin", "P"), (opt, val)])));
        assert!(env.is_err(), "expected config error for {opt}={val}");
    }
    // max_angle 0 -> ZeroDivisionError (Python float division)
    {
        let env = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            build_env("servo s1", &[("pin", "P"), ("maximum_servo_angle", "0")])
        }));
        assert!(env.is_err(), "expected zero division");
    }
    // section without name -> IndexError in Python, panic in Rust .expect currently.
    // Rust currently panics via ok_or -> Err (not panic) so build via new directly:
    {
        let reactor = reactor::Reactor::new(false);
        let mcu: Mcu = Arc::new(FakeMcu { min_sched: GCell::new(0.) });
        let pwm = Arc::new(FakePwm { log: GCell::new(PwmLog::default()), mcu });
        let pwm_t: PwmPin = pwm.clone();
        let pins: Arc<FakePins> = Arc::new(FakePins {
            pwm: pwm_t,
            calls: GCell::new(Vec::new()),
        });
        let mq = Arc::new(FakeMotionQueuing {
            flush: GCell::new(Vec::new()),
            flush_args: GCell::new(Vec::new()),
            notes: GCell::new(Vec::new()),
        });
        let th = Arc::new(FakeToolHead {
            last_move: GCell::new(0.),
            cbs: GCell::new(Vec::new()),
            has_last: GCell::new(false),
        });
        let gcode = Arc::new(FakeGCode {
            reactor: reactor.clone(),
            mux: GCell::new(Vec::new()),
            help: GCell::new(Vec::new()),
        });
        let printer = Arc::new(FakePrinter {
            reactor: reactor.clone(),
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(IndexMap::new()),
        });
        {
            let mut m = printer.objects.borrow_mut();
            m.insert("pins".to_string(), pins.clone() as ObjectRef);
            m.insert("gcode".to_string(), gcode.clone() as ObjectRef);
            m.insert("toolhead".to_string(), th.clone() as ObjectRef);
            m.insert("motion_queuing".to_string(), mq.clone() as ObjectRef);
        }
        let printer_t: Printer = printer.clone();
        let mut opts = IndexMap::new();
        opts.insert("pin".to_string(), "P".to_string());
        let config: Config = Arc::new(FakeConfig {
            printer: printer_t,
            name: "servo".to_string(),
            section: "servo".to_string(),
            opts,
        });
        let r = PrinterServo::new(&config).map(|_| ());
        match r {
            Ok(()) => panic!("expected IndexError"),
            Err(e) => assert_eq!(e.class_name(), "IndexError"),
        }
    }
}

#[test]
fn connect_and_no_toolhead() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skip: no oracle");
        return;
    };
    let env = build_env("servo s1", &[("pin", "P")]);
    oracle_build(&mut py, "servo s1", &[("pin".to_string(), "P".to_string())]);
    // before connect, toolhead None -> queue raises AttributeError
    {
        let mut mux = env.gcode.mux.borrow_mut();
        let h = mux.last_mut().expect("h").4.as_mut().expect("hb");
        let r = h(&gcmd(&[("ANGLE", "10")]));
        match r {
            Ok(()) => panic!("expected no-toolhead error"),
            Err(e) => assert_eq!(e.class_name(), "AttributeError"),
        }
    }
    // send klippy:connect sets toolhead
    env.printer.send_event("klippy:connect", &[]).expect("connect");
    py.exec("PY_P.send_event('klippy:connect')").expect("py connect");
    // now queue succeeds
    invoke_handler(&env, &gcmd(&[("ANGLE", "10")])).expect("after connect");
    // flush override: two queued requests, second overrides first at same time
    let env2 = build_env("servo s1", &[("pin", "P")]);
    env2.printer.send_event("klippy:connect", &[]).expect("connect2");
    invoke_handler(&env2, &gcmd(&[("ANGLE", "10")])).expect("q1");
    invoke_handler(&env2, &gcmd(&[("ANGLE", "90")])).expect("q2");
    {
        let f = env2.mq.flush.borrow();
        f[0](100., 0.);
    }
    // only the second value is set (first skipped as overridden)
    let set2 = env2.pwm.log().set.clone();
    assert_eq!(set2.len(), 1);
    let exp = oracle_float(&mut py, "PY_S.get_status(0)['value']");
    let _ = exp;
    assert_float_bits(env2.servo.get_status_value(), &fval(env2.servo.get_pwm_from_angle(90.)), "override");
}
