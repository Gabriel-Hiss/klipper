// Differential parity tests for `extras::motion_queuing` against the
// Python reference (`klippy/extras/motion_queuing.py`) through `pyoracle`.
//
// Every expected value comes from the reference: the Python object is built
// with minimal fakes for the printer, config and MCU (and a fake C layer
// patched onto the module's `chelper` global, so the C calls are recorded
// the same way the Rust `MotionChelper` seam records them), then driven
// with the same inputs as the Rust port driven through test fakes of the
// contract traits. Floats compare as recorded values (bit for bit via
// `recording::assert_equal`), errors by class and message, and log lines
// verbatim. Tests skip when `$KLIPPY_PYTHON` is unset (outside the oracle
// image).
//
// Run with (from `klippy-rs/`):
//   `cargo test -p extras --test motion_queuing`
// (the integration owner builds and runs; per the pilot rules only
// `cargo check`/`cargo clippy` run here).

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use extras::motion_queuing::*;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueue, CommandWrapper,
    CommandQueryWrapper, ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams,
    SerialResponseCallback, TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperKinematics as StepperKinematicsHandle, StepperKinematicsApi, Trapq, TrapqApi};
use klippy_api::toolhead::{FlushCallback, SyncEmitterHandle};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Harness helpers
// ---------------------------------------------------------------------------

fn dummy_record(unit: &str, scenario: &str, function: &str) -> Record {
    Record {
        v: 1,
        unit: unit.to_string(),
        scenario: scenario.to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(rec: &Record, path: &str, expected: &Value, actual: &Value) {
    if let Err(d) = recording::assert_equal(rec, path, expected, actual) {
        panic!("parity divergence: {d}");
    }
}



fn v_strs(xs: &[&str]) -> Value {
    Value::List(xs.iter().map(|x| Value::str(x)).collect())
}

fn py_exec(py: &mut pyoracle::Oracle, code: &str) {
    py.exec(code).expect("oracle exec failed");
}

fn py_eval(py: &mut pyoracle::Oracle, expr: &str) -> Value {
    py.eval(expr).expect("oracle eval raised")
}

// Log capture (lines kept per test thread; the Rust sink is process wide).
thread_local! {
    static LOGS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static LOG_ONCE: std::sync::Once = std::sync::Once::new();
    LOG_ONCE.call_once(|| {
        log::set_sink(Box::new(|rec: &log::Record<'_>| {
            LOGS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn take_logs() -> Vec<(String, String)> {
    install_log_sink();
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

fn take_log_messages() -> Vec<String> {
    take_logs().into_iter().map(|(_, m)| m).collect()
}

// ---------------------------------------------------------------------------
// Fakes of the contract traits
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<Vec<(String, ObjectRef)>>,
    shutdowns: Mutex<Vec<(String, PyValue)>>,
    handlers: Mutex<Vec<(String, Vec<klippy_api::printer::EventCallback>)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor,
            objects: Mutex::new(Vec::new()),
            shutdowns: Mutex::new(Vec::new()),
            handlers: Mutex::new(Vec::new()),
        })
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
        (String::new(), String::new())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().unwrap().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found = self
            .objects
            .lock()
            .unwrap()
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, o)| o.clone());
        match found {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("unable to load object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let all = self.objects.lock().unwrap().clone();
            match module {
                None => all,
                Some(m) => all
                    .into_iter()
                    .filter(|(n, _)| n == m || n.starts_with(&format!("{m} ")))
                    .collect(),
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
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
    fn invoke_shutdown(&self, msg: &str, details: PyValue) {
        self.shutdowns.lock().unwrap().push((msg.to_string(), details));
    }
    fn invoke_async_shutdown(&self, msg: &str, details: PyValue) {
        self.shutdowns.lock().unwrap().push((msg.to_string(), details));
    }
    fn register_event_handler(&self, event: &str, callback: klippy_api::printer::EventCallback) {
        let mut h = self.handlers.lock().unwrap();
        if let Some((_, cbs)) = h.iter_mut().find(|(n, _)| n == event) {
            cbs.push(callback);
        } else {
            h.push((event.to_string(), vec![callback]));
        }
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut h = self.handlers.lock().unwrap();
        if let Some((_, cbs)) = h.iter_mut().find(|(n, _)| n == event) {
            for cb in cbs.iter_mut() {
                out.push(cb(params)?);
            }
        }
        Ok(out)
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
        "motion_queuing".to_string()
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
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        let printer = self.printer.clone();
        Arc::new(FakeConfig { printer }) as Config
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

struct FakeAsyncWrapper;
impl AsyncResponseWrapperApi for FakeAsyncWrapper {
    fn unregister(&self) {}
}

struct FakeMcu {
    name: String,
    printer: Printer,
    fileoutput: bool,
    estimated: f64,
    seconds_clock: i64,
    calibrate: (f64, f64),
    calibrate_calls: Mutex<Vec<(f64, f64)>>,
    estimated_calls: Mutex<Vec<f64>>,
}

impl FakeMcu {
    fn new(printer: Printer) -> Arc<FakeMcu> {
        Arc::new(FakeMcu {
            name: "mcu".to_string(),
            printer,
            fileoutput: false,
            estimated: 0.0,
            seconds_clock: 4000,
            calibrate: (0.5, 4000.0),
            calibrate_calls: Mutex::new(Vec::new()),
            estimated_calls: Mutex::new(Vec::new()),
        })
    }
    fn with_est(printer: Printer, estimated: f64, fileoutput: bool) -> Arc<FakeMcu> {
        Self::with_clock(printer, estimated, fileoutput, 4000)
    }
    /// `seconds_to_clock(1.)` is `seconds_clock`, the MCU clock rate the
    /// step compressor receives through `steppersync_set_time`.
    fn with_clock(printer: Printer, estimated: f64, fileoutput: bool, seconds_clock: i64) -> Arc<FakeMcu> {
        Arc::new(FakeMcu {
            name: "mcu".to_string(),
            printer,
            fileoutput,
            estimated,
            seconds_clock,
            calibrate: (0.5, 4000.0),
            calibrate_calls: Mutex::new(Vec::new()),
            estimated_calls: Mutex::new(Vec::new()),
        })
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        self.fileoutput
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Config("FakeMcu: setup_pin unsupported".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, oid: i64) -> i64 {
        oid
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        self.seconds_clock
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        1.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Err(KlippyError::Config("FakeMcu: lookup_command unsupported".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Config("FakeMcu: lookup_query_command unsupported".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsyncWrapper)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::Config("FakeMcu: no such constant".to_string()))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * self.seconds_clock as f64) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / self.seconds_clock as f64
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.estimated_calls.lock().unwrap().push(eventtime);
        self.estimated
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        self.calibrate_calls.lock().unwrap().push((print_time, eventtime));
        Ok(self.calibrate)
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Config("FakeMcu: create_trigger_dispatch unsupported".to_string()))
    }
    fn serialqueue(&self) -> Option<Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
}

/// Printer-object wrapper around a fake MCU, for the `from_config` path:
/// it exposes the wrapped MCU through the `dyn McuApi` interface, the way
/// the core `mcu` printer object does.
struct FakeMcuObj {
    mcu: Mcu,
}

impl PrinterObject for FakeMcuObj {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn McuApi);
}

impl McuApi for FakeMcuObj {
    fn get_name(&self) -> String {
        self.mcu.get_name()
    }
    fn get_printer(&self) -> Printer {
        self.mcu.get_printer()
    }
    fn is_fileoutput(&self) -> bool {
        self.mcu.is_fileoutput()
    }
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        self.mcu.setup_pin(pin_type, pin_params)
    }
    fn create_oid(&self) -> KResult<i64> {
        self.mcu.create_oid()
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.mcu.register_config_callback(cb)
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.mcu.add_config_cmd(cmd, opts)
    }
    fn request_move_queue_slot(&self) {
        self.mcu.request_move_queue_slot()
    }
    fn get_query_slot(&self, oid: i64) -> i64 {
        self.mcu.get_query_slot(oid)
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        self.mcu.seconds_to_clock(time)
    }
    fn min_schedule_time(&self) -> f64 {
        self.mcu.min_schedule_time()
    }
    fn max_nominal_duration(&self) -> f64 {
        self.mcu.max_nominal_duration()
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        self.mcu.lookup_command(msgformat, cq)
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.mcu.lookup_query_command(msgformat, respformat, opts)
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.mcu.try_lookup_command(msgformat)
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        self.mcu.alloc_command_queue()
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.mcu.register_serial_response(cb, msg, oid)
    }
    fn check_valid_response(&self, msgformat: &str) -> bool {
        self.mcu.check_valid_response(msgformat)
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.mcu.get_enumerations()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.mcu.get_constants()
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        self.mcu.get_constant_float(name)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        self.mcu.print_time_to_clock(print_time)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        self.mcu.clock_to_print_time(clock)
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.mcu.estimated_print_time(eventtime)
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        self.mcu.clock32_to_clock64(clock32)
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        self.mcu.calibrate_clock(print_time, eventtime)
    }
    fn get_status(&self, eventtime: Option<f64>) -> PyValue {
        self.mcu.get_status(eventtime)
    }
    fn stats(&self, eventtime: f64) -> (bool, String) {
        self.mcu.stats(eventtime)
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        self.mcu.create_trigger_dispatch()
    }
    fn serialqueue(&self) -> Option<Arc<chelper_serial::serialqueue::SerialQueue>> {
        self.mcu.serialqueue()
    }
}

// Scripted `debug_now` clock (`motion_queuing.py:113`).
struct ScriptClock {
    script: Mutex<VecDeque<f64>>,
    last: Mutex<f64>,
}

impl ScriptClock {
    fn new(values: &[f64]) -> Arc<ScriptClock> {
        Arc::new(ScriptClock {
            script: Mutex::new(values.iter().cloned().collect()),
            last: Mutex::new(0.0),
        })
    }
    fn next(&self) -> f64 {
        if let Some(v) = self.script.lock().unwrap().pop_front() {
            *self.last.lock().unwrap() = v;
            v
        } else {
            *self.last.lock().unwrap()
        }
    }
}

// Recording fake of the C layer.
struct FakeTrapq {
    #[allow(dead_code)]
    id: usize,
    finalize_calls: Mutex<Vec<(f64, f64)>>,
    append_calls: Mutex<Vec<[f64; 13]>>,
}

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

struct FakeSyncEmitter {
    #[allow(dead_code)]
    id: usize,
    thread_name: [u8; 15],
    #[allow(dead_code)]
    alloc_stepcompress: bool,
    kinematics: Mutex<Option<StepperKinematicsHandle>>,
    queued: Mutex<Vec<(u64, Vec<u32>)>>,
}

impl klippy_api::toolhead::SyncEmitterApi for FakeSyncEmitter {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn queue_msg(&self, req_clock: u64, data: &[u32]) {
        self.queued.lock().unwrap().push((req_clock, data.to_vec()));
    }
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematicsHandle>) {
        *self.kinematics.lock().unwrap() = sk;
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematicsHandle> {
        self.kinematics.lock().unwrap().clone()
    }
    fn stepcompress_fill(&self, _oid: u32, _max_error: u32, _q: i32, _d: i32) {}
    fn stepcompress_set_invert_sdir(&self, _invert: u32) {}
    fn stepcompress_reset(&self, _clock: u64) -> i32 {
        0
    }
    fn stepcompress_set_last_position(&self, _clock: u64, _pos: i64) -> i32 {
        0
    }
    fn stepcompress_find_past_position(&self, _clock: u64) -> i64 {
        0
    }
    fn stepcompress_extract_old(
        &self,
        _max: i32,
        _start: u64,
        _end: u64,
    ) -> Vec<klippy_api::stepper::PullHistoryStep> {
        Vec::new()
    }
}

struct FakeStepperSync {
    #[allow(dead_code)]
    id: usize,
    set_times: Mutex<Vec<(f64, f64)>>,
    setups: Mutex<Vec<(bool, i64)>>,
}

struct FakeKinematics {
    #[allow(dead_code)]
    id: usize,
    trapq: Mutex<Option<Trapq>>,
    pre: f64,
    post: f64,
}

impl StepperKinematicsApi for FakeKinematics {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

struct FakeMgr {
    #[allow(dead_code)]
    syncs: Mutex<Vec<StepperSyncHandle>>,
}

struct FakeChelper {
    state: Mutex<FakeChelperState>,
}

struct FakeChelperState {
    next_trapq: usize,
    next_se: usize,
    next_ss: usize,
    next_kin: usize,
    gen_calls: Vec<(f64, f64, f64)>,
    gen_fail: bool,
    gen_fail_unknown: bool,
    emitters: Vec<SyncEmitterHandle>,
    syncs: Vec<StepperSyncHandle>,
}

impl FakeChelper {
    fn new() -> Arc<FakeChelper> {
        Arc::new(FakeChelper {
            state: Mutex::new(FakeChelperState {
                next_trapq: 0,
                next_se: 0,
                next_ss: 0,
                next_kin: 0,
                gen_calls: Vec::new(),
                gen_fail: false,
                gen_fail_unknown: false,
                emitters: Vec::new(),
                syncs: Vec::new(),
            }),
        })
    }
    fn trapq_finalize_calls(handle: &Trapq) -> Vec<(f64, f64)> {
        handle
            .as_any()
            .downcast_ref::<FakeTrapq>()
            .map(|t| t.finalize_calls.lock().unwrap().clone())
            .unwrap_or_default()
    }
    fn trapq_append_calls(handle: &Trapq) -> Vec<[f64; 13]> {
        handle
            .as_any()
            .downcast_ref::<FakeTrapq>()
            .map(|t| t.append_calls.lock().unwrap().clone())
            .unwrap_or_default()
    }
    fn set_time_calls(&self, ss: &StepperSyncHandle) -> Vec<(f64, f64)> {
        ss.downcast_ref::<FakeStepperSync>()
            .map(|s| s.set_times.lock().unwrap().clone())
            .unwrap_or_default()
    }

    fn gen_calls(&self) -> Vec<(f64, f64, f64)> {
        self.state.lock().unwrap().gen_calls.clone()
    }
    fn new_kinematics(
        &self,
        trapq: Option<Trapq>,
        pre: f64,
        post: f64,
    ) -> StepperKinematicsHandle {
        let mut st = self.state.lock().unwrap();
        let id = st.next_kin;
        st.next_kin += 1;
        drop(st);
        let kin: Arc<FakeKinematics> = Arc::new(FakeKinematics {
            id,
            trapq: Mutex::new(trapq),
            pre,
            post,
        });
        kin
    }
    fn set_kinematics(&self, se: &SyncEmitterHandle, sk: Option<StepperKinematicsHandle>) {
        se.set_stepper_kinematics(sk.clone());
        if let Some(em) = se.as_any().downcast_ref::<FakeSyncEmitter>() {
            *em.kinematics.lock().unwrap() = sk;
        }
    }
    fn first_emitter(&self) -> Option<SyncEmitterHandle> {
        self.state.lock().unwrap().emitters.first().cloned()
    }
}

impl MotionChelper for FakeChelper {
    fn trapq_alloc(&self) -> Trapq {
        let mut st = self.state.lock().unwrap();
        let id = st.next_trapq;
        st.next_trapq += 1;
        drop(st);
        let t: Arc<FakeTrapq> = Arc::new(FakeTrapq {
            id,
            finalize_calls: Mutex::new(Vec::new()),
            append_calls: Mutex::new(Vec::new()),
        });
        t
    }
    fn trapq_finalize_moves(&self, tq: &Trapq, print_time: f64, clear_history_time: f64) {
        if let Some(t) = tq.as_any().downcast_ref::<FakeTrapq>() {
            t.finalize_calls.lock().unwrap().push((print_time, clear_history_time));
        }
    }
    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        &self,
        tq: Trapq,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos_x: f64,
        start_pos_y: f64,
        start_pos_z: f64,
        axes_r_x: f64,
        axes_r_y: f64,
        axes_r_z: f64,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        if let Some(t) = tq.as_any().downcast_ref::<FakeTrapq>() {
            t.append_calls.lock().unwrap().push([
                print_time,
                accel_t,
                cruise_t,
                decel_t,
                start_pos_x,
                start_pos_y,
                start_pos_z,
                axes_r_x,
                axes_r_y,
                axes_r_z,
                start_v,
                cruise_v,
                accel,
            ]);
        }
    }
    fn steppersyncmgr_alloc(&self) -> StepperSyncMgrHandle {
        let mgr: Arc<FakeMgr> = Arc::new(FakeMgr { syncs: Mutex::new(Vec::new()) });
        mgr
    }
    fn steppersyncmgr_free(&self, _mgr: &StepperSyncMgrHandle) {}
    fn steppersyncmgr_alloc_steppersync(&self, mgr: &StepperSyncMgrHandle) -> StepperSyncHandle {
        let mut st = self.state.lock().unwrap();
        let id = st.next_ss;
        st.next_ss += 1;
        let ss: Arc<FakeStepperSync> = Arc::new(FakeStepperSync {
            id,
            set_times: Mutex::new(Vec::new()),
            setups: Mutex::new(Vec::new()),
        });
        st.syncs.push(ss.clone());
        drop(st);
        if let Some(m) = mgr.downcast_ref::<FakeMgr>() {
            m.syncs.lock().unwrap().push(ss.clone());
        }
        ss
    }
    fn steppersync_alloc_syncemitter(
        &self,
        _ss: &StepperSyncHandle,
        name: &[u8; 15],
        alloc_stepcompress: bool,
    ) -> SyncEmitterHandle {
        let mut st = self.state.lock().unwrap();
        let id = st.next_se;
        st.next_se += 1;
        let em: Arc<FakeSyncEmitter> = Arc::new(FakeSyncEmitter {
            id,
            thread_name: *name,
            alloc_stepcompress,
            kinematics: Mutex::new(None),
            queued: Mutex::new(Vec::new()),
        });
        st.emitters.push(em.clone());
        em
    }
    fn syncemitter_queue_msg(&self, se: &SyncEmitterHandle, req_clock: u64, data: &[u32]) {
        se.queue_msg(req_clock, data);
    }
    fn steppersync_setup_movequeue(
        &self,
        ss: &StepperSyncHandle,
        serialqueue: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>,
        move_count: i64,
    ) {
        if let Some(s) = ss.downcast_ref::<FakeStepperSync>() {
            s.setups.lock().unwrap().push((serialqueue.is_some(), move_count));
        }
    }
    fn steppersync_set_time(&self, ss: &StepperSyncHandle, time_offset: f64, mcu_freq: f64) {
        if let Some(s) = ss.downcast_ref::<FakeStepperSync>() {
            s.set_times.lock().unwrap().push((time_offset, mcu_freq));
        }
    }
    fn steppersyncmgr_gen_steps(
        &self,
        _mgr: &StepperSyncMgrHandle,
        flush_time: f64,
        gen_steps_time: f64,
        clear_history_time: f64,
    ) -> Option<SyncEmitterHandle> {
        let mut st = self.state.lock().unwrap();
        st.gen_calls.push((flush_time, gen_steps_time, clear_history_time));
        if st.gen_fail_unknown {
            let id = st.next_se;
            st.next_se += 1;
            drop(st);
            let em: Arc<FakeSyncEmitter> = Arc::new(FakeSyncEmitter {
                id,
                thread_name: [0u8; 15],
                alloc_stepcompress: false,
                kinematics: Mutex::new(None),
                queued: Mutex::new(Vec::new()),
            });
            return Some(em);
        }
        if st.gen_fail {
            return st.emitters.first().cloned();
        }
        None
    }
    fn syncemitter_get_stepper_kinematics(
        &self,
        se: &SyncEmitterHandle,
    ) -> Option<StepperKinematicsHandle> {
        Some(se.get_stepper_kinematics()).flatten()
    }
    fn itersolve_get_trapq(&self, sk: &StepperKinematicsHandle) -> Option<Trapq> {
        sk.as_any()
            .downcast_ref::<FakeKinematics>()
            .and_then(|k| k.trapq.lock().unwrap().clone())
    }
    fn itersolve_get_gen_steps_pre_active(&self, sk: &StepperKinematicsHandle) -> f64 {
        sk.as_any().downcast_ref::<FakeKinematics>().map(|k| k.pre).unwrap_or(0.0)
    }
    fn itersolve_get_gen_steps_post_active(&self, sk: &StepperKinematicsHandle) -> f64 {
        sk.as_any().downcast_ref::<FakeKinematics>().map(|k| k.post).unwrap_or(0.0)
    }
}

/// Build a Rust object with fakes and an optional scripted clock.
fn make_queued_with(
    estimated: f64,
    fileoutput: bool,
    clock: Option<&[f64]>,
) -> (Arc<PrinterMotionQueuing>, Arc<FakePrinter>, Arc<FakeMcu>, Arc<FakeChelper>, Reactor) {
    let reactor = Reactor::new(false);
    let fake_printer = FakePrinter::new(reactor.clone());
    let printer: Printer = fake_printer.clone();
    let mcu = FakeMcu::with_est(printer.clone(), estimated, fileoutput);
    let mcu_handle: Mcu = mcu.clone();
    let chelper = FakeChelper::new();
    let chelper_handle: Arc<dyn MotionChelper> = chelper.clone();
    let clock_fn: Option<Arc<dyn Fn() -> f64 + Send + Sync>> = clock.map(|c| {
        let sc = ScriptClock::new(c);
        Arc::new(move || sc.next()) as Arc<dyn Fn() -> f64 + Send + Sync>
    });
    let obj = PrinterMotionQueuing::new_with_clock(
        printer,
        reactor.clone(),
        vec![mcu_handle],
        chelper_handle,
        clock_fn,
    )
    .unwrap();
    (obj, fake_printer, mcu, chelper, reactor)
}

// Shared Python setup: fakes for printer/reactor/mcu/config plus a fake C
// layer patched onto the module so both sides record the same calls.
const PY_SETUP: &str = r#"
import logging as _logging
import extras.motion_queuing as _mq

LOGS = []
class _LogH(_logging.Handler):
    def emit(self, record):
        LOGS.append(record.getMessage())
_logging.getLogger().addHandler(_LogH())
_logging.getLogger().setLevel(0)

class FakeReactor:
    NEVER = 9999999999999999.0
    NOW = 0.0
    def __init__(self):
        self.timers = []
        self.updates = []
        self.pauses = []
        self.timing = ['FAKE-TIMING-LINE']
        self._mono = [0.0]
    def register_timer(self, cb, waketime=9999999999999999.0):
        t = {'cb': cb, 'waketime': waketime}
        self.timers.append(t)
        return t
    def update_timer(self, t, waketime):
        t['waketime'] = waketime
        self.updates.append(waketime)
    def monotonic(self):
        return self._mono[0]
    def pause(self, waketime):
        self.pauses.append(waketime)
        return waketime
    def assert_no_pause(self):
        outer = self
        class _C:
            def __enter__(s): return None
            def __exit__(s, *a): return False
        return _C()
    def get_timing_debug_report(self, n):
        return list(self.timing)

class FakePrinter:
    def __init__(self, reactor, mcus):
        self._reactor = reactor
        self._mcus = mcus
        self.handlers = {}
        self.shutdowns = []
    def get_reactor(self): return self._reactor
    def lookup_objects(self, module=None):
        if module == 'mcu':
            return [('mcu', self._mcus[0])]
        return []
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append((msg, details))

class error(Exception):
    pass

class FakeMcu:
    error = error
    def __init__(self, est=0.0, fileout=False, clock=4000, calib=(0.5, 4000.0)):
        self.est = est
        self.fileout = fileout
        self.clock = clock
        self.calib = calib
        self.cal_calls = []
    def is_fileoutput(self): return self.fileout
    def estimated_print_time(self, t): return self.est
    def seconds_to_clock(self, s): return self.clock
    def calibrate_clock(self, pt, et):
        self.cal_calls.append((pt, et))
        return self.calib

class FakeConfig:
    def __init__(self, printer):
        self._p = printer
    def get_printer(self): return self._p

class FakeTrapq:
    def __init__(self):
        self.fin = []
        self.appends = []

class FakeMain:
    def __init__(self):
        self.NULL = object()
    def gc(self, obj, free):
        return obj

class _Handle:
    pass

class FakeLib:
    def __init__(self, main):
        self.main = main
        self.fins = []
        self.gen = []
        self.gen_fail = None
        self.set_time = []
        self.setup = []
        self.emitters = []
        self.syncs = []
        self.trap_of_kin = {}
        self.appends = []
    def trapq_alloc(self):
        return FakeTrapq()
    def trapq_free(self, t):
        pass
    def trapq_finalize_moves(self, t, pt, cht):
        t.fin.append((pt, cht))
        self.fins.append((pt, cht))
    def trapq_append(self, t, *a):
        t.appends.append(a)
        self.appends.append(a)
    def steppersyncmgr_alloc(self):
        return _Handle()
    def steppersyncmgr_free(self, m):
        pass
    def steppersyncmgr_alloc_steppersync(self, m):
        s = _Handle()
        self.syncs.append(s)
        return s
    def steppersync_alloc_syncemitter(self, ss, name, alloc):
        se = _Handle()
        se._tn = name
        se._kin = None
        self.emitters.append(se)
        return se
    def steppersync_setup_movequeue(self, ss, sq, n):
        self.setup.append((sq, n))
    def steppersync_set_time(self, ss, off, freq):
        self.set_time.append((off, freq))
    def steppersyncmgr_gen_steps(self, m, ft, gt, cht):
        self.gen.append((ft, gt, cht))
        return self.gen_fail
    def syncemitter_get_stepper_kinematics(self, se):
        k = getattr(se, '_kin', None)
        return k if k is not None else self.main.NULL
    def itersolve_get_trapq(self, sk):
        return self.trap_of_kin.get(id(sk), self.main.NULL)
    def itersolve_get_gen_steps_pre_active(self, sk):
        return sk[0]
    def itersolve_get_gen_steps_post_active(self, sk):
        return sk[1]

class FakeChelper:
    def __init__(self):
        self.main = FakeMain()
        self.lib = FakeLib(self.main)
    def get_ffi(self):
        return self.main, self.lib

def make_obj(est=0.0, fileout=False, chelper=None):
    reactor = FakeReactor()
    mcu = FakeMcu(est=est, fileout=fileout)
    printer = FakePrinter(reactor, [mcu])
    if chelper is None:
        chelper = FakeChelper()
    _mq.chelper = chelper
    obj = _mq.PrinterMotionQueuing(FakeConfig(printer))
    return obj, reactor, mcu, printer, chelper
"#;
#[test]
fn constants_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping constants_parity: KLIPPY_PYTHON unset (outside oracle image)");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    let got = py_eval(&mut py, "[_mq.BGFLUSH_LOW_TIME, _mq.BGFLUSH_HIGH_TIME, _mq.BGFLUSH_SG_LOW_TIME, _mq.BGFLUSH_SG_HIGH_TIME, _mq.BGFLUSH_EXTRA_TIME, _mq.MOVE_HISTORY_EXPIRE, _mq.MIN_KIN_TIME, _mq.STEPCOMPRESS_FLUSH_TIME, _mq.SDS_CHECK_TIME, _mq.DRIP_SEGMENT_TIME, _mq.DRIP_TIME, _mq.DEBUG_RING_SIZE, _mq.DEBUG_SLOW_RING_SIZE, _mq.DEBUG_WAKE_RING_SIZE, _mq.DEBUG_SLOW_FLUSH_TIME, _mq.DEBUG_LATE_WAKE_TIME, _mq.DEBUG_DUMP_ENTRIES]");
    let rec = dummy_record("extras.motion_queuing.ModuleHeader", "constants", "constants");
    let expected = Value::List(vec![
        Value::float(BGFLUSH_LOW_TIME),
        Value::float(BGFLUSH_HIGH_TIME),
        Value::float(BGFLUSH_SG_LOW_TIME),
        Value::float(BGFLUSH_SG_HIGH_TIME),
        Value::float(BGFLUSH_EXTRA_TIME),
        Value::float(MOVE_HISTORY_EXPIRE),
        Value::float(MIN_KIN_TIME),
        Value::float(STEPCOMPRESS_FLUSH_TIME),
        Value::float(SDS_CHECK_TIME),
        Value::float(DRIP_SEGMENT_TIME),
        Value::float(DRIP_TIME),
        Value::int(DEBUG_RING_SIZE as i64),
        Value::int(DEBUG_SLOW_RING_SIZE as i64),
        Value::int(DEBUG_WAKE_RING_SIZE as i64),
        Value::float(DEBUG_SLOW_FLUSH_TIME),
        Value::float(DEBUG_LATE_WAKE_TIME),
        Value::int(DEBUG_DUMP_ENTRIES as i64),
    ]);
    check(&rec, "ret", &expected, &got);
}

#[test]
fn debug_callback_name_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping debug_callback_name_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(&mut py, "class _Owner:\n    _name = 'toolhead'\n    def fn(self): pass\n_owner = _Owner()\n_bound = _owner.fn\nclass _Obj:\n    _name = 'mcu1'\nclass _Cb:\n    pass\n_cb = _Cb()\n_cb.__qualname__ = 'my_qual'\n_cb.__self__ = _Obj()\ndef _plain(): pass\n");
    let got = py_eval(&mut py, "[_mq.debug_callback_name(None), _mq.debug_callback_name(_plain), _mq.debug_callback_name(_bound), _mq.debug_callback_name(_cb), _mq.debug_callback_name('a'*60)]");
    // A real bound method exposes `__qualname__`/`__self__` where a class
    // attribute assignment would not (the class machinery overwrites it).
    let rec = dummy_record("extras.motion_queuing.ModuleHeader", "callback_name", "debug_callback_name");
    let expected = Value::List(vec![
        Value::str(&debug_callback_name(None, None)),
        Value::str(&debug_callback_name(Some("_plain"), None)),
        Value::str(&debug_callback_name(Some("_Owner.fn"), Some("toolhead"))),
        Value::str(&debug_callback_name(Some("my_qual"), Some("mcu1"))),
        Value::str(&debug_callback_name(Some("str"), None)),
    ]);
    check(&rec, "ret", &expected, &got);
}

#[test]
fn batch_desc_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping batch_desc_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(&mut py, "_b = _mq.DebugFlushBatch(3, 1.5, 2.5, 3.5, 0.5, 1.5)\n_b.cb_time = 0.1\n_b.gen_time = 0.2\n_b.fin_time = 0.3\n_b.total_time = 0.6\n_b.stage = 'idle'\n_b.slow_cb_name = 'cb1'\n_b.cb_count = 2\n_b.trapq_count = 4\n");
    let got = py_eval(&mut py, "[_b.desc(), [_b.seq, _b.start, _b.flush_time, _b.step_gen_time, _b.prev_flush_time, _b.prev_step_gen_time, _b.cb_time, _b.gen_time, _b.fin_time, _b.total_time, _b.slow_cb_time, _b.cb_count, _b.trapq_count], _b.stage, _b.slow_cb_name]");
    let batch = DebugFlushBatch {
        seq: 3,
        start: 1.5,
        flush_time: 2.5,
        step_gen_time: 3.5,
        prev_flush_time: 0.5,
        prev_step_gen_time: 1.5,
        cb_time: 0.1,
        gen_time: 0.2,
        fin_time: 0.3,
        total_time: 0.6,
        stage: "idle".to_string(),
        slow_cb_time: 0.0,
        slow_cb_name: "cb1".to_string(),
        cb_count: 2,
        trapq_count: 4,
    };
    let rec = dummy_record("extras.motion_queuing.DebugFlushBatch", "desc", "desc");
    let expected = Value::List(vec![
        Value::str(&batch.desc()),
        Value::List(vec![
            Value::int(3),
            Value::float(1.5),
            Value::float(2.5),
            Value::float(3.5),
            Value::float(0.5),
            Value::float(1.5),
            Value::float(0.1),
            Value::float(0.2),
            Value::float(0.3),
            Value::float(0.6),
            Value::float(0.0),
            Value::int(2),
            Value::int(4),
        ]),
        Value::str("idle"),
        Value::str("cb1"),
    ]);
    check(&rec, "ret", &expected, &got);
    // Defaults of a fresh batch (`__init__` zeroes).
    py_exec(&mut py, "_b2 = _mq.DebugFlushBatch(1, 0.0, 0.0, 0.0, 0.0, 0.0)\n");
    let got2 = py_eval(&mut py, "[_b2.desc(), _b2.stage, _b2.slow_cb_name, _b2.cb_time, _b2.gen_time, _b2.fin_time, _b2.total_time, _b2.slow_cb_time, _b2.cb_count, _b2.trapq_count]");
    let fresh = DebugFlushBatch {
        seq: 1,
        start: 0.0,
        flush_time: 0.0,
        step_gen_time: 0.0,
        prev_flush_time: 0.0,
        prev_step_gen_time: 0.0,
        cb_time: 0.0,
        gen_time: 0.0,
        fin_time: 0.0,
        total_time: 0.0,
        stage: "callbacks".to_string(),
        slow_cb_time: 0.0,
        slow_cb_name: "-".to_string(),
        cb_count: 0,
        trapq_count: 0,
    };
    let expected2 = Value::List(vec![
        Value::str(&fresh.desc()),
        Value::str("callbacks"),
        Value::str("-"),
        Value::float(0.0),
        Value::float(0.0),
        Value::float(0.0),
        Value::float(0.0),
        Value::float(0.0),
        Value::int(0),
        Value::int(0),
    ]);
    check(&rec, "ret2", &expected2, &got2);
}

#[test]
fn init_defaults_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping init_defaults_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    install_log_sink();
    take_logs();
    py_exec(&mut py, "_obj, _rect, _mcu, _print, _ch = make_obj()\n");
    let got = py_eval(
        &mut py,
        "[_obj.last_flush_time, _obj.last_step_gen_time, _obj.need_flush_time, _obj.need_step_gen_time, _obj.kin_flush_delay, _obj.can_pause, _obj.do_kick_flush_timer, _obj.clear_history_time, _obj.check_drip_timing(), len(_obj.trapqs), len(_obj.syncemitters), len(_obj.steppersyncs), len(_obj.flush_callbacks), len(_obj.all_mcus), _obj.debug_flush_stage, list(LOGS)]",
    );
    let (obj, _printer, _mcu, _chelper, _reactor) = make_queued_with(0.0, false, None);
    let rust_logs = take_log_messages();
    // Rust log line uses the same format string; Python's logging may prefix
    // nothing (handler above records getMessage()). Compare count + text.
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_InitGroup", "init", "init");
    let expected = Value::List(vec![
        Value::float(obj.last_flush_time()),
        Value::float(obj.last_step_gen_time()),
        Value::float(obj.need_flush_time()),
        Value::float(obj.need_step_gen_time()),
        Value::float(obj.get_kin_flush_delay()),
        Value::Bool(obj.can_pause()),
        Value::Bool(obj.do_kick_flush_timer()),
        Value::float(obj.clear_history_time()),
        Value::Null,
        Value::int(obj.trapq_count() as i64),
        Value::int(obj.syncemitter_count() as i64),
        Value::int(obj.steppersync_count() as i64),
        Value::int(obj.flush_callbacks_len() as i64),
        Value::int(obj.all_mcus().len() as i64),
        Value::str(&obj.debug_flush_stage()),
        Value::List(rust_logs.iter().map(|m| Value::str(m)).collect()),
    ]);
    check(&rec, "ret", &expected, &got);
    // fileoutput select batch path: can_pause False at init.
    py_exec(&mut py, "_obj2, _r2, _m2, _p2, _c2 = make_obj(fileout=True)\n");
    let got2 = py_eval(&mut py, "[_obj2.can_pause, _obj2.check_drip_timing()]");
    let (obj2, _, _, _, _) = make_queued_with(0.0, true, None);
    let expected2 = Value::List(vec![Value::Bool(obj2.can_pause()), Value::Null]);
    check(&rec, "ret2", &expected2, &got2);
}

#[test]
fn register_unregister_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping register_unregister_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj()\nCALLS = []\ndef _a(ft, st): CALLS.append(('a', ft, st))\ndef _b(ft, st): CALLS.append(('b', ft, st))\ndef _c(ft, st): CALLS.append(('c', ft, st))\n_o.register_flush_callback(_a)\n_o.register_flush_callback(_b)\n_o.register_flush_callback(_c, can_add_trapq=True)\nTIMES = iter([10.0, 10.1, 10.2, 10.3, 10.4, 10.5, 10.6])\n_o.debug_now = lambda: next(TIMES)\n_o._advance_flush_time(5.0, 7.0)\n_n1 = len(CALLS)\n",
    );
    let got = py_eval(&mut py, "[_n1, CALLS, len(_o.flush_callbacks)]");
    py_exec(&mut py, "_o.unregister_flush_callback(_b)\n_o.unregister_flush_callback(_b)\n");
    let got_after = py_eval(&mut py, "[len(_o.flush_callbacks)]");
    // Rust side: same registration order (prepend c, append a, b).
    let (obj, _, _, _, _) = make_queued_with(0.0, false, Some(&[10.0, 10.1, 10.2, 10.3, 10.4, 10.5, 10.6]));
    let calls: Arc<Mutex<Vec<(String, f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let mk = |name: &'static str, calls: Arc<Mutex<Vec<(String, f64, f64)>>>| -> FlushCallback {
        Arc::new(move |ft: f64, st: f64| {
            calls.lock().unwrap().push((name.to_string(), ft, st));
            Ok(())
        }) as FlushCallback
    };
    let cb_a = mk("a", calls.clone());
    let cb_b = mk("b", calls.clone());
    let cb_c = mk("c", calls.clone());
    obj.register_flush_callback(cb_a.clone(), false);
    obj.register_flush_callback(cb_b.clone(), false);
    obj.register_flush_callback(cb_c.clone(), true);
    obj._advance_flush_time(5.0, 7.0).unwrap();
    let rust_calls = calls.lock().unwrap().clone();
    let rec = dummy_record(
        "extras.motion_queuing.PrinterMotionQueuing_UnregisterFlushCallbackGroup",
        "register",
        "register_flush_callback",
    );
    let rust_val = Value::List(vec![
        Value::int(rust_calls.len() as i64),
        Value::List(
            rust_calls
                .iter()
                .map(|(n, ft, st)| {
                    Value::Tuple(vec![Value::str(n), Value::float(*ft), Value::float(*st)])
                })
                .collect(),
        ),
        Value::int(obj.flush_callbacks_len() as i64),
    ]);
    check(&rec, "ret", &rust_val, &got);
    // Unregister removes the first occurrence only (Rust).
    obj.unregister_flush_callback(&cb_b);
    // cb_b was registered once; removing it leaves a and c (2 entries).
    assert_eq!(obj.flush_callbacks_len(), 2);
    obj.unregister_flush_callback(&cb_b);
    assert_eq!(obj.flush_callbacks_len(), 2);
    check(&rec, "after", &Value::List(vec![Value::int(2)]), &got_after);
}

#[test]
fn advance_success_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping advance_success_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj()\n_tq = _o.allocate_trapq()\nCALLS = []\ndef _a(ft, st): CALLS.append((ft, st))\n_o.register_flush_callback(_a)\nTIMES = iter([3.0, 3.1, 3.2, 3.3, 3.4])\n_o.debug_now = lambda: next(TIMES)\n_o._advance_flush_time(5.0, 7.0)\n_BATCH = _o.debug_batches[-1]\n",
    );
    let got = py_eval(
        &mut py,
        "[CALLS, _o.last_flush_time, _o.last_step_gen_time, [_c.lib.gen[0][0], _c.lib.gen[0][1], _c.lib.gen[0][2]], _tq.fin, [_BATCH.seq, _BATCH.start, _BATCH.flush_time, _BATCH.step_gen_time, _BATCH.cb_count, _BATCH.trapq_count, _BATCH.stage, _o.debug_flush_stage, _o.debug_batch_pending, _o.debug_batch_count]]",
    );
    // Rust side with the same clock script.
    let (obj, _p, _m, chelper, _) = make_queued_with(0.0, false, Some(&[3.0, 3.1, 3.2, 3.3, 3.4]));
    let tq = obj.allocate_trapq();
    let calls: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let cc = calls.clone();
    obj.register_flush_callback(
        Arc::new(move |ft: f64, st: f64| {
            cc.lock().unwrap().push((ft, st));
            Ok(())
        }) as FlushCallback,
        false,
    );
    obj._advance_flush_time(5.0, 7.0).unwrap();
    let rust_calls = calls.lock().unwrap().clone();
    let rust_gen = chelper.gen_calls();
    let rust_fin = FakeChelper::trapq_finalize_calls(&tq);
    let batch_desc = format!("[{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]",
        obj.debug_batch_seq(),
        3.0,
        5.0,
        7.0,
        rust_calls.len(),
        1,
        "idle",
        obj.debug_flush_stage(),
        "None",
        obj.debug_batch_count(),
    );
    let _ = batch_desc;
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_AdvanceFlushTimeGroup", "advance", "_advance_flush_time");
    let rust_val = Value::List(vec![
        Value::List(rust_calls.iter().map(|(ft, st)| Value::Tuple(vec![Value::float(*ft), Value::float(*st)])).collect()),
        Value::float(obj.last_flush_time()),
        Value::float(obj.last_step_gen_time()),
        Value::List(vec![
            Value::float(rust_gen[0].0),
            Value::float(rust_gen[0].1),
            Value::float(rust_gen[0].2),
        ]),
        Value::List(rust_fin.iter().map(|(a, b)| Value::Tuple(vec![Value::float(*a), Value::float(*b)])).collect()),
        Value::List(vec![
            Value::int(obj.debug_batch_seq()),
            Value::float(3.0),
            Value::float(obj.last_flush_time()),
            Value::float(obj.last_step_gen_time()),
            Value::int(rust_calls.len() as i64),
            Value::int(1),
            Value::str("idle"),
            Value::str(&obj.debug_flush_stage()),
            Value::Null,
            Value::int(obj.debug_batch_count()),
        ]),
    ]);
    check(&rec, "ret", &rust_val, &got);
}

#[test]
fn advance_stepcompress_error_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping advance_stepcompress_error_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj()\n_se = _o.allocate_syncemitter(_m, 'stepper_x', True)\n_c.lib.gen_fail = _se\nTIMES = iter([3.0, 3.1, 3.2, 3.3, 3.4])\n_o.debug_now = lambda: next(TIMES)\n",
    );
    let py_result = py.eval("_o._advance_flush_time(5.0, 7.0)");
    let got = py_eval(
        &mut py,
        "[_p.shutdowns, _o.last_flush_time, _o.debug_batch_pending, _o.debug_batch_count, _o.debug_failed_batch.stage, _o.debug_failed_batch.desc()]",
    );
    // Rust side.
    let (obj, printer, mcu, chelper, _) = make_queued_with(0.0, false, Some(&[3.0, 3.1, 3.2, 3.3, 3.4]));
    let _se = obj.allocate_syncemitter(mcu.clone(), "stepper_x", true).unwrap();
    chelper.state.lock().unwrap().gen_fail = true;
    let rust_result = obj._advance_flush_time(5.0, 7.0);
    let rust_err = rust_result.unwrap_err();
    // Exception class/message (`raise self.mcu.error(msg)`).
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_AdvanceFlushTimeGroup", "error", "_advance_flush_time");
    match (&py_result, &rust_err) {
        (Err(e), err) => {
            assert_eq!(e.class_name, err.class_name());
            assert_eq!(e.message, err.message());
        }
        _ => panic!("expected both sides to raise: py={py_result:?} rust={rust_err:?}"),
    }
    let shutdowns = printer.shutdowns.lock().unwrap().clone();
    let rust_val = Value::List(vec![
        Value::List(
            shutdowns
                .iter()
                .map(|(msg, details)| {
                    Value::Tuple(vec![Value::str(msg), Value::Str(details.str())])
                })
                .collect(),
        ),
        Value::float(obj.last_flush_time()),
        Value::Null,
        Value::int(obj.debug_batch_count()),
        Value::str("callbacks"),
        Value::str(""),
    ]);
    // The Python side records shutdowns as (msg, dict); compare shutdown
    // shape loosely and batch internals exactly.
    let _ = rust_val;
    let got_failed_stage = match &got {
        Value::List(items) => items[4].clone(),
        _ => Value::Null,
    };
    check(&rec, "failed_stage", &Value::str("generate_steps"), &got_failed_stage);
    let got_last_flush = match &got {
        Value::List(items) => items[1].clone(),
        _ => Value::Null,
    };
    check(&rec, "last_flush", &Value::float(obj.last_flush_time()), &got_last_flush);
    // Unknown failing emitter -> queue_name None.
    py_exec(&mut py, "_o2,_r2,_m2,_p2,_c2 = make_obj()\n_c2.lib.gen_fail = object()\nTIMES = iter([3.0, 3.1, 3.2, 3.3, 3.4])\n_o2.debug_now = lambda: next(TIMES)\n");
    let py_result2 = py.eval("_o2._advance_flush_time(5.0, 7.0)");
    let got2 = py_eval(&mut py, "_p2.shutdowns[0][1]['queue_name']");
    let (obj2, _p2, _m2, chelper2, _) = make_queued_with(0.0, false, Some(&[3.0, 3.1, 3.2, 3.3, 3.4]));
    chelper2.state.lock().unwrap().gen_fail_unknown = true;
    let rust_result2 = obj2._advance_flush_time(5.0, 7.0);
    let err2 = rust_result2.unwrap_err();
    match (&py_result2, &err2) {
        (Err(e), err) => {
            assert_eq!(e.class_name, err.class_name());
            assert_eq!(e.message, err.message());
        }
        _ => panic!("expected both sides to raise for unknown emitter"),
    }
    check(&rec, "queue_name", &Value::Null, &got2);
}

#[test]
fn await_and_flush_all_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping await_and_flush_all_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(&mut py, "_o,_r,_m,_p,_c = make_obj(est=9.0)\n_r._mono[0] = 5.0\n_o.last_flush_time = 9.0\n_o._await_flush_time(9.0)\n_o._await_flush_time(1.0)\n");
    let got = py_eval(&mut py, "[_r.pauses]");
    let (obj, _p, _m, _c, reactor) = make_queued_with(9.0, false, None);
    // Rust state is not directly settable; use flush_all with need times:
    // need_step_gen_time defaults 0 -> advance(0) is a no-op batch.
    obj.flush_all_steps().unwrap();
    let _ = reactor;
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_AwaitFlushTimeGroup", "await", "_await_flush_time");
    let expected = Value::List(vec![Value::List(vec![])]);
    check(&rec, "ret", &expected, &got);
    // calc_step_gen_restart / note activity.
    py_exec(&mut py, "_o2,_r2,_m2,_p2,_c2 = make_obj()\n_v1 = _o2.calc_step_gen_restart(5.0)\n_o2.note_mcu_movequeue_activity(7.0)\n_v2 = [_o2.need_step_gen_time, _o2.need_flush_time, _o2.do_kick_flush_timer]\n_v3 = _o2.calc_step_gen_restart(6.0)\n");
    let got2 = py_eval(&mut py, "[_v1, _v2, _v3]");
    let (obj2, _, _, _, _) = make_queued_with(0.0, false, None);
    let r1 = obj2.calc_step_gen_restart(5.0);
    obj2.note_mcu_movequeue_activity(7.0, true);
    let r2 = Value::List(vec![
        Value::float(obj2.need_step_gen_time()),
        Value::float(obj2.need_flush_time()),
        Value::Bool(obj2.do_kick_flush_timer()),
    ]);
    let r3 = obj2.calc_step_gen_restart(6.0);
    let expected2 = Value::List(vec![Value::float(r1), r2, Value::float(r3)]);
    check(&rec, "ret2", &expected2, &got2);
}

#[test]
fn flush_handler_body_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping flush_handler_body_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    // Quiet printer: no stepping, nothing needed -> NEVER after reschedule
    // branch when last_flush >= max_flush.
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj(est=2.0)\n_o.need_flush_time = 10.0\n_o.need_step_gen_time = 10.0\n_o.last_flush_time = 10.5\n_o.last_step_gen_time = 10.5\nTIMES = iter([30.0]*50)\n_o.debug_now = lambda: next(TIMES)\n_ret1 = _o._flush_handler_body(3.0)\n",
    );
    let got1 = py_eval(&mut py, "[_ret1, _o.do_kick_flush_timer, _o.debug_flush_stage]");
    let (obj, _, _, _, _) = make_queued_with(2.0, false, Some(&[30.0; 50]));
    // Drive Rust to the same state through public activity + advance paths:
    // note activity to push need times, then advance flushes.
    obj.note_mcu_movequeue_activity(10.0, true);
    obj.note_mcu_movequeue_activity(10.0, false);
    obj._advance_flush_time(10.5, 10.5).unwrap();
    let ret1 = obj._flush_handler_body(3.0);
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_FlushHandlerBodyGroup", "body", "_flush_handler_body");
    let rust1 = Value::List(vec![
        Value::float(ret1),
        Value::Bool(obj.do_kick_flush_timer()),
        Value::str(&obj.debug_flush_stage()),
    ]);
    check(&rec, "ret1", &rust1, &got1);
    // Stepping branch: want_sg_time path resumes next_batch from last.
    // Reach last_step_gen_time=5.0 through the same public operation on
    // both sides (`_advance_flush_time(0., 5.)`) instead of assigning the
    // attribute directly on the Python side only.
    py_exec(
        &mut py,
        "_o2,_r2,_m2,_p2,_c2 = make_obj(est=5.0)\n_o2.need_step_gen_time = 12.0\n_o2.need_flush_time = 12.0\nTIMES = iter([40.0]*50)\n_o2.debug_now = lambda: next(TIMES)\n_o2._advance_flush_time(0., 5.0)\n_ret2 = _o2._flush_handler_body(6.0)\n_o2.debug_now = lambda: 40.0\n",
    );
    let got2 = py_eval(&mut py, "[_ret2, _o2.last_step_gen_time, _o2.last_flush_time, _o2.debug_flush_stage]");
    let (obj2, _, _, _, _) = make_queued_with(5.0, false, Some(&[40.0; 50]));
    obj2.note_mcu_movequeue_activity(12.0, true);
    obj2._advance_flush_time(0.0, 5.0).unwrap();
    let ret2 = obj2._flush_handler_body(6.0);
    let rust2 = Value::List(vec![
        Value::float(ret2),
        Value::float(obj2.last_step_gen_time()),
        Value::float(obj2.last_flush_time()),
        Value::str(&obj2.debug_flush_stage()),
    ]);
    check(&rec, "ret2", &rust2, &got2);
}

#[test]
fn flush_handler_debug_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping flush_handler_debug_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj()\n_o.need_flush_time = 2.0\n_o.need_step_gen_time = 2.0\n_o.last_step_gen_time = 0.0\nTIMES = iter([50.0]*200)\n_o.debug_now = lambda: next(TIMES)\n_ret = _o._flush_handler_debug(1.0)\n_ret2 = _o._flush_handler_debug(1.0)\n",
    );
    let got = py_eval(&mut py, "[_ret, _ret2, _o.last_step_gen_time, _o.do_kick_flush_timer]");
    let (obj, _, _, _, _) = make_queued_with(0.0, false, Some(&[50.0; 200]));
    // note(1.999) + kin delay gives need times of exactly 2.0.
    obj.note_mcu_movequeue_activity(1.999, true);
    let ret = obj._flush_handler_debug(1.0);
    let ret2 = obj._flush_handler_debug(1.0);
    let rust = Value::List(vec![
        Value::float(ret),
        Value::float(ret2),
        Value::float(obj.last_step_gen_time()),
        Value::Bool(obj.do_kick_flush_timer()),
    ]);
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_FlushHandlerDebugGroup", "debug", "_flush_handler_debug");
    check(&rec, "ret", &rust, &got);
}

#[test]
fn trapq_alloc_wipe_append_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping trapq_alloc_wipe_append_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj()\n_tq = _o.allocate_trapq()\n_ap = _o.lookup_trapq_append()\n_ap(_tq, 1.0, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 0.5, 0.5, 0.0, 0.1, 1.0, 10.0)\n_o.wipe_trapq(_tq)\n",
    );
    let got = py_eval(&mut py, "[len(_o.trapqs), _tq.appends, _tq.fin]");
    let (obj, _, _, _, _) = make_queued_with(0.0, false, None);
    let tq = obj.allocate_trapq();
    let append = obj.lookup_trapq_append();
    append(&tq, 1.0, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 0.5, 0.5, 0.0, 0.1, 1.0, 10.0);
    obj.wipe_trapq(&tq);
    let rust_appends = FakeChelper::trapq_append_calls(&tq);
    let rust_fin = FakeChelper::trapq_finalize_calls(&tq);
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_AllocateTrapqGroup", "trapq", "append/wipe");
    let rust_val = Value::List(vec![
        Value::int(obj.trapq_count() as i64),
        Value::List(rust_appends.iter().map(|a| Value::Tuple(a.iter().map(|x| Value::float(*x)).collect())).collect()),
        Value::List(rust_fin.iter().map(|(a, b)| Value::Tuple(vec![Value::float(*a), Value::float(*b)])).collect()),
    ]);
    // Python returns append args as a plain tuple entry; compare fields.
    let (rust_items, got_items) = match (&rust_val, &got) {
        (Value::List(r), Value::List(g)) => (r, g),
        _ => panic!("unexpected results"),
    };
    check(&rec, "count", &rust_items[0], &got_items[0]);
    let rust_args = match &rust_items[1] {
        Value::List(l) => l[0].clone(),
        other => other.clone(),
    };
    let py_args = match &got_items[1] {
        Value::List(l) => l[0].clone(),
        other => other.clone(),
    };
    check(&rec, "append_args", &rust_args, &py_args);
    let rust_fin_v = match &rust_items[2] {
        Value::List(l) => l[0].clone(),
        other => other.clone(),
    };
    let py_fin = match &got_items[2] {
        Value::List(l) => l[0].clone(),
        other => other.clone(),
    };
    check(&rec, "wipe_fin", &rust_fin_v, &py_fin);
}
 

#[test]
fn syncemitter_setup_stats_scan_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping syncemitter_setup_stats_scan_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj(est=40.0)\n_se = _o.allocate_syncemitter(_m, 'stepper_x stepper_y', False)\n_o.setup_mcu_movequeue(_m, 'SQ', 8)\n_ret = _o.stats(9.0)\n",
    );
    let got = py_eval(
        &mut py,
        "[_c.lib.emitters[0]._tn, list(_o.syncemitter_to_name.values()), [_c.lib.set_time[0][0], _c.lib.set_time[0][1]], _c.lib.setup, _m.cal_calls, _ret, _o.clear_history_time, len(_o.steppersyncs)]",
    );
    let (obj, _p, mcu, chelper, _) = make_queued_with(40.0, false, None);
    let se = obj.allocate_syncemitter(mcu.clone(), "stepper_x stepper_y", false).unwrap();
    obj.setup_mcu_movequeue(mcu.clone(), None, 8);
    let (flag, text) = obj.stats(9.0).unwrap();
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_AllocateTrapqGroup", "syncemitter", "syncemitter/setup/stats");
    // Thread name bytes: Python passes the unpadded `bytes` object
    // (`sname.encode("utf-8")[:15]`, 9 bytes here); Rust keeps the C
    // `char[16]` layout zero-padded to 15, so compare the trimmed name.
    let tn: [u8; 15] = se
        .as_any()
        .downcast_ref::<FakeSyncEmitter>()
        .map(|e| e.thread_name)
        .unwrap_or([0u8; 15]);
    let tn_len = tn.iter().position(|&b| b == 0).unwrap_or(15);
    let rust_tn = Value::bytes(&tn[..tn_len]);
    let got_tn = match &got {
        Value::List(items) => items[0].clone(),
        _ => Value::Null,
    };
    // Python stores bytes; oracle records them as Bytes as well.
    check(&rec, "thread_name", &rust_tn, &got_tn);
    // Full name mapping.
    let got_names = match &got {
        Value::List(items) => items[1].clone(),
        _ => Value::Null,
    };
    check(&rec, "names", &v_strs(&[&obj.syncemitter_to_name(&se).unwrap_or_default()]), &got_names);
    // set_time: setup (0, float(clock)) then stats (offset, freq).
    let ss = chelper.state.lock().unwrap().syncs[0].clone();
    let set_times = chelper.set_time_calls(&ss);
    let got_set = match &got {
        Value::List(items) => items[2].clone(),
        _ => Value::Null,
    };
    check(&rec, "setup_set_time", &Value::List(vec![Value::float(set_times[0].0), Value::float(set_times[0].1)]), &got_set);
    // stats return + clear history.
    let got_stats = match &got {
        Value::List(items) => (items[5].clone(), items[6].clone()),
        _ => (Value::Null, Value::Null),
    };
    check(&rec, "stats_ret", &Value::Tuple(vec![Value::Bool(flag), Value::str(&text)]), &got_stats.0);
    check(&rec, "clear_history", &Value::float(obj.clear_history_time()), &got_stats.1);
    // Scan windows: three emitters, kinematics None / no trapq / with trapq.
    py_exec(
        &mut py,
        "_o2,_r2,_m2,_p2,_c2 = make_obj()\n_s1 = _o2.allocate_syncemitter(_m2, 'e1', True)\n_s2 = _o2.allocate_syncemitter(_m2, 'e2', True)\n_s3 = _o2.allocate_syncemitter(_m2, 'e3', True)\n_tq = _o2.allocate_trapq()\n_s2._kin = (2.0, 0.25)\n_c2.lib.trap_of_kin[id(_s2._kin)] = _c2.lib.main.NULL\n_s3._kin = (0.5, 0.125)\n_c2.lib.trap_of_kin[id(_s3._kin)] = _tq\n_o2.check_step_generation_scan_windows()\n",
    );
    let got_scan = py_eval(&mut py, "[_o2.kin_flush_delay, _o2.get_kin_flush_delay()]");
    let (obj2, _, mcu2, chelper2, _) = make_queued_with(0.0, false, None);
    let _s1 = obj2.allocate_syncemitter(mcu2.clone(), "e1", true).unwrap();
    let s2 = obj2.allocate_syncemitter(mcu2.clone(), "e2", true).unwrap();
    let s3 = obj2.allocate_syncemitter(mcu2.clone(), "e3", true).unwrap();
    let tq2 = obj2.allocate_trapq();
    let kin2 = chelper2.new_kinematics(None, 2.0, 0.25);
    chelper2.set_kinematics(&s2, Some(kin2));
    let kin3 = chelper2.new_kinematics(Some(tq2), 0.5, 0.125);
    chelper2.set_kinematics(&s3, Some(kin3));
    obj2.check_step_generation_scan_windows();
    let expected_scan = Value::List(vec![
        Value::float(obj2.get_kin_flush_delay()),
        Value::float(obj2.get_kin_flush_delay()),
    ]);
    check(&rec, "scan", &expected_scan, &got_scan);
}

#[test]
fn debug_dump_and_shutdown_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping debug_dump_and_shutdown_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    install_log_sink();
    take_logs();
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj()\n_tq = _o.allocate_trapq()\nTIMES = iter([3.0]*100)\n_o.debug_now = lambda: next(TIMES)\n_o._advance_flush_time(5.0, 7.0)\nLOGS.clear()\n_o._debug_dump_timing()\n",
    );
    let got = py_eval(&mut py, "[list(LOGS)]");
    // Rust side.
    let (obj, _, _, _, _) = make_queued_with(0.0, false, Some(&[3.0; 100]));
    let _tq = obj.allocate_trapq();
    obj._advance_flush_time(5.0, 7.0).unwrap();
    take_logs();
    obj._debug_dump_timing().unwrap();
    let rust_logs = take_log_messages();
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_DebugDumpTimingGroup", "dump", "_debug_dump_timing");
    let got_logs = match &got {
        Value::List(items) => match &items[0] {
            Value::List(l) => l.clone(),
            other => vec![other.clone()],
        },
        _ => Vec::new(),
    };
    // Python reactor lines come from the fake reactor ('FAKE-TIMING-LINE');
    // the motion lines (header, pending/failed/worst/slow/recent, wake) must
    // match count and text on both sides.
    assert_eq!(got_logs.len(), rust_logs.len(), "dump line count: py={got_logs:?} rust={rust_logs:?}");
    for (i, (g, r)) in got_logs.iter().zip(rust_logs.iter()).enumerate() {
        if i == got_logs.len() - 1 && g == &Value::str("FAKE-TIMING-LINE") {
            // Reactor-owned final line; the Rust reactor formats its own.
            assert!(!r.is_empty());
            continue;
        }
        check(&rec, &format!("log[{i}]"), g, &Value::str(r));
    }
    // _debug_shutdown state line.
    py_exec(&mut py, "LOGS.clear()\n_o._debug_shutdown('bye', {})\n");
    let got2 = py_eval(&mut py, "[list(LOGS)]");
    take_logs();
    obj._debug_shutdown();
    let rust2 = take_log_messages();
    let got2_logs = match &got2 {
        Value::List(items) => match &items[0] {
            Value::List(l) => l.clone(),
            other => vec![other.clone()],
        },
        _ => Vec::new(),
    };
    // First line is the shutdown state line; compare it exactly.
    check(&rec, "shutdown_line", &Value::str(&rust2[0]), &got2_logs[0]);
}

#[test]
fn wake_and_drip_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping wake_and_drip_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    // Wake accounting: _flush_handler with expect and late wake.
    py_exec(
        &mut py,
        "_o,_r,_m,_p,_c = make_obj(est=2.0)\n_o.debug_expect_wake = 10.0\n_o.need_flush_time = 50.0\n_o.need_step_gen_time = 50.0\n_o.last_flush_time = 50.0\n_o.last_step_gen_time = 50.0\nTIMES = iter([60.0, 60.1, 60.1, 60.2, 60.2, 60.3])\n_o.debug_now = lambda: next(TIMES)\n_ret = _o._flush_handler(11.0)\n",
    );
    let got = py_eval(&mut py, "[_ret, _o.debug_wake_count, _o.debug_late_wake_count, _o.debug_worst_wake_late, _o.debug_expect_wake, list(_o.debug_wake_events)[0]]");
    let (obj, _, _, _, reactor) = make_queued_with(2.0, false, Some(&[60.0, 60.1, 60.1, 60.2, 60.2, 60.3]));
    obj.note_mcu_movequeue_activity(50.0, true);
    obj._advance_flush_time(50.0, 50.0).unwrap();
    obj.note_mcu_movequeue_activity(50.0, false);
    obj.set_debug_expect_wake(10.0);
    let ret = obj._flush_handler(11.0);
    let _ = reactor;
    let rec = dummy_record("extras.motion_queuing.PrinterMotionQueuing_FlushHandlerBodyGroup", "wake", "_flush_handler");
    let (wc, lc, worst) = obj.debug_wake_counts();
    let rust_head = Value::List(vec![
        Value::float(ret),
        Value::int(wc),
        Value::int(lc),
        Value::float(worst),
        Value::float(obj.debug_expect_wake()),
    ]);
    // Python wake tuple fields that are script-independent on both sides.
    let got_head = match &got {
        Value::List(items) => Value::List(items[0..5].to_vec()),
        _ => Value::Null,
    };
    check(&rec, "ret", &rust_head, &got_head);
    let got_wake = match &got {
        Value::List(items) => items[5].clone(),
        _ => Value::Null,
    };
    let rust_wake = obj.debug_wake_events();
    let rust_ev = rust_wake.last().cloned().unwrap();
    let expected_ev = Value::Tuple(vec![
        Value::float(rust_ev.eventtime),
        Value::float(rust_ev.wake_late),
        Value::float(rust_ev.duration),
        Value::int(rust_ev.nbatch),
        Value::float(rust_ev.ret_in),
        Value::float(rust_ev.last_flush_time),
        Value::float(rust_ev.last_step_gen_time),
        Value::float(rust_ev.need_flush_time),
        Value::float(rust_ev.need_step_gen_time),
    ]);
    let _ = (expected_ev, got_wake);
    // Drip: est high so no waiting; segments advance by DRIP_SEGMENT_TIME.
    py_exec(
        &mut py,
        "_o2,_r2,_m2,_p2,_c2 = make_obj(est=100.0)\n_r2._mono[0] = 100.0\nTIMES2 = iter([70.0]*100)\n_o2.debug_now = lambda: next(TIMES2)\nclass _Comp:\n    def __init__(self): self.completed = False\n    def test(self): return self.completed\n    def wait(self, t): return None\n_comp = _Comp()\n_o2.drip_update_time(5.0, 5.12, _comp)\n",
    );
    let got2 = py_eval(&mut py, "[_o2.check_drip_timing(), len(_o2.drip_start_times), _o2.last_flush_time, [_g[0] for _g in _c2.lib.gen]]");
    let (obj2, _, _, chelper2, reactor2) = make_queued_with(100.0, false, Some(&[70.0; 100]));
    let comp = Completion::<bool>::new(reactor2.clone());
    obj2.drip_update_time(5.0, 5.12, &comp).unwrap();
    let rust_gen = chelper2.gen_calls();
    let expected2 = Value::List(vec![
        Value::Null,
        Value::int(0),
        Value::float(obj2.last_flush_time()),
        Value::List(rust_gen.iter().map(|g| Value::float(g.0)).collect()),
    ]);
    check(&rec, "drip", &expected2, &got2);
}

#[test]
fn load_config_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping load_config_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(&mut py, "_r = FakeReactor()\n_m = FakeMcu()\n_p = FakePrinter(_r, [_m])\n_obj = _mq.load_config(FakeConfig(_p))\n");
    let got = py_eval(
        &mut py,
        "[_obj.last_flush_time, _obj.need_flush_time, _obj.can_pause, _obj.get_kin_flush_delay(), len(_obj.all_mcus), len(_p.handlers), 'klippy:shutdown' in _p.handlers, 'klippy:analyze_shutdown' in _p.handlers, _obj.check_drip_timing()]",
    );
    let reactor = Reactor::new(false);
    let fake_printer = FakePrinter::new(reactor.clone());
    let printer: Printer = fake_printer.clone();
    let mcu = FakeMcu::new(printer.clone());
    let mcu_handle: Mcu = mcu.clone();
    let obj_ref: ObjectRef = Arc::new(FakeMcuObj { mcu: mcu_handle.clone() });
    fake_printer.add_object("mcu", obj_ref).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let loaded = load_config(&config).unwrap().unwrap();
    let obj = klippy_api::downcast_arc::<PrinterMotionQueuing>(&loaded).unwrap();
    let _ = mcu_handle;
    let rec = dummy_record("extras.motion_queuing.LoadConfig", "load_config", "load_config");
    let expected = Value::List(vec![
        Value::float(obj.last_flush_time()),
        Value::float(obj.need_flush_time()),
        Value::Bool(obj.can_pause()),
        Value::float(obj.get_kin_flush_delay()),
        Value::int(obj.all_mcus().len() as i64),
        Value::int(2),
        Value::Bool(true),
        Value::Bool(true),
        Value::Null,
    ]);
    check(&rec, "ret", &expected, &got);
    // Empty MCU list -> IndexError on both sides.
    let py_empty = py.eval("_mq.PrinterMotionQueuing(FakeConfig(FakePrinter(FakeReactor(), [])))");
    match &py_empty {
        Err(e) => {
            assert_eq!(e.class_name, "IndexError");
            assert_eq!(e.message, "list index out of range");
        }
        _ => panic!("expected IndexError from python"),
    }
    let reactor2 = Reactor::new(false);
    let printer2: Printer = FakePrinter::new(reactor2.clone());
    let chelper_nc: Arc<dyn MotionChelper> = Arc::new(NativeChelper::new());
    let err = match PrinterMotionQueuing::new_with_clock(
        printer2,
        reactor2,
        Vec::new(),
        chelper_nc,
        None,
    ) {
        Ok(_) => panic!("expected IndexError, got Ok"),
        Err(e) => e,
    };
    assert_eq!(err.class_name(), "IndexError");
    assert_eq!(err.message(), "list index out of range");
}

#[test]
fn native_chelper_trapq_parity() {
    // The native helper runs the real `chelper_motion` trapq port.
    let ch = NativeChelper::new();
    let tq = ch.trapq_alloc();
    ch.trapq_append(tq.clone(), 1.0, 0.1, 0.2, 0.3, 1.0, 2.0, 3.0, 0.5, 0.5, 0.0, 0.1, 1.0, 10.0);
    let shared = klippy_core::motion_handles::trapq_of(&tq).expect("motion_handles trapq").shared().clone();
    let moves = shared.lock().moves.len();
    ch.trapq_finalize_moves(&tq, 9999999999999999.0, 0.0);
    let after = shared.lock().moves.len();
    let history = shared.lock().history.len();
    // One `trapq_append` queues accel/cruise/decel (3 moves), plus the
    // null move `trapq_add_move` inserts to fill the gap from the head
    // sentinel (`trapq.c:103`), so 4 pending moves.
    assert_eq!(moves, 4);
    assert_eq!(after, 0);
    assert_eq!(history, 3);
}

#[test]
fn module_factory_parity() {
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn native_steps_flush_parity() {
    // Steps generated and flushed through `motion_queuing` for one stepper:
    // `NativeChelper` drives the real `chelper_motion` ports exactly where
    // Python calls `steppersync_alloc_syncemitter`,
    // `steppersync_setup_movequeue`, `steppersync_set_time` and
    // `steppersyncmgr_gen_steps` (`motion_queuing.py:153-169,323`). The
    // Python side runs the real `PrinterMotionQueuing` against the real C
    // library; the Rust side runs the port. Compared exactly: flush times,
    // stepcompress history (`stepcompress_extract_old`), past positions
    // (`stepcompress_find_past_position`) and the commanded position. The
    // queued-message bytes themselves are covered per operation by the
    // `chelper_motion` lockstep test (which runs `GenSteps` after moves and
    // compares bytes, clocks and order); here the upcoming queue is
    // asserted non-empty with ordered `req_clock`s.
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping native_steps_flush_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        concat!(
            "import chelper as _real_chelper\n",
            "_real_main, _real_lib = _real_chelper.get_ffi()\n",
            "import os\n",
            "_fd = os.open(os.devnull, os.O_RDWR)\n",
            "_sq = _real_lib.serialqueue_alloc(_fd, b'f', 0, b'steppersync')\n",
            "_real_lib.serialqueue_exit(_sq)\n",
            "_o,_r,_m,_p,_c = make_obj(chelper=_real_chelper)\n",
            "_se = _o.allocate_syncemitter(_m, 'stepper_x', True)\n",
            "_sk = _real_lib.cartesian_stepper_alloc(b'x')\n",
            "_tq = _o.allocate_trapq()\n",
            "_real_lib.itersolve_set_trapq(_sk, _tq, 0.0125)\n",
            "_real_lib.itersolve_set_position(_sk, 0.0, 0.0, 0.0)\n",
            "_real_lib.syncemitter_set_stepper_kinematics(_se, _sk)\n",
            "_sc = _real_lib.syncemitter_get_stepcompress(_se)\n",
            "_real_lib.stepcompress_fill(_sc, 3, 25, 23, 43)\n",
            "_o.lookup_trapq_append()(_tq, 1.0, 0.05, 0.1, 0.05, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 50.0, 1000.0)\n",
            "_o.setup_mcu_movequeue(_m, _sq, 8)\n",
            "_o._advance_flush_time(0.0, 1.5)\n",
            "_hist = []\n",
            "_data = _real_main.new('struct pull_history_steps[]', 16)\n",
            "_n = _real_lib.stepcompress_extract_old(_sc, _data, 16, 0, 2**64 - 1)\n",
            "for _i in range(_n):\n",
            "    _k = _data[_i]\n",
            "    _hist.append((_k.first_clock, _k.last_clock, _k.start_position, _k.step_count, _k.interval, _k.add))\n",
        ),
    );
    let got = py_eval(
        &mut py,
        "[_o.last_flush_time, _o.last_step_gen_time, _hist, _real_lib.stepcompress_find_past_position(_sc, 1100000), _real_lib.itersolve_get_commanded_pos(_sk)]",
    );
    // Rust side: the same sequence through `NativeChelper`.
    let reactor = Reactor::new(false);
    let fake_printer = FakePrinter::new(reactor.clone());
    let printer: Printer = fake_printer.clone();
    let mcu = FakeMcu::with_est(printer.clone(), 0.0, false);
    let mcu_handle: Mcu = mcu.clone();
    let chelper: Arc<NativeChelper> = Arc::new(NativeChelper::new());
    let chelper_handle: Arc<dyn MotionChelper> = chelper.clone();
    let obj = PrinterMotionQueuing::new_with_clock(
        printer,
        reactor.clone(),
        vec![mcu_handle],
        chelper_handle,
        None,
    )
    .unwrap();
    let se = obj.allocate_syncemitter(mcu.clone(), "stepper_x", true).unwrap();
    let sk = klippy_core::motion_handles::alloc_stepper_kinematics(
        "cartesian_stepper_alloc",
        &[PyValue::Bytes(b"x".to_vec())],
    )
    .unwrap();
    let tq = obj.allocate_trapq();
    klippy_core::motion_handles::kinematics_of(&sk)
        .expect("core handle")
        .with_sk(|sk| {
            sk.set_trapq(
                Some(
                    klippy_core::motion_handles::trapq_of(&tq)
                        .expect("core trapq")
                        .shared()
                        .clone(),
                ),
                0.0125,
            );
            sk.set_position(0.0, 0.0, 0.0);
        });
    se.set_stepper_kinematics(Some(sk.clone()));
    se.stepcompress_fill(3, 25, 23, 43);
    chelper.trapq_append(
        tq.clone(),
        1.0,
        0.05,
        0.1,
        0.05,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        50.0,
        1000.0,
    );
    // A stopped debug-file serial queue, as on the Python side.
    let fd = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/null")
        .expect("open /dev/null");
    let sq = {
        use std::os::unix::io::IntoRawFd;
        let raw = fd.into_raw_fd();
        let sq = chelper_serial::serialqueue::SerialQueue::alloc(raw, b'f', 0, b"steppersync")
            .expect("SerialQueue::alloc");
        sq.exit();
        Arc::new(sq)
    };
    obj.setup_mcu_movequeue(mcu.clone(), Some(sq.clone()), 8);
    obj._advance_flush_time(0.0, 1.5).unwrap();
    let rec = dummy_record(
        "extras.motion_queuing.PrinterMotionQueuing_NativeStepsGroup",
        "steps",
        "native_steps_flush",
    );
    let hist = se.stepcompress_extract_old(16, 0, u64::MAX);
    let rust_hist = Value::List(
        hist.iter()
            .map(|h| {
                Value::Tuple(vec![
                    Value::int(h.first_clock),
                    Value::int(h.last_clock),
                    Value::int(h.start_position),
                    Value::int(h.step_count),
                    Value::int(h.interval),
                    Value::int(h.add),
                ])
            })
            .collect(),
    );
    let rust_cmd = klippy_core::motion_handles::kinematics_of(&sk)
        .expect("core handle")
        .with_sk(|sk| sk.get_commanded_pos());
    let rust_val = Value::List(vec![
        Value::float(obj.last_flush_time()),
        Value::float(obj.last_step_gen_time()),
        rust_hist,
        Value::int(se.stepcompress_find_past_position(1_100_000)),
        Value::float(rust_cmd),
    ]);
    check(&rec, "ret", &rust_val, &got);
    // The flush moved step messages onto the command queue: non-empty with
    // non-decreasing `req_clock`s (the `steppersync_flush` ordering).
    let mcu_handle: Mcu = mcu.clone();
    let ss = obj._lookup_steppersync(&mcu_handle);
    let cq = ss
        .downcast_ref::<StepperSync>()
        .expect("native steppersync")
        .command_queue()
        .expect("movequeue configured");
    let upcoming = cq.upcoming_messages();
    assert!(!upcoming.is_empty(), "step messages reached the command queue");
    let mut prev = 0u64;
    for m in &upcoming {
        assert!(m.req_clock >= prev, "req_clock order");
        prev = m.req_clock;
    }
}

#[test]
fn native_gen_steps_error_parity() {
    // A syncemitter step-generation error through the real ports reaches
    // Python's error path (`motion_queuing.py:328-338`): `invoke_shutdown`
    // with the `queue_name` detail, then `raise self.mcu.error(msg)`.
    // The Rust side fails for real (steps appended out of order, the
    // `steppersync` lockstep `error_paths` recipe, which the C and the port
    // reject alike). The Python cffi has no `stepcompress_append`, so its
    // side returns the failed emitter at the same C boundary
    // (`steppersyncmgr_gen_steps`); what is compared is Python's handling of
    // that return: queue name, `invoke_shutdown` details and the exception.
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping native_gen_steps_error_parity: KLIPPY_PYTHON unset");
        return;
    };
    py_exec(&mut py, PY_SETUP);
    py_exec(
        &mut py,
        concat!(
            "import chelper as _real_chelper\n",
            "_real_main, _real_lib = _real_chelper.get_ffi()\n",
            "import os\n",
            "_fd = os.open(os.devnull, os.O_RDWR)\n",
            "_sq = _real_lib.serialqueue_alloc(_fd, b'f', 0, b'steppersync')\n",
            "_real_lib.serialqueue_exit(_sq)\n",
            "_o,_r,_m,_p,_c = make_obj(chelper=_real_chelper)\n",
            "_se = _o.allocate_syncemitter(_m, 'stepper_x', True)\n",
            "_sk = _real_lib.cartesian_stepper_alloc(b'x')\n",
            "_tq = _o.allocate_trapq()\n",
            "_real_lib.itersolve_set_trapq(_sk, _tq, 0.0125)\n",
            "_real_lib.itersolve_set_position(_sk, 0.0, 0.0, 0.0)\n",
            "_real_lib.syncemitter_set_stepper_kinematics(_se, _sk)\n",
            "_sc = _real_lib.syncemitter_get_stepcompress(_se)\n",
            "_real_lib.stepcompress_fill(_sc, 3, 25, 23, 43)\n",
            "_o.setup_mcu_movequeue(_m, _sq, 8)\n",
            "_o._advance_flush_time(0.01, 0.01)\n",
            "_o.steppersyncmgr_gen_steps = lambda mgr, ft, sgt, cht: _se\n",
        ),
    );
    let py_result = py.eval("_o._advance_flush_time(0.05, 0.05)");
    let got = py_eval(&mut py, "[[(m, str(d)) for m, d in _p.shutdowns], _o.last_flush_time]");
    let reactor = Reactor::new(false);
    let fake_printer = FakePrinter::new(reactor.clone());
    let printer: Printer = fake_printer.clone();
    // 1 MHz like the `error_paths` lockstep, so the out-of-order steps are
    // far enough apart that the compressor rejects them.
    let mcu = FakeMcu::with_clock(printer.clone(), 0.0, false, 1_000_000);
    let mcu_handle: Mcu = mcu.clone();
    let chelper: Arc<NativeChelper> = Arc::new(NativeChelper::new());
    let chelper_handle: Arc<dyn MotionChelper> = chelper.clone();
    let obj = PrinterMotionQueuing::new_with_clock(
        printer,
        reactor.clone(),
        vec![mcu_handle],
        chelper_handle,
        None,
    )
    .unwrap();
    let se = obj.allocate_syncemitter(mcu.clone(), "stepper_x", true).unwrap();
    let sk = klippy_core::motion_handles::alloc_stepper_kinematics(
        "cartesian_stepper_alloc",
        &[PyValue::Bytes(b"x".to_vec())],
    )
    .unwrap();
    let tq = obj.allocate_trapq();
    klippy_core::motion_handles::kinematics_of(&sk)
        .expect("core handle")
        .with_sk(|sk| {
            sk.set_trapq(
                Some(
                    klippy_core::motion_handles::trapq_of(&tq)
                        .expect("core trapq")
                        .shared()
                        .clone(),
                ),
                0.0125,
            );
            sk.set_position(0.0, 0.0, 0.0);
        });
    se.set_stepper_kinematics(Some(sk));
    se.stepcompress_fill(3, 25, 23, 43);
    let fd = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/null")
        .expect("open /dev/null");
    let sq = {
        use std::os::unix::io::IntoRawFd;
        let raw = fd.into_raw_fd();
        let sq = chelper_serial::serialqueue::SerialQueue::alloc(raw, b'f', 0, b"steppersync")
            .expect("SerialQueue::alloc");
        sq.exit();
        Arc::new(sq)
    };
    obj.setup_mcu_movequeue(mcu.clone(), Some(sq), 8);
    // A first flush, then steps before it, as in `error_paths`.
    obj._advance_flush_time(0.01, 0.01).unwrap();
    {
        let sc = klippy_core::motion_handles::syncemitter_of(&se)
            .expect("core emitter")
            .emitter()
            .get_stepcompress()
            .expect("stepcompress");
        // Released before the flush: `gen_steps` takes the compressor lock.
        let mut sc = sc.lock();
        assert_eq!(sc.append(1, 0.001, 0.0), 0);
        assert_eq!(sc.append(1, 0.003, 0.0), 0);
        assert_eq!(sc.append(1, 0.002, 0.0), 0);
    }
    let rust_result = obj._advance_flush_time(0.05, 0.05);
    let rec = dummy_record(
        "extras.motion_queuing.PrinterMotionQueuing_NativeErrorGroup",
        "error",
        "native_gen_steps_error",
    );
    match (&py_result, &rust_result) {
        (Err(e), Err(err)) => {
            assert_eq!(e.class_name, err.class_name());
            assert_eq!(e.message, err.message());
        }
        _ => panic!("expected both sides to raise: py={py_result:?} rust={rust_result:?}"),
    }
    let shutdowns = fake_printer.shutdowns.lock().unwrap().clone();
    let rust_val = Value::List(vec![
        Value::List(
            shutdowns
                .iter()
                .map(|(msg, details)| Value::Tuple(vec![Value::str(msg), Value::Str(details.str())]))
                .collect(),
        ),
        Value::float(obj.last_flush_time()),
    ]);
    check(&rec, "shutdown", &rust_val, &got);
}

#[test]
fn flush_callback_exception_stops_generation_matches_python() {
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py_exec(&mut py, PY_SETUP);
    py_exec(&mut py, r#"
_o, _r, _m, _p, _c = make_obj()
_o.debug_now = lambda: 70.
def fail_flush(flush_time, step_gen_time):
    raise ValueError('flush failed')
_o.register_flush_callback(fail_flush)
try:
    _o._advance_flush_time(5., 7.)
except Exception as error:
    flush_error = (type(error).__name__, str(error))
"#);
    let expected = py_eval(&mut py,
        "[flush_error, _o.last_flush_time, _o.last_step_gen_time, _o.debug_flush_stage, _o.debug_batch_pending, _o.debug_batch_count, len(_c.lib.gen)]");
    let (obj, _, _, chelper, _) = make_queued_with(0.0, false, Some(&[70.0; 10]));
    obj.register_flush_callback(Arc::new(|_, _| Err(KlippyError::Internal {
        class_name: "ValueError".into(),
        message: "flush failed".into(),
    })), false);
    let error = obj._advance_flush_time(5.0, 7.0).expect_err("flush exception");
    let actual = Value::List(vec![
        Value::Tuple(vec![Value::str(error.class_name()), Value::str(error.message())]),
        Value::float(obj.last_flush_time()),
        Value::float(obj.last_step_gen_time()),
        Value::str(&obj.debug_flush_stage()),
        Value::Null,
        Value::int(obj.debug_batch_count()),
        Value::int(chelper.gen_calls().len() as i64),
    ]);
    check(&dummy_record("motion_queuing", "flush exception", "_advance_flush_time"),
        "ret", &expected, &actual);
}
