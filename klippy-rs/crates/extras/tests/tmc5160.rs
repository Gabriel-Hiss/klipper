//! Parity tests for `klippy/extras/tmc5160.py`.
//!
//! All expected values come live from the frozen reference through
//! the `pyoracle` crate (`$KLIPPY_PYTHON` is the CPython 3.8.2
//! binary; the test skips when it is unset): for each case the
//! oracle builds the reference object with minimal Python fakes
//! for the printer, config and MCU in `exec`, calls it with
//! `eval`, and the port is driven through the same inputs via the
//! fakes below and compared with `recording::assert_equal`.
//!
//! Compared against the oracle:
//! - `Registers`, `ReadRegisters`, `Fields`, `TMC_FREQUENCY` tables;
//! - `tmc.FieldHelper` operations driven on the `tmc5160` tables
//!   (`lookup_register`, `get_field`, `set_field` / register values,
//!   `set_config_field`, `pretty_format`, `get_reg_fields`, signed-field
//!   handling, `KeyError` for unknown fields);
//! - `TMC5160CurrentHelper` config reads (`run_current`, `hold_current`,
//!   `sense_resistor` with their defaults and bounds), `_calc_current`,
//!   `get_current`, `set_current` (the `GLOBALSCALER` / `IHOLD_IRUN`
//!   register writes and `req_hold_current` update), over a grid of
//!   currents / sense resistors;
//! - the `TMC5160.__init__` `set_config_field` defaults
//!   (`tmc5160.py:345-386`) applied through the reference
//!   `FieldHelper.set_config_field` and through `Tmc5160::assemble`
//!   (which also runs the `tmc` helpers the Python `__init__` runs);
//! - `get_phase_offset` and the four `get_status` keys the reference
//!   command helper writes (`tmc5160.py:337-338`);
//! - error paths (missing `run_current`, `above` / `maxval` violations).

// Test fakes are plain synchronised bags of state, not printer objects.
#![allow(clippy::disallowed_types, clippy::type_complexity)]

use std::sync::{Arc, Mutex};

use extras::stepper_enable::PrinterStepperEnableApi;
use extras::tmc::{
    ConfigFieldDefault, FieldHelper, McuTmcApi, RegisterRaw, TMCCommandHelperApi,
};
use extras::tmc5160::{
    TMC_FREQUENCY, Tmc5160, Tmc5160CurrentHelper, fields_table, make_field_helper,
    module_factory, registers_table, READ_REGISTERS,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListsOpts,
    GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandWrapper, CommandQueryWrapper,
    ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, McuApi, ParsePinOpts, Pin,
    PinParams, PinResolver, PinsApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, PrinterApi};
use klippy_api::stepper::Stepper;
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{downcast, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::value::Value as Rv;
use recording::{Record, RecordKind, SideEffects};

// ---------------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            objects: Mutex::new(IndexMap::new()),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
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
        let mut m = self.objects.lock().unwrap();
        if m.contains_key(name) {
            return Err(KlippyError::config(format!("Duplicate name '{name}'")));
        }
        m.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let m = self.objects.lock().unwrap();
        match m.get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.lock().unwrap().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let m = self.objects.lock().unwrap();
        match m.get(section) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown object '{section}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[klippy_api::object::EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

fn unimpl<T>(name: &str) -> T {
    panic!("fake should not be reached at {name}")
}

struct FakePins;

impl PinsApi for FakePins {
    fn parse_pin(&self, name: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        unimpl(name)
    }
    fn lookup_pin(&self, name: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        unimpl(name)
    }
    fn setup_pin(&self, name: &str, _pin_desc: &str) -> KResult<Pin> {
        unimpl(name)
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, name: &str) -> KResult<PinResolver> {
        unimpl(name)
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, name: &str) -> KResult<()> {
        unimpl(name)
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeGCode {
    mux: Mutex<Vec<(String, String, Option<String>)>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        name: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        unimpl(name)
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
            .push((cmd.to_string(), key.to_string(), value.map(|v| v.to_string())));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, name: &str) -> KResult<()> {
        unimpl(name)
    }
    fn run_script(&self, name: &str) -> KResult<()> {
        unimpl(name)
    }
    fn run_script_from_command(&self, name: &str) -> KResult<()> {
        unimpl(name)
    }
    fn get_mutex(&self) -> ReactorMutex {
        unimpl("get_mutex")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        unimpl("create_gcode_command")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        unimpl("cmd_m112")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        unimpl("process_commands")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeStepperEnable;

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        unimpl("register_stepper")
    }
    fn set_motors_enable(
        &self,
        _stepper_names: &[String],
        _enable: bool,
    ) -> KResult<bool> {
        unimpl("set_motors_enable")
    }
    fn motor_off(&self) -> KResult<()> {
        unimpl("motor_off")
    }
    fn lookup_enable(
        &self,
        name: &str,
    ) -> KResult<Arc<extras::stepper_enable::EnableTracking>> {
        unimpl(name)
    }
    fn get_steppers(&self) -> Vec<String> {
        Vec::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
}

klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

struct FakeWebhooks;

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, name: &str, _callback: WebHooksCallback) -> KResult<()> {
        unimpl(name)
    }
    fn register_mux_endpoint(
        &self,
        _path: &str,
        _key: &str,
        _value: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn call_remote_method(
        &self,
        name: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        unimpl(name)
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

struct FakeMcu;
impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        unimpl("get_printer")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        unimpl("setup_pin")
    }
    fn create_oid(&self) -> KResult<i64> {
        unimpl("create_oid")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        unimpl("register_config_callback")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        unimpl("add_config_cmd")
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        unimpl("get_query_slot")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        unimpl("seconds_to_clock")
    }
    fn min_schedule_time(&self) -> f64 {
        unimpl("min_schedule_time")
    }
    fn max_nominal_duration(&self) -> f64 {
        unimpl("max_nominal_duration")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<CommandWrapper> {
        unimpl("lookup_command")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        unimpl("lookup_query_command")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        unimpl("alloc_command_queue")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msgformat: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        unimpl("register_serial_response")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        unimpl("check_valid_response")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        unimpl("get_constant_float")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        unimpl("print_time_to_clock")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        unimpl("clock_to_print_time")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        unimpl("estimated_print_time")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        unimpl("clock32_to_clock64")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        unimpl("calibrate_clock")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        unimpl("get_status")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        unimpl("stats")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        unimpl("create_trigger_dispatch")
    }
}


struct FakeConfig {
    printer: Printer,
    name: String,
    floats: IndexMap<String, f64>,
    ints: IndexMap<String, i64>,
    bools: IndexMap<String, bool>,
    strings: IndexMap<String, String>,
    reads: Arc<Mutex<Vec<String>>>,
}

fn fmt_f64(v: f64) -> String {
    pycompat::text::float_str(v)
}

impl FakeConfig {
    fn new(printer: Printer, name: &str) -> Self {
        FakeConfig {
            printer,
            name: name.to_string(),
            floats: IndexMap::new(),
            ints: IndexMap::new(),
            bools: IndexMap::new(),
            strings: IndexMap::new(),
            reads: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn with_floats(mut self, items: &[(&str, f64)]) -> Self {
        for (k, v) in items {
            self.floats.insert(k.to_string(), *v);
        }
        self
    }
    fn with_ints(mut self, items: &[(&str, i64)]) -> Self {
        for (k, v) in items {
            self.ints.insert(k.to_string(), *v);
        }
        self
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.reads.lock().unwrap().push(format!("get:{option}"));
        match self.strings.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => Ok(opts.default.flatten()),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.reads.lock().unwrap().push(format!("getint:{option}"));
        let v = match self.ints.get(option) {
            Some(v) => *v,
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    ))),
                    Some(d) => Ok(d),
                };
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {minval}",
                    self.name
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {maxval}",
                    self.name
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads.lock().unwrap().push(format!("getfloat:{option}"));
        let v = match self.floats.get(option) {
            Some(v) => *v,
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    ))),
                    Some(d) => Ok(d),
                };
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    fmt_f64(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    fmt_f64(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    fmt_f64(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    fmt_f64(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.reads.lock().unwrap().push(format!("getboolean:{option}"));
        match self.bools.get(option) {
            Some(v) => Ok(Some(*v)),
            None => Ok(opts.default.flatten()),
        }
    }
    fn getchoice(&self, option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        self.reads.lock().unwrap().push(format!("getchoice:{option}"));
        match self.ints.get(option) {
            Some(v) => Ok(PyValue::Int(pycompat::PyInt::from_i64(*v))),
            None => Ok(PyValue::None),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, section: &str) -> Config {
        // `stepper_y` provides `microsteps: 256` (`mres 0`) for the
        // microstep helper; `interpolate` defaults to True.
        Arc::new(
            FakeConfig::new(self.printer.clone(), section)
                .with_ints(&[("microsteps", 0)]),
        )
    }
    fn has_section(&self, section: &str) -> bool {
        section == "stepper_y"
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

/// Builds a printer with the objects the `tmc` helpers look up.
fn make_printer() -> (Printer, Arc<FakeGCode>, Arc<FakeWebhooks>) {
    let printer: Printer = FakePrinter::new();
    let gcode: Arc<FakeGCode> = Arc::new(FakeGCode { mux: Mutex::new(Vec::new()) });
    let webhooks: Arc<FakeWebhooks> = Arc::new(FakeWebhooks);
    printer.add_object("pins", Arc::new(FakePins)).expect("pins");
    printer
        .add_object("gcode", gcode.clone())
        .expect("gcode");
    printer
        .add_object("stepper_enable", Arc::new(FakeStepperEnable))
        .expect("stepper_enable");
    printer
        .add_object("webhooks", webhooks.clone())
        .expect("webhooks");
    (printer, gcode, webhooks)
}

fn make_config(items: &[(&str, f64)]) -> (Printer, Config, Arc<Mutex<Vec<String>>>) {
    let (printer, _gcode, _webhooks) = make_printer();
    let cfg = FakeConfig::new(printer.clone(), "tmc5160 stepper_y").with_floats(items);
    let reads = cfg.reads.clone();
    let config: Config = Arc::new(cfg);
    (printer, config, reads)
}

struct FakeMcuTmc {
    fields: Arc<FieldHelper>,
    writes: Arc<Mutex<Vec<(String, i64, Option<f64>)>>>,
    regs: IndexMap<String, i64>,
    mcu: klippy_api::mcu::Mcu,
}

impl FakeMcuTmc {
    fn new(fields: Arc<FieldHelper>) -> (Self, Arc<Mutex<Vec<(String, i64, Option<f64>)>>>) {
        let writes = Arc::new(Mutex::new(Vec::new()));
        let regs = registers_table();
        let fake = FakeMcuTmc {
            fields,
            writes: writes.clone(),
            regs,
            mcu: Arc::new(FakeMcu),
        };
        (fake, writes)
    }
}

impl McuTmcApi for FakeMcuTmc {
    fn get_fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }
    fn get_register(&self, reg_name: &str) -> KResult<i64> {
        Ok(self.fields.get_register_value(reg_name).unwrap_or(0))
    }
    fn get_register_raw(&self, reg_name: &str) -> KResult<RegisterRaw> {
        Ok(RegisterRaw {
            data: self.fields.get_register_value(reg_name).unwrap_or(0),
            receive_time: 0.0,
            spi_status: None,
        })
    }
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        self.writes.lock().unwrap().push((reg_name.to_string(), val, print_time));
        Ok(())
    }
    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(TMC_FREQUENCY)
    }
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        self.mcu.clone()
    }
    fn name_to_reg_value(&self, reg_name: &str) -> Option<i64> {
        self.regs.get(reg_name).copied()
    }
}

fn make_mcu() -> (Arc<dyn McuTmcApi>, Arc<FieldHelper>, Arc<Mutex<Vec<(String, i64, Option<f64>)>>>) {
    let fields = make_field_helper();
    let (fake, writes) = FakeMcuTmc::new(fields.clone());
    (Arc::new(fake), fields, writes)
}

// ---------------------------------------------------------------------------
// Oracle plumbing
// ---------------------------------------------------------------------------

fn record_for(function: &str, ret: Rv) -> Record {
    Record {
        v: 1,
        unit: "extras.tmc5160".to_string(),
        scenario: "oracle".to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: Some(ret),
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn assert_val(function: &str, path: &str, expected: Rv, actual: Rv) {
    let rec = record_for(function, expected.clone());
    if let Err(d) = recording::assert_equal(&rec, path, &expected, &actual) {
        panic!("parity mismatch ({d})");
    }
}

fn py_float(x: f64) -> Rv {
    Rv::float(x)
}

fn oracle_tuple(v: &Rv) -> Vec<Rv> {
    match v {
        Rv::Tuple(items) => items.clone(),
        Rv::List(items) => items.clone(),
        other => panic!("expected tuple/list, got {other:?}"),
    }
}

fn oracle_int(v: &Rv) -> i64 {
    match v {
        Rv::Int(i) => *i,
        Rv::BigInt(s) => s.parse().unwrap_or_else(|_| panic!("bigint {s}")),
        other => panic!("expected int, got {other:?}"),
    }
}

fn oracle_str(v: &Rv) -> String {
    match v {
        Rv::Str(s) => s.clone(),
        other => panic!("expected str, got {other:?}"),
    }
}

/// Port-side value as a comparable recording value.
fn rv_of(v: &PyValue) -> Rv {
    match v {
        PyValue::None => Rv::Null,
        PyValue::Bool(b) => Rv::Bool(*b),
        PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
            Ok(v) => Rv::Int(v),
            Err(_) => Rv::BigInt(n.inner().to_string()),
        },
        PyValue::Float(f) => py_float(*f),
        PyValue::Str(s) => Rv::Str(s.clone()),
        PyValue::List(items) => Rv::List(items.iter().map(rv_of).collect()),
        PyValue::Tuple(items) => Rv::Tuple(items.iter().map(rv_of).collect()),
        PyValue::Dict(pairs) => {
            Rv::Dict(pairs.iter().map(|(k, v)| (rv_of(k), rv_of(v))).collect())
        }
        other => panic!("cannot compare {other:?}"),
    }
}

/// Python fake-config preamble for oracle-side helpers.
const PY_FAKES: &str = r#"
import math as _math
_sentinel = object()
class _FakeConfig:
    def __init__(self, name, opts):
        self._name = name
        self._opts = dict(opts)
    def get_printer(self):
        return None
    def get_name(self):
        return self._name
    def _wrap(self, option, default, minval, maxval, above, below, conv):
        if option not in self._opts:
            if default is not _sentinel:
                return default
            raise Exception("Option '%s' in section '%s' must be specified" % (option, self._name))
        v = conv(self._opts[option])
        if minval is not None and v < minval:
            raise Exception("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise Exception("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise Exception("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise Exception("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        return self._wrap(option, default, minval, maxval, above, below, float)
    def getint(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        return self._wrap(option, default, minval, maxval, above, below, int)
    def getboolean(self, option, default=_sentinel, note_valid=True):
        if option not in self._opts:
            if default is not _sentinel:
                return default
            raise Exception("Option '%s' in section '%s' must be specified" % (option, self._name))
        return self._opts[option]
    def get(self, option, default=_sentinel, note_valid=True):
        if option not in self._opts:
            if default is not _sentinel:
                return default
            raise Exception("Option '%s' in section '%s' must be specified" % (option, self._name))
        return self._opts[option]
class _FakeMcu:
    def __init__(self, fields):
        self._fields = fields
        self.writes = []
    def get_fields(self):
        return self._fields
    def set_register(self, reg_name, val, print_time=None):
        self.writes.append((reg_name, val, print_time))
    def get_tmc_frequency(self):
        return 12000000.
import extras.tmc as _tmc
import extras.tmc5160 as _m
_f = _tmc.FieldHelper(_m.Fields, _m.SignedFields, _m.FieldFormatters)
_fm = _FakeMcu(_f)
"#;

fn setup_oracle(py: &mut pyoracle::Oracle) {
    py.exec("import extras.tmc, extras.tmc5160").expect("import reference");
    py.exec(PY_FAKES).expect("fakes");
}

// Rust value builders for table comparisons.
fn rust_registers_value() -> Rv {
    Rv::Dict(
        registers_table()
            .iter()
            .map(|(k, v)| (Rv::Str(k.clone()), Rv::Int(*v)))
            .collect(),
    )
}

fn rust_fields_value() -> Rv {
    Rv::Dict(
        fields_table()
            .iter()
            .map(|(reg, fields)| {
                let inner: Vec<(Rv, Rv)> = fields
                    .iter()
                    .map(|(k, v)| (Rv::Str(k.clone()), Rv::Int(*v)))
                    .collect();
                (Rv::Str(reg.clone()), Rv::Dict(inner))
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn tables_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    setup_oracle(&mut py);
    assert_val("Registers", "", py.eval("_m.Registers").unwrap(), rust_registers_value());
    assert_val(
        "Fields",
        "",
        py.eval("_m.Fields").unwrap(),
        rust_fields_value(),
    );
    assert_val(
        "ReadRegisters",
        "",
        py.eval("_m.ReadRegisters").unwrap(),
        Rv::List(READ_REGISTERS.iter().map(|s| Rv::Str(s.to_string())).collect()),
    );
    assert_val(
        "TMC_FREQUENCY",
        "",
        py.eval("_m.TMC_FREQUENCY").unwrap(),
        py_float(TMC_FREQUENCY),
    );
}

#[test]
fn field_helper_ops_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    setup_oracle(&mut py);
    // set_field / get_field round trips on representative registers.
    let cases = [
        ("toff", 3i64),
        ("toff", 0),
        ("toff", 15),
        ("mres", 4),
        ("mres", 0),
        ("irun", 31),
        ("ihold", 7),
        ("globalscaler", 200),
        ("globalscaler", 0),
        ("semin", 0),
        ("semax", 15),
        ("sgt", 127),
        ("sgt", -64),
        ("pwm_scale_auto", 255),
        ("pwm_scale_auto", -256),
        ("multistep_filt", 1),
        ("version", 196),
        ("mres", 8),
        ("cs_actual", 16),
        ("tpowerdown", 10),
        ("lost_steps", 999),
    ];
    let helper = make_field_helper();
    for (field, value) in cases {
        py.exec(&format!(
            "_pv = _f.set_field({field:?}, {value}); _gv = _f.get_field({field:?})"
        ))
        .expect("oracle set/get");
        let pv = oracle_int(&py.eval("_pv").unwrap());
        let gv = oracle_int(&py.eval("_gv").unwrap());
        let rust_pv = helper.set_field(field, value, None, None).expect("rust set_field");
        let rust_gv = helper.get_field(field, None, None).expect("rust get_field");
        assert_val("set_field", "", Rv::Int(pv), Rv::Int(rust_pv));
        assert_val("get_field", "", Rv::Int(gv), Rv::Int(rust_gv));
    }
    // lookup_register for every field of every register.
    for (reg, fields) in fields_table() {
        for field in fields.keys() {
            py.exec(&format!("_lr = _f.lookup_register({field:?})")).expect("oracle lookup");
            let expected = py.eval("_lr").unwrap();
            let actual = helper_lookup(&reg, field);
            assert_val("lookup_register", "", expected, actual);
        }
    }
    // pretty_format on representative register values.
    let pretty_cases: [(&str, i64); 10] = [
        ("CHOPCONF", 0x12345678),
        ("DRV_STATUS", 0xFFFFFFFF),
        ("DRV_STATUS", 0x00000000),
        ("DRV_STATUS", 0x00010000),
        ("GSTAT", 0x7),
        ("IOIN", 0),
        ("IOIN", 0xC00000FF),
        ("GCONF", 0xFFFF),
        ("COOLCONF", 0x00000000),
        ("PWMCONF", 0xFF000000),
    ];
    for (reg, value) in pretty_cases {
        py.exec(&format!("_pf = _f.pretty_format({reg:?}, {value})")).expect("oracle pretty");
        let expected = oracle_str(&py.eval("_pf").unwrap());
        let helper = make_field_helper();
        let actual = helper.pretty_format(reg, value);
        assert_val("pretty_format", "", Rv::Str(expected), Rv::Str(actual));
    }
    // get_reg_fields.
    for (reg, value) in pretty_cases {
        py.exec(&format!("_gr = _f.get_reg_fields({reg:?}, {value})")).expect("oracle reg_fields");
        let expected = py.eval("_gr").unwrap();
        let helper = make_field_helper();
        let actual = helper.get_reg_fields(reg, value);
        let actual_val = Rv::Dict(
            actual.iter().map(|(k, v)| (Rv::Str(k.clone()), Rv::Int(*v))).collect(),
        );
        assert_val("get_reg_fields", "", expected, actual_val);
    }
    // Unknown field raises KeyError on both sides.
    let err = py.eval("_f.get_field('no_such_field')").unwrap_err();
    assert_eq!(err.class_name, "KeyError", "oracle unknown-field error class");
    let helper = make_field_helper();
    let rust_err = helper.get_field("no_such_field", None, None).expect_err("rust unknown field");
    assert_eq!(rust_err.class_name(), "KeyError");
    assert_val("unknown_field_msg", "", Rv::Str(err.message.clone()), Rv::Str(rust_err.message().to_string()));
}

fn helper_lookup(reg: &str, field: &str) -> Rv {
    let helper = make_field_helper();
    match helper.lookup_register(field) {
        Some(r) => {
            assert_eq!(r, reg, "lookup_register({field}) must map to {reg}");
            Rv::Str(r)
        }
        None => Rv::Null,
    }
}

fn current_grid() -> Vec<(f64, f64, f64)> {
    vec![
        (1.0, 0.5, 0.075),
        (2.0, 1.0, 0.075),
        (0.5, 1.5, 0.075),
        (3.0, 3.0, 0.075),
        (10.0, 10.0, 0.075),
        (2.0, 1.0, 0.11),
        (1.5, 0.25, 0.05),
        (4.0, 0.1, 0.075),
        (0.2, 0.2, 0.075),
        (5.0, 2.5, 0.075),
        (1.2, 0.6, 0.075),
    ]
}

#[test]
fn current_math_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    setup_oracle(&mut py);
    for (run, hold, sense) in current_grid() {
        py.exec(&format!(
            "_cfg = _FakeConfig('tmc5160 stepper_y', {{'run_current': {run:?}, 'hold_current': {hold:?}, 'sense_resistor': {sense:?}}}); \
             _h = _m.TMC5160CurrentHelper(_cfg, _fm)"
        ))
        .expect("construct helper");
        // Direct _calc_current comparison with equal inputs.
        let expected_tuple = py
            .eval(&format!("list(_h._calc_current({run:?}, {hold:?}))"))
            .unwrap();
        let (_printer, config, _reads) =
            make_config(&[("run_current", run), ("hold_current", hold), ("sense_resistor", sense)]);
        let (mcu, _fields, _writes) = make_mcu();
        let helper = Tmc5160CurrentHelper::new(&config, mcu).expect("rust helper");
        let (gs, ir, ih) = helper.calc_current(run, hold).expect("rust calc");
        let items = oracle_tuple(&expected_tuple);
        assert_val("_calc_current[0]", "", Rv::Int(oracle_int(&items[0])), Rv::Int(gs));
        assert_val("_calc_current[1]", "", Rv::Int(oracle_int(&items[1])), Rv::Int(ir));
        assert_val("_calc_current[2]", "", Rv::Int(oracle_int(&items[2])), Rv::Int(ih));
        // get_current tuple comparison, floats by bits.
        let expected_gc = py.eval("list(_h.get_current())").unwrap();
        let (rc, hc, req, max) = helper.get_current();
        let items = oracle_tuple(&expected_gc);
        assert_val("get_current.run", "", items[0].clone(), py_float(rc));
        assert_val("get_current.hold", "", items[1].clone(), py_float(hc.unwrap()));
        assert_val("get_current.req", "", items[2].clone(), py_float(req.unwrap()));
        assert_val("get_current.max", "", items[3].clone(), py_float(max));
    }
}

#[test]
fn current_set_writes_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    setup_oracle(&mut py);
    for (run, hold, sense) in [(2.0, 1.0, 0.075), (1.0, 0.5, 0.075), (3.0, 1.5, 0.11)] {
        py.exec(&format!(
            "_cfg = _FakeConfig('tmc5160 stepper_y', {{'run_current': {run:?}, 'hold_current': {hold:?}, 'sense_resistor': {sense:?}}}); \
             _fm.writes.clear(); \
             _h = _m.TMC5160CurrentHelper(_cfg, _fm); \
             _h.set_current(1.5, 0.75, 42.0); \
             _w = list(_fm.writes); \
             _g = _h.get_current()"
        ))
        .expect("oracle set_current");
        let writes_val = py.eval("_w").unwrap();
        let get_val = py.eval("_g").unwrap();
        let (_printer, config, _reads) =
            make_config(&[("run_current", run), ("hold_current", hold), ("sense_resistor", sense)]);
        let (mcu, _fields, writes) = make_mcu();
        let helper = Tmc5160CurrentHelper::new(&config, mcu).expect("rust helper");
        helper.set_current(1.5, 0.75, 42.0).expect("rust set_current");
        let rust_writes = writes.lock().unwrap();
        let wl = match &writes_val {
            Rv::List(items) => items.clone(),
            other => panic!("writes not a list: {other:?}"),
        };
        assert_eq!(wl.len(), rust_writes.len(), "GLOBALSCALER/IHOLD_IRUN write count");
        for (expected, (reg, val, pt)) in wl.iter().zip(rust_writes.iter()) {
            let parts = oracle_tuple(expected);
            assert_val("write.reg", "", parts[0].clone(), Rv::Str(reg.clone()));
            assert_val("write.val", "", parts[1].clone(), Rv::Int(*val));
            assert_val("write.time", "", parts[2].clone(), py_float(pt.unwrap()));
        }
        let items = oracle_tuple(&get_val);
        let (rc, hc, req, max) = helper.get_current();
        assert_val("after.run", "", items[0].clone(), py_float(rc));
        assert_val("after.hold", "", items[1].clone(), py_float(hc.unwrap()));
        assert_val("after.req", "", items[2].clone(), py_float(req.unwrap()));
        assert_val("after.max", "", items[3].clone(), py_float(max));
    }
}

#[test]
fn config_reads_and_errors_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    setup_oracle(&mut py);
    // Missing run_current: both sides raise "must be specified".
    py.exec("_cfg = _FakeConfig('tmc5160 stepper_y', {})").expect("cfg");
    let err = py.eval("_m.TMC5160CurrentHelper(_cfg, _fm)").unwrap_err();
    let (_printer, config, _reads) = make_config(&[]);
    let (mcu, _fields, _writes) = make_mcu();
    let rust_err = match Tmc5160CurrentHelper::new(&config, mcu) { Ok(_) => panic!("missing run_current must fail"), Err(e) => e };
    assert_val(
        "missing_option_msg",
        "",
        Rv::Str(err.message.clone()),
        Rv::Str(rust_err.message().to_string()),
    );
    // run_current above /maxval violations.
    for bad in [0.0, -1.0, 10.5] {
        py.exec(&format!(
            "_cfg = _FakeConfig('tmc5160 stepper_y', {{'run_current': {bad:?}}}); \
             _r = _m.TMC5160CurrentHelper(_cfg, _fm)"
        ))
        .ok();
        let err = py
            .eval("_m.TMC5160CurrentHelper(_cfg, _fm)")
            .unwrap_err();
        let (_printer, config, _reads) = make_config(&[("run_current", bad)]);
        let (mcu, _fields, _writes) = make_mcu();
        let rust_err = match Tmc5160CurrentHelper::new(&config, mcu) { Ok(_) => panic!("bad run_current must fail"), Err(e) => e };
        assert_val(
            &format!("run_current={bad} msg"),
            "",
            Rv::Str(err.message.clone()),
            Rv::Str(rust_err.message().to_string()),
        );
    }
    // hold_current / sense_resistor defaults (omitted options).
    py.exec(
        "_cfg = _FakeConfig('tmc5160 stepper_y', {'run_current': 2.0}); \
         _h = _m.TMC5160CurrentHelper(_cfg, _fm); \
         _d = (_h.req_hold_current, _h.sense_resistor)",
    )
    .expect("defaults");
    let (_printer, config, _reads) = make_config(&[("run_current", 2.0)]);
    let (mcu, _fields, _writes) = make_mcu();
    let helper = Tmc5160CurrentHelper::new(&config, mcu).expect("defaults helper");
    let d = py.eval("_d").unwrap();
    let items = oracle_tuple(&d);
    assert_val("default_hold", "", items[0].clone(), py_float(helper.req_hold_current()));
    assert_val(
        "default_sense",
        "",
        items[1].clone(),
        py_float(helper.sense_resistor),
    );
    // set_config_field bounds: signed field `sgt`, bool field `multistep_filt`.
    py.exec(
        "_cfg = _FakeConfig('tmc5160 stepper_y', {'driver_SGT': 100}); \
         _f.set_config_field(_cfg, 'sgt', 0)",
    )
    .ok();
    let err = py
        .eval("_f.set_config_field(_FakeConfig('tmc5160 stepper_y', {'driver_SGT': 100}), 'sgt', 0)")
        .unwrap_err();
    {
        let (printer, _g, _w) = make_printer();
        let mut full = FakeConfig::new(printer, "tmc5160 stepper_y");
        full.ints.insert("driver_SGT".to_string(), 100);
        let config: Config = Arc::new(full);
        let helper = make_field_helper();
        let rust_err = helper
            .set_config_field(&config, "sgt", ConfigFieldDefault::Int(0))
            .expect_err("sgt bound");
        assert_val(
            "sgt_bound_msg",
            "",
            Rv::Str(err.message.clone()),
            Rv::Str(rust_err.message().to_string()),
        );
    }
}

#[test]
fn tmc5160_defaults_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    setup_oracle(&mut py);
    // The manual oracle sequence mirrors `TMC5160.__init__` helper order:
    // current helper, microstep (CHOPCONF via `TMCCommandHelper`),
    // wave table, stealthChop / CoolStep / high-velocity thresholds,
    // then the `set_config_field` defaults of `tmc5160.py:345-386`.
    // Register order is insertion order, so sequence must match.
    py.exec(
        "_f.registers.clear(); \
         _fm.writes.clear(); \
         _cfg = _FakeConfig('tmc5160 stepper_y', {'run_current': 2.0}); \
         _h = _m.TMC5160CurrentHelper(_cfg, _fm); \
         _f.set_field('mres', 0); \
         _f.set_field('intpol', 1); \
         _f.set_config_field(_cfg, 'mslut0', 0xAAAAB554); \
         _f.set_config_field(_cfg, 'mslut1', 0x4A9554AA); \
         _f.set_config_field(_cfg, 'mslut2', 0x24492929); \
         _f.set_config_field(_cfg, 'mslut3', 0x10104222); \
         _f.set_config_field(_cfg, 'mslut4', 0xFBFFFFFF); \
         _f.set_config_field(_cfg, 'mslut5', 0xB5BB777D); \
         _f.set_config_field(_cfg, 'mslut6', 0x49295556); \
         _f.set_config_field(_cfg, 'mslut7', 0x00404222); \
         _f.set_config_field(_cfg, 'w0', 2); \
         _f.set_config_field(_cfg, 'w1', 1); \
         _f.set_config_field(_cfg, 'w2', 1); \
         _f.set_config_field(_cfg, 'w3', 1); \
         _f.set_config_field(_cfg, 'x1', 128); \
         _f.set_config_field(_cfg, 'x2', 255); \
         _f.set_config_field(_cfg, 'x3', 255); \
         _f.set_config_field(_cfg, 'start_sin', 0); \
         _f.set_config_field(_cfg, 'start_sin90', 247); \
         _f.set_field('tpwmthrs', 0xfffff); \
         _f.set_field('en_pwm_mode', 0); \
         _f.set_field('tcoolthrs', 0); \
         _f.set_field('thigh', 0); \
         _f.set_config_field(_cfg, 'multistep_filt', True); \
         _f.set_config_field(_cfg, 'toff', 3); \
         _f.set_config_field(_cfg, 'hstrt', 5); \
         _f.set_config_field(_cfg, 'hend', 2); \
         _f.set_config_field(_cfg, 'fd3', 0); \
         _f.set_config_field(_cfg, 'disfdcc', 0); \
         _f.set_config_field(_cfg, 'chm', 0); \
         _f.set_config_field(_cfg, 'tbl', 2); \
         _f.set_config_field(_cfg, 'vhighfs', 0); \
         _f.set_config_field(_cfg, 'vhighchm', 0); \
         _f.set_config_field(_cfg, 'tpfd', 4); \
         _f.set_config_field(_cfg, 'diss2g', 0); \
         _f.set_config_field(_cfg, 'diss2vs', 0); \
         _f.set_config_field(_cfg, 'semin', 0); \
         _f.set_config_field(_cfg, 'seup', 0); \
         _f.set_config_field(_cfg, 'semax', 0); \
         _f.set_config_field(_cfg, 'sedn', 0); \
         _f.set_config_field(_cfg, 'seimin', 0); \
         _f.set_config_field(_cfg, 'sgt', 0); \
         _f.set_config_field(_cfg, 'sfilt', 0); \
         _f.set_config_field(_cfg, 'drvstrength', 0); \
         _f.set_config_field(_cfg, 'bbmclks', 4); \
         _f.set_config_field(_cfg, 'bbmtime', 0); \
         _f.set_config_field(_cfg, 'filt_isense', 0); \
         _f.set_config_field(_cfg, 'iholddelay', 6); \
         _f.set_config_field(_cfg, 'pwm_ofs', 30); \
         _f.set_config_field(_cfg, 'pwm_grad', 0); \
         _f.set_config_field(_cfg, 'pwm_freq', 0); \
         _f.set_config_field(_cfg, 'pwm_autoscale', True); \
         _f.set_config_field(_cfg, 'pwm_autograd', True); \
         _f.set_config_field(_cfg, 'freewheel', 0); \
         _f.set_config_field(_cfg, 'pwm_reg', 4); \
         _f.set_config_field(_cfg, 'pwm_lim', 12); \
         _f.set_config_field(_cfg, 'tpowerdown', 10); \
         _regs = dict(_f.registers); \
         _pp = type('_PhaseProbe', (object,), {'fields': _f})(); \
         _phases = _tmc.TMCCommandHelper._get_phases(_pp); \
         _f.set_field('mres', 4); \
         _phases_mres4 = _tmc.TMCCommandHelper._get_phases(_pp); \
         _f.set_field('mres', 0); \
         _cp = type('_CmdProbe', (object,), {})(); \
         _cp.stepper = None; \
         _cp.mcu_phase_offset = None; \
         _cp.current_helper = _h; \
         _cp.echeck_helper = type('_Echeck', (object,), \
             {'get_status': (lambda self, eventtime=None: {})})(); \
         _status = _tmc.TMCCommandHelper.get_status(_cp)",
    )
    .expect("oracle defaults");
    let expected_regs = py.eval("_regs").unwrap();
    let (_printer, config, _reads) = make_config(&[("run_current", 2.0)]);
    let (mcu, _fields, _writes) = make_mcu();
    let obj = Tmc5160::assemble(&config, mcu).expect("rust Tmc5160");
    let rust_regs = Rv::Dict(
        obj.fields()
            .registers()
            .iter()
            .map(|(k, v)| (Rv::Str(k.clone()), Rv::Int(*v)))
            .collect(),
    );
    assert_val("registers_after_defaults", "", expected_regs, rust_regs);
    // `get_phase_offset` and `get_status` are the command helper's
    // (`tmc5160.py:337-338`); the phase count and the four status keys
    // the reference writes come from the reference methods themselves.
    let expected_phases = oracle_int(&py.eval("_phases").unwrap());
    let obj_ref = Arc::new(obj);
    let (mcu_phase_offset, phases) = obj_ref.get_phase_offset();
    assert!(mcu_phase_offset.is_none(), "mcu_phase_offset is unset before a sync");
    assert_val(
        "get_phase_offset.phases",
        "",
        Rv::Int(expected_phases),
        Rv::Int(phases),
    );
    // The shift is `256 >> mres`, so a microstep value of 16 also runs.
    let expected_phases_mres4 = oracle_int(&py.eval("_phases_mres4").unwrap());
    obj_ref.fields().set_field("mres", 4, None, None).expect("set mres");
    let (_offset, phases_mres4) = obj_ref.get_phase_offset();
    obj_ref.fields().set_field("mres", 0, None, None).expect("restore mres");
    assert_val(
        "get_phase_offset.phases_mres4",
        "",
        Rv::Int(expected_phases_mres4),
        Rv::Int(phases_mres4),
    );
    let expected_status = match py.eval("_status").unwrap() {
        Rv::Dict(pairs) => pairs,
        other => panic!("expected reference status dict, got {other:?}"),
    };
    let got_status = match obj_ref.get_status(1.0) {
        Some(PyValue::Dict(pairs)) => pairs,
        other => panic!("get_status must return a dict, got {other:?}"),
    };
    for (key, value) in expected_status {
        let name = oracle_str(&key);
        let actual = got_status
            .iter()
            .find(|(k, _)| matches!(k, PyValue::Str(s) if s.as_str() == name.as_str()))
            .map(|(_, v)| v.clone())
            .unwrap_or_else(|| panic!("status lacks {name}"));
        assert_val(&format!("get_status.{name}"), "", value, rv_of(&actual));
    }
}

#[test]
fn factory_and_defaults_without_oracle() {
    // Always-run structural checks that need no reference values: the
    // factory exposes `load_config_prefix`, which applies the current
    // config and the documented defaults; `TMC_FREQUENCY` matches the
    // `tmc5160.py:9` literal mirrored by the port.
    let factory: ModuleFactory = module_factory();
    assert!(factory.load_config_prefix.is_some(), "load_config_prefix present");
    assert!(factory.load_config.is_none(), "single-token load_config absent (Python has none)");
    let (_printer, config, _reads) = make_config(&[("run_current", 2.0)]);
    let (mcu, _fields, _writes) = make_mcu();
    let obj = Tmc5160::assemble(&config, mcu).expect("Tmc5160::assemble");
    let obj_ref = Arc::new(obj);
    // Downcast through the printer object works (object model check).
    let as_obj: ObjectRef = obj_ref.clone();
    assert!(downcast::<Tmc5160>(as_obj.as_ref()).is_some(), "downcast to Tmc5160");
    // The object declares the command-helper phase-offset interface.
    let phase: Option<Arc<dyn TMCCommandHelperApi>> =
        klippy_api::interface::<dyn TMCCommandHelperApi>(&as_obj);
    assert!(phase.is_some(), "TMCCommandHelperApi interface");
    // Current bits stay in range over the whole helper grid (property,
    // not a transcribed reference value).
    let (_printer, config, _reads) = make_config(&[("run_current", 1.0), ("hold_current", 0.5)]);
    let (mcu, _fields, _writes) = make_mcu();
    let helper = Tmc5160CurrentHelper::new(&config, mcu).expect("helper");
    for run in [0.1, 0.5, 1.0, 5.0, 10.0] {
        let (gs, ir, ih) = helper.calc_current(run, run / 2.0).expect("calc");
        assert!((gs == 0 || (32..=255).contains(&gs)), "globalscaler in range: {gs}");
        assert!((0..=31).contains(&ir), "irun in range: {ir}");
        assert!((0..=31).contains(&ih), "ihold in range: {ih}");
    }
}

