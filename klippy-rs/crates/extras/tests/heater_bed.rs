//! Parity tests for `extras::heater_bed` (port of `klippy/extras/heater_bed.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterHeaterBed` is
//! built on both sides with minimal Python/Rust fakes for the printer, the
//! config wrapper, the `gcode` dispatcher and the `heaters` collection,
//! then fed identical construction and `M140`/`M190` commands and the
//! registered commands, `setup_heater('B')` calls, `set_temperature`
//! arguments, `get_status` dicts, `stats` tuples and errors (class and
//! message) are compared exactly.
//!
//! The bed heater on the Rust side is a real [`extras::heaters::Heater`]
//! (watermark control, fake sensor/pin/MCU); the Python fake mirrors that
//! real heater's observable state (range  -10..120, smoothed/last temp 0,
//! power 0) so `get_status`/`stats` delegation is compared for real.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::heater_bed::PrinterHeaterBed;
use extras::heaters::{
    Heater, HeaterApi, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandWrapper, ConfigCallback,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi,
    PwmPin, PwmPinApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.heater_bed as heater_bed

class _Sentinel: pass
_SENTINEL = _Sentinel

class CommandError(Exception): pass

class FakeGCode:
    def __init__(self):
        self.registered = {}
    def register_command(self, cmd, func):
        self.registered[cmd] = func

class FakeHeater:
    # Mirrors extras.heaters.Heater with min_temp=-10, max_temp=120 and
    # no temperature callbacks yet: smoothed/last temp 0.0, power 0.0.
    def __init__(self):
        self.min_temp = -10.
        self.max_temp = 120.
        self.target = 0.0
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise CommandError(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        self.target = degrees
    def get_status(self, eventtime):
        return {'temperature': round(0.0, 2), 'target': self.target,
                'power': 0.0}
    def stats(self, eventtime):
        is_active = self.target != 0. or 0.0 > 50.
        return is_active, 'B: target=%.0f temp=%.1f pwm=%.3f' % (
            self.target, 0.0, 0.0)

class FakeHeaters:
    def __init__(self, heater):
        self.heater = heater
        self.setup_calls = []
        self.set_calls = []
    def setup_heater(self, config, gcode_id=None):
        self.setup_calls.append(gcode_id)
        return self.heater
    def set_temperature(self, heater, temp, wait=False):
        self.set_calls.append((temp, wait))
        heater.set_temp(temp)

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self, with_heaters=True, with_gcode=True):
        self.gcode = FakeGCode()
        self.heater = FakeHeater()
        self.heaters = FakeHeaters(self.heater)
        self.objects = {}
        if with_gcode:
            self.objects['gcode'] = self.gcode
        if with_heaters:
            self.objects['heaters'] = self.heaters
    def load_object(self, config, section, default=_SENTINEL):
        if section in self.objects:
            return self.objects[section]
        if default is not _SENTINEL:
            return default
        raise self.config_error("Unable to load module '%s'" % (section,))
    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is not _SENTINEL:
            return default
        raise self.config_error("Unknown config object '%s'" % (name,))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'heater_bed'

class FakeGCmd:
    error = CommandError
    def __init__(self, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = params
    def get(self, name, default=_SENTINEL, parser=str, minval=None, maxval=None,
            above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _SENTINEL:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
        except Exception:
            raise self.error(
                "Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s"
                             % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s"
                             % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s"
                             % (self._commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s"
                             % (self._commandline, name, below))
        return value
    def get_float(self, name, default=_SENTINEL, minval=None, maxval=None,
                  above=None, below=None):
        return self.get(name, default, parser=float, minval=minval, maxval=maxval,
                        above=above, below=below)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct DummyObject;

impl PrinterObject for DummyObject {
    klippy_api::printer_object_base!();
}

// Sensor with a fixed report delta; no callbacks are ever fired, so the
// heater's smoothed/last temps stay 0.0 like the Python fake.
struct FakeSensor;

impl PrinterObject for FakeSensor {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);
}

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 {
        0.3
    }
}

struct FakeMcu;

impl PrinterObject for FakeMcu {
    klippy_api::printer_object_base!();
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used by heater_bed tests")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is on PinsApi, not McuApi")
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
    fn seconds_to_clock(&self, time: f64) -> i64 {
        time as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.1
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command not used")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command not used")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("register_serial_response not used")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Ok(0.)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        print_time as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, 0.))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("trigger dispatch not used")
    }
}

struct FakePwm;

impl PrinterObject for FakePwm {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PwmPinApi);
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        Arc::new(FakeMcu) as Mcu
    }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: Option<bool>) -> klippy_api::KResult<()> { Ok(()) }
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, _print_time: f64, _value: f64) -> klippy_api::KResult<()> { Ok(()) }
}

struct FakePins;

impl PrinterObject for FakePins {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PinsApi);
}
impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin not used")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Ok(Pin::Pwm(Arc::new(FakePwm) as PwmPin))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(
        &self,
        _chip_name: &str,
    ) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

struct FakeHeaters {
    heater: Arc<Heater>,
    setup_calls: Mutex<Vec<String>>,
    set_calls: Mutex<Vec<(f64, bool)>>,
}

// `setup_heater` returns the prebuilt `Heater`; declaration is via the shared
// `printer_object!` macro so `load_interface::<dyn PrinterHeatersApi>` works.
klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {}
    fn setup_heater(
        &self,
        _config: &Config,
        gcode_id: Option<String>,
    ) -> KResult<Arc<Heater>> {
        self.setup_calls
            .lock()
            .push(gcode_id.clone().unwrap_or_default());
        Ok(self.heater.clone())
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        Err(KlippyError::config(format!("Unknown heater '{heater_name}'")))
    }
    fn setup_sensor(
        &self,
        _config: &Config,
    ) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Err(KlippyError::config("setup_sensor not used by heater_bed tests"))
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()> {
        self.set_calls.lock().push((temp, wait));
        HeaterApi::set_temp(&**heater, temp)
    }
}

struct FakeGCode {
    handlers: Mutex<Vec<(String, GCodeCommandHandler)>>,
}

impl FakeGCode {
    fn new() -> Arc<FakeGCode> {
        Arc::new(FakeGCode { handlers: Mutex::new(Vec::new()) })
    }
    fn registered_names(&self) -> Vec<String> {
        let handlers = self.handlers.lock();
        let mut names: Vec<String> = handlers.iter().map(|(c, _)| c.clone()).collect();
        names.sort();
        names
    }
    fn call(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handlers = self.handlers.lock();
        let entry = handlers
            .iter_mut()
            .find(|(c, _)| c == cmd)
            .expect("command registered");
        entry.1(gcmd)
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut handlers = self.handlers.lock();
        if let Some(func) = func {
            handlers.push((cmd.to_string(), func));
            Ok(None)
        } else {
            let pos = handlers.iter().position(|(c, _)| c == cmd);
            match pos {
                Some(i) => Ok(Some(handlers.swap_remove(i).1)),
                None => Ok(None),
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
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            commandline: String::new(),
            params: IndexMap::new(),
        })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
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

struct FakeGCmd {
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn new(command: &str, s: Option<&str>) -> GCmd {
        let mut params = IndexMap::new();
        if let Some(s) = s {
            params.insert("S".to_string(), s.to_string());
        }
        let commandline = if let Some(s) = s {
            format!("{command} S={s}")
        } else {
            command.to_string()
        };
        Arc::new(FakeGCmd { commandline, params })
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(PyValue::None) => Ok(PyValue::None),
                Some(v) => Ok(v),
            },
            Some(v) => Ok(PyValue::Str(v.clone())),
        }
    }
    fn get_int(&self, _name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
            Some(v) => match v.trim().parse::<f64>() {
                Ok(f) => Ok(Some(f)),
                Err(_) => Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {v}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_command(&self) -> String {
        String::new()
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
        false
    }
}

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter { objects: Mutex::new(IndexMap::new()) })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
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
        let mut objects = self.objects.lock();
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
        let objects = self.objects.lock();
        match objects.get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let objects = self.objects.lock();
            match module {
                None => objects
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect(),
                Some(module) => {
                    let prefix = format!("{module} ");
                    objects
                        .iter()
                        .filter(|(k, _)| k.as_str() == module || k.starts_with(&prefix))
                        .map(|(k, v)| (k.clone(), v.clone()))
                        .collect()
                }
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let mut objects = self.objects.lock();
        if let Some(o) = objects.get(section) {
            return Ok(Some(o.clone()));
        }
        // `Heater::new` loads these side objects; dummies stand in for them.
        if section.starts_with("verify_heater") || section == "pid_calibrate" {
            let dummy: ObjectRef = Arc::new(DummyObject);
            objects.insert(section.to_string(), dummy.clone());
            return Ok(Some(dummy));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
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
    name: String,
    floats: IndexMap<String, f64>,
}

impl FakeConfig {
    fn heater_config(printer: Printer) -> Config {
        let mut floats = IndexMap::new();
        floats.insert("min_temp".to_string(), -10.0);
        floats.insert("max_temp".to_string(), 120.0);
        floats.insert("min_extrude_temp".to_string(), 170.0);
        floats.insert("max_power".to_string(), 1.0);
        floats.insert("smooth_time".to_string(), 1.0);
        floats.insert("pwm_cycle_time".to_string(), 0.1);
        floats.insert("max_delta".to_string(), 2.0);
        Arc::new(FakeConfig {
            printer,
            name: "heater_bed B".to_string(),
            floats,
        })
    }
    fn bed_config(printer: Printer) -> Config {
        Arc::new(FakeConfig {
            printer,
            name: "heater_bed".to_string(),
            floats: IndexMap::new(),
        })
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        if option == "heater_pin" {
            Ok(Some("PA0".to_string()))
        } else {
            Ok(None)
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(self.floats.get(option).copied())
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        if option == "control" {
            Ok(PyValue::Str("watermark".to_string()))
        } else {
            Ok(PyValue::None)
        }
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
        panic!("getsection is not used by heater_bed")
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

/// Builds a real `Heater` on top of the given printer (which must already
/// hold `pins` and `gcode`).
fn build_heater(printer: &Printer) -> Arc<Heater> {
    let cfg = FakeConfig::heater_config(printer.clone());
    let sensor: Arc<dyn TemperatureSensorApi> = Arc::new(FakeSensor);
    Heater::new(&cfg, sensor).expect("rust Heater::new")
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("heater_bed fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping heater_bed oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

struct RustPair {
    bed: PrinterHeaterBed,
    gcode: Arc<FakeGCode>,
    heaters: Arc<FakeHeaters>,
    heater: Arc<Heater>,
    config: Config,
}

fn fresh_pair() -> RustPair {
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let gcode = FakeGCode::new();
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("pins", Arc::new(FakePins) as ObjectRef);
    let heater = build_heater(&printer_ref);
    let heaters = Arc::new(FakeHeaters {
        heater: heater.clone(),
        setup_calls: Mutex::new(Vec::new()),
        set_calls: Mutex::new(Vec::new()),
    });
    printer.add("heaters", heaters.clone() as ObjectRef);
    let config = FakeConfig::bed_config(printer_ref);
    let bed = PrinterHeaterBed::new(&config).expect("rust PrinterHeaterBed::new");
    RustPair { bed, gcode, heaters, heater, config }
}

fn fresh_py(py: &mut pyoracle::Oracle) {
    py.exec("p = FakePrinter(); c = FakeConfig(p); bed = heater_bed.load_config(c)")
        .expect("py load_config");
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(i) => Value::int(
            i.0.to_string().parse::<i64>().expect("test int in i64 range"),
        ),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => {
            Value::List(items.iter().map(pyvalue_to_value).collect())
        }
        PyValue::Tuple(items) => {
            Value::Tuple(items.iter().map(pyvalue_to_value).collect())
        }
        PyValue::Dict(dict) => Value::Dict(
            dict.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host:{h}")),
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    check_ok(label, expected, actual);
}

fn check_ok(label: &str, expected: Value, actual: Value) {
    let record = Record {
        v: 1,
        unit: "heater_bed".to_string(),
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
#[allow(clippy::let_unit_value)]

fn check_unit(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let _ = actual.expect("rust handler must succeed");
    check_ok(label, expected, Value::Null);
}

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<()>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust succeeded but call must fail"),
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

fn set_calls_value(calls: &[(f64, bool)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(t, w)| Value::Tuple(vec![Value::float(*t), Value::Bool(*w)]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn construction_registers_m140_m190_and_sets_up_bed() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rust = fresh_pair();

    let expected = py
        .eval("sorted(list(p.gcode.registered.keys()))")
        .expect("py registered");
    let names = rust.gcode.registered_names();
    check(
        "registered command names",
        Ok(expected),
        Value::List(names.iter().map(|n| Value::str(n)).collect()),
    );

    let expected = py.eval("p.heaters.setup_calls").expect("py setup calls");
    let calls = rust.heaters.setup_calls.lock().clone();
    check(
        "setup_heater gcode id",
        Ok(expected),
        Value::List(calls.iter().map(|n| Value::str(n)).collect()),
    );
}

#[test]
fn m140_sets_temperature_without_wait() {
    let Some(mut py) = oracle() else { return };
    for s in ["60", "0", "-5", "60.5"] {
        fresh_py(&mut py);
        let rust = fresh_pair();
        let expected = py
            .eval(&format!(
                "p.gcode.registered['M140'](FakeGCmd('M140', 'M140 S={s}', {{'S': '{s}'}}))"
            ))
            .expect("py M140");
        let gcmd = FakeGCmd::new("M140", Some(s));
        let actual = rust.gcode.call("M140", &gcmd);
        check_unit(&format!("M140 S={s}"), Ok(expected), actual);

        let expected = py.eval("p.heaters.set_calls").expect("py set calls");
        let calls = rust.heaters.set_calls.lock().clone();
        check(&format!("set_calls after M140 S={s}"), Ok(expected), set_calls_value(&calls));

        let expected = py.eval("bed.get_status(0.0)").expect("py status");
        let status = rust.bed.get_status(0.0).unwrap().expect("status");
        check(&format!("get_status after M140 S={s}"), Ok(expected.clone()), pyvalue_to_value(&status));
        let expected = py.eval("bed.stats(0.0)").expect("py stats");
        let (active, msg) = rust.bed.stats(0.0).expect("stats");
        check(&format!("stats after M140 S={s}"), Ok(expected), Value::Tuple(vec![Value::Bool(active), Value::str(&msg)]));
    }
}

#[test]
fn m140_without_s_defaults_to_zero() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rust = fresh_pair();
    let expected = py
        .eval("p.gcode.registered['M140'](FakeGCmd('M140', 'M140', {}))")
        .expect("py M140");
    let gcmd = FakeGCmd::new("M140", None);
    check_unit("M140 without S", Ok(expected), rust.gcode.call("M140", &gcmd));
    let expected = py.eval("p.heaters.set_calls").expect("py set calls");
    let calls = rust.heaters.set_calls.lock().clone();
    check("set_calls after bare M140", Ok(expected), set_calls_value(&calls));
}

#[test]
fn m190_waits_for_temperature() {
    let Some(mut py) = oracle() else { return };
    for s in ["60", "0"] {
        fresh_py(&mut py);
        let rust = fresh_pair();
        let expected = py
            .eval(&format!(
                "p.gcode.registered['M190'](FakeGCmd('M190', 'M190 S={s}', {{'S': '{s}'}}))"
            ))
            .expect("py M190");
        let gcmd = FakeGCmd::new("M190", Some(s));
        check_unit(&format!("M190 S={s}"), Ok(expected), rust.gcode.call("M190", &gcmd));
        let expected = py.eval("p.heaters.set_calls").expect("py set calls");
        let calls = rust.heaters.set_calls.lock().clone();
        check(&format!("set_calls after M190 S={s}"), Ok(expected), set_calls_value(&calls));
    }
}

#[test]
fn direct_cmd_methods_match_handlers() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rust = fresh_pair();
    // cmd_M140 with wait=False matches M140; cmd_M190 delegates with True.
    let expected = py
        .eval("bed.cmd_M140(FakeGCmd('M140', 'M140 S=60', {'S': '60'}))")
        .expect("py cmd_M140");
    let gcmd = FakeGCmd::new("M140", Some("60"));
    check_unit("cmd_M140", Ok(expected), rust.bed.cmd_M140(&gcmd, false));
    let expected = py
        .eval("bed.cmd_M190(FakeGCmd('M190', 'M190 S=60', {'S': '60'}))")
        .expect("py cmd_M190");
    let gcmd = FakeGCmd::new("M190", Some("60"));
    check_unit("cmd_M190", Ok(expected), rust.bed.cmd_M190(&gcmd));
    let expected = py.eval("p.heaters.set_calls").expect("py set calls");
    let calls = rust.heaters.set_calls.lock().clone();
    check("set_calls direct methods", Ok(expected), set_calls_value(&calls));
}

#[test]
fn invalid_float_raises_command_error() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rust = fresh_pair();
    let expected = py
        .eval("p.gcode.registered['M140'](FakeGCmd('M140', 'M140 S=abc', {'S': 'abc'}))");
    let gcmd = FakeGCmd::new("M140", Some("abc"));
    check_err("M140 S=abc", expected, rust.gcode.call("M140", &gcmd));
    // A failed parse must not reach set_temperature on either side.
    let expected = py.eval("p.heaters.set_calls").expect("py set calls");
    let calls = rust.heaters.set_calls.lock().clone();
    check("set_calls after failed M140", Ok(expected), set_calls_value(&calls));
}

#[test]
fn out_of_range_temperature_raises_command_error() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rust = fresh_pair();
    let expected = py
        .eval("p.gcode.registered['M140'](FakeGCmd('M140', 'M140 S=200', {'S': '200'}))");
    let gcmd = FakeGCmd::new("M140", Some("200"));
    check_err("M140 S=200", expected, rust.gcode.call("M140", &gcmd));
    // The call still reached set_temperature on both sides.
    let expected = py.eval("p.heaters.set_calls").expect("py set calls");
    let calls = rust.heaters.set_calls.lock().clone();
    check("set_calls after out-of-range M140", Ok(expected), set_calls_value(&calls));
    // Target is unchanged on both sides.
    let expected = py.eval("bed.get_status(0.0)").expect("py status");
    let status = rust.bed.get_status(0.0).unwrap().expect("status");
    check("status after failed set", Ok(expected), pyvalue_to_value(&status));
}

#[test]
fn get_status_and_stats_match_heater() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rust = fresh_pair();
    let expected = py.eval("bed.get_status(0.0)").expect("py status");
    let status = rust.bed.get_status(0.0).unwrap().expect("status");
    let actual = pyvalue_to_value(&status);
    check("get_status initial", Ok(expected), actual);
    let expected = py.eval("bed.stats(0.0)").expect("py stats");
    let (active, msg) = rust.bed.stats(0.0).expect("stats");
    check(
        "stats initial",
        Ok(expected),
        Value::Tuple(vec![Value::Bool(active), Value::str(&msg)]),
    );
    // After setting a target the delegates see the new target.
    py.exec("p.heater.set_temp(60.0)").expect("py set_temp");
    HeaterApi::set_temp(&*rust.heater, 60.0).expect("rust set_temp");
    let expected = py.eval("bed.get_status(0.0)").expect("py status after");
    let status = rust.bed.get_status(0.0).unwrap().expect("status");
    let actual = pyvalue_to_value(&status);
    check("get_status after set", Ok(expected), actual);
    let expected = py.eval("bed.stats(0.0)").expect("py stats after");
    let (active, msg) = rust.bed.stats(0.0).expect("stats");
    check(
        "stats after set",
        Ok(expected),
        Value::Tuple(vec![Value::Bool(active), Value::str(&msg)]),
    );
}

#[test]
fn missing_heaters_object_fails_load_config() {
    let Some(mut py) = oracle() else { return };
    py.exec("p = FakePrinter(with_heaters=False); c = FakeConfig(p)")
        .expect("py setup");
    let expected = py.eval("heater_bed.load_config(c)");

    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let gcode = FakeGCode::new();
    printer.add("gcode", gcode.clone() as ObjectRef);
    let config = FakeConfig::bed_config(printer_ref);
    let actual = PrinterHeaterBed::new(&config);
    match (expected, actual) {
        (Err(e), Err(rust_e)) => {
            assert_eq!(
                rust_e.class_name(),
                e.class_name.as_str(),
                "missing heaters: exception class",
            );
            assert_eq!(rust_e.message(), e.message.as_str(), "missing heaters: message");
        }
        _ => panic!("missing heaters: both sides must raise a config error"),
    }
}

#[test]
fn missing_gcode_object_fails_after_setup() {
    let Some(mut py) = oracle() else { return };
    py.exec("p = FakePrinter(with_gcode=False); c = FakeConfig(p)")
        .expect("py setup");
    let expected = py.eval("heater_bed.load_config(c)");

    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let gcode = FakeGCode::new();
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("pins", Arc::new(FakePins) as ObjectRef);
    let heater = build_heater(&printer_ref);
    let heaters = Arc::new(FakeHeaters {
        heater,
        setup_calls: Mutex::new(Vec::new()),
        set_calls: Mutex::new(Vec::new()),
    });
    printer.add("heaters", heaters.clone() as ObjectRef);
    // Drop `gcode` so the bed object's gcode lookup fails.
    printer.objects.lock().shift_remove("gcode");
    let config = FakeConfig::bed_config(printer_ref);
    let actual = PrinterHeaterBed::new(&config);
    match (expected, actual) {
        (Err(e), Err(rust_e)) => {
            assert_eq!(
                rust_e.class_name(),
                e.class_name.as_str(),
                "missing gcode: exception class",
            );
            assert_eq!(rust_e.message(), e.message.as_str(), "missing gcode: message");
        }
        _ => panic!("missing gcode: both sides must raise a config error"),
    }
    // setup_heater('B') ran before the gcode lookup on both sides.
    let expected = py.eval("p.heaters.setup_calls").expect("py setup calls");
    let calls = heaters.setup_calls.lock().clone();
    check(
        "setup ran before gcode lookup",
        Ok(expected),
        Value::List(calls.iter().map(|n| Value::str(n)).collect()),
    );
}
#[test]
fn module_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let factory = extras::heater_bed::module_factory();
    let load_config = factory
        .load_config
        .expect("heater_bed exposes load_config");
    assert!(
        factory.load_config_prefix.is_none(),
        "heater_bed has no load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "heater_bed has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "heater_bed has no add_printer_objects"
    );
    // A fresh printer, like the Python `p2` below, so the factory's
    // registrations are not duplicated against `rust`.
    let printer2 = FakePrinter::new();
    let printer2_ref: Printer = printer2.clone();
    let gcode2 = FakeGCode::new();
    printer2.add("gcode", gcode2.clone() as ObjectRef);
    printer2.add("pins", Arc::new(FakePins) as ObjectRef);
    let heater2 = build_heater(&printer2_ref);
    let heaters2 = Arc::new(FakeHeaters {
        heater: heater2,
        setup_calls: Mutex::new(Vec::new()),
        set_calls: Mutex::new(Vec::new()),
    });
    printer2.add("heaters", heaters2.clone() as ObjectRef);
    let config2 = FakeConfig::bed_config(printer2_ref);
    let obj: ObjectRef = load_config(&config2)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let bed_ref = klippy_api::downcast_arc::<PrinterHeaterBed>(&obj)
        .expect("downcast to PrinterHeaterBed");

    py.exec("p2 = FakePrinter(); c2 = FakeConfig(p2); bed2 = heater_bed.load_config(c2)")
        .expect("py factory load_config");
    let expected = py
        .eval("sorted(list(p2.gcode.registered.keys()))")
        .expect("py registered via factory");
    check(
        "factory registered names",
        Ok(expected),
        Value::List(
            gcode2
                .registered_names()
                .iter()
                .map(|n| Value::str(n))
                .collect(),
        ),
    );
    let expected = py.eval("bed2.get_status(0.0)").expect("py factory status");
    let status = bed_ref.get_status(0.0).unwrap().expect("factory status");
    let actual = pyvalue_to_value(&status);
    check("factory get_status == heater", Ok(expected), actual);
    let expected = py.eval("bed2.stats(0.0)").expect("py factory stats");
    let (active, msg) = bed_ref.stats(0.0).expect("factory stats");
    check(
        "factory stats == heater",
        Ok(expected),
        Value::Tuple(vec![Value::Bool(active), Value::str(&msg)]),
    );
}


