//! Parity tests for `extras::heater_fan` (port of `klippy/extras/heater_fan.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`.
//! The Python side runs the real `extras.heater_fan` over a fake
//! `extras.fan` module (`FakeFan` records `default_shutdown_speed`,
//! `set_speed` values and reports `{'speed': ..., 'rpm': None}`), a fake
//! heaters registry (`FakeHeater` + `FakeHeatersObj`) and a fake printer /
//! reactor / config. The Rust side runs the port over the same
//! abstractions: real [`Fan`](extras::fan::Fan) and real
//! [`Heater`](extras::heaters::Heater) objects built on fakes of the contract
//! traits (pins, MCU, motion queuing, gcode), so the fan decisions, timer
//! return values, status dicts, config reads (with defaults and bounds),
//! construction errors, `klippy:ready` resolution errors and the
//! `default_shutdown_speed=1.` handoff to `Fan` are compared exactly
//! (floats bit-for-bit through the recording value format, errors by class
//! and message).
//!
//! `FakeFan` replaces the real `fan.Fan` on the Python side: the fan unit
//! owns the PWM / kick waveform contract and is covered by its own tests.
//! On the Rust side the real `Fan` is constructed, so the test also proves
//! that `heater_fan` hands over `default_shutdown_speed=1.` (pin start
//! value) and that the fan status speed tracks the speeds `heater_fan`
//! decides. The reactor timer arming (`monotonic() + PIN_MIN_TIME`)
//! has no observable channel in the frozen contracts (a real `Reactor`
//! with no introspection); the Python fake pins the arming values to the
//! reference, and the Rust `handle_ready` follows the same code path with
//! its callback behaviour compared directly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

use extras::heaters::{
    Heater, HeaterApi, PrinterHeatersApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use extras::heater_fan::{load_config_prefix, module_factory, PrinterHeaterFan, PIN_MIN_TIME};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntOpts, GetIntListOpts,
    GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandHandler, GCmd, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, DigitalOutPin, DigitalOutPinApi, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, PwmPin,
    PwmPinApi, SerialResponseCallback,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::Trapq;
use klippy_api::toolhead::{FlushCallback, MotionQueuingApi, SyncEmitterHandle, TrapqAppend};
use kos::sync::Mutex as KosMutex;
use pycompat::text::{float_from_string, float_str, int_from_string};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
//
// `heater_fan.py` does `from . import fan`, so a fake `extras.fan` module
// is installed in `sys.modules` before the real module is imported. The
// fake printer / reactor / config reproduce `configfile.ConfigWrapper`
// getters (including error texts) and `printer.lookup_object` /
// `load_object`; config errors use a local `Error` class matching
// `configparser.Error`'s name so the oracle reports class `Error` like
// `KlippyError::Config`.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types

class Error(Exception):
    pass

class _Sentinel: pass
_SENTINEL = _Sentinel()

class FakeReactor:
    def __init__(self):
        self.timers = []
    def monotonic(self):
        return 1000.0
    def register_timer(self, cb, waketime):
        self.timers.append((cb, waketime))
        return ('timer', len(self.timers) - 1)

class FakePrinter:
    def __init__(self, reactor):
        self._reactor = reactor
        self.objects = {}
        self.handlers = {}
        self.load_calls = []
    def get_reactor(self):
        return self._reactor
    def get_start_args(self):
        return {'debugoutput': None}
    def add_object(self, name, obj):
        if name in self.objects:
            raise Error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is _SENTINEL:
            raise Error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        return [(n, self.objects[n]) for n in self.objects if n == module or n.startswith(module + ' ')]
    def load_object(self, config, section, default=_SENTINEL):
        self.load_calls.append(section)
        if section in self.objects:
            return self.objects[section]
        if default is _SENTINEL:
            return None
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

class FakeConfig:
    error = Error
    def __init__(self, printer, name, values):
        self._printer = printer
        self._name = name
        self._values = dict(values)
        self.reads = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=_SENTINEL, note_valid=True):
        self.reads.append(('get', option))
        if option not in self._values:
            if default is not _SENTINEL:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        return self._values[option]
    def getint(self, option, default=_SENTINEL, minval=None, maxval=None, note_valid=True):
        if option not in self._values:
            if default is not _SENTINEL:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        try:
            v = int(self._values[option])
        except Exception:
            raise Error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        return v
    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None, above=None, below=None, note_valid=True):
        def _fo(v):
            return 'None' if v is None else str(v)
        if default is _SENTINEL:
            _fd = 'sentinel'
        elif default is None:
            _fd = 'None'
        else:
            _fd = str(default)
        self.reads.append(('getfloat', option, '%s|%s|%s|%s|%s' % (_fd, _fo(minval), _fo(maxval), _fo(above), _fo(below))))
        if option not in self._values:
            if default is not _SENTINEL:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        try:
            v = float(self._values[option])
        except Exception:
            raise Error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v
    def getboolean(self, option, default=_SENTINEL, note_valid=True):
        if option not in self._values:
            if default is not _SENTINEL:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        raw = str(self._values[option]).lower()
        if raw in ('1', 'yes', 'true', 'on'):
            return True
        if raw in ('0', 'no', 'false', 'off'):
            return False
        raise Error("Unable to parse option '%s' in section '%s'" % (option, self._name))
    def getlist(self, option, default=_SENTINEL, sep=',', count=None, note_valid=True):
        if default is _SENTINEL:
            _d = 'sentinel'
        elif default is None:
            _d = 'None'
        else:
            _d = ','.join(default)
        self.reads.append(('getlist', option, _d))
        if option not in self._values:
            if default is not _SENTINEL:
                return list(default)
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        raw = self._values[option]
        if len(str(raw).strip()) == 0:
            parts = []
        else:
            parts = [p.strip() for p in str(raw).split(sep)]
        if count is not None and len(parts) != count:
            raise Error("Option '%s' in section '%s' must have %d elements" % (option, self._name, count))
        return parts
    def getchoice(self, option, choices, default=_SENTINEL, note_valid=True):
        if option not in self._values:
            if default is not _SENTINEL:
                c = default
            else:
                raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        else:
            c = self._values[option]
        if c not in choices:
            raise Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self._name))
        return choices[c]

class FakeHeater:
    def __init__(self, cur=0., target=0.):
        self.cur = float(cur)
        self.target = float(target)
    def get_temp(self, eventtime):
        return (self.cur, self.target)
    def set_temps(self, cur, target):
        self.cur = float(cur)
        self.target = float(target)

class FakeHeatersObj:
    def __init__(self, heaters):
        self._heaters = dict(heaters)
    def lookup_heater(self, name):
        if name not in self._heaters:
            raise Error("Unknown heater '%s'" % (name,))
        return self._heaters[name]

fan_inits = []
class FakeFan:
    def __init__(self, config, default_shutdown_speed=0.):
        fan_inits.append(float(default_shutdown_speed))
        self._speed = 0.
        self.speeds = []
    def get_mcu(self):
        return None
    def set_speed(self, value, print_time=None):
        self.speeds.append(float(value))
        self._speed = float(value)
    def get_status(self, eventtime):
        return {'speed': self._speed, 'rpm': None}

_fake_fan = types.ModuleType('extras.fan')
_fake_fan.Fan = FakeFan
sys.modules['extras.fan'] = _fake_fan

import extras.heater_fan as _hf
import configparser

def make_pair(heaters_spec, values, with_heaters=True):
    reactor = FakeReactor()
    printer = FakePrinter(reactor)
    heaters_obj = FakeHeatersObj({n: FakeHeater(c, t) for (n, c, t) in heaters_spec})
    if with_heaters:
        printer.objects['heaters'] = heaters_obj
    config = FakeConfig(printer, 'heater_fan nozzle_fan', values)
    del fan_inits[:]
    obj = _hf.PrinterHeaterFan(config)
    return (obj, printer, config, reactor, heaters_obj)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakeMcu {
    est: f64,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used in heater_fan tests")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is on PinsApi, not McuApi in heater_fan tests")
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
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
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
    ) -> AsyncResponseWrapper {
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
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        self.est
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
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("trigger dispatch not used")
    }
}

klippy_api::printer_object!(FakeMcu);

struct FakePwm {
    mcu: Mcu,
    setups: klippy_api::GCell<Vec<String>>,
    pwm_calls: klippy_api::GCell<Vec<(f64, f64)>>,
}

impl FakePwm {
    fn new() -> Arc<Self> {
        let mcu: Mcu = Arc::new(FakeMcu { est: 1000.0 });
        Arc::new(FakePwm {
            mcu,
            setups: klippy_api::GCell::new(Vec::new()),
            pwm_calls: klippy_api::GCell::new(Vec::new()),
        })
    }
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.setups.borrow_mut().push(format!("max_duration:{max_duration:?}"));
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) -> klippy_api::KResult<()> {
        self.setups
            .borrow_mut()
            .push(format!("cycle_time:{cycle_time:?}:{hardware_pwm:?}"));
        Ok(())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        self.setups
            .borrow_mut()
            .push(format!("start_value:{start_value:?}:{shutdown_value:?}"));
    }
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> klippy_api::KResult<()> {
        self.pwm_calls.borrow_mut().push((print_time, value));
        Ok(())
    }
}

klippy_api::printer_object!(FakePwm; dyn PwmPinApi);

struct FakeDigital {
    mcu: Mcu,
}

impl DigitalOutPinApi for FakeDigital {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_start_value(&self, _start_value: bool, _shutdown_value: bool) {}
    fn set_digital(&self, _print_time: f64, _value: bool) {}
}

klippy_api::printer_object!(FakeDigital; dyn DigitalOutPinApi);

/// Pins fake returning one `FakePwm` per pin description, so the heater and
/// fan PWMs (and their call logs) stay separate.
struct FakePins {
    mcu: Mcu,
    pwms: klippy_api::GCell<IndexMap<String, Arc<FakePwm>>>,
}

impl FakePins {
    fn new() -> Arc<Self> {
        Arc::new(FakePins {
            mcu: Arc::new(FakeMcu { est: 1000.0 }),
            pwms: klippy_api::GCell::new(IndexMap::new()),
        })
    }
    fn pwm_for(&self, desc: &str) -> Arc<FakePwm> {
        if let Some(p) = self.pwms.borrow().get(desc) {
            return p.clone();
        }
        let p = FakePwm::new();
        self.pwms.borrow_mut().insert(desc.to_string(), p.clone());
        p
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin not used")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        match pin_type {
            "pwm" => Ok(Pin::Pwm(self.pwm_for(pin_desc) as PwmPin)),
            "digital_out" => Ok(Pin::DigitalOut(Arc::new(FakeDigital {
                mcu: self.mcu.clone(),
            }) as DigitalOutPin)),
            other => Err(KlippyError::config(format!(
                "unknown pin type '{other}'"
            ))),
        }
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("pin resolver not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeMq {
    inner: Arc<KosMutex<MqInner>>,
}

impl Clone for FakeMq {
    fn clone(&self) -> Self {
        FakeMq { inner: self.inner.clone() }
    }
}

impl Default for FakeMq {
    fn default() -> Self {
        FakeMq { inner: Arc::new(KosMutex::new(MqInner::default())) }
    }
}

#[derive(Default)]
struct MqInner {
    flush_cbs: Vec<FlushCallback>,
    activities: Vec<(f64, bool)>,
}

impl MotionQueuingApi for FakeMq {
    fn allocate_trapq(&self) -> Trapq {
        panic!("trapq not faked")
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        panic!("trapq_append not faked")
    }
    fn wipe_trapq(&self, _t: &Trapq) {}
    fn allocate_syncemitter(
        &self,
        _m: Mcu,
        _n: &str,
        _a: bool,
    ) -> KResult<SyncEmitterHandle> {
        Err(KlippyError::config("syncemitter not faked"))
    }
    fn queue_syncemitter_msg(&self, _s: &SyncEmitterHandle, _r: u64, _d: &[u32]) {}
    fn setup_mcu_movequeue(&self, _m: Mcu, _s: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _c: i64) {}
    fn register_flush_callback(&self, cb: FlushCallback, _c: bool) {
        self.inner.lock().flush_cbs.push(cb);
    }
    fn unregister_flush_callback(&self, _cb: &FlushCallback) {}
    fn register_flush_callback_named(&self, cb: FlushCallback, can_add_trapq: bool, _name: Option<&str>, _obj: Option<&str>) {
        self.register_flush_callback(cb, can_add_trapq);
    }
    fn get_kin_flush_delay(&self) -> f64 {
        0.0
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        Ok(())
    }
    fn calc_step_gen_restart(&self, _e: f64) -> f64 {
        0.0
    }
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        self.inner
            .lock()
            .activities
            .push((mq_time, is_step_gen));
    }
    fn drip_update_time(
        &self,
        _s: f64,
        _e: f64,
        _d: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}

klippy_api::printer_object!(FakeMq; dyn MotionQueuingApi);

struct FakeGCode {
    commands: klippy_api::GCell<Vec<String>>,
    mux: klippy_api::GCell<Vec<String>>,
}

impl FakeGCode {
    fn new() -> Arc<Self> {
        Arc::new(FakeGCode {
            commands: klippy_api::GCell::new(Vec::new()),
            mux: klippy_api::GCell::new(Vec::new()),
        })
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.commands.borrow_mut().push(cmd.to_string());
        let _ = opts;
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.borrow_mut().push(format!(
            "{cmd}:{key}:{}:{}",
            value.unwrap_or(""),
            desc.unwrap_or("")
        ));
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
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("mutex not used")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command not used")
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

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeSensor {
    report_delta: f64,
    minmax: klippy_api::GCell<Option<(f64, f64)>>,
    callback: klippy_api::GCell<Option<TemperatureCallback>>,
}

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        *self.minmax.borrow_mut() = Some((min_temp, max_temp));
        Ok(())
    }
    fn setup_callback(&self, callback: TemperatureCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.report_delta
    }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi);

/// Fake `heaters` registry carrying real `Heater` objects; only
/// `lookup_heater` is exercised along the `heater_fan` path, the remaining
/// methods mirror the registry semantics and are never called here.
struct FakeHeaters {
    heaters: IndexMap<String, Arc<Heater>>,
    factories: klippy_api::GCell<IndexMap<String, SensorFactory>>,
    monitors: klippy_api::GCell<Vec<String>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories
            .borrow_mut()
            .insert(sensor_type.to_string(), factory);
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        Err(KlippyError::config("setup_heater not used in heater_fan tests"))
    }
    fn get_all_heaters(&self) -> Vec<String> {
        self.heaters.keys().cloned().collect()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        match self.heaters.get(heater_name) {
            Some(h) => Ok(h.clone()),
            None => Err(KlippyError::config(format!(
                "Unknown heater '{heater_name}'"
            ))),
        }
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Err(KlippyError::config("setup_sensor not used in heater_fan tests"))
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn extras::heaters::SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        Err(KlippyError::config("register_sensor not used in heater_fan tests"))
    }
    fn register_monitor(&self, config: &Config) {
        self.monitors.borrow_mut().push(config.get_name());
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        for heater in self.heaters.values() {
            let _ = heater.set_temp(0.0);
        }
    }
    fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, _wait: bool) -> KResult<()> {
        heater.set_temp(temp)
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

#[derive(Clone)]
struct FakePrinterState {
    objects: IndexMap<String, ObjectRef>,
    reactor: Reactor,
    load_calls: Vec<String>,
}

struct FakePrinter {
    state: klippy_api::GCell<FakePrinterState>,
    handlers: KosMutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakePrinter {
            state: klippy_api::GCell::new(FakePrinterState {
                objects: IndexMap::new(),
                reactor,
                load_calls: Vec::new(),
            }),
            handlers: KosMutex::new(Vec::new()),
        })
    }
    fn load_calls(&self) -> Vec<String> {
        self.state.borrow().load_calls.clone()
    }
    fn ready_handler_names(&self) -> Vec<String> {
        self.handlers
            .lock()
            .iter()
            .filter(|(name, _)| name == "klippy:ready")
            .map(|(name, _)| name.clone())
            .collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("debugoutput".to_string()), PyValue::None);
        PyValue::Dict(map)
    }
    fn get_reactor(&self) -> Reactor {
        self.state.borrow().reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut st = self.state.borrow_mut();
        if st.objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        st.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let st = self.state.borrow();
        if let Some(o) = st.objects.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.state
            .borrow()
            .objects
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let mut st = self.state.borrow_mut();
        st.load_calls.push(section.to_string());
        if let Some(o) = st.objects.get(section).cloned() {
            return Ok(Some(o));
        }
        match default {
            LookupDefault::Raise => Ok(None),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut pending: Vec<EventCallback> = Vec::new();
        {
            let mut handlers = self.handlers.lock();
            let mut rest = Vec::new();
            for (name, cb) in std::mem::take(&mut *handlers) {
                if name == event {
                    pending.push(cb);
                } else {
                    rest.push((name, cb));
                }
            }
            *handlers = rest;
        }
        let mut results = Vec::new();
        for cb in pending.iter_mut() {
            results.push(cb(params)?);
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

#[derive(Clone, Debug, PartialEq)]
struct ReadRec {
    method: String,
    option: String,
    detail: String,
}

struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
    reads: klippy_api::GCell<Vec<ReadRec>>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, options: IndexMap<String, String>) -> Arc<Self> {
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            options,
            reads: klippy_api::GCell::new(Vec::new()),
        })
    }
    fn reads(&self) -> Vec<ReadRec> {
        self.reads.borrow().clone()
    }
    fn log(&self, method: &str, option: &str, detail: String) {
        self.reads.borrow_mut().push(ReadRec {
            method: method.to_string(),
            option: option.to_string(),
            detail,
        });
    }
}

fn fmt_opt_float(v: Option<f64>) -> String {
    match v {
        Some(f) => float_str(f),
        None => "None".to_string(),
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
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        if let Some(raw) = self.options.get(option) {
            let v = int_from_string(raw, 10).map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                ))
            })?;
            let parsed: i64 = v.inner().to_string().parse().unwrap_or(0);
            if let Some(minv) = opts.minval {
                if parsed < minv {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have minimum of {minv}",
                        self.name
                    )));
                }
            }
            if let Some(maxv) = opts.maxval {
                if parsed > maxv {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have maximum of {maxv}",
                        self.name
                    )));
                }
            }
            return Ok(Some(parsed));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let default_s = match opts.default {
            None => "sentinel".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => float_str(v),
        };
        self.log(
            "getfloat",
            option,
            format!(
                "{default_s}|{}|{}|{}|{}",
                fmt_opt_float(opts.minval),
                fmt_opt_float(opts.maxval),
                fmt_opt_float(opts.above),
                fmt_opt_float(opts.below)
            ),
        );
        if let Some(raw) = self.options.get(option) {
            let v = float_from_string(raw).map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                ))
            })?;
            if let Some(minv) = opts.minval {
                if v < minv {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have minimum of {}",
                        self.name,
                        float_str(minv)
                    )));
                }
            }
            if let Some(maxv) = opts.maxval {
                if v > maxv {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have maximum of {}",
                        self.name,
                        float_str(maxv)
                    )));
                }
            }
            if let Some(ab) = opts.above {
                if v <= ab {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be above {}",
                        self.name,
                        float_str(ab)
                    )));
                }
            }
            if let Some(be) = opts.below {
                if v >= be {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be below {}",
                        self.name,
                        float_str(be)
                    )));
                }
            }
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        if let Some(raw) = self.options.get(option) {
            let low = raw.to_lowercase();
            if ["1", "yes", "true", "on"].contains(&low.as_str()) {
                return Ok(Some(true));
            }
            if ["0", "no", "false", "off"].contains(&low.as_str()) {
                return Ok(Some(false));
            }
            return Err(KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'",
                self.name
            )));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let key: PyValue = if let Some(raw) = self.options.get(option) {
            PyValue::Str(raw.clone())
        } else {
            match opts.default {
                None => {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    )));
                }
                Some(d) => d.unwrap_or(PyValue::None),
            }
        };
        match choices {
            PyValue::Dict(m) => {
                if let Some(v) = m.get(&key) {
                    Ok(v.clone())
                } else {
                    Err(KlippyError::config(format!(
                        "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                        key.str(),
                        self.name
                    )))
                }
            }
            _ => Ok(PyValue::None),
        }
    }
    fn getlists(&self, _option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        Ok(opts.default.unwrap_or(Some(PyValue::None)).unwrap_or(PyValue::None))
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        let default_s = match &opts.default {
            None => "sentinel".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => v.join(","),
        };
        self.log("getlist", option, default_s);
        if let Some(raw) = self.options.get(option) {
            let raw = raw.as_str();
            if raw.trim().is_empty() {
                let parts: Vec<String> = Vec::new();
                if let Some(count) = opts.count {
                    if parts.len() != count {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have {count} elements",
                            self.name
                        )));
                    }
                }
                return Ok(Some(parts));
            }
            let parts: Vec<String> = raw.split(&opts.sep).map(|p| p.trim().to_string()).collect();
            if let Some(count) = opts.count {
                if parts.len() != count {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have {count} elements",
                        self.name
                    )));
                }
            }
            return Ok(Some(parts));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getintlist(&self, _option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(opts.default.unwrap())
    }
    fn getfloatlist(&self, _option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(opts.default.unwrap())
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection not used")
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

klippy_api::printer_object!(FakeConfig);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn pyvalue_to_value(value: &PyValue) -> Value {
    match value {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().expect("int fits i64")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::Host(id) => Value::Opaque {
            type_name: "Host".to_string(),
            addr: format!("0x{id:x}"),
        },
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("heater_fan fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping heater_fan oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.heater_fan".to_string(),
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

fn check_err_val<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(_) => panic!("{label}: oracle returned a value but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned a value but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

fn py_float_lit(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "float('inf')" } else { "float('-inf')" }.to_string()
    } else {
        format!("{v:?}")
    }
}

fn rust_opts(pairs: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut m = IndexMap::new();
    for (k, v) in pairs {
        m.insert(k.to_string(), v.to_string());
    }
    m
}

/// Only the three reads owned by `heater_fan` (fan option reads come from
/// the fan unit and are excluded here).
fn hf_reads(reads: &[ReadRec]) -> Vec<ReadRec> {
    reads
        .iter()
        .filter(|r| ["heater", "heater_temp", "fan_speed"].contains(&r.option.as_str()))
        .cloned()
        .collect()
}

fn reads_value(reads: &[ReadRec]) -> Value {
    Value::List(
        reads
            .iter()
            .map(|r| Value::Tuple(vec![Value::str(&r.method), Value::str(&r.option), Value::str(&r.detail)]))
            .collect(),
    )
}


fn py_values_lit(values: &IndexMap<String, String>) -> String {
    let parts: Vec<String> = values
        .iter()
        .map(|(k, v)| format!("{k:?}: {v:?}"))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn str_list_value(items: &[String]) -> Value {
    Value::List(items.iter().map(|s| Value::str(s)).collect())
}

fn float_list_value(items: &[f64]) -> Value {
    Value::List(items.iter().map(|f| Value::float(*f)).collect())
}

// ---------------------------------------------------------------------------
// Fixture builders.
// ---------------------------------------------------------------------------

/// One heater to register: `(name, current, target)`.
type HeaterSpec = (String, f64, f64);

/// Builds a matched Python/Rust pair. `values` drives the `heater_fan`
/// config (`heater`, `heater_temp`, `fan_speed` plus `pin`); `specs` drives
/// the registered heaters. When `with_heaters` is false the `heaters`
/// object is left unregistered (missing-object case).
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    values: &[(&str, &str)],
    specs: &[HeaterSpec],
    with_heaters: bool,
) -> Fixture {
    let values_map = rust_opts(values);
    let py_specs = specs
        .iter()
        .map(|(n, c, t)| format!("({n:?}, {}, {})", py_float_lit(*c), py_float_lit(*t)))
        .collect::<Vec<_>>()
        .join(", ");
    let db = if with_heaters { "True" } else { "False" };
    py.exec(&format!(
        "pair = make_pair([{py_specs}], {vals}, with_heaters={db})",
        py_specs = py_specs,
        vals = py_values_lit(&values_map),
        db = db
    ))
    .expect("py make_pair");

    // Rust side.
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(reactor.clone());
    let printer: Printer = printer_arc.clone();
    let pins = FakePins::new();
    let mq = Arc::new(FakeMq::default());
    let gcode = FakeGCode::new();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
        st.objects.insert("motion_queuing".to_string(), mq.clone() as ObjectRef);
        st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    }
    let mut heater_map = IndexMap::new();
    let mut heater_objs = Vec::new();
    for (name, cur, target) in specs {
        let heater = build_heater(&printer, &pins, name, *cur, *target);
        heater_map.insert(name.clone(), heater.clone());
        heater_objs.push(heater);
    }
    let heaters_obj = Arc::new(FakeHeaters {
        heaters: heater_map,
        factories: klippy_api::GCell::new(IndexMap::new()),
        monitors: klippy_api::GCell::new(Vec::new()),
    });
    if with_heaters {
        let mut st = printer_arc.state.borrow_mut();
        st.objects
            .insert("heaters".to_string(), heaters_obj.clone() as ObjectRef);
    }
    let mut cfg_map = values_map.clone();
    cfg_map.entry("pin".to_string()).or_insert_with(|| "PF7".to_string());
    let cfg = FakeConfig::new(printer.clone(), "heater_fan nozzle_fan", cfg_map);
    let cfg_cfg: Config = cfg.clone();
    let obj = PrinterHeaterFan::new(&cfg_cfg).expect("rust PrinterHeaterFan::new");
    Fixture {
        obj,
        printer: printer_arc,
        config: cfg,
        pins,
        heater_objs,
        heater_names: specs.iter().map(|(n, _, _)| n.clone()).collect(),
    }
}

fn build_heater(printer: &Printer, pins: &Arc<FakePins>, name: &str, cur: f64, target: f64) -> Arc<Heater> {
    let opts = rust_opts(&[
        ("min_temp", "0.0"),
        ("max_temp", "300.0"),
        ("heater_pin", "PA0"),
        ("control", "watermark"),
    ]);
    let _ = pins;
    let cfg = FakeConfig::new(
        printer.clone(),
        &format!("heater_generic {name}"),
        opts,
    );
    let sensor = Arc::new(FakeSensor {
        report_delta: 0.3,
        minmax: klippy_api::GCell::new(None),
        callback: klippy_api::GCell::new(None),
    });
    let cfg_cfg: Config = cfg.clone();
    let heater = Heater::new(&cfg_cfg, sensor as Arc<dyn TemperatureSensorApi>).expect("rust Heater::new");
    heater.set_temp(target).expect("rust set_temp");
    heater.temperature_callback(1000.0, cur).expect("temperature_callback");
    heater
}

struct Fixture {
    obj: PrinterHeaterFan,
    printer: Arc<FakePrinter>,
    config: Arc<FakeConfig>,
    pins: Arc<FakePins>,
    heater_objs: Vec<Arc<Heater>>,
    heater_names: Vec<String>,
}

impl Fixture {
    /// Drives both sides' heater `index` to `(cur, target)` at a fresh
    /// read time, keeping the two sides' `get_temp` identical.
    fn set_heater(&self, py: &mut pyoracle::Oracle, index: usize, cur: f64, target: f64, read_time: f64) {
        let name = self.heater_names[index].clone();
        py.exec(&format!(
            "pair[4]._heaters[{name:?}].set_temps({}, {})",
            py_float_lit(cur),
            py_float_lit(target)
        ))
        .expect("py set_temps");
        let heater = &self.heater_objs[index];
        heater.set_temp(target).expect("rust set_temp");
        heater.temperature_callback(read_time, cur).expect("temperature_callback");
    }

    fn fan_status_value(&self) -> Value {
        pyvalue_to_value(&self.obj.fan().get_status(1000.0))
    }

    fn fan_start_value(&self) -> f64 {
        let pwm = self.pins.pwm_for("PF7");
        let setups = pwm.setups.borrow().clone();
        let start = setups.iter().find(|s| s.starts_with("start_value:")).expect("fan start value set");
        let parts: Vec<&str> = start.split(':').collect();
        parts[2].parse::<f64>().expect("start value float")
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn pin_min_time_matches_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("_hf.PIN_MIN_TIME").expect("py PIN_MIN_TIME");
    check("PIN_MIN_TIME", Ok(expected), Value::float(PIN_MIN_TIME));
}

#[test]
fn module_factory_shape() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_some());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("hasattr(_hf, 'load_config') , hasattr(_hf, 'load_config_prefix')");
    check(
        "factory_shape",
        expected,
        Value::Tuple(vec![Value::Bool(false), Value::Bool(true)]),
    );
}

#[test]
fn construction_defaults() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[];
    let specs: Vec<HeaterSpec> = vec![("extruder".to_string(), 0.0, 0.0)];
    let fx = fresh_pair(&mut py, values, &specs, true);

    let expected_names = py.eval("list(pair[0].heater_names)").expect("py names");
    check("heater_names", Ok(expected_names), str_list_value(&fx.obj.heater_names()));
    let expected_ht = py.eval("float(pair[0].heater_temp)").expect("py heater_temp");
    check("heater_temp", Ok(expected_ht), Value::float(fx.obj.heater_temp()));
    let expected_fs = py.eval("float(pair[0].fan_speed)").expect("py fan_speed");
    check("fan_speed", Ok(expected_fs), Value::float(fx.obj.fan_speed()));
    let expected_ls = py.eval("float(pair[0].last_speed)").expect("py last_speed");
    check("last_speed", Ok(expected_ls), Value::float(fx.obj.last_speed()));

    // The Python `default_shutdown_speed` handoff is observable on the
    // Rust side as the fan pin start-value shutdown power.
    let expected_shutdown = py.eval("float(fan_inits[0])").expect("py fan inits");
    check("fan_shutdown_speed", Ok(expected_shutdown), Value::float(fx.fan_start_value()));

    let expected_loads = py.eval("list(pair[1].load_calls)").expect("py loads");
    let rust_loads = fx.printer.load_calls();
    // `load_interface` / `Heater::new` also load `motion_queuing` /
    // `verify_heater` / `pid_calibrate` on the Rust side; only the
    // `heaters` load belongs to this module.
    let rust_loads: Vec<String> = rust_loads.into_iter().filter(|n| n == "heaters").collect();
    let expected_loads = match expected_loads {
        Value::List(l) if l.len() == 1 => l,
        o => panic!("unexpected loads shape {o:?}"),
    };
    check(
        "load_calls",
        Ok(Value::List(expected_loads)),
        str_list_value(&rust_loads),
    );

    let py_ready_count = py
        .eval("len(pair[1].handlers.get('klippy:ready', []))")
        .expect("py ready");
    let rust_ready = fx.printer.ready_handler_names();
    assert_eq!(rust_ready.len(), 1, "klippy:ready registered once");
    check(
        "klippy_ready_count",
        Ok(py_ready_count),
        Value::int(rust_ready.len() as i64),
    );

    let expected_reads = py.eval("list(pair[2].reads)").expect("py reads");
    let rust_reads = hf_reads(&fx.config.reads());
    check("config_reads", Ok(expected_reads), reads_value(&rust_reads));
}

#[test]
fn construction_custom_values() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[
        ("heater", "extruder, heater_bed"),
        ("heater_temp", "60.0"),
        ("fan_speed", "0.6"),
    ];
    let specs: Vec<HeaterSpec> = vec![
        ("extruder".to_string(), 0.0, 0.0),
        ("heater_bed".to_string(), 0.0, 0.0),
    ];
    let fx = fresh_pair(&mut py, values, &specs, true);

    let expected_names = py.eval("list(pair[0].heater_names)").expect("py names");
    check("heater_names", Ok(expected_names), str_list_value(&fx.obj.heater_names()));
    let expected_ht = py.eval("float(pair[0].heater_temp)").expect("py heater_temp");
    check("heater_temp", Ok(expected_ht), Value::float(fx.obj.heater_temp()));
    let expected_fs = py.eval("float(pair[0].fan_speed)").expect("py fan_speed");
    check("fan_speed", Ok(expected_fs), Value::float(fx.obj.fan_speed()));

    let expected_reads = py.eval("list(pair[2].reads)").expect("py reads");
    let rust_reads = hf_reads(&fx.config.reads());
    check("config_reads", Ok(expected_reads), reads_value(&rust_reads));
}

#[test]
fn fan_speed_above_max_errors() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("fan_speed", "1.5")];
    let py_vals = py_values_lit(&rust_opts(values));
    let expected = py.eval(&format!(
        "make_pair([('extruder', 0.0, 0.0)], {py_vals}, with_heaters=True)"
    ));
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(reactor);
    let printer: Printer = printer_arc.clone();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), FakePins::new() as ObjectRef);
        st.objects.insert("motion_queuing".to_string(), Arc::new(FakeMq::default()) as ObjectRef);
        st.objects.insert("gcode".to_string(), FakeGCode::new() as ObjectRef);
        let heaters_obj = Arc::new(FakeHeaters {
            heaters: IndexMap::new(),
            factories: klippy_api::GCell::new(IndexMap::new()),
            monitors: klippy_api::GCell::new(Vec::new()),
        });
        st.objects.insert("heaters".to_string(), heaters_obj as ObjectRef);
    }
    let mut cfg_map = rust_opts(values);
    cfg_map.insert("pin".to_string(), "PF7".to_string());
    let cfg = FakeConfig::new(printer, "heater_fan nozzle_fan", cfg_map);
    let cfg_cfg: Config = cfg.clone();
    let actual = PrinterHeaterFan::new(&cfg_cfg).map(|_| ());
    check_err_val("fan_speed_max", expected, actual);
}

#[test]
fn fan_speed_below_min_errors() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("fan_speed", "-0.5")];
    let py_vals = py_values_lit(&rust_opts(values));
    let expected = py.eval(&format!(
        "make_pair([('extruder', 0.0, 0.0)], {py_vals}, with_heaters=True)"
    ));
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(reactor);
    let printer: Printer = printer_arc.clone();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), FakePins::new() as ObjectRef);
        st.objects.insert("motion_queuing".to_string(), Arc::new(FakeMq::default()) as ObjectRef);
        st.objects.insert("gcode".to_string(), FakeGCode::new() as ObjectRef);
        let heaters_obj = Arc::new(FakeHeaters {
            heaters: IndexMap::new(),
            factories: klippy_api::GCell::new(IndexMap::new()),
            monitors: klippy_api::GCell::new(Vec::new()),
        });
        st.objects.insert("heaters".to_string(), heaters_obj as ObjectRef);
    }
    let mut cfg_map = rust_opts(values);
    cfg_map.insert("pin".to_string(), "PF7".to_string());
    let cfg = FakeConfig::new(printer, "heater_fan nozzle_fan", cfg_map);
    let cfg_cfg: Config = cfg.clone();
    let actual = PrinterHeaterFan::new(&cfg_cfg).map(|_| ());
    check_err_val("fan_speed_min", expected, actual);
}

#[test]
fn heater_temp_parse_error() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("heater_temp", "abc")];
    let py_vals = py_values_lit(&rust_opts(values));
    let expected = py.eval(&format!(
        "make_pair([('extruder', 0.0, 0.0)], {py_vals}, with_heaters=True)"
    ));
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(reactor);
    let printer: Printer = printer_arc.clone();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), FakePins::new() as ObjectRef);
        st.objects.insert("motion_queuing".to_string(), Arc::new(FakeMq::default()) as ObjectRef);
        st.objects.insert("gcode".to_string(), FakeGCode::new() as ObjectRef);
        let heaters_obj = Arc::new(FakeHeaters {
            heaters: IndexMap::new(),
            factories: klippy_api::GCell::new(IndexMap::new()),
            monitors: klippy_api::GCell::new(Vec::new()),
        });
        st.objects.insert("heaters".to_string(), heaters_obj as ObjectRef);
    }
    let mut cfg_map = rust_opts(values);
    cfg_map.insert("pin".to_string(), "PF7".to_string());
    let cfg = FakeConfig::new(printer, "heater_fan nozzle_fan", cfg_map);
    let cfg_cfg: Config = cfg.clone();
    let actual = PrinterHeaterFan::new(&cfg_cfg).map(|_| ());
    check_err_val("heater_temp_parse", expected, actual);
}

#[test]
fn handle_ready_registers_timer() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[];
    let specs: Vec<HeaterSpec> = vec![("extruder".to_string(), 0.0, 0.0)];
    let fx = fresh_pair(&mut py, values, &specs, true);

    let expected = py.eval("pair[0].handle_ready()");
    check("py_ready_none", expected, Value::Null);
    let _wakes = py.eval("[w for (_cb, w) in pair[3].timers]").expect("py wakes");
    let wake0 = py.eval("pair[3].timers[0][1]").expect("py wake0");
    let exp_wake = py.eval("1000.0 + _hf.PIN_MIN_TIME").expect("py exp wake");
    check("timer_waketime", Ok(exp_wake), wake0);

    fx.obj.handle_ready().expect("rust handle_ready");
    let waking = py.eval("len(pair[3].timers)").expect("py timer count");
    check("timer_count", Ok(waking), Value::int(1));

    // The event handler path resolves the same heaters without error.
    let expected = py.eval("pair[1].send_event('klippy:ready')");
    check("py_send_ready", expected, Value::List(vec![Value::Null]));
    let fired = fx.printer.send_event("klippy:ready", &[]).expect("rust send ready");
    assert_eq!(fired.len(), 1);
    assert_eq!(pyvalue_to_value(&fired[0]), Value::Null);
}

#[test]
fn handle_ready_unknown_heater_errors() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("heater", "extruder")];
    let mut py_vals_map = rust_opts(values);
    py_vals_map.insert("pin".to_string(), "PF7".to_string());
    let py_vals = py_values_lit(&py_vals_map);
    py.exec(&format!(
        "pair = make_pair([('other', 0.0, 0.0)], {py_vals}, with_heaters=True)"
    ))
    .expect("py make");
    let expected = py.eval("pair[0].handle_ready()");

    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(reactor);
    let printer: Printer = printer_arc.clone();
    let pins = FakePins::new();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
        st.objects.insert("motion_queuing".to_string(), Arc::new(FakeMq::default()) as ObjectRef);
        st.objects.insert("gcode".to_string(), FakeGCode::new() as ObjectRef);
    }
    let heater = build_heater(&printer, &pins, "other", 0.0, 0.0);
    let mut heater_map = IndexMap::new();
    heater_map.insert("other".to_string(), heater);
    let heaters_obj = Arc::new(FakeHeaters {
        heaters: heater_map,
        factories: klippy_api::GCell::new(IndexMap::new()),
        monitors: klippy_api::GCell::new(Vec::new()),
    });
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters_obj as ObjectRef);
    }
    let mut cfg_map = rust_opts(values);
    cfg_map.insert("pin".to_string(), "PF7".to_string());
    let cfg = FakeConfig::new(printer, "heater_fan nozzle_fan", cfg_map);
    let cfg_cfg: Config = cfg.clone();
    let obj = PrinterHeaterFan::new(&cfg_cfg).expect("rust new");
    check_err_val("unknown_heater_ready", expected, obj.handle_ready());
}

#[test]
fn handle_ready_missing_heaters_errors() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[];
    let mut py_vals_map = rust_opts(values);
    py_vals_map.insert("pin".to_string(), "PF7".to_string());
    let py_vals = py_values_lit(&py_vals_map);
    py.exec(&format!(
        "pair = make_pair([('extruder', 0.0, 0.0)], {py_vals}, with_heaters=False)"
    ))
    .expect("py make");
    let expected = py.eval("pair[0].handle_ready()");

    let fx = {
        let reactor = Reactor::new(false);
        let printer_arc = FakePrinter::new(reactor);
        let printer: Printer = printer_arc.clone();
        {
            let mut st = printer_arc.state.borrow_mut();
            st.objects.insert("pins".to_string(), FakePins::new() as ObjectRef);
            st.objects.insert("motion_queuing".to_string(), Arc::new(FakeMq::default()) as ObjectRef);
            st.objects.insert("gcode".to_string(), FakeGCode::new() as ObjectRef);
        }
        let mut cfg_map = rust_opts(values);
        cfg_map.insert("pin".to_string(), "PF7".to_string());
        let cfg = FakeConfig::new(printer, "heater_fan nozzle_fan", cfg_map);
        let cfg_cfg: Config = cfg.clone();
        let obj = PrinterHeaterFan::new(&cfg_cfg).expect("rust new");
        (obj, printer_arc)
    };
    check_err_val("missing_heaters_ready", expected, fx.0.handle_ready());
}

#[test]
fn callback_truth_table() {
    // (current, target) -> expected fan on/off at the default 50C threshold.
    let cases: &[(f64, f64, bool)] = &[
        (30.0, 0.0, false),
        (30.0, 200.0, true),
        (60.0, 0.0, true),
        (50.0, 0.0, false),
        (50.0001, 0.0, true),
        (49.9999, 0.0, false),
        (0.0, 0.0, false),
        (0.0, 1.0, true),
        (100.0, 0.0, true),
    ];
    for (index, (cur, target, expect_on)) in cases.iter().enumerate() {
        let Some(mut py) = oracle() else { return };
        let values: &[(&str, &str)] = &[];
        let specs: Vec<HeaterSpec> = vec![("extruder".to_string(), 0.0, 0.0)];
        let fx = fresh_pair(&mut py, values, &specs, true);
        // The reference resolves heaters only in handle_ready, so a
        // callback before it sees no heaters and never drives the fan.
        py.exec("pair[0].handle_ready()").expect("py handle_ready");
        fx.obj.handle_ready().expect("rust handle_ready");
        fx.set_heater(&mut py, 0, *cur, *target, 1001.0);
        let eventtime = 1000.0;
        let expected_next = py.eval(&format!("pair[0].callback({eventtime:?})")).expect("py cb");
        let expected_speeds = py.eval("list(pair[0].fan.speeds)").expect("py speeds");
        let expected_status = py.eval("pair[0].get_status(1000.0)").expect("py status");
        let expected_ls = py.eval("float(pair[0].last_speed)").expect("py ls");

        let before = fx.obj.last_speed();
        let next = fx.obj.callback(eventtime);
        let rust_speeds: Vec<f64> = if fx.obj.last_speed() != before {
            vec![fx.obj.last_speed()]
        } else {
            Vec::new()
        };
        let label = format!("cb_{index} on={expect_on}");
        check(&format!("{label} next"), Ok(expected_next), Value::float(next));
        check(&format!("{label} last_speed"), Ok(expected_ls), Value::float(fx.obj.last_speed()));
        check(&format!("{label} speeds"), Ok(expected_speeds), float_list_value(&rust_speeds));
        check(&format!("{label} status"), Ok(expected_status), fx.fan_status_value());
    }
}

#[test]
fn callback_custom_threshold_and_speed() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("heater_temp", "60.0"), ("fan_speed", "0.6")];
    let specs: Vec<HeaterSpec> = vec![("extruder".to_string(), 55.0, 0.0)];
    let fx = fresh_pair(&mut py, values, &specs, true);
    py.exec("pair[0].handle_ready()").expect("py handle_ready");
    fx.obj.handle_ready().expect("rust handle_ready");
    let mut rust_speeds: Vec<f64> = Vec::new();

    // Below the custom threshold with no target: off.
    let expected_next = py.eval("pair[0].callback(1000.0)").expect("py cb off");
    let expected_speeds = py.eval("list(pair[0].fan.speeds)").expect("py speeds off");
    let before = fx.obj.last_speed();
    let next_rust = fx.obj.callback(1000.0);
    if fx.obj.last_speed() != before {
        rust_speeds.push(fx.obj.last_speed());
    }
    check("custom_off_next", Ok(expected_next), Value::float(next_rust));
    check("custom_off_speeds", Ok(expected_speeds), float_list_value(&rust_speeds));

    // Above the custom threshold: on at the custom speed.
    fx.set_heater(&mut py, 0, 61.0, 0.0, 1001.0);
    let expected_next = py.eval("pair[0].callback(1001.0)").expect("py cb on");
    let expected_speeds = py.eval("list(pair[0].fan.speeds)").expect("py speeds on");
    let expected_status = py.eval("pair[0].get_status(1001.0)").expect("py status on");
    let before = fx.obj.last_speed();
    let next_rust = fx.obj.callback(1001.0);
    if fx.obj.last_speed() != before {
        rust_speeds.push(fx.obj.last_speed());
    }
    check("custom_on_next", Ok(expected_next), Value::float(next_rust));
    check("custom_on_speeds", Ok(expected_speeds), float_list_value(&rust_speeds));
    check("custom_on_status", Ok(expected_status), fx.fan_status_value());
}

#[test]
fn callback_two_heaters_or_logic() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("heater", "extruder, heater_bed")];
    let specs: Vec<HeaterSpec> = vec![
        ("extruder".to_string(), 0.0, 0.0),
        ("heater_bed".to_string(), 0.0, 0.0),
    ];
    let fx = fresh_pair(&mut py, values, &specs, true);
    py.exec("pair[0].handle_ready()").expect("py handle_ready");
    fx.obj.handle_ready().expect("rust handle_ready");
    let mut rust_speeds: Vec<f64> = Vec::new();

    // Both off.
    let expected_next = py.eval("pair[0].callback(1000.0)").expect("py cb both off");
    let before = fx.obj.last_speed();
    let next_rust = fx.obj.callback(1000.0);
    if fx.obj.last_speed() != before {
        rust_speeds.push(fx.obj.last_speed());
    }
    check("both_off_next", Ok(expected_next), Value::float(next_rust));
    assert_eq!(fx.obj.last_speed(), 0.0);

    // Second heater turns on by target.
    fx.set_heater(&mut py, 1, 25.0, 200.0, 1001.0);
    let expected_next = py.eval("pair[0].callback(1001.0)").expect("py cb one on");
    let expected_speeds = py.eval("list(pair[0].fan.speeds)").expect("py speeds one on");
    let expected_status = py.eval("pair[0].get_status(1001.0)").expect("py status one on");
    let before = fx.obj.last_speed();
    let next_rust = fx.obj.callback(1001.0);
    if fx.obj.last_speed() != before {
        rust_speeds.push(fx.obj.last_speed());
    }
    check("one_on_next", Ok(expected_next), Value::float(next_rust));
    check("one_on_speeds", Ok(expected_speeds), float_list_value(&rust_speeds));
    check("one_on_status", Ok(expected_status), fx.fan_status_value());
}

#[test]
fn callback_transitions_only_on_change() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[];
    let specs: Vec<HeaterSpec> = vec![("extruder".to_string(), 0.0, 0.0)];
    let fx = fresh_pair(&mut py, values, &specs, true);
    py.exec("pair[0].handle_ready()").expect("py handle_ready");
    fx.obj.handle_ready().expect("rust handle_ready");

    // Off, off, on, on, off, off: set_speed fires only on changes.
    let phases: &[(f64, f64, f64)] = &[
        (10.0, 0.0, 1000.0),
        (20.0, 0.0, 1001.0),
        (20.0, 210.0, 1002.0),
        (25.0, 210.0, 1003.0),
        (10.0, 0.0, 1004.0),
        (10.0, 0.0, 1005.0),
    ];
    let mut rust_speeds: Vec<f64> = Vec::new();
    for (i, (cur, target, eventtime)) in phases.iter().enumerate() {
        fx.set_heater(&mut py, 0, *cur, *target, 1001.0 + i as f64);
        let expected_next = py.eval(&format!("pair[0].callback({eventtime:?})")).expect("py cb");
        let expected_speeds = py.eval("list(pair[0].fan.speeds)").expect("py speeds");
        let expected_ls = py.eval("float(pair[0].last_speed)").expect("py ls");
        let before = fx.obj.last_speed();
        let next = fx.obj.callback(*eventtime);
        if fx.obj.last_speed() != before {
            rust_speeds.push(fx.obj.last_speed());
        }
        let label = format!("transition_{i}");
        check(&format!("{label} next"), Ok(expected_next), Value::float(next));
        check(&format!("{label} last_speed"), Ok(expected_ls), Value::float(fx.obj.last_speed()));
        check(&format!("{label} speeds"), Ok(expected_speeds), float_list_value(&rust_speeds));
    }
}

#[test]
fn load_config_prefix_returns_object() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[];
    let py_vals = py_values_lit(&rust_opts(values));
    let expected = py.eval(&format!(
        "make_pair([('extruder', 0.0, 0.0)], {py_vals}, with_heaters=True)[0].get_status(1000.0)"
            ));
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(reactor);
    let printer: Printer = printer_arc.clone();
    let pins = FakePins::new();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
        st.objects.insert("motion_queuing".to_string(), Arc::new(FakeMq::default()) as ObjectRef);
        st.objects.insert("gcode".to_string(), FakeGCode::new() as ObjectRef);
    }
    let heater = build_heater(&printer, &pins, "extruder", 0.0, 0.0);
    let mut heater_map = IndexMap::new();
    heater_map.insert("extruder".to_string(), heater);
    let heaters_obj = Arc::new(FakeHeaters {
        heaters: heater_map,
        factories: klippy_api::GCell::new(IndexMap::new()),
        monitors: klippy_api::GCell::new(Vec::new()),
    });
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters_obj as ObjectRef);
    }
    let mut cfg_map = rust_opts(values);
    cfg_map.insert("pin".to_string(), "PF7".to_string());
    let cfg = FakeConfig::new(printer, "heater_fan nozzle_fan", cfg_map);
    let cfg_cfg: Config = cfg.clone();
    let loaded = load_config_prefix(&cfg_cfg).expect("rust load").expect("loaded object");
    let boxed: Arc<PrinterHeaterFan> = klippy_api::downcast_arc(&loaded).expect("downcast");
    check("prefix_status", expected, pyvalue_to_value(&boxed.get_status(1000.0)));
    // The factory entry point is the same function.
    let f = module_factory();
    assert!(f.load_config_prefix.is_some());
    let loaded2 = (f.load_config_prefix.unwrap())(&cfg_cfg).expect("factory load");
    assert!(loaded2.is_some());
}
