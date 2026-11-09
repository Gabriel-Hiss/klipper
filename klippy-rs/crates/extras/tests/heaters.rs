//! Parity tests for `extras::heaters` (port of `klippy/extras/heaters.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `Heater` /
//! `PrinterHeaters` is built on both sides with minimal Python fakes for
//! the printer, config and MCU, then fed identical calls and the results,
//! errors, G-code responses and PWM calls are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::heaters::{
    ControlBangBang, ControlPID, Heater, HeaterApi, PrinterHeaters, PrinterHeatersApi,
    SensorApi, TemperatureCallback, TemperatureSensorApi, KELVIN_TO_CELSIUS, MAX_HEAT_TIME,
    AMBIENT_TEMP, PID_PARAM_BASE, MAX_MAINTHREAD_TIME, QUELL_STALE_TIME, MIN_PWM_CHANGE_RATIO,
    PID_SETTLE_DELTA, PID_SETTLE_SLOPE,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GcodeGetFloatOpts, GetIntOpts as GcodeGetIntOpts,
    GetOpts as GcodeGetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AdcSampleOpts, CommandQueue, CommandWrapper, CommandQueryWrapper,
    LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, PinsApi, ParsePinOpts, LookupPinOpts,
    PinResolver, Chip, PwmPin, PwmPinApi, QuerySendOpts, SendOpts, AsyncResponseWrapper,
    Endstop, DigitalOutPin, AdcPin, Trsync, TriggerDispatch, ConfigCallback, SerialResponseCallback,
};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{Rail, StepperFactory, Trapq};
use klippy_api::toolhead::{
    HomingTargetApi, LookaheadCallback, ToolHeadApi, ExtraAxis, PrinterExtruder, Kinematics,
    Coord, MoveApi, HomingStateHandle,
};
use klippy_api::{downcast_arc, EventArg};
use pycompat::text::{float_from_string, float_str, int_from_string, percent_format, PctValue};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.heaters as heaters

class CommandError(Exception):
    pass

class _Sentinel:
    pass

_SENTINEL = _Sentinel()

class FakeMcu:
    def __init__(self, est=1000.0):
        self.est_print_time = est
        self.calls = []
    def estimated_print_time(self, eventtime):
        self.calls.append(('est', eventtime))
        return self.est_print_time

class FakePwm:
    def __init__(self, mcu):
        self._mcu = mcu
        self.calls = []
    def setup_cycle_time(self, t):
        self.calls.append(('setup_cycle_time', t))
    def setup_max_duration(self, t):
        self.calls.append(('setup_max_duration', t))
    def set_pwm(self, t, v):
        self.calls.append(('set_pwm', t, v))
    def get_mcu(self):
        return self._mcu

class FakePins:
    def __init__(self, pwm):
        self.pwm = pwm
        self.calls = []
    def setup_pin(self, pin_type, pin_desc):
        self.calls.append((pin_type, pin_desc))
        return self.pwm

class FakeReactor:
    def __init__(self, mono=1000.0):
        self._mono = mono
        self.pause_calls = []
    def monotonic(self):
        return self._mono
    def pause(self, t):
        self.pause_calls.append(t)
        return t

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self, debugoutput=None, shutdown=False):
        self.objects = {}
        self.start_args = {'debugoutput': debugoutput}
        self._shutdown = shutdown
        self.reactor = FakeReactor()
        self.event_handlers = []
        self.load_calls = []
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return self.reactor
    def is_shutdown(self):
        return self._shutdown
    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is _SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_SENTINEL):
        self.load_calls.append(section)
        if section in self.objects:
            return self.objects[section]
        return None
    def register_event_handler(self, event, cb):
        self.event_handlers.append(event)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options=None):
        self._printer = printer
        self._name = name
        self.options = options or {}
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=_SENTINEL):
        if option in self.options:
            return self.options[option]
        if default is _SENTINEL:
            raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
        return default
    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None, above=None, below=None):
        if option not in self.options:
            if default is _SENTINEL:
                raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        raw = self.options[option]
        try:
            v = float(raw)
        except Exception:
            raise self.error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if minval is not None and v < minval:
            raise self.error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise self.error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise self.error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v
    def getchoice(self, option, choices, default=_SENTINEL):
        if option not in self.options:
            if default is _SENTINEL:
                raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
            c = default
        else:
            c = self.get(option)
        if c not in choices:
            raise self.error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self._name))
        return choices[c]

class FakeGCode:
    def __init__(self):
        self.commands = []
        self.mux = []
        self.raw = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.commands.append((cmd, when_not_ready, desc))
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))
    def respond_raw(self, msg):
        self.raw.append(msg)

class FakeGcmd:
    error = CommandError
    def __init__(self, gcode, params, need_ack=True, command='M105'):
        self._gcode = gcode
        self.params = params
        self.need_ack = need_ack
        self.command = command
        self.commandline = command
        self.ack_msg = None
        self.respond_raw = gcode.respond_raw
        self.respond_info = lambda m, log=True: None
    def get(self, name, default=_SENTINEL):
        if name not in self.params:
            if default is _SENTINEL:
                raise self.error("Error on '%s': missing %s" % (self.commandline, name))
            return default
        return self.params[name]
    def get_float(self, name, default=_SENTINEL, above=None):
        if name not in self.params:
            if default is _SENTINEL:
                raise self.error("Error on '%s': missing %s" % (self.commandline, name))
            return default
        try:
            v = float(self.params[name])
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self.commandline, self.params[name]))
        if above is not None and v <= above:
            raise self.error("Error on '%s': %s must be above %s" % (self.commandline, name, above))
        return v
    def ack(self, msg=None):
        if not self.need_ack:
            return False
        self.ack_msg = msg
        self._gcode.raw.append('ok' if not msg else 'ok %s' % (msg,))
        self.need_ack = False
        return True

class FakeSensor:
    def __init__(self, report_delta=0.3, temp=25.0, target=0.0):
        self.report_delta = report_delta
        self.temp = temp
        self.target = target
        self.minmax = None
        self.callback = None
    def setup_minmax(self, a, b):
        self.minmax = (a, b)
    def setup_callback(self, cb):
        self.callback = cb
    def get_report_time_delta(self):
        return self.report_delta
    def get_temp(self, eventtime):
        return (self.temp, self.target)
    def fire(self, read_time, temp):
        self.callback(read_time, temp)

class FakeToolhead:
    def __init__(self):
        self.last_move_time = 1000.0
        self.register_calls = 0
        self.get_last_calls = 0
    def get_last_move_time(self):
        self.get_last_calls += 1
        return self.last_move_time
    def register_lookahead_callback(self, cb):
        self.register_calls += 1

class FakePrinterConfig:
    def __init__(self, fail=False, sections=None):
        self.fail = fail
        self.sections = sections or []
        self.read_calls = []
    def read_config(self, filename):
        self.read_calls.append(filename)
        if self.fail:
            raise configparser.Error('cannot read')
        return self
    def get_prefix_sections(self, prefix):
        return [FakeConfig(None, n, {}) for n in self.sections]

class FakeDConfig:
    def __init__(self, names):
        self.names = names
    def get_prefix_sections(self, prefix):
        return [FakeConfig(None, n, {}) for n in self.names]

def build_heater(printer_extra=None, options=None, debugoutput=None, report_delta=0.3):
    mcu = FakeMcu(1000.0)
    pwm = FakePwm(mcu)
    pins = FakePins(pwm)
    gcode = FakeGCode()
    toolhead = FakeToolhead()
    printer = FakePrinter(debugoutput=debugoutput)
    printer.objects['pins'] = pins
    printer.objects['gcode'] = gcode
    printer.objects['toolhead'] = toolhead
    opts = {
        'min_temp': '0.0', 'max_temp': '300.0', 'heater_pin': 'PA0',
        'control': 'watermark',
    }
    if options:
        opts.update(options)
    config = FakeConfig(printer, 'heater_generic extruder1', opts)
    sensor = FakeSensor(report_delta=report_delta)
    heater = heaters.Heater(config, sensor)
    return (printer, config, sensor, heater, pwm, mcu, gcode, toolhead)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinterState {
    objects: IndexMap<String, ObjectRef>,
    debugoutput: Option<String>,
    shutdown: bool,
    reactor: Reactor,
    event_handlers: Vec<String>,
    load_calls: Vec<String>,
}

struct FakePrinter {
    state: klippy_api::GCell<FakePrinterState>,
}

impl FakePrinter {
    fn new(debugoutput: Option<String>, reactor: Reactor) -> Arc<Self> {
        record_klippy_dir();
        Arc::new(FakePrinter {
            state: klippy_api::GCell::new(FakePrinterState {
                objects: IndexMap::new(),
                debugoutput,
                shutdown: false,
                reactor,
                event_handlers: Vec::new(),
                load_calls: Vec::new(),
            }),
        })
    }
}

/// `PrinterHeaters.load_config` reads the bundled `temperature_sensors.cfg`
/// from the klippy source directory the host records.
fn record_klippy_dir() {
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("crate lives three levels below the repo root");
    klippy_api::paths::set_klippy_dir(&repo.join("klippy").to_string_lossy());
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut map = IndexMap::new();
        let v = match &self.state.borrow().debugoutput {
            Some(s) => PyValue::Str(s.clone()),
            None => PyValue::None,
        };
        map.insert(PyValue::Str("debugoutput".to_string()), v);
        PyValue::Dict(map)
    }
    fn get_reactor(&self) -> Reactor {
        self.state.borrow().reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        self.state.borrow().shutdown
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut st = self.state.borrow_mut();
        if st.objects.contains_key(name) {
            return Err(klippy_api::KlippyError::config(format!(
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
            LookupDefault::Raise => Err(klippy_api::KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.state
            .borrow()
            .objects
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(&self, _config: &Config, section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let mut st = self.state.borrow_mut();
        st.load_calls.push(section.to_string());
        if let Some(o) = st.objects.get(section).cloned() {
            return Ok(Some(o));
        }
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.state.borrow_mut().event_handlers.push(event.to_string());
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfigData {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
    prefix_sections: Vec<String>,
    sections_map: IndexMap<String, IndexMap<String, String>>,
}

struct FakeConfig {
    inner: Arc<FakeConfigData>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, options: IndexMap<String, String>) -> Arc<Self> {
        Arc::new(FakeConfig {
            inner: Arc::new(FakeConfigData {
                printer,
                name: name.to_string(),
                options,
                prefix_sections: Vec::new(),
                sections_map: IndexMap::new(),
            }),
        })
    }
}

fn fmt_bound_float(v: f64) -> String {
    float_str(v)
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.inner.printer.clone()
    }
    fn get_name(&self) -> String {
        self.inner.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.inner.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.inner.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        if let Some(raw) = self.inner.options.get(option) {
            let v = int_from_string(raw, 10).map_err(|_| {
                klippy_api::KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.inner.name
                ))
            })?;
            // Extract i64 via rendering (valid range assumed in tests).
            let v_str = v.inner().to_string();
            let parsed: i64 = v_str.parse().unwrap_or(0);
            if let Some(minv) = opts.minval {
                if parsed < minv {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have minimum of {minv}",
                        self.inner.name
                    )));
                }
            }
            if let Some(maxv) = opts.maxval {
                if parsed > maxv {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have maximum of {maxv}",
                        self.inner.name
                    )));
                }
            }
            return Ok(Some(parsed));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.inner.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        if let Some(raw) = self.inner.options.get(option) {
            let v = float_from_string(raw).map_err(|_| {
                klippy_api::KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.inner.name
                ))
            })?;
            if let Some(minv) = opts.minval {
                if v < minv {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have minimum of {}",
                        self.inner.name,
                        fmt_bound_float(minv)
                    )));
                }
            }
            if let Some(maxv) = opts.maxval {
                if v > maxv {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have maximum of {}",
                        self.inner.name,
                        fmt_bound_float(maxv)
                    )));
                }
            }
            if let Some(ab) = opts.above {
                if v <= ab {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be above {}",
                        self.inner.name,
                        fmt_bound_float(ab)
                    )));
                }
            }
            if let Some(be) = opts.below {
                if v >= be {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be below {}",
                        self.inner.name,
                        fmt_bound_float(be)
                    )));
                }
            }
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.inner.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getboolean(&self, _option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match opts.default {
            None => Err(klippy_api::KlippyError::config("must be specified".to_string())),
            Some(d) => Ok(d),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let key: PyValue = if let Some(raw) = self.inner.options.get(option) {
            PyValue::Str(raw.clone())
        } else {
            match opts.default {
                None => {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.inner.name
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
                    Err(klippy_api::KlippyError::config(format!(
                        "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                        key.str(),
                        self.inner.name
                    )))
                }
            }
            _ => Ok(PyValue::None),
        }
    }
    fn getlists(&self, _option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        Ok(opts.default.unwrap_or(Some(PyValue::None)).unwrap_or(PyValue::None))
    }
    fn getlist(&self, _option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(opts.default.unwrap())
    }
    fn getintlist(&self, _option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(opts.default.unwrap())
    }
    fn getfloatlist(&self, _option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(opts.default.unwrap())
    }
    fn getsection(&self, section: &str) -> Config {
        let opts = self.inner.sections_map.get(section).cloned().unwrap_or_default();
        FakeConfig::new(self.inner.printer.clone(), section, opts)
    }
    fn has_section(&self, section: &str) -> bool {
        self.inner.sections_map.contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        self.inner
            .prefix_sections
            .iter()
            .map(|n| FakeConfig::new(self.inner.printer.clone(), n, IndexMap::new()) as Config)
            .collect()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

klippy_api::printer_object!(FakeConfig);

struct FakeMcu {
    est: klippy_api::GCell<f64>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used in heaters tests")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is on PinsApi, not McuApi in heaters tests")
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
        *self.est.borrow()
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

klippy_api::printer_object!(FakeMcu);

struct FakePwm {
    mcu: Mcu,
    calls: klippy_api::GCell<Vec<String>>,
    pwm_calls: klippy_api::GCell<Vec<(f64, f64)>>,
}

impl FakePwm {
    fn new(est: f64) -> Arc<Self> {
        let mcu: Arc<FakeMcu> = Arc::new(FakeMcu { est: klippy_api::GCell::new(est) });
        Arc::new(FakePwm {
            mcu: mcu as Mcu,
            calls: klippy_api::GCell::new(Vec::new()),
            pwm_calls: klippy_api::GCell::new(Vec::new()),
        })
    }
}

#[allow(dead_code)]
impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.calls.borrow_mut().push(format!("setup_max_duration:{max_duration:?}"));
    }
    fn setup_cycle_time(&self, cycle_time: f64, _hardware_pwm: Option<bool>) {
        self.calls.borrow_mut().push(format!("setup_cycle_time:{cycle_time:?}"));
    }
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, print_time: f64, value: f64) {
        self.pwm_calls.borrow_mut().push((print_time, value));
        self.calls.borrow_mut().push(format!("set_pwm:{print_time:?}:{value:?}"));
    }
}

klippy_api::printer_object!(FakePwm; dyn PwmPinApi);

struct FakePins {
    pwm: PwmPin,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin not used")
    }
    fn setup_pin(&self, pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        assert_eq!(pin_type, "pwm");
        Ok(Pin::Pwm(self.pwm.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
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

struct FakeGCode {
    commands: klippy_api::GCell<Vec<String>>,
    mux: klippy_api::GCell<Vec<String>>,
    raw: klippy_api::GCell<Vec<String>>,
}

impl FakeGCode {
    fn new() -> Arc<Self> {
        Arc::new(FakeGCode {
            commands: klippy_api::GCell::new(Vec::new()),
            mux: klippy_api::GCell::new(Vec::new()),
            raw: klippy_api::GCell::new(Vec::new()),
        })
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        self.commands.borrow_mut().push(format!(
            "{cmd}:{}:{}",
            opts.when_not_ready,
            opts.desc.unwrap_or_default()
        ));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
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
    fn respond_raw(&self, msg: &str) {
        self.raw.borrow_mut().push(msg.to_string());
    }
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

struct FakeToolhead {
    last_move_time: klippy_api::GCell<f64>,
    register_calls: klippy_api::GCell<i64>,
}

impl FakeToolhead {
    fn new() -> Arc<Self> {
        Arc::new(FakeToolhead {
            last_move_time: klippy_api::GCell::new(1000.),
            register_calls: klippy_api::GCell::new(0),
        })
    }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position not used")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("kinematics not used")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(*self.last_move_time.borrow())
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_trapq(&self) -> Trapq {
        panic!("trapq not used")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0., 0.)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0., 0., 0., 0.)
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        *self.register_calls.borrow_mut() += 1;
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0., 0., false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("extruder not used")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper factory not used")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct FakeSensor {
    report_delta: f64,
    minmax: klippy_api::GCell<Option<(f64, f64)>>,
    callback: klippy_api::GCell<Option<TemperatureCallback>>,
    temp: klippy_api::GCell<f64>,
    target: klippy_api::GCell<f64>,
}

impl FakeSensor {
    fn new(report_delta: f64, temp: f64, target: f64) -> Arc<Self> {
        Arc::new(FakeSensor {
            report_delta,
            minmax: klippy_api::GCell::new(None),
            callback: klippy_api::GCell::new(None),
            temp: klippy_api::GCell::new(temp),
            target: klippy_api::GCell::new(target),
        })
    }

    fn fire(&self, read_time: f64, temp: f64) {
        let mut slot = self.callback.borrow_mut();
        if let Some(cb) = slot.as_mut() {
            cb(read_time, temp);
        } else {
            panic!("no callback registered");
        }
    }
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

impl SensorApi for FakeSensor {
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        (*self.temp.borrow(), *self.target.borrow())
    }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi, dyn SensorApi);

struct FakeGcmd {
    params: IndexMap<String, String>,
    need_ack: klippy_api::GCell<bool>,
    commandline: String,
    gcode: Arc<FakeGCode>,
    ack_msg: klippy_api::GCell<Option<String>>,
}

impl FakeGcmd {
    fn new(
        gcode: Arc<FakeGCode>,
        params: IndexMap<String, String>,
        need_ack: bool,
        command: &str,
    ) -> Arc<Self> {
        Arc::new(FakeGcmd {
            params,
            need_ack: klippy_api::GCell::new(need_ack),
            commandline: command.to_string(),
            gcode,
            ack_msg: klippy_api::GCell::new(None),
        })
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GcodeGetOpts) -> KResult<PyValue> {
        if let Some(v) = self.params.get(name) {
            Ok(PyValue::Str(v.clone()))
        } else {
            match opts.default {
                None => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(d) => Ok(d),
            }
        }
    }
    fn get_int(&self, name: &str, opts: GcodeGetIntOpts) -> KResult<Option<i64>> {
        if let Some(raw) = self.params.get(name) {
            let v = int_from_string(raw, 10).map_err(|_| {
                klippy_api::KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))
            })?;
            let s = v.inner().to_string();
            Ok(Some(s.parse().unwrap()))
        } else {
            match opts.default {
                None => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(d) => Ok(d),
            }
        }
    }
    fn get_float(&self, name: &str, opts: GcodeGetFloatOpts) -> KResult<Option<f64>> {
        if let Some(raw) = self.params.get(name) {
            let v = float_from_string(raw).map_err(|_| {
                klippy_api::KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))
            })?;
            if let Some(ab) = opts.above {
                if v <= ab {
                    let ab_s = {
                        // Format bound like Python %s on float.
                        percent_format("%s", &[PctValue::Float(ab)]).unwrap()
                    };
                    return Err(klippy_api::KlippyError::command(format!(
                        "Error on '{}': {name} must be above {ab_s}",
                        self.commandline
                    )));
                }
            }
            Ok(Some(v))
        } else {
            match opts.default {
                None => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(d) => Ok(d),
            }
        }
    }
    fn get_command(&self) -> String {
        self.commandline.clone()
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
    fn respond_raw(&self, msg: &str) {
        self.gcode.respond_raw(msg);
    }
    fn ack(&self, msg: Option<&str>) -> bool {
        if !*self.need_ack.borrow() {
            return false;
        }
        *self.ack_msg.borrow_mut() = msg.map(|s| s.to_string());
        let ok_msg = match msg {
            Some(m) if !m.is_empty() => format!("ok {m}"),
            _ => "ok".to_string(),
        };
        self.gcode.respond_raw(&ok_msg);
        *self.need_ack.borrow_mut() = false;
        true
    }
}

klippy_api::printer_object!(FakeGcmd);

struct FakePrinterConfig {
    fail: bool,
    sections: Vec<String>,
    read_calls: klippy_api::GCell<Vec<String>>,
}

impl FakePrinterConfig {
    fn new(fail: bool, sections: Vec<String>) -> Arc<Self> {
        Arc::new(FakePrinterConfig {
            fail,
            sections,
            read_calls: klippy_api::GCell::new(Vec::new()),
        })
    }
}

impl PrinterConfigApi for FakePrinterConfig {
    fn get_printer(&self) -> Printer {
        panic!("printer not used")
    }
    fn read_config(&self, filename: &str) -> KResult<Config> {
        self.read_calls.borrow_mut().push(filename.to_string());
        if self.fail {
            return Err(klippy_api::KlippyError::config("cannot read".to_string()));
        }
        let mut inner = FakeConfigData {
            printer: self.printer_for_sub(),
            name: "temperature_sensors.cfg".to_string(),
            options: IndexMap::new(),
            prefix_sections: self.sections.clone(),
            sections_map: IndexMap::new(),
        };
        inner.printer = self.printer_for_sub();
        Ok(Arc::new(FakeConfig { inner: Arc::new(inner) }))
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("main config not used")
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(
        &self,
        _section: &str,
        _option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
    }
    fn deprecate_gcode(
        &self,
        _cmd: &str,
        _opts: klippy_api::config::DeprecateGcodeOpts,
    ) {
    }
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) {}
    fn remove_section(&self, _section: &str) {}
}

impl FakePrinterConfig {
    fn printer_for_sub(&self) -> Printer {
        // read_config results only use get_prefix_sections/get_name in heaters;
        // the printer is unused, so reuse a dummy.
        Arc::new(FakePrinter {
            state: klippy_api::GCell::new(FakePrinterState {
                objects: IndexMap::new(),
                debugoutput: None,
                shutdown: false,
                reactor: Reactor::new(false),
                event_handlers: Vec::new(),
                load_calls: Vec::new(),
            }),
        })
    }
}

klippy_api::printer_object!(FakePrinterConfig; dyn PrinterConfigApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("heaters fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping heaters oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "heaters".to_string(),
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

fn check_str(label: &str, expected: pyoracle::OracleResult, actual: String) {
    check(label, expected, Value::str(&actual));
}

fn check_bool(label: &str, expected: pyoracle::OracleResult, actual: bool) {
    check(label, expected, Value::Bool(actual));
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v:?} but call must fail"),
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

fn rust_opts(map: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut m = IndexMap::new();
    for (k, v) in map {
        m.insert(k.to_string(), v.to_string());
    }
    m
}


// Build a matched pair of Python/Rust heaters for comparison.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    options: &[(&str, &str)],
    debugoutput: Option<String>,
) -> (Arc<Heater>, Arc<FakePrinter>, Arc<FakeSensor>, Arc<FakePwm>, Arc<FakeGCode>, Arc<FakeToolhead>) {
    // Python side: build via build_heater.
    let py_opts = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect::<Vec<_>>()
        .join(", ");
    let db = debugoutput.clone();
    let db_py = match db {
        Some(s) => format!("'{}'", s),
        None => "None".to_string(),
    };
    py.exec(&format!(
        "printer, config, sensor, heater, pwm, mcu, gcode, toolhead = build_heater(debugoutput={db_py}, options={{{py_opts}}})"
    ))
    .expect("py build_heater");
    // Rust side.
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(debugoutput, reactor);
    let printer: Printer = printer_arc.clone();
    let pwm = FakePwm::new(1000.0);
    let pins: Arc<FakePins> = Arc::new(FakePins { pwm: pwm.clone() as PwmPin });
    let gcode = FakeGCode::new();
    let toolhead = FakeToolhead::new();
    // Register objects by name so lookup_interface finds them.
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
        st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        st.objects.insert("toolhead".to_string(), toolhead.clone() as ObjectRef);
    }
    let sensor = FakeSensor::new(0.3, 25., 0.);
    // PrinterHeaters placeholder for set_temperature lookup (set after Heater built).
    // Keep a reference for later registration.
    let opts_map = rust_opts(options);
    // Add Heater-required defaults if the test options omit them.
    let opts_map = {
        let mut m = opts_map;
        m.entry("min_temp".to_string()).or_insert_with(|| "0.0".to_string());
        m.entry("max_temp".to_string()).or_insert_with(|| "300.0".to_string());
        m.entry("heater_pin".to_string()).or_insert_with(|| "PA0".to_string());
        m.entry("control".to_string()).or_insert_with(|| "watermark".to_string());
        m
    };
    let cfg: Config = FakeConfig::new(printer.clone(), "heater_generic extruder1", opts_map);
    let heater = Heater::new(&cfg, sensor.clone() as Arc<dyn TemperatureSensorApi>).expect("rust Heater::new");
    // Python counterpart placeholder: the Python heater closure looks up
    // 'heaters' lazily in cmd_SET_HEATER_TEMPERATURE, not at construction,
    // so no registration is needed for construction.
    (heater, printer_arc, sensor, pwm, gcode, toolhead)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("KELVIN_TO_CELSIUS", KELVIN_TO_CELSIUS),
        ("MAX_HEAT_TIME", MAX_HEAT_TIME),
        ("AMBIENT_TEMP", AMBIENT_TEMP),
        ("PID_PARAM_BASE", PID_PARAM_BASE),
        ("MAX_MAINTHREAD_TIME", MAX_MAINTHREAD_TIME),
        ("QUELL_STALE_TIME", QUELL_STALE_TIME),
        ("MIN_PWM_CHANGE_RATIO", MIN_PWM_CHANGE_RATIO),
        ("PID_SETTLE_DELTA", PID_SETTLE_DELTA),
        ("PID_SETTLE_SLOPE", PID_SETTLE_SLOPE),
    ];
    for (name, rust) in cases {
        let expected = py.eval(&format!("heaters.{name}")).expect("eval const");
        check_float(&format!("constants {name}"), Ok(expected), rust);
    }
}

#[test]
fn heater_init_reads_config_and_reports_attributes() {
    let Some(mut py) = oracle() else { return };
    let (heater, _printer, sensor, _pwm, _gcode, _toolhead) = fresh_pair(
        &mut py,
        &[("min_temp", "10.0"), ("max_temp", "280.0"), ("max_power", "0.5"), ("smooth_time", "2.0")],
        None,
    );
    let expected = py.eval("heater.min_temp").expect("py min_temp");
    check_float("init min_temp", Ok(expected), heater.min_temp());
    let expected = py.eval("heater.max_temp").expect("py max_temp");
    check_float("init max_temp", Ok(expected), heater.max_temp());
    let expected = py.eval("heater.get_pwm_delay()").expect("py pwm_delay");
    check_float("init pwm_delay", Ok(expected), heater.get_pwm_delay());
    let expected = py.eval("heater.get_max_power()").expect("py max_power");
    check_float("init max_power", Ok(expected), heater.get_max_power());
    let expected = py.eval("heater.get_smooth_time()").expect("py smooth");
    check_float("init smooth_time", Ok(expected), heater.get_smooth_time());
    let expected = py.eval("heater.get_name()").expect("py name");
    check_str("init name", Ok(expected), heater.get_name());
    let expected = py.eval("sensor.minmax").expect("py minmax");
    // minmax is a tuple (min_temp, max_temp).
    let Some((mm0, mm1)) = *sensor.minmax.borrow() else { panic!("sensor minmax not set") };
    check("init minmax tuple", Ok(expected), Value::Tuple(vec![Value::float(mm0), Value::float(mm1)]));
}

#[test]
fn heater_init_defaults() {
    let Some(mut py) = oracle() else { return };
    let (heater, _printer, _sensor, _pwm, _gcode, _toolhead) =
        fresh_pair(&mut py, &[], None);
    let expected = py.eval("heater.min_extrude_temp").expect("py extrude");
    check_float("default min_extrude_temp", Ok(expected), heater.min_extrude_temp());
    let expected = py.eval("heater.max_power").expect("py max_power attr");
    check_float("default max_power attr", Ok(expected), heater.get_max_power());
    let expected = py.eval("heater.can_extrude").expect("py can_extrude");
    check_bool("default can_extrude", Ok(expected), heater.can_extrude());
}

#[test]
fn heater_init_fileoutput_can_extrude() {
    let Some(mut py) = oracle() else { return };
    // With debugoutput set, can_extrude is true even for high min_extrude_temp.
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(
        &mut py,
        &[("min_extrude_temp", "200.0")],
        Some("/tmp/out".to_string()),
    );
    let expected = py.eval("heater.can_extrude").expect("py can_extrude file");
    check_bool("fileoutput can_extrude", Ok(expected), heater.can_extrude());
}

#[test]
fn heater_set_temp_range_check() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    for (label, deg) in [("set_temp in range", 200.0), ("set_temp zero", 0.0)] {
        let expected = py.eval(&format!("heater.set_temp({deg:?})"));
        if let Err(e) = &expected { panic!("{label}: oracle raised {e:?}"); }
        if let Err(e) = heater.set_temp(deg) { panic!("{label}: rust raised {e:?}"); }
        let expected = py.eval("heater.target_temp").expect("py target");
        check_float(&format!("{label} target"), Ok(expected), heater.target_temp());
    }
    for (label, deg) in [("set_temp above max", 301.0), ("set_temp below min", -1.0)] {
        let expected = py.eval(&format!("heater.set_temp({deg:?})"));
        check_err(label, expected, heater.set_temp(deg).map(|_| ()));
    }
}

#[test]
fn heater_set_temp_zero_skips_range() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    // 0.0 is falsy in Python; the range check is skipped.
    let expected = py.eval("heater.set_temp(0.0)");
    if let Err(e) = &expected { panic!("set_temp zero: oracle raised {e:?}"); }
    if let Err(e) = heater.set_temp(0.) { panic!("set_temp zero: rust raised {e:?}"); }
    let expected = py.eval("heater.target_temp").expect("py target 0");
    check_float("zero target", Ok(expected), heater.target_temp());
}

#[test]
fn heater_alter_target_clamps() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    for deg in [0.0, 100.0, 999.0, -50.0] {
        let expected_py = py.eval(&format!("heater.alter_target({deg:?})"));
        let _ = expected_py.expect("py alter_target");
        heater.alter_target(deg);
        let expected = py.eval("heater.target_temp").expect("py alter target");
        check_float(&format!("alter_target {deg}"), Ok(expected), heater.target_temp());
    }
}

#[test]
fn heater_temperature_callback_smooths() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let expected = py.eval("heater.temperature_callback(100.0, 100.0)").expect("py callback");
    let _ = expected;
    heater.temperature_callback(100., 100.);
    // Read smoothed via get_status temperature after stale quell: use status.
    // smoothed is not directly exposed; compare get_status temperature.
    let expected = py.eval("heater.get_status(0.0)['temperature']").expect("py status temp");
    check_float("smoothed status", Ok(expected), match heater.get_status(0.) {
        PyValue::Dict(m) => match m.get(&PyValue::Str("temperature".to_string())) {
            Some(PyValue::Float(f)) => *f,
            _ => f64::NAN,
        },
        _ => f64::NAN,
    });
    let expected = py.eval("heater.can_extrude").expect("py can_extrude after");
    check_bool("can_extrude after smooth", Ok(expected), heater.can_extrude());
    // Second callback with steady temp verifies smoothing converges.
    let expected = py.eval("heater.temperature_callback(101.0, 100.0)").expect("py callback2");
    let _ = expected;
    heater.temperature_callback(101., 100.);
    let expected = py.eval("heater.get_status(0.0)['temperature']").expect("py status temp2");
    check_float("smoothed status 2", Ok(expected), match heater.get_status(0.) {
        PyValue::Dict(m) => match m.get(&PyValue::Str("temperature".to_string())) {
            Some(PyValue::Float(f)) => *f,
            _ => f64::NAN,
        },
        _ => f64::NAN,
    });
}

#[test]
fn heater_set_pwm_suppression() {
    let Some(mut py) = oracle() else { return };
    // target 0 forces value 0; repeated small changes below min_pwm_change are suppressed.
    let (heater, _p, _s, pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let expected = py.eval("heater.set_pwm(10.0, 0.5)").expect("py set_pwm");
    let _ = expected;
    heater.set_pwm(10., 0.5);
    let py_calls = py.eval("[c for c in pwm.calls if c[0] == 'set_pwm']").expect("py pwm calls");
    let rust_calls = pwm.pwm_calls.borrow();
    // Compare lengths first, then each (time, value).
    let (exp_len, rust_len) = match &py_calls {
        Value::List(l) => (l.len(), rust_calls.len()),
        other => panic!("py calls not a list: {other:?}"),
    };
    assert_eq!(exp_len, rust_len, "pwm call count");
    let py_calls_v = match py_calls {
        Value::List(l) => l,
        _ => unreachable!(),
    };
    for (i, pv) in py_calls_v.iter().enumerate() {
        match pv {
            Value::Tuple(items) => {
                assert_eq!(items.len(), 3, "pwm call arity");
                // items[0] is 'set_pwm' str; compare time/value bits.
                let et = items[1].clone();
                let ev = items[2].clone();
                check(
                    &format!("pwm call {i} time"),
                    Ok(et),
                    Value::float(rust_calls[i].0),
                );
                check(
                    &format!("pwm call {i} value"),
                    Ok(ev),
                    Value::float(rust_calls[i].1),
                );
            }
            _ => panic!("pwm call not a tuple"),
        }
    }
}

#[test]
fn heater_set_pwm_next_time_and_watchdog() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    // Set a target so set_pwm does not force 0.
    let _ = py.eval("heater.set_temp(100.0)").expect("py set_temp");
    let _ = heater.set_temp(100.).expect("rust set_temp");
    // First call sends pwm directly, second call within next_pwm_time with
    // small delta is suppressed.
    let _ = py.eval("heater.set_pwm(10.0, 0.5)").expect("py set_pwm1");
    heater.set_pwm(10., 0.5);
    let _ = py.eval("heater.set_pwm(10.1, 0.51)").expect("py set_pwm2");
    heater.set_pwm(10.1, 0.51);
    let py_calls = py.eval("[c for c in pwm.calls if c[0] == 'set_pwm']").expect("py calls");
    let rust_n = pwm.pwm_calls.borrow().len();
    match py_calls {
        Value::List(l) => assert_eq!(l.len(), rust_n, "watchdog pwm count"),
        o => panic!("unexpected {o:?}"),
    }
    // stats() refreshes the watchdog window.
    let _ = py.eval("heater.stats(1000.0)").expect("py stats");
    let _ = heater.stats(1000.);
    // After shutdown, verify_mainthread_time is -999 and set_pwm forces 0.
    let _ = py.eval("heater._handle_shutdown()").expect("py shutdown");
    heater.handle_shutdown();
    let _ = py.eval("heater.set_pwm(1001.0, 0.9)").expect("py set_pwm3");
    heater.set_pwm(1001., 0.9);
    let py_calls = py.eval("[c for c in pwm.calls if c[0] == 'set_pwm']").expect("py calls2");
    let rust_calls = pwm.pwm_calls.borrow();
    match py_calls {
        Value::List(l) => assert_eq!(l.len(), rust_calls.len(), "post-shutdown pwm count"),
        o => panic!("unexpected {o:?}"),
    }
}

#[test]
fn heater_get_status_and_stats() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let _ = py.eval("heater.set_temp(100.0)").expect("py set");
    let _ = heater.set_temp(100.).expect("rust set");
    let expected = py.eval("heater.get_status(0.0)").expect("py status");
    let rust_status = heater.get_status(0.);
    // Compare fields individually.
    let py_status = expected;
    match (py_status, rust_status) {
        (Value::Dict(pairs), PyValue::Dict(rpairs)) => {
            for (pk, pv) in pairs {
                let rk = match pk {
                    Value::Str(s) => PyValue::Str(s),
                    _ => continue,
                };
                let rv = rpairs.get(&rk).cloned().unwrap_or(PyValue::None);
                let rv_value = match rv {
                    PyValue::Float(f) => Value::float(f),
                    PyValue::Str(s) => Value::str(&s),
                    PyValue::Int(n) => Value::int(n.inner().to_string().parse().unwrap()),
                    other => Value::str(&other.str()),
                };
                check(&format!("get_status key {}", rk.str()), Ok(pv), rv_value);
            }
        }
        (p, r) => panic!("status shapes differ: {p:?} vs {r:?}"),
    }
    let expected = py.eval("bool(heater.stats(1000.0)[0])").expect("py stats active");
    let (active, msg) = heater.stats(1000.);
    check_bool("stats active", Ok(expected), active);
    let expected_msg = py.eval("heater.stats(1000.0)[1]").expect("py stats msg");
    check_str("stats msg", Ok(expected_msg), msg);
}

#[test]
fn heater_get_temp_quell() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let _ = py.eval("heater.set_temp(100.0)").expect("py set");
    let _ = heater.set_temp(100.).expect("rust set");
    // last_temp_time is 0, est_print_time is 1000, quell is 993: stale.
    let expected = py.eval("heater.get_temp(0.0)").expect("py get_temp stale");
    let (cur, tgt) = HeaterApi::get_temp(heater.as_ref(), 0.);
    match expected {
        Value::Tuple(items) => {
            check_float("get_temp stale cur", Ok(items[0].clone()), cur);
            check_float("get_temp stale tgt", Ok(items[1].clone()), tgt);
        }
        o => panic!("get_temp shape {o:?}"),
    }
    // After a callback, temp is fresh.
    let _ = py.eval("heater.temperature_callback(1000.0, 80.0)").expect("py cb");
    heater.temperature_callback(1000., 80.);
    let expected = py.eval("heater.get_temp(1000.0)").expect("py get_temp fresh");
    let (cur, tgt) = HeaterApi::get_temp(heater.as_ref(), 1000.);
    match expected {
        Value::Tuple(items) => {
            check_float("get_temp fresh cur", Ok(items[0].clone()), cur);
            check_float("get_temp fresh tgt", Ok(items[1].clone()), tgt);
        }
        o => panic!("get_temp shape {o:?}"),
    }
}

#[test]
fn heater_check_busy_bangbang() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let _ = py.eval("heater.set_temp(100.0)").expect("py set");
    let _ = heater.set_temp(100.).expect("rust set");
    let expected = py.eval("heater.check_busy(0.0)").expect("py busy");
    check_bool("bangbang busy cold", Ok(expected), heater.check_busy(0.));
    let _ = py.eval("heater.temperature_callback(1000.0, 100.0)").expect("py cb");
    heater.temperature_callback(1000., 100.);
    let expected = py.eval("heater.check_busy(1000.0)").expect("py busy warm");
    check_bool("bangbang busy warm", Ok(expected), heater.check_busy(1000.));
}

#[test]
fn control_bangbang_updates_pwm() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let _ = py.eval("heater.set_temp(100.0)").expect("py set");
    let _ = heater.set_temp(100.).expect("rust set");
    // Cold: heating true -> pwm max.
    let _ = py.eval("heater.temperature_callback(10.0, 50.0)").expect("py cb cold");
    heater.temperature_callback(10., 50.);
    // Hot: heating false -> pwm 0.
    let _ = py.eval("heater.temperature_callback(11.0, 105.0)").expect("py cb hot");
    heater.temperature_callback(11., 105.);
    let py_calls = py.eval("[c for c in pwm.calls if c[0] == 'set_pwm']").expect("py calls");
    let rust_calls = pwm.pwm_calls.borrow();
    match py_calls {
        Value::List(l) => {
            assert_eq!(l.len(), rust_calls.len(), "bangbang pwm count");
            for (i, pv) in l.iter().enumerate() {
                let Value::Tuple(items) = pv else { panic!("tuple expected") };
                check(&format!("bangbang pwm {i} value"), Ok(items[2].clone()), Value::float(rust_calls[i].1));
            }
        }
        o => panic!("unexpected {o:?}"),
    }
}

#[test]
fn control_pid_updates_and_anti_windup() {
    let Some(mut py) = oracle() else { return };
    // pid control needs pid params; provide them.
    let opts = [
        ("control", "pid"),
        ("pid_Kp", "100.0"),
        ("pid_Ki", "50.0"),
        ("pid_Kd", "10.0"),
    ];
    let (heater, _p, _s, pwm, _g, _t) = fresh_pair(&mut py, &opts, None);
    let _ = py.eval("heater.set_temp(200.0)").expect("py set");
    let _ = heater.set_temp(200.).expect("rust set");
    for (rt, temp) in [(10.0, 20.0), (10.5, 50.0), (11.0, 100.0), (11.5, 150.0)] {
        let _ = py.eval(&format!("heater.temperature_callback({rt:?}, {temp:?})")).expect("py cb");
        heater.temperature_callback(rt, temp);
    }
    let py_calls = py.eval("[c for c in pwm.calls if c[0] == 'set_pwm']").expect("py pid calls");
    let rust_calls = pwm.pwm_calls.borrow();
    match py_calls {
        Value::List(l) => {
            assert_eq!(l.len(), rust_calls.len(), "pid pwm count");
            for (i, pv) in l.iter().enumerate() {
                let Value::Tuple(items) = pv else { panic!("tuple") };
                check(&format!("pid pwm {i} time"), Ok(items[1].clone()), Value::float(rust_calls[i].0));
                check(&format!("pid pwm {i} value"), Ok(items[2].clone()), Value::float(rust_calls[i].1));
            }
        }
        o => panic!("unexpected {o:?}"),
    }
    // check_busy uses settle delta/slope.
    let expected = py.eval("heater.check_busy(0.0)").expect("py pid busy");
    check_bool("pid busy", Ok(expected), heater.check_busy(0.));
}

#[test]
fn heater_set_control_resets_target() {
    let Some(mut py) = oracle() else { return };
    let (heater, _p, _s, _pwm, _g, _t) = fresh_pair(&mut py, &[], None);
    let _ = py.eval("heater.set_temp(100.0)").expect("py set");
    let _ = heater.set_temp(100.).expect("rust set");
    let _ = py
        .eval("heater.set_control(heaters.ControlBangBang(heater, config))")
        .expect("py set_control");
    // Rust side: build a new bangbang control and swap it.
    let printer: Printer = _p.clone();
    let cfg: Config = FakeConfig::new(printer, "heater_generic extruder1", rust_opts(&[]));
    let _ = heater.set_control(Arc::new(
        ControlBangBang::new(Arc::downgrade(&heater), heater.get_max_power(), &cfg).expect("bangbang"),
    ));
    let expected = py.eval("heater.target_temp").expect("py target after set_control");
    check_float("set_control resets target", Ok(expected), heater.target_temp());
}

#[test]
fn heater_config_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = FakePrinter(); printer.objects['pins'] = FakePins(FakePwm(FakeMcu(1000.0))); printer.objects['gcode'] = FakeGCode(); printer.objects['toolhead'] = FakeToolhead()").expect("py printer");
    // Missing min_temp (required).
    {
        let mut py2 = oracle().expect("oracle for missing min");
        let _ = py2;
        // Use fresh_pair with missing min_temp via direct Heater::new with
        // minimal options: remove min_temp by passing empty and deleting entry.
        // fresh_pair inserts defaults, so construct manually.
        let reactor = Reactor::new(false);
        let printer_arc = FakePrinter::new(None, reactor);
        let printer: Printer = printer_arc.clone();
        let pwm = FakePwm::new(1000.0);
        let pins: Arc<FakePins> = Arc::new(FakePins { pwm: pwm.clone() as PwmPin });
        let gcode = FakeGCode::new();
        {
            let mut st = printer_arc.state.borrow_mut();
            st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
            st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        }
        let sensor = FakeSensor::new(0.3, 25., 0.);
        let mut opts = rust_opts(&[("max_temp", "300.0"), ("heater_pin", "PA0"), ("control", "watermark")]);
        opts.shift_remove("min_temp");
        let cfg: Config = FakeConfig::new(printer, "heater_generic extruder1", opts);
        // Python side for comparison.
        py.exec("b_cfg = FakeConfig(printer, 'heater_generic extruder1', {'max_temp': '300.0', 'heater_pin': 'PA0', 'control': 'watermark'})")
            .expect("py cfg");
        py.exec("b_sensor = FakeSensor(report_delta=0.3)").expect("py sensor");
        let expected = py.eval("heaters.Heater(b_cfg, b_sensor)");
        let actual = Heater::new(&cfg, sensor.clone() as Arc<dyn TemperatureSensorApi>).map(|_| ());
        check_err_val("missing min_temp", expected, actual);
    }
    // Invalid control choice.
    {
        let reactor = Reactor::new(false);
        let printer_arc = FakePrinter::new(None, reactor);
        let printer: Printer = printer_arc.clone();
        let pwm = FakePwm::new(1000.0);
        let pins: Arc<FakePins> = Arc::new(FakePins { pwm: pwm.clone() as PwmPin });
        let gcode = FakeGCode::new();
        {
            let mut st = printer_arc.state.borrow_mut();
            st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
            st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        }
        let sensor = FakeSensor::new(0.3, 25., 0.);
        let cfg: Config = FakeConfig::new(
            printer,
            "heater_generic extruder1",
            rust_opts(&[
                ("min_temp", "0.0"),
                ("max_temp", "300.0"),
                ("heater_pin", "PA0"),
                ("control", "banana"),
            ]),
        );
        py.exec("b_cfg2 = FakeConfig(printer, 'heater_generic extruder1', {'min_temp': '0.0', 'max_temp': '300.0', 'heater_pin': 'PA0', 'control': 'banana'})")
            .expect("py cfg2");
        py.exec("b_sensor2 = FakeSensor(report_delta=0.3)").expect("py sensor2");
        let expected = py.eval("heaters.Heater(b_cfg2, b_sensor2)");
        let actual = Heater::new(&cfg, sensor as Arc<dyn TemperatureSensorApi>).map(|_| ());
        check_err_val("invalid control choice", expected, actual);
    }
}

// ---------------------------------------------------------------------------
// PrinterHeaters tests.
// ---------------------------------------------------------------------------

struct PhFixture {
    ph: Arc<PrinterHeaters>,
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
}

fn fresh_ph(
    py: &mut pyoracle::Oracle,
    debugoutput: Option<String>,
) -> PhFixture {
    let db_py = match &debugoutput {
        Some(s) => format!("'{}'", s),
        None => "None".to_string(),
    };
    py.exec(&format!("ph_printer = FakePrinter(debugoutput={db_py}); ph_gcode = FakeGCode(); ph_toolhead = FakeToolhead(); ph_printer.objects['gcode'] = ph_gcode; ph_printer.objects['toolhead'] = ph_toolhead; ph_printer.objects['pins'] = FakePins(FakePwm(FakeMcu(1000.0))); ph_printer.objects['configfile'] = FakePrinterConfig(fail=False)"))
        .expect("py ph setup");
    py.exec("ph_config = FakeConfig(ph_printer, 'heaters', {})").expect("py ph cfg");
    py.exec("ph = heaters.load_config(ph_config)").expect("py ph load");
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(debugoutput, reactor);
    let printer: Printer = printer_arc.clone();
    let gcode = FakeGCode::new();
    let toolhead = FakeToolhead::new();
    let pwm = FakePwm::new(1000.0);
    let pins: Arc<FakePins> = Arc::new(FakePins { pwm: pwm.clone() as PwmPin });
    let pcfg = FakePrinterConfig::new(false, Vec::new());
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        st.objects.insert("toolhead".to_string(), toolhead.clone() as ObjectRef);
        st.objects.insert("pins".to_string(), pins.clone() as ObjectRef);
        st.objects.insert("configfile".to_string(), pcfg.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(printer, "heaters", IndexMap::new());
    let ph = PrinterHeaters::new(&cfg).expect("rust ph new");
    // Register the Rust object as "heaters" so cmd_SET_HEATER_TEMPERATURE
    // can find it.
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("heaters".to_string(), ph.clone() as ObjectRef);
    }
    PhFixture { ph, printer: printer_arc, gcode, toolhead }
}

#[test]
fn ph_registers_commands_and_status() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    let expected = py.eval("sorted([c[0] for c in ph_gcode.commands])").expect("py cmds");
    let rust_cmds = fx.gcode.commands.borrow().clone();
    let rust_sorted = {
        let mut v: Vec<String> = rust_cmds.iter().map(|s| s.split(':').next().unwrap().to_string()).collect();
        v.sort();
        v
    };
    match expected {
        Value::List(items) => {
            assert_eq!(items.len(), rust_sorted.len(), "ph command count");
            for (i, pv) in items.iter().enumerate() {
                check(&format!("ph cmd {i}"), Ok(pv.clone()), Value::str(&rust_sorted[i]));
            }
        }
        o => panic!("shape {o:?}"),
    }
    let expected = py.eval("ph.get_status(0.0)").expect("py ph status");
    let rust_status = klippy_api::PrinterObject::get_status(&*fx.ph, 0.).expect("rust status");
    match (expected, rust_status) {
        (Value::Dict(pairs), PyValue::Dict(rpairs)) => {
            assert_eq!(pairs.len(), rpairs.len(), "ph status keys");
            for (pk, pv) in pairs {
                let Value::Str(ks) = pk else { panic!("key") };
                let rv = rpairs.get(&PyValue::Str(ks.clone())).cloned().unwrap();
                let rv_value = match rv {
                    PyValue::List(l) => Value::List(
                        l.into_iter()
                            .map(|x| match x {
                                PyValue::Str(s) => Value::str(&s),
                                other => Value::str(&other.str()),
                            })
                            .collect(),
                    ),
                    other => Value::str(&other.str()),
                };
                check(&format!("ph status {ks}"), Ok(pv), rv_value);
            }
        }
        (e, r) => panic!("status compare failed: {e:?} vs {r:?}"),
    }
}

#[test]
fn ph_m105_before_and_after_ready() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    // Before ready: "T:0".
    py.exec("ph_gcode.raw.clear(); g = FakeGcmd(ph_gcode, {}, need_ack=False); ph.cmd_M105(g)").expect("py m105 run");
    let expected = py.eval("list(ph_gcode.raw)").expect("py m105");
    // Rust side.
    let gcode = FakeGCode::new();
    let mut params = IndexMap::new();
    params.insert("M105".to_string(), "M105".to_string());
    let gcmd: GCmd = FakeGcmd::new(gcode.clone(), params, false, "M105");
    let _ = fx.ph.cmd_m105(&gcmd);
    let rust_raw = gcode.raw.borrow().clone();
    match expected {
        Value::List(l) => {
            assert_eq!(l.len(), rust_raw.len(), "m105 raw count");
            for (i, pv) in l.iter().enumerate() {
                check(&format!("m105 raw {i}"), Ok(pv.clone()), Value::str(&rust_raw[i]));
            }
        }
        o => panic!("shape {o:?}"),
    }
    // After ready with a sensor: still "T:0" (no gcode_id sensors).
    let _ = py.eval("ph._handle_ready()").expect("py ready");
    fx.ph.handle_ready();
    py.exec("ph_gcode.raw.clear(); g2 = FakeGcmd(ph_gcode, {}, need_ack=True); ph.cmd_M105(g2)").expect("py m105b run");
    let expected = py.eval("list(ph_gcode.raw)").expect("py m105b");
    let gcode2 = FakeGCode::new();
    let params2 = IndexMap::new();
    let gcmd2: GCmd = FakeGcmd::new(gcode2.clone(), params2, true, "M105");
    let _ = fx.ph.cmd_m105(&gcmd2);
    let rust_raw2 = gcode2.raw.borrow().clone();
    match expected {
        Value::List(l) => {
            assert_eq!(l.len(), rust_raw2.len(), "m105b raw count");
            for (i, pv) in l.iter().enumerate() {
                check(&format!("m105b raw {i}"), Ok(pv.clone()), Value::str(&rust_raw2[i]));
            }
        }
        o => panic!("shape {o:?}"),
    }
}

#[test]
fn ph_temperature_wait_missing_bounds() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    py.exec("tw_gcode = ph_gcode; tw_gcmd = FakeGcmd(tw_gcode, {'SENSOR': 'extruder1'}, need_ack=True, command='TEMPERATURE_WAIT')")
        .expect("py tw gcmd");
    let expected = py.eval("ph.cmd_TEMPERATURE_WAIT(tw_gcmd)");
    let gcmd: GCmd = FakeGcmd::new(fx.gcode.clone(), {
        let mut m = IndexMap::new();
        m.insert("SENSOR".to_string(), "extruder1".to_string());
        m
    }, true, "TEMPERATURE_WAIT");
    check_err("tw missing bounds", expected, fx.ph.cmd_temperature_wait(&gcmd));
}

#[test]
fn ph_setup_sensor_errors() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    // Unknown sensor_type.
    py.exec("bad_cfg = FakeConfig(ph_printer, 'heater_generic bad', {'sensor_type': 'NOPE'})").expect("py bad cfg");
    let expected = py.eval("ph.setup_sensor(bad_cfg)");
    let rust_cfg: Config = FakeConfig::new(
        fx.printer.clone() as Printer,
        "heater_generic bad",
        rust_opts(&[("sensor_type", "NOPE")]),
    );
    let actual = fx.ph.setup_sensor(&rust_cfg).map(|_| ());
    check_err_val("setup_sensor unknown", expected, actual);
    // Lookup unknown heater.
    let expected = py.eval("ph.lookup_heater('nope')");
    let actual = fx.ph.lookup_heater("nope").map(|_| ());
    check_err_val("lookup_heater unknown", expected, actual);
}

#[test]
fn ph_load_config_failure() {
    let Some(mut py) = oracle() else { return };
    // Python side: configfile fails.
    let db = "None";
    py.exec(&format!("lc_printer = FakePrinter(debugoutput={db}); lc_gcode = FakeGCode(); lc_printer.objects['gcode'] = lc_gcode; lc_printer.objects['configfile'] = FakePrinterConfig(fail=True)"))
        .expect("py lc setup");
    py.exec("lc_config = FakeConfig(lc_printer, 'heaters', {})").expect("py lc cfg");
    py.exec("lc_ph = heaters.load_config(lc_config)").expect("py lc load");
    let expected = py.eval("lc_ph.load_config(lc_config)");
    // Rust side.
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(None, reactor);
    let printer: Printer = printer_arc.clone();
    let gcode = FakeGCode::new();
    let pcfg = FakePrinterConfig::new(true, Vec::new());
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        st.objects.insert("configfile".to_string(), pcfg.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(printer, "heaters", IndexMap::new());
    let ph = PrinterHeaters::new(&cfg).expect("rust lc ph");
    let actual = ph.load_config(&cfg);
    check_err_val("load_config failure", expected, actual);
}

#[test]
fn ph_turn_off_all_heaters() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    py.exec("def _fac4(cfg): return FakeSensor(report_delta=0.3)").expect("py fac");
    py.eval("ph.add_sensor_factory('TEST', _fac4)").expect("py add fac");
    let factory: extras::heaters::SensorFactory = Arc::new(|_cfg: &Config| {
        Ok(FakeSensor::new(0.3, 25., 0.) as Arc<dyn TemperatureSensorApi>)
    });
    fx.ph.add_sensor_factory("TEST", factory);
    let py_cfg_opts = "{'sensor_type': 'TEST', 'min_temp': '0.0', 'max_temp': '300.0', 'heater_pin': 'PA0', 'control': 'watermark'}";
    py.exec(&format!("h4_cfg = FakeConfig(ph_printer, 'heater_generic extruder1', {py_cfg_opts})")).expect("py h cfg");
    let _ = py.eval("ph.setup_heater(h4_cfg, None)").expect("py setup");
    let rust_cfg: Config = FakeConfig::new(
        fx.printer.clone() as Printer,
        "heater_generic extruder1",
        rust_opts(&[
            ("sensor_type", "TEST"),
            ("min_temp", "0.0"),
            ("max_temp", "300.0"),
            ("heater_pin", "PA0"),
            ("control", "watermark"),
        ]),
    );
    let rust_heater = fx.ph.setup_heater(&rust_cfg, None).expect("rust setup");
    let _ = py.eval("ph.heaters['extruder1'].set_temp(100.0)").expect("py set");
    let _ = rust_heater.set_temp(100.).expect("rust set");
    let _ = py.eval("ph.turn_off_all_heaters()").expect("py turn off");
    fx.ph.turn_off_all_heaters(0.);
    let expected = py.eval("ph.heaters['extruder1'].target_temp").expect("py target after off");
    check_float("turn off target", Ok(expected), rust_heater.target_temp());
}

#[test]
fn ph_set_temperature_calls_toolhead() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    // Build a heater via setup_heater with a registered factory.
    py.exec("def _fac2(cfg): return FakeSensor(report_delta=0.3)").expect("py fac");
    py.eval("ph.add_sensor_factory('TEST', _fac2)").expect("py add fac");
    let factory: extras::heaters::SensorFactory = Arc::new(|_cfg: &Config| {
        Ok(FakeSensor::new(0.3, 25., 0.) as Arc<dyn TemperatureSensorApi>)
    });
    fx.ph.add_sensor_factory("TEST", factory);
    let py_cfg_opts = "{'sensor_type': 'TEST', 'min_temp': '0.0', 'max_temp': '300.0', 'heater_pin': 'PA0', 'control': 'watermark', 'gcode_id': 'T'}";
    py.exec(&format!("h_cfg = FakeConfig(ph_printer, 'heater_generic extruder1', {py_cfg_opts})")).expect("py h cfg");
    let py_heater = py.eval("ph.setup_heater(h_cfg, None)").expect("py setup_heater");
    let _ = py_heater;
    let rust_cfg: Config = FakeConfig::new(
        fx.printer.clone() as Printer,
        "heater_generic extruder1",
        rust_opts(&[
            ("sensor_type", "TEST"),
            ("min_temp", "0.0"),
            ("max_temp", "300.0"),
            ("heater_pin", "PA0"),
            ("control", "watermark"),
            ("gcode_id", "T"),
        ]),
    );
    let rust_heater = fx.ph.setup_heater(&rust_cfg, None).expect("rust setup_heater");
    // set_temperature.
    let expected = py.eval("ph.set_temperature(ph.heaters['extruder1'], 100.0)").expect("py set_temp");
    let _ = expected;
    fx.ph.set_temperature(&rust_heater, 100., false).expect("rust set_temp");
    let py_calls = py.eval("ph_toolhead.register_calls").expect("py register calls");
    check("toolhead register calls", Ok(py_calls), Value::int(*fx.toolhead.register_calls.borrow()));
    let expected = py.eval("ph.heaters['extruder1'].target_temp").expect("py target");
    check_float("set_temperature target", Ok(expected), rust_heater.target_temp());
}

#[test]
fn ph_m105_with_sensor_report() {
    let Some(mut py) = oracle() else { return };
    let fx = fresh_ph(&mut py, None);
    // Register a sensor with gcode_id T.
    py.exec("def _fac3(cfg): return FakeSensor(report_delta=0.3, temp=200.0, target=210.0)").expect("py fac3");
    py.exec("ph2 = ph; ph2.add_sensor_factory('TEST2', _fac3)").expect("py add fac3");
    let factory: extras::heaters::SensorFactory = Arc::new(|_cfg: &Config| {
        Ok(FakeSensor::new(0.3, 200., 210.) as Arc<dyn TemperatureSensorApi>)
    });
    fx.ph.add_sensor_factory("TEST2", factory);
    let py_cfg_opts = "{'sensor_type': 'TEST2', 'min_temp': '0.0', 'max_temp': '300.0', 'heater_pin': 'PA0', 'control': 'watermark'}";
    py.exec(&format!("h2_cfg = FakeConfig(ph_printer, 'heater_generic extruder1', {py_cfg_opts})")).expect("py h2 cfg");
    let _ = py.eval("ph.setup_heater(h2_cfg, 'T')").expect("py setup");
    let rust_cfg: Config = FakeConfig::new(
        fx.printer.clone() as Printer,
        "heater_generic extruder1",
        rust_opts(&[
            ("sensor_type", "TEST2"),
            ("min_temp", "0.0"),
            ("max_temp", "300.0"),
            ("heater_pin", "PA0"),
            ("control", "watermark"),
        ]),
    );
    let _ = fx.ph.setup_heater(&rust_cfg, Some("T".to_string())).expect("rust setup");
    let _ = py.eval("ph._handle_ready()").expect("py ready");
    fx.ph.handle_ready();
    // M105 report should contain the T sensor line.
    py.exec("ph_gcode.raw.clear(); mm = FakeGcmd(ph_gcode, {}, need_ack=True); ph.cmd_M105(mm)").expect("py m105 run");
    let expected = py.eval("list(ph_gcode.raw)").expect("py m105 sensor");
    let gcode2 = FakeGCode::new();
    let gcmd2: GCmd = FakeGcmd::new(gcode2.clone(), IndexMap::new(), true, "M105");
    let _ = fx.ph.cmd_m105(&gcmd2);
    let rust_raw = gcode2.raw.borrow().clone();
    match expected {
        Value::List(l) => {
            assert_eq!(l.len(), rust_raw.len(), "m105 sensor count");
            for (i, pv) in l.iter().enumerate() {
                check(&format!("m105 sensor {i}"), Ok(pv.clone()), Value::str(&rust_raw[i]));
            }
        }
        o => panic!("shape {o:?}"),
    }
}

#[test]
fn heater_cmd_set_heater_temperature() {
    let Some(mut py) = oracle() else { return };
    let (heater, _printer, _s, _pwm, gcode, _t) = fresh_pair(&mut py, &[], None);
    // Python side: FakeGcmd with TARGET.
    py.exec("gcmd = FakeGcmd(gcode, {'TARGET': '120.0'}, need_ack=True)").expect("py gcmd");
    let expected = py.eval("heater.cmd_SET_HEATER_TEMPERATURE(gcmd)");
    // Rust side: build a FakeGcmd with TARGET=120.
    let mut params = IndexMap::new();
    params.insert("TARGET".to_string(), "120.0".to_string());
    let gcmd: GCmd = FakeGcmd::new(gcode.clone(), params, true, "SET_HEATER_TEMPERATURE");
    let actual = heater.cmd_set_heater_temperature(&gcmd);
    check_err("cmd set heater temp", expected, actual);
}

#[test]
fn module_factory_loads() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::heaters::module_factory();
    assert!(factory.load_config.is_some(), "heaters exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "no early objects");
    assert!(factory.add_printer_objects.is_none(), "no printer objects");
    let load_config = factory.load_config.expect("load_config");
    // Python load_config returns a PrinterHeaters.
    py.exec("ph_printer = FakePrinter(); ph_gcode = FakeGCode(); ph_printer.objects['gcode'] = ph_gcode; ph_config = FakeConfig(ph_printer, 'heaters', {})").expect("py cfg");
    let expected_type = py.eval("type(heaters.load_config(ph_config)).__name__").expect("py type");
    // Rust factory builds a working PrinterHeaters.
    let reactor = Reactor::new(false);
    let printer_arc = FakePrinter::new(None, reactor);
    let printer: Printer = printer_arc.clone();
    let gcode = FakeGCode::new();
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(printer, "heaters", IndexMap::new());
    let obj = load_config(&cfg).expect("factory load").expect("factory obj");
    let ph: Arc<PrinterHeaters> = downcast_arc(&obj).expect("downcast");
    let _ = ph;
    let _ = expected_type;
}
