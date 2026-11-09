//! Parity tests for `extras::tsl1401cl_filament_width_sensor` (port of
//! `klippy/extras/tsl1401cl_filament_width_sensor.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `FilamentWidthSensor`
//! is built on both sides with minimal Python/Rust fakes for the printer,
//! config, pins/ADC, gcode and toolhead, fed identical ADC samples and
//! toolhead positions, and the resulting state, G-code responses, scripts
//! and config-error texts are compared bit-for-bit (floats by bits via the
//! recording value format).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

use extras::tsl1401cl_filament_width_sensor::FilamentWidthSensor;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts,
    GetOpts as GGetOpts, GCmd, RegisterCommandOpts, GCodeCommandHandler, OutputCallback,
};
use klippy_api::mcu::{
    AdcCallback, AdcPinApi, AdcSampleOpts, AddConfigCmdOpts, AsyncResponseWrapper,
    AsyncResponseWrapperApi, CommandQueryWrapper, CommandQueue, LookupQueryCommandOpts,
    Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi, LookupPinOpts, PinResolver,
    Chip, CommandWrapper, TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, LookaheadCallback, PrinterExtruder, ToolHeadApi,
};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Exception, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
from extras import tsl1401cl_filament_width_sensor as tsl

class Error(Exception):
    pass

sentinel = object()

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
        self.updates = []
    def register_timer(self, cb, waketime=NEVER):
        handle = ('timer', len(self.timers))
        self.timers.append((cb, waketime))
        return handle
    def update_timer(self, timer, waketime):
        self.updates.append((timer, waketime))

class FakeMcuAdc:
    def __init__(self):
        self.samples_calls = []
        self.callback = None
    def setup_adc_sample(self, report_time, sample_time=0., sample_count=1, batch_num=1, minval=0., maxval=1., range_check_count=0):
        self.samples_calls.append((report_time, sample_time, sample_count, batch_num, minval, maxval, range_check_count))
    def setup_adc_callback(self, cb):
        self.callback = cb
    def invoke(self, samples):
        self.callback(samples)

class FakePins:
    def __init__(self):
        self.setup_pin_calls = []
        self.adc = FakeMcuAdc()
    def setup_pin(self, pin_type, pin_desc):
        self.setup_pin_calls.append((pin_type, pin_desc))
        return self.adc

class FakeGCode:
    def __init__(self):
        self.registered = []
        self.scripts = []
        self.from_command_scripts = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered.append(cmd)
        assert func is not None
    def run_script(self, script):
        self.scripts.append(script)
    def run_script_from_command(self, script):
        self.from_command_scripts.append(script)

class FakeToolHead:
    def __init__(self, pos):
        self.pos = list(pos)
    def get_position(self):
        return list(self.pos)

class FakePrinter:
    def __init__(self, reactor, objs):
        self._reactor = reactor
        self._objs = dict(objs)
        self.handlers = {}
    def get_reactor(self):
        return self._reactor
    def lookup_object(self, name, default=sentinel):
        if name in self._objs:
            return self._objs[name]
        if default is sentinel:
            raise Error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        for cb in self.handlers.get(event, []):
            cb(*params)

class FakeConfig:
    def __init__(self, printer, values, section='filament_width_sensor'):
        self._printer = printer
        self.values = dict(values)
        self.section = section
        self.reads = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def get(self, option, default=sentinel, note_valid=True):
        self.reads.append('get:' + option)
        if option not in self.values:
            if default is not sentinel:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return self.values[option]
    def getfloat(self, option, default=sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if above is None:
            self.reads.append('getfloat:' + option + ':above=None')
        else:
            self.reads.append('getfloat:' + option + ':above=' + str(above))
        if option not in self.values:
            if default is not sentinel:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        v = float(self.values[option])
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v

class FakeGcmd:
    def __init__(self):
        self.infos = []
    def respond_info(self, msg, log=True):
        self.infos.append((msg, log))
    def respond_raw(self, msg):
        self.infos.append((msg, False))

def _build(values, pos):
    reactor = FakeReactor()
    pins = FakePins()
    gcode = FakeGCode()
    toolhead = FakeToolHead(pos)
    printer = FakePrinter(reactor, {'pins': pins, 'gcode': gcode, 'toolhead': toolhead})
    config = FakeConfig(printer, values)
    sensor = tsl.FilamentWidthSensor(config)
    return (reactor, pins, gcode, toolhead, printer, config, sensor)

def _state():
    return (sensor.nominal_filament_dia, sensor.measurement_delay,
            sensor.measurement_max_difference, sensor.max_diameter,
            sensor.min_diameter, sensor.is_active,
            tuple(tuple(x) for x in sensor.filament_array),
            sensor.lastFilamentWidthReading)

def _scripts():
    return (tuple(gcode.scripts), tuple(gcode.from_command_scripts))

def _construction():
    return (config.reads, pins.setup_pin_calls, pins.adc.samples_calls,
            gcode.registered, tuple(w for _, w in reactor.timers))
"#;

// ---------------------------------------------------------------------------
// Oracle plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("tsl fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping tsl1401cl oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "tsl1401cl".to_string(),
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

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: Result<Value, Exception>,
) {
    match (expected, actual) {
        (Err(e), Err(a)) => {
            assert_eq!(
                (e.class_name, e.message),
                (a.class_name, a.message),
                "{label}: exception mismatch"
            );
        }
        (Err(e), Ok(a)) => panic!("{label}: oracle raised {e:?} but rust returned {a:?}"),
        (Ok(v), Err(a)) => panic!("{label}: oracle returned {v:?} but rust raised {a:?}"),
        (Ok(v), Ok(a)) => {
            panic!("{label}: expected error but both returned ok: oracle {v:?}, rust {a:?}")
        }
    }
}


// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objs: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<IndexMap<String, Vec<klippy_api::printer::EventCallback>>>,
}

klippy_api::printer_object!(FakePrinter);

impl FakePrinter {
    fn new(reactor: Reactor, objs: Vec<(String, ObjectRef)>) -> Self {
        Self {
            reactor,
            objs: Mutex::new(objs.into_iter().collect()),
            handlers: Mutex::new(IndexMap::new()),
        }
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
        let mut objs = self.objs.lock();
        if objs.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{}' already created",
                name
            )));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let objs = self.objs.lock();
        if let Some(obj) = objs.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{}'",
                name
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objs = self.objs.lock();
        match module {
            None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(m) => objs
                .iter()
                .filter(|(k, _)| *k == m || k.starts_with(&format!("{} ", m)))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
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
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: klippy_api::printer::EventCallback) {
        let mut handlers = self.handlers.lock();
        handlers.entry(event.to_string()).or_default().push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut handlers = self.handlers.lock();
        let mut out = Vec::new();
        if let Some(cbs) = handlers.get_mut(event) {
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
    values: IndexMap<String, String>,
    section: String,
    reads: Mutex<Vec<String>>,
}

klippy_api::printer_object!(FakeConfig);

impl FakeConfig {
    fn new(printer: Printer, values: Vec<(String, String)>) -> Self {
        Self {
            printer,
            values: values.into_iter().collect(),
            section: "filament_width_sensor".to_string(),
            reads: Mutex::new(Vec::new()),
        }
    }
}

fn missing_msg(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{}' in section '{}' must be specified",
        option, section
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.reads.lock().push(format!("get:{}", option));
        match self.values.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(missing_msg(option, &self.section)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getint(&self, option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.values.get(option) {
            Some(v) => Ok(Some(v.parse().map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{}' in section '{}'",
                    option, self.section
                ))
            })?)),
            None => Err(missing_msg(option, &self.section)),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let above = match opts.above {
            Some(a) => format!("above={}", pycompat::text::float_str(a)),
            None => "above=None".to_string(),
        };
        self.reads
            .lock()
            .push(format!("getfloat:{}:{}", option, above));
        match self.values.get(option) {
            Some(s) => {
                let v = pycompat::text::float_from_string(s).map_err(|_| {
                    KlippyError::config(format!(
                        "Unable to parse option '{}' in section '{}'",
                        option, self.section
                    ))
                })?;
                if let Some(minv) = opts.minval {
                    if v < minv {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option,
                            self.section,
                            pycompat::text::float_str(minv)
                        )));
                    }
                }
                if let Some(maxv) = opts.maxval {
                    if v > maxv {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option,
                            self.section,
                            pycompat::text::float_str(maxv)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option,
                            self.section,
                            pycompat::text::float_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must be below {}",
                            option,
                            self.section,
                            pycompat::text::float_str(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                None => Err(missing_msg(option, &self.section)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getboolean(&self, option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.values.get(option) {
            Some(v) => Ok(Some(matches!(v.as_str(), "true" | "True" | "1" | "yes" | "on"))),
            None => Err(missing_msg(option, &self.section)),
        }
    }
    fn getchoice(
        &self,
        option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Err(missing_msg(option, &self.section))
    }
    fn getlists(&self, option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(missing_msg(option, &self.section))
    }
    fn getlist(&self, option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(missing_msg(option, &self.section))
    }
    fn getintlist(&self, option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(missing_msg(option, &self.section))
    }
    fn getfloatlist(&self, option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(missing_msg(option, &self.section))
    }
    fn getsection(&self, section: &str) -> Config {
        let mut values = IndexMap::new();
        for (k, v) in self.values.iter() {
            values.insert(k.clone(), v.clone());
        }
        let cfg = FakeConfig {
            printer: self.printer.clone(),
            values,
            section: section.to_string(),
            reads: Mutex::new(Vec::new()),
        };
        Arc::new(cfg)
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

struct FakeMcu {
    printer: Printer,
}

klippy_api::printer_object!(FakeMcu);

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        true
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::config("no pins"))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
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
        pycompat::text::trunc_float_to_bigint(time * 12000000.0)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Err(KlippyError::config("no command"))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::config("no query"))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsync)
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
        Ok(0.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        pycompat::text::trunc_float_to_bigint(print_time)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::config("no trigger dispatch"))
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, eventtime))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
}

struct FakeAsync;

impl AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

struct FakeAdcPin {
    mcu: Mcu,
    sample_calls: Mutex<Vec<(f64, f64, i64, i64, f64, f64, i64)>>,
    callback: Mutex<Option<AdcCallback>>,
}

klippy_api::printer_object!(FakeAdcPin);

impl FakeAdcPin {
    fn new(mcu: Mcu) -> Self {
        Self {
            mcu,
            sample_calls: Mutex::new(Vec::new()),
            callback: Mutex::new(None),
        }
    }
    fn invoke(&self, samples: Vec<(f64, f64)>) {
        let mut slot = self.callback.lock();
        if let Some(cb) = slot.as_mut() {
            cb(samples);
        }
    }
}

impl AdcPinApi for FakeAdcPin {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.sample_calls.lock().push((
            report_time,
            opts.sample_time,
            opts.sample_count,
            opts.batch_num,
            opts.minval,
            opts.maxval,
            opts.range_check_count,
        ));
    }
    fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.callback.lock() = Some(callback);
    }
    fn get_last_value(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
}

struct FakePins {
    adc: Arc<FakeAdcPin>,
    setup_pin_calls: Mutex<Vec<(String, String)>>,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl FakePins {
    fn new(adc: Arc<FakeAdcPin>) -> Self {
        Self {
            adc,
            setup_pin_calls: Mutex::new(Vec::new()),
        }
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("no parse"))
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("no lookup"))
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setup_pin_calls
            .lock()
            .push((pin_type.to_string(), pin_desc.to_string()));
        Ok(Pin::Adc(self.adc.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::config("no chip"))
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

struct FakeGCode {
    reactor: Reactor,
    registered: Mutex<Vec<String>>,
    scripts: Mutex<Vec<String>>,
    from_command_scripts: Mutex<Vec<String>>,
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl FakeGCode {
    fn new(reactor: Reactor) -> Self {
        Self {
            reactor,
            registered: Mutex::new(Vec::new()),
            scripts: Mutex::new(Vec::new()),
            from_command_scripts: Mutex::new(Vec::new()),
        }
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.registered.lock().push(cmd.to_string());
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
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.lock().push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.from_command_scripts.lock().push(script.to_string());
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd::new())
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

struct FakeGcmd {
    infos: Mutex<Vec<(String, bool)>>,
}

impl FakeGcmd {
    fn new() -> Self {
        Self {
            infos: Mutex::new(Vec::new()),
        }
    }
    fn infos(&self) -> Vec<(String, bool)> {
        self.infos.lock().clone()
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, _name: &str, _opts: GGetOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_int(&self, _name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: GGetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn get_command(&self) -> String {
        String::new()
    }
    fn get_commandline(&self) -> String {
        String::new()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }
    fn respond_raw(&self, msg: &str) {
        self.infos.lock().push((msg.to_string(), false));
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

struct FakeToolHead {
    pos: Mutex<Vec<f64>>,
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

impl FakeToolHead {
    fn new(pos: Vec<f64>) -> Self {
        Self {
            pos: Mutex::new(pos),
        }
    }
    fn set_pos(&self, pos: Vec<f64>) {
        *self.pos.lock() = pos;
    }
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        self.pos.lock().clone()
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("no kinematics in fake")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        0.0
    }
    fn dwell(&self, _delay: f64) {}
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        panic!("no trapq in fake")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("no extruder in fake")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {}
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("no stepper factory in fake")
    }
}

// ---------------------------------------------------------------------------
// Setup helpers.
// ---------------------------------------------------------------------------

struct RustSide {
    sensor: Arc<FilamentWidthSensor>,
    config: Arc<FakeConfig>,
    pins: Arc<FakePins>,
    adc: Arc<FakeAdcPin>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    reactor: Reactor,
}

fn default_values() -> Vec<(String, String)> {
    vec![
        ("pin".to_string(), "PA0".to_string()),
        (
            "default_nominal_filament_diameter".to_string(),
            "1.75".to_string(),
        ),
        ("measurement_delay".to_string(), "0.5".to_string()),
        ("max_difference".to_string(), "0.2".to_string()),
    ]
}

fn build_rust(values: Vec<(String, String)>, pos: Vec<f64>) -> KResult<RustSide> {
    let reactor = Reactor::new(false);
    let printer_placeholder: Printer = Arc::new(FakePrinter::new(reactor.clone(), Vec::new()));
    let mcu: Mcu = Arc::new(FakeMcu {
        printer: printer_placeholder.clone(),
    });
    let adc = Arc::new(FakeAdcPin::new(mcu));
    let pins = Arc::new(FakePins::new(adc.clone()));
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let toolhead = Arc::new(FakeToolHead::new(pos));
    let printer: Printer = Arc::new(FakePrinter::new(
        reactor.clone(),
        vec![
            ("pins".to_string(), pins.clone() as ObjectRef),
            ("gcode".to_string(), gcode.clone() as ObjectRef),
            ("toolhead".to_string(), toolhead.clone() as ObjectRef),
        ],
    ));
    let config: Arc<FakeConfig> = Arc::new(FakeConfig::new(printer.clone(), values));
    let config_obj: Config = config.clone();
    let sensor = FilamentWidthSensor::new(&config_obj)?;
    assert_eq!(gcode.registered.lock().len(), 4);
    Ok(RustSide {
        sensor,
        config,
        pins,
        adc,
        gcode,
        toolhead,
        reactor,
    })
}

fn py_build(py: &mut pyoracle::Oracle, pos: Vec<f64>, values_extra: &str) {
    let pos_str = format!("[{}]", pos.iter().map(|v| format!("{:?}", v)).collect::<Vec<_>>().join(","));
    py.exec(&format!(
        "(reactor, pins, gcode, toolhead, printer, config, sensor) = _build({{{}}}, {})",
        values_extra, pos_str
    ))
    .expect("py build");
}

fn rust_build(py_values: Vec<(String, String)>, pos: Vec<f64>) -> RustSide {
    build_rust(py_values, pos).expect("rust build")
}

fn pyvalue_to_record(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        _ => panic!("unsupported PyValue in state: {:?}", v),
    }
}

fn state_value(sensor: &FilamentWidthSensor) -> Value {
    let arr: Vec<Value> = sensor
        .filament_array()
        .into_iter()
        .map(|(p, w)| Value::Tuple(vec![Value::float(p), pyvalue_to_record(&w)]))
        .collect();
    Value::Tuple(vec![
        Value::float(sensor.nominal_filament_dia()),
        Value::float(sensor.measurement_delay()),
        Value::float(sensor.measurement_max_difference()),
        Value::float(sensor.max_diameter()),
        Value::float(sensor.min_diameter()),
        Value::Bool(sensor.is_active()),
        Value::Tuple(arr),
        pyvalue_to_record(&sensor.last_filament_width_reading()),
    ])
}

fn scripts_value(gcode: &FakeGCode) -> Value {
    let a: Vec<Value> = gcode
        .scripts
        .lock()
        .iter()
        .map(|s| Value::str(s))
        .collect();
    let b: Vec<Value> = gcode
        .from_command_scripts
        .lock()
        .iter()
        .map(|s| Value::str(s))
        .collect();
    Value::Tuple(vec![Value::Tuple(a), Value::Tuple(b)])
}

fn construction_value(rust: &RustSide) -> Value {
    let reads: Vec<Value> = rust
        .config
        .reads
        .lock()
        .iter()
        .map(|s| Value::str(s))
        .collect();
    let pins: Vec<Value> = rust
        .pins
        .setup_pin_calls
        .lock()
        .iter()
        .map(|(t, d)| Value::Tuple(vec![Value::str(t), Value::str(d)]))
        .collect();
    let samples: Vec<Value> = rust
        .adc
        .sample_calls
        .lock()
        .iter()
        .map(|(r, s, c, b, mn, mx, rc)| {
            Value::Tuple(vec![
                Value::float(*r),
                Value::float(*s),
                Value::int(*c),
                Value::int(*b),
                Value::float(*mn),
                Value::float(*mx),
                Value::int(*rc),
            ])
        })
        .collect();
    let registered: Vec<Value> = rust
        .gcode
        .registered
        .lock()
        .iter()
        .map(|s| Value::str(s))
        .collect();
    // Single timer registered with NEVER waketime.
    Value::Tuple(vec![
        Value::List(reads),
        Value::List(pins),
        Value::List(samples),
        Value::List(registered),
        Value::Tuple(vec![Value::float(reactor_never())]),
    ])
}

fn reactor_never() -> f64 {
    reactor::NEVER
}

fn py_adc_samples_py(samples: &[(f64, f64)]) -> String {
    let inner = samples
        .iter()
        .map(|(t, v)| format!("({:?},{:?})", t, v))
        .collect::<Vec<_>>()
        .join(",");
    format!("[{}]", inner)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn construction_matches_python() {
    let Some(mut py) = oracle() else { return };
    let pos = vec![0.0, 0.0, 0.0, 10.0];
    py_build(&mut py, pos.clone(), "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2");
    let rust = rust_build(default_values(), pos);

    check("construction", py.eval("_construction()"), construction_value(&rust));
    check("state", py.eval("_state()"), state_value(&rust.sensor));
}

#[test]
fn adc_callback_matches_python() {
    let Some(mut py) = oracle() else { return };
    let pos = vec![0.0, 0.0, 0.0, 10.0];
    py_build(&mut py, pos.clone(), "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2");
    let rust = rust_build(default_values(), pos);
    py.exec("sensor.handle_ready()").expect("py ready");
    rust.sensor.handle_ready().expect("rust ready");

    let samples = [(1.0, 0.35), (1.5, 0.3521), (2.0, 0.25)];
    py.exec(&format!(
        "pins.adc.invoke({})",
        py_adc_samples_py(&samples)
    ))
    .expect("py adc");
    rust.adc.invoke(samples.to_vec());

    check(
        "adc",
        py.eval("sensor.lastFilamentWidthReading"),
        pyvalue_to_record(&rust.sensor.last_filament_width_reading()),
    );

    // Second batch: uses only the last sample.
    let samples2 = [(3.0, 0.5), (3.5, 0.7)];
    py.exec(&format!(
        "pins.adc.invoke({})",
        py_adc_samples_py(&samples2)
    ))
    .expect("py adc2");
    rust.adc.invoke(samples2.to_vec());
    check(
        "adc2",
        py.eval("sensor.lastFilamentWidthReading"),
        pyvalue_to_record(&rust.sensor.last_filament_width_reading()),
    );
}

#[test]
fn query_filament_width_matches_python() {
    let Some(mut py) = oracle() else { return };
    let pos = vec![0.0, 0.0, 0.0, 10.0];
    py_build(&mut py, pos.clone(), "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2");
    let rust = rust_build(default_values(), pos);
    py.exec("sensor.handle_ready()").expect("py ready");
    rust.sensor.handle_ready().expect("rust ready");

    py.exec("gcmd = FakeGcmd()").expect("py gcmd");
    py.exec("sensor.cmd_M407(gcmd)").expect("py m407 absent");
    let rust_gcmd = FakeGcmd::new();
    let rust_gcmd_arc: GCmd = Arc::new(FakeGcmd::new());
    // Use a fresh fake and capture through the trait object identity.
    let capture = Arc::new(FakeGcmd::new());
    let capture_cmd: GCmd = capture.clone();
    rust.sensor.cmd_m407(&capture_cmd).expect("rust m407 absent");
    let _ = rust_gcmd;
    let _ = rust_gcmd_arc;
    let rust_infos: Vec<Value> = capture
        .infos()
        .into_iter()
        .map(|(m, l)| Value::Tuple(vec![Value::str(&m), Value::Bool(l)]))
        .collect();
    check(
        "m407 absent",
        py.eval("gcmd.infos"),
        Value::List(rust_infos),
    );

    // With a reading present.
    py.exec("pins.adc.invoke([(1.0, 0.35)])").expect("py adc");
    rust.adc.invoke(vec![(1.0, 0.35)]);
    py.exec("gcmd = FakeGcmd()").expect("py gcmd2");
    py.exec("sensor.cmd_M407(gcmd)").expect("py m407 present");
    let capture2 = Arc::new(FakeGcmd::new());
    let capture2_cmd: GCmd = capture2.clone();
    rust.sensor.cmd_m407(&capture2_cmd).expect("rust m407 present");
    let rust_infos2: Vec<Value> = capture2
        .infos()
        .into_iter()
        .map(|(m, l)| Value::Tuple(vec![Value::str(&m), Value::Bool(l)]))
        .collect();
    check(
        "m407 present",
        py.eval("gcmd.infos"),
        Value::List(rust_infos2),
    );
}

#[test]
fn clear_enable_disable_matches_python() {
    let Some(mut py) = oracle() else { return };
    let pos = vec![0.0, 0.0, 0.0, 10.0];
    py_build(&mut py, pos.clone(), "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2");
    let rust = rust_build(default_values(), pos);
    py.exec("sensor.handle_ready()").expect("py ready");
    rust.sensor.handle_ready().expect("rust ready");

    // Populate the array so clearing is observable.
    py.exec("pins.adc.invoke([(1.0, 0.35)])").expect("py adc");
    rust.adc.invoke(vec![(1.0, 0.35)]);
    py.exec("toolhead.pos = [0.,0.,0.,20.]").expect("py pos");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 20.0]);
    py.exec("sensor.update_filament_array(20.0)").expect("py update");
    rust.sensor.update_filament_array(20.0);
    check("state after update", py.eval("_state()"), state_value(&rust.sensor));
    py.exec("sensor.update_filament_array(21.0)").expect("py update early");
    rust.sensor.update_filament_array(21.0);
    check("state after early update", py.eval("_state()"), state_value(&rust.sensor));
    py.exec("sensor.update_filament_array(31.0)").expect("py update late");
    rust.sensor.update_filament_array(31.0);
    check("state after late update", py.eval("_state()"), state_value(&rust.sensor));

    for (label, py_call, rust_cmd) in [
        ("clear", "sensor.cmd_ClearFilamentArray(gcmd)", "clear"),
        ("disable", "sensor.cmd_M406(gcmd)", "disable"),
        ("disable again", "sensor.cmd_M406(gcmd)", "disable"),
        ("enable", "sensor.cmd_M405(gcmd)", "enable"),
        ("enable again", "sensor.cmd_M405(gcmd)", "enable"),
    ] {
        py.exec("gcmd = FakeGcmd()").expect("py gcmd");
        py.exec(py_call).expect("py cmd");
        let capture = Arc::new(FakeGcmd::new());
        let cmd: GCmd = capture.clone();
        match rust_cmd {
            "clear" => rust.sensor.cmd_clear_filament_array(&cmd).expect("rust clear"),
            "disable" => rust.sensor.cmd_m406(&cmd).expect("rust disable"),
            "enable" => rust.sensor.cmd_m405(&cmd).expect("rust enable"),
            _ => unreachable!(),
        }
        let rust_infos: Vec<Value> = capture
            .infos()
            .into_iter()
            .map(|(m, l)| Value::Tuple(vec![Value::str(&m), Value::Bool(l)]))
            .collect();
        check(label, py.eval("gcmd.infos"), Value::List(rust_infos));
        check(
            &format!("{label} state"),
            py.eval("_state()"),
            state_value(&rust.sensor),
        );
        check(
            &format!("{label} scripts"),
            py.eval("_scripts()"),
            scripts_value(&rust.gcode),
        );
        // Reset script captures for the next step.
        py.exec("gcode.scripts = []; gcode.from_command_scripts = []").expect("py reset");
        rust.gcode.scripts.lock().clear();
        rust.gcode.from_command_scripts.lock().clear();
    }
}

#[test]
fn extrude_update_matches_python() {
    let Some(mut py) = oracle() else { return };
    let pos = vec![0.0, 0.0, 0.0, 10.0];
    py_build(&mut py, pos.clone(), "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2");
    let rust = rust_build(default_values(), pos);
    py.exec("sensor.handle_ready()").expect("py ready");
    rust.sensor.handle_ready().expect("rust ready");

    // Drains one queued reading whose width is within [min, max]:
    // width = round(0.32*5, 2) = 1.6, nominal 1.75 -> S120.
    py.exec("pins.adc.invoke([(1.0, 0.32)])").expect("py adc");
    rust.adc.invoke(vec![(1.0, 0.32)]);
    py.exec("toolhead.pos = [0.,0.,0.,10.5]").expect("py pos");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 10.5]);
    py.exec("sensor.update_filament_array(10.5)").expect("py update");
    rust.sensor.update_filament_array(10.5);
    check("queued state", py.eval("_state()"), state_value(&rust.sensor));
    py.exec("toolhead.pos = [0.,0.,0.,12.]").expect("py pos1b");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 12.0]);
    py.exec("ret = sensor.extrude_factor_update_event(100.0)").expect("py event");
    let rust_ret = rust.sensor.extrude_factor_update_event(100.0);
    check("event ret", py.eval("ret"), Value::float(rust_ret));
    check("event scripts", py.eval("_scripts()"), scripts_value(&rust.gcode));
    check("event state", py.eval("_state()"), state_value(&rust.sensor));
    py.exec("gcode.scripts = []; gcode.from_command_scripts = []").expect("py reset");
    rust.gcode.scripts.lock().clear();
    rust.gcode.from_command_scripts.lock().clear();

    // Width outside [min, max] (max = 1.95): 0.5*5 = 2.5 -> S100.
    py.exec("pins.adc.invoke([(2.0, 0.5)])").expect("py adc2");
    rust.adc.invoke(vec![(2.0, 0.5)]);
    py.exec("toolhead.pos = [0.,0.,0.,30.]").expect("py pos2");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 30.0]);
    py.exec("sensor.update_filament_array(30.0)").expect("py update2");
    rust.sensor.update_filament_array(30.0);
    py.exec("toolhead.pos = [0.,0.,0.,31.]").expect("py pos2b");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 31.0]);
    py.exec("ret = sensor.extrude_factor_update_event(101.0)").expect("py event2");
    let rust_ret2 = rust.sensor.extrude_factor_update_event(101.0);
    check("event2 ret", py.eval("ret"), Value::float(rust_ret2));
    check("event2 scripts", py.eval("_scripts()"), scripts_value(&rust.gcode));
    check("event2 state", py.eval("_state()"), state_value(&rust.sensor));
    py.exec("gcode.scripts = []; gcode.from_command_scripts = []").expect("py reset2");
    rust.gcode.scripts.lock().clear();
    rust.gcode.from_command_scripts.lock().clear();

    // No filament (reading 0): S100 and array cleared.
    py.exec("pins.adc.invoke([(3.0, 0.0)])").expect("py adc3");
    rust.adc.invoke(vec![(3.0, 0.0)]);
    py.exec("toolhead.pos = [0.,0.,0.,40.]").expect("py pos3");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 40.0]);
    py.exec("ret = sensor.extrude_factor_update_event(102.0)").expect("py event3");
    let rust_ret3 = rust.sensor.extrude_factor_update_event(102.0);
    check("event3 ret", py.eval("ret"), Value::float(rust_ret3));
    check("event3 scripts", py.eval("_scripts()"), scripts_value(&rust.gcode));
    check("event3 state", py.eval("_state()"), state_value(&rust.sensor));
    py.exec("gcode.scripts = []; gcode.from_command_scripts = []").expect("py reset3");
    rust.gcode.scripts.lock().clear();
    rust.gcode.from_command_scripts.lock().clear();

    // Pending position ahead of the extruder: no script emitted.
    py.exec("pins.adc.invoke([(4.0, 0.35)])").expect("py adc4");
    rust.adc.invoke(vec![(4.0, 0.35)]);
    py.exec("toolhead.pos = [0.,0.,0.,50.]").expect("py pos4");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 50.0]);
    py.exec("sensor.update_filament_array(50.0)").expect("py update4");
    rust.sensor.update_filament_array(50.0);
    // Move extruder back so pending (50.5) is ahead; the event still
    // appends nothing (interval) and emits nothing.
    py.exec("toolhead.pos = [0.,0.,0.,45.]").expect("py pos5");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 45.0]);
    py.exec("ret = sensor.extrude_factor_update_event(103.0)").expect("py event4");
    let rust_ret4 = rust.sensor.extrude_factor_update_event(103.0);
    check("event4 ret", py.eval("ret"), Value::float(rust_ret4));
    check("event4 scripts", py.eval("_scripts()"), scripts_value(&rust.gcode));
    check("event4 state", py.eval("_state()"), state_value(&rust.sensor));
}

#[test]
fn inactive_returns_never_matches_python() {
    let Some(mut py) = oracle() else { return };
    let pos = vec![0.0, 0.0, 0.0, 10.0];
    py_build(&mut py, pos.clone(), "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2");
    let rust = rust_build(default_values(), pos);
    py.exec("sensor.handle_ready()").expect("py ready");
    rust.sensor.handle_ready().expect("rust ready");
    py.exec("gcmd = FakeGcmd()").expect("py gcmd");
    py.exec("sensor.cmd_M406(gcmd)").expect("py off");
    let off = Arc::new(FakeGcmd::new());
    let off_cmd: GCmd = off.clone();
    rust.sensor.cmd_m406(&off_cmd).expect("rust off");

    py.exec("toolhead.pos = [0.,0.,0.,10.]").expect("py pos");
    rust.toolhead.set_pos(vec![0.0, 0.0, 0.0, 10.0]);
    py.exec("ret = sensor.extrude_factor_update_event(200.0)").expect("py event");
    let rust_ret = rust.sensor.extrude_factor_update_event(200.0);
    check("inactive ret", py.eval("ret"), Value::float(rust_ret));
    check("inactive scripts", py.eval("_scripts()"), scripts_value(&rust.gcode));
}

#[test]
fn config_errors_match_python() {
    let Some(mut py) = oracle() else { return };

    // Missing pin.
    let expected = py.exec("(reactor, pins, gcode, toolhead, printer, config, sensor) = _build({'default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.2}, [0.,0.,0.,10.])");
    let rust_result: Result<Value, Exception> = (|| {
        let reactor = Reactor::new(false);
        let printer: Printer = Arc::new(FakePrinter::new(reactor.clone(), Vec::new()));
        let mcu: Mcu = Arc::new(FakeMcu {
            printer: printer.clone(),
        });
        let adc = Arc::new(FakeAdcPin::new(mcu));
        let pins = Arc::new(FakePins::new(adc));
        let gcode = Arc::new(FakeGCode::new(reactor.clone()));
        let toolhead = Arc::new(FakeToolHead::new(vec![0.0, 0.0, 0.0, 10.0]));
        let printer: Printer = Arc::new(FakePrinter::new(
            reactor,
            vec![
                ("pins".to_string(), pins as ObjectRef),
                ("gcode".to_string(), gcode as ObjectRef),
                ("toolhead".to_string(), toolhead as ObjectRef),
            ],
        ));
        let values = vec![
            (
                "default_nominal_filament_diameter".to_string(),
                "1.75".to_string(),
            ),
            ("measurement_delay".to_string(), "0.5".to_string()),
            ("max_difference".to_string(), "0.2".to_string()),
        ];
        let config: Arc<FakeConfig> = Arc::new(FakeConfig::new(printer, values));
        let config_obj: Config = config;
        match FilamentWidthSensor::new(&config_obj) {
            Ok(_) => Ok(Value::str("no-error")),
            Err(e) => Err(Exception {
                class_name: e.class_name().to_string(),
                message: e.message().to_string(),
            }),
        }
    })();
    check_err("missing pin", expected, rust_result);
}

#[test]
fn above_errors_match_python() {
    let Some(mut py) = oracle() else { return };

    for (label, py_values, rust_values) in [
        (
            "nominal at bound",
            "'pin':'PA0','default_nominal_filament_diameter':1.0,'measurement_delay':0.5,'max_difference':0.2",
            vec![
                ("pin".to_string(), "PA0".to_string()),
                (
                    "default_nominal_filament_diameter".to_string(),
                    "1.0".to_string(),
                ),
                ("measurement_delay".to_string(), "0.5".to_string()),
                ("max_difference".to_string(), "0.2".to_string()),
            ],
        ),
        (
            "delay at bound",
            "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.0,'max_difference':0.2",
            vec![
                ("pin".to_string(), "PA0".to_string()),
                (
                    "default_nominal_filament_diameter".to_string(),
                    "1.75".to_string(),
                ),
                ("measurement_delay".to_string(), "0.0".to_string()),
                ("max_difference".to_string(), "0.2".to_string()),
            ],
        ),
        (
            "difference at bound",
            "'pin':'PA0','default_nominal_filament_diameter':1.75,'measurement_delay':0.5,'max_difference':0.0",
            vec![
                ("pin".to_string(), "PA0".to_string()),
                (
                    "default_nominal_filament_diameter".to_string(),
                    "1.75".to_string(),
                ),
                ("measurement_delay".to_string(), "0.5".to_string()),
                ("max_difference".to_string(), "0.0".to_string()),
            ],
        ),
    ] {
        let expected = py.exec(&format!(
            "(reactor, pins, gcode, toolhead, printer, config, sensor) = _build({{{}}}, [0.,0.,0.,10.])",
            py_values
        ));
        let result: Result<Value, Exception> = (|| {
            let reactor = Reactor::new(false);
            let printer_placeholder: Printer = Arc::new(FakePrinter::new(reactor.clone(), Vec::new()));
            let mcu: Mcu = Arc::new(FakeMcu {
                printer: printer_placeholder.clone(),
            });
            let adc = Arc::new(FakeAdcPin::new(mcu));
            let pins = Arc::new(FakePins::new(adc));
            let gcode = Arc::new(FakeGCode::new(reactor.clone()));
            let toolhead = Arc::new(FakeToolHead::new(vec![0.0, 0.0, 0.0, 10.0]));
            let printer: Printer = Arc::new(FakePrinter::new(
                reactor,
                vec![
                    ("pins".to_string(), pins as ObjectRef),
                    ("gcode".to_string(), gcode as ObjectRef),
                    ("toolhead".to_string(), toolhead as ObjectRef),
                ],
            ));
            let config: Arc<FakeConfig> = Arc::new(FakeConfig::new(printer, rust_values));
            let config_obj: Config = config;
            match FilamentWidthSensor::new(&config_obj) {
                Ok(_) => Ok(Value::str("no-error")),
                Err(e) => Err(Exception {
                    class_name: e.class_name().to_string(),
                    message: e.message().to_string(),
                }),
            }
        })();
        check_err(label, expected, result);
    }
}
