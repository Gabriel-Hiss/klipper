//! Parity tests for `extras::spi_temperature` (port of
//! `klippy/extras/spi_temperature.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). Each scenario builds a sensor on
//! both sides from the same config options: Python uses the real
//! `configfile.ConfigWrapper`, `extras.bus.MCU_SPI_from_config` and
//! `extras.spi_temperature`, with minimal fakes for the printer, pins and
//! MCU; the port runs against fakes of the contract traits (with its real
//! `extras::bus`). Both sides execute the same steps (`setup_minmax`,
//! `setup_callback`, MCU config callbacks, `thermocouple_result` responses,
//! `calc_temp`, `calc_adc`, `get_report_time_delta`) and render one trace:
//! results (floats by `repr`), raised errors (class and message), temperature
//! callback calls, then every MCU config command, command lookup, command
//! send, serial response registration, pin-sharing reset, reserved pin,
//! deprecation notice and `logging.warning` line. The traces must be equal.
//!
//! Errors the Python response handler raises (no callback installed)
//! surface as panics in the port; the trace compares them as `Class|message`.
//!
//! There are no recordings under `tests/fixtures/recordings/` for these
//! units, so expected values come from the oracle alone.

use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi,
};
use extras::spi_temperature::{module_factory, sensors, SensorBase, REPORT_TIME};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueryWrapper,
    CommandQueryWrapperApi, CommandQueue, CommandWrapper, CommandWrapperApi, ConfigCallback,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams,
    PinResolver, PinResolverApi, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::{float_from_string, float_repr, float_str, int_from_string, trunc_float_to_bigint};
use pycompat::value::py_equal;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import sys
import types
_mcu_mod = types.ModuleType("mcu")
class _McuModError(Exception):
    pass
_McuModError.__name__ = "error"
_mcu_mod.error = _McuModError
def _get_printer_mcu(printer, name):
    if name == 'mcu':
        return printer.lookup_object(name)
    return printer.lookup_object('mcu ' + name)
_mcu_mod.get_printer_mcu = _get_printer_mcu
sys.modules['mcu'] = _mcu_mod
import configfile
import extras.bus as bus
import extras.spi_temperature as spit

class _PinsError(Exception):
    pass
_PinsError.__name__ = "error"

class _McuError(Exception):
    pass
_McuError.__name__ = "error"

def _norm(data):
    return [bytes(p) if isinstance(p, list) else p for p in data]

class FakeLog:
    def __init__(self):
        self.lines = []
    def warning(self, msg):
        self.lines.append("WARNING|%s" % (msg,))
fakelog = FakeLog()
spit.logging = fakelog

class FakeCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append("send|%s|%r|%r|%r" % (self.fmt, _norm(list(data)), minclock, reqclock))

class FakeQuery:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt

class FakeMcu:
    error = _McuError
    def __init__(self, printer, name="mcu"):
        self._printer = printer
        self._name = name
        self._next_oid = 0
        self.config_cmds = []
        self.lookups = []
        self.sends = []
        self.config_cbs = []
        self.serial_cbs = []
    def get_name(self):
        return self._name
    def get_printer(self):
        return self._printer
    def get_enumerations(self):
        return {}
    def get_constants(self):
        return {}
    def create_oid(self):
        self._next_oid += 1
        return self._next_oid
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append("%r|%r|%r" % (cmd, is_init, on_restart))
    def alloc_command_queue(self):
        return 7
    def register_config_callback(self, cb):
        self.config_cbs.append(cb)
    def try_lookup_command(self, fmt):
        self.lookups.append("try|%s" % fmt)
        return None
    def lookup_command(self, fmt, cq=None):
        self.lookups.append("cmd|%s|%r" % (fmt, cq))
        return FakeCmd(self, fmt)
    def lookup_query_command(self, fmt, resp, oid=None, cq=None, is_async=False):
        self.lookups.append("query|%s|%s|%r|%r" % (fmt, resp, oid, cq))
        return FakeQuery(self, fmt)
    def seconds_to_clock(self, t):
        return int(t * 1000000)
    def get_query_slot(self, oid):
        return 1000 + oid
    def clock32_to_clock64(self, clock32):
        return clock32 + 4294967296
    def clock_to_print_time(self, clock):
        return clock / 1000000.0
    def register_serial_response(self, cb, fmt, oid=None):
        self.serial_cbs.append("%s|%s" % (fmt, oid))

class FakeResolver:
    def __init__(self, pins):
        self.pins = pins
    def reserve_pin(self, pin, reserve_name):
        self.pins.reserved.append("%s|%s" % (pin, reserve_name))

class FakePins:
    error = _PinsError
    def __init__(self, printer):
        self.printer = printer
        self.chips = {}
        self.active = {}
        self.reserved = []
        self.resets = []
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        desc = pin_desc.strip()
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        if chip_name not in self.chips:
            raise self.error("Unknown pin chip name '%s'" % (chip_name,))
        share_name = "%s:%s" % (chip_name, pin)
        if share_name in self.active:
            return self.active[share_name]
        params = {'chip': self.chips[chip_name], 'chip_name': chip_name,
                  'pin': pin, 'invert': 0, 'pullup': 0, 'share_type': share_type}
        self.active[share_name] = params
        return params
    def reset_pin_sharing(self, pin_params):
        share_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        self.resets.append(share_name)
        del self.active[share_name]
    def get_pin_resolver(self, chip_name):
        return FakeResolver(self)

class FakeConfigFile:
    def __init__(self, printer):
        self.printer = printer
        self.deprecated = []
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        self.deprecated.append("%s|%r" % (feature, msg))

class FakeHeaters:
    def __init__(self):
        self.factories = []
    def add_sensor_factory(self, sensor_type, factory):
        self.factories.append((sensor_type, factory))

class FakePrinter:
    def __init__(self):
        self.objects = {}
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            return None
        raise configparser.Error("missing %s" % name)
    def load_object(self, config, section, default=configfile.sentinel):
        if section in self.objects:
            return self.objects[section]
        raise configparser.Error("Unknown module %s" % section)

def make_world(section, values):
    p = FakePrinter()
    mcu = FakeMcu(p, "mcu")
    aux = FakeMcu(p, "aux")
    pins = FakePins(p)
    pins.chips = {"mcu": mcu, "aux": aux}
    cf = FakeConfigFile(p)
    p.objects = {"pins": pins, "configfile": cf, "mcu": mcu, "mcu aux": aux}
    fileconfig = configparser.RawConfigParser()
    fileconfig.add_section(section)
    for k, v in values.items():
        fileconfig.set(section, k, v)
    cfg = configfile.ConfigWrapper(p, fileconfig, {}, section)
    return p, mcu, aux, pins, cf, cfg

def exc_line(e):
    return "EXC|%s|%s" % (type(e).__name__, e)

def run(factory, values, steps, observe=True):
    out = []
    fakelog.lines = []
    p, mcu, aux, pins, cf, cfg = make_world('extruder', values)
    sensor = None
    try:
        sensor = factory(cfg)
    except Exception as e:
        out.append(exc_line(e))
    if sensor is not None:
        if observe:
            out.append("oid|%d" % (sensor.oid,))
        for st in steps:
            op = st[0]
            try:
                if op == 'minmax':
                    sensor.setup_minmax(st[1], st[2])
                    if observe:
                        out.append("minmax|%d|%d" % (sensor.min_sample_value, sensor.max_sample_value))
                elif op == 'callback':
                    sensor.setup_callback(lambda t, v: out.append("cb|%r|%r" % (t, v)))
                elif op == 'config':
                    for m in (mcu, aux):
                        for cb in m.config_cbs:
                            cb()
                elif op == 'resp':
                    sensor._handle_spi_response({'oid': sensor.oid, 'next_clock': st[1], 'value': st[2], 'fault': st[3]})
                elif op == 'temp':
                    out.append("temp|%r" % (sensor.calc_temp(st[1]),))
                elif op == 'adc':
                    out.append("adc|%r" % (sensor.calc_adc(st[1]),))
                elif op == 'delta':
                    out.append("delta|%r" % (sensor.get_report_time_delta(),))
            except Exception as e:
                out.append(exc_line(e))
    for name, m in (('mcu', mcu), ('aux', aux)):
        for tag, lst in (('cfg', m.config_cmds), ('lookup', m.lookups), ('send', m.sends), ('serial', m.serial_cbs)):
            for x in lst:
                out.append("%s_%s|%s" % (name, tag, x))
    for x in pins.resets:
        out.append("reset|%s" % (x,))
    for x in pins.reserved:
        out.append("reserved|%s" % (x,))
    for x in cf.deprecated:
        out.append("deprecated|%s" % (x,))
    for x in fakelog.lines:
        out.append("log|%s" % (x,))
    return out

def load_world():
    global loaded, load_out
    p, mcu, aux, pins, cf, cfg = make_world('spi_temperature', {})
    fh = FakeHeaters()
    p.objects['heaters'] = fh
    ret = spit.load_config(cfg)
    load_out = ["ret|%r" % (ret,)] + ["factory|%s" % (n,) for n, k in fh.factories]
    loaded = dict(fh.factories)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Log = Arc<GCell<Vec<String>>>;

fn new_log() -> Log {
    Arc::new(GCell::new(Vec::new()))
}

fn py_repr(v: &PyValue) -> String {
    match v {
        PyValue::None => "None".to_string(),
        PyValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
        PyValue::Int(n) => n.inner().to_string(),
        PyValue::Float(f) => float_repr(*f),
        PyValue::Str(s) => pycompat::text::str_repr(s),
        PyValue::Bytes(b) => pycompat::text::bytes_repr(b),
        PyValue::List(l) => format!("[{}]", l.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        other => panic!("py_repr: unsupported value {other:?}"),
    }
}

fn list_repr(v: &[PyValue]) -> String {
    format!("[{}]", v.iter().map(py_repr).collect::<Vec<_>>().join(", "))
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn opt_repr<T: ToString>(v: Option<T>) -> String {
    v.map(|x| x.to_string()).unwrap_or_else(|| "None".to_string())
}

struct FakeCmd {
    log: Log,
    fmt: String,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.log.borrow_mut().push(format!(
            "send|{}|{}|{}|{}",
            self.fmt,
            list_repr(&data),
            opts.minclock,
            opts.reqclock
        ));
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        panic!("send_wait_ack is not used by spi_temperature")
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeQuery;

impl CommandQueryWrapperApi for FakeQuery {
    fn send(&self, _data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        panic!("query send is not used by spi_temperature")
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used by spi_temperature")
    }
}

struct FakeAsync;

impl AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

struct FakeMcu {
    name: String,
    printer: GCell<Option<Printer>>,
    next_oid: GCell<i64>,
    config_cmds: Log,
    lookups: Log,
    sends: Log,
    serial_cbs: Log,
    serial_cb: GCell<Option<SerialResponseCallback>>,
    config_cbs: GCell<Vec<ConfigCallback>>,
}

impl FakeMcu {
    fn new(name: &str) -> Arc<FakeMcu> {
        Arc::new(FakeMcu {
            name: name.to_string(),
            printer: GCell::new(None),
            next_oid: GCell::new(0),
            config_cmds: new_log(),
            lookups: new_log(),
            sends: new_log(),
            serial_cbs: new_log(),
            serial_cb: GCell::new(None),
            config_cbs: GCell::new(Vec::new()),
        })
    }

    /// Runs the registered config callbacks in registration order and stops
    /// at the first error, like the Python loop.
    fn run_config_cbs(&self) -> KResult<()> {
        let mut cbs = std::mem::take(&mut *self.config_cbs.borrow_mut());
        let mut result = Ok(());
        for cb in cbs.iter_mut() {
            result = cb();
            if result.is_err() {
                break;
            }
        }
        *self.config_cbs.borrow_mut() = cbs;
        result
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.borrow().clone().expect("printer set")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by spi_temperature")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut oid = self.next_oid.borrow_mut();
        *oid += 1;
        Ok(*oid)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.config_cbs.borrow_mut().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.config_cmds.borrow_mut().push(format!(
            "{}|{}|{}",
            pycompat::text::str_repr(cmd),
            py_bool(opts.is_init),
            py_bool(opts.on_restart)
        ));
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by spi_temperature")
    }
    fn get_query_slot(&self, oid: i64) -> i64 {
        1000 + oid
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        trunc_float_to_bigint(time * 1000000.0)
            .inner()
            .to_string()
            .parse::<i64>()
            .expect("clock fits i64")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by spi_temperature")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by spi_temperature")
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        self.lookups.borrow_mut().push(format!("cmd|{}|{}", msgformat, opt_repr(cq.map(|c| c.0))));
        Ok(Arc::new(FakeCmd { log: self.sends.clone(), fmt: msgformat.to_string() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.lookups.borrow_mut().push(format!(
            "query|{}|{}|{}|{}",
            msgformat,
            respformat,
            opt_repr(opts.oid),
            opt_repr(opts.cq.map(|c| c.0))
        ));
        Ok(Arc::new(FakeQuery))
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.lookups.borrow_mut().push(format!("try|{msgformat}"));
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.serial_cbs.borrow_mut().push(format!("{}|{}", msg, opt_repr(oid)));
        *self.serial_cb.borrow_mut() = Some(cb);
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by spi_temperature")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by spi_temperature")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by spi_temperature")
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000000.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by spi_temperature")
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32 + 4294967296
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by spi_temperature")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by spi_temperature")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by spi_temperature")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by spi_temperature")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakeResolver {
    reserved: Log,
}

impl PinResolverApi for FakeResolver {
    fn reserve_pin(&self, pin: &str, reserve_name: &str) -> KResult<()> {
        self.reserved.borrow_mut().push(format!("{pin}|{reserve_name}"));
        Ok(())
    }
    fn alias_pin(&self, _alias: &str, _pin: &str) -> KResult<()> {
        panic!("alias_pin is not used by spi_temperature")
    }
    fn update_command(&self, _cmd: &str) -> KResult<String> {
        panic!("update_command is not used by spi_temperature")
    }
}

struct FakePins {
    chips: GCell<IndexMap<String, Mcu>>,
    active: GCell<IndexMap<String, PinParams>>,
    reserved: Log,
    resets: Log,
}

impl FakePins {
    fn new() -> Arc<FakePins> {
        Arc::new(FakePins {
            chips: GCell::new(IndexMap::new()),
            active: GCell::new(IndexMap::new()),
            reserved: new_log(),
            resets: new_log(),
        })
    }

    fn pin_error(msg: String) -> KlippyError {
        KlippyError::Pins(msg)
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        let desc = pin_desc.trim();
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu", desc),
            Some((c, p)) => (c.trim(), p.trim()),
        };
        let chip = self.chips.borrow().get(chip_name).cloned();
        match chip {
            None => Err(FakePins::pin_error(format!("Unknown pin chip name '{chip_name}'"))),
            Some(chip) => Ok(PinParams {
                chip,
                chip_name: chip_name.to_string(),
                pin: pin.to_string(),
                invert: 0,
                pullup: 0,
                share_type: None,
            }),
        }
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let mut params = self.parse_pin(
            pin_desc,
            ParsePinOpts { can_invert: opts.can_invert, can_pullup: opts.can_pullup },
        )?;
        let share_name = format!("{}:{}", params.chip_name, params.pin);
        if let Some(existing) = self.active.borrow().get(&share_name) {
            return Ok(existing.clone());
        }
        params.share_type = opts.share_type;
        self.active.borrow_mut().insert(share_name, params.clone());
        Ok(params)
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("PinsApi::setup_pin is not used by spi_temperature")
    }
    fn reset_pin_sharing(&self, pin_params: PinParams) {
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        self.resets.borrow_mut().push(share_name.clone());
        self.active.borrow_mut().shift_remove(&share_name);
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Ok(Arc::new(FakeResolver { reserved: self.reserved.clone() }))
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by spi_temperature")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by spi_temperature")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeConfigFile {
    deprecated: Log,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by spi_temperature")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config is not used by spi_temperature")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("read_main_config is not used by spi_temperature")
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {}
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, feature: &str, msg: Option<String>) {
        let msg = match msg {
            None => "None".to_string(),
            Some(m) => pycompat::text::str_repr(&m),
        };
        self.deprecated.borrow_mut().push(format!("{feature}|{msg}"));
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> { Ok(()) }
    fn remove_section(&self, _section: &str) {}
}

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

/// The `heaters` object: records the registered sensor factories.
struct FakeHeaters {
    factories: GCell<Vec<(String, SensorFactory)>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories.borrow_mut().push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used by spi_temperature")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("get_all_heaters is not used by spi_temperature")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("lookup_heater is not used by spi_temperature")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used by spi_temperature")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used by spi_temperature")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("register_monitor is not used by spi_temperature")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used by spi_temperature")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature is not used by spi_temperature")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter { objects: GCell::new(IndexMap::new()) })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
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
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::ReturnNone => Ok(None),
                LookupDefault::Raise => Err(KlippyError::Config(format!("missing {name}"))),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects is not used by spi_temperature")
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(section) {
            Some(obj) => Ok(Some(obj.clone())),
            None => Err(KlippyError::Config(format!("Unknown module {section}"))),
        }
    }
    fn run(&self) -> Option<String> {
        panic!("run is not used by spi_temperature")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_shutdown is not used by spi_temperature")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown is not used by spi_temperature")
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {
        panic!("register_event_handler is not used by spi_temperature")
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("send_event is not used by spi_temperature")
    }
    fn request_exit(&self, _result: &str) {}
}

/// `configfile.ConfigWrapper` over a string option map, with the parsing
/// and error texts of `configfile.py:29-86` and `configparser`.
struct FakeConfig {
    printer: Printer,
    section: String,
    values: IndexMap<String, String>,
}

impl FakeConfig {
    fn build(printer: Printer, section: &str, values: &[(&str, &str)]) -> Config {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            values: values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        })
    }

    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
    }

    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.section
        ))
    }

    fn parse_int(&self, option: &str, raw: &str) -> KResult<i64> {
        let value = int_from_string(raw, 10).map_err(|_| self.unparsable(option))?;
        value.inner().to_string().parse::<i64>().map_err(|_| self.unparsable(option))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.values.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.values.get(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let v = self.parse_int(option, raw)?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {minval}",
                    self.section
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {maxval}",
                    self.section
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.values.get(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let v = float_from_string(raw).map_err(|_| self.unparsable(option))?;
        let bound = |what: &str, limit: f64| {
            KlippyError::Config(format!(
                "Option '{option}' in section '{}' must {what} {}",
                self.section,
                float_str(limit)
            ))
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(bound("have minimum of", minval));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(bound("have maximum of", maxval));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(bound("be above", above));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(bound("be below", below));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let Some(raw) = self.values.get(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        match raw.to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(Some(true)),
            "0" | "no" | "false" | "off" => Ok(Some(false)),
            _ => Err(self.unparsable(option)),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let PyValue::Dict(dict) = choices else {
            panic!("choices must be a dict");
        };
        let int_mode = matches!(dict.keys().next(), Some(PyValue::Int(_)));
        let c = match self.values.get(option) {
            None => match opts.default {
                None => return Err(self.missing(option)),
                Some(d) => d.unwrap_or(PyValue::None),
            },
            Some(raw) if int_mode => PyValue::Int(PyInt::from_i64(self.parse_int(option, raw)?)),
            Some(raw) => PyValue::Str(raw.clone()),
        };
        match dict.iter().find(|(k, _)| py_equal(k, &c)) {
            Some((_, v)) => Ok(v.clone()),
            None => Err(KlippyError::Config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                c.str(),
                self.section
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by spi_temperature")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by spi_temperature")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by spi_temperature")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by spi_temperature")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by spi_temperature")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by spi_temperature")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by spi_temperature")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options is not used by spi_temperature")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("deprecate is not used by spi_temperature")
    }
}

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG_RECORDS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_RECORDS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn log_guard() {
    install_log_sink();
    LOG_RECORDS.with(|logs| logs.borrow_mut().clear());
}

fn take_logs() -> Vec<(String, String)> {
    LOG_RECORDS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

// ---------------------------------------------------------------------------
// Scenario runner.
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum Step {
    MinMax(f64, f64),
    Callback,
    Config,
    Resp { next_clock: i64, value: i64, fault: i64 },
    Temp(i64),
    Adc(f64),
    Delta,
}

fn py_float(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x.is_infinite() {
        let text = if x > 0.0 { "float('inf')" } else { "float('-inf')" };
        text.to_string()
    } else {
        format!("{x:?}")
    }
}

fn py_str(s: &str) -> String {
    format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
}

fn step_py(step: &Step) -> String {
    match step {
        Step::MinMax(a, b) => format!("('minmax', {}, {})", py_float(*a), py_float(*b)),
        Step::Callback => "('callback',)".to_string(),
        Step::Config => "('config',)".to_string(),
        Step::Resp { next_clock, value, fault } => {
            format!("('resp', {next_clock}, {value}, {fault})")
        }
        Step::Temp(adc) => format!("('temp', {adc})"),
        Step::Adc(temp) => format!("('adc', {})", py_float(*temp)),
        Step::Delta => "('delta',)".to_string(),
    }
}

fn values_py(values: &[(&str, &str)]) -> String {
    format!(
        "{{{}}}",
        values
            .iter()
            .map(|(k, v)| format!("{}: {}", py_str(k), py_str(v)))
            .collect::<Vec<_>>()
            .join(", ")
    )
}

fn steps_py(steps: &[Step]) -> String {
    format!("[{}]", steps.iter().map(step_py).collect::<Vec<_>>().join(", "))
}

/// A constructed sensor: the heaters-facing handle and the concrete type
/// (absent when the sensor came out of a registered factory).
type Built = (Arc<dyn TemperatureSensorApi>, Option<Arc<SensorBase>>);

fn concrete(sensor: Arc<SensorBase>) -> Built {
    (sensor.clone(), Some(sensor))
}

const KINDS: [&str; 4] = ["MAX6675", "MAX31855", "MAX31856", "MAX31865"];

fn build_kind(kind: &str, config: &Config) -> KResult<Built> {
    Ok(concrete(match kind {
        "MAX6675" => SensorBase::max6675(config)?,
        "MAX31855" => SensorBase::max31855(config)?,
        "MAX31856" => SensorBase::max31856(config)?,
        "MAX31865" => SensorBase::max31865(config)?,
        other => panic!("unknown sensor kind {other}"),
    }))
}

fn exc_line(err: &KlippyError) -> String {
    format!("EXC|{}|{}", err.class_name(), err.message())
}

/// Runs `f`, turning a panic into the `EXC|Class|message` line of the
/// Python exception the port reports as `Class: message`.
fn guarded<R>(f: impl FnOnce() -> R) -> Result<R, String> {
    catch_unwind(AssertUnwindSafe(f)).map_err(|payload| {
        let text = if let Some(s) = payload.downcast_ref::<String>() {
            s.clone()
        } else if let Some(s) = payload.downcast_ref::<&str>() {
            (*s).to_string()
        } else {
            panic!("panic payload is not a string")
        };
        let (class, message) = text.split_once(": ").expect("panic text is `Class: message`");
        format!("EXC|{class}|{message}")
    })
}

fn run_rust(
    build: &dyn Fn(&Config) -> KResult<Built>,
    values: &[(&str, &str)],
    steps: &[Step],
    observe: bool,
) -> Vec<String> {
    log_guard();

    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    let mcu = FakeMcu::new("mcu");
    let aux = FakeMcu::new("aux");
    *mcu.printer.borrow_mut() = Some(printer_api.clone());
    *aux.printer.borrow_mut() = Some(printer_api.clone());
    let pins = FakePins::new();
    pins.chips.borrow_mut().insert("mcu".to_string(), mcu.clone());
    pins.chips.borrow_mut().insert("aux".to_string(), aux.clone());
    let configfile = Arc::new(FakeConfigFile { deprecated: new_log() });
    printer.add("pins", pins.clone());
    printer.add("configfile", configfile.clone());
    printer.add("mcu", mcu.clone());
    printer.add("mcu aux", aux.clone());

    let config = FakeConfig::build(printer_api, "extruder", values);
    let out = new_log();
    let built = match build(&config) {
        Ok(built) => Some(built),
        Err(err) => {
            out.borrow_mut().push(exc_line(&err));
            None
        }
    };
    if let Some((sensor, concrete)) = &built {
        if observe {
            let oid = concrete.as_ref().expect("concrete sensor").oid();
            out.borrow_mut().push(format!("oid|{oid}"));
        }
        for step in steps {
            match step {
                Step::MinMax(min_temp, max_temp) => {
                    match sensor.setup_minmax(*min_temp, *max_temp) {
                        Ok(()) => {
                            if observe {
                                let c = concrete.as_ref().expect("concrete sensor");
                                out.borrow_mut().push(format!(
                                    "minmax|{}|{}",
                                    c.min_sample_value(),
                                    c.max_sample_value()
                                ));
                            }
                        }
                        Err(err) => out.borrow_mut().push(exc_line(&err)),
                    }
                }
                Step::Callback => {
                    let sink = out.clone();
                    sensor.setup_callback(Box::new(move |read_time, temp| {
                        sink.borrow_mut()
                            .push(format!("cb|{}|{}", float_repr(read_time), float_repr(temp)));
                    }));
                }
                Step::Config => {
                    let result = mcu.run_config_cbs().and_then(|()| aux.run_config_cbs());
                    if let Err(err) = result {
                        out.borrow_mut().push(exc_line(&err));
                    }
                }
                Step::Resp { next_clock, value, fault } => {
                    let cb = mcu
                        .serial_cb
                        .cloned()
                        .or_else(|| aux.serial_cb.cloned())
                        .expect("sensor registered a serial response");
                    let mut params = IndexMap::new();
                    for (key, v) in [
                        ("oid", concrete.as_ref().expect("concrete sensor").oid()),
                        ("next_clock", *next_clock),
                        ("value", *value),
                        ("fault", *fault),
                    ] {
                        params.insert(PyValue::Str(key.to_string()), PyValue::Int(PyInt::from_i64(v)));
                    }
                    if let Err(line) = guarded(|| cb(PyValue::Dict(params))) {
                        out.borrow_mut().push(line);
                    }
                }
                Step::Temp(adc) => {
                    let c = concrete.as_ref().expect("concrete sensor");
                    let line = match c.calc_temp(*adc) {
                        Ok(temp) => format!("temp|{}", float_repr(temp)),
                        Err(err) => exc_line(&err),
                    };
                    out.borrow_mut().push(line);
                }
                Step::Adc(temp) => {
                    let c = concrete.as_ref().expect("concrete sensor");
                    let line = match c.calc_adc(*temp) {
                        Ok(adc) => format!("adc|{adc}"),
                        Err(err) => exc_line(&err),
                    };
                    out.borrow_mut().push(line);
                }
                Step::Delta => {
                    out.borrow_mut()
                        .push(format!("delta|{}", float_repr(sensor.get_report_time_delta())));
                }
            }
        }
    }
    for (name, m) in [("mcu", &mcu), ("aux", &aux)] {
        for (tag, log) in [
            ("cfg", &m.config_cmds),
            ("lookup", &m.lookups),
            ("send", &m.sends),
            ("serial", &m.serial_cbs),
        ] {
            for line in log.borrow().iter() {
                out.borrow_mut().push(format!("{name}_{tag}|{line}"));
            }
        }
    }
    for line in pins.resets.borrow().iter() {
        out.borrow_mut().push(format!("reset|{line}"));
    }
    for line in pins.reserved.borrow().iter() {
        out.borrow_mut().push(format!("reserved|{line}"));
    }
    for line in configfile.deprecated.borrow().iter() {
        out.borrow_mut().push(format!("deprecated|{line}"));
    }
    for (level, message) in take_logs() {
        out.borrow_mut().push(format!("log|{level}|{message}"));
    }
    let lines = out.borrow().clone();
    lines
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("spi_temperature fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping spi_temperature oracle tests: set KLIPPY_PYTHON to a CPython \
                 3.8.2 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn str_list(lines: &[String]) -> Value {
    Value::List(lines.iter().map(|s| Value::str(s)).collect())
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.spi_temperature".to_string(),
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

/// Runs one scenario on both sides and compares the traces.
fn compare(
    py: &mut pyoracle::Oracle,
    label: &str,
    kind: &str,
    values: &[(&str, &str)],
    steps: &[Step],
) {
    let expected = py.eval(&format!(
        "run(spit.{kind}, {}, {})",
        values_py(values),
        steps_py(steps)
    ));
    let actual = run_rust(&|c| build_kind(kind, c), values, steps, true);
    check(&format!("{label} [{kind}]"), expected, str_list(&actual));
}

// ---------------------------------------------------------------------------
// Sample data.
// ---------------------------------------------------------------------------

/// Raw `value` fields: sign bits and field boundaries of every chip.
fn raw_samples() -> Vec<i64> {
    vec![
        -32, -1, 0, 1, 2, 3, 7, 31, 32, 33, 100, 255, 256, 0x7FFE, 0x7FFF, 0x8000, 0xFFFE,
        0xFFFF, 0x10000, 0x1FFF, 0x2000, 0x3FFF, 0x4000, 0x7FFFF, 0x80000, 0xFFFFF, 0x1FFFFF,
        0x3FFFFF, 0x7FFFFFF, 0x12345678, 0x7FFFFFFF, 0x80000000, 0xFFFFFFE0, 0xFFFFFFFF,
        0x40000 << 5, (0x40000 << 5) | 0x1F, (0x3FFFF << 5) | 0x1F, 0x1FFF << 18, 0x2000 << 18,
        (0x2000 << 18) | 0x3FFFF, 0x1FFF << 3, 0x2000 << 3, (0x2000 << 3) | 7, 0x1FFFF << 1,
        0x12D0 << 3, 0x3E8 << 18,
    ]
}

fn temp_samples() -> Vec<f64> {
    vec![
        -1e300, -300.0, -273.15, -50.0, -1.0, -0.001953125, -0.0, 0.0, 0.1, 0.5, 1.0, 20.5,
        25.0, 25.3, 99.99, 100.0, 150.5, 250.0, 300.0, 300.25, 400.0, 1000.0, 1023.75, 1024.0,
        1768.3, 1768.30001, 2000.0, 2047.96875, 2048.0, 4095.75, 32768.0, 2.0e5, 1e10, 1e300,
        1.7976931348623157e308, 5e-324, f64::NAN, f64::INFINITY, f64::NEG_INFINITY,
    ]
}

fn minmax_pairs() -> Vec<(f64, f64)> {
    vec![
        (0.0, 100.0),
        (100.0, 0.0),
        (25.0, 25.0),
        (-300.0, 2000.0),
        (20.5, 300.25),
        (0.0, 1e30),
        (-1e30, 1e30),
        (1768.3, 1768.3),
        (-273.15, 1768.4),
        (10.0, 4096.0),
        (-0.0, 0.0),
    ]
}

fn conversion_steps() -> Vec<Step> {
    let mut steps = vec![Step::Delta];
    for adc in raw_samples() {
        steps.push(Step::Temp(adc));
    }
    for temp in temp_samples() {
        steps.push(Step::Adc(temp));
    }
    for (a, b) in minmax_pairs() {
        steps.push(Step::MinMax(a, b));
    }
    steps
}

fn response_steps() -> Vec<Step> {
    let mut steps = vec![Step::Delta, Step::MinMax(10.0, 300.0), Step::Callback, Step::Config];
    for (i, value) in raw_samples().into_iter().enumerate() {
        let next_clock = [0, 123456, 1000000, 0xFFFFFFFF][i % 4];
        steps.push(Step::Resp { next_clock, value, fault: 0 });
    }
    steps
}

const DEFAULT_PIN: &[(&str, &str)] = &[("sensor_pin", "PA4")];

fn with_pin<'a>(extra: &[(&'a str, &'a str)]) -> Vec<(&'a str, &'a str)> {
    let mut values: Vec<(&str, &str)> = DEFAULT_PIN.to_vec();
    values.extend_from_slice(extra);
    values
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_entry_points() {
    let f = module_factory();
    assert!(f.load_config.is_some());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
    assert_eq!(REPORT_TIME.to_bits(), 0.300_f64.to_bits());
}

#[test]
fn load_config_registers_sensor_factories() {
    let Some(mut py) = oracle() else { return };
    py.exec("load_world()").expect("py load_config");
    let expected_names = py.eval("load_out").expect("py load_out");

    // Rust: load_config through the module factory.
    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    let heaters = Arc::new(FakeHeaters { factories: GCell::new(Vec::new()) });
    printer.add("heaters", heaters.clone());
    let config = FakeConfig::build(printer_api, "spi_temperature", &[]);
    let load_config = module_factory().load_config.expect("load_config entry point");
    let ret = load_config(&config).expect("rust load_config");
    let mut actual = vec![format!("ret|{}", if ret.is_none() { "None" } else { "object" })];
    for (name, _) in heaters.factories.borrow().iter() {
        actual.push(format!("factory|{name}"));
    }
    check("load_config registrations", Ok(expected_names), str_list(&actual));

    // `sensors()` lists the same names in the same order.
    let table: Vec<String> = sensors().iter().map(|(n, _)| (*n).to_string()).collect();
    assert_eq!(table, KINDS.map(String::from).to_vec());

    // Registered factories build the same sensors as the Python classes.
    let registered: Vec<(String, SensorFactory)> = heaters.factories.borrow().clone();
    let steps = [Step::MinMax(5.0, 250.0), Step::Callback, Step::Config, Step::Delta];
    let option_sets: [&[(&str, &str)]; 3] = [
        &[("sensor_pin", "PA4")],
        &[("sensor_pin", "aux:PB1"), ("spi_speed", "1000000"), ("spi_bus", "spi2")],
        &[
            ("sensor_pin", "PA4"),
            ("tc_type", "T"),
            ("tc_averaging_count", "16"),
            ("rtd_num_of_wires", "3"),
            ("rtd_nominal_r", "1000"),
        ],
    ];
    for (name, factory) in &registered {
        for values in option_sets {
            let expected = py.eval(&format!(
                "(load_world(), run(loaded[{}], {}, {}, False))[1]",
                py_str(name),
                values_py(values),
                steps_py(&steps)
            ));
            let factory = factory.clone();
            let actual = run_rust(
                &|c| Ok((factory(c)?, None)),
                values,
                &steps,
                false,
            );
            check(&format!("registered factory {name} {values:?}"), expected, str_list(&actual));
        }
    }
}

#[test]
fn spi_options_for_every_chip() {
    let Some(mut py) = oracle() else { return };
    let spi_sets: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("missing sensor_pin", vec![]),
        ("default", with_pin(&[])),
        ("aux mcu pin", vec![("sensor_pin", "aux:PB1")]),
        ("unknown chip", vec![("sensor_pin", "bogus:PB1")]),
        ("pin None", vec![("sensor_pin", "None")]),
        ("spi_speed", with_pin(&[("spi_speed", "1000000")])),
        ("spi_speed minimum", with_pin(&[("spi_speed", "99999")])),
        ("spi_speed bad", with_pin(&[("spi_speed", "fast")])),
        ("spi_bus", with_pin(&[("spi_bus", "spi1")])),
        (
            "software spi",
            with_pin(&[
                ("spi_software_sclk_pin", "PB3"),
                ("spi_software_miso_pin", "PB1"),
                ("spi_software_mosi_pin", "PB2"),
            ]),
        ),
        (
            "software spi other mcu",
            with_pin(&[
                ("spi_software_sclk_pin", "PB3"),
                ("spi_software_miso_pin", "PB1"),
                ("spi_software_mosi_pin", "aux:PB2"),
            ]),
        ),
        (
            "software spi missing pin",
            with_pin(&[("spi_software_sclk_pin", "PB3"), ("spi_software_mosi_pin", "PB2")]),
        ),
    ];
    let steps = [Step::MinMax(0.0, 250.0), Step::Config, Step::Delta];
    for kind in KINDS {
        for (label, values) in &spi_sets {
            compare(&mut py, label, kind, values, &steps);
        }
    }
}

#[test]
fn max31856_config_options() {
    let Some(mut py) = oracle() else { return };
    let steps = [Step::MinMax(0.0, 400.0), Step::Config];
    let mut sets: Vec<Vec<(&str, &str)>> = vec![with_pin(&[])];
    for v in ["true", "false", "on", "off", "1", "0", "YES", "maybe", ""] {
        sets.push(with_pin(&[("tc_use_50Hz_filter", v)]));
    }
    for v in ["B", "E", "J", "K", "N", "R", "S", "T", "X", "k", "", "K "] {
        sets.push(with_pin(&[("tc_type", v)]));
    }
    for v in ["1", "2", "4", "8", "16", "3", "0", "32", "x", "1.0"] {
        sets.push(with_pin(&[("tc_averaging_count", v)]));
    }
    sets.push(with_pin(&[
        ("tc_use_50Hz_filter", "true"),
        ("tc_type", "R"),
        ("tc_averaging_count", "8"),
    ]));
    sets.push(with_pin(&[
        ("tc_use_50Hz_filter", "no"),
        ("tc_type", "S"),
        ("tc_averaging_count", "16"),
    ]));
    // The chip is configured before the pin is read: option errors come first.
    sets.push(vec![("tc_type", "Q")]);
    sets.push(vec![("tc_use_50Hz_filter", "x"), ("tc_type", "Q")]);
    for (i, values) in sets.iter().enumerate() {
        compare(&mut py, &format!("max31856 options #{i}"), "MAX31856", values, &steps);
    }
}

#[test]
fn max31865_config_options() {
    let Some(mut py) = oracle() else { return };
    let steps = [
        Step::MinMax(0.0, 300.0),
        Step::Config,
        Step::Temp(0),
        Step::Temp(8000),
        Step::Temp(0x7FFE),
        Step::Adc(0.0),
        Step::Adc(100.0),
        Step::Adc(250.0),
    ];
    let mut sets: Vec<Vec<(&str, &str)>> = vec![with_pin(&[])];
    for v in ["100", "1000", "500.5", "0", "-5", "abc", "inf", "nan", "1e-320", "1e308", ""] {
        sets.push(with_pin(&[("rtd_nominal_r", v)]));
    }
    for v in ["430", "4300", "0", "-1", "xyz", "inf", "nan", "1e-320", "1e308"] {
        sets.push(with_pin(&[("rtd_reference_r", v)]));
    }
    for v in ["true", "false", "on", "off", "1", "0", "maybe"] {
        sets.push(with_pin(&[("rtd_use_50Hz_filter", v)]));
    }
    for v in ["2", "3", "4", "0", "-3", "x", "3.0", " 3 "] {
        sets.push(with_pin(&[("rtd_num_of_wires", v)]));
    }
    sets.push(with_pin(&[
        ("rtd_nominal_r", "1000"),
        ("rtd_reference_r", "4300"),
        ("rtd_use_50Hz_filter", "true"),
        ("rtd_num_of_wires", "3"),
    ]));
    sets.push(vec![("rtd_nominal_r", "0")]);
    sets.push(vec![("rtd_nominal_r", "10"), ("rtd_num_of_wires", "z")]);
    for (i, values) in sets.iter().enumerate() {
        compare(&mut py, &format!("max31865 options #{i}"), "MAX31865", values, &steps);
    }
}

#[test]
fn conversions_for_every_chip() {
    let Some(mut py) = oracle() else { return };
    let steps = conversion_steps();
    for kind in KINDS {
        compare(&mut py, "conversions", kind, &with_pin(&[]), &steps);
    }
    // 3-wire, scaled RTD: different resistance ratio.
    let scaled = with_pin(&[("rtd_nominal_r", "1000"), ("rtd_reference_r", "4300")]);
    compare(&mut py, "conversions scaled", "MAX31865", &scaled, &steps);
    let degenerate = with_pin(&[("rtd_nominal_r", "inf")]);
    compare(&mut py, "conversions inf nominal", "MAX31865", &degenerate, &steps);
    let nan_nominal = with_pin(&[("rtd_nominal_r", "nan")]);
    compare(&mut py, "conversions nan nominal", "MAX31865", &nan_nominal, &steps);
}

#[test]
fn temperature_responses_for_every_chip() {
    let Some(mut py) = oracle() else { return };
    let steps = response_steps();
    for kind in KINDS {
        compare(&mut py, "responses", kind, &with_pin(&[]), &steps);
    }
    compare(&mut py, "responses aux", "MAX31856", &[("sensor_pin", "aux:PB1")], &steps);
    let scaled = with_pin(&[("rtd_nominal_r", "1000"), ("rtd_reference_r", "4300")]);
    compare(&mut py, "responses scaled", "MAX31865", &scaled, &steps);
}

#[test]
fn response_without_callback_or_config() {
    let Some(mut py) = oracle() else { return };
    let resp = |next_clock, value, fault| Step::Resp { next_clock, value, fault };
    let scenarios: Vec<(&str, Vec<Step>)> = vec![
        // No callback installed: the Python handler raises TypeError.
        ("no callback", vec![Step::Config, resp(1000, 0x1234, 0)]),
        ("no callback no config", vec![resp(1000, 0x1234, 0)]),
        // The callback runs before the MCU config callbacks: report clock 0.
        (
            "no config",
            vec![Step::MinMax(0.0, 100.0), Step::Callback, resp(5000000, 0x4321, 0), resp(0, 1, 0)],
        ),
        // A fault never reaches the callback, with or without one installed.
        ("fault no callback", vec![Step::Config, resp(1000, 0x1234, 1)]),
        (
            "fault then reading",
            vec![
                Step::Config,
                Step::Callback,
                resp(1000, 0x1234, 4),
                resp(2000, 0x1234, 0),
                resp(3000, 0x1234, 0x80),
            ],
        ),
        // Callback replaced between samples.
        (
            "callback replaced",
            vec![
                Step::Config,
                Step::Callback,
                resp(1000, 0x1234, 0),
                Step::Callback,
                resp(2000, 0x2345, 0),
            ],
        ),
        // Configuring twice repeats the commands.
        ("config twice", vec![Step::MinMax(20.0, 80.0), Step::Config, Step::Config]),
    ];
    for kind in KINDS {
        for (label, steps) in &scenarios {
            compare(&mut py, label, kind, &with_pin(&[]), steps);
        }
    }
}

#[test]
fn faults_for_every_chip() {
    let Some(mut py) = oracle() else { return };
    let mut after_config = vec![Step::MinMax(0.0, 300.0), Step::Callback, Step::Config];
    let mut before_config = vec![Step::Callback];
    for fault in 1..=255 {
        after_config.push(Step::Resp { next_clock: 1000 * fault, value: 0x1234 + fault, fault });
        if fault % 17 == 1 {
            before_config.push(Step::Resp { next_clock: 7, value: 9, fault });
        }
    }
    after_config.push(Step::Resp { next_clock: 2_000_000, value: 0x4000, fault: 0 });
    before_config.push(Step::Config);
    for kind in KINDS {
        compare(&mut py, "all fault bytes", kind, &with_pin(&[]), &after_config);
        compare(&mut py, "faults before config", kind, &with_pin(&[]), &before_config);
    }
    // The 3-wire RTD resends its own configuration register.
    let three_wire = with_pin(&[("rtd_num_of_wires", "3"), ("rtd_use_50Hz_filter", "1")]);
    compare(&mut py, "rtd fault resend", "MAX31865", &three_wire, &after_config);
}

#[test]
fn setup_minmax_errors() {
    let Some(mut py) = oracle() else { return };
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    let pairs = [
        (nan, 100.0),
        (0.0, nan),
        (inf, 100.0),
        (0.0, inf),
        (-inf, 100.0),
        (0.0, -inf),
        (1e300, 1.7976931348623157e308),
        (0.0, 100.0),
    ];
    let mut steps = Vec::new();
    for (a, b) in pairs {
        steps.push(Step::MinMax(a, b));
        steps.push(Step::Config);
    }
    for kind in KINDS {
        compare(&mut py, "minmax errors", kind, &with_pin(&[]), &steps);
    }
    let degenerate = with_pin(&[("rtd_nominal_r", "inf")]);
    compare(&mut py, "minmax zero division", "MAX31865", &degenerate, &steps);
}
