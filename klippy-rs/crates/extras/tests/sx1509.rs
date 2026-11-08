//! Parity tests for `extras::sx1509` (port of `klippy/extras/sx1509.py`).

use extras::bus::McuI2cApi;
use extras::sx1509::{SX1509, Sx1509DigitalOut, Sx1509Pwm};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, Chip, CommandQueue, CommandWrapper, CommandQueryWrapper, ConfigCallback,
    DigitalOutPinApi, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin,
    PinParams, PinResolver, PinsApi, PwmPinApi, QuerySendOpts, SendOpts, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import configparser
import extras.sx1509 as sx1509
import extras.bus as bus
import pins

class FakeReactor:
    def monotonic(self):
        return 100.0

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self._pins = None
        self._handlers = {}
        self._reactor = FakeReactor()
    def lookup_object(self, name, default=None):
        if name == 'pins':
            return self._pins
        return default
    def register_event_handler(self, event, cb):
        self._handlers.setdefault(event, []).append(cb)
    def get_reactor(self):
        return self._reactor

class FakeMcu:
    def __init__(self, printer):
        self._printer = printer
        self.callbacks = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'mcu'
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def estimated_print_time(self, curtime):
        return 500.0
    def print_time_to_clock(self, pt):
        return int(pt * 10)

class FakeI2C:
    def __init__(self, mcu):
        self.mcu = mcu
        self.writes = []
        self.noacks = []
    def get_mcu(self):
        return self.mcu
    def get_oid(self):
        return 7
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        self.writes.append((list(data), minclock, reqclock))
    def i2c_write_noack(self, data, minclock=0, reqclock=0):
        self.noacks.append((list(data), minclock, reqclock))

class FakePins:
    error = pins.error
    def __init__(self):
        self.chips = {}
    def register_chip(self, chip_name, chip):
        self.chips[chip_name.strip()] = chip

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name):
        self._printer = printer
        self._name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name

_fresh_holder = {}
def make_i2c_from_config(config, default_speed=100000, async_write_only=False):
    return _fresh_holder['i2c']
bus.MCU_I2C_from_config = make_i2c_from_config

def fresh():
    p = FakePrinter()
    pins_obj = FakePins()
    p._pins = pins_obj
    m = FakeMcu(p)
    i2c = FakeI2C(m)
    _fresh_holder['i2c'] = i2c
    _fresh_holder['printer'] = p
    _fresh_holder['pins'] = pins_obj
    _fresh_holder['mcu'] = m
    _fresh_holder['i2c_obj'] = i2c
    c = FakeConfig(p, 'sx1509 test')
    sx = sx1509.SX1509(c)
    _fresh_holder['sx'] = sx
    _fresh_holder['config'] = c
    return sx
"#;

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<IndexMap<String, Vec<EventCallback>>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(IndexMap::new()),
            reactor: Reactor::new(false),
        }
    }
    fn set_object(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
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
        (String::new(), "ready".to_string())
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
        let found = self.objects.borrow().get(name).cloned();
        if let Some(o) = found {
            return Ok(Some(o));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => Err(KlippyError::config(format!("Printer object '{name}' not loaded"))),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().entry(event.to_string()).or_insert_with(Vec::new).push(callback);
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
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
        panic!("getsection is not used by sx1509")
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
    callbacks: GCell<Vec<ConfigCallback>>,
}

impl FakeMcu {
    fn new(printer: Printer) -> FakeMcu {
        FakeMcu { printer, callbacks: GCell::new(Vec::new()) }
    }
    fn fire_config(&self) -> KResult<()> {
        let mut cbs = self.callbacks.borrow_mut();
        for cb in cbs.iter_mut() {
            cb()?;
        }
        Ok(())
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(7)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.callbacks.borrow_mut().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn lookup_query_command(&self, _msgformat: &str, _respformat: &str, _opts: LookupQueryCommandOpts) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(&self, _cb: klippy_api::mcu::SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("register_serial_response is not used by sx1509 test")
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
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        py_trunc_i64(print_time * 10.0)
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        500.0
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        0
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
}

fn py_trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = pycompat::text::trunc_float_to_bigint(f).0.to_string();
    digits.parse::<i64>().unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

struct FakeI2c {
    mcu: Mcu,
    writes: GCell<Vec<(Vec<i64>, i64, i64)>>,
    noacks: GCell<Vec<(Vec<i64>, i64, i64)>>,
}

impl FakeI2c {
    fn new(mcu: Mcu) -> FakeI2c {
        FakeI2c { mcu, writes: GCell::new(Vec::new()), noacks: GCell::new(Vec::new()) }
    }
    fn writes_snapshot(&self) -> Vec<(Vec<i64>, i64, i64)> {
        self.writes.borrow().clone()
    }
    fn noacks_snapshot(&self) -> Vec<(Vec<i64>, i64, i64)> {
        self.noacks.borrow().clone()
    }
}

impl McuI2cApi for FakeI2c {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_oid(&self) -> i64 {
        7
    }
    fn get_i2c_address(&self) -> i64 {
        0
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        self.writes.borrow_mut().push((data.iter().map(|b| *b as i64).collect(), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.noacks.borrow_mut().push((data.iter().map(|b| *b as i64).collect(), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn i2c_read(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn i2c_transfer(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

struct FakePins {
    chips: GCell<IndexMap<String, Chip>>,
}

impl FakePins {
    fn new() -> FakePins {
        FakePins { chips: GCell::new(IndexMap::new()) }
    }
    fn chips_snapshot(&self) -> Vec<String> {
        self.chips.borrow().keys().cloned().collect()
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::config("not used by sx1509 test"))
    }
    fn register_chip(&self, chip_name: &str, chip: Chip) -> KResult<()> {
        self.chips.borrow_mut().insert(chip_name.trim().to_string(), chip);
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("sx1509 fakes");
            Some(py)
        }
        None => {
            println!("skipping sx1509 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
            None
        }
    }
}

struct RustPair {
    #[allow(dead_code)]
    fake_printer: Arc<FakePrinter>,
    mcu_arc: Arc<FakeMcu>,
    mcu: Mcu,
    i2c_arc: Arc<FakeI2c>,
    sx: Arc<SX1509>,
    fake_pins: Arc<FakePins>,
    #[allow(dead_code)]
    config: Config,
}

fn fresh_rust() -> RustPair {
    let fake_printer = Arc::new(FakePrinter::new());
    let printer: Printer = fake_printer.clone();
    let mcu_arc = Arc::new(FakeMcu::new(printer.clone()));
    let mcu: Mcu = mcu_arc.clone();
    let i2c_arc = Arc::new(FakeI2c::new(mcu.clone()));
    let i2c: Arc<dyn McuI2cApi> = i2c_arc.clone();
    let sx = SX1509::from_parts(printer.clone(), i2c.clone(), mcu.clone(), 7);
    let fake_pins = Arc::new(FakePins::new());
    let pins_obj: ObjectRef = fake_pins.clone();
    fake_printer.set_object("pins", pins_obj);
    fake_pins.register_chip("sx1509_test", sx.clone() as Chip).expect("register chip");
    let config: Config = Arc::new(FakeConfig { printer: printer.clone(), name: "sx1509 test".to_string() });
    RustPair { fake_printer, mcu_arc, mcu, i2c_arc, sx, fake_pins, config }
}

fn fresh_py(py: &mut pyoracle::Oracle) {
    py.exec("fresh()").expect("py fresh");
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "sx1509".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(actual.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(actual.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

fn pin_params(pin: &str, invert: i64, mcu: Mcu) -> PinParams {
    PinParams { chip: mcu, chip_name: "sx1509_test".to_string(), pin: pin.to_string(), invert, pullup: 0, share_type: None }
}

fn dict_value(pairs: Vec<(i64, i64)>) -> Value {
    Value::Dict(pairs.into_iter().map(|(k, v)| (Value::int(k), Value::int(v))).collect())
}

fn i2c_list_value(entries: Vec<(Vec<i64>, i64, i64)>) -> Value {
    Value::List(
        entries
            .into_iter()
            .map(|(data, minclock, reqclock)| {
                Value::Tuple(vec![
                    Value::List(data.into_iter().map(Value::int).collect()),
                    Value::int(minclock),
                    Value::int(reqclock),
                ])
            })
            .collect(),
    )
}

#[test]
fn module_header_registers_match_python() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    let expected = py.eval("_fresh_holder['sx'].reg_dict").expect("py reg_dict");
    check("module_header reg_dict", Ok(expected), dict_value(rs.sx.reg_dict_snapshot()));
    let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict").expect("py reg_i_on_dict");
    check("module_header reg_i_on_dict", Ok(expected), dict_value(rs.sx.reg_i_on_dict_snapshot()));
    let expected = py.eval("_fresh_holder['sx'].get_oid()").expect("py get_oid");
    check("get_oid", Ok(expected), Value::int(rs.sx.get_oid()));
    let expected = py.eval("_fresh_holder['sx'].get_mcu().get_name()").expect("py get_mcu");
    check("get_mcu name", Ok(expected), Value::str(&rs.sx.get_mcu().get_name()));
    assert_eq!(rs.fake_pins.chips_snapshot(), vec!["sx1509_test".to_string()]);
}

#[test]
fn clear_set_bits_and_set_register() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    for (label, reg, bitmask) in [("clear DIR bit 0", 0x0E, 1), ("set DATA bits", 0x10, 0x55), ("clear DATA bit 0", 0x10, 1)] {
        if label.starts_with("clear") {
            py.exec(&format!("_fresh_holder['sx'].clear_bits_in_register({reg}, {bitmask})")).expect("py clear");
            rs.sx.clear_bits_in_register(reg, bitmask);
        } else {
            py.exec(&format!("_fresh_holder['sx'].set_bits_in_register({reg}, {bitmask})")).expect("py set");
            rs.sx.set_bits_in_register(reg, bitmask);
        }
        let expected = py.eval(&format!("_fresh_holder['sx'].reg_dict[{reg}]")).expect("py reg_dict read");
        let actual_val = rs.sx.reg_dict_snapshot().into_iter().find(|(k, _)| *k == reg).map(|(_, v)| v).unwrap();
        check(&format!("clear_set_bits {label}"), Ok(expected), Value::int(actual_val));
    }
    for reg in [0x2A, 0x65] {
        py.exec(&format!("_fresh_holder['sx'].set_bits_in_register({reg}, 3)")).expect("py set byte");
        rs.sx.set_bits_in_register(reg, 3);
        py.exec(&format!("_fresh_holder['sx'].clear_bits_in_register({reg}, 1)")).expect("py clear byte");
        rs.sx.clear_bits_in_register(reg, 1);
        let expected = py.eval(&format!("_fresh_holder['sx'].reg_i_on_dict[{reg}]")).expect("py i_on read");
        let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == reg).map(|(_, v)| v).unwrap();
        check(&format!("clear_set_bits byte {reg}"), Ok(expected), Value::int(actual_val));
    }
    py.exec("_fresh_holder['sx'].set_register(0x2A, -256)").expect("py set_register");
    rs.sx.set_register(0x2A, -256);
    let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict[0x2A]").expect("py i_on read");
    let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == 0x2A).map(|(_, v)| v).unwrap();
    check("clear_set_bits set_register negative", Ok(expected), Value::int(actual_val));
    py.exec("_fresh_holder['sx'].set_register(0x99, 123)").expect("py unknown set");
    rs.sx.set_register(0x99, 123);
    let expected = py.eval("0x99 in _fresh_holder['sx'].reg_dict or 0x99 in _fresh_holder['sx'].reg_i_on_dict").expect("py unknown");
    check("clear_set_bits unknown no-op", Ok(expected), Value::Bool(false));
}

#[test]
fn send_register_i2c_bytes_and_clocks() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    for (reg, ptime) in [(0x0E, 10.0), (0x2A, 20.0), (0x10, 30.5)] {
        py.exec(&format!("_fresh_holder['sx'].send_register({reg}, {ptime:?})")).expect("py send");
        rs.sx.send_register(reg, ptime);
    }
    let expected = py.eval("_fresh_holder['i2c_obj'].noacks").expect("py noacks");
    check("send_register noacks", Ok(expected), i2c_list_value(rs.i2c_arc.noacks_snapshot()));
    let expected = py.eval("_fresh_holder['sx']._last_clock").expect("py last_clock");
    check("send_register last_clock", Ok(expected), Value::int(rs.sx.last_clock()));
}

#[test]
fn handle_connect_sequence() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    py.exec("_fresh_holder['sx'].handle_connect()").expect("py handle_connect");
    rs.sx.handle_connect().expect("rust handle_connect");
    let expected = py.eval("_fresh_holder['i2c_obj'].writes").expect("py writes");
    check("handle_connect writes", Ok(expected), i2c_list_value(rs.i2c_arc.writes_snapshot()));
    let expected = py.eval("_fresh_holder['i2c_obj'].noacks").expect("py noacks");
    check("handle_connect noacks", Ok(expected), i2c_list_value(rs.i2c_arc.noacks_snapshot()));
}

#[test]
fn setup_pin_routing_and_errors() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    py.exec("_fresh_holder['sx'].setup_pin('digital_out', {'pin': 'PIN_3', 'invert': 0})").expect("py digital setup");
    let p = rs.sx.chip_setup_pin("digital_out", pin_params("PIN_3", 0, rs.mcu.clone())).expect("rust digital setup");
    assert!(matches!(p, Pin::DigitalOut(_)), "digital_out must route to DigitalOut");
    py.exec("_fresh_holder['sx'].setup_pin('pwm', {'pin': 'PIN_3', 'invert': 0})").expect("py pwm setup");
    let p = rs.sx.chip_setup_pin("pwm", pin_params("PIN_3", 0, rs.mcu.clone())).expect("rust pwm setup");
    assert!(matches!(p, Pin::Pwm(_)), "pwm must route to Pwm");
    let expected = py.eval("_fresh_holder['sx'].setup_pin('digital_in', {'pin': 'PIN_3', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("digital_in", pin_params("PIN_3", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin wrong type", expected, &err);
    let expected = py.eval("_fresh_holder['sx'].setup_pin('digital_out', {'pin': 'ABC_3', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("digital_out", pin_params("ABC_3", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin wrong prefix", expected, &err);
    let expected = py.eval("_fresh_holder['sx'].setup_pin('digital_out', {'pin': 'PI', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("digital_out", pin_params("PI", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin short pin", expected, &err);
    let expected = py.eval("_fresh_holder['sx'].setup_pin('digital_out', {'pin': 'PIN_', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("digital_out", pin_params("PIN_", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin empty number", expected, &err);
    let expected = py.eval("_fresh_holder['sx'].setup_pin('pwm', {'pin': 'PIN_x', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("pwm", pin_params("PIN_x", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin non-numeric", expected, &err);
    let expected = py.eval("_fresh_holder['sx'].setup_pin('pwm', {'pin': 'PIN_16', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("pwm", pin_params("PIN_16", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin pwm out of range", expected, &err);
    let expected = py.eval("_fresh_holder['sx'].setup_pin('digital_out', {'pin': 'PIN_-1', 'invert': 0})");
    let err = rs.sx.chip_setup_pin("digital_out", pin_params("PIN_-1", 0, rs.mcu.clone())).err().expect("rust must fail");
    check_err("setup_pin negative pin", expected, &err);
}

#[test]
fn digital_out_behaviour() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    py.exec("_fresh_holder['dout'] = _fresh_holder['sx'].setup_pin('digital_out', {'pin': 'PIN_3', 'invert': 0})").expect("py dout");
    let dout_conc: Arc<Sx1509DigitalOut> = Arc::new(Sx1509DigitalOut::new(rs.sx.clone(), pin_params("PIN_3", 0, rs.mcu.clone())).expect("dout new"));
    let dout: Arc<dyn DigitalOutPinApi> = dout_conc.clone();
    let expected = py.eval("_fresh_holder['dout']._sxpin").expect("py sxpin");
    check("dout sxpin", Ok(expected), Value::int(dout_conc.sxpin()));
    let expected = py.eval("_fresh_holder['sx'].reg_dict[0x0E]").expect("py DIR");
    check("dout DIR", Ok(expected), Value::int(rs.sx.reg_dict_snapshot().into_iter().find(|(k, _)| *k == 0x0E).unwrap().1));
    let expected = py.eval("_fresh_holder['dout']._max_duration").expect("py max_duration");
    check("dout max_duration", Ok(expected), Value::float(dout_conc.max_duration()));
    let expected = py.eval("_fresh_holder['dout']._start_value").expect("py start");
    check("dout start", Ok(expected), Value::int(dout_conc.start_value()));
    let expected = py.eval("_fresh_holder['dout']._build_config()");
    let rust_err = rs.mcu_arc.fire_config().err().expect("rust dout _build_config must fail");
    match expected {
        Err(py_err) => {
            assert_eq!(rust_err.class_name(), py_err.class_name.as_str(), "dout _build_config class");
            assert_eq!(rust_err.message(), py_err.message.as_str(), "dout _build_config message");
        }
        Ok(v) => panic!("py dout _build_config unexpectedly ok {v:?}"),
    }
    py.exec("_fresh_holder['dout'].setup_max_duration(0)").expect("py clear max");
    dout.setup_max_duration(0.0);
    py.exec("for cb in _fresh_holder['mcu'].callbacks: cb()").expect("py config ok");
    rs.mcu_arc.fire_config().expect("rust config ok after clearing");
    py.exec("_fresh_holder['dout'].setup_start_value(True, True)").expect("py start True");
    dout.setup_start_value(true, true);
    let expected = py.eval("_fresh_holder['sx'].reg_dict[0x10]").expect("py REG_DATA");
    check("dout setup_start_value REG_DATA", Ok(expected), Value::int(rs.sx.reg_dict_snapshot().into_iter().find(|(k, _)| *k == 0x10).unwrap().1));
    let expected = py.eval("_fresh_holder['dout']._shutdown_value").expect("py shutdown");
    check("dout shutdown ignores arg", Ok(expected), Value::int(dout_conc.shutdown_value()));
    py.exec("_fresh_holder['dout_inv'] = _fresh_holder['sx'].setup_pin('digital_out', {'pin': 'PIN_4', 'invert': 1})").expect("py dout inv");
    let dout_inv_conc = Arc::new(Sx1509DigitalOut::new(rs.sx.clone(), pin_params("PIN_4", 1, rs.mcu.clone())).expect("dout inv new"));
    let dout_inv: Arc<dyn DigitalOutPinApi> = dout_inv_conc.clone();
    py.exec("_fresh_holder['dout_inv'].setup_start_value(True, False)").expect("py inv start");
    dout_inv.setup_start_value(true, false);
    let expected = py.eval("_fresh_holder['sx'].reg_dict[0x10]").expect("py REG_DATA inv");
    check("dout inv REG_DATA", Ok(expected), Value::int(rs.sx.reg_dict_snapshot().into_iter().find(|(k, _)| *k == 0x10).unwrap().1));
    for (ptime, value) in [(10.0, true), (20.0, false)] {
        let py_val = if value { "True" } else { "False" };
        py.exec(&format!("_fresh_holder['dout'].set_digital({ptime:?}, {py_val})")).expect("py set_digital");
        dout.set_digital(ptime, value);
    }
    py.exec("_fresh_holder['dout'].set_pwm(30.0, 0.4)").expect("py set_pwm low");
    dout_conc.set_pwm(30.0, 0.4);
    py.exec("_fresh_holder['dout'].set_pwm(40.0, 0.5)").expect("py set_pwm high");
    dout_conc.set_pwm(40.0, 0.5);
    let expected = py.eval("_fresh_holder['i2c_obj'].noacks").expect("py noacks dout");
    check("dout noacks", Ok(expected), i2c_list_value(rs.i2c_arc.noacks_snapshot()));
}

#[test]
fn pwm_behaviour() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    let rs = fresh_rust();
    py.exec("_fresh_holder['pwm'] = _fresh_holder['sx'].setup_pin('pwm', {'pin': 'PIN_3', 'invert': 0})").expect("py pwm");
    let pwm_conc: Arc<Sx1509Pwm> = Arc::new(Sx1509Pwm::new(rs.sx.clone(), pin_params("PIN_3", 0, rs.mcu.clone())).expect("pwm new"));
    let pwm: Arc<dyn PwmPinApi> = pwm_conc.clone();
    for reg in [0x00, 0x06, 0x0E, 0x20, 0x10] {
        let expected = py.eval(&format!("_fresh_holder['sx'].reg_dict[{reg}]")).expect("py reg");
        let actual_val = rs.sx.reg_dict_snapshot().into_iter().find(|(k, _)| *k == reg).unwrap().1;
        check(&format!("pwm init reg {reg}"), Ok(expected), Value::int(actual_val));
    }
    let expected = py.eval("_fresh_holder['pwm']._i_on_reg").expect("py i_on_reg");
    check("pwm i_on_reg", Ok(expected), Value::int(pwm_conc.i_on_reg()));
    let expected = py.eval("_fresh_holder['pwm']._build_config()");
    let rust_err = rs.mcu_arc.fire_config().err().expect("rust pwm _build_config must fail");
    match expected {
        Err(py_err) => {
            assert_eq!(rust_err.class_name(), py_err.class_name.as_str(), "pwm _build_config hw class");
            assert_eq!(rust_err.message(), py_err.message.as_str(), "pwm _build_config hw message");
        }
        Ok(v) => panic!("py pwm _build_config unexpectedly ok {v:?}"),
    }
    py.exec("_fresh_holder['pwm'].setup_cycle_time(0.1, True)").expect("py hw on");
    pwm.setup_cycle_time(0.1, Some(true));
    let expected = py.eval("_fresh_holder['pwm']._build_config()");
    let rust_err = rs.mcu_arc.fire_config().err().expect("rust pwm still fails on max");
    match expected {
        Err(py_err) => {
            assert_eq!(rust_err.class_name(), py_err.class_name.as_str(), "pwm _build_config max class");
            assert_eq!(rust_err.message(), py_err.message.as_str(), "pwm _build_config max message");
        }
        Ok(v) => panic!("py pwm _build_config unexpectedly ok {v:?}"),
    }
    py.exec("_fresh_holder['pwm'].setup_max_duration(0)").expect("py clear max");
    pwm.setup_max_duration(0.0);
    py.exec("for cb in _fresh_holder['mcu'].callbacks: cb()").expect("py pwm config ok");
    rs.mcu_arc.fire_config().expect("rust pwm config ok");
    let expected = py.eval("_fresh_holder['pwm']._cycle_time").expect("py cycle");
    check("pwm cycle", Ok(expected), Value::float(pwm_conc.cycle_time()));
    let expected = py.eval("_fresh_holder['pwm']._hardware_pwm").expect("py hw");
    check("pwm hw", Ok(expected), Value::Bool(pwm_conc.hardware_pwm()));
    for ptime in [10.0, 10.25] {
        let expected = py.eval(&format!("_fresh_holder['pwm'].next_aligned_print_time({ptime:?})")).expect("py aligned");
        let actual = pwm.next_aligned_print_time(ptime, None);
        check(&format!("pwm next_aligned {ptime:?}"), Ok(expected), Value::float(actual));
    }
    for s in [0.0, 0.5, 1.0] {
        py.exec(&format!("_fresh_holder['pwm'].setup_start_value({s:?}, {s:?})")).expect("py pwm start");
        pwm.setup_start_value(s, s);
        let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict[_fresh_holder['pwm']._i_on_reg]").expect("py i_on");
        let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == pwm_conc.i_on_reg()).unwrap().1;
        check(&format!("pwm setup_start s={s:?}"), Ok(expected), Value::int(actual_val));
    }
    py.exec("_fresh_holder['pwm_inv'] = _fresh_holder['sx'].setup_pin('pwm', {'pin': 'PIN_4', 'invert': 1})").expect("py pwm inv");
    let pwm_inv_conc = Arc::new(Sx1509Pwm::new(rs.sx.clone(), pin_params("PIN_4", 1, rs.mcu.clone())).expect("pwm inv new"));
    let pwm_inv: Arc<dyn PwmPinApi> = pwm_inv_conc.clone();
    let reg = pwm_inv_conc.i_on_reg();
    py.exec("_fresh_holder['pwm_inv'].setup_start_value(0.25, 0.25)").expect("py pwm inv start");
    pwm_inv.setup_start_value(0.25, 0.25);
    let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict[_fresh_holder['pwm_inv']._i_on_reg]").expect("py pwm inv i_on");
    let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == reg).unwrap().1;
    check("pwm setup_start inv", Ok(expected), Value::int(actual_val));
    for s in [-0.5, 1.5] {
        py.exec(&format!("_fresh_holder['pwm'].setup_start_value({s:?}, {s:?})")).expect("py pwm clamp");
        pwm.setup_start_value(s, s);
        let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict[_fresh_holder['pwm']._i_on_reg]").expect("py i_on clamp");
        let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == pwm_conc.i_on_reg()).unwrap().1;
        check(&format!("pwm clamp s={s:?}"), Ok(expected), Value::int(actual_val));
    }
    for v in [0.0, 0.5, 1.0] {
        py.exec(&format!("_fresh_holder['pwm'].set_pwm(10.0, {v:?})")).expect("py set_pwm");
        pwm.set_pwm(10.0, v);
        let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict[_fresh_holder['pwm']._i_on_reg]").expect("py i_on set_pwm");
        let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == pwm_conc.i_on_reg()).unwrap().1;
        check(&format!("pwm set_pwm v={v:?}"), Ok(expected), Value::int(actual_val));
    }
    for v in [0.0, 0.5, 1.0] {
        py.exec(&format!("_fresh_holder['pwm_inv'].set_pwm(10.0, {v:?})")).expect("py inv set_pwm");
        pwm_inv.set_pwm(10.0, v);
        let expected = py.eval("_fresh_holder['sx'].reg_i_on_dict[_fresh_holder['pwm_inv']._i_on_reg]").expect("py inv i_on set_pwm");
        let actual_val = rs.sx.reg_i_on_dict_snapshot().into_iter().find(|(k, _)| *k == reg).unwrap().1;
        check(&format!("pwm inv set_pwm v={v:?}"), Ok(expected), Value::int(actual_val));
    }
}

#[test]
fn load_config_prefix_factory() {
    let Some(mut py) = oracle() else { return };
    fresh_py(&mut py);
    py.exec("factory_sx = sx1509.load_config_prefix(_fresh_holder['config'])").expect("py factory");
    let factory = extras::sx1509::module_factory();
    assert!(factory.load_config.is_none(), "sx1509 has no load_config");
    assert!(factory.load_config_prefix.is_some(), "sx1509 exposes load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "no early objects");
    assert!(factory.add_printer_objects.is_none(), "no add_printer_objects");
    let expected = py.eval("factory_sx.get_oid()").expect("py factory oid");
    let rs = fresh_rust();
    let expected_dict = py.eval("factory_sx.reg_dict").expect("py factory reg_dict");
    check("factory reg_dict", Ok(expected_dict), dict_value(rs.sx.reg_dict_snapshot()));
    check("factory oid", Ok(expected), Value::int(rs.sx.get_oid()));
}
