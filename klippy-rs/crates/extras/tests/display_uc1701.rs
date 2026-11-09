//! Parity tests for `klippy/extras/display/uc1701.py` against `pyoracle`.
//!
//! The reference is loaded with minimal Python fakes for the printer, config
//! and bus; the Rust port is driven through matching fakes of the contract
//! traits. The bus traits come from the bus owner (`extras::bus`); the
//! transport trait is the module's own `io.send` surface. Every expected
//! value comes from the oracle, never from the source. `Oracle::start()`
//! returns `None` outside the oracle image; those tests then skip.

use std::sync::{Arc, Mutex};

use extras::bus::{McuBusDigitalOutApi, McuI2cApi, McuSpiApi};
use extras::display::uc1701::*;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueryWrapper, CommandQueue, CommandWrapper,
    ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin,
    PinParams, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{GCell, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;

// ---------------------------------------------------------------------------
// Recording fake transport (`io.send`).
// ---------------------------------------------------------------------------

#[derive(Default, Debug)]
struct Recorder {
    calls: Mutex<Vec<(Vec<u8>, bool)>>,
}

impl Recorder {
    fn new() -> Recorder {
        Recorder::default()
    }
    fn calls(&self) -> Vec<(Vec<u8>, bool)> {
        self.calls.lock().unwrap().clone()
    }
    fn clear(&self) {
        self.calls.lock().unwrap().clear();
    }
}

struct FakeIo {
    rec: Arc<Recorder>,
}

impl DisplayTransport for FakeIo {
    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()> {
        self.rec
            .calls
            .lock()
            .unwrap()
            .push((cmds.to_vec(), is_data));
        Ok(())
    }
}

fn display_with(columns: usize, x_offset: usize) -> (DisplayBase, Arc<Recorder>) {
    let rec = Arc::new(Recorder::new());
    let io: Arc<dyn DisplayTransport> = Arc::new(FakeIo {
        rec: Arc::clone(&rec),
    });
    (DisplayBase::new(io, columns, x_offset), rec)
}

// ---------------------------------------------------------------------------
// Rust fakes for the printer / MCU / pins / config contracts.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: GCell::new(IndexMap::new()),
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
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(self.objects.borrow().get(name).cloned())
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
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
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[klippy_api::EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}


/// Deterministic MCU: `estimated_print_time` ignores its input so the real
/// reactor clock cannot leak into `ResetHelper` timing.
struct FakeMcu {
    printer: std::sync::OnceLock<Printer>,
}

impl FakeMcu {
    fn new() -> FakeMcu {
        FakeMcu {
            printer: std::sync::OnceLock::new(),
        }
    }
    fn bind(&self, printer: Printer) {
        if self.printer.set(printer).is_err() {
            panic!("FakeMcu bound twice");
        }
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.get().cloned().expect("FakeMcu must be bound")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by uc1701")
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
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 1000.0) as i64
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
        panic!("lookup_command is not used by uc1701")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by uc1701")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by uc1701")
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
        panic!("get_constant_float is not used by uc1701")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * 1000.0) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        1005.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by uc1701")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by uc1701")
    }
}

/// Fake `pins` object: every pin resolves on the shared fake MCU.
struct FakePins {
    mcu: Mcu,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by uc1701")
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: None,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by uc1701")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by uc1701")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used by uc1701")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

/// One fake world: printer, MCU and pins wired to each other so the real
/// bus constructors from the bus owner can run against the fakes.
struct World {
    printer: Printer,
    mcu: Mcu,
}

fn make_world() -> World {
    let printer_inner = Arc::new(FakePrinter::new());
    let printer: Printer = printer_inner.clone();
    let mcu_inner = Arc::new(FakeMcu::new());
    let mcu: Mcu = mcu_inner.clone();
    mcu_inner.bind(printer.clone());
    let pins = Arc::new(FakePins { mcu: mcu.clone() });
    let pins_obj: ObjectRef = pins;
    printer_inner.set_object("pins", pins_obj);
    let mcu_obj: ObjectRef = mcu_inner;
    printer_inner.set_object("mcu", mcu_obj);
    World { printer, mcu }
}

/// Fake config mirroring `configfile.ConfigWrapper` messages for the
/// getters the chips use (`get`, `getint`, `getboolean`), recording the
/// read order as `method:option` strings.
struct FakeConfig {
    printer: Printer,
    name: String,
    values: IndexMap<String, String>,
    reads: Mutex<Vec<String>>,
}

impl FakeConfig {
    fn with_printer(printer: Printer, values: &[(&str, &str)]) -> (Config, Arc<FakeConfig>) {
        let mut map = IndexMap::new();
        for (k, v) in values {
            map.insert(k.to_string(), v.to_string());
        }
        let cfg = Arc::new(FakeConfig {
            printer,
            name: "display".to_string(),
            values: map,
            reads: Mutex::new(Vec::new()),
        });
        (cfg.clone(), cfg)
    }

    fn reads(&self) -> Vec<String> {
        self.reads.lock().unwrap().clone()
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
        self.reads
            .lock()
            .unwrap()
            .push(format!("get:{option}"));
        match self.values.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(klippy_api::error::KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.reads
            .lock()
            .unwrap()
            .push(format!("getint:{option}"));
        match self.values.get(option) {
            None => match opts.default {
                None => Err(klippy_api::error::KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            },
            Some(raw) => {
                let v: i64 = raw.trim().parse().map_err(|_| {
                    klippy_api::error::KlippyError::Config(format!(
                        "Unable to parse option '{option}' in section '{}'",
                        self.name
                    ))
                })?;
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(klippy_api::error::KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.name
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(klippy_api::error::KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.name
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by uc1701")
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.reads
            .lock()
            .unwrap()
            .push(format!("getboolean:{option}"));
        match self.values.get(option) {
            None => match opts.default {
                None => Err(klippy_api::error::KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            },
            Some(raw) => {
                let v = match raw.trim().to_ascii_lowercase().as_str() {
                    "1" | "yes" | "true" | "on" => true,
                    "0" | "no" | "false" | "off" => false,
                    _ => {
                        return Err(klippy_api::error::KlippyError::Config(format!(
                            "Unable to parse option '{option}' in section '{}'",
                            self.name
                        )))
                    }
                };
                Ok(Some(v))
            }
        }
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: klippy_api::config::GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by uc1701")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by uc1701")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by uc1701")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by uc1701")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by uc1701")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by uc1701")
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

/// Recording fake `MCU_SPI` (bus-owner trait, `klippy/extras/bus.py`).
struct FakeSpi {
    mcu: Mcu,
    sent: Mutex<Vec<(Vec<u8>, i64, i64)>>,
}

impl FakeSpi {
    fn new(mcu: Mcu) -> FakeSpi {
        FakeSpi {
            mcu,
            sent: Mutex::new(Vec::new()),
        }
    }
}

impl McuSpiApi for FakeSpi {
    fn get_oid(&self) -> i64 {
        1
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn setup_shutdown_msg(&self, _shutdown_seq: &[u8]) -> KResult<()> {
        Ok(())
    }
    fn spi_send(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.sent
            .lock()
            .unwrap()
            .push((data.to_vec(), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn spi_transfer(&self, _data: &[u8], _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn spi_transfer_with_preface(
        &self,
        _preface_data: &[u8],
        _data: &[u8],
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

/// Recording fake `MCU_I2C` (bus-owner trait, `klippy/extras/bus.py`).
struct FakeI2c {
    mcu: Mcu,
    sent: Mutex<Vec<(Vec<u8>, i64, i64)>>,
}

impl FakeI2c {
    fn new(mcu: Mcu) -> FakeI2c {
        FakeI2c {
            mcu,
            sent: Mutex::new(Vec::new()),
        }
    }
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 {
        1
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_i2c_address(&self) -> i64 {
        60
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write_noack(&self, data: &[u8], opts: SendOpts) -> KResult<()> {
        self.sent
            .lock()
            .unwrap()
            .push((data.to_vec(), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn i2c_write(&self, _data: &[u8], _opts: QuerySendOpts) -> KResult<()> {
        Ok(())
    }
    fn i2c_read(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

/// Recording fake `MCU_bus_digital_out` (bus-owner trait, `klippy/extras/bus.py`).
struct FakePin {
    mcu: Mcu,
    sent: Mutex<Vec<(bool, i64, i64)>>,
}

impl FakePin {
    fn new(mcu: Mcu) -> FakePin {
        FakePin {
            mcu,
            sent: Mutex::new(Vec::new()),
        }
    }
}

impl McuBusDigitalOutApi for FakePin {
    fn get_oid(&self) -> i64 {
        1
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn update_digital_out(&self, value: bool, opts: SendOpts) -> KResult<()> {
        self.sent
            .lock()
            .unwrap()
            .push((value, opts.minclock, opts.reqclock));
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Oracle scaffolding: load the reference with fakes.
// ---------------------------------------------------------------------------

const ORACLE_SETUP: &str = r#"
import sys, types
sys.path.insert(0, 'klippy')
import logging
class Error(Exception):
    pass
class _RecIO(object):
    def __init__(self): self.calls = []
    def send(self, cmds, is_data=False):
        self.calls.append((list(cmds), bool(is_data)))
class _FakeReactor(object):
    def monotonic(self): return 12345.0
class _FakePrinter(object):
    def get_reactor(self): return _FakeReactor()
class _FakeMcu(object):
    def get_name(self): return 'mcu'
    def get_printer(self): return _FakePrinter()
    def estimated_print_time(self, eventtime): return 1005.0
    def print_time_to_clock(self, print_time): return int(print_time * 1000)
class _FakePin(object):
    def __init__(self, mcu, pin_desc, cmd_queue=None):
        self.mcu_obj = mcu
        self.pin_desc = pin_desc
        self.sent = []
        _fake_bus.last_pin = self
    def get_mcu(self): return self.mcu_obj
    def get_command_queue(self): return 7
    def update_digital_out(self, value, minclock=0, reqclock=0):
        self.sent.append((bool(value), minclock, reqclock))
class _FakeSpi(object):
    def __init__(self):
        self.sent = []
        self.mcu_obj = _FakeMcu()
    def get_mcu(self): return self.mcu_obj
    def get_command_queue(self): return 7
    def spi_send(self, data, minclock=0, reqclock=0):
        self.sent.append((list(data), minclock, reqclock))
class _FakeI2c(object):
    def __init__(self):
        self.sent = []
        self.mcu_obj = _FakeMcu()
    def get_mcu(self): return self.mcu_obj
    def get_command_queue(self): return 7
    def i2c_write_noack(self, data, minclock=0, reqclock=0):
        self.sent.append((list(data), minclock, reqclock))
class _FakeBus(object):
    def __init__(self):
        self.last_spi = None
        self.last_i2c = None
        self.last_pin = None
    def MCU_SPI_from_config(self, config, mode, pin_option='cs_pin', default_speed=100000, **kw):
        # Mirror `bus.py MCU_SPI_from_config` read order.
        config.get(pin_option)
        config.getint('spi_speed', default_speed)
        if config.get('spi_software_sclk_pin', None) is not None:
            for name in ['miso', 'mosi', 'sclk']:
                config.get('spi_software_%s_pin' % (name,))
        else:
            config.get('spi_bus', None)
        self.last_spi = _FakeSpi()
        return self.last_spi
    def MCU_I2C_from_config(self, config, default_addr=None, default_speed=100000, async_write_only=False):
        # Mirror `bus.py MCU_I2C_from_config` read order.
        config.get('i2c_mcu', 'mcu')
        config.getint('i2c_speed', default_speed)
        if default_addr is None:
            config.getint('i2c_address')
        else:
            config.getint('i2c_address', default_addr)
        if config.get('i2c_software_scl_pin', None) is not None:
            for name in ['scl', 'sda']:
                config.get('i2c_software_%s_pin' % (name,))
        else:
            config.get('i2c_bus', None)
        self.last_i2c = _FakeI2c()
        return self.last_i2c
    def MCU_bus_digital_out(self, mcu, pin_desc, cmd_queue=None):
        return _FakePin(mcu, pin_desc, cmd_queue)
_fake_bus = _FakeBus()
class _FakeConfig(object):
    def __init__(self, values=None):
        self._values = dict(values or {})
        self.reads = []
    def get_name(self): return 'display'
    def get_printer(self): return _FakePrinter()
    def get(self, option, default=None):
        self.reads.append('get:' + option)
        return self._values.get(option, default)
    def getint(self, option, default=None, minval=None, maxval=None):
        self.reads.append('getint:' + option)
        if option not in self._values:
            if default is None:
                raise Error("Option '%s' in section 'display' must be specified" % (option,))
            return default
        try:
            v = int(self._values[option])
        except Exception:
            raise Error("Unable to parse option '%s' in section 'display'" % (option,))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section 'display' must have minimum of %s" % (option, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section 'display' must have maximum of %s" % (option, maxval))
        return v
    def getboolean(self, option, default=False):
        self.reads.append('getboolean:' + option)
        if option not in self._values:
            return default
        v = self._values[option].strip().lower()
        if v in ('1', 'yes', 'true', 'on'): return True
        if v in ('0', 'no', 'false', 'off'): return False
        raise Error("Unable to parse option '%s' in section 'display'" % (option,))
import importlib
uc1701 = importlib.import_module('extras.display.uc1701')
uc1701.bus = _fake_bus
io = _RecIO()
disp = uc1701.DisplayBase(io, 128, 0)
"#;

struct Harness {
    py: pyoracle::Oracle,
}

impl Harness {
    fn start() -> Option<Harness> {
        let mut py = pyoracle::Oracle::start()?;
        py.exec(ORACLE_SETUP).unwrap_or_else(|e| {
            panic!(
                "oracle setup failed: class={} message={}",
                e.class_name, e.message
            )
        });
        Some(Harness { py })
    }
}

fn oracle_eval(h: &mut Harness, expr: &str) -> recording::Value {
    h.py.eval(expr).unwrap_or_else(|e| {
        panic!(
            "oracle eval {expr:?} failed: class={} message={}",
            e.class_name, e.message
        )
    })
}

fn oracle_vram_flat(h: &mut Harness) -> Vec<i64> {
    match oracle_eval(h, "[list(p) for p in disp.vram]") {
        recording::Value::List(pages) => pages
            .iter()
            .flat_map(|p| match p {
                recording::Value::List(bs) => bs
                    .iter()
                    .map(|b| match b {
                        recording::Value::Int(i) => *i,
                        _ => panic!(),
                    })
                    .collect::<Vec<_>>(),
                _ => panic!(),
            })
            .collect(),
        _ => panic!(),
    }
}
fn spi_transport() -> (Arc<dyn DisplayTransport>, Arc<FakeSpi>, Arc<FakePin>) {
    let world = make_world();
    let spi = Arc::new(FakeSpi::new(world.mcu.clone()));
    let dc = Arc::new(FakePin::new(world.mcu.clone()));
    let spi_dyn: Arc<dyn McuSpiApi> = spi.clone();
    let dc_dyn: Arc<dyn McuBusDigitalOutApi> = dc.clone();
    let io: Arc<dyn DisplayTransport> = Arc::new(Spi4wire::from_parts(spi_dyn, dc_dyn));
    (io, spi, dc)
}

fn i2c_transport(world: &World) -> (Arc<dyn DisplayTransport>, Arc<FakeI2c>) {
    let i2c = Arc::new(FakeI2c::new(world.mcu.clone()));
    let i2c_dyn: Arc<dyn McuI2cApi> = i2c.clone();
    let io: Arc<dyn DisplayTransport> = Arc::new(I2cDisplay::from_parts(i2c_dyn));
    (io, i2c)
}

fn dc_value(sent: &[(bool, i64, i64)]) -> recording::Value {
    recording::Value::List(
        sent.iter()
            .map(|(v, minclock, reqclock)| {
                recording::Value::Tuple(vec![
                    recording::Value::Bool(*v),
                    recording::Value::Int(*minclock),
                    recording::Value::Int(*reqclock),
                ])
            })
            .collect(),
    )
}

fn bus_value(sent: &[(Vec<u8>, i64, i64)]) -> recording::Value {
    recording::Value::List(
        sent.iter()
            .map(|(data, minclock, reqclock)| {
                recording::Value::Tuple(vec![
                    recording::Value::List(
                        data.iter()
                            .map(|b| recording::Value::Int(*b as i64))
                            .collect(),
                    ),
                    recording::Value::Int(*minclock),
                    recording::Value::Int(*reqclock),
                ])
            })
            .collect(),
    )
}

fn vram_flat(vram: &[Vec<u8>]) -> Vec<i64> {
    vram.iter()
        .flat_map(|p| p.iter().map(|b| *b as i64))
        .collect()
}

fn calls_value(calls: &[(Vec<u8>, bool)]) -> recording::Value {
    recording::Value::List(
        calls
            .iter()
            .map(|(data, is_data)| {
                recording::Value::Tuple(vec![
                    recording::Value::List(
                        data.iter()
                            .map(|b| recording::Value::Int(*b as i64))
                            .collect(),
                    ),
                    recording::Value::Bool(*is_data),
                ])
            })
            .collect(),
    )
}

fn oracle_calls(h: &mut Harness) -> recording::Value {
    h.py.eval("[ (x[0], x[1]) for x in io.calls ]").unwrap()
}

/// Chip-layer config reads shared by both sides (bus-internal reads are
/// filtered out; the Python fake bus mirrors `bus.py` read order so the
/// chip-layer sequence must still align).
fn chip_layer_reads(reads: &[String]) -> Vec<String> {
    reads
        .iter()
        .filter_map(|r| {
            let option = r.split(':').nth(1)?;
            match option {
                "cs_pin" | "a0_pin" | "dc_pin" | "rst_pin" | "reset_pin" | "contrast"
                | "vcomh" | "invert" | "x_offset" => Some(r.clone()),
                _ => None,
            }
        })
        .collect()
}

fn oracle_reads(h: &mut Harness, out: &str) -> Vec<String> {
    let raw = match oracle_eval(h, out) {
        recording::Value::List(items) => items
            .iter()
            .map(|v| match v {
                recording::Value::Str(s) => s.clone(),
                recording::Value::Int(i) => i.to_string(),
                _ => panic!(),
            })
            .collect::<Vec<String>>(),
        _ => panic!(),
    };
    chip_layer_reads(&raw)
}

#[test]
fn dimensions_and_initial_vram_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    let (rust, _rec) = display_with(128, 0);
    assert_eq!(
        oracle_eval(&mut h, "disp.get_dimensions()"),
        recording::Value::Tuple(vec![recording::Value::Int(16), recording::Value::Int(4)])
    );
    assert_eq!(oracle_vram_flat(&mut h), vram_flat(&rust.vram_snapshot()));
    assert_eq!(rust.get_dimensions(), (16, 4));
    assert_eq!(rust.columns(), 128);
    assert_eq!(rust.x_offset(), 0);
    assert_eq!(
        oracle_eval(&mut h, "disp.columns"),
        recording::Value::Int(128)
    );
}

#[test]
fn write_text_flush_and_clear_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    let (rust, rec) = display_with(128, 0);
    let code = "disp.write_text(0, 0, b'Hello'); disp.write_text(5, 1, b'World'); disp.write_text(14, 2, b'ABCDE'); disp.write_text(16, 0, b'Overflow'); disp.write_text(0, 3, b'OK')";
    h.py.exec(code)
        .unwrap_or_else(|e| panic!("setup/write failed: {}", e.message));
    rust.write_text(0, 0, b"Hello");
    rust.write_text(5, 1, b"World");
    rust.write_text(14, 2, b"ABCDE");
    rust.write_text(16, 0, b"Overflow");
    rust.write_text(0, 3, b"OK");
    assert_eq!(
        vram_flat(&rust.vram_snapshot()),
        oracle_vram_flat(&mut h),
        "vram after write_text"
    );

    h.py.exec("io.calls = []").unwrap();
    rec.clear();
    h.py.exec("disp.flush()").unwrap();
    rust.flush().unwrap();
    assert_eq!(
        calls_value(&rec.calls()),
        oracle_calls(&mut h),
        "flush send stream"
    );

    // Flush with no changes sends nothing.
    rec.clear();
    h.py.exec("io.calls = []").unwrap();
    h.py.exec("disp.flush()").unwrap();
    rust.flush().unwrap();
    assert!(rec.calls().is_empty(), "empty flush must not send");
    assert_eq!(
        oracle_eval(&mut h, "len(io.calls)"),
        recording::Value::Int(0)
    );

    // `clear` + flush emits the same full stream on both sides.
    h.py.exec("disp.clear(); io.calls = []").unwrap();
    rec.clear();
    h.py.exec("disp.flush()").unwrap();
    rust.clear();
    rust.flush().unwrap();
    assert_eq!(
        calls_value(&rec.calls()),
        oracle_calls(&mut h),
        "clear+flush send stream"
    );
    assert_eq!(
        vram_flat(&rust.vram_snapshot()),
        oracle_vram_flat(&mut h),
        "vram after clear"
    );
}

#[test]
fn swizzle_graphics_and_glyph_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    let (rust, _rec) = display_with(132, 3);
    h.py.exec("disp = __import__('importlib').import_module('extras.display.uc1701').DisplayBase(io, 132, 3)").unwrap();
    // `_swizzle_bits` on a known input, compared directly.
    h.py.exec(
        "sw_top, sw_bot = disp._swizzle_bits(bytes([0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81]*2))",
    )
    .unwrap();
    let (rust_top, rust_bot) = DisplayBase::swizzle_bits(&[
        0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81, 0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42,
        0x81,
    ]);
    assert_eq!(
        oracle_eval(&mut h, "list(sw_top)"),
        recording::Value::List(
            rust_top
                .iter()
                .map(|b| recording::Value::Int(*b as i64))
                .collect()
        )
    );
    assert_eq!(
        oracle_eval(&mut h, "list(sw_bot)"),
        recording::Value::List(
            rust_bot
                .iter()
                .map(|b| recording::Value::Int(*b as i64))
                .collect()
        )
    );

    h.py.exec("disp.write_graphics(0, 0, bytes([255]*16))")
        .unwrap();
    rust.write_graphics(0, 0, &[255u8; 16]);
    // Bounds: x>=16, y>=4 and len!=16 are all no-ops on both sides.
    h.py.exec("disp.write_graphics(16, 0, bytes([255]*16)); disp.write_graphics(0, 4, bytes([255]*16)); disp.write_graphics(0, 0, bytes([255]*15))").unwrap();
    rust.write_graphics(16, 0, &[255u8; 16]);
    rust.write_graphics(0, 4, &[255u8; 16]);
    rust.write_graphics(0, 0, &[255u8; 15]);
    h.py.exec("disp.write_graphics(0, 0, bytes([0x81,0x42,0x24,0x18,0x18,0x24,0x42,0x81]*2))")
        .unwrap();
    rust.write_graphics(
        0,
        0,
        &[
            0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81, 0x81, 0x42, 0x24, 0x18, 0x18, 0x24,
            0x42, 0x81,
        ],
    );
    assert_eq!(
        vram_flat(&rust.vram_snapshot()),
        oracle_vram_flat(&mut h),
        "vram after write_graphics"
    );

    // `write_glyph`: text glyphs -> 1, unknown -> 0, icon at x>=15 falls
    // through to the text-glyph check (0 for an icon-only name).
    h.py.exec("a = disp.write_glyph(0, 1, 'degrees'); b = disp.write_glyph(1, 1, 'bogus'); c = disp.write_glyph(0, 1, 'right_arrow')").unwrap();
    assert_eq!(oracle_eval(&mut h, "a"), recording::Value::Int(1));
    assert_eq!(oracle_eval(&mut h, "b"), recording::Value::Int(0));
    assert_eq!(oracle_eval(&mut h, "c"), recording::Value::Int(1));
    assert_eq!(rust.write_glyph(0, 1, "degrees"), 1);
    assert_eq!(rust.write_glyph(1, 1, "bogus"), 0);
    assert_eq!(rust.write_glyph(0, 1, "right_arrow"), 1);
    assert_eq!(
        vram_flat(&rust.vram_snapshot()),
        oracle_vram_flat(&mut h),
        "vram after text glyphs"
    );

    // `set_glyphs` with a 16x16 icon (plus a glyph without `icon16x16`),
    // then `write_glyph` draws it (return 2); the same icon at x=15 -> 0.
    h.py.exec("disp.set_glyphs({'myicon': {'icon16x16': ([i for i in range(16)], [255 - i for i in range(16)])}, 'plain': {}}); r1 = disp.write_glyph(2, 2, 'myicon'); r2 = disp.write_glyph(15, 2, 'myicon')").unwrap();
    let mut glyphs = IndexMap::new();
    glyphs.insert(
        "myicon".to_string(),
        Some((
            (0u8..16).collect(),
            (0u8..16).map(|i| 255u8.wrapping_sub(i)).collect(),
        )),
    );
    glyphs.insert("plain".to_string(), None);
    rust.set_glyphs(&glyphs);
    assert_eq!(oracle_eval(&mut h, "r1"), recording::Value::Int(2));
    assert_eq!(oracle_eval(&mut h, "r2"), recording::Value::Int(0));
    assert_eq!(rust.write_glyph(2, 2, "myicon"), 2);
    assert_eq!(rust.write_glyph(15, 2, "myicon"), 0);
    assert_eq!(
        vram_flat(&rust.vram_snapshot()),
        oracle_vram_flat(&mut h),
        "vram after icon glyph"
    );
}

#[test]
fn spi4wire_and_i2c_send_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    // Python `SPI4wire.send`: DC toggles before the SPI send, both with
    // reqclock=BACKGROUND_PRIORITY_CLOCK.
    h.py.exec("cfg_spi = _FakeConfig({'cs_pin': 'PA0', 'a0_pin': 'PA1'}); spi_obj = uc1701.SPI4wire(cfg_spi, 'a0_pin'); dc_pin = _fake_bus.last_pin; spi_obj.send([0x01, 0x02]); spi_obj.send([0x03], is_data=True)").unwrap();
    let oracle_dc = oracle_eval(&mut h, "[(c[0], c[1], c[2]) for c in dc_pin.sent]");
    let oracle_spi = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_spi.sent]",
    );
    let world = make_world();
    let spi = Arc::new(FakeSpi::new(world.mcu.clone()));
    let dc = Arc::new(FakePin::new(world.mcu.clone()));
    let spi_dyn: Arc<dyn McuSpiApi> = spi.clone();
    let dc_dyn: Arc<dyn McuBusDigitalOutApi> = dc.clone();
    let rust_spi = Spi4wire::from_parts(spi_dyn, dc_dyn);
    rust_spi.send(&[0x01, 0x02], false).unwrap();
    rust_spi.send(&[0x03], true).unwrap();
    let rust_dc: Vec<recording::Value> = dc
        .sent
        .lock()
        .unwrap()
        .iter()
        .map(|(v, minclock, reqclock)| {
            recording::Value::Tuple(vec![
                recording::Value::Bool(*v),
                recording::Value::Int(*minclock),
                recording::Value::Int(*reqclock),
            ])
        })
        .collect();
    assert_eq!(
        recording::Value::List(rust_dc),
        oracle_dc,
        "SPI4wire DC toggles"
    );
    // SPI payloads are checked on the oracle side shape and on Rust via a
    // directly-held fake below (same calls, fresh objects).
    let spi2 = Arc::new(FakeSpi::new(world.mcu.clone()));
    let dc2 = Arc::new(FakePin::new(world.mcu.clone()));
    let spi2_dyn: Arc<dyn McuSpiApi> = spi2.clone();
    let dc2_dyn: Arc<dyn McuBusDigitalOutApi> = dc2.clone();
    let rust_spi2 = Spi4wire::from_parts(spi2_dyn, dc2_dyn);
    rust_spi2.send(&[0x01, 0x02], false).unwrap();
    rust_spi2.send(&[0x03], true).unwrap();
    let rust_spi_vals: Vec<recording::Value> = spi2
        .sent
        .lock()
        .unwrap()
        .iter()
        .map(|(data, minclock, reqclock)| {
            recording::Value::Tuple(vec![
                recording::Value::List(
                    data.iter()
                        .map(|b| recording::Value::Int(*b as i64))
                        .collect(),
                ),
                recording::Value::Int(*minclock),
                recording::Value::Int(*reqclock),
            ])
        })
        .collect();
    assert_eq!(
        recording::Value::List(rust_spi_vals),
        oracle_spi,
        "SPI4wire spi_send calls"
    );
    assert_eq!(BACKGROUND_PRIORITY_CLOCK, 0x7fff_ffff_0000_0000i64);

    // Python `I2C.send`: 0x40 prefix for data, 0x00 for commands.
    h.py.exec("cfg_i2c = _FakeConfig({}); i2c_obj = uc1701.I2C(cfg_i2c, 60); i2c_obj.send([0x01, 0x02], is_data=True); i2c_obj.send([0x03])").unwrap();
    let oracle_i2c = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_i2c.sent]",
    );
    let i2c = Arc::new(FakeI2c::new(world.mcu.clone()));
    let i2c_dyn: Arc<dyn McuI2cApi> = i2c.clone();
    let rust_i2c = I2cDisplay::from_parts(i2c_dyn);
    rust_i2c.send(&[0x01, 0x02], true).unwrap();
    rust_i2c.send(&[0x03], false).unwrap();
    let rust_i2c_vals: Vec<recording::Value> = i2c
        .sent
        .lock()
        .unwrap()
        .iter()
        .map(|(data, minclock, reqclock)| {
            recording::Value::Tuple(vec![
                recording::Value::List(
                    data.iter()
                        .map(|b| recording::Value::Int(*b as i64))
                        .collect(),
                ),
                recording::Value::Int(*minclock),
                recording::Value::Int(*reqclock),
            ])
        })
        .collect();
    assert_eq!(
        recording::Value::List(rust_i2c_vals),
        oracle_i2c,
        "I2C i2c_write_noack calls"
    );
}

#[test]
fn reset_helper_init_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    // With a pin: toggle 0, 1, 1 at print_time + .100/.200/.300.
    h.py.exec("rh = uc1701.ResetHelper('rst_pin', _fake_bus.last_spi if _fake_bus.last_spi is not None else _FakeSpi()); rh.init()").unwrap();
    // The Python ResetHelper above built its pin through the fake bus
    // (`MCU_bus_digital_out`); read what that pin recorded.
    let oracle_pin_sent = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_pin.sent]",
    );
    let world = make_world();
    let pin = Arc::new(FakePin::new(world.mcu.clone()));
    let pin_dyn: Arc<dyn McuBusDigitalOutApi> = pin.clone();
    let rust_rh = ResetHelper::from_parts(Some(pin_dyn));
    rust_rh.init().unwrap();
    let rust_vals: Vec<recording::Value> = pin
        .sent
        .lock()
        .unwrap()
        .iter()
        .map(|(v, minclock, reqclock)| {
            recording::Value::Tuple(vec![
                recording::Value::Bool(*v),
                recording::Value::Int(*minclock),
                recording::Value::Int(*reqclock),
            ])
        })
        .collect();
    assert_eq!(
        recording::Value::List(rust_vals),
        oracle_pin_sent,
        "ResetHelper toggle clocks"
    );

    // Without a pin: `init` is a no-op (no error, no sends).
    h.py.exec("uc1701.ResetHelper(None, _FakeSpi()).init()")
        .unwrap();
    ResetHelper::from_parts(None).init().unwrap();
}

#[test]
fn uc1701_init_and_config_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    // Defaults: contrast=40. The chip's own SPI/DC fakes collect the
    // init stream (init cmds, then the initial full-frame flush).
    h.py.exec("cfg_uc = _FakeConfig({'a0_pin': 'PA1', 'cs_pin': 'PA0'}); chip = uc1701.UC1701(cfg_uc); chip.init()").unwrap();
    assert_eq!(
        oracle_eval(&mut h, "chip.contrast"),
        recording::Value::Int(40)
    );
    let oracle_dc = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_pin.sent]",
    );
    let oracle_spi = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_spi.sent]",
    );
    let world = make_world();
    let (io, spi, dc) = spi_transport();
    let rust = Uc1701::from_parts(io, 40, ResetHelper::from_parts(None));
    assert_eq!(rust.contrast(), 40);
    rust.init().unwrap();
    assert_eq!(
        dc_value(&dc.sent.lock().unwrap()),
        oracle_dc,
        "UC1701 DC stream"
    );
    assert_eq!(
        bus_value(&spi.sent.lock().unwrap()),
        oracle_spi,
        "UC1701 SPI stream"
    );
    // `from_config` picks up the same contrast default through the real
    // bus constructors.
    let (cfg, cfg_rec) = FakeConfig::with_printer(
        world.printer.clone(),
        &[("a0_pin", "PA1"), ("cs_pin", "PA0")],
    );
    let rust_cfg = Uc1701::from_config(&cfg).unwrap();
    assert_eq!(rust_cfg.contrast(), 40);
    assert_eq!(
        oracle_reads(&mut h, "cfg_uc.reads"),
        chip_layer_reads(&cfg_rec.reads())
    );

    // Custom contrast flows into the init bytes.
    h.py.exec("cfg_uc2 = _FakeConfig({'a0_pin': 'PA1', 'cs_pin': 'PA0', 'contrast': '7'}); chip2 = uc1701.UC1701(cfg_uc2); chip2.init()").unwrap();
    let oracle_dc2 = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_pin.sent]",
    );
    let oracle_spi2 = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_spi.sent]",
    );
    let (io2, spi2, dc2) = spi_transport();
    let rust2 = Uc1701::from_parts(io2, 7, ResetHelper::from_parts(None));
    assert_eq!(rust2.contrast(), 7);
    rust2.init().unwrap();
    assert_eq!(
        dc_value(&dc2.sent.lock().unwrap()),
        oracle_dc2,
        "UC1701 DC stream contrast=7"
    );
    assert_eq!(
        bus_value(&spi2.sent.lock().unwrap()),
        oracle_spi2,
        "UC1701 SPI stream contrast=7"
    );

    // Out-of-range contrast: same class and message on both sides.
    let py_res = h
        .py
        .exec("uc1701.UC1701(_FakeConfig({'a0_pin': 'PA1', 'cs_pin': 'PA0', 'contrast': '64'}))");
    let (cfg_bad, _bad_rec) = FakeConfig::with_printer(
        world.printer.clone(),
        &[("a0_pin", "PA1"), ("cs_pin", "PA0"), ("contrast", "64")],
    );
    let rust_res = Uc1701::from_config(&cfg_bad);
    match (py_res, rust_res) {
        (Err(py_err), Err(rust_err)) => {
            assert_eq!(
                rust_err.class_name(),
                py_err.class_name.as_str(),
                "contrast error class"
            );
            assert_eq!(
                rust_err.message(),
                py_err.message.as_str(),
                "contrast error message"
            );
        }
        _ => panic!("contrast=64 must fail on both sides"),
    }
}

#[test]
fn ssd1306_init_and_config_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    let world = make_world();
    for (label, invert_py, invert_cfg) in [("normal", "False", false), ("invert", "True", true)] {
        h.py.exec(&format!(
            "cfg_ssd = _FakeConfig({{'cs_pin': 'PA0', 'dc_pin': 'PA1', 'invert': '{invert_py}'}}); chip = uc1701.SSD1306(cfg_ssd); chip.init()"
        ))
        .unwrap_or_else(|e| panic!("{label}: py SSD1306 failed: {}", e.message));
        assert_eq!(
            oracle_eval(&mut h, "chip.contrast"),
            recording::Value::Int(239),
            "{label} contrast default"
        );
        assert_eq!(
            oracle_eval(&mut h, "chip.vcomh"),
            recording::Value::Int(0),
            "{label} vcomh default"
        );
        assert_eq!(
            oracle_eval(&mut h, "chip.invert"),
            recording::Value::Bool(invert_cfg),
            "{label} invert"
        );
        let oracle_dc = oracle_eval(
            &mut h,
            "[(c[0], c[1], c[2]) for c in _fake_bus.last_pin.sent]",
        );
        let oracle_spi = oracle_eval(
            &mut h,
            "[(c[0], c[1], c[2]) for c in _fake_bus.last_spi.sent]",
        );
        let (io, spi, dc) = spi_transport();
        let rust = Ssd1306::from_parts(
            io,
            128,
            0,
            239,
            0,
            invert_cfg,
            ResetHelper::from_parts(None),
        );
        assert_eq!(rust.contrast(), 239, "{label} rust contrast");
        assert_eq!(rust.vcomh(), 0, "{label} rust vcomh");
        assert_eq!(rust.invert(), invert_cfg, "{label} rust invert");
        rust.init().unwrap();
        assert_eq!(
            dc_value(&dc.sent.lock().unwrap()),
            oracle_dc,
            "{label} SSD1306 DC stream"
        );
        assert_eq!(
            bus_value(&spi.sent.lock().unwrap()),
            oracle_spi,
            "{label} SSD1306 SPI stream"
        );
        // The real `from_config` resolves the same attributes, and its
        // cs_pin selection reads match the reference's.
        let (cfg, cfg_rec) = FakeConfig::with_printer(
            world.printer.clone(),
            &[
                ("cs_pin", "PA0"),
                ("dc_pin", "PA1"),
                ("invert", if invert_cfg { "true" } else { "false" }),
            ],
        );
        let rust_cfg = Ssd1306::from_config(&cfg, 128, 0).unwrap();
        assert_eq!(rust_cfg.contrast(), 239, "{label} from_config contrast");
        assert_eq!(rust_cfg.vcomh(), 0, "{label} from_config vcomh");
        assert_eq!(rust_cfg.invert(), invert_cfg, "{label} from_config invert");
        assert_eq!(
            oracle_reads(&mut h, "cfg_ssd.reads"),
            chip_layer_reads(&cfg_rec.reads()),
            "{label} config read order"
        );
    }

    // I2C path (no cs_pin): init stream must match as well.
    h.py.exec(
        "cfg_i2c_ssd = _FakeConfig({}); chip_i2c = uc1701.SSD1306(cfg_i2c_ssd); chip_i2c.init()",
    )
    .unwrap();
    let oracle_i2c = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_i2c.sent]",
    );
    let (io, i2c) = i2c_transport(&world);
    let rust = Ssd1306::from_parts(
        io,
        128,
        0,
        239,
        0,
        false,
        ResetHelper::from_parts(None),
    );
    rust.init().unwrap();
    assert_eq!(
        bus_value(&i2c.sent.lock().unwrap()),
        oracle_i2c,
        "SSD1306 I2C init stream"
    );
    let (cfg, cfg_rec) = FakeConfig::with_printer(world.printer.clone(), &[]);
    let rust_cfg = Ssd1306::from_config(&cfg, 128, 0).unwrap();
    assert_eq!(rust_cfg.contrast(), 239, "from_config I2C contrast");
    assert_eq!(
        oracle_reads(&mut h, "cfg_i2c_ssd.reads"),
        chip_layer_reads(&cfg_rec.reads()),
        "from_config I2C read order"
    );

    // Out-of-range vcomh: same class and message.
    let py_res =
        h.py.exec("uc1701.SSD1306(_FakeConfig({'cs_pin': 'PA0', 'dc_pin': 'PA1', 'vcomh': '64'}))");
    let (cfg_bad, _bad_rec) = FakeConfig::with_printer(
        world.printer.clone(),
        &[
            ("cs_pin", "PA0"),
            ("dc_pin", "PA1"),
            ("vcomh", "64"),
        ],
    );
    let rust_res = Ssd1306::from_config(&cfg_bad, 128, 0);
    match (py_res, rust_res) {
        (Err(py_err), Err(rust_err)) => {
            assert_eq!(
                rust_err.class_name(),
                py_err.class_name.as_str(),
                "vcomh error class"
            );
            assert_eq!(
                rust_err.message(),
                py_err.message.as_str(),
                "vcomh error message"
            );
        }
        _ => panic!("vcomh=64 must fail on both sides"),
    }
}

#[test]
fn sh1106_config_match_oracle() {
    let Some(mut h) = Harness::start() else {
        println!("skipped: pyoracle unavailable");
        return;
    };
    let world = make_world();
    h.py.exec("cfg_sh = _FakeConfig({'cs_pin': 'PA0', 'dc_pin': 'PA1', 'x_offset': '2'}); chip = uc1701.SH1106(cfg_sh); chip.init()").unwrap();
    assert_eq!(
        oracle_eval(&mut h, "chip.columns"),
        recording::Value::Int(132)
    );
    assert_eq!(
        oracle_eval(&mut h, "chip.x_offset"),
        recording::Value::Int(2)
    );
    assert_eq!(
        oracle_eval(&mut h, "len(chip.vram[0])"),
        recording::Value::Int(132)
    );
    let oracle_dc = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_pin.sent]",
    );
    let oracle_spi = oracle_eval(
        &mut h,
        "[(c[0], c[1], c[2]) for c in _fake_bus.last_spi.sent]",
    );
    let (io, spi, dc) = spi_transport();
    let rust = Sh1106::from_parts(io, 2, 239, 0, false, ResetHelper::from_parts(None));
    rust.init().unwrap();
    assert_eq!(
        dc_value(&dc.sent.lock().unwrap()),
        oracle_dc,
        "SH1106 DC stream"
    );
    assert_eq!(
        bus_value(&spi.sent.lock().unwrap()),
        oracle_spi,
        "SH1106 SPI stream"
    );
    assert_eq!(rust.vram_snapshot().len(), 8);
    assert_eq!(rust.vram_snapshot()[0].len(), 132);
    // `from_config` mirrors the reads (x_offset first) and attributes.
    let (cfg, cfg_rec) = FakeConfig::with_printer(
        world.printer.clone(),
        &[("cs_pin", "PA0"), ("dc_pin", "PA1"), ("x_offset", "2")],
    );
    let rust_cfg = Sh1106::from_config(&cfg).unwrap();
    assert_eq!(rust_cfg.columns(), 132);
    assert_eq!(rust_cfg.x_offset(), 2);
    assert_eq!(rust_cfg.vram_snapshot()[0].len(), 132);
    assert_eq!(
        oracle_reads(&mut h, "cfg_sh.reads"),
        chip_layer_reads(&cfg_rec.reads()),
        "SH1106 config read order"
    );

    // `x_offset` out of range (max 3): same class and message.
    let py_res = h
        .py
        .exec("uc1701.SH1106(_FakeConfig({'cs_pin': 'PA0', 'dc_pin': 'PA1', 'x_offset': '4'}))");
    let (cfg_bad, _bad_rec) = FakeConfig::with_printer(
        world.printer.clone(),
        &[("cs_pin", "PA0"), ("dc_pin", "PA1"), ("x_offset", "4")],
    );
    let rust_res = Sh1106::from_config(&cfg_bad);
    match (py_res, rust_res) {
        (Err(py_err), Err(rust_err)) => {
            assert_eq!(
                rust_err.class_name(),
                py_err.class_name.as_str(),
                "x_offset error class"
            );
            assert_eq!(
                rust_err.message(),
                py_err.message.as_str(),
                "x_offset error message"
            );
        }
        _ => panic!("x_offset=4 must fail on both sides"),
    }

    // Non-numeric `x_offset`: parse failure on both sides.
    let py_res2 = h
        .py
        .exec("uc1701.SH1106(_FakeConfig({'cs_pin': 'PA0', 'dc_pin': 'PA1', 'x_offset': 'abc'}))");
    let (cfg_bad2, _bad2_rec) = FakeConfig::with_printer(
        world.printer.clone(),
        &[("cs_pin", "PA0"), ("dc_pin", "PA1"), ("x_offset", "abc")],
    );
    let rust_res2 = Sh1106::from_config(&cfg_bad2);
    match (py_res2, rust_res2) {
        (Err(py_err), Err(rust_err)) => {
            assert_eq!(
                rust_err.class_name(),
                py_err.class_name.as_str(),
                "x_offset parse class"
            );
            assert_eq!(
                rust_err.message(),
                py_err.message.as_str(),
                "x_offset parse message"
            );
        }
        _ => panic!("x_offset=abc must fail on both sides"),
    }
}
