//! Parity tests for `extras::tmc2130` (port of `klippy/extras/tmc2130.py`).
//!
//! Every expected value comes live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + frozen `klippy/`): minimal Python fakes
//! for printer/config/SPI/pins/gcode/webhooks are installed, the reference
//! classes are constructed, and the same calls are driven on the Rust port
//! through matching fakes. Floats compare as recording values (bit-for-bit),
//! errors by Python class and message, SPI command bytes exactly.

use extras::tmc2130::{
    field_formatters, fields_table, lookup_tmc_spi_chain_with, read_registers, registers,
    signed_fields, McuTmcSpi, McuTmcSpiChain, Tmc2130, TmcCurrentHelper, TMC_FREQUENCY,
    MAX_CURRENT,
};
use extras::tmc::{FieldHelper, McuTmcApi, RegisterRaw};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi};
use klippy_api::stepper::Stepper;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandApi, GetFloatOpts as GGetFloatOpts, GetOpts as GGetOpts, GetIntOpts as GGetIntOpts, GCodeCommandHandler, GCmd, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandWrapper, Chip, CommandQueryWrapper, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, AsyncResponseWrapper, AsyncResponseWrapperApi,
    ParsePinOpts, PinResolver, PinsApi,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi,
};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SECTION: &str = "tmc2130 stepper_x";

// ---------------------------------------------------------------------------
// Python-side fakes (built once per oracle process).
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configfile
import extras.tmc2130 as tmc2130
from extras import tmc, bus

class CommandError(Exception):
    pass
class Error(Exception):
    pass
class FakeReactor:
    def mutex(self, is_locked=False):
        import contextlib
        @contextlib.contextmanager
        def cm():
            yield
        return cm()
    def monotonic(self):
        return 0.0

class FakePinParams(dict):
    pass

class FakePins:
    def __init__(self):
        self._params = FakePinParams()
    def lookup_pin(self, pin, share_type=None):
        if 'pin' not in self._params:
            self._params['pin'] = 'PA0'
            self._params['chip'] = FakeMcuClock()
        return self._params
    def register_chip(self, name, obj):
        pass

class FakeGcode:
    def register_mux_command(self, cmd, key, value, func, desc=None):
        pass

class FakeWebhooks:
    def register_mux_endpoint(self, path, key, value, callback):
        pass

class FakeStepperEnable:
    pass

class FakePrinter:
    def __init__(self, debugoutput=None):
        self._reactor = FakeReactor()
        self._debugoutput = debugoutput
        self._pins = FakePins()
        self._gcode = FakeGcode()
        self._webhooks = FakeWebhooks()
        self.command_error = CommandError
    def get_start_args(self):
        return {'debugoutput': self._debugoutput}
    def get_reactor(self):
        return self._reactor
    def lookup_object(self, name, default=None):
        objs = {'pins': self._pins, 'gcode': self._gcode, 'webhooks': self._webhooks}
        if name in objs:
            return objs[name]
        if default is None:
            return None
        raise Exception('no object ' + name)
    def load_object(self, config, section, default=None):
        return FakeStepperEnable()
    def register_event_handler(self, event, cb):
        pass

class FakeConfig:
    error = Error
    def __init__(self, printer, name, floats, ints, bools, strs):
        self._printer = printer
        self._name = name
        self._floats = dict(floats)
        self._ints = dict(ints)
        self._bools = dict(bools)
        self._strs = dict(strs)
        self.reads = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def _wrapper(self, opt, store, conv, default, minval=None, maxval=None,
                 above=None, below=None):
        if opt not in store:
            if default is not configfile.sentinel:
                return default
            raise Error("Option '%s' in section '%s' must be specified"
                        % (opt, self._name))
        v = conv(store[opt])
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s"
                        % (opt, self._name, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s"
                        % (opt, self._name, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s"
                        % (opt, self._name, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s"
                        % (opt, self._name, below))
        return v
    def getfloat(self, opt, default=configfile.sentinel, minval=None,
                 maxval=None, above=None, below=None, note_valid=True):
        self.reads.append(('getfloat', opt))
        return self._wrapper(opt, self._floats, float, default, minval, maxval,
                             above, below)
    def getint(self, opt, default=configfile.sentinel, minval=None, maxval=None,
               note_valid=True):
        self.reads.append(('getint', opt))
        return self._wrapper(opt, self._ints, int, default, minval, maxval)
    def getboolean(self, opt, default=configfile.sentinel, note_valid=True):
        self.reads.append(('getboolean', opt))
        return self._wrapper(opt, self._bools, bool, default)
    def get(self, opt, default=configfile.sentinel, note_valid=True):
        self.reads.append(('get', opt))
        return self._wrapper(opt, self._strs, str, default)
    def getchoice(self, option, choices, default=configfile.sentinel, note_valid=True):
        self.reads.append(('getchoice', option))
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default, note_valid=note_valid)
        else:
            c = self.get(option, default, note_valid=note_valid)
        if c not in choices:
            raise Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self._name))
        return choices[c]
    def getsection(self, section):
        return type(self)(self._printer, section, self._floats, self._ints, self._bools, self._strs)
    def has_section(self, section):
        return section == 'stepper_x'

class FakeMcuClock:
    def print_time_to_clock(self, print_time):
        return int(print_time * 10000000)

class FakeSpi:
    def __init__(self, transfers, clock):
        self.transfers = list(transfers)
        self.clock = clock
        self.sends = []
        self.transfer_calls = []
        self.preface_calls = []
    def spi_send(self, data, minclock=0, reqclock=0):
        self.sends.append((list(data), minclock, reqclock))
        return None
    def spi_transfer(self, data, minclock=0, reqclock=0):
        self.transfer_calls.append((list(data), minclock, reqclock))
        resp, rt = self.transfers.pop(0)
        return {'response': bytes(resp), '#receive_time': float(rt)}
    def spi_transfer_with_preface(self, preface_data, data, minclock=0, reqclock=0):
        self.preface_calls.append((list(preface_data), list(data), minclock, reqclock))
        resp, rt = self.transfers.pop(0)
        return {'response': bytes(resp), '#receive_time': float(rt)}
    def print_time_to_clock(self, print_time):
        return self.clock.print_time_to_clock(print_time)
    def get_mcu(self):
        return self.clock

class FakeMcuTmcSpi:
    def __init__(self, fields):
        self._fields = fields
        self.register_calls = []
        self.transfer_results = []
    def get_fields(self):
        return self._fields
    def set_register(self, reg_name, val, print_time=None):
        self.register_calls.append((reg_name, int(val), print_time))

def make_fields():
    return tmc.FieldHelper(tmc2130.Fields, tmc2130.SignedFields, tmc2130.FieldFormatters)

def calc_tuple(run_current, hold_current, sense_resistor, fields):
    m = FakeMcuTmcSpi(fields)
    c = FakeConfig(FakePrinter(None), "tmc2130 stepper_x",
                   {'run_current': run_current, 'hold_current': hold_current, 'sense_resistor': sense_resistor},
                   {}, {}, {})
    h = tmc2130.TMCCurrentHelper(c, m)
    return (h.sense_resistor, h.req_hold_current, fields.registers.copy())

def fresh_chain(chain_len, transfers, debugoutput=None):
    printer = FakePrinter(debugoutput)
    spi = FakeSpi(transfers, FakeMcuClock())
    chain = tmc2130.MCU_TMC_SPI_chain.__new__(tmc2130.MCU_TMC_SPI_chain)
    chain.printer = printer
    chain.chain_len = chain_len
    chain.mutex = printer.get_reactor().mutex()
    chain.spi = spi
    chain.taken_chain_positions = []
    return chain, spi

def fresh_mcu_spi(chain, chain_pos=1):
    m = tmc2130.MCU_TMC_SPI.__new__(tmc2130.MCU_TMC_SPI)
    m.printer = chain.printer
    m.name = 'stepper_x'
    m.tmc_spi = chain
    m.chain_pos = chain_pos
    m.mutex = chain.mutex
    m.name_to_reg = dict(tmc2130.Registers)
    m.fields = chain.fields if hasattr(chain, 'fields') else make_fields()
    m.tmc_frequency = tmc2130.TMC_FREQUENCY
    return m
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    debugoutput: Option<String>,
    pins: Option<Arc<FakePins>>,
    gcode: Option<Arc<FakeGcode>>,
    webhooks: Option<Arc<FakeWebhooks>>,
    stepper_enable: Arc<FakeStepperEnable>,
}



impl PrinterObject for FakePrinter {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(
        self: Arc<Self>,
    ) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut d = indexmap::IndexMap::new();
        let v = match &self.debugoutput {
            Some(s) => PyValue::Str(s.clone()),
            None => PyValue::None,
        };
        d.insert(PyValue::Str("debugoutput".to_string()), v);
        PyValue::Dict(d)
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let obj = match name {
            "pins" => self.pins.clone().map(|p| p as ObjectRef),
            "gcode" => self.gcode.clone().map(|p| p as ObjectRef),
            "webhooks" => self.webhooks.clone().map(|p| p as ObjectRef),
            _ => None,
        };
        match obj {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(
        &self,
        _module: Option<&str>,
    ) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(Some(self.stepper_enable.clone() as ObjectRef))
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _cb: EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeMcu {
    name: String,
    printer: Printer,
}

struct FakeAsyncWrapper;

impl AsyncResponseWrapperApi for FakeAsyncWrapper {
    fn unregister(&self) {}
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin not used".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
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
        pycompat::text::trunc_float_to_bigint(time * 1000000.0)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
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
        Err(KlippyError::Mcu("lookup_command not used".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu("lookup_query_command not used".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsyncWrapper)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        indexmap::IndexMap::new()
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::Mcu("get_constant_float not used".to_string()))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        pycompat::text::trunc_float_to_bigint(print_time * 10000000.0)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
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
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Err(KlippyError::Mcu("calibrate_clock not used".to_string()))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Mcu("create_trigger_dispatch not used".to_string()))
    }
}

struct FakePins {
    mcu: Mcu,
}

impl klippy_api::PrinterObject for FakePins {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
    klippy_api::printer_interfaces!(dyn PinsApi);
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Err(KlippyError::Mcu("parse_pin not used".to_string()))
    }
    fn lookup_pin(&self, _pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: "PA0".to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin not used".to_string()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::Mcu("get_pin_resolver not used".to_string()))
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

struct FakeGcode;

impl klippy_api::PrinterObject for FakeGcode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
}

struct FakeGcmd;

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
    fn get_command_parameters(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
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

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
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
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
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
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd)
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

struct FakeWebhooks;
struct FakeStepperEnable;

impl klippy_api::PrinterObject for FakeWebhooks {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn WebHooksApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
}

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        Ok(())
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
        _method: &str,
        _params: &indexmap::IndexMap<String, PyValue>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
}

impl klippy_api::PrinterObject for FakeStepperEnable {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterStepperEnableApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
}

/// The oracle's `FakeStepperEnable` is `class FakeStepperEnable: pass`, so
/// every method the reference tmc helper could call on it raises
/// `AttributeError`. `TMCCommandHelper.__init__` only stores the object.
fn fake_stepper_enable_attribute(attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'FakeStepperEnable' object has no attribute '{attr}'"),
    }
}

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        Err(fake_stepper_enable_attribute("register_stepper"))
    }
    fn set_motors_enable(&self, _stepper_names: &[String], _enable: bool) -> KResult<bool> {
        Err(fake_stepper_enable_attribute("set_motors_enable"))
    }
    fn motor_off(&self) -> KResult<()> {
        Err(fake_stepper_enable_attribute("motor_off"))
    }
    fn lookup_enable(&self, _name: &str) -> KResult<Arc<EnableTracking>> {
        Err(fake_stepper_enable_attribute("lookup_enable"))
    }
    fn get_steppers(&self) -> Vec<String> {
        Vec::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
}

fn fake_printer(debugoutput: Option<String>) -> Printer {
    let base = Arc::new(FakePrinter {
        reactor: Reactor::new(false),
        debugoutput,
        pins: None,
        gcode: Some(Arc::new(FakeGcode)),
        webhooks: Some(Arc::new(FakeWebhooks)),
        stepper_enable: Arc::new(FakeStepperEnable),
    });
    let mcu: Mcu = Arc::new(FakeMcu {
        name: "mcu".to_string(),
        printer: base.clone(),
    });
    let with_pins = Arc::new(FakePrinter {
        reactor: Reactor::new(false),
        debugoutput: base.debugoutput.clone(),
        pins: Some(Arc::new(FakePins { mcu })),
        gcode: Some(Arc::new(FakeGcode)),
        webhooks: Some(Arc::new(FakeWebhooks)),
        stepper_enable: Arc::new(FakeStepperEnable),
    });
    with_pins
}

struct FakeConfig {
    printer: Printer,
    name: String,
    floats: std::collections::HashMap<String, f64>,
    ints: std::collections::HashMap<String, i64>,
    bools: std::collections::HashMap<String, bool>,
    strs: std::collections::HashMap<String, String>,
    reads: std::sync::Mutex<Vec<String>>,
}

impl FakeConfig {
    fn new(
        printer: Printer,
        name: &str,
        floats: &[(&str, f64)],
        ints: &[(&str, i64)],
        bools: &[(&str, bool)],
        strs: &[(&str, &str)],
    ) -> Arc<FakeConfig> {
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            floats: floats
                .iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect(),
            ints: ints
                .iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect(),
            bools: bools
                .iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect(),
            strs: strs
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
            reads: std::sync::Mutex::new(Vec::new()),
        })
    }
    fn mark(&self, what: &str, opt: &str) {
        self.reads
            .lock()
            .unwrap()
            .push(format!("{what}:{opt}"));
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.to_string()
    }
    fn get(
        &self,
        option: &str,
        opts: GetStrOpts,
    ) -> KResult<Option<String>> {
        self.mark("get", option);
        if let Some(v) = self.strs.get(option) {
            return Ok(Some(v.clone()));
        }
        Ok(opts.default.unwrap_or(None))
    }
    fn getint(
        &self,
        option: &str,
        opts: GetIntOpts,
    ) -> KResult<Option<i64>> {
        self.mark("getint", option);
        let v = if let Some(v) = self.ints.get(option) {
            *v
        } else {
            return Ok(opts.default.unwrap_or(None));
        };
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(KlippyError::Config(
                    format!("Error: value {v} below minimum {minv}"),
                ));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(KlippyError::Config(
                    format!("Error: value {v} above maximum {maxv}"),
                ));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(
        &self,
        option: &str,
        opts: GetFloatOpts,
    ) -> KResult<Option<f64>> {
        self.mark("getfloat", option);
        let v = if let Some(v) = self.floats.get(option) {
            *v
        } else {
            return Ok(opts.default.unwrap_or(None));
        };
        if let Some(a) = opts.above {
            if v <= a {
                return Err(KlippyError::Config(
                    format!("Error: value {v} not above {a}"),
                ));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(KlippyError::Config(
                    format!("Error: value {v} not below {b}"),
                ));
            }
        }
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(KlippyError::Config(
                    format!("Error: value {v} below minimum {minv}"),
                ));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(KlippyError::Config(
                    format!("Error: value {v} above maximum {maxv}"),
                ));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(
        &self,
        option: &str,
        opts: GetBoolOpts,
    ) -> KResult<Option<bool>> {
        self.mark("getboolean", option);
        if let Some(v) = self.bools.get(option) {
            return Ok(Some(*v));
        }
        Ok(opts.default.unwrap_or(None))
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        self.mark("getchoice", option);
        let dict = match choices {
            PyValue::Dict(d) => d,
            _ => {
                return Err(KlippyError::Config(
                    "Choice is not a dict".to_string(),
                ))
            }
        };
        // Klipper getchoice with int keys routes through getint.
        let c = match self.ints.get(option) {
            Some(v) => *v,
            None => {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                )))
            }
        };
        for (k, v) in dict.iter() {
            if matches!(k, PyValue::Int(_)) && k.str() == c.to_string() {
                return Ok(v.clone());
            }
        }
        Err(KlippyError::Config(format!(
            "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
            self.name
        )))
    }
    fn getlists(
        &self,
        _option: &str,
        opts: GetListsOpts,
    ) -> KResult<PyValue> {
        Ok(opts.default.unwrap_or(None).unwrap_or(PyValue::None))
    }
    fn getlist(
        &self,
        _option: &str,
        opts: GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        Ok(opts.default.unwrap_or(None))
    }
    fn getintlist(
        &self,
        _option: &str,
        opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(opts.default.unwrap_or(None))
    }
    fn getfloatlist(
        &self,
        _option: &str,
        opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(opts.default.unwrap_or(None))
    }
    fn getsection(&self, section: &str) -> Config {
        let config: Arc<FakeConfig> = Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            floats: self.floats.clone(),
            ints: self.ints.clone(),
            bools: self.bools.clone(),
            strs: self.strs.clone(),
            reads: std::sync::Mutex::new(Vec::new()),
        });
        config as Config
    }
    fn has_section(&self, section: &str) -> bool {
        section == "stepper_x"
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

struct FakeSpi {
    mcu: Mcu,
    transfers: std::sync::Mutex<Vec<(Vec<u8>, f64)>>,
    sends: std::sync::Mutex<Vec<(Vec<u8>, i64, i64)>>,
    transfer_calls: std::sync::Mutex<Vec<(Vec<u8>, i64, i64)>>,
    preface_calls: std::sync::Mutex<Vec<(Vec<u8>, Vec<u8>, i64, i64)>>,
}

impl FakeSpi {
    fn new(printer: Printer, transfers: Vec<(Vec<u8>, f64)>) -> Arc<FakeSpi> {
        Arc::new(FakeSpi {
            mcu: Arc::new(FakeMcu {
                name: "mcu".to_string(),
                printer,
            }),
            transfers: std::sync::Mutex::new(transfers),
            sends: std::sync::Mutex::new(Vec::new()),
            transfer_calls: std::sync::Mutex::new(Vec::new()),
            preface_calls: std::sync::Mutex::new(Vec::new()),
        })
    }
}

fn spi_params(transfers: &mut Vec<(Vec<u8>, f64)>) -> PyValue {
    let (resp, rt) = transfers.pop().expect("transfer");
    let mut d = indexmap::IndexMap::new();
    d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(resp));
    d.insert(PyValue::Str("#receive_time".to_string()), PyValue::Float(rt));
    PyValue::Dict(d)
}

impl extras::bus::McuSpiApi for FakeSpi {
    fn get_oid(&self) -> i64 {
        1
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn setup_shutdown_msg(&self, _shutdown_seq: &[u8]) -> KResult<()> {
        Ok(())
    }
    fn spi_send(&self, data: &[u8], opts: klippy_api::mcu::SendOpts) -> KResult<()> {
        self.sends
            .lock()
            .unwrap()
            .push((data.to_vec(), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn spi_transfer(
        &self,
        data: &[u8],
        opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        self.transfer_calls
            .lock()
            .unwrap()
            .push((data.to_vec(), opts.minclock, opts.reqclock));
        Ok(spi_params(&mut self.transfers.lock().unwrap()))
    }
    fn spi_transfer_with_preface(
        &self,
        preface_data: &[u8],
        data: &[u8],
        opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        self.preface_calls.lock().unwrap().push((
            preface_data.to_vec(),
            data.to_vec(),
            opts.minclock,
            opts.reqclock,
        ));
        Ok(spi_params(&mut self.transfers.lock().unwrap()))
    }
}

struct FakeMcuTmc {
    fields: Arc<FieldHelper>,
    register_calls: std::sync::Mutex<Vec<(String, i64, Option<f64>)>>,
    mcu: Mcu,
}

impl McuTmcApi for FakeMcuTmc {
    fn get_fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }
    fn get_register_raw(&self, _reg_name: &str) -> KResult<RegisterRaw> {
        Err(KlippyError::Command("get_register_raw not used".to_string()))
    }
    fn get_register(&self, _reg_name: &str) -> KResult<i64> {
        Err(KlippyError::Command("get_register not used".to_string()))
    }
    fn set_register(
        &self,
        reg_name: &str,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        self.register_calls.lock().unwrap().push((
            reg_name.to_string(),
            val,
            print_time,
        ));
        Ok(())
    }
    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(TMC_FREQUENCY)
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn name_to_reg_value(&self, reg_name: &str) -> Option<i64> {
        registers().get(reg_name).copied()
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("tmc2130 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping tmc2130 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.tmc2130".to_string(),
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
    actual: &Result<Value, KlippyError>,
) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => {
            let actual = actual.as_ref().err().expect("error comparison");
            assert_eq!(
                actual.class_name(),
                e.class_name,
                "{label}: error class"
            );
            assert_eq!(
                actual.message(),
                e.message.as_str(),
                "{label}: error message"
            );
            // Compare the Rust error class and message against the
            // reference exception (strings are exact).
            return;
        }
    };
    panic!("{label}: oracle returned {expected:?}, expected an error");
}

fn vec_u8_values(data: &[u8]) -> Value {
    Value::List(
        data.iter()
            .map(|b| Value::int(*b as i64))
            .collect(),
    )
}

fn fields_rust() -> Arc<FieldHelper> {
    FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None)
}

fn chain_rust(
    printer: Printer,
    chain_len: i64,
    spi: Arc<FakeSpi>,
) -> Arc<McuTmcSpiChain> {
    Arc::new(McuTmcSpiChain::new(printer, chain_len, spi))
}

fn mcu_tmc_rust(
    config: &Config,
    fields: Arc<FieldHelper>,
    chain: Arc<McuTmcSpiChain>,
    chain_pos: i64,
) -> Arc<McuTmcSpi> {
    Arc::new(McuTmcSpi::with_chain(
        config,
        registers(),
        fields,
        TMC_FREQUENCY,
        chain,
        chain_pos,
    ))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn register_tables_match_python() {
    let Some(mut py) = oracle() else { return };
    let reg_val = Value::Dict(
        registers()
            .iter()
            .map(|(k, v)| (Value::str(k), Value::int(*v)))
            .collect(),
    );
    let _ = py.eval("dict(tmc2130.Registers)");
    // oracle dict order may differ from IndexMap order in the recorded
    // dict; compare as sorted key/value lists instead of dict equality.
    let reg_sorted_actual: Vec<(String, i64)> = registers()
        .iter()
        .map(|(k, v)| (k.clone(), *v))
        .collect();
    let mut reg_sorted_actual = reg_sorted_actual;
    reg_sorted_actual.sort();
    let expected_reg = py.eval("sorted((k, v) for k, v in tmc2130.Registers.items())");
    check("Registers sorted", expected_reg, {
        Value::List(
            reg_sorted_actual
                .iter()
                .map(|(k, v)| {
                    Value::Tuple(vec![Value::str(k), Value::int(*v)])
                })
                .collect(),
        )
    });
    let _ = reg_val;

    let read_actual = Value::List(
        read_registers()
            .iter()
            .map(|s| Value::str(s))
            .collect(),
    );
    let expected = py.eval("list(tmc2130.ReadRegisters)");
    check("ReadRegisters", expected, read_actual);

    let fields_actual = Value::List(
        fields_table()
            .iter()
            .flat_map(|(reg, fs)| {
                fs.iter()
                    .map(move |(f, m)| {
                        Value::Tuple(vec![
                            Value::str(reg),
                            Value::str(f),
                            Value::int(*m),
                        ])
                    })
            })
            .collect(),
    );
    let _ = fields_actual;
    let expected = py
        .eval("sorted((r, f, m) for r, fs in tmc2130.Fields.items() for f, m in fs.items())");
    let mut actual_sorted: Vec<(String, String, i64)> = fields_table()
        .iter()
        .flat_map(|(r, fs)| {
            fs.iter()
                .map(move |(f, m)| (r.clone(), f.clone(), *m))
        })
        .collect();
    actual_sorted.sort();
    let actual = Value::List(
        actual_sorted
            .iter()
            .map(|(r, f, m)| {
                Value::Tuple(vec![
                    Value::str(r),
                    Value::str(f),
                    Value::int(*m),
                ])
            })
            .collect(),
    );
    check("Fields sorted", expected, actual);

    let signed_actual = Value::List(
        signed_fields().iter().map(|s| Value::str(s)).collect(),
    );
    let expected = py.eval("list(tmc2130.SignedFields)");
    check("SignedFields", expected, signed_actual);
}

#[test]
fn field_helper_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("CHOPCONF", "mres", 0x00400007u32),
        ("CHOPCONF", "vsense", 0x00020000u32 | 0x10),
        ("CHOPCONF", "toff", 0x0A),
        ("IOIN", "version", 0x42 << 24),
        ("DRV_STATUS", "cs_actual", 0x1F << 16 | 0x1234),
        ("MSCURACT", "cur_a", 0x1FF),
        ("MSCURACT", "cur_b", 0x1FF << 16),
        ("COOLCONF", "sgt", 0x7F << 16),
        ("GSTAT", "reset", 1),
        ("GSTAT", "drv_err", 2),
    ];
    for (reg, field, raw) in cases {
        let raw = raw as i64;
        let helper = FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None);
        let actual = helper
            .get_field(field, Some(raw), Some(reg))
            .expect("get_field");
        py.exec("fh = make_fields()").expect("py fields");
        let expected = py.eval(&format!("fh.get_field('{field}', {raw}, '{reg}')"));
        check(
            &format!("get_field {reg}.{field} {raw:#x}"),
            expected,
            Value::int(actual),
        );
    }
    // pretty_format for a populated DRV_STATUS and IOIN.
    let helper = FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None);
    for (reg, raw) in [("DRV_STATUS", 0x03FF1234i64), ("IOIN", 0x42 << 24 | 0x15)] {
        let actual = helper.pretty_format(reg, raw);
        py.exec("fh = make_fields()").expect("py fields");
        let expected = py.eval(&format!("fh.pretty_format('{reg}', {raw:#x})"));
        check(
            &format!("pretty_format {reg} {raw:#x}"),
            expected,
            Value::str(&actual),
        );
    }
}

#[test]
fn field_helper_set_config_field_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("fh = make_fields()").expect("py fields");
    let helper = FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None);
    let config = FakeConfig::new(
        fake_printer(None),
        SECTION,
        &[],
        &[],
        &[],
        &[],
    ) as Config;
    let actual = helper.set_config_field(&config, "toff", extras::tmc::ConfigFieldDefault::Int(4)).expect("toff");
    let expected = py
        .eval("fh.set_config_field(FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {}, {}, {}, {}), 'toff', 4)");
    check("set_config_field toff default", expected, Value::int(actual));
    let actual = helper
        .set_config_field(&config, "pwm_autoscale", extras::tmc::ConfigFieldDefault::Bool(true))
        .expect("autoscale");
    let expected = py
        .eval("fh.set_config_field(FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {}, {}, {}, {}), 'pwm_autoscale', True)");
    check("set_config_field autoscale", expected, Value::int(actual));
    let actual = helper.set_config_field(&config, "toff", extras::tmc::ConfigFieldDefault::Int(4)).expect("toff again");
    let expected = py
        .eval("fh.set_config_field(FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {}, {}, {}, {}), 'toff', 4)");
    check("set_config_field toff again", expected, Value::int(actual));
}

#[test]
fn current_helper_matches_python() {
    let Some(mut py) = oracle() else { return };
    let grid: &[(f64, f64)] = &[
        (0.5, 0.3),
        (1.0, 0.5),
        (1.25, 1.25),
        (2.0, 2.0),
        (0.3, 1.0),
    ];
    for (run, hold) in grid {
        let fields = fields_rust();
        let printer = fake_printer(None);
        let mcu = Arc::new(FakeMcuTmc {
            fields: fields.clone(),
            register_calls: Default::default(),
            mcu: Arc::new(FakeMcu {
                name: "mcu".to_string(),
                printer: printer.clone(),
            }),
        });
        let config = FakeConfig::new(
            printer.clone(),
            SECTION,
            &[("run_current", *run), ("hold_current", *hold)],
            &[],
            &[],
            &[],
        ) as Config;
        let helper = TmcCurrentHelper::new(&config, mcu).expect("helper");
        // Compare bit-exact currents.
        let (run_cur, hold_cur, req_hold, max_cur) = helper.get_current();
        py.exec(&format!(
                "m = FakeMcuTmcSpi(make_fields()); h = tmc2130.TMCCurrentHelper(FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {{'run_current': {run:?}, 'hold_current': {hold:?}}}, {{}}, {{}}, {{}}), m)",
            ))
            .expect("py helper");
        let expected = py
            .eval("(m._fields.get_field('vsense'), m._fields.get_field('irun'), m._fields.get_field('ihold'), h.get_current())");
        let actual = Value::Tuple(vec![
            Value::int(
                fields
                    .get_field("vsense", None, None)
                    .expect("vsense"),
            ),
            Value::int(fields.get_field("irun", None, None).expect("irun")),
            Value::int(fields.get_field("ihold", None, None).expect("ihold")),
            Value::Tuple(vec![
                Value::float(run_cur),
                Value::float(hold_cur),
                Value::float(req_hold),
                Value::float(max_cur),
            ]),
        ]);
        check(
            &format!("currents run={run} hold={hold}"),
            expected,
            actual,
        );
    }
}

#[test]
fn current_helper_sense_resistor_matches_python() {
    let Some(mut py) = oracle() else { return };
    // `sense_resistor` is this unit's third config option; the reference uses
    // it for both bit calculations (`tmc2130.py:134-142`).
    let grid: &[(f64, f64, f64)] = &[
        (1.0, 0.5, 0.110),
        (1.0, 0.5, 0.075),
        (1.8, 1.8, 0.05),
        (0.4, 0.1, 0.22),
    ];
    for (run, hold, sense) in grid {
        let fields = fields_rust();
        let printer = fake_printer(None);
        let mcu = Arc::new(FakeMcuTmc {
            fields: fields.clone(),
            register_calls: Default::default(),
            mcu: Arc::new(FakeMcu {
                name: "mcu".to_string(),
                printer: printer.clone(),
            }),
        });
        let config = FakeConfig::new(
            printer.clone(),
            SECTION,
            &[
                ("run_current", *run),
                ("hold_current", *hold),
                ("sense_resistor", *sense),
            ],
            &[],
            &[],
            &[],
        ) as Config;
        let helper = TmcCurrentHelper::new(&config, mcu).expect("helper");
        let actual = Value::Tuple(vec![
            Value::float(helper.sense_resistor()),
            Value::float(helper.req_hold_current()),
            Value::Dict(
                fields
                    .registers()
                    .iter()
                    .map(|(k, v)| (Value::str(k), Value::int(*v)))
                    .collect(),
            ),
        ]);
        py.exec(&format!(
            "t = calc_tuple({run:?}, {hold:?}, {sense:?}, make_fields())",
        ))
        .expect("py calc_tuple");
        let expected = py.eval("t");
        check(
            &format!("sense_resistor run={run} hold={hold} sense={sense}"),
            expected,
            actual,
        );
    }
}

#[test]
fn current_helper_set_current_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fields = fields_rust();
    let printer = fake_printer(None);
    let mcu = Arc::new(FakeMcuTmc {
        fields: fields.clone(),
        register_calls: Default::default(),
        mcu: Arc::new(FakeMcu {
            name: "mcu".to_string(),
            printer: printer.clone(),
        }),
    });
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[("run_current", 0.5), ("hold_current", 0.25)],
        &[],
        &[],
        &[],
    ) as Config;
    let helper = TmcCurrentHelper::new(&config, mcu.clone()).expect("helper");
    helper
        .set_current(1.5, 0.75, Some(100.0))
        .expect("set_current");
    let rust_calls = mcu.register_calls.lock().unwrap().clone();
    let actual_calls = Value::List(
        rust_calls
            .iter()
            .map(|(r, v, t)| {
                Value::Tuple(vec![
                    Value::str(r),
                    Value::int(*v),
                    match t {
                        Some(tt) => Value::float(*tt),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    );
    py.exec("m2 = FakeMcuTmcSpi(make_fields()); h2 = tmc2130.TMCCurrentHelper(FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {'run_current': 0.5, 'hold_current': 0.25}, {}, {}, {}), m2)")
        .expect("py helper");
    py.exec("h2.set_current(1.5, 0.75, 100.0)").expect("py set_current");
    let expected = py.eval("list(m2.register_calls)");
    check("set_current register writes", expected, actual_calls);

}

#[test]
fn spi_chain_build_cmd_matches_python() {
    let Some(mut py) = oracle() else { return };
    let printer = fake_printer(None);
    for (chain_len, chain_pos) in [(1, 1), (2, 1), (2, 2), (4, 2), (4, 4)] {
        let spi = FakeSpi::new(printer.clone(), Vec::new());
        let chain = chain_rust(printer.clone(), chain_len, spi);
        let data = vec![0x6Cu8, 0x01, 0x02, 0x03, 0x04];
        let actual = chain.build_cmd(&data, chain_pos);
        py.exec(&format!(
                "ch, _ = fresh_chain({}, [], None)",
                chain_len
            ))
            .expect("py chain");
        let expected = py
            .eval(&format!("list(ch._build_cmd([0x6c, 0x01, 0x02, 0x03, 0x04], {}))", chain_pos));
        check(
            &format!("build_cmd len={chain_len} pos={chain_pos}"),
            expected,
            vec_u8_values(&actual),
        );
    }
}

#[test]
fn spi_chain_reg_read_write_matches_python() {
    let Some(mut py) = oracle() else { return };
    // reg_read non-debug, single chip.
    let response = vec![0xAAu8, 0x12, 0x34, 0x56, 0x78];
    let printer = fake_printer(None);
    let spi = FakeSpi::new(printer.clone(), vec![(response.clone(), 0.25)]);
    let chain = chain_rust(printer.clone(), 1, spi.clone());
    let actual = chain.reg_read(0x6c, 1).expect("reg_read");
    let actual_val = Value::Tuple(vec![
        Value::int(actual.spi_status),
        Value::int(actual.data),
        Value::float(actual.receive_time),
    ]);
    py.exec("ch, _spi = fresh_chain(1, [([0xAA, 0x12, 0x34, 0x56, 0x78], 0.25)], None)")
        .expect("py read chain");
    let expected = py
        .eval("((lambda r: (r['spi_status'], r['data'], r['#receive_time']))(ch.reg_read(0x6c, 1)))");
    check("reg_read single", expected, actual_val);
    // SPI command bytes observed by the fake.
    let sends = spi.sends.lock().unwrap().clone();
    let actual_cmd = Value::List(
        sends.iter().map(|(d, _, _)| vec_u8_values(d)).collect(),
    );
    py.exec("ch, sp = fresh_chain(1, [([0xAA, 0x12, 0x34, 0x56, 0x78], 0.25)], None)")
        .expect("py read chain 2");
    py.exec("ch.reg_read(0x6c, 1)").expect("py reg_read call");
    let expected = py
        .eval("[list(d) for d, _, _ in sp.sends]");
    check("reg_read command bytes", expected, actual_cmd);

    // reg_write with print time, single chip.
    let write_resp = vec![0u8, 0x12, 0x34, 0x56, 0x78];
    let spi = FakeSpi::new(printer.clone(), vec![(write_resp, 0.5)]);
    let chain = chain_rust(printer.clone(), 1, spi.clone());
    let actual = chain.reg_write(0x6c, 0x12345678, 1, Some(42.0)).expect("reg_write");
    py.exec("ch, sp = fresh_chain(1, [([0x00, 0x12, 0x34, 0x56, 0x78], 0.5)], None)")
        .expect("py write chain");
    let expected = py
        .eval("ch.reg_write(0x6c, 0x12345678, 1, 42.0)");
    check("reg_write previous data", expected, Value::int(actual));
    let preface = spi.preface_calls.lock().unwrap().clone();
    let actual_preface = Value::List(
        preface
            .iter()
            .map(|(a, b, mc, rc)| {
                Value::Tuple(vec![
                    vec_u8_values(a),
                    vec_u8_values(b),
                    Value::int(*mc),
                    Value::int(*rc),
                ])
            })
            .collect(),
    );
    py.exec("ch, sp = fresh_chain(1, [([0x00, 0x12, 0x34, 0x56, 0x78], 0.5)], None)")
        .expect("py write chain 2");
    py.exec("ch.reg_write(0x6c, 0x12345678, 1, 42.0)")
        .expect("py reg_write call");
    let expected = py
        .eval("[(list(a), list(b), mc, rc) for a, b, mc, rc in sp.preface_calls]");
    check("reg_write preface bytes", expected, actual_preface);

    // debugoutput short-circuit (no transfer).
    let dpi = fake_printer(Some("/tmp/debug".to_string()));
    let spi = FakeSpi::new(dpi.clone(), Vec::new());
    let chain = chain_rust(dpi, 1, spi.clone());
    let actual = chain.reg_read(0x6c, 1).expect("debug read");
    py.exec("ch, _ = fresh_chain(1, [], '/tmp/debug')")
        .expect("py debug chain");
    let expected = py
        .eval("((lambda r: (r['spi_status'], r['data'], r['#receive_time']))(ch.reg_read(0x6c, 1)))");
    check(
        "reg_read debugoutput",
        expected,
        Value::Tuple(vec![
            Value::int(actual.spi_status),
            Value::int(actual.data),
            Value::float(actual.receive_time),
        ]),
    );
    // A fresh chain for the write: the oracle's `fresh_chain` starts with no
    // recorded sends, so reuse would compare the read command here.
    let dpi = fake_printer(Some("/tmp/debug".to_string()));
    let spi = FakeSpi::new(dpi.clone(), Vec::new());
    let chain = chain_rust(dpi, 1, spi.clone());
    let actual = chain
        .reg_write(0x6c, 0x0BADF00D, 1, None)
        .expect("debug write");
    py.exec("ch, sp = fresh_chain(1, [], '/tmp/debug')")
        .expect("py debug write chain");
    let expected = py
        .eval("(ch.reg_write(0x6c, 0x0BADF00D, 1), [list(d) for d, _, _ in sp.sends])");
    let sends = spi.sends.lock().unwrap().clone();
    check(
        "reg_write debugoutput",
        expected,
        Value::Tuple(vec![
            Value::int(actual),
            Value::List(
                sends
                    .iter()
                    .map(|(d, _, _)| vec_u8_values(d))
                    .collect(),
            ),
        ]),
    );
}

#[test]
fn spi_chain_daisy_slice_matches_python() {
    let Some(mut py) = oracle() else { return };
    // Two-chip chain, position 1: response slice starts at byte 5.
    let response = vec![
        0, 0, 0, 0, 0, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1, 0, 0, 0, 0, 0,
    ];
    let printer = fake_printer(None);
    let spi = FakeSpi::new(printer.clone(), vec![(response, 1.0)]);
    let chain = chain_rust(printer, 2, spi);
    let actual = chain.reg_read(0x10, 1).expect("daisy read");
    py.exec("ch, _ = fresh_chain(2, [([0, 0, 0, 0, 0, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1, 0, 0, 0, 0, 0], 1.0)], None)")
        .expect("py daisy chain");
    let expected = py
        .eval("((lambda r: (r['spi_status'], r['data'], r['#receive_time']))(ch.reg_read(0x10, 1)))");
    check(
        "reg_read daisy pos1 slice",
        expected,
        Value::Tuple(vec![
            Value::int(actual.spi_status),
            Value::int(actual.data),
            Value::float(actual.receive_time),
        ]),
    );
}

#[test]
fn mcu_tmc_spi_matches_python() {
    let Some(mut py) = oracle() else { return };
    // set_register success path: one transfer whose previous data equals val.
    let printer = fake_printer(None);
    let fields = fields_rust();
    let val = 0x12345678i64;
    let resp = vec![0, 0x12, 0x34, 0x56, 0x78];
    let spi = FakeSpi::new(printer.clone(), vec![(resp.clone(), 0.1)]);
    let chain = chain_rust(printer.clone(), 1, spi.clone());
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[("run_current", 1.0)],
        &[],
        &[],
        &[],
    ) as Config;
    let mcu = mcu_tmc_rust(&config, fields.clone(), chain, 1);
    mcu.set_register("IHOLD_IRUN", val, None).expect("set_register");
    assert_eq!(spi.preface_calls.lock().unwrap().len(), 1);
    py.exec("ch, _ = fresh_chain(1, [([0, 0x12, 0x34, 0x56, 0x78], 0.1)], None); m = fresh_mcu_spi(ch)")
        .expect("py mcu spi chain");
    let expected = py
        .eval("m.set_register('IHOLD_IRUN', 0x12345678)");
    check("set_register success", expected, Value::Null);

    // decode_spi_status
    let actual = Value::Dict(
        McuTmcSpi::decode_spi_status(0b1011)
            .iter()
            .map(|(k, v)| (Value::str(k), Value::int(*v)))
            .collect(),
    );
    let expected = py
        .eval("tmc2130.MCU_TMC_SPI.decode_spi_status(0b1011)");
    check("decode_spi_status 0b1011", expected, actual);

    // Error path: previous data never matches -> command_error after 5 tries.
    let printer = fake_printer(None);
    let spi = FakeSpi::new(printer.clone(), vec![(vec![0, 0, 0, 0, 0], 0.0); 5]);
    let chain = chain_rust(printer.clone(), 1, spi);
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[("run_current", 1.0)],
        &[],
        &[],
        &[],
    ) as Config;
    let mcu = mcu_tmc_rust(
        &config,
        fields_rust(),
        chain,
        1,
    );
    let err = mcu.set_register("IHOLD_IRUN", 0x11, None).unwrap_err();
    py.exec("ch, _ = fresh_chain(1, [([0, 0, 0, 0, 0], 0.0)] * 5, None); m = fresh_mcu_spi(ch)")
        .expect("py retry setup");
    let expected = py.eval("m.set_register('IHOLD_IRUN', 0x11)");
    check_err(
        "set_register exhausted",
        expected,
        &Err::<Value, KlippyError>(err),
    );
}

#[test]
fn lookup_chain_matches_python() {
    let Some(mut py) = oracle() else { return };
    // Unchained: no chain_length.
    let printer = fake_printer(None);
    let spi = FakeSpi::new(printer.clone(), Vec::new());
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[],
        &[],
        &[],
        &[],
    ) as Config;
    let make = |c: &Config, len: i64| -> KResult<Arc<McuTmcSpiChain>> {
        let _ = c;
        Ok(Arc::new(McuTmcSpiChain::new(
            printer.clone(),
            len,
            spi.clone(),
        )))
    };
    let (_, pos) = lookup_tmc_spi_chain_with(&config, None, &make).expect("unchained");
    assert_eq!(pos, 1);
    py.exec("c = FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {}, {}, {}, {})")
        .expect("py config");
    py.exec("bus.MCU_SPI_from_config = lambda config, *a, **k: FakeSpi([], FakeMcuClock())")
        .expect("py bus stub");
    let expected = py
        .eval("tmc2130.lookup_tmc_spi_chain(c)[1]");
    check("lookup unchained pos", expected, Value::int(pos));

    // Duplicate position error.
    let printer = fake_printer(None);
    let spi2 = FakeSpi::new(printer.clone(), Vec::new());
    let chain2 = Arc::new(McuTmcSpiChain::new(
        printer.clone(),
        2,
        spi2,
    ));
    assert!(chain2.claim_chain_position(1));
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[],
        &[("chain_length", 2), ("chain_position", 1)],
        &[],
        &[("cs_pin", "PA0")],
    ) as Config;
    let make = |_c: &Config, _len: i64| -> KResult<Arc<McuTmcSpiChain>> {
        unreachable!("chain already exists")
    };
    let err = lookup_tmc_spi_chain_with(&config, Some(chain2), &make).err().expect("dup error");
    py.exec("ch, _ = fresh_chain(2, [], None); ch.taken_chain_positions = [1]; p = FakePrinter(None); p._pins._params['class'] = ch; cok = FakeConfig(p, 'tmc2130 stepper_x', {}, {'chain_length': 2, 'chain_position': 1}, {}, {'cs_pin': 'PA0'})")
        .expect("py duplicate setup");
    let expected = py.eval("tmc2130.lookup_tmc_spi_chain(cok)");
    check_err(
        "lookup duplicate position",
        expected,
        &Err::<Value, KlippyError>(err),
    );

    // Length mismatch error.
    let printer = fake_printer(None);
    let spi3 = FakeSpi::new(printer.clone(), Vec::new());
    let chain3 = Arc::new(McuTmcSpiChain::new(
        printer.clone(),
        3,
        spi3,
    ));
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[],
        &[("chain_length", 2), ("chain_position", 1)],
        &[],
        &[("cs_pin", "PA0")],
    ) as Config;
    let err = lookup_tmc_spi_chain_with(&config, Some(chain3), &make).err().expect("mismatch error");
    py.exec("ch, _ = fresh_chain(3, [], None); p = FakePrinter(None); p._pins._params['class'] = ch; cok = FakeConfig(p, 'tmc2130 stepper_x', {}, {'chain_length': 2, 'chain_position': 1}, {}, {'cs_pin': 'PA0'})")
        .expect("py mismatch setup");
    let expected = py.eval("tmc2130.lookup_tmc_spi_chain(cok)");
    check_err(
        "lookup length mismatch",
        expected,
        &Err::<Value, KlippyError>(err),
    );
}

#[test]
fn tmc2130_field_defaults_match_python() {
    let Some(mut py) = oracle() else { return };
    // Build a full TMC2130 with minimal config (all driver defaults).
    let printer = fake_printer(None);
    let spi = FakeSpi::new(printer.clone(), Vec::new());
    let chain = chain_rust(printer.clone(), 1, spi);
    let fields = fields_rust();
    let config = FakeConfig::new(
        printer.clone(),
        SECTION,
        &[("run_current", 1.0)],
        &[("microsteps", 16)],
        &[],
        &[],
    ) as Config;
    let mcu = mcu_tmc_rust(&config, fields.clone(), chain, 1);
    let obj = Tmc2130::assemble(&config, fields.clone(), mcu).expect("TMC2130");
    let regs = obj.fields().registers();
    let actual = Value::Dict(
        regs.iter()
            .map(|(k, v)| (Value::str(k), Value::int(*v)))
            .collect(),
    );
    py.exec("bus.MCU_SPI_from_config = lambda config, *a, **k: FakeSpi([], FakeMcuClock())")
        .expect("py bus stub");
    py.exec("c2 = FakeConfig(FakePrinter(None), 'tmc2130 stepper_x', {'run_current': 1.0}, {'microsteps': 16}, {}, {})")
        .expect("py defaults config");
    py.exec("obj2 = tmc2130.TMC2130(c2)").expect("py TMC2130 init");
    let expected = py.eval("dict(obj2.fields.registers)");
    check("TMC2130 register defaults", expected, actual);
    // Current defaults: run_current is required, hold defaults to MAX.
    let (run_cur, _hold_cur, req_hold, max_cur) = obj.current_helper().get_current();
    assert_eq!(max_cur, MAX_CURRENT);
    assert_eq!(req_hold, MAX_CURRENT);
    assert!(run_cur > 0.0 && run_cur <= MAX_CURRENT + 1e-9);
}

#[test]
fn tmc_frequency_is_13200000() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("tmc2130.TMC_FREQUENCY");
    check("TMC_FREQUENCY", expected, Value::float(TMC_FREQUENCY));
    let expected = py.eval("tmc2130.MAX_CURRENT");
    check("MAX_CURRENT", expected, Value::float(MAX_CURRENT));
}
