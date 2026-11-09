//! Parity tests for `extras::tmc2240` (port of `klippy/extras/tmc2240.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`, with the real `tmc`, `tmc2130`,
//! `tmc_uart` and `bus` modules). Python objects are built with minimal
//! fakes for printer, config, pins and the TMC transport, driven with the
//! same calls as the Rust port through fakes of the contract traits.
//!
//! Compared:
//! - `Registers`, `ReadRegisters`, `Fields`, `SignedFields`, `TMC_FREQUENCY`
//!   and every `FieldFormatters` entry (through `FieldHelper.pretty_format`
//!   of every register and several values);
//! - `TMC2240CurrentHelper` construction (config options read with their
//!   defaults and bounds, register values set, `Rref`, `req_hold_current`,
//!   `get_current`), the private current calculations over a grid of
//!   currents, and `set_current` register writes (register, value,
//!   print_time), including the NaN `int()` error and every bounds error;
//! - `load_config_prefix` transport selection and ordering: `uart_pin`
//!   (present, empty, absent), `chain_length`, `cs_pin`, with the sibling
//!   transports (`MCU_TMC_uart`, `MCU_TMC_SPI`) running against a pins fake
//!   that rejects every pin. The config options read in order, the
//!   `lookup_pin` arguments and the raised error (class and message) are
//!   compared.
//!
//! `TMC2240.assemble` (everything the constructor does after choosing the
//! transport) is compared against the reference constructor with the SPI
//! transport replaced by a fake: config options read in order with their
//! defaults and bounds, `load_object`, event handlers, mux commands, the
//! bulk endpoint, the heaters monitor, the virtual pin chip, every register
//! value set, `get_status` and `get_phase_offset`, and the raised errors.
//! The transports themselves (`MCU_TMC_SPI`, `MCU_TMC_uart`) belong to their
//! owners and are only driven through their failing pin lookup above.

use extras::heaters::{Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi};
use extras::tmc::{FieldHelper, McuTmcApi, RegisterRaw, TMCCommandHelperApi, TmcCurrentHelperApi};
use extras::tmc2240::{
    field_formatters, fields, load_config_prefix, module_factory, read_registers, registers,
    signed_fields, TMC2240CurrentHelper, TMC2240, TMC_FREQUENCY,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::Stepper;
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::text::{float_repr, str_repr};
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
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
sys.modules['mcu'] = _mcu_mod
sys.modules['stepper'] = types.ModuleType("stepper")
import extras.tmc2240 as tmc2240
import extras.tmc as tmc

class _PinsError(Exception):
    pass
_PinsError.__name__ = "error"

_sentinel = object()

def _r(v):
    return '-' if v is None else repr(v)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, values):
        self._printer = printer
        self.section = section
        self.values = values
        self.log = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def _lookup(self, option, default):
        if option not in self.values:
            if default is not _sentinel:
                return False, default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return True, self.values[option]
    def get(self, option, default=_sentinel, note_valid=True):
        self.log.append("get|%s|%s" % (option, 'required' if default is _sentinel else repr(default)))
        found, v = self._lookup(option, default)
        return v
    def getint(self, option, default=_sentinel, minval=None, maxval=None, note_valid=True):
        self.log.append("getint|%s|%s|%s|%s" % (option, 'required' if default is _sentinel else repr(default), _r(minval), _r(maxval)))
        found, v = self._lookup(option, default)
        if not found:
            return v
        try:
            v = int(v)
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        self.log.append("getfloat|%s|%s|%s|%s|%s|%s" % (option, 'required' if default is _sentinel else repr(default), _r(minval), _r(maxval), _r(above), _r(below)))
        found, v = self._lookup(option, default)
        if not found:
            return v
        try:
            v = float(v)
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise configparser.Error("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v
    def getboolean(self, option, default=_sentinel, note_valid=True):
        if default is _sentinel:
            shown = 'required'
        else:
            shown = repr(default)
        self.log.append("getboolean|%s|%s" % (option, shown))
        found, v = self._lookup(option, default)
        if not found:
            return v
        if v.lower() in ('1', 'yes', 'true', 'on'):
            return True
        if v.lower() in ('0', 'no', 'false', 'off'):
            return False
        raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
    def getchoice(self, option, choices, default=_sentinel, note_valid=True):
        self.log.append("getchoice|%s|%s" % (option, 'required' if default is _sentinel else repr(default)))
        found, v = self._lookup(option, default)
        if not found:
            return choices[v]
        try:
            v = int(v)
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if v not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (v, option, self.section))
        return choices[v]
    def getsection(self, section):
        sub = FakeConfig(self._printer, section, self.values)
        sub.log = self.log
        return sub
    def has_section(self, section):
        return section == 'stepper_x'

class FakeReactor:
    def mutex(self, is_locked=False):
        return object()

class FakePins:
    error = _PinsError
    def __init__(self):
        self.log = []
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        self.log.append("lookup_pin|%r|%r|%r|%r" % (pin_desc, can_invert, can_pullup, share_type))
        raise self.error("Unknown pin chip name '%s'" % (pin_desc,))
    def register_chip(self, chip_name, chip):
        self.log.append("register_chip|%s" % (chip_name,))

class FakeGcode:
    def __init__(self, log):
        self.log = log
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.log.append("register_mux_command|%s|%s|%r|%r" % (cmd, key, value, desc))

class FakeWebhooks:
    def __init__(self, log):
        self.log = log
    def register_mux_endpoint(self, path, key, value, callback):
        self.log.append("register_mux_endpoint|%s|%s|%r" % (path, key, value))

class FakeHeaters:
    def __init__(self, log):
        self.log = log
    def register_monitor(self, config):
        self.log.append("register_monitor|%s" % (config.get_name(),))

class FakeStepperEnable:
    pass

class FakePrinter:
    def __init__(self):
        self.pins = FakePins()
        self.log = []
        self.objects = {
            "pins": self.pins,
            "gcode": FakeGcode(self.log),
            "webhooks": FakeWebhooks(self.log),
            "heaters": FakeHeaters(self.log),
            "stepper_enable": FakeStepperEnable(),
        }
        self.reactor = FakeReactor()
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return {}
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_sentinel):
        self.log.append("load_object|%s" % (section,))
        return self.lookup_object(section, default)
    def register_event_handler(self, event, cb):
        self.log.append("register_event_handler|%s" % (event,))

def run_load(values):
    p = FakePrinter()
    cfg = FakeConfig(p, "tmc2240 stepper_x", values)
    err = None
    try:
        tmc2240.load_config_prefix(cfg)
    except Exception as e:
        err = (type(e).__name__, str(e))
    return (err, cfg.log, p.pins.log)

class FakeMcuTmcBuild:
    def __init__(self, fields):
        self.fields = fields
        self.name_to_reg = tmc2240.Registers
    def get_fields(self):
        return self.fields
    def get_mcu(self):
        return None
    def get_tmc_frequency(self):
        return tmc2240.TMC_FREQUENCY
    def set_register(self, reg_name, val, print_time=None):
        raise Exception("set_register is not reached while building")

def run_build(values):
    p = FakePrinter()
    cfg = FakeConfig(p, "tmc2240 stepper_x", values)
    created = []
    def make_spi(config, name_to_reg, fields, tmc_frequency):
        m = FakeMcuTmcBuild(fields)
        created.append(m)
        return m
    orig = tmc2240.tmc2130.MCU_TMC_SPI
    tmc2240.tmc2130.MCU_TMC_SPI = make_spi
    err = obj = None
    try:
        obj = tmc2240.TMC2240(cfg)
    except Exception as e:
        err = (type(e).__name__, str(e))
    finally:
        tmc2240.tmc2130.MCU_TMC_SPI = orig
    regs = list(created[0].fields.registers.items()) if created else None
    status = None if obj is None else obj.get_status(0.)
    phase = None if obj is None else obj.get_phase_offset()
    return (err, cfg.log, p.log, p.pins.log, regs, status, phase)

class FakeMcuTmc:
    def __init__(self):
        self.fields = tmc.FieldHelper(tmc2240.Fields, tmc2240.SignedFields, tmc2240.FieldFormatters)
        self.log = []
    def get_fields(self):
        return self.fields
    def set_register(self, reg_name, val, print_time=None):
        self.log.append("%s|%d|%r" % (reg_name, val, print_time))

class Helper:
    pass

def make_helper(values):
    p = FakePrinter()
    cfg = FakeConfig(p, "tmc2240 stepper_x", values)
    mcu = FakeMcuTmc()
    w = Helper()
    w.cfg = cfg
    w.mcu = mcu
    w.h = None
    w.err = None
    try:
        w.h = tmc2240.TMC2240CurrentHelper(cfg, mcu)
    except Exception as e:
        w.err = (type(e).__name__, str(e))
    return w

def snapshot(w):
    h = w.h
    cur = None
    if h is not None:
        cur = h.get_current()
    return (w.err, w.cfg.log, list(w.mcu.fields.registers.items()), cur,
            None if h is None else h.Rref,
            None if h is None else h.req_hold_current,
            w.mcu.log)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Log = Arc<GCell<Vec<String>>>;

fn new_log() -> Log {
    Arc::new(GCell::new(Vec::new()))
}

struct FakePrinter {
    objects: GCell<Vec<(String, ObjectRef)>>,
    reactor: Reactor,
    /// `load_object` and `register_event_handler` calls, in order, plus the
    /// gcode, webhooks and heaters fakes' registrations (one shared log).
    log: Log,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: GCell::new(Vec::new()),
            reactor: Reactor::new(false),
            log: new_log(),
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
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.borrow_mut().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found = self
            .objects
            .borrow()
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, o)| o.clone());
        match (found, default) {
            (Some(obj), _) => Ok(Some(obj)),
            (None, LookupDefault::Raise) => {
                Err(KlippyError::config(format!("Unknown config object '{name}'")))
            }
            (None, _) => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects.borrow().clone().into_iter().map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.log.borrow_mut().push(format!("load_object|{section}"));
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.log.borrow_mut().push(format!("register_event_handler|{event}"));
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    section: String,
    values: Vec<(String, String)>,
    log: Log,
}

impl FakeConfig {
    fn new(printer: Printer, values: &[(&str, &str)]) -> Arc<FakeConfig> {
        Arc::new(FakeConfig {
            printer,
            section: "tmc2240 stepper_x".to_string(),
            values: values
                .iter()
                .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
                .collect(),
            log: new_log(),
        })
    }

    /// `config.getsection(section)`: same options, same log.
    fn for_section(&self, section: &str) -> Arc<FakeConfig> {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            values: self.values.clone(),
            log: self.log.clone(),
        })
    }

    fn lookup(&self, option: &str) -> Option<String> {
        self.values.iter().find(|(k, _)| k == option).map(|(_, v)| v.clone())
    }

    fn required(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option, self.section
        ))
    }

    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{}' in section '{}'",
            option, self.section
        ))
    }

    fn bound_error(&self, option: &str, what: &str, bound: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{}' in section '{}' must {} {}",
            option, self.section, what, bound
        ))
    }
}

fn dash<T>(v: Option<T>, render: impl Fn(T) -> String) -> String {
    v.map(render).unwrap_or_else(|| "-".to_string())
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        let default = match &opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(s)) => str_repr(s),
        };
        self.log.borrow_mut().push(format!("get|{option}|{default}"));
        match (self.lookup(option), opts.default) {
            (Some(v), _) => Ok(Some(v)),
            (None, None) => Err(self.required(option)),
            (None, Some(d)) => Ok(d),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let default = match opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => v.to_string(),
        };
        self.log.borrow_mut().push(format!(
            "getint|{option}|{default}|{}|{}",
            dash(opts.minval, |v| v.to_string()),
            dash(opts.maxval, |v| v.to_string())
        ));
        let Some(raw) = self.lookup(option) else {
            return match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            };
        };
        let v: i64 = raw.parse().map_err(|_| self.unparsable(option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(self.bound_error(option, "have minimum of", &min.to_string()));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(self.bound_error(option, "have maximum of", &max.to_string()));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let default = match opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => float_repr(v),
        };
        self.log.borrow_mut().push(format!(
            "getfloat|{option}|{default}|{}|{}|{}|{}",
            dash(opts.minval, float_repr),
            dash(opts.maxval, float_repr),
            dash(opts.above, float_repr),
            dash(opts.below, float_repr)
        ));
        let Some(raw) = self.lookup(option) else {
            return match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            };
        };
        let v: f64 = raw.parse().map_err(|_| self.unparsable(option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(self.bound_error(option, "have minimum of", &float_repr(min)));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(self.bound_error(option, "have maximum of", &float_repr(max)));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(self.bound_error(option, "be above", &float_repr(above)));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(self.bound_error(option, "be below", &float_repr(below)));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let default = match (opts.default, opts.default_is_int) {
            (None, _) => "required",
            (Some(None), _) => "None",
            (Some(Some(true)), false) => "True",
            (Some(Some(false)), false) => "False",
            (Some(Some(true)), true) => "1",
            (Some(Some(false)), true) => "0",
        };
        self.log.borrow_mut().push(format!("getboolean|{option}|{default}"));
        let Some(raw) = self.lookup(option) else {
            return match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            };
        };
        match raw.to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(Some(true)),
            "0" | "no" | "false" | "off" => Ok(Some(false)),
            _ => Err(self.unparsable(option)),
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let default = match opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(_)) => panic!("getchoice defaults are not used"),
        };
        self.log.borrow_mut().push(format!("getchoice|{option}|{default}"));
        let PyValue::Dict(map) = choices else {
            panic!("the microsteps choices are a dict")
        };
        let Some(raw) = self.lookup(option) else {
            return Err(self.required(option));
        };
        let v: i64 = raw.parse().map_err(|_| self.unparsable(option))?;
        match map.get(&PyValue::Int(PyInt::from_i64(v))) {
            Some(found) => Ok(found.clone()),
            None => Err(KlippyError::config(format!(
                "Choice '{v}' for option '{option}' in section '{}' is not a valid choice",
                self.section
            ))),
        }
    }
    fn getlists(
        &self,
        _option: &str,
        _opts: GetListsOpts,
    ) -> KResult<PyValue> {
        panic!("getlists is not used")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not reached before the transport fails")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used")
    }
    fn getsection(&self, section: &str) -> Config {
        self.for_section(section)
    }
    fn has_section(&self, section: &str) -> bool {
        section == "stepper_x"
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options is not used")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

/// A pins object that rejects every `lookup_pin` after logging it.
struct FakePins {
    log: Log,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        fn py_bool(b: bool) -> &'static str {
            if b { "True" } else { "False" }
        }
        self.log.borrow_mut().push(format!(
            "lookup_pin|{}|{}|{}|{}",
            str_repr(pin_desc),
            py_bool(opts.can_invert),
            py_bool(opts.can_pullup),
            match &opts.share_type {
                Some(s) => str_repr(s),
                None => "None".to_string(),
            }
        ));
        Err(KlippyError::Pins(format!("Unknown pin chip name '{pin_desc}'")))
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { panic!("reset_pin_sharing is not used") }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used")
    }
    fn register_chip(&self, chip_name: &str, _chip: Chip) -> KResult<()> {
        self.log.borrow_mut().push(format!("register_chip|{chip_name}"));
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

fn opt_repr(v: Option<&str>) -> String {
    v.map(str_repr).unwrap_or_else(|| "None".to_string())
}

/// Logs every `register_mux_command`; the other methods are not reached by
/// the TMC helpers.
struct FakeGcode {
    log: Log,
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        panic!("register_command is not used")
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.log.borrow_mut().push(format!(
            "register_mux_command|{cmd}|{key}|{}|{}",
            opt_repr(value),
            opt_repr(desc)
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("register_output_handler is not used")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used")
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used")
    }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

/// Logs `register_mux_endpoint` (`TMCStallguardDump`'s bulk endpoint).
struct FakeWebhooks {
    log: Log,
}

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        panic!("register_endpoint is not used")
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        self.log.borrow_mut().push(format!(
            "register_mux_endpoint|{path}|{key}|{}",
            str_repr(value)
        ));
        Ok(())
    }
    fn call_remote_method(
        &self,
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        panic!("call_remote_method is not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used")
    }
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

/// Logs `register_monitor`, the only call `TMCErrorCheck` makes.
struct FakeHeaters {
    log: Log,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {
        panic!("add_sensor_factory is not used")
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("get_all_heaters is not used")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("lookup_heater is not used")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used")
    }
    fn register_monitor(&self, config: &Config) {
        self.log
            .borrow_mut()
            .push(format!("register_monitor|{}", config.get_name()));
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature is not used")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

/// `TMCCommandHelper.__init__` only stores `stepper_enable`.
struct FakeStepperEnable;

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        panic!("register_stepper is not used")
    }
    fn set_motors_enable(&self, _stepper_names: &[String], _enable: bool) -> KResult<bool> {
        panic!("set_motors_enable is not used")
    }
    fn motor_off(&self) -> KResult<()> {
        panic!("motor_off is not used")
    }
    fn lookup_enable(&self, _name: &str) -> KResult<Arc<EnableTracking>> {
        panic!("lookup_enable is not used")
    }
    fn get_steppers(&self) -> Vec<String> {
        panic!("get_steppers is not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used")
    }
}

klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

/// The `Mcu` a TMC transport reports; the current helper never uses it.
struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        panic!("unused")
    }
    fn get_printer(&self) -> Printer {
        panic!("unused")
    }
    fn is_fileoutput(&self) -> bool {
        panic!("unused")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("unused")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("unused")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("unused")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("unused")
    }
    fn request_move_queue_slot(&self) {
        panic!("unused")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("unused")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("unused")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("unused")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("unused")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("unused")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("unused")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("unused")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("unused")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("unused")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("unused")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("unused")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("unused")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("unused")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("unused")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("unused")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("unused")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("unused")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("unused")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("unused")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("unused")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("unused")
    }
}

/// `mcu_tmc` of the current-helper tests: a real `FieldHelper` seeded with
/// this module's tables and a log of `set_register` calls.
struct FakeMcuTmc {
    fields: Arc<FieldHelper>,
    log: Log,
}

impl FakeMcuTmc {
    fn new() -> Arc<FakeMcuTmc> {
        Arc::new(FakeMcuTmc {
            fields: FieldHelper::new(fields(), signed_fields(), field_formatters(), None),
            log: new_log(),
        })
    }
}

impl McuTmcApi for FakeMcuTmc {
    fn get_fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }
    fn get_register(&self, _reg_name: &str) -> KResult<i64> {
        panic!("unused")
    }
    fn get_register_raw(&self, _reg_name: &str) -> KResult<RegisterRaw> {
        panic!("unused")
    }
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        self.log.borrow_mut().push(format!(
            "{reg_name}|{val}|{}",
            print_time.map(float_repr).unwrap_or_else(|| "None".to_string())
        ));
        Ok(())
    }
    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(TMC_FREQUENCY)
    }
    fn get_mcu(&self) -> Mcu {
        Arc::new(FakeMcu)
    }
    fn name_to_reg_value(&self, name: &str) -> Option<i64> {
        registers().get(name).copied()
    }
}

// ---------------------------------------------------------------------------
// Oracle plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("tmc2240 python fakes");
            Some(py)
        }
        None => {
            println!("skipping tmc2240 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
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
        unit: "extras.tmc2240".to_string(),
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

/// A finite or non-finite `f64` as a Python float expression.
fn pyf(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x == f64::INFINITY {
        "float('inf')".to_string()
    } else if x == f64::NEG_INFINITY {
        "float('-inf')".to_string()
    } else {
        format!("{x:?}")
    }
}

/// An optional float: `None` compares with Python `None`.
fn opt_float(x: Option<f64>) -> Value {
    match x {
        Some(v) => Value::float(v),
        None => Value::Null,
    }
}

fn py_dict(values: &[(&str, &str)]) -> String {
    let items: Vec<String> = values
        .iter()
        .map(|(k, v)| format!("{}: {}", str_repr(k), str_repr(v)))
        .collect();
    format!("{{{}}}", items.join(", "))
}

fn strs(v: Vec<String>) -> Value {
    Value::List(v.iter().map(|s| Value::str(s)).collect())
}

fn error_value(err: Option<&KlippyError>) -> Value {
    match err {
        None => Value::Null,
        Some(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
    }
}

fn registers_pairs(fields: &FieldHelper) -> Value {
    Value::List(
        fields
            .registers()
            .into_iter()
            .map(|(k, v)| Value::Tuple(vec![Value::str(&k), Value::int(v)]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tables and formatters.
// ---------------------------------------------------------------------------

#[test]
fn module_header_tables_match_python() {
    let Some(mut py) = oracle() else { return };
    check("TMC_FREQUENCY", py.eval("tmc2240.TMC_FREQUENCY"), Value::float(TMC_FREQUENCY));
    check(
        "Registers",
        py.eval("list(tmc2240.Registers.items())"),
        Value::List(
            registers()
                .iter()
                .map(|(k, v)| Value::Tuple(vec![Value::str(k), Value::int(*v)]))
                .collect(),
        ),
    );
    check(
        "ReadRegisters",
        py.eval("list(tmc2240.ReadRegisters)"),
        strs(read_registers()),
    );
    check(
        "Fields",
        py.eval("[(r, list(f.items())) for r, f in tmc2240.Fields.items()]"),
        Value::List(
            fields()
                .iter()
                .map(|(reg, reg_fields)| {
                    Value::Tuple(vec![
                        Value::str(reg),
                        Value::List(
                            reg_fields
                                .iter()
                                .map(|(f, m)| Value::Tuple(vec![Value::str(f), Value::int(*m)]))
                                .collect(),
                        ),
                    ])
                })
                .collect(),
        ),
    );
    check(
        "SignedFields",
        py.eval("list(tmc2240.SignedFields)"),
        strs(signed_fields()),
    );
}

#[test]
fn field_formatters_match_python() {
    let Some(mut py) = oracle() else { return };
    let helper = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
    let values: [i64; 8] = [
        0,
        1,
        0x5a5a5a5a,
        0x12345678,
        0xffffffff,
        0x80000001,
        0x00001fff,
        0x1fff1fff,
    ];
    for reg in registers().keys() {
        for value in values {
            let expected = py.eval(&format!(
                "tmc.FieldHelper(tmc2240.Fields, tmc2240.SignedFields, tmc2240.FieldFormatters).pretty_format({}, {})",
                str_repr(reg),
                value
            ));
            check(
                &format!("pretty_format {reg} {value:#x}"),
                expected,
                Value::str(&helper.pretty_format(reg, value)),
            );
        }
    }
}

#[test]
fn field_helper_over_tables_matches_python() {
    let Some(mut py) = oracle() else { return };
    let helper = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
    let mut cases: Vec<(String, String, i64)> = Vec::new();
    for (reg, reg_fields) in fields() {
        for field in reg_fields.keys() {
            for value in [0x5a5a5a5a_i64, 0xffffffff, 0x80000001, 0x01ff01ab] {
                cases.push((reg.clone(), field.clone(), value));
            }
        }
    }
    for (reg, field, value) in &cases {
        let expected = py.eval(&format!(
            "tmc.FieldHelper(tmc2240.Fields, tmc2240.SignedFields, tmc2240.FieldFormatters).get_field({}, {}, {})",
            str_repr(field),
            value,
            str_repr(reg)
        ));
        let actual = helper
            .get_field(field, Some(*value), Some(reg))
            .expect("get_field");
        check(&format!("get_field {reg}.{field} {value:#x}"), expected, Value::int(actual));
    }
}

// ---------------------------------------------------------------------------
// TMC2240CurrentHelper.
// ---------------------------------------------------------------------------

struct RustHelper {
    result: KResult<TMC2240CurrentHelper>,
    mcu: Arc<FakeMcuTmc>,
    config: Arc<FakeConfig>,
}

fn rust_helper(values: &[(&str, &str)]) -> RustHelper {
    let printer: Printer = FakePrinter::new();
    let config = FakeConfig::new(printer, values);
    let mcu = FakeMcuTmc::new();
    let cfg: Config = config.clone();
    let result = TMC2240CurrentHelper::new(&cfg, mcu.clone() as Arc<dyn McuTmcApi>);
    RustHelper { result, mcu, config }
}

impl RustHelper {
    fn snapshot(&self) -> Value {
        let (cur, rref, req) = match &self.result {
            Ok(h) => {
                let (run, hold, req_hold, ifs) = h.get_current();
                (
                    Value::Tuple(vec![
                        Value::float(run),
                        opt_float(hold),
                        opt_float(req_hold),
                        Value::float(ifs),
                    ]),
                    Value::float(h.rref()),
                    opt_float(h.req_hold_current()),
                )
            }
            Err(_) => (Value::Null, Value::Null, Value::Null),
        };
        Value::Tuple(vec![
            error_value(self.result.as_ref().err()),
            strs(self.config.log.cloned()),
            registers_pairs(&self.mcu.fields),
            cur,
            rref,
            req,
            strs(self.mcu.log.cloned()),
        ])
    }
}

fn py_make_helper(py: &mut pyoracle::Oracle, name: &str, values: &[(&str, &str)]) {
    py.exec(&format!("{name} = make_helper({})", py_dict(values)))
        .expect("make_helper");
}

#[test]
fn current_helper_construction_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<Vec<(&str, &str)>> = vec![
        vec![("run_current", "0.8")],
        vec![("rref", "12000.0"), ("run_current", "0.8"), ("hold_current", "0.4")],
        vec![("rref", "20000.0"), ("run_current", "0.5"), ("hold_current", "0.25")],
        vec![("rref", "60000.0"), ("run_current", "0.2"), ("hold_current", "0.2")],
        vec![("rref", "30000.0"), ("run_current", "0.3"), ("hold_current", "0.45")],
        vec![("rref", "12000.0"), ("run_current", "2.1")],
        vec![("rref", "12000.0"), ("run_current", "2.12")],
        vec![("rref", "12000.0"), ("run_current", "1.0"), ("hold_current", "0.0")],
        vec![("rref", "12000.0"), ("run_current", "1.0"), ("hold_current", "9.0")],
        vec![("rref", "11999.0"), ("run_current", "0.8")],
        vec![("rref", "60001.0"), ("run_current", "0.2")],
        vec![("rref", "abc"), ("run_current", "0.2")],
        vec![("run_current", "0.0")],
        vec![("run_current", "-1.0")],
        vec![("run_current", "9.0")],
        vec![("run_current", "nan")],
        vec![("run_current", "inf")],
        vec![("rref", "12000.0"), ("run_current", "0.8"), ("hold_current", "nan")],
        vec![],
    ];
    for (index, values) in cases.iter().enumerate() {
        let rust = rust_helper(values);
        py_make_helper(&mut py, "w", values);
        check(
            &format!("construct #{index} {values:?}"),
            py.eval("snapshot(w)"),
            rust.snapshot(),
        );
    }
}

#[test]
fn current_helper_calculations_match_python() {
    let Some(mut py) = oracle() else { return };
    let currents: [f64; 12] = [
        0.0001, 0.01, 0.05, 0.1, 0.2, 0.4, 0.5, 0.8, 1.0, 1.4, 2.0, 2.12,
    ];
    for rref in [12000.0_f64, 20000.0, 33333.0, 60000.0] {
        let values_owned = [
            ("rref".to_string(), pyf(rref)),
            ("run_current".to_string(), "0.2".to_string()),
        ];
        let values: Vec<(&str, &str)> = values_owned
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        // `{rref:?}` of 33333.0 reads back through float().
        let rust = rust_helper(&values);
        let helper = rust.result.as_ref().expect("helper");
        py_make_helper(&mut py, "w", &values);

        let ranges: Vec<Value> = (0..4)
            .map(|r| Value::float(helper.get_ifs_rms(Some(r)).expect("ifs")))
            .collect();
        check(
            &format!("ifs_rms rref={rref}"),
            py.eval("[w.h._get_ifs_rms(r) for r in range(4)]"),
            Value::List(ranges),
        );
        check(
            &format!("ifs_rms default rref={rref}"),
            py.eval("w.h._get_ifs_rms()"),
            Value::float(helper.get_ifs_rms(None).expect("ifs")),
        );

        let expr_list = currents.iter().map(|c| pyf(*c)).collect::<Vec<_>>().join(", ");
        check(
            &format!("calc_current_range rref={rref}"),
            py.eval(&format!("[w.h._calc_current_range(c) for c in [{expr_list}]]")),
            Value::List(
                currents
                    .iter()
                    .map(|c| Value::int(helper.calc_current_range(*c).expect("range")))
                    .collect(),
            ),
        );
        check(
            &format!("calc_globalscaler rref={rref}"),
            py.eval(&format!("[w.h._calc_globalscaler(c) for c in [{expr_list}]]")),
            Value::List(
                currents
                    .iter()
                    .map(|c| Value::int(helper.calc_globalscaler(*c).expect("gscaler")))
                    .collect(),
            ),
        );
        let scalers: [i64; 6] = [0, 1, 32, 100, 255, 256];
        let scaler_list = scalers.iter().map(|s| s.to_string()).collect::<Vec<_>>().join(", ");
        let mut expected_bits = Vec::new();
        for c in &currents {
            for s in &scalers {
                expected_bits.push(Value::int(helper.calc_current_bits(*c, *s).expect("bits")));
            }
        }
        check(
            &format!("calc_current_bits rref={rref}"),
            py.eval(&format!(
                "[w.h._calc_current_bits(c, s) for c in [{expr_list}] for s in [{scaler_list}]]"
            )),
            Value::List(expected_bits),
        );
        let mut expected_pairs = Vec::new();
        for run in &currents {
            for hold in &currents {
                let (g, i, h) = helper.calc_current(*run, Some(*hold)).expect("calc_current");
                expected_pairs.push(Value::Tuple(vec![Value::int(g), Value::int(i), Value::int(h)]));
            }
        }
        check(
            &format!("calc_current rref={rref}"),
            py.eval(&format!(
                "[w.h._calc_current(r, h) for r in [{expr_list}] for h in [{expr_list}]]"
            )),
            Value::List(expected_pairs),
        );
        check(
            &format!("calc_current_from_field rref={rref}"),
            py.eval("[w.h._calc_current_from_field(f) for f in ['irun', 'ihold']]"),
            Value::List(
                ["irun", "ihold"]
                    .iter()
                    .map(|f| Value::float(helper.calc_current_from_field(f).expect("from field")))
                    .collect(),
            ),
        );
    }
}

#[test]
fn current_helper_set_current_matches_python() {
    let Some(mut py) = oracle() else { return };
    let values: [(&str, &str); 3] = [
        ("rref", "12000.0"),
        ("run_current", "0.8"),
        ("hold_current", "0.4"),
    ];
    let rust = rust_helper(&values);
    let helper = rust.result.as_ref().expect("helper");
    py_make_helper(&mut py, "w", &values);
    let calls: [(f64, Option<f64>, f64); 9] = [
        (1.0, Some(0.5), 42.0),
        (0.05, Some(0.05), 1.5),
        (1.2, Some(2.0), 0.0),
        (2.1, Some(2.1), 100.25),
        (0.8, Some(0.4), 7.0),
        (0.0001, Some(0.0001), 8.0),
        (1.4, Some(0.1), 9.5),
        (1.0, None, 11.0),
        (f64::NAN, Some(0.4), 10.0),
    ];
    for (index, (run, hold, print_time)) in calls.iter().enumerate() {
        let rust_result = helper.set_current(*run, *hold, *print_time);
        let expected = py.exec(&format!(
            "w.h.set_current({}, {}, {})",
            pyf(*run),
            hold.map_or_else(|| "None".to_string(), pyf),
            pyf(*print_time)
        ));
        // An exception raised by Python comes back as the oracle error and is
        // compared with the Rust `Err`; the state check below follows.
        match (&expected, &rust_result) {
            (Ok(_), Ok(())) => {}
            (Err(e), Err(r)) => {
                assert_eq!(e.class_name, r.class_name(), "set_current #{index} error class");
                assert_eq!(e.message, r.message(), "set_current #{index} error message");
            }
            (Ok(_), Err(r)) => panic!("set_current #{index}: Python succeeded, Rust raised {r}"),
            (Err(e), Ok(())) => panic!(
                "set_current #{index}: Python raised {}: {}, Rust succeeded",
                e.class_name, e.message
            ),
        }
        check(
            &format!("set_current #{index} ({run}, {hold:?}, {print_time})"),
            py.eval("snapshot(w)"),
            rust.snapshot(),
        );
    }
}

// ---------------------------------------------------------------------------
// load_config_prefix: transport selection and ordering.
// ---------------------------------------------------------------------------

fn rust_load(values: &[(&str, &str)]) -> Value {
    let printer = FakePrinter::new();
    let pins_log = new_log();
    printer
        .add_object("pins", Arc::new(FakePins { log: pins_log.clone() }))
        .expect("add pins");
    let config = FakeConfig::new(printer.clone() as Printer, values);
    let cfg: Config = config.clone();
    let result = load_config_prefix(&cfg);
    Value::Tuple(vec![
        error_value(result.as_ref().err()),
        strs(config.log.cloned()),
        strs(pins_log.cloned()),
    ])
}

#[test]
fn load_config_prefix_selects_transport_like_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("spi", vec![("cs_pin", "PA4")]),
        ("spi without cs_pin", vec![]),
        ("uart", vec![("uart_pin", "PA10")]),
        ("uart with empty pin", vec![("uart_pin", "")]),
        ("uart and cs_pin", vec![("uart_pin", "PA10"), ("cs_pin", "PA4")]),
        ("chain_length 1", vec![("chain_length", "1"), ("cs_pin", "PA4")]),
        ("chain_length abc", vec![("chain_length", "abc"), ("cs_pin", "PA4")]),
        ("chain_length 2", vec![("chain_length", "2"), ("cs_pin", "PA4")]),
        ("chain_length 2 without cs_pin", vec![("chain_length", "2")]),
        (
            "uart with tx_pin",
            vec![("uart_pin", "PA10"), ("tx_pin", "PA11"), ("uart_address", "9")],
        ),
    ];
    for (label, values) in &cases {
        let expected = py.eval(&format!("run_load({})", py_dict(values)));
        check(&format!("load_config_prefix {label}"), expected, rust_load(values));
    }
}

#[test]
fn module_factory_has_only_load_config_prefix() {
    let factory = module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

// ---------------------------------------------------------------------------
// TMC2240 construction over the SPI path's helpers (transport replaced).
// ---------------------------------------------------------------------------

fn py_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(i) => Value::int(i64::try_from(i.inner()).expect("small int")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Dict(map) => Value::Dict(
            map.iter()
                .map(|(k, v)| (py_to_value(k), py_to_value(v)))
                .collect(),
        ),
        other => panic!("unexpected status value {other:?}"),
    }
}

/// `TMC2240.assemble` (everything `TMC2240.__init__` does after choosing
/// the transport) against the printer fakes of `SETUP`.
fn rust_build(values: &[(&str, &str)]) -> Value {
    let printer = FakePrinter::new();
    let pins_log = new_log();
    let log = printer.log.clone();
    let objects: Vec<(&str, ObjectRef)> = vec![
        ("pins", Arc::new(FakePins { log: pins_log.clone() })),
        ("gcode", Arc::new(FakeGcode { log: log.clone() })),
        ("webhooks", Arc::new(FakeWebhooks { log: log.clone() })),
        ("heaters", Arc::new(FakeHeaters { log: log.clone() })),
        ("stepper_enable", Arc::new(FakeStepperEnable)),
    ];
    for (name, object) in objects {
        printer.add_object(name, object).expect("add object");
    }
    let config = FakeConfig::new(printer.clone() as Printer, values);
    let cfg: Config = config.clone();
    // `TMC2240.__init__` reads `uart_pin` before it chooses the transport.
    cfg.get(
        "uart_pin",
        GetStrOpts {
            default: Some(None),
            ..Default::default()
        },
    )
    .expect("uart_pin");
    let mcu_tmc = FakeMcuTmc::new();
    let fields = mcu_tmc.fields.clone();
    let result = TMC2240::assemble(&cfg, fields.clone(), mcu_tmc.clone());
    let (status, phase) = match &result {
        Ok(obj) => {
            let (offset, phases) = obj.get_phase_offset();
            (
                py_to_value(&obj.get_status(0.0).unwrap().expect("TMC2240 has a status")),
                Value::Tuple(vec![
                    offset.map(Value::int).unwrap_or(Value::Null),
                    Value::int(phases),
                ]),
            )
        }
        Err(_) => (Value::Null, Value::Null),
    };
    Value::Tuple(vec![
        error_value(result.as_ref().err()),
        strs(config.log.cloned()),
        strs(log.cloned()),
        strs(pins_log.cloned()),
        registers_pairs(&fields),
        status,
        phase,
    ])
}

#[test]
fn tmc2240_build_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("defaults", vec![("run_current", "0.8"), ("microsteps", "16")]),
        (
            "rref and hold current",
            vec![
                ("run_current", "0.8"),
                ("hold_current", "0.5"),
                ("rref", "15000"),
                ("microsteps", "32"),
            ],
        ),
        (
            "hold above run",
            vec![("run_current", "0.8"), ("hold_current", "1.2"), ("microsteps", "16")],
        ),
        ("tiny run current", vec![("run_current", "0.05"), ("microsteps", "16")]),
        ("range 2 run current", vec![("run_current", "1.5"), ("microsteps", "16")]),
        ("maximum run current", vec![("run_current", "2.12"), ("microsteps", "16")]),
        (
            "driver options",
            vec![
                ("run_current", "0.8"),
                ("microsteps", "64"),
                ("interpolate", "false"),
                ("diag0_pin", "PA1"),
                ("driver_TOFF", "7"),
                ("driver_SGT", "-5"),
                ("driver_MULTISTEP_FILT", "0"),
                ("driver_PWM_AUTOSCALE", "False"),
                ("driver_SG4_THRS", "12"),
                ("driver_TPOWERDOWN", "100"),
                ("driver_IHOLDDELAY", "9"),
                ("driver_SLOPE_CONTROL", "2"),
            ],
        ),
        ("missing run_current", vec![("microsteps", "16")]),
        ("run_current above max", vec![("run_current", "3.0"), ("microsteps", "16")]),
        ("run_current zero", vec![("run_current", "0"), ("microsteps", "16")]),
        (
            "hold_current zero",
            vec![("run_current", "0.8"), ("hold_current", "0"), ("microsteps", "16")],
        ),
        (
            "rref below minimum",
            vec![("run_current", "0.8"), ("rref", "1000"), ("microsteps", "16")],
        ),
        (
            "rref above maximum",
            vec![("run_current", "0.8"), ("rref", "70000"), ("microsteps", "16")],
        ),
        (
            "rref not a number",
            vec![("run_current", "0.8"), ("rref", "abc"), ("microsteps", "16")],
        ),
        ("missing microsteps", vec![("run_current", "0.8")]),
        ("invalid microsteps", vec![("run_current", "0.8"), ("microsteps", "3")]),
        (
            "driver field above maximum",
            vec![("run_current", "0.8"), ("microsteps", "16"), ("driver_TOFF", "16")],
        ),
        (
            "signed driver field below minimum",
            vec![("run_current", "0.8"), ("microsteps", "16"), ("driver_SGT", "-65")],
        ),
        (
            "boolean driver field not parsable",
            vec![("run_current", "0.8"), ("microsteps", "16"), ("driver_PWM_AUTOGRAD", "maybe")],
        ),
    ];
    for (label, values) in &cases {
        let expected = py.eval(&format!("run_build({})", py_dict(values)));
        check(&format!("TMC2240 build {label}"), expected, rust_build(values));
    }
}
