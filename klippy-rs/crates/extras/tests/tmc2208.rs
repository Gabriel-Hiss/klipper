//! Parity tests for `extras::tmc2208` (port of `klippy/extras/tmc2208.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): the register/field tables, the
//! frequency constant, the merged field formatters and `read_translate`
//! are evaluated on the Python side and checked against the Rust values
//! exactly (floats by bits via the recording value model).
//!
//! `tmc2208_build_matches_python` runs the real `TMC2208.__init__` against
//! the real `Tmc2208::assemble` (and `Tmc2208::new`, which feeds it). Both
//! sides build the real UART transport (`tmc_uart.MCU_TMC_uart` /
//! `McuTmcUart`) over fakes of the printer services, the pins object and
//! the MCU; the Python transport class is only wrapped to capture the
//! `FieldHelper` it receives. `stepper.parse_step_distance` is the real
//! function on both sides (the Rust tool head fake hands out the core
//! port). Compared in order: every config read (option, default, bounds),
//! the printer calls (`load_object`, `add_object`, event handlers, mux
//! commands, the bulk endpoint), pin lookups and chip registrations, MCU
//! calls, the `FieldHelper` register map (entries and insertion order),
//! `get_status`, `get_phase_offset` and the raised error (class and
//! message).

use std::sync::Arc;
use extras::heaters::{Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi};
use extras::tmc::{FieldHelper, TMCCommandHelperApi};
use extras::tmc2208::{
    field_formatters, fields_table, read_registers, read_translate, registers, signed_fields,
    Tmc2208, TMC_FREQUENCY,
};
use extras::tmc_uart::McuTmcUart;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueryWrapper, CommandQueue,
    CommandWrapper, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{Rail, RailOpts, Stepper, StepperFactory, StepperFactoryApi, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, LookaheadCallback, PrinterExtruder, ToolHeadApi,
};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::text::{float_repr, str_repr};
use pycompat::{PyInt, PyValue};
use pyoracle::OracleResult;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

const SETUP: &str = r#"
import extras.tmc2208 as tmc2208
import extras.tmc as tmc

def bare_object():
    fh = tmc.FieldHelper(tmc2208.Fields, tmc2208.SignedFields, tmc2208.FieldFormatters)
    obj = tmc2208.TMC2208.__new__(tmc2208.TMC2208)
    obj.fields = fh
    return obj
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("tmc2208 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping tmc2208 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.tmc2208".to_string(),
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

fn fields() -> Arc<FieldHelper> {
    FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None)
}

#[test]
fn tmc_frequency_matches_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("tmc2208.TMC_FREQUENCY");
    check(
        "tmc_frequency_matches_python",
        expected,
        Value::float(TMC_FREQUENCY),
    );
}

#[test]
fn registers_table_matches_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("[[k, v] for k, v in tmc2208.Registers.items()]");
    let actual = Value::List(
        registers()
            .iter()
            .map(|(k, v)| Value::List(vec![Value::str(k), Value::int(*v)]))
            .collect(),
    );
    check("registers_table_matches_python", expected, actual);
}

#[test]
fn read_registers_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("list(tmc2208.ReadRegisters)");
    let actual = Value::List(
        read_registers()
            .iter()
            .map(|r| Value::str(r))
            .collect(),
    );
    check("read_registers_match_python", expected, actual);
}

#[test]
fn signed_fields_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("list(tmc2208.SignedFields)");
    let actual = Value::List(
        signed_fields()
            .iter()
            .map(|f| Value::str(f))
            .collect(),
    );
    check("signed_fields_match_python", expected, actual);
}

#[test]
fn fields_table_matches_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("[(reg, list(fields.items())) for reg, fields in tmc2208.Fields.items()]");
    let actual = Value::List(
        fields_table()
            .iter()
            .map(|(reg, fields)| {
                Value::Tuple(vec![
                    Value::str(reg),
                    Value::List(
                        fields
                            .iter()
                            .map(|(name, mask)| {
                                Value::Tuple(vec![Value::str(name), Value::int(*mask)])
                            })
                            .collect(),
                    ),
                ])
            })
            .collect(),
    );
    check("fields_table_matches_python", expected, actual);
}

#[test]
fn field_formatter_keys_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("list(tmc2208.FieldFormatters)");
    let actual = Value::List(
        field_formatters()
            .keys()
            .map(|name| Value::str(name))
            .collect(),
    );
    check("field_formatter_keys_match_python", expected, actual);
}

#[test]
fn field_formatters_match_python() {
    let Some(mut py) = oracle() else { return };
    let formatters = field_formatters();
    // Out-of-range `sel_a` values panic in Rust the same way Python raises
    // IndexError, so only values Python evaluates are compared here.
    let cases: &[(&str, &[i64])] = &[
        ("sel_a", &[-2, -1, 0, 1]),
        ("s2vsa", &[-1, 0, 1, 2]),
        ("s2vsb", &[-1, 0, 1, 2]),
    ];
    for (name, values) in cases {
        let to_call = formatters.get(*name).map(Arc::clone);
        let Some(to_call) = to_call else {
            panic!("field formatters: {name} missing");
        };
        for v in *values {
            let label = format!("field_formatters_match_python {name} {v}");
            let expected = py.eval(&format!("tmc2208.FieldFormatters[\"{name}\"]({v})"));
            check(&label, expected, Value::str(&to_call(*v)));
        }
    }
}

#[test]
fn read_translate_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fields = fields();
    let cases: &[(&str, i64)] = &[
        ("IOIN", 0x0000_0000),
        ("IOIN", 0x0000_0100),
        ("IOIN", 0x1234_5678),
        ("IOIN", 0x0000_01ff),
        ("IOIN", 0xdddd_dddd),
        ("IOIN", -1),
        ("IOIN", 0xffff_ffff),
        ("GCONF", 0x1234),
        ("DRV_STATUS", 0xdead_beef),
        ("IFCNT", 0),
        ("CHOPCONF", 0x8000_0000),
    ];
    for (reg_name, val) in cases {
        let label = format!("read_translate_matches_python {reg_name} {val:#x}");
        let expr = format!("bare_object().read_translate(\"{reg_name}\", {val})");
        let expected = py.eval(&expr);
        check(&label, expected, {
            let (name, v) = read_translate(&fields, reg_name, *val);
            Value::Tuple(vec![Value::str(&name), Value::int(v)])
        });
    }
}

#[test]
fn module_factory_shape_matches_python() {
    let factory = extras::tmc2208::module_factory();
    assert!(
        factory.load_config.is_none(),
        "tmc2208 has no load_config"
    );
    assert!(
        factory.load_config_prefix.is_some(),
        "tmc2208 exposes load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "tmc2208 has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "tmc2208 has no add_printer_objects"
    );
}

// ---------------------------------------------------------------------------
// TMC2208 construction: `TMC2208.__init__` against `Tmc2208::assemble`.
// ---------------------------------------------------------------------------

const SECTION: &str = "tmc2208 stepper_x";

/// Python fakes for the construction test. `MCU_TMC_uart` stays the real
/// class; `run_build` only wraps it to capture the `FieldHelper` argument so
/// the register map is available when construction raises.
const BUILD_SETUP: &str = r#"
import configparser
import extras.tmc_uart as tmc_uart
import extras.tmc2208 as tmc2208

SECTION = "tmc2208 stepper_x"

_sentinel = object()

def _r(v):
    return '-' if v is None else repr(v)

def _d(default):
    return 'required' if default is _sentinel else repr(default)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, values, log):
        self._printer = printer
        self.section = section
        self.values = values
        self.log = log
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
    def _unparsable(self, option):
        return configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
    def _bound(self, option, what, bound):
        return configparser.Error("Option '%s' in section '%s' must %s %s" % (option, self.section, what, bound))
    def get(self, option, default=_sentinel, note_valid=True):
        self.log.append("get|%s|%s" % (option, _d(default)))
        found, v = self._lookup(option, default)
        return v
    def getint(self, option, default=_sentinel, minval=None, maxval=None, note_valid=True):
        self.log.append("getint|%s|%s|%s|%s" % (option, _d(default), _r(minval), _r(maxval)))
        found, v = self._lookup(option, default)
        if not found:
            return v
        try:
            v = int(v)
        except Exception:
            raise self._unparsable(option)
        if minval is not None and v < minval:
            raise self._bound(option, "have minimum of", minval)
        if maxval is not None and v > maxval:
            raise self._bound(option, "have maximum of", maxval)
        return v
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        self.log.append("getfloat|%s|%s|%s|%s|%s|%s" % (option, _d(default), _r(minval), _r(maxval), _r(above), _r(below)))
        found, v = self._lookup(option, default)
        if not found:
            return v
        try:
            v = float(v)
        except Exception:
            raise self._unparsable(option)
        if minval is not None and v < minval:
            raise self._bound(option, "have minimum of", minval)
        if maxval is not None and v > maxval:
            raise self._bound(option, "have maximum of", maxval)
        if above is not None and v <= above:
            raise self._bound(option, "be above", above)
        if below is not None and v >= below:
            raise self._bound(option, "be below", below)
        return v
    def getboolean(self, option, default=_sentinel, note_valid=True):
        # set_config_field passes int defaults (0, 1) as well as bools; the
        # log renders them as bools since the stored field value is the same.
        if default is _sentinel:
            shown = 'required'
        elif default is None:
            shown = 'None'
        else:
            shown = repr(bool(default))
        self.log.append("getboolean|%s|%s" % (option, shown))
        found, v = self._lookup(option, default)
        if not found:
            return v
        if v.lower() in ('1', 'yes', 'true', 'on'):
            return True
        if v.lower() in ('0', 'no', 'false', 'off'):
            return False
        raise self._unparsable(option)
    def getchoice(self, option, choices, default=_sentinel, note_valid=True):
        self.log.append("getchoice|%s|%s" % (option, _d(default)))
        found, v = self._lookup(option, default)
        if not found:
            return choices[v]
        try:
            v = int(v)
        except Exception:
            raise self._unparsable(option)
        if v not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (v, option, self.section))
        return choices[v]
    def getlist(self, option, default=_sentinel, sep=',', count=None, note_valid=True):
        self.log.append("getlist|%s|%s" % (option, _d(default)))
        found, v = self._lookup(option, default)
        if found:
            raise Exception("getlist values are not used")
        return v
    def getlists(self, option, default=_sentinel, seps=(',',), count=None, parser=str, note_valid=True):
        self.log.append("getlists|%s|%s" % (option, _d(default)))
        found, v = self._lookup(option, default)
        if found:
            raise Exception("getlists values are not used")
        return v
    def getsection(self, section):
        return FakeConfig(self._printer, section, self.values, self.log)
    def has_section(self, section):
        return section == 'stepper_x'

class FakeReactor:
    def mutex(self, is_locked=False):
        return object()

class FakeMcu:
    def __init__(self, printer, log):
        self._printer = printer
        self.log = log
        self._next_oid = 0
    def get_printer(self):
        return self._printer
    def create_oid(self):
        oid = self._next_oid
        self._next_oid += 1
        self.log.append("create_oid|%d" % (oid,))
        return oid
    def alloc_command_queue(self):
        self.log.append("alloc_command_queue")
        return object()
    def register_config_callback(self, cb):
        self.log.append("register_config_callback")

class _PinsError(Exception):
    pass
_PinsError.__name__ = "error"

class FakePins:
    error = _PinsError
    def __init__(self, mcu, log):
        self.mcu = mcu
        self.log = log
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        self.log.append("lookup_pin|%r|%r|%r|%r" % (pin_desc, can_invert, can_pullup, share_type))
        return {'chip': self.mcu, 'chip_name': 'mcu', 'pin': pin_desc,
                'invert': 0, 'pullup': 0, 'share_type': share_type}
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
        self.log = []
        self.pins_log = []
        self.mcu_log = []
        self.reactor = FakeReactor()
        self.mcu = FakeMcu(self, self.mcu_log)
        self.objects = {
            "pins": FakePins(self.mcu, self.pins_log),
            "gcode": FakeGcode(self.log),
            "webhooks": FakeWebhooks(self.log),
            "heaters": FakeHeaters(self.log),
            "stepper_enable": FakeStepperEnable(),
        }
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
    def add_object(self, name, obj):
        self.log.append("add_object|%s" % (name,))
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.log.append("register_event_handler|%s" % (event,))

def run_build(values, regs_on_error):
    p = FakePrinter()
    cfg = FakeConfig(p, SECTION, values, [])
    created = []
    real_uart = tmc_uart.MCU_TMC_uart
    def recording_uart(config, name_to_reg, fields, max_addr, tmc_frequency):
        created.append(fields)
        return real_uart(config, name_to_reg, fields, max_addr, tmc_frequency)
    tmc_uart.MCU_TMC_uart = recording_uart
    err = obj = None
    try:
        obj = tmc2208.TMC2208(cfg)
    except Exception as e:
        err = (type(e).__name__, str(e))
    finally:
        tmc_uart.MCU_TMC_uart = real_uart
    regs = None
    if created and (obj is not None or regs_on_error):
        regs = list(created[0].registers.items())
    status = None if obj is None else obj.get_status(0.)
    phase = None if obj is None else obj.get_phase_offset()
    return (err, cfg.log, p.log, p.pins_log, p.mcu_log, regs, status, phase)
"#;

fn build_oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(BUILD_SETUP).expect("tmc2208 build fakes");
            Some(py)
        }
        None => {
            println!("skipping tmc2208 build oracle test: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
            None
        }
    }
}

type Log = Arc<GCell<Vec<String>>>;

fn new_log() -> Log {
    Arc::new(GCell::new(Vec::new()))
}

fn push(log: &Log, line: String) {
    log.borrow_mut().push(line);
}

struct FakePrinter {
    objects: GCell<Vec<(String, ObjectRef)>>,
    reactor: Reactor,
    /// `load_object`, `add_object` and `register_event_handler` calls plus the
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

    /// Seeds a fake service without logging (the Python fakes start with
    /// the same objects in their dict).
    fn insert(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().push((name.to_string(), obj));
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
        push(&self.log, format!("add_object|{name}"));
        self.insert(name, obj);
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
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().clone()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        push(&self.log, format!("load_object|{section}"));
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        push(&self.log, format!("register_event_handler|{event}"));
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
    fn new(printer: Printer, section: &str, values: &[(&str, &str)]) -> Arc<FakeConfig> {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            values: values
                .iter()
                .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
                .collect(),
            log: new_log(),
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
        push(&self.log, format!("get|{option}|{default}"));
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
        push(
            &self.log,
            format!(
                "getint|{option}|{default}|{}|{}",
                dash(opts.minval, |v| v.to_string()),
                dash(opts.maxval, |v| v.to_string())
            ),
        );
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
        push(
            &self.log,
            format!(
                "getfloat|{option}|{default}|{}|{}|{}|{}",
                dash(opts.minval, float_repr),
                dash(opts.maxval, float_repr),
                dash(opts.above, float_repr),
                dash(opts.below, float_repr)
            ),
        );
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
        let default = match opts.default {
            None => "required",
            Some(None) => "None",
            Some(Some(true)) => "True",
            Some(Some(false)) => "False",
        };
        push(&self.log, format!("getboolean|{option}|{default}"));
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
        push(&self.log, format!("getchoice|{option}|{default}"));
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
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        let default = match &opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(PyValue::Tuple(items))) if items.is_empty() => "()".to_string(),
            Some(Some(other)) => panic!("getlists default {other:?} is not used"),
        };
        push(&self.log, format!("getlists|{option}|{default}"));
        if self.lookup(option).is_some() {
            panic!("getlists values are not used");
        }
        match opts.default {
            None => Err(self.required(option)),
            Some(d) => Ok(d.unwrap_or(PyValue::None)),
        }
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        let default = match &opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(_)) => panic!("getlist list defaults are not used"),
        };
        push(&self.log, format!("getlist|{option}|{default}"));
        if self.lookup(option).is_some() {
            panic!("getlist values are not used");
        }
        match opts.default {
            None => Err(self.required(option)),
            Some(d) => Ok(d),
        }
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used")
    }
    /// `config.getsection(section)`: same options, same log.
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            values: self.values.clone(),
            log: self.log.clone(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        section == "stepper_x"
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("get_prefix_options is not used")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

/// The MCU behind every pin: logs the calls `MCU_TMC_uart_bitbang` makes
/// while it is built.
struct FakeMcu {
    printer: Printer,
    next_oid: GCell<i64>,
    log: Log,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        panic!("unused")
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        panic!("unused")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("unused")
    }
    fn create_oid(&self) -> KResult<i64> {
        let oid = *self.next_oid.borrow();
        *self.next_oid.borrow_mut() = oid + 1;
        push(&self.log, format!("create_oid|{oid}"));
        Ok(oid)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        push(&self.log, "register_config_callback".to_string());
        Ok(())
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
        push(&self.log, "alloc_command_queue".to_string());
        CommandQueue(0)
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
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("unused")
    }
}

/// Resolves every pin to `FakeMcu` (pin name unchanged, no pullup) after
/// logging the lookup.
struct FakePins {
    mcu: Mcu,
    log: Log,
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn opt_repr(v: Option<&str>) -> String {
    v.map(str_repr).unwrap_or_else(|| "None".to_string())
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        push(
            &self.log,
            format!(
                "lookup_pin|{}|{}|{}|{}",
                str_repr(pin_desc),
                py_bool(opts.can_invert),
                py_bool(opts.can_pullup),
                opt_repr(opts.share_type.as_deref())
            ),
        );
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used")
    }
    fn register_chip(&self, chip_name: &str, _chip: Chip) -> KResult<()> {
        push(&self.log, format!("register_chip|{chip_name}"));
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

/// Logs every `register_mux_command`; the other methods are not reached
/// while building.
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
        push(
            &self.log,
            format!(
                "register_mux_command|{cmd}|{key}|{}|{}",
                opt_repr(value),
                opt_repr(desc)
            ),
        );
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
        push(&self.log, format!("register_mux_endpoint|{path}|{key}|{}", str_repr(value)));
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

/// Logs `register_monitor` (`TMCErrorCheck` calls it only for drivers with
/// an `adc_temp` field).
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
        push(&self.log, format!("register_monitor|{}", config.get_name()));
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

/// The `stepper` module as seen through the tool head: `parse_step_distance`
/// is the core port of the function `TMCtstepHelper` calls directly in
/// Python (`tmc.py:695`).
struct FakeStepperFactory;

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper is not used")
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_rail is not used")
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_multi_rail is not used")
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail is not used")
    }
    fn parse_step_distance(
        &self,
        config: &Config,
        units_in_radians: Option<bool>,
        note_valid: bool,
    ) -> KResult<(f64, f64)> {
        klippy_core::stepper::parse_step_distance(config, units_in_radians, note_valid)
    }
}

/// The Rust `TMCtstepHelper` reaches `stepper.parse_step_distance` through
/// the tool head; nothing else is used.
struct FakeToolHead;

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used")
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used")
    }
    fn dwell(&self, _delay: f64) {
        panic!("dwell is not used")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used")
    }
    fn stepper_factory(&self) -> StepperFactory {
        Arc::new(FakeStepperFactory)
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

/// One printer with its fakes and a config over `values`, the Rust mirror
/// of `FakePrinter()` + `FakeConfig(p, SECTION, values, [])`.
struct World {
    printer: Arc<FakePrinter>,
    config: Arc<FakeConfig>,
    pins_log: Log,
    mcu_log: Log,
}

fn world(values: &[(&str, &str)]) -> World {
    let printer = FakePrinter::new();
    let log = printer.log.clone();
    let pins_log = new_log();
    let mcu_log = new_log();
    let mcu: Mcu = Arc::new(FakeMcu {
        printer: printer.clone() as Printer,
        next_oid: GCell::new(0),
        log: mcu_log.clone(),
    });
    printer.insert("pins", Arc::new(FakePins { mcu, log: pins_log.clone() }));
    printer.insert("gcode", Arc::new(FakeGcode { log: log.clone() }));
    printer.insert("webhooks", Arc::new(FakeWebhooks { log: log.clone() }));
    printer.insert("heaters", Arc::new(FakeHeaters { log }));
    printer.insert("stepper_enable", Arc::new(FakeStepperEnable));
    // Python has no tool head object: `TMCtstepHelper` calls
    // `stepper.parse_step_distance` directly; the Rust port reaches the
    // same function through `toolhead.stepper_factory()`.
    printer.insert("toolhead", Arc::new(FakeToolHead));
    let config = FakeConfig::new(printer.clone() as Printer, SECTION, values);
    World { printer, config, pins_log, mcu_log }
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

/// The `run_build` tuple for one Rust construction outcome.
fn build_value(w: &World, result: &KResult<Tmc2208>, regs: Value) -> Value {
    let (status, phase) = match result {
        Ok(obj) => {
            let (offset, phases) = obj.get_phase_offset();
            (
                py_to_value(&obj.get_status(0.0).expect("TMC2208 has a status")),
                Value::Tuple(vec![offset.map(Value::int).unwrap_or(Value::Null), Value::int(phases)]),
            )
        }
        Err(_) => (Value::Null, Value::Null),
    };
    Value::Tuple(vec![
        error_value(result.as_ref().err()),
        strs(w.config.log.cloned()),
        strs(w.printer.log.cloned()),
        strs(w.pins_log.cloned()),
        strs(w.mcu_log.cloned()),
        regs,
        status,
        phase,
    ])
}

/// `TMC2208.__init__` split as the port splits it: the real UART transport
/// built over the module tables (`tmc2208.py:188-190`), then
/// `Tmc2208::assemble`. The register map is kept on error.
fn rust_build_assemble(values: &[(&str, &str)]) -> Value {
    let w = world(values);
    let cfg: Config = w.config.clone();
    let fields = FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None);
    let result = McuTmcUart::new(&cfg, registers(), fields.clone(), 0, TMC_FREQUENCY)
        .and_then(|mcu_tmc| Tmc2208::assemble(&cfg, fields.clone(), Arc::new(mcu_tmc)));
    build_value(&w, &result, registers_pairs(&fields))
}

/// `Tmc2208::new`, whose `FieldHelper` is only reachable on success.
fn rust_build_new(values: &[(&str, &str)]) -> Value {
    let w = world(values);
    let cfg: Config = w.config.clone();
    let result = Tmc2208::new(&cfg);
    let regs = match &result {
        Ok(obj) => registers_pairs(obj.fields()),
        Err(_) => Value::Null,
    };
    build_value(&w, &result, regs)
}

#[test]
fn tmc2208_build_matches_python() {
    let Some(mut py) = build_oracle() else { return };
    let base: Vec<(&'static str, &'static str)> =
        vec![("uart_pin", "PA0"), ("run_current", "0.8"), ("microsteps", "16")];
    let with = |extra: &[(&'static str, &'static str)]| -> Vec<(&'static str, &'static str)> {
        let mut v: Vec<(&str, &str)> = base
            .iter()
            .filter(|(k, _)| !extra.iter().any(|(e, _)| e == k))
            .copied()
            .collect();
        v.extend_from_slice(extra);
        v
    };
    let without = |option: &str| -> Vec<(&'static str, &'static str)> {
        base.iter().filter(|(k, _)| *k != option).copied().collect()
    };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("defaults", base.clone()),
        ("tx_pin and uart_address 0", with(&[("tx_pin", "PA1"), ("uart_address", "0")])),
        (
            "every driver option",
            with(&[
                ("driver_MULTISTEP_FILT", "false"),
                ("driver_TOFF", "4"),
                ("driver_HSTRT", "7"),
                ("driver_HEND", "11"),
                ("driver_TBL", "1"),
                ("driver_IHOLDDELAY", "15"),
                ("driver_PWM_OFS", "200"),
                ("driver_PWM_GRAD", "50"),
                ("driver_PWM_FREQ", "3"),
                ("driver_PWM_AUTOSCALE", "0"),
                ("driver_PWM_AUTOGRAD", "off"),
                ("driver_FREEWHEEL", "2"),
                ("driver_PWM_REG", "15"),
                ("driver_PWM_LIM", "0"),
                ("driver_TPOWERDOWN", "255"),
            ]),
        ),
        (
            "stealthchop_threshold",
            with(&[("rotation_distance", "40"), ("stealthchop_threshold", "100")]),
        ),
        (
            "stealthchop_threshold full steps and 256 microsteps",
            with(&[
                ("rotation_distance", "8"),
                ("full_steps_per_rotation", "400"),
                ("microsteps", "256"),
                ("stealthchop_threshold", "0.5"),
            ]),
        ),
        (
            "stealthchop_threshold zero",
            with(&[("rotation_distance", "40"), ("stealthchop_threshold", "0")]),
        ),
        (
            "stealthchop_threshold tiny clamps tpwmthrs",
            with(&[("rotation_distance", "40"), ("stealthchop_threshold", "0.0001")]),
        ),
        (
            "stealthchop_threshold negative",
            with(&[("rotation_distance", "40"), ("stealthchop_threshold", "-1")]),
        ),
        (
            "stealthchop_threshold without rotation_distance",
            with(&[("stealthchop_threshold", "100")]),
        ),
        (
            "stealthchop_threshold with invalid full steps",
            with(&[
                ("rotation_distance", "40"),
                ("full_steps_per_rotation", "201"),
                ("stealthchop_threshold", "100"),
            ]),
        ),
        ("interpolate false", with(&[("interpolate", "false"), ("microsteps", "256")])),
        ("interpolate unparsable", with(&[("interpolate", "maybe")])),
        ("microsteps 1", with(&[("microsteps", "1")])),
        (
            "hold_current and sense_resistor",
            with(&[("hold_current", "0.5"), ("sense_resistor", "0.150")]),
        ),
        ("hold_current above run_current", with(&[("hold_current", "1.2")])),
        ("tiny run_current", with(&[("run_current", "0.05")])),
        ("missing run_current", without("run_current")),
        ("run_current above maximum", with(&[("run_current", "3.0")])),
        ("run_current zero", with(&[("run_current", "0")])),
        ("hold_current zero", with(&[("hold_current", "0")])),
        ("missing uart_pin", without("uart_pin")),
        ("uart_address above maximum", with(&[("uart_address", "1")])),
        ("uart_address negative", with(&[("uart_address", "-1")])),
        ("driver field above maximum", with(&[("driver_TOFF", "16")])),
        ("driver field negative", with(&[("driver_TPOWERDOWN", "-1")])),
        ("driver field not a number", with(&[("driver_PWM_GRAD", "abc")])),
        ("boolean driver field not parsable", with(&[("driver_PWM_AUTOGRAD", "maybe")])),
        ("boolean driver field as 2", with(&[("driver_MULTISTEP_FILT", "2")])),
        ("missing microsteps", without("microsteps")),
        ("invalid microsteps", with(&[("microsteps", "3")])),
        ("microsteps not a number", with(&[("microsteps", "x")])),
    ];
    for (label, values) in &cases {
        let expected = py.eval(&format!("run_build({}, True)", py_dict(values)));
        check(&format!("TMC2208 assemble {label}"), expected, rust_build_assemble(values));
        let expected = py.eval(&format!("run_build({}, False)", py_dict(values)));
        check(&format!("TMC2208 new {label}"), expected, rust_build_new(values));
    }
}
