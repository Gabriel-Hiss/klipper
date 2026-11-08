//! Parity tests for `klippy/extras/mcp4728.py`.
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `mcp4728` object is
//! built on both sides with minimal Python fakes for the config wrapper
//! (`FakeConfig`, mirroring `configfile.ConfigWrapper.getfloat` error text
//! and ordering) and the bus (`FakeI2C`, replacing
//! `MCU_I2C_from_config`), fed identical option dicts, and the constructor
//! result is compared exactly: the `getfloat` call log (option, default,
//! min/max, above), the `i2c_write` byte payloads, and the raised error
//! (`class: message`). `set_dac` byte packing is compared on a small grid
//! too. There are no recordings under `tests/fixtures/recordings/` for
//! this unit; expected values come from the oracle alone.
//!
//! The bus I2C handle is faked on both sides: config-option reads inside
//! `MCU_I2C_from_config` and its queueing belong to the `bus` port.
//!
//! Integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test mcp4728
//! ```

use std::sync::Arc;
use kos::sync::Mutex;

use extras::bus::McuI2cApi;
use extras::mcp4728::{load_config_prefix, module_factory, Mcp4728};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper,
    ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, QuerySendOpts,
    SendOpts, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use pycompat::text::{float_from_string, float_repr};
use pycompat::PyValue;
use reactor::Reactor;

use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.mcp4728

class error(Exception):
    pass

class FakeConfig:
    def __init__(self, section, opts):
        self.section = section
        self.opts = dict(opts)
        self.call_log = []
    def get_printer(self):
        return None
    def get_name(self):
        return self.section
    def getfloat(self, option, default=None, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        self.call_log.append([option, default, minval, maxval, above])
        if option not in self.opts:
            return default
        try:
            v = float(self.opts[option])
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

class FakeI2C:
    def __init__(self, fail_after):
        self.fail_after = fail_after
        self.writes = []
    def i2c_write(self, data):
        if len(self.writes) >= self.fail_after:
            raise error("simulated i2c failure")
        self.writes.append(list(data))
    def get_i2c_address(self):
        return 0x60

def construct(section, opts, fail_after):
    cfg = FakeConfig(section, opts)
    i2c = FakeI2C(fail_after)
    extras.bus.MCU_I2C_from_config = lambda config, default_addr=None, _i2c=i2c: _i2c
    try:
        extras.mcp4728.mcp4728(cfg)
        return (None, i2c.writes, cfg.call_log)
    except Exception as e:
        return ("%s: %s" % (type(e).__name__, str(e)), i2c.writes, cfg.call_log)
"#;

const SECTION: &str = "mcp4728 test";

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("mcp4728 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping mcp4728 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

/// One constructor scenario: config options as raw strings (as a real
/// config file would hold them), plus how many `i2c_write` calls the fake
/// bus tolerates before raising `error`.
struct Scenario {
    label: &'static str,
    opts: &'static [(&'static str, &'static str)],
    fail_after: usize,
}

const SCENARIOS: &[Scenario] = &[
    Scenario { label: "no options", opts: &[], fail_after: usize::MAX },
    Scenario { label: "scale only", opts: &[("scale", "2.5")], fail_after: usize::MAX },
    Scenario {
        label: "all channels at bounds",
        opts: &[
            ("scale", "1.0"),
            ("channel_a", "0.0"),
            ("channel_b", "0.25"),
            ("channel_c", "0.5"),
            ("channel_d", "1.0"),
        ],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "rounding with scale 3",
        opts: &[
            ("scale", "3.0"),
            ("channel_a", "1.0"),
            ("channel_b", "2.0"),
            ("channel_c", "3.0"),
            ("channel_d", "1.5"),
        ],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "default scale",
        opts: &[("channel_a", "0.5"), ("channel_c", "1.0")],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "negative zero channel",
        opts: &[("scale", "1.0"), ("channel_a", "-0.0")],
        fail_after: usize::MAX,
    },
    Scenario { label: "scale zero", opts: &[("scale", "0.0")], fail_after: usize::MAX },
    Scenario { label: "scale negative", opts: &[("scale", "-1.0")], fail_after: usize::MAX },
    Scenario { label: "channel below min", opts: &[("channel_a", "-0.5")], fail_after: usize::MAX },
    Scenario {
        label: "channel above max",
        opts: &[("scale", "1.0"), ("channel_b", "1.5")],
        fail_after: usize::MAX,
    },
    Scenario { label: "bad scale string", opts: &[("scale", "abc")], fail_after: usize::MAX },
    Scenario { label: "bad channel string", opts: &[("channel_c", "x")], fail_after: usize::MAX },
    Scenario {
        label: "nan scale",
        opts: &[("scale", "nan"), ("channel_a", "1.0")],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "inf scale finite channel",
        opts: &[("scale", "inf"), ("channel_a", "1.0")],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "inf scale inf channel",
        opts: &[("scale", "inf"), ("channel_a", "inf")],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "huge scale overflow",
        opts: &[("scale", "1e308"), ("channel_a", "1e308")],
        fail_after: usize::MAX,
    },
    Scenario {
        label: "i2c failure on first write",
        opts: &[("channel_a", "1.0")],
        fail_after: 0,
    },
    Scenario {
        label: "i2c failure on second write",
        opts: &[("channel_a", "0.0"), ("channel_b", "1.0")],
        fail_after: 1,
    },
];

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct GetfloatCall {
    option: String,
    default: Option<Option<f64>>,
    minval: Option<f64>,
    maxval: Option<f64>,
    above: Option<f64>,
}

struct FakeConfig {
    section: String,
    printer: Printer,
    opts: Vec<(String, String)>,
    call_log: Mutex<Vec<GetfloatCall>>,
}

impl FakeConfig {
    fn new(section: &str, opts: &[(&str, &str)]) -> FakeConfig {
        FakeConfig {
            section: section.to_string(),
            printer: Arc::new(FakePrinter),
            opts: opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            call_log: Mutex::new(Vec::new()),
        }
    }

    fn result_value(&self) -> Value {
        Value::List(
            self.call_log
                .lock()
                .iter()
                .map(|c| {
                    let default = match c.default {
                        Some(Some(v)) => Value::float(v),
                        _ => Value::Null,
                    };
                    let bound = |b: Option<f64>| match b {
                        Some(v) => Value::float(v),
                        None => Value::Null,
                    };
                    Value::List(vec![
                        Value::str(&c.option),
                        default,
                        bound(c.minval),
                        bound(c.maxval),
                        bound(c.above),
                    ])
                })
                .collect(),
        )
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("fake config: unused get")
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("fake config: unused getint")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.call_log.lock().push(GetfloatCall {
            option: option.to_string(),
            default: opts.default,
            minval: opts.minval,
            maxval: opts.maxval,
            above: opts.above,
        });
        let Some((_, raw)) = self.opts.iter().find(|(k, _)| k == option) else {
            return Ok(opts.default.and_then(|d| d));
        };
        let v = match float_from_string(raw) {
            Ok(v) => v,
            Err(_) => {
                return Err(KlippyError::Config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.section
                )));
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    float_repr(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    float_repr(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.section,
                    float_repr(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("fake config: unused getboolean")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("fake config: unused getchoice")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("fake config: unused getlists")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("fake config: unused getlist")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("fake config: unused getintlist")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("fake config: unused getfloatlist")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("fake config: unused getsection")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("fake config: unused has_section")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("fake config: unused get_prefix_sections")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("fake config: unused get_prefix_options")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        panic!("fake config: unused deprecate")
    }
}

struct FakePrinter;

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        panic!("fake printer: unused get_start_args")
    }
    fn get_reactor(&self) -> Reactor {
        panic!("fake printer: unused get_reactor")
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("fake printer: unused get_state_message")
    }
    fn is_shutdown(&self) -> bool {
        panic!("fake printer: unused is_shutdown")
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("fake printer: unused update_error_msg")
    }
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("fake printer: unused add_object")
    }
    fn lookup_object(&self, _name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        panic!("fake printer: unused lookup_object")
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("fake printer: unused lookup_objects")
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("fake printer: unused load_object")
    }
    fn run(&self) -> Option<String> {
        panic!("fake printer: unused run")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("fake printer: unused set_rollover_info")
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("fake printer: unused invoke_shutdown")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("fake printer: unused invoke_async_shutdown")
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {
        panic!("fake printer: unused register_event_handler")
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("fake printer: unused send_event")
    }
    fn request_exit(&self, _result: &str) {
        panic!("fake printer: unused request_exit")
    }
}

/// Never called: only satisfies `McuI2cApi::get_mcu` (the port never asks
/// its I2C handle for the MCU).
struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        panic!("fake mcu: unused")
    }
    fn get_printer(&self) -> Printer {
        panic!("fake mcu: unused")
    }
    fn is_fileoutput(&self) -> bool {
        panic!("fake mcu: unused")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("fake mcu: unused")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("fake mcu: unused")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("fake mcu: unused")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("fake mcu: unused")
    }
    fn request_move_queue_slot(&self) {
        panic!("fake mcu: unused")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("fake mcu: unused")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("fake mcu: unused")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("fake mcu: unused")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("fake mcu: unused")
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("fake mcu: unused")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("fake mcu: unused")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("fake mcu: unused")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("fake mcu: unused")
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("fake mcu: unused")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("fake mcu: unused")
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        panic!("fake mcu: unused")
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        panic!("fake mcu: unused")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("fake mcu: unused")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("fake mcu: unused")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("fake mcu: unused")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("fake mcu: unused")
    }
    fn clock32_to_clock64(&self, _clock: i64) -> i64 {
        panic!("fake mcu: unused")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("fake mcu: unused")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("fake mcu: unused")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("fake mcu: unused")
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("fake mcu: unused")
    }
}

struct FakeI2C {
    fail_after: usize,
    writes: Mutex<Vec<Vec<u8>>>,
}

impl FakeI2C {
    fn new(fail_after: usize) -> FakeI2C {
        FakeI2C { fail_after, writes: Mutex::new(Vec::new()) }
    }
    fn writes_value(&self) -> Value {
        Value::List(
            self.writes
                .lock()
                .iter()
                .map(|w| Value::List(w.iter().map(|v| Value::int(*v as i64)).collect()))
                .collect(),
        )
    }
}

impl McuI2cApi for FakeI2C {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        Arc::new(FakeMcu)
    }
    fn get_i2c_address(&self) -> i64 {
        0x60
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write_noack(&self, _data: &[u8], _opts: SendOpts) -> KResult<()> {
        panic!("fake i2c: unused i2c_write_noack")
    }
    fn i2c_write(&self, data: &[u8], _opts: QuerySendOpts) -> KResult<()> {
        let mut writes = self.writes.lock();
        if writes.len() >= self.fail_after {
            return Err(KlippyError::Mcu("simulated i2c failure".to_string()));
        }
        writes.push(data.to_vec());
        Ok(())
    }
    fn i2c_read(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        panic!("fake i2c: unused i2c_read")
    }
    fn i2c_transfer(&self, _write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        panic!("fake i2c: unused i2c_transfer")
    }
}

// ---------------------------------------------------------------------------
// Oracle comparison plumbing.
// ---------------------------------------------------------------------------

fn py_str(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.mcp4728".to_string(),
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

/// The Python module exposes only `load_config_prefix`
/// (`klippy/extras/mcp4728.py:22`).
#[test]
fn module_factory_has_load_config_prefix() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_some());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
    let _ = load_config_prefix as fn(&Config) -> KResult<Option<ObjectRef>>;
}

/// Constructor parity: config reads (with defaults/bounds), I2C payloads
/// and raised errors for every scenario, against the live reference.
#[test]
fn construction_matches_python() {
    let Some(mut py) = oracle() else { return };
    for sc in SCENARIOS {
        let opts = sc
            .opts
            .iter()
            .map(|(k, v)| format!("{}: {}", py_str(k), py_str(v)))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "construct({}, {{{}}}, {})",
            py_str(SECTION),
            opts,
            sc.fail_after
        ));
        let cfg = Arc::new(FakeConfig::new(SECTION, sc.opts));
        let config: Config = cfg.clone();
        let i2c = Arc::new(FakeI2C::new(sc.fail_after));
        let i2c_handle: Arc<dyn McuI2cApi> = i2c.clone();
        let result = Mcp4728::from_i2c(&config, i2c_handle);
        let error = match &result {
            Ok(_) => Value::Null,
            Err(e) => Value::str(&format!("{}: {}", e.class_name(), e.message())),
        };
        let actual = Value::Tuple(vec![error, i2c.writes_value(), cfg.result_value()]);
        check(&format!("construction: {}", sc.label), expected, actual);
    }
}

/// `set_dac` byte packing on a grid of channel/value pairs, including
/// out-of-range values the constructor never produces (the method is
/// public on the Python class).
#[test]
fn set_dac_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "cfg = FakeConfig({}, {{}})\ndac_i2c = FakeI2C(999999999)\n\
         extras.bus.MCU_I2C_from_config = lambda config, default_addr=None, _i2c=dac_i2c: _i2c\n\
         dac_obj = extras.mcp4728.mcp4728(cfg)",
        py_str(SECTION),
    ))
    .expect("build python dac_obj");
    let cfg = Arc::new(FakeConfig::new(SECTION, &[]));
    let config: Config = cfg.clone();
    let i2c = Arc::new(FakeI2C::new(usize::MAX));
    let i2c_handle: Arc<dyn McuI2cApi> = i2c.clone();
    let mcp = Mcp4728::from_i2c(&config, i2c_handle).expect("empty construction");
    for (dac, value) in [
        (0, 0), (0, 1), (0, 255), (0, 256), (0, 4095), (1, 4095), (2, 2048), (3, 4095),
        (0, 4096), (0, -1), (2, 10000),
    ] {
        py.exec("dac_i2c.writes.clear()").expect("clear writes");
        let expected = py.eval(&format!(
            "(dac_obj.set_dac({dac}, {value}) or list(dac_i2c.writes))"
        ));
        i2c.writes.lock().clear();
        mcp.set_dac(dac, value).expect("set_dac");
        check(
            &format!("set_dac({dac}, {value})"),
            expected,
            i2c.writes_value(),
        );
    }
}
