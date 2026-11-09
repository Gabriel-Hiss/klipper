//! Parity tests for `klippy/extras/display/menu_keys.py`.
//!
//! Every expected value comes live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + frozen `klippy/`; the test skips with a
//! message when `$KLIPPY_PYTHON` is unset, exactly like the other unit
//! tests). For each scenario the oracle loads the real `menu_keys.py`
//! with minimal Python fakes (a fake reactor, a recording fake `buttons`
//! object, a fake printer and a real `configfile.ConfigWrapper`), and
//! the port is driven through the same fakes in Rust (fake `PrinterApi`,
//! fake `ConfigApi`, recording fake [`PrinterButtonsApi`] served as the
//! `buttons` object through `load_object`). Compared byte-for-byte via
//! `recording::assert_equal` (floats by hex + repr):
//! - the `buttons` registration calls (method, pins, float args, steps);
//! - the menu callback log (`key`, `eventtime`) after driving every
//!   callback with fixed eventtimes;
//! - `long_click_event` return values (`reactor.NEVER`);
//! - the kill-button shutdown message;
//! - construction errors: exception class and message.
//!
//! The long-press timer arithmetic itself (`eventtime + 0.8`) is
//! additionally run end to end through the real `reactor` crate in
//! [`long_press_timer_through_reactor`]: its fire time is jittered in
//! real time, so only the callback names and loose time bounds are
//! asserted there.
//!
//! Integration owner runs:
//! `cargo test -p extras --test display_menu_keys`

use std::sync::Arc;

use extras::buttons::{
    AdcButtonCallback, ButtonCallback, DebounceCallback, PrinterButtonsApi, PushCallback,
};
use extras::display::menu_keys::{AnyButtonCallback, MenuKeyCallback, MenuKeys};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use kos::sync::Mutex;
use pycompat::text::{float_from_string, float_repr, int_from_string};
use pycompat::value::py_equal;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NEVER};
use recording::value::{Record, RecordKind, Value};

// ---------------------------------------------------------------------------
// Fakes of the contract traits.
// ---------------------------------------------------------------------------

type CallbackLog = Arc<Mutex<Vec<(String, f64)>>>;

#[allow(dead_code)]
/// Fake `PrinterApi`: real reactor, recording `invoke_shutdown`.
struct FakePrinter {
    reactor: Reactor,
    shutdown: Mutex<Vec<String>>,
    /// `load_object` sections, recorded when [`MenuKeys::new`] is used.
    loads: Mutex<Vec<String>>,
    /// Registered objects (never populated in these tests).
    objects: Mutex<Vec<(String, ObjectRef)>>,
    /// The `buttons` object `load_object("buttons")` returns.
    buttons_obj: ObjectRef,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), String::new())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock();
        for (n, obj) in objects.iter() {
            if n == name {
                return Ok(Some(obj.clone()));
            }
        }
        if name == "buttons" {
            return Ok(Some(self.buttons_obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.lock().clone()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.loads.lock().push(section.to_string());
        if section == "buttons" {
            return Ok(Some(self.buttons_obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{section}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        panic!("unexpected PrinterApi::run");
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("unexpected PrinterApi::set_rollover_info");
    }
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdown.lock().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("unexpected PrinterApi::invoke_async_shutdown");
    }
    fn register_event_handler(&self, _event: &str, _cb: EventCallback) {
        panic!("unexpected PrinterApi::register_event_handler");
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("unexpected PrinterApi::send_event")
    }
    fn request_exit(&self, _result: &str) {
        panic!("unexpected PrinterApi::request_exit");
    }
}

#[allow(dead_code)]
/// Fake `ConfigApi` backed by raw (section, option) string pairs. The
/// typed getters replicate `configfile.py` `_get_wrapper` and
/// `getchoice`/`getlists` error texts.
struct FakeConfig {
    printer: Printer,
    section: String,
    opts: Vec<(String, String)>,
    reads: Mutex<Vec<String>>,
}

fn config_error(section: &str, option: &str, what: &str) -> KlippyError {
    KlippyError::config(format!("{what} '{option}' in section '{section}'"))
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<String> {
        self.opts
            .iter()
            .find(|(k, _)| k == option)
            .map(|(_, v)| v.clone())
    }
    fn missing<T>(&self, option: &str) -> KResult<T> {
        Err(KlippyError::config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        )))
    }
    fn parse_fail<T>(&self, option: &str) -> KResult<T> {
        Err(config_error(&self.section, option, "Unable to parse option"))
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
        self.reads.lock().push(option.to_string());
        match self.raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => self.missing(option),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.reads.lock().push(option.to_string());
        match self.raw(option) {
            None => match opts.default {
                None => self.missing(option),
                Some(inner) => Ok(inner),
            },
            Some(raw) => {
                let v = match int_from_string(&raw, 10) {
                    Ok(n) => match n.0.to_string().parse::<i64>() {
                        Ok(v) => v,
                        Err(_) => return self.parse_fail(option),
                    },
                    Err(_) => return self.parse_fail(option),
                };
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.section
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.section
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads.lock().push(option.to_string());
        match self.raw(option) {
            None => match opts.default {
                None => self.missing(option),
                Some(inner) => Ok(inner),
            },
            Some(raw) => {
                let v = match float_from_string(&raw) {
                    Ok(v) => v,
                    Err(_) => return self.parse_fail(option),
                };
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            float_repr(minval)
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            float_repr(maxval)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            float_repr(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            float_repr(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("unexpected ConfigApi::getboolean")
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        self.reads.lock().push(option.to_string());
        // Normalize `choices` to (keys, mapped values): a `List` is an
        // identity map, as in `getchoice` (`configfile.py:77-78`).
        let (keys, mapped): (Vec<PyValue>, Vec<PyValue>) = match choices {
            PyValue::List(items) => (items.clone(), items.clone()),
            PyValue::Dict(pairs) => {
                let keys: Vec<PyValue> = pairs.keys().cloned().collect();
                (keys, pairs.values().cloned().collect())
            }
            _ => panic!("getchoice choices must be a list or dict"),
        };
        // Int mode: the first key is an int (`configfile.py:79`).
        let int_mode = matches!(keys.first(), Some(PyValue::Int(_)));
        // The key (from the option, or from the default key).
        let key: PyValue = match self.raw(option) {
            None => match opts.default {
                None => return self.missing(option),
                Some(inner) => inner.unwrap_or(PyValue::None),
            },
            Some(_) if int_mode => {
                match self.getint(
                    option,
                    GetIntOpts { default: Some(None), ..Default::default() },
                )? {
                    Some(n) => PyValue::Int(PyInt::from_i64(n)),
                    None => PyValue::None,
                }
            }
            Some(raw) => PyValue::Str(raw),
        };
        for (k, v) in keys.iter().zip(mapped.iter()) {
            if py_equal(k, &key) {
                return Ok(v.clone());
            }
        }
        let c = match &key {
            PyValue::None => "None".to_string(),
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Str(s) => s.clone(),
            other => other.str(),
        };
        Err(KlippyError::config(format!(
            "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
            self.section
        )))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("unexpected ConfigApi::getlists")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("unexpected ConfigApi::getlist")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("unexpected ConfigApi::getintlist")
    }
    fn getfloatlist(
        &self,
        option: &str,
        opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        self.reads.lock().push(option.to_string());
        match self.raw(option) {
            None => match opts.default {
                None => self.missing(option),
                Some(inner) => Ok(inner),
            },
            Some(raw) => {
                // `lparser` (`configfile.py:89-102`) for a single separator.
                let parts: Vec<String> = if raw.trim().is_empty() {
                    Vec::new()
                } else {
                    raw.split(opts.sep.as_str()).map(|p| p.trim().to_string()).collect()
                };
                let mut values = Vec::with_capacity(parts.len());
                for p in &parts {
                    match float_from_string(p) {
                        Ok(v) => values.push(v),
                        Err(_) => return self.parse_fail(option),
                    }
                }
                if let Some(count) = opts.count {
                    if values.len() != count {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have {count} elements",
                            self.section
                        )));
                    }
                }
                Ok(Some(values))
            }
        }
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("unexpected ConfigApi::getsection")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("unexpected ConfigApi::has_section")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("unexpected ConfigApi::get_prefix_sections")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("unexpected ConfigApi::get_prefix_options")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        panic!("unexpected ConfigApi::deprecate")
    }
}

/// Recording fake for [`PrinterButtonsApi`]. Calls are stored as
/// [`recording::Value`] lists with exactly the shapes the Python fake
/// records, so the oracle comparison needs no extra formatting.
struct FakeButtons {
    calls: Mutex<Vec<Value>>,
    cw: Mutex<Option<PushCallback>>,
    ccw: Mutex<Option<PushCallback>>,
    state_cbs: Mutex<Vec<(Vec<String>, ButtonCallback)>>,
    push_cbs: Mutex<Vec<(String, PushCallback)>>,
    adc_push_cbs: Mutex<Vec<(String, PushCallback)>>,
    adc_cbs: Mutex<Vec<(String, AdcButtonCallback)>>,
}

fn call_record(parts: Vec<Value>) -> Value {
    Value::List(parts)
}

impl FakeButtons {
    fn new() -> FakeButtons {
        FakeButtons {
            calls: Mutex::new(Vec::new()),
            cw: Mutex::new(None),
            ccw: Mutex::new(None),
            state_cbs: Mutex::new(Vec::new()),
            push_cbs: Mutex::new(Vec::new()),
            adc_push_cbs: Mutex::new(Vec::new()),
            adc_cbs: Mutex::new(Vec::new()),
        }
    }
    fn record(&self, parts: Vec<Value>) {
        self.calls.lock().push(call_record(parts));
    }
    fn call_cw(&self, eventtime: f64) {
        let cb = self.cw.lock();
        cb.as_ref().expect("cw callback registered")(eventtime);
    }
    fn call_ccw(&self, eventtime: f64) {
        let cb = self.ccw.lock();
        cb.as_ref().expect("ccw callback registered")(eventtime);
    }
    fn call_push(&self, pin: &str, eventtime: f64) {
        let cbs = self.push_cbs.lock();
        for (p, cb) in cbs.iter() {
            if p == pin {
                cb(eventtime);
                return;
            }
        }
        panic!("push callback for pin {pin} not registered");
    }
    fn call_adc_push(&self, pin: &str, eventtime: f64) {
        let cbs = self.adc_push_cbs.lock();
        for (p, cb) in cbs.iter() {
            if p == pin {
                cb(eventtime);
                return;
            }
        }
        panic!("adc push callback for pin {pin} not registered");
    }
    fn call_state(&self, pin: &str, eventtime: f64, state: bool) {
        {
            let cbs = self.state_cbs.lock();
            for (pins, cb) in cbs.iter() {
                if pins.len() == 1 && pins[0] == pin {
                    cb(eventtime, if state { 1 } else { 0 });
                    return;
                }
            }
        }
        let cbs = self.adc_cbs.lock();
        for (p, cb) in cbs.iter() {
            if p == pin {
                cb(eventtime, state);
                return;
            }
        }
        panic!("state callback for pin {pin} not registered");
    }
    fn calls(&self) -> Value {
        Value::List(self.calls.lock().clone())
    }
}

klippy_api::printer_object!(FakeButtons; dyn PrinterButtonsApi);

impl PrinterButtonsApi for FakeButtons {
    fn register_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: AdcButtonCallback,
    ) -> KResult<()> {
        self.record(vec![
            Value::str("register_adc_button"),
            Value::str(pin),
            Value::float(min_val),
            Value::float(max_val),
            Value::float(pullup),
        ]);
        self.adc_cbs.lock().push((pin.to_string(), callback));
        Ok(())
    }
    fn register_debounce_button(
        &self,
        _pin: &str,
        _callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_debounce_button")
    }
    fn register_debounce_adc_button(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_debounce_adc_button")
    }
    fn register_adc_button_push(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: PushCallback,
    ) -> KResult<()> {
        self.record(vec![
            Value::str("register_adc_button_push"),
            Value::str(pin),
            Value::float(min_val),
            Value::float(max_val),
            Value::float(pullup),
        ]);
        self.adc_push_cbs.lock().push((pin.to_string(), callback));
        Ok(())
    }
    fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) -> KResult<()> {
        self.record(vec![
            Value::str("register_buttons"),
            Value::List(pins.iter().map(|p| Value::str(p)).collect()),
        ]);
        self.state_cbs.lock().push((pins.to_vec(), callback));
        Ok(())
    }
    fn register_rotary_encoder(
        &self,
        pin1: &str,
        pin2: &str,
        cw_callback: PushCallback,
        ccw_callback: PushCallback,
        steps_per_detent: i64,
    ) -> KResult<()> {
        self.record(vec![
            Value::str("register_rotary_encoder"),
            Value::str(pin1),
            Value::str(pin2),
            Value::int(steps_per_detent),
        ]);
        *self.cw.lock() = Some(cw_callback);
        *self.ccw.lock() = Some(ccw_callback);
        Ok(())
    }
    fn register_button_push(&self, pin: &str, callback: PushCallback) -> KResult<()> {
        self.record(vec![Value::str("register_button_push"), Value::str(pin)]);
        self.push_cbs.lock().push((pin.to_string(), callback));
        Ok(())
    }
}

/// Build the Rust side of a scenario: fake printer with a real reactor,
/// fake config and recording buttons, then the ported constructor.
struct RustSide {
    keys: Option<Arc<MenuKeys>>,
    init_err: Option<KlippyError>,
    printer: Arc<FakePrinter>,
    buttons: Arc<FakeButtons>,
    cb: CallbackLog,
}

fn build_rust(opts: &[(String, String)]) -> RustSide {
    let reactor = Reactor::new(false);
    let buttons = Arc::new(FakeButtons::new());
    let buttons_obj: ObjectRef = Arc::clone(&buttons) as ObjectRef;
    let printer = Arc::new(FakePrinter {
        reactor,
        shutdown: Mutex::new(Vec::new()),
        loads: Mutex::new(Vec::new()),
        objects: Mutex::new(Vec::new()),
        buttons_obj,
    });
    let printer_ref: Printer = printer.clone();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_ref,
        section: "test_menu".to_string(),
        opts: opts.to_vec(),
        reads: Mutex::new(Vec::new()),
    });
    let cb: CallbackLog = Arc::new(Mutex::new(Vec::new()));
    let cb_clone = Arc::clone(&cb);
    let callback: MenuKeyCallback =
        Arc::new(move |key: &str, et: f64| cb_clone.lock().push((key.to_string(), et)));
    let result = MenuKeys::new(&config, callback);
    match result {
        Ok(keys) => RustSide {
            keys: Some(keys),
            init_err: None,
            printer,
            buttons,
            cb,
        },
        Err(e) => RustSide {
            keys: None,
            init_err: Some(e),
            printer,
            buttons,
            cb,
        },
    }
}

fn cb_value(log: &CallbackLog) -> Value {
    Value::List(
        log.lock()
            .iter()
            .map(|(k, et)| Value::List(vec![Value::str(k), Value::float(*et)]))
            .collect(),
    )
}

fn shutdown_value(printer: &FakePrinter) -> Value {
    Value::List(
        printer
            .shutdown
            .lock()
            .iter()
            .map(|m| Value::List(vec![Value::str(m)]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Oracle harness.
// ---------------------------------------------------------------------------

fn repo_root() -> std::path::PathBuf {
    let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .and_then(|p| p.parent())
        .and_then(|p| p.parent())
        .map(std::path::Path::to_path_buf)
        .unwrap_or_else(|| std::path::PathBuf::from("."))
}

fn menu_keys_path() -> String {
    let path = repo_root().join("klippy/extras/display/menu_keys.py");
    format!("{}", path.display()).replace('\\', "\\\\")
}

/// One-line Python string literal (double quotes, escapes).
fn py_str(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

const PY_SETUP: &str = r#"
import configparser, importlib.util
spec = importlib.util.spec_from_file_location("menu_keys_py", "__PATH__")
menu_keys_py = importlib.util.module_from_spec(spec)
spec.loader.exec_module(menu_keys_py)
from configfile import ConfigWrapper

class FakeReactor:
    NEVER = 9999999999999999.
    NOW = 0.
    def __init__(self):
        self._cbs = []
    def register_timer(self, cb, waketime=9999999999999999.):
        self._cbs.append(cb)
        return len(self._cbs)
    def update_timer(self, timer, waketime):
        return None

class FakeButtons:
    def __init__(self):
        self.calls = []
        self.cw_cb = None
        self.ccw_cb = None
        self.state_cbs = {}
        self.push_cbs = {}
        self.adc_push_cbs = {}
        self.adc_cbs = {}
    def register_rotary_encoder(self, pin1, pin2, cw, ccw, steps):
        self.calls.append(['register_rotary_encoder', pin1, pin2, steps])
        self.cw_cb = cw
        self.ccw_cb = ccw
    def register_button_push(self, pin, cb):
        self.calls.append(['register_button_push', pin])
        self.push_cbs[pin] = cb
    def register_buttons(self, pins, cb):
        self.calls.append(['register_buttons', list(pins)])
        self.state_cbs[tuple(pins)] = cb
    def register_adc_button_push(self, pin, amin, amax, pullup, cb):
        self.calls.append(['register_adc_button_push', pin, amin, amax, pullup])
        self.adc_push_cbs[pin] = cb
    def register_adc_button(self, pin, amin, amax, pullup, cb):
        self.calls.append(['register_adc_button', pin, amin, amax, pullup])
        self.adc_cbs[pin] = cb

class FakePrinter:
    def __init__(self, reactor, buttons):
        self._reactor = reactor
        self._buttons = buttons
        self.shutdown = []
    def get_reactor(self):
        return self._reactor
    def load_object(self, config, section):
        return self._buttons
    def lookup_object(self, name):
        return self._buttons
    def invoke_shutdown(self, msg, details=None):
        self.shutdown.append([msg])

REACTOR = FakeReactor()
BUTTONS = FakeButtons()
PRINTER = FakePrinter(REACTOR, BUTTONS)
CB = []
RET = []
def mk_cb(key, eventtime):
    CB.append([key, eventtime])
def build(opts):
    fc = configparser.RawConfigParser()
    fc.add_section('test_menu')
    for k, v in opts.items():
        fc.set('test_menu', k, v)
    return ConfigWrapper(PRINTER, fc, {}, 'test_menu')
CFG = build(__OPTS__)
try:
    MK = menu_keys_py.MenuKeys(CFG, mk_cb)
    INIT_ERR = None
except Exception as e:
    INIT_ERR = [type(e).__name__, str(e)]
"#;

/// One driven callback. `State` goes through the registered
/// (eventtime, state) callback for `pin` (digital or ADC).
#[derive(Clone, Debug)]
enum Drive {
    Cw(f64),
    Ccw(f64),
    Click(f64, bool),
    Push(&'static str, f64),
    AdcPush(&'static str, f64),
    State(&'static str, f64, bool),
    LongClick(f64),
}

fn py_drive(d: &Drive) -> String {
    match *d {
        Drive::Cw(et) => format!("BUTTONS.cw_cb({et:?})"),
        Drive::Ccw(et) => format!("BUTTONS.ccw_cb({et:?})"),
        Drive::Click(et, state) => {
            format!("MK.click_callback({et:?}, {})", if state { "True" } else { "False" })
        }
        Drive::Push(pin, et) => format!("BUTTONS.push_cbs[{}]({et:?})", py_str(pin)),
        Drive::AdcPush(pin, et) => {
            format!("BUTTONS.adc_push_cbs[{}]({et:?})", py_str(pin))
        }
        Drive::State(pin, et, state) => {
            let st = if state { "True" } else { "False" };
            format!(
                "(BUTTONS.state_cbs.get(({},)) or BUTTONS.adc_cbs[{}])({et:?}, {st})",
                py_str(pin),
                py_str(pin)
            )
        }
        Drive::LongClick(et) => format!("RET.append(MK.long_click_event({et:?}))"),
    }
}

struct ScenarioOut {
    py_calls: Value,
    py_cb: Value,
    py_shutdown: Value,
    py_rets: Value,
    py_init_err: Value,
    rs_calls: Value,
    rs_cb: Value,
    rs_shutdown: Value,
    rs_rets: Vec<f64>,
    rs_init_err: Value,
}

/// Run one scenario on both sides. Returns `None` when the oracle is
/// unavailable (tests then skip, like the other unit tests).
fn run_scenario(
    opts: Vec<(String, String)>,
    drives: &[Drive],
) -> Option<ScenarioOut> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        return None;
    };
    let opts_literal = {
        let parts: Vec<String> = opts
            .iter()
            .map(|(k, v)| format!("{}: {}", py_str(k), py_str(v)))
            .collect();
        format!("{{{}}}", parts.join(", "))
    };
    let setup = PY_SETUP
        .replace("__PATH__", &menu_keys_path())
        .replace("__OPTS__", &opts_literal);
    py.exec(&setup)
        .unwrap_or_else(|e| panic!("setup exec raised {e:?}"));
    let py_init_err = py.eval("INIT_ERR").expect("INIT_ERR eval");
    // Rust side first: drives only happen when both sides constructed, so
    // logs on either side stay comparable after a construction error.
    let side = build_rust(&opts);
    let both_ok = py_init_err == Value::Null && side.keys.is_some();
    if both_ok {
        for d in drives {
            let code = py_drive(d);
            py.eval(&code)
                .unwrap_or_else(|e| panic!("drive `{code}` raised {e:?}"));
        }
    }
    let py_calls = py.eval("BUTTONS.calls").expect("calls eval");
    let py_cb = py.eval("CB").expect("CB eval");
    let py_shutdown = py.eval("PRINTER.shutdown").expect("shutdown eval");
    let py_rets = py.eval("RET").expect("RET eval");
    let mut rets: Vec<f64> = Vec::new();
    if let Some(keys) = &side.keys {
        for d in drives {
            match *d {
                Drive::Cw(et) => side.buttons.call_cw(et),
                Drive::Ccw(et) => side.buttons.call_ccw(et),
                Drive::Click(et, state) => keys.click_callback(et, state),
                Drive::Push(pin, et) => side.buttons.call_push(pin, et),
                Drive::AdcPush(pin, et) => side.buttons.call_adc_push(pin, et),
                Drive::State(pin, et, state) => side.buttons.call_state(pin, et, state),
                Drive::LongClick(et) => rets.push(keys.long_click_event(et)),
            }
        }
    }
    let rs_init_err = match &side.init_err {
        None => Value::Null,
        Some(e) => Value::List(vec![
            Value::str(e.class_name()),
            Value::str(e.message()),
        ]),
    };
    Some(ScenarioOut {
        py_calls,
        py_cb,
        py_shutdown,
        py_rets,
        py_init_err,
        rs_calls: side.buttons.calls(),
        rs_cb: cb_value(&side.cb),
        rs_shutdown: shutdown_value(&side.printer),
        rs_rets: rets,
        rs_init_err,
    })
}

fn dummy_record() -> Record {
    Record {
        v: 1,
        unit: "extras.display.menu_keys".to_string(),
        scenario: "parity".to_string(),
        seq: 1,
        function: "run_scenario".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: Default::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: &Value, actual: &Value) {
    if let Err(d) = recording::assert_equal(&dummy_record(), label, expected, actual) {
        panic!("{label}: {d}");
    }
}

/// `long_click_event` returns the reactor `NEVER` waketime (1e16).
/// Compared by bits: `recording::py_float_repr` spells that integral
/// value `10000000000000000.0` while CPython spells it `1e+16`.
fn check_rets(expected: &Value, actual: &[f64]) {
    let Value::List(items) = expected else {
        panic!("rets: oracle returned {expected:?}, expected a list");
    };
    assert_eq!(items.len(), actual.len(), "rets: length differs");
    for (i, (e, a)) in items.iter().zip(actual.iter()).enumerate() {
        let Value::Float(f) = e else {
            panic!("rets[{i}]: oracle returned {e:?}, expected a float");
        };
        let expected_bits = f.to_f64().expect("rets float parses").to_bits();
        assert_eq!(expected_bits, a.to_bits(), "rets[{i}]: bit mismatch");
    }
}

/// Compare every channel of one scenario between the oracle and the port.
fn check_scenario(opts: Vec<(String, String)>, drives: &[Drive]) {
    let Some(out) = run_scenario(opts, drives) else {
        println!(
            "skipping menu_keys parity: set KLIPPY_PYTHON to a CPython 3.8.2 \
             binary and run with the repo checkout available"
        );
        return;
    };
    check("init_err", &out.py_init_err, &out.rs_init_err);
    check("calls", &out.py_calls, &out.rs_calls);
    check("cb", &out.py_cb, &out.rs_cb);
    check("shutdown", &out.py_shutdown, &out.rs_shutdown);
    check_rets(&out.py_rets, &out.rs_rets);
}

fn opts(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

#[test]
fn empty_config_registers_nothing() {
    check_scenario(Vec::new(), &[]);
}

#[test]
fn encoder_default_config() {
    // No click pins: only the rotary encoder is registered. Drives start
    // at `last == 0`, so early eventtimes count as fast.
    check_scenario(
        opts(&[("encoder_pins", "mcu:PA1,mcu:PA2")]),
        &[
            Drive::Cw(-5.0),
            Drive::Cw(-4.99),
            Drive::Cw(0.5),
            Drive::Ccw(0.5),
            Drive::Ccw(0.52),
            Drive::Ccw(5.0),
        ],
    );
}

#[test]
fn encoder_steps_two_and_spaced_pins() {
    check_scenario(
        opts(&[
            ("encoder_pins", "  mcu:PA1 ,  mcu:PA2  "),
            ("encoder_steps_per_detent", "2"),
        ]),
        &[],
    );
}

#[test]
fn encoder_pins_without_comma() {
    check_scenario(opts(&[("encoder_pins", "mcu:PA1")]), &[]);
}

#[test]
fn encoder_pins_with_three_entries() {
    check_scenario(opts(&[("encoder_pins", "mcu:PA1,mcu:PA2,mcu:PA3")]), &[]);
}

#[test]
fn encoder_steps_invalid() {
    check_scenario(
        opts(&[
            ("encoder_pins", "mcu:PA1,mcu:PA2"),
            ("encoder_steps_per_detent", "3"),
        ]),
        &[],
    );
}

#[test]
fn encoder_steps_junk() {
    check_scenario(
        opts(&[
            ("encoder_pins", "mcu:PA1,mcu:PA2"),
            ("encoder_steps_per_detent", "junk"),
        ]),
        &[],
    );
}

#[test]
fn encoder_fast_rate_zero() {
    check_scenario(
        opts(&[
            ("encoder_pins", "mcu:PA1,mcu:PA2"),
            ("encoder_fast_rate", "0"),
        ]),
        &[],
    );
}

#[test]
fn encoder_fast_rate_negative_and_junk() {
    check_scenario(
        opts(&[
            ("encoder_pins", "mcu:PA1,mcu:PA2"),
            ("encoder_fast_rate", "-0.5"),
        ]),
        &[],
    );
    check_scenario(
        opts(&[
            ("encoder_pins", "mcu:PA1,mcu:PA2"),
            ("encoder_fast_rate", "junk"),
        ]),
        &[],
    );
}

#[test]
fn encoder_fast_rate_custom_boundaries() {
    // fast_rate 0.1: diff 0.1 exactly is fast (`<=`), anything above slow.
    check_scenario(
        opts(&[
            ("encoder_pins", "mcu:PA1,mcu:PA2"),
            ("encoder_fast_rate", "0.1"),
        ]),
        &[
            Drive::Cw(0.1),
            Drive::Cw(0.2),
            Drive::Cw(0.300001),
            Drive::Ccw(1.0),
            Drive::Ccw(1.05),
            Drive::Ccw(1.200001),
        ],
    );
}

#[test]
fn all_push_buttons() {
    check_scenario(
        opts(&[
            ("click_pin", "mcu:PA9"),
            ("back_pin", "mcu:PA5"),
            ("up_pin", "mcu:PA6"),
            ("down_pin", "mcu:PA7"),
            ("kill_pin", "mcu:PA8"),
        ]),
        &[
            Drive::Push("mcu:PA5", 10.0),
            Drive::Push("mcu:PA6", 11.0),
            Drive::Push("mcu:PA7", 12.0),
            Drive::Push("mcu:PA8", 13.0),
            Drive::State("mcu:PA9", 20.0, true),
            Drive::State("mcu:PA9", 20.5, false),
        ],
    );
}

#[test]
fn analog_click_and_back_default_pullup() {
    check_scenario(
        opts(&[
            ("click_pin", "mcu:PA9"),
            ("analog_range_click_pin", "1,4"),
            ("back_pin", "mcu:PA5"),
            ("analog_range_back_pin", "0.5,2.5"),
        ]),
        &[
            Drive::AdcPush("mcu:PA5", 30.0),
            Drive::State("mcu:PA9", 31.0, true),
            Drive::State("mcu:PA9", 31.5, false),
        ],
    );
}

#[test]
fn analog_click_custom_pullup() {
    check_scenario(
        opts(&[
            ("click_pin", "mcu:PA9"),
            ("analog_range_click_pin", "1,4"),
            ("analog_pullup_resistor", "1000"),
        ]),
        &[],
    );
}

#[test]
fn analog_range_wrong_count() {
    // The failure happens during `back_pin` registration, after the
    // click pin registered fine: the partial call list must match.
    check_scenario(
        opts(&[
            ("click_pin", "mcu:PA9"),
            ("back_pin", "mcu:PA5"),
            ("analog_range_back_pin", "1"),
        ]),
        &[],
    );
}

#[test]
fn analog_pullup_zero() {
    check_scenario(
        opts(&[
            ("back_pin", "mcu:PA5"),
            ("analog_range_back_pin", "1,2"),
            ("analog_pullup_resistor", "0"),
        ]),
        &[],
    );
}

#[test]
fn analog_range_junk_element() {
    check_scenario(
        opts(&[
            ("back_pin", "mcu:PA5"),
            ("analog_range_back_pin", "1,junk"),
        ]),
        &[],
    );
}

#[test]
fn click_long_click_and_release_quirk() {
    // A release with `is_short_click` still set fires `click` again:
    // only `long_click_event` clears the flag (Python quirk, reproduced).
    let opts = opts(&[("click_pin", "mcu:PA9")]);
    check_scenario(
        opts,
        &[
            Drive::Click(1.0, true),
            Drive::Click(1.5, false),
            Drive::Click(2.0, false),
            Drive::LongClick(3.0),
            Drive::Click(4.0, false),
            Drive::Click(5.0, true),
            Drive::Click(5.2, false),
        ],
    );
}

#[test]
fn long_click_returns_never() {
    // Direct (no oracle): the timer callback returns the reactor NEVER.
    let side = build_rust(&opts(&[("click_pin", "mcu:PA9")]));
    let keys = side.keys.expect("construction succeeds");
    assert_eq!(keys.long_click_event(7.0), NEVER);
}

#[test]
fn long_press_timer_through_reactor() {
    use std::sync::atomic::{AtomicBool, Ordering};
    // Watchdog: finalize after 4 real seconds if the run loop deadlocks.
    fn run_with_timeout(r: &Reactor) {
        use kos::thread as kthread;
        use kos::time as ktime;
        let r_watch = r.clone();
        let done = Arc::new(AtomicBool::new(false));
        let fired = Arc::new(AtomicBool::new(false));
        let done2 = Arc::clone(&done);
        let fired2 = Arc::clone(&fired);
        let watchdog = kthread::spawn(move || {
            let deadline = ktime::monotonic() + 4.0;
            while ktime::monotonic() < deadline {
                if done2.load(Ordering::SeqCst) {
                    return;
                }
                ktime::sleep(0.01);
            }
            if !done2.load(Ordering::SeqCst) {
                fired2.store(true, Ordering::SeqCst);
                r_watch.finalize();
            }
        });
        r.run();
        done.store(true, Ordering::SeqCst);
        watchdog.join();
        assert!(
            !fired.load(Ordering::SeqCst),
            "reactor run timed out after 4s (baton deadlock?)"
        );
    }
    fn build() -> (Reactor, RustSide) {
        let reactor = Reactor::new(false);
        let buttons = Arc::new(FakeButtons::new());
        let buttons_obj: ObjectRef = Arc::clone(&buttons) as ObjectRef;
        let printer = Arc::new(FakePrinter {
            reactor: reactor.clone(),
            shutdown: Mutex::new(Vec::new()),
            loads: Mutex::new(Vec::new()),
            objects: Mutex::new(Vec::new()),
            buttons_obj,
        });
        let printer_ref: Printer = printer.clone();
        let config: Config = Arc::new(FakeConfig {
            printer: printer_ref,
            section: "test_menu".to_string(),
            opts: opts(&[("click_pin", "mcu:PA9")]),
            reads: Mutex::new(Vec::new()),
        });
        let cb: CallbackLog = Arc::new(Mutex::new(Vec::new()));
        let cb_clone = Arc::clone(&cb);
        let callback: MenuKeyCallback =
            Arc::new(move |key: &str, et: f64| cb_clone.lock().push((key.to_string(), et)));
        let keys = MenuKeys::new(&config, callback)
            .expect("construction succeeds");
        (reactor, RustSide {
            keys: Some(keys),
            init_err: None,
            printer,
            buttons,
            cb,
        })
    }
    // Long press: fire at press + 0.8 through the real reactor.
    {
        let (reactor, side) = build();
        let keys = side.keys.expect("built");
        let t0 = reactor.monotonic();
        keys.click_callback(t0, true);
        reactor.register_timer(
            {
                let r = reactor.clone();
                move |_| {
                    r.end();
                    NEVER
                }
            },
            t0 + 1.3,
        );
        run_with_timeout(&reactor);
        let log = side.cb.lock().clone();
        assert_eq!(log.len(), 1, "long press: callbacks = {log:?}");
        assert_eq!(log[0].0, "long_click");
        assert!(
            (t0 + 0.8 - 0.05) < log[0].1 && log[0].1 < t0 + 1.3,
            "long_click eventtime {} not near press + 0.8 (t0 = {t0})",
            log[0].1
        );
    }
    // Short press: release disarms before the 0.8s mark.
    {
        let (reactor, side) = build();
        let keys = side.keys.expect("built");
        let t0 = reactor.monotonic();
        keys.click_callback(t0, true);
        keys.click_callback(t0 + 0.1, false);
        reactor.register_timer(
            {
                let r = reactor.clone();
                move |_| {
                    r.end();
                    NEVER
                }
            },
            t0 + 1.0,
        );
        run_with_timeout(&reactor);
        let log = side.cb.lock().clone();
        assert_eq!(
            log.iter().map(|(k, _)| k.as_str()).collect::<Vec<_>>(),
            vec!["click"],
            "short press: callbacks = {log:?}"
        );
    }
}

/// `State` and `Click` drives hit the same `click_callback`; the
/// `State` drive additionally proves the stored (eventtime, state)
/// closure is wired correctly. Covered in [`all_push_buttons`].
#[test]
fn callback_wiring_sanity() {
    // No oracle: stored closures must exist for encoder, push and state.
    let side = build_rust(&opts(&[
        ("encoder_pins", "mcu:PA1,mcu:PA2"),
        ("click_pin", "mcu:PA9"),
        ("back_pin", "mcu:PA5"),
    ]));
    let keys = side.keys.expect("construction succeeds");
    side.buttons.call_cw(0.0);
    side.buttons.call_ccw(0.0);
    side.buttons.call_push("mcu:PA5", 1.0);
    side.buttons.call_state("mcu:PA9", 2.0, true);
    side.buttons.call_state("mcu:PA9", 2.5, false);
    keys.kill_callback(3.0);
    let names: Vec<String> = side
        .cb
        .lock()
        .iter()
        .map(|(k, _)| k.clone())
        .collect();
    assert_eq!(
        names,
        vec![
            "fast_up",
            "fast_down",
            "back",
            "click",
        ]
    );
    assert_eq!(
        *side.printer.shutdown.lock(),
        vec!["Shutdown due to kill button!".to_string()]
    );
    // `AnyButtonCallback` mismatch is an internal error, not a panic.
    let printer_ref: Printer = side.printer.clone();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_ref,
        section: "test_menu".to_string(),
        opts: opts(&[("back_pin", "mcu:PA5")]),
        reads: Mutex::new(Vec::new()),
    });
    let err = keys
        .register_button(
            &config,
            "back_pin",
            AnyButtonCallback::State(Arc::new(|_, _| {})),
            true,
        )
        .expect_err("arity mismatch must error");
    assert_eq!(err.class_name(), "TypeError");
}
