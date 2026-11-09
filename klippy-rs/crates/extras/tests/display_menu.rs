//! Parity tests for `extras::display::menu` (port of
//! `klippy/extras/display/menu.py`).
//!
//! Every expected value comes from the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a Python
//! `MenuManager` and a Rust `MenuManager` are built from the same menu
//! specification with matching minimal fakes for the printer, the config
//! wrappers, the display, `virtual_sdcard`, `buttons` and `gcode`. The
//! `gcode_macro` object and `menu_keys` are the real modules of each side
//! (`extras.gcode_macro` / `extras.display.menu_keys` in Python,
//! `PrinterGCodeMacro` / `MenuKeys` in Rust), so templates, template
//! contexts and button wiring are compared as they run in production. Both
//! sides then run the same key, press, navigation and drawing sequences.
//! Compared: returned values (floats by bits), raised errors (class and
//! message), G-code scripts queued and dispatched, `get_status` dicts,
//! display draw calls, event names, button registrations, config options
//! read with their defaults, and the children order.
//!
//! `SyntaxError` and `IndentationError` messages are compared by class
//! only. They come from the CPython parser, and the class decides how
//! `menu.py` reacts. The template texts in the scenarios avoid `{` unless
//! they are meant to be Jinja expressions, because `{` opens one with
//! the Klipper delimiters.

use extras::buttons::{
    AdcButtonCallback, ButtonCallback, DebounceCallback, PrinterButtonsApi, PushCallback,
};
use extras::display::display::PrinterLCDApi;
use extras::display::menu::{
    LookupRef, MenuElement, MenuItemKwargs, MenuManager, RunScriptOpts, ScriptValue,
    TemplateContext, TIMER_DELAY,
};
use extras::gcode_macro::PrinterGCodeMacro;
use extras::virtual_sdcard::VirtualSDApi;
use extras_macros_templates::gcode_macro::Value as TplValue;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, RegisterCommandOpts};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::{float_from_string, float_str, str_repr};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import contextlib
import gcode as gcode_mod
import extras.gcode_macro as gcode_macro_mod
import extras.display.menu as menu_mod

_sentinel = object()

class FakeReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0
    def __init__(self):
        self.timers = []
        self.callbacks = []
        self.updates = []
    def monotonic(self):
        return 1.0
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append((cb, waketime))
        return len(self.timers)
    def update_timer(self, timer, waketime):
        self.updates.append((timer, waketime))
    def register_callback(self, cb, waketime=NOW):
        self.callbacks.append(cb)
        return len(self.callbacks)
    def assert_no_pause(self):
        return contextlib.nullcontext()

class FakeGcode:
    error = gcode_mod.CommandError
    def __init__(self):
        self.scripts = []
    def run_script(self, script):
        self.scripts.append(script)
        if 'BAD' in script:
            raise gcode_mod.CommandError('boom')

class FakeDisplay:
    def __init__(self):
        self.draws = []
        self.redraws = 0
    def get_dimensions(self):
        return (20, 4)
    def draw_text(self, y, x, text, eventtime):
        self.draws.append([y, x, text, eventtime])
        return x + len(text)
    def request_redraw(self):
        self.redraws += 1

class FakeSdcard:
    def __init__(self, files):
        self.files = files
    def get_file_list(self, check_subdirs=False):
        return self.files

class FakeButtons:
    def __init__(self):
        self.log = []
        self.cbs = {}
    def register_buttons(self, pins, cb):
        self.log.append('register_buttons [%s]' % ', '.join(repr(p) for p in pins))
        for p in pins:
            self.cbs[p] = cb
    def register_button_push(self, pin, cb):
        self.log.append('register_button_push %s' % (pin,))
        self.cbs[pin] = cb
    def register_rotary_encoder(self, pin1, pin2, cw, ccw, steps):
        self.log.append('register_rotary_encoder %s %s %d' % (pin1, pin2, steps))
        self.cbs['cw'] = cw
        self.cbs['ccw'] = ccw

class FakePrinterConfig:
    def __init__(self, fail):
        self.printer = None
        self.fail = fail
        self.reads = []
    def read_config(self, filename):
        self.reads.append(filename)
        if self.fail:
            raise Exception("no such file: %s" % filename)
        return FakeConfig(self.printer, filename, {})

class FakePrinter:
    def __init__(self, fail_cfg=False):
        self.objects = {}
        self.handlers = {}
        self.events = []
        self.shutdowns = []
        self.reactor = FakeReactor()
        self.gcode = FakeGcode()
        self.pconfig = FakePrinterConfig(fail_cfg)
        self.pconfig.printer = self
        self.config_error = configparser.Error
        self.command_error = gcode_mod.CommandError
        self.objects['configfile'] = self.pconfig
        self.objects['gcode'] = self.gcode
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        if name in self.objects:
            raise Exception("duplicate object %s" % name)
        self.objects[name] = obj
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        return list(self.objects.items())
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if section == 'gcode_macro':
            obj = gcode_macro_mod.load_config(config)
        elif section == 'buttons':
            obj = FakeButtons()
        else:
            raise configparser.Error("Unknown config object '%s'" % (section,))
        self.objects[section] = obj
        return obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        self.events.append([event, list(params)])
        out = []
        for cb in self.handlers.get(event, []):
            out.append(cb(*params))
        return out
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options=None):
        self._printer = printer
        self._name = name
        self._options = dict(options or {})
        self._children = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def _wrap(self, parser, option, default, minval=None, maxval=None, above=None):
        if option not in self._options:
            if default is not _sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified"
                                     % (option, self._name))
        try:
            v = parser(self._options[option])
        except configparser.Error:
            raise
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'"
                                     % (option, self._name))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s"
                                     % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s"
                                     % (option, self._name, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s"
                                     % (option, self._name, above))
        return v
    def get(self, option, default=_sentinel, note_valid=True):
        return self._wrap(str, option, default)
    def getint(self, option, default=_sentinel, minval=None, maxval=None, note_valid=True):
        return self._wrap(int, option, default, minval, maxval)
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        return self._wrap(float, option, default, minval, maxval, above)
    def getboolean(self, option, default=_sentinel, note_valid=True):
        def parse(v):
            states = {'1': True, 'yes': True, 'true': True, 'on': True,
                      '0': False, 'no': False, 'false': False, 'off': False}
            if v.lower() not in states:
                raise ValueError('Not a boolean: %s' % v)
            return states[v.lower()]
        return self._wrap(parse, option, default)
    def getchoice(self, option, choices, default=_sentinel, note_valid=True):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default)
        else:
            c = self.get(option, default)
        if c not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s'"
                                     " is not a valid choice" % (c, option, self._name))
        return choices[c]
    def get_prefix_sections(self, prefix):
        return [c for c in self._children if c.get_name().startswith(prefix)]
    def get_prefix_options(self, prefix):
        return [o for o in self._options if o.startswith(prefix)]

def make_world(spec, files, opts, fail_cfg=False):
    p = FakePrinter(fail_cfg)
    if files is not None:
        p.objects['virtual_sdcard'] = FakeSdcard(files)
    cfg = FakeConfig(p, 'display', opts)
    cfg._children = [FakeConfig(p, name, o) for name, o in spec]
    disp = FakeDisplay()
    mgr = menu_mod.MenuManager(cfg, disp)
    return p, mgr, disp
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes (mirror the Python fakes above).
// ---------------------------------------------------------------------------

struct FakePrinterConfig {
    printer: Printer,
    fail: bool,
    reads: GCell<Vec<String>>,
}

impl PrinterConfigApi for FakePrinterConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, filename: &str) -> KResult<Config> {
        self.reads.borrow_mut().push(filename.to_string());
        if self.fail {
            return Err(KlippyError::config(format!("no such file: {filename}")));
        }
        Ok(FakeConfig::new(self.printer.clone(), filename, &[]))
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("unused")
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
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(
        &self,
        _mcu: &klippy_api::mcu::Mcu,
        _feature: &str,
        _msg: Option<String>,
    ) {
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) {}
    fn remove_section(&self, _section: &str) {}
}

klippy_api::printer_object!(FakePrinterConfig; dyn PrinterConfigApi);

/// `ConfigWrapper` fake with the option semantics and error texts of the
/// real getters, for the options the menu and `menu_keys` read.
struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
    children: Vec<Config>,
}

impl FakeConfig {
    #[allow(clippy::new_ret_no_self)]
    fn new(printer: Printer, name: &str, options: &[(&str, &str)]) -> Config {
        let options = options
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect::<IndexMap<_, _>>();
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            options,
            children: Vec::new(),
        })
    }

    fn required(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option, self.name
        ))
    }

    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{}' in section '{}'",
            option, self.name
        ))
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
        match (self.options.get(option), opts.default) {
            (Some(v), _) => Ok(Some(v.clone())),
            (None, Some(default)) => Ok(default),
            (None, None) => Err(self.required(option)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.options.get(option) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.required(option)),
            };
        };
        let value = raw.trim().parse::<i64>().map_err(|_| self.unparsable(option))?;
        if opts.minval.is_some_and(|min| value < min) {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must have minimum of {}",
                option,
                self.name,
                opts.minval.unwrap_or_default()
            )));
        }
        if opts.maxval.is_some_and(|max| value > max) {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must have maximum of {}",
                option,
                self.name,
                opts.maxval.unwrap_or_default()
            )));
        }
        Ok(Some(value))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.options.get(option) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.required(option)),
            };
        };
        let value = float_from_string(raw).map_err(|_| self.unparsable(option))?;
        if let Some(min) = opts.minval.filter(|min| value < *min) {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must have minimum of {}",
                option,
                self.name,
                float_str(min)
            )));
        }
        if let Some(max) = opts.maxval.filter(|max| value > *max) {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must have maximum of {}",
                option,
                self.name,
                float_str(max)
            )));
        }
        if let Some(above) = opts.above.filter(|above| value <= *above) {
            return Err(KlippyError::config(format!(
                "Option '{}' in section '{}' must be above {}",
                option,
                self.name,
                float_str(above)
            )));
        }
        Ok(Some(value))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let Some(raw) = self.options.get(option) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.required(option)),
            };
        };
        match raw.to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(Some(true)),
            "0" | "no" | "false" | "off" => Ok(Some(false)),
            _ => Err(self.unparsable(option)),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let entries: Vec<(PyValue, PyValue)> = match choices {
            PyValue::List(items) => items.iter().map(|i| (i.clone(), i.clone())).collect(),
            PyValue::Dict(map) => map.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            other => panic!("choices: {other:?}"),
        };
        let int_keys = matches!(entries.first(), Some((PyValue::Int(_), _)));
        let key = if int_keys {
            let default = opts.default.map(|d| {
                d.map(|v| match v {
                    PyValue::Int(n) => n.inner().to_string().parse::<i64>().expect("int default"),
                    other => panic!("int choice default: {other:?}"),
                })
            });
            let value = self.getint(
                option,
                GetIntOpts {
                    default,
                    ..Default::default()
                },
            )?;
            value.map(|v| PyValue::Int(PyInt::from_i64(v))).unwrap_or(PyValue::None)
        } else {
            let default = opts.default.map(|d| {
                d.map(|v| match v {
                    PyValue::Str(s) => s,
                    other => panic!("str choice default: {other:?}"),
                })
            });
            let value = self.get(
                option,
                GetStrOpts {
                    default,
                    ..Default::default()
                },
            )?;
            value.map(PyValue::Str).unwrap_or(PyValue::None)
        };
        match entries.iter().find(|(k, _)| k.repr() == key.repr()) {
            Some((_, value)) => Ok(value.clone()),
            None => Err(KlippyError::config(format!(
                "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                key.str(),
                option,
                self.name
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("unused")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("unused")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("unused")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("unused")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("unused")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.children
            .iter()
            .filter(|c| c.get_name().starts_with(prefix))
            .cloned()
            .collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> Vec<String> {
        self.options
            .keys()
            .filter(|k| k.starts_with(prefix))
            .cloned()
            .collect()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

/// `GCode` fake that records dispatched scripts and fails on `BAD`.
struct FakeGcodeHandle {
    scripts: GCell<Vec<String>>,
    reactor: Reactor,
}

impl GCodeApi for FakeGcodeHandle {
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
        _a: &str,
        _b: &str,
        _c: Option<&str>,
        _f: GCodeCommandHandler,
        _d: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _c: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.borrow_mut().push(script.to_string());
        if script.contains("BAD") {
            return Err(KlippyError::command("boom"));
        }
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(&self, _c: &str, _cl: &str, _p: &IndexMap<String, String>) -> GCmd {
        panic!("unused in menu tests")
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGcodeHandle; dyn GCodeApi);

/// Display fake (the `display` argument of `MenuManager(config, display)`).
struct FakeDisplay {
    cols: i64,
    rows: i64,
    draws: GCell<Vec<String>>,
    redraws: GCell<i64>,
}

impl PrinterLCDApi for FakeDisplay {
    fn get_dimensions(&self) -> (i64, i64) {
        (self.cols, self.rows)
    }
    fn draw_text(&self, y: i64, x: i64, text: &str, eventtime: f64) -> KResult<i64> {
        self.draws
            .borrow_mut()
            .push(format!("{} {} {} {}", y, x, text, float_str(eventtime)));
        Ok(x + text.chars().count() as i64)
    }
    fn request_redraw(&self) {
        *self.redraws.borrow_mut() += 1;
    }
}

/// `virtual_sdcard` fake.
struct FakeSdcard {
    files: Vec<(String, u64)>,
}

impl VirtualSDApi for FakeSdcard {
    fn get_file_list(&self, _check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        Ok(self.files.clone())
    }
    fn is_active(&self) -> bool {
        panic!("unused")
    }
    fn do_pause(&self) {
        panic!("unused")
    }
    fn do_resume(&self) -> KResult<()> {
        panic!("unused")
    }
    fn do_cancel(&self) -> KResult<()> {
        panic!("unused")
    }
    fn is_cmd_from_sd(&self) -> bool {
        panic!("unused")
    }
    fn get_file_position(&self) -> i64 {
        panic!("unused")
    }
    fn set_file_position(&self, _pos: i64) {
        panic!("unused")
    }
}

klippy_api::printer_object!(FakeSdcard; dyn VirtualSDApi);

/// `buttons` fake: logs registrations and keeps the callbacks.
#[derive(Default)]
struct FakeButtons {
    log: GCell<Vec<String>>,
    buttons: GCell<IndexMap<String, ButtonCallback>>,
    pushes: GCell<IndexMap<String, PushCallback>>,
}

impl PrinterButtonsApi for FakeButtons {
    fn register_adc_button(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: AdcButtonCallback,
    ) -> KResult<()> {
        panic!("unused")
    }
    fn register_debounce_button(
        &self,
        _pin: &str,
        _callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        panic!("unused")
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
        panic!("unused")
    }
    fn register_adc_button_push(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: PushCallback,
    ) -> KResult<()> {
        panic!("unused")
    }
    fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) -> KResult<()> {
        let reprs: Vec<String> = pins.iter().map(|p| str_repr(p)).collect();
        self.log
            .borrow_mut()
            .push(format!("register_buttons [{}]", reprs.join(", ")));
        for pin in pins {
            self.buttons.borrow_mut().insert(pin, callback.clone());
        }
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
        self.log.borrow_mut().push(format!(
            "register_rotary_encoder {pin1} {pin2} {steps_per_detent}"
        ));
        self.pushes.borrow_mut().insert("cw".to_string(), cw_callback);
        self.pushes.borrow_mut().insert("ccw".to_string(), ccw_callback);
        Ok(())
    }
    fn register_button_push(&self, pin: &str, callback: PushCallback) -> KResult<()> {
        self.log
            .borrow_mut()
            .push(format!("register_button_push {pin}"));
        self.pushes.borrow_mut().insert(pin.to_string(), callback);
        Ok(())
    }
}

klippy_api::printer_object!(FakeButtons; dyn PrinterButtonsApi);

#[derive(Default)]
struct PrinterInner {
    objects: IndexMap<String, ObjectRef>,
    handlers: IndexMap<String, Vec<EventCallback>>,
    events: Vec<String>,
    shutdowns: Vec<String>,
}

struct FakePrinter {
    inner: GCell<PrinterInner>,
    reactor: Reactor,
    buttons: GCell<Option<Arc<FakeButtons>>>,
}

klippy_api::printer_object!(FakePrinter);

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
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut inner = self.inner.borrow_mut();
        if inner.objects.contains_key(name) {
            return Err(KlippyError::config(format!("Duplicate object '{name}'")));
        }
        inner.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found = self.inner.borrow().objects.get(name).cloned();
        match found {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::ReturnNone => Ok(None),
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.inner
            .borrow()
            .objects
            .iter()
            .map(|(name, obj)| (name.clone(), obj.clone()))
            .collect()
    }
    fn load_object(
        &self,
        config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let existing = self.inner.borrow().objects.get(section).cloned();
        if let Some(obj) = existing {
            return Ok(Some(obj));
        }
        let obj: ObjectRef = match section {
            "gcode_macro" => Arc::new(PrinterGCodeMacro::new(config)) as ObjectRef,
            "buttons" => {
                let buttons = Arc::new(FakeButtons::default());
                *self.buttons.borrow_mut() = Some(buttons.clone());
                buttons as ObjectRef
            }
            _ => {
                return Err(KlippyError::config(format!(
                    "Unknown config object '{section}'"
                )))
            }
        };
        self.inner
            .borrow_mut()
            .objects
            .insert(section.to_string(), obj.clone());
        Ok(Some(obj))
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.inner.borrow_mut().shutdowns.push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.inner
            .borrow_mut()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.inner.borrow_mut().events.push(event.to_string());
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.display.menu".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record_for(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

/// Message text without the CPython object address (`... object at 0x7f..>`),
/// the config path of `Cannot load config` and the parser text of syntax errors.
fn message_key(class: &str, message: &str) -> String {
    if matches!(class, "SyntaxError" | "IndentationError") {
        return String::new();
    }
    if message.starts_with("Cannot load config '") {
        return "Cannot load config".to_string();
    }
    // `ast.literal_eval` names the unconverted node (`<_ast.Call object at ...>`);
    // the class and the leading text identify the failure.
    if message.starts_with("malformed node or string") {
        return "malformed node or string".to_string();
    }
    match message.find(" object at 0x") {
        Some(at) => message[..at].to_string(),
        None => message.to_string(),
    }
}

/// Compare a value or an exception (class and message) from both sides.
fn check_outcome(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    match (expected, actual) {
        (Ok(expected), Ok(actual)) => {
            if let Err(d) = assert_equal(&record_for(label), "ret", &expected, &actual) {
                panic!("{label}: {d}");
            }
        }
        (Err(expected), Err(actual)) => {
            if expected.class_name != actual.class_name() {
                panic!(
                    "{label}: class {} != {} ({:?} vs {:?})",
                    expected.class_name,
                    actual.class_name(),
                    expected.message,
                    actual.message()
                );
            }
            let want = message_key(&expected.class_name, &expected.message);
            let got = message_key(actual.class_name(), actual.message());
            if want != got {
                panic!("{label}: message {:?} != {:?}", expected.message, actual.message());
            }
        }
        (Ok(expected), Err(actual)) => panic!(
            "{label}: oracle returned {} but rust raised {}: {}",
            expected.render(),
            actual.class_name(),
            actual.message()
        ),
        (Err(expected), Ok(actual)) => panic!(
            "{label}: oracle raised {}: {} but rust returned {actual}",
            expected.class_name, expected.message
        ),
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("menu fakes");
            Some(py)
        }
        None => {
            println!("skipping display menu oracle tests: set KLIPPY_PYTHON to CPython 3.8.2");
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Menu specifications shared by both sides.
// ---------------------------------------------------------------------------

type Opts = Vec<(String, String)>;
type Spec = Vec<(String, Opts)>;
type Files = Option<Vec<(String, u64)>>;

fn opts(items: &[(&str, &str)]) -> Opts {
    items
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

fn item(name: &str, items: &[(&str, &str)]) -> (String, Opts) {
    (name.to_string(), opts(items))
}

fn standard_spec() -> Spec {
    vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main item1",
            &[("type", "command"), ("name", "Item 1"), ("gcode", "G28")],
        ),
        item(
            "menu __main input1",
            &[
                ("type", "input"),
                ("name", "Value"),
                ("input", "5.0"),
                ("input_min", "0"),
                ("input_max", "10"),
                ("input_step", "1"),
                ("gcode", "M220 S1"),
            ],
        ),
    ]
}

fn py_opts(o: &Opts) -> String {
    let body: Vec<String> = o
        .iter()
        .map(|(k, v)| format!("{}: {}", str_repr(k), str_repr(v)))
        .collect();
    format!("{{{}}}", body.join(", "))
}

fn py_spec(spec: &Spec) -> String {
    let body: Vec<String> = spec
        .iter()
        .map(|(name, o)| format!("({}, {})", str_repr(name), py_opts(o)))
        .collect();
    format!("[{}]", body.join(", "))
}

fn py_files(files: &Files) -> String {
    match files {
        None => "None".to_string(),
        Some(files) => {
            let body: Vec<String> = files
                .iter()
                .map(|(name, size)| format!("({}, {})", str_repr(name), size))
                .collect();
            format!("[{}]", body.join(", "))
        }
    }
}

/// Python statement that builds `p`, `mgr`, `disp` from the specification.
fn py_world_code(display_opts: &Opts, spec: &Spec, files: &Files, fail_cfg: bool) -> String {
    format!(
        "p, mgr, disp = make_world({}, {}, {}, {})",
        py_spec(spec),
        py_files(files),
        py_opts(display_opts),
        if fail_cfg { "True" } else { "False" }
    )
}

/// Build the Python world and run `update_context(1.0)`.
fn py_world(py: &mut pyoracle::Oracle, display_opts: &Opts, spec: &Spec, files: &Files) {
    py.exec(&py_world_code(display_opts, spec, files, false))
        .unwrap_or_else(|e| panic!("py world: {}: {}", e.class_name, e.message));
    py.exec("mgr.update_context(1.0)").expect("py update_context");
}

/// Everything a test needs from the Rust side.
struct RustWorld {
    manager: Arc<MenuManager>,
    printer: Arc<FakePrinter>,
    display: Arc<FakeDisplay>,
    gcode: Arc<FakeGcodeHandle>,
    config_reads: Arc<FakePrinterConfig>,
}

/// Build the Rust manager from the same specification. Errors of
/// `MenuManager::new` come back unchanged.
fn rust_world_result(
    display_opts: &Opts,
    spec: &Spec,
    files: &Files,
    fail_cfg: bool,
) -> KResult<RustWorld> {
    let printer = Arc::new(FakePrinter {
        inner: GCell::new(PrinterInner::default()),
        reactor: Reactor::new(false),
        buttons: GCell::new(None),
    });
    let printer_ref: Printer = printer.clone();
    let gcode = Arc::new(FakeGcodeHandle {
        scripts: GCell::new(Vec::new()),
        reactor: printer.reactor.clone(),
    });
    let display = Arc::new(FakeDisplay {
        cols: 20,
        rows: 4,
        draws: GCell::new(Vec::new()),
        redraws: GCell::new(0),
    });
    let config_reads = Arc::new(FakePrinterConfig {
        printer: printer_ref.clone(),
        fail: fail_cfg,
        reads: GCell::new(Vec::new()),
    });
    {
        let mut inner = printer.inner.borrow_mut();
        inner
            .objects
            .insert("configfile".to_string(), config_reads.clone() as ObjectRef);
        inner
            .objects
            .insert("gcode".to_string(), gcode.clone() as ObjectRef);
        if let Some(files) = files {
            let sdcard: ObjectRef = Arc::new(FakeSdcard { files: files.clone() }) as ObjectRef;
            inner.objects.insert("virtual_sdcard".to_string(), sdcard);
        }
    }
    let children: Vec<Config> = spec
        .iter()
        .map(|(name, o)| {
            let pairs: Vec<(&str, &str)> =
                o.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
            FakeConfig::new(printer_ref.clone(), name, &pairs)
        })
        .collect();
    let display_cfg: Config = Arc::new(FakeConfig {
        printer: printer_ref.clone(),
        name: "display".to_string(),
        options: display_opts.iter().cloned().collect(),
        children,
    });
    let manager = MenuManager::new(&display_cfg, display.clone())?;
    // `MenuManager.__init__` does not build the context; `begin` does.
    Ok(RustWorld {
        manager,
        printer,
        display,
        gcode,
        config_reads,
    })
}

fn rust_world(display_opts: &Opts, spec: &Spec, files: &Files) -> RustWorld {
    match rust_world_result(display_opts, spec, files, false) {
        Ok(world) => {
            world.manager.update_context(1.0).expect("update_context");
            world
        }
        Err(e) => panic!("rust world: {}: {}", e.class_name(), e.message()),
    }
}

fn rust_status_value(status: &PyValue) -> Value {
    match status {
        PyValue::Dict(map) => {
            let mut items = Vec::new();
            for (k, v) in map {
                let key = match k {
                    PyValue::Str(s) => s.clone(),
                    other => panic!("status key: {other:?}"),
                };
                items.push((Value::str(&key), py_to_value(v)));
            }
            Value::Dict(items)
        }
        other => panic!("status: {other:?}"),
    }
}

/// A `PyValue` as the comparison value (the shapes the menu context holds).
fn py_to_value(value: &PyValue) -> Value {
    match value {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.inner().to_string().parse::<i64>().expect("small int")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        other => panic!("unexpected value {other:?}"),
    }
}

/// `context['menu']` of a template context.
fn menu_dict(context: &TemplateContext) -> IndexMap<PyValue, PyValue> {
    match context.get("menu") {
        Some(TplValue::Py(PyValue::Dict(menu))) => menu.clone(),
        _ => panic!("context has no menu dict"),
    }
}

fn menu_entry(context: &TemplateContext, key: &str) -> Option<PyValue> {
    menu_dict(context).get(&PyValue::Str(key.to_string())).cloned()
}

fn input_of(context: &TemplateContext) -> f64 {
    match menu_entry(context, "input") {
        Some(PyValue::Float(f)) => f,
        _ => panic!("menu.input is not a float"),
    }
}

/// `context['menu']['input']` as the comparison value: `None` when the item
/// is not editing and its `input` template fails to evaluate.
fn input_value_of(context: &TemplateContext) -> Value {
    match menu_entry(context, "input") {
        Some(PyValue::Float(f)) => Value::float(f),
        Some(PyValue::None) => Value::Null,
        other => panic!("menu.input is not a float or None: {other:?}"),
    }
}

/// Sorted keys of `context['menu']`.
fn menu_keys(context: &TemplateContext) -> Value {
    let mut keys: Vec<String> = menu_dict(context)
        .keys()
        .map(|k| match k {
            PyValue::Str(s) => s.clone(),
            other => panic!("menu key {other:?}"),
        })
        .collect();
    keys.sort();
    str_list(&keys)
}

fn str_list(items: &[String]) -> Value {
    Value::List(items.iter().map(|s| Value::str(s)).collect())
}

fn bool_list(items: &[bool]) -> Value {
    Value::List(items.iter().map(|b| Value::Bool(*b)).collect())
}

fn opt_int(value: Option<i64>) -> Value {
    value.map(Value::int).unwrap_or(Value::Null)
}

fn opt_str(value: Option<String>) -> Value {
    value.map(|s| Value::str(&s)).unwrap_or(Value::Null)
}

fn find(world: &RustWorld, name: &str) -> Arc<MenuElement> {
    world
        .manager
        .lookup_menuitem(Some(name))
        .expect("lookup")
        .expect("item")
}

fn no_opts() -> RunScriptOpts {
    RunScriptOpts::default()
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn stripliterals_and_flat() {
    let Some(mut py) = oracle() else { return };
    for (label, input) in [
        ("flat_quoted", "\"abc\""),
        ("flat_squoted", "'abc'"),
        ("flat_plain", "plain"),
        ("flat_lone", "\""),
        ("flat_pair", "\"\""),
        ("flat_mixed", "\"a'"),
    ] {
        let expected = py.eval(&format!(
            "menu_mod.MenuManager.stripliterals({})",
            str_repr(input),
        ));
        check(label, expected, Value::str(&MenuManager::stripliterals(input)));
    }
    check(
        "asflat",
        py.eval("menu_mod.MenuManager.asflat('a\\nb')"),
        Value::str(&MenuManager::asflat("a\nb")),
    );
    check(
        "asflat_breaks",
        py.eval("menu_mod.MenuManager.asflat('a\\r\\nb\\x0bc\\x1cd\\u2028e')"),
        Value::str(&MenuManager::asflat("a\r\nb\u{b}c\u{1c}d\u{2028}e")),
    );
    check(
        "asflat_quoted_lines",
        py.eval("menu_mod.MenuManager.asflat('\"a\\nb\"')"),
        Value::str(&MenuManager::asflat("\"a\nb\"")),
    );
}

#[test]
fn timer_delay_and_status() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    check(
        "timer_delay",
        py.eval("menu_mod.TIMER_DELAY"),
        Value::float(TIMER_DELAY),
    );
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    check(
        "get_status",
        py.eval("mgr.get_status(1.5)"),
        rust_status_value(&world.manager.get_status(1.5)),
    );
    check(
        "timer_event",
        py.eval("mgr.timer_event(5.0)"),
        Value::float(world.manager.timer_event(5.0).expect("timer_event")),
    );
    // `menu_timeout` and `menu_root` come from the display section.
    let display_opts = opts(&[("menu_timeout", "7"), ("menu_root", "__main")]);
    py_world(&mut py, &display_opts, &standard_spec(), &None);
    let world = rust_world(&display_opts, &standard_spec(), &None);
    check(
        "get_status_timeout",
        py.eval("mgr.get_status(1.5)"),
        rust_status_value(&world.manager.get_status(1.5)),
    );
    // The menu is a printer object: its status reaches templates through
    // `printer.menu`.
    let menu_object = world
        .printer
        .lookup_object("menu", LookupDefault::Raise)
        .expect("lookup")
        .expect("menu object");
    check(
        "registered_status",
        py.eval("p.lookup_object('menu').get_status(2.0)"),
        rust_status_value(&menu_object.get_status(2.0).expect("menu status")),
    );
}

#[test]
fn element_basics() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    let cmd = find(&world, "__main item1");
    let base = find(&world, "__main");
    assert!(base.is_container());
    assert!(cmd.is_command());
    // `get_ns` relative paths.
    for (label, name) in [
        ("ns_self", "."),
        ("ns_child", ".child"),
        ("ns_parent", "..other"),
        ("ns_plain", "plain"),
        ("ns_space", "  .x  "),
    ] {
        let expected = py.eval(&format!(
            "mgr.menuitems['__main item1'].get_ns({})",
            str_repr(name)
        ));
        check(label, expected, Value::str(&cmd.get_ns(name)));
    }
    check(
        "render_name",
        py.eval("mgr.menuitems['__main item1'].render_name()"),
        Value::str(&cmd.render_name(false).expect("render")),
    );
    check(
        "is_enabled",
        py.eval("mgr.menuitems['__main item1'].is_enabled()"),
        Value::Bool(cmd.is_enabled().expect("enabled")),
    );
    check(
        "cursor_and_index",
        py.eval("(mgr.menuitems['__main item1'].cursor, mgr.menuitems['__main item1'].index)"),
        Value::Tuple(vec![Value::str(&cmd.cursor()), opt_int(cmd.index())]),
    );
    // Root name and children.
    let mut children = world.manager.lookup_children("__main");
    children.extend(world.manager.lookup_children("nope"));
    check(
        "children",
        py.eval("mgr.lookup_children('__main') + mgr.lookup_children('nope')"),
        str_list(&children),
    );
    // lookup unknown raises, lookup of `None` yields `None`.
    check_outcome(
        "lookup_unknown",
        py.eval("mgr.lookup_menuitem('__main nope')"),
        world
            .manager
            .lookup_menuitem(Some("__main nope"))
            .map(|v| match v {
                Some(_) => Value::str("found"),
                None => Value::Null,
            }),
    );
    check_outcome(
        "lookup_none",
        py.eval("mgr.lookup_menuitem(None)"),
        world.manager.lookup_menuitem(None).map(|v| match v {
            Some(_) => Value::str("found"),
            None => Value::Null,
        }),
    );
    // An element without a name raises when rendered.
    let kwargs = MenuItemKwargs {
        ns: Some("x".to_string()),
        ..Default::default()
    };
    let unnamed = world.manager.menuitem_from("command", &kwargs).expect("command");
    check_outcome(
        "unnamed_render",
        py.eval("menu_mod.MenuCommand(mgr, None, ns='x').render_name()"),
        unnamed.render_name(false).map(|s| Value::str(&s)),
    );
    check(
        "generated_ns_given",
        py.eval(
            "__import__('re').sub('__menu_[0-9a-f]+', '__menu_ID', \
             menu_mod.MenuCommand(mgr, None, ns='x $$ $other ${other} $__id', name='n').get_ns())",
        ),
        Value::str(&normalize_generated(
            &world
                .manager
                .menuitem_from(
                    "command",
                    &MenuItemKwargs {
                        ns: Some("x $$ $other ${other} $__id".to_string()),
                        name: Some("n".to_string()),
                        ..Default::default()
                    },
                )
                .expect("command")
                .get_ns("."),
        )),
    );
}

/// Replace the generated `__menu_<hex>` id so both sides compare equal.
fn normalize_generated(text: &str) -> String {
    let mut out = String::new();
    let mut rest = text;
    while let Some(at) = rest.find("__menu_") {
        out.push_str(&rest[..at]);
        out.push_str("__menu_ID");
        let tail = &rest[at + "__menu_".len()..];
        let hex_len = tail
            .find(|c: char| !c.is_ascii_hexdigit())
            .unwrap_or(tail.len());
        rest = &tail[hex_len..];
    }
    out.push_str(rest);
    out
}

#[test]
fn input_editing() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    let item = find(&world, "__main input1");
    assert!(item.is_input());
    assert!(!item.is_scrollable());
    item.start_editing().expect("start");
    py.exec("mgr.menuitems['__main input1'].start_editing()")
        .expect("py start_editing");
    check(
        "editing",
        py.eval("mgr.menuitems['__main input1'].is_editing()"),
        Value::Bool(item.is_editing()),
    );
    let ctx_expr = "mgr.menuitems['__main input1'].get_context()['menu']['input']";
    let ctx = item.get_context(None).expect("context");
    check("input_value", py.eval(ctx_expr), Value::float(input_of(&ctx)));
    item.inc_value(false).expect("inc");
    py.exec("mgr.menuitems['__main input1'].inc_value(False)")
        .expect("py inc_value");
    let ctx = item.get_context(None).expect("context");
    check("input_inc", py.eval(ctx_expr), Value::float(input_of(&ctx)));
    item.dec_value(false).expect("dec");
    py.exec("mgr.menuitems['__main input1'].dec_value(False)")
        .expect("py dec_value");
    let ctx = item.get_context(None).expect("context");
    check("input_dec", py.eval(ctx_expr), Value::float(input_of(&ctx)));
    // Clamped at the bounds, with and without the fast rate.
    for _ in 0..12 {
        item.inc_value(false).expect("inc");
        py.exec("mgr.menuitems['__main input1'].inc_value(False)")
            .expect("py inc_value");
    }
    let ctx = item.get_context(None).expect("context");
    check("input_max", py.eval(ctx_expr), Value::float(input_of(&ctx)));
    item.dec_value(true).expect("dec fast");
    py.exec("mgr.menuitems['__main input1'].dec_value(True)")
        .expect("py dec_value fast");
    let ctx = item.get_context(None).expect("context");
    check("input_fast_dec", py.eval(ctx_expr), Value::float(input_of(&ctx)));
    item.stop_editing().expect("stop");
    py.exec("mgr.menuitems['__main input1'].stop_editing()")
        .expect("py stop_editing");
    check(
        "stopped",
        py.eval("mgr.menuitems['__main input1'].is_editing()"),
        Value::Bool(item.is_editing()),
    );
    // Without an edit session the step calls do nothing.
    item.inc_value(false).expect("inc idle");
    py.exec("mgr.menuitems['__main input1'].inc_value(False)")
        .expect("py inc idle");
    check(
        "idle_step",
        py.eval("mgr.menuitems['__main input1'].is_editing()"),
        Value::Bool(item.is_editing()),
    );
}

#[test]
fn input_fast_rate_step() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main wide",
            &[
                ("type", "input"),
                ("name", "Wide"),
                ("input", "5"),
                ("input_min", "0"),
                ("input_max", "1000"),
                ("input_step", "1"),
            ],
        ),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    let wide = find(&world, "__main wide");
    py.exec("w = mgr.menuitems['__main wide']; w.start_editing()")
        .expect("py start");
    wide.start_editing().expect("start");
    let ctx_expr = "w.get_context()['menu']['input']";
    for (label, py_code, fast, inc) in [
        ("fast_inc", "w.inc_value(True)", true, true),
        ("fast_inc2", "w.inc_value(True)", true, true),
        ("slow_inc", "w.inc_value(False)", false, true),
        ("fast_dec", "w.dec_value(True)", true, false),
        ("slow_dec", "w.dec_value(False)", false, false),
    ] {
        py.exec(py_code).expect("py step");
        if inc {
            wide.inc_value(fast).expect("inc");
        } else {
            wide.dec_value(fast).expect("dec");
        }
        let ctx = wide.get_context(None).expect("context");
        check(label, py.eval(ctx_expr), Value::float(input_of(&ctx)));
    }
}

#[test]
fn input_template_errors() {
    let Some(mut py) = oracle() else { return };
    // (input, input_min, input_max) per case; the templates are plain text
    // that renders to itself and goes through `ast.literal_eval`.
    let cases: [(&str, &str, &str); 14] = [
        ("5", "0", "10"),
        ("abc", "0", "10"),
        ("'7'", "0", "10"),
        ("[1]", "0", "10"),
        ("'x'", "0", "10"),
        ("5", "abc", "10"),
        ("5", "0", "abc"),
        ("50", "0", "10"),
        ("-5", "0", "10"),
        ("1e400", "0", "10"),
        ("1 2", "0", "10"),
        ("5", "[]", "10"),
        ("None", "0", "10"),
        ("True", "0", "10"),
    ];
    let mut spec: Spec = vec![item("menu __main", &[("type", "list"), ("name", "Main")])];
    for (i, (input, min, max)) in cases.iter().enumerate() {
        spec.push(item(
            &format!("menu __main in{i}"),
            &[
                ("type", "input"),
                ("name", "In"),
                ("input", input),
                ("input_min", min),
                ("input_max", max),
                ("input_step", "1"),
            ],
        ));
    }
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    for i in 0..cases.len() {
        let name = format!("__main in{i}");
        let element = find(&world, &name);
        let expected = py.eval(&format!(
            "(lambda it: (it.start_editing(), it.get_context()['menu']['input'])[1])(mgr.menuitems['{name}'])"
        ));
        let actual = element
            .start_editing()
            .and_then(|_| element.get_context(None))
            .map(|ctx| input_value_of(&ctx));
        check_outcome(&format!("input_case_{i}"), expected, actual);
        check(
            &format!("input_case_{i}_editing"),
            py.eval(&format!("mgr.menuitems['{name}'].is_editing()")),
            Value::Bool(element.is_editing()),
        );
        // A failed start leaves the bounds the evaluation stored; stepping
        // reports the same error (or moves the same value) on both sides.
        let expected = py.eval(&format!(
            "(lambda it: (it.inc_value(True), it.get_context()['menu']['input'])[1])(mgr.menuitems['{name}'])"
        ));
        let actual = element
            .inc_value(true)
            .and_then(|_| element.get_context(None))
            .map(|ctx| input_value_of(&ctx));
        check_outcome(&format!("input_case_{i}_step"), expected, actual);
    }
}

#[test]
fn enable_templates_use_literal_eval() {
    let Some(mut py) = oracle() else { return };
    // Texts the `enable` template renders to; each goes through
    // `ast.literal_eval` and `bool()` (menu.py:113-116). `true`, `false`,
    // `1`, `0` and the like are read by `getboolean` and never reach it.
    let cases: [&str; 38] = [
        "True",
        "False",
        "None",
        "2",
        "-1",
        "+1",
        "1.5",
        "0.0",
        "1e3",
        ".5",
        "5.",
        "0x10",
        "0b0",
        "0o17",
        "1e400",
        "'x'",
        "''",
        "\"a\"",
        "'a' 'b'",
        "b''",
        "b'x'",
        "'a\\x41'",
        "[]",
        "[0]",
        "()",
        "(1,)",
        "(1)",
        "abc",
        "1+2",
        "not 1",
        "f(1)",
        "x.y",
        "[x]",
        "1 if 1 else 2",
        "[x for x in y]",
        "",
        " 1",
        "1 2",
    ];
    let mut spec: Spec = vec![item("menu __main", &[("type", "list"), ("name", "Main")])];
    for (i, text) in cases.iter().enumerate() {
        spec.push(item(
            &format!("menu __main e{i}"),
            &[("type", "command"), ("name", "E"), ("enable", text)],
        ));
    }
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    for (i, text) in cases.iter().enumerate() {
        let name = format!("__main e{i}");
        let expected = py.eval(&format!("mgr.menuitems['{name}'].is_enabled()"));
        let actual = find(&world, &name).is_enabled().map(Value::Bool);
        check_outcome(&format!("enable_{i}_{text:?}"), expected, actual);
    }
    // The plain `enable` options read by `getboolean`.
    let mut spec: Spec = vec![item("menu __main", &[("type", "list"), ("name", "Main")])];
    for (i, text) in ["true", "false", "yes", "off", "ON", "No"].iter().enumerate() {
        spec.push(item(
            &format!("menu __main b{i}"),
            &[("type", "command"), ("name", "B"), ("enable", text)],
        ));
    }
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    for i in 0..6 {
        let name = format!("__main b{i}");
        check(
            &format!("bool_enable_{i}"),
            py.eval(&format!("mgr.menuitems['{name}'].is_enabled()")),
            Value::Bool(find(&world, &name).is_enabled().expect("enabled")),
        );
    }
}

#[test]
fn navigation_and_press() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    // Same begin/down/press sequence on both sides.
    py.exec(
        "mgr.begin(1.0); \
         mgr.down(False); \
         sel = mgr.menuitems['__main'].selected; \
         mgr.press('click'); \
         mgr.dispatch_gcode(1.0)",
    )
    .expect("py navigation");
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    world.manager.begin(1.0).expect("begin");
    world.manager.down(false).expect("down");
    let selected = world.manager.stack_peek(0).and_then(|c| c.selected());
    world.manager.press("click").expect("press");
    world.manager.dispatch_gcode();
    let recorded = world.gcode.scripts.borrow().clone();
    check(
        "selection_and_gcode",
        py.eval("(sel, p.gcode.scripts)"),
        Value::Tuple(vec![opt_int(selected), str_list(&recorded)]),
    );
    check(
        "running",
        py.eval("mgr.is_running()"),
        Value::Bool(world.manager.is_running()),
    );
    // Every further press and up/down/back step ends at the same place.
    py.exec(
        "mgr.up(False); mgr.up(False); mgr.up(True); mgr.down(True); mgr.down(False); \
         states = [(mgr.stack_size(), mgr.stack_peek().selected)]; \
         mgr.press('long_click'); mgr.dispatch_gcode(2.0); \
         mgr.back(); states.append((mgr.stack_size(), mgr.is_running()))",
    )
    .expect("py steps");
    world.manager.up(false).expect("up");
    world.manager.up(false).expect("up");
    world.manager.up(true).expect("fast up");
    world.manager.down(true).expect("fast down");
    world.manager.down(false).expect("down");
    let mut states = vec![Value::Tuple(vec![
        Value::int(world.manager.stack_size() as i64),
        opt_int(world.manager.stack_peek(0).and_then(|c| c.selected())),
    ])];
    world.manager.press("long_click").expect("long press");
    world.manager.dispatch_gcode();
    world.manager.back(false, true).expect("back");
    states.push(Value::Tuple(vec![
        Value::int(world.manager.stack_size() as i64),
        Value::Bool(world.manager.is_running()),
    ]));
    check("steps", py.eval("states"), Value::List(states));
    check(
        "scripts_after_steps",
        py.eval("p.gcode.scripts"),
        str_list(&world.gcode.scripts.borrow().clone()),
    );
}

#[test]
fn input_press_and_realtime() {
    let Some(mut py) = oracle() else { return };
    let mut spec = standard_spec();
    spec.push(item(
        "menu __main rt",
        &[
            ("type", "input"),
            ("name", "RT"),
            ("input", "1"),
            ("input_min", "0"),
            ("input_max", "100"),
            ("input_step", "1"),
            ("realtime", "true"),
            ("gcode", "M221 S1"),
        ],
    ));
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    // Select the first input, click to edit, step up, click to run the
    // script and leave the editor.
    py.exec(
        "mgr.begin(1.0); mgr.down(False); mgr.down(False); \
         states = [mgr.menuitems['__main input1'].is_editing()]; \
         mgr.press('click'); \
         states.append(mgr.menuitems['__main input1'].is_editing()); \
         mgr.down(False); \
         vals = [mgr.menuitems['__main input1'].get_context()['menu']['input']]; \
         mgr.press('click'); \
         states.append(mgr.menuitems['__main input1'].is_editing()); \
         mgr.dispatch_gcode(1.0)",
    )
    .expect("py press flow");
    let input1 = find(&world, "__main input1");
    world.manager.begin(1.0).expect("begin");
    world.manager.down(false).expect("down");
    world.manager.down(false).expect("down");
    let mut states = vec![input1.is_editing()];
    world.manager.press("click").expect("press");
    states.push(input1.is_editing());
    world.manager.down(false).expect("down");
    let vals = [input_of(&input1.get_context(None).expect("context"))];
    world.manager.press("click").expect("press");
    states.push(input1.is_editing());
    world.manager.dispatch_gcode();
    check("press_states", py.eval("states"), bool_list(&states));
    check(
        "press_values",
        py.eval("vals"),
        Value::List(vals.iter().map(|v| Value::float(*v)).collect()),
    );
    check(
        "press_scripts",
        py.eval("p.gcode.scripts"),
        str_list(&world.gcode.scripts.borrow().clone()),
    );
    // The realtime input runs its script from `heartbeat` once the value
    // changed and 0.25 s passed.
    py.exec(
        "rt = mgr.menuitems['__main rt']; rt.start_editing(); rt.heartbeat(10.0); \
         rt.inc_value(); rt.heartbeat(10.1); rt.heartbeat(11.0); mgr.dispatch_gcode(0.0)",
    )
    .expect("py realtime");
    let rt = find(&world, "__main rt");
    rt.start_editing().expect("start");
    rt.heartbeat(10.0).expect("heartbeat");
    rt.inc_value(false).expect("inc");
    rt.heartbeat(10.1).expect("heartbeat");
    rt.heartbeat(11.0).expect("heartbeat");
    world.manager.dispatch_gcode();
    check(
        "realtime_scripts",
        py.eval("p.gcode.scripts"),
        str_list(&world.gcode.scripts.borrow().clone()),
    );
}

#[test]
fn run_script_results_and_scroller() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main cmd",
            &[
                ("type", "command"),
                ("name", "Ab~x~cdef"),
                ("gcode", "G1 X1"),
                ("enter", "ENTERED"),
            ],
        ),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    let cmd = find(&world, "__main cmd");
    // `run_script` with and without a known script, queued or render only.
    py.exec(
        "c = mgr.menuitems['__main cmd']; \
         rs = [c.run_script('gcode', render_only=True), c.run_script('nope', render_only=True), \
               c.run_script('gcode', event='click', render_only=True), c.run_script('gcode'), \
               c.run_script('gcode', event='', render_only=True)]; \
         scripts = [c.get_script('gcode') is not None, c.get_script('nope') is not None]",
    )
    .expect("py run_script");
    let render_only = |event: Option<&str>| RunScriptOpts {
        event: event.map(str::to_string),
        render_only: true,
        ..Default::default()
    };
    let results = vec![
        opt_str(cmd.run_script("gcode", &render_only(None)).expect("gcode")),
        opt_str(cmd.run_script("nope", &render_only(None)).expect("nope")),
        opt_str(cmd.run_script("gcode", &render_only(Some("click"))).expect("click")),
        opt_str(cmd.run_script("gcode", &no_opts()).expect("queued")),
        opt_str(cmd.run_script("gcode", &render_only(Some(""))).expect("empty event")),
    ];
    check("run_script_results", py.eval("rs"), Value::List(results));
    check(
        "get_script",
        py.eval("scripts"),
        Value::List(vec![
            Value::Bool(cmd.get_script("gcode").is_some()),
            Value::Bool(cmd.get_script("nope").is_some()),
        ]),
    );
    py.exec("mgr.dispatch_gcode(0.0)").expect("py dispatch");
    world.manager.dispatch_gcode();
    check(
        "queued_script",
        py.eval("p.gcode.scripts"),
        str_list(&world.gcode.scripts.borrow().clone()),
    );
    // The scroller: a pending request moves the position on each half
    // second heartbeat, and `~...~` glyph placeholders count as one chunk.
    py.exec(
        "names = [c.render_name(True)]; \
         c.need_scroller(True); c.heartbeat(1.0); names.append(c.render_name(True)); \
         c.need_scroller(True); c.heartbeat(1.2); names.append(c.render_name(True)); \
         c.heartbeat(1.6); names.append(c.render_name(True)); \
         c.need_scroller(True); c.heartbeat(2.2); names.append(c.render_name(True)); \
         c.need_scroller(True); c.heartbeat(3.0); names.append(c.render_name(True)); \
         c.need_scroller(None); c.heartbeat(4.0); names.append(c.render_name(True)); \
         c.need_scroller(True); c.heartbeat(5.0); c.select(); names.append(c.render_name(True)); \
         c.need_scroller(True); c.heartbeat(6.0); names.append(c.render_name(False)); \
         names.append(c.render_name(True)); \
         names.append(c._MenuElement__slice_name('Ab~x~cdef', 0)); \
         names.append(c._MenuElement__slice_name('Ab~x~cdef', 3)); \
         names.append(c._MenuElement__slice_name('Ab~x~cdef', 40)); \
         names.append(c._MenuElement__slice_name('~a~~b~', 1))",
    )
    .expect("py scroller");
    let mut names = vec![cmd.render_name(true).expect("name")];
    cmd.need_scroller(Some(true));
    cmd.heartbeat(1.0).expect("heartbeat");
    names.push(cmd.render_name(true).expect("name"));
    cmd.need_scroller(Some(true));
    cmd.heartbeat(1.2).expect("heartbeat");
    names.push(cmd.render_name(true).expect("name"));
    cmd.heartbeat(1.6).expect("heartbeat");
    names.push(cmd.render_name(true).expect("name"));
    cmd.need_scroller(Some(true));
    cmd.heartbeat(2.2).expect("heartbeat");
    names.push(cmd.render_name(true).expect("name"));
    cmd.need_scroller(Some(true));
    cmd.heartbeat(3.0).expect("heartbeat");
    names.push(cmd.render_name(true).expect("name"));
    cmd.need_scroller(None);
    cmd.heartbeat(4.0).expect("heartbeat");
    names.push(cmd.render_name(true).expect("name"));
    cmd.need_scroller(Some(true));
    cmd.heartbeat(5.0).expect("heartbeat");
    cmd.select();
    names.push(cmd.render_name(true).expect("name"));
    cmd.need_scroller(Some(true));
    cmd.heartbeat(6.0).expect("heartbeat");
    names.push(cmd.render_name(false).expect("name"));
    names.push(cmd.render_name(true).expect("name"));
    names.push(cmd.slice_name("Ab~x~cdef", 0));
    names.push(cmd.slice_name("Ab~x~cdef", 3));
    names.push(cmd.slice_name("Ab~x~cdef", 40));
    names.push(cmd.slice_name("~a~~b~", 1));
    check("scroller_names", py.eval("names"), str_list(&names));
}

#[test]
fn disabled_item_is_never_enabled() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    py.exec(
        "c = FakeConfig(p, 'menu __main off', {'type': 'disabled', 'name': 'Off'}); \
         off = menu_mod.MenuDisabled(mgr, c); \
         mgr.add_menuitem(off.get_ns(), off)",
    )
    .expect("py disabled");
    let kwargs = MenuItemKwargs {
        name: Some("ignored".to_string()),
        ..Default::default()
    };
    let off = world.manager.menuitem_from("disabled", &kwargs).expect("disabled");
    check(
        "disabled",
        py.eval("mgr.menuitems['__main off'].is_enabled()"),
        Value::Bool(off.is_enabled().expect("enabled")),
    );
    // `MenuDisabled` drops its keyword arguments and renders `''`.
    check(
        "disabled_name",
        py.eval("menu_mod.MenuDisabled(mgr, None, name='ignored').render_name()"),
        Value::str(&off.render_name(false).expect("name")),
    );
    // A disabled section in the config never shows in a list.
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item("menu __main off", &[("type", "disabled"), ("name", "Off")]),
        item("menu __main on", &[("type", "command"), ("name", "On")]),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    py.exec("mgr.begin(1.0); names = [i.render_name() for i in mgr.stack_peek()]")
        .expect("py begin");
    let world = rust_world(&opts(&[]), &spec, &None);
    world.manager.begin(1.0).expect("begin");
    let top = world.manager.stack_peek(0).expect("top");
    let names: Vec<String> = top
        .iter_items()
        .iter()
        .map(|i| i.render_name(false).expect("name"))
        .collect();
    check("disabled_hidden", py.eval("names"), str_list(&names));
}

#[test]
fn unknown_type_errors() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    let kwargs = MenuItemKwargs::default();
    check_outcome(
        "bad_choice",
        py.eval("mgr.menuitem_from('nope')"),
        world
            .manager
            .menuitem_from("nope", &kwargs)
            .map(|_| Value::str("ok")),
    );
}

#[test]
fn constructor_errors() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Opts, Spec)> = vec![
        (
            "unknown_root",
            opts(&[("menu_root", "nope")]),
            standard_spec(),
        ),
        (
            "bad_timeout",
            opts(&[("menu_timeout", "soon")]),
            standard_spec(),
        ),
        (
            "bad_reverse",
            opts(&[("menu_reverse_navigation", "maybe")]),
            standard_spec(),
        ),
        (
            "bad_type",
            opts(&[]),
            vec![item("menu __main bad", &[("type", "wrong"), ("name", "X")])],
        ),
        (
            "missing_type",
            opts(&[]),
            vec![item("menu __main bad", &[("name", "X")])],
        ),
        (
            "missing_name",
            opts(&[]),
            vec![item("menu __main bad", &[("type", "command")])],
        ),
        (
            "bad_index",
            opts(&[]),
            vec![item(
                "menu __main bad",
                &[("type", "command"), ("name", "X"), ("index", "first")],
            )],
        ),
        (
            "zero_step",
            opts(&[]),
            vec![item(
                "menu __main bad",
                &[
                    ("type", "input"),
                    ("name", "X"),
                    ("input", "1"),
                    ("input_step", "0"),
                ],
            )],
        ),
        (
            "missing_input",
            opts(&[]),
            vec![item("menu __main bad", &[("type", "input"), ("name", "X")])],
        ),
        (
            "bad_realtime",
            opts(&[]),
            vec![item(
                "menu __main bad",
                &[
                    ("type", "input"),
                    ("name", "X"),
                    ("input", "1"),
                    ("realtime", "sometimes"),
                ],
            )],
        ),
        (
            "bad_encoder_detent",
            opts(&[("encoder_steps_per_detent", "3")]),
            standard_spec(),
        ),
    ];
    for (label, display_opts, spec) in cases {
        let expected = py.exec(&py_world_code(&display_opts, &spec, &None, false));
        let actual = rust_world_result(&display_opts, &spec, &None, false).map(|_| Value::Null);
        check_outcome(label, expected, actual);
    }
    // A `menu.cfg` that cannot be read.
    let expected = py.exec(&py_world_code(&opts(&[]), &standard_spec(), &None, true));
    let actual =
        rust_world_result(&opts(&[]), &standard_spec(), &None, true).map(|_| Value::Null);
    check_outcome("menu_cfg_unreadable", expected, actual);
}

#[test]
fn bundled_config_is_read_once() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    let reads = world.config_reads.reads.borrow().clone();
    check(
        "menu_cfg_reads",
        py.eval(
            "[r.replace('\\\\', '/').endswith('klippy/extras/display/menu.cfg') \
             for r in p.pconfig.reads]",
        ),
        bool_list(
            &reads
                .iter()
                .map(|r| r.replace('\\', "/").ends_with("klippy/extras/display/menu.cfg"))
                .collect::<Vec<_>>(),
        ),
    );
    // The manager registers itself and a ready handler.
    check(
        "registered_objects",
        py.eval("sorted(n for n in p.objects if n in ('menu', 'gcode_macro', 'buttons'))"),
        {
            let mut names: Vec<String> = world
                .printer
                .lookup_objects(None)
                .into_iter()
                .map(|(n, _)| n)
                .filter(|n| matches!(n.as_str(), "menu" | "gcode_macro" | "buttons"))
                .collect();
            names.sort();
            str_list(&names)
        },
    );
    check(
        "ready_handler",
        py.eval("sorted(p.handlers)"),
        str_list(&{
            let mut events: Vec<String> =
                world.printer.inner.borrow().handlers.keys().cloned().collect();
            events.sort();
            events
        }),
    );
}

#[test]
fn children_index_order() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item("menu __main a", &[("type", "command"), ("name", "A")]),
        item("menu __main b", &[("type", "command"), ("name", "B"), ("index", "0")]),
        item("menu __main c", &[("type", "command"), ("name", "C"), ("index", "9")]),
        item("menu __main d", &[("type", "command"), ("name", "D"), ("index", "-2")]),
        item("menu __main e", &[("type", "command"), ("name", "E"), ("index", "-9")]),
        // A redeclared name keeps the first position.
        item("menu __main a", &[("type", "command"), ("name", "A again")]),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    check(
        "children_order",
        py.eval("mgr.lookup_children('__main')"),
        str_list(&world.manager.lookup_children("__main")),
    );
    check(
        "redeclared_name",
        py.eval("mgr.menuitems['__main a'].render_name()"),
        Value::str(&find(&world, "__main a").render_name(false).expect("name")),
    );
}

#[test]
fn container_selection_and_lookup() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item("menu __main sub", &[("type", "list"), ("name", "Sub")]),
        item("menu __main sub deep", &[("type", "command"), ("name", "Deep")]),
        item("menu __main sub deep2", &[("type", "command"), ("name", "Deep2")]),
        item("menu __main leaf", &[("type", "command"), ("name", "Leaf")]),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    py.exec(
        "m = mgr.menuitems['__main']; sub = mgr.menuitems['__main sub']; \
         leaf = mgr.menuitems['__main leaf']; deep2 = mgr.menuitems['__main sub deep2']; \
         m.populate(); m.update_items(); sub.populate(); sub.update_items()",
    )
    .expect("py populate");
    let main = find(&world, "__main");
    let sub = find(&world, "__main sub");
    let leaf = find(&world, "__main leaf");
    let deep2 = find(&world, "__main sub deep2");
    main.populate().expect("populate");
    main.update_items().expect("update_items");
    sub.populate().expect("populate");
    sub.update_items().expect("update_items");
    let names = |c: &Arc<MenuElement>| -> Value {
        str_list(
            &c.iter_items()
                .iter()
                .map(|i| i.render_name(false).expect("name"))
                .collect::<Vec<_>>(),
        )
    };
    check("main_items", py.eval("[i.render_name() for i in m]"), names(&main));
    check("sub_items", py.eval("[i.render_name() for i in sub]"), names(&sub));
    check(
        "len",
        py.eval("(len(m), len(sub))"),
        Value::Tuple(vec![Value::int(main.len() as i64), Value::int(sub.len() as i64)]),
    );
    // Index lookups by name and element, with and without look_inside.
    let by_name = LookupRef::Named("  __main leaf ".to_string());
    let by_leaf = LookupRef::Element(leaf.clone());
    let by_deep = LookupRef::Element(deep2.clone());
    let by_sub = LookupRef::Element(sub.clone());
    check(
        "index_of",
        py.eval(
            "(m.index_of('  __main leaf '), m.index_of(leaf), m.index_of(deep2), \
              m.index_of(deep2, True), m.index_of(sub), sub.index_of(deep2), \
              m._index_of('nope'), m.index_of(m, True))",
        ),
        Value::Tuple(vec![
            opt_int(main.index_of(&by_name, false)),
            opt_int(main.index_of(&by_leaf, false)),
            opt_int(main.index_of(&by_deep, false)),
            opt_int(main.index_of(&by_deep, true)),
            opt_int(main.index_of(&by_sub, false)),
            opt_int(sub.index_of(&by_deep, false)),
            opt_int(main.index_of_item(&LookupRef::Named("nope".to_string()))),
            opt_int(main.index_of(&LookupRef::Element(main.clone()), true)),
        ]),
    );
    // Selection moves and clamps.
    py.exec(
        "sels = [m.selected, m.init_selection(), m.selected]; \
         [m.select_next() for _ in range(5)]; sels.append(m.selected); \
         [m.select_prev() for _ in range(5)]; sels.append(m.selected); \
         sels.append(m.select_item(leaf)); sels.append(m.select_item(deep2)); \
         sels.append(m.selected_item() is leaf); \
         m.select_at(None); sels.append(m.selected_item() is None); \
         m.select_next(); sels.append(m.selected); \
         m.select_at(None); m.select_prev(); sels.append(m.selected); \
         m.select_at(99); sels.append(m.selected_item() is None)",
    )
    .expect("py selection");
    let mut sels = vec![
        opt_int(main.selected()),
        {
            main.init_selection();
            Value::Null
        },
        opt_int(main.selected()),
    ];
    for _ in 0..5 {
        main.select_next();
    }
    sels.push(opt_int(main.selected()));
    for _ in 0..5 {
        main.select_prev();
    }
    sels.push(opt_int(main.selected()));
    sels.push(opt_int(main.select_item(&leaf)));
    sels.push(opt_int(main.select_item(&deep2)));
    sels.push(Value::Bool(
        main.selected_item().is_some_and(|i| Arc::ptr_eq(&i, &leaf)),
    ));
    main.select_at(None);
    sels.push(Value::Bool(main.selected_item().is_none()));
    main.select_next();
    sels.push(opt_int(main.selected()));
    main.select_at(None);
    main.select_prev();
    sels.push(opt_int(main.selected()));
    main.select_at(Some(99));
    sels.push(Value::Bool(main.selected_item().is_none()));
    check("selection", py.eval("sels"), Value::List(sels));
    // A container may not contain itself; unknown names are rejected.
    check_outcome(
        "recursive_relation",
        py.eval("m.insert_item(m)"),
        main.insert_item(&LookupRef::Element(main.clone()), None)
            .map(|_| Value::Null),
    );
    check_outcome(
        "insert_unknown",
        py.eval("m.insert_item('nope')"),
        main.insert_item(&LookupRef::Named("nope".to_string()), None)
            .map(|_| Value::Null),
    );
    // Inserting at an index puts the item first; a negative index counts
    // from the end.
    py.exec(
        "m.insert_item(deep2, 0); m.insert_item(leaf, -1); m.update_items(); \
         order = [i.render_name() for i in m]",
    )
    .expect("py insert at index");
    main.insert_item(&LookupRef::Element(deep2.clone()), Some(0))
        .expect("insert");
    main.insert_item(&LookupRef::Element(leaf.clone()), Some(-1))
        .expect("insert");
    main.update_items().expect("update_items");
    check("insert_at_index", py.eval("order"), names(&main));
}

#[test]
fn key_events_redraw() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    py.exec(
        "mgr.begin(1.0); mgr.key_event('down', 2.0); mgr.key_event('bogus', 2.0); \
         mgr.key_event('fast_down', 2.0); mgr.key_event('up', 2.0); mgr.key_event('fast_up', 2.0); \
         mgr.key_event('back', 2.0)",
    )
    .expect("py keys");
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    world.manager.begin(1.0).expect("begin");
    for key in ["down", "bogus", "fast_down", "up", "fast_up", "back"] {
        world.manager.key_event(key, 2.0).expect(key);
    }
    check(
        "redraws",
        py.eval("disp.redraws"),
        Value::int(*world.display.redraws.borrow()),
    );
    check(
        "running_after_back",
        py.eval("mgr.is_running()"),
        Value::Bool(world.manager.is_running()),
    );
    // A click starts the menu and a second click presses the selection.
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    py.exec(
        "mgr.key_event('click', 3.0); r = [mgr.is_running()]; mgr.key_event('click', 4.0); \
         mgr.key_event('long_click', 5.0); mgr.dispatch_gcode(0.0); r.append(mgr.is_running())",
    )
    .expect("py clicks");
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    world.manager.key_event("click", 3.0).expect("click");
    let mut running = vec![world.manager.is_running()];
    world.manager.key_event("click", 4.0).expect("click");
    world.manager.key_event("long_click", 5.0).expect("long click");
    world.manager.dispatch_gcode();
    running.push(world.manager.is_running());
    check("click_start", py.eval("r"), bool_list(&running));
    check(
        "click_scripts",
        py.eval("p.gcode.scripts"),
        str_list(&world.gcode.scripts.borrow().clone()),
    );
}

#[test]
fn events_and_status_after_begin() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    py.exec("mgr.begin(1.0)").expect("py begin");
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    world.manager.begin(1.0).expect("begin");
    let actual = world.printer.inner.borrow().events.clone();
    check("events", py.eval("[e[0] for e in p.events]"), str_list(&actual));
    check(
        "status",
        py.eval("mgr.get_status(1.0)"),
        rust_status_value(&world.manager.get_status(1.0)),
    );
    // Leaving through `exit` sends the exit event and stops the menu.
    py.exec("mgr.exit(); mgr.exit(True)").expect("py exit");
    world.manager.exit(false).expect("exit");
    world.manager.exit(true).expect("exit");
    let actual = world.printer.inner.borrow().events.clone();
    check(
        "events_after_exit",
        py.eval("[e[0] for e in p.events]"),
        str_list(&actual),
    );
    check(
        "running_after_exit",
        py.eval("mgr.is_running()"),
        Value::Bool(world.manager.is_running()),
    );
}

#[test]
fn invalid_root_stops_the_menu() {
    let Some(mut py) = oracle() else { return };
    // A root that is no container logs an error and leaves the menu stopped.
    let display_opts = opts(&[("menu_root", "__main item1")]);
    py_world(&mut py, &display_opts, &standard_spec(), &None);
    py.exec("mgr.begin(1.0); mgr.key_event('click', 2.0)")
        .expect("py begin");
    let world = rust_world(&display_opts, &standard_spec(), &None);
    world.manager.begin(1.0).expect("begin");
    world.manager.key_event("click", 2.0).expect("click");
    check(
        "invalid_root_running",
        py.eval("(mgr.is_running(), mgr.stack_size())"),
        Value::Tuple(vec![
            Value::Bool(world.manager.is_running()),
            Value::int(world.manager.stack_size() as i64),
        ]),
    );
    check(
        "invalid_root_events",
        py.eval("[e[0] for e in p.events]"),
        str_list(&world.printer.inner.borrow().events.clone()),
    );
    // Pressing, navigating and drawing do nothing while stopped.
    py.exec("mgr.press('click'); mgr.up(); mgr.down(); mgr.back(); mgr.exit()")
        .expect("py idle");
    world.manager.press("click").expect("press");
    world.manager.up(false).expect("up");
    world.manager.down(false).expect("down");
    world.manager.back(false, true).expect("back");
    world.manager.exit(false).expect("exit");
    check(
        "idle_screen_update",
        py.eval("mgr.screen_update_event(2.0)"),
        Value::Bool(world.manager.screen_update_event(2.0).expect("update")),
    );
}

#[test]
fn timeout_and_reverse_navigation() {
    let Some(mut py) = oracle() else { return };
    let display_opts = opts(&[("menu_timeout", "3"), ("menu_reverse_navigation", "true")]);
    py_world(&mut py, &display_opts, &standard_spec(), &None);
    let world = rust_world(&display_opts, &standard_spec(), &None);
    py.exec(
        "mgr.begin(1.0); mgr.down(False); sel_down = mgr.menuitems['__main'].selected; \
         mgr.up(False); sel_up = mgr.menuitems['__main'].selected; \
         running = [mgr.timeout_check(1.0) or mgr.is_running() for _ in range(6)]",
    )
    .expect("py timeout");
    world.manager.begin(1.0).expect("begin");
    world.manager.down(false).expect("down");
    let sel_down = world.manager.stack_peek(0).and_then(|c| c.selected());
    world.manager.up(false).expect("up");
    let sel_up = world.manager.stack_peek(0).and_then(|c| c.selected());
    let mut running = Vec::new();
    for _ in 0..6 {
        world.manager.timeout_check(1.0).expect("timeout_check");
        running.push(world.manager.is_running());
    }
    check(
        "reverse_selection",
        py.eval("(sel_down, sel_up)"),
        Value::Tuple(vec![opt_int(sel_down), opt_int(sel_up)]),
    );
    check("timeout_running", py.eval("running"), bool_list(&running));
    // The exit at the timeout sends the exit event and runs the leave script.
    check(
        "timeout_events",
        py.eval("[e[0] for e in p.events]"),
        str_list(&world.printer.inner.borrow().events.clone()),
    );
}

#[test]
fn screen_update_draws_rows() {
    let Some(mut py) = oracle() else { return };
    let draws_expr = "['%d %d %s %s' % (d[0], d[1], d[2], d[3]) for d in disp.draws]";
    // A selected long name followed by blank rows: Python reuses the stale
    // `selected`/`current` of the last item for the blank rows, which keeps
    // resetting the scroller.
    let long_spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main long",
            &[
                ("type", "command"),
                ("name", "A very long item name here"),
                ("gcode", "G1"),
            ],
        ),
    ];
    // More items than rows: the viewport follows the selection.
    let mut tall_spec = standard_spec();
    for i in 0..4 {
        tall_spec.push(item(
            &format!("menu __main extra{i}"),
            &[("type", "command"), ("name", &format!("Extra {i}"))],
        ));
    }
    // A folder suffix and an input being edited.
    let folder_spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item("menu __main folder", &[("type", "list"), ("name", "Folder")]),
        item(
            "menu __main input",
            &[
                ("type", "input"),
                ("name", "Input"),
                ("input", "1"),
                ("input_min", "0"),
                ("input_max", "5"),
            ],
        ),
    ];
    for (label, spec, downs) in [
        ("standard", standard_spec(), 2usize),
        ("long", long_spec, 1),
        ("tall", tall_spec, 5),
        ("folder", folder_spec, 2),
    ] {
        py_world(&mut py, &opts(&[]), &spec, &None);
        let mut python = String::from("mgr.begin(1.0); r = [mgr.screen_update_event(2.0)]; ");
        for step in 0..downs {
            python.push_str(&format!(
                "mgr.down(False); r.append(mgr.screen_update_event({}.0)); ",
                3 + step
            ));
        }
        python.push_str(&format!(
            "r.append(mgr.screen_update_event({}.0)); r.append(mgr.screen_update_event({}.0))",
            3 + downs,
            4 + downs
        ));
        py.exec(&python).expect("py draw sequence");
        let world = rust_world(&opts(&[]), &spec, &None);
        world.manager.begin(1.0).expect("begin");
        let mut returned = vec![world.manager.screen_update_event(2.0).expect("update")];
        for step in 0..downs {
            world.manager.down(false).expect("down");
            returned.push(
                world
                    .manager
                    .screen_update_event(3.0 + step as f64)
                    .expect("update"),
            );
        }
        returned.push(
            world
                .manager
                .screen_update_event(3.0 + downs as f64)
                .expect("update"),
        );
        returned.push(
            world
                .manager
                .screen_update_event(4.0 + downs as f64)
                .expect("update"),
        );
        check(&format!("{label}_returns"), py.eval("r"), bool_list(&returned));
        check(
            &format!("{label}_draws"),
            py.eval(draws_expr),
            str_list(&world.display.draws.borrow().clone()),
        );
    }
    // An editing input shows `*` instead of the cursor.
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    py.exec(
        "mgr.begin(1.0); mgr.down(False); mgr.down(False); mgr.press('click'); \
         mgr.screen_update_event(2.0)",
    )
    .expect("py editing");
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    world.manager.begin(1.0).expect("begin");
    world.manager.down(false).expect("down");
    world.manager.down(false).expect("down");
    world.manager.press("click").expect("press");
    world.manager.screen_update_event(2.0).expect("update");
    check(
        "editing_draws",
        py.eval(draws_expr),
        str_list(&world.display.draws.borrow().clone()),
    );
}

#[test]
fn vsd_list_files() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item("menu __main sd", &[("type", "vsdlist"), ("name", "SD")]),
    ];
    let files: Files = Some(vec![
        ("a.gcode".to_string(), 10),
        ("b c.gcode".to_string(), 20),
        ("it's.gcode".to_string(), 30),
    ]);
    py_world(&mut py, &opts(&[]), &spec, &files);
    py.exec("v = mgr.menuitems['__main sd']; v.populate(); v.update_items()")
        .expect("py populate");
    let world = rust_world(&opts(&[]), &spec, &files);
    let vsd = find(&world, "__main sd");
    vsd.populate().expect("populate");
    vsd.update_items().expect("update_items");
    let items = vsd.iter_items();
    let names: Vec<String> = items
        .iter()
        .map(|it| it.render_name(false).expect("name"))
        .collect();
    check(
        "vsd_names",
        py.eval("[it.render_name() for it in v]"),
        str_list(&names),
    );
    let render_only = RunScriptOpts {
        render_only: true,
        ..Default::default()
    };
    let scripts: Vec<Value> = items[1..]
        .iter()
        .map(|it| opt_str(it.run_script("gcode", &render_only).expect("script")))
        .collect();
    check(
        "vsd_scripts",
        py.eval("[it.run_script('gcode', render_only=True) for it in list(v)[1:]]"),
        Value::List(scripts),
    );
    // Pressing a file runs `M23 /<name>`.
    py.exec(
        "mgr.begin(1.0); mgr.down(False); mgr.press('click'); mgr.down(False); mgr.press('click'); \
         mgr.dispatch_gcode(0.0)",
    )
    .expect("py press");
    world.manager.begin(1.0).expect("begin");
    world.manager.down(false).expect("down");
    world.manager.press("click").expect("press");
    world.manager.down(false).expect("down");
    world.manager.press("click").expect("press");
    world.manager.dispatch_gcode();
    check(
        "vsd_press",
        py.eval("(p.gcode.scripts, mgr.stack_size())"),
        Value::Tuple(vec![
            str_list(&world.gcode.scripts.borrow().clone()),
            Value::int(world.manager.stack_size() as i64),
        ]),
    );
    // Without the `virtual_sdcard` object only the back item remains.
    py_world(&mut py, &opts(&[]), &spec, &None);
    py.exec("v = mgr.menuitems['__main sd']; v.populate(); v.update_items()")
        .expect("py populate");
    let world = rust_world(&opts(&[]), &spec, &None);
    let vsd = find(&world, "__main sd");
    vsd.populate().expect("populate");
    vsd.update_items().expect("update_items");
    check(
        "vsd_no_sdcard",
        py.eval("len(v)"),
        Value::int(vsd.len() as i64),
    );
}

#[test]
fn dispatch_gcode_survives_errors() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main bad",
            &[("type", "command"), ("name", "Bad"), ("gcode", "BAD G1")],
        ),
        item(
            "menu __main good",
            &[("type", "command"), ("name", "Good"), ("gcode", "G28")],
        ),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    py.exec(
        "mgr.begin(1.0); mgr.down(False); mgr.press('click'); mgr.down(False); \
         mgr.press('click'); mgr.dispatch_gcode(0.0); queue = list(mgr.gcode_queue); \
         mgr.queue_gcode(''); mgr.queue_gcode(None); queue_after = list(mgr.gcode_queue)",
    )
    .expect("py dispatch");
    let world = rust_world(&opts(&[]), &spec, &None);
    world.manager.begin(1.0).expect("begin");
    world.manager.down(false).expect("down");
    world.manager.press("click").expect("press");
    world.manager.down(false).expect("down");
    world.manager.press("click").expect("press");
    world.manager.dispatch_gcode();
    let queue = world.manager.gcode_queue();
    world.manager.queue_gcode("");
    let queue_after = world.manager.gcode_queue();
    check(
        "dispatched",
        py.eval("(p.gcode.scripts, queue, queue_after)"),
        Value::Tuple(vec![
            str_list(&world.gcode.scripts.borrow().clone()),
            str_list(&queue),
            str_list(&queue_after),
        ]),
    );
}

#[test]
fn push_container_and_stack() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item("menu __main sub", &[("type", "list"), ("name", "Sub"), ("enter", "ENTER"), ("leave", "LEAVE")]),
        item("menu __main sub deep", &[("type", "command"), ("name", "Deep"), ("gcode", "DEEP")]),
        item("menu __main leaf", &[("type", "command"), ("name", "Leaf")]),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    // Not running: nothing is pushed.
    let sub = find(&world, "__main sub");
    check(
        "push_idle",
        py.eval("mgr.push_container(mgr.menuitems['__main sub'])"),
        Value::Bool(world.manager.push_container(&sub).expect("push")),
    );
    py.exec("mgr.begin(1.0)").expect("py begin");
    world.manager.begin(1.0).expect("begin");
    check(
        "push_running",
        py.eval("mgr.push_container(mgr.menuitems['__main sub'])"),
        Value::Bool(world.manager.push_container(&sub).expect("push")),
    );
    check(
        "push_same",
        py.eval("mgr.push_container(mgr.menuitems['__main sub'])"),
        Value::Bool(world.manager.push_container(&sub).expect("push")),
    );
    check_outcome(
        "push_non_container",
        py.eval("mgr.stack_push(mgr.menuitems['__main leaf'])"),
        world
            .manager
            .stack_push(&find(&world, "__main leaf"))
            .map(|_| Value::Null),
    );
    // Back out through the `..` item runs leave and enter scripts.
    py.exec(
        "mgr.press('click'); mgr.dispatch_gcode(0.0); mgr.back(); mgr.dispatch_gcode(0.0); \
         depth = mgr.stack_size()",
    )
    .expect("py back");
    world.manager.press("click").expect("press");
    world.manager.dispatch_gcode();
    world.manager.back(false, true).expect("back");
    world.manager.dispatch_gcode();
    check(
        "stack_scripts",
        py.eval("(p.gcode.scripts, depth)"),
        Value::Tuple(vec![
            str_list(&world.gcode.scripts.borrow().clone()),
            Value::int(world.manager.stack_size() as i64),
        ]),
    );
    check(
        "stack_events",
        py.eval("[e[0] for e in p.events]"),
        str_list(&world.printer.inner.borrow().events.clone()),
    );
    // Popping an empty stack returns nothing.
    py.exec("mgr.exit(True); mgr.stack_pop(); mgr.stack_pop(); popped = mgr.stack_pop()")
        .expect("py pops");
    world.manager.exit(true).expect("exit");
    world.manager.stack_pop(true).expect("pop");
    world.manager.stack_pop(true).expect("pop");
    let popped = world.manager.stack_pop(true).expect("pop");
    check("pop_empty", py.eval("popped is None"), Value::Bool(popped.is_none()));
}

#[test]
fn template_context_and_scripts() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main cmd",
            &[
                ("type", "command"),
                ("name", "N {menu.ns}|{menu.eventtime}|{printer.menu.rows}"),
                ("gcode", "G {menu.event} {menu.ns}\nM {printer.menu.cols}"),
                ("long_click", "LONG {menu.event}"),
            ],
        ),
        item(
            "menu __main inp",
            &[
                ("type", "input"),
                ("name", "In {'%.1f' % menu.input}"),
                ("input", "{printer.menu.rows}"),
                ("input_min", "{printer.menu.cols - 20}"),
                ("input_max", "{printer.menu.rows * 2}"),
                ("input_step", "0.5"),
                ("gcode", "SET {menu.event} {'%.1f' % menu.input} {menu.ns}"),
            ],
        ),
        item(
            "menu __main hide",
            &[
                ("type", "command"),
                ("name", "Hidden once running"),
                ("enable", "{not printer.menu.running}"),
            ],
        ),
        item(
            "menu __main show",
            &[
                ("type", "command"),
                ("name", "Shown once running"),
                ("enable", "{printer.menu.running}"),
            ],
        ),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    let names = ["cmd", "inp", "hide", "show"];
    // Before the menu runs: names, enable flags and context keys.
    py.exec(
        "its = [mgr.menuitems['__main ' + n] for n in ('cmd', 'inp', 'hide', 'show')]; \
         before = [[i.render_name(), i.is_enabled(), sorted(i.get_context()['menu'])] for i in its]",
    )
    .expect("py before");
    let items: Vec<Arc<MenuElement>> = names
        .iter()
        .map(|n| find(&world, &format!("__main {n}")))
        .collect();
    let snapshot = |items: &[Arc<MenuElement>]| -> Value {
        Value::List(
            items
                .iter()
                .map(|i| {
                    Value::List(vec![
                        Value::str(&i.render_name(false).expect("name")),
                        Value::Bool(i.is_enabled().expect("enabled")),
                        menu_keys(&i.get_context(None).expect("context")),
                    ])
                })
                .collect(),
        )
    };
    check("before_begin", py.eval("before"), snapshot(&items));
    // Run the scripts with different events and compare the rendered text.
    py.exec(
        "c, n, h, s = its; \
         out = [c.run_script('gcode', event='click', render_only=True), \
                c.run_script('long_click', event='long_click', render_only=True), \
                c.run_script('gcode', render_only=True), \
                n.run_script('gcode', event='change', render_only=True), \
                c.get_context()['menu']['event'], c.get_context()['menu']['ns'], \
                n.get_context()['menu']['ns'], n.get_context()['menu']['input'], \
                sorted(c.get_context({'extra': 1, 'menu': {'x': 1}})['menu'])]",
    )
    .expect("py scripts");
    let c = &items[0];
    let n = &items[1];
    let render_only = |event: Option<&str>| RunScriptOpts {
        event: event.map(str::to_string),
        render_only: true,
        ..Default::default()
    };
    let overlay: TemplateContext = {
        let mut map: TemplateContext = IndexMap::new();
        map.insert("extra".to_string(), TplValue::py(PyValue::Int(PyInt::from_i64(1))));
        let mut menu = IndexMap::new();
        menu.insert(
            PyValue::Str("x".to_string()),
            PyValue::Int(PyInt::from_i64(1)),
        );
        map.insert("menu".to_string(), TplValue::py(PyValue::Dict(menu)));
        map
    };
    let out = vec![
        opt_str(c.run_script("gcode", &render_only(Some("click"))).expect("click")),
        opt_str(
            c.run_script("long_click", &render_only(Some("long_click")))
                .expect("long_click"),
        ),
        opt_str(c.run_script("gcode", &render_only(None)).expect("gcode")),
        opt_str(n.run_script("gcode", &render_only(Some("change"))).expect("change")),
        py_to_value(&menu_entry(&c.get_context(None).expect("context"), "event").expect("event")),
        py_to_value(&menu_entry(&c.get_context(None).expect("context"), "ns").expect("ns")),
        py_to_value(&menu_entry(&n.get_context(None).expect("context"), "ns").expect("ns")),
        py_to_value(&menu_entry(&n.get_context(None).expect("context"), "input").expect("input")),
        menu_keys(&c.get_context(Some(&overlay)).expect("overlay context")),
    ];
    check("script_outputs", py.eval("out"), Value::List(out));
    // Begin the menu: the enable templates flip with `printer.menu.running`.
    py.exec(
        "mgr.begin(1.0); mgr.update_context(2.5); \
         after = [[i.render_name(), i.is_enabled()] for i in its]; \
         shown = [i.render_name() for i in mgr.stack_peek()]; \
         ev = [mgr.get_context()['menu']['eventtime'], sorted(mgr.get_context()['menu'])]",
    )
    .expect("py after");
    world.manager.begin(1.0).expect("begin");
    world.manager.update_context(2.5).expect("update_context");
    let after = Value::List(
        items
            .iter()
            .map(|i| {
                Value::List(vec![
                    Value::str(&i.render_name(false).expect("name")),
                    Value::Bool(i.is_enabled().expect("enabled")),
                ])
            })
            .collect(),
    );
    check("after_begin", py.eval("after"), after);
    let top = world.manager.stack_peek(0).expect("top");
    check(
        "shown_items",
        py.eval("shown"),
        str_list(
            &top.iter_items()
                .iter()
                .map(|i| i.render_name(false).expect("name"))
                .collect::<Vec<_>>(),
        ),
    );
    let context = world.manager.get_context(None);
    check(
        "manager_context",
        py.eval("ev"),
        Value::List(vec![
            py_to_value(&menu_entry(&context, "eventtime").expect("eventtime")),
            menu_keys(&context),
        ]),
    );
    // Editing an input renders the live value, and the click script sees it.
    py.exec(
        "mgr.down(False); mgr.down(False); mgr.press('click'); mgr.down(False); mgr.down(False); \
         edit = [n.render_name(), n.is_editing(), mgr.menuitems['__main inp'].get_context()['menu']['input']]; \
         mgr.press('click'); mgr.dispatch_gcode(0.0)",
    )
    .expect("py edit");
    world.manager.down(false).expect("down");
    world.manager.down(false).expect("down");
    world.manager.press("click").expect("press");
    world.manager.down(false).expect("down");
    world.manager.down(false).expect("down");
    let edit = Value::List(vec![
        Value::str(&n.render_name(false).expect("name")),
        Value::Bool(n.is_editing()),
        Value::float(input_of(&n.get_context(None).expect("context"))),
    ]);
    world.manager.press("click").expect("press");
    world.manager.dispatch_gcode();
    check("edit_state", py.eval("edit"), edit);
    check(
        "edit_scripts",
        py.eval("p.gcode.scripts"),
        str_list(&world.gcode.scripts.borrow().clone()),
    );
}

#[test]
fn template_errors_reach_the_caller() {
    let Some(mut py) = oracle() else { return };
    let spec: Spec = vec![
        item("menu __main", &[("type", "list"), ("name", "Main")]),
        item(
            "menu __main broken",
            &[
                ("type", "command"),
                ("name", "Broken"),
                ("gcode", "{action_raise_error('nope')}"),
            ],
        ),
    ];
    py_world(&mut py, &opts(&[]), &spec, &None);
    let world = rust_world(&opts(&[]), &spec, &None);
    let broken = find(&world, "__main broken");
    check_outcome(
        "broken_script",
        py.eval("mgr.menuitems['__main broken'].run_script('gcode', render_only=True)"),
        broken
            .run_script(
                "gcode",
                &RunScriptOpts {
                    render_only: true,
                    ..Default::default()
                },
            )
            .map(opt_str),
    );
    // Without `update_context` the manager context has no `menu` entry.
    py.exec("p2, mgr2, disp2 = make_world([('menu __main', {'type': 'list', 'name': 'M'})], None, {})")
        .expect("py second world");
    let fresh_spec: Spec = vec![item("menu __main", &[("type", "list"), ("name", "M")])];
    let fresh = rust_world_result(
        &opts(&[]),
        &fresh_spec,
        &None,
        false,
    )
    .expect("fresh world");
    check_outcome(
        "no_menu_context",
        py.eval("mgr2.menuitems['__main'].get_context()"),
        fresh
            .manager
            .lookup_menuitem(Some("__main"))
            .expect("lookup")
            .expect("item")
            .get_context(None)
            .map(|_| Value::Null),
    );
}

#[test]
fn menu_keys_are_wired_to_key_event() {
    let Some(mut py) = oracle() else { return };
    let display_opts = opts(&[
        ("click_pin", "^PA1"),
        ("back_pin", "PA4"),
        ("up_pin", "PA2"),
        ("down_pin", "PA3"),
        ("encoder_pins", "^PB1, ^PB2"),
        ("encoder_steps_per_detent", "2"),
    ]);
    py_world(&mut py, &display_opts, &standard_spec(), &None);
    let world = rust_world(&display_opts, &standard_spec(), &None);
    let buttons = world
        .printer
        .buttons
        .borrow()
        .clone()
        .expect("buttons were loaded");
    check(
        "registrations",
        py.eval("p.objects['buttons'].log"),
        str_list(&buttons.log.borrow().clone()),
    );
    // Press and release the click button, then use the other buttons and
    // the encoder; the menu ends in the same state with the same redraws.
    py.exec(
        "b = p.objects['buttons'].cbs; \
         b['^PA1'](2.0, 1); b['^PA1'](2.1, 0); \
         b['PA3'](3.0); b['PA3'](3.1); b['PA2'](3.2); \
         b['ccw'](4.0); b['ccw'](4.01); b['cw'](4.5); b['cw'](4.51); \
         b['PA4'](5.0); \
         wired = (disp.redraws, mgr.is_running(), mgr.stack_size(), \
                  [mgr.stack_peek().selected] if mgr.stack_peek() else [])",
    )
    .expect("py buttons");
    let click = buttons.buttons.borrow().get("^PA1").cloned().expect("click callback");
    click(2.0, 1);
    click(2.1, 0);
    let push = |name: &str, time: f64| {
        let callback = buttons.pushes.borrow().get(name).cloned().expect("push callback");
        callback(time);
    };
    push("PA3", 3.0);
    push("PA3", 3.1);
    push("PA2", 3.2);
    push("ccw", 4.0);
    push("ccw", 4.01);
    push("cw", 4.5);
    push("cw", 4.51);
    push("PA4", 5.0);
    let selected: Vec<Value> = world
        .manager
        .stack_peek(0)
        .map(|c| vec![opt_int(c.selected())])
        .unwrap_or_default();
    check(
        "wired_effects",
        py.eval("wired"),
        Value::Tuple(vec![
            Value::int(*world.display.redraws.borrow()),
            Value::Bool(world.manager.is_running()),
            Value::int(world.manager.stack_size() as i64),
            Value::List(selected),
        ]),
    );
    // No pins configured: nothing is registered and the options default.
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    let buttons = world
        .printer
        .buttons
        .borrow()
        .clone()
        .expect("buttons were loaded");
    check(
        "no_registrations",
        py.eval("p.objects['buttons'].log"),
        str_list(&buttons.log.borrow().clone()),
    );
}

#[test]
fn populate_and_script_callbacks() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &opts(&[]), &standard_spec(), &None);
    let world = rust_world(&opts(&[]), &standard_spec(), &None);
    py.exec(r#"
population_calls = []
script_calls = []
def script_callback(el, context):
    script_calls.append([context['menu']['ns'], context['menu']['event']])
    return 'CALL'
def populate_callback(el):
    population_calls.append(el.get_ns())
    child = el.manager.menuitem_from(
        'command', ns='dynamic child', name='Dynamic', gcode=script_callback)
    el.insert_item(child)
dynamic = mgr.menuitem_from(
    'list', ns='dynamic', name='Dynamic list', populate=populate_callback)
dynamic.populate()
dynamic.update_items()
first_names = [el.render_name() for el in dynamic]
dynamic.populate()
dynamic.update_items()
second_names = [el.render_name() for el in dynamic]
callback_result = list(dynamic)[1].run_script('gcode', event='custom')
mgr.dispatch_gcode(0.0)
"#).expect("Python callbacks");
    let population_calls = Arc::new(GCell::new(Vec::<String>::new()));
    let script_calls = Arc::new(GCell::new(Vec::<Value>::new()));
    let populate_log = population_calls.clone();
    let script_log = script_calls.clone();
    let manager = world.manager.clone();
    let dynamic = world.manager.menuitem_from(
        "list",
        &MenuItemKwargs {
            ns: Some("dynamic".to_string()),
            name: Some("Dynamic list".to_string()),
            populate: Some(Arc::new(move |container| {
                populate_log.borrow_mut().push(container.get_ns("."));
                let script_log = script_log.clone();
                let child = manager.menuitem_from(
                    "command",
                    &MenuItemKwargs {
                        ns: Some("dynamic child".to_string()),
                        name: Some("Dynamic".to_string()),
                        gcode: Some(ScriptValue::Callback(Arc::new(move |_element, context| {
                            script_log.borrow_mut().push(Value::List(vec![
                                py_to_value(&menu_entry(context, "ns").expect("namespace")),
                                py_to_value(&menu_entry(context, "event").expect("event")),
                            ]));
                            Ok(Some("CALL".to_string()))
                        }))),
                        ..Default::default()
                    },
                )?;
                container.insert_item(&LookupRef::Element(child), None)
            })),
            ..Default::default()
        },
    ).expect("dynamic list");
    let names = |container: &Arc<MenuElement>| {
        str_list(&container.iter_items().iter()
            .map(|el| el.render_name(false).expect("name")).collect::<Vec<_>>())
    };
    dynamic.populate().expect("populate");
    dynamic.update_items().expect("items");
    check("first_population", py.eval("first_names"), names(&dynamic));
    dynamic.populate().expect("repopulate");
    dynamic.update_items().expect("items");
    check("second_population", py.eval("second_names"), names(&dynamic));
    check(
        "callback_return",
        py.eval("callback_result"),
        opt_str(dynamic.iter_items()[1].run_script(
            "gcode",
            &RunScriptOpts { event: Some("custom".to_string()), ..Default::default() },
        ).expect("script")),
    );
    world.manager.dispatch_gcode();
    check("population_calls", py.eval("population_calls"), str_list(&population_calls.borrow()));
    check("script_calls", py.eval("script_calls"), Value::List(script_calls.borrow().clone()));
    check("callback_gcode", py.eval("p.gcode.scripts"), str_list(&world.gcode.scripts.borrow()));
    check("populate_events", py.eval("[e[0] for e in p.events]"), str_list(&world.printer.inner.borrow().events));

    py.exec(r#"
def failing_populate(el):
    el.insert_item(el.manager.menuitem_from('command', name='Inserted'))
    raise gcode_mod.CommandError('populate failed')
failing = mgr.menuitem_from('list', ns='failing', name='Failing', populate=failing_populate)
"#).expect("failing callback");
    let manager = world.manager.clone();
    let failing = world.manager.menuitem_from(
        "list",
        &MenuItemKwargs {
            ns: Some("failing".to_string()),
            name: Some("Failing".to_string()),
            populate: Some(Arc::new(move |container| {
                let child = manager.menuitem_from(
                    "command",
                    &MenuItemKwargs { name: Some("Inserted".to_string()), ..Default::default() },
                )?;
                container.insert_item(&LookupRef::Element(child), None)?;
                Err(KlippyError::command("populate failed"))
            })),
            ..Default::default()
        },
    ).expect("failing list");
    check_outcome(
        "populate_exception",
        py.eval("failing.populate()"),
        failing.populate().map(|_| Value::Null),
    );
    py.exec("failing.update_items()").expect("partial population");
    failing.update_items().expect("partial population");
    check("partial_population", py.eval("[el.render_name() for el in failing]"), names(&failing));
    check("aborted_populate_events", py.eval("[e[0] for e in p.events]"), str_list(&world.printer.inner.borrow().events));
}
