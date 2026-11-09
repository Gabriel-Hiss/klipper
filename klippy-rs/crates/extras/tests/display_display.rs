//! Parity tests for `extras::display::display` (port of
//! `klippy/extras/display/display.py`).
//!
//! Every expected value comes from the live Python reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`). Both sides are built from
//! the same config text. The Python side parses it with the real
//! `configfile.ConfigFileReader` and the real `gcode_macro.PrinterGCodeMacro`
//! (Jinja2); the Rust side parses it with the ported `ConfigFileReader` and
//! renders through the ported `extras::gcode_macro`. The shipped
//! `klippy/extras/display/display.cfg` is read by both sides from disk, so the
//! default templates, data groups and glyphs are compared as well.
//!
//! LCD chips record calls on both sides. Most scenarios use a secondary
//! display, where the reference does not construct a menu. Primary-display
//! scenarios construct the real menu and load its bundled configuration.
//! Comparisons cover chip calls, return values, errors, redraw state,
//! registered commands, handlers, objects and log messages.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit.

use std::cell::RefCell;
use std::sync::{Arc, Once};

use extras::display::display::{
    lcd_chips, lookup_display_templates, module_factory, ChipCtor, DisplayGroup, DisplayTemplateApi,
    GlyphTable, LcdChipApi, PrinterDisplayTemplateApi, PrinterLCD, PrinterLCDApi, REDRAW_MIN_TIME,
    REDRAW_TIME, SET_DISPLAY_GROUP_HELP,
};
use extras::gcode_macro::PrinterGCodeMacro;
use extras_macros_templates::gcode_macro::Value as TplValue;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GcGetFloatOpts,
    GetIntOpts as GcGetIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{log, EventArg, ObjectRef, PrinterObject};
use kos::sync::Mutex;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging as _logging
import contextlib as _ctx
import os as _os
import extras.display.display as display_mod
from extras import gcode_macro as gcode_macro_mod
from gcode import GCodeCommand as _GC, CommandError as _CE
import configfile as _cf
from configfile import ConfigWrapper as _CW, error as _config_error
from collections import OrderedDict

ORIG_CHIPS = list(display_mod.LCD_chips)
GLYPH_RET = {'extruder': 2, 'bed': 2, 'bed_heat1': 2, 'bed_heat2': 2,
             'fan1': 2, 'fan2': 2, 'feedrate': 2, 'clock': 2}

class FakeTimer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
        self.updates = []
    def monotonic(self):
        return 100.0
    def register_timer(self, cb, waketime=NEVER):
        timer = FakeTimer(cb, waketime)
        self.timers.append(timer)
        return timer
    def update_timer(self, timer, waketime):
        timer.waketime = waketime
        self.updates.append(waketime)
    @_ctx.contextmanager
    def assert_no_pause(self):
        yield

class FakePrinter:
    config_error = _config_error
    command_error = _CE
    _sentinel = object()
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = OrderedDict()
        self.handlers = OrderedDict()
        self.shutdowns = []
    def get_printer(self):
        return self
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        if name in self.objects:
            raise _config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is self._sentinel:
            raise _config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        return list(self.objects.items())
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if section in ('buttons', 'query_adc'):
            import importlib
            obj = importlib.import_module('extras.' + section).load_config(config)
            self.add_object(section, obj)
            return obj
        if default is self._sentinel:
            raise _config_error("Unknown config object '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def invoke_async_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)

class FakeStatus:
    def __init__(self, st):
        self.st = st
    def get_status(self, eventtime):
        return dict(self.st)

class FakeGCmd(_GC):
    def __init__(self, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = params
        self._need_ack = False
        self.respond_info = lambda msg, log=True: None
        self.respond_raw = lambda msg: None

class FakeGCode:
    error = _CE
    def __init__(self):
        self.mux = []
        self.handlers = {}
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))
        self.handlers[value] = func
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        pass
    def dispatch(self, value, commandline, params):
        gcmd = FakeGCmd('SET_DISPLAY_GROUP', commandline, params)
        return self.handlers[value](gcmd)
    def run_script(self, script):
        pass
    def run_script_from_command(self, script):
        pass
    def respond_info(self, msg, log=True):
        pass

class FakeConfigfile:
    def __init__(self, printer, fail=False):
        self.printer = printer
        self.fail = fail
        self.read_calls = []
    def read_config(self, filename):
        self.read_calls.append(filename)
        if self.fail:
            raise RuntimeError('cannot read')
        rdr = _cf.ConfigFileReader()
        data = rdr.read_config_file(filename)
        fileconfig = rdr.build_fileconfig(data, filename)
        return _CW(self.printer, fileconfig, {}, 'printer')

class FakeChip:
    cols = 16
    def __init__(self, config):
        self.log = []
        self.glyph_ret = {}
        self.fail_text = None
        self.redraw = False
        self.fail_init = False
        self.fail_flush = False
    def set_glyphs(self, glyphs):
        self.log.append(('set_glyphs', norm_glyphs(glyphs)))
    def get_dimensions(self):
        return (self.cols, 4)
    def init(self):
        if self.fail_init:
            raise RuntimeError('init boom')
        self.log.append(('init',))
    def clear(self):
        self.log.append(('clear',))
    def flush(self):
        if self.fail_flush:
            raise RuntimeError('flush boom')
        self.log.append(('flush',))
    def write_text(self, x, y, data):
        if self.redraw:
            self.display.request_redraw()
        if self.fail_text is not None and data == self.fail_text:
            raise RuntimeError('chip boom')
        self.log.append(('write_text', x, y, bytes(data)))
    def write_glyph(self, x, y, name):
        self.log.append(('write_glyph', x, y, name))
        return self.glyph_ret.get(name, 1)
    def write_graphics(self, x, y, data):
        self.log.append(('write_graphics', x, y, list(data)))

class FakeChip20(FakeChip):
    cols = 20

display_mod.LCD_chips['fake16'] = FakeChip
display_mod.LCD_chips['fake20'] = FakeChip20


log_records = []
class _Cap(_logging.Handler):
    def emit(self, record):
        log_records.append((record.levelname, record.getMessage()))
_root = _logging.getLogger()
_root.addHandler(_Cap())
_root.setLevel(0)

def norm_glyphs(glyphs):
    return [[name, [[k, [list(v[0]), list(v[1])] if k == 'icon16x16'
                         else [v[0], list(v[1])]] for k, v in d.items()]]
            for name, d in glyphs.items()]

def make_cfg(printer, name, text):
    rdr = _cf.ConfigFileReader()
    fileconfig = rdr.build_fileconfig(text, 'test.cfg')
    return _CW(printer, fileconfig, {}, name)

def statuses(variant):
    st = OrderedDict([
        ('toolhead', {'estimated_print_time': 12.5, 'extruder': 'extruder'}),
        ('extruder', {'temperature': 200.4, 'target': 210.0}),
        ('heater_bed', {'temperature': 60.0, 'target': 0.0}),
        ('fan', {'speed': 0.5}),
        ('gcode_move', {'speed_factor': 1.0}),
        ('display_status', {'progress': 0.25, 'message': 'Printing now'}),
        ('idle_timeout', {'printing_time': 3725.5}),
        ('virtual_sdcard', {'progress': 0.25}),
    ])
    if variant == 1:
        st['extruder'] = {'temperature': 25.0, 'target': 0.0}
        st['heater_bed'] = {'temperature': 20.0, 'target': 60.0}
        st['fan'] = {'speed': 0.0}
        st['gcode_move'] = {'speed_factor': 1.5}
        st['display_status'] = {'progress': 1.0, 'message': None}
        st['idle_timeout'] = {'printing_time': 0.0}
        st['virtual_sdcard'] = {'progress': 0.0}
    return st

class World:
    def __init__(self, text, variant=0, fail_cfg=False):
        self.text = text
        self.printer = FakePrinter()
        self.gcode = FakeGCode()
        p = self.printer
        p.objects['gcode'] = self.gcode
        p.objects['gcode_macro'] = gcode_macro_mod.PrinterGCodeMacro(
            make_cfg(p, 'gcode_macro', ''))
        for name, st in statuses(variant).items():
            p.objects[name] = FakeStatus(st)
        self.configfile = FakeConfigfile(p, fail_cfg)
        p.objects['configfile'] = self.configfile
    def cfg(self, section):
        return make_cfg(self.printer, section, self.text)

def make_lcd(w, section='display my_display'):
    del log_records[:]
    lcd = display_mod.PrinterLCD(w.cfg(section))
    lcd.lcd_chip.glyph_ret.update(GLYPH_RET)
    lcd.lcd_chip.display = lcd
    return lcd

def snap(lcd):
    return (list(lcd.lcd_chip.log), lcd.redraw_request_pending, lcd.redraw_time,
            lcd.menu is not None, list(log_records))

def outcome(fn, lcd):
    try:
        r = fn()
    except Exception as e:
        return ('err', type(e).__name__, str(e), snap(lcd))
    return ('ok', r, snap(lcd))

def summary(dt, w):
    return (
        [(n, list(t.get_params().items()))
         for n, t in dt.get_display_templates().items()],
        [(g, [(r, c) for r, c, _ in grp.data_items])
         for g, grp in dt.get_display_data_groups().items()],
        norm_glyphs(dt.get_display_glyphs()),
        list(w.printer.objects),
        display_mod.lookup_display_templates(w.cfg('display')) is dt,
        len(w.configfile.read_calls),
        _os.path.basename(w.configfile.read_calls[0]),
        w.configfile.read_calls[0].replace('\\', '/').endswith(
            'extras/display/display.cfg'),
    )
"#;

// ---------------------------------------------------------------------------
// Config texts.
// ---------------------------------------------------------------------------

const DISPLAY_SECTION: &str = "display my_display";
const DISPLAY16: &str = "[display my_display]\nlcd_type: fake16\n";
const DISPLAY20: &str = "[display my_display]\nlcd_type: fake20\n";
const PRIMARY16: &str = "[display]\nlcd_type: fake16\ndisplay_group: normal\nmenu_root: __display_test\n\
[menu __display_test]\ntype: list\nname: Test\n\
[display_data normal marker]\nposition: 0,0\ntext: NORMAL\n";

const G16A: [&str; 16] = [
    "................",
    "..************..",
    ".*............*.",
    ".*..**....**..*.",
    ".*............*.",
    ".*..********..*.",
    ".*............*.",
    "..************..",
    "0000111100001111",
    "................",
    "...*.*.*.*.*.*..",
    "................",
    "*..............*",
    "................",
    "..............**",
    "................",
];

const G16B: [&str; 16] = [
    "*...............",
    ".*..............",
    "..*.............",
    "...*............",
    "....*...........",
    ".....*..........",
    "......*.........",
    ".......*........",
    "........*.......",
    ".........*......",
    "..........*.....",
    "...........*....",
    "............*...",
    ".............*..",
    "..............*.",
    "...............*",
];

const G5: [&str; 8] = [
    "..*..", ".***.", "*...*", "*****", "*...*", "01010", ".....", "11111",
];

fn rows(rows: &[&str]) -> String {
    rows.iter().map(|r| format!("  {r}\n")).collect()
}

fn user_cfg() -> String {
    format!(
        "[display my_display]\n\
         lcd_type: fake16\n\
         display_group: mygroup\n\
         \n\
         [display_template _fan_speed]\n\
         text:\n  custom {{ printer.fan.speed }}\n\
         \n\
         [display_template mine]\n\
         param_a: 5\n\
         param_b: \"x\"\n\
         param_c: [1, 2.5, None, True]\n\
         param_d: {{'k': (1, 2)}}\n\
         param_e: -3\n\
         text: {{ param_a }}|{{ param_b }}|{{ x }}\n\
         \n\
         [display_data mygroup b]\n\
         position: 2, 3\n\
         text: bee {{ render(\"mine\", param_a=9, x=2) }}\n\
         \n\
         [display_data mygroup a]\n\
         position: 1,0\n\
         text: ayy\n\
         \n\
         [display_data mygroup empty]\n\
         position: +0, -0\n\
         text:\n\
         \n\
         [display_data mygroup c]\n\
         position:  1 , 0\n\
         text: cee\n\
         \n\
         [display_data other z]\n\
         position: 7, 8\n\
         text: zed\n\
         \n\
         [display_data _default_16x4 extruder]\n\
         position: 5, 5\n\
         text: override\n\
         \n\
         [display_glyph myicon]\n\
         data:\n{g16a}\
         hd44780_data:\n{g5}\
         hd44780_slot: 3\n\
         \n\
         [display_glyph bed]\n\
         data:\n{g16b}\
         \n\
         [display_glyph onlyslot]\n\
         hd44780_data:\n{g5}\
         hd44780_slot: 0\n",
        g16a = rows(&G16A),
        g5 = rows(&G5),
        g16b = rows(&G16B),
    )
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type ConfigData = IndexMap<String, IndexMap<String, String>>;
/// `register_mux_command` arguments: command, key, value and help.
type MuxEntry = (String, String, Option<String>, Option<String>);
/// A `DisplayTemplate.render` case: label, Python call, context, keyword arguments.
type RenderCase = (&'static str, &'static str, IndexMap<String, TplValue>, IndexMap<String, PyValue>);
/// A `SET_DISPLAY_GROUP` case: label, mux value, command line, parameters.
type DispatchCase = (&'static str, Option<&'static str>, &'static str, Vec<(&'static str, &'static str)>);

/// Parse config text with the ported `ConfigFileReader` into plain data.
fn parse_config(text: &str, filename: &str) -> ConfigData {
    let fileconfig = klippy_core::configfile::ConfigFileReader::build_fileconfig(text, filename)
        .expect("parse config text");
    let mut data = IndexMap::new();
    for section in fileconfig.sections() {
        let mut opts = IndexMap::new();
        for option in fileconfig.options(&section).expect("options") {
            let value = fileconfig
                .get(&section, &option)
                .expect("get option")
                .unwrap_or_default();
            opts.insert(option, value);
        }
        data.insert(section, opts);
    }
    data
}

fn pyint_i64(n: &PyInt) -> Option<i64> {
    n.0.to_string().parse::<i64>().ok()
}

struct FakeConfig {
    printer: Printer,
    section: String,
    data: Arc<ConfigData>,
}

impl FakeConfig {
    #[allow(clippy::new_ret_no_self)]
    fn new(printer: Printer, section: &str, data: Arc<ConfigData>) -> Config {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            data,
        })
    }
    fn raw(&self, option: &str) -> Option<String> {
        self.data
            .get(&self.section)
            .and_then(|opts| opts.get(&option.to_lowercase()))
            .cloned()
    }
    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
    }
    fn unsupported(method: &str) -> KlippyError {
        KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: format!("FakeConfig.{method} is not used by display"),
        }
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
        match self.raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let v = pycompat::text::int_from_string(&raw, 10)
            .ok()
            .and_then(|n| pyint_i64(&n))
            .ok_or_else(|| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.section
                ))
            })?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {min}",
                    self.section
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {max}",
                    self.section
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.raw(option) else {
            return opts.default.ok_or_else(|| self.missing(option));
        };
        let value = pycompat::text::float_from_string(&raw)
            .map_err(|_| KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'", self.section
            )))?;
        if opts.minval.is_some_and(|v| value < v)
            || opts.maxval.is_some_and(|v| value > v)
            || opts.above.is_some_and(|v| value <= v)
            || opts.below.is_some_and(|v| value >= v)
        {
            return Err(KlippyError::config(format!("Invalid {option}")));
        }
        Ok(Some(value))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let Some(raw) = self.raw(option) else {
            return opts.default.ok_or_else(|| self.missing(option));
        };
        match raw.to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(Some(true)),
            "0" | "no" | "false" | "off" => Ok(Some(false)),
            _ => Err(KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'", self.section
            ))),
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let key = match self.raw(option) {
            Some(raw) => match choices {
                PyValue::List(items) if matches!(items.first(), Some(PyValue::Int(_))) => {
                    PyValue::Int(pycompat::text::int_from_string(&raw, 10)
                        .map_err(|_| KlippyError::config(format!("Invalid {option}")))?)
                }
                _ => PyValue::Str(raw),
            },
            None => match opts.default {
                None => return Err(self.missing(option)),
                Some(value) => value.unwrap_or(PyValue::None),
            },
        };
        if let PyValue::List(items) = choices {
            return if items.contains(&key) {
                Ok(key)
            } else {
                Err(KlippyError::config(format!("Invalid {option}")))
            };
        }
        let c = key.str();
        let PyValue::Dict(choices) = choices else {
            return Err(Self::unsupported("getchoice"));
        };
        match choices.get(&PyValue::Str(c.clone())) {
            Some(v) => Ok(v.clone()),
            None => Err(KlippyError::config(format!(
                "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
                self.section
            ))),
        }
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(Self::unsupported("getlists"))
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(Self::unsupported("getlist"))
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(Self::unsupported("getintlist"))
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(Self::unsupported("getfloatlist"))
    }
    fn getsection(&self, section: &str) -> Config {
        FakeConfig::new(self.printer.clone(), section, self.data.clone())
    }
    fn has_section(&self, section: &str) -> bool {
        self.data.contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.data
            .keys()
            .filter(|s| s.starts_with(prefix))
            .map(|s| self.getsection(s))
            .collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> Vec<String> {
        self.data
            .get(&self.section)
            .map(|opts| {
                opts.keys()
                    .filter(|o| o.starts_with(prefix))
                    .cloned()
                    .collect()
            })
            .unwrap_or_default()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

struct FakeConfigfile {
    printer: Printer,
    fail: bool,
    read_calls: Mutex<Vec<String>>,
}

impl PrinterConfigApi for FakeConfigfile {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, filename: &str) -> KResult<Config> {
        self.read_calls.lock().push(filename.to_string());
        if self.fail {
            return Err(KlippyError::Internal {
                class_name: "RuntimeError".to_string(),
                message: "cannot read".to_string(),
            });
        }
        let mut reader = klippy_core::configfile::ConfigFileReader::new();
        let text = reader
            .read_config_file(filename)
            .map_err(|e| KlippyError::config(e.to_string()))?;
        let data = parse_config(&text, filename);
        Ok(FakeConfig::new(
            self.printer.clone(),
            "printer",
            Arc::new(data),
        ))
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::config("read_main_config is not used by display"))
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
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) {}
    fn remove_section(&self, _section: &str) {}
}

klippy_api::printer_object!(FakeConfigfile; dyn PrinterConfigApi);

struct FakeStatus {
    st: PyValue,
}

impl PrinterObject for FakeStatus {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.st.clone())
    }
}

// -- G-code ------------------------------------------------------------------

struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn new(command: &str, commandline: &str, params: &[(&str, &str)]) -> Arc<FakeGCmd> {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        })
    }
    fn missing(&self, name: &str) -> KlippyError {
        KlippyError::command(format!(
            "Error on '{}': missing {name}",
            self.commandline
        ))
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(raw) => Ok(PyValue::Str(raw.clone())),
        }
    }
    fn get_int(&self, name: &str, opts: GcGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(raw) => raw.parse::<i64>().map(Some).map_err(|_| {
                KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))
            }),
        }
    }
    fn get_float(&self, name: &str, opts: GcGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(raw) => pycompat::text::float_from_string(raw).map(Some).map_err(|_| {
                KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))
            }),
        }
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.commandline.replacen(&self.command, "", 1)
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeGCode {
    reactor: Reactor,
    mux: Mutex<Vec<MuxEntry>>,
    handlers: Mutex<Vec<(Option<String>, GCodeCommandHandler)>>,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            reactor,
            mux: Mutex::new(Vec::new()),
            handlers: Mutex::new(Vec::new()),
        })
    }
    /// `handlers[value](FakeGCmd(...))`.
    fn dispatch(&self, value: Option<&str>, commandline: &str, params: &[(&str, &str)]) -> KResult<()> {
        let gcmd: GCmd = FakeGCmd::new("SET_DISPLAY_GROUP", commandline, params);
        let mut handlers = self.handlers.lock();
        let handler = handlers
            .iter_mut()
            .rev()
            .find(|(v, _)| v.as_deref() == value)
            .expect("handler registered for the mux value");
        (handler.1)(&gcmd)
    }
    fn mux_value(&self) -> Value {
        Value::List(
            self.mux
                .lock()
                .iter()
                .map(|(cmd, key, value, desc)| {
                    Value::Tuple(vec![
                        Value::str(cmd),
                        Value::str(key),
                        opt_str(value.as_deref()),
                        opt_str(desc.as_deref()),
                    ])
                })
                .collect(),
        )
    }
}

impl GCodeApi for FakeGCode {
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
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.map(str::to_string),
            desc.map(str::to_string),
        ));
        self.handlers
            .lock()
            .push((value.map(str::to_string), func));
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
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        klippy_core::gcode_parse::is_traditional_gcode(cmd)
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

// -- Printer -----------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
    shutdowns: Mutex<Vec<String>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor,
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            shutdowns: Mutex::new(Vec::new()),
        })
    }
    fn set_object(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
    fn object_names(&self) -> Value {
        Value::List(self.objects.lock().keys().map(|k| Value::str(k)).collect())
    }
    /// `list(printer.handlers)`: event names in first-registration order.
    fn handler_names(&self) -> Value {
        let mut names: Vec<String> = Vec::new();
        for (event, _) in self.handlers.lock().iter() {
            if !names.contains(event) {
                names.push(event.clone());
            }
        }
        Value::List(names.iter().map(|n| Value::str(n)).collect())
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
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(
        &self,
        config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(section).cloned() {
            return Ok(Some(obj));
        }
        let factory = match section {
            "buttons" => Some(extras::buttons::module_factory()),
            "query_adc" => Some(extras::query_adc::module_factory()),
            _ => None,
        };
        if let Some(factory) = factory {
            let object = factory.load_config.expect("load_config")(config)?;
            if let Some(object) = &object {
                self.add_object(section, object.clone())?;
            }
            return Ok(object);
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{section}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut returns = Vec::new();
        let mut handlers = self.handlers.lock();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                returns.push(cb(params)?);
            }
        }
        Ok(returns)
    }
    fn request_exit(&self, _result: &str) {}
}

// -- Printer status objects ---------------------------------------------------

fn st(entries: &[(&str, PyValue)]) -> PyValue {
    PyValue::Dict(
        entries
            .iter()
            .map(|(k, v)| (PyValue::Str(k.to_string()), v.clone()))
            .collect(),
    )
}

fn f(x: f64) -> PyValue {
    PyValue::Float(x)
}

fn s(x: &str) -> PyValue {
    PyValue::Str(x.to_string())
}

/// `statuses(variant)` of the Python setup.
fn statuses(variant: u8) -> Vec<(&'static str, PyValue)> {
    let mut out: Vec<(&'static str, PyValue)> = vec![
        ("toolhead", st(&[("estimated_print_time", f(12.5)), ("extruder", s("extruder"))])),
        ("extruder", st(&[("temperature", f(200.4)), ("target", f(210.0))])),
        ("heater_bed", st(&[("temperature", f(60.0)), ("target", f(0.0))])),
        ("fan", st(&[("speed", f(0.5))])),
        ("gcode_move", st(&[("speed_factor", f(1.0))])),
        ("display_status", st(&[("progress", f(0.25)), ("message", s("Printing now"))])),
        ("idle_timeout", st(&[("printing_time", f(3725.5))])),
        ("virtual_sdcard", st(&[("progress", f(0.25))])),
    ];
    if variant == 1 {
        let replace = |out: &mut Vec<(&'static str, PyValue)>, name: &str, value: PyValue| {
            let slot = out.iter_mut().find(|(n, _)| *n == name).expect("status");
            slot.1 = value;
        };
        replace(&mut out, "extruder", st(&[("temperature", f(25.0)), ("target", f(0.0))]));
        replace(&mut out, "heater_bed", st(&[("temperature", f(20.0)), ("target", f(60.0))]));
        replace(&mut out, "fan", st(&[("speed", f(0.0))]));
        replace(&mut out, "gcode_move", st(&[("speed_factor", f(1.5))]));
        replace(&mut out, "display_status", st(&[("progress", f(1.0)), ("message", PyValue::None)]));
        replace(&mut out, "idle_timeout", st(&[("printing_time", f(0.0))]));
        replace(&mut out, "virtual_sdcard", st(&[("progress", f(0.0))]));
    }
    out
}

// -- LCD chip and menu --------------------------------------------------------

/// `GLYPH_RET` of the Python setup: columns each glyph draws (default 1).
const GLYPH_RET: [(&str, i64); 8] = [
    ("extruder", 2),
    ("bed", 2),
    ("bed_heat1", 2),
    ("bed_heat2", 2),
    ("fan1", 2),
    ("fan2", 2),
    ("feedrate", 2),
    ("clock", 2),
];

struct ChipState {
    log: Mutex<Vec<Value>>,
    glyph_ret: Mutex<IndexMap<String, i64>>,
    fail_text: Mutex<Option<Vec<u8>>>,
    redraw: Mutex<Option<std::sync::Weak<PrinterLCD>>>,
    fail_init: Mutex<bool>,
    fail_flush: Mutex<bool>,
}

impl ChipState {
    fn new() -> Arc<ChipState> {
        Arc::new(ChipState {
            log: Mutex::new(Vec::new()),
            glyph_ret: Mutex::new(IndexMap::new()),
            fail_text: Mutex::new(None),
            redraw: Mutex::new(None),
            fail_init: Mutex::new(false),
            fail_flush: Mutex::new(false),
        })
    }
}

struct FakeChip {
    cols: i64,
    st: Arc<ChipState>,
}

fn ints(values: &[i64]) -> Value {
    Value::List(values.iter().map(|v| Value::int(*v)).collect())
}

/// `norm_glyphs(glyphs)` of the Python setup.
fn glyph_value(glyphs: &GlyphTable) -> Value {
    Value::List(
        glyphs
            .iter()
            .map(|(name, data)| {
                let mut entries = Vec::new();
                if let Some((icon1, icon2)) = &data.icon16x16 {
                    entries.push(Value::List(vec![
                        Value::str("icon16x16"),
                        Value::List(vec![ints(icon1), ints(icon2)]),
                    ]));
                }
                if let Some((slot, idata)) = &data.icon5x8 {
                    entries.push(Value::List(vec![
                        Value::str("icon5x8"),
                        Value::List(vec![Value::int(*slot), ints(idata)]),
                    ]));
                }
                Value::List(vec![Value::str(name), Value::List(entries)])
            })
            .collect(),
    )
}

impl LcdChipApi for FakeChip {
    fn set_glyphs(&self, glyphs: &GlyphTable) {
        self.st.log.lock().push(Value::Tuple(vec![
            Value::str("set_glyphs"),
            glyph_value(glyphs),
        ]));
    }
    fn get_dimensions(&self) -> (i64, i64) {
        (self.cols, 4)
    }
    fn init(&self) -> KResult<()> {
        if *self.st.fail_init.lock() {
            return Err(KlippyError::Internal {
                class_name: "RuntimeError".to_string(),
                message: "init boom".to_string(),
            });
        }
        self.st.log.lock().push(Value::Tuple(vec![Value::str("init")]));
        Ok(())
    }
    fn clear(&self) {
        self.st.log.lock().push(Value::Tuple(vec![Value::str("clear")]));
    }
    fn flush(&self) -> KResult<()> {
        if *self.st.fail_flush.lock() {
            return Err(KlippyError::Internal {
                class_name: "RuntimeError".to_string(),
                message: "flush boom".to_string(),
            });
        }
        self.st.log.lock().push(Value::Tuple(vec![Value::str("flush")]));
        Ok(())
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        if let Some(lcd) = self.st.redraw.lock().as_ref().and_then(|lcd| lcd.upgrade()) {
            lcd.request_redraw();
        }
        if self.st.fail_text.lock().as_deref() == Some(data) {
            return Err(KlippyError::Internal {
                class_name: "RuntimeError".to_string(),
                message: "chip boom".to_string(),
            });
        }
        self.st.log.lock().push(Value::Tuple(vec![
            Value::str("write_text"),
            Value::int(x),
            Value::int(y),
            Value::bytes(data),
        ]));
        Ok(())
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.st.log.lock().push(Value::Tuple(vec![
            Value::str("write_glyph"),
            Value::int(x),
            Value::int(y),
            Value::str(glyph_name),
        ]));
        Ok(self.st.glyph_ret.lock().get(glyph_name).copied().unwrap_or(1))
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.st.log.lock().push(Value::Tuple(vec![
            Value::str("write_graphics"),
            Value::int(x),
            Value::int(y),
            Value::List(data.iter().map(|b| Value::int(i64::from(*b))).collect()),
        ]));
        Ok(())
    }
}

fn fake_ctor(cols: i64, st: &Arc<ChipState>) -> ChipCtor {
    let st = st.clone();
    Arc::new(move |_config: &Config| {
        Ok(Arc::new(FakeChip {
            cols,
            st: st.clone(),
        }) as Arc<dyn LcdChipApi>)
    })
}


// -- Log capture ---------------------------------------------------------------

thread_local! {
    static LOGS: RefCell<Vec<(String, String)>> = const { RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        log::set_sink(Box::new(|rec: &log::Record<'_>| {
            LOGS.with(|logs| {
                logs.borrow_mut()
                    .push((rec.level.name().to_string(), rec.message.to_string()));
            });
        }));
    });
}

fn logs_value() -> Value {
    LOGS.with(|logs| {
        Value::List(
            logs.borrow()
                .iter()
                .map(|(level, msg)| Value::Tuple(vec![Value::str(level), Value::str(msg)]))
                .collect(),
        )
    })
}

// -- World and LCD ---------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    configfile: Arc<FakeConfigfile>,
    data: Arc<ConfigData>,
}

impl World {
    fn new(text: &str, variant: u8, fail_cfg: bool) -> World {
        install_log_sink();
        let reactor = Reactor::new(false);
        let printer = FakePrinter::new(reactor.clone());
        let printer_dyn: Printer = printer.clone();
        let data = Arc::new(parse_config(text, "test.cfg"));
        let gcode = FakeGCode::new(reactor);
        printer.set_object("gcode", gcode.clone());
        let macro_cfg = FakeConfig::new(printer_dyn.clone(), "gcode_macro", data.clone());
        printer.set_object("gcode_macro", Arc::new(PrinterGCodeMacro::new(&macro_cfg)));
        for (name, status) in statuses(variant) {
            printer.set_object(name, Arc::new(FakeStatus { st: status }));
        }
        let configfile = Arc::new(FakeConfigfile {
            printer: printer_dyn,
            fail: fail_cfg,
            read_calls: Mutex::new(Vec::new()),
        });
        printer.set_object("configfile", configfile.clone());
        World {
            printer,
            gcode,
            configfile,
            data,
        }
    }

    fn cfg(&self, section: &str) -> Config {
        FakeConfig::new(self.printer.clone(), section, self.data.clone())
    }
}

struct Lcd {
    lcd: Arc<PrinterLCD>,
    chip: Arc<ChipState>,
}

fn try_lcd(w: &World, section: &str) -> KResult<Lcd> {
    LOGS.with(|logs| logs.borrow_mut().clear());
    let chip = ChipState::new();
    let chips = vec![
        ("fake16".to_string(), fake_ctor(16, &chip)),
        ("fake20".to_string(), fake_ctor(20, &chip)),
    ];
    let lcd = PrinterLCD::with_chips(&w.cfg(section), &chips)?;
    for (name, ret) in GLYPH_RET {
        chip.glyph_ret.lock().insert(name.to_string(), ret);
    }
    Ok(Lcd { lcd, chip })
}

fn build_lcd(w: &World, section: &str) -> Lcd {
    match try_lcd(w, section) {
        Ok(l) => l,
        Err(e) => panic!("PrinterLCD::with_chips: {}: {}", e.class_name(), e.message()),
    }
}

/// `snap(lcd)` of the Python setup.
fn snap(l: &Lcd) -> Value {
    Value::Tuple(vec![
        Value::List(l.chip.log.lock().clone()),
        Value::Bool(l.lcd.redraw_request_pending()),
        Value::float(l.lcd.redraw_time()),
        Value::Bool(l.lcd.menu().is_some()),
        logs_value(),
    ])
}

/// `outcome(fn, lcd)` of the Python setup.
fn outcome(r: KResult<Value>, l: &Lcd) -> Value {
    match r {
        Ok(v) => Value::Tuple(vec![Value::str("ok"), v, snap(l)]),
        Err(e) => Value::Tuple(vec![
            Value::str("err"),
            Value::str(e.class_name()),
            Value::str(e.message()),
            snap(l),
        ]),
    }
}

fn opt_str(v: Option<&str>) -> Value {
    match v {
        Some(s) => Value::str(s),
        None => Value::Null,
    }
}

/// A Rust `PyValue` as a recording value.
fn pv(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(pyint_i64(n).expect("small int")),
        PyValue::Float(x) => Value::float(*x),
        PyValue::Str(x) => Value::str(x),
        PyValue::List(items) => Value::List(items.iter().map(pv).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pv).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (pv(k), pv(v))).collect()),
        other => panic!("unexpected value {other:?}"),
    }
}

fn group_is(l: &Lcd, name: &str) -> bool {
    match (l.lcd.show_data_group(), l.lcd.display_data_groups().get(name)) {
        (Some(shown), Some(group)) => Arc::ptr_eq(&shown, group),
        _ => false,
    }
}

/// `summary(dt, w)` of the Python setup.
fn summary(dt: &Arc<dyn PrinterDisplayTemplateApi>, w: &World) -> Value {
    let templates = Value::List(
        dt.get_display_templates()
            .iter()
            .map(|(name, t)| {
                Value::Tuple(vec![
                    Value::str(name),
                    Value::List(
                        t.get_params()
                            .iter()
                            .map(|(k, v)| Value::Tuple(vec![Value::str(k), pv(v)]))
                            .collect(),
                    ),
                ])
            })
            .collect(),
    );
    let groups = Value::List(
        dt.get_display_data_groups()
            .iter()
            .map(|(name, g)| {
                Value::Tuple(vec![
                    Value::str(name),
                    Value::List(
                        g.item_positions()
                            .iter()
                            .map(|(r, c)| Value::Tuple(vec![Value::int(*r), Value::int(*c)]))
                            .collect(),
                    ),
                ])
            })
            .collect(),
    );
    let again = lookup_display_templates(&w.cfg("display")).expect("second lookup");
    let reads = w.configfile.read_calls.lock().clone();
    Value::Tuple(vec![
        templates,
        groups,
        glyph_value(&dt.get_display_glyphs()),
        w.printer.object_names(),
        Value::Bool(Arc::ptr_eq(&again, dt)),
        Value::int(reads.len() as i64),
        Value::str(
            std::path::Path::new(&reads[0])
                .file_name()
                .and_then(|n| n.to_str())
                .expect("file name"),
        ),
        Value::Bool(reads[0].replace('\\', "/").ends_with("extras/display/display.cfg")),
    ])
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("display fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping display oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.display.display".to_string(),
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

fn check_exc(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let expected = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?}, expected an error"),
        Err(e) => e,
    };
    assert_eq!(expected.class_name, actual.class_name(), "{label}: class");
    assert_eq!(expected.message, actual.message(), "{label}: message");
}

fn py_str(text: &str) -> String {
    let mut out = String::from("'");
    for c in text.chars() {
        match c {
            '\'' => out.push_str("\\'"),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('\'');
    out
}

/// Build the Python world `w` from `text`.
fn py_world(py: &mut pyoracle::Oracle, text: &str, variant: u8, fail_cfg: bool) {
    let fail = if fail_cfg { "True" } else { "False" };
    py.exec(&format!(
        "w = World({}, {variant}, {fail})",
        py_str(text)
    ))
    .expect("python world");
}

/// Build the Python world `w` and the LCD `lcd` of section `display`.
fn py_lcd(py: &mut pyoracle::Oracle, text: &str, variant: u8) {
    py_world(py, text, variant, false);
    py.exec("lcd = make_lcd(w)").expect("python lcd");
}

// ---------------------------------------------------------------------------
// Tests: module surface.
// ---------------------------------------------------------------------------

#[test]
fn constants_chip_table_and_factory() {
    let Some(mut py) = oracle() else { return };
    check("REDRAW_TIME", py.eval("display_mod.REDRAW_TIME"), Value::float(REDRAW_TIME));
    check(
        "REDRAW_MIN_TIME",
        py.eval("display_mod.REDRAW_MIN_TIME"),
        Value::float(REDRAW_MIN_TIME),
    );
    check(
        "cmd_SET_DISPLAY_GROUP_help",
        py.eval("display_mod.PrinterLCD.cmd_SET_DISPLAY_GROUP_help"),
        Value::str(SET_DISPLAY_GROUP_HELP),
    );
    check(
        "LCD_chips keys",
        py.eval("ORIG_CHIPS"),
        Value::List(lcd_chips().iter().map(|(name, _)| Value::str(name)).collect()),
    );
    let factory = module_factory();
    check(
        "module entry points",
        py.eval("[hasattr(display_mod, name) for name in ('load_config', 'load_config_prefix', 'add_early_printer_objects', 'add_printer_objects')]"),
        Value::List(vec![
            Value::Bool(factory.load_config.is_some()),
            Value::Bool(factory.load_config_prefix.is_some()),
            Value::Bool(factory.add_early_printer_objects.is_some()),
            Value::Bool(factory.add_printer_objects.is_some()),
        ]),
    );
}

#[test]
fn load_config_rejects_unknown_lcd_type() {
    let Some(mut py) = oracle() else { return };
    let text = "[display]\nlcd_type: bogus\n";
    py_world(&mut py, text, 0, false);
    let expected = py.eval("display_mod.load_config(w.cfg('display'))");
    let w = World::new(text, 0, false);
    let load = module_factory().load_config.expect("load_config");
    let err = load(&w.cfg("display")).err().expect("unknown lcd_type");
    check_exc("load_config bogus lcd_type", expected, &err);
}

/// `extras/display/__init__.py`: the package entry points, the auxiliary
/// display checks, and delegation to `display.load_config` (reached through
/// the unknown `lcd_type` error).
#[test]
fn package_entry_points_check_auxiliary_sections() {
    let Some(mut py) = oracle() else { return };
    py.exec("import importlib\ndisplay_pkg = importlib.import_module('extras.display')")
        .expect("python display package");
    let factory = extras::display::module_factory();
    check(
        "package entry points",
        py.eval("[hasattr(display_pkg, name) for name in ('load_config', 'load_config_prefix', 'add_early_printer_objects', 'add_printer_objects')]"),
        Value::List(vec![
            Value::Bool(factory.load_config.is_some()),
            Value::Bool(factory.load_config_prefix.is_some()),
            Value::Bool(factory.add_early_printer_objects.is_some()),
            Value::Bool(factory.add_printer_objects.is_some()),
        ]),
    );
    let load = factory.load_config.expect("load_config");
    let load_prefix = factory.load_config_prefix.expect("load_config_prefix");
    let bogus = "lcd_type: bogus\n";
    let cases = [
        ("primary display", format!("[display]\n{bogus}"), "display", false),
        ("auxiliary without primary", format!("[display aux]\n{bogus}"), "display aux", true),
        (
            "auxiliary named display",
            format!("[display]\n{bogus}[display display]\n{bogus}"),
            "display display",
            true,
        ),
        (
            "auxiliary delegates to display.load_config",
            format!("[display]\n{bogus}[display aux]\n{bogus}"),
            "display aux",
            true,
        ),
    ];
    for (label, text, section, prefix) in cases {
        py_world(&mut py, &text, 0, false);
        let entry = if prefix { "load_config_prefix" } else { "load_config" };
        let expected = py.eval(&format!("display_pkg.{entry}(w.cfg({}))", py_str(section)));
        let w = World::new(&text, 0, false);
        let entry_fn = if prefix { load_prefix } else { load };
        let err = entry_fn(&w.cfg(section)).err().expect(label);
        check_exc(label, expected, &err);
    }
}

// ---------------------------------------------------------------------------
// Tests: PrinterDisplayTemplate / DisplayTemplate / DisplayGroup.
// ---------------------------------------------------------------------------

#[test]
fn default_templates_groups_and_glyphs() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, DISPLAY16, 0, false);
    py.exec("dt = display_mod.lookup_display_templates(w.cfg('display'))")
        .expect("python lookup");
    let expected = py.eval("summary(dt, w)");
    let w = World::new(DISPLAY16, 0, false);
    let dt = lookup_display_templates(&w.cfg("display")).expect("lookup");
    check("default summary", expected, summary(&dt, &w));
}

#[test]
fn user_sections_override_and_extend_defaults() {
    let Some(mut py) = oracle() else { return };
    let text = user_cfg();
    py_world(&mut py, &text, 0, false);
    py.exec("dt = display_mod.lookup_display_templates(w.cfg('display'))")
        .expect("python lookup");
    let expected = py.eval("summary(dt, w)");
    let w = World::new(&text, 0, false);
    let dt = lookup_display_templates(&w.cfg("display")).expect("lookup");
    check("user summary", expected, summary(&dt, &w));
}

#[test]
fn display_template_render_merges_context_and_params() {
    let Some(mut py) = oracle() else { return };
    let text = user_cfg();
    py_world(&mut py, &text, 0, false);
    py.exec("t = display_mod.lookup_display_templates(w.cfg('display')).get_display_templates()['mine']")
        .expect("python template");
    let w = World::new(&text, 0, false);
    let dt = lookup_display_templates(&w.cfg("display")).expect("lookup");
    let templates = dt.get_display_templates();
    let t = templates.get("mine").expect("mine");

    let ctx = |entries: &[(&str, PyValue)]| -> IndexMap<String, TplValue> {
        entries
            .iter()
            .map(|(k, v)| (k.to_string(), TplValue::py(v.clone())))
            .collect()
    };
    let kw = |entries: &[(&str, PyValue)]| -> IndexMap<String, PyValue> {
        entries.iter().map(|(k, v)| (k.to_string(), v.clone())).collect()
    };
    let int = |n: i64| PyValue::Int(PyInt::from_i64(n));

    let cases: Vec<RenderCase> = vec![
        ("defaults", "t.render({})", ctx(&[]), kw(&[])),
        (
            "context value and param override",
            "t.render({'x': 42, 'param_a': 'ctx'}, param_a=7)",
            ctx(&[("x", int(42)), ("param_a", s("ctx"))]),
            kw(&[("param_a", int(7))]),
        ),
        (
            "context param is overridden by the default",
            "t.render({'param_a': 'ctx'})",
            ctx(&[("param_a", s("ctx"))]),
            kw(&[]),
        ),
        (
            "string param",
            "t.render({}, param_b='yy', param_a=1.5)",
            ctx(&[]),
            kw(&[("param_b", s("yy")), ("param_a", f(1.5))]),
        ),
    ];
    for (label, expr, context, kwargs) in cases {
        let expected = py.eval(expr);
        let actual = t.render(&context, &kwargs).expect(label);
        check(label, expected, Value::str(&actual));
    }

    for (label, expr, kwargs) in [
        ("unknown parameter", "t.render({}, bogus=1)", kw(&[("bogus", int(1))])),
        ("context name is not a parameter", "t.render({}, x=1)", kw(&[("x", int(1))])),
    ] {
        let expected = py.eval(expr);
        let err = t.render(&ctx(&[]), &kwargs).expect_err(label);
        check_exc(label, expected, &err);
    }
}

#[test]
fn template_and_group_loading_errors() {
    let Some(mut py) = oracle() else { return };
    let g16 = |n: usize| rows(&G16A[..n]);
    let mut cases: Vec<(String, String)> = vec![
        (
            "template name with three tokens".to_string(),
            "[display_template a b]\ntext: x\n".to_string(),
        ),
        (
            "param is not a literal".to_string(),
            "[display_template t]\nparam_x: foo\ntext: x\n".to_string(),
        ),
        (
            "template without text".to_string(),
            "[display_template t]\nparam_x: 1\n".to_string(),
        ),
        (
            "data section with two tokens".to_string(),
            "[display_data g]\nposition: 0, 0\ntext: x\n".to_string(),
        ),
        (
            "data section with four tokens".to_string(),
            "[display_data g a b]\nposition: 0, 0\ntext: x\n".to_string(),
        ),
        (
            "data without position".to_string(),
            "[display_data g a]\ntext: x\n".to_string(),
        ),
        (
            "data without text".to_string(),
            "[display_data g a]\nposition: 0, 0\n".to_string(),
        ),
        (
            "glyph row too short".to_string(),
            format!("[display_glyph x]\ndata:\n{}  ...............\n", g16(15)),
        ),
        (
            "glyph row with a bad character".to_string(),
            format!("[display_glyph x]\ndata:\n{}  ..............x.\n", g16(15)),
        ),
        (
            "glyph with too few rows".to_string(),
            format!("[display_glyph x]\ndata:\n{}", g16(15)),
        ),
        (
            "glyph with too many rows".to_string(),
            format!("[display_glyph x]\ndata:\n{}  ................\n  ................\n", g16(16)),
        ),
        (
            "hd44780 glyph without slot".to_string(),
            format!("[display_glyph x]\nhd44780_data:\n{}", rows(&G5)),
        ),
        (
            "hd44780 slot too high".to_string(),
            format!("[display_glyph x]\nhd44780_data:\n{}hd44780_slot: 8\n", rows(&G5)),
        ),
        (
            "hd44780 slot negative".to_string(),
            format!("[display_glyph x]\nhd44780_data:\n{}hd44780_slot: -1\n", rows(&G5)),
        ),
        (
            "hd44780 slot not a number".to_string(),
            format!("[display_glyph x]\nhd44780_data:\n{}hd44780_slot: abc\n", rows(&G5)),
        ),
        (
            "hd44780 row too wide".to_string(),
            format!(
                "[display_glyph x]\nhd44780_data:\n  ......\n{}hd44780_slot: 1\n",
                rows(&G5[..7])
            ),
        ),
        (
            "hd44780 glyph with too few rows".to_string(),
            format!("[display_glyph x]\nhd44780_data:\n{}hd44780_slot: 1\n", rows(&G5[..7])),
        ),
        (
            "valid 16x16 glyph".to_string(),
            format!("[display_glyph x]\ndata:\n{}", g16(16)),
        ),
    ];
    for bad in ["", "1", "a, b", "1, 2, 3", "1.5, 2", "1,", ",1"] {
        cases.push((
            format!("position '{bad}'"),
            format!("[display_data g a]\nposition: {bad}\ntext: x\n"),
        ));
    }
    for (label, extra) in cases {
        let text = format!("{DISPLAY16}\n{extra}");
        py_world(&mut py, &text, 0, false);
        let expected = py.eval("display_mod.lookup_display_templates(w.cfg('display'))");
        let w = World::new(&text, 0, false);
        match (expected, lookup_display_templates(&w.cfg("display"))) {
            (Err(e), Err(err)) => check_exc(&label, Err(e), &err),
            (Ok(_), Ok(_)) => {}
            (Ok(_), Err(err)) => panic!("{label}: python loaded, rust failed: {err}"),
            (Err(e), Ok(_)) => panic!("{label}: python raised {}: {}", e.class_name, e.message),
        }
    }
}

#[test]
fn unreadable_default_config_is_a_config_error() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, DISPLAY16, 0, true);
    py.exec(
        r#"
def read_fail():
    try:
        display_mod.lookup_display_templates(w.cfg('display'))
    except Exception as e:
        return (type(e).__name__,
                str(e).replace(w.configfile.read_calls[0], '@FILE@'))
"#,
    )
    .expect("python helper");
    let expected = py.eval("read_fail()");
    let w = World::new(DISPLAY16, 0, true);
    let err = lookup_display_templates(&w.cfg("display")).err().expect("fails");
    let filename = w.configfile.read_calls.lock()[0].clone();
    let actual = Value::Tuple(vec![
        Value::str(err.class_name()),
        Value::str(&err.message().replace(&filename, "@FILE@")),
    ]);
    check("cannot load config", expected, actual);
    check(
        "failed load does not cache",
        py.eval("w.printer.lookup_object('display_template', None) is None"),
        Value::Bool(w.printer.lookup_object(
            "display_template", LookupDefault::ReturnNone,
        ).expect("lookup").is_none()),
    );
}

#[test]
fn position_forms_are_parsed_like_int() {
    let Some(mut py) = oracle() else { return };
    let text = format!(
        "{DISPLAY16}\n\
         [display_data g a]\nposition: +1, -2\ntext: x\n\
         [display_data g b]\nposition:  3 ,4 \ntext: x\n\
         [display_data g c]\nposition: 1_0, 2\ntext: x\n"
    );
    py_world(&mut py, &text, 0, false);
    py.exec("dt = display_mod.lookup_display_templates(w.cfg('display'))")
        .expect("python lookup");
    let expected = py.eval("[(r, c) for r, c, _ in dt.get_display_data_groups()['g'].data_items]");
    let w = World::new(&text, 0, false);
    let dt = lookup_display_templates(&w.cfg("display")).expect("lookup");
    let groups = dt.get_display_data_groups();
    let positions = groups.get("g").expect("group g").item_positions();
    check(
        "positions",
        expected,
        Value::List(
            positions
                .iter()
                .map(|(r, c)| Value::Tuple(vec![Value::int(*r), Value::int(*c)]))
                .collect(),
        ),
    );
}

// ---------------------------------------------------------------------------
// Tests: PrinterLCD construction.
// ---------------------------------------------------------------------------

fn construction_state(w: &World, l: &Lcd) -> Value {
    let dims = l.lcd.get_dimensions();
    Value::Tuple(vec![
        w.printer.handler_names(),
        w.gcode.mux_value(),
        Value::List(l.lcd.display_data_groups().keys().map(|k| Value::str(k)).collect()),
        Value::List(l.lcd.display_templates().keys().map(|k| Value::str(k)).collect()),
        w.printer.object_names(),
        Value::Bool(l.lcd.menu().is_some()),
        Value::Tuple(vec![Value::int(dims.0), Value::int(dims.1)]),
        snap(l),
    ])
}

const CONSTRUCTION_EXPR: &str = "(list(w.printer.handlers), w.gcode.mux, list(lcd.display_data_groups), \
     list(lcd.display_templates), list(w.printer.objects), \
     lcd.menu is not None, lcd.get_dimensions(), snap(lcd))";

#[test]
fn primary_display_registers_menu_group_and_two_commands() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, PRIMARY16, 0, false);
    py.exec("lcd = make_lcd(w, 'display')").expect("primary display");
    let expected = py.eval(&format!(
        "({CONSTRUCTION_EXPR}, lcd.show_data_group is lcd.display_data_groups['normal'])"
    ));
    let w = World::new(PRIMARY16, 0, false);
    let l = build_lcd(&w, "display");
    check(
        "16 column display",
        expected,
        Value::Tuple(vec![
            construction_state(&w, &l),
            Value::Bool(group_is(&l, "normal")),
        ]),
    );
}

#[test]
fn twenty_column_display_defaults_to_the_20x4_group() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY20, 0);
    let expected = py.eval(&format!(
        "({CONSTRUCTION_EXPR}, lcd.show_data_group is lcd.display_data_groups['_default_20x4'])"
    ));
    let w = World::new(DISPLAY20, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    check(
        "20 column display",
        expected,
        Value::Tuple(vec![
            construction_state(&w, &l),
            Value::Bool(group_is(&l, "_default_20x4")),
        ]),
    );
}

#[test]
fn secondary_display_has_no_menu_and_one_command() {
    let Some(mut py) = oracle() else { return };
    let text = DISPLAY16;
    py_lcd(&mut py, text, 0);
    let expected = py.eval(CONSTRUCTION_EXPR);
    let w = World::new(text, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    check("secondary display", expected, construction_state(&w, &l));
}

#[test]
fn display_group_option_selects_the_group() {
    let Some(mut py) = oracle() else { return };
    let text = "[display my_display]\nlcd_type: fake16\ndisplay_group: _multiextruder_16x4\n";
    py_lcd(&mut py, text, 0);
    let expected = py.eval(
        "(lcd.show_data_group is lcd.display_data_groups['_multiextruder_16x4'], \
         lcd.show_data_group is lcd.display_data_groups['_default_16x4'])",
    );
    let w = World::new(text, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    check(
        "display_group option",
        expected,
        Value::Tuple(vec![
            Value::Bool(group_is(&l, "_multiextruder_16x4")),
            Value::Bool(group_is(&l, "_default_16x4")),
        ]),
    );
}

#[test]
fn construction_errors() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("missing lcd_type", "[display my_display]\n"),
        ("unknown lcd_type", "[display my_display]\nlcd_type: bogus\n"),
        (
            "unknown display_group",
            "[display my_display]\nlcd_type: fake16\ndisplay_group: nope\n",
        ),
        ("lcd_type is case sensitive", "[display my_display]\nlcd_type: FAKE16\n"),
    ];
    for (label, text) in cases {
        py_world(&mut py, text, 0, false);
        let expected = py.eval("make_lcd(w)");
        let w = World::new(text, 0, false);
        let err = try_lcd(&w, DISPLAY_SECTION).err().expect(label);
        check_exc(label, expected, &err);
    }
}

#[test]
fn glyphs_reach_the_chip_in_config_order() {
    let Some(mut py) = oracle() else { return };
    let text = user_cfg();
    py_lcd(&mut py, &text, 0);
    let expected = py.eval("list(lcd.lcd_chip.log)");
    let w = World::new(&text, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    check("glyph table", expected, Value::List(l.chip.log.lock().clone()));
}

// ---------------------------------------------------------------------------
// Tests: screen updates.
// ---------------------------------------------------------------------------

#[test]
fn ready_initializes_the_chip() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    let expected = py.eval("(w.printer.send_event('klippy:ready'), snap(lcd), lcd.lcd_chip.log[-1])");
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    let returned = w.printer.send_event("klippy:ready", &[]).expect("ready");
    let last = l.chip.log.lock().last().cloned().expect("chip log");
    check(
        "klippy:ready",
        expected,
        Value::Tuple(vec![
            Value::List(returned.iter().map(pv).collect()),
            snap(&l),
            last,
        ]),
    );
}

fn run_updates(label: &str, text: &str, variant: u8, times: &[f64]) {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, text, variant);
    let w = World::new(text, variant, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    for (i, t) in times.iter().enumerate() {
        let expected = py.eval(&format!("outcome(lambda: lcd.screen_update_event({t:?}), lcd)"));
        let actual = outcome(l.lcd.screen_update_event(*t).map(Value::float), &l);
        check(&format!("{label} #{i}"), expected, actual);
    }
}

#[test]
fn default_16x4_screen() {
    run_updates("16x4", DISPLAY16, 0, &[10.0, 10.5]);
}

#[test]
fn default_16x4_screen_with_idle_state() {
    run_updates("16x4 variant", DISPLAY16, 1, &[3.25]);
}

#[test]
fn default_20x4_screen() {
    run_updates("20x4", DISPLAY20, 0, &[10.0]);
}

#[test]
fn default_20x4_screen_with_idle_state() {
    run_updates("20x4 variant", DISPLAY20, 1, &[10.0]);
}

#[test]
fn multiextruder_16x4_screen() {
    let text = "[display my_display]\nlcd_type: fake16\ndisplay_group: _multiextruder_16x4\n";
    run_updates("multiextruder", text, 0, &[7.0]);
}

#[test]
fn user_group_screen_renders_nested_templates() {
    run_updates("user group", &user_cfg(), 0, &[1.0]);
}

#[test]
fn menu_drawing_the_screen_skips_the_data_group() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, PRIMARY16, 0, false);
    py.exec("lcd = make_lcd(w, 'display')").expect("primary display");
    let w = World::new(PRIMARY16, 0, false);
    let l = build_lcd(&w, "display");
    for running in [false, true] {
        if running {
            py.exec("lcd.menu.begin(3.0)").expect("begin menu");
            l.lcd.menu().expect("primary menu").begin(3.0).expect("begin menu");
        }
        py.exec("lcd.lcd_chip.log.clear()").expect("clear chip log");
        l.chip.log.lock().clear();
        let expected = py.eval(
            "(lcd.screen_update_event(4.0), \
             any(c[0] == 'write_text' and c[3] == b'NORMAL' for c in lcd.lcd_chip.log), \
             [c[0] for c in lcd.lcd_chip.log if c[0] in ('clear', 'flush')])",
        );
        let next = l.lcd.screen_update_event(4.0).expect("screen update");
        let calls = l.chip.log.lock();
        let normal = calls.iter().any(|call| match call {
            Value::Tuple(parts) => parts.first() == Some(&Value::str("write_text"))
                && parts.get(3) == Some(&Value::bytes(b"NORMAL")),
            _ => false,
        });
        let framing = calls.iter().filter_map(|call| match call {
            Value::Tuple(parts) if parts.first() == Some(&Value::str("clear"))
                || parts.first() == Some(&Value::str("flush")) => parts.first().cloned(),
            _ => None,
        }).collect();
        check("primary menu screen ownership", expected, Value::Tuple(vec![
            Value::float(next), Value::Bool(normal), Value::List(framing),
        ]));
    }
}

#[test]
fn redraw_requests_set_the_pending_state_once() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    // First request arms the timer, the second is ignored while pending.
    check(
        "request_redraw",
        py.eval("(lcd.request_redraw(), lcd.request_redraw(), snap(lcd))"),
        {
            l.lcd.request_redraw();
            l.lcd.request_redraw();
            Value::Tuple(vec![
                Value::Null,
                Value::Null,
                snap(&l),
            ])
        },
    );
    // The next screen update clears the flag and schedules REDRAW_MIN_TIME ahead.
    let expected = py.eval("outcome(lambda: lcd.screen_update_event(5.0), lcd)");
    let actual = outcome(l.lcd.screen_update_event(5.0).map(Value::float), &l);
    check("update after request", expected, actual);
}

#[test]
fn redraw_requested_while_drawing_returns_redraw_time() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    py.exec("lcd.lcd_chip.redraw = True").expect("redraw during write");
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    *l.chip.redraw.lock() = Some(Arc::downgrade(&l.lcd));
    // A chip callback requests redraw during drawing on both sides.
    for t in [2.0, 3.0] {
        let expected = py.eval(&format!("outcome(lambda: lcd.screen_update_event({t:?}), lcd)"));
        let actual = outcome(l.lcd.screen_update_event(t).map(Value::float), &l);
        check(&format!("poked update at {t}"), expected, actual);
    }
}

// ---------------------------------------------------------------------------
// Tests: drawing helpers.
// ---------------------------------------------------------------------------

#[test]
fn draw_text_writes_text_and_glyph_segments() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    let cases: [(&str, &str, i64, i64, &str); 7] = [
        ("plain", "'hello'", 0, 0, "hello"),
        ("glyph between text", "'ab~degrees~cd~x~'", 1, 2, "ab~degrees~cd~x~"),
        ("wide glyph advances two", "'~extruder~200~bed~'", 3, 0, "~extruder~200~bed~"),
        ("unterminated glyph", "'a~b'", 0, 3, "a~b"),
        ("empty", "''", 2, 2, ""),
        ("two tildes", "'~~'", 0, 0, "~~"),
        ("non ascii text counts characters", "'caf\\u00e9~fan1~z'", 0, 1, "caf\u{e9}~fan1~z"),
    ];
    // `menu.MenuManager` reaches `draw_text` through `PrinterLCDApi`.
    let display: &dyn PrinterLCDApi = &*l.lcd;
    for (label, py_text, row, col, rust_text) in cases {
        let expected = py.eval(&format!(
            "outcome(lambda: lcd.draw_text({row}, {col}, {py_text}, 0.), lcd)"
        ));
        let actual = outcome(display.draw_text(row, col, rust_text, 0.).map(Value::int), &l);
        check(label, expected, actual);
    }
}

#[test]
fn draw_text_propagates_chip_errors_after_earlier_writes() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    py.exec("lcd.lcd_chip.fail_text = b'BOOM'").expect("python chip");
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    *l.chip.fail_text.lock() = Some(b"BOOM".to_vec());
    let expected = py.eval("outcome(lambda: lcd.draw_text(0, 1, 'ok~fan1~BOOM~bed~tail', 0.), lcd)");
    let actual = outcome(
        l.lcd.draw_text(0, 1, "ok~fan1~BOOM~bed~tail", 0.).map(Value::int),
        &l,
    );
    check("chip error", expected, actual);
}

#[test]
fn draw_progress_bar_writes_graphics_cells() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    let cases: [(&str, &str, i64, i64, i64, f64); 14] = [
        ("quarter", "0.25", 2, 0, 10, 0.25),
        ("full", "1.0", 1, 10, 6, 1.0),
        ("empty", "0.0", 0, 0, 1, 0.0),
        ("half four cells", "0.5", 3, 5, 4, 0.5),
        ("round half up", "0.0625", 2, 1, 4, 0.0625),
        ("below zero", "-0.5", 2, 0, 3, -0.5),
        ("above one", "1.5", 2, 0, 10, 1.5),
        ("just above one", "1.01", 2, 0, 10, 1.01),
        ("zero width", "0.5", 2, 0, 0, 0.5),
        ("negative width", "0.5", 2, 0, -2, 0.5),
        ("nan", "float('nan')", 2, 0, 4, f64::NAN),
        ("infinity", "float('inf')", 2, 0, 4, f64::INFINITY),
        ("negative infinity", "float('-inf')", 2, 0, 4, f64::NEG_INFINITY),
        ("negative column", "0.75", 1, -3, 5, 0.75),
    ];
    for (label, py_value, row, col, width, value) in cases {
        let expected = py.eval(&format!(
            "outcome(lambda: lcd.draw_progress_bar({row}, {col}, {width}, {py_value}), lcd)"
        ));
        let actual = outcome(
            l.lcd.draw_progress_bar(row, col, width, value).map(|t| Value::str(&t)),
            &l,
        );
        check(label, expected, actual);
    }
}

// ---------------------------------------------------------------------------
// Tests: show() errors and the template callables.
// ---------------------------------------------------------------------------

fn group_cfg(items: &[(&str, &str, &str)]) -> String {
    let mut text = String::from("[display my_display]\nlcd_type: fake16\ndisplay_group: g\n");
    for (name, position, body) in items {
        text.push_str(&format!(
            "\n[display_data g {name}]\nposition: {position}\ntext: {body}\n"
        ));
    }
    text
}

type ShowCase<'a> = (&'a str, Vec<(&'a str, &'a str, &'a str)>);

/// Run `show` directly and through `screen_update_event` for one group.
fn compare_show(py: &mut pyoracle::Oracle, label: &str, items: &[(&str, &str, &str)]) {
    let text = group_cfg(items);
    py_lcd(py, &text, 0);
    let w = World::new(&text, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);

    let expected = py.eval("outcome(lambda: lcd.show_data_group.show(lcd, lcd.display_templates, 1.0), lcd)");
    let group = l.lcd.show_data_group().expect("group g");
    let actual = outcome(
        group
            .show(&l.lcd, &l.lcd.display_templates(), 1.0)
            .map(|()| Value::Null),
        &l,
    );
    check(&format!("{label}: show"), expected, actual);

    let expected = py.eval("outcome(lambda: lcd.screen_update_event(2.0), lcd)");
    let actual = outcome(l.lcd.screen_update_event(2.0).map(Value::float), &l);
    check(&format!("{label}: screen_update_event"), expected, actual);
}

fn compare_show_cases(cases: Vec<ShowCase<'_>>) {
    let Some(mut py) = oracle() else { return };
    for (label, items) in cases {
        compare_show(&mut py, label, &items);
    }
}

#[test]
fn show_draws_items_in_position_order() {
    compare_show_cases(vec![(
        "ordered items",
        vec![
            ("late", "3, 0", "late"),
            ("first", "0, 5", "first"),
            ("second", "0, 6", "\n  sec\n  ond"),
        ],
    )]);
}

#[test]
fn show_without_items_raises_index_error() {
    compare_show_cases(vec![("no items", vec![("empty", "0, 0", "")])]);
}

#[test]
fn show_unknown_template_is_logged_after_earlier_items() {
    compare_show_cases(vec![(
        "unknown template",
        vec![
            ("a", "0, 0", "before"),
            ("b", "1, 0", "{ render(\"nope\") }"),
            ("c", "2, 0", "after"),
        ],
    )]);
}

#[test]
fn show_template_parameter_errors() {
    let one = |label, body| -> ShowCase<'static> { (label, vec![("a", "0, 0", body)]) };
    compare_show_cases(vec![
        one("bad parameter", "{ render(\"_fan_speed\", bogus=1) }"),
        one("render without name", "{ render() }"),
        one("render with two names", "{ render(\"_fan_speed\", \"x\") }"),
        one("render name keyword", "{ render(name=\"_fan_speed\") }"),
        one("render name twice", "{ render(\"_fan_speed\", name=\"x\") }"),
        one("render with a number", "{ render(3) }"),
        one("render with a list", "{ render([1]) }"),
    ]);
}

#[test]
fn show_progress_bar_callable() {
    let one = |label, body| -> ShowCase<'static> { (label, vec![("a", "0, 0", body)]) };
    compare_show_cases(vec![
        one("progress bar", "{ draw_progress_bar(1, 2, 3, 0.4) }x"),
        one("progress bar keywords", "{ draw_progress_bar(1, 2, value=0.9, width=2) }"),
        one("progress bar missing two", "{ draw_progress_bar(1, 2) }"),
        one("progress bar missing four", "{ draw_progress_bar() }"),
        one("progress bar missing one", "{ draw_progress_bar(1, 2, 3) }"),
        one("progress bar too many", "{ draw_progress_bar(1, 2, 3, 0.5, 5) }"),
        one("progress bar unexpected keyword", "{ draw_progress_bar(1, 2, 3, 0.5, extra=1) }"),
        one("progress bar duplicate", "{ draw_progress_bar(1, 2, 3, 0.5, row=1) }"),
        one("progress bar negative shift", "{ draw_progress_bar(1, 2, 3, 2.0) }"),
        one("progress bar string value", "{ draw_progress_bar(1, 2, 3, \"x\") }"),
        one("progress bar bool width", "{ draw_progress_bar(1, 2, True, 0.5) }"),
    ]);
}

#[test]
fn show_nested_render_and_glyph_text() {
    compare_show_cases(vec![(
        "nested render",
        vec![
            ("a", "0, 0", "{ render(\"_printing_time\") }~clock~{ render(\"_fan_speed\") }"),
            ("b", "1, 0", "{ render(\"_heater_temperature\", param_heater_name=\"heater_bed\") }"),
        ],
    )]);
}

#[test]
fn chip_error_during_show_is_logged_and_the_screen_is_flushed() {
    let Some(mut py) = oracle() else { return };
    let text = group_cfg(&[("a", "0, 0", "fine"), ("b", "1, 0", "BOOM"), ("c", "2, 0", "tail")]);
    py_lcd(&mut py, &text, 0);
    py.exec("lcd.lcd_chip.fail_text = b'BOOM'").expect("python chip");
    let w = World::new(&text, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    *l.chip.fail_text.lock() = Some(b"BOOM".to_vec());
    let expected = py.eval("outcome(lambda: lcd.screen_update_event(2.0), lcd)");
    let actual = outcome(l.lcd.screen_update_event(2.0).map(Value::float), &l);
    check("chip error is logged", expected, actual);
}

// ---------------------------------------------------------------------------
// Tests: SET_DISPLAY_GROUP.
// ---------------------------------------------------------------------------

#[test]
fn set_display_group_switches_the_group_for_both_registrations() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, PRIMARY16, 0, false);
    py.exec("lcd = make_lcd(w, 'display')").expect("primary display");
    let w = World::new(PRIMARY16, 0, false);
    let l = build_lcd(&w, "display");
    let cases: Vec<DispatchCase> = vec![
        (
            "named display",
            Some("display"),
            "SET_DISPLAY_GROUP DISPLAY=display GROUP=_default_20x4",
            vec![("GROUP", "_default_20x4")],
        ),
        (
            "default display",
            None,
            "SET_DISPLAY_GROUP GROUP=_multiextruder_16x4",
            vec![("GROUP", "_multiextruder_16x4")],
        ),
        (
            "unknown group",
            Some("display"),
            "SET_DISPLAY_GROUP DISPLAY=display GROUP=nope",
            vec![("GROUP", "nope")],
        ),
        (
            "missing group",
            None,
            "SET_DISPLAY_GROUP DISPLAY=display",
            vec![],
        ),
        (
            "group names are case sensitive",
            Some("display"),
            "SET_DISPLAY_GROUP DISPLAY=display GROUP=_DEFAULT_16X4",
            vec![("GROUP", "_DEFAULT_16X4")],
        ),
        (
            "back to the default",
            None,
            "SET_DISPLAY_GROUP GROUP=_default_16x4",
            vec![("GROUP", "_default_16x4")],
        ),
        (
            "empty group name",
            Some("display"),
            "SET_DISPLAY_GROUP DISPLAY=display GROUP=",
            vec![("GROUP", "")],
        ),
    ];
    for (label, value, commandline, params) in cases {
        let py_value = match value {
            Some(v) => py_str(v),
            None => "None".to_string(),
        };
        let py_params = format!(
            "{{{}}}",
            params
                .iter()
                .map(|(k, v)| format!("{}: {}", py_str(k), py_str(v)))
                .collect::<Vec<_>>()
                .join(", ")
        );
        let expected = py.eval(&format!(
            "(outcome(lambda: w.gcode.dispatch({py_value}, {}, {py_params}), lcd), \
             [n for n, g in lcd.display_data_groups.items() if g is lcd.show_data_group])",
            py_str(commandline)
        ));
        let result = w.gcode.dispatch(value, commandline, &params).map(|()| Value::Null);
        let shown: Vec<Value> = l
            .lcd
            .display_data_groups()
            .keys()
            .filter(|name| group_is(&l, name))
            .map(|name| Value::str(name))
            .collect();
        check(
            label,
            expected,
            Value::Tuple(vec![outcome(result, &l), Value::List(shown)]),
        );
    }
}

#[test]
fn switched_group_is_the_one_drawn() {
    let Some(mut py) = oracle() else { return };
    let text = user_cfg();
    py_lcd(&mut py, &text, 0);
    py.exec("w.gcode.dispatch('display my_display', 'SET_DISPLAY_GROUP DISPLAY=\"display my_display\" GROUP=other', {'GROUP': 'other'})")
        .expect("python switch");
    let w = World::new(&text, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    w.gcode
        .dispatch(
            Some(DISPLAY_SECTION),
            "SET_DISPLAY_GROUP DISPLAY=\"display my_display\" GROUP=other",
            &[("GROUP", "other")],
        )
        .expect("switch group");
    let expected = py.eval("outcome(lambda: lcd.screen_update_event(6.0), lcd)");
    let actual = outcome(l.lcd.screen_update_event(6.0).map(Value::float), &l);
    check("drawn group", expected, actual);
}

struct TemplateCacheView(Arc<dyn PrinterDisplayTemplateApi>);

impl PrinterDisplayTemplateApi for TemplateCacheView {
    fn get_display_templates(&self) -> IndexMap<String, Arc<dyn DisplayTemplateApi>> {
        self.0.get_display_templates()
    }
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        self.0.get_display_data_groups()
    }
    fn get_display_glyphs(&self) -> GlyphTable {
        self.0.get_display_glyphs()
    }
}

klippy_api::printer_object!(TemplateCacheView; dyn PrinterDisplayTemplateApi);

#[test]
fn template_cache_lookup_uses_the_registered_interface() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, DISPLAY16, 0, false);
    py.exec(r#"
base = display_mod.lookup_display_templates(w.cfg('display'))
class CacheView:
    get_display_templates = staticmethod(base.get_display_templates)
    get_display_data_groups = staticmethod(base.get_display_data_groups)
    get_display_glyphs = staticmethod(base.get_display_glyphs)
dt = CacheView()
w.printer.objects['display_template'] = dt
"#).expect("alternative cache provider");
    let w = World::new(DISPLAY16, 0, false);
    let base = lookup_display_templates(&w.cfg(DISPLAY_SECTION)).expect("initial cache");
    w.printer.set_object("display_template", Arc::new(TemplateCacheView(base)));
    let dt = lookup_display_templates(&w.cfg(DISPLAY_SECTION)).expect("cache interface");
    check("alternative cache provider", py.eval("summary(dt, w)"), summary(&dt, &w));
}

#[test]
fn chip_initialization_and_flush_errors_propagate() {
    let Some(mut py) = oracle() else { return };
    py_lcd(&mut py, DISPLAY16, 0);
    let w = World::new(DISPLAY16, 0, false);
    let l = build_lcd(&w, DISPLAY_SECTION);
    py.exec("lcd.lcd_chip.fail_init = True").expect("fail init");
    *l.chip.fail_init.lock() = true;
    let expected = py.eval("outcome(lambda: lcd.handle_ready(), lcd)");
    check("init exception", expected, outcome(l.lcd.handle_ready().map(|()| Value::Null), &l));
    py.exec("lcd.lcd_chip.fail_init = False; lcd.lcd_chip.fail_flush = True")
        .expect("fail flush");
    *l.chip.fail_init.lock() = false;
    *l.chip.fail_flush.lock() = true;
    let expected = py.eval("outcome(lambda: lcd.screen_update_event(2.0), lcd)");
    check("flush exception", expected, outcome(l.lcd.screen_update_event(2.0).map(Value::float), &l));
}

#[test]
fn python_control_whitespace_is_accepted_in_display_positions_and_glyphs() {
    let Some(mut py) = oracle() else { return };
    let glyph = G16A.iter().map(|line| format!("  \u{1c}{line}\u{1f}\n"))
        .collect::<String>();
    let text = format!(
        "{DISPLAY16}\n[display_data control item]\nposition: \u{1c}1\u{1f}, \u{1d}2\u{1e}\ntext: x\n\
[display_glyph control]\ndata:\n{glyph}"
    );
    py_world(&mut py, &text, 0, false);
    py.exec("dt = display_mod.lookup_display_templates(w.cfg('display'))")
        .expect("control whitespace config");
    let w = World::new(&text, 0, false);
    let dt = lookup_display_templates(&w.cfg(DISPLAY_SECTION)).expect("control whitespace");
    check("control whitespace", py.eval("summary(dt, w)"), summary(&dt, &w));
}
