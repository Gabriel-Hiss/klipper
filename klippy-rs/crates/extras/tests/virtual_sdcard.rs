//! Parity tests for `extras::virtual_sdcard` (port of
//! `klippy/extras/virtual_sdcard.py`).
//!
//! Every expected value comes from the live Python reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`). Each scenario builds a
//! `VirtualSD` on both sides from the same config and the same files on disk,
//! runs the same list of steps (G-code commands dispatched to the registered
//! handlers, the real reactor driven until the print ends, status/stats
//! queries, `klippy:analyze_shutdown` events, ...) and compares one
//! transcript of text lines:
//!
//! - `REG:`/`HANDLER:` the commands (with `when_not_ready` and help text) and
//!   event handlers registered at construction,
//! - `INIT:` the construction result or the config error,
//! - `CMD:` the outcome of every dispatched command (class and message of the
//!   error), `RESP:` every `respond_raw`, `RUN:` every `gcode.run_script`,
//! - `STATS:` every `print_stats` call, `EVENT:` every `send_event`,
//! - `STATUS:`/`STATSLINE:`/`LIST:`/`POS:` the object's `get_status`, `stats`,
//!   `get_file_list` and file position API,
//! - `LOG:` every log record of the module (level, message, exception text).
//!
//! The two sides use the real `reactor` (greenlets in Python, baton-passing
//! threads in Rust), the real `gcode_macro` for the `on_error_gcode`
//! template, real files and the real `print_stats` call protocol (a recording
//! fake on both sides). The g-code run by the print is a scripted fake: a line
//! whose first word is a registered command is dispatched to its handler;
//! `FAIL <msg>` raises a command error, `RAISE <msg>` a `RuntimeError`,
//! `SLOW` sleeps 50 ms, `JUMP <pos> <times>` calls `set_file_position` (like
//! `sdcard_loop`), `TRIGGER_PAUSE`/`TRIGGER_CANCEL` start a reactor timer that
//! calls `do_pause`/`do_cancel` from another greenlet, `SELF_CANCEL` calls
//! `do_cancel` from inside the command and `SHUTDOWN` sends
//! `klippy:analyze_shutdown`.
//!
//! Wall-clock values cannot match between two processes, so the `%.6f`
//! decimals of `[DEBUG-motion]` lines are replaced by `F` on both sides; every
//! counter, position, name and state in those lines is compared exactly.
//! Slow-command classification uses a 50 ms sleep against the 20 ms
//! threshold, so it is deterministic unless the host stalls for 20 ms inside a
//! fast command.

use extras::gcode_macro::PrinterGCodeMacro;
use extras::print_stats::PrintStatsApi;
use extras::virtual_sdcard::{self, debug_cmd_name, VirtualSDApi};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{interface, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use kos::sync::Mutex;
use pycompat::text::{str_repr, str_repr_ascii};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, ReactorMutex, NEVER, NOW};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, LazyLock, Once};

// ---------------------------------------------------------------------------
// Python side
// ---------------------------------------------------------------------------

const SETUP: &str = r##"
import logging as _logging
import time as _time
import reactor as _rmod
import extras.virtual_sdcard as vsd_mod
from extras import gcode_macro as gcode_macro_mod
from gcode import GCodeCommand as _GC, CommandError as _CE
from configfile import ConfigWrapper as _CW, error as _config_error
from configparser import RawConfigParser as _RP
from collections import OrderedDict

T = []
PENDING = [0]
JUMPS = {}
P = G = V = None

class FakePrinter:
    def __init__(self):
        self.reactor = _rmod.Reactor()
        self.objects = OrderedDict()
        self.handlers = {}
    def get_printer(self):
        return self
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return {}
    def is_shutdown(self):
        return False
    def add_object(self, name, obj):
        self.objects[name] = obj
    _sentinel = object()
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is self._sentinel:
            raise _config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        return [(k, v) for k, v in self.objects.items()
                if k == module or k.startswith(module + ' ')]
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if default is self._sentinel:
            raise _config_error("Unable to load module '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        T.append("HANDLER:" + event)
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        T.append("EVENT:" + event)
        return [cb(*params) for cb in self.handlers.get(event, [])]

class FakePrintStats:
    def set_current_file(self, filename):
        T.append("STATS:set_current_file:" + filename)
    def note_start(self):
        T.append("STATS:note_start")
    def note_pause(self):
        T.append("STATS:note_pause")
    def note_complete(self):
        T.append("STATS:note_complete")
    def note_error(self, message):
        T.append("STATS:note_error:" + message)
    def note_cancel(self):
        T.append("STATS:note_cancel")
    def reset(self):
        T.append("STATS:reset")

class FakeHeaters:
    def get_status(self, eventtime):
        return {}

def _trigger(kind):
    r = P.reactor
    PENDING[0] += 1
    def cb(et):
        try:
            if kind == 'pause':
                V.do_pause()
                T.append("TRIGGER:pause:done")
            else:
                try:
                    V.do_cancel()
                    T.append("TRIGGER:cancel:done")
                except Exception as e:
                    T.append("TRIGGER:cancel:%s:%s" % (type(e).__name__, e))
        finally:
            PENDING[0] -= 1
        return r.NEVER
    r.register_timer(cb, r.NOW)

class FakeGCode:
    error = _CE
    def __init__(self, printer):
        self.printer = printer
        self.cmds = OrderedDict()
        self.mutex = printer.reactor.mutex()
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        T.append("REG:%s:%s:%s" % (cmd, when_not_ready, desc))
        self.cmds[cmd] = func
    def get_mutex(self):
        return self.mutex
    def respond_raw(self, msg):
        T.append("RESP:" + msg)
    def respond_info(self, msg, log=True):
        T.append("INFO:" + msg)
    def run_script(self, script):
        T.append("RUN:" + ascii(script))
        toks = script.split()
        first = toks[0] if toks else ''
        if first in self.cmds:
            gcmd = _GC(self, first, script, {}, False)
            self.cmds[first](gcmd)
        elif first == 'FAIL':
            raise _CE(script[len('FAIL'):].strip())
        elif first == 'RAISE':
            raise RuntimeError(script[len('RAISE'):].strip())
        elif first == 'SLOW':
            _time.sleep(0.05)
        elif first == 'JUMP':
            n = int(toks[1])
            m = int(toks[2])
            c = JUMPS.get(script, 0)
            if c < m:
                JUMPS[script] = c + 1
                V.set_file_position(n)
        elif first == 'TRIGGER_PAUSE':
            _trigger('pause')
        elif first == 'TRIGGER_CANCEL':
            _trigger('cancel')
        elif first == 'SELF_CANCEL':
            try:
                V.do_cancel()
            except Exception as e:
                T.append("SELFCANCEL:%s:%s" % (type(e).__name__, e))
        elif first.startswith('LONGFAIL'):
            raise _CE('long fail')
        elif first == 'SHUTDOWN':
            P.send_event('klippy:analyze_shutdown', 'msg', {})

log_records = []
class _Cap(_logging.Handler):
    def emit(self, record):
        if not record.pathname.endswith(('virtual_sdcard.py', 'gcode_macro.py')):
            return
        detail = ''
        if record.exc_info and record.exc_info[1] is not None:
            e = record.exc_info[1]
            detail = type(e).__name__ + ': ' + str(e)
        T.append("LOG:%s:%s|%s" % (record.levelname, record.getMessage(), detail))
_root = _logging.getLogger()
_root.addHandler(_Cap())
_root.setLevel(0)

def make_cfg(printer, name, opts):
    p = _RP(strict=False)
    p.add_section(name)
    for k, v in opts.items():
        p.set(name, k, v)
    return _CW(printer, p, {}, name)

def build(path, opts, heaters):
    global P, G, V
    PENDING[0] = 0
    JUMPS.clear()
    P = FakePrinter()
    G = FakeGCode(P)
    P.objects['gcode'] = G
    P.objects['print_stats'] = FakePrintStats()
    if heaters:
        P.objects['heaters'] = FakeHeaters()
    macro_cfg = make_cfg(P, 'gcode_macro', {})
    P.objects['gcode_macro'] = gcode_macro_mod.PrinterGCodeMacro(macro_cfg)
    del T[:]
    o = dict(opts)
    if path is not None:
        o['path'] = path
    cfg = make_cfg(P, 'virtual_sdcard', o)
    try:
        V = vsd_mod.load_config(cfg)
        T.append("INIT:ok")
    except Exception as e:
        V = None
        T.append("INIT:%s:%s" % (type(e).__name__, e))

def drive():
    r = P.reactor
    ticks = [0]
    def monitor(et):
        ticks[0] += 1
        idle = (not V.is_active()) and PENDING[0] == 0
        if idle or ticks[0] > 20000:
            if not idle:
                T.append("DRIVE_TIMEOUT")
            r.end()
            return r.NEVER
        return et + 0.001
    r.register_timer(monitor, r.monotonic() + 0.001)
    r.run()

def run_steps(steps):
    for st in steps:
        k = st[0]
        if k == 'cmd':
            command, line, params = st[1], st[2], st[3]
            gcmd = _GC(G, command, line, params, False)
            try:
                G.cmds[command](gcmd)
                T.append("CMD:%s:ok" % (line,))
            except _CE as e:
                T.append("CMD:%s:CommandError:%s" % (line, e))
            except Exception as e:
                T.append("CMD:%s:%s:%s" % (line, type(e).__name__, e))
        elif k == 'drive':
            drive()
        elif k == 'status':
            T.append("STATUS:" + repr(V.get_status(0.)))
        elif k == 'stats':
            T.append("STATSLINE:" + repr(V.stats(0.)))
        elif k == 'list':
            try:
                T.append("LIST:" + repr(V.get_file_list(st[1])))
            except Exception as e:
                T.append("LIST:ERR:%s:%s" % (type(e).__name__, e))
        elif k == 'shutdown':
            P.send_event('klippy:analyze_shutdown', 'msg', {})
        elif k == 'lock':
            m = G.mutex
            m.lock()
            def unlock_cb(et, m=m):
                m.unlock()
                return P.reactor.NEVER
            P.reactor.register_timer(unlock_cb, P.reactor.monotonic() + st[1] / 1000.)
        elif k == 'setpos':
            V.set_file_position(st[1])
            T.append("POS:%d" % (V.get_file_position(),))
        elif k == 'cmdfromsd':
            T.append("CMDFROMSD:%s" % (V.is_cmd_from_sd(),))
        elif k == 'resume':
            try:
                V.do_resume()
                T.append("RESUME:ok")
            except Exception as e:
                T.append("RESUME:%s:%s" % (type(e).__name__, e))
        elif k == 'pause':
            V.do_pause()
            T.append("PAUSE:done")
        elif k == 'cancel':
            try:
                V.do_cancel()
                T.append("CANCEL:ok")
            except Exception as e:
                T.append("CANCEL:%s:%s" % (type(e).__name__, e))
"##;

fn q(s: &str) -> String {
    str_repr(s)
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

// ---------------------------------------------------------------------------
// Transcript and log capture
// ---------------------------------------------------------------------------

struct Transcript {
    lines: Mutex<Vec<String>>,
}

impl Transcript {
    fn new() -> Arc<Transcript> {
        Arc::new(Transcript { lines: Mutex::new(Vec::new()) })
    }
    fn push(&self, line: impl Into<String>) {
        self.lines.lock().push(line.into());
    }
    fn clear(&self) {
        self.lines.lock().clear();
    }
    fn take(&self) -> Vec<String> {
        self.lines.lock().clone()
    }
}

static CURRENT: LazyLock<Mutex<Option<Arc<Transcript>>>> = LazyLock::new(|| Mutex::new(None));
static SINK: Once = Once::new();

fn install_sink() {
    SINK.call_once(|| {
        let installed = log::set_sink(Box::new(|rec: &log::Record<'_>| {
            let current = CURRENT.lock().clone();
            if let Some(t) = current {
                t.push(format!(
                    "LOG:{}:{}|{}",
                    rec.level.name(),
                    rec.message,
                    rec.exc_text.unwrap_or("")
                ));
            }
        }));
        assert!(installed, "a log sink was already installed");
    });
}

// ---------------------------------------------------------------------------
// Rust-side fakes
// ---------------------------------------------------------------------------

struct FakeConfig {
    printer: Printer,
    name: String,
    opts: IndexMap<String, String>,
}

impl FakeConfig {
    fn build(printer: Printer, name: &str, opts: &[(String, String)]) -> Config {
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            opts: opts.iter().cloned().collect(),
        })
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
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        unimplemented!("getint is not read by virtual_sdcard")
    }
    fn getfloat(&self, _o: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        unimplemented!("getfloat is not read by virtual_sdcard")
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        unimplemented!("getboolean is not read by virtual_sdcard")
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        unimplemented!("getchoice is not read by virtual_sdcard")
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unimplemented!("getlists is not read by virtual_sdcard")
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unimplemented!("getlist is not read by virtual_sdcard")
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unimplemented!("getintlist is not read by virtual_sdcard")
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        unimplemented!("getfloatlist is not read by virtual_sdcard")
    }
    fn getsection(&self, _section: &str) -> Config {
        unimplemented!("getsection is not used by virtual_sdcard")
    }
    fn has_section(&self, _section: &str) -> bool {
        unimplemented!("has_section is not used by virtual_sdcard")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        unimplemented!("get_prefix_sections is not used by virtual_sdcard")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        unimplemented!("get_prefix_options is not used by virtual_sdcard")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        unimplemented!("deprecate is not used by virtual_sdcard")
    }
}

/// `GCodeCommand` with the real parameter helpers' messages
/// (`klippy/gcode.py:34-96`).
struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    tr: Arc<Transcript>,
}

impl FakeGCmd {
    fn missing(&self, name: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{}': missing {name}", self.commandline))
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
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            };
        };
        let Ok(v) = raw.trim().parse::<i64>() else {
            return Err(KlippyError::command(format!(
                "Error on '{}': unable to parse {raw}",
                self.commandline
            )));
        };
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have minimum of {min}",
                    self.commandline
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have maximum of {max}",
                    self.commandline
                )));
            }
        }
        Ok(Some(v))
    }
    fn get_float(&self, _name: &str, _opts: GGetFloatOpts) -> KResult<Option<f64>> {
        unimplemented!("get_float is not read by virtual_sdcard")
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
        // `GCodeCommand.get_raw_command_parameters` for a line that starts
        // with the command (`gcode.py:40-53`).
        let skip = self.command.chars().count();
        let rest: String = self.commandline.chars().skip(skip).collect();
        match rest.chars().next() {
            Some(c) if c.is_whitespace() => rest.chars().skip(1).collect(),
            _ => rest,
        }
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.tr.push(format!("INFO:{msg}"));
    }
    fn respond_raw(&self, msg: &str) {
        self.tr.push(format!("RESP:{msg}"));
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

type HandlerCell = Arc<GCell<Option<GCodeCommandHandler>>>;

struct FakeGCode {
    reactor: Reactor,
    tr: Arc<Transcript>,
    pending: Arc<AtomicI64>,
    /// `self.mutex = printer.reactor.mutex()` of the Python fake.
    mutex: ReactorMutex,
    cmds: GCell<IndexMap<String, HandlerCell>>,
    vsd: GCell<Option<Arc<dyn VirtualSDApi>>>,
    printer: GCell<Option<Printer>>,
    jumps: GCell<IndexMap<String, i64>>,
}

impl FakeGCode {
    fn new(reactor: Reactor, tr: Arc<Transcript>, pending: Arc<AtomicI64>) -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            mutex: reactor.mutex(false),
            reactor,
            tr,
            pending,
            cmds: GCell::new(IndexMap::new()),
            vsd: GCell::new(None),
            printer: GCell::new(None),
            jumps: GCell::new(IndexMap::new()),
        })
    }

    fn make_gcmd(&self, command: &str, commandline: &str, params: IndexMap<String, String>) -> GCmd {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params,
            tr: self.tr.clone(),
        })
    }

    fn has_cmd(&self, name: &str) -> bool {
        self.cmds.borrow().contains_key(name)
    }

    /// Calls the registered handler without holding any borrow across the
    /// call (handlers pause the reactor).
    fn call_cmd(&self, name: &str, gcmd: &GCmd) -> KResult<()> {
        let cell = self.cmds.borrow().get(name).cloned().expect("registered command");
        let mut handler = cell.borrow_mut().take().expect("handler is not re-entered");
        let result = handler(gcmd);
        *cell.borrow_mut() = Some(handler);
        result
    }

    fn vsd(&self) -> Arc<dyn VirtualSDApi> {
        self.vsd.borrow().clone().expect("virtual_sdcard object")
    }

    fn trigger(&self, kind: &'static str) {
        let vsd = self.vsd();
        let pending = self.pending.clone();
        let tr = self.tr.clone();
        pending.fetch_add(1, Ordering::SeqCst);
        self.reactor.register_timer(
            move |_et| {
                if kind == "pause" {
                    vsd.do_pause();
                    tr.push("TRIGGER:pause:done");
                } else {
                    match vsd.do_cancel() {
                        Ok(()) => tr.push("TRIGGER:cancel:done"),
                        Err(e) => {
                            tr.push(format!("TRIGGER:cancel:{}:{}", e.class_name(), e.message()))
                        }
                    }
                }
                pending.fetch_sub(1, Ordering::SeqCst);
                NEVER
            },
            NOW,
        );
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.tr.push(format!(
            "REG:{cmd}:{}:{}",
            py_bool(opts.when_not_ready),
            opts.desc.as_deref().unwrap_or("None")
        ));
        if let Some(func) = func {
            self.cmds
                .borrow_mut()
                .insert(cmd.to_string(), Arc::new(GCell::new(Some(func))));
        }
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
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.tr.push(format!("RUN:{}", str_repr_ascii(script)));
        let toks: Vec<&str> = script.split_whitespace().collect();
        let first = toks.first().copied().unwrap_or("");
        if self.has_cmd(first) {
            let gcmd = self.make_gcmd(first, script, IndexMap::new());
            return self.call_cmd(first, &gcmd);
        }
        match first {
            "FAIL" => Err(KlippyError::command(script["FAIL".len()..].trim())),
            "RAISE" => Err(KlippyError::Internal {
                class_name: "RuntimeError".to_string(),
                message: script["RAISE".len()..].trim().to_string(),
            }),
            "SLOW" => {
                kos::time::sleep(0.05);
                Ok(())
            }
            "JUMP" => {
                let n: i64 = toks[1].parse().expect("JUMP position");
                let m: i64 = toks[2].parse().expect("JUMP count");
                let done = self.jumps.borrow().get(script).copied().unwrap_or(0);
                if done < m {
                    self.jumps.borrow_mut().insert(script.to_string(), done + 1);
                    self.vsd().set_file_position(n);
                }
                Ok(())
            }
            "TRIGGER_PAUSE" => {
                self.trigger("pause");
                Ok(())
            }
            "TRIGGER_CANCEL" => {
                self.trigger("cancel");
                Ok(())
            }
            "SELF_CANCEL" => {
                if let Err(e) = self.vsd().do_cancel() {
                    self.tr.push(format!("SELFCANCEL:{}:{}", e.class_name(), e.message()));
                }
                Ok(())
            }
            s if s.starts_with("LONGFAIL") => Err(KlippyError::command("long fail")),
            "SHUTDOWN" => {
                let printer = self.printer.borrow().clone().expect("printer");
                printer.send_event(
                    "klippy:analyze_shutdown",
                    &[
                        EventArg::Value(PyValue::Str("msg".to_string())),
                        EventArg::Value(PyValue::Dict(IndexMap::new())),
                    ],
                )?;
                Ok(())
            }
            _ => Ok(()),
        }
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.mutex.clone()
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        self.make_gcmd(command, commandline, params.clone())
    }
    fn respond_raw(&self, msg: &str) {
        self.tr.push(format!("RESP:{msg}"));
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.tr.push(format!("INFO:{msg}"));
    }
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

struct FakePrintStats {
    tr: Arc<Transcript>,
}

impl PrintStatsApi for FakePrintStats {
    fn set_current_file(&self, filename: &str) {
        self.tr.push(format!("STATS:set_current_file:{filename}"));
    }
    fn note_start(&self) {
        self.tr.push("STATS:note_start");
    }
    fn note_pause(&self) {
        self.tr.push("STATS:note_pause");
    }
    fn note_complete(&self) {
        self.tr.push("STATS:note_complete");
    }
    fn note_error(&self, message: &str) {
        self.tr.push(format!("STATS:note_error:{message}"));
    }
    fn note_cancel(&self) {
        self.tr.push("STATS:note_cancel");
    }
    fn reset(&self) {
        self.tr.push("STATS:reset");
    }
}

klippy_api::printer_object!(FakePrintStats; dyn PrintStatsApi);

/// A printer object with an empty `get_status` (`'heaters' in printer`).
struct FakeHeaters;

impl PrinterObject for FakeHeaters {
    klippy_api::printer_object_base!();
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(PyValue::Dict(IndexMap::new())))
    }
}

struct FakePrinter {
    reactor: Reactor,
    tr: Arc<Transcript>,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn set_object(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
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
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objs = self.objects.lock();
        objs.iter()
            .filter(|(k, _)| match module {
                None => true,
                Some(m) => k.as_str() == m || k.starts_with(&format!("{m} ")),
            })
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(section) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("Unable to load module '{section}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.tr.push(format!("HANDLER:{event}"));
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.tr.push(format!("EVENT:{event}"));
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

// ---------------------------------------------------------------------------
// Scenario description and runner
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum Step {
    Cmd { command: String, line: String, params: Vec<(String, String)> },
    Drive,
    Status,
    Stats,
    List(bool),
    Shutdown,
    Lock(u32),
    SetPos(i64),
    CmdFromSd,
    Resume,
    Pause,
    Cancel,
}

fn cmd(command: &str, line: &str) -> Step {
    Step::Cmd { command: command.to_string(), line: line.to_string(), params: Vec::new() }
}

fn cmdp(command: &str, line: &str, params: &[(&str, &str)]) -> Step {
    Step::Cmd {
        command: command.to_string(),
        line: line.to_string(),
        params: params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
    }
}

fn py_steps(steps: &[Step]) -> String {
    let items: Vec<String> = steps
        .iter()
        .map(|s| match s {
            Step::Cmd { command, line, params } => {
                let ps: Vec<String> =
                    params.iter().map(|(k, v)| format!("{}: {}", q(k), q(v))).collect();
                format!("('cmd', {}, {}, {{{}}})", q(command), q(line), ps.join(", "))
            }
            Step::Drive => "('drive',)".to_string(),
            Step::Status => "('status',)".to_string(),
            Step::Stats => "('stats',)".to_string(),
            Step::List(sub) => format!("('list', {})", py_bool(*sub)),
            Step::Shutdown => "('shutdown',)".to_string(),
            Step::Lock(ms) => format!("('lock', {ms})"),
            Step::SetPos(n) => format!("('setpos', {n})"),
            Step::CmdFromSd => "('cmdfromsd',)".to_string(),
            Step::Resume => "('resume',)".to_string(),
            Step::Pause => "('pause',)".to_string(),
            Step::Cancel => "('cancel',)".to_string(),
        })
        .collect();
    format!("[{}]", items.join(", "))
}

struct Spec {
    label: String,
    path: Option<String>,
    opts: Vec<(String, String)>,
    heaters: bool,
    steps: Vec<Step>,
}

impl Spec {
    fn new(label: &str, path: Option<&str>) -> Spec {
        Spec {
            label: label.to_string(),
            path: path.map(|p| p.to_string()),
            opts: Vec::new(),
            heaters: false,
            steps: Vec::new(),
        }
    }
    fn in_dir(label: &str, dir: &Path) -> Spec {
        Spec::new(label, Some(dir.to_str().expect("utf-8 temp path")))
    }
    fn opt(mut self, key: &str, value: &str) -> Spec {
        self.opts.push((key.to_string(), value.to_string()));
        self
    }
    fn heaters(mut self) -> Spec {
        self.heaters = true;
        self
    }
    fn steps(mut self, steps: Vec<Step>) -> Spec {
        self.steps = steps;
        self
    }
}

struct World {
    reactor: Reactor,
    tr: Arc<Transcript>,
    gcode: Arc<FakeGCode>,
    obj: Option<ObjectRef>,
    vsd: Option<Arc<dyn VirtualSDApi>>,
    pending: Arc<AtomicI64>,
}

fn build_world(spec: &Spec) -> World {
    let tr = Transcript::new();
    let reactor = Reactor::new(false);
    let pending = Arc::new(AtomicI64::new(0));
    let printer = Arc::new(FakePrinter {
        reactor: reactor.clone(),
        tr: tr.clone(),
        objects: Mutex::new(IndexMap::new()),
        handlers: Mutex::new(Vec::new()),
    });
    let gcode = FakeGCode::new(reactor.clone(), tr.clone(), pending.clone());
    printer.set_object("gcode", gcode.clone());
    printer.set_object("print_stats", Arc::new(FakePrintStats { tr: tr.clone() }));
    if spec.heaters {
        printer.set_object("heaters", Arc::new(FakeHeaters));
    }
    let printer_dyn: Printer = printer.clone();
    let macro_config = FakeConfig::build(printer_dyn.clone(), "gcode_macro", &[]);
    printer.set_object("gcode_macro", Arc::new(PrinterGCodeMacro::new(&macro_config)));
    *gcode.printer.borrow_mut() = Some(printer_dyn.clone());
    tr.clear();
    *CURRENT.lock() = Some(tr.clone());

    let mut opts = spec.opts.clone();
    if let Some(path) = &spec.path {
        opts.push(("path".to_string(), path.clone()));
    }
    let config = FakeConfig::build(printer_dyn, "virtual_sdcard", &opts);
    let load = virtual_sdcard::module_factory().load_config.expect("load_config entry point");
    let (obj, vsd) = match load(&config) {
        Ok(Some(obj)) => {
            tr.push("INIT:ok");
            let vsd = interface::<dyn VirtualSDApi>(&obj).expect("VirtualSDApi interface");
            *gcode.vsd.borrow_mut() = Some(vsd.clone());
            (Some(obj), Some(vsd))
        }
        Ok(None) => panic!("{}: load_config returned no object", spec.label),
        Err(e) => {
            tr.push(format!("INIT:{}:{}", e.class_name(), e.message()));
            (None, None)
        }
    };
    World { reactor, tr, gcode, obj, vsd, pending }
}

impl World {
    fn drive(&self) {
        let vsd = self.vsd.clone().expect("object");
        let pending = self.pending.clone();
        let tr = self.tr.clone();
        let reactor = self.reactor.clone();
        let ticks = AtomicI64::new(0);
        self.reactor.register_timer(
            move |et| {
                let t = ticks.fetch_add(1, Ordering::SeqCst) + 1;
                let idle = !vsd.is_active() && pending.load(Ordering::SeqCst) == 0;
                if idle || t > 20000 {
                    if !idle {
                        tr.push("DRIVE_TIMEOUT");
                    }
                    reactor.end();
                    return NEVER;
                }
                et + 0.001
            },
            self.reactor.monotonic() + 0.001,
        );
        self.reactor.run();
    }

    fn run_steps(&self, steps: &[Step]) {
        let vsd = self.vsd.clone().expect("object");
        let obj = self.obj.clone().expect("object");
        for step in steps {
            match step {
                Step::Cmd { command, line, params } => {
                    let params: IndexMap<String, String> = params.iter().cloned().collect();
                    let gcmd = self.gcode.make_gcmd(command, line, params);
                    match self.gcode.call_cmd(command, &gcmd) {
                        Ok(()) => self.tr.push(format!("CMD:{line}:ok")),
                        Err(e) => self
                            .tr
                            .push(format!("CMD:{line}:{}:{}", e.class_name(), e.message())),
                    }
                }
                Step::Drive => self.drive(),
                Step::Status => {
                    let st = obj.get_status(0.).unwrap().expect("get_status");
                    self.tr.push(format!("STATUS:{}", st.repr()));
                }
                Step::Stats => {
                    let (active, text) = obj.stats(0.).expect("stats");
                    let v = PyValue::Tuple(vec![PyValue::Bool(active), PyValue::Str(text)]);
                    self.tr.push(format!("STATSLINE:{}", v.repr()));
                }
                Step::List(sub) => match vsd.get_file_list(*sub) {
                    Ok(list) => {
                        let v = PyValue::List(
                            list.into_iter()
                                .map(|(name, size)| {
                                    PyValue::Tuple(vec![
                                        PyValue::Str(name),
                                        PyValue::Int(PyInt::from_u64(size)),
                                    ])
                                })
                                .collect(),
                        );
                        self.tr.push(format!("LIST:{}", v.repr()));
                    }
                    Err(e) => self.tr.push(format!("LIST:ERR:{}:{}", e.class_name(), e.message())),
                },
                Step::Shutdown => {
                    let printer = self.gcode.printer.borrow().clone().expect("printer");
                    printer
                        .send_event(
                            "klippy:analyze_shutdown",
                            &[
                                EventArg::Value(PyValue::Str("msg".to_string())),
                                EventArg::Value(PyValue::Dict(IndexMap::new())),
                            ],
                        )
                        .expect("send_event");
                }
                Step::Lock(ms) => {
                    // `m = G.mutex; m.lock()`, unlocked by a timer.
                    let m = self.gcode.mutex.clone();
                    m.lock();
                    self.reactor.register_timer(
                        move |_et| {
                            m.unlock();
                            NEVER
                        },
                        self.reactor.monotonic() + f64::from(*ms) / 1000.,
                    );
                }
                Step::SetPos(n) => {
                    vsd.set_file_position(*n);
                    self.tr.push(format!("POS:{}", vsd.get_file_position()));
                }
                Step::CmdFromSd => self.tr.push(format!(
                    "CMDFROMSD:{}",
                    py_bool(vsd.is_cmd_from_sd())
                )),
                Step::Resume => match vsd.do_resume() {
                    Ok(()) => self.tr.push("RESUME:ok"),
                    Err(e) => self.tr.push(format!("RESUME:{}:{}", e.class_name(), e.message())),
                },
                Step::Pause => {
                    vsd.do_pause();
                    self.tr.push("PAUSE:done");
                }
                Step::Cancel => match vsd.do_cancel() {
                    Ok(()) => self.tr.push("CANCEL:ok"),
                    Err(e) => self.tr.push(format!("CANCEL:{}:{}", e.class_name(), e.message())),
                },
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Oracle plumbing and comparison
// ---------------------------------------------------------------------------

/// Serializes the scenarios: the log sink, the oracle namespace and the
/// temp directories are process-wide.
static SERIAL: LazyLock<Mutex<Option<pyoracle::Oracle>>> = LazyLock::new(|| Mutex::new(None));

/// Replaces the `%.6f` decimals of `[DEBUG-motion]` lines (wall-clock
/// values) by `F`.
fn normalize(line: &str) -> String {
    if !line.contains("[DEBUG-motion]") {
        return line.to_string();
    }
    let chars: Vec<char> = line.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        let mut j = i;
        if chars[j] == '-' {
            j += 1;
        }
        let digits_start = j;
        while j < chars.len() && chars[j].is_ascii_digit() {
            j += 1;
        }
        if j > digits_start && j < chars.len() && chars[j] == '.' {
            let frac_start = j + 1;
            let mut k = frac_start;
            while k < chars.len() && chars[k].is_ascii_digit() {
                k += 1;
            }
            if k - frac_start == 6 {
                out.push('F');
                i = k;
                continue;
            }
        }
        out.push(chars[i]);
        i += 1;
    }
    out
}

fn as_value(lines: &[String]) -> Value {
    Value::List(lines.iter().map(|s| Value::Str(s.clone())).collect())
}

fn compare(label: &str, expected: pyoracle::OracleResult, actual: Vec<String>) {
    let expected = match expected {
        Ok(Value::List(items)) => items
            .into_iter()
            .map(|v| match v {
                Value::Str(s) => s,
                other => panic!("{label}: transcript entry is not a string: {other:?}"),
            })
            .collect::<Vec<String>>(),
        Ok(other) => panic!("{label}: oracle returned {other:?}"),
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let expected: Vec<String> = expected.iter().map(|l| normalize(l)).collect();
    let actual: Vec<String> = actual.iter().map(|l| normalize(l)).collect();
    if expected != actual {
        let n = expected.iter().zip(actual.iter()).take_while(|(a, b)| a == b).count();
        panic!(
            "{label}: transcripts differ at line {n} (expected {} lines, got {})\n  expected: {:?}\n  actual:   {:?}\n--- expected ---\n{}\n--- actual ---\n{}",
            expected.len(),
            actual.len(),
            expected.get(n),
            actual.get(n),
            expected.join("\n"),
            actual.join("\n"),
        );
    }
    let record = Record {
        v: 1,
        unit: "extras.virtual_sdcard".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "transcript".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "transcript", &as_value(&expected), &as_value(&actual)) {
        panic!("{label}: {d}");
    }
}

/// Runs one scenario on both sides; `false` when the oracle is not available.
fn parity(spec: Spec) {
    let mut guard = SERIAL.lock();
    if guard.is_none() {
        match pyoracle::Oracle::start() {
            Some(mut py) => {
                py.exec(SETUP).expect("virtual_sdcard fakes");
                *guard = Some(py);
            }
            None => {
                println!(
                    "skipping virtual_sdcard oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                     binary and run with the repo checkout available"
                );
                return;
            }
        }
    }
    let py = guard.as_mut().expect("oracle");
    install_sink();

    // Python side.
    let path_py = match &spec.path {
        Some(p) => q(p),
        None => "None".to_string(),
    };
    let opts_py: Vec<String> = spec.opts.iter().map(|(k, v)| format!("{}: {}", q(k), q(v))).collect();
    py.exec(&format!(
        "build({path_py}, {{{}}}, {})",
        opts_py.join(", "),
        py_bool(spec.heaters)
    ))
    .unwrap_or_else(|e| panic!("{}: build raised {}: {}", spec.label, e.class_name, e.message));
    if spec.steps.is_empty() {
        // Nothing to run: only the construction transcript is compared.
    } else {
        py.exec(&format!("run_steps({})", py_steps(&spec.steps)))
            .unwrap_or_else(|e| panic!("{}: steps raised {}: {}", spec.label, e.class_name, e.message));
    }
    let expected = py.eval("list(T)");

    // Rust side.
    let world = build_world(&spec);
    if !spec.steps.is_empty() {
        world.run_steps(&spec.steps);
    }
    let actual = world.tr.take();
    *CURRENT.lock() = None;
    world.reactor.finalize();

    compare(&spec.label, expected, actual);
}

// ---------------------------------------------------------------------------
// Temp trees
// ---------------------------------------------------------------------------

/// A temp directory removed when dropped.
struct Tree(PathBuf);

impl std::ops::Deref for Tree {
    type Target = Path;
    fn deref(&self) -> &Path {
        &self.0
    }
}

impl Drop for Tree {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn tree(tag: &str, files: &[(&str, Vec<u8>)]) -> Tree {
    let dir = std::env::temp_dir().join(format!("vsd_parity_{}_{tag}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("create temp dir");
    for (name, data) in files {
        let path = dir.join(name);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).expect("create parent");
        }
        std::fs::write(&path, data).expect("write file");
    }
    Tree(dir)
}

fn b(s: &str) -> Vec<u8> {
    s.as_bytes().to_vec()
}

/// Lines of filler g-code that pad a file past the 8192-character read size.
fn filler(count: usize) -> String {
    (0..count).map(|i| format!("G1 X{i} Y{i} E0.{i:04}\n")).collect()
}

// ---------------------------------------------------------------------------
// Tests: construction
// ---------------------------------------------------------------------------

#[test]
fn init_registers_commands_and_handlers() {
    let dir = tree("init", &[]);
    parity(Spec::in_dir("init", &dir));
}

#[test]
fn init_requires_path() {
    parity(Spec::new("init_no_path", None));
}

#[test]
fn init_custom_on_error_option() {
    let dir = tree("init2", &[]);
    parity(Spec::in_dir("init_custom_on_error", &dir).opt("on_error_gcode", "ERRHOOK {1 + 2}"));
}

#[test]
fn path_is_expanded_and_normalized() {
    for (i, path) in [
        "~/vsd_parity_nonexistent/x/../y//",
        "//vsd_parity_nonexistent//b",
        "///vsd_parity_nonexistent_c",
        "./vsd_parity_rel/../r2",
        "~vsd_parity_no_such_user/q",
        "/vsd_parity_nonexistent/../vsd_parity_other/./z",
        "vsd_parity_relative_dir",
    ]
    .iter()
    .enumerate()
    {
        parity(
            Spec::new(&format!("path_{i}"), Some(path)).steps(vec![
                cmd("M20", "M20"),
                Step::List(false),
                Step::List(true),
            ]),
        );
    }
}

// ---------------------------------------------------------------------------
// Tests: file listing
// ---------------------------------------------------------------------------

fn listing_tree(tag: &str) -> Tree {
    let dir = tree(
        tag,
        &[
            ("b.gcode", b("G1 X1\n")),
            ("Alpha.gcode", b("G28\n")),
            ("zeta.GCODE", b("M84\n")),
            ("gcode", b("no dot\n")),
            ("x.g", b("a")),
            ("y.gco", b("abc")),
            ("notes.txt", b("hello world\n")),
            (".hidden.gcode", b("M0\n")),
            ("\u{e9}t\u{e9}.gcode", b("M117 \u{e9}\n")),
            ("Mixed.Case.GcOdE", b("M1\n")),
            ("sub/deep/inner.gco", b("G92 E0\n")),
            ("sub/.dot.gcode", b("M2\n")),
            ("sub/skip.txt", b("txt")),
            ("sub/Upper.G", b("M3\n")),
            ("empty/readme.md", b("md")),
        ],
    );
    #[cfg(unix)]
    {
        use std::os::unix::fs::symlink;
        symlink("b.gcode", dir.join("lnk.gcode")).expect("file symlink");
        symlink("sub", dir.join("linkdir")).expect("dir symlink");
    }
    dir
}

#[test]
fn file_list_flat_and_recursive() {
    let dir = listing_tree("list");
    parity(Spec::in_dir("list", &dir).steps(vec![
        Step::List(false),
        Step::List(true),
        cmd("M20", "M20"),
    ]));
}

#[test]
fn file_list_errors() {
    // Missing directory: flat listing fails with the command error, the
    // recursive walk silently finds nothing.
    parity(Spec::new("list_missing", Some("/vsd_parity_missing_dir")).steps(vec![
        Step::List(false),
        Step::List(true),
        cmd("M20", "M20"),
    ]));
    // A broken symlink is listed by os.walk and fails in getsize.
    let dir = tree("list_broken", &[("ok.gcode", b("M1\n"))]);
    #[cfg(unix)]
    std::os::unix::fs::symlink("nowhere.gcode", dir.join("broken.gcode")).expect("symlink");
    parity(Spec::in_dir("list_broken", &dir).steps(vec![Step::List(false), Step::List(true)]));
    // The configured path is a regular file.
    let dir = tree("list_file", &[("plain.gcode", b("M1\n"))]);
    let file = dir.join("plain.gcode");
    let spec = Spec::new("list_path_is_file", Some(file.to_str().expect("utf-8")));
    parity(spec.steps(vec![Step::List(false), Step::List(true), cmd("M20", "M20")]));
}

// ---------------------------------------------------------------------------
// Tests: plain commands
// ---------------------------------------------------------------------------

#[test]
fn simple_commands() {
    let dir = listing_tree("simple");
    parity(Spec::in_dir("simple_commands", &dir).steps(vec![
        cmd("M21", "M21"),
        cmd("M27", "M27"),
        cmd("M28", "M28 file.gcode"),
        cmd("M29", "M29"),
        cmd("M30", "M30 x"),
        cmdp("M26", "M26 S100", &[("S", "100")]),
        cmd("M26", "M26"),
        cmdp("M26", "M26 S-1", &[("S", "-1")]),
        cmdp("M26", "M26 Sabc", &[("S", "abc")]),
        Step::Status,
        Step::Stats,
        cmd("M25", "M25"),
        Step::CmdFromSd,
        Step::SetPos(1234),
        cmd("SDCARD_RESET_FILE", "SDCARD_RESET_FILE"),
        Step::Status,
        Step::Cancel,
        Step::Pause,
    ]));
}

#[test]
fn m23_selection() {
    let dir = listing_tree("m23");
    parity(Spec::in_dir("m23", &dir).steps(vec![
        cmd("M23", "M23 b.gcode"),
        Step::Status,
        cmd("M27", "M27"),
        cmd("M23", "M23 /alpha.GCODE"),
        Step::Status,
        cmd("M23", "M23   notes.txt  "),
        cmd("M23", "M23 \u{e9}t\u{e9}.gcode"),
        cmd("M23", "M23 \u{c9}T\u{c9}.GCODE"),
        cmd("M23", "M23 sub/deep/inner.gco"),
        cmd("M23", "M23 missing.gcode"),
        Step::Status,
        cmd("M23", "M23"),
        cmd("M23", "M23 lnk.gcode"),
        cmd("M27", "M27"),
        cmd("SDCARD_RESET_FILE", "SDCARD_RESET_FILE"),
        cmd("M27", "M27"),
        Step::Status,
    ]));
}

#[test]
fn sdcard_print_file_selection() {
    let dir = listing_tree("spf");
    parity(Spec::in_dir("sdcard_print_file", &dir).steps(vec![
        cmdp("SDCARD_PRINT_FILE", "SDCARD_PRINT_FILE", &[]),
        cmdp("SDCARD_PRINT_FILE", "SDCARD_PRINT_FILE FILENAME=", &[("FILENAME", "")]),
        cmdp(
            "SDCARD_PRINT_FILE",
            "SDCARD_PRINT_FILE FILENAME=nothing.gcode",
            &[("FILENAME", "nothing.gcode")],
        ),
        Step::Status,
        cmdp(
            "SDCARD_PRINT_FILE",
            "SDCARD_PRINT_FILE FILENAME=/SUB/Deep/INNER.gco",
            &[("FILENAME", "/SUB/Deep/INNER.gco")],
        ),
        Step::Status,
        Step::Stats,
        cmdp("SDCARD_PRINT_FILE", "SDCARD_PRINT_FILE FILENAME=b.gcode", &[("FILENAME", "b.gcode")]),
        cmdp("M23", "M23 b.gcode", &[]),
        cmdp("M26", "M26 S3", &[("S", "3")]),
        cmd("M24", "M24"),
        Step::Resume,
        Step::Drive,
        Step::Status,
        Step::Stats,
        cmdp("SDCARD_PRINT_FILE", "SDCARD_PRINT_FILE FILENAME=/b.gcode", &[("FILENAME", "/b.gcode")]),
        Step::Drive,
        Step::Status,
    ]));
}

// ---------------------------------------------------------------------------
// Tests: printing
// ---------------------------------------------------------------------------

#[test]
fn print_simple_file() {
    let dir = tree(
        "print_simple",
        &[
            ("ok.gcode", b("G28\nG1 X10 Y10\nM117 hi\nG1 X20\n")),
            ("nonl.gcode", b("G1 X1\nG1 X2\nG1 X3")),
            ("empty.gcode", Vec::new()),
            ("blank.gcode", b("\n\n  \nM27\nM21\n")),
        ],
    );
    parity(Spec::in_dir("print_simple", &dir).steps(vec![
        cmd("M23", "M23 ok.gcode"),
        Step::Stats,
        cmd("M24", "M24"),
        Step::Stats,
        cmd("M27", "M27"),
        cmd("M23", "M23 other.gcode"),
        cmdp("M26", "M26 S5", &[("S", "5")]),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Stats,
        cmd("M27", "M27"),
        cmd("M23", "M23 nonl.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmd("M23", "M23 empty.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmd("M23", "M23 blank.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
    ]));
}

#[test]
fn print_without_file_and_seek_errors() {
    let dir = tree("print_nofile", &[("f.gcode", b("G1 X1\nG1 X2\n"))]);
    parity(Spec::in_dir("print_no_file", &dir).steps(vec![
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Stats,
        cmd("M23", "M23 f.gcode"),
        cmdp("M26", "M26 S4", &[("S", "4")]),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmdp("M26", "M26 S999", &[("S", "999")]),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Resume,
        Step::Drive,
        Step::Status,
    ]));
}

#[test]
fn print_multibyte_and_crlf() {
    let dir = tree(
        "print_utf8",
        &[(
            "crlf.gcode",
            b("G1 X1\r\nM117 h\u{e9}llo w\u{f6}rld \u{2713}\r\nG1 X3 ; \u{fc}n\u{ef}\r\nG1 \u{1f600} X4\r\n"),
        )],
    );
    parity(Spec::in_dir("print_utf8", &dir).steps(vec![
        cmd("M23", "M23 crlf.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmd("M23", "M23 crlf.gcode"),
        cmdp("M26", "M26 S6", &[("S", "6")]),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
    ]));
}

#[test]
fn print_command_errors() {
    let dir = tree(
        "print_err",
        &[
            ("fail.gcode", b("G1 X1\nFAIL boom happened\nG1 X3\n")),
            ("raise.gcode", b("G1 X1\nRAISE kaput\nG1 X3\n")),
            ("busy.gcode", b("G1 X1\nM24\nG1 X3\n")),
            ("reset.gcode", b("G1 X1\nSDCARD_RESET_FILE\nG1 X3\n")),
            ("write.gcode", b("M28 f\nG1 X3\n")),
        ],
    );
    let steps = |file: &str| {
        vec![
            cmd("M23", &format!("M23 {file}")),
            cmd("M24", "M24"),
            Step::Drive,
            Step::Status,
            Step::Stats,
        ]
    };
    for (name, file) in [
        ("fail", "fail.gcode"),
        ("raise", "raise.gcode"),
        ("busy", "busy.gcode"),
        ("reset", "reset.gcode"),
        ("write", "write.gcode"),
    ] {
        // Default on_error_gcode without and with a `heaters` object.
        parity(Spec::in_dir(&format!("err_{name}"), &dir).steps(steps(file)));
        parity(Spec::in_dir(&format!("err_{name}_heaters"), &dir).heaters().steps(steps(file)));
    }
    // Custom hooks: a plain template and a failing script. A template that
    // fails to render is not compared: the error class and the logged
    // exception text come from the gcode_macro port, not from this module.
    parity(
        Spec::in_dir("err_hook", &dir)
            .opt("on_error_gcode", "ERRHOOK {1 + 2}\nFAIL in hook\nG1")
            .steps(steps("fail.gcode")),
    );
    parity(
        Spec::in_dir("err_hook_raise", &dir)
            .opt("on_error_gcode", "RAISE inside hook")
            .steps(steps("fail.gcode")),
    );
    parity(
        Spec::in_dir("err_hook_cancel", &dir)
            .opt("on_error_gcode", "CANCEL_PRINT")
            .steps(steps("fail.gcode")),
    );
}

#[test]
fn print_pause_from_the_file_and_resume() {
    let dir = tree(
        "print_pause",
        &[("pause.gcode", b("G1 X1\nM25\nG1 X2\nG1 X3\n"))],
    );
    parity(Spec::in_dir("print_pause", &dir).steps(vec![
        cmd("M23", "M23 pause.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmd("M27", "M27"),
        Step::Stats,
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Drive,
        cmd("M27", "M27"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
    ]));
}

fn trigger_file(trigger: &str) -> Vec<u8> {
    // The trigger is scheduled in the first chunk and runs when the print
    // yields after reading the second one.
    b(&format!("G1 X1\n{trigger}\n{}M117 tail\n", filler(700)))
}

#[test]
fn print_pause_from_another_greenlet() {
    let dir = tree(
        "print_trigger_pause",
        &[("t.gcode", trigger_file("TRIGGER_PAUSE"))],
    );
    parity(Spec::in_dir("trigger_pause", &dir).steps(vec![
        cmd("M23", "M23 t.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Stats,
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
    ]));
}

#[test]
fn print_cancel_from_another_greenlet() {
    let dir = tree(
        "print_trigger_cancel",
        &[("t.gcode", trigger_file("TRIGGER_CANCEL"))],
    );
    parity(Spec::in_dir("trigger_cancel", &dir).steps(vec![
        cmd("M23", "M23 t.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmd("M27", "M27"),
        Step::Stats,
    ]));
}

#[test]
fn print_cancel_from_inside_the_print() {
    let dir = tree(
        "print_self_cancel",
        &[("s.gcode", b("G1 X1\nSELF_CANCEL\nG1 X2\nG1 X3\n"))],
    );
    parity(Spec::in_dir("self_cancel", &dir).steps(vec![
        cmd("M23", "M23 s.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        cmd("M27", "M27"),
        Step::Cancel,
        Step::Status,
    ]));
}

#[test]
fn print_file_position_jumps() {
    // "G1 X1\n" is 6 bytes, "G1 X2\n" 6 bytes.
    let dir = tree(
        "print_jump",
        &[
            ("jump.gcode", b("G1 X1\nG1 X2\nJUMP 6 2\nG1 X3\nG1 X4\n")),
            ("jump_neg.gcode", b("G1 X1\nJUMP -5 1\nG1 X2\n")),
            ("jump_far.gcode", b("G1 X1\nJUMP 99999 1\nG1 X2\n")),
            ("jump_mid.gcode", b("G1 \u{e9}\u{e9}\u{e9}\nJUMP 5 1\nG1 X2\n")),
            ("jump_same.gcode", b("G1 X1\nJUMP 12 1\nG1 X2\nG1 X3\n")),
        ],
    );
    for file in ["jump", "jump_neg", "jump_far", "jump_mid", "jump_same"] {
        parity(Spec::in_dir(&format!("print_{file}"), &dir).steps(vec![
            cmd("M23", &format!("M23 {file}.gcode")),
            cmd("M24", "M24"),
            Step::Drive,
            Step::Status,
            Step::Stats,
        ]));
    }
}

#[test]
fn print_read_errors() {
    let mut late = b(&filler(500));
    late.extend_from_slice(b"G1 \xff\n");
    let dir = tree(
        "print_bad",
        &[
            ("bad_first.gcode", b"G1 X1\nG1 \xffX\n".to_vec()),
            ("bad_cont.gcode", b"G1 X1\nG1 \xe2\x82A\n".to_vec()),
            ("bad_eof.gcode", b"G1 X1\nG1 X\xc3".to_vec()),
            ("bad_surrogate.gcode", b"G1 X1\n\xed\xa0\x80\n".to_vec()),
            ("bad_overlong.gcode", b"G1 X1\n\xc0\x80\n".to_vec()),
            ("bad_late.gcode", late),
        ],
    );
    for file in ["bad_first", "bad_cont", "bad_eof", "bad_surrogate", "bad_overlong", "bad_late"] {
        parity(Spec::in_dir(&format!("read_{file}"), &dir).steps(vec![
            cmd("M23", &format!("M23 {file}.gcode")),
            cmd("M24", "M24"),
            Step::Drive,
            Step::Status,
            Step::Shutdown,
            Step::Stats,
        ]));
    }
}

/// A file crossing several 8192-character reads: multibyte text (the reads
/// adapt their byte size), a `\r` as the last character of the first read and
/// a slow command, ending with a shutdown dump of the chunk rings.
fn big_file() -> Vec<u8> {
    let mut text = String::new();
    // 8191 characters, then "\r\n" so the first 8192-byte read ends in "\r".
    text.push_str("G1 ");
    text.push_str(&"a".repeat(8191 - 3));
    text.push_str("\r\n");
    for i in 0..700 {
        text.push_str(&format!("G1 X{i} ; \u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\n"));
        if i == 300 {
            text.push_str("SLOW\n");
        }
    }
    text.push_str("   SHUTDOWN extra args\n");
    text.push_str("G1 X9999\n");
    b(&text)
}

#[test]
fn print_big_file_chunks_and_shutdown_dump() {
    let dir = tree("print_big", &[("big.gcode", big_file())]);
    parity(Spec::in_dir("print_big", &dir).steps(vec![
        cmd("M23", "M23 big.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Stats,
        Step::Shutdown,
        // Resuming runs to the end of the file.
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
        Step::Shutdown,
    ]));
}

#[test]
fn print_slow_command_ring_entries() {
    let mut text = String::new();
    for i in 0..30 {
        text.push_str("SLOW\n");
        text.push_str(&format!("G1 X{i}\n"));
    }
    text.push_str("FAIL stop here\n");
    let dir = tree("print_slow", &[("slow.gcode", b(&text))]);
    parity(Spec::in_dir("print_slow", &dir).steps(vec![
        cmd("M23", "M23 slow.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Shutdown,
    ]));
}

#[test]
fn print_failed_command_with_long_name() {
    let dir = tree(
        "print_longname",
        &[(
            "long.gcode",
            b("G1 X1\nLONGFAIL_ABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789 x\nG1 X2\n"),
        )],
    );
    parity(Spec::in_dir("print_long_name", &dir).steps(vec![
        cmd("M23", "M23 long.gcode"),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Shutdown,
        Step::Status,
    ]));
}

#[test]
fn print_waits_for_the_gcode_mutex() {
    let dir = tree(
        "print_mutex",
        &[("m.gcode", b(&format!("G1 X1\n{}SHUTDOWN\nG1 X2\n", filler(5))))],
    );
    parity(Spec::in_dir("print_mutex", &dir).steps(vec![
        cmd("M23", "M23 m.gcode"),
        Step::Lock(10),
        cmd("M24", "M24"),
        Step::Drive,
        Step::Status,
    ]));
}

#[test]
fn shutdown_dump_while_idle_and_with_a_file() {
    let dir = tree("shutdown_idle", &[("f.gcode", b("G1 X1\nG1 X2\n"))]);
    parity(Spec::in_dir("shutdown_idle", &dir).steps(vec![
        Step::Shutdown,
        cmd("M23", "M23 f.gcode"),
        Step::Shutdown,
        cmdp("M26", "M26 S3", &[("S", "3")]),
        cmd("M24", "M24"),
        Step::Shutdown,
        Step::Drive,
        Step::Shutdown,
        Step::Stats,
    ]));
}

#[test]
fn busy_errors_and_position_api() {
    let dir = tree("busy", &[("f.gcode", b("G1 X1\nG1 X2\nG1 X3\n"))]);
    parity(Spec::in_dir("busy", &dir).steps(vec![
        cmd("M23", "M23 f.gcode"),
        Step::SetPos(77),
        Step::CmdFromSd,
        cmd("M24", "M24"),
        Step::Stats,
        Step::Resume,
        cmd("M23", "M23 f.gcode"),
        cmdp("M26", "M26 S1", &[("S", "1")]),
        cmd("M24", "M24"),
        cmdp("SDCARD_PRINT_FILE", "SDCARD_PRINT_FILE FILENAME=f.gcode", &[("FILENAME", "f.gcode")]),
        Step::Status,
        Step::Drive,
        Step::Status,
        Step::SetPos(5),
        Step::Resume,
        Step::Drive,
        Step::Status,
    ]));
}

// ---------------------------------------------------------------------------
// Tests: helpers
// ---------------------------------------------------------------------------

#[test]
fn debug_cmd_name_matches_python() {
    let mut guard = SERIAL.lock();
    if guard.is_none() {
        match pyoracle::Oracle::start() {
            Some(mut py) => {
                py.exec(SETUP).expect("virtual_sdcard fakes");
                *guard = Some(py);
            }
            None => {
                println!("skipping virtual_sdcard oracle tests: KLIPPY_PYTHON is not set");
                return;
            }
        }
    }
    let py = guard.as_mut().expect("oracle");
    let long = "X".repeat(40);
    let inputs = [
        "G1 X10 Y10",
        "  M117 hello  ",
        "G28",
        "",
        "   ",
        " leading space",
        "\tTAB_COMMAND arg",
        "SET_PRINT_STATS_INFO TOTAL_LAYER=10",
        "ABCDEFGHIJKLMNOPQRSTUVWX",
        "ABCDEFGHIJKLMNOPQRSTUVWXY",
        "ABCDEFGHIJKLMNOPQRSTUVWXY z",
        "\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9}\u{e9} arg",
        "G1\rX",
        long.as_str(),
    ];
    for input in inputs {
        let expected = py.eval(&format!("vsd_mod.debug_cmd_name({})", q(input)));
        let actual = Value::Str(debug_cmd_name(input));
        let expected = match expected {
            Ok(v) => v,
            Err(e) => panic!("oracle raised {}: {}", e.class_name, e.message),
        };
        assert_eq!(expected, actual, "debug_cmd_name({input:?})");
    }
}

#[test]
fn module_factory_entry_points() {
    let f = virtual_sdcard::module_factory();
    assert_eq!(f.entry_points(), pyoracle::entry_points("extras.virtual_sdcard"), "entry points of extras.virtual_sdcard");
}
