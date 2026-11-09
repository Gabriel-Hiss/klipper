//! Parity tests for `klippy_core::klippy` (port of `klippy/klippy.py`).
//!
//! Every expected value comes from `pyoracle`: the real `klippy.py`
//! runs in CPython 3.8.2 next to the real `util`, `queuelogger` and
//! `msgproto`, with minimal fakes for the modules `klippy.py` imports
//! (`reactor`, `gcode`, `webhooks`, `configfile`, `pins`, `mcu`,
//! `toolhead`) and for two extras modules. The Rust side runs the same
//! scenario through the port and the fakes of the contract traits
//! below, and each step compares one observation: state message and
//! category, shutdown flag, `run_result`, object names, the trace of
//! fake calls and handler calls, and the log records.
//!
//! Log records are normalized the way the frozen normalizer does it
//! (the `Start printer at` time and the numbers of the
//! `[DEBUG-motion] host` snapshot, which are host values); the
//! snapshot lines themselves are compared exactly in
//! `host_state_lines_match_python` against a Python run that has
//! `os.getpid`, `time.time`, `resource`, `/proc/self/status` and the
//! rest patched.

use std::path::PathBuf;
use std::sync::{Arc, LazyLock};

use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use klippy_core::klippy::{
    arg_dictionary, git_version_block, host_state_lines, import_test, import_test_names,
    parse_cli, run_main, start_printer_line, versions_block, HostEnv, HostInfo, MainOptions,
    Printer as RustPrinter, Rusage, MESSAGE_READY, MESSAGE_RESTART, MESSAGE_STARTUP,
    MODULE_CONFIGFILE, MODULE_GCODE, MODULE_MCU, MODULE_PINS, MODULE_TOOLHEAD, MODULE_WEBHOOKS,
    PYTHON_VERSION,
};
use klippy_core::queuelogger::{self, QueueListener};
use klippy_core::util::GitInfo;
use kos::sync::Mutex;
use pycompat::{PyInt, PyValue};
use pyoracle::{Oracle, OracleResult};
use reactor::{Reactor, NOW};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python side.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import builtins
import collections
import configparser
import contextlib
import faulthandler
import gc
import importlib
import io
import logging
import optparse
import os
import resource
import sys
import threading
import time
import types

import queuelogger
import util

os.environ["TZ"] = "NZST-12NZDT,M9.5.0,M4.1.0/3"
time.tzset()
os.environ.pop("COLUMNS", None)

TREE = "@TREE@"
ROOT = "@ROOT@"
TRACE = []
LOG_LINES = []
MODE = {}
SECTIONS = []
START_ARGS = []
RESULTS = []
IMPORTS = []
IMPORT_MISSING = set()

class _Capture(logging.Handler):
    def emit(self, record):
        LOG_LINES.append((record.levelname, record.getMessage()))

logging.getLogger().addHandler(_Capture())
logging.getLogger().setLevel(logging.INFO)

def reset():
    del TRACE[:]
    del LOG_LINES[:]
    MODE.clear()
    del SECTIONS[:]
    del START_ARGS[:]
    del RESULTS[:]
    del IMPORTS[:]
    IMPORT_MISSING.clear()

class Obj:
    def __init__(self, name):
        self.name = name

class _NoPause:
    def __enter__(self):
        return self
    def __exit__(self, *args):
        return None

class FakeReactor:
    def __init__(self, gc_checking=False):
        self.callbacks = []
        self.mono = 100.0
        self.gc_stats = (0.0, 0.0, 0.0)
    def monotonic(self):
        return self.mono
    def get_gc_stats(self):
        return self.gc_stats
    def register_callback(self, cb, waketime=0.):
        self.callbacks.append(cb)
    def register_async_callback(self, cb, waketime=0.):
        self.callbacks.append(cb)
    def assert_no_pause(self):
        return _NoPause()
    def end(self):
        pass
    def run(self):
        cbs, self.callbacks = self.callbacks, []
        for cb in cbs:
            cb(0.0)
    def finalize(self):
        pass

reactor_mod = types.ModuleType("reactor")
reactor_mod.Reactor = FakeReactor
sys.modules["reactor"] = reactor_mod

pins_error = type("error", (Exception,), {})
mcu_error = type("error", (Exception,), {})
class CommandError(Exception):
    pass

def raise_mode(key):
    kind = MODE.get(key)
    if kind is None:
        return
    msg = "boom " + key
    if kind == "config":
        raise configparser.Error(msg)
    if kind == "pins":
        raise pins_error(msg)
    if kind == "mcu":
        raise mcu_error(msg)
    if kind == "protocol":
        import msgproto
        raise msgproto.error(msg)
    if kind == "foreign":
        raise type("error", (Exception,), {})(msg)
    if kind == "runtime":
        raise RuntimeError(msg)
    if kind == "command":
        raise CommandError(msg)
    raise ValueError(kind)

def _early_gcode(printer):
    TRACE.append("early:gcode")
    printer.add_object("gcode", Obj("gcode"))
    printer.add_object("gcode_io", Obj("gcode_io"))
gcode_mod = types.ModuleType("gcode")
gcode_mod.CommandError = CommandError
gcode_mod.add_early_printer_objects = _early_gcode
sys.modules["gcode"] = gcode_mod

def _early_webhooks(printer):
    TRACE.append("early:webhooks")
    printer.add_object("webhooks", Obj("webhooks"))
webhooks_mod = types.ModuleType("webhooks")
webhooks_mod.add_early_printer_objects = _early_webhooks
sys.modules["webhooks"] = webhooks_mod

def _pins_add(config):
    TRACE.append("pins")
    raise_mode("pins")
    config.get_printer().add_object("pins", Obj("pins"))
pins_mod = types.ModuleType("pins")
pins_mod.error = pins_error
pins_mod.add_printer_objects = _pins_add
sys.modules["pins"] = pins_mod

def _mcu_add(config):
    TRACE.append("mcu")
    raise_mode("mcu")
    config.get_printer().add_object("mcu", Obj("mcu"))
mcu_mod = types.ModuleType("mcu")
mcu_mod.error = mcu_error
mcu_mod.add_printer_objects = _mcu_add
sys.modules["mcu"] = mcu_mod

def _toolhead_add(config):
    TRACE.append("toolhead")
    raise_mode("toolhead")
    config.get_printer().add_object("toolhead", Obj("toolhead"))
toolhead_mod = types.ModuleType("toolhead")
toolhead_mod.add_printer_objects = _toolhead_add
sys.modules["toolhead"] = toolhead_mod

class FakeConfigWrapper:
    def __init__(self, printer, name):
        self._printer = printer
        self._name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def getsection(self, section):
        return FakeConfigWrapper(self._printer, section)
    def get_prefix_sections(self, prefix):
        return [self.getsection(s) for s in SECTIONS if s.startswith(prefix)]

class FakePrinterConfig:
    def __init__(self, printer):
        TRACE.append("configfile_new")
        self.printer = printer
    def read_main_config(self):
        TRACE.append("read_main_config")
        raise_mode("read_main_config")
        return FakeConfigWrapper(self.printer, "printer")
    def log_config(self, config):
        TRACE.append("log_config")
    def check_unused_options(self, config):
        TRACE.append("check_unused_options")
        raise_mode("check_unused_options")
configfile_mod = types.ModuleType("configfile")
configfile_mod.error = configparser.Error
configfile_mod.sentinel = object()
configfile_mod.PrinterConfig = FakePrinterConfig
sys.modules["configfile"] = configfile_mod

def _canbus_load(config):
    TRACE.append("load_config:canbus_ids:" + config.get_name())
    raise_mode("load_canbus")
    printer = config.get_printer()
    START_ARGS.append(dict(printer.get_start_args()))
    if MODE.get("hooks") == "main":
        printer.register_event_handler("klippy:ready", lambda: printer.request_exit(RESULTS.pop(0)))
        printer.register_event_handler("klippy:firmware_restart", lambda: TRACE.append("firmware_restart"))
        printer.register_event_handler("klippy:disconnect", lambda: TRACE.append("disconnect"))
    return Obj("canbus")
def _canbus_prefix(config):
    TRACE.append("load_config_prefix:canbus_ids:" + config.get_name())
    return Obj("canbus_prefix")
_canbus = types.ModuleType("extras.canbus_ids")
_canbus.load_config = _canbus_load
_canbus.load_config_prefix = _canbus_prefix
sys.modules["extras.canbus_ids"] = _canbus

def _ad_load(config):
    TRACE.append("load_config:ad5206:" + config.get_name())
    return None
_ad = types.ModuleType("extras.ad5206")
_ad.load_config = _ad_load
sys.modules["extras.ad5206"] = _ad

klippy_mod = importlib.import_module("klippy")
REAL_DIR = os.path.dirname(os.path.abspath(klippy_mod.__file__))

def use_tree(path):
    klippy_mod.__file__ = path + "/klippy.py"
    util.__file__ = path + "/util.py"
use_tree(TREE)

START = {"config_file": "printer.cfg", "apiserver": None, "start_reason": "startup"}

def new_printer(args=None, bg=None):
    return klippy_mod.Printer(FakeReactor(), bg, dict(START if args is None else args))

def rec(tag, ret=None, boom=None):
    def handler(*args):
        TRACE.append(tag + ":" + ",".join(str(a) for a in args))
        if boom is not None:
            raise boom
        return ret
    return handler

def observe(p):
    msg, cat = p.get_state_message()
    return {
        "state": (msg, cat),
        "shutdown": p.is_shutdown(),
        "run_result": p.run_result,
        "dumped": p.debug_host_dumped,
        "objects": [n for n, o in p.objects.items() if o is not None],
        "start_args": p.get_start_args(),
        "seen_args": list(START_ARGS),
        "trace": list(TRACE),
        "log": list(LOG_LINES),
    }

def norm(r):
    if r is None or isinstance(r, (bool, int, float, str)):
        return r
    if isinstance(r, (list, tuple)):
        return type(r)(norm(x) for x in r)
    if isinstance(r, dict):
        return dict((k, norm(v)) for k, v in r.items())
    return "<object>"

def call_obs(p, fn):
    try:
        r = fn()
    except Exception as e:
        return ("raised", type(e).__name__, str(e), observe(p))
    return ("returned", norm(r), observe(p))

class _FdStringIO(io.StringIO):
    def fileno(self):
        return sys.__stderr__.fileno()

def _fake_import(name, package=None):
    IMPORTS.append(name)
    if name in IMPORT_MISSING:
        raise ModuleNotFoundError("No module named %r" % (name,), name=name)
    mod = sys.modules.get(name)
    if mod is None:
        mod = types.ModuleType(name)
    return mod

def run_main_py(argv, git=None, cpu=None):
    saved = (sys.argv, sys.version, util.get_git_version, util.get_cpu_info,
             util.get_device_info, util.get_linux_version, time.sleep,
             importlib.import_module)
    out = io.StringIO()
    err = _FdStringIO()
    sys.argv = list(argv)
    sys.version = "3.8.2"
    if git is not None:
        util.get_git_version = lambda from_file=True: git
    if cpu is not None:
        util.get_cpu_info = lambda: cpu[0]
        util.get_device_info = lambda: cpu[1]
        util.get_linux_version = lambda: cpu[2]
    time.sleep = lambda secs: None
    importlib.import_module = _fake_import
    code = 0
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                klippy_mod.main()
            except SystemExit as e:
                code = e.code
            except Exception as e:
                code = 1
                err.write("%s: %s\n" % (type(e).__name__, e))
    finally:
        (sys.argv, sys.version, util.get_git_version, util.get_cpu_info,
         util.get_device_info, util.get_linux_version, time.sleep,
         importlib.import_module) = saved
    return {
        "exit": (code, out.getvalue(), err.getvalue()),
        "start_args": list(START_ARGS),
        "trace": list(TRACE),
        "log": list(LOG_LINES),
    }

class _Stop(BaseException):
    pass

_PARSER = []

def get_parser():
    if _PARSER:
        return _PARSER[0]
    orig = optparse.OptionParser.parse_args
    def fake(self, args=None, values=None):
        _PARSER.append(self)
        raise _Stop()
    optparse.OptionParser.parse_args = fake
    saved = sys.argv
    sys.argv = ["klippy.py"]
    try:
        klippy_mod.main()
    except _Stop:
        pass
    finally:
        optparse.OptionParser.parse_args = orig
        sys.argv = saved
    return _PARSER[0]

def parse_py(argv):
    parser = get_parser()
    saved = sys.argv
    sys.argv = list(argv)
    out = io.StringIO()
    err = io.StringIO()
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                options, args = parser.parse_args(list(argv[1:]))
            except SystemExit as e:
                return ("exit", e.code, out.getvalue(), err.getvalue())
    finally:
        sys.argv = saved
    return ("ok", {
        "apiserver": options.apiserver,
        "debuginput": options.debuginput,
        "debugoutput": options.debugoutput,
        "dictionary": options.dictionary,
        "import_test": bool(options.import_test),
        "inputtty": options.inputtty,
        "logfile": options.logfile,
        "verbose": bool(options.verbose),
    }, list(args))

def host_lines_py(d):
    saved = (os.getpid, time.time, gc.get_count, threading.active_count,
             resource.getrusage, resource.getrlimit, builtins.open,
             sys.version, sys.platform)
    p = new_printer()
    p.reactor.mono = d["mono"]
    p.reactor.gc_stats = tuple(d["gc_times"])
    p.in_shutdown_state = d["shutdown"]
    p.run_result = d["run_result"]
    del LOG_LINES[:]
    real_open = builtins.open
    def fake_open(path, *args, **kwargs):
        if path == "/proc/self/status":
            if d["status"] is None:
                raise OSError("no status")
            return io.StringIO(d["status"])
        return real_open(path, *args, **kwargs)
    def fake_getrusage(who):
        if d["rusage"] is None:
            raise OSError("no rusage")
        r = d["rusage"]
        return types.SimpleNamespace(
            ru_utime=r["user"], ru_stime=r["system"], ru_maxrss=r["maxrss"],
            ru_minflt=r["minflt"], ru_majflt=r["majflt"], ru_nvcsw=r["nvcsw"],
            ru_nivcsw=r["nivcsw"])
    os.getpid = lambda: d["pid"]
    time.time = lambda: d["wall"]
    gc.get_count = lambda: tuple(d["gc_count"])
    threading.active_count = lambda: d["threads"]
    resource.getrusage = fake_getrusage
    resource.getrlimit = lambda which: tuple(d["core"] if which == resource.RLIMIT_CORE else d["nofile"])
    builtins.open = fake_open
    sys.version = d["version"]
    sys.platform = d["platform"]
    try:
        p._debug_host_state()
    finally:
        (os.getpid, time.time, gc.get_count, threading.active_count,
         resource.getrusage, resource.getrlimit, builtins.open,
         sys.version, sys.platform) = saved
    return [m for (lvl, m) in LOG_LINES]

def start_line_py(systime, mono):
    saved = time.time
    p = new_printer()
    p.reactor.callbacks = []
    p.reactor.mono = mono
    del LOG_LINES[:]
    time.time = lambda: systime
    try:
        p.run()
    finally:
        time.time = saved
    return LOG_LINES[0]

def import_run_py(missing):
    reset()
    IMPORT_MISSING.update(missing)
    saved = importlib.import_module
    importlib.import_module = _fake_import
    try:
        try:
            klippy_mod.import_test()
        except SystemExit as e:
            return (list(IMPORTS), "exit", e.code)
        except Exception as e:
            return (list(IMPORTS), type(e).__name__, str(e))
    finally:
        importlib.import_module = saved
"#;

// ---------------------------------------------------------------------------
// Rust side: shared state.
// ---------------------------------------------------------------------------

static SERIAL: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));
static TRACE: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));
// Reactor greenlets log from OS threads. SERIAL protects each test's
// capture window; the shared buffer includes every greenlet's records.
static LOGS: LazyLock<Mutex<Vec<(String, String)>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static RUN_EXCEPTIONS: LazyLock<Mutex<Vec<(String, String)>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static MODE: LazyLock<Mutex<IndexMap<String, String>>> =
    LazyLock::new(|| Mutex::new(IndexMap::new()));
static SECTIONS: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static SEEN_ARGS: LazyLock<Mutex<Vec<PyValue>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static RESULTS: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));

/// Temporary source tree: `<root>/klippy` holds `extras/`, `kinematics/`
/// and a `.version` file; `<root>` holds scratch files.
static ROOT: LazyLock<PathBuf> = LazyLock::new(|| {
    let root = std::env::temp_dir().join(format!("klippy-core-klippy-test-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    let klippy = root.join("klippy");
    for dir in ["extras/pkg", "extras/nopkg", "kinematics"] {
        std::fs::create_dir_all(klippy.join(dir)).expect("create tree");
    }
    for file in [
        "extras/__init__.py",
        "extras/canbus_ids.py",
        "extras/ad5206.py",
        "extras/pkg/__init__.py",
        "extras/junk.txt",
        "extras/nopkg/helper.py",
        "kinematics/__init__.py",
        "kinematics/cartesian.py",
        "kinematics/notes",
        "../dbg.gcode",
    ] {
        std::fs::write(klippy.join(file), b"").expect("write tree file");
    }
    std::fs::write(klippy.join(".version"), b"v-test-1\n").expect("write .version");
    root
});

fn tree() -> String {
    ROOT.join("klippy").to_string_lossy().into_owned()
}

fn root_str() -> String {
    ROOT.to_string_lossy().into_owned()
}

fn scratch(name: &str) -> String {
    ROOT.join(name).to_string_lossy().into_owned()
}

static LOGGING: LazyLock<()> = LazyLock::new(|| {
    // Both sides run in the same non-UTC zone; `Start printer at` and
    // the build-file stamps use local time.
    std::env::set_var("TZ", "NZST-12NZDT,M9.5.0,M4.1.0/3");
    kos::time::tzset();
    install_log_sink();
});

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        let _installed = log::set_sink(Box::new(|rec: &log::Record<'_>| {
            if rec.level >= queuelogger::root_level() {
                LOGS.lock().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
                if matches!(rec.message, "Unhandled exception during run" | "Repeat unhandled exception during run") {
                    if let Some(text) = rec.exc_text {
                        RUN_EXCEPTIONS.lock().push((rec.message.to_string(), text.to_string()));
                    }
                }
            }
        }));
    });
}

fn begin() -> kos::sync::MutexGuard<'static, ()> {
    let guard = SERIAL.lock();
    LazyLock::force(&LOGGING);
    queuelogger::root_set_level(log::Level::Info);
    // `run_main` records it; tests that build the printer directly too.
    klippy_api::paths::set_klippy_dir(&tree());
    clear_state();
    guard
}

fn clear_state() {
    TRACE.lock().clear();
    LOGS.lock().clear();
    RUN_EXCEPTIONS.lock().clear();
    MODE.lock().clear();
    SECTIONS.lock().clear();
    SEEN_ARGS.lock().clear();
    RESULTS.lock().clear();
}

fn trace(entry: impl Into<String>) {
    TRACE.lock().push(entry.into());
}

fn raise_mode(key: &str) -> KResult<()> {
    let kind = MODE.lock().get(key).cloned();
    let Some(kind) = kind else {
        return Ok(());
    };
    let msg = format!("boom {key}");
    Err(match kind.as_str() {
        "config" => KlippyError::config(msg),
        "pins" => KlippyError::Pins(msg),
        "mcu" => KlippyError::Mcu(msg),
        "protocol" => KlippyError::Protocol(msg),
        "foreign" => KlippyError::Other { class_name: "error".to_string(), message: msg },
        "runtime" => KlippyError::Internal { class_name: "RuntimeError".to_string(), message: msg },
        "command" => KlippyError::command(msg),
        other => panic!("unknown mode {other}"),
    })
}

// ---------------------------------------------------------------------------
// Rust side: fakes of the contract traits.
// ---------------------------------------------------------------------------

struct MarkerObj;
klippy_api::printer_object!(MarkerObj);

fn marker() -> ObjectRef {
    Arc::new(MarkerObj)
}

struct FakeConfig {
    printer: klippy_api::printer::Printer,
    name: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> klippy_api::printer::Printer {
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
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        SECTIONS.lock().iter().any(|s| s == section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        let names: Vec<String> = SECTIONS
            .lock()
            .iter()
            .filter(|s| s.starts_with(prefix))
            .cloned()
            .collect();
        names.iter().map(|s| self.getsection(s)).collect()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeConfigFile {
    printer: klippy_api::printer::Printer,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> klippy_api::printer::Printer {
        self.printer.clone()
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config is not used by klippy.py")
    }
    fn read_main_config(&self) -> KResult<Config> {
        trace("read_main_config");
        raise_mode("read_main_config")?;
        Ok(Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: "printer".to_string(),
        }))
    }
    fn log_config(&self, _config: &Config) {
        trace("log_config");
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        trace("check_unused_options");
        raise_mode("check_unused_options")
    }
    fn runtime_warning(&self, _msg: &str) {
        panic!("runtime_warning is not used by klippy.py")
    }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("deprecate is not used by klippy.py")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {
        panic!("deprecate_gcode is not used by klippy.py")
    }
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {
        panic!("deprecate_mcu_code is not used by klippy.py")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by klippy.py")
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> {
        panic!("set is not used by klippy.py")
    }
    fn remove_section(&self, _section: &str) {
        panic!("remove_section is not used by klippy.py")
    }
}
klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

fn early_gcode(printer: &klippy_api::printer::Printer) -> KResult<()> {
    trace("early:gcode");
    printer.add_object("gcode", marker())?;
    printer.add_object("gcode_io", marker())?;
    Ok(())
}

fn early_webhooks(printer: &klippy_api::printer::Printer) -> KResult<()> {
    trace("early:webhooks");
    printer.add_object("webhooks", marker())?;
    Ok(())
}

fn create_configfile(printer: &klippy_api::printer::Printer) -> KResult<()> {
    trace("configfile_new");
    let obj: ObjectRef = Arc::new(FakeConfigFile { printer: printer.clone() });
    printer.add_object("configfile", obj)?;
    Ok(())
}

fn pins_add(config: &Config) -> KResult<()> {
    trace("pins");
    raise_mode("pins")?;
    config.get_printer().add_object("pins", marker())
}

fn mcu_add(config: &Config) -> KResult<()> {
    trace("mcu");
    raise_mode("mcu")?;
    config.get_printer().add_object("mcu", marker())
}

fn toolhead_add(config: &Config) -> KResult<()> {
    trace("toolhead");
    raise_mode("toolhead")?;
    config.get_printer().add_object("toolhead", marker())
}

fn load_canbus(config: &Config) -> KResult<Option<ObjectRef>> {
    trace(format!("load_config:canbus_ids:{}", config.get_name()));
    raise_mode("load_canbus")?;
    let printer = config.get_printer();
    SEEN_ARGS.lock().push(printer.get_start_args());
    let hooks_main = MODE.lock().get("hooks").map(String::as_str) == Some("main");
    if hooks_main {
        let p = printer.clone();
        printer.register_event_handler(
            "klippy:ready",
            Box::new(move |_params: &[EventArg]| {
                let result = RESULTS.lock().remove(0);
                p.request_exit(&result);
                Ok(PyValue::None)
            }),
        );
        printer.register_event_handler(
            "klippy:firmware_restart",
            Box::new(|_params: &[EventArg]| {
                trace("firmware_restart");
                Ok(PyValue::None)
            }),
        );
        printer.register_event_handler(
            "klippy:disconnect",
            Box::new(|_params: &[EventArg]| {
                trace("disconnect");
                Ok(PyValue::None)
            }),
        );
    }
    Ok(Some(marker()))
}

fn load_canbus_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    trace(format!("load_config_prefix:canbus_ids:{}", config.get_name()));
    Ok(Some(marker()))
}

fn load_ad5206(config: &Config) -> KResult<Option<ObjectRef>> {
    trace(format!("load_config:ad5206:{}", config.get_name()));
    Ok(None)
}

fn registry() -> IndexMap<String, ModuleFactory> {
    let none = ModuleFactory::default();
    IndexMap::from([
        (
            MODULE_GCODE.to_string(),
            ModuleFactory { add_early_printer_objects: Some(early_gcode), ..none },
        ),
        (
            MODULE_WEBHOOKS.to_string(),
            ModuleFactory { add_early_printer_objects: Some(early_webhooks), ..none },
        ),
        (
            MODULE_CONFIGFILE.to_string(),
            ModuleFactory { add_early_printer_objects: Some(create_configfile), ..none },
        ),
        (
            MODULE_PINS.to_string(),
            ModuleFactory { add_printer_objects: Some(pins_add), ..none },
        ),
        (
            MODULE_MCU.to_string(),
            ModuleFactory { add_printer_objects: Some(mcu_add), ..none },
        ),
        (
            MODULE_TOOLHEAD.to_string(),
            ModuleFactory { add_printer_objects: Some(toolhead_add), ..none },
        ),
        (
            "extras.canbus_ids".to_string(),
            ModuleFactory {
                load_config: Some(load_canbus),
                load_config_prefix: Some(load_canbus_prefix),
                ..none
            },
        ),
        (
            "extras.ad5206".to_string(),
            ModuleFactory { load_config: Some(load_ad5206), ..none },
        ),
    ])
}

fn env() -> Arc<HostEnv> {
    Arc::new(HostEnv { modules: registry(), klippy_dir: tree() })
}

fn rec(tag: &str, ret: PyValue, boom: Option<KlippyError>) -> EventCallback {
    let tag = tag.to_string();
    Box::new(move |params: &[EventArg]| {
        let args: Vec<String> = params
            .iter()
            .map(|a| match a {
                EventArg::Value(v) => v.str(),
                _ => "<object>".to_string(),
            })
            .collect();
        trace(format!("{tag}:{}", args.join(",")));
        match &boom {
            Some(e) => Err(e.clone()),
            None => Ok(ret.clone()),
        }
    })
}

fn rec_plain(tag: &str) -> EventCallback {
    rec(tag, PyValue::None, None)
}

fn runtime_error(msg: &str) -> KlippyError {
    KlippyError::Internal { class_name: "RuntimeError".to_string(), message: msg.to_string() }
}

fn pystr(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn pyint(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn start_args(debuginput: bool) -> IndexMap<String, PyValue> {
    let mut args = IndexMap::from([
        ("config_file".to_string(), pystr("printer.cfg")),
        ("apiserver".to_string(), PyValue::None),
        ("start_reason".to_string(), pystr("startup")),
    ]);
    if debuginput {
        args.insert("debuginput".to_string(), pystr("dbg"));
    }
    args
}

fn new_rs(debuginput: bool) -> Arc<RustPrinter> {
    RustPrinter::new(Reactor::new(false), None, start_args(debuginput), env()).expect("construct printer")
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<Oracle> {
    match Oracle::start() {
        Some(mut py) => {
            let setup = SETUP
                .replace("@TREE@", &tree())
                .replace("@ROOT@", &root_str());
            py.exec(&setup).expect("klippy oracle setup");
            Some(py)
        }
        None => {
            println!("skipping klippy oracle tests: KLIPPY_PYTHON is not set");
            None
        }
    }
}

fn run(py: &mut Oracle, code: &str) {
    if let Err(e) = py.exec(code) {
        panic!("python failed: {}: {}\n{code}", e.class_name, e.message);
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "klippy".to_string(),
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

fn check(label: &str, expected: OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn norm_message(message: &str) -> String {
    if message.starts_with("Start printer at ") {
        return "Start printer at <T>".to_string();
    }
    if let Some(rest) = message.strip_prefix("[DEBUG-motion] host ") {
        let cut = rest.find(['=', ':']).unwrap_or(rest.len());
        return format!("[DEBUG-motion] host {}", &rest[..cut]);
    }
    message.to_string()
}

fn norm_log(value: Value) -> Value {
    let Value::List(items) = value else {
        return value;
    };
    // Debug records carry host-specific text (tracebacks of util's failed
    // reads and git runs), so they are not compared.
    Value::List(
        items
            .into_iter()
            .filter(|item| !matches!(item, Value::Tuple(parts) if parts.first() == Some(&Value::str("DEBUG"))))
            .map(|item| match item {
                Value::Tuple(parts) if parts.len() == 2 => match &parts[1] {
                    Value::Str(m) => Value::Tuple(vec![parts[0].clone(), Value::str(&norm_message(m))]),
                    _ => Value::Tuple(parts),
                },
                other => other,
            })
            .collect(),
    )
}

/// Applies the normalizer rules to a value tree: `log` lists and the
/// descriptor number in `gcode_fd`.
fn norm_value(value: Value) -> Value {
    match value {
        Value::Dict(pairs) => Value::Dict(
            pairs
                .into_iter()
                .map(|(k, v)| {
                    let v = match &k {
                        Value::Str(name) if name == "log" => norm_log(v),
                        Value::Str(name) if name == "gcode_fd" => Value::str("<fd>"),
                        _ => norm_value(v),
                    };
                    (k, v)
                })
                .collect(),
        ),
        Value::List(items) => Value::List(items.into_iter().map(norm_value).collect()),
        Value::Tuple(items) => Value::Tuple(items.into_iter().map(norm_value).collect()),
        other => other,
    }
}

fn check_obs(label: &str, expected: OracleResult, actual: Value) {
    check(label, expected.map(norm_value), norm_value(actual));
}

fn recorder_of(value: &PyValue) -> Value {
    match value {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().expect("small int")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(recorder_of).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(recorder_of).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (recorder_of(k), recorder_of(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host{h}")),
    }
}

fn strs(items: &[String]) -> Value {
    Value::List(items.iter().map(|s| Value::str(s)).collect())
}

fn opt_string(o: Option<String>) -> Value {
    match o {
        Some(s) => Value::str(&s),
        None => Value::Null,
    }
}

fn logs_value() -> Value {
    Value::List(
        LOGS.lock().iter()
            .map(|(level, message)| Value::Tuple(vec![Value::str(level), Value::str(message)]))
            .collect(),
    )
}

fn observe(rs: &RustPrinter) -> Value {
    let (message, category) = rs.get_state_message();
    let names: Vec<String> = rs.lookup_objects(None).into_iter().map(|(n, _)| n).collect();
    let seen: Vec<Value> = SEEN_ARGS.lock().iter().map(recorder_of).collect();
    let trace_entries = TRACE.lock().clone();
    Value::Dict(vec![
        (
            Value::str("state"),
            Value::Tuple(vec![Value::str(&message), Value::str(&category)]),
        ),
        (Value::str("shutdown"), Value::Bool(rs.is_shutdown())),
        (Value::str("run_result"), opt_string(rs.run_result())),
        (Value::str("dumped"), Value::Bool(rs.debug_host_dumped())),
        (Value::str("objects"), strs(&names)),
        (Value::str("start_args"), recorder_of(&rs.get_start_args())),
        (Value::str("seen_args"), Value::List(seen)),
        (Value::str("trace"), strs(&trace_entries)),
        (Value::str("log"), logs_value()),
    ])
}

fn object_value(obj: Option<ObjectRef>) -> Value {
    match obj {
        Some(_) => Value::str("<object>"),
        None => Value::Null,
    }
}

/// One comparison step: the Python expression `expr` (using `p`) against
/// `result` of the same call on `rs`, each followed by an observation.
fn step(py: &mut Oracle, label: &str, expr: &str, rs: &RustPrinter, result: KResult<Value>) {
    let actual = match result {
        Ok(v) => Value::Tuple(vec![Value::str("returned"), v, observe(rs)]),
        Err(e) => Value::Tuple(vec![
            Value::str("raised"),
            Value::str(e.class_name()),
            Value::str(e.message()),
            observe(rs),
        ]),
    };
    check_obs(label, py.eval(&format!("call_obs(p, lambda: {expr})")), actual);
}

fn py_list(items: &[&str]) -> String {
    let parts: Vec<String> = items.iter().map(|s| format!("{s:?}")).collect();
    format!("[{}]", parts.join(", "))
}

/// Fresh printer pair: `p` in the oracle and the returned Rust printer,
/// with the same fake modes and config sections.
fn fresh(py: &mut Oracle, modes: &[(&str, &str)], sections: &[&str], debuginput: bool) -> Arc<RustPrinter> {
    let mut code = String::from("reset()\n");
    for (key, kind) in modes {
        code += &format!("MODE[{key:?}] = {kind:?}\n");
    }
    code += &format!("SECTIONS[:] = {}\n", py_list(sections));
    code += if debuginput {
        "p = new_printer(dict(START, debuginput='dbg'))\n"
    } else {
        "p = new_printer()\n"
    };
    run(py, &code);
    clear_state();
    for (key, kind) in modes {
        MODE.lock().insert(key.to_string(), kind.to_string());
    }
    *SECTIONS.lock() = sections.iter().map(|s| s.to_string()).collect();
    new_rs(debuginput)
}

fn unit(r: KResult<()>) -> KResult<Value> {
    r.map(|()| Value::Null)
}

// ---------------------------------------------------------------------------
// Tests: constants, objects, events.
// ---------------------------------------------------------------------------

#[test]
fn message_constants_match_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    check("message_ready", py.eval("klippy_mod.message_ready"), Value::str(MESSAGE_READY));
    check("message_startup", py.eval("klippy_mod.message_startup"), Value::str(MESSAGE_STARTUP));
    check("message_restart", py.eval("klippy_mod.message_restart"), Value::str(MESSAGE_RESTART));
}

#[test]
fn init_state_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    check_obs("fresh printer", py.eval("observe(p)"), observe(&rs));
    check(
        "category after startup",
        py.eval("p.get_state_message()"),
        {
            let (m, c) = rs.get_state_message();
            Value::Tuple(vec![Value::str(&m), Value::str(&c)])
        },
    );
    let ready = fresh(&mut py, &[], &[], false);
    run(&mut py, "p._set_state(klippy_mod.message_ready)");
    ready.set_state(MESSAGE_READY);
    check_obs("ready", py.eval("observe(p)"), observe(&ready));
}

#[test]
fn objects_match_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    let names = ["extra", "gcode_move", "gcode_move extra", "gcode_move_other", "gcode_move  two"];
    for name in names {
        step(
            &mut py,
            &format!("add {name}"),
            &format!("p.add_object({name:?}, Obj({name:?}))"),
            &rs,
            rs.add_object(name, marker()).map(|()| Value::Null),
        );
    }
    step(
        &mut py,
        "duplicate add",
        "p.add_object('extra', Obj('again'))",
        &rs,
        unit(rs.add_object("extra", marker())),
    );
    step(
        &mut py,
        "duplicate early object",
        "p.add_object('gcode', Obj('again'))",
        &rs,
        unit(rs.add_object("gcode", marker())),
    );
    step(
        &mut py,
        "lookup found",
        "p.lookup_object('extra')",
        &rs,
        rs.lookup_object("extra", LookupDefault::Raise).map(object_value),
    );
    step(
        &mut py,
        "lookup missing",
        "p.lookup_object('nope')",
        &rs,
        rs.lookup_object("nope", LookupDefault::Raise).map(object_value),
    );
    step(
        &mut py,
        "lookup missing with default None",
        "p.lookup_object('nope', None)",
        &rs,
        rs.lookup_object("nope", LookupDefault::ReturnNone).map(object_value),
    );
    for module in ["gcode_move", "gcode", "heaters", "extra", "gcode_move_other"] {
        let found: Vec<Value> = rs
            .lookup_objects(Some(module))
            .into_iter()
            .map(|(n, _)| Value::str(&n))
            .collect();
        check(
            &format!("lookup_objects({module})"),
            py.eval(&format!("[n for n, o in p.lookup_objects({module:?})]")),
            Value::List(found),
        );
    }
    let all: Vec<Value> = rs
        .lookup_objects(None)
        .into_iter()
        .map(|(n, _)| Value::str(&n))
        .collect();
    check(
        "lookup_objects()",
        py.eval("[n for n, o in p.lookup_objects()]"),
        Value::List(all),
    );
}

#[test]
fn load_object_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    run(&mut py, "cfg = FakeConfigWrapper(p, 'printer')");
    let cfg: Config = Arc::new(FakeConfig { printer: rs.clone(), name: "printer".to_string() });
    let sections = [
        "canbus_ids",
        "canbus_ids",
        "canbus_ids foo",
        "canbus_ids  foo bar",
        "canbus_ids\tbaz",
        "ad5206",
        "ad5206",
        "ad5206 prefix",
        "nosuchmod",
        "nosuchmod x",
        "",
        "   ",
    ];
    for section in sections {
        for (label, default_py, default_rs) in [
            ("raise", "", LookupDefault::Raise),
            ("none", ", None", LookupDefault::ReturnNone),
        ] {
            step(
                &mut py,
                &format!("load_object({section:?}) {label}"),
                &format!("p.load_object(cfg, {section:?}{default_py})"),
                &rs,
                rs.load_object(&cfg, section, default_rs).map(object_value),
            );
        }
    }
}

#[test]
fn read_config_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let all = ["canbus_ids", "printer", "ad5206", "canbus_ids foo"];
    // Clean run, without and with a background logger.
    let rs = fresh(&mut py, &[], &all, false);
    step(&mut py, "read_config", "p._read_config()", &rs, unit(rs.read_config()));
    run(&mut py, "reset()\nSECTIONS[:] = ['canbus_ids']\nbg = queuelogger.QueueListener(ROOT + '/bg-py.log')\np = new_printer(None, bg)");
    clear_state();
    *SECTIONS.lock() = vec!["canbus_ids".to_string()];
    let bg = Arc::new(QueueListener::new(&scratch("bg-rs.log")).expect("queue listener"));
    let rs = RustPrinter::new(Reactor::new(false), Some(bg.clone()), start_args(false), env()).expect("construct printer");
    step(&mut py, "read_config with logger", "p._read_config()", &rs, unit(rs.read_config()));
    run(&mut py, "bg.stop()");
    bg.stop();
    // Every stage that can fail.
    for (key, kind) in [
        ("read_main_config", "config"),
        ("pins", "pins"),
        ("mcu", "mcu"),
        ("load_canbus", "runtime"),
        ("load_canbus", "command"),
        ("toolhead", "config"),
        ("check_unused_options", "config"),
        ("check_unused_options", "runtime"),
    ] {
        let rs = fresh(&mut py, &[(key, kind)], &all, false);
        step(
            &mut py,
            &format!("read_config fails at {key} with {kind}"),
            "p._read_config()",
            &rs,
            unit(rs.read_config()),
        );
    }
    // A section named like an early object is returned as it is.
    let rs = fresh(&mut py, &[], &["gcode", "webhooks", "pins"], false);
    step(&mut py, "early objects as sections", "p._read_config()", &rs, unit(rs.read_config()));
}

#[test]
fn events_match_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    run(
        &mut py,
        "p.register_event_handler('ev', rec('h1', 'r1'))\n\
         p.register_event_handler('ev', rec('h2', 7))\n\
         p.register_event_handler('ev', rec('h3', None))",
    );
    rs.register_event_handler("ev", rec("h1", pystr("r1"), None));
    rs.register_event_handler("ev", rec("h2", pyint(7), None));
    rs.register_event_handler("ev", rec("h3", PyValue::None, None));
    step(
        &mut py,
        "send_event with params",
        "p.send_event('ev', 'x', 1, 2.5, None, {'k': [1]})",
        &rs,
        rs.send_event(
            "ev",
            &[
                EventArg::Value(pystr("x")),
                EventArg::Value(pyint(1)),
                EventArg::Value(PyValue::Float(2.5)),
                EventArg::Value(PyValue::None),
                EventArg::Value(PyValue::Dict(IndexMap::from([(
                    pystr("k"),
                    PyValue::List(vec![pyint(1)]),
                )]))),
            ],
        )
        .map(|v| Value::List(v.iter().map(recorder_of).collect())),
    );
    step(
        &mut py,
        "send_event without handlers",
        "p.send_event('nobody')",
        &rs,
        rs.send_event("nobody", &[]).map(|v| Value::List(v.iter().map(recorder_of).collect())),
    );
    // An error ends the dispatch and propagates.
    run(
        &mut py,
        "p.register_event_handler('ev2', rec('e1'))\n\
         p.register_event_handler('ev2', rec('e2', boom=configparser.Error('kaboom')))\n\
         p.register_event_handler('ev2', rec('e3'))",
    );
    rs.register_event_handler("ev2", rec_plain("e1"));
    rs.register_event_handler("ev2", rec("e2", PyValue::None, Some(KlippyError::config("kaboom"))));
    rs.register_event_handler("ev2", rec_plain("e3"));
    step(
        &mut py,
        "handler error propagates",
        "p.send_event('ev2')",
        &rs,
        rs.send_event("ev2", &[]).map(|v| Value::List(v.iter().map(recorder_of).collect())),
    );
    // A handler registered during dispatch runs in the same dispatch.
    run(
        &mut py,
        "def adder(*args):\n    TRACE.append('adder:')\n    p.register_event_handler('ev3', rec('late', 'rl'))\n    return 'ra'\n\
         p.register_event_handler('ev3', adder)\n\
         p.register_event_handler('ev3', rec('after', 'rt'))",
    );
    let adder_printer = rs.clone();
    rs.register_event_handler(
        "ev3",
        Box::new(move |_params: &[EventArg]| {
            trace("adder:");
            adder_printer.register_event_handler("ev3", rec("late", pystr("rl"), None));
            Ok(pystr("ra"))
        }),
    );
    rs.register_event_handler("ev3", rec("after", pystr("rt"), None));
    step(
        &mut py,
        "handler registered during dispatch",
        "p.send_event('ev3')",
        &rs,
        rs.send_event("ev3", &[]).map(|v| Value::List(v.iter().map(recorder_of).collect())),
    );
    step(
        &mut py,
        "second dispatch sees the late handler once",
        "p.send_event('ev3')",
        &rs,
        {
            // The adder registers another `late` handler each time.
            rs.send_event("ev3", &[]).map(|v| Value::List(v.iter().map(recorder_of).collect()))
        },
    );
}

// ---------------------------------------------------------------------------
// Tests: connect.
// ---------------------------------------------------------------------------

fn connect_step(py: &mut Oracle, label: &str, rs: &RustPrinter) {
    step(py, label, "p._connect(0.0)", rs, unit(rs.connect(0.0)));
}

#[test]
fn connect_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let canbus = ["canbus_ids"];

    // Clean connect with identify, connect and ready handlers.
    let rs = fresh(&mut py, &[], &canbus, false);
    run(
        &mut py,
        "p.register_event_handler('klippy:mcu_identify', rec('identify', 'i'))\n\
         p.register_event_handler('klippy:connect', rec('c1'))\n\
         p.register_event_handler('klippy:connect', rec('c2'))\n\
         p.register_event_handler('klippy:ready', rec('r1'))\n\
         p.register_event_handler('klippy:ready', rec('r2'))",
    );
    rs.register_event_handler("klippy:mcu_identify", rec("identify", pystr("i"), None));
    for (event, tag) in [
        ("klippy:connect", "c1"),
        ("klippy:connect", "c2"),
        ("klippy:ready", "r1"),
        ("klippy:ready", "r2"),
    ] {
        rs.register_event_handler(event, rec_plain(tag));
    }
    connect_step(&mut py, "clean connect", &rs);

    // Errors of the first phase, one per except clause.
    for (key, kind) in [
        ("read_main_config", "config"),
        ("pins", "pins"),
        ("mcu", "protocol"),
        ("mcu", "foreign"),
        ("check_unused_options", "runtime"),
        ("check_unused_options", "command"),
        ("load_canbus", "runtime"),
    ] {
        let rs = fresh(&mut py, &[(key, kind)], &canbus, false);
        connect_step(&mut py, &format!("connect fails at {key} with {kind}"), &rs);
    }

    // MCU error: state, notify event with the details, MCU build dump.
    let rs = fresh(&mut py, &[("mcu", "mcu")], &canbus, false);
    run(&mut py, "p.register_event_handler('klippy:notify_mcu_error', rec('notify'))");
    rs.register_event_handler("klippy:notify_mcu_error", rec_plain("notify"));
    connect_step(&mut py, "mcu error", &rs);

    // The notify handler raising escapes `_connect`.
    let rs = fresh(&mut py, &[("mcu", "mcu")], &canbus, false);
    run(
        &mut py,
        "p.register_event_handler('klippy:notify_mcu_error', rec('notify', boom=RuntimeError('notify boom')))",
    );
    rs.register_event_handler(
        "klippy:notify_mcu_error",
        rec("notify", PyValue::None, Some(runtime_error("notify boom"))),
    );
    connect_step(&mut py, "mcu error with failing notify handler", &rs);

    // A duplicate object is a config error.
    let rs = fresh(&mut py, &[], &canbus, false);
    run(&mut py, "p.add_object('pins', Obj('pre'))");
    rs.add_object("pins", marker()).expect("pre-register");
    connect_step(&mut py, "duplicate object", &rs);

    // A connect handler error is an internal error.
    let rs = fresh(&mut py, &[], &canbus, false);
    run(
        &mut py,
        "p.register_event_handler('klippy:connect', rec('c1', boom=CommandError('cmd boom')))\n\
         p.register_event_handler('klippy:connect', rec('c2'))",
    );
    rs.register_event_handler(
        "klippy:connect",
        rec("c1", PyValue::None, Some(KlippyError::command("cmd boom"))),
    );
    rs.register_event_handler("klippy:connect", rec_plain("c2"));
    connect_step(&mut py, "connect handler error", &rs);

    // A connect handler that changes the state ends `_connect`.
    let rs = fresh(&mut py, &[], &canbus, false);
    run(
        &mut py,
        "p.register_event_handler('klippy:connect', lambda: p.invoke_shutdown('stop in connect'))\n\
         p.register_event_handler('klippy:connect', rec('c2'))\n\
         p.register_event_handler('klippy:ready', rec('r1'))",
    );
    let stopper = rs.clone();
    rs.register_event_handler(
        "klippy:connect",
        Box::new(move |_params: &[EventArg]| {
            stopper.invoke_shutdown("stop in connect", PyValue::Dict(IndexMap::new()));
            Ok(PyValue::None)
        }),
    );
    rs.register_event_handler("klippy:connect", rec_plain("c2"));
    rs.register_event_handler("klippy:ready", rec_plain("r1"));
    connect_step(&mut py, "connect handler shuts down", &rs);

    // Ready handler errors shut the printer down.
    let rs = fresh(&mut py, &[], &canbus, false);
    run(
        &mut py,
        "p.register_event_handler('klippy:ready', rec('r1', boom=RuntimeError('ready boom')))\n\
         p.register_event_handler('klippy:ready', rec('r2'))\n\
         p.register_event_handler('klippy:shutdown', rec('s1'))\n\
         p.register_event_handler('klippy:analyze_shutdown', rec('a1'))",
    );
    rs.register_event_handler("klippy:ready", rec("r1", PyValue::None, Some(runtime_error("ready boom"))));
    rs.register_event_handler("klippy:ready", rec_plain("r2"));
    rs.register_event_handler("klippy:shutdown", rec_plain("s1"));
    rs.register_event_handler("klippy:analyze_shutdown", rec_plain("a1"));
    connect_step(&mut py, "ready handler error", &rs);

    // A ready handler that changes the state skips the rest.
    let rs = fresh(&mut py, &[], &canbus, false);
    run(
        &mut py,
        "p.register_event_handler('klippy:ready', lambda: p.invoke_shutdown('stop in ready'))\n\
         p.register_event_handler('klippy:ready', rec('r2'))",
    );
    let stopper = rs.clone();
    rs.register_event_handler(
        "klippy:ready",
        Box::new(move |_params: &[EventArg]| {
            stopper.invoke_shutdown("stop in ready", PyValue::Dict(IndexMap::new()));
            Ok(PyValue::None)
        }),
    );
    rs.register_event_handler("klippy:ready", rec_plain("r2"));
    connect_step(&mut py, "ready handler shuts down", &rs);

    // With debuginput the first error ends the run.
    let rs = fresh(&mut py, &[("read_main_config", "config")], &canbus, true);
    connect_step(&mut py, "config error with debuginput", &rs);
}

// ---------------------------------------------------------------------------
// Tests: state, shutdown.
// ---------------------------------------------------------------------------

#[test]
fn shutdown_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    run(
        &mut py,
        "p.register_event_handler('klippy:shutdown', rec('s1'))\n\
         p.register_event_handler('klippy:shutdown', rec('s2', boom=RuntimeError('s2 boom')))\n\
         p.register_event_handler('klippy:shutdown', rec('s3'))\n\
         p.register_event_handler('klippy:analyze_shutdown', rec('a1'))\n\
         p.register_event_handler('klippy:analyze_shutdown', rec('a2', boom=ValueError('a2 boom')))\n\
         p.register_event_handler('klippy:analyze_shutdown', rec('a3'))",
    );
    rs.register_event_handler("klippy:shutdown", rec_plain("s1"));
    rs.register_event_handler("klippy:shutdown", rec("s2", PyValue::None, Some(runtime_error("s2 boom"))));
    rs.register_event_handler("klippy:shutdown", rec_plain("s3"));
    rs.register_event_handler("klippy:analyze_shutdown", rec_plain("a1"));
    rs.register_event_handler(
        "klippy:analyze_shutdown",
        rec(
            "a2",
            PyValue::None,
            Some(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "a2 boom".to_string(),
            }),
        ),
    );
    rs.register_event_handler("klippy:analyze_shutdown", rec_plain("a3"));
    step(
        &mut py,
        "invoke_shutdown",
        "p.invoke_shutdown('die', {'why': 1})",
        &rs,
        {
            rs.invoke_shutdown("die", PyValue::Dict(IndexMap::from([(pystr("why"), pyint(1))])));
            Ok(Value::Null)
        },
    );
    step(&mut py, "second invoke_shutdown", "p.invoke_shutdown('again')", &rs, {
        rs.invoke_shutdown("again", PyValue::Dict(IndexMap::new()));
        Ok(Value::Null)
    });

    // Default details, from the ready state.
    let rs = fresh(&mut py, &[], &[], false);
    run(
        &mut py,
        "p._set_state(klippy_mod.message_ready)\n\
         p.register_event_handler('klippy:analyze_shutdown', rec('a1'))",
    );
    rs.set_state(MESSAGE_READY);
    rs.register_event_handler("klippy:analyze_shutdown", rec_plain("a1"));
    step(&mut py, "shutdown from ready", "p.invoke_shutdown('plain')", &rs, {
        rs.invoke_shutdown("plain", PyValue::Dict(IndexMap::new()));
        Ok(Value::Null)
    });

    // With debuginput a shutdown requests error_exit.
    let rs = fresh(&mut py, &[], &[], true);
    step(&mut py, "shutdown with debuginput", "p.invoke_shutdown('dbg down')", &rs, {
        rs.invoke_shutdown("dbg down", PyValue::Dict(IndexMap::new()));
        Ok(Value::Null)
    });
}

#[test]
fn state_updates_match_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    let void = |rs: &RustPrinter, f: &dyn Fn(&RustPrinter)| -> KResult<Value> {
        f(rs);
        Ok(Value::Null)
    };
    step(&mut py, "set_state err1", "p._set_state('err1')", &rs, void(&rs, &|p| p.set_state("err1")));
    step(&mut py, "set_state err2 keeps err1", "p._set_state('err2')", &rs, void(&rs, &|p| p.set_state("err2")));
    step(
        &mut py,
        "update_error_msg with other old message",
        "p.update_error_msg('other', 'new')",
        &rs,
        void(&rs, &|p| p.update_error_msg("other", "new")),
    );
    step(
        &mut py,
        "update_error_msg to the ready message",
        "p.update_error_msg('err1', klippy_mod.message_ready)",
        &rs,
        void(&rs, &|p| p.update_error_msg("err1", MESSAGE_READY)),
    );
    step(
        &mut py,
        "update_error_msg to the startup message",
        "p.update_error_msg('err1', klippy_mod.message_startup)",
        &rs,
        void(&rs, &|p| p.update_error_msg("err1", MESSAGE_STARTUP)),
    );
    step(
        &mut py,
        "update_error_msg",
        "p.update_error_msg('err1', 'err3 100% done %s')",
        &rs,
        void(&rs, &|p| p.update_error_msg("err1", "err3 100% done %s")),
    );
    step(&mut py, "get_state_message", "p.get_state_message()", &rs, {
        let (m, c) = rs.get_state_message();
        Ok(Value::Tuple(vec![Value::str(&m), Value::str(&c)]))
    });
    step(&mut py, "request_exit", "p.request_exit('exit')", &rs, void(&rs, &|p| p.request_exit("exit")));
    step(
        &mut py,
        "request_exit again",
        "p.request_exit('error_exit')",
        &rs,
        void(&rs, &|p| p.request_exit("error_exit")),
    );

    // The ready state is left only through update rules and shutdown.
    let rs = fresh(&mut py, &[], &[], false);
    step(
        &mut py,
        "set_state ready",
        "p._set_state(klippy_mod.message_ready)",
        &rs,
        void(&rs, &|p| p.set_state(MESSAGE_READY)),
    );
    step(
        &mut py,
        "update_error_msg in ready",
        "p.update_error_msg(klippy_mod.message_ready, 'no change')",
        &rs,
        void(&rs, &|p| p.update_error_msg(MESSAGE_READY, "no change")),
    );
    step(
        &mut py,
        "set_state error after ready",
        "p._set_state('later error')",
        &rs,
        void(&rs, &|p| p.set_state("later error")),
    );

    // debuginput: any state but ready requests error_exit.
    let rs = fresh(&mut py, &[], &[], true);
    step(
        &mut py,
        "set_state ready with debuginput",
        "p._set_state(klippy_mod.message_ready)",
        &rs,
        void(&rs, &|p| p.set_state(MESSAGE_READY)),
    );
    step(&mut py, "set_state error with debuginput", "p._set_state('dbg error')", &rs, void(&rs, &|p| p.set_state("dbg error")));

    // Rollover info reaches the logger, and is logged unless asked not to.
    run(
        &mut py,
        "reset()\nbg = queuelogger.QueueListener(ROOT + '/rollover-py.log')\np = new_printer(None, bg)",
    );
    clear_state();
    let bg = Arc::new(QueueListener::new(&scratch("rollover-rs.log")).expect("queue listener"));
    let rs = RustPrinter::new(Reactor::new(false), Some(bg.clone()), start_args(false), env()).expect("construct printer");
    let info = |bg: &QueueListener| {
        Value::List(
            bg.rollover_info_sorted()
                .into_iter()
                .map(|(k, v)| Value::Tuple(vec![Value::str(&k), Value::str(&v)]))
                .collect(),
        )
    };
    step(
        &mut py,
        "set_rollover_info",
        "(p.set_rollover_info('config', 'CFG %s'), sorted(bg.rollover_info.items()))[1]",
        &rs,
        {
            rs.set_rollover_info("config", Some("CFG %s"), true);
            Ok(info(&bg))
        },
    );
    step(
        &mut py,
        "set_rollover_info without log",
        "(p.set_rollover_info('aaa', 'A', False), sorted(bg.rollover_info.items()))[1]",
        &rs,
        {
            rs.set_rollover_info("aaa", Some("A"), false);
            Ok(info(&bg))
        },
    );
    step(
        &mut py,
        "set_rollover_info None",
        "(p.set_rollover_info('config', None), sorted(bg.rollover_info.items()))[1]",
        &rs,
        {
            rs.set_rollover_info("config", None, true);
            Ok(info(&bg))
        },
    );
    run(&mut py, "bg.stop()");
    bg.stop();
    // Without a logger only the log line remains.
    let rs = fresh(&mut py, &[], &[], false);
    step(&mut py, "set_rollover_info without logger", "p.set_rollover_info('x', 'X')", &rs, {
        rs.set_rollover_info("x", Some("X"), true);
        Ok(Value::Null)
    });
}

#[test]
fn async_shutdown_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    run(
        &mut py,
        "p.register_event_handler('klippy:shutdown', rec('s1'))\n\
         p.register_event_handler('klippy:shutdown', lambda: p.request_exit('exit'))\n\
         p.reactor.register_callback(lambda et: p.invoke_async_shutdown('async down', {'a': 2}))",
    );
    rs.register_event_handler("klippy:shutdown", rec_plain("s1"));
    let exiter = rs.clone();
    rs.register_event_handler(
        "klippy:shutdown",
        Box::new(move |_params: &[EventArg]| {
            exiter.request_exit("exit");
            Ok(PyValue::None)
        }),
    );
    // The reactor runs `_connect` first (registered by the constructor);
    // the next callback starts the asynchronous shutdown from inside the
    // running reactor, so the self-pipe of the Rust reactor exists.
    let starter = rs.clone();
    rs.get_reactor().register_callback(
        move |_et: f64| {
            starter.invoke_async_shutdown("async down", PyValue::Dict(IndexMap::from([(pystr("a"), pyint(2))])));
        },
        NOW,
    );
    step(
        &mut py,
        "invoke_async_shutdown",
        "(p.reactor.run(), p.reactor.run())[1]",
        &rs,
        {
            rs.get_reactor().run();
            Ok(Value::Null)
        },
    );
}

// ---------------------------------------------------------------------------
// Tests: run.
// ---------------------------------------------------------------------------

#[test]
fn run_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };

    // Normal exit: connect, ready, exit request, disconnect.
    let rs = fresh(&mut py, &[("hooks", "main")], &["canbus_ids"], false);
    run(
        &mut py,
        "RESULTS[:] = ['exit']\n\
         p.register_event_handler('klippy:ready', rec('r1'))\n\
         p.register_event_handler('klippy:disconnect', rec('d1'))",
    );
    RESULTS.lock().push("exit".to_string());
    rs.register_event_handler("klippy:ready", rec_plain("r1"));
    rs.register_event_handler("klippy:disconnect", rec_plain("d1"));
    step(&mut py, "run to exit", "p.run()", &rs, Ok(opt_string(rs.run())));

    // firmware_restart sends its event before disconnect.
    let rs = fresh(&mut py, &[("hooks", "main")], &["canbus_ids"], false);
    run(&mut py, "RESULTS[:] = ['firmware_restart']");
    RESULTS.lock().push("firmware_restart".to_string());
    step(&mut py, "run to firmware_restart", "p.run()", &rs, Ok(opt_string(rs.run())));

    // An error of the connect phase with debuginput ends the run.
    let rs = fresh(&mut py, &[("read_main_config", "config")], &["canbus_ids"], true);
    step(&mut py, "run with a config error and debuginput", "p.run()", &rs, Ok(opt_string(rs.run())));

    // A disconnect handler error is logged with the host snapshot.
    let rs = fresh(&mut py, &[("hooks", "main")], &["canbus_ids"], false);
    run(
        &mut py,
        "RESULTS[:] = ['exit']\n\
         p.register_event_handler('klippy:disconnect', rec('d1', boom=RuntimeError('disc boom')))",
    );
    RESULTS.lock().push("exit".to_string());
    rs.register_event_handler("klippy:disconnect", rec("d1", PyValue::None, Some(runtime_error("disc boom"))));
    step(&mut py, "run with failing disconnect handler", "p.run()", &rs, Ok(opt_string(rs.run())));

    // A failing connect leaves `run` through the unhandled-exception path:
    // the MCU error is reported, the notify handler raises, the reactor is
    // run again to shut down.
    let rs = fresh(&mut py, &[("mcu", "mcu")], &["canbus_ids"], false);
    run(
        &mut py,
        "p.register_event_handler('klippy:notify_mcu_error', rec('notify', boom=RuntimeError('notify boom')))\n\
         p.register_event_handler('klippy:shutdown', rec('s1'))\n\
         p.register_event_handler('klippy:shutdown', lambda: p.request_exit('exit'))",
    );
    rs.register_event_handler(
        "klippy:notify_mcu_error",
        rec("notify", PyValue::None, Some(runtime_error("notify boom"))),
    );
    rs.register_event_handler("klippy:shutdown", rec_plain("s1"));
    let exiter = rs.clone();
    rs.register_event_handler(
        "klippy:shutdown",
        Box::new(move |_params: &[EventArg]| {
            exiter.request_exit("exit");
            Ok(PyValue::None)
        }),
    );
    step(&mut py, "run with an exception out of connect", "p.run()", &rs, Ok(opt_string(rs.run())));
}

#[test]
fn start_printer_line_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    for (systime, mono) in [
        (0.0_f64, 1.0_f64),
        (1.9, 100.25),
        (1623244245.123456, 12345.678),
        (978307200.96, 0.04),
        (1700000000.05, 99999.95),
        (4102444799.0, 3.0),
    ] {
        let expected = py.eval(&format!("start_line_py({systime:?}, {mono:?})"));
        let line = start_printer_line(systime, mono);
        check(
            &format!("start line {systime}"),
            expected,
            Value::Tuple(vec![Value::str("INFO"), Value::str(&line)]),
        );
    }
    // The live line has the same shape.
    let rs = new_rs(false);
    let exiter = rs.clone();
    rs.get_reactor().register_callback(move |_et: f64| exiter.request_exit("exit"), NOW);
    clear_state();
    // `connect` runs first and logs nothing about the clock.
    assert_eq!(rs.run(), Some("exit".to_string()));
    let logs = LOGS.lock().clone();
    let line = logs
        .iter()
        .find(|(_, m)| m.starts_with("Start printer at "))
        .map(|(_, m)| m.clone())
        .expect("Start printer line");
    let (stamp, tail) = line
        .strip_prefix("Start printer at ")
        .and_then(|r| r.split_once(" ("))
        .expect("line shape");
    let nums: Vec<&str> = tail.trim_end_matches(')').split(' ').collect();
    assert_eq!(nums.len(), 2, "{line}");
    let systime: f64 = nums[0].parse().expect("systime");
    let expected = py.eval(&format!("time.asctime(time.localtime({systime:?}))"));
    let Ok(Value::Str(py_stamp)) = expected else { panic!("asctime") };
    // `%.1f` may round up to the next second.
    let earlier = py.eval(&format!("time.asctime(time.localtime({:?}))", systime - 1.0));
    let Ok(Value::Str(py_earlier)) = earlier else { panic!("asctime") };
    assert!(stamp == py_stamp || stamp == py_earlier, "{stamp} vs {py_stamp}");
}

// ---------------------------------------------------------------------------
// Tests: host snapshot.
// ---------------------------------------------------------------------------

fn py_f(x: f64) -> String {
    format!("{x:?}")
}

fn py_b(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_opt_str(s: &Option<String>) -> String {
    match s {
        Some(s) => format!("{s:?}"),
        None => "None".to_string(),
    }
}

struct HostCase {
    info: HostInfo,
    shutdown: bool,
    run_result: Option<String>,
}

fn host_case_py(case: &HostCase) -> String {
    let i = &case.info;
    let rusage = match &i.rusage {
        None => "None".to_string(),
        Some(r) => format!(
            "{{'user': {}, 'system': {}, 'maxrss': {}, 'minflt': {}, 'majflt': {}, 'nvcsw': {}, 'nivcsw': {}}}",
            py_f(r.user),
            py_f(r.system),
            r.maxrss,
            r.minflt,
            r.majflt,
            r.nvcsw,
            r.nivcsw
        ),
    };
    let (core, nofile) = match &i.rusage {
        Some(r) => (r.core_limit, r.nofile_limit),
        None => ((0, 0), (0, 0)),
    };
    format!(
        "host_lines_py({{'pid': {}, 'wall': {}, 'mono': {}, 'version': {:?}, 'platform': {:?}, \
         'gc_count': ({}, {}, {}), 'gc_times': ({}, {}, {}), 'threads': {}, 'rusage': {}, \
         'core': ({}, {}), 'nofile': ({}, {}), 'status': {}, 'shutdown': {}, 'run_result': {}}})",
        i.pid,
        py_f(i.wall_time),
        py_f(i.monotonic),
        i.python_version,
        i.platform,
        i.gc_count.0,
        i.gc_count.1,
        i.gc_count.2,
        py_f(i.gc_times[0]),
        py_f(i.gc_times[1]),
        py_f(i.gc_times[2]),
        i.threads,
        rusage,
        core.0,
        core.1,
        nofile.0,
        nofile.1,
        py_opt_str(&i.proc_status),
        py_b(case.shutdown),
        py_opt_str(&case.run_result),
    )
}

#[test]
fn host_state_lines_match_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let long_line = format!("VmPeak:\t{}", "9".repeat(200));
    let cases = [
        HostCase {
            info: HostInfo {
                pid: 4242,
                wall_time: 1623244245.123456,
                monotonic: 100.0,
                python_version: "3.8.2 (default, test)".to_string(),
                platform: "linux".to_string(),
                gc_count: (700, 10, 3),
                gc_times: [0.001, 0.002, 0.003],
                threads: 3,
                rusage: Some(Rusage {
                    user: 1.5,
                    system: 0.25,
                    maxrss: 12345,
                    minflt: 100,
                    majflt: 2,
                    nvcsw: 50,
                    nivcsw: 6,
                    core_limit: (0, -1),
                    nofile_limit: (1024, 1048576),
                }),
                proc_status: Some(
                    "Name:\ttest\nVmPeak:\t  100 kB\nVmSize:\t  200 kB\nVmHWM:\t  90 kB\nVmRSS:\t  80 kB\nVmSwap:\t   0 kB\nThreads:\t3\nFDSize:\t64\nOther:\tvalue\n"
                        .to_string(),
                ),
            },
            shutdown: false,
            run_result: None,
        },
        HostCase {
            info: HostInfo {
                pid: 1,
                wall_time: 0.5,
                monotonic: 12345.678901,
                python_version: format!("3.8.2 it's {}", "x".repeat(600)),
                platform: "linux2".to_string(),
                gc_count: (0, 0, 0),
                gc_times: [0.0, 1.5, 100.0],
                threads: 1,
                rusage: None,
                proc_status: None,
            },
            shutdown: true,
            run_result: Some("error_exit".to_string()),
        },
        HostCase {
            info: HostInfo {
                pid: 99,
                wall_time: 2.0,
                monotonic: 3.0,
                python_version: "3.8.2".to_string(),
                platform: "linux".to_string(),
                gc_count: (1, 2, 3),
                gc_times: [0.1, 0.2, 0.3],
                threads: 12,
                rusage: Some(Rusage {
                    user: 0.000001,
                    system: 123456.789012,
                    maxrss: 1,
                    minflt: 2,
                    majflt: 3,
                    nvcsw: 4,
                    nivcsw: 5,
                    core_limit: (-1, -1),
                    nofile_limit: (-1, 5),
                }),
                proc_status: Some(format!(
                    "{long_line}\nFDSize:\t64\nVmRSS:\t1 kB\nVmPeakX:\t5\nvmpeak:\t6\nThreads:\t12\n"
                )),
            },
            shutdown: false,
            run_result: Some("restart".to_string()),
        },
    ];
    for (idx, case) in cases.iter().enumerate() {
        let expected = py.eval(&host_case_py(case));
        let lines = host_state_lines(&case.info, case.shutdown, case.run_result.as_deref());
        check(&format!("host lines {idx}"), expected, strs(&lines));
    }
}

#[test]
fn debug_host_state_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let rs = fresh(&mut py, &[], &[], false);
    step(&mut py, "first snapshot", "p._debug_host_state()", &rs, {
        rs.debug_host_state();
        Ok(Value::Null)
    });
    step(&mut py, "second snapshot is a no-op", "p._debug_host_state()", &rs, {
        rs.debug_host_state();
        Ok(Value::Null)
    });
}

// ---------------------------------------------------------------------------
// Tests: startup helpers.
// ---------------------------------------------------------------------------

#[test]
fn arg_dictionary_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    run(
        &mut py,
        "class _Vals:\n    dictionary = None\n\
         class _Parser:\n    values = _Vals()",
    );
    let mut dict: Option<IndexMap<String, String>> = None;
    for value in ["file.dict", "mcu1=a.dict", "a=b=c", "=x", "", "mcu1=again.dict", "file.dict2"] {
        run(
            &mut py,
            &format!("klippy_mod.arg_dictionary(None, None, {value:?}, _Parser())"),
        );
        arg_dictionary(dict.get_or_insert_with(IndexMap::new), value);
        let actual = Value::Dict(
            dict.as_ref()
                .expect("dictionary")
                .iter()
                .map(|(k, v)| (Value::str(k), Value::str(v)))
                .collect(),
        );
        check(&format!("arg_dictionary({value:?})"), py.eval("_Parser.values.dictionary"), actual);
    }
}

fn options_value(o: &MainOptions) -> Value {
    let opt = |s: &Option<String>| match s {
        Some(s) => Value::str(s),
        None => Value::Null,
    };
    let dictionary = match &o.dictionary {
        None => Value::Null,
        Some(d) => Value::Dict(d.iter().map(|(k, v)| (Value::str(k), Value::str(v))).collect()),
    };
    Value::Dict(vec![
        (Value::str("apiserver"), opt(&o.apiserver)),
        (Value::str("debuginput"), opt(&o.debuginput)),
        (Value::str("debugoutput"), opt(&o.debugoutput)),
        (Value::str("dictionary"), dictionary),
        (Value::str("import_test"), Value::Bool(o.import_test)),
        (Value::str("inputtty"), Value::str(&o.inputtty)),
        (Value::str("logfile"), opt(&o.logfile)),
        (Value::str("verbose"), Value::Bool(o.verbose)),
    ])
}

fn parse_value(argv: &[String]) -> Value {
    match parse_cli(argv) {
        Ok(parsed) => Value::Tuple(vec![
            Value::str("ok"),
            options_value(&parsed.options),
            strs(&parsed.args),
        ]),
        Err(exit) => Value::Tuple(vec![
            Value::str("exit"),
            Value::Int(i64::from(exit.code)),
            Value::str(&exit.stdout),
            Value::str(&exit.stderr),
        ]),
    }
}

#[test]
fn cli_parse_matches_optparse() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let cases: Vec<Vec<&str>> = vec![
        vec!["klippy.py", "printer.cfg"],
        vec![
            "klippy.py", "--debuginput", "g", "-I", "/tmp/x", "-a", "/tmp/sock", "--logfile", "log", "-v",
            "--debugoutput", "out", "-d", "a.dict", "--dictionary=b=c.dict", "printer.cfg",
        ],
        vec!["klippy.py", "-ivalue.g", "-v", "printer.cfg"],
        vec!["klippy.py", "--", "-v"],
        vec!["klippy.py", "a", "--", "-v", "--import-test"],
        vec!["klippy.py", "-d", "one.dict", "-d", "mcu=two.dict", "-d", "one.dict", "-o", "out", "printer.cfg"],
        vec!["klippy.py", "--debugi", "x", "--log=y", "--api", "z", "--input", "t", "--imp", "f"],
        vec!["klippy.py", "--debugi=", "f"],
        vec!["klippy.py", "a", "-v", "b", "-", "-vvv"],
        vec!["klippy.py", "-ia", "f"],
        vec!["klippy.py", "-lx", "-oy", "-ddict", "f"],
        vec!["klippy.py", "--d"],
        vec!["klippy.py", "--debug", "x"],
        vec!["klippy.py", "--zz"],
        vec!["klippy.py", "--zz=1"],
        vec!["klippy.py", "-z"],
        vec!["klippy.py", "-vz"],
        vec!["klippy.py", "-v=1"],
        vec!["klippy.py", "--import-test=1"],
        vec!["klippy.py", "--help=3"],
        vec!["klippy.py", "-i"],
        vec!["klippy.py", "--logfile"],
        vec!["klippy.py", "-d"],
        vec!["klippy.py", "-vi"],
        vec!["klippy.py", "-I"],
        vec!["klippy.py", "-h"],
        vec!["klippy.py", "--help"],
        vec!["klippy.py", "--he"],
        vec!["klippy.py", "-vh"],
        vec!["/usr/local/bin/klippy.py", "-h"],
        vec!["/usr/local/bin/klippy.py", "--zz"],
        vec!["klippy.py", ""],
        vec!["klippy.py", "-i", ""],
        vec!["klippy.py"],
    ];
    for case in cases {
        let argv: Vec<String> = case.iter().map(|s| s.to_string()).collect();
        let items: Vec<String> = argv.iter().map(|s| format!("{s:?}")).collect();
        check(
            &format!("parse {case:?}"),
            py.eval(&format!("parse_py([{}])", items.join(", "))),
            parse_value(&argv),
        );
    }
}

fn names_value(names: &[String]) -> Value {
    strs(names)
}

#[test]
fn import_test_matches_python_on_the_real_tree() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    let real = py
        .eval("REAL_DIR")
        .expect("klippy dir");
    let Value::Str(real_dir) = real else { panic!("klippy dir") };
    // The module imported the real file before `use_tree` moved it.
    run(&mut py, &format!("use_tree({real_dir:?})"));
    let names = import_test_names(&real_dir).expect("listing");
    assert!(names.iter().any(|n| n.starts_with("extras.")), "extras listed: {names:?}");
    assert!(names.iter().any(|n| n.starts_with("kinematics.")), "kinematics listed");
    check(
        "import order",
        py.eval("import_run_py([])"),
        Value::Tuple(vec![names_value(&names), Value::str("exit"), Value::Int(0)]),
    );
    let all = HostEnv {
        modules: names
            .iter()
            .map(|n| (n.clone(), ModuleFactory::default()))
            .collect(),
        klippy_dir: real_dir.clone(),
    };
    let exit = import_test(&all);
    assert_eq!((exit.code, exit.stderr.as_str()), (0, ""), "{exit:?}");
    // The second name missing: Python stops there with ModuleNotFoundError.
    let missing = names[1].clone();
    let upto: Vec<String> = names[..2].to_vec();
    check(
        "missing import",
        py.eval(&format!("import_run_py([{missing:?}])")),
        Value::Tuple(vec![
            names_value(&upto),
            Value::str("ModuleNotFoundError"),
            Value::str(&format!("No module named '{missing}'")),
        ]),
    );
    let partial = HostEnv {
        modules: names
            .iter()
            .filter(|n| **n != missing)
            .map(|n| (n.clone(), ModuleFactory::default()))
            .collect(),
        klippy_dir: real_dir,
    };
    let exit = import_test(&partial);
    assert_eq!(exit.code, 1);
    assert_eq!(exit.stderr, format!("ModuleNotFoundError: No module named '{missing}'\n"));
    run(&mut py, &format!("use_tree({:?})", tree()));
}

// ---------------------------------------------------------------------------
// Tests: main().
// ---------------------------------------------------------------------------

fn main_value(argv: &[String]) -> Value {
    let exit = run_main(argv, env());
    let seen: Vec<Value> = SEEN_ARGS.lock().iter().map(recorder_of).collect();
    let trace_entries = TRACE.lock().clone();
    Value::Dict(vec![
        (
            Value::str("exit"),
            Value::Tuple(vec![
                Value::Int(i64::from(exit.code)),
                Value::str(&exit.stdout),
                Value::str(&exit.stderr),
            ]),
        ),
        (Value::str("start_args"), Value::List(seen)),
        (Value::str("trace"), strs(&trace_entries)),
        (Value::str("log"), logs_value()),
    ])
}

/// One `main()` scenario in both implementations.
fn main_step(py: &mut Oracle, label: &str, argv: &[&str], modes: &[(&str, &str)], results: &[&str]) {
    let items: Vec<String> = argv.iter().map(|s| format!("{s:?}")).collect();
    let mut code = String::from("reset()\n");
    for (key, kind) in modes {
        code += &format!("MODE[{key:?}] = {kind:?}\n");
    }
    code += &format!("SECTIONS[:] = ['canbus_ids']\nRESULTS[:] = {}\n", py_list(results));
    run(py, &code);
    clear_state();
    for (key, kind) in modes {
        MODE.lock().insert(key.to_string(), kind.to_string());
    }
    *SECTIONS.lock() = vec!["canbus_ids".to_string()];
    *RESULTS.lock() = results.iter().map(|s| s.to_string()).collect();
    let argv: Vec<String> = argv.iter().map(|s| s.to_string()).collect();
    let expected = py.eval(&format!("run_main_py([{}])", items.join(", ")));
    check_obs(label, expected, main_value(&argv));
}

#[test]
fn main_matches_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    run(&mut py, "faulthandler.disable()");
    let dbg_file = scratch("dbg.gcode");
    let dbg = dbg_file.as_str();
    let hooks = [("hooks", "main")];
    // Arguments that end before anything is started.
    main_step(&mut py, "help", &["klippy.py", "-h"], &[], &[]);
    main_step(&mut py, "no arguments", &["klippy.py"], &[], &[]);
    main_step(&mut py, "two arguments", &["klippy.py", "a", "b"], &[], &[]);
    main_step(&mut py, "unknown option", &["klippy.py", "--bogus", "a"], &[], &[]);
    main_step(&mut py, "ambiguous option", &["klippy.py", "--d", "a"], &[], &[]);
    main_step(&mut py, "missing value", &["klippy.py", "-i"], &[], &[]);
    main_step(&mut py, "missing debuginput", &["klippy.py", "-i", "/nonexistent/dbg", "printer.cfg"], &[], &[]);
    main_step(&mut py, "debugoutput without dictionary", &["klippy.py", "-i", dbg, "-o", "out", "printer.cfg"], &[], &[]);
    // Import test against the temporary tree: `extras.pkg` and
    // `kinematics.cartesian` are not in the registry.
    run(
        &mut py,
        "reset()\nIMPORT_MISSING.update(['extras.pkg', 'kinematics.cartesian'])",
    );
    clear_state();
    let expected = py.eval("run_main_py(['klippy.py', '--import-test'])");
    check_obs("import test missing modules", expected, main_value(&["klippy.py".to_string(), "--import-test".to_string()]));
    // Runs that start printers.
    main_step(&mut py, "single run", &["klippy.py", "-i", dbg, "printer.cfg"], &hooks, &["exit"]);
    main_step(
        &mut py,
        "dictionaries and debugoutput",
        &["klippy.py", "-i", dbg, "-v", "-o", "out", "-d", "a.dict", "-d", "mcu=b.dict", "-a", "/tmp/api", "printer.cfg"],
        &hooks,
        &["exit"],
    );
    main_step(
        &mut py,
        "restarts",
        &["klippy.py", "-i", dbg, "printer.cfg"],
        &hooks,
        &["restart", "firmware_restart", "exit"],
    );
    main_step(
        &mut py,
        "error exit",
        &["klippy.py", "-i", dbg, "printer.cfg"],
        &[("read_main_config", "config")],
        &[],
    );
    let logfile_path = scratch("main.log");
    let logfile = logfile_path.as_str();
    main_step(
        &mut py,
        "log file and versions block",
        &["klippy.py", "-i", dbg, "-l", logfile, "printer.cfg"],
        &hooks,
        &["exit"],
    );
    let tty_path = scratch("tty");
    let tty = tty_path.as_str();
    main_step(&mut py, "pty input", &["klippy.py", "-I", tty, "printer.cfg"], &hooks, &["exit"]);
}

#[test]
fn git_and_versions_blocks_match_python() {
    let _serial = begin();
    let Some(mut py) = oracle() else { return };
    run(&mut py, "faulthandler.disable()");
    let dbg = scratch("dbg.gcode");
    let logfile = scratch("git.log");
    let many_untracked: Vec<Vec<String>> = (0..12)
        .map(|i| vec!["??".to_string(), format!("klippy/extras/f{i:02}.py")])
        .chain((0..13).map(|i| vec!["M".to_string(), format!("m{i:02}.cfg")]))
        .collect();
    let cases: Vec<(&str, GitInfo)> = vec![
        (
            "empty status",
            GitInfo {
                version: "v0-1-gabc".to_string(),
                file_status: vec![],
                branch: "main".to_string(),
                remote: "origin".to_string(),
                url: "https://example.org/repo.git".to_string(),
            },
        ),
        (
            "untracked and modified",
            GitInfo {
                version: "v1".to_string(),
                file_status: vec![
                    vec!["??".to_string(), "klippy/extras/new.py".to_string()],
                    vec!["M".to_string(), "x.cfg".to_string()],
                    vec!["??".to_string(), "klippy/other.py".to_string()],
                    vec!["!!".to_string(), "klippy/kinematics/k.py".to_string()],
                    vec!["??".to_string(), "klippy/extras/not python.txt".to_string()],
                ],
                branch: "b".to_string(),
                remote: "r".to_string(),
                url: "u".to_string(),
            },
        ),
        (
            "many files and a dirty version",
            GitInfo {
                version: "v2-dirty".to_string(),
                file_status: many_untracked,
                branch: "b".to_string(),
                remote: "r".to_string(),
                url: "u".to_string(),
            },
        ),
        (
            "only modified",
            GitInfo {
                version: "v3".to_string(),
                file_status: vec![vec!["M".to_string(), "a b.cfg".to_string()]],
                branch: "b".to_string(),
                remote: "r".to_string(),
                url: "u".to_string(),
            },
        ),
        (
            "empty pair",
            GitInfo {
                version: "v4".to_string(),
                file_status: vec![vec![]],
                branch: "b".to_string(),
                remote: "r".to_string(),
                url: "u".to_string(),
            },
        ),
        (
            "short pair",
            GitInfo {
                version: "v5".to_string(),
                file_status: vec![vec!["M".to_string()]],
                branch: "b".to_string(),
                remote: "r".to_string(),
                url: "u".to_string(),
            },
        ),
        (
            "long pair",
            GitInfo {
                version: "v6".to_string(),
                file_status: vec![vec!["M".to_string(), "a".to_string(), "b".to_string()]],
                branch: "b".to_string(),
                remote: "r".to_string(),
                url: "u".to_string(),
            },
        ),
    ];
    let argv = vec![
        "klippy.py".to_string(),
        "-i".to_string(),
        dbg.clone(),
        "-l".to_string(),
        logfile.clone(),
        "printer.cfg".to_string(),
    ];
    for (label, info) in cases {
        let status: Vec<String> = info
            .file_status
            .iter()
            .map(|pair| {
                let items: Vec<String> = pair.iter().map(|s| format!("{s:?}")).collect();
                format!("[{}]", items.join(", "))
            })
            .collect();
        let git = format!(
            "{{'version': {:?}, 'file_status': [{}], 'branch': {:?}, 'remote': {:?}, 'url': {:?}}}",
            info.version,
            status.join(", "),
            info.branch,
            info.remote,
            info.url
        );
        let args: Vec<String> = argv.iter().map(|s| format!("{s:?}")).collect();
        run(
            &mut py,
            "reset()\nMODE['hooks'] = 'main'\nSECTIONS[:] = ['canbus_ids']\nRESULTS[:] = ['exit']",
        );
        let expected = py.eval(&format!(
            "run_main_py([{}], git={git}, cpu=('cpu-x', 'dev-y', 'linux-z'))",
            args.join(", ")
        ));
        let Ok(Value::Dict(pairs)) = expected else { panic!("{label}: run_main_py failed") };
        let get = |name: &str| -> Value {
            pairs
                .iter()
                .find(|(k, _)| *k == Value::str(name))
                .map(|(_, v)| v.clone())
                .unwrap_or_else(|| panic!("{label}: no {name}"))
        };
        let Value::Tuple(exit) = get("exit") else { panic!("{label}: exit") };
        let Value::List(seen) = get("start_args") else { panic!("{label}: start_args") };
        let Value::List(log_lines) = get("log") else { panic!("{label}: log") };
        match git_version_block(&info) {
            Err(e) => {
                assert_eq!(exit[0], Value::Int(1), "{label}: exit code");
                assert_eq!(
                    exit[2],
                    Value::str(&format!("{}: {}\n", e.class_name(), e.message())),
                    "{label}: error text"
                );
                assert!(seen.is_empty(), "{label}: no printer starts");
            }
            Ok((version, desc)) => {
                assert_eq!(exit[0], Value::Int(0), "{label}: exit code");
                let Value::Dict(start) = &seen[0] else { panic!("{label}: start args") };
                let software = start
                    .iter()
                    .find(|(k, _)| *k == Value::str("software_version"))
                    .map(|(_, v)| v.clone())
                    .expect("software_version");
                assert_eq!(software, Value::str(&version), "{label}: software_version");
                let versions = versions_block(&argv, &version, &desc, "cpu-x", "dev-y", "linux-z", PYTHON_VERSION);
                let found = log_lines.iter().any(|line| {
                    matches!(line, Value::Tuple(parts)
                        if parts.len() == 2 && parts[1] == Value::str(&versions))
                });
                assert!(found, "{label}: versions block {versions:?} not in {log_lines:?}");
            }
        }
    }
}

#[test]
fn timer_failure_runs_shutdown_and_matches_python_exit() {
    let _serial = begin();
    let mut py = oracle().expect("the Python oracle is required");
    for repeat in [false, true] {
        let rs = fresh(&mut py, &[], &["canbus_ids"], false);
        run(&mut py, &format!(r#"
import runpy
import traceback
run_exceptions = []
class CaptureRunException(logging.Handler):
    def emit(self, record):
        if record.getMessage() in ('Unhandled exception during run', 'Repeat unhandled exception during run'):
            run_exceptions.append((record.getMessage(), ''.join(
                traceback.format_exception_only(*record.exc_info[:2])).strip()))
run_exception_handler = CaptureRunException()
logging.getLogger().addHandler(run_exception_handler)
real_reactor = runpy.run_path(os.path.join(REAL_DIR, 'reactor.py'))['Reactor']()
for callback in p.reactor.callbacks:
    real_reactor.register_callback(callback)
p.reactor = real_reactor
def fail_timer(eventtime):
    raise ValueError('timer failed')
p.reactor.register_timer(fail_timer, p.reactor.NOW)
if {repeat}:
    p.reactor.register_timer(fail_timer, p.reactor.NOW)
p.register_event_handler('klippy:shutdown', lambda: p.request_exit('exit'))
timer_result = p.run()
p.reactor.finalize()
logging.getLogger().removeHandler(run_exception_handler)
timer_observation = (
    timer_result, p.in_shutdown_state, p.get_state_message(),
    [entry for entry in LOG_LINES if entry[1] in (
        'Unhandled exception during run', 'Repeat unhandled exception during run')])
timer_observation += (run_exceptions,)
"#, repeat = if repeat { "True" } else { "False" }));
        let reactor = rs.get_reactor();
        reactor.register_timer(|_| panic!("ValueError: timer failed"), NOW);
        if repeat {
            reactor.register_timer(|_| panic!("ValueError: timer failed"), NOW);
        }
        let exiter = rs.clone();
        rs.register_event_handler("klippy:shutdown", Box::new(move |_| {
            exiter.request_exit("exit");
            Ok(PyValue::None)
        }));
        let result = rs.run();
        reactor.finalize();
        let logs = match logs_value() {
            Value::List(entries) => Value::List(entries.into_iter().filter(|entry| {
                matches!(entry, Value::Tuple(fields) if fields.get(1).is_some_and(|message|
                    *message == Value::str("Unhandled exception during run")
                    || *message == Value::str("Repeat unhandled exception during run")))
            }).collect()),
            _ => unreachable!("log list"),
        };
        let (message, category) = rs.get_state_message();
        check("timer exception shutdown", py.eval("timer_observation"), Value::Tuple(vec![
            opt_string(result),
            Value::Bool(rs.is_shutdown()),
            Value::Tuple(vec![Value::str(&message), Value::str(&category)]),
            logs,
            Value::List(RUN_EXCEPTIONS.lock().iter().map(|(message, text)|
                Value::Tuple(vec![Value::str(message), Value::str(text)])).collect()),
        ]));
    }
}

#[test]
fn early_webhooks_bind_error_leaves_constructor_like_python() {
    let _serial = begin();
    let mut py = oracle().expect("the Python oracle is required");
    let socket = scratch("uncreated-api-dir/klippy.sock");
    run(&mut py, &format!(r#"
import runpy
real_webhooks = runpy.run_path(os.path.join(REAL_DIR, 'webhooks.py'))
saved_early_webhooks = webhooks_mod.add_early_printer_objects
webhooks_mod.add_early_printer_objects = real_webhooks['add_early_printer_objects']
try:
    try:
        new_printer(dict(START, apiserver={socket:?}))
        early_failure = None
    except Exception as error:
        early_failure = (type(error).__name__, str(error))
finally:
    webhooks_mod.add_early_printer_objects = saved_early_webhooks
"#));
    let mut args = start_args(false);
    args.insert("apiserver".to_string(), PyValue::Str(socket));
    let host = Arc::new(HostEnv {
        modules: IndexMap::from([(
            MODULE_WEBHOOKS.to_string(), klippy_core::webhooks::module_factory(),
        )]),
        klippy_dir: tree(),
    });
    let actual = match RustPrinter::new(Reactor::new(false), None, args, host) {
        Err(error) => Value::Tuple(vec![Value::str(error.class_name()), Value::str(error.message())]),
        Ok(_) => Value::Null,
    };
    check("early socket error", py.eval("early_failure"), actual);
}
