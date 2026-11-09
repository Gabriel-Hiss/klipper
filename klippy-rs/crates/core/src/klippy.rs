//! Port of `klippy/klippy.py`: the `Printer` object, `import_test()`,
//! `arg_dictionary()`, `setup_fault_handler()` and `main()`.
//!
//! The [`Printer`] is a [`PrinterApi`] service. Objects are
//! `Arc<dyn PrinterObject>` in an insertion-ordered map and other
//! modules reach each other through `lookup_interface`; the only
//! thing the printer needs from the outside is the table that Python
//! gets from `import`: [`HostEnv::modules`], a [`ModuleRegistry`]
//! from import name to [`ModuleFactory`].
//!
//! Registry keys are the Python import names: `extras.<name>` and
//! `kinematics.<name>` for the optional modules, and the plain module
//! names the printer itself imports (`gcode`, `webhooks`, `configfile`,
//! `pins`, `mcu`, `toolhead`):
//!
//! - `gcode` and `webhooks`: `add_early_printer_objects`, called by
//!   [`Printer::new`] in that order (`klippy/klippy.py:42`).
//! - `configfile`: its `add_early_printer_objects` stands for
//!   `configfile.PrinterConfig(self)` of `_read_config`
//!   (`klippy/klippy.py:120`) and must register the `configfile`
//!   object with the [`PrinterConfigApi`] interface. The printer calls
//!   it when `_read_config` runs, not at construction.
//! - `pins`, `mcu`: `add_printer_objects` before the section sweep;
//!   `toolhead`: `add_printer_objects` after it
//!   (`klippy/klippy.py:125-130`).
//! - `extras.*`: `load_config` / `load_config_prefix`, used by
//!   [`Printer::load_object`]. A missing entry, or a missing function,
//!   is the "module file or attribute does not exist" branch.
//!
//! Deliberate divergences from the Python original:
//!
//! - A module whose factory returns `None` is stored as a present
//!   object with no value, so `load_object` short-circuits on it like
//!   Python; `lookup_objects` omits it because [`ObjectRef`] cannot be
//!   `None`.
//! - Rust panics inside event handlers are turned into Python
//!   exceptions (`ReactorError` for the reactor's pause check) at the
//!   handler boundary, as the original's `except` clauses see them.
//! - Without a Python interpreter `sys.version` is [`PYTHON_VERSION`],
//!   `gc.get_count()` is `(0, 0, 0)` and the faulthandler stack dump
//!   only names the dumping thread on stderr. The frozen normalizer
//!   masks the snapshot lines.

use std::ffi::{c_int, c_long, c_ulong};
use std::io::Read;
use std::os::fd::IntoRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::config::{Config, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::object::{lookup_interface, EventArg, ObjectRef};
use klippy_api::printer::{
    EventCallback, LookupDefault, ModuleFactory, PrinterApi, EV_KLIPPY_ANALYZE_SHUTDOWN,
    EV_KLIPPY_CONNECT, EV_KLIPPY_DISCONNECT, EV_KLIPPY_FIRMWARE_RESTART, EV_KLIPPY_MCU_IDENTIFY,
    EV_KLIPPY_NOTIFY_MCU_ERROR, EV_KLIPPY_READY, EV_KLIPPY_SHUTDOWN,
};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NOW};

use crate::configfile::py_join;
use crate::queuelogger::{self, QueueListener};
use crate::util;

// ---------------------------------------------------------------------------
// Module constants (`klippy/klippy.py:11-23`).
// ---------------------------------------------------------------------------

/// `message_ready` (`klippy/klippy.py:11`).
pub const MESSAGE_READY: &str = "Printer is ready";

/// `message_startup` (`klippy/klippy.py:13`).
pub const MESSAGE_STARTUP: &str = "\nPrinter is not ready\nThe klippy host software is attempting to connect.  Please\nretry in a few moments.\n";

/// `message_restart` (`klippy/klippy.py:19`).
pub const MESSAGE_RESTART: &str = "\nOnce the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software.\nPrinter is halted\n";

/// optparse usage (`klippy/klippy.py:331`).
pub const USAGE: &str = "%prog [options] <config file>";

/// Registry key of `gcode` (`klippy/klippy.py:42`).
pub const MODULE_GCODE: &str = "gcode";
/// Registry key of `webhooks` (`klippy/klippy.py:42`).
pub const MODULE_WEBHOOKS: &str = "webhooks";
/// Registry key of `configfile` (`klippy/klippy.py:120`).
pub const MODULE_CONFIGFILE: &str = "configfile";
/// Registry key of `pins` (`klippy/klippy.py:125`).
pub const MODULE_PINS: &str = "pins";
/// Registry key of `mcu` (`klippy/klippy.py:125`).
pub const MODULE_MCU: &str = "mcu";
/// Registry key of `toolhead` (`klippy/klippy.py:129`).
pub const MODULE_TOOLHEAD: &str = "toolhead";

/// `sys.version` stand-in for the diagnostics and the versions block.
pub const PYTHON_VERSION: &str = "3.8.2";

/// `sys.platform` on the supported hosts.
pub const PLATFORM: &str = "linux";

/// Import name to [`ModuleFactory`]; see the module docs for the keys.
pub type ModuleRegistry = IndexMap<String, ModuleFactory>;

/// What the printer needs from the host process.
pub struct HostEnv {
    /// The importable modules.
    pub modules: ModuleRegistry,
    /// `os.path.dirname(__file__)` of `klippy.py`: the directory that
    /// holds `extras/` and `kinematics/`, and whose parent is the
    /// source tree that `util` inspects for git and MCU build files.
    pub klippy_dir: String,
}

fn fmt(fmt_str: &str, args: &[PctValue]) -> String {
    percent_format(fmt_str, args).expect("constant klippy format")
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn empty_details() -> PyValue {
    PyValue::Dict(IndexMap::new())
}

/// `str.split()` whitespace (`\x1c`-`\x1f` included).
fn is_py_space(c: char) -> bool {
    c.is_whitespace() || matches!(c, '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{1f}')
}

/// Text of `logging.exception`'s traceback for an error.
fn traceback_of(e: &KlippyError) -> String {
    format!("{}: {}", e.class_name(), e.message())
}

/// A Rust panic seen as the Python exception it stands for.
fn panic_error(payload: Box<dyn std::any::Any + Send>) -> KlippyError {
    let text = if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "panic".to_string()
    };
    let (class_name, message) = text.split_once(": ").unwrap_or(("Exception", &text));
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: message.to_string(),
    }
}

// ---------------------------------------------------------------------------
// Host state diagnostics (`klippy/klippy.py:213-258`).
// ---------------------------------------------------------------------------

/// `resource.getrusage(RUSAGE_SELF)` and the two `getrlimit` pairs
/// (`klippy/klippy.py:231-239`). Limits keep CPython's signed `long`
/// conversion, so an unlimited value reads `-1`.
#[derive(Clone, Debug, PartialEq)]
pub struct Rusage {
    /// `ru_utime`.
    pub user: f64,
    /// `ru_stime`.
    pub system: f64,
    /// `ru_maxrss`.
    pub maxrss: i64,
    /// `ru_minflt`.
    pub minflt: i64,
    /// `ru_majflt`.
    pub majflt: i64,
    /// `ru_nvcsw`.
    pub nvcsw: i64,
    /// `ru_nivcsw`.
    pub nivcsw: i64,
    /// `getrlimit(RLIMIT_CORE)`.
    pub core_limit: (i64, i64),
    /// `getrlimit(RLIMIT_NOFILE)`.
    pub nofile_limit: (i64, i64),
}

/// Everything `_debug_host_state` reads from the host.
#[derive(Clone, Debug, PartialEq)]
pub struct HostInfo {
    /// `os.getpid()`.
    pub pid: i64,
    /// `time.time()`.
    pub wall_time: f64,
    /// `self.reactor.monotonic()`.
    pub monotonic: f64,
    /// `sys.version`, before the `[:512]` cut.
    pub python_version: String,
    /// `sys.platform`.
    pub platform: String,
    /// `gc.get_count()`.
    pub gc_count: (i64, i64, i64),
    /// `self.reactor.get_gc_stats()`.
    pub gc_times: [f64; 3],
    /// `threading.active_count()`.
    pub threads: i64,
    /// The rusage snapshot, `None` for the ImportError/OSError/
    /// ValueError branch.
    pub rusage: Option<Rusage>,
    /// `f.read(8192)` of `/proc/self/status`, `None` when unreadable.
    pub proc_status: Option<String>,
}

#[repr(C)]
struct CTimeval {
    tv_sec: c_long,
    tv_usec: c_long,
}

#[repr(C)]
struct CRusage {
    ru_utime: CTimeval,
    ru_stime: CTimeval,
    ru_maxrss: c_long,
    ru_ixrss: c_long,
    ru_idrss: c_long,
    ru_isrss: c_long,
    ru_minflt: c_long,
    ru_majflt: c_long,
    ru_nswap: c_long,
    ru_inblock: c_long,
    ru_oublock: c_long,
    ru_msgsnd: c_long,
    ru_msgrcv: c_long,
    ru_nsignals: c_long,
    ru_nvcsw: c_long,
    ru_nivcsw: c_long,
}

#[repr(C)]
struct CRlimit {
    rlim_cur: c_ulong,
    rlim_max: c_ulong,
}

unsafe extern "C" {
    fn getrusage(who: c_int, usage: *mut CRusage) -> c_int;
    fn getrlimit(resource: c_int, rlim: *mut CRlimit) -> c_int;
}

const RUSAGE_SELF: c_int = 0;
const RLIMIT_CORE: c_int = 4;
#[cfg(any(target_arch = "mips", target_arch = "mips64"))]
const RLIMIT_NOFILE: c_int = 5;
#[cfg(not(any(target_arch = "mips", target_arch = "mips64")))]
const RLIMIT_NOFILE: c_int = 7;

fn timeval_seconds(tv: &CTimeval) -> f64 {
    tv.tv_sec as f64 + tv.tv_usec as f64 * 0.000001
}

// `c_long` is 32 bits on MIPS32 and 64 bits on x86_64.
#[allow(clippy::unnecessary_cast)]
fn rlimit_pair(resource: c_int) -> Option<(i64, i64)> {
    let mut rl = CRlimit { rlim_cur: 0, rlim_max: 0 };
    // SAFETY: `rl` is valid out storage for one `struct rlimit`.
    let rc = unsafe { getrlimit(resource, &mut rl) };
    if rc != 0 {
        return None;
    }
    Some((rl.rlim_cur as c_long as i64, rl.rlim_max as c_long as i64))
}

#[allow(clippy::unnecessary_cast)]
fn collect_rusage() -> Option<Rusage> {
    let mut ru = CRusage {
        ru_utime: CTimeval { tv_sec: 0, tv_usec: 0 },
        ru_stime: CTimeval { tv_sec: 0, tv_usec: 0 },
        ru_maxrss: 0,
        ru_ixrss: 0,
        ru_idrss: 0,
        ru_isrss: 0,
        ru_minflt: 0,
        ru_majflt: 0,
        ru_nswap: 0,
        ru_inblock: 0,
        ru_oublock: 0,
        ru_msgsnd: 0,
        ru_msgrcv: 0,
        ru_nsignals: 0,
        ru_nvcsw: 0,
        ru_nivcsw: 0,
    };
    // SAFETY: `ru` is valid out storage for one `struct rusage`.
    let rc = unsafe { getrusage(RUSAGE_SELF, &mut ru) };
    if rc != 0 {
        return None;
    }
    Some(Rusage {
        user: timeval_seconds(&ru.ru_utime),
        system: timeval_seconds(&ru.ru_stime),
        maxrss: ru.ru_maxrss as i64,
        minflt: ru.ru_minflt as i64,
        majflt: ru.ru_majflt as i64,
        nvcsw: ru.ru_nvcsw as i64,
        nivcsw: ru.ru_nivcsw as i64,
        core_limit: rlimit_pair(RLIMIT_CORE)?,
        nofile_limit: rlimit_pair(RLIMIT_NOFILE)?,
    })
}

fn read_proc_status() -> Option<String> {
    let file = std::fs::File::open("/proc/self/status").ok()?;
    let mut bytes = Vec::new();
    file.take(8192).read_to_end(&mut bytes).ok()?;
    Some(String::from_utf8_lossy(&bytes).into_owned())
}

/// The live collector behind [`Printer::debug_host_state`].
pub fn collect_host_info(reactor: &Reactor) -> HostInfo {
    let proc_status = read_proc_status();
    let threads = proc_status
        .as_deref()
        .and_then(|s| {
            s.lines()
                .find_map(|l| l.strip_prefix("Threads:"))
                .and_then(|v| v.trim().parse::<i64>().ok())
        })
        .unwrap_or(1);
    HostInfo {
        pid: i64::from(std::process::id()),
        wall_time: kos::time::wall_time(),
        monotonic: reactor.monotonic(),
        python_version: PYTHON_VERSION.to_string(),
        platform: PLATFORM.to_string(),
        gc_count: (0, 0, 0),
        gc_times: reactor.get_gc_stats(),
        threads,
        rusage: collect_rusage(),
        proc_status,
    }
}

fn pair_text(pair: (i64, i64)) -> String {
    format!("({}, {})", pair.0, pair.1)
}

/// The `logging.error` lines of `_debug_host_state` in order
/// (`klippy/klippy.py:220-254`), including the closing
/// `host Python thread stacks to stderr` line.
pub fn host_state_lines(
    info: &HostInfo,
    in_shutdown_state: bool,
    run_result: Option<&str>,
) -> Vec<String> {
    let mut lines = Vec::new();
    let gc_count = PyValue::Tuple(vec![
        PyValue::Int(PyInt::from_i64(info.gc_count.0)),
        PyValue::Int(PyInt::from_i64(info.gc_count.1)),
        PyValue::Int(PyInt::from_i64(info.gc_count.2)),
    ])
    .repr();
    let gc_times = PyValue::Tuple(info.gc_times.iter().map(|t| PyValue::Float(*t)).collect()).repr();
    lines.push(fmt(
        "[DEBUG-motion] host pid=%d walltime=%.6f monotonic=%.6f shutdown=%s run_result=%s threads=%d gc_count=%s gc_times=%s",
        &[
            pct_int(info.pid),
            PctValue::Float(info.wall_time),
            PctValue::Float(info.monotonic),
            PctValue::Bool(in_shutdown_state),
            PctValue::Str(run_result.unwrap_or("None").to_string()),
            pct_int(info.threads),
            PctValue::Str(gc_count),
            PctValue::Str(gc_times),
        ],
    ));
    lines.push(fmt(
        "[DEBUG-motion] host runtime python=%r platform=%s",
        &[
            PctValue::Str(info.python_version.chars().take(512).collect()),
            PctValue::Str(info.platform.clone()),
        ],
    ));
    match &info.rusage {
        None => lines.push("[DEBUG-motion] host resource snapshot unavailable".to_string()),
        Some(usage) => lines.push(fmt(
            "[DEBUG-motion] host resources user=%.6f system=%.6f maxrss=%d minflt=%d majflt=%d nvcsw=%d nivcsw=%d core_limit=%s nofile_limit=%s",
            &[
                PctValue::Float(usage.user),
                PctValue::Float(usage.system),
                pct_int(usage.maxrss),
                pct_int(usage.minflt),
                pct_int(usage.majflt),
                pct_int(usage.nvcsw),
                pct_int(usage.nivcsw),
                PctValue::Str(pair_text(usage.core_limit)),
                PctValue::Str(pair_text(usage.nofile_limit)),
            ],
        )),
    }
    match &info.proc_status {
        None => lines.push("[DEBUG-motion] host proc_status unavailable".to_string()),
        Some(data) => {
            let fields = [
                "VmPeak:", "VmSize:", "VmHWM:", "VmRSS:", "VmSwap:", "Threads:", "FDSize:",
            ];
            let status: Vec<String> = data
                .lines()
                .filter(|line| fields.iter().any(|f| line.starts_with(f)))
                .map(|line| line.chars().take(160).collect())
                .collect();
            lines.push(fmt(
                "[DEBUG-motion] host proc_status: %s",
                &[PctValue::Str(status.join("; "))],
            ));
        }
    }
    lines.push("[DEBUG-motion] host Python thread stacks to stderr".to_string());
    lines
}

// ---------------------------------------------------------------------------
// Error classification for `_connect` (`klippy/klippy.py:141-163`).
// ---------------------------------------------------------------------------

/// Which `except` clause of `_connect` a [`KlippyError`] lands in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConnectErrorKind {
    /// `except (self.config_error, pins.error)`.
    Config,
    /// `except msgproto.error`.
    Protocol,
    /// `except mcu.error`.
    Mcu,
    /// `except Exception`.
    Internal,
}

/// The `except` clause for `err`, in the order of the original.
/// Exception identity is retained even when distinct Python classes have
/// the same name (`pins.error`, `msgproto.error`, `mcu.error`).
pub fn classify_connect_error(err: &KlippyError) -> ConnectErrorKind {
    match err {
        KlippyError::Config(_) | KlippyError::Pins(_) => ConnectErrorKind::Config,
        KlippyError::Protocol(_) => ConnectErrorKind::Protocol,
        KlippyError::Mcu(_) => ConnectErrorKind::Mcu,
        _ => ConnectErrorKind::Internal,
    }
}

// ---------------------------------------------------------------------------
// The Printer object.
// ---------------------------------------------------------------------------

type Handler = Arc<GCell<EventCallback>>;
type HandlerList = Arc<GCell<Vec<Handler>>>;

struct PrinterState {
    start_args: IndexMap<String, PyValue>,
    state_message: String,
    in_shutdown_state: bool,
    run_result: Option<String>,
    debug_host_dumped: bool,
    event_handlers: IndexMap<String, HandlerList>,
    /// `collections.OrderedDict`. `None` values are modules whose
    /// factory returned `None`.
    objects: IndexMap<String, Option<ObjectRef>>,
}

/// The `Printer` object (`klippy/klippy.py:25`).
pub struct Printer {
    reactor: Reactor,
    state: GCell<PrinterState>,
    env: Arc<HostEnv>,
    bglogger: Option<Arc<QueueListener>>,
    self_ref: Weak<Printer>,
}

fn call_handler(cb: &Handler, params: &[EventArg]) -> KResult<PyValue> {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut guard = cb.borrow_mut();
        (*guard)(params)
    }));
    match result {
        Ok(r) => r,
        Err(payload) => Err(panic_error(payload)),
    }
}

fn handler_at(list: &HandlerList, index: usize) -> Option<Handler> {
    list.borrow().get(index).cloned()
}

impl Printer {
    /// `Printer.__init__(main_reactor, bglogger, start_args)`
    /// (`klippy/klippy.py:28`).
    pub fn new(
        reactor: Reactor,
        bglogger: Option<Arc<QueueListener>>,
        start_args: IndexMap<String, PyValue>,
        env: Arc<HostEnv>,
    ) -> KResult<Arc<Printer>> {
        let printer = Arc::new_cyclic(|weak| Printer {
            reactor,
            state: GCell::new(PrinterState {
                start_args,
                state_message: MESSAGE_STARTUP.to_string(),
                in_shutdown_state: false,
                run_result: None,
                debug_host_dumped: false,
                event_handlers: IndexMap::new(),
                objects: IndexMap::new(),
            }),
            env,
            bglogger,
            self_ref: weak.clone(),
        });
        let weak = printer.self_ref.clone();
        printer.reactor.register_callback(
            move |eventtime: f64| {
                if let Some(p) = weak.upgrade() {
                    if let Err(e) = p.connect(eventtime) {
                        panic!("{}: {}", e.class_name(), e.message());
                    }
                }
            },
            NOW,
        );
        let weak = printer.self_ref.clone();
        printer.register_event_handler(
            EV_KLIPPY_ANALYZE_SHUTDOWN,
            Box::new(move |_params: &[EventArg]| {
                if let Some(p) = weak.upgrade() {
                    p.debug_host_state();
                }
                Ok(PyValue::None)
            }),
        );
        let api = printer.api();
        for name in [MODULE_GCODE, MODULE_WEBHOOKS] {
            if let Some(create) = printer
                .env
                .modules
                .get(name)
                .and_then(|f| f.add_early_printer_objects)
            {
                create(&api)?;
            }
        }
        Ok(printer)
    }

    /// The `Arc<dyn PrinterApi>` handle the module entry points take.
    fn api(&self) -> klippy_api::printer::Printer {
        let arc: Arc<Printer> = self.self_ref.upgrade().expect("Printer lives in an Arc");
        arc
    }

    /// `self.state_message`.
    pub fn state_message(&self) -> String {
        self.state.borrow().state_message.clone()
    }

    /// `self.run_result`.
    pub fn run_result(&self) -> Option<String> {
        self.state.borrow().run_result.clone()
    }

    /// `self.debug_host_dumped`.
    pub fn debug_host_dumped(&self) -> bool {
        self.state.borrow().debug_host_dumped
    }

    /// `name in self.objects`.
    pub fn has_object(&self, name: &str) -> bool {
        self.state.borrow().objects.contains_key(name)
    }

    /// `self.start_args[key] = value`.
    pub fn start_args_set(&self, key: &str, value: PyValue) {
        self.state.borrow_mut().start_args.insert(key.to_string(), value);
    }

    fn handler_list(&self, event: &str) -> Option<HandlerList> {
        self.state.borrow().event_handlers.get(event).cloned()
    }

    /// `_read_config` (`klippy/klippy.py:119`).
    pub fn read_config(&self) -> KResult<()> {
        let api = self.api();
        if let Some(create) = self
            .env
            .modules
            .get(MODULE_CONFIGFILE)
            .and_then(|f| f.add_early_printer_objects)
        {
            create(&api)?;
        }
        let pconfig = lookup_interface::<dyn PrinterConfigApi>(self, "configfile", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let config = pconfig.read_main_config()?;
        if self.bglogger.is_some() {
            pconfig.log_config(&config);
        }
        for name in [MODULE_PINS, MODULE_MCU] {
            self.add_printer_objects(name, &config)?;
        }
        for section_config in config.get_prefix_sections("") {
            self.load_object(&config, &section_config.get_name(), LookupDefault::ReturnNone)?;
        }
        self.add_printer_objects(MODULE_TOOLHEAD, &config)?;
        pconfig.check_unused_options(&config)
    }

    fn add_printer_objects(&self, module: &str, config: &Config) -> KResult<()> {
        let create = self
            .env
            .modules
            .get(module)
            .and_then(|f| f.add_printer_objects);
        match create {
            Some(create) => create(config),
            None => Ok(()),
        }
    }

    /// `_connect` (`klippy/klippy.py:133`). An `Err` is an exception
    /// that Python lets escape from the reactor callback.
    pub fn connect(&self, _eventtime: f64) -> KResult<()> {
        let first: KResult<bool> = (|| {
            self.read_config()?;
            self.send_event(EV_KLIPPY_MCU_IDENTIFY, &[])?;
            if let Some(list) = self.handler_list(EV_KLIPPY_CONNECT) {
                let mut i = 0;
                while let Some(cb) = handler_at(&list, i) {
                    if self.state_message() != MESSAGE_STARTUP {
                        return Ok(false);
                    }
                    call_handler(&cb, &[])?;
                    i += 1;
                }
            }
            Ok(true)
        })();
        match first {
            Ok(true) => {}
            Ok(false) => return Ok(()),
            Err(e) => return self.connect_failed(&e),
        }
        let ready: KResult<()> = {
            self.set_state(MESSAGE_READY);
            let _guard = self.reactor.assert_no_pause();
            (|| {
                if let Some(list) = self.handler_list(EV_KLIPPY_READY) {
                    let mut i = 0;
                    while let Some(cb) = handler_at(&list, i) {
                        if self.state_message() != MESSAGE_READY {
                            return Ok(());
                        }
                        call_handler(&cb, &[])?;
                        i += 1;
                    }
                }
                Ok(())
            })()
        };
        if let Err(e) = ready {
            log::exception("Unhandled exception during ready callback", &traceback_of(&e));
            self.invoke_shutdown_impl(
                &format!("Internal error during ready callback: {}", e.message()),
                empty_details(),
            );
        }
        Ok(())
    }

    /// The `except` clauses of `_connect` (`klippy/klippy.py:141-163`).
    fn connect_failed(&self, e: &KlippyError) -> KResult<()> {
        let tb = traceback_of(e);
        match classify_connect_error(e) {
            ConnectErrorKind::Config => {
                log::exception("Config error", &tb);
                self.set_state(&format!("{}\n{}", e.message(), MESSAGE_RESTART));
                Ok(())
            }
            ConnectErrorKind::Protocol => self.connect_mcu_failed("Protocol error", e, &tb),
            ConnectErrorKind::Mcu => self.connect_mcu_failed("MCU error during connect", e, &tb),
            ConnectErrorKind::Internal => {
                log::exception("Unhandled exception during connect", &tb);
                self.set_state(&format!(
                    "Internal error during connect: {}\n{}",
                    e.message(),
                    MESSAGE_RESTART
                ));
                Ok(())
            }
        }
    }

    fn connect_mcu_failed(&self, msg: &str, e: &KlippyError, tb: &str) -> KResult<()> {
        log::exception(msg, tb);
        self.set_state(msg);
        let mut details = IndexMap::new();
        details.insert(
            PyValue::Str("error".to_string()),
            PyValue::Str(e.message().to_string()),
        );
        self.send_event(
            EV_KLIPPY_NOTIFY_MCU_ERROR,
            &[
                EventArg::Value(PyValue::Str(msg.to_string())),
                EventArg::Value(PyValue::Dict(details)),
            ],
        )?;
        util::dump_mcu_build(&py_join(&self.env.klippy_dir, ".."));
        Ok(())
    }

    /// `_set_state` (`klippy/klippy.py:60`).
    pub fn set_state(&self, msg: &str) {
        {
            let mut st = self.state.borrow_mut();
            if st.state_message == MESSAGE_READY || st.state_message == MESSAGE_STARTUP {
                st.state_message = msg.to_string();
            }
        }
        if msg != MESSAGE_READY && !self.is_shutdown() {
            self.debug_host_state();
        }
        let debuginput = !matches!(
            self.state.borrow().start_args.get("debuginput"),
            None | Some(PyValue::None)
        );
        if msg != MESSAGE_READY && debuginput {
            self.request_exit("error_exit");
        }
    }

    /// `run` (`klippy/klippy.py:175`).
    pub fn run(&self) -> Option<String> {
        let systime = kos::time::wall_time();
        let monotime = self.reactor.monotonic();
        log::info(&start_printer_line(systime, monotime));
        // Enter main reactor loop
        if let Err(tb) = self.run_reactor() {
            let msg = "Unhandled exception during run";
            log::exception(msg, &tb);
            // Exception from a reactor callback - try to shutdown
            let weak = self.self_ref.clone();
            self.reactor.register_callback(
                move |_eventtime: f64| {
                    if let Some(p) = weak.upgrade() {
                        p.invoke_shutdown_impl(msg, empty_details());
                    }
                },
                NOW,
            );
            if let Err(tb) = self.run_reactor() {
                log::exception("Repeat unhandled exception during run", &tb);
                // Another exception - try to exit
                self.state.borrow_mut().run_result = Some("error_exit".to_string());
                self.debug_host_state();
            }
        }
        // Check restart flags
        let run_result = self.run_result();
        let post: KResult<()> = (|| {
            if run_result.as_deref() == Some("firmware_restart") {
                self.send_event(EV_KLIPPY_FIRMWARE_RESTART, &[])?;
            }
            self.send_event(EV_KLIPPY_DISCONNECT, &[])?;
            Ok(())
        })();
        if let Err(e) = post {
            log::exception("Unhandled exception during post run", &traceback_of(&e));
            self.debug_host_state();
        }
        run_result
    }

    /// `self.reactor.run()` as a `try` body: the text of the exception
    /// that left it, if any.
    fn run_reactor(&self) -> Result<(), String> {
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.reactor.run()))
            .map_err(|payload| traceback_of(&panic_error(payload)))
    }

    /// `_debug_host_state` (`klippy/klippy.py:213`).
    pub fn debug_host_state(&self) {
        // One bounded snapshot per printer instance, never on the
        // motion path.
        {
            let mut st = self.state.borrow_mut();
            if st.debug_host_dumped {
                return;
            }
            st.debug_host_dumped = true;
        }
        let dumped = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let info = collect_host_info(&self.reactor);
            let (shutdown, run_result) = {
                let st = self.state.borrow();
                (st.in_shutdown_state, st.run_result.clone())
            };
            for line in host_state_lines(&info, shutdown, run_result.as_deref()) {
                log::error(&line);
            }
            // faulthandler.dump_traceback(file=sys.stderr, all_threads=True):
            // the host has no Python frames to list, so only the dumping
            // thread is named (stderr is not part of the log contract).
            eprintln!(
                "Current thread {:?} (most recent call first):\n  <no Python frames in the Rust host>",
                std::thread::current().id()
            );
        }));
        if let Err(payload) = dumped {
            // Diagnostics must not replace the original error or shutdown.
            log::exception(
                "[DEBUG-motion] Unable to dump host diagnostics",
                &traceback_of(&panic_error(payload)),
            );
        }
    }

    /// `invoke_shutdown` (`klippy/klippy.py:259`).
    fn invoke_shutdown_impl(&self, msg: &str, details: PyValue) {
        if self.is_shutdown() {
            return;
        }
        log::error(&fmt(
            "Transition to shutdown state: %s",
            &[PctValue::Str(msg.to_string())],
        ));
        self.state.borrow_mut().in_shutdown_state = true;
        self.set_state(msg);
        let _guard = self.reactor.assert_no_pause();
        self.run_shutdown_handlers(EV_KLIPPY_SHUTDOWN, "Exception during shutdown handler", &[]);
        self.run_shutdown_handlers(
            EV_KLIPPY_ANALYZE_SHUTDOWN,
            "Exception in analyze_shutdown handler",
            &[
                EventArg::Value(PyValue::Str(msg.to_string())),
                EventArg::Value(details),
            ],
        );
    }

    /// One shutdown handler list, each handler in its own bare `try`.
    fn run_shutdown_handlers(&self, event: &str, log_line: &str, params: &[EventArg]) {
        let Some(list) = self.handler_list(event) else {
            return;
        };
        let mut i = 0;
        while let Some(cb) = handler_at(&list, i) {
            if let Err(e) = call_handler(&cb, params) {
                log::exception(log_line, &traceback_of(&e));
            }
            i += 1;
        }
    }
}

/// The `Start printer at ...` line of `run` (`klippy/klippy.py:178`).
pub fn start_printer_line(systime: f64, monotime: f64) -> String {
    fmt(
        "Start printer at %s (%.1f %.1f)",
        &[
            PctValue::Str(util::asctime_from_mtime(systime).unwrap_or_default()),
            PctValue::Float(systime),
            PctValue::Float(monotime),
        ],
    )
}

// ---------------------------------------------------------------------------
// PrinterApi (`klippy/klippy.py:25`).
// ---------------------------------------------------------------------------

impl PrinterApi for Printer {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(
            self.state
                .borrow()
                .start_args
                .iter()
                .map(|(k, v)| (PyValue::Str(k.clone()), v.clone()))
                .collect(),
        )
    }

    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }

    fn get_state_message(&self) -> (String, String) {
        let message = self.state_message();
        let category = if message == MESSAGE_READY {
            "ready"
        } else if message == MESSAGE_STARTUP {
            "startup"
        } else if self.is_shutdown() {
            "shutdown"
        } else {
            "error"
        };
        (message, category.to_string())
    }

    fn is_shutdown(&self) -> bool {
        self.state.borrow().in_shutdown_state
    }

    fn update_error_msg(&self, old_msg: &str, new_msg: &str) {
        {
            let mut st = self.state.borrow_mut();
            let is_boot = |m: &str| m == MESSAGE_READY || m == MESSAGE_STARTUP;
            if st.state_message != old_msg || is_boot(&st.state_message) || is_boot(new_msg) {
                return;
            }
            st.state_message = new_msg.to_string();
        }
        log::error(new_msg);
    }

    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut st = self.state.borrow_mut();
        if st.objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        st.objects.insert(name.to_string(), Some(obj));
        Ok(())
    }

    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let stored = self.state.borrow().objects.get(name).cloned();
        match stored {
            Some(obj) => Ok(obj),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }

    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let st = self.state.borrow();
        let live = |name: &String, obj: &Option<ObjectRef>| obj.clone().map(|o| (name.clone(), o));
        match module {
            None => st.objects.iter().filter_map(|(n, o)| live(n, o)).collect(),
            Some(module) => {
                let prefix = format!("{module} ");
                let mut out = Vec::new();
                if let Some((n, o)) = st.objects.get_key_value(module) {
                    out.extend(live(n, o));
                }
                out.extend(
                    st.objects
                        .iter()
                        .filter(|(n, _)| n.starts_with(&prefix))
                        .filter_map(|(n, o)| live(n, o)),
                );
                out
            }
        }
    }

    fn load_object(
        &self,
        config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let stored = self.state.borrow().objects.get(section).cloned();
        if let Some(obj) = stored {
            return Ok(obj);
        }
        let mut parts = section.split(is_py_space).filter(|p| !p.is_empty());
        let Some(module_name) = parts.next() else {
            // `module_parts[0]` of an empty split.
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            });
        };
        let is_prefix = parts.next().is_some();
        let init = self
            .env
            .modules
            .get(&format!("extras.{module_name}"))
            .and_then(|f| {
                if is_prefix {
                    f.load_config_prefix
                } else {
                    f.load_config
                }
            });
        let Some(init) = init else {
            return match default {
                LookupDefault::ReturnNone => Ok(None),
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unable to load module '{section}'"
                ))),
            };
        };
        let obj = init(&config.getsection(section))?;
        self.state
            .borrow_mut()
            .objects
            .insert(section.to_string(), obj.clone());
        Ok(obj)
    }

    fn run(&self) -> Option<String> {
        Printer::run(self)
    }

    fn set_rollover_info(&self, name: &str, info: Option<&str>, log_info: bool) {
        if log_info {
            log::info(info.unwrap_or("None"));
        }
        if let Some(bg) = &self.bglogger {
            bg.set_rollover_info(name, info);
        }
    }

    fn invoke_shutdown(&self, msg: &str, details: PyValue) {
        self.invoke_shutdown_impl(msg, details);
    }

    fn invoke_async_shutdown(&self, msg: &str, details: PyValue) {
        let weak = self.self_ref.clone();
        let msg = msg.to_string();
        self.reactor.register_async_callback(
            move |_eventtime: f64| {
                if let Some(p) = weak.upgrade() {
                    p.invoke_shutdown_impl(&msg, details.clone());
                }
            },
            NOW,
        );
    }

    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        let list = {
            let mut st = self.state.borrow_mut();
            st.event_handlers
                .entry(event.to_string())
                .or_insert_with(|| Arc::new(GCell::new(Vec::new())))
                .clone()
        };
        list.borrow_mut().push(Arc::new(GCell::new(callback)));
    }

    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let Some(list) = self.handler_list(event) else {
            return Ok(results);
        };
        let mut i = 0;
        while let Some(cb) = handler_at(&list, i) {
            results.push(call_handler(&cb, params)?);
            i += 1;
        }
        Ok(results)
    }

    fn request_exit(&self, result: &str) {
        {
            let mut st = self.state.borrow_mut();
            if st.run_result.is_none() {
                st.run_result = Some(result.to_string());
            }
        }
        self.reactor.end();
    }
}

// ---------------------------------------------------------------------------
// Startup (`klippy/klippy.py:293-329`).
// ---------------------------------------------------------------------------

/// `arg_dictionary` (`klippy/klippy.py:308`): the `-d/--dictionary`
/// optparse callback applied to `parser.values.dictionary`.
pub fn arg_dictionary(dictionary: &mut IndexMap<String, String>, value: &str) {
    let (key, fname) = match value.split_once('=') {
        Some((mcu_name, fname)) => (format!("dictionary_{mcu_name}"), fname.to_string()),
        None => ("dictionary".to_string(), value.to_string()),
    };
    dictionary.insert(key, fname);
}

/// Python's `open()` failure as the exception the traceback ends in.
fn io_error(path: &str, e: &std::io::Error) -> KlippyError {
    let errno = e.raw_os_error().unwrap_or(0);
    let class_name = match errno {
        1 | 13 => "PermissionError",
        2 => "FileNotFoundError",
        17 => "FileExistsError",
        20 => "NotADirectoryError",
        21 => "IsADirectoryError",
        _ => "OSError",
    };
    let text = e.to_string();
    let strerror = text
        .strip_suffix(&format!(" (os error {errno})"))
        .unwrap_or(&text);
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: format!(
            "[Errno {errno}] {strerror}: {}",
            PyValue::Str(path.to_string()).repr()
        ),
    }
}

/// The module names `import_test` imports, in listing order
/// (`klippy/klippy.py:293-305`).
pub fn import_test_names(klippy_dir: &str) -> KResult<Vec<String>> {
    let mut names = Vec::new();
    for mname in ["extras", "kinematics"] {
        let dir = py_join(klippy_dir, mname);
        let entries = std::fs::read_dir(&dir).map_err(|e| io_error(&dir, &e))?;
        for entry in entries {
            let entry = entry.map_err(|e| io_error(&dir, &e))?;
            let fname = entry.file_name().to_string_lossy().into_owned();
            let module_name = if fname.ends_with(".py") && fname != "__init__.py" {
                fname[..fname.len() - 3].to_string()
            } else {
                let iname = entry.path().join("__init__.py");
                if !iname.exists() {
                    continue;
                }
                fname
            };
            names.push(format!("{mname}.{module_name}"));
        }
    }
    Ok(names)
}

/// `import_test` (`klippy/klippy.py:293`): import every optional
/// module, then `sys.exit(0)`. A name missing from the registry is the
/// `ImportError` that ends the original with a traceback.
pub fn import_test(env: &HostEnv) -> MainExit {
    let names = match import_test_names(&env.klippy_dir) {
        Ok(names) => names,
        Err(e) => return exception_exit(&e),
    };
    for name in names {
        if !env.modules.contains_key(&name) {
            return exception_exit(&KlippyError::Internal {
                class_name: "ModuleNotFoundError".to_string(),
                message: format!("No module named '{name}'"),
            });
        }
    }
    MainExit {
        code: 0,
        stdout: String::new(),
        stderr: String::new(),
    }
}

static FAULT_HANDLER_ENABLED: AtomicBool = AtomicBool::new(false);

/// `setup_fault_handler` (`klippy/klippy.py:317`). The process-wide
/// flag stands for `faulthandler.is_enabled()`.
pub fn setup_fault_handler() {
    if FAULT_HANDLER_ENABLED.swap(true, Ordering::SeqCst) {
        log::info("[DEBUG-motion] Existing faulthandler left enabled");
        return;
    }
    log::info("[DEBUG-motion] Fatal Python thread dumps enabled on stderr");
}

// ---------------------------------------------------------------------------
// optparse (`klippy/klippy.py:332-351`).
// ---------------------------------------------------------------------------

/// How a process ends: exit status plus what it wrote.
#[derive(Clone, Debug, PartialEq)]
pub struct MainExit {
    /// Exit status (`sys.exit(-1)` is `-1`; an uncaught exception is 1).
    pub code: i32,
    /// Text for stdout.
    pub stdout: String,
    /// Text for stderr.
    pub stderr: String,
}

fn exception_exit(e: &KlippyError) -> MainExit {
    MainExit {
        code: 1,
        stdout: String::new(),
        stderr: format!("{}\n", traceback_of(e)),
    }
}

/// Parsed options of `main()`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MainOptions {
    /// `-i/--debuginput`.
    pub debuginput: Option<String>,
    /// `-I/--input-tty`.
    pub inputtty: String,
    /// `-a/--api-server`.
    pub apiserver: Option<String>,
    /// `-l/--logfile`.
    pub logfile: Option<String>,
    /// `-v`.
    pub verbose: bool,
    /// `-o/--debugoutput`.
    pub debugoutput: Option<String>,
    /// `-d/--dictionary`; `None` until the first `-d`
    /// (`parser.values.dictionary is None`, `klippy/klippy.py:313`).
    pub dictionary: Option<IndexMap<String, String>>,
    /// `--import-test`.
    pub import_test: bool,
}

/// What `opts.parse_args()` returns.
#[derive(Clone, Debug, PartialEq)]
pub struct ParsedArgs {
    /// `options`.
    pub options: MainOptions,
    /// `args`.
    pub args: Vec<String>,
}

#[derive(Clone, Copy, PartialEq)]
enum Opt {
    Help,
    Debuginput,
    InputTty,
    ApiServer,
    Logfile,
    Verbose,
    Debugoutput,
    Dictionary,
    ImportTest,
}

impl Opt {
    fn takes_value(self) -> bool {
        !matches!(self, Opt::Help | Opt::Verbose | Opt::ImportTest)
    }
}

static LONG_OPTS: [(&str, Opt); 8] = [
    ("--help", Opt::Help),
    ("--debuginput", Opt::Debuginput),
    ("--input-tty", Opt::InputTty),
    ("--api-server", Opt::ApiServer),
    ("--logfile", Opt::Logfile),
    ("--debugoutput", Opt::Debugoutput),
    ("--dictionary", Opt::Dictionary),
    ("--import-test", Opt::ImportTest),
];

fn short_opt(c: char) -> Option<Opt> {
    Some(match c {
        'h' => Opt::Help,
        'i' => Opt::Debuginput,
        'I' => Opt::InputTty,
        'a' => Opt::ApiServer,
        'l' => Opt::Logfile,
        'v' => Opt::Verbose,
        'o' => Opt::Debugoutput,
        'd' => Opt::Dictionary,
        _ => return None,
    })
}

/// `format_option_help` of the parser built in `main()` at the default
/// 78 column width (`COLUMNS` unset).
const OPTIONS_HELP: &str = "Options:\n  -h, --help            show this help message and exit\n  -i DEBUGINPUT, --debuginput=DEBUGINPUT\n                        read commands from file instead of from tty port\n  -I INPUTTTY, --input-tty=INPUTTTY\n                        input tty name (default is /tmp/printer)\n  -a APISERVER, --api-server=APISERVER\n                        api server unix domain socket filename\n  -l LOGFILE, --logfile=LOGFILE\n                        write log to file instead of stderr\n  -v                    enable debug messages\n  -o DEBUGOUTPUT, --debugoutput=DEBUGOUTPUT\n                        write output to file instead of to serial port\n  -d DICTIONARY, --dictionary=DICTIONARY\n                        file to read for mcu protocol dictionary\n  --import-test         perform an import module test\n";

/// `os.path.basename(sys.argv[0])`, the `%prog` of optparse.
fn prog_name(argv: &[String]) -> String {
    let arg0 = argv.first().map(String::as_str).unwrap_or("");
    arg0.rsplit('/').next().unwrap_or("").to_string()
}

fn usage_text(prog: &str) -> String {
    format!("Usage: {}\n", USAGE.replace("%prog", prog))
}

/// `OptionParser.error(msg)`: usage, a blank line and the message on
/// stderr, exit status 2.
fn parser_error(prog: &str, msg: &str) -> MainExit {
    MainExit {
        code: 2,
        stdout: String::new(),
        stderr: format!("{}\n{}: error: {}\n", usage_text(prog), prog, msg),
    }
}

/// `-h`: `print_help()` then `exit()`.
fn parser_help(prog: &str) -> MainExit {
    MainExit {
        code: 0,
        stdout: format!("{}\n{}", usage_text(prog), OPTIONS_HELP),
        stderr: String::new(),
    }
}

/// `optparse._match_abbrev` over the long options.
fn match_long_opt(typed: &str) -> Result<(&'static str, Opt), String> {
    if let Some(found) = LONG_OPTS.iter().find(|(name, _)| *name == typed) {
        return Ok(*found);
    }
    let mut possibilities: Vec<&(&str, Opt)> = LONG_OPTS
        .iter()
        .filter(|(name, _)| name.starts_with(typed))
        .collect();
    match possibilities.len() {
        1 => Ok(*possibilities[0]),
        0 => Err(format!("no such option: {typed}")),
        _ => {
            possibilities.sort_by(|a, b| a.0.cmp(b.0));
            let names: Vec<&str> = possibilities.iter().map(|p| p.0).collect();
            Err(format!("ambiguous option: {typed} ({}?)", names.join(", ")))
        }
    }
}

/// `option.process`: store the value, or run the `-d` callback.
fn process_option(options: &mut MainOptions, opt: Opt, value: Option<String>) -> bool {
    let value = value.unwrap_or_default();
    match opt {
        Opt::Help => return true,
        Opt::Debuginput => options.debuginput = Some(value),
        Opt::InputTty => options.inputtty = value,
        Opt::ApiServer => options.apiserver = Some(value),
        Opt::Logfile => options.logfile = Some(value),
        Opt::Verbose => options.verbose = true,
        Opt::Debugoutput => options.debugoutput = Some(value),
        Opt::Dictionary => {
            arg_dictionary(options.dictionary.get_or_insert_with(IndexMap::new), &value)
        }
        Opt::ImportTest => options.import_test = true,
    }
    false
}

/// `opts.parse_args(argv[1:])` for the parser of `main()`
/// (`klippy/klippy.py:332-351`), including optparse's error texts,
/// long option abbreviations and `-h`.
pub fn parse_cli(argv: &[String]) -> Result<ParsedArgs, MainExit> {
    let prog = prog_name(argv);
    let mut options = MainOptions {
        inputtty: "/tmp/printer".to_string(),
        ..Default::default()
    };
    let mut rargs: std::collections::VecDeque<String> = argv.iter().skip(1).cloned().collect();
    let mut largs: Vec<String> = Vec::new();
    let requires = |opt: &str| parser_error(&prog, &format!("{opt} option requires 1 argument"));
    while let Some(arg) = rargs.front().cloned() {
        if arg == "--" {
            rargs.pop_front();
            break;
        } else if arg.starts_with("--") {
            rargs.pop_front();
            let (typed, explicit) = match arg.split_once('=') {
                Some((opt, value)) => (opt.to_string(), Some(value.to_string())),
                None => (arg.clone(), None),
            };
            let (name, opt) = match match_long_opt(&typed) {
                Ok(found) => found,
                Err(msg) => return Err(parser_error(&prog, &msg)),
            };
            let value = if opt.takes_value() {
                match explicit {
                    Some(value) => Some(value),
                    None => match rargs.pop_front() {
                        Some(value) => Some(value),
                        None => return Err(requires(name)),
                    },
                }
            } else if explicit.is_some() {
                return Err(parser_error(
                    &prog,
                    &format!("{name} option does not take a value"),
                ));
            } else {
                None
            };
            if process_option(&mut options, opt, value) {
                return Err(parser_help(&prog));
            }
        } else if arg.starts_with('-') && arg.chars().count() > 1 {
            rargs.pop_front();
            let chars: Vec<char> = arg.chars().collect();
            let mut i = 1;
            while i < chars.len() {
                let name = format!("-{}", chars[i]);
                let Some(opt) = short_opt(chars[i]) else {
                    return Err(parser_error(&prog, &format!("no such option: {name}")));
                };
                i += 1;
                let mut value = None;
                let mut stop = false;
                if opt.takes_value() {
                    if i < chars.len() {
                        rargs.push_front(chars[i..].iter().collect());
                        stop = true;
                    }
                    match rargs.pop_front() {
                        Some(v) => value = Some(v),
                        None => return Err(requires(&name)),
                    }
                }
                if process_option(&mut options, opt, value) {
                    return Err(parser_help(&prog));
                }
                if stop {
                    break;
                }
            }
        } else {
            largs.push(arg);
            rargs.pop_front();
        }
    }
    largs.extend(rargs);
    Ok(ParsedArgs { options, args: largs })
}

// ---------------------------------------------------------------------------
// The git and versions blocks of `main()` (`klippy/klippy.py:379-414`).
// ---------------------------------------------------------------------------

fn capped_names(mut names: Vec<String>) -> String {
    if names.len() > 10 {
        let extra = names.len() - 10;
        names.truncate(10);
        names.push(format!("(+{extra} files)"));
    }
    names.join(", ")
}

/// `(software_version, extra_git_desc)` from `util.get_git_version()`.
/// An entry of `file_status` that is not a `(code, fname)` pair is the
/// `ValueError` of the tuple unpacking in the original.
pub fn git_version_block(info: &util::GitInfo) -> KResult<(String, String)> {
    let mut pairs: Vec<(&str, &str)> = Vec::new();
    for entry in &info.file_status {
        if entry.len() != 2 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: if entry.len() < 2 {
                    format!("not enough values to unpack (expected 2, got {})", entry.len())
                } else {
                    "too many values to unpack (expected 2)".to_string()
                },
            });
        }
        pairs.push((entry[0].as_str(), entry[1].as_str()));
    }
    let extra_files: Vec<String> = pairs
        .iter()
        .filter(|(code, fname)| {
            (*code == "??" || *code == "!!")
                && fname.ends_with(".py")
                && (fname.starts_with("klippy/kinematics/") || fname.starts_with("klippy/extras/"))
        })
        .map(|(_, fname)| fname.to_string())
        .collect();
    let modified_files: Vec<String> = pairs
        .iter()
        .filter(|(code, _)| *code == "M")
        .map(|(_, fname)| fname.to_string())
        .collect();
    let mut git_vers = info.version.clone();
    let mut extra_git_desc = String::new();
    if !extra_files.is_empty() {
        if !git_vers.ends_with("-dirty") {
            git_vers.push_str("-dirty");
        }
        extra_git_desc += &format!("\nUntracked files: {}", capped_names(extra_files));
    }
    if !modified_files.is_empty() {
        extra_git_desc += &format!("\nModified files: {}", capped_names(modified_files));
    }
    extra_git_desc += &format!("\nBranch: {}", info.branch);
    extra_git_desc += &format!("\nRemote: {}", info.remote);
    extra_git_desc += &format!("\nTracked URL: {}", info.url);
    Ok((git_vers, extra_git_desc))
}

/// The `versions` text logged and stored as rollover info.
pub fn versions_block(
    sys_argv: &[String],
    software_version: &str,
    extra_git_desc: &str,
    cpu_info: &str,
    device: &str,
    linux_version: &str,
    python_version: &str,
) -> String {
    let argv_repr = PyValue::List(sys_argv.iter().map(|s| PyValue::Str(s.clone())).collect()).repr();
    [
        format!("Args: {argv_repr}"),
        format!(
            "Git version: {}{}",
            PyValue::Str(software_version.to_string()).repr(),
            extra_git_desc
        ),
        format!("CPU: {cpu_info}"),
        format!("Device: {device}"),
        format!("Linux: {linux_version}"),
        format!("Python: {}", PyValue::Str(python_version.to_string()).repr()),
    ]
    .join("\n")
}

// ---------------------------------------------------------------------------
// `main()` (`klippy/klippy.py:330-442`).
// ---------------------------------------------------------------------------

fn level_for(verbose: bool) -> log::Level {
    if verbose {
        log::Level::Debug
    } else {
        log::Level::Info
    }
}

/// `main()` with `argv` as `sys.argv`. `gc.disable()` and the per-run
/// `gc.collect()` have no counterpart here.
pub fn run_main(argv: &[String], env: Arc<HostEnv>) -> MainExit {
    // Modules resolve their bundled files from it, like Python's
    // `os.path.dirname(__file__)`.
    klippy_api::paths::set_klippy_dir(&env.klippy_dir);
    let ParsedArgs { options, args } = match parse_cli(argv) {
        Ok(parsed) => parsed,
        Err(exit) => return exit,
    };
    if options.import_test {
        return import_test(&env);
    }
    if args.len() != 1 {
        return parser_error(&prog_name(argv), "Incorrect number of arguments");
    }
    match start(argv, &options, &args[0], env) {
        Ok(code) => MainExit {
            code,
            stdout: String::new(),
            stderr: String::new(),
        },
        Err(e) => exception_exit(&e),
    }
}

/// Everything after the argument checks. `Ok` is the `sys.exit` status
/// (0 for a plain return), `Err` an uncaught exception.
fn start(
    argv: &[String],
    options: &MainOptions,
    config_file: &str,
    env: Arc<HostEnv>,
) -> KResult<i32> {
    let mut start_args: IndexMap<String, PyValue> = IndexMap::new();
    start_args.insert("config_file".to_string(), PyValue::Str(config_file.to_string()));
    start_args.insert(
        "apiserver".to_string(),
        options.apiserver.clone().map(PyValue::Str).unwrap_or(PyValue::None),
    );
    start_args.insert("start_reason".to_string(), PyValue::Str("startup".to_string()));

    let debuglevel = level_for(options.verbose);
    let truthy = |o: &Option<String>| o.as_deref().filter(|s| !s.is_empty()).map(str::to_string);
    if let Some(debuginput) = truthy(&options.debuginput) {
        start_args.insert("debuginput".to_string(), PyValue::Str(debuginput.clone()));
        let file = std::fs::File::open(&debuginput).map_err(|e| io_error(&debuginput, &e))?;
        // Python keeps the file object, and so the descriptor, for the
        // whole run.
        start_args.insert(
            "gcode_fd".to_string(),
            PyValue::Int(PyInt::from_i64(i64::from(file.into_raw_fd()))),
        );
    } else {
        let fd = util::create_pty(&options.inputtty).map_err(|e| KlippyError::Internal {
            class_name: e.kind.to_string(),
            message: e.msg.clone(),
        })?;
        start_args.insert("gcode_fd".to_string(), fd);
    }
    if let Some(debugoutput) = truthy(&options.debugoutput) {
        start_args.insert("debugoutput".to_string(), PyValue::Str(debugoutput));
        match &options.dictionary {
            Some(dictionary) => {
                for (key, fname) in dictionary {
                    start_args.insert(key.clone(), PyValue::Str(fname.clone()));
                }
            }
            None => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "'NoneType' object is not iterable".to_string(),
                })
            }
        }
    }
    let mut bglogger: Option<Arc<QueueListener>> = None;
    if let Some(logfile) = truthy(&options.logfile) {
        start_args.insert("log_file".to_string(), PyValue::Str(logfile.clone()));
        bglogger = Some(Arc::new(queuelogger::setup_bg_logging(&logfile, debuglevel)?));
    } else {
        queuelogger::root_set_level(debuglevel);
    }
    log::info("Starting Klippy...");
    setup_fault_handler();
    let git_info = util::get_git_version(&env.klippy_dir, true);
    let (git_vers, extra_git_desc) = git_version_block(&git_info)?;
    start_args.insert("software_version".to_string(), PyValue::Str(git_vers.clone()));
    let cpu_info = util::get_cpu_info();
    start_args.insert("cpu_info".to_string(), PyValue::Str(cpu_info.clone()));
    let device = util::get_device_info();
    start_args.insert("device".to_string(), PyValue::Str(device.clone()));
    let linux_version = util::get_linux_version();
    start_args.insert("linux_version".to_string(), PyValue::Str(linux_version.clone()));
    let mut versions = String::new();
    if bglogger.is_some() {
        versions = versions_block(
            argv,
            &git_vers,
            &extra_git_desc,
            &cpu_info,
            &device,
            &linux_version,
            PYTHON_VERSION,
        );
        log::info(&versions);
    } else if truthy(&options.debugoutput).is_none() {
        log::warning("No log file specified! Severe timing issues may result!");
    }

    // `chelper.get_ffi()`, first reached through `reactor.Reactor()`
    // (`reactor.py:116`), routes every C `errorf` to
    // `logging.error(FFI_main.string(msg))` (`chelper/__init__.py:308`):
    // the bytes repr at ERROR level.
    chelper_serial::set_logging_callback(|msg: &str| {
        log::error(&pycompat::text::bytes_repr(msg.as_bytes()));
    });

    // Start Printer() class
    let mut res: Option<String>;
    loop {
        if let Some(bg) = &bglogger {
            bg.clear_rollover_info();
            bg.set_rollover_info("versions", Some(&versions));
        }
        let main_reactor = Reactor::new(true);
        let printer = Printer::new(main_reactor.clone(), bglogger.clone(), start_args.clone(), env.clone())?;
        res = printer.run();
        if matches!(res.as_deref(), Some("exit") | Some("error_exit")) {
            break;
        }
        kos::time::sleep(1.0);
        main_reactor.finalize();
        drop(printer);
        log::info("Restarting printer");
        start_args.insert(
            "start_reason".to_string(),
            res.clone().map(PyValue::Str).unwrap_or(PyValue::None),
        );
    }
    if let Some(bg) = &bglogger {
        bg.stop();
    }
    Ok(if res.as_deref() == Some("error_exit") { -1 } else { 0 })
}

/// The frozen `klippy/klippy.py` is the host itself and has no
/// `load_config`, `load_config_prefix` or `add_printer_objects`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}

#[cfg(test)]
mod callback_error_tests {
    use super::*;

    #[test]
    fn callback_panic_traceback_matches_python() {
        let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
        for (class_name, message) in [
            ("ValueError", "callback failed"),
            ("ReactorError", "Internal error - reactor pause disabled"),
            ("error", "MCU callback failed"),
        ] {
            let expected = py.eval(&format!(
                "str(type({class_name:?}, (Exception,), {{}})({message:?}).__class__.__name__) + ': ' + {message:?}"
            )).expect("Python exception text");
            let actual = traceback_of(&panic_error(Box::new(format!("{class_name}: {message}"))));
            assert_eq!(expected, recording::Value::str(&actual));
        }
    }
}
