//! Port of the `statistics` module (`klippy/extras/statistics.py`).
//!
//! Two printer objects are created by [`load_config`]:
//!
//! - [`PrinterSysStats`] (object name `system_stats`): periodic host
//!   statistics read from `time.process_time()`, `os.getloadavg()` and
//!   `/proc/meminfo`. Its [`PrinterSysStats::stats`] always returns `(False,
//!   msg)`, so it only contributes to the log line, never enables it.
//!   Its `get_status` exposes `sysload` / `cputime` /
//!   `memavail`. It stops reading memory on `klippy:disconnect`, like the
//!   Python `mem_file = None`.
//!
//! - [`PrinterStats`] (the [`load_config`] return value, registered under the
//!   `[statistics]` section name): collects every printer object's
//!   `stats(eventtime)` callback on `klippy:ready` and once per second calls
//!   them in an `assert_no_pause` region, logging `Stats %.1f: %s` when at
//!   least one callback returns `True`, exactly like the Python aggregate.
//!
//! Host OS access (`time.process_time`, `os.getloadavg`, `/proc/meminfo`)
//! goes through the [`HostStats`] trait; [`SystemHostStats`] is the real
//! implementation. Tests inject a fake so both sides see identical values.
//!
//! Faithfulness notes (deliberate divergences):
//!
//! - `os.getloadavg` is called via a local `extern "C"` declaration because
//!   `kos` does not wrap it, following the `strerror` precedent in
//!   `chelper_serial`. `time.process_time` goes through
//!   [`kos::time::clock_gettime_f64`] with the Linux `CLOCK_PROCESS_CPUTIME_ID`
//!   clock id, which `kos::time` does not export.
//! - The memory file is re-read on every `stats` call instead of held open
//!   with a seek. `/proc/meminfo` is a procfs file that never disappears, so
//!   the two are indistinguishable except if the file were deleted after
//!   opening, which never happens.
//! - With an empty callback list the Python `max([])` would raise
//!   `ValueError`. That cannot happen in production because `system_stats`
//!   is always added first and always collected; this port simply skips the
//!   log line instead.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{ModuleFactory, Printer, EV_KLIPPY_DISCONNECT, EV_KLIPPY_READY};
use pycompat::text::{int_from_string, percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::Arc;

/// Stats callback collected by [`PrinterStats::handle_ready`]:
/// `stats(eventtime) -> (should_log, message)` in Python terms.
pub type StatsCallback = Arc<dyn Fn(f64) -> (bool, String) + Send + Sync>;

/// Host OS statistics source (`statistics.py stats`).
///
/// `process_time` is `time.process_time()`, `load_avg` is
/// `os.getloadavg()[0]`, and `meminfo` returns the whole `/proc/meminfo`
/// content (`None` when the file is unavailable or a read fails, which the
/// Python code treats as "skip the memory update").
pub trait HostStats: Send + Sync + 'static {
    /// `time.process_time()`.
    fn process_time(&self) -> f64;
    /// `os.getloadavg()[0]`.
    fn load_avg(&self) -> f64;
    /// Contents of `/proc/meminfo`, or `None` when unavailable.
    fn meminfo(&self) -> Option<String>;
}

/// The real [`HostStats`] implementation.
pub struct SystemHostStats;

// Linux asm-generic `<time.h>` clock id of the process CPU time, which
// `kos::time` does not export.
const CLOCK_PROCESS_CPUTIME_ID: i32 = 2;

impl HostStats for SystemHostStats {
    fn process_time(&self) -> f64 {
        kos::time::clock_gettime_f64(CLOCK_PROCESS_CPUTIME_ID)
    }

    fn load_avg(&self) -> f64 {
        let mut load = [0f64; 1];
        // SAFETY: glibc getloadavg, `load` has room for `nelem` doubles.
        let rc = unsafe { getloadavg(load.as_mut_ptr(), 1) };
        if rc != 1 {
            // Unreachable on Linux; `os.getloadavg` never fails here.
            return 0.;
        }
        load[0]
    }

    fn meminfo(&self) -> Option<String> {
        std::fs::read_to_string("/proc/meminfo").ok()
    }
}

// SAFETY: glibc `getloadavg`; the buffer is valid for `nelem` doubles.
unsafe extern "C" {
    fn getloadavg(loadavg: *mut f64, nelem: std::os::raw::c_int) -> std::os::raw::c_int;
}

/// Constant-format `%` helper (`percent_format` cannot fail on these).
fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant statistics format")
}

/// Mutable state of [`PrinterSysStats`], mirroring the Python instance dict.
struct SysStats {
    last_process_time: f64,
    total_process_time: f64,
    last_load_avg: f64,
    last_mem_avail: PyInt,
    /// `mem_file is not None`; cleared by the `klippy:disconnect` handler.
    mem_open: bool,
}

impl Default for SysStats {
    fn default() -> Self {
        SysStats {
            last_process_time: 0.,
            total_process_time: 0.,
            last_load_avg: 0.,
            last_mem_avail: PyInt::from_i64(0),
            mem_open: true,
        }
    }
}

/// Inner shared state of [`PrinterSysStats`].
struct SysInner {
    state: GCell<SysStats>,
    host: Arc<dyn HostStats>,
}

/// `PrinterSysStats` (`statistics.py:8`), registered as the `system_stats`
/// printer object.
pub struct PrinterSysStats {
    inner: Arc<SysInner>,
}

impl PrinterSysStats {
    /// `PrinterSysStats(config)` with the real [`SystemHostStats`].
    pub fn new(config: &Config) -> PrinterSysStats {
        PrinterSysStats::new_with_host(config, Arc::new(SystemHostStats))
    }

    /// `PrinterSysStats(config)` with an injected [`HostStats`].
    pub fn new_with_host(config: &Config, host: Arc<dyn HostStats>) -> PrinterSysStats {
        let printer = config.get_printer();
        // The disconnect handler captures the inner state, not the outer
        // object, so it can be registered before the object exists.
        let inner = Arc::new(SysInner {
            state: GCell::new(SysStats::default()),
            host,
        });
        let handler_inner = inner.clone();
        printer.register_event_handler(
            EV_KLIPPY_DISCONNECT,
            Box::new(move |_params| {
                handler_inner.disconnect();
                Ok(PyValue::None)
            }),
        );
        PrinterSysStats { inner }
    }

    /// `stats(eventtime)` (`statistics.py:24`).
    pub fn stats(&self, _eventtime: f64) -> (bool, String) {
        self.inner.stats()
    }
}

impl SysInner {
    /// `stats(eventtime)` (`statistics.py:24`): progress accumulation, load
    /// average and the optional `memavail` suffix.
    fn stats(&self) -> (bool, String) {
        let host = &self.host;
        let mut st = self.state.borrow_mut();
        let ptime = host.process_time();
        let pdiff = ptime - st.last_process_time;
        st.last_process_time = ptime;
        if pdiff > 0. {
            st.total_process_time += pdiff;
        }
        st.last_load_avg = host.load_avg();
        let mut msg = pct_format(
            "sysload=%.2f cputime=%.3f",
            &[
                PctValue::Float(st.last_load_avg),
                PctValue::Float(st.total_process_time),
            ],
        );
        if st.mem_open {
            if let Some(data) = host.meminfo() {
                for line in data.split('\n') {
                    if line.starts_with("MemAvailable:") {
                        let token = line.split_whitespace().nth(1);
                        let parsed = token
                            .and_then(|text| int_from_string(text, 10).ok());
                        // `int()` raising abandons the loop with no update,
                        // exactly like the Python `try/except: pass`.
                        if let Some(value) = parsed {
                            st.last_mem_avail = value;
                            msg = pct_format(
                                "%s memavail=%d",
                                &[
                                    PctValue::Str(msg),
                                    PctValue::Int(st.last_mem_avail.clone()),
                                ],
                            );
                        }
                        break;
                    }
                }
            }
        }
        (false, msg)
    }

    /// `_disconnect` (`statistics.py:20`).
    fn disconnect(&self) {
        self.state.borrow_mut().mem_open = false;
    }

    /// `get_status(eventtime)` (`statistics.py:47`).
    fn get_status(&self) -> PyValue {
        let st = self.state.borrow();
        let mut status = IndexMap::new();
        status.insert(PyValue::Str("sysload".to_string()), PyValue::Float(st.last_load_avg));
        status.insert(
            PyValue::Str("cputime".to_string()),
            PyValue::Float(st.total_process_time),
        );
        status.insert(
            PyValue::Str("memavail".to_string()),
            PyValue::Int(st.last_mem_avail.clone()),
        );
        PyValue::Dict(status)
    }
}

impl PrinterObject for PrinterSysStats {
    klippy_api::printer_object_base!();

    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.inner.get_status()))
    }

    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        Some(self.inner.stats())
    }
}

/// Inner shared state of [`PrinterStats`].
struct StatsInner {
    printer: Printer,
    reactor: Reactor,
    /// Registered with `NEVER`; set after the `Arc` exists because the timer
    /// callback captures it.
    timer: GCell<Option<Timer>>,
    /// Bound `stats` methods, in lookup order (`stats_cb` in Python).
    stats_cbs: GCell<Vec<StatsCallback>>,
}

/// `PrinterStats` (`statistics.py:52`), the [`load_config`] return object
/// registered under the `statistics` section name. It has no `get_status`.
pub struct PrinterStats {
    inner: Arc<StatsInner>,
}

impl PrinterStats {
    /// `PrinterStats(config)` (`statistics.py:53`).
    pub fn new(config: &Config) -> PrinterStats {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let inner = Arc::new(StatsInner {
            printer: printer.clone(),
            reactor: reactor.clone(),
            timer: GCell::new(None),
            stats_cbs: GCell::new(Vec::new()),
        });
        let timer_inner = inner.clone();
        let timer = reactor.register_timer(
            move |eventtime| timer_inner.generate_stats(eventtime),
            NEVER,
        );
        // `GCell::set` requires `T: Copy` and `Timer` is not `Copy`; assign
        // through the mutable borrow instead.
        *inner.timer.borrow_mut() = Some(timer);
        let ready_inner = inner.clone();
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params| {
                ready_inner.handle_ready();
                Ok(PyValue::None)
            }),
        );
        PrinterStats { inner }
    }

    /// `handle_ready` (`statistics.py:59`).
    pub fn handle_ready(&self) {
        self.inner.handle_ready()
    }

    /// `generate_stats(eventtime)` (`statistics.py:64`); the timer callback.
    pub fn generate_stats(&self, eventtime: f64) -> f64 {
        self.inner.generate_stats(eventtime)
    }
}

impl StatsInner {
    /// `handle_ready` (`statistics.py:59`): collect every object's `stats`
    /// bound method in registry order, then arm the timer unless the
    /// printer started with a `debugoutput` file.
    fn handle_ready(&self) {
        self.stats_cbs.borrow_mut().clear();
        for (_name, obj) in self.printer.lookup_objects(None) {
            // A section whose factory returned `None` has no `stats`.
            let Some(obj) = obj else { continue };
            // `hasattr(o, 'stats')` maps to `PrinterObject::stats`
            // returning `Some`. The `None` test cannot happen here: probing
            // it would invoke `stats` (for `PrinterSysStats` that advances
            // the CPU-time counters), so each closure defers the test to
            // call time and non-providers yield `(False, "")`, which never
            // makes `generate_stats` log and joins to nothing.
            self.stats_cbs.borrow_mut().push(Arc::new(move |eventtime: f64| {
                obj.stats(eventtime).unwrap_or((false, String::new()))
            }));
        }
        // `self.printer.get_start_args().get('debugoutput') is None`
        if start_arg_none(&self.printer.get_start_args(), "debugoutput") {
            let timer = self.timer.borrow();
            if let Some(timer) = timer.as_ref() {
                self.reactor.update_timer(timer, NOW);
            }
        }
    }

    /// `generate_stats(eventtime)` (`statistics.py:64`).
    fn generate_stats(&self, eventtime: f64) -> f64 {
        // GCell: the callback list is copied out before calling them so no
        // borrow is held across arbitrary code.
        let cbs = self.stats_cbs.cloned();
        let stats: Vec<(bool, String)> = {
            let _guard = self.reactor.assert_no_pause();
            cbs.iter().map(|cb| cb(eventtime)).collect()
        };
        if stats.iter().any(|(should_log, _)| *should_log) {
            let parts: Vec<&str> = stats
                .iter()
                .map(|(_, message)| message.as_str())
                .filter(|message| !message.is_empty())
                .collect();
            let stats_str = parts.join(" ");
            log::info(&pct_format(
                "Stats %.1f: %s",
                &[PctValue::Float(eventtime), PctValue::Str(stats_str)],
            ));
        }
        eventtime + 1.
    }
}

/// Whether `start_args[key]` is missing or `None`.
fn start_arg_none(args: &PyValue, key: &str) -> bool {
    let value = match args {
        PyValue::Dict(map) => map.get(&PyValue::Str(key.to_string())),
        _ => None,
    };
    matches!(value, None | Some(PyValue::None))
}

klippy_api::printer_object!(PrinterStats);

/// `load_config` (`statistics.py:72`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let sys = PrinterSysStats::new(config);
    printer.add_object("system_stats", Arc::new(sys) as ObjectRef)?;
    Ok(Some(Arc::new(PrinterStats::new(config)) as ObjectRef))
}

/// The frozen `statistics` module factory.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
