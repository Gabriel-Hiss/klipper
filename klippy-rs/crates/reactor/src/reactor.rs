//! Port of `klippy/reactor.py` (`SelectReactor` / `PollReactor`):
//! timers, fds, completions, callbacks, mutexes, greenlets.
//!
//! The dispatcher, timer order, copy-on-write timer list,
//! `update_timer` semantics, greenlet caching (`_g_dispatch`,
//! `_greenlets`), self-pipe and `_next_timer` rules follow
//! `reactor.py` line by line. Greenlets are OS threads with strict
//! baton passing: exactly one reactor greenlet runs at a time.

use kos::cconv;
use kos::fd::{self, PollFd, POLLHUP, POLLIN, POLLOUT};
use kos::libm::ceil;
use kos::sync::{Condvar, Mutex, Sender, channel};
use kos::thread::{self as kthread, tls};
use std::collections::{BTreeMap, VecDeque};
use std::ops::DerefMut;
use std::sync::Arc;
fn make_pipe() -> std::io::Result<(i32, i32)> {
    kos::fd::pipe()
}

/// `NOW = 0.` (`reactor.py _NOW`).
pub const NOW: f64 = 0.;
/// `NEVER = 9999999999999999.` (`reactor.py _NEVER`).
pub const NEVER: f64 = 9999999999999999.;
/// `DEBUG_RING_SIZE`.
pub const DEBUG_RING_SIZE: usize = 32;
/// `DEBUG_STALL_TIME`.
pub const DEBUG_STALL_TIME: f64 = 0.020;
/// `DEBUG_LATE_TIME`.
pub const DEBUG_LATE_TIME: f64 = 0.020;
/// `DEBUG_DUMP_ENTRIES`.
#[allow(dead_code)]
pub const DEBUG_DUMP_ENTRIES: usize = 10;
/// Upper bound on greenlets (safety; Python is unbounded).
pub const MAX_GREENLETS: usize = 100000;

/// Error type (port of `ReactorError`).
#[derive(Debug, Clone)]
pub struct ReactorError(pub String);
impl std::fmt::Display for ReactorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for ReactorError {}

type TimerCb = Box<dyn FnMut(f64) -> f64 + Send>;
type FdCb = Arc<dyn Fn(f64) + Send + Sync>;
type AsyncItem = Box<dyn FnMut(&Reactor) + Send>;

// Like greenlet.GreenletExit, cancellation unwinds suspended callback stacks
// without reporting a callback failure or invoking the process panic hook.
pub(crate) struct GreenletExit;

struct TimerEntry {
    waketime: f64,
    timer_is_running: bool,
    cb: Option<TimerCb>,
}

struct FdEntry {
    fd: i32,
    read_cb: Option<FdCb>,
    write_cb: Option<FdCb>,
    want_read: bool,
    want_write: bool,
}

struct Greenlet {
    id: usize,
    timer_id: Option<u64>,
    handle: Option<kthread::JoinHandle>,
    finished: bool,
}

#[allow(clippy::type_complexity)]
struct DebugState {
    gc_events: VecDeque<(f64, i32, f64, (i64, i64, i64))>,
    stalls: VecDeque<(f64, f64, f64, f64, f64)>,
    late_timers: VecDeque<(f64, f64, String)>,
    last_check: f64,
    check_timeout: f64,
    gc_count: i64,
    stall_count: i64,
    late_count: i64,
    worst_gc: f64,
    worst_stall: f64,
    worst_late: f64,
}

struct Inner {
    process: bool,
    gc_checking: bool,
    last_gc_times: [f64; 3],
    debug: DebugState,
    // Monotonic keys preserve registration order without retaining dead entries
    // or allowing an old Timer handle to refer to a new registration.
    timers: BTreeMap<u64, TimerEntry>,
    next_timer_id: u64,
    next_timer: f64,
    pipe_fds: Option<(i32, i32)>,
    fds: Vec<FdEntry>,
    g_dispatch: Option<usize>,
    running: usize,
    wake: Vec<Option<f64>>,
    // Persistent per-greenlet park flags: `switch_to` parks the
    // caller until a later `switch_to(me, _)` clears the flag, so a
    // wakeup is remembered even if the target has not started
    // waiting yet (greenlet `switch` semantics).
    parked: Vec<bool>,
    cached: Vec<usize>,
    greenlets: Vec<Greenlet>,
    prevent_pause_count: i64,
    terminated: bool,
    /// Stand-in for `gc.get_count()`: generation counters with CPython's
    /// update rules (see [`Reactor::check_gc`]).
    gc_counts: [i64; 3],
    pending_panic: Option<Box<dyn std::any::Any + Send>>,
}

#[allow(clippy::type_complexity)]
struct ReactorInner {
    mu: Mutex<Inner>,
    cv: Condvar,
    async_tx: Sender<AsyncItem>,
    async_rx: kos::sync::Receiver<AsyncItem>,
    gc_hook: Mutex<Option<Box<dyn FnMut(i32) + Send>>>,
}

/// The reactor (port of `SelectReactor`). Cheap to clone; every
/// clone shares one dispatch state.
#[derive(Clone)]
pub struct Reactor {
    inner: Arc<ReactorInner>,
}

impl Reactor {
    /// Create a reactor (`gc_checking` like `SelectReactor(gc_checking)`).
    pub fn new(gc_checking: bool) -> Reactor {
        let (tx, rx) = channel::<AsyncItem>();
        let inner = Arc::new(ReactorInner {
            mu: Mutex::new(Inner {
                process: false,
                gc_checking,
                last_gc_times: [0.; 3],
                debug: DebugState {
                    gc_events: VecDeque::new(),
                    stalls: VecDeque::new(),
                    late_timers: VecDeque::new(),
                    last_check: 0.,
                    check_timeout: 0.,
                    gc_count: 0,
                    stall_count: 0,
                    late_count: 0,
                    worst_gc: 0.,
                    worst_stall: 0.,
                    worst_late: 0.,
                },
                timers: BTreeMap::new(),
                next_timer_id: 0,
                next_timer: NEVER,
                pipe_fds: None,
                fds: Vec::new(),
                g_dispatch: None,
                running: 0,
                wake: Vec::new(),
                parked: Vec::new(),
                cached: Vec::new(),
                greenlets: Vec::new(),
                prevent_pause_count: 0,
                terminated: false,
                gc_counts: [0; 3],
                pending_panic: None,
            }),
            cv: Condvar::new(),
            async_tx: tx,
            async_rx: rx,
            gc_hook: Mutex::new(None),
        });
        // Greenlet 0 is the main/run caller (no thread).
        let r = Reactor { inner };
        r.ensure_greenlet(0);
        Reactor { inner: r.inner }
    }

    /// Install the GC hook (`_check_gc` mapping; see lib docs).
    pub fn set_gc_hook<F: FnMut(i32) + Send + 'static>(&self, hook: F) {
        *self.inner.gc_hook.lock() = Some(Box::new(hook));
    }

    /// `monotonic()` (port of `reactor.monotonic` / `chelper.get_monotonic`).
    pub fn monotonic(&self) -> f64 {
        kos::time::monotonic()
    }

    fn ensure_greenlet(&self, id: usize) {
        let mut g = self.inner.mu.lock();
        while g.greenlets.len() <= id {
            let n = g.greenlets.len();
            g.greenlets.push(Greenlet { id: n, timer_id: None, handle: None, finished: false });
        }
        while g.wake.len() <= id {
            g.wake.push(None);
        }
        while g.parked.len() <= id {
            // Greenlet 0 (the run caller) starts runnable; every
            // dispatcher starts suspended until switched to,
            // like a freshly created greenlet.
            let n = g.parked.len();
            g.parked.push(n != 0);
        }
    }
    fn current(&self) -> usize {
        tls::get().unwrap_or(0)
    }
    // ---- baton -------------------------------------------------
    //
    // Exactly one greenlet runs at a time. `running` names it, and
    // `parked` names which greenlets are suspended inside
    // `switch_to`. `switch_to(target, val)` stores `val` for the
    // target, clears `parked[target]`, parks the caller, and
    // waits until someone clears `parked[me]` with a later
    // `switch_to(me, _)`. The flags are persistent, so a wakeup
    // is never lost when the target has not started waiting yet
    // (greenlet `switch(value)` returns `value` to the target's
    // pending `switch`), unlike a bare `running` token which a
    // dispatcher can steal before `run()` arms its wait.
    fn switch_to(&self, target: usize, val: Option<f64>) -> Option<f64> {
        let me = self.current();
        if me > target {
            self.ensure_greenlet(me);
        } else {
            self.ensure_greenlet(target);
        }
        if target == me {
            return val;
        }
        let mut g = self.inner.mu.lock();
        if g.terminated {
            drop(g);
            if me != 0 {
                std::panic::resume_unwind(Box::new(GreenletExit));
            }
            return None;
        }
        // Re-grow the per-greenlet arrays under the lock: `finalize`
        // may have truncated them between `ensure_greenlet` above
        // and this lock acquisition.
        let need = me.max(target);
        while g.wake.len() <= need {
            g.wake.push(None);
        }
        while g.parked.len() <= need {
            let n = g.parked.len();
            g.parked.push(n != 0);
        }
        g.wake[target] = val;
        g.parked[target] = false;
        g.parked[me] = true;
        g.running = target;
        self.inner.cv.notify_all();
        while !g.terminated && g.parked[me] {
            g = self.inner.cv.wait(g);
        }
        if g.terminated {
            drop(g);
            if me != 0 {
                std::panic::resume_unwind(Box::new(GreenletExit));
            }
            return None;
        }
        g.wake[me].take()
    }

    // ---- timers ------------------------------------------------

    /// Register a timer (`register_timer`).
    pub fn register_timer<F: FnMut(f64) -> f64 + Send + 'static>(&self, cb: F, waketime: f64) -> Timer {
        let mut g = self.inner.mu.lock();
        let id = g.next_timer_id;
        g.next_timer_id = id.checked_add(1).expect("reactor timer id overflow");
        g.timers.insert(id, TimerEntry { waketime, timer_is_running: false, cb: Some(Box::new(cb)) });
        if waketime < g.next_timer {
            g.next_timer = waketime;
        }
        Timer { reactor: self.clone(), id }
    }

    /// `update_timer` (ignored while the timer is running).
    pub fn update_timer(&self, timer: &Timer, waketime: f64) {
        let mut g = self.inner.mu.lock();
        if g.timers.get(&timer.id).map(|t| t.timer_is_running).unwrap_or(false) {
            return;
        }
        if let Some(t) = g.timers.get_mut(&timer.id) {
            t.waketime = waketime;
        }
        if waketime < g.next_timer {
            g.next_timer = waketime;
        }
    }

    /// `unregister_timer` (removes the registration, including during dispatch).
    pub fn unregister_timer(&self, timer: &Timer) {
        let mut g = self.inner.mu.lock();
        g.timers.remove(&timer.id);
    }

    fn timer_waketime(&self, id: u64) -> Option<f64> {
        self.inner.mu.lock().timers.get(&id).map(|t| t.waketime)
    }

    fn check_gc(&self, eventtime: f64) -> bool {
        let mut g = self.inner.mu.lock();
        if !g.gc_checking {
            return false;
        }
        // Rust has no garbage collector. `gc.get_count()` is modelled by
        // generation counters with CPython's rules: each idle check stands
        // for one generation-0 allocation, and `gc.collect(level)` clears
        // generations 0..=level and adds one to the next generation. So a
        // collection fires once per 700 idle checks, never on every check.
        g.gc_counts[0] += 1;
        let c = g.gc_counts;
        let gi = (c[0], c[1], c[2]);
        if gi.0 < 700 {
            return false;
        }
        let mut level = 0;
        if gi.1 >= 10 {
            level = 1;
            if gi.2 >= 10 {
                level = 2;
            }
        }
        for count in &mut g.gc_counts[..=level] {
            *count = 0;
        }
        if level < 2 {
            g.gc_counts[level + 1] += 1;
        }
        g.last_gc_times[level] = eventtime;
        let start = self.monotonic();
        drop(g);
        if let Some(h) = self.inner.gc_hook.lock().deref_mut() {
            h(level as i32);
        }
        let elapsed = self.monotonic() - start;
        let mut g = self.inner.mu.lock();
        g.debug.gc_count += 1;
        if elapsed > g.debug.worst_gc {
            g.debug.worst_gc = elapsed;
        }
        g.debug.gc_events.push_back((start, level as i32, elapsed, gi));
        while g.debug.gc_events.len() > DEBUG_RING_SIZE {
            g.debug.gc_events.pop_front();
        }
        true
    }

    /// `get_gc_stats`.
    pub fn get_gc_stats(&self) -> [f64; 3] {
        self.inner.mu.lock().last_gc_times
    }

    fn debug_note_stall(&self, eventtime: f64, gap: f64, over: f64, sleep_time: f64) {
        let mut g = self.inner.mu.lock();
        g.debug.stall_count += 1;
        if over > g.debug.worst_stall {
            g.debug.worst_stall = over;
        }
        let mut next_in = g.next_timer - eventtime;
        if next_in > 999. {
            next_in = 999.;
        }
        g.debug.stalls.push_back((eventtime, gap, over, sleep_time, next_in));
        while g.debug.stalls.len() > DEBUG_RING_SIZE {
            g.debug.stalls.pop_front();
        }
    }

    fn debug_note_late_timer(&self, eventtime: f64, late: f64, name: &str) {
        let mut g = self.inner.mu.lock();
        g.debug.late_count += 1;
        if late > g.debug.worst_late {
            g.debug.worst_late = late;
        }
        let short: String = name.chars().take(48).collect();
        g.debug.late_timers.push_back((eventtime, late, short));
        while g.debug.late_timers.len() > DEBUG_RING_SIZE {
            g.debug.late_timers.pop_front();
        }
    }

    /// `get_timing_debug_report` (read-only snapshot; safe from a
    /// shutdown handler). Formats match `reactor.py` exactly.
    pub fn get_timing_debug_report(&self, max_entries: usize) -> Vec<String> {
        let g = self.inner.mu.lock();
        let next_in = (g.next_timer - self.monotonic()).min(999.);
        let gc = (g.gc_counts[0], g.gc_counts[1], g.gc_counts[2]);
        let mut out = vec![format!(
            "[DEBUG-motion] version=host-v4 reactor stalls={} worst_stall={:.6} late_timers={} worst_late={:.6} gc_runs={} worst_gc={:.6} gc_times=({:.6}, {:.6}, {:.6}) gc_count=({}s, {}s, {}s) timers={} greenlets={} cached={} prevent_pause={} next_timer_in={:.6}",
            g.debug.stall_count,
            g.debug.worst_stall,
            g.debug.late_count,
            g.debug.worst_late,
            g.debug.gc_count,
            g.debug.worst_gc,
            g.last_gc_times[0],
            g.last_gc_times[1],
            g.last_gc_times[2],
            gc.0,
            gc.1,
            gc.2,
            g.timers.len(),
            g.greenlets.len(),
            g.cached.len(),
            g.prevent_pause_count,
            next_in,
        )];
        let stalls: Vec<_> = g.debug.stalls.iter().copied().collect();
        for (et, gap, over, s, n) in stalls[stalls.len().saturating_sub(max_entries)..].iter() {
            out.push(format!(
                "[DEBUG-motion] reactor stall at={et:.6} gap={gap:.6} over={over:.6} sleep={s:.6} next_timer_in={n:.6}"
            ));
        }
        let lates: Vec<_> = g.debug.late_timers.iter().cloned().collect();
        for (et, late, name) in lates[lates.len().saturating_sub(max_entries)..].iter() {
            out.push(format!("[DEBUG-motion] reactor late_timer at={et:.6} late={late:.6} cb={name}"));
        }
        let gce: Vec<_> = g.debug.gc_events.iter().copied().collect();
        for (gs, lvl, gt, counts) in gce[gce.len().saturating_sub(max_entries)..].iter() {
            out.push(format!(
                "[DEBUG-motion] reactor gc at={:.6} level={} time={:.6} counts=({}, {}, {})",
                gs, lvl, gt, counts.0, counts.1, counts.2
            ));
        }
        out
    }

    fn check_timers(&self, eventtime: f64, busy: bool) -> f64 {
        {
            let mut g = self.inner.mu.lock();
            let last_check = g.debug.last_check;
            let sleep_time = g.debug.check_timeout;
            g.debug.last_check = eventtime;
            g.debug.check_timeout = 0.;
            if last_check != 0. {
                let gap = eventtime - last_check;
                let over = gap - sleep_time;
                if over > DEBUG_STALL_TIME {
                    drop(g);
                    self.debug_note_stall(eventtime, gap, over, sleep_time);
                    g = self.inner.mu.lock();
                }
            }
            if eventtime < g.next_timer {
                if busy {
                    return 0.;
                }
                drop(g);
                if self.check_gc(eventtime) {
                    return 0.;
                }
                let mut g = self.inner.mu.lock();
                let timeout = (0.001f64.max(g.next_timer - eventtime)).min(1.);
                g.debug.check_timeout = timeout;
                return timeout;
            }
            g.next_timer = NEVER;
        }
        // Snapshot of timer ids in registration order (copy-on-write:
        // timers registered during the pass do not fire this pass).
        let snapshot: Vec<u64> = {
            let g = self.inner.mu.lock();
            g.timers.keys().copied().collect()
        };
        let g_dispatch_at_start = self.inner.mu.lock().g_dispatch;
        for id in snapshot {
            let waketime = match self.timer_waketime(id) {
                Some(w) => w,
                None => continue,
            };
            if eventtime >= waketime {
                if waketime != 0. && eventtime - waketime > DEBUG_LATE_TIME {
                    self.debug_note_late_timer(eventtime, eventtime - waketime, &format!("timer{id}"));
                }
                // Take the callback out so it can run without the lock.
                // Restore it only if it remains registered after the callback.
                let mut cb = {
                    let mut g = self.inner.mu.lock();
                    match g.timers.get_mut(&id) {
                        Some(t) => {
                            t.waketime = NEVER;
                            t.timer_is_running = true;
                            t.cb.take()
                        }
                        None => None,
                    }
                };
                let returned = match cb.as_mut() {
                    Some(f) => f(eventtime),
                    None => NEVER,
                };
                {
                    let mut g = self.inner.mu.lock();
                    if let Some(t) = g.timers.get_mut(&id) {
                        t.cb = cb;
                        t.waketime = returned;
                        t.timer_is_running = false;
                    }
                }
                if self.inner.mu.lock().g_dispatch != g_dispatch_at_start {
                    let mut g = self.inner.mu.lock();
                    if returned < g.next_timer {
                        g.next_timer = returned;
                    }
                    drop(g);
                    self.end_greenlet(g_dispatch_at_start.unwrap_or(0));
                    return 0.;
                }
                let mut g = self.inner.mu.lock();
                if returned < g.next_timer {
                    g.next_timer = returned;
                }
            } else {
                let mut g = self.inner.mu.lock();
                if waketime < g.next_timer {
                    g.next_timer = waketime;
                }
            }
        }
        0.
    }

    // ---- callbacks/completions --------------------------------

    /// Create a completion (`completion()`).
    pub fn completion<R>(&self) -> Completion<R> {
        Completion::new(self.clone())
    }

    /// Register a one-shot callback (`register_callback`); returns
    /// its completion.
    pub fn register_callback<F, R>(&self, cb: F, waketime: f64) -> Completion<R>
    where
        F: FnMut(f64) -> R + Send + 'static,
        R: Send + 'static,
    {
        ReactorCallback::new(self.clone(), cb, waketime).completion
    }

    /// Thread-safe callback registration (`register_async_callback`).
    pub fn register_async_callback<F: FnMut(f64) + Send + 'static>(&self, cb: F, waketime: f64) {
        let tx = self.clone();
        let cell: Arc<Mutex<Option<F>>> = Arc::new(Mutex::new(Some(cb)));
        let item: AsyncItem = Box::new(move |reactor: &Reactor| {
            let f = cell.lock().take().expect("one-shot async callback");
            reactor.register_callback::<_, ()>(f, waketime);
        });
        tx.inner.async_tx.put(item);
        // Write one byte to the self-pipe; ignore errors (Python).
        if let Some((_, w)) = self.inner.mu.lock().pipe_fds {
            fd::write(w, b".").ok();
        }
    }

    /// Generic async-callback that discards the result (used when
    /// the caller only needs the side effect).
    pub fn register_async_callback_erased(&self, item: AsyncItem) {
        self.inner.async_tx.put(item);
        if let Some((_, w)) = self.inner.mu.lock().pipe_fds {
            fd::write(w, b".").ok();
        }
    }

    /// Thread-safe completion (`async_complete`).
    pub fn async_complete<R>(&self, completion: &Completion<R>, result: R)
    where
        R: Send + 'static,
    {
        let comp = completion.clone();
        let cell: Arc<Mutex<Option<R>>> = Arc::new(Mutex::new(Some(result)));
        let item: AsyncItem = Box::new(move |_: &Reactor| {
            let r = cell.lock().take().expect("one-shot async result");
            comp.complete(r);
        });
        self.inner.async_tx.put(item);
        if let Some((_, w)) = self.inner.mu.lock().pipe_fds {
            fd::write(w, b".").ok();
        }
    }

    fn setup_async_callbacks(&self) {
        let (r, w) = make_pipe().expect("os.pipe");
        fd::set_nonblock(r).unwrap();
        fd::set_nonblock(w).unwrap();
        // Hold the lock only for the bookkeeping assignment.
        self.inner.mu.lock().pipe_fds = Some((r, w));
        let reactor = self.clone();
        let read_w = w;
        let _ = read_w;
        self.register_fd(r, move |_: f64| { reactor.got_pipe_signal(); }, None::<fn(f64)>);
    }

    fn got_pipe_signal(&self) {
        if let Some((r, _)) = self.inner.mu.lock().pipe_fds {
            let mut buf = [0u8; 4096];
            fd::read(r, &mut buf).ok();
        }
        loop {
            let item = self.inner.async_rx.get_nowait();
            match item {
                Some(mut item) => item(self),
                None => break,
            }
        }
    }

    // ---- greenlets --------------------------------------------

    /// `pause` (`reactor.pause` ∶ greenlets/OS threads with baton passing).
    pub fn pause(&self, waketime: f64) -> f64 {
        if self.inner.mu.lock().g_dispatch.is_none() {
            return self.sys_pause(waketime);
        }
        if self.inner.mu.lock().prevent_pause_count != 0 {
            self.verify_can_pause();
        }
        let g = self.current();
        let g_dispatch = self.inner.mu.lock().g_dispatch.unwrap();
        if g != g_dispatch {
            // Already paused once: timer resume returns eventtime.
            let r = self.switch_to(g_dispatch, Some(waketime));
            return r.unwrap_or(0.);
        }
        // Pausing the dispatcher: register self-wake timer, NOW.
        let reactor = self.clone();
        let wake_timer = self.register_timer(
            move |eventtime| {
                // `g.switch(eventtime)` (reactor.py pause/_check_timers):
                // its return value is the argument of the next switch to
                // this dispatcher, i.e. the waketime of the paused
                // greenlet's next `pause` (or NEVER from `_end_greenlet`),
                // and `_check_timers` assigns it as the timer waketime.
                reactor.switch_to(g, Some(eventtime)).unwrap_or(NEVER)
            },
            waketime,
        );
        {
            let mut inner = self.inner.mu.lock();
            if let Some(gr) = inner.greenlets.get_mut(g) {
                gr.timer_id = Some(wake_timer.id);
            }
        }
        self.inner.mu.lock().next_timer = NOW;
        let g_next = self.inner.mu.lock().cached.pop();
        if let Some(next) = g_next {
            // Switch to cached dispatcher (it sets _g_dispatch when
            // it resumes inside _end_greenlet).
            let eventtime = self.switch_to(next, None);
            return eventtime.unwrap_or(0.);
        }
        // No cached greenlets: switch to run().
        let eventtime = self.switch_to(0, None);
        eventtime.unwrap_or(0.)
    }

    fn sys_pause(&self, waketime: f64) -> f64 {
        // Pause using system sleep for when reactor not running.
        kos::time::sleep_until(waketime)
    }

    fn end_greenlet(&self, g_old: usize) {
        // Cleanup the internal timer of the finished greenlet.
        let timer_id = {
            let g = self.inner.mu.lock();
            g.greenlets.get(g_old).and_then(|gr| gr.timer_id)
        };
        if let Some(id) = timer_id {
            let fake = Timer { reactor: self.clone(), id };
            self.unregister_timer(&fake);
        }
        {
            let mut g = self.inner.mu.lock();
            if let Some(gr) = g.greenlets.get_mut(g_old) {
                gr.timer_id = None;
            }
            g.cached.push(g_old);
        }
        // Switch back to the current dispatcher; our NEVER becomes
        // the (now unregistered) greenlet timer's waketime. On
        // reactivation as cached dispatcher, _g_dispatch = g_old.
        let current = self.inner.mu.lock().g_dispatch;
        if let Some(cur) = current {
            self.switch_to(cur, Some(NEVER));
        }
        let mut g = self.inner.mu.lock();
        // Only reinstall ourselves as dispatcher when we were
        // reactivated via the cached path; after `finalize` sets
        // `terminated`, leave `_g_dispatch` alone (Python's
        // greenlet.throw() in finalize makes this line unreachable).
        if !g.terminated {
            g.g_dispatch = Some(g_old);
        }
    }

    /// `assert_no_pause` (temporarily disabling pauses). The
    /// returned guard enters on construction like Python's
    /// `ReactorPreventPause.__enter__`; dropping it leaves like
    /// `__exit__`.
    pub fn assert_no_pause(&self) -> ReactorPreventPause {
        self.inner.mu.lock().prevent_pause_count += 1;
        ReactorPreventPause { reactor: self.clone() }
    }

    /// `verify_can_pause` presence, raising `ReactorError` while
    /// pauses are disabled.
    pub fn verify_can_pause(&self) {
        if self.inner.mu.lock().prevent_pause_count != 0 {
            panic!("reactor.ReactorError: Internal error - reactor pause disabled");
        }
    }

    /// `mutex(is_locked)` factory: a new, independent mutex.
    pub fn mutex(&self, is_locked: bool) -> ReactorMutex {
        ReactorMutex {
            reactor: self.clone(),
            state: Arc::new(kos::sync::Mutex::new(MutexState {
                is_locked,
                next_pending: false,
                queue: Vec::new(),
            })),
        }
    }

    // ---- fds ---------------------------------------------------

    /// `register_fd`. Callbacks stay registered while running and may
    /// reenter on another dispatch greenlet if they pause.
    pub fn register_fd<F, G>(&self, fd_: i32, read_cb: F, write_cb: Option<G>) -> FdHandler
    where
        F: Fn(f64) + Send + Sync + 'static,
        G: Fn(f64) + Send + Sync + 'static,
    {
        let mut g = self.inner.mu.lock();
        // Replace existing (Python overwrites the dict entry).
        if let Some(e) = g.fds.iter_mut().find(|e| e.fd == fd_) {
            e.read_cb = Some(Arc::new(read_cb));
            e.write_cb = write_cb.map(|c| Arc::new(c) as FdCb);
            e.want_read = true;
            e.want_write = false;
        } else {
            g.fds.push(FdEntry {
                fd: fd_,
                read_cb: Some(Arc::new(read_cb)),
                write_cb: write_cb.map(|c| Arc::new(c) as FdCb),
                want_read: true,
                want_write: false,
            });
        }
        FdHandler { reactor: self.clone(), fd: fd_ }
    }

    /// `unregister_fd`.
    pub fn unregister_fd(&self, handler: &FdHandler) {
        let mut g = self.inner.mu.lock();
        if let Some(pos) = g.fds.iter().position(|e| e.fd == handler.fd) {
            g.fds.remove(pos);
        }
    }

    /// `set_fd_wake`.
    pub fn set_fd_wake(&self, handler: &FdHandler, is_readable: bool, is_writeable: bool) {
        let mut g = self.inner.mu.lock();
        if let Some(e) = g.fds.iter_mut().find(|e| e.fd == handler.fd) {
            e.want_read = is_readable;
            e.want_write = is_writeable;
        }
    }

    fn check_fds(&self, eventtime: f64, hdls: &[(i32, i16)]) -> f64 {
        let g_dispatch_at_start = self.inner.mu.lock().g_dispatch;
        for (fd_, event) in hdls.iter().copied() {
            // Python resolves the handler once per ready event. Keep both
            // callbacks for that event even if the read callback unregisters
            // or replaces its fd; later events look up the registration again.
            let callbacks = {
                let g = self.inner.mu.lock();
                g.fds.iter().find(|e| e.fd == fd_)
                    .map(|e| (e.read_cb.clone(), e.write_cb.clone()))
            };
            let Some((read_cb, write_cb)) = callbacks else {
                continue;
            };
            if event & (POLLIN | POLLHUP) != 0 {
                if let Some(cb) = read_cb {
                    cb(eventtime);
                }
                if self.inner.mu.lock().g_dispatch != g_dispatch_at_start {
                    self.end_greenlet(g_dispatch_at_start.unwrap_or(0));
                    return self.monotonic();
                }
            }
            if event & POLLOUT != 0 {
                if let Some(cb) = write_cb {
                    cb(eventtime);
                }
                if self.inner.mu.lock().g_dispatch != g_dispatch_at_start {
                    self.end_greenlet(g_dispatch_at_start.unwrap_or(0));
                    return self.monotonic();
                }
            }
        }
        eventtime
    }

    // ---- main loop ---------------------------------------------

    fn dispatch_loop(&self, id: usize) {
        tls::set(id);
        // A fresh dispatch greenlet starts suspended, like a new
        // greenlet before its first `switch`: do not run until
        // `run()` switches to us. The `parked` flag persists, so
        // an early switch is not lost.
        {
            let mut g = self.inner.mu.lock();
            // Defensive: `finalize` may have truncated `parked`
            // after we were spawned; a missing slot means
            // terminated (sit out the wait and exit below).
            while !g.terminated && g.parked.get(id).copied().unwrap_or(true) {
                g = self.inner.cv.wait(g);
            }
            if g.terminated {
                return;
            }
        }
        let mut busy = true;
        let mut eventtime = self.monotonic();
        while self.inner.mu.lock().process && !self.inner.mu.lock().terminated {
            let timeout = self.check_timers(eventtime, busy);
            busy = false;
            if self.inner.mu.lock().terminated {
                break;
            }
            if self.inner.mu.lock().process {
                let fds: Vec<PollFd> = {
                    let g = self.inner.mu.lock();
                    g.fds
                        .iter()
                        .map(|e| {
                            let mut ev = POLLHUP;
                            if e.want_read {
                                ev |= POLLIN;
                            }
                            if e.want_write {
                                ev |= POLLOUT;
                            }
                            PollFd::new(e.fd, ev)
                        })
                        .collect()
                };
                let mut fds: Vec<PollFd> = fds;
                let ms = poll_timeout_ms(timeout);
                fd::poll_retry(&mut fds, ms).ok();
                if self.inner.mu.lock().terminated {
                    break;
                }
                eventtime = self.monotonic();
                let ready: Vec<(i32, i16)> = fds.iter().filter(|p| p.revents != 0).map(|p| (p.fd, p.revents)).collect();
                if !ready.is_empty() {
                    busy = true;
                    eventtime = self.check_fds(eventtime, &ready);
                }
            }
        }
        self.mark_finished(id);
    }

    fn mark_finished(&self, id: usize) {
        let mut g = self.inner.mu.lock();
        if let Some(gr) = g.greenlets.get_mut(id) {
            gr.finished = true;
        }
        drop(g);
        // Return control to run().
        self.switch_to(0, None);
    }

    fn dispatch_panic(&self, id: usize, payload: Box<dyn std::any::Any + Send>) {
        let mut g = self.inner.mu.lock();
        if let Some(gr) = g.greenlets.get_mut(id) {
            gr.finished = true;
            if let Some(timer_id) = gr.timer_id.take() {
                g.timers.remove(&timer_id);
            }
        }
        g.cached.retain(|&cached| cached != id);
        if payload.is::<GreenletExit>() {
            return;
        }
        g.pending_panic = Some(payload);
        g.process = false;
        // An uncaught greenlet exception returns to its parent, run().
        // Do not park this failed thread: it has no stack to resume.
        g.running = 0;
        g.parked[0] = false;
        g.wake[0] = None;
        self.inner.cv.notify_all();
    }

    /// `run()` (main loop). Creates dispatcher greenlets while
    /// `_process` is set; returns after [`end`](Self::end). Callback panics
    /// unwind from this thread, and a later `run()` can perform shutdown.
    pub fn run(&self) {
        if self.inner.mu.lock().pipe_fds.is_none() {
            self.setup_async_callbacks();
        }
        {
            let mut g = self.inner.mu.lock();
            g.process = true;
            g.prevent_pause_count = 0;
            g.terminated = false;
        }
        while self.inner.mu.lock().process {
            let id = {
                let g = self.inner.mu.lock();
                g.greenlets.iter().map(|gr| gr.id).max().map(|m| m + 1).unwrap_or(0)
            };
            self.ensure_greenlet(id);
            {
                let mut g = self.inner.mu.lock();
                g.g_dispatch = Some(id);
            }
            let reactor = self.clone();
            let handle = kthread::spawn(move || {
                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    reactor.dispatch_loop(id);
                }));
                if let Err(payload) = result {
                    reactor.dispatch_panic(id, payload);
                }
            });
            {
                let mut g = self.inner.mu.lock();
                if let Some(gr) = g.greenlets.get_mut(id) {
                    gr.handle = Some(handle);
                }
            }
            // Switch to the new dispatcher; returns when it pauses
            // to run() (no cached) or finishes (end).
            self.switch_to(id, None);
            // If we returned because the dispatcher paused to run(),
            // spawn a fresh one on the next iteration. If end() was
            // called, the loop condition exits.
        }
        let payload = {
            let mut g = self.inner.mu.lock();
            g.g_dispatch = None;
            g.pending_panic.take()
        };
        if let Some(payload) = payload {
            std::panic::resume_unwind(payload);
        }
    }

    /// `end()` (just clears `_process`).
    pub fn end(&self) {
        self.inner.mu.lock().process = false;
    }

    /// `finalize()` (kill greenlets, clear caches, close self-pipe).
    pub fn finalize(&self) {
        let mut g = self.inner.mu.lock();
        g.g_dispatch = None;
        g.cached.clear();
        g.terminated = true;
        // Stop the dispatch loop immediately so `run()` does not
        // spawn a fresh dispatcher while finalizing.
        g.process = false;
        let pipe = g.pipe_fds.take();
        drop(g);
        self.inner.cv.notify_all();
        if let Some((r, w)) = pipe {
            fd::close(r);
            fd::close(w);
        }
        // Suspended greenlets unwind with GreenletExit before they are joined.
        // Skip self. Reset state only after the join: parked threads still
        // access these arrays while waking to observe termination.
        let me = self.current();
        let mut handles: Vec<kthread::JoinHandle> = Vec::new();
        {
            let mut g = self.inner.mu.lock();
            for gr in g.greenlets.iter_mut() {
                if gr.id != me {
                    if let Some(h) = gr.handle.take() {
                        handles.push(h);
                    }
                }
            }
        }
        for h in handles {
            h.join();
        }
        let mut g = self.inner.mu.lock();
        g.process = false;
        g.terminated = false;
        // Keep the main greenlet (id 0, the `run()` caller): later
        // `run()` calls allocate dispatcher ids above it. This also
        // matches the `end_and_finalize` test (`greenlet_count()==1`).
        g.greenlets.retain(|gr| gr.id == 0);
        g.wake.truncate(1);
        g.parked.truncate(1);
        g.cached.clear();
    }

    /// Number of live greenlets (for tests; Python `_all_greenlets`).
    pub fn greenlet_count(&self) -> usize {
        self.inner.mu.lock().greenlets.len()
    }

    /// Number of cached dispatcher greenlets (`_cached_dispatch_greenlets`).
    pub fn cached_count(&self) -> usize {
        self.inner.mu.lock().cached.len()
    }
}

/// `ceil(timeout * 1000)` as poll milliseconds, via libm `ceil`
/// and `cconv` (never bare float casts).
fn poll_timeout_ms(timeout: f64) -> i32 {
    let ms = ceil(timeout * 1000.);
    let v = cconv::f64_to_i64(ms);
    if v < 0 {
        -1
    } else if v > i32::MAX as i64 {
        i32::MAX
    } else {
        v as i32
    }
}

// ---- handles -------------------------------------------------

/// Timer handle (`ReactorTimer`).
#[allow(dead_code)]
pub struct Timer {
    reactor: Reactor,
    id: u64,
}

/// File handler (`ReactorFileHandler`).
#[allow(dead_code)]
pub struct FdHandler {
    reactor: Reactor,
    fd: i32,
}

/// One-shot callback (`ReactorCallback`).
#[allow(dead_code)]
pub struct ReactorCallback<R> {
    /// The completion to `complete` on return.
    pub completion: Completion<R>,
    reactor: Reactor,
    timer: Timer,
}

impl<R> ReactorCallback<R> {
    /// Register `callback(eventtime) -> result` as a one-shot timer.
    /// `invoke` unregisters the timer first, calls `callback`,
    /// completes with its return value, then returns `NEVER`.
    pub fn new<F>(reactor: Reactor, cb: F, waketime: f64) -> Self
    where
        F: FnMut(f64) -> R + Send + 'static,
        R: Send + 'static,
    {
        let completion = Completion::new(reactor.clone());
        let comp = completion.clone();
        let reactor2 = reactor.clone();
        let timer_slot: Arc<Mutex<Option<Timer>>> = Arc::new(Mutex::new(None));
        let slot2 = Arc::clone(&timer_slot);
        // One-shot cell: Option<F> lets the first invocation move it out.
        let cell: Arc<Mutex<Option<F>>> = Arc::new(Mutex::new(Some(cb)));
        let cell2 = Arc::clone(&cell);
        let timer = reactor.register_timer(
            move |eventtime| {
                if let Some(t) = slot2.lock().take() {
                    reactor2.unregister_timer(&t);
                }
                let mut taken = cell2.lock().take().expect("one-shot callback not taken yet");
                let res = taken(eventtime);
                comp.complete(res);
                NEVER
            },
            waketime,
        );
        *timer_slot.lock() = Some(Timer { reactor: reactor.clone(), id: timer.id });
        ReactorCallback { completion, reactor, timer }
    }
}

/// Completion (`ReactorCompletion`).
pub struct Completion<R> {
    reactor: Reactor,
    data: Arc<CompletionData<R>>,
}

struct CompletionData<R> {
    mu: Mutex<Option<R>>,
    waiting: Mutex<Vec<usize>>,
}

impl<R> Clone for Completion<R> {
    fn clone(&self) -> Self {
        Completion { reactor: self.reactor.clone(), data: Arc::clone(&self.data) }
    }
}

impl<R> Completion<R> {
    /// New empty completion (pending).
    pub fn new(reactor: Reactor) -> Self {
        Completion { reactor, data: Arc::new(CompletionData { mu: Mutex::new(None), waiting: Mutex::new(Vec::new()) }) }
    }
    /// `test()` — `true` once completed.
    pub fn test(&self) -> bool {
        self.data.mu.lock().is_some()
    }
    /// `complete(result)` — store result, wake all waiters with NOW.
    /// Repeated completes overwrite (last wins, like Python).
    pub fn complete(&self, result: R) {
        *self.data.mu.lock() = Some(result);
        let waiters = std::mem::take(&mut *self.data.waiting.lock());
        for id in waiters {
            let timer_id = {
                let g = self.reactor.inner.mu.lock();
                g.greenlets.get(id).and_then(|gr| gr.timer_id)
            };
            if let Some(tid) = timer_id {
                let t = Timer { reactor: self.reactor.clone(), id: tid };
                self.reactor.update_timer(&t, NOW);
            }
        }
    }
    /// `wait(waketime, waketime_result)`: park until completed or
    /// the pause timeout fires. Returns a clone of the stored result, or
    /// `None` on timeout (like returning `waketime_result`). The result
    /// stays stored (`klippy/reactor.py:40-48`), so `multi_complete` and
    /// `home_wait` can wait on the same completion.
    pub fn wait(&self, waketime: f64) -> Option<R>
    where
        R: Clone,
    {
        if self.test() {
            return self.data.mu.lock().clone();
        }
        let cur = self.reactor.current();
        self.data.waiting.lock().push(cur);
        self.reactor.pause(waketime);
        // Remove one occurrence (Python `list.remove`).
        let mut w = self.data.waiting.lock();
        if let Some(pos) = w.iter().position(|&x| x == cur) {
            w.remove(pos);
        }
        drop(w);
        if self.test() {
            self.data.mu.lock().clone()
        } else {
            None
        }
    }
    /// `wait` with an explicit waketime result.
    pub fn wait_or(&self, waketime: f64, waketime_result: R) -> R
    where
        R: Clone,
    {
        match self.wait(waketime) {
            Some(r) => r,
            None => waketime_result,
        }
    }
}

/// Mutex (`ReactorMutex`, `reactor.py`): FIFO greenlet queue with direct
/// hand-off. Each mutex owns its lock flag and queue, like the Python
/// object; a clone is another reference to the same mutex (Python passes
/// the object around, e.g. `gcode.get_mutex()`).
#[derive(Clone)]
pub struct ReactorMutex {
    reactor: Reactor,
    state: Arc<kos::sync::Mutex<MutexState>>,
}

/// `ReactorMutex` attributes: `is_locked`, `next_pending`, `queue`.
struct MutexState {
    is_locked: bool,
    next_pending: bool,
    /// Waiting greenlets, as reactor greenlet ids.
    queue: Vec<usize>,
}

impl ReactorMutex {
    /// `test()` — `is_locked`.
    pub fn test(&self) -> bool {
        self.state.lock().is_locked
    }
    /// `lock()` / `__enter__`.
    pub fn lock(&self) {
        {
            let mut st = self.state.lock();
            if !st.is_locked {
                st.is_locked = true;
                return;
            }
        }
        let cur = self.reactor.current();
        self.state.lock().queue.push(cur);
        loop {
            self.reactor.pause(NEVER);
            let mut st = self.state.lock();
            if st.next_pending && st.queue.first() == Some(&cur) {
                st.next_pending = false;
                st.queue.remove(0);
                return;
            }
        }
    }
    /// `unlock()` / `__exit__`.
    pub fn unlock(&self) {
        let head = {
            let mut st = self.state.lock();
            if st.queue.is_empty() {
                st.is_locked = false;
                return;
            }
            st.next_pending = true;
            st.queue[0]
        };
        let timer_id = self.reactor.inner.mu.lock().greenlets.get(head).and_then(|gr| gr.timer_id);
        if let Some(tid) = timer_id {
            let t = Timer { reactor: self.reactor.clone(), id: tid };
            self.reactor.update_timer(&t, NOW);
        }
    }
}

/// `assert_no_pause` guard (`ReactorPreventPause`).
pub struct ReactorPreventPause {
    reactor: Reactor,
}
impl Drop for ReactorPreventPause {
    fn drop(&mut self) {
        let mut g = self.reactor.inner.mu.lock();
        g.prevent_pause_count -= 1;
    }
}
impl std::ops::Deref for ReactorPreventPause {
    type Target = Reactor;
    fn deref(&self) -> &Reactor {
        &self.reactor
    }
}

/// Install `&self` as the prevent-pause counter (enter side).
impl ReactorPreventPause {
    /// Explicit enter (for alternative syntax); guards enter on
    /// construction in `Reactor::assert_no_pause`.
    pub fn enter(&self) {}
}

#[cfg(test)]
mod tests;
