//! Port of `klippy/reactor.py` (the EPoll variant klippy selects;
//! here the poll variant `PollReactor`, which klippy also selects
//! when `select.poll` exists — always on Linux).
//!
//! Greenlets are OS threads created by `kos` with strict baton
//! passing: exactly one reactor greenlet runs at a time, and
//! switches happen only where the Python switches (`pause`,
//! `completion.wait`, `ReactorMutex.lock` contention, timer/fd
//! return).  Callbacks never preempt each other: two statements
//! without an intervening pause never interleave.
//!
//! # Rust closure API
//!
//! - Timers: [`Reactor::register_timer`] takes
//!   `FnMut(eventtime) -> waketime` and returns a [`Timer`].
//!   The callback returns the next waketime (`NEVER` to sleep,
//!   `NOW` to run on the next pass).  [`Reactor::update_timer`]
//!   re-arms a timer (ignored while it is running, like Python).
//!   [`Reactor::unregister_timer`] removes it, even during its callback.
//!   Only registered timers occupy dispatch storage; handles remain stable
//!   and dispatch follows registration order.
//! - Callbacks: [`Reactor::register_callback`] wraps a one-shot
//!   timer ([`ReactorCallback`]) and returns a typed
//!   [`Completion`].  `register_async_callback` / `Reactor::async_complete`
//!   are the thread-safe entry points (self-pipe + channel, like
//!   Python's `_async_queue` self-pipe); direct `complete` from
//!   another thread is unsupported (same as Python).
//! - Completions: [`Completion::complete`], [`Completion::test`],
//!   [`Completion::wait`] (with timeout and waketime result).
//! - Files: [`Reactor::register_fd`] takes reentrant `Fn(eventtime)`
//!   read/write closures. They remain registered during a pause and use
//!   interior mutability for state. [`Reactor::unregister_fd`] removes the
//!   handler; [`Reactor::set_fd_wake`] updates polling interest.
//! - Cooperative sleeps: [`Reactor::pause`] returns the
//!   `eventtime` of the dispatch pass that resumed it (not
//!   `monotonic()` at return).  [`Reactor::assert_no_pause`] (a
//!   guard) makes `pause` raise [`ReactorError`] while held.
//! - Main loop: [`Reactor::run`] blocks the caller like `run()`. A callback
//!   panic unwinds from `run()` on that caller, allowing a second shutdown
//!   pass. [`Reactor::end`] stops dispatch; [`Reactor::finalize`] unwinds
//!   suspended callback stacks before joining them, so a canceled pause
//!   never returns to its callback.
//!
//! Constants: [`NOW`] (`0.`), [`NEVER`] (`9999999999999999.`),
//! [`monotonic`] (port of `chelper.get_monotonic` via
//! `kos::time::monotonic`).
//!
//! GC note: Python's `_check_gc` collects when
//! `gc.get_count()[0] >= 700` while idle and can stall dispatch.
//! Rust has no GC; the equivalent hook is the
//! `gc_checking` flag plus [`Reactor::set_gc_hook`] (called with
//! the gc level the Python picker would choose) — when no hook is
//! set, `_check_gc` reports not-busy and dispatch is unaffected
//! (documented mapping).

#![warn(missing_docs)]
#![forbid(unsafe_op_in_unsafe_fn)]

mod reactor;

pub use reactor::{
    Completion, FdHandler, Reactor, ReactorCallback, ReactorError, ReactorMutex,
    ReactorPreventPause, Timer, MAX_GREENLETS, NEVER, NOW,
};

/// Re-export of the monotonic clock for convenience.
pub fn monotonic() -> f64 {
    kos::time::monotonic()
}

/// Resume cancellation of a suspended reactor greenlet, or return an ordinary
/// panic payload for conversion to an application error. Exception handlers must
/// not swallow cancellation, just as `except Exception` excludes `GreenletExit`.
pub fn propagate_greenlet_exit(
    payload: Box<dyn std::any::Any + Send>,
) -> Box<dyn std::any::Any + Send> {
    if payload.is::<reactor::GreenletExit>() {
        std::panic::resume_unwind(payload);
    }
    payload
}
