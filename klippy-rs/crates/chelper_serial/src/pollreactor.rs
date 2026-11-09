//! Port of `klippy/chelper/pollreactor.c` / `pollreactor.h`: the
//! `poll(2)` + timer dispatch loop the serial background thread runs.
//!
//! The C reactor has no locking (all callbacks run on the background
//! thread; the main thread only ever calls `pollreactor_do_exit` /
//! reads timer times for dumps). The port keeps the same structure,
//! with the state behind `kos::sync::Mutex` so timer values read from
//! the main thread (stats/dump paths) are race-free.

use kos::{cconv, fd, libm, sync, time};
use std::sync::atomic::{AtomicBool, Ordering};

/// `PR_NOW` from `pollreactor.h`.
pub const PR_NOW: f64 = 0.;
/// `PR_NEVER` from `pollreactor.h`.
pub const PR_NEVER: f64 = 9999999999999999.;

/// fd callback: `(context, eventtime) -> ()`.
pub type FdCallback<C> = Box<dyn Fn(&C, f64) + Send + Sync>;
/// Timer callback: `(context, eventtime) -> next waketime` (the return
/// value becomes the timer's waketime, like the C `double` return).
pub type TimerCallback<C> = Box<dyn Fn(&C, f64) -> f64 + Send + Sync>;

struct TimerState<C> {
    waketime: f64,
    callback: Option<TimerCallback<C>>,
}

struct TimerInner<C> {
    timers: Vec<TimerState<C>>,
    next_timer: f64,
}

/// Port of `struct pollreactor`.
pub struct PollReactor<C> {
    fds: sync::Mutex<Vec<fd::PollFd>>,
    fd_callbacks: sync::Mutex<Vec<Option<FdCallback<C>>>>,
    timers: sync::Mutex<TimerInner<C>>,
    must_exit: AtomicBool,
}

impl<C> PollReactor<C> {
    /// Port of `pollreactor_alloc(num_fds, num_timers, callback_data)`.
    pub fn new(num_fds: usize, num_timers: usize) -> Self {
        let mut timers = Vec::with_capacity(num_timers);
        for _ in 0..num_timers {
            timers.push(TimerState { waketime: PR_NEVER, callback: None });
        }
        let mut fds = Vec::with_capacity(num_fds);
        for _ in 0..num_fds {
            fds.push(fd::PollFd::new(-1, 0));
        }
        let mut fd_callbacks = Vec::with_capacity(num_fds);
        for _ in 0..num_fds {
            fd_callbacks.push(None);
        }
        PollReactor {
            fds: sync::Mutex::new(fds),
            fd_callbacks: sync::Mutex::new(fd_callbacks),
            timers: sync::Mutex::new(TimerInner { timers, next_timer: PR_NEVER }),
            must_exit: AtomicBool::new(false),
        }
    }

    /// Port of `pollreactor_add_fd`: `events = POLLHUP | (write_only ? 0 : POLLIN)`.
    pub fn add_fd(&self, pos: usize, fd_: i32, write_only: bool, callback: FdCallback<C>) {
        {
            let mut fds = self.fds.lock();
            fds[pos] = fd::PollFd::new(fd_, fd::POLLHUP | if write_only { 0 } else { fd::POLLIN });
        }
        self.fd_callbacks.lock()[pos] = Some(callback);
    }

    /// Port of `pollreactor_add_timer`: the timer starts at `PR_NEVER`.
    pub fn add_timer(&self, pos: usize, callback: TimerCallback<C>) {
        let mut timers = self.timers.lock();
        timers.timers[pos].callback = Some(callback);
        timers.timers[pos].waketime = PR_NEVER;
    }

    /// Port of `pollreactor_get_timer`.
    pub fn get_timer(&self, pos: usize) -> f64 {
        self.timers.lock().timers[pos].waketime
    }

    /// Port of `pollreactor_update_timer`.
    pub fn update_timer(&self, pos: usize, waketime: f64) {
        let mut timers = self.timers.lock();
        timers.timers[pos].waketime = waketime;
        if waketime < timers.next_timer {
            timers.next_timer = waketime;
        }
    }

    /// Port of `pollreactor_check_timers`: run all due timer
    /// callbacks (in slot order, re-reading each waketime as the loop
    /// advances, like the C) and return the `poll` timeout in
    /// milliseconds (`0` when `busy`, else
    /// `clamp(ceil((next_timer - eventtime) * 1000), 1, 1000)`).
    fn check_timers(&self, ctx: &C, eventtime: f64, busy: bool) -> i32 {
        let mut busy = busy;
        let due = eventtime
            >= {
                let timers = self.timers.lock();
                timers.next_timer
            };
        if due {
            self.timers.lock().next_timer = PR_NEVER;
            let num_timers = self.timers.lock().timers.len();
            for i in 0..num_timers {
                let t = self.timers.lock().timers[i].waketime;
                if eventtime >= t {
                    busy = true;
                    // The callback runs without the timers lock held
                    // (the C invokes it single-threaded with no lock);
                    // a concurrent `update_timer` can land during the
                    // call, exactly as in the C.
                    let cb = {
                        let mut timers = self.timers.lock();
                        timers.timers[i].callback.take()
                    };
                    let new_t = match &cb {
                        Some(f) => f(ctx, eventtime),
                        None => t,
                    };
                    let mut timers = self.timers.lock();
                    timers.timers[i].waketime = new_t;
                    if timers.timers[i].callback.is_none() {
                        timers.timers[i].callback = cb;
                    }
                    if new_t < timers.next_timer {
                        timers.next_timer = new_t;
                    }
                } else {
                    let mut timers = self.timers.lock();
                    if t < timers.next_timer {
                        timers.next_timer = t;
                    }
                }
            }
        }
        if busy {
            return 0;
        }
        let next_timer = self.timers.lock().next_timer;
        let timeout = libm::ceil((next_timer - eventtime) * 1000.);
        if timeout < 1. {
            1
        } else if timeout > 1000. {
            1000
        } else {
            // C: `(int)timeout` with `timeout` already in [1, 1000];
            // the cast is exact there.
            cconv::f64_to_i32(timeout)
        }
    }

    /// Port of `pollreactor_run`.
    pub fn run(&self, ctx: &C) {
        let mut eventtime = time::monotonic();
        let mut busy = true;
        while !self.must_exit.load(Ordering::SeqCst) {
            let timeout = self.check_timers(ctx, eventtime, busy);
            busy = false;
            let ret = {
                let mut fds = self.fds.lock();
                fd::poll(&mut fds, timeout)
            };
            eventtime = time::monotonic();
            match ret {
                Ok(n) if n > 0 => {
                    busy = true;
                    // The C invokes, in slot order, every fd whose
                    // revents is nonzero. The callbacks are taken out
                    // under the lock so they run single-threaded as in
                    // the C, then restored in place.
                    let num_fds = self.fds.lock().len();
                    for i in 0..num_fds {
                        if self.fds.lock()[i].revents == 0 {
                            continue;
                        }
                        let cb = {
                            let mut callbacks = self.fd_callbacks.lock();
                            callbacks[i].take()
                        };
                        if let Some(f) = cb {
                            f(ctx, eventtime);
                            let mut callbacks = self.fd_callbacks.lock();
                            if callbacks[i].is_none() {
                                callbacks[i] = Some(f);
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(_) => {
                    crate::report_errno("poll", -1, last_errno());
                    self.do_exit();
                }
            }
        }
    }

    /// Port of `pollreactor_do_exit`.
    pub fn do_exit(&self) {
        self.must_exit.store(true, Ordering::SeqCst);
    }

    /// Port of `pollreactor_is_exit`.
    pub fn is_exit(&self) -> bool {
        self.must_exit.load(Ordering::SeqCst)
    }
}

fn last_errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
}

/// Port of the C `fd_set_non_blocking`: `fcntl(F_GETFL)` then
/// `fcntl(F_SETFL, flags | O_NONBLOCK)`. The error return is ignored
/// by the C callers; kos' version reports the same errno.
pub fn fd_set_non_blocking(fd_: i32) {
    let _ = fd::set_nonblock(fd_);
}
