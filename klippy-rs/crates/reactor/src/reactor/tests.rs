//! Behavioural tests transcribed from `reactor.py` (do not run here;
//! the integration owner runs them under the vclock).
use super::*;
use std::sync::Arc;

fn fresh() -> Reactor {
    Reactor::new(false)
}

// Run the reactor with a watchdog: fail after 4 seconds of real
// time instead of hanging forever on a baton deadlock. On timeout
// the watchdog finalizes the reactor so `run()` unwinds and the
// test panics with a clear message.
fn run_with_timeout(r: &Reactor) {
    use kos::thread as kthread;
    use kos::time as ktime;
    use std::sync::atomic::{AtomicBool, Ordering};
    let r_watch = r.clone();
    let done = Arc::new(AtomicBool::new(false));
    let fired = Arc::new(AtomicBool::new(false));
    let done2 = Arc::clone(&done);
    let fired2 = Arc::clone(&fired);
    let watchdog = kthread::spawn(move || {
        let deadline = ktime::monotonic() + 4.;
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
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| r.run()));
    done.store(true, Ordering::SeqCst);
    watchdog.join();
    assert!(!fired.load(Ordering::SeqCst), "reactor run timed out after 4s (baton deadlock?)");
    if let Err(payload) = result {
        std::panic::resume_unwind(payload);
    }
}

fn end_timer(r: &Reactor, at: f64) {
    let rc = r.clone();
    r.register_timer(move |_| { rc.end(); NEVER }, at);
}
#[test]
fn timer_order_registration() {
    let r = fresh();
    let order = Arc::new(Mutex::new(Vec::new()));
    for i in 0..3 {
        let o = Arc::clone(&order);
        let rc = r.clone();
        r.register_timer(move |_| { o.lock().push(i); NEVER }, rc.monotonic() + 0.01 + (i as f64) * 0.005);
    }
    end_timer(&r, r.monotonic() + 0.05);
    run_with_timeout(&r);
    assert_eq!(*order.lock(), vec![0, 1, 2]);
}

#[test]
fn completion_wait_timeout() {
    let r = fresh();
    let c: Completion<()> = r.completion();
    let c2 = c.clone();
    let rc = r.clone();
    r.register_timer(move |_| { c2.complete(()); NEVER }, rc.monotonic() + 0.01);
    end_timer(&r, r.monotonic() + 0.1);
    let cc = c.clone();
    let rc2 = r.clone();
    r.register_callback(move |_| {
        cc.wait_or(rc2.monotonic() + 0.05, ());
    }, NOW);
    run_with_timeout(&r);
}

#[test]
fn mutex_handoff_order() {
    let r = fresh();
    let m = Arc::new(r.mutex(false));
    let log = Arc::new(Mutex::new(Vec::new()));
    let m2 = Arc::clone(&m);
    let l2 = Arc::clone(&log);
    r.register_callback(move |_| {
        m2.lock();
        l2.lock().push("first");
        m2.unlock();
    }, NOW);
    let m4 = Arc::clone(&m);
    let l3 = Arc::clone(&log);
    let mono_ok = Arc::new(Mutex::new(false));
    let m_ok = Arc::clone(&mono_ok);
    let rc = r.clone();
    let at = r.monotonic() + 0.01;
    r.register_callback(move |_| {
        m4.lock();
        l3.lock().push("second");
        *m_ok.lock() = rc.monotonic() > 0.;
        m4.unlock();
    }, at);
    end_timer(&r, r.monotonic() + 0.1);
    run_with_timeout(&r);
    assert_eq!(*log.lock(), vec!["first", "second"]);
    assert!(*mono_ok.lock());
}

/// Each `ReactorMutex` owns its lock flag and queue (`reactor.py`
/// `ReactorMutex.__init__`): creating, locking or unlocking one mutex
/// never changes another, such as the gcode mutex.
#[test]
fn mutexes_are_independent() {
    let r = fresh();
    let gcode = r.mutex(false);
    gcode.lock();
    let led = r.mutex(false);
    assert!(gcode.test(), "creating an unlocked mutex unlocked another");
    assert!(!led.test(), "a new unlocked mutex reports locked");
    led.lock();
    led.unlock();
    assert!(gcode.test(), "unlocking one mutex unlocked another");
    let preset = r.mutex(true);
    assert!(preset.test(), "mutex(True) starts unlocked");
    gcode.unlock();
    assert!(!gcode.test(), "unlock left the mutex locked");
    assert!(preset.test(), "unlocking one mutex unlocked another");
}

#[test]
fn assert_no_pause_violates() {
    let r = fresh();
    let r2 = r.clone();
    let ok = Arc::new(Mutex::new(false));
    let ok2 = Arc::clone(&ok);
    r.register_callback(move |_| {
        let _guard = r2.assert_no_pause();
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| { r2.pause(NEVER); }));
        *ok2.lock() = panicked.is_err();
        r2.end();
    }, NOW);
    run_with_timeout(&r);
    assert!(*ok.lock());
}

#[test]
fn pause_returns_eventtime() {
    let r = fresh();
    let r2 = r.clone();
    let et_slot = Arc::new(Mutex::new(0.));
    let et2 = Arc::clone(&et_slot);
    r.register_callback(move |_| {
        let et = r2.pause(r2.monotonic() + 0.01);
        *et2.lock() = et;
        r2.end();
    }, NOW);
    run_with_timeout(&r);
    assert!(*et_slot.lock() > 0.);
}

#[test]
fn async_callback_from_another_thread() {
    let r = fresh();
    let done = Arc::new(Mutex::new(false));
    let d2 = Arc::clone(&done);
    r.register_async_callback(move |_| { *d2.lock() = true; }, NOW);
    let r2 = r.clone();
    let d3 = Arc::clone(&done);
    let rc = r.clone();
    let at = r.monotonic() + 0.01;
    r.register_timer(move |_| {
        let d4 = Arc::clone(&d3);
        r2.register_async_callback(move |_| { *d4.lock() = true; }, NOW);
        NEVER
    }, at);
    end_timer(&rc, rc.monotonic() + 0.1);
    run_with_timeout(&r);
    assert!(*done.lock());
}

#[test]
fn nested_pause_cached() {
    let r = fresh();
    let r2 = r.clone();
    let slot = Arc::new(Mutex::new((0., 0.)));
    let slot2 = Arc::clone(&slot);
    r.register_callback(move |_| {
        let a = r2.pause(r2.monotonic() + 0.01);
        let b = r2.pause(r2.monotonic() + 0.02);
        *slot2.lock() = (a, b);
        r2.end();
    }, NOW);
    run_with_timeout(&r);
    let (a, b) = *slot.lock();
    assert!(b >= a, "second pause eventtime {b} before first {a}");
    // Python caches the old dispatcher in `_end_greenlet` only after
    // the first dispatcher's timer callback returns (reactor.py 273-276,
    // 346-356). At the point inside the callback `_cached_dispatch_greenlets`
    // is still empty; after the callback returns the old dispatcher
    // is retired and cached, so read the count after `run()` exits.
    assert!(r.cached_count() >= 1, "old dispatcher not cached after nested pause");
}

#[test]
fn end_and_finalize() {
    let r = fresh();
    end_timer(&r, r.monotonic() + 0.01);
    run_with_timeout(&r);
    r.finalize();
    assert_eq!(r.greenlet_count(), 1);
}

#[test]
fn readable_fd_reenters_while_callback_waits_matches_python() {
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py.exec(r#"
import os, reactor
r = reactor.Reactor()
rd, wr = os.pipe()
os.write(wr, b'x')
events = []
completion = r.completion()
def read(eventtime):
    if not events:
        events.append('outer-start')
        completion.wait(r.monotonic() + 1.)
        events.append('outer-end')
        r.end()
    else:
        events.append('inner')
        os.read(rd, 1)
        r.unregister_fd(handler)
        completion.complete(True)
handler = r.register_fd(rd, read)
r.run()
r.finalize()
os.close(rd)
os.close(wr)
"#).expect("Python reentrant fd");

    let r = fresh();
    let (rd, wr) = make_pipe().unwrap();
    fd::write(wr, b"x").unwrap();
    let events = Arc::new(Mutex::new(Vec::new()));
    let out = events.clone();
    let handler = Arc::new(Mutex::new(None::<FdHandler>));
    let handler2 = handler.clone();
    let completion = r.completion::<bool>();
    let rc = r.clone();
    *handler.lock() = Some(r.register_fd(rd, move |_| {
        let outer = out.lock().is_empty();
        if outer {
            out.lock().push(recording::Value::str("outer-start"));
            completion.wait(rc.monotonic() + 1.);
            out.lock().push(recording::Value::str("outer-end"));
            rc.end();
        } else {
            out.lock().push(recording::Value::str("inner"));
            fd::read(rd, &mut [0u8; 1]).unwrap();
            rc.unregister_fd(handler2.lock().as_ref().unwrap());
            completion.complete(true);
        }
    }, None::<fn(f64)>));
    run_with_timeout(&r);
    r.finalize();
    fd::close(rd);
    fd::close(wr);
    assert_eq!(py.eval("events").unwrap(), recording::Value::List(events.lock().clone()));
}

#[test]
fn fd_unregister_during_read_keeps_current_write_matches_python() {
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py.exec(r#"
import os, reactor
r = reactor.Reactor()
rd, wr = os.pipe()
events = []
def read(eventtime):
    events.append('read')
    r.unregister_fd(handler)
handler = r.register_fd(rd, read, lambda eventtime: events.append('write'))
r.set_fd_wake(handler, True, True)
r._check_fds(0., [(rd, r._READ | r._WRITE), (rd, r._READ | r._WRITE)])
r.finalize()
os.close(rd)
os.close(wr)
"#).expect("Python unregister during fd dispatch");

    let r = fresh();
    let (rd, wr) = make_pipe().unwrap();
    let events = Arc::new(Mutex::new(Vec::new()));
    let reads = events.clone();
    let writes = events.clone();
    let handler = Arc::new(Mutex::new(None::<FdHandler>));
    let handler2 = handler.clone();
    let rc = r.clone();
    let h = r.register_fd(rd, move |_| {
        reads.lock().push(recording::Value::str("read"));
        rc.unregister_fd(handler2.lock().as_ref().unwrap());
    }, Some(move |_| {
        writes.lock().push(recording::Value::str("write"));
    }));
    r.set_fd_wake(&h, true, true);
    *handler.lock() = Some(h);
    r.check_fds(0., &[(rd, POLLIN | POLLOUT), (rd, POLLIN | POLLOUT)]);
    fd::close(rd);
    fd::close(wr);
    assert_eq!(py.eval("events").unwrap(), recording::Value::List(events.lock().clone()));
}

#[test]
fn callback_panics_leave_run_and_allow_restart_matches_python() {
    use recording::Value;
    for is_fd in [false, true] {
        for pauses in [false, true] {
            let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
            py.exec(&format!(r#"
import os, reactor
r = reactor.Reactor()
events = []
rd, wr = os.pipe()
def fail(eventtime):
    events.append('callback')
    if {pauses}:
        if {is_fd}:
            os.read(rd, 1)
        r.pause(r.monotonic() + .001)
    raise ValueError('callback failed')
if {is_fd}:
    os.write(wr, b'x')
    handler = r.register_fd(rd, fail)
else:
    r.register_timer(fail, r.NOW)
try:
    r.run()
except Exception as error:
    events.append(type(error).__name__ + ': ' + str(error))
events.append(r._g_dispatch is None)
if {is_fd}:
    r.unregister_fd(handler)
def shutdown(eventtime):
    events.append('shutdown')
    r.end()
r.register_callback(shutdown)
r.run()
r.finalize()
os.close(rd)
os.close(wr)
"#, is_fd = if is_fd { "True" } else { "False" },
                pauses = if pauses { "True" } else { "False" })).expect("Python callback failure");
            let r = fresh();
            let events = Arc::new(Mutex::new(Vec::new()));
            let out = events.clone();
            let (rd, wr) = make_pipe().unwrap();
            let rc = r.clone();
            let fail = move |_| {
                out.lock().push(Value::str("callback"));
                if pauses {
                    if is_fd {
                        fd::read(rd, &mut [0u8; 1]).unwrap();
                    }
                    rc.pause(rc.monotonic() + 0.001);
                }
                panic!("ValueError: callback failed");
            };
            let handler = if is_fd {
                fd::write(wr, b"x").unwrap();
                Some(r.register_fd(rd, fail, None::<fn(f64)>))
            } else {
                r.register_timer(move |eventtime| { fail(eventtime); NEVER }, NOW);
                None
            };
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run_with_timeout(&r)));
            if let Err(payload) = result {
                let text = payload.downcast_ref::<String>().map(String::as_str)
                    .or_else(|| payload.downcast_ref::<&str>().copied()).expect("exception text");
                events.lock().push(Value::str(text));
            }
            events.lock().push(Value::Bool(r.inner.mu.lock().g_dispatch.is_none()));
            if let Some(handler) = handler {
                r.unregister_fd(&handler);
            }
            let out = events.clone();
            let rc = r.clone();
            r.register_callback(move |_| {
                out.lock().push(Value::str("shutdown"));
                rc.end();
            }, NOW);
            run_with_timeout(&r);
            r.finalize();
            fd::close(rd);
            fd::close(wr);
            assert_eq!(py.eval("events").unwrap(), Value::List(events.lock().clone()),
                "fd={is_fd}, pauses={pauses}");
        }
    }
}

#[test]
fn pthread_panic_ends_only_worker_matches_python() {
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py.exec(r#"
import contextlib, io, threading
def fail():
    raise ValueError('worker failed')
with contextlib.redirect_stderr(io.StringIO()):
    worker = threading.Thread(target=fail)
    worker.start()
    worker.join()
worker_finished = not worker.is_alive()
"#).expect("Python worker failure");
    let worker = kthread::spawn(|| panic!("ValueError: worker failed"));
    worker.join();
    assert_eq!(py.eval("worker_finished").unwrap(), recording::Value::Bool(true));
}

#[test]
fn timer_unregister_self_and_mid_pass_matches_python() {
    use recording::Value;
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py.exec(r#"
import reactor
r = reactor.Reactor()
events = []
counts = []
def first(eventtime):
    events.append('first')
    r.unregister_timer(first_timer)
    r.unregister_timer(removed_timer)
    r.register_timer(lambda eventtime: events.append('new') or r.NEVER, r.NOW)
    return r.NOW
first_timer = r.register_timer(first, r.NOW)
removed_timer = r.register_timer(lambda eventtime: events.append('removed') or r.NEVER, r.NOW)
r.register_timer(lambda eventtime: events.append('last') or r.NEVER, r.NOW)
for eventtime in [1., 2., 3.]:
    r._check_timers(eventtime, False)
    counts.append(len(r._timers))
"#).expect("Python unregister during timer dispatch");

    let r = fresh();
    let events = Arc::new(Mutex::new(Vec::new()));
    let first_slot = Arc::new(Mutex::new(None::<Timer>));
    let removed_slot = Arc::new(Mutex::new(None::<Timer>));
    let first2 = first_slot.clone();
    let removed2 = removed_slot.clone();
    let out = events.clone();
    let rc = r.clone();
    *first_slot.lock() = Some(r.register_timer(move |_| {
        out.lock().push(Value::str("first"));
        rc.unregister_timer(first2.lock().as_ref().unwrap());
        rc.unregister_timer(removed2.lock().as_ref().unwrap());
        let out = out.clone();
        rc.register_timer(move |_| { out.lock().push(Value::str("new")); NEVER }, NOW);
        NOW
    }, NOW));
    let out = events.clone();
    *removed_slot.lock() = Some(r.register_timer(move |_| {
        out.lock().push(Value::str("removed"));
        NEVER
    }, NOW));
    let out = events.clone();
    r.register_timer(move |_| { out.lock().push(Value::str("last")); NEVER }, NOW);
    let mut counts = Vec::new();
    for eventtime in [1., 2., 3.] {
        r.check_timers(eventtime, false);
        counts.push(Value::Int(r.inner.mu.lock().timers.len() as i64));
    }
    assert_eq!(py.eval("events").unwrap(), Value::List(events.lock().clone()));
    assert_eq!(py.eval("counts").unwrap(), Value::List(counts));
}

#[test]
fn timer_storage_stays_bounded_matches_python() {
    use recording::Value;
    let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
    py.exec(r#"
import reactor
r = reactor.Reactor()
survivor = r.register_timer(lambda eventtime: r.NEVER)
counts = []
for cycle in range(1000):
    timer = r.register_timer(lambda eventtime: r.NEVER)
    r.unregister_timer(timer)
    r.register_callback(lambda eventtime: None)
    r._check_timers(float(cycle + 1), False)
    counts.append(len(r._timers))
r.update_timer(survivor, r.NOW)
r._check_timers(1001., False)
counts.append(len(r._timers))
"#).expect("Python repeated timer registration");

    let r = fresh();
    let survivor = r.register_timer(|_| NEVER, NEVER);
    let mut counts = Vec::new();
    for cycle in 0..1000 {
        let timer = r.register_timer(|_| NEVER, NEVER);
        r.unregister_timer(&timer);
        r.register_callback(|_| (), NOW);
        r.check_timers(f64::from(cycle + 1), false);
        counts.push(Value::Int(r.inner.mu.lock().timers.len() as i64));
    }
    r.update_timer(&survivor, NOW);
    r.check_timers(1001., false);
    counts.push(Value::Int(r.inner.mu.lock().timers.len() as i64));
    assert_eq!(py.eval("counts").unwrap(), Value::List(counts));
}

#[test]
fn finalize_unwinds_paused_callback_matches_python() {
    use recording::Value;
    struct OnDrop(Arc<Mutex<Vec<Value>>>);
    impl Drop for OnDrop {
        fn drop(&mut self) {
            self.0.lock().push(Value::str("finally"));
        }
    }
    for resume_first in [false, true] {
        let mut py = pyoracle::Oracle::start().expect("the Python oracle is required");
        py.exec(&format!(r#"
import reactor
r = reactor.Reactor()
events = []
def paused(eventtime):
    events.append('before')
    try:
        if {resume_first}:
            r.pause(r.NOW)
            events.append('resumed')
            r.register_callback(lambda eventtime: r.end())
        r.pause(r.NEVER)
        events.append('after')
    finally:
        events.append('finally')
r.register_callback(paused)
if not {resume_first}:
    r.register_callback(lambda eventtime: r.end())
r.run()
r.finalize()
"#, resume_first = if resume_first { "True" } else { "False" }))
            .expect("Python finalize suspended callback");

        let r = fresh();
        let events = Arc::new(Mutex::new(Vec::new()));
        let out = events.clone();
        let rc = r.clone();
        r.register_callback(move |_| {
            out.lock().push(Value::str("before"));
            let _on_drop = OnDrop(out.clone());
            if resume_first {
                rc.pause(NOW);
                out.lock().push(Value::str("resumed"));
                let stop = rc.clone();
                rc.register_callback(move |_| stop.end(), NOW);
            }
            rc.pause(NEVER);
            out.lock().push(Value::str("after"));
        }, NOW);
        if !resume_first {
            let stop = r.clone();
            r.register_callback(move |_| stop.end(), NOW);
        }
        run_with_timeout(&r);
        r.finalize();
        assert_eq!(py.eval("events").unwrap(), Value::List(events.lock().clone()),
            "resume_first={resume_first}");
    }
}

// `gc.get_count()` model: generation 0 fills after 700 idle checks, a
// collection clears the collected generations, and every 10th
// generation-0 collection promotes the next one to level 1. A counter
// that is never cleared makes every idle check after the 700th collect,
// so `_check_timers` returns 0 forever and the dispatch loop spins.
#[test]
fn gc_check_collects_once_per_generation_fill() {
    let r = Reactor::new(true);
    let levels = Arc::new(Mutex::new(Vec::new()));
    let seen = Arc::clone(&levels);
    r.set_gc_hook(move |level| seen.lock().push(level));
    let mut fired_at = Vec::new();
    for check in 1..=(700 * 11) {
        if r.check_gc(check as f64) {
            fired_at.push(check);
        }
    }
    let expected: Vec<i64> = (1..=11).map(|n| 700 * n).collect();
    assert_eq!(fired_at, expected);
    let mut expected_levels = vec![0; 10];
    expected_levels.push(1);
    assert_eq!(*levels.lock(), expected_levels);
    assert_eq!(r.get_gc_stats(), [7000., 7700., 0.]);
}
