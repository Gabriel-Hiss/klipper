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
    r.run();
    done.store(true, Ordering::SeqCst);
    watchdog.join();
    assert!(!fired.load(Ordering::SeqCst), "reactor run timed out after 4s (baton deadlock?)");
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
