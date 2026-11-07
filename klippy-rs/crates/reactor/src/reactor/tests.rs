//! Behavioural tests transcribed from `reactor.py` (do not run here;
//! the integration owner runs them under the vclock).
use super::*;
use std::sync::Arc;

fn fresh() -> Reactor {
    Reactor::new(false)
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
    r.run();
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
    r.run();
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
    let rc = r.clone();
    let at = r.monotonic() + 0.01;
    r.register_callback(move |_| {
        m4.lock();
        l3.lock().push("second");
        assert!(rc.monotonic() > 0.);
        m4.unlock();
    }, at);
    end_timer(&r, r.monotonic() + 0.1);
    r.run();
    assert_eq!(*log.lock(), vec!["first", "second"]);
}

#[test]
fn assert_no_pause_violates() {
    let r = fresh();
    let r2 = r.clone();
    r.register_callback(move |_| {
        let _guard = r2.assert_no_pause();
        let panicked = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| { r2.pause(NEVER); }));
        assert!(panicked.is_err());
        r2.end();
    }, NOW);
    r.run();
}

#[test]
fn pause_returns_eventtime() {
    let r = fresh();
    let r2 = r.clone();
    r.register_callback(move |_| {
        let et = r2.pause(r2.monotonic() + 0.01);
        assert!(et > 0.);
        r2.end();
    }, NOW);
    r.run();
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
    r.run();
    assert!(*done.lock());
}

#[test]
fn nested_pause_cached() {
    let r = fresh();
    let r2 = r.clone();
    r.register_callback(move |_| {
        let a = r2.pause(r2.monotonic() + 0.01);
        let b = r2.pause(r2.monotonic() + 0.02);
        assert!(b >= a);
        assert!(r2.cached_count() >= 1);
        r2.end();
    }, NOW);
    r.run();
}

#[test]
fn end_and_finalize() {
    let r = fresh();
    end_timer(&r, r.monotonic() + 0.01);
    r.run();
    r.finalize();
    assert_eq!(r.greenlet_count(), 1);
}
