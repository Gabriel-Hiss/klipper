//! Parity tests for `core::clocksync` (port of `klippy/clocksync.py`).
//!
//! Everything is compared against the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a fresh
//! `ClockSync` / `SecondarySync` is built on both sides with minimal
//! Python fakes for the reactor and the serial link, fed identical
//! `get_uptime` / `get_clock` responses, and the resulting state
//! (`dump_debug`, `stats`), the clock conversions, the
//! `set_clock_est` calls and the log lines are compared bit-for-bit
//! (floats by bits via the recording value format).
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply; expected values come
//! from the oracle alone.

use klippy_core::clocksync::{ClockParams, ClockSerialApi, ClockSync, SecondarySync};
use klippy_api::error::KResult;
use klippy_api::log;
use kos::sync::Mutex;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import clocksync
import logging as _logging

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb)
        return ('timer', len(self.timers) - 1)
    def update_timer(self, timer, waketime): pass
    def pause(self, waketime): return waketime
    def monotonic(self): return 1000.0

class FakeMsgParser:
    def __init__(self, freq): self.freq = freq
    def get_constant_float(self, name):
        assert name == 'CLOCK_FREQ'
        return self.freq
    def create_command(self, msg):
        assert msg == 'get_clock'
        return ('cmd', msg)

class FakeSerial:
    def __init__(self, freq, responses):
        self.msgparser = FakeMsgParser(freq)
        self.responses = list(responses)
        self.est_calls = []
        self.raw_sends = []
        self.callbacks = {}
        self._cq = 0
    def get_msgparser(self): return self.msgparser
    def send_with_response(self, msg, response):
        name, params = self.responses.pop(0)
        assert name == msg, (name, msg)
        return params
    def alloc_command_queue(self):
        self._cq += 1
        return ('cq', self._cq)
    def register_response(self, cb, name, oid=None):
        self.callbacks[name] = cb
    def raw_send(self, cmd, minclock, reqclock, cmd_queue):
        self.raw_sends.append((cmd, minclock, reqclock, cmd_queue))
    def set_clock_est(self, freq, conv_time, conv_clock):
        self.est_calls.append((freq, conv_time, conv_clock))

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

_reactor = FakeReactor()

def _step(clock, sent, recv):
    sync._handle_clock({'clock': int(clock), '#sent_time': float(sent), '#receive_time': float(recv)})
    return (sync.dump_debug(), sync.stats(0.), sync.get_clock(10.),
            sync.estimated_print_time(10.), sync.clock_to_print_time(123456),
            sync.is_active())
"#;

const FREQ: f64 = 12000000.0;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn fresh_reactor() -> Reactor {
    Reactor::new(false)
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("clocksync fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping clocksync oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

#[allow(clippy::type_complexity)]
struct FakeSerial {
    freq: f64,
    responses: Mutex<VecDeque<ClockParams>>,
    est_calls: Mutex<Vec<(f64, f64, i64)>>,
    raw_sends: Mutex<Vec<(i64, i64, i64, i64)>>,
    cmd: i64,
    cq: i64,
    callbacks: Mutex<Vec<(String, Box<dyn Fn(ClockParams) + Send + Sync>)>>,
}

impl FakeSerial {
    fn with_responses(freq: f64, responses: Vec<ClockParams>) -> FakeSerial {
        FakeSerial {
            freq,
            responses: Mutex::new(responses.into()),
            est_calls: Mutex::new(Vec::new()),
            raw_sends: Mutex::new(Vec::new()),
            cmd: 7,
            cq: 42,
            callbacks: Mutex::new(Vec::new()),
        }
    }
    fn est_calls(&self) -> Vec<(f64, f64, i64)> {
        self.est_calls.lock().clone()
    }
    fn fire(&self, name: &str, params: ClockParams) {
        let cbs = self.callbacks.lock();
        for (n, cb) in cbs.iter() {
            if n == name {
                cb(params);
                return;
            }
        }
        panic!("no {name} callback registered");
    }
}

impl ClockSerialApi for FakeSerial {
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        assert_eq!(name, "CLOCK_FREQ");
        Ok(self.freq)
    }
    fn send_with_response(&self, msg: &str, response: &str) -> KResult<ClockParams> {
        assert!(
            matches!((msg, response), ("get_uptime", "uptime") | ("get_clock", "clock")),
            "send_with_response({msg}, {response})"
        );
        Ok(self.responses.lock().pop_front().expect("queued response"))
    }
    fn create_command(&self, msg: &str) -> KResult<i64> {
        assert_eq!(msg, "get_clock");
        Ok(self.cmd)
    }
    fn alloc_command_queue(&self) -> i64 {
        self.cq
    }
    fn register_response(&self, name: &str, cb: Box<dyn Fn(ClockParams) + Send + Sync>) {
        self.callbacks.lock().push((name.to_string(), cb));
    }
    fn raw_send(&self, cmd: i64, minclock: i64, reqclock: i64, cmd_queue: i64) {
        self.raw_sends.lock().push((cmd, minclock, reqclock, cmd_queue));
    }
    fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: i64) {
        self.est_calls.lock().push((freq, conv_time, conv_clock));
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn trunc_i64(f: f64) -> i64 {
    pycompat::text::trunc_float_to_bigint(f)
        .0
        .to_string()
        .parse::<i64>()
        .expect("fits")
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "clocksync".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn step_value(sync: &ClockSync) -> Value {
    Value::Tuple(vec![
        Value::str(&sync.dump_debug()),
        Value::str(&sync.stats(0.)),
        Value::int(sync.get_clock(10.)),
        Value::float(sync.estimated_print_time(10.)),
        Value::float(sync.clock_to_print_time(123456)),
        Value::Bool(sync.is_active()),
    ])
}

fn est_value(calls: &[(f64, f64, i64)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(f, t, c)| Value::Tuple(vec![Value::float(*f), Value::float(*t), Value::int(*c)]))
            .collect(),
    )
}

/// Connects a fresh pair of ClockSync objects (Rust and Python) in
/// file mode at 12 MHz and returns them plus shared handles.
fn file_synced(
    py: &mut pyoracle::Oracle,
) -> (ClockSync, Arc<FakeSerial>) {
    let rust_serial = Arc::new(FakeSerial::with_responses(FREQ, Vec::new()));
    let rust = ClockSync::new(fresh_reactor());
    rust.connect_file(rust_serial.clone(), false).expect("connect_file");
    py.exec("sync = clocksync.ClockSync(_reactor)")
        .expect("py ClockSync");
    py.exec("ser = FakeSerial(12000000.0, [])").expect("py serial");
    py.exec("sync.connect_file(ser)").expect("py connect_file");
    (rust, rust_serial)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn initial_state_matches_python() {
    let Some(mut py) = oracle() else { return };
    let rust = ClockSync::new(fresh_reactor());
    py.exec("sync = clocksync.ClockSync(_reactor)").expect("py ClockSync");
    check(
        "initial dump_debug",
        py.eval("sync.dump_debug()"),
        Value::str(&rust.dump_debug()),
    );
    check(
        "initial stats",
        py.eval("sync.stats(0.)"),
        Value::str(&rust.stats(0.)),
    );
    check(
        "initial conversions",
        py.eval(
            "(sync.print_time_to_clock(1.5), sync.print_time_to_clock(-1.5), \
             sync.clock_to_print_time(5), sync.get_clock(100.), sync.get_clock(-0.5), \
             sync.estimated_print_time(100.), sync.is_active())",
        ),
        Value::Tuple(vec![
            Value::int(rust.print_time_to_clock(1.5)),
            Value::int(rust.print_time_to_clock(-1.5)),
            Value::float(rust.clock_to_print_time(5)),
            Value::int(rust.get_clock(100.)),
            Value::int(rust.get_clock(-0.5)),
            Value::float(rust.estimated_print_time(100.)),
            Value::Bool(rust.is_active()),
        ]),
    );
}

#[test]
fn connect_file_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, rust_serial) = file_synced(&mut py);
    // `set_clock_est` conv_time is the reactor monotonic time, which
    // runs in real time on the Rust side and is fixed at 1000.0 on
    // the Python side, so only freq/conv_clock are compared.
    let calls = rust_serial.est_calls();
    assert_eq!(calls.len(), 1, "connect_file emits one set_clock_est");
    assert_eq!(calls[0].0, 1000000000000.0, "connect_file freq 1e12");
    assert_eq!(calls[0].2, 0, "connect_file conv_clock 0");
    check(
        "file dump_debug",
        py.eval("sync.dump_debug()"),
        Value::str(&rust.dump_debug()),
    );
    check(
        "file stats",
        py.eval("sync.stats(0.)"),
        Value::str(&rust.stats(0.)),
    );
    check(
        "file conversions",
        py.eval(
            "(sync.print_time_to_clock(1.5), sync.print_time_to_clock(-1.5), \
             sync.clock_to_print_time(36000000), sync.get_clock(2.), sync.get_clock(0.0000001), \
             sync.estimated_print_time(2.), sync.estimate_clock_systime(24000000), sync.is_active())",
        ),
        Value::Tuple(vec![
            Value::int(rust.print_time_to_clock(1.5)),
            Value::int(rust.print_time_to_clock(-1.5)),
            Value::float(rust.clock_to_print_time(36000000)),
            Value::int(rust.get_clock(2.)),
            Value::int(rust.get_clock(0.0000001)),
            Value::float(rust.estimated_print_time(2.)),
            Value::float(rust.estimate_clock_systime(24000000)),
            Value::Bool(rust.is_active()),
        ]),
    );
    let (py_freq, py_clock) = match py.eval("[c[0] for c in ser.est_calls]") {
        Ok(Value::List(items)) => {
            assert_eq!(items.len(), 1, "python connect_file est_calls");
            match &items[0] {
                Value::Float(f) => (f.to_f64().expect("f64"), 0i64),
                other => panic!("unexpected {other:?}"),
            }
        }
        other => panic!("unexpected {other:?}"),
    };
    assert_eq!(py_freq, 1000000000000.0, "python connect_file freq 1e12");
    assert_eq!(py_clock, 0, "python connect_file conv_clock 0");
}

#[test]
fn connect_file_paced_matches_python() {
    let Some(mut py) = oracle() else { return };
    let rust_serial = Arc::new(FakeSerial::with_responses(FREQ, Vec::new()));
    let rust = ClockSync::new(fresh_reactor());
    rust.connect_file(rust_serial.clone(), true).expect("connect_file pace");
    py.exec("sync = clocksync.ClockSync(_reactor)").expect("py ClockSync");
    py.exec("ser = FakeSerial(12000000.0, [])").expect("py serial");
    py.exec("sync.connect_file(ser, pace=True)").expect("py connect_file pace");
    let calls = rust_serial.est_calls();
    assert_eq!(calls.len(), 1, "paced connect_file emits one set_clock_est");
    assert_eq!(calls[0].0, FREQ, "paced freq == mcu_freq");
    assert_eq!(calls[0].2, 0, "paced conv_clock 0");
    check(
        "paced dump_debug",
        py.eval("sync.dump_debug()"),
        Value::str(&rust.dump_debug()),
    );
}

#[test]
fn handle_clock_regression_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, rust_serial) = file_synced(&mut py);
    rust_serial.est_calls.lock().clear();
    py.exec("ser.est_calls.clear()").expect("clear est_calls");
    let samples = [
        (2_400_000i64, 0.2f64, 0.2003f64),
        (4_800_000, 0.4, 0.4003),
        (7_200_000, 0.6, 0.6003),
    ];
    for (i, &sample) in samples.iter().enumerate() {
        rust.handle_clock(ClockParams {
            high: 0,
            clock: sample.0,
            sent_time: sample.1,
            receive_time: sample.2,
        });
        check(
            &format!("step {i} state"),
            py.eval(&format!("_step({}, {}, {})", sample.0, sample.1, sample.2)),
            step_value(&rust),
        );
        check(
            &format!("step {i} set_clock_est"),
            py.eval("list(ser.est_calls)"),
            est_value(&rust_serial.est_calls()),
        );
    }
}

#[test]
fn handle_clock_outlier_ignoring_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, rust_serial) = file_synced(&mut py);
    rust_serial.est_calls.lock().clear();
    py.exec("ser.est_calls.clear()").expect("clear est_calls");
    // The second sample jumps far ahead of the prediction with
    // sent_time < last_prediction_time + 10: it must be ignored
    // (no set_clock_est call) after the first sample's new-minimum
    // round-trip log.
    let good = (2_400_000i64, 0.2f64, 0.2003f64);
    rust.handle_clock(ClockParams {
        high: 0,
        clock: good.0,
        sent_time: good.1,
        receive_time: good.2,
    });
    py.eval(&format!("_step({}, {}, {})", good.0, good.1, good.2))
        .expect("py step1");
    let bad = (200_000_000i64, 2.0f64, 2.0003f64);
    rust.handle_clock(ClockParams {
        high: 0,
        clock: bad.0,
        sent_time: bad.1,
        receive_time: bad.2,
    });
    check(
        "ignoring state",
        py.eval(&format!("_step({}, {}, {})", bad.0, bad.1, bad.2)),
        step_value(&rust),
    );
    // One set_clock_est (from the first sample) and no more.
    assert_eq!(
        rust_serial.est_calls().len(),
        1,
        "ignored sample makes no set_clock_est call"
    );
    check(
        "ignoring set_clock_est",
        py.eval("list(ser.est_calls)"),
        est_value(&rust_serial.est_calls()),
    );
}

#[test]
fn handle_clock_outlier_resetting_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, rust_serial) = file_synced(&mut py);
    rust_serial.est_calls.lock().clear();
    py.exec("ser.est_calls.clear()").expect("clear est_calls");
    // The second sample is far below the prediction with
    // sent_time >= last_prediction_time + 10: prediction variance is
    // reset and the regression still runs (set_clock_est is called).
    let good = (2_400_000i64, 0.2f64, 0.2003f64);
    rust.handle_clock(ClockParams {
        high: 0,
        clock: good.0,
        sent_time: good.1,
        receive_time: good.2,
    });
    py.eval(&format!("_step({}, {}, {})", good.0, good.1, good.2))
        .expect("py step1");
    let bad = (5_000_000i64, 25.0f64, 25.0003f64);
    rust.handle_clock(ClockParams {
        high: 0,
        clock: bad.0,
        sent_time: bad.1,
        receive_time: bad.2,
    });
    check(
        "resetting state",
        py.eval(&format!("_step({}, {}, {})", bad.0, bad.1, bad.2)),
        step_value(&rust),
    );
    assert_eq!(
        rust_serial.est_calls().len(),
        2,
        "reset path still feeds the regression"
    );
    check(
        "resetting set_clock_est",
        py.eval("list(ser.est_calls)"),
        est_value(&rust_serial.est_calls()),
    );
}

#[test]
fn handle_clock_zero_sent_time_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, rust_serial) = file_synced(&mut py);
    rust_serial.est_calls.lock().clear();
    py.exec("ser.est_calls.clear()").expect("clear est_calls");
    // A zero sent_time extends last_clock and resets queries_pending
    // but returns before the regression: no set_clock_est call.
    rust.handle_clock(ClockParams {
        high: 0,
        clock: 123456,
        sent_time: 0.0,
        receive_time: 0.0,
    });
    check(
        "zero sent_time state",
        py.eval("_step(123456, 0.0, 0.0)"),
        step_value(&rust),
    );
    assert!(
        rust_serial.est_calls().is_empty(),
        "zero sent_time makes no set_clock_est call"
    );
}

#[test]
fn handle_clock_wraps_32bit_matches_python() {
    let Some(mut py) = oracle() else { return };
    let (rust, rust_serial) = file_synced(&mut py);
    // A clock near 2^32-1 followed by a wrapped clock: last_clock
    // must carry into the 64-bit range identically.
    rust.handle_clock(ClockParams {
        high: 0,
        clock: 4294967000,
        sent_time: 0.2,
        receive_time: 0.2003,
    });
    py.eval("_step(4294967000, 0.2, 0.2003)").expect("py wrap step1");
    let wrapped = ClockParams {
        high: 0,
        clock: 100,
        sent_time: 0.4,
        receive_time: 0.4003,
    };
    rust.handle_clock(wrapped);
    check(
        "wrapped state",
        py.eval("_step(100, 0.4, 0.4003)"),
        step_value(&rust),
    );
    for clock32 in [50i64, 3000000000] {
        check(
            &format!("clock32_to_clock64({clock32})"),
            py.eval(&format!("sync.clock32_to_clock64({clock32})")),
            Value::int(rust.clock32_to_clock64(clock32)),
        );
    }
    let _ = rust_serial;
}

#[test]
fn connect_matches_python() {
    let Some(mut py) = oracle() else { return };
    // Initial uptime followed by eight well-behaved get_clock
    // responses priming the regression (the 8 x 0.05s pauses on the
    // Rust side sleep in real time).
    let mut responses = vec![ClockParams {
        high: 0,
        clock: 240000,
        sent_time: 0.1,
        receive_time: 0.1003,
    }];
    for i in 1..=8i64 {
        let sent = 0.1 + 0.05 * i as f64;
        responses.push(ClockParams {
            high: 0,
            clock: 240000 + 600000 * i,
            sent_time: sent,
            receive_time: sent + 0.0003,
        });
    }
    let rust_serial = Arc::new(FakeSerial::with_responses(FREQ, responses));
    let rust = ClockSync::new(fresh_reactor());
    rust.connect(rust_serial.clone()).expect("connect");

    py.exec("sync = clocksync.ClockSync(_reactor)").expect("py ClockSync");
    py.exec(
        "resp = [('get_uptime', {'high': 0, 'clock': 240000, '#sent_time': 0.1})]\n\
         for i in range(1, 9):\n\
         \x20   st_ = 0.1 + 0.05 * i\n\
         \x20   resp.append(('get_clock', {'clock': 240000 + 600000 * i, \
         '#sent_time': st_, '#receive_time': st_ + 0.0003}))",
    )
    .expect("py responses");
    py.exec("ser = FakeSerial(12000000.0, resp)").expect("py serial");
    py.exec("sync.connect(ser)").expect("py connect");

    assert_eq!(rust_serial.est_calls().len(), 8, "eight priming samples");
    check(
        "connect dump_debug",
        py.eval("sync.dump_debug()"),
        Value::str(&rust.dump_debug()),
    );
    check(
        "connect set_clock_est",
        py.eval("list(ser.est_calls)"),
        est_value(&rust_serial.est_calls()),
    );
    // The `clock` response callback was registered: fire it with a
    // sample predicted from each side's own clock estimate.
    let est_r = rust.clock_est();
    let pred = trunc_i64((1.0 - est_r.0) * est_r.2 + est_r.1);
    check(
        "callback registered",
        py.eval("'clock' in ser.callbacks"),
        Value::Bool(true),
    );
    py.exec(&format!(
        "ser.callbacks['clock']({{'clock': {pred}, '#sent_time': 1.0, '#receive_time': 1.0003}})"
    ))
    .expect("py fire callback");
    rust_serial.fire(
        "clock",
        ClockParams {
            high: 0,
            clock: pred,
            sent_time: 1.0,
            receive_time: 1.0003,
        },
    );
    check(
        "after callback dump_debug",
        py.eval("sync.dump_debug()"),
        Value::str(&rust.dump_debug()),
    );
    // Each period: one raw_send, queries_pending grows past 4 after
    // five events (is_active() == queries_pending <= 4).
    let next_r = rust.get_clock_event(10.0);
    let next_p = py.eval("sync._get_clock_event(10.)").expect("py get_clock_event");
    check(
        "get_clock_event next",
        Ok(next_p),
        Value::float(next_r),
    );
    check(
        "get_clock_event raw_send",
        py.eval("ser.raw_sends[-1][1:3]"),
        Value::Tuple(vec![Value::int(0), Value::int(0)]),
    );
    assert_eq!(
        *rust_serial.raw_sends.lock(),
        vec![(rust_serial.cmd, 0, 0, rust_serial.cq)],
        "raw_send(cmd, 0, 0, cmd_queue)"
    );
    for _ in 0..4 {
        let _ = rust.get_clock_event(10.0);
        py.eval("sync._get_clock_event(10.)").expect("py get_clock_event");
    }
    check("is_active after 5 queries", py.eval("sync.is_active()"), Value::Bool(rust.is_active()));
    assert!(!rust.is_active(), "5 pending queries is not active");
}

#[test]
fn secondary_init_and_connect_file_matches_python() {
    let Some(mut py) = oracle() else { return };
    let rust = ClockSync::new(fresh_reactor());
    let rust_serial = Arc::new(FakeSerial::with_responses(FREQ, Vec::new()));
    let sec = SecondarySync::new(fresh_reactor(), rust.clone());
    py.exec("main = clocksync.ClockSync(_reactor)").expect("py main");
    py.exec("sec = clocksync.SecondarySync(_reactor, main)").expect("py secondary");

    // clock_adj = (0., 1.) before connect: conversions are identity.
    check(
        "secondary init dump_debug",
        py.eval("sec.dump_debug()"),
        Value::str(&sec.dump_debug()),
    );
    check(
        "secondary init stats",
        py.eval("sec.stats(0.)"),
        Value::str(&sec.stats(0.)),
    );
    check(
        "secondary init conversions",
        py.eval(
            "(sec.print_time_to_clock(2.0), sec.clock_to_print_time(3), \
             sec.get_clock(100.), sec.estimated_print_time(100.), \
             sec.is_active())",
        ),
        Value::Tuple(vec![
            Value::int(sec.print_time_to_clock(2.0)),
            Value::float(sec.clock_to_print_time(3)),
            Value::int(sec.get_clock(100.)),
            Value::float(sec.estimated_print_time(100.)),
            Value::Bool(sec.is_active()),
        ]),
    );
    // connect_file sets clock_adj = (0., mcu_freq).
    rust.connect_file(rust_serial.clone(), false).expect("rust main connect_file");
    py.exec("main_ser = FakeSerial(12000000.0, [])").expect("py serial");
    py.exec("main.connect_file(main_ser)").expect("py main connect_file");
    sec.connect_file(rust_serial, false).expect("secondary connect_file");
    py.exec("sec_ser = FakeSerial(12000000.0, [])").expect("py sec serial");
    py.exec("sec.connect_file(sec_ser)").expect("py secondary connect_file");
    check(
        "secondary file dump_debug",
        py.eval("sec.dump_debug()"),
        Value::str(&sec.dump_debug()),
    );
    check(
        "secondary file stats",
        py.eval("sec.stats(0.)"),
        Value::str(&sec.stats(0.)),
    );
    check("secondary clock_adj", py.eval("sec.clock_adj"), {
        let (off, freq) = sec.clock_adj();
        Value::Tuple(vec![Value::float(off), Value::float(freq)])
    });
}

#[test]
fn secondary_calibrate_matches_python() {
    let Some(mut py) = oracle() else { return };
    let main = ClockSync::new(fresh_reactor());
    let main_serial = Arc::new(FakeSerial::with_responses(FREQ, Vec::new()));
    let sec_serial = Arc::new(FakeSerial::with_responses(FREQ, Vec::new()));
    main.connect_file(main_serial, false).expect("main connect_file");
    let sec = SecondarySync::new(fresh_reactor(), main.clone());
    sec.connect_file(sec_serial, false).expect("secondary connect_file");

    py.exec("main = clocksync.ClockSync(_reactor)").expect("py main");
    py.exec("main_ser = FakeSerial(12000000.0, [])").expect("py serial");
    py.exec("main.connect_file(main_ser)").expect("py main connect_file");
    py.exec("sec = clocksync.SecondarySync(_reactor, main)").expect("py secondary");
    py.exec("sec_ser = FakeSerial(12000000.0, [])").expect("py sec serial");
    py.exec("sec.connect_file(sec_ser)").expect("py secondary connect_file");

    // Give the primary a non-trivial estimate, then calibrate twice:
    // the second call takes the last_sync_time branch.
    let sample = ClockParams {
        high: 0,
        clock: 6000000,
        sent_time: 0.5,
        receive_time: 0.5003,
    };
    main.handle_clock(sample);
    py.exec("main._handle_clock({'clock': 6000000, '#sent_time': 0.5, '#receive_time': 0.5003})")
        .expect("py main sample");

    let cal1 = sec.calibrate_clock(3.0, 2.0).expect("calibrate 1");
    check(
        "calibrate 1",
        py.eval("sec.calibrate_clock(3.0, 2.0)"),
        Value::Tuple(vec![Value::float(cal1.0), Value::float(cal1.1)]),
    );
    check(
        "calibrate 1 state",
        py.eval(
            "(sec.dump_debug(), sec.stats(0.), sec.clock_adj, sec.last_sync_time, \
             sec.print_time_to_clock(5.0), sec.clock_to_print_time(6000000))",
        ),
        {
            let (off, freq) = sec.clock_adj();
            Value::Tuple(vec![
                Value::str(&sec.dump_debug()),
                Value::str(&sec.stats(0.)),
                Value::Tuple(vec![Value::float(off), Value::float(freq)]),
                Value::float(sec.last_sync_time()),
                Value::int(sec.print_time_to_clock(5.0)),
                Value::float(sec.clock_to_print_time(6000000)),
            ])
        },
    );
    let cal2 = sec.calibrate_clock(8.0, 2.0).expect("calibrate 2");
    check(
        "calibrate 2",
        py.eval("sec.calibrate_clock(8.0, 2.0)"),
        Value::Tuple(vec![Value::float(cal2.0), Value::float(cal2.1)]),
    );
}

/// Captured log lines of the Rust side, installed once per process.
#[allow(clippy::type_complexity)]
fn log_capture() -> Arc<Mutex<Vec<(String, String)>>> {
    use std::sync::LazyLock;
    static CAPTURE: LazyLock<Arc<Mutex<Vec<(String, String)>>>> =
        LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
    let captured = CAPTURE.clone();
    let sink_capture = captured.clone();
    log::set_sink(Box::new(move |rec: &log::Record<'_>| {
        sink_capture
            .lock()
            .push((rec.level.name().to_string(), rec.message.to_string()));
    }));
    captured
}

#[test]
fn handle_clock_log_lines_match_python() {
    let Some(mut py) = oracle() else { return };
    let captured = log_capture();
    let (rust, rust_serial) = file_synced(&mut py);
    rust_serial.est_calls.lock().clear();
    py.exec("ser.est_calls.clear()").expect("clear est_calls");
    captured.lock().clear();
    py.exec("log_lines.clear()").expect("clear py log");

    // One sample per line: minimum rtt, outlier-ignoring (after its
    // own minimum-rtt line), outlier-resetting (ditto). The sent
    // times carry a `.421` marker that no other test uses, so the
    // capture filter ignores foreign lines in this process.
    let samples = [
        (2_369_052_000i64, 197.421f64, 197.4213f64),
        (2_560_000_000, 198.421, 198.4213),
        (1_000_000, 207.421, 207.4213),
    ];
    for &sample in samples.iter() {
        rust.handle_clock(ClockParams {
            high: 0,
            clock: sample.0,
            sent_time: sample.1,
            receive_time: sample.2,
        });
        py.exec(&format!(
            "_step({}, {}, {})",
            sample.0, sample.1, sample.2
        ))
        .expect("py step");
    }
    check(
        "log state",
        py.eval("(sync.dump_debug(), list(ser.est_calls))"),
        {
            let calls = rust_serial.est_calls();
            Value::Tuple(vec![
                Value::str(&rust.dump_debug()),
                est_value(&calls),
            ])
        },
    );
    let expected: Vec<Value> = match py.eval("list(log_lines)") {
        Ok(Value::List(items)) => items,
        other => panic!("unexpected py log_lines: {other:?}"),
    };
    let rust_lines: Vec<Value> = captured
        .lock()
        .iter()
        .filter(|(_, msg)| msg.contains(".421"))
        .map(|(level, msg)| {
            Value::Tuple(vec![Value::str(level), Value::str(msg)])
        })
        .collect();
    // Python captures the bare message; the (level, message) wrapper
    // is compared by mapping Python lines to the same shape? No:
    // compare bare to bare.
    let py_messages: Vec<Value> = expected;
    let rust_messages: Vec<Value> = rust_lines
        .into_iter()
        .map(|v| match v {
            Value::Tuple(mut items) => items.swap_remove(1),
            other => other,
        })
        .collect();
    check("log lines", Ok(Value::List(py_messages)), Value::List(rust_messages));
}
