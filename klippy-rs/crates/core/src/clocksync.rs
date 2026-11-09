//! Port of the `clocksync` module (`klippy/clocksync.py`).
//!
//! `ClockSync` estimates the MCU clock from periodic `get_clock`
//! responses, and `SecondarySync` maps the primary MCU's print time
//! onto a secondary MCU. The module has no printer object, config
//! options or G-code commands: the Python `ClockSync` /
//! `SecondarySync` constructors are the factory (`klippy/mcu.py`
//! builds them directly), so [`module_factory`] is empty.

use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::SCell;
use klippy_api::log;
use klippy_api::printer::ModuleFactory;
use pycompat::text::{float_true_div, percent_format, trunc_float_to_bigint, PctValue, PyError};
use pycompat::PyInt;
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::Arc;

/// `RTT_AGE` (`klippy/clocksync.py:8`).
pub const RTT_AGE: f64 = 0.000010 / (60. * 60.);
/// `DECAY` (`klippy/clocksync.py:9`).
pub const DECAY: f64 = 1. / 30.;
/// `TRANSMIT_EXTRA` (`klippy/clocksync.py:10`).
pub const TRANSMIT_EXTRA: f64 = 0.001;

/// Parameters of a `clock` / `uptime` response (`klippy/clocksync.py:35`).
///
/// The firmware always supplies `#sent_time` / `#receive_time`
/// (`klippy/serialhdl.py:51`), so both are plain fields; a Python
/// dict missing them would raise `KeyError`, which a real serial
/// adapter never lets through.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockParams {
    /// `high` for `uptime`, else `0`.
    pub high: i64,
    /// `clock`, the raw 32-bit clock.
    pub clock: i64,
    /// `#sent_time`.
    pub sent_time: f64,
    /// `#receive_time`.
    pub receive_time: f64,
}

/// Handle for a message created by `create_command`.
pub type CommandId = i64;
/// Handle for a queue allocated by `alloc_command_queue`.
pub type CommandQueueId = i64;

/// The subset of `SerialReader` (`klippy/serialhdl.py:14`) that
/// `klippy/clocksync.py` calls. The Python code reaches it through
/// `serial.msgparser.get_constant_float` / `serial.get_msgparser()`,
/// which are the same object, so the trait exposes them flattened.
pub trait ClockSerialApi: Send + Sync {
    /// `msgparser.get_constant_float(name)`.
    fn get_constant_float(&self, name: &str) -> KResult<f64>;
    /// `serial.send_with_response(msg, response)` (params dict).
    fn send_with_response(&self, msg: &str, response: &str) -> KResult<ClockParams>;
    /// `serial.get_msgparser().create_command(msg)`.
    fn create_command(&self, msg: &str) -> KResult<CommandId>;
    /// `serial.alloc_command_queue()`.
    fn alloc_command_queue(&self) -> CommandQueueId;
    /// `serial.register_response(cb, name)` with `cb = _handle_clock`.
    fn register_response(&self, name: &str, cb: Box<dyn Fn(ClockParams) + Send + Sync>);
    /// `serial.raw_send(cmd, minclock, reqclock, cmd_queue)`.
    fn raw_send(&self, cmd: CommandId, minclock: i64, reqclock: i64, cmd_queue: CommandQueueId);
    /// `serial.set_clock_est(freq, conv_time, conv_clock)`.
    fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: i64);
}

/// Handle to the serial connection.
pub type SerialHandle = Arc<dyn ClockSerialApi>;

/// Mutable state of `ClockSync`, one `SCell` mirroring the Python
/// instance dict (`klippy/clocksync.py:13`).
struct State {
    mcu_freq: f64,
    last_clock: i64,
    /// `clock_est`: `(sample_time, clock, freq)`.
    clock_est: (f64, f64, f64),
    // Minimum round-trip-time tracking.
    min_half_rtt: f64,
    min_rtt_time: f64,
    // Linear regression of mcu clock and system sent_time.
    time_avg: f64,
    time_variance: f64,
    clock_avg: f64,
    clock_covariance: f64,
    prediction_variance: f64,
    last_prediction_time: f64,
    queries_pending: i64,
}

impl Default for State {
    fn default() -> Self {
        State {
            mcu_freq: 1.,
            last_clock: 0,
            clock_est: (0., 0., 0.),
            min_half_rtt: 999999999.9,
            min_rtt_time: 0.,
            time_avg: 0.,
            time_variance: 0.,
            clock_avg: 0.,
            clock_covariance: 0.,
            prediction_variance: 0.,
            last_prediction_time: 0.,
            queries_pending: 0,
        }
    }
}

/// The state shared with the timer and response callbacks (the
/// Python instance is referenced from both `__init__` and
/// `_get_clock_event`).
struct Shared {
    state: SCell<State>,
    serial: SCell<Option<SerialHandle>>,
    get_clock_cmd: SCell<Option<CommandId>>,
    cmd_queue: SCell<Option<CommandQueueId>>,
}

impl Default for Shared {
    fn default() -> Self {
        Shared {
            state: SCell::new(State::default()),
            serial: SCell::new(None),
            get_clock_cmd: SCell::new(None),
            cmd_queue: SCell::new(None),
        }
    }
}

/// Port of the `ClockSync` class (`klippy/clocksync.py:12`).
pub struct ClockSync {
    reactor: Reactor,
    shared: Arc<Shared>,
    timer: Arc<Timer>,
}

impl Clone for ClockSync {
    fn clone(&self) -> Self {
        ClockSync {
            reactor: self.reactor.clone(),
            shared: Arc::clone(&self.shared),
            timer: Arc::clone(&self.timer),
        }
    }
}

/// `int(float)`: truncation toward zero (`klippy/clocksync.py:124`).
///
/// `trunc_float_to_bigint` covers exact truncation; the i64 conversion
/// clamps for values outside i64 range, which real clocks never reach.
fn py_trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = trunc_float_to_bigint(f).0.to_string();
    digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant clocksync format")
}

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

/// Python `max` with two arguments (first maximal item wins).
fn py_max2(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// `_get_clock_event` (`klippy/clocksync.py:59`).
fn get_clock_event(shared: &Shared, eventtime: f64) -> f64 {
    let serial = shared.serial.borrow().clone().expect("serial connected");
    let cmd = *shared.get_clock_cmd.borrow();
    let cmd_queue = *shared.cmd_queue.borrow();
    serial.raw_send(cmd.expect("get_clock_cmd"), 0, 0, cmd_queue.expect("cmd_queue"));
    shared.state.borrow_mut().queries_pending += 1;
    // Use an unusual time for the next event so clock messages
    // don't resonate with other periodic events.
    eventtime + 0.9839
}

/// `_handle_clock` (`klippy/clocksync.py:65`). Invoked from the
/// background thread in Python; the port keeps the direct call.
fn handle_clock(shared: &Shared, params: ClockParams) {
    let serial = shared.serial.borrow().clone().expect("serial connected");
    let mut st = shared.state.borrow_mut();
    st.queries_pending = 0;
    // Extend clock to 64bit.
    let delta = params.clock.wrapping_sub(st.last_clock) & 0xffff_ffff;
    st.last_clock = st.last_clock.wrapping_add(delta);
    let clock = st.last_clock;
    // Check if this is the best round-trip-time seen so far.
    let sent_time = params.sent_time;
    if sent_time == 0.0 {
        return;
    }
    let receive_time = params.receive_time;
    let half_rtt = 0.5 * (receive_time - sent_time);
    let aged_rtt = (sent_time - st.min_rtt_time) * RTT_AGE;
    if half_rtt < st.min_half_rtt + aged_rtt {
        st.min_half_rtt = half_rtt;
        st.min_rtt_time = sent_time;
        let msg = pct_format(
            "new minimum rtt %.3f: hrtt=%.6f freq=%d",
            &[
                PctValue::Float(sent_time),
                PctValue::Float(half_rtt),
                PctValue::Float(st.clock_est.2),
            ],
        );
        log::debug(&msg);
    }
    // Filter out samples that are extreme outliers.
    let exp_clock = (sent_time - st.time_avg) * st.clock_est.2 + st.clock_avg;
    let clock_diff = clock as f64 - exp_clock;
    let clock_diff2 = clock_diff * clock_diff;
    if clock_diff2 > 25. * st.prediction_variance
        && clock_diff2 > (0.000500 * st.mcu_freq) * (0.000500 * st.mcu_freq)
    {
        if clock_diff > 0. && sent_time < st.last_prediction_time + 10. {
            // `clock > exp_clock` compares the int clock to the float
            // prediction; the float difference carries the same sign
            // for every reachable clock value.
            let stddev = pycompat::libm::sqrt(st.prediction_variance);
            let msg = pct_format(
                "Ignoring clock sample %.3f: freq=%d diff=%d stddev=%.3f",
                &[
                    PctValue::Float(sent_time),
                    PctValue::Float(st.clock_est.2),
                    PctValue::Float(clock_diff),
                    PctValue::Float(stddev),
                ],
            );
            log::debug(&msg);
            return;
        }
        let stddev = pycompat::libm::sqrt(st.prediction_variance);
        let msg = pct_format(
            "Resetting prediction variance %.3f: freq=%d diff=%d stddev=%.3f",
            &[
                PctValue::Float(sent_time),
                PctValue::Float(st.clock_est.2),
                PctValue::Float(clock_diff),
                PctValue::Float(stddev),
            ],
        );
        log::info(&msg);
        st.prediction_variance = (0.001 * st.mcu_freq) * (0.001 * st.mcu_freq);
    } else {
        st.last_prediction_time = sent_time;
        st.prediction_variance =
            (1. - DECAY) * (st.prediction_variance + clock_diff2 * DECAY);
    }
    // Add clock and sent_time to linear regression.
    let diff_sent_time = sent_time - st.time_avg;
    st.time_avg += DECAY * diff_sent_time;
    st.time_variance =
        (1. - DECAY) * (st.time_variance + diff_sent_time * diff_sent_time * DECAY);
    let diff_clock = clock as f64 - st.clock_avg;
    st.clock_avg += DECAY * diff_clock;
    st.clock_covariance = (1. - DECAY)
        * (st.clock_covariance + diff_sent_time * diff_clock * DECAY);
    // Update prediction from linear regression.
    let new_freq = st.clock_covariance / st.time_variance;
    let pred_stddev = pycompat::libm::sqrt(st.prediction_variance);
    let (time_avg, clock_avg, min_half_rtt) = (st.time_avg, st.clock_avg, st.min_half_rtt);
    drop(st);
    serial.set_clock_est(
        new_freq,
        time_avg + TRANSMIT_EXTRA,
        py_trunc_i64(clock_avg - 3. * pred_stddev),
    );
    shared.state.borrow_mut().clock_est = (time_avg + min_half_rtt, clock_avg, new_freq);
}

impl ClockSync {
    /// `ClockSync(reactor)` (`klippy/clocksync.py:13`).
    pub fn new(reactor: Reactor) -> ClockSync {
        let shared = Arc::new(Shared::default());
        let cb_shared = Arc::clone(&shared);
        let timer = Arc::new(reactor.register_timer(
            move |eventtime| get_clock_event(&cb_shared, eventtime),
            NEVER,
        ));
        ClockSync { reactor, shared, timer }
    }

    /// `connect(serial)` (`klippy/clocksync.py:30`).
    pub fn connect(&self, serial: SerialHandle) -> KResult<()> {
        *self.shared.serial.borrow_mut() = Some(Arc::clone(&serial));
        self.shared.state.borrow_mut().mcu_freq =
            serial.get_constant_float("CLOCK_FREQ")?;
        // Load initial clock and frequency.
        let params = serial.send_with_response("get_uptime", "uptime")?;
        let last_clock = params.high.wrapping_shl(32) | params.clock;
        let time_avg = params.sent_time;
        let clock_avg = last_clock as f64;
        {
            let mut st = self.shared.state.borrow_mut();
            st.last_clock = last_clock;
            st.clock_avg = clock_avg;
            st.time_avg = time_avg;
            st.clock_est = (time_avg, clock_avg, st.mcu_freq);
            st.prediction_variance = (0.001 * st.mcu_freq) * (0.001 * st.mcu_freq);
        }
        // Enable periodic get_clock timer.
        for _ in 0..8 {
            self.reactor.pause(self.reactor.monotonic() + 0.050);
            self.shared.state.borrow_mut().last_prediction_time = -9999.;
            let params = serial.send_with_response("get_clock", "clock")?;
            handle_clock(&self.shared, params);
        }
        *self.shared.get_clock_cmd.borrow_mut() = Some(serial.create_command("get_clock")?);
        *self.shared.cmd_queue.borrow_mut() = Some(serial.alloc_command_queue());
        let cb_shared = Arc::clone(&self.shared);
        serial.register_response(
            "clock",
            Box::new(move |params| handle_clock(&cb_shared, params)),
        );
        self.reactor.update_timer(&self.timer, NOW);
        Ok(())
    }

    /// `connect_file(serial, pace=False)` (`klippy/clocksync.py:50`).
    pub fn connect_file(&self, serial: SerialHandle, pace: bool) -> KResult<()> {
        *self.shared.serial.borrow_mut() = Some(Arc::clone(&serial));
        let mcu_freq = serial.get_constant_float("CLOCK_FREQ")?;
        {
            let mut st = self.shared.state.borrow_mut();
            st.mcu_freq = mcu_freq;
            st.clock_est = (0., 0., mcu_freq);
        }
        let freq = if pace { mcu_freq } else { 1000000000000. };
        serial.set_clock_est(freq, self.reactor.monotonic(), 0);
        Ok(())
    }

    /// `_get_clock_event(eventtime)` (`klippy/clocksync.py:59`).
    pub fn get_clock_event(&self, eventtime: f64) -> f64 {
        get_clock_event(&self.shared, eventtime)
    }

    /// `_handle_clock(params)` (`klippy/clocksync.py:65`).
    pub fn handle_clock(&self, params: ClockParams) {
        handle_clock(&self.shared, params);
    }

    /// `print_time_to_clock(print_time)` (`klippy/clocksync.py:123`).
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        py_trunc_i64(print_time * self.shared.state.borrow().mcu_freq)
    }

    /// `clock_to_print_time(clock)` (`klippy/clocksync.py:125`).
    pub fn clock_to_print_time(&self, clock: i64) -> f64 {
        clock as f64 / self.shared.state.borrow().mcu_freq
    }

    /// `get_clock(eventtime)` (`klippy/clocksync.py:128`).
    pub fn get_clock(&self, eventtime: f64) -> i64 {
        let (sample_time, clock, freq) = self.shared.state.borrow().clock_est;
        py_trunc_i64(clock + (eventtime - sample_time) * freq)
    }

    /// `estimate_clock_systime(reqclock)` (`klippy/clocksync.py:131`).
    pub fn estimate_clock_systime(&self, reqclock: i64) -> f64 {
        let (sample_time, clock, freq) = self.shared.state.borrow().clock_est;
        (reqclock as f64 - clock) / freq + sample_time
    }

    /// `estimated_print_time(eventtime)` (`klippy/clocksync.py:134`).
    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.clock_to_print_time(self.get_clock(eventtime))
    }

    /// `clock32_to_clock64(clock32)` (`klippy/clocksync.py:137`).
    pub fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        let last_clock = self.shared.state.borrow().last_clock;
        let mut clock_diff = clock32.wrapping_sub(last_clock) & 0xffff_ffff;
        clock_diff -= (clock_diff & 0x8000_0000) << 1;
        last_clock.wrapping_add(clock_diff)
    }

    /// `is_active()` (`klippy/clocksync.py:142`).
    pub fn is_active(&self) -> bool {
        self.shared.state.borrow().queries_pending <= 4
    }

    /// `dump_debug()` (`klippy/clocksync.py:144`).
    pub fn dump_debug(&self) -> String {
        let st = self.shared.state.borrow();
        let (sample_time, clock, freq) = st.clock_est;
        pct_format(
            "clocksync state: mcu_freq=%d last_clock=%d clock_est=(%.3f %d %.3f) \
             min_half_rtt=%.6f min_rtt_time=%.3f time_avg=%.3f(%.3f) \
             clock_avg=%.3f(%.3f) pred_variance=%.3f",
            &[
                PctValue::Float(st.mcu_freq),
                pct_int(st.last_clock),
                PctValue::Float(sample_time),
                PctValue::Float(clock),
                PctValue::Float(freq),
                PctValue::Float(st.min_half_rtt),
                PctValue::Float(st.min_rtt_time),
                PctValue::Float(st.time_avg),
                PctValue::Float(st.time_variance),
                PctValue::Float(st.clock_avg),
                PctValue::Float(st.clock_covariance),
                PctValue::Float(st.prediction_variance),
            ],
        )
    }

    /// `stats(eventtime)` (`klippy/clocksync.py:155`).
    pub fn stats(&self, _eventtime: f64) -> String {
        pct_format(
            "freq=%d",
            &[PctValue::Float(self.shared.state.borrow().clock_est.2)],
        )
    }

    /// `calibrate_clock(print_time, eventtime)` (`klippy/clocksync.py:158`).
    pub fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0., self.shared.state.borrow().mcu_freq))
    }

    /// `clock_est` (`klippy/clocksync.py:131`): `(sample_time, clock, freq)`.
    pub fn clock_est(&self) -> (f64, f64, f64) {
        self.shared.state.borrow().clock_est
    }

    /// `mcu_freq` (`klippy/clocksync.py:19`).
    pub fn mcu_freq(&self) -> f64 {
        self.shared.state.borrow().mcu_freq
    }
}

/// Port of the `SecondarySync` class (`klippy/clocksync.py:163`),
/// syncing a secondary MCU to the primary MCU.
pub struct SecondarySync {
    reactor: Reactor,
    clock: ClockSync,
    main_sync: ClockSync,
    /// `clock_adj`: `(adjusted_offset, adjusted_freq)`.
    clock_adj: SCell<(f64, f64)>,
    last_sync_time: SCell<f64>,
}

impl SecondarySync {
    /// `SecondarySync(reactor, main_sync)` (`klippy/clocksync.py:164`).
    pub fn new(reactor: Reactor, main_sync: ClockSync) -> SecondarySync {
        let clock = ClockSync::new(reactor.clone());
        SecondarySync {
            reactor,
            clock,
            main_sync,
            clock_adj: SCell::new((0., 1.)),
            last_sync_time: SCell::new(0.),
        }
    }

    /// `connect(serial)` (`klippy/clocksync.py:169`).
    pub fn connect(&self, serial: SerialHandle) -> KResult<()> {
        self.clock.connect(Arc::clone(&serial))?;
        let mcu_freq = self.clock.mcu_freq();
        *self.clock_adj.borrow_mut() = (0., mcu_freq);
        let curtime = self.reactor.monotonic();
        let main_print_time = self.main_sync.estimated_print_time(curtime);
        let local_print_time = self.estimated_print_time(curtime);
        *self.clock_adj.borrow_mut() = (main_print_time - local_print_time, mcu_freq);
        self.calibrate_clock(0., curtime)?;
        Ok(())
    }

    /// `connect_file(serial, pace=False)` (`klippy/clocksync.py:177`).
    pub fn connect_file(&self, serial: SerialHandle, pace: bool) -> KResult<()> {
        self.clock.connect_file(Arc::clone(&serial), pace)?;
        *self.clock_adj.borrow_mut() = (0., self.clock.mcu_freq());
        Ok(())
    }

    /// `print_time_to_clock(print_time)` (`klippy/clocksync.py:181`).
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 {
        let (offset, freq) = *self.clock_adj.borrow();
        py_trunc_i64((print_time - offset) * freq)
    }

    /// `clock_to_print_time(clock)` (`klippy/clocksync.py:184`).
    pub fn clock_to_print_time(&self, clock: i64) -> f64 {
        let (offset, freq) = *self.clock_adj.borrow();
        clock as f64 / freq + offset
    }

    /// `get_clock(eventtime)` (inherited, `klippy/clocksync.py:128`).
    pub fn get_clock(&self, eventtime: f64) -> i64 {
        self.clock.get_clock(eventtime)
    }

    /// `estimate_clock_systime(reqclock)` (inherited, `klippy/clocksync.py:131`).
    pub fn estimate_clock_systime(&self, reqclock: i64) -> f64 {
        self.clock.estimate_clock_systime(reqclock)
    }

    /// `estimated_print_time(eventtime)` (inherited, `klippy/clocksync.py:134`).
    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.clock_to_print_time(self.get_clock(eventtime))
    }

    /// `clock32_to_clock64(clock32)` (inherited, `klippy/clocksync.py:137`).
    pub fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        self.clock.clock32_to_clock64(clock32)
    }

    /// `is_active()` (inherited, `klippy/clocksync.py:142`).
    pub fn is_active(&self) -> bool {
        self.clock.is_active()
    }

    /// `dump_debug()` (`klippy/clocksync.py:188`).
    pub fn dump_debug(&self) -> String {
        let (offset, freq) = *self.clock_adj.borrow();
        pct_format(
            "%s clock_adj=(%.3f %.3f)",
            &[
                PctValue::Str(self.clock.dump_debug()),
                PctValue::Float(offset),
                PctValue::Float(freq),
            ],
        )
    }

    /// `stats(eventtime)` (`klippy/clocksync.py:192`).
    pub fn stats(&self, eventtime: f64) -> String {
        pct_format(
            "%s adj=%d",
            &[
                PctValue::Str(self.clock.stats(eventtime)),
                PctValue::Float(self.clock_adj.borrow().1),
            ],
        )
    }

    /// `calibrate_clock(print_time, eventtime)` (`klippy/clocksync.py:195`).
    pub fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        // Calculate: est_print_time = main_sync.estimatated_print_time()
        let (ser_time, ser_clock, ser_freq) = self.main_sync.clock_est();
        let main_mcu_freq = self.main_sync.mcu_freq();
        let est_main_clock = (eventtime - ser_time) * ser_freq + ser_clock;
        let est_print_time = float_true_div(est_main_clock, main_mcu_freq).map_err(map_py_error)?;
        // Determine sync1_print_time and sync2_print_time.
        let sync1_print_time = py_max2(print_time, est_print_time);
        let sync2_print_time = py_max2(
            py_max2(sync1_print_time + 4., *self.last_sync_time.borrow()),
            print_time + 2.5 * (print_time - est_print_time),
        );
        // Calc sync2_sys_time (inverse of main_sync.estimatated_print_time).
        let sync2_main_clock = sync2_print_time * main_mcu_freq;
        let sync2_sys_time =
            ser_time + float_true_div(sync2_main_clock - ser_clock, ser_freq).map_err(map_py_error)?;
        // Adjust freq so estimated print_time will match at sync2_print_time.
        let sync1_clock = self.print_time_to_clock(sync1_print_time);
        let sync2_clock = self.get_clock(sync2_sys_time);
        let adjusted_freq = float_true_div(
            sync2_clock.wrapping_sub(sync1_clock) as f64,
            sync2_print_time - sync1_print_time,
        )
        .map_err(map_py_error)?;
        let adjusted_offset = sync1_print_time
            - float_true_div(sync1_clock as f64, adjusted_freq).map_err(map_py_error)?;
        // Apply new values.
        *self.clock_adj.borrow_mut() = (adjusted_offset, adjusted_freq);
        *self.last_sync_time.borrow_mut() = sync2_print_time;
        Ok((adjusted_offset, adjusted_freq))
    }

    /// `clock_adj` (`klippy/clocksync.py:167`).
    pub fn clock_adj(&self) -> (f64, f64) {
        *self.clock_adj.borrow()
    }

    /// `last_sync_time` (`klippy/clocksync.py:168`).
    pub fn last_sync_time(&self) -> f64 {
        *self.last_sync_time.borrow()
    }
}

/// The frozen `klippy/clocksync.py` module has no `load_config`,
/// `load_config_prefix` or `add_printer_objects`: `klippy/mcu.py`
/// constructs `ClockSync` / `SecondarySync` directly.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
