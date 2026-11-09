//! Differential parity tests for the `trdispatch` unit.
//!
//! The test drives the frozen C library (`chelper_ref`, the unchanged
//! `klippy/chelper` sources) and the Rust port through identical inputs
//! and compares outputs bit for bit.
//!
//! Both sides run in lockstep over real socketpair serial queues (UART
//! type): each MCU of the homing group has its own queue pair, one for
//! the reference and one for the port, plus a scripted peer fd. The
//! peers inject identical `trsync_state` response frames; the test then
//! compares the bytes the two sides emit (`trsync_trigger` fan-out and
//! `trsync_set_timeout` extensions), the pull results and every
//! `errorf` log line.
//!
//! Cases run in a fixed order (see `CASES`):
//!
//! 1. `trdispatch_alloc`: allocation, start with no MCUs, double
//!    start, stop when idle. Deterministic without threads; plain
//!    `#[test]`.
//! 2. `trdispatch_trigger`: two MCUs, steady `can_trigger` reports,
//!    then a trigger on one MCU fanning out to both.
//! 3. `trdispatch_timeout`: three MCUs with staggered clocks; timeout
//!    extension as estimates advance.
//! 4. `trdispatch_latched`: reports ignored once `can_trigger` is false.
//! 5. `trdispatch_unknown`: reports for unknown oids (and bad CRCs)
//!    are ignored.
//! 6. `trdispatch_stop`: stop while timeout messages are in flight,
//!    then restart with a new reason.
//!
//! Cases 2-6 need the virtual clock (`parity/vclock`): the background
//! thread's wakeups decide how messages pack into blocks, so the byte
//! stream is only reproducible when time is virtual. With `VCLOCK_SOCK`
//! set, an `.init_array` constructor runs the cases before any `main`
//! and exits with libtest's status and result lines. Without it, `main`
//! (`harness = false`) or the `#[test]` functions (libtest harness) run
//! them.
//!
//! `VCLOCK_SOCK` is required; without it the clock cases fail. A
//! watchdog thread aborts after 600 virtual seconds so an unexpected
//! blocking pull cannot hang the scenario.

use std::ffi::c_void;
use std::sync::{Arc, LazyLock};

use chelper_serial::msgblock::{message_alloc_and_encode, MESSAGE_MAX, MESSAGE_SYNC};
use chelper_serial::serialqueue::CommandQueue;
use chelper_serial::trdispatch::{Trdispatch, TrdispatchMcu};
use chelper_serial::{alloc_commandqueue, free_commandqueue, SerialQueue};
use kos::{fd, sync, thread, time};

unsafe extern "C" {
    fn trdispatch_alloc() -> *mut c_void;
    fn trdispatch_start(td: *mut c_void, dispatch_reason: u32);
    fn trdispatch_stop(td: *mut c_void);
    fn trdispatch_mcu_alloc(
        td: *mut c_void,
        sq: *mut c_void,
        cq: *mut c_void,
        trsync_oid: u32,
        set_timeout_msgtag: u32,
        trigger_msgtag: u32,
        state_msgtag: u32,
    ) -> *mut c_void;
    fn trdispatch_mcu_setup(
        tdm: *mut c_void,
        last_status_clock: u64,
        expire_clock: u64,
        expire_ticks: u64,
        min_extend_ticks: u64,
    );
    fn serialqueue_alloc(
        serial_fd: i32,
        serial_fd_type: u8,
        client_id: i32,
        name: *mut u8,
    ) -> *mut c_void;
    fn serialqueue_exit(sq: *mut c_void);
    fn serialqueue_free(sq: *mut c_void);
    fn serialqueue_alloc_commandqueue() -> *mut c_void;
    fn serialqueue_free_commandqueue(cq: *mut c_void);
    fn serialqueue_set_clock_est(sq: *mut c_void, est_freq: f64, conv_time: f64, conv_clock: u64);
    fn set_python_logging_callback(cb: extern "C" fn(*const u8));
    fn msgblock_crc16_ccitt(buf: *mut u8, len: u8) -> u16;
    fn socketpair(domain: i32, ty: i32, protocol: i32, sv: *mut i32) -> i32;
    fn free(p: *mut c_void);
}

// ---- log capture (process-global) --------------------------------------------

struct Capture {
    c_logs: sync::Mutex<Vec<String>>,
    port_logs: sync::Mutex<Vec<String>>,
}

static CAPTURE: LazyLock<Capture> = LazyLock::new(|| Capture {
    c_logs: sync::Mutex::new(Vec::new()),
    port_logs: sync::Mutex::new(Vec::new()),
});

fn install_capture() {
    LazyLock::force(&CAPTURE);
    // SAFETY: plain function pointer stored in the C global.
    unsafe { set_python_logging_callback(c_log_callback) };
    chelper_serial::set_logging_callback(|msg: &str| {
        capture_port_logs().push(msg.to_string());
    });
}

fn capture_port_logs() -> sync::MutexGuard<'static, Vec<String>> {
    CAPTURE.port_logs.lock()
}

fn c_logs() -> sync::MutexGuard<'static, Vec<String>> {
    CAPTURE.c_logs.lock()
}

fn port_logs() -> sync::MutexGuard<'static, Vec<String>> {
    CAPTURE.port_logs.lock()
}

extern "C" fn c_log_callback(msg: *const u8) {
    // SAFETY: the C `errorf` passes a NUL-terminated buffer.
    let s = unsafe { std::ffi::CStr::from_ptr(msg as *const std::os::raw::c_char) };
    c_logs().push(s.to_string_lossy().into_owned());
}

fn under_vclock() -> bool {
    std::env::var_os("VCLOCK_SOCK").is_some()
}

fn start_watchdog() {
    let _detached = thread::spawn(|| {
        time::sleep(600.0);
        eprintln!("trdispatch test watchdog: scenario still running after 600 virtual seconds");
        std::process::abort();
    });
}

fn assert_bytes_eq(a: &[u8], b: &[u8], what: &str) {
    assert_eq!(
        a.len(),
        b.len(),
        "{what}: length mismatch (ref {} vs port {})",
        a.len(),
        b.len()
    );
    assert_eq!(a, b, "{what}: byte mismatch");
}

fn assert_vec_eq<T: PartialEq + std::fmt::Debug>(a: &[T], b: &[T], what: &str) {
    assert_eq!(a, b, "{what}: mismatch");
}

// ---- peer scripting ------------------------------------------------------------

/// Build a peer frame with the reference CRC (peer scripting does
/// not depend on the implementation under test for CRCs).
fn peer_frame(seq_nibble: u8, payload: &[u8], corrupt_crc: bool) -> Vec<u8> {
    let mut frame = vec![0u8; 2 + payload.len() + 3];
    frame[0] = frame.len() as u8;
    frame[1] = 0x10 | (seq_nibble & 0x0f);
    frame[2..2 + payload.len()].copy_from_slice(payload);
    let crc = unsafe { msgblock_crc16_ccitt(frame.as_mut_ptr(), (frame.len() - 3) as u8) };
    let crc = if corrupt_crc {
        crc.wrapping_add(1)
    } else {
        crc
    };
    let len = frame.len();
    frame[len - 3] = (crc >> 8) as u8;
    frame[len - 2] = (crc & 0xff) as u8;
    frame[len - 1] = MESSAGE_SYNC;
    frame
}

/// Encode one `trsync_state` payload the way the MCU dictionary
/// would: `trsync_state oid=%c can_trigger=%c trigger_reason=%c
/// clock=%u`, with the given leading msgtag.
fn trsync_state_payload(
    state_msgtag: u32,
    oid: u32,
    can_trigger: u32,
    trigger_reason: u32,
    clock: u32,
) -> Vec<u8> {
    let qm = message_alloc_and_encode(&[state_msgtag, oid, can_trigger, trigger_reason, clock]);
    qm.msg[..qm.len as usize].to_vec()
}

/// The scripted MCU end of one socketpair.
struct Peer {
    fd: i32,
    rx: Vec<u8>,
    transcript: Vec<u8>,
}

impl Peer {
    fn send(&self, bytes: &[u8]) {
        let mut off = 0;
        while off < bytes.len() {
            match fd::write(self.fd, &bytes[off..]) {
                Ok(n) if n > 0 => off += n,
                other => panic!("peer write failed: {other:?}"),
            }
        }
    }

    /// Read whatever is available and parse complete blocks into the
    /// transcript. Returns the sequence nibble of each new block.
    fn drain(&mut self) -> Vec<u8> {
        let mut chunk = [0u8; 4096];
        while let Ok(n) = fd::read(self.fd, &mut chunk) {
            if n == 0 {
                break;
            }
            self.rx.extend_from_slice(&chunk[..n]);
        }
        let mut parsed = Vec::new();
        loop {
            while self.rx.first() == Some(&MESSAGE_SYNC) {
                self.rx.remove(0);
            }
            if self.rx.is_empty() {
                break;
            }
            let len = self.rx[0] as usize;
            assert!(
                (chelper_serial::msgblock::MESSAGE_MIN..=MESSAGE_MAX).contains(&len),
                "peer saw a bad block length {len:#x}"
            );
            if self.rx.len() < len {
                break;
            }
            let block: Vec<u8> = self.rx.drain(..len).collect();
            self.transcript.extend_from_slice(&block);
            parsed.push(block[1] & 0x0f);
        }
        parsed
    }
}

// ---- lockstep wrappers -----------------------------------------------------------

/// Queues for one MCU on one side. The port side shares `Arc`
/// ownership between the test handle and the dispatch entry, so the
/// entry keeps the queues alive after the test drops its handle.
struct Queues {
    is_ref: bool,
    c_sq: *mut c_void,
    c_cq: *mut c_void,
    port: Option<Arc<PortQueues>>,
}

struct PortQueues {
    sq: Arc<SerialQueue>,
    cq: Arc<CommandQueue>,
}

impl Queues {
    fn alloc(is_ref: bool, serial_fd: i32, index: usize) -> Queues {
        let name = format!("td{index}");
        if is_ref {
            let mut name16 = [0u8; 16];
            let n = name.len().min(16);
            name16[..n].copy_from_slice(&name.as_bytes()[..n]);
            name16[15] = 0;
            let sq =
                unsafe { serialqueue_alloc(serial_fd, b'u', 0x100 + index as i32, name16.as_mut_ptr()) };
            assert!(!sq.is_null(), "ref serialqueue_alloc returned null");
            let cq = unsafe { serialqueue_alloc_commandqueue() };
            assert!(!cq.is_null(), "ref alloc_commandqueue returned null");
            Queues {
                is_ref: true,
                c_sq: sq,
                c_cq: cq,
                port: None,
            }
        } else {
            let sq = SerialQueue::alloc(serial_fd, b'u', 0x100 + index as i32, name.as_bytes())
                .expect("port serialqueue_alloc returned None");
            let cq = alloc_commandqueue();
            Queues {
                is_ref: false,
                c_sq: std::ptr::null_mut(),
                c_cq: std::ptr::null_mut(),
                port: Some(Arc::new(PortQueues { sq: Arc::new(sq), cq })),
            }
        }
    }

    fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: u64) {
        if self.is_ref {
            unsafe { serialqueue_set_clock_est(self.c_sq, freq, conv_time, conv_clock) };
        } else {
            let p = self.port.as_ref().expect("port queues freed");
            p.sq.set_clock_est(freq, conv_time, conv_clock);
        }
    }

    fn free_queues(&mut self, sq_fd: i32, peer_fd: i32) {
        if self.is_ref {
            unsafe {
                serialqueue_exit(self.c_sq);
                serialqueue_free(self.c_sq);
                serialqueue_free_commandqueue(self.c_cq);
            }
            self.c_sq = std::ptr::null_mut();
            self.c_cq = std::ptr::null_mut();
        } else if let Some(p) = self.port.take() {
            // Same order as the C side: freeing the serial queue drains the
            // command queues it still lists, then the command queue is freed.
            let PortQueues { sq, cq } = Arc::try_unwrap(p)
                .unwrap_or_else(|_| panic!("port queues still shared at teardown"));
            Arc::try_unwrap(sq)
                .unwrap_or_else(|_| panic!("port serial queue still shared at teardown"))
                .free();
            free_commandqueue(cq);
        }
        fd::close(sq_fd);
        fd::close(peer_fd);
    }
}

struct Td {
    is_ref: bool,
    c_td: *mut c_void,
    port: Option<Arc<Trdispatch>>,
}

struct Tdm {
    is_ref: bool,
    c_tdm: *mut c_void,
    port: Option<TrdispatchMcu>,
}

impl Td {
    fn alloc(is_ref: bool) -> Td {
        if is_ref {
            let td = unsafe { trdispatch_alloc() };
            assert!(!td.is_null(), "ref trdispatch_alloc returned null");
            Td {
                is_ref: true,
                c_td: td,
                port: None,
            }
        } else {
            Td {
                is_ref: false,
                c_td: std::ptr::null_mut(),
                port: Some(Arc::new(Trdispatch::new())),
            }
        }
    }

    fn mcu_alloc(
        &self,
        q: &Queues,
        oid: u32,
        set_timeout_tag: u32,
        trigger_tag: u32,
        state_tag: u32,
    ) -> Tdm {
        if self.is_ref {
            assert!(q.is_ref, "ref mcu_alloc with port queues");
            let tdm = unsafe {
                trdispatch_mcu_alloc(
                    self.c_td,
                    q.c_sq,
                    q.c_cq,
                    oid,
                    set_timeout_tag,
                    trigger_tag,
                    state_tag,
                )
            };
            assert!(!tdm.is_null(), "ref trdispatch_mcu_alloc returned null");
            Tdm {
                is_ref: true,
                c_tdm: tdm,
                port: None,
            }
        } else {
            let t = self.port.as_ref().expect("port dispatch freed");
            let p = q.port.as_ref().expect("port queues freed");
            let mcu = t.mcu_alloc(
                Arc::clone(&p.sq),
                Arc::clone(&p.cq),
                oid,
                set_timeout_tag,
                trigger_tag,
                state_tag,
            );
            Tdm {
                is_ref: false,
                c_tdm: std::ptr::null_mut(),
                port: Some(mcu),
            }
        }
    }

    fn start(&self, reason: u32) {
        if self.is_ref {
            unsafe { trdispatch_start(self.c_td, reason) };
        } else {
            self.port.as_ref().expect("port dispatch freed").start(reason);
        }
    }

    fn stop(&self) {
        if self.is_ref {
            unsafe { trdispatch_stop(self.c_td) };
        } else {
            self.port.as_ref().expect("port dispatch freed").stop();
        }
    }

    fn free_td(mut self) {
        self.free_td_in_place();
    }

    fn free_td_in_place(&mut self) {
        if self.is_ref {
            unsafe { free(self.c_td) };
            self.c_td = std::ptr::null_mut();
        } else {
            self.port.take();
        }
    }
}

impl Tdm {
    fn setup(&self, last: u64, expire: u64, expire_ticks: u64, min_extend: u64) {
        if self.is_ref {
            unsafe { trdispatch_mcu_setup(self.c_tdm, last, expire, expire_ticks, min_extend) };
        } else {
            self.port
                .as_ref()
                .expect("port mcu freed")
                .setup(last, expire, expire_ticks, min_extend);
        }
    }

    fn free_tdm(mut self) {
        if self.is_ref {
            unsafe { free(self.c_tdm) };
            self.c_tdm = std::ptr::null_mut();
        } else if let Some(m) = self.port.take() {
            m.free();
        }
    }
}

// ---- scenario scaffolding ----------------------------------------------------------

const STATE_TAG: u32 = 0x9c;
const SET_TIMEOUT_TAG: u32 = 0x9d;
const TRIGGER_TAG: u32 = 0x9e;

/// One MCU: queues (reference or port) plus its scripted peer.
struct McuLane {
    q: Queues,
    peer: Peer,
    sq_fd: i32,
    next_peer_seq: u8,
}

impl McuLane {
    fn new(is_ref: bool, index: usize) -> McuLane {
        let (sq_fd, peer_fd) = new_socketpair();
        fd::set_nonblock(peer_fd).expect("peer nonblock");
        McuLane {
            q: Queues::alloc(is_ref, sq_fd, index),
            peer: Peer {
                fd: peer_fd,
                rx: Vec::new(),
                transcript: Vec::new(),
            },
            sq_fd,
            next_peer_seq: 8,
        }
    }
}

/// Final transcripts and log lines of both sides.
type FinishOut = (Vec<Vec<u8>>, Vec<Vec<u8>>, Vec<String>, Vec<String>);

/// Both implementations of one homing group, in lockstep.
struct World {
    td_ref: Td,
    td_port: Td,
    ref_lanes: Vec<McuLane>,
    port_lanes: Vec<McuLane>,
    mcus_ref: Vec<Tdm>,
    mcus_port: Vec<Tdm>,
    oids: Vec<u32>,
}

impl World {
    fn new(oids: &[u32]) -> World {
        assert!(!oids.is_empty(), "world needs at least one mcu");
        let td_ref = Td::alloc(true);
        let td_port = Td::alloc(false);
        let mut ref_lanes = Vec::new();
        let mut port_lanes = Vec::new();
        for i in 0..oids.len() {
            ref_lanes.push(McuLane::new(true, i));
            port_lanes.push(McuLane::new(false, i));
        }
        let mut mcus_ref = Vec::new();
        let mut mcus_port = Vec::new();
        for i in 0..oids.len() {
            mcus_ref.push(td_ref.mcu_alloc(
                &ref_lanes[i].q,
                oids[i],
                SET_TIMEOUT_TAG,
                TRIGGER_TAG,
                STATE_TAG,
            ));
            mcus_port.push(td_port.mcu_alloc(
                &port_lanes[i].q,
                oids[i],
                SET_TIMEOUT_TAG,
                TRIGGER_TAG,
                STATE_TAG,
            ));
        }
        World {
            td_ref,
            td_port,
            ref_lanes,
            port_lanes,
            mcus_ref,
            mcus_port,
            oids: oids.to_vec(),
        }
    }

    fn setup_all(&self, last: u64, expire: u64, expire_ticks: u64, min_extend: u64) {
        for m in &self.mcus_ref {
            m.setup(last, expire, expire_ticks, min_extend);
        }
        for m in &self.mcus_port {
            m.setup(last, expire, expire_ticks, min_extend);
        }
    }

    fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: u64) {
        for l in self.ref_lanes.iter().chain(self.port_lanes.iter()) {
            l.q.set_clock_est(freq, conv_time, conv_clock);
        }
    }

    fn start(&self, reason: u32) {
        self.td_ref.start(reason);
        self.td_port.start(reason);
    }

    fn stop(&self) {
        self.td_ref.stop();
        self.td_port.stop();
    }

    /// Let the queues settle, then clear transcripts so later
    /// comparisons only see trdispatch traffic.
    fn handshake(&mut self) {
        for _ in 0..50 {
            time::sleep(0.001);
            for l in self.ref_lanes.iter_mut().chain(self.port_lanes.iter_mut()) {
                l.peer.drain();
            }
        }
        for l in self.ref_lanes.iter_mut().chain(self.port_lanes.iter_mut()) {
            l.peer.transcript.clear();
            l.peer.rx.clear();
        }
    }

    /// Send the same response bytes to lane `i` on both sides.
    fn inject(&mut self, i: usize, can_trigger: u32, trigger_reason: u32, clock: u32) {
        let payload =
            trsync_state_payload(STATE_TAG, self.oids[i], can_trigger, trigger_reason, clock);
        let seq_r = self.ref_lanes[i].next_peer_seq;
        self.ref_lanes[i].peer.send(&peer_frame(seq_r, &payload, false));
        self.ref_lanes[i].next_peer_seq = bump_seq(seq_r);
        let seq_p = self.port_lanes[i].next_peer_seq;
        self.port_lanes[i].peer.send(&peer_frame(seq_p, &payload, false));
        self.port_lanes[i].next_peer_seq = bump_seq(seq_p);
    }

    fn sleep_drain(&mut self, secs: f64) {
        time::sleep(secs);
        for l in self.ref_lanes.iter_mut().chain(self.port_lanes.iter_mut()) {
            l.peer.drain();
        }
    }

    /// Compare every emitted byte on every lane, then clear.
    fn expect_transcripts_eq(&mut self, what: &str) {
        for (i, (r, p)) in self
            .ref_lanes
            .iter_mut()
            .zip(self.port_lanes.iter_mut())
            .enumerate()
        {
            r.peer.drain();
            p.peer.drain();
            assert_bytes_eq(
                &r.peer.transcript,
                &p.peer.transcript,
                &format!("{what} lane {i}"),
            );
            r.peer.transcript.clear();
            p.peer.transcript.clear();
        }
    }

    fn finish(mut self) -> FinishOut {
        // Unregister every fast reader before freeing anything (the C
        // `trdispatch_stop`; idempotent when already stopped).
        self.stop();
        for m in self.mcus_ref.drain(..) {
            m.free_tdm();
        }
        for m in self.mcus_port.drain(..) {
            m.free_tdm();
        }
        let mut ref_tx = Vec::new();
        let mut port_tx = Vec::new();
        for l in self.ref_lanes.iter_mut() {
            l.peer.drain();
            ref_tx.push(std::mem::take(&mut l.peer.transcript));
        }
        for l in self.port_lanes.iter_mut() {
            l.peer.drain();
            port_tx.push(std::mem::take(&mut l.peer.transcript));
        }
        for l in self.ref_lanes.iter_mut().chain(self.port_lanes.iter_mut()) {
            let (sq_fd, peer_fd) = (l.sq_fd, l.peer.fd);
            l.q.free_queues(sq_fd, peer_fd);
            l.sq_fd = -1;
            l.peer.fd = -1;
        }
        // Free the dispatch groups in place: take each field out and
        // leave a null/empty value behind (`finish` owns `self`, so no
        // other code observes the half-freed state).
        self.td_ref.free_td_in_place();
        self.td_port.free_td_in_place();
        (ref_tx, port_tx, c_logs().clone(), port_logs().clone())
    }
}

fn bump_seq(s: u8) -> u8 {
    let n = s.wrapping_add(1) & 0x0f;
    if n == 0 {
        1
    } else {
        n
    }
}

fn new_socketpair() -> (i32, i32) {
    const AF_UNIX: i32 = 1;
    const SOCK_STREAM: i32 = 1;
    let mut sv = [0i32; 2];
    // SAFETY: `sv` is valid storage for two fds.
    let rc = unsafe { socketpair(AF_UNIX, SOCK_STREAM, 0, sv.as_mut_ptr()) };
    assert_eq!(rc, 0, "socketpair failed");
    (sv[0], sv[1])
}

// ---- case 1: allocation and start/stop without threads --------------------------------

fn alloc_setup() {
    c_logs().clear();
    port_logs().clear();
    // Start with no MCUs is a no-op; double start keeps the first
    // reason; stop when idle is a no-op.
    for is_ref in [true, false] {
        let td = Td::alloc(is_ref);
        td.start(7);
        td.stop();
        td.start(7);
        td.start(9);
        td.stop();
        td.stop();
        td.free_td();
    }
    assert_vec_eq(&c_logs(), &port_logs(), "alloc log lines");
}

// ---- cases 2-6: lockstep scenarios ------------------------------------------------------

const FREQ: f64 = 100_000_000.0;

fn base_time() -> f64 {
    time::monotonic()
}

/// Two MCUs, steady reports, then a trigger on one fanning out.
fn trigger_fanout() {
    c_logs().clear();
    port_logs().clear();
    let mut w = World::new(&[11, 12]);
    let base = base_time();
    w.set_clock_est(FREQ, base, 0);
    w.setup_all(0, 8_000_000, 4_000_000, 1_000_000);
    w.handshake();
    w.start(2);
    // Steady reports from both MCUs.
    w.inject(0, 1, 0, 1_000_000);
    w.inject(1, 1, 0, 1_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("steady reports");
    // Trigger on MCU 0 fans out to both.
    w.inject(0, 0, 1, 2_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("trigger fanout");
    // Further reports are ignored (`can_trigger` is false).
    w.inject(1, 1, 0, 3_000_000);
    w.sleep_drain(0.005);
    w.expect_transcripts_eq("post-trigger quiet");
    let (ref_tx, port_tx, ref_logs, port_logs_) = w.finish();
    assert_eq!(ref_tx, port_tx, "final transcripts");
    assert_vec_eq(&ref_logs, &port_logs_, "trigger log lines");
}

/// Three MCUs with staggered clocks; timeout extension as estimates advance.
fn timeout_extend() {
    c_logs().clear();
    port_logs().clear();
    let mut w = World::new(&[21, 22, 23]);
    let base = base_time();
    w.set_clock_est(FREQ, base, 0);
    // Small extend threshold so every report extends at least one MCU.
    w.setup_all(0, 0, 10_000_000, 1);
    w.handshake();
    w.start(4);
    // Staggered reports: lane 0 lags, lanes 1-2 advance.
    w.inject(0, 1, 0, 1_000_000);
    w.sleep_drain(0.005);
    w.expect_transcripts_eq("lagging lane");
    w.inject(1, 1, 0, 5_000_000);
    w.inject(2, 1, 0, 5_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("advanced lanes");
    // Advance the clock estimate (as mcu.py would on reconnect) and
    // report again: extensions must track the new estimate.
    w.set_clock_est(FREQ, base + 1.0, 100_000_000);
    w.setup_all(100_000_000, 108_000_000, 10_000_000, 1);
    w.inject(0, 1, 0, 101_000_000);
    w.inject(1, 1, 0, 105_000_000);
    w.inject(2, 1, 0, 109_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("advanced estimate");
    let (ref_tx, port_tx, ref_logs, port_logs_) = w.finish();
    assert_eq!(ref_tx, port_tx, "final transcripts");
    assert_vec_eq(&ref_logs, &port_logs_, "timeout log lines");
}

/// Reports ignored once `can_trigger` is false (trigger latched).
fn can_trigger_false() {
    c_logs().clear();
    port_logs().clear();
    let mut w = World::new(&[31, 32]);
    let base = base_time();
    w.set_clock_est(FREQ, base, 0);
    w.setup_all(0, 8_000_000, 4_000_000, 1_000_000);
    w.handshake();
    w.start(1);
    w.inject(1, 0, 3, 2_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("first trigger");
    // A late steady report and a second trigger both change nothing.
    w.inject(0, 1, 0, 3_000_000);
    w.inject(1, 0, 1, 4_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("latched quiet");
    let (ref_tx, port_tx, ref_logs, port_logs_) = w.finish();
    assert_eq!(ref_tx, port_tx, "final transcripts");
    assert_vec_eq(&ref_logs, &port_logs_, "latched log lines");
}

/// Reports for unknown oids (and bad CRCs) are ignored.
fn unknown_oid() {
    c_logs().clear();
    port_logs().clear();
    let mut w = World::new(&[41, 42]);
    let base = base_time();
    w.set_clock_est(FREQ, base, 0);
    w.setup_all(0, 8_000_000, 4_000_000, 1_000_000);
    w.handshake();
    w.start(2);
    // Unknown oid on each lane.
    for i in 0..2 {
        let payload = trsync_state_payload(STATE_TAG, 999, 0, 1, 1_000_000);
        let seq_r = w.ref_lanes[i].next_peer_seq;
        w.ref_lanes[i].peer.send(&peer_frame(seq_r, &payload, false));
        w.ref_lanes[i].next_peer_seq = bump_seq(seq_r);
        let seq_p = w.port_lanes[i].next_peer_seq;
        w.port_lanes[i].peer.send(&peer_frame(seq_p, &payload, false));
        w.port_lanes[i].next_peer_seq = bump_seq(seq_p);
    }
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("unknown oid");
    // Bad CRCs (dropped by the serial layer) are ignored too.
    for i in 0..2 {
        let payload = trsync_state_payload(STATE_TAG, w.oids[i], 0, 1, 2_000_000);
        let seq_r = w.ref_lanes[i].next_peer_seq;
        w.ref_lanes[i].peer.send(&peer_frame(seq_r, &payload, true));
        w.ref_lanes[i].next_peer_seq = bump_seq(seq_r);
        let seq_p = w.port_lanes[i].next_peer_seq;
        w.port_lanes[i].peer.send(&peer_frame(seq_p, &payload, true));
        w.port_lanes[i].next_peer_seq = bump_seq(seq_p);
    }
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("bad crc");
    // A real trigger still works afterwards.
    w.inject(0, 0, 1, 3_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("trigger after unknowns");
    let (ref_tx, port_tx, ref_logs, port_logs_) = w.finish();
    assert_eq!(ref_tx, port_tx, "final transcripts");
    assert_vec_eq(&ref_logs, &port_logs_, "unknown-oid log lines");
}

/// Stop while timeout messages are in flight, then restart.
fn stop_inflight() {
    c_logs().clear();
    port_logs().clear();
    let mut w = World::new(&[51, 52]);
    let base = base_time();
    w.set_clock_est(FREQ, base, 0);
    w.setup_all(0, 0, 10_000_000, 1);
    w.handshake();
    w.start(2);
    w.inject(0, 1, 0, 1_000_000);
    w.inject(1, 1, 0, 2_000_000);
    // Stop immediately, while the timeout extensions may still be
    // queued, then drain and compare.
    w.stop();
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("stop inflight");
    // Reports while stopped change nothing (readers unregistered).
    w.inject(0, 0, 1, 3_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("stopped quiet");
    // Restart with a new reason and trigger.
    w.start(9);
    w.inject(1, 0, 1, 4_000_000);
    w.sleep_drain(0.010);
    w.expect_transcripts_eq("restart trigger");
    let (ref_tx, port_tx, ref_logs, port_logs_) = w.finish();
    assert_eq!(ref_tx, port_tx, "final transcripts");
    assert_vec_eq(&ref_logs, &port_logs_, "stop log lines");
}

// ---- case table and entry points -----------------------------------------------------------

/// Every parity case in execution order. The flag marks the cases whose
/// byte stream depends on background-thread wakeups and therefore needs
/// the virtual clock.
const CASES: [(&str, bool, fn()); 6] = [
    ("trdispatch_alloc", false, alloc_setup),
    ("trdispatch_trigger", true, trigger_fanout),
    ("trdispatch_timeout", true, timeout_extend),
    ("trdispatch_latched", true, can_trigger_false),
    ("trdispatch_unknown", true, unknown_oid),
    ("trdispatch_stop", true, stop_inflight),
];

/// Process-wide setup shared by every entry point: the log callbacks,
/// the capture and the watchdog are singletons.
fn setup() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        chelper_ref::link();
        install_capture();
        if under_vclock() {
            start_watchdog();
        }
    });
}

fn run_case(needs_clock: bool, case: fn()) {
    assert!(
        !needs_clock || under_vclock(),
        "trdispatch parity requires VCLOCK_SOCK and parity/vclock"
    );
    case();
}

/// Run every case on the calling thread, printing libtest-style result
/// lines. Returns whether all cases passed.
fn run_all_cases() -> bool {
    use std::io::Write;
    setup();
    println!();
    println!("running {} tests", CASES.len());
    let mut failed = Vec::new();
    for (name, needs_clock, case) in CASES {
        let ok = std::panic::catch_unwind(|| run_case(needs_clock, case)).is_ok();
        println!("test {name} ... {}", if ok { "ok" } else { "FAILED" });
        if !ok {
            failed.push(name);
        }
    }
    if !failed.is_empty() {
        println!();
        println!("failures:");
        for name in &failed {
            println!("    {name}");
        }
    }
    println!();
    println!(
        "test result: {}. {} passed; {} failed; 0 ignored; 0 measured; 0 filtered out",
        if failed.is_empty() { "ok" } else { "FAILED" },
        CASES.len() - failed.len(),
        failed.len()
    );
    let _ = std::io::stdout().flush();
    failed.is_empty()
}

/// Entry point with `harness = false`. Under the libtest harness rustc
/// replaces it with libtest's `main`. Under the virtual clock the
/// constructor below has already run every case and exited.
fn main() {
    if !run_all_cases() {
        std::process::exit(101);
    }
}

/// Under the virtual clock libtest cannot run any test: it spawns a thread
/// per test (also with `--test-threads=1`) and parks its main thread on a
/// raw futex, which `libvclock` does not interpose, so the main thread
/// keeps the baton and the test thread never starts. This constructor runs
/// before any `main`, on the registered main thread: it runs every case,
/// prints the libtest result lines and exits through glibc `exit`, which
/// deregisters from `vclockd`. glibc passes `argc`/`argv`/`envp` to
/// `.init_array` entries.
#[used]
#[unsafe(link_section = ".init_array")]
static RUN_BEFORE_MAIN: unsafe extern "C" fn(
    std::ffi::c_int,
    *const *const std::ffi::c_char,
    *const *const std::ffi::c_char,
) = run_before_main;

unsafe extern "C" fn run_before_main(
    argc: std::ffi::c_int,
    argv: *const *const std::ffi::c_char,
    _envp: *const *const std::ffi::c_char,
) {
    if !under_vclock() {
        return;
    }
    // `--list` only enumerates the libtest tests and starts no test thread.
    let listing = (0..argc.max(0) as usize).any(|i| {
        // SAFETY: glibc passes the process argv: `argc` valid C strings.
        unsafe { std::ffi::CStr::from_ptr(*argv.add(i)) }.to_bytes() == b"--list"
    });
    if listing {
        return;
    }
    let ok = run_all_cases();
    std::process::exit(if ok { 0 } else { 101 });
}

/// The log capture is process-global, so libtest cases never overlap even
/// when libtest runs several test threads.
static LIBTEST_SERIAL: LazyLock<sync::Mutex<()>> = LazyLock::new(|| sync::Mutex::new(()));

/// Body of the libtest `#[test]` functions, which run only without the
/// virtual clock. With `harness = false` rustc drops the `#[test]`
/// functions, leaving this unused.
#[allow(dead_code)]
fn run_libtest_case(name: &str) {
    let _guard = LIBTEST_SERIAL.lock();
    setup();
    let (_, needs_clock, case) = CASES
        .iter()
        .find(|(n, _, _)| *n == name)
        .expect("unknown trdispatch case");
    run_case(*needs_clock, *case);
}

#[test]
fn alloc_setup_case() {
    run_libtest_case("trdispatch_alloc");
}

// The cases below depend on background-thread wakeups and only produce the
// reference byte stream under the virtual clock, where the `.init_array`
// constructor runs them (`cargo_test.sh chelper_serial:trdispatch@vclock`).
// A plain `cargo test` lists them as ignored instead of failing.

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn trigger_case() {
    run_libtest_case("trdispatch_trigger");
}

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn timeout_case() {
    run_libtest_case("trdispatch_timeout");
}

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn latched_case() {
    run_libtest_case("trdispatch_latched");
}

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn unknown_case() {
    run_libtest_case("trdispatch_unknown");
}

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn stop_case() {
    run_libtest_case("trdispatch_stop");
}
