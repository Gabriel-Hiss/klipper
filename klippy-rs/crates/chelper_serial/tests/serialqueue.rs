//! Differential parity tests for the `serialqueue` unit.
//!
//! The test drives the frozen C library (`chelper_ref`, the unchanged
//! `klippy/chelper` sources) and the Rust port through identical inputs
//! and compares outputs bit for bit.
//!
//! Cases run in a fixed order (see `CASES`):
//!
//! 1. `msgblock` unit differential (CRC, check, decode, encode, clock
//!    helpers, message fill/encode) against the C functions. Pure
//!    functions; no virtual clock needed.
//! 2. Debug-file replay: every recording under
//!    `tests/fixtures/recordings/serialqueue/*.jsonl` is replayed against
//!    both sides in file mode (`SQT_DEBUGFILE`, output to a temp file).
//!    The written bytes and every `errorf` log line are compared.
//! 3. UART and CAN socketpair scenarios cover startup without a clock,
//!    acks, duplicate-ack and second-NAK fast retransmission, timeout
//!    retransmission, notify completion, data messages and fast readers,
//!    a bad-CRC frame, `min_clock`/`req_clock` gating, a background
//!    priority message, a `send_batch`, the receive and pending-block
//!    windows, sequence rollover, the stats
//!    string, the clock estimate and `extract_old` (including the
//!    `[DEBUG-motion]` dump). The peer transcripts, pull results,
//!    fast-reader events, stats, extract results and every log line are
//!    compared. CAN comparisons include frame IDs, lengths and payloads;
//!    the C implementation leaves padding and unused data uninitialized.
//!
//! Sections 2 and 3 need the virtual clock (`parity/vclock`): the
//! background thread's wakeups decide how messages pack into blocks, so
//! the byte stream is only reproducible when time is virtual. Both
//! sections run the reference and the port in lockstep: every action is
//! applied to both sides back to back, at the same virtual instant, and
//! virtual time only moves while every thread is blocked. The two
//! background threads therefore observe identical absolute timestamps and
//! the compared floats (sent/receive times, dump lines) are bit-identical
//! without any rebasing.
//!
//! `VCLOCK_SOCK` is required; without it the clock cases fail. A watchdog
//! thread aborts after 600 virtual seconds so an unexpected blocking pull
//! cannot hang the scenario.
//!
//! Run command (integration owner):
//!
//! ```sh
//! make -C parity/vclock x86_64
//! parity/vclock/build/x86_64/vclockd --sock /tmp/vc.sock --epoch 1700000000 --trace /tmp/vc.trace &
//! binary=$(cargo test --manifest-path klippy-rs/Cargo.toml -p chelper_serial \
//!   --test serialqueue --no-run --message-format=json | jq -r \
//!   'select(.reason == "compiler-artifact" and .target.name == "serialqueue" and .executable != null) | .executable')
//! LD_PRELOAD=$PWD/parity/vclock/build/x86_64/libvclock.so VCLOCK_SOCK=/tmp/vc.sock \
//!   "$binary" --test-threads=1
//! ```
//!
//! The target works with or without `harness = false`. Every case runs on
//! the process main thread, one after the other: libtest would run each
//! test on its own thread while its main thread waits on a raw futex that
//! keeps the virtual-clock baton, and the logging callbacks are
//! process-global. With `VCLOCK_SOCK` set, an `.init_array` constructor
//! runs the cases before any `main` and exits with libtest's status and
//! result lines. Without it, `main` (`harness = false`) or the four
//! `#[test]` functions (libtest harness) run them.

use std::ffi::c_void;
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, LazyLock};

use chelper_serial::msgblock::{
    crc16_ccitt, encode_int, ClockEstimate, MESSAGE_MAX, MESSAGE_MIN,
    MESSAGE_PAYLOAD_MAX, MESSAGE_SYNC,
};
use chelper_serial::serialqueue::CommandQueue;
use chelper_serial::{
    alloc_commandqueue, free_commandqueue, SerialQueue, BACKGROUND_PRIORITY_CLOCK,
};
use kos::{fd, sync, thread, time};
use recording::hexfloat::f64_from_hex;
use recording::{load_unit, Record, Value};

// ---- reference C declarations (klippy/chelper/*.h) ----------------------

#[repr(C)]
#[derive(Clone, Copy, Default)]
struct CClockEstimate {
    conv_clock: u64,
    conv_time: f64,
    est_freq: f64,
}

#[repr(C)]
struct CPullQueueMessage {
    msg: [u8; MESSAGE_MAX],
    len: i32,
    sent_time: f64,
    receive_time: f64,
    notify_id: u64,
}

/// `struct queue_message`; the `msg` array follows `len` directly (no
/// padding), the union fields start at offset 72.
#[repr(C)]
struct CQueueMessage {
    len: i32,
    msg: [u8; MESSAGE_MAX],
    min_clock: u64,
    req_clock: u64,
    notify_id: u64,
    node_next: *mut c_void,
    node_prev: *mut c_void,
}

/// `struct fastreader`.
#[repr(C)]
struct CFastReader {
    node: [*mut c_void; 2],
    func: extern "C" fn(*mut CFastReader, f64, *mut u8, i32),
    prefix_len: i32,
    prefix: [u8; MESSAGE_MAX],
}

unsafe extern "C" {
    fn serialqueue_alloc(serial_fd: i32, serial_fd_type: u8, client_id: i32, name: *mut u8) -> *mut c_void;
    fn serialqueue_exit(sq: *mut c_void);
    fn serialqueue_free(sq: *mut c_void);
    fn serialqueue_alloc_commandqueue() -> *mut c_void;
    fn serialqueue_free_commandqueue(cq: *mut c_void);
    fn serialqueue_add_fastreader(sq: *mut c_void, fr: *mut CFastReader);
    fn serialqueue_rm_fastreader(sq: *mut c_void, fr: *mut CFastReader);
    fn serialqueue_send_batch(sq: *mut c_void, cq: *mut c_void, msgs: *mut c_void);
    fn serialqueue_send(sq: *mut c_void, cq: *mut c_void, msg: *mut u8, len: i32,
                        min_clock: u64, req_clock: u64, notify_id: u64);
    fn serialqueue_pull(sq: *mut c_void, pqm: *mut CPullQueueMessage);
    fn serialqueue_set_wire_frequency(sq: *mut c_void, frequency: f64);
    fn serialqueue_set_receive_window(sq: *mut c_void, receive_window: i32);
    fn serialqueue_set_clock_est(sq: *mut c_void, est_freq: f64, conv_time: f64, conv_clock: u64);
    fn serialqueue_get_clock_est(sq: *mut c_void, ce: *mut CClockEstimate);
    fn serialqueue_get_stats(sq: *mut c_void, buf: *mut u8, len: i32);
    fn serialqueue_extract_old(sq: *mut c_void, sentq: i32, q: *mut CPullQueueMessage, max: i32) -> i32;
    fn set_python_logging_callback(cb: extern "C" fn(*const u8));
    fn msgblock_crc16_ccitt(buf: *mut u8, len: u8) -> u16;
    fn msgblock_check(need_sync: *mut u8, buf: *mut u8, buf_len: i32) -> i32;
    fn msgblock_decode(data: *mut u32, data_len: i32, msg: *mut u8, msg_len: i32) -> i32;
    fn message_fill(data: *mut u8, len: i32) -> *mut CQueueMessage;
    fn message_alloc_and_encode(data: *mut u32, len: i32) -> *mut CQueueMessage;
    fn message_free(qm: *mut CQueueMessage);
    fn clock_from_clock32(last_clock: u64, clock32: u32) -> u64;
    fn clock_to_time(ce: *mut CClockEstimate, clock: u64) -> f64;
    fn clock_from_time(ce: *mut CClockEstimate, time: f64) -> u64;
    fn clock_fill(ce: *mut CClockEstimate, est_freq: f64, conv_time: f64, conv_clock: u64);
    fn socketpair(domain: i32, ty: i32, protocol: i32, sv: *mut i32) -> i32;
}

// ---- log and fast-reader capture (process-global) -------------------------

#[derive(Clone, PartialEq, Debug)]
struct FastEvent {
    eventtime_bits: u64,
    len: i32,
    data: Vec<u8>,
}

struct Capture {
    c_logs: sync::Mutex<Vec<String>>,
    port_logs: sync::Mutex<Vec<String>>,
    c_fast: sync::Mutex<Vec<FastEvent>>,
    port_fast: sync::Mutex<Vec<FastEvent>>,
}

static CAPTURE: LazyLock<Capture> = LazyLock::new(|| Capture {
    c_logs: sync::Mutex::new(Vec::new()),
    port_logs: sync::Mutex::new(Vec::new()),
    c_fast: sync::Mutex::new(Vec::new()),
    port_fast: sync::Mutex::new(Vec::new()),
});

fn capture() -> &'static Capture {
    &CAPTURE
}

/// Install the C and port log sinks. Called once from the test body,
/// before any background thread exists (the lazy capture is forced
/// here so no other thread ever races on its initialization).
fn install_capture() {
    LazyLock::force(&CAPTURE);
    // SAFETY: plain function pointer stored in the C global.
    unsafe { set_python_logging_callback(c_log_callback) };
    chelper_serial::set_logging_callback(|msg: &str| {
        c_or_port_logs(false).push(msg.to_string());
    });
}

fn c_or_port_logs(is_ref: bool) -> sync::MutexGuard<'static, Vec<String>> {
    if is_ref {
        capture().c_logs.lock()
    } else {
        capture().port_logs.lock()
    }
}

fn c_logs() -> sync::MutexGuard<'static, Vec<String>> {
    c_or_port_logs(true)
}

fn port_logs() -> sync::MutexGuard<'static, Vec<String>> {
    c_or_port_logs(false)
}

extern "C" fn c_log_callback(msg: *const u8) {
    // SAFETY: the C `errorf` passes a NUL-terminated buffer.
    let s = unsafe { std::ffi::CStr::from_ptr(msg as *const std::os::raw::c_char) };
    c_logs().push(s.to_string_lossy().into_owned());
}

fn fast_event(eventtime: f64, data: *mut u8, len: i32) -> FastEvent {
    // SAFETY: both libraries pass a buffer holding at least `len` bytes.
    let bytes = unsafe { std::slice::from_raw_parts(data, len.max(0) as usize) };
    FastEvent { eventtime_bits: eventtime.to_bits(), len, data: bytes.to_vec() }
}

extern "C" fn c_fast_cb(_fr: *mut CFastReader, eventtime: f64, data: *mut u8, len: i32) {
    capture().c_fast.lock().push(fast_event(eventtime, data, len));
}

extern "C" fn port_fast_cb(_ctx: *mut c_void, eventtime: f64, data: *mut u8, len: i32) {
    capture().port_fast.lock().push(fast_event(eventtime, data, len));
}

fn under_vclock() -> bool {
    std::env::var_os("VCLOCK_SOCK").is_some()
}

/// Abort the process if a scenario is still running after 600 virtual
/// seconds (for example a `pull` that never gets its message).
fn start_watchdog() {
    let _detached = thread::spawn(|| {
        time::sleep(600.0);
        eprintln!("serialqueue test watchdog: scenario still running after 600 virtual seconds");
        std::process::abort();
    });
}

// ---- small comparison helpers -------------------------------------------

fn assert_bytes_eq(a: &[u8], b: &[u8], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: length mismatch (ref {} vs port {})", a.len(), b.len());
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        assert_eq!(x, y, "{what}: first differing byte at offset {i}: ref 0x{x:02x} vs port 0x{y:02x}");
    }
}

fn assert_vec_eq<T: PartialEq + std::fmt::Debug>(a: &[T], b: &[T], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: length mismatch: ref {} vs port {}", a.len(), b.len());
    for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
        assert_eq!(x, y, "{what}: first differing element at index {i}");
    }
}

fn assert_f64_bits(a: f64, b: f64, what: &str) {
    assert_eq!(a.to_bits(), b.to_bits(), "{what}: {a:?} (0x{:x}) vs {b:?} (0x{:x})",
               a.to_bits(), b.to_bits());
}

// Deterministic pseudo-random corpus (no time/HashMap anywhere).
fn corpus_buffers() -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    out.push(Vec::new());
    out.push(vec![0u8]);
    out.push(vec![0xff; 64]);
    // LCG over several lengths, including boundary lens 0..=64+. The C
    // CRC takes a `uint8_t` length, so no buffer exceeds 255 bytes.
    let mut state: u32 = 0x1234_5678;
    for &len in &[0usize, 1, 2, 3, 5, 8, 16, 59, 60, 64, 65, 100, 255] {
        let mut buf = Vec::with_capacity(len);
        for _ in 0..len {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            buf.push((state >> 24) as u8);
        }
        out.push(buf);
    }
    out
}

fn frame_with(payload: &[u8], seq: u8, corrupt_crc: bool) -> Vec<u8> {
    // Build a msgblock frame with the port encoder + port CRC, then
    // verify the C check against the same bytes in `check_frame_diff`.
    let mut frame = vec![0u8; 2 + payload.len() + 3];
    frame[0] = frame.len() as u8;
    frame[1] = seq;
    frame[2..2 + payload.len()].copy_from_slice(payload);
    let mut crc = crc16_ccitt(&frame[..frame.len() - 3]);
    if corrupt_crc {
        crc = crc.wrapping_add(1);
    }
    let len = frame.len();
    frame[len - 3] = (crc >> 8) as u8;
    frame[len - 2] = (crc & 0xff) as u8;
    frame[len - 1] = MESSAGE_SYNC;
    frame
}

fn check_frame_diff(buf: &[u8], need_sync_in: u8) {
    let ref_res = unsafe {
        let mut ns = need_sync_in;
        let mut b = buf.to_vec();
        let r = msgblock_check(&mut ns, b.as_mut_ptr(), b.len() as i32);
        (r, ns)
    };
    let port_res = {
        let mut ns = need_sync_in;
        let r = chelper_serial::msgblock::msgblock_check(&mut ns, buf);
        (r, ns)
    };
    assert_eq!(ref_res, port_res, "msgblock_check mismatch for buffer {:02x?} (need_sync={})",
               buf, need_sync_in);
}

// ---- section 1: msgblock units -------------------------------------------

fn msgblock_units() {
    // CRC differential.
    for buf in corpus_buffers() {
        let expect = unsafe { msgblock_crc16_ccitt(buf.as_ptr() as *mut u8, buf.len() as u8) };
        let got = crc16_ccitt(&buf);
        assert_eq!(expect, got, "crc16 mismatch for buffer of length {}", buf.len());
    }

    // check differential over generated buffers.
    for buf in corpus_buffers() {
        check_frame_diff(&buf, 0);
        check_frame_diff(&buf, 1);
        check_frame_diff(&buf, 0x7e);
        // valid frames at various seqs/payloads
        for seq in [0x10u8, 0x11, 0x1f, 0x00, 0x20, 0x90] {
            let frame = frame_with(&buf[..buf.len().min(16)], seq, false);
            check_frame_diff(&frame, 0);
            check_frame_diff(&frame, 1);
            // truncated prefixes
            for cut in 0..frame.len() {
                check_frame_diff(&frame[..cut], 0);
            }
            // corrupted variants
            let mut bad = frame.clone();
            let last = bad.len() - 1;
            bad[last] = 0x00;
            check_frame_diff(&bad, 0);
            let mut bad = frame.clone();
            bad[0] = 0x04;
            check_frame_diff(&bad, 0);
            let mut bad = frame.clone();
            bad[0] = 0x41;
            check_frame_diff(&bad, 0);
            let mut with_tail = frame.clone();
            with_tail.extend_from_slice(&[0x99, 0x88, 0x77]);
            check_frame_diff(&with_tail, 0);
            let mut with_garbage = vec![0xde, 0xad, 0xbe, 0xef];
            with_garbage.extend_from_slice(&frame);
            check_frame_diff(&with_garbage, 0);
            let mut with_sync_garbage = vec![0x7e, 0x01, 0x02];
            with_sync_garbage.extend_from_slice(&frame);
            check_frame_diff(&with_sync_garbage, 0);
        }
    }
    // Fully random fuzz buffers through check.
    let mut state: u32 = 0x9e37_79b9;
    for i in 0..400 {
        let len = (i % 130) as usize;
        let mut buf = Vec::with_capacity(len);
        for _ in 0..len {
            state = state.wrapping_mul(1664525).wrapping_add(1013904223);
            buf.push((state >> 24) as u8);
        }
        check_frame_diff(&buf, (i % 2) as u8);
    }

    // The C encoder is private; its public allocation helper exposes the
    // encoded bytes without transcribing the signed VLQ boundaries.
    for v in [
        0u32, 1, 31, 32, 95, 96, 127, u32::MAX,
        (-32i32) as u32, (-33i32) as u32,
        (-4096i32) as u32, (-4097i32) as u32, 12287, 12288,
        (-524288i32) as u32, (-524289i32) as u32, 1572863, 1572864,
        (-67108864i32) as u32, (-67108865i32) as u32, 201326591, 201326592,
    ] {
        let mut input = [v];
        let reference = unsafe { message_alloc_and_encode(input.as_mut_ptr(), 1) };
        assert!(!reference.is_null());
        let mut buf = [0u8; 8];
        let next = encode_int(&mut buf, 0, v);
        let expected = unsafe { &(&(*reference).msg)[..(*reference).len as usize] };
        assert_bytes_eq(expected, &buf[..next], &format!("encode_int({v})"));
        unsafe { message_free(reference) };
    }

    // decode differential: encode values with the port encoder,
    // wrap in a frame (port CRC), decode with both.
    let value_sets: Vec<Vec<u32>> = vec![
        vec![],
        vec![0],
        vec![0, 1, 127, 128],
        vec![96, 12288, 1572864, 201326592, 0xffff_ffff],
        vec![0xffff_ffe0, 0xffff_f000, 0xffff_8000, 0x1234_5678],
        vec![u32::MAX, u32::MAX / 2, 1 << 26, (1 << 26) - 1],
    ];
    for values in &value_sets {
        let mut payload = [0u8; MESSAGE_MAX];
        let mut p = 0_usize;
        for &v in values.iter() {
            p = encode_int(&mut payload, p, v);
        }
        if p > MESSAGE_PAYLOAD_MAX {
            continue;
        }
        let mut frame = vec![0u8; 2 + p + 3];
        frame[0] = frame.len() as u8;
        frame[1] = 0x10;
        frame[2..2 + p].copy_from_slice(&payload[..p]);
        let crc = crc16_ccitt(&frame[..frame.len() - 3]);
        let len = frame.len();
        frame[len - 3] = (crc >> 8) as u8;
        frame[len - 2] = (crc & 0xff) as u8;
        frame[len - 1] = MESSAGE_SYNC;
        // ref decode
        let mut ref_out = vec![0u32; values.len()];
        let ref_rc = unsafe {
            msgblock_decode(ref_out.as_mut_ptr(), values.len() as i32,
                            frame.as_mut_ptr(), frame.len() as i32)
        };
        let mut port_out = vec![0u32; values.len()];
        let port_rc = chelper_serial::msgblock::msgblock_decode(&mut port_out, &frame);
        assert_eq!(ref_rc, port_rc, "msgblock_decode rc for {values:?}");
        assert_vec_eq(&ref_out, &port_out, "msgblock_decode values");
        assert_vec_eq(&ref_out, values, "msgblock_decode roundtrip");
        // truncated decode (more slots than encoded values remain)
        let mut ref_out2 = vec![0u32; values.len() + 1];
        let ref_rc2 = unsafe {
            msgblock_decode(ref_out2.as_mut_ptr(), (values.len() + 1) as i32,
                            frame.as_mut_ptr(), frame.len() as i32)
        };
        let mut port_out2 = vec![0u32; values.len() + 1];
        let port_rc2 = chelper_serial::msgblock::msgblock_decode(&mut port_out2, &frame);
        assert_eq!(ref_rc2, port_rc2, "msgblock_decode overlong rc for {values:?}");
        // malformed: trailing garbage payload (p != end)
        let mut bad = frame.clone();
        bad.insert(bad.len() - 3, 0x42);
        bad[0] = bad.len() as u8;
        let crc = crc16_ccitt(&bad[..bad.len() - 3]);
        let blen = bad.len();
        bad[blen - 3] = (crc >> 8) as u8;
        bad[blen - 2] = (crc & 0xff) as u8;
        let mut ref_out3 = vec![0u32; values.len()];
        let ref_rc3 = unsafe {
            msgblock_decode(ref_out3.as_mut_ptr(), values.len() as i32,
                            bad.as_mut_ptr(), bad.len() as i32)
        };
        let mut port_out3 = vec![0u32; values.len()];
        let port_rc3 = chelper_serial::msgblock::msgblock_decode(&mut port_out3, &bad);
        assert_eq!(ref_rc3, port_rc3, "msgblock_decode malformed rc for {values:?}");
    }

    // clock differential (bit-exact, including int-width edges).
    let ce_cases: Vec<(f64, f64, u64)> = vec![
        (1e12, 4173.18, 0),
        (1e12, 0.0, 0),
        (0.5, 1.5, 100),
        (12345.678, -100.25, u64::MAX - 100),
        (1.0, 0.0, u64::MAX),
        (1e12, 4173.18, 1 << 62),
        (2.0, 1e15, 123456789),
        (0.0, 5.0, 7),
    ];
    for (freq, conv_time, magic) in &ce_cases {
        let (freq, conv_time, magic) = (*freq, *conv_time, *magic);
        let mut ref_ce = CClockEstimate { conv_clock: 0xdead, conv_time: 9.9, est_freq: 3.3 };
        let mut port_ce = ClockEstimate { conv_clock: 0xdead, conv_time: 9.9, est_freq: 3.3 };
        unsafe { clock_fill(&mut ref_ce, freq, conv_time, magic) };
        chelper_serial::msgblock::clock_fill(&mut port_ce, freq, conv_time, magic);
        assert_eq!(ref_ce.conv_clock, port_ce.conv_clock, "clock_fill conv_clock");
        assert_f64_bits(ref_ce.conv_time, port_ce.conv_time, "clock_fill conv_time");
        assert_f64_bits(ref_ce.est_freq, port_ce.est_freq, "clock_fill est_freq");
        for &clock in &[0u64, 1, 123456789, 1 << 32, u64::MAX, u64::MAX / 2, magic, magic.wrapping_add(777)] {
            let t_ref = unsafe { clock_to_time(&mut ref_ce, clock) };
            let t_port = chelper_serial::msgblock::clock_to_time(&port_ce, clock);
            assert_f64_bits(t_ref, t_port, &format!("clock_to_time clock={clock:#x}"));
        }
        for &t in &[0.0f64, 1.0, -1.0, conv_time, conv_time + 0.5, 4173.195] {
            let c_ref = unsafe { clock_from_time(&mut ref_ce, t) };
            let c_port = chelper_serial::msgblock::clock_from_time(&port_ce, t);
            assert_eq!(c_ref, c_port, "clock_from_time time={t:?}");
        }
    }
    for &(last, c32) in &[(0u64, 0u32), (0, 1), (0, 0x7fff_ffff), (0, 0x8000_0000),
                          (0, 0xffff_ffff), (123456789, 0u32), (u64::MAX, 0u32),
                          (u64::MAX, 0xffff_ffff), (1 << 63, 0x8000_0001),
                          (0xffff_ffff_ffff_ffff, 0xffff_ffff)] {
        let a = unsafe { clock_from_clock32(last, c32) };
        let b = chelper_serial::msgblock::clock_from_clock32(last, c32);
        assert_eq!(a, b, "clock_from_clock32 last={last:#x} c32={c32:#x}");
    }

    // message_fill / message_alloc_and_encode differential.
    for payload in [Vec::<u8>::new(), vec![1], vec![1, 2, 3, 4, 5, 6, 7, 8],
                    (0..64u8).collect::<Vec<_>>()] {
        let len = payload.len() as i32;
        let ref_qm = unsafe { message_fill(payload.as_ptr() as *mut u8, len) };
        let ref_len = unsafe { (*ref_qm).len };
        let ref_msg: [u8; MESSAGE_MAX] = unsafe { (*ref_qm).msg };
        let port_qm = chelper_serial::msgblock::message_fill(&payload, len);
        assert_eq!(ref_len, port_qm.len, "message_fill len ({} bytes)", payload.len());
        assert_bytes_eq(&ref_msg, &port_qm.msg, "message_fill msg");
        unsafe { message_free(ref_qm) };
    }
    for values in [
        Vec::<u32>::new(),
        vec![0],
        vec![1, 2, 3],
        (0..20u32).collect(),
        vec![0x1234_5678; 30],
        // overflows MESSAGE_PAYLOAD_MAX -> "Encode error" on both sides
        vec![0x7fff_ffff; 40],
    ] {
        let ref_qm = unsafe {
            message_alloc_and_encode(values.as_ptr() as *mut u32, values.len() as i32)
        };
        let ref_len = unsafe { (*ref_qm).len };
        let ref_msg: [u8; MESSAGE_MAX] = unsafe { (*ref_qm).msg };
        let port_qm = chelper_serial::msgblock::message_alloc_and_encode(&values);
        assert_eq!(ref_len, port_qm.len, "message_alloc_and_encode len for {} values", values.len());
        assert_bytes_eq(&ref_msg, &port_qm.msg, "message_alloc_and_encode msg");
        unsafe { message_free(ref_qm) };
    }
    // Compare the complete error stream with the reference, including
    // encode-overflow failures.
    assert_vec_eq(&c_logs().clone(), &port_logs().clone(), "msgblock log lines");
    c_logs().clear();
    port_logs().clear();
}

// ---- lockstep wrappers ----------------------------------------------------

/// A pulled or extracted message with its times as raw bits.
#[derive(Clone, PartialEq, Debug)]
struct Pulled {
    len: i32,
    notify_id: u64,
    sent_bits: u64,
    recv_bits: u64,
    head: Vec<u8>,
}

impl Pulled {
    fn of(len: i32, notify_id: u64, sent: f64, recv: f64, msg: &[u8]) -> Pulled {
        Pulled {
            len,
            notify_id,
            sent_bits: sent.to_bits(),
            recv_bits: recv.to_bits(),
            head: msg[..len.max(0) as usize].to_vec(),
        }
    }
}

fn zero_pqm() -> CPullQueueMessage {
    CPullQueueMessage { msg: [0; MESSAGE_MAX], len: 0, sent_time: 0.0, receive_time: 0.0, notify_id: 0 }
}

enum SqKind {
    Ref(*mut c_void),
    Port(Option<SerialQueue>),
}

/// One serialqueue, either the C reference or the port.
struct Sq {
    kind: SqKind,
}

enum CqKind {
    Ref(*mut c_void),
    Port(Option<Arc<CommandQueue>>),
}

struct Cq {
    kind: CqKind,
}

impl Cq {
    fn alloc(is_ref: bool) -> Cq {
        if is_ref {
            let cq = unsafe { serialqueue_alloc_commandqueue() };
            assert!(!cq.is_null(), "ref alloc_commandqueue returned null");
            Cq { kind: CqKind::Ref(cq) }
        } else {
            Cq { kind: CqKind::Port(Some(alloc_commandqueue())) }
        }
    }

    fn raw(&self) -> *mut c_void {
        match &self.kind {
            CqKind::Ref(p) => *p,
            CqKind::Port(_) => panic!("raw() on a port command queue"),
        }
    }

    fn port(&self) -> &Arc<CommandQueue> {
        match &self.kind {
            CqKind::Port(Some(c)) => c,
            _ => panic!("port() on a reference or freed command queue"),
        }
    }

    fn free(&mut self) {
        match &mut self.kind {
            CqKind::Ref(p) => {
                if !p.is_null() {
                    unsafe { serialqueue_free_commandqueue(*p) };
                    *p = std::ptr::null_mut();
                }
            }
            CqKind::Port(o) => {
                if let Some(c) = o.take() {
                    free_commandqueue(c);
                }
            }
        }
    }
}

impl Drop for Cq {
    fn drop(&mut self) {
        self.free();
    }
}

impl Sq {
    fn alloc(is_ref: bool, serial_fd: i32, ty: u8, client_id: i32, name: &[u8]) -> Sq {
        if is_ref {
            let mut name16 = [0u8; 16];
            let n = name.len().min(16);
            name16[..n].copy_from_slice(&name[..n]);
            name16[15] = 0;
            let sq = unsafe { serialqueue_alloc(serial_fd, ty, client_id, name16.as_mut_ptr()) };
            assert!(!sq.is_null(), "ref serialqueue_alloc returned null");
            Sq { kind: SqKind::Ref(sq) }
        } else {
            let sq = SerialQueue::alloc(serial_fd, ty, client_id, name)
                .expect("port serialqueue_alloc returned None");
            Sq { kind: SqKind::Port(Some(sq)) }
        }
    }

    fn with<R>(&self, r: impl FnOnce(*mut c_void) -> R, p: impl FnOnce(&SerialQueue) -> R) -> R {
        match &self.kind {
            SqKind::Ref(ptr) => {
                assert!(!ptr.is_null(), "ref serialqueue already freed");
                r(*ptr)
            }
            SqKind::Port(Some(s)) => p(s),
            SqKind::Port(None) => panic!("port serialqueue already freed"),
        }
    }

    fn set_wire_frequency(&self, f: f64) {
        self.with(|sq| unsafe { serialqueue_set_wire_frequency(sq, f) },
                  |s| s.set_wire_frequency(f));
    }

    fn set_receive_window(&self, w: i32) {
        self.with(|sq| unsafe { serialqueue_set_receive_window(sq, w) },
                  |s| s.set_receive_window(w));
    }

    fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: u64) {
        self.with(|sq| unsafe { serialqueue_set_clock_est(sq, freq, conv_time, conv_clock) },
                  |s| s.set_clock_est(freq, conv_time, conv_clock));
    }

    fn clock_est(&self) -> (u64, u64, u64) {
        self.with(
            |sq| {
                let mut ce = CClockEstimate::default();
                unsafe { serialqueue_get_clock_est(sq, &mut ce) };
                (ce.conv_clock, ce.conv_time.to_bits(), ce.est_freq.to_bits())
            },
            |s| {
                let ce = s.get_clock_est();
                (ce.conv_clock, ce.conv_time.to_bits(), ce.est_freq.to_bits())
            },
        )
    }

    fn send(&self, cq: &Cq, msg: &[u8], min_clock: u64, req_clock: u64, notify_id: u64) {
        self.with(
            |sq| {
                let mut buf = msg.to_vec();
                unsafe {
                    serialqueue_send(sq, cq.raw(), buf.as_mut_ptr(), buf.len() as i32,
                                     min_clock, req_clock, notify_id)
                };
            },
            |s| s.send(cq.port(), msg, min_clock, req_clock, notify_id),
        );
    }

    /// `serialqueue_send_batch` with `min_clock = req_clock = 0` messages.
    fn send_batch(&self, cq: &Cq, msgs: &[Vec<u8>]) {
        self.with(
            |sq| {
                // Build the C intrusive list: root <-> qm1 <-> qm2 ... <-> root.
                let mut root: Box<[*mut c_void; 2]> =
                    Box::new([std::ptr::null_mut(), std::ptr::null_mut()]);
                let root_node = &mut *root as *mut [*mut c_void; 2] as *mut c_void;
                let mut prev_node = root_node;
                for m in msgs {
                    let mut data = m.clone();
                    let qm = unsafe { message_fill(data.as_mut_ptr(), data.len() as i32) };
                    let node = unsafe { std::ptr::addr_of_mut!((*qm).node_next) } as *mut c_void;
                    // SAFETY: every node starts with `next` then `prev` pointers.
                    unsafe {
                        *(prev_node as *mut *mut c_void) = node;
                        *(node as *mut *mut c_void).add(1) = prev_node;
                    }
                    prev_node = node;
                }
                unsafe {
                    *(prev_node as *mut *mut c_void) = root_node;
                    *(root_node as *mut *mut c_void).add(1) = prev_node;
                    serialqueue_send_batch(sq, cq.raw(), root_node);
                }
            },
            |s| {
                let batch: Vec<Box<chelper_serial::QueueMessage>> =
                    msgs.iter().map(|m| chelper_serial::msgblock::message_fill(m, m.len() as i32)).collect();
                s.send_batch(cq.port(), batch);
            },
        );
    }

    fn pull(&self) -> Pulled {
        self.with(
            |sq| {
                let mut pqm = zero_pqm();
                unsafe { serialqueue_pull(sq, &mut pqm) };
                assert_ne!(pqm.len, -1, "ref pull returned the exit sentinel");
                Pulled::of(pqm.len, pqm.notify_id, pqm.sent_time, pqm.receive_time, &pqm.msg)
            },
            |s| {
                let m = s.pull().expect("port pull returned None before exit");
                Pulled::of(m.len, m.notify_id, m.sent_time, m.receive_time, &m.msg)
            },
        )
    }

    fn stats(&self) -> String {
        let buf = self.with(
            |sq| {
                let mut buf = vec![0u8; 512];
                unsafe { serialqueue_get_stats(sq, buf.as_mut_ptr(), buf.len() as i32) };
                buf
            },
            |s| {
                let mut buf = vec![0u8; 512];
                s.get_stats(&mut buf);
                buf
            },
        );
        let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        String::from_utf8_lossy(&buf[..end]).into_owned()
    }

    fn extract(&self, sentq: bool) -> Vec<Pulled> {
        self.with(
            |sq| {
                let mut arr: Vec<CPullQueueMessage> = (0..1024).map(|_| zero_pqm()).collect();
                let n = unsafe { serialqueue_extract_old(sq, sentq as i32, arr.as_mut_ptr(), 1024) };
                arr[..n as usize]
                    .iter()
                    .map(|p| Pulled::of(p.len, p.notify_id, p.sent_time, p.receive_time, &p.msg))
                    .collect()
            },
            |s| {
                s.extract_old(sentq, 1024)
                    .iter()
                    .map(|p| Pulled::of(p.len, p.notify_id, p.sent_time, p.receive_time, &p.msg))
                    .collect()
            },
        )
    }

    fn add_fastreader(&self, prefix: &[u8]) -> Option<Box<CFastReader>> {
        self.with(
            |sq| {
                let mut fr = Box::new(CFastReader {
                    node: [std::ptr::null_mut(); 2],
                    func: c_fast_cb,
                    prefix_len: prefix.len() as i32,
                    prefix: [0; MESSAGE_MAX],
                });
                fr.prefix[..prefix.len()].copy_from_slice(prefix);
                unsafe { serialqueue_add_fastreader(sq, &mut *fr) };
                Some(fr)
            },
            |s| {
                // SAFETY: the callback ignores its context pointer.
                unsafe { s.add_fastreader(port_fast_cb, prefix, std::ptr::null_mut()) };
                None
            },
        )
    }

    fn rm_fastreader(&self, fr: &mut Option<Box<CFastReader>>) {
        match &self.kind {
            SqKind::Ref(sq) => {
                let f = fr.as_mut().expect("ref fastreader registered");
                unsafe { serialqueue_rm_fastreader(*sq, &mut **f) };
            }
            SqKind::Port(Some(s)) => {
                // SAFETY: same function and context as the registration.
                unsafe { s.rm_fastreader(port_fast_cb, std::ptr::null_mut()) };
            }
            SqKind::Port(None) => panic!("port serialqueue already freed"),
        }
        *fr = None;
    }

    fn exit(&self) {
        self.with(|sq| unsafe { serialqueue_exit(sq) }, |s| s.exit());
    }

    fn free(&mut self) {
        match &mut self.kind {
            SqKind::Ref(p) => {
                if !p.is_null() {
                    unsafe { serialqueue_free(*p) };
                    *p = std::ptr::null_mut();
                }
            }
            SqKind::Port(o) => {
                if let Some(s) = o.take() {
                    s.free();
                }
            }
        }
    }
}

impl Drop for Sq {
    fn drop(&mut self) {
        self.free();
    }
}

// ---- section 2: debug-file replay ---------------------------------------

fn value_bytes(v: &Value) -> Vec<u8> {
    match v {
        Value::Bytes(b) => b.clone(),
        Value::List(items) => items
            .iter()
            .map(|x| match x {
                Value::Int(i) => *i as u8,
                _ => panic!("expected int in byte list, got {x:?}"),
            })
            .collect(),
        _ => panic!("expected bytes/list value, got {v:?}"),
    }
}

fn value_u64(v: &Value) -> u64 {
    match v {
        Value::Int(i) => *i as u64,
        Value::BigInt(s) => s.parse::<u64>().expect("bigint clock value"),
        _ => panic!("expected int value, got {v:?}"),
    }
}

fn value_i32(v: &Value) -> i32 {
    match v {
        Value::Int(i) => *i as i32,
        _ => panic!("expected int value, got {v:?}"),
    }
}

fn value_f64(v: &Value) -> f64 {
    match v {
        Value::Float(fv) => f64_from_hex(&fv.hex).expect("hex float"),
        _ => panic!("expected float value, got {v:?}"),
    }
}

fn value_addr(v: &Value) -> String {
    match v {
        Value::Opaque { addr, .. } => addr.clone(),
        _ => panic!("expected opaque pointer, got {v:?}"),
    }
}

static FILE_COUNTER: AtomicUsize = AtomicUsize::new(0);

fn temp_file_path(tag: &str) -> String {
    let dir = std::env::temp_dir();
    let n = FILE_COUNTER.fetch_add(1, Ordering::SeqCst);
    format!("{}/sqparity_{}_{}_{}.bin", dir.display(), std::process::id(), tag, n)
}

struct ReplaySq {
    addr: String,
    sq: [Sq; 2],
    files: [std::fs::File; 2],
    paths: [String; 2],
}

struct ReplayCq {
    addr: String,
    cq: [Cq; 2],
}

/// Replay one recorded scenario against the reference and the port in
/// lockstep. Returns the debug-file bytes of every serialqueue, one
/// vector per side.
fn replay_scenario(records: &[Record]) -> [Vec<Vec<u8>>; 2] {
    let mut cqs: Vec<ReplayCq> = Vec::new();
    let mut sqs: Vec<ReplaySq> = Vec::new();

    fn find_sq(sqs: &[ReplaySq], addr: &str) -> usize {
        sqs.iter().position(|s| s.addr == addr).expect("sq addr in recording")
    }
    fn find_cq(cqs: &[ReplayCq], addr: &str) -> usize {
        cqs.iter().position(|c| c.addr == addr).expect("cq addr in recording")
    }

    for rec in records {
        match rec.function.as_str() {
            "serialqueue_alloc" => {
                let fd_type = value_bytes(&rec.args[1])[0];
                let client_id = match &rec.args[2] {
                    Value::Int(i) => *i as i32,
                    _ => 0,
                };
                let name = value_bytes(&rec.args[3]);
                let addr = value_addr(rec.ret.as_ref().expect("alloc ret"));
                let paths = [temp_file_path("ref-debugfile"), temp_file_path("port-debugfile")];
                let files = [
                    std::fs::File::create(&paths[0]).expect("temp file"),
                    std::fs::File::create(&paths[1]).expect("temp file"),
                ];
                let sq = [
                    Sq::alloc(true, files[0].as_raw_fd(), fd_type, client_id, &name),
                    Sq::alloc(false, files[1].as_raw_fd(), fd_type, client_id, &name),
                ];
                sqs.push(ReplaySq { addr, sq, files, paths });
            }
            "serialqueue_alloc_commandqueue" => {
                let addr = value_addr(rec.ret.as_ref().expect("alloc_commandqueue ret"));
                cqs.push(ReplayCq { addr, cq: [Cq::alloc(true), Cq::alloc(false)] });
            }
            "serialqueue_set_clock_est" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                let est_freq = value_f64(&rec.args[1]);
                // The recorded conv_time is wall time of the recording
                // machine; both sides get the current virtual time so the
                // relative schedule a message sees is the recorded one.
                let base = time::monotonic();
                let conv_clock = value_u64(&rec.args[3]);
                for sq in &s.sq {
                    sq.set_clock_est(est_freq, base, conv_clock);
                }
            }
            "serialqueue_set_wire_frequency" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                let freq = value_f64(&rec.args[1]);
                for sq in &s.sq {
                    sq.set_wire_frequency(freq);
                }
            }
            "serialqueue_set_receive_window" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                let window = value_i32(&rec.args[1]);
                for sq in &s.sq {
                    sq.set_receive_window(window);
                }
            }
            "serialqueue_send" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                let c = &cqs[find_cq(&cqs, &value_addr(&rec.args[1]))];
                let msg = value_bytes(&rec.args[2]);
                let min_clock = value_u64(&rec.args[4]);
                let req_clock = value_u64(&rec.args[5]);
                let notify_id = value_u64(&rec.args[6]);
                for i in 0..2 {
                    s.sq[i].send(&c.cq[i], &msg, min_clock, req_clock, notify_id);
                }
            }
            "serialqueue_exit" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                // The recording stops the thread right after the last send.
                // Let the background threads drain everything first, or
                // the exit flag would be set before they ever run.
                time::sleep(3.0);
                for sq in &s.sq {
                    sq.exit();
                }
            }
            "serialqueue_free" => {
                let idx = find_sq(&sqs, &value_addr(&rec.args[0]));
                for sq in sqs[idx].sq.iter_mut() {
                    sq.free();
                }
            }
            "serialqueue_free_commandqueue" => {
                let idx = find_cq(&cqs, &value_addr(&rec.args[0]));
                for cq in cqs[idx].cq.iter_mut() {
                    cq.free();
                }
            }
            "serialqueue_get_stats" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                assert_eq!(s.sq[0].stats(), s.sq[1].stats(), "stats string ({})", rec.scenario);
            }
            "serialqueue_extract_old" => {
                let s = &sqs[find_sq(&sqs, &value_addr(&rec.args[0]))];
                let sentq = matches!(&rec.args[1], Value::Int(1));
                assert_vec_eq(&s.sq[0].extract(sentq), &s.sq[1].extract(sentq), "extract_old");
            }
            other => panic!("unsupported recorded function in replay: {other} (scenario {})", rec.scenario),
        }
    }

    // Command queues the recording never freed are empty by now (freeing
    // a serialqueue drains them), so freeing them is silent on both sides.
    for c in cqs.iter_mut() {
        for cq in c.cq.iter_mut() {
            cq.free();
        }
    }

    let mut out: [Vec<Vec<u8>>; 2] = [Vec::new(), Vec::new()];
    for s in sqs {
        for (side, dest) in out.iter_mut().enumerate() {
            let bytes = std::fs::read(&s.paths[side]).unwrap_or_default();
            let _ = std::fs::remove_file(&s.paths[side]);
            dest.push(bytes);
        }
        drop(s.files);
    }
    out
}

fn debug_file_replay() {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../tests/fixtures/recordings");
    let scenarios = load_unit(&root, "serialqueue").expect("load serialqueue recordings");
    assert!(!scenarios.is_empty(), "no serialqueue recordings found under {root:?}");
    for (i, records) in scenarios.iter().enumerate() {
        c_logs().clear();
        port_logs().clear();
        let [ref_files, port_files] = replay_scenario(records);
        assert_eq!(ref_files.len(), port_files.len(), "scenario {i}: debug file count");
        let total: usize = ref_files.iter().map(|b| b.len()).sum();
        assert!(total > 0, "scenario {i}: the replay wrote no debug-file bytes");
        for (j, (r, p)) in ref_files.iter().zip(port_files.iter()).enumerate() {
            assert_bytes_eq(r, p, &format!("scenario {i} debug file {j} ({} records)", records.len()));
        }
        let ref_logs: Vec<String> = c_logs().clone();
        let port_log_lines: Vec<String> = port_logs().clone();
        assert_vec_eq(&ref_logs, &port_log_lines, &format!("scenario {i} log lines"));
    }
    c_logs().clear();
    port_logs().clear();
}

// ---- section 3: pipe scenario ---------------------------------------------

/// Build a peer frame with the reference CRC (peer scripting does
/// not depend on the implementation under test for CRCs).
fn peer_frame(seq_nibble: u8, payload: &[u8], corrupt_crc: bool) -> Vec<u8> {
    let mut frame = vec![0u8; 2 + payload.len() + 3];
    frame[0] = frame.len() as u8;
    frame[1] = 0x10 | (seq_nibble & 0x0f);
    frame[2..2 + payload.len()].copy_from_slice(payload);
    let crc = unsafe { msgblock_crc16_ccitt(frame.as_mut_ptr(), (frame.len() - 3) as u8) };
    let crc = if corrupt_crc { crc.wrapping_add(1) } else { crc };
    let len = frame.len();
    frame[len - 3] = (crc >> 8) as u8;
    frame[len - 2] = (crc & 0xff) as u8;
    frame[len - 1] = MESSAGE_SYNC;
    frame
}

/// An ack frame: the peer expects block `next_seq` next.
fn peer_ack(next_seq: u8) -> Vec<u8> {
    peer_frame(next_seq, &[], false)
}

/// The scripted MCU end of one socketpair.
struct Peer {
    fd: i32,
    rx: Vec<u8>,
    transcript: Vec<u8>,
    can_peer_id: Option<u32>,
    can_packets: Vec<Vec<u8>>,
}

impl Peer {
    fn send(&self, bytes: &[u8]) {
        if let Some(id) = self.can_peer_id {
            for payload in bytes.chunks(8) {
                let mut frame = [0; 16];
                frame[..4].copy_from_slice(&id.to_ne_bytes());
                frame[4] = payload.len() as u8;
                frame[8..8 + payload.len()].copy_from_slice(payload);
                assert_eq!(fd::write(self.fd, &frame).expect("CAN peer write"), frame.len());
            }
            return;
        }
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
            if self.can_peer_id.is_some() {
                assert_eq!(n, 16, "CAN frame size");
                let len = chunk[4] as usize;
                assert!(len <= 8, "CAN data length");
                // Padding and bytes past can_dlc are uninitialized in the
                // reference. Compare only its defined frame fields.
                let mut packet = chunk[..5].to_vec();
                packet.extend_from_slice(&chunk[8..8 + len]);
                self.can_packets.push(packet);
                self.rx.extend_from_slice(&chunk[8..8 + len]);
            } else {
                self.rx.extend_from_slice(&chunk[..n]);
            }
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
            assert!((MESSAGE_MIN..=MESSAGE_MAX).contains(&len), "peer saw a bad block length {len:#x}");
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

/// The reference and the port, each with its own peer.
struct Lane {
    name: &'static str,
    sq: Sq,
    cq: Cq,
    peer: Peer,
    sq_fd: i32,
    fastreader: Option<Box<CFastReader>>,
    notify: Option<Pulled>,
    data: Option<Pulled>,
}

impl Drop for Lane {
    fn drop(&mut self) {
        self.sq.free();
        self.cq.free();
        fd::close(self.sq_fd);
        fd::close(self.peer.fd);
    }
}

fn new_socketpair(can: bool) -> (i32, i32) {
    const AF_UNIX: i32 = 1;
    const SOCK_STREAM: i32 = 1;
    const SOCK_DGRAM: i32 = 2;
    let mut sv = [0i32; 2];
    // SAFETY: `sv` is valid storage for two fds.
    let rc = unsafe { socketpair(AF_UNIX, if can { SOCK_DGRAM } else { SOCK_STREAM }, 0, sv.as_mut_ptr()) };
    assert_eq!(rc, 0, "socketpair failed");
    (sv[0], sv[1])
}

fn peer_send_all(lanes: &[Lane], bytes: &[u8]) {
    for l in lanes {
        l.peer.send(bytes);
    }
}

/// Wait for the scripted number of reference blocks, then compare the port
/// with the observed reference sequence numbers, not a handwritten packet.
fn expect_blocks(lanes: &mut [Lane], count: usize, max_steps: u32, what: &str) {
    let mut got: Vec<Vec<u8>> = vec![Vec::new(); lanes.len()];
    for _ in 0..max_steps {
        time::sleep(0.001);
        for (g, l) in got.iter_mut().zip(lanes.iter_mut()) {
            g.extend(l.peer.drain());
        }
        if got[0].len() >= count && got.iter().all(|g| g.len() >= got[0].len()) {
            break;
        }
    }
    assert!(got[0].len() >= count, "reference did not finish {what}");
    for (g, l) in got.iter().zip(lanes.iter()).skip(1) {
        assert_vec_eq(&got[0], g, &format!("{} peer blocks: {what}", l.name));
    }
}

/// Observe a scripted delay and compare every block with the reference.
fn expect_quiet(lanes: &mut [Lane], steps: u32, what: &str) {
    for _ in 0..steps {
        time::sleep(0.001);
        let reference = lanes[0].peer.drain();
        for l in lanes.iter_mut().skip(1) {
            let blocks = l.peer.drain();
            assert_vec_eq(&reference, &blocks, &format!("{} peer delay: {what}", l.name));
        }
    }
}

struct LaneResult {
    transcript: Vec<u8>,
    can_packets: Vec<Vec<u8>>,
    notify: Pulled,
    data: Pulled,
    stats: String,
    clock_est: (u64, u64, u64),
    extract_sent: Vec<Pulled>,
    extract_recv: Vec<Pulled>,
    fast: Vec<FastEvent>,
    logs: Vec<String>,
}

fn pipe_scenario(ty: u8) {
    c_logs().clear();
    port_logs().clear();
    capture().c_fast.lock().clear();
    capture().port_fast.lock().clear();

    let mut lanes: Vec<Lane> = Vec::new();
    for (is_ref, name) in [(true, "ref"), (false, "port")] {
        let (sq_fd, peer_fd) = new_socketpair(ty == b'c');
        fd::set_nonblock(peer_fd).expect("peer nonblock");
        let sq = Sq::alloc(is_ref, sq_fd, ty, 0x100, b"sqtest");
        let cq = Cq::alloc(is_ref);
        lanes.push(Lane {
            name,
            sq,
            cq,
            peer: Peer {
                fd: peer_fd, rx: Vec::new(), transcript: Vec::new(),
                can_peer_id: (ty == b'c').then_some(0x101), can_packets: Vec::new(),
            },
            sq_fd,
            fastreader: None,
            notify: None,
            data: None,
        });
    }
    let base = time::monotonic();
    for l in &lanes {
        l.sq.set_wire_frequency(250_000.0);
    }
    assert_eq!(lanes[0].sq.clock_est(), lanes[1].sq.clock_est(), "clock estimate");
    if ty == b'c' {
        // A valid CAN packet for a different client must not enter the
        // serial framing buffer.
        for l in &lanes {
            let mut frame = [0; 16];
            frame[..4].copy_from_slice(&0x201u32.to_ne_bytes());
            frame[4] = 1;
            frame[8] = 0x55;
            assert_eq!(fd::write(l.peer.fd, &frame).expect("other-client CAN write"), 16);
        }
        time::sleep(0.001);
    }

    // Phase A: one message, acked after 4 ms (first RTT sample).
    for l in &lanes {
        l.sq.send(&l.cq, &[1, 2, 3], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "A: first block");
    time::sleep(0.003);
    peer_send_all(&lanes, &peer_ack(2));
    time::sleep(0.001);
    for l in &lanes {
        l.sq.set_clock_est(1e12, base, 0);
    }

    // Phase B: two messages share one block; acked after 3 ms.
    for l in &lanes {
        l.sq.send(&l.cq, &[4, 5], 0, 0, 0);
        l.sq.send(&l.cq, &[6, 7, 8, 9], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "B: shared block");
    time::sleep(0.002);
    peer_send_all(&lanes, &peer_ack(3));
    time::sleep(0.001);

    // Phase C: a duplicate ack is a NAK and triggers a fast retransmit.
    for l in &lanes {
        l.sq.send(&l.cq, &[10, 11, 12, 13], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "C: block 3");
    peer_send_all(&lanes, &peer_ack(3));
    expect_blocks(&mut lanes, 1, 50, "C: NAK retransmit of block 3");
    peer_send_all(&lanes, &peer_ack(4));
    time::sleep(0.001);

    // Phase C2: two blocks in flight, NAK, then a second NAK for the same
    // retransmit window ("second nak" branch of retransmit_event).
    for l in &lanes {
        l.sq.send(&l.cq, &[14, 15], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "C2: block 4");
    for l in &lanes {
        l.sq.send(&l.cq, &[16], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "C2: block 5");
    peer_send_all(&lanes, &peer_ack(4));
    expect_blocks(&mut lanes, 2, 50, "C2: NAK retransmit of blocks 4 and 5");
    peer_send_all(&lanes, &peer_ack(5));
    time::sleep(0.001);
    peer_send_all(&lanes, &peer_ack(5));
    expect_blocks(&mut lanes, 1, 50, "C2: second NAK retransmit of block 5");
    peer_send_all(&lanes, &peer_ack(6));
    time::sleep(0.001);

    // Phase D: no ack, so the retransmit timer fires (timeout path).
    for l in &lanes {
        l.sq.send(&l.cq, &[20, 21], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "D: block 6");
    expect_blocks(&mut lanes, 1, 600, "D: timeout retransmit of block 6");
    peer_send_all(&lanes, &peer_ack(7));
    time::sleep(0.001);

    // Phase E: a message with a notify id; the ack completes it.
    for l in &lanes {
        l.sq.send(&l.cq, &[30], 0, 0, 7);
    }
    expect_blocks(&mut lanes, 1, 50, "E: notify block");
    peer_send_all(&lanes, &peer_ack(8));
    time::sleep(0.001);
    for l in lanes.iter_mut() {
        let p = l.sq.pull();
        l.notify = Some(p);
    }

    // Phase F: a data frame from the peer reaches pull and the fast reader.
    for l in lanes.iter_mut() {
        l.fastreader = l.sq.add_fastreader(&[0x41]);
    }
    peer_send_all(&lanes, &peer_frame(8, &[0x40], false));
    time::sleep(0.001);
    assert_eq!(lanes[0].sq.pull(), lanes[1].sq.pull(), "nonmatching fast reader");
    peer_send_all(&lanes, &peer_frame(8, &[0x41, 0x42, 0x43], false));
    time::sleep(0.001);
    for l in lanes.iter_mut() {
        let p = l.sq.pull();
        l.data = Some(p);
        l.sq.rm_fastreader(&mut l.fastreader);
    }

    // Phase G: a frame with a bad CRC is dropped and counted invalid.
    peer_send_all(&lanes, &peer_frame(8, &[0x44, 0x45], true));
    time::sleep(0.005);
    // A valid frame acknowledging an unsent block is counted invalid.
    peer_send_all(&lanes, &peer_frame(15, &[0x46], false));
    time::sleep(0.001);

    // Phase H: req_clock 20 ms ahead with min_clock 0. send_batch pulls
    // min_clock to req_clock - (3 << 29), so the message waits for that.
    let ahead = kos::cconv::f64_to_u64((time::monotonic() - base) * 1e12) + 20_000_000_000;
    for l in &lanes {
        l.sq.send(&l.cq, &[40, 41], 0, ahead, 0);
    }
    expect_quiet(&mut lanes, 5, "H: message must wait for min_clock");
    expect_blocks(&mut lanes, 1, 100, "H: min_clock release");
    peer_send_all(&lanes, &peer_ack(9));
    time::sleep(0.001);

    // Phase I: a background priority message.
    for l in &lanes {
        l.sq.send(&l.cq, &[50], 0, BACKGROUND_PRIORITY_CLOCK, 0);
    }
    expect_blocks(&mut lanes, 1, 300, "I: background block");
    peer_send_all(&lanes, &peer_ack(10));
    time::sleep(0.001);

    // Phase J: a batch of three messages in one block.
    for l in &lanes {
        l.sq.send_batch(&l.cq, &[vec![60], vec![61, 62], vec![63]]);
    }
    expect_blocks(&mut lanes, 1, 50, "J: batch block");
    peer_send_all(&lanes, &peer_ack(11));
    time::sleep(0.001);

    // Phase W: receive window. A block in flight leaves no room for the
    // next one until its ack arrives.
    for l in &lanes {
        l.sq.set_receive_window(70);
        l.sq.send(&l.cq, &[70, 71, 72], 0, 0, 0);
    }
    expect_blocks(&mut lanes, 1, 50, "W: first windowed block");
    for l in &lanes {
        l.sq.send(&l.cq, &[73], 0, 0, 0);
    }
    expect_quiet(&mut lanes, 5, "W: window must hold the second block");
    peer_send_all(&lanes, &peer_ack(12));
    expect_blocks(&mut lanes, 1, 50, "W: second windowed block after the ack");
    peer_send_all(&lanes, &peer_ack(13));
    time::sleep(0.01);

    // Fill more than MAX_PENDING_BLOCKS and acknowledge the first window.
    // The full payloads force one message per protocol block.
    for l in &lanes {
        l.sq.set_receive_window(0);
        let batch = vec![vec![1; MESSAGE_PAYLOAD_MAX]; 13];
        l.sq.send_batch(&l.cq, &batch);
    }
    expect_blocks(&mut lanes, 12, 50, "pending block limit and sequence rollover");
    expect_quiet(&mut lanes, 5, "pending block limit");
    peer_send_all(&lanes, &peer_ack(9));
    expect_blocks(&mut lanes, 1, 50, "pending block release");
    peer_send_all(&lanes, &peer_ack(10));
    time::sleep(0.001);

    // Collect results, then shut both sides down.
    let mut results: Vec<LaneResult> = Vec::new();
    for l in lanes.iter_mut() {
        let stats = l.sq.stats();
        let clock_est = l.sq.clock_est();
        let extract_sent = l.sq.extract(true);
        let extract_recv = l.sq.extract(false);
        l.sq.exit();
        l.sq.free();
        l.cq.free();
        let is_ref = l.name == "ref";
        results.push(LaneResult {
            transcript: std::mem::take(&mut l.peer.transcript),
            can_packets: std::mem::take(&mut l.peer.can_packets),
            notify: l.notify.take().expect("notify pull"),
            data: l.data.take().expect("data pull"),
            stats,
            clock_est,
            extract_sent,
            extract_recv,
            fast: if is_ref { capture().c_fast.lock().clone() } else { capture().port_fast.lock().clone() },
            logs: if is_ref { c_logs().clone() } else { port_logs().clone() },
        });
    }
    let port = results.pop().expect("port result");
    let reference = results.pop().expect("ref result");

    assert_bytes_eq(&reference.transcript, &port.transcript, "pipe transcript");
    assert_vec_eq(&reference.can_packets, &port.can_packets, "CAN packets");
    assert_eq!(reference.notify, port.notify, "notify pull");
    assert_eq!(reference.data, port.data, "data pull");
    assert_eq!(reference.stats, port.stats, "stats string");
    assert_eq!(reference.clock_est, port.clock_est, "clock estimate");
    assert_vec_eq(&reference.extract_sent, &port.extract_sent, "extract_old(sent)");
    assert_vec_eq(&reference.extract_recv, &port.extract_recv, "extract_old(receive)");
    assert_vec_eq(&reference.fast, &port.fast, "fast reader events");
    assert_vec_eq(&reference.logs, &port.logs, "pipe log lines");
}

// ---- case table and entry points ------------------------------------------

fn uart_scenario() {
    pipe_scenario(b'u');
}

fn can_scenario() {
    pipe_scenario(b'c');
}

/// Every parity case in execution order. The flag marks the cases whose
/// byte stream depends on background-thread wakeups and therefore needs
/// the virtual clock.
const CASES: [(&str, bool, fn()); 4] = [
    ("serialqueue_msgblock", false, msgblock_units),
    ("serialqueue_recordings", true, debug_file_replay),
    ("serialqueue_uart", true, uart_scenario),
    ("serialqueue_can", true, can_scenario),
];

/// Process-wide setup shared by every entry point: the log callbacks, the
/// capture and the watchdog are singletons.
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
        "serialqueue parity requires VCLOCK_SOCK and parity/vclock"
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
    let _serial = LIBTEST_SERIAL.lock();
    setup();
    let (_, needs_clock, case) = CASES
        .iter()
        .copied()
        .find(|(n, _, _)| *n == name)
        .expect("case listed in CASES");
    run_case(needs_clock, case);
}

#[test]
fn serialqueue_msgblock() {
    run_libtest_case("serialqueue_msgblock");
}

// The cases below depend on background-thread wakeups and only produce the
// reference byte stream under the virtual clock, where the `.init_array`
// constructor runs them (`cargo_test.sh chelper_serial:serialqueue@vclock`).
// A plain `cargo test` lists them as ignored instead of failing.

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn serialqueue_recordings() {
    run_libtest_case("serialqueue_recordings");
}

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn serialqueue_uart() {
    run_libtest_case("serialqueue_uart");
}

#[test]
#[ignore = "needs the virtual clock (VCLOCK_SOCK); runs under parity/vclock"]
fn serialqueue_can() {
    run_libtest_case("serialqueue_can");
}
