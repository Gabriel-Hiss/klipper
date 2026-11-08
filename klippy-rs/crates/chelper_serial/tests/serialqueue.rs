//! Differential parity tests for the `serialqueue` unit.
//!
//! The test drives the frozen C library (`chelper_ref`, the unchanged
//! `klippy/chelper` sources) and the Rust port through identical
//! inputs and compares outputs bit for bit.
//!
//! Sections (all inside one `#[test]`; see below):
//! 1. `msgblock` unit differential (CRC, check, decode, encode, clock
//!    helpers) against the C functions. Pure functions; no virtual
//!    clock needed.
//! 2. Debug-file replay: every recording under
//!    `tests/fixtures/recordings/serialqueue/*.jsonl` is replayed
//!    against both sides in file mode (`SQT_DEBUGFILE`, output to a
//!    temp file), and the written bytes plus the `errorf` log lines
//!    are compared byte for byte.
//! 3. Pipe scenario with a scripted peer covering acks, duplicate-ack
//!    NAK fast retransmit, RTO retransmit, notify (`send_wait_ack`)
//!    completion, a data message, a bad-CRC frame, the stats string
//!    and `extract_old` (sent + receive) results. The peer transcript,
//!    pull results, stats string and extract_old results are
//!    compared between the two sides. Absolute times in pull/extract
//!    results are compared after subtracting the run's base time
//!    (see below); the `[DEBUG-motion]` dump block produced by
//!    `extract_old(sentq=1)` carries absolute-time fields, so the
//!    dump *shape* (line count) is compared while its float fields
//!    are not (its integer fields are covered via the stats string).
//!
//! Which tests need the virtual clock (`parity/vclock`):
//!
//! Sections 2 and 3 need vclock. Block packing is decided by the
//! background thread's wakeup timing: without a virtual clock, wall
//! clock scheduling decides whether queued messages pack into one
//! block or split across blocks, so the written byte stream is
//! nondeterministic even though each side stays internally
//! consistent. Under vclock the two runs (reference first, then
//! port, sequentially in this process) replay deterministically, and
//! because virtual time is only ever advanced by jumps to relative
//! deadlines, subtracting the run's base time (`kos::time::monotonic`
//! captured right after `set_clock_est`/`set_wire_frequency`) makes
//! the two timelines comparable. The recorded `conv_time` values are
//! wall-clock times from the recording machine; both sides receive
//! the same estimate re-based onto the current clock
//! (`conv_time = base`, with the recorded frequency and conv_clock),
//! so the relative schedule a message observes (min_clock gating,
//! req_clock windowing) is the one the recording encodes.
//!
//! Run command (integration owner):
//!
//! ```sh
//! make -C parity/vclock x86_64
//! parity/vclock/build/x86_64/vclockd --sock /tmp/vc.sock --epoch 1700000000 --trace /tmp/vc.trace &
//! LD_PRELOAD=$PWD/parity/vclock/build/x86_64/libvclock.so VCLOCK_SOCK=/tmp/vc.sock \
//!   cargo test -p chelper_serial --test serialqueue -- --nocapture
//! ```
//!
//! `msgblock_units` alone does not need vclock and can run without
//! it. All sections run inside one `#[test]` because the log capture
//! channels below are process-global (the C `python_logging_callback`
//! and the port's `set_logging_callback` are singletons); the single
//! test keeps them sequential with no extra test-threads flags needed.

use std::ffi::c_void;
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicUsize, Ordering};

use chelper_serial::msgblock::{
    crc16_ccitt, encode_int, ClockEstimate, MESSAGE_MAX, MESSAGE_PAYLOAD_MAX, MESSAGE_SYNC,
};
use chelper_serial::serialqueue::CommandQueue;
use chelper_serial::{alloc_commandqueue, free_commandqueue, PullQueueMessage, SerialQueue};
use kos::fd;
use kos::sync;
use kos::time;
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

#[repr(C)]
struct CQueueMessage {
    len: i32,
    _pad: i32,
    msg: [u8; MESSAGE_MAX],
    min_clock: u64,
    req_clock: u64,
    notify_id: u64,
    _node_next: *mut c_void,
    _node_prev: *mut c_void,
}

unsafe extern "C" {
    fn serialqueue_alloc(serial_fd: i32, serial_fd_type: u8, client_id: i32, name: *mut u8) -> *mut c_void;
    fn serialqueue_exit(sq: *mut c_void);
    fn serialqueue_free(sq: *mut c_void);
    fn serialqueue_alloc_commandqueue() -> *mut c_void;
    fn serialqueue_free_commandqueue(cq: *mut c_void);
    fn serialqueue_send(sq: *mut c_void, cq: *mut c_void, msg: *mut u8, len: i32,
                        min_clock: u64, req_clock: u64, notify_id: u64);
    fn serialqueue_pull(sq: *mut c_void, pqm: *mut CPullQueueMessage);
    fn serialqueue_set_wire_frequency(sq: *mut c_void, frequency: f64);
    fn serialqueue_set_receive_window(sq: *mut c_void, receive_window: i32);
    fn serialqueue_set_clock_est(sq: *mut c_void, est_freq: f64, conv_time: f64, conv_clock: u64);
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
}

// ---- log capture (process-global, C + port) -----------------------------

static C_LOGS: AtomicUsize = AtomicUsize::new(0);
static PORT_LOGS: AtomicUsize = AtomicUsize::new(0);

fn install_log_capture() {
    let c_box: Box<sync::Mutex<Vec<String>>> = Box::new(sync::Mutex::new(Vec::new()));
    C_LOGS.store(Box::into_raw(c_box) as usize, Ordering::SeqCst);
    let p_box: Box<sync::Mutex<Vec<String>>> = Box::new(sync::Mutex::new(Vec::new()));
    PORT_LOGS.store(Box::into_raw(p_box) as usize, Ordering::SeqCst);
    unsafe {
        set_python_logging_callback(c_log_callback);
    }
    chelper_serial::set_logging_callback(|msg: &str| {
        port_logs().push(msg.to_string());
    });
}

fn c_logs() -> sync::MutexGuard<'static, Vec<String>> {
    let raw = C_LOGS.load(Ordering::SeqCst);
    assert!(raw != 0, "C log capture not installed");
    // SAFETY: installed once by install_log_capture for the process
    // lifetime (leaked box).
    unsafe { (*(raw as *const sync::Mutex<Vec<String>>)).lock() }
}

fn port_logs() -> sync::MutexGuard<'static, Vec<String>> {
    let raw = PORT_LOGS.load(Ordering::SeqCst);
    assert!(raw != 0, "port log capture not installed");
    // SAFETY: installed once by install_log_capture for the process
    // lifetime (leaked box).
    unsafe { (*(raw as *const sync::Mutex<Vec<String>>)).lock() }
}

extern "C" fn c_log_callback(msg: *const u8) {
    let s = unsafe { std::ffi::CStr::from_ptr(msg as *const std::os::raw::c_char) };
    let owned = s.to_string_lossy().into_owned();
    c_logs().push(owned);
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
    // LCG over several lengths, including boundary lens 0..=64+.
    let mut state: u32 = 0x1234_5678;
    for &len in &[0usize, 1, 2, 3, 5, 8, 16, 59, 60, 64, 65, 100, 256] {
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
    install_log_capture();

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

    // encode_int exact-byte spot checks (from the C bounds).
    for (v, expect) in [
        (0u32, vec![0x00]),
        (1, vec![0x01]),
        (31, vec![0x1f]),
        (32, vec![0x20]),
        (95, vec![0x5f]),
        (96, vec![0x80, 0x60]),
        (127, vec![0x80, 0x7f]),
        (u32::MAX, vec![0x8f, 0xff, 0xff, 0xff, 0x7f]),
        (0xffff_ffffu32.wrapping_sub(31), vec![0xe0]), // -32 in one byte
        (0xffff_ffffu32.wrapping_sub(32), vec![0xff, 0x7f]), // -33 needs two
    ] {
        let mut buf = [0u8; 8];
        let next = encode_int(&mut buf, 0, v);
        assert_eq!(&buf[..next], &expect[..], "encode_int({v}) bytes");
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
            assert_f64_bits(t_ref, t_port, "clock_to_time clock={clock:#x}");
        }
        for &t in &[0.0f64, 1.0, -1.0, conv_time, conv_time + 0.5, 4173.195, 1e15, -1e15,
                       f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
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
        let mut ref_msg = [0u8; MESSAGE_MAX];
        ref_msg.copy_from_slice(&unsafe { (*ref_qm).msg });
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
        let mut ref_msg = [0u8; MESSAGE_MAX];
        ref_msg.copy_from_slice(&unsafe { (*ref_qm).msg });
        let port_qm = chelper_serial::msgblock::message_alloc_and_encode(&values);
        assert_eq!(ref_len, port_qm.len, "message_alloc_and_encode len for {} values", values.len());
        assert_bytes_eq(&ref_msg, &port_qm.msg, "message_alloc_and_encode msg");
        unsafe { message_free(ref_qm) };
    }
    // The overflow case must have logged "Encode error" on both capture channels.
    {
        let ref_last = c_logs().last().cloned().unwrap_or_default();
        let port_last = port_logs().last().cloned().unwrap_or_default();
        assert!(ref_last.contains("Encode error"), "C Encode error log: {ref_last}");
        assert!(port_last.contains("Encode error"), "port Encode error log: {port_last}");
    }
    c_logs().clear();
    port_logs().clear();
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

fn temp_file_path(tag: &str) -> String {
    let dir = std::env::temp_dir();
    format!("{}/{}_{}_{}.bin", dir.display(), std::process::id(), tag, time::monotonic().to_bits())
}

fn replay_debug_files_side(records: &[Record], is_ref: bool) -> Vec<Vec<u8>> {
    let side = if is_ref { "ref" } else { "port" };
    if is_ref {
        c_logs().clear();
    } else {
        port_logs().clear();
    }
    let mut sq_addrs: Vec<String> = Vec::new();
    let mut sq_files: Vec<std::fs::File> = Vec::new();
    let mut sq_paths: Vec<String> = Vec::new();
    let mut ref_sqs: Vec<*mut c_void> = Vec::new();
    let mut port_sqs: Vec<Option<SerialQueue>> = Vec::new();
    let mut cq_addrs: Vec<String> = Vec::new();
    let mut ref_cqs: Vec<*mut c_void> = Vec::new();
    let mut port_cqs: Vec<Option<std::sync::Arc<CommandQueue>>> = Vec::new();

    let find_sq = |sq_addrs: &[String], addr: &str| -> usize {
        sq_addrs.iter().position(|a| a == addr).expect("sq addr in recording")
    };
    let find_cq = |cq_addrs: &[String], addr: &str| -> usize {
        cq_addrs.iter().position(|a| a == addr).expect("cq addr in recording")
    };

    for rec in records {
        match rec.function.as_str() {
            "serialqueue_alloc" => {
                let fd_type = value_bytes(&rec.args[1])[0];
                let client_id = match &rec.args[2] {
                    Value::Int(i) => *i as i32,
                    _ => 0,
                };
                let name_bytes = value_bytes(&rec.args[3]);
                let path = temp_file_path(&format!("{side}-debugfile"));
                let file = std::fs::File::create(&path).expect("temp file");
                let addr = value_addr(rec.ret.as_ref().expect("alloc ret"));
                if is_ref {
                    let mut name16 = [0u8; 16];
                    let n = name_bytes.len().min(16);
                    name16[..n].copy_from_slice(&name_bytes[..n]);
                    name16[15] = 0;
                    let sq = unsafe {
                        serialqueue_alloc(file.as_raw_fd(), fd_type, client_id, name16.as_mut_ptr())
                    };
                    assert!(!sq.is_null(), "ref serialqueue_alloc returned null");
                    ref_sqs.push(sq);
                } else {
                    let sq = SerialQueue::alloc(file.as_raw_fd(), fd_type, client_id, &name_bytes)
                        .expect("port serialqueue_alloc");
                    port_sqs.push(Some(sq));
                }
                sq_addrs.push(addr);
                sq_files.push(file);
                sq_paths.push(path);
            }
            "serialqueue_alloc_commandqueue" => {
                let addr = value_addr(rec.ret.as_ref().expect("alloc_commandqueue ret"));
                if is_ref {
                    let cq = unsafe { serialqueue_alloc_commandqueue() };
                    assert!(!cq.is_null(), "ref alloc_commandqueue returned null");
                    ref_cqs.push(cq);
                } else {
                    port_cqs.push(Some(alloc_commandqueue()));
                }
                cq_addrs.push(addr);
            }
            "serialqueue_set_clock_est" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                let est_freq = value_f64(&rec.args[1]);
                let _recorded_conv_time = value_f64(&rec.args[2]);
                let conv_clock = value_u64(&rec.args[3]);
                // Re-base conv_time onto the current clock so both
                // sides share the same clock estimate (see the module
                // docs); frequency and conv_clock are verbatim.
                let base = time::monotonic();
                if is_ref {
                    unsafe { serialqueue_set_clock_est(ref_sqs[sq], est_freq, base, conv_clock) };
                } else {
                    port_sqs[sq].as_ref().expect("port sq").set_clock_est(est_freq, base, conv_clock);
                }
            }
            "serialqueue_set_wire_frequency" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                let freq = value_f64(&rec.args[1]);
                if is_ref {
                    unsafe { serialqueue_set_wire_frequency(ref_sqs[sq], freq) };
                } else {
                    port_sqs[sq].as_ref().expect("port sq").set_wire_frequency(freq);
                }
            }
            "serialqueue_set_receive_window" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                let window = match &rec.args[1] {
                    Value::Int(i) => *i as i32,
                    _ => 0,
                };
                if is_ref {
                    unsafe { serialqueue_set_receive_window(ref_sqs[sq], window) };
                } else {
                    port_sqs[sq].as_ref().expect("port sq").set_receive_window(window);
                }
            }
            "serialqueue_send" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                let cq = find_cq(&cq_addrs, &value_addr(&rec.args[1]));
                let msg = value_bytes(&rec.args[2]);
                let len = value_u64(&rec.args[3]) as i32;
                let min_clock = value_u64(&rec.args[4]);
                let req_clock = value_u64(&rec.args[5]);
                let notify_id = value_u64(&rec.args[6]);
                if is_ref {
                    let mut buf = msg.clone();
                    unsafe {
                        serialqueue_send(ref_sqs[sq], ref_cqs[cq], buf.as_mut_ptr(), len,
                                         min_clock, req_clock, notify_id)
                    };
                } else {
                    port_sqs[sq].as_ref().expect("port sq").send(
                        port_cqs[cq].as_ref().expect("port cq"),
                        &msg,
                        min_clock, req_clock, notify_id,
                    );
                }
            }
            "serialqueue_exit" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                if is_ref {
                    unsafe { serialqueue_exit(ref_sqs[sq]) };
                } else if let Some(s) = port_sqs[sq].as_ref() {
                    s.exit();
                }
            }
            "serialqueue_free" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                if is_ref {
                    unsafe { serialqueue_free(ref_sqs[sq]) };
                    ref_sqs[sq] = std::ptr::null_mut();
                } else if let Some(s) = port_sqs[sq].take() {
                    s.free();
                }
            }
            "serialqueue_free_commandqueue" => {
                let cq = find_cq(&cq_addrs, &value_addr(&rec.args[0]));
                if is_ref {
                    unsafe { serialqueue_free_commandqueue(ref_cqs[cq]) };
                    ref_cqs[cq] = std::ptr::null_mut();
                } else if let Some(cq_val) = port_cqs[cq].take() {
                    free_commandqueue(cq_val);
                }
            }
            "serialqueue_get_stats" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                let mut buf = vec![0u8; 4096];
                if is_ref {
                    unsafe { serialqueue_get_stats(ref_sqs[sq], buf.as_mut_ptr(), buf.len() as i32) };
                } else {
                    port_sqs[sq].as_ref().expect("port sq").get_stats(&mut buf);
                }
            }
            "serialqueue_pull" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                if is_ref {
                    let mut pqm = CPullQueueMessage {
                        msg: [0; MESSAGE_MAX],
                        len: 0,
                        sent_time: 0.0,
                        receive_time: 0.0,
                        notify_id: 0,
                    };
                    unsafe { serialqueue_pull(ref_sqs[sq], &mut pqm) };
                } else {
                    let _ = port_sqs[sq].as_ref().expect("port sq").pull();
                }
            }
            "serialqueue_extract_old" => {
                let sq = find_sq(&sq_addrs, &value_addr(&rec.args[0]));
                let sentq = matches!(&rec.args[1], Value::Int(1));
                if is_ref {
                    let mut arr = Vec::with_capacity(1024);
                    for _ in 0..1024 {
                        arr.push(CPullQueueMessage {
                            msg: [0; MESSAGE_MAX],
                            len: 0,
                            sent_time: 0.0,
                            receive_time: 0.0,
                            notify_id: 0,
                        });
                    }
                    unsafe { serialqueue_extract_old(ref_sqs[sq], sentq as i32, arr.as_mut_ptr(), 1024) };
                } else {
                    let _ = port_sqs[sq].as_ref().expect("port sq").extract_old(sentq, 1024);
                }
            }
            other => panic!("unsupported recorded function in replay: {other} (scenario {})", rec.scenario),
        }
    }

    // Command queues the recording allocated but never freed are
    // empty by now (sq free drains them), so freeing them is silent
    // on both sides; do it so the process does not accumulate cq
    // allocations. The byte stream is unaffected.
    for cq in ref_cqs.iter() {
        if !cq.is_null() {
            unsafe { serialqueue_free_commandqueue(*cq) };
        }
    }
    for cq in port_cqs.iter_mut() {
        if let Some(cq_val) = cq.take() {
            free_commandqueue(cq_val);
        }
    }

    drop(sq_files);
    sq_paths.iter().map(|p| mk_read_file(p)).collect()
}

fn mk_read_file(path: &str) -> Vec<u8> {
    let bytes = std::fs::read(path).unwrap_or_default();
    let _ = std::fs::remove_file(path);
    bytes
}

fn debug_file_replay() {
    install_log_capture();
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/fixtures/recordings");
    let scenarios = load_unit(&root, "serialqueue").expect("load serialqueue recordings");
    assert!(!scenarios.is_empty(), "no serialqueue recordings found under {root:?}");
    for (i, records) in scenarios.iter().enumerate() {
        let ref_bytes = replay_debug_files_side(records, true);
        let ref_logs: Vec<String> = c_logs().clone();
        let port_bytes = replay_debug_files_side(records, false);
        let port_logs: Vec<String> = port_logs().clone();
        assert_eq!(ref_bytes.len(), port_bytes.len(), "scenario {i}: debug file count");
        for (j, (r, p)) in ref_bytes.iter().zip(port_bytes.iter()).enumerate() {
            assert_bytes_eq(r, p, &format!("scenario {i} debug file {j} ({} records)", records.len()));
        }
        assert_vec_eq(&ref_logs, &port_logs, &format!("scenario {i} log lines"));
    }
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

fn peer_ack(next_seq: u8) -> Vec<u8> {
    peer_frame(next_seq, &[], false)
}

struct ScriptedPeer {
    rfd: i32,
    wfd: i32,
    rx: Vec<u8>,
    transcript: Vec<u8>,
    next_expected: u8,
}

impl ScriptedPeer {
    fn new(rfd: i32, wfd: i32) -> Self {
        ScriptedPeer { rfd, wfd, rx: Vec::new(), transcript: Vec::new(), next_expected: 2 }
    }

    fn peer_send(&mut self, bytes: &[u8]) {
        // The peer write end is nonblocking; scripted bursts are small.
        let mut off = 0;
        while off < bytes.len() {
            match fd::write(self.wfd, &bytes[off..]) {
                Ok(n) if n > 0 => off += n,
                Ok(_) => break,
                Err(_) => { time::sleep_until(time::monotonic() + 0.001); }
            }
        }
    }

    /// Poll briefly and read whatever arrived; parse complete
    /// blocks into the transcript. Returns block seq nibbles parsed.
    fn pump(&mut self) -> Vec<u8> {
        let mut pfds = [fd::PollFd::new(self.rfd, fd::POLLIN)];
        let _ = fd::poll(&mut pfds, 2);
        let mut chunk = [0u8; 4096];
        match fd::read(self.rfd, &mut chunk) {
            Ok(n) if n > 0 => self.rx.extend_from_slice(&chunk[..n]),
            _ => {}
        }
        let mut parsed = Vec::new();
        loop {
            if self.rx.first() == Some(&MESSAGE_SYNC) {
                self.rx.remove(0);
                continue;
            }
            if self.rx.is_empty() {
                break;
            }
            let len = self.rx[0] as usize;
            if len == 0 || self.rx.len() < len {
                break;
            }
            let block: Vec<u8> = self.rx.drain(..len).collect();
            self.transcript.extend_from_slice(&block);
            parsed.push(block[1] & 0x0f);
        }
        parsed
    }

    /// Pump until `n` blocks have been observed, or the step budget
    /// (2 ms polls) runs out. Returns the seq nibbles observed.
    fn wait_blocks(&mut self, n: usize) -> Vec<u8> {
        let mut got = Vec::new();
        for _ in 0..1000 {
            got.extend(self.pump());
            if got.len() >= n {
                return got;
            }
        }
        panic!("peer timed out waiting for {n} blocks; got {got:?}");
    }
}

#[derive(Clone, PartialEq, Debug)]
struct OutputMessage {
    len: i32,
    notify_id: u64,
    sent_off: u64,
    recv_off: u64,
    head: Vec<u8>,
}

fn out_msg(len: i32, notify_id: u64, sent: f64, recv: f64, msg: &[u8], base: f64) -> OutputMessage {
    OutputMessage {
        len,
        notify_id,
        sent_off: (sent - base).to_bits(),
        recv_off: (recv - base).to_bits(),
        head: msg[..len.max(0) as usize].to_vec(),
    }
}

fn split_dump(logs: &[String]) -> (Vec<String>, usize) {
    let start = logs
        .iter()
        .position(|l| l.contains("dump now="))
        .unwrap_or(logs.len());
    (logs[..start].to_vec(), logs.len() - start)
}

unsafe fn sq_send_ref(sq: *mut c_void, cq: *mut c_void, msg: &[u8], min: u64, req: u64, notify: u64) {
    let mut buf = msg.to_vec();
    serialqueue_send(sq, cq, buf.as_mut_ptr(), msg.len() as i32, min, req, notify);
}

unsafe fn sq_pull_ref(sq: *mut c_void, base: f64) -> OutputMessage {
    let mut pqm = CPullQueueMessage {
        msg: [0; MESSAGE_MAX],
        len: 0,
        sent_time: 0.0,
        receive_time: 0.0,
        notify_id: 0,
    };
    serialqueue_pull(sq, &mut pqm);
    assert_ne!(pqm.len, -1, "ref pull returned exit sentinel");
    out_msg(pqm.len, pqm.notify_id, pqm.sent_time, pqm.receive_time, &pqm.msg, base)
}

unsafe fn sq_stats_ref(sq: *mut c_void) -> String {
    let mut buf = vec![0u8; 512];
    serialqueue_get_stats(sq, buf.as_mut_ptr(), buf.len() as i32);
    let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..end]).into_owned()
}

unsafe fn sq_extract_ref(sq: *mut c_void, sentq: i32, base: f64) -> Vec<OutputMessage> {
    let mut arr = Vec::with_capacity(1024);
    for _ in 0..1024 {
        arr.push(CPullQueueMessage {
            msg: [0; MESSAGE_MAX],
            len: 0,
            sent_time: 0.0,
            receive_time: 0.0,
            notify_id: 0,
        });
    }
    let n = serialqueue_extract_old(sq, sentq, arr.as_mut_ptr(), 1024) as usize;
    arr[..n]
        .iter()
        .map(|p| out_msg(p.len, p.notify_id, p.sent_time, p.receive_time, &p.msg, base))
        .collect()
}

struct PipeResult {
    transcript: Vec<u8>,
    notify_pull: OutputMessage,
    data_pull: OutputMessage,
    stats: String,
    extract_sent: Vec<OutputMessage>,
    extract_recv: Vec<OutputMessage>,
    logs: Vec<String>,
    dump_line_count: usize,
}

fn run_pipe_scenario_ref() -> PipeResult {
    c_logs().clear();
    let (rfd, wfd) = fd::pipe().expect("peer pipe");
    fd::set_nonblock(rfd).expect("peer nonblock");
    fd::set_nonblock(wfd).expect("peer nonblock");
    let mut peer = ScriptedPeer::new(rfd, wfd);
    let mut name16 = *b"sqtest\0\0\0\0\0\0\0\0\0\0";
    let sq = unsafe { serialqueue_alloc(wfd, b'u', 0, name16.as_mut_ptr()) };
    assert!(!sq.is_null(), "ref pipe serialqueue_alloc");
    let cq = unsafe { serialqueue_alloc_commandqueue() };
    let base = time::monotonic();
    unsafe { serialqueue_set_wire_frequency(sq, 250_000.0) };
    unsafe { serialqueue_set_clock_est(sq, 1e12, base, 0) };
    let _ = &mut name16;

    // Phase A: one message, ack next (0x12).
    unsafe { sq_send_ref(sq, cq, &[1, 2, 3], 0, 0, 0) };
    assert_eq!(peer.wait_blocks(1), vec![1]);
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase B: two small messages, one block (seq 2), ack (0x13).
    unsafe { sq_send_ref(sq, cq, &[4, 5], 0, 0, 0) };
    unsafe { sq_send_ref(sq, cq, &[6, 7, 8, 9], 0, 0, 0) };
    assert_eq!(peer.wait_blocks(1), vec![2]);
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase C: one message, duplicate ack -> fast retransmit,
    // peer acks the retransmit (0x14).
    unsafe { sq_send_ref(sq, cq, &[10, 11, 12, 13], 0, 0, 0) };
    assert_eq!(peer.wait_blocks(1), vec![3]);
    let dup = peer_ack(peer.next_expected);
    peer.peer_send(&dup);
    peer.peer_send(&dup);
    assert_eq!(peer.wait_blocks(1), vec![3], "retransmit of seq 3");
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase D: one message, no ack -> timeout retransmit,
    // peer acks the retransmit (0x15).
    unsafe { sq_send_ref(sq, cq, &[20, 21], 0, 0, 0) };
    assert_eq!(peer.wait_blocks(1), vec![4]);
    assert_eq!(peer.wait_blocks(1), vec![4], "timeout retransmit of seq 4");
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase E: notify message, ack (0x16), pull -> notification.
    unsafe { sq_send_ref(sq, cq, &[30], 0, 0, 7) };
    assert_eq!(peer.wait_blocks(1), vec![5]);
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;
    let notify_pull = unsafe { sq_pull_ref(sq, base) };
    assert_eq!(notify_pull.len, 0, "notify pull len");
    assert_eq!(notify_pull.notify_id, 7, "notify pull id");

    // Phase F: peer data frame -> pull.
    peer.peer_send(&peer_frame(0x16, &[0x41, 0x42, 0x43], false));
    let data_pull = unsafe { sq_pull_ref(sq, base) };
    assert_eq!(data_pull.head, vec![0x41, 0x42, 0x43], "data payload");

    // Phase G: bad-CRC frame (dropped; counted invalid).
    peer.peer_send(&peer_frame(0x16, &[0x44, 0x45], true));
    // Give the bg thread a beat to consume it before stats.
    for _ in 0..50 {
        peer.pump();
        time::sleep_until(time::monotonic() + 0.002);
    }

    let stats = unsafe { sq_stats_ref(sq) };
    let extract_sent = unsafe { sq_extract_ref(sq, 1, base) };
    let extract_recv = unsafe { sq_extract_ref(sq, 0, base) };

    unsafe { serialqueue_exit(sq) };
    unsafe { serialqueue_free(sq) };
    unsafe { serialqueue_free_commandqueue(cq) };
    fd::close(wfd);
    fd::close(rfd);

    let (logs, dump_line_count) = split_dump(&c_logs().clone());
    PipeResult {
        transcript: peer.transcript,
        notify_pull,
        data_pull,
        stats,
        extract_sent,
        extract_recv,
        logs,
        dump_line_count,
    }
}

fn sq_pull_port(sq: &SerialQueue, base: f64) -> OutputMessage {
    let pqm: PullQueueMessage = sq.pull().expect("port pull returned None before exit");
    out_msg(pqm.len, pqm.notify_id, pqm.sent_time, pqm.receive_time, &pqm.msg, base)
}

fn run_pipe_scenario_port() -> PipeResult {
    port_logs().clear();
    let (rfd, wfd) = fd::pipe().expect("peer pipe");
    fd::set_nonblock(rfd).expect("peer nonblock");
    fd::set_nonblock(wfd).expect("peer nonblock");
    let mut peer = ScriptedPeer::new(rfd, wfd);
    let sq = SerialQueue::alloc(wfd, b'u', 0, b"sqtest").expect("port pipe serialqueue_alloc");
    let cq = alloc_commandqueue();
    let base = time::monotonic();
    sq.set_wire_frequency(250_000.0);
    sq.set_clock_est(1e12, base, 0);

    // Phase A
    sq.send(&cq, &[1, 2, 3], 0, 0, 0);
    assert_eq!(peer.wait_blocks(1), vec![1]);
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase B
    sq.send(&cq, &[4, 5], 0, 0, 0);
    sq.send(&cq, &[6, 7, 8, 9], 0, 0, 0);
    assert_eq!(peer.wait_blocks(1), vec![2]);
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase C
    sq.send(&cq, &[10, 11, 12, 13], 0, 0, 0);
    assert_eq!(peer.wait_blocks(1), vec![3]);
    let dup = peer_ack(peer.next_expected);
    peer.peer_send(&dup);
    peer.peer_send(&dup);
    assert_eq!(peer.wait_blocks(1), vec![3], "retransmit of seq 3");
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase D
    sq.send(&cq, &[20, 21], 0, 0, 0);
    assert_eq!(peer.wait_blocks(1), vec![4]);
    assert_eq!(peer.wait_blocks(1), vec![4], "timeout retransmit of seq 4");
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;

    // Phase E
    sq.send(&cq, &[30], 0, 0, 7);
    assert_eq!(peer.wait_blocks(1), vec![5]);
    peer.peer_send(&peer_ack(peer.next_expected));
    peer.next_expected += 1;
    let notify_pull = sq_pull_port(&sq, base);
    assert_eq!(notify_pull.len, 0, "notify pull len");
    assert_eq!(notify_pull.notify_id, 7, "notify pull id");

    // Phase F
    peer.peer_send(&peer_frame(0x16, &[0x41, 0x42, 0x43], false));
    let data_pull = sq_pull_port(&sq, base);
    assert_eq!(data_pull.head, vec![0x41, 0x42, 0x43], "data payload");

    // Phase G
    peer.peer_send(&peer_frame(0x16, &[0x44, 0x45], true));
    for _ in 0..50 {
        peer.pump();
        time::sleep_until(time::monotonic() + 0.002);
    }

    let mut stat_buf = vec![0u8; 512];
    sq.get_stats(&mut stat_buf);
    let end = stat_buf.iter().position(|&b| b == 0).unwrap_or(stat_buf.len());
    let stats = String::from_utf8_lossy(&stat_buf[..end]).into_owned();
    let extract_sent: Vec<OutputMessage> = sq
        .extract_old(true, 1024)
        .iter()
        .map(|p| out_msg(p.len, p.notify_id, p.sent_time, p.receive_time, &p.msg, base))
        .collect();
    let extract_recv: Vec<OutputMessage> = sq
        .extract_old(false, 1024)
        .iter()
        .map(|p| out_msg(p.len, p.notify_id, p.sent_time, p.receive_time, &p.msg, base))
        .collect();

    sq.exit();
    sq.free();
    free_commandqueue(cq);
    fd::close(wfd);
    fd::close(rfd);

    let (logs, dump_line_count) = split_dump(&port_logs().clone());
    PipeResult {
        transcript: peer.transcript,
        notify_pull,
        data_pull,
        stats,
        extract_sent,
        extract_recv,
        logs,
        dump_line_count,
    }
}

fn pipe_scenario() {
    let ref_res = run_pipe_scenario_ref();
    let port_res = run_pipe_scenario_port();

    assert_bytes_eq(&ref_res.transcript, &port_res.transcript, "pipe transcript");
    assert_eq!(ref_res.notify_pull, port_res.notify_pull, "notify pull (normalized)");
    assert_eq!(ref_res.data_pull, port_res.data_pull, "data pull (normalized)");
    assert_eq!(ref_res.stats, port_res.stats, "stats string");
    assert_vec_eq(&ref_res.extract_sent, &port_res.extract_sent, "extract_old(sent)");
    assert_vec_eq(&ref_res.extract_recv, &port_res.extract_recv, "extract_old(receive)");
    assert_vec_eq(&ref_res.logs, &port_res.logs, "pipe log lines (pre-dump)");
    assert_eq!(ref_res.dump_line_count, port_res.dump_line_count, "dump line count");
}

#[test]
fn serialqueue_parity() {
    chelper_ref::link();
    install_log_capture();
    msgblock_units();
    debug_file_replay();
    pipe_scenario();
}
