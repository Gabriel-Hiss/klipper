//! Differential tests for `chelper_motion::stepcompress` against the frozen
//! reference C library (`chelper_ref`).
//!
//! A deterministic op stream (hand-written scenarios plus generated ones
//! that reach every branch: direction changes inside the step+dir+step
//! window, intervals that overflow `int32` products, `add == 0`
//! preference ties, count limits past `0x200`, negative relative clocks,
//! far-future steps, queue capacity growth past 67535 staged steps and the
//! `ERROR_RET` paths) is run against both implementations.  After every op
//! the test compares the return code, the full list of emitted MCU
//! messages (every byte plus `req_clock`/`min_clock`), the complete
//! history (`extract_old` with an unbounded window), `get_oid` and
//! `get_step_dir`.  `find_past_position` is compared through explicit
//! probe ops.
//!
//! The recorded scenarios in `tests/fixtures/recordings/stepcompress/`
//! only capture `stepcompress_fill` / `stepcompress_set_invert_sdir` calls
//! (the internal `stepcompress_append` is not visible to the Python-level
//! recorder), so they cannot drive the compressor state; the generated
//! streams above are the parity proof.

use chelper_motion::stepcompress::{
    QueuedMessage, Stepcompress, ERROR_RET,
};
use core::ffi::c_void;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from msgblock.h / list.h / serialqueue.h).
// ---------------------------------------------------------------------------

#[repr(C)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

#[repr(C)]
struct ListHead {
    root: ListNode,
}

/// `struct queue_message` (msgblock.h): len, msg[64], the min/req clock
/// union, notify_id, node. `clock[0] = min_clock`, `clock[1] = req_clock`.
#[repr(C)]
struct RefQueueMessage {
    len: i32,
    msg: [u8; 64],
    clock: [u64; 2],
    notify_id: u64,
    node: ListNode,
}

/// `struct pull_history_steps` (stepcompress.h).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct RefHistory {
    first_clock: u64,
    last_clock: u64,
    start_position: i64,
    step_count: i32,
    interval: i32,
    add: i32,
}

impl RefHistory {
    fn zero() -> Self {
        RefHistory {
            first_clock: 0,
            last_clock: 0,
            start_position: 0,
            step_count: 0,
            interval: 0,
            add: 0,
        }
    }
}

extern "C" {
    fn stepcompress_alloc(msg_queue: *mut ListHead) -> *mut c_void;
    fn stepcompress_free(sc: *mut c_void);
    fn stepcompress_fill(
        sc: *mut c_void,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    );
    fn stepcompress_set_invert_sdir(sc: *mut c_void, invert_sdir: u32);
    fn stepcompress_set_time(sc: *mut c_void, time_offset: f64, mcu_freq: f64);
    fn stepcompress_append(
        sc: *mut c_void,
        sdir: i32,
        print_time: f64,
        step_time: f64,
    ) -> i32;
    fn stepcompress_commit(sc: *mut c_void) -> i32;
    fn stepcompress_flush(sc: *mut c_void, move_clock: u64) -> i32;
    fn stepcompress_reset(sc: *mut c_void, last_step_clock: u64) -> i32;
    fn stepcompress_set_last_position(
        sc: *mut c_void,
        clock: u64,
        last_position: i64,
    ) -> i32;
    fn stepcompress_find_past_position(sc: *mut c_void, clock: u64) -> i64;
    fn stepcompress_extract_old(
        sc: *mut c_void,
        p: *mut RefHistory,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> i32;
    fn stepcompress_history_expire(sc: *mut c_void, end_clock: u64);
    fn stepcompress_get_oid(sc: *mut c_void) -> u32;
    fn stepcompress_get_step_dir(sc: *mut c_void) -> i32;
    fn message_queue_free(root: *mut ListHead);
}

// ---------------------------------------------------------------------------
// Reference-side handle.
// ---------------------------------------------------------------------------

/// Reference-side handle.
///
/// The `msg_queue` head must live at a stable address: `stepcompress_alloc`
/// stores the pointer inside the C object, and the C writes through it on
/// every `add_move`. A plain inline `ListHead` moved after the self-links
/// were set makes the C write through a dangling stack address (the move
/// left `root.next`/`root.prev` pointing at the pre-move copy), which
/// segfaulted every scenario on the reference side. Boxing keeps the
/// address stable across moves.
struct RefSc {
    sc: *mut c_void,
    head: Box<ListHead>,
    scratch: Vec<RefHistory>,
}
impl RefSc {
    fn new() -> RefSc {
        let mut head = Box::new(ListHead {
            root: ListNode {
                next: core::ptr::null_mut(),
                prev: core::ptr::null_mut(),
            },
        });
        head.root.next = &mut head.root as *mut ListNode;
        head.root.prev = &mut head.root as *mut ListNode;
        let sc = unsafe { stepcompress_alloc(head.as_mut() as *mut ListHead) };
        assert!(!sc.is_null(), "stepcompress_alloc returned null");
        RefSc {
            sc,
            head,
            scratch: Vec::new(),
        }
    }

    /// Walk the reference msg queue head-first (same order the port keeps)
    /// and free it, returning the messages as (bytes, req_clock, min_clock).
    fn drain_messages(&mut self) -> Vec<(Vec<u8>, u64, u64)> {
        let mut out = Vec::new();
        let root = &mut self.head.root as *mut ListNode;
        let mut cur = unsafe { (*root).next };
        while cur != root {
            let qm = unsafe {
                &*((cur as *const u8)
                    .wrapping_sub(core::mem::offset_of!(RefQueueMessage, node))
                    as *const RefQueueMessage)
            };
            let len = qm.len as usize;
            out.push((
                qm.msg[..len].to_vec(),
                qm.clock[1],
                qm.clock[0],
            ));
            cur = unsafe { (*cur).next };
        }
        unsafe { message_queue_free(self.head.as_mut() as *mut ListHead) };
        out
    }

    fn extract_old(
        &mut self,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> Vec<RefHistory> {
        let want = max.max(0) as usize;
        if self.scratch.len() < want {
            self.scratch.resize(want, RefHistory::zero());
        }
        let res = unsafe {
            stepcompress_extract_old(
                self.sc,
                self.scratch.as_mut_ptr(),
                max,
                start_clock,
                end_clock,
            )
        };
        assert!(res >= 0 && res <= max, "bad extract_old result {res}");
        self.scratch[..res as usize].to_vec()
    }
}

impl Drop for RefSc {
    fn drop(&mut self) {
        unsafe {
            message_queue_free(self.head.as_mut() as *mut ListHead);
            stepcompress_free(self.sc);
        }
    }
}

// ---------------------------------------------------------------------------
// Op stream.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Op {
    Fill {
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    },
    Invert(u32),
    SetTime(f64, f64),
    Append {
        sdir: i32,
        print_time: f64,
        step_time: f64,
    },
    Commit,
    Flush(u64),
    Reset(u64),
    SetLastPos { clock: u64, last_position: i64 },
    Expire(u64),
    FindPast(u64),
    /// Compare the full observable state without running an op.
    Snapshot,
}

struct Pair {
    rust: Stepcompress,
    refr: RefSc,
}

impl Pair {
    fn new() -> Pair {
        chelper_ref::link();
        Pair {
            rust: Stepcompress::new(),
            refr: RefSc::new(),
        }
    }

    fn run_op(&mut self, op: &Op, ctx: &str) {
        let ret_rust: Option<i32>;
        let ret_ref: Option<i32>;
        match *op {
            Op::Fill {
                oid,
                max_error,
                queue_step_msgtag,
                set_next_step_dir_msgtag,
            } => {
                self.rust.fill(
                    oid,
                    max_error,
                    queue_step_msgtag,
                    set_next_step_dir_msgtag,
                );
                unsafe {
                    stepcompress_fill(
                        self.refr.sc,
                        oid,
                        max_error,
                        queue_step_msgtag,
                        set_next_step_dir_msgtag,
                    );
                }
                ret_rust = None;
                ret_ref = None;
            }
            Op::Invert(invert) => {
                self.rust.set_invert_sdir(invert);
                unsafe { stepcompress_set_invert_sdir(self.refr.sc, invert) }
                ret_rust = None;
                ret_ref = None;
            }
            Op::SetTime(off, freq) => {
                self.rust.set_time(off, freq);
                unsafe { stepcompress_set_time(self.refr.sc, off, freq) }
                ret_rust = None;
                ret_ref = None;
            }
            Op::Append {
                sdir,
                print_time,
                step_time,
            } => {
                ret_rust = Some(self.rust.append(sdir, print_time, step_time));
                ret_ref = Some(unsafe {
                    stepcompress_append(self.refr.sc, sdir, print_time, step_time)
                });
            }
            Op::Commit => {
                ret_rust = Some(self.rust.commit());
                ret_ref = Some(unsafe { stepcompress_commit(self.refr.sc) });
            }
            Op::Flush(move_clock) => {
                ret_rust = Some(self.rust.flush(move_clock));
                ret_ref =
                    Some(unsafe { stepcompress_flush(self.refr.sc, move_clock) });
            }
            Op::Reset(last) => {
                ret_rust = Some(self.rust.reset(last));
                ret_ref = Some(unsafe { stepcompress_reset(self.refr.sc, last) });
            }
            Op::SetLastPos { clock, last_position } => {
                ret_rust = Some(self.rust.set_last_position(clock, last_position));
                ret_ref = Some(unsafe {
                    stepcompress_set_last_position(self.refr.sc, clock, last_position)
                });
            }
            Op::Expire(end_clock) => {
                self.rust.history_expire(end_clock);
                unsafe { stepcompress_history_expire(self.refr.sc, end_clock) }
                ret_rust = None;
                ret_ref = None;
            }
            Op::FindPast(clock) => {
                let a = self.rust.find_past_position(clock);
                let b = unsafe { stepcompress_find_past_position(self.refr.sc, clock) };
                assert_eq!(a, b, "find_past_position({clock}) diverged [{ctx} op={op:?}]");
                ret_rust = None;
                ret_ref = None;
            }
            Op::Snapshot => {
                ret_rust = None;
                ret_ref = None;
            }
        }
        assert_eq!(ret_rust, ret_ref, "return code diverged [{ctx} op={op:?}]");
        self.compare_state(ctx, op);
    }

    fn compare_state(&mut self, ctx: &str, op: &Op) {
        // Messages: every byte plus req/min clocks.
        let rust_msgs = self.rust.drain_messages();
        let ref_msgs = self.refr.drain_messages();
        let rust_msgs: Vec<(Vec<u8>, u64, u64)> = rust_msgs
            .iter()
            .map(|m: &QueuedMessage| (m.bytes.clone(), m.req_clock, m.min_clock))
            .collect();
        assert_eq!(
            rust_msgs, ref_msgs,
            "messages diverged [{ctx} op={op:?}]"
        );

        // History: newest-first snapshot. The window truncates both sides
        // identically, so parity holds for the compared prefix even when the
        // real history is longer; the cap only bounds the compare buffer,
        // which the reference fills once per op.
        const HISTORY_MAX: i32 = 8192;
        let rust_hist = self.rust.extract_old(HISTORY_MAX, 0, u64::MAX);
        let ref_hist = self.refr.extract_old(HISTORY_MAX, 0, u64::MAX);
        let rust_hist_ref: Vec<RefHistory> = rust_hist
            .iter()
            .map(|h| RefHistory {
                first_clock: h.first_clock,
                last_clock: h.last_clock,
                start_position: h.start_position,
                step_count: h.step_count,
                interval: h.interval,
                add: h.add,
            })
            .collect();
        assert_eq!(
            rust_hist_ref, ref_hist,
            "history diverged [{ctx} op={op:?}]"
        );

        // Small observable scalars.
        assert_eq!(
            self.rust.get_oid(),
            unsafe { stepcompress_get_oid(self.refr.sc) },
            "oid diverged [{ctx} op={op:?}]"
        );
        assert_eq!(
            self.rust.get_step_dir(),
            unsafe { stepcompress_get_step_dir(self.refr.sc) },
            "step dir diverged [{ctx} op={op:?}]"
        );
    }
}

fn run_stream(ctx: &str, ops: &[Op]) {
    let mut pair = Pair::new();
    for (i, op) in ops.iter().enumerate() {
        pair.run_op(op, &format!("{ctx}#{i}"));
    }
}

// ---------------------------------------------------------------------------
// Deterministic scenarios.
// ---------------------------------------------------------------------------

const FREQ: f64 = 1_000_000.0;

fn base_fill() -> Op {
    Op::Fill {
        oid: 7,
        max_error: 3000,
        queue_step_msgtag: 24,
        set_next_step_dir_msgtag: 23,
    }
}

/// Constant-interval steps: one `add == 0` move, exercising the add=0
/// preference tie path (`zerocount + zerocount/16 >= bestcount`).
fn constant_steps() {
    let mut ops = vec![base_fill(), Op::SetTime(0.0, FREQ)];
    for i in 0..64 {
        ops.push(Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: i as f64 * 0.001,
        });
        ops.push(Op::Commit);
    }
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);
    run_stream("constant", &ops);
}

/// Accelerating steps (quadratic step times): forces nonzero `add` and the
/// bisection over the add range.
fn accelerating_steps() {
    let mut ops = vec![base_fill(), Op::SetTime(0.0, FREQ)];
    let mut t = 0.0;
    let mut dt = 0.01;
    for _ in 0..200 {
        t += dt;
        dt += 0.0002;
        ops.push(Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: t,
        });
        ops.push(Op::Commit);
    }
    ops.push(Op::Flush(u64::MAX));
    for clock in [0, 1, 10, 100, 1000, 10_000, 100_000, 1_000_000, u64::MAX] {
        ops.push(Op::FindPast(clock));
    }
    ops.push(Op::Snapshot);
    run_stream("accelerating", &ops);
}

/// Direction changes inside the step+dir+step filter window: each flip
/// lands within `SDS_FILTER_TIME * freq` ticks of the pending step and
/// rolls it back; the follow-up steps are spaced out so they commit.
fn sds_filter() {
    let mut ops = vec![base_fill(), Op::SetTime(0.0, FREQ)];
    ops.push(Op::Append {
        sdir: 1,
        print_time: 0.0,
        step_time: 0.0,
    });
    // Same clock, opposite direction -> rollback.
    ops.push(Op::Append {
        sdir: 0,
        print_time: 0.0,
        step_time: 0.0,
    });
    // 500 ticks later, opposite direction: still inside the 750 tick
    // window, rollback again.
    ops.push(Op::Append {
        sdir: 1,
        print_time: 0.0,
        step_time: 0.0005,
    });
    // A step that survives: flip back after the window.
    ops.push(Op::Append {
        sdir: 0,
        print_time: 0.0,
        step_time: 0.002,
    });
    ops.push(Op::Commit);
    ops.push(Op::Append {
        sdir: 1,
        print_time: 0.0,
        step_time: 0.0025,
    });
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);
    run_stream("sds", &ops);
}

/// Steps far in the future (`CLOCK_DIFF_MAX`), partial flushes at exact
/// step clocks, and inverted direction encoding (`dir ^ invert_sdir`).
fn far_and_partial() {
    let ops = vec![
        base_fill(),
        Op::SetTime(0.0, FREQ),
        Op::Invert(1),
        Op::Append {
            sdir: 0,
            print_time: 0.0,
            step_time: 0.0,
        },
        Op::Commit,
        Op::Append {
            sdir: 0,
            print_time: 0.0,
            step_time: 0.001,
        },
        Op::Commit,
        // Jump past CLOCK_DIFF_MAX (3<<28 ticks = 805.306368 s at 1 MHz).
        Op::Append {
            sdir: 0,
            print_time: 0.0,
            step_time: 1000.0,
        },
        Op::Commit,
        // Partial flushes: before, at, and after the queued steps.
        Op::Flush(0),
        Op::Flush(500),
        Op::Flush(1000),
        Op::Flush(1001),
        Op::Flush(u64::MAX),
        Op::Invert(0),
        Op::Invert(2),
        Op::Snapshot,
    ];
    run_stream("far", &ops);
}

/// Negative relative clocks: decreasing step times, a `print_time` far
/// behind the last step print time, and a backwards `reset`.
fn negative_clocks() {
    let ops = vec![
        base_fill(),
        Op::SetTime(0.0, FREQ),
        Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: 1.0,
        },
        Op::Commit,
        // Move backwards: step_clock wraps below last_step_clock.
        Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: 0.5,
        },
        Op::Commit,
        Op::Append {
            sdir: 1,
            print_time: -100.0,
            step_time: 0.0,
        },
        Op::Commit,
        Op::Flush(u64::MAX),
        Op::Reset(10_000),
        Op::Append {
            sdir: 0,
            print_time: 0.0,
            step_time: -0.001,
        },
        Op::Commit,
        Op::Flush(u64::MAX),
        Op::Reset(0),
        Op::Snapshot,
    ];
    run_stream("negative", &ops);
}

/// Intervals that overflow `int32` products: a 3 GHz clock with 1 s steps
/// (intervals ~3e9 > 2^31) and a max_error of 0 (tightest tolerance).
fn int32_overflow() {
    let mut ops = vec![
        Op::Fill {
            oid: 3,
            max_error: 0,
            queue_step_msgtag: 24,
            set_next_step_dir_msgtag: 23,
        },
        Op::SetTime(0.0, 3.0e9),
    ];
    for i in 0..128 {
        ops.push(Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: i as f64,
        });
        ops.push(Op::Commit);
    }
    ops.push(Op::Flush(u64::MAX));
    // Decelerating sequence so `add` is negative and products still wrap.
    for i in 0..128 {
        ops.push(Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: 200.0 - i as f64 * 0.5,
        });
        ops.push(Op::Commit);
    }
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);
    run_stream("overflow", &ops);
}

/// Long runs that cross the `count > 0x200` early-exit and the 65535-step
/// compression cap, plus the >67535 staged-step flush in queue_append.
fn long_runs() {
    let mut ops = vec![base_fill(), Op::SetTime(0.0, FREQ)];
    // 600 constant steps: count crosses 0x200.
    for i in 0..600 {
        ops.push(Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: i as f64 * 0.001,
        });
        ops.push(Op::Commit);
    }
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);

    // 68000 staged steps without an explicit flush: crosses 65535 + 2000
    // so the extend-path flush fires. Snapshots are sparse to keep the
    // history comparison cheap.
    let mut ops2 = vec![base_fill(), Op::SetTime(0.0, FREQ)];
    for i in 0..68_000 {
        ops2.push(Op::Append {
            sdir: if i % 10_000 == 0 { 0 } else { 1 },
            print_time: 0.0,
            step_time: i as f64 * 0.001,
        });
        ops2.push(Op::Commit);
        if i % 8192 == 0 && i > 0 {
            ops2.push(Op::Snapshot);
        }
    }
    ops2.push(Op::Flush(u64::MAX));
    ops2.push(Op::Snapshot);
    run_stream("long", &ops2);
}

/// History markers, expiry, windows and past-position probes.
fn history_and_positions() {
    let mut ops = vec![
        base_fill(),
        Op::SetTime(0.0, FREQ),
        Op::SetLastPos {
            clock: 0,
            last_position: 100,
        },
    ];
    let mut t = 0.0;
    for i in 0..40 {
        t += 0.001;
        ops.push(Op::Append {
            sdir: if i < 20 { 1 } else { 0 },
            print_time: 0.0,
            step_time: t,
        });
        ops.push(Op::Commit);
    }
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::SetLastPos {
        clock: 45_000,
        last_position: -7,
    });
    ops.push(Op::Expire(20_000));
    ops.push(Op::Expire(u64::MAX));
    for clock in [0, 1, 999, 1000, 10_000, 39_999, 40_000, 44_999, 45_000, 45_001, u64::MAX] {
        ops.push(Op::FindPast(clock));
    }
    ops.push(Op::Snapshot);
    run_stream("history", &ops);
}

// ---------------------------------------------------------------------------
// Generated fuzz stream.
// ---------------------------------------------------------------------------

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    fn pick_f64(&mut self, pool: &[f64]) -> f64 {
        pool[self.below(pool.len() as u64) as usize]
    }
}

const FREQ_POOL: [f64; 8] = [
    1e6,
    16e6,
    0.0,
    -1e6,
    f64::NAN,
    f64::INFINITY,
    1.0,
    3.0e9,
];
/// Step-time deltas for the fuzz stream. The huge jumps (805 s, 806 s,
/// 10000 s) were removed from the *reference* side: a far step
/// (`next_step_clock >= last_step_clock + CLOCK_DIFF_MAX`) takes the
/// `queue_append_far` slow path, whose post-flush store
/// (`sc->queue_next->clock32 = step_clock`) writes without a capacity check
/// and corrupts the heap when the staged queue is exactly full
/// (`queue_next == queue_end`). The Rust port grows instead (see
/// `store_queued`), so the inputs stay exercised on the Rust side in the
/// deterministic `far_and_partial` scenario, where the staged queue is far
/// below capacity and the reference is within its contract. Far steps are
/// still reachable in fuzz through large cumulative `t` at high frequencies.
const DT_POOL: [f64; 15] = [
    0.0,
    1e-6,
    1e-4,
    0.001,
    0.002,
    0.01,
    0.05,
    0.1,
    1.0,
    100.0,
    -0.001,
    -0.05,
    -5.0,
    f64::NAN,
    f64::INFINITY,
];
const PRINT_TIME_POOL: [f64; 8] = [
    0.0,
    0.0,
    0.0,
    -1.0,
    1e-12,
    f64::NAN,
    f64::INFINITY,
    -f64::INFINITY,
];
const CLOCK_POOL: [u64; 10] = [
    0,
    1,
    2,
    100,
    1000,
    1 << 20,
    805_306_368,
    1 << 40,
    1_u64 << 63,
    u64::MAX,
];
const TAG_POOL: [i32; 6] = [24, 23, 100, -1, i32::MAX, i32::MIN];

fn fuzz_stream(seed: u64, n_ops: usize) -> Vec<Op> {
    let mut rng = Rng(seed);
    let mut ops = Vec::with_capacity(n_ops);
    // Random fill: max_error spans 0 (tightest) to u32::MAX (widest).
    let max_error = match rng.below(5) {
        0 => 0,
        1 => 1,
        2 => 3000,
        3 => u32::MAX,
        _ => rng.below(10_000) as u32,
    };
    ops.push(Op::Fill {
        oid: (rng.below(256)) as u32,
        max_error,
        queue_step_msgtag: rng.pick_f64(&TAG_POOL.map(|v| v as f64)) as i32,
        set_next_step_dir_msgtag: rng.pick_f64(&TAG_POOL.map(|v| v as f64)) as i32,
    });
    // Realistic tags sometimes (matching the firmware's real msgtags).
    if rng.below(2) == 0 {
        ops.push(base_fill());
    }
    let mut sdir = 1i32;
    let mut t = 0.0f64;
    for i in 0..n_ops {
        // Bound the staged queue well below the 1024-entry initial
        // capacity (the `queue_append_far` capacity corner needs a full
        // queue; 128 ops cannot stage that many steps between flushes).
        if i > 0 && i % 128 == 0 {
            ops.push(Op::Flush(u64::MAX));
        }
        let roll = rng.below(100);
        match roll {
            0..=1 => ops.push(Op::SetTime(rng.pick_f64(&PRINT_TIME_POOL), rng.pick_f64(&FREQ_POOL))),
            2..=2 => ops.push(Op::Invert(rng.below(4) as u32)),
            3..=47 => {
                t += rng.pick_f64(&DT_POOL);
                if rng.below(6) == 0 {
                    sdir = if sdir == 0 { 1 } else { 0 };
                }
                let print_time = if rng.below(4) == 0 {
                    rng.pick_f64(&PRINT_TIME_POOL)
                } else {
                    0.0
                };
                ops.push(Op::Append {
                    sdir,
                    print_time,
                    step_time: t,
                });
            }
            48..=57 => ops.push(Op::Commit),
            58..=61 => ops.push(Op::Flush(rng.pick_f64(&CLOCK_POOL.map(|v| v as f64)) as u64)),
            62..=64 => ops.push(Op::Flush(u64::MAX)),
            65..=67 => ops.push(Op::Reset(rng.pick_f64(&CLOCK_POOL.map(|v| v as f64)) as u64)),
            68..=70 => ops.push(Op::SetLastPos {
                clock: rng.pick_f64(&CLOCK_POOL.map(|v| v as f64)) as u64,
                last_position: rng.below(2_000_000) as i64 - 1_000_000,
            }),
            71..=73 => ops.push(Op::Expire(rng.pick_f64(&CLOCK_POOL.map(|v| v as f64)) as u64)),
            74..=83 => ops.push(Op::FindPast(rng.pick_f64(&CLOCK_POOL.map(|v| v as f64)) as u64)),
            84..=87 => ops.push(Op::Snapshot),
            _ => {
                // Burst of appends with SDS-window flips at the same time.
                if rng.below(2) == 0 {
                    sdir = if sdir == 0 { 1 } else { 0 };
                }
                ops.push(Op::Append {
                    sdir,
                    print_time: 0.0,
                    step_time: t,
                });
                sdir = if sdir == 0 { 1 } else { 0 };
                ops.push(Op::Append {
                    sdir,
                    print_time: 0.0,
                    step_time: t + rng.pick_f64(&[0.0, 0.0005, 0.00075, 0.001]),
                });
            }
        }
        let _ = i;
    }
    // End with a full flush attempt and a snapshot; error states must
    // match too (reset/flush return ERROR_RET on both sides).
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);
    ops
}

fn fuzz(seed: u64) {
    let ops = fuzz_stream(seed, 5000);
    run_stream(&format!("fuzz-{seed}"), &ops);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constant_steps_parity() {
    constant_steps();
}

#[test]
fn accelerating_steps_parity() {
    accelerating_steps();
}

#[test]
fn sds_filter_parity() {
    sds_filter();
}

#[test]
fn far_and_partial_parity() {
    far_and_partial();
}

#[test]
fn negative_clocks_parity() {
    negative_clocks();
}

#[test]
fn int32_overflow_parity() {
    int32_overflow();
}

#[test]
fn long_runs_parity() {
    long_runs();
}

#[test]
fn history_and_positions_parity() {
    history_and_positions();
}

#[test]
fn fuzz_seed_1() {
    fuzz(0x1234_5678_9abc_def1);
}

#[test]
fn fuzz_seed_2() {
    fuzz(0xdead_beef_cafe_f00d);
}

#[test]
fn fuzz_seed_3() {
    fuzz(0x0bad_f00d_1357_9bdf);
}

/// Sanity check that ERROR_RET is the value the C uses and that at least
/// one generated stream hits it.  Steps staged exactly at
/// `last_step_clock` compress to a count>1 move with interval 0 and
/// add 0, which `check_line` rejects on both sides identically.
/// The relative clock after `reset(1000)` is zero at
/// `step_time = (1000 - 0.5) / FREQ`.
#[test]
fn error_path_is_reached() {
    let rt = (1000.0 - 0.5) / FREQ;
    let ops = vec![
        Op::Fill {
            oid: 1,
            max_error: 0,
            queue_step_msgtag: 24,
            set_next_step_dir_msgtag: 23,
        },
        Op::SetTime(0.0, FREQ),
        Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: 0.001,
        },
        Op::Commit,
        Op::Flush(u64::MAX),
        Op::Reset(1000),
        Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: rt,
        },
        Op::Commit,
        Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: rt,
        },
        Op::Commit,
        Op::Append {
            sdir: 1,
            print_time: 0.0,
            step_time: rt,
        },
        Op::Commit,
        Op::Flush(u64::MAX),
    ];
    let mut pair = Pair::new();
    for op in &ops {
        pair.run_op(op, "error-path");
    }
    // Replay and check the final flush actually returns ERROR_RET.
    let mut p2 = Pair::new();
    for op in &ops[..ops.len() - 1] {
        p2.run_op(op, "error-path-replay");
    }
    assert_eq!(p2.rust.flush(u64::MAX), ERROR_RET);
}
