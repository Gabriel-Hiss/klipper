//! Differential parity tests for `chelper_motion::itersolve`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - the return code of `itersolve_generate_steps` and the full
//!   `Stepcompress` state after each call (emitted MCU message bytes and
//!   clocks plus the step-command history from `stepcompress_extract_old`,
//!   and the pending step direction), and
//! - every `StepperKinematics` field the solver mutates
//!   (`commanded_pos`, `last_flush_time`, `last_move_time`, `step_dist`,
//!   the `gen_steps_*` fields, and `active_flags`), and
//! - the return values of `itersolve_check_active`,
//!   `itersolve_calc_position_from_coord`, `itersolve_is_active_axis`,
//!   and the void `itersolve_set_position` / `itersolve_set_trapq` /
//!   `itersolve_get_trapq` paths.
//!
//! The C side is allocated through the reference
//! `cartesian_stepper_alloc` (from `kin_cartesian.c`); the Rust side
//! uses an equivalent test-local Cartesian callback
//! (`move_get_coord(m, t).x/.y/.z`) and the same active flag.
//!
//! `itersolve_generate_steps` is internal to the C library and not
//! observable from Python, so it never appears in the recordings under
//! `tests/fixtures/recordings/itersolve/` (those record only the
//! `__visible` entry points); the generated differential streams below
//! are the parity proof for the solver, and the same streams cover the
//! visible functions against the reference C library, which is the
//! oracle for this C unit.
//!
//! The sequences cover: a forward move and a reversal (direction-change
//! detection in the solver), inactive moves with nonzero
//! `gen_steps_pre_active` / `gen_steps_post_active` (pre/post-active step
//! generation), flush times inside and past a move, gaps in `print_time`
//! (null moves), `tq = None`, and deterministic pseudo-random runs.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test itersolve`
use chelper_motion::itersolve::{
    StepperCalcCb, StepperKinematics, AF_X, AF_Y, AF_Z,
};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{move_get_coord, Move, Trapq};
use core::ffi::c_void;

use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// stepcompress.h / msgblock.h / list.h).
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

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout. Only used to
/// read back the state the C solver mutates and to set the two fields
/// that have no C setter (`gen_steps_pre_active`,
/// `gen_steps_post_active`, written by the extruder/shaper C code).
#[repr(C)]
#[allow(dead_code)]
struct RefSk {
    step_dist: f64,
    commanded_pos: f64,
    sc: *mut c_void,
    last_flush_time: f64,
    last_move_time: f64,
    tq: *mut CTrapq,
    active_flags: i32,
    gen_steps_pre_active: f64,
    gen_steps_post_active: f64,
    calc_position_cb: *mut c_void,
    post_cb: *mut c_void,
}
enum CTrapq {}

unsafe extern "C" {
    fn trapq_alloc() -> *mut CTrapq;
    fn trapq_free(tq: *mut CTrapq);
    fn trapq_check_sentinels(tq: *mut CTrapq);
    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        tq: *mut CTrapq,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos_x: f64,
        start_pos_y: f64,
        start_pos_z: f64,
        axes_r_x: f64,
        axes_r_y: f64,
        axes_r_z: f64,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    );
    fn trapq_finalize_moves(tq: *mut CTrapq, print_time: f64, clear_history_time: f64);
    fn stepcompress_alloc(msg_queue: *mut ListHead) -> *mut c_void;
    fn stepcompress_free(sc: *mut c_void);
    fn stepcompress_fill(
        sc: *mut c_void,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    );
    fn stepcompress_set_time(sc: *mut c_void, time_offset: f64, mcu_freq: f64);
    fn stepcompress_flush(sc: *mut c_void, move_clock: u64) -> i32;
    fn stepcompress_extract_old(
        sc: *mut c_void,
        p: *mut RefHistory,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> i32;
    fn stepcompress_get_oid(sc: *mut c_void) -> u32;
    fn stepcompress_get_step_dir(sc: *mut c_void) -> i32;
    fn message_queue_free(root: *mut ListHead);
    fn cartesian_stepper_alloc(axis: i8) -> *mut RefSk;
    fn itersolve_generate_steps(sk: *mut RefSk, sc: *mut c_void, flush_time: f64) -> i32;
    fn itersolve_check_active(sk: *mut RefSk, flush_time: f64) -> f64;
    fn itersolve_is_active_axis(sk: *mut RefSk, axis: i8) -> i32;
    fn itersolve_set_trapq(sk: *mut RefSk, tq: *mut CTrapq, step_dist: f64);
    fn itersolve_get_trapq(sk: *mut RefSk) -> *mut CTrapq;
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
    fn itersolve_get_gen_steps_pre_active(sk: *mut RefSk) -> f64;
    fn itersolve_get_gen_steps_post_active(sk: *mut RefSk) -> f64;
    fn free(ptr: *mut c_void);
}

// ---------------------------------------------------------------------------
// Reference-side Stepcompress handle (as in the stepcompress tests).
// ---------------------------------------------------------------------------

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

    fn extract_old(&mut self, max: i32) -> Vec<RefHistory> {
        let want = max.max(0) as usize;
        if self.scratch.len() < want {
            self.scratch.resize(want, RefHistory::zero());
        }
        let res = unsafe {
            stepcompress_extract_old(self.sc, self.scratch.as_mut_ptr(), max, 0, u64::MAX)
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
// Test-local Cartesian callbacks (the Rust equivalent of kin_cartesian.c).
// ---------------------------------------------------------------------------

fn cart_x(_sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    move_get_coord(m, move_time).x
}

fn cart_y(_sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    move_get_coord(m, move_time).y
}

fn cart_z(_sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    move_get_coord(m, move_time).z
}

// ---------------------------------------------------------------------------
// Op stream.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct MoveParams {
    print_time: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    start_v: f64,
    cruise_v: f64,
    accel: f64,
}

#[derive(Clone, Debug)]
enum Op {
    SetGenActive { pre: f64, post: f64 },
    SetPosition { x: f64, y: f64, z: f64 },
    CalcFromCoord { x: f64, y: f64, z: f64 },
    Append(MoveParams),
    GenerateSteps(f64),
    CheckActive(f64),
    IsActiveAxis(u8),
    Flush(u64),
    SetTime(f64, f64),
    SetTrapqNone(f64),
    SetTrapqSome(f64),
    Finalize(f64, f64),
    Snapshot,
}

struct Pair {
    rust_sk: StepperKinematics,
    c_sk: *mut RefSk,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
    rust_tq: Arc<Mutex<Trapq>>,
    c_tq: *mut CTrapq,
}

impl Pair {
    fn new(axis: u8, step_dist: f64) -> Pair {
        chelper_ref::link();
        let (rust_cb, flags): (StepperCalcCb, i32) = match axis {
            b'x' => (cart_x as StepperCalcCb, AF_X),
            b'y' => (cart_y as StepperCalcCb, AF_Y),
            _ => (cart_z as StepperCalcCb, AF_Z),
        };
        let rust_sk = StepperKinematics::new(rust_cb, flags);
        let c_sk = unsafe { cartesian_stepper_alloc(axis as i8) };
        assert!(!c_sk.is_null(), "cartesian_stepper_alloc returned null");
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "trapq_alloc returned null");
        let rust_tq = Arc::new(Mutex::new(Trapq::new()));
        let mut ref_sc = RefSc::new();
        unsafe {
            stepcompress_fill(ref_sc.sc, 7, 3000, 24, 23);
            stepcompress_set_time(ref_sc.sc, 0.0, 1_000_000.0);
        }
        let mut rust_sc = Stepcompress::new();
        rust_sc.fill(7, 3000, 24, 23);
        rust_sc.set_time(0.0, 1_000_000.0);
        let mut pair = Pair {
            rust_sk,
            c_sk,
            rust_sc,
            ref_sc,
            rust_tq,
            c_tq,
        };
        pair.set_trapq(true, step_dist);
        pair
    }

    fn set_trapq(&mut self, some: bool, step_dist: f64) {
        if some {
            unsafe { itersolve_set_trapq(self.c_sk, self.c_tq, step_dist) };
            self.rust_sk
                .set_trapq(Some(self.rust_tq.clone()), step_dist);
        } else {
            unsafe { itersolve_set_trapq(self.c_sk, core::ptr::null_mut(), step_dist) };
            self.rust_sk.set_trapq(None, step_dist);
        }
    }

    fn compare_sk(&self, ctx: &str) {
        let c = unsafe { &*self.c_sk };
        let r = &self.rust_sk;
        assert_eq!(
            c.commanded_pos.to_bits(),
            r.commanded_pos.to_bits(),
            "{ctx}: commanded_pos C={:?} Rust={:?}",
            c.commanded_pos,
            r.commanded_pos
        );
        assert_eq!(
            c.step_dist.to_bits(),
            r.step_dist.to_bits(),
            "{ctx}: step_dist C={:?} Rust={:?}",
            c.step_dist,
            r.step_dist
        );
        assert_eq!(
            c.last_flush_time.to_bits(),
            r.last_flush_time.to_bits(),
            "{ctx}: last_flush_time C={:?} Rust={:?}",
            c.last_flush_time,
            r.last_flush_time
        );
        assert_eq!(
            c.last_move_time.to_bits(),
            r.last_move_time.to_bits(),
            "{ctx}: last_move_time C={:?} Rust={:?}",
            c.last_move_time,
            r.last_move_time
        );
        assert_eq!(
            c.active_flags, r.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.active_flags, r.active_flags
        );
        assert_eq!(
            c.gen_steps_pre_active.to_bits(),
            r.gen_steps_pre_active.to_bits(),
            "{ctx}: gen_steps_pre_active C={:?} Rust={:?}",
            c.gen_steps_pre_active,
            r.gen_steps_pre_active
        );
        assert_eq!(
            c.gen_steps_post_active.to_bits(),
            r.gen_steps_post_active.to_bits(),
            "{ctx}: gen_steps_post_active C={:?} Rust={:?}",
            c.gen_steps_post_active,
            r.gen_steps_post_active
        );
    }

    fn compare_sc(&mut self, ctx: &str) {
        let rust_msgs: Vec<(Vec<u8>, u64, u64)> = self
            .rust_sc
            .drain_messages()
            .iter()
            .map(|m: &QueuedMessage| (m.bytes.clone(), m.req_clock, m.min_clock))
            .collect();
        let ref_msgs = self.ref_sc.drain_messages();
        assert_eq!(rust_msgs, ref_msgs, "{ctx}: messages diverged");
        const HISTORY_MAX: i32 = 8192;
        let rust_hist = self.rust_sc.extract_old(HISTORY_MAX, 0, u64::MAX);
        let ref_hist = self.ref_sc.extract_old(HISTORY_MAX);
        assert_eq!(
            rust_hist.len(),
            ref_hist.len(),
            "{ctx}: history length diverged (Rust={} C={})",
            rust_hist.len(),
            ref_hist.len()
        );
        for (i, (rh, ch)) in rust_hist.iter().zip(ref_hist.iter()).enumerate() {
            assert_eq!(
                (rh.first_clock, rh.last_clock, rh.start_position, rh.step_count, rh.interval, rh.add),
                (ch.first_clock, ch.last_clock, ch.start_position, ch.step_count, ch.interval, ch.add),
                "{ctx}: history[{i}] diverged"
            );
        }
        assert_eq!(
            self.rust_sc.get_step_dir(),
            unsafe { stepcompress_get_step_dir(self.ref_sc.sc) },
            "{ctx}: step dir diverged"
        );
        assert_eq!(
            self.rust_sc.get_oid(),
            unsafe { stepcompress_get_oid(self.ref_sc.sc) },
            "{ctx}: oid diverged"
        );
    }

    fn compare_all(&mut self, ctx: &str) {
        self.compare_sk(ctx);
        self.compare_sc(ctx);
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe {
            free(self.c_sk as *mut c_void);
            trapq_free(self.c_tq);
        }
    }
}

// ---------------------------------------------------------------------------
// Op driver.
// ---------------------------------------------------------------------------

fn run_op(pair: &mut Pair, op: &Op, ctx: &str) {
    match *op {
        Op::SetGenActive { pre, post } => {
            pair.rust_sk.gen_steps_pre_active = pre;
            pair.rust_sk.gen_steps_post_active = post;
            let c = unsafe { &mut *pair.c_sk };
            c.gen_steps_pre_active = pre;
            c.gen_steps_post_active = post;
        }
        Op::SetPosition { x, y, z } => {
            unsafe { itersolve_set_position(pair.c_sk, x, y, z) };
            pair.rust_sk.set_position(x, y, z);
        }
        Op::CalcFromCoord { x, y, z } => {
            let c = unsafe { itersolve_calc_position_from_coord(pair.c_sk, x, y, z) };
            let r = pair.rust_sk.calc_position_from_coord(x, y, z);
            assert_eq!(
                c.to_bits(),
                r.to_bits(),
                "{ctx}: calc_position_from_coord C={c:?} Rust={r:?}"
            );
        }
        Op::Append(p) => {
            unsafe {
                trapq_append(
                    pair.c_tq,
                    p.print_time,
                    p.accel_t,
                    p.cruise_t,
                    p.decel_t,
                    p.start_pos[0],
                    p.start_pos[1],
                    p.start_pos[2],
                    p.axes_r[0],
                    p.axes_r[1],
                    p.axes_r[2],
                    p.start_v,
                    p.cruise_v,
                    p.accel,
                )
            };
            pair.rust_tq.lock().append(
                p.print_time,
                p.accel_t,
                p.cruise_t,
                p.decel_t,
                p.start_pos[0],
                p.start_pos[1],
                p.start_pos[2],
                p.axes_r[0],
                p.axes_r[1],
                p.axes_r[2],
                p.start_v,
                p.cruise_v,
                p.accel,
            );
        }
        Op::GenerateSteps(flush_time) => {
            let c = unsafe { itersolve_generate_steps(pair.c_sk, pair.ref_sc.sc, flush_time) };
            let r = pair.rust_sk.generate_steps(&mut pair.rust_sc, flush_time);
            assert_eq!(c, r, "{ctx}: generate_steps({flush_time}) return diverged");
        }
        Op::CheckActive(flush_time) => {
            let c = unsafe { itersolve_check_active(pair.c_sk, flush_time) };
            let r = pair.rust_sk.check_active(flush_time);
            assert_eq!(
                c.to_bits(),
                r.to_bits(),
                "{ctx}: check_active({flush_time}) C={c:?} Rust={r:?}"
            );
        }
        Op::IsActiveAxis(axis) => {
            let c = unsafe { itersolve_is_active_axis(pair.c_sk, axis as i8) };
            let r = pair.rust_sk.is_active_axis(axis);
            assert_eq!(c, r, "{ctx}: is_active_axis({axis}) diverged");
        }
        Op::Flush(move_clock) => {
            let c = unsafe { stepcompress_flush(pair.ref_sc.sc, move_clock) };
            let r = pair.rust_sc.flush(move_clock);
            assert_eq!(c, r, "{ctx}: stepcompress_flush return diverged");
        }
        Op::SetTime(off, freq) => {
            unsafe { stepcompress_set_time(pair.ref_sc.sc, off, freq) };
            pair.rust_sc.set_time(off, freq);
        }
        Op::SetTrapqNone(step_dist) => {
            pair.set_trapq(false, step_dist);
        }
        Op::SetTrapqSome(step_dist) => {
            pair.set_trapq(true, step_dist);
        }
        Op::Finalize(pt, cht) => {
            unsafe { trapq_finalize_moves(pair.c_tq, pt, cht) };
            pair.rust_tq.lock().finalize_moves(pt, cht);
        }
        Op::Snapshot => {}
    }
    // Visible-function probes and field checks after every op.
    assert_eq!(
        unsafe { itersolve_get_commanded_pos(pair.c_sk) }.to_bits(),
        pair.rust_sk.get_commanded_pos().to_bits(),
        "{ctx}: get_commanded_pos diverged"
    );
    assert_eq!(
        unsafe { itersolve_get_gen_steps_pre_active(pair.c_sk) }.to_bits(),
        pair.rust_sk.get_gen_steps_pre_active().to_bits(),
        "{ctx}: get_gen_steps_pre_active diverged"
    );
    assert_eq!(
        unsafe { itersolve_get_gen_steps_post_active(pair.c_sk) }.to_bits(),
        pair.rust_sk.get_gen_steps_post_active().to_bits(),
        "{ctx}: get_gen_steps_post_active diverged"
    );
    if pair.rust_sk.tq.is_none() {
        assert!(
            unsafe { itersolve_get_trapq(pair.c_sk) }.is_null(),
            "{ctx}: C trapq is not null"
        );
    } else {
        assert_eq!(
            unsafe { itersolve_get_trapq(pair.c_sk) },
            pair.c_tq,
            "{ctx}: C trapq mismatch"
        );
        assert!(
            Arc::ptr_eq(pair.rust_sk.get_trapq().as_ref().unwrap(), &pair.rust_tq),
            "{ctx}: Rust trapq mismatch"
        );
    }
    pair.compare_all(ctx);
}

fn run_stream(ctx: &str, axis: u8, step_dist: f64, ops: &[Op]) {
    let mut pair = Pair::new(axis, step_dist);
    pair.compare_all(&format!("{ctx}#init"));
    for (i, op) in ops.iter().enumerate() {
        run_op(&mut pair, op, &format!("{ctx}#{i}"));
    }
}

// ---------------------------------------------------------------------------
// Deterministic scenarios.
// ---------------------------------------------------------------------------

fn cruise(print_time: f64, start: [f64; 3], axes: [f64; 3], v: f64, t: f64) -> MoveParams {
    MoveParams {
        print_time,
        accel_t: 0.0,
        cruise_t: t,
        decel_t: 0.0,
        start_pos: start,
        axes_r: axes,
        start_v: 0.0,
        cruise_v: v,
        accel: 0.0,
    }
}

fn trapezoid(
    print_time: f64,
    start: [f64; 3],
    axes: [f64; 3],
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    start_v: f64,
    cruise_v: f64,
    accel: f64,
) -> MoveParams {
    MoveParams {
        print_time,
        accel_t,
        cruise_t,
        decel_t,
        start_pos: start,
        axes_r: axes,
        start_v,
        cruise_v,
        accel,
    }
}

/// Forward cruise on X, then a reversal, then a more-axes mixture.
fn forward_then_reverse() {
    let ops = vec![
        Op::SetGenActive { pre: 0.0, post: 0.0 },
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::CalcFromCoord { x: 3.5, y: 1.25, z: -2.0 },
        Op::CheckActive(10.0),
        // X cruise 0 -> +20 mm (v=20 for 1 s), starting at print_time 1.
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, 1.0)),
        Op::CheckActive(10.0),
        Op::GenerateSteps(1.5), // flush halfway through the move
        Op::Flush(u64::MAX),
        Op::GenerateSteps(2.5), // past the move end
        Op::Flush(u64::MAX),
        // Reversal: back along X from 20 to 5.
        Op::Append(cruise(2.0, [20.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 15.0, 1.0)),
        Op::GenerateSteps(3.5),
        Op::Flush(u64::MAX),
        // Trapezoid: accel/cruise/decel along X.
        Op::Append(trapezoid(
            3.0,
            [5.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            0.25,
            0.5,
            0.25,
            0.0,
            30.0,
            120.0,
        )),
        Op::GenerateSteps(4.5),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    run_stream("forward-then-reverse", b'x', 0.0125, &ops);
}

/// Inactive moves with nonzero pre/post-active windows.
fn inactive_and_prepost() {
    let ops = vec![
        Op::SetGenActive { pre: 0.25, post: 0.25 },
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        // Inactive for the X stepper (moves only in Y).
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 10.0, 1.0)),
        // Active X move.
        Op::Append(cruise(2.0, [0.0, 10.0, 0.0], [1.0, 0.0, 0.0], 10.0, 1.0)),
        // Inactive gap move, then another active move.
        Op::Append(cruise(3.0, [10.0, 10.0, 0.0], [0.0, 0.0, 0.0], 0.0, 1.0)),
        Op::Append(cruise(4.0, [10.0, 10.0, 0.0], [1.0, 0.0, 0.0], 5.0, 1.0)),
        Op::CheckActive(10.0),
        Op::GenerateSteps(4.5),
        Op::Flush(u64::MAX),
        Op::GenerateSteps(10.0),
        Op::Flush(u64::MAX),
        Op::IsActiveAxis(b'x'),
        Op::IsActiveAxis(b'y'),
        Op::IsActiveAxis(b'z'),
        Op::IsActiveAxis(b'a'),
        Op::Snapshot,
    ];
    run_stream("inactive-prepost", b'x', 0.0125, &ops);
}

/// No trapq attached, then re-attached mid-stream.
fn trapq_none_and_back() {
    let ops = vec![
        Op::SetTrapqNone(0.0025),
        Op::GenerateSteps(5.0),
        Op::CheckActive(5.0),
        Op::SetTrapqSome(0.0025),
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 10.0, 1.0)),
        Op::CheckActive(2.5),
        Op::GenerateSteps(2.5),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    run_stream("trapq-none", b'z', 0.0025, &ops);
}

// Small deterministic generator (SplitMix64), as in the trapq tests.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^ (z >> 31)
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }

    fn f64t(&mut self, table: &[f64]) -> f64 {
        table[self.pick(table.len())]
    }
}

const PRINT_TIMES: &[f64] = &[
    0.0, 0.5, 1.0, 1.5, 2.0, 2.25, 3.0, 4.0, 4.5, 5.0, 6.0, 8.0, 10.0,
];
const DURS: &[f64] = &[0.05, 0.1, 0.25, 0.5, 1.0];
const VELS: &[f64] = &[1.0, 5.0, 20.0, 40.0];
const ACCELS: &[f64] = &[0.0, 50.0, 120.0, 300.0];
const PREPOST: &[f64] = &[0.0, 0.0, 0.05, 0.2];

fn random_stream(seed: u64, axis: u8, step_dist: f64, rounds: usize) {
    let mut rng = Rng(seed);
    let axis_idx = match axis {
        b'x' => 0,
        b'y' => 1,
        _ => 2,
    };
    let mut ops: Vec<Op> = Vec::new();
    ops.push(Op::SetGenActive {
        pre: rng.f64t(PREPOST),
        post: rng.f64t(PREPOST),
    });
    let mut pos = [0.0f64; 3];
    ops.push(Op::SetPosition {
        x: pos[0],
        y: pos[1],
        z: pos[2],
    });
    let mut time = rng.f64t(&[0.0, 0.5, 1.0]);
    for _ in 0..rounds {
        // Append 1-3 moves.
        for _ in 0..(1 + rng.pick(3)) {
            let sign: f64 = if rng.pick(4) == 0 { -1.0 } else { 1.0 };
            let mut axes = [0.0f64; 3];
            // Occasionally an inactive move in another axis.
            if rng.pick(5) == 0 {
                axes[(axis_idx + 1) % 3] = sign;
            } else {
                axes[axis_idx] = sign;
            }
            let accel_t = rng.f64t(DURS);
            let cruise_t = rng.f64t(DURS);
            let decel_t = rng.f64t(DURS);
            let start_v = rng.f64t(VELS);
            let cruise_v = rng.f64t(VELS);
            let accel = rng.f64t(ACCELS);
            let p = trapezoid(
                time,
                pos,
                axes,
                accel_t,
                cruise_t,
                decel_t,
                start_v,
                cruise_v,
                accel,
            );
            // Advance the tracked position by the trapezoid distance.
            let dist = (start_v + 0.5 * accel * accel_t) * accel_t
                + cruise_v * cruise_t
                + (cruise_v - 0.5 * accel * decel_t) * decel_t;
            pos[axis_idx] += axes[axis_idx] * dist;
            time += accel_t + cruise_t + decel_t;
            if rng.pick(4) == 0 {
                time += rng.f64t(DURS); // a gap (null move)
            }
            ops.push(Op::Append(p));
        }
        // Step generation: inside the last move or past the queued end.
        let flush = if rng.pick(2) == 0 {
            time
        } else {
            time + rng.f64t(DURS)
        };
        ops.push(Op::CheckActive(flush));
        ops.push(Op::GenerateSteps(flush));
        if rng.pick(3) == 0 {
            ops.push(Op::Flush(u64::MAX));
        }
        if rng.pick(4) == 0 {
            ops.push(Op::SetPosition {
                x: pos[0],
                y: pos[1],
                z: pos[2],
            });
        }
        if rng.pick(5) == 0 {
            ops.push(Op::CalcFromCoord {
                x: pos[0],
                y: pos[1],
                z: pos[2],
            });
        }
        if rng.pick(6) == 0 {
            ops.push(Op::SetGenActive {
                pre: rng.f64t(PREPOST),
                post: rng.f64t(PREPOST),
            });
        }
    }
    ops.push(Op::GenerateSteps(time + 1.0));
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);
    run_stream(&format!("random-{seed}-{axis}"), axis, step_dist, &ops);
}

#[test]
fn directed_scenarios_match_c() {
    forward_then_reverse();
    inactive_and_prepost();
    trapq_none_and_back();
}

#[test]
fn generated_streams_match_c() {
    // Gaps keep `print_time` nondecreasing within each stream; the
    // flush times are always >= the previous flush time.
    random_stream(1, b'x', 0.0125, 10);
    random_stream(42, b'y', 0.0125, 10);
    random_stream(0xdeadbeefcafe, b'z', 0.0025, 10);
    random_stream(987654321, b'x', 0.00625, 14);
}
