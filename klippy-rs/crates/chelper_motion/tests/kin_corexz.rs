//! Differential parity tests for `chelper_motion::kin_corexz` against the
//! frozen reference C library (`chelper_ref`, the unchanged
//! `klippy/chelper/kin_corexz.c` sources).
//!
//! The C unit is three functions: the `'+'`/`'-'` position callbacks
//! (`x + z` / `x - z` of `move_get_coord`) and `corexz_stepper_alloc`.
//! The tests compare, bit for bit (`f64::to_bits`):
//!
//! - the state of `corexz_stepper_alloc(b'+')` / `corexz_stepper_alloc(b'-')`
//!   on both sides (every [`StepperKinematics`] field: the zeroed double
//!   fields, null `tq`/`sc`/`post_cb`, `active_flags == AF_X | AF_Z`, and
//!   the selected callback pointer identity), and
//! - the return value of each callback (the C callback read out of the
//!   allocated C object and called with a `struct move`) over a table of
//!   moves with zero/negative/fractional/large coordinates and a range of
//!   `move_time` values (start, middle, end and beyond the move), and
//! - the end-to-end solver behaviour with the CoreXZ callbacks installed
//!   on both sides (return codes of `itersolve_generate_steps`, the full
//!   `Stepcompress` state after each call: emitted message bytes and
//!   clocks plus the step-command history from `stepcompress_extract_old`,
//!   the pending step direction, and the `StepperKinematics` fields the
//!   solver mutates) over a stream with XZ acceleration/deceleration,
//!   a direction change, a `tq = None` gap, and flush times inside and
//!   past the moves.
//!
//! `corexz_stepper_alloc` is internal to the C library and not observable
//! from Python, so it never appears in the recordings under
//! `tests/fixtures/recordings/` (those record only the `__visible` entry
//! points); the reference C library is the oracle for this unit, and the
//! generated streams above are the parity proof.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_corexz`
use chelper_motion::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Z};
use chelper_motion::kin_corexz::{
    corexz_stepper_alloc, corexz_stepper_minus_calc_position,
    corexz_stepper_plus_calc_position,
};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{Coord, Move, Trapq};
use core::ffi::c_void;

use std::cell::RefCell;
use std::rc::Rc;

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

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout.
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

/// `struct coord` (trapq.h).
#[repr(C)]
#[derive(Clone, Copy)]
struct RefCoord {
    x: f64,
    y: f64,
    z: f64,
}

/// `struct move` (trapq.h).
#[repr(C)]
struct RefMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: RefCoord,
    axes_r: RefCoord,
    node: ListNode,
}

enum CTrapq {}

type RefCalcCb = unsafe extern "C" fn(*mut RefSk, *mut RefMove, f64) -> f64;

unsafe extern "C" {
    fn trapq_alloc() -> *mut CTrapq;
    fn trapq_free(tq: *mut CTrapq);
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
    #[link_name = "corexz_stepper_alloc"]
    fn corexz_stepper_alloc_c(kind: i8) -> *mut RefSk;
    fn itersolve_generate_steps(sk: *mut RefSk, sc: *mut c_void, flush_time: f64) -> i32;
    fn itersolve_set_trapq(sk: *mut RefSk, tq: *mut CTrapq, step_dist: f64);
    fn free(ptr: *mut c_void);
}

// ---------------------------------------------------------------------------
// Reference-side Stepcompress handle (as in the stepcompress/itersolve tests).
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
// Callback differential tests.
// ---------------------------------------------------------------------------

fn ref_move(m: &Move) -> RefMove {
    RefMove {
        print_time: m.print_time,
        move_t: m.move_t,
        start_v: m.start_v,
        half_accel: m.half_accel,
        start_pos: RefCoord {
            x: m.start_pos.x,
            y: m.start_pos.y,
            z: m.start_pos.z,
        },
        axes_r: RefCoord {
            x: m.axes_r.x,
            y: m.axes_r.y,
            z: m.axes_r.z,
        },
        node: ListNode {
            next: core::ptr::null_mut(),
            prev: core::ptr::null_mut(),
        },
    }
}

const MOVES: &[Move] = &[
    // Zero move at the origin.
    Move {
        print_time: 0.0,
        move_t: 1.0,
        start_v: 0.0,
        half_accel: 0.0,
        start_pos: Coord::new(0.0, 0.0, 0.0),
        axes_r: Coord::new(0.0, 0.0, 0.0),
    },
    // XZ cruise.
    Move {
        print_time: 1.5,
        move_t: 1.0,
        start_v: 20.0,
        half_accel: 0.0,
        start_pos: Coord::new(10.0, 5.0, 20.0),
        axes_r: Coord::new(0.8, 0.0, 0.6),
    },
    // Accelerating diagonal with a Y component (ignored by CoreXZ).
    Move {
        print_time: 2.0,
        move_t: 0.5,
        start_v: 5.0,
        half_accel: 30.0,
        start_pos: Coord::new(-3.25, 7.5, 2.125),
        axes_r: Coord::new(-0.6, 0.5, 0.8),
    },
    // Decelerating negative Z creep.
    Move {
        print_time: 3.25,
        move_t: 2.0,
        start_v: 1.0,
        half_accel: -0.25,
        start_pos: Coord::new(120.0, -40.0, 305.0),
        axes_r: Coord::new(0.0, 0.0, -1.0),
    },
    // Large coordinates, fractional velocities.
    Move {
        print_time: 10.0,
        move_t: 4.0,
        start_v: 0.001,
        half_accel: 123.456,
        start_pos: Coord::new(1.0e6, 2.5e5, -9.75e5),
        axes_r: Coord::new(0.123, -0.456, 0.789),
    },
];

const MOVE_TIMES: &[f64] = &[0.0, 0.05, 0.25, 0.5, 1.0, 1.5, 2.0, 4.0, 10.0];

#[test]
fn alloc_state_matches_c() {
    chelper_ref::link();

    for (kind, want_cb) in [
        (b'+' as i8, corexz_stepper_plus_calc_position as StepperCalcCb),
        (b'-' as i8, corexz_stepper_minus_calc_position as StepperCalcCb),
    ] {
        let rust_sk = corexz_stepper_alloc(kind);
        assert_eq!(rust_sk.step_dist.to_bits(), 0.0f64.to_bits(), "kind {kind}: step_dist");
        assert_eq!(
            rust_sk.commanded_pos.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: commanded_pos"
        );
        assert_eq!(
            rust_sk.last_flush_time.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: last_flush_time"
        );
        assert_eq!(
            rust_sk.last_move_time.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: last_move_time"
        );
        assert_eq!(
            rust_sk.gen_steps_pre_active.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: gen_steps_pre_active"
        );
        assert_eq!(
            rust_sk.gen_steps_post_active.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: gen_steps_post_active"
        );
        assert!(rust_sk.tq.is_none(), "kind {kind}: tq");
        assert!(rust_sk.sc.is_none(), "kind {kind}: sc");
        assert!(rust_sk.post_cb.is_none(), "kind {kind}: post_cb");
        assert_eq!(rust_sk.active_flags, AF_X | AF_Z, "kind {kind}: active_flags");
        // Address comparison: fn-pointer `==` is flagged as unpredictable,
        // but both pointers live in this one test binary.
        assert_eq!(
            rust_sk.calc_position_cb as usize, want_cb as usize,
            "kind {kind}: callback"
        );

        let c_sk = unsafe { corexz_stepper_alloc_c(kind) };
        assert!(!c_sk.is_null(), "kind {kind}: C alloc returned null");
        let c = unsafe { &*c_sk };
        assert_eq!(c.step_dist.to_bits(), 0.0f64.to_bits(), "kind {kind}: C step_dist");
        assert_eq!(c.commanded_pos.to_bits(), 0.0f64.to_bits(), "kind {kind}: C commanded_pos");
        assert_eq!(c.last_flush_time.to_bits(), 0.0f64.to_bits(), "kind {kind}: C last_flush_time");
        assert_eq!(c.last_move_time.to_bits(), 0.0f64.to_bits(), "kind {kind}: C last_move_time");
        assert_eq!(
            c.gen_steps_pre_active.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: C gen_steps_pre_active"
        );
        assert_eq!(
            c.gen_steps_post_active.to_bits(),
            0.0f64.to_bits(),
            "kind {kind}: C gen_steps_post_active"
        );
        assert!(c.sc.is_null(), "kind {kind}: C sc");
        assert!(c.tq.is_null(), "kind {kind}: C tq");
        assert!(c.post_cb.is_null(), "kind {kind}: C post_cb");
        assert_eq!(c.active_flags, AF_X | AF_Z, "kind {kind}: C active_flags");
        assert!(!c.calc_position_cb.is_null(), "kind {kind}: C callback non-null");
        unsafe { free(c_sk as *mut c_void) };
    }

    // The two sides must select the same logical callback for each kind:
    // the C value read out of a `'+'` object and a `'-'` object differ.
    let plus_sk = unsafe { corexz_stepper_alloc_c(b'+' as i8) };
    let minus_sk = unsafe { corexz_stepper_alloc_c(b'-' as i8) };
    assert!(!plus_sk.is_null() && !minus_sk.is_null());
    let plus_cb = unsafe { (*plus_sk).calc_position_cb };
    let minus_cb = unsafe { (*minus_sk).calc_position_cb };
    assert_ne!(plus_cb, minus_cb, "'+' and '-' must select distinct callbacks");
    unsafe {
        free(plus_sk as *mut c_void);
        free(minus_sk as *mut c_void);
    }
}

#[test]
fn calc_position_matches_c() {
    chelper_ref::link();

    for (kind, rust_cb) in [
        (b'+' as i8, corexz_stepper_plus_calc_position as StepperCalcCb),
        (b'-' as i8, corexz_stepper_minus_calc_position as StepperCalcCb),
    ] {
        let rust_sk = corexz_stepper_alloc(kind);
        assert_eq!(
            rust_sk.calc_position_cb as usize, rust_cb as usize,
            "kind {kind}: alloc callback"
        );
        let c_sk = unsafe { corexz_stepper_alloc_c(kind) };
        assert!(!c_sk.is_null(), "kind {kind}: C alloc returned null");
        let c_cb: RefCalcCb =
            unsafe { core::mem::transmute::<*mut c_void, RefCalcCb>((*c_sk).calc_position_cb) };
        for (mi, m) in MOVES.iter().enumerate() {
            for t in MOVE_TIMES {
                let c_val = unsafe { c_cb(c_sk, &mut ref_move(m) as *mut RefMove, *t) };
                let r_val = rust_cb(&rust_sk, m, *t);
                assert_eq!(
                    c_val.to_bits(),
                    r_val.to_bits(),
                    "kind {kind} move {mi} t={t}: C={c_val:?} Rust={r_val:?}"
                );
            }
        }
        unsafe { free(c_sk as *mut c_void) };
    }
}

// ---------------------------------------------------------------------------
// End-to-end: solver behaviour with the CoreXZ callbacks installed.
// ---------------------------------------------------------------------------

struct Pair {
    rust_sk: StepperKinematics,
    c_sk: *mut RefSk,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
    rust_tq: Rc<RefCell<Trapq>>,
    c_tq: *mut CTrapq,
}

impl Pair {
    fn new(kind: i8, step_dist: f64) -> Pair {
        chelper_ref::link();
        let rust_sk = corexz_stepper_alloc(kind);
        let c_sk = unsafe { corexz_stepper_alloc_c(kind) };
        assert!(!c_sk.is_null(), "kind {kind}: C alloc returned null");
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "kind {kind}: trapq_alloc returned null");
        let rust_tq = Rc::new(RefCell::new(Trapq::new()));
        let ref_sc = RefSc::new();
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

    #[allow(clippy::too_many_arguments)]
    fn append(
        &mut self,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos: [f64; 3],
        axes_r: [f64; 3],
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        unsafe {
            trapq_append(
                self.c_tq,
                print_time,
                accel_t,
                cruise_t,
                decel_t,
                start_pos[0],
                start_pos[1],
                start_pos[2],
                axes_r[0],
                axes_r[1],
                axes_r[2],
                start_v,
                cruise_v,
                accel,
            )
        };
        self.rust_tq.borrow_mut().append(
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            start_pos[0],
            start_pos[1],
            start_pos[2],
            axes_r[0],
            axes_r[1],
            axes_r[2],
            start_v,
            cruise_v,
            accel,
        );
    }

    fn finalize(&mut self, print_time: f64, clear_history_time: f64) {
        unsafe { trapq_finalize_moves(self.c_tq, print_time, clear_history_time) };
        self.rust_tq
            .borrow_mut()
            .finalize_moves(print_time, clear_history_time);
    }

    fn generate(&mut self, flush_time: f64, ctx: &str) {
        let rust_rc = self.rust_sk.generate_steps(&mut self.rust_sc, flush_time);
        let c_rc = unsafe { itersolve_generate_steps(self.c_sk, self.ref_sc.sc, flush_time) };
        assert_eq!(rust_rc, c_rc, "{ctx}: generate_steps return code");
        self.compare_all(ctx);
    }

    fn compare_sk(&self, ctx: &str) {
        let c = unsafe { &*self.c_sk };
        let r = &self.rust_sk;
        for (name, c_val, r_val) in [
            ("commanded_pos", c.commanded_pos, r.commanded_pos),
            ("step_dist", c.step_dist, r.step_dist),
            ("last_flush_time", c.last_flush_time, r.last_flush_time),
            ("last_move_time", c.last_move_time, r.last_move_time),
            (
                "gen_steps_pre_active",
                c.gen_steps_pre_active,
                r.gen_steps_pre_active,
            ),
            (
                "gen_steps_post_active",
                c.gen_steps_post_active,
                r.gen_steps_post_active,
            ),
        ] {
            assert_eq!(
                c_val.to_bits(),
                r_val.to_bits(),
                "{ctx}: {name} C={c_val:?} Rust={r_val:?}"
            );
        }
        assert_eq!(
            c.active_flags, r.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.active_flags, r.active_flags
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
                (
                    rh.first_clock,
                    rh.last_clock,
                    rh.start_position,
                    rh.step_count,
                    rh.interval,
                    rh.add
                ),
                (
                    ch.first_clock,
                    ch.last_clock,
                    ch.start_position,
                    ch.step_count,
                    ch.interval,
                    ch.add
                ),
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

fn run_stream(kind: i8) {
    let ctx = format!("kind {}", char::from(kind as u8));
    let mut pair = Pair::new(kind, 0.01);

    // XZ trapezoid: accelerate in X+Z, cruise, decelerate
    // (accelerate, cruise and decelerate happen inside one append).
    pair.append(
        0.0, 0.25, 0.5, 0.25, [10.0, 5.0, 20.0], [0.8, 0.0, 0.6], 0.0, 40.0, 160.0,
    );
    pair.generate(0.3, &format!("{ctx} after first append"));

    // Reversal in X and Z from the end of the first move.
    pair.append(
        1.0, 0.25, 0.5, 0.25, [34.0, 5.0, 38.0], [-0.8, 0.0, -0.6], 0.0, 40.0, 160.0,
    );
    pair.generate(1.2, &format!("{ctx} after reversal append"));
    pair.generate(2.5, &format!("{ctx} past second move"));

    // No trapq attached: the solver returns without touching the compressor.
    pair.set_trapq(false, 0.01);
    pair.generate(3.0, &format!("{ctx} with tq = None"));

    // Re-attached, a Y-only move is inactive for AF_X | AF_Z, then an XZ move.
    pair.set_trapq(true, 0.01);
    pair.append(2.0, 0.1, 0.2, 0.1, [10.0, 5.0, 20.0], [0.0, 1.0, 0.0], 0.0, 10.0, 100.0);
    pair.append(2.4, 0.1, 0.2, 0.1, [10.0, 8.0, 20.0], [0.6, 0.0, 0.8], 0.0, 10.0, 100.0);
    pair.generate(2.6, &format!("{ctx} after Y-only and XZ moves"));
    pair.finalize(2.8, 0.0);
    pair.generate(2.8, &format!("{ctx} after finalize"));
    pair.generate(5.0, &format!("{ctx} final flush"));
}

#[test]
fn solver_streams_match_c() {
    run_stream(b'+' as i8);
    run_stream(b'-' as i8);
}
