//! Differential parity tests for `chelper_motion::kin_winch`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - the state after `winch_stepper_alloc` (every `StepperKinematics`
//!   field plus the anchor and the `AF_X | AF_Y | AF_Z` active flags),
//! - `itersolve_calc_position_from_coord` (the `winch_stepper_calc_position`
//!   callback) over a table of anchors and coordinates, including
//!   signed zeros, negatives, infinities and extremes,
//! - `itersolve_set_position` / `itersolve_get_commanded_pos`,
//! - `itersolve_is_active_axis` for axis characters inside and outside
//!   `'x'..='z'`,
//! - `itersolve_generate_steps` over trapq moves (including a reversal
//!   for solver direction changes, an inactive move, pre/post-active
//!   step generation, and flush times inside and past a move):
//!   return codes, emitted MCU message bytes/clocks, the
//!   step-command history, the pending step direction, and every
//!   mutated `StepperKinematics` field.
//!
//! The C side is allocated through the reference `winch_stepper_alloc`
//! (from `kin_winch.c`); the Rust side through
//! `kin_winch::winch_stepper_alloc`. `itersolve_generate_steps` is
//! internal to the C library and not observable from Python, so -- like
//! the `itersolve` tests -- the generated differential streams are the
//! parity proof for the callback in context.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_winch`

use chelper_motion::itersolve::{AF_X, AF_Y, AF_Z};
use chelper_motion::kin_winch::{winch_stepper_alloc as make_winch, WinchStepper};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{Move, Trapq};
use core::ffi::c_void;

use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// stepcompress.h / msgblock.h / kin_winch.c).
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
/// `gen_steps_post_active`).
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

/// `struct winch_stepper` (kin_winch.c): `sk` first, then `anchor`. Used
/// to read back the anchor the C `winch_stepper_alloc` stored.
#[repr(C)]
#[allow(dead_code)]
struct RefWinchStepper {
    core: RefSk,
    anchor: RefCoord,
}

enum CTrapq {}

unsafe extern "C" {
    fn winch_stepper_alloc(anchor_x: f64, anchor_y: f64, anchor_z: f64) -> *mut RefWinchStepper;
    fn itersolve_calc_position_from_coord(sk: *mut RefWinchStepper, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefWinchStepper, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefWinchStepper) -> f64;
    fn itersolve_is_active_axis(sk: *mut RefWinchStepper, axis: i8) -> i32;
    fn itersolve_set_trapq(sk: *mut RefWinchStepper, tq: *mut CTrapq, step_dist: f64);
    fn itersolve_generate_steps(
        sk: *mut RefWinchStepper,
        sc: *mut c_void,
        flush_time: f64,
    ) -> i32;
    fn itersolve_check_active(sk: *mut RefWinchStepper, flush_time: f64) -> f64;
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
    fn free(ptr: *mut c_void);
}

// ---------------------------------------------------------------------------
// Reference-side Stepcompress handle (as in the itersolve tests).
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
// Allocation: the zeroed state the C allocation produces.
// ---------------------------------------------------------------------------

#[test]
fn alloc_zeroed_state() {
    chelper_ref::link();
    for anchor in [
        [0.0, 0.0, 0.0],
        [1.5, -2.5, 3.25],
        [-100.25, 200.5, -0.75],
        [f64::INFINITY, -1.0, 42.0],
    ] {
        let c_sk = unsafe { winch_stepper_alloc(anchor[0], anchor[1], anchor[2]) };
        assert!(!c_sk.is_null(), "winch_stepper_alloc returned null");
        let c = unsafe { &*c_sk };
        let r = make_winch(anchor[0], anchor[1], anchor[2]);

        assert_eq!(
            c.anchor.x.to_bits(),
            r.anchor.x.to_bits(),
            "anchor.x anchor={anchor:?}"
        );
        assert_eq!(
            c.anchor.y.to_bits(),
            r.anchor.y.to_bits(),
            "anchor.y anchor={anchor:?}"
        );
        assert_eq!(
            c.anchor.z.to_bits(),
            r.anchor.z.to_bits(),
            "anchor.z anchor={anchor:?}"
        );
        assert_eq!(
            c.core.active_flags,
            AF_X | AF_Y | AF_Z,
            "C active_flags anchor={anchor:?}"
        );
        assert_eq!(
            r.sk.active_flags,
            AF_X | AF_Y | AF_Z,
            "Rust active_flags anchor={anchor:?}"
        );
        for (name, cf, rf) in [
            ("step_dist", c.core.step_dist, r.sk.step_dist),
            ("commanded_pos", c.core.commanded_pos, r.sk.commanded_pos),
            ("last_flush_time", c.core.last_flush_time, r.sk.last_flush_time),
            ("last_move_time", c.core.last_move_time, r.sk.last_move_time),
            (
                "gen_steps_pre_active",
                c.core.gen_steps_pre_active,
                r.sk.gen_steps_pre_active,
            ),
            (
                "gen_steps_post_active",
                c.core.gen_steps_post_active,
                r.sk.gen_steps_post_active,
            ),
        ] {
            assert_eq!(
                cf.to_bits(),
                rf.to_bits(),
                "{name} anchor={anchor:?}: C={cf:?} Rust={rf:?}"
            );
            assert_eq!(
                cf.to_bits(),
                0.0f64.to_bits(),
                "{name} anchor={anchor:?} not zeroed"
            );
        }
        assert!(c.core.tq.is_null(), "C tq not null anchor={anchor:?}");
        assert!(c.core.sc.is_null(), "C sc not null anchor={anchor:?}");
        assert!(
            !c.core.calc_position_cb.is_null(),
            "C calc_position_cb null anchor={anchor:?}"
        );
        assert!(c.core.post_cb.is_null(), "C post_cb not null anchor={anchor:?}");
        assert!(r.sk.tq.is_none(), "Rust tq not None anchor={anchor:?}");
        assert!(r.sk.sc.is_none(), "Rust sc not None anchor={anchor:?}");
        assert!(r.sk.post_cb.is_none(), "Rust post_cb not None anchor={anchor:?}");

        unsafe { free(c_sk as *mut c_void) };
}
}

/// The installed callback is the winch callback, on the Rust side too.
#[test]
fn callback_identity() {
    chelper_ref::link();
    let r = make_winch(1.0, 2.0, 3.0);
    let m = Move::default();
    let cb = r.sk.calc_position_cb;
    assert_eq!(
        cb(&r.sk, &m, None, 500.0).to_bits(),
        r.calc_position(&m, 500.0).to_bits(),
        "callback disagrees with method"
    );
}

// ---------------------------------------------------------------------------
// Direct callback: `itersolve_calc_position_from_coord`.
// ---------------------------------------------------------------------------

#[test]
fn calc_position_from_coord_matches() {
    chelper_ref::link();
    let anchors = [
        [0.0, 0.0, 0.0],
        [10.0, 20.0, 30.0],
        [-5.0, 3.25, 7.5],
        [100.0, -100.0, 0.001],
        [0.0, -0.0, 1.0],
        [f64::INFINITY, 1.0, 1.0],
        [250.0, 250.0, 250.0],
    ];
    let coords = [
        [0.0, 0.0, 0.0],
        [10.0, 20.0, 30.0],
        [3.0, 4.0, 0.0],
        [-7.5, 2.25, -1.75],
        [1.0e-9, -2.5e-9, 3.5e-9],
        [1.0e150, -1.0e150, 1.0e150],
        [42.0, 42.0, 42.0],
        [0.0, 0.0, 0.0],
        [-0.0, -0.0, -0.0],
        [1.0, 2.0, 3.0],
        [0.0, 100.0, 0.0],
    ];
    for anchor in anchors {
        let c_sk = unsafe { winch_stepper_alloc(anchor[0], anchor[1], anchor[2]) };
        assert!(!c_sk.is_null(), "winch_stepper_alloc returned null");
        let r = make_winch(anchor[0], anchor[1], anchor[2]);
        for coord in coords {
            let c = unsafe {
                itersolve_calc_position_from_coord(c_sk, coord[0], coord[1], coord[2])
            };
            let rv = r.sk.calc_position_from_coord(coord[0], coord[1], coord[2]);
            assert_eq!(
                c.to_bits(),
                rv.to_bits(),
                "anchor={anchor:?} coord={coord:?}: C={c:?} Rust={rv:?}"
            );
        }
        unsafe { free(c_sk as *mut c_void) };
    }
}


// ---------------------------------------------------------------------------
// `itersolve_set_position` / `itersolve_get_commanded_pos`.
// ---------------------------------------------------------------------------

#[test]
fn set_position_and_commanded_pos_match() {
    chelper_ref::link();
    let anchor = [12.0, -4.0, 9.0];
    let c_sk = unsafe { winch_stepper_alloc(anchor[0], anchor[1], anchor[2]) };
    assert!(!c_sk.is_null());
    let mut r = make_winch(anchor[0], anchor[1], anchor[2]);
    for pos in [
        [0.0, 0.0, 0.0],
        [12.0, -4.0, 9.0],
        [15.0, 0.0, 9.0],
        [-3.0, 44.0, -2.0],
        [1.0e-6, 2.0e-6, 3.0e-6],
    ] {
        unsafe { itersolve_set_position(c_sk, pos[0], pos[1], pos[2]) };
        r.sk.set_position(pos[0], pos[1], pos[2]);
        let c = unsafe { itersolve_get_commanded_pos(c_sk) };
        assert_eq!(
            c.to_bits(),
            r.sk.get_commanded_pos().to_bits(),
            "pos={pos:?}: C={c:?} Rust={:?}",
            r.sk.get_commanded_pos()
        );
        // The commanded position is the distance to the anchor.
        assert_eq!(
            c.to_bits(),
            r.sk.calc_position_from_coord(pos[0], pos[1], pos[2]).to_bits(),
            "pos={pos:?}: commanded is not the callback value"
        );
    }
    unsafe { free(c_sk as *mut c_void) };
}

// ---------------------------------------------------------------------------
// `itersolve_is_active_axis`.
// ---------------------------------------------------------------------------

#[test]
fn is_active_axis_matches() {
    chelper_ref::link();
    let c_sk = unsafe { winch_stepper_alloc(1.0, 2.0, 3.0) };
    assert!(!c_sk.is_null());
    let r = make_winch(1.0, 2.0, 3.0);
    for axis in [b'x', b'y', b'z', b'w', b'E', b'v', b'{', 0u8, 0x7f, 0xff] {
        let c = unsafe { itersolve_is_active_axis(c_sk, axis as i8) };
        let rv = r.sk.is_active_axis(axis);
        assert_eq!(c, rv, "axis={axis:?}: C={c} Rust={rv}");
        let want = i32::from(matches!(axis, b'x' | b'y' | b'z'));
        assert_eq!(rv, want, "axis={axis:?} active mismatch");
    }
    unsafe { free(c_sk as *mut c_void) };
}

// ---------------------------------------------------------------------------
// `itersolve_generate_steps` differential over trapq moves.
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
    Append(MoveParams),
    GenerateSteps(f64),
    CheckActive(f64),
    Finalize(f64, f64),
}

struct Pair {
    rust: WinchStepper,
    c_sk: *mut RefWinchStepper,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
    rust_tq: Arc<Mutex<Trapq>>,
    c_tq: *mut CTrapq,
}

impl Pair {
    fn new(anchor: [f64; 3], step_dist: f64) -> Pair {
        chelper_ref::link();
        let c_sk = unsafe { winch_stepper_alloc(anchor[0], anchor[1], anchor[2]) };
        assert!(!c_sk.is_null(), "winch_stepper_alloc returned null");
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "trapq_alloc returned null");
        let rust_tq = Arc::new(Mutex::new(Trapq::new()));
        let ref_sc = RefSc::new();
        unsafe {
            stepcompress_fill(ref_sc.sc, 7, 3000, 24, 23);
            stepcompress_set_time(ref_sc.sc, 0.0, 1_000_000.0);
        }
        let mut rust_sc = Stepcompress::new();
        rust_sc.fill(7, 3000, 24, 23);
        rust_sc.set_time(0.0, 1_000_000.0);
        let mut pair = Pair {
            rust: make_winch(anchor[0], anchor[1], anchor[2]),
            c_sk,
            rust_sc,
            ref_sc,
            rust_tq,
            c_tq,
        };
        unsafe { itersolve_set_trapq(pair.c_sk, pair.c_tq, step_dist) };
        pair.rust
            .sk
            .set_trapq(Some(pair.rust_tq.clone()), step_dist);
        pair
    }

    fn set_position(&mut self, x: f64, y: f64, z: f64) {
        unsafe { itersolve_set_position(self.c_sk, x, y, z) };
        self.rust.sk.set_position(x, y, z);
    }

    fn compare_sk(&self, ctx: &str) {
        let c = unsafe { &*self.c_sk };
        let r = &self.rust.sk;
        for (name, cf, rf) in [
            ("commanded_pos", c.core.commanded_pos, r.commanded_pos),
            ("step_dist", c.core.step_dist, r.step_dist),
            ("last_flush_time", c.core.last_flush_time, r.last_flush_time),
            ("last_move_time", c.core.last_move_time, r.last_move_time),
            (
                "gen_steps_pre_active",
                c.core.gen_steps_pre_active,
                r.gen_steps_pre_active,
            ),
            (
                "gen_steps_post_active",
                c.core.gen_steps_post_active,
                r.gen_steps_post_active,
            ),
        ] {
            assert_eq!(
                cf.to_bits(),
                rf.to_bits(),
                "{ctx}: {name} C={cf:?} Rust={rf:?}"
            );
        }
        assert_eq!(
            c.core.active_flags, r.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.core.active_flags, r.active_flags
        );
        assert_eq!(
            c.anchor.x.to_bits(),
            self.rust.anchor.x.to_bits(),
            "{ctx}: anchor.x"
        );
        assert_eq!(
            c.anchor.y.to_bits(),
            self.rust.anchor.y.to_bits(),
            "{ctx}: anchor.y"
        );
        assert_eq!(
            c.anchor.z.to_bits(),
            self.rust.anchor.z.to_bits(),
            "{ctx}: anchor.z"
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

fn run_op(pair: &mut Pair, op: &Op, ctx: &str) {
    match *op {
        Op::SetGenActive { pre, post } => {
            pair.rust.sk.gen_steps_pre_active = pre;
            pair.rust.sk.gen_steps_post_active = post;
            let c = unsafe { &mut *pair.c_sk };
            c.core.gen_steps_pre_active = pre;
            c.core.gen_steps_post_active = post;
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
            let r = pair.rust.sk.generate_steps(&mut pair.rust_sc, flush_time);
            assert_eq!(c, r, "{ctx}: generate_steps({flush_time}) return diverged");
        }
        Op::CheckActive(flush_time) => {
            let c = unsafe { itersolve_check_active(pair.c_sk, flush_time) };
            let r = pair.rust.sk.check_active(flush_time);
            assert_eq!(
                c.to_bits(),
                r.to_bits(),
                "{ctx}: check_active({flush_time}) C={c:?} Rust={r:?}"
            );
        }
        Op::Finalize(pt, cht) => {
            unsafe { trapq_finalize_moves(pair.c_tq, pt, cht) };
            pair.rust_tq.lock().finalize_moves(pt, cht);
        }
    }
}

fn run_stream(ctx: &str, anchor: [f64; 3], step_dist: f64, ops: &[Op]) {
    let mut pair = Pair::new(anchor, step_dist);
    pair.compare_all(&format!("{ctx}#init"));
    for (i, op) in ops.iter().enumerate() {
        run_op(&mut pair, op, &format!("{ctx}#{i}"));
        pair.compare_all(&format!("{ctx}#{i}"));
    }
}

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

/// A move straight toward the anchor: the stepper distance shrinks
/// monotonically, forward flushing inside and past the move.
#[test]
fn generate_steps_toward_anchor() {
    let anchor = [0.0, 0.0, 0.0];
    let mut pair = Pair::new(anchor, 0.01);
    pair.set_position(10.0, 0.0, 0.0);
    pair.compare_all("toward#init");
    run_op(
        &mut pair,
        &Op::Append(cruise(1.0, [10.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 2.0, 4.0)),
        "toward#0",
    );
    pair.compare_all("toward#0");
    run_op(&mut pair, &Op::GenerateSteps(2.0), "toward#1");
    pair.compare_all("toward#1");
    run_op(&mut pair, &Op::CheckActive(2.0), "toward#2");
    pair.compare_all("toward#2");
    run_op(&mut pair, &Op::GenerateSteps(10.0), "toward#3");
    pair.compare_all("toward#3");
}

/// A move away from the anchor, then a reversal through it (the
/// distance reaches its minimum mid-move: solver direction change),
/// plus an inactive move and pre/post-active windows.
#[test]
fn generate_steps_reversal_and_inactive() {
    let ops = [
        Op::SetGenActive { pre: 0.05, post: 0.05 },
        Op::Append(cruise(1.0, [2.0, 0.0, 0.0], [1.0, 0.0, 0.0], 2.0, 2.0)),
        Op::Append(cruise(3.0, [6.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 2.0, 6.0)),
        Op::Append(cruise(10.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0, 1.0)),
        Op::Append(cruise(11.5, [0.0, 4.0, 0.0], [0.0, -1.0, 0.0], 1.5, 2.0)),
        Op::GenerateSteps(1.5),
        Op::CheckActive(1.5),
        Op::GenerateSteps(4.0),
        Op::GenerateSteps(9.0),
        Op::GenerateSteps(20.0),
        Op::Finalize(20.0, 5.0),
        Op::GenerateSteps(25.0),
    ];
    let mut pair = Pair::new([0.0, 0.0, 0.0], 0.005);
    pair.set_position(2.0, 0.0, 0.0);
    pair.compare_all("reversal#init");
    for (i, op) in ops.iter().enumerate() {
        run_op(&mut pair, op, &format!("reversal#{i}"));
        pair.compare_all(&format!("reversal#{i}"));
    }
}

/// Diagonal moves with an off-origin anchor, an accelerated move
/// (nonzero `accel`/`accel_t`), and a time gap between moves.
#[test]
fn generate_steps_diagonal_accel_gap() {
    let ops = [
        Op::Append(cruise(1.0, [3.0, 4.0, 1.0], [-0.6, -0.8, 0.0], 1.0, 3.0)),
        Op::Append(MoveParams {
            print_time: 7.0,
            accel_t: 1.0,
            cruise_t: 2.0,
            decel_t: 1.0,
            start_pos: [1.2, 1.6, 1.0],
            axes_r: [-0.6, -0.8, 0.0],
            start_v: 0.0,
            cruise_v: 1.0,
            accel: 1.0,
        }),
        Op::GenerateSteps(2.0),
        Op::GenerateSteps(6.5),
        Op::GenerateSteps(12.0),
    ];
    run_stream("diagonal", [0.0, 0.0, 1.0], 0.001, &ops);
}

/// A full pseudo-random callback sweep: deterministic LCG anchors and
/// coordinates through `itersolve_calc_position_from_coord`.
#[test]
fn calc_position_pseudo_random() {
    chelper_ref::link();
    let mut state: u64 = 0x2545_F491_4F6C_DD1D;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let u = (state >> 11) as f64 / (1u64 << 53) as f64;
        // Spread over [-500, 500].
        u * 1000.0 - 500.0
    };
    for _ in 0..512 {
        let anchor = [next(), next(), next()];
        let coord = [next(), next(), next()];
        let c_sk = unsafe { winch_stepper_alloc(anchor[0], anchor[1], anchor[2]) };
        assert!(!c_sk.is_null());
        let r = make_winch(anchor[0], anchor[1], anchor[2]);
        let c = unsafe {
            itersolve_calc_position_from_coord(c_sk, coord[0], coord[1], coord[2])
        };
        let rv = r.sk.calc_position_from_coord(coord[0], coord[1], coord[2]);
        assert_eq!(
            c.to_bits(),
            rv.to_bits(),
            "anchor={anchor:?} coord={coord:?}: C={c:?} Rust={rv:?}"
        );
        unsafe { free(c_sk as *mut c_void) };
    }
}
