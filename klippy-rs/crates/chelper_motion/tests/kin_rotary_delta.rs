//! Differential parity tests for `chelper_motion::kin_rotary_delta`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - all fields set by `rotary_delta_stepper_alloc` (the stored
//!   `cos`/`sin`, `shoulder_radius`/`shoulder_height`, the squared arm
//!   lengths, and the `active_flags` / zeroed solver state), and
//! - the angle returned by `itersolve_calc_position_from_coord` over a
//!   wide coordinate grid, and
//! - the step stream produced by `itersolve_generate_steps` over
//!   appended trapezoid moves, including the emitted stepcompress
//!   messages/history and the mutated `stepper_kinematics` fields.
//!
//! The C side is allocated through the reference
//! `rotary_delta_stepper_alloc` (from `kin_rotary_delta.c`); the Rust
//! side uses `chelper_motion::kin_rotary_delta::rotary_delta_stepper_alloc`.
//! The solver entry points are exercised through `itersolve` exactly as
//! in `tests/itersolve.rs`.
//!
//! `rotary_two_arm_calc` is `static inline` in the C source and not
//! callable from outside the translation unit; it is covered on both
//! sides through `calc_position_from_coord` / `generate_steps`, which
//! evaluate every branch of it (the division by `dy`, the quadratic
//! root, and the final `atan2`).
//! Degenerate inputs (offsets making `sj y` zero, coordinates outside
//! the arm reach) are included on purpose: the reference divides
//! `1. / dy` and takes `sqrt` of a possibly negative value, and both
//! sides must produce the same bits, NaN included.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_rotary_delta`
use chelper_motion::itersolve::{StepperKinematics, AF_X, AF_Y, AF_Z};
use chelper_motion::kin_rotary_delta::{
    rotary_delta_stepper_alloc as rust_rotary_delta_stepper_alloc,
    rotary_stepper_calc_position, RotaryStepper,
};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::Trapq;
use core::ffi::c_void;

use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// stepcompress.h / msgblock.h / list.h / kin_rotary_delta.c).
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

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout, as in the
/// `itersolve` tests.
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

/// `struct rotary_stepper` (kin_rotary_delta.c): the embedded stepper
/// kinematics followed by the geometry fields in C order.
#[repr(C)]
struct RefRotaryStepper {
    sk: RefSk,
    cos: f64,
    sin: f64,
    shoulder_radius: f64,
    shoulder_height: f64,
    upper_arm2: f64,
    lower_arm2: f64,
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
    fn rotary_delta_stepper_alloc(
        shoulder_radius: f64,
        shoulder_height: f64,
        angle: f64,
        upper_arm: f64,
        lower_arm: f64,
    ) -> *mut RefSk;
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
// Rust/C stepper pair.
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

struct Pair {
    rust_stepper: Box<RotaryStepper>,
    c_stepper: *mut RefSk,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
    rust_tq: Arc<Mutex<Trapq>>,
    c_tq: *mut CTrapq,
}

impl Pair {
    fn new(geometry: [f64; 5], step_dist: f64) -> Pair {
        chelper_ref::link();
        let rust_stepper = rust_rotary_delta_stepper_alloc(
            geometry[0],
            geometry[1],
            geometry[2],
            geometry[3],
            geometry[4],
        );
        let c_stepper = unsafe {
            rotary_delta_stepper_alloc(
                geometry[0],
                geometry[1],
                geometry[2],
                geometry[3],
                geometry[4],
            )
        };
        assert!(
            !c_stepper.is_null(),
            "rotary_delta_stepper_alloc returned null"
        );
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
            rust_stepper,
            c_stepper,
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
            unsafe { itersolve_set_trapq(self.c_stepper, self.c_tq, step_dist) };
            self.rust_stepper
                .sk
                .set_trapq(Some(self.rust_tq.clone()), step_dist);
        } else {
            unsafe { itersolve_set_trapq(self.c_stepper, core::ptr::null_mut(), step_dist) };
            self.rust_stepper.sk.set_trapq(None, step_dist);
        }
    }

    fn rust_sk(&self) -> &StepperKinematics {
        &self.rust_stepper.sk
    }

    fn rust_sk_mut(&mut self) -> &mut StepperKinematics {
        &mut self.rust_stepper.sk
    }

    /// Compare the geometry fields plus the solver state both allocators set.
    fn compare_alloc(&self, ctx: &str) {
        let c = unsafe { &*(self.c_stepper as *const RefRotaryStepper) };
        let r = &*self.rust_stepper;
        assert_eq!(c.cos.to_bits(), r.cos.to_bits(), "{ctx}: cos");
        assert_eq!(c.sin.to_bits(), r.sin.to_bits(), "{ctx}: sin");
        assert_eq!(
            c.shoulder_radius.to_bits(),
            r.shoulder_radius.to_bits(),
            "{ctx}: shoulder_radius"
        );
        assert_eq!(
            c.shoulder_height.to_bits(),
            r.shoulder_height.to_bits(),
            "{ctx}: shoulder_height"
        );
        assert_eq!(
            c.upper_arm2.to_bits(),
            r.upper_arm2.to_bits(),
            "{ctx}: upper_arm2"
        );
        assert_eq!(
            c.lower_arm2.to_bits(),
            r.lower_arm2.to_bits(),
            "{ctx}: lower_arm2"
        );
        let c_sk = &c.sk;
        let r_sk = &r.sk;
        assert_eq!(c_sk.step_dist.to_bits(), r_sk.step_dist.to_bits(), "{ctx}: step_dist");
        assert_eq!(
            c_sk.commanded_pos.to_bits(),
            r_sk.commanded_pos.to_bits(),
            "{ctx}: commanded_pos"
        );
        assert_eq!(
            c_sk.last_flush_time.to_bits(),
            r_sk.last_flush_time.to_bits(),
            "{ctx}: last_flush_time"
        );
        assert_eq!(
            c_sk.last_move_time.to_bits(),
            r_sk.last_move_time.to_bits(),
            "{ctx}: last_move_time"
        );
        assert_eq!(c_sk.active_flags, r_sk.active_flags, "{ctx}: active_flags");
        assert_eq!(
            c_sk.gen_steps_pre_active.to_bits(),
            r_sk.gen_steps_pre_active.to_bits(),
            "{ctx}: gen_steps_pre_active"
        );
        assert_eq!(
            c_sk.gen_steps_post_active.to_bits(),
            r_sk.gen_steps_post_active.to_bits(),
            "{ctx}: gen_steps_post_active"
        );
        assert_eq!(
            r_sk.active_flags,
            AF_X | AF_Y | AF_Z,
            "{ctx}: Rust active_flags must be AF_X|AF_Y|AF_Z"
        );
    }

    fn compare_sk(&self, ctx: &str) {
        let c = unsafe { &*self.c_stepper };
        let r = self.rust_sk();
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
        assert_eq!(
            unsafe { itersolve_get_commanded_pos(self.c_stepper) }.to_bits(),
            self.rust_sk().get_commanded_pos().to_bits(),
            "{ctx}: get_commanded_pos diverged"
        );
        assert_eq!(
            unsafe { itersolve_get_gen_steps_pre_active(self.c_stepper) }.to_bits(),
            self.rust_sk().get_gen_steps_pre_active().to_bits(),
            "{ctx}: get_gen_steps_pre_active diverged"
        );
        assert_eq!(
            unsafe { itersolve_get_gen_steps_post_active(self.c_stepper) }.to_bits(),
            self.rust_sk().get_gen_steps_post_active().to_bits(),
            "{ctx}: get_gen_steps_post_active diverged"
        );
    }

    fn calc_from_coord(&self, ctx: &str, x: f64, y: f64, z: f64) {
        let c = unsafe { itersolve_calc_position_from_coord(self.c_stepper, x, y, z) };
        let r = self.rust_sk().calc_position_from_coord(x, y, z);
        assert_eq!(
            c.to_bits(),
            r.to_bits(),
            "{ctx}: calc_position_from_coord({x}, {y}, {z}) C={c:?} Rust={r:?}"
        );
    }

    fn set_position(&mut self, ctx: &str, x: f64, y: f64, z: f64) {
        unsafe { itersolve_set_position(self.c_stepper, x, y, z) };
        self.rust_sk_mut().set_position(x, y, z);
        let c = unsafe { itersolve_get_commanded_pos(self.c_stepper) };
        let r = self.rust_sk().get_commanded_pos();
        assert_eq!(
            c.to_bits(),
            r.to_bits(),
            "{ctx}: set_position({x}, {y}, {z}) C={c:?} Rust={r:?}"
        );
    }

    fn append(&mut self, p: &MoveParams) {
        unsafe {
            trapq_append(
                self.c_tq,
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
        self.rust_tq.lock().append(
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

    fn generate_steps(&mut self, ctx: &str, flush_time: f64) {
        let c = unsafe { itersolve_generate_steps(self.c_stepper, self.ref_sc.sc, flush_time) };
        let r = self
            .rust_stepper
            .sk
            .generate_steps(&mut self.rust_sc, flush_time);
        assert_eq!(c, r, "{ctx}: generate_steps({flush_time}) return diverged");
        self.compare_all(ctx);
    }

    fn check_active(&mut self, ctx: &str, flush_time: f64) {
        let c = unsafe { itersolve_check_active(self.c_stepper, flush_time) };
        let r = self
            .rust_stepper
            .sk
            .check_active(flush_time);
        assert_eq!(
            c.to_bits(),
            r.to_bits(),
            "{ctx}: check_active({flush_time}) C={c:?} Rust={r:?}"
        );
    }

    fn is_active_axis(&self, ctx: &str, axis: u8) {
        let c = unsafe { itersolve_is_active_axis(self.c_stepper, axis as i8) };
        let r = self.rust_sk().is_active_axis(axis);
        assert_eq!(c, r, "{ctx}: is_active_axis({axis}) diverged");
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe {
            free(self.c_stepper as *mut c_void);
            trapq_free(self.c_tq);
        }
    }
}

// ---------------------------------------------------------------------------
// Allocation parity.
// ---------------------------------------------------------------------------

#[test]
fn alloc_fields_match_c() {
    chelper_ref::link();
    let geometries: [&[f64; 5]; 5] = [
        // shoulder_radius, shoulder_height, angle, upper_arm, lower_arm
        &[35.0, 100.0, 0.0, 40.0, 90.0],
        &[35.0, 100.0, 0.5, 40.0, 90.0],
        &[25.0, 120.0, -1.0, 30.0, 80.0],
        &[0.0, 0.0, 0.0, 1.0, 2.0],
        &[120.0, 80.0, core::f64::consts::PI, 60.0, 150.0],
    ];
    for (i, geometry) in geometries.iter().enumerate() {
        let pair = Pair::new(**geometry, 0.0125);
        pair.compare_alloc(&format!("geometry-{i}"));
    }
}

// ---------------------------------------------------------------------------
// calc_position parity over a coordinate grid.
// ---------------------------------------------------------------------------

#[test]
fn calc_position_from_coord_matches_c() {
    chelper_ref::link();
    let pair = Pair::new([35.0, 100.0, 0.0, 40.0, 90.0], 0.0125);
    pair.compare_alloc("alloc");
    // Reachable region around the origin: shoulder-to-effector distance
    // between |upper-lower| and upper+lower, plus on/off-boundary points.
    // z == shoulder_height makes sj y zero (1. / dy overflows to inf and
    // the angle comes out NaN) and (35, 0, 100) makes both offsets zero;
    // both sides must agree bit for bit, NaN included.
    let points: &[[f64; 3]] = &[
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 3.0],
        [-5.0, 5.0, -2.0],
        [15.0, -10.0, 7.5],
        [-15.0, -15.0, 5.0],
        [20.0, 15.0, -8.0],
        [0.0, 0.0, 10.0],
        [10.0, -5.0, 0.0],
        [0.0, 0.0, 100.0],
        [35.0, 0.0, 100.0],
        [100.0, 0.0, 0.0],
        [-100.0, 50.0, 100.0],
        [-50.0, -50.0, -50.0],
    ];
    for (i, point) in points.iter().enumerate() {
        pair.calc_from_coord(&format!("point-{i}"), point[0], point[1], point[2]);
    }
    // A dense sweep across the workspace for the same geometry.
    let mut n = 0;
    for &x in &[-20.0, -10.0, 0.0, 10.0, 20.0] {
        for &y in &[-20.0, -10.0, 0.0, 10.0, 20.0] {
            for &z in &[-10.0, -5.0, 0.0, 5.0, 10.0] {
                pair.calc_from_coord(&format!("sweep-{n}"), x, y, z);
                n += 1;
            }
        }
    }
    // A rotated shoulder (angle = 0.5) must agree as well.
    let rotated = Pair::new([35.0, 100.0, 0.5, 40.0, 90.0], 0.0125);
    rotated.compare_alloc("rotated-alloc");
    let mut n = 0;
    for &x in &[-10.0, 0.0, 10.0] {
        for &y in &[-10.0, 0.0, 10.0] {
            for &z in &[-5.0, 0.0, 5.0, 100.0] {
                rotated.calc_from_coord(&format!("rotated-{n}"), x, y, z);
                n += 1;
            }
        }
    }
    // A different geometry entirely.
    let other = Pair::new([25.0, 120.0, -1.0, 30.0, 80.0], 0.0125);
    other.compare_alloc("other-alloc");
    for &x in &[-8.0, 0.0, 8.0] {
        for &y in &[-8.0, 0.0, 8.0] {
            for &z in &[-4.0, 0.0, 4.0, 120.0] {
                other.calc_from_coord("other", x, y, z);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Step generation parity.
// ---------------------------------------------------------------------------

#[test]
fn generate_steps_matches_c() {
    chelper_ref::link();
    // Axis-aligned cruises and a trapezoid, with flushes inside and past
    // each move, a reversal, and a rotated shoulder geometry.
    let mut pair = Pair::new([35.0, 100.0, 0.0, 40.0, 90.0], 0.0125);
    pair.compare_alloc("steps-alloc");
    pair.compare_all("steps-init");
    pair.set_position("steps-init", 0.0, 0.0, 0.0);
    pair.compare_all("steps-setpos");

    pair.append(&cruise(1.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, 1.0));
    pair.check_active("steps-x", 10.0);
    pair.is_active_axis("steps-x", b'x');
    pair.is_active_axis("steps-x", b'y');
    pair.is_active_axis("steps-x", b'z');
    pair.generate_steps("steps-x-mid", 1.5);
    pair.generate_steps("steps-x-end", 2.5);

    pair.append(&cruise(2.0, [20.0, 0.0, 0.0], [0.0, 1.0, 0.0], 15.0, 1.0));
    pair.generate_steps("steps-y", 3.5);

    // Reversal: back along X from +20 to +5.
    pair.append(&cruise(3.0, [20.0, 15.0, 0.0], [-1.0, 0.0, 0.0], 15.0, 1.0));
    pair.generate_steps("steps-rev", 4.5);

    // Trapezoid: accel/cruise/decel along Z from the previous end point.
    pair.append(&trapezoid(
        4.0,
        [5.0, 15.0, 0.0],
        [0.0, 0.0, 1.0],
        0.25,
        0.5,
        0.25,
        0.0,
        10.0,
        80.0,
    ));
    pair.generate_steps("steps-z", 5.5);
    pair.generate_steps("steps-final", 10.0);
    pair.set_position("steps-final", 5.0, 15.0, 10.0);
    pair.compare_all("steps-final");

    // Rotated shoulder, smaller step distance.
    let mut pair = Pair::new([35.0, 100.0, 0.5, 40.0, 90.0], 0.00625);
    pair.compare_alloc("rot-alloc");
    pair.set_position("rot-init", 0.0, 0.0, 0.0);
    pair.append(&cruise(1.0, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 10.0, 1.0));
    pair.check_active("rot", 10.0);
    pair.generate_steps("rot-mid", 1.25);
    pair.generate_steps("rot-end", 3.0);
    pair.append(&trapezoid(
        2.0,
        [0.0, 10.0, 0.0],
        [1.0, 0.0, 0.0],
        0.25,
        0.5,
        0.25,
        0.0,
        10.0,
        100.0,
    ));
    pair.generate_steps("rot-trap", 4.0);
    pair.compare_all("rot-final");
}

// ---------------------------------------------------------------------------
// The Rust callback is registered by the allocator.
// ---------------------------------------------------------------------------

#[test]
fn callback_is_registered() {
    chelper_ref::link();
    let stepper = rust_rotary_delta_stepper_alloc(35.0, 100.0, 0.5, 40.0, 90.0);
    assert_eq!(stepper.cos, kos::libm::cos(0.5));
    assert_eq!(stepper.sin, kos::libm::sin(0.5));
    assert_eq!(stepper.upper_arm2, 1600.0);
    assert_eq!(stepper.lower_arm2, 8100.0);
    assert_eq!(stepper.sk.active_flags, AF_X | AF_Y | AF_Z);
    assert_eq!(stepper.sk.commanded_pos, 0.0);
    assert_eq!(stepper.sk.step_dist, 0.0);
    assert_eq!(stepper.sk.last_flush_time, 0.0);
    assert_eq!(stepper.sk.last_move_time, 0.0);
    let cb: fn(&StepperKinematics, &chelper_motion::trapq::Move, Option<usize>, f64) -> f64 =
        stepper.sk.calc_position_cb;
    assert!(core::ptr::fn_addr_eq(
        cb,
        rotary_stepper_calc_position as fn(&StepperKinematics, &chelper_motion::trapq::Move, Option<usize>, f64) -> f64
    ));
}
