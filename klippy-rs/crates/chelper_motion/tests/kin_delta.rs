//! Differential parity tests for `chelper_motion::kin_delta`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`, with NaN-vs-NaN
//! accepted where the radicand is negative and `sqrt` returns NaN on both
//! sides):
//!
//! - the fields set by `delta_stepper_alloc` (`arm2`, `tower_x`,
//!   `tower_y`, `active_flags`, and the zeroed solver state), and
//! - `itersolve_calc_position_from_coord` through the delta callback
//!   (the C `__visible` entry point that reaches
//!   `delta_stepper_calc_position`), and
//! - `itersolve_set_position` / `itersolve_get_commanded_pos`,
//!   `itersolve_is_active_axis`, `itersolve_check_active`, and
//! - `itersolve_generate_steps` over `trapq_append` streams: the return
//!   code, the solver state, the step-command history from
//!   `stepcompress_extract_old`, and the emitted MCU message bytes. The
//!   solver evaluates the delta callback at real move times, so these
//!   streams pin the callback on moving coordinates, which
//!   `calc_position_from_coord` (a synthetic zero-velocity move) cannot.
//!
//! There are no recordings under `tests/fixtures/recordings/kin_delta/`,
//! so the reference C library is the oracle for this C unit.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_delta`
use chelper_motion::itersolve::StepperKinematics;
use chelper_motion::kin_delta::{delta_stepper_alloc, DeltaStepper};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{Move, Trapq};

use core::ffi::c_void;
use std::cell::RefCell;
use std::rc::Rc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// stepcompress.h / msgblock.h / list.h / kin_delta.c).
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
/// read back the state the C solver mutates.
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

/// `struct delta_stepper` (kin_delta.c): the embedded `sk` followed by
/// `arm2`, `tower_x`, `tower_y`.
#[repr(C)]
struct RefDelta {
    sk: RefSk,
    arm2: f64,
    tower_x: f64,
    tower_y: f64,
}

enum CTrapq {}

unsafe extern "C" {
    #[link_name = "delta_stepper_alloc"]
    fn c_delta_stepper_alloc(arm2: f64, tower_x: f64, tower_y: f64) -> *mut RefDelta;
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
    fn itersolve_generate_steps(sk: *mut RefSk, sc: *mut c_void, flush_time: f64) -> i32;
    fn itersolve_check_active(sk: *mut RefSk, flush_time: f64) -> f64;
    fn itersolve_is_active_axis(sk: *mut RefSk, axis: i8) -> i32;
    fn itersolve_set_trapq(sk: *mut RefSk, tq: *mut CTrapq, step_dist: f64);
    fn itersolve_get_trapq(sk: *mut RefSk) -> *mut CTrapq;
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
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
// Comparison helper: bit equality, with NaN compared only by kind (the
// NaN payload of `sqrt` of a negative radicand is not part of the
// contract, and both sides produce NaN there).
// ---------------------------------------------------------------------------

fn assert_f64_eq(ctx: &str, c: f64, r: f64) {
    if c.is_nan() || r.is_nan() {
        assert!(
            c.is_nan() && r.is_nan(),
            "{ctx}: NaN mismatch C={c:?} Rust={r:?}"
        );
        return;
    }
    assert_eq!(
        c.to_bits(),
        r.to_bits(),
        "{ctx}: C={c:?} Rust={r:?}"
    );
}

// ---------------------------------------------------------------------------
// C/Rust pair.
// ---------------------------------------------------------------------------

struct Pair {
    rust: Box<DeltaStepper>,
    c_delta: *mut RefDelta,
    rust_tq: Rc<RefCell<Trapq>>,
    c_tq: *mut CTrapq,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
}

impl Pair {
    fn new(arm2: f64, tower_x: f64, tower_y: f64, step_dist: f64) -> Pair {
        chelper_ref::link();
        let rust = delta_stepper_alloc(arm2, tower_x, tower_y);
        let c_delta = unsafe { c_delta_stepper_alloc(arm2, tower_x, tower_y) };
        assert!(!c_delta.is_null(), "delta_stepper_alloc returned null");
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "trapq_alloc returned null");
        let rust_tq = Rc::new(RefCell::new(Trapq::new()));
        let ref_sc = RefSc::new();
        unsafe {
            stepcompress_fill(ref_sc.sc, 7, 3000, 24, 23);
            stepcompress_set_time(ref_sc.sc, 0.0, 1_000_000.0);
            trapq_check_sentinels(c_tq);
        }
        let mut rust_sc = Stepcompress::new();
        rust_sc.fill(7, 3000, 24, 23);
        rust_sc.set_time(0.0, 1_000_000.0);
        let mut pair = Pair {
            rust,
            c_delta,
            rust_tq,
            c_tq,
            rust_sc,
            ref_sc,
        };
        pair.set_trapq(true, step_dist);
        pair.set_position(0.0, 0.0, 0.0);
        pair
    }

    fn c_sk(&self) -> *mut RefSk {
        unsafe { &mut (*self.c_delta).sk as *mut RefSk }
    }

    fn set_trapq(&mut self, some: bool, step_dist: f64) {
        if some {
            unsafe { trapq_check_sentinels(self.c_tq) };
            unsafe {
                itersolve_set_trapq(self.c_sk(), self.c_tq, step_dist)
            };
            self.rust.sk.set_trapq(Some(self.rust_tq.clone()), step_dist);
        } else {
            unsafe {
                itersolve_set_trapq(self.c_sk(), core::ptr::null_mut(), step_dist)
            };
            self.rust.sk.set_trapq(None, step_dist);
        }
    }

    fn set_position(&mut self, x: f64, y: f64, z: f64) {
        unsafe { itersolve_set_position(self.c_sk(), x, y, z) };
        self.rust.sk.set_position(x, y, z);
    }

    fn calc_position_from_coord(&self, x: f64, y: f64, z: f64) -> f64 {
        unsafe { itersolve_calc_position_from_coord(self.c_sk(), x, y, z) }
    }

    fn compare_sk(&self, ctx: &str) {
        let c = unsafe { &(*self.c_delta).sk };
        let r = &self.rust.sk;
        assert_f64_eq(&format!("{ctx}: commanded_pos"), c.commanded_pos, r.commanded_pos);
        assert_f64_eq(&format!("{ctx}: step_dist"), c.step_dist, r.step_dist);
        assert_f64_eq(
            &format!("{ctx}: last_flush_time"),
            c.last_flush_time,
            r.last_flush_time,
        );
        assert_f64_eq(
            &format!("{ctx}: last_move_time"),
            c.last_move_time,
            r.last_move_time,
        );
        assert_eq!(
            c.active_flags, r.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.active_flags, r.active_flags
        );
        assert_f64_eq(
            &format!("{ctx}: gen_steps_pre_active"),
            c.gen_steps_pre_active,
            r.gen_steps_pre_active,
        );
        assert_f64_eq(
            &format!("{ctx}: gen_steps_post_active"),
            c.gen_steps_post_active,
            r.gen_steps_post_active,
        );
        assert_f64_eq(&format!("{ctx}: arm2"), unsafe { (*self.c_delta).arm2 }, self.rust.arm2);
        assert_f64_eq(
            &format!("{ctx}: tower_x"),
            unsafe { (*self.c_delta).tower_x },
            self.rust.tower_x,
        );
        assert_f64_eq(
            &format!("{ctx}: tower_y"),
            unsafe { (*self.c_delta).tower_y },
            self.rust.tower_y,
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
        assert_f64_eq(
            &format!("{ctx}: get_commanded_pos"),
            unsafe { itersolve_get_commanded_pos(self.c_sk()) },
            self.rust.sk.get_commanded_pos(),
        );
        if self.rust.sk.tq.is_none() {
            assert!(
                unsafe { itersolve_get_trapq(self.c_sk()) }.is_null(),
                "{ctx}: C trapq is not null"
            );
        } else {
            assert_eq!(
                unsafe { itersolve_get_trapq(self.c_sk()) },
                self.c_tq,
                "{ctx}: C trapq mismatch"
            );
        }
    }

    fn append(&mut self, p: MoveParams) {
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
        self.rust_tq.borrow_mut().append(
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

    fn generate_steps(&mut self, flush_time: f64, ctx: &str) {
        let c = unsafe { itersolve_generate_steps(self.c_sk(), self.ref_sc.sc, flush_time) };
        let r = self
            .rust
            .sk
            .generate_steps(&mut self.rust_sc, flush_time);
        assert_eq!(
            c, r,
            "{ctx}: generate_steps({flush_time}) return diverged"
        );
        self.compare_all(ctx);
    }

    fn check_active(&mut self, flush_time: f64, ctx: &str) {
        let c = unsafe { itersolve_check_active(self.c_sk(), flush_time) };
        let r = self.rust.sk.check_active(flush_time);
        assert_f64_eq(&format!("{ctx}: check_active({flush_time})"), c, r);
        self.compare_all(ctx);
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe {
            free(self.c_delta as *mut c_void);
            trapq_free(self.c_tq);
        }
    }
}

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

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn alloc_fields_match_c() {
    chelper_ref::link();
    // A typical tower A of a radius-100 delta with a 255 mm arm
    // (`kinematics/delta.py` towers: cos/sin of 210 degrees times the
    // radius), plus tower C and a degenerate zero tower.
    let cases: [(f64, f64, f64); 3] = [
        (255.0 * 255.0, -86.60254037844386, -50.0),
        (255.0 * 255.0, 0.0, 100.0),
        (0.0, 0.0, 0.0),
    ];
    for (i, &(arm2, tower_x, tower_y)) in cases.iter().enumerate() {
        let rust = delta_stepper_alloc(arm2, tower_x, tower_y);
        let c_delta = unsafe { c_delta_stepper_alloc(arm2, tower_x, tower_y) };
        assert!(!c_delta.is_null(), "delta_stepper_alloc returned null");
        let c = unsafe { &*c_delta };
        let ctx = format!("alloc case {i}");
        assert_f64_eq(&format!("{ctx}: arm2"), c.arm2, rust.arm2);
        assert_f64_eq(&format!("{ctx}: tower_x"), c.tower_x, rust.tower_x);
        assert_f64_eq(&format!("{ctx}: tower_y"), c.tower_y, rust.tower_y);
        let r = &rust.sk;
        assert_eq!(
            c.sk.active_flags, r.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.sk.active_flags, r.active_flags
        );
        assert_f64_eq(&format!("{ctx}: step_dist"), c.sk.step_dist, r.step_dist);
        assert_f64_eq(&format!("{ctx}: commanded_pos"), c.sk.commanded_pos, r.commanded_pos);
        assert_f64_eq(
            &format!("{ctx}: last_flush_time"),
            c.sk.last_flush_time,
            r.last_flush_time,
        );
        assert_f64_eq(
            &format!("{ctx}: last_move_time"),
            c.sk.last_move_time,
            r.last_move_time,
        );
        assert_f64_eq(
            &format!("{ctx}: gen_steps_pre_active"),
            c.sk.gen_steps_pre_active,
            r.gen_steps_pre_active,
        );
        assert_f64_eq(
            &format!("{ctx}: gen_steps_post_active"),
            c.sk.gen_steps_post_active,
            r.gen_steps_post_active,
        );
        unsafe { free(c_delta as *mut c_void) };
    }
}

#[test]
fn calc_position_from_coord_matches_c() {
    // Tower A and tower C configurations with a spread of coordinates,
    // including a coordinate on the tower axis (dx = dy = 0), negative
    // radicands (NaN on both sides), and fractional inputs.
    let configs: [(f64, f64, f64); 2] = [
        (255.0 * 255.0, -86.60254037844386, -50.0),
        (250.0 * 250.0, 0.0, 100.0),
    ];
    let coords: [(f64, f64, f64); 10] = [
        (0.0, 0.0, 0.0),
        (10.0, 20.0, 30.0),
        (-40.0, 15.0, 5.0),
        (0.0, 0.0, 100.0),
        (-86.60254037844386, -50.0, 7.5),
        (0.1, -0.2, 0.3),
        (123.456, -98.765, 43.21),
        (-200.0, 150.0, -10.0),
        (1000.0, 1000.0, 0.0),
        (0.0, 100.0, -5.0),
    ];
    for (ci, &(arm2, tower_x, tower_y)) in configs.iter().enumerate() {
        let mut pair = Pair::new(arm2, tower_x, tower_y, 0.01);
        for (i, &(x, y, z)) in coords.iter().enumerate() {
            let ctx = format!("calc case {ci}.{i} coord=({x}, {y}, {z})");
            let c = pair.calc_position_from_coord(x, y, z);
            let r = pair.rust.sk.calc_position_from_coord(x, y, z);
            assert_f64_eq(&ctx, c, r);
        }
        pair.compare_all(&format!("calc cfg {ci}"));
    }
}

#[test]
fn calc_position_from_coord_sweep_matches_c() {
    // Deterministic pseudo-random sweep over arm2/tower/coordinate
    // values, including negative radicands, compared bit for bit.
    let mut pair = Pair::new(65025.0, -86.60254037844386, -50.0, 0.005);
    let mut state: u64 = 0x1234_5678_9abc_def0;
    let mut next = move || {
        // SplitMix64, reduced to a double in [-200, 200] / [0, 70000].
        state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z = z ^ (z >> 31);
        (z % 400_000) as f64 / 1000.0 - 200.0
    };
    for i in 0..2000 {
        let arm2: f64 = 70_000.0 - next() * 350.0;
        let tower_x = next();
        let tower_y = next();
        let x = next();
        let y = next();
        let z = next() / 2.0;
        // Rebuild the pair for the new tower geometry and re-seat it at
        // the origin, then evaluate both sides.
        pair.rust = delta_stepper_alloc(arm2, tower_x, tower_y);
        unsafe { free(pair.c_delta as *mut c_void) };
        pair.c_delta = unsafe { c_delta_stepper_alloc(arm2, tower_x, tower_y) };
        assert!(!pair.c_delta.is_null(), "delta_stepper_alloc null at {i}");
        pair.set_position(0.0, 0.0, 0.0);
        let ctx = format!("sweep {i}: arm2={arm2} tower=({tower_x}, {tower_y}) coord=({x}, {y}, {z})");
        let c = pair.calc_position_from_coord(x, y, z);
        let r = pair.rust.sk.calc_position_from_coord(x, y, z);
        assert_f64_eq(&ctx, c, r);
        pair.compare_sk(&ctx);
    }
}

#[test]
fn set_position_updates_commanded_pos() {
    let mut pair = Pair::new(65025.0, -86.60254037844386, -50.0, 0.01);
    let coords: [(f64, f64, f64); 5] = [
        (0.0, 0.0, 0.0),
        (10.0, 20.0, 30.0),
        (-40.5, 15.25, 100.0),
        (0.0, -86.60254037844386, -50.0),
        (1000.0, 1000.0, 0.0),
    ];
    for (i, &(x, y, z)) in coords.iter().enumerate() {
        let ctx = format!("set_position case {i}");
        pair.set_position(x, y, z);
        pair.compare_all(&ctx);
    }
}

#[test]
fn is_active_axis_all_axes() {
    let pair = Pair::new(65025.0, 0.0, 100.0, 0.01);
    for axis in [b'x', b'y', b'z', b'w', b'a', b'\0'] {
        let c = unsafe { itersolve_is_active_axis(pair.c_sk(), axis as i8) };
        let r = pair.rust.sk.is_active_axis(axis);
        assert_eq!(c, r, "is_active_axis({axis}) C={c} Rust={r}");
    }
    // Delta steppers are active on all of X, Y and Z.
    for axis in [b'x', b'y', b'z'] {
        assert_eq!(pair.rust.sk.is_active_axis(axis), 1, "axis {axis}");
    }
}

#[test]
fn generate_steps_matches_c() {
    // One tower, a few trapezoidal streams. Each stream is generated in
    // slices with flush times inside and past each move, and
    // check_active is probed before generation.
    let mut pair = Pair::new(65025.0, -86.60254037844386, -50.0, 0.001);
    pair.check_active(0.0, "delta-gen initial");
    let moves = [
        cruise(0.0, [0.0, 0.0, 0.0], [0.6, 0.0, 0.8], 10.0, 1.0),
        MoveParams {
            print_time: 1.0,
            accel_t: 0.25,
            cruise_t: 0.5,
            decel_t: 0.25,
            start_pos: [6.0, 0.0, 8.0],
            axes_r: [0.0, -0.6, -0.8],
            start_v: 0.0,
            cruise_v: 4.0,
            accel: 16.0,
        },
        cruise(2.0, [6.0, -3.0, 4.0], [0.0, 0.0, 1.0], 5.0, 0.5),
    ];
    for mv in moves.iter() {
        pair.append(*mv);
    }
    pair.check_active(0.5, "delta-gen after append");
    let flush_times = [0.1, 0.5, 1.0, 1.5, 2.0, 2.25, 2.5, 3.0, 100.0];
    for (i, ft) in flush_times.iter().enumerate() {
        pair.generate_steps(*ft, &format!("delta-gen flush {i}"));
    }
    pair.check_active(100.0, "delta-gen final");
    pair.compare_all("delta-gen final");
}

#[test]
fn generate_steps_reverse_matches_c() {
    // Reversal: the solver's direction-change path with the delta
    // callback, plus a move with no trapq attached (generate_steps must
    // return 0 without touching the stepcompress state).
    let mut pair = Pair::new(62500.0, 0.0, 100.0, 0.0025);
    pair.append(cruise(0.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, 0.4));
    pair.append(cruise(0.4, [8.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 20.0, 0.4));
    pair.generate_steps(0.4, "delta-reverse mid");
    pair.generate_steps(0.8, "delta-reverse end");
    pair.compare_all("delta-reverse final");
    pair.set_trapq(false, 0.0025);
    pair.generate_steps(1.0, "delta-no-trapq");
    pair.set_trapq(true, 0.0025);
    pair.compare_all("delta-no-trapq final");
}

#[test]
fn callback_direct_matches_calc_from_coord() {
    // The Rust callback invoked directly on a moving coordinate must
    // agree with the same value reached through
    // calc_position_from_coord's synthetic move for the same XYZ point,
    // and the direct path must not disturb the solver state.
    let pair = Pair::new(65025.0, -86.60254037844386, -50.0, 0.01);
    let m = Move {
        print_time: 3.0,
        move_t: 2.0,
        start_v: 5.0,
        half_accel: 2.0,
        start_pos: chelper_motion::trapq::Coord::new(1.0, 2.0, 3.0),
        axes_r: chelper_motion::trapq::Coord::new(0.6, -0.48, 0.64),
    };
    for t in [0.0, 0.5, 1.0, 2.0] {
        let direct = pair.rust.calc_position(&m, t);
        let c = m.get_coord(t);
        let via = pair.rust.sk.calc_position_from_coord(c.x, c.y, c.z);
        assert_f64_eq(&format!("direct t={t}"), direct, via);
        let c_ref = pair.calc_position_from_coord(c.x, c.y, c.z);
        assert_f64_eq(&format!("direct ref t={t}"), c_ref, direct);
    }
}

#[test]
fn stepperskinematics_surface() {
    // The embedded StepperKinematics keeps the zeroed defaults of the C
    // allocation everywhere except active_flags and the callback, which
    // the solver dispatches through.
    let ds: &DeltaStepper = &delta_stepper_alloc(100.0, 3.0, 4.0);
    let sk: &StepperKinematics = &ds.sk;
    assert_f64_eq("default step_dist", sk.step_dist, 0.0);
    assert_f64_eq("default commanded_pos", sk.commanded_pos, 0.0);
    assert_f64_eq("default last_flush_time", sk.last_flush_time, 0.0);
    assert_f64_eq("default last_move_time", sk.last_move_time, 0.0);
    assert!(sk.tq.is_none());
    assert!(sk.sc.is_none());
    assert!(sk.post_cb.is_none());
    assert_f64_eq("default gen pre", sk.get_gen_steps_pre_active(), 0.0);
    assert_f64_eq("default gen post", sk.get_gen_steps_post_active(), 0.0);
    assert_f64_eq("arm2", ds.arm2, 100.0);
    assert_f64_eq("tower_x", ds.tower_x, 3.0);
    assert_f64_eq("tower_y", ds.tower_y, 4.0);
}
