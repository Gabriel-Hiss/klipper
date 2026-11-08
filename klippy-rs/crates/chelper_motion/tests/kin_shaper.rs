//! Differential parity tests for `chelper_motion::kin_shaper`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - the return codes of `input_shaper_set_sk` and
//!   `input_shaper_set_shaper_params`,
//! - the resulting `struct shaper_pulses` of every axis (pulses, `p_ind`,
//!   and the stale entries past `num_pulses`, which the C keeps after a
//!   failed reconfiguration),
//! - the embedded `StepperKinematics` of the shaper (`commanded_pos`, the
//!   `gen_steps_pre_active`/`gen_steps_post_active` windows and
//!   `active_flags`) and of the original kinematics, so the commanded
//!   position forwarding of `shaper_commanded_pos_post_fixup` is compared
//!   too,
//! - the scratch move (`start_pos` and `move_t = 2 * DUMMY_T`), and
//! - the whole step generation pipeline (`itersolve_generate_steps` on top
//!   of the shaper): return codes plus the full `Stepcompress` state
//!   (emitted MCU message bytes and clocks, the step-command history, oid
//!   and step direction), so the convolution, including its walk across
//!   neighbouring moves, is compared on the emitted steps.
//!
//! `kin_shaper.c` has no Python-level observable (it is C-only behind cffi
//! in `chelper/__init__.py`), and the recordings hold no `input_shaper_*`
//! calls, so the reference C library is the oracle for this unit (see the
//! wave context: `chelper_ref` for C units).
//!
//! Scenarios: the `set_sk` flag matrix (single axis, multi axis, no axis),
//! `set_shaper_params` validation (unknown axis, out-of-range `n`, empty
//! pulse trains, inactive axis, non-increasing times), shaped step
//! generation on X, Y, Z and the xyz variant on CoreXY, `update_sk`
//! after the original kinematics' flags change, the post-callback
//! forwarding, and a deterministic pseudo-random stream.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_shaper`
//! (the integration owner builds and runs; per the pilot rules only
//! `cargo check`/`cargo clippy` run here, via WSL with a per-checkout
//! `CARGO_TARGET_DIR`).

use chelper_motion::itersolve::{
    StepperCalcCb, StepperKinematics, AF_X, AF_Y, AF_Z,
};
use chelper_motion::kin_shaper::{
    input_shaper_alloc as rust_input_shaper_alloc,
    input_shaper_set_shaper_params as rust_input_shaper_set_shaper_params,
    input_shaper_set_sk as rust_input_shaper_set_sk,
    input_shaper_update_sk as rust_input_shaper_update_sk,
    InputShaper, DUMMY_T,
};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{move_get_coord, Move, Trapq};
use core::ffi::c_void;

use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// list.h / kin_shaper.c).
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
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

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout. The shaper's
/// `struct input_shaper` embeds it first, so a shaper pointer is also a
/// pointer to this struct.
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
    post_cb: Option<extern "C" fn(*mut RefSk)>,
}

/// `struct move` (trapq.h).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct RefMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    node: ListNode,
}

/// One pulse `{ double t, a; }` (`kin_shaper.c`).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct RefPulse {
    t: f64,
    a: f64,
}

/// `struct shaper_pulses` (`kin_shaper.c`).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct RefPulses {
    num_pulses: i32,
    p_ind: i32,
    pulses: [RefPulse; 10],
}

/// `struct input_shaper` (`kin_shaper.c`): the embedded kinematics first.
#[repr(C)]
#[allow(dead_code)]
struct RefInputShaper {
    sk: RefSk,
    orig_sk: *mut RefSk,
    m: RefMove,
    sp: [RefPulses; 3],
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
    fn corexy_stepper_alloc(kind: i8) -> *mut RefSk;
    fn itersolve_generate_steps(sk: *mut RefSk, sc: *mut c_void, flush_time: f64) -> i32;
    fn itersolve_check_active(sk: *mut RefSk, flush_time: f64) -> f64;
    fn itersolve_set_trapq(sk: *mut RefSk, tq: *mut CTrapq, step_dist: f64);
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
    fn itersolve_get_gen_steps_pre_active(sk: *mut RefSk) -> f64;
    fn itersolve_get_gen_steps_post_active(sk: *mut RefSk) -> f64;
    #[link_name = "input_shaper_alloc"]
    fn c_input_shaper_alloc() -> *mut RefSk;
    #[link_name = "input_shaper_set_sk"]
    fn c_input_shaper_set_sk(sk: *mut RefSk, orig_sk: *mut RefSk) -> i32;
    #[link_name = "input_shaper_update_sk"]
    fn c_input_shaper_update_sk(sk: *mut RefSk);
    #[link_name = "input_shaper_set_shaper_params"]
    fn c_input_shaper_set_shaper_params(
        sk: *mut RefSk,
        axis: i8,
        n: i32,
        a: *const f64,
        t: *const f64,
    ) -> i32;
    fn free(ptr: *mut c_void);
}

/// Reference-side Stepcompress handle (as in the itersolve tests).
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
// Test-local original-kinematics callbacks (the Rust equivalents of
// kin_cartesian.c / kin_corexy.c).
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

/// `corexy_stepper_plus_calc_position` from `kin_corexy.c`: `c.x + c.y`.
fn corexy_plus(_sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    let c = move_get_coord(m, move_time);
    c.x + c.y
}

/// Post callback used to exercise `shaper_commanded_pos_post_fixup`.
fn bump(_sk: &mut StepperKinematics) {
    // Genuinely mutates; would be a no-op otherwise.
    _sk.commanded_pos += 1.0;
}

/// C-side equivalent of `bump` for the `post_cb` test.
extern "C" fn c_bump(sk: *mut RefSk) {
    unsafe {
        (*sk).commanded_pos += 1.0;
    }
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
    SetPosition { x: f64, y: f64, z: f64 },
    CalcFromCoord { x: f64, y: f64, z: f64 },
    Append(MoveParams),
    GenerateSteps(f64),
    CheckActive(f64),
    Flush(u64),
    SetTime(f64, f64),
    SetParams { axis: u8, n: i32, a: Vec<f64>, t: Vec<f64> },
    /// Change the original kinematics' active flags, then `update_sk`.
    UpdateSk { flags: i32 },
    Finalize(f64, f64),
    Snapshot,
}

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    CartX,
    CartY,
    CartZ,
    CorexyPlus,
}

struct Pair {
    rust_orig: Box<StepperKinematics>,
    rust_is: Box<InputShaper>,
    c_orig: *mut RefSk,
    c_is: *mut RefSk,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
    rust_tq: Arc<Mutex<Trapq>>,
    c_tq: *mut CTrapq,
}

impl Pair {
    fn new(kind: Kind, step_dist: f64, with_post_cb: bool) -> Pair {
        chelper_ref::link();
        let (rust_cb, flags): (StepperCalcCb, i32) = match kind {
            Kind::CartX => (cart_x as StepperCalcCb, AF_X),
            Kind::CartY => (cart_y as StepperCalcCb, AF_Y),
            Kind::CartZ => (cart_z as StepperCalcCb, AF_Z),
            Kind::CorexyPlus => (corexy_plus as StepperCalcCb, AF_X | AF_Y),
        };
        let mut rust_orig = Box::new(StepperKinematics::new(rust_cb, flags));
        // `Stepcompress` is constructed through its port constructor.
        let mut rust_sc = Stepcompress::new();
        rust_sc.fill(7, 3000, 24, 23);
        rust_sc.set_time(0.0, 1_000_000.0);
        let mut ref_sc = RefSc::new();
        unsafe {
            stepcompress_fill(ref_sc.sc, 7, 3000, 24, 23);
            stepcompress_set_time(ref_sc.sc, 0.0, 1_000_000.0);
        }
        let c_orig = match kind {
            Kind::CartX => unsafe { cartesian_stepper_alloc(b'x' as i8) },
            Kind::CartY => unsafe { cartesian_stepper_alloc(b'y' as i8) },
            Kind::CartZ => unsafe { cartesian_stepper_alloc(b'z' as i8) },
            Kind::CorexyPlus => unsafe { corexy_stepper_alloc(b'+' as i8) },
        };
        assert!(!c_orig.is_null(), "original kinematics alloc returned null");
        if with_post_cb {
            rust_orig.post_cb = Some(bump);
            unsafe {
                (*c_orig).post_cb = Some(c_bump);
            }
        }
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "trapq_alloc returned null");
        let rust_tq = Arc::new(Mutex::new(Trapq::new()));
        let mut pair = Pair {
            rust_orig,
            rust_is: rust_input_shaper_alloc(),
            c_orig,
            c_is: std::ptr::null_mut(),
            rust_sc,
            ref_sc,
            rust_tq,
            c_tq,
        };
        // Mirror the Python wiring: the original kinematics gets the trapq
        // and its position first, then the shaper wraps it, then the
        // shaper's own kinematics gets the trapq.
        unsafe { itersolve_set_trapq(pair.c_orig, pair.c_tq, step_dist) };
        pair.rust_orig
            .set_trapq(Some(pair.rust_tq.clone()), step_dist);
        pair.set_original_position(0.0, 0.0, 0.0);
        let c_is = unsafe { c_input_shaper_alloc() };
        assert!(!c_is.is_null(), "input_shaper_alloc returned null");
        pair.c_is = c_is;
        let c_ret = unsafe { c_input_shaper_set_sk(pair.c_is, pair.c_orig) };
        let r_ret = unsafe {
            rust_input_shaper_set_sk(
                &mut pair.rust_is,
                pair.rust_orig.as_mut() as *mut StepperKinematics,
            )
        };
        assert_eq!(c_ret, r_ret, "new: input_shaper_set_sk return diverged");
        unsafe { itersolve_set_trapq(pair.c_is, pair.c_tq, step_dist) };
        pair.rust_is
            .sk
            .set_trapq(Some(pair.rust_tq.clone()), step_dist);
        assert_eq!(pair.rust_is.sk.active_flags, flags, "new: active_flags");
        pair
    }

    fn set_original_position(&mut self, x: f64, y: f64, z: f64) {
        unsafe { itersolve_set_position(self.c_orig, x, y, z) };
        self.rust_orig.set_position(x, y, z);
    }

    fn set_params(&mut self, axis: u8, n: i32, a: &[f64], t: &[f64], ctx: &str) {
        let c = unsafe {
            c_input_shaper_set_shaper_params(
                self.c_is,
                axis as i8,
                n,
                a.as_ptr(),
                t.as_ptr(),
            )
        };
        let r = rust_input_shaper_set_shaper_params(&mut self.rust_is, axis, n, a, t);
        assert_eq!(c, r, "{ctx}: input_shaper_set_shaper_params return diverged");
        self.compare_all(ctx);
    }

    fn compare_sk(&self, ctx: &str) {
        let c = unsafe { &*self.c_is };
        let r = &self.rust_is.sk;
        let co = unsafe { &*self.c_orig };
        let ro = &*self.rust_orig;
        for (name, cv, rv) in [
            ("commanded_pos", c.commanded_pos, r.commanded_pos),
            ("step_dist", c.step_dist, r.step_dist),
            ("last_flush_time", c.last_flush_time, r.last_flush_time),
            ("last_move_time", c.last_move_time, r.last_move_time),
            ("gen_steps_pre_active", c.gen_steps_pre_active, r.gen_steps_pre_active),
            ("gen_steps_post_active", c.gen_steps_post_active, r.gen_steps_post_active),
            ("orig.commanded_pos", co.commanded_pos, ro.commanded_pos),
            ("orig.last_flush_time", co.last_flush_time, ro.last_flush_time),
            ("orig.last_move_time", co.last_move_time, ro.last_move_time),
        ] {
            assert_eq!(
                cv.to_bits(),
                rv.to_bits(),
                "{ctx}: {name} C={cv:?} Rust={rv:?}"
            );
        }
        assert_eq!(
            c.active_flags, r.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.active_flags, r.active_flags
        );
        assert_eq!(
            co.active_flags, ro.active_flags,
            "{ctx}: orig.active_flags C={} Rust={}",
            co.active_flags, ro.active_flags
        );
        // The scratch move: shaped position plus move_t = 2 * DUMMY_T.
        let full = unsafe { &*(self.c_is as *const RefInputShaper) };
        let dummy = self.rust_is.dummy_move();
        assert_eq!(
            (full.m.start_pos[0].to_bits(), full.m.start_pos[1].to_bits(), full.m.start_pos[2].to_bits()),
            (dummy.start_pos.x.to_bits(), dummy.start_pos.y.to_bits(), dummy.start_pos.z.to_bits()),
            "{ctx}: scratch start_pos C={:?} Rust={:?}",
            full.m.start_pos,
            dummy.start_pos
        );
        assert_eq!(
            full.m.move_t.to_bits(),
            dummy.move_t.to_bits(),
            "{ctx}: scratch move_t C={:?} Rust={:?}",
            full.m.move_t,
            dummy.move_t
        );
        assert_eq!(
            (2.0 * DUMMY_T).to_bits(),
            dummy.move_t.to_bits(),
            "{ctx}: scratch move_t is not 2 * DUMMY_T"
        );
    }

    fn compare_pulses(&self, ctx: &str) {
        let full = unsafe { &*(self.c_is as *const RefInputShaper) };
        for axis in 0..3 {
            let c = &full.sp[axis];
            let r = self.rust_is.shaper_pulses(axis);
            assert_eq!(
                c.num_pulses, r.num_pulses,
                "{ctx}: axis {axis} num_pulses C={} Rust={}",
                c.num_pulses, r.num_pulses
            );
            assert_eq!(
                c.p_ind, r.p_ind,
                "{ctx}: axis {axis} p_ind C={} Rust={}",
                c.p_ind, r.p_ind
            );
            for i in 0..10 {
                assert_eq!(
                    (c.pulses[i].t.to_bits(), c.pulses[i].a.to_bits()),
                    (r.pulses[i].t.to_bits(), r.pulses[i].a.to_bits()),
                    "{ctx}: axis {axis} pulse {i} C=({:?},{:?}) Rust=({:?},{:?})",
                    c.pulses[i].t,
                    c.pulses[i].a,
                    r.pulses[i].t,
                    r.pulses[i].a
                );
            }
        }
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
        self.compare_pulses(ctx);
        self.compare_sc(ctx);
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe {
            free(self.c_is as *mut c_void);
            free(self.c_orig as *mut c_void);
            trapq_free(self.c_tq);
        }
    }
}

// ---------------------------------------------------------------------------
// Op driver.
// ---------------------------------------------------------------------------

fn run_op(pair: &mut Pair, op: &Op, ctx: &str) {
    match op {
        Op::SetPosition { x, y, z } => {
            unsafe { itersolve_set_position(pair.c_is, *x, *y, *z) };
            pair.rust_is.sk.set_position(*x, *y, *z);
        }
        Op::CalcFromCoord { x, y, z } => {
            let c = unsafe { itersolve_calc_position_from_coord(pair.c_is, *x, *y, *z) };
            let r = pair.rust_is.sk.calc_position_from_coord(*x, *y, *z);
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
            unsafe { trapq_check_sentinels(pair.c_tq) };
            pair.rust_tq.lock().check_sentinels();
            let c =
                unsafe { itersolve_generate_steps(pair.c_is, pair.ref_sc.sc, *flush_time) };
            let r = pair.rust_is.sk.generate_steps(&mut pair.rust_sc, *flush_time);
            assert_eq!(c, r, "{ctx}: generate_steps({flush_time}) return diverged");
        }
        Op::CheckActive(flush_time) => {
            let c = unsafe { itersolve_check_active(pair.c_is, *flush_time) };
            let r = pair.rust_is.sk.check_active(*flush_time);
            assert_eq!(
                c.to_bits(),
                r.to_bits(),
                "{ctx}: check_active({flush_time}) C={c:?} Rust={r:?}"
            );
        }
        Op::Flush(move_clock) => {
            let c = unsafe { stepcompress_flush(pair.ref_sc.sc, *move_clock) };
            let r = pair.rust_sc.flush(*move_clock);
            assert_eq!(c, r, "{ctx}: stepcompress_flush return diverged");
        }
        Op::SetTime(off, freq) => {
            unsafe { stepcompress_set_time(pair.ref_sc.sc, *off, *freq) };
            pair.rust_sc.set_time(*off, *freq);
        }
        Op::SetParams { axis, n, a, t } => {
            pair.set_params(*axis, *n, a, t, ctx);
        }
        Op::UpdateSk { flags } => {
            unsafe {
                (*pair.c_orig).active_flags = *flags;
                c_input_shaper_update_sk(pair.c_is);
            }
            pair.rust_orig.active_flags = *flags;
            rust_input_shaper_update_sk(&mut pair.rust_is);
        }
        Op::Finalize(pt, cht) => {
            unsafe { trapq_finalize_moves(pair.c_tq, *pt, *cht) };
            pair.rust_tq.lock().finalize_moves(*pt, *cht);
        }
        Op::Snapshot => {}
    }
    // Visible-function probes after every op.
    assert_eq!(
        unsafe { itersolve_get_commanded_pos(pair.c_is) }.to_bits(),
        pair.rust_is.sk.get_commanded_pos().to_bits(),
        "{ctx}: get_commanded_pos diverged"
    );
    assert_eq!(
        unsafe { itersolve_get_gen_steps_pre_active(pair.c_is) }.to_bits(),
        pair.rust_is.sk.get_gen_steps_pre_active().to_bits(),
        "{ctx}: get_gen_steps_pre_active diverged"
    );
    assert_eq!(
        unsafe { itersolve_get_gen_steps_post_active(pair.c_is) }.to_bits(),
        pair.rust_is.sk.get_gen_steps_post_active().to_bits(),
        "{ctx}: get_gen_steps_post_active diverged"
    );
    pair.compare_all(ctx);
}

fn run_stream(ctx: &str, kind: Kind, step_dist: f64, params: &[ShaperParam], ops: &[Op]) {
    let mut pair = Pair::new(kind, step_dist, false);
    for (i, p) in params.iter().enumerate() {
        pair.set_params(p.axis, p.n, &p.a, &p.t, &format!("{ctx}#param{i}"));
    }
    pair.compare_all(&format!("{ctx}#init"));
    for (i, op) in ops.iter().enumerate() {
        run_op(&mut pair, op, &format!("{ctx}#{i}"));
    }
}

struct ShaperParam {
    axis: u8,
    n: i32,
    a: Vec<f64>,
    t: Vec<f64>,
}

fn zv() -> ShaperParam {
    ShaperParam {
        axis: b'x',
        n: 2,
        a: vec![0.5, 0.5],
        t: vec![0.0, 0.011],
    }
}

fn mzv_3() -> ShaperParam {
    ShaperParam {
        axis: b'x',
        n: 3,
        a: vec![0.28, 0.44, 0.28],
        t: vec![0.0, 0.02, 0.041],
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

// ---------------------------------------------------------------------------
// Deterministic scenarios.
// ---------------------------------------------------------------------------

/// `input_shaper_set_sk` flag matrix, including the no-axis failure.
#[test]
fn set_sk_flag_matrix() {
    chelper_ref::link();
    for (ctx, flags, cb) in [
        ("x", AF_X, cart_x as StepperCalcCb),
        ("y", AF_Y, cart_y as StepperCalcCb),
        ("z", AF_Z, cart_z as StepperCalcCb),
        ("xy", AF_X | AF_Y, corexy_plus as StepperCalcCb),
        ("xyz", AF_X | AF_Y | AF_Z, corexy_plus as StepperCalcCb),
    ] {
        let mut orig = StepperKinematics::new(cb, flags);
        orig.commanded_pos = 3.25;
        orig.last_flush_time = 1.5;
        orig.last_move_time = 2.5;
        let mut rust_is = rust_input_shaper_alloc();
        let c_orig = unsafe { cartesian_stepper_alloc(b'x' as i8) };
        unsafe {
            (*c_orig).active_flags = flags;
            (*c_orig).commanded_pos = 3.25;
            (*c_orig).last_flush_time = 1.5;
            (*c_orig).last_move_time = 2.5;
        }
        let c_is = unsafe { c_input_shaper_alloc() };
        let c_ret = unsafe { c_input_shaper_set_sk(c_is, c_orig) };
        let r_ret = unsafe {
            rust_input_shaper_set_sk(&mut rust_is, &mut orig as *mut StepperKinematics)
        };
        assert_eq!(c_ret, r_ret, "{ctx}: set_sk return diverged");
        assert_eq!(c_ret, 0, "{ctx}: set_sk should succeed");
        assert_eq!(
            unsafe { (*c_is).commanded_pos.to_bits() },
            rust_is.sk.commanded_pos.to_bits(),
            "{ctx}: copied commanded_pos diverged"
        );
        assert_eq!(
            unsafe { (*c_is).last_flush_time.to_bits() },
            rust_is.sk.last_flush_time.to_bits(),
            "{ctx}: copied last_flush_time diverged"
        );
        assert_eq!(
            unsafe { (*c_is).last_move_time.to_bits() },
            rust_is.sk.last_move_time.to_bits(),
            "{ctx}: copied last_move_time diverged"
        );
        assert_eq!(
            unsafe { (*c_is).active_flags },
            rust_is.sk.active_flags,
            "{ctx}: copied active_flags diverged"
        );
        unsafe {
            free(c_is as *mut c_void);
            free(c_orig as *mut c_void);
        }
    }
    // No active axis: -1, and the Rust shaper is left untouched.
    let mut orig = StepperKinematics::new(cart_x as StepperCalcCb, 0);
    orig.commanded_pos = 3.25;
    let mut rust_is = rust_input_shaper_alloc();
    let c_orig = unsafe { cartesian_stepper_alloc(b'x' as i8) };
    unsafe {
        (*c_orig).active_flags = 0;
        (*c_orig).commanded_pos = 3.25;
    }
    let c_is = unsafe { c_input_shaper_alloc() };
    let c_ret = unsafe { c_input_shaper_set_sk(c_is, c_orig) };
    let r_ret = unsafe { rust_input_shaper_set_sk(&mut rust_is, &mut orig as *mut StepperKinematics) };
    assert_eq!(c_ret, r_ret, "no-axis: set_sk return diverged");
    assert_eq!(c_ret, -1, "no-axis: set_sk should fail");
    assert_eq!(rust_is.sk.active_flags, 0, "no-axis: Rust active_flags changed");
    assert_eq!(
        rust_is.sk.commanded_pos.to_bits(),
        0.0_f64.to_bits(),
        "no-axis: Rust commanded_pos changed"
    );
    assert!(
        rust_is.orig_sk().is_null(),
        "no-axis: Rust orig_sk attached"
    );
    unsafe {
        free(c_is as *mut c_void);
        free(c_orig as *mut c_void);
    }
}

/// `set_shaper_params` validation and reconfiguration semantics.
#[test]
fn set_shaper_params_validation() {
    let mut pair = Pair::new(Kind::CartX, 0.0125, false);
    pair.compare_all("init");
    // Valid first configuration.
    pair.set_params(b'x', 2, &[0.5, 0.5], &[0.0, 0.011], "valid zv");
    // Unknown axis.
    pair.set_params(b'w', 2, &[0.5, 0.5], &[0.0, 0.011], "unknown axis");
    // Inactive axes: ignored with status 0.
    pair.set_params(b'y', 2, &[0.5, 0.5], &[0.0, 0.011], "inactive y");
    pair.set_params(b'z', 3, &[0.28, 0.44, 0.28], &[0.0, 0.02, 0.041], "inactive z");
    // Out-of-range counts: clear the pulses.
    pair.set_params(
        b'x',
        11,
        &[1.0; 11],
        &[0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10],
        "n too large",
    );
    pair.set_params(b'x', 2, &[0.5, 0.5], &[0.0, 0.011], "valid again");
    // Non-increasing times: fails, keeps the previous pulses and
    // recomputes the generation windows from them.
    pair.set_params(b'x', 2, &[0.5, 0.5], &[0.02, 0.02], "equal times");
    pair.set_params(b'x', 3, &[0.3, 0.4, 0.3], &[0.02, 0.01, 0.03], "decreasing times");
    // Empty pulse train disables shaping.
    pair.set_params(b'x', 0, &[], &[], "empty");
    // A fresh three-pulse train after a disable.
    pair.set_params(b'x', 3, &[0.28, 0.44, 0.28], &[0.0, 0.02, 0.041], "mzv again");
    // Ten pulses, the array capacity.
    pair.set_params(
        b'x',
        10,
        &[0.1; 10],
        &[0.0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045],
        "ten pulses",
    );
}

/// Shaped step generation on the X axis with a two-pulse shaper.
#[test]
fn shaped_steps_x() {
    let ops = vec![
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::CalcFromCoord { x: 10.0, y: 5.0, z: 0.0 },
        // X cruise 0 -> +20 mm (v=20 for 1 s).
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, 1.0)),
        Op::CheckActive(10.0),
        Op::GenerateSteps(1.5),
        Op::Flush(u64::MAX),
        Op::GenerateSteps(2.5),
        Op::Flush(u64::MAX),
        // Reversal: back along X from 20 to 5, with a gap in print_time.
        Op::Append(cruise(3.0, [20.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 15.0, 1.0)),
        Op::GenerateSteps(4.5),
        Op::Flush(u64::MAX),
        // Short diagonal move at low speed: pulse excursions span
        // neighbouring moves.
        Op::Append(cruise(4.0, [5.0, 5.0, 0.0], [1.0, 0.0, 0.0], 2.0, 0.1)),
        Op::GenerateSteps(4.05),
        Op::Flush(u64::MAX),
        Op::GenerateSteps(4.5),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    run_stream("shaped-x", Kind::CartX, 0.0125, &[zv()], &ops);
}

/// Shaped step generation on the Y axis with a three-pulse shaper.
#[test]
fn shaped_steps_y() {
    let mut p = mzv_3();
    p.axis = b'y';
    let ops = vec![
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 12.0, 1.0)),
        Op::GenerateSteps(1.25),
        Op::Flush(u64::MAX),
        Op::GenerateSteps(2.5),
        Op::Flush(u64::MAX),
        Op::Append(cruise(2.0, [0.0, 12.0, 0.0], [0.0, -1.0, 0.0], 12.0, 1.0)),
        Op::GenerateSteps(3.5),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    run_stream("shaped-y", Kind::CartY, 0.0125, &[p], &ops);
}

/// Shaped step generation on the Z axis.
#[test]
fn shaped_steps_z() {
    let mut p = mzv_3();
    p.axis = b'z';
    let ops = vec![
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 8.0, 0.5)),
        Op::GenerateSteps(1.2),
        Op::Flush(u64::MAX),
        Op::GenerateSteps(2.0),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    run_stream("shaped-z", Kind::CartZ, 0.0125, &[p], &ops);
}

/// The xyz variant on a CoreXY original: only shaped axes are convolved,
/// the rest keeps the plain coordinate.
#[test]
fn shaped_steps_corexy_xyz() {
    let mut px = zv();
    px.axis = b'x';
    let mut py = mzv_3();
    py.axis = b'y';
    let mut pz = zv();
    pz.axis = b'z'; // inactive on this original: no-op
    let ops = vec![
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        // Diagonal: x and y together.
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [0.6, 0.8, 0.0], 25.0, 1.0)),
        Op::GenerateSteps(1.5),
        Op::Flush(u64::MAX),
        Op::GenerateSteps(2.5),
        Op::Flush(u64::MAX),
        // X-only reversal.
        Op::Append(cruise(2.0, [15.0, 20.0, 0.0], [-1.0, 0.0, 0.0], 10.0, 0.5)),
        Op::GenerateSteps(3.0),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    run_stream("shaped-corexy", Kind::CorexyPlus, 0.0125, &[px, py, pz], &ops);
}

/// `update_sk` after the original kinematics changes its flags.
#[test]
fn update_sk_after_flag_change() {
    let ops = vec![
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, 1.0)),
        Op::GenerateSteps(2.5),
        Op::Flush(u64::MAX),
        // The original now also drives Y: the xyz variant replaces the
        // x variant and the generation windows widen.
        Op::UpdateSk { flags: AF_X | AF_Y },
        Op::SetParams {
            axis: b'y',
            n: 3,
            a: vec![0.28, 0.44, 0.28],
            t: vec![0.0, 0.02, 0.041],
        },
        Op::Append(cruise(3.0, [20.0, 0.0, 0.0], [0.6, 0.8, 0.0], 25.0, 1.0)),
        Op::GenerateSteps(4.5),
        Op::Flush(u64::MAX),
        // And down to Y only.
        Op::UpdateSk { flags: AF_Y },
        Op::Append(cruise(5.0, [35.0, 20.0, 0.0], [0.0, 1.0, 0.0], 16.0, 1.0)),
        Op::GenerateSteps(6.5),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    let mut py = mzv_3();
    py.axis = b'y';
    run_stream(
        "update-sk",
        Kind::CartX,
        0.0125,
        &[zv(), py],
        &ops,
    );
}

/// `shaper_commanded_pos_post_fixup` forwards the commanded position to
/// an original kinematics carrying its own post callback.
#[test]
fn post_cb_forwarding() {
    let mut pair = Pair::new(Kind::CartX, 0.0125, true);
    pair.set_params(b'x', 2, &[0.5, 0.5], &[0.0, 0.011], "zv");
    let ops = vec![
        Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 },
        Op::Append(cruise(1.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 20.0, 1.0)),
        Op::GenerateSteps(2.5),
        Op::Flush(u64::MAX),
        // A second range from an unflushed position: another forward.
        Op::Append(cruise(3.0, [20.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 20.0, 1.0)),
        Op::GenerateSteps(4.5),
        Op::Flush(u64::MAX),
        Op::Snapshot,
    ];
    for (i, op) in ops.iter().enumerate() {
        run_op(&mut pair, op, &format!("post-cb#{i}"));
    }
}

/// Deterministic pseudo-random stream over shaped generation.
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

    /// A double in [0, hi) with a coarse granularity.
    fn window(&mut self, hi: f64) -> f64 {
        let v = (self.next() % 1_000_000) as f64 / 1_000_000.0;
        v * hi
    }
}

#[test]
fn random_stream() {
    let mut rng = Rng(0x9e3779b97f4a7c15);
    let mut ops: Vec<Op> = Vec::new();
    ops.push(Op::SetPosition { x: 0.0, y: 0.0, z: 0.0 });
    let mut t = 1.0;
    let mut pos = 0.0;
    for _ in 0..40 {
        match rng.next() % 5 {
            0 | 1 => {
                let dir = if rng.next() % 2 == 0 { 1.0 } else { -1.0 };
                let v = 1.0 + rng.window(80.0);
                let dur = 0.05 + rng.window(0.6);
                ops.push(Op::Append(cruise(t, [pos, 0.0, 0.0], [dir, 0.0, 0.0], v, dur)));
                pos += dir * v * dur;
                t += dur + rng.window(0.05);
            }
            2 => {
                ops.push(Op::GenerateSteps(t + rng.window(0.3)));
                ops.push(Op::Flush(u64::MAX));
            }
            3 => {
                let n = (2 + rng.next() % 4) as i32;
                let mut a = Vec::new();
                let mut tt = Vec::new();
                let mut acc = 0.0;
                for _ in 0..n {
                    a.push(0.05 + rng.window(0.6));
                    tt.push(acc);
                    acc += 0.005 + rng.window(0.04);
                }
                ops.push(Op::SetParams { axis: b'x', n, a, t: tt });
            }
            _ => {
                ops.push(Op::GenerateSteps(t + 0.2));
                ops.push(Op::Flush(u64::MAX));
                ops.push(Op::Finalize(t, t));
            }
        }
    }
    ops.push(Op::GenerateSteps(t + 1.0));
    ops.push(Op::Flush(u64::MAX));
    ops.push(Op::Snapshot);
    run_stream("random", Kind::CartX, 0.0125, &[zv()], &ops);
}
