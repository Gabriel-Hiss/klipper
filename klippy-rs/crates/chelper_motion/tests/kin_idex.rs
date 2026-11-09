//! Differential parity tests for `chelper_motion::kin_idex`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - `dual_carriage_alloc` defaults: the embedded `StepperKinematics`
//!   state, the scratch move (`move_t = 2 * DUMMY_T`, everything else
//!   zero), and the unit X/Y scales with zero offsets,
//! - `dual_carriage_set_sk`: the copied `active_flags` (and that the
//!   position callback is installed, by calling it),
//! - `dual_carriage_set_transform`: the return code, the stored
//!   scale/offset, and the `active_flags` adjustment for both axes
//!   (including the invalid-axis path, which changes nothing),
//! - `dual_carriage_calc_position`, called through the installed
//!   `calc_position_cb` on both sides: the return value and the scratch
//!   move `start_pos` left behind, across transforms (identity,
//!   scaled/offset, mirror, zero-scale) and wrapped axes.
//!
//! The wrapped kinematics is the reference C `cartesian_stepper_alloc`
//! (and the test-local Cartesian callback on the Rust side, the same
//! trick as the `itersolve` tests); the C callback is invoked through
//! the `calc_position_cb` field of the reference `stepper_kinematics`,
//! exactly how `itersolve_generate_steps` calls it.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_idex`
use chelper_motion::itersolve::{
    StepperCalcCb, StepperKinematics, AF_X, AF_Y, AF_Z,
};
use chelper_motion::kin_idex::{
    dual_carriage_alloc, dual_carriage_set_sk, dual_carriage_set_transform, DualCarriage,
};
use chelper_motion::trapq::{move_get_coord, Coord, Move};
use core::ffi::c_void;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// kin_cartesian.c / kin_idex.c).
// ---------------------------------------------------------------------------

#[repr(C)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

#[repr(C)]
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

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout.
#[repr(C)]
#[allow(dead_code)]
struct RefSk {
    step_dist: f64,
    commanded_pos: f64,
    sc: *mut c_void,
    last_flush_time: f64,
    last_move_time: f64,
    tq: *mut c_void,
    active_flags: i32,
    gen_steps_pre_active: f64,
    gen_steps_post_active: f64,
    calc_position_cb: *mut c_void,
    post_cb: *mut c_void,
}

/// `struct dual_carriage_stepper` (kin_idex.c). `dual_carriage_alloc`
/// returns `&dc->sk`, which is `dc` itself since `sk` is the first
/// field.
#[repr(C)]
struct RefDc {
    sk: RefSk,
    orig_sk: *mut RefSk,
    m: RefMove,
    x_scale: f64,
    x_offs: f64,
    y_scale: f64,
    y_offs: f64,
}

type RefCalcCb = unsafe extern "C" fn(*mut RefSk, *mut RefMove, f64) -> f64;

unsafe extern "C" {
    fn cartesian_stepper_alloc(axis: i8) -> *mut RefSk;
    #[link_name = "dual_carriage_alloc"]
    fn c_dual_carriage_alloc() -> *mut RefSk;
    #[link_name = "dual_carriage_set_sk"]
    fn c_dual_carriage_set_sk(sk: *mut RefSk, orig_sk: *mut RefSk);
    #[link_name = "dual_carriage_set_transform"]
    fn c_dual_carriage_set_transform(sk: *mut RefSk, axis: i8, scale: f64, offs: f64) -> i32;
    fn free(ptr: *mut c_void);
}

// ---------------------------------------------------------------------------
// Test-local Cartesian callbacks (the Rust equivalent of kin_cartesian.c).
// ---------------------------------------------------------------------------

fn cart_x(_sk: &StepperKinematics, m: &Move, _at: Option<usize>, move_time: f64) -> f64 {
    move_get_coord(m, move_time).x
}

fn cart_y(_sk: &StepperKinematics, m: &Move, _at: Option<usize>, move_time: f64) -> f64 {
    move_get_coord(m, move_time).y
}

fn cart_z(_sk: &StepperKinematics, m: &Move, _at: Option<usize>, move_time: f64) -> f64 {
    move_get_coord(m, move_time).z
}

fn rust_cb(axis: u8) -> StepperCalcCb {
    match axis {
        b'x' => cart_x as StepperCalcCb,
        b'y' => cart_y as StepperCalcCb,
        _ => cart_z as StepperCalcCb,
    }
}

fn rust_flags(axis: u8) -> i32 {
    match axis {
        b'x' => AF_X,
        b'y' => AF_Y,
        _ => AF_Z,
    }
}

// ---------------------------------------------------------------------------
// Pair: one Rust wrapper, one C wrapper, each with its own orig kinematics.
// ---------------------------------------------------------------------------

struct Pair {
    rust_dc: DualCarriage,
    rust_orig: Box<StepperKinematics>,
    c_dc_sk: *mut RefSk,
    c_orig_sk: *mut RefSk,
}

impl Pair {
    fn new(axis: u8) -> Pair {
        chelper_ref::link();
        let mut rust_orig = Box::new(StepperKinematics::new(rust_cb(axis), rust_flags(axis)));
        let mut rust_dc = dual_carriage_alloc();
        // SAFETY: `rust_orig` is boxed and lives in the same `Pair` as the wrapper.
        unsafe { dual_carriage_set_sk(&mut rust_dc, &mut *rust_orig) };
        let c_orig_sk = unsafe { cartesian_stepper_alloc(axis as i8) };
        assert!(!c_orig_sk.is_null(), "cartesian_stepper_alloc returned null");
        let c_dc_sk = unsafe { c_dual_carriage_alloc() };
        assert!(!c_dc_sk.is_null(), "dual_carriage_alloc returned null");
        unsafe { c_dual_carriage_set_sk(c_dc_sk, c_orig_sk) };
        Pair {
            rust_dc,
            rust_orig,
            c_dc_sk,
            c_orig_sk,
        }
    }

    fn c_dc(&self) -> &RefDc {
        unsafe { &*(self.c_dc_sk as *const RefDc) }
    }

    fn set_transform(&mut self, axis: u8, scale: f64, offs: f64, ctx: &str) {
        let c_ret = unsafe { c_dual_carriage_set_transform(self.c_dc_sk, axis as i8, scale, offs) };
        let r_ret = dual_carriage_set_transform(&mut self.rust_dc, axis, scale, offs);
        assert_eq!(c_ret, r_ret, "{ctx}: set_transform return C={c_ret} Rust={r_ret}");
        self.compare_state(ctx);
    }

    fn compare_state(&self, ctx: &str) {
        let c = self.c_dc();
        let r = &self.rust_dc;
        assert_eq!(
            c.sk.active_flags, r.sk.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.sk.active_flags, r.sk.active_flags
        );
        for (name, c_val, r_val) in [
            ("x_scale", c.x_scale, r.x_transform().0),
            ("x_offs", c.x_offs, r.x_transform().1),
            ("y_scale", c.y_scale, r.y_transform().0),
            ("y_offs", c.y_offs, r.y_transform().1),
            ("m.move_t", c.m.move_t, r.scratch_move().move_t),
        ] {
            assert_eq!(
                c_val.to_bits(),
                r_val.to_bits(),
                "{ctx}: {name} C={c_val:?} Rust={r_val:?}"
            );
        }
        // The wrapper's solver state stays zeroed (neither side runs the
        // solver here), matching the C zeroed allocation.
        for (name, c_val, r_val) in [
            ("step_dist", c.sk.step_dist, r.sk.step_dist),
            ("commanded_pos", c.sk.commanded_pos, r.sk.commanded_pos),
            ("last_flush_time", c.sk.last_flush_time, r.sk.last_flush_time),
            ("last_move_time", c.sk.last_move_time, r.sk.last_move_time),
            ("gen_steps_pre_active", c.sk.gen_steps_pre_active, r.sk.gen_steps_pre_active),
            ("gen_steps_post_active", c.sk.gen_steps_post_active, r.sk.gen_steps_post_active),
        ] {
            assert_eq!(
                c_val.to_bits(),
                r_val.to_bits(),
                "{ctx}: sk.{name} C={c_val:?} Rust={r_val:?}"
            );
        }
    }

    /// Call the installed `calc_position_cb` on both sides with the same
    /// move and compare the return value and the scratch move state.
    fn calc(&mut self, m: &Move, move_time: f64, ctx: &str) {
        let mut cm = RefMove {
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
        };
        let r_orig = self.rust_orig.commanded_pos;
        let c_orig = unsafe { (*self.c_orig_sk).commanded_pos };
        let c_ret = unsafe {
            let cb: RefCalcCb = core::mem::transmute((*self.c_dc_sk).calc_position_cb);
            cb(self.c_dc_sk, &mut cm, move_time)
        };
        let r_cb = self.rust_dc.sk.calc_position_cb;
        let r_ret = r_cb(&self.rust_dc.sk, m, None, move_time);
        assert_eq!(
            c_ret.to_bits(),
            r_ret.to_bits(),
            "{ctx}: calc_position C={c_ret:?} Rust={r_ret:?}"
        );
        let c = self.c_dc();
        let rm = self.rust_dc.scratch_move();
        for (name, c_val, r_val) in [
            ("m.start_pos.x", c.m.start_pos.x, rm.start_pos.x),
            ("m.start_pos.y", c.m.start_pos.y, rm.start_pos.y),
            ("m.start_pos.z", c.m.start_pos.z, rm.start_pos.z),
            ("m.print_time", c.m.print_time, rm.print_time),
            ("m.move_t", c.m.move_t, rm.move_t),
            ("m.start_v", c.m.start_v, rm.start_v),
            ("m.half_accel", c.m.half_accel, rm.half_accel),
            ("m.axes_r.x", c.m.axes_r.x, rm.axes_r.x),
            ("m.axes_r.y", c.m.axes_r.y, rm.axes_r.y),
            ("m.axes_r.z", c.m.axes_r.z, rm.axes_r.z),
        ] {
            assert_eq!(
                c_val.to_bits(),
                r_val.to_bits(),
                "{ctx}: {name} C={c_val:?} Rust={r_val:?}"
            );
        }
        // The wrapped kinematics' commanded position is not updated by
        // the wrapper (map-motion.md: `orig_sk->commanded_pos` is not
        // updated by `dual_carriage_calc_position`).
        assert_eq!(
            c_orig.to_bits(),
            unsafe { (*self.c_orig_sk).commanded_pos }.to_bits(),
            "{ctx}: orig commanded_pos changed on the C side"
        );
        assert_eq!(
            r_orig.to_bits(),
            self.rust_orig.commanded_pos.to_bits(),
            "{ctx}: orig commanded_pos changed on the Rust side"
        );
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe {
            free(self.c_dc_sk as *mut c_void);
            free(self.c_orig_sk as *mut c_void);
        }
    }
}

// ---------------------------------------------------------------------------
// Scenarios.
const MOVES: &[Move] = &[
    // Stationary at the origin.
    Move {
        print_time: 0.0,
        move_t: 1.0,
        start_v: 0.0,
        half_accel: 0.0,
        start_pos: Coord::new(0.0, 0.0, 0.0),
        axes_r: Coord::new(0.0, 0.0, 0.0),
    },
    // Cruise on X from (10, 20, 5).
    Move {
        print_time: 1.5,
        move_t: 2.0,
        start_v: 25.0,
        half_accel: 0.0,
        start_pos: Coord::new(10.0, 20.0, 5.0),
        axes_r: Coord::new(1.0, 0.0, 0.0),
    },
    // Accelerating move on all three axes.
    Move {
        print_time: 2.25,
        move_t: 0.75,
        start_v: 3.5,
        half_accel: 40.0,
        start_pos: Coord::new(-12.5, 7.25, 100.0),
        axes_r: Coord::new(0.6, 0.48, 0.64),
    },
    // Negative direction, decelerating to a stop.
    Move {
        print_time: 3.0,
        move_t: 1.25,
        start_v: 80.0,
        half_accel: -32.0,
        start_pos: Coord::new(200.0, -50.0, 0.0),
        axes_r: Coord::new(-1.0, 0.0, 0.0),
    },
];

const TIMES: &[f64] = &[0.0, 0.125, 0.5, 0.75, 1.0];

/// Alloc defaults, `set_sk` flag copy, and identity-transform calc.
#[test]
fn alloc_and_identity() {
    chelper_ref::link();
    // Exercise alloc before wrapping (the wrapped `active_flags` are
    // copied by set_sk).
    let c_orig = unsafe { cartesian_stepper_alloc(b'x' as i8) };
    let c_dc = unsafe { c_dual_carriage_alloc() };
    let c = unsafe { &*(c_dc as *const RefDc) };
    assert_eq!(c.sk.active_flags, 0, "C alloc active_flags");
    assert_eq!(c.m.move_t.to_bits(), (1000.0f64).to_bits(), "C alloc move_t");
    assert_eq!(c.x_scale.to_bits(), 1.0f64.to_bits(), "C alloc x_scale");
    assert_eq!(c.y_scale.to_bits(), 1.0f64.to_bits(), "C alloc y_scale");
    assert_eq!(c.x_offs.to_bits(), 0.0f64.to_bits(), "C alloc x_offs");
    assert_eq!(c.y_offs.to_bits(), 0.0f64.to_bits(), "C alloc y_offs");
    unsafe {
        free(c_dc as *mut c_void);
        free(c_orig as *mut c_void);
    }

    let c_orig = unsafe { cartesian_stepper_alloc(b'x' as i8) };
    let c_dc = unsafe { c_dual_carriage_alloc() };
    let mut r_dc = dual_carriage_alloc();
    let mut r_orig = Box::new(StepperKinematics::new(cart_x as StepperCalcCb, AF_X));
    // SAFETY: `r_orig` is boxed and moved into the same `Pair` as the wrapper.
    unsafe { dual_carriage_set_sk(&mut r_dc, &mut *r_orig) };
    unsafe { c_dual_carriage_set_sk(c_dc, c_orig) };

    let mut pair = Pair {
        rust_dc: r_dc,
        rust_orig: r_orig,
        c_dc_sk: c_dc,
        c_orig_sk: c_orig,
    };
    // set_sk copies the wrapped active flags; with identity scales the
    // transformed position is the wrapped cartesian position.
    assert_eq!(pair.c_dc().sk.active_flags, AF_X, "C set_sk active_flags");
    assert_eq!(pair.rust_dc.sk.active_flags, AF_X, "Rust set_sk active_flags");
    pair.compare_state("identity: after set_sk");
    for (i, m) in MOVES.iter().enumerate() {
        for (j, t) in TIMES.iter().enumerate() {
            pair.calc(m, *t, &format!("identity move {i} time {j}"));
        }
    }
}

/// Scale/offset transforms on X and Y for an X-wrapped kinematics,
/// including mirror (negative scale) and zero-scale deactivation.
#[test]
fn transforms_drive_calc_and_flags() {
    let mut pair = Pair::new(b'x');
    let transforms: [(u8, f64, f64); 8] = [
        (b'x', 2.0, 10.0),
        (b'x', -1.0, 0.0), // MIRROR
        (b'x', 0.5, -3.75),
        (b'x', 0.0, 42.0), // zero scale: deactivates X
        (b'y', 3.0, -7.5),
        (b'y', -2.0, 100.0),
        (b'y', 0.0, 1.0), // zero scale: deactivates Y (not set here)
        (b'x', 1.0, 0.0), // back to identity
    ];
    for (i, (axis, scale, offs)) in transforms.iter().enumerate() {
        pair.set_transform(*axis, *scale, *offs, &format!("transform {i}"));
        for (j, m) in MOVES.iter().enumerate() {
            for (k, t) in TIMES.iter().enumerate() {
                let ctx = format!("transform {i} ({axis}={scale}*{offs} move {j} time {k})");
                pair.calc(m, *t, &ctx);
                pair.compare_state(&ctx);
            }
        }
    }
}

/// The zero/nonzero scale active-flag matrix on both axes:
/// a nonzero scale only reactivates the axis when the wrapped
/// kinematics has it active.
#[test]
fn active_flag_matrix() {
    let mut pair = Pair::new(b'y');
    pair.set_transform(b'x', 5.0, 0.0, "x nonzero, orig lacks AF_X: flag stays clear");
    assert_eq!(pair.rust_dc.sk.active_flags, AF_Y, "Rust flags after x set");
    assert_eq!(pair.c_dc().sk.active_flags, AF_Y, "C flags after x set");
    pair.set_transform(b'x', 0.0, 0.0, "x zero: clears AF_X");
    pair.set_transform(b'y', 1.0, 0.0, "y nonzero, orig has AF_Y: flag set");
    assert_eq!(pair.rust_dc.sk.active_flags, AF_Y);
    pair.set_transform(b'y', 0.0, 0.0, "y zero: clears AF_Y");
    assert_eq!(pair.rust_dc.sk.active_flags, 0);
    assert_eq!(pair.c_dc().sk.active_flags, 0);
    pair.set_transform(b'y', -0.0, 0.0, "y negative zero: still zero");
    pair.set_transform(b'y', 2.0, 1.0, "y nonzero again");
    assert_eq!(pair.rust_dc.sk.active_flags, AF_Y);
    // The transforms took effect on calc as well.
    pair.set_transform(b'y', 2.0, 1.0, "y transform for calc");
    for m in MOVES {
        pair.calc(m, 0.5, "flag matrix calc");
    }
}

/// Y- and Z-wrapped kinematics: the wrapper passes the transformed
/// coordinates through whatever the wrapped callback returns.
#[test]
fn other_wrapped_axes() {
    for axis in [b'y', b'z'] {
        let mut pair = Pair::new(axis);
        pair.set_transform(b'x', 1.5, 2.25, &format!("{}: x transform", axis as char));
        pair.set_transform(b'y', -0.75, 4.5, &format!("{}: y transform", axis as char));
        for (i, m) in MOVES.iter().enumerate() {
            for (j, t) in TIMES.iter().enumerate() {
                pair.calc(m, *t, &format!("{} move {i} time {j}", axis as char));
            }
        }
    }
}

/// Invalid axes return -1 and change nothing.
#[test]
fn invalid_axis() {
    let mut pair = Pair::new(b'x');
    pair.set_transform(b'x', 2.0, 5.0, "baseline");
    pair.set_transform(b'y', 0.25, -1.0, "baseline y");
    for axis in [b'z', b'Z', b'0', 0u8, b'e'] {
        let ctx = format!("invalid axis {axis}");
        pair.set_transform(axis, 3.0, 9.0, &ctx);
        for m in MOVES {
            pair.calc(m, 0.5, &format!("{ctx} calc"));
        }
    }
}

/// Zeroed scratch move after alloc, otherwise-untouched state.
#[test]
fn alloc_defaults_match() {
    chelper_ref::link();
    let c_dc = unsafe { c_dual_carriage_alloc() };
    let r_dc = dual_carriage_alloc();
    let c = unsafe { &*(c_dc as *const RefDc) };
    let rm = r_dc.scratch_move();
    for (name, c_val, r_val) in [
        ("m.print_time", c.m.print_time, rm.print_time),
        ("m.move_t", c.m.move_t, rm.move_t),
        ("m.start_v", c.m.start_v, rm.start_v),
        ("m.half_accel", c.m.half_accel, rm.half_accel),
        ("m.start_pos.x", c.m.start_pos.x, rm.start_pos.x),
        ("m.start_pos.y", c.m.start_pos.y, rm.start_pos.y),
        ("m.start_pos.z", c.m.start_pos.z, rm.start_pos.z),
        ("m.axes_r.x", c.m.axes_r.x, rm.axes_r.x),
        ("m.axes_r.y", c.m.axes_r.y, rm.axes_r.y),
        ("m.axes_r.z", c.m.axes_r.z, rm.axes_r.z),
        ("sk.step_dist", c.sk.step_dist, r_dc.sk.step_dist),
        ("sk.commanded_pos", c.sk.commanded_pos, r_dc.sk.commanded_pos),
        ("sk.last_flush_time", c.sk.last_flush_time, r_dc.sk.last_flush_time),
        ("sk.last_move_time", c.sk.last_move_time, r_dc.sk.last_move_time),
        ("sk.gen_steps_pre_active", c.sk.gen_steps_pre_active, r_dc.sk.gen_steps_pre_active),
        ("sk.gen_steps_post_active", c.sk.gen_steps_post_active, r_dc.sk.gen_steps_post_active),
    ] {
        assert_eq!(
            c_val.to_bits(),
            r_val.to_bits(),
            "alloc: {name} C={c_val:?} Rust={r_val:?}"
        );
    }
    assert_eq!(c.sk.active_flags, r_dc.sk.active_flags, "alloc: active_flags");
    assert_eq!(r_dc.x_transform(), (1.0, 0.0), "alloc: x transform");
    assert_eq!(r_dc.y_transform(), (1.0, 0.0), "alloc: y transform");
    unsafe { free(c_dc as *mut c_void) };
}
