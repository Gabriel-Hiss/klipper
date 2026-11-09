//! Differential parity tests for `chelper_motion::kin_corexy`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/kin_corexy.c` sources) with
//! identical inputs and compares, bit for bit (`f64::to_bits`, with
//! `NaN == NaN` accepted where a `NaN` propagates through the formula):
//!
//! - the freshly allocated `StepperKinematics` state (all `f64` fields
//!   zero, `active_flags = AF_X | AF_Y`, no trapq/compressor and no
//!   post callback) for `b'+'`, `b'-'`, and kinds the C leaves with a
//!   null `calc_position_cb` (for which the port faults on use, mirroring
//!   the null-pointer call),
//! - `itersolve_calc_position_from_coord` results for both steppers over
//!   corner-case and pseudo-random coordinates (including infinities and
//!   NaNs),
//! - the `calc_position_cb` itself, called directly with arbitrary moves
//!   and move times, so the `move_get_coord` path is compared beyond the
//!   synthetic move `itersolve_calc_position_from_coord` builds, and
//! - `itersolve_set_position` / `itersolve_get_commanded_pos` and
//!   `itersolve_is_active_axis` for both steppers.
//!
//! No recordings exist under `tests/fixtures/recordings/kin_corexy/`; the
//! reference C library is the oracle for this C unit, as with the sibling
//! `trapq` / `itersolve` tests.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_corexy`
//! (the integration owner builds and runs; per the pilot rules only
//! `cargo check`/`cargo clippy` run here, via WSL with a per-checkout
//! `CARGO_TARGET_DIR`).

use std::panic::{catch_unwind, AssertUnwindSafe};

use chelper_motion::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Y};
use chelper_motion::kin_corexy as port;
use chelper_motion::trapq::{Coord, Move};
use core::ffi::c_void;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h).
// ---------------------------------------------------------------------------

enum CTrapq {}

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

#[repr(C)]
#[derive(Clone, Copy)]
struct RefMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_x: f64,
    start_y: f64,
    start_z: f64,
    axes_r_x: f64,
    axes_r_y: f64,
    axes_r_z: f64,
    next: *mut RefMove,
    prev: *mut RefMove,
}

type CCalcCb = unsafe extern "C" fn(*mut RefSk, *const RefMove, f64) -> f64;

unsafe extern "C" {
    fn corexy_stepper_alloc(kind: i8) -> *mut RefSk;
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
    fn itersolve_is_active_axis(sk: *mut RefSk, axis: i8) -> i32;
    fn free(ptr: *mut c_void);
}

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn c_cb(c_sk: *mut RefSk) -> CCalcCb {
    let raw = unsafe { (*c_sk).calc_position_cb };
    assert!(!raw.is_null());
    unsafe { std::mem::transmute::<*mut c_void, CCalcCb>(raw) }
}

fn ref_move(m: &Move) -> RefMove {
    RefMove {
        print_time: m.print_time,
        move_t: m.move_t,
        start_v: m.start_v,
        half_accel: m.half_accel,
        start_x: m.start_pos.x,
        start_y: m.start_pos.y,
        start_z: m.start_pos.z,
        axes_r_x: m.axes_r.x,
        axes_r_y: m.axes_r.y,
        axes_r_z: m.axes_r.z,
        next: std::ptr::null_mut(),
        prev: std::ptr::null_mut(),
    }
}

/// One alloc on each side: the reference `corexy_stepper_alloc` in C and
/// `port::corexy_stepper_alloc` in Rust. The C allocation is freed by
/// [`Pair::teardown`].
struct Pair {
    rust_sk: StepperKinematics,
    c_sk: *mut RefSk,
}

impl Pair {
    fn new(kind: u8) -> Pair {
        chelper_ref::link();
        let c_sk = unsafe { corexy_stepper_alloc(kind as i8) };
        assert!(!c_sk.is_null(), "corexy_stepper_alloc returned null");
        Pair {
            rust_sk: port::corexy_stepper_alloc(kind),
            c_sk,
        }
    }

    fn teardown(self) {
        unsafe { free(self.c_sk as *mut c_void) };
    }
}

/// The zeroed-allocation state both sides must show right after alloc.
fn assert_alloc_state(pair: &Pair, ctx: &str) {
    let c = unsafe { &*pair.c_sk };
    let r = &pair.rust_sk;
    assert_eq!(0.0f64.to_bits(), c.step_dist.to_bits(), "{ctx}: step_dist");
    assert_eq!(0.0f64.to_bits(), c.commanded_pos.to_bits(), "{ctx}: commanded_pos C=0.0");
    assert_eq!(0.0f64.to_bits(), c.last_flush_time.to_bits(), "{ctx}: last_flush_time");
    assert_eq!(0.0f64.to_bits(), c.last_move_time.to_bits(), "{ctx}: last_move_time");
    assert_eq!(0.0f64.to_bits(), c.gen_steps_pre_active.to_bits(), "{ctx}: gen_steps_pre_active");
    assert_eq!(0.0f64.to_bits(), c.gen_steps_post_active.to_bits(), "{ctx}: gen_steps_post_active");
    assert!(c.sc.is_null(), "{ctx}: sc must be null");
    assert!(c.tq.is_null(), "{ctx}: tq must be null");
    assert!(c.post_cb.is_null(), "{ctx}: post_cb must be null");
    assert_eq!(AF_X | AF_Y, c.active_flags, "{ctx}: active_flags C");
    // Rust side: the same state.
    assert_eq!(0.0f64.to_bits(), r.step_dist.to_bits(), "{ctx}: step_dist Rust");
    assert_eq!(0.0f64.to_bits(), r.commanded_pos.to_bits(), "{ctx}: commanded_pos Rust");
    assert_eq!(0.0f64.to_bits(), r.last_flush_time.to_bits(), "{ctx}: last_flush_time Rust");
    assert_eq!(0.0f64.to_bits(), r.last_move_time.to_bits(), "{ctx}: last_move_time Rust");
    assert_eq!(0.0f64.to_bits(), r.gen_steps_pre_active.to_bits(), "{ctx}: gen_steps_pre_active Rust");
    assert_eq!(0.0f64.to_bits(), r.gen_steps_post_active.to_bits(), "{ctx}: gen_steps_post_active Rust");
    assert!(r.sc.is_none(), "{ctx}: sc Rust");
    assert!(r.tq.is_none(), "{ctx}: tq Rust");
    assert!(r.post_cb.is_none(), "{ctx}: post_cb Rust");
    assert_eq!(AF_X | AF_Y, r.active_flags, "{ctx}: active_flags Rust");
}

/// Bit-for-bit comparison, with `NaN == NaN` for inputs that propagate a
/// `NaN` through the formula. The sweep drives both sides with
/// indefinite-`NaN` shapes (e.g. `half_accel = inf`, `move_time = 0`,
/// so `inf * 0` generates the x86 indefinite `QNaN`); which source
/// `NaN` a compiler register order propagates is not part of the
/// contract this unit pins, matching the sibling tests
/// (`kin_cartesian.rs`, `kin_delta.rs`).
fn assert_f64_eq(ctx: &str, c: f64, r: f64) {
    if c.is_nan() || r.is_nan() {
        assert!(c.is_nan() && r.is_nan(), "{ctx}: NaN mismatch C={c:?} Rust={r:?}");
        return;
    }
    assert_eq!(c.to_bits(), r.to_bits(), "{ctx}: C={c:?} Rust={r:?}");
}

fn compare_from_coord(pair: &Pair, x: f64, y: f64, z: f64, ctx: &str) {
    let c = unsafe { itersolve_calc_position_from_coord(pair.c_sk, x, y, z) };
    let r = pair.rust_sk.calc_position_from_coord(x, y, z);
    assert_f64_eq(
        &format!("{ctx}: from_coord({x:?}, {y:?}, {z:?})"),
        c,
        r,
    );
}

fn compare_callback(pair: &Pair, m: &Move, t: f64, ctx: &str) {
    let rm = ref_move(m);
    let c = unsafe { c_cb(pair.c_sk)(pair.c_sk, &rm, t) };
    let r = (pair.rust_sk.calc_position_cb)(&pair.rust_sk, m, None, t);
    assert_f64_eq(
        &format!("{ctx}: calc_position(t={t:?}, start={:?}, axes={:?}, v={:?}, a={:?})", m.start_pos, m.axes_r, m.start_v, m.half_accel),
        c,
        r,
    );
}

#[test]
fn alloc_state_and_callback_selection() {
    for (kind, name) in [(b'+', "plus"), (b'-', "minus")] {
        let pair = Pair::new(kind);
        assert_alloc_state(&pair, name);
        let cb: StepperCalcCb = pair.rust_sk.calc_position_cb;
        let expected: StepperCalcCb = if kind == b'+' {
            port::corexy_stepper_plus_calc_position
        } else {
            port::corexy_stepper_minus_calc_position
        };
        assert!(std::ptr::fn_addr_eq(cb, expected), "{name}: callback");
        pair.teardown();
    }
}

#[test]
fn alloc_invalid_kind_keeps_null_callback_semantics() {
    // The C zeroes the allocation and leaves `calc_position_cb` null for
    // any kind but `'+'`/`'-'`; calling it faults. The port keeps the
    // allocation and faults on use.
    for kind in [0u8, b'x', b'z', b'.', 0xff] {
        let pair = Pair::new(kind);
        assert_alloc_state(&pair, "invalid kind");
        let c = unsafe { &*pair.c_sk };
        assert!(c.calc_position_cb.is_null(), "kind {kind:#x}: C callback must be null");
        let rust_sk = &pair.rust_sk;
        let cb: StepperCalcCb = rust_sk.calc_position_cb;
        let plus: StepperCalcCb = port::corexy_stepper_plus_calc_position;
        let minus: StepperCalcCb = port::corexy_stepper_minus_calc_position;
        assert!(!std::ptr::fn_addr_eq(cb, plus) && !std::ptr::fn_addr_eq(cb, minus), "kind {kind:#x}: must not select plus/minus");
        let m = Move::default();
        let out = catch_unwind(AssertUnwindSafe(|| cb(rust_sk, &m, None, 1.0)));
        assert!(out.is_err(), "kind {kind:#x}: null callback must fault");
        pair.teardown();
    }
}

#[test]
fn from_coord_matches() {
    let coords = [
        (0.0f64, 0.0f64, 0.0f64),
        (1.0, 2.0, 3.0),
        (-1.5, 2.5, -3.5),
        (0.0, -0.0, 1.0),
        (-0.0, -0.0, -0.0),
        (1e-300, -1e-300, 5e-324),
        (f64::MAX, -f64::MAX, f64::MIN_POSITIVE),
        (f64::INFINITY, 1.0, 0.0),
        (f64::INFINITY, f64::NEG_INFINITY, 0.0),
        (f64::NAN, 1.0, 0.0),
        (1.0, f64::NAN, 0.0),
        (-f64::INFINITY, -f64::INFINITY, f64::INFINITY),
    ];
    for kind in [b'+', b'-'] {
        let pair = Pair::new(kind);
        for (x, y, z) in coords {
            compare_from_coord(&pair, x, y, z, "corner");
        }
        // Deterministic pseudo-random coordinates (xorshift64).
        let mut state: u64 = 0x9e3779b97f4a7c15_u64 ^ u64::from(kind);
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for _ in 0..4096 {
            let x = (f64::from_bits(next()) * -2.0) + 1.0;
            let y = (f64::from_bits(next()) * -2.0) + 1.0;
            let z = f64::from_bits(next());
            compare_from_coord(&pair, x, y, z, "random");
        }
        pair.teardown();
    }
}

#[test]
fn callback_matches_arbitrary_moves() {
    let starts = [
        Coord::new(0.0, 0.0, 0.0),
        Coord::new(1.5, -2.5, 3.5),
        Coord::new(f64::INFINITY, -0.0, f64::NAN),
        Coord::new(-1e-300, 1e300, -5e-324),
    ];
    let axes = [
        Coord::new(0.0, 0.0, 0.0),
        Coord::new(1.0, 0.0, 0.0),
        Coord::new(-0.6, 0.8, 0.0),
        Coord::new(f64::NAN, f64::INFINITY, -1.0),
    ];
    let times = [0.0f64, -0.0, 0.125, -1.5, 1.0, 1000.0, f64::INFINITY, f64::NAN, -f64::INFINITY];
    let vels = [
        (0.0f64, 0.0f64),
        (1.0, 0.0),
        (-2.0, 0.5),
        (0.0, -3.0),
        (f64::NAN, f64::INFINITY),
        (-f64::INFINITY, -1e-300),
    ];
    for kind in [b'+', b'-'] {
        let pair = Pair::new(kind);
        for start in starts {
            for axes_r in axes {
                for t in times {
                    for (start_v, half_accel) in vels {
                        let m = Move {
                            print_time: 10.0,
                            move_t: 2.0,
                            start_v,
                            half_accel,
                            start_pos: start,
                            axes_r,
                        };
                        compare_callback(&pair, &m, t, "sweep");
                    }
                }
            }
        }
        pair.teardown();
    }
}

#[test]
fn set_position_and_active_axis() {
    let coords = [
        (0.0f64, 0.0f64, 0.0f64),
        (1.0, 2.0, 3.0),
        (-4.0, 0.5, 100.0),
        (f64::INFINITY, -f64::INFINITY, f64::NAN),
    ];
    for kind in [b'+', b'-'] {
        let mut pair = Pair::new(kind);
        for (x, y, z) in coords {
            unsafe { itersolve_set_position(pair.c_sk, x, y, z) };
            pair.rust_sk.set_position(x, y, z);
            let c = unsafe { itersolve_get_commanded_pos(pair.c_sk) };
            let r = pair.rust_sk.get_commanded_pos();
            assert_f64_eq(
                &format!("kind {kind}: commanded_pos after set_position({x:?}, {y:?}, {z:?})"),
                c,
                r,
            );
        }
        for axis in [b'w', b'x', b'y', b'z', b'{', 0u8] {
            let c = unsafe { itersolve_is_active_axis(pair.c_sk, axis as i8) };
            let r = pair.rust_sk.is_active_axis(axis);
            assert_eq!(c, r, "kind {kind}: is_active_axis({axis:#x}) C={c} Rust={r}");
        }
        pair.teardown();
    }
}
