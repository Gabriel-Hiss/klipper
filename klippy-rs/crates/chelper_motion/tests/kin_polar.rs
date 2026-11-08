//! Differential parity tests for `chelper_motion::kin_polar`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - the `active_flags` and post-callback presence chosen by
//!   `polar_stepper_alloc`, plus the zeroed `stepper_kinematics`
//!   fields (`step_dist`, `commanded_pos`, flush/move times, and the
//!   `gen_steps_*` fields), including the unknown-type case where the
//!   C leaves the callback null and the port installs a panicking
//!   stub under the same flags (see `kin_cartesian.rs` precedent),
//! - the `calc_position_cb` result for the radius (`'r'`) and angle
//!   (`'a'`) steppers over hand-picked and seeded moves, including
//!   the origin, every `atan2` quadrant, and commanded angles at and
//!   just outside the `+/-pi` unwrapping boundaries, and
//! - the `post_cb` normalisation of the angle stepper for commanded
//!   angles at, just outside, and far outside `[-pi, pi]` (the C
//!   applies a single `2*pi` reduction, with no loop).
//!
//! `polar_stepper_alloc` is only reachable through `stepper.py`'s
//! `setup_itersolve`, so it never appears in the recordings under
//! `tests/fixtures/recordings/`; the reference C library, which is
//! the oracle for this C unit, is the parity proof (as for the
//! `itersolve` and `trapq` differential tests).
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_polar`
use chelper_motion::itersolve::{StepperCalcCb, StepperKinematics, StepperPostCb, AF_X, AF_Y};
use chelper_motion::kin_polar::{
    polar_stepper_alloc as rust_polar_alloc, polar_stepper_angle_calc_position,
    polar_stepper_angle_post_fixup, polar_stepper_radius_calc_position,
};
use chelper_motion::trapq::Move;

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h).
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

/// `struct move` (trapq.h).
#[repr(C)]
#[derive(Clone)]
struct RefMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    node: ListNode,
}

type RefCalcCb = unsafe extern "C" fn(*mut RefSk, *mut RefMove, f64) -> f64;
type RefPostCb = unsafe extern "C" fn(*mut RefSk);

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout.
#[repr(C)]
struct RefSk {
    step_dist: f64,
    commanded_pos: f64,
    sc: *mut core::ffi::c_void,
    last_flush_time: f64,
    last_move_time: f64,
    tq: *mut core::ffi::c_void,
    active_flags: i32,
    gen_steps_pre_active: f64,
    gen_steps_post_active: f64,
    calc_position_cb: Option<RefCalcCb>,
    post_cb: Option<RefPostCb>,
}

unsafe extern "C" {
    fn polar_stepper_alloc(type_: i8) -> *mut RefSk;
    fn free(ptr: *mut core::ffi::c_void);
}

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn move_from(p: &RefMove) -> Move {
    Move {
        print_time: p.print_time,
        move_t: p.move_t,
        start_v: p.start_v,
        half_accel: p.half_accel,
        start_pos: chelper_motion::trapq::Coord {
            x: p.start_pos[0],
            y: p.start_pos[1],
            z: p.start_pos[2],
        },
        axes_r: chelper_motion::trapq::Coord {
            x: p.axes_r[0],
            y: p.axes_r[1],
            z: p.axes_r[2],
        },
    }
}

fn ref_move(
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
) -> RefMove {
    RefMove {
        print_time,
        move_t,
        start_v,
        half_accel,
        start_pos,
        axes_r,
        node: ListNode {
            next: core::ptr::null_mut(),
            prev: core::ptr::null_mut(),
        },
    }
}

struct Pair {
    rust_sk: StepperKinematics,
    c_sk: *mut RefSk,
}

impl Pair {
    fn new(type_: u8) -> Pair {
        chelper_ref::link();
        let c_sk = unsafe { polar_stepper_alloc(type_ as i8) };
        assert!(!c_sk.is_null(), "polar_stepper_alloc returned null");
        Pair {
            rust_sk: rust_polar_alloc(type_),
            c_sk,
        }
    }

    fn c_calc(&self, m: &mut RefMove, move_time: f64) -> f64 {
        let cb = unsafe { (*self.c_sk).calc_position_cb };
        assert!(cb.is_some(), "reference calc_position_cb is null");
        unsafe { cb.unwrap()(self.c_sk, m as *mut RefMove, move_time) }
    }

    fn rust_calc(&self, m: &Move, move_time: f64) -> f64 {
        let cb: StepperCalcCb = self.rust_sk.calc_position_cb;
        cb(&self.rust_sk, m, move_time)
    }

    fn c_commanded(&self) -> f64 {
        unsafe { (*self.c_sk).commanded_pos }
    }

    fn set_commanded(&mut self, v: f64) {
        self.rust_sk.commanded_pos = v;
        unsafe { (*self.c_sk).commanded_pos = v };
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe { free(self.c_sk as *mut core::ffi::c_void) };
    }
}

fn bits(v: f64) -> u64 {
    v.to_bits()
}

/// Deterministic xorshift64* so the sweep is reproducible.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }

    /// A finite double in [-bound, bound].
    fn next_coord(&mut self, bound: f64) -> f64 {
        let u = (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
        u * 2.0 * bound - bound
    }
}

fn check_alloc_fields(type_: u8) {
    let zero = 0.0f64.to_bits();
    let p = Pair::new(type_);
    let c = unsafe { &*p.c_sk };
    assert_eq!(c.active_flags, AF_X | AF_Y, "active_flags for {type_:?}");
    assert_eq!(p.rust_sk.active_flags, AF_X | AF_Y);
    assert_eq!(bits(c.step_dist), zero, "step_dist zeroed");
    assert_eq!(bits(c.commanded_pos), zero, "commanded_pos zeroed");
    assert_eq!(bits(c.last_flush_time), zero);
    assert_eq!(bits(c.last_move_time), zero);
    assert_eq!(bits(c.gen_steps_pre_active), zero);
    assert_eq!(bits(c.gen_steps_post_active), zero);
    assert!(c.sc.is_null() && c.tq.is_null(), "sc/tq null after alloc");
}

fn check_move_both(
    type_: u8,
    commanded_pos: f64,
    m: &RefMove,
    move_times: &[f64],
) {
    let mut p = Pair::new(type_);
    p.set_commanded(commanded_pos);
    let rm = move_from(m);
    let mut cm = m.clone();
    for &t in move_times {
        let c_v = p.c_calc(&mut cm, t);
        let r_v = p.rust_calc(&rm, t);
        assert_eq!(
            bits(c_v),
            bits(r_v),
            "calc mismatch: type={:?} commanded={} t={} c={c_v:?} r={r_v:?}",
            as_char(type_),
            commanded_pos,
            t,
        );
    }
}

fn as_char(b: u8) -> char {
    b as char
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn alloc_radius_fields() {
    chelper_ref::link();
    check_alloc_fields(b'r');
    let p = Pair::new(b'r');
    let c = unsafe { &*p.c_sk };
    assert!(c.calc_position_cb.is_some(), "'r' sets a calc callback");
    assert!(c.post_cb.is_none(), "'r' leaves post_cb null");
    assert!(p.rust_sk.post_cb.is_none());
    assert!(
        std::ptr::fn_addr_eq(
            p.rust_sk.calc_position_cb,
            polar_stepper_radius_calc_position as StepperCalcCb
        ),
        "rust 'r' callback identity"
    );
}

#[test]
fn alloc_angle_fields() {
    chelper_ref::link();
    check_alloc_fields(b'a');
    let p = Pair::new(b'a');
    let c = unsafe { &*p.c_sk };
    assert!(c.calc_position_cb.is_some(), "'a' sets a calc callback");
    assert!(c.post_cb.is_some(), "'a' sets post_cb");
    assert!(p.rust_sk.post_cb.is_some());
    assert!(
        std::ptr::fn_addr_eq(
            p.rust_sk.calc_position_cb,
            polar_stepper_angle_calc_position as StepperCalcCb
        ),
        "rust 'a' callback identity"
    );
    assert!(
        std::ptr::fn_addr_eq(
            p.rust_sk.post_cb.unwrap(),
            polar_stepper_angle_post_fixup as StepperPostCb
        ),
        "rust 'a' post callback identity"
    );
}

#[test]
fn alloc_unknown_type_fields() {
    chelper_ref::link();
    // The C leaves calc_position_cb null for unknown types while still
    // setting active_flags (line 57 is outside the `if`). The port cannot
    // hold a null callback, so it installs a panicking callback under the
    // same flags (see `kin_cartesian.rs` precedent); assert the stub is
    // distinct from both real callbacks and panics on first use.
    for type_ in [b'x', b'z', 0u8, 0xff] {
        check_alloc_fields(type_);
        let p = Pair::new(type_);
        let c = unsafe { &*p.c_sk };
        assert!(
            c.calc_position_cb.is_none(),
            "unknown type {type_:?} leaves C callback null"
        );
        assert!(c.post_cb.is_none(), "unknown type {type_:?} post null");
        assert!(p.rust_sk.post_cb.is_none());
        assert_eq!(p.rust_sk.active_flags, AF_X | AF_Y);
        assert!(
            !std::ptr::fn_addr_eq(
                p.rust_sk.calc_position_cb,
                polar_stepper_radius_calc_position as StepperCalcCb
            ),
            "unknown type {type_:?}: stub must not be the radius callback"
        );
        assert!(
            !std::ptr::fn_addr_eq(
                p.rust_sk.calc_position_cb,
                polar_stepper_angle_calc_position as StepperCalcCb
            ),
            "unknown type {type_:?}: stub must not be the angle callback"
        );
        let rm = move_from(&ref_move(0.0, 1.0, 0.0, 0.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]));
        let cb = p.rust_sk.calc_position_cb;
        let sk = &p.rust_sk;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            cb(sk, &rm, 0.0)
        }));
        let err = result.expect_err("unknown-type callback must panic (C null crash)");
        let msg = err
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| err.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("");
        assert!(
            msg.contains("null calc_position_cb"),
            "unknown type {type_:?}: panic message must name null callback, got {msg:?}"
        );
    }
}

#[test]
#[should_panic(expected = "null calc_position_cb")]
fn unknown_type_callback_panics() {
    let sk = rust_polar_alloc(b'x');
    let m = Move::default();
    let _ = (sk.calc_position_cb)(&sk, &m, 0.0);
}

#[test]
fn radius_matches_c() {
    chelper_ref::link();
    // The second command of the pair is at the origin; the C has a
    // documented XXX for x==y==0 on the angle callback only, so the
    // radius must produce 0.0 there with no special-casing.
    let moves = [
        ref_move(0.0, 1.0, 0.0, 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        (ref_move(5.0, 2.0, 10.0, -5.0, [3.0, 4.0, 1.0], [0.6, 0.8, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-3.0, -4.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 10.0, 1.0, 2.0, [-100.0, 50.0, 0.0], [0.5, -0.5, 0.0])),
    ];
    for m in &moves {
        check_move_both(b'r', 0.0, m, &[0.0, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, -1.0]);
    }
    // Origin, exactly: radius is exactly 0.0.
    let p = Pair::new(b'r');
    let mut cm = moves[0].clone();
    assert_eq!(bits(p.c_calc(&mut cm, 0.0)), bits(0.0));
    assert_eq!(bits(p.rust_calc(&move_from(&moves[0]), 0.0)), bits(0.0));
}

#[test]
fn angle_matches_c() {
    chelper_ref::link();
    let moves = [
        // atan2 over all four quadrants, on the axes, and at the
        // origin (the C XXX: atan2(0, 0) == 0 in glibc).
        ref_move(0.0, 1.0, 0.0, 0.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [0.0, 1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [0.0, -1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [1.0, 1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, -1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [1.0, -1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])),
        // Moving (interpolated positions, including negative time).
        (ref_move(5.0, 2.0, 10.0, -5.0, [3.0, 4.0, 1.0], [0.6, 0.8, 0.0])),
        (ref_move(0.0, 10.0, 1.0, 2.0, [-100.0, 50.0, 0.0], [0.5, -0.5, 0.0])),
        // Very small and very large radii.
        (ref_move(0.0, 1.0, 0.0, 0.0, [1e-300, 1e-300, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [1e300, -1e300, 0.0], [0.0, 0.0, 0.0])),
    ];
    for m in &moves {
        check_move_both(b'a', 0.0, m, &[0.0, 0.25, 0.5, 1.0, 2.0, 5.0, -1.0]);
    }
}

#[test]
fn angle_unwrapping_matches_c() {
    chelper_ref::link();
    // Points just above/below the branch cut so atan2 returns
    // angles just below/above +/-pi, crossed with commanded angles
    // on both sides and at the strict-comparison boundaries.
    let targets = [
        ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, 1e-8, 0.0], [0.0, 0.0, 0.0]),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, -1e-8, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [-1.0, -1.0, 0.0], [0.0, 0.0, 0.0])),
        (ref_move(0.0, 1.0, 0.0, 0.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0])),
    ];
    let commanded = [
        0.0,
        PI,
        -PI,
        PI - 1e-9,
        -PI + 1e-9,
        PI + 1e-9,
        -PI - 1e-9,
        2.0 * PI,
        -2.0 * PI,
        100.0,
        -100.0,
    ];
    for m in &targets {
        for &cmd in &commanded {
            check_move_both(b'a', cmd, m, &[0.0]);
        }
    }
}

#[test]
fn angle_post_fixup_matches_c() {
    chelper_ref::link();
    let values = [
        0.0,
        PI,
        -PI,
        PI - 1e-12,
        -PI + 1e-12,
        PI + 1e-12,
        -PI - 1e-12,
        PI.next_up(),
        (-PI).next_down(),
        2.0 * PI,
        -2.0 * PI,
        3.0 * PI,
        -3.0 * PI,
        100.0,
        -100.0,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ];
    for &v in &values {
        let mut p = Pair::new(b'a');
        p.set_commanded(v);
        let cb = unsafe { (*p.c_sk).post_cb };
        assert!(cb.is_some(), "angle post_cb is set");
        unsafe { cb.unwrap()(p.c_sk) };
        let c_after = p.c_commanded();
        polar_stepper_angle_post_fixup(&mut p.rust_sk);
        let r_after = p.rust_sk.commanded_pos;
        assert_eq!(
            bits(c_after),
            bits(r_after),
            "post fixup mismatch for commanded={v:?} c={c_after:?} r={r_after:?}"
        );
    }
}

#[test]
fn seeded_moves_match_c() {
    chelper_ref::link();
    for seed in [1u64, 42, 0xdeadbeefcafe, 987654321] {
        let mut rng = Rng(seed);
        for _ in 0..64 {
            let m = ref_move(
                rng.next_coord(5.0),
                rng.next_coord(5.0).abs() + 0.1,
                rng.next_coord(50.0),
                rng.next_coord(50.0),
                [
                    rng.next_coord(200.0),
                    rng.next_coord(200.0),
                    rng.next_coord(20.0),
                ],
                [
                    rng.next_coord(1.0),
                    rng.next_coord(1.0),
                    rng.next_coord(1.0),
                ],
            );
            for &t in &[
                rng.next_coord(12.0),
                rng.next_coord(12.0),
                rng.next_coord(12.0),
            ] {
                check_move_both(b'r', 0.0, &m, &[t]);
                // Cross the unwrapping branches with a random
                // commanded angle, including near the cut.
                let cmd = if rng.next_u64().is_multiple_of(2) {
                    rng.next_coord(2.0 * PI + 2.0)
                } else {
                    PI + rng.next_coord(1.0)
                };
                check_move_both(b'a', cmd, &m, &[t]);
            }
        }
    }
}
