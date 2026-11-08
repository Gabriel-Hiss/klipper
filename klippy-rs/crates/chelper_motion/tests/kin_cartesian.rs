//! Differential parity tests for `chelper_motion::kin_cartesian`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - `cartesian_stepper_alloc` for every byte value: the zeroed
//!   `StepperKinematics` fields (`step_dist`, `commanded_pos`, the
//!   null `sc`/`tq`/`post_cb`, `last_flush_time`, `last_move_time`,
//!   the `gen_steps_*` fields, `active_flags`) and the installed
//!   callback,
//! - the three `cart_stepper_*_calc_position` callbacks against the
//!   `move_get_coord` formula of `trapq.c` over representative moves and
//!   move times (including zero moves, single-axis and diagonal moves,
//!   negative and past-the-end times, signed zeros), and
//! - the `__visible` `itersolve` entry points resolved through the C
//!   callbacks (`itersolve_calc_position_from_coord`,
//!   `itersolve_set_position` / `itersolve_get_commanded_pos`,
//!   `itersolve_is_active_axis`), which observe the C callbacks exactly
//!   as the solver does.
//!
//! The C callbacks are `static` in `kin_cartesian.c` and observable only
//! through the `itersolve` entry points; the arbitrary-move callback
//! values are therefore pinned by the `move_get_coord` formula (the same
//! formula `trapq.rs` is proven to match bit for bit against the C
//! library in the `trapq` tests), and the through-`itersolve` probes pin
//! the C callbacks themselves.
//!
//! Recordings in `tests/fixtures/recordings/kin_cartesian/` record only
//! `cartesian_stepper_alloc` calls (the sole `__visible` entry point of
//! this unit); `recorded_replay` replays every one of them through both
//! the reference and the port and compares the allocated state.
//!
//! `pyoracle` does not apply here: the unit is C, and the reference is
//! the `chelper_ref` C library.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_cartesian`
use chelper_motion::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Y, AF_Z};
use chelper_motion::kin_cartesian::{
    cart_stepper_x_calc_position, cart_stepper_y_calc_position, cart_stepper_z_calc_position,
    cartesian_stepper_alloc as rust_cartesian_stepper_alloc,
};
use chelper_motion::trapq::{Coord, Move};
use core::ffi::c_void;
use recording::{load_file, record_files, Record, Value};
use std::path::PathBuf;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h).
// ---------------------------------------------------------------------------

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout. Only used to
/// read back the state the C allocator creates.
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

unsafe extern "C" {
    fn cartesian_stepper_alloc(axis: i8) -> *mut RefSk;
    fn free(ptr: *mut c_void);
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
    fn itersolve_is_active_axis(sk: *mut RefSk, axis: i8) -> i32;
}

// ---------------------------------------------------------------------------
// Comparison helpers.
// ---------------------------------------------------------------------------

/// Bit-for-bit comparison, with `NaN == NaN` for inputs that propagate a
/// `NaN` through the formula (the payload of a propagated `NaN` is not
/// the part of the contract this unit pins).
fn assert_bits_eq(ctx: &str, c: f64, r: f64) {
    if c.is_nan() && r.is_nan() {
        return;
    }
    assert_eq!(c.to_bits(), r.to_bits(), "{ctx}: C={c:?} Rust={r:?}");
}

fn assert_sk_state_match(ctx: &str, c: &RefSk, r: &StepperKinematics) {
    assert_eq!(
        c.step_dist.to_bits(),
        r.step_dist.to_bits(),
        "{ctx}: step_dist C={:?} Rust={:?}",
        c.step_dist,
        r.step_dist
    );
    assert_eq!(
        c.commanded_pos.to_bits(),
        r.commanded_pos.to_bits(),
        "{ctx}: commanded_pos C={:?} Rust={:?}",
        c.commanded_pos,
        r.commanded_pos
    );
    assert!(c.sc.is_null(), "{ctx}: C sc is not null");
    assert!(r.sc.is_none(), "{ctx}: Rust sc is not None");
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
    assert!(c.tq.is_null(), "{ctx}: C tq is not null");
    assert!(r.tq.is_none(), "{ctx}: Rust tq is not None");
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
    assert!(c.post_cb.is_null(), "{ctx}: C post_cb is not null");
    assert!(r.post_cb.is_none(), "{ctx}: Rust post_cb is not None");
}

fn expected_callback(axis: u8) -> Option<StepperCalcCb> {
    match axis {
        b'x' => Some(cart_stepper_x_calc_position),
        b'y' => Some(cart_stepper_y_calc_position),
        b'z' => Some(cart_stepper_z_calc_position),
        _ => None,
    }
}

/// Allocate a matching pair through the C and the Rust allocator and
/// compare the full allocated state, including which callback is
/// installed. The C side must be freed by the caller.
fn check_alloc(axis: u8, ctx: &str) -> (*mut RefSk, StepperKinematics) {
    chelper_ref::link();
    let c_ptr = unsafe { cartesian_stepper_alloc(axis as i8) };
    assert!(!c_ptr.is_null(), "{ctx}: C cartesian_stepper_alloc returned null");
    let rust_sk = rust_cartesian_stepper_alloc(axis);
    let c = unsafe { &*c_ptr };
    assert_sk_state_match(ctx, c, &rust_sk);
    match expected_callback(axis) {
        None => {
            // Axis other than x/y/z: the C struct carries a null callback
            // and no active flags. The Rust port cannot hold a null
            // callback (see the `itersolve` contract); it installs a
            // panicking stub under the same zero flags, so the stub must
            // not be one of the three real callbacks.
            assert!(c.calc_position_cb.is_null(), "{ctx}: C calc_position_cb is not null");
            assert!(!std::ptr::fn_addr_eq(rust_sk.calc_position_cb,
                cart_stepper_x_calc_position as StepperCalcCb),
                "{ctx}: invalid-axis stub must not be the x callback");
            assert!(!std::ptr::fn_addr_eq(rust_sk.calc_position_cb,
                cart_stepper_y_calc_position as StepperCalcCb),
                "{ctx}: invalid-axis stub must not be the y callback");
            assert!(!std::ptr::fn_addr_eq(rust_sk.calc_position_cb,
                cart_stepper_z_calc_position as StepperCalcCb),
                "{ctx}: invalid-axis stub must not be the z callback");
        }
        Some(want) => {
            assert!(!c.calc_position_cb.is_null(), "{ctx}: C calc_position_cb is null");
            assert!(std::ptr::fn_addr_eq(rust_sk.calc_position_cb, want),
                "{ctx}: installed callback mismatch");
        }
    }
    (c_ptr, rust_sk)
}

// ---------------------------------------------------------------------------
// Allocation.
// ---------------------------------------------------------------------------

#[test]
fn alloc_matches_c_for_all_axes() {
    for axis in 0u8..=255 {
        let ctx = format!("axis {axis} (0x{axis:02x} {axis:?})");
        let (c_ptr, _rust_sk) = check_alloc(axis, &ctx);
        unsafe { free(c_ptr as *mut c_void) };
    }
    // The three documented axes also pin their exact flag values.
    for (axis, flags) in [(b'x', AF_X), (b'y', AF_Y), (b'z', AF_Z)] {
        let sk = rust_cartesian_stepper_alloc(axis);
        assert_eq!(sk.active_flags, flags, "axis {axis}: active_flags");
    }
}

// ---------------------------------------------------------------------------
// The three callbacks against the `move_get_coord` formula.
// ---------------------------------------------------------------------------

fn zeroed_sk() -> StepperKinematics {
    rust_cartesian_stepper_alloc(b'x')
}


const MOVES: &[(&str, Move)] = &[
    (
        "zero move",
        Move {
            print_time: 0.0,
            move_t: 0.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Coord::new(0.0, 0.0, 0.0),
            axes_r: Coord::new(0.0, 0.0, 0.0),
        },
    ),
    (
        "degenerate at position",
        Move {
            print_time: 1.0,
            move_t: 1.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Coord::new(10.5, -3.25, 7.0),
            axes_r: Coord::new(0.0, 0.0, 0.0),
        },
    ),
    (
        "pure X cruise",
        Move {
            print_time: 1.0,
            move_t: 2.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Coord::new(0.0, 5.0, -5.0),
            axes_r: Coord::new(1.0, 0.0, 0.0),
        },
    ),
    (
        "pure Y accel",
        Move {
            print_time: 2.0,
            move_t: 0.5,
            start_v: 10.0,
            half_accel: 50.0,
            start_pos: Coord::new(1.0, 2.0, 3.0),
            axes_r: Coord::new(0.0, 1.0, 0.0),
        },
    ),
    (
        "pure Z decel",
        Move {
            print_time: 5.0,
            move_t: 1.5,
            start_v: 30.0,
            half_accel: -20.0,
            start_pos: Coord::new(-1.0, -2.0, 100.0),
            axes_r: Coord::new(0.0, 0.0, -1.0),
        },
    ),
    (
        "diagonal",
        Move {
            print_time: 0.0,
            move_t: 4.0,
            start_v: 12.5,
            half_accel: 37.5,
            start_pos: Coord::new(300.0, 0.5, -0.5),
            axes_r: Coord::new(0.5, 0.5, 0.7071067811865475),
        },
    ),
    (
        "negative velocity and accel",
        Move {
            print_time: 10.0,
            move_t: 3.0,
            start_v: -20.0,
            half_accel: -10.0,
            start_pos: Coord::new(-110.0, 20.0, 337.5),
            axes_r: Coord::new(-1.0, 1.0, 0.0),
        },
    ),
    (
        "signed zeros",
        Move {
            print_time: -1.0,
            move_t: 1.0,
            start_v: -0.0,
            half_accel: 0.0,
            start_pos: Coord::new(-0.0, 0.0, -0.0),
            axes_r: Coord::new(-0.0, 0.0, -0.0),
        },
    ),
];

const MOVE_TIMES: &[f64] = &[
    -1.0,
    -0.001,
    -0.0,
    0.0,
    0.001,
    0.25,
    0.5,
    1.0,
    2.0,
    3.0,
    4.0,
    10.0,
    100.0,
];

#[test]
fn callbacks_match_move_get_coord_formula() {
    let sk = zeroed_sk();
    for (name, m) in MOVES {
        for t in MOVE_TIMES {
            // `move_get_coord` from trapq.c, in source order:
            // `(start_v + half_accel * move_time) * move_time`, then
            // `start_pos + axes_r * move_dist` per axis.
            let move_dist = (m.start_v + m.half_accel * t) * t;
            let ex = m.start_pos.x + m.axes_r.x * move_dist;
            let ey = m.start_pos.y + m.axes_r.y * move_dist;
            let ez = m.start_pos.z + m.axes_r.z * move_dist;
            let ctx = format!("{name} at move_time {t}");
            assert_bits_eq(&format!("{ctx}: x"), ex, cart_stepper_x_calc_position(&sk, m, *t));
            assert_bits_eq(&format!("{ctx}: y"), ey, cart_stepper_y_calc_position(&sk, m, *t));
            assert_bits_eq(&format!("{ctx}: z"), ez, cart_stepper_z_calc_position(&sk, m, *t));
        }
    }
    // The callbacks ignore the stepper state (the `sk` argument carries
    // no per-axis state in this module): two differently allocated
    // steppers must return the same value.
    let m = MOVES[5].1;
    let t = 0.25;
    for axis in [b'x', b'y', b'z'] {
        let reference = cart_stepper_x_calc_position(&rust_cartesian_stepper_alloc(axis), &m, t);
        assert_bits_eq(
            "sk ignored",
            cart_stepper_x_calc_position(&sk, &m, t),
            reference,
        );
    }
}

// ---------------------------------------------------------------------------
// Through-`itersolve` probes against the C callbacks.
// ---------------------------------------------------------------------------

const COORDS: &[(f64, f64, f64)] = &[
    (0.0, 0.0, 0.0),
    (-0.0, 0.0, -0.0),
    (3.5, 1.25, -2.0),
    (-110.0, -20.0, 300.0),
    (337.5, -337.5, 0.5),
    (f64::MIN_POSITIVE, -f64::MIN_POSITIVE, 1.0),
    (f64::from_bits(1), f64::from_bits(u64::MAX >> 1), -1.0),
    (f64::INFINITY, f64::NEG_INFINITY, 1.0),
    (f64::MAX, -f64::MAX, 0.25),
    (1.7976931348623157e308, -1.7976931348623157e308, -4.9e-324),
];

#[test]
fn calc_position_from_coord_matches_c() {
    chelper_ref::link();
    for axis in [b'x', b'y', b'z'] {
        let (c_ptr, rust_sk) = check_alloc(axis, &format!("axis {axis}"));
        for (x, y, z) in COORDS {
            let c = unsafe { itersolve_calc_position_from_coord(c_ptr, *x, *y, *z) };
            let r = rust_sk.calc_position_from_coord(*x, *y, *z);
            assert_bits_eq(
                &format!("axis {axis} calc_position_from_coord({x}, {y}, {z})"),
                c,
                r,
            );
        }
        unsafe { free(c_ptr as *mut c_void) };
    }
}

#[test]
fn set_position_and_is_active_axis_match_c() {
    chelper_ref::link();
    for axis in [b'x', b'y', b'z'] {
        let (c_ptr, mut rust_sk) = check_alloc(axis, &format!("axis {axis}"));
        for (x, y, z) in COORDS {
            unsafe { itersolve_set_position(c_ptr, *x, *y, *z) };
            rust_sk.set_position(*x, *y, *z);
            let c = unsafe { itersolve_get_commanded_pos(c_ptr) };
            let r = rust_sk.get_commanded_pos();
            assert_bits_eq(
                &format!("axis {axis} set_position({x}, {y}, {z}) commanded_pos"),
                c,
                r,
            );
        }
        for probe in [b'x', b'y', b'z', b'a', b'X', b'w', 0x00, 0xff] {
            let c = unsafe { itersolve_is_active_axis(c_ptr, probe as i8) };
            let r = rust_sk.is_active_axis(probe);
            assert_eq!(c, r, "axis {axis}: is_active_axis({probe})");
        }
        unsafe { free(c_ptr as *mut c_void) };
    }
}

// ---------------------------------------------------------------------------
// Recorded sequences.
// ---------------------------------------------------------------------------

fn recordings_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../tests/fixtures/recordings")
}

#[test]
fn recorded_replay() {
    chelper_ref::link();
    let root = recordings_root();
    let files = record_files(&root, "kin_cartesian").expect("list kin_cartesian recordings");
    assert!(!files.is_empty(), "no kin_cartesian recordings found under {root:?}");
    let mut total = 0usize;
    for file in &files {
        let records = load_file(file).expect("parse recording");
        replay_one(&records, file);
        total += records.len();
    }
    assert!(total > 0, "kin_cartesian recordings are empty");
}

fn replay_one(records: &[Record], file: &std::path::Path) {
    let ctx = format!("{} ", file.display());
    for rec in records {
        assert_eq!(rec.unit, "kin_cartesian", "{ctx}seq {}: unit", rec.seq);
        assert_eq!(
            rec.function, "cartesian_stepper_alloc",
            "{ctx}seq {}: function",
            rec.seq
        );
        assert!(rec.exc.is_none(), "{ctx}seq {}: unexpected exception", rec.seq);
        assert!(
            rec.side_effects.mutated_args.is_empty()
                && rec.side_effects.mcu_messages.is_empty()
                && rec.side_effects.log_lines.is_empty(),
            "{ctx}seq {}: unexpected side effects",
            rec.seq
        );
        assert_eq!(rec.args.len(), 1, "{ctx}seq {}: arg count", rec.seq);
        let axis: u8 = match &rec.args[0] {
            Value::Bytes(bytes) => {
                assert_eq!(bytes.len(), 1, "{ctx}seq {}: axis byte length", rec.seq);
                bytes[0]
            }
            other => panic!("{ctx}seq {}: axis argument {other:?}", rec.seq),
        };
        // The recorded return is an opaque pointer; the observable state
        // is the allocated struct, compared against the live reference.
        match &rec.ret {
            Some(Value::Opaque { .. }) => {}
            other => panic!("{ctx}seq {}: return {other:?}", rec.seq),
        }
        let (c_ptr, _rust_sk) = check_alloc(axis, &format!("{ctx}seq {} axis {axis}", rec.seq));
        unsafe { free(c_ptr as *mut c_void) };
    }
}

