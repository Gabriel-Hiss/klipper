//! Port of `klippy/chelper/kin_corexy.c`.
//!
//! CoreXY steppers are driven by the X/Y coordinate sums: the `+` stepper
//! reports `x + y` and the `-` stepper reports `x - y`, both evaluated
//! with [`move_get_coord`] at `move_time` into the move. The C
//! `corexy_stepper_alloc` zeroes the allocation and only sets
//! `calc_position_cb` and `active_flags = AF_X | AF_Y`; the Rust
//! [`StepperKinematics::new`] has the same zeroed defaults, so
//! [`corexy_stepper_alloc`] is a plain constructor.
//!
//! For a `kind` other than `b'+'` / `b'-'` the C keeps a null
//! `calc_position_cb` (only `klippy/kinematics/corexy.py` calls this, with
//! exactly `b'+'` and `b'-'`, so the null pointer is never dereferenced in
//! practice). [`StepperKinematics`] cannot hold a null callback, so the
//! port falls back to a callback that panics on use, which preserves the
//! C behaviour that the allocation succeeds and any use faults.
//!
//! Floating point matches the C: `f64`, one addition/subtraction of two
//! coordinates, no reassociation or fused operations.

use crate::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Y};
use crate::trapq::{move_get_coord, Move};

/// `corexy_stepper_plus_calc_position` from `kin_corexy.c`: `x + y` of
/// the move's coordinate at `move_time`.
pub fn corexy_stepper_plus_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    c.x + c.y
}

/// `corexy_stepper_minus_calc_position` from `kin_corexy.c`: `x - y` of
/// the move's coordinate at `move_time`.
pub fn corexy_stepper_minus_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    c.x - c.y
}

/// Stand-in for a null `calc_position_cb`: any use faults, as calling a
/// null function pointer does in the C.
fn corexy_stepper_null_calc_position(
    _sk: &StepperKinematics,
    _m: &Move,
    _at: Option<usize>,
    _move_time: f64,
) -> f64 {
    panic!("corexy_stepper_alloc: null calc_position_cb called");
}

/// `corexy_stepper_alloc` from `kin_corexy.c`: `b'+'` selects the
/// `x + y` callback, `b'-'` selects `x - y`; both set
/// `active_flags = AF_X | AF_Y` on an otherwise zeroed stepper.
pub fn corexy_stepper_alloc(kind: u8) -> StepperKinematics {
    let calc_position_cb: StepperCalcCb = match kind {
        b'+' => corexy_stepper_plus_calc_position,
        b'-' => corexy_stepper_minus_calc_position,
        _ => corexy_stepper_null_calc_position,
    };
    StepperKinematics::new(calc_position_cb, AF_X | AF_Y)
}
