//! Port of `klippy/chelper/kin_cartesian.c`.

use crate::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Y, AF_Z};
use crate::trapq::{move_get_coord, Move};

/// `cart_stepper_x_calc_position` from `kin_cartesian.c`: the X coordinate
/// of `m` at `move_time` seconds into the move.
pub fn cart_stepper_x_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    move_time: f64,
) -> f64 {
    move_get_coord(m, move_time).x
}

/// `cart_stepper_y_calc_position` from `kin_cartesian.c`: the Y coordinate
/// of `m` at `move_time` seconds into the move.
pub fn cart_stepper_y_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    move_time: f64,
) -> f64 {
    move_get_coord(m, move_time).y
}

/// `cart_stepper_z_calc_position` from `kin_cartesian.c`: the Z coordinate
/// of `m` at `move_time` seconds into the move.
pub fn cart_stepper_z_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    move_time: f64,
) -> f64 {
    move_get_coord(m, move_time).z
}

/// Stand-in for the null `calc_position_cb` that the C
/// `cartesian_stepper_alloc` leaves for an axis other than `x`, `y`, or
/// `z`. Calling a null function pointer in the C reference crashes; this
/// callback panics instead, so the crash happens at first use, as in the
/// reference, rather than silently returning a position.
fn cart_stepper_invalid_calc_position(
    _sk: &StepperKinematics,
    _m: &Move,
    _move_time: f64,
) -> f64 {
    panic!("cart_stepper_invalid_calc_position: null calc_position_cb in C reference")
}

/// `cartesian_stepper_alloc` from `kin_cartesian.c`: a zeroed
/// [`StepperKinematics`] with the per-axis position callback and active
/// flag for `b'x'`, `b'y'`, or `b'z'`.
///
/// For any other axis byte the C code returns the zeroed struct with a null
/// callback and `active_flags = 0` (`memset` and no branch matches). The
/// Rust struct cannot hold a null callback, so [`StepperKinematics::new`]
/// is given [`cart_stepper_invalid_calc_position`], which panics if ever
/// invoked, matching the C crash on first use. `klippy` only ever passes
/// `x`, `y`, and `z` here (see `klippy/kinematics/cartesian.py` and
/// `klippy/stepper.py` `setup_itersolve`).
pub fn cartesian_stepper_alloc(axis: u8) -> StepperKinematics {
    match axis {
        b'x' => StepperKinematics::new(cart_stepper_x_calc_position as StepperCalcCb, AF_X),
        b'y' => StepperKinematics::new(cart_stepper_y_calc_position as StepperCalcCb, AF_Y),
        b'z' => StepperKinematics::new(cart_stepper_z_calc_position as StepperCalcCb, AF_Z),
        _ => StepperKinematics::new(cart_stepper_invalid_calc_position as StepperCalcCb, 0),
    }
}
