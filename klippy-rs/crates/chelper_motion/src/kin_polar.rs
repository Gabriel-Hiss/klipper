//! Port of `klippy/chelper/kin_polar.c`.
//!
//! The C module holds two stepper-kinematics callbacks for a polar
//! machine (see `klippy/kinematics/polar.py`):
//!
//! - the arm stepper (`type == 'r'`) reports the radial distance from
//!   the bed centre: `sqrt(x*x + y*y)` of the interpolated move
//!   position,
//! - the bed stepper (`type == 'a'`) reports the polar angle
//!   `atan2(y, x)` of the interpolated position, unwrapped against
//!   the previous commanded angle so that a rotation across the
//!   `-pi`/`pi` branch cut does not jump, and normalised with one
//!   `2*pi` reduction after each step range.
//!
//! The C `sk->active_flags = AF_X | AF_Y` applies to both steppers.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations in the exact source order. `atan2` goes
//! through `kos::libm` (the system libm, as C `atan2` does) and `sqrt`
//! uses `f64::sqrt` (correctly rounded, as the C `sqrt` is).

use std::f64::consts::PI;

use kos::libm;

use crate::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Y};
use crate::trapq::{move_get_coord, Move};

/// `polar_stepper_radius_calc_position` from `kin_polar.c`.
///
/// The radial distance of the move position `move_time` seconds into
/// `m`: `sqrt(c.x*c.x + c.y*c.y)` of `move_get_coord(m, move_time)`.
/// The C `XXX - handle x==y==0` comment on the angle callback does
/// not apply here: the origin yields `0.0` with no special-casing.
pub fn polar_stepper_radius_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    (c.x * c.x + c.y * c.y).sqrt()
}

/// `polar_stepper_angle_calc_position` from `kin_polar.c`.
///
/// The polar angle `atan2(y, x)` of the move position `move_time`
/// seconds into `m`, shifted by `2*pi` when it differs from the last
/// commanded angle by more than `pi` in either direction, so the
/// solver sees a continuous angle across the branch cut.
pub fn polar_stepper_angle_calc_position(
    sk: &StepperKinematics,
    m: &Move,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    // XXX - handle x==y==0
    let mut angle = libm::atan2(c.y, c.x);
    if angle - sk.commanded_pos > PI {
        angle -= 2. * PI;
    } else if angle - sk.commanded_pos < -PI {
        angle += 2. * PI;
    }
    angle
}

/// `polar_stepper_angle_post_fixup` from `kin_polar.c`.
///
/// Normalises the commanded angle by a single `2*pi` reduction
/// after the solver updates it (a single step, as in the C: no
/// loop, so an angle far outside `[-pi, pi]` stays outside).
pub fn polar_stepper_angle_post_fixup(sk: &mut StepperKinematics) {
    if sk.commanded_pos < -PI {
        sk.commanded_pos += 2.0 * PI;
    } else if sk.commanded_pos > PI {
        sk.commanded_pos -= 2.0 * PI;
    }
}

/// Stand-in for the null `calc_position_cb` that the C
/// `polar_stepper_alloc` leaves for a type other than `'r'` or `'a'`.
/// Calling a null function pointer in the C reference crashes; this
/// callback panics instead, so the crash happens at first use, as in
/// the reference, rather than silently returning a position.
fn polar_stepper_invalid_calc_position(
    _sk: &StepperKinematics,
    _m: &Move,
    _move_time: f64,
) -> f64 {
    panic!("polar_stepper_invalid_calc_position: null calc_position_cb in C reference")
}

/// `polar_stepper_alloc` from `kin_polar.c`.
///
/// Mirrors the C allocation: a zeroed [`StepperKinematics`] with
/// `active_flags = AF_X | AF_Y`, the radius callback for `type ==
/// 'r'` and the angle callback plus the post-range normalisation
/// for `type == 'a'`.
///
/// For any other type byte the C code leaves `calc_position_cb` null
/// (neither branch matches) but still sets `active_flags` (line 57 is
/// outside the `if`). The Rust struct cannot hold a null callback,
/// so [`StepperKinematics::new`] is given
/// [`polar_stepper_invalid_calc_position`], which panics if ever
/// invoked, matching the C crash on first use. `klippy` only ever
/// passes `a` and `r` here (see `klippy/kinematics/polar.py` and
/// `klippy/stepper.py` `setup_itersolve`).
pub fn polar_stepper_alloc(type_: u8) -> StepperKinematics {
    match type_ {
        b'r' => StepperKinematics::new(
            polar_stepper_radius_calc_position as StepperCalcCb,
            AF_X | AF_Y,
        ),
        b'a' => {
            let mut sk = StepperKinematics::new(
                polar_stepper_angle_calc_position as StepperCalcCb,
                AF_X | AF_Y,
            );
            sk.post_cb = Some(polar_stepper_angle_post_fixup);
            sk
        }
        _ => StepperKinematics::new(
            polar_stepper_invalid_calc_position as StepperCalcCb,
            AF_X | AF_Y,
        ),
    }
}
