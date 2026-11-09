//! Port of `klippy/chelper/kin_corexz.c`.
//!
//! CoreXZ kinematics: two steppers drive the X and Z axes together. The
//! `'+'` stepper tracks `x + z` and the `'-'` stepper tracks `x - z`, so
//! both report the same `AF_X | AF_Z` active flags. The callbacks read the
//! position from [`move_get_coord`] exactly as the C code does; floating
//! point is `f64` throughout, evaluated in the source order with no
//! `mul_add` or reassociation (see `parity/maps/map-motion.md` F4).
//!
//! The C `corexz_stepper_alloc` leaves `calc_position_cb` null for any
//! `type` other than `'+'`/`'-'`, but that state is unrepresentable in
//! [`StepperKinematics`] (its callback is never null) and no caller can
//! produce it: the only Python callers pass `b'+'`/`b'-'`
//! (`klippy/kinematics/corexz.py`, `klippy/kinematics/hybrid_corexz.py`).
//! Any other byte therefore installs a panicking stub, matching the
//! `kin_cartesian` invalid-axis rule (allocate, flags set, fault on use).
//! Note this differs from the C in one observable way: the C zeroes the
//! allocation and leaves `calc_position_cb` null, so a direct null check
//! sees null; the Rust struct cannot hold a null callback.

use crate::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Z};
use crate::trapq::{move_get_coord, Move};

/// `corexz_stepper_plus_calc_position` from `kin_corexz.c`: the stepper
/// position is `c.x + c.z` where `c` is the move position at `move_time`.
pub fn corexz_stepper_plus_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    c.x + c.z
}

/// `corexz_stepper_minus_calc_position` from `kin_corexz.c`: the stepper
/// position is `c.x - c.z` where `c` is the move position at `move_time`.
pub fn corexz_stepper_minus_calc_position(
    _sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    c.x - c.z
}
/// The `'+'`/`'-'` type bytes passed by the Python callers.
const COREXZ_PLUS: i8 = b'+' as i8;
const COREXZ_MINUS: i8 = b'-' as i8;

/// Stand-in for the null `calc_position_cb` that the C
/// `corexz_stepper_alloc` leaves for any other `type`: any use faults,
/// as calling a null function pointer does in the C.
fn corexz_stepper_invalid_calc_position(
    _sk: &StepperKinematics,
    _m: &Move,
    _at: Option<usize>,
    _move_time: f64,
) -> f64 {
    panic!("corexz_stepper_alloc: null calc_position_cb called");
}

/// `corexz_stepper_alloc` from `kin_corexz.c`: a zeroed
/// [`StepperKinematics`] over `AF_X | AF_Z`, with the `'+'`/`'-'` callback
/// selected by `ty` (`b'+'`/`b'-'` from the Python callers).
pub fn corexz_stepper_alloc(ty: i8) -> StepperKinematics {
    let calc_position_cb: StepperCalcCb = match ty {
        COREXZ_PLUS => corexz_stepper_plus_calc_position,
        COREXZ_MINUS => corexz_stepper_minus_calc_position,
        _ => corexz_stepper_invalid_calc_position,
    };
    StepperKinematics::new(calc_position_cb, AF_X | AF_Z)
}
