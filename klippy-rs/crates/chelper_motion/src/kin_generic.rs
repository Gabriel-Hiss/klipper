//! Port of `klippy/chelper/kin_generic.c`: generic cartesian kinematics
//! stepper position calculation.
//!
//! A generic stepper reports the linear combination
//! `a.x * x + a.y * y + a.z * z` of the move coordinates, where the
//! coefficients are those last set by
//! [`generic_cartesian_stepper_set_coeffs`] (C:
//! [`GenericCartesianStepper::set_coeffs`]). The active axes are those
//! with a nonzero coefficient.
//!
//! The C struct `generic_cartesian_stepper` embeds `struct
//! stepper_kinematics` first and recovers the outer struct from the `sk`
//! pointer with `container_of`. The Rust port mirrors this with
//! [`GenericCartesianStepper`], which embeds [`StepperKinematics`]
//! first under `#[repr(C)]` (so a reference to the embedded `sk` has the
//! same address as the outer struct). The calc callback
//! [`generic_cartesian_stepper_calc_position`] casts the `sk` reference
//! back to the outer struct exactly like the C `container_of`, so the
//! callback only works on a stepper allocated by
//! [`generic_cartesian_stepper_alloc`] (or `Box`ed from one by value);
//! calling it on a standalone `StepperKinematics` would read unrelated
//! memory.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order (see
//! `parity/maps/map-motion.md` F4).

use crate::itersolve::{StepperKinematics, AF_X, AF_Y, AF_Z};
use crate::trapq::{move_get_coord, Coord, Move};

/// `struct generic_cartesian_stepper` from `kin_generic.c`: the stepper
/// kinematics plus the linear-combination coefficients `a` (`struct
/// coord`). `sk` is the first field, as in the C struct, so the calc
/// callback can recover this struct from `&sk` the way C recovers it
/// with `container_of` (the field is at offset 0).
#[repr(C)]
pub struct GenericCartesianStepper {
    /// The embedded stepper kinematics (C: `sk`).
    pub sk: StepperKinematics,
    /// The linear-combination coefficients (C: `a`).
    pub a: Coord,
}

impl GenericCartesianStepper {
    /// The stepper position at `move_time` into `m`:
    /// `a.x * c.x + a.y * c.y + a.z * c.z` in this order, where `c` is
    /// [`move_get_coord`]`(m, move_time)` (C:
    /// `generic_cartesian_stepper_calc_position`).
    pub fn calc_position(&self, m: &Move, move_time: f64) -> f64 {
        let c = move_get_coord(m, move_time);
        self.a.x * c.x + self.a.y * c.y + self.a.z * c.z
    }

    /// `generic_cartesian_stepper_set_coeffs` from `kin_generic.c`:
    /// store the coefficients and recompute `active_flags` from scratch
    /// (a coefficient is active when it is nonzero; `-0.0` is inactive
    /// and `NaN` is active, matching `if (a_x)` in C).
    pub fn set_coeffs(&mut self, a_x: f64, a_y: f64, a_z: f64) {
        self.a.x = a_x;
        self.a.y = a_y;
        self.a.z = a_z;
        self.sk.active_flags = 0;
        if a_x != 0.0 {
            self.sk.active_flags |= AF_X;
        }
        if a_y != 0.0 {
            self.sk.active_flags |= AF_Y;
        }
        if a_z != 0.0 {
            self.sk.active_flags |= AF_Z;
        }
    }
}

/// `generic_cartesian_stepper_calc_position` from `kin_generic.c`:
/// the [`crate::itersolve::StepperCalcCb`] for a generic stepper.
///
/// # Safety
///
/// `sk` must be the `sk` field of a [`GenericCartesianStepper`]
/// allocated by [`generic_cartesian_stepper_alloc`] (as `container_of`
/// requires in C). The function reads the coefficients from the
/// enclosing struct through the recovered pointer.
pub fn generic_cartesian_stepper_calc_position(
    sk: &StepperKinematics,
    m: &Move,
    move_time: f64,
) -> f64 {
    // C: container_of(sk, struct generic_cartesian_stepper, sk). The
    // embedded `sk` is the first field of the #[repr(C)] outer struct,
    // so the two pointers are identical.
    let stepper = sk as *const StepperKinematics as *const GenericCartesianStepper;
    // SAFETY: `sk` is the first field of a live GenericCartesianStepper
    // (see the safety contract above), so the recovered pointer is a
    // valid pointer to the enclosing struct. Only the coefficients are
    // read; the returned value is computed from the copied `Coord`.
    let a = unsafe { (*stepper).a };
    let c = move_get_coord(m, move_time);
    a.x * c.x + a.y * c.y + a.z * c.z
}

/// `generic_cartesian_stepper_alloc` from `kin_generic.c`: a
/// zero-initialised stepper kinematics with this module's calc callback,
/// then [`GenericCartesianStepper::set_coeffs`]`(a_x, a_y, a_z)` (which
/// also sets `active_flags`). All other `sk` fields are zero (`tq`/`sc`
/// `None`, `post_cb` `None`), exactly as after the C zeroed allocation.
pub fn generic_cartesian_stepper_alloc(
    a_x: f64,
    a_y: f64,
    a_z: f64,
) -> Box<GenericCartesianStepper> {
    let mut stepper = Box::new(GenericCartesianStepper {
        sk: StepperKinematics::new(generic_cartesian_stepper_calc_position, 0),
        a: Coord::new(0.0, 0.0, 0.0),
    });
    stepper.set_coeffs(a_x, a_y, a_z);
    stepper
}
