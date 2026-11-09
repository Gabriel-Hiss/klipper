//! Port of `klippy/chelper/kin_deltesian.c`.
//!
//! The C file defines `struct deltesian_stepper` with a
//! `stepper_kinematics` embedded first, followed by `arm2` (the squared
//! arm length) and `arm_x` (the X offset of the arm pivot). The position
//! callback computes `sqrt(arm2 - dx*dx) + c.z` with
//! `dx = c.x - arm_x` and `c = move_get_coord(m, move_time)`, and the
//! allocator sets `active_flags` to `AF_X | AF_Z`. The Python side
//! (`klippy/kinematics/deltesian.py`) requests this with
//! `setup_itersolve('deltesian_stepper_alloc', arm2, arm_x)`.
//!
//! [`DeltesianStepper`] mirrors the C struct field for field. Its
//! [`StepperKinematics`] member is first, and the struct has
//! `#[repr(C)]`, so the address of `sk` equals the address of the whole
//! struct. [`deltesian_stepper_calc_position`] recovers the extra state
//! from the `sk` reference it receives with the same `container_of`
//! pointer cast the C code uses. Callers must therefore call the
//! callback through `deltesian_stepper.sk`; the `StepperKinematics`
//! must not be detached from its `DeltesianStepper`.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations in the exact source order
//! (`arm2 - dx*dx`, then `sqrt`, then `+ c.z`). `sqrt` uses
//! `f64::sqrt`, correctly rounded and permitted by the architecture
//! contract for the same reason the C `sqrt` matches glibc bit for bit.

use crate::itersolve::{StepperCalcCb, StepperKinematics, AF_X, AF_Z};
use crate::trapq::{move_get_coord, Move};

/// `struct deltesian_stepper` from `kin_deltesian.c`: the shared stepper
/// kinematics plus the arm parameters, with `sk` first so the
/// `container_of` cast in the callback is valid.
#[repr(C)]
pub struct DeltesianStepper {
    /// The shared stepper kinematics (the C `sk` member).
    pub sk: StepperKinematics,
    /// Squared arm length (`arm2`, passed by the caller as `arm**2`).
    pub arm2: f64,
    /// X offset of the arm pivot (`arm_x`).
    pub arm_x: f64,
}

impl DeltesianStepper {
    /// Recover the stepper state from the embedded kinematics reference,
    /// mirroring the C `container_of(sk, struct deltesian_stepper, sk)`
    /// cast. Valid because `sk` is the first member of a `#[repr(C)]`
    /// struct, so both pointers are equal.
    fn from_sk(sk: &StepperKinematics) -> &Self {
        // SAFETY: `sk` must be the embedded kinematics of a live
        // `DeltesianStepper` (the caller calls the callback through
        // `DeltesianStepper::sk`, as the C code only ever invokes the
        // callback with the `&ds->sk` that `deltesian_stepper_alloc`
        // registered). With `#[repr(C)]` and `sk` first, `sk` is at
        // offset 0 of the outer struct, so the cast yields the matching
        // `DeltesianStepper`.
        unsafe { &*(sk as *const StepperKinematics as *const DeltesianStepper) }
    }
}

/// `deltesian_stepper_calc_position` from `kin_deltesian.c`:
/// `sqrt(arm2 - dx*dx) + c.z` with `dx = c.x - arm_x` and
/// `c = move_get_coord(m, move_time)`, in source order.
pub fn deltesian_stepper_calc_position(
    sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    let ds = DeltesianStepper::from_sk(sk);
    let c = move_get_coord(m, move_time);
    let dx = c.x - ds.arm_x;
    (ds.arm2 - dx * dx).sqrt() + c.z
}

/// `deltesian_stepper_alloc` from `kin_deltesian.c`: a zeroed stepper
/// kinematics with the position callback installed and `active_flags`
/// set to `AF_X | AF_Z`, plus `arm2` and `arm_x`. The C version returns
/// a heap pointer freed with `free`; this port returns the owned value
/// and the caller keeps it alive for the callback's lifetime.
pub fn deltesian_stepper_alloc(arm2: f64, arm_x: f64) -> DeltesianStepper {
    let sk = StepperKinematics::new(
        deltesian_stepper_calc_position as StepperCalcCb,
        AF_X | AF_Z,
    );
    DeltesianStepper { sk, arm2, arm_x }
}
