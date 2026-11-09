//! Port of `klippy/chelper/kin_delta.c`.
//!
//! The delta stepper position solver for one tower. The C
//! `struct delta_stepper` embeds `struct stepper_kinematics` as its first
//! field and stores the squared arm length plus the tower's XY location
//! beside it; [`DeltaStepper`] mirrors that layout. The `calc_position`
//! callback ([`delta_stepper_calc_position`]) recovers the extra fields from
//! the `sk` pointer it receives, exactly like the C `container_of` pattern,
//! so [`DeltaStepper`] is `#[repr(C)]` with `sk` first and the callback must
//! only be installed on a `StepperKinematics` that is the first field of a
//! [`DeltaStepper`] (as [`delta_stepper_alloc`] sets up).
//!
//! Floating point matches the C: `sqrt` is the correctly rounded
//! `f64::sqrt` (the same operation as the C `sqrt` from `math.h` here), no
//! `mul_add`, no reassociation, operations evaluated in the exact source
//! order: `sqrt((arm2 - dx * dx) - dy * dy) + c.z` with
//! `dx = tower_x - c.x` and `dy = tower_y - c.y`.

use crate::itersolve::{StepperKinematics, AF_X, AF_Y, AF_Z};
use crate::trapq::{move_get_coord, Move};

/// `struct delta_stepper` from `kin_delta.c`: a stepper kinematics with
/// the squared arm length and the tower location in XY space.
///
/// The C struct is `stepper_kinematics` followed by `arm2`, `tower_x` and
/// `tower_y`; `#[repr(C)]` with `sk` first keeps the same layout so the
/// `container_of` recovery in [`delta_stepper_calc_position`] is valid.
#[repr(C)]
pub struct DeltaStepper {
    /// The embedded stepper kinematics (`delta_stepper.sk`).
    pub sk: StepperKinematics,
    /// Squared arm length (`delta_stepper.arm2`).
    pub arm2: f64,
    /// Tower X location (`delta_stepper.tower_x`).
    pub tower_x: f64,
    /// Tower Y location (`delta_stepper.tower_y`).
    pub tower_y: f64,
}

impl DeltaStepper {
    /// Stepper position `move_time` seconds into `m`, computed directly
    /// from this stepper's own fields. Equivalent to running
    /// [`delta_stepper_calc_position`] on this stepper's `sk`.
    pub fn calc_position(&self, m: &Move, move_time: f64) -> f64 {
        let c = move_get_coord(m, move_time);
        let dx = self.tower_x - c.x;
        let dy = self.tower_y - c.y;
        (self.arm2 - dx * dx - dy * dy).sqrt() + c.z
    }
}

/// `delta_stepper_calc_position` from `kin_delta.c`: the
/// `StepperCalcCb` installed by [`delta_stepper_alloc`].
///
/// `sk` must be the `sk` field of a [`DeltaStepper`], as in the C code
/// where the callback is only ever reached through the pointer returned by
/// `delta_stepper_alloc`. The recovery mirrors `container_of(sk, struct
/// delta_stepper, sk)`.
pub fn delta_stepper_calc_position(
    sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    // SAFETY: callers only pass the `sk` of a `DeltaStepper`, whose first
    // field is `sk`, so the two pointers are equal, as with `container_of`.
    let ds = unsafe { &*std::ptr::from_ref(sk).cast::<DeltaStepper>() };
    ds.calc_position(m, move_time)
}

/// `delta_stepper_alloc` from `kin_delta.c`: allocate a delta stepper for
/// a tower with squared arm length `arm2` at (`tower_x`, `tower_y`).
///
/// All other stepper kinematics state is zeroed, matching the C `memset`
/// of the freshly `malloc`ed struct, and `active_flags` is
/// `AF_X | AF_Y | AF_Z` as in the C. The C returns a bare
/// `struct stepper_kinematics *` to heap memory managed by the caller;
/// this returns the owning `Box<DeltaStepper>`.
pub fn delta_stepper_alloc(arm2: f64, tower_x: f64, tower_y: f64) -> Box<DeltaStepper> {
    Box::new(DeltaStepper {
        sk: StepperKinematics::new(delta_stepper_calc_position, AF_X | AF_Y | AF_Z),
        arm2,
        tower_x,
        tower_y,
    })
}
