//! Port of `klippy/chelper/kin_winch.c`.
//!
//! Cable winch stepper kinematics: the stepper position is the distance
//! from the current toolhead position to a fixed anchor point.
//!
//! [`WinchStepper`] mirrors `struct winch_stepper`: a
//! [`StepperKinematics`] (`sk`) followed by the `anchor` [`Coord`]. The
//! `winch_stepper_calc_position` callback is the `calc_position_cb`
//! stored in the `StepperKinematics`; like the C
//! `container_of` pattern it recovers its own `WinchStepper` from the
//! `sk` reference it receives, so it is only valid on a
//! [`WinchStepper`] created by [`winch_stepper_alloc`].
//!
//! [`winch_stepper_alloc`] mirrors `winch_stepper_alloc`: the anchor is
//! stored verbatim and the stepper is active on all three axes
//! (`AF_X | AF_Y | AF_Z`); every other `StepperKinematics` field keeps
//! its zeroed-allocation value.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order.

use crate::itersolve::{StepperKinematics, AF_X, AF_Y, AF_Z};
use crate::trapq::{move_get_coord, Coord, Move};

/// `struct winch_stepper` from `kin_winch.c`: the shared
/// `StepperKinematics` first, then the anchor the position is
/// measured from.
#[repr(C)]
pub struct WinchStepper {
    /// The `struct stepper_kinematics sk` field.
    pub sk: StepperKinematics,
    /// The `struct coord anchor` field.
    pub anchor: Coord,
}

/// `winch_stepper_calc_position` from `kin_winch.c`: the position of
/// the stepper at `move_time` into `m`, the distance from
/// `move_get_coord(m, move_time)` to the anchor: `sqrt(dx*dx +
/// dy*dy + dz*dz)` with `dx`/`dy`/`dz` the anchor-minus-position
/// differences, evaluated in this order.
///
/// This is the `calc_position_cb` used by [`winch_stepper_alloc`]. It
/// recovers the owning [`WinchStepper`] from `sk` exactly like the C
/// `container_of` pattern, so `sk` must be the `sk` field of a
/// `WinchStepper` (the first field, at offset zero).
pub fn winch_stepper_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: `WinchStepper` is `#[repr(C)]` with `sk` as its first
    // field, so a `*const StepperKinematics` naming that field is a
    // valid `*const WinchStepper`. Callers must only pass the `sk`
    // of a `WinchStepper`, as in the C code.
    let hs = unsafe { &*(sk as *const StepperKinematics).cast::<WinchStepper>() };
    hs.calc_position(m, move_time)
}

/// `winch_stepper_alloc` from `kin_winch.c`: allocate a winch stepper
/// fixed at (`anchor_x`, `anchor_y`, `anchor_z`) with the distance
/// callback installed and `active_flags = AF_X | AF_Y | AF_Z`. The
/// C allocation is zeroed first; [`StepperKinematics::new`] reproduces
/// that zeroed state (all positions/times `0.0`, `tq`/`sc`/`post_cb`
/// unset).
pub fn winch_stepper_alloc(anchor_x: f64, anchor_y: f64, anchor_z: f64) -> WinchStepper {
    WinchStepper {
        sk: StepperKinematics::new(winch_stepper_calc_position, AF_X | AF_Y | AF_Z),
        anchor: Coord::new(anchor_x, anchor_y, anchor_z),
    }
}

impl WinchStepper {
    /// `winch_stepper_calc_position` for this stepper, without the
    /// `container_of` recovery (the anchor is read directly).
    pub fn calc_position(&self, m: &Move, move_time: f64) -> f64 {
        let c = move_get_coord(m, move_time);
        let dx = self.anchor.x - c.x;
        let dy = self.anchor.y - c.y;
        let dz = self.anchor.z - c.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}
