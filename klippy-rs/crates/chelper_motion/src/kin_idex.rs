//! Port of `klippy/chelper/kin_idex.c`.
//!
//! A dual-carriage (IDEX) wrapper around another stepper kinematics:
//! the position of the wrapped kinematics is computed from a scratch move
//! whose X/Y start positions are the source move's coordinates scaled and
//! offset by the configured transform, at the dummy time [`DUMMY_T`]
//! (the axes and timing of the scratch move are irrelevant, its
//! `start_pos` carries the transformed coordinates).
//!
//! The wrapped kinematics is borrowed through a raw pointer, exactly like
//! `struct dual_carriage_stepper { struct stepper_kinematics *orig_sk; }`:
//! the caller owns it and keeps it alive (Python's `idex_modes` keeps the
//! original kinematics), and the wrapper only reads its `calc_position_cb`,
//! `active_flags` and `commanded_pos` (as the `polar` callback does). [`DualCarriage`]
//! embeds the wrapper [`StepperKinematics`] as its first field so that
//! [`dual_carriage_calc_position`] can recover the wrapper state from the
//! `sk` pointer it receives, exactly like the C `container_of` pattern
//! ([`#[repr(C)]`] guarantees the `sk` field is at offset 0).
//!
//! Lifecycle mirrors the C entry points: [`dual_carriage_alloc`] builds a
//! zeroed wrapper, [`dual_carriage_set_sk`] attaches the wrapped
//! kinematics (copying its `active_flags` and installing
//! [`dual_carriage_calc_position`] as the callback), and
//! [`dual_carriage_set_transform`] sets the per-axis scale/offset,
//! adjusting `active_flags` (a zero scale deactivates the axis; a nonzero
//! scale reactivates it only if the wrapped kinematics has the axis
//! active).
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations in the exact source order
//! (`pos.x * x_scale + x_offs`).

use std::cell::RefCell;

use crate::itersolve::{StepperKinematics, AF_X, AF_Y};
use crate::trapq::{move_get_coord, Move};

/// `DUMMY_T` from `kin_idex.c`: the dummy time at which the wrapped
/// kinematics is evaluated.
pub const DUMMY_T: f64 = 500.0;

/// `struct dual_carriage_stepper` from `kin_idex.c`: a wrapper stepper
/// kinematics with the wrapped kinematics' `sk` embedded first (C
/// `struct stepper_kinematics sk`), the wrapped kinematics, the scratch
/// move, and the per-axis scale/offset transforms.
#[repr(C)]
pub struct DualCarriage {
    /// The wrapper `struct stepper_kinematics`, at offset 0 so the
    /// position callback can recover `DualCarriage` from the `sk`
    /// pointer (the C `container_of` pattern).
    pub sk: StepperKinematics,
    /// The wrapped kinematics (null until [`dual_carriage_set_sk`]).
    orig_sk: *mut StepperKinematics,
    /// The scratch move (`dc->m`): only its `start_pos` is rewritten on
    /// each callback (`move_t` is `2 * DUMMY_T`, everything else zero,
    /// as after the C zeroed allocation). It lives in its own heap
    /// allocation: the callback receives `&StepperKinematics`, which the
    /// compiler treats as read-only and unaliased for the call, so a store
    /// into this struct through the recovered container is undefined
    /// behaviour (LLVM drops it on the MIPS32 build). The store through
    /// the `Box` pointer does not derive from the callback argument.
    m: Box<RefCell<Move>>,
    /// X axis scale and offset.
    x_scale: f64,
    /// X axis offset.
    x_offs: f64,
    /// Y axis scale and offset.
    y_scale: f64,
    /// Y axis offset.
    y_offs: f64,
}

impl DualCarriage {
    /// The current X axis `(scale, offset)` transform.
    pub fn x_transform(&self) -> (f64, f64) {
        (self.x_scale, self.x_offs)
    }

    /// The current Y axis `(scale, offset)` transform.
    pub fn y_transform(&self) -> (f64, f64) {
        (self.y_scale, self.y_offs)
    }

    /// The scratch move as last written by the position callback.
    pub fn scratch_move(&self) -> Move {
        *self.m.borrow()
    }
}

/// `dual_carriage_calc_position` from `kin_idex.c`: compute the wrapped
/// stepper position for the transformed coordinates of `m` at
/// `move_time`, evaluating the wrapped kinematics at [`DUMMY_T`] on the
/// scratch move.
pub fn dual_carriage_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: `sk` points to the first field (`sk`, at offset 0 by
    // `#[repr(C)]`) of a live `DualCarriage`; the callback is only ever
    // installed on such a wrapper (`dual_carriage_alloc` /
    // `dual_carriage_set_sk`), mirroring the C `container_of`.
    let dc = unsafe { &*(sk as *const StepperKinematics as *const DualCarriage) };
    let pos = move_get_coord(m, move_time);
    {
        let mut dm = dc.m.borrow_mut();
        dm.start_pos.x = pos.x * dc.x_scale + dc.x_offs;
        dm.start_pos.y = pos.y * dc.y_scale + dc.y_offs;
        dm.start_pos.z = pos.z;
    }
    // The C dereferences `orig_sk` unconditionally (a null dereference
    // if `dual_carriage_set_sk` was never called).
    assert!(!dc.orig_sk.is_null(), "dual_carriage: orig_sk not set");
    // SAFETY: `dual_carriage_set_sk` stored a pointer to a live kinematics
    // the caller keeps alive for the wrapper's lifetime (same as the C).
    let orig = unsafe { &*dc.orig_sk };
    let calc_position_cb = orig.calc_position_cb;
    let dm = dc.m.borrow();
    calc_position_cb(orig, &dm, DUMMY_T)
}

/// `dual_carriage_set_sk` from `kin_idex.c`: attach the wrapped
/// kinematics, installing this module's position callback and copying
/// the wrapped `active_flags`.
///
/// # Safety
///
/// `orig_sk` must point to a live kinematics that outlives `dc` and is not
/// mutated while `dc` generates steps (same contract as the C).
pub unsafe fn dual_carriage_set_sk(dc: &mut DualCarriage, orig_sk: *mut StepperKinematics) {
    dc.sk.calc_position_cb = dual_carriage_calc_position;
    // SAFETY: caller contract above.
    dc.sk.active_flags = unsafe { (*orig_sk).active_flags };
    dc.orig_sk = orig_sk;
}

/// `dual_carriage_set_transform` from `kin_idex.c`: set the scale and
/// offset for `axis` (`b'x'` or `b'y'`), adjusting `active_flags`.
/// Returns `0` on success, `-1` for any other axis (with no state
/// change), exactly like the C.
pub fn dual_carriage_set_transform(
    dc: &mut DualCarriage,
    axis: u8,
    scale: f64,
    offs: f64,
) -> i32 {
    if axis == b'x' {
        dc.x_scale = scale;
        dc.x_offs = offs;
        if scale == 0.0 {
            dc.sk.active_flags &= !AF_X;
        } else if orig_active_flags(dc) & AF_X != 0 {
            dc.sk.active_flags |= AF_X;
        }
        return 0;
    }
    if axis == b'y' {
        dc.y_scale = scale;
        dc.y_offs = offs;
        if scale == 0.0 {
            dc.sk.active_flags &= !AF_Y;
        } else if orig_active_flags(dc) & AF_Y != 0 {
            dc.sk.active_flags |= AF_Y;
        }
        return 0;
    }
    -1
}

/// The wrapped kinematics' `active_flags` (read in
/// `dual_carriage_set_transform` exactly where the C reads
/// `orig_sk->active_flags`).
fn orig_active_flags(dc: &DualCarriage) -> i32 {
    assert!(!dc.orig_sk.is_null(), "dual_carriage: orig_sk not set");
    // SAFETY: set by `dual_carriage_set_sk` to a live kinematics.
    unsafe { (*dc.orig_sk).active_flags }
}

/// `dual_carriage_alloc` from `kin_idex.c`: a zeroed wrapper with
/// `m.move_t = 2 * DUMMY_T` and unit X/Y scales. The embedded `sk`
/// carries this module's position callback from the start (the C leaves
/// it null until `dual_carriage_set_sk`, but a null callback would only
/// crash the reference; the Rust `StepperKinematics` never has a null
/// callback).
pub fn dual_carriage_alloc() -> DualCarriage {
    let m = Move {
        move_t: 2. * DUMMY_T,
        ..Default::default()
    };
    DualCarriage {
        sk: StepperKinematics::new(dual_carriage_calc_position, 0),
        orig_sk: std::ptr::null_mut(),
        m: Box::new(RefCell::new(m)),
        x_scale: 1.0,
        x_offs: 0.0,
        y_scale: 1.0,
        y_offs: 0.0,
    }
}

// `sk` must be the first field, at offset 0, for the `container_of`
// recovery in `dual_carriage_calc_position` to be valid.
const _: () = {
    assert!(core::mem::offset_of!(DualCarriage, sk) == 0);
};
