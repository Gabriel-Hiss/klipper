//! Port of `klippy/chelper/kin_rotary_delta.c`.
//!
//! A rotary delta arm is constrained to rotate about its shoulder axis, so
//! each stepper only reports its upper arm angle. [`RotaryStepper`] mirrors
//! `struct rotary_stepper`: [`StepperKinematics`] embedded first (at offset
//! 0, as in the C struct) followed by the same named `f64` fields in the
//! same order.
//!
//! [`rotary_delta_stepper_alloc`] is the `__visible`
//! `rotary_delta_stepper_alloc` entry point. The returned stepper's solver
//! callback is [`rotary_stepper_calc_position`], which recovers the
//! [`RotaryStepper`] state from the `&StepperKinematics` it receives via
//! the same offset-0 `container_of` pointer cast the C code uses.
//!
//! [`rotary_two_arm_calc`] is the C `static inline` two-arm angle solver;
//! it is exposed so tests can drive it directly. The formula is evaluated
//! in the exact C order, including the division by `dy` (a zero `dy` yields
//! the same infinities/NaNs as the C on the same target) and the square
//! root of a possibly negative argument (an unreachable target yields NaN).
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order (see
//! `parity/maps/map-motion.md` F4).

use kos::libm;

use crate::itersolve::{StepperKinematics, AF_X, AF_Y, AF_Z};
use crate::trapq::{move_get_coord, Move};

/// `rotary_two_arm_calc` from `kin_rotary_delta.c`: upper arm angle given
/// the effector joint position `(dx, dy)` relative to the shoulder joint,
/// plus the squared upper and lower arm lengths.
///
/// The C comment gives the relations this solves: `elbow_x**2 + elbow_y**2
/// = upper_arm**2` and `(effector_x - elbow_x)**2 + (effector_y - elbow_y)**2
/// = lower_arm**2`.
pub fn rotary_two_arm_calc(dx: f64, dy: f64, upper_arm2: f64, lower_arm2: f64) -> f64 {
    // Determine constants such that: elbow_y = c1 - c2*elbow_x
    let inv_dy = 1.0 / dy;
    let c1 = 0.5 * inv_dy * (dx * dx + dy * dy + upper_arm2 - lower_arm2);
    let c2 = dx * inv_dy;
    // Calculate scaled elbow coordinates via quadratic equation.
    let scale = c2 * c2 + 1.0;
    let scaled_elbow_x = c1 * c2 + libm::sqrt(scale * upper_arm2 - c1 * c1);
    let scaled_elbow_y = c1 * scale - c2 * scaled_elbow_x;
    // Calculate angle in radians
    libm::atan2(scaled_elbow_y, scaled_elbow_x)
}

/// `struct rotary_stepper` from `kin_rotary_delta.c`, with the same field
/// names and order. `sk` sits at offset 0 so the solver callback can
/// recover this struct from the `&StepperKinematics` it is passed, exactly
/// like the C `container_of(sk, struct rotary_stepper, sk)`.
#[repr(C)]
pub struct RotaryStepper {
    /// Embedded solver state (`struct stepper_kinematics sk`).
    pub sk: StepperKinematics,
    /// `cos(angle)` (`rotary_stepper.cos`).
    pub cos: f64,
    /// `sin(angle)` (`rotary_stepper.sin`).
    pub sin: f64,
    /// Distance from the shoulder axis to the shoulder joint
    /// (`rotary_stepper.shoulder_radius`).
    pub shoulder_radius: f64,
    /// Shoulder joint height (`rotary_stepper.shoulder_height`).
    pub shoulder_height: f64,
    /// Squared upper arm length (`rotary_stepper.upper_arm2`).
    pub upper_arm2: f64,
    /// Squared lower arm length (`rotary_stepper.lower_arm2`).
    pub lower_arm2: f64,
}

/// `rotary_stepper_calc_position` from `kin_rotary_delta.c`: the upper arm
/// angle for the move position at `move_time`, with the coordinate rotated
/// and shifted into the shoulder joint frame.
pub fn rotary_stepper_calc_position(
    sk: &StepperKinematics,
    m: &Move,
    _at: Option<usize>,
    move_time: f64,
) -> f64 {
    let c = move_get_coord(m, move_time);
    // SAFETY: `sk` is the embedded `sk` field at offset 0 of the
    // `RotaryStepper` that [`rotary_delta_stepper_alloc`] allocated, so
    // this pointer cast recovers the same struct the C `container_of`
    // computes. The solver only ever invokes the callback on such a
    // pointer.
    let rs = unsafe { &*(sk as *const StepperKinematics as *const RotaryStepper) };
    // Rotate and shift axes to an origin at shoulder joint with upper
    // arm constrained to xy plane and x aligned to shoulder platform.
    let sjz = c.y * rs.cos - c.x * rs.sin;
    let sjx = c.x * rs.cos + c.y * rs.sin - rs.shoulder_radius;
    let sjy = c.z - rs.shoulder_height;
    // Calculate angle in radians
    rotary_two_arm_calc(sjx, sjy, rs.upper_arm2, rs.lower_arm2 - sjz * sjz)
}

/// `rotary_delta_stepper_alloc` from `kin_rotary_delta.c`: allocate a
/// rotary delta stepper with the given geometry. The C code zeroes the
/// allocation, so all solver state starts at `0.0` via
/// [`StepperKinematics::new`]; the C function returns `&rs->sk`, and the
/// Rust caller uses `stepper.sk`.
pub fn rotary_delta_stepper_alloc(
    shoulder_radius: f64,
    shoulder_height: f64,
    angle: f64,
    upper_arm: f64,
    lower_arm: f64,
) -> Box<RotaryStepper> {
    Box::new(RotaryStepper {
        sk: StepperKinematics::new(
            rotary_stepper_calc_position,
            AF_X | AF_Y | AF_Z,
        ),
        cos: libm::cos(angle),
        sin: libm::sin(angle),
        shoulder_radius,
        shoulder_height,
        upper_arm2: upper_arm * upper_arm,
        lower_arm2: lower_arm * lower_arm,
    })
}
