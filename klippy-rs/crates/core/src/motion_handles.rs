//! The concrete objects behind the opaque `klippy_api` motion handles.
//!
//! Python passes chelper (cffi) pointers between modules: `trapq_alloc()`
//! from `motion_queuing`, `toolhead.get_trapq()`,
//! `stepper.get_stepper_kinematics()`, and the `*_stepper_alloc` results
//! `stepper.setup_itersolve` installs. The Rust contract hides them behind
//! [`TrapqApi`] and [`StepperKinematicsApi`]; [`TrapqHandle`] and
//! [`StepperKinematicsHandle`] are the only types behind those traits, so
//! every module that needs the chelper object downcasts with [`trapq_of`]
//! or [`kinematics_of`] (a foreign handle is a hard error, never a silent
//! skip or a shadow copy).
//!
//! Step generation runs on one thread per syncemitter
//! (`klippy/chelper/steppersync.c`), so both handles are `Send + Sync`: the
//! trapq is a [`SharedTrapq`] (`Arc<kos::sync::Mutex<Trapq>>`) and each
//! stepper kinematics allocation sits behind its own mutex.

use std::any::Any;
use std::sync::Arc;

use chelper_motion::itersolve::StepperKinematics as Sk;
use chelper_motion::kin_cartesian::cartesian_stepper_alloc;
use chelper_motion::kin_corexy::corexy_stepper_alloc;
use chelper_motion::kin_corexz::corexz_stepper_alloc;
use chelper_motion::kin_delta::{delta_stepper_alloc, DeltaStepper};
use chelper_motion::kin_deltesian::{deltesian_stepper_alloc, DeltesianStepper};
use chelper_motion::kin_extruder::{extruder_stepper_alloc, ExtruderStepper};
use chelper_motion::kin_generic::{generic_cartesian_stepper_alloc, GenericCartesianStepper};
use chelper_motion::kin_idex::{dual_carriage_alloc, DualCarriage};
use chelper_motion::kin_polar::polar_stepper_alloc;
use chelper_motion::kin_rotary_delta::{rotary_delta_stepper_alloc, RotaryStepper};
use chelper_motion::kin_shaper::{input_shaper_alloc, InputShaper};
use chelper_motion::kin_winch::{winch_stepper_alloc, WinchStepper};
use chelper_motion::trapq::{SharedTrapq, Trapq as CTrapq};
use klippy_api::stepper::{StepperKinematics, StepperKinematicsApi, Trapq, TrapqApi};
use klippy_api::{KResult, KlippyError};
use kos::sync::Mutex;
use pycompat::value::type_name;
use pycompat::PyValue;

/// The chelper `struct trapq` behind a [`Trapq`] handle.
pub struct TrapqHandle {
    tq: SharedTrapq,
}

impl TrapqHandle {
    /// `ffi_lib.trapq_alloc()`.
    pub fn alloc() -> Trapq {
        Arc::new(TrapqHandle { tq: Arc::new(Mutex::new(CTrapq::new())) })
    }

    /// The shared trapq, as installed into a stepper kinematics by
    /// `itersolve_set_trapq` and read by the step generation threads.
    pub fn shared(&self) -> &SharedTrapq {
        &self.tq
    }
}

impl TrapqApi for TrapqHandle {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// The [`TrapqHandle`] behind `tq`.
pub fn trapq_of(tq: &Trapq) -> KResult<&TrapqHandle> {
    tq.as_any().downcast_ref::<TrapqHandle>().ok_or_else(|| foreign_handle("trapq"))
}

/// One `*_stepper_alloc` result. Every variant is boxed so the embedded
/// `stepper_kinematics` keeps a stable address: the position callbacks
/// recover their container from it (the C `container_of` pattern), and
/// `input_shaper_set_sk` / `dual_carriage_set_sk` store a pointer to it.
pub enum KinAlloc {
    /// `cartesian_stepper_alloc`, `corexy_stepper_alloc`,
    /// `corexz_stepper_alloc`, `polar_stepper_alloc`.
    Plain(Box<Sk>),
    /// `delta_stepper_alloc`.
    Delta(Box<DeltaStepper>),
    /// `deltesian_stepper_alloc`.
    Deltesian(Box<DeltesianStepper>),
    /// `rotary_delta_stepper_alloc`.
    RotaryDelta(Box<RotaryStepper>),
    /// `winch_stepper_alloc`.
    Winch(Box<WinchStepper>),
    /// `generic_cartesian_stepper_alloc`.
    GenericCartesian(Box<GenericCartesianStepper>),
    /// `extruder_stepper_alloc`.
    Extruder(Box<ExtruderStepper>),
    /// `dual_carriage_alloc`.
    DualCarriage(Box<DualCarriage>),
    /// `input_shaper_alloc`.
    InputShaper(Box<InputShaper>),
}

// SAFETY: `DualCarriage` and `InputShaper` hold a `*mut StepperKinematics`
// to another handle's kinematics (`dual_carriage_set_sk`,
// `input_shaper_set_sk`). Python keeps that original alive
// (`idex_modes`, `input_shaper` hold it) and detaches it from step
// generation while wrapped, so only the wrapper's generating thread reads
// it, exactly as in the C. Every other field is plain owned data.
unsafe impl Send for KinAlloc {}

impl KinAlloc {
    /// The embedded `struct stepper_kinematics`.
    pub fn sk(&self) -> &Sk {
        match self {
            KinAlloc::Plain(s) => s,
            KinAlloc::Delta(s) => &s.sk,
            KinAlloc::Deltesian(s) => &s.sk,
            KinAlloc::RotaryDelta(s) => &s.sk,
            KinAlloc::Winch(s) => &s.sk,
            KinAlloc::GenericCartesian(s) => &s.sk,
            KinAlloc::Extruder(s) => &s.sk,
            KinAlloc::DualCarriage(s) => &s.sk,
            KinAlloc::InputShaper(s) => &s.sk,
        }
    }

    /// The embedded `struct stepper_kinematics`, mutably.
    pub fn sk_mut(&mut self) -> &mut Sk {
        match self {
            KinAlloc::Plain(s) => s,
            KinAlloc::Delta(s) => &mut s.sk,
            KinAlloc::Deltesian(s) => &mut s.sk,
            KinAlloc::RotaryDelta(s) => &mut s.sk,
            KinAlloc::Winch(s) => &mut s.sk,
            KinAlloc::GenericCartesian(s) => &mut s.sk,
            KinAlloc::Extruder(s) => &mut s.sk,
            KinAlloc::DualCarriage(s) => &mut s.sk,
            KinAlloc::InputShaper(s) => &mut s.sk,
        }
    }
}

/// A chelper stepper kinematics allocation behind a [`StepperKinematics`]
/// handle.
pub struct StepperKinematicsHandle {
    alloc: Mutex<KinAlloc>,
}

impl StepperKinematicsHandle {
    /// Wrap one allocation as a contract handle.
    pub fn wrap(alloc: KinAlloc) -> StepperKinematics {
        Arc::new(StepperKinematicsHandle { alloc: Mutex::new(alloc) })
    }

    /// Run `f` on the allocation (kind-specific calls such as
    /// `extruder_set_pressure_advance` or `input_shaper_set_shaper_params`).
    pub fn with<R>(&self, f: impl FnOnce(&mut KinAlloc) -> R) -> R {
        f(&mut self.alloc.lock())
    }

    /// Run `f` on the embedded `stepper_kinematics` (the `itersolve_*`
    /// calls).
    pub fn with_sk<R>(&self, f: impl FnOnce(&mut Sk) -> R) -> R {
        self.with(|a| f(a.sk_mut()))
    }

    /// The address of the embedded `stepper_kinematics`, as the C passes it
    /// to `input_shaper_set_sk` / `dual_carriage_set_sk`. Stable for the
    /// handle's lifetime because every variant is boxed.
    pub fn sk_ptr(&self) -> *mut Sk {
        self.with(|a| a.sk_mut() as *mut Sk)
    }
}

impl StepperKinematicsApi for StepperKinematicsHandle {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// The [`StepperKinematicsHandle`] behind `sk`.
pub fn kinematics_of(sk: &StepperKinematics) -> KResult<&StepperKinematicsHandle> {
    sk.as_any()
        .downcast_ref::<StepperKinematicsHandle>()
        .ok_or_else(|| foreign_handle("stepper kinematics"))
}

/// `getattr(ffi_lib, alloc_func)(*params)` from `stepper.setup_itersolve`
/// (`klippy/stepper.py:74-77`): call the named chelper allocator with the
/// Python arguments, converted the way cffi converts them (`char` from a
/// one-byte `bytes`, `double` from an `int` or `float`).
pub fn alloc_stepper_kinematics(alloc_func: &str, params: &[PyValue]) -> KResult<StepperKinematics> {
    let alloc = match alloc_func {
        "cartesian_stepper_alloc" => {
            let [axis] = char_args(alloc_func, params)?;
            KinAlloc::Plain(Box::new(cartesian_stepper_alloc(axis)))
        }
        "corexy_stepper_alloc" => {
            let [kind] = char_args(alloc_func, params)?;
            KinAlloc::Plain(Box::new(corexy_stepper_alloc(kind)))
        }
        "corexz_stepper_alloc" => {
            let [kind] = char_args(alloc_func, params)?;
            KinAlloc::Plain(Box::new(corexz_stepper_alloc(kind as i8)))
        }
        "polar_stepper_alloc" => {
            let [kind] = char_args(alloc_func, params)?;
            KinAlloc::Plain(Box::new(polar_stepper_alloc(kind)))
        }
        "delta_stepper_alloc" => {
            let [arm2, tower_x, tower_y] = double_args(alloc_func, params)?;
            KinAlloc::Delta(delta_stepper_alloc(arm2, tower_x, tower_y))
        }
        "deltesian_stepper_alloc" => {
            let [arm2, arm_x] = double_args(alloc_func, params)?;
            KinAlloc::Deltesian(Box::new(deltesian_stepper_alloc(arm2, arm_x)))
        }
        "rotary_delta_stepper_alloc" => {
            let [radius, height, angle, upper, lower] = double_args(alloc_func, params)?;
            KinAlloc::RotaryDelta(rotary_delta_stepper_alloc(radius, height, angle, upper, lower))
        }
        "winch_stepper_alloc" => {
            let [x, y, z] = double_args(alloc_func, params)?;
            KinAlloc::Winch(Box::new(winch_stepper_alloc(x, y, z)))
        }
        "generic_cartesian_stepper_alloc" => {
            let [a_x, a_y, a_z] = double_args(alloc_func, params)?;
            KinAlloc::GenericCartesian(generic_cartesian_stepper_alloc(a_x, a_y, a_z))
        }
        "extruder_stepper_alloc" => {
            let [] = double_args(alloc_func, params)?;
            KinAlloc::Extruder(extruder_stepper_alloc())
        }
        "input_shaper_alloc" => {
            let [] = double_args(alloc_func, params)?;
            KinAlloc::InputShaper(input_shaper_alloc())
        }
        "dual_carriage_alloc" => {
            let [] = double_args(alloc_func, params)?;
            KinAlloc::DualCarriage(Box::new(dual_carriage_alloc()))
        }
        _ => {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: format!(
                    "cffi library '_chelper' has no function, constant or global variable named '{alloc_func}'"
                ),
            })
        }
    };
    Ok(StepperKinematicsHandle::wrap(alloc))
}

fn foreign_handle(what: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: format!("{what} handle was not created by klippy_core::motion_handles"),
    }
}

fn check_arity(alloc_func: &str, params: &[PyValue], n: usize) -> KResult<()> {
    if params.len() == n {
        return Ok(());
    }
    Err(KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: format!("{alloc_func} expects {n} arguments, got {}", params.len()),
    })
}

/// cffi's `char` argument conversion: a `bytes` of length 1.
fn char_args<const N: usize>(alloc_func: &str, params: &[PyValue]) -> KResult<[u8; N]> {
    check_arity(alloc_func, params, N)?;
    let mut out = [0u8; N];
    for (slot, p) in out.iter_mut().zip(params) {
        *slot = match p {
            PyValue::Bytes(b) if b.len() == 1 => b[0],
            other => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "initializer for ctype 'char' must be a bytes of length 1, not {}",
                        type_name(other)
                    ),
                })
            }
        };
    }
    Ok(out)
}

/// cffi's `double` argument conversion: any `int` or `float`.
fn double_args<const N: usize>(alloc_func: &str, params: &[PyValue]) -> KResult<[f64; N]> {
    check_arity(alloc_func, params, N)?;
    let mut out = [0.0f64; N];
    for (slot, p) in out.iter_mut().zip(params) {
        *slot = match p {
            PyValue::Float(_) | PyValue::Int(_) | PyValue::Bool(_) => p.to_f64(),
            _ => None,
        }
        .ok_or_else(|| KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("must be real number, not {}", type_name(p)),
        })?;
    }
    Ok(out)
}
