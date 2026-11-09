//! The concrete objects behind the opaque `klippy_api` motion handles.
//!
//! Python passes chelper (cffi) pointers between modules: `trapq_alloc()`
//! from `motion_queuing`, `toolhead.get_trapq()`,
//! `stepper.get_stepper_kinematics()`, and the `*_stepper_alloc` results
//! `stepper.setup_itersolve` installs. The Rust contract hides them behind
//! [`TrapqApi`], [`StepperKinematicsApi`] and [`SyncEmitterApi`];
//! [`TrapqHandle`], [`StepperKinematicsHandle`] and [`CoreSyncEmitter`] are
//! the only types behind those traits, so every module that needs the
//! chelper object downcasts with [`trapq_of`], [`kinematics_of`] or
//! [`syncemitter_of`] (a foreign handle is a hard error, never a silent
//! skip or a shadow copy).
//!
//! Step generation runs on one thread per syncemitter
//! (`klippy/chelper/steppersync.c`), so all three handles are `Send + Sync`:
//! the trapq is a [`SharedTrapq`] (`Arc<kos::sync::Mutex<Trapq>>`), each
//! stepper kinematics allocation sits behind its own mutex, and the
//! syncemitter is the ported [`SyncEmitter`] behind an `Arc` (thread safe
//! through `kos` locks, like the C).

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
use klippy_api::stepper::{
    PullHistoryStep, StepperKinematics, StepperKinematicsApi, Trapq, TrapqApi,
};
use klippy_api::toolhead::{SyncEmitterApi, SyncEmitterHandle as SyncEmitter};
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

    /// Wrap an existing shared trapq (the `itersolve_get_trapq` read-back
    /// of a trapq installed through `set_trapq`): same queue, new handle,
    /// like the C returning the same `struct trapq *`.
    pub fn from_shared(tq: SharedTrapq) -> Trapq {
        Arc::new(TrapqHandle { tq })
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

/// `ffi_lib.trapq_set_position(tq, print_time, x, y, z)` on the chelper
/// trapq behind `tq` (`toolhead.set_position`, `klippy/toolhead.py:386`).
pub fn trapq_set_position(tq: &Trapq, print_time: f64, x: f64, y: f64, z: f64) -> KResult<()> {
    trapq_of(tq)?.shared().lock().set_position(print_time, x, y, z);
    Ok(())
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

/// The ported `struct syncemitter` behind a [`SyncEmitter`] handle: the
/// `Arc` keeps the C object alive for `motion_queuing`'s emitter list and
/// for the steppers that use it, exactly like the C `se_list`.
pub struct CoreSyncEmitter {
    se: Arc<chelper_motion::steppersync::SyncEmitter>,
    sk: Mutex<Option<StepperKinematics>>,
}

impl CoreSyncEmitter {
    /// Wrap a ported emitter as a contract handle.
    pub fn wrap(se: Arc<chelper_motion::steppersync::SyncEmitter>) -> SyncEmitter {
        Arc::new(CoreSyncEmitter { se, sk: Mutex::new(None) })
    }

    /// The ported C object.
    pub fn emitter(&self) -> &Arc<chelper_motion::steppersync::SyncEmitter> {
        &self.se
    }
}

/// The [`CoreSyncEmitter`] behind `se`.
pub fn syncemitter_of(se: &SyncEmitter) -> KResult<&CoreSyncEmitter> {
    se.as_any()
        .downcast_ref::<CoreSyncEmitter>()
        .ok_or_else(|| foreign_handle("syncemitter"))
}

impl SyncEmitterApi for CoreSyncEmitter {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn queue_msg(&self, req_clock: u64, data: &[u32]) {
        self.se.queue_msg(req_clock, data);
    }

    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) {
        // `syncemitter_set_stepper_kinematics` (`steppersync.c:58`): null
        // clears the registration. The ported emitter holds a raw pointer
        // to the embedded `Sk`, so resolve it to a stable address first.
        let ptr = match sk.as_ref() {
            Some(sk) => match kinematics_of(sk) {
                Ok(h) => h.sk_ptr(),
                Err(e) => panic!("{}: {}", e.class_name(), e.message()),
            },
            None => std::ptr::null_mut(),
        };
        // SAFETY: the `StepperKinematicsHandle` (kept in `self.sk` below,
        // mirroring the C, where the emitter registration and the
        // `motion_queuing.syncemitters` list keep the objects alive)
        // outlives the registration, and `gen_steps` is the only
        // concurrent reader, exactly as in the C.
        unsafe { self.se.set_stepper_kinematics(ptr) };
        *self.sk.lock() = sk;
    }

    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        // `syncemitter_get_stepper_kinematics` (`steppersync.c:66`): the
        // registered handle, kept alive above like the C keeps `se->sk`.
        self.sk.lock().clone()
    }

    fn stepcompress_fill(
        &self,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    ) {
        // Do not hold the compressor lock while queueing (the port's
        // `queue_msg` locks it too); copy the handle out first.
        let sc = self.se.get_stepcompress().expect(
            "TypeError: syncemitter was allocated without a stepcompress (alloc_stepcompress=False)",
        );
        sc.lock()
            .fill(oid, max_error, queue_step_msgtag, set_next_step_dir_msgtag);
    }

    fn stepcompress_set_invert_sdir(&self, invert: u32) {
        let sc = self.se.get_stepcompress().expect(
            "TypeError: syncemitter was allocated without a stepcompress (alloc_stepcompress=False)",
        );
        sc.lock().set_invert_sdir(invert);
    }

    fn stepcompress_reset(&self, last_step_clock: u64) -> i32 {
        let sc = self.se.get_stepcompress().expect(
            "TypeError: syncemitter was allocated without a stepcompress (alloc_stepcompress=False)",
        );
        let ret = sc.lock().reset(last_step_clock);
        ret
    }

    fn stepcompress_set_last_position(&self, clock: u64, last_position: i64) -> i32 {
        let sc = self.se.get_stepcompress().expect(
            "TypeError: syncemitter was allocated without a stepcompress (alloc_stepcompress=False)",
        );
        let ret = sc.lock().set_last_position(clock, last_position);
        ret
    }

    fn stepcompress_find_past_position(&self, clock: u64) -> i64 {
        let sc = self.se.get_stepcompress().expect(
            "TypeError: syncemitter was allocated without a stepcompress (alloc_stepcompress=False)",
        );
        let ret = sc.lock().find_past_position(clock);
        ret
    }

    fn stepcompress_extract_old(
        &self,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> Vec<PullHistoryStep> {
        let sc = self.se.get_stepcompress().expect(
            "TypeError: syncemitter was allocated without a stepcompress (alloc_stepcompress=False)",
        );
        let rows = sc.lock().extract_old(max, start_clock, end_clock);
        rows
            .iter()
            .map(|hs| PullHistoryStep {
                first_clock: hs.first_clock as i64,
                last_clock: hs.last_clock as i64,
                start_position: hs.start_position,
                interval: i64::from(hs.interval),
                step_count: i64::from(hs.step_count),
                add: i64::from(hs.add),
            })
            .collect()
    }
}

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
