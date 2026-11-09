// Port of `klippy/extras/motion_queuing.py` (the whole file).
//
// This module is the host-side bridge between the Python motion code and
// the C `trapq` / `steppersync` / `itersolve` helpers. The orchestration
// layer ported here reproduces the Python object faithfully:
//
// - [`PrinterMotionQueuing`] with the same field defaults and methods as
//   `PrinterMotionQueuing` ([`motion_queuing.py:75`]), including the
//   `[DEBUG-motion] v4` timing rings and log lines.
// - [`DebugFlushBatch`] (`motion_queuing.py:45`) with the same fields and
//   the same `desc()` format.
// - [`debug_callback_name`] (`motion_queuing.py:31`).
//
// The C boundary is the [`MotionChelper`] trait. Its methods mirror one
// `ffi_lib` entry each (`trapq_alloc`, `trapq_finalize_moves`,
// `trapq_append`, `steppersyncmgr_alloc`, ...). The production
// implementation [`NativeChelper`] drives the real `chelper_motion`
// `trapq` and `steppersync` ports (`trapq_*` plus `steppersyncmgr_alloc`,
// `steppersyncmgr_gen_steps` and friends), exactly where Python calls
// `steppersync_alloc_syncemitter`, `steppersync_setup_movequeue`,
// `steppersync_set_time` and `steppersyncmgr_gen_steps`.

use std::collections::VecDeque;
use std::sync::Arc;

use chelper_serial::serialqueue::SerialQueue;
use klippy_core::motion_handles::{trapq_of, CoreSyncEmitter, TrapqHandle};
use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::{Mcu, McuApi};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject, interface, missing_interface};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::stepper::Trapq;
use klippy_api::stepper::StepperKinematics as StepperKinematicsHandle;
use klippy_api::toolhead::{FlushCallback, MotionQueuingApi, SyncEmitterHandle, TrapqAppend};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor, Timer, NEVER, NOW};

// ---------------------------------------------------------------------------
// Module constants (`motion_queuing.py:9-29`)
// ---------------------------------------------------------------------------

/// `BGFLUSH_LOW_TIME` (`motion_queuing.py:9`).
pub const BGFLUSH_LOW_TIME: f64 = 0.200;
/// `BGFLUSH_HIGH_TIME` (`motion_queuing.py:10`).
pub const BGFLUSH_HIGH_TIME: f64 = 0.400;
/// `BGFLUSH_SG_LOW_TIME` (`motion_queuing.py:11`).
pub const BGFLUSH_SG_LOW_TIME: f64 = 1.000;
/// `BGFLUSH_SG_HIGH_TIME` (`motion_queuing.py:12`).
pub const BGFLUSH_SG_HIGH_TIME: f64 = 1.250;
/// `BGFLUSH_EXTRA_TIME` (`motion_queuing.py:13`).
pub const BGFLUSH_EXTRA_TIME: f64 = 0.250;

/// `MOVE_HISTORY_EXPIRE` (`motion_queuing.py:15`).
pub const MOVE_HISTORY_EXPIRE: f64 = 30.;
/// `MIN_KIN_TIME` (`motion_queuing.py:16`).
pub const MIN_KIN_TIME: f64 = 0.100;
/// `STEPCOMPRESS_FLUSH_TIME` (`motion_queuing.py:17`).
pub const STEPCOMPRESS_FLUSH_TIME: f64 = 0.050;
/// `SDS_CHECK_TIME` (`motion_queuing.py:18`).
pub const SDS_CHECK_TIME: f64 = 0.001;

/// `DRIP_SEGMENT_TIME` (`motion_queuing.py:20`).
pub const DRIP_SEGMENT_TIME: f64 = 0.050;
/// `DRIP_TIME` (`motion_queuing.py:21`).
pub const DRIP_TIME: f64 = 0.100;

/// `DEBUG_RING_SIZE` (`motion_queuing.py:24`).
pub const DEBUG_RING_SIZE: usize = 32;
/// `DEBUG_SLOW_RING_SIZE` (`motion_queuing.py:25`).
pub const DEBUG_SLOW_RING_SIZE: usize = 16;
/// `DEBUG_WAKE_RING_SIZE` (`motion_queuing.py:26`).
pub const DEBUG_WAKE_RING_SIZE: usize = 48;
/// `DEBUG_SLOW_FLUSH_TIME` (`motion_queuing.py:27`).
pub const DEBUG_SLOW_FLUSH_TIME: f64 = 0.010;
/// `DEBUG_LATE_WAKE_TIME` (`motion_queuing.py:28`).
pub const DEBUG_LATE_WAKE_TIME: f64 = 0.020;
/// `DEBUG_DUMP_ENTRIES` (`motion_queuing.py:29`).
pub const DEBUG_DUMP_ENTRIES: usize = 10;

// ---------------------------------------------------------------------------
// `debug_callback_name` (`motion_queuing.py:31`)
// ---------------------------------------------------------------------------

/// `debug_callback_name` (`motion_queuing.py:31`): the display name used on
/// the `[DEBUG-motion]` reporting path.
///
/// The Python version takes the callback object and reads `__qualname__`
/// (falling back to `__name__`, then the type name), appends the
/// `__self__._name` of a bound method as `name(obj)` when present, and
/// truncates to 48 characters. Rust flush callbacks
/// ([`FlushCallback`]) carry no Python attributes, so callers resolve the
/// same inputs first: `name` is the resolved function name (`__qualname__`
/// / `__name__` / type name equivalent) and `obj_name` is the `__self__`
/// `_name` when the callback is a bound method. `None` name stands for the
/// `cb is None` case and yields `"-"`.
pub fn debug_callback_name(name: Option<&str>, obj_name: Option<&str>) -> String {
    let mut s = match name {
        None => "-".to_string(),
        Some(n) => n.to_string(),
    };
    if let Some(o) = obj_name {
        if !o.is_empty() {
            s = fmt("%s(%s)", vec![PctValue::Str(s), PctValue::Str(o.to_string())]);
        }
    }
    // Python `name[:48]` slices characters, not bytes.
    s.chars().take(48).collect()
}

// ---------------------------------------------------------------------------
// `DebugFlushBatch` (`motion_queuing.py:45`)
// ---------------------------------------------------------------------------

/// Phase timing for one `_advance_flush_time()` batch (host side only)
/// (`motion_queuing.py:45`).
#[derive(Clone, Debug)]
pub struct DebugFlushBatch {
    /// `seq` (`motion_queuing.py:47`).
    pub seq: i64,
    /// `start` (`motion_queuing.py:47`).
    pub start: f64,
    /// `flush_time` (`motion_queuing.py:47`).
    pub flush_time: f64,
    /// `step_gen_time` (`motion_queuing.py:47`).
    pub step_gen_time: f64,
    /// `prev_flush_time` (`motion_queuing.py:48`).
    pub prev_flush_time: f64,
    /// `prev_step_gen_time` (`motion_queuing.py:48`).
    pub prev_step_gen_time: f64,
    /// `cb_time` (`motion_queuing.py:49`).
    pub cb_time: f64,
    /// `gen_time` (`motion_queuing.py:49`).
    pub gen_time: f64,
    /// `fin_time` (`motion_queuing.py:49`).
    pub fin_time: f64,
    /// `total_time` (`motion_queuing.py:49`).
    pub total_time: f64,
    /// `stage` (`motion_queuing.py:50`).
    pub stage: String,
    /// `slow_cb_time` (`motion_queuing.py:50`).
    pub slow_cb_time: f64,
    /// `slow_cb_name` (`motion_queuing.py:50`).
    pub slow_cb_name: String,
    /// `cb_count` (`motion_queuing.py:50`).
    pub cb_count: i64,
    /// `trapq_count` (`motion_queuing.py:50`).
    pub trapq_count: i64,
}

impl DebugFlushBatch {
    /// `desc()` (`motion_queuing.py:64`).
    pub fn desc(&self) -> String {
        fmt(
            "seq=%d at=%.6f total=%.6f cb=%.6f gen=%.6f fin=%.6f \
             stage=%s flush=%.9f step_gen=%.9f prev_flush=%.9f \
             prev_step_gen=%.9f cbs=%d slow_cb=%.6f slow_cb_name=%s \
             trapqs=%d",
            vec![
                PctValue::Int(PyInt::from_i64(self.seq)),
                PctValue::Float(self.start),
                PctValue::Float(self.total_time),
                PctValue::Float(self.cb_time),
                PctValue::Float(self.gen_time),
                PctValue::Float(self.fin_time),
                PctValue::Str(self.stage.clone()),
                PctValue::Float(self.flush_time),
                PctValue::Float(self.step_gen_time),
                PctValue::Float(self.prev_flush_time),
                PctValue::Float(self.prev_step_gen_time),
                PctValue::Int(PyInt::from_i64(self.cb_count)),
                PctValue::Float(self.slow_cb_time),
                PctValue::Str(self.slow_cb_name.clone()),
                PctValue::Int(PyInt::from_i64(self.trapq_count)),
            ],
        )
    }
}

/// One `[DEBUG-motion]` wake event (`_debug_note_wake`,
/// `motion_queuing.py:401`): fields in tuple order
/// `(eventtime, wake_late, duration, nbatch, ret_in, last_flush_time,
/// last_step_gen_time, need_flush_time, need_step_gen_time)`.
#[derive(Clone, Debug, PartialEq)]
pub struct WakeEvent {
    /// `eventtime`.
    pub eventtime: f64,
    /// `wake_late`.
    pub wake_late: f64,
    /// `duration`.
    pub duration: f64,
    /// `nbatch`.
    pub nbatch: i64,
    /// `ret_in`.
    pub ret_in: f64,
    /// `last_flush_time`.
    pub last_flush_time: f64,
    /// `last_step_gen_time`.
    pub last_step_gen_time: f64,
    /// `need_flush_time`.
    pub need_flush_time: f64,
    /// `need_step_gen_time`.
    pub need_step_gen_time: f64,
}

// ---------------------------------------------------------------------------
// C-helper seam (`chelper.get_ffi()` in the Python module)
// ---------------------------------------------------------------------------

/// Opaque `steppersyncmgr` handle from the C layer
/// (`motion_queuing.py:84`).
pub type StepperSyncMgrHandle = Arc<dyn std::any::Any + Send + Sync>;
/// Opaque `steppersync` handle from the C layer (`motion_queuing.py:150`).
pub type StepperSyncHandle = Arc<dyn std::any::Any + Send + Sync>;

/// The C boundary (`chelper.get_ffi()` in `motion_queuing.py`): one method
/// per `ffi_lib` function the module calls.
pub trait MotionChelper: Send + Sync + 'static {
    /// `trapq_alloc` (`motion_queuing.py:135`).
    fn trapq_alloc(&self) -> Trapq;
    /// `trapq_finalize_moves` (`motion_queuing.py:82,140,345`).
    fn trapq_finalize_moves(&self, tq: &Trapq, print_time: f64, clear_history_time: f64);
    /// `trapq_append` (`motion_queuing.py:143`).
    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        &self,
        tq: Trapq,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos_x: f64,
        start_pos_y: f64,
        start_pos_z: f64,
        axes_r_x: f64,
        axes_r_y: f64,
        axes_r_z: f64,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    );
    /// `steppersyncmgr_alloc` (`motion_queuing.py:84`).
    fn steppersyncmgr_alloc(&self) -> StepperSyncMgrHandle;
    /// `steppersyncmgr_free` (`motion_queuing.py:85`, via `gc`).
    fn steppersyncmgr_free(&self, mgr: &StepperSyncMgrHandle);
    /// `steppersyncmgr_alloc_steppersync` (`motion_queuing.py:150`).
    fn steppersyncmgr_alloc_steppersync(&self, mgr: &StepperSyncMgrHandle) -> StepperSyncHandle;
    /// `steppersync_alloc_syncemitter` (`motion_queuing.py:158`): `name` is
    /// the first 15 UTF-8 bytes of the last token (see
    /// [`PrinterMotionQueuing::allocate_syncemitter`]).
    fn steppersync_alloc_syncemitter(
        &self,
        ss: &StepperSyncHandle,
        name: &[u8; 15],
        alloc_stepcompress: bool,
    ) -> SyncEmitterHandle;
    /// `syncemitter_queue_msg` (`klippy/stepper.py:210`, `pwm_tool.py:110`):
    /// queue a message on a syncemitter; `data` holds the `uint32_t` words.
    fn syncemitter_queue_msg(&self, se: &SyncEmitterHandle, req_clock: u64, data: &[u32]);
    /// `steppersync_setup_movequeue` (`motion_queuing.py:167`):
    /// `serialqueue` is `mcu._serial.get_serialqueue()` (`mcu.py:1226`).
    fn steppersync_setup_movequeue(
        &self,
        ss: &StepperSyncHandle,
        serialqueue: Option<std::sync::Arc<SerialQueue>>,
        move_count: i64,
    );
    /// `steppersync_set_time` (`motion_queuing.py:169,176`).
    fn steppersync_set_time(&self, ss: &StepperSyncHandle, time_offset: f64, mcu_freq: f64);
    /// `steppersyncmgr_gen_steps` (`motion_queuing.py:89,323`): returns the
    /// failing syncemitter, or `None` (the C `NULL`).
    fn steppersyncmgr_gen_steps(
        &self,
        mgr: &StepperSyncMgrHandle,
        flush_time: f64,
        gen_steps_time: f64,
        clear_history_time: f64,
    ) -> Option<SyncEmitterHandle>;
    /// `syncemitter_get_stepper_kinematics` (`motion_queuing.py:199`).
    fn syncemitter_get_stepper_kinematics(
        &self,
        se: &SyncEmitterHandle,
    ) -> Option<StepperKinematicsHandle>;
    /// `itersolve_get_trapq` (`motion_queuing.py:202`).
    fn itersolve_get_trapq(&self, sk: &StepperKinematicsHandle) -> Option<Trapq>;
    /// `itersolve_get_gen_steps_pre_active` (`motion_queuing.py:205`).
    fn itersolve_get_gen_steps_pre_active(&self, sk: &StepperKinematicsHandle) -> f64;
    /// `itersolve_get_gen_steps_post_active` (`motion_queuing.py:206`).
    fn itersolve_get_gen_steps_post_active(&self, sk: &StepperKinematicsHandle) -> f64;
}

// --- Native (production) objects for the `steppersync` C structs -----------
/// The concrete `steppersyncmgr` payload ([`StepperSyncMgrHandle`]): the
/// real ported manager plus the contract handles of every emitter allocated
/// through it. The table maps a failed ported emitter back to the exact
/// `SyncEmitterHandle` the callers hold, so the `syncemitter_to_name` error
/// path (`motion_queuing.py:328-338`) identifies it by `Arc::ptr_eq`, as
/// the C identifies the `struct syncemitter *`.
pub struct StepperSyncMgr {
    inner: chelper_motion::steppersync::SteppersSyncMgr,
    emitters: kos::sync::Mutex<Vec<(usize, SyncEmitterHandle)>>,
}

impl StepperSyncMgr {
    fn mgr(&self) -> &chelper_motion::steppersync::SteppersSyncMgr {
        &self.inner
    }

    /// Remember `handle` (wrapping the ported emitter at `raw`) for the
    /// failed-emitter lookup below.
    fn remember(&self, raw: &Arc<chelper_motion::steppersync::SyncEmitter>, handle: &SyncEmitterHandle) {
        let key = Arc::as_ptr(raw) as *const () as usize;
        self.emitters.lock().push((key, handle.clone()));
    }

    /// The contract handle wrapping `failed`, by `Arc` identity.
    fn lookup(&self, failed: &Arc<chelper_motion::steppersync::SyncEmitter>) -> Option<SyncEmitterHandle> {
        let key = Arc::as_ptr(failed) as *const () as usize;
        self.emitters
            .lock()
            .iter()
            .find(|(k, _)| *k == key)
            .map(|(_, h)| h.clone())
    }
}

/// The concrete `steppersync` payload ([`StepperSyncHandle`]): the real
/// ported per-MCU object.
pub struct StepperSync {
    inner: Arc<chelper_motion::steppersync::SteppersSync>,
}

impl StepperSync {
    /// The command queue messages are sent on (the C `ss->cq`), `None`
    /// before [`MotionChelper::steppersync_setup_movequeue`].
    pub fn command_queue(&self) -> Option<Arc<chelper_serial::serialqueue::CommandQueue>> {
        self.inner.command_queue()
    }
}

/// The production [`MotionChelper`]: `trapq` ops run on the real
/// `chelper_motion::trapq` port and every `steppersync*` call drives the
/// real `chelper_motion::steppersync` port. Dropping the
/// [`PrinterMotionQueuing`] drops the manager (`steppersyncmgr_free`,
/// which stops every emitter thread and frees every command queue); the
/// C `gc` registration pairs alloc/free the same way.
pub struct NativeChelper;

impl NativeChelper {
    /// A fresh native helper.
    pub fn new() -> NativeChelper {
        NativeChelper
    }
}

impl Default for NativeChelper {
    fn default() -> NativeChelper {
        Self::new()
    }
}

impl MotionChelper for NativeChelper {
    fn trapq_alloc(&self) -> Trapq {
        TrapqHandle::alloc()
    }

    fn trapq_finalize_moves(&self, tq: &Trapq, print_time: f64, clear_history_time: f64) {
        trapq_of(tq)
            .expect("trapq allocated through motion_handles")
            .shared()
            .lock()
            .finalize_moves(print_time, clear_history_time);
    }

    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        &self,
        tq: Trapq,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos_x: f64,
        start_pos_y: f64,
        start_pos_z: f64,
        axes_r_x: f64,
        axes_r_y: f64,
        axes_r_z: f64,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        trapq_of(&tq).expect("trapq allocated through motion_handles").shared().lock().append(
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            start_pos_x,
            start_pos_y,
            start_pos_z,
            axes_r_x,
            axes_r_y,
            axes_r_z,
            start_v,
            cruise_v,
            accel,
        );
    }

    fn steppersyncmgr_alloc(&self) -> StepperSyncMgrHandle {
        let mgr: Arc<StepperSyncMgr> = Arc::new(StepperSyncMgr {
            inner: chelper_motion::steppersync::SteppersSyncMgr::alloc(),
            emitters: kos::sync::Mutex::new(Vec::new()),
        });
        mgr
    }

    fn steppersyncmgr_free(&self, _mgr: &StepperSyncMgrHandle) {
        // Reference-counted in Rust; the last `Arc` drop runs
        // `steppersyncmgr_free` (stops every emitter thread, frees every
        // command queue). The C `gc` registration pairs alloc/free the
        // same way.
    }

    fn steppersyncmgr_alloc_steppersync(&self, mgr: &StepperSyncMgrHandle) -> StepperSyncHandle {
        let mgr = mgr
            .downcast_ref::<StepperSyncMgr>()
            .expect("steppersyncmgr allocated through NativeChelper");
        let ss: Arc<StepperSync> = Arc::new(StepperSync { inner: mgr.mgr().alloc_steppersync() });
        ss
    }

    fn steppersync_alloc_syncemitter(
        &self,
        ss: &StepperSyncHandle,
        name: &[u8; 15],
        alloc_stepcompress: bool,
    ) -> SyncEmitterHandle {
        let ss = ss
            .downcast_ref::<StepperSync>()
            .expect("steppersync allocated through NativeChelper");
        CoreSyncEmitter::wrap(ss.inner.alloc_syncemitter(name, alloc_stepcompress))
    }

    fn syncemitter_queue_msg(&self, se: &SyncEmitterHandle, req_clock: u64, data: &[u32]) {
        se.queue_msg(req_clock, data);
    }

    fn steppersync_setup_movequeue(
        &self,
        ss: &StepperSyncHandle,
        serialqueue: Option<std::sync::Arc<SerialQueue>>,
        move_count: i64,
    ) {
        let ss = ss
            .downcast_ref::<StepperSync>()
            .expect("steppersync allocated through NativeChelper");
        // `serialqueue` is `mcu._serial.get_serialqueue()` (`mcu.py:1226`,
        // `motion_queuing.py:163-167`). `None` (no serial session yet)
        // leaves the sync unconfigured; the port's flush then queues
        // instead of crashing on the C NULL `sq` (`steppersync.rs`
        // documents the difference).
        if let Some(sq) = serialqueue {
            let move_num = usize::try_from(move_count).unwrap_or(0);
            ss.inner.setup_movequeue(sq, move_num);
        }
    }

    fn steppersync_set_time(&self, ss: &StepperSyncHandle, time_offset: f64, mcu_freq: f64) {
        let ss = ss
            .downcast_ref::<StepperSync>()
            .expect("steppersync allocated through NativeChelper");
        ss.inner.set_time(time_offset, mcu_freq);
    }

    fn steppersyncmgr_gen_steps(
        &self,
        mgr: &StepperSyncMgrHandle,
        flush_time: f64,
        gen_steps_time: f64,
        clear_history_time: f64,
    ) -> Option<SyncEmitterHandle> {
        let mgr = mgr
            .downcast_ref::<StepperSyncMgr>()
            .expect("steppersyncmgr allocated through NativeChelper");
        // The port returns the failed emitter; map it back to the exact
        // contract handle the callers hold (by `Arc` identity, as the C
        // returns the same `struct syncemitter *`), so the
        // `syncemitter_to_name` error path names it. Do not hold any
        // stepcompress lock here; `gen_steps` locks them on the background
        // threads.
        let failed = mgr.mgr().gen_steps(flush_time, gen_steps_time, clear_history_time);
        match failed {
            Some(se) => mgr.lookup(&se).or_else(|| Some(CoreSyncEmitter::wrap(se))),
            None => None,
        }
    }

    fn syncemitter_get_stepper_kinematics(
        &self,
        se: &SyncEmitterHandle,
    ) -> Option<StepperKinematicsHandle> {
        se.get_stepper_kinematics()
    }

    fn itersolve_get_trapq(&self, sk: &StepperKinematicsHandle) -> Option<Trapq> {
        klippy_core::motion_handles::kinematics_of(sk)
            .ok()
            .and_then(|h| h.with(|a| a.sk().tq.clone().map(TrapqHandle::from_shared)))
    }

    fn itersolve_get_gen_steps_pre_active(&self, sk: &StepperKinematicsHandle) -> f64 {
        klippy_core::motion_handles::kinematics_of(sk)
            .ok()
            .map(|h| h.with(|a| a.sk().gen_steps_pre_active))
            .unwrap_or(0.0)
    }

    fn itersolve_get_gen_steps_post_active(&self, sk: &StepperKinematicsHandle) -> f64 {
        klippy_core::motion_handles::kinematics_of(sk)
            .ok()
            .map(|h| h.with(|a| a.sk().gen_steps_post_active))
            .unwrap_or(0.0)
    }
}



// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

type BatchShared = Arc<GCell<DebugFlushBatch>>;

struct FlushCbEntry {
    cb: FlushCallback,
    // Resolved display name for `debug_callback_name`; `None` for a plain
    // closure (Python would report the function's `__qualname__`).
    name: Option<String>,
}

struct State {
    trapqs: Vec<Trapq>,
    syncemitters: Vec<SyncEmitterHandle>,
    syncemitter_to_name: Vec<(SyncEmitterHandle, String)>,
    steppersyncs: Vec<(Mcu, StepperSyncHandle)>,
    flush_callbacks: Vec<FlushCbEntry>,
    kin_flush_delay: f64,
    clear_history_time: f64,
    can_pause: bool,
    do_kick_flush_timer: bool,
    last_flush_time: f64,
    last_step_gen_time: f64,
    need_flush_time: f64,
    need_step_gen_time: f64,
    debug_flush_time: f64,
    debug_step_gen_time: f64,
    debug_flush_stage: String,
    debug_batch_seq: i64,
    debug_batch_count: i64,
    debug_batch_pending: Option<BatchShared>,
    debug_failed_batch: Option<BatchShared>,
    debug_worst_batch: Option<BatchShared>,
    debug_batches: VecDeque<BatchShared>,
    debug_slow_batches: VecDeque<BatchShared>,
    debug_wake_events: VecDeque<WakeEvent>,
    debug_batch_totals: [f64; 4],
    debug_expect_wake: f64,
    debug_worst_wake_late: f64,
    debug_wake_count: i64,
    debug_late_wake_count: i64,
    drip_start_times: Vec<f64>,
    flush_timer: Option<Timer>,
}

impl State {
    fn new(can_pause: bool) -> State {
        State {
            trapqs: Vec::new(),
            syncemitters: Vec::new(),
            syncemitter_to_name: Vec::new(),
            steppersyncs: Vec::new(),
            flush_callbacks: Vec::new(),
            kin_flush_delay: SDS_CHECK_TIME,
            clear_history_time: 0.0,
            can_pause,
            do_kick_flush_timer: true,
            last_flush_time: 0.0,
            last_step_gen_time: 0.0,
            need_flush_time: 0.0,
            need_step_gen_time: 0.0,
            debug_flush_time: 0.0,
            debug_step_gen_time: 0.0,
            debug_flush_stage: "idle".to_string(),
            debug_batch_seq: 0,
            debug_batch_count: 0,
            debug_batch_pending: None,
            debug_failed_batch: None,
            debug_worst_batch: None,
            debug_batches: VecDeque::new(),
            debug_slow_batches: VecDeque::new(),
            debug_wake_events: VecDeque::new(),
            debug_batch_totals: [0.0, 0.0, 0.0, 0.0],
            debug_expect_wake: 0.0,
            debug_worst_wake_late: 0.0,
            debug_wake_count: 0,
            debug_late_wake_count: 0,
            drip_start_times: Vec::new(),
            flush_timer: None,
        }
    }
}

fn push_bounded<T>(deque: &mut VecDeque<T>, item: T, maxlen: usize) {
    deque.push_back(item);
    while deque.len() > maxlen {
        deque.pop_front();
    }
}

fn fmt(format: &str, args: Vec<PctValue>) -> String {
    percent_format(format, &args).expect("motion_queuing format string is valid")
}

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

// ---------------------------------------------------------------------------
// `PrinterMotionQueuing` (`motion_queuing.py:75`)
// ---------------------------------------------------------------------------

/// `PrinterMotionQueuing` (`motion_queuing.py:75`).
pub struct PrinterMotionQueuing {
    printer: Printer,
    reactor: Reactor,
    mcu: Mcu,
    all_mcus: Vec<Mcu>,
    chelper: Arc<dyn MotionChelper>,
    steppersyncmgr: StepperSyncMgrHandle,
    clock: Arc<dyn Fn() -> f64 + Send + Sync>,
    state: GCell<State>,
}

impl PrinterMotionQueuing {
    /// `PrinterMotionQueuing(config)` (`motion_queuing.py:76`): the
    /// printer and reactor come from the config's printer; the C layer is
    /// the native helper; the clock is `reactor.monotonic`.
    pub fn from_config(config: &Config) -> KResult<Arc<PrinterMotionQueuing>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let objects = printer.lookup_objects(Some("mcu"));
        let mut mcus = Vec::new();
        for (name, obj) in objects {
            let mcu: Mcu = interface::<dyn McuApi>(&obj)
                .ok_or_else(|| missing_interface::<dyn McuApi>(&name))?;
            mcus.push(mcu);
        }
        Self::new_with_clock(
            printer,
            reactor,
            mcus,
            Arc::new(NativeChelper::new()),
            None,
        )
    }

    /// Create the object with explicit collaborators (the form the tests
    /// drive: fake MCUs and a fake [`MotionChelper`] with a scripted
    /// clock). `clock` replaces `self.debug_now` (`motion_queuing.py:113`);
    /// `None` uses `reactor.monotonic()`.
    pub fn new_with_clock(
        printer: Printer,
        reactor: Reactor,
        mcus: Vec<Mcu>,
        chelper: Arc<dyn MotionChelper>,
        clock: Option<Arc<dyn Fn() -> f64 + Send + Sync>>,
    ) -> KResult<Arc<PrinterMotionQueuing>> {
        // `self.mcu = self.all_mcus[0]` (`motion_queuing.py:98`).
        let Some(mcu) = mcus.first().cloned() else {
            return Err(index_error());
        };
        let can_pause = !mcu.is_fileoutput();
        let flush_debug = !can_pause;
        let steppersyncmgr = chelper.steppersyncmgr_alloc();
        let clock: Arc<dyn Fn() -> f64 + Send + Sync> = match clock {
            Some(c) => c,
            None => {
                let r = reactor.clone();
                Arc::new(move || r.monotonic())
            }
        };
        let this = Arc::new(PrinterMotionQueuing {
            printer: printer.clone(),
            reactor: reactor.clone(),
            mcu,
            all_mcus: mcus,
            chelper,
            steppersyncmgr,
            clock,
            state: GCell::new(State::new(can_pause)),
        });
        // `self.flush_timer = self.reactor.register_timer(flush_handler)`
        // (`motion_queuing.py:103-106`). The handler is chosen once, here,
        // from `can_pause` at init (`motion_queuing.py:104-105`); later
        // `can_pause` changes do not swap it.
        {
            let weak = Arc::downgrade(&this);
            let timer = reactor.register_timer(
                move |eventtime| {
                    let Some(s) = weak.upgrade() else {
                        return NEVER;
                    };
                    if flush_debug {
                        s._flush_handler_debug(eventtime)
                    } else {
                        s._flush_handler(eventtime)
                    }
                },
                NEVER,
            );
            this.state.with_mut(|s| s.flush_timer = Some(timer));
        }
        // `self.printer.register_event_handler(...)` (lines 126-128).
        {
            let weak = Arc::downgrade(&this);
            printer.register_event_handler(
                "klippy:shutdown",
                Box::new(move |_params: &[EventArg]| {
                    if let Some(s) = weak.upgrade() {
                        s._handle_shutdown();
                    }
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let weak = Arc::downgrade(&this);
            printer.register_event_handler(
                "klippy:analyze_shutdown",
                Box::new(move |_params: &[EventArg]| {
                    if let Some(s) = weak.upgrade() {
                        s._debug_shutdown();
                    }
                    Ok(PyValue::None)
                }),
            );
        }
        // `logging.info("[DEBUG-motion] version=host-v4 ...")` (129-131).
        log::info(&fmt(
            "[DEBUG-motion] version=host-v4 motion_queuing timing rings batches=%d slow=%d wakes=%d",
            vec![
                PctValue::Int(PyInt::from_i64(DEBUG_RING_SIZE as i64)),
                PctValue::Int(PyInt::from_i64(DEBUG_SLOW_RING_SIZE as i64)),
                PctValue::Int(PyInt::from_i64(DEBUG_WAKE_RING_SIZE as i64)),
            ],
        ));
        Ok(this)
    }

    /// `PrinterMotionQueuing(config)` with the reactor clock.
    pub fn new(
        printer: Printer,
        reactor: Reactor,
        mcus: Vec<Mcu>,
        chelper: Arc<dyn MotionChelper>,
    ) -> KResult<Arc<PrinterMotionQueuing>> {
        Self::new_with_clock(printer, reactor, mcus, chelper, None)
    }

    // --- state accessors (the attributes tests read directly in Python) --

    /// `self.mcu` (`motion_queuing.py:98`).
    pub fn mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    /// `self.can_pause` (`motion_queuing.py:99`).
    pub fn can_pause(&self) -> bool {
        self.state.borrow().can_pause
    }
    /// `self.last_flush_time`.
    pub fn last_flush_time(&self) -> f64 {
        self.state.borrow().last_flush_time
    }
    /// `self.last_step_gen_time`.
    pub fn last_step_gen_time(&self) -> f64 {
        self.state.borrow().last_step_gen_time
    }
    /// `self.need_flush_time`.
    pub fn need_flush_time(&self) -> f64 {
        self.state.borrow().need_flush_time
    }
    /// `self.need_step_gen_time`.
    pub fn need_step_gen_time(&self) -> f64 {
        self.state.borrow().need_step_gen_time
    }
    /// `self.clear_history_time`.
    pub fn clear_history_time(&self) -> f64 {
        self.state.borrow().clear_history_time
    }
    /// `self.do_kick_flush_timer`.
    pub fn do_kick_flush_timer(&self) -> bool {
        self.state.borrow().do_kick_flush_timer
    }
    /// `self.debug_flush_stage`.
    pub fn debug_flush_stage(&self) -> String {
        self.state.borrow().debug_flush_stage.clone()
    }
    /// `self.debug_batch_seq`.
    pub fn debug_batch_seq(&self) -> i64 {
        self.state.borrow().debug_batch_seq
    }
    /// `self.debug_batch_count`.
    pub fn debug_batch_count(&self) -> i64 {
        self.state.borrow().debug_batch_count
    }
    /// `self.debug_batch_totals`.
    pub fn debug_batch_totals(&self) -> [f64; 4] {
        self.state.borrow().debug_batch_totals
    }
    /// `self.debug_expect_wake`.
    pub fn debug_expect_wake(&self) -> f64 {
        self.state.borrow().debug_expect_wake
    }
    /// Set `self.debug_expect_wake` (a plain Python attribute the parity
    /// tests assign directly, like `obj.debug_expect_wake = 10.0`).
    pub fn set_debug_expect_wake(&self, value: f64) {
        self.state.with_mut(|s| s.debug_expect_wake = value);
    }
    /// `self.debug_wake_count` / late count / worst late.
    pub fn debug_wake_counts(&self) -> (i64, i64, f64) {
        let s = self.state.borrow();
        (s.debug_wake_count, s.debug_late_wake_count, s.debug_worst_wake_late)
    }
    /// Snapshot of `self.debug_batch_pending`, as a `desc()` string.
    pub fn debug_batch_pending_desc(&self) -> Option<String> {
        let shared = self.state.borrow().debug_batch_pending.clone();
        shared.map(|b| b.borrow().desc())
    }
    /// `desc()` of the recorded batches in `self.debug_batches` order.
    pub fn debug_batches_desc(&self) -> Vec<String> {
        self.state.borrow().debug_batches.iter().map(|b| b.borrow().desc()).collect()
    }
    /// `desc()` of the recorded batches in `self.debug_slow_batches` order.
    pub fn debug_slow_batches_desc(&self) -> Vec<String> {
        self.state.borrow().debug_slow_batches.iter().map(|b| b.borrow().desc()).collect()
    }
    /// Snapshot of `self.debug_wake_events`.
    pub fn debug_wake_events(&self) -> Vec<WakeEvent> {
        self.state.borrow().debug_wake_events.iter().cloned().collect()
    }
    /// `self.debug_failed_batch` / `self.debug_worst_batch` as `desc()`.
    pub fn debug_failed_worst_desc(&self) -> (Option<String>, Option<String>) {
        let s = self.state.borrow();
        (
            s.debug_failed_batch.as_ref().map(|b| b.borrow().desc()),
            s.debug_worst_batch.as_ref().map(|b| b.borrow().desc()),
        )
    }
    /// The registered flush callbacks, in order.
    pub fn flush_callbacks_len(&self) -> usize {
        self.state.borrow().flush_callbacks.len()
    }
    /// `len(self.trapqs)`.
    pub fn trapq_count(&self) -> usize {
        self.state.borrow().trapqs.len()
    }
    /// `len(self.syncemitters)`.
    pub fn syncemitter_count(&self) -> usize {
        self.state.borrow().syncemitters.len()
    }
    /// `len(self.steppersyncs)`.
    pub fn steppersync_count(&self) -> usize {
        self.state.borrow().steppersyncs.len()
    }
    /// `self.all_mcus` (`motion_queuing.py:97`).
    pub fn all_mcus(&self) -> Vec<Mcu> {
        self.all_mcus.clone()
    }
    /// `self.syncemitter_to_name.get(se)` for the failed-emitter path.
    pub fn syncemitter_to_name(&self, se: &SyncEmitterHandle) -> Option<String> {
        self.state.borrow().syncemitter_to_name.iter().find_map(|(k, v)| {
            if Arc::ptr_eq(k, se) { Some(v.clone()) } else { None }
        })
    }
    /// `self.drip_start_times` (`motion_queuing.py:124`).
    pub fn drip_start_times(&self) -> Vec<f64> {
        self.state.borrow().drip_start_times.clone()
    }

    // --- C trapq tracking (`motion_queuing.py:132-143`) -------------------

    /// `allocate_trapq` (`motion_queuing.py:133`).
    pub fn allocate_trapq(&self) -> Trapq {
        let trapq = self.chelper.trapq_alloc();
        self.state.with_mut(|s| s.trapqs.push(trapq.clone()));
        trapq
    }

    /// `wipe_trapq` (`motion_queuing.py:138`).
    pub fn wipe_trapq(&self, trapq: &Trapq) {
        self.chelper.trapq_finalize_moves(trapq, NEVER, 0.0);
    }

    /// `lookup_trapq_append` (`motion_queuing.py:141`).
    pub fn lookup_trapq_append(&self) -> TrapqAppend {
        let chelper = self.chelper.clone();
        Arc::new(move |tq: &Trapq,
                  print_time: f64,
                  accel_t: f64,
                  cruise_t: f64,
                  decel_t: f64,
                  x: f64,
                  y: f64,
                  z: f64,
                  axes_rx: f64,
                  axes_ry: f64,
                  axes_rz: f64,
                  start_v: f64,
                  cruise_v: f64,
                  accel: f64| {
            chelper.trapq_append(
                tq.clone(),
                print_time,
                accel_t,
                cruise_t,
                decel_t,
                x,
                y,
                z,
                axes_rx,
                axes_ry,
                axes_rz,
                start_v,
                cruise_v,
                accel,
            );
        })
    }

    // --- C steppersync tracking (`motion_queuing.py:144-169`) --------------

    /// `_lookup_steppersync` (`motion_queuing.py:145`): identity (`is`)
    /// search over `self.steppersyncs`, allocating on first use.
    pub fn _lookup_steppersync(&self, mcu: &Mcu) -> StepperSyncHandle {
        {
            let ops = self.state.borrow().steppersyncs.clone();
            if let Some((_, ss)) = ops.iter().find(|(m, _)| Arc::ptr_eq(m, mcu)) {
                return ss.clone();
            }
        }
        let ss = self.chelper.steppersyncmgr_alloc_steppersync(&self.steppersyncmgr);
        self.state.with_mut(|s| s.steppersyncs.push((mcu.clone(), ss.clone())));
        ss
    }

    /// `allocate_syncemitter` (`motion_queuing.py:153`).
    ///
    /// Python: `sname = name.split()[-1]`,
    /// `thread_name = sname.encode("utf-8")[:15]`, then
    /// `syncemitter_to_name[se] = name` (the *full* name). An empty or
    /// whitespace-only `name` raises `IndexError("list index out of
    /// range")` in Python, from `[-1]` on an empty list.
    pub fn allocate_syncemitter(
        &self,
        mcu: Mcu,
        name: &str,
        alloc_stepcompress: bool,
    ) -> KResult<SyncEmitterHandle> {
        let sname = name.split_whitespace().last().ok_or_else(index_error)?;
        let mut thread_name = [0u8; 15];
        let raw = sname.as_bytes();
        let n = raw.len().min(15);
        thread_name[..n].copy_from_slice(&raw[..n]);
        let ss = self._lookup_steppersync(&mcu);
        let se = self.chelper.steppersync_alloc_syncemitter(&ss, &thread_name, alloc_stepcompress);
        // Remember the raw-pointer -> handle mapping for the failed-emitter
        // lookup in `steppersyncmgr_gen_steps` below.
        if let Some(mgr) = self.steppersyncmgr.downcast_ref::<StepperSyncMgr>() {
            if let Some(core) = se.as_any().downcast_ref::<CoreSyncEmitter>() {
                mgr.remember(core.emitter(), &se);
            }
        }
        self.state.with_mut(|s| {
            s.syncemitter_to_name.push((se.clone(), name.to_string()));
            s.syncemitters.push(se.clone());
        });
        Ok(se)
    }

    /// `syncemitter_queue_msg` (`klippy/stepper.py:210`): queue a message
    /// on a syncemitter from [`Self::allocate_syncemitter`].
    pub fn queue_syncemitter_msg(&self, syncemitter: &SyncEmitterHandle, req_clock: u64, data: &[u32]) {
        self.chelper.syncemitter_queue_msg(syncemitter, req_clock, data);
    }

    /// `setup_mcu_movequeue` (`motion_queuing.py:163`).
    pub fn setup_mcu_movequeue(&self, mcu: Mcu, serialqueue: Option<std::sync::Arc<SerialQueue>>, move_count: i64) {
        let ss = self._lookup_steppersync(&mcu);
        self.chelper.steppersync_setup_movequeue(&ss, serialqueue, move_count);
        // `mcu_freq = float(mcu.seconds_to_clock(1.))`.
        let mcu_freq = mcu.seconds_to_clock(1.) as f64;
        self.chelper.steppersync_set_time(&ss, 0.0, mcu_freq);
    }

    /// `stats` (`motion_queuing.py:170`): calibrate every steppersync
    /// clock, then expire history. Python returns `(False, "")`.
    ///
    /// The [`MotionQueuingApi`] trait has no `stats` member (reported
    /// contract gap); core reads this inherent method through the concrete
    /// object, as Python's stats loop does through the object.
    pub fn stats(&self, eventtime: f64) -> KResult<(bool, String)> {
        let sync_time = self.last_step_gen_time();
        let syncs = self.state.borrow().steppersyncs.clone();
        for (mcu, ss) in &syncs {
            let (offset, freq) = mcu.calibrate_clock(sync_time, eventtime)?;
            self.chelper.steppersync_set_time(ss, offset, freq);
        }
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        self.state.with_mut(|s| {
            s.clear_history_time = (est_print_time - MOVE_HISTORY_EXPIRE).max(0.0);
        });
        Ok((false, String::new()))
    }

    /// `register_flush_callback` (`motion_queuing.py:182`).
    pub fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool) {
        self.register_flush_callback_named(callback, can_add_trapq, None, None);
    }

    /// [`MotionQueuingApi::register_flush_callback_named`]: store the
    /// resolved `debug_callback_name` alongside the callback, so the
    /// slow-batch record reports the same name Python would
    /// (`motion_queuing.py:31-43,313`).
    pub fn register_flush_callback_named(
        &self,
        callback: FlushCallback,
        can_add_trapq: bool,
        name: Option<&str>,
        obj_name: Option<&str>,
    ) {
        let name = debug_callback_name(name, obj_name);
        self.state.with_mut(|s| {
            if can_add_trapq {
                s.flush_callbacks.insert(0, FlushCbEntry { cb: callback, name: Some(name.clone()) });
            } else {
                s.flush_callbacks.push(FlushCbEntry { cb: callback, name: Some(name) });
            }
        });
    }

    /// `unregister_flush_callback` (`motion_queuing.py:187`).
    pub fn unregister_flush_callback(&self, callback: &FlushCallback) {
        self.state.with_mut(|s| {
            if let Some(pos) = s
                .flush_callbacks
                .iter()
                .position(|e| Arc::ptr_eq(&e.cb, callback))
            {
                s.flush_callbacks.remove(pos);
            }
        });
    }

    /// `get_kin_flush_delay` (`motion_queuing.py:193`).
    pub fn get_kin_flush_delay(&self) -> f64 {
        self.state.borrow().kin_flush_delay
    }

    /// `check_step_generation_scan_windows` (`motion_queuing.py:195`).
    pub fn check_step_generation_scan_windows(&self) {
        let syncemitters = self.state.borrow().syncemitters.clone();
        let mut kin_flush_delay = SDS_CHECK_TIME;
        for se in &syncemitters {
            let Some(sk) = self.chelper.syncemitter_get_stepper_kinematics(se) else {
                continue;
            };
            if self.chelper.itersolve_get_trapq(&sk).is_none() {
                continue;
            }
            let pre_active = self.chelper.itersolve_get_gen_steps_pre_active(&sk);
            let post_active = self.chelper.itersolve_get_gen_steps_post_active(&sk);
            kin_flush_delay = kin_flush_delay.max(pre_active).max(post_active);
        }
        self.state.with_mut(|s| s.kin_flush_delay = kin_flush_delay);
    }

    // --- Shutdown diagnostics (`motion_queuing.py:209-279`) -----------------

    /// `_handle_shutdown` (`motion_queuing.py:210`).
    pub fn _handle_shutdown(&self) {
        self.state.with_mut(|s| s.can_pause = false);
    }

    /// `_debug_shutdown` (`motion_queuing.py:212`): read the Python
    /// bookkeeping only; never flush or query hardware. Diagnostics never
    /// replace the original shutdown reason.
    pub fn _debug_shutdown(&self) {
        let snap = {
            let s = self.state.borrow();
            (
                s.debug_flush_stage.clone(),
                s.debug_flush_time,
                s.debug_step_gen_time,
                s.last_flush_time,
                s.last_step_gen_time,
                s.need_flush_time,
                s.need_step_gen_time,
                s.clear_history_time,
                s.kin_flush_delay,
                s.can_pause,
                s.do_kick_flush_timer,
                s.drip_start_times.len(),
                s.trapqs.len(),
                s.syncemitters.len(),
                s.steppersyncs.len(),
                s.flush_callbacks.len(),
            )
        };
        let args = |v: bool| PctValue::Bool(v);
        log::error(&fmt(
            "[DEBUG-motion] motion flush stage=%s attempted_flush=%.9f attempted_step_gen=%.9f last_flush=%.9f last_step_gen=%.9f need_flush=%.9f need_step_gen=%.9f clear_history=%.9f kin_flush_delay=%.9f can_pause=%s kick_timer=%s drip_count=%d trapqs=%d syncemitters=%d steppersyncs=%d flush_callbacks=%d",
            vec![
                PctValue::Str(snap.0),
                PctValue::Float(snap.1),
                PctValue::Float(snap.2),
                PctValue::Float(snap.3),
                PctValue::Float(snap.4),
                PctValue::Float(snap.5),
                PctValue::Float(snap.6),
                PctValue::Float(snap.7),
                PctValue::Float(snap.8),
                args(snap.9),
                args(snap.10),
                PctValue::Int(PyInt::from_i64(snap.11 as i64)),
                PctValue::Int(PyInt::from_i64(snap.12 as i64)),
                PctValue::Int(PyInt::from_i64(snap.13 as i64)),
                PctValue::Int(PyInt::from_i64(snap.14 as i64)),
                PctValue::Int(PyInt::from_i64(snap.15 as i64)),
            ],
        ));
        if let Err(e) = self._debug_dump_timing() {
            log::exception(
                "[DEBUG-motion] Unable to dump motion timing",
                &format!("{}: {}", e.class_name(), e.message()),
            );
        }
    }

    /// `_debug_dump_timing` (`motion_queuing.py:234`).
    pub fn _debug_dump_timing(&self) -> KResult<()> {
        let snap = {
            let s = self.state.borrow();
            (
                s.debug_batch_count,
                s.debug_batch_totals,
                s.debug_wake_count,
                s.debug_late_wake_count,
                s.debug_worst_wake_late,
                s.debug_expect_wake,
                s.debug_batch_pending.is_some(),
                s.debug_batch_pending.clone(),
                s.debug_failed_batch.clone(),
                s.debug_worst_batch.clone(),
                s.debug_slow_batches.clone(),
                s.debug_batches.clone(),
                s.debug_wake_events.clone(),
            )
        };
        let tot = snap.1;
        log::error(&fmt(
            "[DEBUG-motion] version=host-v4 motion timing batches=%d cb_total=%.6f gen_total=%.6f fin_total=%.6f time_total=%.6f wakes=%d late_wakes=%d worst_late_wake=%.6f expect_wake=%.6f pending=%s",
            vec![
                PctValue::Int(PyInt::from_i64(snap.0)),
                PctValue::Float(tot[0]),
                PctValue::Float(tot[1]),
                PctValue::Float(tot[2]),
                PctValue::Float(tot[3]),
                PctValue::Int(PyInt::from_i64(snap.2)),
                PctValue::Int(PyInt::from_i64(snap.3)),
                PctValue::Float(snap.4),
                PctValue::Float(snap.5),
                PctValue::Bool(snap.6),
            ],
        ));
        for (label, batch) in [("pending", &snap.7), ("failed", &snap.8), ("worst", &snap.9)] {
            if let Some(b) = batch {
                log::error(&fmt(
                    "[DEBUG-motion] motion batch %s %s",
                    vec![
                        PctValue::Str(label.to_string()),
                        PctValue::Str(b.borrow().desc()),
                    ],
                ));
            }
        }
        for b in snap.10.iter().rev().take(DEBUG_DUMP_ENTRIES).rev() {
            log::error(&fmt(
                "[DEBUG-motion] motion batch slow %s",
                vec![PctValue::Str(b.borrow().desc())],
            ));
        }
        for b in snap.11.iter().rev().take(DEBUG_DUMP_ENTRIES).rev() {
            log::error(&fmt(
                "[DEBUG-motion] motion batch recent %s",
                vec![PctValue::Str(b.borrow().desc())],
            ));
        }
        for ev in snap.12.iter().rev().take(DEBUG_DUMP_ENTRIES).rev() {
            log::error(&fmt(
                "[DEBUG-motion] motion wake at=%.6f late=%.6f dur=%.6f batches=%d ret_in=%.6f last_flush=%.9f last_step_gen=%.9f need_flush=%.9f need_step_gen=%.9f",
                vec![
                    PctValue::Float(ev.eventtime),
                    PctValue::Float(ev.wake_late),
                    PctValue::Float(ev.duration),
                    PctValue::Int(PyInt::from_i64(ev.nbatch)),
                    PctValue::Float(ev.ret_in),
                    PctValue::Float(ev.last_flush_time),
                    PctValue::Float(ev.last_step_gen_time),
                    PctValue::Float(ev.need_flush_time),
                    PctValue::Float(ev.need_step_gen_time),
                ],
            ));
        }
        for line in self.reactor.get_timing_debug_report(DEBUG_DUMP_ENTRIES) {
            log::error(&fmt("%s", vec![PctValue::Str(line)]));
        }
        Ok(())
    }

    /// `_debug_note_batch` (`motion_queuing.py:263`).
    pub fn _debug_note_batch(&self, batch: BatchShared) {
        self.state.with_mut(|s| {
            s.debug_batch_count += 1;
            let (cb, gen, fin, total, stage) = {
                let b = batch.borrow();
                (b.cb_time, b.gen_time, b.fin_time, b.total_time, b.stage.clone())
            };
            s.debug_batch_totals[0] += cb;
            s.debug_batch_totals[1] += gen;
            s.debug_batch_totals[2] += fin;
            s.debug_batch_totals[3] += total;
            push_bounded(&mut s.debug_batches, batch.clone(), DEBUG_RING_SIZE);
            let worst_is_stale = match &s.debug_worst_batch {
                None => true,
                Some(w) => total > w.borrow().total_time,
            };
            if worst_is_stale {
                s.debug_worst_batch = Some(batch.clone());
            }
            if stage != "idle" {
                // Batch did not complete - keep the failure timing.
                s.debug_failed_batch = Some(batch.clone());
                push_bounded(&mut s.debug_slow_batches, batch, DEBUG_SLOW_RING_SIZE);
            } else if total > DEBUG_SLOW_FLUSH_TIME {
                push_bounded(&mut s.debug_slow_batches, batch, DEBUG_SLOW_RING_SIZE);
            }
        });
    }

    // --- Flush tracking (`motion_queuing.py:280-506`) -----------------------

    /// `_advance_flush_time` (`motion_queuing.py:280`). Returns the raised
    /// `KlippyError` (a flush-callback / stepcompress failure); the batch
    /// record is finalized in the `finally` block either way.
    pub fn _advance_flush_time(&self, want_flush_time: f64, want_step_gen_time: f64) -> KResult<()> {
        let (flush_time, step_gen_time) = {
            let s = self.state.borrow();
            let flush_time = want_flush_time
                .max(s.last_flush_time)
                .max(want_step_gen_time - STEPCOMPRESS_FLUSH_TIME);
            let step_gen_time = want_step_gen_time
                .max(s.last_step_gen_time)
                .max(flush_time);
            (flush_time, step_gen_time)
        };
        // Debug batch record (host side only).
        let (seq, prev_flush, prev_step) = self.state.with_mut(|s| {
            s.debug_flush_time = flush_time;
            s.debug_step_gen_time = step_gen_time;
            s.debug_flush_stage = "callbacks".to_string();
            s.debug_batch_seq += 1;
            (s.debug_batch_seq, s.last_flush_time, s.last_step_gen_time)
        });
        let t_start = (self.clock)();
        let batch: BatchShared = Arc::new(GCell::new(DebugFlushBatch {
            seq,
            start: t_start,
            flush_time,
            step_gen_time,
            prev_flush_time: prev_flush,
            prev_step_gen_time: prev_step,
            cb_time: 0.0,
            gen_time: 0.0,
            fin_time: 0.0,
            total_time: 0.0,
            stage: "callbacks".to_string(),
            slow_cb_time: 0.0,
            slow_cb_name: "-".to_string(),
            cb_count: 0,
            trapq_count: 0,
        }));
        self.state.with_mut(|s| s.debug_batch_pending = Some(batch.clone()));
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self._advance_flush_time_body(&batch, flush_time, step_gen_time, t_start)));
        batch.with_mut(|b| b.total_time = (self.clock)() - t_start);
        self.state.with_mut(|s| s.debug_batch_pending = None);
        self._debug_note_batch(batch);
        match result {
            Ok(r) => r,
            Err(payload) => std::panic::resume_unwind(payload),
        }
    }

    fn _advance_flush_time_body(
        &self,
        batch: &BatchShared,
        flush_time: f64,
        step_gen_time: f64,
        t_start: f64,
    ) -> KResult<()> {
        // Invoke flush callbacks (if any).
        let mut slow_cb_name: Option<String> = None;
        let mut t_phase = t_start;
        {
            let callbacks = self.state.borrow().flush_callbacks.iter().map(|e| (e.cb.clone(), e.name.clone().unwrap_or_else(|| "-".to_string()))).collect::<Vec<_>>();
            let _no_pause = self.reactor.assert_no_pause();
            for (cb, name) in &callbacks {
                cb(flush_time, step_gen_time)?;
                let t_cb = (self.clock)();
                let cb_time = t_cb - t_phase;
                t_phase = t_cb;
                let slow_cb_time = batch.borrow().slow_cb_time;
                if cb_time > slow_cb_time {
                    batch.with_mut(|b| b.slow_cb_time = cb_time);
                    // `name` is already the resolved `debug_callback_name`
                    // stored at registration (`motion_queuing.py:313` reads
                    // the callback object; Rust resolves it once there).
                    slow_cb_name = Some(name.clone());
                }
            }
        }
        let cb_count = self.state.borrow().flush_callbacks.len() as i64;
        batch.with_mut(|b| {
            b.cb_count = cb_count;
            b.cb_time = t_phase - t_start;
        });
        if let Some(n) = slow_cb_name {
            batch.with_mut(|b| b.slow_cb_name = n);
        }
        // Determine maximum history to keep.
        let trapq_free_time = step_gen_time - self.get_kin_flush_delay();
        let mut clear_history_time = self.state.borrow().clear_history_time;
        if !self.state.borrow().can_pause {
            clear_history_time = (trapq_free_time - MOVE_HISTORY_EXPIRE).max(0.0);
        }
        // Generate stepper movement and transmit.
        self.state.with_mut(|s| s.debug_flush_stage = "generate_steps".to_string());
        batch.with_mut(|b| b.stage = "generate_steps".to_string());
        let ret = self.chelper.steppersyncmgr_gen_steps(
            &self.steppersyncmgr,
            flush_time,
            step_gen_time,
            clear_history_time,
        );
        let t_gen = (self.clock)();
        batch.with_mut(|b| b.gen_time = t_gen - t_phase);
        if let Some(ret) = ret {
            let msg = "Internal error in stepcompress";
            let queue_name = self.syncemitter_to_name(&ret);
            let mut details = IndexMap::new();
            details.insert(
                PyValue::Str("queue_name".to_string()),
                queue_name.map(PyValue::Str).unwrap_or(PyValue::None),
            );
            details.insert(PyValue::Str("flush_time".to_string()), PyValue::Float(flush_time));
            details.insert(PyValue::Str("step_gen_time".to_string()), PyValue::Float(step_gen_time));
            details.insert(
                PyValue::Str("last_flush_time".to_string()),
                PyValue::Float(self.state.borrow().last_flush_time),
            );
            details.insert(
                PyValue::Str("last_step_gen_time".to_string()),
                PyValue::Float(self.state.borrow().last_step_gen_time),
            );
            details.insert(
                PyValue::Str("clear_history_time".to_string()),
                PyValue::Float(clear_history_time),
            );
            self.printer.invoke_shutdown(msg, PyValue::Dict(details));
            // `raise self.mcu.error(msg)`.
            return Err(KlippyError::Mcu(msg.to_string()));
        }
        self.state.with_mut(|s| {
            s.last_flush_time = flush_time;
            s.last_step_gen_time = step_gen_time;
        });
        // Move processed trapq entries to history, and expire old history.
        self.state.with_mut(|s| s.debug_flush_stage = "finalize_trapqs".to_string());
        batch.with_mut(|b| b.stage = "finalize_trapqs".to_string());
        let trapqs = self.state.borrow().trapqs.clone();
        for trapq in &trapqs {
            self.chelper.trapq_finalize_moves(trapq, trapq_free_time, clear_history_time);
        }
        batch.with_mut(|b| b.trapq_count = trapqs.len() as i64);
        batch.with_mut(|b| b.fin_time = (self.clock)() - t_gen);
        batch.with_mut(|b| b.stage = "idle".to_string());
        self.state.with_mut(|s| s.debug_flush_stage = "idle".to_string());
        Ok(())
    }

    /// `_await_flush_time` (`motion_queuing.py:355`).
    pub fn _await_flush_time(&self, want_flush_time: f64) {
        loop {
            if self.last_flush_time() >= want_flush_time || !self.state.borrow().can_pause {
                return;
            }
            let systime = self.reactor.monotonic();
            let est_print_time = self.mcu.estimated_print_time(systime);
            let wait = want_flush_time - BGFLUSH_HIGH_TIME - est_print_time;
            if wait <= 0.0 {
                return;
            }
            self.reactor.pause(systime + 1.0f64.min(wait));
        }
    }

    /// `flush_all_steps` (`motion_queuing.py:365`).
    pub fn flush_all_steps(&self) -> KResult<()> {
        let flush_time = self.need_step_gen_time();
        self._await_flush_time(flush_time);
        self._advance_flush_time(flush_time, 0.0)
    }

    /// `calc_step_gen_restart` (`motion_queuing.py:369`).
    pub fn calc_step_gen_restart(&self, est_print_time: f64) -> f64 {
        let kin_time = (est_print_time + MIN_KIN_TIME).max(self.last_step_gen_time());
        kin_time + self.get_kin_flush_delay()
    }

    /// `_flush_handler` (`motion_queuing.py:372`).
    pub fn _flush_handler(&self, eventtime: f64) -> f64 {
        let t_enter = (self.clock)();
        let expect = self.debug_expect_wake();
        let wake_late = if expect != 0.0 { eventtime - expect } else { 0.0 };
        let seq = self.debug_batch_seq();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self._flush_handler_body(eventtime)
        }));
        let ret = match result {
            Ok(r) => r,
            Err(payload) => {
                // Python: the `finally` records the wake block even while
                // the exception propagates. `_flush_handler_body` catches
                // everything itself and returns NEVER, so unwinding here
                // would be a Rust-internal panic; resume after recording.
                self._debug_note_wake(
                    eventtime,
                    (self.clock)() - t_enter,
                    wake_late,
                    self.debug_batch_seq() - seq,
                    NEVER,
                );
                std::panic::resume_unwind(payload);
            }
        };
        self._debug_note_wake(
            eventtime,
            (self.clock)() - t_enter,
            wake_late,
            self.debug_batch_seq() - seq,
            ret,
        );
        ret
    }

    /// `_debug_note_wake` (`motion_queuing.py:389`).
    pub fn _debug_note_wake(
        &self,
        eventtime: f64,
        duration: f64,
        wake_late: f64,
        nbatch: i64,
        ret: f64,
    ) {
        self.state.with_mut(|s| {
            s.debug_wake_count += 1;
            if wake_late > DEBUG_LATE_WAKE_TIME {
                s.debug_late_wake_count += 1;
                if wake_late > s.debug_worst_wake_late {
                    s.debug_worst_wake_late = wake_late;
                }
            }
            let ret_in: f64;
            if ret >= NEVER {
                s.debug_expect_wake = 0.0;
                ret_in = -1.0;
            } else {
                s.debug_expect_wake = ret;
                ret_in = ret - eventtime;
            }
            let ev = WakeEvent {
                eventtime,
                wake_late,
                duration,
                nbatch,
                ret_in,
                last_flush_time: s.last_flush_time,
                last_step_gen_time: s.last_step_gen_time,
                need_flush_time: s.need_flush_time,
                need_step_gen_time: s.need_step_gen_time,
            };
            push_bounded(&mut s.debug_wake_events, ev, DEBUG_WAKE_RING_SIZE);
        });
    }

    /// `_flush_handler_body` (`motion_queuing.py:405`).
    pub fn _flush_handler_body(&self, eventtime: f64) -> f64 {
        match self._flush_handler_body_inner(eventtime) {
            Ok(ret) => ret,
            Err(e) => {
                log::exception(
                    "Exception in flush_handler",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                self.printer.invoke_shutdown("Exception in flush_handler", PyValue::Dict(IndexMap::new()));
                NEVER
            }
        }
    }

    fn _flush_handler_body_inner(&self, eventtime: f64) -> KResult<f64> {
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        // Reschedule timer
        let aggr_sg_time = self.need_step_gen_time() - 2.0 * self.get_kin_flush_delay();
        if self.last_step_gen_time() < aggr_sg_time {
            // Actively stepping - want more aggressive flushing
            let mut want_sg_time = est_print_time + BGFLUSH_SG_HIGH_TIME;
            let batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME;
            let next_batch_time = self.last_step_gen_time() + batch_time;
            if next_batch_time > est_print_time {
                // Improve run-to-run reproducibility by batching from last
                let mut next_batch_time = next_batch_time;
                if next_batch_time > want_sg_time + 0.005 {
                    // Delay flushing until next wakeup
                    next_batch_time = self.last_step_gen_time();
                }
                want_sg_time = next_batch_time;
            }
            want_sg_time = want_sg_time.min(aggr_sg_time);
            // Flush motion queues (if needed)
            if want_sg_time > self.last_step_gen_time() {
                self._advance_flush_time(0.0, want_sg_time)?;
            }
        } else {
            // Not stepping (or only step remnants) - use relaxed flushing
            let mut want_flush_time = est_print_time + BGFLUSH_HIGH_TIME;
            let max_flush_time = self.need_flush_time() + BGFLUSH_EXTRA_TIME;
            want_flush_time = want_flush_time.min(max_flush_time);
            // Flush motion queues (if needed)
            if want_flush_time > self.last_flush_time() {
                self._advance_flush_time(want_flush_time, 0.0)?;
            }
        }
        let aggr_sg_time = self.need_step_gen_time() - 2.0 * self.get_kin_flush_delay();
        let waketime = if self.last_step_gen_time() < aggr_sg_time {
            self.last_step_gen_time() - BGFLUSH_SG_LOW_TIME
        } else {
            self.state.with_mut(|s| s.do_kick_flush_timer = true);
            let max_flush_time = self.need_flush_time() + BGFLUSH_EXTRA_TIME;
            if self.last_flush_time() >= max_flush_time {
                return Ok(NEVER);
            }
            self.last_flush_time() - BGFLUSH_LOW_TIME
        };
        Ok(eventtime + waketime - est_print_time)
    }

    /// `_flush_handler_debug` (`motion_queuing.py:447`).
    pub fn _flush_handler_debug(&self, _eventtime: f64) -> f64 {
        match self._flush_handler_debug_inner() {
            Ok(ret) => ret,
            Err(e) => {
                log::exception(
                    "Exception in flush_handler_debug",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                self.printer.invoke_shutdown("Exception in flush_handler_debug", PyValue::Dict(IndexMap::new()));
                NEVER
            }
        }
    }

    fn _flush_handler_debug_inner(&self) -> KResult<f64> {
        let faux_time = self.need_flush_time() - 1.5;
        let batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME;
        let mut flush_count: i64 = 0;
        while self.last_step_gen_time() < faux_time {
            let mut target = self.last_step_gen_time() + batch_time;
            if flush_count > 100 && faux_time > target {
                // `int((faux_time-target) / batch_time) * batch_time`: Python
                // `int()` truncates toward zero.
                let scaled = (faux_time - target) / batch_time;
                let int_scaled = pycompat::text::trunc_float_to_bigint(scaled);
                let int_scaled_f = int_scaled.to_f64_exact().unwrap_or(0.0);
                target += int_scaled_f * batch_time;
            }
            self._advance_flush_time(0.0, target)?;
            flush_count += 1;
        }
        if flush_count != 0 {
            return Ok(NOW);
        }
        self._advance_flush_time(self.need_flush_time() + BGFLUSH_EXTRA_TIME, 0.0)?;
        self.state.with_mut(|s| s.do_kick_flush_timer = true);
        Ok(NEVER)
    }

    /// `note_mcu_movequeue_activity` (`motion_queuing.py:468`).
    pub fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        let mut mq_time = mq_time;
        if is_step_gen {
            mq_time += self.get_kin_flush_delay();
            self.state
                .with_mut(|s| s.need_step_gen_time = s.need_step_gen_time.max(mq_time));
        }
        self.state
            .with_mut(|s| s.need_flush_time = s.need_flush_time.max(mq_time));
        if self.do_kick_flush_timer() {
            self.state.with_mut(|s| s.do_kick_flush_timer = false);
            // `self.reactor.update_timer(self.flush_timer, self.reactor.NOW)`.
            self.state.with_mut(|s| {
                if let Some(timer) = s.flush_timer.as_ref() {
                    self.reactor.update_timer(timer, NOW);
                }
            });
        }
    }

    /// `drip_update_time` (`motion_queuing.py:477`).
    pub fn drip_update_time(
        &self,
        start_time: f64,
        end_time: f64,
        drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        self.state.with_mut(|s| s.drip_start_times.push(start_time));
        self._await_flush_time(start_time);
        // Disable background flushing from timer
        self.state.with_mut(|s| {
            if let Some(timer) = s.flush_timer.as_ref() {
                self.reactor.update_timer(timer, NEVER);
            }
        });
        self.state.with_mut(|s| s.do_kick_flush_timer = false);
        self._advance_flush_time(start_time - SDS_CHECK_TIME, start_time)?;
        // Flush in segments until drip_completion signal
        let mut flush_time = start_time;
        while flush_time < end_time {
            if drip_completion.test() {
                break;
            }
            let curtime = self.reactor.monotonic();
            let est_print_time = self.mcu.estimated_print_time(curtime);
            let wait_time = flush_time - est_print_time - DRIP_TIME;
            if wait_time > 0.0 && self.state.borrow().can_pause {
                // Pause before sending more steps
                let _ = drip_completion.wait(curtime + wait_time);
                continue;
            }
            flush_time = (flush_time + DRIP_SEGMENT_TIME).min(end_time);
            self.note_mcu_movequeue_activity(flush_time, true);
            self._advance_flush_time(flush_time - SDS_CHECK_TIME, flush_time)?;
        }
        // Restore background flushing
        self.state.with_mut(|s| {
            if let Some(timer) = s.flush_timer.as_ref() {
                self.reactor.update_timer(timer, NOW);
            }
        });
        self._advance_flush_time(flush_time + self.get_kin_flush_delay(), 0.0)?;
        // `self.drip_start_times.remove(start_time)` (first occurrence).
        self.state.with_mut(|s| {
            if let Some(pos) = s
                .drip_start_times
                .iter()
                .position(|&t| t == start_time)
            {
                s.drip_start_times.remove(pos);
            }
        });
        Ok(())
    }

    /// `check_drip_timing` (`motion_queuing.py:503`).
    pub fn check_drip_timing(&self) -> Option<f64> {
        let s = self.state.borrow();
        if s.drip_start_times.is_empty() {
            None
        } else {
            s.drip_start_times.iter().cloned().fold(None, |acc: Option<f64>, t| {
                Some(acc.map_or(t, |a: f64| a.min(t)))
            })
        }
    }
}

impl PrinterObject for PrinterMotionQueuing {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn MotionQueuingApi);

    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        PrinterMotionQueuing::stats(self, eventtime).ok()
    }
}

impl MotionQueuingApi for PrinterMotionQueuing {
    fn allocate_trapq(&self) -> Trapq {
        PrinterMotionQueuing::allocate_trapq(self)
    }

    fn lookup_trapq_append(&self) -> TrapqAppend {
        PrinterMotionQueuing::lookup_trapq_append(self)
    }

    fn wipe_trapq(&self, trapq: &Trapq) {
        PrinterMotionQueuing::wipe_trapq(self, trapq)
    }

    fn allocate_syncemitter(
        &self,
        mcu: Mcu,
        name: &str,
        alloc_stepcompress: bool,
    ) -> KResult<SyncEmitterHandle> {
        PrinterMotionQueuing::allocate_syncemitter(self, mcu, name, alloc_stepcompress)
    }

    fn queue_syncemitter_msg(&self, syncemitter: &SyncEmitterHandle, req_clock: u64, data: &[u32]) {
        PrinterMotionQueuing::queue_syncemitter_msg(self, syncemitter, req_clock, data)
    }

    fn setup_mcu_movequeue(&self, mcu: Mcu, serialqueue: Option<std::sync::Arc<SerialQueue>>, move_count: i64) {
        PrinterMotionQueuing::setup_mcu_movequeue(self, mcu, serialqueue, move_count)
    }

    fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool) {
        PrinterMotionQueuing::register_flush_callback(self, callback, can_add_trapq)
    }

    fn register_flush_callback_named(
        &self,
        callback: FlushCallback,
        can_add_trapq: bool,
        name: Option<&str>,
        obj_name: Option<&str>,
    ) {
        PrinterMotionQueuing::register_flush_callback_named(self, callback, can_add_trapq, name, obj_name)
    }

    fn unregister_flush_callback(&self, callback: &FlushCallback) {
        PrinterMotionQueuing::unregister_flush_callback(self, callback)
    }

    fn get_kin_flush_delay(&self) -> f64 {
        PrinterMotionQueuing::get_kin_flush_delay(self)
    }

    fn check_step_generation_scan_windows(&self) {
        PrinterMotionQueuing::check_step_generation_scan_windows(self)
    }

    fn flush_all_steps(&self) -> KResult<()> {
        PrinterMotionQueuing::flush_all_steps(self)
    }

    fn calc_step_gen_restart(&self, est_print_time: f64) -> f64 {
        PrinterMotionQueuing::calc_step_gen_restart(self, est_print_time)
    }

    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        PrinterMotionQueuing::note_mcu_movequeue_activity(self, mq_time, is_step_gen)
    }

    fn drip_update_time(
        &self,
        start_time: f64,
        end_time: f64,
        drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        PrinterMotionQueuing::drip_update_time(self, start_time, end_time, drip_completion)
    }

    fn check_drip_timing(&self) -> Option<f64> {
        PrinterMotionQueuing::check_drip_timing(self)
    }
}

/// `load_config` (`motion_queuing.py:508`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let this = PrinterMotionQueuing::from_config(config)?;
    Ok(Some(this))
}

/// The printer module entry points (`load_config` at
/// `motion_queuing.py:508`; the Python module has no other factory).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
