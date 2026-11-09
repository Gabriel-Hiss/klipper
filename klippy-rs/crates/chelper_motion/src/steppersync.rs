//! Port of `klippy/chelper/steppersync.c` / `steppersync.h`.
//!
//! The mcu can only queue a limited number of step commands. This unit
//! tracks when the slots of the mcu move queue become free so new commands
//! can be transmitted, and it orders the commands of every stepper of one
//! mcu by their requested clock so no stepper starves the others of queue
//! space.
//!
//! Object model (the C intrusive lists become owned vectors of `Arc`s):
//!
//! - [`SyncEmitter`] (`struct syncemitter`): the message queue of one
//!   stepper (or of a non-stepper user such as `pwm_tool`). When it owns a
//!   [`Stepcompress`] it also owns a background step-generation thread,
//!   exactly like the C `se_background_thread`.
//! - [`SteppersSync`] (`struct steppersync`): the emitters of one mcu, its
//!   serial queue and command queue, the print-time to clock conversion and
//!   the binary heap of move-queue slot release clocks.
//! - [`SteppersSyncMgr`] (`struct steppersyncmgr`): every
//!   [`SteppersSync`], driven by [`SteppersSyncMgr::gen_steps`].
//!
//! In the C, `stepcompress_alloc` receives the emitter's `msg_queue` and
//! links its `queue_step` / `set_next_step_dir` messages straight into it.
//! The Rust [`Stepcompress`] buffers its messages instead
//! ([`Stepcompress::drain_messages`]). Every insertion into an emitter
//! queue ([`SyncEmitter::queue_msg`]) and every move-queue flush first moves
//! the compressor's pending messages to the tail of the emitter queue, so
//! the queue holds the messages in the same order as the C list.
//!
//! All threads, mutexes and condition variables go through `kos`, so the
//! virtual-clock shim (`parity/vclock`) sees the same thread structure as
//! the C. Log lines go through `chelper_serial`'s `errorf` (the C
//! `pyhelper.c` callback).
//!
//! Differences forced by the Rust building blocks, none reachable from
//! `klippy`:
//!
//! - `kos` panics when `pthread_create`, `pthread_mutex_init` or
//!   `pthread_cond_init` fails. The C logs `se alloc` through
//!   `report_errno` and returns NULL. A `pthread_join` failure is not
//!   reported (`kos::thread::JoinHandle::join` returns nothing); the C
//!   logs `se pthread_join`.
//! - The C dereferences a NULL `move_clocks` / `sq` when messages are
//!   flushed before [`SteppersSync::setup_movequeue`], and overflows a
//!   zero-sized heap when `move_num` is 0. The port leaves the messages
//!   queued when no serial queue is configured, and treats an empty heap
//!   as "slot available at clock 0" (nothing is stored).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::Arc;

use chelper_serial::msgblock::{clock_fill, message_alloc_and_encode, message_fill};
use chelper_serial::serialqueue::CommandQueue;
use chelper_serial::{
    alloc_commandqueue, clock_from_time, errorf, free_commandqueue, ClockEstimate, QueueMessage,
    SerialQueue, MAX_CLOCK,
};
use kos::sync::{Condvar, Mutex, MutexGuard};
use kos::thread::{self, JoinHandle};

use crate::itersolve::StepperKinematics;
use crate::stepcompress::{SharedStepcompress, Stepcompress};

/// Pending messages of one emitter, oldest first (`se->msg_queue`).
type MsgQueue = VecDeque<Box<QueueMessage>>;

/****************************************************************
 * SyncEmitter - message generation for each stepper
 ****************************************************************/

/// A `*mut StepperKinematics` that may cross to the step-generation
/// thread. The thread only dereferences it while the main thread waits in
/// [`SteppersSyncMgr::gen_steps`] (the C contract).
#[derive(Clone, Copy)]
struct SkPtr(*mut StepperKinematics);

// SAFETY: the pointer is only dereferenced by the step-generation thread
// between `se_start_gen_steps` and `se_finalize_gen_steps`, while the
// caller of `SyncEmitter::set_stepper_kinematics` guarantees exclusive
// access (see its safety contract).
unsafe impl Send for SkPtr {}

/// Variables protected by `se->lock`.
struct WorkState {
    /// `have_work`: 0 idle, 1 request pending, -1 exit request.
    have_work: i32,
    /// Kinematics of the request (`se->sk` at start time).
    sk: SkPtr,
    bg_gen_steps_time: f64,
    bg_flush_clock: u64,
    bg_clear_history_clock: u64,
    bg_result: i32,
}

/// State shared between an emitter and its background thread
/// (`se->lock`, `se->cond` and the variables they protect).
struct WorkerShared {
    state: Mutex<WorkState>,
    cond: Condvar,
}

/// The background step-generation thread of an emitter with a
/// compressor.
struct Worker {
    shared: Arc<WorkerShared>,
    /// `se->tid`; `None` once the thread was joined.
    handle: Mutex<Option<JoinHandle>>,
}

/// Port of `struct syncemitter`: the transmit queue of one stepper or
/// other move-queue user, plus its optional step compressor and
/// background step-generation thread.
pub struct SyncEmitter {
    /// `se->name`: at most 15 bytes, up to the first NUL.
    name: String,
    /// `se->sc` (`None` when allocated without a compressor).
    sc: Option<SharedStepcompress>,
    /// `se->sk` (null when unset).
    sk: AtomicPtr<StepperKinematics>,
    /// `se->msg_queue`.
    msg_queue: Mutex<MsgQueue>,
    /// Present exactly when `sc` is.
    worker: Option<Worker>,
}

impl SyncEmitter {
    /// Port of `syncemitter_alloc`: copy the name like `strncpy` into
    /// `char name[16]` followed by `name[15] = '\0'`, and with
    /// `alloc_stepcompress` allocate the compressor and start the
    /// background thread.
    fn alloc(name: &[u8], alloc_stepcompress: bool) -> SyncEmitter {
        let max = name.len().min(15);
        let end = name[..max].iter().position(|&b| b == 0).unwrap_or(max);
        let mut se = SyncEmitter {
            name: String::from_utf8_lossy(&name[..end]).into_owned(),
            sc: None,
            sk: AtomicPtr::new(std::ptr::null_mut()),
            msg_queue: Mutex::new(VecDeque::new()),
            worker: None,
        };
        if !alloc_stepcompress {
            return se;
        }
        let sc: SharedStepcompress = Arc::new(Mutex::new(Stepcompress::new()));
        let shared = Arc::new(WorkerShared {
            state: Mutex::new(WorkState {
                have_work: 0,
                sk: SkPtr(std::ptr::null_mut()),
                bg_gen_steps_time: 0.0,
                bg_flush_clock: 0,
                bg_clear_history_clock: 0,
                bg_result: 0,
            }),
            cond: Condvar::new(),
        });
        let handle = {
            let shared = Arc::clone(&shared);
            let sc = Arc::clone(&sc);
            let name = se.name.clone();
            thread::spawn_named(&se.name, move || background_thread(&shared, &sc, &name))
        };
        se.sc = Some(sc);
        se.worker = Some(Worker { shared, handle: Mutex::new(Some(handle)) });
        se
    }

    /// Port of `syncemitter_get_stepcompress`: this emitter's compressor
    /// (`None` if allocated without one). Release its lock before calling
    /// [`SyncEmitter::queue_msg`] or [`SteppersSyncMgr::gen_steps`], which
    /// lock it too.
    pub fn get_stepcompress(&self) -> Option<SharedStepcompress> {
        self.sc.clone()
    }

    /// Port of `syncemitter_set_stepper_kinematics`: store a reference to
    /// the stepper kinematics (null clears it).
    ///
    /// # Safety
    /// A non-null `sk` must stay valid until it is replaced or the emitter
    /// is dropped, and must not be accessed by anyone else while
    /// [`SteppersSyncMgr::gen_steps`] runs (its step-generation thread
    /// mutates it), as in the C.
    pub unsafe fn set_stepper_kinematics(&self, sk: *mut StepperKinematics) {
        self.sk.store(sk, Ordering::SeqCst);
    }

    /// Port of `syncemitter_get_stepper_kinematics` (null when unset).
    pub fn get_stepper_kinematics(&self) -> *mut StepperKinematics {
        self.sk.load(Ordering::SeqCst)
    }

    /// Port of `syncemitter_queue_msg`: queue an mcu command that consumes
    /// space in the mcu move queue (`min_clock = req_clock`; a
    /// `req_clock` of 0 marks a command that does not use the move queue).
    /// An encoding overflow logs `Encode error` and queues an empty
    /// message, as `message_alloc_and_encode` does.
    pub fn queue_msg(&self, req_clock: u64, data: &[u32]) {
        let mut qm = message_alloc_and_encode(data);
        qm.min_clock = req_clock;
        qm.req_clock = req_clock;
        let mut queue = self.msg_queue.lock();
        self.collect_stepcompress_msgs(&mut queue);
        queue.push_back(qm);
    }

    /// Move the compressor's pending messages to the tail of `queue`
    /// (where the C `stepcompress` links them directly).
    fn collect_stepcompress_msgs(&self, queue: &mut MsgQueue) {
        let Some(sc) = &self.sc else {
            return;
        };
        for m in sc.lock().drain_messages() {
            let mut qm = message_fill(&m.bytes, m.bytes.len() as i32);
            qm.min_clock = m.min_clock;
            qm.req_clock = m.req_clock;
            queue.push_back(qm);
        }
    }

    /// Port of `se_start_gen_steps`: signal the background thread to start
    /// step generation.
    fn start_gen_steps(&self, gen_steps_time: f64, flush_clock: u64, clear_history_clock: u64) {
        let sk = self.get_stepper_kinematics();
        let Some(worker) = &self.worker else {
            return;
        };
        if sk.is_null() {
            return;
        }
        let shared = &worker.shared;
        {
            let mut st = shared.state.lock();
            while st.have_work != 0 {
                st = shared.cond.wait(st);
            }
            st.sk = SkPtr(sk);
            st.bg_gen_steps_time = gen_steps_time;
            st.bg_flush_clock = flush_clock;
            st.bg_clear_history_clock = clear_history_clock;
            st.have_work = 1;
        }
        shared.cond.notify_one();
    }

    /// Port of `se_finalize_gen_steps`: wait for the background thread to
    /// complete the last step generation request and return its result.
    fn finalize_gen_steps(&self) -> i32 {
        let Some(worker) = &self.worker else {
            return 0;
        };
        if self.get_stepper_kinematics().is_null() {
            return 0;
        }
        let shared = &worker.shared;
        let mut st = shared.state.lock();
        while st.have_work != 0 {
            st = shared.cond.wait(st);
        }
        st.bg_result
    }

    /// The thread-exit part of `syncemitter_free`: wait for pending work,
    /// request exit and join the thread. Idempotent.
    fn stop_worker(&self) {
        let Some(worker) = &self.worker else {
            return;
        };
        let Some(handle) = worker.handle.lock().take() else {
            return;
        };
        let shared = &worker.shared;
        {
            let mut st = shared.state.lock();
            while st.have_work != 0 {
                st = shared.cond.wait(st);
            }
            st.have_work = -1;
            shared.cond.notify_one();
        }
        handle.join();
    }
}

impl Drop for SyncEmitter {
    /// Port of `syncemitter_free`: exit the background thread; the
    /// compressor and any remaining messages are released with `self`.
    fn drop(&mut self) {
        self.stop_worker();
    }
}

/// Port of `se_generate_steps`: generate steps (via itersolve), flush
/// them and expire the history.
fn generate_steps(sc: &SharedStepcompress, st: &WorkState) -> i32 {
    if st.sk.0.is_null() {
        return 0;
    }
    let mut sc = sc.lock();
    // SAFETY: `SyncEmitter::set_stepper_kinematics` guarantees the
    // pointer is valid and exclusively ours while generation runs.
    let sk = unsafe { &mut *st.sk.0 };
    // Generate steps
    let ret = sk.generate_steps(&mut sc, st.bg_gen_steps_time);
    if ret != 0 {
        return ret;
    }
    // Flush steps
    let ret = sc.flush(st.bg_flush_clock);
    if ret != 0 {
        return ret;
    }
    // Clear history
    sc.history_expire(st.bg_clear_history_clock);
    0
}

/// Port of `se_background_thread`: the main loop of the step-generation
/// thread (named by `kos::thread::spawn_named`, the C `set_thread_name`).
fn background_thread(shared: &WorkerShared, sc: &SharedStepcompress, name: &str) {
    let mut st = shared.state.lock();
    loop {
        if st.have_work == 0 {
            st = shared.cond.wait(st);
            continue;
        }
        if st.have_work < 0 {
            // Exit request
            break;
        }
        // Request to generate steps
        let result = generate_steps(sc, &st);
        st.bg_result = result;
        if result != 0 {
            errorf(&format!("Error in syncemitter '{name}' step generation"));
        }
        st.have_work = 0;
        shared.cond.notify_one();
    }
}

/****************************************************************
 * StepperSync - sort move queue for a micro-controller
 ****************************************************************/

/// Fields of `struct steppersync`.
struct SsState {
    /// `ss->sq`.
    sq: Option<Arc<SerialQueue>>,
    /// `ss->cq`.
    cq: Option<Arc<CommandQueue>>,
    /// `ss->se_list`, in allocation order.
    emitters: Vec<Arc<SyncEmitter>>,
    /// `ss->ce`: converts print time to clock.
    ce: ClockEstimate,
    /// `ss->move_clocks`: a binary min-heap of the clocks at which each
    /// mcu move-queue slot becomes available (`num_move_clocks` entries).
    move_clocks: Vec<u64>,
}

/// Port of `struct steppersync`: the emitters of one mcu and the
/// bookkeeping of its move queue.
pub struct SteppersSync {
    state: Mutex<SsState>,
}

impl SteppersSync {
    fn new() -> SteppersSync {
        SteppersSync {
            state: Mutex::new(SsState {
                sq: None,
                cq: None,
                emitters: Vec::new(),
                ce: ClockEstimate::default(),
                move_clocks: Vec::new(),
            }),
        }
    }

    /// Port of `steppersync_alloc_syncemitter`: allocate a new emitter on
    /// this mcu. `name` follows the C `char name[16]` (at most 15 bytes,
    /// up to the first NUL); it names the background thread and the step
    /// generation error line. `alloc_stepcompress` also allocates a
    /// [`Stepcompress`] and the background step-generation thread.
    pub fn alloc_syncemitter(&self, name: &[u8], alloc_stepcompress: bool) -> Arc<SyncEmitter> {
        let se = Arc::new(SyncEmitter::alloc(name, alloc_stepcompress));
        self.state.lock().emitters.push(Arc::clone(&se));
        se
    }

    /// Port of `steppersync_setup_movequeue`: transmit through `sq` on a
    /// newly allocated command queue, with `move_num` mcu move-queue
    /// slots, all available at clock 0. A previous command queue is freed
    /// with `serialqueue_free_commandqueue` (which logs
    /// `Memory leak! Can't free non-empty commandqueue` if it still holds
    /// messages).
    pub fn setup_movequeue(&self, sq: Arc<SerialQueue>, move_num: usize) {
        let mut st = self.state.lock();
        if let Some(cq) = st.cq.take() {
            free_commandqueue(cq);
        }
        st.sq = Some(sq);
        st.cq = Some(alloc_commandqueue());
        st.move_clocks = vec![0; move_num];
    }

    /// Port of `steppersync_set_time`: set the conversion rate of
    /// `print_time` to mcu clock, here and in every emitter's compressor.
    pub fn set_time(&self, time_offset: f64, mcu_freq: f64) {
        let mut st = self.state.lock();
        clock_fill(&mut st.ce, mcu_freq, time_offset, 0);
        for se in &st.emitters {
            if let Some(sc) = &se.sc {
                sc.lock().set_time(time_offset, mcu_freq);
            }
        }
    }

    /// The command queue messages are sent on (the C `ss->cq`), `None`
    /// before [`SteppersSync::setup_movequeue`].
    pub fn command_queue(&self) -> Option<Arc<CommandQueue>> {
        self.state.lock().cq.clone()
    }
}

impl SsState {
    /// Port of `heap_replace`: replace the earliest slot release clock
    /// with `req_clock` and restore the heap order.
    fn heap_replace(&mut self, req_clock: u64) {
        let mc = &mut self.move_clocks;
        let nmc = mc.len();
        if nmc == 0 {
            return;
        }
        let mut pos = 0;
        loop {
            let child1_pos = 2 * pos + 1;
            let child2_pos = 2 * pos + 2;
            let child2_clock = if child2_pos < nmc { mc[child2_pos] } else { u64::MAX };
            let child1_clock = if child1_pos < nmc { mc[child1_pos] } else { u64::MAX };
            if req_clock <= child1_clock && req_clock <= child2_clock {
                mc[pos] = req_clock;
                break;
            }
            if child1_clock < child2_clock {
                mc[pos] = child1_clock;
                pos = child1_pos;
            } else {
                mc[pos] = child2_clock;
                pos = child2_pos;
            }
        }
    }

    /// Port of `steppersync_flush`: find and transmit any scheduled
    /// commands prior to `move_clock`, ordered by `req_clock` across the
    /// emitters (the first emitter wins ties).
    fn flush(&mut self, move_clock: u64) {
        let (Some(sq), Some(cq)) = (self.sq.clone(), self.cq.clone()) else {
            return;
        };
        let emitters = std::mem::take(&mut self.emitters);
        let mut queues: Vec<MutexGuard<'_, MsgQueue>> = emitters
            .iter()
            .map(|se| {
                let mut queue = se.msg_queue.lock();
                se.collect_stepcompress_msgs(&mut queue);
                queue
            })
            .collect();
        // Order commands by the reqclock of each pending command
        let mut msgs: Vec<Box<QueueMessage>> = Vec::new();
        loop {
            // Find message with lowest reqclock
            let mut req_clock = MAX_CLOCK;
            let mut found: Option<usize> = None;
            for (i, queue) in queues.iter().enumerate() {
                if let Some(m) = queue.front() {
                    if m.req_clock < req_clock {
                        found = Some(i);
                        req_clock = m.req_clock;
                    }
                }
            }
            let Some(i) = found else {
                break;
            };
            let min_clock = queues[i].front().map_or(0, |m| m.min_clock);
            if min_clock != 0 && req_clock > move_clock {
                break;
            }
            let Some(mut qm) = queues[i].pop_front() else {
                break;
            };

            let next_avail = self.move_clocks.first().copied().unwrap_or(0);
            if min_clock != 0 {
                // The min_clock field is overloaded to indicate that the
                // command uses the 'move queue' and to store the time that
                // move queue item becomes available.
                self.heap_replace(min_clock);
            }
            // Reset the min_clock to its normal meaning (minimum transmit time)
            qm.min_clock = next_avail;

            // Batch this command
            msgs.push(qm);
        }
        drop(queues);
        self.emitters = emitters;

        // Transmit commands
        if !msgs.is_empty() {
            sq.send_batch(&cq, msgs);
        }
    }
}

/****************************************************************
 * StepperSyncMgr - manage a list of steppersync
 ****************************************************************/

/// Port of `struct steppersyncmgr`: every [`SteppersSync`], in allocation
/// order. Dropping it is `steppersyncmgr_free`.
pub struct SteppersSyncMgr {
    ss_list: Mutex<Vec<Arc<SteppersSync>>>,
}

impl SteppersSyncMgr {
    /// Port of `steppersyncmgr_alloc`.
    pub fn alloc() -> SteppersSyncMgr {
        SteppersSyncMgr { ss_list: Mutex::new(Vec::new()) }
    }

    /// Port of `steppersyncmgr_alloc_steppersync`: a new, unconfigured
    /// [`SteppersSync`].
    pub fn alloc_steppersync(&self) -> Arc<SteppersSync> {
        let ss = Arc::new(SteppersSync::new());
        self.ss_list.lock().push(Arc::clone(&ss));
        ss
    }

    /// Port of `steppersyncmgr_gen_steps`: generate steps up to
    /// `gen_steps_time` on every emitter's background thread (in
    /// parallel), flush them up to `flush_time`, expire the compressor
    /// history before `clear_history_time`, then transmit each mcu's
    /// commands up to `flush_time`.
    ///
    /// Returns the last emitter (in allocation order) whose step
    /// generation failed, or `None`. Once an emitter has failed, the move
    /// queue of its mcu and of every later mcu is not flushed, as in the
    /// C.
    pub fn gen_steps(
        &self,
        flush_time: f64,
        gen_steps_time: f64,
        clear_history_time: f64,
    ) -> Option<Arc<SyncEmitter>> {
        let ss_list = self.ss_list.lock();
        // Prepare trapqs for step generation
        for ss in ss_list.iter() {
            let st = ss.state.lock();
            for se in &st.emitters {
                let sk = se.get_stepper_kinematics();
                if se.sc.is_none() || sk.is_null() {
                    continue;
                }
                // SAFETY: see `SyncEmitter::set_stepper_kinematics`.
                if let Some(tq) = unsafe { (*sk).get_trapq() } {
                    tq.lock().check_sentinels();
                }
            }
        }
        // Start step generation threads
        for ss in ss_list.iter() {
            let st = ss.state.lock();
            let flush_clock = clock_from_time(&st.ce, flush_time);
            let clear_clock = clock_from_time(&st.ce, clear_history_time);
            for se in &st.emitters {
                se.start_gen_steps(gen_steps_time, flush_clock, clear_clock);
            }
        }
        // Wait for step generation threads to complete
        let mut failed_se: Option<Arc<SyncEmitter>> = None;
        for ss in ss_list.iter() {
            let mut st = ss.state.lock();
            for se in &st.emitters {
                if se.finalize_gen_steps() != 0 {
                    failed_se = Some(Arc::clone(se));
                }
            }
            if failed_se.is_some() {
                continue;
            }
            let flush_clock = clock_from_time(&st.ce, flush_time);
            st.flush(flush_clock);
        }
        failed_se
    }
}

impl Drop for SteppersSyncMgr {
    /// Port of `steppersyncmgr_free`: per mcu in allocation order, free
    /// the command queue (logging a leak if it is not empty), then stop
    /// every emitter's background thread.
    fn drop(&mut self) {
        let ss_list = std::mem::take(&mut *self.ss_list.lock());
        for ss in ss_list {
            let mut st = ss.state.lock();
            st.move_clocks = Vec::new();
            if let Some(cq) = st.cq.take() {
                free_commandqueue(cq);
            }
            for se in std::mem::take(&mut st.emitters) {
                se.stop_worker();
            }
        }
    }
}
