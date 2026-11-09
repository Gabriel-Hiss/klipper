//! Differential parity tests for the `steppersync` unit.
//!
//! Every scenario is a list of operations applied in lockstep to the frozen
//! C library (`chelper_ref`, the unchanged `klippy/chelper` sources) and to
//! the port. After every operation the test compares everything the unit
//! makes observable:
//!
//! - the messages handed to `serialqueue_send_batch`: bytes, `len`,
//!   `min_clock`, `req_clock` and `notify_id`, in order. Each serial queue
//!   is a debug-file queue (`'f'` on `/dev/null`) whose background thread
//!   is stopped (`serialqueue_exit`) right after allocation, so every batch
//!   stays on the command queue's upcoming list. The C list is walked
//!   through `ss->cq` (`cq->upcoming.msg_queue`); the port's through
//!   `SteppersSync::command_queue` and `CommandQueue::upcoming_messages`.
//! - the `steppersyncmgr_gen_steps` result (which emitter failed, by
//!   allocation index),
//! - every step compressor's history (`stepcompress_extract_old`),
//!   `stepcompress_find_past_position` at fixed clocks and the pending step
//!   direction, which shows the step generation, flush and history expiry
//!   done on the background threads,
//! - the stepper kinematics state the step threads mutate
//!   (`commanded_pos`, `last_flush_time`, `last_move_time`) and the
//!   registered kinematics pointer,
//! - the return codes of the direct `stepcompress` calls,
//! - every log line. The C `stepcompress.c` / `itersolve.c` error lines
//!   (`stepcompress o=...` and the fork's `[DEBUG-motion]` compressor and
//!   solver dumps) are dropped from the C side first: the Rust compressor and
//!   solver do not reproduce them (see their module docs). The serial
//!   queue's own `[DEBUG-motion] ... serial=...` lines are ported and stay
//!   compared. Log lines of one operation are compared sorted, since the
//!   step threads of several failing emitters log in scheduling order.
//!
//! Nothing here depends on background-thread wakeup timing (the step
//! threads are joined by `gen_steps` and the serial threads are stopped),
//! so every case is a plain `#[test]`; none needs the virtual clock.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test steppersync`

use std::ffi::c_void;
use std::fs::File;
use std::os::unix::io::AsRawFd;
use std::sync::{Arc, LazyLock, Once};

use chelper_motion::itersolve::StepperKinematics;
use chelper_motion::kin_cartesian::cartesian_stepper_alloc as port_cartesian_stepper_alloc;
use chelper_motion::steppersync::{SteppersSync, SteppersSyncMgr, SyncEmitter};
use chelper_motion::trapq::{SharedTrapq, Trapq};
use chelper_serial::{SerialQueue, BACKGROUND_PRIORITY_CLOCK, MAX_CLOCK};
use kos::sync::{Mutex, MutexGuard};

// ---- reference C declarations (klippy/chelper/*.h) ----------------------

#[repr(C)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

#[repr(C)]
struct ListHead {
    root: ListNode,
}

/// `struct queue_message` (msgblock.h): `msg` follows `len` directly, the
/// `min_clock` / `req_clock` union starts at offset 72.
#[repr(C)]
struct CQueueMessage {
    len: i32,
    msg: [u8; 64],
    min_clock: u64,
    req_clock: u64,
    notify_id: u64,
    node: ListNode,
}

/// `struct message_sub_queue` (serialqueue.c).
#[repr(C)]
struct CSubQueue {
    msg_queue: ListHead,
    node: ListNode,
}

/// `struct command_queue` (serialqueue.c).
#[repr(C)]
struct CCommandQueue {
    ready: CSubQueue,
    upcoming: CSubQueue,
}

/// Leading fields of `struct steppersync` (steppersync.c).
#[repr(C)]
struct CSteppersync {
    ssm_node: ListNode,
    sq: *mut c_void,
    cq: *mut CCommandQueue,
}

/// `struct pull_history_steps` (stepcompress.h).
#[repr(C)]
#[derive(Clone, Copy)]
struct CHistory {
    first_clock: u64,
    last_clock: u64,
    start_position: i64,
    step_count: i32,
    interval: i32,
    add: i32,
}

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout.
#[repr(C)]
#[allow(dead_code)]
struct CSk {
    step_dist: f64,
    commanded_pos: f64,
    sc: *mut c_void,
    last_flush_time: f64,
    last_move_time: f64,
    tq: *mut c_void,
    active_flags: i32,
    gen_steps_pre_active: f64,
    gen_steps_post_active: f64,
    calc_position_cb: *mut c_void,
    post_cb: *mut c_void,
}

unsafe extern "C" {
    fn steppersyncmgr_alloc() -> *mut c_void;
    fn steppersyncmgr_free(ssm: *mut c_void);
    fn steppersyncmgr_alloc_steppersync(ssm: *mut c_void) -> *mut CSteppersync;
    fn steppersyncmgr_gen_steps(
        ssm: *mut c_void,
        flush_time: f64,
        gen_steps_time: f64,
        clear_history_time: f64,
    ) -> *mut c_void;
    fn steppersync_alloc_syncemitter(
        ss: *mut CSteppersync,
        name: *mut u8,
        alloc_stepcompress: i32,
    ) -> *mut c_void;
    fn steppersync_setup_movequeue(ss: *mut CSteppersync, sq: *mut c_void, move_num: i32);
    fn steppersync_set_time(ss: *mut CSteppersync, time_offset: f64, mcu_freq: f64);
    fn syncemitter_get_stepcompress(se: *mut c_void) -> *mut c_void;
    fn syncemitter_set_stepper_kinematics(se: *mut c_void, sk: *mut CSk);
    fn syncemitter_get_stepper_kinematics(se: *mut c_void) -> *mut CSk;
    fn syncemitter_queue_msg(se: *mut c_void, req_clock: u64, data: *mut u32, len: i32);
    fn stepcompress_fill(
        sc: *mut c_void,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    );
    fn stepcompress_set_invert_sdir(sc: *mut c_void, invert_sdir: u32);
    fn stepcompress_append(sc: *mut c_void, sdir: i32, print_time: f64, step_time: f64) -> i32;
    fn stepcompress_reset(sc: *mut c_void, last_step_clock: u64) -> i32;
    fn stepcompress_set_last_position(sc: *mut c_void, clock: u64, last_position: i64) -> i32;
    fn stepcompress_find_past_position(sc: *mut c_void, clock: u64) -> i64;
    fn stepcompress_get_step_dir(sc: *mut c_void) -> i32;
    fn stepcompress_extract_old(
        sc: *mut c_void,
        p: *mut CHistory,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> i32;
    fn cartesian_stepper_alloc(axis: i8) -> *mut CSk;
    fn itersolve_set_trapq(sk: *mut CSk, tq: *mut c_void, step_dist: f64);
    fn itersolve_set_position(sk: *mut CSk, x: f64, y: f64, z: f64);
    fn trapq_alloc() -> *mut c_void;
    fn trapq_free(tq: *mut c_void);
    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        tq: *mut c_void,
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
    fn trapq_finalize_moves(tq: *mut c_void, print_time: f64, clear_history_time: f64);
    fn serialqueue_alloc(serial_fd: i32, serial_fd_type: u8, client_id: i32, name: *mut u8)
        -> *mut c_void;
    fn serialqueue_exit(sq: *mut c_void);
    fn serialqueue_free(sq: *mut c_void);
    fn set_python_logging_callback(cb: extern "C" fn(*const u8));
    fn free(ptr: *mut c_void);
}

// ---- log capture (process-global) -------------------------------------------

static C_LOGS: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));
static PORT_LOGS: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));
/// The unfiltered C lines of the last operation, shown when a scenario guard
/// fails (the operation's own lines were already compared and consumed).
static LAST_C_LOGS: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));
/// The log sinks are process-global, so cases never overlap even when
/// libtest runs several test threads.
static SERIAL: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));
static INSTALL: Once = Once::new();

fn lock<T>(m: &'static LazyLock<Mutex<T>>) -> MutexGuard<'static, T> {
    m.lock()
}

extern "C" fn c_log_callback(msg: *const u8) {
    // SAFETY: the C `errorf` passes a NUL-terminated buffer.
    let s = unsafe { std::ffi::CStr::from_ptr(msg as *const std::os::raw::c_char) };
    lock(&C_LOGS).push(s.to_string_lossy().into_owned());
}

fn install_capture() {
    INSTALL.call_once(|| {
        // Force the sinks before any thread can log.
        LazyLock::force(&C_LOGS);
        LazyLock::force(&PORT_LOGS);
        chelper_ref::link();
        // SAFETY: plain function pointer stored in the C global.
        unsafe { set_python_logging_callback(c_log_callback) };
        chelper_serial::set_logging_callback(|msg: &str| lock(&PORT_LOGS).push(msg.to_string()));
    });
}

/// Take the log lines of both sides, dropping the C compressor/solver
/// lines the Rust units do not reproduce, sorted.
fn take_logs() -> (Vec<String>, Vec<String>) {
    let raw_c = std::mem::take(&mut *lock(&C_LOGS));
    lock(&LAST_C_LOGS).clone_from(&raw_c);
    let mut c: Vec<String> = raw_c
        .into_iter()
        .filter(|l| {
            let compressor_dump = l.starts_with("[DEBUG-motion]") && !l.contains(" serial=");
            !compressor_dump && !l.starts_with("stepcompress o=")
        })
        .collect();
    let mut p = std::mem::take(&mut *lock(&PORT_LOGS));
    c.sort();
    p.sort();
    (c, p)
}

// ---- operations ---------------------------------------------------------------

/// One trapezoidal move (`trapq_append` arguments).
#[derive(Clone, Copy, Debug)]
struct MoveParams {
    print_time: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    cruise_v: f64,
    accel: f64,
}

/// A move of `dist` along the unit vector `axes_r` from rest to rest.
fn trap(print_time: f64, start_pos: [f64; 3], axes_r: [f64; 3], dist: f64, v: f64, accel: f64)
    -> MoveParams {
    let mut cruise_v = v;
    let mut accel_t = v / accel;
    let accel_d = 0.5 * accel * accel_t * accel_t;
    let cruise_t = if 2.0 * accel_d >= dist {
        cruise_v = (dist * accel).sqrt();
        accel_t = cruise_v / accel;
        0.0
    } else {
        (dist - 2.0 * accel_d) / v
    };
    MoveParams { print_time, accel_t, cruise_t, decel_t: accel_t, start_pos, axes_r, cruise_v, accel }
}

#[derive(Clone, Debug)]
enum Op {
    /// `steppersyncmgr_alloc_steppersync`.
    AllocSs,
    /// `steppersync_alloc_syncemitter(ss, name, sc)`.
    AllocSe { ss: usize, name: &'static [u8], sc: bool },
    /// `stepcompress_fill(sc, oid, max_error, 20 + oid, 40 + oid)`.
    Fill { se: usize, oid: u32, max_error: u32 },
    /// `stepcompress_set_invert_sdir`.
    InvertSdir { se: usize, invert: u32 },
    /// A fresh stopped serial queue and `steppersync_setup_movequeue`.
    Setup { ss: usize, move_num: usize },
    /// `steppersync_set_time`.
    SetTime { ss: usize, offset: f64, freq: f64 },
    /// A cartesian kinematics on the shared trapq, registered with
    /// `syncemitter_set_stepper_kinematics`.
    AttachSk { se: usize, axis: u8, step_dist: f64 },
    /// `syncemitter_set_stepper_kinematics(se, NULL)`.
    DetachSk { se: usize },
    /// `itersolve_set_position` on the attached kinematics.
    SetPosition { se: usize, pos: [f64; 3] },
    /// `trapq_append` on the shared trapq.
    Move(MoveParams),
    /// `trapq_finalize_moves`.
    Finalize { print_time: f64, clear: f64 },
    /// `syncemitter_queue_msg`.
    QueueMsg { se: usize, req_clock: u64, data: Vec<u32> },
    /// `stepcompress_append` (return compared).
    ScAppend { se: usize, sdir: i32, print_time: f64, step_time: f64 },
    /// `stepcompress_reset` (return compared).
    ScReset { se: usize, clock: u64 },
    /// `stepcompress_set_last_position` (return compared).
    ScSetLastPosition { se: usize, clock: u64, pos: i64 },
    /// `steppersyncmgr_gen_steps` (failed emitter compared).
    GenSteps { flush: f64, gen: f64, clear: f64 },
    /// Coverage guard on the reference: whether the last `GenSteps`
    /// reported a failure in the C.
    ExpectFailed(bool),
}

// ---- the two sides ------------------------------------------------------------

/// One observed message: bytes, `min_clock`, `req_clock`, `notify_id`.
type Msg = (Vec<u8>, u64, u64, u64);

/// One history entry as a tuple.
type Hist = (u64, u64, i64, i32, i32, i32);

const HISTORY_MAX: i32 = 4096;
const PAST_CLOCKS: [u64; 6] = [0, 400_000, 700_000, 1_000_000, 1_600_000, 3_000_000];

fn null_file() -> File {
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/null")
        .expect("open /dev/null")
}

fn c_name(name: &[u8]) -> [u8; 32] {
    let mut buf = [0u8; 32];
    let n = name.len().min(31);
    buf[..n].copy_from_slice(&name[..n]);
    buf
}

struct CSide {
    ssm: *mut c_void,
    ss: Vec<*mut CSteppersync>,
    se: Vec<*mut c_void>,
    sk: Vec<*mut CSk>,
    sq: Vec<*mut c_void>,
    tq: *mut c_void,
    last_failed: Option<usize>,
}

struct PortSide {
    ssm: Option<SteppersSyncMgr>,
    ss: Vec<Arc<SteppersSync>>,
    se: Vec<Arc<SyncEmitter>>,
    /// Every kinematics ever attached, kept alive until teardown. Boxed
    /// so the registered pointers stay valid while the vector grows.
    #[allow(clippy::vec_box)]
    sk: Vec<Box<StepperKinematics>>,
    sq: Vec<Arc<SerialQueue>>,
    tq: SharedTrapq,
    last_failed: Option<usize>,
}

struct Lockstep {
    c: CSide,
    p: PortSide,
    /// Per emitter: index into the `sk` vectors of the attached kinematics.
    attached: Vec<Option<usize>>,
    files: Vec<File>,
}

impl Lockstep {
    fn new() -> Lockstep {
        // SAFETY: plain allocations of the reference library.
        let (ssm, tq) = unsafe { (steppersyncmgr_alloc(), trapq_alloc()) };
        assert!(!ssm.is_null() && !tq.is_null());
        Lockstep {
            c: CSide { ssm, ss: Vec::new(), se: Vec::new(), sk: Vec::new(), sq: Vec::new(), tq,
                       last_failed: None },
            p: PortSide {
                ssm: Some(SteppersSyncMgr::alloc()),
                ss: Vec::new(),
                se: Vec::new(),
                sk: Vec::new(),
                sq: Vec::new(),
                tq: Arc::new(kos::sync::Mutex::new(Trapq::new())),
                last_failed: None,
            },
            attached: Vec::new(),
            files: Vec::new(),
        }
    }

    fn port_mgr(&self) -> &SteppersSyncMgr {
        self.p.ssm.as_ref().expect("manager alive")
    }

    fn c_sc(&self, se: usize) -> *mut c_void {
        // SAFETY: a live emitter of the reference.
        let sc = unsafe { syncemitter_get_stepcompress(self.c.se[se]) };
        assert!(!sc.is_null(), "emitter {se} has no C stepcompress");
        sc
    }

    fn port_sc(&self, se: usize) -> chelper_motion::stepcompress::SharedStepcompress {
        self.p.se[se].get_stepcompress().expect("emitter has a port stepcompress")
    }

    fn apply(&mut self, op: &Op) {
        match op {
            Op::AllocSs => {
                // SAFETY: live manager.
                let ss = unsafe { steppersyncmgr_alloc_steppersync(self.c.ssm) };
                assert!(!ss.is_null());
                self.c.ss.push(ss);
                let ss = self.port_mgr().alloc_steppersync();
                self.p.ss.push(ss);
            }
            Op::AllocSe { ss, name, sc } => {
                let mut buf = c_name(name);
                // SAFETY: live steppersync; `buf` holds at least 16 bytes.
                let se = unsafe {
                    steppersync_alloc_syncemitter(self.c.ss[*ss], buf.as_mut_ptr(), i32::from(*sc))
                };
                assert!(!se.is_null(), "C syncemitter alloc failed");
                self.c.se.push(se);
                let se = self.p.ss[*ss].alloc_syncemitter(name, *sc);
                self.p.se.push(se);
                self.attached.push(None);
                // SAFETY: live emitter.
                let c_has_sc = !unsafe { syncemitter_get_stepcompress(self.c.se[self.c.se.len() - 1]) }
                    .is_null();
                let p_has_sc = self.p.se[self.p.se.len() - 1].get_stepcompress().is_some();
                assert_eq!(c_has_sc, p_has_sc, "stepcompress presence");
            }
            Op::Fill { se, oid, max_error } => {
                let (q, d) = (20 + *oid as i32, 40 + *oid as i32);
                // SAFETY: live compressor.
                unsafe { stepcompress_fill(self.c_sc(*se), *oid, *max_error, q, d) };
                self.port_sc(*se).lock().fill(*oid, *max_error, q, d);
            }
            Op::InvertSdir { se, invert } => {
                // SAFETY: live compressor.
                unsafe { stepcompress_set_invert_sdir(self.c_sc(*se), *invert) };
                self.port_sc(*se).lock().set_invert_sdir(*invert);
            }
            Op::Setup { ss, move_num } => {
                let fc = null_file();
                let fp = null_file();
                let mut name = c_name(b"steppersync");
                // SAFETY: valid fd and 16-byte name; the thread is stopped
                // before any message is queued.
                let csq = unsafe { serialqueue_alloc(fc.as_raw_fd(), b'f', 0, name.as_mut_ptr()) };
                assert!(!csq.is_null(), "C serialqueue_alloc failed");
                unsafe {
                    serialqueue_exit(csq);
                    steppersync_setup_movequeue(self.c.ss[*ss], csq, *move_num as i32);
                }
                self.c.sq.push(csq);
                let psq = SerialQueue::alloc(fp.as_raw_fd(), b'f', 0, b"steppersync")
                    .expect("port SerialQueue::alloc failed");
                psq.exit();
                let psq = Arc::new(psq);
                self.p.ss[*ss].setup_movequeue(Arc::clone(&psq), *move_num);
                self.p.sq.push(psq);
                self.files.push(fc);
                self.files.push(fp);
            }
            Op::SetTime { ss, offset, freq } => {
                // SAFETY: live steppersync.
                unsafe { steppersync_set_time(self.c.ss[*ss], *offset, *freq) };
                self.p.ss[*ss].set_time(*offset, *freq);
            }
            Op::AttachSk { se, axis, step_dist } => {
                // SAFETY: plain reference allocations; the kinematics is
                // freed only after the manager.
                let csk = unsafe { cartesian_stepper_alloc(*axis as i8) };
                assert!(!csk.is_null());
                unsafe {
                    itersolve_set_trapq(csk, self.c.tq, *step_dist);
                    syncemitter_set_stepper_kinematics(self.c.se[*se], csk);
                }
                self.c.sk.push(csk);
                let mut psk = Box::new(port_cartesian_stepper_alloc(*axis));
                psk.set_trapq(Some(Arc::clone(&self.p.tq)), *step_dist);
                let ptr: *mut StepperKinematics = &mut *psk;
                // SAFETY: the box lives in `self.p.sk` until after the
                // manager is dropped and is only touched between calls.
                unsafe { self.p.se[*se].set_stepper_kinematics(ptr) };
                self.p.sk.push(psk);
                self.attached[*se] = Some(self.c.sk.len() - 1);
            }
            Op::DetachSk { se } => {
                // SAFETY: live emitter.
                unsafe { syncemitter_set_stepper_kinematics(self.c.se[*se], std::ptr::null_mut()) };
                // SAFETY: clearing the registration.
                unsafe { self.p.se[*se].set_stepper_kinematics(std::ptr::null_mut()) };
                self.attached[*se] = None;
            }
            Op::SetPosition { se, pos } => {
                let k = self.attached[*se].expect("kinematics attached");
                // SAFETY: live kinematics; step threads are idle.
                unsafe { itersolve_set_position(self.c.sk[k], pos[0], pos[1], pos[2]) };
                self.p.sk[k].set_position(pos[0], pos[1], pos[2]);
            }
            Op::Move(m) => {
                // SAFETY: live trapq; step threads are idle.
                unsafe {
                    trapq_append(self.c.tq, m.print_time, m.accel_t, m.cruise_t, m.decel_t,
                                 m.start_pos[0], m.start_pos[1], m.start_pos[2], m.axes_r[0],
                                 m.axes_r[1], m.axes_r[2], 0.0, m.cruise_v, m.accel)
                };
                self.p.tq.lock().append(m.print_time, m.accel_t, m.cruise_t, m.decel_t,
                                        m.start_pos[0], m.start_pos[1], m.start_pos[2],
                                        m.axes_r[0], m.axes_r[1], m.axes_r[2], 0.0, m.cruise_v,
                                        m.accel);
            }
            Op::Finalize { print_time, clear } => {
                // SAFETY: live trapq.
                unsafe { trapq_finalize_moves(self.c.tq, *print_time, *clear) };
                self.p.tq.lock().finalize_moves(*print_time, *clear);
            }
            Op::QueueMsg { se, req_clock, data } => {
                let mut cdata = data.clone();
                // SAFETY: live emitter; `cdata` holds `len` values.
                unsafe {
                    syncemitter_queue_msg(self.c.se[*se], *req_clock, cdata.as_mut_ptr(),
                                          cdata.len() as i32)
                };
                self.p.se[*se].queue_msg(*req_clock, data);
            }
            Op::ScAppend { se, sdir, print_time, step_time } => {
                // SAFETY: live compressor.
                let c = unsafe { stepcompress_append(self.c_sc(*se), *sdir, *print_time, *step_time) };
                let p = self.port_sc(*se).lock().append(*sdir, *print_time, *step_time);
                assert_eq!(c, p, "stepcompress_append return");
            }
            Op::ScReset { se, clock } => {
                // SAFETY: live compressor.
                let c = unsafe { stepcompress_reset(self.c_sc(*se), *clock) };
                let p = self.port_sc(*se).lock().reset(*clock);
                assert_eq!(c, p, "stepcompress_reset return");
            }
            Op::ScSetLastPosition { se, clock, pos } => {
                // SAFETY: live compressor.
                let c = unsafe { stepcompress_set_last_position(self.c_sc(*se), *clock, *pos) };
                let p = self.port_sc(*se).lock().set_last_position(*clock, *pos);
                assert_eq!(c, p, "stepcompress_set_last_position return");
            }
            Op::GenSteps { flush, gen, clear } => {
                // SAFETY: live manager; every registered kinematics is alive.
                let c = unsafe { steppersyncmgr_gen_steps(self.c.ssm, *flush, *gen, *clear) };
                self.c.last_failed = if c.is_null() {
                    None
                } else {
                    Some(self.c.se.iter().position(|&s| s == c).expect("C failed emitter is known"))
                };
                let p = self.port_mgr().gen_steps(*flush, *gen, *clear);
                self.p.last_failed = p.map(|p| {
                    self.p.se.iter().position(|s| Arc::ptr_eq(s, &p)).expect("port failed emitter is known")
                });
                assert_eq!(self.c.last_failed, self.p.last_failed, "gen_steps failed emitter");
            }
            // Checked by `run`, which knows the operation index.
            Op::ExpectFailed(_) => {}
        }
    }

    fn c_upcoming(&self, ss: usize) -> Option<Vec<Msg>> {
        // SAFETY: live steppersync; its command queue (if any) is either
        // live or leaked (never freed while non-empty).
        let cq = unsafe { (*self.c.ss[ss]).cq };
        if cq.is_null() {
            return None;
        }
        let mut out = Vec::new();
        // SAFETY: walk the C list; every node is the `node` field of a
        // live `struct queue_message`, and no thread touches it.
        unsafe {
            let root = std::ptr::addr_of_mut!((*cq).upcoming.msg_queue.root);
            let mut cur = (*root).next;
            while cur != root {
                let qm = &*((cur as *const u8).sub(std::mem::offset_of!(CQueueMessage, node))
                    as *const CQueueMessage);
                let len = qm.len.clamp(0, 64) as usize;
                out.push((qm.msg[..len].to_vec(), qm.min_clock, qm.req_clock, qm.notify_id));
                cur = (*cur).next;
            }
        }
        Some(out)
    }

    fn port_upcoming(&self, ss: usize) -> Option<Vec<Msg>> {
        let cq = self.p.ss[ss].command_queue()?;
        Some(
            cq.upcoming_messages()
                .iter()
                .map(|m| {
                    let len = m.len.clamp(0, 64) as usize;
                    (m.msg[..len].to_vec(), m.min_clock, m.req_clock, m.notify_id)
                })
                .collect(),
        )
    }

    fn c_history(&self, se: usize) -> Vec<Hist> {
        let mut buf = vec![
            CHistory { first_clock: 0, last_clock: 0, start_position: 0, step_count: 0,
                       interval: 0, add: 0 };
            HISTORY_MAX as usize
        ];
        // SAFETY: live compressor; `buf` holds `HISTORY_MAX` entries.
        let n = unsafe {
            stepcompress_extract_old(self.c_sc(se), buf.as_mut_ptr(), HISTORY_MAX, 0, u64::MAX)
        };
        assert!((0..=HISTORY_MAX).contains(&n));
        buf[..n as usize]
            .iter()
            .map(|h| (h.first_clock, h.last_clock, h.start_position, h.step_count, h.interval, h.add))
            .collect()
    }

    fn port_history(&self, se: usize) -> Vec<Hist> {
        self.port_sc(se)
            .lock()
            .extract_old(HISTORY_MAX, 0, u64::MAX)
            .iter()
            .map(|h| (h.first_clock, h.last_clock, h.start_position, h.step_count, h.interval, h.add))
            .collect()
    }

    /// Compare everything observable after one operation.
    fn compare(&self, ctx: &str) {
        let (c_logs, p_logs) = take_logs();
        assert_eq!(c_logs, p_logs, "{ctx}: log lines");
        for ss in 0..self.c.ss.len() {
            let c = self.c_upcoming(ss);
            let p = self.port_upcoming(ss);
            assert_eq!(c, p, "{ctx}: messages sent by steppersync {ss}");
        }
        for se in 0..self.c.se.len() {
            if self.p.se[se].get_stepcompress().is_none() {
                continue;
            }
            assert_eq!(self.c_history(se), self.port_history(se), "{ctx}: history of emitter {se}");
            let c_sc = self.c_sc(se);
            let p_sc = self.port_sc(se);
            let p_sc = p_sc.lock();
            for clock in PAST_CLOCKS {
                // SAFETY: live compressor.
                let c = unsafe { stepcompress_find_past_position(c_sc, clock) };
                assert_eq!(c, p_sc.find_past_position(clock),
                           "{ctx}: emitter {se} find_past_position({clock})");
            }
            // SAFETY: live compressor.
            let c_dir = unsafe { stepcompress_get_step_dir(c_sc) };
            assert_eq!(c_dir, p_sc.get_step_dir(), "{ctx}: emitter {se} step dir");
        }
        for (se, attached) in self.attached.iter().enumerate() {
            // SAFETY: live emitter.
            let c_ptr = unsafe { syncemitter_get_stepper_kinematics(self.c.se[se]) };
            let p_ptr = self.p.se[se].get_stepper_kinematics();
            match attached {
                None => {
                    assert!(c_ptr.is_null() && p_ptr.is_null(), "{ctx}: emitter {se} kinematics unset");
                }
                Some(k) => {
                    assert_eq!(c_ptr, self.c.sk[*k], "{ctx}: emitter {se} C kinematics");
                    assert_eq!(p_ptr, &*self.p.sk[*k] as *const StepperKinematics as *mut _,
                               "{ctx}: emitter {se} port kinematics");
                }
            }
        }
        for (k, (&csk, psk)) in self.c.sk.iter().zip(self.p.sk.iter()).enumerate() {
            // SAFETY: live kinematics; step threads are idle.
            let c = unsafe { &*csk };
            assert_eq!(
                (c.commanded_pos.to_bits(), c.last_flush_time.to_bits(), c.last_move_time.to_bits()),
                (psk.commanded_pos.to_bits(), psk.last_flush_time.to_bits(),
                 psk.last_move_time.to_bits()),
                "{ctx}: kinematics {k} state"
            );
        }
    }

    /// `steppersyncmgr_free`, then free the serial queues and the rest.
    fn teardown(mut self, ctx: &str) {
        // SAFETY: reference objects freed in the C order; nothing uses
        // them afterwards.
        unsafe { steppersyncmgr_free(self.c.ssm) };
        drop(self.p.ssm.take());
        let (c_logs, p_logs) = take_logs();
        assert_eq!(c_logs, p_logs, "{ctx}: teardown log lines");
        // SAFETY: as above.
        unsafe {
            for &sq in &self.c.sq {
                serialqueue_free(sq);
            }
            for &sk in &self.c.sk {
                free(sk as *mut c_void);
            }
            trapq_free(self.c.tq);
        }
        self.p.se.clear();
        self.p.ss.clear();
        self.p.sq.clear();
        let (c_logs, p_logs) = take_logs();
        assert_eq!(c_logs, p_logs, "{ctx}: serialqueue free log lines");
        self.files.clear();
    }
}

/// Run one scenario on both sides and compare after every operation.
fn run(name: &str, ops: &[Op]) {
    let _serial = lock(&SERIAL);
    install_capture();
    take_logs();
    let mut ls = Lockstep::new();
    for (i, op) in ops.iter().enumerate() {
        let ctx = format!("{name} op {i} {op:?}");
        ls.apply(op);
        if let Op::ExpectFailed(failed) = op {
            assert_eq!(
                ls.c.last_failed.is_some(),
                *failed,
                "{ctx}: scenario guard: the reference gen_steps result changed; C log of the \
                 previous operation: {:?}",
                &*lock(&LAST_C_LOGS)
            );
        }
        ls.compare(&ctx);
    }
    ls.teardown(name);
}

// ---- scenario builders --------------------------------------------------------

/// A mcu with `n` cartesian steppers (x, y, z, ...) at 1 MHz, a move queue
/// of `move_num` slots and an X/Y move pattern.
fn base_mcu(ss: usize, first_se: usize, axes: &[u8], move_num: usize, freq: f64, offset: f64)
    -> Vec<Op> {
    let mut ops = vec![Op::AllocSs];
    for (i, &axis) in axes.iter().enumerate() {
        let se = first_se + i;
        let name: &'static [u8] = match axis {
            b'x' => b"stepper_x",
            b'y' => b"stepper_y",
            _ => b"stepper_z",
        };
        ops.push(Op::AllocSe { ss, name, sc: true });
        ops.push(Op::Fill { se, oid: se as u32 + 3, max_error: 25 });
    }
    ops.push(Op::Setup { ss, move_num });
    ops.push(Op::SetTime { ss, offset, freq });
    for (i, &axis) in axes.iter().enumerate() {
        ops.push(Op::AttachSk { se: first_se + i, axis, step_dist: 0.0125 });
    }
    ops
}

fn xy_moves(t0: f64) -> Vec<Op> {
    vec![
        Op::Move(trap(t0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 10.0, 50.0, 1000.0)),
        Op::Move(trap(t0 + 0.3, [10.0, 0.0, 0.0], [0.0, 1.0, 0.0], 8.0, 40.0, 1000.0)),
        Op::Move(trap(t0 + 0.6, [10.0, 8.0, 0.0], [-0.6, -0.8, 0.0], 10.0, 60.0, 2000.0)),
        Op::Move(trap(t0 + 0.85, [4.0, 0.0, 0.0], [0.0, 0.0, 1.0], 2.0, 10.0, 500.0)),
    ]
}

// ---- cases ----------------------------------------------------------------------

/// Three steppers and a non-stepper emitter on one mcu: steps from the
/// background threads interleaved by clock with `queue_msg` commands
/// (`req_clock` 0 and non-zero, ties between emitters), direction changes
/// and an inverted direction.
#[test]
fn steppersync_multi_stepper_one_mcu() {
    let mut ops = base_mcu(0, 0, b"xyz", 16, 1_000_000.0, 0.0);
    ops.push(Op::AllocSe { ss: 0, name: b"pwm_tool fan", sc: false });
    ops.push(Op::InvertSdir { se: 1, invert: 1 });
    ops.extend(xy_moves(0.5));
    // The stepper reset command of `stepper.py` (req_clock 0) and a
    // pwm update with a clock.
    ops.push(Op::QueueMsg { se: 0, req_clock: 0, data: vec![9, 3, 0] });
    ops.push(Op::QueueMsg { se: 3, req_clock: 520_000, data: vec![12, 7, 520_000, 255] });
    ops.push(Op::QueueMsg { se: 1, req_clock: 530_000, data: vec![13, 4, 1] });
    ops.push(Op::QueueMsg { se: 2, req_clock: 530_000, data: vec![13, 5, 2] });
    ops.push(Op::QueueMsg { se: 3, req_clock: 530_000, data: vec![12, 7, 530_000, 0] });
    for (k, t) in [0.55, 0.65, 0.8, 0.95, 1.1, 1.2, 1.5].iter().enumerate() {
        ops.push(Op::GenSteps { flush: *t, gen: t + 0.05, clear: t - 0.3 });
        ops.push(Op::ExpectFailed(false));
        ops.push(Op::QueueMsg { se: 3, req_clock: ((t + 0.02) * 1e6) as u64,
                                data: vec![12, 7, k as u32, 128] });
        ops.push(Op::QueueMsg { se: k % 3, req_clock: 0, data: vec![9, 3 + (k % 3) as u32, 0] });
        ops.push(Op::Finalize { print_time: t - 0.1, clear: t - 0.4 });
    }
    run("multi_stepper_one_mcu", &ops);
}

/// Two mcus with different clock conversions and an unconfigured mcu with
/// an idle emitter; an emitter allocated after `set_time` picks up the
/// conversion on the next `set_time`.
#[test]
fn steppersync_two_mcus() {
    let mut ops = base_mcu(0, 0, b"xy", 8, 1_000_000.0, 0.0);
    ops.extend(base_mcu(1, 2, b"z", 8, 2_000_000.0, 0.25));
    ops.push(Op::AllocSs);
    ops.push(Op::AllocSe { ss: 2, name: b"idle", sc: false });
    ops.push(Op::AllocSe { ss: 1, name: b"extruder", sc: true });
    ops.push(Op::Fill { se: 4, oid: 9, max_error: 50 });
    ops.push(Op::SetTime { ss: 1, offset: 0.25, freq: 2_000_000.0 });
    ops.push(Op::AttachSk { se: 4, axis: b'x', step_dist: 0.005 });
    ops.extend(xy_moves(0.5));
    ops.push(Op::QueueMsg { se: 2, req_clock: 600_000, data: vec![30, 1] });
    ops.push(Op::QueueMsg { se: 0, req_clock: 600_000, data: vec![30, 2] });
    for t in [0.6, 0.9, 1.05, 1.3, 2.0] {
        ops.push(Op::GenSteps { flush: t, gen: t, clear: 0.0 });
        ops.push(Op::ExpectFailed(false));
    }
    run("two_mcus", &ops);
}

/// A two-slot move queue: `min_clock` comes from the slot heap, commands
/// past the flush clock stay queued, and `req_clock` 0, background
/// priority, `MAX_CLOCK` / `u64::MAX` (never sent) and far-future clocks
/// (the 31-bit `min_clock` adjustment of `send_batch`) are mixed in.
#[test]
fn steppersync_movequeue_saturation() {
    let mut ops = vec![
        Op::AllocSs,
        Op::AllocSe { ss: 0, name: b"a", sc: false },
        Op::AllocSe { ss: 0, name: b"b", sc: false },
        Op::Setup { ss: 0, move_num: 2 },
        Op::SetTime { ss: 0, offset: 0.0, freq: 1_000_000.0 },
    ];
    for i in 0..12u64 {
        let se = (i % 2) as usize;
        ops.push(Op::QueueMsg { se, req_clock: 100_000 + 37_000 * i, data: vec![50, se as u32, i as u32] });
    }
    ops.push(Op::QueueMsg { se: 1, req_clock: 0, data: vec![51] });
    ops.push(Op::QueueMsg { se: 0, req_clock: BACKGROUND_PRIORITY_CLOCK, data: vec![52] });
    ops.push(Op::QueueMsg { se: 1, req_clock: 3 << 30, data: vec![53] });
    ops.push(Op::GenSteps { flush: 0.25, gen: 0.25, clear: 0.0 });
    ops.push(Op::GenSteps { flush: 0.3, gen: 0.3, clear: 0.0 });
    ops.push(Op::QueueMsg { se: 0, req_clock: 0, data: vec![54] });
    ops.push(Op::GenSteps { flush: 0.6, gen: 0.6, clear: 0.0 });
    ops.push(Op::QueueMsg { se: 1, req_clock: MAX_CLOCK, data: vec![55] });
    ops.push(Op::QueueMsg { se: 0, req_clock: u64::MAX, data: vec![56] });
    ops.push(Op::QueueMsg { se: 0, req_clock: 0, data: vec![57] });
    ops.push(Op::GenSteps { flush: 2.0, gen: 2.0, clear: 0.0 });
    ops.push(Op::GenSteps { flush: 5000.0, gen: 5000.0, clear: 0.0 });
    run("movequeue_saturation", &ops);
}

/// Step history expiry by `clear_history_time`, position markers from
/// `set_last_position`, and a `reset` between moves.
#[test]
fn steppersync_history_expiry() {
    let mut ops = base_mcu(0, 0, b"xy", 32, 1_000_000.0, 0.0);
    ops.extend(xy_moves(0.5));
    ops.push(Op::ScSetLastPosition { se: 0, clock: 400_000, pos: 1000 });
    ops.push(Op::GenSteps { flush: 0.7, gen: 0.7, clear: 0.0 });
    ops.push(Op::ExpectFailed(false));
    ops.push(Op::GenSteps { flush: 0.9, gen: 1.0, clear: 0.6 });
    ops.push(Op::ExpectFailed(false));
    // Generate the y steps of `xy_moves` (they end near 1.29 s) before the
    // reset at 1.3 s, so no step earlier than the reset clock is pending.
    ops.push(Op::GenSteps { flush: 1.2, gen: 1.3, clear: 0.8 });
    ops.push(Op::ExpectFailed(false));
    ops.push(Op::ScReset { se: 1, clock: 1_300_000 });
    ops.push(Op::ScSetLastPosition { se: 1, clock: 1_300_000, pos: -50 });
    ops.push(Op::SetPosition { se: 1, pos: [4.0, 0.0, 0.0] });
    // Starts after the last `xy_moves` move ends (1.35 + 0.22 s): an
    // overlapping move would make the step times go backwards.
    ops.push(Op::Move(trap(1.6, [4.0, 0.0, 0.0], [0.0, 1.0, 0.0], 5.0, 50.0, 1000.0)));
    ops.push(Op::GenSteps { flush: 1.5, gen: 1.6, clear: 1.0 });
    ops.push(Op::ExpectFailed(false));
    ops.push(Op::GenSteps { flush: 2.0, gen: 2.0, clear: 1.45 });
    ops.push(Op::ExpectFailed(false));
    ops.push(Op::GenSteps { flush: 3.0, gen: 3.0, clear: 3.0 });
    ops.push(Op::ExpectFailed(false));
    run("history_expiry", &ops);
}

/// Step generation failures: the failing emitter is reported (the last
/// one when several fail), its mcu and every later mcu are not flushed,
/// and the error line carries the 15-byte name. Also an emitter with a
/// compressor but no kinematics (its `reset` output is still sent), an
/// encode overflow in `queue_msg` and a batch of only empty messages.
#[test]
fn steppersync_error_paths() {
    let mut ops = vec![
        Op::AllocSs,
        Op::AllocSe { ss: 0, name: b"stepper_extruder1", sc: true },
        Op::AllocSe { ss: 0, name: b"ab\0cd", sc: false },
        Op::Fill { se: 0, oid: 1, max_error: 25 },
        Op::Setup { ss: 0, move_num: 8 },
        Op::SetTime { ss: 0, offset: 0.0, freq: 1_000_000.0 },
        Op::AllocSs,
        Op::AllocSe { ss: 1, name: b"stepper_late_mcu_name", sc: true },
        Op::AllocSe { ss: 1, name: b"no_kinematics", sc: true },
        Op::Fill { se: 2, oid: 2, max_error: 25 },
        Op::Fill { se: 3, oid: 3, max_error: 25 },
        Op::Setup { ss: 1, move_num: 8 },
        Op::SetTime { ss: 1, offset: 0.0, freq: 1_000_000.0 },
        Op::AttachSk { se: 0, axis: b'z', step_dist: 0.01 },
        Op::AttachSk { se: 2, axis: b'z', step_dist: 0.01 },
        // Encode overflow: an empty message alone in its batch.
        Op::QueueMsg { se: 1, req_clock: 0, data: vec![0x7fff_ffff; 12] },
        Op::GenSteps { flush: 0.01, gen: 0.01, clear: 0.0 },
        Op::ExpectFailed(false),
        // The emitter without kinematics: steps only through `reset`.
        Op::ScAppend { se: 3, sdir: 1, print_time: 0.02, step_time: 0.0 },
        Op::ScAppend { se: 3, sdir: 1, print_time: 0.021, step_time: 0.0 },
        Op::ScReset { se: 3, clock: 0 },
        Op::QueueMsg { se: 1, req_clock: 15_000, data: vec![60, 1] },
        Op::QueueMsg { se: 2, req_clock: 15_000, data: vec![61, 2] },
        // Backwards steps on emitter 0 (mcu 0) fail in the step thread.
        Op::ScAppend { se: 0, sdir: 1, print_time: 0.001, step_time: 0.0 },
        Op::ScAppend { se: 0, sdir: 1, print_time: 0.003, step_time: 0.0 },
        Op::ScAppend { se: 0, sdir: 1, print_time: 0.002, step_time: 0.0 },
        Op::GenSteps { flush: 0.05, gen: 0.05, clear: 0.0 },
        Op::ExpectFailed(true),
        // Emitter 2 (mcu 1) fails as well: the later emitter is reported.
        Op::ScAppend { se: 2, sdir: 0, print_time: 0.06, step_time: 0.0 },
        Op::ScAppend { se: 2, sdir: 0, print_time: 0.08, step_time: 0.0 },
        Op::ScAppend { se: 2, sdir: 0, print_time: 0.07, step_time: 0.0 },
        Op::GenSteps { flush: 0.1, gen: 0.1, clear: 0.0 },
        Op::ExpectFailed(true),
        // Without its kinematics emitter 0 no longer generates; mcu 1
        // still fails, mcu 0 is flushed.
        Op::DetachSk { se: 0 },
        Op::GenSteps { flush: 0.2, gen: 0.2, clear: 0.0 },
        Op::ExpectFailed(true),
        Op::DetachSk { se: 2 },
        Op::GenSteps { flush: 0.3, gen: 0.3, clear: 0.0 },
        Op::ExpectFailed(false),
    ];
    ops.push(Op::QueueMsg { se: 1, req_clock: 0, data: vec![62] });
    run("error_paths", &ops);
}

/// Reconfiguring the move queue frees a non-empty command queue (a leak
/// log line), restarts the slot heap and continues on the new serial
/// queue; teardown frees a command queue that still holds messages.
#[test]
fn steppersync_setup_movequeue_again() {
    let mut ops = base_mcu(0, 0, b"x", 3, 1_000_000.0, 0.0);
    ops.extend(xy_moves(0.5));
    ops.push(Op::GenSteps { flush: 0.6, gen: 0.6, clear: 0.0 });
    ops.push(Op::Setup { ss: 0, move_num: 4 });
    ops.push(Op::GenSteps { flush: 0.7, gen: 0.7, clear: 0.0 });
    ops.push(Op::Setup { ss: 0, move_num: 1 });
    ops.push(Op::Setup { ss: 0, move_num: 5 });
    ops.push(Op::GenSteps { flush: 2.0, gen: 2.0, clear: 0.0 });
    ops.push(Op::ExpectFailed(false));
    run("setup_movequeue_again", &ops);
}
