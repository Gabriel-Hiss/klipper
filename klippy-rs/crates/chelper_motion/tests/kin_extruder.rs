//! Differential parity tests for `chelper_motion::kin_extruder`.
//!
//! Every expected value comes from the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/kin_extruder.c`
//! sources), which is the oracle for this C unit: there is no
//! Python object for `kin_extruder.c` to query through `pyoracle`
//! and `tests/fixtures/recordings/` holds no `kin_extruder`
//! recordings (the units are `recordable: false`). Each test drives
//! the Rust port and the C library with identical inputs and
//! compares bit for bit (`f64::to_bits`):
//!
//! - all `extruder_set_pressure_advance` state: the `pa_list`
//!   entries, `half_smooth_time`, `inv_half_smooth_time2`, and the
//!   `gen_steps_pre_active` / `gen_steps_post_active` solver fields
//!   (including the C quirk that a zero `smooth_time` returns before
//!   recomputing `inv_half_smooth_time2`, leaving it stale, and the
//!   retain-same-PA shortcut that still updates the smoothing
//!   fields),
//! - the position callback (`extruder_calc_position`) for every move
//!   of scripted trapqs and for many `(move, move_time)` points,
//!   covering both closed-form integrals, the can-PA flag
//!   (`axes_r.y`), pressure-advance selection by `print_time`,
//!   window crossing into previous and future moves, null moves,
//!   the tail sentinel, and the no-PA fast path,
//! - `itersolve_calc_position_from_coord` on both sides (the
//!   synthetic-move path used by `itersolve_set_position`), and
//! - full `itersolve_generate_steps` streams with the extruder
//!   callback attached, comparing stepcompress messages and
//!   history.
//!
//! One C path is deliberately not exercised: when the smoothing
//! window extends before the first real move, the C prev-walk steps
//! past the head sentinel (whose `move_t` is 0) into heap memory
//! (`trapq.c` / `kin_extruder.c` `pa_range_integrate`). Its result
//! depends on heap contents. The port stops at the head sentinel
//! instead; see `kin_extruder.rs` module docs. All cases below keep
//! the window within the queue or reaching the tail sentinel (which
//! is well defined in C).
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_extruder`
use chelper_motion::itersolve::AF_X;
use chelper_motion::kin_extruder::{
    extruder_calc_position, extruder_stepper_alloc as rust_extruder_stepper_alloc,
    ExtruderStepper, PaParams,
};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{Move, Trapq};
use core::ffi::c_void;

use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from kin_extruder.c / itersolve.h /
// trapq.h / stepcompress.h / list.h).
// ---------------------------------------------------------------------------

#[repr(C)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

#[repr(C)]
struct ListHead {
    root: ListNode,
}

#[repr(C)]
struct CoordC {
    x: f64,
    y: f64,
    z: f64,
}

#[repr(C)]
struct CMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: CoordC,
    axes_r: CoordC,
    node: ListNode,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct RefHistory {
    first_clock: u64,
    last_clock: u64,
    start_position: i64,
    step_count: i32,
    interval: i32,
    add: i32,
}

impl RefHistory {
    fn zero() -> Self {
        RefHistory {
            first_clock: 0,
            last_clock: 0,
            start_position: 0,
            step_count: 0,
            interval: 0,
            add: 0,
        }
    }
}

/// `struct queue_message` (msgblock.h).
#[repr(C)]
struct RefQueueMessage {
    len: i32,
    msg: [u8; 64],
    clock: [u64; 2],
    notify_id: u64,
    node: ListNode,
}

/// `struct stepper_kinematics` (itersolve.h).
#[repr(C)]
#[allow(dead_code)]
struct RefSk {
    step_dist: f64,
    commanded_pos: f64,
    sc: *mut c_void,
    last_flush_time: f64,
    last_move_time: f64,
    tq: *mut CTrapq,
    active_flags: i32,
    gen_steps_pre_active: f64,
    gen_steps_post_active: f64,
    calc_position_cb: *mut c_void,
    post_cb: *mut c_void,
}

/// `struct trapq` (trapq.h).
#[repr(C)]
struct CTrapq {
    moves: ListHead,
    history: ListHead,
}

/// `struct pa_params` (kin_extruder.c).
#[repr(C)]
struct CPaParams {
    pressure_advance: f64,
    active_print_time: f64,
    node: ListNode,
}

/// `struct extruder_stepper` (kin_extruder.c).
#[repr(C)]
struct CExtruderStepper {
    sk: RefSk,
    pa_list: ListHead,
    half_smooth_time: f64,
    inv_half_smooth_time2: f64,
}

type CCalcCb = unsafe extern "C" fn(*mut RefSk, *mut CMove, f64) -> f64;

unsafe extern "C" {
    fn trapq_alloc() -> *mut CTrapq;
    fn trapq_free(tq: *mut CTrapq);
    #[allow(clippy::too_many_arguments)]
    fn trapq_append(
        tq: *mut CTrapq,
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
    fn extruder_stepper_alloc() -> *mut CExtruderStepper;
    fn extruder_stepper_free(sk: *mut RefSk);
    fn extruder_set_pressure_advance(
        sk: *mut RefSk,
        print_time: f64,
        pressure_advance: f64,
        smooth_time: f64,
    );
    fn itersolve_set_trapq(sk: *mut RefSk, tq: *mut CTrapq, step_dist: f64);
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_generate_steps(sk: *mut RefSk, sc: *mut c_void, flush_time: f64) -> i32;
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
    fn itersolve_get_gen_steps_pre_active(sk: *mut RefSk) -> f64;
    fn itersolve_get_gen_steps_post_active(sk: *mut RefSk) -> f64;
    fn stepcompress_alloc(msg_queue: *mut ListHead) -> *mut c_void;
    fn stepcompress_free(sc: *mut c_void);
    fn stepcompress_fill(
        sc: *mut c_void,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    );
    fn stepcompress_set_time(sc: *mut c_void, time_offset: f64, mcu_freq: f64);
    fn stepcompress_extract_old(
        sc: *mut c_void,
        p: *mut RefHistory,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> i32;
    fn stepcompress_get_oid(sc: *mut c_void) -> u32;
    fn stepcompress_get_step_dir(sc: *mut c_void) -> i32;
    fn message_queue_free(root: *mut ListHead);
}

// ---------------------------------------------------------------------------
// Reference-side helpers.
// ---------------------------------------------------------------------------

/// All `pa_params` entries of the C extruder, oldest first.
fn c_pa_list(c_es: *const CExtruderStepper) -> Vec<PaParams> {
    let root = unsafe { &(*c_es).pa_list.root } as *const ListNode as *mut ListNode;
    let mut out = Vec::new();
    let mut cur = unsafe { (*root).next };
    while cur != root {
        let pa = unsafe {
            &*((cur as *const u8).wrapping_sub(core::mem::offset_of!(CPaParams, node))
                as *const CPaParams)
        };
        out.push(PaParams {
            pressure_advance: pa.pressure_advance,
            active_print_time: pa.active_print_time,
        });
        cur = unsafe { (*cur).next };
    }
    out
}

/// The `n`-th move (0 = head sentinel) of the C trapq `moves` list.
fn c_move_at(c_tq: *const CTrapq, n: usize) -> *mut CMove {
    let root = unsafe { &(*c_tq).moves.root } as *const ListNode;
    let mut cur = unsafe { (*root).next };
    for _ in 0..n {
        cur = unsafe { (*cur).next };
    }
    unsafe {
        &mut *((cur as *mut u8).wrapping_sub(core::mem::offset_of!(CMove, node))
            as *mut CMove)
    }
}

fn c_calc_cb(c_es: *const CExtruderStepper) -> CCalcCb {
    let raw = unsafe { (*c_es).sk.calc_position_cb };
    assert!(!raw.is_null(), "C calc_position_cb is null");
    unsafe { core::mem::transmute::<*mut c_void, CCalcCb>(raw) }
}

struct RefSc {
    sc: *mut c_void,
    head: Box<ListHead>,
    scratch: Vec<RefHistory>,
}

impl RefSc {
    fn new() -> RefSc {
        let mut head = Box::new(ListHead {
            root: ListNode {
                next: core::ptr::null_mut(),
                prev: core::ptr::null_mut(),
            },
        });
        head.root.next = &mut head.root as *mut ListNode;
        head.root.prev = &mut head.root as *mut ListNode;
        let sc = unsafe { stepcompress_alloc(head.as_mut() as *mut ListHead) };
        assert!(!sc.is_null(), "stepcompress_alloc returned null");
        RefSc {
            sc,
            head,
            scratch: Vec::new(),
        }
    }

    fn drain_messages(&mut self) -> Vec<(Vec<u8>, u64, u64)> {
        let mut out = Vec::new();
        let root = &mut self.head.root as *mut ListNode;
        let mut cur = unsafe { (*root).next };
        while cur != root {
            let qm = unsafe {
                &*((cur as *const u8)
                    .wrapping_sub(core::mem::offset_of!(RefQueueMessage, node))
                    as *const RefQueueMessage)
            };
            let len = qm.len as usize;
            out.push((
                qm.msg[..len].to_vec(),
                qm.clock[1],
                qm.clock[0],
            ));
            cur = unsafe { (*cur).next };
        }
        unsafe { message_queue_free(self.head.as_mut() as *mut ListHead) };
        out
    }

    fn extract_old(&mut self, max: i32) -> Vec<RefHistory> {
        let want = max.max(0) as usize;
        if self.scratch.len() < want {
            self.scratch.resize(want, RefHistory::zero());
        }
        let res = unsafe {
            stepcompress_extract_old(self.sc, self.scratch.as_mut_ptr(), max, 0, u64::MAX)
        };
        assert!(res >= 0 && res <= max, "bad extract_old result {res}");
        self.scratch[..res as usize].to_vec()
    }
}

impl Drop for RefSc {
    fn drop(&mut self) {
        unsafe {
            message_queue_free(self.head.as_mut() as *mut ListHead);
            stepcompress_free(self.sc);
        }
    }
}

// ---------------------------------------------------------------------------
// Op stream.
// ---------------------------------------------------------------------------

/// One `trapq_append` call (accel/cruise/decel segments).
#[derive(Clone, Copy, Debug)]
struct MoveParams {
    print_time: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    start_v: f64,
    cruise_v: f64,
    accel: f64,
}

/// Extruder `trapq_append` call: `start_pos`/`axes_r` are
/// `(x, can_pa, 0)` / `(1, can_pa, 0)`, as in `extruder.py`
/// `process_move`.
#[allow(clippy::too_many_arguments)]
fn extrude(
    print_time: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    start_e: f64,
    can_pa: bool,
    start_v: f64,
    cruise_v: f64,
    accel: f64,
) -> MoveParams {
    MoveParams {
        print_time,
        accel_t,
        cruise_t,
        decel_t,
        start_pos: [start_e, 0.0, 0.0],
        axes_r: [1.0, if can_pa { 1.0 } else { 0.0 }, 0.0],
        start_v,
        cruise_v,
        accel,
    }
}

#[derive(Clone, Debug)]
enum Op {
    Append(MoveParams),
    SetPressureAdvance {
        print_time: f64,
        pressure_advance: f64,
        smooth_time: f64,
    },
    SetLastFlushTime(f64),
    CalcAll { times: Vec<f64> },
    CalcFromCoord { x: f64, y: f64, z: f64 },
    SetPosition { x: f64, y: f64, z: f64 },
    GenerateSteps(f64),
}

struct Pair {
    rust_es: Box<ExtruderStepper>,
    c_es: *mut CExtruderStepper,
    rust_tq: Arc<Mutex<Trapq>>,
    c_tq: *mut CTrapq,
    rust_sc: Option<Stepcompress>,
    ref_sc: Option<RefSc>,
}

impl Pair {
    fn new(step_dist: f64) -> Pair {
        chelper_ref::link();
        let rust_es = rust_extruder_stepper_alloc();
        let c_es = unsafe { extruder_stepper_alloc() };
        assert!(!c_es.is_null(), "C extruder_stepper_alloc returned null");
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "C trapq_alloc returned null");
        let rust_tq = Arc::new(Mutex::new(Trapq::new()));
        let mut pair = Pair {
            rust_es,
            c_es,
            rust_tq,
            c_tq,
            rust_sc: None,
            ref_sc: None,
        };
        pair.set_trapq(step_dist);
        pair
    }

    fn set_trapq(&mut self, step_dist: f64) {
        unsafe { itersolve_set_trapq(&mut (*self.c_es).sk, self.c_tq, step_dist) };
        self.rust_es
            .sk
            .set_trapq(Some(self.rust_tq.clone()), step_dist);
    }

    fn set_compressor(&mut self) {
        let ref_sc = RefSc::new();
        unsafe {
            stepcompress_fill(ref_sc.sc, 7, 3000, 24, 23);
            stepcompress_set_time(ref_sc.sc, 0.0, 1_000_000.0);
        }
        let mut rust_sc = Stepcompress::new();
        rust_sc.fill(7, 3000, 24, 23);
        rust_sc.set_time(0.0, 1_000_000.0);
        self.ref_sc = Some(ref_sc);
        self.rust_sc = Some(rust_sc);
    }

    fn rust_moves(&self) -> Vec<Move> {
        let tq = self.rust_tq.lock();
        let mut list = Vec::with_capacity(tq.moves.len() + 2);
        list.push(*tq.head_sentinel());
        list.extend(tq.moves.iter().copied());
        list.push(*tq.tail_sentinel());
        list
    }

    fn compare_pa(&self, ctx: &str) {
        let c = unsafe { &*self.c_es };
        let r = &*self.rust_es;
        let c_list = c_pa_list(self.c_es);
        assert_eq!(c_list.len(), r.pa_list.len(), "{ctx}: pa_list length");
        for (i, (cl, rl)) in c_list.iter().zip(r.pa_list.iter()).enumerate() {
            assert_eq!(
                cl.pressure_advance.to_bits(),
                rl.pressure_advance.to_bits(),
                "{ctx}: pa_list[{i}].pressure_advance C={:?} Rust={:?}",
                cl.pressure_advance,
                rl.pressure_advance
            );
            assert_eq!(
                cl.active_print_time.to_bits(),
                rl.active_print_time.to_bits(),
                "{ctx}: pa_list[{i}].active_print_time C={:?} Rust={:?}",
                cl.active_print_time,
                rl.active_print_time
            );
        }
        assert_eq!(
            c.half_smooth_time.to_bits(),
            r.half_smooth_time.to_bits(),
            "{ctx}: half_smooth_time C={:?} Rust={:?}",
            c.half_smooth_time,
            r.half_smooth_time
        );
        assert_eq!(
            c.inv_half_smooth_time2.to_bits(),
            r.inv_half_smooth_time2.to_bits(),
            "{ctx}: inv_half_smooth_time2 C={:?} Rust={:?}",
            c.inv_half_smooth_time2,
            r.inv_half_smooth_time2
        );
        assert_eq!(
            unsafe { itersolve_get_gen_steps_pre_active(&mut (*self.c_es).sk) }.to_bits(),
            r.sk.gen_steps_pre_active.to_bits(),
            "{ctx}: gen_steps_pre_active diverged"
        );
        assert_eq!(
            unsafe { itersolve_get_gen_steps_post_active(&mut (*self.c_es).sk) }.to_bits(),
            r.sk.gen_steps_post_active.to_bits(),
            "{ctx}: gen_steps_post_active diverged"
        );
        assert_eq!(
            c.sk.active_flags, r.sk.active_flags,
            "{ctx}: active_flags C={} Rust={}",
            c.sk.active_flags, r.sk.active_flags
        );
        assert_eq!(
            c.sk.last_flush_time.to_bits(),
            r.sk.last_flush_time.to_bits(),
            "{ctx}: last_flush_time diverged"
        );
    }

    fn compare_sc(&mut self, ctx: &str) {
        let Some(rust_sc) = self.rust_sc.as_mut() else {
            return;
        };
        let rust_msgs: Vec<(Vec<u8>, u64, u64)> = rust_sc
            .drain_messages()
            .iter()
            .map(|m: &QueuedMessage| (m.bytes.clone(), m.req_clock, m.min_clock))
            .collect();
        let ref_msgs = self
            .ref_sc
            .as_mut()
            .expect("ref compressor pairs rust_sc")
            .drain_messages();
        assert_eq!(rust_msgs, ref_msgs, "{ctx}: messages diverged");
        const HISTORY_MAX: i32 = 8192;
        let rust_hist = rust_sc.extract_old(HISTORY_MAX, 0, u64::MAX);
        let ref_hist = self
            .ref_sc
            .as_mut()
            .expect("ref compressor pairs rust_sc")
            .extract_old(HISTORY_MAX);
        assert_eq!(
            rust_hist.len(),
            ref_hist.len(),
            "{ctx}: history length diverged (Rust={} C={})",
            rust_hist.len(),
            ref_hist.len()
        );
        for (i, (rh, ch)) in rust_hist.iter().zip(ref_hist.iter()).enumerate() {
            assert_eq!(
                (
                    rh.first_clock,
                    rh.last_clock,
                    rh.start_position,
                    rh.step_count,
                    rh.interval,
                    rh.add
                ),
                (
                    ch.first_clock,
                    ch.last_clock,
                    ch.start_position,
                    ch.step_count,
                    ch.interval,
                    ch.add
                ),
                "{ctx}: history[{i}] diverged"
            );
        }
        assert_eq!(
            rust_sc.get_step_dir(),
            unsafe { stepcompress_get_step_dir(self.ref_sc.as_ref().unwrap().sc) },
            "{ctx}: step dir diverged"
        );
        assert_eq!(
            rust_sc.get_oid(),
            unsafe { stepcompress_get_oid(self.ref_sc.as_ref().unwrap().sc) },
            "{ctx}: oid diverged"
        );
    }

    fn compare_all(&mut self, ctx: &str) {
        self.compare_pa(ctx);
        assert_eq!(
            unsafe { itersolve_get_commanded_pos(&mut (*self.c_es).sk) }.to_bits(),
            self.rust_es.sk.commanded_pos.to_bits(),
            "{ctx}: commanded_pos diverged"
        );
        self.compare_sc(ctx);
    }

    fn run_op(&mut self, op: &Op, ctx: &str) {
        match op {
            Op::Append(p) => {
                unsafe {
                    trapq_append(
                        self.c_tq,
                        p.print_time,
                        p.accel_t,
                        p.cruise_t,
                        p.decel_t,
                        p.start_pos[0],
                        p.start_pos[1],
                        p.start_pos[2],
                        p.axes_r[0],
                        p.axes_r[1],
                        p.axes_r[2],
                        p.start_v,
                        p.cruise_v,
                        p.accel,
                    )
                };
                self.rust_tq.lock().append(
                    p.print_time,
                    p.accel_t,
                    p.cruise_t,
                    p.decel_t,
                    p.start_pos[0],
                    p.start_pos[1],
                    p.start_pos[2],
                    p.axes_r[0],
                    p.axes_r[1],
                    p.axes_r[2],
                    p.start_v,
                    p.cruise_v,
                    p.accel,
                );
            }
            Op::SetPressureAdvance {
                print_time,
                pressure_advance,
                smooth_time,
            } => {
                unsafe {
                    extruder_set_pressure_advance(
                        &mut (*self.c_es).sk,
                        *print_time,
                        *pressure_advance,
                        *smooth_time,
                    )
                };
                self.rust_es.set_pressure_advance(
                    *print_time,
                    *pressure_advance,
                    *smooth_time,
                );
            }
            Op::SetLastFlushTime(t) => {
                unsafe { (*self.c_es).sk.last_flush_time = *t };
                self.rust_es.sk.last_flush_time = *t;
            }
            Op::CalcAll { times } => {
                // Real moves only (both sentinels excluded): the
                // sentinels are covered by dedicated tests. A time is
                // sampled only when the smoothing window does not
                // extend before the first real move: in C the
                // prev-walk would step past the head sentinel into
                // heap memory there (see module docs).
                let cb = c_calc_cb(self.c_es);
                let moves = self.rust_moves();
                let hst = self.rust_es.half_smooth_time;
                for idx in 1..moves.len().saturating_sub(1) {
                    let mv = &moves[idx];
                    // Sanity: Rust and C move lists agree bit for bit.
                    let cm = c_move_at(self.c_tq, idx);
                    assert_eq!(
                        unsafe { (*cm).print_time }.to_bits(),
                        mv.print_time.to_bits(),
                        "{ctx}: move[{idx}].print_time diverged"
                    );
                    assert_eq!(
                        unsafe { (*cm).move_t }.to_bits(),
                        mv.move_t.to_bits(),
                        "{ctx}: move[{idx}].move_t diverged"
                    );
                    assert_eq!(
                        unsafe { (*cm).start_v }.to_bits(),
                        mv.start_v.to_bits(),
                        "{ctx}: move[{idx}].start_v diverged"
                    );
                    assert_eq!(
                        unsafe { (*cm).half_accel }.to_bits(),
                        mv.half_accel.to_bits(),
                        "{ctx}: move[{idx}].half_accel diverged"
                    );
                    assert_eq!(
                        unsafe { (*cm).start_pos.x }.to_bits(),
                        mv.start_pos.x.to_bits(),
                        "{ctx}: move[{idx}].start_pos.x diverged"
                    );
                    assert_eq!(
                        unsafe { (*cm).axes_r.y }.to_bits(),
                        mv.axes_r.y.to_bits(),
                        "{ctx}: move[{idx}].axes_r.y diverged"
                    );
                    let prev_total: f64 =
                        moves[..idx].iter().map(|p| p.move_t).sum();
                    for t in times {
                        if t + prev_total < hst {
                            continue;
                        }
                        let c = unsafe { cb(&mut (*self.c_es).sk, cm, *t) };
                        let r = self.rust_es.calc_position(mv, Some(idx), *t);
                        assert_eq!(
                            c.to_bits(),
                            r.to_bits(),
                            "{ctx}: calc pos move[{idx}] t={t:?} C={c:?} Rust={r:?}"
                        );
                    }
                }
            }
            Op::CalcFromCoord { x, y, z } => {
                let c =
                    unsafe { itersolve_calc_position_from_coord(&mut (*self.c_es).sk, *x, *y, *z) };
                let r = self.rust_es.sk.calc_position_from_coord(*x, *y, *z);
                assert_eq!(c.to_bits(), r.to_bits(), "{ctx}: calc_from_coord diverged");
            }
            Op::SetPosition { x, y, z } => {
                unsafe { itersolve_set_position(&mut (*self.c_es).sk, *x, *y, *z) };
                self.rust_es.sk.set_position(*x, *y, *z);
            }
            Op::GenerateSteps(t) => {
                let c = unsafe {
                    itersolve_generate_steps(
                        &mut (*self.c_es).sk,
                        self.ref_sc.as_ref().expect("compressor set").sc,
                        *t,
                    )
                };
                let rust_sc =
                    self.rust_sc.as_mut().expect("compressor set");
                let r = self.rust_es.sk.generate_steps(rust_sc, *t);
                assert_eq!(c, r, "{ctx}: generate_steps return diverged");
            }
        }
        self.compare_all(ctx);
    }

    fn run_stream(&mut self, ctx: &str, ops: &[Op]) {
        self.compare_all(&format!("{ctx}#init"));
        for (i, op) in ops.iter().enumerate() {
            self.run_op(op, &format!("{ctx}#{i}"));
        }
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe {
            extruder_stepper_free(&mut (*self.c_es).sk);
            trapq_free(self.c_tq);
        }
    }
}

// ---------------------------------------------------------------------------
// Scenarios.
// ---------------------------------------------------------------------------

/// Alloc defaults: one zeroed `pa_params`, `active_flags == AF_X`,
/// zeroed smoothing fields and gen-steps fields.
#[test]
fn alloc_defaults() {
    let mut pair = Pair::new(0.025);
    pair.compare_all("alloc");
    assert_eq!(
        c_pa_list(pair.c_es),
        vec![PaParams::default()],
        "C pa_list after alloc"
    );
    assert_eq!(pair.rust_es.pa_list.len(), 1, "Rust pa_list after alloc");
    assert_eq!(
        pair.rust_es.sk.active_flags, AF_X,
        "Rust active_flags"
    );
    assert_eq!(pair.rust_es.half_smooth_time.to_bits(), 0.0f64.to_bits());
    assert_eq!(
        pair.rust_es.inv_half_smooth_time2.to_bits(),
        0.0f64.to_bits()
    );
    // Direct callback on the empty queue fast path (no PA yet).
    let cb = c_calc_cb(pair.c_es);
    let m = Move::default();
    let c = unsafe { cb(&mut (*pair.c_es).sk, &mut *c_move_at(pair.c_tq, 0), 0.25) };
    let r = pair.rust_es.calc_position(&m, None, 0.25);
    // C evaluated on the head sentinel (all fields zero), Rust on an
    // equivalent synthetic move: both are start_pos.x + distance.
    assert_eq!(c.to_bits(), r.to_bits());
}

/// Pressure-advance state machine: append, retain, stale inverse,
/// cleanup, and the zero-smooth early return.
#[test]
fn pressure_advance_state_machine() {
    let mut pair = Pair::new(0.025);
    let ops = [
        Op::SetPressureAdvance {
            print_time: 5.0,
            pressure_advance: 0.03,
            smooth_time: 0.04,
        },
        Op::SetPressureAdvance {
            print_time: 10.0,
            pressure_advance: 0.05,
            smooth_time: 0.04,
        },
        // Same PA, different smooth_time: no new entry, inv updated.
        Op::SetPressureAdvance {
            print_time: 12.0,
            pressure_advance: 0.05,
            smooth_time: 0.08,
        },
        // Zero smooth_time: cleanup runs, no append, inv stays stale.
        Op::SetLastFlushTime(15.0),
        Op::SetPressureAdvance {
            print_time: 20.0,
            pressure_advance: 0.07,
            smooth_time: 0.0,
        },
        // Full cleanup, then append.
        Op::SetPressureAdvance {
            print_time: 30.0,
            pressure_advance: 0.02,
            smooth_time: 0.04,
        },
        Op::SetPressureAdvance {
            print_time: 40.0,
            pressure_advance: 0.04,
            smooth_time: 0.04,
        },
        Op::SetPressureAdvance {
            print_time: 50.0,
            pressure_advance: 0.06,
            smooth_time: 0.04,
        },
        // Roll the flush forward: entries at 30/40 are cleaned up.
        Op::SetLastFlushTime(100.0),
        Op::SetPressureAdvance {
            print_time: 110.0,
            pressure_advance: 0.08,
            smooth_time: 0.04,
        },
        // Back to disabled: gen-steps cleared, inv stale again.
        Op::SetPressureAdvance {
            print_time: 120.0,
            pressure_advance: 0.08,
            smooth_time: 0.0,
        },
    ];
    pair.run_stream("pa-state", &ops);
    // Final list: cleanup above deleted everything but the 50.0
    // entry (its successor at 110.0 is fresh); the zero-smooth
    // call appended nothing.
    let c_list = c_pa_list(pair.c_es);
    assert_eq!(c_list.len(), 2, "C pa_list length");
    assert_eq!(c_list[0].pressure_advance.to_bits(), 0.06f64.to_bits());
    assert_eq!(c_list[0].active_print_time.to_bits(), 50.0f64.to_bits());
    assert_eq!(c_list[1].pressure_advance.to_bits(), 0.08f64.to_bits());
    assert_eq!(c_list[1].active_print_time.to_bits(), 110.0f64.to_bits());
}

/// Partial cleanup: the loop stops at the first entry whose
/// successor is fresh.
#[test]
fn partial_cleanup() {
    let mut pair = Pair::new(0.025);
    let ops = [
        Op::SetPressureAdvance {
            print_time: 5.0,
            pressure_advance: 0.02,
            smooth_time: 0.04,
        },
        Op::SetPressureAdvance {
            print_time: 10.0,
            pressure_advance: 0.03,
            smooth_time: 0.04,
        },
        Op::SetPressureAdvance {
            print_time: 20.0,
            pressure_advance: 0.04,
            smooth_time: 0.04,
        },
        Op::SetLastFlushTime(15.0),
        // cleanup_time = 15 - 0.02 = 14.98: the 0.0 and 5.0 entries
        // go, the 10.0 entry stays (successor at 20.0 is fresh).
        Op::SetPressureAdvance {
            print_time: 30.0,
            pressure_advance: 0.05,
            smooth_time: 0.04,
        },
    ];
    pair.run_stream("pa-clean", &ops);
    let c_list = c_pa_list(pair.c_es);
    let times: Vec<u64> = c_list
        .iter()
        .map(|p| p.active_print_time.to_bits())
        .collect();
    assert_eq!(times, vec![10.0f64.to_bits(), 20.0f64.to_bits(), 30.0f64.to_bits()]);
}

/// Position callback over cruise, trapezoid, extrude-only,
/// negative-velocity and gapped moves; window crossings within one
/// move only (hst small).
#[test]
fn calc_positions_basic() {
    let mut pair = Pair::new(0.01);
    let times = vec![0.0, 0.05, 0.1, 0.3, 0.5, 0.7];
    let ops = [
        Op::CalcAll { times: times.clone() },
        Op::SetPressureAdvance {
            print_time: 0.0,
            pressure_advance: 0.03,
            smooth_time: 0.04,
        },
        Op::CalcAll { times: times.clone() },
        Op::Append(extrude(1.0, 0.0, 1.0, 0.0, 0.0, true, 0.0, 2.0, 0.0)),
        Op::Append(extrude(2.0, 0.2, 0.6, 0.2, 0.5, true, 0.0, 3.0, 5.0)),
        Op::Append(extrude(3.0, 0.2, 0.6, 0.2, 1.2, false, 0.0, 3.0, 5.0)),
        Op::Append(extrude(4.0, 0.2, 0.6, 0.2, 1.8, true, 3.0, 3.0, -5.0)),
        Op::Append(extrude(10.0, 0.1, 0.5, 0.1, 2.4, true, 0.0, 1.5, 2.0)),
        Op::CalcAll { times: times.clone() },
        Op::Append(extrude(15.0, 0.0, 0.5, 0.0, -1.0, true, -1.0, -1.0, 0.0)),
        Op::CalcAll { times },
        // Synthetic-move path (window inside the 1000s move).
        Op::CalcFromCoord {
            x: 3.5,
            y: 1.25,
            z: -2.0,
        },
        Op::SetPosition {
            x: 3.5,
            y: 1.25,
            z: -2.0,
        },
    ];
    pair.run_stream("calc-basic", &ops);
}

/// Large smooth time: the window spans several short moves on both
/// sides, crosses null moves and the tail sentinel.
#[test]
fn calc_positions_window_crossing() {
    let mut pair = Pair::new(0.01);
    let times = vec![0.0, 0.1, 0.2, 0.39, 0.4];
    let ops = [
        Op::Append(extrude(10.0, 0.0, 1.0, 0.0, 0.0, true, 0.0, 1.0, 0.0)),
        Op::Append(extrude(11.0, 0.0, 0.2, 0.0, 0.4, true, 0.0, 2.0, 0.0)),
        Op::Append(extrude(11.2, 0.0, 0.2, 0.0, 0.8, true, 0.0, 2.0, 0.0)),
        Op::Append(extrude(11.4, 0.0, 0.2, 0.0, 1.2, true, 0.0, -2.0, 0.0)),
        Op::Append(extrude(20.0, 0.0, 0.2, 0.0, 2.0, true, 0.0, 2.0, 0.0)),
        Op::Append(extrude(20.2, 0.0, 0.2, 0.0, 2.4, true, 0.0, 2.0, 0.0)),
        Op::SetPressureAdvance {
            print_time: 0.0,
            pressure_advance: 0.04,
            smooth_time: 1.0,
        },
        Op::CalcAll { times: times.clone() },
        // PA zero (active at 25.0, before the new move at 30.0)
        // but smoothing on: the average still changes the
        // position of accelerating moves.
        Op::SetPressureAdvance {
            print_time: 25.0,
            pressure_advance: 0.0,
            smooth_time: 1.0,
        },
        Op::Append(extrude(30.0, 0.5, 1.0, 0.5, 5.0, true, 0.0, 4.0, 3.0)),
        Op::CalcAll { times },
    ];
    pair.run_stream("calc-cross", &ops);
}

/// Pressure-advance selection: three moves at print times 2, 7 and
/// 12 pick up the entries at 0.0, 5.0 and 10.0 respectively; moves
/// with the can-PA flag clear ignore the entries.
#[test]
fn pressure_advance_selection() {
    let mut pair = Pair::new(0.01);
    let times = vec![0.0, 0.2, 0.5, 0.9];
    let ops = [
        Op::Append(extrude(2.0, 0.3, 0.4, 0.3, 0.0, true, 0.0, 2.0, 4.0)),
        Op::Append(extrude(7.0, 0.3, 0.4, 0.3, 1.0, true, 0.0, 2.0, 4.0)),
        Op::Append(extrude(12.0, 0.3, 0.4, 0.3, 2.0, false, 0.0, 2.0, 4.0)),
        Op::Append(extrude(12.5, 0.3, 0.4, 0.3, 3.0, true, 0.0, 2.0, 4.0)),
        Op::SetPressureAdvance {
            print_time: 5.0,
            pressure_advance: 0.02,
            smooth_time: 0.04,
        },
        Op::SetPressureAdvance {
            print_time: 10.0,
            pressure_advance: 0.05,
            smooth_time: 0.04,
        },
        Op::CalcAll { times },
    ];
    pair.run_stream("pa-select", &ops);
}

/// End-to-end step generation with the extruder callback.
#[test]
fn generate_steps_stream() {
    let mut pair = Pair::new(0.02);
    pair.set_compressor();
    let ops = [
        Op::Append(extrude(1.0, 0.0, 2.0, 0.0, 0.0, true, 0.0, 0.5, 0.0)),
        Op::Append(extrude(3.0, 0.3, 1.0, 0.3, 1.0, true, 0.0, 1.0, 2.0)),
        Op::Append(extrude(5.0, 0.3, 1.0, 0.3, 2.5, true, 0.0, 1.0, -2.0)),
        Op::SetPressureAdvance {
            print_time: 0.0,
            pressure_advance: 0.03,
            smooth_time: 0.04,
        },
        Op::GenerateSteps(2.5),
        Op::GenerateSteps(3.5),
        Op::GenerateSteps(5.0),
        Op::GenerateSteps(10.0),
        // Flush far past the tail sentinel.
        Op::GenerateSteps(100.0),
        Op::CalcFromCoord {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    ];
    pair.run_stream("gen-steps", &ops);
}

/// Deterministic pseudo-random sweep over pressure-advance
/// parameters and move geometry.
#[test]
fn pseudorandom_sweep() {
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            self.0 >> 11
        }
        fn range(&mut self, lo: f64, hi: f64) -> f64 {
            let frac = (self.next() & 0x1f_ffff) as f64 / 0x20_0000 as f64;
            lo + frac * (hi - lo)
        }
    }
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut pair = Pair::new(0.01);
    pair.set_compressor();
    for k in 0..24 {
        let print_time = rng.range(0.0, 40.0) + (k as f64) * 10.0;
        let pa = rng.range(0.0, 0.08);
        let smooth = if k % 5 == 4 {
            0.0
        } else {
            rng.range(0.0, 0.15)
        };
        pair.run_op(
            &Op::SetPressureAdvance {
                print_time,
                pressure_advance: pa,
                smooth_time: smooth,
            },
            &format!("random#set-pa-{k}"),
        );
    }
    let mut t = 50.0;
    for k in 0..20 {
        let cruise_t = rng.range(0.2, 1.5);
        let v = rng.range(-2.0, 2.0);
        let a = rng.range(-4.0, 4.0);
        let can_pa = !rng.next().is_multiple_of(3);
        pair.run_op(
            &Op::Append(extrude(
                t,
                0.05 * cruise_t,
                cruise_t,
                0.05 * cruise_t,
                (k as f64) * 0.3,
                can_pa,
                0.0,
                v,
                a,
            )),
            &format!("random#append-{k}"),
        );
        t += rng.range(2.0, 3.0);
    }
    let times = vec![0.0, 0.25, 0.5, 1.0];
    pair.run_op(&Op::CalcAll { times }, "random#calc");
    pair.run_op(&Op::GenerateSteps(1e3), "random#gen");
    pair.run_op(
        &Op::CalcFromCoord {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        "random#coord",
    );
}

/// Tail sentinel move as the current move (post-active steps past
/// the last real move): the window integrates over real moves.
#[test]
fn tail_sentinel_move() {
    let mut pair = Pair::new(0.01);
    pair.set_compressor();
    let ops = [
        Op::Append(extrude(1.0, 0.0, 1.0, 0.0, 0.0, true, 0.0, 1.0, 0.0)),
        Op::SetPressureAdvance {
            print_time: 0.0,
            pressure_advance: 0.05,
            smooth_time: 0.1,
        },
        Op::GenerateSteps(5.0),
    ];
    pair.run_stream("tail", &ops);
    // The tail sentinel itself as a position source: build the
    // equivalent Rust move and call the C callback on the C tail.
    // Times stay at least `hst` (0.05) into the tail so the smoothing
    // window does not extend before the head sentinel, where the C
    // prev-walk steps past the head into heap memory.
    let moves = pair.rust_moves();
    let tail = moves.last().expect("tail exists");
    let cb = c_calc_cb(pair.c_es);
    let c_tail = c_move_at(pair.c_tq, moves.len() - 1);
    // The tail print_time is still the 0.0 "needs update" marker on
    // both sides: trapq_add_move writes it and nothing calls
    // trapq_check_sentinels before generate_steps.
    assert_eq!(
        unsafe { (*c_tail).print_time }.to_bits(),
        tail.print_time.to_bits(),
        "tail print_time C={:?} Rust={:?}",
        unsafe { (*c_tail).print_time },
        tail.print_time
    );
    assert_eq!(
        unsafe { (*c_tail).move_t }.to_bits(),
        tail.move_t.to_bits(),
        "tail move_t diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).start_v }.to_bits(),
        tail.start_v.to_bits(),
        "tail start_v diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).half_accel }.to_bits(),
        tail.half_accel.to_bits(),
        "tail half_accel diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).start_pos.x }.to_bits(),
        tail.start_pos.x.to_bits(),
        "tail start_pos.x diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).start_pos.y }.to_bits(),
        tail.start_pos.y.to_bits(),
        "tail start_pos.y diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).start_pos.z }.to_bits(),
        tail.start_pos.z.to_bits(),
        "tail start_pos.z diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).axes_r.x }.to_bits(),
        tail.axes_r.x.to_bits(),
        "tail axes_r.x diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).axes_r.y }.to_bits(),
        tail.axes_r.y.to_bits(),
        "tail axes_r.y diverged"
    );
    assert_eq!(
        unsafe { (*c_tail).axes_r.z }.to_bits(),
        tail.axes_r.z.to_bits(),
        "tail axes_r.z diverged"
    );
    for t in [0.06f64, 0.1, 0.5] {
        let c = unsafe { cb(&mut (*pair.c_es).sk, c_tail, t) };
        let r = extruder_calc_position(&pair.rust_es.sk, tail, Some(moves.len() - 1), t);
        assert_eq!(c.to_bits(), r.to_bits(), "tail pos t={t:?}");
    }
}
