//! Differential parity tests for `chelper_motion::kin_generic`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the frozen `klippy/chelper/` sources) with identical
//! inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - the stepper position on direct moves across a wide grid of
//!   coefficients and move parameters, through the C calc callback
//!   (called with the same move bytes the Rust side gets) and through
//!   `itersolve_calc_position_from_coord`,
//! - `active_flags` after `generic_cartesian_stepper_alloc` and after
//!   every `generic_cartesian_stepper_set_coeffs`, plus
//!   `itersolve_is_active_axis` per axis,
//! - `commanded_pos` after `itersolve_set_position`, and
//! - the full `itersolve_generate_steps` step stream end to end (emitted
//!   step commands, history, pending direction, and mutated `sk` fields)
//!   for a generic stepper.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_generic`

use chelper_motion::itersolve::{AF_X, AF_Y, AF_Z};
use chelper_motion::kin_generic::{
    generic_cartesian_stepper_alloc as rust_generic_cartesian_stepper_alloc,
    generic_cartesian_stepper_calc_position, GenericCartesianStepper,
};
use chelper_motion::stepcompress::{QueuedMessage, Stepcompress};
use chelper_motion::trapq::{Coord, Move, Trapq};
use core::ffi::c_void;

use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Reference C declarations (layouts from itersolve.h / trapq.h /
// stepcompress.h / msgblock.h / list.h).
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

#[repr(C)]
struct ListHead {
    root: ListNode,
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

/// `struct pull_history_steps` (stepcompress.h).
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

/// `struct stepper_kinematics` (itersolve.h), x86_64 layout.
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
enum CTrapq {}

/// `struct move` (trapq.h): the four scalars, the two coords, the list
/// node.
#[repr(C)]
#[derive(Clone, Copy)]
struct CMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    node: ListNode,
}

impl CMove {
    fn from_move(m: &Move) -> CMove {
        CMove {
            print_time: m.print_time,
            move_t: m.move_t,
            start_v: m.start_v,
            half_accel: m.half_accel,
            start_pos: [m.start_pos.x, m.start_pos.y, m.start_pos.z],
            axes_r: [m.axes_r.x, m.axes_r.y, m.axes_r.z],
            node: ListNode {
                next: core::ptr::null_mut(),
                prev: core::ptr::null_mut(),
            },
        }
    }
}

type RefCalcCb = unsafe extern "C" fn(*mut RefSk, *mut CMove, f64) -> f64;

unsafe extern "C" {
    fn generic_cartesian_stepper_alloc(a_x: f64, a_y: f64, a_z: f64) -> *mut RefSk;
    fn generic_cartesian_stepper_set_coeffs(sk: *mut RefSk, a_x: f64, a_y: f64, a_z: f64);
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn itersolve_set_position(sk: *mut RefSk, x: f64, y: f64, z: f64);
    fn itersolve_get_commanded_pos(sk: *mut RefSk) -> f64;
    fn itersolve_is_active_axis(sk: *mut RefSk, axis: i8) -> i32;
    fn trapq_alloc() -> *mut CTrapq;
    fn trapq_free(tq: *mut CTrapq);
    fn trapq_check_sentinels(tq: *mut CTrapq);
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
    fn stepcompress_get_step_dir(sc: *mut c_void) -> i32;
    fn message_queue_free(root: *mut ListHead);
    fn itersolve_generate_steps(sk: *mut RefSk, sc: *mut c_void, flush_time: f64) -> i32;
    fn itersolve_set_trapq(sk: *mut RefSk, tq: *mut CTrapq, step_dist: f64);
    fn free(ptr: *mut c_void);
}

// ---------------------------------------------------------------------------
// Reference-side Stepcompress handle (as in the stepcompress tests).
// ---------------------------------------------------------------------------

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
            out.push((qm.msg[..len].to_vec(), qm.clock[1], qm.clock[0]));
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
// Helpers.
// ---------------------------------------------------------------------------

fn bits(value: f64) -> u64 {
    value.to_bits()
}

/// An active-axis expected flag value for one axis' coefficient.
fn flag_for(coeff: f64, flag: i32) -> i32 {
    if coeff != 0.0 { flag } else { 0 }
}

/// Allocates the Rust and C steppers with the same coefficients.
fn alloc_pair(a_x: f64, a_y: f64, a_z: f64) -> (Box<GenericCartesianStepper>, *mut RefSk) {
    chelper_ref::link();
    let rust_stepper = rust_generic_cartesian_stepper_alloc(a_x, a_y, a_z);
    let c_sk = unsafe { generic_cartesian_stepper_alloc(a_x, a_y, a_z) };
    assert!(!c_sk.is_null(), "generic_cartesian_stepper_alloc null");
    (rust_stepper, c_sk)
}

/// Frees just the C stepper (the C `alloc` uses `malloc`).
fn free_c_sk(c_sk: *mut RefSk) {
    unsafe { free(c_sk as *mut c_void) };
}

/// Checks `active_flags` and `is_active_axis` on both sides after the
/// Rust and C steppers were given `coeffs`.
fn check_flags(rust_stepper: &GenericCartesianStepper, c_sk: *mut RefSk, coeffs: [f64; 3], ctx: &str) {
    let expect = flag_for(coeffs[0], AF_X) | flag_for(coeffs[1], AF_Y) | flag_for(coeffs[2], AF_Z);
    assert_eq!(
        rust_stepper.sk.active_flags, expect,
        "{ctx}: Rust active_flags {} != expected {expect}",
        rust_stepper.sk.active_flags
    );
    let c_flags = unsafe { (*c_sk).active_flags };
    assert_eq!(
        c_flags, expect,
        "{ctx}: C active_flags {c_flags} != expected {expect}"
    );
    for ((axis, _flag, coeff), name) in [(b'x', AF_X, coeffs[0]), (b'y', AF_Y, coeffs[1]), (b'z', AF_Z, coeffs[2])]
        .into_iter()
        .zip(["x", "y", "z"])
    {
        let expect_active = i32::from(coeff != 0.0);
        let rust_active = rust_stepper.sk.is_active_axis(axis);
        assert_eq!(
            rust_active, expect_active,
            "{ctx}: Rust is_active_axis({name}) {rust_active} != {expect_active}"
        );
        let c_active = unsafe { itersolve_is_active_axis(c_sk, axis as i8) };
        assert_eq!(
            c_active, expect_active,
            "{ctx}: C is_active_axis({name}) {c_active} != {expect_active}"
        );
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn alloc_active_flags_all_combinations() {
    let values = [0.0, 1.5];
    for a_x in values {
        for a_y in values {
            for a_z in values {
                let coeffs = [a_x, a_y, a_z];
                let (rust_stepper, c_sk) = alloc_pair(a_x, a_y, a_z);
                check_flags(&rust_stepper, c_sk, coeffs, &format!("alloc {coeffs:?}"));
                // Other `sk` fields start zeroed on both sides.
                assert_eq!(bits(rust_stepper.sk.step_dist), 0, "step_dist not zero");
                assert_eq!(
                    bits(rust_stepper.sk.commanded_pos),
                    bits(unsafe { (*c_sk).commanded_pos }),
                    "commanded_pos differs"
                );
                assert_eq!(
                    bits(rust_stepper.sk.last_flush_time),
                    bits(unsafe { (*c_sk).last_flush_time }),
                    "last_flush_time differs"
                );
                assert_eq!(
                    bits(rust_stepper.sk.last_move_time),
                    bits(unsafe { (*c_sk).last_move_time }),
                    "last_move_time differs"
                );
                assert_eq!(
                    bits(rust_stepper.sk.gen_steps_pre_active),
                    bits(unsafe { (*c_sk).gen_steps_pre_active }),
                    "gen_steps_pre_active differs"
                );
                assert_eq!(
                    bits(rust_stepper.sk.gen_steps_post_active),
                    bits(unsafe { (*c_sk).gen_steps_post_active }),
                    "gen_steps_post_active differs"
                );
                assert!(rust_stepper.sk.tq.is_none(), "tq not None");
                free_c_sk(c_sk);
            }
        }
    }
}

#[test]
fn set_coeffs_active_flags_sequences() {
    let (mut rust_stepper, c_sk) = alloc_pair(0.0, 0.0, 0.0);
    check_flags(&rust_stepper, c_sk, [0.0, 0.0, 0.0], "initial");
    let sequences: [[f64; 3]; 12] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 0.0, 0.0],
        [-2.5, -2.5, -2.5],
        [-0.0, -0.0, -0.0],
        [0.0, 0.0, -0.0],
        [3.0, 0.0, -3.0],
        [0.5, -0.25, 0.125],
        [f64::NAN, 0.0, 0.0],
        [0.0, f64::NAN, f64::INFINITY],
    ];
    for coeffs in sequences {
        rust_stepper.set_coeffs(coeffs[0], coeffs[1], coeffs[2]);
        unsafe { generic_cartesian_stepper_set_coeffs(c_sk, coeffs[0], coeffs[1], coeffs[2]) };
        let ctx = format!("set_coeffs {coeffs:?}");
        // Stored coefficients match bit for bit (NaN payload = Rust `NAN`).
        assert_eq!(bits(rust_stepper.a.x), bits(coeffs[0]), "{ctx}: a.x");
        assert_eq!(bits(rust_stepper.a.y), bits(coeffs[1]), "{ctx}: a.y");
        assert_eq!(bits(rust_stepper.a.z), bits(coeffs[2]), "{ctx}: a.z");
        check_flags(&rust_stepper, c_sk, coeffs, &ctx);
        // The calc callback stored in `sk` is this module's function.
        assert_eq!(
            rust_stepper.sk.calc_position_cb as *const () as usize,
            generic_cartesian_stepper_calc_position as *const () as usize,
            "{ctx}: calc_position_cb identity"
        );
    }
    free_c_sk(c_sk);
}

#[test]
fn calc_from_coord_differential() {
    let coeff_sets: [[f64; 3]; 10] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.5, -0.5, 2.0],
        [0.0, 0.0, 0.0],
        [-1.0, -2.0, -3.0],
        [std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2, -1.0],
        [1e300, 1e300, 1e300],
        [f64::NAN, f64::INFINITY, -f64::INFINITY],
    ];
    let coords: [[f64; 3]; 12] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 0.0, 3.0],
        [10.0, 20.0, 30.0],
        [-5.0, -5.0, -5.0],
        [-0.0, -0.0, -0.0],
        [123.456, -789.012, 0.0001],
        [1e300, 1e300, 1e300],
        [f64::INFINITY, -f64::INFINITY, 0.0],
        [5e-324, -5e-324, 1e-300],
        [f64::NAN, 0.0, 0.0],
    ];
    for coeffs in coeff_sets {
        let (rust_stepper, c_sk) = alloc_pair(coeffs[0], coeffs[1], coeffs[2]);
        for coord in coords {
            let ctx = format!("coeffs {coeffs:?} coord {coord:?}");
            let c_result = unsafe { itersolve_calc_position_from_coord(c_sk, coord[0], coord[1], coord[2]) };
            let rust_result = rust_stepper.sk.calc_position_from_coord(coord[0], coord[1], coord[2]);
            assert_eq!(
                bits(rust_result),
                bits(c_result),
                "{ctx}: Rust {rust_result:?} C {c_result:?}"
            );
        }
        free_c_sk(c_sk);
    }
}

#[test]
fn direct_move_calc_differential() {
    // Any move byte-for-byte, at any move_time (including past the move
    // end, before zero, and non-finite times).
    let coeff_sets: [[f64; 3]; 6] = [
        [1.0, 0.0, 0.0],
        [0.25, -0.75, 1.5],
        [0.0, 0.0, 0.0],
        [-2.0, 3.0, -0.5],
        [1e200, -1e200, 1e-200],
        [f64::INFINITY, 1.0, 0.0],
    ];
    let base_moves: [Move; 6] = [
        Move {
            print_time: 10.0,
            move_t: 2.0,
            start_v: 0.0,
            half_accel: 50.0,
            start_pos: Coord::new(0.0, 0.0, 0.0),
            axes_r: Coord::new(1.0, 0.0, 0.0),
        },
        Move {
            print_time: -1.0,
            move_t: 0.0,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos: Coord::new(-1.0, 2.0, -3.0),
            axes_r: Coord::new(0.0, 0.0, 0.0),
        },
        Move {
            print_time: 100.25,
            move_t: 3.5,
            start_v: 10.0,
            half_accel: -20.0,
            start_pos: Coord::new(1.5, -2.5, 3.5),
            axes_r: Coord::new(0.6, 0.8, -0.0),
        },
        Move {
            print_time: 0.0,
            move_t: 1.0,
            start_v: -5.0,
            half_accel: 1000.0,
            start_pos: Coord::new(100.0, 100.0, 100.0),
            axes_r: Coord::new(-1.0, 1.0, 0.5),
        },
        Move {
            print_time: 5.0,
            move_t: 10.0,
            start_v: 1e300,
            half_accel: -1e300,
            start_pos: Coord::new(1e300, -1e300, 0.0),
            axes_r: Coord::new(1e-300, 1e300, -1e-300),
        },
        Move {
            print_time: f64::NAN,
            move_t: f64::INFINITY,
            start_v: -0.0,
            half_accel: -0.0,
            start_pos: Coord::new(-0.0, 0.0, f64::INFINITY),
            axes_r: Coord::new(1.0, -1.0, 1.0),
        },
    ];
    let move_times: [f64; 10] = [
        0.0, -0.0, 0.5, 1.0, 2.0, 3.5, -1.25, 1e300, f64::NAN, f64::INFINITY,
    ];
    for coeffs in coeff_sets {
        let (rust_stepper, c_sk) = alloc_pair(coeffs[0], coeffs[1], coeffs[2]);
        let cb: RefCalcCb = unsafe { std::mem::transmute((*c_sk).calc_position_cb) };
        for m in base_moves {
            for t in move_times {
                let ctx = format!("coeffs {coeffs:?} move {m:?} t {t:?}");
                let mut c_move = CMove::from_move(&m);
                let c_result = unsafe { cb(c_sk, &mut c_move, t) };
                let rust_result = rust_stepper.calc_position(&m, t);
                let rust_cb_result = (rust_stepper.sk.calc_position_cb)(&rust_stepper.sk, &m, None, t);
                assert_eq!(
                    bits(rust_result),
                    bits(c_result),
                    "{ctx}: method Rust {rust_result:?} C {c_result:?}"
                );
                assert_eq!(
                    bits(rust_cb_result),
                    bits(c_result),
                    "{ctx}: cb Rust {rust_cb_result:?} C {c_result:?}"
                );
            }
        }
        free_c_sk(c_sk);
    }
}

#[test]
fn set_position_commanded_pos_differential() {
    let coeff_sets: [[f64; 3]; 5] = [
        [1.0, 0.0, 0.0],
        [0.5, 0.5, -1.0],
        [0.0, 0.0, 0.0],
        [2.0, 0.0, -1.0],
        [f64::INFINITY, 1.0, 1.0],
    ];
    let positions: [[f64; 3]; 6] = [
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 3.0],
        [-10.0, 0.0, 10.0],
        [-0.0, -0.0, -0.0],
        [123.456, -789.012, 1e300],
        [f64::INFINITY, 0.0, -f64::INFINITY],
    ];
    for coeffs in coeff_sets {
        let (mut rust_stepper, c_sk) = alloc_pair(coeffs[0], coeffs[1], coeffs[2]);
        // A fresh stepper starts with zero commanded_pos on both sides.
        assert_eq!(
            bits(rust_stepper.sk.get_commanded_pos()),
            bits(unsafe { itersolve_get_commanded_pos(c_sk) }),
            "initial commanded_pos"
        );
        for pos in positions {
            let ctx = format!("coeffs {coeffs:?} pos {pos:?}");
            rust_stepper.sk.set_position(pos[0], pos[1], pos[2]);
            unsafe { itersolve_set_position(c_sk, pos[0], pos[1], pos[2]) };
            let c_cmd = unsafe { itersolve_get_commanded_pos(c_sk) };
            assert_eq!(
                bits(rust_stepper.sk.get_commanded_pos()),
                bits(c_cmd),
                "{ctx}: commanded_pos Rust {:?} C {c_cmd:?}",
                rust_stepper.sk.get_commanded_pos()
            );
            assert_eq!(
                bits(rust_stepper.sk.commanded_pos),
                bits(unsafe { (*c_sk).commanded_pos }),
                "{ctx}: raw commanded_pos"
            );
        }
        free_c_sk(c_sk);
    }
}

/// End-to-end check that the calc callback drives
/// `itersolve_generate_steps` identically on both sides.
struct StepPair {
    rust_stepper: Box<GenericCartesianStepper>,
    c_sk: *mut RefSk,
    rust_sc: Stepcompress,
    ref_sc: RefSc,
    rust_tq: Arc<Mutex<Trapq>>,
    c_tq: *mut CTrapq,
}

impl StepPair {
    fn new(coeffs: [f64; 3], step_dist: f64) -> StepPair {
        let (mut rust_stepper, c_sk) = alloc_pair(coeffs[0], coeffs[1], coeffs[2]);
        let c_tq = unsafe { trapq_alloc() };
        assert!(!c_tq.is_null(), "trapq_alloc returned null");
        let rust_tq = Arc::new(Mutex::new(Trapq::new()));
        let ref_sc = RefSc::new();
        unsafe {
            stepcompress_fill(ref_sc.sc, 7, 3000, 24, 23);
            stepcompress_set_time(ref_sc.sc, 0.0, 1_000_000.0);
        }
        let mut rust_sc = Stepcompress::new();
        rust_sc.fill(7, 3000, 24, 23);
        rust_sc.set_time(0.0, 1_000_000.0);
        unsafe { itersolve_set_trapq(c_sk, c_tq, step_dist) };
        rust_stepper.sk.set_trapq(Some(rust_tq.clone()), step_dist);
        StepPair {
            rust_stepper,
            c_sk,
            rust_sc,
            ref_sc,
            rust_tq,
            c_tq,
        }
    }

    fn append(&self, params: (f64, f64, f64, f64, [f64; 3], [f64; 3], f64, f64, f64)) {
        let (print_time, accel_t, cruise_t, decel_t, start_pos, axes_r, start_v, cruise_v, accel) =
            params;
        unsafe {
            trapq_append(
                self.c_tq,
                print_time,
                accel_t,
                cruise_t,
                decel_t,
                start_pos[0],
                start_pos[1],
                start_pos[2],
                axes_r[0],
                axes_r[1],
                axes_r[2],
                start_v,
                cruise_v,
                accel,
            );
            trapq_check_sentinels(self.c_tq);
        }
        self.rust_tq.lock().append(
            print_time,
            accel_t,
            cruise_t,
            decel_t,
            start_pos[0],
            start_pos[1],
            start_pos[2],
            axes_r[0],
            axes_r[1],
            axes_r[2],
            start_v,
            cruise_v,
            accel,
        );
        self.rust_tq.lock().check_sentinels();
    }

    fn generate_steps(&mut self, flush_time: f64, ctx: &str) {
        let rust_ret = self.rust_stepper.sk.generate_steps(&mut self.rust_sc, flush_time);
        let c_ret = unsafe { itersolve_generate_steps(self.c_sk, self.ref_sc.sc, flush_time) };
        assert_eq!(rust_ret, c_ret, "{ctx}: generate_steps return");
        let c = unsafe { &*self.c_sk };
        let r = &self.rust_stepper.sk;
        assert_eq!(
            bits(c.commanded_pos),
            bits(r.commanded_pos),
            "{ctx}: commanded_pos"
        );
        assert_eq!(
            bits(c.last_flush_time),
            bits(r.last_flush_time),
            "{ctx}: last_flush_time"
        );
        assert_eq!(
            bits(c.last_move_time),
            bits(r.last_move_time),
            "{ctx}: last_move_time"
        );
        assert_eq!(c.active_flags, r.active_flags, "{ctx}: active_flags");
        let rust_msgs: Vec<(Vec<u8>, u64, u64)> = self
            .rust_sc
            .drain_messages()
            .iter()
            .map(|m: &QueuedMessage| (m.bytes.clone(), m.req_clock, m.min_clock))
            .collect();
        let ref_msgs = self.ref_sc.drain_messages();
        assert_eq!(rust_msgs, ref_msgs, "{ctx}: messages diverged");
        const HISTORY_MAX: i32 = 8192;
        let rust_hist = self.rust_sc.extract_old(HISTORY_MAX, 0, u64::MAX);
        let ref_hist = self.ref_sc.extract_old(HISTORY_MAX);
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
            self.rust_sc.get_step_dir(),
            unsafe { stepcompress_get_step_dir(self.ref_sc.sc) },
            "{ctx}: step dir diverged"
        );
    }
}

impl Drop for StepPair {
    fn drop(&mut self) {
        unsafe {
            free(self.c_sk as *mut c_void);
            trapq_free(self.c_tq);
        }
    }
}

#[test]
fn generate_steps_differential() {
    // Pure X, pure Y, pure Z, mixed X+Y, all axes, and no axes (a zero
    // coefficient stepper generates no steps).
    let coeff_sets: [[f64; 3]; 6] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.5, 0.5, 0.0],
        [1.0, -1.0, 0.25],
        [0.0, 0.0, 0.0],
    ];
    for coeffs in coeff_sets {
        let ctx = format!("coeffs {coeffs:?}");
        let mut pair = StepPair::new(coeffs, 0.01);
        // A forward move, then a reversal, then a diagonal move.
        pair.append((
            100.0, 0.1, 1.0, 0.1, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 0.0, 12.0, 10.0,
        ));
        pair.generate_steps(101.2, &format!("{ctx} after first"));
        pair.append((
            101.2, 0.1, 1.0, 0.1, [12.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.0, 12.0, 10.0,
        ));
        pair.generate_steps(102.4, &format!("{ctx} after reversal"));
        pair.append((
            102.4, 0.2, 0.8, 0.2, [0.0, 0.0, 0.0], [0.6, 0.8, 0.5], 0.0, 8.0, 20.0,
        ));
        pair.generate_steps(103.6, &format!("{ctx} after diagonal"));
        // Flush times inside and past the last move.
        pair.generate_steps(103.0, &format!("{ctx} flush inside"));
        pair.generate_steps(200.0, &format!("{ctx} flush past end"));
    }
}
