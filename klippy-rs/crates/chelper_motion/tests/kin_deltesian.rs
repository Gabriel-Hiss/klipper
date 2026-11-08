//! Differential parity tests for `chelper_motion::kin_deltesian`.
//!
//! Both the Rust port and the reference C library (`chelper_ref`, the
//! frozen `klippy/chelper/kin_deltesian.c` sources) are driven with
//! identical inputs and compared bit for bit (`f64::to_bits`):
//!
//! - the state right after `deltesian_stepper_alloc` (zeroed
//!   `StepperKinematics` fields, `active_flags == AF_X | AF_Z`), and
//! - the position reported through `itersolve_calc_position_from_coord`
//!   and the direct position callback at many `(x, y, z)`, `move_time`
//!   and move combinations, including moves whose radicand is negative
//!   (both sides return NaN), zero moves, and asymmetric arm geometry
//!   taken from `klippy/kinematics/deltesian.py` (`arm2[0]`, `-arm_x[0]`
//!   and `arm2[1]`, `arm_x[1]`).
//!
//! The C callback is invoked through the function pointer stored in the
//! allocated `stepper_kinematics` (the header type
//! `sk_calc_callback`), so the comparison exercises the same entry point
//! `itersolve_generate_steps` uses.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test kin_deltesian`

use chelper_motion::itersolve::{StepperCalcCb, AF_X, AF_Z};
use chelper_motion::kin_deltesian::{
    deltesian_stepper_alloc as rust_deltesian_stepper_alloc,
    deltesian_stepper_calc_position, DeltesianStepper,
};
use chelper_motion::trapq::{move_get_coord, Coord, Move};
use core::ffi::c_void;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

/// `struct move` from `trapq.h` (x86_64 layout). `node` is never touched
/// by the position callback; the fields are read by `move_get_coord`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct CMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    half_accel: f64,
    start_pos: [f64; 3],
    axes_r: [f64; 3],
    node: ListNode,
}

/// `struct stepper_kinematics` from `itersolve.h`, x86_64 layout. Only
/// used to call back into the C position solver.
#[repr(C)]
struct RefSk {
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

type SkCalcCallback = unsafe extern "C" fn(*mut RefSk, *mut CMove, f64) -> f64;

unsafe extern "C" {
    fn deltesian_stepper_alloc(arm2: f64, arm_x: f64) -> *mut RefSk;
    fn itersolve_calc_position_from_coord(sk: *mut RefSk, x: f64, y: f64, z: f64) -> f64;
    fn free(ptr: *mut c_void);
}


fn c_move(m: &Move) -> CMove {
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

/// Call the C position callback stored in the allocated stepper.
unsafe fn c_calc_position(sk: *mut RefSk, m: &CMove, move_time: f64) -> f64 {
    let ptr = unsafe { (*sk).calc_position_cb };
    assert!(!ptr.is_null(), "C stepper has no position callback");
    let cb: SkCalcCallback = unsafe { core::mem::transmute(ptr) };
    let mut cm = *m;
    unsafe { cb(sk, &mut cm as *mut CMove, move_time) }
}

struct Pair {
    rust_stepper: DeltesianStepper,
    c_sk: *mut RefSk,
}

impl Pair {
    fn new(arm2: f64, arm_x: f64) -> Pair {
        chelper_ref::link();
        let rust_stepper = rust_deltesian_stepper_alloc(arm2, arm_x);
        let c_sk = unsafe { deltesian_stepper_alloc(arm2, arm_x) };
        assert!(!c_sk.is_null(), "deltesian_stepper_alloc returned null");
        let pair = Pair {
            rust_stepper,
            c_sk,
        };
        pair.check_alloc_state(&format!("init arm2={arm2:?} arm_x={arm_x:?}"));
        pair
    }

    fn check_alloc_state(&self, ctx: &str) {
        let (c, r) = (unsafe { &*self.c_sk }, &self.rust_stepper.sk);
        assert_eq!(c.active_flags, AF_X | AF_Z, "{ctx}: C active_flags");
        assert_eq!(r.active_flags, AF_X | AF_Z, "{ctx}: Rust active_flags");
        assert_eq!(
            r.step_dist.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: Rust step_dist"
        );
        assert_eq!(
            c.step_dist.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: C step_dist"
        );
        assert_eq!(
            r.commanded_pos.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: Rust commanded_pos"
        );
        assert_eq!(
            c.commanded_pos.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: C commanded_pos"
        );
        assert_eq!(
            r.last_flush_time.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: Rust last_flush_time"
        );
        assert_eq!(
            c.last_flush_time.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: C last_flush_time"
        );
        assert_eq!(
            r.last_move_time.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: Rust last_move_time"
        );
        assert_eq!(
            c.last_move_time.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: C last_move_time"
        );
        assert_eq!(
            r.gen_steps_pre_active.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: Rust gen_steps_pre_active"
        );
        assert_eq!(
            c.gen_steps_pre_active.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: C gen_steps_pre_active"
        );
        assert_eq!(
            r.gen_steps_post_active.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: Rust gen_steps_post_active"
        );
        assert_eq!(
            c.gen_steps_post_active.to_bits(),
            0.0f64.to_bits(),
            "{ctx}: C gen_steps_post_active"
        );
        assert!(r.tq.is_none(), "{ctx}: Rust tq must start as None");
        assert!(r.sc.is_none(), "{ctx}: Rust sc must start as None");
        assert!(unsafe { (*self.c_sk).tq }.is_null(), "{ctx}: C tq");
        assert!(unsafe { (*self.c_sk).sc }.is_null(), "{ctx}: C sc");
        assert!(
            r.post_cb.is_none(),
            "{ctx}: Rust post_cb must start as None"
        );
        assert!(
            unsafe { (*self.c_sk).post_cb }.is_null(),
            "{ctx}: C post_cb"
        );
        assert_eq!(
            r.calc_position_cb as usize,
            deltesian_stepper_calc_position as StepperCalcCb as usize,
            "{ctx}: Rust calc_position_cb"
        );
        assert!(
            !unsafe { (*self.c_sk).calc_position_cb }.is_null(),
            "{ctx}: C calc_position_cb"
        );
    }

    fn check_coord(&self, ctx: &str, x: f64, y: f64, z: f64) {
        let c = unsafe { itersolve_calc_position_from_coord(self.c_sk, x, y, z) };
        let r = self.rust_stepper.sk.calc_position_from_coord(x, y, z);
        assert_eq!(
            c.to_bits(),
            r.to_bits(),
            "{ctx}: calc from coord ({x:?}, {y:?}, {z:?}) C={c:?} Rust={r:?}"
        );
    }

    fn check_move(&self, ctx: &str, m: &Move, move_time: f64) {
        let cm = c_move(m);
        let c = unsafe { c_calc_position(self.c_sk, &cm, move_time) };
        let r = deltesian_stepper_calc_position(&self.rust_stepper.sk, m, move_time);
        assert_eq!(
            c.to_bits(),
            r.to_bits(),
            "{ctx}: calc from move @{move_time:?} C={c:?} Rust={r:?}"
        );
        let r2 = (self.rust_stepper.sk.calc_position_cb)(&self.rust_stepper.sk, m, move_time);
        assert_eq!(
            r.to_bits(),
            r2.to_bits(),
            "{ctx}: Rust callback through sk pointer"
        );
    }
}

impl Drop for Pair {
    fn drop(&mut self) {
        unsafe { free(self.c_sk as *mut c_void) }
    }
}

fn zero_move(start: [f64; 3]) -> Move {
    Move {
        print_time: 0.0,
        move_t: 1000.0,
        start_v: 0.0,
        half_accel: 0.0,
        start_pos: Coord::new(start[0], start[1], start[2]),
        axes_r: Coord::new(0.0, 0.0, 0.0),
    }
}

#[test]
fn alloc_state_and_coords_match_c() {
    // Symmetric and asymmetric arm geometry as requested by
    // `klippy/kinematics/deltesian.py`: (arm2[0], -arm_x[0]) for the
    // left pillar and (arm2[1], arm_x[1]) for the right pillar.
    for (arm2, arm_x) in [
        (100.0 * 100.0, -50.0),
        (120.0 * 120.0, 50.0),
        (30.25 * 30.25, -15.75),
        (0.0, 0.0),
        (1.0, 0.0),
    ] {
        let pair = Pair::new(arm2, arm_x);
        for (x, y, z) in [
            (0.0, 0.0, 0.0),
            (arm_x, 0.0, 0.0),
            (arm_x, 7.5, -3.25),
            (arm_x + 0.5, 0.0, 1.0),
            (arm_x - 10.0, 2.0, 5.0),
            (arm_x + 10.0, -2.0, 5.0),
            (arm_x + 100.0, 0.0, 0.0),
            (arm_x - 100.0, 0.0, 0.0),
            (arm_x + 1e6, 0.0, 0.0),
            (arm_x + 1.0, 0.0, 1e6),
            (arm_x, 0.0, -1.0),
        ] {
            pair.check_coord(&format!("arm2={arm2:?} arm_x={arm_x:?}"), x, y, z);
        }
        // Y never enters the deltesian formula; sweep it anyway.
        for y in [-1e9, -1.0, 0.0, 1.0, 1e9] {
            pair.check_coord("y-sweep", arm_x, y, 2.0);
        }
    }
}

#[test]
fn moves_match_c() {
    let pair = Pair::new(100.0 * 100.0, -50.0);
    let moves = [
        zero_move([0.0, 0.0, 0.0]),
        Move {
            start_v: 10.0,
            half_accel: 5.0,
            start_pos: Coord::new(0.0, 0.0, 0.0),
            axes_r: Coord::new(1.0, 0.0, 0.0),
            move_t: 2.0,
            print_time: 1.0,
        },
        Move {
            start_v: 20.0,
            half_accel: -10.0,
            start_pos: Coord::new(10.0, 5.0, 3.0),
            axes_r: Coord::new(0.0, 1.0, 0.0),
            move_t: 1.5,
            print_time: 2.0,
        },
        Move {
            start_v: 0.0,
            half_accel: 250.0,
            start_pos: Coord::new(-50.0, 0.0, -1.0),
            axes_r: Coord::new(0.6, 0.0, 0.8),
            move_t: 0.25,
            print_time: 3.0,
        },
        Move {
            start_v: 30.0,
            half_accel: 0.0,
            start_pos: Coord::new(200.0, 0.0, 0.0),
            axes_r: Coord::new(1.0, 0.0, 0.0),
            move_t: 1.0,
            print_time: 4.0,
        },
    ];
    for (i, m) in moves.iter().enumerate() {
        for t in [0.0, 0.25, 0.5, m.move_t / 2.0, m.move_t, -0.5, 3.0] {
            pair.check_move(&format!("move {i}"), m, t);
        }
    }
    // The Rust side composition agrees with move_get_coord + sqrt order.
    let m = &moves[1];
    for t in [0.0, 0.5, 1.0, 2.0] {
        let c = move_get_coord(m, t);
        let dx = c.x - (-50.0);
        let expected = (100.0 * 100.0 - dx * dx).sqrt() + c.z;
        let r = deltesian_stepper_calc_position(&pair.rust_stepper.sk, m, t);
        assert_eq!(expected.to_bits(), r.to_bits(), "composition @{t:?}");
    }
}

/// Small deterministic generator (SplitMix64), as in the itersolve tests.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^ (z >> 31)
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }

    fn f64t(&mut self, table: &[f64]) -> f64 {
        table[self.pick(table.len())]
    }
}

#[test]
fn generated_inputs_match_c() {
    let mut rng = Rng(0xdeadbeef);
    for _ in 0..8 {
        let arm2 = rng.f64t(&[1.0, 900.0, 10000.0, 14400.0]);
        let arm_x = rng.f64t(&[-50.0, -15.75, 0.0, 33.0, 50.0]);
        let pair = Pair::new(arm2, arm_x);
        for _ in 0..30 {
            let x = rng.f64t(&[-200.0, -50.0, 0.0, 12.5, 50.0, 200.0, 1e6]);
            let y = rng.f64t(&[-5.0, 0.0, 5.0]);
            let z = rng.f64t(&[-10.0, 0.0, 1.5, 100.0]);
            pair.check_coord("generated", x, y, z);
        }
        for _ in 0..30 {
            let m = Move {
                print_time: rng.f64t(&[0.0, 1.0, 10.0]),
                move_t: rng.f64t(&[0.0, 0.1, 1.0, 5.0]),
                start_v: rng.f64t(&[-20.0, 0.0, 10.0, 50.0]),
                half_accel: rng.f64t(&[-100.0, 0.0, 25.0, 500.0]),
                start_pos: Coord::new(
                    rng.f64t(&[-100.0, 0.0, 25.0]),
                    rng.f64t(&[-5.0, 0.0, 5.0]),
                    rng.f64t(&[-10.0, 0.0, 3.0]),
                ),
                axes_r: Coord::new(
                    rng.f64t(&[-1.0, 0.0, 0.6, 1.0]),
                    rng.f64t(&[-1.0, 0.0, 1.0]),
                    rng.f64t(&[-1.0, 0.0, 0.8, 1.0]),
                ),
            };
            let t = rng.f64t(&[-1.0, 0.0, 0.05, 0.5, 1.0, 5.0]);
            pair.check_move("generated", &m, t);
        }
    }
}
