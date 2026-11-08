//! Differential parity tests for `chelper_motion::trapq`.
//!
//! Each test drives the Rust port and the reference C library
//! (`chelper_ref`, the unchanged `klippy/chelper/trapq.c` sources) with
//! identical inputs and compares, bit for bit (`f64::to_bits`):
//!
//! - the `trapq_extract_old` count and every `pull_move` field, and
//! - the position of every pulled move at its start, middle, and end,
//!   evaluated with the `move_get_coord` formula of `trapq.c`.
//!
//! Inputs:
//!
//! 1. `recorded_sequences_replay`: every recording under
//!    `tests/fixtures/recordings/trapq/*.jsonl` (parsed with the
//!    `recording` crate), mapped by opaque address onto one C and one
//!    Rust `Trapq`. After every recorded call both sides are pulled over
//!    the full window and compared. The recordings themselves record no
//!    `trapq_extract_old` calls (only `trapq_alloc`, `trapq_append`,
//!    `trapq_finalize_moves`, `trapq_set_position`), so the pull below is
//!    the recorded-behaviour oracle: any state divergence changes the
//!    pulled moves.
//! 2. `generated_sequences_match_c`: deterministic pseudo-random
//!    sequences covering zero-length moves, accel/cruise/decel-only
//!    moves, `MAX_NULL_MOVE` null-move padding (first-move and later
//!    gaps), overlapping times, negative and zero times, and history
//!    expiry via `finalize_moves`/`set_position` with early, inside, and
//!    late `clear_history_time` / `print_time` values. Extract windows
//!    probe empty, inside, boundary, and past-the-end ranges.
//!
//! Run with (from `klippy-rs/`):
//!   `cargo test -p chelper_motion --test trapq`
//! (the integration owner builds and runs; per the pilot rules only
//! `cargo check`/`cargo clippy` run here, via WSL with a per-checkout
//! `CARGO_TARGET_DIR`).

use std::path::PathBuf;

use chelper_motion::trapq::{move_get_coord, Move, PullMove, Trapq, NEVER_TIME};
use recording::{load_file, record_files, Record, Value};

// ---------------------------------------------------------------------------
// Reference C declarations (exact signatures from trapq.h / chelper/__init__.py).
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone)]
struct CPullMove {
    print_time: f64,
    move_t: f64,
    start_v: f64,
    accel: f64,
    start_x: f64,
    start_y: f64,
    start_z: f64,
    x_r: f64,
    y_r: f64,
    z_r: f64,
}

enum CTrapq {}

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
    fn trapq_finalize_moves(tq: *mut CTrapq, print_time: f64, clear_history_time: f64);
    fn trapq_set_position(tq: *mut CTrapq, print_time: f64, pos_x: f64, pos_y: f64, pos_z: f64);
    fn trapq_extract_old(
        tq: *const CTrapq,
        p: *mut CPullMove,
        max: i32,
        start_time: f64,
        end_time: f64,
    ) -> i32;
}

// ---------------------------------------------------------------------------
// Comparison helpers.
// ---------------------------------------------------------------------------

fn pull_fields(p: &CPullMove) -> [f64; 10] {
    [
        p.print_time,
        p.move_t,
        p.start_v,
        p.accel,
        p.start_x,
        p.start_y,
        p.start_z,
        p.x_r,
        p.y_r,
        p.z_r,
    ]
}

fn rust_pull_fields(p: &PullMove) -> [f64; 10] {
    [
        p.print_time,
        p.move_t,
        p.start_v,
        p.accel,
        p.start_x,
        p.start_y,
        p.start_z,
        p.x_r,
        p.y_r,
        p.z_r,
    ]
}

fn assert_pull_match(ctx: &str, idx: usize, c: &CPullMove, r: &PullMove) {
    let cf = pull_fields(c);
    let rf = rust_pull_fields(r);
    for (i, (cb, rb)) in cf.iter().zip(rf.iter()).enumerate() {
        assert_eq!(
            cb.to_bits(),
            rb.to_bits(),
            "{ctx}: pull[{idx}] field {i}: C={cb:?} Rust={rb:?}"
        );
    }
    // Positions from the C pull via the move_get_coord formula, compared
    // against the same computation on the Rust pull. The pulled start
    // positions themselves already pin the `move_get_coord` chaining done
    // inside C `trapq_append`/`trapq_check_sentinels`, since every segment
    // start is derived from the previous segment's end in C.
    for t in [0.0, c.move_t / 2.0, c.move_t] {
        let cd = (c.start_v + 0.5 * c.accel * t) * t;
        let cx = c.start_x + c.x_r * cd;
        let cy = c.start_y + c.y_r * cd;
        let cz = c.start_z + c.z_r * cd;
        let rc = r.get_coord(t);
        assert_eq!(cx.to_bits(), rc.x.to_bits(), "{ctx}: pull[{idx}] t={t}: x");
        assert_eq!(cy.to_bits(), rc.y.to_bits(), "{ctx}: pull[{idx}] t={t}: y");
        assert_eq!(cz.to_bits(), rc.z.to_bits(), "{ctx}: pull[{idx}] t={t}: z");
    }
}

/// Pull both traps over `[start_time, end_time]`, compare count, fields, and
/// positions bit for bit.
fn check_extract(
    ctx: &str,
    c_ptr: *mut CTrapq,
    rust: &Trapq,
    max: i32,
    start_time: f64,
    end_time: f64,
) {
    const BUF: usize = 8192;
    assert!(max <= BUF as i32, "{ctx}: max {max} exceeds buffer");
    let mut c_out = vec![CPullMove {
        print_time: f64::NAN,
        move_t: f64::NAN,
        start_v: f64::NAN,
        accel: f64::NAN,
        start_x: f64::NAN,
        start_y: f64::NAN,
        start_z: f64::NAN,
        x_r: f64::NAN,
        y_r: f64::NAN,
        z_r: f64::NAN,
    }; BUF];
    let mut r_out = vec![PullMove::default(); BUF];
    let c_n = unsafe { trapq_extract_old(c_ptr, c_out.as_mut_ptr(), max, start_time, end_time) };
    let r_n = rust.extract_old(&mut r_out, max, start_time, end_time);
    assert_eq!(
        c_n, r_n,
        "{ctx}: extract count (max={max}, window=({start_time}, {end_time}))"
    );
    for i in 0..(c_n as usize) {
        assert_pull_match(ctx, i, &c_out[i], &r_out[i]);
    }
}

/// Pull both sides over the whole history and over several probe windows,
/// including boundary and reversed windows and `max` edge values.
fn check_all_windows(ctx: &str, c_ptr: *mut CTrapq, rust: &Trapq) {
    check_extract(ctx, c_ptr, rust, 512, -1.0e9, NEVER_TIME);
    check_extract(ctx, c_ptr, rust, 512, 0.0, NEVER_TIME);
    check_extract(ctx, c_ptr, rust, 512, -100.0, 100.0);
    check_extract(ctx, c_ptr, rust, 512, 50.0, 50.0);
    check_extract(ctx, c_ptr, rust, 512, 60.0, 40.0);
    check_extract(ctx, c_ptr, rust, 1, -1.0e9, NEVER_TIME);
    check_extract(ctx, c_ptr, rust, 2, -1.0e9, NEVER_TIME);
    check_extract(ctx, c_ptr, rust, 0, -1.0e9, NEVER_TIME);
    check_extract(ctx, c_ptr, rust, -3, -1.0e9, NEVER_TIME);
}

fn call_append(ctx: &str, c_ptr: *mut CTrapq, rust: &mut Trapq, a: &[f64; 13]) {
    unsafe {
        trapq_append(
            c_ptr, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11],
            a[12],
        )
    };
    rust.append(
        a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12],
    );
    check_all_windows(ctx, c_ptr, rust);
}

fn call_finalize(ctx: &str, c_ptr: *mut CTrapq, rust: &mut Trapq, pt: f64, cht: f64) {
    unsafe { trapq_finalize_moves(c_ptr, pt, cht) };
    rust.finalize_moves(pt, cht);
    check_all_windows(ctx, c_ptr, rust);
}

fn call_set_position(ctx: &str, c_ptr: *mut CTrapq, rust: &mut Trapq, pt: f64, x: f64, y: f64, z: f64) {
    unsafe { trapq_set_position(c_ptr, pt, x, y, z) };
    rust.set_position(pt, x, y, z);
    check_all_windows(ctx, c_ptr, rust);
}

// ---------------------------------------------------------------------------
// Generated differential sequences.
// ---------------------------------------------------------------------------

/// Small deterministic generator (SplitMix64) so runs are reproducible.
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

const TIMES: &[f64] = &[
    -5.0, -1.0, -0.001, 0.0, 0.001, 0.25, 1.0, 2.5, 5.0, 10.0, 12.571, 25.0, 40.0, 60.0,
    100.0, 9999999999999999.9,
];
const DURS: &[f64] = &[
    0.0, 0.001, 0.0033333333333333335, 0.016666666666666666, 0.06666666666666668, 0.25, 0.5,
    1.0, 4.0, 11.646666666666667, 74.56713333333332,
];
const POS: &[f64] = &[
    -110.0, -20.0, -12.0, -2.0, -0.0, 0.0, 0.5, 10.0, 110.0, 300.0, 337.5,
];
const RATIOS: &[f64] = &[
    -1.0, -0.7071067811865475, -0.0, 0.0, 0.5, 0.7071067811865475, 1.0, 2.0,
];
const VELS: &[f64] = &[-120.0, -0.0, 0.0, 0.5, 20.0, 150.0, 300.0, 700.0];
const ACCELS: &[f64] = &[-9000.0, -300.0, 0.0, 300.0, 9000.0];

/// The 13 `trapq_append` arguments (missing `tq`): sampled independently,
/// so zero-length, accel/cruise/decel-only, all-zero, and extreme-gap
/// combinations all occur.
fn gen_append(rng: &mut Rng) -> [f64; 13] {
    [
        rng.f64t(TIMES),
        rng.f64t(DURS),
        rng.f64t(DURS),
        rng.f64t(DURS),
        rng.f64t(POS),
        rng.f64t(POS),
        rng.f64t(POS),
        rng.f64t(RATIOS),
        rng.f64t(RATIOS),
        rng.f64t(RATIOS),
        rng.f64t(VELS),
        rng.f64t(VELS),
        rng.f64t(ACCELS),
    ]
}

#[test]
fn generated_sequences_match_c() {
    chelper_ref::link();
    for seed in [1u64, 42, 0xdeadbeefcafe, 987654321] {
        let mut rng = Rng(seed);
        let c_ptr = unsafe { trapq_alloc() };
        assert!(!c_ptr.is_null());
        let mut rust = Trapq::new();
        let base = format!("seed {seed}");

        // Directed probes first: MAX_NULL_MOVE padding on the first gap
        // (prev start <= 0, gap > 1s so the null move is clamped to 1s),
        // a later gap, zero-length and single-segment appends, and
        // history expiry that lands exactly on segment boundaries.
        let directed: [([f64; 13], &str); 6] = [
            (
                [0.0, 0.25, 0.4333333333333334, 0.25, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 20.0, 300.0],
                "directed accel+cruise+decel",
            ),
            (
                [100.0, 0.5, 0.0, 0.5, 10.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 5.0, 0.0],
                "directed first gap, clamped null move",
            ),
            (
                [101.5, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "directed zero-length",
            ),
            (
                [102.0, 0.0, 2.0, 0.0, 20.0, 5.0, 0.0, 0.5, 0.5, 0.0, 30.0, 30.0, 500.0],
                "directed cruise only",
            ),
            (
                [120.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 10.0, 100.0],
                "directed accel only after gap",
            ),
            (
                [121.0, 0.0, 0.0, 1.0, 5.0, 0.0, 0.0, -1.0, 0.0, 0.0, 10.0, 10.0, -100.0],
                "directed decel only",
            ),
        ];
        for (a, name) in directed {
            call_append(&format!("{base} {name}"), c_ptr, &mut rust, &a);
        }
        call_finalize(&format!("{base} finalize boundary"), c_ptr, &mut rust, 100.0, 0.0);
        call_finalize(&format!("{base} finalize clamped null"), c_ptr, &mut rust, 99.5, 99.5);
        call_finalize(&format!("{base} finalize inside history"), c_ptr, &mut rust, 101.0, 0.5);
        call_finalize(&format!("{base} finalize clear newest kept"), c_ptr, &mut rust, 200.0, 1.0e250);
        call_set_position(&format!("{base} set_position mid"), c_ptr, &mut rust, 100.5, 1.0, 2.0, 3.0);
        call_set_position(&format!("{base} set_position past"), c_ptr, &mut rust, 500.0, 0.0, 0.0, 0.0);
        call_set_position(&format!("{base} set_position before"), c_ptr, &mut rust, -5.0, 7.0, 7.0, 7.0);

        for i in 0..1500 {
            match rng.pick(10) {
                0..=4 => {
                    let a = gen_append(&mut rng);
                    call_append(&format!("{base} op {i} append {a:?}"), c_ptr, &mut rust, &a);
                }
                5..=6 => {
                    let pt = rng.f64t(TIMES);
                    let cht = rng.f64t(TIMES);
                    call_finalize(
                        &format!("{base} op {i} finalize {pt} {cht}"),
                        c_ptr,
                        &mut rust,
                        pt,
                        cht,
                    );
                }
                7..=8 => {
                    let pt = rng.f64t(TIMES);
                    let (x, y, z) = (rng.f64t(POS), rng.f64t(POS), rng.f64t(POS));
                    call_set_position(
                        &format!("{base} op {i} set_position {pt} {x} {y} {z}"),
                        c_ptr,
                        &mut rust,
                        pt,
                        x,
                        y,
                        z,
                    );
                }
                _ => {
                    // Empty and boundary extract windows on current state.
                    let a = rng.f64t(TIMES);
                    let b = rng.f64t(TIMES);
                    let mx: i32 = [-1, 0, 1, 3, 7, 512][rng.pick(6)];
                    check_extract(&format!("{base} op {i} probe"), c_ptr, &rust, mx, a, b);
                }
            }
        }
        unsafe { trapq_free(c_ptr) };
    }
}

/// The `move_get_coord` chaining done inside C `trapq_append` must agree
/// with the Rust formula on the moved segments: pin the formula itself on
/// fixed values (the chaining through C is pinned by `assert_pull_match`,
/// since every C-pulled segment start is derived from the previous C
/// segment end).
#[test]
fn move_get_coord_formula() {
    let m = Move {
        print_time: 1.0,
        move_t: 2.0,
        start_v: 20.0,
        half_accel: 150.0,
        start_pos: chelper_motion::trapq::Coord::new(300.0, 0.0, 0.0),
        axes_r: chelper_motion::trapq::Coord::new(1.0, 0.0, 0.0),
    };
    let c = move_get_coord(&m, 0.25);
    // (20 + 150*0.25) * 0.25 = 57.5 * 0.25 = 14.375; 300 + 14.375.
    assert_eq!(c.x.to_bits(), 314.375f64.to_bits());
    assert_eq!(c.y.to_bits(), 0.0f64.to_bits());
    assert_eq!(c.z.to_bits(), 0.0f64.to_bits());
}

// ---------------------------------------------------------------------------
// Recorded sequences.
// ---------------------------------------------------------------------------

fn f64val(v: &Value) -> f64 {
    match v {
        Value::Float(fv) => fv.to_f64().expect("decodable float hex"),
        Value::Int(i) => *i as f64,
        // Recorded `axes_r_y` is sometimes a Python bool: `extruder.py`
        // passes `can_pressure_advance` (a bool) as the pressure-advance
        // flag, and CFFI converts it to 1.0/0.0 for the `double` parameter.
        Value::Bool(b) => {
            if *b {
                1.0
            } else {
                0.0
            }
        }
        other => panic!("expected float/int argument, got {other:?}"),
    }
}

fn opaque_addr(v: &Value) -> String {
    match v {
        Value::Opaque { addr, .. } => addr.clone(),
        other => panic!("expected opaque trapq pointer, got {other:?}"),
    }
}

fn recordings_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../../tests/fixtures/recordings")
}

#[test]
fn recorded_sequences_replay() {
    chelper_ref::link();
    let root = recordings_root();
    let files = record_files(&root, "trapq").expect("list trapq recordings");
    assert!(!files.is_empty(), "no trapq recordings found under {root:?}");
    for file in &files {
        let records = load_file(file).expect("parse recording");
        replay_one(&records, file);
    }
}

fn replay_one(records: &[Record], file: &std::path::Path) {
    let mut pairs: Vec<(String, *mut CTrapq, Trapq)> = Vec::new();
    let ctx = format!("{} ", file.display());
    for rec in records {
        match rec.function.as_str() {
            "trapq_alloc" => {
                let addr = match &rec.ret {
                    Some(Value::Opaque { addr, .. }) => addr.clone(),
                    other => panic!("{ctx}seq {}: alloc ret {other:?}", rec.seq),
                };
                let c_ptr = unsafe { trapq_alloc() };
                assert!(!c_ptr.is_null());
                pairs.push((addr, c_ptr, Trapq::new()));
            }
            "trapq_append" => {
                let a = &rec.args;
                assert_eq!(a.len(), 14, "{ctx}seq {}: append arity", rec.seq);
                let addr = opaque_addr(&a[0]);
                let idx = pairs
                    .iter()
                    .position(|(k, _, _)| *k == addr)
                    .unwrap_or_else(|| panic!("{ctx}seq {}: unknown trapq", rec.seq));
                let v: Vec<f64> = a[1..].iter().map(f64val).collect();
                call_append(
                    &format!("{ctx}seq {} append", rec.seq),
                    pairs[idx].1,
                    &mut pairs[idx].2,
                    &[v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12]],
                );
            }
            "trapq_finalize_moves" => {
                let a = &rec.args;
                assert_eq!(a.len(), 3, "{ctx}seq {}: finalize arity", rec.seq);
                let addr = opaque_addr(&a[0]);
                let idx = pairs
                    .iter()
                    .position(|(k, _, _)| *k == addr)
                    .unwrap_or_else(|| panic!("{ctx}seq {}: unknown trapq", rec.seq));
                let pt = f64val(&a[1]);
                let cht = f64val(&a[2]);
                call_finalize(
                    &format!("{ctx}seq {} finalize", rec.seq),
                    pairs[idx].1,
                    &mut pairs[idx].2,
                    pt,
                    cht,
                );
            }
            "trapq_set_position" => {
                let a = &rec.args;
                assert_eq!(a.len(), 5, "{ctx}seq {}: set_position arity", rec.seq);
                let addr = opaque_addr(&a[0]);
                let idx = pairs
                    .iter()
                    .position(|(k, _, _)| *k == addr)
                    .unwrap_or_else(|| panic!("{ctx}seq {}: unknown trapq", rec.seq));
                let pt = f64val(&a[1]);
                let (x, y, z) = (f64val(&a[2]), f64val(&a[3]), f64val(&a[4]));
                call_set_position(
                    &format!("{ctx}seq {} set_position", rec.seq),
                    pairs[idx].1,
                    &mut pairs[idx].2,
                    pt,
                    x,
                    y,
                    z,
                );
            }
            "trapq_free" => {
                let addr = opaque_addr(&rec.args[0]);
                let idx = pairs
                    .iter()
                    .position(|(k, _, _)| *k == addr)
                    .unwrap_or_else(|| panic!("{ctx}seq {}: unknown trapq", rec.seq));
                let (_, c_ptr, _) = pairs.remove(idx);
                unsafe { trapq_free(c_ptr) };
            }
            other => panic!("{ctx}seq {}: unexpected function {other}", rec.seq),
        }
    }
    for (addr, c_ptr, rust) in &pairs {
        check_extract(&format!("{ctx}final {addr}"), *c_ptr, rust, 512, -1.0e9, NEVER_TIME);
    }
    for (_, c_ptr, _) in pairs {
        unsafe { trapq_free(c_ptr) };
    }
}
