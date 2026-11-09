//! Port of `klippy/chelper/kin_shaper.c`.
//!
//! Input shapers convolve the commanded position with a train of pulses so
//! the stepper steps on the smoothed position. The shaper wraps an original
//! kinematics ([`StepperKinematics`]) and computes
//! `calc_position(axis) = sum_i a_i * pos(t + pulse_t_i)` over neighbouring
//! moves, shifting the pulses around the zero-time mean so constant-speed
//! motion is unchanged (see `shift_pulses`).
//!
//! Like the C, the shaper's own kinematics comes first in [`InputShaper`],
//! so the address of the embedded `sk` is the address of the whole struct.
//! The callbacks recover the shaper from the `sk` reference they are handed
//! (the C `container_of` pattern); the shaper must therefore not be moved
//! once its `sk` has been handed out. [`input_shaper_alloc`] returns a
//! `Box`, which keeps the address stable.
//!
//! The wrapped kinematics is borrowed through a raw pointer, exactly like
//! `struct input_shaper { struct stepper_kinematics *orig_sk; }`: the caller
//! owns the original kinematics and it must outlive the shaper; Python
//! keeps it alive in `input_shaper.py` (`orig_stepper_kinematics` list).
//! Replacing the original kinematics (dual carriage mode change) and
//! calling [`input_shaper_update_sk`] re-reads its flags, as in the C.
//! The scratch move (`m` in the C) lives in its own heap allocation
//! (`Box<UnsafeCell<Move>>`) because the callbacks receive
//! `&StepperKinematics` while the scratch is rewritten before each call to
//! the original kinematics. The compiler marks that reference read-only and
//! unaliased for the whole call, so a store into memory reached by casting
//! it to the container is undefined behaviour; on the MIPS32 build LLVM
//! dropped those stores and every shaped axis reported position 0. A store
//! through the pointer held in the `Box` does not derive from the callback
//! argument. Step generation only ever calls a kinematics from its own step
//! thread, matching the C, so the scratch needs no locking.
//!
//! Neighbouring moves are found in the trapq the caller attached to the
//! shaper's `sk` (the C list pointers on `struct move` have no Rust
//! equivalent; the Rust `itersolve` hands out copies of the moves, so the
//! current move is re-located there by exact field equality). If the move
//! cannot be located in that trapq (a scratch move from
//! `itersolve_calc_position_from_coord`, whose time window never reaches
//! beyond the current move for any realistic pulse train), only the
//! current move is considered, which reproduces the C evaluation in the
//! same window.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order (see
//! `parity/maps/map-motion.md`, section 3.12).

use std::cell::UnsafeCell;

use crate::itersolve::{StepperKinematics, StepperCalcCb, AF_X, AF_Y, AF_Z};
use crate::trapq::{move_get_coord, move_get_distance, Move, MoveList};

/// `DUMMY_T` from `kin_shaper.c`: the evaluation time of the original
/// kinematics on the scratch move.
pub const DUMMY_T: f64 = 500.0;

/// `KIN_FLAGS` from `kin_shaper.c`: `AF_X`, `AF_Y`, `AF_Z` by axis index.
const KIN_FLAGS: [i32; 3] = [AF_X, AF_Y, AF_Z];

/// One shaper pulse from `struct shaper_pulses` in `kin_shaper.c`:
/// the pulse time `t` and the normalised magnitude `a`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShaperPulse {
    /// Pulse time (shifted, see [`ShaperPulses::shift_pulses`]).
    pub t: f64,
    /// Pulse magnitude (normalised so the magnitudes sum to 1).
    pub a: f64,
}

/// `struct shaper_pulses` from `kin_shaper.c`: the pulse train of one
/// axis. `num_pulses`/`p_ind` mirror the C `int` fields; pulses past
/// `num_pulses` keep the contents of the previous `init_shaper` call,
/// exactly like the C array.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ShaperPulses {
    /// Number of valid pulses in `pulses`.
    pub num_pulses: i32,
    /// Index of the first pulse with `t > 0`.
    pub p_ind: i32,
    /// Pulse storage, in the C order (`t` then `a` per pulse).
    pub pulses: [ShaperPulse; 10],
}

impl ShaperPulses {
    /// `shift_pulses` from `kin_shaper.c`: shift the pulses around the
    /// magnitude-weighted mean time so the shaper is the identity for
    /// constant-speed motion, then find `p_ind` (the first pulse with
    /// positive time).
    pub fn shift_pulses(&mut self) {
        let mut ts = 0.0;
        for i in 0..self.num_pulses {
            let p = &self.pulses[i as usize];
            ts += p.a * p.t;
        }
        for i in 0..self.num_pulses {
            self.pulses[i as usize].t -= ts;
        }
        let mut i = 0;
        while i < self.num_pulses {
            if self.pulses[i as usize].t > 0.0 {
                break;
            }
            i += 1;
        }
        self.p_ind = i;
    }

    /// `init_shaper` from `kin_shaper.c`: normalise the magnitudes, reverse
    /// the pulse order and negate the times, then shift. Returns `0` on
    /// success, `-1` when `n` is out of range (clearing `num_pulses`) or
    /// when the times are not strictly increasing (leaving the previous
    /// pulses untouched), exactly as in the C.
    pub fn init_shaper(&mut self, n: i32, a: &[f64], t: &[f64]) -> i32 {
        if !(0..=10).contains(&n) {
            self.num_pulses = 0;
            return -1;
        }
        for i in 1..n {
            if t[(i - 1) as usize] >= t[i as usize] {
                return -1;
            }
        }
        let mut sum_a = 0.0;
        for i in 0..n {
            sum_a += a[i as usize];
        }
        let inv_a = 1.0 / sum_a;
        for i in 0..n {
            self.pulses[(n - i - 1) as usize] = ShaperPulse {
                a: a[i as usize] * inv_a,
                t: -t[i as usize],
            };
        }
        self.num_pulses = n;
        self.shift_pulses();
        0
    }
}

/// `get_axis_position` from `kin_shaper.c`: the position of one axis of
/// `m` at `move_time`, in the C order (`start_pos + axes_r * distance`).
/// `axis` is the axis index (0 = `x`, 1 = `y`, 2 = `z`).
pub fn get_axis_position(m: &Move, axis: usize, move_time: f64) -> f64 {
    let move_dist = move_get_distance(m, move_time);
    match axis {
        0 => m.start_pos.x + m.axes_r.x * move_dist,
        1 => m.start_pos.y + m.axes_r.y * move_dist,
        _ => m.start_pos.z + m.axes_r.z * move_dist,
    }
}

/// `struct input_shaper` from `kin_shaper.c`: the wrapped kinematics and
/// its per-axis pulse trains. The C fields keep their names and order:
/// `sk` (the stepper kinematics that reports shaped positions), `orig_sk`
/// (the wrapped kinematics), `m` (the scratch move the original
/// kinematics is evaluated on) and `sp` (one [`ShaperPulses`] per axis).
#[repr(C)]
pub struct InputShaper {
    /// The stepper kinematics for this shaper; `StepperKinematics` is the
    /// first member so the address of this field is the address of the
    /// struct itself, as in the C `container_of`.
    pub sk: StepperKinematics,
    orig_sk: *mut StepperKinematics,
    /// Scratch move, kept outside the struct: see the module docs.
    m: Box<UnsafeCell<Move>>,
    sp: [ShaperPulses; 3],
}

impl InputShaper {
    /// The wrapped kinematics, or null if [`input_shaper_set_sk`] has not
    /// been called yet (the C `orig_sk` starts out null).
    pub fn orig_sk(&self) -> *mut StepperKinematics {
        self.orig_sk
    }

    /// The scratch move (`m` in the C) the original kinematics is evaluated
    /// on at [`DUMMY_T`]; its `start_pos` holds the shaped position.
    pub fn dummy_move(&self) -> Move {
        *unsafe { &*self.m.get() }
    }

    /// The pulse train of one axis (0 = `x`, 1 = `y`, 2 = `z`).
    pub fn shaper_pulses(&self, axis: usize) -> &ShaperPulses {
        &self.sp[axis]
    }

    /// `calc_position` from `kin_shaper.c`: the convolved position of
    /// `m` on `axis` at `move_time`, summed over `sp` with neighbouring
    /// moves taken from the trapq attached to the shaper's `sk`. The C is
    /// `static`; the Rust version is public because the callbacks below —
    /// and tests — need it. `m` is located in that trapq by exact field
    /// equality; when it is not found (a scratch move), only `m` itself is
    /// considered.
    pub fn calc_position(
        &self,
        m: &Move,
        axis: usize,
        move_time: f64,
        sp: &ShaperPulses,
    ) -> f64 {
        if let Some(tq) = &self.sk.tq {
            let tq = tq.lock();
            let seq = tq.move_seq();
            if let Some(cur) = seq.find(m, move_eq) {
                return calc_position_in_seq(&seq, cur, axis, move_time, sp);
            }
        }
        calc_position_in_seq(std::slice::from_ref(m), 0, axis, move_time, sp)
    }

    /// `shaper_note_generation_time` from `kin_shaper.c`: the step
    /// generation windows implied by the configured pulses of the active
    /// axes.
    fn note_generation_time(&mut self) {
        let mut pre_active = 0.0;
        let mut post_active = 0.0;
        for (i, flag) in KIN_FLAGS.iter().enumerate() {
            let sp = &self.sp[i];
            if (self.sk.active_flags & flag) != 0 && sp.num_pulses != 0 {
                let pre_cand = sp.pulses[(sp.num_pulses - 1) as usize].t;
                if pre_cand > pre_active {
                    pre_active = pre_cand;
                }
                let post_cand = -sp.pulses[0].t;
                if post_cand > post_active {
                    post_active = post_cand;
                }
            }
        }
        self.sk.gen_steps_pre_active = pre_active;
        self.sk.gen_steps_post_active = post_active;
    }
}

/// Exact field equality of two moves, used to re-locate a move in the
/// trapq sequence (see [`InputShaper::calc_position`]).
fn move_eq(a: &Move, b: &Move) -> bool {
    a.print_time == b.print_time
        && a.move_t == b.move_t
        && a.start_v == b.start_v
        && a.half_accel == b.half_accel
        && a.start_pos == b.start_pos
        && a.axes_r == b.axes_r
}

/// `calc_position` from `kin_shaper.c` over an explicit move sequence:
/// backwards over the pulses at `p_ind - 1` and down, then forwards from
/// `p_ind` up. The `t += pulse.t` / `t -= pulse.t` order (and its rounding)
/// is part of the result, so it is kept exactly. Walking past a sequence
/// end stops there; in the C the list past the sentinels is garbage that
/// no reachable call reads (pulse excursions stay inside the real moves
/// for any shaper produced from `shaper_defs`).
fn calc_position_in_seq<L: MoveList + ?Sized>(
    seq: &L,
    cur: usize,
    axis: usize,
    move_time: f64,
    sp: &ShaperPulses,
) -> f64 {
    let mut res = 0.0;
    let mut t = move_time;
    let mut prev = cur;
    let mut i = sp.p_ind - 1;
    while i >= 0 {
        t += sp.pulses[i as usize].t;
        while t < 0.0 {
            if prev == 0 {
                break;
            }
            prev -= 1;
            t += seq.get(prev).move_t;
        }
        res += sp.pulses[i as usize].a * get_axis_position(seq.get(prev), axis, t);
        t -= sp.pulses[i as usize].t;
        i -= 1;
    }
    t = move_time;
    let mut m_idx = cur;
    let mut i = sp.p_ind;
    while i < sp.num_pulses {
        t += sp.pulses[i as usize].t;
        while t > seq.get(m_idx).move_t {
            if m_idx + 1 >= seq.len() {
                break;
            }
            t -= seq.get(m_idx).move_t;
            m_idx += 1;
        }
        res += sp.pulses[i as usize].a * get_axis_position(seq.get(m_idx), axis, t);
        t -= sp.pulses[i as usize].t;
        i += 1;
    }
    res
}

/// Recover the [`InputShaper`] owning `sk` (the C `container_of`).
///
/// # Safety
///
/// `sk` must be the `sk` member of a live [`InputShaper`] (the callbacks
/// receive exactly that reference from the solver), and the returned
/// reference must not outlive it.
unsafe fn from_sk(sk: &StepperKinematics) -> &InputShaper {
    unsafe { &*(sk as *const StepperKinematics as *const InputShaper) }
}

/// Mutable counterpart of [`from_sk`].
///
/// # Safety
///
/// Same contract as [`from_sk`], plus the caller must have exclusive
/// access to the shaper.
unsafe fn from_sk_mut(sk: &mut StepperKinematics) -> &mut InputShaper {
    unsafe { &mut *(sk as *mut StepperKinematics as *mut InputShaper) }
}

/// Call the wrapped kinematics on the scratch move at [`DUMMY_T`].
///
/// # Safety
///
/// Called from the `StepperCalcCb` callbacks; the shaper is live and the
/// original kinematics has been attached.
unsafe fn call_orig(is: &InputShaper) -> f64 {
    let orig = unsafe { &*is.orig_sk };
    let scratch = unsafe { &*is.m.get() };
    (orig.calc_position_cb)(orig, scratch, DUMMY_T)
}

/// `shaper_x_calc_position` from `kin_shaper.c`.
fn shaper_x_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: the solver hands us the shaper's own `sk`; the shaper is
    // alive for the whole step generation (same as the C).
    let is = unsafe { from_sk(sk) };
    if is.sp[0].num_pulses == 0 {
        if is.orig_sk.is_null() {
            return move_get_coord(m, move_time).x;
        }
        let orig = unsafe { &*is.orig_sk };
        return (orig.calc_position_cb)(orig, m, move_time);
    }
    let pos = is.calc_position(m, 0, move_time, &is.sp[0]);
    unsafe {
        (*is.m.get()).start_pos.x = pos;
    }
    unsafe { call_orig(is) }
}

/// `shaper_y_calc_position` from `kin_shaper.c`.
fn shaper_y_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: see `shaper_x_calc_position`.
    let is = unsafe { from_sk(sk) };
    if is.sp[1].num_pulses == 0 {
        if is.orig_sk.is_null() {
            return move_get_coord(m, move_time).y;
        }
        let orig = unsafe { &*is.orig_sk };
        return (orig.calc_position_cb)(orig, m, move_time);
    }
    let pos = is.calc_position(m, 1, move_time, &is.sp[1]);
    unsafe {
        (*is.m.get()).start_pos.y = pos;
    }
    unsafe { call_orig(is) }
}

/// `shaper_z_calc_position` from `kin_shaper.c`.
fn shaper_z_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: see `shaper_x_calc_position`.
    let is = unsafe { from_sk(sk) };
    if is.sp[2].num_pulses == 0 {
        if is.orig_sk.is_null() {
            return move_get_coord(m, move_time).z;
        }
        let orig = unsafe { &*is.orig_sk };
        return (orig.calc_position_cb)(orig, m, move_time);
    }
    let pos = is.calc_position(m, 2, move_time, &is.sp[2]);
    unsafe {
        (*is.m.get()).start_pos.z = pos;
    }
    unsafe { call_orig(is) }
}

/// `shaper_xyz_calc_position` from `kin_shaper.c`: the plain coordinate,
/// with each shaped axis replaced by its shaped position.
fn shaper_xyz_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: see `shaper_x_calc_position`.
    let is = unsafe { from_sk(sk) };
    if is.orig_sk.is_null() {
        return move_get_coord(m, move_time).x;
    }
    unsafe {
        (*is.m.get()).start_pos = move_get_coord(m, move_time);
    }
    if is.sp[0].num_pulses != 0 {
        let pos = is.calc_position(m, 0, move_time, &is.sp[0]);
        unsafe {
            (*is.m.get()).start_pos.x = pos;
        }
    }
    if is.sp[1].num_pulses != 0 {
        let pos = is.calc_position(m, 1, move_time, &is.sp[1]);
        unsafe {
            (*is.m.get()).start_pos.y = pos;
        }
    }
    if is.sp[2].num_pulses != 0 {
        let pos = is.calc_position(m, 2, move_time, &is.sp[2]);
        unsafe {
            (*is.m.get()).start_pos.z = pos;
        }
    }
    unsafe { call_orig(is) }
}

/// `shaper_commanded_pos_post_fixup` from `kin_shaper.c`: forward the
/// commanded position to the original kinematics, letting it apply its
/// own post fixup, then copy the result back.
fn shaper_commanded_pos_post_fixup(sk: &mut StepperKinematics) {
    // SAFETY: the solver holds `&mut` of the shaper's `sk` for the whole
    // range; the shaper is alive and not otherwise aliased, as in the C.
    let is = unsafe { from_sk_mut(sk) };
    if is.orig_sk.is_null() {
        return;
    }
    let pos = is.sk.commanded_pos;
    let new_pos = unsafe {
        let orig = &mut *is.orig_sk;
        orig.commanded_pos = pos;
        if let Some(post_cb) = orig.post_cb {
            post_cb(orig);
        }
        orig.commanded_pos
    };
    is.sk.commanded_pos = new_pos;
}

/// Pick the `calc_position_cb` for the C if-chain over the original
/// kinematics' active flags, as in `input_shaper_set_sk` and
/// `input_shaper_update_sk`.
fn pick_calc_cb(kin_flags: i32) -> StepperCalcCb {
    if kin_flags == AF_X {
        shaper_x_calc_position
    } else if kin_flags == AF_Y {
        shaper_y_calc_position
    } else if kin_flags == AF_Z {
        shaper_z_calc_position
    } else {
        shaper_xyz_calc_position
    }
}

/// `input_shaper_alloc` from `kin_shaper.c`: a zeroed shaper whose scratch
/// move has `move_t = 2 * DUMMY_T`. The C has no callback installed yet;
/// the Rust `StepperKinematics` cannot hold a null callback, so the
/// xyz variant is installed until a wrapped kinematics is attached (it
/// only falls back to a plain coordinate lookup while unattached).
pub fn input_shaper_alloc() -> Box<InputShaper> {
    Box::new(InputShaper {
        sk: StepperKinematics::new(shaper_xyz_calc_position, 0),
        orig_sk: std::ptr::null_mut(),
        m: Box::new(UnsafeCell::new(Move {
            move_t: 2.0 * DUMMY_T,
            ..Move::default()
        })),
        sp: [ShaperPulses::default(); 3],
    })
}
/// `input_shaper_set_sk` from `kin_shaper.c`: attach the original
/// kinematics. Returns `0` on success and `-1` when the original has no
/// active axis (leaving the shaper untouched), exactly as in the C.
///
/// # Safety
///
/// `orig_sk` must point to a live original kinematics that outlives this
/// shaper (same as the C).
pub unsafe fn input_shaper_set_sk(is: &mut InputShaper, orig_sk: *mut StepperKinematics) -> i32 {
    // SAFETY: the caller guarantees `orig_sk` points to a live original
    // kinematics that outlives the shaper (same as the C).
    let flags = unsafe { (*orig_sk).active_flags };
    if flags == AF_X {
        is.sk.calc_position_cb = shaper_x_calc_position;
    } else if flags == AF_Y {
        is.sk.calc_position_cb = shaper_y_calc_position;
    } else if flags == AF_Z {
        is.sk.calc_position_cb = shaper_z_calc_position;
    } else if flags & (AF_X | AF_Y | AF_Z) != 0 {
        is.sk.calc_position_cb = shaper_xyz_calc_position;
    } else {
        return -1;
    }
    // SAFETY: see above.
    let orig = unsafe { &*orig_sk };
    is.sk.active_flags = orig.active_flags;
    is.orig_sk = orig_sk;
    is.sk.commanded_pos = orig.commanded_pos;
    is.sk.last_flush_time = orig.last_flush_time;
    is.sk.last_move_time = orig.last_move_time;
    if orig.post_cb.is_some() {
        is.sk.post_cb = Some(shaper_commanded_pos_post_fixup);
    }
    0
}

/// `input_shaper_update_sk` from `kin_shaper.c`: re-read the wrapped
/// kinematics' active flags (which may have changed behind the shaper's
/// back) and refresh the step generation windows.
pub fn input_shaper_update_sk(is: &mut InputShaper) {
    // SAFETY: `input_shaper_set_sk` attached the original kinematics;
    // the caller guarantees it is still alive.
    let kin_flags = unsafe { (*is.orig_sk).active_flags & (AF_X | AF_Y | AF_Z) };
    is.sk.calc_position_cb = pick_calc_cb(kin_flags);
    // SAFETY: see above.
    is.sk.active_flags = unsafe { (*is.orig_sk).active_flags };
    is.note_generation_time();
}

/// `input_shaper_set_shaper_params` from `kin_shaper.c`: configure one
/// axis' pulse train. Returns `0` on success and `-1` for an unknown axis
/// or invalid pulses. When the axis is inactive on the wrapped kinematics
/// the update is ignored (status `0`), exactly as in the C.
pub fn input_shaper_set_shaper_params(
    is: &mut InputShaper,
    axis: u8,
    n: i32,
    a: &[f64],
    t: &[f64],
) -> i32 {
    let axis_ind = i32::from(axis) - i32::from(b'x');
    if !(0..3).contains(&axis_ind) {
        return -1;
    }
    let mut status = 0;
    // SAFETY: `input_shaper_set_sk` attached the original kinematics, but
    // being called without one would dereference null in the C; the port
    // treats it as an inactive axis instead of crashing.
    let active = !is.orig_sk.is_null()
        && unsafe { (*is.orig_sk).active_flags & KIN_FLAGS[axis_ind as usize] } != 0;
    if active {
        status = is.sp[axis_ind as usize].init_shaper(n, a, t);
        is.note_generation_time();
    }
    status
}
