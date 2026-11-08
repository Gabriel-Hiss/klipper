//! Port of `klippy/chelper/itersolve.c` / `itersolve.h`.
//!
//! The solver turns trapezoidal moves into step times. For each portion of
//! a move it solves `calc_position_cb(sk, m, t) == target` with the secant
//! method, falling back to bisection inside a `(low_time, high_time)`
//! bracket, detects direction changes, and submits each found step through
//! [`Stepcompress::append`], committing the previous pending step through
//! [`Stepcompress::commit`] whenever the guess is still too far from the
//! target.
//!
//! [`StepperKinematics`] mirrors `struct stepper_kinematics` from
//! `itersolve.h`, with the same field names and defaults after a zeroed C
//! allocation. The C callback pointers are modelled as function-pointer
//! types ([`StepperCalcCb`] / [`StepperPostCb`]), exactly like the C
//! typedefs, so a `kin_*` port supplies a plain function (stateless
//! kinematics such as Cartesian/CoreXY/CoreXZ) or a function recovering
//! its own state from the `sk` pointer the same way the C `container_of`
//! pattern does. A kinematics module that needs extra state (delta tower
//! positions, shaper pulses, extruder pressure-advance) allocates its
//! own struct with `StepperKinematics` embedded first and recovers the
//! state from the `sk` pointer it receives, mirroring the C `container_of`
//! pattern; closure captures are never used in the solver path.
//!
//! The C `struct trapq *tq` is shared by every stepper of a kinematics
//! object (`klippy/stepper.py` passes `toolhead.get_trapq()` to each
//! axis), so the field is a [`SharedTrapq`] (`Arc<Mutex<Trapq>>`): the
//! step generation threads of `steppersync.c` read it concurrently. The C
//! `struct stepcompress *sc` field is unused by this unit
//! (`itersolve_generate_steps` receives the compressor as an argument),
//! but is kept for field-by-field parity with the C struct.
//!
//! Traversal follows the C intrusive list, including the head/tail
//! sentinels: [`Trapq::moves`] are iterated between
//! [`Trapq::head_sentinel`] and [`Trapq::tail_sentinel`], reading the
//! tail sentinel exactly as left by the caller (the caller in
//! `steppersyncmgr_gen_steps` runs `trapq_check_sentinels` first; a caller
//! who does not gets the dirty-tail behaviour of the C, because the
//! sentinel is copied as stored here too).
//!
//! The fork's `[DEBUG-motion]` error capture (`itersolve_debug_dump`) is
//! not reproduced; it only writes log lines on the `stepcompress_append`
//! / `stepcompress_commit` error paths and does not change the emitted
//! step times, the stepper state, or the return codes (the same choice
//! `stepcompress.rs` makes for its debug rings). The commit return value
//! is ignored, as in the upstream solver.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order (see
//! `parity/maps/map-motion.md` F4).

use crate::stepcompress::{SharedStepcompress, Stepcompress};
use crate::trapq::{Coord, Move, SharedTrapq};

/// `AF_X` from `itersolve.h`: the X axis is active for this stepper.
pub const AF_X: i32 = 1 << 0;
/// `AF_Y` from `itersolve.h`: the Y axis is active for this stepper.
pub const AF_Y: i32 = 1 << 1;
/// `AF_Z` from `itersolve.h`: the Z axis is active for this stepper.
pub const AF_Z: i32 = 1 << 2;

/// `sk_calc_callback` from `itersolve.h`: the stepper position at
/// `move_time` into `m`. Implementations read `sk` (for example its
/// `commanded_pos`, as the polar angle callback does) but must not modify
/// the move.
pub type StepperCalcCb = fn(&StepperKinematics, &Move, f64) -> f64;
/// `sk_post_callback` from `itersolve.h`: run after the commanded
/// position is updated at the end of a step range (polar angle
/// normalisation, shaper forwarding).
pub type StepperPostCb = fn(&mut StepperKinematics);

/// `struct stepper_kinematics` from `itersolve.h`, with the same field
/// names. All `f64` state starts at `0.0` after the C zeroed allocation;
/// `tq`/`sc`/`post_cb` start as `None`.
pub struct StepperKinematics {
    /// Distance per step (set by [`StepperKinematics::set_trapq`]).
    pub step_dist: f64,
    /// Last commanded stepper position.
    pub commanded_pos: f64,
    /// Unused by `itersolve` (the compressor is passed to
    /// [`StepperKinematics::generate_steps`]); kept for field parity with
    /// the C struct.
    pub sc: Option<SharedStepcompress>,

    /// Print time up to which steps were flushed (last `flush_time`).
    pub last_flush_time: f64,
    /// Print time up to which full moves were stepped.
    pub last_move_time: f64,
    /// The shared move queue, if any.
    pub tq: Option<SharedTrapq>,
    /// `AF_X`/`AF_Y`/`AF_Z` bit mask of active axes.
    pub active_flags: i32,
    /// Seconds of steps generated before stepper activity.
    pub gen_steps_pre_active: f64,
    /// Seconds of steps generated after stepper activity.
    pub gen_steps_post_active: f64,

    /// Position solver for this kinematics (never null in the Rust port;
    /// a null C callback would only crash the reference and is outside
    /// what `klippy` can produce).
    pub calc_position_cb: StepperCalcCb,
    /// Optional post-range fixup (`None` mirrors a null C pointer).
    pub post_cb: Option<StepperPostCb>,
}

impl StepperKinematics {
    /// A zeroed stepper kinematics for `calc_position_cb` over
    /// `active_flags` (`cartesian_stepper_alloc` and friends).
    pub fn new(calc_position_cb: StepperCalcCb, active_flags: i32) -> Self {
        Self {
            step_dist: 0.0,
            commanded_pos: 0.0,
            sc: None,
            last_flush_time: 0.0,
            last_move_time: 0.0,
            tq: None,
            active_flags,
            gen_steps_pre_active: 0.0,
            gen_steps_post_active: 0.0,
            calc_position_cb,
            post_cb: None,
        }
    }

    /// `itersolve_generate_steps` from `itersolve.c`.
    pub fn generate_steps(&mut self, sc: &mut Stepcompress, flush_time: f64) -> i32 {
        let last_flush_time = self.last_flush_time;
        self.last_flush_time = flush_time;
        let Some(tq) = self.tq.clone() else {
            return 0;
        };
        let list = {
            let tq = tq.lock();
            let mut list: Vec<Move> = Vec::with_capacity(tq.moves.len() + 2);
            list.push(*tq.head_sentinel());
            list.extend(tq.moves.iter().copied());
            list.push(*tq.tail_sentinel());
            list
        };
        let mut m_idx = 0usize;
        while m_idx + 1 < list.len()
            && last_flush_time >= list[m_idx].print_time + list[m_idx].move_t
        {
            m_idx += 1;
        }
        let mut force_steps_time = self.last_move_time + self.gen_steps_post_active;
        let mut skip_count: i32 = 0;
        loop {
            if m_idx >= list.len() {
                return 0;
            }
            let move_start = list[m_idx].print_time;
            let move_end = move_start + list[m_idx].move_t;
            if check_active(self.active_flags, &list[m_idx]) {
                if skip_count != 0 && self.gen_steps_pre_active != 0.0 {
                    // Generate steps leading up to stepper activity.
                    let mut abs_start = move_start - self.gen_steps_pre_active;
                    if abs_start < last_flush_time {
                        abs_start = last_flush_time;
                    }
                    if abs_start < force_steps_time {
                        abs_start = force_steps_time;
                    }
                    let mut pm_idx = m_idx as isize - 1;
                    while {
                        skip_count -= 1;
                        skip_count != 0
                            && pm_idx >= 0
                            && list[pm_idx as usize].print_time > abs_start
                    } {
                        pm_idx -= 1;
                    }
                    if pm_idx >= 0 {
                        loop {
                            let ret = self.gen_steps_range(
                                sc,
                                &list[pm_idx as usize],
                                abs_start,
                                flush_time,
                            );
                            if ret != 0 {
                                return ret;
                            }
                            pm_idx += 1;
                            if pm_idx as usize == m_idx {
                                break;
                            }
                        }
                    }
                }
                // Generate steps for this move.
                let ret =
                    self.gen_steps_range(sc, &list[m_idx], last_flush_time, flush_time);
                if ret != 0 {
                    return ret;
                }
                if move_end >= flush_time {
                    self.last_move_time = flush_time;
                    return 0;
                }
                skip_count = 0;
                self.last_move_time = move_end;
                force_steps_time = self.last_move_time + self.gen_steps_post_active;
            } else {
                if move_start < force_steps_time {
                    // Generate steps just past stepper activity.
                    let mut abs_end = force_steps_time;
                    if abs_end > flush_time {
                        abs_end = flush_time;
                    }
                    let ret = self.gen_steps_range(sc, &list[m_idx], last_flush_time, abs_end);
                    if ret != 0 {
                        return ret;
                    }
                    skip_count = 1;
                } else {
                    // This move doesn't impact this stepper.
                    skip_count += 1;
                }
                if flush_time + self.gen_steps_pre_active <= move_end {
                    return 0;
                }
            }
            m_idx += 1;
        }
    }

    /// `itersolve_gen_steps_range` from `itersolve.c`: generate step times
    /// for the `[abs_start, abs_end]` portion of `m`.
    fn gen_steps_range(
        &mut self,
        sc: &mut Stepcompress,
        m: &Move,
        abs_start: f64,
        abs_end: f64,
    ) -> i32 {
        let calc_position_cb = self.calc_position_cb;
        let half_step = 0.5 * self.step_dist;
        let mut start = abs_start - m.print_time;
        let mut end = abs_end - m.print_time;
        if start < 0.0 {
            start = 0.0;
        }
        if end > m.move_t {
            end = m.move_t;
        }
        let mut old_guess = TimePos {
            time: start,
            position: self.commanded_pos,
        };
        let mut guess = old_guess;
        let mut sdir = sc.get_step_dir();
        let mut is_dir_change = false;
        let mut have_bracket = false;
        let mut check_oscillate = false;
        let mut target = self.commanded_pos
            + if sdir != 0 { half_step } else { -half_step };
        let mut last_time = start;
        let mut low_time = start;
        let mut high_time = start + SEEK_TIME_RESET;
        if high_time > end {
            high_time = end;
        }
        loop {
            // Use the "secant method" to guess a new time from previous guesses.
            let guess_dist = guess.position - target;
            let og_dist = old_guess.position - target;
            let mut next_time =
                (old_guess.time * guess_dist - guess.time * og_dist) / (guess_dist - og_dist);
            if !(next_time > low_time && next_time < high_time) {
                // Next guess is outside bounds checks, or NaN.
                if have_bracket {
                    // A poor guess: fall back to bisection.
                    next_time = (low_time + high_time) * 0.5;
                    check_oscillate = false;
                } else if guess.time >= end {
                    // No more steps present in requested time range.
                    break;
                } else {
                    // Might be a poor guess: limit to exponential search.
                    next_time = high_time;
                    high_time = 2.0 * high_time - last_time;
                    if high_time > end {
                        high_time = end;
                    }
                }
            }
            // Calculate position at next_time guess.
            old_guess = guess;
            guess.time = next_time;
            guess.position = calc_position_cb(self, m, next_time);
            let guess_dist = guess.position - target;
            if guess_dist.abs() > 0.000000001 {
                // Guess does not look close enough: update bounds.
                let rel_dist = if sdir != 0 { guess_dist } else { -guess_dist };
                if rel_dist > 0.0 {
                    // Found position past target, so step is definitely present.
                    if have_bracket && old_guess.time <= low_time {
                        if check_oscillate {
                            // Force bisect next to avoid persistent oscillations.
                            old_guess = guess;
                        }
                        check_oscillate = true;
                    }
                    high_time = guess.time;
                    have_bracket = true;
                } else if rel_dist < -(half_step + half_step + 0.000000010) {
                    // Found direction change.
                    sdir = if sdir != 0 { 0 } else { 1 };
                    target = if sdir != 0 {
                        target + half_step + half_step
                    } else {
                        target - half_step - half_step
                    };
                    low_time = last_time;
                    high_time = guess.time;
                    is_dir_change = true;
                    have_bracket = true;
                    check_oscillate = false;
                } else {
                    low_time = guess.time;
                }
                if !have_bracket || high_time - low_time > 0.000000001 {
                    if !is_dir_change && rel_dist >= -half_step {
                        // Avoid rollback if stepper fully reaches step position.
                        // Preserve the original ignored commit return value;
                        // the fork only captured it for the debug dump.
                        let _ = sc.commit();
                    }
                    // Guess is not close enough: guess again with new time.
                    continue;
                }
            }
            // Found next step: submit it.
            let ret = sc.append(sdir, m.print_time, guess.time);
            if ret != 0 {
                return ret;
            }
            target = if sdir != 0 {
                target + half_step + half_step
            } else {
                target - half_step - half_step
            };
            // Reset bounds checking.
            let mut seek_time_delta = 1.5 * (guess.time - last_time);
            if seek_time_delta < 0.000000001 {
                seek_time_delta = 0.000000001;
            }
            if is_dir_change && seek_time_delta > SEEK_TIME_RESET {
                seek_time_delta = SEEK_TIME_RESET;
            }
            last_time = guess.time;
            low_time = guess.time;
            high_time = guess.time + seek_time_delta;
            if high_time > end {
                high_time = end;
            }
            is_dir_change = false;
            have_bracket = false;
            check_oscillate = false;
        }
        self.commanded_pos = target
            - if sdir != 0 { half_step } else { -half_step };
        if let Some(post_cb) = self.post_cb {
            post_cb(self);
        }
        0
    }

    /// `itersolve_check_active` from `itersolve.c`.
    pub fn check_active(&mut self, flush_time: f64) -> f64 {
        let Some(tq) = self.tq.clone() else {
            return 0.0;
        };
        {
            let mut tq = tq.lock();
            tq.check_sentinels();
        }
        let list = {
            let tq = tq.lock();
            let mut list: Vec<Move> = Vec::with_capacity(tq.moves.len() + 2);
            list.push(*tq.head_sentinel());
            list.extend(tq.moves.iter().copied());
            list.push(*tq.tail_sentinel());
            list
        };
        let mut m_idx = 0usize;
        while m_idx + 1 < list.len()
            && self.last_flush_time >= list[m_idx].print_time + list[m_idx].move_t
        {
            m_idx += 1;
        }
        loop {
            if m_idx >= list.len() {
                return 0.0;
            }
            let m = &list[m_idx];
            if check_active(self.active_flags, m) {
                return m.print_time;
            }
            if flush_time <= m.print_time + m.move_t {
                return 0.0;
            }
            m_idx += 1;
        }
    }

    /// `itersolve_is_active_axis` from `itersolve.c`.
    pub fn is_active_axis(&self, axis: u8) -> i32 {
        if !(b'x'..=b'z').contains(&axis) {
            return 0;
        }
        i32::from((self.active_flags & (AF_X << u32::from(axis - b'x'))) != 0)
    }

    /// `itersolve_set_trapq` from `itersolve.c`.
    pub fn set_trapq(&mut self, tq: Option<SharedTrapq>, step_dist: f64) {
        self.tq = tq;
        self.step_dist = step_dist;
    }

    /// `itersolve_get_trapq` from `itersolve.c`.
    pub fn get_trapq(&self) -> Option<SharedTrapq> {
        self.tq.clone()
    }

    /// `itersolve_calc_position_from_coord` from `itersolve.c`.
    pub fn calc_position_from_coord(&self, x: f64, y: f64, z: f64) -> f64 {
        let m = Move {
            start_pos: Coord::new(x, y, z),
            move_t: 1000.0,
            ..Move::default()
        };
        (self.calc_position_cb)(self, &m, 500.0)
    }

    /// `itersolve_set_position` from `itersolve.c`.
    pub fn set_position(&mut self, x: f64, y: f64, z: f64) {
        self.commanded_pos = self.calc_position_from_coord(x, y, z);
    }

    /// `itersolve_get_commanded_pos` from `itersolve.c`.
    pub fn get_commanded_pos(&self) -> f64 {
        self.commanded_pos
    }

    /// `itersolve_get_gen_steps_pre_active` from `itersolve.c`.
    pub fn get_gen_steps_pre_active(&self) -> f64 {
        self.gen_steps_pre_active
    }

    /// `itersolve_get_gen_steps_post_active` from `itersolve.c`.
    pub fn get_gen_steps_post_active(&self) -> f64 {
        self.gen_steps_post_active
    }
}

/// `struct timepos` from `itersolve.c`.
#[derive(Clone, Copy)]
struct TimePos {
    time: f64,
    position: f64,
}

/// `SEEK_TIME_RESET` from `itersolve.c`.
const SEEK_TIME_RESET: f64 = 0.000100;

/// `check_active` from `itersolve.c`: whether `m` is likely to cause
/// movement on a stepper with `active_flags`.
fn check_active(active_flags: i32, m: &Move) -> bool {
    (active_flags & AF_X != 0 && m.axes_r.x != 0.0)
        || (active_flags & AF_Y != 0 && m.axes_r.y != 0.0)
        || (active_flags & AF_Z != 0 && m.axes_r.z != 0.0)
}
