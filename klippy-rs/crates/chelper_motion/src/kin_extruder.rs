//! Port of `klippy/chelper/kin_extruder.c`.
//!
//! The extruder stepper position without pressure advance is
//! `start_pos.x + move_get_distance(m, move_time)`. With pressure
//! advance it is the triangularly weighted average over
//! `[move_time - hst, move_time + hst]` (`hst = smooth_time / 2`) of
//! `position(t) + pressure_advance * velocity(t)`, computed with the
//! closed-form definite integrals below. Only moves with
//! `axes_r.y != 0` ("can pressure advance", the flag `extruder.py`
//! writes into the extruder trapq) use the pressure-advance value
//! whose `active_print_time` is the last one not after the move's
//! `print_time` (the first list entry if all are later).
//!
//! [`ExtruderStepper`] mirrors `struct extruder_stepper`: the
//! [`StepperKinematics`] (`sk`) is embedded first, exactly as in C,
//! so the [`StepperCalcCb`] ([`extruder_calc_position`]) recovers the
//! rest of the state from the `sk` pointer with the `container_of`
//! pattern. The allocation is the `Box` returned by
//! [`extruder_stepper_alloc`]; keep the box address stable and drive
//! the solver through its `sk` field. The C `sk` pointer is only
//! meaningful for such an allocation, as in C.
//!
//! Neighbour moves are found in the trapq attached to `sk` (the
//! solver passes the current `Move` by value, so pointer identity is
//! unavailable): the callback rebuilds the solver's move list
//! (`[head sentinel, moves..., tail sentinel]`) and locates the
//! current move by exact bit equality of all its fields. A move that
//! is not in that list (the synthetic move behind
//! `itersolve_calc_position_from_coord`, or no trapq attached) is
//! treated as isolated: only its own segment contributes. That
//! matches the C for the synthetic move while the smoothing window
//! stays inside it (`klippy` limits `smooth_time` to 0.200, so this
//! is always the case for that call path).
//!
//! Traversal stops at the list ends. The C prev-walk can step past
//! the head sentinel (whose `move_t` is 0) into heap memory when the
//! window extends before the first real move, and the C next-walk can
//! step past the tail sentinel for a window larger than the remaining
//! queue; both are undefined behaviour whose values depend on heap
//! contents. The port includes the sentinels and stops there instead.
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order.

use crate::itersolve::{StepperKinematics, AF_X};
use crate::trapq::{move_get_distance, Move, MoveList};

/// `struct pa_params` from `kin_extruder.c`: one pressure-advance
/// setting, active from `active_print_time` until the next entry.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PaParams {
    /// Pressure advance (`pa->pressure_advance`).
    pub pressure_advance: f64,
    /// Print time the entry becomes active (`pa->active_print_time`).
    pub active_print_time: f64,
}

/// `struct extruder_stepper` from `kin_extruder.c`, with the
/// [`StepperKinematics`] (`sk`) embedded first.
///
/// `pa_list` is oldest first (C: appended with `list_add_tail`) and
/// always holds at least one entry.
#[repr(C)]
pub struct ExtruderStepper {
    /// The solver state (`sk`).
    pub sk: StepperKinematics,
    /// Pressure-advance entries (`pa_list`), oldest first.
    pub pa_list: Vec<PaParams>,
    /// `half_smooth_time` (`smooth_time / 2`).
    pub half_smooth_time: f64,
    /// `inv_half_smooth_time2` (`1. / (hst * hst)`; stale while
    /// `half_smooth_time` is 0, as in C).
    pub inv_half_smooth_time2: f64,
}

impl ExtruderStepper {
    /// `extruder_set_pressure_advance` from `kin_extruder.c`.
    pub fn set_pressure_advance(
        &mut self,
        print_time: f64,
        pressure_advance: f64,
        smooth_time: f64,
    ) {
        let hst = smooth_time * 0.5;
        let old_hst = self.half_smooth_time;
        self.half_smooth_time = hst;
        self.sk.gen_steps_pre_active = hst;
        self.sk.gen_steps_post_active = hst;

        // Cleanup old pressure advance parameters.
        let cleanup_time = self.sk.last_flush_time
            - if old_hst > hst { old_hst } else { hst };
        while self.pa_list.len() > 1 {
            if self.pa_list[1].active_print_time >= cleanup_time {
                break;
            }
            self.pa_list.remove(0);
        }

        if hst == 0.0 {
            return;
        }
        self.inv_half_smooth_time2 = 1.0 / (hst * hst);

        if self.pa_list
            .last()
            .expect("pa_list always holds an entry")
            .pressure_advance
            == pressure_advance
        {
            // Retain old pa_params.
            return;
        }
        // Add new pressure advance parameters.
        self.pa_list.push(PaParams {
            pressure_advance,
            active_print_time: print_time,
        });
    }

    /// Extruder position at `move_time` into `m`
    /// (`extruder_calc_position` from `kin_extruder.c`). Neighbour
    /// moves come from the trapq attached to `sk` (see module docs).
    pub fn calc_position(&self, m: &Move, move_time: f64) -> f64 {
        let hst = self.half_smooth_time;
        if hst == 0.0 {
            // Pressure advance not enabled.
            return m.start_pos.x + move_get_distance(m, move_time);
        }
        // Apply pressure advance and average over smooth_time.
        let area = self.pa_area(m, move_time, hst);
        m.start_pos.x + area * self.inv_half_smooth_time2
    }

    /// `pa_range_integrate` over the solver's move list from the trapq
    /// attached to `sk`, walked in place; only `m` itself when no trapq
    /// is attached or `m` is not one of its moves.
    fn pa_area(&self, m: &Move, move_time: f64, hst: f64) -> f64 {
        if let Some(tq) = &self.sk.tq {
            let tq = tq.lock();
            let seq = tq.move_seq();
            if let Some(idx) = seq.find(m, |a, b| move_bits(a) == move_bits(b)) {
                return pa_range_integrate(&seq, idx, move_time, &self.pa_list, hst);
            }
        }
        pa_range_integrate(std::slice::from_ref(m), 0, move_time, &self.pa_list, hst)
    }
}

/// `extruder_stepper_alloc` from `kin_extruder.c`: a zeroed
/// kinematics with `calc_position_cb = extruder_calc_position`,
/// `active_flags = AF_X` and one zeroed `pa_params` entry.
pub fn extruder_stepper_alloc() -> Box<ExtruderStepper> {
    Box::new(ExtruderStepper {
        sk: StepperKinematics::new(extruder_calc_position, AF_X),
        pa_list: vec![PaParams::default()],
        half_smooth_time: 0.0,
        inv_half_smooth_time2: 0.0,
    })
}

/// `extruder_calc_position` from `kin_extruder.c`: the
/// [`StepperCalcCb`] for an [`ExtruderStepper`]'s `sk`. The state is
/// recovered from the `sk` pointer (`container_of` with offset 0, so
/// `sk` must be the first field of [`ExtruderStepper`]).
pub fn extruder_calc_position(sk: &StepperKinematics, m: &Move, move_time: f64) -> f64 {
    // SAFETY: `extruder_stepper_alloc` returns a `Box<ExtruderStepper>`
    // whose first field is `sk`; callers only pass the `sk` of such an
    // allocation, as in C where the callback receives the embedded
    // `sk` pointer.
    let es = unsafe { &*(core::ptr::from_ref(sk) as *const ExtruderStepper) };
    es.calc_position(m, move_time)
}

/// `extruder_integrate` from `kin_extruder.c`: the definite integral
/// of `position(t) = base + t * (start_v + t * half_accel)` over
/// `[start, end]`.
fn extruder_integrate(
    base: f64,
    start_v: f64,
    half_accel: f64,
    start: f64,
    end: f64,
) -> f64 {
    let half_v = 0.5 * start_v;
    let sixth_a = (1.0 / 3.0) * half_accel;
    let si = start * (base + start * (half_v + start * sixth_a));
    let ei = end * (base + end * (half_v + end * sixth_a));
    ei - si
}

/// `extruder_integrate_time` from `kin_extruder.c`: the definite
/// integral of `t * position(t)` over `[start, end]`.
fn extruder_integrate_time(
    base: f64,
    start_v: f64,
    half_accel: f64,
    start: f64,
    end: f64,
) -> f64 {
    let half_b = 0.5 * base;
    let third_v = (1.0 / 3.0) * start_v;
    let eighth_a = 0.25 * half_accel;
    let si = start * start * (half_b + start * (third_v + start * eighth_a));
    let ei = end * end * (half_b + end * (third_v + end * eighth_a));
    ei - si
}

/// `pa_move_integrate` from `kin_extruder.c`: the time-weighted
/// integral of the pressure-advanced position of `m` over
/// `[start, end]`, relative to `time_offset`.
fn pa_move_integrate(
    m: &Move,
    pa_list: &[PaParams],
    base: f64,
    start: f64,
    end: f64,
    time_offset: f64,
) -> f64 {
    let mut start = start;
    let mut end = end;
    if start < 0.0 {
        start = 0.0;
    }
    if end > m.move_t {
        end = m.move_t;
    }
    // Determine pressure_advance value.
    let can_pressure_advance = m.axes_r.y != 0.0;
    let mut pressure_advance = 0.0;
    if can_pressure_advance {
        let mut idx = pa_list.len() - 1;
        while pa_list[idx].active_print_time > m.print_time && idx != 0 {
            idx -= 1;
        }
        pressure_advance = pa_list[idx].pressure_advance;
    }
    // Calculate base position and velocity with pressure advance.
    let base = base + pressure_advance * m.start_v;
    let start_v = m.start_v + pressure_advance * 2.0 * m.half_accel;
    // Calculate definitive integral.
    let ha = m.half_accel;
    let iext = extruder_integrate(base, start_v, ha, start, end);
    let wgt_ext = extruder_integrate_time(base, start_v, ha, start, end);
    wgt_ext - time_offset * iext
}

/// `pa_range_integrate` from `kin_extruder.c`: the time-weighted
/// integral of the pressure-advanced position of `list[idx]` over
/// `[move_time - hst, move_time + hst]`, integrating over previous
/// and future moves as needed. Traversal stops at the list ends (the
/// head/tail sentinels are list members and are integrated when
/// reached; see the module docs on the C walk past the ends).
fn pa_range_integrate<L: MoveList + ?Sized>(
    list: &L,
    idx: usize,
    move_time: f64,
    pa_list: &[PaParams],
    hst: f64,
) -> f64 {
    // Calculate integral for the current move.
    let mut res = 0.0;
    let mut start = move_time - hst;
    let mut end = move_time + hst;
    let start_base = list.get(idx).start_pos.x;
    res += pa_move_integrate(list.get(idx), pa_list, 0.0, start, move_time, start);
    res -= pa_move_integrate(list.get(idx), pa_list, 0.0, move_time, end, end);
    // Integrate over previous moves.
    let mut prev = idx;
    while start < 0.0 {
        if prev == 0 {
            break;
        }
        prev -= 1;
        let pm = list.get(prev);
        start += pm.move_t;
        let base = pm.start_pos.x - start_base;
        res += pa_move_integrate(pm, pa_list, base, start, pm.move_t, start);
        if prev == 0 {
            break;
        }
    }
    // Integrate over future moves.
    let mut cur = idx;
    while end > list.get(cur).move_t {
        end -= list.get(cur).move_t;
        cur += 1;
        let base = list.get(cur).start_pos.x - start_base;
        res -= pa_move_integrate(list.get(cur), pa_list, base, 0.0, end, end);
        if cur + 1 >= list.len() {
            break;
        }
    }
    res
}

/// Exact bit equality of every field, used to re-locate the move the
/// solver hands the callback (a copy of its list entry).
fn move_bits(m: &Move) -> [u64; 10] {
    [
        m.print_time.to_bits(),
        m.move_t.to_bits(),
        m.start_v.to_bits(),
        m.half_accel.to_bits(),
        m.start_pos.x.to_bits(),
        m.start_pos.y.to_bits(),
        m.start_pos.z.to_bits(),
        m.axes_r.x.to_bits(),
        m.axes_r.y.to_bits(),
        m.axes_r.z.to_bits(),
    ]
}
