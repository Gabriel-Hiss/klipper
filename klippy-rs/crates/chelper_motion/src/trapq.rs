//! Port of `klippy/chelper/trapq.c` / `trapq.h`.
//!
//! The trapezoid velocity queue used by [`crate`] kinematics and the
//! stepper code. This module owns the types shared with the `itersolve`
//! and `kinematics` ports.
//!
//! # Types (field names match the C structs)
//!
//! - [`Coord`] mirrors `struct coord`: a point or direction in XYZ
//!   space (`x`, `y`, `z`). In a [`Move`], `start_pos` is where the move
//!   begins and `axes_r` is the unit direction (per-axis ratio) of the
//!   move; for extruder trapqs `axes_r.x` carries the extrude ratio and
//!   `axes_r.y` carries the pressure-advance flag.
//! - [`Move`] mirrors `struct move`: one trapezoidal segment with
//!   `print_time` (when it starts), `move_t` (duration), `start_v`
//!   (velocity at the start), `half_accel` (half the acceleration, so
//!   `accel = 2 * half_accel`), `start_pos`, and `axes_r`. Null moves
//!   (time-gap padding) have `start_v == 0` and `half_accel == 0` and
//!   are never reported by [`Trapq::extract_old`].
//! - [`PullMove`] mirrors `struct pull_move`: the view of a move handed
//!   out by [`Trapq::extract_old` and to the kinematics code, with
//!   `accel` instead of `half_accel` and flat `start_x`, `start_y`,
//!   `start_z`, `x_r`, `y_r`, `z_r` fields.
//! - [`Trapq`] mirrors `struct trapq`: the `moves` list (oldest first,
//!   with head/tail sentinels) and the `history` list (newest first),
//!   both modelled as [`VecDeque`]s.
//!
//! The position at `move_time` into a move is
//! [`move_get_coord`] (see [`Move::get_distance`] for the scalar
//! distance); [`PullMove`] provides the same value from the pulled
//! fields via [`PullMove::get_coord`].
//!
//! Floating point matches the C: `f64` throughout, no `mul_add`, no
//! reassociation, operations evaluated in the exact source order.

use std::collections::VecDeque;

/// `struct coord` from `trapq.h`: an XYZ point or direction.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Coord {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

impl Coord {
    /// A coordinate from its three components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// `struct move` from `trapq.h`: one trapezoidal velocity segment.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Move {
    /// Print time the segment starts at.
    pub print_time: f64,
    /// Duration of the segment.
    pub move_t: f64,
    /// Velocity at the start of the segment.
    pub start_v: f64,
    /// Half the acceleration (`accel = 2 * half_accel`).
    pub half_accel: f64,
    /// Position at the start of the segment.
    pub start_pos: Coord,
    /// Unit direction of the segment (per-axis ratio).
    pub axes_r: Coord,
}

impl Move {
    /// Distance moved `move_time` seconds into this move:
    /// `(start_v + half_accel * move_time) * move_time`, in this order.
    pub fn get_distance(&self, move_time: f64) -> f64 {
        move_get_distance(self, move_time)
    }

    /// XYZ coordinates `move_time` seconds into this move.
    pub fn get_coord(&self, move_time: f64) -> Coord {
        move_get_coord(self, move_time)
    }
}

/// `move_get_distance` from `trapq.c`.
pub fn move_get_distance(m: &Move, move_time: f64) -> f64 {
    (m.start_v + m.half_accel * move_time) * move_time
}

/// `move_get_coord` from `trapq.c`.
pub fn move_get_coord(m: &Move, move_time: f64) -> Coord {
    let move_dist = move_get_distance(m, move_time);
    Coord::new(
        m.start_pos.x + m.axes_r.x * move_dist,
        m.start_pos.y + m.axes_r.y * move_dist,
        m.start_pos.z + m.axes_r.z * move_dist,
    )
}

/// `struct pull_move` from `trapq.h`: a move as handed out by
/// [`Trapq::extract_old` and the kinematics code.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PullMove {
    /// Print time the segment starts at.
    pub print_time: f64,
    /// Duration of the segment.
    pub move_t: f64,
    /// Velocity at the start of the segment.
    pub start_v: f64,
    /// Acceleration (`2 * half_accel` of the `Move`).
    pub accel: f64,
    /// X of the start position.
    pub start_x: f64,
    /// Y of the start position.
    pub start_y: f64,
    /// Z of the start position.
    pub start_z: f64,
    /// X component of the unit direction.
    pub x_r: f64,
    /// Y component of the unit direction.
    pub y_r: f64,
    /// Z component of the unit direction.
    pub z_r: f64,
}

impl PullMove {
    /// Distance moved `move_time` seconds into this move, from the pulled
    /// fields: `(start_v + 0.5 * accel * move_time) * move_time`.
    pub fn get_distance(&self, move_time: f64) -> f64 {
        (self.start_v + 0.5 * self.accel * move_time) * move_time
    }

    /// XYZ coordinates `move_time` seconds into this move.
    pub fn get_coord(&self, move_time: f64) -> Coord {
        let move_dist = self.get_distance(move_time);
        Coord::new(
            self.start_x + self.x_r * move_dist,
            self.start_y + self.y_r * move_dist,
            self.start_z + self.z_r * move_dist,
        )
    }
}

/// `NEVER_TIME` from `trapq.c`: "no time" sentinel value.
pub const NEVER_TIME: f64 = 9999999999999999.9;
/// `MAX_NULL_MOVE` from `trapq.c`: the longest first null move.
pub const MAX_NULL_MOVE: f64 = 1.0;

/// A trapq shared between the toolhead, every stepper that reads it and
/// the step generation threads (`steppersync.c` runs one pthread per
/// syncemitter, all reading the same trapq while the main thread waits).
pub type SharedTrapq = std::sync::Arc<kos::sync::Mutex<Trapq>>;

/// `struct trapq` from `trapq.h`, plus the list sentinels.
pub struct Trapq {
    /// Pending moves, oldest first (C: `moves` list between the head
    /// sentinel `print_time = -1` and the tail sentinel).
    pub moves: VecDeque<Move>,
    /// Expired moves, newest first (C: `history` list).
    pub history: VecDeque<Move>,
    head_sentinel: Move,
    tail_sentinel: Move,
}

impl Trapq {
    /// `trapq_alloc` from `trapq.c`: an empty queue with head sentinel
    /// (`print_time = -1`) and tail sentinel
    /// (`print_time = move_t = NEVER_TIME`).
    pub fn new() -> Self {
        let head_sentinel = Move {
            print_time: -1.0,
            ..Move::default()
        };
        let tail_sentinel = Move {
            print_time: NEVER_TIME,
            move_t: NEVER_TIME,
            ..Move::default()
        };
        Self {
            moves: VecDeque::new(),
            history: VecDeque::new(),
            head_sentinel,
            tail_sentinel,
        }
    }

    /// The head sentinel move (`print_time = -1`).
    pub fn head_sentinel(&self) -> &Move {
        &self.head_sentinel
    }

    /// The tail sentinel move; `print_time` is `NEVER_TIME` except
    /// between [`Trapq::add_move`] and [`Trapq::check_sentinels`], where
    /// it is `0.0` ("needs update").
    pub fn tail_sentinel(&self) -> &Move {
        &self.tail_sentinel
    }

    /// `trapq_check_sentinels` from `trapq.c`: refresh the tail sentinel
    /// to the end of the last pending move.
    pub fn check_sentinels(&mut self) {
        if self.tail_sentinel.print_time != 0.0 {
            return;
        }
        let Some(m) = self.moves.back() else {
            self.tail_sentinel.print_time = NEVER_TIME;
            return;
        };
        self.tail_sentinel.print_time = m.print_time + m.move_t;
        self.tail_sentinel.start_pos = move_get_coord(m, m.move_t);
    }

    /// `trapq_add_move` from `trapq.c`: append a move, inserting a null
    /// move first when there is a gap before its `print_time`.
    pub fn add_move(&mut self, m: Move) {
        let prev = self.moves.back().unwrap_or(&self.head_sentinel);
        // C: prev->print_time + prev->move_t, and prev->print_time <= 0.
        let prev_start = prev.print_time;
        let prev_end_time = prev.print_time + prev.move_t;
        if prev_end_time < m.print_time {
            let mut null_move = Move {
                start_pos: m.start_pos,
                ..Move::default()
            };
            if prev_start <= 0.0 && m.print_time > MAX_NULL_MOVE {
                null_move.print_time = m.print_time - MAX_NULL_MOVE;
            } else {
                null_move.print_time = prev_end_time;
            }
            null_move.move_t = m.print_time - null_move.print_time;
            self.moves.push_back(null_move);
        }
        self.moves.push_back(m);
        self.tail_sentinel.print_time = 0.0;
    }

    /// `trapq_append` from `trapq.c`: queue the accel/cruise/decel
    /// segments of one move, chaining their start positions through
    /// [`move_get_coord`].
    #[allow(clippy::too_many_arguments)]
    pub fn append(
        &mut self,
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
        let mut print_time = print_time;
        let mut start_pos = Coord::new(start_pos_x, start_pos_y, start_pos_z);
        let axes_r = Coord::new(axes_r_x, axes_r_y, axes_r_z);
        if accel_t != 0.0 {
            let m = Move {
                print_time,
                move_t: accel_t,
                start_v,
                half_accel: 0.5 * accel,
                start_pos,
                axes_r,
            };
            self.add_move(m);
            print_time += accel_t;
            start_pos = move_get_coord(&m, accel_t);
        }
        if cruise_t != 0.0 {
            let m = Move {
                print_time,
                move_t: cruise_t,
                start_v: cruise_v,
                half_accel: 0.0,
                start_pos,
                axes_r,
            };
            self.add_move(m);
            print_time += cruise_t;
            start_pos = move_get_coord(&m, cruise_t);
        }
        if decel_t != 0.0 {
            let m = Move {
                print_time,
                move_t: decel_t,
                start_v: cruise_v,
                half_accel: -0.5 * accel,
                start_pos,
                axes_r,
            };
            self.add_move(m);
        }
    }

    /// `trapq_finalize_moves` from `trapq.c`: move expired moves from
    /// `moves` to `history` (null moves are dropped) and free history
    /// entries older than `clear_history_time`.
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        while let Some(m) = self.moves.front() {
            if m.print_time + m.move_t > print_time {
                break;
            }
            let m = self.moves.pop_front().expect("front move exists");
            if m.start_v != 0.0 || m.half_accel != 0.0 {
                self.history.push_front(m);
            }
        }
        if self.moves.is_empty() {
            self.tail_sentinel.print_time = NEVER_TIME;
        }
        if self.history.is_empty() {
            return;
        }
        while self.history.len() > 1 {
            let Some(m) = self.history.back() else {
                break;
            };
            if m.print_time + m.move_t > clear_history_time {
                break;
            }
            self.history.pop_back();
        }
    }

    /// `trapq_set_position` from `trapq.c`: flush the queue, truncate or
    /// prune the history at `print_time`, and note the new position.
    pub fn set_position(&mut self, print_time: f64, pos_x: f64, pos_y: f64, pos_z: f64) {
        self.finalize_moves(NEVER_TIME, 0.0);
        while let Some(m) = self.history.front() {
            if m.print_time < print_time {
                if m.print_time + m.move_t > print_time {
                    let mut m = *m;
                    m.move_t = print_time - m.print_time;
                    *self.history.front_mut().expect("front history exists") = m;
                }
                break;
            }
            self.history.pop_front();
        }
        let m = Move {
            print_time,
            start_pos: Coord::new(pos_x, pos_y, pos_z),
            ..Move::default()
        };
        self.history.push_front(m);
    }

    /// `trapq_extract_old` from `trapq.c`: copy up to `max` moves that
    /// overlap `[start_time, end_time]` into `out`, most recent first
    /// across `moves` (newest to oldest) and then `history` (newest to
    /// oldest). Null moves in `moves` are skipped. Returns the count.
    pub fn extract_old(
        &self,
        out: &mut [PullMove],
        max: i32,
        start_time: f64,
        end_time: f64,
    ) -> i32 {
        let mut res: i32 = 0;
        for m in self.moves.iter().rev() {
            if start_time >= m.print_time + m.move_t || res >= max {
                break;
            }
            if end_time <= m.print_time || (m.start_v == 0.0 && m.half_accel == 0.0) {
                continue;
            }
            if (res as usize) < out.len() {
                out[res as usize] = copy_pull_move(m);
            }
            res += 1;
        }
        for m in self.history.iter() {
            if start_time >= m.print_time + m.move_t || res >= max {
                break;
            }
            if end_time <= m.print_time {
                continue;
            }
            if (res as usize) < out.len() {
                out[res as usize] = copy_pull_move(m);
            }
            res += 1;
        }
        res
    }
}

impl Default for Trapq {
    fn default() -> Self {
        Self::new()
    }
}

/// `copy_pull_move` from `trapq.c`.
fn copy_pull_move(m: &Move) -> PullMove {
    PullMove {
        print_time: m.print_time,
        move_t: m.move_t,
        start_v: m.start_v,
        accel: 2.0 * m.half_accel,
        start_x: m.start_pos.x,
        start_y: m.start_pos.y,
        start_z: m.start_pos.z,
        x_r: m.axes_r.x,
        y_r: m.axes_r.y,
        z_r: m.axes_r.z,
    }
}

/// `move_alloc` from `trapq.c`: a zeroed `Move`.
pub fn move_alloc() -> Move {
    Move::default()
}
