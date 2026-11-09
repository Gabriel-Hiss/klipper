//! Port of the `toolhead` module (`klippy/toolhead.py`).
//!
//! Covers `Move`, `LookAheadQueue`, `ToolHead` (motion planning,
//! print-time tracking, priming/pause states, drip moves, extra axes,
//! velocity limits) and `ToolHeadCommandHelper` (`G4`, `M400`,
//! `SET_VELOCITY_LIMIT`, `M204`), plus `add_printer_objects`.
//!
//! `ToolHead::new` resolves `kinematics` through the loader registry
//! ([`register_kinematics_loader`]) and `add_printer_objects` reaches
//! `kinematics.extruder.add_printer_objects` through
//! [`register_extruder_objects`]; the `kinematics` crate depends on this
//! crate, so it registers both. `trapq_set_position` is the direct
//! `crate::motion_handles::trapq_set_position` call, and the stepper
//! factory served by `ToolHeadApi::stepper_factory` is the ported
//! `crate::stepper::StepperFactory` (the `import stepper` functions).

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetFloatOpts as GcmdGetFloatOpts, GCodeApi, GCodeCommandHandler, RegisterCommandOpts,
};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::McuApi;
use klippy_api::object::{load_interface, lookup_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_SHUTDOWN};
use std::sync::{Arc, LazyLock, Weak};
use kos::sync::Mutex;
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{
    Coord, ExtraAxis, ExtraAxisApi, HomingTargetApi, Kinematics, LoadKinematics,
    LookaheadCallback, MotionQueuing, MotionQueuingApi, MoveApi, PrinterExtruder,
    PrinterExtruderApi, ToolHead as ToolHeadHandle, ToolHeadApi, TrapqAppend,
};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};

/// `LOOKAHEAD_FLUSH_TIME` (`klippy/toolhead.py:116`).
pub const LOOKAHEAD_FLUSH_TIME: f64 = 0.150;
/// `BUFFER_TIME_HIGH` (`klippy/toolhead.py:195`).
pub const BUFFER_TIME_HIGH: f64 = 2.0;
/// `BUFFER_TIME_START` (`klippy/toolhead.py:196`).
pub const BUFFER_TIME_START: f64 = 0.250;
/// `PRIMING_CMD_TIME` (`klippy/toolhead.py:197`).
pub const PRIMING_CMD_TIME: f64 = 0.100;


fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant toolhead format")
}


/// Python `max` with two arguments (first maximal item wins).
fn py_max2(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python `min` with two arguments (first minimal item wins).
fn py_min2(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python float true division: a zero divisor raises `ZeroDivisionError`.
fn py_div(a: f64, b: f64) -> KResult<f64> {
    if b == 0.0 {
        return Err(KlippyError::Internal {
            class_name: "ZeroDivisionError".to_string(),
            message: "float division by zero".to_string(),
        });
    }
    Ok(a / b)
}

// ---------------------------------------------------------------------------
// Move
// ---------------------------------------------------------------------------

struct MoveInner {
    start_pos: Vec<f64>,
    end_pos: Vec<f64>,
    axes_d: Vec<f64>,
    axes_r: Vec<f64>,
    move_d: f64,
    is_kinematic_move: bool,
    accel: f64,
    junction_deviation: f64,
    max_start_v2: f64,
    max_cruise_v2: f64,
    delta_v2: f64,
    next_junction_v2: f64,
    max_mcr_start_v2: f64,
    mcr_delta_v2: f64,
    min_move_t: f64,
    start_v: f64,
    cruise_v: f64,
    end_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    timing_callbacks: Vec<LookaheadCallback>,
}

pub struct Move {
    inner: GCell<MoveInner>,
    #[allow(dead_code)]
    printer: Printer,
}

impl Move {
    /// `Move(toolhead, start_pos, end_pos, speed)`
    /// (`klippy/toolhead.py:15`). `max_velocity`, `max_accel`,
    /// `junction_deviation` and `mcr_pseudo_accel` are the toolhead
    /// values the Python reads off its `toolhead` argument.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        start_pos: Vec<f64>,
        end_pos: Vec<f64>,
        speed: f64,
        max_velocity: f64,
        max_accel: f64,
        junction_deviation: f64,
        mcr_pseudo_accel: f64,
        printer: Printer,
    ) -> KResult<Move> {
        let mut velocity = py_min2(speed, max_velocity);
        let mut accel = max_accel;
        let mut is_kinematic_move = true;
        let axes_d: Vec<f64> = start_pos
            .iter()
            .zip(end_pos.iter())
            .map(|(sp, ep)| ep - sp)
            .collect();
        let mut move_d =
            (axes_d[0] * axes_d[0] + axes_d[1] * axes_d[1] + axes_d[2] * axes_d[2]).sqrt();
        let mut end_pos_out = end_pos;
        let mut axes_d_out = axes_d;
        let inv_move_d: f64;
        if move_d < 0.000000001 {
            // Extrude only move.
            let mut ep = Vec::with_capacity(end_pos_out.len());
            ep.push(start_pos[0]);
            ep.push(start_pos[1]);
            ep.push(start_pos[2]);
            ep.extend_from_slice(&end_pos_out[3..]);
            end_pos_out = ep;
            axes_d_out[0] = 0.0;
            axes_d_out[1] = 0.0;
            axes_d_out[2] = 0.0;
            move_d = axes_d_out[3..]
                .iter()
                .map(|ad| ad.abs())
                .fold(0.0f64, py_max2);
            inv_move_d = if move_d != 0.0 { 1.0 / move_d } else { 0.0 };
            accel = 99999999.9;
            velocity = speed;
            is_kinematic_move = false;
        } else {
            inv_move_d = 1.0 / move_d;
        }
        let axes_r: Vec<f64> = axes_d_out.iter().map(|d| d * inv_move_d).collect();
        let min_move_t = py_div(move_d, velocity)?;
        let delta_v2 = 2.0 * move_d * accel;
        let mcr_delta_v2 = 2.0 * move_d * mcr_pseudo_accel;
        Ok(Move {
            inner: GCell::new(MoveInner {
                start_pos,
                end_pos: end_pos_out,
                axes_d: axes_d_out,
                axes_r,
                move_d,
                is_kinematic_move,
                accel,
                junction_deviation,
                max_start_v2: 0.0,
                max_cruise_v2: kos::libm::pow(velocity, 2.0),
                delta_v2,
                next_junction_v2: 999999999.9,
                max_mcr_start_v2: 0.0,
                mcr_delta_v2,
                min_move_t,
                start_v: 0.0,
                cruise_v: 0.0,
                end_v: 0.0,
                accel_t: 0.0,
                cruise_t: 0.0,
                decel_t: 0.0,
                timing_callbacks: Vec::new(),
            }),
            printer,
        })
    }

    /// `limit_speed` (`klippy/toolhead.py:52`).
    pub fn limit_speed(&self, speed: f64, accel: f64) {
        let mut inner = self.inner.borrow_mut();
        let speed2 = kos::libm::pow(speed, 2.0);
        if speed2 < inner.max_cruise_v2 {
            inner.max_cruise_v2 = speed2;
            inner.min_move_t = inner.move_d / speed;
        }
        inner.accel = py_min2(inner.accel, accel);
        inner.delta_v2 = 2.0 * inner.move_d * inner.accel;
        inner.mcr_delta_v2 = py_min2(inner.mcr_delta_v2, inner.delta_v2);
    }

    /// `limit_next_junction_speed` (`klippy/toolhead.py:60`).
    pub fn limit_next_junction_speed(&self, speed: f64) {
        let mut inner = self.inner.borrow_mut();
        inner.next_junction_v2 = py_min2(inner.next_junction_v2, kos::libm::pow(speed, 2.0));
    }

    /// `move_error` (`klippy/toolhead.py:62`).
    pub fn move_error(&self, msg: &str) -> KlippyError {
        let ep = self.inner.borrow().end_pos.clone();
        KlippyError::Command(pct_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                PctValue::Str(msg.to_string()),
                PctValue::Float(ep[0]),
                PctValue::Float(ep[1]),
                PctValue::Float(ep[2]),
                PctValue::Float(ep[3]),
            ],
        ))
    }

    /// `move_error` with the default message.
    pub fn move_error_default(&self) -> KlippyError {
        self.move_error("Move out of range")
    }

    /// `calc_junction` (`klippy/toolhead.py:66`).
    pub fn calc_junction(&self, prev_move: &Move, extra_axes: &[ExtraAxis]) {
        let (is_kin, prev_is_kin) = {
            let inner = self.inner.borrow();
            let prev = prev_move.inner.borrow();
            (inner.is_kinematic_move, prev.is_kinematic_move)
        };
        if !is_kin || !prev_is_kin {
            return;
        }
        // Allow extra axes to calculate maximum junction.
        let mut ea_v2: Vec<f64> = Vec::new();
        for (e_index, ea) in extra_axes.iter().enumerate() {
            ea_v2.push(ea.calc_junction(prev_move, self, e_index + 3));
        }
        let (max_start_v2, axes_r, prev_axes_r, delta_v2, prev_delta_v2,
             junction_deviation, prev_junction_deviation, accel, prev_accel) = {
            let inner = self.inner.borrow();
            let prev = prev_move.inner.borrow();
            let mut m = inner.max_cruise_v2;
            m = py_min2(m, prev.max_cruise_v2);
            m = py_min2(m, prev.next_junction_v2);
            m = py_min2(m, prev.max_start_v2 + prev.delta_v2);
            for v in &ea_v2 {
                m = py_min2(m, *v);
            }
            (
                m,
                inner.axes_r.clone(),
                prev.axes_r.clone(),
                inner.delta_v2,
                prev.delta_v2,
                inner.junction_deviation,
                prev.junction_deviation,
                inner.accel,
                prev.accel,
            )
        };
        let mut max_start_v2 = max_start_v2;
        // Find max velocity using "approximated centripetal velocity".
        let junction_cos_theta = -(axes_r[0] * prev_axes_r[0]
            + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]);
        let sin_theta_d2 = py_max2(0.5 * (1.0 - junction_cos_theta), 0.0).sqrt();
        let cos_theta_d2 = py_max2(0.5 * (1.0 + junction_cos_theta), 0.0).sqrt();
        let one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;
        if one_minus_sin_theta_d2 > 0.0 && cos_theta_d2 > 0.0 {
            let r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            let move_jd_v2 = r_jd * junction_deviation * accel;
            let pmove_jd_v2 = r_jd * prev_junction_deviation * prev_accel;
            // Approximated circle must contact moves no further than mid-move
            //   centripetal_v2 = .5 * self.move_d * self.accel * tan_theta_d2
            let quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            let move_centripetal_v2 = delta_v2 * quarter_tan_theta_d2;
            let pmove_centripetal_v2 = prev_delta_v2 * quarter_tan_theta_d2;
            max_start_v2 = py_min2(max_start_v2, move_jd_v2);
            max_start_v2 = py_min2(max_start_v2, pmove_jd_v2);
            max_start_v2 = py_min2(max_start_v2, move_centripetal_v2);
            max_start_v2 = py_min2(max_start_v2, pmove_centripetal_v2);
        }
        let mut inner = self.inner.borrow_mut();
        inner.max_start_v2 = max_start_v2;
        let prev = prev_move.inner.borrow();
        inner.max_mcr_start_v2 = py_min2(max_start_v2, prev.max_mcr_start_v2 + prev.mcr_delta_v2);
    }

    /// `set_junction` (`klippy/toolhead.py:100`).
    pub fn set_junction(&self, start_v2: f64, cruise_v2: f64, end_v2: f64) -> KResult<()> {
        let mut inner = self.inner.borrow_mut();
        // Determine accel, cruise, and decel portions of the move distance.
        let half_inv_accel = py_div(0.5, inner.accel)?;
        let accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        let decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        let cruise_d = inner.move_d - accel_d - decel_d;
        // Determine move velocities.
        let start_v = start_v2.sqrt();
        inner.start_v = start_v;
        let cruise_v = cruise_v2.sqrt();
        inner.cruise_v = cruise_v;
        let end_v = end_v2.sqrt();
        inner.end_v = end_v;
        // Determine time spent in each portion of move (time is the
        // distance divided by average velocity).
        inner.accel_t = py_div(accel_d, (start_v + cruise_v) * 0.5)?;
        inner.cruise_t = py_div(cruise_d, cruise_v)?;
        inner.decel_t = py_div(decel_d, (end_v + cruise_v) * 0.5)?;
        Ok(())
    }

    fn take_timing_callbacks(&self) -> Vec<LookaheadCallback> {
        std::mem::take(&mut self.inner.borrow_mut().timing_callbacks)
    }
}

impl MoveApi for Move {
    fn start_pos(&self) -> Coord {
        Coord(self.inner.borrow().start_pos.clone())
    }
    fn end_pos(&self) -> Coord {
        Coord(self.inner.borrow().end_pos.clone())
    }
    fn axes_d(&self) -> Coord {
        Coord(self.inner.borrow().axes_d.clone())
    }
    fn axes_r(&self) -> Coord {
        Coord(self.inner.borrow().axes_r.clone())
    }
    fn move_d(&self) -> f64 {
        self.inner.borrow().move_d
    }
    fn is_kinematic_move(&self) -> bool {
        self.inner.borrow().is_kinematic_move
    }
    fn accel(&self) -> f64 {
        self.inner.borrow().accel
    }
    fn max_cruise_v2(&self) -> f64 {
        self.inner.borrow().max_cruise_v2
    }
    fn start_v(&self) -> f64 {
        self.inner.borrow().start_v
    }
    fn cruise_v(&self) -> f64 {
        self.inner.borrow().cruise_v
    }
    fn accel_t(&self) -> f64 {
        self.inner.borrow().accel_t
    }
    fn cruise_t(&self) -> f64 {
        self.inner.borrow().cruise_t
    }
    fn decel_t(&self) -> f64 {
        self.inner.borrow().decel_t
    }
    fn limit_speed(&self, speed: f64, accel: f64) {
        Move::limit_speed(self, speed, accel);
    }
    fn move_error(&self, msg: String) -> KlippyError {
        self.move_error(msg.as_str())
    }
}

// ---------------------------------------------------------------------------
// LookAheadQueue
// ---------------------------------------------------------------------------

/// Port of the `LookAheadQueue` class (`klippy/toolhead.py:120`).
pub struct LookAheadQueue {
    queue: Vec<Arc<Move>>,
    junction_flush: f64,
}

impl Default for LookAheadQueue {
    fn default() -> LookAheadQueue {
        LookAheadQueue::new()
    }
}

impl LookAheadQueue {
    /// `LookAheadQueue.__init__` (`klippy/toolhead.py:121`).
    pub fn new() -> LookAheadQueue {
        LookAheadQueue {
            queue: Vec::new(),
            junction_flush: LOOKAHEAD_FLUSH_TIME,
        }
    }

    /// `reset` (`klippy/toolhead.py:124`).
    pub fn reset(&mut self) {
        self.queue.clear();
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
    }

    /// `set_flush_time` (`klippy/toolhead.py:127`).
    pub fn set_flush_time(&mut self, flush_time: f64) {
        self.junction_flush = flush_time;
    }

    /// `is_empty` (`klippy/toolhead.py:129`).
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    /// `get_last` (`klippy/toolhead.py:131`).
    pub fn get_last(&self) -> Option<Arc<Move>> {
        self.queue.last().cloned()
    }

    /// `flush` (`klippy/toolhead.py:135`).
    pub fn flush(&mut self, lazy: bool) -> KResult<Vec<Arc<Move>>> {
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
        let mut update_flush_count = lazy;
        let mut flush_count = self.queue.len() as i64;
        // Traverse queue from last to first move and determine maximum
        // junction speed assuming the robot comes to a complete stop
        // after the last move.
        #[allow(clippy::type_complexity)]
        let mut junction_info: Vec<Option<(Arc<Move>, f64, Option<f64>, f64)>> =
            vec![None; self.queue.len()];
        let mut next_start_v2 = 0.0f64;
        let mut next_mcr_start_v2 = 0.0f64;
        let mut peak_cruise_v2 = 0.0f64;
        let mut pending_cv2_assign: i64 = 0;
        let mut i = flush_count - 1;
        while i >= 0 {
            let move_ = self.queue[i as usize].clone();
            let (move_delta_v2, move_max_start_v2, move_max_cruise_v2,
                 move_mcr_delta_v2, move_max_mcr_start_v2) = {
                let inner = move_.inner.borrow();
                (
                    inner.delta_v2,
                    inner.max_start_v2,
                    inner.max_cruise_v2,
                    inner.mcr_delta_v2,
                    inner.max_mcr_start_v2,
                )
            };
            let reachable_start_v2 = next_start_v2 + move_delta_v2;
            let start_v2 = py_min2(move_max_start_v2, reachable_start_v2);
            let mut cruise_v2: Option<f64> = None;
            pending_cv2_assign += 1;
            let reach_mcr_start_v2 = next_mcr_start_v2 + move_mcr_delta_v2;
            let mcr_start_v2 = py_min2(move_max_mcr_start_v2, reach_mcr_start_v2);
            if mcr_start_v2 < reach_mcr_start_v2 {
                // It's possible for this move to accelerate.
                if mcr_start_v2 + move_mcr_delta_v2 > next_mcr_start_v2
                    || pending_cv2_assign > 1
                {
                    // This move can both accel and decel, or this is a
                    // full accel move followed by a full decel move.
                    if update_flush_count && peak_cruise_v2 != 0.0 {
                        flush_count = i + pending_cv2_assign;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = (mcr_start_v2 + reach_mcr_start_v2) * 0.5;
                }
                cruise_v2 = Some(py_min2(
                    py_min2((start_v2 + reachable_start_v2) * 0.5, move_max_cruise_v2),
                    peak_cruise_v2,
                ));
                pending_cv2_assign = 0;
            }
            junction_info[i as usize] = Some((move_, start_v2, cruise_v2, next_start_v2));
            next_start_v2 = start_v2;
            next_mcr_start_v2 = mcr_start_v2;
            i -= 1;
        }
        if update_flush_count || flush_count == 0 {
            return Ok(Vec::new());
        }
        // Traverse queue in forward direction to propagate cruise_v2.
        let mut prev_cruise_v2 = 0.0f64;
        let mut j = 0i64;
        while j < flush_count {
            let (move_, start_v2, cruise_v2, next_start_v2) =
                junction_info[j as usize].clone().unwrap();
            let cruise_v2 = match cruise_v2 {
                None => {
                    // This move can't accelerate - propagate cruise_v2 from previous.
                    py_min2(prev_cruise_v2, start_v2)
                }
                Some(v) => v,
            };
            move_.set_junction(
                py_min2(start_v2, cruise_v2),
                cruise_v2,
                py_min2(next_start_v2, cruise_v2),
            )?;
            prev_cruise_v2 = cruise_v2;
            j += 1;
        }
        // Remove processed moves from the queue (Python slices, so a
        // flush_count past the end just empties the queue).
        let n = (flush_count as usize).min(self.queue.len());
        Ok(self.queue.drain(..n).collect())
    }

    /// `add_move` (`klippy/toolhead.py:186`). Returns `True` when the
    /// caller should flush; the Python returns `None` for the first
    /// move, which the caller treats as falsy.
    pub fn add_move(&mut self, move_: Arc<Move>, extra_axes: &[ExtraAxis]) -> bool {
        self.queue.push(move_);
        if self.queue.len() == 1 {
            return false;
        }
        let last = self.queue.last().unwrap().clone();
        let prev = self.queue[self.queue.len() - 2].clone();
        last.calc_junction(&prev, extra_axes);
        let min_move_t = last.inner.borrow().min_move_t;
        self.junction_flush -= min_move_t;
        // Check if enough moves have been queued to reach the target flush time.
        self.junction_flush <= 0.0
    }
}

// ---------------------------------------------------------------------------
// Extra-axis views and DummyExtruder
// ---------------------------------------------------------------------------

/// Port of the `DummyExtruder` class (`klippy/kinematics/extruder.py:292`),
/// created by `ToolHead.__init__` when no extruder is configured.
pub struct DummyExtruder {
    #[allow(dead_code)]
    printer: Printer,
}

impl DummyExtruder {
    /// `DummyExtruder(printer)` (`klippy/kinematics/extruder.py:293`).
    pub fn new(printer: Printer) -> DummyExtruder {
        DummyExtruder { printer }
    }
}

impl ExtraAxisApi for DummyExtruder {
    fn get_name(&self) -> String {
        "".to_string()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        Some("E".to_string())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn check_move(&self, mv: &dyn MoveApi, _ea_index: usize) -> KResult<()> {
        Err(mv.move_error("Extrude when no extruder present".to_string()))
    }
    fn calc_junction(&self, _prev_move: &dyn MoveApi, mv: &dyn MoveApi, _ea_index: usize) -> f64 {
        mv.max_cruise_v2()
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl PrinterExtruderApi for DummyExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::Command("Extruder not configured".to_string()))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

// ---------------------------------------------------------------------------
// ToolHead
// ---------------------------------------------------------------------------

struct State {
    lookahead: LookAheadQueue,
    commanded_pos: Vec<f64>,
    max_velocity: f64,
    max_accel: f64,
    min_cruise_ratio: f64,
    square_corner_velocity: f64,
    junction_deviation: f64,
    mcr_pseudo_accel: f64,
    check_stall_time: f64,
    print_stall: i64,
    can_pause: bool,
    need_check_pause: f64,
    print_time: f64,
    special_queuing_state: String,
    priming_timer: Option<Timer>,
    kin: Option<Kinematics>,
    extruder: PrinterExtruder,
    extra_axes: Vec<ExtraAxis>,
    extra_axes_status: IndexMap<String, i64>,
}

/// The `ToolHead` object (`klippy/toolhead.py:200`).
pub struct ToolHead {
    printer: Printer,
    reactor: Reactor,
    mcu: klippy_api::mcu::Mcu,
    motion_queuing: MotionQueuing,
    stepper_factory: StepperFactory,
    trapq: Trapq,
    trapq_append: TrapqAppend,
    state: GCell<State>,
    weak: Weak<ToolHead>,
}

impl ToolHead {
    /// `ToolHead.__init__` (`klippy/toolhead.py:201`), including the
    /// `import kinematics.<name>` / `load_kinematics(self, config)` step,
    /// which resolves the name through the registry filled by
    /// [`register_kinematics_loader`] (the `kinematics` crate depends on
    /// this crate, so the core cannot import it). The flush callback is
    /// registered before the kinematics object is loaded and the shutdown
    /// handler after it, like the Python.
    pub fn new(config: &Config) -> KResult<Arc<ToolHead>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let mcu = lookup_interface::<dyn McuApi>(&*printer, "mcu", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let max_velocity = config
            .getfloat(
                "max_velocity",
                GetFloatOpts { above: Some(0.0), ..Default::default() },
            )?
            .expect("required");
        let max_accel = config
            .getfloat(
                "max_accel",
                GetFloatOpts { above: Some(0.0), ..Default::default() },
            )?
            .expect("required");
        let min_cruise_ratio = config
            .getfloat(
                "minimum_cruise_ratio",
                GetFloatOpts {
                    default: Some(Some(0.5)),
                    below: Some(1.0),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("defaulted");
        let square_corner_velocity = config
            .getfloat(
                "square_corner_velocity",
                GetFloatOpts {
                    default: Some(Some(5.0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("defaulted");
        let scv2 = kos::libm::pow(square_corner_velocity, 2.0);
        let junction_deviation = scv2 * (2.0f64.sqrt() - 1.0) / max_accel;
        let mcr_pseudo_accel = max_accel * (1.0 - min_cruise_ratio);
        let can_pause = !mcu.is_fileoutput();
        let motion_queuing =
            load_interface::<dyn MotionQueuingApi>(&*printer, config, "motion_queuing", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let dummy: PrinterExtruder = Arc::new(DummyExtruder::new(Arc::clone(&printer)));
        let dummy_axis: ExtraAxis = Arc::clone(&dummy) as ExtraAxis;
        let extra_axes = vec![dummy_axis];
        let mut extra_axes_status = IndexMap::new();
        {
            let names: Vec<String> = extra_axes.iter().map(|ea| ea.get_name()).collect();
            for (e_index, n) in names.into_iter().enumerate() {
                if !n.is_empty() {
                    extra_axes_status.insert(n, (e_index + 3) as i64);
                }
            }
        }
        let toolhead = Arc::new_cyclic(|weak: &Weak<ToolHead>| {
            // Setup for generating moves.
            let flush_weak = weak.clone();
            let cb: klippy_api::toolhead::FlushCallback =
                Arc::new(move |flush_time, step_gen_time| {
                    if let Some(th) = flush_weak.upgrade() {
                        th.handle_step_flush(flush_time, step_gen_time);
                    }
                });
            motion_queuing.register_flush_callback(cb, true);
            ToolHead {
                printer: Arc::clone(&printer),
                reactor,
                mcu,
                motion_queuing: Arc::clone(&motion_queuing),
                stepper_factory: Arc::new(crate::stepper::StepperFactory),
                trapq: motion_queuing.allocate_trapq(),
                trapq_append: motion_queuing.lookup_trapq_append(),
                state: GCell::new(State {
                    lookahead: {
                        let mut la = LookAheadQueue::new();
                        la.set_flush_time(BUFFER_TIME_HIGH);
                        la
                    },
                    commanded_pos: vec![0.0, 0.0, 0.0, 0.0],
                    max_velocity,
                    max_accel,
                    min_cruise_ratio,
                    square_corner_velocity,
                    junction_deviation,
                    mcr_pseudo_accel,
                    check_stall_time: 0.0,
                    print_stall: 0,
                    can_pause,
                    need_check_pause: -1.0,
                    print_time: 0.0,
                    special_queuing_state: "NeedPrime".to_string(),
                    priming_timer: None,
                    kin: None,
                    extruder: dummy,
                    extra_axes,
                    extra_axes_status,
                }),
                weak: weak.clone(),
            }
        });
        // Create kinematics class. `gcode.Coord` is looked up here.
        lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?;
        let kin_name = config
            .get("kinematics", GetStrOpts { default: None, ..Default::default() })?
            .expect("required");
        let handle: ToolHeadHandle = Arc::clone(&toolhead) as ToolHeadHandle;
        let loaded = match lookup_kinematics_loader(&kin_name) {
            Some(loader) => loader(&handle, config),
            None => Err(KlippyError::Other {
                class_name: "ModuleNotFoundError".to_string(),
                message: pct_format(
                    "No module named 'kinematics.%s'",
                    &[PctValue::Str(kin_name.clone())],
                ),
            }),
        };
        let kin = match loaded {
            Ok(kin) => kin,
            Err(e) => {
                // `except config.error: raise`.
                if matches!(&e, KlippyError::Config(_)) {
                    return Err(e);
                }
                // `except self.printer.lookup_object('pins').error: raise`.
                printer.lookup_object("pins", LookupDefault::Raise)?;
                if matches!(&e, KlippyError::Other { class_name, .. } if class_name == "error") {
                    return Err(e);
                }
                let msg = pct_format(
                    "Error loading kinematics '%s'",
                    &[PctValue::Str(kin_name)],
                );
                log::exception(&msg, e.message());
                return Err(KlippyError::config(msg));
            }
        };
        toolhead.state.borrow_mut().kin = Some(kin);
        // Register handlers.
        {
            let weak = Arc::downgrade(&toolhead);
            printer.register_event_handler(
                EV_KLIPPY_SHUTDOWN,
                Box::new(move |_params| {
                    if let Some(th) = weak.upgrade() {
                        th.handle_shutdown();
                    }
                    Ok(PyValue::None)
                }),
            );
        }
        Ok(toolhead)
    }

    fn kin(&self) -> Kinematics {
        self.state.borrow().kin.clone().expect("kinematics not loaded")
    }

    fn limits(&self) -> (f64, f64, f64, f64) {
        let st = self.state.borrow();
        (st.max_velocity, st.max_accel, st.junction_deviation, st.mcr_pseudo_accel)
    }

    /// `_calc_junction_deviation` (`klippy/toolhead.py:534`).
    fn calc_junction_deviation(&self) {
        let mut st = self.state.borrow_mut();
        let scv2 = kos::libm::pow(st.square_corner_velocity, 2.0);
        st.junction_deviation = scv2 * (2.0f64.sqrt() - 1.0) / st.max_accel;
        st.mcr_pseudo_accel = st.max_accel * (1.0 - st.min_cruise_ratio);
    }

    /// `_advance_move_time` (`klippy/toolhead.py:258`).
    fn advance_move_time(&self, next_print_time: f64) {
        let mut st = self.state.borrow_mut();
        st.print_time = py_max2(st.print_time, next_print_time);
    }

    /// `_calc_print_time` (`klippy/toolhead.py:260`).
    fn calc_print_time(&self) -> KResult<()> {
        let curtime = self.reactor.monotonic();
        let est_print_time = self.mcu.estimated_print_time(curtime);
        let kin_time = self.motion_queuing.calc_step_gen_restart(est_print_time);
        let min_print_time = py_max2(est_print_time + BUFFER_TIME_START, kin_time);
        let raised = {
            let mut st = self.state.borrow_mut();
            if min_print_time > st.print_time {
                st.print_time = min_print_time;
                true
            } else {
                false
            }
        };
        if raised {
            let print_time = self.state.borrow().print_time;
            self.printer.send_event(
                "toolhead:sync_print_time",
                &[
                    klippy_api::object::EventArg::Value(PyValue::Float(curtime)),
                    klippy_api::object::EventArg::Value(PyValue::Float(est_print_time)),
                    klippy_api::object::EventArg::Value(PyValue::Float(print_time)),
                ],
            )?;
        }
        Ok(())
    }

    /// `_process_lookahead` (`klippy/toolhead.py:269`).
    fn process_lookahead(&self, lazy: bool) -> KResult<()> {
        let moves = {
            let mut st = self.state.borrow_mut();
            st.lookahead.flush(lazy)?
        };
        if moves.is_empty() {
            return Ok(());
        }
        // Resync print_time if necessary.
        let need_calc = !self.state.borrow().special_queuing_state.is_empty();
        if need_calc {
            // Transition from "NeedPrime"/"Priming" state to main state.
            {
                let mut st = self.state.borrow_mut();
                st.special_queuing_state = String::new();
                st.need_check_pause = -1.0;
            }
            self.calc_print_time()?;
        }
        // Queue moves into trapezoid motion queue (trapq).
        let mut next_move_time = self.state.borrow().print_time;
        let extra_axes_snapshot: Vec<ExtraAxis> =
            self.state.borrow().extra_axes.clone();
        {
            let _guard = self.reactor.assert_no_pause();
            for move_ in &moves {
                if move_.is_kinematic_move() {
                    let start_pos = move_.start_pos();
                    let axes_r = move_.axes_r();
                    (self.trapq_append)(
                        &self.trapq,
                        next_move_time,
                        move_.accel_t(),
                        move_.cruise_t(),
                        move_.decel_t(),
                        start_pos.0[0],
                        start_pos.0[1],
                        start_pos.0[2],
                        axes_r.0[0],
                        axes_r.0[1],
                        axes_r.0[2],
                        move_.start_v(),
                        move_.cruise_v(),
                        move_.accel(),
                    );
                }
                for (e_index, ea) in extra_axes_snapshot.iter().enumerate() {
                    let axes_d = move_.axes_d();
                    if axes_d.0[e_index + 3] != 0.0 {
                        ea.process_move(next_move_time, &**move_, e_index + 3);
                    }
                }
                next_move_time =
                    next_move_time + move_.accel_t() + move_.cruise_t() + move_.decel_t();
                let mut cbs = move_.take_timing_callbacks();
                for mut cb in cbs.drain(..) {
                    cb(next_move_time)?;
                }
            }
        }
        // Generate steps for moves.
        self.advance_move_time(next_move_time);
        self.motion_queuing
            .note_mcu_movequeue_activity(next_move_time, true);
        Ok(())
    }

    /// `_flush_lookahead` (`klippy/toolhead.py:300`).
    fn flush_lookahead(&self, is_runout: bool) -> KResult<()> {
        // Transit from "NeedPrime"/"Priming"/main state to "NeedPrime".
        let prev_print_time = self.state.borrow().print_time;
        self.process_lookahead(false)?;
        let mut st = self.state.borrow_mut();
        st.special_queuing_state = "NeedPrime".to_string();
        st.need_check_pause = -1.0;
        st.lookahead.set_flush_time(BUFFER_TIME_HIGH);
        st.check_stall_time = 0.0;
        if is_runout && prev_print_time != st.print_time {
            st.check_stall_time = st.print_time;
        }
        Ok(())
    }

    /// `_handle_step_flush` (`klippy/toolhead.py:310`).
    fn handle_step_flush(&self, _flush_time: f64, step_gen_time: f64) {
        let special = self.state.borrow().special_queuing_state.clone();
        if !special.is_empty() {
            return;
        }
        // In "main" state - flush lookahead if buffer runs low.
        let kin_flush_delay = self.motion_queuing.get_kin_flush_delay();
        let print_time = self.state.borrow().print_time;
        if step_gen_time >= print_time - kin_flush_delay - 0.001 {
            if let Err(e) = self.flush_lookahead(true) {
                // `motion_queuing._flush_handler` catches whatever the flush
                // callbacks raise.
                log::exception(
                    "Exception in flush_handler",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                self.printer
                    .invoke_shutdown("Exception in flush_handler", PyValue::Dict(IndexMap::new()));
            }
        }
    }

    /// `_priming_handler` (`klippy/toolhead.py:327`).
    fn priming_handler(&self, _eventtime: f64) -> f64 {
        let timer = self.state.borrow_mut().priming_timer.take();
        if let Some(t) = timer {
            self.reactor.unregister_timer(&t);
        }
        let r: KResult<()> = (|| {
            let special = self.state.borrow().special_queuing_state.clone();
            if special == "Priming" {
                self.flush_lookahead(true)?;
            }
            Ok(())
        })();
        if let Err(e) = r {
            log::exception(
                "Exception in priming_handler",
                &format!("{}: {}", e.class_name(), e.message()),
            );
            self.printer.invoke_shutdown(
                "Exception in priming_handler",
                PyValue::Dict(IndexMap::new()),
            );
        }
        NEVER
    }

    /// `_check_priming_state` (`klippy/toolhead.py:337`).
    fn check_priming_state(&self, eventtime: f64) {
        let empty = self.state.borrow().lookahead.is_empty();
        if empty {
            // In "NeedPrime" state and can remain there.
            return;
        }
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        {
            let mut st = self.state.borrow_mut();
            if st.check_stall_time != 0.0 {
                // Was in "NeedPrime" state and got there from idle input.
                if est_print_time < st.check_stall_time {
                    st.print_stall += 1;
                }
                st.check_stall_time = 0.0;
            }
        }
        // Transition from "NeedPrime"/"Priming" state to "Priming" state.
        let print_time: f64;
        {
            let mut st = self.state.borrow_mut();
            st.special_queuing_state = "Priming".to_string();
            st.need_check_pause = -1.0;
            print_time = st.print_time;
        }
        if self.state.borrow().priming_timer.is_none() {
            let weak = self.weak.clone();
            let reactor = self.reactor.clone();
            let timer = reactor.register_timer(
                move |eventtime| {
                    if let Some(th) = weak.upgrade() {
                        th.priming_handler(eventtime)
                    } else {
                        NEVER
                    }
                },
                NEVER,
            );
            self.state.borrow_mut().priming_timer = Some(timer);
        }
        let will_pause_time = print_time - est_print_time - BUFFER_TIME_HIGH;
        let wtime = eventtime + py_max2(0.0, will_pause_time) + PRIMING_CMD_TIME;
        let st = self.state.borrow();
        if let Some(t) = &st.priming_timer {
            self.reactor.update_timer(t, wtime);
        }
    }

    /// `_check_pause` (`klippy/toolhead.py:356`).
    fn check_pause(&self) {
        let mut eventtime = self.reactor.monotonic();
        let special = self.state.borrow().special_queuing_state.clone();
        if !special.is_empty() {
            // In "NeedPrime"/"Priming" state - update priming expiration timer.
            self.check_priming_state(eventtime);
        }
        // Check if there are lots of queued moves and pause if so.
        let mut did_pause = false;
        loop {
            let est_print_time = self.mcu.estimated_print_time(eventtime);
            let pause_time =
                self.state.borrow().print_time - est_print_time - BUFFER_TIME_HIGH;
            if pause_time <= 0.0 {
                break;
            }
            if !self.state.borrow().can_pause {
                self.state.borrow_mut().need_check_pause = NEVER;
                return;
            }
            let pause_time = py_max2(0.005, py_min2(1.0, pause_time));
            eventtime = self.reactor.pause(eventtime + pause_time);
            did_pause = true;
        }
        let special = self.state.borrow().special_queuing_state.clone();
        if special.is_empty() {
            // In main state - defer pause checking.
            let print_time = self.state.borrow().print_time;
            self.state.borrow_mut().need_check_pause = print_time;
            if !did_pause {
                // May be falling behind - yield to avoid starving other tasks.
                self.reactor.pause(NOW);
            }
        }
    }

    /// `_build_extra_axes_status` (`klippy/toolhead.py:430`).
    fn build_extra_axes_status(&self) {
        let mut st = self.state.borrow_mut();
        let names: Vec<String> = st.extra_axes.iter().map(|ea| ea.get_name()).collect();
        let mut status = IndexMap::new();
        for (e_index, n) in names.into_iter().enumerate() {
            if !n.is_empty() {
                status.insert(n, (e_index + 3) as i64);
            }
        }
        st.extra_axes_status = status;
    }

    /// `_handle_shutdown` (`klippy/toolhead.py:519`).
    fn handle_shutdown(&self) {
        let mut st = self.state.borrow_mut();
        st.can_pause = false;
        st.lookahead.reset();
    }

    /// `_drip_load_trapq` (`klippy/toolhead.py:459`).
    fn drip_load_trapq(&self, submit_move: Arc<Move>) -> KResult<(f64, f64)> {
        // Queue move into trapezoid motion queue (trapq).
        if submit_move.move_d() != 0.0 {
            let end_pos = submit_move.end_pos().0;
            self.state.borrow_mut().commanded_pos = end_pos;
            let extra_axes_snapshot: Vec<ExtraAxis> =
                self.state.borrow().extra_axes.clone();
            self.state
                .borrow_mut()
                .lookahead
                .add_move(Arc::clone(&submit_move), &extra_axes_snapshot);
        }
        let moves = self.state.borrow_mut().lookahead.flush(false)?;
        self.calc_print_time()?;
        let start_time = self.state.borrow().print_time;
        let mut end_time = start_time;
        for move_ in &moves {
            let start_pos = move_.start_pos();
            let axes_r = move_.axes_r();
            (self.trapq_append)(
                &self.trapq,
                end_time,
                move_.accel_t(),
                move_.cruise_t(),
                move_.decel_t(),
                start_pos.0[0],
                start_pos.0[1],
                start_pos.0[2],
                axes_r.0[0],
                axes_r.0[1],
                axes_r.0[2],
                move_.start_v(),
                move_.cruise_v(),
                move_.accel(),
            );
            end_time = end_time + move_.accel_t() + move_.cruise_t() + move_.decel_t();
        }
        self.state.borrow_mut().lookahead.reset();
        Ok((start_time, end_time))
    }

    /// `get_last_move_time` (`klippy/toolhead.py:320`).
    fn get_last_move_time_impl(&self) -> KResult<f64> {
        let special = self.state.borrow().special_queuing_state.clone();
        if !special.is_empty() {
            self.flush_lookahead(false)?;
            self.calc_print_time()?;
        } else {
            self.process_lookahead(false)?;
        }
        Ok(self.state.borrow().print_time)
    }

    /// `dwell` (`klippy/toolhead.py:417`).
    fn dwell_impl(&self, delay: f64) -> KResult<()> {
        self.flush_lookahead(false)?;
        let next_print_time = self.get_last_move_time_impl()? + py_max2(0.0, delay);
        self.advance_move_time(next_print_time);
        self.check_pause();
        Ok(())
    }

    /// `wait_moves` (`klippy/toolhead.py:422`).
    fn wait_moves_impl(&self) -> KResult<()> {
        self.flush_lookahead(false)?;
        let mut eventtime = self.reactor.monotonic();
        loop {
            let special_empty = self.state.borrow().special_queuing_state.is_empty();
            let print_time = self.state.borrow().print_time;
            if !(special_empty || print_time >= self.mcu.estimated_print_time(eventtime)) {
                break;
            }
            if !self.state.borrow().can_pause {
                break;
            }
            eventtime = self.reactor.pause(eventtime + 0.100);
        }
        Ok(())
    }
}

impl PrinterObject for ToolHead {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ToolHeadApi, dyn HomingTargetApi);

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(ToolHeadApi::get_status(self, eventtime))
    }

    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(ToolHeadApi::stats(self, eventtime))
    }
}

impl HomingTargetApi for ToolHead {
    fn get_position(&self) -> Vec<f64> {
        self.state.borrow().commanded_pos.clone()
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.flush_step_generation()?;
        let print_time = self.state.borrow().print_time;
        crate::motion_handles::trapq_set_position(&self.trapq, print_time, newpos[0], newpos[1], newpos[2])?;
        {
            let mut st = self.state.borrow_mut();
            // self.commanded_pos[:3] = newpos[:3]
            let keep_from = 3.min(st.commanded_pos.len());
            let rhs_len = 3.min(newpos.len());
            let mut next = Vec::with_capacity(rhs_len + st.commanded_pos.len() - keep_from);
            next.extend_from_slice(&newpos[..rhs_len]);
            next.extend_from_slice(&st.commanded_pos[keep_from..]);
            st.commanded_pos = next;
        }
        let kin = self.kin();
        kin.set_position(newpos, homing_axes)?;
        self.printer
            .send_event("toolhead:set_position", &[] as &[klippy_api::object::EventArg])?;
        Ok(())
    }

    fn get_kinematics(&self) -> Kinematics {
        self.kin()
    }

    fn flush_step_generation(&self) -> KResult<()> {
        self.flush_lookahead(false)?;
        self.motion_queuing.flush_all_steps()?;
        Ok(())
    }

    fn get_last_move_time(&self) -> f64 {
        // The contract has no error channel: a failing flush leaves the
        // print time where it is.
        match self.get_last_move_time_impl() {
            Ok(t) => t,
            Err(_) => self.state.borrow().print_time,
        }
    }

    fn dwell(&self, delay: f64) {
        let _ = self.dwell_impl(delay);
    }

    fn drip_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        // Create and verify move is valid.
        let mut newpos_full = Vec::new();
        newpos_full.extend_from_slice(&newpos[..newpos.len().min(3)]);
        newpos_full.extend_from_slice(&self.state.borrow().commanded_pos[3..]);
        let (max_velocity, max_accel, junction_deviation, mcr_pseudo_accel) = self.limits();
        let move_ = Arc::new(Move::new(
            self.state.borrow().commanded_pos.clone(),
            newpos_full,
            speed,
            max_velocity,
            max_accel,
            junction_deviation,
            mcr_pseudo_accel,
            Arc::clone(&self.printer),
        )?);
        if move_.move_d() != 0.0 {
            let kin = self.kin();
            kin.check_move(&*move_)?;
        }
        // Make sure stepper movement doesn't start before nominal start time.
        let kin_flush_delay = self.motion_queuing.get_kin_flush_delay();
        self.dwell_impl(kin_flush_delay)?;
        // Transmit move in "drip" mode.
        self.process_lookahead(false)?;
        let (start_time, end_time) = self.drip_load_trapq(Arc::clone(&move_))?;
        self.motion_queuing
            .drip_update_time(start_time, end_time, drip_completion)?;
        // Move finished; cleanup any remnants on trapq.
        self.motion_queuing.wipe_trapq(&self.trapq);
        Ok(())
    }
}

impl ToolHeadApi for ToolHead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let (max_velocity, max_accel, junction_deviation, mcr_pseudo_accel) = self.limits();
        let curpos = self.state.borrow().commanded_pos.clone();
        let move_ = Arc::new(Move::new(
            curpos,
            newpos,
            speed,
            max_velocity,
            max_accel,
            junction_deviation,
            mcr_pseudo_accel,
            Arc::clone(&self.printer),
        )?);
        if move_.move_d() == 0.0 {
            return Ok(());
        }
        if move_.is_kinematic_move() {
            let kin = self.kin();
            kin.check_move(&*move_)?;
        }
        let extra_axes = self.state.borrow().extra_axes.clone();
        for (e_index, ea) in extra_axes.iter().enumerate() {
            let axes_d = move_.axes_d();
            if axes_d.0[e_index + 3] != 0.0 {
                ea.check_move(&*move_, e_index + 3)?;
            }
        }
        self.state.borrow_mut().commanded_pos = move_.end_pos().0;
        let want_flush = {
            let extra_axes_snapshot: Vec<ExtraAxis> =
                self.state.borrow().extra_axes.clone();
            self.state
                .borrow_mut()
                .lookahead
                .add_move(Arc::clone(&move_), &extra_axes_snapshot)
        };
        if want_flush {
            self.process_lookahead(true)?;
        }
        if self.state.borrow().print_time > self.state.borrow().need_check_pause {
            self.check_pause();
        }
        Ok(())
    }

    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        let mut curpos = self.state.borrow().commanded_pos.clone();
        for (i, c) in coord.iter().enumerate() {
            if let Some(v) = c {
                if i >= curpos.len() {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list assignment index out of range".to_string(),
                    });
                }
                curpos[i] = *v;
            }
        }
        self.r#move(curpos, speed)?;
        self.printer
            .send_event("toolhead:manual_move", &[] as &[klippy_api::object::EventArg])?;
        Ok(())
    }

    fn wait_moves(&self) {
        let _ = self.wait_moves_impl();
    }

    fn get_status(&self, eventtime: f64) -> PyValue {
        let print_time = self.state.borrow().print_time;
        let estimated_print_time = self.mcu.estimated_print_time(eventtime);
        let extra_name = self
            .state
            .borrow()
            .extra_axes
            .first()
            .map(|ea| ea.get_name())
            .unwrap_or_default();
        let kin_res = self.kin().get_status(eventtime);
        let mut map: IndexMap<PyValue, PyValue> = match kin_res {
            PyValue::Dict(d) => d,
            other => return other,
        };
        let st = self.state.borrow();
        let position: Vec<PyValue> =
            st.commanded_pos.iter().map(|v| PyValue::Float(*v)).collect();
        let extra: Vec<(PyValue, PyValue)> = st
            .extra_axes_status
            .iter()
            .map(|(k, v)| (PyValue::Str(k.clone()), PyValue::Int(PyInt::from_i64(*v))))
            .collect();
        map.insert(PyValue::Str("print_time".to_string()), PyValue::Float(print_time));
        map.insert(
            PyValue::Str("stalls".to_string()),
            PyValue::Int(PyInt::from_i64(st.print_stall)),
        );
        map.insert(
            PyValue::Str("estimated_print_time".to_string()),
            PyValue::Float(estimated_print_time),
        );
        map.insert(PyValue::Str("extruder".to_string()), PyValue::Str(extra_name));
        map.insert(PyValue::Str("position".to_string()), PyValue::Tuple(position));
        map.insert(PyValue::Str("max_velocity".to_string()), PyValue::Float(st.max_velocity));
        map.insert(PyValue::Str("max_accel".to_string()), PyValue::Float(st.max_accel));
        map.insert(
            PyValue::Str("minimum_cruise_ratio".to_string()),
            PyValue::Float(st.min_cruise_ratio),
        );
        map.insert(
            PyValue::Str("square_corner_velocity".to_string()),
            PyValue::Float(st.square_corner_velocity),
        );
        map.insert(
            PyValue::Str("extra_axes".to_string()),
            PyValue::Dict(extra.into_iter().collect()),
        );
        PyValue::Dict(map)
    }

    fn get_trapq(&self) -> Trapq {
        self.trapq.clone()
    }

    fn get_max_velocity(&self) -> (f64, f64) {
        let st = self.state.borrow();
        (st.max_velocity, st.max_accel)
    }

    fn set_max_velocities(
        &self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: Option<f64>,
        min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        {
            let mut st = self.state.borrow_mut();
            if let Some(v) = max_velocity {
                st.max_velocity = v;
            }
            if let Some(v) = max_accel {
                st.max_accel = v;
            }
            if let Some(v) = square_corner_velocity {
                st.square_corner_velocity = v;
            }
            if let Some(v) = min_cruise_ratio {
                st.min_cruise_ratio = v;
            }
        }
        self.calc_junction_deviation();
        let st = self.state.borrow();
        (st.max_velocity, st.max_accel, st.square_corner_velocity, st.min_cruise_ratio)
    }

    fn register_lookahead_callback(&self, mut callback: LookaheadCallback) -> KResult<()> {
        let last = self.state.borrow().lookahead.get_last();
        match last {
            None => callback(self.get_last_move_time_impl()?),
            Some(last_move) => {
                last_move.inner.borrow_mut().timing_callbacks.push(callback);
                Ok(())
            }
        }
    }

    fn check_busy(&self, eventtime: f64) -> (f64, f64, bool) {
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        let st = self.state.borrow();
        (st.print_time, est_print_time, st.lookahead.is_empty())
    }

    fn limit_next_junction_speed(&self, speed: f64) {
        let last = self.state.borrow().lookahead.get_last();
        if let Some(m) = last {
            m.limit_next_junction_speed(speed);
        }
    }

    fn set_extruder(&self, extruder: PrinterExtruder, extrude_pos: f64) {
        {
            let mut st = self.state.borrow_mut();
            st.extra_axes[0] = Arc::clone(&extruder) as ExtraAxis;
            st.extruder = extruder;
            st.commanded_pos[3] = extrude_pos;
        }
        self.build_extra_axes_status();
    }

    fn get_extruder(&self) -> PrinterExtruder {
        Arc::clone(&self.state.borrow().extruder)
    }

    fn add_extra_axis(&self, ea: ExtraAxis, axis_pos: f64) {
        let _ = self.flush_lookahead(false);
        {
            let mut st = self.state.borrow_mut();
            st.extra_axes.push(ea);
            st.commanded_pos.push(axis_pos);
        }
        self.build_extra_axes_status();
        let _ = self.printer.send_event(
            "toolhead:update_extra_axes",
            &[] as &[klippy_api::object::EventArg],
        );
    }

    fn remove_extra_axis(&self, ea: &ExtraAxis) {
        let _ = self.flush_lookahead(false);
        let found = {
            let st = self.state.borrow();
            st.extra_axes.iter().position(|x| Arc::ptr_eq(x, ea))
        };
        if let Some(idx) = found {
            let ea_index = idx + 3;
            let mut st = self.state.borrow_mut();
            st.commanded_pos.remove(ea_index);
            st.extra_axes.remove(idx);
            drop(st);
            self.build_extra_axes_status();
            let _ = self.printer.send_event(
                "toolhead:update_extra_axes",
                &[] as &[klippy_api::object::EventArg],
            );
        }
    }

    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        let mut v: Vec<Option<ExtraAxis>> = vec![None, None, None];
        v.extend(self.state.borrow().extra_axes.clone().into_iter().map(Some));
        v
    }

    fn stats(&self, eventtime: f64) -> (bool, String) {
        let est_print_time = self.mcu.estimated_print_time(eventtime);
        let (print_time, print_stall, special) = {
            let st = self.state.borrow();
            (st.print_time, st.print_stall, st.special_queuing_state.clone())
        };
        let buffer_time = print_time - est_print_time;
        let is_active = buffer_time > -60.0 || special.is_empty();
        let msg = pct_format(
            "print_time=%.3f buffer_time=%.3f print_stall=%d",
            &[
                PctValue::Float(print_time),
                PctValue::Float(py_max2(buffer_time, 0.0)),
                PctValue::Int(PyInt::from_i64(print_stall)),
            ],
        );
        (is_active, msg)
    }

    fn stepper_factory(&self) -> StepperFactory {
        Arc::clone(&self.stepper_factory)
    }
}

// ---------------------------------------------------------------------------
// ToolHeadCommandHelper
// ---------------------------------------------------------------------------

/// Port of the `ToolHeadCommandHelper` class (`klippy/toolhead.py:553`).
pub struct ToolHeadCommandHelper {
    #[allow(dead_code)]
    printer: Printer,
    #[allow(dead_code)]
    toolhead: ToolHeadHandle,
}

impl ToolHeadCommandHelper {
    /// `ToolHeadCommandHelper.__init__` (`klippy/toolhead.py:554`).
    pub fn new(config: &Config) -> KResult<ToolHeadCommandHelper> {
        let printer = config.get_printer();
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        // Register commands.
        {
            let th = Arc::clone(&toolhead);
            let handler: GCodeCommandHandler = Box::new(move |gcmd| {
                let delay = gcmd.get_float(
                    "P",
                    GcmdGetFloatOpts {
                        default: Some(Some(0.0)),
                        minval: Some(0.0),
                        ..Default::default()
                    },
                )?
                .expect("defaulted")
                    / 1000.0;
                th.dwell(delay);
                Ok(())
            });
            gcode.register_command("G4", Some(handler), RegisterCommandOpts::default())?;
        }
        {
            let th = Arc::clone(&toolhead);
            let handler: GCodeCommandHandler = Box::new(move |_gcmd| {
                th.wait_moves();
                Ok(())
            });
            gcode.register_command("M400", Some(handler), RegisterCommandOpts::default())?;
        }
        {
            let th = Arc::clone(&toolhead);
            let printer2 = Arc::clone(&printer);
            let handler: GCodeCommandHandler = Box::new(move |gcmd| {
                Self::cmd_set_velocity_limit(&th, &printer2, gcmd)
            });
            gcode.register_command(
                "SET_VELOCITY_LIMIT",
                Some(handler),
                RegisterCommandOpts {
                    desc: Some("Set printer velocity limits".to_string()),
                    ..Default::default()
                },
            )?;
        }
        {
            let th = Arc::clone(&toolhead);
            let handler: GCodeCommandHandler = Box::new(move |gcmd| Self::cmd_m204(&th, gcmd));
            gcode.register_command("M204", Some(handler), RegisterCommandOpts::default())?;
        }
        Ok(ToolHeadCommandHelper { printer, toolhead })
    }

    fn cmd_set_velocity_limit(
        toolhead: &ToolHeadHandle,
        printer: &Printer,
        gcmd: &klippy_api::gcode::GCmd,
    ) -> KResult<()> {
        let above0 = GcmdGetFloatOpts {
            default: Some(None),
            above: Some(0.0),
            ..Default::default()
        };
        let max_velocity = gcmd.get_float("VELOCITY", above0)?;
        let max_accel = gcmd.get_float("ACCEL", above0)?;
        let square_corner_velocity = gcmd.get_float(
            "SQUARE_CORNER_VELOCITY",
            GcmdGetFloatOpts {
                default: Some(None),
                minval: Some(0.0),
                ..Default::default()
            },
        )?;
        let min_cruise_ratio = gcmd.get_float(
            "MINIMUM_CRUISE_RATIO",
            GcmdGetFloatOpts {
                default: Some(None),
                minval: Some(0.0),
                below: Some(1.0),
                ..Default::default()
            },
        )?;
        let (mv, ma, scv, mcr) = toolhead.set_max_velocities(
            max_velocity,
            max_accel,
            square_corner_velocity,
            min_cruise_ratio,
        );
        let msg = pct_format(
            "max_velocity: %.6f\nmax_accel: %.6f\nminimum_cruise_ratio: %.6f\nsquare_corner_velocity: %.6f",
            &[
                PctValue::Float(mv),
                PctValue::Float(ma),
                PctValue::Float(mcr),
                PctValue::Float(scv),
            ],
        );
        printer.set_rollover_info("toolhead", Some(&format!("toolhead: {}", msg)), true);
        if max_velocity.is_none()
            && max_accel.is_none()
            && square_corner_velocity.is_none()
            && min_cruise_ratio.is_none()
        {
            gcmd.respond_info(&msg, false);
        }
        Ok(())
    }

    fn cmd_m204(toolhead: &ToolHeadHandle, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        // Use S for accel.
        let accel = gcmd.get_float(
            "S",
            GcmdGetFloatOpts { default: Some(None), above: Some(0.0), ..Default::default() },
        )?;
        if let Some(accel) = accel {
            toolhead.set_max_velocities(None, Some(accel), None, None);
            return Ok(());
        }
        // Use minimum of P and T for accel.
        let above0 = GcmdGetFloatOpts {
            default: Some(None),
            above: Some(0.0),
            ..Default::default()
        };
        let p = gcmd.get_float("P", above0)?;
        let t = gcmd.get_float("T", above0)?;
        if p.is_none() || t.is_none() {
            gcmd.respond_info(
                &pct_format(
                    "Invalid M204 command \"%s\"",
                    &[PctValue::Str(gcmd.get_commandline())],
                ),
                true,
            );
            return Ok(());
        }
        toolhead.set_max_velocities(None, Some(py_min2(p.unwrap(), t.unwrap())), None, None);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Kinematics loader registry and module entry points
// ---------------------------------------------------------------------------

fn kinematics_loader_registry() -> &'static Mutex<IndexMap<String, LoadKinematics>> {
    static REGISTRY: LazyLock<Mutex<IndexMap<String, LoadKinematics>>> = LazyLock::new(|| Mutex::new(IndexMap::new()));
    &REGISTRY
}

/// Register a `load_kinematics` for a `kinematics` name. The
/// `kinematics` crate registers its modules here; `ToolHead`'s
/// production entry point resolves the configured name through the
/// registry exactly like `importlib.import_module` in Python.
pub fn register_kinematics_loader(name: &str, loader: LoadKinematics) {
    kinematics_loader_registry()
        .lock()
        .insert(name.to_string(), loader);
}

fn lookup_kinematics_loader(name: &str) -> Option<LoadKinematics> {
    kinematics_loader_registry().lock().get(name).copied()
}

/// `kinematics.extruder.add_printer_objects(config)`.
pub type ExtruderObjectsHook = fn(&Config) -> KResult<()>;

fn extruder_objects_hook() -> &'static Mutex<Option<ExtruderObjectsHook>> {
    static HOOK: LazyLock<Mutex<Option<ExtruderObjectsHook>>> =
        LazyLock::new(|| Mutex::new(None));
    &HOOK
}

/// Register `kinematics.extruder.add_printer_objects` (the `kinematics`
/// crate depends on this crate, so `add_printer_objects` below reaches it
/// through this hook, like the loaders above).
pub fn register_extruder_objects(hook: ExtruderObjectsHook) {
    *extruder_objects_hook().lock() = Some(hook);
}

/// Build the printer objects (`klippy/toolhead.py:604`).
pub fn add_printer_objects(config: &Config) -> KResult<()> {
    let printer = config.get_printer();
    let toolhead = ToolHead::new(config)?;
    let toolhead_obj: ObjectRef = toolhead;
    printer.add_object("toolhead", toolhead_obj)?;
    ToolHeadCommandHelper::new(config)?;
    // Load default extruder objects.
    let hook = *extruder_objects_hook().lock();
    match hook {
        Some(hook) => hook(config)?,
        None => {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "module 'kinematics.extruder' has no attribute 'add_printer_objects'"
                    .to_string(),
            })
        }
    }
    // Load some default modules.
    for module_name in [
        "gcode_move",
        "homing",
        "idle_timeout",
        "statistics",
        "manual_probe",
        "tuning_tower",
        "garbage_collection",
    ] {
        printer.load_object(config, module_name, LookupDefault::Raise)?;
    }
    Ok(())
}

/// Module entry point for the `toolhead` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        add_printer_objects: Some(add_printer_objects),
        ..ModuleFactory::default()
    }
}
