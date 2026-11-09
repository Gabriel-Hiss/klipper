//! Port of the `homing` module (`klippy/extras/homing.py`).
//!
//! Helper code for homing and probing moves: `HomingMove` drives one move to
//! endstops, `Homing` is the state handed to `kin.home()`, `PrinterHoming` is the
//! `homing` printer object that owns `G28`.

use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::gcode::{GCmd, GCodeApi, GetOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::mcu::Endstop;
use klippy_api::object::lookup_interface;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::{Rail, Stepper};
use klippy_api::toolhead::{
    HomingMoveApi, HomingStateApi, HomingStateHandle, HomingTarget, HomingTargetApi,
    PrinterHomingApi, ToolHead, ToolHeadApi,
};
use klippy_api::{EventArg, GCell, KResult, KlippyError, ObjectRef};
use pycompat::libm;
use pycompat::text::{float_str, float_true_div, py_sum_floats, str_repr, PyError};
use pycompat::PyValue;
use reactor::{Completion, NEVER, NOW};

use crate::manual_probe::ProbeResult;
use crate::probe::{PrinterProbeApi, ProbeSessionApi};
use crate::stepper_enable::PrinterStepperEnableApi;

/// `HOMING_START_DELAY` (`homing.py:8`).
pub const HOMING_START_DELAY: f64 = 0.001;
/// `ENDSTOP_SAMPLE_TIME` (`homing.py:9`).
pub const ENDSTOP_SAMPLE_TIME: f64 = 0.000015;
/// `ENDSTOP_SAMPLE_COUNT` (`homing.py:10`).
pub const ENDSTOP_SAMPLE_COUNT: i64 = 4;

/// A Python exception that klippy does not catch by class.
fn py_error(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.to_string() }
}

/// `IndexError` of a list subscript.
fn index_error() -> KlippyError {
    py_error("IndexError", "list index out of range")
}

/// `TypeError` of `None - float` (a `None` entry of a move position).
fn none_operand_error() -> KlippyError {
    py_error("TypeError", "unsupported operand type(s) for -: 'NoneType' and 'float'")
}

fn from_py(e: PyError) -> KlippyError {
    py_error(e.kind, &e.msg)
}

/// `coord[0], coord[1], coord[2]` of a position list.
fn coord3(coord: &[f64]) -> KResult<[f64; 3]> {
    match coord {
        [x, y, z, ..] => Ok([*x, *y, *z]),
        _ => Err(index_error()),
    }
}

/// `seq[idx]` of a position list that may hold `None`.
fn opt_at(seq: &[Option<f64>], idx: usize) -> KResult<Option<f64>> {
    seq.get(idx).copied().ok_or_else(index_error)
}

/// `math.sqrt(sum([d*d for d in axes_d[:3]]))`.
fn move_length(axes_d: &[f64]) -> f64 {
    let squares: Vec<f64> = axes_d.iter().take(3).map(|d| d * d).collect();
    libm::sqrt(py_sum_floats(&squares))
}

/// `multi_complete(printer, completions)` (`homing.py:13`): the single
/// completion itself, or a completion that completes when all completions in
/// the list complete, or early when one of them reports an error.
pub fn multi_complete(printer: &Printer, completions: Vec<Completion<bool>>) -> Completion<bool> {
    let reactor = printer.get_reactor();
    if completions.len() == 1 {
        return completions[0].clone();
    }
    // Build completion that waits for all completions
    let all = completions.clone();
    let cp = reactor.register_callback(
        move |_eventtime| {
            for c in &all {
                c.wait(NEVER);
            }
            false
        },
        NOW,
    );
    // If any completion indicates an error, then exit main completion early
    for c in completions {
        let cp = cp.clone();
        reactor.register_callback(
            move |_eventtime| {
                if c.wait(NEVER) == Some(true) {
                    cp.complete(true);
                }
                false
            },
            NOW,
        );
    }
    cp
}

/// `StepperPosition` (`homing.py:26`): tracking of one stepper position during a
/// homing/probing move.
#[derive(Clone)]
pub struct StepperPosition {
    pub stepper: Stepper,
    pub endstop_name: String,
    pub stepper_name: String,
    pub start_pos: i64,
    pub start_cmd_pos: f64,
    pub halt_pos: Option<i64>,
    pub trig_pos: Option<i64>,
}

impl StepperPosition {
    /// `StepperPosition.__init__(stepper, endstop_name)` (`homing.py:27`).
    pub fn new(stepper: Stepper, endstop_name: &str) -> StepperPosition {
        let stepper_name = stepper.get_name(None);
        let start_pos = stepper.get_mcu_position(None);
        let start_cmd_pos = stepper.mcu_to_commanded_position(start_pos);
        StepperPosition {
            stepper,
            endstop_name: endstop_name.to_string(),
            stepper_name,
            start_pos,
            start_cmd_pos,
            halt_pos: None,
            trig_pos: None,
        }
    }

    /// `note_home_end(trigger_time)` (`homing.py:34`).
    pub fn note_home_end(&mut self, trigger_time: f64) {
        self.halt_pos = Some(self.stepper.get_mcu_position(None));
        self.trig_pos = Some(self.stepper.get_past_mcu_position(trigger_time));
    }

    /// `verify_no_probe_skew(haltpos)` (`homing.py:37`).
    pub fn verify_no_probe_skew(&self, _haltpos: &[f64]) {
        let new_start_pos = self.stepper.get_mcu_position(Some(self.start_cmd_pos));
        if new_start_pos != self.start_pos {
            log::warning(&format!(
                "Stepper '{}' position skew after probe: pos {} now {}",
                self.stepper.get_name(None),
                self.start_pos,
                new_start_pos
            ));
        }
    }

    fn halt_pos(&self) -> i64 {
        self.halt_pos.expect("note_home_end ran before the halt position is read")
    }

    fn trig_pos(&self) -> i64 {
        self.trig_pos.expect("note_home_end ran before the trigger position is read")
    }
}

/// `HomingMove` (`homing.py:45`): implementation of homing/probing moves.
pub struct HomingMove {
    printer: Printer,
    endstops: Vec<(Endstop, String)>,
    toolhead: HomingTarget,
    stepper_positions: GCell<Vec<StepperPosition>>,
}

impl HomingMove {
    /// `HomingMove.__init__(printer, endstops, toolhead=None)` (`homing.py:46`).
    pub fn new(
        printer: &Printer,
        endstops: Vec<(Endstop, String)>,
        toolhead: Option<HomingTarget>,
    ) -> KResult<Arc<HomingMove>> {
        let endstops: Vec<(Endstop, String)> =
            endstops.into_iter().filter(|(es, _)| !es.get_steppers().is_empty()).collect();
        let toolhead = match toolhead {
            Some(toolhead) => toolhead,
            None => lookup_interface::<dyn HomingTargetApi>(
                &**printer,
                "toolhead",
                LookupDefault::Raise,
            )?
            .expect("Raise never yields None"),
        };
        Ok(Arc::new(HomingMove {
            printer: printer.clone(),
            endstops,
            toolhead,
            stepper_positions: GCell::new(Vec::new()),
        }))
    }

    /// `stepper_positions` attribute (`homing.py:52`).
    pub fn stepper_positions(&self) -> Vec<StepperPosition> {
        self.stepper_positions.cloned()
    }

    /// `get_mcu_endstops()` (`homing.py:53`).
    pub fn get_mcu_endstops(&self) -> Vec<Endstop> {
        self.endstops.iter().map(|(es, _name)| es.clone()).collect()
    }

    /// `_calc_endstop_rate(mcu_endstop, movepos, speed)` (`homing.py:55`).
    fn calc_endstop_rate(
        &self,
        mcu_endstop: &Endstop,
        movepos: &[Option<f64>],
        speed: f64,
    ) -> KResult<f64> {
        let startpos = self.toolhead.get_position();
        let mut axes_d = Vec::new();
        for (mp, sp) in movepos.iter().zip(startpos.iter()) {
            match mp {
                Some(mp) => axes_d.push(mp - sp),
                None => return Err(none_operand_error()),
            }
        }
        let move_d = move_length(&axes_d);
        let move_t = float_true_div(move_d, speed).map_err(from_py)?;
        let start3 = coord3(&startpos)?;
        let mut max_steps: Option<f64> = None;
        for s in mcu_endstop.get_steppers() {
            let from = s.calc_position_from_coord(start3);
            let to_coord = match movepos {
                [Some(x), Some(y), Some(z), ..] => [*x, *y, *z],
                [_, _, _, ..] => return Err(none_operand_error()),
                _ => return Err(index_error()),
            };
            let to = s.calc_position_from_coord(to_coord);
            let steps =
                float_true_div((from - to).abs(), s.get_step_dist()).map_err(from_py)?;
            max_steps = match max_steps {
                Some(best) => Some(if steps > best { steps } else { best }),
                None => Some(steps),
            };
        }
        let max_steps = match max_steps {
            Some(steps) => steps,
            None => return Err(py_error("ValueError", "max() arg is an empty sequence")),
        };
        if max_steps <= 0.0 {
            return Ok(0.001);
        }
        float_true_div(move_t, max_steps).map_err(from_py)
    }

    /// `calc_toolhead_pos(kin_spos, offsets)` (`homing.py:68`).
    pub fn calc_toolhead_pos(
        &self,
        kin_spos: &IndexMap<String, f64>,
        offsets: &IndexMap<String, i64>,
    ) -> KResult<Vec<f64>> {
        let mut kin_spos = kin_spos.clone();
        let kin = self.toolhead.get_kinematics();
        for stepper in kin.get_steppers() {
            let sname = stepper.get_name(None);
            let current = match kin_spos.get(&sname) {
                Some(current) => *current,
                None => return Err(py_error("KeyError", &str_repr(&sname))),
            };
            let offset = offsets.get(&sname).copied().unwrap_or(0);
            let moved = current + (offset as f64) * stepper.get_step_dist();
            kin_spos.insert(sname, moved);
        }
        let thpos = self.toolhead.get_position();
        let cpos = kin.calc_position(&kin_spos)?;
        let mut result: Vec<f64> = cpos
            .iter()
            .zip(thpos.iter().take(3))
            .map(|(cp, tp)| cp.unwrap_or(*tp))
            .collect();
        result.extend(thpos.iter().skip(3));
        Ok(result)
    }

    /// `homing_move(movepos, speed, probe_pos=False, triggered=True,
    /// check_triggered=True)` (`homing.py:78`). Pauses the reactor.
    pub fn homing_move(
        self: &Arc<Self>,
        movepos: &[Option<f64>],
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<Vec<f64>> {
        // Notify start of homing/probing move
        let handle: Arc<dyn HomingMoveApi> = self.clone();
        self.printer.send_event("homing:homing_move_begin", &[EventArg::handle(handle.clone())])?;
        // Note start location
        self.toolhead.flush_step_generation()?;
        let kin = self.toolhead.get_kinematics();
        let mut kin_spos: IndexMap<String, f64> = IndexMap::new();
        for s in kin.get_steppers() {
            let name = s.get_name(None);
            kin_spos.insert(name, s.get_commanded_position());
        }
        let mut positions = Vec::new();
        for (es, name) in &self.endstops {
            for s in es.get_steppers() {
                positions.push(StepperPosition::new(s, name));
            }
        }
        *self.stepper_positions.borrow_mut() = positions;
        // Start endstop checking
        let print_time = self.toolhead.get_last_move_time()?;
        let mut endstop_triggers = Vec::new();
        for (mcu_endstop, _name) in &self.endstops {
            let rest_time = self.calc_endstop_rate(mcu_endstop, movepos, speed)?;
            let wait = mcu_endstop.home_start(
                print_time,
                ENDSTOP_SAMPLE_TIME,
                ENDSTOP_SAMPLE_COUNT,
                rest_time,
                Some(triggered),
            )?;
            endstop_triggers.push(wait);
        }
        let all_endstop_trigger = multi_complete(&self.printer, endstop_triggers);
        self.toolhead.dwell(HOMING_START_DELAY)?;
        // Issue move
        let mut error: Option<String> = None;
        let movepos_full: Vec<f64> = match movepos.iter().copied().collect::<Option<Vec<f64>>>() {
            Some(full) => full,
            None => return Err(none_operand_error()),
        };
        match self.toolhead.drip_move(movepos_full.clone(), speed, &all_endstop_trigger) {
            Ok(()) => {}
            Err(KlippyError::Command(msg)) => {
                error = Some(format!("Error during homing move: {}", msg));
            }
            Err(e) => return Err(e),
        }
        // Wait for endstops to trigger
        let mut trigger_times: IndexMap<String, f64> = IndexMap::new();
        let move_end_print_time = self.toolhead.get_last_move_time()?;
        for (mcu_endstop, name) in &self.endstops {
            let trigger_time = match mcu_endstop.home_wait(move_end_print_time) {
                Ok(trigger_time) => trigger_time,
                Err(KlippyError::Command(msg)) => {
                    if error.is_none() {
                        error = Some(format!("Error during homing {}: {}", name, msg));
                    }
                    continue;
                }
                Err(e) => return Err(e),
            };
            if trigger_time > 0.0 {
                trigger_times.insert(name.clone(), trigger_time);
            } else if check_triggered && error.is_none() {
                error = Some(format!("No trigger on {} after full movement", name));
            }
        }
        // Determine stepper halt positions
        self.toolhead.flush_step_generation()?;
        for sp in self.stepper_positions.borrow_mut().iter_mut() {
            let tt = trigger_times.get(&sp.endstop_name).copied().unwrap_or(move_end_print_time);
            sp.note_home_end(tt);
        }
        let positions = self.stepper_positions.cloned();
        let trigpos: Vec<f64>;
        if probe_pos {
            let halt_steps: IndexMap<String, i64> = positions
                .iter()
                .map(|sp| (sp.stepper_name.clone(), sp.halt_pos().wrapping_sub(sp.start_pos)))
                .collect();
            let trig_steps: IndexMap<String, i64> = positions
                .iter()
                .map(|sp| (sp.stepper_name.clone(), sp.trig_pos().wrapping_sub(sp.start_pos)))
                .collect();
            let trigpos_calc = self.calc_toolhead_pos(&kin_spos, &trig_steps)?;
            let mut haltpos = trigpos_calc.clone();
            if trig_steps != halt_steps {
                haltpos = self.calc_toolhead_pos(&kin_spos, &halt_steps)?;
            }
            self.toolhead.set_position(haltpos.clone(), "")?;
            for sp in &positions {
                sp.verify_no_probe_skew(&haltpos);
            }
            trigpos = trigpos_calc;
        } else {
            let mut haltpos = movepos_full.clone();
            trigpos = movepos_full.clone();
            let over_steps: IndexMap<String, i64> = positions
                .iter()
                .map(|sp| (sp.stepper_name.clone(), sp.halt_pos().wrapping_sub(sp.trig_pos())))
                .collect();
            if over_steps.values().any(|v| *v != 0) {
                self.toolhead.set_position(movepos_full.clone(), "")?;
                let mut halt_kin_spos: IndexMap<String, f64> = IndexMap::new();
                for s in kin.get_steppers() {
                    let name = s.get_name(None);
                    halt_kin_spos.insert(name, s.get_commanded_position());
                }
                haltpos = self.calc_toolhead_pos(&halt_kin_spos, &over_steps)?;
            }
            self.toolhead.set_position(haltpos, "")?;
        }
        // Signal homing/probing move complete
        match self.printer.send_event("homing:homing_move_end", &[EventArg::handle(handle)]) {
            Ok(_) => {}
            Err(KlippyError::Command(msg)) => {
                if error.is_none() {
                    error = Some(msg);
                }
            }
            Err(e) => return Err(e),
        }
        if let Some(error) = error {
            return Err(KlippyError::Command(error));
        }
        Ok(trigpos)
    }

    /// `check_no_movement()` (`homing.py:155`): the name of the first endstop whose
    /// stepper did not move.
    pub fn check_no_movement(&self) -> Option<String> {
        if let PyValue::Dict(args) = self.printer.get_start_args() {
            let debuginput = args.get(&PyValue::Str("debuginput".to_string()));
            if !matches!(debuginput, None | Some(PyValue::None)) {
                return None;
            }
        }
        for sp in self.stepper_positions.borrow().iter() {
            if Some(sp.start_pos) == sp.trig_pos {
                return Some(sp.endstop_name.clone());
            }
        }
        None
    }
}

impl HomingMoveApi for HomingMove {
    fn get_mcu_endstops(&self) -> Vec<Endstop> {
        HomingMove::get_mcu_endstops(self)
    }

    fn check_no_movement(&self) -> Option<String> {
        HomingMove::check_no_movement(self)
    }
}

/// `Homing` (`homing.py:164`): state tracking of homing requests.
pub struct Homing {
    me: Weak<Homing>,
    printer: Printer,
    toolhead: ToolHead,
    changed_axes: GCell<Vec<i64>>,
    trigger_mcu_pos: GCell<IndexMap<String, i64>>,
    adjust_pos: GCell<IndexMap<String, f64>>,
}

impl Homing {
    /// `Homing.__init__(printer)` (`homing.py:165`).
    pub fn new(printer: &Printer) -> KResult<Arc<Homing>> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        Ok(Arc::new_cyclic(|me| Homing {
            me: me.clone(),
            printer: printer.clone(),
            toolhead,
            changed_axes: GCell::new(Vec::new()),
            trigger_mcu_pos: GCell::new(IndexMap::new()),
            adjust_pos: GCell::new(IndexMap::new()),
        }))
    }

    fn handle(&self) -> HomingStateHandle {
        self.me.upgrade().expect("a Homing is only reachable through its Arc")
    }

    /// `set_axes(axes)` (`homing.py:171`).
    pub fn set_axes(&self, axes: Vec<i64>) {
        self.changed_axes.replace(axes);
    }

    /// `get_axes()` (`homing.py:173`).
    pub fn get_axes(&self) -> Vec<i64> {
        self.changed_axes.cloned()
    }

    /// `get_trigger_position(stepper_name)` (`homing.py:175`).
    pub fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64> {
        match self.trigger_mcu_pos.borrow().get(stepper_name) {
            Some(pos) => Ok(*pos),
            None => Err(py_error("KeyError", &str_repr(stepper_name))),
        }
    }

    /// `set_stepper_adjustment(stepper_name, adjustment)` (`homing.py:177`).
    pub fn set_stepper_adjustment(&self, stepper_name: &str, adjustment: f64) {
        self.adjust_pos.borrow_mut().insert(stepper_name.to_string(), adjustment);
    }

    /// `_fill_coord(coord)` (`homing.py:179`): fill the `None` entries of `coord` with
    /// the current toolhead position.
    fn fill_coord(&self, coord: &[Option<f64>]) -> KResult<Vec<f64>> {
        let mut thcoord = self.toolhead.get_position();
        for (i, c) in coord.iter().enumerate() {
            if let Some(value) = c {
                match thcoord.get_mut(i) {
                    Some(slot) => *slot = *value,
                    None => {
                        return Err(py_error("IndexError", "list assignment index out of range"))
                    }
                }
            }
        }
        Ok(thcoord)
    }

    /// `set_homed_position(pos)` (`homing.py:186`).
    pub fn set_homed_position(&self, pos: &[Option<f64>]) -> KResult<()> {
        let filled = self.fill_coord(pos)?;
        self.toolhead.set_position(filled, "")
    }

    /// `_set_start_position(forcepos)` (`homing.py:189`).
    fn set_start_position(&self, forcepos: &[Option<f64>]) -> KResult<()> {
        let mut homing_axes = String::new();
        for (axis, name) in ["x", "y", "z"].iter().enumerate() {
            if opt_at(forcepos, axis)?.is_some() {
                homing_axes.push_str(name);
            }
        }
        let startpos = self.fill_coord(forcepos)?;
        self.toolhead.set_position(startpos, &homing_axes)
    }

    /// `_retract_move(homing_info, forcepos, movepos)` (`homing.py:194`). Pauses the
    /// reactor.
    fn retract_move(
        &self,
        homing_info: &klippy_api::stepper::HomingInfo,
        forcepos: &[Option<f64>],
        movepos: &[Option<f64>],
    ) -> KResult<Vec<f64>> {
        // Determine retract position and move to it
        let startpos = self.fill_coord(forcepos)?;
        let homepos = self.fill_coord(movepos)?;
        let axes_d: Vec<f64> = homepos.iter().zip(startpos.iter()).map(|(hp, sp)| hp - sp).collect();
        let move_d = move_length(&axes_d);
        let ratio = float_true_div(homing_info.retract_dist, move_d).map_err(from_py)?;
        let retract_r = if ratio < 1.0 { ratio } else { 1.0 };
        let retractpos: Vec<f64> =
            homepos.iter().zip(axes_d.iter()).map(|(hp, ad)| hp - ad * retract_r).collect();
        self.toolhead.r#move(retractpos.clone(), homing_info.retract_speed)?;
        // Force new kinematic position
        let startpos: Vec<f64> =
            retractpos.iter().zip(axes_d.iter()).map(|(rp, ad)| rp - ad * retract_r).collect();
        self.toolhead.set_position(startpos, "")?;
        Ok(homepos)
    }

    /// `_do_home_rails(rails, forcepos, movepos)` (`homing.py:208`). Pauses the reactor.
    fn do_home_rails(
        &self,
        rails: &[Rail],
        forcepos: &[Option<f64>],
        movepos: &[Option<f64>],
    ) -> KResult<()> {
        // Notify of upcoming homing operation
        let rail_args = || EventArg::List(rails.iter().map(|r| EventArg::handle(r.clone())).collect());
        self.printer.send_event(
            "homing:home_rails_begin",
            &[EventArg::handle(self.handle()), rail_args()],
        )?;
        // Alter kinematics class to think printer is at forcepos
        let mut homepos = self.fill_coord(movepos)?;
        self.set_start_position(forcepos)?;
        // Perform first home
        let endstops: Vec<(Endstop, String)> =
            rails.iter().flat_map(|rail| rail.get_endstops()).collect();
        let homing_info = rails.first().ok_or_else(index_error)?.get_homing_info();
        let mut hmove = HomingMove::new(&self.printer, endstops.clone(), None)?;
        hmove.homing_move(&some_all(&homepos), homing_info.speed, false, true, true)?;
        // Perform second home
        if homing_info.retract_dist != 0.0 {
            homepos = self.retract_move(&homing_info, forcepos, movepos)?;
            // Home again
            hmove = HomingMove::new(&self.printer, endstops, None)?;
            hmove.homing_move(&some_all(&homepos), homing_info.second_homing_speed, false, true, true)?;
            if let Some(name) = hmove.check_no_movement() {
                return Err(KlippyError::Command(format!(
                    "Endstop {} still triggered after retract",
                    name
                )));
            }
        }
        // Signal home operation complete
        self.toolhead.flush_step_generation()?;
        let trigger_pos: IndexMap<String, i64> = hmove
            .stepper_positions()
            .iter()
            .map(|sp| (sp.stepper_name.clone(), sp.trig_pos()))
            .collect();
        self.trigger_mcu_pos.replace(trigger_pos);
        self.adjust_pos.replace(IndexMap::new());
        self.printer.send_event(
            "homing:home_rails_end",
            &[EventArg::handle(self.handle()), rail_args()],
        )?;
        let adjust_pos = self.adjust_pos.cloned();
        if adjust_pos.values().any(|v| *v != 0.0) {
            // Apply any homing offsets
            let kin = self.toolhead.get_kinematics();
            let mut homepos = self.toolhead.get_position();
            let mut kin_spos: IndexMap<String, f64> = IndexMap::new();
            for s in kin.get_steppers() {
                let name = s.get_name(None);
                let commanded = s.get_commanded_position();
                let adjust = adjust_pos.get(&s.get_name(None)).copied().unwrap_or(0.0);
                kin_spos.insert(name, commanded + adjust);
            }
            let newpos = kin.calc_position(&kin_spos)?;
            for (axis, axis_name) in ["x", "y", "z"].iter().enumerate() {
                if opt_at(forcepos, axis)?.is_some() {
                    match opt_at(&newpos, axis)? {
                        Some(value) => {
                            *homepos.get_mut(axis).ok_or_else(|| {
                                py_error("IndexError", "list assignment index out of range")
                            })? = value;
                        }
                        None => {
                            return Err(KlippyError::Command(format!(
                                "Cannot determine position of toolhead on axis {} after homing",
                                axis_name
                            )));
                        }
                    }
                }
            }
            self.toolhead.set_position(homepos, "")?;
        }
        Ok(())
    }

    /// `_create_probe_gcmd(attempt_num, probe_speed)` (`homing.py:236`).
    fn create_probe_gcmd(&self, attempt_num: i64, probe_speed: f64) -> KResult<GCmd> {
        let gcode = lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let mut params: IndexMap<String, String> = IndexMap::new();
        params.insert("HOME_ATTEMPT_NUM".to_string(), attempt_num.to_string());
        params.insert("SAMPLES".to_string(), "1".to_string());
        params.insert("PROBE_SPEED".to_string(), float_str(probe_speed));
        Ok(gcode.create_gcode_command("G28", "", &params))
    }

    /// `_probing_home(probe_session, attempt_num, probe_speed)` (`homing.py:241`).
    /// Pauses the reactor.
    fn probing_home(
        &self,
        probe_session: &Arc<dyn ProbeSessionApi>,
        attempt_num: i64,
        probe_speed: f64,
    ) -> KResult<()> {
        // Run probe
        let gcmd = self.create_probe_gcmd(attempt_num, probe_speed)?;
        probe_session.run_probe(&gcmd)?;
        let results: Vec<ProbeResult> = probe_session.pull_probed_results()?;
        let ppos = results.first().ok_or_else(index_error)?;
        // Update toolhead position
        let mut curpos = self.toolhead.get_position();
        *curpos.get_mut(2).ok_or_else(index_error)? -= ppos.bed_z;
        self.toolhead.set_position(curpos, "")
    }

    /// `_do_home_z_via_probe(rails, forcepos, movepos)` (`homing.py:249`). Pauses the
    /// reactor.
    fn do_home_z_via_probe(
        &self,
        rails: &[Rail],
        forcepos: &[Option<f64>],
        movepos: &[Option<f64>],
    ) -> KResult<()> {
        if opt_at(movepos, 0)?.is_some() || opt_at(movepos, 1)?.is_some() {
            return Err(KlippyError::Command(
                "Can only home Z with probe:z_virtual_endstop".to_string(),
            ));
        }
        let homing_info = rails.first().ok_or_else(index_error)?.get_homing_info();
        // Create probe session
        let probe = lookup_interface::<dyn PrinterProbeApi>(
            &*self.printer,
            "probe",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let gcmd = self.create_probe_gcmd(1, homing_info.speed)?;
        let probe_session = probe.start_probe_session(&gcmd)?;
        // Alter kinematics class to think printer is at forcepos
        self.set_start_position(forcepos)?;
        // Perform first home
        self.probing_home(&probe_session, 1, homing_info.speed)?;
        // Perform second home
        if homing_info.retract_dist != 0.0 {
            self.retract_move(&homing_info, forcepos, movepos)?;
            // Home again
            self.probing_home(&probe_session, 2, homing_info.second_homing_speed)?;
        }
        probe_session.end_probe_session()
    }

    /// `home_rails(rails, forcepos, movepos)` (`homing.py:286`). Pauses the reactor.
    pub fn home_rails(
        &self,
        rails: &[Rail],
        forcepos: &[Option<f64>],
        movepos: &[Option<f64>],
    ) -> KResult<()> {
        // Check if this is actually a Z home via probe request
        let endstops: Vec<(Endstop, String)> =
            rails.iter().flat_map(|rail| rail.get_endstops()).collect();
        let probe_endstops: Vec<bool> =
            endstops.iter().map(|(es, _name)| es.get_position_endstop().is_some()).collect();
        if probe_endstops.len() == 1 && probe_endstops[0] {
            // Perform Z home via probe
            return self.do_home_z_via_probe(rails, forcepos, movepos);
        }
        if probe_endstops.iter().any(|p| *p) {
            return Err(KlippyError::Command(
                "Can't home to mix of probe:z_virtual_endstop rails".to_string(),
            ));
        }
        // Perform normal home to endstops
        self.do_home_rails(rails, forcepos, movepos)
    }
}

/// A fully known position as the `Option` list `HomingMove::homing_move` takes.
fn some_all(pos: &[f64]) -> Vec<Option<f64>> {
    pos.iter().map(|v| Some(*v)).collect()
}

impl HomingStateApi for Homing {
    fn set_axes(&self, axes: Vec<i64>) {
        Homing::set_axes(self, axes)
    }

    fn get_axes(&self) -> Vec<i64> {
        Homing::get_axes(self)
    }

    fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64> {
        Homing::get_trigger_position(self, stepper_name)
    }

    fn set_stepper_adjustment(&self, stepper_name: &str, adjustment: f64) {
        Homing::set_stepper_adjustment(self, stepper_name, adjustment)
    }

    fn set_homed_position(&self, pos: Vec<Option<f64>>) -> KResult<()> {
        Homing::set_homed_position(self, &pos)
    }

    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        Homing::home_rails(self, &rails, &forcepos, &movepos)
    }
}

/// `PrinterHoming` (`homing.py:301`): the `homing` printer object.
pub struct PrinterHoming {
    printer: Printer,
}

impl PrinterHoming {
    /// `PrinterHoming.__init__(config)` (`homing.py:302`).
    pub fn new(config: &Config) -> KResult<Arc<PrinterHoming>> {
        let printer = config.get_printer();
        // Register g-code commands
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let homing = Arc::new(PrinterHoming { printer });
        let handler = homing.clone();
        gcode.register_command(
            "G28",
            Some(Box::new(move |gcmd: &GCmd| handler.cmd_g28(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        Ok(homing)
    }

    /// `manual_home(toolhead, endstops, pos, speed, probe_pos, triggered,
    /// check_triggered)` (`homing.py:307`). Pauses the reactor.
    #[allow(clippy::too_many_arguments)]
    pub fn manual_home(
        &self,
        toolhead: HomingTarget,
        endstops: Vec<(Endstop, String)>,
        pos: &[Option<f64>],
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<Vec<f64>> {
        let hmove = HomingMove::new(&self.printer, endstops, Some(toolhead))?;
        match hmove.homing_move(pos, speed, probe_pos, triggered, check_triggered) {
            Err(KlippyError::Command(msg)) => {
                if self.printer.is_shutdown() {
                    return Err(KlippyError::Command(
                        "Homing failed due to printer shutdown".to_string(),
                    ));
                }
                Err(KlippyError::Command(msg))
            }
            other => other,
        }
    }

    /// `check_probe_first_home(gcmd)` (`homing.py:320`).
    pub fn check_probe_first_home(&self, gcmd: &GCmd) -> KResult<bool> {
        if gcmd.get_command() != "G28" {
            return Ok(false);
        }
        let attempt = gcmd.get(
            "HOME_ATTEMPT_NUM",
            GetOpts { default: Some(PyValue::None), ..Default::default() },
        )?;
        Ok(matches!(&attempt, PyValue::Str(s) if s == "1"))
    }

    /// `probing_move(mcu_probe, pos, speed, check_movement=True)` (`homing.py:323`).
    /// Pauses the reactor.
    pub fn probing_move(
        &self,
        mcu_probe: Endstop,
        pos: &[Option<f64>],
        speed: f64,
        check_movement: bool,
    ) -> KResult<Vec<f64>> {
        let endstops = vec![(mcu_probe, "probe".to_string())];
        let hmove = HomingMove::new(&self.printer, endstops, None)?;
        let epos = match hmove.homing_move(pos, speed, true, true, true) {
            Err(KlippyError::Command(msg)) => {
                if self.printer.is_shutdown() {
                    return Err(KlippyError::Command(
                        "Probing failed due to printer shutdown".to_string(),
                    ));
                }
                return Err(KlippyError::Command(msg));
            }
            other => other?,
        };
        if check_movement && hmove.check_no_movement().is_some() {
            return Err(KlippyError::Command("Probe triggered prior to movement".to_string()));
        }
        Ok(epos)
    }

    /// `cmd_G28(gcmd)` (`homing.py:338`). Pauses the reactor.
    pub fn cmd_g28(&self, gcmd: &GCmd) -> KResult<()> {
        // Move to origin
        let mut axes: Vec<i64> = Vec::new();
        for (pos, axis) in ["X", "Y", "Z"].iter().enumerate() {
            let value = gcmd.get(
                axis,
                GetOpts { default: Some(PyValue::None), ..Default::default() },
            )?;
            if !matches!(value, PyValue::None) {
                axes.push(pos as i64);
            }
        }
        if axes.is_empty() {
            axes = vec![0, 1, 2];
        }
        let homing_state = Homing::new(&self.printer)?;
        homing_state.set_axes(axes);
        let toolhead = lookup_interface::<dyn HomingTargetApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let kin = toolhead.get_kinematics();
        let state: HomingStateHandle = homing_state;
        match kin.home(state) {
            Err(KlippyError::Command(msg)) => {
                if self.printer.is_shutdown() {
                    return Err(KlippyError::Command(
                        "Homing failed due to printer shutdown".to_string(),
                    ));
                }
                let stepper_enable = lookup_interface::<dyn PrinterStepperEnableApi>(
                    &*self.printer,
                    "stepper_enable",
                    LookupDefault::Raise,
                )?
                .expect("Raise never yields None");
                stepper_enable.motor_off()?;
                Err(KlippyError::Command(msg))
            }
            other => other,
        }
    }
}

impl PrinterHomingApi for PrinterHoming {
    #[allow(clippy::too_many_arguments)]
    fn manual_home(
        &self,
        toolhead: HomingTarget,
        endstops: Vec<(Endstop, String)>,
        pos: Vec<Option<f64>>,
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<Vec<f64>> {
        PrinterHoming::manual_home(
            self,
            toolhead,
            endstops,
            &pos,
            speed,
            probe_pos,
            triggered,
            check_triggered,
        )
    }

    fn probing_move(
        &self,
        mcu_probe: Endstop,
        pos: Vec<Option<f64>>,
        speed: f64,
        check_movement: bool,
    ) -> KResult<Vec<f64>> {
        PrinterHoming::probing_move(self, mcu_probe, &pos, speed, check_movement)
    }

    fn check_probe_first_home(&self, gcmd: &GCmd) -> KResult<bool> {
        PrinterHoming::check_probe_first_home(self, gcmd)
    }
}

klippy_api::printer_object!(PrinterHoming; dyn PrinterHomingApi);

/// `load_config(config)` (`homing.py:357`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterHoming::new(config)? as ObjectRef))
}

/// Module entry point for the `homing` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
