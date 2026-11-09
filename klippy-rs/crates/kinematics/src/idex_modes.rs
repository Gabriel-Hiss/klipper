//! Port of `klippy/kinematics/idex_modes.py`.

use std::sync::Arc;

use chelper_motion::kin_idex::{dual_carriage_set_sk, dual_carriage_set_transform};
use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts, GetIntOpts, GetOpts, RegisterCommandOpts};
use klippy_api::object::{lookup_interface, EventArg};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_DUAL_CARRIAGE_UPDATE_KINEMATICS};
use klippy_api::stepper::{Rail, Stepper, StepperKinematics};
use klippy_api::toolhead::{HomingStateHandle, ToolHeadApi};
use klippy_core::motion_handles::{alloc_stepper_kinematics, kinematics_of, KinAlloc};
use pycompat::PyValue;

/// `idex_modes.py:10`.
pub const INACTIVE: &str = "INACTIVE";
/// `idex_modes.py:11`.
pub const PRIMARY: &str = "PRIMARY";
/// `idex_modes.py:12`.
pub const COPY: &str = "COPY";
/// `idex_modes.py:13`.
pub const MIRROR: &str = "MIRROR";

/// `DualCarriages.VALID_MODES` (`idex_modes.py:16`).
pub const VALID_MODES: [&str; 4] = [INACTIVE, PRIMARY, COPY, MIRROR];

pub const CMD_SET_DUAL_CARRIAGE_HELP: &str = "Configure the dual carriages mode";
pub const CMD_SAVE_DUAL_CARRIAGE_STATE_HELP: &str = "Save dual carriages modes and positions";
pub const CMD_RESTORE_DUAL_CARRIAGE_STATE_HELP: &str = "Restore dual carriages modes and positions";

fn index_error_list() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

fn index_error_string() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "string index out of range".to_string(),
    }
}

/// Python list indexing `pos[axis]`; negative indices wrap.
fn pos_at(pos: &[f64], axis: i64) -> KResult<f64> {
    let len = pos.len() as i64;
    let idx = if axis < 0 { len + axis } else { axis };
    if idx < 0 || idx >= len {
        return Err(index_error_list());
    }
    Ok(pos[idx as usize])
}


/// `'xyz'[axis]` including negative string indexing.
fn xyz_at(axis: i64) -> KResult<char> {
    let chars = ['x', 'y', 'z'];
    let len = chars.len() as i64;
    let idx = if axis < 0 { len + axis } else { axis };
    if idx < 0 || idx >= len {
        return Err(index_error_string());
    }
    Ok(chars[idx as usize])
}

/// Python `min(a, b)`: the second argument wins only when strictly smaller
/// (matters for `-0.0` / `nan` bit parity).
fn py_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// Python `max(a, b)`: the second argument wins only when strictly greater.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a { b } else { a }
}

/// Python `abs` (`fabs`: clears the sign bit, so `abs(-0.0)` is `0.0`).
fn py_abs(a: f64) -> f64 {
    a.abs()
}

/// `DualCarriagesRail` transform state (`mode`, `scale`, `offset`).
#[derive(Clone, Debug)]
struct RailState {
    mode: String,
    scale: f64,
    offset: f64,
}


/// One dual-carriage rail wrapper (`idex_modes.py:360` `DualCarriagesRail`).
pub struct DualCarriagesRail {
    printer: Printer,
    rail: Rail,
    dual_rail: Option<Rail>,
    sks: Vec<Option<StepperKinematics>>,
    axis: i64,
    safe_dist: Option<f64>,
    state: GCell<RailState>,
}

impl DualCarriagesRail {
    /// `DualCarriagesRail(printer, rail, dual_rail, axis, safe_dist, active)`
    /// (`idex_modes.py:362`).
    pub fn new(
        printer: Printer,
        rail: Rail,
        dual_rail: Option<Rail>,
        axis: i64,
        safe_dist: Option<f64>,
        active: bool,
    ) -> DualCarriagesRail {
        let sks: Vec<Option<StepperKinematics>> =
            rail.get_steppers().into_iter().map(|s| s.get_stepper_kinematics()).collect();
        DualCarriagesRail {
            printer,
            rail,
            dual_rail,
            sks,
            axis,
            safe_dist,
            state: GCell::new(RailState {
                mode: if active { PRIMARY.to_string() } else { INACTIVE.to_string() },
                scale: if active { 1.0 } else { 0.0 },
                offset: 0.0,
            }),
        }
    }

    /// `DualCarriagesRail.get_name()` (the rail's full name).
    pub fn get_name(&self) -> String {
        self.rail.get_name(Some(false))
    }

    /// The wrapped rail.
    pub fn rail(&self) -> Rail {
        self.rail.clone()
    }

    /// The rail this carriage is paired with, if any.
    pub fn dual_rail(&self) -> Option<Rail> {
        self.dual_rail.clone()
    }

    /// `DualCarriagesRail.axis`.
    pub fn axis(&self) -> i64 {
        self.axis
    }

    /// `DualCarriagesRail.safe_dist`.
    pub fn safe_dist(&self) -> Option<f64> {
        self.safe_dist
    }

    /// `DualCarriagesRail.mode`.
    pub fn mode(&self) -> String {
        self.state.borrow().mode.clone()
    }

    /// `DualCarriagesRail.scale`.
    pub fn scale(&self) -> f64 {
        self.state.borrow().scale
    }

    /// `DualCarriagesRail.offset`.
    pub fn offset(&self) -> f64 {
        self.state.borrow().offset
    }

    /// `DualCarriagesRail.is_active()`.
    pub fn is_active(&self) -> bool {
        self.state.borrow().mode != INACTIVE
    }

    /// `DualCarriagesRail.get_axis_position(position)`.
    pub fn get_axis_position(&self, position: &[f64]) -> KResult<f64> {
        let (scale, offset) = { let st = self.state.borrow(); (st.scale, st.offset) };
        Ok(pos_at(position, self.axis)? * scale + offset)
    }

    /// `DualCarriagesRail.apply_transform()`.
    pub fn apply_transform(&self) -> KResult<()> {
        let enc_axes = [b'x', b'y'];
        let (scale, offset) = { let st = self.state.borrow(); (st.scale, st.offset) };
        for sk in &self.sks {
            // Python list indexing wraps negative indices (`ENC_AXES[-1]` is `b'y'`).
            let idx = if self.axis < 0 { 2 + self.axis } else { self.axis };
            if !(0..=1).contains(&idx) {
                return Err(index_error_list());
            }
            let axis_byte = enc_axes[idx as usize];
            // A stepper without kinematics is a NULL pointer in the C call.
            let sk = sk.as_ref().expect("dual_carriage_set_transform: NULL stepper_kinematics (kin_idex.c:51)");
            kinematics_of(sk)?.with(|alloc| match alloc {
                KinAlloc::DualCarriage(dc) => {
                    dual_carriage_set_transform(dc, axis_byte, scale, offset);
                    Ok(())
                }
                _ => Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "stepper kinematics is not a dual_carriage allocation".to_string(),
                }),
            })?;
        }
        self.printer.send_event(EV_DUAL_CARRIAGE_UPDATE_KINEMATICS, &[])?;
        Ok(())
    }

    /// `DualCarriagesRail.activate(mode, position, old_position=None)`.
    pub fn activate(&self, mode: &str, position: &[f64], old_position: Option<&[f64]>) -> KResult<()> {
        let old_pos: &[f64] = old_position.unwrap_or(position);
        let old_axis_position = self.get_axis_position(old_pos)?;
        let scale = if mode == MIRROR { -1.0 } else { 1.0 };
        let offset = old_axis_position - pos_at(position, self.axis)? * scale;
        {
            let mut st = self.state.borrow_mut();
            st.scale = scale;
            st.offset = offset;
        }
        self.apply_transform()?;
        self.state.borrow_mut().mode = mode.to_string();
        Ok(())
    }

    /// `DualCarriagesRail.inactivate(position)`.
    pub fn inactivate(&self, position: &[f64]) -> KResult<()> {
        let offset = self.get_axis_position(position)?;
        {
            let mut st = self.state.borrow_mut();
            st.offset = offset;
            st.scale = 0.0;
        }
        self.apply_transform()?;
        self.state.borrow_mut().mode = INACTIVE.to_string();
        Ok(())
    }

    /// `DualCarriagesRail.override_axis_scaling(new_scale, position)`.
    pub fn override_axis_scaling(&self, new_scale: f64, position: &[f64]) -> KResult<()> {
        let old_axis_position = self.get_axis_position(position)?;
        let scale = new_scale.copysign(self.scale());
        let offset = old_axis_position - pos_at(position, self.axis)? * scale;
        {
            let mut st = self.state.borrow_mut();
            st.scale = scale;
            st.offset = offset;
        }
        self.apply_transform()?;
        Ok(())
    }
}

/// Snapshot saved by `SAVE_DUAL_CARRIAGE_STATE`
/// (`save_dual_carriage_state`, `idex_modes.py:286`).
#[derive(Clone, Debug)]
pub struct SavedDualCarriageState {
    /// `carriage_modes`: carriage name -> mode, in `dc_rails` order.
    pub carriage_modes: IndexMap<String, String>,
    /// `carriage_positions`: carriage name -> axis position.
    pub carriage_positions: IndexMap<String, f64>,
    /// `toolhead_position` (a tuple in Python).
    pub toolhead_position: Vec<f64>,
}

/// The dual-carriage controller (`idex_modes.py:15` `DualCarriages`).
pub struct DualCarriages {
    printer: Printer,
    primary_rails: Vec<Arc<DualCarriagesRail>>,
    dual_rails: Vec<Option<Arc<DualCarriagesRail>>>,
    dc_rails: IndexMap<String, Arc<DualCarriagesRail>>,
    dc_stepper_kinematics: Vec<StepperKinematics>,
    orig_stepper_kinematics: Vec<StepperKinematics>,
    saved_states: GCell<IndexMap<String, SavedDualCarriageState>>,
    axes: Vec<i64>,
}

impl DualCarriages {
    /// `DualCarriages(printer, primary_rails, dual_rails, axes, safe_dist)`
    /// (`idex_modes.py:17`). Registers the `dual_carriage` object, the
    /// `klippy:ready` handler and the three G-code commands.
    pub fn new(
        printer: Printer,
        primary_rails: Vec<Rail>,
        dual_rails: Vec<Option<Rail>>,
        axes: Vec<i64>,
        safe_dist: Vec<Option<f64>>,
    ) -> KResult<Arc<DualCarriages>> {
        // Python runs `_init_steppers` first (`idex_modes.py:19`).
        let (dc_stepper_kinematics, orig_stepper_kinematics) =
            Self::init_steppers(&primary_rails, &dual_rails)?;

        // Copy `safe_dist`, filling in the defaults (`idex_modes.py:20-26`).
        // Every list index is checked where Python indexes it, so short
        // lists raise `IndexError` at the same point.
        let mut safe_dist = safe_dist;
        for (i, dc) in dual_rails.iter().enumerate() {
            let Some(dc) = dc else { continue };
            if list_at(&safe_dist, i)?.is_some() {
                continue;
            }
            let pc = list_at(&primary_rails, i)?;
            let (pc_min, pc_max) = pc.get_range();
            let (dc_min, dc_max) = dc.get_range();
            safe_dist[i] = Some(py_min(py_abs(pc_min - dc_min), py_abs(pc_max - dc_max)));
        }

        let mut activated = [false; 3];
        let mut rail_wrappers = Vec::with_capacity(primary_rails.len());
        for (i, c) in primary_rails.iter().enumerate() {
            let axis = *list_at(&axes, i)?;
            // Python: `activate, activated[axes[i]] = not activated[axes[i]], True`.
            let idx = if axis < 0 { 3 + axis } else { axis };
            if !(0..3).contains(&idx) {
                return Err(index_error_list());
            }
            let activate = !activated[idx as usize];
            activated[idx as usize] = true;
            rail_wrappers.push(Arc::new(DualCarriagesRail::new(
                printer.clone(),
                c.clone(),
                list_at(&dual_rails, i)?.clone(),
                axis,
                *list_at(&safe_dist, i)?,
                activate,
            )));
        }
        let mut dual_wrappers: Vec<Option<Arc<DualCarriagesRail>>> = Vec::with_capacity(dual_rails.len());
        for (i, c) in dual_rails.iter().enumerate() {
            dual_wrappers.push(match c {
                None => None,
                Some(c) => Some(Arc::new(DualCarriagesRail::new(
                    printer.clone(),
                    c.clone(),
                    Some(list_at(&primary_rails, i)?.clone()),
                    *list_at(&axes, i)?,
                    *list_at(&safe_dist, i)?,
                    false,
                ))),
            });
        }

        // `dc_rails`: short rail name -> wrapper, in construction order.
        let mut dc_rails: IndexMap<String, Arc<DualCarriagesRail>> = IndexMap::new();
        for wrapper in rail_wrappers.iter().chain(dual_wrappers.iter().flatten()) {
            let name = wrapper.rail().get_name(Some(true));
            dc_rails.insert(name, wrapper.clone());
        }

        let mut axes_set = axes.clone();
        axes_set.sort_unstable();
        axes_set.dedup();

        let self_arc = Arc::new(DualCarriages {
            printer: printer.clone(),
            primary_rails: rail_wrappers,
            dual_rails: dual_wrappers,
            dc_rails,
            dc_stepper_kinematics,
            orig_stepper_kinematics,
            saved_states: GCell::new(IndexMap::new()),
            axes: axes_set,
        });

        printer.add_object("dual_carriage", self_arc.clone())?;
        {
            let handler_self = self_arc.clone();
            printer.register_event_handler(
                "klippy:ready",
                Box::new(move |_params: &[EventArg]| {
                    handler_self.handle_ready()?;
                    Ok(PyValue::None)
                }),
            );
        }
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        {
            let cmd_self = self_arc.clone();
            gcode.register_command(
                "SET_DUAL_CARRIAGE",
                Some(Box::new(move |gcmd: &GCmd| cmd_self.cmd_set_dual_carriage(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_SET_DUAL_CARRIAGE_HELP.to_string()),
                },
            )?;
        }
        {
            let cmd_self = self_arc.clone();
            gcode.register_command(
                "SAVE_DUAL_CARRIAGE_STATE",
                Some(Box::new(move |gcmd: &GCmd| cmd_self.cmd_save_dual_carriage_state(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_SAVE_DUAL_CARRIAGE_STATE_HELP.to_string()),
                },
            )?;
        }
        {
            let cmd_self = self_arc.clone();
            gcode.register_command(
                "RESTORE_DUAL_CARRIAGE_STATE",
                Some(Box::new(move |gcmd: &GCmd| {
                    cmd_self.cmd_restore_dual_carriage_state(gcmd)
                })),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_RESTORE_DUAL_CARRIAGE_STATE_HELP.to_string()),
                },
            )?;
        }
        Ok(self_arc)
    }

    /// `_init_steppers(primary_rails + dual_rails)` (`idex_modes.py:60`).
    /// Steppers are deduplicated by identity in first-seen order; the
    /// Python `set` order is unobservable because every stepper receives
    /// its own wrapper. Returns `(dc_stepper_kinematics,
    /// orig_stepper_kinematics)`.
    ///
    /// Each wrapper keeps a raw pointer to its original kinematics
    /// (`dual_carriage_set_sk`); the controller keeps the originals alive
    /// in `orig_stepper_kinematics`, as Python does.
    fn init_steppers(
        primary_rails: &[Rail],
        dual_rails: &[Option<Rail>],
    ) -> KResult<(Vec<StepperKinematics>, Vec<StepperKinematics>)> {
        let mut steppers: Vec<Stepper> = Vec::new();
        let grids: Vec<&Rail> = primary_rails.iter().chain(dual_rails.iter().flatten()).collect();
        for rail in grids {
            let c_steppers = rail.get_steppers();
            if c_steppers.is_empty() {
                return Err(KlippyError::config(format!(
                    "At least one stepper must be associated with carriage: {}",
                    rail.get_name(Some(false))
                )));
            }
            for s in c_steppers {
                if !steppers.iter().any(|st: &Stepper| Arc::ptr_eq(st, &s)) {
                    steppers.push(s);
                }
            }
        }
        let mut dc_kinematics = Vec::with_capacity(steppers.len());
        let mut orig_kinematics = Vec::with_capacity(steppers.len());
        for s in &steppers {
            let sk = alloc_stepper_kinematics("dual_carriage_alloc", &[])?;
            let orig_sk = s
                .get_stepper_kinematics()
                .expect("dual_carriage_set_sk: NULL orig_sk (kin_idex.c:43)");
            let orig_ptr = kinematics_of(&orig_sk)?.sk_ptr();
            kinematics_of(&sk)?.with(|alloc| {
                let KinAlloc::DualCarriage(dc) = alloc else {
                    unreachable!("dual_carriage_alloc returns a dual carriage allocation")
                };
                // SAFETY: `orig_sk` is boxed behind its handle and is kept
                // alive by the controller for as long as the wrapper.
                unsafe { dual_carriage_set_sk(dc, orig_ptr) };
            });
            dc_kinematics.push(sk.clone());
            orig_kinematics.push(orig_sk);
            s.set_stepper_kinematics(Some(sk));
        }
        Ok((dc_kinematics, orig_kinematics))
    }

    /// `DualCarriages.dc_stepper_kinematics`.
    pub fn dc_stepper_kinematics(&self) -> &[StepperKinematics] {
        &self.dc_stepper_kinematics
    }

    /// `DualCarriages.orig_stepper_kinematics`.
    pub fn orig_stepper_kinematics(&self) -> &[StepperKinematics] {
        &self.orig_stepper_kinematics
    }

    /// `DualCarriages.primary_rails`.
    pub fn primary_rails(&self) -> &[Arc<DualCarriagesRail>] {
        &self.primary_rails
    }

    /// `DualCarriages.dual_rails`.
    pub fn dual_rails(&self) -> &[Option<Arc<DualCarriagesRail>>] {
        &self.dual_rails
    }

    /// `DualCarriages.dc_rails` (short rail name -> wrapper, ordered).
    pub fn dc_rails(&self) -> &IndexMap<String, Arc<DualCarriagesRail>> {
        &self.dc_rails
    }

    fn toolhead(&self) -> KResult<klippy_api::toolhead::ToolHead> {
        Ok(lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None"))
    }

    /// `DualCarriages.get_axes()`.
    pub fn get_axes(&self) -> Vec<i64> {
        self.axes.clone()
    }

    /// `DualCarriages.get_primary_rail(axis)`: the underlying `Rail` of
    /// the active `PRIMARY` wrapper on `axis`, or `None`.
    pub fn get_primary_rail(&self, axis: i64) -> Option<Rail> {
        for dc_rail in self.dc_rails.values() {
            if dc_rail.mode() == PRIMARY && dc_rail.axis() == axis {
                return Some(dc_rail.rail());
            }
        }
        None
    }

    /// `DualCarriages.get_dc_rail_wrapper(rail)`.
    pub fn get_dc_rail_wrapper(&self, rail: &Rail) -> Option<Arc<DualCarriagesRail>> {
        for dc_rail in self.dc_rails.values() {
            if Arc::ptr_eq(&dc_rail.rail(), rail) {
                return Some(dc_rail.clone());
            }
        }
        None
    }

    /// `DualCarriages.get_transform(rail)`: `(scale, offset)` or `(0., 0.)`.
    pub fn get_transform(&self, rail: &Rail) -> (f64, f64) {
        match self.get_dc_rail_wrapper(rail) {
            Some(dc_rail) => (dc_rail.scale(), dc_rail.offset()),
            None => (0.0, 0.0),
        }
    }

    /// `DualCarriages.is_active(rail)`.
    pub fn is_active(&self, rail: &Rail) -> bool {
        match self.get_dc_rail_wrapper(rail) {
            Some(dc_rail) => dc_rail.is_active(),
            None => false,
        }
    }

    /// `DualCarriages.toggle_active_dc_rail(target_dc)`.
    pub fn toggle_active_dc_rail(&self, target_dc: &Arc<DualCarriagesRail>) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let pos = toolhead.get_position();
        let kin = toolhead.get_kinematics();
        let axis = target_dc.axis();
        for dc in self.dc_rails.values() {
            if !Arc::ptr_eq(dc, target_dc) && dc.axis() == axis && dc.is_active() {
                let pos_clone = pos.clone();
                dc.inactivate(&pos_clone)?;
            }
        }
        if target_dc.mode() != PRIMARY {
            let mut newpos = pos.clone();
            let target_pos = target_dc.get_axis_position(&pos)?;
            put_pos(&mut newpos, axis, target_pos)?;
            let newpos_clone = newpos.clone();
            target_dc.activate(PRIMARY, &newpos_clone, Some(&pos))?;
            toolhead.set_position(newpos.clone(), "")?;
        }
        kin.update_limits(axis, target_dc.rail().get_range())?;
        Ok(())
    }

    /// `DualCarriages.home(homing_state, axis)`.
    pub fn home(&self, homing_state: HomingStateHandle, axis: i64) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let kin = toolhead.get_kinematics();
        let homing_rails: Vec<Arc<DualCarriagesRail>> = self
            .primary_rails
            .iter()
            .filter(|r| r.axis() == axis)
            .cloned()
            .collect();
        for dc_rail in &homing_rails {
            let mut dcs: Vec<Arc<DualCarriagesRail>> = self
                .dc_rails
                .values()
                .filter(|dc| {
                    Arc::ptr_eq(&dc_rail.rail(), &dc.rail())
                        || dc.dual_rail().map(|dr| Arc::ptr_eq(&dc_rail.rail(), &dr)).unwrap_or(false)
                })
                .cloned()
                .collect();
            if dcs.len() > 1
                && (self.get_dc_order(&dcs[0], &dcs[1]) > 0)
                    != dcs[0].rail().get_homing_info().positive_dir
            {
                dcs.reverse();
            }
            for dc in &dcs {
                self.toggle_active_dc_rail(dc)?;
                kin.home_axis(homing_state.clone(), dc.axis(), dc.rail())?;
            }
        }
        let Some(first) = homing_rails.first() else {
            return Err(index_error_list());
        };
        self.activate_dc_mode(first, PRIMARY)?;
        Ok(())
    }

    /// `DualCarriages.get_status(eventtime)`; `eventtime` is unused.
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut carriages = IndexMap::new();
        for dc in self.dc_rails.values() {
            carriages.insert(PyValue::Str(dc.get_name()), PyValue::Str(dc.mode()));
        }
        let mut status = IndexMap::new();
        status.insert(PyValue::Str("carriages".to_string()), PyValue::Dict(carriages));
        if self.dc_rails.len() == 2 {
            for (i, dc) in self.dc_rails.values().enumerate() {
                status.insert(
                    PyValue::Str(format!("carriage_{i}")),
                    PyValue::Str(dc.mode()),
                );
            }
        }
        PyValue::Dict(status)
    }

    /// `DualCarriages.get_kin_range(toolhead, axis)`.
    pub fn get_kin_range(
        &self,
        toolhead: &klippy_api::toolhead::ToolHead,
        axis: i64,
    ) -> KResult<(f64, f64)> {
        let pos = toolhead.get_position();
        let axis_pos = pos_at(&pos, axis)?;
        let mut range_min = -1e10;
        let mut range_max = 1e10;
        for carriage in self.dc_rails.values() {
            if carriage.axis() != axis {
                continue;
            }
            // `dcs = [carriage] + [dc for dc in ... if carriage.rail is dc.dual_rail]`.
            let mut dcs: Vec<Arc<DualCarriagesRail>> = vec![carriage.clone()];
            for dc in self.dc_rails.values() {
                if let Some(dr) = dc.dual_rail() {
                    if Arc::ptr_eq(&carriage.rail(), &dr) {
                        dcs.push(dc.clone());
                    }
                }
            }
            let mut axes_pos = Vec::with_capacity(dcs.len());
            for dc in &dcs {
                axes_pos.push(dc.get_axis_position(&pos)?);
            }
            if !dcs[0].is_active() {
                continue;
            }
            let mode0 = dcs[0].mode();
            let (rmin, rmax) = dcs[0].rail().get_range();
            if mode0 == PRIMARY {
                range_min = py_max(range_min, rmin);
                range_max = py_min(range_max, rmax);
            } else if mode0 == COPY {
                range_min = py_max(range_min, axis_pos + rmin - axes_pos[0]);
                range_max = py_min(range_max, axis_pos + rmax - axes_pos[0]);
            } else if mode0 == MIRROR {
                range_min = py_max(range_min, axis_pos + axes_pos[0] - rmax);
                range_max = py_min(range_max, axis_pos + axes_pos[0] - rmin);
            }
            let safe_dist = dcs[0].safe_dist();
            let safe_dist_val = match safe_dist {
                None | Some(0.0) => None,
                Some(v) => Some(v),
            };
            if safe_dist_val.is_none() || dcs.len() == 1 {
                continue;
            }
            let safe_dist = safe_dist_val.unwrap();
            let mode1 = dcs[1].mode();
            if mode0 == mode1
                || ((mode0 == PRIMARY && mode1 == COPY) || (mode0 == COPY && mode1 == PRIMARY))
            {
                continue;
            }
            let dcs_dist = axes_pos[1] - axes_pos[0];
            let mut safe_move_dist = if self.get_dc_order(&dcs[0], &dcs[1]) > 0 {
                dcs_dist + safe_dist
            } else {
                dcs_dist - safe_dist
            };
            if dcs[1].is_active() {
                safe_move_dist *= 0.5;
            }
            if mode0 == PRIMARY || mode0 == COPY {
                if self.get_dc_order(&dcs[0], &dcs[1]) > 0 {
                    range_min = py_max(range_min, axis_pos + safe_move_dist);
                } else {
                    range_max = py_min(range_max, axis_pos + safe_move_dist);
                }
            } else {
                // `dcs[0].mode == MIRROR`
                if self.get_dc_order(&dcs[0], &dcs[1]) > 0 {
                    range_max = py_min(range_max, axis_pos - safe_move_dist);
                } else {
                    range_min = py_max(range_min, axis_pos - safe_move_dist);
                }
            }
        }
        if range_min > range_max {
            return Ok((range_min, range_min));
        }
        Ok((range_min, range_max))
    }

    /// `DualCarriages.get_dc_order(first_dc, second_dc)`.
    pub fn get_dc_order(&self, first_dc: &Arc<DualCarriagesRail>, second_dc: &Arc<DualCarriagesRail>) -> i64 {
        if Arc::ptr_eq(first_dc, second_dc) {
            return 0;
        }
        let first_rail = first_dc.rail();
        let second_rail = second_dc.rail();
        let first_homing_info = first_rail.get_homing_info();
        let second_homing_info = second_rail.get_homing_info();
        if first_homing_info.positive_dir != second_homing_info.positive_dir {
            return if first_homing_info.positive_dir { 1 } else { -1 };
        }
        if first_rail.get_homing_info().position_endstop > second_rail.get_homing_info().position_endstop {
            return 1;
        }
        -1
    }

    /// `DualCarriages.activate_dc_mode(dc, mode)`.
    pub fn activate_dc_mode(&self, dc: &Arc<DualCarriagesRail>, mode: &str) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        let axis = dc.axis();
        if mode == INACTIVE {
            dc.inactivate(&toolhead.get_position())?;
        } else if mode == PRIMARY {
            self.toggle_active_dc_rail(dc)?;
        } else {
            dc.activate(mode, &toolhead.get_position(), None)?;
        }
        kin.update_limits(axis, self.get_kin_range(&toolhead, axis)?)?;
        Ok(())
    }

    /// `DualCarriages._handle_ready()` (`klippy:ready` handler).
    pub fn handle_ready(&self) -> KResult<()> {
        for dc_rail in self.dc_rails.values() {
            dc_rail.apply_transform()?;
        }
        Ok(())
    }

    /// `DualCarriages.cmd_SET_DUAL_CARRIAGE(gcmd)`.
    pub fn cmd_set_dual_carriage(&self, gcmd: &GCmd) -> KResult<()> {
        let carriage_val = gcmd.get(
            "CARRIAGE",
            GetOpts { default: Some(PyValue::None), ..Default::default() },
        )?;
        let carriage_str = match carriage_val {
            PyValue::None => return Err(KlippyError::command("CARRIAGE must be specified")),
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let dc_rail: Arc<DualCarriagesRail> = match self.dc_rails.get(&carriage_str) {
            Some(rail) => rail.clone(),
            None => {
                let mut resolved: Option<Arc<DualCarriagesRail>> = None;
                if self.dc_rails.len() == 2 {
                    match int_carriage_index(&carriage_str) {
                        CarriageIndex::Index(text) => {
                            let index = match text.parse::<i64>() {
                                Ok(i @ (0 | 1)) => i,
                                _ => {
                                    return Err(KlippyError::command(format!(
                                        "Invalid CARRIAGE={text} index"
                                    )));
                                }
                            };
                            resolved = Some(if index != 0 {
                                match self.dual_rails.first() {
                                    Some(Some(r)) => r.clone(),
                                    _ => {
                                        // `dual_rails[0]` is `None`: `dc_rail`
                                        // stays `None` below.
                                        return Err(KlippyError::command(format!(
                                            "Invalid CARRIAGE={carriage_str} specified"
                                        )));
                                    }
                                }
                            } else {
                                match self.primary_rails.first() {
                                    Some(r) => r.clone(),
                                    None => {
                                        return Err(index_error_list());
                                    }
                                }
                            });
                        }
                        CarriageIndex::NotANumber => {}
                    }
                }
                match resolved {
                    Some(r) => r,
                    None => {
                        return Err(KlippyError::command(format!(
                            "Invalid CARRIAGE={carriage_str} specified"
                        )))
                    }
                }
            }
        };

        let mode_val = gcmd.get(
            "MODE",
            GetOpts {
                default: Some(PyValue::Str(PRIMARY.to_string())),
                ..Default::default()
            },
        )?;
        let mode = mode_val.str().to_uppercase();
        if !VALID_MODES.contains(&mode.as_str()) {
            return Err(KlippyError::command(format!("Invalid mode={mode} specified")));
        }
        if mode == COPY || mode == MIRROR {
            let primary = self.get_primary_rail(dc_rail.axis());
            let same = primary
                .as_ref()
                .map(|r| Arc::ptr_eq(r, &dc_rail.rail()))
                .unwrap_or(false);
            if primary.is_none() || same {
                return Err(KlippyError::command(
                    "Must activate another carriage as PRIMARY first",
                ));
            }
            let curtime = self.printer.get_reactor().monotonic();
            let toolhead = self.toolhead()?;
            let kin = toolhead.get_kinematics();
            let axis_ch = xyz_at(dc_rail.axis())?;
            let status = kin.get_status(curtime)?;
            if !homed_axes_contains(&status, axis_ch)? {
                return Err(KlippyError::command(format!(
                    "Axis {} must be homed prior to enabling mode={}",
                    axis_ch.to_uppercase(),
                    mode
                )));
            }
        }
        if mode == INACTIVE {
            let active_dcs: Vec<&Arc<DualCarriagesRail>> = self
                .dc_rails
                .values()
                .filter(|dc| dc.is_active() && dc.axis() == dc_rail.axis())
                .collect();
            if active_dcs.len() == 1 && Arc::ptr_eq(active_dcs[0], &dc_rail) {
                let axis_ch = xyz_at(dc_rail.axis())?;
                return Err(KlippyError::command(format!(
                    "Cannot deactivate the only active carriage for axis {}",
                    axis_ch.to_uppercase()
                )));
            }
        }
        self.activate_dc_mode(&dc_rail, &mode)
    }

    /// `DualCarriages.cmd_SAVE_DUAL_CARRIAGE_STATE(gcmd)`.
    pub fn cmd_save_dual_carriage_state(&self, gcmd: &GCmd) -> KResult<()> {
        let state_name = gcmd.get(
            "NAME",
            GetOpts {
                default: Some(PyValue::Str("default".to_string())),
                ..Default::default()
            },
        )?;
        let state_name = match state_name {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let saved = self.save_dual_carriage_state()?;
        self.saved_states.borrow_mut().insert(state_name, saved);
        Ok(())
    }

    /// `DualCarriages.save_dual_carriage_state()`.
    pub fn save_dual_carriage_state(&self) -> KResult<SavedDualCarriageState> {
        let toolhead = self.toolhead()?;
        let pos = toolhead.get_position();
        let mut carriage_modes = IndexMap::new();
        let mut carriage_positions = IndexMap::new();
        for dc in self.dc_rails.values() {
            carriage_modes.insert(dc.get_name(), dc.mode());
            carriage_positions.insert(dc.get_name(), dc.get_axis_position(&pos)?);
        }
        Ok(SavedDualCarriageState {
            carriage_modes,
            carriage_positions,
            toolhead_position: pos,
        })
    }

    /// `DualCarriages.cmd_RESTORE_DUAL_CARRIAGE_STATE(gcmd)`.
    pub fn cmd_restore_dual_carriage_state(&self, gcmd: &GCmd) -> KResult<()> {
        let state_name = gcmd.get(
            "NAME",
            GetOpts {
                default: Some(PyValue::Str("default".to_string())),
                ..Default::default()
            },
        )?;
        let state_name = match state_name {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let saved_state = self
            .saved_states
            .borrow()
            .get(&state_name)
            .cloned();
        let saved_state = match saved_state {
            Some(s) => s,
            None => {
                return Err(KlippyError::command(format!(
                    "Unknown DUAL_CARRIAGE state: {state_name}"
                )))
            }
        };
        let move_speed = gcmd.get_float(
            "MOVE_SPEED",
            GetFloatOpts { default: Some(Some(0.0)), above: Some(0.0), ..Default::default() },
        )?;
        let move_flag = gcmd.get_int(
            "MOVE",
            GetIntOpts { default: Some(Some(1)), ..Default::default() },
        )?;
        let move_speed = move_speed.unwrap_or(0.0);
        let move_flag = move_flag.unwrap_or(1);
        self.restore_dual_carriage_state(&saved_state, move_flag, move_speed)
    }

    /// `DualCarriages.restore_dual_carriage_state(saved_state, move,
    /// move_speed=0.)`.
    pub fn restore_dual_carriage_state(
        &self,
        saved_state: &SavedDualCarriageState,
        move_flag: i64,
        move_speed: f64,
    ) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let mut move_pos = toolhead.get_position();
        let dcs: Vec<Arc<DualCarriagesRail>> =
            self.dc_rails.values().cloned().collect();
        if move_flag != 0 {
            let mut homing_speed = 99999999.0;
            let mut cur_pos = Vec::with_capacity(dcs.len());
            for dc in &dcs {
                self.toggle_active_dc_rail(dc)?;
                homing_speed = py_min(homing_speed, dc.rail().get_homing_info().speed);
                cur_pos.push(toolhead.get_position());
            }
            let mut dl = Vec::with_capacity(dcs.len());
            for (i, dc) in dcs.iter().enumerate() {
                let saved = carriage_position_of(saved_state, &dc.get_name())?;
                dl.push(saved - pos_at(&cur_pos[i], dc.axis())?);
            }
            for &axis in &self.axes {
                let dc_ind: Vec<usize> = dcs
                    .iter()
                    .enumerate()
                    .filter(|(_, dc)| dc.axis() == axis)
                    .map(|(i, _)| i)
                    .collect();
                let abs_dl: Vec<f64> = dc_ind.iter().map(|&i| py_abs(dl[i])).collect();
                // `abs_dl.index(max(abs_dl))`: `max` keeps the first
                // strictly greater element and `index` finds that element.
                let mut max_pos: Option<usize> = None;
                for (p, &v) in abs_dl.iter().enumerate() {
                    match max_pos {
                        None => max_pos = Some(p),
                        Some(m) if v > abs_dl[m] => max_pos = Some(p),
                        _ => {}
                    }
                }
                let max_pos = max_pos.ok_or_else(|| KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: "max() arg is an empty sequence".to_string(),
                })?;
                let primary_ind = dc_ind[max_pos];
                let primary_dc = dcs[primary_ind].clone();
                self.toggle_active_dc_rail(&primary_dc)?;
                let primary_saved = carriage_position_of(saved_state, &primary_dc.get_name())?;
                put_pos(&mut move_pos, axis, primary_saved)?;
                for &secondary_ind in &dc_ind {
                    if secondary_ind == primary_ind {
                        continue;
                    }
                    if py_min(py_abs(dl[primary_ind]), py_abs(dl[secondary_ind])) < 0.000000001 {
                        continue;
                    }
                    let dc_mode = if dl[primary_ind] * dl[secondary_ind] > 0.0 {
                        COPY
                    } else {
                        MIRROR
                    };
                    dcs[secondary_ind].activate(dc_mode, &cur_pos[primary_ind], None)?;
                    dcs[secondary_ind].override_axis_scaling(
                        py_abs(dl[secondary_ind] / dl[primary_ind]),
                        &cur_pos[primary_ind],
                    )?;
                }
            }
            let speed = if move_speed != 0.0 { move_speed } else { homing_speed };
            let coord: Vec<Option<f64>> = move_pos.iter().map(|&v| Some(v)).collect();
            toolhead.manual_move(coord, speed)?;
            toolhead.flush_step_generation()?;
        }
        // Inactivate all carriages to restore scaling coefficients and
        // compute kinematics limits.
        for dc in &dcs {
            dc.inactivate(&move_pos)?;
        }
        // Restore the toolhead position for axes with no primary carriage.
        for &axis in &self.axes {
            let saved_pos = pos_at(&saved_state.toolhead_position, axis)?;
            put_pos(&mut move_pos, axis, saved_pos)?;
        }
        toolhead.set_position(move_pos.clone(), "")?;
        let saved_modes = saved_state.carriage_modes.clone();
        // `saved_modes[dc.get_name()]` raises `KeyError` for a missing name.
        let saved_mode_of = |dc: &Arc<DualCarriagesRail>| -> KResult<String> {
            saved_modes
                .get(&dc.get_name())
                .cloned()
                .ok_or_else(|| KlippyError::Internal {
                    class_name: "KeyError".to_string(),
                    message: format!("'{}'", dc.get_name()),
                })
        };
        let mut saved_primary_dcs: Vec<Arc<DualCarriagesRail>> = Vec::new();
        for dc in self.dc_rails.values() {
            if saved_mode_of(dc)? == PRIMARY {
                saved_primary_dcs.push(dc.clone());
            }
        }
        for dc in &saved_primary_dcs {
            self.activate_dc_mode(dc, PRIMARY)?;
        }
        for dc in self.dc_rails.values() {
            if !saved_primary_dcs.iter().any(|p| Arc::ptr_eq(p, dc)) {
                let mode = saved_mode_of(dc)?;
                self.activate_dc_mode(dc, &mode)?;
            }
        }
        Ok(())
    }
}

impl klippy_api::PrinterObject for DualCarriages {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn DualCarriagesApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_status(eventtime)))
    }
}

/// The methods other kinematics modules call on the `dual_carriage`
/// printer object (`generic_cartesian.py`, `cartesian.py`,
/// `hybrid_corexy.py`, `hybrid_corexz.py`).
pub trait DualCarriagesApi: Send + Sync + 'static {
    /// `DualCarriages.get_axes()`.
    fn get_axes(&self) -> Vec<i64>;
    /// `DualCarriages.get_primary_rail(axis)`.
    fn get_primary_rail(&self, axis: i64) -> Option<Rail>;
    /// `DualCarriages.get_dc_rail_wrapper(rail)`.
    fn get_dc_rail_wrapper(&self, rail: &Rail) -> Option<Arc<DualCarriagesRail>>;
    /// `DualCarriages.get_transform(rail)`.
    fn get_transform(&self, rail: &Rail) -> (f64, f64);
    /// `DualCarriages.is_active(rail)`.
    fn is_active(&self, rail: &Rail) -> bool;
    /// `DualCarriages.home(homing_state, axis)`.
    fn home(&self, homing_state: HomingStateHandle, axis: i64) -> KResult<()>;
    /// `DualCarriages.save_dual_carriage_state()`.
    fn save_dual_carriage_state(&self) -> KResult<SavedDualCarriageState>;
    /// `DualCarriages.restore_dual_carriage_state(saved_state, move, move_speed)`.
    fn restore_dual_carriage_state(
        &self,
        saved_state: &SavedDualCarriageState,
        move_flag: i64,
        move_speed: f64,
    ) -> KResult<()>;
}

impl DualCarriagesApi for DualCarriages {
    fn get_axes(&self) -> Vec<i64> {
        DualCarriages::get_axes(self)
    }
    fn get_primary_rail(&self, axis: i64) -> Option<Rail> {
        DualCarriages::get_primary_rail(self, axis)
    }
    fn get_dc_rail_wrapper(&self, rail: &Rail) -> Option<Arc<DualCarriagesRail>> {
        DualCarriages::get_dc_rail_wrapper(self, rail)
    }
    fn get_transform(&self, rail: &Rail) -> (f64, f64) {
        DualCarriages::get_transform(self, rail)
    }
    fn is_active(&self, rail: &Rail) -> bool {
        DualCarriages::is_active(self, rail)
    }
    fn home(&self, homing_state: HomingStateHandle, axis: i64) -> KResult<()> {
        DualCarriages::home(self, homing_state, axis)
    }
    fn save_dual_carriage_state(&self) -> KResult<SavedDualCarriageState> {
        DualCarriages::save_dual_carriage_state(self)
    }
    fn restore_dual_carriage_state(
        &self,
        saved_state: &SavedDualCarriageState,
        move_flag: i64,
        move_speed: f64,
    ) -> KResult<()> {
        DualCarriages::restore_dual_carriage_state(self, saved_state, move_flag, move_speed)
    }
}

/// Python `list[i]`: `IndexError` past the end.
fn list_at<T>(list: &[T], i: usize) -> KResult<&T> {
    list.get(i).ok_or_else(index_error_list)
}

fn put_pos(pos: &mut [f64], axis: i64, value: f64) -> KResult<()> {
    let len = pos.len() as i64;
    let idx = if axis < 0 { len + axis } else { axis };
    if idx < 0 || idx >= len {
        return Err(index_error_list());
    }
    pos[idx as usize] = value;
    Ok(())
}

fn carriage_position_of(saved_state: &SavedDualCarriageState, name: &str) -> KResult<f64> {
    saved_state.carriage_positions.get(name).copied().ok_or_else(|| KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: format!("'{name}'"),
    })
}

enum CarriageIndex {
    /// The decimal text of the Python int (`%d` prints all its digits).
    Index(String),
    NotANumber,
}

/// `int(carriage_str.strip())` with the `ValueError` swallowed.
fn int_carriage_index(carriage_str: &str) -> CarriageIndex {
    let stripped = carriage_str.trim_matches(|c: char| c.is_whitespace() || matches!(c, '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{1f}'));
    match pycompat::text::int_from_string(stripped, 10) {
        Ok(v) => CarriageIndex::Index(v.inner().to_string()),
        Err(_) => CarriageIndex::NotANumber,
    }
}

/// `axis not in kin.get_status(curtime)['homed_axes']`, negated: a string
/// containing the axis letter, or a list holding it. A missing key is the
/// `KeyError` Python raises.
fn homed_axes_contains(status: &PyValue, axis: char) -> KResult<bool> {
    if let PyValue::Dict(map) = status {
        for (k, v) in map {
            if matches!(k, PyValue::Str(s) if s == "homed_axes") {
                return Ok(match v {
                    PyValue::Str(s) => s.contains(axis),
                    PyValue::List(items) | PyValue::Tuple(items) => items
                        .iter()
                        .any(|item| matches!(item, PyValue::Str(s) if *s == axis.to_string())),
                    _ => false,
                });
            }
        }
    }
    Err(KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: "'homed_axes'".to_string(),
    })
}


/// Module entry point. The Python module has no factory entry point
/// (`idex_modes` is imported by the kinematics modules), so every field
/// is `None`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}

