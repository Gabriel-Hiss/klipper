//! Port of the `gcode_move` module (`klippy/extras/gcode_move.py`).

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetFloatOpts, GetIntOpts, GetOpts, GCodeApi, GCodeMoveApi, GCmd, MoveTransform,
    RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{lookup_interface, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::GCell;
use pycompat::text::{float_from_string, percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, Weak};

#[derive(Clone)]
struct SavedState {
    absolute_coord: bool,
    absolute_extrude: bool,
    base_position: Vec<f64>,
    last_position: Vec<f64>,
    homing_position: Vec<f64>,
    speed: f64,
    speed_factor: f64,
    extrude_factor: f64,
}

/// `klippy/extras/gcode_move.py:GCodeMove`.
pub struct GCodeMove {
    printer: Printer,
    is_printer_ready: GCell<bool>,
    absolute_coord: GCell<bool>,
    absolute_extrude: GCell<bool>,
    base_position: GCell<Vec<f64>>,
    last_position: GCell<Vec<f64>>,
    homing_position: GCell<Vec<f64>>,
    axis_map: GCell<IndexMap<String, usize>>,
    speed: GCell<f64>,
    speed_factor: GCell<f64>,
    extrude_factor: GCell<f64>,
    saved_states: GCell<IndexMap<String, SavedState>>,
    /// `self.move_transform`: the registered transform (`None` until one is set).
    move_transform: GCell<Option<MoveTransform>>,
    /// The object behind `self.move_with_transform` / `self.position_with_transform`:
    /// the toolhead from `_handle_ready`, or the last transform set. Setting a `None`
    /// transform clears `move_transform` but leaves these bound methods unchanged.
    with_transform: GCell<Option<MoveTransform>>,
}

fn default_axis_map() -> IndexMap<String, usize> {
    let mut m = IndexMap::new();
    m.insert("X".to_string(), 0usize);
    m.insert("Y".to_string(), 1usize);
    m.insert("Z".to_string(), 2usize);
    m.insert("E".to_string(), 3usize);
    m
}

fn coord_padded(v: &[f64]) -> Vec<f64> {
    let mut out = v.to_vec();
    while out.len() < 4 {
        out.push(0.0);
    }
    out
}

fn list_pyvalue(v: &[f64]) -> String {
    let list: Vec<PyValue> = v.iter().map(|&f| PyValue::Float(f)).collect();
    PyValue::List(list).repr()
}

fn pct_float_list(v: &[f64]) -> PctValue {
    PctValue::Str(list_pyvalue(v))
}

impl GCodeMove {
    /// `GCodeMove.__init__(config)` (`klippy/extras/gcode_move.py:9`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let this = Arc::new(GCodeMove {
            printer: printer.clone(),
            is_printer_ready: GCell::new(false),
            absolute_coord: GCell::new(true),
            absolute_extrude: GCell::new(true),
            base_position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            last_position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            homing_position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            axis_map: GCell::new(default_axis_map()),
            speed: GCell::new(25.0),
            speed_factor: GCell::new(1.0 / 60.0),
            extrude_factor: GCell::new(1.0),
            saved_states: GCell::new(IndexMap::new()),
            move_transform: GCell::new(None),
            with_transform: GCell::new(None),
        });
        Self::register_commands(&this, &gcode);
        Self::register_events(&this);
        Ok(this)
    }

    fn register_commands(this: &Arc<Self>, gcode: &Arc<dyn GCodeApi>) {
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_G1(gcmd));
            gcode
                .register_command(
                    "G1",
                    Some(h),
                    RegisterCommandOpts { when_not_ready: false, desc: None },
                )
                .expect("register G1");
        }
        for cmd in ["G20", "G21", "M82", "M83", "G90", "G91", "G92", "M220", "M221"] {
            let t = Arc::clone(this);
            let h = match cmd {
                "G20" => Box::new(move |gcmd: &GCmd| t.cmd_G20(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "G21" => Box::new(move |gcmd: &GCmd| t.cmd_G21(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "M82" => Box::new(move |gcmd: &GCmd| t.cmd_M82(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "M83" => Box::new(move |gcmd: &GCmd| t.cmd_M83(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "G90" => Box::new(move |gcmd: &GCmd| t.cmd_G90(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "G91" => Box::new(move |gcmd: &GCmd| t.cmd_G91(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "G92" => Box::new(move |gcmd: &GCmd| t.cmd_G92(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "M220" => Box::new(move |gcmd: &GCmd| t.cmd_M220(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                "M221" => Box::new(move |gcmd: &GCmd| t.cmd_M221(gcmd)) as Box<
                    dyn FnMut(&GCmd) -> KResult<()> + Send,
                >,
                _ => unreachable!(),
            };
            gcode
                .register_command(
                    cmd,
                    Some(h),
                    RegisterCommandOpts { when_not_ready: false, desc: None },
                )
                .expect("register coordinate command");
        }
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_SET_GCODE_OFFSET(gcmd));
            gcode
                .register_command(
                    "SET_GCODE_OFFSET",
                    Some(h),
                    RegisterCommandOpts {
                        when_not_ready: false,
                        desc: Some("Set a virtual offset to g-code positions".to_string()),
                    },
                )
                .expect("register SET_GCODE_OFFSET");
        }
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_SAVE_GCODE_STATE(gcmd));
            gcode
                .register_command(
                    "SAVE_GCODE_STATE",
                    Some(h),
                    RegisterCommandOpts {
                        when_not_ready: false,
                        desc: Some("Save G-Code coordinate state".to_string()),
                    },
                )
                .expect("register SAVE_GCODE_STATE");
        }
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_RESTORE_GCODE_STATE(gcmd));
            gcode
                .register_command(
                    "RESTORE_GCODE_STATE",
                    Some(h),
                    RegisterCommandOpts {
                        when_not_ready: false,
                        desc: Some("Restore a previously saved G-Code state".to_string()),
                    },
                )
                .expect("register RESTORE_GCODE_STATE");
        }
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_G1(gcmd));
            gcode
                .register_command(
                    "G0",
                    Some(h),
                    RegisterCommandOpts { when_not_ready: false, desc: None },
                )
                .expect("register G0");
        }
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_M114(gcmd));
            gcode
                .register_command(
                    "M114",
                    Some(h),
                    RegisterCommandOpts { when_not_ready: true, desc: None },
                )
                .expect("register M114");
        }
        {
            let t = Arc::clone(this);
            let h = Box::new(move |gcmd: &GCmd| t.cmd_GET_POSITION(gcmd));
            gcode
                .register_command(
                    "GET_POSITION",
                    Some(h),
                    RegisterCommandOpts {
                        when_not_ready: true,
                        desc: Some(
                            "Return information on the current location of the toolhead".to_string(),
                        ),
                    },
                )
                .expect("register GET_POSITION");
        }
    }

    fn register_events(this: &Arc<Self>) {
        let printer = this.printer.clone();
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("klippy:ready", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.handle_ready()?;
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("klippy:shutdown", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.handle_shutdown();
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("klippy:analyze_shutdown", Box::new(move |args| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.handle_analyze_shutdown();
                }
                let _ = args;
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("toolhead:set_position", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.reset_last_position()?;
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("toolhead:manual_move", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.reset_last_position()?;
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("toolhead:update_extra_axes", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.update_extra_axes()?;
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("gcode:command_error", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.reset_last_position()?;
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("extruder:activate_extruder", Box::new(move |_| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.handle_activate_extruder()?;
                }
                Ok(PyValue::None)
            }));
        }
        {
            let weak = Arc::downgrade(this);
            printer.register_event_handler("homing:home_rails_end", Box::new(move |args| {
                if let Some(obj) = Weak::upgrade(&weak) {
                    obj.handle_home_rails_end(args)?;
                }
                Ok(PyValue::None)
            }));
        }
    }

    fn handle_ready(&self) -> KResult<()> {
        self.is_printer_ready.set(true);
        if self.move_transform.borrow().is_none() {
            let toolhead =
                lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                    .expect("Raise never yields None");
            *self.with_transform.borrow_mut() = Some(MoveTransform::ToolHead(toolhead));
        }
        self.reset_last_position()
    }

    fn handle_shutdown(&self) {
        self.is_printer_ready.set(false);
    }

    fn handle_analyze_shutdown(&self) {
        let absolute_coord = self.absolute_coord.get();
        let absolute_extrude = self.absolute_extrude.get();
        let base_position = self.base_position.cloned();
        let last_position = self.last_position.cloned();
        let homing_position = self.homing_position.cloned();
        let speed_factor = self.speed_factor.get();
        let extrude_factor = self.extrude_factor.get();
        let speed = self.speed.get();
        let fmt = "gcode state: absolute_coord=%s absolute_extrude=%s base_position=%s last_position=%s homing_position=%s speed_factor=%s extrude_factor=%s speed=%s";
        let args = vec![
            PctValue::Bool(absolute_coord),
            PctValue::Bool(absolute_extrude),
            pct_float_list(&base_position),
            pct_float_list(&last_position),
            pct_float_list(&homing_position),
            PctValue::Float(speed_factor),
            PctValue::Float(extrude_factor),
            PctValue::Float(speed),
        ];
        if let Ok(msg) = percent_format(fmt, &args) {
            log::info(&msg);
        }
    }

    fn handle_activate_extruder(&self) -> KResult<()> {
        self.reset_last_position()?;
        self.extrude_factor.set(1.0);
        let last_e = self.last_position.borrow().get(3).copied().unwrap_or(0.0);
        self.base_position.borrow_mut()[3] = last_e;
        Ok(())
    }

    fn handle_home_rails_end(&self, args: &[EventArg]) -> KResult<()> {
        self.reset_last_position()?;
        if args.len() < 1 {
            return Ok(());
        }
        let homing = match args[0].as_handle::<dyn klippy_api::toolhead::HomingStateApi>() {
            Some(h) => h,
            None => return Ok(()),
        };
        let axes = homing.get_axes();
        for axis in axes {
            let idx = axis;
            let homing_val = {
                let homing_pos = self.homing_position.borrow();
                if idx < 0 || (idx as usize) >= homing_pos.len() {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list assignment index out of range".to_string(),
                    });
                }
                homing_pos[idx as usize]
            };
            let mut base = self.base_position.borrow_mut();
            if idx < 0 || (idx as usize) >= base.len() {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list assignment index out of range".to_string(),
                });
            }
            base[idx as usize] = homing_val;
        }
        Ok(())
    }

    /// `self.move_with_transform(pos, speed)`.
    fn do_move(&self, pos: Vec<f64>, speed: f64) -> KResult<()> {
        let transform = self.with_transform.borrow().clone();
        match transform {
            Some(MoveTransform::ToolHead(th)) => th.r#move(pos, speed),
            Some(MoveTransform::Custom(c)) => c.r#move(pos, speed),
            None => Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not callable".to_string(),
            }),
        }
    }

    /// `self.position_with_transform()` (initially `lambda: [0., 0., 0., 0.]`).
    fn get_position_transform(&self) -> KResult<Vec<f64>> {
        let transform = self.with_transform.borrow().clone();
        match transform {
            Some(MoveTransform::ToolHead(th)) => Ok(th.get_position()),
            Some(MoveTransform::Custom(c)) => c.get_position(),
            None => Ok(vec![0.0, 0.0, 0.0, 0.0]),
        }
    }

    fn gcode_position(&self) -> Vec<f64> {
        let last = self.last_position.cloned();
        let base = self.base_position.cloned();
        let n = last.len().min(base.len());
        let mut p: Vec<f64> = (0..n).map(|i| last[i] - base[i]).collect();
        if p.len() > 3 {
            let ef = self.extrude_factor.get();
            p[3] /= ef;
        }
        p
    }

    fn update_extra_axes(&self) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let extra_axes = toolhead.get_extra_axes();
        let mut axis_map = default_axis_map();
        for (index, ea) in extra_axes.iter().enumerate() {
            let Some(ea) = ea.as_ref() else { continue };
            let Some(gcode_id) = ea.get_axis_gcode_id() else { continue };
            if gcode_id.len() != 1
                || !gcode_id.chars().next().is_some_and(|c| c.is_ascii_uppercase())
                || axis_map.contains_key(&gcode_id)
                || gcode_id == "F"
                || gcode_id == "N"
            {
                continue;
            }
            axis_map.insert(gcode_id, index);
        }
        *self.axis_map.borrow_mut() = axis_map;
        {
            let mut base = self.base_position.borrow_mut();
            if extra_axes.len() > 4 {
                if base.len() < extra_axes.len() {
                    base.resize(extra_axes.len(), 0.0);
                }
                for v in base[4..extra_axes.len()].iter_mut() {
                    *v = 0.0;
                }
            } else {
                base.truncate(4);
            }
        }
        self.reset_last_position()?;
        Ok(())
    }

    /// `cmd_G1` (`klippy/extras/gcode_move.py:134`), also registered as `G0`.
    pub fn cmd_G1(&self, gcmd: &GCmd) -> KResult<()> {
        let commandline = gcmd.get_commandline();
        let params = gcmd.get_command_parameters();
        let axis_map = self.axis_map.cloned();
        let absolute_coord = self.absolute_coord.get();
        let absolute_extrude = self.absolute_extrude.get();
        let extrude_factor = self.extrude_factor.get();
        for (axis, pos) in axis_map.iter() {
            if !params.contains_key(axis) {
                continue;
            }
            let raw = &params[axis];
            let v = match float_from_string(raw) {
                Ok(v) => v,
                Err(_) => {
                    return Err(KlippyError::command(format!(
                        "Unable to parse move '{}'",
                        commandline
                    )));
                }
            };
            let mut v = v;
            let mut eff_absolute = absolute_coord;
            if axis == "E" {
                v *= extrude_factor;
                if !absolute_extrude {
                    eff_absolute = false;
                }
            }
            let mut last = self.last_position.borrow_mut();
            let base = self.base_position.borrow();
            if !eff_absolute {
                last[*pos] += v;
            } else {
                last[*pos] = v + base[*pos];
            }
        }
        if params.contains_key("F") {
            let raw = &params["F"];
            let gcode_speed = match float_from_string(raw) {
                Ok(v) => v,
                Err(_) => {
                    return Err(KlippyError::command(format!(
                        "Unable to parse move '{}'",
                        commandline
                    )));
                }
            };
            if gcode_speed <= 0.0 {
                return Err(KlippyError::command(format!(
                    "Invalid speed in '{}'",
                    commandline
                )));
            }
            let sf = self.speed_factor.get();
            self.speed.set(gcode_speed * sf);
        }
        let pos = self.last_position.cloned();
        let spd = self.speed.get();
        self.do_move(pos, spd)
    }

    /// `cmd_G20` (`klippy/extras/gcode_move.py:163`).
    pub fn cmd_G20(&self, _gcmd: &GCmd) -> KResult<()> {
        Err(KlippyError::command(
            "Machine does not support G20 (inches) command".to_string(),
        ))
    }

    /// `cmd_G21` (`klippy/extras/gcode_move.py:166`).
    pub fn cmd_G21(&self, _gcmd: &GCmd) -> KResult<()> {
        Ok(())
    }

    /// `cmd_M82` (`klippy/extras/gcode_move.py:169`).
    pub fn cmd_M82(&self, _gcmd: &GCmd) -> KResult<()> {
        self.absolute_extrude.set(true);
        Ok(())
    }

    /// `cmd_M83` (`klippy/extras/gcode_move.py:172`).
    pub fn cmd_M83(&self, _gcmd: &GCmd) -> KResult<()> {
        self.absolute_extrude.set(false);
        Ok(())
    }

    /// `cmd_G90` (`klippy/extras/gcode_move.py:175`).
    pub fn cmd_G90(&self, _gcmd: &GCmd) -> KResult<()> {
        self.absolute_coord.set(true);
        Ok(())
    }

    /// `cmd_G91` (`klippy/extras/gcode_move.py:178`).
    pub fn cmd_G91(&self, _gcmd: &GCmd) -> KResult<()> {
        self.absolute_coord.set(false);
        Ok(())
    }

    /// `cmd_G92` (`klippy/extras/gcode_move.py:181`).
    pub fn cmd_G92(&self, gcmd: &GCmd) -> KResult<()> {
        let mut offsets: Vec<Option<f64>> = Vec::new();
        for a in ['X', 'Y', 'Z', 'E'] {
            let name = a.to_string();
            offsets.push(gcmd.get_float(
                &name,
                GetFloatOpts { default: Some(None), ..Default::default() },
            )?);
        }
        let extrude_factor = self.extrude_factor.get();
        for (i, offset) in offsets.iter().enumerate() {
            if let Some(mut offset) = offset {
                if i == 3 {
                    offset *= extrude_factor;
                }
                let last_val = self.last_position.borrow()[i];
                self.base_position.borrow_mut()[i] = last_val - offset;
            }
        }
        if offsets.iter().all(|o| o.is_none()) {
            let last4: Vec<f64> = self.last_position.borrow()[..4].to_vec();
            self.base_position.borrow_mut()[..4].copy_from_slice(&last4);
        }
        Ok(())
    }

    /// `cmd_M114` (`klippy/extras/gcode_move.py:191`).
    pub fn cmd_M114(&self, gcmd: &GCmd) -> KResult<()> {
        let p = self.gcode_position();
        let mut padded = p.clone();
        while padded.len() < 4 {
            padded.push(0.0);
        }
        let args = vec![
            PctValue::Float(padded[0]),
            PctValue::Float(padded[1]),
            PctValue::Float(padded[2]),
            PctValue::Float(padded[3]),
        ];
        let msg = percent_format("X:%.3f Y:%.3f Z:%.3f E:%.3f", &args)
            .unwrap_or_default();
        gcmd.respond_raw(&msg);
        Ok(())
    }

    /// `cmd_M220` (`klippy/extras/gcode_move.py:195`).
    pub fn cmd_M220(&self, gcmd: &GCmd) -> KResult<()> {
        let s = gcmd.get_float(
            "S",
            GetFloatOpts { default: Some(Some(100.0)), above: Some(0.0), ..Default::default() },
        )?
        .unwrap();
        let value = s / (60.0 * 100.0);
        let old_speed = self.speed.get();
        let old_factor = self.speed_factor.get();
        let gcode_speed = old_speed / old_factor;
        self.speed.set(gcode_speed * value);
        self.speed_factor.set(value);
        Ok(())
    }

    /// `cmd_M221` (`klippy/extras/gcode_move.py:200`).
    pub fn cmd_M221(&self, gcmd: &GCmd) -> KResult<()> {
        let s = gcmd.get_float(
            "S",
            GetFloatOpts { default: Some(Some(100.0)), above: Some(0.0), ..Default::default() },
        )?
        .unwrap();
        let new_extrude_factor = s / 100.0;
        let last_e_pos = self.last_position.borrow()[3];
        let base_e = self.base_position.borrow()[3];
        let old_factor = self.extrude_factor.get();
        let e_value = (last_e_pos - base_e) / old_factor;
        self.base_position.borrow_mut()[3] = last_e_pos - e_value * new_extrude_factor;
        self.extrude_factor.set(new_extrude_factor);
        Ok(())
    }

    /// `cmd_SET_GCODE_OFFSET` (`klippy/extras/gcode_move.py:208`).
    pub fn cmd_SET_GCODE_OFFSET(&self, gcmd: &GCmd) -> KResult<()> {
        let mut move_delta = [0.0, 0.0, 0.0, 0.0];
        for (pos, axis) in ['X', 'Y', 'Z', 'E'].iter().enumerate() {
            let mut offset =
                gcmd.get_float(&axis.to_string(), GetFloatOpts { default: Some(None), ..Default::default() })?;
            if offset.is_none() {
                offset = gcmd.get_float(&format!("{}_ADJUST", axis), GetFloatOpts { default: Some(None), ..Default::default() })?;
                if offset.is_none() {
                    continue;
                }
                offset = Some(offset.unwrap() + self.homing_position.borrow()[pos]);
            }
            let offset = offset.unwrap();
            let delta = offset - self.homing_position.borrow()[pos];
            move_delta[pos] = delta;
            self.base_position.borrow_mut()[pos] += delta;
            self.homing_position.borrow_mut()[pos] = offset;
        }
        let move_flag = gcmd.get_int(
            "MOVE",
            GetIntOpts { default: Some(Some(0)), ..Default::default() },
        )?
        .unwrap();
        if move_flag != 0 {
            let speed = gcmd.get_float(
                "MOVE_SPEED",
                GetFloatOpts { default: Some(Some(self.speed.get())), above: Some(0.0), ..Default::default() },
            )?
            .unwrap();
            for pos in 0..4 {
                self.last_position.borrow_mut()[pos] += move_delta[pos];
            }
            let pos_vec = self.last_position.cloned();
            self.do_move(pos_vec, speed)?;
        }
        Ok(())
    }

    /// `cmd_SAVE_GCODE_STATE` (`klippy/extras/gcode_move.py:228`).
    pub fn cmd_SAVE_GCODE_STATE(&self, gcmd: &GCmd) -> KResult<()> {
        let state_name = gcmd_get_name(gcmd)?;
        let state = SavedState {
            absolute_coord: self.absolute_coord.get(),
            absolute_extrude: self.absolute_extrude.get(),
            base_position: self.base_position.cloned(),
            last_position: self.last_position.cloned(),
            homing_position: self.homing_position.cloned(),
            speed: self.speed.get(),
            speed_factor: self.speed_factor.get(),
            extrude_factor: self.extrude_factor.get(),
        };
        self.saved_states.borrow_mut().insert(state_name, state);
        Ok(())
    }

    /// `cmd_RESTORE_GCODE_STATE` (`klippy/extras/gcode_move.py:240`).
    pub fn cmd_RESTORE_GCODE_STATE(&self, gcmd: &GCmd) -> KResult<()> {
        let state_name = gcmd_get_name(gcmd)?;
        let state = {
            let saved = self.saved_states.borrow();
            match saved.get(&state_name) {
                Some(s) => s.clone(),
                None => {
                    return Err(KlippyError::command(format!(
                        "Unknown g-code state: {}",
                        state_name
                    )));
                }
            }
        };
        self.absolute_coord.set(state.absolute_coord);
        self.absolute_extrude.set(state.absolute_extrude);
        {
            let mut base = self.base_position.borrow_mut();
            base[..4].copy_from_slice(&state.base_position[..4]);
        }
        *self.homing_position.borrow_mut() = state.homing_position.clone();
        self.speed.set(state.speed);
        self.speed_factor.set(state.speed_factor);
        self.extrude_factor.set(state.extrude_factor);
        let e_diff = self.last_position.borrow()[3] - state.last_position[3];
        self.base_position.borrow_mut()[3] += e_diff;
        let move_flag = gcmd.get_int(
            "MOVE",
            GetIntOpts { default: Some(Some(0)), ..Default::default() },
        )?
        .unwrap();
        if move_flag != 0 {
            let speed = gcmd.get_float(
                "MOVE_SPEED",
                GetFloatOpts { default: Some(Some(self.speed.get())), above: Some(0.0), ..Default::default() },
            )?
            .unwrap();
            {
                let saved_last = state.last_position.clone();
                let mut last = self.last_position.borrow_mut();
                last[..3].copy_from_slice(&saved_last[..3]);
            }
            let pos_vec = self.last_position.cloned();
            self.do_move(pos_vec, speed)?;
        }
        Ok(())
    }

    /// `cmd_GET_POSITION` (`klippy/extras/gcode_move.py:263`).
    pub fn cmd_GET_POSITION(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead = match lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::ReturnNone,
        )? {
            Some(t) => t,
            None => {
                return Err(KlippyError::command("Printer not ready".to_string()));
            }
        };
        let kin = toolhead.get_kinematics();
        let steppers = kin.get_steppers();
        let mcu_parts: Vec<String> = steppers
            .iter()
            .map(|s| {
                let name = s.get_name(None);
                let pos = s.get_mcu_position(None);
                percent_format("%s:%d", &[PctValue::Str(name), PctValue::Int(PyInt::from_i64(pos))])
                    .unwrap_or_default()
            })
            .collect();
        let mcu_pos = mcu_parts.join(" ");
        let cinfo: Vec<(String, f64)> = steppers
            .iter()
            .map(|s| (s.get_name(None), s.get_commanded_position()))
            .collect();
        let stepper_parts: Vec<String> = cinfo
            .iter()
            .map(|(a, v)| {
                percent_format("%s:%.6f", &[PctValue::Str(a.clone()), PctValue::Float(*v)])
                    .unwrap_or_default()
            })
            .collect();
        let stepper_pos = stepper_parts.join(" ");
        let mut cmap = IndexMap::new();
        for (a, v) in cinfo.iter() {
            cmap.insert(a.clone(), *v);
        }
        let kin_vals = kin.calc_position(&cmap)?;
        let mut kin_parts = Vec::new();
        for (a, v) in "XYZ".chars().zip(kin_vals.iter()) {
            let fv = match v {
                Some(f) => *f,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: "unsupported format string passed to None.__format__".to_string(),
                    });
                }
            };
            kin_parts.push(
                percent_format(
                    "%s:%.6f",
                    &[PctValue::Str(a.to_string()), PctValue::Float(fv)],
                )
                .unwrap_or_default(),
            );
        }
        let kin_pos = kin_parts.join(" ");
        let th_vec = toolhead.get_position();
        let mut toolhead_parts = Vec::new();
        for (a, v) in "XYZE".chars().zip(th_vec.iter().take(4)) {
            toolhead_parts.push(
                percent_format(
                    "%s:%.6f",
                    &[PctValue::Str(a.to_string()), PctValue::Float(*v)],
                )
                .unwrap_or_default(),
            );
        }
        let toolhead_pos = toolhead_parts.join(" ");
        let last_vec = self.last_position.cloned();
        let mut gcode_parts = Vec::new();
        for (a, v) in "XYZE".chars().zip(last_vec.iter()) {
            gcode_parts.push(
                percent_format(
                    "%s:%.6f",
                    &[PctValue::Str(a.to_string()), PctValue::Float(*v)],
                )
                .unwrap_or_default(),
            );
        }
        let gcode_pos = gcode_parts.join(" ");
        let base_vec = self.base_position.cloned();
        let mut base_parts = Vec::new();
        for (a, v) in "XYZE".chars().zip(base_vec.iter()) {
            base_parts.push(
                percent_format(
                    "%s:%.6f",
                    &[PctValue::Str(a.to_string()), PctValue::Float(*v)],
                )
                .unwrap_or_default(),
            );
        }
        let base_pos = base_parts.join(" ");
        let homing_vec = self.homing_position.cloned();
        let mut homing_parts = Vec::new();
        for (a, v) in "XYZ".chars().zip(homing_vec.iter()) {
            homing_parts.push(
                percent_format(
                    "%s:%.6f",
                    &[PctValue::Str(a.to_string()), PctValue::Float(*v)],
                )
                .unwrap_or_default(),
            );
        }
        let homing_pos = homing_parts.join(" ");
        let fmt = "mcu: %s\nstepper: %s\nkinematic: %s\ntoolhead: %s\ngcode: %s\ngcode base: %s\ngcode homing: %s";
        let args = vec![
            PctValue::Str(mcu_pos),
            PctValue::Str(stepper_pos),
            PctValue::Str(kin_pos),
            PctValue::Str(toolhead_pos),
            PctValue::Str(gcode_pos),
            PctValue::Str(base_pos),
            PctValue::Str(homing_pos),
        ];
        let msg = percent_format(fmt, &args).unwrap_or_default();
        gcmd.respond_info(&msg, true);
        Ok(())
    }
}

fn gcmd_get_name(gcmd: &GCmd) -> KResult<String> {
    let v = gcmd.get(
        "NAME",
        GetOpts { default: Some(PyValue::Str("default".to_string())), ..Default::default() },
    )?;
    Ok(match v {
        PyValue::Str(s) => s,
        other => other.str(),
    })
}

impl GCodeMoveApi for GCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let last = self.last_position.cloned();
        let speed_factor = self.speed_factor.get();
        let speed_val = self.speed.get();
        let extrude = self.extrude_factor.get();
        let abs_coord = self.absolute_coord.get();
        let abs_extr = self.absolute_extrude.get();
        let homing = self.homing_position.cloned();
        let gpos = self.gcode_position();
        let axis = self.axis_map.cloned();
        let mut dict = IndexMap::new();
        dict.insert(PyValue::Str("speed_factor".to_string()), PyValue::Float(speed_factor * 60.0));
        dict.insert(PyValue::Str("speed".to_string()), PyValue::Float(speed_val / speed_factor));
        dict.insert(PyValue::Str("extrude_factor".to_string()), PyValue::Float(extrude));
        dict.insert(PyValue::Str("absolute_coordinates".to_string()), PyValue::Bool(abs_coord));
        dict.insert(PyValue::Str("absolute_extrude".to_string()), PyValue::Bool(abs_extr));
        dict.insert(
            PyValue::Str("homing_origin".to_string()),
            PyValue::Tuple(coord_padded(&homing).into_iter().map(PyValue::Float).collect()),
        );
        dict.insert(
            PyValue::Str("position".to_string()),
            PyValue::Tuple(coord_padded(&last).into_iter().map(PyValue::Float).collect()),
        );
        dict.insert(
            PyValue::Str("gcode_position".to_string()),
            PyValue::Tuple(coord_padded(&gpos).into_iter().map(PyValue::Float).collect()),
        );
        let mut amap = IndexMap::new();
        for (k, v) in axis.iter() {
            amap.insert(PyValue::Str(k.clone()), PyValue::Int(PyInt::from_i64(*v as i64)));
        }
        dict.insert(PyValue::Str("axis_map".to_string()), PyValue::Dict(amap));
        PyValue::Dict(dict)
    }

    fn set_move_transform(
        &self,
        transform: Option<MoveTransform>,
        force: bool,
    ) -> KResult<Option<MoveTransform>> {
        if self.move_transform.borrow().is_some() && !force {
            return Err(KlippyError::config(
                "G-Code move transform already specified".to_string(),
            ));
        }
        let mut old = self.move_transform.borrow().clone();
        if old.is_none() {
            if let Ok(Some(th)) = lookup_interface::<dyn ToolHeadApi>(
                &*self.printer,
                "toolhead",
                LookupDefault::ReturnNone,
            ) {
                old = Some(MoveTransform::ToolHead(th));
            }
        }
        *self.move_transform.borrow_mut() = transform.clone();
        // `self.move_with_transform = transform.move`
        let Some(transform) = transform else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'move'".to_string(),
            });
        };
        *self.with_transform.borrow_mut() = Some(transform);
        Ok(old)
    }

    fn reset_last_position(&self) -> KResult<()> {
        if self.is_printer_ready.get() {
            let pos = self.get_position_transform()?;
            *self.last_position.borrow_mut() = pos;
        }
        Ok(())
    }

    fn cmd_g1(&self, gcmd: &GCmd) -> KResult<()> {
        self.cmd_G1(gcmd)
    }
}

impl PrinterObject for GCodeMove {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeMoveApi);
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(<Self as GCodeMoveApi>::get_status(self, Some(eventtime))))
    }
}

/// `load_config(config)` (`klippy/extras/gcode_move.py:293`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = GCodeMove::new(config)?;
    Ok(Some(obj as ObjectRef))
}

/// Module entry point for the `gcode_move` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
