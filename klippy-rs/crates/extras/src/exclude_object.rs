//! Port of the `exclude_object` module (`klippy/extras/exclude_object.py`).
//!
//! Lazily installs itself as the `gcode_move` move transform on the first
//! exclusion and then ignores (but tracks) any move inside an excluded object.
//! Extrusion offsets are tracked per extruder so that the transform forwards
//! positions relative to the last printed object.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandHandler, GCodeMove, GCodeMoveApi, GCmd, GetOpts,
    MoveTransform, MoveTransformApi, MoveTransformHandle, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{load_interface, lookup_interface, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::{GCell, ObjectRef};
use pycompat::json as pyjson;
use pycompat::PyValue;
use std::sync::Arc;
use crate::tuning_tower::TuningTowerApi;

/// Command help texts (note the exact upstream spelling, kept identical).
const HELP_START: &str = "Marks the beginning the current object as labeled";
const HELP_END: &str = "Marks the end the current object";
const HELP_EXCLUDE_OBJECT: &str = "Cancel moves inside a specified objects";
const HELP_EXCLUDE_OBJECT_DEFINE: &str = "Provides a summary of an object";

/// `AttributeError` the way CPython reports a missing attribute on this class.
fn attr_error(attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'ExcludeObject' object has no attribute '{attr}'"),
    }
}

/// `AttributeError` for a `None` transform (`NoneType` has no such attribute).
fn none_attr_error(attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'NoneType' object has no attribute '{attr}'"),
    }
}

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `gcmd.get(name, default=None)` → the raw value (`None` when absent).
fn get_opt(gcmd: &GCmd, name: &str) -> KResult<PyValue> {
    gcmd.get(
        name,
        GetOpts {
            default: Some(PyValue::None),
            ..GetOpts::default()
        },
    )
}

/// `gcmd.get(name)` → the required value.
fn get_required(gcmd: &GCmd, name: &str) -> KResult<PyValue> {
    gcmd.get(name, GetOpts::default())
}

fn pyvalue_str(v: &PyValue) -> Option<String> {
    match v {
        PyValue::Str(s) => Some(s.clone()),
        _ => None,
    }
}

fn dict_get<'a>(v: &'a PyValue, key: &str) -> Option<&'a PyValue> {
    match v {
        PyValue::Dict(d) => d.get(&PyValue::Str(key.to_string())),
        _ => None,
    }
}

/// `obj["name"]` of an object definition dict.
fn object_name(v: &PyValue) -> KResult<String> {
    match dict_get(v, "name") {
        Some(PyValue::Str(s)) => Ok(s.clone()),
        _ => Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "'name'".to_string(),
        }),
    }
}

/// Python `max(a, b)`: `b` only when `b > a` (NaN never wins).
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

fn dict_name_dict(name: String) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("name".to_string()), PyValue::Str(name));
    PyValue::Dict(d)
}

fn json_decode_error(e: pycompat::text::PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: "JSONDecodeError".to_string(),
        message: e.msg,
    }
}

/// `klippy/extras/exclude_object.py:11` `ExcludeObject`.
pub struct ExcludeObject {
    printer: Printer,
    gcode: GCode,
    gcode_move: GCodeMove,
    toolhead: GCell<Option<ToolHead>>,
    objects: GCell<Vec<PyValue>>,
    excluded_objects: GCell<Vec<String>>,
    current_object: GCell<Option<String>>,
    in_excluded_region: GCell<bool>,
    next_transform: GCell<Option<MoveTransform>>,
    last_position: GCell<Vec<f64>>,
    last_position_extruded: GCell<Vec<f64>>,
    last_position_excluded: GCell<Vec<f64>>,
    extrusion_offsets: GCell<Option<IndexMap<String, Vec<f64>>>>,
    max_position_extruded: GCell<f64>,
    max_position_excluded: GCell<f64>,
    extruder_adj: GCell<f64>,
    initial_extrusion_moves: GCell<i64>,
    last_speed: GCell<f64>,
    was_excluded_at_start: GCell<bool>,
}

impl ExcludeObject {
    /// `ExcludeObject.__init__(config)` (`exclude_object.py:12`).
    pub fn create(config: &Config) -> KResult<Arc<ExcludeObject>> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let gcode_move = load_interface::<dyn GCodeMoveApi>(
            &*printer,
            config,
            "gcode_move",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let this = Arc::new(ExcludeObject {
            printer,
            gcode,
            gcode_move,
            toolhead: GCell::new(None),
            objects: GCell::new(vec![]),
            excluded_objects: GCell::new(vec![]),
            current_object: GCell::new(None),
            in_excluded_region: GCell::new(false),
            next_transform: GCell::new(None),
            last_position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            last_position_extruded: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            last_position_excluded: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            extrusion_offsets: GCell::new(None),
            max_position_extruded: GCell::new(0.0),
            max_position_excluded: GCell::new(0.0),
            extruder_adj: GCell::new(0.0),
            initial_extrusion_moves: GCell::new(0),
            last_speed: GCell::new(0.0),
            was_excluded_at_start: GCell::new(false),
        });
        this.reset_state();
        this.register_handlers()?;
        Ok(this)
    }

    fn register_handlers(self: &Arc<Self>) -> KResult<()> {
        {
            let cb = self.clone();
            self.printer.register_event_handler(
                "klippy:connect",
                Box::new(move |_| cb.handle_connect().map(|_| PyValue::None)),
            );
        }
        {
            let cb = self.clone();
            self.printer.register_event_handler(
                "virtual_sdcard:reset_file",
                Box::new(move |_| cb.reset_file().map(|_| PyValue::None)),
            );
        }
        let opts = |desc: &str| RegisterCommandOpts {
            when_not_ready: false,
            desc: Some(desc.to_string()),
        };
        {
            let cb = self.clone();
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd| cb.cmd_exclude_object_start(gcmd));
            self.gcode
                .register_command("EXCLUDE_OBJECT_START", Some(handler), opts(HELP_START))?;
        }
        {
            let cb = self.clone();
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd| cb.cmd_exclude_object_end(gcmd));
            self.gcode
                .register_command("EXCLUDE_OBJECT_END", Some(handler), opts(HELP_END))?;
        }
        {
            let cb = self.clone();
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd| cb.cmd_exclude_object(gcmd));
            self.gcode
                .register_command("EXCLUDE_OBJECT", Some(handler), opts(HELP_EXCLUDE_OBJECT))?;
        }
        {
            let cb = self.clone();
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd| cb.cmd_exclude_object_define(gcmd));
            self.gcode.register_command(
                "EXCLUDE_OBJECT_DEFINE",
                Some(handler),
                opts(HELP_EXCLUDE_OBJECT_DEFINE),
            )?;
        }
        Ok(())
    }


    fn reset_state(&self) {
        *self.objects.borrow_mut() = Vec::new();
        *self.excluded_objects.borrow_mut() = Vec::new();
        *self.current_object.borrow_mut() = None;
        self.in_excluded_region.set(false);
    }

    /// `_register_transform` (`exclude_object.py:38`).
    fn register_transform(self: &Arc<Self>) -> KResult<()> {
        if self.next_transform.borrow().is_some() {
            return Ok(());
        }
        if self.tuning_is_active()? {
            log::info(
                "The ExcludeObject move transform is not being loaded due to Tuning tower being Active",
            );
            return Ok(());
        }
        let handle: MoveTransformHandle = self.clone();
        let old = self
            .gcode_move
            .set_move_transform(MoveTransform::Custom(handle), true)?;
        *self.next_transform.borrow_mut() = old;
        *self.extrusion_offsets.borrow_mut() = Some(IndexMap::new());
        self.max_position_extruded.set(0.0);
        self.max_position_excluded.set(0.0);
        self.extruder_adj.set(0.0);
        self.initial_extrusion_moves.set(5);
        *self.last_position.borrow_mut() = vec![0.0, 0.0, 0.0, 0.0];
        self.get_position()?;
        let lp = self.last_position.cloned();
        *self.last_position_extruded.borrow_mut() = lp.clone();
        *self.last_position_excluded.borrow_mut() = lp;
        Ok(())
    }

    /// `_handle_connect` (`exclude_object.py:59`).
    pub fn handle_connect(&self) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?;
        *self.toolhead.borrow_mut() = toolhead;
        Ok(())
    }

    /// `_unregister_transform` (`exclude_object.py:62`).
    pub fn unregister_transform(&self) -> KResult<()> {
        let next = match self.next_transform.borrow().clone() {
            Some(next) => next,
            None => return Ok(()),
        };
        if self.tuning_is_active()? {
            log::error(
                "The Exclude Object move transform was not unregistered because it is not at the head of the transform chain.",
            );
            return Ok(());
        }
        self.gcode_move.set_move_transform(next, true)?;
        *self.next_transform.borrow_mut() = None;
        self.gcode_move.reset_last_position();
        Ok(())
    }

    /// `_reset_file` (`exclude_object.py:81`).
    pub fn reset_file(&self) -> KResult<()> {
        self.reset_state();
        self.unregister_transform()
    }

    /// `_get_extrusion_offsets` (`exclude_object.py:85`): returns the
    /// `(extruder name, offset)` pair; callers mutate the copy and store it
    /// back with [`ExcludeObject::store_extrusion_offset`].
    fn extrusion_offset(&self, num_coord: usize) -> KResult<(String, Vec<f64>)> {
        let toolhead = self
            .toolhead
            .borrow()
            .clone()
            .ok_or_else(|| attr_error("toolhead"))?;
        if self.extrusion_offsets.borrow().is_none() {
            return Err(attr_error("extrusion_offsets"));
        }
        let ename = toolhead.get_extruder().get_name();
        let mut offsets = self.extrusion_offsets.borrow_mut();
        let offsets = offsets.as_mut().expect("checked above");
        let offset = offsets
            .entry(ename.clone())
            .or_insert_with(|| vec![0.0; num_coord]);
        if offset.len() < num_coord {
            offset.extend(std::iter::repeat_n(0.0, num_coord - offset.len()));
        }
        Ok((ename, offset.clone()))
    }

    fn store_extrusion_offset(&self, ename: &str, offset: Vec<f64>) {
        if let Some(offsets) = self.extrusion_offsets.borrow_mut().as_mut() {
            offsets.insert(ename.to_string(), offset);
        }
    }

    /// `get_position` (`exclude_object.py:95`).
    pub fn get_position(&self) -> KResult<Vec<f64>> {
        let next = self
            .next_transform
            .borrow()
            .clone()
            .ok_or_else(|| none_attr_error("get_position"))?;
        let pos = match next {
            MoveTransform::ToolHead(th) => th.get_position(),
            MoveTransform::Custom(t) => t.get_position()?,
        };
        let (ename, offset) = self.extrusion_offset(pos.len())?;
        {
            let mut last = self.last_position.borrow_mut();
            if pos.len() > last.len() {
                return Err(index_error());
            }
            for i in 0..pos.len() {
                last[i] = pos[i] + offset[i];
            }
        }
        self.store_extrusion_offset(&ename, offset);
        Ok(self.last_position.cloned())
    }

    fn tuning_is_active(&self) -> KResult<bool> {
        let tower = lookup_interface::<dyn TuningTowerApi>(
            &*self.printer,
            "tuning_tower",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        Ok(tower.is_active())
    }

    /// `_normal_move` (`exclude_object.py:102`).
    fn normal_move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        if newpos.len() < 4 {
            return Err(index_error());
        }
        let (ename, mut offset) = self.extrusion_offset(newpos.len())?;
        if self.initial_extrusion_moves.get() > 0 && self.last_position.borrow()[3] != newpos[3] {
            let n = self.initial_extrusion_moves.get() - 1;
            self.initial_extrusion_moves.set(n);
        }
        *self.last_position.borrow_mut() = newpos.clone();
        *self.last_position_extruded.borrow_mut() = self.last_position.cloned();
        let m = py_max(self.max_position_extruded.get(), newpos[3]);
        self.max_position_extruded.set(m);
        let excluded = self.last_position_excluded.cloned();
        if (offset[0] != 0.0 || offset[1] != 0.0)
            && (newpos[0] != excluded[0] || newpos[1] != excluded[1])
        {
            for (i, slot) in offset.iter_mut().enumerate().take(newpos.len()) {
                if i != 3 {
                    *slot = 0.0;
                }
            }
            offset[3] += self.extruder_adj.get();
            self.extruder_adj.set(0.0);
        }
        if offset[2] != 0.0 && newpos[2] != excluded[2] {
            offset[2] = 0.0;
        }
        if self.extruder_adj.get() != 0.0 && newpos[3] != excluded[3] {
            offset[3] += self.extruder_adj.get();
            self.extruder_adj.set(0.0);
        }
        let mut tx_pos = newpos.clone();
        for i in 0..newpos.len() {
            tx_pos[i] = newpos[i] - offset[i];
        }
        self.store_extrusion_offset(&ename, offset);
        self.forward_move(tx_pos, speed)
    }

    /// `_ignore_move` (`exclude_object.py:145`).
    fn ignore_move(&self, newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        if newpos.len() < 4
            || newpos.len() > self.last_position_extruded.borrow().len()
            || newpos.len() > self.last_position.borrow().len()
        {
            return Err(index_error());
        }
        let (ename, mut offset) = self.extrusion_offset(newpos.len())?;
        let extruded = self.last_position_extruded.cloned();
        for i in 0..newpos.len() {
            if i != 3 {
                offset[i] = newpos[i] - extruded[i];
            }
        }
        offset[3] += newpos[3] - self.last_position.borrow()[3];
        *self.last_position.borrow_mut() = newpos.clone();
        *self.last_position_excluded.borrow_mut() = self.last_position.cloned();
        let m = py_max(self.max_position_excluded.get(), newpos[3]);
        self.max_position_excluded.set(m);
        self.store_extrusion_offset(&ename, offset);
        Ok(())
    }

    /// `_move_into_excluded_region` (`exclude_object.py:155`).
    fn move_into_excluded_region(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.in_excluded_region.set(true);
        self.ignore_move(newpos, speed)
    }

    /// `_move_from_excluded_region` (`exclude_object.py:159`).
    fn move_from_excluded_region(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.in_excluded_region.set(false);
        let adj = self.max_position_excluded.get() - self.last_position_excluded.borrow()[3]
            - (self.max_position_extruded.get() - self.last_position_extruded.borrow()[3]);
        self.extruder_adj.set(adj);
        self.normal_move(newpos, speed)
    }

    fn forward_move(&self, tx_pos: Vec<f64>, speed: f64) -> KResult<()> {
        match self.next_transform.borrow().clone() {
            Some(MoveTransform::ToolHead(th)) => th.r#move(tx_pos, speed),
            Some(MoveTransform::Custom(t)) => t.r#move(tx_pos, speed),
            None => Err(none_attr_error("move")),
        }
    }

    /// `_test_in_excluded_region` (`exclude_object.py:169`).
    pub fn test_in_excluded_region(&self) -> bool {
        let current = match self.current_object.borrow().clone() {
            Some(c) => c,
            None => return false,
        };
        self.excluded_objects.borrow().contains(&current)
            && self.initial_extrusion_moves.get() == 0
    }

    pub fn get_last_speed(&self) -> f64 {
        self.last_speed.get()
    }

    pub fn was_excluded_at_start(&self) -> bool {
        self.was_excluded_at_start.get()
    }

    /// `cmd_EXCLUDE_OBJECT_START` (`exclude_object.py:199`).
    pub fn cmd_exclude_object_start(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let name = pyvalue_str(&get_required(gcmd, "NAME")?)
            .unwrap_or_default()
            .to_uppercase();
        if !self
            .objects
            .borrow()
            .iter()
            .any(|o| object_name(o).map(|n| n == name).unwrap_or(false))
        {
            self.add_object_definition(dict_name_dict(name.clone()));
        }
        *self.current_object.borrow_mut() = Some(name);
        let was = self.test_in_excluded_region();
        self.was_excluded_at_start.set(was);
        Ok(())
    }

    /// `cmd_EXCLUDE_OBJECT_END` (`exclude_object.py:207`).
    pub fn cmd_exclude_object_end(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        if self.current_object.borrow().is_none() && self.next_transform.borrow().is_some() {
            gcmd.respond_info(
                "EXCLUDE_OBJECT_END called, but no object is currently active",
                true,
            );
            return Ok(());
        }
        let name = pyvalue_str(&get_opt(gcmd, "NAME")?);
        if let Some(n) = &name {
            let current = self.current_object.borrow().clone();
            if Some(n.to_uppercase()) != current {
                let cur_text = current.unwrap_or_else(|| "None".to_string());
                gcmd.respond_info(
                    &format!(
                        "EXCLUDE_OBJECT_END NAME={} does not match the current object NAME={}",
                        n.to_uppercase(),
                        cur_text
                    ),
                    true,
                );
            }
        }
        *self.current_object.borrow_mut() = None;
        Ok(())
    }

    /// `cmd_EXCLUDE_OBJECT` (`exclude_object.py:221`).
    pub fn cmd_exclude_object(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let reset = get_opt(gcmd, "RESET")?;
        let current = get_opt(gcmd, "CURRENT")?;
        let name = pyvalue_str(&get_opt(gcmd, "NAME")?)
            .unwrap_or_default()
            .to_uppercase();
        if reset.truthy() {
            if !name.is_empty() {
                self.unexclude_object(&name);
            } else {
                *self.excluded_objects.borrow_mut() = Vec::new();
            }
        } else if !name.is_empty() {
            if !self.excluded_objects.borrow().contains(&name) {
                self.exclude_object(&name)?;
            }
        } else if current.truthy() {
            match self.current_object.borrow().clone() {
                Some(cur) if !cur.is_empty() => {
                    self.exclude_object(&cur)?;
                }
                _ => {
                    return Err(KlippyError::Command(
                        "There is no current object to cancel".to_string(),
                    ));
                }
            }
        } else {
            self.list_excluded_objects(gcmd);
        }
        Ok(())
    }

    /// `cmd_EXCLUDE_OBJECT_DEFINE` (`exclude_object.py:248`).
    pub fn cmd_exclude_object_define(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let reset = get_opt(gcmd, "RESET")?;
        let name = pyvalue_str(&get_opt(gcmd, "NAME")?)
            .unwrap_or_default()
            .to_uppercase();
        if reset.truthy() {
            self.reset_file()?;
        } else if !name.is_empty() {
            let mut parameters = gcmd.get_command_parameters();
            parameters.shift_remove("NAME");
            let center = parameters.shift_remove("CENTER");
            let polygon = parameters.shift_remove("POLYGON");
            let mut obj = match dict_name_dict(name) {
                PyValue::Dict(d) => d,
                _ => unreachable!(),
            };
            for (k, v) in parameters {
                obj.insert(PyValue::Str(k), PyValue::Str(v));
            }
            if let Some(center) = center {
                let text = format!("[{center}]");
                let value = pyjson::loads(&text).map_err(json_decode_error)?;
                obj.insert(PyValue::Str("center".to_string()), value);
            }
            if let Some(polygon) = polygon {
                let value = pyjson::loads(&polygon).map_err(json_decode_error)?;
                obj.insert(PyValue::Str("polygon".to_string()), value);
            }
            self.add_object_definition(PyValue::Dict(obj));
        } else {
            self.list_objects(gcmd)?;
        }
        Ok(())
    }

    fn add_object_definition(&self, definition: PyValue) {
        let mut objs = self.objects.borrow_mut();
        objs.push(definition);
        objs.sort_by(|a, b| {
            let ka = dict_get(a, "name").map(PyValue::str).unwrap_or_default();
            let kb = dict_get(b, "name").map(PyValue::str).unwrap_or_default();
            // Stable: equal keys keep insertion order, as Python `sorted`.
            ka.cmp(&kb)
        });
    }

    fn exclude_object(self: &Arc<Self>, name: &str) -> KResult<()> {
        // `self._register_transform()` may need to install `self` as the
        // transform, so it takes the `Arc`.
        Self::register_transform(self)?;
        self.gcode
            .respond_info(&format!("Excluding object {}", name.to_uppercase()), true);
        let mut excluded = self.excluded_objects.borrow_mut();
        if !excluded.contains(&name.to_string()) {
            excluded.push(name.to_string());
            excluded.sort();
        }
        Ok(())
    }


    fn unexclude_object(&self, name: &str) {
        self.gcode
            .respond_info(&format!("Unexcluding object {}", name.to_uppercase()), true);
        let mut excluded = self.excluded_objects.borrow_mut();
        if let Some(pos) = excluded.iter().position(|x| x == name) {
            excluded.remove(pos);
            excluded.sort();
        }
    }

    fn list_objects(&self, gcmd: &GCmd) -> KResult<()> {
        // `gcmd.get('JSON', None) is not None` → presence, not truthiness.
        let json_present = matches!(get_opt(gcmd, "JSON")?, PyValue::Str(_));
        let object_list = if json_present {
            pyjson::dumps(&PyValue::List(self.objects.borrow().clone())).map_err(|e| {
                KlippyError::Internal {
                    class_name: e.kind.to_string(),
                    message: e.msg,
                }
            })?
        } else {
            let names: Vec<String> = self
                .objects
                .borrow()
                .iter()
                .map(object_name)
                .collect::<KResult<Vec<String>>>()?;
            names.join(" ")
        };
        gcmd.respond_info(&format!("Known objects: {object_list}"), true);
        Ok(())
    }

    fn list_excluded_objects(&self, gcmd: &GCmd) {
        let object_list = self.excluded_objects.borrow().join(" ");
        gcmd.respond_info(&format!("Excluded objects: {object_list}"), true);
    }
}

impl MoveTransformApi for ExcludeObject {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let in_excluded = self.test_in_excluded_region();
        self.last_speed.set(speed);
        if in_excluded {
            if self.in_excluded_region.get() {
                self.ignore_move(newpos, speed)
            } else {
                self.move_into_excluded_region(newpos, speed)
            }
        } else if self.in_excluded_region.get() {
            self.move_from_excluded_region(newpos, speed)
        } else {
            self.normal_move(newpos, speed)
        }
    }

    fn get_position(&self) -> KResult<Vec<f64>> {
        self.get_position()
    }
}

impl PrinterObject for ExcludeObject {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn MoveTransformApi);

    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("objects".to_string()),
            PyValue::List(self.objects.borrow().clone()),
        );
        map.insert(
            PyValue::Str("excluded_objects".to_string()),
            PyValue::List(
                self.excluded_objects
                    .borrow()
                    .iter()
                    .map(|s| PyValue::Str(s.clone()))
                    .collect(),
            ),
        );
        map.insert(
            PyValue::Str("current_object".to_string()),
            match self.current_object.borrow().clone() {
                Some(c) => PyValue::Str(c),
                None => PyValue::None,
            },
        );
        Some(PyValue::Dict(map))
    }
}

/// `load_config(config)` (`exclude_object.py:303`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(ExcludeObject::create(config)? as ObjectRef))
}

/// Module entry point for the `exclude_object` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
