//! Real-time Z compensation from a temperature sensor.
//!
//! The move transform retains its last offset above the configured Z threshold
//! and on Z-only moves. Disabling it also retains that offset; position queries
//! still recalculate compensation, matching the reference transform chain.
//! Z homing resets the reference temperature and current offset.

use std::sync::Arc;

use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeMoveApi, GetFloatOpts as CommandFloatOpts,
    GetIntOpts as CommandIntOpts, MoveTransform, MoveTransformApi, RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{HomingStateApi, ToolHeadApi};
use klippy_api::{load_interface, lookup_interface, GCell, KResult, KlippyError, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;

use crate::heaters::{PrinterHeatersApi, SensorApi, TemperatureSensorApi};

pub const KELVIN_TO_CELSIUS: f64 = -273.15;
pub const CMD_SET_Z_THERMAL_ADJUST_HELP: &str = "Set/query Z Thermal Adjust parameters.";

#[derive(Default)]
struct TemperatureState {
    last_temp: f64,
    measured_min: f64,
    measured_max: f64,
    smoothed_temp: f64,
    last_temp_time: f64,
}

struct AdjustmentState {
    temp_coeff: f64,
    off_above_z: f64,
    max_z_adjust_mm: f64,
    inv_smooth_time: f64,
    ref_temperature: f64,
    ref_temp_override: bool,
    z_adjust_mm: f64,
    last_z_adjust_mm: f64,
    adjust_enable: bool,
    last_position: Vec<f64>,
    next_transform: Option<MoveTransform>,
    z_step_dist: Option<f64>,
}

pub struct ZThermalAdjuster {
    printer: Printer,
    temperature: kos::sync::Mutex<TemperatureState>,
    adjustment: GCell<AdjustmentState>,
    sensor: GCell<Option<Arc<dyn TemperatureSensorApi>>>,
}

fn internal(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal { class_name: class_name.into(), message: message.into() }
}

impl ZThermalAdjuster {
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?.unwrap();
        let temp_coeff = config.getfloat("temp_coeff", GetFloatOpts {
            default: Some(Some(0.)), minval: Some(-1.), maxval: Some(1.), ..Default::default()
        })?.unwrap();
        let off_above_z = config.getfloat("z_adjust_off_above", GetFloatOpts {
            default: Some(Some(99999999.)), ..Default::default()
        })?.unwrap();
        let max_z_adjust_mm = config.getfloat("max_z_adjustment", GetFloatOpts {
            default: Some(Some(99999999.)), ..Default::default()
        })?.unwrap();
        let this = Arc::new(Self {
            printer: printer.clone(),
            temperature: kos::sync::Mutex::new(TemperatureState::default()),
            adjustment: GCell::new(AdjustmentState {
                temp_coeff, off_above_z, max_z_adjust_mm, inv_smooth_time: 0.,
                ref_temperature: 0., ref_temp_override: false,
                z_adjust_mm: 0., last_z_adjust_mm: 0., adjust_enable: true,
                last_position: vec![0.; 4], next_transform: None, z_step_dist: None,
            }),
            sensor: GCell::new(None),
        });
        let handler = this.clone();
        printer.register_event_handler("klippy:connect", Box::new(move |_| {
            handler.handle_connect()?;
            Ok(PyValue::None)
        }));
        let handler = this.clone();
        printer.register_event_handler("homing:home_rails_end", Box::new(move |args| {
            let homing = args.first().and_then(|a| a.as_handle::<dyn HomingStateApi>())
                .ok_or_else(|| internal("AttributeError", "homing state has no attribute 'get_axes'"))?;
            handler.handle_homing_move_end(&*homing);
            Ok(PyValue::None)
        }));
        let smooth_time = config.getfloat("smooth_time", GetFloatOpts {
            default: Some(Some(2.)), above: Some(0.), ..Default::default()
        })?.unwrap();
        this.adjustment.borrow_mut().inv_smooth_time = 1. / smooth_time;
        let min_temp = config.getfloat("min_temp", GetFloatOpts {
            minval: Some(KELVIN_TO_CELSIUS), ..Default::default()
        })?.unwrap();
        let max_temp = config.getfloat("max_temp", GetFloatOpts {
            above: Some(min_temp), ..Default::default()
        })?.unwrap();
        let heaters = load_interface::<dyn PrinterHeatersApi>(
            &*printer, config, "heaters", LookupDefault::Raise,
        )?.unwrap();
        let sensor = heaters.setup_sensor(config)?;
        sensor.setup_minmax(min_temp, max_temp)?;
        let handler = this.clone();
        sensor.setup_callback(Box::new(move |time, temp| handler.temperature_callback(time, temp)));
        this.sensor.replace(Some(sensor));
        heaters.register_sensor(config, this.clone(), None)?;
        let handler = this.clone();
        gcode.register_command("SET_Z_THERMAL_ADJUST", Some(Box::new(move |gcmd| {
            handler.cmd_set_z_thermal_adjust(gcmd)
        })), RegisterCommandOpts {
            desc: Some(CMD_SET_Z_THERMAL_ADJUST_HELP.into()), ..Default::default()
        })?;
        Ok(this)
    }

    pub fn handle_connect(self: &Arc<Self>) -> KResult<()> {
        // The first lookup precedes installing the transform in the reference.
        lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?;
        let gcode_move = lookup_interface::<dyn GCodeMoveApi>(
            &*self.printer, "gcode_move", LookupDefault::Raise,
        )?.unwrap();
        let next = gcode_move.set_move_transform(MoveTransform::Custom(self.clone()), true)?;
        self.adjustment.borrow_mut().next_transform = next;
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer, "toolhead", LookupDefault::Raise,
        )?.unwrap();
        let kin = toolhead.get_kinematics();
        let mut index = None;
        for (i, stepper) in kin.get_steppers().iter().enumerate() {
            let name = stepper.get_name(None);
            if name == "stepper_z" && index.is_none() { index = Some(i); }
        }
        // Python evaluates `kin.get_steppers()` before `steppers.index(...)` raises.
        let steppers = kin.get_steppers();
        let index = index.ok_or_else(|| internal("ValueError", "'stepper_z' is not in list"))?;
        let stepper = steppers.get(index).ok_or_else(|| internal("IndexError", "list index out of range"))?;
        self.adjustment.borrow_mut().z_step_dist = Some(stepper.get_step_dist());
        Ok(())
    }

    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let t = self.temperature.lock();
        let s = self.adjustment.borrow();
        PyValue::Dict([
            ("temperature", PyValue::Float(t.smoothed_temp)),
            ("measured_min_temp", PyValue::Float(round_py_nd(t.measured_min, 2))),
            ("measured_max_temp", PyValue::Float(round_py_nd(t.measured_max, 2))),
            ("current_z_adjust", PyValue::Float(s.z_adjust_mm)),
            ("z_adjust_ref_temperature", PyValue::Float(s.ref_temperature)),
            ("enabled", PyValue::Bool(s.adjust_enable)),
        ].into_iter().map(|(k, v)| (PyValue::Str(k.into()), v)).collect())
    }

    pub fn handle_homing_move_end(&self, homing_state: &dyn HomingStateApi) {
        if homing_state.get_axes().contains(&2) {
            let temp = self.temperature.lock().smoothed_temp;
            let mut s = self.adjustment.borrow_mut();
            s.ref_temperature = temp;
            s.ref_temp_override = false;
            s.z_adjust_mm = 0.;
        }
    }

    pub fn calc_adjust(&self, mut pos: Vec<f64>) -> KResult<Vec<f64>> {
        let z = *pos.get(2).ok_or_else(|| internal("IndexError", "list index out of range"))?;
        let temp = self.temperature.lock().smoothed_temp;
        let mut s = self.adjustment.borrow_mut();
        if z < s.off_above_z {
            let delta_t = temp - s.ref_temperature;
            // Python multiplies by -1 (sign of a NaN result follows the multiply, not negation).
            #[allow(clippy::neg_multiply)]
            let adjust = -1. * s.temp_coeff * delta_t;
            let sign = if adjust <= 0. { -1. } else { 1. };
            let step_dist = s.z_step_dist.ok_or_else(|| internal(
                "AttributeError", "'ZThermalAdjuster' object has no attribute 'z_step_dist'",
            ))?;
            if (adjust - s.z_adjust_mm).abs() > step_dist {
                let limit = s.max_z_adjust_mm * sign;
                // Python min(key=abs) retains its first operand on ties and NaNs.
                s.z_adjust_mm = if adjust.abs() < limit.abs() { adjust } else { limit };
            }
        }
        pos[2] = z + s.z_adjust_mm;
        s.last_z_adjust_mm = s.z_adjust_mm;
        Ok(pos)
    }

    pub fn calc_unadjust(&self, mut pos: Vec<f64>) -> KResult<Vec<f64>> {
        let z = pos.get_mut(2).ok_or_else(|| internal("IndexError", "list index out of range"))?;
        *z -= self.adjustment.borrow().z_adjust_mm;
        Ok(pos)
    }

    fn next_transform(&self, method: &str) -> KResult<MoveTransform> {
        self.adjustment.borrow().next_transform.clone().ok_or_else(|| internal(
            "AttributeError", &format!("'NoneType' object has no attribute '{}'", method),
        ))
    }

    pub fn get_position(&self) -> KResult<Vec<f64>> {
        let pos = match self.next_transform("get_position")? {
            MoveTransform::ToolHead(t) => t.get_position(),
            MoveTransform::Custom(t) => t.get_position()?,
        };
        let pos = self.calc_unadjust(pos)?;
        let last = self.calc_adjust(pos.clone())?;
        self.adjustment.borrow_mut().last_position = last;
        Ok(pos)
    }

    pub fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let unchanged_xy = {
            let s = self.adjustment.borrow();
            newpos[..newpos.len().min(2)] == s.last_position[..s.last_position.len().min(2)]
                || !s.adjust_enable
        };
        let adjusted = if unchanged_xy {
            if newpos.len() < 4 { return Err(internal("IndexError", "list index out of range")); }
            vec![newpos[0], newpos[1], newpos[2] + self.adjustment.borrow().last_z_adjust_mm, newpos[3]]
        } else {
            self.calc_adjust(newpos.clone())?
        };
        match self.next_transform("move")? {
            MoveTransform::ToolHead(t) => t.r#move(adjusted, speed)?,
            MoveTransform::Custom(t) => t.r#move(adjusted, speed)?,
        }
        self.adjustment.borrow_mut().last_position = newpos;
        Ok(())
    }

    pub fn temperature_callback(&self, read_time: f64, temp: f64) {
        let mut t = self.temperature.lock();
        let time_diff = read_time - t.last_temp_time;
        t.last_temp = temp;
        t.last_temp_time = read_time;
        let temp_diff = temp - t.smoothed_temp;
        let scaled = time_diff * self.adjustment.borrow().inv_smooth_time;
        let adj_time = if 1. < scaled { 1. } else { scaled };
        t.smoothed_temp += temp_diff * adj_time;
        if t.smoothed_temp < t.measured_min { t.measured_min = t.smoothed_temp; }
        if t.smoothed_temp > t.measured_max { t.measured_max = t.smoothed_temp; }
    }

    pub fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        (self.temperature.lock().smoothed_temp, 0.)
    }

    pub fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, percent_format("%s: temp=%.1f", &[
            PctValue::Str("z_thermal_adjust".into()),
            PctValue::Float(self.temperature.lock().smoothed_temp),
        ]).expect("static temperature format"))
    }

    pub fn cmd_set_z_thermal_adjust(&self, gcmd: &GCmd) -> KResult<()> {
        let enable = gcmd.get_int("ENABLE", CommandIntOpts {
            default: Some(None), minval: Some(0), maxval: Some(1),
        })?;
        let coeff = gcmd.get_float("TEMP_COEFF", CommandFloatOpts {
            default: Some(None), minval: Some(-1.), maxval: Some(1.), ..Default::default()
        })?;
        let ref_temp = gcmd.get_float("REF_TEMP", CommandFloatOpts {
            default: Some(None), minval: Some(KELVIN_TO_CELSIUS), ..Default::default()
        })?;
        let reset = {
            let mut s = self.adjustment.borrow_mut();
            if let Some(temp) = ref_temp { s.ref_temperature = temp; s.ref_temp_override = true; }
            if let Some(coeff) = coeff { s.temp_coeff = coeff; }
            if let Some(enable) = enable {
                let enable = enable != 0;
                let changed = enable != s.adjust_enable;
                if changed { s.adjust_enable = enable; }
                changed
            } else { false }
        };
        if reset {
            lookup_interface::<dyn GCodeMoveApi>(&*self.printer, "gcode_move", LookupDefault::Raise)?
                .unwrap().reset_last_position()?;
        }
        let msg = {
            let s = self.adjustment.borrow();
            percent_format(concat!(
                "enable: %s\ntemp_coeff: %f mm/degC\nref_temp: %.2f degC%s\n",
                "-------------------\nCurrent Z temp: %.2f degC\nApplied Z adjustment: %.4f mm"
            ), &[
                PctValue::Str(if s.adjust_enable { "1 (enabled)" } else { "0 (disabled)" }.into()),
                PctValue::Float(s.temp_coeff), PctValue::Float(s.ref_temperature),
                PctValue::Str(if s.ref_temp_override { " (manual)" } else { "" }.into()),
                PctValue::Float(self.temperature.lock().smoothed_temp), PctValue::Float(s.z_adjust_mm),
            ]).map_err(|e| internal(e.kind, &e.msg))?
        };
        gcmd.respond_info(&msg, true);
        Ok(())
    }
}

impl SensorApi for ZThermalAdjuster {
    fn get_temp(&self, eventtime: f64) -> (f64, f64) { self.get_temp(eventtime) }
}
impl MoveTransformApi for ZThermalAdjuster {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> { self.r#move(newpos, speed) }
    fn get_position(&self) -> KResult<Vec<f64>> { self.get_position() }
}
impl PrinterObject for ZThermalAdjuster {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn SensorApi, dyn MoveTransformApi);
    fn get_status(&self, eventtime: f64) -> Option<PyValue> { Some(self.get_status(eventtime)) }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> { Some(self.stats(eventtime)) }
}

pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(ZThermalAdjuster::new(config)?))
}
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: Some(load_config), ..Default::default() }
}
