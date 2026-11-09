//! Port of the `hall_filament_width_sensor` module
//! (`klippy/extras/hall_filament_width_sensor.py`).
//!
//! The sensor reads two ADC pins and calibrates a filament diameter
//! from them, queues readings by extruder position, reports runout
//! through a `filament_switch_sensor.RunoutHelper`, and drives the
//! `M221` flow factor from the queued readings.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeApi, GCodeCommandHandler, GCmd, RegisterCommandOpts};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AdcPin, AdcSampleOpts, Pin, PinsApi};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, Printer, EV_KLIPPY_READY};
use klippy_api::toolhead::ToolHead;
use klippy_api::{lookup_interface, PrinterObject};
use pycompat::int::PyInt;
use pycompat::libm::pow as py_pow;
use pycompat::text::{round_py, round_py_nd, PctValue, percent_format};
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::Arc;
/// `ADC_REPORT_TIME: float = 0.500` (`klippy/extras/hall_filament_width_sensor.py:8`).
pub const ADC_REPORT_TIME: f64 = 0.500;
/// `ADC_SAMPLE_TIME: float = 0.03` (`klippy/extras/hall_filament_width_sensor.py:9`).
pub const ADC_SAMPLE_TIME: f64 = 0.03;
/// `ADC_SAMPLE_COUNT: int = 15` (`klippy/extras/hall_filament_width_sensor.py:10`).
pub const ADC_SAMPLE_COUNT: i64 = 15;

// `RunoutHelper` is owned by `filament_switch_sensor`
// (`klippy/extras/filament_switch_sensor.py:9`) and used through its
// concrete type; it is a helper, not a printer object, so it is
// constructed from the config like the reference (`:100`).
use crate::filament_switch_sensor::RunoutHelper;


/// `klippy/extras/hall_filament_width_sensor.py:12 HallFilamentWidthSensor`.
///
/// Python stores plain attributes here; the mutable ones live in
/// `GCell`s like every printer object.
#[allow(dead_code)]
pub struct HallFilamentWidthSensor {
    printer: Printer,
    reactor: Reactor,
    pin1: String,
    pin2: String,
    dia1: f64,
    dia2: f64,
    rawdia1: i64,
    rawdia2: i64,
    measurement_interval_mm: i64,
    nominal_filament_dia: f64,
    measurement_delay: f64,
    measurement_max_difference: f64,
    max_diameter: f64,
    min_diameter: f64,
    diameter: GCell<f64>,
    is_active: GCell<bool>,
    runout_dia_min: f64,
    runout_dia_max: f64,
    is_log: GCell<bool>,
    enable_flow_compensation: GCell<bool>,
    use_current_dia_while_delay: bool,
    sensor_disable_pending: GCell<bool>,
    flow_comp_disable_pending: GCell<bool>,
    array_clear_pending: GCell<bool>,
    filament_array: GCell<Vec<(f64, f64)>>,
    last_filament_width_reading: GCell<i64>,
    last_filament_width_reading2: GCell<i64>,
    first_extruder_update_position: GCell<f64>,
    filament_width: GCell<f64>,
    toolhead: GCell<Option<ToolHead>>,
    ppins: GCell<Option<Arc<dyn PinsApi>>>,
    mcu_adc: GCell<Option<AdcPin>>,
    mcu_adc2: GCell<Option<AdcPin>>,
    extrude_factor_update_timer: GCell<Option<Timer>>,
    gcode: GCell<Option<GCode>>,
    runout_helper: GCell<Option<Arc<RunoutHelper>>>,
}

/// `round(read_value * 10000)` (`hall_filament_width_sensor.py:114`):
/// Python `round` returns an arbitrary-precision int; ADC values are
/// within `[0, 1]` times `10000`, so the i64 conversion never
/// overflows in practice. Overflow saturates like a C cast would.
fn rounding_adc_reading(value: f64) -> i64 {
    let py = round_py(value).unwrap_or_else(|_| PyInt::from_i64(0));
    let text = py.inner().to_string();
    text.parse::<i64>()
        .unwrap_or(if text.starts_with('-') { i64::MIN } else { i64::MAX })
}

/// `gcmd.respond_info` format helper: Python `"%.3f" % value`.
fn fmt_percent_3f(value: f64) -> String {
    percent_format("%.3f", &[PctValue::Float(value)])
        .expect("percent format of %.3f with a float cannot fail")
}

impl HallFilamentWidthSensor {
    /// `HallFilamentWidthSensor.__init__(config)`
    /// (`klippy/extras/hall_filament_width_sensor.py:13`).
    ///
    /// The `RunoutHelper` is constructed by the caller (the
    /// `load_config` entry point and the tests) and attached with
    /// [`HallFilamentWidthSensor::set_runout_helper`] after this
    /// constructor returns, mirroring the Python order where the
    /// helper is built last (`:100`) after hall's own registrations
    /// (`:62`, `:64`, `:74`, `:77`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer: Printer = config.get_printer();
        let reactor = printer.get_reactor();
        let pin1 = config
            .get("adc1", GetStrOpts::default())?
            .expect("required config option returned no value");
        let pin2 = config
            .get("adc2", GetStrOpts::default())?
            .expect("required config option returned no value");
        let dia1 = config
            .getfloat(
                "Cal_dia1",
                GetFloatOpts {
                    default: Some(Some(1.5)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let dia2 = config
            .getfloat(
                "Cal_dia2",
                GetFloatOpts {
                    default: Some(Some(2.0)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let rawdia1 = config
            .getint(
                "Raw_dia1",
                GetIntOpts {
                    default: Some(Some(9500)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let rawdia2 = config
            .getint(
                "Raw_dia2",
                GetIntOpts {
                    default: Some(Some(10500)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let measurement_interval_mm = config
            .getint(
                "measurement_interval",
                GetIntOpts {
                    default: Some(Some(10)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let nominal_filament_dia = config
            .getfloat(
                "default_nominal_filament_diameter",
                GetFloatOpts {
                    above: Some(1.0),
                    ..Default::default()
                },
            )?
            .expect("required config option returned no value");
        let measurement_delay = config
            .getfloat(
                "measurement_delay",
                GetFloatOpts {
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("required config option returned no value");
        let measurement_max_difference = config
            .getfloat(
                "max_difference",
                GetFloatOpts {
                    default: Some(Some(0.2)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let max_diameter = nominal_filament_dia + measurement_max_difference;
        let min_diameter = nominal_filament_dia - measurement_max_difference;
        let diameter = nominal_filament_dia;
        let is_active = config
            .getboolean(
                "enable",
                GetBoolOpts {
                    default: Some(Some(false)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let runout_dia_min = config
            .getfloat(
                "min_diameter",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let runout_dia_max = config
            .getfloat(
                "max_diameter",
                GetFloatOpts {
                    default: Some(Some(max_diameter)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let is_log = config
            .getboolean(
                "logging",
                GetBoolOpts {
                    default: Some(Some(false)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let enable_flow_compensation = config
            .getboolean(
                "enable_flow_compensation",
                GetBoolOpts {
                    default: Some(Some(true)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        let use_current_dia_while_delay = config
            .getboolean(
                "use_current_dia_while_delay",
                GetBoolOpts {
                    default: Some(Some(false)),
                    ..Default::default()
                },
            )?
            .expect("defaulted config option returned no value");
        if rawdia1 == rawdia2 {
            return Err(KlippyError::config(
                "hall_filament_width_sensor: raw_dia1 and raw_dia2 must be \
                 different",
            ));
        }
        if min_diameter <= 0.0 {
            return Err(KlippyError::config(
                "hall_filament_width_sensor: max_difference must be less than \
                 default_nominal_filament_diameter",
            ));
        }

        let sensor = HallFilamentWidthSensor {
            printer,
            reactor,
            pin1,
            pin2,
            dia1,
            dia2,
            rawdia1,
            rawdia2,
            measurement_interval_mm,
            nominal_filament_dia,
            measurement_delay,
            measurement_max_difference,
            max_diameter,
            min_diameter,
            diameter: GCell::new(diameter),
            is_active: GCell::new(is_active),
            runout_dia_min,
            runout_dia_max,
            is_log: GCell::new(is_log),
            enable_flow_compensation: GCell::new(enable_flow_compensation),
            use_current_dia_while_delay,
            sensor_disable_pending: GCell::new(false),
            flow_comp_disable_pending: GCell::new(false),
            array_clear_pending: GCell::new(false),
            filament_array: GCell::new(Vec::new()),
            last_filament_width_reading: GCell::new(0),
            last_filament_width_reading2: GCell::new(0),
            first_extruder_update_position: GCell::new(0.0),
            filament_width: GCell::new(nominal_filament_dia),
            toolhead: GCell::new(None),
            ppins: GCell::new(None),
            mcu_adc: GCell::new(None),
            mcu_adc2: GCell::new(None),
            extrude_factor_update_timer: GCell::new(None),
            gcode: GCell::new(None),
            runout_helper: GCell::new(None),
        };
        let arc = Arc::new(sensor);
        arc.init()?;
        Ok(arc)
    }

    /// Registration and ADC setup, in the Python `__init__` order.
    fn init(self: &Arc<Self>) -> KResult<()> {
        // `self.printer.register_event_handler("klippy:ready", self.handle_ready)`
        let ready = self.clone();
        let ready_cb: EventCallback = Box::new(move |_args| {
            ready.handle_ready()?;
            Ok(PyValue::None)
        });
        self.printer
            .register_event_handler(EV_KLIPPY_READY, ready_cb);
        // `self.ppins = self.printer.lookup_object('pins')`
        let ppins = lookup_interface::<dyn PinsApi>(&*self.printer, "pins", LookupDefault::Raise)?
            .expect("Raise lookup never yields None");
        *self.ppins.borrow_mut() = Some(ppins.clone());
        // First ADC pin (pin1).
        let sensor1 = self.clone();
        let adc1 = match ppins.setup_pin("adc", &self.pin1)? {
            Pin::Adc(adc) => adc,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_digital_out' object has no attribute \
                              'setup_adc_sample'"
                        .to_string(),
                });
            }
        };
        adc1.setup_adc_sample(
            ADC_REPORT_TIME,
            AdcSampleOpts {
                sample_time: ADC_SAMPLE_TIME,
                sample_count: ADC_SAMPLE_COUNT,
                ..Default::default()
            },
        );
        adc1.setup_adc_callback(Box::new(move |samples| {
            sensor1.adc_callback(&samples);
        }));
        *self.mcu_adc.borrow_mut() = Some(adc1);
        // Second ADC pin (pin2).
        let sensor2 = self.clone();
        let adc2 = match ppins.setup_pin("adc", &self.pin2)? {
            Pin::Adc(adc) => adc,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_digital_out' object has no attribute \
                              'setup_adc_sample'"
                        .to_string(),
                });
            }
        };
        adc2.setup_adc_sample(
            ADC_REPORT_TIME,
            AdcSampleOpts {
                sample_time: ADC_SAMPLE_TIME,
                sample_count: ADC_SAMPLE_COUNT,
                ..Default::default()
            },
        );
        adc2.setup_adc_callback(Box::new(move |samples| {
            sensor2.adc2_callback(&samples);
        }));
        *self.mcu_adc2.borrow_mut() = Some(adc2);
        // `self.extrude_factor_update_timer = self.reactor.register_timer(...)`
        let timer_sensor = self.clone();
        let timer = self.reactor.register_timer(
            move |eventtime| timer_sensor.extrude_factor_update_event(eventtime)
                .unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message())),
            NEVER,
        );
        *self.extrude_factor_update_timer.borrow_mut() = Some(timer);
        // `self.gcode = self.printer.lookup_object('gcode')`
        let gcode = lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
            .expect("Raise lookup never yields None");
        *self.gcode.borrow_mut() = Some(gcode);
        // Command registration.
        self.register_commands()?;
        Ok(())
    }

    /// Registers the seven hall g-code commands (`:77-98`).
    fn register_commands(self: &Arc<Self>) -> KResult<()> {
        macro_rules! cmd {
            ($name:literal, $help:literal, $method:ident) => {{
                let sensor = self.clone();
                let handler: GCodeCommandHandler =
                    Box::new(move |gcmd| sensor.$method(gcmd));
                self.gcode_handle().register_command(
                    $name,
                    Some(handler),
                    RegisterCommandOpts {
                        when_not_ready: false,
                        desc: Some($help.to_string()),
                    },
                )?;
            }};
        }
        cmd!(
            "QUERY_FILAMENT_WIDTH",
            "Report the filament width in mm and show sensor status",
            cmd_m407
        );
        cmd!(
            "RESET_FILAMENT_WIDTH_SENSOR",
            "Clear all filament width readings",
            cmd_clear_filament_array
        );
        cmd!(
            "DISABLE_FILAMENT_WIDTH_SENSOR",
            "Disable the filament width sensor",
            cmd_m406
        );
        cmd!(
            "ENABLE_FILAMENT_WIDTH_SENSOR",
            "Enable the filament width sensor and enable or disable flow \
             compensation",
            cmd_m405
        );
        cmd!(
            "QUERY_RAW_FILAMENT_WIDTH",
            "Report the raw filament width sensor values",
            cmd_get_raw_values
        );
        cmd!(
            "ENABLE_FILAMENT_WIDTH_LOG",
            "Enable filament width sensor logging",
            cmd_log_enable
        );
        cmd!(
            "DISABLE_FILAMENT_WIDTH_LOG",
            "Disable filament width sensor logging",
            cmd_log_disable
        );
        Ok(())
    }

    /// Attaches the `RunoutHelper` built by the caller and applies
    /// `:101` (`self.runout_helper.sensor_enabled = self.is_active`).
    pub fn set_runout_helper(&self, helper: Arc<RunoutHelper>) {
        *self.runout_helper.borrow_mut() = Some(helper.clone());
        helper.set_sensor_enabled(self.is_active.get());
    }

    fn runout_helper(&self) -> Arc<RunoutHelper> {
        self.runout_helper
            .borrow()
            .clone()
            .expect("runout_helper is attached during load_config")
    }


    fn gcode_handle(&self) -> GCode {
        self.gcode
            .borrow()
            .clone()
            .expect("gcode is looked up in __init__")
    }

    /// `handle_ready` (`:103`).
    pub fn handle_ready(&self) -> KResult<()> {
        let toolhead = lookup_interface::<dyn klippy_api::toolhead::ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise lookup never yields None");
        *self.toolhead.borrow_mut() = Some(toolhead);
        let timer_cell = self.extrude_factor_update_timer.borrow();
        let timer = timer_cell
            .as_ref()
            .expect("extrude_factor_update_timer registered in __init__");
        self.reactor.update_timer(timer, NOW);
        Ok(())
    }

    /// `adc_callback` (`:111`).
    pub fn adc_callback(&self, samples: &[(f64, f64)]) {
        let (_read_time, read_value) = *samples.last().expect("list index out of range");
        self.last_filament_width_reading
            .set(rounding_adc_reading(read_value * 10000.0));
    }

    /// `adc2_callback` (`:116`).
    pub fn adc2_callback(&self, samples: &[(f64, f64)]) {
        let (_read_time, read_value) = *samples.last().expect("list index out of range");
        self.last_filament_width_reading2
            .set(rounding_adc_reading(read_value * 10000.0));
        if self.is_active.get() {
            let diameter_new = round_py_nd(
                (self.dia2 - self.dia1) / ((self.rawdia2 - self.rawdia1) as f64)
                    * (((self.last_filament_width_reading.get()
                        + self.last_filament_width_reading2.get())
                        - self.rawdia1) as f64)
                    + self.dia1,
                2,
            );
            let diameter = self.diameter.get();
            self.diameter.set((5.0 * diameter + diameter_new) / 6.0);
        } else {
            self.diameter.set(self.nominal_filament_dia);
        }
    }

    /// `update_filament_array` (`:130`).
    pub fn update_filament_array(&self, last_epos: f64) -> KResult<()> {
        let mut array = self.filament_array.borrow_mut();
        if !array.is_empty() {
            let next_reading_position = array.last().expect("non-empty").0
                + (self.measurement_interval_mm as f64);
            if next_reading_position <= (last_epos + self.measurement_delay) {
                array.push((last_epos + self.measurement_delay, self.diameter.get()));
                let diameter = self.diameter.get();
                let is_log = self.is_log.get();
                if is_log {
                    self.gcode_handle()
                        .respond_info(&format!("Filament width:{}", fmt_percent_3f(diameter)), true);
                }
            }
        } else {
            array.push((self.measurement_delay + last_epos, self.diameter.get()));
            *self.first_extruder_update_position.borrow_mut() =
                self.measurement_delay + last_epos;
        }
        Ok(())
    }

    /// `extrude_factor_update_event` (`:151`).
    pub fn extrude_factor_update_event(&self, eventtime: f64) -> KResult<f64> {
        let sensor_disable_pending = self.sensor_disable_pending.get();
        let flow_comp_disable_pending = self.flow_comp_disable_pending.get();
        let array_clear_pending = self.array_clear_pending.get();
        if sensor_disable_pending || flow_comp_disable_pending || array_clear_pending {
            self.gcode_handle().run_script("M221 S100")?;
        }
        if array_clear_pending {
            self.array_clear_pending.set(false);
            self.filament_array.borrow_mut().clear();
        }
        if flow_comp_disable_pending {
            self.flow_comp_disable_pending.set(false);
        }
        if sensor_disable_pending {
            self.sensor_disable_pending.set(false);
            self.filament_array.borrow_mut().clear();
            self.is_active.set(false);
            let helper = self.runout_helper();
            helper.set_sensor_enabled(false);
            helper.note_filament_present(eventtime, true)?;
            return Ok(NEVER);
        }
        let pos = {
            let toolhead = self
                .toolhead
                .borrow()
                .clone()
                .ok_or_else(|| KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'get_position'"
                        .to_string(),
                })?;
            toolhead.get_position()
        };
        let last_epos = pos[3];
        self.update_filament_array(last_epos)?;
        let diameter = self.diameter.get();
        self.runout_helper().note_filament_present(
            eventtime,
            self.runout_dia_min <= diameter && diameter <= self.runout_dia_max,
        )?;
        if diameter > self.runout_dia_min {
            let array_nonempty = !self.filament_array.borrow().is_empty();
            if array_nonempty {
                let pending_position = self.filament_array.borrow()[0].0;
                if pending_position <= last_epos {
                    let item = self.filament_array.borrow_mut().remove(0);
                    self.filament_width.set(item.1);
                } else {
                    let first = self.first_extruder_update_position.get();
                    if self.use_current_dia_while_delay && first == pending_position {
                        self.filament_width.set(diameter);
                    } else if first == pending_position {
                        self.filament_width.set(self.nominal_filament_dia);
                    }
                }
                if self.enable_flow_compensation.get() {
                    let filament_width = self.filament_width.get();
                    if filament_width <= self.max_diameter
                        && filament_width >= self.min_diameter
                    {
                        let percentage = round_py(
                            py_pow(self.nominal_filament_dia, 2.0)
                                / py_pow(filament_width, 2.0)
                                * 100.0,
                        )
                        .unwrap_or_else(|_| PyInt::from_i64(0));
                        self.gcode_handle().run_script(&format!(
                            "M221 S{}",
                            PyValue::Int(percentage.clone()).str()
                        ))?;
                    } else {
                        self.gcode_handle().run_script("M221 S100")?;
                    }
                }
            }
        } else {
            if self.enable_flow_compensation.get() {
                self.gcode_handle().run_script("M221 S100")?;
            }
            self.filament_array.borrow_mut().clear();
        }
        if self.is_active.get() {
            Ok(eventtime + 1.0)
        } else {
            Ok(NEVER)
        }
    }

    /// `cmd_M407` / `QUERY_FILAMENT_WIDTH` (`:213`).
    pub fn cmd_m407(&self, gcmd: &GCmd) -> KResult<()> {
        let mut response = "Filament diameter: ".to_string();
        if self.is_active.get() {
            response.push_str(&format!("{} mm", fmt_percent_3f(self.diameter.get())));
        } else {
            response.push_str("sensor disabled");
        }
        response.push_str("\nFilament state:");
        if self.is_active.get() {
            if self.runout_helper().filament_present() {
                response.push_str(" present");
            } else {
                response.push_str(" not present");
            }
        } else {
            response.push_str(" sensor disabled");
        }
        response.push_str(&format!(
            "\nWidth sensor:{}",
            if self.is_active.get() { " ON" } else { " OFF" }
        ));
        response.push_str(&format!(
            "\nFlow compensation:{}",
            if self.enable_flow_compensation.get() {
                " ON"
            } else {
                " OFF"
            }
        ));
        gcmd.respond_info(&response, true);
        Ok(())
    }

    /// `cmd_ClearFilamentArray` / `RESET_FILAMENT_WIDTH_SENSOR` (`:234`).
    pub fn cmd_clear_filament_array(&self, gcmd: &GCmd) -> KResult<()> {
        self.filament_array.borrow_mut().clear();
        if self.is_active.get() {
            self.array_clear_pending.set(true);
        } else {
            self.gcode_handle().run_script_from_command("M221 S100")?;
        }
        gcmd.respond_info("Filament width measurements cleared!", true);
        Ok(())
    }

    /// `cmd_M405` / `ENABLE_FILAMENT_WIDTH_SENSOR` (`:245`).
    pub fn cmd_m405(&self, gcmd: &GCmd) -> KResult<()> {
        let flow_comp = gcmd.get_int(
            "FLOW_COMPENSATION",
            klippy_api::gcode::GetIntOpts {
                default: Some(None),
                minval: Some(0),
                maxval: Some(1),
            },
        )?;
        if let Some(flow_comp) = flow_comp {
            if self.enable_flow_compensation.get() && flow_comp == 0 {
                self.flow_comp_disable_pending.set(true);
                self.enable_flow_compensation.set(false);
            } else if !self.enable_flow_compensation.get() && flow_comp != 0 {
                self.flow_comp_disable_pending.set(false);
                self.enable_flow_compensation.set(true);
            }
        }
        let was_active = self.is_active.get();
        self.is_active.set(true);
        self.runout_helper().set_sensor_enabled(true);
        let response = format!(
            "Filament width sensor: ON\nFlow compensation: {}",
            if self.enable_flow_compensation.get() {
                "ON"
            } else {
                "OFF"
            }
        );
        if !was_active {
            self.sensor_disable_pending.set(false);
            let timer_cell = self.extrude_factor_update_timer.borrow();
            let timer = timer_cell
                .as_ref()
                .expect("extrude_factor_update_timer registered in __init__");
            self.reactor.update_timer(timer, NOW);
        }
        gcmd.respond_info(&response, true);
        Ok(())
    }

    /// `cmd_M406` / `DISABLE_FILAMENT_WIDTH_SENSOR` (`:269`).
    pub fn cmd_m406(&self, gcmd: &GCmd) -> KResult<()> {
        if self.is_active.get() {
            self.sensor_disable_pending.set(true);
        } else {
            self.filament_array.borrow_mut().clear();
            self.runout_helper().set_sensor_enabled(false);
            self.runout_helper().note_filament_present(NOW, true)?;
            self.gcode_handle().run_script_from_command("M221 S100")?;
        }
        gcmd.respond_info("Filament width sensor: OFF", true);
        Ok(())
    }

    /// `cmd_Get_Raw_Values` / `QUERY_RAW_FILAMENT_WIDTH` (`:282`).
    pub fn cmd_get_raw_values(&self, gcmd: &GCmd) -> KResult<()> {
        let response = format!(
            "ADC1={} ADC2={} RAW={}",
            self.last_filament_width_reading.get(),
            self.last_filament_width_reading2.get(),
            self.last_filament_width_reading.get()
                + self.last_filament_width_reading2.get()
        );
        gcmd.respond_info(&response, true);
        Ok(())
    }

    /// `get_status` (`:290`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        let mut status = match self.runout_helper().get_status(eventtime) {
            PyValue::Dict(map) => map,
            _ => IndexMap::new(),
        };
        status.insert(
            PyValue::Str("Diameter".to_string()),
            PyValue::Float(self.diameter.get()),
        );
        status.insert(
            PyValue::Str("Raw".to_string()),
            PyValue::Int(PyInt::from_i64(
                self.last_filament_width_reading.get()
                    + self.last_filament_width_reading2.get(),
            )),
        );
        status.insert(
            PyValue::Str("is_active".to_string()),
            PyValue::Bool(self.is_active.get()),
        );
        status.insert(
            PyValue::Str("flow_compensation_enabled".to_string()),
            PyValue::Bool(self.enable_flow_compensation.get()),
        );
        PyValue::Dict(status)
    }

    /// `cmd_log_enable` / `ENABLE_FILAMENT_WIDTH_LOG` (`:301`).
    pub fn cmd_log_enable(&self, gcmd: &GCmd) -> KResult<()> {
        self.is_log.set(true);
        gcmd.respond_info("Filament width logging: ON", true);
        Ok(())
    }

    /// `cmd_log_disable` / `DISABLE_FILAMENT_WIDTH_LOG` (`:307`).
    pub fn cmd_log_disable(&self, gcmd: &GCmd) -> KResult<()> {
        self.is_log.set(false);
        gcmd.respond_info("Filament width logging: OFF", true);
        Ok(())
    }
}
impl PrinterObject for HallFilamentWidthSensor {
    klippy_api::printer_object_base!();

    /// `get_status(eventtime)` (`:290`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// `load_config(config)` (`klippy/extras/hall_filament_width_sensor.py:311`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let sensor = HallFilamentWidthSensor::new(config)?;
    let helper = Arc::new(RunoutHelper::new(config)?);
    sensor.set_runout_helper(helper);
    Ok(Some(sensor as ObjectRef))
}
/// Module entry point for the `hall_filament_width_sensor` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
