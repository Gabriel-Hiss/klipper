//! Port of the `tsl1401cl_filament_width_sensor` module
//! (`klippy/extras/tsl1401cl_filament_width_sensor.py`).

use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCode, GCodeApi, RegisterCommandOpts};
use klippy_api::mcu::{AdcPin, AdcSampleOpts, Pin, PinsApi};
use klippy_api::object::{lookup_interface, EventArg, ObjectRef};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_KLIPPY_READY,
};
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::GCell;
use pycompat::text::{float_pow, round_py, round_py_nd};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::Arc;

pub const ADC_REPORT_TIME: f64 = 0.500;
pub const ADC_SAMPLE_TIME: f64 = 0.001;
pub const ADC_SAMPLE_COUNT: i64 = 8;
pub const MEASUREMENT_INTERVAL_MM: f64 = 10.0;

struct State {
    is_active: bool,
    filament_array: Vec<(f64, PyValue)>,
    last_filament_width_reading: PyValue,
    toolhead: Option<ToolHead>,
}

pub struct FilamentWidthSensor {
    printer: Printer,
    reactor: Reactor,
    pin: String,
    nominal_filament_dia: f64,
    measurement_delay: f64,
    measurement_max_difference: f64,
    max_diameter: f64,
    min_diameter: f64,
    gcode: GCell<Option<GCode>>,
    state: GCell<State>,
    timer: GCell<Option<Timer>>,
    mcu_adc: GCell<Option<AdcPin>>,
}

klippy_api::printer_object!(FilamentWidthSensor);

fn required_str(
    config: &Config,
    option: &str,
) -> KResult<String> {
    config.get(option, GetStrOpts::default())?.ok_or_else(|| {
        KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option,
            config.get_name()
        ))
    })
}

fn required_float(
    config: &Config,
    option: &str,
    above: f64,
) -> KResult<f64> {
    config
        .getfloat(
            option,
            GetFloatOpts {
                above: Some(above),
                ..Default::default()
            },
        )?
        .ok_or_else(|| {
            KlippyError::config(format!(
                "Option '{}' in section '{}' must be specified",
                option,
                config.get_name()
            ))
        })
}

impl FilamentWidthSensor {
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let pin = required_str(config, "pin")?;
        let nominal_filament_dia = required_float(
            config,
            "default_nominal_filament_diameter",
            1.0,
        )?;
        let measurement_delay = required_float(config, "measurement_delay", 0.0)?;
        let measurement_max_difference = required_float(config, "max_difference", 0.0)?;
        let max_diameter = nominal_filament_dia + measurement_max_difference;
        let min_diameter = nominal_filament_dia - measurement_max_difference;

        let sensor = Arc::new(FilamentWidthSensor {
            printer: printer.clone(),
            reactor: reactor.clone(),
            pin: pin.clone(),
            nominal_filament_dia,
            measurement_delay,
            measurement_max_difference,
            max_diameter,
            min_diameter,
            gcode: GCell::new(None),
            state: GCell::new(State {
                is_active: true,
                filament_array: Vec::new(),
                // Python: `self.lastFilamentWidthReading = 0` (an int).
                last_filament_width_reading: PyValue::Int(PyInt::from_i64(0)),
                toolhead: None,
            }),
            timer: GCell::new(None),
            mcu_adc: GCell::new(None),
        });

        let sensor_for_ready = sensor.clone();
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_args: &[EventArg]| {
                sensor_for_ready.handle_ready()?;
                Ok(PyValue::None)
            }),
        );

        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::config("Unknown config object 'pins'"))?;
        let pin_obj = pins.setup_pin("adc", &pin)?;
        let adc = match pin_obj {
            Pin::Adc(adc) => adc,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "setup_pin('adc') did not return an ADC pin".to_string(),
                });
            }
        };
        adc.setup_adc_sample(
            ADC_REPORT_TIME,
            AdcSampleOpts {
                sample_time: ADC_SAMPLE_TIME,
                sample_count: ADC_SAMPLE_COUNT,
                ..Default::default()
            },
        );
        let sensor_for_adc = sensor.clone();
        adc.setup_adc_callback(Box::new(move |samples| {
            sensor_for_adc.adc_callback(samples);
        }));
        *sensor.mcu_adc.borrow_mut() = Some(adc);

        let sensor_for_timer = sensor.clone();
        let timer = reactor.register_timer(
            move |eventtime| sensor_for_timer.extrude_factor_update_event(eventtime),
            NEVER,
        );
        *sensor.timer.borrow_mut() = Some(timer);

        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::config("Unknown config object 'gcode'"))?;
        *sensor.gcode.borrow_mut() = Some(gcode.clone());
        let s1 = sensor.clone();
        gcode.register_command(
            "QUERY_FILAMENT_WIDTH",
            Some(Box::new(move |gcmd| s1.cmd_m407(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let s2 = sensor.clone();
        gcode.register_command(
            "RESET_FILAMENT_WIDTH_SENSOR",
            Some(Box::new(move |gcmd| s2.cmd_clear_filament_array(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let s3 = sensor.clone();
        gcode.register_command(
            "DISABLE_FILAMENT_WIDTH_SENSOR",
            Some(Box::new(move |gcmd| s3.cmd_m406(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let s4 = sensor.clone();
        gcode.register_command(
            "ENABLE_FILAMENT_WIDTH_SENSOR",
            Some(Box::new(move |gcmd| s4.cmd_m405(gcmd))),
            RegisterCommandOpts::default(),
        )?;

        Ok(sensor)
    }

    pub fn pin(&self) -> &str {
        &self.pin
    }

    pub fn nominal_filament_dia(&self) -> f64 {
        self.nominal_filament_dia
    }

    pub fn measurement_delay(&self) -> f64 {
        self.measurement_delay
    }

    pub fn measurement_max_difference(&self) -> f64 {
        self.measurement_max_difference
    }

    pub fn max_diameter(&self) -> f64 {
        self.max_diameter
    }

    pub fn min_diameter(&self) -> f64 {
        self.min_diameter
    }

    pub fn is_active(&self) -> bool {
        self.state.borrow().is_active
    }

    pub fn filament_array(&self) -> Vec<(f64, PyValue)> {
        self.state.borrow().filament_array.clone()
    }

    pub fn last_filament_width_reading(&self) -> PyValue {
        self.state.borrow().last_filament_width_reading.clone()
    }

    fn boxed_gcode(&self) -> Option<GCode> {
        self.gcode.borrow().clone()
    }

    fn update_timer(&self, waketime: f64) {
        let timer_borrow = self.timer.borrow();
        if let Some(timer) = timer_borrow.as_ref() {
            self.reactor.update_timer(timer, waketime);
        }
    }

    pub fn handle_ready(&self) -> KResult<()> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .ok_or_else(|| KlippyError::config("Unknown config object 'toolhead'"))?;
        self.state.borrow_mut().toolhead = Some(toolhead);
        self.update_timer(NOW);
        Ok(())
    }

    pub fn adc_callback(&self, samples: Vec<(f64, f64)>) {
        let Some(&(_, read_value)) = samples.last() else {
            return;
        };
        // Python: `round(read_value * 5, 2)` returns a float.
        let width = round_py_nd(read_value * 5.0, 2);
        self.state.borrow_mut().last_filament_width_reading = PyValue::Float(width);
    }

    pub fn update_filament_array(&self, last_epos: f64) {
        let mut state = self.state.borrow_mut();
        let reading = state.last_filament_width_reading.clone();
        if !state.filament_array.is_empty() {
            let next_reading_position =
                state.filament_array[state.filament_array.len() - 1].0
                    + MEASUREMENT_INTERVAL_MM;
            if next_reading_position <= (last_epos + self.measurement_delay) {
                state.filament_array.push((
                    last_epos + self.measurement_delay,
                    reading,
                ));
            }
        } else {
            state.filament_array.push((
                self.measurement_delay + last_epos,
                reading,
            ));
        }
    }

    pub fn extrude_factor_update_event(&self, eventtime: f64) -> f64 {
        let last_epos = {
            let state = self.state.borrow();
            let Some(toolhead) = state.toolhead.as_ref() else {
                return NEVER;
            };
            let pos = toolhead.get_position();
            pos[3]
        };
        self.update_filament_array(last_epos);

        let script: Option<String> = {
            let mut state = self.state.borrow_mut();
            let last = state.last_filament_width_reading.to_f64().unwrap_or(0.0);
            if last > 0.5 {
                if !state.filament_array.is_empty() {
                    let pending_position = state.filament_array[0].0;
                    if pending_position <= last_epos {
                        let (_, filament_width) = state.filament_array.remove(0);
                        let filament_width =
                            filament_width.to_f64().unwrap_or(0.0);
                        if (filament_width <= self.max_diameter)
                            && (filament_width >= self.min_diameter)
                        {
                            let nominal_pow =
                                float_pow(self.nominal_filament_dia, 2.0).unwrap_or(0.0);
                            let width_pow_new = float_pow(filament_width, 2.0).unwrap_or(0.0);
                            let value = nominal_pow / width_pow_new * 100.0;
                            match round_py(value) {
                                Ok(percentage) => Some(format!(
                                    "M221 S{}",
                                    PyValue::Int(percentage).str()
                                )),
                                Err(_) => Some("M221 S100".to_string()),
                            }
                        } else {
                            Some("M221 S100".to_string())
                        }
                    } else {
                        None
                    }
                } else {
                    None
                }
            } else {
                state.filament_array.clear();
                Some("M221 S100".to_string())
            }
        };

        if let Some(script) = script {
            if let Some(gcode) = self.boxed_gcode() {
                let _ = gcode.run_script(&script);
            }
        }

        if self.state.borrow().is_active {
            eventtime + 1.0
        } else {
            NEVER
        }
    }

    pub fn cmd_m407(&self, gcmd: &GCmd) -> KResult<()> {
        let last = self.state.borrow().last_filament_width_reading.clone();
        let response = if last.to_f64().unwrap_or(0.0) > 0.0 {
            format!("Filament dia (measured mm): {}", last.str())
        } else {
            "Filament NOT present".to_string()
        };
        gcmd.respond_info(&response, true);
        Ok(())
    }

    pub fn cmd_clear_filament_array(
        &self,
        gcmd: &GCmd,
    ) -> KResult<()> {
        self.state.borrow_mut().filament_array.clear();
        gcmd.respond_info("Filament width measurements cleared!", true);
        if let Some(gcode) = self.boxed_gcode() {
            return gcode.run_script_from_command("M221 S100");
        }
        Ok(())
    }

    pub fn cmd_m405(&self, gcmd: &GCmd) -> KResult<()> {
        let is_active = self.state.borrow().is_active;
        let response = if is_active {
            "Filament width sensor is already On".to_string()
        } else {
            self.state.borrow_mut().is_active = true;
            self.update_timer(NOW);
            "Filament width sensor Turned On".to_string()
        };
        gcmd.respond_info(&response, true);
        Ok(())
    }

    pub fn cmd_m406(&self, gcmd: &GCmd) -> KResult<()> {
        if !self.state.borrow().is_active {
            gcmd.respond_info("Filament width sensor is already Off", true);
            return Ok(());
        }
        self.state.borrow_mut().is_active = false;
        self.update_timer(NEVER);
        self.state.borrow_mut().filament_array.clear();
        let result = if let Some(gcode) = self.boxed_gcode() {
            gcode.run_script_from_command("M221 S100")
        } else {
            Ok(())
        };
        gcmd.respond_info("Filament width sensor Turned Off", true);
        result
    }
}

pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let sensor = FilamentWidthSensor::new(config)?;
    Ok(Some(sensor))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        ..Default::default()
    }
}
