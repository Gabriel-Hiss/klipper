//! Port of `klippy/extras/temperature_fan.py`.
//!
//! A fan whose speed is driven by a temperature sensor. The sensor
//! callback updates the target speed through a control algorithm
//! (bang-bang or PID) and `set_tf_speed` rate-limits the resulting
//! `Fan.set_speed` calls.

use indexmap::IndexMap;
use pycompat::PyValue;
use pycompat::text::{percent_format, round_py_nd, PctValue};
use std::sync::{Arc, Weak};

use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts};
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts as GcmdGetFloatOpts};
use klippy_api::object::{load_interface, lookup_interface, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{KResult, KlippyError};

use crate::fan::Fan;
use crate::heaters::{PrinterHeatersApi, SensorApi, TemperatureSensorApi};

/// `klippy/extras/temperature_fan.py:8`.
pub const KELVIN_TO_CELSIUS: f64 = -273.15;
/// `klippy/extras/temperature_fan.py:9`.
pub const MAX_FAN_TIME: f64 = 5.0;
/// `klippy/extras/temperature_fan.py:10`.
pub const AMBIENT_TEMP: f64 = 25.;
/// `klippy/extras/temperature_fan.py:11`.
pub const PID_PARAM_BASE: f64 = 255.;
/// `klippy/extras/temperature_fan.py:140`.
pub const PID_SETTLE_DELTA: f64 = 1.;
/// `klippy/extras/temperature_fan.py:141`.
pub const PID_SETTLE_SLOPE: f64 = 0.1;

fn py_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

fn py_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

fn format_message(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_default()
}

// ---------------------------------------------------------------------------
// ControlBangBang (klippy/extras/temperature_fan.py:117-134, owned by unit
// `extras.temperature_fan.ControlBangBang`)
// ---------------------------------------------------------------------------

/// `ControlBangBang` (`temperature_fan.py:117`).
pub struct ControlBangBang {
    max_delta: f64,
    heating: bool,
}

impl ControlBangBang {
    /// `ControlBangBang(temperature_fan, config)` (`temperature_fan.py:118`).
    pub fn new(config: &Config) -> KResult<ControlBangBang> {
        let max_delta = config
            .getfloat(
                "max_delta",
                GetFloatOpts {
                    default: Some(Some(2.0)),
                    above: Some(0.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(2.0);
        Ok(ControlBangBang {
            max_delta,
            heating: false,
        })
    }

    /// `control.max_delta`.
    pub fn max_delta(&self) -> f64 {
        self.max_delta
    }

    /// `control.heating`.
    pub fn heating(&self) -> bool {
        self.heating
    }
}

// ---------------------------------------------------------------------------
// ControlPID (temperature_fan.py:143-182, owned by unit
// `extras.temperature_fan.ControlBangBang`)
// ---------------------------------------------------------------------------

/// `ControlPID` (`temperature_fan.py:143`).
pub struct ControlPID {
    kp: f64,
    ki: f64,
    kd: f64,
    min_deriv_time: f64,
    temp_integ_max: f64,
    prev_temp: f64,
    prev_temp_time: f64,
    prev_temp_deriv: f64,
    prev_temp_integ: f64,
}

impl ControlPID {
    /// `ControlPID(temperature_fan, config)` (`temperature_fan.py:144`).
    pub fn new(config: &Config, max_speed: f64) -> KResult<ControlPID> {
        let kp = config.getfloat("pid_Kp", GetFloatOpts::default())?.unwrap() / PID_PARAM_BASE;
        let ki = config.getfloat("pid_Ki", GetFloatOpts::default())?.unwrap() / PID_PARAM_BASE;
        let kd = config.getfloat("pid_Kd", GetFloatOpts::default())?.unwrap() / PID_PARAM_BASE;
        let min_deriv_time = config
            .getfloat(
                "pid_deriv_time",
                GetFloatOpts {
                    default: Some(Some(2.)),
                    above: Some(0.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(2.);
        let mut temp_integ_max = 0.;
        if ki != 0. {
            temp_integ_max = max_speed / ki;
        }
        Ok(ControlPID {
            kp,
            ki,
            kd,
            min_deriv_time,
            temp_integ_max,
            prev_temp: AMBIENT_TEMP,
            prev_temp_time: 0.,
            prev_temp_deriv: 0.,
            prev_temp_integ: 0.,
        })
    }

    /// `control.Kp`.
    pub fn kp(&self) -> f64 {
        self.kp
    }

    /// `control.Ki`.
    pub fn ki(&self) -> f64 {
        self.ki
    }

    /// `control.Kd`.
    pub fn kd(&self) -> f64 {
        self.kd
    }

    /// `control.prev_temp`.
    pub fn prev_temp(&self) -> f64 {
        self.prev_temp
    }

    /// `control.prev_temp_time`.
    pub fn prev_temp_time(&self) -> f64 {
        self.prev_temp_time
    }

    /// `control.prev_temp_deriv`.
    pub fn prev_temp_deriv(&self) -> f64 {
        self.prev_temp_deriv
    }

    /// `control.prev_temp_integ`.
    pub fn prev_temp_integ(&self) -> f64 {
        self.prev_temp_integ
    }
}

enum ControlAlgo {
    BangBang(ControlBangBang),
    Pid(ControlPID),
}

struct TemperatureFanState {
    min_temp: f64,
    max_temp: f64,
    max_speed_conf: f64,
    max_speed: f64,
    min_speed_conf: f64,
    min_speed: f64,
    last_temp: f64,
    target_temp_conf: f64,
    target_temp: f64,
    next_speed_time: f64,
    last_speed_value: f64,
    speed_delay: f64,
    control: ControlAlgo,
}

impl TemperatureFanState {
    /// `TemperatureFan.set_tf_speed` (`temperature_fan.py:49-63`); `st` is
    /// the live state so control callbacks can share the lock.
    fn set_tf_speed(&mut self, read_time: f64, value: f64, fan: &Fan) -> KResult<()> {
        let mut value = value;
        if value <= 0. {
            value = 0.;
        } else if value < self.min_speed {
            value = self.min_speed;
        }
        if self.target_temp <= 0. {
            value = 0.;
        }
        if (read_time < self.next_speed_time || self.last_speed_value == 0.)
            && (value - self.last_speed_value).abs() < 0.05
        {
            // No significant change in value - can suppress update
            return Ok(());
        }
        let speed_time = read_time + self.speed_delay;
        self.next_speed_time = speed_time + 0.75 * MAX_FAN_TIME;
        self.last_speed_value = value;
        fan.set_speed(value, Some(speed_time))
    }

    /// `control.temperature_callback` for the installed algorithm
    /// (`temperature_fan.py:122,157`).
    fn control_temperature_callback(
        &mut self,
        read_time: f64,
        temp: f64,
        fan: &Fan,
    ) -> KResult<()> {
        match &mut self.control {
            ControlAlgo::BangBang(bb) => {
                let target_temp = self.target_temp;
                if bb.heating && temp >= target_temp + bb.max_delta {
                    bb.heating = false;
                } else if !bb.heating && temp <= target_temp - bb.max_delta {
                    bb.heating = true;
                }
                if bb.heating {
                    self.set_tf_speed(read_time, 0., fan)
                } else {
                    self.set_tf_speed(read_time, self.max_speed, fan)
                }
            }
            ControlAlgo::Pid(pid) => {
                let (speed, temp_deriv, temp_integ, co, bounded_co) = {
                    let target_temp = self.target_temp;
                    let time_diff = read_time - pid.prev_temp_time;
                    // Calculate change of temperature
                    let temp_diff = temp - pid.prev_temp;
                    let temp_deriv = if time_diff >= pid.min_deriv_time {
                        temp_diff / time_diff
                    } else {
                        (pid.prev_temp_deriv * (pid.min_deriv_time - time_diff) + temp_diff)
                            / pid.min_deriv_time
                    };
                    // Calculate accumulated temperature "error"
                    let temp_err = target_temp - temp;
                    let temp_integ = pid.prev_temp_integ + temp_err * time_diff;
                    let temp_integ = py_max(0., py_min(pid.temp_integ_max, temp_integ));
                    // Calculate output
                    let co = pid.kp * temp_err + pid.ki * temp_integ - pid.kd * temp_deriv;
                    let bounded_co = py_max(0., py_min(self.max_speed, co));
                    (py_max(self.min_speed, self.max_speed - bounded_co), temp_deriv, temp_integ, co, bounded_co)
                };
                // Matches Python order: `set_tf_speed` runs before the
                // `prev_*` state is stored; if it raises, the store is
                // skipped and the error propagates.
                self.set_tf_speed(read_time, speed, fan)?;
                if let ControlAlgo::Pid(pid) = &mut self.control {
                    pid.prev_temp = temp;
                    pid.prev_temp_time = read_time;
                    pid.prev_temp_deriv = temp_deriv;
                    if co == bounded_co {
                        pid.prev_temp_integ = temp_integ;
                    }
                }
                Ok(())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// TemperatureFan (temperature_fan.py:13-111)
// ---------------------------------------------------------------------------

/// `TemperatureFan` (`temperature_fan.py:13`).
pub struct TemperatureFan {
    name: String,
    fan: Arc<Fan>,
    state: kos::sync::Mutex<TemperatureFanState>,
    _sensor: Arc<dyn TemperatureSensorApi>,
}

impl TemperatureFan {
    /// `TemperatureFan.name` (`temperature_fan.py:15`).
    pub fn name(&self) -> &str {
        &self.name
    }
    /// `TemperatureFan.__init__(config)` (`temperature_fan.py:14-47`).
    pub fn new(config: &Config) -> KResult<Arc<TemperatureFan>> {
        let name = config
            .get_name()
            .split_whitespace()
            .nth(1)
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?
            .to_string();
        let printer = config.get_printer();
        let fan = Fan::new(config, 1.)?;
        let min_temp = config
            .getfloat(
                "min_temp",
                GetFloatOpts {
                    minval: Some(KELVIN_TO_CELSIUS),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("GetFloatOpts::default means required, which raises first");
        let max_temp = config
            .getfloat(
                "max_temp",
                GetFloatOpts {
                    above: Some(min_temp),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("GetFloatOpts::default means required, which raises first");
        let pheaters = load_interface::<dyn PrinterHeatersApi>(
            &*printer,
            config,
            "heaters",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let sensor = pheaters.setup_sensor(config)?;
        let tf = Arc::new(TemperatureFan {
            name: name.clone(),
            fan,
            _sensor: sensor.clone(),
            state: kos::sync::Mutex::new(TemperatureFanState {
                min_temp,
                max_temp,
                max_speed_conf: 0.,
                max_speed: 0.,
                min_speed_conf: 0.,
                min_speed: 0.,
                last_temp: 0.,
                target_temp_conf: 0.,
                target_temp: 0.,
                next_speed_time: 0.,
                last_speed_value: 0.,
                speed_delay: 0.,
                control: ControlAlgo::BangBang(ControlBangBang {
                    max_delta: 2.0,
                    heating: false,
                }),
            }),
        });
        sensor.setup_minmax(min_temp, max_temp)?;
        let weak: Weak<TemperatureFan> = Arc::downgrade(&tf);
        sensor.setup_callback(Box::new(move |read_time: f64, temp: f64| {
            if let Some(tf) = weak.upgrade() {
                let _ = tf.temperature_callback(read_time, temp);
            }
        }));
        pheaters.register_sensor(config, tf.clone(), None)?;
        let speed_delay = sensor.get_report_time_delta();
        let max_speed_conf = config
            .getfloat(
                "max_speed",
                GetFloatOpts {
                    default: Some(Some(1.)),
                    above: Some(0.),
                    maxval: Some(1.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(1.);
        let min_speed_conf = config
            .getfloat(
                "min_speed",
                GetFloatOpts {
                    default: Some(Some(0.3)),
                    minval: Some(0.),
                    maxval: Some(1.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.3);
        let target_temp_conf = config
            .getfloat(
                "target_temp",
                GetFloatOpts {
                    default: Some(Some(if max_temp > 40. { 40. } else { max_temp })),
                    minval: Some(min_temp),
                    maxval: Some(max_temp),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(if max_temp > 40. { 40. } else { max_temp });
        let mut choices = IndexMap::new();
        choices.insert(
            PyValue::Str("watermark".to_string()),
            PyValue::Str("watermark".to_string()),
        );
        choices.insert(PyValue::Str("pid".to_string()), PyValue::Str("pid".to_string()));
        let choice = config.getchoice("control", &PyValue::Dict(choices), GetChoiceOpts::default())?;
        let control = match &choice {
            PyValue::Str(s) if s == "watermark" => {
                ControlAlgo::BangBang(ControlBangBang::new(config)?)
            }
            PyValue::Str(s) if s == "pid" => {
                ControlAlgo::Pid(ControlPID::new(config, max_speed_conf)?)
            }
            other => {
                return Err(KlippyError::config(format!(
                    "Choice '{}' for option 'control' in section '{}' is not a valid choice",
                    other.str(),
                    config.get_name()
                )));
            }
        };
        {
            let mut st = tf.state.lock();
            st.speed_delay = speed_delay;
            st.max_speed_conf = max_speed_conf;
            st.max_speed = max_speed_conf;
            st.min_speed_conf = min_speed_conf;
            st.min_speed = min_speed_conf;
            st.target_temp_conf = target_temp_conf;
            st.target_temp = target_temp_conf;
            st.control = control;
        }
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let tf_cmd = tf.clone();
        gcode.register_mux_command(
            "SET_TEMPERATURE_FAN_TARGET",
            "TEMPERATURE_FAN",
            Some(&name),
            Box::new(move |gcmd: &GCmd| tf_cmd.cmd_set_temperature_fan_target(gcmd)),
            Some(Self::SET_TEMPERATURE_FAN_TARGET_HELP),
        )?;
        Ok(tf)
    }

    pub const SET_TEMPERATURE_FAN_TARGET_HELP: &'static str =
        "Sets a temperature fan target and fan speed limits";

    /// `TemperatureFan.set_tf_speed` (`temperature_fan.py:49`).
    pub fn set_tf_speed(&self, read_time: f64, value: f64) -> KResult<()> {
        let mut st = self.state.lock();
        st.set_tf_speed(read_time, value, &self.fan)
    }

    /// `TemperatureFan.temperature_callback` (`temperature_fan.py:64`).
    pub fn temperature_callback(&self, read_time: f64, temp: f64) -> KResult<()> {
        let mut st = self.state.lock();
        st.last_temp = temp;
        st.control_temperature_callback(read_time, temp, &self.fan)
    }

    /// `TemperatureFan.get_temp` (`temperature_fan.py:67`).
    pub fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        let st = self.state.lock();
        (st.last_temp, st.target_temp)
    }

    /// `TemperatureFan.get_min_speed` (`temperature_fan.py:69`).
    pub fn get_min_speed(&self) -> f64 {
        self.state.lock().min_speed
    }

    /// `TemperatureFan.get_max_speed` (`temperature_fan.py:71`).
    pub fn get_max_speed(&self) -> f64 {
        self.state.lock().max_speed
    }

    /// `TemperatureFan.get_status` (`temperature_fan.py:73`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        let mut status = self.fan.get_status(eventtime);
        if let PyValue::Dict(m) = &mut status {
            let last_temp = self.state.lock().last_temp;
            m.insert(
                PyValue::Str("temperature".to_string()),
                PyValue::Float(round_py_nd(last_temp, 2)),
            );
            let target_temp = self.state.lock().target_temp;
            m.insert(
                PyValue::Str("target".to_string()),
                PyValue::Float(target_temp),
            );
        }
        status
    }

    /// `TemperatureFan.cmd_SET_TEMPERATURE_FAN_TARGET`
    /// (`temperature_fan.py:80`).
    pub fn cmd_set_temperature_fan_target(&self, gcmd: &GCmd) -> KResult<()> {
        let target_temp_conf = self.state.lock().target_temp_conf;
        let temp = gcmd
            .get_float(
                "TARGET",
                GcmdGetFloatOpts {
                    default: Some(Some(target_temp_conf)),
                    ..GcmdGetFloatOpts::default()
                },
            )?
            .expect("get_float with a default yields a value");
        self.set_temp(temp)?;
        let min_speed = self.state.lock().min_speed;
        let min_speed = gcmd
            .get_float(
                "MIN_SPEED",
                GcmdGetFloatOpts {
                    default: Some(Some(min_speed)),
                    ..GcmdGetFloatOpts::default()
                },
            )?
            .expect("get_float with a default yields a value");
        let max_speed = self.state.lock().max_speed;
        let max_speed = gcmd
            .get_float(
                "MAX_SPEED",
                GcmdGetFloatOpts {
                    default: Some(Some(max_speed)),
                    ..GcmdGetFloatOpts::default()
                },
            )?
            .expect("get_float with a default yields a value");
        if min_speed > max_speed {
            return Err(KlippyError::command(format_message(
                "Requested min speed (%.1f) is greater than max speed (%.1f)",
                &[PctValue::Float(min_speed), PctValue::Float(max_speed)],
            )));
        }
        self.set_min_speed(min_speed)?;
        self.set_max_speed(max_speed)?;
        Ok(())
    }

    /// `TemperatureFan.set_temp` (`temperature_fan.py:92`).
    pub fn set_temp(&self, degrees: f64) -> KResult<()> {
        let (min_temp, max_temp) = {
            let st = self.state.lock();
            (st.min_temp, st.max_temp)
        };
        if degrees != 0. && (degrees < min_temp || degrees > max_temp) {
            return Err(KlippyError::command(format_message(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)",
                &[
                    PctValue::Float(degrees),
                    PctValue::Float(min_temp),
                    PctValue::Float(max_temp),
                ],
            )));
        }
        self.state.lock().target_temp = degrees;
        Ok(())
    }

    /// `TemperatureFan.set_min_speed` (`temperature_fan.py:99`).
    // `speed < 0. or speed > 1.` is false for NaN, so Python accepts it;
    // `RangeInclusive::contains` would reject it.
    #[allow(clippy::manual_range_contains)]
    pub fn set_min_speed(&self, speed: f64) -> KResult<()> {
        if speed != 0. && (speed < 0. || speed > 1.) {
            return Err(KlippyError::command(format_message(
                "Requested min speed (%.1f) out of range (0.0 : 1.0)",
                &[PctValue::Float(speed)],
            )));
        }
        self.state.lock().min_speed = speed;
        Ok(())
    }

    /// `TemperatureFan.set_max_speed` (`temperature_fan.py:106`).
    // Same NaN semantics as `set_min_speed`.
    #[allow(clippy::manual_range_contains)]
    pub fn set_max_speed(&self, speed: f64) -> KResult<()> {
        if speed != 0. && (speed < 0. || speed > 1.) {
            return Err(KlippyError::command(format_message(
                "Requested max speed (%.1f) out of range (0.0 : 1.0)",
                &[PctValue::Float(speed)],
            )));
        }
        self.state.lock().max_speed = speed;
        Ok(())
    }
}

impl PrinterObject for TemperatureFan {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn SensorApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(TemperatureFan::get_status(self, eventtime)))
    }
}

impl SensorApi for TemperatureFan {
    fn get_temp(&self, eventtime: f64) -> (f64, f64) {
        TemperatureFan::get_temp(self, eventtime)
    }
}

/// `load_config_prefix(config)` (`temperature_fan.py:184-185`).
pub fn load_config_prefix(
    config: &Config,
) -> KResult<Option<klippy_api::object::ObjectRef>> {
    let tf = TemperatureFan::new(config)?;
    Ok(Some(tf as klippy_api::object::ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
