//! Port of the `heaters` module (`klippy/extras/heaters.py`).
//!
//! Tracks PWM controlled heaters and their temperature control. The module
//! owns the `Heater` object, the watermark and PID control algorithms, and
//! the `PrinterHeaters` registry object. Other modules reach a heater
//! through [`HeaterApi`] and register sensor factories through
//! [`PrinterHeatersApi`].

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetChoiceOpts, GetFloatOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GetFloatOpts as GcodeGetFloatOpts,
    GetOpts as GcodeGetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::mcu::Pin;
use klippy_api::object::{lookup_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_GCODE_REQUEST_RESTART,
    EV_KLIPPY_READY, EV_KLIPPY_SHUTDOWN,
};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{EventArg, GCell};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;
use std::sync::{Arc, Weak};

/// `klippy/extras/heaters.py:13`.
pub const KELVIN_TO_CELSIUS: f64 = -273.15;
/// `klippy/extras/heaters.py:14`.
pub const MAX_HEAT_TIME: f64 = 3.0;
/// `klippy/extras/heaters.py:15`.
pub const AMBIENT_TEMP: f64 = 25.;
/// `klippy/extras/heaters.py:16`.
pub const PID_PARAM_BASE: f64 = 255.;
/// `klippy/extras/heaters.py:17`.
pub const MAX_MAINTHREAD_TIME: f64 = 5.0;
/// `klippy/extras/heaters.py:18`.
pub const QUELL_STALE_TIME: f64 = 7.0;
/// `klippy/extras/heaters.py:19`.
pub const MIN_PWM_CHANGE_RATIO: f64 = 0.05;
/// `klippy/extras/heaters.py:189`.
pub const PID_SETTLE_DELTA: f64 = 1.;
/// `klippy/extras/heaters.py:190`.
pub const PID_SETTLE_SLOPE: f64 = 0.1;

/// Temperature sample callback `cb(read_time, temp)` (`heaters.py:88`).
pub type TemperatureCallback = Box<dyn FnMut(f64, f64) + Send>;

/// A temperature sensor chip used by [`Heater`].
///
/// This is the duck type behind `sensor` in `heaters.py`: the object a
/// sensor factory returns (`thermistor`, `adc_temperature`, `lm75`, ...).
/// `Heater` only calls these three methods on it; reading a temperature
/// back goes through [`SensorApi`], which sensor chips do not implement.
pub trait TemperatureSensorApi: Send + Sync + 'static {
    /// `sensor.setup_minmax(min_temp, max_temp)` (`heaters.py:30`). An
    /// exception the sensor raises (e.g. the converter's `calc_adc`)
    /// propagates out of `Heater.__init__` and its caller.
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()>;
    /// `sensor.setup_callback(callback)` (`heaters.py:31`).
    fn setup_callback(&self, callback: TemperatureCallback);
    /// `sensor.get_report_time_delta()` (`heaters.py:32`).
    fn get_report_time_delta(&self) -> f64;
}

/// An object registered with `heaters.register_sensor` (`heaters.py:301`)
/// or named by `TEMPERATURE_WAIT SENSOR=`: [`Heater`], `temperature_sensor`,
/// `temperature_fan`, `z_thermal_adjust`, ... `PrinterHeaters` only reads
/// its temperature (`heaters.py:336,384`).
pub trait SensorApi: Send + Sync + 'static {
    /// `psensor.get_temp(eventtime)` -> `(current, target)`.
    fn get_temp(&self, eventtime: f64) -> (f64, f64);
}

/// A heater temperature-control algorithm (`heaters.py:166,192`).
///
/// `pid_calibrate.ControlAutoTune` is another implementation of this duck
/// type, swapped in with [`Heater::set_control`].
pub trait ControlApi: Send + Sync + 'static {
    /// `control.temperature_update(read_time, temp, target_temp)`; an exception
    /// (`ControlAutoTune.calc_pid`'s `ZeroDivisionError`) leaves
    /// `Heater.temperature_callback`.
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64) -> KResult<()>;
    /// `control.check_busy(eventtime, smoothed_temp, target_temp)`.
    fn check_busy(&self, eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool;
}

/// A heater object (`heaters.py:21`).
pub trait HeaterApi: Send + Sync + 'static {
    /// `heater.get_name()` (`heaters.py:101`).
    fn get_name(&self) -> String;
    /// `heater.get_pwm_delay()` (`heaters.py:103`).
    fn get_pwm_delay(&self) -> f64;
    /// `heater.get_max_power()` (`heaters.py:105`).
    fn get_max_power(&self) -> f64;
    /// `heater.get_smooth_time()` (`heaters.py:107`).
    fn get_smooth_time(&self) -> f64;
    /// `heater.set_pwm(read_time, value)` (`heaters.py:72`).
    fn set_pwm(&self, read_time: f64, value: f64) -> KResult<()>;
    /// `heater.temperature_callback(read_time, temp)` (`heaters.py:87`).
    fn temperature_callback(&self, read_time: f64, temp: f64) -> KResult<()>;
    /// `heater.set_temp(degrees)` (`heaters.py:109`).
    fn set_temp(&self, degrees: f64) -> KResult<()>;
    /// `heater.get_temp(eventtime)` (`heaters.py:116`).
    fn get_temp(&self, eventtime: f64) -> (f64, f64);
    /// `heater.check_busy(eventtime)` (`heaters.py:123`).
    fn check_busy(&self, eventtime: f64) -> bool;
    /// `heater.set_control(control)` (`heaters.py:127`).
    fn set_control(&self, control: Arc<dyn ControlApi>) -> Arc<dyn ControlApi>;
    /// `heater.alter_target(target_temp)` (`heaters.py:133`).
    fn alter_target(&self, target_temp: f64);
    /// `heater.stats(eventtime)` (`heaters.py:137`).
    fn stats(&self, eventtime: f64) -> (bool, String);
    /// `heater.get_status(eventtime)` (`heaters.py:148`).
    fn get_status(&self, eventtime: f64) -> PyValue;
    /// `heater.min_temp`.
    fn min_temp(&self) -> f64;
    /// `heater.max_temp`.
    fn max_temp(&self) -> f64;
    /// `heater.min_extrude_temp`.
    fn min_extrude_temp(&self) -> f64;
    /// `heater.can_extrude`.
    fn can_extrude(&self) -> bool;
    /// `heater.target_temp`.
    fn target_temp(&self) -> f64;
}

/// The `heaters` printer object (`heaters.py:242`).
pub trait PrinterHeatersApi: Send + Sync + 'static {
    /// `heaters.add_sensor_factory(sensor_type, factory)` (`heaters.py:273`).
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory);
    /// `heaters.setup_heater(config, gcode_id=None)` (`heaters.py:275`).
    fn setup_heater(&self, config: &Config, gcode_id: Option<String>) -> KResult<Arc<Heater>>;
    /// `heaters.get_all_heaters()` (`heaters.py:286`).
    fn get_all_heaters(&self) -> Vec<String>;
    /// `heaters.lookup_heater(heater_name)` (`heaters.py:288`).
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>>;
    /// `heaters.setup_sensor(config)` (`heaters.py:293`).
    fn setup_sensor(&self, config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>>;
    /// `heaters.register_sensor(config, psensor, gcode_id=None)` (`heaters.py:301`).
    fn register_sensor(
        &self,
        config: &Config,
        psensor: Arc<dyn SensorApi>,
        gcode_id: Option<String>,
    ) -> KResult<()>;
    /// `heaters.register_monitor(config)` (`heaters.py:316`).
    fn register_monitor(&self, config: &Config);
    /// `heaters.turn_off_all_heaters(print_time=0.)` (`heaters.py:322`).
    fn turn_off_all_heaters(&self, print_time: f64);
    /// `heaters.set_temperature(heater, temp, wait=False)` (`heaters.py:360`).
    fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()>;
}

/// Sensor factory: `factory(config) -> sensor` (`heaters.py:273,300`).
pub type SensorFactory =
    Arc<dyn Fn(&Config) -> KResult<Arc<dyn TemperatureSensorApi>> + Send + Sync>;

fn is_fileoutput(printer: &Printer) -> bool {
    match printer.get_start_args() {
        PyValue::Dict(map) => match map.get(&PyValue::Str("debugoutput".to_string())) {
            Some(PyValue::None) | None => false,
            Some(_) => true,
        },
        _ => false,
    }
}

fn percent(fmt: &str, args: &[PctValue]) -> String {
    match percent_format(fmt, args) {
        Ok(s) => s,
        Err(_) => String::new(),
    }
}

fn float_opts(required: bool, default: Option<f64>) -> GetFloatOpts {
    let mut opts = GetFloatOpts::default();
    opts.default = if required { None } else { Some(default) };
    opts
}

// Watermark control (`heaters.py:166`).

struct BangBangState {
    heating: bool,
}

/// `ControlBangBang` (`heaters.py:166`).
pub struct ControlBangBang {
    heater: Weak<Heater>,
    heater_max_power: f64,
    max_delta: f64,
    state: kos::sync::Mutex<BangBangState>,
}

impl ControlBangBang {
    /// `ControlBangBang(heater, config)` (`heaters.py:167`).
    pub fn new(heater: Weak<Heater>, heater_max_power: f64, config: &Config) -> KResult<Self> {
        let mut opts = float_opts(false, Some(2.0));
        opts.above = Some(0.);
        let max_delta = config.getfloat("max_delta", opts)?.unwrap();
        Ok(ControlBangBang {
            heater,
            heater_max_power,
            max_delta,
            state: kos::sync::Mutex::new(BangBangState { heating: false }),
        })
    }

    /// `control.max_delta`.
    pub fn max_delta(&self) -> f64 {
        self.max_delta
    }

    /// `control.heating`.
    pub fn heating(&self) -> bool {
        self.state.lock().heating
    }
}

impl ControlApi for ControlBangBang {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64) -> KResult<()> {
        let heating = {
            let mut st = self.state.lock();
            if st.heating && temp >= target_temp + self.max_delta {
                st.heating = false;
            } else if !st.heating && temp <= target_temp - self.max_delta {
                st.heating = true;
            }
            st.heating
        };
        if let Some(heater) = self.heater.upgrade() {
            if heating {
                heater.set_pwm(read_time, self.heater_max_power)?;
            } else {
                heater.set_pwm(read_time, 0.)?;
            }
        }
        Ok(())
    }

    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        smoothed_temp < target_temp - self.max_delta
    }
}

// PID control (`heaters.py:192`).

struct PidState {
    prev_temp: f64,
    prev_temp_time: f64,
    prev_temp_deriv: f64,
    prev_temp_integ: f64,
}

/// `ControlPID` (`heaters.py:192`).
pub struct ControlPID {
    heater: Weak<Heater>,
    heater_max_power: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    min_deriv_time: f64,
    temp_integ_max: f64,
    state: kos::sync::Mutex<PidState>,
}

impl ControlPID {
    /// `ControlPID(heater, config)` (`heaters.py:193`).
    pub fn new(
        heater: Weak<Heater>,
        heater_max_power: f64,
        smooth_time: f64,
        config: &Config,
    ) -> KResult<Self> {
        let kp = config.getfloat("pid_Kp", float_opts(true, None))?.unwrap() / PID_PARAM_BASE;
        let ki = config.getfloat("pid_Ki", float_opts(true, None))?.unwrap() / PID_PARAM_BASE;
        let kd = config.getfloat("pid_Kd", float_opts(true, None))?.unwrap() / PID_PARAM_BASE;
        let temp_integ_max = if ki != 0. { heater_max_power / ki } else { 0. };
        Ok(ControlPID {
            heater,
            heater_max_power,
            kp,
            ki,
            kd,
            min_deriv_time: smooth_time,
            temp_integ_max,
            state: kos::sync::Mutex::new(PidState {
                prev_temp: AMBIENT_TEMP,
                prev_temp_time: 0.,
                prev_temp_deriv: 0.,
                prev_temp_integ: 0.,
            }),
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

    /// `control.prev_temp_deriv`.
    pub fn prev_temp_deriv(&self) -> f64 {
        self.state.lock().prev_temp_deriv
    }
}

impl ControlApi for ControlPID {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64) -> KResult<()> {
        let bounded_co = {
            let mut st = self.state.lock();
            let time_diff = read_time - st.prev_temp_time;
            let temp_diff = temp - st.prev_temp;
            let temp_deriv = if time_diff >= self.min_deriv_time {
                temp_diff / time_diff
            } else {
                (st.prev_temp_deriv * (self.min_deriv_time - time_diff) + temp_diff)
                    / self.min_deriv_time
            };
            let temp_err = target_temp - temp;
            let mut temp_integ = st.prev_temp_integ + temp_err * time_diff;
            temp_integ = f64::max(0., f64::min(self.temp_integ_max, temp_integ));
            let co = self.kp * temp_err + self.ki * temp_integ - self.kd * temp_deriv;
            let bounded_co = f64::max(0., f64::min(self.heater_max_power, co));
            st.prev_temp = temp;
            st.prev_temp_time = read_time;
            st.prev_temp_deriv = temp_deriv;
            if co == bounded_co {
                st.prev_temp_integ = temp_integ;
            }
            bounded_co
        };
        if let Some(heater) = self.heater.upgrade() {
            heater.set_pwm(read_time, bounded_co)?;
        }
        Ok(())
    }

    fn check_busy(&self, _eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        let temp_diff = target_temp - smoothed_temp;
        let prev_deriv = self.state.lock().prev_temp_deriv;
        f64::abs(temp_diff) > PID_SETTLE_DELTA || f64::abs(prev_deriv) > PID_SETTLE_SLOPE
    }
}

// Heater (`heaters.py:21`).

struct HeaterState {
    min_temp: f64,
    max_temp: f64,
    min_extrude_temp: f64,
    max_power: f64,
    min_pwm_change: f64,
    smooth_time: f64,
    inv_smooth_time: f64,
    pwm_delay: f64,
    verify_mainthread_time: f64,
    last_temp: f64,
    smoothed_temp: f64,
    target_temp: f64,
    last_temp_time: f64,
    next_pwm_time: f64,
    last_pwm_value: f64,
    can_extrude: bool,
    control: Option<Arc<dyn ControlApi>>,
    mcu_pwm: Option<klippy_api::mcu::PwmPin>,
}

impl Default for HeaterState {
    fn default() -> Self {
        HeaterState {
            min_temp: 0.,
            max_temp: 0.,
            min_extrude_temp: 0.,
            max_power: 0.,
            min_pwm_change: 0.,
            smooth_time: 0.,
            inv_smooth_time: 0.,
            pwm_delay: 0.,
            verify_mainthread_time: -999.,
            last_temp: 0.,
            smoothed_temp: 0.,
            target_temp: 0.,
            last_temp_time: 0.,
            next_pwm_time: 0.,
            last_pwm_value: 0.,
            can_extrude: false,
            control: None,
            mcu_pwm: None,
        }
    }
}

/// `Heater` (`heaters.py:21`).
pub struct Heater {
    printer: Printer,
    name: String,
    short_name: String,
    sensor: Arc<dyn TemperatureSensorApi>,
    state: kos::sync::Mutex<HeaterState>,
    // Python's self.lock spans control callbacks. Keep it separate from the
    // state mutex so those callbacks can call set_pwm and alter_target.
    control_lock: kos::sync::Mutex<()>,
}

impl Heater {
    fn control_arc(&self) -> Arc<dyn ControlApi> {
        self.state.lock().control.clone().expect("heater control is set")
    }

    fn mcu_pwm_handle(&self) -> klippy_api::mcu::PwmPin {
        self.state
            .lock()
            .mcu_pwm
            .clone()
            .expect("heater pin is set")
    }

    /// `Heater.__init__(config, sensor)` (`heaters.py:22`).
    pub fn new(config: &Config, sensor: Arc<dyn TemperatureSensorApi>) -> KResult<Arc<Heater>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let short_name = name.split_whitespace().last().unwrap_or("").to_string();
        let mut min_opts = float_opts(true, None);
        min_opts.minval = Some(KELVIN_TO_CELSIUS);
        let min_temp = config.getfloat("min_temp", min_opts)?.unwrap();
        let mut max_opts = float_opts(true, None);
        max_opts.above = Some(min_temp);
        let max_temp = config.getfloat("max_temp", max_opts)?.unwrap();
        let heater = Arc::new(Heater {
            printer: printer.clone(),
            name,
            short_name: short_name.clone(),
            sensor: sensor.clone(),
            control_lock: kos::sync::Mutex::new(()),
            state: kos::sync::Mutex::new(HeaterState {
                min_temp,
                max_temp,
                ..Default::default()
            }),
        });
        sensor.setup_minmax(min_temp, max_temp)?;
        {
            let heater_cb = Arc::downgrade(&heater);
            sensor.setup_callback(Box::new(move |read_time: f64, temp: f64| {
                let Some(heater_cb) = heater_cb.upgrade() else { return };
                // Python lets the exception escape into the sensor's callback caller
                // (serial response handler or reactor timer), which reports it.
                if let Err(e) = heater_cb.temperature_callback(read_time, temp) {
                    e.raise();
                }
            }));
        }
        let pwm_delay = sensor.get_report_time_delta();
        let mut extrude_opts = float_opts(false, Some(170.));
        extrude_opts.minval = Some(min_temp);
        extrude_opts.maxval = Some(max_temp);
        let min_extrude_temp = config.getfloat("min_extrude_temp", extrude_opts)?.unwrap();
        let can_extrude = min_extrude_temp <= 0. || is_fileoutput(&printer);
        let mut power_opts = float_opts(false, Some(1.));
        power_opts.above = Some(0.);
        power_opts.maxval = Some(1.);
        let max_power = config.getfloat("max_power", power_opts)?.unwrap();
        let min_pwm_change = max_power * MIN_PWM_CHANGE_RATIO;
        let mut smooth_opts = float_opts(false, Some(1.));
        smooth_opts.above = Some(0.);
        let smooth_time = config.getfloat("smooth_time", smooth_opts)?.unwrap();
        let inv_smooth_time = 1. / smooth_time;
        {
            let mut st = heater.state.lock();
            st.pwm_delay = pwm_delay;
            st.min_extrude_temp = min_extrude_temp;
            st.max_power = max_power;
            st.min_pwm_change = min_pwm_change;
            st.smooth_time = smooth_time;
            st.inv_smooth_time = inv_smooth_time;
            st.can_extrude = can_extrude;
        }
        let mut choices = IndexMap::new();
        choices.insert(PyValue::Str("watermark".to_string()), PyValue::Str("watermark".to_string()));
        choices.insert(PyValue::Str("pid".to_string()), PyValue::Str("pid".to_string()));
        let choice = config.getchoice("control", &PyValue::Dict(choices), GetChoiceOpts::default())?;
        let control: Arc<dyn ControlApi> = match &choice {
            PyValue::Str(s) if s == "watermark" => {
                Arc::new(ControlBangBang::new(Arc::downgrade(&heater), max_power, config)?)
            }
            PyValue::Str(s) if s == "pid" => {
                Arc::new(ControlPID::new(Arc::downgrade(&heater), max_power, smooth_time, config)?)
            }
            other => {
                return Err(KlippyError::config(format!(
                    "Choice '{}' for option 'control' in section '{}' is not a valid choice",
                    other.str(),
                    config.get_name()
                )));
            }
        };
        heater.state.lock().control = Some(control);
        let heater_pin = config.get("heater_pin", GetStrOpts::default())?.unwrap();
        let pins = lookup_interface::<dyn klippy_api::mcu::PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let pin = pins.setup_pin("pwm", &heater_pin)?;
        let mcu_pwm = match pin {
            Pin::Pwm(p) => p,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "heater_pin is not a pwm pin".to_string(),
                });
            }
        };
        let mut cycle_opts = float_opts(false, Some(0.100));
        cycle_opts.above = Some(0.);
        cycle_opts.maxval = Some(pwm_delay);
        let pwm_cycle_time = config.getfloat("pwm_cycle_time", cycle_opts)?.unwrap();
        mcu_pwm.setup_cycle_time(pwm_cycle_time, None)?;
        mcu_pwm.setup_max_duration(MAX_HEAT_TIME);
        heater.state.lock().mcu_pwm = Some(mcu_pwm);
        printer.load_object(config, &format!("verify_heater {short_name}"), LookupDefault::Raise)?;
        printer.load_object(config, "pid_calibrate", LookupDefault::Raise)?;
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let heater_cmd = heater.clone();
        gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE",
            "HEATER",
            Some(&short_name),
            Box::new(move |gcmd: &GCmd| heater_cmd.cmd_set_heater_temperature(gcmd)),
            Some(Self::SET_HEATER_TEMPERATURE_HELP),
        )?;
        let heater_shutdown = heater.clone();
        printer.register_event_handler(
            EV_KLIPPY_SHUTDOWN,
            Box::new(move |_params: &[EventArg]| {
                heater_shutdown.handle_shutdown();
                Ok(PyValue::None)
            }),
        );
        Ok(heater)
    }

    /// `Heater._handle_shutdown()` (`heaters.py:98`).
    pub fn handle_shutdown(&self) {
        self.state.lock().verify_mainthread_time = -999.;
    }

    /// Help for `SET_HEATER_TEMPERATURE` (`heaters.py:155`).
    pub const SET_HEATER_TEMPERATURE_HELP: &'static str = "Sets a heater temperature";

    pub fn cmd_set_heater_temperature(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let mut opts = GcodeGetFloatOpts::default();
        opts.default = Some(Some(0.));
        let temp = gcmd.get_float("TARGET", opts)?.unwrap();
        let obj = self
            .printer
            .lookup_object("heaters", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let pheaters = klippy_api::require_interface::<dyn PrinterHeatersApi>(&obj, "heaters")?;
        pheaters.set_temperature(self, temp, false)
    }
}

impl SensorApi for Heater {
    fn get_temp(&self, eventtime: f64) -> (f64, f64) {
        HeaterApi::get_temp(self, eventtime)
    }
}

impl HeaterApi for Heater {
    fn get_name(&self) -> String {
        self.name.clone()
    }

    fn get_pwm_delay(&self) -> f64 {
        self.state.lock().pwm_delay
    }

    fn get_max_power(&self) -> f64 {
        self.state.lock().max_power
    }

    fn get_smooth_time(&self) -> f64 {
        self.state.lock().smooth_time
    }

    fn set_pwm(&self, read_time: f64, value: f64) -> KResult<()> {
        let mut value = value;
        let (pwm_time, mcu_pwm) = {
            let mut st = self.state.lock();
            if st.target_temp <= 0. || read_time > st.verify_mainthread_time {
                value = 0.;
            }
            if (read_time < st.next_pwm_time || st.last_pwm_value == 0.)
                && f64::abs(value - st.last_pwm_value) < st.min_pwm_change
            {
                return Ok(());
            }
            let pwm_time = read_time + st.pwm_delay;
            st.next_pwm_time = pwm_time + MAX_HEAT_TIME - (3. * st.pwm_delay + 0.001);
            st.last_pwm_value = value;
            let mcu_pwm = st.mcu_pwm.clone().expect("heater pin is set");
            (pwm_time, mcu_pwm)
        };
        mcu_pwm.set_pwm(pwm_time, value)
    }

    fn temperature_callback(&self, read_time: f64, temp: f64) -> KResult<()> {
        let _control_guard = self.control_lock.lock();
        let time_diff = {
            let mut st = self.state.lock();
            let time_diff = read_time - st.last_temp_time;
            st.last_temp = temp;
            st.last_temp_time = read_time;
            time_diff
        };
        let (control, target_temp) = {
            let st = self.state.lock();
            (
                st.control.clone().expect("heater control is set"),
                st.target_temp,
            )
        };
        control.temperature_update(read_time, temp, target_temp)?;
        let mut st = self.state.lock();
        let temp_diff = temp - st.smoothed_temp;
        let adj_time = f64::min(time_diff * st.inv_smooth_time, 1.);
        st.smoothed_temp += temp_diff * adj_time;
        st.can_extrude = st.smoothed_temp >= st.min_extrude_temp;
        Ok(())
    }

    fn set_temp(&self, degrees: f64) -> KResult<()> {
        let _control_guard = self.control_lock.lock();
        if degrees != 0. {
            let st = self.state.lock();
            if degrees < st.min_temp || degrees > st.max_temp {
                let msg = percent(
                    "Requested temperature (%.1f) out of range (%.1f:%.1f)",
                    &[
                        PctValue::Float(degrees),
                        PctValue::Float(st.min_temp),
                        PctValue::Float(st.max_temp),
                    ],
                );
                return Err(KlippyError::command(msg));
            }
        }
        self.state.lock().target_temp = degrees;
        Ok(())
    }

    fn get_temp(&self, eventtime: f64) -> (f64, f64) {
        let mcu_pwm = self.mcu_pwm_handle();
        let est = mcu_pwm.get_mcu().estimated_print_time(eventtime);
        let quell_time = est - QUELL_STALE_TIME;
        let _control_guard = self.control_lock.lock();
        let st = self.state.lock();
        if st.last_temp_time < quell_time {
            (0., st.target_temp)
        } else {
            (st.smoothed_temp, st.target_temp)
        }
    }

    fn check_busy(&self, eventtime: f64) -> bool {
        let _control_guard = self.control_lock.lock();
        let control = self.control_arc();
        let (smoothed, target) = {
            let st = self.state.lock();
            (st.smoothed_temp, st.target_temp)
        };
        control.check_busy(eventtime, smoothed, target)
    }

    fn set_control(&self, control: Arc<dyn ControlApi>) -> Arc<dyn ControlApi> {
        let _control_guard = self.control_lock.lock();
        let mut st = self.state.lock();
        let old = st.control.clone().expect("heater control is set");
        st.control = Some(control);
        st.target_temp = 0.;
        old
    }

    fn alter_target(&self, target_temp: f64) {
        let target_temp = if target_temp != 0. {
            let st = self.state.lock();
            let t = target_temp;
            f64::max(st.min_temp, f64::min(st.max_temp, t))
        } else {
            target_temp
        };
        self.state.lock().target_temp = target_temp;
    }

    fn stats(&self, eventtime: f64) -> (bool, String) {
        let mcu_pwm = self.mcu_pwm_handle();
        let est = mcu_pwm.get_mcu().estimated_print_time(eventtime);
        if !self.printer.is_shutdown() {
            self.state.lock().verify_mainthread_time = est + MAX_MAINTHREAD_TIME;
        }
        let _control_guard = self.control_lock.lock();
        let (target_temp, last_temp, last_pwm_value) = {
            let st = self.state.lock();
            (st.target_temp, st.last_temp, st.last_pwm_value)
        };
        let is_active = target_temp != 0. || last_temp > 50.;
        let msg = percent(
            "%s: target=%.0f temp=%.1f pwm=%.3f",
            &[
                PctValue::Str(self.short_name.clone()),
                PctValue::Float(target_temp),
                PctValue::Float(last_temp),
                PctValue::Float(last_pwm_value),
            ],
        );
        (is_active, msg)
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let _control_guard = self.control_lock.lock();
        let (target_temp, smoothed_temp, last_pwm_value) = {
            let st = self.state.lock();
            (st.target_temp, st.smoothed_temp, st.last_pwm_value)
        };
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(smoothed_temp, 2)),
        );
        map.insert(PyValue::Str("target".to_string()), PyValue::Float(target_temp));
        map.insert(PyValue::Str("power".to_string()), PyValue::Float(last_pwm_value));
        PyValue::Dict(map)
    }

    fn min_temp(&self) -> f64 {
        self.state.lock().min_temp
    }

    fn max_temp(&self) -> f64 {
        self.state.lock().max_temp
    }

    fn min_extrude_temp(&self) -> f64 {
        self.state.lock().min_extrude_temp
    }

    fn can_extrude(&self) -> bool {
        self.state.lock().can_extrude
    }

    fn target_temp(&self) -> f64 {
        self.state.lock().target_temp
    }
}

impl PrinterObject for Heater {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn HeaterApi, dyn SensorApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(HeaterApi::get_status(self, eventtime)))
    }

    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(HeaterApi::stats(self, eventtime))
    }
}

// PrinterHeaters (`heaters.py:242`).

struct PrinterHeatersState {
    sensor_factories: IndexMap<String, SensorFactory>,
    heaters: IndexMap<String, Arc<Heater>>,
    gcode_id_to_sensor: IndexMap<String, Arc<dyn SensorApi>>,
    available_heaters: Vec<String>,
    available_sensors: Vec<String>,
    available_monitors: Vec<String>,
    has_started: bool,
    have_load_sensors: bool,
}

/// `PrinterHeaters` (`heaters.py:242`).
pub struct PrinterHeaters {
    printer: Printer,
    inner: GCell<PrinterHeatersState>,
    self_ref: Weak<PrinterHeaters>,
}

impl PrinterHeaters {
    fn own_arc(&self) -> Arc<PrinterHeaters> {
        self.self_ref.upgrade().expect("PrinterHeaters is alive while its methods run")
    }

    /// `PrinterHeaters.__init__(config)` (`heaters.py:243`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let heaters = Arc::new_cyclic(|weak| PrinterHeaters {
            printer: printer.clone(),
            inner: GCell::new(PrinterHeatersState {
                sensor_factories: IndexMap::new(),
                heaters: IndexMap::new(),
                gcode_id_to_sensor: IndexMap::new(),
                available_heaters: Vec::new(),
                available_sensors: Vec::new(),
                available_monitors: Vec::new(),
                has_started: false,
                have_load_sensors: false,
            }),
            self_ref: weak.clone(),
        });
        let ready = heaters.clone();
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params: &[EventArg]| {
                ready.handle_ready();
                Ok(PyValue::None)
            }),
        );
        let restart = heaters.clone();
        printer.register_event_handler(
            EV_GCODE_REQUEST_RESTART,
            Box::new(move |_params: &[EventArg]| {
                restart.turn_off_all_heaters(0.);
                Ok(PyValue::None)
            }),
        );
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let off = heaters.clone();
        gcode.register_command(
            "TURN_OFF_HEATERS",
            Some(Box::new(move |gcmd: &GCmd| off.cmd_turn_off_heaters(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(Self::TURN_OFF_HEATERS_HELP.to_string()),
            },
        )?;
        let m105 = heaters.clone();
        gcode.register_command(
            "M105",
            Some(Box::new(move |gcmd: &GCmd| m105.cmd_m105(gcmd))),
            RegisterCommandOpts {
                when_not_ready: true,
                desc: None,
            },
        )?;
        Ok(heaters)
    }

    /// Help for `TURN_OFF_HEATERS` (`heaters.py:325`).
    pub const TURN_OFF_HEATERS_HELP: &'static str = "Turn off all heaters";
    /// Help for `TEMPERATURE_WAIT` (`heaters.py:366`).
    pub const TEMPERATURE_WAIT_HELP: &'static str = "Wait for a temperature on a sensor";

    /// `PrinterHeaters._handle_ready()` (`heaters.py:329`).
    pub fn handle_ready(&self) {
        self.inner.borrow_mut().has_started = true;
    }

    /// `PrinterHeaters.load_config(config)` (`heaters.py:260`).
    pub fn load_config(&self, _config: &Config) -> KResult<()> {
        self.inner.borrow_mut().have_load_sensors = true;
        let pconfig = lookup_interface::<dyn PrinterConfigApi>(
            self.printer.as_ref(),
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise yields Some");
        // `os.path.join(os.path.dirname(__file__), 'temperature_sensors.cfg')`
        let filename = klippy_api::paths::klippy_path(&["extras", "temperature_sensors.cfg"]);
        let dconfig = match pconfig.read_config(&filename) {
            Ok(c) => c,
            Err(e) => {
                klippy_api::log::exception(
                    "Unable to load temperature_sensors.cfg",
                    &e.message().to_string(),
                );
                return Err(KlippyError::config(format!("Cannot load config '{filename}'")));
            }
        };
        for c in dconfig.get_prefix_sections("") {
            let section = c.get_name();
            self.printer.load_object(&dconfig, &section, LookupDefault::Raise)?;
        }
        Ok(())
    }

    /// `PrinterHeaters._get_temp(eventtime)` (`heaters.py:331`).
    pub fn get_temp_report(&self, eventtime: f64) -> String {
        let has_started = self.inner.borrow().has_started;
        if !has_started {
            return "T:0".to_string();
        }
        let entries: Vec<(String, Arc<dyn SensorApi>)> = {
            let st = self.inner.borrow();
            let mut v: Vec<(String, Arc<dyn SensorApi>)> = st
                .gcode_id_to_sensor
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect();
            v.sort_by(|a, b| a.0.cmp(&b.0));
            v
        };
        if entries.is_empty() {
            return "T:0".to_string();
        }
        let mut out = Vec::new();
        for (gcode_id, sensor) in entries {
            let (cur, target) = sensor.get_temp(eventtime);
            out.push(percent(
                "%s:%.1f /%.1f",
                &[
                    PctValue::Str(gcode_id),
                    PctValue::Float(cur),
                    PctValue::Float(target),
                ],
            ));
        }
        out.join(" ")
    }

    fn wait_for_temperature(&self, heater: &Arc<Heater>) -> KResult<()> {
        if is_fileoutput(&self.printer) {
            return Ok(());
        }
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            self.printer.as_ref(),
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise yields Some");
        let gcode = lookup_interface::<dyn GCodeApi>(self.printer.as_ref(), "gcode", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let reactor = self.printer.get_reactor();
        let mut eventtime = reactor.monotonic();
        while !self.printer.is_shutdown() && heater.check_busy(eventtime) {
            let _print_time = toolhead.get_last_move_time()?;
            gcode.respond_raw(&self.get_temp_report(eventtime));
            eventtime = reactor.pause(eventtime + 1.);
        }
        Ok(())
    }

    /// `PrinterHeaters.cmd_TURN_OFF_HEATERS` (`heaters.py:326`).
    pub fn cmd_turn_off_heaters(&self, _gcmd: &GCmd) -> KResult<()> {
        self.turn_off_all_heaters(0.);
        Ok(())
    }

    /// `PrinterHeaters.cmd_M105` (`heaters.py:341`).
    pub fn cmd_m105(&self, gcmd: &GCmd) -> KResult<()> {
        let msg = self.get_temp_report(self.printer.get_reactor().monotonic());
        if !gcmd.ack(Some(&msg)) {
            gcmd.respond_raw(&msg);
        }
        Ok(())
    }

    /// `PrinterHeaters.cmd_TEMPERATURE_WAIT` (`heaters.py:367`).
    pub fn cmd_temperature_wait(&self, gcmd: &GCmd) -> KResult<()> {
        let sensor_name = match gcmd.get("SENSOR", GcodeGetOpts::default())? {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let mut min_opts = GcodeGetFloatOpts::default();
        min_opts.default = Some(Some(f64::NEG_INFINITY));
        let min_temp = gcmd.get_float("MINIMUM", min_opts)?.unwrap();
        let mut max_opts = GcodeGetFloatOpts::default();
        max_opts.default = Some(Some(f64::INFINITY));
        max_opts.above = Some(min_temp);
        let max_temp = gcmd.get_float("MAXIMUM", max_opts)?.unwrap();
        if min_temp == f64::NEG_INFINITY && max_temp == f64::INFINITY {
            return Err(KlippyError::command(
                "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM.",
            ));
        }
        if is_fileoutput(&self.printer) {
            return Ok(());
        }
        let sensor: Arc<dyn SensorApi> = {
            let heaters = self.inner.borrow();
            if let Some(h) = heaters.heaters.get(&sensor_name) {
                h.clone()
            } else {
                drop(heaters);
                let obj = self
                    .printer
                    .lookup_object(&sensor_name, LookupDefault::Raise)?
                    .expect("Raise yields Some");
                klippy_api::require_interface::<dyn SensorApi>(&obj, &sensor_name)?
            }
        };
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            self.printer.as_ref(),
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise yields Some");
        let reactor = self.printer.get_reactor();
        let mut eventtime = reactor.monotonic();
        while !self.printer.is_shutdown() {
            let (temp, _target) = sensor.get_temp(eventtime);
            if temp >= min_temp && temp <= max_temp {
                return Ok(());
            }
            let _print_time = toolhead.get_last_move_time()?;
            gcmd.respond_raw(&self.get_temp_report(eventtime));
            eventtime = reactor.pause(eventtime + 1.);
        }
        Ok(())
    }
}

impl PrinterHeatersApi for PrinterHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.inner
            .borrow_mut()
            .sensor_factories
            .insert(sensor_type.to_string(), factory);
    }

    fn setup_heater(&self, config: &Config, gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        let heater_name = config
            .get_name()
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        if self.inner.borrow().heaters.contains_key(&heater_name) {
            return Err(KlippyError::config(format!(
                "Heater {heater_name} already registered"
            )));
        }
        let sensor = self.setup_sensor(config)?;
        let heater = Heater::new(config, sensor)?;
        self.inner.borrow_mut().heaters.insert(heater_name.clone(), heater.clone());
        self.register_sensor(config, heater.clone(), gcode_id)?;
        self.inner.borrow_mut().available_heaters.push(config.get_name());
        Ok(heater)
    }

    fn get_all_heaters(&self) -> Vec<String> {
        self.inner.borrow().available_heaters.clone()
    }

    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        match self.inner.borrow().heaters.get(heater_name) {
            Some(h) => Ok(h.clone()),
            None => Err(KlippyError::config(format!("Unknown heater '{heater_name}'"))),
        }
    }

    fn setup_sensor(&self, config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        let need_load = !self.inner.borrow().have_load_sensors;
        if need_load {
            self.load_config(config)?;
        }
        let sensor_type = config.get("sensor_type", GetStrOpts::default())?.unwrap();
        let factory = match self.inner.borrow().sensor_factories.get(&sensor_type) {
            Some(f) => f.clone(),
            None => {
                return Err(KlippyError::config(format!(
                    "Unknown temperature sensor '{sensor_type}'"
                )));
            }
        };
        factory(config)
    }

    fn register_sensor(
        &self,
        config: &Config,
        psensor: Arc<dyn SensorApi>,
        gcode_id: Option<String>,
    ) -> KResult<()> {
        let sensor_name = config.get_name();
        self.inner.borrow_mut().available_sensors.push(sensor_name.clone());
        let gcode = lookup_interface::<dyn GCodeApi>(self.printer.as_ref(), "gcode", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let owner = self.own_arc();
        gcode.register_mux_command(
            "TEMPERATURE_WAIT",
            "SENSOR",
            Some(&sensor_name),
            Box::new(move |gcmd: &GCmd| owner.cmd_temperature_wait(gcmd)),
            Some(Self::TEMPERATURE_WAIT_HELP),
        )?;
        let gcode_id = match gcode_id {
            Some(id) => Some(id),
            None => config.get(
                "gcode_id",
                GetStrOpts {
                    default: Some(None),
                    note_valid: true,
                },
            )?,
        };
        if let Some(gcode_id) = gcode_id {
            if self.inner.borrow().gcode_id_to_sensor.contains_key(&gcode_id) {
                return Err(KlippyError::config(format!(
                    "G-Code sensor id {gcode_id} already registered"
                )));
            }
            self.inner.borrow_mut().gcode_id_to_sensor.insert(gcode_id, psensor);
        }
        Ok(())
    }

    fn register_monitor(&self, config: &Config) {
        self.inner.borrow_mut().available_monitors.push(config.get_name());
    }

    fn turn_off_all_heaters(&self, _print_time: f64) {
        let heaters: Vec<Arc<Heater>> =
            self.inner.borrow().heaters.values().cloned().collect();
        for heater in heaters {
            let _ = heater.set_temp(0.);
        }
    }

    fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            self.printer.as_ref(),
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise yields Some");
        toolhead.register_lookahead_callback(Box::new(|_pt: f64| Ok(())))?;
        heater.set_temp(temp)?;
        if wait && temp != 0. {
            self.wait_for_temperature(heater)?;
        }
        Ok(())
    }
}

impl PrinterObject for PrinterHeaters {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterHeatersApi);

    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let st = self.inner.borrow();
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("available_heaters".to_string()),
            PyValue::List(st.available_heaters.iter().map(|s| PyValue::Str(s.clone())).collect()),
        );
        map.insert(
            PyValue::Str("available_sensors".to_string()),
            PyValue::List(st.available_sensors.iter().map(|s| PyValue::Str(s.clone())).collect()),
        );
        map.insert(
            PyValue::Str("available_monitors".to_string()),
            PyValue::List(st.available_monitors.iter().map(|s| PyValue::Str(s.clone())).collect()),
        );
        Ok(Some(PyValue::Dict(map)))
    }
}

/// `load_config(config)` (`heaters.py:391`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterHeaters::new(config)? as ObjectRef))
}

/// Module entry point for the `heaters` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
