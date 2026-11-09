//! Port of the `spi_temperature` module (`klippy/extras/spi_temperature.py`).
//!
//! Support for common SPI based thermocouple and RTD temperature sensors:
//! MAX6675, MAX31855, MAX31856 and MAX31865. `load_config` registers the
//! four sensor factories with the `heaters` object; a factory builds a
//! [`SensorBase`] (the `SensorBase` subclass of the same name in Python).
//!
//! The MCU delivers `thermocouple_result` responses from the serial reader
//! thread, so the sensor state lives behind `kos` mutexes. The response
//! handler cannot return an error: where the Python handler raises, the port
//! panics with `Class: message` and the serial dispatcher logs
//! `Exception in serial callback` exactly like `serialhdl.py:65`.

use crate::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};
use crate::heaters::{
    PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{AddConfigCmdOpts, Mcu, SendOpts};
use klippy_api::object::{load_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use kos::sync::Mutex;
use pycompat::text::{float_pow, float_true_div, percent_format, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// SensorBase constants (`spi_temperature.py:15`)
// ---------------------------------------------------------------------------

pub const REPORT_TIME: f64 = 0.300;
pub const MAX_INVALID_COUNT: i64 = 3;

// ---------------------------------------------------------------------------
// MAX31856 thermocouple constants (`spi_temperature.py:73`)
// ---------------------------------------------------------------------------

pub const MAX31856_CR0_REG: i64 = 0x00;
pub const MAX31856_CR0_AUTOCONVERT: i64 = 0x80;
pub const MAX31856_CR0_1SHOT: i64 = 0x40;
pub const MAX31856_CR0_OCFAULT1: i64 = 0x20;
pub const MAX31856_CR0_OCFAULT0: i64 = 0x10;
pub const MAX31856_CR0_CJ: i64 = 0x08;
pub const MAX31856_CR0_FAULT: i64 = 0x04;
pub const MAX31856_CR0_FAULTCLR: i64 = 0x02;
pub const MAX31856_CR0_FILT50HZ: i64 = 0x01;
pub const MAX31856_CR0_FILT60HZ: i64 = 0x00;

pub const MAX31856_CR1_REG: i64 = 0x01;
pub const MAX31856_CR1_AVGSEL1: i64 = 0x00;
pub const MAX31856_CR1_AVGSEL2: i64 = 0x10;
pub const MAX31856_CR1_AVGSEL4: i64 = 0x20;
pub const MAX31856_CR1_AVGSEL8: i64 = 0x30;
pub const MAX31856_CR1_AVGSEL16: i64 = 0x70;

pub const MAX31856_MASK_REG: i64 = 0x02;
pub const MAX31856_MASK_COLD_JUNCTION_HIGH_FAULT: i64 = 0x20;
pub const MAX31856_MASK_COLD_JUNCTION_LOW_FAULT: i64 = 0x10;
pub const MAX31856_MASK_THERMOCOUPLE_HIGH_FAULT: i64 = 0x08;
pub const MAX31856_MASK_THERMOCOUPLE_LOW_FAULT: i64 = 0x04;
pub const MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT: i64 = 0x02;
pub const MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT: i64 = 0x01;

pub const MAX31856_CJHF_REG: i64 = 0x03;
pub const MAX31856_CJLF_REG: i64 = 0x04;
pub const MAX31856_LTHFTH_REG: i64 = 0x05;
pub const MAX31856_LTHFTL_REG: i64 = 0x06;
pub const MAX31856_LTLFTH_REG: i64 = 0x07;
pub const MAX31856_LTLFTL_REG: i64 = 0x08;
pub const MAX31856_CJTO_REG: i64 = 0x09;
pub const MAX31856_CJTH_REG: i64 = 0x0A;
pub const MAX31856_CJTL_REG: i64 = 0x0B;
pub const MAX31856_LTCBH_REG: i64 = 0x0C;
pub const MAX31856_LTCBM_REG: i64 = 0x0D;
pub const MAX31856_LTCBL_REG: i64 = 0x0E;

pub const MAX31856_SR_REG: i64 = 0x0F;
pub const MAX31856_FAULT_CJRANGE: i64 = 0x80;
pub const MAX31856_FAULT_TCRANGE: i64 = 0x40;
pub const MAX31856_FAULT_CJHIGH: i64 = 0x20;
pub const MAX31856_FAULT_CJLOW: i64 = 0x10;
pub const MAX31856_FAULT_TCHIGH: i64 = 0x08;
pub const MAX31856_FAULT_TCLOW: i64 = 0x04;
pub const MAX31856_FAULT_OVUV: i64 = 0x02;
pub const MAX31856_FAULT_OPEN: i64 = 0x01;

pub const MAX31856_SCALE: i64 = 5;
pub const MAX31856_MULT: f64 = 0.0078125;

// ---------------------------------------------------------------------------
// MAX31855 / MAX6675 thermocouple constants (`spi_temperature.py:196,226`)
// ---------------------------------------------------------------------------

pub const MAX31855_SCALE: i64 = 18;
pub const MAX31855_MULT: f64 = 0.25;

pub const MAX6675_SCALE: i64 = 3;
pub const MAX6675_MULT: f64 = 0.25;

// ---------------------------------------------------------------------------
// MAX31865 RTD constants (`spi_temperature.py:254`)
// ---------------------------------------------------------------------------

pub const MAX31865_CONFIG_REG: i64 = 0x00;
pub const MAX31865_RTDMSB_REG: i64 = 0x01;
pub const MAX31865_RTDLSB_REG: i64 = 0x02;
pub const MAX31865_HFAULTMSB_REG: i64 = 0x03;
pub const MAX31865_HFAULTLSB_REG: i64 = 0x04;
pub const MAX31865_LFAULTMSB_REG: i64 = 0x05;
pub const MAX31865_LFAULTLSB_REG: i64 = 0x06;
pub const MAX31865_FAULTSTAT_REG: i64 = 0x07;

pub const MAX31865_CONFIG_BIAS: i64 = 0x80;
pub const MAX31865_CONFIG_MODEAUTO: i64 = 0x40;
pub const MAX31865_CONFIG_1SHOT: i64 = 0x20;
pub const MAX31865_CONFIG_3WIRE: i64 = 0x10;
pub const MAX31865_CONFIG_FAULTCLEAR: i64 = 0x02;
pub const MAX31865_CONFIG_FILT50HZ: i64 = 0x01;

pub const MAX31865_FAULT_HIGHTHRESH: i64 = 0x80;
pub const MAX31865_FAULT_LOWTHRESH: i64 = 0x40;
pub const MAX31865_FAULT_REFINLOW: i64 = 0x20;
pub const MAX31865_FAULT_REFINHIGH: i64 = 0x10;
pub const MAX31865_FAULT_RTDINLOW: i64 = 0x08;
pub const MAX31865_FAULT_OVUV: i64 = 0x04;

pub const MAX31865_ADC_MAX: i64 = 1 << 15;

/// Callendar-Van Dusen constants for platinum resistance thermometers.
pub const CVD_A: f64 = 3.9083e-3;
pub const CVD_B: f64 = -5.775e-7;

/// Response format registered for every sensor (`spi_temperature.py:34`).
const THERMOCOUPLE_RESULT_MSG: &str =
    "thermocouple_result oid=%c next_clock=%u value=%u fault=%c";

// ---------------------------------------------------------------------------
// Python semantics helpers
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

fn fmt(format: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(format, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

/// A Python exception raised by the serial response handler. The caller's
/// panic guard (the serial dispatcher) reports it like `logging.exception`.
fn raise(err: KlippyError) -> ! {
    err.raise()
}

/// `int(x)` of a float (`Objects/floatobject.c:float___trunc___impl`).
fn py_int_of_float(x: f64) -> KResult<PyInt> {
    if x.is_nan() {
        return Err(internal("ValueError", "cannot convert float NaN to integer".to_string()));
    }
    if x.is_infinite() {
        return Err(internal(
            "OverflowError",
            "cannot convert float infinity to integer".to_string(),
        ));
    }
    Ok(trunc_float_to_bigint(x))
}

/// `max(0, min(upper, adc))` for a Python int.
fn clamp_adc(adc: &PyInt, upper: i64) -> i64 {
    match adc.inner().to_string().parse::<i64>() {
        Ok(v) => v.clamp(0, upper),
        Err(_) => {
            if adc.sign() > 0 {
                upper
            } else {
                0
            }
        }
    }
}

/// `int(raw / mult + 0.5)` clamped to `0..=upper`, the shared body of the
/// thermocouple `calc_adc` methods.
fn thermocouple_adc(temp: f64, mult: f64, upper: i64) -> KResult<i64> {
    let adc = py_int_of_float((temp / mult) + 0.5)?;
    Ok(clamp_adc(&adc, upper))
}

/// Reads `params[key]` of a `thermocouple_result` response as an int.
fn response_int(params: &PyValue, key: &str) -> KResult<i64> {
    let PyValue::Dict(entries) = params else {
        return Err(internal(
            "TypeError",
            format!("'{}' object is not subscriptable", pycompat::value::type_name(params)),
        ));
    };
    let wanted = PyValue::Str(key.to_string());
    let value = entries
        .iter()
        .find(|(k, _)| pycompat::value::py_equal(k, &wanted))
        .map(|(_, v)| v)
        .ok_or_else(|| internal("KeyError", pycompat::value::repr_key(key)))?;
    match value {
        PyValue::Int(n) => n.inner().to_string().parse::<i64>().map_err(|_| {
            internal("OverflowError", format!("int too large to convert to i64: {}", value.str()))
        }),
        PyValue::Bool(b) => Ok(i64::from(*b)),
        other => Err(internal(
            "TypeError",
            format!("unsupported operand type(s): '{}'", pycompat::value::type_name(other)),
        )),
    }
}

/// An int from the dict `getchoice` returns.
fn choice_int(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => n
            .inner()
            .to_string()
            .parse::<i64>()
            .map_err(|_| internal("OverflowError", "int too large to convert to i64".to_string())),
        other => Err(internal(
            "TypeError",
            format!("expected int from getchoice, got {}", pycompat::value::type_name(other)),
        )),
    }
}

// ---------------------------------------------------------------------------
// Chip specific behaviour
// ---------------------------------------------------------------------------

/// The chip specific half of the Python classes: `handle_fault`, `calc_temp`
/// and `calc_adc` differ per subclass of `SensorBase`.
enum Chip {
    Max31856,
    Max31855,
    Max6675,
    Max31865 {
        adc_to_resist_div_nominal: f64,
        config_reg: Vec<u8>,
    },
}

/// Mutable sensor state shared with the serial reader thread.
struct SensorState {
    min_sample_value: i64,
    max_sample_value: i64,
    report_clock: i64,
}

/// `SensorBase` (`spi_temperature.py:18`) together with the chip subclass
/// selected at construction: [`SensorBase::max6675`],
/// [`SensorBase::max31855`], [`SensorBase::max31856`] and
/// [`SensorBase::max31865`] mirror the Python class constructors.
pub struct SensorBase {
    chip: Chip,
    chip_type: &'static str,
    mcu: Mcu,
    spi: Arc<McuSpi>,
    oid: i64,
    state: Mutex<SensorState>,
    callback: Mutex<Option<TemperatureCallback>>,
}

impl SensorBase {
    /// `SensorBase.__init__(config, chip_type, config_cmd, spi_mode)`
    /// (`spi_temperature.py:19`).
    fn new(
        config: &Config,
        chip: Chip,
        chip_type: &'static str,
        config_cmd: Option<&[u8]>,
        spi_mode: i64,
    ) -> KResult<Arc<SensorBase>> {
        let spi = mcu_spi_from_config(config, spi_mode, "sensor_pin", 4000000, None, false)?;
        if let Some(config_cmd) = config_cmd {
            spi.spi_send(config_cmd, SendOpts::default())?;
        }
        let mcu = spi.get_mcu();
        // Reader chip configuration
        let oid = mcu.create_oid()?;
        let sensor = Arc::new(SensorBase {
            chip,
            chip_type,
            mcu: mcu.clone(),
            spi,
            oid,
            state: Mutex::new(SensorState {
                min_sample_value: 0,
                max_sample_value: 0,
                report_clock: 0,
            }),
            callback: Mutex::new(None),
        });
        let handler = Arc::clone(&sensor);
        mcu.register_serial_response(
            Arc::new(move |params: PyValue| handler.handle_spi_response(&params)),
            THERMOCOUPLE_RESULT_MSG,
            Some(oid),
        );
        let builder = Arc::clone(&sensor);
        mcu.register_config_callback(Box::new(move || builder.build_config()))?;
        Ok(sensor)
    }

    /// `MAX6675(config)` (`spi_temperature.py:230`).
    pub fn max6675(config: &Config) -> KResult<Arc<SensorBase>> {
        SensorBase::new(config, Chip::Max6675, "MAX6675", None, 0)
    }

    /// `MAX31855(config)` (`spi_temperature.py:200`).
    pub fn max31855(config: &Config) -> KResult<Arc<SensorBase>> {
        SensorBase::new(config, Chip::Max31855, "MAX31855", None, 0)
    }

    /// `MAX31856(config)` (`spi_temperature.py:126`).
    pub fn max31856(config: &Config) -> KResult<Arc<SensorBase>> {
        let config_cmd = max31856_build_spi_init(config)?;
        SensorBase::new(config, Chip::Max31856, "MAX31856", Some(&config_cmd), 1)
    }

    /// `MAX31865(config)` (`spi_temperature.py:284`).
    pub fn max31865(config: &Config) -> KResult<Arc<SensorBase>> {
        let rtd_nominal_r = config
            .getfloat(
                "rtd_nominal_r",
                GetFloatOpts { default: Some(Some(100.)), above: Some(0.), ..Default::default() },
            )?
            .expect("default given");
        let rtd_reference_r = config
            .getfloat(
                "rtd_reference_r",
                GetFloatOpts { default: Some(Some(430.)), above: Some(0.), ..Default::default() },
            )?
            .expect("default given");
        let adc_to_resist = rtd_reference_r / (MAX31865_ADC_MAX as f64);
        let adc_to_resist_div_nominal = adc_to_resist / rtd_nominal_r;
        let config_reg = max31865_build_spi_init(config)?;
        let chip = Chip::Max31865 { adc_to_resist_div_nominal, config_reg: config_reg.clone() };
        SensorBase::new(config, chip, "MAX31865", Some(&config_reg), 1)
    }

    /// `sensor.chip_type`.
    pub fn chip_type(&self) -> &'static str {
        self.chip_type
    }

    /// `sensor.oid`.
    pub fn oid(&self) -> i64 {
        self.oid
    }

    /// `sensor.min_sample_value`.
    pub fn min_sample_value(&self) -> i64 {
        self.state.lock().min_sample_value
    }

    /// `sensor.max_sample_value`.
    pub fn max_sample_value(&self) -> i64 {
        self.state.lock().max_sample_value
    }

    /// `SensorBase._build_config` (`spi_temperature.py:44`).
    fn build_config(&self) -> KResult<()> {
        self.mcu.add_config_cmd(
            &fmt(
                "config_thermocouple oid=%u spi_oid=%u thermocouple_type=%s",
                &[
                    int(self.oid),
                    int(McuSpiApi::get_oid(&*self.spi)),
                    PctValue::Str(self.chip_type.to_string()),
                ],
            )?,
            AddConfigCmdOpts::default(),
        )?;
        let clock = self.mcu.get_query_slot(self.oid);
        let report_clock = self.mcu.seconds_to_clock(REPORT_TIME);
        let (min_value, max_value) = {
            let mut state = self.state.lock();
            state.report_clock = report_clock;
            (state.min_sample_value, state.max_sample_value)
        };
        self.mcu.add_config_cmd(
            &fmt(
                concat!(
                    "query_thermocouple oid=%u clock=%u rest_ticks=%u",
                    " min_value=%u max_value=%u max_invalid_count=%u"
                ),
                &[
                    int(self.oid),
                    int(clock),
                    int(report_clock),
                    int(min_value),
                    int(max_value),
                    int(MAX_INVALID_COUNT),
                ],
            )?,
            AddConfigCmdOpts { is_init: true, on_restart: false },
        )
    }

    /// `SensorBase._handle_spi_response` (`spi_temperature.py:56`).
    fn handle_spi_response(&self, params: &PyValue) {
        if let Err(err) = self.try_handle_spi_response(params) {
            raise(err);
        }
    }

    fn try_handle_spi_response(&self, params: &PyValue) -> KResult<()> {
        let fault = response_int(params, "fault")?;
        if fault != 0 {
            let value = response_int(params, "value")?;
            return self.handle_fault(value, fault);
        }
        let temp = self.calc_temp(response_int(params, "value")?)?;
        let next_clock = self.mcu.clock32_to_clock64(response_int(params, "next_clock")?);
        let report_clock = self.state.lock().report_clock;
        let last_read_clock = next_clock - report_clock;
        let last_read_time = self.mcu.clock_to_print_time(last_read_clock as f64);
        let mut callback = self.callback.lock();
        match callback.as_mut() {
            Some(callback) => {
                callback(last_read_time, temp);
                Ok(())
            }
            None => Err(internal("TypeError", "'NoneType' object is not callable".to_string())),
        }
    }

    /// `SensorBase.report_fault` (`spi_temperature.py:65`).
    fn report_fault(&self, msg: &str) {
        log::warning(msg);
    }

    /// `handle_fault(adc, fault)` of the chip class.
    pub fn handle_fault(&self, _adc: i64, fault: i64) -> KResult<()> {
        match &self.chip {
            Chip::Max31856 => {
                if fault & MAX31856_FAULT_CJRANGE != 0 {
                    self.report_fault("Max31856: Cold Junction Range Fault");
                }
                if fault & MAX31856_FAULT_TCRANGE != 0 {
                    self.report_fault("Max31856: Thermocouple Range Fault");
                }
                if fault & MAX31856_FAULT_CJHIGH != 0 {
                    self.report_fault("Max31856: Cold Junction High Fault");
                }
                if fault & MAX31856_FAULT_CJLOW != 0 {
                    self.report_fault("Max31856: Cold Junction Low Fault");
                }
                if fault & MAX31856_FAULT_TCHIGH != 0 {
                    self.report_fault("Max31856: Thermocouple High Fault");
                }
                if fault & MAX31856_FAULT_TCLOW != 0 {
                    self.report_fault("Max31856: Thermocouple Low Fault");
                }
                if fault & MAX31856_FAULT_OVUV != 0 {
                    self.report_fault("Max31856: Over/Under Voltage Fault");
                }
                if fault & MAX31856_FAULT_OPEN != 0 {
                    self.report_fault("Max31856: Thermocouple Open Fault");
                }
                Ok(())
            }
            Chip::Max31855 => {
                if fault & 0x1 != 0 {
                    self.report_fault("MAX31855 : Open Circuit");
                }
                if fault & 0x2 != 0 {
                    self.report_fault("MAX31855 : Short to GND");
                }
                if fault & 0x4 != 0 {
                    self.report_fault("MAX31855 : Short to Vcc");
                }
                Ok(())
            }
            Chip::Max6675 => {
                if fault & 0x02 != 0 {
                    self.report_fault("Max6675 : Device ID error");
                }
                if fault & 0x04 != 0 {
                    self.report_fault("Max6675 : Thermocouple Open Fault");
                }
                Ok(())
            }
            Chip::Max31865 { config_reg, .. } => {
                if fault & 0x80 != 0 {
                    self.report_fault("Max31865 RTD input is disconnected");
                }
                if fault & 0x40 != 0 {
                    self.report_fault("Max31865 RTD input is shorted");
                }
                if fault & 0x20 != 0 {
                    self.report_fault(
                        "Max31865 VREF- is greater than 0.85 * VBIAS, FORCE- open",
                    );
                }
                if fault & 0x10 != 0 {
                    self.report_fault("Max31865 VREF- is less than 0.85 * VBIAS, FORCE- open");
                }
                if fault & 0x08 != 0 {
                    self.report_fault("Max31865 VRTD- is less than 0.85 * VBIAS, FORCE- open");
                }
                if fault & 0x04 != 0 {
                    self.report_fault("Max31865 Overvoltage or undervoltage fault");
                }
                if fault & 0xfc == 0 {
                    self.report_fault("Max31865 Unspecified error");
                }
                // Attempt to clear the fault
                self.spi.spi_send(config_reg, SendOpts::default())
            }
        }
    }

    /// `calc_temp(adc)` of the chip class.
    pub fn calc_temp(&self, adc: i64) -> KResult<f64> {
        match &self.chip {
            Chip::Max31856 => Ok(MAX31856_MULT * (signed_adc(adc, MAX31856_SCALE, 0x40000, 0x3FFFF) as f64)),
            Chip::Max31855 => Ok(MAX31855_MULT * (signed_adc(adc, MAX31855_SCALE, 0x2000, 0x1FFF) as f64)),
            Chip::Max6675 => Ok(MAX6675_MULT * (signed_adc(adc, MAX6675_SCALE, 0x2000, 0x1FFF) as f64)),
            Chip::Max31865 { adc_to_resist_div_nominal, .. } => {
                let adc = adc >> 1; // remove fault bit
                let r_div_nominal = (adc as f64) * adc_to_resist_div_nominal;
                // Resistance (relative to rtd_nominal_r) is calculated using:
                //  R_div_nominal = 1. + CVD_A * temp + CVD_B * temp**2
                // Solve for temp using quadratic equation:
                //  temp = (-b +- sqrt(b**2 - 4ac)) / 2a
                let a_squared = float_pow(CVD_A, 2.0)
                    .map_err(|e| internal(e.kind, e.msg))?;
                let radicand = a_squared - 4. * CVD_B * (1. - r_div_nominal);
                if radicand < 0.0 {
                    return Err(internal("ValueError", "math domain error".to_string()));
                }
                let discriminant = radicand.sqrt();
                Ok((-CVD_A + discriminant) / (2. * CVD_B))
            }
        }
    }

    /// `calc_adc(temp)` of the chip class.
    pub fn calc_adc(&self, temp: f64) -> KResult<i64> {
        match &self.chip {
            Chip::Max31856 => Ok(thermocouple_adc(temp, MAX31856_MULT, 0x3FFFF)? << MAX31856_SCALE),
            Chip::Max31855 => Ok(thermocouple_adc(temp, MAX31855_MULT, 0x1FFF)? << MAX31855_SCALE),
            Chip::Max6675 => Ok(thermocouple_adc(temp, MAX6675_MULT, 0x1FFF)? << MAX6675_SCALE),
            Chip::Max31865 { adc_to_resist_div_nominal, .. } => {
                // Calculate relative resistance via Callendar-Van Dusen formula:
                //  resistance = rtd_nominal_r * (1 + CVD_A * temp + CVD_B * temp**2)
                // Melting point of platinum; `min(temp, 1768.3)` keeps a NaN.
                let temp = if 1768.3 < temp { 1768.3 } else { temp };
                let r_div_nominal = 1. + CVD_A * temp + CVD_B * temp * temp;
                let quotient = float_true_div(r_div_nominal, *adc_to_resist_div_nominal)
                    .map_err(|e| internal(e.kind, e.msg))?;
                let adc = py_int_of_float(quotient + 0.5)?;
                let adc = clamp_adc(&adc, MAX31865_ADC_MAX - 1);
                Ok(adc << 1) // Add fault bit
            }
        }
    }
}

/// `adc >> scale`, then the two's complement sign fix shared by the
/// thermocouple `calc_temp` methods.
fn signed_adc(adc: i64, scale: i64, sign_bit: i64, mask: i64) -> i64 {
    let mut adc = adc >> scale;
    // Fix sign bit:
    if adc & sign_bit != 0 {
        adc = -((adc & mask) + 1);
    }
    adc
}

impl TemperatureSensorApi for SensorBase {
    /// `SensorBase.setup_minmax` (`spi_temperature.py:36`): an exception
    /// from `calc_adc` (`int(nan)`, `int(inf)`) propagates to the caller.
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        let min_adc = self.calc_adc(min_temp)?;
        let max_adc = self.calc_adc(max_temp)?;
        let mut state = self.state.lock();
        state.min_sample_value = min_adc.min(max_adc);
        state.max_sample_value = min_adc.max(max_adc);
        Ok(())
    }

    /// `SensorBase.setup_callback` (`spi_temperature.py:40`).
    fn setup_callback(&self, callback: TemperatureCallback) {
        *self.callback.lock() = Some(callback);
    }

    /// `SensorBase.get_report_time_delta` (`spi_temperature.py:42`).
    fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }
}

// ---------------------------------------------------------------------------
// Chip configuration
// ---------------------------------------------------------------------------

/// `MAX31856.build_spi_init` (`spi_temperature.py:157`).
fn max31856_build_spi_init(config: &Config) -> KResult<Vec<u8>> {
    let mut cmds: Vec<i64> = Vec::new();
    let mut value = MAX31856_CR0_AUTOCONVERT;
    if config
        .getboolean(
            "tc_use_50Hz_filter",
            GetBoolOpts { default: Some(Some(false)), note_valid: true, ..Default::default() },
        )?
        .expect("default given")
    {
        value |= MAX31856_CR0_FILT50HZ;
    }
    cmds.push(0x80 + MAX31856_CR0_REG);
    cmds.push(value);

    let mut types: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (name, bits) in [
        ("B", 0b0000),
        ("E", 0b0001),
        ("J", 0b0010),
        ("K", 0b0011),
        ("N", 0b0100),
        ("R", 0b0101),
        ("S", 0b0110),
        ("T", 0b0111),
    ] {
        types.insert(PyValue::Str(name.to_string()), PyValue::Int(PyInt::from_i64(bits)));
    }
    let mut value = choice_int(&config.getchoice(
        "tc_type",
        &PyValue::Dict(types),
        GetChoiceOpts { default: Some(Some(PyValue::Str("K".to_string()))), note_valid: true },
    )?)?;
    let mut averages: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (count, bits) in [
        (1, MAX31856_CR1_AVGSEL1),
        (2, MAX31856_CR1_AVGSEL2),
        (4, MAX31856_CR1_AVGSEL4),
        (8, MAX31856_CR1_AVGSEL8),
        (16, MAX31856_CR1_AVGSEL16),
    ] {
        averages.insert(
            PyValue::Int(PyInt::from_i64(count)),
            PyValue::Int(PyInt::from_i64(bits)),
        );
    }
    value |= choice_int(&config.getchoice(
        "tc_averaging_count",
        &PyValue::Dict(averages),
        GetChoiceOpts { default: Some(Some(PyValue::Int(PyInt::from_i64(1)))), note_valid: true },
    )?)?;
    cmds.push(value);

    cmds.push(MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT | MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT);
    Ok(cmds.into_iter().map(|b| b as u8).collect())
}

/// `MAX31865.build_spi_init` (`spi_temperature.py:330`).
fn max31865_build_spi_init(config: &Config) -> KResult<Vec<u8>> {
    let mut value =
        MAX31865_CONFIG_BIAS | MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_FAULTCLEAR;
    if config
        .getboolean(
            "rtd_use_50Hz_filter",
            GetBoolOpts { default: Some(Some(false)), note_valid: true, ..Default::default() },
        )?
        .expect("default given")
    {
        value |= MAX31865_CONFIG_FILT50HZ;
    }
    if config
        .getint(
            "rtd_num_of_wires",
            GetIntOpts { default: Some(Some(2)), ..Default::default() },
        )?
        .expect("default given")
        == 3
    {
        value |= MAX31865_CONFIG_3WIRE;
    }
    let cmd = 0x80 + MAX31865_CONFIG_REG;
    Ok(vec![cmd as u8, value as u8])
}

// ---------------------------------------------------------------------------
// Sensor registration
// ---------------------------------------------------------------------------

/// `Sensors` (`spi_temperature.py:346`): sensor type name and factory, in
/// the dict's insertion order.
pub fn sensors() -> Vec<(&'static str, SensorFactory)> {
    fn factory(build: fn(&Config) -> KResult<Arc<SensorBase>>) -> SensorFactory {
        Arc::new(move |config: &Config| {
            let sensor: Arc<dyn TemperatureSensorApi> = build(config)?;
            Ok(sensor)
        })
    }
    vec![
        ("MAX6675", factory(SensorBase::max6675)),
        ("MAX31855", factory(SensorBase::max31855)),
        ("MAX31856", factory(SensorBase::max31856)),
        ("MAX31865", factory(SensorBase::max31865)),
    ]
}

/// `load_config(config)` (`spi_temperature.py:353`): registers the sensor
/// factories with `heaters`; the section has no object of its own.
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    // Register sensors
    let printer = config.get_printer();
    let pheaters =
        load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
    for (name, factory) in sensors() {
        pheaters.add_sensor_factory(name, factory);
    }
    Ok(None)
}

/// Module entry points of `spi_temperature.py`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
