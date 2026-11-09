//! Port of the `ads131m0x` module (`klippy/extras/ads131m0x.py`).
//!
//! ADS131M0x ADC support (ADS131M02, ADS131M04), used as a bulk sensor
//! source by `load_cell` and `trigger_analog`. The Python module is a
//! library, not a printer object: it has no `load_config`, so
//! [`module_factory`] is empty. `load_cell` builds a sensor with the
//! constructors in [`ADS131M0X_SENSOR_TYPES`] and then calls the sensor's
//! methods (`add_client`, `get_range`, `get_samples_per_second`,
//! `get_status`, `lookup_sensor_error`, `setup_trigger_analog`,
//! `get_mcu`). [`ADS131M0X`] provides those methods and implements
//! [`TriggerAnalogSensor`] for `trigger_analog`. The SPI handle is the
//! `bus.MCU_SPI` object owned by [`crate::bus`]; the reader/batching
//! wiring uses [`crate::bulk_sensor`].

use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandWrapper, Mcu, PinsApi, QuerySendOpts, SendOpts,
};
use klippy_api::object::lookup_interface;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use pycompat::int::PyInt;
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;

use crate::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};
use crate::trigger_analog::TriggerAnalogSensor;

/// SPI commands (`ads131m0x.py:15-18`).
pub const NULL_CMD: i64 = 0x00;
/// SPI commands.
pub const RESET_CMD: i64 = 0x11;
/// SPI commands.
pub const STANDBY_CMD: i64 = 0x22;
/// SPI commands.
pub const WAKEUP_CMD: i64 = 0x33;
/// `ads131m0x.py:20`.
pub const RESET_ACK: i64 = 0xFF22;
/// Minimum delay after RESET (`ads131m0x.py:22`).
pub const T_REGACQ: f64 = 5e-6;

/// Register addresses (`ads131m0x.py:25-30`).
pub const REG_ID: i64 = 0x00;
/// Register addresses.
pub const REG_STATUS: i64 = 0x01;
/// Register addresses.
pub const REG_MODE: i64 = 0x02;
/// Register addresses.
pub const REG_CLOCK: i64 = 0x03;
/// Register addresses.
pub const REG_GAIN1: i64 = 0x04;
/// Register addresses.
pub const REG_CFG: i64 = 0x06;

/// Sensor configuration parameters (`ads131m0x.py:33-35`).
pub const WORD24_MODE: i64 = 0x100;
/// Sensor configuration parameters.
pub const PWR_MODE: i64 = 0x2;
/// Sensor configuration parameters.
pub const GC_MODE: i64 = 0x100;
/// Status mask ignoring F_RESYNC and DRDYx (`ads131m0x.py:38`).
pub const STATUS_REG_MASK: i64 = 0xBFFC;

/// OSR to CLOCK register value (`ads131m0x.py:41-51`), insertion order kept.
pub const OSR_TO_REG: &[ (i64, i64) ] = &[
    (64, 8),
    (128, 0),
    (256, 1),
    (512, 2),
    (1024, 3),
    (2048, 4),
    (4096, 5),
    (8192, 6),
    (16384, 7),
];
/// Gain to GAIN1 register value (`ads131m0x.py:54-63`).
pub const GAIN_TO_REG: &[ (i64, i64) ] = &[
    (1, 0x00),
    (2, 0x01),
    (4, 0x02),
    (8, 0x03),
    (16, 0x04),
    (32, 0x05),
    (64, 0x06),
    (128, 0x07),
];
/// Global-chop delay to CFG register value (`ads131m0x.py:66-83`).
pub const GC_DLY_TO_REG: &[ (i64, i64) ] = &[
    (2, 0x00),
    (4, 0x01),
    (8, 0x02),
    (16, 0x03),
    (32, 0x04),
    (64, 0x05),
    (128, 0x06),
    (256, 0x07),
    (512, 0x08),
    (1024, 0x09),
    (2048, 0x0a),
    (4096, 0x0b),
    (8192, 0x0c),
    (16384, 0x0d),
    (32768, 0x0e),
    (65536, 0x0f),
];

/// `ads131m0x.py:85`.
pub const UPDATE_INTERVAL: f64 = 0.100;
/// `ads131m0x.py:86`.
pub const MINIMUM_CLOCK_FREQ: i64 = 300000;
/// `ads131m0x.py:87`.
pub const MAXIMUM_CLOCK_FREQ: i64 = 8400000;
/// `ads131m0x.py:88`.
pub const NOMINAL_CLOCK_FREQ: i64 = 8192000;
/// `ads131m0x.py:89`.
pub const MAX_SAMPLE_RATE_DEVIATION: f64 = 0.5;

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_float(v: f64) -> PctValue {
    PctValue::Float(v)
}

fn pct_str(v: &str) -> PctValue {
    PctValue::Str(v.to_string())
}

fn fmt_pct(fmt: &str, args: &[PctValue]) -> String {
    match percent_format(fmt, args) {
        Ok(s) => s,
        Err(e) => format!("{}: {}", e.kind, e.msg),
    }
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

fn attr_error(what: &str) -> KlippyError {
    internal("AttributeError", format!("'NoneType' object has no attribute '{what}'"))
}

/// `params[key]` (`KeyError` when the dict lacks the key).
fn dict_get(params: &PyValue, key: &str) -> KResult<PyValue> {
    let PyValue::Dict(dict) = params else {
        return Err(internal(
            "TypeError",
            format!("'{}' object is not subscriptable", pycompat::value::type_name(params)),
        ));
    };
    dict.get(&PyValue::Str(key.to_string())).cloned().ok_or_else(|| {
        internal("KeyError", pycompat::value::repr_key(key))
    })
}

/// `params['response']` as bytes (`msgproto` yields bytes for `%*s`).
/// The test fake returns the raw response bytes instead of a params
/// dict, so a bare `Bytes` value is also accepted.
fn dict_get_bytes(params: &PyValue, key: &str) -> KResult<Vec<u8>> {
    match params {
        PyValue::Bytes(data) => Ok(data.clone()),
        _ => match dict_get(params, key)? {
            PyValue::Bytes(data) => Ok(data),
            value => Err(internal(
                "TypeError",
                format!(
                    "a bytes-like object is required, not '{}'",
                    pycompat::value::type_name(&value)
                ),
            )),
        },
    }
}

/// The `int` a `getchoice` over an all-`int` mapping returns.
fn choice_int(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => i64::try_from(&n.0).map_err(|_| {
            internal("OverflowError", "Python int too large to convert to C long".to_string())
        }),
        other => Err(internal(
            "TypeError",
            format!(
                "int() argument must be a string, a bytes-like object or a number, not '{}'",
                pycompat::value::type_name(other)
            ),
        )),
    }
}

/// `for ptime, val in samples` over one `(ptime, val)` bulk sample.
fn unpack_sample(sample: &PyValue) -> KResult<(f64, i64)> {
    let PyValue::Tuple(items) = sample else {
        return Err(internal(
            "TypeError",
            format!("cannot unpack non-iterable {} object", pycompat::value::type_name(sample)),
        ));
    };
    if items.len() > 2 {
        return Err(internal("ValueError", "too many values to unpack (expected 2)".to_string()));
    }
    if items.len() < 2 {
        return Err(internal(
            "ValueError",
            format!("not enough values to unpack (expected 2, got {})", items.len()),
        ));
    }
    let PyValue::Float(ptime) = &items[0] else {
        return Err(internal(
            "TypeError",
            format!("sample time must be a float, not '{}'", pycompat::value::type_name(&items[0])),
        ));
    };
    Ok((*ptime, choice_int(&items[1])?))
}

/// Port of `klippy/extras/ads131m0x.py:ADS131M0X` (single class covering
/// `ADS131M02`/`ADS131M04` through the factory functions).
pub struct ADS131M0X {
    printer: Printer,
    name: String,
    sensor_type: String,
    sensor_id: i64,
    num_channels: i64,
    channel: i64,
    clock_freq: f64,
    gain: i64,
    enable_global_chop: bool,
    gc_dly: Option<i64>,
    osr: i64,
    sps: f64,
    spi: Arc<McuSpi>,
    mcu: Mcu,
    oid: i64,
    data_ready_pin: String,
    ffreader: FixedFreqReader,
    batch_bulk: BatchBulkHelper,
    query_cmd: GCell<Option<CommandWrapper>>,
    last_error_count: GCell<i64>,
    consecutive_fails: GCell<i64>,
    sensor_errors: GCell<IndexMap<i64, String>>,
}

impl ADS131M0X {
    /// Port of `ADS131M0X.__init__` (`ads131m0x.py:92-190`).
    pub fn new(config: &Config, sensor_type: &str, num_channels: i64) -> KResult<Arc<ADS131M0X>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        let channel = config
            .getint(
                "adc_channel",
                GetIntOpts {
                    default: Some(Some(0)),
                    minval: Some(0),
                    maxval: Some(num_channels - 1),
                    note_valid: true,
                },
            )?
            .unwrap_or(0);
        let clock_freq = match config.get("pwm_clock", GetStrOpts {
            default: Some(None),
            note_valid: true,
        })? {
            None => config
                .getint(
                    "clock_freq",
                    GetIntOpts {
                        default: None,
                        minval: Some(MINIMUM_CLOCK_FREQ),
                        maxval: Some(MAXIMUM_CLOCK_FREQ),
                        note_valid: true,
                    },
                )?
                .unwrap_or(0) as f64,
            Some(pwm_clock_name) => {
                let pwm_config = config.getsection(&pwm_clock_name.split_whitespace().collect::<Vec<_>>().join(" "));
                let freq = pwm_config.getfloat(
                    "frequency",
                    GetFloatOpts {
                        default: Some(None),
                        minval: Some(MINIMUM_CLOCK_FREQ as f64),
                        maxval: Some(MAXIMUM_CLOCK_FREQ as f64),
                        ints: klippy_api::config::IntArgs::MINVAL | klippy_api::config::IntArgs::MAXVAL,
                        note_valid: false,
                        ..Default::default()
                    },
                )?;
                match freq {
                    None => {
                        return Err(KlippyError::config(fmt_pct(
                            "pwm_clock '%s' must support and specify a 'frequency' parameter",
                            &[pct_str(&pwm_clock_name)],
                        )))
                    }
                    Some(f) => f,
                }
            }
        };
        let gain_keys: Vec<PyValue> = GAIN_TO_REG
            .iter()
            .map(|(k, _)| py_int(*k))
            .collect();
        let gain = choice_int(&config.getchoice(
            "gain",
            &PyValue::List(gain_keys),
            GetChoiceOpts {
                default: Some(Some(py_int(128))),
                note_valid: true,
            },
        )?)?;
        let enable_global_chop = config
            .getboolean(
                "enable_global_chop",
                GetBoolOpts {
                    default: Some(Some(false)),
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(false);
        let gc_dly = if enable_global_chop {
            let keys: Vec<PyValue> = GC_DLY_TO_REG
                .iter()
                .map(|(k, _)| py_int(*k))
                .collect();
            Some(choice_int(&config.getchoice(
                "global_chop_delay",
                &PyValue::List(keys),
                GetChoiceOpts {
                    default: Some(Some(py_int(16))),
                    note_valid: true,
                },
            )?)?)
        } else {
            None
        };
        let config_sample_rate = config
            .getfloat(
                "sample_rate",
                GetFloatOpts {
                    default: Some(Some(500.0)),
                    above: Some(0.0),
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(500.0);
        let mut best_osr: Option<i64> = None;
        let mut best_sps: Option<f64> = None;
        for (osr, _) in OSR_TO_REG {
            let sps = calc_sample_rate(*osr, clock_freq, enable_global_chop, gc_dly);
            if best_sps.is_none()
                || (sps - config_sample_rate).abs() < (best_sps.unwrap() - config_sample_rate).abs()
            {
                best_osr = Some(*osr);
                best_sps = Some(sps);
            }
        }
        let best_sps = best_sps.unwrap();
        if (best_sps - config_sample_rate).abs() >= MAX_SAMPLE_RATE_DEVIATION * config_sample_rate {
            return Err(KlippyError::config(fmt_pct(
                "Requested sample rate %.1f Hz is not available with the configured parameters",
                &[pct_float(config_sample_rate)],
            )));
        }
        log::info(&fmt_pct(
            "%s '%s' configured sample_rate = %.1f SPS",
            &[pct_str(sensor_type), pct_str(&name), pct_float(best_sps)],
        ));
        let spi = mcu_spi_from_config(config, 1, "cs_pin", 4000000, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid()?;
        let drdy_pin = config
            .get("data_ready_pin", GetStrOpts::default())?
            .unwrap_or_default();
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let drdy_params = pins.lookup_pin(&drdy_pin, Default::default())?;
        let data_ready_pin = drdy_params.pin.clone();
        if !Arc::ptr_eq(&drdy_params.chip, &mcu) {
            return Err(KlippyError::config(fmt_pct(
                "%s config error: SPI communication and data_ready_pin must be on the same MCU",
                &[pct_str(sensor_type)],
            )));
        }
        // Clock tracking
        let chip_smooth = best_sps * UPDATE_INTERVAL * 2.0;
        let ffreader = FixedFreqReader::new(mcu.clone(), chip_smooth, "<i")?;
        // Process messages in batches
        let sensor = Arc::new_cyclic(|weak: &Weak<ADS131M0X>| {
            let process = weak.clone();
            let start = weak.clone();
            let finish = weak.clone();
            let batch_bulk = BatchBulkHelper::new(
                printer.clone(),
                Box::new(move |eventtime| match process.upgrade() {
                    Some(sensor) => sensor._process_batch(eventtime),
                    None => Ok(PyValue::None),
                }),
                Some(Box::new(move || match start.upgrade() {
                    Some(sensor) => sensor._start_measurements(),
                    None => Ok(()),
                })),
                Some(Box::new(move || match finish.upgrade() {
                    Some(sensor) => sensor._finish_measurements(),
                    None => Ok(()),
                })),
                UPDATE_INTERVAL,
            );
            ADS131M0X {
                printer: printer.clone(),
                name,
                sensor_type: sensor_type.to_string(),
                sensor_id: 0x20 | num_channels,
                num_channels,
                channel,
                clock_freq,
                gain,
                enable_global_chop,
                gc_dly,
                osr: best_osr.unwrap(),
                sps: best_sps,
                spi,
                mcu: mcu.clone(),
                oid,
                data_ready_pin,
                ffreader,
                batch_bulk,
                query_cmd: GCell::new(None),
                last_error_count: GCell::new(0),
                consecutive_fails: GCell::new(0),
                sensor_errors: GCell::new(IndexMap::new()),
            }
        });
        mcu.add_config_cmd(
            &fmt_pct(
                "config_ads131m0x oid=%d spi_oid=%d channel=%d num_channels=%d data_ready_pin=%s",
                &[
                    pct_int(sensor.oid),
                    pct_int(sensor.spi.get_oid()),
                    pct_int(sensor.channel),
                    pct_int(sensor.num_channels),
                    pct_str(&sensor.data_ready_pin),
                ],
            ),
            AddConfigCmdOpts::default(),
        )?;
        mcu.add_config_cmd(
            &fmt_pct("query_ads131m0x oid=%d rest_ticks=0", &[pct_int(sensor.oid)]),
            AddConfigCmdOpts {
                is_init: false,
                on_restart: true,
            },
        )?;
        let weak = Arc::downgrade(&sensor);
        mcu.register_config_callback(Box::new(move || match weak.upgrade() {
            Some(sensor) => sensor._build_config(),
            None => Ok(()),
        }))?;
        Ok(sensor)
    }

    /// `self.query_ads131m0x_cmd`, raising Python's `AttributeError` for
    /// `method` before `_build_config` ran.
    fn query_cmd(&self, method: &str) -> KResult<CommandWrapper> {
        self.query_cmd.cloned().ok_or_else(|| attr_error(method))
    }

    /// `get_mcu` (`ads131m0x.py:206`).
    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    /// `get_samples_per_second` (`ads131m0x.py:209`).
    pub fn get_samples_per_second(&self) -> f64 {
        self.sps
    }

    /// `get_status` (`ads131m0x.py:212`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d = PyValue::Dict(IndexMap::new());
        if let PyValue::Dict(m) = &mut d {
            m.insert(
                PyValue::Str("errors".to_string()),
                py_int(self.last_error_count.get()),
            );
            m.insert(
                PyValue::Str("overflows".to_string()),
                py_int(self.ffreader.get_last_overflows()),
            );
            m.insert(
                PyValue::Str("sample_rate".to_string()),
                PyValue::Float(self.get_samples_per_second()),
            );
        }
        d
    }

    /// `lookup_sensor_error` (`ads131m0x.py:219`).
    pub fn lookup_sensor_error(&self, error_code: i64) -> String {
        self.sensor_errors
            .cloned()
            .get(&error_code)
            .cloned()
            .unwrap_or_else(|| fmt_pct("Unknown %s error", &[pct_str(&self.sensor_type)]))
    }

    /// `get_range` (`ads131m0x.py:225`): the minimum and maximum value of
    /// the sensor, used to detect if a data value is saturated.
    pub fn get_range(&self) -> (i64, i64) {
        (-0x800000, 0x7FFFFF)
    }

    /// `_calc_sample_rate` (`ads131m0x.py:228`).
    pub fn _calc_sample_rate(&self, osr: i64) -> f64 {
        calc_sample_rate(
            osr,
            self.clock_freq,
            self.enable_global_chop,
            self.gc_dly,
        )
    }

    /// `add_client` (`ads131m0x.py:234`): direct pass through to the
    /// bulk sensor API.
    pub fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch_bulk.add_client(callback)
    }

    /// `_convert_samples` (`ads131m0x.py:239`): converts `(ptime, val)`
    /// in place into `(round(ptime, 6), val, round(val * adc_factor, 9))`
    /// and drops samples whose unsigned top byte is neither `0x00` nor
    /// `0xFF`.
    pub fn _convert_samples(&self, samples: &mut Vec<PyValue>) -> KResult<()> {
        let adc_factor = 1. / (1i64 << 23) as f64;
        let mut count = 0usize;
        for i in 0..samples.len() {
            let (ptime, val) = unpack_sample(&samples[i])?;
            let top_byte = (val >> 24) & 0xFF;
            if top_byte != 0x00 && top_byte != 0xFF {
                self.last_error_count.set(self.last_error_count.get() + 1);
                log::error(&fmt_pct(
                    "'%s' sample error: %s",
                    &[
                        pct_str(&self.name),
                        pct_str(&self.lookup_sensor_error(top_byte)),
                    ],
                ));
                continue;
            }
            samples[count] = PyValue::Tuple(vec![
                PyValue::Float(round_py_nd(ptime, 6)),
                py_int(val),
                PyValue::Float(round_py_nd(val as f64 * adc_factor, 9)),
            ]);
            count += 1;
        }
        samples.truncate(count);
        Ok(())
    }

    /// `_build_config` (`ads131m0x.py:197`). Registered with the MCU at
    /// startup; the reactor wrapper is added by the caller.
    pub fn _build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        let query = self.mcu.lookup_command(
            "query_ads131m0x oid=%c rest_ticks=%u",
            Some(cmdqueue),
        )?;
        *self.query_cmd.borrow_mut() = Some(query);
        self.ffreader.setup_query_command(
            "query_ads131m0x_status oid=%c",
            self.oid,
            Some(cmdqueue),
        )?;
        let mut errors = IndexMap::new();
        if let Some(inner) = self.mcu.get_enumerations().get("ads131m0x_error:") {
            for (k, v) in inner.iter() {
                let _ = errors.insert(*v, k.clone());
            }
        }
        *self.sensor_errors.borrow_mut() = errors;
        Ok(())
    }

    /// `setup_trigger_analog` (`ads131m0x.py:192`).
    pub fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        self.mcu.add_config_cmd(
            &fmt_pct(
                "ads131m0x_attach_trigger_analog oid=%d trigger_analog_oid=%d",
                &[pct_int(self.oid), pct_int(trigger_analog_oid)],
            ),
            AddConfigCmdOpts {
                is_init: true,
                on_restart: false,
            },
        )
    }

    /// `_start_measurements` (`ads131m0x.py:255`). Pauses the reactor.
    pub fn _start_measurements(&self) -> KResult<()> {
        self.last_error_count.set(0);
        self.consecutive_fails.set(0);
        // Be sure to halt bulk reading before resetting
        self.query_cmd("send_wait_ack")?.send_wait_ack(
            vec![py_int(self.oid), py_int(0)],
            SendOpts::default(),
        )?;
        self.reset_chip()?;
        self.setup_chip()?;
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps));
        self.query_cmd("send")?.send(
            vec![py_int(self.oid), py_int(rest_ticks)],
            SendOpts::default(),
        )?;
        log::info(&fmt_pct(
            "%s starting '%s' measurements",
            &[pct_str(&self.sensor_type), pct_str(&self.name)],
        ));
        // Initialize clock tracking
        self.ffreader.note_start()
    }

    /// `_finish_measurements` (`ads131m0x.py:270`). Pauses the reactor.
    pub fn _finish_measurements(&self) -> KResult<()> {
        // Don't use serial connection after shutdown
        if self.printer.is_shutdown() {
            return Ok(());
        }
        // Halt bulk reading
        self.query_cmd("send_wait_ack")?.send_wait_ack(
            vec![py_int(self.oid), py_int(0)],
            SendOpts::default(),
        )?;
        self.ffreader.note_end()?;
        log::info(&fmt_pct(
            "%s finished '%s' measurements",
            &[pct_str(&self.sensor_type), pct_str(&self.name)],
        ));
        Ok(())
    }

    /// `_process_batch` (`ads131m0x.py:280`): the batch callback of the
    /// batch helper. Pauses the reactor.
    pub fn _process_batch(&self, _eventtime: f64) -> KResult<PyValue> {
        let mut samples = self.ffreader.pull_samples()?;
        self._convert_samples(&mut samples)?;
        let mut msg = IndexMap::new();
        msg.insert(PyValue::Str("data".to_string()), PyValue::List(samples));
        msg.insert(
            PyValue::Str("errors".to_string()),
            py_int(self.last_error_count.get()),
        );
        msg.insert(
            PyValue::Str("overflows".to_string()),
            py_int(self.ffreader.get_last_overflows()),
        );
        Ok(PyValue::Dict(msg))
    }

    /// `_convert_to_spi_frame` (`ads131m0x.py:286`).
    pub fn _convert_to_spi_frame(vals_16: &[i64]) -> Vec<u8> {
        let mut result = Vec::new();
        let pad = std::cmp::max(0, 4 - vals_16.len() as i64);
        for val in vals_16.iter().chain(std::iter::repeat_n(&0i64, pad as usize)) {
            result.push(((val & 0xFF00) >> 8) as u8);
            result.push((val & 0xFF) as u8);
            result.push(0x00);
        }
        result
    }

    /// `reset_chip` (`ads131m0x.py:295`).
    pub fn reset_chip(&self) -> KResult<()> {
        let is_batch_mode = self.mcu.is_fileoutput();
        // Read ID register to validate the sensor type (upper byte)
        let id_val = self.read_reg(REG_ID)?;
        if !is_batch_mode && (id_val >> 8) != self.sensor_id {
            return Err(KlippyError::command(fmt_pct(
                "Invalid %s ID register (got 0x%x vs 0x%x).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty chip.",
                &[pct_str(&self.sensor_type), pct_int(id_val), pct_int(self.sensor_id)],
            )));
        }
        // Send RESET command
        self.send_command_16(RESET_CMD, 0)?;
        // Wait for tREGACQ after RESET before reading acknowledgment
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(curtime);
        let minclock = self.mcu.print_time_to_clock(print_time + T_REGACQ);
        let ack = self.send_command_16(NULL_CMD, minclock)?;
        if !is_batch_mode && ack != RESET_ACK {
            return Err(KlippyError::command(fmt_pct(
                "Failed to reset %s (got 0x%x vs 0x%x).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty chip.",
                &[pct_str(&self.sensor_type), pct_int(ack), pct_int(RESET_ACK)],
            )));
        }
        Ok(())
    }

    /// `setup_chip` (`ads131m0x.py:319`).
    pub fn setup_chip(&self) -> KResult<()> {
        let is_batch_mode = self.mcu.is_fileoutput();
        let mode_val = WORD24_MODE;
        self.write_reg(REG_MODE, mode_val)?;
        let actual_mode_val = self.read_reg(REG_MODE)?;
        if !is_batch_mode && actual_mode_val != mode_val {
            return Err(KlippyError::command(fmt_pct(
                "Failed to set MODE register to %x, got %x",
                &[pct_int(mode_val), pct_int(actual_mode_val)],
            )));
        }

        // Configure CLOCK register
        // Bits 11-8: channel mask, bit 5: TBM, bits 4:2: OSR, bits 1:0: PWR
        let osr_code = match OSR_TO_REG.iter().find(|(k, _)| *k == self.osr) {
            Some((_, v)) => *v,
            None => 0,
        };
        let ch_en = 1 << (self.channel + 8);
        let clock_val = (osr_code << 2) | ch_en | PWR_MODE;
        self.write_reg(REG_CLOCK, clock_val)?;
        let actual_clock_val = self.read_reg(REG_CLOCK)?;
        if !is_batch_mode && actual_clock_val != clock_val {
            return Err(KlippyError::command(fmt_pct(
                "Failed to set CLOCK register to %x, got %x",
                &[pct_int(clock_val), pct_int(actual_clock_val)],
            )));
        }

        // Configure GAIN1 register
        let mut gain_val = 0;
        for i in 0..self.num_channels {
            let g = match GAIN_TO_REG.iter().find(|(k, _)| *k == self.gain) {
                Some((_, v)) => *v,
                None => 0,
            };
            gain_val |= g << (i * 4);
        }
        self.write_reg(REG_GAIN1, gain_val)?;
        let actual_gain_val = self.read_reg(REG_GAIN1)?;
        if !is_batch_mode && actual_gain_val != gain_val {
            return Err(KlippyError::command(fmt_pct(
                "Failed to set GAIN1 register to %x, got %x",
                &[pct_int(gain_val), pct_int(actual_gain_val)],
            )));
        }

        // Configure CFG register (global chop settings)
        let cfg_val = if self.enable_global_chop {
            let g = match GC_DLY_TO_REG
                .iter()
                .find(|(k, _)| Some(*k) == self.gc_dly)
            {
                Some((_, v)) => *v,
                None => 0,
            };
            (g << 9) | GC_MODE
        } else {
            0
        };
        self.write_reg(REG_CFG, cfg_val)?;
        let actual_cfg_val = self.read_reg(REG_CFG)?;
        if !is_batch_mode && actual_cfg_val != cfg_val {
            return Err(KlippyError::command(fmt_pct(
                "Failed to set CFG register to %x, got %x",
                &[pct_int(cfg_val), pct_int(actual_cfg_val)],
            )));
        }

        // Validate STATUS register
        let actual_status_val = self.read_reg(REG_STATUS)?;
        if !is_batch_mode
            && (actual_status_val & STATUS_REG_MASK) != WORD24_MODE
        {
            return Err(KlippyError::command(fmt_pct(
                "Invalid STATUS register value %x",
                &[pct_int(actual_status_val)],
            )));
        }
        // NULL_CMD being the last sent command during initialization sequence
        // ensures subsequent data reads by MCU will always read STATUS register
        Ok(())
    }

    /// `read_reg` (`ads131m0x.py:372`).
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        // RREG command: 101a aaaa annn nnnn
        // a[6:0] = register address (bits 13:7), n[6:0] = count-1
        let cmd = 0xA000 | (reg << 7);
        // First transaction: send RREG command as a full-frame SPI transaction
        let cmd_bytes = Self::_convert_to_spi_frame(&[cmd]);
        self.spi.spi_send(&cmd_bytes, SendOpts::default())?;
        // Second transaction: run NULL_CMD and read the register value
        self.send_command_16(NULL_CMD, 0)
    }

    /// `write_reg` (`ads131m0x.py:382`).
    pub fn write_reg(&self, reg: i64, value: i64) -> KResult<()> {
        // WREG command: 011a aaaa annn nnnn
        // a[6:0] = start address (bits 13:7), n[6:0] = count-1
        let cmd = 0x6000 | (reg << 7);
        let data = Self::_convert_to_spi_frame(&[cmd, value]);
        self.spi.spi_send(&data, SendOpts::default())
    }

    /// `send_command_16` (`ads131m0x.py:389`).
    pub fn send_command_16(&self, cmd: i64, minclock: i64) -> KResult<i64> {
        // Send 16-bit command as a full-frame SPI transaction
        let cmd_bytes = Self::_convert_to_spi_frame(&[cmd]);
        let response = self
            .spi
            .spi_transfer(&cmd_bytes, QuerySendOpts { minclock, ..Default::default() })?;
        let resp = dict_get_bytes(&response, "response")?;
        if resp.is_empty() {
            return Ok(0);
        }
        Ok(((resp[0] as i64) << 8) | (resp[1] as i64))
    }
}

fn calc_sample_rate(
    osr: i64,
    clock_freq: f64,
    enable_global_chop: bool,
    gc_dly: Option<i64>,
) -> f64 {
    if enable_global_chop {
        clock_freq / (2. * (gc_dly.unwrap() as f64 + 3. * osr as f64))
    } else {
        clock_freq / (2. * osr as f64)
    }
}

impl TriggerAnalogSensor for ADS131M0X {
    fn get_mcu(&self) -> Mcu {
        ADS131M0X::get_mcu(self)
    }

    fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        ADS131M0X::setup_trigger_analog(self, trigger_analog_oid)
    }

    fn get_samples_per_second(&self) -> f64 {
        ADS131M0X::get_samples_per_second(self)
    }

    fn lookup_sensor_error(&self, error_code: i64) -> KResult<String> {
        Ok(ADS131M0X::lookup_sensor_error(self, error_code))
    }
}

/// `ADS131M02(config)` (`ads131m0x.py:399`).
pub fn ads131m02(config: &Config) -> KResult<Arc<ADS131M0X>> {
    ADS131M0X::new(config, "ADS131M02", 2)
}

/// `ADS131M04(config)` (`ads131m0x.py:402`).
pub fn ads131m04(config: &Config) -> KResult<Arc<ADS131M0X>> {
    ADS131M0X::new(config, "ADS131M04", 4)
}

/// A sensor constructor of [`ADS131M0X_SENSOR_TYPES`].
pub type SensorConstructor = fn(&Config) -> KResult<Arc<ADS131M0X>>;

/// `ADS131M0X_SENSOR_TYPES` (`ads131m0x.py:406`), in the Python dict's
/// order.
pub const ADS131M0X_SENSOR_TYPES: [(&str, SensorConstructor); 2] =
    [("ads131m02", ads131m02), ("ads131m04", ads131m04)];

/// The Python module defines no entry points (library only), so the
/// factory is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
