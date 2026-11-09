//! Port of `klippy/extras/hx71x.py`: the HX711 and HX717 load cell ADCs.
//!
//! The Python module is a library, not a printer object: it has no
//! `load_config`, so [`module_factory`] is empty. `load_cell` and
//! `load_cell_probe` build a sensor with the constructors in
//! [`HX71X_SENSOR_TYPES`] and then call the sensor's methods
//! (`add_client`, `get_range`, `get_samples_per_second`, `get_status`,
//! `lookup_sensor_error`, `setup_trigger_analog`, `get_mcu`).
//! [`HX71xBase`] provides those methods and implements
//! [`TriggerAnalogSensor`] for `trigger_analog`.

use crate::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::trigger_analog::TriggerAnalogSensor;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{AddConfigCmdOpts, CommandWrapper, LookupPinOpts, Mcu, PinsApi, SendOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, GCell};
use pycompat::text::round_py_nd;
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, Weak};

// ---------------------------------------------------------------------------
// Constants (`hx71x.py:12-14`)
// ---------------------------------------------------------------------------

/// `UPDATE_INTERVAL`.
pub const UPDATE_INTERVAL: f64 = 0.10;
/// `SAMPLE_ERROR_DESYNC`.
pub const SAMPLE_ERROR_DESYNC: i64 = -0x80000000;
/// `SAMPLE_ERROR_LONG_READ`.
pub const SAMPLE_ERROR_LONG_READ: i64 = 0x40000000;

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

/// The `int` a `getchoice` over an all-`int` mapping returns.
fn choice_int(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => i64::try_from(&n.0)
            .map_err(|_| internal("OverflowError", "Python int too large to convert to C long".to_string())),
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

// ---------------------------------------------------------------------------
// HX71xBase (`hx71x.py:17-155`)
// ---------------------------------------------------------------------------

/// Port of the `HX71xBase` class.
pub struct HX71xBase {
    printer: Printer,
    /// `self.name`.
    pub name: String,
    /// `self.sensor_type`.
    pub sensor_type: &'static str,
    last_error_count: GCell<i64>,
    consecutive_fails: GCell<i64>,
    /// `self.mcu`.
    pub mcu: Mcu,
    /// `self.oid`.
    pub oid: i64,
    /// `self.dout_pin`.
    pub dout_pin: String,
    /// `self.sclk_pin`.
    pub sclk_pin: String,
    /// `self.sps`.
    pub sps: i64,
    /// `self.gain_channel`.
    pub gain_channel: i64,
    /// `self.ffreader`.
    pub ffreader: FixedFreqReader,
    /// `self.batch_bulk`.
    pub batch_bulk: BatchBulkHelper,
    query_hx71x_cmd: GCell<Option<CommandWrapper>>,
}

/// `{k: v}` mapping with `int` keys handed to `getchoice`.
fn int_choices(options: &[(i64, i64)]) -> PyValue {
    let mut map = IndexMap::new();
    for (k, v) in options {
        map.insert(py_int(*k), py_int(*v));
    }
    PyValue::Dict(map)
}

/// `{k: v}` mapping with `str` keys handed to `getchoice`.
fn str_choices(options: &[(&str, i64)]) -> PyValue {
    let mut map = IndexMap::new();
    for (k, v) in options {
        map.insert(PyValue::Str((*k).to_string()), py_int(*v));
    }
    PyValue::Dict(map)
}

impl HX71xBase {
    /// `HX71xBase.__init__(config, sensor_type, sample_rate_options,
    /// default_sample_rate, gain_options, default_gain)`.
    pub fn new(
        config: &Config,
        sensor_type: &'static str,
        sample_rate_options: &[(i64, i64)],
        default_sample_rate: i64,
        gain_options: &[(&str, i64)],
        default_gain: &str,
    ) -> KResult<Arc<HX71xBase>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .expect("section names are not empty")
            .to_string();
        // Chip options
        let dout_pin_name = config
            .get("dout_pin", GetStrOpts::default())?
            .expect("required option yields a value");
        let sclk_pin_name = config
            .get("sclk_pin", GetStrOpts::default())?
            .expect("required option yields a value");
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let dout_ppin = ppins.lookup_pin(&dout_pin_name, LookupPinOpts::default())?;
        let sclk_ppin = ppins.lookup_pin(&sclk_pin_name, LookupPinOpts::default())?;
        let mcu = Arc::clone(&dout_ppin.chip);
        let oid = mcu.create_oid()?;
        if !Arc::ptr_eq(&sclk_ppin.chip, &mcu) {
            return Err(KlippyError::Config(format!(
                "{name} config error: All pins must be connected to the same MCU"
            )));
        }
        let dout_pin = dout_ppin.pin;
        let sclk_pin = sclk_ppin.pin;
        // Samples per second choices
        let sps = choice_int(&config.getchoice(
            "sample_rate",
            &int_choices(sample_rate_options),
            GetChoiceOpts {
                default: Some(Some(py_int(default_sample_rate))),
                note_valid: true,
            },
        )?)?;
        // gain/channel choices
        let gain_channel = choice_int(&config.getchoice(
            "gain",
            &str_choices(gain_options),
            GetChoiceOpts {
                default: Some(Some(PyValue::Str(default_gain.to_string()))),
                note_valid: true,
            },
        )?)?;
        // Clock tracking
        let chip_smooth = sps as f64 * UPDATE_INTERVAL * 2.0;
        let ffreader = FixedFreqReader::new(Arc::clone(&mcu), chip_smooth, "<i")?;
        // Process messages in batches
        let sensor = Arc::new_cyclic(|weak: &Weak<HX71xBase>| {
            let process = weak.clone();
            let start = weak.clone();
            let finish = weak.clone();
            let batch_bulk = BatchBulkHelper::new(
                Arc::clone(&printer),
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
            HX71xBase {
                printer: Arc::clone(&printer),
                name,
                sensor_type,
                last_error_count: GCell::new(0),
                consecutive_fails: GCell::new(0),
                mcu: Arc::clone(&mcu),
                oid,
                dout_pin,
                sclk_pin,
                sps,
                gain_channel,
                ffreader,
                batch_bulk,
                query_hx71x_cmd: GCell::new(None),
            }
        });
        // Command Configuration
        mcu.add_config_cmd(
            &format!(
                "config_hx71x oid={} gain_channel={} dout_pin={} sclk_pin={}",
                sensor.oid, sensor.gain_channel, sensor.dout_pin, sensor.sclk_pin
            ),
            AddConfigCmdOpts::default(),
        )?;
        mcu.add_config_cmd(
            &format!("query_hx71x oid={} rest_ticks=0", sensor.oid),
            AddConfigCmdOpts { on_restart: true, ..AddConfigCmdOpts::default() },
        )?;
        let weak = Arc::downgrade(&sensor);
        mcu.register_config_callback(Box::new(move || match weak.upgrade() {
            Some(sensor) => sensor._build_config(),
            None => Ok(()),
        }))?;
        Ok(sensor)
    }

    /// `self.last_error_count`.
    pub fn last_error_count(&self) -> i64 {
        self.last_error_count.get()
    }

    /// `self.consecutive_fails`.
    pub fn consecutive_fails(&self) -> i64 {
        self.consecutive_fails.get()
    }

    /// `setup_trigger_analog(trigger_analog_oid)`.
    pub fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        self.mcu.add_config_cmd(
            &format!(
                "hx71x_attach_trigger_analog oid={} trigger_analog_oid={}",
                self.oid, trigger_analog_oid
            ),
            AddConfigCmdOpts { is_init: true, ..AddConfigCmdOpts::default() },
        )
    }

    /// `_build_config`: the MCU config callback.
    pub fn _build_config(&self) -> KResult<()> {
        let cmd_queue = self.mcu.alloc_command_queue();
        let cmd = self
            .mcu
            .lookup_command("query_hx71x oid=%c rest_ticks=%u", Some(cmd_queue))?;
        *self.query_hx71x_cmd.borrow_mut() = Some(cmd);
        self.ffreader
            .setup_query_command("query_hx71x_status oid=%c", self.oid, Some(cmd_queue))
    }

    /// `get_mcu()`.
    pub fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.mcu)
    }

    /// `get_samples_per_second()`.
    pub fn get_samples_per_second(&self) -> i64 {
        self.sps
    }

    /// `get_status(eventtime)`.
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut status = IndexMap::new();
        status.insert(PyValue::Str("errors".to_string()), py_int(self.last_error_count.get()));
        status.insert(
            PyValue::Str("overflows".to_string()),
            py_int(self.ffreader.get_last_overflows()),
        );
        status.insert(
            PyValue::Str("sample_rate".to_string()),
            py_int(self.get_samples_per_second()),
        );
        PyValue::Dict(status)
    }

    /// `lookup_sensor_error(error_code)`.
    pub fn lookup_sensor_error(&self, error_code: i64) -> String {
        format!("Unknown hx71x error {error_code}")
    }

    /// `get_range()`: the minimum and maximum value of the sensor.
    pub fn get_range(&self) -> (i64, i64) {
        (-0x800000, 0x7FFFFF)
    }

    /// `add_client(callback)`: direct pass through to the bulk sensor API.
    pub fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch_bulk.add_client(callback)
    }

    /// `_convert_samples(samples)`: converts `(ptime, val)` in place into
    /// `(round(ptime, 6), val, round(val * adc_factor, 9))` and cuts the
    /// list at the first error sample.
    pub fn _convert_samples(&self, samples: &mut Vec<PyValue>) -> KResult<()> {
        let adc_factor = 1. / (1i64 << 23) as f64;
        let mut count = 0usize;
        for i in 0..samples.len() {
            let (ptime, val) = unpack_sample(&samples[i])?;
            if val == SAMPLE_ERROR_DESYNC || val == SAMPLE_ERROR_LONG_READ {
                self.last_error_count.set(self.last_error_count.get() + 1);
                break; // additional errors are duplicates
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

    /// `self.query_hx71x_cmd`, raising Python's `AttributeError` for
    /// `method` before `_build_config` ran.
    fn query_cmd(&self, method: &str) -> KResult<CommandWrapper> {
        self.query_hx71x_cmd.cloned().ok_or_else(|| {
            internal(
                "AttributeError",
                format!("'NoneType' object has no attribute '{method}'"),
            )
        })
    }

    /// `_start_measurements`: the start callback of the batch helper.
    /// Pauses the reactor.
    pub fn _start_measurements(&self) -> KResult<()> {
        self.consecutive_fails.set(0);
        self.last_error_count.set(0);
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(1. / (10. * self.sps as f64));
        self.query_cmd("send")?
            .send(vec![py_int(self.oid), py_int(rest_ticks)], SendOpts::default())?;
        log::info(&format!("{} starting '{}' measurements", self.sensor_type, self.name));
        // Initialize clock tracking
        self.ffreader.note_start()
    }

    /// `_finish_measurements`: the stop callback of the batch helper.
    /// Pauses the reactor.
    pub fn _finish_measurements(&self) -> KResult<()> {
        // don't use serial connection after shutdown
        if self.printer.is_shutdown() {
            return Ok(());
        }
        // Halt bulk reading
        self.query_cmd("send_wait_ack")?
            .send_wait_ack(vec![py_int(self.oid), py_int(0)], SendOpts::default())?;
        self.ffreader.note_end()?;
        log::info(&format!("{} finished '{}' measurements", self.sensor_type, self.name));
        Ok(())
    }

    /// `_process_batch(eventtime)`: the batch callback of the batch helper.
    /// Pauses the reactor.
    pub fn _process_batch(&self, _eventtime: f64) -> KResult<PyValue> {
        let prev_overflows = self.ffreader.get_last_overflows();
        let prev_error_count = self.last_error_count.get();
        let mut samples = self.ffreader.pull_samples()?;
        self._convert_samples(&mut samples)?;
        let overflows = self.ffreader.get_last_overflows() - prev_overflows;
        let errors = self.last_error_count.get() - prev_error_count;
        if errors > 0 {
            log::error(&format!("{}: Forced sensor restart due to error", self.name));
            self._finish_measurements()?;
            self._start_measurements()?;
        } else if overflows > 0 {
            self.consecutive_fails.set(self.consecutive_fails.get() + 1);
            if self.consecutive_fails.get() > 4 {
                log::error(&format!("{}: Forced sensor restart due to overflows", self.name));
                self._finish_measurements()?;
                self._start_measurements()?;
            }
        } else {
            self.consecutive_fails.set(0);
        }
        let mut msg = IndexMap::new();
        msg.insert(PyValue::Str("data".to_string()), PyValue::List(samples));
        msg.insert(PyValue::Str("errors".to_string()), py_int(self.last_error_count.get()));
        msg.insert(
            PyValue::Str("overflows".to_string()),
            py_int(self.ffreader.get_last_overflows()),
        );
        Ok(PyValue::Dict(msg))
    }
}

impl TriggerAnalogSensor for HX71xBase {
    fn get_mcu(&self) -> Mcu {
        HX71xBase::get_mcu(self)
    }

    fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        HX71xBase::setup_trigger_analog(self, trigger_analog_oid)
    }

    fn get_samples_per_second(&self) -> f64 {
        HX71xBase::get_samples_per_second(self) as f64
    }

    fn lookup_sensor_error(&self, error_code: i64) -> KResult<String> {
        Ok(HX71xBase::lookup_sensor_error(self, error_code))
    }
}

// ---------------------------------------------------------------------------
// Sensor constructors (`hx71x.py:158-178`)
// ---------------------------------------------------------------------------

/// `HX711(config)`.
pub fn HX711(config: &Config) -> KResult<Arc<HX71xBase>> {
    HX71xBase::new(
        config,
        "hx711",
        // HX711 sps options
        &[(80, 80), (10, 10)],
        80,
        // HX711 gain/channel options
        &[("A-128", 1), ("B-32", 2), ("A-64", 3)],
        "A-128",
    )
}

/// `HX717(config)`.
pub fn HX717(config: &Config) -> KResult<Arc<HX71xBase>> {
    HX71xBase::new(
        config,
        "hx717",
        // HX717 sps options
        &[(320, 320), (80, 80), (20, 20), (10, 10)],
        320,
        // HX717 gain/channel options
        &[("A-128", 1), ("B-64", 2), ("A-64", 3), ("B-8", 4)],
        "A-128",
    )
}

/// A sensor constructor of [`HX71X_SENSOR_TYPES`].
pub type SensorConstructor = fn(&Config) -> KResult<Arc<HX71xBase>>;

/// `HX71X_SENSOR_TYPES`, in the Python dict's order.
pub const HX71X_SENSOR_TYPES: [(&str, SensorConstructor); 2] =
    [("hx711", HX711), ("hx717", HX717)];

/// The Python module defines no entry points (library only), so the
/// factory is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
