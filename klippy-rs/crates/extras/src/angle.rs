//! Port of the `angle` module (`klippy/extras/angle.py`).
//!
//! SPI magnetic angle sensors: five sensor helpers (`a1333`, `as5047d`,
//! `tle5012b`, `mt6816`, `mt6826s`), the bulk `Angle` reader with
//! 64-bucket calibration interpolation, and the `AngleCalibration`
//! stepper-coupled calibration flow.
//!
//! Parity notes:
//! - Float arithmetic follows the Python source line by line; `%` /
//!   `%.3f` text goes through `pycompat`, `math.sqrt` calls libm.
//! - `AngleCalibration.load_calibration` reproduces
//!   `numpy.linalg.lstsq(..., rcond=None)` with an in-tree one-sided
//!   Jacobi SVD (no numpy dependency in the port).
//! - Collaborators are the owner modules' concrete types and traits:
//!   `crate::bus::mcu_spi_from_config` / `McuSpi` (`bus.MCU_SPI`),
//!   `crate::bulk_sensor::{BulkDataQueue, BatchBulkHelper}`,
//!   `crate::force_move::ForceMoveApi` and
//!   `crate::tmc::TMCCommandHelperApi`; core services (`gcode`,
//!   `configfile`, `toolhead`, `mcu`) are reached with
//!   `lookup_interface` like the Python `printer.lookup_object`.
//! - `Angle.add_client` is the interface `AngleCalibration` reaches
//!   through `printer.lookup_object(self.name)` (angle.py:167), so it
//!   is declared here as [`AngleApi`].
//!
//! Integer twists shared with the Python code live here:
//! [`py_trunc_i64`] / [`trunc_bigint`] / percent helpers.

pub mod calibration;
pub mod helpers;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AddConfigCmdOpts, CommandWrapper, Mcu, SendOpts};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::{ObjectRef, PrinterObject};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue, PyError};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, OnceLock, Weak};

use crate::bulk_sensor::{
    BatchBulkHelper, BulkDataQueue, ClientCallback, MAX_BULK_MSG_SIZE, SENSOR_BULK_FMT,
};
use crate::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};

pub use calibration::AngleCalibration;
pub use helpers::SensorHelperApi;

// ---------------------------------------------------------------------------
// Constants (angle.py lines 9-16, 604-608)
// ---------------------------------------------------------------------------

/// `MIN_MSG_TIME = 0.100` (line 9).
pub const MIN_MSG_TIME: f64 = 0.100;
/// `TCODE_ERROR = 0xff` (line 10).
pub const TCODE_ERROR: i64 = 0xff;
/// `TRINAMIC_DRIVERS` (lines 12-13).
pub const TRINAMIC_DRIVERS: [&str; 6] =
    ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"];
/// `CALIBRATION_BITS = 6` (line 15): 64 calibration buckets.
pub const CALIBRATION_BITS: i64 = 6;
/// `ANGLE_BITS = 16` (line 16): angles span 0..65535.
pub const ANGLE_BITS: i64 = 16;
/// `BYTES_PER_SAMPLE = 3` (line 604).
pub const BYTES_PER_SAMPLE: i64 = 3;
/// `MAX_BULK_MSG_SIZE` (`bulk_sensor.py:203`), imported from the owner
/// `crate::bulk_sensor` (value 51).
/// `SAMPLES_PER_BLOCK = MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE` (line 605).
pub const SAMPLES_PER_BLOCK: i64 = MAX_BULK_MSG_SIZE / BYTES_PER_SAMPLE;
/// `SAMPLE_PERIOD = 0.000400` (line 607).
pub const SAMPLE_PERIOD: f64 = 0.000400;
/// `BATCH_UPDATES = 0.100` (line 608).
pub const BATCH_UPDATES: f64 = 0.100;

// ---------------------------------------------------------------------------
// Python-semantics helpers
// ---------------------------------------------------------------------------

/// `int(float)` truncation toward zero (CPython `int(f)`).
/// Out-of-range values clamp like `clock.rs::py_trunc_i64` does.
pub fn py_trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = trunc_float_to_bigint(f).0.to_string();
    digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

/// `int(float)` as a `PyInt` (exact, no clamping).
pub fn trunc_bigint(f: f64) -> PyInt {
    trunc_float_to_bigint(f)
}

/// `PyValue` to `i64`: `Int` / `Float` (via `int(float)`) / `Bool`.
/// Returns `None` for non-numeric values.
pub fn pyvalue_to_i64(v: &PyValue) -> Option<i64> {
    match v {
        PyValue::Int(n) => n.0.to_string().parse::<i64>().ok(),
        PyValue::Float(f) => {
            if f.is_finite() {
                let digits = trunc_float_to_bigint(*f).0.to_string();
                Some(
                    digits
                        .parse::<i64>()
                        .unwrap_or(if *f < 0.0 { i64::MIN } else { i64::MAX }),
                )
            } else {
                None
            }
        }
        PyValue::Bool(b) => Some(i64::from(*b)),
        _ => None,
    }
}

/// `PyValue` to `f64` (`float()` semantics for numbers).
pub fn pyvalue_to_f64(v: &PyValue) -> Option<f64> {
    match v {
        PyValue::Float(f) => Some(*f),
        PyValue::Int(n) => n.0.to_string().parse::<f64>().ok(),
        PyValue::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
        _ => None,
    }
}

/// `pycompat::PyError` to the crossed-module `KlippyError`.
pub fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

/// `percent_format` with the Python error surfaced as `Other`.
pub fn fmt_percent(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(map_py_error)
}

/// `%` formatting with a constant format: arguments are right by
/// construction, so a failure is an internal error.
pub fn fmt_const(fmt: &str, args: &[PctValue]) -> String {
    match percent_format(fmt, args) {
        Ok(s) => s,
        Err(e) => format!("%-format failed: {}", e.msg),
    }
}

pub fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}
pub fn pct_float(v: f64) -> PctValue {
    PctValue::Float(v)
}
pub fn pct_str(v: &str) -> PctValue {
    PctValue::Str(v.to_string())
}

/// `list index out of range` (`IndexError`).
pub fn index_error() -> KlippyError {
    KlippyError::Other {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `KeyError` with the `repr(key)` message text.
pub fn key_error(key: &str) -> KlippyError {
    KlippyError::Other {
        class_name: "KeyError".to_string(),
        message: pycompat::value::repr_key(key),
    }
}

/// `AttributeError` for a duck-typed method the Python class lacks.
pub fn no_attribute_error(what: &str, attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'{what}' object has no attribute '{attr}'"),
    }
}

/// Dict lookup on a `PyValue` dict (`params['response']` etc.).
/// Missing field raises `KeyError` like CPython dict access.
pub fn dict_get<'a>(dict: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match dict {
        PyValue::Dict(map) => map
            .get(&PyValue::Str(key.to_string()))
            .ok_or_else(|| key_error(key)),
        _ => Err(no_attribute_error("PyValue", "get")),
    }
}

/// `float` division with the CPython zero check
/// (`ZeroDivisionError: float division by zero`).
pub fn float_div(a: f64, b: f64) -> KResult<f64> {
    pycompat::text::float_true_div(a, b).map_err(map_py_error)
}

/// `a % b` for floats (CPython rules incl. `ZeroDivisionError`).
pub fn float_mod(a: f64, b: f64) -> KResult<f64> {
    pycompat::text::float_mod(a, b).map_err(map_py_error)
}

/// Config `get_name().split()[-1]`: last whitespace token of a
/// non-empty name. An empty name maps to `""` (unreachable in
/// practice: printer sections are non-empty).
pub fn name_last_token(name: &str) -> String {
    name.split_whitespace().last().unwrap_or("").to_string()
}

// ---------------------------------------------------------------------------
// `Angle` interface other code reaches (`angle.py:660`)
// ---------------------------------------------------------------------------

/// `Angle` as the calibration flow reaches it:
/// `self.printer.lookup_object(self.name).add_client(handle_batch)`
/// (`angle.py:167`), a direct pass through to
/// `batch_bulk.add_client` (`angle.py:660-661`).
pub trait AngleApi: Send + Sync + 'static {
    /// `add_client(client_cb)` (`angle.py:660`).
    fn add_client(&self, client_cb: ClientCallback) -> KResult<()>;
}

// ---------------------------------------------------------------------------
// Sensor type
// ---------------------------------------------------------------------------

/// The `sensor_type` choice (`angle.py:620-626`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SensorKind {
    A1333,
    AS5047D,
    TLE5012B,
    MT6816,
    MT6826S,
}

impl SensorKind {
    /// `config.getchoice('sensor_type', {s: s for s in sensors})`
    /// (line 625).
    pub fn from_config(config: &Config) -> KResult<SensorKind> {
        let mut choices = IndexMap::new();
        for s in ["a1333", "as5047d", "tle5012b", "mt6816", "mt6826s"] {
            choices.insert(PyValue::Str(s.to_string()), PyValue::Str(s.to_string()));
        }
        let v = config.getchoice(
            "sensor_type",
            &PyValue::Dict(choices),
            GetChoiceOpts { default: None, note_valid: true },
        )?;
        match &v {
            PyValue::Str(s) if s == "a1333" => Ok(SensorKind::A1333),
            PyValue::Str(s) if s == "as5047d" => Ok(SensorKind::AS5047D),
            PyValue::Str(s) if s == "tle5012b" => Ok(SensorKind::TLE5012B),
            PyValue::Str(s) if s == "mt6816" => Ok(SensorKind::MT6816),
            PyValue::Str(s) if s == "mt6826s" => Ok(SensorKind::MT6826S),
            PyValue::Str(s) => Err(KlippyError::config(format!(
                "Choice '{s}' for option 'sensor_type' in section '{}' is not a valid choice",
                config.get_name()
            ))),
            other => Err(KlippyError::config(format!(
                "Choice '{}' for option 'sensor_type' in section '{}' is not a valid choice",
                other.str(),
                config.get_name()
            ))),
        }
    }

    /// `SPI_MODE` of the helper class.
    pub fn spi_mode(self) -> i64 {
        helpers::spi_mode(self)
    }

    /// `SPI_SPEED` of the helper class.
    pub fn spi_speed(self) -> i64 {
        helpers::spi_speed(self)
    }
}

// ---------------------------------------------------------------------------
// Angle printing object (angle.py:610-751)
// ---------------------------------------------------------------------------

/// Everything `Angle.__init__` assigns after the sensor section is
/// read; `attr` names the attribute Python would fail on.
struct AngleSetup {
    spi: Arc<McuSpi>,
    mcu: Mcu,
    oid: i64,
    helper: Arc<dyn SensorHelperApi>,
    bulk_queue: BulkDataQueue,
    batch_bulk: BatchBulkHelper,
    name: String,
    query_spi_angle_cmd: GCell<Option<CommandWrapper>>,
    start_clock: GCell<i64>,
    time_shift: GCell<i64>,
    sample_ticks: GCell<i64>,
    last_sequence: GCell<i64>,
    last_angle: GCell<i64>,
}

/// `class Angle` (`angle.py:610`): the `[angle <name>]` object.
pub struct Angle {
    printer: Printer,
    sample_period: f64,
    calibration: Arc<AngleCalibration>,
    setup: OnceLock<AngleSetup>,
}

impl Angle {
    /// `Angle.__init__(config)` (lines 611-649).
    pub fn new(config: &Config) -> KResult<Arc<Angle>> {
        let printer: Printer = config.get_printer();
        let sample_period = config
            .getfloat(
                "sample_period",
                GetFloatOpts {
                    default: Some(Some(SAMPLE_PERIOD)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("getfloat with a default returns a value");
        let calibration = AngleCalibration::new(config)?;
        let angle = Arc::new(Angle {
            printer: Arc::clone(&printer),
            sample_period,
            calibration,
            setup: OnceLock::new(),
        });
        // Sensor type (lines 620-628).
        let sensor_kind = SensorKind::from_config(config)?;
        let spi = mcu_spi_from_config(
            config,
            sensor_kind.spi_mode(),
            "cs_pin",
            sensor_kind.spi_speed(),
            None,
            false,
        )?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid()?;
        let helper = helpers::create_helper(sensor_kind, config, Arc::clone(&spi), oid)?;
        // Setup mcu sensor_spi_angle bulk query code (lines 630-640).
        mcu.add_config_cmd(
            &fmt_const(
                "config_spi_angle oid=%d spi_oid=%d spi_angle_type=%s",
                &[
                    pct_int(oid),
                    pct_int(spi.get_oid()),
                    pct_str(sensor_type_name(sensor_kind)),
                ],
            ),
            AddConfigCmdOpts::default(),
        )?;
        mcu.add_config_cmd(
            &fmt_const("query_spi_angle oid=%d clock=0 rest_ticks=0 time_shift=0", &[
                pct_int(oid),
            ]),
            AddConfigCmdOpts { is_init: false, on_restart: true },
        )?;
        {
            let build: Weak<Angle> = Arc::downgrade(&angle);
            mcu.register_config_callback(Box::new(move || match build.upgrade() {
                Some(dev) => dev.build_config(),
                None => Ok(()),
            }))?;
        }
        // `self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=oid)` (line 641).
        let bulk_queue = BulkDataQueue::new(&mcu, SENSOR_BULK_FMT, Some(oid));
        // `self.name = config.get_name().split()[1]` (line 646).
        let name = split_name_second(&config.get_name())?;
        // Process messages in batches (lines 643-645).
        let process: Weak<Angle> = Arc::downgrade(&angle);
        let start = process.clone();
        let finish = process.clone();
        let batch_bulk = BatchBulkHelper::new(
            Arc::clone(&printer),
            Box::new(move |eventtime| match process.upgrade() {
                Some(dev) => dev.process_batch(eventtime),
                None => Ok(PyValue::Dict(IndexMap::new())),
            }),
            Some(Box::new(move || match start.upgrade() {
                Some(dev) => dev.start_measurements(),
                None => Ok(()),
            })),
            Some(Box::new(move || match finish.upgrade() {
                Some(dev) => dev.finish_measurements(),
                None => Ok(()),
            })),
            BATCH_UPDATES,
        );
        let mut header = IndexMap::new();
        header.insert(
            PyValue::Str("header".to_string()),
            PyValue::Tuple(vec![
                PyValue::Str("time".to_string()),
                PyValue::Str("angle".to_string()),
            ]),
        );
        batch_bulk.add_mux_endpoint("angle/dump_angle", "sensor", &name, PyValue::Dict(header))?;
        let setup = AngleSetup {
            spi,
            mcu,
            oid,
            helper,
            bulk_queue,
            batch_bulk,
            name,
            query_spi_angle_cmd: GCell::new(None),
            start_clock: GCell::new(0),
            time_shift: GCell::new(0),
            sample_ticks: GCell::new(0),
            last_sequence: GCell::new(0),
            last_angle: GCell::new(0),
        };
        if angle.setup.set(setup).is_err() {
            unreachable!("Angle setup is assigned once");
        }
        Ok(angle)
    }

    /// The attributes assigned in `__init__`; `attr` is the one Python
    /// would fail on if the object were used before it finished.
    fn setup(&self, attr: &str) -> KResult<&AngleSetup> {
        self.setup
            .get()
            .ok_or_else(|| no_attribute_error("Angle", attr))
    }

    /// `_build_config` (lines 650-657).
    pub fn build_config(&self) -> KResult<()> {
        let setup = self.setup("_build_config")?;
        let freq: i64 = setup.mcu.seconds_to_clock(1.0);
        let mut shift: i64 = setup.time_shift.get();
        while (TCODE_ERROR << shift) as f64 / (freq as f64) < 0.002 {
            shift += 1;
        }
        setup.time_shift.set(shift);
        let cmdqueue = setup.spi.get_command_queue();
        let cmd = setup.mcu.lookup_command(
            "query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c",
            Some(cmdqueue),
        )?;
        setup.query_spi_angle_cmd.replace(Some(cmd));
        Ok(())
    }

    /// `get_status` (lines 658-659): `{'temperature': ...}`.
    pub fn temperature_status(&self) -> PyValue {
        let temp = self.setup.get().and_then(|s| s.helper.last_temperature());
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("temperature".to_string()),
            temp.map(PyValue::Float).unwrap_or(PyValue::None),
        );
        PyValue::Dict(map)
    }

    /// `add_client` (line 660).
    pub fn add_client(&self, client_cb: ClientCallback) -> KResult<()> {
        self.setup("batch_bulk")?.batch_bulk.add_client(client_cb)
    }

    /// `self.batch_bulk` (`bulk_sensor.BatchBulkHelper`).
    pub fn batch_bulk(&self) -> KResult<&BatchBulkHelper> {
        Ok(&self.setup("batch_bulk")?.batch_bulk)
    }

    /// `_extract_samples` (lines 663-717). Returns `(samples, error_count)`
    /// where a sample is `(ptime, angle)`.
    pub fn extract_samples(
        &self,
        raw_samples: &[PyValue],
    ) -> KResult<(Vec<(f64, i64)>, i64)> {
        let setup = self.setup("_extract_samples")?;
        let sample_ticks: i64 = setup.sample_ticks.get();
        let start_clock: i64 = setup.start_clock.get();
        let mut last_sequence: i64 = setup.last_sequence.get();
        let mut last_angle: i64 = setup.last_angle.get();
        let is_absolute = setup.helper.is_tcode_absolute();
        let mut time_shift: i64 = 0;
        let mut static_delay: f64 = 0.0;
        let mut last_chip_mcu_clock: i64 = 0;
        let mut last_chip_clock: i64 = 0;
        let mut chip_freq: f64 = 0.0;
        let mut inv_chip_freq: f64 = 0.0;
        if is_absolute {
            let (a, b, f) = setup.helper.get_tcode_params()?;
            last_chip_mcu_clock = a;
            last_chip_clock = b;
            chip_freq = f;
            inv_chip_freq = float_div(1.0, chip_freq)?;
        } else {
            time_shift = setup.time_shift.get();
            static_delay = setup.helper.get_static_delay()?;
        }
        let mut error_count: i64 = 0;
        let mut samples: Vec<(f64, i64)> = Vec::new();
        for params in raw_samples {
            let seq_val = dict_get(params, "sequence")?;
            let sequence: i64 = pyvalue_to_i64(seq_val).unwrap_or(0);
            let seq_diff: i64 = (sequence - last_sequence) & 0xffff;
            last_sequence = last_sequence.wrapping_add(seq_diff);
            let samp_count: i64 = last_sequence.wrapping_mul(SAMPLES_PER_BLOCK);
            let msg_mclock: i64 = start_clock
                .wrapping_add(samp_count.wrapping_mul(sample_ticks));
            let data_val = dict_get(params, "data")?;
            let d_owned: Vec<u8> = match data_val {
                PyValue::Bytes(b) => b.clone(),
                PyValue::List(items) => items
                    .iter()
                    .map(|v| pyvalue_to_i64(v).unwrap_or(0).rem_euclid(256) as u8)
                    .collect(),
                _ => Vec::new(),
            };
            let blocks = d_owned.len() as i64 / BYTES_PER_SAMPLE;
            for i in 0..blocks {
                let base = (i * BYTES_PER_SAMPLE) as usize;
                let tcode: i64 = d_owned[base] as i64;
                if tcode == TCODE_ERROR {
                    error_count += 1;
                    continue;
                }
                let raw_angle: i64 =
                    d_owned[base + 1] as i64 | ((d_owned[base + 2] as i64) << 8);
                let mut angle_diff: i64 = (raw_angle - last_angle) & 0xffff;
                angle_diff -= (angle_diff & 0x8000) << 1;
                last_angle = last_angle.wrapping_add(angle_diff);
                let mclock: i64 = msg_mclock.wrapping_add(i.wrapping_mul(sample_ticks));
                let sclock: f64 = if is_absolute {
                    // tcode is the tle5012b frame counter (lines 694-699).
                    let mdiff: i64 = mclock - last_chip_mcu_clock;
                    let chip_mclock: i64 = last_chip_clock
                        .wrapping_add(py_trunc_i64(mdiff as f64 * chip_freq + 0.5));
                    let mut cdiff: i64 = ((tcode << 10) - chip_mclock) & 0xffff;
                    cdiff -= (cdiff & 0x8000) << 1;
                    (mclock as f64) + ((cdiff - 0x800) as f64) * inv_chip_freq
                } else {
                    // tcode is the mcu clock offset shifted by time_shift.
                    let tcode_shifted: i64 = tcode.wrapping_shl(time_shift as u32);
                    (mclock.wrapping_add(tcode_shifted)) as f64
                };
                let ptime = pycompat::text::round_py_nd(
                    setup.mcu.clock_to_print_time(sclock) - static_delay,
                    6,
                );
                samples.push((ptime, last_angle));
            }
        }
        setup.last_sequence.set(last_sequence);
        setup.last_angle.set(last_angle);
        Ok((samples, error_count))
    }

    /// `_is_measuring` (lines 719-720).
    pub fn is_measuring(&self) -> KResult<bool> {
        Ok(self.setup("start_clock")?.start_clock.get() != 0)
    }

    /// `_start_measurements` (lines 721-733).
    pub fn start_measurements(&self) -> KResult<()> {
        let setup = self.setup("_start_measurements")?;
        klippy_api::log::info(&fmt_const(
            "Starting angle '%s' measurements",
            &[pct_str(&setup.name)],
        ));
        setup.helper.start()?;
        // Start bulk reading.
        setup.bulk_queue.clear_queue();
        setup.last_sequence.set(0);
        let systime: f64 = self.printer.get_reactor().monotonic();
        let print_time: f64 = setup.mcu.estimated_print_time(systime) + MIN_MSG_TIME;
        let reqclock: i64 = setup.mcu.print_time_to_clock(print_time);
        setup.start_clock.set(reqclock);
        let rest_ticks: i64 = setup.mcu.seconds_to_clock(self.sample_period);
        setup.sample_ticks.set(rest_ticks);
        let cmd = setup
            .query_spi_angle_cmd
            .borrow()
            .clone()
            .ok_or_else(|| no_attribute_error("NoneType", "send"))?;
        cmd.send(
            vec![
                PyValue::Int(PyInt::from_i64(setup.oid)),
                PyValue::Int(PyInt::from_i64(reqclock)),
                PyValue::Int(PyInt::from_i64(rest_ticks)),
                PyValue::Int(PyInt::from_i64(setup.time_shift.get())),
            ],
            SendOpts { minclock: 0, reqclock },
        )
    }

    /// `_finish_measurements` (lines 734-739).
    pub fn finish_measurements(&self) -> KResult<()> {
        let setup = self.setup("_finish_measurements")?;
        // Halt bulk reading.
        let cmd = setup
            .query_spi_angle_cmd
            .borrow()
            .clone()
            .ok_or_else(|| no_attribute_error("NoneType", "send_wait_ack"))?;
        cmd.send_wait_ack(
            vec![
                PyValue::Int(PyInt::from_i64(setup.oid)),
                PyValue::Int(PyInt::from_i64(0)),
                PyValue::Int(PyInt::from_i64(0)),
                PyValue::Int(PyInt::from_i64(0)),
            ],
            SendOpts::default(),
        )?;
        setup.bulk_queue.clear_queue();
        setup.helper.set_last_temperature(None);
        klippy_api::log::info(&fmt_const(
            "Stopped angle '%s' measurements",
            &[pct_str(&setup.name)],
        ));
        Ok(())
    }

    /// `_process_batch` (lines 740-751). Returns the batch dict
    /// (empty dict when there is nothing to report).
    pub fn process_batch(&self, _eventtime: f64) -> KResult<PyValue> {
        let setup = self.setup("_process_batch")?;
        if setup.helper.is_tcode_absolute() {
            setup.helper.update_clock()?;
        }
        let raw_samples: Vec<PyValue> = setup.bulk_queue.pull_queue();
        if raw_samples.is_empty() {
            return Ok(PyValue::Dict(IndexMap::new()));
        }
        let (mut samples, error_count) = self.extract_samples(&raw_samples)?;
        if samples.is_empty() {
            return Ok(PyValue::Dict(IndexMap::new()));
        }
        let offset: Option<f64> = self.calibration.apply_calibration(&mut samples)?;
        let mut map = IndexMap::new();
        let data: Vec<PyValue> = samples
            .iter()
            .map(|(t, a)| {
                PyValue::Tuple(vec![
                    PyValue::Float(*t),
                    PyValue::Int(PyInt::from_i64(*a)),
                ])
            })
            .collect();
        map.insert(PyValue::Str("data".to_string()), PyValue::List(data));
        map.insert(
            PyValue::Str("errors".to_string()),
            PyValue::Int(PyInt::from_i64(error_count)),
        );
        map.insert(
            PyValue::Str("position_offset".to_string()),
            offset.map(PyValue::Float).unwrap_or(PyValue::None),
        );
        Ok(PyValue::Dict(map))
    }
}

impl AngleApi for Angle {
    fn add_client(&self, client_cb: ClientCallback) -> KResult<()> {
        Angle::add_client(self, client_cb)
    }
}

fn sensor_type_name(kind: SensorKind) -> &'static str {
    match kind {
        SensorKind::A1333 => "a1333",
        SensorKind::AS5047D => "as5047d",
        SensorKind::TLE5012B => "tle5012b",
        SensorKind::MT6816 => "mt6816",
        SensorKind::MT6826S => "mt6826s",
    }
}

fn split_name_second(name: &str) -> KResult<String> {
    name.split_whitespace().nth(1).map(str::to_string).ok_or_else(index_error)
}

impl PrinterObject for Angle {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn AngleApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.temperature_status())
    }
}

/// `load_config_prefix(config)` (lines 753-754).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let angle = Angle::new(config)?;
    Ok(Some(angle as ObjectRef))
}

/// Module entry point for the `angle` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
