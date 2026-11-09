//! Port of the `ldc1612` module (`klippy/extras/ldc1612.py`).
//!
//! The LDC1612 is an eddy-current frequency sensor. It is not a printer
//! object: `probe_eddy_current` constructs [`LDC1612`] directly with its
//! section config and an optional `EddyCalibration`
//! (`probe_eddy_current.py:1027-1029`), like the Python class. The Python
//! module defines no loader, so [`module_factory`] exposes no entry point.
//!
//! Collaborators are the owners' real types: `crate::bus::mcu_i2c_from_config`
//! (`bus.MCU_I2C_from_config`), `crate::bulk_sensor::FixedFreqReader` and
//! `crate::bulk_sensor::BatchBulkHelper`. The calibration argument is
//! reached through `crate::probe_eddy_current::EddyCalibrationApi`
//! (`get_calibration`, `apply_calibration`). In the other direction
//! `trigger_analog` drives the sensor through [`TriggerAnalogSensor`] and
//! `probe_eddy_current` through [`LDC1612Api`], both implemented below.

use crate::bulk_sensor::{
    BatchBulkHelper, BatchCallback, ClientCallback, FixedFreqReader, StartStopCallback,
};
use crate::bus::{mcu_i2c_from_config, McuI2c, McuI2cApi};
use crate::probe_eddy_current::EddyCalibrationApi;
use crate::trigger_analog::TriggerAnalogSensor;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetIntOpts, GetStrOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler};
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandWrapper, ConfigCallback, LookupPinOpts, Mcu, PinsApi,
    QuerySendOpts, SendOpts,
};
use klippy_api::object::lookup_interface;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::GCell;
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

pub const MIN_MSG_TIME: f64 = 0.100;
pub const BATCH_UPDATES: f64 = 0.100;
pub const LDC1612_ADDR: i64 = 0x2a;
pub const DEFAULT_LDC1612_FREQ: i64 = 12_000_000;
pub const SETTLETIME: f64 = 0.005;
pub const DRIVECUR: i64 = 15;
pub const DEGLITCH: i64 = 0x05;

pub const LDC1612_MANUF_ID: i64 = 0x5449;
pub const LDC1612_DEV_ID: i64 = 0x3055;

pub const REG_RCOUNT0: i64 = 0x08;
pub const REG_OFFSET0: i64 = 0x0c;
pub const REG_SETTLECOUNT0: i64 = 0x10;
pub const REG_CLOCK_DIVIDERS0: i64 = 0x14;
pub const REG_ERROR_CONFIG: i64 = 0x19;
pub const REG_CONFIG: i64 = 0x1a;
pub const REG_MUX_CONFIG: i64 = 0x1b;
pub const REG_DRIVE_CURRENT0: i64 = 0x1e;
pub const REG_MANUFACTURER_ID: i64 = 0x7e;
pub const REG_DEVICE_ID: i64 = 0x7f;

/// `DriveCurrentCalibrate.cmd_LDC_CALIBRATE_help` (`ldc1612.py:49`).
pub const CMD_LDC_CALIBRATE_HELP: &str = "Calibrate LDC1612 DRIVE_CURRENT register";

/// One raw bulk sample `(ptime, val)` as `FixedFreqReader.pull_samples`
/// returns it (`val` is the unpacked `">I"` uint32).
pub type RawSample = (f64, i64);

fn pct(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn internal(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

/// `name.split()[-1]` (`ldc1612.py:44,79`).
fn last_token(name: &str) -> String {
    name.split_whitespace().last().unwrap_or_default().to_string()
}

fn pyint_to_i64(n: &PyInt) -> Option<i64> {
    n.inner().to_string().parse().ok()
}

/// `bytearray(params['response'])` (`ldc1612.py:150`). `MCU_I2C.i2c_read`
/// returns `None` after a failed `i2c_transfer` (`bus.py:295-300`), so the
/// subscript raises `TypeError` there; a dict without the key raises
/// `KeyError`.
fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
    let map = match params {
        PyValue::Dict(map) => map,
        other => {
            return Err(internal(
                "TypeError",
                format!(
                    "'{}' object is not subscriptable",
                    pycompat::value::type_name(other)
                ),
            ));
        }
    };
    let response = map
        .iter()
        .find(|(k, _)| matches!(k, PyValue::Str(s) if s == "response"))
        .map(|(_, v)| v)
        .ok_or_else(|| internal("KeyError", "'response'"))?;
    match response {
        PyValue::Bytes(bytes) => Ok(bytes.clone()),
        PyValue::List(values) | PyValue::Tuple(values) => values
            .iter()
            .map(|v| match v {
                PyValue::Int(n) => pyint_to_i64(n)
                    .and_then(|b| u8::try_from(b).ok())
                    .ok_or_else(|| internal("ValueError", "byte must be in range(0, 256)")),
                other => Err(internal(
                    "TypeError",
                    format!(
                        "'{}' object cannot be interpreted as an integer",
                        pycompat::value::type_name(other)
                    ),
                )),
            })
            .collect(),
        PyValue::Str(_) => Err(internal("TypeError", "string argument without an encoding")),
        other => Err(internal(
            "TypeError",
            format!("cannot convert '{}' object to bytearray", pycompat::value::type_name(other)),
        )),
    }
}

/// `for ptime, val in samples` (`ldc1612.py:172`) over the
/// `FixedFreqReader.pull_samples()` tuples.
fn decode_raw_samples(samples: Vec<PyValue>) -> KResult<Vec<RawSample>> {
    let mut out = Vec::with_capacity(samples.len());
    for sample in &samples {
        let items: &[PyValue] = match sample {
            PyValue::Tuple(items) | PyValue::List(items) => items,
            other => {
                return Err(internal(
                    "TypeError",
                    format!(
                        "cannot unpack non-iterable {} object",
                        pycompat::value::type_name(other)
                    ),
                ));
            }
        };
        if items.len() != 2 {
            let msg = if items.len() < 2 {
                format!("not enough values to unpack (expected 2, got {})", items.len())
            } else {
                "too many values to unpack (expected 2)".to_string()
            };
            return Err(internal("ValueError", msg));
        }
        let ptime = match &items[0] {
            PyValue::Float(f) => *f,
            PyValue::Int(n) => n.inner().to_string().parse::<f64>().unwrap_or(f64::NAN),
            other => {
                return Err(internal(
                    "TypeError",
                    format!("type {} doesn't define __round__ method", pycompat::value::type_name(other)),
                ));
            }
        };
        let val = match &items[1] {
            PyValue::Int(n) => pyint_to_i64(n)
                .ok_or_else(|| internal("OverflowError", "Python int too large to convert to C long"))?,
            other => {
                return Err(internal(
                    "TypeError",
                    format!(
                        "unsupported operand type(s) for &: '{}' and 'int'",
                        pycompat::value::type_name(other)
                    ),
                ));
            }
        };
        out.push((ptime, val));
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// DriveCurrentCalibrate (klippy/extras/ldc1612.py:35-73)
// ---------------------------------------------------------------------------

/// `DriveCurrentCalibrate` (`ldc1612.py:35`).
pub struct DriveCurrentCalibrate {
    printer: Printer,
    name: String,
    drive_cur: i64,
    /// `self.sensor`: the `LDC1612` that constructs this helper, stored
    /// once that object exists.
    sensor: GCell<Option<Arc<LDC1612>>>,
}

impl DriveCurrentCalibrate {
    /// `DriveCurrentCalibrate.__init__(config, sensor)` (`ldc1612.py:36`).
    fn new(config: &Config) -> KResult<Arc<DriveCurrentCalibrate>> {
        let printer = config.get_printer();
        let drive_cur = config
            .getint(
                "reg_drive_current",
                GetIntOpts {
                    default: Some(Some(DRIVECUR)),
                    minval: Some(0),
                    maxval: Some(31),
                    note_valid: true,
                },
            )?
            .unwrap_or(DRIVECUR);
        let name = config.get_name();
        let dcc = Arc::new(DriveCurrentCalibrate {
            printer: printer.clone(),
            name: name.clone(),
            drive_cur,
            sensor: GCell::new(None),
        });
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let handler_dcc = dcc.clone();
        let handler: GCodeCommandHandler =
            Box::new(move |gcmd: &GCmd| handler_dcc.cmd_ldc_calibrate(gcmd));
        gcode.register_mux_command(
            "LDC_CALIBRATE_DRIVE_CURRENT",
            "CHIP",
            Some(&last_token(&name)),
            handler,
            Some(CMD_LDC_CALIBRATE_HELP),
        )?;
        Ok(dcc)
    }

    fn sensor(&self) -> Arc<LDC1612> {
        self.sensor
            .borrow()
            .clone()
            .expect("LDC1612::new stores the sensor before returning")
    }

    /// `DriveCurrentCalibrate.get_drive_current` (`ldc1612.py:47`).
    pub fn get_drive_current(&self) -> i64 {
        self.drive_cur
    }

    /// `DriveCurrentCalibrate.cmd_LDC_CALIBRATE` (`ldc1612.py:50`).
    pub fn cmd_ldc_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let sensor = self.sensor();
        // `handle_batch` reads the enclosing `is_in_progress` at call time.
        let is_in_progress = Arc::new(GCell::new(true));
        let client_flag = is_in_progress.clone();
        let handle_batch: ClientCallback = Box::new(move |_msg: PyValue| Ok(client_flag.get()));
        sensor.add_client(handle_batch)?;
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        toolhead.dwell(0.100)?;
        toolhead.wait_moves()?;
        let old_config = sensor.read_reg(REG_CONFIG)?;
        sensor.set_reg(REG_CONFIG, 0x001 | (1 << 9))?;
        toolhead.wait_moves()?;
        toolhead.dwell(0.100)?;
        toolhead.wait_moves()?;
        let reg_drive_current0 = sensor.read_reg(REG_DRIVE_CURRENT0)?;
        sensor.set_reg(REG_CONFIG, old_config)?;
        is_in_progress.set(false);
        // Report found value to user
        let drive_cur = (reg_drive_current0 >> 6) & 0x1f;
        let msg = pct(
            "%s: reg_drive_current: %d\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
            &[PctValue::Str(self.name.clone()), int(drive_cur)],
        )?;
        gcmd.respond_info(&msg, true);
        let configfile =
            lookup_interface::<dyn PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        configfile.set(&self.name, "reg_drive_current", &pct("%d", &[int(drive_cur)])?)?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// LDC1612 (klippy/extras/ldc1612.py:76-238)
// ---------------------------------------------------------------------------

/// `LDC1612` (`ldc1612.py:76`). Held in an `Arc` by `probe_eddy_current`;
/// never registered with the printer.
pub struct LDC1612 {
    name: String,
    calibration: Option<Arc<dyn EddyCalibrationApi>>,
    dccal: Arc<DriveCurrentCalibrate>,
    data_rate: i64,
    i2c: Arc<McuI2c>,
    mcu: Mcu,
    oid: i64,
    query_ldc1612_cmd: GCell<Option<CommandWrapper>>,
    clock_freq: i64,
    sensor_div: i64,
    freq_conv: f64,
    sensor_errors: GCell<IndexMap<i64, String>>,
    ffreader: FixedFreqReader,
    last_error_count: GCell<i64>,
    batch_bulk: BatchBulkHelper,
}

impl LDC1612 {
    /// `LDC1612(config, calibration=None)` (`ldc1612.py:77`).
    pub fn new(
        config: &Config,
        calibration: Option<Arc<dyn EddyCalibrationApi>>,
    ) -> KResult<Arc<LDC1612>> {
        let printer = config.get_printer();
        let name = last_token(&config.get_name());
        let dccal = DriveCurrentCalibrate::new(config)?;
        let data_rate = 400;
        // Setup mcu sensor_ldc1612 bulk query code
        let i2c = mcu_i2c_from_config(config, Some(LDC1612_ADDR), 400_000, false)?;
        let mcu = i2c.get_mcu();
        let oid = mcu.create_oid()?;
        let clock_freq = config
            .getint(
                "frequency",
                GetIntOpts {
                    default: Some(Some(DEFAULT_LDC1612_FREQ)),
                    minval: Some(2_000_000),
                    maxval: Some(40_000_000),
                    note_valid: true,
                },
            )?
            .unwrap_or(DEFAULT_LDC1612_FREQ);
        // Determine sensor divider (want 4*max_hz < clock_ref)
        let max_hz = config
            .getfloat(
                "max_sensor_hz",
                GetFloatOpts {
                    default: Some(Some(5_000_000.)),
                    minval: Some(3_000_000.),
                    maxval: Some(20_000_000.),
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(5_000_000.);
        let sensor_div =
            kos::cconv::f64_to_i64(kos::libm::ceil(4. * max_hz / (clock_freq as f64)));
        // `float(self.clock_freq * self.sensor_div) / (1<<28)`: the int
        // product is converted, then divided.
        let freq_conv = ((clock_freq * sensor_div) as f64) / ((1i64 << 28) as f64);
        if let Some(cal) = &calibration {
            let (cal_freqs, _cal_zpos) = cal.get_calibration();
            if let Some((&first, rest)) = cal_freqs.split_first() {
                // `max()` keeps the first item unless a later one compares greater.
                let cal_max = rest.iter().fold(first, |m, &f| if f > m { f } else { m });
                if cal_max > max_hz {
                    let pconfig = lookup_interface::<dyn PrinterConfigApi>(
                        &*printer,
                        "configfile",
                        LookupDefault::Raise,
                    )?
                    .expect("LookupDefault::Raise never yields None");
                    let msg = pct(
                        "ldc1612 %s: Should set 'max_sensor_hz' to at least %d",
                        &[PctValue::Str(name.clone()), PctValue::Float(kos::libm::ceil(cal_max))],
                    )?;
                    pconfig.runtime_warning(&msg);
                }
            }
        }
        // Configure intb and mcu object
        let no_default = GetStrOpts { default: Some(None), note_valid: true };
        if config.get("intb_pin", no_default)?.is_some() {
            let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
            let intb_pin = config
                .get("intb_pin", GetStrOpts::default())?
                .expect("a required option yields a value");
            let pin_params = ppins.lookup_pin(&intb_pin, LookupPinOpts::default())?;
            if !Arc::ptr_eq(&pin_params.chip, &mcu) {
                return Err(KlippyError::config("ldc1612 intb_pin must be on same mcu"));
            }
            mcu.add_config_cmd(
                &pct(
                    "config_ldc1612_with_intb oid=%d i2c_oid=%d intb_pin=%s",
                    &[int(oid), int(i2c.get_oid()), PctValue::Str(pin_params.pin.clone())],
                )?,
                AddConfigCmdOpts::default(),
            )?;
        } else {
            mcu.add_config_cmd(
                &pct("config_ldc1612 oid=%d i2c_oid=%d", &[int(oid), int(i2c.get_oid())])?,
                AddConfigCmdOpts::default(),
            )?;
        }
        mcu.add_config_cmd(
            &pct("query_ldc1612 oid=%d rest_ticks=0", &[int(oid)])?,
            AddConfigCmdOpts { is_init: false, on_restart: true },
        )?;
        // The callbacks below reach the finished object through this cell,
        // filled before `new` returns.
        let cell: Arc<GCell<Option<Arc<LDC1612>>>> = Arc::new(GCell::new(None));
        let get = |cell: &GCell<Option<Arc<LDC1612>>>| {
            cell.borrow().clone().expect("LDC1612::new stores the sensor before returning")
        };
        let config_cell = cell.clone();
        let build_config: ConfigCallback = Box::new(move || get(&config_cell).build_config());
        mcu.register_config_callback(build_config)?;
        // Bulk sample message reading
        let chip_smooth = (data_rate as f64) * BATCH_UPDATES * 2.;
        let ffreader = FixedFreqReader::new(mcu.clone(), chip_smooth, ">I")?;
        // Process messages in batches
        let batch_cell = cell.clone();
        let batch_cb: BatchCallback =
            Box::new(move |eventtime| get(&batch_cell).process_batch(eventtime));
        let start_cell = cell.clone();
        let start_cb: StartStopCallback =
            Box::new(move || get(&start_cell).start_measurements());
        let stop_cell = cell.clone();
        let stop_cb: StartStopCallback =
            Box::new(move || get(&stop_cell).finish_measurements());
        let batch_bulk =
            BatchBulkHelper::new(printer, batch_cb, Some(start_cb), Some(stop_cb), BATCH_UPDATES);
        let ldc = Arc::new(LDC1612 {
            name,
            calibration,
            dccal: dccal.clone(),
            data_rate,
            i2c,
            mcu,
            oid,
            query_ldc1612_cmd: GCell::new(None),
            clock_freq,
            sensor_div,
            freq_conv,
            sensor_errors: GCell::new(IndexMap::new()),
            ffreader,
            last_error_count: GCell::new(0),
            batch_bulk,
        });
        cell.replace(Some(ldc.clone()));
        dccal.sensor.replace(Some(ldc.clone()));
        let mut start_resp = IndexMap::new();
        start_resp.insert(
            PyValue::Str("header".to_string()),
            PyValue::Tuple(vec![
                PyValue::Str("time".to_string()),
                PyValue::Str("frequency".to_string()),
                PyValue::Str("z".to_string()),
            ]),
        );
        ldc.batch_bulk.add_mux_endpoint(
            "ldc1612/dump_ldc1612",
            "sensor",
            &ldc.name,
            PyValue::Dict(start_resp),
        )?;
        Ok(ldc)
    }

    /// `LDC1612.setup_trigger_analog(trigger_analog_oid)` (`ldc1612.py:130`).
    pub fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        self.mcu.add_config_cmd(
            &pct(
                "ldc1612_attach_trigger_analog oid=%d trigger_analog_oid=%d",
                &[int(self.oid), int(trigger_analog_oid)],
            )?,
            AddConfigCmdOpts { is_init: true, on_restart: false },
        )
    }

    /// `LDC1612._build_config` (`ldc1612.py:134`), an MCU config callback.
    fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.i2c.get_command_queue();
        let cmd = self
            .mcu
            .lookup_command("query_ldc1612 oid=%c rest_ticks=%u", Some(cmdqueue))?;
        self.query_ldc1612_cmd.replace(Some(cmd));
        self.ffreader
            .setup_query_command("query_status_ldc1612 oid=%c", self.oid, Some(cmdqueue))?;
        let enumerations = self.mcu.get_enumerations();
        let mut sensor_errors = IndexMap::new();
        if let Some(errors) = enumerations.get("ldc1612_error:") {
            for (k, v) in errors.iter() {
                sensor_errors.insert(*v, k.clone());
            }
        }
        self.sensor_errors.replace(sensor_errors);
        Ok(())
    }

    /// `LDC1612.get_mcu` (`ldc1612.py:142`).
    pub fn get_mcu(&self) -> Mcu {
        self.i2c.get_mcu()
    }

    /// `LDC1612.get_samples_per_second` (`ldc1612.py:144`).
    pub fn get_samples_per_second(&self) -> i64 {
        self.data_rate
    }

    /// `LDC1612.read_reg(reg)` (`ldc1612.py:146`).
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        if self.mcu.is_fileoutput() {
            return Ok(0);
        }
        let params = self.i2c.i2c_read(&[reg as u8], 2, QuerySendOpts::default())?;
        let response = response_bytes(&params)?;
        if response.len() < 2 {
            return Err(internal("IndexError", "bytearray index out of range"));
        }
        Ok(((response[0] as i64) << 8) | (response[1] as i64))
    }

    /// `LDC1612.set_reg(reg, val, minclock=0)` (`ldc1612.py:152`).
    pub fn set_reg(&self, reg: i64, val: i64) -> KResult<()> {
        self.i2c.i2c_write(
            &[reg as u8, ((val >> 8) & 0xff) as u8, (val & 0xff) as u8],
            QuerySendOpts::default(),
        )
    }

    /// `LDC1612.add_client(cb)` (`ldc1612.py:155`).
    pub fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        self.batch_bulk.add_client(cb)
    }

    /// `LDC1612.lookup_sensor_error(error)` (`ldc1612.py:157`).
    pub fn lookup_sensor_error(&self, error: i64) -> String {
        self.sensor_errors
            .borrow()
            .get(&error)
            .cloned()
            .unwrap_or_else(|| "Unknown ldc1612 error".to_string())
    }

    /// `LDC1612.convert_raw_to_frequency(raw_value)` (`ldc1612.py:159`).
    pub fn convert_raw_to_frequency(&self, raw_value: f64) -> f64 {
        raw_value * self.freq_conv
    }

    /// `LDC1612.convert_frequency_to_raw(freq)` (`ldc1612.py:161`).
    pub fn convert_frequency_to_raw(&self, freq: f64) -> KResult<i64> {
        let v = freq / self.freq_conv + 0.5;
        if v.is_nan() {
            return Err(internal("ValueError", "cannot convert float NaN to integer"));
        }
        if v.is_infinite() {
            return Err(internal("OverflowError", "cannot convert float infinity to integer"));
        }
        Ok(kos::cconv::f64_to_i64(v))
    }

    /// `LDC1612._convert_samples(samples)` (`ldc1612.py:164`).
    fn convert_samples(&self, samples: Vec<RawSample>) -> KResult<Vec<(f64, f64, f64)>> {
        let freq_conv = self.freq_conv;
        let mut out: Vec<(f64, f64, f64)> = Vec::with_capacity(samples.len());
        let mut errors: IndexMap<String, i64> = IndexMap::new();
        let mut log_once = |msg: String| *errors.entry(msg).or_insert(0) += 1;
        for (ptime, val) in samples {
            let mv = val & 0x0fff_ffff;
            if val > 0x03ff_ffff || val == 0x0 {
                self.last_error_count.set(self.last_error_count.get() + 1);
                if (val >> 16 & 0xffff) == 0xffff {
                    // Encoded error from sensor_ldc1612.c
                    log_once(self.lookup_sensor_error(val & 0xffff));
                    continue;
                }
                let error_bits = (val >> 28) & 0x0f;
                if error_bits & 0x8 != 0 || mv == 0x0000000 {
                    log_once("Frequency under valid range".to_string());
                }
                if error_bits & 0x4 != 0 || mv > 0x3ffffff {
                    let kind = if error_bits & 0x4 != 0 { "hard" } else { "soft" };
                    log_once(pct("Frequency over valid %s range", &[PctValue::Str(kind.to_string())])?);
                }
                if error_bits & 0x2 != 0 {
                    log_once("Conversion Watchdog timeout".to_string());
                }
                if error_bits & 0x1 != 0 {
                    log_once("Amplitude Low/High warning".to_string());
                }
            }
            out.push((round_py_nd(ptime, 6), round_py_nd(freq_conv * (mv as f64), 3), 999.9));
        }
        for (msg, count) in errors.iter() {
            log::error(&pct(
                "%s: %s (%d)",
                &[PctValue::Str(self.name.clone()), PctValue::Str(msg.clone()), int(*count)],
            )?);
        }
        Ok(out)
    }

    /// `self.query_ldc1612_cmd`, `None` until `_build_config` ran.
    fn query_ldc1612_cmd(&self, method: &str) -> KResult<CommandWrapper> {
        self.query_ldc1612_cmd.borrow().clone().ok_or_else(|| {
            internal("AttributeError", format!("'NoneType' object has no attribute '{method}'"))
        })
    }

    /// `LDC1612._start_measurements` (`ldc1612.py:196`).
    pub fn start_measurements(&self) -> KResult<()> {
        // In case of miswiring, testing LDC1612 device ID prevents treating
        // noise or wrong signal as a correctly initialized device
        let manuf_id = self.read_reg(REG_MANUFACTURER_ID)?;
        let dev_id = self.read_reg(REG_DEVICE_ID)?;
        if (manuf_id != LDC1612_MANUF_ID || dev_id != LDC1612_DEV_ID) && !self.mcu.is_fileoutput() {
            return Err(KlippyError::Command(pct(
                "Invalid ldc1612 id (got %x,%x vs %x,%x).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty ldc1612 chip.",
                &[int(manuf_id), int(dev_id), int(LDC1612_MANUF_ID), int(LDC1612_DEV_ID)],
            )?));
        }
        // Setup chip in requested query rate
        let rcount0 = (self.clock_freq as f64) / (16. * (self.data_rate as f64));
        self.set_reg(REG_RCOUNT0, kos::cconv::f64_to_i64(rcount0 + 0.5))?;
        self.set_reg(REG_OFFSET0, 0)?;
        self.set_reg(
            REG_SETTLECOUNT0,
            kos::cconv::f64_to_i64(SETTLETIME * (self.clock_freq as f64) / 16. + 0.5),
        )?;
        self.set_reg(REG_CLOCK_DIVIDERS0, (self.sensor_div << 12) | 1)?;
        self.set_reg(REG_ERROR_CONFIG, (0x1f << 11) | 1)?;
        self.set_reg(REG_MUX_CONFIG, 0x0208 | DEGLITCH)?;
        self.set_reg(REG_CONFIG, 0x001 | (1 << 12) | (1 << 10) | (1 << 9))?;
        self.set_reg(REG_DRIVE_CURRENT0, self.dccal.get_drive_current() << 11)?;
        // Start bulk reading
        let rest_ticks = self.mcu.seconds_to_clock(0.5 / (self.data_rate as f64));
        self.query_ldc1612_cmd("send")?
            .send(vec![py_int(self.oid), py_int(rest_ticks)], SendOpts::default())?;
        log::info(&pct("LDC1612 starting '%s' measurements", &[PctValue::Str(self.name.clone())])?);
        // Initialize clock tracking
        self.ffreader.note_start()?;
        self.last_error_count.set(0);
        Ok(())
    }

    /// `LDC1612._finish_measurements` (`ldc1612.py:225`).
    pub fn finish_measurements(&self) -> KResult<()> {
        // Halt bulk reading
        self.query_ldc1612_cmd("send_wait_ack")?
            .send_wait_ack(vec![py_int(self.oid), py_int(0)], SendOpts::default())?;
        self.ffreader.note_end()?;
        log::info(&pct("LDC1612 finished '%s' measurements", &[PctValue::Str(self.name.clone())])?);
        Ok(())
    }

    /// `LDC1612._process_batch(eventtime)` (`ldc1612.py:230`).
    pub fn process_batch(&self, _eventtime: f64) -> KResult<PyValue> {
        let raw = decode_raw_samples(self.ffreader.pull_samples()?)?;
        let mut samples = self.convert_samples(raw)?;
        if samples.is_empty() {
            return Ok(PyValue::Dict(IndexMap::new()));
        }
        if let Some(cal) = &self.calibration {
            cal.apply_calibration(&mut samples)?;
        }
        let data = samples
            .into_iter()
            .map(|(t, f, z)| {
                PyValue::Tuple(vec![PyValue::Float(t), PyValue::Float(f), PyValue::Float(z)])
            })
            .collect();
        let mut msg = IndexMap::new();
        msg.insert(PyValue::Str("data".to_string()), PyValue::List(data));
        msg.insert(PyValue::Str("errors".to_string()), py_int(self.last_error_count.get()));
        msg.insert(
            PyValue::Str("overflows".to_string()),
            py_int(self.ffreader.get_last_overflows()),
        );
        Ok(PyValue::Dict(msg))
    }

    /// `self.name` (`ldc1612.py:79`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.oid` (`ldc1612.py:89`).
    pub fn oid(&self) -> i64 {
        self.oid
    }

    /// `self.clock_freq` (`ldc1612.py:91`).
    pub fn clock_freq(&self) -> i64 {
        self.clock_freq
    }

    /// `self.sensor_div` (`ldc1612.py:95`).
    pub fn sensor_div(&self) -> i64 {
        self.sensor_div
    }

    /// `self.freq_conv` (`ldc1612.py:96`).
    pub fn freq_conv(&self) -> f64 {
        self.freq_conv
    }

    /// `self.last_error_count` (`ldc1612.py:122`).
    pub fn last_error_count(&self) -> i64 {
        self.last_error_count.get()
    }

    /// `self.dccal` (`ldc1612.py:81`).
    pub fn dccal(&self) -> &DriveCurrentCalibrate {
        &self.dccal
    }

    /// `self.ffreader` (`ldc1612.py:121`).
    pub fn ffreader(&self) -> &FixedFreqReader {
        &self.ffreader
    }

    /// `self.batch_bulk` (`ldc1612.py:124`).
    pub fn batch_bulk(&self) -> &BatchBulkHelper {
        &self.batch_bulk
    }
}

/// The calls `trigger_analog.MCU_trigger_analog` makes on its sensor.
impl TriggerAnalogSensor for LDC1612 {
    fn get_mcu(&self) -> Mcu {
        LDC1612::get_mcu(self)
    }
    fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        LDC1612::setup_trigger_analog(self, trigger_analog_oid)
    }
    fn get_samples_per_second(&self) -> f64 {
        self.data_rate as f64
    }
    fn lookup_sensor_error(&self, error_code: i64) -> KResult<String> {
        Ok(LDC1612::lookup_sensor_error(self, error_code))
    }
}

/// The `LDC1612` calls `probe_eddy_current` makes on its `sensor_helper`
/// beyond [`TriggerAnalogSensor`] (`probe_eddy_current.py:731,739,806,1076`).
pub trait LDC1612Api: TriggerAnalogSensor + 'static {
    /// `add_client(cb)` (`ldc1612.py:139`)
    fn add_client(&self, cb: ClientCallback) -> KResult<()>;
    /// `convert_raw_to_frequency(raw_value)` (`ldc1612.py:144`)
    fn convert_raw_to_frequency(&self, raw_value: f64) -> f64;
    /// `convert_frequency_to_raw(freq)` (`ldc1612.py:146`):
    /// `int(freq / freq_conv + 0.5)`.
    fn convert_frequency_to_raw(&self, freq: f64) -> KResult<i64>;
}

/// The calls `probe_eddy_current` makes on its `sensor_helper`.
impl LDC1612Api for LDC1612 {
    fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        LDC1612::add_client(self, cb)
    }
    fn convert_raw_to_frequency(&self, raw_value: f64) -> f64 {
        LDC1612::convert_raw_to_frequency(self, raw_value)
    }
    fn convert_frequency_to_raw(&self, freq: f64) -> KResult<i64> {
        LDC1612::convert_frequency_to_raw(self, freq)
    }
}

/// `ModuleFactory` for `ldc1612`: the Python module defines no loader.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
