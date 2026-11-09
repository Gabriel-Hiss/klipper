//! Port of the `ads1220` module (`klippy/extras/ads1220.py`).
//!
//! The ADS1220 is a 4-byte-wide SPI ADC used as a bulk sensor source
//! (for example by `load_cell`). The reference builds its transport
//! with `bus.MCU_SPI_from_config(config, 1, default_speed=...)` and
//! its reader/batching with `bulk_sensor.FixedFreqReader` and
//! `bulk_sensor.BatchBulkHelper`; this port does the same through
//! the merged [`crate::bus`] and [`crate::bulk_sensor`] ports. The
//! DSP-era `cs_pin` / `spi_speed` config handling lives inside
//! `mcu_spi_from_config`, so `load_config`-style assembly is the
//! constructor below, exactly as the reference's `__init__` does.
//!
//! `lookup_sensor_error` reproduces the reference behaviour exactly:
//! the format string contains no placeholder, so `"Unknown ads1220
//! error" % (error_code,)` raises `TypeError: not all arguments
//! converted during string formatting` in CPython. The port raises
//! the same exception instead of returning a string.
//!
//! The Python module defines no `load_config` / `load_config_prefix`
//! / `add_printer_objects` entry points: `load_cell` constructs
//! `ADS1220(config)` directly (`klippy/extras/load_cell.py:539`),
//! so [`module_factory`] is empty.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetChoiceOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandWrapper, LookupPinOpts, Mcu, PinsApi, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, GCell, PrinterObject};
use pycompat::text::round_py_nd;
use pycompat::{PyInt, PyValue};

use crate::bulk_sensor::{BatchBulkHelper, ClientCallback, FixedFreqReader};
use crate::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};

pub const BYTES_PER_SAMPLE: usize = 4;
/// `bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE` (`51 // 4`).
pub const MAX_SAMPLES_PER_MESSAGE: usize =
    (crate::bulk_sensor::MAX_BULK_MSG_SIZE as usize) / BYTES_PER_SAMPLE;
pub const UPDATE_INTERVAL: f64 = 0.10;
pub const RESET_CMD: u8 = 0x06;
pub const START_SYNC_CMD: u8 = 0x08;
pub const RREG_CMD: u8 = 0x20;
pub const WREG_CMD: u8 = 0x40;
pub const NOOP_CMD: u8 = 0x0;
pub const RESET_STATE: [u8; 4] = [0x0, 0x0, 0x0, 0x0];

/// `[0xff, 0x1]` spelling of `hexify` (`ads1220.py:23`).
pub fn hexify(byte_array: &[u8]) -> String {
    let parts: Vec<String> = byte_array.iter().map(|b| format!("{:#x}", b)).collect();
    format!("[{}]", parts.join(", "))
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
fn dict_get_bytes(params: &PyValue, key: &str) -> KResult<Vec<u8>> {
    match dict_get(params, key)? {
        PyValue::Bytes(data) => Ok(data),
        value => Err(internal(
            "TypeError",
            format!("a bytes-like object is required, not '{}'", pycompat::value::type_name(&value)),
        )),
    }
}

fn as_i64(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => n.0.to_string().parse::<i64>().map_err(|_| {
            internal("OverflowError", "int too large to convert to i64".to_string())
        }),
        PyValue::Bool(b) => Ok(i64::from(*b)),
        value => Err(internal("TypeError", pycompat::value::type_name(value).to_string())),
    }
}

fn as_f64(value: &PyValue) -> KResult<f64> {
    match value {
        PyValue::Float(f) => Ok(*f),
        PyValue::Int(n) => n.0.to_string().parse::<f64>().map_err(|_| {
            internal("OverflowError", "int too large to convert to float".to_string())
        }),
        PyValue::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        value => Err(internal(
            "TypeError",
            format!("unsupported operand type(s) for round(): '{}'", pycompat::value::type_name(value)),
        )),
    }
}

fn choice_map(pairs: &[(&str, i64)]) -> PyValue {
    let mut map = IndexMap::new();
    for (key, value) in pairs {
        map.insert(PyValue::Str((*key).to_string()), py_int(*value));
    }
    PyValue::Dict(map)
}

/// Value of `config.getchoice(option, choices, default=...)`
/// (`ads1220.py` chip options) as an integer.
fn getchoice_i64(config: &Config, option: &str, choices: &PyValue, default: &str) -> KResult<i64> {
    let value = config.getchoice(
        option,
        choices,
        GetChoiceOpts {
            default: Some(Some(PyValue::Str(default.to_string()))),
            note_valid: true,
        },
    )?;
    as_i64(&value)
}

/// State shared with the `BatchBulkHelper` callbacks, which are
/// created before the `Ads1220` itself exists.
struct Ads1220Shared {
    printer: Printer,
    name: String,
    last_error_count: GCell<i64>,
    consecutive_fails: GCell<i64>,
    gain: i64,
    sps: i64,
    is_turbo: bool,
    mux: i64,
    pga_bypass: bool,
    vref: i64,
    spi: Arc<McuSpi>,
    mcu: Mcu,
    oid: i64,
    data_ready_pin: String,
    ffreader: FixedFreqReader,
    query_ads1220_cmd: GCell<Option<CommandWrapper>>,
}

impl Ads1220Shared {
    fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        let cmd = self.mcu.lookup_command("query_ads1220 oid=%c rest_ticks=%u", Some(cmdqueue))?;
        *self.query_ads1220_cmd.borrow_mut() = Some(cmd);
        self.ffreader.setup_query_command("query_ads1220_status oid=%c", self.oid, Some(cmdqueue))
    }

    fn send_command(&self, cmd: u8) -> KResult<()> {
        self.spi.spi_send(&[cmd], SendOpts::default())
    }

    fn read_reg(&self, reg: i64, byte_count: i64) -> KResult<Vec<u8>> {
        let mut read_command = vec![RREG_CMD | ((reg as u8) << 2) | ((byte_count as u8) - 1)];
        // `[NOOP_CMD] * byte_count`; Python's list multiplication with a
        // negative count yields an empty list.
        if byte_count > 0 {
            read_command.extend(std::iter::repeat_n(NOOP_CMD, byte_count as usize));
        }
        let params = self.spi.spi_transfer(&read_command, QuerySendOpts::default())?;
        // Python: bytearray(params['response'][1:]).
        let response = dict_get_bytes(&params, "response")?;
        if response.len() <= 1 {
            Ok(Vec::new())
        } else {
            Ok(response[1..].to_vec())
        }
    }

    fn write_reg(&self, reg: i64, register_bytes: &[u8]) -> KResult<()> {
        let mut write_command = vec![
            WREG_CMD | ((reg as u8) << 2) | ((register_bytes.len() as u8) - 1),
        ];
        write_command.extend_from_slice(register_bytes);
        self.spi.spi_send(&write_command, SendOpts::default())?;
        let stored_val = self.read_reg(reg, register_bytes.len() as i64)?;
        if stored_val != register_bytes {
            if self.mcu.is_fileoutput() {
                return Ok(());
            }
            return Err(KlippyError::command(format!(
                "Failed to set ADS1220 register [{:#x}] to {}: got {}. This may be a connection problem (e.g. faulty wiring)",
                reg,
                hexify(register_bytes),
                hexify(&stored_val)
            )));
        }
        Ok(())
    }

    fn reset_chip(&self) -> KResult<()> {
        // the reset command takes 50us to complete
        self.send_command(RESET_CMD)?;
        // read startup register state and validate
        let val = self.read_reg(0x0, 4)?;
        if val != RESET_STATE {
            if self.mcu.is_fileoutput() {
                return Ok(());
            }
            return Err(KlippyError::command(format!(
                "Invalid ads1220 reset state (got {} vs {}).\nThis is generally indicative of connection problems\n(e.g. faulty wiring) or a faulty ADS1220 chip.",
                hexify(&val),
                hexify(&RESET_STATE)
            )));
        }
        Ok(())
    }

    fn setup_chip(&self) -> KResult<()> {
        let continuous: i64 = 0x1; // enable continuous conversions
        let mode: i64 = if self.is_turbo { 0x2 } else { 0x0 }; // turbo mode
        let sps_list = if self.is_turbo {
            &ADS1220_SPS_TURBO
        } else {
            &ADS1220_SPS_NORMAL
        };
        let data_rate = sps_list
            .iter()
            .position(|(k, _)| k == &self.sps.to_string())
            .map(|i| i as i64)
            .unwrap_or(0);
        let pga_int: i64 = if self.pga_bypass { 1 } else { 0 };
        let reg_values: Vec<u8> = vec![
            ((self.mux << 4) | (self.gain << 1) | pga_int) as u8,
            ((data_rate << 5) | (mode << 3) | (continuous << 2)) as u8,
            (self.vref << 6) as u8,
            0x0,
        ];
        self.write_reg(0x0, &reg_values)?;
        // start measurements immediately
        self.send_command(START_SYNC_CMD)
    }

    fn start_measurements(&self) -> KResult<()> {
        self.last_error_count.set(0);
        self.consecutive_fails.set(0);
        // Start bulk reading
        self.reset_chip()?;
        self.setup_chip()?;
        let rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * (self.sps as f64)));
        let cmd = self.query_ads1220_cmd.borrow().clone().ok_or_else(|| attr_error("send"))?;
        cmd.send(vec![py_int(self.oid), py_int(rest_ticks)], SendOpts::default())?;
        klippy_api::log::info(&format!("ADS1220 starting '{}' measurements", self.name));
        // Initialize clock tracking
        self.ffreader.note_start()
    }

    fn finish_measurements(&self) -> KResult<()> {
        // don't use serial connection after shutdown
        if self.printer.is_shutdown() {
            return Ok(());
        }
        // Halt bulk reading
        let cmd =
            self.query_ads1220_cmd.borrow().clone().ok_or_else(|| attr_error("send_wait_ack"))?;
        cmd.send_wait_ack(vec![py_int(self.oid), py_int(0)], SendOpts::default())?;
        self.ffreader.note_end()?;
        klippy_api::log::info(&format!("ADS1220 finished '{}' measurements", self.name));
        Ok(())
    }

    fn process_batch(&self, _eventtime: f64) -> KResult<PyValue> {
        let mut samples = self.ffreader.pull_samples()?;
        Ads1220::convert_samples(&mut samples)?;
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("data".to_string()), PyValue::List(samples));
        map.insert(
            PyValue::Str("errors".to_string()),
            py_int(self.last_error_count.get()),
        );
        map.insert(
            PyValue::Str("overflows".to_string()),
            py_int(self.ffreader.get_last_overflows()),
        );
        Ok(PyValue::Dict(map))
    }
}

/// `self.sps_normal` (`ads1220.py:39`), in reference order.
const ADS1220_SPS_NORMAL: [(&str, i64); 7] = [
    ("20", 20),
    ("45", 45),
    ("90", 90),
    ("175", 175),
    ("330", 330),
    ("600", 600),
    ("1000", 1000),
];
/// `self.sps_turbo` (`ads1220.py:41`), in reference order.
const ADS1220_SPS_TURBO: [(&str, i64); 7] = [
    ("40", 40),
    ("90", 90),
    ("180", 180),
    ("350", 350),
    ("660", 660),
    ("1200", 1200),
    ("2000", 2000),
];

/// Port of `klippy/extras/ads1220.py:ADS1220`.
pub struct Ads1220 {
    shared: Arc<Ads1220Shared>,
    batch: BatchBulkHelper,
}

impl Ads1220 {
    /// `ADS1220.__init__` (`ads1220.py:28-103`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .map(|s| s.to_string())
            .unwrap_or_default();

        // Chip options
        // Gain
        let gain = getchoice_i64(
            config,
            "gain",
            &choice_map(&[
                ("1", 0x0),
                ("2", 0x1),
                ("4", 0x2),
                ("8", 0x3),
                ("16", 0x4),
                ("32", 0x5),
                ("64", 0x6),
                ("128", 0x7),
            ]),
            "128",
        )?;
        // Sample rate
        // `sps_options = sps_normal.copy(); sps_options.update(sps_turbo)`:
        // an updated key (e.g. "90") keeps its normal position with the
        // turbo value, exactly like a Python dict update.
        let sps_choices = {
            let mut merged: IndexMap<String, i64> = IndexMap::new();
            for (key, value) in ADS1220_SPS_NORMAL.iter().chain(ADS1220_SPS_TURBO.iter()) {
                merged.insert((*key).to_string(), *value);
            }
            let mut map = IndexMap::new();
            for (key, value) in &merged {
                map.insert(PyValue::Str(key.clone()), py_int(*value));
            }
            PyValue::Dict(map)
        };
        let sps_value = config.getchoice(
            "sample_rate",
            &sps_choices,
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("660".to_string()))),
                note_valid: true,
            },
        )?;
        let sps = as_i64(&sps_value)?;
        let is_turbo = ADS1220_SPS_TURBO.iter().any(|(k, _)| k == &sps.to_string());
        // Input multiplexer: AINP and AINN
        let mux = getchoice_i64(
            config,
            "input_mux",
            &choice_map(&[
                ("AIN0_AIN1", 0b0000),
                ("AIN0_AIN2", 0b0001),
                ("AIN0_AIN3", 0b0010),
                ("AIN1_AIN2", 0b0011),
                ("AIN1_AIN3", 0b0100),
                ("AIN2_AIN3", 0b0101),
                ("AIN1_AIN0", 0b0110),
                ("AIN3_AIN2", 0b0111),
                ("AIN0_AVSS", 0b1000),
                ("AIN1_AVSS", 0b1001),
                ("AIN2_AVSS", 0b1010),
                ("AIN3_AVSS", 0b1011),
            ]),
            "AIN0_AIN1",
        )?;
        // PGA Bypass
        let pga_bypass_config = config.getboolean(
            "pga_bypass",
            GetBoolOpts { default: Some(Some(false)), note_valid: true, ..Default::default() },
        )?;
        // bypass PGA when AVSS is the negative input
        let force_pga_bypass = mux >= 0b1000;
        let pga_bypass = force_pga_bypass || pga_bypass_config.unwrap_or(false);
        // Voltage Reference
        let vref = getchoice_i64(
            config,
            "vref",
            &choice_map(&[
                ("internal", 0b0),
                ("REF0", 0b01),
                ("REF1", 0b10),
                ("analog_supply", 0b11),
            ]),
            "internal",
        )?;
        // check for conflict between REF1 and AIN0/AIN3
        let mux_conflict = [0b0000, 0b0001, 0b0010, 0b0100, 0b0101, 0b0110, 0b0111, 0b1000, 0b1011];
        if vref == 0b10 && mux_conflict.contains(&mux) {
            return Err(KlippyError::config(
                "ADS1220 config error: AIN0/REFP1 and AIN3/REFN1 cant be used as a voltage reference and an input at the same time",
            ));
        }
        // SPI Setup
        let spi_speed = if is_turbo { 512000 } else { 256000 };
        let spi = mcu_spi_from_config(config, 1, "cs_pin", spi_speed, None, false)?;
        let mcu = spi.get_mcu();
        let oid = mcu.create_oid()?;
        // Data Ready (DRDY) Pin
        let drdy_pin = config.get("data_ready_pin", GetStrOpts::default())?.expect("required");
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .ok_or_else(|| {
                KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "printer object 'pins' does not provide dyn PinsApi".to_string(),
                }
            })?;
        let drdy_ppin = ppins.lookup_pin(
            &drdy_pin,
            LookupPinOpts { can_invert: false, can_pullup: false, share_type: None },
        )?;
        let data_ready_pin = drdy_ppin.pin.clone();
        let drdy_pin_mcu = drdy_ppin.chip.clone();
        if !Arc::ptr_eq(&drdy_pin_mcu, &mcu) {
            return Err(KlippyError::config(
                "ADS1220 config error: SPI communication and data_ready_pin must be on the same MCU",
            ));
        }
        // Clock tracking
        let chip_smooth = (sps as f64) * UPDATE_INTERVAL * 2.0;
        // Measurement conversion
        let ffreader = FixedFreqReader::new(mcu.clone(), chip_smooth, "<i")?;
        let shared = Arc::new(Ads1220Shared {
            printer: printer.clone(),
            name,
            last_error_count: GCell::new(0),
            consecutive_fails: GCell::new(0),
            gain,
            sps,
            is_turbo,
            mux,
            pga_bypass,
            vref,
            spi: spi.clone(),
            mcu: mcu.clone(),
            oid,
            data_ready_pin,
            ffreader,
            query_ads1220_cmd: GCell::new(None),
        });
        // Process messages in batches
        let batch_cb = {
            let shared = shared.clone();
            Box::new(move |eventtime: f64| shared.process_batch(eventtime))
        };
        let start_cb = {
            let shared = shared.clone();
            Box::new(move || shared.start_measurements())
        };
        let stop_cb = {
            let shared = shared.clone();
            Box::new(move || shared.finish_measurements())
        };
        let batch = BatchBulkHelper::new(
            printer.clone(),
            batch_cb,
            Some(start_cb),
            Some(stop_cb),
            UPDATE_INTERVAL,
        );
        // Command Configuration
        let cmd = format!(
            "config_ads1220 oid={} spi_oid={} data_ready_pin={}",
            shared.oid,
            shared.spi.get_oid(),
            shared.data_ready_pin
        );
        shared.mcu.add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        shared.mcu.add_config_cmd(
            &format!("query_ads1220 oid={} rest_ticks=0", shared.oid),
            AddConfigCmdOpts { is_init: false, on_restart: true },
        )?;
        let registration = shared.clone();
        shared.mcu.register_config_callback(Box::new(move || registration.build_config()))?;
        Ok(Arc::new(Ads1220 { shared, batch }))
    }

    /// `self.gain` (`ads1220.py:37`).
    pub fn gain(&self) -> i64 {
        self.shared.gain
    }

    /// `self.sps` (`ads1220.py:45`).
    pub fn sps(&self) -> i64 {
        self.shared.sps
    }

    /// `self.is_turbo` (`ads1220.py:47`).
    pub fn is_turbo(&self) -> bool {
        self.shared.is_turbo
    }

    /// `self.mux` (`ads1220.py:55`).
    pub fn mux(&self) -> i64 {
        self.shared.mux
    }

    /// `self.pga_bypass` (`ads1220.py:58-61`).
    pub fn pga_bypass(&self) -> bool {
        self.shared.pga_bypass
    }

    /// `self.vref` (`ads1220.py:65`).
    pub fn vref(&self) -> i64 {
        self.shared.vref
    }

    /// `self.name` (`ads1220.py:30`).
    pub fn name(&self) -> &str {
        &self.shared.name
    }

    /// `self.data_ready_pin` (`ads1220.py:83`).
    pub fn data_ready_pin(&self) -> &str {
        &self.shared.data_ready_pin
    }

    /// `setup_trigger_analog(trigger_analog_oid)` (`ads1220.py:105`).
    pub fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        self.shared.mcu.add_config_cmd(
            &format!(
                "ads1220_attach_trigger_analog oid={} trigger_analog_oid={}",
                self.shared.oid, trigger_analog_oid
            ),
            AddConfigCmdOpts { is_init: true, on_restart: false },
        )
    }

    /// `get_mcu()` (`ads1220.py:117`).
    pub fn get_mcu(&self) -> Mcu {
        self.shared.mcu.clone()
    }

    /// `get_samples_per_second()` (`ads1220.py:120`).
    pub fn get_samples_per_second(&self) -> i64 {
        self.shared.sps
    }

    /// `get_status(eventtime)` (`ads1220.py:123`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("errors".to_string()),
            py_int(self.shared.last_error_count.get()),
        );
        map.insert(
            PyValue::Str("overflows".to_string()),
            py_int(self.shared.ffreader.get_last_overflows()),
        );
        map.insert(
            PyValue::Str("sample_rate".to_string()),
            py_int(self.get_samples_per_second()),
        );
        PyValue::Dict(map)
    }

    /// `lookup_sensor_error(error_code)` (`ads1220.py:130`).
    ///
    /// Reference quirk: the format string has no placeholder, so the
    /// `%` operation raises `TypeError` instead of returning a string.
    pub fn lookup_sensor_error(&self, _error_code: i64) -> KResult<String> {
        // "Unknown ads1220 error" % (error_code,)
        Err(internal(
            "TypeError",
            "not all arguments converted during string formatting".to_string(),
        ))
    }

    /// `get_range()` (`ads1220.py:135`): the tuple of the sensor's
    /// minimum and maximum value, used to detect saturation.
    pub fn get_range(&self) -> (i64, i64) {
        (-0x800000, 0x7FFFFF)
    }

    /// `add_client(callback)` (`ads1220.py:139`): direct pass through
    /// to the `bulk_sensor` API.
    pub fn add_client(&self, callback: ClientCallback) -> KResult<()> {
        self.batch.add_client(callback)
    }

    /// `_convert_samples(samples)` (`ads1220.py:143`): pure conversion.
    pub fn convert_samples(samples: &mut [PyValue]) -> KResult<()> {
        let adc_factor = 1.0 / 8388608.0_f64; // 1. / (1 << 23)
        // `del samples[count:]` in the reference is a no-op: the loop
        // covers every element, so `count` always ends at `len()`.
        for sample in samples.iter_mut() {
            let (ptime_value, val) = match &*sample {
                PyValue::Tuple(items) => match items.len() {
                    2 => (items[0].clone(), items[1].clone()),
                    got => {
                        return Err(internal(
                            "ValueError",
                            if got < 2 {
                                format!("not enough values to unpack (expected 2, got {got})")
                            } else {
                                "too many values to unpack (expected 2)".to_string()
                            },
                        ))
                    }
                },
                other => {
                    return Err(internal(
                        "TypeError",
                        format!(
                            "cannot unpack non-iterable {} object",
                            pycompat::value::type_name(other)
                        ),
                    ))
                }
            };
            let ptime = as_f64(&ptime_value)?;
            let val_f = as_f64(&val)?;
            let converted_ptime = match &ptime_value {
                // `round(int, 6)` returns the int unchanged.
                PyValue::Int(_) => ptime_value.clone(),
                _ => PyValue::Float(round_py_nd(ptime, 6)),
            };
            *sample = PyValue::Tuple(vec![
                converted_ptime,
                val,
                PyValue::Float(round_py_nd(val_f * adc_factor, 9)),
            ]);
        }
        Ok(())
    }

    /// `_start_measurements()` (`ads1220.py:152`).
    pub fn start_measurements(&self) -> KResult<()> {
        self.shared.start_measurements()
    }

    /// `_finish_measurements()` (`ads1220.py:164`).
    pub fn finish_measurements(&self) -> KResult<()> {
        self.shared.finish_measurements()
    }

    /// `_process_batch(eventtime)` (`ads1220.py:173`).
    pub fn process_batch(&self, eventtime: f64) -> KResult<PyValue> {
        self.shared.process_batch(eventtime)
    }

    /// `reset_chip()` (`ads1220.py:179`).
    pub fn reset_chip(&self) -> KResult<()> {
        self.shared.reset_chip()
    }

    /// `setup_chip()` (`ads1220.py:193`).
    pub fn setup_chip(&self) -> KResult<()> {
        self.shared.setup_chip()
    }

    /// `read_reg(reg, byte_count)` (`ads1220.py:206`).
    pub fn read_reg(&self, reg: i64, byte_count: i64) -> KResult<Vec<u8>> {
        self.shared.read_reg(reg, byte_count)
    }

    /// `send_command(cmd)` (`ads1220.py:212`).
    pub fn send_command(&self, cmd: u8) -> KResult<()> {
        self.shared.send_command(cmd)
    }

    /// `write_reg(reg, register_bytes)` (`ads1220.py:215`).
    pub fn write_reg(&self, reg: i64, register_bytes: &[u8]) -> KResult<()> {
        self.shared.write_reg(reg, register_bytes)
    }

    /// The `BatchBulkHelper` driving this sensor (`ads1220.py:93`).
    pub fn batch_helper(&self) -> &BatchBulkHelper {
        &self.batch
    }

    /// `self.query_ads1220_cmd` (`ads1220.py:103`): the query command
    /// wrapper built in `_build_config`, if it has run.
    pub fn query_ads1220_cmd(&self) -> Option<CommandWrapper> {
        self.shared.query_ads1220_cmd.borrow().clone()
    }
}

impl PrinterObject for Ads1220 {
    klippy_api::printer_object_base!();
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(Ads1220::get_status(self, eventtime))
    }
}

/// `ADS1220` is the `trigger_analog` sensor of `load_cell_probe`
/// (`ads1220.py:105-131`, used through `trigger_analog.MCU_trigger_analog`).
impl crate::trigger_analog::TriggerAnalogSensor for Ads1220 {
    fn get_mcu(&self) -> Mcu {
        Ads1220::get_mcu(self)
    }
    fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        Ads1220::setup_trigger_analog(self, trigger_analog_oid)
    }
    fn get_samples_per_second(&self) -> f64 {
        Ads1220::get_samples_per_second(self) as f64
    }
    fn lookup_sensor_error(&self, error_code: i64) -> KResult<String> {
        Ads1220::lookup_sensor_error(self, error_code)
    }
}

/// `ADS1220_SENSOR_TYPE = {"ads1220": ADS1220}` (`ads1220.py:229`).
pub const SENSOR_TYPE_NAME: &str = "ads1220";

/// The Python module defines no `load_config` / `load_config_prefix` /
/// `add_printer_objects` entry points (the class is constructed
/// directly by `load_cell`), so the factory is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
