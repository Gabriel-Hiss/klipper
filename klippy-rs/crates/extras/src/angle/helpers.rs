//! Sensor helpers (`klippy/extras/angle.py:267-603`).
//!
//! `HelperA1333`, `HelperAS5047D`, `HelperTLE5012B`, `HelperMT6816`
//! and `HelperMT6826S`. The chip SPI clocks and the debug register
//! commands are reproduced byte for byte; responder dicts from the
//! `spi_transfer` / `spi_angle_transfer` query wrappers are read as
//! `PyValue` dicts.

use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCmd, GetOpts, GetParser, GCodeApi, GCodeCommandHandler};
use klippy_api::mcu::{CommandQueryWrapper, Mcu, QuerySendOpts};
use klippy_api::printer::{LookupDefault, Printer};
use klippy_api::lookup_interface;
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

use super::{
    dict_get, fmt_const, index_error, name_last_token, pct_int, pct_str, SensorKind,
};
use crate::bus::{McuSpi, McuSpiApi};
use crate::force_move::ForceMoveApi;
use crate::angle::calibration::get_microsteps_from_configfile;

/// The helper surface `Angle` uses (duck-typed in Python).
pub trait SensorHelperApi: Send + Sync + 'static {
    /// `is_tcode_absolute`.
    fn is_tcode_absolute(&self) -> bool;
    /// `last_temperature`.
    fn last_temperature(&self) -> Option<f64>;
    /// Sets `last_temperature` (`_finish_measurements` clears it).
    fn set_last_temperature(&self, temp: Option<f64>);
    /// `get_static_delay()` — absent on `HelperTLE5012B`.
    fn get_static_delay(&self) -> KResult<f64>;
    /// `get_tcode_params()` — only on `HelperTLE5012B`.
    fn get_tcode_params(&self) -> KResult<(i64, i64, f64)>;
    /// `start()` (lines 276, 289, 389, 447, 530).
    fn start(&self) -> KResult<()>;
    /// `update_clock()` — only on `HelperTLE5012B`.
    fn update_clock(&self) -> KResult<()>;
}

/// `SPI_MODE` per kind (lines 268, 281, 296, 415, 461).
pub fn spi_mode(kind: SensorKind) -> i64 {
    match kind {
        SensorKind::A1333 => 3,
        SensorKind::AS5047D => 1,
        SensorKind::TLE5012B => 1,
        SensorKind::MT6816 => 3,
        SensorKind::MT6826S => 3,
    }
}

/// `SPI_SPEED` per kind (lines 269, 282, 297, 416, 462).
pub fn spi_speed(kind: SensorKind) -> i64 {
    match kind {
        SensorKind::A1333 => 10_000_000,
        SensorKind::AS5047D => int_one_div_trunc(0.000000350),
        SensorKind::TLE5012B => 4_000_000,
        SensorKind::MT6816 => 10_000_000,
        SensorKind::MT6826S => 10_000_000,
    }
}

/// `int(1. / .000000350)` (line 282): truncation toward zero.
fn int_one_div_trunc(d: f64) -> i64 {
    super::py_trunc_i64(1.0 / d)
}

/// Builds the helper for `sensor_type` (line 626).
pub fn create_helper(
    kind: SensorKind,
    config: &Config,
    spi: Arc<McuSpi>,
    oid: i64,
) -> KResult<Arc<dyn SensorHelperApi>> {
    match kind {
        SensorKind::A1333 => {
            Ok(HelperA1333::new(config, spi, oid) as Arc<dyn SensorHelperApi>)
        }
        SensorKind::AS5047D => {
            Ok(HelperAS5047D::new(config, spi, oid) as Arc<dyn SensorHelperApi>)
        }
        SensorKind::TLE5012B => Ok(
            HelperTLE5012B::new(config, spi, oid)? as Arc<dyn SensorHelperApi>,
        ),
        SensorKind::MT6816 => Ok(
            HelperMT6816::new(config, spi, oid)? as Arc<dyn SensorHelperApi>,
        ),
        SensorKind::MT6826S => Ok(
            HelperMT6826S::new(config, spi, oid)? as Arc<dyn SensorHelperApi>,
        ),
    }
}

fn no_static_delay() -> KlippyError {
    super::no_attribute_error("HelperTLE5012B", "get_static_delay")
}
fn no_tcode_params() -> KlippyError {
    super::no_attribute_error("SensorHelper", "get_tcode_params")
}
fn no_update_clock() -> KlippyError {
    super::no_attribute_error("SensorHelper", "update_clock")
}

/// Request `oid` / `clock` / `response` from a query-wrapper answer.
fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
    let resp = dict_get(params, "response")?;
    match resp {
        PyValue::Bytes(b) => Ok(b.clone()),
        PyValue::List(items) => Ok(items
            .iter()
            .map(|v| super::pyvalue_to_i64(v).unwrap_or(0).rem_euclid(256) as u8)
            .collect()),
        _ => Ok(Vec::new()),
    }
}

fn response_clock(params: &PyValue) -> KResult<i64> {
    let clock = dict_get(params, "clock")?;
    Ok(super::pyvalue_to_i64(clock).unwrap_or(0))
}

fn resp_byte(resp: &[u8], idx: usize) -> KResult<u8> {
    resp.get(idx).copied().ok_or_else(index_error)
}

/// `gcmd.get("REG", minval=0, maxval=X, parser=lambda x: int(x, 0))`.
pub fn gcmd_reg(gcmd: &GCmd, maxval: i64) -> KResult<i64> {
    let v = gcmd.get(
        "REG",
        GetOpts {
            default: None,
            parser: GetParser::IntAutoBase,
            minval: Some(PyValue::Int(PyInt::from_i64(0))),
            maxval: Some(PyValue::Int(PyInt::from_i64(maxval))),
            above: None,
            below: None,
        },
    )?;
    Ok(super::pyvalue_to_i64(&v).unwrap_or(0))
}

/// `gcmd.get("VAL", minval=0, maxval=0xffff, parser=lambda x: int(x, 0))`.
pub fn gcmd_val(gcmd: &GCmd) -> KResult<i64> {
    let v = gcmd.get(
        "VAL",
        GetOpts {
            default: None,
            parser: GetParser::IntAutoBase,
            minval: Some(PyValue::Int(PyInt::from_i64(0))),
            maxval: Some(PyValue::Int(PyInt::from_i64(0xffff))),
            above: None,
            below: None,
        },
    )?;
    Ok(super::pyvalue_to_i64(&v).unwrap_or(0))
}

/// `cmd_ANGLE_DEBUG_READ_help` (lines 402, 449, 577).
pub const ANGLE_DEBUG_READ_HELP: &str = "Query low-level angle sensor register";
/// `cmd_ANGLE_DEBUG_WRITE_help` (line 407).
pub const ANGLE_DEBUG_WRITE_HELP: &str = "Set low-level angle sensor register";
/// `cmd_ANGLE_CHIP_CALIBRATE_help` (line 541).
pub const ANGLE_CHIP_CALIBRATE_HELP: &str = "Run MT6826s calibration sequence";

// ---------------------------------------------------------------------------
// HelperA1333 (lines 267-278)
// ---------------------------------------------------------------------------

/// `class HelperA1333` (lines 267-278).
pub struct HelperA1333 {
    spi: Arc<McuSpi>,
    last_temperature: GCell<Option<f64>>,
}

impl HelperA1333 {
    /// `HelperA1333(config, spi, oid)` (lines 270-273).
    pub fn new(_config: &Config, spi: Arc<McuSpi>, _oid: i64) -> Arc<HelperA1333> {
        Arc::new(HelperA1333 {
            spi,
            last_temperature: GCell::new(None),
        })
    }
}

impl SensorHelperApi for HelperA1333 {
    fn is_tcode_absolute(&self) -> bool {
        false
    }
    fn last_temperature(&self) -> Option<f64> {
        self.last_temperature.get()
    }
    fn set_last_temperature(&self, temp: Option<f64>) {
        self.last_temperature.replace(temp);
    }
    fn get_static_delay(&self) -> KResult<f64> {
        Ok(0.000001)
    }
    fn get_tcode_params(&self) -> KResult<(i64, i64, f64)> {
        Err(no_tcode_params())
    }
    fn start(&self) -> KResult<()> {
        // Setup for angle query (line 278).
        self.spi.spi_transfer(&[0x32, 0x00], QuerySendOpts::default())?;
        Ok(())
    }
    fn update_clock(&self) -> KResult<()> {
        Err(no_update_clock())
    }
}

// ---------------------------------------------------------------------------
// HelperAS5047D (lines 280-293)
// ---------------------------------------------------------------------------

/// `class HelperAS5047D` (lines 280-293).
pub struct HelperAS5047D {
    spi: Arc<McuSpi>,
    last_temperature: GCell<Option<f64>>,
}

impl HelperAS5047D {
    /// `HelperAS5047D(config, spi, oid)` (lines 283-286).
    pub fn new(_config: &Config, spi: Arc<McuSpi>, _oid: i64) -> Arc<HelperAS5047D> {
        Arc::new(HelperAS5047D {
            spi,
            last_temperature: GCell::new(None),
        })
    }
}

impl SensorHelperApi for HelperAS5047D {
    fn is_tcode_absolute(&self) -> bool {
        false
    }
    fn last_temperature(&self) -> Option<f64> {
        self.last_temperature.get()
    }
    fn set_last_temperature(&self, temp: Option<f64>) {
        self.last_temperature.replace(temp);
    }
    fn get_static_delay(&self) -> KResult<f64> {
        Ok(0.000100)
    }
    fn get_tcode_params(&self) -> KResult<(i64, i64, f64)> {
        Err(no_tcode_params())
    }
    fn start(&self) -> KResult<()> {
        // Clear any errors from device (lines 291-293).
        self.spi.spi_transfer(&[0xff, 0xfc], QuerySendOpts::default())?; // Read DIAAGC
        self.spi.spi_transfer(&[0x40, 0x01], QuerySendOpts::default())?; // Read ERRFL
        self.spi.spi_transfer(&[0xc0, 0x00], QuerySendOpts::default())?; // Read NOP
        Ok(())
    }
    fn update_clock(&self) -> KResult<()> {
        Err(no_update_clock())
    }
}

// ---------------------------------------------------------------------------
// HelperTLE5012B (lines 295-412)
// ---------------------------------------------------------------------------

struct TleState {
    last_chip_mcu_clock: i64,
    last_chip_clock: i64,
    chip_freq: f64,
}

/// `class HelperTLE5012B` (lines 295-412).
pub struct HelperTLE5012B {
    printer: Printer,
    spi: Arc<McuSpi>,
    oid: i64,
    mcu: Mcu,
    spi_angle_transfer_cmd: GCell<Option<CommandQueryWrapper>>,
    state: GCell<TleState>,
    last_temperature: GCell<Option<f64>>,
}

impl HelperTLE5012B {
    /// `HelperTLE5012B(config, spi, oid)` (lines 298-316).
    pub fn new(
        config: &Config,
        spi: Arc<McuSpi>,
        oid: i64,
    ) -> KResult<Arc<HelperTLE5012B>> {
        let this = Arc::new(HelperTLE5012B {
            printer: config.get_printer(),
            spi: spi.clone(),
            oid,
            mcu: spi.get_mcu(),
            spi_angle_transfer_cmd: GCell::new(None),
            state: GCell::new(TleState {
                last_chip_mcu_clock: 0,
                last_chip_clock: 0,
                chip_freq: 0.0,
            }),
            last_temperature: GCell::new(None),
        });
        {
            let cb = Arc::clone(&this);
            this.mcu.register_config_callback(Box::new(move || cb.build_config()))?;
        }
        let name = name_last_token(&config.get_name());
        let gcode = lookup_interface::<dyn GCodeApi>(
            &*this.printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        {
            let cb = Arc::clone(&this);
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd: &GCmd| cb.cmd_angle_debug_read(gcmd));
            gcode.register_mux_command(
                "ANGLE_DEBUG_READ",
                "CHIP",
                Some(&name),
                handler,
                Some(ANGLE_DEBUG_READ_HELP),
            )?;
        }
        {
            let cb = Arc::clone(&this);
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd: &GCmd| cb.cmd_angle_debug_write(gcmd));
            gcode.register_mux_command(
                "ANGLE_DEBUG_WRITE",
                "CHIP",
                Some(&name),
                handler,
                Some(ANGLE_DEBUG_WRITE_HELP),
            )?;
        }
        Ok(this)
    }

    /// `_build_config` (lines 317-322).
    pub fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        let cmd = self.mcu.lookup_query_command(
            "spi_angle_transfer oid=%c data=%*s",
            "spi_angle_transfer_response oid=%c clock=%u response=%*s",
            klippy_api::mcu::LookupQueryCommandOpts {
                oid: Some(self.oid),
                cq: Some(cmdqueue),
                is_async: false,
            },
        )?;
        self.spi_angle_transfer_cmd.replace(Some(cmd));
        Ok(())
    }

    /// `_calc_crc(data)` (lines 325-334). Python ints are unbounded;
    /// `crc` stays below 512 here, carried as `i64` throughout.
    pub fn calc_crc(&self, data: &[u8]) -> i64 {
        let mut crc: i64 = 0xff;
        for d in data {
            crc ^= *d as i64;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ 0x1d;
                } else {
                    crc <<= 1;
                }
            }
        }
        (!crc) & 0xff
    }

    /// `_send_spi(msg)` (lines 335-345).
    pub fn send_spi(&self, msg: &[u8]) -> KResult<PyValue> {
        for _retry in 0..5 {
            let params: PyValue = if msg[0] & 0x04 != 0 {
                let cmd = self
                    .spi_angle_transfer_cmd
                    .borrow()
                    .clone()
                    .ok_or_else(|| super::no_attribute_error("NoneType", "send"))?;
                cmd.send(
                    vec![
                        PyValue::Int(PyInt::from_i64(self.oid)),
                        PyValue::Bytes(msg.to_vec()),
                    ],
                    QuerySendOpts::default(),
                )?
            } else {
                self.spi.spi_transfer(msg, QuerySendOpts::default())?
            };
            let resp = response_bytes(&params)?;
            let mut crc_data: Vec<u8> = msg[..2].to_vec();
            if resp.len() > 3 {
                crc_data.extend_from_slice(&resp[2..resp.len() - 2]);
            }
            let crc = self.calc_crc(&crc_data);
            let last = *resp.last().ok_or_else(index_error)? as i64;
            if crc == last {
                return Ok(params);
            }
        }
        Err(KlippyError::command("Unable to query tle5012b chip"))
    }

    /// `_read_reg(reg)` (lines 346-353).
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let mut cw: i64 = 0x8000 | ((reg & 0x3f) << 4) | 0x01;
        if (0x05..=0x11).contains(&reg) {
            cw |= 0x5000;
        }
        let msg: [u8; 6] = [
            (cw >> 8) as u8,
            (cw & 0xff) as u8,
            0,
            0,
            0,
            0,
        ];
        let params = self.send_spi(&msg)?;
        let resp = response_bytes(&params)?;
        Ok(((resp_byte(&resp, 2)? as i64) << 8) | (resp_byte(&resp, 3)? as i64))
    }

    /// `_write_reg(reg, val)` (lines 354-364).
    pub fn write_reg(&self, reg: i64, val: i64) -> KResult<()> {
        let mut cw: i64 = ((reg & 0x3f) << 4) | 0x01;
        if (0x05..=0x11).contains(&reg) {
            cw |= 0x5000;
        }
        let msg: [u8; 6] = [
            (cw >> 8) as u8,
            (cw & 0xff) as u8,
            ((val >> 8) & 0xff) as u8,
            (val & 0xff) as u8,
            0,
            0,
        ];
        for _retry in 0..5 {
            self.send_spi(&msg)?;
            let rval = self.read_reg(reg)?;
            if rval == val {
                return Ok(());
            }
        }
        Err(KlippyError::command("Unable to write to tle5012b chip"))
    }

    /// `_mask_reg(reg, off, on)` (lines 365-367).
    pub fn mask_reg(&self, reg: i64, off: i64, on: i64) -> KResult<()> {
        let rval = self.read_reg(reg)?;
        self.write_reg(reg, (rval & !off) | on)
    }

    /// `_query_clock()` (lines 368-378).
    pub fn query_clock(&self) -> KResult<(i64, i64)> {
        let msg: [u8; 8] = [0x84, 0x42, 0, 0, 0, 0, 0, 0];
        let params = self.send_spi(&msg)?;
        let resp = response_bytes(&params)?;
        let mcu_clock: i64 =
            self.mcu.clock32_to_clock64(response_clock(&params)?);
        let chip_clock: i64 = (((resp_byte(&resp, 2)? as i64) & 0x7e) << 9)
            | (((resp_byte(&resp, 4)? as i64) & 0x3e) << 4);
        // Calculate temperature (lines 375-377).
        let temper: i64 = (resp_byte(&resp, 5)? as i64)
            - (((resp_byte(&resp, 4)? as i64) & 0x01) << 8);
        let temp: f64 = (temper as f64 + 152.0) / 2.776;
        self.last_temperature.replace(Some(temp));
        Ok((mcu_clock, chip_clock))
    }

    /// `update_clock()` (lines 379-388).
    pub fn update_clock_inner(&self) -> KResult<()> {
        let (mcu_clock, chip_clock) = self.query_clock()?;
        let (last_mcu, last_chip, last_freq) = {
            let s = self.state.borrow();
            (s.last_chip_mcu_clock, s.last_chip_clock, s.chip_freq)
        };
        let mdiff: i64 = mcu_clock - last_mcu;
        let chip_mclock: i64 = last_chip
            .wrapping_add(super::py_trunc_i64((mdiff as f64) * last_freq + 0.5));
        let mut cdiff: i64 = (chip_clock - chip_mclock) & 0xffff;
        cdiff -= (cdiff & 0x8000) << 1;
        let new_chip_clock: i64 = chip_mclock + cdiff;
        let new_freq = super::float_div(new_chip_clock as f64 - last_chip as f64, mdiff as f64)?;
        self.state.with_mut(|s| {
            s.last_chip_clock = new_chip_clock;
            s.last_chip_mcu_clock = mcu_clock;
            s.chip_freq = new_freq;
        });
        Ok(())
    }

    /// `cmd_ANGLE_DEBUG_READ` (lines 403-406).
    pub fn cmd_angle_debug_read(&self, gcmd: &GCmd) -> KResult<()> {
        let reg = gcmd_reg(gcmd, 0x30)?;
        let val = self.read_reg(reg)?;
        gcmd.respond_info(
            &fmt_const(
                "ANGLE REG[0x%02x] = 0x%04x",
                &[pct_int(reg), pct_int(val)],
            ),
            true,
        );
        Ok(())
    }

    /// `cmd_ANGLE_DEBUG_WRITE` (lines 408-412).
    pub fn cmd_angle_debug_write(&self, gcmd: &GCmd) -> KResult<()> {
        let reg = gcmd_reg(gcmd, 0x30)?;
        let val = gcmd_val(gcmd)?;
        self.write_reg(reg, val)
    }
}

impl SensorHelperApi for HelperTLE5012B {
    fn is_tcode_absolute(&self) -> bool {
        true
    }
    fn last_temperature(&self) -> Option<f64> {
        self.last_temperature.get()
    }
    fn set_last_temperature(&self, temp: Option<f64>) {
        self.last_temperature.replace(temp);
    }
    fn get_static_delay(&self) -> KResult<f64> {
        Err(no_static_delay())
    }
    fn get_tcode_params(&self) -> KResult<(i64, i64, f64)> {
        let s = self.state.borrow();
        Ok((s.last_chip_mcu_clock, s.last_chip_clock, s.chip_freq))
    }
    fn start(&self) -> KResult<()> {
        // Clear any errors from device (line 391).
        self.read_reg(0x00)?; // Read STAT
        // Initialize chip (lines 393-395).
        self.mask_reg(0x06, 0xc003, 0x4000)?; // MOD1: 42.7us, IIF disable
        self.mask_reg(0x08, 0x0007, 0x0001)?; // MOD2: Predict off, autocal=1
        self.mask_reg(0x0e, 0x0003, 0x0000)?; // MOD4: IIF mode
        // Setup starting clock values (lines 397-401).
        let (mcu_clock, chip_clock) = self.query_clock()?;
        let seconds: f64 = 1.0 / 750000.0;
        let ticks: i64 = self.mcu.seconds_to_clock(seconds);
        let freq = super::float_div(32.0, ticks as f64)?;
        self.state.with_mut(|s| {
            s.last_chip_clock = chip_clock;
            s.last_chip_mcu_clock = mcu_clock;
            s.chip_freq = freq;
        });
        self.update_clock_inner()
    }
    fn update_clock(&self) -> KResult<()> {
        self.update_clock_inner()
    }
}

// ---------------------------------------------------------------------------
// HelperMT6816 (lines 414-458)
// ---------------------------------------------------------------------------

/// `class HelperMT6816` (lines 414-458).
pub struct HelperMT6816 {
    printer: Printer,
    spi: Arc<McuSpi>,
    oid: i64,
    mcu: Mcu,
    spi_angle_transfer_cmd: GCell<Option<CommandQueryWrapper>>,
    last_temperature: GCell<Option<f64>>,
}

impl HelperMT6816 {
    /// `HelperMT6816(config, spi, oid)` (lines 417-430).
    pub fn new(
        config: &Config,
        spi: Arc<McuSpi>,
        oid: i64,
    ) -> KResult<Arc<HelperMT6816>> {
        let this = Arc::new(HelperMT6816 {
            printer: config.get_printer(),
            spi: spi.clone(),
            oid,
            mcu: spi.get_mcu(),
            spi_angle_transfer_cmd: GCell::new(None),
            last_temperature: GCell::new(None),
        });
        {
            let cb = Arc::clone(&this);
            this.mcu.register_config_callback(Box::new(move || cb.build_config()))?;
        }
        let name = name_last_token(&config.get_name());
        let gcode = lookup_interface::<dyn GCodeApi>(
            &*this.printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let cb = Arc::clone(&this);
        let handler: GCodeCommandHandler =
            Box::new(move |gcmd: &GCmd| cb.cmd_angle_debug_read(gcmd));
        gcode.register_mux_command(
            "ANGLE_DEBUG_READ",
            "CHIP",
            Some(&name),
            handler,
            Some(ANGLE_DEBUG_READ_HELP),
        )?;
        Ok(this)
    }

    /// `_build_config` (lines 431-436).
    pub fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        let cmd = self.mcu.lookup_query_command(
            "spi_angle_transfer oid=%c data=%*s",
            "spi_angle_transfer_response oid=%c clock=%u response=%*s",
            klippy_api::mcu::LookupQueryCommandOpts {
                oid: Some(self.oid),
                cq: Some(cmdqueue),
                is_async: false,
            },
        )?;
        self.spi_angle_transfer_cmd.replace(Some(cmd));
        Ok(())
    }

    /// `_send_spi(msg)` (lines 437-438).
    pub fn send_spi(&self, msg: &[u8]) -> KResult<PyValue> {
        self.spi.spi_transfer(msg, QuerySendOpts::default())
    }

    /// `_read_reg(reg)` (lines 441-446).
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let msg: [u8; 3] = [reg as u8, 0, 0];
        let params = self.send_spi(&msg)?;
        let resp = response_bytes(&params)?;
        Ok(((resp_byte(&resp, 1)? as i64) << 8) | (resp_byte(&resp, 2)? as i64))
    }

    /// `cmd_ANGLE_DEBUG_READ` (lines 450-458). The register is fixed
    /// to `0x83` (there is no `REG` parameter for this chip).
    pub fn cmd_angle_debug_read(&self, gcmd: &GCmd) -> KResult<()> {
        let reg: i64 = 0x83;
        let val = self.read_reg(reg)?;
        gcmd.respond_info(
            &fmt_const(
                "ANGLE REG[0x%02x] = 0x%04x",
                &[pct_int(reg), pct_int(val)],
            ),
            true,
        );
        let angle: i64 = val >> 2;
        let parity: i64 = (((val >> 1) as u64).count_ones() as i64) % 2;
        let deg: f64 = (angle as f64) * 360.0 / ((1i64 << 14) as f64);
        gcmd.respond_info(
            &fmt_const(
                "Angle %i ~ %.2f",
                &[
                    pct_int(angle),
                    pycompat::text::PctValue::Float(deg),
                ],
            ),
            true,
        );
        gcmd.respond_info(
            &fmt_const("No Mag: %i", &[pct_int(val >> 1 & 0x1)]),
            true,
        );
        gcmd.respond_info(
            &fmt_const(
                "Parity: %i == %i",
                &[pct_int(parity), pct_int(val & 0x1)],
            ),
            true,
        );
        Ok(())
    }
}

impl SensorHelperApi for HelperMT6816 {
    fn is_tcode_absolute(&self) -> bool {
        false
    }
    fn last_temperature(&self) -> Option<f64> {
        self.last_temperature.get()
    }
    fn set_last_temperature(&self, temp: Option<f64>) {
        self.last_temperature.replace(temp);
    }
    fn get_static_delay(&self) -> KResult<f64> {
        Ok(0.000001)
    }
    fn get_tcode_params(&self) -> KResult<(i64, i64, f64)> {
        Err(no_tcode_params())
    }
    fn start(&self) -> KResult<()> {
        Ok(())
    }
    fn update_clock(&self) -> KResult<()> {
        Err(no_update_clock())
    }
}

// ---------------------------------------------------------------------------
// HelperMT6826S (lines 460-603)
// ---------------------------------------------------------------------------

/// `class HelperMT6826S` (lines 460-603).
pub struct HelperMT6826S {
    printer: Printer,
    stepper_name: Option<String>,
    spi: Arc<McuSpi>,
    oid: i64,
    mcu: Mcu,
    spi_angle_transfer_cmd: GCell<Option<CommandQueryWrapper>>,
    last_temperature: GCell<Option<f64>>,
}

impl HelperMT6826S {
    /// `HelperMT6826S(config, spi, oid)` (lines 463-486).
    pub fn new(
        config: &Config,
        spi: Arc<McuSpi>,
        oid: i64,
    ) -> KResult<Arc<HelperMT6826S>> {
        let stepper_name: Option<String> = config.get(
            "stepper",
            GetStrOpts { default: Some(None), note_valid: true },
        )?;
        let this = Arc::new(HelperMT6826S {
            printer: config.get_printer(),
            stepper_name,
            spi: spi.clone(),
            oid,
            mcu: spi.get_mcu(),
            spi_angle_transfer_cmd: GCell::new(None),
            last_temperature: GCell::new(None),
        });
        {
            let cb = Arc::clone(&this);
            this.mcu.register_config_callback(Box::new(move || cb.build_config()))?;
        }
        let name = name_last_token(&config.get_name());
        let gcode = lookup_interface::<dyn GCodeApi>(
            &*this.printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        {
            let cb = Arc::clone(&this);
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd: &GCmd| cb.cmd_angle_debug_read(gcmd));
            gcode.register_mux_command(
                "ANGLE_DEBUG_READ",
                "CHIP",
                Some(&name),
                handler,
                Some(ANGLE_DEBUG_READ_HELP),
            )?;
        }
        {
            let cb = Arc::clone(&this);
            let handler: GCodeCommandHandler =
                Box::new(move |gcmd: &GCmd| cb.cmd_angle_chip_calibrate(gcmd));
            gcode.register_mux_command(
                "ANGLE_CHIP_CALIBRATE",
                "CHIP",
                Some(&name),
                handler,
                Some(ANGLE_CHIP_CALIBRATE_HELP),
            )?;
        }
        Ok(this)
    }

    /// `_build_config` (lines 487-492).
    pub fn build_config(&self) -> KResult<()> {
        let cmdqueue = self.spi.get_command_queue();
        let cmd = self.mcu.lookup_query_command(
            "spi_angle_transfer oid=%c data=%*s",
            "spi_angle_transfer_response oid=%c clock=%u response=%*s",
            klippy_api::mcu::LookupQueryCommandOpts {
                oid: Some(self.oid),
                cq: Some(cmdqueue),
                is_async: false,
            },
        )?;
        self.spi_angle_transfer_cmd.replace(Some(cmd));
        Ok(())
    }

    /// `_send_spi(msg)` (lines 493-495).
    pub fn send_spi(&self, msg: &[u8]) -> KResult<PyValue> {
        self.spi.spi_transfer(msg, QuerySendOpts::default())
    }

    /// `_read_reg(reg)` (lines 498-503).
    pub fn read_reg(&self, reg: i64) -> KResult<i64> {
        let reg = 0x3000 | reg;
        let msg: [u8; 3] = [(reg >> 8) as u8, (reg & 0xff) as u8, 0];
        let params = self.send_spi(&msg)?;
        let resp = response_bytes(&params)?;
        Ok(resp_byte(&resp, 2)? as i64)
    }

    /// `_write_reg(reg, data)` (lines 504-507).
    pub fn write_reg(&self, reg: i64, data: i64) -> KResult<()> {
        let reg = 0x6000 | reg;
        let msg: [u8; 3] = [(reg >> 8) as u8, (reg & 0xff) as u8, data as u8];
        self.send_spi(&msg)?;
        Ok(())
    }

    /// `crc8(data)` (lines 508-519).
    pub fn crc8(&self, data: &[u8]) -> i64 {
        let polynomial: i64 = 0x07;
        let mut crc: i64 = 0x00;
        for byte in data {
            crc ^= *byte as i64;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = ((crc << 1) ^ polynomial) & 0xff;
                } else {
                    crc = (crc << 1) & 0xff;
                }
            }
        }
        crc
    }

    /// `_read_angle(reg)` (lines 520-529).
    pub fn read_angle(&self, reg: i64) -> KResult<(i64, i64, i64, i64)> {
        let reg = 0x3000 | reg;
        let msg: [u8; 6] = [(reg >> 8) as u8, (reg & 0xff) as u8, 0, 0, 0, 0];
        let params = self.send_spi(&msg)?;
        let resp = response_bytes(&params)?;
        let angle: i64 = ((resp_byte(&resp, 2)? as i64) << 7)
            | ((resp_byte(&resp, 3)? as i64) >> 1);
        let status: i64 = resp_byte(&resp, 4)? as i64;
        let crc_computed =
            self.crc8(&[resp_byte(&resp, 2)?, resp_byte(&resp, 3)?, resp_byte(&resp, 4)?]);
        let crc: i64 = resp_byte(&resp, 5)? as i64;
        Ok((angle, status, crc, crc_computed))
    }

    /// `get_microsteps` (lines 534-540).
    pub fn get_microsteps(&self) -> KResult<(i64, i64)> {
        let stepper_name: String = self.stepper_name.clone().unwrap_or_default();
        get_microsteps_from_configfile(&self.printer, &stepper_name)
    }

    /// `cmd_ANGLE_CHIP_CALIBRATE` (lines 542-576).
    pub fn cmd_angle_chip_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let fmove = force_move_from_printer(&self.printer)?;
        // The Python looks the stepper up before the `None` check
        // (line 544-546), so `stepper=None` dies in the lookup.
        let mcu_stepper = {
            let name: String = self.stepper_name.clone().unwrap_or_default();
            fmove.lookup_stepper(Some(&name))?
        };
        if self.stepper_name.is_none() {
            gcmd.respond_info("stepper not defined", true);
            return Ok(());
        }
        gcmd.respond_info("MT6826S Run calibration sequence", true);
        gcmd.respond_info(
            "Motor will do 18+ rotations - ensure pulley is disconnected",
            true,
        );
        let req_freq: i64 = (self.read_reg(0x00e)? >> 4) & 0x7;
        // Minimal calibration speed (lines 552-555).
        let rpm: i64 = (3200 >> req_freq) + 1;
        let rps: f64 = super::float_div(rpm as f64, 60.0)?;
        let (microsteps, full_steps) = self.get_microsteps()?;
        let step_dist: f64 = mcu_stepper.get_step_dist();
        let full_step_dist: f64 = step_dist * (microsteps as f64);
        let rotation_dist: f64 = (full_steps as f64) * full_step_dist;
        let speed: f64 = rps * rotation_dist;
        fmove.manual_move(&mcu_stepper, 2.0 * rotation_dist, speed, 0.0)?;
        self.write_reg(0x155, 0x5e)?;
        fmove.manual_move(&mcu_stepper, 20.0 * rotation_dist, speed, 0.0)?;
        let mut val = self.read_reg(0x113)?;
        let mut code: i64 = val >> 6;
        gcmd.respond_info(&status_line(code), true);
        while code == 1 {
            fmove.manual_move(&mcu_stepper, 5.0 * rotation_dist, speed, 0.0)?;
            val = self.read_reg(0x113)?;
            code = val >> 6;
            gcmd.respond_info(&status_line(code), true);
        }
        if code == 2 {
            gcmd.respond_info("Calibration failed", true);
        }
        if code == 3 {
            gcmd.respond_info("Calibration success, please poweroff sensor", true);
        }
        Ok(())
    }

    /// `cmd_ANGLE_DEBUG_READ` (lines 578-601).
    pub fn cmd_angle_debug_read(&self, gcmd: &GCmd) -> KResult<()> {
        let reg = gcmd_reg(gcmd, 0x155)?;
        if reg == 0x003 {
            let (angle, status, crc1, crc_computed) = self.read_angle(reg)?;
            gcmd.respond_info(
                &fmt_const("ANGLE REG[0x003] = 0x%02x", &[pct_int(angle >> 7)]),
                true,
            );
            gcmd.respond_info(
                &fmt_const("ANGLE REG[0x004] = 0x%02x", &[pct_int((angle << 1) & 0xff)]),
                true,
            );
            let deg: f64 = (angle as f64) * 360.0 / ((1i64 << 15) as f64);
            gcmd.respond_info(
                &fmt_const(
                    "Angle %i ~ %.2f",
                    &[pct_int(angle), pycompat::text::PctValue::Float(deg)],
                ),
                true,
            );
            gcmd.respond_info(
                &fmt_const("Weak Mag: %i", &[pct_int(status >> 1 & 0x1)]),
                true,
            );
            gcmd.respond_info(
                &fmt_const("Under Voltage: %i", &[pct_int(status >> 2 & 0x1)]),
                true,
            );
            gcmd.respond_info(
                &fmt_const(
                    "CRC: 0x%02x == 0x%02x",
                    &[pct_int(crc1), pct_int(crc_computed)],
                ),
                true,
            );
        } else if reg == 0x00e {
            let val = self.read_reg(reg)?;
            gcmd.respond_info(&fmt_const("GPIO_DS = %i", &[pct_int(val >> 7)]), true);
            gcmd.respond_info(
                &fmt_const("AUTOCAL_FREQ = %i", &[pct_int(val >> 4 & 0x7)]),
                true,
            );
        } else if reg == 0x113 {
            let val = self.read_reg(reg)?;
            gcmd.respond_info(&status_line(val >> 6), true);
        } else {
            let val = self.read_reg(reg)?;
            gcmd.respond_info(
                &fmt_const(
                    "REG[0x%04x] = 0x%02x",
                    &[pct_int(reg), pct_int(val)],
                ),
                true,
            );
        }
        Ok(())
    }
}

impl SensorHelperApi for HelperMT6826S {
    fn is_tcode_absolute(&self) -> bool {
        false
    }
    fn last_temperature(&self) -> Option<f64> {
        self.last_temperature.get()
    }
    fn set_last_temperature(&self, temp: Option<f64>) {
        self.last_temperature.replace(temp);
    }
    fn get_static_delay(&self) -> KResult<f64> {
        Ok(0.00001)
    }
    fn get_tcode_params(&self) -> KResult<(i64, i64, f64)> {
        Err(no_tcode_params())
    }
    fn start(&self) -> KResult<()> {
        let val = self.read_reg(0x00d)?;
        // Set histeresis to 0.003 degree (line 533).
        self.write_reg(0x00d, (val & 0xf8) | 0x5)
    }
    fn update_clock(&self) -> KResult<()> {
        Err(no_update_clock())
    }
}

const STATUS_MAP: [&str; 4] = [
    "No Calibration",
    "Running Calibration",
    "Calibration Failed",
    "Calibration Successful",
];

fn status_line(code: i64) -> String {
    // `status_map[code]`; code is `val >> 6` of a byte, always in
    // range (the Python KeyError path is unreachable).
    fmt_const("Status: %s", &[pct_str(STATUS_MAP[code as usize])])
}

fn force_move_from_printer(printer: &Printer) -> KResult<Arc<dyn ForceMoveApi>> {
    // `self.printer.lookup_object('force_move')` (line 543).
    Ok(
        lookup_interface::<dyn ForceMoveApi>(&**printer, "force_move", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None"),
    )
}

