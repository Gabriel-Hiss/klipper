//! Port of the `bme280` module (`klippy/extras/bme280.py`).
//!
//! I2C temperature/pressure/humidity sensor chip (`BMP280`, `BME280`,
//! `BME680`, `BMP180`, `BMP388`). The module registers the `BME280` sensor
//! factory with `heaters`; the object the factory builds is a sensor chip
//! ([`TemperatureSensorApi`]) that is also the printer object `bme280 <name>`
//! whose status carries `temperature`, `pressure` and, per chip, `humidity`
//! and `gas`.
//!
//! The chip is identified and configured on `klippy:connect`; the sampling
//! timer is then started and reports through the callback the heater
//! installed with `setup_callback`.

use crate::bus::{self, McuI2cApi};
use crate::heaters::{PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{Mcu, QuerySendOpts};
use klippy_api::object::{load_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::{log, EventArg, GCell};
use pycompat::text::{percent_format, round_py_nd, str_repr, PctValue};
use pycompat::value::type_name;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::{Arc, Weak};

/// `REPORT_TIME` (`klippy/extras/bme280.py:9`).
pub const REPORT_TIME: f64 = 0.8;
/// `BME280_CHIP_ADDR` (`klippy/extras/bme280.py:10`).
pub const BME280_CHIP_ADDR: i64 = 0x76;
/// Default `i2c_speed` passed to `bus.MCU_I2C_from_config` (`bme280.py:134`).
pub const BME280_I2C_SPEED: i64 = 100000;

const STATUS_IM_UPDATE: i64 = 1;
const MODE: i64 = 1;
const MODE_PERIODIC: i64 = 3;
const RUN_GAS: i64 = 1 << 4;
const NB_CONV_0: i64 = 0;
const EAS_NEW_DATA: i64 = 1 << 7;
const MEASURE_IN_PROGRESS: i64 = 1 << 5;
const RESET_CHIP_VALUE: i64 = 0xB6;

const BMP388_REG_VAL_PRESS_EN: i64 = 0x01;
const BMP388_REG_VAL_TEMP_EN: i64 = 0x02;
const BMP388_REG_VAL_PRESS_OS_NO: i64 = 0b000;
const BMP388_REG_VAL_TEMP_OS_NO: i64 = 0b000000;
const BMP388_REG_VAL_ODR_50_HZ: i64 = 0x02;
const BMP388_REG_VAL_DRDY_EN: i64 = 0b100000;
const BMP388_REG_VAL_NORMAL_MODE: i64 = 0x30;

const BME_CHIP_ID_REG: u8 = 0xD0;
const BMP3_CHIP_ID_REG: u8 = 0x00;

/// Largest shift whose Python `int` result this port builds (Python raises
/// `MemoryError` long before a shift this wide succeeds).
const SHIFT_LIMIT: i64 = 1 << 26;

/// `BME680_GAS_CONSTANTS` (`klippy/extras/bme280.py:54-71`).
const BME680_GAS_CONSTANTS: [(f64, f64); 16] = [
    (1., 8000000.),
    (1., 4000000.),
    (1., 2000000.),
    (1., 1000000.),
    (1., 499500.4995),
    (0.99, 248262.1648),
    (1., 125000.),
    (0.992, 63004.03226),
    (1., 31281.28128),
    (1., 15625.),
    (0.998, 7812.5),
    (0.995, 3906.25),
    (1., 1953.125),
    (0.99, 976.5625),
    (1., 488.28125),
    (1., 244.140625),
];

/// The register map `self.chip_registers` points at.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Regs {
    Bme280,
    Bmp388,
    Bme680,
    Bmp180,
}

impl Regs {
    /// `self.chip_registers[reg_name]`; a missing name is a `KeyError`.
    fn lookup(self, name: &str) -> KResult<i64> {
        let reg = match self {
            // `BME280_REGS` (`klippy/extras/bme280.py:12-18`).
            Regs::Bme280 => match name {
                "RESET" => 0xE0,
                "CTRL_HUM" => 0xF2,
                "STATUS" => 0xF3,
                "CTRL_MEAS" => 0xF4,
                "CONFIG" => 0xF5,
                "PRESSURE_MSB" => 0xF7,
                "PRESSURE_LSB" => 0xF8,
                "PRESSURE_XLSB" => 0xF9,
                "TEMP_MSB" => 0xFA,
                "TEMP_LSB" => 0xFB,
                "TEMP_XLSB" => 0xFC,
                "HUM_MSB" => 0xFD,
                "HUM_LSB" => 0xFE,
                "CAL_1" => 0x88,
                "CAL_2" => 0xE1,
                _ => return Err(key_error(name)),
            },
            // `BMP388_REGS` (`klippy/extras/bme280.py:20-34`).
            Regs::Bmp388 => match name {
                "CMD" => 0x7E,
                "STATUS" => 0x03,
                "PWR_CTRL" => 0x1B,
                "OSR" => 0x1C,
                "ORD" => 0x1D,
                "INT_CTRL" => 0x19,
                "CAL_1" => 0x31,
                "TEMP_MSB" => 0x09,
                "TEMP_LSB" => 0x08,
                "TEMP_XLSB" => 0x07,
                "PRESS_MSB" => 0x06,
                "PRESS_LSB" => 0x05,
                "PRESS_XLSB" => 0x04,
                _ => return Err(key_error(name)),
            },
            // `BME680_REGS` (`klippy/extras/bme280.py:43-52`).
            Regs::Bme680 => match name {
                "RESET" => 0xE0,
                "CTRL_HUM" => 0x72,
                "CTRL_GAS_1" => 0x71,
                "CTRL_GAS_0" => 0x70,
                "GAS_WAIT_0" => 0x64,
                "RES_HEAT_0" => 0x5A,
                "IDAC_HEAT_0" => 0x50,
                "STATUS" => 0x73,
                "EAS_STATUS_0" => 0x1D,
                "CTRL_MEAS" => 0x74,
                "CONFIG" => 0x75,
                "GAS_R_LSB" => 0x2B,
                "GAS_R_MSB" => 0x2A,
                "PRESSURE_MSB" => 0x1F,
                "PRESSURE_LSB" => 0x20,
                "PRESSURE_XLSB" => 0x21,
                "TEMP_MSB" => 0x22,
                "TEMP_LSB" => 0x23,
                "TEMP_XLSB" => 0x24,
                "HUM_MSB" => 0x25,
                "HUM_LSB" => 0x26,
                "CAL_1" => 0x88,
                "CAL_2" => 0xE1,
                "RES_HEAT_VAL" => 0x00,
                "RES_HEAT_RANGE" => 0x02,
                "RANGE_SWITCHING_ERROR" => 0x04,
                _ => return Err(key_error(name)),
            },
            // `BMP180_REGS` (`klippy/extras/bme280.py:73-81`).
            Regs::Bmp180 => match name {
                "RESET" => 0xE0,
                "CAL_1" => 0xAA,
                "CTRL_MEAS" => 0xF4,
                "REG_MSB" => 0xF6,
                "REG_LSB" => 0xF7,
                "CRV_TEMP" => 0x2E,
                "CRV_PRES" => 0x34,
                _ => return Err(key_error(name)),
            },
        };
        Ok(reg)
    }
}

/// `BME_CHIPS` (`klippy/extras/bme280.py:94-97`).
fn bme_chip_name(chip_id: i64) -> Option<&'static str> {
    match chip_id {
        0x58 => Some("BMP280"),
        0x60 => Some("BME280"),
        0x61 => Some("BME680"),
        0x55 => Some("BMP180"),
        0x50 => Some("BMP388"),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Python exception and number helpers
// ---------------------------------------------------------------------------

/// A Python builtin exception klippy does not catch by class.
fn exc(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn key_error(key: &str) -> KlippyError {
    exc("KeyError", str_repr(key))
}

fn index_error() -> KlippyError {
    exc("IndexError", "bytearray index out of range")
}

fn value_error(msg: &str) -> KlippyError {
    exc("ValueError", msg)
}

fn type_error(msg: impl Into<String>) -> KlippyError {
    exc("TypeError", msg)
}

/// `f"{class}: {message}"`, the last line of a Python traceback.
fn exc_text(e: &KlippyError) -> String {
    format!("{}: {}", e.class_name(), e.message())
}

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant bme280 format renders")
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_float(v: f64) -> PctValue {
    PctValue::Float(v)
}

/// `bytes[i]` on a `bytearray`.
fn byte(data: &[u8], i: usize) -> KResult<i64> {
    data.get(i).map(|b| *b as i64).ok_or_else(index_error)
}

/// `bytes[a:b]`: Python slices clamp to the length.
fn slice(data: &[u8], a: usize, b: usize) -> &[u8] {
    let n = data.len();
    &data[a.min(n)..b.min(n)]
}

/// `a / b` on floats: a zero divisor raises `ZeroDivisionError`.
fn div(a: f64, b: f64) -> KResult<f64> {
    if b == 0.0 {
        return Err(exc("ZeroDivisionError", "float division by zero"));
    }
    Ok(a / b)
}

/// Python `max(a, b)`: the first argument wins unless the second is greater.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python `min(a, b)`: the first argument wins unless the second is smaller.
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python `int(x)` on a float (truncation). Values beyond the `i64` range
/// saturate; every caller feeds the result to a byte write or a shift whose
/// Python result is out of range for the byte write as well.
fn py_int_of(x: f64) -> KResult<i64> {
    if x.is_nan() {
        return Err(value_error("cannot convert float NaN to integer"));
    }
    if x.is_infinite() {
        return Err(exc("OverflowError", "cannot convert float infinity to integer"));
    }
    Ok(x as i64)
}

/// Python `a >> b` with `a >= 0`-style `i64` operands.
fn py_shr(a: i64, b: i64) -> KResult<i64> {
    if b < 0 {
        return Err(value_error("negative shift count"));
    }
    Ok(if b >= 64 {
        if a >= 0 {
            0
        } else {
            -1
        }
    } else {
        a >> b
    })
}

/// Python `a << b`. A result outside `i64` saturates: such a value can only
/// reach an I2C byte write, where Python raises the same `ValueError`.
fn py_shl(a: i64, b: i64) -> KResult<i64> {
    if b < 0 {
        return Err(value_error("negative shift count"));
    }
    if a == 0 {
        return Ok(0);
    }
    if b >= 64 {
        return Ok(if a > 0 { i64::MAX } else { i64::MIN });
    }
    let r = (a as i128) << b;
    Ok(r.clamp(i64::MIN as i128, i64::MAX as i128) as i64)
}

/// `((v << n) + 2) / 4` on a Python int `v` (`_compensate_pressure_bmp180`).
fn shl_plus_two_over_four(v: i64, n: i64) -> KResult<f64> {
    if n < 0 {
        return Err(value_error("negative shift count"));
    }
    if n > SHIFT_LIMIT {
        return Err(exc("MemoryError", ""));
    }
    let shifted = PyInt::from_i64(v).0 << (n as usize);
    let total = PyInt::new(shifted + 2u32);
    let f = total
        .to_f64_exact()
        .ok_or_else(|| exc("OverflowError", "integer division result too large for a float"))?;
    Ok(f / 4.)
}

/// `pow(2, k)` formatted with `%d`: an exact int for `k >= 0`, a float below
/// (`%d` truncates `2 ** k < 1` to 0).
fn pow2_pct(k: i64) -> KResult<PctValue> {
    if k < 0 {
        return Ok(pct_int(0));
    }
    if k > SHIFT_LIMIT {
        return Err(exc("MemoryError", ""));
    }
    Ok(PctValue::Int(PyInt::new(PyInt::from_i64(1).0 << (k as usize))))
}

/// `pow(2, k) - 1` formatted with `%d`; for `k < 0` Python subtracts from a
/// float, and `%d` truncates `2 ** k - 1` (in (-1, -0.5]) to 0.
fn pow2_minus_one_pct(k: i64) -> KResult<PctValue> {
    if k < 0 {
        return Ok(pct_int(0));
    }
    if k > SHIFT_LIMIT {
        return Err(exc("MemoryError", ""));
    }
    Ok(PctValue::Int(PyInt::new((PyInt::from_i64(1).0 << (k as usize)) - 1u32)))
}

// ---------------------------------------------------------------------------
// Module-level helpers (`klippy/extras/bme280.py:102-127`)
// ---------------------------------------------------------------------------

/// `get_twos_complement(val, bit_size)` (`klippy/extras/bme280.py:102`).
pub fn get_twos_complement(val: i64, bit_size: i64) -> i64 {
    if val & (1i64 << (bit_size - 1)) != 0 {
        val - (1i64 << bit_size)
    } else {
        val
    }
}

/// `get_unsigned_short(bits)` (`klippy/extras/bme280.py:108`) on a bytearray.
pub fn get_unsigned_short(bits: &[u8]) -> KResult<i64> {
    let hi = byte(bits, 1)?;
    let lo = byte(bits, 0)?;
    Ok(hi << 8 | lo)
}

/// `get_signed_short(bits)` (`klippy/extras/bme280.py:112`).
pub fn get_signed_short(bits: &[u8]) -> KResult<i64> {
    Ok(get_twos_complement(get_unsigned_short(bits)?, 16))
}

/// `get_signed_byte(bits)` (`klippy/extras/bme280.py:117`).
pub fn get_signed_byte(bits: i64) -> i64 {
    get_twos_complement(bits, 8)
}

/// `get_unsigned_short_msb(bits)` (`klippy/extras/bme280.py:121`).
pub fn get_unsigned_short_msb(bits: &[u8]) -> KResult<i64> {
    let hi = byte(bits, 0)?;
    let lo = byte(bits, 1)?;
    Ok(hi << 8 | lo)
}

/// `get_signed_short_msb(bits)` (`klippy/extras/bme280.py:125`).
pub fn get_signed_short_msb(bits: &[u8]) -> KResult<i64> {
    Ok(get_twos_complement(get_unsigned_short_msb(bits)?, 16))
}

// ---------------------------------------------------------------------------
// Calibration parsers (`klippy/extras/bme280.py:177-267`)
// ---------------------------------------------------------------------------

/// `self.dig`: calibration values as floats (every Python entry is an int or
/// a float that is exact in a double).
type Dig = IndexMap<String, f64>;

fn ins(dig: &mut Dig, key: &str, value: f64) {
    dig.insert(key.to_string(), value);
}

fn dig_val(dig: &Dig, key: &str) -> KResult<f64> {
    dig.get(key).copied().ok_or_else(|| key_error(key))
}

fn read_calibration_data_bmp280(c: &[u8]) -> KResult<Dig> {
    let mut dig = Dig::new();
    ins(&mut dig, "T1", get_unsigned_short(slice(c, 0, 2))? as f64);
    ins(&mut dig, "T2", get_signed_short(slice(c, 2, 4))? as f64);
    ins(&mut dig, "T3", get_signed_short(slice(c, 4, 6))? as f64);

    ins(&mut dig, "P1", get_unsigned_short(slice(c, 6, 8))? as f64);
    ins(&mut dig, "P2", get_signed_short(slice(c, 8, 10))? as f64);
    ins(&mut dig, "P3", get_signed_short(slice(c, 10, 12))? as f64);
    ins(&mut dig, "P4", get_signed_short(slice(c, 12, 14))? as f64);
    ins(&mut dig, "P5", get_signed_short(slice(c, 14, 16))? as f64);
    ins(&mut dig, "P6", get_signed_short(slice(c, 16, 18))? as f64);
    ins(&mut dig, "P7", get_signed_short(slice(c, 18, 20))? as f64);
    ins(&mut dig, "P8", get_signed_short(slice(c, 20, 22))? as f64);
    ins(&mut dig, "P9", get_signed_short(slice(c, 22, 24))? as f64);
    Ok(dig)
}

fn read_calibration_data_bmp388(c: &[u8]) -> KResult<Dig> {
    let mut dig = Dig::new();
    ins(&mut dig, "T1", get_unsigned_short(slice(c, 0, 2))? as f64 / 0.00390625);
    ins(&mut dig, "T2", get_unsigned_short(slice(c, 2, 4))? as f64 / 1073741824.0);
    ins(&mut dig, "T3", get_signed_byte(byte(c, 4)?) as f64 / 281474976710656.0);

    let p1 = get_signed_short(slice(c, 5, 7))? - 16384;
    ins(&mut dig, "P1", p1 as f64 / 1048576.0);
    let p2 = get_signed_short(slice(c, 7, 9))? - 16384;
    ins(&mut dig, "P2", p2 as f64 / 536870912.0);
    ins(&mut dig, "P3", get_signed_byte(byte(c, 9)?) as f64 / 4294967296.0);
    ins(&mut dig, "P4", get_signed_byte(byte(c, 10)?) as f64 / 137438953472.0);
    ins(&mut dig, "P5", get_unsigned_short(slice(c, 11, 13))? as f64 / 0.125);
    ins(&mut dig, "P6", get_unsigned_short(slice(c, 13, 15))? as f64 / 64.0);
    ins(&mut dig, "P7", get_signed_byte(byte(c, 15)?) as f64 / 256.0);
    ins(&mut dig, "P8", get_signed_byte(byte(c, 16)?) as f64 / 32768.0);
    ins(&mut dig, "P9", get_signed_short(slice(c, 17, 19))? as f64 / 281474976710656.0);
    ins(&mut dig, "P10", get_signed_byte(byte(c, 19)?) as f64 / 281474976710656.0);
    ins(&mut dig, "P11", get_signed_byte(byte(c, 20)?) as f64 / 36893488147419103232.0);
    Ok(dig)
}

fn read_calibration_data_bme280(c1: &[u8], c2: &[u8]) -> KResult<Dig> {
    let mut dig = read_calibration_data_bmp280(c1)?;
    ins(&mut dig, "H1", (byte(c1, 25)? & 0xFF) as f64);
    ins(&mut dig, "H2", get_signed_short(slice(c2, 0, 2))? as f64);
    ins(&mut dig, "H3", (byte(c2, 2)? & 0xFF) as f64);
    let h4 = (byte(c2, 3)? << 4) | (byte(c2, 4)? & 0x0F);
    ins(&mut dig, "H4", get_twos_complement(h4, 12) as f64);
    let h5 = (byte(c2, 5)? << 4) | ((byte(c2, 4)? & 0xF0) >> 4);
    ins(&mut dig, "H5", get_twos_complement(h5, 12) as f64);
    ins(&mut dig, "H6", get_twos_complement(byte(c2, 6)?, 8) as f64);
    Ok(dig)
}

fn read_calibration_data_bme680(c1: &[u8], c2: &[u8]) -> KResult<Dig> {
    let mut dig = Dig::new();
    ins(&mut dig, "T1", get_unsigned_short(slice(c2, 8, 10))? as f64);
    ins(&mut dig, "T2", get_signed_short(slice(c1, 2, 4))? as f64);
    ins(&mut dig, "T3", get_signed_byte(byte(c1, 4)?) as f64);

    ins(&mut dig, "P1", get_unsigned_short(slice(c1, 6, 8))? as f64);
    ins(&mut dig, "P2", get_signed_short(slice(c1, 8, 10))? as f64);
    ins(&mut dig, "P3", byte(c1, 10)? as f64);
    ins(&mut dig, "P4", get_signed_short(slice(c1, 12, 14))? as f64);
    ins(&mut dig, "P5", get_signed_short(slice(c1, 14, 16))? as f64);
    ins(&mut dig, "P6", get_signed_byte(byte(c1, 17)?) as f64);
    ins(&mut dig, "P7", get_signed_byte(byte(c1, 16)?) as f64);
    ins(&mut dig, "P8", get_signed_short(slice(c1, 20, 22))? as f64);
    ins(&mut dig, "P9", get_signed_short(slice(c1, 22, 24))? as f64);
    ins(&mut dig, "P10", byte(c1, 24)? as f64);

    let h1 = (byte(c2, 2)? << 4) | (byte(c2, 1)? & 0x0F);
    ins(&mut dig, "H1", get_twos_complement(h1, 12) as f64);
    let h2 = (byte(c2, 0)? << 4) | ((byte(c2, 1)? & 0xF0) >> 4);
    ins(&mut dig, "H2", get_twos_complement(h2, 12) as f64);
    ins(&mut dig, "H3", get_signed_byte(byte(c2, 3)?) as f64);
    ins(&mut dig, "H4", get_signed_byte(byte(c2, 4)?) as f64);
    ins(&mut dig, "H5", get_signed_byte(byte(c2, 5)?) as f64);
    ins(&mut dig, "H6", byte(c2, 6)? as f64);
    ins(&mut dig, "H7", get_signed_byte(byte(c2, 7)?) as f64);

    ins(&mut dig, "G1", get_signed_byte(byte(c2, 12)?) as f64);
    ins(&mut dig, "G2", get_signed_short(slice(c2, 10, 12))? as f64);
    ins(&mut dig, "G3", get_signed_byte(byte(c2, 13)?) as f64);
    Ok(dig)
}

fn read_calibration_data_bmp180(c: &[u8]) -> KResult<Dig> {
    let mut dig = Dig::new();
    ins(&mut dig, "AC1", get_signed_short_msb(slice(c, 0, 2))? as f64);
    ins(&mut dig, "AC2", get_signed_short_msb(slice(c, 2, 4))? as f64);
    ins(&mut dig, "AC3", get_signed_short_msb(slice(c, 4, 6))? as f64);
    ins(&mut dig, "AC4", get_unsigned_short_msb(slice(c, 6, 8))? as f64);
    ins(&mut dig, "AC5", get_unsigned_short_msb(slice(c, 8, 10))? as f64);
    ins(&mut dig, "AC6", get_unsigned_short_msb(slice(c, 10, 12))? as f64);

    ins(&mut dig, "B1", get_signed_short_msb(slice(c, 12, 14))? as f64);
    ins(&mut dig, "B2", get_signed_short_msb(slice(c, 14, 16))? as f64);

    ins(&mut dig, "MB", get_signed_short_msb(slice(c, 16, 18))? as f64);
    ins(&mut dig, "MC", get_signed_short_msb(slice(c, 18, 20))? as f64);
    ins(&mut dig, "MD", get_signed_short_msb(slice(c, 20, 22))? as f64);
    Ok(dig)
}

// ---------------------------------------------------------------------------
// `BME280` (`klippy/extras/bme280.py:129`)
// ---------------------------------------------------------------------------

/// The `_sample_*` method `_init_bmxx80` registers with the sample timer.
#[derive(Clone, Copy)]
enum SampleKind {
    Bme280,
    Bmp388,
    Bme680,
    Bmp180,
}

/// Port of the `BME280` class.
pub struct Bme280 {
    printer: Printer,
    reactor: Reactor,
    i2c: Arc<dyn McuI2cApi>,
    mcu: Mcu,
    name: String,
    iir_filter: i64,
    os_temp: i64,
    os_hum: i64,
    os_pres: i64,
    gas_heat_temp: i64,
    gas_heat_duration: i64,
    temp: GCell<f64>,
    pressure: GCell<f64>,
    humidity: GCell<f64>,
    gas: GCell<f64>,
    t_fine: GCell<f64>,
    min_temp: GCell<f64>,
    max_temp: GCell<f64>,
    max_sample_time: GCell<Option<f64>>,
    sample_timer: GCell<Option<Timer>>,
    dig: GCell<Option<Arc<Dig>>>,
    chip_type: GCell<&'static str>,
    chip_registers: GCell<Regs>,
    /// `self._callback`; unset until `setup_callback` (Python has no attribute).
    callback: GCell<Option<TemperatureCallback>>,
    /// `self.last_gas_time`; Python sets it only when it registers the connect handler.
    last_gas_time: GCell<Option<f64>>,
}

fn get_int(config: &Config, option: &str, default: i64) -> KResult<i64> {
    Ok(config
        .getint(
            option,
            GetIntOpts { default: Some(Some(default)), minval: None, maxval: None, note_valid: true },
        )?
        .expect("getint with a default never returns None"))
}

impl Bme280 {
    /// `BME280.__init__(config)` (`klippy/extras/bme280.py:130-160`).
    pub fn new(config: &Config) -> KResult<Arc<Bme280>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .ok_or_else(|| exc("IndexError", "list index out of range"))?
            .to_string();
        let reactor = printer.get_reactor();
        let i2c: Arc<dyn McuI2cApi> =
            bus::mcu_i2c_from_config(config, Some(BME280_CHIP_ADDR), BME280_I2C_SPEED, false)?;
        let mcu = i2c.get_mcu();
        let iir_filter = get_int(config, "bme280_iir_filter", 1)?;
        let os_temp = get_int(config, "bme280_oversample_temp", 2)?;
        let os_hum = get_int(config, "bme280_oversample_hum", 2)?;
        let os_pres = get_int(config, "bme280_oversample_pressure", 2)?;
        let gas_heat_temp = get_int(config, "bme280_gas_target_temp", 320)?;
        let gas_heat_duration = get_int(config, "bme280_gas_heat_duration", 150)?;
        log::info(&pct(
            "BMxx80: Oversampling: Temp %dx Humid %dx Pressure %dx",
            &[
                pow2_pct(os_temp.saturating_sub(1))?,
                pow2_pct(os_hum.saturating_sub(1))?,
                pow2_pct(os_pres.saturating_sub(1))?,
            ],
        ));
        log::info(&pct("BMxx80: IIR: %dx", &[pow2_minus_one_pct(iir_filter)?]));
        let iir_filter = iir_filter & 0x07;

        let this = Arc::new(Bme280 {
            printer: printer.clone(),
            reactor,
            i2c,
            mcu,
            name,
            iir_filter,
            os_temp,
            os_hum,
            os_pres,
            gas_heat_temp,
            gas_heat_duration,
            temp: GCell::new(0.),
            pressure: GCell::new(0.),
            humidity: GCell::new(0.),
            gas: GCell::new(0.),
            t_fine: GCell::new(0.),
            min_temp: GCell::new(0.),
            max_temp: GCell::new(0.),
            max_sample_time: GCell::new(None),
            sample_timer: GCell::new(None),
            dig: GCell::new(None),
            chip_type: GCell::new("BMP280"),
            chip_registers: GCell::new(Regs::Bme280),
            callback: GCell::new(None),
            last_gas_time: GCell::new(None),
        });
        printer.add_object(&format!("bme280 {}", this.name), this.clone() as ObjectRef)?;
        let debugoutput = match printer.get_start_args() {
            PyValue::Dict(d) => {
                matches!(d.get(&PyValue::Str("debugoutput".to_string())), Some(v) if !matches!(v, PyValue::None))
            }
            _ => false,
        };
        if debugoutput {
            return Ok(this);
        }
        let connect = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args: &[EventArg]| {
                connect.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        this.last_gas_time.set(Some(0.));
        Ok(this)
    }

    /// `handle_connect` (`klippy/extras/bme280.py:162-164`).
    pub fn handle_connect(self: &Arc<Self>) -> KResult<()> {
        self.init_bmxx80()?;
        let timer = self.sample_timer.borrow();
        match timer.as_ref() {
            Some(timer) => {
                self.reactor.update_timer(timer, NOW);
                Ok(())
            }
            None => Err(exc("AttributeError", "'NoneType' object has no attribute 'waketime'")),
        }
    }

    // -- state accessors (Python attribute reads) --

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn chip_type(&self) -> &'static str {
        self.chip_type.get()
    }

    /// `self.dig`, `None` until `_init_bmxx80` read the calibration data.
    pub fn dig(&self) -> Option<IndexMap<String, f64>> {
        self.dig.borrow().as_ref().map(|d| (**d).clone())
    }

    pub fn temp(&self) -> f64 {
        self.temp.get()
    }

    pub fn pressure(&self) -> f64 {
        self.pressure.get()
    }

    pub fn humidity(&self) -> f64 {
        self.humidity.get()
    }

    pub fn gas(&self) -> f64 {
        self.gas.get()
    }

    pub fn t_fine(&self) -> f64 {
        self.t_fine.get()
    }

    pub fn iir_filter(&self) -> i64 {
        self.iir_filter
    }

    pub fn os_temp(&self) -> i64 {
        self.os_temp
    }

    pub fn os_hum(&self) -> i64 {
        self.os_hum
    }

    pub fn os_pres(&self) -> i64 {
        self.os_pres
    }

    pub fn gas_heat_temp(&self) -> i64 {
        self.gas_heat_temp
    }

    pub fn gas_heat_duration(&self) -> i64 {
        self.gas_heat_duration
    }

    pub fn max_sample_time(&self) -> Option<f64> {
        self.max_sample_time.get()
    }

    pub fn last_gas_time(&self) -> Option<f64> {
        self.last_gas_time.get()
    }

    pub fn has_sample_timer(&self) -> bool {
        self.sample_timer.borrow().is_some()
    }

    fn dig_map(&self) -> KResult<Arc<Dig>> {
        match &*self.dig.borrow() {
            Some(d) => Ok(d.clone()),
            None => Err(type_error("'NoneType' object is not subscriptable")),
        }
    }

    /// `self.reactor.pause(self.reactor.monotonic() + secs)`.
    fn pause_for(&self, secs: f64) {
        self.reactor.pause(self.reactor.monotonic() + secs);
    }

    // -- I2C register access (`klippy/extras/bme280.py:756-779`) --

    /// `read_id` (`klippy/extras/bme280.py:756-760`).
    pub fn read_id(&self) -> KResult<i64> {
        let params = self.i2c.i2c_read(&[BME_CHIP_ID_REG], 1, QuerySendOpts::default())?;
        byte(&response_bytes(&params)?, 0)
    }

    /// `read_bmp3_id` (`klippy/extras/bme280.py:762-766`).
    pub fn read_bmp3_id(&self) -> KResult<i64> {
        let params = self.i2c.i2c_read(&[BMP3_CHIP_ID_REG], 1, QuerySendOpts::default())?;
        byte(&response_bytes(&params)?, 0)
    }

    /// `read_register` (`klippy/extras/bme280.py:768-772`).
    pub fn read_register(&self, reg_name: &str, read_len: i64) -> KResult<Vec<u8>> {
        let reg = self.chip_registers.get().lookup(reg_name)?;
        let params = self.i2c.i2c_read(&[reg as u8], read_len, QuerySendOpts::default())?;
        response_bytes(&params)
    }

    /// `write_register` (`klippy/extras/bme280.py:774-779`). Python wraps a
    /// scalar in a list; `data` is already that list. A value outside
    /// `range(0, 256)` fails when the message is encoded (`msgproto`'s
    /// `bytearray(v)`), which is `ValueError`.
    pub fn write_register(&self, reg_name: &str, data: &[i64]) -> KResult<()> {
        let reg = self.chip_registers.get().lookup(reg_name)?;
        let mut bytes = Vec::with_capacity(data.len() + 1);
        for v in std::iter::once(&reg).chain(data.iter()) {
            if !(0..=255).contains(v) {
                return Err(value_error("byte must be in range(0, 256)"));
            }
            bytes.push(*v as u8);
        }
        self.i2c.i2c_write(&bytes, QuerySendOpts::default())
    }

    /// `_init_bmxx80` (`klippy/extras/bme280.py:176-408`).
    pub fn init_bmxx80(self: &Arc<Self>) -> KResult<()> {
        let chip_id = {
            let id = self.read_id()?;
            if id != 0 {
                id
            } else {
                self.read_bmp3_id()?
            }
        };
        match bme_chip_name(chip_id) {
            None => log::info(&pct("bme280: Unknown Chip ID received %#x", &[pct_int(chip_id)])),
            Some(chip_type) => {
                self.chip_type.set(chip_type);
                log::info(&pct(
                    "bme280: Found Chip %s at %#x",
                    &[PctValue::Str(chip_type.to_string()), pct_int(self.i2c.get_i2c_address())],
                ));
            }
        }

        // Reset chip
        self.write_register("RESET", &[RESET_CHIP_VALUE])?;
        self.pause_for(0.5);

        // Make sure non-volatile memory has been copied to registers
        let chip_type = self.chip_type.get();
        if chip_type != "BMP180" {
            // BMP180 has no status register available
            let mut status = byte(&self.read_register("STATUS", 1)?, 0)?;
            while status & STATUS_IM_UPDATE != 0 {
                self.pause_for(0.01);
                status = byte(&self.read_register("STATUS", 1)?, 0)?;
            }
        }

        let kind = match chip_type {
            "BME680" => {
                self.max_sample_time.set(Some(
                    (1.25
                        + (2.3 * self.os_temp as f64)
                        + ((2.3 * self.os_pres as f64) + 0.575)
                        + ((2.3 * self.os_hum as f64) + 0.575))
                        / 1000.,
                ));
                self.chip_registers.set(Regs::Bme680);
                SampleKind::Bme680
            }
            "BMP180" => {
                self.chip_registers.set(Regs::Bmp180);
                SampleKind::Bmp180
            }
            "BMP388" => {
                self.chip_registers.set(Regs::Bmp388);
                self.write_register(
                    "PWR_CTRL",
                    &[BMP388_REG_VAL_PRESS_EN | BMP388_REG_VAL_TEMP_EN | BMP388_REG_VAL_NORMAL_MODE],
                )?;
                self.write_register("OSR", &[BMP388_REG_VAL_PRESS_OS_NO | BMP388_REG_VAL_TEMP_OS_NO])?;
                self.write_register("ORD", &[BMP388_REG_VAL_ODR_50_HZ])?;
                self.write_register("INT_CTRL", &[BMP388_REG_VAL_DRDY_EN])?;
                SampleKind::Bmp388
            }
            "BME280" => {
                self.max_sample_time.set(Some(
                    (1.25
                        + (2.3 * self.os_temp as f64)
                        + ((2.3 * self.os_pres as f64) + 0.575)
                        + ((2.3 * self.os_hum as f64) + 0.575))
                        / 1000.,
                ));
                self.chip_registers.set(Regs::Bme280);
                SampleKind::Bme280
            }
            _ => {
                self.max_sample_time.set(Some(
                    (1.25 + (2.3 * self.os_temp as f64) + ((2.3 * self.os_pres as f64) + 0.575))
                        / 1000.,
                ));
                self.chip_registers.set(Regs::Bme280);
                SampleKind::Bme280
            }
        };
        let weak: Weak<Bme280> = Arc::downgrade(self);
        let timer = self.reactor.register_timer(
            move |eventtime| {
                let Some(this) = weak.upgrade() else { return NEVER };
                let result = match kind {
                    SampleKind::Bme280 => this.sample_bme280(eventtime),
                    SampleKind::Bmp388 => this.sample_bmp388(eventtime),
                    SampleKind::Bme680 => this.sample_bme680(eventtime),
                    SampleKind::Bmp180 => this.sample_bmp180(eventtime),
                };
                // Python lets the exception escape the timer callback and the
                // reactor dispatch loop dies with it; a panic is the Rust
                // equivalent (`lm75` precedent).
                result.unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message()))
            },
            NEVER,
        );
        self.sample_timer.replace(Some(timer));

        // Read out and calculate the trimming parameters
        let (cal_1, cal_2) = if chip_type == "BMP180" {
            (self.read_register("CAL_1", 22)?, Vec::new())
        } else if chip_type == "BMP388" {
            (self.read_register("CAL_1", 21)?, Vec::new())
        } else {
            (self.read_register("CAL_1", 26)?, self.read_register("CAL_2", 16)?)
        };
        let dig = match chip_type {
            "BME280" => Some(read_calibration_data_bme280(&cal_1, &cal_2)?),
            "BMP280" => Some(read_calibration_data_bmp280(&cal_1)?),
            "BME680" => Some(read_calibration_data_bme680(&cal_1, &cal_2)?),
            "BMP180" => Some(read_calibration_data_bmp180(&cal_1)?),
            "BMP388" => Some(read_calibration_data_bmp388(&cal_1)?),
            _ => None,
        };
        if let Some(dig) = dig {
            self.dig.replace(Some(Arc::new(dig)));
        }

        if chip_type == "BME280" || chip_type == "BMP280" {
            let max_standby_time = REPORT_TIME - self.max_sample_time.get().unwrap_or(0.);
            // 0.5 ms
            let mut t_sb: i64 = 0;
            if chip_type == "BME280" {
                if max_standby_time > 1. {
                    t_sb = 5;
                } else if max_standby_time > 0.5 {
                    t_sb = 4;
                } else if max_standby_time > 0.25 {
                    t_sb = 3;
                } else if max_standby_time > 0.125 {
                    t_sb = 2;
                } else if max_standby_time > 0.0625 {
                    t_sb = 1;
                } else if max_standby_time > 0.020 {
                    t_sb = 7;
                } else if max_standby_time > 0.010 {
                    t_sb = 6;
                }
            } else if max_standby_time > 4. {
                t_sb = 7;
            } else if max_standby_time > 2. {
                t_sb = 6;
            } else if max_standby_time > 1. {
                t_sb = 5;
            } else if max_standby_time > 0.5 {
                t_sb = 4;
            } else if max_standby_time > 0.25 {
                t_sb = 3;
            } else if max_standby_time > 0.125 {
                t_sb = 2;
            } else if max_standby_time > 0.0625 {
                t_sb = 1;
            }

            let cfg = py_shl(t_sb, 5)? | py_shl(self.iir_filter, 2)?;
            self.write_register("CONFIG", &[cfg])?;
            if chip_type == "BME280" {
                self.write_register("CTRL_HUM", &[self.os_hum])?;
            }
            // Enter normal (periodic) mode
            let meas = py_shl(self.os_temp, 5)? | py_shl(self.os_pres, 2)? | MODE_PERIODIC;
            self.write_register("CTRL_MEAS", &[meas])?;
        }

        if chip_type == "BME680" {
            self.write_register("CONFIG", &[py_shl(self.iir_filter, 2)?])?;
            // Should be set once and reused on every mode register write
            self.write_register("CTRL_HUM", &[self.os_hum & 0x07])?;
            let gas_wait_0 = self.calc_gas_heater_duration(self.gas_heat_duration);
            self.write_register("GAS_WAIT_0", &[gas_wait_0])?;
            let res_heat_0 = self.calc_gas_heater_resistance(self.gas_heat_temp)?;
            self.write_register("RES_HEAT_0", &[res_heat_0])?;
            // Set initial heater current to reach Gas heater target on start
            self.write_register("IDAC_HEAT_0", &[96])?;
        }
        Ok(())
    }

    /// `self._callback(self.mcu.estimated_print_time(measured_time), self.temp)`.
    fn call_callback(&self, measured_time: f64) -> KResult<()> {
        // Python looks the attribute up before it evaluates the arguments.
        let callback = self.callback.borrow_mut().take();
        let Some(mut callback) = callback else {
            return Err(exc("AttributeError", "'BME280' object has no attribute '_callback'"));
        };
        // The callback may call back into the printer; keep no borrow while it runs.
        callback(self.mcu.estimated_print_time(measured_time), self.temp.get());
        let mut slot = self.callback.borrow_mut();
        if slot.is_none() {
            *slot = Some(callback);
        }
        Ok(())
    }

    /// The temperature range check of every `_sample_*` method.
    fn check_temp_range(&self, chip: &str) {
        let temp = self.temp.get();
        let min_temp = self.min_temp.get();
        let max_temp = self.max_temp.get();
        if temp < min_temp || temp > max_temp {
            self.printer.invoke_shutdown(
                &pct(
                    &format!("{chip} temperature %0.1f outside range of %0.1f:%.01f"),
                    &[pct_float(temp), pct_float(min_temp), pct_float(max_temp)],
                ),
                PyValue::Dict(IndexMap::new()),
            );
        }
    }

    /// `self.temp = self.pressure = self.humidity = .0` of the `except` branch
    /// of `_sample_bme280`.
    fn zero_bme280(&self) {
        self.temp.set(0.);
        self.pressure.set(0.);
        self.humidity.set(0.);
    }

    /// `_sample_bme280` (`klippy/extras/bme280.py:410-438`).
    pub fn sample_bme280(&self, _eventtime: f64) -> KResult<f64> {
        let read = match self.chip_type.get() {
            "BME280" => self.read_register("PRESSURE_MSB", 8).map(Some),
            "BMP280" => self.read_register("PRESSURE_MSB", 6).map(Some),
            _ => Ok(None),
        };
        let data = match read {
            Ok(Some(data)) => data,
            Ok(None) => return Ok(NEVER),
            Err(e) => {
                log::exception("BME280: Error reading data", &exc_text(&e));
                self.zero_bme280();
                return Ok(NEVER);
            }
        };

        let temp_raw = (byte(&data, 3)? << 12) | (byte(&data, 4)? << 4) | (byte(&data, 5)? >> 4);
        self.temp.set(self.compensate_temp(temp_raw)?);
        let pressure_raw = (byte(&data, 0)? << 12) | (byte(&data, 1)? << 4) | (byte(&data, 2)? >> 4);
        self.pressure.set(self.compensate_pressure_bme280(pressure_raw)? / 100.);
        if self.chip_type.get() == "BME280" {
            let humid_raw = (byte(&data, 6)? << 8) | byte(&data, 7)?;
            self.humidity.set(self.compensate_humidity_bme280(humid_raw)?);
        }
        self.check_temp_range("BME280");
        let measured_time = self.reactor.monotonic();
        self.call_callback(measured_time)?;
        Ok(measured_time + REPORT_TIME)
    }

    /// `_sample_bmp388` (`klippy/extras/bme280.py:440-455`).
    pub fn sample_bmp388(&self, _eventtime: f64) -> KResult<f64> {
        let status = self.read_register("STATUS", 1)?;
        if byte(&status, 0)? & 0b100000 != 0 {
            let temp = self.sample_bmp388_temp()?;
            self.temp.set(temp);
            self.check_temp_range("BME280");
        }

        if byte(&status, 0)? & 0b010000 != 0 {
            // Int results are 30000 or 125000; `/ 100.` of either is exact.
            let press = self.sample_bmp388_press()?.to_f64().unwrap_or(f64::NAN);
            self.pressure.set(press / 100.0);
        }

        let measured_time = self.reactor.monotonic();
        self.call_callback(measured_time)?;
        Ok(measured_time + REPORT_TIME)
    }

    /// `_sample_bmp388_temp` (`klippy/extras/bme280.py:457-475`).
    pub fn sample_bmp388_temp(&self) -> KResult<f64> {
        let xlsb = self.read_register("TEMP_XLSB", 1)?;
        let lsb = self.read_register("TEMP_LSB", 1)?;
        let msb = self.read_register("TEMP_MSB", 1)?;
        let adc_t = (byte(&msb, 0)? << 16) + (byte(&lsb, 0)? << 8) + byte(&xlsb, 0)?;

        let dig = self.dig_map()?;
        let partial_data1 = adc_t as f64 - dig_val(&dig, "T1")?;
        let partial_data2 = dig_val(&dig, "T2")? * partial_data1;

        let mut t_fine = partial_data2;
        t_fine += (partial_data1 * partial_data1) * dig_val(&dig, "T3")?;

        t_fine = t_fine.clamp(-40.0, 85.0);
        self.t_fine.set(t_fine);
        Ok(t_fine)
    }

    /// `_sample_bmp388_press` (`klippy/extras/bme280.py:477-511`). The clamps
    /// assign the int literals `30000` and `125000`, so a clamped result is a
    /// Python int while an in-range one is a float.
    pub fn sample_bmp388_press(&self) -> KResult<PyValue> {
        let xlsb = self.read_register("PRESS_XLSB", 1)?;
        let lsb = self.read_register("PRESS_LSB", 1)?;
        let msb = self.read_register("PRESS_MSB", 1)?;
        let adc_p = (byte(&msb, 0)? << 16) + (byte(&lsb, 0)? << 8) + byte(&xlsb, 0)?;

        let dig = self.dig_map()?;
        let t_fine = self.t_fine.get();
        let partial_data1 = dig_val(&dig, "P6")? * t_fine;
        let partial_data2 = dig_val(&dig, "P7")? * (t_fine * t_fine);
        let mut partial_data3 = dig_val(&dig, "P8")?;
        partial_data3 *= t_fine * t_fine * t_fine;
        let mut partial_out1 = dig_val(&dig, "P5")?;
        partial_out1 += partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = dig_val(&dig, "P2")? * t_fine;
        let partial_data2 = dig_val(&dig, "P3")? * (t_fine * t_fine);
        let mut partial_data3 = dig_val(&dig, "P4")?;
        partial_data3 *= t_fine * t_fine * t_fine;
        let partial_out2 =
            adc_p as f64 * (dig_val(&dig, "P1")? + partial_data1 + partial_data2 + partial_data3);

        // `adc_P * adc_P` is an exact Python int; it stays exact as a double.
        let adc = adc_p as f64;
        let partial_data1 = adc * adc;
        let partial_data2 = dig_val(&dig, "P9")? + (dig_val(&dig, "P10")? * t_fine);
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + adc * adc * adc * dig_val(&dig, "P11")?;

        let comp_press = partial_out1 + partial_out2 + partial_data4;

        if comp_press < 30000. {
            return Ok(PyValue::Int(PyInt::from_i64(30000)));
        }
        if comp_press > 125000. {
            return Ok(PyValue::Int(PyInt::from_i64(125000)));
        }

        Ok(PyValue::Float(comp_press))
    }

    /// The `try` block of `_sample_bme680` after the measurement pause
    /// (`klippy/extras/bme280.py:528-542`). `Ok(None)` is "nothing in progress
    /// and no new data".
    fn read_bme680_data(&self, run_gas: bool) -> KResult<Option<(Vec<u8>, Vec<u8>)>> {
        // wait until results are ready
        let mut status = byte(&self.read_register("EAS_STATUS_0", 1)?, 0)?;
        while status & MEASURE_IN_PROGRESS != 0 {
            let Some(max_sample_time) = self.max_sample_time.get() else {
                return Err(type_error("unsupported operand type(s) for +: 'float' and 'NoneType'"));
            };
            self.pause_for(max_sample_time);
            status = byte(&self.read_register("EAS_STATUS_0", 1)?, 0)?;
        }

        // Nothing in progress and no new data
        if status & EAS_NEW_DATA == 0 {
            return Ok(None);
        }
        let data = self.read_register("PRESSURE_MSB", 8)?;
        let gas_data = if run_gas { self.read_register("GAS_R_MSB", 2)? } else { vec![0, 0] };
        Ok(Some((data, gas_data)))
    }

    /// `_sample_bme680` (`klippy/extras/bme280.py:513-577`).
    pub fn sample_bme680(&self, _eventtime: f64) -> KResult<f64> {
        let mut run_gas = false;
        // Check VOC once a while
        let Some(last_gas_time) = self.last_gas_time.get() else {
            return Err(exc("AttributeError", "'BME280' object has no attribute 'last_gas_time'"));
        };
        if self.reactor.monotonic() - last_gas_time > 3. {
            let gas_config = RUN_GAS | NB_CONV_0;
            self.write_register("CTRL_GAS_1", &[gas_config])?;
            run_gas = true;
        }

        // Enter forced mode
        let meas = py_shl(self.os_temp, 5)? | py_shl(self.os_pres, 2)? | MODE;
        self.write_register("CTRL_MEAS", &[meas])?;
        let mut max_sample_time = self.max_sample_time.get();
        if run_gas {
            max_sample_time = match max_sample_time {
                Some(v) => Some(v + self.gas_heat_duration as f64 / 1000.),
                None => {
                    return Err(type_error("unsupported operand type(s) for +=: 'NoneType' and 'float'"))
                }
            };
        }
        match max_sample_time {
            Some(v) => self.pause_for(v),
            None => {
                return Err(type_error("unsupported operand type(s) for +: 'float' and 'NoneType'"))
            }
        }
        let (data, gas_data) = match self.read_bme680_data(run_gas) {
            Ok(Some(read)) => read,
            Ok(None) => return Ok(self.reactor.monotonic() + REPORT_TIME),
            Err(e) => {
                log::exception("BME680: Error reading data", &exc_text(&e));
                self.temp.set(0.);
                self.pressure.set(0.);
                self.humidity.set(0.);
                self.gas.set(0.);
                return Ok(NEVER);
            }
        };

        let temp_raw = (byte(&data, 3)? << 12) | (byte(&data, 4)? << 4) | (byte(&data, 5)? >> 4);
        if temp_raw != 0x80000 {
            self.temp.set(self.compensate_temp(temp_raw)?);
        }
        let pressure_raw = (byte(&data, 0)? << 12) | (byte(&data, 1)? << 4) | (byte(&data, 2)? >> 4);
        if pressure_raw != 0x80000 {
            self.pressure.set(self.compensate_pressure_bme680(pressure_raw)? / 100.);
        }
        let humid_raw = (byte(&data, 6)? << 8) | byte(&data, 7)?;
        self.humidity.set(self.compensate_humidity_bme680(humid_raw)?);

        let gas_valid = (byte(&gas_data, 1)? & 0x20) == 0x20;
        if gas_valid {
            let gas_heater_stable = (byte(&gas_data, 1)? & 0x10) == 0x10;
            if !gas_heater_stable {
                log::warning("BME680: Gas heater didn't reach target");
            }
            let gas_raw = (byte(&gas_data, 0)? << 2) | ((byte(&gas_data, 1)? & 0xC0) >> 6);
            let gas_range = byte(&gas_data, 1)? & 0x0F;
            self.gas.set(self.compensate_gas(gas_raw, gas_range)?);
            // Disable gas measurement on success
            self.write_register("CTRL_GAS_1", &[NB_CONV_0])?;
            self.last_gas_time.set(Some(self.reactor.monotonic()));
        }

        self.check_temp_range("BME680");
        let measured_time = self.reactor.monotonic();
        self.call_callback(measured_time)?;
        Ok(measured_time + REPORT_TIME)
    }

    /// First `try` block of `_sample_bmp180` (`klippy/extras/bme280.py:582-587`).
    fn read_bmp180_temp(&self) -> KResult<i64> {
        self.pause_for(0.01);
        let data = self.read_register("REG_MSB", 2)?;
        Ok((byte(&data, 0)? << 8) | byte(&data, 1)?)
    }

    /// Second `try` block of `_sample_bmp180` (`klippy/extras/bme280.py:595-602`).
    fn read_bmp180_pressure(&self) -> KResult<i64> {
        self.pause_for(0.01);
        let data = self.read_register("REG_MSB", 3)?;
        py_shr(
            (byte(&data, 0)? << 16) | (byte(&data, 1)? << 8) | byte(&data, 2)?,
            8i64.saturating_sub(self.os_pres),
        )
    }

    /// `_sample_bmp180` (`klippy/extras/bme280.py:579-613`).
    pub fn sample_bmp180(&self, _eventtime: f64) -> KResult<f64> {
        let meas = self.chip_registers.get().lookup("CRV_TEMP")?;
        self.write_register("CTRL_MEAS", &[meas])?;

        let temp_raw = match self.read_bmp180_temp() {
            Ok(v) => v,
            Err(e) => {
                log::exception("BMP180: Error reading temperature", &exc_text(&e));
                self.temp.set(0.);
                self.pressure.set(0.);
                return Ok(NEVER);
            }
        };

        let meas = self.chip_registers.get().lookup("CRV_PRES")? | py_shl(self.os_pres, 6)?;
        self.write_register("CTRL_MEAS", &[meas])?;

        let pressure_raw = match self.read_bmp180_pressure() {
            Ok(v) => v,
            Err(e) => {
                log::exception("BMP180: Error reading pressure", &exc_text(&e));
                self.temp.set(0.);
                self.pressure.set(0.);
                return Ok(NEVER);
            }
        };

        self.temp.set(self.compensate_temp_bmp180(temp_raw)?);
        self.pressure.set(self.compensate_pressure_bmp180(pressure_raw)? / 100.);
        self.check_temp_range("BMP180");
        let measured_time = self.reactor.monotonic();
        self.call_callback(measured_time)?;
        Ok(measured_time + REPORT_TIME)
    }

    /// `_compensate_temp` (`klippy/extras/bme280.py:616-623`).
    pub fn compensate_temp(&self, raw_temp: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let raw = raw_temp as f64;
        let var1 = (raw / 16384. - (dig_val(&dig, "T1")? / 1024.)) * dig_val(&dig, "T2")?;
        let var2 = ((raw / 131072.) - (dig_val(&dig, "T1")? / 8192.))
            * ((raw / 131072.) - (dig_val(&dig, "T1")? / 8192.))
            * dig_val(&dig, "T3")?;
        let t_fine = var1 + var2;
        self.t_fine.set(t_fine);
        Ok(t_fine / 5120.0)
    }

    /// `_compensate_pressure_bme280` (`klippy/extras/bme280.py:625-641`).
    pub fn compensate_pressure_bme280(&self, raw_pressure: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let t_fine = self.t_fine.get();
        let var1 = t_fine / 2. - 64000.;
        let mut var2 = var1 * var1 * dig_val(&dig, "P6")? / 32768.;
        var2 += var1 * dig_val(&dig, "P5")? * 2.;
        var2 = var2 / 4. + (dig_val(&dig, "P4")? * 65536.);
        let mut var1 =
            (dig_val(&dig, "P3")? * var1 * var1 / 524288. + dig_val(&dig, "P2")? * var1) / 524288.;
        var1 = (1. + var1 / 32768.) * dig_val(&dig, "P1")?;
        if var1 == 0. {
            return Ok(0.);
        }
        let mut pressure = 1048576.0 - raw_pressure as f64;
        pressure = ((pressure - var2 / 4096.) * 6250.) / var1;
        let var1 = dig_val(&dig, "P9")? * pressure * pressure / 2147483648.;
        let var2 = pressure * dig_val(&dig, "P8")? / 32768.;
        Ok(pressure + (var1 + var2 + dig_val(&dig, "P7")?) / 16.)
    }

    /// `_compensate_pressure_bme680` (`klippy/extras/bme280.py:643-661`).
    pub fn compensate_pressure_bme680(&self, raw_pressure: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let t_fine = self.t_fine.get();
        let var1 = t_fine / 2. - 64000.;
        let mut var2 = var1 * var1 * dig_val(&dig, "P6")? / 131072.;
        var2 += var1 * dig_val(&dig, "P5")? * 2.;
        var2 = var2 / 4. + (dig_val(&dig, "P4")? * 65536.);
        let mut var1 =
            (dig_val(&dig, "P3")? * var1 * var1 / 16384. + dig_val(&dig, "P2")? * var1) / 524288.;
        var1 = (1. + var1 / 32768.) * dig_val(&dig, "P1")?;
        if var1 == 0. {
            return Ok(0.);
        }
        let mut pressure = 1048576.0 - raw_pressure as f64;
        pressure = ((pressure - var2 / 4096.) * 6250.) / var1;
        let var1 = dig_val(&dig, "P9")? * pressure * pressure / 2147483648.;
        let var2 = pressure * dig_val(&dig, "P8")? / 32768.;
        let var3 = (pressure / 256.)
            * (pressure / 256.)
            * (pressure / 256.)
            * (dig_val(&dig, "P10")? / 131072.);
        Ok(pressure + (var1 + var2 + var3 + (dig_val(&dig, "P7")? * 128.)) / 16.)
    }

    /// `_compensate_humidity_bme280` (`klippy/extras/bme280.py:663-674`).
    pub fn compensate_humidity_bme280(&self, raw_humidity: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let t_fine = self.t_fine.get();
        let mut humidity = t_fine - 76800.;
        let h1 = raw_humidity as f64
            - (dig_val(&dig, "H4")? * 64. + dig_val(&dig, "H5")? / 16384. * humidity);
        let h2 = dig_val(&dig, "H2")? / 65536.
            * (1.
                + dig_val(&dig, "H6")? / 67108864.
                    * humidity
                    * (1. + dig_val(&dig, "H3")? / 67108864. * humidity));
        humidity = h1 * h2;
        humidity *= 1. - dig_val(&dig, "H1")? * humidity / 524288.;
        Ok(py_min(100., py_max(0., humidity)))
    }

    /// `_compensate_humidity_bme680` (`klippy/extras/bme280.py:676-688`).
    pub fn compensate_humidity_bme680(&self, raw_humidity: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let temp_comp = self.temp.get();

        let var1 = raw_humidity as f64
            - ((dig_val(&dig, "H1")? * 16.) + ((dig_val(&dig, "H3")? / 2.) * temp_comp));
        let var2 = var1
            * ((dig_val(&dig, "H2")? / 262144.)
                * (1.
                    + ((dig_val(&dig, "H4")? / 16384.) * temp_comp)
                    + ((dig_val(&dig, "H5")? / 1048576.) * temp_comp * temp_comp)));
        let var3 = dig_val(&dig, "H6")? / 16384.;
        let var4 = dig_val(&dig, "H7")? / 2097152.;
        let humidity = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
        Ok(py_min(100., py_max(0., humidity)))
    }

    /// `_compensate_gas` (`klippy/extras/bme280.py:690-696`).
    pub fn compensate_gas(&self, gas_raw: i64, gas_range: i64) -> KResult<f64> {
        let gas_switching_error = byte(&self.read_register("RANGE_SWITCHING_ERROR", 1)?, 0)?;
        let (k1, k2) = usize::try_from(gas_range)
            .ok()
            .and_then(|i| BME680_GAS_CONSTANTS.get(i))
            .copied()
            .ok_or_else(|| exc("KeyError", gas_range.to_string()))?;
        let var1 = (1340. + 5. * gas_switching_error as f64) * k1;
        div(var1 * k2, gas_raw as f64 - 512. + var1)
    }

    /// `_calc_gas_heater_resistance` (`klippy/extras/bme280.py:698-711`).
    pub fn calc_gas_heater_resistance(&self, target_temp: i64) -> KResult<i64> {
        let amb_temp = self.temp.get();
        let heater_data = self.read_register("RES_HEAT_VAL", 3)?;
        let res_heat_val = get_signed_byte(byte(&heater_data, 0)?) as f64;
        let res_heat_range = ((byte(&heater_data, 2)? & 0x30) >> 4) as f64;
        let dig = self.dig_map()?;
        let var1 = (dig_val(&dig, "G1")? / 16.) + 49.;
        let var2 = ((dig_val(&dig, "G2")? / 32768.) * 0.0005) + 0.00235;
        let var3 = dig_val(&dig, "G3")? / 1024.;
        let var4 = var1 * (1. + (var2 * target_temp as f64));
        let var5 = var4 + (var3 * amb_temp);
        let res_heat = 3.4
            * ((var5 * (4. / (4. + res_heat_range)) * (1. / (1. + (res_heat_val * 0.002)))) - 25.);
        py_int_of(res_heat)
    }

    /// `_calc_gas_heater_duration` (`klippy/extras/bme280.py:713-723`).
    pub fn calc_gas_heater_duration(&self, duration_ms: i64) -> i64 {
        if duration_ms >= 4032 {
            return 0xff;
        }
        let mut duration_ms = duration_ms;
        let mut factor: i64 = 0;
        while duration_ms > 0x3F {
            duration_ms /= 4;
            factor += 1;
        }
        duration_ms + (factor * 64)
    }

    /// `_compensate_temp_bmp180` (`klippy/extras/bme280.py:725-731`).
    pub fn compensate_temp_bmp180(&self, raw_temp: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let x1 = (raw_temp as f64 - dig_val(&dig, "AC6")?) * dig_val(&dig, "AC5")? / 32768.;
        let x2 = div(dig_val(&dig, "MC")? * 2048., x1 + dig_val(&dig, "MD")?)?;
        let b5 = x1 + x2;
        self.t_fine.set(b5);
        Ok((b5 + 8.) / 16. / 10.)
    }

    /// `_compensate_pressure_bmp180` (`klippy/extras/bme280.py:733-754`).
    pub fn compensate_pressure_bmp180(&self, raw_pressure: i64) -> KResult<f64> {
        let dig = self.dig_map()?;
        let b5 = self.t_fine.get();
        let b6 = b5 - 4000.;
        let x1 = (dig_val(&dig, "B2")? * (b6 * b6 / 4096.)) / 2048.;
        let x2 = dig_val(&dig, "AC2")? * b6 / 2048.;
        let x3 = x1 + x2;
        let b3 = shl_plus_two_over_four(py_int_of(dig_val(&dig, "AC1")? * 4. + x3)?, self.os_pres)?;
        let x1 = dig_val(&dig, "AC3")? * b6 / 8192.;
        let x2 = (dig_val(&dig, "B1")? * (b6 * b6 / 4096.)) / 65536.;
        let x3 = ((x1 + x2) + 2.) / 4.;
        let b4 = dig_val(&dig, "AC4")? * (x3 + 32768.) / 32768.;
        let b7 = (raw_pressure as f64 - b3) * py_shr(50000, self.os_pres)? as f64;
        let mut p = if b7 < 2147483648. { div(b7 * 2., b4)? } else { div(b7, b4)? * 2. };
        let mut x1 = (p / 256.) * (p / 256.);
        x1 = (x1 * 3038.) / 65536.;
        let x2 = (-7357. * p) / 65536.;
        p += (x1 + x2 + 3791.) / 16.;
        Ok(p)
    }
}

/// `bytearray(params['response'])` (`klippy/extras/bme280.py:759,766,771`).
fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
    let PyValue::Dict(dict) = params else {
        return Err(type_error(format!("'{}' object is not subscriptable", type_name(params))));
    };
    let Some(response) = dict.get(&PyValue::Str("response".to_string())) else {
        return Err(key_error("response"));
    };
    match response {
        PyValue::Bytes(b) => Ok(b.clone()),
        PyValue::List(items) | PyValue::Tuple(items) => items.iter().map(element_byte).collect(),
        PyValue::Dict(d) => d.keys().map(element_byte).collect(),
        PyValue::Int(n) => {
            let n = i64::try_from(n.inner()).map_err(|_| {
                exc(
                    "OverflowError",
                    "cannot fit 'int' into an index-sized integer",
                )
            })?;
            if n < 0 {
                return Err(value_error("negative count"));
            }
            if n > (1 << 32) {
                return Err(exc("MemoryError", ""));
            }
            Ok(vec![0; n as usize])
        }
        PyValue::Bool(b) => Ok(vec![0; *b as usize]),
        PyValue::Str(_) => Err(type_error("string argument without an encoding")),
        other => Err(type_error(format!("cannot convert '{}' object to bytearray", type_name(other)))),
    }
}

/// One element of an iterable handed to `bytearray(...)`.
fn element_byte(item: &PyValue) -> KResult<u8> {
    let v = match item {
        PyValue::Int(n) => i64::try_from(n.inner()).ok(),
        PyValue::Bool(b) => Some(*b as i64),
        other => {
            return Err(type_error(format!(
                "'{}' object cannot be interpreted as an integer",
                type_name(other)
            )))
        }
    };
    match v {
        Some(v) if (0..=255).contains(&v) => Ok(v as u8),
        _ => Err(value_error("byte must be in range(0, 256)")),
    }
}

impl TemperatureSensorApi for Bme280 {
    /// `setup_minmax(min_temp, max_temp)` (`klippy/extras/bme280.py:166-168`).
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        self.min_temp.set(min_temp);
        self.max_temp.set(max_temp);
    }

    /// `setup_callback(cb)` (`klippy/extras/bme280.py:170-171`).
    fn setup_callback(&self, callback: TemperatureCallback) {
        self.callback.replace(Some(callback));
    }

    /// `get_report_time_delta()` (`klippy/extras/bme280.py:173-174`).
    fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }
}

impl PrinterObject for Bme280 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);

    /// `get_status(eventtime)` (`klippy/extras/bme280.py:781-790`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut data: IndexMap<PyValue, PyValue> = IndexMap::new();
        data.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(self.temp.get(), 2)),
        );
        data.insert(PyValue::Str("pressure".to_string()), PyValue::Float(self.pressure.get()));
        let chip_type = self.chip_type.get();
        if chip_type == "BME280" || chip_type == "BME680" {
            data.insert(PyValue::Str("humidity".to_string()), PyValue::Float(self.humidity.get()));
        }
        if chip_type == "BME680" {
            data.insert(PyValue::Str("gas".to_string()), PyValue::Float(self.gas.get()));
        }
        Some(PyValue::Dict(data))
    }
}

// ---------------------------------------------------------------------------
// Module entry points (`klippy/extras/bme280.py:793-796`)
// ---------------------------------------------------------------------------

/// `load_config(config)`.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    // Register sensor
    let printer = config.get_printer();
    let pheaters =
        load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
    let factory: SensorFactory =
        Arc::new(|config: &Config| Ok(Bme280::new(config)? as Arc<dyn TemperatureSensorApi>));
    pheaters.add_sensor_factory("BME280", factory);
    Ok(None)
}

/// Module entry points for the `BME280` sensor.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
