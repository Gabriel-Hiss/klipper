//! Port of `klippy/extras/tmc2660.py` (the TMC2660 stepper driver).
//!
//! The module covers the five inventory units: the register/field tables
//! ([`registers`], [`read_registers`], [`fields`], [`signed_fields`],
//! [`field_formatters`], [`MAX_CURRENT`]), [`Tmc2660CurrentHelper`] (the
//! `run_current` / `sense_resistor` current helper), [`McuTmc2660Spi`]
//! (the SPI shim, including the `debugoutput` short-circuit in
//! `get_register_raw`), and [`Tmc2660`] (the `[tmc2660 <stepper>]`
//! printer object, [`load_config_prefix`] and [`module_factory`]).
//!
//! [`FieldHelper`] is the `tmc.py` field-packing helper from
//! [`crate::tmc`], and [`McuSpi`] / [`mcu_spi_from_config`] are the
//! `bus.py` SPI helper from [`crate::bus`]; their `tmc` command-helper
//! half is [`TMCCommandHelper`]. [`McuTmc2660Spi`] implements
//! [`crate::tmc::McuTmcApi`], [`Tmc2660CurrentHelper`] implements
//! [`crate::tmc::TmcCurrentHelperApi`], and [`Tmc2660`] declares
//! [`crate::tmc::TMCCommandHelperApi`] because `endstop_phase` and
//! `angle` look the driver up by name and call `get_phase_offset`.
//!
//! An exception raised inside the deferred current update registered by
//! `idle_timeout:printing` / `idle_timeout:ready` is not caught in Python:
//! it unwinds the reactor and ends the process with a traceback. The port
//! panics with the exception class and message in the same place.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetIntOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{Mcu, QuerySendOpts, SendOpts};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    ModuleFactory, Printer, EV_IDLE_TIMEOUT_PRINTING, EV_IDLE_TIMEOUT_READY,
};
use pycompat::int::PyInt;
use pycompat::text::str_repr;
use pycompat::PyValue;
use reactor::{ReactorMutex, NOW};

use crate::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};
use crate::tmc::{
    ConfigFieldDefault, FieldFormatter, FieldHelper, McuTmcApi, RegisterRaw, TMCCommandHelper,
    TMCCommandHelperApi, TmcCurrentHelperApi,
};

/// `MAX_CURRENT = 2.400` (tmc2660.py line 113).
pub const MAX_CURRENT: f64 = 2.400;

// ---------------------------------------------------------------------------
// Registration tables (units ModuleHeader / ModuleHeader_1).
// ---------------------------------------------------------------------------

/// Register name -> register address (`Registers`, tmc2660.py:10).
pub fn registers() -> IndexMap<String, i64> {
    IndexMap::from([
        ("DRVCONF".to_string(), 0xE),
        ("SGCSCONF".to_string(), 0xC),
        ("SMARTEN".to_string(), 0xA),
        ("CHOPCONF".to_string(), 0x8),
        ("DRVCTRL".to_string(), 0x0),
    ])
}

/// Registers readable via RDSEL (`ReadRegisters`, tmc2660.py:15).
pub fn read_registers() -> Vec<String> {
    [
        "READRSP@RDSEL0".to_string(),
        "READRSP@RDSEL1".to_string(),
        "READRSP@RDSEL2".to_string(),
    ]
    .into()
}

/// Register name -> (field name -> mask) (`Fields`, tmc2660.py:17-95),
/// with the Python dict insertion order.
pub fn fields() -> IndexMap<String, IndexMap<String, i64>> {
    let reg = |pairs: &[(&str, i64)]| {
        IndexMap::from_iter(pairs.iter().map(|(n, m)| (n.to_string(), *m)))
    };
    IndexMap::from([
        (
            "DRVCTRL".to_string(),
            reg(&[("mres", 0x0f), ("dedge", 0x01 << 8), ("intpol", 0x01 << 9)]),
        ),
        (
            "CHOPCONF".to_string(),
            reg(&[
                ("toff", 0x0f),
                ("hstrt", 0x7 << 4),
                ("hend", 0x0f << 7),
                ("hdec", 0x03 << 11),
                ("rndtf", 0x01 << 13),
                ("chm", 0x01 << 14),
                ("tbl", 0x03 << 15),
            ]),
        ),
        (
            "SMARTEN".to_string(),
            reg(&[
                ("semin", 0x0f),
                ("seup", 0x03 << 5),
                ("semax", 0x0f << 8),
                ("sedn", 0x03 << 13),
                ("seimin", 0x01 << 15),
            ]),
        ),
        (
            "SGCSCONF".to_string(),
            reg(&[("cs", 0x1f), ("sgt", 0x7F << 8), ("sfilt", 0x01 << 16)]),
        ),
        (
            "DRVCONF".to_string(),
            reg(&[
                ("rdsel", 0x03 << 4),
                ("vsense", 0x01 << 6),
                ("sdoff", 0x01 << 7),
                ("ts2g", 0x03 << 8),
                ("diss2g", 0x01 << 10),
                ("slpl", 0x03 << 12),
                ("slph", 0x03 << 14),
                ("tst", 0x01 << 16),
            ]),
        ),
        (
            "READRSP@RDSEL0".to_string(),
            reg(&[
                ("stallguard", 0x01 << 4),
                ("ot", 0x01 << 5),
                ("otpw", 0x01 << 6),
                ("s2ga", 0x01 << 7),
                ("s2gb", 0x01 << 8),
                ("ola", 0x01 << 9),
                ("olb", 0x01 << 10),
                ("stst", 0x01 << 11),
                ("mstep", 0x3ff << 14),
            ]),
        ),
        (
            "READRSP@RDSEL1".to_string(),
            reg(&[
                ("stallguard", 0x01 << 4),
                ("ot", 0x01 << 5),
                ("otpw", 0x01 << 6),
                ("s2ga", 0x01 << 7),
                ("s2gb", 0x01 << 8),
                ("ola", 0x01 << 9),
                ("olb", 0x01 << 10),
                ("stst", 0x01 << 11),
                ("sg_result", 0x3ff << 14),
            ]),
        ),
        (
            "READRSP@RDSEL2".to_string(),
            reg(&[
                ("stallguard", 0x01 << 4),
                ("ot", 0x01 << 5),
                ("otpw", 0x01 << 6),
                ("s2ga", 0x01 << 7),
                ("s2gb", 0x01 << 8),
                ("ola", 0x01 << 9),
                ("olb", 0x01 << 10),
                ("stst", 0x01 << 11),
                ("se", 0x1f << 14),
                ("sg_result@rdsel2", 0x1f << 19),
            ]),
        ),
    ])
}

/// Signed fields (`SignedFields`, tmc2660.py:97).
pub fn signed_fields() -> Vec<String> {
    vec!["sgt".to_string()]
}

/// `FieldFormatters`, tmc2660.py:99-106: the `tmc2130.FieldFormatters`
/// table (tmc2130.py:97-112) updated with the tmc2660 overrides, in
/// `dict.update` order (overwritten keys keep their original position).
pub fn field_formatters() -> IndexMap<String, FieldFormatter> {
    let mut map: IndexMap<String, FieldFormatter> = IndexMap::new();
    let mut insert = |name: &str, f: Arc<dyn Fn(i64) -> String + Send + Sync>| {
        map.insert(name.to_string(), f);
    };
    // tmc2130.py:97-112, in order.
    insert("i_scale_analog", Arc::new(|v| if v != 0 { "1(ExtVREF)".into() } else { String::new() }));
    insert("shaft", Arc::new(|v| if v != 0 { "1(Reverse)".into() } else { String::new() }));
    insert("reset", Arc::new(|v| if v != 0 { "1(Reset)".into() } else { String::new() }));
    insert("drv_err", Arc::new(|v| if v != 0 { "1(ErrorShutdown!)".into() } else { String::new() }));
    insert("uv_cp", Arc::new(|v| if v != 0 { "1(Undervoltage!)".into() } else { String::new() }));
    insert("version", Arc::new(format_version_hash_x));
    insert("mres", Arc::new(|v| format!("{}({}usteps)", v, mres_step(v))));
    insert("otpw", Arc::new(|v| if v != 0 { "1(OvertempWarning!)".into() } else { String::new() }));
    insert("ot", Arc::new(|v| if v != 0 { "1(OvertempError!)".into() } else { String::new() }));
    insert("s2ga", Arc::new(|v| if v != 0 { "1(ShortToGND_A!)".into() } else { String::new() }));
    insert("s2gb", Arc::new(|v| if v != 0 { "1(ShortToGND_B!)".into() } else { String::new() }));
    insert("ola", Arc::new(|v| if v != 0 { "1(OpenLoad_A!)".into() } else { String::new() }));
    insert("olb", Arc::new(|v| if v != 0 { "1(OpenLoad_B!)".into() } else { String::new() }));
    insert("cs_actual", Arc::new(|v| if v != 0 { format!("{}", v) } else { "0(Reset?)".into() }));
    // tmc2660.py:100-106 overrides (note: chm/vsense override to
    // "0(...)" when false, sdoff/diss2g to "" when false).
    insert("chm", Arc::new(|v| if v != 0 { "1(constant toff)".into() } else { "0(spreadCycle)".into() }));
    insert("vsense", Arc::new(|v| if v != 0 { "1(165mV)".into() } else { "0(305mV)".into() }));
    insert("sdoff", Arc::new(|v| if v != 0 { "1(Step/Dir disabled!)".into() } else { String::new() }));
    insert("diss2g", Arc::new(|v| if v != 0 { "1(Short to GND disabled!)".into() } else { String::new() }));
    insert("se", Arc::new(|v| if v != 0 { format!("{}", v) } else { "0(Reset?)".into() }));
    map
}

/// The Python `"%#x" % v` formatter body (`0x`-prefixed lowercase hex,
/// with the sign before the prefix for negative values).
fn format_version_hash_x(v: i64) -> String {
    if v < 0 {
        format!("-0x{:x}", v.unsigned_abs())
    } else {
        format!("0x{:x}", v as u64)
    }
}

/// `0x100 >> v` (`tmc2130.py:104`): Python ints are unbounded, so this is
/// `0` for `v >= 9`; shifting `i64` by `>= 64` or a negative amount would
/// panic, so cap it. For negative `v` Python raises (`ValueError: negative
/// shift count`); the formatter has no error channel, so it returns `0`.
fn mres_step(v: i64) -> i64 {
    if !(0..64).contains(&v) {
        return 0;
    }
    0x100i64 >> v
}

// ---------------------------------------------------------------------------
// Small shared helpers.
// ---------------------------------------------------------------------------

/// `KeyError(key)` as Python raises it (unhandled -> internal error).
fn key_error(key: &str) -> KlippyError {
    KlippyError::Internal { class_name: "KeyError".into(), message: str_repr(key) }
}

/// `ValueError(message)` as Python raises it (unhandled -> internal error).
fn value_error(msg: &str) -> KlippyError {
    KlippyError::Internal { class_name: "ValueError".into(), message: msg.to_string() }
}

/// An exception the reference does not catch unwinds the reactor, which ends
/// the process with a traceback.
fn unhandled(e: KlippyError) -> ! {
    e.raise()
}

/// A value required by the ConfigApi contract but returned as `None`
/// (unreachable with a compliant ConfigApi; the core raises instead).
fn required<T>(option: &str, value: Option<T>) -> KResult<T> {
    value.ok_or_else(|| KlippyError::Internal {
        class_name: "AssertionError".into(),
        message: format!("required option '{option}' returned None"),
    })
}

/// `dict.get(key)` for `PyValue` dicts (missing -> `None`).
fn dict_get(value: &PyValue, key: &str) -> PyValue {
    match value {
        PyValue::Dict(dict) => {
            dict.get(&PyValue::Str(key.to_string())).cloned().unwrap_or(PyValue::None)
        }
        _ => PyValue::None,
    }
}

/// Saturating conversion of a Python int to `i64` (only used before a clamp).
fn pyint_saturating(n: &PyInt) -> i64 {
    match n.0.to_string().parse::<i64>() {
        Ok(v) => v,
        Err(_) => {
            if n.sign() < 0 {
                i64::MIN
            } else {
                i64::MAX
            }
        }
    }
}

/// `bytearray(v)`: bytes or a list of ints in range(0, 256).
fn bytes_of(value: &PyValue) -> KResult<Vec<u8>> {
    match value {
        PyValue::Bytes(b) => Ok(b.clone()),
        PyValue::List(items) => {
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                match item {
                    PyValue::Int(n) => match n.0.to_string().parse::<i64>() {
                        Ok(v) if (0..=255).contains(&v) => out.push(v as u8),
                        _ => {
                            return Err(value_error("byte must be in range(0, 256)"));
                        }
                    },
                    _ => {
                        return Err(KlippyError::Internal {
                            class_name: "TypeError".into(),
                            message: "'int' object expected".into(),
                        });
                    }
                }
            }
            Ok(out)
        }
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".into(),
            message: "'bytes' or iterable of ints expected".into(),
        }),
    }
}

/// `f64` argument of an event handler (`print_time`); mirrors the Python
/// `TypeError` for the wrong number of event arguments.
fn event_time(handler: &str, args: &[EventArg]) -> KResult<f64> {
    match args {
        [EventArg::Value(PyValue::Float(f))] => Ok(*f),
        [EventArg::Value(v)] => v.to_f64().ok_or_else(|| KlippyError::Internal {
            class_name: "TypeError".into(),
            message: format!(
                "unsupported operand type(s) for -: '{}' and 'float'",
                pycompat::value::type_name(v)
            ),
        }),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".into(),
            message: format!("{handler}() takes exactly 1 argument ({} given)", args.len()),
        }),
    }
}

/// 3-byte SPI frame `[((val >> 16) | reg) & 0xff, (val >> 8) & 0xff, val & 0xff]`.
fn spi_triple(val: i64, reg: i64) -> [u8; 3] {
    let v = val as u64;
    let reg = reg as u64;
    [(((v >> 16) | reg) & 0xff) as u8, ((v >> 8) & 0xff) as u8, (v & 0xff) as u8]
}

// ---------------------------------------------------------------------------
// Current calculation (pure, shared by `Tmc2660CurrentHelper`).
// ---------------------------------------------------------------------------

/// `_calc_current_bits(self, current, vsense)` (tmc2660.py:137-141).
/// Uses C-cast truncation exactly where Python's `int()` does.
pub fn calc_current_bits(current: f64, sense_resistor: f64, vsense: bool) -> KResult<i64> {
    let vref = if vsense { 0.165 } else { 0.310 };
    let x = 32.0 * sense_resistor * current * 2.0f64.sqrt() / vref + 0.5;
    if x.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".into(),
            message: "cannot convert float NaN to integer".into(),
        });
    }
    if x.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".into(),
            message: "cannot convert float infinity to integer".into(),
        });
    }
    // max(0, min(31, int(x) - 1)): the clamps dominate outside [2, 32),
    // and the truncated conversion is exact inside.
    if x >= 32.0 {
        Ok(31)
    } else if x < 2.0 {
        Ok(0)
    } else {
        Ok(kos::cconv::f64_to_i64(x) - 1)
    }
}

/// `_calc_current_from_bits(self, cs, vsense)` (tmc2660.py:143-145).
pub fn calc_current_from_bits(cs: i64, sense_resistor: f64, vsense: bool) -> KResult<f64> {
    let vref = if vsense { 0.165 } else { 0.310 };
    let num = ((cs + 1) as f64) * vref;
    let den = 32.0 * sense_resistor * 2.0f64.sqrt();
    if den == 0.0 {
        return Err(KlippyError::Internal {
            class_name: "ZeroDivisionError".into(),
            message: "float division by zero".into(),
        });
    }
    Ok(num / den)
}

/// `_calc_current(self, run_current)` (tmc2660.py:147-158).
pub fn calc_current(run_current: f64, sense_resistor: f64) -> KResult<(bool, i64)> {
    let mut vsense = true;
    let mut irun = calc_current_bits(run_current, sense_resistor, true)?;
    if irun == 31 {
        let cur = calc_current_from_bits(irun, sense_resistor, true)?;
        if cur < run_current {
            let irun2 = calc_current_bits(run_current, sense_resistor, false)?;
            let cur2 = calc_current_from_bits(irun2, sense_resistor, false)?;
            if (run_current - cur2).abs() < (run_current - cur).abs() {
                vsense = false;
                irun = irun2;
            }
        }
    }
    Ok((vsense, irun))
}

// ---------------------------------------------------------------------------
// TMC2660 SPI shim (`MCU_TMC2660_SPI`).
// ---------------------------------------------------------------------------

/// `MCU_TMC2660_SPI` (tmc2660.py:192).
pub struct McuTmc2660Spi {
    printer: Printer,
    mutex: ReactorMutex,
    spi: Arc<McuSpi>,
    name_to_reg: IndexMap<String, i64>,
    fields: Arc<FieldHelper>,
}

impl McuTmc2660Spi {
    /// `MCU_TMC2660_SPI(config, name_to_reg, fields)` (tmc2660.py:193).
    pub fn new(
        config: &Config,
        name_to_reg: IndexMap<String, i64>,
        fields: Arc<FieldHelper>,
    ) -> KResult<Arc<McuTmc2660Spi>> {
        let printer = config.get_printer();
        let spi = mcu_spi_from_config(config, 0, "cs_pin", 4000000, None, false)?;
        let tmc = McuTmc2660Spi::new_with_spi(printer, spi, name_to_reg, fields);
        Ok(tmc)
    }

    /// Same state as [`McuTmc2660Spi::new`] with the SPI object supplied
    /// directly (instead of `bus.MCU_SPI_from_config(config, 0,
    /// default_speed=4000000)`), e.g. for tests with a fake MCU.
    pub fn new_with_spi(
        printer: Printer,
        spi: Arc<McuSpi>,
        name_to_reg: IndexMap<String, i64>,
        fields: Arc<FieldHelper>,
    ) -> Arc<McuTmc2660Spi> {
        let reactor = printer.get_reactor();
        let mutex = reactor.mutex(false);
        Arc::new(McuTmc2660Spi { printer, mutex, spi, name_to_reg, fields })
    }

    /// `get_fields()` (tmc2660.py:199).
    pub fn get_fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }

    /// `get_register_raw(reg_name)` (tmc2660.py:201): switches `rdsel` to
    /// the read register, re-selects first when changed, reads the 3-byte
    /// response; returns the `{'data': ..., '#receive_time': ...}` dict.
    pub fn get_register_raw(&self, reg_name: &str) -> KResult<PyValue> {
        let new_rdsel = read_registers()
            .iter()
            .position(|r| r == reg_name)
            .ok_or_else(|| value_error(&format!("{} is not in list", str_repr(reg_name))))?;
        let reg = self
            .name_to_reg
            .get("DRVCONF")
            .copied()
            .ok_or_else(|| key_error("DRVCONF"))?;
        if !matches!(dict_get(&self.printer.get_start_args(), "debugoutput"), PyValue::None) {
            return Ok(PyValue::Dict(IndexMap::from([
                (PyValue::Str("data".into()), PyValue::Int(PyInt::from_i64(0))),
                (PyValue::Str("#receive_time".into()), PyValue::Float(0.0)),
            ])));
        }
        self.mutex.lock();
        let result = (|| {
            let old_rdsel = self.fields.get_field("rdsel", None, None)?;
            let val = self.fields.set_field("rdsel", new_rdsel as i64, None, None)?;
            let msg = spi_triple(val, reg);
            if new_rdsel as i64 != old_rdsel {
                // Must set RDSEL value first.
                self.spi.spi_send(&msg, SendOpts { minclock: 0, reqclock: 0 })?;
            }
            let params = self.spi.spi_transfer(
                &msg,
                QuerySendOpts { minclock: 0, reqclock: 0, retry: true },
            )?;
            let pr = bytes_of(&dict_get(&params, "response"))?;
            if pr.len() < 3 {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".into(),
                    message: "index out of range".into(),
                });
            }
            let data = (i64::from(pr[0]) << 16) | (i64::from(pr[1]) << 8) | i64::from(pr[2]);
            Ok::<PyValue, KlippyError>(PyValue::Dict(IndexMap::from([
                (PyValue::Str("data".into()), PyValue::Int(PyInt::from_i64(data))),
                (
                    PyValue::Str("#receive_time".into()),
                    dict_get(&params, "#receive_time"),
                ),
            ])))
        })();
        self.mutex.unlock();
        result
    }

    /// `get_register(reg_name)` (tmc2660.py:222).
    pub fn get_register(&self, reg_name: &str) -> KResult<i64> {
        let raw = self.get_register_raw(reg_name)?;
        match dict_get(&raw, "data") {
            PyValue::Int(n) => Ok(pyint_saturating(&n)),
            _ => Err(KlippyError::Internal {
                class_name: "TypeError".into(),
                message: "get_register data is not an int".into(),
            }),
        }
    }

    /// `set_register(reg_name, val, print_time=None)` (tmc2660.py:224).
    pub fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        let minclock = match print_time {
            Some(t) => self.spi.get_mcu().print_time_to_clock(t),
            None => 0,
        };
        let reg = self
            .name_to_reg
            .get(reg_name)
            .copied()
            .ok_or_else(|| key_error(reg_name))?;
        let msg = spi_triple(val, reg);
        self.mutex.lock();
        let result = self.spi.spi_send(&msg, SendOpts { minclock, reqclock: 0 });
        self.mutex.unlock();
        result
    }

    /// `get_tmc_frequency()` (tmc2660.py:232): the TMC2660 has none.
    pub fn get_tmc_frequency(&self) -> Option<f64> {
        None
    }

    /// `get_mcu()` (tmc2660.py:234).
    pub fn get_mcu(&self) -> Mcu {
        self.spi.get_mcu()
    }

    /// `name_to_reg[reg_name]` (tmc2660.py:207,223).
    pub fn name_to_reg_value(&self, reg_name: &str) -> Option<i64> {
        self.name_to_reg.get(reg_name).copied()
    }
}

impl McuTmcApi for McuTmc2660Spi {
    fn get_fields(&self) -> Arc<FieldHelper> {
        McuTmc2660Spi::get_fields(self)
    }

    fn get_register(&self, reg_name: &str) -> KResult<i64> {
        McuTmc2660Spi::get_register(self, reg_name)
    }

    fn get_register_raw(&self, reg_name: &str) -> KResult<RegisterRaw> {
        let raw = McuTmc2660Spi::get_register_raw(self, reg_name)?;
        let data = match dict_get(&raw, "data") {
            PyValue::Int(n) => pyint_saturating(&n),
            _ => 0,
        };
        let receive_time = dict_get(&raw, "#receive_time").to_f64().unwrap_or(0.0);
        Ok(RegisterRaw { data, receive_time, spi_status: None })
    }

    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        McuTmc2660Spi::set_register(self, reg_name, val, print_time)
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        McuTmc2660Spi::get_tmc_frequency(self)
    }

    fn get_mcu(&self) -> Mcu {
        McuTmc2660Spi::get_mcu(self)
    }

    fn name_to_reg_value(&self, reg_name: &str) -> Option<i64> {
        McuTmc2660Spi::name_to_reg_value(self, reg_name)
    }
}

// ---------------------------------------------------------------------------
// Current helper (`TMC2660CurrentHelper`).
// ---------------------------------------------------------------------------

/// `TMC2660CurrentHelper` (tmc2660.py:115).
pub struct Tmc2660CurrentHelper {
    printer: Printer,
    name: String,
    mcu_tmc: Arc<McuTmc2660Spi>,
    fields: Arc<FieldHelper>,
    current: GCell<f64>,
    sense_resistor: f64,
    idle_current_percentage: i64,
}

impl Tmc2660CurrentHelper {
    /// `TMC2660CurrentHelper(config, mcu_tmc)` (tmc2660.py:116).
    pub fn new(config: &Config, mcu_tmc: Arc<McuTmc2660Spi>) -> KResult<Arc<Tmc2660CurrentHelper>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let name = match name.split_whitespace().last() {
            Some(token) => token.to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".into(),
                    message: "list index out of range".into(),
                });
            }
        };
        let current = required(
            "run_current",
            config.getfloat(
                "run_current",
                GetFloatOpts {
                    default: None,
                    minval: Some(0.1),
                    maxval: Some(MAX_CURRENT),
                    ..Default::default()
                },
            )?,
        )?;
        let sense_resistor = required(
            "sense_resistor",
            config.getfloat("sense_resistor", GetFloatOpts::default())?,
        )?;
        let fields = mcu_tmc.get_fields();
        let (vsense, cs) = calc_current(current, sense_resistor)?;
        fields.set_field("cs", cs, None, None)?;
        fields.set_field("vsense", if vsense { 1 } else { 0 }, None, None)?;
        let idle_current_percentage = required(
            "idle_current_percent",
            config.getint(
                "idle_current_percent",
                GetIntOpts {
                    default: Some(Some(100)),
                    minval: Some(0),
                    maxval: Some(100),
                    ..Default::default()
                },
            )?,
        )?;
        let this = Arc::new(Tmc2660CurrentHelper {
            printer,
            name,
            mcu_tmc,
            fields,
            current: GCell::new(current),
            sense_resistor,
            idle_current_percentage,
        });
        if idle_current_percentage < 100 {
            let printing = this.clone();
            this.printer.register_event_handler(
                EV_IDLE_TIMEOUT_PRINTING,
                Box::new(move |args: &[EventArg]| {
                    let print_time = event_time("_handle_printing", args)?;
                    printing.handle_printing(print_time);
                    Ok(PyValue::None)
                }),
            );
            let ready = this.clone();
            this.printer.register_event_handler(
                EV_IDLE_TIMEOUT_READY,
                Box::new(move |args: &[EventArg]| {
                    let print_time = event_time("_handle_ready", args)?;
                    ready.handle_ready(print_time);
                    Ok(PyValue::None)
                }),
            );
        }
        Ok(this)
    }

    /// `_calc_current_bits` (tmc2660.py:137): see [`calc_current_bits`].
    pub fn calc_current_bits(&self, current: f64, vsense: bool) -> KResult<i64> {
        calc_current_bits(current, self.sense_resistor, vsense)
    }

    /// `_calc_current_from_bits` (tmc2660.py:143): see
    /// [`calc_current_from_bits`].
    pub fn calc_current_from_bits(&self, cs: i64, vsense: bool) -> KResult<f64> {
        calc_current_from_bits(cs, self.sense_resistor, vsense)
    }

    /// `_calc_current` (tmc2660.py:147): see [`calc_current`].
    pub fn calc_current(&self, run_current: f64) -> KResult<(bool, i64)> {
        calc_current(run_current, self.sense_resistor)
    }

    /// `_handle_printing(print_time)` (tmc2660.py:160).
    pub fn handle_printing(self: &Arc<Self>, print_time: f64) {
        let print_time = print_time - 0.1; // Schedule slightly before deadline.
        let this = self.clone();
        let _ = this.printer.get_reactor().register_callback(
            move |_ev: f64| {
                if let Err(e) = this.update_current(this.current.get(), print_time) {
                    unhandled(e);
                }
            },
            NOW,
        );
    }

    /// `_handle_ready(print_time)` (tmc2660.py:165).
    pub fn handle_ready(self: &Arc<Self>, print_time: f64) {
        let current = self.current.get() * (self.idle_current_percentage as f64) / 100.0;
        let this = self.clone();
        let _ = this.printer.get_reactor().register_callback(
            move |_ev: f64| {
                if let Err(e) = this.update_current(current, print_time) {
                    unhandled(e);
                }
            },
            NOW,
        );
    }

    /// `_update_current(current, print_time)` (tmc2660.py:170).
    pub fn update_current(&self, current: f64, print_time: f64) -> KResult<()> {
        let (vsense, cs) = self.calc_current(current)?;
        let val = self.fields.set_field("cs", cs, None, None)?;
        self.mcu_tmc.set_register("SGCSCONF", val, Some(print_time))?;
        // Only update VSENSE if we need to
        if i64::from(vsense) != self.fields.get_field("vsense", None, None)? {
            let val = self.fields.set_field("vsense", i64::from(vsense), None, None)?;
            self.mcu_tmc.set_register("DRVCONF", val, Some(print_time))?;
        }
        Ok(())
    }

    /// `get_current()` (tmc2660.py:179).
    pub fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        (self.current.get(), None, None, MAX_CURRENT)
    }

    /// `set_current(run_current, hold_current, print_time)` (tmc2660.py:182);
    /// `hold_current` is unused, like in Python.
    pub fn set_current(&self, run_current: f64, _hold_current: f64, print_time: f64) -> KResult<()> {
        self.current.set(run_current);
        self.update_current(run_current, print_time)
    }

    /// The `config` section name's last token (`self.name`, tmc2660.py:118).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the current `run_current`.
    pub fn current(&self) -> f64 {
        self.current.get()
    }
}

impl TmcCurrentHelperApi for Tmc2660CurrentHelper {
    fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        Tmc2660CurrentHelper::get_current(self)
    }

    fn set_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()> {
        // `hold_current` is unused by the Python original (tmc2660.py:182).
        let hold = hold_current.unwrap_or(0.0);
        Tmc2660CurrentHelper::set_current(self, run_current, hold, print_time)
    }
}

// ---------------------------------------------------------------------------
// Printer object (`TMC2660`).
// ---------------------------------------------------------------------------

/// `TMC2660` (tmc2660.py:242): the `[tmc2660 <stepper>]` printer object.
pub struct Tmc2660 {
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<McuTmc2660Spi>,
    current_helper: Arc<Tmc2660CurrentHelper>,
    cmd_helper: Arc<TMCCommandHelper>,
}

impl Tmc2660 {
    /// `TMC2660(config)` (tmc2660.py:243).
    pub fn new(config: &Config) -> KResult<Tmc2660> {
        let fields = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
        fields.set_field("sdoff", 0, None, None)?; // Access DRVCTRL in step/dir mode.
        let mcu_tmc = McuTmc2660Spi::new(config, registers(), fields.clone())?;
        let current_helper = Tmc2660CurrentHelper::new(config, mcu_tmc.clone())?;
        let cmd_helper = TMCCommandHelper::new(
            config,
            mcu_tmc.clone(),
            current_helper.clone(),
        )?;
        cmd_helper.setup_register_dump(read_registers(), None)?;
        Tmc2660::setup_driver_config(config, &fields)?;
        Ok(Tmc2660 { fields, mcu_tmc, current_helper, cmd_helper })
    }

    /// The driver-field block of `TMC2660.__init__` (tmc2660.py:255-283):
    /// CHOPCONF, SMARTEN, SGSCONF, DRVCONF defaults (with the
    /// `driver_HEND + driver_HSTRT must be <= 15` check).
    pub fn setup_driver_config(config: &Config, fields: &Arc<FieldHelper>) -> KResult<()> {
        // CHOPCONF
        fields.set_config_field(config, "tbl", ConfigFieldDefault::Int(2))?;
        fields.set_config_field(config, "rndtf", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "hdec", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "chm", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "hend", ConfigFieldDefault::Int(3))?;
        fields.set_config_field(config, "hstrt", ConfigFieldDefault::Int(3))?;
        fields.set_config_field(config, "toff", ConfigFieldDefault::Int(4))?;
        if fields.get_field("chm", None, None)? == 0
            && fields.get_field("hstrt", None, None)? + fields.get_field("hend", None, None)? > 15
        {
            return Err(KlippyError::Config(
                "driver_HEND + driver_HSTRT must be <= 15".to_string(),
            ));
        }
        // SMARTEN
        fields.set_config_field(config, "seimin", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "sedn", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "semax", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "seup", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "semin", ConfigFieldDefault::Int(0))?;
        // SGSCONF
        fields.set_config_field(config, "sfilt", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "sgt", ConfigFieldDefault::Int(0))?;
        // DRVCONF
        fields.set_config_field(config, "slph", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "slpl", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "diss2g", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "ts2g", ConfigFieldDefault::Int(3))?;
        Ok(())
    }

    /// `self.get_phase_offset = cmdhelper.get_phase_offset` (tmc2660.py:252).
    /// `mcu_phase_offset` is an int (or `None`), so `(Option<i64>, i64)`.
    pub fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmd_helper.get_phase_offset()
    }

    /// `self.get_status = cmdhelper.get_status` (tmc2660.py:253).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.cmd_helper.get_status(Some(eventtime))
    }

    /// The shared field helper.
    pub fn fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }

    /// The shared MCU shim.
    pub fn mcu_tmc(&self) -> Arc<McuTmc2660Spi> {
        self.mcu_tmc.clone()
    }

    /// The shared current helper.
    pub fn current_helper(&self) -> Arc<Tmc2660CurrentHelper> {
        self.current_helper.clone()
    }
}

impl PrinterObject for Tmc2660 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_status(eventtime)))
    }
}

/// `self.get_phase_offset = cmdhelper.get_phase_offset` (tmc2660.py:252);
/// `endstop_phase` and `angle` look the driver up ("tmc2660 <stepper>") and
/// call it, so the printer object declares the trait.
impl TMCCommandHelperApi for Tmc2660 {
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmd_helper.get_phase_offset()
    }
}

/// `load_config_prefix(config)` (tmc2660.py:285).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(Tmc2660::new(config)?) as ObjectRef))
}

/// Module entry point: same entry points the Python module has
/// (`tmc2660.py:285`; `load_config_prefix` only).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config_prefix: Some(load_config_prefix), ..Default::default() }
}
