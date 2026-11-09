//! Port of `klippy/extras/tmc.py`: the code shared by the TMC stepper
//! driver modules (`tmc2130`, `tmc2208`, `tmc2209`, `tmc2240`, `tmc2660`,
//! `tmc5160`).
//!
//! The module has no `load_config`: the drivers build [`FieldHelper`],
//! [`TMCCommandHelper`], [`TMCVirtualPinHelper`] and call the
//! configuration helpers. The Python duck types the helpers talk to are the
//! traits [`McuTmcApi`] (`MCU_TMC_SPI`, `MCU_TMC_uart`, `MCU_TMC2660_SPI`) and
//! [`TmcCurrentHelperApi`] (`TMCCurrentHelper`, `TMC2240CurrentHelper`, ...).

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts as ConfigGetIntOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GetFloatOpts as GcmdGetFloatOpts, GetIntOpts as GcmdGetIntOpts, GetOpts,
};
use klippy_api::mcu::{Chip, ChipApi, Endstop, Mcu, Pin, PinParams, PinsApi};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_HOMING_MOVE_BEGIN, EV_HOMING_MOVE_END,
    EV_KLIPPY_CONNECT, EV_KLIPPY_MCU_IDENTIFY, EV_STEPPER_SET_DIR_INVERTED,
    EV_STEPPER_SYNC_MCU_POSITION,
};
use klippy_api::stepper::{McuStepperApi, Stepper};
use klippy_api::toolhead::{HomingMoveApi, ToolHeadApi};
use klippy_api::{load_interface, log, lookup_interface, EventArg, GCell};
use pycompat::text::{float_true_div, percent_format, round_py_nd, str_repr, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{ReactorMutex, Timer, NEVER, NOW};
use std::sync::{Arc, Weak};

use crate::bulk_sensor::{BatchBulkHelper, BATCH_INTERVAL};
use crate::force_move::ForceMoveApi;
use crate::heaters::PrinterHeatersApi;
use crate::stepper_enable::PrinterStepperEnableApi;

// ---------------------------------------------------------------------------
// Python helpers
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

/// `KeyError(key)`: `str(e)` is the key's repr.
fn key_error(key: &str) -> KlippyError {
    internal("KeyError", str_repr(key))
}

fn none_attr_error(attr: &str) -> KlippyError {
    internal("AttributeError", format!("'NoneType' object has no attribute '{}'", attr))
}

fn is_command_error(e: &KlippyError) -> bool {
    matches!(e, KlippyError::Command(_))
}

/// An exception the reference does not catch unwinds the reactor, which ends
/// the process with a traceback.
fn unhandled(e: KlippyError) -> ! {
    panic!("{}: {}", e.class_name(), e.message())
}

fn empty_dict() -> PyValue {
    PyValue::Dict(IndexMap::new())
}

fn int_value(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn opt_int_value(v: Option<i64>) -> PyValue {
    match v {
        Some(v) => int_value(v),
        None => PyValue::None,
    }
}

fn opt_float_value(v: Option<f64>) -> PyValue {
    match v {
        Some(v) => PyValue::Float(v),
        None => PyValue::None,
    }
}

fn str_key(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn status_dict(pairs: Vec<(&str, PyValue)>) -> PyValue {
    let mut d = IndexMap::new();
    for (k, v) in pairs {
        d.insert(str_key(k), v);
    }
    PyValue::Dict(d)
}

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_default()
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

/// Python `int(x)` of a float.
fn py_int_of_float(f: f64) -> KResult<PyInt> {
    if f.is_nan() {
        return Err(internal("ValueError", "cannot convert float NaN to integer".to_string()));
    }
    if f.is_infinite() {
        return Err(internal(
            "OverflowError",
            "cannot convert float infinity to integer".to_string(),
        ));
    }
    Ok(trunc_float_to_bigint(f))
}

fn true_div(a: f64, b: f64) -> KResult<f64> {
    float_true_div(a, b).map_err(|e| internal(e.kind, e.msg))
}

fn value_as_i64(v: &PyValue) -> i64 {
    match v {
        PyValue::Int(n) => pyint_saturating(n),
        PyValue::Bool(b) => i64::from(*b),
        _ => 0,
    }
}

fn lookup_required<T>(printer: &Printer, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    Ok(lookup_interface::<T>(&**printer, name, LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

fn load_required<T>(printer: &Printer, config: &Config, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    Ok(load_interface::<T>(&**printer, config, name, LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

/// `" ".join(config.get_name().split()[1:])`
fn stepper_name_of(config: &Config) -> String {
    let name = config.get_name();
    name.split_whitespace().skip(1).collect::<Vec<_>>().join(" ")
}

fn with_mutex<R>(mutex: &ReactorMutex, f: impl FnOnce() -> R) -> R {
    mutex.lock();
    let r = f();
    mutex.unlock();
    r
}

fn step_event_stepper(params: &[EventArg]) -> KResult<Stepper> {
    match params.first().and_then(|p| p.as_handle::<dyn McuStepperApi>()) {
        Some(s) => Ok(s),
        None => Err(internal(
            "TypeError",
            "stepper event handler requires a stepper argument".to_string(),
        )),
    }
}

// ---------------------------------------------------------------------------
// Field helpers (`tmc.py:11-82`)
// ---------------------------------------------------------------------------

/// `ffs(mask)`: position of the first bit set in a mask (`tmc.py:16`).
pub fn ffs(mask: i64) -> i64 {
    i64::from(bit_length(mask & mask.wrapping_neg())) - 1
}

/// `int.bit_length()`
fn bit_length(v: i64) -> u32 {
    64 - v.unsigned_abs().leading_zeros()
}

fn shift_amount(mask: i64) -> KResult<u32> {
    let f = ffs(mask);
    if f < 0 {
        return Err(internal("ValueError", "negative shift count".to_string()));
    }
    Ok(f as u32)
}

/// A per-field value formatter (`field_formatters[name]`).
pub type FieldFormatter = Arc<dyn Fn(i64) -> String + Send + Sync>;

/// The default value given to [`FieldHelper::set_config_field`]: Python passes
/// `True`/`False` for single bit fields and ints otherwise.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfigFieldDefault {
    Bool(bool),
    Int(i64),
}

impl ConfigFieldDefault {
    fn as_int(self) -> i64 {
        match self {
            ConfigFieldDefault::Bool(b) => i64::from(b),
            ConfigFieldDefault::Int(i) => i,
        }
    }
}

/// `FieldHelper` (`tmc.py:19`).
pub struct FieldHelper {
    /// `all_fields`: register name to field name to mask.
    pub all_fields: IndexMap<String, IndexMap<String, i64>>,
    signed_fields: Vec<String>,
    field_formatters: IndexMap<String, FieldFormatter>,
    registers: GCell<IndexMap<String, i64>>,
    field_to_register: IndexMap<String, String>,
}

impl FieldHelper {
    /// `FieldHelper(all_fields, signed_fields, field_formatters, registers=None)`
    pub fn new(
        all_fields: IndexMap<String, IndexMap<String, i64>>,
        signed_fields: Vec<String>,
        field_formatters: IndexMap<String, FieldFormatter>,
        registers: Option<IndexMap<String, i64>>,
    ) -> Arc<FieldHelper> {
        let mut field_to_register = IndexMap::new();
        for (reg, fields) in all_fields.iter() {
            for f in fields.keys() {
                field_to_register.insert(f.clone(), reg.clone());
            }
        }
        Arc::new(FieldHelper {
            all_fields,
            signed_fields,
            field_formatters,
            registers: GCell::new(registers.unwrap_or_default()),
            field_to_register,
        })
    }

    /// Snapshot of `self.registers` (insertion ordered).
    pub fn registers(&self) -> IndexMap<String, i64> {
        self.registers.cloned()
    }

    /// `self.registers.get(reg_name)`
    pub fn get_register_value(&self, reg_name: &str) -> Option<i64> {
        self.registers.borrow().get(reg_name).copied()
    }

    /// `lookup_register(field_name, default=None)`
    pub fn lookup_register(&self, field_name: &str) -> Option<String> {
        self.field_to_register.get(field_name).cloned()
    }

    fn lookup_mask(&self, reg_name: &str, field_name: &str) -> KResult<i64> {
        let fields = self.all_fields.get(reg_name).ok_or_else(|| key_error(reg_name))?;
        fields.get(field_name).copied().ok_or_else(|| key_error(field_name))
    }

    /// `get_field(field_name, reg_value=None, reg_name=None)`
    pub fn get_field(
        &self,
        field_name: &str,
        reg_value: Option<i64>,
        reg_name: Option<&str>,
    ) -> KResult<i64> {
        let reg_name = match reg_name {
            Some(r) => r.to_string(),
            None => self
                .field_to_register
                .get(field_name)
                .cloned()
                .ok_or_else(|| key_error(field_name))?,
        };
        let reg_value = match reg_value {
            Some(v) => v,
            None => self.registers.borrow().get(&reg_name).copied().unwrap_or(0),
        };
        let mask = self.lookup_mask(&reg_name, field_name)?;
        let shift = shift_amount(mask)?;
        let mut field_value = (reg_value & mask) >> shift;
        if self.signed_fields.iter().any(|f| f == field_name)
            && ((reg_value & mask).wrapping_shl(1)) > mask
        {
            field_value -= 1i64.wrapping_shl(bit_length(field_value));
        }
        Ok(field_value)
    }

    /// `set_field(field_name, field_value, reg_value=None, reg_name=None)`:
    /// returns the register value with the field bits filled.
    pub fn set_field(
        &self,
        field_name: &str,
        field_value: i64,
        reg_value: Option<i64>,
        reg_name: Option<&str>,
    ) -> KResult<i64> {
        let reg_name = match reg_name {
            Some(r) => r.to_string(),
            None => self
                .field_to_register
                .get(field_name)
                .cloned()
                .ok_or_else(|| key_error(field_name))?,
        };
        let reg_value = match reg_value {
            Some(v) => v,
            None => self.registers.borrow().get(&reg_name).copied().unwrap_or(0),
        };
        let mask = self.lookup_mask(&reg_name, field_name)?;
        let shift = shift_amount(mask)?;
        let new_value = (reg_value & !mask) | ((field_value.wrapping_shl(shift)) & mask);
        self.registers.borrow_mut().insert(reg_name, new_value);
        Ok(new_value)
    }

    /// `set_config_field(config, field_name, default)`
    pub fn set_config_field(
        &self,
        config: &Config,
        field_name: &str,
        default: ConfigFieldDefault,
    ) -> KResult<i64> {
        let config_name = format!("driver_{}", field_name.to_uppercase());
        let reg_name = self
            .field_to_register
            .get(field_name)
            .cloned()
            .ok_or_else(|| key_error(field_name))?;
        let mask = self.lookup_mask(&reg_name, field_name)?;
        let maxval = mask >> shift_amount(mask)?;
        let val = if maxval == 1 {
            let v = config.getboolean(
                &config_name,
                GetBoolOpts { default: Some(Some(default.as_int() != 0)), note_valid: true },
            )?;
            i64::from(v.unwrap_or(default.as_int() != 0))
        } else if self.signed_fields.iter().any(|f| f == field_name) {
            config
                .getint(
                    &config_name,
                    ConfigGetIntOpts {
                        default: Some(Some(default.as_int())),
                        minval: Some(-(maxval / 2 + 1)),
                        maxval: Some(maxval / 2),
                        note_valid: true,
                    },
                )?
                .unwrap_or(default.as_int())
        } else {
            config
                .getint(
                    &config_name,
                    ConfigGetIntOpts {
                        default: Some(Some(default.as_int())),
                        minval: Some(0),
                        maxval: Some(maxval),
                        note_valid: true,
                    },
                )?
                .unwrap_or(default.as_int())
        };
        self.set_field(field_name, val, None, None)
    }

    /// `pretty_format(reg_name, reg_value)`
    pub fn pretty_format(&self, reg_name: &str, reg_value: i64) -> String {
        let mut reg_fields: Vec<(i64, &String)> = match self.all_fields.get(reg_name) {
            Some(m) => m.iter().map(|(n, mask)| (*mask, n)).collect(),
            None => Vec::new(),
        };
        reg_fields.sort();
        let mut fields = Vec::new();
        for (_mask, field_name) in reg_fields {
            let field_value = self
                .get_field(field_name, Some(reg_value), Some(reg_name))
                .unwrap_or_else(|e| unhandled(e));
            let sval = match self.field_formatters.get(field_name) {
                Some(f) => f(field_value),
                None => field_value.to_string(),
            };
            if !sval.is_empty() && sval != "0" {
                fields.push(format!(" {}={}", field_name, sval));
            }
        }
        pct(
            "%-11s %08x%s",
            &[
                PctValue::Str(format!("{}:", reg_name)),
                PctValue::Int(PyInt::from_i64(reg_value)),
                PctValue::Str(fields.join("")),
            ],
        )
    }

    /// `get_reg_fields(reg_name, reg_value)`
    pub fn get_reg_fields(&self, reg_name: &str, reg_value: i64) -> IndexMap<String, i64> {
        let mut out = IndexMap::new();
        if let Some(fields) = self.all_fields.get(reg_name) {
            for field_name in fields.keys() {
                let v = self
                    .get_field(field_name, Some(reg_value), Some(reg_name))
                    .unwrap_or_else(|e| unhandled(e));
                out.insert(field_name.clone(), v);
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Contracts of the duck types the helpers use
// ---------------------------------------------------------------------------

/// The dict returned by `mcu_tmc.get_register_raw(reg_name)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegisterRaw {
    /// `"data"`
    pub data: i64,
    /// `"#receive_time"`
    pub receive_time: f64,
    /// `"spi_status"`: present only on SPI drivers.
    pub spi_status: Option<i64>,
}

/// The `mcu_tmc` object (`MCU_TMC_SPI`, `MCU_TMC_uart`, `MCU_TMC2660_SPI`).
pub trait McuTmcApi: Send + Sync + 'static {
    /// `get_fields()`
    fn get_fields(&self) -> Arc<FieldHelper>;
    /// `get_register(reg_name)`
    fn get_register(&self, reg_name: &str) -> KResult<i64>;
    /// `get_register_raw(reg_name)`
    fn get_register_raw(&self, reg_name: &str) -> KResult<RegisterRaw>;
    /// `set_register(reg_name, val, print_time=None)`
    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()>;
    /// `get_tmc_frequency()`
    fn get_tmc_frequency(&self) -> Option<f64>;
    /// `get_mcu()`
    fn get_mcu(&self) -> Mcu;
    /// `name_to_reg.get(reg_name)`
    fn name_to_reg_value(&self, reg_name: &str) -> Option<i64>;
}

/// The `current_helper` object (`TMCCurrentHelper`, `TMC2240CurrentHelper`,
/// `TMC5160CurrentHelper`, `TMC2660CurrentHelper`).
pub trait TmcCurrentHelperApi: Send + Sync + 'static {
    /// `get_current()`: `(run_current, hold_current, req_hold_current, max_current)`.
    fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64);
    /// `set_current(run_current, hold_current, print_time)`
    fn set_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()>;
}

/// `get_phase_offset` of the driver objects, copied from `TMCCommandHelper`
/// (`tmc.py:414`); `angle` and `endstop_phase` call it.
pub trait TMCCommandHelperApi: Send + Sync + 'static {
    /// `get_phase_offset()`: `(mcu_phase_offset, phases)`.
    fn get_phase_offset(&self) -> (Option<i64>, i64);
}

/// `read_translate(reg_name, val)` of `setup_register_dump`.
pub type ReadTranslate = Arc<dyn Fn(&str, i64) -> (String, i64) + Send + Sync>;

// ---------------------------------------------------------------------------
// Periodic error checking (`tmc.py:85-222`)
// ---------------------------------------------------------------------------

/// A `reg_info` list of `TMCErrorCheck`.
#[derive(Clone, Debug)]
pub struct RegInfo {
    pub last_value: i64,
    pub reg_name: String,
    pub mask: i64,
    pub err_mask: i64,
    pub cs_actual_mask: i64,
}

/// `TMCErrorCheck` (`tmc.py:89`).
pub struct TMCErrorCheck {
    printer: Printer,
    stepper_name: String,
    mcu_tmc: Arc<dyn McuTmcApi>,
    fields: Arc<FieldHelper>,
    check_timer: GCell<Option<Timer>>,
    last_drv_status: GCell<Option<i64>>,
    last_drv_fields: GCell<Option<IndexMap<String, i64>>>,
    /// `gstat_reg_info` (`None` when the driver has no `drv_err` field).
    pub gstat_reg_info: Option<GCell<RegInfo>>,
    clear_gstat: bool,
    irun_field: String,
    /// `drv_status_reg_info`
    pub drv_status_reg_info: GCell<RegInfo>,
    adc_temp: GCell<Option<i64>>,
    adc_temp_reg: Option<String>,
    weak: Weak<TMCErrorCheck>,
}

impl TMCErrorCheck {
    /// `TMCErrorCheck(config, mcu_tmc)`
    pub fn new(config: &Config, mcu_tmc: Arc<dyn McuTmcApi>) -> KResult<Arc<TMCErrorCheck>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let name_parts: Vec<&str> = name.split_whitespace().collect();
        let stepper_name = name_parts.iter().skip(1).copied().collect::<Vec<_>>().join(" ");
        let fields = mcu_tmc.get_fields();
        // Setup for GSTAT query
        let gstat_reg_info = fields.lookup_register("drv_err").map(|reg_name| {
            GCell::new(RegInfo {
                last_value: 0,
                reg_name,
                mask: 0xffffffff,
                err_mask: 0xffffffff,
                cs_actual_mask: 0,
            })
        });
        let mut clear_gstat = true;
        // Setup for DRV_STATUS query
        let mut irun_field = "irun".to_string();
        let mut reg_name = "DRV_STATUS".to_string();
        let mut mask = 0i64;
        let mut err_mask = 0i64;
        let mut cs_actual_mask = 0i64;
        match name_parts.first().copied() {
            Some("tmc2130") => {
                // TMC2130 driver quirks
                clear_gstat = false;
                cs_actual_mask = fields.lookup_mask(&reg_name, "cs_actual")?;
            }
            Some("tmc2660") => {
                // TMC2660 driver quirks
                irun_field = "cs".to_string();
                reg_name = "READRSP@RDSEL2".to_string();
                cs_actual_mask = fields.lookup_mask(&reg_name, "se")?;
            }
            _ => {}
        }
        let err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"];
        let warn_fields = ["otpw", "t120", "t143", "t150", "t157"];
        for f in err_fields.iter().chain(warn_fields.iter()) {
            let reg_fields = fields.all_fields.get(&reg_name).ok_or_else(|| key_error(&reg_name))?;
            if let Some(m) = reg_fields.get(*f) {
                mask |= *m;
                if err_fields.contains(f) {
                    err_mask |= *m;
                }
            }
        }
        let drv_status_reg_info = GCell::new(RegInfo {
            last_value: 0,
            reg_name,
            mask,
            err_mask,
            cs_actual_mask,
        });
        // Setup for temperature query
        let adc_temp_reg = fields.lookup_register("adc_temp");
        if adc_temp_reg.is_some() {
            let pheaters = load_required::<dyn PrinterHeatersApi>(&printer, config, "heaters")?;
            pheaters.register_monitor(config);
        }
        Ok(Arc::new_cyclic(|weak| TMCErrorCheck {
            printer,
            stepper_name,
            mcu_tmc,
            fields,
            check_timer: GCell::new(None),
            last_drv_status: GCell::new(None),
            last_drv_fields: GCell::new(None),
            gstat_reg_info,
            clear_gstat,
            irun_field,
            drv_status_reg_info,
            adc_temp: GCell::new(None),
            adc_temp_reg,
            weak: weak.clone(),
        }))
    }

    /// `_query_register(reg_info, try_clear=False)`: returns the cleared flags.
    pub fn _query_register(&self, reg_info: &GCell<RegInfo>, try_clear: bool) -> KResult<i64> {
        let RegInfo { mut last_value, reg_name, mask, err_mask, cs_actual_mask } =
            reg_info.cloned();
        let mut try_clear = try_clear;
        let mut cleared_flags = 0i64;
        let mut count = 0;
        loop {
            let val = match self.mcu_tmc.get_register(&reg_name) {
                Ok(v) => v,
                Err(e) if is_command_error(&e) => {
                    count += 1;
                    if count < 3 && e.message().starts_with("Unable to read tmc uart") {
                        // Allow more retries on a TMC UART read error
                        let reactor = self.printer.get_reactor();
                        reactor.pause(reactor.monotonic() + 0.050);
                        continue;
                    }
                    return Err(e);
                }
                Err(e) => return Err(e),
            };
            if val & mask != last_value & mask {
                let fmt = self.fields.pretty_format(&reg_name, val);
                log::info(&format!("TMC '{}' reports {}", self.stepper_name, fmt));
            }
            last_value = val;
            reg_info.borrow_mut().last_value = val;
            let mut done = false;
            if val & err_mask == 0 {
                if cs_actual_mask == 0 || val & cs_actual_mask != 0 {
                    done = true;
                } else {
                    // CS_ACTUAL field of zero - indicates a driver reset, unless
                    // checks are not running, the current is low or hold is off.
                    let irun = self.fields.get_field(&self.irun_field, None, None)?;
                    done = self.check_timer.borrow().is_none()
                        || irun < 4
                        || (self.irun_field == "irun"
                            && self.fields.get_field("ihold", None, None)? == 0);
                }
            }
            if done {
                break;
            }
            count += 1;
            if count >= 3 {
                let fmt = self.fields.pretty_format(&reg_name, val);
                return Err(KlippyError::command(format!(
                    "TMC '{}' reports error: {}",
                    self.stepper_name, fmt
                )));
            }
            if try_clear && val & err_mask != 0 {
                try_clear = false;
                cleared_flags |= val & err_mask;
                self.mcu_tmc.set_register(&reg_name, val & err_mask, None)?;
            }
        }
        let _ = last_value;
        Ok(cleared_flags)
    }

    /// `_query_temperature()`
    pub fn _query_temperature(&self) -> KResult<()> {
        let reg = self.adc_temp_reg.clone().ok_or_else(|| key_error("None"))?;
        match self.mcu_tmc.get_register(&reg) {
            Ok(v) => {
                self.adc_temp.set(Some(v));
                Ok(())
            }
            Err(e) if is_command_error(&e) => {
                // Ignore comms error for temperature
                self.adc_temp.set(None);
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// `_do_periodic_check(eventtime)`: the check timer callback.
    pub fn _do_periodic_check(&self, eventtime: f64) -> f64 {
        let outcome = (|| -> KResult<()> {
            self._query_register(&self.drv_status_reg_info, false)?;
            if let Some(gstat) = &self.gstat_reg_info {
                self._query_register(gstat, false)?;
            }
            if self.adc_temp_reg.is_some() {
                self._query_temperature()?;
            }
            Ok(())
        })();
        match outcome {
            Ok(()) => eventtime + 1.,
            Err(e) if is_command_error(&e) => {
                self.printer.invoke_shutdown(e.message(), empty_dict());
                NEVER
            }
            Err(e) => unhandled(e),
        }
    }

    /// `stop_checks()`
    pub fn stop_checks(&self) {
        let timer = self.check_timer.borrow_mut().take();
        if let Some(timer) = timer {
            self.printer.get_reactor().unregister_timer(&timer);
        }
    }

    /// `start_checks()`: true when the driver reports that it was reset.
    pub fn start_checks(&self) -> KResult<bool> {
        if self.check_timer.borrow().is_some() {
            self.stop_checks();
        }
        let mut cleared_flags = 0i64;
        self._query_register(&self.drv_status_reg_info, false)?;
        if let Some(gstat) = &self.gstat_reg_info {
            cleared_flags = self._query_register(gstat, self.clear_gstat)?;
        }
        let reactor = self.printer.get_reactor();
        let curtime = reactor.monotonic();
        let this = self.weak.upgrade().expect("TMCErrorCheck built by new()");
        let timer = reactor.register_timer(move |eventtime| this._do_periodic_check(eventtime), curtime + 1.);
        *self.check_timer.borrow_mut() = Some(timer);
        if cleared_flags != 0 {
            let reset_mask = self.fields.lookup_mask("GSTAT", "reset")?;
            if cleared_flags & reset_mask != 0 {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `get_status(eventtime=None)`
    pub fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        if self.check_timer.borrow().is_none() {
            return status_dict(vec![("drv_status", PyValue::None), ("temperature", PyValue::None)]);
        }
        let temp = match self.adc_temp.get() {
            Some(adc) => PyValue::Float(round_py_nd((adc - 2038) as f64 / 7.7, 2)),
            None => PyValue::None,
        };
        let (last_value, reg_name) = {
            let info = self.drv_status_reg_info.borrow();
            (info.last_value, info.reg_name.clone())
        };
        if Some(last_value) != self.last_drv_status.get() {
            self.last_drv_status.set(Some(last_value));
            let fields = self.fields.get_reg_fields(&reg_name, last_value);
            let kept: IndexMap<String, i64> = fields.into_iter().filter(|(_, v)| *v != 0).collect();
            *self.last_drv_fields.borrow_mut() = Some(kept);
        }
        let drv = match self.last_drv_fields.borrow().as_ref() {
            Some(m) => PyValue::Dict(m.iter().map(|(k, v)| (str_key(k), int_value(*v))).collect()),
            None => PyValue::None,
        };
        status_dict(vec![("drv_status", drv), ("temperature", temp)])
    }
}



// ---------------------------------------------------------------------------
// Record driver status (`tmc.py:224-312`)
// ---------------------------------------------------------------------------

/// `TMCStallguardDump` (`tmc.py:228`).
pub struct TMCStallguardDump {
    printer: Printer,
    mcu_tmc: Arc<dyn McuTmcApi>,
    mcu: Mcu,
    fields: Arc<FieldHelper>,
    sg4_reg_name: Option<String>,
    optimized_spi: GCell<bool>,
    samples: GCell<Vec<(f64, i64, i64)>>,
    query_timer: GCell<Option<Timer>>,
    error: GCell<Option<KlippyError>>,
    batch_bulk: GCell<Option<BatchBulkHelper>>,
    weak: Weak<TMCStallguardDump>,
}

impl TMCStallguardDump {
    /// `TMCStallguardDump(config, mcu_tmc)`
    pub fn new(config: &Config, mcu_tmc: Arc<dyn McuTmcApi>) -> KResult<Arc<TMCStallguardDump>> {
        let printer = config.get_printer();
        let stepper_name = stepper_name_of(config);
        let mcu = mcu_tmc.get_mcu();
        let fields = mcu_tmc.get_fields();
        let mut sg4_reg_name = None;
        let has_drv_status =
            fields.all_fields.get("DRV_STATUS").map(|m| !m.is_empty()).unwrap_or(false);
        if has_drv_status {
            // New drivers have separate register for SG4 result
            if mcu_tmc.name_to_reg_value("SG_RESULT").unwrap_or(0) != 0 {
                sg4_reg_name = Some("SG_RESULT".to_string());
            }
            // 2240 supports both SG2 & SG4
            if sg4_reg_name.is_none() && mcu_tmc.name_to_reg_value("SG4_RESULT").unwrap_or(0) != 0 {
                sg4_reg_name = Some("SG4_RESULT".to_string());
            }
        }
        let this = Arc::new_cyclic(|weak| TMCStallguardDump {
            printer: printer.clone(),
            mcu_tmc,
            mcu,
            fields,
            sg4_reg_name,
            optimized_spi: GCell::new(false),
            samples: GCell::new(Vec::new()),
            query_timer: GCell::new(None),
            error: GCell::new(None),
            batch_bulk: GCell::new(None),
            weak: weak.clone(),
        });
        // It is possible to support TMC2660, just disable it for now
        if !has_drv_status {
            return Ok(this);
        }
        let dump = this.clone();
        let start = this.clone();
        let stop = this.clone();
        let batch_bulk = BatchBulkHelper::new(
            printer,
            Box::new(move |eventtime| dump._dump(eventtime)),
            Some(Box::new(move || start._start())),
            Some(Box::new(move || stop._stop())),
            BATCH_INTERVAL,
        );
        let header = PyValue::Tuple(vec![str_key("time"), str_key("sg_result"), str_key("cs_actual")]);
        let api_resp = status_dict(vec![("header", header)]);
        batch_bulk.add_mux_endpoint("tmc/stallguard_dump", "name", &stepper_name, api_resp)?;
        *this.batch_bulk.borrow_mut() = Some(batch_bulk);
        Ok(this)
    }

    /// `_start()`
    pub fn _start(&self) -> KResult<()> {
        *self.error.borrow_mut() = None;
        let status = self.mcu_tmc.get_register_raw("DRV_STATUS")?;
        if status.spi_status.map(|s| s != 0).unwrap_or(false) {
            self.optimized_spi.set(true);
        }
        let reactor = self.printer.get_reactor();
        let this = self.weak.upgrade().expect("TMCStallguardDump built by new()");
        let timer = reactor.register_timer(move |eventtime| this._query_tmc(eventtime), NOW);
        *self.query_timer.borrow_mut() = Some(timer);
        Ok(())
    }

    /// `_stop()`
    pub fn _stop(&self) -> KResult<()> {
        let timer = self.query_timer.borrow_mut().take();
        match timer {
            Some(timer) => self.printer.get_reactor().unregister_timer(&timer),
            None => return Err(none_attr_error("waketime")),
        }
        self.samples.borrow_mut().clear();
        Ok(())
    }

    /// `_query_tmc(eventtime)`: the query timer callback.
    pub fn _query_tmc(&self, eventtime: f64) -> f64 {
        let mut sg_result = -1i64;
        let mut cs_actual = -1i64;
        let mut recv_time = eventtime;
        let outcome = (|| -> KResult<()> {
            let sg4_is_sg4 = self.sg4_reg_name.as_deref() == Some("SG4_RESULT");
            if self.optimized_spi.get() || sg4_is_sg4 {
                // TMC2130/TMC5160/TMC2240
                let status = self.mcu_tmc.get_register_raw("DRV_STATUS")?;
                let reg_val = status.data;
                cs_actual = self.fields.get_field("cs_actual", Some(reg_val), None)?;
                sg_result = self.fields.get_field("sg_result", Some(reg_val), None)?;
                let is_stealth = self.fields.get_field("stealth", Some(reg_val), None)?;
                recv_time = status.receive_time;
                if is_stealth != 0 && sg4_is_sg4 {
                    let sg4_ret = self.mcu_tmc.get_register_raw("SG4_RESULT")?;
                    sg_result = sg4_ret.data;
                    recv_time = sg4_ret.receive_time;
                }
            } else if self.sg4_reg_name.as_deref() == Some("SG_RESULT") {
                // TMC2209
                let sg4_ret = self.mcu_tmc.get_register_raw("SG_RESULT")?;
                sg_result = sg4_ret.data;
                recv_time = sg4_ret.receive_time;
            }
            Ok(())
        })();
        if let Err(e) = outcome {
            if is_command_error(&e) {
                *self.error.borrow_mut() = Some(e);
                return NEVER;
            }
            unhandled(e);
        }
        let print_time = self.mcu.estimated_print_time(recv_time);
        self.samples.borrow_mut().push((print_time, sg_result, cs_actual));
        if self.optimized_spi.get() {
            return eventtime + 0.001;
        }
        // UART queried as fast as possible
        eventtime + 0.005
    }

    /// `_dump(eventtime)`: the batch callback.
    pub fn _dump(&self, _eventtime: f64) -> KResult<PyValue> {
        if let Some(e) = self.error.cloned() {
            return Err(e);
        }
        let samples = std::mem::take(&mut *self.samples.borrow_mut());
        let samples = PyValue::List(
            samples
                .into_iter()
                .map(|(t, sg, cs)| PyValue::Tuple(vec![PyValue::Float(t), int_value(sg), int_value(cs)]))
                .collect(),
        );
        Ok(status_dict(vec![("data", samples)]))
    }
}

// ---------------------------------------------------------------------------
// G-Code command helpers (`tmc.py:315-554`)
// ---------------------------------------------------------------------------

const CMD_INIT_TMC_HELP: &str = "Initialize TMC stepper driver registers";
const CMD_SET_TMC_FIELD_HELP: &str = "Set a register field of a TMC driver";
const CMD_SET_TMC_CURRENT_HELP: &str = "Set the current of a TMC driver";
const CMD_DUMP_TMC_HELP: &str = "Read and display TMC stepper driver registers";

/// `TMCCommandHelper` (`tmc.py:319`).
pub struct TMCCommandHelper {
    printer: Printer,
    stepper_name: String,
    name: String,
    mcu_tmc: Arc<dyn McuTmcApi>,
    current_helper: Arc<dyn TmcCurrentHelperApi>,
    fields: Arc<FieldHelper>,
    stepper: GCell<Option<Stepper>>,
    mcu_phase_offset: GCell<Option<i64>>,
    toff: GCell<Option<i64>>,
    stepper_enable: Arc<dyn PrinterStepperEnableApi>,
    enable_mutex: ReactorMutex,
    read_registers: GCell<Option<Vec<String>>>,
    read_translate: GCell<Option<ReadTranslate>>,
    /// `echeck_helper`
    pub echeck_helper: Arc<TMCErrorCheck>,
    /// `record_helper`
    pub record_helper: Arc<TMCStallguardDump>,
    weak: Weak<TMCCommandHelper>,
}

impl TMCCommandHelper {
    /// `TMCCommandHelper(config, mcu_tmc, current_helper)`
    pub fn new(
        config: &Config,
        mcu_tmc: Arc<dyn McuTmcApi>,
        current_helper: Arc<dyn TmcCurrentHelperApi>,
    ) -> KResult<Arc<TMCCommandHelper>> {
        let printer = config.get_printer();
        let stepper_name = stepper_name_of(config);
        let cfg_name = config.get_name();
        let name = cfg_name.split_whitespace().last().unwrap_or("").to_string();
        let fields = mcu_tmc.get_fields();
        let stepper_enable =
            load_required::<dyn PrinterStepperEnableApi>(&printer, config, "stepper_enable")?;
        let enable_mutex = printer.get_reactor().mutex(false);
        // Common tmc helpers
        let echeck_helper = TMCErrorCheck::new(config, mcu_tmc.clone())?;
        let record_helper = TMCStallguardDump::new(config, mcu_tmc.clone())?;
        tmc_microstep_helper(config, &*mcu_tmc)?;
        let this = Arc::new_cyclic(|weak| TMCCommandHelper {
            printer: printer.clone(),
            stepper_name,
            name: name.clone(),
            mcu_tmc,
            current_helper,
            fields,
            stepper: GCell::new(None),
            mcu_phase_offset: GCell::new(None),
            toff: GCell::new(None),
            stepper_enable,
            enable_mutex,
            read_registers: GCell::new(None),
            read_translate: GCell::new(None),
            echeck_helper,
            record_helper,
            weak: weak.clone(),
        });
        // Register callbacks
        for event in [EV_STEPPER_SYNC_MCU_POSITION, EV_STEPPER_SET_DIR_INVERTED] {
            let h = this.clone();
            printer.register_event_handler(
                event,
                Box::new(move |params: &[EventArg]| {
                    let stepper = step_event_stepper(params)?;
                    h._handle_sync_mcu_pos(&stepper)?;
                    Ok(PyValue::None)
                }),
            );
        }
        let h = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_MCU_IDENTIFY,
            Box::new(move |_params: &[EventArg]| {
                h._handle_mcu_identify()?;
                Ok(PyValue::None)
            }),
        );
        let h = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params: &[EventArg]| {
                h._handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        // Register commands
        let gcode = lookup_required::<dyn GCodeApi>(&printer, "gcode")?;
        let h = this.clone();
        gcode.register_mux_command(
            "SET_TMC_FIELD",
            "STEPPER",
            Some(&name),
            Box::new(move |gcmd: &GCmd| h.cmd_SET_TMC_FIELD(gcmd)),
            Some(CMD_SET_TMC_FIELD_HELP),
        )?;
        let h = this.clone();
        gcode.register_mux_command(
            "INIT_TMC",
            "STEPPER",
            Some(&name),
            Box::new(move |gcmd: &GCmd| h.cmd_INIT_TMC(gcmd)),
            Some(CMD_INIT_TMC_HELP),
        )?;
        let h = this.clone();
        gcode.register_mux_command(
            "SET_TMC_CURRENT",
            "STEPPER",
            Some(&name),
            Box::new(move |gcmd: &GCmd| h.cmd_SET_TMC_CURRENT(gcmd)),
            Some(CMD_SET_TMC_CURRENT_HELP),
        )?;
        Ok(this)
    }

    /// `_init_registers(print_time=None)`
    pub fn _init_registers(&self, print_time: Option<f64>) -> KResult<()> {
        // Send registers
        let names: Vec<String> = self.fields.registers().keys().cloned().collect();
        for reg_name in names {
            // Val may change during loop
            let val = self.fields.get_register_value(&reg_name).ok_or_else(|| key_error(&reg_name))?;
            self.mcu_tmc.set_register(&reg_name, val, print_time)?;
        }
        Ok(())
    }

    /// `cmd_INIT_TMC(gcmd)`
    pub fn cmd_INIT_TMC(&self, _gcmd: &GCmd) -> KResult<()> {
        log::info(&format!("INIT_TMC {}", self.name));
        let toolhead = lookup_required::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let print_time = toolhead.get_last_move_time();
        self._init_registers(Some(print_time))
    }

    /// `cmd_SET_TMC_FIELD(gcmd)`
    pub fn cmd_SET_TMC_FIELD(&self, gcmd: &GCmd) -> KResult<()> {
        let field_name = gcmd.get("FIELD", GetOpts::default())?.str().to_lowercase();
        let reg_name = match self.fields.lookup_register(&field_name) {
            Some(r) => r,
            None => {
                return Err(KlippyError::command(format!("Unknown field name '{}'", field_name)))
            }
        };
        let value = gcmd.get_int("VALUE", GcmdGetIntOpts { default: Some(None), ..Default::default() })?;
        let velocity = gcmd.get_float(
            "VELOCITY",
            GcmdGetFloatOpts { default: Some(None), minval: Some(0.), ..Default::default() },
        )?;
        if value.is_none() == velocity.is_none() {
            return Err(KlippyError::command("Specify either VALUE or VELOCITY"));
        }
        let value = match velocity {
            Some(velocity) => {
                if self.mcu_tmc.get_tmc_frequency().is_none() {
                    return Err(KlippyError::command("VELOCITY parameter not supported by this driver"));
                }
                let stepper = self.stepper.cloned();
                tmc_tstep_helper(&*self.mcu_tmc, velocity, stepper.as_ref(), None)?
            }
            None => value.expect("exactly one of VALUE and VELOCITY is set"),
        };
        let reg_val = self.fields.set_field(&field_name, value, None, None)?;
        let toolhead = lookup_required::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let print_time = toolhead.get_last_move_time();
        self.mcu_tmc.set_register(&reg_name, reg_val, Some(print_time))
    }

    /// `cmd_SET_TMC_CURRENT(gcmd)`
    pub fn cmd_SET_TMC_CURRENT(&self, gcmd: &GCmd) -> KResult<()> {
        let ch = &self.current_helper;
        let (mut prev_cur, mut prev_hold_cur, req_hold_cur, max_cur) = ch.get_current();
        let run_current = gcmd.get_float(
            "CURRENT",
            GcmdGetFloatOpts {
                default: Some(None),
                minval: Some(0.),
                maxval: Some(max_cur),
                ..Default::default()
            },
        )?;
        let hold_current = gcmd.get_float(
            "HOLDCURRENT",
            GcmdGetFloatOpts {
                default: Some(None),
                above: Some(0.),
                maxval: Some(max_cur),
                ..Default::default()
            },
        )?;
        if run_current.is_some() || hold_current.is_some() {
            let run_current = run_current.unwrap_or(prev_cur);
            let hold_current = hold_current.or(req_hold_cur);
            let toolhead = lookup_required::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
            let print_time = toolhead.get_last_move_time();
            ch.set_current(run_current, hold_current, print_time)?;
            let cur = ch.get_current();
            prev_cur = cur.0;
            prev_hold_cur = cur.1;
        }
        // Report values
        let msg = match prev_hold_cur {
            None => pct("Run Current: %0.2fA", &[PctValue::Float(prev_cur)]),
            Some(hold) => pct(
                "Run Current: %0.2fA Hold Current: %0.2fA",
                &[PctValue::Float(prev_cur), PctValue::Float(hold)],
            ),
        };
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    // Stepper phase tracking

    /// `_get_phases()`
    pub fn _get_phases(&self) -> KResult<i64> {
        let mres = self.fields.get_field("mres", None, None)?;
        let shifted = match u32::try_from(mres) {
            Ok(s) => 256i64.checked_shr(s).unwrap_or(0),
            Err(_) => return Err(internal("ValueError", "negative shift count".to_string())),
        };
        Ok(shifted * 4)
    }

    /// `get_phase_offset()`
    pub fn get_phase_offset(&self) -> (Option<i64>, i64) {
        let phases = self._get_phases().unwrap_or_else(|e| unhandled(e));
        (self.mcu_phase_offset.get(), phases)
    }

    /// `_query_phase()`
    pub fn _query_phase(&self) -> KResult<i64> {
        let mut field_name = "mscnt";
        if self.fields.lookup_register(field_name).is_none() {
            // TMC2660 uses MSTEP
            field_name = "mstep";
        }
        let reg_name = self
            .fields
            .lookup_register(field_name)
            .ok_or_else(|| internal("KeyError", "None".to_string()))?;
        let reg = self.mcu_tmc.get_register(&reg_name)?;
        self.fields.get_field(field_name, Some(reg), None)
    }

    /// `_handle_sync_mcu_pos(stepper)`
    pub fn _handle_sync_mcu_pos(&self, stepper: &Stepper) -> KResult<()> {
        if stepper.get_name(None) != self.stepper_name {
            return Ok(());
        }
        let driver_phase = match self._query_phase() {
            Ok(p) => p,
            Err(e) if is_command_error(&e) => {
                log::info(&format!("Unable to obtain tmc {} phase", self.stepper_name));
                self.mcu_phase_offset.set(None);
                let enable_line = self.stepper_enable.lookup_enable(&self.stepper_name)?;
                if enable_line.is_motor_enabled() {
                    return Err(e);
                }
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let mut driver_phase = driver_phase;
        if !stepper.get_dir_inverted().0 {
            driver_phase = 1023 - driver_phase;
        }
        let phases = self._get_phases()?;
        if phases == 0 {
            return Err(internal(
                "ZeroDivisionError",
                "integer division or modulo by zero".to_string(),
            ));
        }
        let scaled = driver_phase as f64 / 1024. * phases as f64 + 0.5;
        let phase = pyint_saturating(&py_int_of_float(scaled)?).rem_euclid(phases);
        let moff = phase.wrapping_sub(stepper.get_mcu_position(None)).rem_euclid(phases);
        if let Some(prev) = self.mcu_phase_offset.get() {
            if prev != moff {
                log::warning(&format!(
                    "Stepper {} phase change (was {} now {})",
                    self.stepper_name, prev, moff
                ));
            }
        }
        self.mcu_phase_offset.set(Some(moff));
        Ok(())
    }

    // Stepper enable/disable tracking

    /// `_do_enable(print_time)`
    pub fn _do_enable(&self, _print_time: f64) -> KResult<()> {
        if let Some(toff) = self.toff.get() {
            // Shared enable via comms handling
            self.fields.set_field("toff", toff, None, None)?;
        }
        self._init_registers(None)?;
        let did_reset = self.echeck_helper.start_checks()?;
        if did_reset {
            self.mcu_phase_offset.set(None);
        }
        // Calculate phase offset
        if self.mcu_phase_offset.get().is_some() {
            return Ok(());
        }
        let gcode = lookup_required::<dyn GCodeApi>(&self.printer, "gcode")?;
        let mutex = gcode.get_mutex();
        with_mutex(&mutex, || -> KResult<()> {
            if self.mcu_phase_offset.get().is_some() {
                return Ok(());
            }
            log::info(&format!("Pausing toolhead to calculate {} phase offset", self.stepper_name));
            let toolhead = lookup_required::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
            toolhead.wait_moves();
            let stepper = self.stepper.cloned().ok_or_else(|| none_attr_error("get_name"))?;
            self._handle_sync_mcu_pos(&stepper)
        })
    }

    /// `_do_disable(print_time)`
    pub fn _do_disable(&self, print_time: f64) -> KResult<()> {
        if self.toff.get().is_some() {
            let val = self.fields.set_field("toff", 0, None, None)?;
            let reg_name = self
                .fields
                .lookup_register("toff")
                .ok_or_else(|| internal("KeyError", "None".to_string()))?;
            self.mcu_tmc.set_register(&reg_name, val, Some(print_time))?;
        }
        self.echeck_helper.stop_checks();
        Ok(())
    }

    /// `_handle_stepper_enable(print_time, is_enable)`: registered as the
    /// enable line state callback.
    pub fn _handle_stepper_enable(&self, print_time: f64, is_enable: bool) {
        let this = self.weak.upgrade().expect("TMCCommandHelper built by new()");
        self.printer.get_reactor().register_callback(
            move |_eventtime| {
                let outcome = with_mutex(&this.enable_mutex, || {
                    if is_enable {
                        this._do_enable(print_time)
                    } else {
                        this._do_disable(print_time)
                    }
                });
                if let Err(e) = outcome {
                    if is_command_error(&e) {
                        this.printer.invoke_shutdown(e.message(), empty_dict());
                    } else {
                        unhandled(e);
                    }
                }
            },
            NOW,
        );
    }

    // Initial startup handling

    /// `_handle_mcu_identify()`
    pub fn _handle_mcu_identify(&self) -> KResult<()> {
        // Lookup stepper object
        let force_move = lookup_required::<dyn ForceMoveApi>(&self.printer, "force_move")?;
        let stepper = force_move.lookup_stepper(Some(&self.stepper_name))?;
        // Note pulse duration and step_both_edge optimizations available
        stepper.setup_default_pulse_duration(0.000000100, true);
        *self.stepper.borrow_mut() = Some(stepper);
        Ok(())
    }

    /// `_handle_connect()`
    pub fn _handle_connect(&self) -> KResult<()> {
        // Check if using step on both edges optimization
        let stepper = self.stepper.cloned().ok_or_else(|| none_attr_error("get_pulse_duration"))?;
        let (_pulse_duration, step_both_edge) = stepper.get_pulse_duration();
        if step_both_edge {
            self.fields.set_field("dedge", 1, None, None)?;
        }
        // Check for soft stepper enable/disable
        let enable_line = self.stepper_enable.lookup_enable(&self.stepper_name)?;
        let this = self.weak.upgrade().expect("TMCCommandHelper built by new()");
        enable_line.register_state_callback(Box::new(move |print_time, is_enable| {
            this._handle_stepper_enable(print_time, is_enable)
        }));
        if !enable_line.has_dedicated_enable() {
            self.toff.set(Some(self.fields.get_field("toff", None, None)?));
            self.fields.set_field("toff", 0, None, None)?;
            log::info(&format!("Enabling TMC virtual enable for '{}'", self.stepper_name));
        }
        // Send init
        if let Err(e) = self._init_registers(None) {
            if !is_command_error(&e) {
                return Err(e);
            }
            log::info(&format!("TMC {} failed to init: {}", self.name, e.message()));
        }
        Ok(())
    }

    /// `get_status(eventtime=None)`
    pub fn get_status(&self, eventtime: Option<f64>) -> PyValue {
        let mut cpos = None;
        if let Some(offset) = self.mcu_phase_offset.get() {
            if let Some(stepper) = self.stepper.cloned() {
                cpos = Some(stepper.mcu_to_commanded_position(offset));
            }
        }
        let current = self.current_helper.get_current();
        let mut res = IndexMap::new();
        res.insert(str_key("mcu_phase_offset"), opt_int_value(self.mcu_phase_offset.get()));
        res.insert(str_key("phase_offset_position"), opt_float_value(cpos));
        res.insert(str_key("run_current"), PyValue::Float(current.0));
        res.insert(str_key("hold_current"), opt_float_value(current.1));
        if let PyValue::Dict(d) = self.echeck_helper.get_status(eventtime) {
            for (k, v) in d {
                res.insert(k, v);
            }
        }
        PyValue::Dict(res)
    }

    // DUMP_TMC support

    /// `setup_register_dump(read_registers, read_translate=None)`
    pub fn setup_register_dump(
        &self,
        read_registers: Vec<String>,
        read_translate: Option<ReadTranslate>,
    ) -> KResult<()> {
        *self.read_registers.borrow_mut() = Some(read_registers);
        *self.read_translate.borrow_mut() = read_translate;
        let gcode = lookup_required::<dyn GCodeApi>(&self.printer, "gcode")?;
        let this = self.weak.upgrade().expect("TMCCommandHelper built by new()");
        gcode.register_mux_command(
            "DUMP_TMC",
            "STEPPER",
            Some(&self.name),
            Box::new(move |gcmd: &GCmd| this.cmd_DUMP_TMC(gcmd)),
            Some(CMD_DUMP_TMC_HELP),
        )
    }

    /// `cmd_DUMP_TMC(gcmd)`
    pub fn cmd_DUMP_TMC(&self, gcmd: &GCmd) -> KResult<()> {
        log::info(&format!("DUMP_TMC {}", self.name));
        let read_registers = self.read_registers.cloned().unwrap_or_default();
        let read_translate = self.read_translate.cloned();
        let translate = |reg_name: String, val: i64| -> (String, i64) {
            match &read_translate {
                Some(f) => f(&reg_name, val),
                None => (reg_name, val),
            }
        };
        let reg_name = gcmd.get("REGISTER", GetOpts { default: Some(PyValue::None), ..Default::default() })?;
        if let PyValue::Str(reg_name) = reg_name {
            let reg_name = reg_name.to_uppercase();
            let val = self.fields.get_register_value(&reg_name);
            let readable = read_registers.contains(&reg_name);
            match val {
                Some(val) if !readable => {
                    // write-only register
                    gcmd.respond_info(&self.fields.pretty_format(&reg_name, val), true);
                }
                _ if readable => {
                    // readable register
                    let val = self.mcu_tmc.get_register(&reg_name)?;
                    let (reg_name, val) = translate(reg_name, val);
                    gcmd.respond_info(&self.fields.pretty_format(&reg_name, val), true);
                }
                _ => {
                    return Err(KlippyError::command(format!("Unknown register name '{}'", reg_name)));
                }
            }
        } else {
            gcmd.respond_info("========== Write-only registers ==========", true);
            for (reg_name, val) in self.fields.registers() {
                if !read_registers.contains(&reg_name) {
                    gcmd.respond_info(&self.fields.pretty_format(&reg_name, val), true);
                }
            }
            gcmd.respond_info("========== Queried registers ==========", true);
            for reg_name in read_registers.iter() {
                let val = self.mcu_tmc.get_register(reg_name)?;
                let (reg_name, val) = translate(reg_name.clone(), val);
                gcmd.respond_info(&self.fields.pretty_format(&reg_name, val), true);
            }
        }
        Ok(())
    }
}

impl TMCCommandHelperApi for TMCCommandHelper {
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        TMCCommandHelper::get_phase_offset(self)
    }
}

// ---------------------------------------------------------------------------
// TMC virtual pins (`tmc.py:557-644`)
// ---------------------------------------------------------------------------

/// `TMCVirtualPinHelper` (`tmc.py:562`): "sensorless homing".
pub struct TMCVirtualPinHelper {
    printer: Printer,
    mcu_tmc: Arc<dyn McuTmcApi>,
    fields: Arc<FieldHelper>,
    diag_pin: Option<String>,
    diag_pin_field: Option<String>,
    mcu_endstop: GCell<Option<Endstop>>,
    dirty_regs: GCell<IndexMap<String, i64>>,
    prev_state: GCell<IndexMap<String, i64>>,
    weak: Weak<TMCVirtualPinHelper>,
}

fn optional_str(config: &Config, option: &str) -> KResult<Option<String>> {
    config.get(option, GetStrOpts { default: Some(None), note_valid: true })
}

fn pins_error(msg: &str) -> KlippyError {
    KlippyError::Other { class_name: "error".to_string(), message: msg.to_string() }
}

impl TMCVirtualPinHelper {
    /// `TMCVirtualPinHelper(config, mcu_tmc)`
    pub fn new(config: &Config, mcu_tmc: Arc<dyn McuTmcApi>) -> KResult<Arc<TMCVirtualPinHelper>> {
        let printer = config.get_printer();
        let fields = mcu_tmc.get_fields();
        let diag_pin;
        let diag_pin_field;
        if fields.lookup_register("diag0_stall").is_some() {
            if optional_str(config, "diag0_pin")?.is_some() {
                diag_pin = config.get("diag0_pin", GetStrOpts::default())?;
                diag_pin_field = Some("diag0_stall".to_string());
            } else {
                diag_pin = optional_str(config, "diag1_pin")?;
                diag_pin_field = Some("diag1_stall".to_string());
            }
        } else {
            diag_pin = optional_str(config, "diag_pin")?;
            diag_pin_field = None;
        }
        let this = Arc::new_cyclic(|weak| TMCVirtualPinHelper {
            printer: printer.clone(),
            mcu_tmc,
            fields,
            diag_pin,
            diag_pin_field,
            mcu_endstop: GCell::new(None),
            dirty_regs: GCell::new(IndexMap::new()),
            prev_state: GCell::new(IndexMap::new()),
            weak: weak.clone(),
        });
        // Register virtual_endstop pin
        let name = config.get_name();
        let name_parts: Vec<&str> = name.split_whitespace().collect();
        let ppins = lookup_required::<dyn PinsApi>(&printer, "pins")?;
        let first = name_parts.first().copied().ok_or_else(|| {
            internal("IndexError", "list index out of range".to_string())
        })?;
        let last = name_parts.last().copied().unwrap_or(first);
        ppins.register_chip(&format!("{}_{}", first, last), this.clone() as Chip)?;
        Ok(this)
    }

    fn _set_field(&self, field_name: Option<&str>, value: i64) -> KResult<()> {
        let field_name = field_name.ok_or_else(|| internal("KeyError", "None".to_string()))?;
        let prev = self.fields.get_field(field_name, None, None)?;
        self.prev_state.borrow_mut().insert(field_name.to_string(), prev);
        let reg_name = self
            .fields
            .lookup_register(field_name)
            .ok_or_else(|| internal("KeyError", "None".to_string()))?;
        let new_val = self.fields.set_field(field_name, value, None, None)?;
        self.dirty_regs.borrow_mut().insert(reg_name, new_val);
        Ok(())
    }

    fn _send_fields(&self) -> KResult<()> {
        let dirty: Vec<(String, i64)> =
            self.dirty_regs.borrow().iter().map(|(k, v)| (k.clone(), *v)).collect();
        for (reg, val) in dirty {
            self.mcu_tmc.set_register(&reg, val, None)?;
        }
        self.dirty_regs.borrow_mut().clear();
        Ok(())
    }

    fn has_endstop(&self, hmove: &dyn HomingMoveApi) -> bool {
        let mine = match self.mcu_endstop.cloned() {
            Some(e) => e,
            None => return false,
        };
        hmove
            .get_mcu_endstops()
            .iter()
            .any(|e| Arc::as_ptr(e) as *const () == Arc::as_ptr(&mine) as *const ())
    }

    /// `handle_homing_move_begin(hmove)`
    pub fn handle_homing_move_begin(&self, hmove: &dyn HomingMoveApi) -> KResult<()> {
        if !self.has_endstop(hmove) {
            return Ok(());
        }
        let mut sg4_thrs = 0;
        if self.fields.lookup_register("sg4_thrs").is_some() {
            sg4_thrs = self.fields.get_field("sg4_thrs", None, None)?;
        }
        // Enable/disable stealthchop
        let reg = self.fields.lookup_register("en_pwm_mode");
        if reg.is_none() {
            // On "stallguard4" drivers, "stealthchop" must be enabled
            self._set_field(Some("tpwmthrs"), 0)?;
            self._set_field(Some("en_spreadcycle"), 0)?;
        } else if sg4_thrs != 0 {
            // TMC2240 using SG4, "stealthchop" must be enabled
            self._set_field(Some("en_pwm_mode"), 1)?;
            self._set_field(Some("tpwmthrs"), 0)?;
            self._set_field(self.diag_pin_field.as_deref(), 1)?;
        } else {
            // On earlier drivers, "stealthchop" must be disabled
            self._set_field(Some("en_pwm_mode"), 0)?;
            self._set_field(self.diag_pin_field.as_deref(), 1)?;
        }
        // Enable tcoolthrs (if not already)
        if self.fields.get_field("tcoolthrs", None, None)? == 0 {
            self._set_field(Some("tcoolthrs"), 0xfffff)?;
        }
        // Disable thigh
        if self.fields.lookup_register("thigh").is_some() {
            self._set_field(Some("thigh"), 0)?;
        }
        self._send_fields()
    }

    /// `handle_homing_move_end(hmove)`
    pub fn handle_homing_move_end(&self, hmove: &dyn HomingMoveApi) -> KResult<()> {
        if !self.has_endstop(hmove) {
            return Ok(());
        }
        // Restore previous state
        let prev: Vec<(String, i64)> =
            self.prev_state.borrow().iter().map(|(k, v)| (k.clone(), *v)).collect();
        for (field, val) in prev {
            self._set_field(Some(&field), val)?;
        }
        self._send_fields()?;
        self.prev_state.borrow_mut().clear();
        Ok(())
    }
}

fn homing_move_arg(params: &[EventArg]) -> KResult<Arc<dyn HomingMoveApi>> {
    match params.first().and_then(|p| p.as_handle::<dyn HomingMoveApi>()) {
        Some(h) => Ok(h),
        None => Err(internal(
            "TypeError",
            "homing event handler requires a homing move argument".to_string(),
        )),
    }
}

impl ChipApi for TMCVirtualPinHelper {
    /// `setup_pin(pin_type, pin_params)`
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        // Validate pin
        let ppins = lookup_required::<dyn PinsApi>(&self.printer, "pins")?;
        if pin_type != "endstop" || pin_params.pin != "virtual_endstop" {
            return Err(pins_error("tmc virtual endstop only useful as endstop"));
        }
        if pin_params.invert != 0 || pin_params.pullup != 0 {
            return Err(pins_error("Can not pullup/invert tmc virtual pin"));
        }
        let diag_pin = match &self.diag_pin {
            Some(p) => p.clone(),
            None => return Err(pins_error("tmc virtual endstop requires diag pin config")),
        };
        // Setup for sensorless homing
        let this = self.weak.upgrade().expect("TMCVirtualPinHelper built by new()");
        let h = this.clone();
        self.printer.register_event_handler(
            EV_HOMING_MOVE_BEGIN,
            Box::new(move |params: &[EventArg]| {
                h.handle_homing_move_begin(&*homing_move_arg(params)?)?;
                Ok(PyValue::None)
            }),
        );
        self.printer.register_event_handler(
            EV_HOMING_MOVE_END,
            Box::new(move |params: &[EventArg]| {
                this.handle_homing_move_end(&*homing_move_arg(params)?)?;
                Ok(PyValue::None)
            }),
        );
        let pin = ppins.setup_pin("endstop", &diag_pin)?;
        if let Pin::Endstop(e) = &pin {
            *self.mcu_endstop.borrow_mut() = Some(e.clone());
        }
        Ok(pin)
    }

    fn get_mcu(&self) -> Mcu {
        self.mcu_tmc.get_mcu()
    }
}

// ---------------------------------------------------------------------------
// Config reading helpers (`tmc.py:647-739`)
// ---------------------------------------------------------------------------

/// `TMCWaveTableHelper(config, mcu_tmc)`: initialize the wave table from
/// config or defaults.
pub fn tmc_wave_table_helper(config: &Config, mcu_tmc: &dyn McuTmcApi) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let defaults: [(&str, i64); 17] = [
        ("mslut0", 0xAAAAB554),
        ("mslut1", 0x4A9554AA),
        ("mslut2", 0x24492929),
        ("mslut3", 0x10104222),
        ("mslut4", 0xFBFFFFFF),
        ("mslut5", 0xB5BB777D),
        ("mslut6", 0x49295556),
        ("mslut7", 0x00404222),
        ("w0", 2),
        ("w1", 1),
        ("w2", 1),
        ("w3", 1),
        ("x1", 128),
        ("x2", 255),
        ("x3", 255),
        ("start_sin", 0),
        ("start_sin90", 247),
    ];
    for (name, default) in defaults {
        fields.set_config_field(config, name, ConfigFieldDefault::Int(default))?;
    }
    Ok(())
}

/// `TMCMicrostepHelper(config, mcu_tmc)`
pub fn tmc_microstep_helper(config: &Config, mcu_tmc: &dyn McuTmcApi) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let stepper_name = stepper_name_of(config);
    if !config.has_section(&stepper_name) {
        return Err(KlippyError::config(format!(
            "Could not find config section '[{}]' required by tmc driver",
            stepper_name
        )));
    }
    let sconfig = config.getsection(&stepper_name);
    let mut steps = IndexMap::new();
    for (k, v) in [(256, 0), (128, 1), (64, 2), (32, 3), (16, 4), (8, 5), (4, 6), (2, 7), (1, 8)] {
        steps.insert(int_value(k), int_value(v));
    }
    let mres = sconfig.getchoice("microsteps", &PyValue::Dict(steps), GetChoiceOpts::default())?;
    fields.set_field("mres", value_as_i64(&mres), None, None)?;
    let interpolate = config.getboolean(
        "interpolate",
        GetBoolOpts { default: Some(Some(true)), note_valid: true },
    )?;
    fields.set_field("intpol", i64::from(interpolate.unwrap_or(true)), None, None)?;
    Ok(())
}

/// `TMCtstepHelper(mcu_tmc, velocity, pstepper=None, config=None)`:
/// calculates TSTEP based values from velocity.
pub fn tmc_tstep_helper(
    mcu_tmc: &dyn McuTmcApi,
    velocity: f64,
    pstepper: Option<&Stepper>,
    config: Option<&Config>,
) -> KResult<i64> {
    if velocity <= 0. {
        return Ok(0xfffff);
    }
    let step_dist = match pstepper {
        Some(pstepper) => pstepper.get_step_dist(),
        None => {
            let config = config.ok_or_else(|| none_attr_error("get_name"))?;
            let stepper_name = stepper_name_of(config);
            let sconfig = config.getsection(&stepper_name);
            let printer = config.get_printer();
            let toolhead = lookup_required::<dyn ToolHeadApi>(&printer, "toolhead")?;
            let (rotation_dist, steps_per_rotation) =
                toolhead.stepper_factory().parse_step_distance(&sconfig, None, false)?;
            true_div(rotation_dist, steps_per_rotation)?
        }
    };
    let mres = mcu_tmc.get_fields().get_field("mres", None, None)?;
    let step_dist_256 = true_div(step_dist, 1i64.wrapping_shl(mres as u32) as f64)?;
    let tmc_freq = mcu_tmc.get_tmc_frequency().ok_or_else(|| {
        internal(
            "TypeError",
            "unsupported operand type(s) for *: 'NoneType' and 'float'".to_string(),
        )
    })?;
    let threshold = pyint_saturating(&py_int_of_float(true_div(tmc_freq * step_dist_256, velocity)? + 0.5)?);
    Ok(threshold.clamp(0, 0xfffff))
}

/// `TMCStealthchopHelper(config, mcu_tmc)`: configure the stealthChop to
/// spreadCycle transition velocity.
pub fn tmc_stealthchop_helper(config: &Config, mcu_tmc: &dyn McuTmcApi) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let mut en_pwm_mode = false;
    let velocity = config.getfloat(
        "stealthchop_threshold",
        GetFloatOpts { default: Some(None), minval: Some(0.), ..Default::default() },
    )?;
    let mut tpwmthrs = 0xfffff;
    if let Some(velocity) = velocity {
        en_pwm_mode = true;
        tpwmthrs = tmc_tstep_helper(mcu_tmc, velocity, None, Some(config))?;
    }
    fields.set_field("tpwmthrs", tpwmthrs, None, None)?;
    if fields.lookup_register("en_pwm_mode").is_some() {
        fields.set_field("en_pwm_mode", i64::from(en_pwm_mode), None, None)?;
    } else {
        // TMC2208 uses en_spreadCycle
        fields.set_field("en_spreadcycle", i64::from(!en_pwm_mode), None, None)?;
    }
    Ok(())
}

/// `TMCVcoolthrsHelper(config, mcu_tmc)`: configure the StallGuard and
/// CoolStep minimum velocity.
pub fn tmc_vcoolthrs_helper(config: &Config, mcu_tmc: &dyn McuTmcApi) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let velocity = config.getfloat(
        "coolstep_threshold",
        GetFloatOpts { default: Some(None), minval: Some(0.), ..Default::default() },
    )?;
    let mut tcoolthrs = 0;
    if let Some(velocity) = velocity {
        tcoolthrs = tmc_tstep_helper(mcu_tmc, velocity, None, Some(config))?;
    }
    fields.set_field("tcoolthrs", tcoolthrs, None, None)?;
    Ok(())
}

/// `TMCVhighHelper(config, mcu_tmc)`: configure the StallGuard and CoolStep
/// maximum velocity and the SpreadCycle-FullStepping (high velocity)
/// threshold.
pub fn tmc_vhigh_helper(config: &Config, mcu_tmc: &dyn McuTmcApi) -> KResult<()> {
    let fields = mcu_tmc.get_fields();
    let velocity = config.getfloat(
        "high_velocity_threshold",
        GetFloatOpts { default: Some(None), minval: Some(0.), ..Default::default() },
    )?;
    let mut thigh = 0;
    if let Some(velocity) = velocity {
        thigh = tmc_tstep_helper(mcu_tmc, velocity, None, Some(config))?;
    }
    fields.set_field("thigh", thigh, None, None)?;
    Ok(())
}

/// `tmc.py` has no `load_config`: the factory is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
