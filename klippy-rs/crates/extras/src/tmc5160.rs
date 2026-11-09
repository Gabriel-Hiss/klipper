//! Port of the `tmc5160` module (`klippy/extras/tmc5160.py`).
//!
//! The module is the register/field tables, [`Tmc5160CurrentHelper`]
//! (the `TMC5160CurrentHelper` of `tmc5160.py:265-318`) and the
//! [`Tmc5160`] printer object (`tmc5160.py:325-387`). Everything the
//! Python class delegates to lives in its owners and is used through
//! them: [`tmc::FieldHelper`], [`tmc::McuTmcApi`],
//! [`tmc::TMCVirtualPinHelper`], [`tmc::TMCCommandHelper`], the
//! wave-table / stealthChop / threshold configuration helpers of
//! [`tmc`], and `tmc2130::McuTmcSpi` (the `MCU_TMC_SPI` of
//! `tmc2130.py:261`).
//!
//! Units owned by this file (see `parity/inventory/units.json`):
//! - `extras.tmc5160.ModuleHeader` .. `ModuleHeader_4`: the register /
//!   field tables, `FieldFormatters`, and the `run_current`,
//!   `hold_current`, `sense_resistor` config reads.
//! - `extras.tmc5160.TMC5160`: the printer object. Its `get_status`
//!   and `get_phase_offset` are the command helper's, exactly as the
//!   Python assignments at `tmc5160.py:337-338` make them.
//! - `extras.tmc5160.LoadConfigPrefix`: the module factory entry
//!   point.
//!
//! `Tmc5160::new` builds the SPI transport through
//! `tmc2130::McuTmcSpi::new(config, registers_table(), fields,
//! TMC_FREQUENCY)`, mirroring `tmc2130.py:261`, and then runs the
//! constructor wiring; `load_config_prefix` is
//! `return TMC5160(config)` (`tmc5160.py:388-389`).

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::{GCell, KResult, KlippyError, PrinterObject};
use pycompat::libm::sqrt;
use pycompat::PyValue;
use std::sync::Arc;

use crate::tmc::{
    ConfigFieldDefault, FieldFormatter, FieldHelper, McuTmcApi, TMCCommandHelper,
    TMCCommandHelperApi, TMCVirtualPinHelper, TmcCurrentHelperApi, tmc_stealthchop_helper,
    tmc_vcoolthrs_helper, tmc_vhigh_helper, tmc_wave_table_helper,
};
use crate::tmc2130::McuTmcSpi;

// ---------------------------------------------------------------------------
// Constants (tmc5160.py:9, 262-263)
// ---------------------------------------------------------------------------

pub const TMC_FREQUENCY: f64 = 12000000.;
pub const VREF: f64 = 0.325;
pub const MAX_CURRENT: f64 = 10.000;

// ---------------------------------------------------------------------------
// Register / field tables (tmc5160.py:11-255)
// ---------------------------------------------------------------------------

/// `Registers` (tmc5160.py:11-73), in source order.
pub fn registers_table() -> IndexMap<String, i64> {
    let mut m = IndexMap::new();
    let entries: [(&str, i64); 61] = [
        ("GCONF", 0x00),
        ("GSTAT", 0x01),
        ("IFCNT", 0x02),
        ("SLAVECONF", 0x03),
        ("IOIN", 0x04),
        ("X_COMPARE", 0x05),
        ("OTP_READ", 0x07),
        ("FACTORY_CONF", 0x08),
        ("SHORT_CONF", 0x09),
        ("DRV_CONF", 0x0A),
        ("GLOBALSCALER", 0x0B),
        ("OFFSET_READ", 0x0C),
        ("IHOLD_IRUN", 0x10),
        ("TPOWERDOWN", 0x11),
        ("TSTEP", 0x12),
        ("TPWMTHRS", 0x13),
        ("TCOOLTHRS", 0x14),
        ("THIGH", 0x15),
        ("RAMPMODE", 0x20),
        ("XACTUAL", 0x21),
        ("VACTUAL", 0x22),
        ("VSTART", 0x23),
        ("A1", 0x24),
        ("V1", 0x25),
        ("AMAX", 0x26),
        ("VMAX", 0x27),
        ("DMAX", 0x28),
        ("D1", 0x2A),
        ("VSTOP", 0x2B),
        ("TZEROWAIT", 0x2C),
        ("XTARGET", 0x2D),
        ("VDCMIN", 0x33),
        ("SW_MODE", 0x34),
        ("RAMP_STAT", 0x35),
        ("XLATCH", 0x36),
        ("ENCMODE", 0x38),
        ("X_ENC", 0x39),
        ("ENC_CONST", 0x3A),
        ("ENC_STATUS", 0x3B),
        ("ENC_LATCH", 0x3C),
        ("ENC_DEVIATION", 0x3D),
        ("MSLUT0", 0x60),
        ("MSLUT1", 0x61),
        ("MSLUT2", 0x62),
        ("MSLUT3", 0x63),
        ("MSLUT4", 0x64),
        ("MSLUT5", 0x65),
        ("MSLUT6", 0x66),
        ("MSLUT7", 0x67),
        ("MSLUTSEL", 0x68),
        ("MSLUTSTART", 0x69),
        ("MSCNT", 0x6A),
        ("MSCURACT", 0x6B),
        ("CHOPCONF", 0x6C),
        ("COOLCONF", 0x6D),
        ("DCCTRL", 0x6E),
        ("DRV_STATUS", 0x6F),
        ("PWMCONF", 0x70),
        ("PWM_SCALE", 0x71),
        ("PWM_AUTO", 0x72),
        ("LOST_STEPS", 0x73),
    ];
    for (k, v) in entries {
        m.insert(k.to_string(), v);
    }
    m
}

/// `ReadRegisters` (tmc5160.py:75-79), in source order.
pub const READ_REGISTERS: &[&str] = &[
    "GCONF", "CHOPCONF", "GSTAT", "DRV_STATUS", "FACTORY_CONF", "IOIN", "LOST_STEPS",
    "MSCNT", "MSCURACT", "OTP_READ", "PWM_SCALE", "PWM_AUTO", "TSTEP",
];

/// `Fields` (tmc5160.py:81-247): register -> (field -> mask), in source order.
///
/// The masks are transcribed verbatim from the reference, where the
/// `<< 0` shifts document each field's bit position.
#[allow(clippy::identity_op)]
pub fn fields_table() -> IndexMap<String, IndexMap<String, i64>> {
    fn single(
        name: &str,
        field: &str,
        mask: i64,
        m: &mut IndexMap<String, IndexMap<String, i64>>,
    ) {
        let mut f = IndexMap::new();
        f.insert(field.to_string(), mask);
        m.insert(name.to_string(), f);
    }
    let mut m: IndexMap<String, IndexMap<String, i64>> = IndexMap::new();
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("semin", 0x0F << 0),
            ("seup", 0x03 << 5),
            ("semax", 0x0F << 8),
            ("sedn", 0x03 << 13),
            ("seimin", 0x01 << 15),
            ("sgt", 0x7F << 16),
            ("sfilt", 0x01 << 24),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("COOLCONF".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("toff", 0x0F << 0),
            ("hstrt", 0x07 << 4),
            ("hend", 0x0F << 7),
            ("fd3", 0x01 << 11),
            ("disfdcc", 0x01 << 12),
            ("chm", 0x01 << 14),
            ("tbl", 0x03 << 15),
            ("vhighfs", 0x01 << 18),
            ("vhighchm", 0x01 << 19),
            ("tpfd", 0x0F << 20),
            ("mres", 0x0F << 24),
            ("intpol", 0x01 << 28),
            ("dedge", 0x01 << 29),
            ("diss2g", 0x01 << 30),
            ("diss2vs", 0x01 << 31),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("CHOPCONF".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("bbmtime", 0x1F << 0),
            ("bbmclks", 0x0F << 8),
            ("otselect", 0x03 << 16),
            ("drvstrength", 0x03 << 18),
            ("filt_isense", 0x03 << 20),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("DRV_CONF".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("sg_result", 0x3FF << 0),
            ("s2vsa", 0x01 << 12),
            ("s2vsb", 0x01 << 13),
            ("stealth", 0x01 << 14),
            ("fsactive", 0x01 << 15),
            ("cs_actual", 0x1F << 16),
            ("stallguard", 0x01 << 24),
            ("ot", 0x01 << 25),
            ("otpw", 0x01 << 26),
            ("s2ga", 0x01 << 27),
            ("s2gb", 0x01 << 28),
            ("ola", 0x01 << 29),
            ("olb", 0x01 << 30),
            ("stst", 0x01 << 31),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("DRV_STATUS".to_string(), f);
    }
    single("FACTORY_CONF", "factory_conf", 0x1F << 0, &mut m);
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("recalibrate", 0x01 << 0),
            ("faststandstill", 0x01 << 1),
            ("en_pwm_mode", 0x01 << 2),
            ("multistep_filt", 0x01 << 3),
            ("shaft", 0x01 << 4),
            ("diag0_error", 0x01 << 5),
            ("diag0_otpw", 0x01 << 6),
            ("diag0_stall", 0x01 << 7),
            ("diag1_stall", 0x01 << 8),
            ("diag1_index", 0x01 << 9),
            ("diag1_onstate", 0x01 << 10),
            ("diag1_steps_skipped", 0x01 << 11),
            ("diag0_int_pushpull", 0x01 << 12),
            ("diag1_poscomp_pushpull", 0x01 << 13),
            ("small_hysteresis", 0x01 << 14),
            ("stop_enable", 0x01 << 15),
            ("direct_mode", 0x01 << 16),
            ("test_mode", 0x01 << 17),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("GCONF".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [("reset", 0x01 << 0), ("drv_err", 0x01 << 1), ("uv_cp", 0x01 << 2)] {
            f.insert(k.to_string(), v);
        }
        m.insert("GSTAT".to_string(), f);
    }
    single("GLOBALSCALER", "globalscaler", 0xFF << 0, &mut m);
    {
        let mut f = IndexMap::new();
        for (k, v) in [("ihold", 0x1F << 0), ("irun", 0x1F << 8), ("iholddelay", 0x0F << 16)] {
            f.insert(k.to_string(), v);
        }
        m.insert("IHOLD_IRUN".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("refl_step", 0x01 << 0),
            ("refr_dir", 0x01 << 1),
            ("encb_dcen_cfg4", 0x01 << 2),
            ("enca_dcin_cfg5", 0x01 << 3),
            ("drv_enn", 0x01 << 4),
            ("enc_n_dco_cfg6", 0x01 << 5),
            ("sd_mode", 0x01 << 6),
            ("swcomp_in", 0x01 << 7),
            ("version", 0xFF << 24),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("IOIN".to_string(), f);
    }
    single("LOST_STEPS", "lost_steps", 0xfffff << 0, &mut m);
    for i in 0..=7 {
        let name = format!("MSLUT{i}");
        let field = format!("mslut{i}");
        let mut f = IndexMap::new();
        f.insert(field, 0xffffffff);
        m.insert(name, f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("x3", 0xFF << 24),
            ("x2", 0xFF << 16),
            ("x1", 0xFF << 8),
            ("w3", 0x03 << 6),
            ("w2", 0x03 << 4),
            ("w1", 0x03 << 2),
            ("w0", 0x03 << 0),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("MSLUTSEL".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [("start_sin", 0xFF << 0), ("start_sin90", 0xFF << 16)] {
            f.insert(k.to_string(), v);
        }
        m.insert("MSLUTSTART".to_string(), f);
    }
    single("MSCNT", "mscnt", 0x3ff << 0, &mut m);
    {
        let mut f = IndexMap::new();
        for (k, v) in [("cur_a", 0x1ff << 0), ("cur_b", 0x1ff << 16)] {
            f.insert(k.to_string(), v);
        }
        m.insert("MSCURACT".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("otp_fclktrim", 0x1f << 0),
            ("otp_s2_level", 0x01 << 5),
            ("otp_bbm", 0x01 << 6),
            ("otp_tbl", 0x01 << 7),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("OTP_READ".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [("pwm_ofs_auto", 0xff << 0), ("pwm_grad_auto", 0xff << 16)] {
            f.insert(k.to_string(), v);
        }
        m.insert("PWM_AUTO".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [
            ("pwm_ofs", 0xFF << 0),
            ("pwm_grad", 0xFF << 8),
            ("pwm_freq", 0x03 << 16),
            ("pwm_autoscale", 0x01 << 18),
            ("pwm_autograd", 0x01 << 19),
            ("freewheel", 0x03 << 20),
            ("pwm_reg", 0x0F << 24),
            ("pwm_lim", 0x0F << 28),
        ] {
            f.insert(k.to_string(), v);
        }
        m.insert("PWMCONF".to_string(), f);
    }
    {
        let mut f = IndexMap::new();
        for (k, v) in [("pwm_scale_sum", 0xff << 0), ("pwm_scale_auto", 0x1ff << 16)] {
            f.insert(k.to_string(), v);
        }
        m.insert("PWM_SCALE".to_string(), f);
    }
    single("TPOWERDOWN", "tpowerdown", 0xff << 0, &mut m);
    single("TPWMTHRS", "tpwmthrs", 0xfffff << 0, &mut m);
    single("TCOOLTHRS", "tcoolthrs", 0xfffff << 0, &mut m);
    single("TSTEP", "tstep", 0xfffff << 0, &mut m);
    single("THIGH", "thigh", 0xfffff << 0, &mut m);
    m
}

/// `SignedFields` (tmc5160.py:249).
pub const SIGNED_FIELDS: &[&str] =
    &["cur_a", "cur_b", "sgt", "xactual", "vactual", "pwm_scale_auto"];

/// `FieldFormatters` (tmc5160.py:250-255): `tmc2130.FieldFormatters`
/// (`tmc2130.py:97-112`) updated with the `s2vsa`/`s2vsb` entries of
/// `tmc5160.py:252-255`.
pub fn field_formatters() -> IndexMap<String, FieldFormatter> {
    fn yes(text: &'static str) -> FieldFormatter {
        Arc::new(move |v: i64| {
            if v != 0 { text.to_string() } else { String::new() }
        })
    }
    let entries: Vec<(&str, FieldFormatter)> = vec![
        ("i_scale_analog", yes("1(ExtVREF)")),
        ("shaft", yes("1(Reverse)")),
        ("reset", yes("1(Reset)")),
        ("drv_err", yes("1(ErrorShutdown!)")),
        ("uv_cp", yes("1(Undervoltage!)")),
        ("version", Arc::new(|v: i64| format!("{:#x}", v))),
        ("mres", Arc::new(|v: i64| format!("{}({}usteps)", v, 0x100 >> v))),
        ("otpw", yes("1(OvertempWarning!)")),
        ("ot", yes("1(OvertempError!)")),
        ("s2ga", yes("1(ShortToGND_A!)")),
        ("s2gb", yes("1(ShortToGND_B!)")),
        ("ola", yes("1(OpenLoad_A!)")),
        ("olb", yes("1(OpenLoad_B!)")),
        ("cs_actual", Arc::new(|v: i64| {
            if v != 0 { v.to_string() } else { "0(Reset?)".to_string() }
        })),
        ("s2vsa", yes("1(ShortToSupply_A!)")),
        ("s2vsb", yes("1(ShortToSupply_B!)")),
    ];
    let mut m: IndexMap<String, FieldFormatter> = IndexMap::new();
    for (name, f) in entries {
        m.insert(name.to_string(), f);
    }
    m
}

/// The `tmc.FieldHelper` of `tmc5160.py:328`, built from these tables
/// (`tmc.FieldHelper(Fields, SignedFields, FieldFormatters)`).
pub fn make_field_helper() -> Arc<FieldHelper> {
    FieldHelper::new(
        fields_table(),
        SIGNED_FIELDS.iter().map(|s| s.to_string()).collect(),
        field_formatters(),
        None,
    )
}
fn unhandled(e: KlippyError) -> ! {
    // An exception the reference does not catch unwinds the reactor,
    // which ends the process with a traceback.
    panic!("{}: {}", e.class_name(), e.message())
}

// ---------------------------------------------------------------------------
// Current helper (tmc5160.py:265-318)
// ---------------------------------------------------------------------------

fn py_int_from_float(f: f64) -> KResult<i64> {
    // Python `int(float)`: OverflowError for inf, ValueError for nan
    // (CPython 3.8 Objects/abstract.c:PyNumber_Long).
    if f.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    if f.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    Ok(f.trunc() as i64)
}

/// Python `min(a, b)`: returns `b` only when `b < a`, so a NaN in the
/// first argument stays (`min(nan, x)` is `nan`).
fn py_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// Port of `TMC5160CurrentHelper` (tmc5160.py:265-318).
pub struct Tmc5160CurrentHelper {
    pub printer: Printer,
    pub name: String,
    pub sense_resistor: f64,
    req_hold_current: GCell<f64>,
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<dyn McuTmcApi>,
}

impl Tmc5160CurrentHelper {
    /// `__init__(config, mcu_tmc)` (tmc5160.py:266-280).
    pub fn new(config: &Config, mcu_tmc: Arc<dyn McuTmcApi>) -> KResult<Self> {
        let printer = config.get_printer();
        let run_current = config
            .getfloat(
                "run_current",
                GetFloatOpts {
                    default: None,
                    minval: None,
                    maxval: Some(MAX_CURRENT),
                    above: Some(0.),
                    below: None,
                    note_valid: true,
                },
            )?
            // `configfile.py` raises this itself when the option is absent;
            // the same message is used here so a `None` cannot diverge.
            .ok_or_else(|| {
                KlippyError::config(format!(
                    "Option 'run_current' in section '{}' must be specified",
                    config.get_name()
                ))
            })?;
        let hold_current = config
            .getfloat(
                "hold_current",
                GetFloatOpts {
                    default: Some(Some(MAX_CURRENT)),
                    minval: None,
                    maxval: Some(MAX_CURRENT),
                    above: Some(0.),
                    below: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(MAX_CURRENT);
        let sense_resistor = config
            .getfloat(
                "sense_resistor",
                GetFloatOpts {
                    default: Some(Some(0.075)),
                    minval: None,
                    maxval: None,
                    above: Some(0.),
                    below: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(0.075);
        let name = config.get_name();
        let name = name.split_whitespace().last().unwrap_or("").to_string();
        let fields = mcu_tmc.get_fields();
        let helper = Tmc5160CurrentHelper {
            printer,
            name,
            sense_resistor,
            req_hold_current: GCell::new(hold_current),
            fields: fields.clone(),
            mcu_tmc,
        };
        let (gscaler, irun, ihold) = helper.calc_current(run_current, hold_current)?;
        fields.set_field("globalscaler", gscaler, None, None)?;
        fields.set_field("ihold", ihold, None, None)?;
        fields.set_field("irun", irun, None, None)?;
        Ok(helper)
    }

    /// `_calc_globalscaler(current)` (tmc5160.py:281-287).
    pub fn calc_globalscaler(&self, current: f64) -> KResult<i64> {
        let v = current * 256. * sqrt(2.) * self.sense_resistor / VREF + 0.5;
        let mut globalscaler = py_int_from_float(v)?;
        if globalscaler < 32 {
            globalscaler = 32;
        }
        // `globalscaler = max(32, globalscaler)` above; Python then maps
        // 256 and above to 0 (the chip takes the encoding literally).
        if globalscaler >= 256 {
            globalscaler = 0;
        }
        Ok(globalscaler)
    }

    /// `_calc_current_bits(current, globalscaler)` (tmc5160.py:288-294).
    pub fn calc_current_bits(&self, current: f64, globalscaler: i64) -> KResult<i64> {
        let gs = if globalscaler == 0 { 256 } else { globalscaler };
        let v = current * 256. * 32. * sqrt(2.) * self.sense_resistor
            / (gs as f64 * VREF)
            - 1.
            + 0.5;
        let cs = py_int_from_float(v)?;
        // `max(0, min(31, cs))`
        Ok(cs.clamp(0, 31))
    }

    /// `_calc_current(run_current, hold_current)` (tmc5160.py:295-299).
    pub fn calc_current(&self, run_current: f64, hold_current: f64) -> KResult<(i64, i64, i64)> {
        let gscaler = self.calc_globalscaler(run_current)?;
        let irun = self.calc_current_bits(run_current, gscaler)?;
        let ihold = self.calc_current_bits(py_min(hold_current, run_current), gscaler)?;
        Ok((gscaler, irun, ihold))
    }

    /// `_calc_current_from_field(field_name)` (tmc5160.py:300-306).
    pub fn calc_current_from_field(&self, field_name: &str) -> KResult<f64> {
        let mut globalscaler = self.fields.get_field("globalscaler", None, None)?;
        if globalscaler == 0 {
            globalscaler = 256;
        }
        let bits = self.fields.get_field(field_name, None, None)?;
        // The numerator is computed as an exact integer product before the
        // float multiply, matching `(globalscaler * (bits + 1) * VREF)`.
        let num = (globalscaler.wrapping_mul(bits + 1)) as f64 * VREF;
        let den = 256. * 32. * sqrt(2.) * self.sense_resistor;
        Ok(num / den)
    }

    /// `req_hold_current` (tmc5160.py:274).
    pub fn req_hold_current(&self) -> f64 {
        self.req_hold_current.get()
    }

    /// `get_current()` (tmc5160.py:307-310).
    pub fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        // `get_field` of a known field of this helper never fails; the
        // Python method likewise unwraps it via `self.fields`.
        let run_current = self.calc_current_from_field("irun").unwrap_or_else(|e| unhandled(e));
        let hold_current = self.calc_current_from_field("ihold").unwrap_or_else(|e| unhandled(e));
        (run_current, Some(hold_current), Some(self.req_hold_current()), MAX_CURRENT)
    }

    /// `set_current(run_current, hold_current, print_time)` (tmc5160.py:311-318).
    pub fn set_current(&self, run_current: f64, hold_current: f64, print_time: f64) -> KResult<()> {
        self.req_hold_current.set(hold_current);
        let (gscaler, irun, ihold) = self.calc_current(run_current, hold_current)?;
        let val = self.fields.set_field("globalscaler", gscaler, None, None)?;
        self.mcu_tmc.set_register("GLOBALSCALER", val, Some(print_time))?;
        self.fields.set_field("ihold", ihold, None, None)?;
        let val = self.fields.set_field("irun", irun, None, None)?;
        self.mcu_tmc.set_register("IHOLD_IRUN", val, Some(print_time))?;
        Ok(())
    }
}

impl TmcCurrentHelperApi for Tmc5160CurrentHelper {
    fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        Tmc5160CurrentHelper::get_current(self)
    }

    fn set_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()> {
        // The Python `set_current` requires a hold current (it is always
        // a float for this helper: `req_hold_current` is set from the
        // config default); `_calc_current` would then evaluate
        // `min(hold_current, run_current)` with a `None` left operand,
        // which CPython rejects comparing `run_current < None`.
        match hold_current {
            Some(hold) => Tmc5160CurrentHelper::set_current(self, run_current, hold, print_time),
            None => Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'<' not supported between instances of 'float' and 'NoneType'"
                    .to_string(),
            }),
        }
    }
}

// ---------------------------------------------------------------------------
// Printer object (tmc5160.py:325-387)
// ---------------------------------------------------------------------------

/// Port of `TMC5160` (tmc5160.py:325-387).
pub struct Tmc5160 {
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<dyn McuTmcApi>,
    cmdhelper: Arc<TMCCommandHelper>,
    current_helper: Arc<Tmc5160CurrentHelper>,
}

impl Tmc5160 {
    /// `TMC5160.__init__(config)` (tmc5160.py:325-387): builds the
    /// field helper and the SPI transport
    /// (`tmc2130.MCU_TMC_SPI(config, Registers, fields, TMC_FREQUENCY)`,
    /// `tmc5160.py:329-330`), then runs the same wiring as [`assemble`].
    pub fn new(config: &Config) -> KResult<Tmc5160> {
        let fields = make_field_helper();
        let mcu_tmc = McuTmcSpi::new(config, registers_table(), fields, TMC_FREQUENCY)?;
        Self::assemble(config, Arc::new(mcu_tmc))
    }

    /// The rest of `__init__`, with the MCU communication already built so
    /// tests can drive the same wiring through a fake `McuTmcApi`.
    ///
    /// The wiring order is the Python one: virtual pins
    /// (`tmc5160.py:332`), current helper (`:334`), command helper with
    /// the register dump (`:335-338`), wave table / stealthChop /
    /// CoolStep / high-velocity thresholds (`:340-343`), then the
    /// `set_config_field` defaults of `tmc5160.py:345-386`.
    pub fn assemble(config: &Config, mcu_tmc: Arc<dyn McuTmcApi>) -> KResult<Tmc5160> {
        let fields = mcu_tmc.get_fields();
        // Allow virtual pins to be created; the pin registry keeps the
        // helper alive, as in the reference (`tmc5160.py:332`).
        TMCVirtualPinHelper::new(config, mcu_tmc.clone())?;
        let current_helper = Arc::new(Tmc5160CurrentHelper::new(config, mcu_tmc.clone())?);
        let cmdhelper = TMCCommandHelper::new(
            config,
            mcu_tmc.clone(),
            current_helper.clone() as Arc<dyn TmcCurrentHelperApi>,
        )?;
        cmdhelper.setup_register_dump(
            READ_REGISTERS.iter().map(|r| r.to_string()).collect(),
            None,
        )?;
        tmc_wave_table_helper(config, &*mcu_tmc)?;
        tmc_stealthchop_helper(config, &*mcu_tmc)?;
        tmc_vcoolthrs_helper(config, &*mcu_tmc)?;
        tmc_vhigh_helper(config, &*mcu_tmc)?;
        let defaults: [(&str, ConfigFieldDefault); 34] = [
            ("multistep_filt", ConfigFieldDefault::Bool(true)),
            ("toff", ConfigFieldDefault::Int(3)),
            ("hstrt", ConfigFieldDefault::Int(5)),
            ("hend", ConfigFieldDefault::Int(2)),
            ("fd3", ConfigFieldDefault::Int(0)),
            ("disfdcc", ConfigFieldDefault::Int(0)),
            ("chm", ConfigFieldDefault::Int(0)),
            ("tbl", ConfigFieldDefault::Int(2)),
            ("vhighfs", ConfigFieldDefault::Int(0)),
            ("vhighchm", ConfigFieldDefault::Int(0)),
            ("tpfd", ConfigFieldDefault::Int(4)),
            ("diss2g", ConfigFieldDefault::Int(0)),
            ("diss2vs", ConfigFieldDefault::Int(0)),
            ("semin", ConfigFieldDefault::Int(0)),
            ("seup", ConfigFieldDefault::Int(0)),
            ("semax", ConfigFieldDefault::Int(0)),
            ("sedn", ConfigFieldDefault::Int(0)),
            ("seimin", ConfigFieldDefault::Int(0)),
            ("sgt", ConfigFieldDefault::Int(0)),
            ("sfilt", ConfigFieldDefault::Int(0)),
            ("drvstrength", ConfigFieldDefault::Int(0)),
            ("bbmclks", ConfigFieldDefault::Int(4)),
            ("bbmtime", ConfigFieldDefault::Int(0)),
            ("filt_isense", ConfigFieldDefault::Int(0)),
            ("iholddelay", ConfigFieldDefault::Int(6)),
            ("pwm_ofs", ConfigFieldDefault::Int(30)),
            ("pwm_grad", ConfigFieldDefault::Int(0)),
            ("pwm_freq", ConfigFieldDefault::Int(0)),
            ("pwm_autoscale", ConfigFieldDefault::Bool(true)),
            ("pwm_autograd", ConfigFieldDefault::Bool(true)),
            ("freewheel", ConfigFieldDefault::Int(0)),
            ("pwm_reg", ConfigFieldDefault::Int(4)),
            ("pwm_lim", ConfigFieldDefault::Int(12)),
            ("tpowerdown", ConfigFieldDefault::Int(10)),
        ];
        for (field, default) in &defaults {
            fields.set_config_field(config, field, *default)?;
        }
        Ok(Tmc5160 { fields, mcu_tmc, cmdhelper, current_helper })
    }

    /// The register helper of this driver (`self.fields`).
    pub fn fields(&self) -> &Arc<FieldHelper> {
        &self.fields
    }

    /// The TMC current helper built in `__init__`
    /// (`tmc5160.py:334`); `SET_TMC_CURRENT` and `get_status` reach it
    /// through the command helper.
    pub fn current_helper(&self) -> &Arc<Tmc5160CurrentHelper> {
        &self.current_helper
    }

    /// The MCU communication this driver was built with.
    pub fn mcu_tmc(&self) -> &Arc<dyn McuTmcApi> {
        &self.mcu_tmc
    }
}

impl PrinterObject for Tmc5160 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);

    /// `get_status` is the command helper's (`tmc5160.py:338`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.cmdhelper.get_status(Some(eventtime)))
    }
}

impl TMCCommandHelperApi for Tmc5160 {
    /// `get_phase_offset` is the command helper's (`tmc5160.py:337`).
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmdhelper.get_phase_offset()
    }
}

/// `load_config_prefix(config)` (tmc5160.py:388-389).
pub fn load_config_prefix(config: &Config) -> KResult<Option<Arc<dyn PrinterObject>>> {
    Ok(Some(Arc::new(Tmc5160::new(config)?)))
}

/// Module factory for section loading (multi-token sections).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config_prefix: Some(load_config_prefix), ..Default::default() }
}
