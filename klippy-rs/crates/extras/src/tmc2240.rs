//! Port of `klippy/extras/tmc2240.py` (TMC2240 configuration).
//!
//! The module owns the TMC2240 register/field tables, `TMC2240CurrentHelper`
//! and the `TMC2240` printer object. The helpers it composes belong to their
//! owners and are imported: `tmc.FieldHelper`, `TMCVirtualPinHelper`,
//! `TMCCommandHelper` and the config helpers from `crate::tmc`,
//! `MCU_TMC_SPI` from `crate::tmc2130` and `MCU_TMC_uart` from
//! `crate::tmc_uart`.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::printer::ModuleFactory;
use klippy_api::{GCell, KResult, KlippyError, ObjectRef, PrinterObject};
use pycompat::libm;
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

use crate::tmc::{
    tmc_stealthchop_helper, tmc_vcoolthrs_helper, tmc_vhigh_helper, tmc_wave_table_helper,
    ConfigFieldDefault, FieldFormatter, FieldHelper, McuTmcApi, TMCCommandHelper,
    TMCCommandHelperApi, TMCVirtualPinHelper, TmcCurrentHelperApi,
};
use crate::tmc2130::{self, McuTmcSpi};
use crate::tmc_uart::McuTmcUart;

/// `TMC_FREQUENCY=12500000.` (tmc2240.py:10).
pub const TMC_FREQUENCY: f64 = 12500000.;

/// `Registers` (tmc2240.py:12-56).
const REGISTERS: &[(&str, i64)] = &[
    ("GCONF", 0x00),
    ("GSTAT", 0x01),
    ("IFCNT", 0x02),
    ("NODECONF", 0x03),
    ("IOIN", 0x04),
    ("DRV_CONF", 0x0A),
    ("GLOBALSCALER", 0x0B),
    ("IHOLD_IRUN", 0x10),
    ("TPOWERDOWN", 0x11),
    ("TSTEP", 0x12),
    ("TPWMTHRS", 0x13),
    ("TCOOLTHRS", 0x14),
    ("THIGH", 0x15),
    ("DIRECT_MODE", 0x2D),
    ("ENCMODE", 0x38),
    ("X_ENC", 0x39),
    ("ENC_CONST", 0x3A),
    ("ENC_STATUS", 0x3B),
    ("ENC_LATCH", 0x3C),
    ("ADC_VSUPPLY_AIN", 0x50),
    ("ADC_TEMP", 0x51),
    ("OTW_OV_VTH", 0x52),
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
    ("DRV_STATUS", 0x6F),
    ("PWMCONF", 0x70),
    ("PWM_SCALE", 0x71),
    ("PWM_AUTO", 0x72),
    ("SG4_THRS", 0x74),
    ("SG4_RESULT", 0x75),
    ("SG4_IND", 0x76),
];

/// `ReadRegisters` (tmc2240.py:58-64).
const READ_REGISTERS: &[&str] = &[
    "GCONF",
    "GSTAT",
    "IOIN",
    "DRV_CONF",
    "GLOBALSCALER",
    "IHOLD_IRUN",
    "TPOWERDOWN",
    "TSTEP",
    "TPWMTHRS",
    "TCOOLTHRS",
    "THIGH",
    "ADC_VSUPPLY_AIN",
    "ADC_TEMP",
    "OTW_OV_VTH",
    "MSCNT",
    "MSCURACT",
    "CHOPCONF",
    "COOLCONF",
    "DRV_STATUS",
    "PWMCONF",
    "PWM_SCALE",
    "PWM_AUTO",
    "SG4_THRS",
    "SG4_RESULT",
    "SG4_IND",
];

/// `Fields` (tmc2240.py:66-253), in the Python insertion order.
/// The `<< 0` shifts mirror the Python source.
#[allow(clippy::identity_op)]
const FIELDS: &[(&str, &[(&str, i64)])] = &[
    (
        "COOLCONF",
        &[
            ("semin", 0x0F << 0),
            ("seup", 0x03 << 5),
            ("semax", 0x0F << 8),
            ("sedn", 0x03 << 13),
            ("seimin", 0x01 << 15),
            ("sgt", 0x7F << 16),
            ("sfilt", 0x01 << 24),
        ],
    ),
    (
        "CHOPCONF",
        &[
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
        ],
    ),
    (
        "DRV_STATUS",
        &[
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
        ],
    ),
    (
        "GCONF",
        &[
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
            ("diag0_pushpull", 0x01 << 12),
            ("diag1_pushpull", 0x01 << 13),
            ("small_hysteresis", 0x01 << 14),
            ("stop_enable", 0x01 << 15),
            ("direct_mode", 0x01 << 16),
        ],
    ),
    (
        "GSTAT",
        &[
            ("reset", 0x01 << 0),
            ("drv_err", 0x01 << 1),
            ("uv_cp", 0x01 << 2),
            ("register_reset", 0x01 << 3),
            ("vm_uvlo", 0x01 << 4),
        ],
    ),
    ("GLOBALSCALER", &[("globalscaler", 0xFF << 0)]),
    (
        "IHOLD_IRUN",
        &[
            ("ihold", 0x1F << 0),
            ("irun", 0x1F << 8),
            ("iholddelay", 0x0F << 16),
            ("irundelay", 0x0F << 24),
        ],
    ),
    (
        "IOIN",
        &[
            ("step", 0x01 << 0),
            ("dir", 0x01 << 1),
            ("encb", 0x01 << 2),
            ("enca", 0x01 << 3),
            ("drv_enn", 0x01 << 4),
            ("encn", 0x01 << 5),
            ("uart_en", 0x01 << 6),
            ("comp_a", 0x01 << 8),
            ("comp_b", 0x01 << 9),
            ("comp_a1_a2", 0x01 << 10),
            ("comp_b1_b2", 0x01 << 11),
            ("output", 0x01 << 12),
            ("ext_res_det", 0x01 << 13),
            ("ext_clk", 0x01 << 14),
            ("adc_err", 0x01 << 15),
            ("silicon_rv", 0x07 << 16),
            ("version", 0xFF << 24),
        ],
    ),
    ("MSLUT0", &[("mslut0", 0xffffffff)]),
    ("MSLUT1", &[("mslut1", 0xffffffff)]),
    ("MSLUT2", &[("mslut2", 0xffffffff)]),
    ("MSLUT3", &[("mslut3", 0xffffffff)]),
    ("MSLUT4", &[("mslut4", 0xffffffff)]),
    ("MSLUT5", &[("mslut5", 0xffffffff)]),
    ("MSLUT6", &[("mslut6", 0xffffffff)]),
    ("MSLUT7", &[("mslut7", 0xffffffff)]),
    (
        "MSLUTSEL",
        &[
            ("x3", 0xFF << 24),
            ("x2", 0xFF << 16),
            ("x1", 0xFF << 8),
            ("w3", 0x03 << 6),
            ("w2", 0x03 << 4),
            ("w1", 0x03 << 2),
            ("w0", 0x03 << 0),
        ],
    ),
    (
        "MSLUTSTART",
        &[
            ("start_sin", 0xFF << 0),
            ("start_sin90", 0xFF << 16),
            ("offset_sin90", 0xFF << 24),
        ],
    ),
    ("MSCNT", &[("mscnt", 0x3ff << 0)]),
    ("MSCURACT", &[("cur_a", 0x1ff << 0), ("cur_b", 0x1ff << 16)]),
    (
        "PWM_AUTO",
        &[("pwm_ofs_auto", 0xff << 0), ("pwm_grad_auto", 0xff << 16)],
    ),
    (
        "PWMCONF",
        &[
            ("pwm_ofs", 0xFF << 0),
            ("pwm_grad", 0xFF << 8),
            ("pwm_freq", 0x03 << 16),
            ("pwm_autoscale", 0x01 << 18),
            ("pwm_autograd", 0x01 << 19),
            ("freewheel", 0x03 << 20),
            ("pwm_meas_sd_enable", 0x01 << 22),
            ("pwm_dis_reg_stst", 0x01 << 23),
            ("pwm_reg", 0x0F << 24),
            ("pwm_lim", 0x0F << 28),
        ],
    ),
    (
        "PWM_SCALE",
        &[("pwm_scale_sum", 0x3ff << 0), ("pwm_scale_auto", 0x1ff << 16)],
    ),
    ("TPOWERDOWN", &[("tpowerdown", 0xff << 0)]),
    ("TPWMTHRS", &[("tpwmthrs", 0xfffff << 0)]),
    ("TCOOLTHRS", &[("tcoolthrs", 0xfffff << 0)]),
    ("TSTEP", &[("tstep", 0xfffff << 0)]),
    ("THIGH", &[("thigh", 0xfffff << 0)]),
    (
        "DRV_CONF",
        &[("current_range", 0x03 << 0), ("slope_control", 0x03 << 4)],
    ),
    (
        "ADC_VSUPPLY_AIN",
        &[("adc_vsupply", 0x1fff << 0), ("adc_ain", 0x1fff << 16)],
    ),
    ("ADC_TEMP", &[("adc_temp", 0x1fff << 0)]),
    (
        "OTW_OV_VTH",
        &[
            ("overvoltage_vth", 0x1fff << 0),
            ("overtempprewarning_vth", 0x1fff << 16),
        ],
    ),
    (
        "SG4_THRS",
        &[
            ("sg4_thrs", 0xFF << 0),
            ("sg4_filt_en", 0x01 << 8),
            ("sg4_angle_offset", 0x01 << 9),
        ],
    ),
    ("SG4_RESULT", &[("sg4_result", 0x3FF << 0)]),
    (
        "SG4_IND",
        &[
            ("sg4_ind_0", 0xFF << 0),
            ("sg4_ind_1", 0xFF << 8),
            ("sg4_ind_2", 0xFF << 16),
            ("sg4_ind_3", 0xFF << 24),
        ],
    ),
];

/// `SignedFields` (tmc2240.py:256).
const SIGNED_FIELDS: &[&str] = &["cur_a", "cur_b", "sgt", "pwm_scale_auto", "offset_sin90"];

/// `Registers` as the `name_to_reg` dict handed to the transports.
pub fn registers() -> IndexMap<String, i64> {
    REGISTERS
        .iter()
        .map(|(name, reg)| ((*name).to_string(), *reg))
        .collect()
}

/// `ReadRegisters`.
pub fn read_registers() -> Vec<String> {
    READ_REGISTERS.iter().map(|name| (*name).to_string()).collect()
}

/// `Fields`: register name to field name to mask.
pub fn fields() -> IndexMap<String, IndexMap<String, i64>> {
    FIELDS
        .iter()
        .map(|(reg, reg_fields)| {
            (
                (*reg).to_string(),
                reg_fields
                    .iter()
                    .map(|(field, mask)| ((*field).to_string(), *mask))
                    .collect(),
            )
        })
        .collect()
}

/// `SignedFields`.
pub fn signed_fields() -> Vec<String> {
    SIGNED_FIELDS.iter().map(|name| (*name).to_string()).collect()
}

fn format_pct(fmt: &str, args: &[PctValue]) -> String {
    // The formats are constants and the arguments always an int and a float.
    percent_format(fmt, args).expect("constant tmc2240 field formatter")
}

fn temp_formatter(v: i64) -> String {
    // "0x%04x(%.1fC)" % (v, ((v - 2038) / 7.7))
    format_pct(
        "0x%04x(%.1fC)",
        &[
            PctValue::Int(PyInt::from_i64(v)),
            PctValue::Float((v - 2038) as f64 / 7.7),
        ],
    )
}

fn volt_formatter(v: i64) -> String {
    // "0x%04x(%.3fV)" % (v, v * 0.009732)
    format_pct(
        "0x%04x(%.3fV)",
        &[
            PctValue::Int(PyInt::from_i64(v)),
            PctValue::Float(v as f64 * 0.009732),
        ],
    )
}

/// `FieldFormatters` (tmc2240.py:258-268): `dict(tmc2130.FieldFormatters)`
/// plus this chip's entries.
pub fn field_formatters() -> IndexMap<String, FieldFormatter> {
    let mut formatters = tmc2130::field_formatters();
    formatters.insert(
        "s2vsa".to_string(),
        Arc::new(|v: i64| {
            if v != 0 { "1(ShortToSupply_A!)".to_string() } else { String::new() }
        }),
    );
    formatters.insert(
        "s2vsb".to_string(),
        Arc::new(|v: i64| {
            if v != 0 { "1(ShortToSupply_B!)".to_string() } else { String::new() }
        }),
    );
    formatters.insert("adc_temp".to_string(), Arc::new(temp_formatter));
    formatters.insert("adc_vsupply".to_string(), Arc::new(volt_formatter));
    formatters.insert(
        "adc_ain".to_string(),
        Arc::new(|v: i64| {
            // "0x%04x(%.3fmV)" % (v, v * 0.3052)
            format_pct(
                "0x%04x(%.3fmV)",
                &[
                    PctValue::Int(PyInt::from_i64(v)),
                    PctValue::Float(v as f64 * 0.3052),
                ],
            )
        }),
    );
    formatters.insert("overvoltage_vth".to_string(), Arc::new(volt_formatter));
    formatters.insert("overtempprewarning_vth".to_string(), Arc::new(temp_formatter));
    formatters
}

/// Python `int(float)`: truncation toward zero, `ValueError` for NaN and
/// `OverflowError` for infinities. Values beyond `i64` saturate, which no
/// caller can distinguish from the exact big integer (every caller clamps).
fn py_int(value: f64) -> KResult<i64> {
    if value.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if value.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    let saturated = if value < 0.0 { i64::MIN } else { i64::MAX };
    Ok(trunc_float_to_bigint(value)
        .0
        .to_string()
        .parse::<i64>()
        .unwrap_or(saturated))
}

/// `TMC2240CurrentHelper` (tmc2240.py:275-341).
pub struct TMC2240CurrentHelper {
    mcu_tmc: Arc<dyn McuTmcApi>,
    fields: Arc<FieldHelper>,
    rref: f64,
    req_hold_current: GCell<Option<f64>>,
}

impl TMC2240CurrentHelper {
    /// `TMC2240CurrentHelper(config, mcu_tmc)` (tmc2240.py:276-293).
    pub fn new(config: &Config, mcu_tmc: Arc<dyn McuTmcApi>) -> KResult<TMC2240CurrentHelper> {
        let fields = mcu_tmc.get_fields();
        let rref = config
            .getfloat(
                "rref",
                GetFloatOpts {
                    default: Some(Some(12000.)),
                    minval: Some(12000.),
                    maxval: Some(60000.),
                    ..Default::default()
                },
            )?
            .ok_or_else(|| required_option(config, "rref"))?;
        let helper = TMC2240CurrentHelper {
            mcu_tmc,
            fields,
            rref,
            req_hold_current: GCell::new(None),
        };
        let max_cur = helper.get_ifs_rms(Some(3))?;
        let run_current = config
            .getfloat(
                "run_current",
                GetFloatOpts {
                    default: None,
                    above: Some(0.),
                    maxval: Some(max_cur),
                    ..Default::default()
                },
            )?
            .ok_or_else(|| required_option(config, "run_current"))?;
        let hold_current = config
            .getfloat(
                "hold_current",
                GetFloatOpts {
                    default: Some(Some(max_cur)),
                    above: Some(0.),
                    maxval: Some(max_cur),
                    ..Default::default()
                },
            )?
            .ok_or_else(|| required_option(config, "hold_current"))?;
        helper.req_hold_current.set(Some(hold_current));
        let current_range = helper.calc_current_range(run_current)?;
        helper.fields.set_field("current_range", current_range, None, None)?;
        let (gscaler, irun, ihold) = helper.calc_current(run_current, Some(hold_current))?;
        helper.fields.set_field("globalscaler", gscaler, None, None)?;
        helper.fields.set_field("ihold", ihold, None, None)?;
        helper.fields.set_field("irun", irun, None, None)?;
        Ok(helper)
    }

    /// `Rref` (tmc2240.py:281).
    pub fn rref(&self) -> f64 {
        self.rref
    }

    /// `req_hold_current` (tmc2240.py:287).
    pub fn req_hold_current(&self) -> Option<f64> {
        self.req_hold_current.get()
    }

    /// `_get_ifs_rms(current_range=None)` (tmc2240.py:294-298).
    pub fn get_ifs_rms(&self, current_range: Option<i64>) -> KResult<f64> {
        let current_range = match current_range {
            Some(range) => range,
            None => self.fields.get_field("current_range", None, None)?,
        };
        const KIFS: [f64; 4] = [11750., 24000., 36000., 36000.];
        Ok((KIFS[index_of(current_range)?] / self.rref) / libm::sqrt(2.))
    }

    /// `_calc_current_range(current)` (tmc2240.py:299-303). The `for` loop
    /// leaves `current_range == 3` when no range fits.
    pub fn calc_current_range(&self, current: f64) -> KResult<i64> {
        for current_range in 0..4 {
            if current <= self.get_ifs_rms(Some(current_range))? {
                return Ok(current_range);
            }
        }
        Ok(3)
    }

    /// `_calc_globalscaler(current)` (tmc2240.py:304-310).
    pub fn calc_globalscaler(&self, current: f64) -> KResult<i64> {
        let ifs_rms = self.get_ifs_rms(None)?;
        let mut globalscaler = py_int(((current * 256.) / ifs_rms) + 0.5)?;
        globalscaler = globalscaler.max(32);
        if globalscaler >= 256 {
            globalscaler = 0;
        }
        Ok(globalscaler)
    }

    /// `_calc_current_bits(current, globalscaler)` (tmc2240.py:311-316).
    pub fn calc_current_bits(&self, current: f64, globalscaler: i64) -> KResult<i64> {
        let ifs_rms = self.get_ifs_rms(None)?;
        let globalscaler = if globalscaler == 0 { 256 } else { globalscaler };
        let cs = py_int((current * 256. * 32.) / (globalscaler as f64 * ifs_rms) - 1. + 0.5)?;
        Ok(cs.clamp(0, 31))
    }

    /// `_calc_current(run_current, hold_current)` (tmc2240.py:317-321).
    /// `hold_current` is `None` only when a caller passes Python `None`.
    pub fn calc_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
    ) -> KResult<(i64, i64, i64)> {
        let gscaler = self.calc_globalscaler(run_current)?;
        let irun = self.calc_current_bits(run_current, gscaler)?;
        // Python `min(hold_current, run_current)` keeps the first argument
        // unless the second is smaller; `run_current < None` is a TypeError.
        let hold = match hold_current {
            Some(hold_current) => {
                if run_current < hold_current { run_current } else { hold_current }
            }
            None => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "'<' not supported between instances of 'float' and 'NoneType'"
                        .to_string(),
                })
            }
        };
        let ihold = self.calc_current_bits(hold, gscaler)?;
        Ok((gscaler, irun, ihold))
    }

    /// `_calc_current_from_field(field_name)` (tmc2240.py:322-328).
    pub fn calc_current_from_field(&self, field_name: &str) -> KResult<f64> {
        let ifs_rms = self.get_ifs_rms(None)?;
        let mut globalscaler = self.fields.get_field("globalscaler", None, None)?;
        if globalscaler == 0 {
            globalscaler = 256;
        }
        let bits = self.fields.get_field(field_name, None, None)?;
        Ok((globalscaler * (bits + 1)) as f64 * ifs_rms / (256. * 32.))
    }

    fn current(&self) -> KResult<(f64, Option<f64>, Option<f64>, f64)> {
        let ifs_rms = self.get_ifs_rms(None)?;
        let run_current = self.calc_current_from_field("irun")?;
        let hold_current = self.calc_current_from_field("ihold")?;
        Ok((run_current, Some(hold_current), self.req_hold_current.get(), ifs_rms))
    }

    fn apply_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()> {
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

impl TmcCurrentHelperApi for TMC2240CurrentHelper {
    /// `get_current()` (tmc2240.py:329-333): `(run_current, hold_current,
    /// req_hold_current, max_current)`. Only fields of this module's own
    /// table are read, so the lookups cannot fail.
    fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        self.current().expect("tmc2240 current fields are in Fields")
    }

    /// `set_current(run_current, hold_current, print_time)` (tmc2240.py:334-341).
    fn set_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()> {
        self.apply_current(run_current, hold_current, print_time)
    }
}

fn required_option(config: &Config, option: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{}' in section '{}' must be specified",
        option,
        config.get_name()
    ))
}

/// Python list indexing by a field value (`KIFS[current_range]`). The
/// two-bit field always lands in `0..4`.
fn index_of(current_range: i64) -> KResult<usize> {
    usize::try_from(current_range)
        .ok()
        .filter(|index| *index < 4)
        .ok_or_else(|| KlippyError::Internal {
            class_name: "IndexError".to_string(),
            message: "list index out of range".to_string(),
        })
}

/// `TMC2240` (tmc2240.py:348-417).
pub struct TMC2240 {
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<dyn McuTmcApi>,
    cmdhelper: Arc<TMCCommandHelper>,
    _virtual_pin_helper: Arc<TMCVirtualPinHelper>,
}

impl TMC2240 {
    /// `TMC2240(config)` (tmc2240.py:349-417).
    pub fn new(config: &Config) -> KResult<TMC2240> {
        // Setup mcu communication
        let fields = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
        let uart_pin = config.get(
            "uart_pin",
            GetStrOpts {
                default: Some(None),
                ..Default::default()
            },
        )?;
        let mcu_tmc: Arc<dyn McuTmcApi> = if uart_pin.is_some() {
            // use UART for communication
            Arc::new(McuTmcUart::new(
                config,
                registers(),
                fields.clone(),
                7,
                TMC_FREQUENCY,
            )?)
        } else {
            // Use SPI bus for communication
            Arc::new(McuTmcSpi::new(
                config,
                registers(),
                fields.clone(),
                TMC_FREQUENCY,
            )?)
        };
        TMC2240::assemble(config, fields, mcu_tmc)
    }

    /// Everything after the transport is chosen (tmc2240.py:360-417), over
    /// the `FieldHelper` the transport was built with.
    pub fn assemble(
        config: &Config,
        fields: Arc<FieldHelper>,
        mcu_tmc: Arc<dyn McuTmcApi>,
    ) -> KResult<TMC2240> {
        // Allow virtual pins to be created
        let virtual_pin_helper = TMCVirtualPinHelper::new(config, mcu_tmc.clone())?;
        // Register commands
        let current_helper = Arc::new(TMC2240CurrentHelper::new(config, mcu_tmc.clone())?);
        let cmdhelper = TMCCommandHelper::new(
            config,
            mcu_tmc.clone(),
            current_helper as Arc<dyn TmcCurrentHelperApi>,
        )?;
        cmdhelper.setup_register_dump(read_registers(), None)?;
        // Setup basic register values
        tmc_wave_table_helper(config, &*mcu_tmc)?;
        fields.set_config_field(config, "offset_sin90", ConfigFieldDefault::Int(0))?;
        tmc_stealthchop_helper(config, &*mcu_tmc)?;
        tmc_vcoolthrs_helper(config, &*mcu_tmc)?;
        tmc_vhigh_helper(config, &*mcu_tmc)?;
        // Allow other registers to be set from the config
        let defaults: Vec<(&str, ConfigFieldDefault)> = vec![
            //   GCONF
            ("multistep_filt", ConfigFieldDefault::Bool(true)),
            //   CHOPCONF
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
            //   COOLCONF
            ("semin", ConfigFieldDefault::Int(0)),
            ("seup", ConfigFieldDefault::Int(0)),
            ("semax", ConfigFieldDefault::Int(0)),
            ("sedn", ConfigFieldDefault::Int(0)),
            ("seimin", ConfigFieldDefault::Int(0)),
            ("sgt", ConfigFieldDefault::Int(0)),
            ("sfilt", ConfigFieldDefault::Int(0)),
            //   IHOLDIRUN
            ("iholddelay", ConfigFieldDefault::Int(6)),
            ("irundelay", ConfigFieldDefault::Int(4)),
            //   PWMCONF
            ("pwm_ofs", ConfigFieldDefault::Int(29)),
            ("pwm_grad", ConfigFieldDefault::Int(0)),
            ("pwm_freq", ConfigFieldDefault::Int(0)),
            ("pwm_autoscale", ConfigFieldDefault::Bool(true)),
            ("pwm_autograd", ConfigFieldDefault::Bool(true)),
            ("freewheel", ConfigFieldDefault::Int(0)),
            ("pwm_reg", ConfigFieldDefault::Int(4)),
            ("pwm_lim", ConfigFieldDefault::Int(12)),
            //   TPOWERDOWN
            ("tpowerdown", ConfigFieldDefault::Int(10)),
            //   SG4_THRS
            ("sg4_thrs", ConfigFieldDefault::Int(0)),
            ("sg4_angle_offset", ConfigFieldDefault::Int(1)),
            //   DRV_CONF
            ("slope_control", ConfigFieldDefault::Int(0)),
        ];
        for (field_name, default) in defaults {
            fields.set_config_field(config, field_name, default)?;
        }
        Ok(TMC2240 {
            fields,
            mcu_tmc,
            cmdhelper,
            _virtual_pin_helper: virtual_pin_helper,
        })
    }

    /// `self.fields` (tmc2240.py:351).
    pub fn fields(&self) -> &Arc<FieldHelper> {
        &self.fields
    }

    /// `self.mcu_tmc` (tmc2240.py:354 / 358).
    pub fn mcu_tmc(&self) -> &Arc<dyn McuTmcApi> {
        &self.mcu_tmc
    }
}

impl TMCCommandHelperApi for TMC2240 {
    /// `self.get_phase_offset = cmdhelper.get_phase_offset` (tmc2240.py:366).
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmdhelper.get_phase_offset()
    }
}

impl PrinterObject for TMC2240 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);

    /// `self.get_status = cmdhelper.get_status` (tmc2240.py:367).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.cmdhelper.get_status(Some(eventtime)))
    }
}

/// `load_config_prefix(config)` (tmc2240.py:419-420).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(TMC2240::new(config)?) as ObjectRef))
}

/// The module has only `load_config_prefix` (tmc2240.py:419).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
