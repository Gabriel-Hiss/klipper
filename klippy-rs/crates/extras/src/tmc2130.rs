//! Port of `klippy/extras/tmc2130.py` (TMC2130 SPI configuration).
//!
//! Units: register/field tables, `TMCCurrentHelper`, `MCU_TMC_SPI_chain`,
//! `lookup_tmc_spi_chain`, `MCU_TMC_SPI`, `TMC2130`, `load_config_prefix`.
//!
//! Cross-module code is imported from its owner: the field register
//! helper and the G-code/virtual-pin/wave/stealth/velocity helpers from
//! `crate::tmc` (`klippy/extras/tmc.py`), the SPI bus from `crate::bus`
//! (`klippy/extras/bus.py`). Services are resolved inside
//! [`load_config_prefix`] as the reference does (no locally injected
//! handles).
//!
//! Contract gap (reported, not worked around): pin-parameter sharing of
//! the daisy-chain object (`cs_pin_params['class']` in
//! `lookup_tmc_spi_chain`, `tmc2130.py:250-251`) has no frozen
//! [`PinParams`](klippy_api::mcu::PinParams) slot, so
//! [`lookup_tmc_spi_chain`] cannot recover the chain a previous driver on the
//! same `cs_pin` built. The sharing logic itself is
//! [`lookup_tmc_spi_chain_with`], which takes the slot's value and builder;
//! production callers reach it through `lookup_tmc_spi_chain` with no
//! existing chain.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{LookupPinOpts, Mcu, PinsApi, QuerySendOpts, SendOpts};
use klippy_api::object::{ObjectRef, lookup_interface};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use pycompat::value::type_name;
use pycompat::PyValue;
use std::sync::Arc;

use crate::bus::{McuSpiApi, mcu_spi_from_config};
use crate::tmc::{
    ConfigFieldDefault, FieldHelper, McuTmcApi, RegisterRaw, TMCCommandHelper,
    TMCCommandHelperApi, TMCVirtualPinHelper,
};

pub const TMC_FREQUENCY: f64 = 13200000.0;
pub const MAX_CURRENT: f64 = 2.0;

// ---------------------------------------------------------------------------
// Register / field tables (`Registers`, `ReadRegisters`, `Fields`,
// `SignedFields`, `FieldFormatters`, `tmc2130.py:11-112`).
// ---------------------------------------------------------------------------

/// `Registers` (`tmc2130.py:11-20`), insertion ordered.
pub fn registers() -> IndexMap<String, i64> {
    let mut m = IndexMap::new();
    m.insert("GCONF".to_string(), 0x00);
    m.insert("GSTAT".to_string(), 0x01);
    m.insert("IOIN".to_string(), 0x04);
    m.insert("IHOLD_IRUN".to_string(), 0x10);
    m.insert("TPOWERDOWN".to_string(), 0x11);
    m.insert("TSTEP".to_string(), 0x12);
    m.insert("TPWMTHRS".to_string(), 0x13);
    m.insert("TCOOLTHRS".to_string(), 0x14);
    m.insert("THIGH".to_string(), 0x15);
    m.insert("XDIRECT".to_string(), 0x2d);
    m.insert("MSLUT0".to_string(), 0x60);
    m.insert("MSLUT1".to_string(), 0x61);
    m.insert("MSLUT2".to_string(), 0x62);
    m.insert("MSLUT3".to_string(), 0x63);
    m.insert("MSLUT4".to_string(), 0x64);
    m.insert("MSLUT5".to_string(), 0x65);
    m.insert("MSLUT6".to_string(), 0x66);
    m.insert("MSLUT7".to_string(), 0x67);
    m.insert("MSLUTSEL".to_string(), 0x68);
    m.insert("MSLUTSTART".to_string(), 0x69);
    m.insert("MSCNT".to_string(), 0x6a);
    m.insert("MSCURACT".to_string(), 0x6b);
    m.insert("CHOPCONF".to_string(), 0x6c);
    m.insert("COOLCONF".to_string(), 0x6d);
    m.insert("DCCTRL".to_string(), 0x6e);
    m.insert("DRV_STATUS".to_string(), 0x6f);
    m.insert("PWMCONF".to_string(), 0x70);
    m.insert("PWM_SCALE".to_string(), 0x71);
    m.insert("ENCM_CTRL".to_string(), 0x72);
    m.insert("LOST_STEPS".to_string(), 0x73);
    m
}

/// `ReadRegisters` (`tmc2130.py:22-25`).
pub fn read_registers() -> Vec<String> {
    vec![
        "GCONF".to_string(),
        "GSTAT".to_string(),
        "IOIN".to_string(),
        "TSTEP".to_string(),
        "XDIRECT".to_string(),
        "MSCNT".to_string(),
        "MSCURACT".to_string(),
        "CHOPCONF".to_string(),
        "DRV_STATUS".to_string(),
        "PWM_SCALE".to_string(),
        "LOST_STEPS".to_string(),
    ]
}

/// `Fields` (`tmc2130.py:27-93`), insertion ordered per register.
pub fn fields_table() -> IndexMap<String, IndexMap<String, i64>> {
    let mut all: IndexMap<String, IndexMap<String, i64>> = IndexMap::new();
    let mut gconf = IndexMap::new();
    gconf.insert("i_scale_analog".to_string(), 1 << 0);
    gconf.insert("internal_rsense".to_string(), 1 << 1);
    gconf.insert("en_pwm_mode".to_string(), 1 << 2);
    gconf.insert("enc_commutation".to_string(), 1 << 3);
    gconf.insert("shaft".to_string(), 1 << 4);
    gconf.insert("diag0_error".to_string(), 1 << 5);
    gconf.insert("diag0_otpw".to_string(), 1 << 6);
    gconf.insert("diag0_stall".to_string(), 1 << 7);
    gconf.insert("diag1_stall".to_string(), 1 << 8);
    gconf.insert("diag1_index".to_string(), 1 << 9);
    gconf.insert("diag1_onstate".to_string(), 1 << 10);
    gconf.insert("diag1_steps_skipped".to_string(), 1 << 11);
    gconf.insert("diag0_int_pushpull".to_string(), 1 << 12);
    gconf.insert("diag1_pushpull".to_string(), 1 << 13);
    gconf.insert("small_hysteresis".to_string(), 1 << 14);
    gconf.insert("stop_enable".to_string(), 1 << 15);
    gconf.insert("direct_mode".to_string(), 1 << 16);
    gconf.insert("test_mode".to_string(), 1 << 17);
    all.insert("GCONF".to_string(), gconf);
    let mut gstat = IndexMap::new();
    gstat.insert("reset".to_string(), 1 << 0);
    gstat.insert("drv_err".to_string(), 1 << 1);
    gstat.insert("uv_cp".to_string(), 1 << 2);
    all.insert("GSTAT".to_string(), gstat);
    let mut ioin = IndexMap::new();
    ioin.insert("step".to_string(), 1 << 0);
    ioin.insert("dir".to_string(), 1 << 1);
    ioin.insert("dcen_cfg4".to_string(), 1 << 2);
    ioin.insert("dcin_cfg5".to_string(), 1 << 3);
    ioin.insert("drv_enn_cfg6".to_string(), 1 << 4);
    ioin.insert("dco".to_string(), 1 << 5);
    ioin.insert("version".to_string(), 0xff << 24);
    all.insert("IOIN".to_string(), ioin);
    let mut ihold_irun = IndexMap::new();
    ihold_irun.insert("ihold".to_string(), 0x1f << 0);
    ihold_irun.insert("irun".to_string(), 0x1f << 8);
    ihold_irun.insert("iholddelay".to_string(), 0x0f << 16);
    all.insert("IHOLD_IRUN".to_string(), ihold_irun);
    all.insert("TPOWERDOWN".to_string(), single("tpowerdown", 0xff));
    all.insert("TSTEP".to_string(), single("tstep", 0xfffff));
    all.insert("TPWMTHRS".to_string(), single("tpwmthrs", 0xfffff));
    all.insert("TCOOLTHRS".to_string(), single("tcoolthrs", 0xfffff));
    all.insert("THIGH".to_string(), single("thigh", 0xfffff));
    all.insert("MSLUT0".to_string(), single("mslut0", 0xffffffff));
    all.insert("MSLUT1".to_string(), single("mslut1", 0xffffffff));
    all.insert("MSLUT2".to_string(), single("mslut2", 0xffffffff));
    all.insert("MSLUT3".to_string(), single("mslut3", 0xffffffff));
    all.insert("MSLUT4".to_string(), single("mslut4", 0xffffffff));
    all.insert("MSLUT5".to_string(), single("mslut5", 0xffffffff));
    all.insert("MSLUT6".to_string(), single("mslut6", 0xffffffff));
    all.insert("MSLUT7".to_string(), single("mslut7", 0xffffffff));
    let mut mslutsel = IndexMap::new();
    mslutsel.insert("x3".to_string(), 0xff << 24);
    mslutsel.insert("x2".to_string(), 0xff << 16);
    mslutsel.insert("x1".to_string(), 0xff << 8);
    mslutsel.insert("w3".to_string(), 0x03 << 6);
    mslutsel.insert("w2".to_string(), 0x03 << 4);
    mslutsel.insert("w1".to_string(), 0x03 << 2);
    mslutsel.insert("w0".to_string(), 0x03 << 0);
    all.insert("MSLUTSEL".to_string(), mslutsel);
    let mut mslutstart = IndexMap::new();
    mslutstart.insert("start_sin".to_string(), 0xff << 0);
    mslutstart.insert("start_sin90".to_string(), 0xff << 16);
    all.insert("MSLUTSTART".to_string(), mslutstart);
    all.insert("MSCNT".to_string(), single("mscnt", 0x3ff));
    let mut mscuract = IndexMap::new();
    mscuract.insert("cur_a".to_string(), 0x1ff);
    mscuract.insert("cur_b".to_string(), 0x1ff << 16);
    all.insert("MSCURACT".to_string(), mscuract);
    let mut chopconf = IndexMap::new();
    chopconf.insert("toff".to_string(), 0x0f);
    chopconf.insert("hstrt".to_string(), 0x07 << 4);
    chopconf.insert("hend".to_string(), 0x0f << 7);
    chopconf.insert("fd3".to_string(), 1 << 11);
    chopconf.insert("disfdcc".to_string(), 1 << 12);
    chopconf.insert("rndtf".to_string(), 1 << 13);
    chopconf.insert("chm".to_string(), 1 << 14);
    chopconf.insert("tbl".to_string(), 0x03 << 15);
    chopconf.insert("vsense".to_string(), 1 << 17);
    chopconf.insert("vhighfs".to_string(), 1 << 18);
    chopconf.insert("vhighchm".to_string(), 1 << 19);
    chopconf.insert("sync".to_string(), 0x0f << 20);
    chopconf.insert("mres".to_string(), 0x0f << 24);
    chopconf.insert("intpol".to_string(), 1 << 28);
    chopconf.insert("dedge".to_string(), 1 << 29);
    chopconf.insert("diss2g".to_string(), 1 << 30);
    all.insert("CHOPCONF".to_string(), chopconf);
    let mut coolconf = IndexMap::new();
    coolconf.insert("semin".to_string(), 0x0f);
    coolconf.insert("seup".to_string(), 0x03 << 5);
    coolconf.insert("semax".to_string(), 0x0f << 8);
    coolconf.insert("sedn".to_string(), 0x03 << 13);
    coolconf.insert("seimin".to_string(), 1 << 15);
    coolconf.insert("sgt".to_string(), 0x7f << 16);
    coolconf.insert("sfilt".to_string(), 1 << 24);
    all.insert("COOLCONF".to_string(), coolconf);
    let mut drv_status = IndexMap::new();
    drv_status.insert("sg_result".to_string(), 0x3ff);
    drv_status.insert("fsactive".to_string(), 1 << 15);
    drv_status.insert("cs_actual".to_string(), 0x1f << 16);
    drv_status.insert("stallguard".to_string(), 1 << 24);
    drv_status.insert("ot".to_string(), 1 << 25);
    drv_status.insert("otpw".to_string(), 1 << 26);
    drv_status.insert("s2ga".to_string(), 1 << 27);
    drv_status.insert("s2gb".to_string(), 1 << 28);
    drv_status.insert("ola".to_string(), 1 << 29);
    drv_status.insert("olb".to_string(), 1 << 30);
    drv_status.insert("stst".to_string(), 1 << 31);
    all.insert("DRV_STATUS".to_string(), drv_status);
    let mut pwmconf = IndexMap::new();
    pwmconf.insert("pwm_ampl".to_string(), 0xff);
    pwmconf.insert("pwm_grad".to_string(), 0xff << 8);
    pwmconf.insert("pwm_freq".to_string(), 0x03 << 16);
    pwmconf.insert("pwm_autoscale".to_string(), 1 << 18);
    pwmconf.insert("pwm_symmetric".to_string(), 1 << 19);
    pwmconf.insert("freewheel".to_string(), 0x03 << 20);
    all.insert("PWMCONF".to_string(), pwmconf);
    all.insert("PWM_SCALE".to_string(), single("pwm_scale", 0xff));
    all.insert("LOST_STEPS".to_string(), single("lost_steps", 0xfffff));
    all
}

fn single(name: &str, mask: i64) -> IndexMap<String, i64> {
    let mut m = IndexMap::new();
    m.insert(name.to_string(), mask);
    m
}

/// `SignedFields` (`tmc2130.py:95`).
pub fn signed_fields() -> Vec<String> {
    vec![
        "cur_a".to_string(),
        "cur_b".to_string(),
        "sgt".to_string(),
    ]
}

/// `FieldFormatters` (`tmc2130.py:97-112`), insertion ordered.
pub fn field_formatters() -> IndexMap<String, Arc<dyn Fn(i64) -> String + Send + Sync>> {
    let mut f: IndexMap<String, Arc<dyn Fn(i64) -> String + Send + Sync>> = IndexMap::new();
    f.insert(
        "i_scale_analog".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(ExtVREF)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "shaft".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(Reverse)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "reset".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(Reset)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "drv_err".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(ErrorShutdown!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "uv_cp".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(Undervoltage!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert("version".to_string(), Arc::new(|v| format!("{v:#x}")));
    f.insert(
        "mres".to_string(),
        Arc::new(|v| format!("{v}({}usteps)", 0x100i64 >> v)),
    );
    f.insert(
        "otpw".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(OvertempWarning!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "ot".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(OvertempError!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "s2ga".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(ShortToGND_A!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "s2gb".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(ShortToGND_B!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "ola".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(OpenLoad_A!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "olb".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(OpenLoad_B!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "cs_actual".to_string(),
        Arc::new(|v| {
            if v != 0 {
                format!("{v}")
            } else {
                "0(Reset?)".to_string()
            }
        }),
    );
    f
}

// ---------------------------------------------------------------------------
// SPI register response (`MCU_TMC_SPI.get_register_raw` result dict,
// `tmc2130.py:202-216`).
// ---------------------------------------------------------------------------

/// Response of a register read (`tmc2130.py:212-216`): the `spi_status`,
/// `data` and `#receive_time` entries of the `reg_read` dict.
#[derive(Debug, Clone, Copy)]
pub struct RegRead {
    pub spi_status: i64,
    pub data: i64,
    pub receive_time: f64,
}

// ---------------------------------------------------------------------------
// Interfaces consumed by this module.
// ---------------------------------------------------------------------------

// The SPI driver is held as `crate::tmc::McuTmcApi` (the trait owned by
// `extras.tmc` for the `mcu_tmc` duck type) and the current helper is
// exposed as `crate::tmc::TmcCurrentHelperApi`, matching the reference
// where `TMCCurrentHelper` and `MCU_TMC_SPI` are the duck-typed objects
// handed to `tmc.TMCCommandHelper` / `tmc.TMCVirtualPinHelper`.

// ---------------------------------------------------------------------------
// Current helper (`TMCCurrentHelper`, `tmc2130.py:121-178`).
// ---------------------------------------------------------------------------

pub struct TmcCurrentHelper {
    #[allow(dead_code)]
    printer: Printer,
    name: String,
    mcu_tmc: Arc<dyn McuTmcApi>,
    fields: Arc<FieldHelper>,
    req_hold_current: GCell<f64>,
    sense_resistor: f64,
}

/// A Python exception raised by the reference code path.
fn py_error(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

fn config_name_tail(full_name: &str) -> String {
    // Python: config.get_name().split()[-1]
    full_name
        .split_whitespace()
        .last()
        .unwrap_or("")
        .to_string()
}

/// `configfile.ConfigWrapper._get_wrapper` for an option with no default
/// (`"Option '%s' in section '%s' must be specified"`).
fn missing_option(option: &str, config: &Config) -> KlippyError {
    KlippyError::Config(format!(
        "Option '{}' in section '{}' must be specified",
        option,
        config.get_name()
    ))
}

impl TmcCurrentHelper {
    pub fn new(
        config: &Config,
        mcu_tmc: Arc<dyn McuTmcApi>,
    ) -> KResult<Arc<TmcCurrentHelper>> {
        let printer = config.get_printer();
        let name = config_name_tail(&config.get_name());
        let fields = mcu_tmc.get_fields();
        let run_current = config
            .getfloat(
                "run_current",
                GetFloatOpts {
                    default: None,
                    minval: None,
                    maxval: Some(MAX_CURRENT),
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                },
            )?
            .ok_or_else(|| missing_option("run_current", config))?;
        let hold_current = config
            .getfloat(
                "hold_current",
                GetFloatOpts {
                    default: Some(Some(MAX_CURRENT)),
                    minval: None,
                    maxval: Some(MAX_CURRENT),
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(MAX_CURRENT);
        let sense_resistor = config
            .getfloat(
                "sense_resistor",
                GetFloatOpts {
                    default: Some(Some(0.110)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(0.110);
        let helper = Arc::new(TmcCurrentHelper {
            printer,
            name,
            mcu_tmc,
            fields: fields.clone(),
            req_hold_current: GCell::new(hold_current),
            sense_resistor,
        });
        let (vsense, irun, ihold) = helper.calc_current(run_current, hold_current);
        fields.set_field("vsense", vsense as i64, None, None)?;
        fields.set_field("ihold", ihold, None, None)?;
        fields.set_field("irun", irun, None, None)?;
        Ok(helper)
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn req_hold_current(&self) -> f64 {
        self.req_hold_current.get()
    }

    pub fn sense_resistor(&self) -> f64 {
        self.sense_resistor
    }

    /// `int(x)` truncation toward zero (Python `int()`).
    fn py_int(x: f64) -> i64 {
        pycompat::text::trunc_float_to_bigint(x)
            .0
            .to_string()
            .parse::<i64>()
            .unwrap_or(0)
    }

    pub fn calc_current_bits(&self, current: f64, vsense: bool) -> i64 {
        // cs = int(32. * sense_resistor * current * math.sqrt(2.) / vref + .5) - 1
        let sense_resistor = self.sense_resistor + 0.020;
        let mut vref = 0.32f64;
        if vsense {
            vref = 0.18;
        }
        let sqrt2 = pycompat::libm::sqrt(2.0);
        let raw = 32.0 * sense_resistor * current * sqrt2 / vref + 0.5;
        let cs = Self::py_int(raw) - 1;
        cs.clamp(0, 31)
    }

    pub fn calc_current_from_bits(&self, cs: i64, vsense: bool) -> f64 {
        let sense_resistor = self.sense_resistor + 0.020;
        let mut vref = 0.32f64;
        if vsense {
            vref = 0.18;
        }
        let sqrt2 = pycompat::libm::sqrt(2.0);
        ((cs as f64) + 1.0) * vref / (32.0 * sense_resistor * sqrt2)
    }

    pub fn calc_current(&self, run_current: f64, hold_current: f64) -> (bool, i64, i64) {
        let mut vsense = true;
        let mut irun = self.calc_current_bits(run_current, true);
        if irun == 31 {
            let cur = self.calc_current_from_bits(irun, true);
            if cur < run_current {
                let irun2 = self.calc_current_bits(run_current, false);
                let cur2 = self.calc_current_from_bits(irun2, false);
                if (run_current - cur2).abs() < (run_current - cur).abs() {
                    vsense = false;
                    irun = irun2;
                }
            }
        }
        let ihold = self.calc_current_bits(
            if hold_current < run_current {
                hold_current
            } else {
                run_current
            },
            vsense,
        );
        (vsense, irun, ihold)
    }

    /// Returns `(run_current, hold_current, req_hold_current, MAX_CURRENT)`.
    pub fn get_current(&self) -> (f64, f64, f64, f64) {
        let irun = self.fields.get_field("irun", None, None).unwrap_or(0);
        let ihold = self
            .fields
            .get_field("ihold", None, None)
            .unwrap_or(0);
        let vsense = self
            .fields
            .get_field("vsense", None, None)
            .unwrap_or(0)
            != 0;
        let run_current = self.calc_current_from_bits(irun, vsense);
        let hold_current = self.calc_current_from_bits(ihold, vsense);
        (
            run_current,
            hold_current,
            self.req_hold_current.get(),
            MAX_CURRENT,
        )
    }

    pub fn set_current(
        &self,
        run_current: f64,
        hold_current: f64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        self.req_hold_current.set(hold_current);
        let (vsense, irun, ihold) =
            self.calc_current(run_current, hold_current);
        let current_vsense = self
            .fields
            .get_field("vsense", None, None)
            .unwrap_or(0)
            != 0;
        if vsense != current_vsense {
            let val = self
                .fields
                .set_field("vsense", vsense as i64, None, None)?;
            self.mcu_tmc.set_register("CHOPCONF", val, print_time)?;
        }
        self.fields.set_field("ihold", ihold, None, None)?;
        let val = self.fields.set_field("irun", irun, None, None)?;
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)?;
        Ok(())
    }
}

impl crate::tmc::TmcCurrentHelperApi for TmcCurrentHelper {
    fn get_current(&self) -> (f64, Option<f64>, Option<f64>, f64) {
        let (run_current, hold_current, req_hold_current, max_current) =
            TmcCurrentHelper::get_current(self);
        (
            run_current,
            Some(hold_current),
            Some(req_hold_current),
            max_current,
        )
    }

    fn set_current(
        &self,
        run_current: f64,
        hold_current: Option<f64>,
        print_time: f64,
    ) -> KResult<()> {
        let hold_current = hold_current.unwrap_or_else(|| self.req_hold_current.get());
        TmcCurrentHelper::set_current(self, run_current, hold_current, Some(print_time))
    }
}

// ---------------------------------------------------------------------------
// SPI chain (`MCU_TMC_SPI_chain`, `tmc2130.py:185-236`).
// ---------------------------------------------------------------------------

/// `MCU_TMC_SPI_chain` (`tmc2130.py:185-236`).
pub struct McuTmcSpiChain {
    printer: Printer,
    chain_len: i64,
    mutex: reactor::ReactorMutex,
    spi: Arc<dyn McuSpiApi>,
    taken_chain_positions: GCell<Vec<i64>>,
}

impl McuTmcSpiChain {
    pub fn new(
        printer: Printer,
        chain_len: i64,
        spi: Arc<dyn McuSpiApi>,
    ) -> McuTmcSpiChain {
        let mutex = printer.get_reactor().mutex(false);
        McuTmcSpiChain {
            printer,
            chain_len,
            mutex,
            spi,
            taken_chain_positions: GCell::new(Vec::new()),
        }
    }

    /// `MCU_TMC_SPI_chain.__init__` (`tmc2130.py:186-195`): builds the SPI
    /// bus through `bus.MCU_SPI_from_config(config, 3, default_speed=4000000,
    /// share_type=share)` where `share` is `"tmc_spi_cs"` for daisy chains.
    pub fn from_config(
        config: &Config,
        chain_len: i64,
    ) -> KResult<Arc<McuTmcSpiChain>> {
        let printer = config.get_printer();
        let share = if chain_len > 1 {
            Some("tmc_spi_cs")
        } else {
            None
        };
        let spi =
            mcu_spi_from_config(config, 3, "cs_pin", 4000000, share, false)? as Arc<dyn McuSpiApi>;
        Ok(Arc::new(McuTmcSpiChain::new(printer, chain_len, spi)))
    }

    pub fn chain_len(&self) -> i64 {
        self.chain_len
    }

    pub fn taken_chain_positions(&self) -> Vec<i64> {
        self.taken_chain_positions.cloned()
    }

    pub fn mutex(&self) -> &reactor::ReactorMutex {
        &self.mutex
    }

    /// `_build_cmd` (`tmc2130.py:196-198`).
    pub fn build_cmd(&self, data: &[u8], chain_pos: i64) -> Vec<u8> {
        Self::build_cmd_static(self.chain_len, data, chain_pos)
    }

    pub fn build_cmd_static(chain_len: i64, data: &[u8], chain_pos: i64) -> Vec<u8> {
        // Python `[0x00] * n` yields an empty list for n <= 0.
        let lead = ((chain_len - chain_pos) * 5).max(0) as usize;
        let tail = ((chain_pos - 1) * 5).max(0) as usize;
        let mut out = vec![0u8; lead];
        out.extend_from_slice(data);
        out.extend(vec![0u8; tail]);
        out
    }

    fn debugoutput_active(&self) -> bool {
        let args = self.printer.get_start_args();
        match args {
            PyValue::Dict(d) => {
                for (k, v) in d.iter() {
                    if k.str() == "debugoutput" && !matches!(v, PyValue::None) {
                        return true;
                    }
                }
                false
            }
            _ => false,
        }
    }

    /// `bytearray(params['response'])` (`tmc2130.py:209`, `:231`).
    fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
        let response = Self::param(params, "response")?;
        match response {
            PyValue::Bytes(b) => Ok(b.clone()),
            PyValue::List(items) => items.iter().map(Self::byte_of).collect(),
            other => Err(py_error(
                "TypeError",
                format!("cannot convert '{}' object to bytearray", type_name(other)),
            )),
        }
    }

    /// One `bytearray(list)` element: an int in `range(0, 256)`.
    fn byte_of(item: &PyValue) -> KResult<u8> {
        let value = match item {
            PyValue::Bool(b) => i64::from(*b),
            PyValue::Int(n) => n.inner().to_string().parse::<i64>().unwrap_or(-1),
            other => {
                return Err(py_error(
                    "TypeError",
                    format!("'{}' object cannot be interpreted as an integer", type_name(other)),
                ))
            }
        };
        u8::try_from(value)
            .map_err(|_| py_error("ValueError", "byte must be in range(0, 256)".to_string()))
    }

    /// `params['#receive_time']` (`tmc2130.py:215`), read only by `reg_read`.
    fn receive_time(params: &PyValue) -> KResult<f64> {
        match Self::param(params, "#receive_time")? {
            PyValue::Float(f) => Ok(*f),
            PyValue::Int(n) => Ok(n.inner().to_string().parse::<f64>().unwrap_or(f64::INFINITY)),
            other => Err(py_error(
                "TypeError",
                format!("must be real number, not {}", type_name(other)),
            )),
        }
    }

    /// `params[key]` on the response dict.
    fn param<'a>(params: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
        let PyValue::Dict(dict) = params else {
            return Err(py_error(
                "TypeError",
                format!("'{}' object is not subscriptable", type_name(params)),
            ));
        };
        dict.iter()
            .find(|(k, _)| k.str() == key)
            .map(|(_, v)| v)
            .ok_or_else(|| py_error("KeyError", format!("'{key}'")))
    }

    /// `pr[(chain_len - chain_pos) * 5 : (chain_len - chain_pos + 1) * 5]`
    /// with Python slice bounds; indexing a short slice raises
    /// `IndexError: bytearray index out of range` (`tmc2130.py:210-214`).
    fn slice_response(response: &[u8], chain_len: i64, chain_pos: i64) -> KResult<[u8; 5]> {
        let len = response.len() as i64;
        let bound = |idx: i64| if idx < 0 { (idx + len).max(0) } else { idx.min(len) };
        let start = bound((chain_len - chain_pos) * 5);
        let end = bound((chain_len - chain_pos + 1) * 5);
        let slice = if start < end { &response[start as usize..end as usize] } else { &[][..] };
        <[u8; 5]>::try_from(slice)
            .map_err(|_| py_error("IndexError", "bytearray index out of range".to_string()))
    }

    pub fn reg_read(&self, reg: i64, chain_pos: i64) -> KResult<RegRead> {
        let cmd = self.build_cmd(&[(reg & 0xff) as u8, 0, 0, 0, 0], chain_pos);
        self.spi.spi_send(&cmd, SendOpts { minclock: 0, reqclock: 0 })?;
        if self.debugoutput_active() {
            return Ok(RegRead {
                spi_status: 0,
                data: 0,
                receive_time: 0.0,
            });
        }
        let params = self.spi.spi_transfer(
            &cmd,
            QuerySendOpts { minclock: 0, reqclock: 0, retry: true },
        )?;
        let response = Self::response_bytes(&params)?;
        let pr = Self::slice_response(&response, self.chain_len, chain_pos)?;
        let receive_time = Self::receive_time(&params)?;
        Ok(RegRead {
            spi_status: pr[0] as i64,
            data: ((pr[1] as i64) << 24)
                | ((pr[2] as i64) << 16)
                | ((pr[3] as i64) << 8)
                | (pr[4] as i64),
            receive_time,
        })
    }

    pub fn reg_write(
        &self,
        reg: i64,
        val: i64,
        chain_pos: i64,
        print_time: Option<f64>,
    ) -> KResult<i64> {
        let minclock = match print_time {
            Some(t) => self.spi.get_mcu().print_time_to_clock(t),
            None => 0,
        };
        let data = [
            ((reg | 0x80) & 0xff) as u8,
            ((val >> 24) & 0xff) as u8,
            ((val >> 16) & 0xff) as u8,
            ((val >> 8) & 0xff) as u8,
            (val & 0xff) as u8,
        ];
        if self.debugoutput_active() {
            self.spi.spi_send(
                &self.build_cmd(&data, chain_pos),
                SendOpts { minclock, reqclock: 0 },
            )?;
            return Ok(val);
        }
        let write_cmd = self.build_cmd(&data, chain_pos);
        let dummy_read = self.build_cmd(&[0, 0, 0, 0, 0], chain_pos);
        let params = self.spi.spi_transfer_with_preface(
            &write_cmd,
            &dummy_read,
            QuerySendOpts { minclock, reqclock: 0, retry: true },
        )?;
        let response = Self::response_bytes(&params)?;
        let pr = Self::slice_response(&response, self.chain_len, chain_pos)?;
        Ok(((pr[1] as i64) << 24)
            | ((pr[2] as i64) << 16)
            | ((pr[3] as i64) << 8)
            | (pr[4] as i64))
    }

    pub fn get_mcu(&self) -> Mcu {
        self.spi.get_mcu()
    }

    /// Claim a chain position (`lookup_tmc_spi_chain`, `tmc2130.py:254-257`
    /// second half). Returns `false` on duplicate (caller raises).
    pub fn claim_chain_position(&self, chain_pos: i64) -> bool {
        let mut taken = self.taken_chain_positions.borrow_mut();
        if taken.contains(&chain_pos) {
            return false;
        }
        taken.push(chain_pos);
        true
    }
}

// ---------------------------------------------------------------------------
// `lookup_tmc_spi_chain` (`tmc2130.py:239-258`).
//
// Production wiring needs the shared-chain instance stored in the CS pin
// parameters (`cs_pin_params['class']`, `tmc2130.py:250-251`). The frozen
// `PinParams` contract has no slot for it, so `existing` is the value the
// caller found in the pin parameters (`None` until the first claimant),
// and `make_chain` must build `MCU_TMC_SPI_chain(config, chain_len)`.
// ---------------------------------------------------------------------------

/// `lookup_tmc_spi_chain(config)` (`tmc2130.py:239-258`).
///
/// Contract gap (reported, not worked around): the shared daisy-chain object
/// lives in the CS pin parameters (`cs_pin_params['class']`,
/// `tmc2130.py:250-251`) and [`PinParams`](klippy_api::mcu::PinParams) has no
/// slot for it, so a second driver on the same `cs_pin` cannot find the chain
/// the first one built. Until that slot exists every claimant builds its own
/// [`McuTmcSpiChain`] (`lookup_tmc_spi_chain_with` supplies the slot).
pub fn lookup_tmc_spi_chain(config: &Config) -> KResult<(Arc<McuTmcSpiChain>, i64)> {
    lookup_tmc_spi_chain_with(config, None, &|config, chain_len| {
        McuTmcSpiChain::from_config(config, chain_len)
    })
}

/// The `lookup_tmc_spi_chain` body with the chain-sharing slot supplied by the
/// caller: `existing` is `cs_pin_params['class']` and `make_chain` builds
/// `MCU_TMC_SPI_chain(config, chain_len)` (`tmc2130.py:250-252`).
pub fn lookup_tmc_spi_chain_with(
    config: &Config,
    existing: Option<Arc<McuTmcSpiChain>>,
    make_chain: &dyn Fn(&Config, i64) -> KResult<Arc<McuTmcSpiChain>>,
) -> KResult<(Arc<McuTmcSpiChain>, i64)> {
    let chain_len = config.getint(
        "chain_length",
        GetIntOpts {
            default: Some(None),
            minval: Some(2),
            maxval: None,
            note_valid: true,
        },
    )?;
    let chain_len = match chain_len {
        None => {
            // Simple, non daisy chained SPI connection.
            return Ok((make_chain(config, 1)?, 1));
        }
        Some(v) => v,
    };
    // Shared SPI bus - lookup existing MCU_TMC_SPI_chain (`tmc2130.py:246-248`).
    let printer = config.get_printer();
    let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?.ok_or_else(
        || KlippyError::Config("Unknown config object 'pins'".to_string()),
    )?;
    let cs_pin = config
        .get("cs_pin", GetStrOpts::default())?
        .ok_or_else(|| missing_option("cs_pin", config))?;
    let _cs_pin_params = ppins.lookup_pin(
        &cs_pin,
        LookupPinOpts {
            can_invert: false,
            can_pullup: false,
            share_type: Some("tmc_spi_cs".to_string()),
        },
    )?;
    let tmc_spi = match existing {
        Some(chain) => chain,
        None => make_chain(config, chain_len)?,
    };
    if chain_len != tmc_spi.chain_len() {
        return Err(KlippyError::Config(
            "TMC SPI chain must have same length".to_string(),
        ));
    }
    let chain_pos = config.getint(
        "chain_position",
        GetIntOpts {
            default: None,
            minval: Some(1),
            maxval: Some(chain_len),
            note_valid: true,
        },
    )?;
    let chain_pos = chain_pos.ok_or_else(|| missing_option("chain_position", config))?;
    if !tmc_spi.claim_chain_position(chain_pos) {
        return Err(KlippyError::Config(
            "TMC SPI chain can not have duplicate position".to_string(),
        ));
    }
    Ok((tmc_spi, chain_pos))
}

// ---------------------------------------------------------------------------
// `MCU_TMC_SPI` (`tmc2130.py:261-298`).
// ---------------------------------------------------------------------------

pub struct McuTmcSpi {
    #[allow(dead_code)]
    printer: Printer,
    name: String,
    tmc_spi: Arc<McuTmcSpiChain>,
    chain_pos: i64,
    name_to_reg: IndexMap<String, i64>,
    fields: Arc<FieldHelper>,
    tmc_frequency: f64,
}

impl McuTmcSpi {
    /// `MCU_TMC_SPI(config, name_to_reg, fields, tmc_frequency)`
    /// (`tmc2130.py:261-269`).
    pub fn new(
        config: &Config,
        name_to_reg: IndexMap<String, i64>,
        fields: Arc<FieldHelper>,
        tmc_frequency: f64,
    ) -> KResult<McuTmcSpi> {
        let (tmc_spi, chain_pos) = lookup_tmc_spi_chain(config)?;
        Ok(McuTmcSpi::with_chain(
            config,
            name_to_reg,
            fields,
            tmc_frequency,
            tmc_spi,
            chain_pos,
        ))
    }

    /// `__init__` with the daisy-chain object already resolved.
    pub fn with_chain(
        config: &Config,
        name_to_reg: IndexMap<String, i64>,
        fields: Arc<FieldHelper>,
        tmc_frequency: f64,
        tmc_spi: Arc<McuTmcSpiChain>,
        chain_pos: i64,
    ) -> McuTmcSpi {
        let printer = config.get_printer();
        let name = config_name_tail(&config.get_name());
        McuTmcSpi {
            printer,
            name,
            tmc_spi,
            chain_pos,
            name_to_reg,
            fields,
            tmc_frequency,
        }
    }

    pub fn get_fields(&self) -> Arc<FieldHelper> {
        self.fields.clone()
    }

    pub fn get_register_raw(&self, reg_name: &str) -> KResult<RegRead> {
        let reg = match self.name_to_reg.get(reg_name) {
            Some(r) => *r,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "KeyError".to_string(),
                    message: format!("'{reg_name}'"),
                })
            }
        };
        self.tmc_spi.mutex().lock();
        // Hold the lock only around the transfer; the transfer itself only
        // pauses the reactor inside the SPI bus.
        let result = self.tmc_spi.reg_read(reg, self.chain_pos);
        self.tmc_spi.mutex().unlock();
        result
    }

    /// `decode_spi_status` (`tmc2130.py:277-283`).
    pub fn decode_spi_status(spi_status: i64) -> IndexMap<String, i64> {
        let mut m = IndexMap::new();
        m.insert("standstill".to_string(), (spi_status >> 3) & 0x1);
        m.insert("sg2".to_string(), (spi_status >> 2) & 0x1);
        m.insert("driver_error".to_string(), (spi_status >> 1) & 0x1);
        m.insert("reset_flag".to_string(), spi_status & 0x1);
        m
    }

    pub fn get_register(&self, reg_name: &str) -> KResult<i64> {
        Ok(self.get_register_raw(reg_name)?.data)
    }

    pub fn set_register(
        &self,
        reg_name: &str,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        let reg = match self.name_to_reg.get(reg_name) {
            Some(r) => *r,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "KeyError".to_string(),
                    message: format!("'{reg_name}'"),
                })
            }
        };
        self.tmc_spi.mutex().lock();
        let mut succeeded = false;
        let mut result: KResult<()> = Ok(());
        for _ in 0..5 {
            match self.tmc_spi.reg_write(reg, val, self.chain_pos, print_time) {
                Ok(v) if v == val => {
                    succeeded = true;
                    break;
                }
                Ok(_) => {}
                Err(e) => {
                    result = Err(e);
                    break;
                }
            }
        }
        self.tmc_spi.mutex().unlock();
        if result.is_err() {
            return result;
        }
        if succeeded {
            return Ok(());
        }
        Err(KlippyError::Command(format!(
            "Unable to write tmc spi '{}' register {}",
            self.name, reg_name
        )))
    }

    pub fn get_tmc_frequency(&self) -> f64 {
        self.tmc_frequency
    }

    pub fn get_mcu(&self) -> Mcu {
        self.tmc_spi.get_mcu()
    }
}

impl McuTmcApi for McuTmcSpi {
    fn get_fields(&self) -> Arc<FieldHelper> {
        McuTmcSpi::get_fields(self)
    }

    fn get_register(&self, reg_name: &str) -> KResult<i64> {
        McuTmcSpi::get_register(self, reg_name)
    }

    fn get_register_raw(&self, reg_name: &str) -> KResult<RegisterRaw> {
        let raw = McuTmcSpi::get_register_raw(self, reg_name)?;
        Ok(RegisterRaw {
            data: raw.data,
            receive_time: raw.receive_time,
            spi_status: Some(raw.spi_status),
        })
    }

    fn set_register(
        &self,
        reg_name: &str,
        val: i64,
        print_time: Option<f64>,
    ) -> KResult<()> {
        McuTmcSpi::set_register(self, reg_name, val, print_time)
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(McuTmcSpi::get_tmc_frequency(self))
    }

    fn get_mcu(&self) -> Mcu {
        McuTmcSpi::get_mcu(self)
    }

    fn name_to_reg_value(&self, reg_name: &str) -> Option<i64> {
        self.name_to_reg.get(reg_name).copied()
    }
}

// ---------------------------------------------------------------------------
// `TMC2130` (`tmc2130.py:305-353`) and `load_config_prefix`.
// ---------------------------------------------------------------------------

pub struct Tmc2130 {
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<McuTmcSpi>,
    current_helper: Arc<TmcCurrentHelper>,
    cmd_helper: Arc<TMCCommandHelper>,
}

impl Tmc2130 {
    /// `TMC2130.__init__` (`tmc2130.py:305-353`): builds the field
    /// register helper and MCU communication, the virtual-pin helper, the
    /// current helper, the command helper (register dump, phase/status),
    /// the wave/stealth/velocity helpers, then the default config fields.
    pub fn new(config: &Config) -> KResult<Tmc2130> {
        let fields = FieldHelper::new(
            fields_table(),
            signed_fields(),
            field_formatters(),
            None,
        );
        let mcu_tmc = Arc::new(McuTmcSpi::new(
            config,
            registers(),
            fields.clone(),
            TMC_FREQUENCY,
        )?);
        Self::assemble(config, fields, mcu_tmc)
    }

    /// Shared register wiring of [`Tmc2130::new`] after MCU communication is
    /// built. Takes the built MCU communication so tests can drive the same
    /// wiring with a fake chain (mirrors the oracle fakes' `__new__` bypass).
    pub fn assemble(
        config: &Config,
        fields: Arc<FieldHelper>,
        mcu_tmc: Arc<McuTmcSpi>,
    ) -> KResult<Tmc2130> {
        // Allow virtual pins to be created (`tmc2130.py:312`).
        TMCVirtualPinHelper::new(config, mcu_tmc.clone())?;
        // Register commands (`tmc2130.py:314-318`).
        let current_helper = TmcCurrentHelper::new(config, mcu_tmc.clone())?;
        let cmd_helper = TMCCommandHelper::new(
            config,
            mcu_tmc.clone(),
            current_helper.clone(),
        )?;
        cmd_helper.setup_register_dump(read_registers(), None)?;
        // Setup basic register values (`tmc2130.py:320-323`).
        crate::tmc::tmc_wave_table_helper(config, &*mcu_tmc)?;
        crate::tmc::tmc_stealthchop_helper(config, &*mcu_tmc)?;
        crate::tmc::tmc_vcoolthrs_helper(config, &*mcu_tmc)?;
        crate::tmc::tmc_vhigh_helper(config, &*mcu_tmc)?;
        // Allow other registers to be set from the config
        // (`tmc2130.py:324-350`).
        // CHOPCONF
        // All defaults are ints except `pwm_autoscale` (`tmc2130.py:347`).
        fields.set_config_field(config, "toff", ConfigFieldDefault::Int(4))?;
        fields.set_config_field(config, "hstrt", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "hend", ConfigFieldDefault::Int(7))?;
        fields.set_config_field(config, "tbl", ConfigFieldDefault::Int(1))?;
        fields.set_config_field(config, "vhighfs", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "vhighchm", ConfigFieldDefault::Int(0))?;
        // COOLCONF
        fields.set_config_field(config, "semin", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "seup", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "semax", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "sedn", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "seimin", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "sgt", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "sfilt", ConfigFieldDefault::Int(0))?;
        // IHOLDIRUN
        fields.set_config_field(config, "iholddelay", ConfigFieldDefault::Int(8))?;
        // PWMCONF
        fields.set_config_field(config, "pwm_ampl", ConfigFieldDefault::Int(128))?;
        fields.set_config_field(config, "pwm_grad", ConfigFieldDefault::Int(4))?;
        fields.set_config_field(config, "pwm_freq", ConfigFieldDefault::Int(1))?;
        fields.set_config_field(config, "pwm_autoscale", ConfigFieldDefault::Bool(true))?;
        fields.set_config_field(config, "freewheel", ConfigFieldDefault::Int(0))?;
        // TPOWERDOWN
        fields.set_config_field(config, "tpowerdown", ConfigFieldDefault::Int(0))?;
        Ok(Tmc2130 {
            fields,
            mcu_tmc,
            current_helper,
            cmd_helper,
        })
    }

    pub fn fields(&self) -> &Arc<FieldHelper> {
        &self.fields
    }

    pub fn mcu_tmc(&self) -> &Arc<McuTmcSpi> {
        &self.mcu_tmc
    }

    pub fn current_helper(&self) -> &TmcCurrentHelper {
        &self.current_helper
    }

    pub fn cmd_helper(&self) -> &TMCCommandHelper {
        &self.cmd_helper
    }
}

impl klippy_api::PrinterObject for Tmc2130 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.cmd_helper.get_status(Some(eventtime)))
    }
}

/// `self.get_phase_offset = cmd_helper.get_phase_offset` (`tmc2130.py:317`);
/// `endstop_phase` and `angle` look drivers up through the owner's trait.
impl TMCCommandHelperApi for Tmc2130 {
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmd_helper.get_phase_offset()
    }
}

pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    // `load_config_prefix(config)` (`tmc2130.py:352-353`).
    Ok(Some(Arc::new(Tmc2130::new(config)?) as ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
