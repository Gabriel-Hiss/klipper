//! Port of `klippy/extras/tmc2208.py` (TMC2208 UART configuration).
//!
//! Units: register/field tables, `read_translate`, `Tmc2208`,
//! `load_config_prefix`.
//!
//! Cross-module code is imported from its owner: the field register
//! helper and the G-code/stealthChop helpers from `crate::tmc`
//! (`klippy/extras/tmc.py`), the current helper and the base field
//! formatters from `crate::tmc2130` (`klippy/extras/tmc2130.py`), the
//! UART MCU from `crate::tmc_uart` (`klippy/extras/tmc_uart.py`).
//! Services are resolved inside [`load_config_prefix`] as the reference
//! does (no locally injected handles).
//!
//! Contract note: the `McuTmcApi` impl for `McuTmcUart` lives in its
//! owner module (`crate::tmc_uart`); this file only coerces the built
//! MCU to `Arc<dyn McuTmcApi>` (the same shape as the `McuTmcSpi` impl
//! in `crate::tmc2130`).

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::object::ObjectRef;
use klippy_api::printer::ModuleFactory;
use klippy_api::PrinterObject;
use pycompat::PyValue;

use crate::tmc::{
    ConfigFieldDefault, FieldHelper, McuTmcApi, TMCCommandHelper, TMCCommandHelperApi,
};
use crate::tmc2130::TmcCurrentHelper;
use crate::tmc_uart::McuTmcUart;

/// `TMC_FREQUENCY` (`klippy/extras/tmc2208.py:9`).
pub const TMC_FREQUENCY: f64 = 12_000_000.;

// ---------------------------------------------------------------------------
// Register / field tables (`Registers`, `ReadRegisters`, `Fields`,
// `SignedFields`, `FieldFormatters`, `tmc2208.py:11-178`).
// ---------------------------------------------------------------------------

/// `Registers` (`tmc2208.py:11-17`), insertion ordered.
pub fn registers() -> IndexMap<String, i64> {
    let mut m = IndexMap::new();
    m.insert("GCONF".to_string(), 0x00);
    m.insert("GSTAT".to_string(), 0x01);
    m.insert("IFCNT".to_string(), 0x02);
    m.insert("SLAVECONF".to_string(), 0x03);
    m.insert("OTP_PROG".to_string(), 0x04);
    m.insert("OTP_READ".to_string(), 0x05);
    m.insert("IOIN".to_string(), 0x06);
    m.insert("FACTORY_CONF".to_string(), 0x07);
    m.insert("IHOLD_IRUN".to_string(), 0x10);
    m.insert("TPOWERDOWN".to_string(), 0x11);
    m.insert("TSTEP".to_string(), 0x12);
    m.insert("TPWMTHRS".to_string(), 0x13);
    m.insert("VACTUAL".to_string(), 0x22);
    m.insert("MSCNT".to_string(), 0x6a);
    m.insert("MSCURACT".to_string(), 0x6b);
    m.insert("CHOPCONF".to_string(), 0x6c);
    m.insert("DRV_STATUS".to_string(), 0x6f);
    m.insert("PWMCONF".to_string(), 0x70);
    m.insert("PWM_SCALE".to_string(), 0x71);
    m.insert("PWM_AUTO".to_string(), 0x72);
    m
}

/// `ReadRegisters` (`tmc2208.py:19-23`).
pub fn read_registers() -> Vec<String> {
    vec![
        "GCONF".to_string(),
        "GSTAT".to_string(),
        "IFCNT".to_string(),
        "OTP_READ".to_string(),
        "IOIN".to_string(),
        "FACTORY_CONF".to_string(),
        "TSTEP".to_string(),
        "MSCNT".to_string(),
        "MSCURACT".to_string(),
        "CHOPCONF".to_string(),
        "DRV_STATUS".to_string(),
        "PWMCONF".to_string(),
        "PWM_SCALE".to_string(),
        "PWM_AUTO".to_string(),
    ]
}

/// `Fields` (`tmc2208.py:25-169`), insertion ordered per register. The
/// `IOIN@...` variants both define `sel_a`/`version`; the later
/// `IOIN@TMC220x` entry wins the field-to-register lookup, matching the
/// dict-comprehension build in `tmc.FieldHelper`.
pub fn fields_table() -> IndexMap<String, IndexMap<String, i64>> {
    let mut all: IndexMap<String, IndexMap<String, i64>> = IndexMap::new();
    let mut gconf = IndexMap::new();
    gconf.insert("i_scale_analog".to_string(), 0x01);
    gconf.insert("internal_rsense".to_string(), 0x01 << 1);
    gconf.insert("en_spreadcycle".to_string(), 0x01 << 2);
    gconf.insert("shaft".to_string(), 0x01 << 3);
    gconf.insert("index_otpw".to_string(), 0x01 << 4);
    gconf.insert("index_step".to_string(), 0x01 << 5);
    gconf.insert("pdn_disable".to_string(), 0x01 << 6);
    gconf.insert("mstep_reg_select".to_string(), 0x01 << 7);
    gconf.insert("multistep_filt".to_string(), 0x01 << 8);
    gconf.insert("test_mode".to_string(), 0x01 << 9);
    all.insert("GCONF".to_string(), gconf);
    let mut gstat = IndexMap::new();
    gstat.insert("reset".to_string(), 0x01);
    gstat.insert("drv_err".to_string(), 0x01 << 1);
    gstat.insert("uv_cp".to_string(), 0x01 << 2);
    all.insert("GSTAT".to_string(), gstat);
    let mut ifcnt = IndexMap::new();
    ifcnt.insert("ifcnt".to_string(), 0xff);
    all.insert("IFCNT".to_string(), ifcnt);
    let mut slaveconf = IndexMap::new();
    slaveconf.insert("senddelay".to_string(), 0x0f << 8);
    all.insert("SLAVECONF".to_string(), slaveconf);
    let mut otp_prog = IndexMap::new();
    otp_prog.insert("otpbit".to_string(), 0x07);
    otp_prog.insert("otpbyte".to_string(), 0x03 << 4);
    otp_prog.insert("otpmagic".to_string(), 0xff << 8);
    all.insert("OTP_PROG".to_string(), otp_prog);
    let mut otp_read = IndexMap::new();
    otp_read.insert("otp_fclktrim".to_string(), 0x1f);
    otp_read.insert("otp_ottrim".to_string(), 0x01 << 5);
    otp_read.insert("otp_internalrsense".to_string(), 0x01 << 6);
    otp_read.insert("otp_tbl".to_string(), 0x01 << 7);
    otp_read.insert("otp_pwm_grad".to_string(), 0x0f << 8);
    otp_read.insert("otp_pwm_autograd".to_string(), 0x01 << 12);
    otp_read.insert("otp_tpwmthrs".to_string(), 0x07 << 13);
    otp_read.insert("otp_pwm_ofs".to_string(), 0x01 << 16);
    otp_read.insert("otp_pwm_reg".to_string(), 0x01 << 17);
    otp_read.insert("otp_pwm_freq".to_string(), 0x01 << 18);
    otp_read.insert("otp_iholddelay".to_string(), 0x03 << 19);
    otp_read.insert("otp_ihold".to_string(), 0x03 << 21);
    otp_read.insert("otp_en_spreadcycle".to_string(), 0x01 << 23);
    all.insert("OTP_READ".to_string(), otp_read);
    let mut ioin_222x = IndexMap::new();
    ioin_222x.insert("pdn_uart".to_string(), 0x01 << 1);
    ioin_222x.insert("spread".to_string(), 0x01 << 2);
    ioin_222x.insert("dir".to_string(), 0x01 << 3);
    ioin_222x.insert("enn".to_string(), 0x01 << 4);
    ioin_222x.insert("step".to_string(), 0x01 << 5);
    ioin_222x.insert("ms1".to_string(), 0x01 << 6);
    ioin_222x.insert("ms2".to_string(), 0x01 << 7);
    ioin_222x.insert("sel_a".to_string(), 0x01 << 8);
    ioin_222x.insert("version".to_string(), 0xff << 24);
    all.insert("IOIN@TMC222x".to_string(), ioin_222x);
    let mut ioin_220x = IndexMap::new();
    ioin_220x.insert("enn".to_string(), 0x01);
    ioin_220x.insert("ms1".to_string(), 0x01 << 2);
    ioin_220x.insert("ms2".to_string(), 0x01 << 3);
    ioin_220x.insert("diag".to_string(), 0x01 << 4);
    ioin_220x.insert("pdn_uart".to_string(), 0x01 << 6);
    ioin_220x.insert("step".to_string(), 0x01 << 7);
    ioin_220x.insert("sel_a".to_string(), 0x01 << 8);
    ioin_220x.insert("dir".to_string(), 0x01 << 9);
    ioin_220x.insert("version".to_string(), 0xff << 24);
    all.insert("IOIN@TMC220x".to_string(), ioin_220x);
    let mut factory_conf = IndexMap::new();
    factory_conf.insert("fclktrim".to_string(), 0x1f);
    factory_conf.insert("ottrim".to_string(), 0x03 << 8);
    all.insert("FACTORY_CONF".to_string(), factory_conf);
    let mut ihold_irun = IndexMap::new();
    ihold_irun.insert("ihold".to_string(), 0x1f);
    ihold_irun.insert("irun".to_string(), 0x1f << 8);
    ihold_irun.insert("iholddelay".to_string(), 0x0f << 16);
    all.insert("IHOLD_IRUN".to_string(), ihold_irun);
    let mut tpowerdown = IndexMap::new();
    tpowerdown.insert("tpowerdown".to_string(), 0xff);
    all.insert("TPOWERDOWN".to_string(), tpowerdown);
    let mut tstep = IndexMap::new();
    tstep.insert("tstep".to_string(), 0xfffff);
    all.insert("TSTEP".to_string(), tstep);
    let mut tpwmthrs = IndexMap::new();
    tpwmthrs.insert("tpwmthrs".to_string(), 0xfffff);
    all.insert("TPWMTHRS".to_string(), tpwmthrs);
    let mut vactual = IndexMap::new();
    vactual.insert("vactual".to_string(), 0xffffff);
    all.insert("VACTUAL".to_string(), vactual);
    let mut mscnt = IndexMap::new();
    mscnt.insert("mscnt".to_string(), 0x3ff);
    all.insert("MSCNT".to_string(), mscnt);
    let mut mscuract = IndexMap::new();
    mscuract.insert("cur_a".to_string(), 0x1ff);
    mscuract.insert("cur_b".to_string(), 0x1ff << 16);
    all.insert("MSCURACT".to_string(), mscuract);
    let mut chopconf = IndexMap::new();
    chopconf.insert("toff".to_string(), 0x0f);
    chopconf.insert("hstrt".to_string(), 0x07 << 4);
    chopconf.insert("hend".to_string(), 0x0f << 7);
    chopconf.insert("tbl".to_string(), 0x03 << 15);
    chopconf.insert("vsense".to_string(), 0x01 << 17);
    chopconf.insert("mres".to_string(), 0x0f << 24);
    chopconf.insert("intpol".to_string(), 0x01 << 28);
    chopconf.insert("dedge".to_string(), 0x01 << 29);
    chopconf.insert("diss2g".to_string(), 0x01 << 30);
    chopconf.insert("diss2vs".to_string(), 0x01 << 31);
    all.insert("CHOPCONF".to_string(), chopconf);
    let mut drv_status = IndexMap::new();
    drv_status.insert("otpw".to_string(), 0x01);
    drv_status.insert("ot".to_string(), 0x01 << 1);
    drv_status.insert("s2ga".to_string(), 0x01 << 2);
    drv_status.insert("s2gb".to_string(), 0x01 << 3);
    drv_status.insert("s2vsa".to_string(), 0x01 << 4);
    drv_status.insert("s2vsb".to_string(), 0x01 << 5);
    drv_status.insert("ola".to_string(), 0x01 << 6);
    drv_status.insert("olb".to_string(), 0x01 << 7);
    drv_status.insert("t120".to_string(), 0x01 << 8);
    drv_status.insert("t143".to_string(), 0x01 << 9);
    drv_status.insert("t150".to_string(), 0x01 << 10);
    drv_status.insert("t157".to_string(), 0x01 << 11);
    drv_status.insert("cs_actual".to_string(), 0x1f << 16);
    drv_status.insert("stealth".to_string(), 0x01 << 30);
    drv_status.insert("stst".to_string(), 0x01 << 31);
    all.insert("DRV_STATUS".to_string(), drv_status);
    let mut pwmconf = IndexMap::new();
    pwmconf.insert("pwm_ofs".to_string(), 0xff);
    pwmconf.insert("pwm_grad".to_string(), 0xff << 8);
    pwmconf.insert("pwm_freq".to_string(), 0x03 << 16);
    pwmconf.insert("pwm_autoscale".to_string(), 0x01 << 18);
    pwmconf.insert("pwm_autograd".to_string(), 0x01 << 19);
    pwmconf.insert("freewheel".to_string(), 0x03 << 20);
    pwmconf.insert("pwm_reg".to_string(), 0xf << 24);
    pwmconf.insert("pwm_lim".to_string(), 0xf << 28);
    all.insert("PWMCONF".to_string(), pwmconf);
    let mut pwm_scale = IndexMap::new();
    pwm_scale.insert("pwm_scale_sum".to_string(), 0xff);
    pwm_scale.insert("pwm_scale_auto".to_string(), 0x1ff << 16);
    all.insert("PWM_SCALE".to_string(), pwm_scale);
    let mut pwm_auto = IndexMap::new();
    pwm_auto.insert("pwm_ofs_auto".to_string(), 0xff);
    pwm_auto.insert("pwm_grad_auto".to_string(), 0xff << 16);
    all.insert("PWM_AUTO".to_string(), pwm_auto);
    all
}

/// `SignedFields` (`tmc2208.py:171`).
pub fn signed_fields() -> Vec<String> {
    vec!["cur_a".to_string(), "cur_b".to_string(), "pwm_scale_auto".to_string()]
}

/// `FieldFormatters` (`tmc2208.py:173-178`): the `tmc2130` base formatters
/// with the `sel_a`, `s2vsa` and `s2vsb` overrides merged in. The override
/// keys are new keys, so dict-update appends them at the end, in source
/// order.
pub fn field_formatters() -> IndexMap<String, Arc<dyn Fn(i64) -> String + Send + Sync>> {
    let mut f = crate::tmc2130::field_formatters();
    f.insert(
        "sel_a".to_string(),
        Arc::new(|v| {
            // `"%d(%s)" % (v, ["TMC222x", "TMC220x"][v])` (`tmc2208.py:175`):
            // a negative index wraps around, an out-of-range index raises
            // IndexError (here: panics out of bounds).
            let names = ["TMC222x", "TMC220x"];
            let idx = if v < 0 { v + names.len() as i64 } else { v };
            let name = names[idx as usize];
            format!("{}({})", v, name)
        }),
    );
    f.insert(
        "s2vsa".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(ShortToSupply_A!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f.insert(
        "s2vsb".to_string(),
        Arc::new(|v| {
            if v != 0 {
                "1(ShortToSupply_B!)".to_string()
            } else {
                String::new()
            }
        }),
    );
    f
}

/// `read_translate` (`tmc2208.py:223-227`): an `IOIN` dump value selects
/// the variant register table by its `sel_a` field. The value itself is
/// never changed.
pub fn read_translate(fields: &FieldHelper, reg_name: &str, val: i64) -> (String, i64) {
    if reg_name == "IOIN" {
        let drv_type = fields
            .get_field("sel_a", Some(val), None)
            .unwrap_or_else(|e| panic!("{}", e.message()));
        let reg_name = if drv_type != 0 { "IOIN@TMC220x" } else { "IOIN@TMC222x" };
        return (reg_name.to_string(), val);
    }
    (reg_name.to_string(), val)
}

// ---------------------------------------------------------------------------
// `TMC2208` (`tmc2208.py:185-227`) and `load_config_prefix`.
// ---------------------------------------------------------------------------

pub struct Tmc2208 {
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<McuTmcUart<Arc<FieldHelper>>>,
    current_helper: Arc<TmcCurrentHelper>,
    cmd_helper: Arc<TMCCommandHelper>,
}

impl Tmc2208 {
    /// `TMC2208.__init__` (`tmc2208.py:186-222`): builds the field
    /// register helper and the UART MCU, then runs the shared register
    /// wiring in the reference order.
    pub fn new(config: &Config) -> KResult<Tmc2208> {
        let fields = FieldHelper::new(fields_table(), signed_fields(), field_formatters(), None);
        let mcu_tmc = Arc::new(McuTmcUart::new(
            config,
            registers(),
            fields.clone(),
            0,
            TMC_FREQUENCY,
        )?);
        Self::assemble(config, fields, mcu_tmc)
    }

    /// Shared register wiring of [`Tmc2208::new`] after MCU communication
    /// is built. Takes the built MCU communication so tests can drive the
    /// same wiring with a fake chain (mirrors the oracle fakes' `__new__`
    /// bypass).
    pub fn assemble(
        config: &Config,
        fields: Arc<FieldHelper>,
        mcu_tmc: Arc<McuTmcUart<Arc<FieldHelper>>>,
    ) -> KResult<Tmc2208> {
        let mcu_api: Arc<dyn McuTmcApi> = mcu_tmc.clone();
        // `self.fields.set_field("pdn_disable", True)` (`tmc2208.py:191`).
        fields.set_field("pdn_disable", 1, None, None)?;
        // Register commands (`tmc2208.py:193-197`).
        let current_helper = TmcCurrentHelper::new(config, mcu_api.clone())?;
        let current_api: Arc<dyn crate::tmc::TmcCurrentHelperApi> = current_helper.clone();
        let cmd_helper = TMCCommandHelper::new(config, mcu_api.clone(), current_api)?;
        let fields_cb = fields.clone();
        cmd_helper.setup_register_dump(
            read_registers(),
            Some(Arc::new(move |reg_name: &str, val: i64| {
                read_translate(&fields_cb, reg_name, val)
            })),
        )?;
        // Setup basic register values (`tmc2208.py:199-200`).
        fields.set_field("mstep_reg_select", 1, None, None)?;
        crate::tmc::tmc_stealthchop_helper(config, &*mcu_api)?;
        // Allow other registers to be set from the config (`tmc2208.py:203-222`).
        fields.set_config_field(config, "multistep_filt", ConfigFieldDefault::Bool(true))?;
        // CHOPCONF
        fields.set_config_field(config, "toff", ConfigFieldDefault::Int(3))?;
        fields.set_config_field(config, "hstrt", ConfigFieldDefault::Int(5))?;
        fields.set_config_field(config, "hend", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "tbl", ConfigFieldDefault::Int(2))?;
        // IHOLDIRUN
        fields.set_config_field(config, "iholddelay", ConfigFieldDefault::Int(8))?;
        // PWMCONF
        fields.set_config_field(config, "pwm_ofs", ConfigFieldDefault::Int(36))?;
        fields.set_config_field(config, "pwm_grad", ConfigFieldDefault::Int(14))?;
        fields.set_config_field(config, "pwm_freq", ConfigFieldDefault::Int(1))?;
        fields.set_config_field(config, "pwm_autoscale", ConfigFieldDefault::Bool(true))?;
        fields.set_config_field(config, "pwm_autograd", ConfigFieldDefault::Bool(true))?;
        fields.set_config_field(config, "freewheel", ConfigFieldDefault::Int(0))?;
        fields.set_config_field(config, "pwm_reg", ConfigFieldDefault::Int(8))?;
        fields.set_config_field(config, "pwm_lim", ConfigFieldDefault::Int(12))?;
        // TPOWERDOWN
        fields.set_config_field(config, "tpowerdown", ConfigFieldDefault::Int(20))?;
        Ok(Tmc2208 {
            fields,
            mcu_tmc,
            current_helper,
            cmd_helper,
        })
    }

    pub fn fields(&self) -> &Arc<FieldHelper> {
        &self.fields
    }

    pub fn mcu_tmc(&self) -> &Arc<McuTmcUart<Arc<FieldHelper>>> {
        &self.mcu_tmc
    }

    pub fn current_helper(&self) -> &TmcCurrentHelper {
        &self.current_helper
    }

    pub fn cmd_helper(&self) -> &TMCCommandHelper {
        &self.cmd_helper
    }

    /// `self.read_translate` (`tmc2208.py:223-227`).
    pub fn read_translate(&self, reg_name: &str, val: i64) -> (String, i64) {
        read_translate(&self.fields, reg_name, val)
    }
}

impl PrinterObject for Tmc2208 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);
    /// `self.get_status = cmd_helper.get_status` (`tmc2208.py:197`).
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.cmd_helper.get_status(Some(eventtime))))
    }
}

/// `self.get_phase_offset = cmd_helper.get_phase_offset` (`tmc2208.py:196`);
/// `endstop_phase` and `angle` look drivers up through the owner's trait.
impl TMCCommandHelperApi for Tmc2208 {
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmd_helper.get_phase_offset()
    }
}

pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    // `load_config_prefix(config)` (`tmc2208.py:229-230`).
    Ok(Some(Arc::new(Tmc2208::new(config)?) as ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
