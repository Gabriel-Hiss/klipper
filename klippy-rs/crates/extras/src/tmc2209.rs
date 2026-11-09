//! Port of `klippy/extras/tmc2209.py` (TMC2209 UART configuration).
//!
//! Units: register/field tables, `TMC2209`, `load_config_prefix`.
//!
//! The tables are the TMC2208 tables plus the TMC2209 deltas
//! (`tmc2209.py:10-49`), so they are built from the owner
//! `crate::tmc2208`; this module adds only the four registers, five
//! field maps and the `SG_RESULT` read-register delta. Field
//! formatters are also owned by `crate::tmc2208` (`tmc2209.py:49`
//! copies that dict unchanged).
//!
//! Construction mirrors `TMC2209.__init__` (`tmc2209.py:57-107`) in
//! order: `tmc.FieldHelper` (`crate::tmc::FieldHelper`), then
//! `tmc_uart.MCU_TMC_uart` (`crate::tmc_uart::McuTmcUart`), the fixed
//! `pdn_disable`/`senddelay` fields, `tmc.TMCVirtualPinHelper`
//! (`crate::tmc::TMCVirtualPinHelper`), `tmc2130.TMCCurrentHelper`
//! (`crate::tmc2130::TmcCurrentHelper`), `tmc.TMCCommandHelper`
//! (`crate::tmc::TMCCommandHelper`) with the register dump, the
//! `mstep_reg_select` preset, `tmc.TMCStealthchopHelper` /
//! `tmc.TMCVcoolthrsHelper` (`crate::tmc::tmc_stealthchop_helper` /
//! `crate::tmc::tmc_vcoolthrs_helper`), and finally the `driver_*`
//! config fields. `get_status`/`get_phase_offset` are the command
//! helper's (`tmc2209.py:72-73` binds them as instance attributes).

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::object::ObjectRef;
use klippy_api::printer::ModuleFactory;
use klippy_api::PrinterObject;
use pycompat::PyValue;

use crate::tmc::{
    ConfigFieldDefault, FieldFormatter, FieldHelper, McuTmcApi, TMCCommandHelper,
    TMCCommandHelperApi, TMCVirtualPinHelper, TmcCurrentHelperApi,
};
use crate::tmc2130::TmcCurrentHelper;
use crate::tmc_uart::McuTmcUart;

/// `TMC_FREQUENCY` (`tmc2209.py:8`).
pub const TMC_FREQUENCY: f64 = 12_000_000.;

/// `max_addr` passed to `MCU_TMC_uart` (`tmc2209.py:61` passes `3`).
pub const TMC_MAX_ADDR: i64 = 3;

/// `Registers` (`tmc2209.py:10-16`): the TMC2208 registers in their
/// order, with the four TMC2209-specific registers appended by the
/// `Registers.update(...)` call.
pub fn registers() -> IndexMap<String, i64> {
    let mut regs = crate::tmc2208::registers();
    regs.insert("TCOOLTHRS".to_string(), 0x14);
    regs.insert("COOLCONF".to_string(), 0x42);
    regs.insert("SGTHRS".to_string(), 0x40);
    regs.insert("SG_RESULT".to_string(), 0x41);
    regs
}

/// `ReadRegisters` (`tmc2209.py:18`): the TMC2208 read list plus
/// `SG_RESULT`, in order.
pub fn read_registers() -> Vec<String> {
    let mut regs = crate::tmc2208::read_registers();
    regs.push("SG_RESULT".to_string());
    regs
}

/// `SignedFields` (`tmc2208.py:171`); `tmc2209.py:59-60` reuses them
/// unchanged when building its `FieldHelper`.
pub fn signed_fields() -> Vec<String> {
    crate::tmc2208::signed_fields()
}

/// `Fields` (`tmc2209.py:20-47`): the TMC2208 fields in their order,
/// with `COOLCONF`, `IOIN`, `SGTHRS`, `SG_RESULT` and `TCOOLTHRS`
/// appended by the `Fields[...] = ...` assignments in source order.
/// The plain `IOIN` entry coexists with the inherited
/// `IOIN@TMC222x` / `IOIN@TMC220x` entries, exactly as in Python
/// (neither variant is the key `"IOIN"`).
pub fn fields() -> IndexMap<String, IndexMap<String, i64>> {
    let mut all = crate::tmc2208::fields_table();
    let mut coolconf = IndexMap::new();
    coolconf.insert("semin".to_string(), 0x0F);
    coolconf.insert("seup".to_string(), 0x03 << 5);
    coolconf.insert("semax".to_string(), 0x0F << 8);
    coolconf.insert("sedn".to_string(), 0x03 << 13);
    coolconf.insert("seimin".to_string(), 0x01 << 15);
    all.insert("COOLCONF".to_string(), coolconf);
    let mut ioin = IndexMap::new();
    ioin.insert("enn".to_string(), 0x01 << 0);
    ioin.insert("ms1".to_string(), 0x01 << 2);
    ioin.insert("ms2".to_string(), 0x01 << 3);
    ioin.insert("diag".to_string(), 0x01 << 4);
    ioin.insert("pdn_uart".to_string(), 0x01 << 6);
    ioin.insert("step".to_string(), 0x01 << 7);
    ioin.insert("spread_en".to_string(), 0x01 << 8);
    ioin.insert("dir".to_string(), 0x01 << 9);
    ioin.insert("version".to_string(), 0xff << 24);
    all.insert("IOIN".to_string(), ioin);
    let mut sgthrs = IndexMap::new();
    sgthrs.insert("sgthrs".to_string(), 0xFF);
    all.insert("SGTHRS".to_string(), sgthrs);
    let mut sg_result = IndexMap::new();
    sg_result.insert("sg_result".to_string(), 0x3FF);
    all.insert("SG_RESULT".to_string(), sg_result);
    let mut tcoolthrs = IndexMap::new();
    tcoolthrs.insert("tcoolthrs".to_string(), 0xfffff);
    all.insert("TCOOLTHRS".to_string(), tcoolthrs);
    all
}

/// Names with a field formatter, in `FieldFormatters` order
/// (`tmc2209.py:49` copies the `tmc2208` dict unchanged).
pub fn field_formatter_names() -> Vec<String> {
    crate::tmc2208::field_formatters()
        .keys()
        .cloned()
        .collect()
}

/// The `FieldFormatters` map passed to `FieldHelper`
/// (`tmc2209.py:59-60` passes the `tmc2208` dict unchanged).
pub fn field_formatters() -> IndexMap<String, FieldFormatter> {
    crate::tmc2208::field_formatters()
}

/// Field formatter dispatch: the `FieldFormatters[name]` lambda applied
/// to `value`, or `None` when the field has no formatter.
pub fn format_field(name: &str, value: i64) -> Option<String> {
    let formatters = crate::tmc2208::field_formatters();
    formatters.get(name).map(|f| f(value))
}

/// The `driver_*` config fields read by `TMC2209.__init__`
/// (`tmc2209.py:81-107`), in call order with the defaults Python
/// passes: `True` for the single-bit fields, ints otherwise.
pub fn config_field_defaults() -> Vec<(&'static str, ConfigFieldDefault)> {
    use ConfigFieldDefault as D;
    vec![
        ("multistep_filt", D::Bool(true)),
        ("toff", D::Int(3)),
        ("hstrt", D::Int(5)),
        ("hend", D::Int(0)),
        ("tbl", D::Int(2)),
        ("semin", D::Int(0)),
        ("seup", D::Int(0)),
        ("semax", D::Int(0)),
        ("sedn", D::Int(0)),
        ("seimin", D::Int(0)),
        ("iholddelay", D::Int(8)),
        ("pwm_ofs", D::Int(36)),
        ("pwm_grad", D::Int(14)),
        ("pwm_freq", D::Int(1)),
        ("pwm_autoscale", D::Bool(true)),
        ("pwm_autograd", D::Bool(true)),
        ("freewheel", D::Int(0)),
        ("pwm_reg", D::Int(8)),
        ("pwm_lim", D::Int(12)),
        ("tpowerdown", D::Int(20)),
        ("sgthrs", D::Int(0)),
    ]
}

/// `TMC2209` (`tmc2209.py:56`).
pub struct TMC2209 {
    fields: Arc<FieldHelper>,
    mcu_tmc: Arc<McuTmcUart<Arc<FieldHelper>>>,
    current_helper: Arc<TmcCurrentHelper>,
    cmd_helper: Arc<TMCCommandHelper>,
}

impl TMC2209 {
    /// `TMC2209.__init__` (`tmc2209.py:57-107`): builds the field
    /// register helper and the UART MCU, then runs the shared register
    /// wiring in the reference order.
    pub fn new(config: &Config) -> KResult<TMC2209> {
        let fields = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
        let mcu_tmc = Arc::new(McuTmcUart::new(
            config,
            registers(),
            fields.clone(),
            TMC_MAX_ADDR,
            TMC_FREQUENCY,
        )?);
        Self::assemble(config, fields, mcu_tmc)
    }

    /// Shared register wiring of [`TMC2209::new`] after MCU communication
    /// is built. Takes the built MCU communication so tests can drive the
    /// same wiring with a fake chain (mirrors the oracle fakes'
    /// `__new__` bypass).
    pub fn assemble(
        config: &Config,
        fields: Arc<FieldHelper>,
        mcu_tmc: Arc<McuTmcUart<Arc<FieldHelper>>>,
    ) -> KResult<TMC2209> {
        let mcu_api: Arc<dyn McuTmcApi> = mcu_tmc.clone();
        // Setup fields for UART (`tmc2209.py:63-65`).
        fields.set_field("pdn_disable", 1, None, None)?;
        fields.set_field("senddelay", 2, None, None)?;
        // Allow virtual pins to be created (`tmc2209.py:67`).
        let _virtual_pins = TMCVirtualPinHelper::new(config, mcu_api.clone())?;
        // Register commands (`tmc2209.py:69-73`).
        let current_helper = TmcCurrentHelper::new(config, mcu_api.clone())?;
        let current_api: Arc<dyn TmcCurrentHelperApi> = current_helper.clone();
        let cmd_helper = TMCCommandHelper::new(config, mcu_api.clone(), current_api)?;
        cmd_helper.setup_register_dump(read_registers(), None)?;
        // Setup basic register values (`tmc2209.py:75-77`).
        fields.set_field("mstep_reg_select", 1, None, None)?;
        crate::tmc::tmc_stealthchop_helper(config, &*mcu_api)?;
        crate::tmc::tmc_vcoolthrs_helper(config, &*mcu_api)?;
        // Allow other registers to be set from the config (`tmc2209.py:79-107`).
        for (field_name, default) in config_field_defaults() {
            fields.set_config_field(config, field_name, default)?;
        }
        Ok(TMC2209 {
            fields,
            mcu_tmc,
            current_helper,
            cmd_helper,
        })
    }

    /// `self.fields` (`tmc2209.py:59`).
    pub fn fields(&self) -> &Arc<FieldHelper> {
        &self.fields
    }

    /// `self.mcu_tmc` (`tmc2209.py:61`).
    pub fn mcu_tmc(&self) -> &Arc<McuTmcUart<Arc<FieldHelper>>> {
        &self.mcu_tmc
    }

    /// The current helper built from `self.mcu_tmc` (`tmc2209.py:69`).
    pub fn current_helper(&self) -> &TmcCurrentHelper {
        &self.current_helper
    }

    /// The command helper (`tmc2209.py:70`), whose `get_status` and
    /// `get_phase_offset` the Python object aliases (`tmc2209.py:72-73`).
    pub fn cmd_helper(&self) -> &TMCCommandHelper {
        &self.cmd_helper
    }
}

impl TMCCommandHelperApi for TMC2209 {
    /// `self.get_phase_offset = cmdhelper.get_phase_offset` (`tmc2209.py:72`).
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        self.cmd_helper.get_phase_offset()
    }
}

impl PrinterObject for TMC2209 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TMCCommandHelperApi);
    /// `self.get_status = cmdhelper.get_status` (`tmc2209.py:73`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.cmd_helper.get_status(Some(eventtime)))
    }
}

/// `load_config_prefix(config)` (`tmc2209.py:109-110`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(TMC2209::new(config)?) as ObjectRef))
}

/// Module entry point for the `tmc2209 <stepper>` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
