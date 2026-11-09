//! Port of the `samd_sercom` module (`klippy/extras/samd_sercom.py`).
//!
//! Programs the SAMD SERCOM pins on the MCU selected by `tx_pin`:
//! one `set_sercom_pin` config command for the tx, clk and (optional) rx
//! pins. There is no runtime behaviour beyond object construction.

use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::lookup_interface;
use klippy_api::mcu::{AddConfigCmdOpts, LookupPinOpts, Mcu, PinsApi};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::ObjectRef;
use std::sync::Arc;

/// `klippy/extras/samd_sercom.py:SamdSERCOM`.
pub struct SamdSERCOM {
    /// `self.printer` (`samd_sercom.py:9`).
    pub printer: Printer,
    /// `self.sercom` (`samd_sercom.py:11`).
    pub sercom: String,
    /// `self.tx_pin` (`samd_sercom.py:12`).
    pub tx_pin: String,
    /// `self.rx_pin` (`samd_sercom.py:13`): `None` when the option is absent.
    pub rx_pin: Option<String>,
    /// `self.clk_pin` (`samd_sercom.py:14`).
    pub clk_pin: String,
    /// `self.mcu` (`samd_sercom.py:18`): chip of `tx_pin`.
    pub mcu: Mcu,
}

impl SamdSERCOM {
    /// `SamdSERCOM.__init__(config)` (`klippy/extras/samd_sercom.py:8`).
    pub fn new(config: &Config) -> KResult<SamdSERCOM> {
        let printer = config.get_printer();
        // Required options: the core raises for a missing one, so `Ok`
        // always carries `Some`; the `expect` is unreachable.
        let sercom = config
            .get("sercom", GetStrOpts::default())?
            .expect("required config option `sercom`");
        let tx_pin = config
            .get("tx_pin", GetStrOpts::default())?
            .expect("required config option `tx_pin`");
        let rx_pin = config.get(
            "rx_pin",
            GetStrOpts {
                default: Some(None),
                note_valid: true,
            },
        )?;
        let clk_pin = config
            .get("clk_pin", GetStrOpts::default())?
            .expect("required config option `clk_pin`");

        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");

        let tx_pin_params = ppins.lookup_pin(&tx_pin, LookupPinOpts::default())?;
        let mcu = tx_pin_params.chip.clone();
        mcu.add_config_cmd(
            &format!(
                "set_sercom_pin bus={sercom} sercom_pin_type=tx pin={}",
                tx_pin_params.pin
            ),
            AddConfigCmdOpts::default(),
        )?;

        let clk_pin_params = ppins.lookup_pin(&clk_pin, LookupPinOpts::default())?;
        if !Arc::ptr_eq(&mcu, &clk_pin_params.chip) {
            return Err(KlippyError::Pins(format!(
                "{}: SERCOM pins must be on same mcu",
                config.get_name()
            )));
        }
        mcu.add_config_cmd(
            &format!(
                "set_sercom_pin bus={sercom} sercom_pin_type=clk pin={}",
                clk_pin_params.pin
            ),
            AddConfigCmdOpts::default(),
        )?;

        // Python `if self.rx_pin:` trims no whitespace but treats `None`
        // and the empty string as falsy.
        if let Some(rx) = &rx_pin {
            if !rx.is_empty() {
                let rx_pin_params = ppins.lookup_pin(rx, LookupPinOpts::default())?;
                if !Arc::ptr_eq(&mcu, &rx_pin_params.chip) {
                    return Err(KlippyError::Pins(format!(
                        "{}: SERCOM pins must be on same mcu",
                        config.get_name()
                    )));
                }
                mcu.add_config_cmd(
                    &format!(
                        "set_sercom_pin bus={sercom} sercom_pin_type=rx pin={}",
                        rx_pin_params.pin
                    ),
                    AddConfigCmdOpts::default(),
                )?;
            }
        }

        Ok(SamdSERCOM {
            printer,
            sercom,
            tx_pin,
            rx_pin,
            clk_pin,
            mcu,
        })
    }
}

klippy_api::printer_object!(SamdSERCOM);

/// `load_config_prefix(config)` (`klippy/extras/samd_sercom.py:40`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(SamdSERCOM::new(config)?) as ObjectRef))
}

/// Module entry point for the `samd_sercom` prefix sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
