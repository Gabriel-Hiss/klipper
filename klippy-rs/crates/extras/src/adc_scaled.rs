//! Port of `klippy/extras/adc_scaled.py`.
//!
//! A `[adc_scaled <name>]` section scales ADC readings between the measured
//! `vref` (max) and `vssa` (min) pins, both smoothed with `smooth_time`, and
//! registers itself as an ADC chip so `setup_pin('adc', ...)` returns scaled
//! readings.

use crate::query_adc::QueryAdcApi;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AdcCallback, AdcPin, AdcPinApi, AdcSampleOpts, ChipApi, Mcu, Pin, PinParams, PinsApi};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{load_interface, lookup_interface, ObjectRef};
use std::sync::Arc;

/// `SAMPLE_TIME` (`adc_scaled.py:7`).
pub const SAMPLE_TIME: f64 = 0.001;
/// `SAMPLE_COUNT` (`adc_scaled.py:8`).
pub const SAMPLE_COUNT: i64 = 8;
/// `REPORT_TIME` (`adc_scaled.py:9`).
pub const REPORT_TIME: f64 = 0.300;

/// Shared mutable state of a [`PrinterADCScaled`] (`last_vref` / `last_vssa`,
/// `adc_scaled.py:39-40`) plus the derived smoothing factor. Public because
/// [`McuScaledAdc::new`] takes it.
pub struct CoreState {
    /// `last_vref` (`adc_scaled.py:39`).
    pub last_vref: (f64, f64),
    /// `last_vssa` (`adc_scaled.py:40`).
    pub last_vssa: (f64, f64),
    /// `1. / smooth_time` (`adc_scaled.py:45`).
    pub inv_smooth_time: f64,
}

fn index_error() -> KlippyError {
    KlippyError::Internal { class_name: "IndexError".to_string(), message: "list index out of range".to_string() }
}

/// `PrinterADCScaled.calc_smooth` (`adc_scaled.py:65-71`).
///
/// `adj_time` follows Python `min(time_diff * inv_smooth_time, 1.)`
/// exactly, including the NaN case (`1. < v` is false for NaN, so the
/// first argument wins).
pub fn calc_smooth(inv_smooth_time: f64, read_time: f64, read_value: f64, last: (f64, f64)) -> (f64, f64) {
    let last_time = last.0;
    let last_value = last.1;
    let time_diff = read_time - last_time;
    let value_diff = read_value - last_value;
    let scaled = time_diff * inv_smooth_time;
    let adj_time = if 1. < scaled { 1. } else { scaled };
    let smoothed_value = last_value + value_diff * adj_time;
    (read_time, smoothed_value)
}

/// `PrinterADCScaled` (`adc_scaled.py:35-77`).
pub struct PrinterADCScaled {
    printer: Printer,
    name: String,
    state: Arc<GCell<CoreState>>,
    mcu: Mcu,
}

klippy_api::printer_object!(PrinterADCScaled; dyn ChipApi);

impl PrinterADCScaled {
    /// `PrinterADCScaled.__init__` (`adc_scaled.py:36-51`).
    ///
    /// Follows the Python order: the section name is split first, then
    /// `vref` and `vssa` are set up via [`config_pin`], then `smooth_time`
    /// is read, the vref/vssa MCUs are compared, and the chip is
    /// registered with `pins`.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let section = config.get_name();
        let name = PrinterADCScaled::parse_name(&section)?;
        // Placeholder factor; the real `1. / smooth_time` is installed
        // after `smooth_time` is read below. Callbacks observe it only
        // when samples arrive, after construction has finished.
        let state = Arc::new(GCell::new(CoreState {
            last_vref: (0., 0.),
            last_vssa: (0., 0.),
            inv_smooth_time: 1. / 2.,
        }));
        let mcu_vref = config_pin(&printer, config, &name, "vref", true, &state)?;
        let mcu_vssa = config_pin(&printer, config, &name, "vssa", false, &state)?;
        let smooth_time = config
            .getfloat(
                "smooth_time",
                GetFloatOpts {
                    default: Some(Some(2.)),
                    above: Some(0.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(2.);
        state.borrow_mut().inv_smooth_time = 1. / smooth_time;
        let mcu = mcu_vref.get_mcu();
        if !Arc::ptr_eq(&mcu, &mcu_vssa.get_mcu()) {
            return Err(KlippyError::Config("vref and vssa must be on same mcu".to_string()));
        }
        let this = Arc::new(PrinterADCScaled { printer: printer.clone(), name: name.clone(), state, mcu });
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        ppins.register_chip(&name, this.clone())?;
        Ok(this)
    }

    /// Section name without the module token (`config.get_name().split()[1]`,
    /// `adc_scaled.py:38`). A single-token name raises `IndexError` in
    /// Python, reproduced here.
    pub fn parse_name(section: &str) -> KResult<String> {
        let mut parts = section.split_whitespace();
        let _ = parts.next();
        parts.next().map(|s| s.to_string()).ok_or_else(index_error)
    }

    /// The section name (`self.name`, `adc_scaled.py:38`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The printer the object was built with.
    pub fn printer(&self) -> &Printer {
        &self.printer
    }

    /// `self.mcu` (`adc_scaled.py:46`).
    pub fn mcu(&self) -> &Mcu {
        &self.mcu
    }

    /// `self.last_vref` (`adc_scaled.py:39`).
    pub fn last_vref(&self) -> (f64, f64) {
        self.state.borrow().last_vref
    }

    /// `self.last_vssa` (`adc_scaled.py:40`).
    pub fn last_vssa(&self) -> (f64, f64) {
        self.state.borrow().last_vssa
    }

    /// `calc_smooth` bound to this object's smoothing factor
    /// (`adc_scaled.py:65-71`).
    pub fn calc_smooth(&self, read_time: f64, read_value: f64, last: (f64, f64)) -> (f64, f64) {
        let inv = self.state.borrow().inv_smooth_time;
        calc_smooth(inv, read_time, read_value, last)
    }

    /// `vref_callback` (`adc_scaled.py:72-74`).
    pub fn vref_callback(&self, samples: &[(f64, f64)]) {
        let Some(&(read_time, read_value)) = samples.last() else { return };
        let mut state = self.state.borrow_mut();
        state.last_vref = calc_smooth(state.inv_smooth_time, read_time, read_value, state.last_vref);
    }

    /// `vssa_callback` (`adc_scaled.py:75-77`).
    pub fn vssa_callback(&self, samples: &[(f64, f64)]) {
        let Some(&(read_time, read_value)) = samples.last() else { return };
        let mut state = self.state.borrow_mut();
        state.last_vssa = calc_smooth(state.inv_smooth_time, read_time, read_value, state.last_vssa);
    }
}

/// `_config_pin` (`adc_scaled.py:52-60`): set up one ADC pin, register its
/// smoothing callback and sampling, and register it with `query_adc` as
/// `<name>:<vref|vssa>`.
fn config_pin(
    printer: &Printer,
    config: &Config,
    name: &str,
    which: &str,
    is_vref: bool,
    state: &Arc<GCell<CoreState>>,
) -> KResult<AdcPin> {
    let option = format!("{which}_pin");
    let pin_name = config
        .get(&option, GetStrOpts::default())?
        .ok_or_else(|| {
            KlippyError::Config(format!(
                "Option '{option}' in section '{}' must be specified",
                config.get_name()
            ))
        })?;
    let ppins = lookup_interface::<dyn PinsApi>(&**printer, "pins", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    let mcu_adc = match ppins.setup_pin("adc", &pin_name)? {
        Pin::Adc(adc) => adc,
        _ => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "pins.setup_pin('adc', ...) did not return an ADC pin".to_string(),
            });
        }
    };
    let state = Arc::clone(state);
    if is_vref {
        mcu_adc.setup_adc_callback(Box::new(move |samples| {
            let Some(&(read_time, read_value)) = samples.last() else { return };
            let mut s = state.borrow_mut();
            s.last_vref = calc_smooth(s.inv_smooth_time, read_time, read_value, s.last_vref);
        }));
    } else {
        mcu_adc.setup_adc_callback(Box::new(move |samples| {
            let Some(&(read_time, read_value)) = samples.last() else { return };
            let mut s = state.borrow_mut();
            s.last_vssa = calc_smooth(s.inv_smooth_time, read_time, read_value, s.last_vssa);
        }));
    }
    mcu_adc.setup_adc_sample(
        REPORT_TIME,
        AdcSampleOpts { sample_time: SAMPLE_TIME, sample_count: SAMPLE_COUNT, ..AdcSampleOpts::default() },
    );
    let query_adc = load_interface::<dyn QueryAdcApi>(&**printer, config, "query_adc", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    let qname = format!("{name}:{which}");
    query_adc.register_adc(&qname, mcu_adc.clone());
    Ok(mcu_adc)
}

impl ChipApi for PrinterADCScaled {
    /// `setup_pin` (`adc_scaled.py:61-64`).
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        if pin_type != "adc" {
            return Err(KlippyError::Config("adc_scaled only supports adc pins".to_string()));
        }
        let scaled = McuScaledAdc::new(&self.printer, &self.mcu, &self.state, &self.name, pin_params)?;
        Ok(Pin::Adc(scaled))
    }

    /// The MCU of the vref pin (`adc_scaled.py:46` is `self.mcu`, built from
    /// `mcu_vref.get_mcu()`).
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
}

/// Inner shared state of [`McuScaledAdc`], so the wrapped ADC callback can
/// reach it without an `Arc<Self>` cycle.
struct ScaledInner {
    state: Arc<GCell<CoreState>>,
    mcu_adc: AdcPin,
    last_state: GCell<(f64, f64)>,
    callback: GCell<Option<AdcCallback>>,
}

/// `MCU_scaled_adc` (`adc_scaled.py:11-33`): an ADC pin whose readings are
/// scaled between the current `vssa` (min) and `vref` (max).
pub struct McuScaledAdc {
    inner: Arc<ScaledInner>,
}

impl McuScaledAdc {
    /// `MCU_scaled_adc.__init__` (`adc_scaled.py:12-21`): set up the
    /// underlying ADC pin through `main.mcu.setup_pin('adc', pin_params)`
    /// and register it with `query_adc` as `<name>:<pin>`. No callback is
    /// installed until [`McuScaledAdc::setup_adc_callback`].
    pub fn new(
        printer: &Printer,
        mcu: &Mcu,
        state: &Arc<GCell<CoreState>>,
        name: &str,
        pin_params: PinParams,
    ) -> KResult<Arc<Self>> {
        let mcu_adc = match mcu.setup_pin("adc", pin_params.clone())? {
            Pin::Adc(adc) => adc,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "mcu.setup_pin('adc', ...) did not return an ADC pin".to_string(),
                });
            }
        };
        let query_adc = lookup_interface::<dyn QueryAdcApi>(&**printer, "query_adc", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let qname = format!("{name}:{}", pin_params.pin);
        query_adc.register_adc(&qname, mcu_adc.clone());
        Ok(Arc::new(McuScaledAdc {
            inner: Arc::new(ScaledInner {
                state: Arc::clone(state),
                mcu_adc,
                last_state: GCell::new((0., 0.)),
                callback: GCell::new(None),
            }),
        }))
    }

    /// `setup_adc_callback` (`adc_scaled.py:29-31`). The registered wrapper
    /// is `_handle_callback` (`adc_scaled.py:22-28`); note Python float
    /// division by zero raises `ZeroDivisionError` when the smoothed
    /// `last_vref == last_vssa`, while this callback cannot return an
    /// error, so Rust division yields inf/NaN there (reported assumption).
    pub fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.inner.callback.borrow_mut() = Some(callback);
        let inner = Arc::clone(&self.inner);
        self.inner.mcu_adc.setup_adc_callback(Box::new(move |samples| {
            let state = inner.state.borrow();
            let max_adc = state.last_vref.1;
            let min_adc = state.last_vssa.1;
            drop(state);
            let adjsamples: Vec<(f64, f64)> =
                samples.iter().map(|&(t, v)| (t, (v - min_adc) / (max_adc - min_adc))).collect();
            if let Some(&last) = adjsamples.last() {
                inner.last_state.set(last);
            }
            let mut callback = inner.callback.borrow_mut().take();
            if let Some(cb) = callback.as_mut() {
                let cb: &mut AdcCallback = cb;
                cb(adjsamples);
            }
            // Put the callback back unless it re-registered a new one
            // while running (as Python would keep the new one).
            if callback.is_some() && inner.callback.borrow().is_none() {
                *inner.callback.borrow_mut() = callback;
            }
        }));
    }

    /// `get_last_value` (`adc_scaled.py:32-33`).
    pub fn get_last_value(&self) -> (f64, f64) {
        self.inner.last_state.get()
    }
}

impl AdcPinApi for McuScaledAdc {
    fn get_mcu(&self) -> Mcu {
        self.inner.mcu_adc.get_mcu()
    }

    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.inner.mcu_adc.setup_adc_sample(report_time, opts);
    }

    fn setup_adc_callback(&self, callback: AdcCallback) {
        McuScaledAdc::setup_adc_callback(self, callback);
    }

    fn get_last_value(&self) -> (f64, f64) {
        McuScaledAdc::get_last_value(self)
    }
}

/// `load_config_prefix` (`adc_scaled.py:79-80`): `return PrinterADCScaled(config)`.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterADCScaled::new(config)? as ObjectRef))
}

/// Module factory for the `[adc_scaled <name>]` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: None, load_config_prefix: Some(load_config_prefix), ..ModuleFactory::default() }
}
