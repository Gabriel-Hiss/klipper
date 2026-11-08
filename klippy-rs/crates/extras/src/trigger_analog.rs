//! Port of the `trigger_analog` module (`klippy/extras/trigger_analog.py`).
//!
//! Wrapper around MCU `trigger_analog` / `sos_filter` objects: a fixed
//! point SOS filter designer ([`DigitalFilter`], [`MCU_SosFilter`]) and
//! the homing interface ([`MCU_trigger_analog`]) used by load-cell and
//! eddy-current probes. The Python module has no `load_config` /
//! `load_config_prefix` / `add_printer_objects`, so [`module_factory`]
//! is empty, matching the Python module.

use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_core::mcu::McuTrsync;
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandWrapper, EndstopApi,
    LookupQueryCommandOpts, Mcu, QuerySendOpts, SendOpts, TriggerDispatch,
};
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::Stepper;
use pycompat::text::{float_repr, percent_format, round_py, PctValue, PyError};
use pycompat::{PyInt, PyValue};
use reactor::Completion;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// SOS filters (Second Order Sectional)
// ---------------------------------------------------------------------------

/// `MAX_INT32` (`trigger_analog.py:14`).
pub const MAX_INT32: i64 = (1 << 31) - 1;
/// `MIN_INT32` (`trigger_analog.py:15`).
pub const MIN_INT32: i64 = -(1 << 31);

/// Which Python exception class a [`DigitalFilter`] raises when SciPy is
/// unavailable (`self.cfg_error` in Python).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CfgErrorKind {
    /// `config.error` / `printer.config_error` (a `configparser.Error`).
    Config,
    /// `gcmd.error` (`gcode.CommandError`), passed by `load_cell_probe`
    /// when the filter is rebuilt from a G-code command.
    Command,
    /// `ImportError` (used by `pre_gen_filt`, `trigger_analog.py:58`).
    Import,
}

fn cfg_error(kind: CfgErrorKind, message: &str) -> KlippyError {
    match kind {
        CfgErrorKind::Config => KlippyError::Config(message.to_string()),
        CfgErrorKind::Command => KlippyError::Command(message.to_string()),
        CfgErrorKind::Import => KlippyError::Other {
            class_name: "ImportError".to_string(),
            message: message.to_string(),
        },
    }
}

fn percent(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_else(|e| format!("{fmt} args={args:?} ({e})"))
}

/// `assert_is_int32(value, frac_bits)` (`trigger_analog.py:16`).
pub fn assert_is_int32(value: i64, frac_bits: i64) -> KResult<i64> {
    if !(MIN_INT32..=MAX_INT32).contains(&value) {
        return Err(fixed_point_overflow(frac_bits));
    }
    Ok(value)
}

fn fixed_point_overflow(frac_bits: i64) -> KlippyError {
    KlippyError::Other {
        class_name: "OverflowError".to_string(),
        message: percent(
            "Fixed point Q%d.%d overflow",
            &[PctValue::Int(PyInt::from_i64(31 - frac_bits)), PctValue::Int(PyInt::from_i64(frac_bits))],
        ),
    }
}

fn py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

/// `to_fixed_32(value, frac_bits=0)` (`trigger_analog.py:24`): `int(round(value * 2**frac_bits))` with overflow check.
pub fn to_fixed_32(value: f64, frac_bits: i64) -> KResult<i64> {
    let factor = 2f64.powi(frac_bits as i32);
    let fixed = round_py(value * factor).map_err(py_error)?;
    // A value outside i64 is outside int32 as well: the Python
    // `assert_is_int32` raises the same overflow error for it.
    match fixed.0.to_string().parse::<i64>() {
        Ok(v) => assert_is_int32(v, frac_bits),
        Err(_) => Err(fixed_point_overflow(frac_bits)),
    }
}

/// `int(v)` of a float: `ValueError`/`OverflowError` for NaN/infinity.
fn int_of_float(v: f64) -> KResult<f64> {
    if v.is_nan() {
        return Err(KlippyError::Other {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if v.is_infinite() {
        return Err(KlippyError::Other {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    Ok(v.trunc())
}

/// `calc_frac_bits(values)` (`trigger_analog.py:28`).
pub fn calc_frac_bits(values: &[f64]) -> KResult<i64> {
    let mut all_int = true;
    for v in values {
        // `v == int(v)`: the int is exact, so this is `v` having no fraction.
        all_int &= *v == int_of_float(*v)?;
    }
    if all_int {
        return Ok(0);
    }
    let mv = values.iter().map(|v| v.abs()).fold(f64::NEG_INFINITY, f64::max);
    // `int(mv).bit_length()`: 0 below 1.0, else the unbiased exponent + 1.
    let bit_length = if mv < 1.0 { 0 } else { ((mv.to_bits() >> 52) & 0x7ff) as i64 - 1023 + 1 };
    let frac_bits = 31 - bit_length;
    if frac_bits <= 0 {
        return Ok(0);
    }
    for v in values {
        // Handle the rare case where rounding causes an overflow.
        if let Err(e) = to_fixed_32(*v, frac_bits) {
            if e.class_name() == "OverflowError" {
                return Ok(frac_bits - 1);
            }
            return Err(e);
        }
    }
    Ok(frac_bits)
}

/// `GeneratedSOS` was replaced by `{}` by `pre_gen_filt` (`trigger_analog.py:56`).
static GENERATED_SOS_CLEARED: AtomicBool = AtomicBool::new(false);

/// Pre-generated SOS filter entry (`trigger_analog.py:44`).
///
/// (`btype`, `float(sps)/frequency`, `int(order)`) -> sections.
fn generated_sos(btype: &str, sps_over_freq: f64, order: i64) -> Option<Vec<Vec<f64>>> {
    if GENERATED_SOS_CLEARED.load(Ordering::SeqCst) {
        return None;
    }
    if btype == "lowpass" && sps_over_freq == 400.0 / 25.0 && order == 4 {
        Some(vec![
            vec![
                0.0009334986129548442,
                0.0018669972259096883,
                0.0009334986129548442,
                1.0,
                -1.3651172372392975,
                0.4775922500725171,
            ],
            vec![1.0, 2.0, 1.0, 1.0, -1.6117270964574348, 0.7445208382054344],
        ])
    } else {
        None
    }
}

/// Restores the pre-generated table (`GeneratedSOS` at import time). Python
/// has no such function; tests that run [`pre_gen_filt`] use it to undo the
/// module-global mutation.
pub fn reset_generated_sos() {
    GENERATED_SOS_CLEARED.store(false, Ordering::SeqCst);
}

/// `pre_gen_filt(btype, sps, freq, order)` (`trigger_analog.py:54`).
///
/// Clears the global table, then designs the filter with
/// `DigitalFilter(sps, ImportError)`. SciPy is not available, so the design
/// raises `ImportError` after the table was cleared, exactly as the
/// reference without SciPy. On success the exact text `sys.stdout.write`
/// would receive is returned.
pub fn pre_gen_filt(btype: &str, sps: f64, freq: f64, order: i64) -> KResult<String> {
    GENERATED_SOS_CLEARED.store(true, Ordering::SeqCst);
    let df = DigitalFilter::new_import_error(sps);
    let fs = df.butter(freq, btype, order)?;
    let mut msgs: Vec<String> = Vec::new();
    msgs.push(percent(
        "    ('%s', %s/%s, %d): [",
        &[
            PctValue::Str(btype.to_string()),
            PctValue::Str(float_repr(sps)),
            PctValue::Str(float_repr(freq)),
            PctValue::Int(PyInt::from_i64(order)),
        ],
    ));
    for data in &fs {
        let coeffs = data.iter().map(|c| float_repr(*c)).collect::<Vec<_>>().join(", ");
        msgs.push(percent("        [%s],", &[PctValue::Str(coeffs)]));
    }
    msgs.push("    ],".to_string());
    msgs.push(String::new());
    Ok(msgs.join("\n"))
}

// ---------------------------------------------------------------------------
// Digital filter designer and container
// ---------------------------------------------------------------------------

/// `DigitalFilter` (`trigger_analog.py:73`).
///
/// `cfg_error` is the Python exception factory the caller passes
/// (`config.error` or `ImportError`); SciPy itself is unavailable in
/// the port, so every `get_scipy_signal` path raises that error exactly
/// like the Python module without SciPy installed. The pre-generated
/// table entry still resolves without SciPy.
pub struct DigitalFilter {
    /// `filter_sections`
    pub filter_sections: GCell<Vec<Vec<f64>>>,
    /// `initial_state` (always `None` in the port: `setup_initial_state`
    /// raises `NameError` like Python before assigning).
    pub initial_state: GCell<Option<Vec<Vec<f64>>>>,
    /// `sample_frequency`
    pub sample_frequency: GCell<f64>,
    cfg_error: CfgErrorKind,
}

impl DigitalFilter {
    /// `DigitalFilter(sps, cfg_error)` with a config-error factory.
    pub fn new(sps: f64, cfg_error: CfgErrorKind) -> DigitalFilter {
        DigitalFilter {
            filter_sections: GCell::new(Vec::new()),
            initial_state: GCell::new(None),
            sample_frequency: GCell::new(sps),
            cfg_error,
        }
    }

    /// `DigitalFilter(sps, ImportError)` as used by `pre_gen_filt`.
    pub fn new_import_error(sps: f64) -> DigitalFilter {
        DigitalFilter::new(sps, CfgErrorKind::Import)
    }

    fn scipy_missing(&self) -> KlippyError {
        cfg_error(self.cfg_error, "DigitalFilter require the SciPy module")
    }

    /// `get_scipy_signal` (`trigger_analog.py:79`): SciPy is not available
    /// on the target, so the import fails and the `cfg_error` is raised.
    pub fn get_scipy_signal(&self) -> KResult<()> {
        Err(self.scipy_missing())
    }

    /// `add_highpass` (`trigger_analog.py:85`).
    pub fn add_highpass(&self, highpass: f64, highpass_order: i64) -> KResult<()> {
        let f = self.butter(highpass, "highpass", highpass_order)?;
        self.filter_sections.borrow_mut().extend(f);
        Ok(())
    }

    /// `add_lowpass` (`trigger_analog.py:88`).
    pub fn add_lowpass(&self, lowpass: f64, lowpass_order: i64) -> KResult<()> {
        let f = self.butter(lowpass, "lowpass", lowpass_order)?;
        self.filter_sections.borrow_mut().extend(f);
        Ok(())
    }

    /// `add_notch` (`trigger_analog.py:91`): needs SciPy.
    pub fn add_notch(&self, _notch_freq: f64, _notch_quality: f64) -> KResult<()> {
        self.get_scipy_signal()?;
        Ok(())
    }

    /// `add_derivative` (`trigger_analog.py:97`).
    pub fn add_derivative(&self) {
        self.filter_sections.borrow_mut().push(vec![1.0, -1.0, 0.0, 1.0, 0.0, 0.0]);
    }

    /// `setup_initial_state` (`trigger_analog.py:100`).
    ///
    /// Python references the undefined name `signal` (`name 'signal' is
    /// not defined`); it is reproduced rather than fixed.
    pub fn setup_initial_state(&self) -> KResult<()> {
        if self.filter_sections.borrow().is_empty() {
            return Ok(());
        }
        Err(KlippyError::Other {
            class_name: "NameError".to_string(),
            message: "name 'signal' is not defined".to_string(),
        })
    }

    /// `_butter(frequency, btype, order)` (`trigger_analog.py:104`).
    pub fn butter(&self, frequency: f64, btype: &str, order: i64) -> KResult<Vec<Vec<f64>>> {
        if frequency == 0.0 {
            return Err(KlippyError::Other {
                class_name: "ZeroDivisionError".to_string(),
                message: "float division by zero".to_string(),
            });
        }
        if let Some(v) = generated_sos(btype, self.sample_frequency.get() / frequency, order) {
            return Ok(v);
        }
        // `signal.butter(...)` needs SciPy.
        Err(self.scipy_missing())
    }

    /// `get_filter_sections` (`trigger_analog.py:111`).
    pub fn get_filter_sections(&self) -> Vec<Vec<f64>> {
        self.filter_sections.cloned()
    }

    /// `get_initial_state` (`trigger_analog.py:113`).
    pub fn get_initial_state(&self) -> Vec<Vec<f64>> {
        match self.initial_state.cloned() {
            None => vec![vec![0.0, 0.0]; self.filter_sections.borrow().len()],
            Some(v) => v,
        }
    }

    /// `get_size` (`trigger_analog.py:117`).
    pub fn get_size(&self) -> usize {
        self.filter_sections.borrow().len()
    }
}

// ---------------------------------------------------------------------------
// Control an `sos_filter` object on the MCU
// ---------------------------------------------------------------------------

/// `MCU_SosFilter` (`trigger_analog.py:121`).
///
/// Shares ownership through `Arc`: `MCU_trigger_analog` holds the same
/// handle the host code configures (`set_filter_design`, …).
#[allow(non_camel_case_types)]
pub struct MCU_SosFilter {
    mcu: Mcu,
    cmd_queue: CommandQueue,
    max_sections: i64,
    design: GCell<Option<Arc<DigitalFilter>>>,
    coeff_frac_bits: GCell<i64>,
    last_calc_frac_bits: GCell<Option<Vec<f64>>>,
    start_value: GCell<f64>,
    offset: GCell<i64>,
    scale: GCell<f64>,
    auto_offset: GCell<bool>,
    oid: i64,
    set_section_cmd: GCell<Option<CommandWrapper>>,
    set_state_cmd: GCell<Option<CommandWrapper>>,
    set_offset_scale_cmd: GCell<Option<CommandWrapper>>,
    set_active_cmd: GCell<Option<CommandWrapper>>,
    last_sent_coeffs: GCell<Vec<Option<Vec<PyValue>>>>,
    last_sent_offset_scale: GCell<Option<Vec<PyValue>>>,
}

impl MCU_SosFilter {
    /// `MCU_SosFilter(mcu, cmd_queue, max_sections)` (`trigger_analog.py:124`).
    pub fn new(mcu: Mcu, cmd_queue: CommandQueue, max_sections: i64) -> KResult<Arc<Self>> {
        let oid = mcu.create_oid()?;
        let this = Arc::new(MCU_SosFilter {
            mcu: mcu.clone(),
            cmd_queue,
            max_sections,
            design: GCell::new(None),
            coeff_frac_bits: GCell::new(0),
            last_calc_frac_bits: GCell::new(None),
            start_value: GCell::new(0.0),
            offset: GCell::new(0),
            scale: GCell::new(1.0),
            auto_offset: GCell::new(false),
            oid,
            set_section_cmd: GCell::new(None),
            set_state_cmd: GCell::new(None),
            set_offset_scale_cmd: GCell::new(None),
            set_active_cmd: GCell::new(None),
            last_sent_coeffs: GCell::new(vec![None; max_sections.max(0) as usize]),
            last_sent_offset_scale: GCell::new(None),
        });
        mcu.add_config_cmd(
            &percent(
                "config_sos_filter oid=%d max_sections=%d",
                &[PctValue::Int(PyInt::from_i64(oid)), PctValue::Int(PyInt::from_i64(max_sections))],
            ),
            AddConfigCmdOpts::default(),
        )?;
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || this2.build_config()))?;
        Ok(this)
    }

    /// `_build_config` (`trigger_analog.py:147`).
    pub fn build_config(&self) -> KResult<()> {
        let cq = Some(self.cmd_queue);
        *self.set_section_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "sos_filter_set_section oid=%c section_idx=%c sos0=%i sos1=%i sos2=%i sos3=%i sos4=%i",
            cq,
        )?);
        *self.set_state_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "sos_filter_set_state oid=%c section_idx=%c state0=%i state1=%i",
            cq,
        )?);
        *self.set_offset_scale_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "sos_filter_set_offset_scale oid=%c offset=%i scale=%i scale_frac_bits=%c auto_offset=%c",
            cq,
        )?);
        *self.set_active_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "sos_filter_set_active oid=%c n_sections=%c coeff_frac_bits=%c",
            cq,
        )?);
        Ok(())
    }

    /// `get_oid` (`trigger_analog.py:161`).
    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    /// `_calc_coeff_frac_bits(filter_sections)` (`trigger_analog.py:165`).
    fn calc_coeff_frac_bits(&self, filter_sections: &[Vec<f64>]) -> KResult<()> {
        let coeffs: Vec<f64> = filter_sections.iter().flat_map(|f| f.iter().copied()).collect();
        if filter_sections.is_empty() || Some(&coeffs) == self.last_calc_frac_bits.borrow().as_ref() {
            return Ok(());
        }
        self.coeff_frac_bits.set(calc_frac_bits(&coeffs)?);
        *self.last_calc_frac_bits.borrow_mut() = Some(coeffs);
        Ok(())
    }

    /// `_convert_filter` (`trigger_analog.py:173`).
    pub fn convert_filter(&self) -> KResult<Vec<Vec<i64>>> {
        let Some(design) = self.design.cloned() else {
            return Ok(Vec::new());
        };
        let filter_sections = design.get_filter_sections();
        self.calc_coeff_frac_bits(&filter_sections)?;
        let frac = self.coeff_frac_bits.get();
        let mut sos_fixed = Vec::new();
        for section in &filter_sections {
            if section.len() != 6 {
                return Err(KlippyError::Other {
                    class_name: "ValueError".to_string(),
                    message: percent(
                        "The number of filter coefficients is %i, must be 6",
                        &[PctValue::Int(PyInt::from_i64(section.len() as i64))],
                    ),
                });
            }
            let mut fixed_section = Vec::new();
            for (col, coeff) in section.iter().enumerate() {
                if col != 3 {
                    fixed_section.push(to_fixed_32(*coeff, frac)?);
                } else if *coeff != 1.0 {
                    return Err(KlippyError::Other {
                        class_name: "ValueError".to_string(),
                        message: percent(
                            "Coefficient 3 is expected to be 1.0 but was %f",
                            &[PctValue::Float(*coeff)],
                        ),
                    });
                }
            }
            sos_fixed.push(fixed_section);
        }
        Ok(sos_fixed)
    }

    /// `_convert_state` (`trigger_analog.py:196`).
    pub fn convert_state(&self) -> KResult<Vec<Vec<i64>>> {
        let Some(design) = self.design.cloned() else {
            return Ok(Vec::new());
        };
        let filter_state = design.get_initial_state();
        let start = self.start_value.get();
        let mut sos_state = Vec::new();
        for section in &filter_state {
            if section.len() != 2 {
                return Err(KlippyError::Other {
                    class_name: "ValueError".to_string(),
                    message: percent(
                        "The number of state elements is %i, must be 2",
                        &[PctValue::Int(PyInt::from_i64(section.len() as i64))],
                    ),
                });
            }
            let mut fixed_state = Vec::new();
            for value in section {
                fixed_state.push(to_fixed_32(value * start, 0)?);
            }
            sos_state.push(fixed_state);
        }
        Ok(sos_state)
    }

    /// `set_start_state` (`trigger_analog.py:216`).
    pub fn set_start_state(&self, start_value: f64) {
        self.start_value.set(start_value);
    }

    /// `set_offset_scale` (`trigger_analog.py:220`).
    pub fn set_offset_scale(&self, offset: i64, scale: f64, auto_offset: bool) {
        self.offset.set(offset);
        self.scale.set(scale);
        self.auto_offset.set(auto_offset);
    }

    /// `set_filter_design` (`trigger_analog.py:226`).
    pub fn set_filter_design(&self, design: Option<Arc<DigitalFilter>>) {
        *self.design.borrow_mut() = design;
    }

    /// `self._xxx_cmd.send(args)`: the command is `None` until `_build_config` ran.
    fn send_cmd(cmd: Option<CommandWrapper>, args: Vec<PyValue>) -> KResult<()> {
        Self::send_cmd_opts(cmd, args, SendOpts::default())
    }

    /// `self._xxx_cmd.send(args, reqclock=...)`.
    fn send_cmd_opts(cmd: Option<CommandWrapper>, args: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        cmd.ok_or_else(none_has_no_send)?.send(args, opts)
    }

    /// `reset_filter` (`trigger_analog.py:230`).
    pub fn reset_filter(&self) -> KResult<()> {
        let sos_fixed = self.convert_filter()?;
        let sos_state = self.convert_state()?;
        let num_sections = sos_fixed.len() as i64;
        if num_sections > self.max_sections {
            return Err(KlippyError::Other {
                class_name: "ValueError".to_string(),
                message: percent(
                    "Too many filter sections: %i, The max is %i",
                    &[PctValue::Int(PyInt::from_i64(num_sections)), PctValue::Int(PyInt::from_i64(self.max_sections))],
                ),
            });
        }
        if sos_state.len() as i64 != num_sections {
            return Err(KlippyError::Other {
                class_name: "ValueError".to_string(),
                message: percent(
                    "The number of filter sections (%i) and state sections (%i) must be equal",
                    &[
                        PctValue::Int(PyInt::from_i64(num_sections)),
                        PctValue::Int(PyInt::from_i64(sos_state.len() as i64)),
                    ],
                ),
            });
        }
        for (i, section) in sos_fixed.iter().enumerate() {
            let args = vec![
                PyValue::Int(PyInt::from_i64(self.oid)),
                PyValue::Int(PyInt::from_i64(i as i64)),
                PyValue::Int(PyInt::from_i64(section[0])),
                PyValue::Int(PyInt::from_i64(section[1])),
                PyValue::Int(PyInt::from_i64(section[2])),
                PyValue::Int(PyInt::from_i64(section[3])),
                PyValue::Int(PyInt::from_i64(section[4])),
            ];
            if self.last_sent_coeffs.borrow().get(i).cloned().flatten().as_ref() == Some(&args) {
                continue;
            }
            Self::send_cmd(self.set_section_cmd.cloned(), args.clone())?;
            {
                let mut last = self.last_sent_coeffs.borrow_mut();
                if last.len() <= i {
                    last.resize_with(i + 1, || None);
                }
                last[i] = Some(args);
            }
        }
        for (i, state) in sos_state.iter().enumerate() {
            Self::send_cmd(
                self.set_state_cmd.cloned(),
                vec![
                    PyValue::Int(PyInt::from_i64(self.oid)),
                    PyValue::Int(PyInt::from_i64(i as i64)),
                    PyValue::Int(PyInt::from_i64(state[0])),
                    PyValue::Int(PyInt::from_i64(state[1])),
                ],
            )?;
        }
        let scale = self.scale.get();
        let scale_frac_bits = calc_frac_bits(&[scale])?;
        let su = to_fixed_32(scale, scale_frac_bits)?;
        let args = vec![
            PyValue::Int(PyInt::from_i64(self.oid)),
            PyValue::Int(PyInt::from_i64(self.offset.get())),
            PyValue::Int(PyInt::from_i64(su)),
            PyValue::Int(PyInt::from_i64(scale_frac_bits)),
            PyValue::Bool(self.auto_offset.get()),
        ];
        if args != self.last_sent_offset_scale.cloned().unwrap_or_default()
            || self.auto_offset.get()
        {
            Self::send_cmd(self.set_offset_scale_cmd.cloned(), args.clone())?;
            *self.last_sent_offset_scale.borrow_mut() = Some(args);
        }
        if self.max_sections != 0 {
            Self::send_cmd(
                self.set_active_cmd.cloned(),
                vec![
                    PyValue::Int(PyInt::from_i64(self.oid)),
                    PyValue::Int(PyInt::from_i64(num_sections)),
                    PyValue::Int(PyInt::from_i64(self.coeff_frac_bits.get())),
                ],
            )?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Trigger Analog
// ---------------------------------------------------------------------------

/// Sensor interface used by [`MCU_trigger_analog`] (`sensor_inst` in
/// Python): the load-cell / eddy-current sensor classes
/// (`hx71x`, `ads1220`, `ads131m0x`, `ldc1612`, `load_cell`, ...).
///
/// Only the methods `trigger_analog.py` calls are declared (same
/// snake_case names); sensor modules implement this trait on their
/// concrete ported types.
pub trait TriggerAnalogSensor: Send + Sync {
    /// `get_mcu()`
    fn get_mcu(&self) -> Mcu;
    /// `setup_trigger_analog(trigger_analog_oid)`
    fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()>;
    /// `get_samples_per_second()`
    fn get_samples_per_second(&self) -> f64;
    /// `lookup_sensor_error(error_code) -> str`. Fallible because some
    /// sensors raise from it (`ads1220.py:131` formats a string without a
    /// conversion specifier, a `TypeError`).
    fn lookup_sensor_error(&self, error_code: i64) -> KResult<String>;
}

/// `MCU_trigger_analog` (`trigger_analog.py:271`).
///
/// Doubles as the homing endstop of the probes (`homing.probing_move(
/// mcu_trigger_analog, ...)`), so it implements [`EndstopApi`].
#[allow(non_camel_case_types)]
pub struct MCU_trigger_analog {
    sensor: Arc<dyn TriggerAnalogSensor>,
    mcu: Mcu,
    sos_filter: GCell<Option<Arc<MCU_SosFilter>>>,
    dispatch: TriggerDispatch,
    last_trigger_time: GCell<f64>,
    raw_min: GCell<i64>,
    raw_max: GCell<i64>,
    last_range_args: GCell<Option<Vec<PyValue>>>,
    trigger_type: GCell<String>,
    trigger_value: GCell<PyValue>,
    last_trigger_args: GCell<Option<Vec<PyValue>>>,
    error_map: GCell<IndexMap<i64, String>>,
    sensor_specific_error: GCell<i64>,
    oid: i64,
    home_cmd: GCell<Option<CommandWrapper>>,
    query_state_cmd: GCell<Option<CommandQueryWrapper>>,
    set_raw_range_cmd: GCell<Option<CommandWrapper>>,
    set_trigger_cmd: GCell<Option<CommandWrapper>>,
}

/// `MCU_trigger_analog.MONITOR_MAX`.
pub const MONITOR_MAX: i64 = 3;
/// `MCU_trigger_analog.REASON_TRIGGER_ANALOG` (`trigger_analog.py:273`).
pub const REASON_TRIGGER_ANALOG: i64 = McuTrsync::REASON_COMMS_TIMEOUT + 1;

/// `None.send(...)`: a command that `_build_config` has not looked up yet.
fn none_has_no_send() -> KlippyError {
    KlippyError::Other {
        class_name: "AttributeError".to_string(),
        message: "'NoneType' object has no attribute 'send'".to_string(),
    }
}

impl MCU_trigger_analog {
    /// `MCU_trigger_analog(sensor_inst)` (`trigger_analog.py:274`).
    ///
    /// The dispatch is the Python `mcu.TriggerDispatch(mcu)` object
    /// (`trigger_analog.py:279`), created through
    /// [`McuApi::create_trigger_dispatch`] before the trigger-analog
    /// object allocates its own oid.
    pub fn new(sensor: Arc<dyn TriggerAnalogSensor>) -> KResult<Arc<Self>> {
        let mcu = sensor.get_mcu();
        let dispatch = mcu.create_trigger_dispatch()?;
        let oid = mcu.create_oid()?;
        let this = Arc::new(MCU_trigger_analog {
            sensor,
            mcu: mcu.clone(),
            sos_filter: GCell::new(None),
            dispatch,
            last_trigger_time: GCell::new(0.0),
            raw_min: GCell::new(0),
            raw_max: GCell::new(0),
            last_range_args: GCell::new(None),
            trigger_type: GCell::new("unspecified".to_string()),
            trigger_value: GCell::new(PyValue::Float(0.0)),
            last_trigger_args: GCell::new(None),
            error_map: GCell::new(IndexMap::new()),
            sensor_specific_error: GCell::new(0),
            oid,
            home_cmd: GCell::new(None),
            query_state_cmd: GCell::new(None),
            set_raw_range_cmd: GCell::new(None),
            set_trigger_cmd: GCell::new(None),
        });
        let this2 = this.clone();
        mcu.register_config_callback(Box::new(move || {
            this2.build_config()
        }))?;
        Ok(this)
    }

    /// `setup_sos_filter(sos_filter)` (`trigger_analog.py:297`).
    pub fn setup_sos_filter(&self, sos_filter: Arc<MCU_SosFilter>) {
        *self.sos_filter.borrow_mut() = Some(sos_filter);
    }

    /// `_build_config` (`trigger_analog.py:300`).
    pub fn build_config(&self) -> KResult<()> {
        self.sensor.setup_trigger_analog(self.oid)?;
        let cmd_queue = self.dispatch.get_command_queue();
        if self.sos_filter.cloned().is_none() {
            let filt = MCU_SosFilter::new(self.mcu.clone(), cmd_queue, 0)?;
            *self.sos_filter.borrow_mut() = Some(filt);
        }
        let sos_oid = self.sos_filter.cloned().expect("sos filter just created").get_oid();
        self.mcu.add_config_cmd(
            &percent(
                "config_trigger_analog oid=%d sos_filter_oid=%d",
                &[PctValue::Int(PyInt::from_i64(self.oid)), PctValue::Int(PyInt::from_i64(sos_oid))],
            ),
            AddConfigCmdOpts::default(),
        )?;
        let cq = Some(cmd_queue);
        *self.query_state_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "trigger_analog_query_state oid=%c",
            "trigger_analog_state oid=%c homing=%c homing_clock=%u",
            LookupQueryCommandOpts { oid: Some(self.oid), cq, is_async: false },
        )?);
        *self.set_raw_range_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "trigger_analog_set_raw_range oid=%c raw_min=%i raw_max=%i",
            cq,
        )?);
        *self.set_trigger_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "trigger_analog_set_trigger oid=%c trigger_analog_type=%c trigger_value=%i",
            cq,
        )?);
        *self.home_cmd.borrow_mut() = Some(self.mcu.lookup_command(
            "trigger_analog_home oid=%c trsync_oid=%c trigger_reason=%c error_reason=%c clock=%u monitor_ticks=%u monitor_max=%u",
            cq,
        )?);
        let errors = self.mcu.get_enumerations().get("trigger_analog_error:").cloned().unwrap_or_default();
        let mut map = IndexMap::new();
        for (k, v) in &errors {
            map.insert(*v, k.clone());
        }
        *self.error_map.borrow_mut() = map;
        *self.sensor_specific_error.borrow_mut() = errors.get("SENSOR_SPECIFIC").copied().unwrap_or(0);
        Ok(())
    }

    /// `get_oid` (`trigger_analog.py:328`).
    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    /// `get_mcu` (`trigger_analog.py:331`).
    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    /// `get_sos_filter` (`trigger_analog.py:334`).
    pub fn get_sos_filter(&self) -> Option<Arc<MCU_SosFilter>> {
        self.sos_filter.cloned()
    }

    /// `get_dispatch` (`trigger_analog.py:337`).
    pub fn get_dispatch(&self) -> TriggerDispatch {
        self.dispatch.clone()
    }

    /// `get_last_trigger_time` (`trigger_analog.py:340`).
    pub fn get_last_trigger_time(&self) -> f64 {
        self.last_trigger_time.get()
    }

    /// `set_trigger(trigger_type, trigger_value)` (`trigger_analog.py:343`).
    pub fn set_trigger(&self, trigger_type: String, trigger_value: PyValue) {
        self.trigger_type.replace(trigger_type);
        self.trigger_value.replace(trigger_value);
    }

    /// `set_raw_range(raw_min, raw_max)` (`trigger_analog.py:347`).
    pub fn set_raw_range(&self, raw_min: i64, raw_max: i64) {
        self.raw_min.set(raw_min);
        self.raw_max.set(raw_max);
    }

    /// `_reset_filter` (`trigger_analog.py:351`).
    pub fn reset_filter(&self) -> KResult<()> {
        let args = vec![
            PyValue::Int(PyInt::from_i64(self.oid)),
            PyValue::Int(PyInt::from_i64(self.raw_min.get())),
            PyValue::Int(PyInt::from_i64(self.raw_max.get())),
        ];
        if self.last_range_args.cloned() != Some(args.clone()) {
            MCU_SosFilter::send_cmd(self.set_raw_range_cmd.cloned(), args.clone())?;
            *self.last_range_args.borrow_mut() = Some(args);
        }
        let args = vec![
            PyValue::Int(PyInt::from_i64(self.oid)),
            PyValue::Str(self.trigger_type.cloned()),
            self.trigger_value.cloned(),
        ];
        if self.last_trigger_args.cloned() != Some(args.clone()) {
            MCU_SosFilter::send_cmd(self.set_trigger_cmd.cloned(), args.clone())?;
            *self.last_trigger_args.borrow_mut() = Some(args);
        }
        let sos_filter = self.sos_filter.cloned().ok_or_else(|| KlippyError::Other {
            class_name: "AttributeError".to_string(),
            message: "'NoneType' object has no attribute 'reset_filter'".to_string(),
        })?;
        sos_filter.reset_filter()
    }

    /// `_clear_home` (`trigger_analog.py:365`).
    pub fn clear_home(&self) -> KResult<f64> {
        let mut args = vec![PyValue::Int(PyInt::from_i64(self.oid))];
        args.extend((0..7).map(|_| PyValue::Int(PyInt::from_i64(0))));
        MCU_SosFilter::send_cmd(self.home_cmd.cloned(), args)?;
        let query = self.query_state_cmd.cloned().ok_or_else(none_has_no_send)?;
        let params = query.send(vec![PyValue::Int(PyInt::from_i64(self.oid))], QuerySendOpts::default())?;
        let homing_clock = dict_get_i64(&params, "homing_clock")?;
        let trigger_ticks = self.mcu.clock32_to_clock64(homing_clock);
        Ok(self.mcu.clock_to_print_time(trigger_ticks as f64))
    }

    /// `get_steppers` (`trigger_analog.py:371`).
    pub fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        self.dispatch.get_steppers()
    }

    /// `home_start` (`trigger_analog.py:374`).
    pub fn home_start(&self, print_time: f64) -> KResult<Completion<bool>> {
        self.last_trigger_time.set(0.0);
        self.reset_filter()?;
        let trigger_completion = self.dispatch.start(print_time);
        let clock = self.mcu.print_time_to_clock(print_time);
        let samples_per_second = self.sensor.get_samples_per_second();
        if samples_per_second == 0.0 {
            return Err(KlippyError::Other {
                class_name: "ZeroDivisionError".to_string(),
                message: "float division by zero".to_string(),
            });
        }
        let sensor_update = 1.0 / samples_per_second;
        let sm_ticks = self.mcu.seconds_to_clock(sensor_update);
        MCU_SosFilter::send_cmd_opts(
            self.home_cmd.cloned(),
            vec![
                PyValue::Int(PyInt::from_i64(self.oid)),
                PyValue::Int(PyInt::from_i64(self.dispatch.get_oid())),
                PyValue::Int(PyInt::from_i64(McuTrsync::REASON_ENDSTOP_HIT)),
                PyValue::Int(PyInt::from_i64(REASON_TRIGGER_ANALOG)),
                PyValue::Int(PyInt::from_i64(clock)),
                PyValue::Int(PyInt::from_i64(sm_ticks)),
                PyValue::Int(PyInt::from_i64(MONITOR_MAX)),
            ],
            SendOpts { minclock: 0, reqclock: clock },
        )?;
        Ok(trigger_completion)
    }

    /// `home_wait` (`trigger_analog.py:387`).
    pub fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.dispatch.wait_end(home_end_time);
        let res = self.dispatch.stop()?;
        let trigger_time = self.clear_home()?;
        if res >= McuTrsync::REASON_COMMS_TIMEOUT {
            if res == McuTrsync::REASON_COMMS_TIMEOUT {
                return Err(KlippyError::Command("Communication timeout during homing".to_string()));
            }
            let error_code = res - REASON_TRIGGER_ANALOG;
            if error_code >= self.sensor_specific_error.get() {
                let sensor_err = error_code - self.sensor_specific_error.get();
                let error_msg = self.sensor.lookup_sensor_error(sensor_err)?;
                return Err(KlippyError::Command(percent("Trigger analog error: %s", &[PctValue::Str(error_msg)])));
            }
            let error_msg = self
                .error_map
                .borrow()
                .get(&error_code)
                .cloned()
                .unwrap_or_else(|| percent("Unknown code %i", &[PctValue::Int(PyInt::from_i64(error_code))]));
            return Err(KlippyError::Command(percent("Trigger analog error: %s", &[PctValue::Str(error_msg)])));
        }
        if res != McuTrsync::REASON_ENDSTOP_HIT {
            return Ok(0.0);
        }
        let mut trigger_time = trigger_time;
        if self.mcu.is_fileoutput() {
            trigger_time = home_end_time;
        }
        self.last_trigger_time.set(trigger_time);
        Ok(trigger_time)
    }
}

impl EndstopApi for MCU_trigger_analog {
    fn get_mcu(&self) -> Mcu {
        MCU_trigger_analog::get_mcu(self)
    }

    /// `MCU_trigger_analog` has no `add_stepper`: the probes add steppers
    /// to `get_dispatch()` instead, so Python raises `AttributeError`.
    fn add_stepper(&self, _stepper: Stepper) {
        panic!("AttributeError: 'MCU_trigger_analog' object has no attribute 'add_stepper'");
    }

    fn get_steppers(&self) -> Vec<Stepper> {
        MCU_trigger_analog::get_steppers(self)
    }

    /// An error here is an uncaught Python exception (`ValueError`,
    /// `OverflowError`, ...), which the trait (no `KResult`) cannot carry.
    fn home_start(
        &self,
        print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        match MCU_trigger_analog::home_start(self, print_time) {
            Ok(completion) => completion,
            Err(e) => panic!("{}: {}", e.class_name(), e.message()),
        }
    }

    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        MCU_trigger_analog::home_wait(self, home_end_time)
    }

    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Err(KlippyError::Other {
            class_name: "AttributeError".to_string(),
            message: "'MCU_trigger_analog' object has no attribute 'query_endstop'".to_string(),
        })
    }
}

/// `params['homing_clock']` of a query response dict.
fn dict_get_i64(params: &PyValue, key: &str) -> KResult<i64> {
    if let PyValue::Dict(d) = params {
        if let Some(PyValue::Int(n)) = d.get(&PyValue::Str(key.to_string())) {
            if let Ok(v) = n.inner().to_string().parse() {
                return Ok(v);
            }
        }
    }
    Err(KlippyError::Other {
        class_name: "KeyError".to_string(),
        message: pycompat::text::str_repr(key),
    })
}

/// Module factory: the Python module defines no `load_config` /
/// `load_config_prefix` / `add_printer_objects`, so this is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
