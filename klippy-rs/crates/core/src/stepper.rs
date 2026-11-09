//! Port of `klippy/stepper.py` (`MCU_stepper`, `PrinterStepper`, `parse_step_distance`,
//! `GenericPrinterRail`, `LookupRail`, `LookupMultiRail`).
//!
//! The frozen contracts this module builds on are in `klippy_api` (`McuStepperApi`,
//! `RailApi`, `StepperFactoryApi`, `McuApi`, `PinsApi`, `ConfigApi`, `PrinterApi`,
//! `MotionQueuingApi`, `SyncEmitterApi`). `stepper.py` has no `load_config` /
//! `load_config_prefix` / `add_printer_objects`, so [`module_factory`] is empty; the
//! module functions reach other modules through [`StepperFactory`]
//! (`ToolHeadApi::stepper_factory()`).
//!
//! Chelper objects: the `itersolve_*` calls run on the real `chelper_motion`
//! `StepperKinematics` behind [`crate::motion_handles::StepperKinematicsHandle`], the
//! trapq handed to `set_trapq` is the [`crate::motion_handles::TrapqHandle`] one, and
//! the stepcompressor is the syncemitter's own (`syncemitter_get_stepcompress`,
//! `klippy/stepper.py:50`), driven through [`SyncEmitterApi`]. `set_stepper_kinematics`
//! registers the kinematics on the emitter (`stepper.py:200`), so allocation
//! (`*_stepper_alloc`, `input_shaper_alloc`, ...) works unchanged.
//!
//! Python raises cffi and `AttributeError` exceptions from methods whose contract
//! signature has no `KResult` (`setup_itersolve`, `set_stepper_kinematics`, `set_trapq`,
//! `set_position`, `calc_position_from_coord`, `get_commanded_position`,
//! `get_mcu_position`, `is_active_axis`, the flush callback). Those paths panic with the
//! Python exception class and text.

use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetBoolOpts, GetFloatOpts, GetIntOpts, GetListsOpts, GetStrOpts, ListParser,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueryWrapper, EndstopApi, LookupPinOpts, LookupQueryCommandOpts,
    Mcu, ParsePinOpts, Pin, PinParams, PinsApi,
};
use klippy_api::object::{load_interface, lookup_interface, EventArg};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT, EV_STEPPER_SET_DIR_INVERTED,
    EV_STEPPER_SYNC_MCU_POSITION,
};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, QueryEndstopsApi, RailApi, RailOpts, Stepper,
    StepperFactoryApi, StepperKinematics, StepperRegistryApi, Trapq,
};
use klippy_api::toolhead::{FlushCallback, MotionQueuingApi, SyncEmitterHandle};
use klippy_api::GCell;
use pycompat::text::{float_true_div, percent_format, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};

use crate::motion_handles::{alloc_stepper_kinematics, kinematics_of, trapq_of};

/// `MIN_BOTH_EDGE_DURATION` (`klippy/stepper.py:17`).
pub const MIN_BOTH_EDGE_DURATION: f64 = 0.000000500;
/// `MIN_OPTIMIZED_BOTH_EDGE_DURATION` (`klippy/stepper.py:18`).
pub const MIN_OPTIMIZED_BOTH_EDGE_DURATION: f64 = 0.000000150;
/// `MAX_STEPCOMPRESS_ERROR` (`klippy/stepper.py:19`).
pub const MAX_STEPCOMPRESS_ERROR: f64 = 0.000025;

// ---------------------------------------------------------------------------
// Error and formatting helpers.
// ---------------------------------------------------------------------------

/// `stepper.py:9` `class error(Exception)`.
pub fn stepper_error(message: impl Into<String>) -> KlippyError {
    KlippyError::Other { class_name: "error".to_string(), message: message.into() }
}

fn py_exception(kind: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: kind.to_string(), message: message.into() }
}

fn from_pyerr(e: pycompat::PyError) -> KlippyError {
    py_exception(e.kind, e.msg)
}

/// A Python exception on a path whose contract signature cannot return it.
fn raise<T>(result: KResult<T>) -> T {
    result.unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message()))
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_format(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(from_pyerr)
}

/// `int(constants.get(name, '0'))` (`klippy/stepper.py:84-86`).
fn const_int(constants: &IndexMap<String, String>, name: &str) -> KResult<i64> {
    let text = constants.get(name).map(String::as_str).unwrap_or("0");
    let parsed = pycompat::text::int_from_string(text, 10).map_err(from_pyerr)?;
    Ok(pyint_to_i64(&parsed))
}

/// The trait surface is `i64` while Python ints are unbounded; values beyond the
/// range clamp.
fn pyint_to_i64(n: &PyInt) -> i64 {
    n.0.to_string().parse::<i64>().unwrap_or_else(|_| if n.sign() < 0 { i64::MIN } else { i64::MAX })
}

/// Python `int(float)` (truncation toward zero); NaN and infinity raise in Python.
fn py_int_from_float(f: f64) -> i64 {
    if f.is_nan() {
        panic!("ValueError: cannot convert float NaN to integer");
    }
    if f.is_infinite() {
        panic!("OverflowError: cannot convert float infinity to integer");
    }
    pyint_to_i64(&trunc_float_to_bigint(f))
}

/// `a / b` with CPython's `ZeroDivisionError`.
fn py_div(a: f64, b: f64) -> KResult<f64> {
    float_true_div(a, b).map_err(from_pyerr)
}

/// `a is b` for two MCU handles.
fn same_mcu(a: &Mcu, b: &Mcu) -> bool {
    std::ptr::eq(Arc::as_ptr(a) as *const (), Arc::as_ptr(b) as *const ())
}

// ---------------------------------------------------------------------------
// Cross-module service lookup (contract v2: `lookup_interface` / `load_interface`).
// ---------------------------------------------------------------------------

fn load_motion_queuing(printer: &Printer, config: &Config) -> KResult<Arc<dyn MotionQueuingApi>> {
    load_interface::<dyn MotionQueuingApi>(&**printer, config, "motion_queuing", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::Config("Unknown config object 'motion_queuing'".to_string()))
}

fn lookup_motion_queuing(printer: &Printer) -> KResult<Arc<dyn MotionQueuingApi>> {
    lookup_interface::<dyn MotionQueuingApi>(&**printer, "motion_queuing", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::Config("Unknown config object 'motion_queuing'".to_string()))
}

fn load_registrar(
    printer: &Printer,
    config: &Config,
    name: &str,
) -> KResult<Arc<dyn StepperRegistryApi>> {
    load_interface::<dyn StepperRegistryApi>(&**printer, config, name, LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::Config(format!("Unknown config object '{name}'")))
}

fn load_query_endstops(printer: &Printer, config: &Config) -> KResult<Arc<dyn QueryEndstopsApi>> {
    load_interface::<dyn QueryEndstopsApi>(&**printer, config, "query_endstops", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::Config("Unknown config object 'query_endstops'".to_string()))
}

fn lookup_pins(printer: &Printer) -> KResult<Arc<dyn PinsApi>> {
    lookup_interface::<dyn PinsApi>(&**printer, "pins", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::Config("Unknown config object 'pins'".to_string()))
}

fn deprecate_mcu_code(printer: &Printer, mcu: &Mcu, feature: &str) -> KResult<()> {
    let configfile =
        lookup_interface::<dyn PrinterConfigApi>(&**printer, "configfile", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Config("Unknown config object 'configfile'".to_string()))?;
    configfile.deprecate_mcu_code(mcu, feature, None);
    Ok(())
}

// ---------------------------------------------------------------------------
// MCU_stepper.
// ---------------------------------------------------------------------------

struct StepperState {
    name: String,
    rotation_dist: f64,
    steps_per_rotation: f64,
    step_pulse_duration: Option<f64>,
    step_dist: f64,
    step_pin: String,
    invert_step: i64,
    dir_pin: String,
    invert_dir: bool,
    orig_invert_dir: bool,
    step_both_edge: bool,
    req_step_both_edge: bool,
    mcu_position_offset: f64,
    reset_cmd_tag: Option<i64>,
    get_position_cmd: Option<CommandQueryWrapper>,
    active_callbacks: Vec<Box<dyn FnMut(f64) + Send>>,
    flush_callback: Option<FlushCallback>,
    syncemitter: Option<SyncEmitterHandle>,
    stepper_kinematics: Option<StepperKinematics>,
    trapq: Option<Trapq>,
}

/// Port of `MCU_stepper` (`klippy/stepper.py:22`).
pub struct McuStepper {
    mcu: Mcu,
    oid: i64,
    units_in_radians: bool,
    me: Weak<McuStepper>,
    state: GCell<StepperState>,
}

klippy_api::printer_object!(McuStepper; dyn McuStepperApi);

impl McuStepper {
    /// `MCU_stepper.__init__` (`klippy/stepper.py:23-57`).
    pub fn new(
        config: &Config,
        step_pin_params: PinParams,
        dir_pin_params: PinParams,
        rotation_dist: f64,
        steps_per_rotation: f64,
        step_pulse_duration: Option<f64>,
        units_in_radians: bool,
    ) -> KResult<Arc<McuStepper>> {
        let name = config.get_name();
        let step_dist = py_div(rotation_dist, steps_per_rotation)?;
        let mcu = step_pin_params.chip.clone();
        let oid = mcu.create_oid()?;
        let stepper = Arc::new_cyclic(|me| McuStepper {
            mcu: mcu.clone(),
            oid,
            units_in_radians,
            me: me.clone(),
            state: GCell::new(StepperState {
                name,
                rotation_dist,
                steps_per_rotation,
                step_pulse_duration,
                step_dist,
                step_pin: step_pin_params.pin.clone(),
                invert_step: step_pin_params.invert,
                dir_pin: dir_pin_params.pin.clone(),
                invert_dir: dir_pin_params.invert != 0,
                orig_invert_dir: dir_pin_params.invert != 0,
                step_both_edge: false,
                req_step_both_edge: false,
                mcu_position_offset: 0.,
                reset_cmd_tag: None,
                get_position_cmd: None,
                active_callbacks: Vec::new(),
                flush_callback: None,
                syncemitter: None,
                stepper_kinematics: None,
                trapq: None,
            }),
        });
        let config_owner = stepper.clone();
        mcu.register_config_callback(Box::new(move || config_owner.build_config()))?;
        let printer = mcu.get_printer();
        if !same_mcu(&dir_pin_params.chip, &mcu) {
            return Err(KlippyError::Config(
                "Stepper dir pin must be on same mcu as step pin".to_string(),
            ));
        }
        let motion_queuing = load_motion_queuing(&printer, config)?;
        let name = stepper.state.borrow().name.clone();
        let syncemitter = motion_queuing.allocate_syncemitter(mcu.clone(), &name, true)?;
        // `stepcompress_set_invert_sdir` (`stepper.py:52`) on the
        // syncemitter's own compressor (`stepper.py:50-51`).
        let invert_dir = stepper.state.borrow().invert_dir;
        syncemitter.stepcompress_set_invert_sdir(u32::from(invert_dir));
        stepper.state.with_mut(|st| {
            st.syncemitter = Some(syncemitter);
        });
        let connect_owner = stepper.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args| {
                connect_owner.query_mcu_position()?;
                Ok(PyValue::None)
            }),
        );
        Ok(stepper)
    }

    fn printer(&self) -> Printer {
        self.mcu.get_printer()
    }

    fn self_object(&self) -> klippy_api::ObjectRef {
        self.me.upgrade().expect("McuStepper is alive while one of its methods runs")
    }

    /// The chelper `stepper_kinematics` pointer Python passes to `itersolve_*`; `None`
    /// is a cffi `TypeError`.
    fn sk(&self) -> StepperKinematics {
        match self.state.borrow().stepper_kinematics.clone() {
            Some(sk) => sk,
            None => panic!(
                "TypeError: initializer for ctype 'struct stepper_kinematics *' must be a cdata \
                 pointer, not NoneType"
            ),
        }
    }

    /// Run an `itersolve_*` call on the stepper's `struct stepper_kinematics`.
    fn with_sk<R>(&self, f: impl FnOnce(&mut chelper_motion::itersolve::StepperKinematics) -> R) -> R {
        let sk = self.sk();
        raise(kinematics_of(&sk)).with_sk(f)
    }

    /// The syncemitter allocated in `new` (`stepper.py:48`).
    fn syncemitter(&self) -> SyncEmitterHandle {
        self.state
            .borrow()
            .syncemitter
            .clone()
            .expect("syncemitter is allocated by McuStepper::new")
    }

    /// `_build_config` (`klippy/stepper.py:78-131`).
    fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;
        let oid = self.oid;
        self.state.with_mut(|st| {
            if st.step_pulse_duration.is_none() {
                st.step_pulse_duration = Some(0.000002);
            }
        });
        let (step_pin, dir_pin, req_step_both_edge, mut pulse, mut invert_step) = {
            let st = self.state.borrow();
            (
                st.step_pin.clone(),
                st.dir_pin.clone(),
                st.req_step_both_edge,
                st.step_pulse_duration.expect("set above"),
                st.invert_step,
            )
        };
        // Check if can enable "step on both edges"
        let constants = mcu.get_constants();
        let ssbe = const_int(&constants, "STEPPER_STEP_BOTH_EDGE")?;
        let sbe = const_int(&constants, "STEPPER_BOTH_EDGE")?;
        let sou = const_int(&constants, "STEPPER_OPTIMIZED_UNSTEP")?;
        // Every disabling condition of `stepper.py:88-101` leaves `want_both_edges` False:
        // a very large pulse duration (rise and fall times may not be symmetric), an older
        // MCU with a large pulse duration, an MCU without step on both edges, or an MCU with
        // optimized step/unstep.
        let disabled = pulse > MIN_BOTH_EDGE_DURATION
            || (sbe != 0 && pulse > MIN_OPTIMIZED_BOTH_EDGE_DURATION)
            || (sbe == 0 && ssbe == 0)
            || sou != 0;
        let want_both_edges = req_step_both_edge && !disabled;
        if ssbe == 0 {
            deprecate_mcu_code(&self.printer(), mcu, "STEPPER_STEP_BOTH_EDGE")?;
        }
        if want_both_edges {
            invert_step = -1;
            self.state.with_mut(|st| st.step_both_edge = true);
            if sbe != 0 {
                // Older MCU requires setting step_pulse_ticks=0 to enable
                pulse = 0.;
                self.state.with_mut(|st| st.step_pulse_duration = Some(0.));
            }
        }
        // Configure stepper object
        let step_pulse_ticks = mcu.seconds_to_clock(pulse);
        let config_cmd = pct_format(
            "config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%u",
            &[
                pct_int(oid),
                PctValue::Str(step_pin),
                PctValue::Str(dir_pin),
                pct_int(invert_step),
                pct_int(step_pulse_ticks),
            ],
        )?;
        mcu.add_config_cmd(&config_cmd, AddConfigCmdOpts::default())?;
        let reset_cmd = pct_format("reset_step_clock oid=%d clock=0", &[pct_int(oid)])?;
        mcu.add_config_cmd(&reset_cmd, AddConfigCmdOpts { is_init: false, on_restart: true })?;
        let step_cmd_tag = mcu
            .lookup_command("queue_step oid=%c interval=%u count=%hu add=%hi", None)?
            .get_command_tag();
        let dir_cmd_tag =
            mcu.lookup_command("set_next_step_dir oid=%c dir=%c", None)?.get_command_tag();
        let reset_cmd_tag =
            mcu.lookup_command("reset_step_clock oid=%c clock=%u", None)?.get_command_tag();
        let get_position_cmd = mcu.lookup_query_command(
            "stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i",
            LookupQueryCommandOpts { oid: Some(oid), cq: None, is_async: false },
        )?;
        let max_error_ticks = mcu.seconds_to_clock(MAX_STEPCOMPRESS_ERROR);
        let syncemitter = self.syncemitter();
        self.state.with_mut(|st| {
            st.reset_cmd_tag = Some(reset_cmd_tag);
            st.get_position_cmd = Some(get_position_cmd);
        });
        // `stepcompress_fill` (`stepper.py:130`) on the syncemitter's own
        // compressor (`stepper.py:50-51`).
        syncemitter.stepcompress_fill(
            oid as u32,
            max_error_ticks as u32,
            step_cmd_tag as i32,
            dir_cmd_tag as i32,
        );
        Ok(())
    }

    /// `_query_mcu_position` (`klippy/stepper.py:212-227`).
    ///
    /// Pauses the reactor.
    fn query_mcu_position(&self) -> KResult<()> {
        if self.mcu.is_fileoutput() {
            return Ok(());
        }
        let get_position_cmd = self
            .state
            .borrow()
            .get_position_cmd
            .clone()
            .expect("TypeError: 'NoneType' object has no attribute 'send'");
        let params = get_position_cmd.send(vec![pyvalue_int(self.oid)], Default::default())?;
        let mut last_pos = dict_int(&params, "pos")?;
        if self.state.borrow().invert_dir {
            last_pos = -last_pos;
        }
        let print_time = self.mcu.estimated_print_time(dict_float(&params, "#receive_time")?);
        let clock = self.mcu.print_time_to_clock(print_time);
        // `stepcompress_set_last_position` (`stepper.py:222`) on the
        // syncemitter's compressor.
        let ret = self.syncemitter().stepcompress_set_last_position(clock as u64, last_pos);
        if ret != 0 {
            return Err(stepper_error("Internal error in stepcompress"));
        }
        self.set_mcu_position(last_pos);
        self.printer()
            .send_event(EV_STEPPER_SYNC_MCU_POSITION, &[EventArg::Object(self.self_object())])?;
        Ok(())
    }

    /// `_set_mcu_position` (`klippy/stepper.py:175-177`).
    fn set_mcu_position(&self, mcu_pos: i64) {
        let step_dist = self.state.borrow().step_dist;
        let mcu_pos_dist = mcu_pos as f64 * step_dist;
        let offset = mcu_pos_dist - self.get_commanded_position();
        self.state.with_mut(|st| st.mcu_position_offset = offset);
    }

    /// `_check_active` (`klippy/stepper.py:245-259`).
    fn check_active(&self, _must_flush_time: f64, max_step_gen_time: f64) -> KResult<()> {
        let ret = self.with_sk(|sk| sk.check_active(max_step_gen_time));
        if ret == 0.0 {
            // Stepper motor still not active
            return Ok(());
        }
        // Motor is active, disable future checking
        let motion_queuing = lookup_motion_queuing(&self.printer())?;
        let (flush_callback, callbacks) = self.state.with_mut(|st| {
            (st.flush_callback.clone(), std::mem::take(&mut st.active_callbacks))
        });
        if let Some(flush_callback) = flush_callback {
            motion_queuing.unregister_flush_callback(&flush_callback);
        }
        // Invoke callbacks
        for mut cb in callbacks {
            cb(ret);
        }
        Ok(())
    }
}

fn pyvalue_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn dict_item<'a>(params: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match params {
        PyValue::Dict(d) => d
            .get(&PyValue::Str(key.to_string()))
            .ok_or_else(|| py_exception("KeyError", pycompat::value::repr_key(key))),
        _ => Err(py_exception("TypeError", "response params are not a dict")),
    }
}

fn dict_int(params: &PyValue, key: &str) -> KResult<i64> {
    match dict_item(params, key)? {
        PyValue::Int(n) => Ok(pyint_to_i64(n)),
        PyValue::Bool(b) => Ok(i64::from(*b)),
        other => Err(py_exception("TypeError", format!("'{key}' is not an int: {}", other.repr()))),
    }
}

fn dict_float(params: &PyValue, key: &str) -> KResult<f64> {
    match dict_item(params, key)? {
        PyValue::Float(f) => Ok(*f),
        PyValue::Int(n) => Ok(n.to_f64_exact().unwrap_or_else(|| pyint_to_i64(n) as f64)),
        other => Err(py_exception("TypeError", format!("'{key}' is not a number: {}", other.repr()))),
    }
}

impl McuStepperApi for McuStepper {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    fn get_name(&self, short: Option<bool>) -> String {
        let name = self.state.borrow().name.clone();
        if short.unwrap_or(false) && name.starts_with("stepper") {
            // Skip an extra symbol after 'stepper'
            return name.chars().skip(8).collect();
        }
        name
    }

    fn get_oid(&self) -> i64 {
        self.oid
    }

    fn units_in_radians(&self) -> bool {
        self.units_in_radians
    }

    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        let st = self.state.borrow();
        (st.step_pulse_duration, st.step_both_edge)
    }

    fn setup_default_pulse_duration(&self, pulse_duration: f64, step_both_edge: bool) {
        self.state.with_mut(|st| {
            if st.step_pulse_duration.is_none() {
                st.step_pulse_duration = Some(pulse_duration);
            }
            st.req_step_both_edge = step_both_edge;
        });
    }

    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        let sk = raise(alloc_stepper_kinematics(alloc_func, params));
        self.set_stepper_kinematics(Some(sk));
    }

    fn get_step_dist(&self) -> f64 {
        self.state.borrow().step_dist
    }

    fn get_rotation_distance(&self) -> (f64, f64) {
        let st = self.state.borrow();
        (st.rotation_dist, st.steps_per_rotation)
    }

    fn set_rotation_distance(&self, rotation_dist: f64) {
        let mcu_pos = self.get_mcu_position(None);
        let steps_per_rotation = self.state.borrow().steps_per_rotation;
        let step_dist = raise(py_div(rotation_dist, steps_per_rotation));
        self.state.with_mut(|st| {
            st.rotation_dist = rotation_dist;
            st.step_dist = step_dist;
        });
        let trapq = self.state.borrow().trapq.clone();
        self.set_trapq(trapq);
        self.set_mcu_position(mcu_pos);
    }

    fn get_dir_inverted(&self) -> (bool, bool) {
        let st = self.state.borrow();
        (st.invert_dir, st.orig_invert_dir)
    }

    fn set_dir_inverted(&self, invert_dir: bool) -> KResult<()> {
        if invert_dir == self.state.borrow().invert_dir {
            return Ok(());
        }
        self.state.with_mut(|st| {
            st.invert_dir = invert_dir;
        });
        // `stepcompress_set_invert_sdir` (`stepper.py:152`) on the
        // syncemitter's compressor.
        self.syncemitter().stepcompress_set_invert_sdir(u32::from(invert_dir));
        self.printer()
            .send_event(EV_STEPPER_SET_DIR_INVERTED, &[EventArg::Object(self.self_object())])?;
        Ok(())
    }

    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64 {
        self.with_sk(|sk| sk.calc_position_from_coord(coord[0], coord[1], coord[2]))
    }

    fn set_position(&self, coord: [f64; 3]) {
        let mcu_pos = self.get_mcu_position(None);
        self.with_sk(|sk| sk.set_position(coord[0], coord[1], coord[2]));
        self.set_mcu_position(mcu_pos);
    }

    fn get_commanded_position(&self) -> f64 {
        self.with_sk(|sk| sk.get_commanded_pos())
    }

    fn get_mcu_position(&self, cmd_pos: Option<f64>) -> i64 {
        let cmd_pos = match cmd_pos {
            Some(p) => p,
            None => self.get_commanded_position(),
        };
        let (offset, step_dist) = {
            let st = self.state.borrow();
            (st.mcu_position_offset, st.step_dist)
        };
        let mcu_pos_dist = cmd_pos + offset;
        let mcu_pos = raise(py_div(mcu_pos_dist, step_dist));
        if mcu_pos >= 0. {
            return py_int_from_float(mcu_pos + 0.5);
        }
        py_int_from_float(mcu_pos - 0.5)
    }

    fn get_past_mcu_position(&self, print_time: f64) -> i64 {
        let clock = self.mcu.print_time_to_clock(print_time);
        // `stepcompress_find_past_position` (`stepper.py:181`) on the
        // syncemitter's compressor.
        self.syncemitter().stepcompress_find_past_position(clock as u64)
    }

    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        let st = self.state.borrow();
        mcu_pos as f64 * st.step_dist - st.mcu_position_offset
    }

    fn dump_steps(&self, count: i64, start_clock: i64, end_clock: i64) -> (Vec<PullHistoryStep>, i64) {
        // `stepcompress_extract_old` (`stepper.py:188`) on the syncemitter's
        // compressor.
        let steps = self.syncemitter().stepcompress_extract_old(
            count as i32,
            start_clock as u64,
            end_clock as u64,
        );
        let count = steps.len() as i64;
        (steps, count)
    }

    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        self.state.borrow().stepper_kinematics.clone()
    }

    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        let old_sk = self.state.borrow().stepper_kinematics.clone();
        let mut mcu_pos = 0;
        if old_sk.is_some() {
            mcu_pos = self.get_mcu_position(None);
        }
        self.state.with_mut(|st| st.stepper_kinematics = sk.clone());
        // `syncemitter_set_stepper_kinematics` (`stepper.py:200`) so the
        // emitter's background thread generates steps for it.
        self.syncemitter().set_stepper_kinematics(sk);
        let trapq = self.state.borrow().trapq.clone();
        self.set_trapq(trapq);
        self.set_mcu_position(mcu_pos);
        old_sk
    }

    fn note_homing_end(&self) -> KResult<()> {
        // `stepcompress_reset` (`stepper.py:206`) on the syncemitter's
        // compressor.
        let ret = self.syncemitter().stepcompress_reset(0);
        if ret != 0 {
            return Err(stepper_error("Internal error in stepcompress"));
        }
        let (reset_cmd_tag, syncemitter) = {
            let st = self.state.borrow();
            (st.reset_cmd_tag, st.syncemitter.clone())
        };
        let reset_cmd_tag = reset_cmd_tag.unwrap_or_else(|| {
            panic!("TypeError: an integer is required (got type NoneType)")
        });
        let syncemitter = syncemitter.expect("syncemitter is allocated by McuStepper::new");
        let data = [reset_cmd_tag as u32, self.oid as u32, 0];
        // `syncemitter_queue_msg` (`stepper.py:210`) on the stepper's own
        // emitter.
        syncemitter.queue_msg(0, &data);
        self.query_mcu_position()
    }

    fn get_trapq(&self) -> Option<Trapq> {
        self.state.borrow().trapq.clone()
    }

    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        let step_dist = self.state.borrow().step_dist;
        let shared = tq.as_ref().map(|tq| raise(trapq_of(tq)).shared().clone());
        self.with_sk(|sk| sk.set_trapq(shared, step_dist));
        self.state.with_mut(|st| std::mem::replace(&mut st.trapq, tq))
    }

    fn add_active_callback(&self, cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        let first = self.state.with_mut(|st| {
            st.active_callbacks.push(cb);
            st.active_callbacks.len() == 1
        });
        if first {
            let motion_queuing = lookup_motion_queuing(&self.printer())?;
            let me = self.me.clone();
            let flush_callback: FlushCallback = Arc::new(move |must_flush_time, max_step_gen_time| {
                if let Some(stepper) = me.upgrade() {
                    stepper.check_active(must_flush_time, max_step_gen_time)?;
                }
                Ok(())
            });
            self.state.with_mut(|st| st.flush_callback = Some(flush_callback.clone()));
            // `register_flush_callback(self._check_active)` (`stepper.py:244`):
            // the reported name is the bound method's `__qualname__`
            // (`motion_queuing.py:35-42` resolves `_check_active` on the
            // stepper, whose `_name` is the config section name).
            let name = self.state.borrow().name.clone();
            motion_queuing.register_flush_callback_named(
                flush_callback,
                false,
                Some("_check_active"),
                Some(&name),
            );
        }
        Ok(())
    }

    fn is_active_axis(&self, axis: char) -> bool {
        // `axis.encode()` must be one byte for cffi's `char` parameter.
        if !axis.is_ascii() {
            panic!("TypeError: initializer for ctype 'char' must be a bytes of length 1");
        }
        self.with_sk(|sk| sk.is_active_axis(axis as u8)) != 0
    }
}

// ---------------------------------------------------------------------------
// PrinterStepper / parse_step_distance / parse_gear_ratio.
// ---------------------------------------------------------------------------

fn gear_pair(v: &PyValue) -> KResult<(f64, f64)> {
    let items: &[PyValue] = match v {
        PyValue::Tuple(items) | PyValue::List(items) => items,
        other => {
            return Err(py_exception(
                "TypeError",
                format!("cannot unpack non-iterable {} object", pycompat::value::type_name(other)),
            ))
        }
    };
    let float = |v: &PyValue| match v {
        PyValue::Float(f) => Ok(*f),
        PyValue::Int(n) => Ok(n.to_f64_exact().unwrap_or_else(|| pyint_to_i64(n) as f64)),
        other => Err(py_exception(
            "TypeError",
            format!("unsupported operand type(s) for /: '{}'", pycompat::value::type_name(other)),
        )),
    };
    match items {
        [g1, g2] => Ok((float(g1)?, float(g2)?)),
        _ if items.len() > 2 => {
            Err(py_exception("ValueError", "too many values to unpack (expected 2)"))
        }
        _ => Err(py_exception(
            "ValueError",
            format!("not enough values to unpack (expected 2, got {})", items.len()),
        )),
    }
}

/// `parse_gear_ratio` (`klippy/stepper.py:288-294`).
pub fn parse_gear_ratio(config: &Config, note_valid: bool) -> KResult<f64> {
    let gear_ratio = config.getlists(
        "gear_ratio",
        GetListsOpts {
            default: Some(Some(PyValue::Tuple(Vec::new()))),
            seps: vec![":".to_string(), ",".to_string()],
            count: Some(2),
            parser: ListParser::Float,
            note_valid,
        },
    )?;
    let mut result = 1.;
    let pairs: &[PyValue] = match &gear_ratio {
        PyValue::Tuple(items) | PyValue::List(items) => items,
        other => {
            return Err(py_exception(
                "TypeError",
                format!("'{}' object is not iterable", pycompat::value::type_name(other)),
            ))
        }
    };
    for pair in pairs {
        let (g1, g2) = gear_pair(pair)?;
        result *= py_div(g1, g2)?;
    }
    Ok(result)
}

/// `parse_step_distance` (`klippy/stepper.py:297-318`). Returns
/// `(rotation_dist, steps_per_rotation)`.
pub fn parse_step_distance(
    config: &Config,
    units_in_radians: Option<bool>,
    note_valid: bool,
) -> KResult<(f64, f64)> {
    // Check rotation_distance and gear_ratio
    let units_in_radians = match units_in_radians {
        Some(units) => units,
        None => {
            // Caller doesn't know if units are in radians - infer it
            let rd = config.get(
                "rotation_distance",
                GetStrOpts { default: Some(None), note_valid: false },
            )?;
            let gr =
                config.get("gear_ratio", GetStrOpts { default: Some(None), note_valid: false })?;
            rd.is_none() && gr.is_some()
        }
    };
    let rotation_dist = if units_in_radians {
        config.get("gear_ratio", GetStrOpts { default: None, note_valid })?;
        2. * std::f64::consts::PI
    } else {
        config
            .getfloat(
                "rotation_distance",
                GetFloatOpts { default: None, above: Some(0.), note_valid, ..Default::default() },
            )?
            .expect("a required option is returned or raises")
    };
    // Check microsteps and full_steps_per_rotation
    let microsteps = config
        .getint(
            "microsteps",
            GetIntOpts { default: None, minval: Some(1), note_valid, ..Default::default() },
        )?
        .expect("a required option is returned or raises");
    let full_steps = config
        .getint(
            "full_steps_per_rotation",
            GetIntOpts { default: Some(Some(200)), minval: Some(1), note_valid, ..Default::default() },
        )?
        .expect("a default is configured");
    if full_steps % 4 != 0 {
        return Err(KlippyError::Config(pct_format(
            "full_steps_per_rotation invalid in section '%s'",
            &[PctValue::Str(config.get_name())],
        )?));
    }
    let gearing = parse_gear_ratio(config, note_valid)?;
    let int_steps = i128::from(full_steps) * i128::from(microsteps);
    Ok((rotation_dist, int_steps as f64 * gearing))
}

/// `PrinterStepper` (`klippy/stepper.py:266-285`).
pub fn printer_stepper(config: &Config, units_in_radians: bool) -> KResult<Stepper> {
    let printer = config.get_printer();
    // Stepper definition
    let ppins = lookup_pins(&printer)?;
    let step_pin = config.get("step_pin", GetStrOpts::default())?.expect("required");
    let step_pin_params = ppins
        .lookup_pin(&step_pin, LookupPinOpts { can_invert: true, can_pullup: false, share_type: None })?;
    let dir_pin = config.get("dir_pin", GetStrOpts::default())?.expect("required");
    let dir_pin_params = ppins
        .lookup_pin(&dir_pin, LookupPinOpts { can_invert: true, can_pullup: false, share_type: None })?;
    let (rotation_dist, steps_per_rotation) =
        parse_step_distance(config, Some(units_in_radians), true)?;
    let step_pulse_duration = config.getfloat(
        "step_pulse_duration",
        GetFloatOpts {
            default: Some(None),
            minval: Some(0.),
            maxval: Some(0.001),
            ..Default::default()
        },
    )?;
    let mcu_stepper = McuStepper::new(
        config,
        step_pin_params,
        dir_pin_params,
        rotation_dist,
        steps_per_rotation,
        step_pulse_duration,
        units_in_radians,
    )?;
    // Register with helper modules
    for mname in ["stepper_enable", "force_move", "motion_report"] {
        let m = load_registrar(&printer, config, mname)?;
        m.register_stepper(config, mcu_stepper.clone())?;
    }
    Ok(mcu_stepper)
}

// ---------------------------------------------------------------------------
// GenericPrinterRail.
// ---------------------------------------------------------------------------

struct EndstopEntry {
    endstop: Arc<dyn EndstopApi>,
    invert: i64,
    pullup: i64,
}

struct RailState {
    stepper_units_in_radians: bool,
    printer: Printer,
    name: String,
    steppers: Vec<Stepper>,
    endstops: Vec<(Arc<dyn EndstopApi>, String)>,
    endstop_map: IndexMap<String, EndstopEntry>,
    endstop_pin: String,
    query_endstops: Option<Arc<dyn QueryEndstopsApi>>,
    position_endstop: f64,
    position_min: f64,
    position_max: f64,
    homing_speed: f64,
    second_homing_speed: f64,
    homing_retract_speed: f64,
    homing_retract_dist: f64,
    homing_positive_dir: bool,
}

/// Port of `GenericPrinterRail` (`klippy/stepper.py:327`).
pub struct GenericPrinterRail {
    state: GCell<RailState>,
}

klippy_api::printer_object!(GenericPrinterRail; dyn RailApi);

fn float_opt(default: Option<Option<f64>>) -> GetFloatOpts {
    GetFloatOpts { default, ..Default::default() }
}

impl GenericPrinterRail {
    /// `GenericPrinterRail.__init__` (`klippy/stepper.py:328-387`).
    pub fn new(config: &Config, opts: RailOpts) -> KResult<Arc<GenericPrinterRail>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let endstop_pin = config.get("endstop_pin", GetStrOpts::default())?.expect("required");
        let rail = Arc::new(GenericPrinterRail {
            state: GCell::new(RailState {
                stepper_units_in_radians: opts.units_in_radians,
                printer: printer.clone(),
                name: name.clone(),
                steppers: Vec::new(),
                endstops: Vec::new(),
                endstop_map: IndexMap::new(),
                endstop_pin: endstop_pin.clone(),
                query_endstops: None,
                position_endstop: 0.,
                position_min: 0.,
                position_max: 0.,
                homing_speed: 0.,
                second_homing_speed: 0.,
                homing_retract_speed: 0.,
                homing_retract_dist: 0.,
                homing_positive_dir: false,
            }),
        });
        // Primary endstop position
        let query_endstops = load_query_endstops(&printer, config)?;
        rail.state.with_mut(|st| st.query_endstops = Some(query_endstops));
        let mcu_endstop = rail.lookup_endstop_checked(Some(&endstop_pin), &name)?;
        let position_endstop = match mcu_endstop.get_position_endstop() {
            Some(position) => position,
            None => config
                .getfloat(
                    "position_endstop",
                    float_opt(opts.default_position_endstop.map(Some)),
                )?
                .expect("a required option is returned or raises"),
        };
        // Axis range
        let (position_min, position_max) = if opts.need_position_minmax {
            let position_min = config
                .getfloat("position_min", float_opt(Some(Some(0.))))?
                .expect("a default is configured");
            let position_max = config
                .getfloat(
                    "position_max",
                    GetFloatOpts { default: None, above: Some(position_min), ..Default::default() },
                )?
                .expect("a required option is returned or raises");
            (position_min, position_max)
        } else {
            (0., position_endstop)
        };
        if position_endstop < position_min || position_endstop > position_max {
            return Err(KlippyError::Config(pct_format(
                "position_endstop in section '%s' must be between position_min and position_max",
                &[PctValue::Str(config.get_name())],
            )?));
        }
        // Homing mechanics
        let homing_speed = config
            .getfloat(
                "homing_speed",
                GetFloatOpts { default: Some(Some(5.0)), above: Some(0.), ..Default::default() },
            )?
            .expect("a default is configured");
        let second_homing_speed = config
            .getfloat(
                "second_homing_speed",
                GetFloatOpts {
                    default: Some(Some(homing_speed / 2.)),
                    above: Some(0.),
                    ..Default::default()
                },
            )?
            .expect("a default is configured");
        let homing_retract_speed = config
            .getfloat(
                "homing_retract_speed",
                GetFloatOpts { default: Some(Some(homing_speed)), above: Some(0.), ..Default::default() },
            )?
            .expect("a default is configured");
        let homing_retract_dist = config
            .getfloat(
                "homing_retract_dist",
                GetFloatOpts { default: Some(Some(5.)), minval: Some(0.), ..Default::default() },
            )?
            .expect("a default is configured");
        let homing_positive_dir = match config
            .getboolean("homing_positive_dir", GetBoolOpts { default: Some(None), ..Default::default() })?
        {
            None => {
                let axis_len = position_max - position_min;
                let inferred = if position_endstop <= position_min + axis_len / 4. {
                    false
                } else if position_endstop >= position_max - axis_len / 4. {
                    true
                } else {
                    return Err(KlippyError::Config(pct_format(
                        "Unable to infer homing_positive_dir in section '%s'",
                        &[PctValue::Str(config.get_name())],
                    )?));
                };
                config.getboolean(
                    "homing_positive_dir",
                    GetBoolOpts { default: Some(Some(inferred)), ..Default::default() },
                )?;
                inferred
            }
            Some(positive_dir) => {
                if (positive_dir && position_endstop == position_min)
                    || (!positive_dir && position_endstop == position_max)
                {
                    return Err(KlippyError::Config(pct_format(
                        "Invalid homing_positive_dir / position_endstop in '%s'",
                        &[PctValue::Str(config.get_name())],
                    )?));
                }
                positive_dir
            }
        };
        rail.state.with_mut(|st| {
            st.position_endstop = position_endstop;
            st.position_min = position_min;
            st.position_max = position_max;
            st.homing_speed = homing_speed;
            st.second_homing_speed = second_homing_speed;
            st.homing_retract_speed = homing_retract_speed;
            st.homing_retract_dist = homing_retract_dist;
            st.homing_positive_dir = homing_positive_dir;
        });
        Ok(rail)
    }

    /// `lookup_endstop` (`klippy/stepper.py:409-433`).
    fn lookup_endstop_checked(
        &self,
        endstop_pin: Option<&str>,
        name: &str,
    ) -> KResult<Arc<dyn EndstopApi>> {
        let endstop_pin = endstop_pin.ok_or_else(|| {
            py_exception("AttributeError", "'NoneType' object has no attribute 'strip'")
        })?;
        let printer = self.state.borrow().printer.clone();
        let ppins = lookup_pins(&printer)?;
        let pin_params =
            ppins.parse_pin(endstop_pin, ParsePinOpts { can_invert: true, can_pullup: true })?;
        // Normalize pin name
        let pin_name = pct_format(
            "%s:%s",
            &[PctValue::Str(pin_params.chip_name.clone()), PctValue::Str(pin_params.pin.clone())],
        )?;
        // Look for already-registered endstop
        let known = self
            .state
            .borrow()
            .endstop_map
            .get(&pin_name)
            .map(|e| (e.endstop.clone(), e.invert, e.pullup));
        match known {
            None => {
                // New endstop, register it
                let mcu_endstop = match ppins.setup_pin("endstop", endstop_pin)? {
                    Pin::Endstop(endstop) => endstop,
                    _ => {
                        return Err(py_exception(
                            "AttributeError",
                            "setup_pin('endstop') did not return an endstop",
                        ))
                    }
                };
                let query_endstops = self.state.with_mut(|st| {
                    st.endstop_map.insert(
                        pin_name,
                        EndstopEntry {
                            endstop: mcu_endstop.clone(),
                            invert: pin_params.invert,
                            pullup: pin_params.pullup,
                        },
                    );
                    st.endstops.push((mcu_endstop.clone(), name.to_string()));
                    st.query_endstops.clone()
                });
                query_endstops
                    .expect("query_endstops is loaded by GenericPrinterRail::new")
                    .register_endstop(mcu_endstop.clone(), name);
                Ok(mcu_endstop)
            }
            Some((mcu_endstop, invert, pullup)) => {
                let changed_invert = pin_params.invert != invert;
                let changed_pullup = pin_params.pullup != pullup;
                if changed_invert || changed_pullup {
                    return Err(KlippyError::Config(pct_format(
                        "Printer rail %s shared endstop pin %s must specify the same \
                         pullup/invert settings",
                        &[PctValue::Str(self.get_name(None)), PctValue::Str(pin_name)],
                    )?));
                }
                Ok(mcu_endstop)
            }
        }
    }

    fn first_stepper(&self) -> Stepper {
        match self.state.borrow().steppers.first() {
            Some(stepper) => stepper.clone(),
            None => panic!("AttributeError: 'GenericPrinterRail' object has no attribute 'get_commanded_position'"),
        }
    }
}

impl RailApi for GenericPrinterRail {
    fn get_name(&self, short: Option<bool>) -> String {
        let name = self.state.borrow().name.clone();
        if short.unwrap_or(false) {
            if name.starts_with("stepper") {
                // Skip an extra symbol after 'stepper'
                return name.chars().skip(8).collect();
            }
            return match name.split_whitespace().last() {
                Some(last) => last.to_string(),
                None => panic!("IndexError: list index out of range"),
            };
        }
        name
    }

    fn get_range(&self) -> (f64, f64) {
        let st = self.state.borrow();
        (st.position_min, st.position_max)
    }

    fn get_homing_info(&self) -> HomingInfo {
        let st = self.state.borrow();
        HomingInfo {
            speed: st.homing_speed,
            position_endstop: st.position_endstop,
            retract_speed: st.homing_retract_speed,
            retract_dist: st.homing_retract_dist,
            positive_dir: st.homing_positive_dir,
            second_homing_speed: st.second_homing_speed,
        }
    }

    fn get_steppers(&self) -> Vec<Stepper> {
        self.state.borrow().steppers.clone()
    }

    fn get_endstops(&self) -> Vec<(Arc<dyn EndstopApi>, String)> {
        self.state.borrow().endstops.clone()
    }

    fn lookup_endstop(&self, endstop_pin: Option<&str>, name: &str) -> KResult<Arc<dyn EndstopApi>> {
        self.lookup_endstop_checked(endstop_pin, name)
    }

    fn add_stepper(
        &self,
        stepper: Stepper,
        endstop_pin: Option<&str>,
        endstop_name: Option<&str>,
    ) -> KResult<()> {
        self.state.with_mut(|st| st.steppers.push(stepper.clone()));
        let mcu_endstop = match endstop_pin {
            Some(pin) => {
                let name = match endstop_name {
                    Some(name) if !name.is_empty() => name.to_string(),
                    _ => stepper.get_name(Some(true)),
                };
                self.lookup_endstop_checked(Some(pin), &name)?
            }
            None => {
                let (pin, name) = {
                    let st = self.state.borrow();
                    (st.endstop_pin.clone(), st.name.clone())
                };
                self.lookup_endstop_checked(Some(&pin), &name)?
            }
        };
        mcu_endstop.add_stepper(stepper);
        Ok(())
    }

    fn add_stepper_from_config(&self, config: &Config) -> KResult<()> {
        let units_in_radians = self.state.borrow().stepper_units_in_radians;
        let stepper = printer_stepper(config, units_in_radians)?;
        let endstop_pin =
            config.get("endstop_pin", GetStrOpts { default: Some(None), ..Default::default() })?;
        self.add_stepper(stepper, endstop_pin.as_deref(), None)
    }

    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        for stepper in self.get_steppers() {
            stepper.setup_itersolve(alloc_func, params);
        }
    }

    fn set_trapq(&self, tq: Option<Trapq>) {
        for stepper in self.get_steppers() {
            stepper.set_trapq(tq.clone());
        }
    }

    fn set_position(&self, coord: [f64; 3]) {
        for stepper in self.get_steppers() {
            stepper.set_position(coord);
        }
    }

    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64 {
        self.first_stepper().calc_position_from_coord(coord)
    }

    fn get_commanded_position(&self) -> f64 {
        self.first_stepper().get_commanded_position()
    }
}

/// `LookupRail` (`klippy/stepper.py:458-463`).
pub fn lookup_rail(config: &Config, opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
    let rail = GenericPrinterRail::new(config, opts)?;
    rail.add_stepper_from_config(config)?;
    Ok(rail)
}

/// `LookupMultiRail` (`klippy/stepper.py:466-475`).
pub fn lookup_multi_rail(config: &Config, opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
    let rail = lookup_rail(config, opts)?;
    for i in 1..99 {
        let section = format!("{}{}", config.get_name(), i);
        if !config.has_section(&section) {
            break;
        }
        rail.add_stepper_from_config(&config.getsection(&section))?;
    }
    Ok(rail)
}

// ---------------------------------------------------------------------------
// Factory.
// ---------------------------------------------------------------------------

/// `StepperFactoryApi` implementation: the module functions of `import stepper`.
pub struct StepperFactory;

impl StepperFactoryApi for StepperFactory {
    fn printer_stepper(&self, config: &Config, units_in_radians: bool) -> KResult<Stepper> {
        printer_stepper(config, units_in_radians)
    }

    fn lookup_rail(&self, config: &Config, opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        lookup_rail(config, opts)
    }

    fn lookup_multi_rail(&self, config: &Config, opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        lookup_multi_rail(config, opts)
    }

    fn generic_rail(&self, config: &Config, opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        Ok(GenericPrinterRail::new(config, opts)?)
    }

    fn parse_step_distance(
        &self,
        config: &Config,
        units_in_radians: Option<bool>,
        note_valid: bool,
    ) -> KResult<(f64, f64)> {
        parse_step_distance(config, units_in_radians, note_valid)
    }
}

/// Module factory. `stepper.py` has no load/entry points.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
