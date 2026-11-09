//! Contract of the `mcu` service.
//!
//! Errors: `mcu.error` is `KlippyError::Mcu`; `pins.error` (`klippy/pins.py:8`) is
//! `KlippyError::Pins`.

use std::sync::Arc;

use crate::error::KResult;
use crate::object::lookup_interface;
use crate::printer::{LookupDefault, Printer, PrinterApi};
use crate::stepper::Stepper;
use indexmap::IndexMap;
use pycompat::PyValue;
use reactor::Completion;

/// Zero-argument callback run once at `klippy:connect` finalize (`klippy/mcu.py:1318`).
/// An error aborts the finalize like the Python exception does.
pub type ConfigCallback = Box<dyn FnMut() -> KResult<()> + Send>;

/// Serial-response callback receiving the firmware response dict (`klippy/mcu.py:1346`).
pub type SerialResponseCallback = Arc<dyn Fn(PyValue) + Send + Sync>;

/// ADC sample callback receiving `(print_time, value)` samples (`klippy/mcu.py:580`).
pub type AdcCallback = Box<dyn FnMut(Vec<(f64, f64)>) + Send>;

/// Opaque command-queue handle from `alloc_command_queue()` (`klippy/mcu.py:1344`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CommandQueue(pub i64);

/// Options for `add_config_cmd` (`klippy/mcu.py:1320`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AddConfigCmdOpts {
    /// `is_init` selects the init-command list.
    pub is_init: bool,
    /// `on_restart` selects the restart-command list.
    pub on_restart: bool,
}

/// Options for `lookup_query_command` (`klippy/mcu.py:1335`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LookupQueryCommandOpts {
    /// `oid` response filter.
    pub oid: Option<i64>,
    /// `cq` command queue, or default when `None`.
    pub cq: Option<CommandQueue>,
    /// `is_async` selects `RetryAsyncCommand`.
    pub is_async: bool,
}

/// Options for `CommandWrapper.send`/`send_wait_ack` (`klippy/mcu.py:122`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SendOpts {
    /// `minclock` MCU clock, default 0.
    pub minclock: i64,
    /// `reqclock` MCU clock, default 0.
    pub reqclock: i64,
}

/// Options for `CommandQueryWrapper.send`/`send_with_preface` (`klippy/mcu.py:101`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuerySendOpts {
    /// `minclock` MCU clock, default 0.
    pub minclock: i64,
    /// `reqclock` MCU clock, default 0.
    pub reqclock: i64,
    /// `retry` controls the retry loop, default true.
    pub retry: bool,
}

impl Default for QuerySendOpts {
    fn default() -> QuerySendOpts {
        QuerySendOpts { minclock: 0, reqclock: 0, retry: true }
    }
}

/// Options for `parse_pin` (`klippy/pins.py:67`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ParsePinOpts {
    /// `can_invert` allows the `!` prefix.
    pub can_invert: bool,
    /// `can_pullup` allows the `^`/`~` prefix.
    pub can_pullup: bool,
}

/// Options for `lookup_pin` (`klippy/pins.py:96`).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LookupPinOpts {
    /// `can_invert` allows the `!` prefix.
    pub can_invert: bool,
    /// `can_pullup` allows the `^`/`~` prefix.
    pub can_pullup: bool,
    /// `share_type` pin-sharing type, default `None`.
    pub share_type: Option<String>,
}

/// Options for `setup_adc_sample` (`klippy/mcu.py:570`).
#[derive(Clone, Debug, PartialEq)]
pub struct AdcSampleOpts {
    /// `sample_time` in seconds, default 0.
    pub sample_time: f64,
    /// `sample_count`, default 1.
    pub sample_count: i64,
    /// `batch_num`, default 1.
    pub batch_num: i64,
    /// `minval`, default 0.
    pub minval: f64,
    /// `maxval`, default 1.
    pub maxval: f64,
    /// `range_check_count`, default 0.
    pub range_check_count: i64,
}

impl Default for AdcSampleOpts {
    fn default() -> AdcSampleOpts {
        AdcSampleOpts {
            sample_time: 0.,
            sample_count: 1,
            batch_num: 1,
            minval: 0.,
            maxval: 1.,
            range_check_count: 0,
        }
    }
}

/// Pin parameters from `PrinterPins.lookup_pin`/`parse_pin` (`klippy/pins.py:67`).
///
/// Virtual chips (`probe` for `probe:z_virtual_endstop`, `sx1509`, `replicape`, TMC virtual
/// pins) register a [`ChipApi`] with `PrinterPins.register_chip`; their pins carry
/// `chip.get_mcu()` in [`PinParams::chip`], and `PrinterPins.setup_pin` dispatches
/// `setup_pin` to the registered chip found by [`PinParams::chip_name`], never through
/// `PinParams::chip`.
#[derive(Clone)]
pub struct PinParams {
    /// The MCU behind the pin: the `MCU` itself for an MCU pin, `chip.get_mcu()` for a
    /// virtual chip's pin.
    pub chip: Mcu,
    /// Chip name.
    pub chip_name: String,
    /// Pin name.
    pub pin: String,
    /// Invert flag (0/1).
    pub invert: i64,
    /// Pullup flag (0/1/-1).
    pub pullup: i64,
    /// Pin-sharing type (`None` until `lookup_pin` records it).
    pub share_type: Option<String>,
}

/// A pin object returned by `setup_pin` (`klippy/mcu.py:1314`).
#[derive(Clone)]
pub enum Pin {
    /// `MCU_endstop` (`klippy/mcu.py:340`).
    Endstop(Endstop),
    /// `MCU_digital_out` (`klippy/mcu.py:408`).
    DigitalOut(DigitalOutPin),
    /// `MCU_pwm` (`klippy/mcu.py:451`).
    Pwm(PwmPin),
    /// `MCU_adc` (`klippy/mcu.py:555`).
    Adc(AdcPin),
}

/// `MCU` (`klippy/mcu.py:1288`).
pub trait McuApi: Send + Sync + 'static {
    /// `get_name` (`klippy/mcu.py:1307`).
    fn get_name(&self) -> String;
    /// `get_printer` (`klippy/mcu.py:1309`).
    fn get_printer(&self) -> Printer;
    /// `is_fileoutput` (`klippy/mcu.py:1311`).
    fn is_fileoutput(&self) -> bool;
    /// `setup_pin` (`klippy/mcu.py:1314`).
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin>;
    /// `create_oid` (`klippy/mcu.py:1316`).
    fn create_oid(&self) -> KResult<i64>;
    /// `register_config_callback` (`klippy/mcu.py:1318`).
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()>;
    /// `add_config_cmd` (`klippy/mcu.py:1320`).
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()>;
    /// `request_move_queue_slot` (`klippy/mcu.py:1322`).
    fn request_move_queue_slot(&self);
    /// `get_query_slot` (`klippy/mcu.py:1324`).
    fn get_query_slot(&self, oid: i64) -> i64;
    /// `seconds_to_clock` (`klippy/mcu.py:1326`).
    fn seconds_to_clock(&self, time: f64) -> i64;
    /// `min_schedule_time` (`klippy/mcu.py:1329`).
    fn min_schedule_time(&self) -> f64;
    /// `max_nominal_duration` (`klippy/mcu.py:1331`).
    fn max_nominal_duration(&self) -> f64;
    /// `lookup_command` (`klippy/mcu.py:1333`).
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper>;
    /// `lookup_query_command` (`klippy/mcu.py:1335`).
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper>;
    /// `try_lookup_command` (`klippy/mcu.py:1339`).
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper>;
    /// `alloc_command_queue` (`klippy/mcu.py:1344`).
    fn alloc_command_queue(&self) -> CommandQueue;
    /// `register_serial_response` (`klippy/mcu.py:1346`).
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper;
    /// `check_valid_response` (`klippy/mcu.py:1349`).
    fn check_valid_response(&self, msgformat: &str) -> bool;
    /// `get_enumerations` (`klippy/mcu.py:1356`).
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>>;
    /// `get_constants` (`klippy/mcu.py:1358`).
    fn get_constants(&self) -> IndexMap<String, String>;
    /// `get_constant_float` (`klippy/mcu.py:1360`).
    fn get_constant_float(&self, name: &str) -> KResult<f64>;
    /// `print_time_to_clock` (`klippy/mcu.py:1363`).
    fn print_time_to_clock(&self, print_time: f64) -> i64;
    /// `clock_to_print_time` (`klippy/mcu.py:1365`) — Python passes fractional clocks
    /// (`bulk_sensor`); integer callers convert with `as f64`, which matches CPython's
    /// int-to-float conversion.
    fn clock_to_print_time(&self, clock: f64) -> f64;
    /// `estimated_print_time` (`klippy/mcu.py:1367`).
    fn estimated_print_time(&self, eventtime: f64) -> f64;
    /// `clock32_to_clock64` (`klippy/mcu.py:1369`).
    fn clock32_to_clock64(&self, clock32: i64) -> i64;
    /// `calibrate_clock` (`klippy/mcu.py:1371`).
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)>;
    /// `get_status` (`klippy/mcu.py:1376`).
    fn get_status(&self, eventtime: Option<f64>) -> PyValue;
    /// `stats` (`klippy/mcu.py:1378`).
    fn stats(&self, eventtime: f64) -> (bool, String);
    /// `TriggerDispatch(mcu)` (`klippy/mcu.py:282`), built by probes and
    /// `trigger_analog`.
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch>;
    /// `mcu._serial.get_serialqueue()` for `motion_queuing.setup_mcu_movequeue`
    /// (`klippy/mcu.py:1226-1227`). `None` before the serial session starts,
    /// exactly like the Python attribute.
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>;
}

/// `mcu.get_printer_mcu(printer, name)` (`klippy/mcu.py:1390`): `'mcu'` is the main
/// MCU, any other name is the object `'mcu <name>'`.
pub fn get_printer_mcu(printer: &Printer, name: &str) -> KResult<Mcu> {
    let objname = if name == "mcu" { name.to_string() } else { format!("mcu {}", name) };
    let mcu = lookup_interface::<dyn McuApi>(&**printer as &dyn PrinterApi, &objname, LookupDefault::Raise)?;
    Ok(mcu.expect("LookupDefault::Raise never yields None"))
}

/// The concrete type behind a [`CommandWrapper`] handle. Python's
/// `CommandQueryWrapper.send_with_preface` encodes through
/// `preface_cmd._cmd` (`klippy/mcu.py:106`), so the MCU port downcasts the
/// preface it created. Every `'static` type gets it, fakes included.
pub trait CommandWrapperAny: std::any::Any {
    fn command_wrapper_any(&self) -> &dyn std::any::Any;
}

impl<T: std::any::Any> CommandWrapperAny for T {
    fn command_wrapper_any(&self) -> &dyn std::any::Any {
        self
    }
}

/// `CommandWrapper` (`klippy/mcu.py:110`).
pub trait CommandWrapperApi: CommandWrapperAny + Send + Sync + 'static {
    /// `send` (`klippy/mcu.py:122`).
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()>;
    /// `send_wait_ack` (`klippy/mcu.py:125`).
    /// Pauses the reactor.
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()>;
    /// `get_command_tag` (`klippy/mcu.py:128`).
    fn get_command_tag(&self) -> i64;
}

/// `CommandQueryWrapper` (`klippy/mcu.py:76`).
pub trait CommandQueryWrapperApi: Send + Sync + 'static {
    /// `send` (`klippy/mcu.py:101`).
    /// Pauses the reactor.
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue>;
    /// `send_with_preface` (`klippy/mcu.py:104`).
    /// Pauses the reactor.
    fn send_with_preface(
        &self,
        preface_cmd: CommandWrapper,
        preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue>;
}

/// `AsyncResponseWrapper` (`klippy/mcu.py:132`).
pub trait AsyncResponseWrapperApi: Send + Sync + 'static {
    /// `unregister` (`klippy/mcu.py:147`).
    fn unregister(&self);
}

/// `MCU_trsync` (`klippy/mcu.py:155`).
pub trait TrsyncApi: Send + Sync + 'static {
    /// `get_mcu` (`klippy/mcu.py:177`).
    fn get_mcu(&self) -> Mcu;
    /// `get_oid` (`klippy/mcu.py:179`).
    fn get_oid(&self) -> i64;
    /// `get_command_queue` (`klippy/mcu.py:181`).
    fn get_command_queue(&self) -> CommandQueue;
    /// `add_stepper` (`klippy/mcu.py:183`).
    fn add_stepper(&self, stepper: Stepper);
    /// `get_steppers` (`klippy/mcu.py:187`).
    fn get_steppers(&self) -> Vec<Stepper>;
    /// `start` (`klippy/mcu.py:242`).
    fn start(
        &self,
        print_time: f64,
        report_offset: f64,
        trigger_completion: Completion<bool>,
        expire_timeout: f64,
    );
    /// `set_home_end_time` (`klippy/mcu.py:265`).
    fn set_home_end_time(&self, home_end_time: f64);
    /// `stop` (`klippy/mcu.py:267`).
    /// Pauses the reactor.
    fn stop(&self) -> KResult<i64>;
}

/// `TriggerDispatch` (`klippy/mcu.py:282`).
pub trait TriggerDispatchApi: Send + Sync + 'static {
    /// `get_oid` (`klippy/mcu.py:289`).
    fn get_oid(&self) -> i64;
    /// `get_command_queue` (`klippy/mcu.py:291`).
    fn get_command_queue(&self) -> CommandQueue;
    /// `add_stepper` (`klippy/mcu.py:293`).
    fn add_stepper(&self, stepper: Stepper) -> KResult<()>;
    /// `get_steppers` (`klippy/mcu.py:309`).
    fn get_steppers(&self) -> Vec<Stepper>;
    /// `start` (`klippy/mcu.py:311`).
    fn start(&self, print_time: f64) -> Completion<bool>;
    /// `wait_end` (`klippy/mcu.py:325`).
    /// Pauses the reactor.
    fn wait_end(&self, end_time: f64);
    /// `stop` (`klippy/mcu.py:331`).
    /// Pauses the reactor.
    fn stop(&self) -> KResult<i64>;
}

/// `MCU_endstop` (`klippy/mcu.py:340`).
pub trait EndstopApi: Send + Sync + 'static {
    /// `get_mcu` (`klippy/mcu.py:351`); probe endstop wrappers raise here.
    fn get_mcu(&self) -> KResult<Mcu>;
    /// `add_stepper` (`klippy/mcu.py:353`).
    fn add_stepper(&self, stepper: Stepper);
    /// `get_steppers` (`klippy/mcu.py:355`).
    fn get_steppers(&self) -> Vec<Stepper>;
    /// `home_start` (`klippy/mcu.py:375`).
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: Option<bool>,
    ) -> KResult<Completion<bool>>;
    /// `home_wait` (`klippy/mcu.py:387`).
    /// Pauses the reactor.
    fn home_wait(&self, home_end_time: f64) -> KResult<f64>;
    /// `query_endstop` (`klippy/mcu.py:401`).
    /// Pauses the reactor.
    fn query_endstop(&self, print_time: f64) -> KResult<i64>;
    /// `get_position_endstop()` of probe endstop wrappers (`klippy/extras/probe.py`);
    /// `None` for a plain `MCU_endstop` (the `hasattr` test of `stepper.py:415`).
    fn get_position_endstop(&self) -> Option<f64> {
        None
    }
}

/// `MCU_digital_out` (`klippy/mcu.py:408`).
pub trait DigitalOutPinApi: Send + Sync + 'static {
    /// `get_mcu` (`klippy/mcu.py:419`).
    fn get_mcu(&self) -> Mcu;
    /// `setup_max_duration` (`klippy/mcu.py:421`).
    fn setup_max_duration(&self, max_duration: f64);
    /// `setup_start_value` (`klippy/mcu.py:423`).
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool);
    /// `set_digital` (`klippy/mcu.py:445`).
    fn set_digital(&self, print_time: f64, value: bool);
}

/// `MCU_pwm` (`klippy/mcu.py:451`).
pub trait PwmPinApi: Send + Sync + 'static {
    /// `get_mcu` (`klippy/mcu.py:466`).
    fn get_mcu(&self) -> Mcu;
    /// `setup_max_duration` (`klippy/mcu.py:468`).
    fn setup_max_duration(&self, max_duration: f64);
    /// `setup_cycle_time` (`klippy/mcu.py:470`); external chips raise `pins.error` for
    /// unsupported parameters (`klippy/extras/replicape.py:40-42`).
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) -> KResult<()>;
    /// `setup_start_value` (`klippy/mcu.py:473`).
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64);
    /// `next_aligned_print_time` (`klippy/mcu.py:531`).
    fn next_aligned_print_time(&self, print_time: f64, allow_early: Option<f64>) -> f64;
    /// `set_pwm` (`klippy/mcu.py:545`).
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()>;
}

/// `MCU_adc` (`klippy/mcu.py:555`).
pub trait AdcPinApi: Send + Sync + 'static {
    /// `get_mcu` (`klippy/mcu.py:568`).
    fn get_mcu(&self) -> Mcu;
    /// `setup_adc_sample` (`klippy/mcu.py:570`).
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts);
    /// `setup_adc_callback` (`klippy/mcu.py:580`).
    fn setup_adc_callback(&self, callback: AdcCallback);
    /// `get_last_value` (`klippy/mcu.py:582`).
    fn get_last_value(&self) -> (f64, f64);
}

/// `PinResolver` (`klippy/pins.py:18`).
pub trait PinResolverApi: Send + Sync + 'static {
    /// `reserve_pin` (`klippy/pins.py:24`).
    fn reserve_pin(&self, pin: &str, reserve_name: &str) -> KResult<()>;
    /// `alias_pin` (`klippy/pins.py:29`).
    fn alias_pin(&self, alias: &str, pin: &str) -> KResult<()>;
    /// `update_command` (`klippy/pins.py:41`).
    fn update_command(&self, cmd: &str) -> KResult<String>;
}

/// `PrinterPins` (`klippy/pins.py:60`).
pub trait PinsApi: Send + Sync + 'static {
    /// `parse_pin` (`klippy/pins.py:67`).
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams>;
    /// `lookup_pin` (`klippy/pins.py:96`).
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams>;
    /// `setup_pin` (`klippy/pins.py:114`).
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin>;
    /// `reset_pin_sharing` (`klippy/pins.py:119`): an unknown share is a `KeyError`.
    fn reset_pin_sharing(&self, pin_params: PinParams) -> KResult<()>;
    /// `get_pin_resolver` (`klippy/pins.py:122`).
    fn get_pin_resolver(&self, chip_name: &str) -> KResult<PinResolver>;
    /// `register_chip` (`klippy/pins.py:126`).
    fn register_chip(&self, chip_name: &str, chip: Chip) -> KResult<()>;
    /// `allow_multi_use_pin` (`klippy/pins.py:132`).
    fn allow_multi_use_pin(&self, pin_desc: &str) -> KResult<()>;
}

/// Chip protocol object (`klippy/pins.py:118`).
pub trait ChipApi: Send + Sync + 'static {
    /// `setup_pin` (`klippy/pins.py:118`).
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin>;
    /// `get_mcu`: the MCU [`PinsApi::parse_pin`] stores in [`PinParams::chip`].
    fn get_mcu(&self) -> KResult<Mcu>;
}

/// `MCU` handle.
pub type Mcu = Arc<dyn McuApi>;
/// `CommandWrapper` handle.
pub type CommandWrapper = Arc<dyn CommandWrapperApi>;
/// `CommandQueryWrapper` handle.
pub type CommandQueryWrapper = Arc<dyn CommandQueryWrapperApi>;
/// `AsyncResponseWrapper` handle.
pub type AsyncResponseWrapper = Arc<dyn AsyncResponseWrapperApi>;
/// `MCU_trsync` handle.
pub type Trsync = Arc<dyn TrsyncApi>;
/// `TriggerDispatch` handle.
pub type TriggerDispatch = Arc<dyn TriggerDispatchApi>;
/// `MCU_endstop` handle.
pub type Endstop = Arc<dyn EndstopApi>;
/// `MCU_digital_out` handle.
pub type DigitalOutPin = Arc<dyn DigitalOutPinApi>;
/// `MCU_pwm` handle.
pub type PwmPin = Arc<dyn PwmPinApi>;
/// `MCU_adc` handle.
pub type AdcPin = Arc<dyn AdcPinApi>;
/// `PrinterPins` handle.
pub type Pins = Arc<dyn PinsApi>;
/// Chip handle.
pub type Chip = Arc<dyn ChipApi>;
/// `PinResolver` handle.
pub type PinResolver = Arc<dyn PinResolverApi>;
