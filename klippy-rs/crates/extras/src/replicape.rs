//! Port of `klippy/extras/replicape.py`.
//!
//! Chips: the PCA9685 PWM driver (`pca9685_pwm`) with its virtual enable
//! pin, the per-stepper DAC enable (`ReplicapeDACEnable`), the BeagleBone
//! servo PWM helper (`servo_pwm`), and the `Replicape` chip object that
//! owns the stepper shift registers (SPI) and registers itself as the
//! `replicape` pin chip.
//!
//! Floating point matches the Python source line by line (`pycompat`
//! conversions, no fused multiply-add, Python `min`/`max` argument order).
//! `pins.error` is `KlippyError::Pins`. `PwmPinApi` carries the Python
//! exceptions of `setup_cycle_time(hardware_pwm=True)` and of `set_pwm`
//! before the config callback ran; `DigitalOutPinApi::set_digital` has no
//! error channel, so those raise through the panic path.

use std::sync::{Arc, OnceLock, Weak};

use indexmap::IndexMap;

use klippy_api::config::{Config, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::{
    get_printer_mcu, AddConfigCmdOpts, Chip, ChipApi, CommandWrapper, DigitalOutPin,
    DigitalOutPinApi, Mcu, Pin, PinParams, PinsApi, PwmPin, PwmPinApi, SendOpts,
};
use klippy_api::object::{lookup_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::Reactor;

use crate::bus::McuSpi;

/// `REPLICAPE_MAX_CURRENT = 3.84`.
pub const REPLICAPE_MAX_CURRENT: f64 = 3.84;
/// `REPLICAPE_PCA9685_BUS = 2`.
pub const REPLICAPE_PCA9685_BUS: i64 = 2;
/// `REPLICAPE_PCA9685_ADDRESS = 0x70`.
pub const REPLICAPE_PCA9685_ADDRESS: i64 = 0x70;
/// `REPLICAPE_PCA9685_CYCLE_TIME = .001`.
pub const REPLICAPE_PCA9685_CYCLE_TIME: f64 = 0.001;
/// `PIN_MIN_TIME = 0.100`.
pub const PIN_MIN_TIME: f64 = 0.100;

/// `SERVO_PINS`: name -> (pwm pin, reserved pin 1, reserved pin 2).
pub const SERVO_PINS: &[(&str, (&str, &str, &str))] = &[
    ("servo0", ("/pwm0", "gpio0_30", "gpio1_18")),
    ("servo1", ("/pwm1", "gpio3_17", "gpio1_19")),
];

/// `pins.error` (`klippy/pins.py:8`).
fn pins_error(message: &str) -> KlippyError {
    KlippyError::Pins(message.to_string())
}

/// `AttributeError: '<class>' object has no attribute '<attr>'`.
fn attribute_error(class: &str, attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'{class}' object has no attribute '{attr}'"),
    }
}

fn fmt(format: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(format, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn pct_int(x: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(x))
}

/// Python `max(a, b)`: `b` only when `b > a`.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python `min(a, b)`: `b` only when `b < a`.
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `ReplicapeStepConfig`: microstep choice -> shift register bits
/// (`None` for `disable`), in the Python dict order.
pub fn step_config() -> IndexMap<String, Option<i64>> {
    let mut m: IndexMap<String, Option<i64>> = IndexMap::new();
    m.insert("disable".to_string(), None);
    m.insert("1".to_string(), Some((1 << 7) | (1 << 5)));
    m.insert("2".to_string(), Some((1 << 7) | (1 << 5) | (1 << 6)));
    m.insert("spread2".to_string(), Some(1 << 5));
    m.insert("4".to_string(), Some((1 << 7) | (1 << 5) | (1 << 4)));
    m.insert("16".to_string(), Some((1 << 7) | (1 << 5) | (1 << 6) | (1 << 4)));
    m.insert("spread4".to_string(), Some((1 << 5) | (1 << 4)));
    m.insert("spread16".to_string(), Some(1 << 7));
    m.insert("stealth4".to_string(), Some((1 << 7) | (1 << 6)));
    m.insert("stealth16".to_string(), Some(0));
    m
}

/// `ReplicapeStepConfig` as the `choices` dict handed to `getchoice`.
fn step_config_choices() -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in step_config() {
        let val = match v {
            Some(n) => PyValue::Int(PyInt::from_i64(n)),
            None => PyValue::None,
        };
        d.insert(PyValue::Str(k), val);
    }
    PyValue::Dict(d)
}

/// The classes of the `self.pins` table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PinKind {
    Pca9685Pwm,
    ReplicapeDacEnable,
}

impl PinKind {
    fn class_name(self) -> &'static str {
        match self {
            PinKind::Pca9685Pwm => "pca9685_pwm",
            PinKind::ReplicapeDacEnable => "ReplicapeDACEnable",
        }
    }
}

#[derive(Default)]
struct ReplicapeState {
    mcu_pwm_start_value: bool,
    mcu_pwm_shutdown_value: bool,
    last_pwm_enable_time: f64,
    /// `self.pins`: pin name -> (class, channel).
    pins: IndexMap<String, (PinKind, i64)>,
    /// `self.servo_pins`: pin name -> shift register index.
    servo_pins: IndexMap<String, usize>,
    enabled_channels: IndexMap<i64, bool>,
    stepper_dacs: IndexMap<i64, f64>,
    sr_disabled: Vec<u8>,
    sr_enabled: Vec<u8>,
}

fn set_once<T>(cell: &OnceLock<T>, value: T) {
    if cell.set(value).is_err() {
        unreachable!("replicape member assigned twice");
    }
}

/// The `replicape` chip object (`class Replicape`).
pub struct Replicape {
    me: Weak<Replicape>,
    host_mcu: OnceLock<Mcu>,
    mcu_pwm_enable: OnceLock<DigitalOutPin>,
    sr_spi: OnceLock<Arc<McuSpi>>,
    state: GCell<ReplicapeState>,
}

impl Replicape {
    /// `Replicape(config)` (`replicape.py:161`).
    pub fn new(config: &Config) -> KResult<Arc<Replicape>> {
        let printer = config.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let obj = Arc::new_cyclic(|me| Replicape {
            me: me.clone(),
            host_mcu: OnceLock::new(),
            mcu_pwm_enable: OnceLock::new(),
            sr_spi: OnceLock::new(),
            state: GCell::new(ReplicapeState::default()),
        });
        ppins.register_chip("replicape", Arc::clone(&obj) as Chip)?;
        let revisions = PyValue::List(vec![PyValue::Str("B3".to_string())]);
        config.getchoice("revision", &revisions, GetChoiceOpts::default())?;
        let host_mcu_name = config
            .get("host_mcu", GetStrOpts::default())?
            .expect("required option yields a value");
        let host_mcu = get_printer_mcu(&printer, &host_mcu_name)?;
        set_once(&obj.host_mcu, host_mcu.clone());

        // Setup enable pin.
        let enable_pin = config
            .get(
                "enable_pin",
                GetStrOpts { default: Some(Some("!gpio0_20".to_string())), note_valid: true },
            )?
            .expect("defaulted option yields a value");
        let mcu_pwm_enable = match ppins.setup_pin("digital_out", &enable_pin)? {
            Pin::DigitalOut(pin) => pin,
            Pin::Endstop(_) => return Err(attribute_error("MCU_endstop", "setup_max_duration")),
            Pin::Adc(_) => return Err(attribute_error("MCU_adc", "setup_max_duration")),
            // `setup_max_duration` and `setup_start_value` exist on `MCU_pwm`,
            // the first use of the missing `set_digital` is the one that raises.
            Pin::Pwm(_) => return Err(attribute_error("MCU_pwm", "set_digital")),
        };
        mcu_pwm_enable.setup_max_duration(0.);
        set_once(&obj.mcu_pwm_enable, mcu_pwm_enable);

        // Setup power pins.
        obj.state.with_mut(|st| {
            for (name, channel) in [
                ("power_e", 5),
                ("power_h", 3),
                ("power_hotbed", 4),
                ("power_fan0", 7),
                ("power_fan1", 8),
                ("power_fan2", 9),
                ("power_fan3", 10),
            ] {
                st.pins.insert(name.to_string(), (PinKind::Pca9685Pwm, channel));
            }
            st.servo_pins.insert("servo0".to_string(), 3);
            st.servo_pins.insert("servo1".to_string(), 2);
        });

        // Setup stepper config.
        let mut stepper_dacs: IndexMap<i64, f64> = IndexMap::new();
        let mut shift_registers: Vec<u8> = vec![1, 0, 0, 1, 1];
        let choices = step_config_choices();
        for (port, name) in "xyzeh".chars().enumerate() {
            let prefix = format!("stepper_{name}_");
            let sc = config.getchoice(
                &format!("{prefix}microstep_mode"),
                &choices,
                GetChoiceOpts {
                    default: Some(Some(PyValue::Str("disable".to_string()))),
                    note_valid: true,
                },
            )?;
            let mut sc: i64 = match sc {
                PyValue::None => continue,
                PyValue::Int(n) => n
                    .0
                    .to_string()
                    .parse()
                    .expect("microstep choice values are small integers"),
                _ => unreachable!("getchoice returns an entry of ReplicapeStepConfig"),
            };
            sc |= i64::from(shift_registers[port]);
            let off_time_high = config
                .getboolean(
                    &format!("{prefix}chopper_off_time_high"),
                    GetBoolOpts { default: Some(Some(false)), note_valid: true, ..Default::default() },
                )?
                .expect("defaulted option yields a value");
            if off_time_high {
                sc |= 1 << 3;
            }
            let hysteresis_high = config
                .getboolean(
                    &format!("{prefix}chopper_hysteresis_high"),
                    GetBoolOpts { default: Some(Some(false)), note_valid: true, ..Default::default() },
                )?
                .expect("defaulted option yields a value");
            if hysteresis_high {
                sc |= 1 << 2;
            }
            let blank_time_high = config
                .getboolean(
                    &format!("{prefix}chopper_blank_time_high"),
                    GetBoolOpts { default: Some(Some(true)), note_valid: true, ..Default::default() },
                )?
                .expect("defaulted option yields a value");
            if blank_time_high {
                sc |= 1 << 1;
            }
            shift_registers[port] = sc as u8;
            let channel = port as i64 + 11;
            let cur = config
                .getfloat(
                    &format!("{prefix}current"),
                    GetFloatOpts {
                        maxval: Some(REPLICAPE_MAX_CURRENT),
                        above: Some(0.),
                        ..GetFloatOpts::default()
                    },
                )?
                .expect("required option yields a value");
            stepper_dacs.insert(channel, cur / REPLICAPE_MAX_CURRENT);
            obj.state.with_mut(|st| {
                st.pins
                    .insert(format!("{prefix}enable"), (PinKind::ReplicapeDacEnable, channel));
            });
        }
        let sr_disabled: Vec<u8> = shift_registers.iter().rev().copied().collect();
        if [0, 1, 2].iter().any(|i| stepper_dacs.contains_key(&(11 + i))) {
            // Enable xyz steppers.
            shift_registers[0] &= !1;
        }
        if [3, 4].iter().any(|i| stepper_dacs.contains_key(&(11 + i))) {
            // Enable eh steppers.
            shift_registers[3] &= !1;
        }
        let standstill_power_down = config
            .getboolean(
                "standstill_power_down",
                GetBoolOpts { default: Some(Some(false)), note_valid: true, ..Default::default() },
            )?
            .expect("defaulted option yields a value");
        if standstill_power_down && !stepper_dacs.is_empty() {
            shift_registers[4] &= !1;
        }
        let sr_enabled: Vec<u8> = shift_registers.iter().rev().copied().collect();
        obj.state.with_mut(|st| {
            st.enabled_channels = st.pins.values().map(|(_, ch)| (*ch, false)).collect();
            st.stepper_dacs = stepper_dacs;
            st.sr_disabled = sr_disabled.clone();
            st.sr_enabled = sr_enabled;
        });

        let mut sr_spi_bus = "spidev1.1";
        if !host_mcu.is_fileoutput()
            && std::path::Path::new("/sys/devices/platform/ocp/481a0000.spi/spi_master/spi2")
                .exists()
        {
            sr_spi_bus = "spidev2.1";
        }
        let sr_spi = McuSpi::new(
            host_mcu,
            Some(sr_spi_bus.to_string()),
            None,
            0,
            50_000_000,
            None,
            false,
        )?;
        set_once(&obj.sr_spi, Arc::clone(&sr_spi));
        sr_spi.setup_shutdown_msg(&sr_disabled)?;
        sr_spi.spi_send(&sr_disabled, SendOpts::default())?;
        Ok(obj)
    }

    fn this(&self) -> Arc<Replicape> {
        self.me.upgrade().expect("replicape is alive while its pins are in use")
    }

    fn host_mcu(&self) -> Mcu {
        self.host_mcu.get().expect("host_mcu is set during construction").clone()
    }

    fn mcu_pwm_enable(&self) -> DigitalOutPin {
        self.mcu_pwm_enable.get().expect("mcu_pwm_enable is set during construction").clone()
    }

    fn sr_spi(&self) -> Arc<McuSpi> {
        Arc::clone(self.sr_spi.get().expect("sr_spi is set during construction"))
    }

    /// `self.stepper_dacs`: channel -> current fraction.
    pub fn stepper_dacs(&self) -> IndexMap<i64, f64> {
        self.state.borrow().stepper_dacs.clone()
    }

    /// `self.enabled_channels`.
    pub fn enabled_channels(&self) -> IndexMap<i64, bool> {
        self.state.borrow().enabled_channels.clone()
    }

    /// `self.sr_enabled`.
    pub fn sr_enabled(&self) -> Vec<u8> {
        self.state.borrow().sr_enabled.clone()
    }

    /// `self.sr_disabled`.
    pub fn sr_disabled(&self) -> Vec<u8> {
        self.state.borrow().sr_disabled.clone()
    }

    /// `self.pins`: pin name -> (class name, channel).
    pub fn pin_map(&self) -> IndexMap<String, (String, i64)> {
        self.state
            .borrow()
            .pins
            .iter()
            .map(|(name, (kind, channel))| (name.clone(), (kind.class_name().to_string(), *channel)))
            .collect()
    }

    /// `note_pwm_start_value(channel, start_value, shutdown_value)`
    /// (`replicape.py:223`).
    pub fn note_pwm_start_value(&self, channel: i64, start_value: f64, shutdown_value: f64) {
        let (start, shutdown) = self.state.with_mut(|st| {
            st.mcu_pwm_start_value |= start_value != 0.;
            st.mcu_pwm_shutdown_value |= shutdown_value != 0.;
            (st.mcu_pwm_start_value, st.mcu_pwm_shutdown_value)
        });
        self.mcu_pwm_enable().setup_start_value(start, shutdown);
        self.state.with_mut(|st| {
            st.enabled_channels.insert(channel, start_value != 0.);
        });
    }

    /// `note_pwm_enable(print_time, channel, is_enable)` (`replicape.py:229`).
    pub fn note_pwm_enable(&self, print_time: f64, channel: i64, is_enable: bool) {
        // Check if need to set the pca9685 enable pin.
        let (pe_time, enable_value, sr) = self.state.with_mut(|st| {
            st.enabled_channels.insert(channel, is_enable);
            let pe_time = py_max(print_time, st.last_pwm_enable_time + PIN_MIN_TIME);
            let on_channels = st.enabled_channels.values().filter(|&&e| e).count();
            let enable_value = if on_channels == 0 {
                Some(false)
            } else if is_enable && on_channels == 1 {
                Some(true)
            } else {
                None
            };
            if enable_value.is_some() {
                st.last_pwm_enable_time = pe_time;
            }
            // Check if need to set the stepper enable lines.
            let sr = if st.stepper_dacs.contains_key(&channel) {
                let on_dacs = st
                    .stepper_dacs
                    .keys()
                    .filter(|c| st.enabled_channels.get(*c).copied().unwrap_or(false))
                    .count();
                if on_dacs == 0 {
                    Some(st.sr_disabled.clone())
                } else if is_enable && on_dacs == 1 {
                    Some(st.sr_enabled.clone())
                } else {
                    None
                }
            } else {
                None
            };
            (pe_time, enable_value, sr)
        });
        if let Some(value) = enable_value {
            self.mcu_pwm_enable().set_digital(pe_time, value);
        }
        let Some(sr) = sr else { return };
        let host_mcu = self.host_mcu();
        let clock = host_mcu.print_time_to_clock(print_time);
        if let Err(err) = self.sr_spi().spi_send(&sr, SendOpts { minclock: clock, reqclock: clock })
        {
            // Python lets the exception end the reactor callback, which
            // takes klippy down; the shutdown is the nearest equivalent.
            host_mcu
                .get_printer()
                .invoke_shutdown(err.message(), PyValue::Dict(IndexMap::new()));
        }
    }
}

klippy_api::printer_object!(Replicape; dyn ChipApi);

impl ChipApi for Replicape {
    /// `setup_pin(pin_type, pin_params)` (`replicape.py:253`).
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        let pin = pin_params.pin.clone();
        let (found, servo_index) = {
            let st = self.state.borrow();
            (st.pins.get(&pin).copied(), st.servo_pins.get(&pin).copied())
        };
        if let Some((kind, channel)) = found {
            let this = self.this();
            return match kind {
                PinKind::Pca9685Pwm => {
                    let pwm = Pca9685Pwm::new(this, channel, pin_type, &pin_params)?;
                    Ok(if pin_type == "digital_out" {
                        Pin::DigitalOut(pwm as DigitalOutPin)
                    } else {
                        Pin::Pwm(pwm as PwmPin)
                    })
                }
                PinKind::ReplicapeDacEnable => {
                    let dac = ReplicapeDacEnable::new(this, channel, pin_type, &pin_params)?;
                    Ok(Pin::DigitalOut(dac as DigitalOutPin))
                }
            };
        }
        if let Some(index) = servo_index {
            // Enable servo pins via shift registers.
            let sr_disabled = self.state.with_mut(|st| {
                st.sr_enabled[index] |= 1;
                st.sr_disabled[index] |= 1;
                st.sr_disabled.clone()
            });
            self.sr_spi().spi_send(&sr_disabled, SendOpts::default())?;
            let servo = ServoPwm::new(self.this(), &pin_params)?;
            return Ok(Pin::Pwm(servo as PwmPin));
        }
        Err(pins_error(&fmt("Unknown replicape pin %s", &[PctValue::Str(pin)])?))
    }

    /// The Python object has no `get_mcu`; the contract needs one for the
    /// `chip` field of the pin params it hands out.
    fn get_mcu(&self) -> KResult<Mcu> {
        Ok(self.host_mcu())
    }
}

// ---------------------------------------------------------------------------
// PCA9685 PWM pin.
// ---------------------------------------------------------------------------

struct Pca9685State {
    max_duration: f64,
    start_value: f64,
    shutdown_value: f64,
    is_enable: bool,
    last_clock: i64,
    pwm_max: f64,
    oid: i64,
    set_cmd: Option<CommandWrapper>,
}

/// `class pca9685_pwm`.
pub struct Pca9685Pwm {
    replicape: Arc<Replicape>,
    mcu: Mcu,
    reactor: Reactor,
    channel: i64,
    invert: i64,
    state: GCell<Pca9685State>,
}

impl Pca9685Pwm {
    /// `pca9685_pwm(replicape, channel, pin_type, pin_params)`
    /// (`replicape.py:17`).
    pub fn new(
        replicape: Arc<Replicape>,
        channel: i64,
        pin_type: &str,
        pin_params: &PinParams,
    ) -> KResult<Arc<Pca9685Pwm>> {
        if pin_type != "digital_out" && pin_type != "pwm" {
            return Err(pins_error("Pin type not supported on replicape"));
        }
        let mcu = replicape.host_mcu();
        let reactor = mcu.get_printer().get_reactor();
        let start = if pin_params.invert != 0 { 1. } else { 0. };
        let this = Arc::new(Pca9685Pwm {
            replicape,
            mcu: mcu.clone(),
            reactor,
            channel,
            invert: pin_params.invert,
            state: GCell::new(Pca9685State {
                max_duration: 2.,
                start_value: start,
                shutdown_value: start,
                is_enable: start != 0.,
                last_clock: 0,
                pwm_max: 0.,
                oid: 0,
                set_cmd: None,
            }),
        });
        let cb_self = Arc::clone(&this);
        mcu.register_config_callback(Box::new(move || cb_self.build_config()))?;
        Ok(this)
    }

    /// `_build_config` (`replicape.py:55`).
    fn build_config(&self) -> KResult<()> {
        let pwm_max = self.mcu.get_constant_float("PCA9685_MAX")?;
        self.state.with_mut(|st| st.pwm_max = pwm_max);
        let cycle_ticks = self.mcu.seconds_to_clock(REPLICAPE_PCA9685_CYCLE_TIME);
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid()?;
        self.state.with_mut(|st| st.oid = oid);
        let (start_value, shutdown_value, max_duration) = {
            let st = self.state.borrow();
            (st.start_value, st.shutdown_value, st.max_duration)
        };
        let cmd = fmt(
            concat!(
                "config_pca9685 oid=%d bus=%d addr=%d channel=%d cycle_ticks=%d",
                " value=%d default_value=%d max_duration=%d"
            ),
            &[
                pct_int(oid),
                pct_int(REPLICAPE_PCA9685_BUS),
                pct_int(REPLICAPE_PCA9685_ADDRESS),
                pct_int(self.channel),
                pct_int(cycle_ticks),
                PctValue::Float(start_value * pwm_max),
                PctValue::Float(shutdown_value * pwm_max),
                pct_int(self.mcu.seconds_to_clock(max_duration)),
            ],
        )?;
        self.mcu.add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        let cmd_queue = self.mcu.alloc_command_queue();
        let set_cmd = self
            .mcu
            .lookup_command("queue_pca9685_out oid=%c clock=%u value=%hu", Some(cmd_queue))?;
        self.state.with_mut(|st| st.set_cmd = Some(set_cmd));
        Ok(())
    }

    /// `set_pwm(print_time, value)` (`replicape.py:72`).
    pub fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        let value = if self.invert != 0 { 1. - value } else { value };
        let pwm_max = self.state.borrow().pwm_max;
        let value = trunc_float_to_bigint(py_max(0., py_min(1., value)) * pwm_max + 0.5);
        let is_enable = !value.is_zero();
        let changed = self.state.with_mut(|st| {
            let changed = is_enable != st.is_enable;
            st.is_enable = is_enable;
            changed
        });
        if changed {
            let replicape = Arc::clone(&self.replicape);
            let channel = self.channel;
            self.reactor.register_async_callback(
                move |_eventtime: f64| replicape.note_pwm_enable(print_time, channel, is_enable),
                0.,
            );
        }
        let (set_cmd, oid, last_clock) = {
            let st = self.state.borrow();
            (st.set_cmd.clone(), st.oid, st.last_clock)
        };
        // `self._set_cmd.send(...)`: `_set_cmd` is `None` until the config
        // callback ran.
        let Some(cmd) = set_cmd else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'send'".to_string(),
            });
        };
        let data = vec![
            PyValue::Int(PyInt::from_i64(oid)),
            PyValue::Int(PyInt::from_i64(clock)),
            PyValue::Int(value),
        ];
        cmd.send(data, SendOpts { minclock: last_clock, reqclock: clock })?;
        self.state.with_mut(|st| st.last_clock = clock);
        Ok(())
    }

    /// `set_digital(print_time, value)` (`replicape.py:86`).
    pub fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        self.set_pwm(print_time, if value { 1. } else { 0. })
    }

    /// `setup_start_value(start_value, shutdown_value)` (`replicape.py:46`).
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let (start_value, shutdown_value) = if self.invert != 0 {
            (1. - start_value, 1. - shutdown_value)
        } else {
            (start_value, shutdown_value)
        };
        let start_value = py_max(0., py_min(1., start_value));
        let shutdown_value = py_max(0., py_min(1., shutdown_value));
        self.state.with_mut(|st| {
            st.start_value = start_value;
            st.shutdown_value = shutdown_value;
        });
        self.replicape.note_pwm_start_value(self.channel, start_value, shutdown_value);
        self.state.with_mut(|st| st.is_enable = start_value != 0.);
    }
}

impl PwmPinApi for Pca9685Pwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.state.with_mut(|st| st.max_duration = max_duration);
    }
    /// `setup_cycle_time` (`replicape.py:40`).
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) -> KResult<()> {
        if hardware_pwm.unwrap_or(false) {
            return Err(pins_error("pca9685 does not support hardware_pwm parameter"));
        }
        if cycle_time != REPLICAPE_PCA9685_CYCLE_TIME {
            if let Ok(msg) = fmt(
                "Ignoring pca9685 cycle time of %.6f (using %.6f)",
                &[PctValue::Float(cycle_time), PctValue::Float(REPLICAPE_PCA9685_CYCLE_TIME)],
            ) {
                log::info(&msg);
            }
        }
        Ok(())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        Pca9685Pwm::setup_start_value(self, start_value, shutdown_value);
    }
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        Pca9685Pwm::set_pwm(self, print_time, value)
    }
}

impl DigitalOutPinApi for Pca9685Pwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.state.with_mut(|st| st.max_duration = max_duration);
    }
    /// The Python class has no `setup_start_value` taking booleans; the
    /// trait asks for one, so it forwards the 0/1 floats.
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        Pca9685Pwm::setup_start_value(
            self,
            if start_value { 1. } else { 0. },
            if shutdown_value { 1. } else { 0. },
        );
    }
    fn set_digital(&self, print_time: f64, value: bool) {
        if let Err(e) = Pca9685Pwm::set_digital(self, print_time, value) {
            e.raise();
        }
    }
}

// ---------------------------------------------------------------------------
// DAC stepper enable pin.
// ---------------------------------------------------------------------------

/// `class ReplicapeDACEnable`.
pub struct ReplicapeDacEnable {
    mcu: Mcu,
    value: f64,
    pwm: Arc<Pca9685Pwm>,
}

impl ReplicapeDacEnable {
    /// `ReplicapeDACEnable(replicape, channel, pin_type, pin_params)`
    /// (`replicape.py:93`).
    pub fn new(
        replicape: Arc<Replicape>,
        channel: i64,
        pin_type: &str,
        pin_params: &PinParams,
    ) -> KResult<Arc<ReplicapeDacEnable>> {
        if pin_type != "digital_out" {
            return Err(pins_error("Replicape virtual enable pin must be digital_out"));
        }
        if pin_params.invert != 0 {
            return Err(pins_error("Replicape virtual enable pin can not be inverted"));
        }
        let mcu = replicape.host_mcu();
        let value = replicape.state.borrow().stepper_dacs.get(&channel).copied();
        let Some(value) = value else {
            return Err(KlippyError::Other {
                class_name: "KeyError".to_string(),
                message: channel.to_string(),
            });
        };
        let pwm = Pca9685Pwm::new(replicape, channel, pin_type, pin_params)?;
        Ok(Arc::new(ReplicapeDacEnable { mcu, value, pwm }))
    }
}

impl DigitalOutPinApi for ReplicapeDacEnable {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        PwmPinApi::setup_max_duration(&*self.pwm, max_duration);
    }
    /// The Python class has no `setup_start_value` (calling it is an
    /// `AttributeError`); the trait requires one, so it reaches the PWM pin.
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        DigitalOutPinApi::setup_start_value(&*self.pwm, start_value, shutdown_value);
    }
    fn set_digital(&self, print_time: f64, value: bool) {
        let result = if value {
            self.pwm.set_pwm(print_time, self.value)
        } else {
            self.pwm.set_pwm(print_time, 0.)
        };
        if let Err(e) = result {
            e.raise();
        }
    }
}

// ---------------------------------------------------------------------------
// Servo PWM pin (host Linux PWM + reserved pins).
// ---------------------------------------------------------------------------

/// `class servo_pwm`.
pub struct ServoPwm {
    mcu_pwm: PwmPin,
}

impl ServoPwm {
    /// `servo_pwm(replicape, pin_params)` (`replicape.py:117`).
    pub fn new(replicape: Arc<Replicape>, pin_params: &PinParams) -> KResult<Arc<ServoPwm>> {
        let config_name = pin_params.pin.clone();
        let host_mcu = replicape.host_mcu();
        let mut pwmchip = "pwmchip0".to_string();
        if !host_mcu.is_fileoutput() {
            // Determine the pwmchip number for the servo channels. The
            // directory holds only a "pwmchipX" entry on the beagle bone black.
            let found = std::fs::read_dir(
                "/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/",
            )
            .ok()
            .and_then(|entries| {
                entries
                    .flatten()
                    .map(|entry| entry.file_name().to_string_lossy().into_owned())
                    .find(|name| name.starts_with("pwmchip"))
            });
            match found {
                Some(name) => pwmchip = name,
                None => return Err(pins_error("Replicape unable to determine pwmchip")),
            }
        }
        let (pwm_pin, resv1, resv2) = SERVO_PINS
            .iter()
            .find(|(name, _)| *name == config_name)
            .map(|(_, pins)| *pins)
            .ok_or_else(|| KlippyError::Other {
                class_name: "KeyError".to_string(),
                message: pycompat::text::str_repr(&config_name),
            })?;
        let mut pwm_params = pin_params.clone();
        pwm_params.pin = format!("{pwmchip}{pwm_pin}");
        // Setup actual pwm pin using linux hardware pwm on host.
        let mcu_pwm = match host_mcu.setup_pin("pwm", pwm_params)? {
            Pin::Pwm(pwm) => pwm,
            Pin::DigitalOut(_) => return Err(attribute_error("MCU_digital_out", "set_pwm")),
            Pin::Endstop(_) => return Err(attribute_error("MCU_endstop", "setup_max_duration")),
            Pin::Adc(_) => return Err(attribute_error("MCU_adc", "setup_max_duration")),
        };
        // Reserve pins to warn user of conflicts.
        let pru_mcu = replicape.mcu_pwm_enable().get_mcu();
        let printer = pru_mcu.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_resolver = ppins.get_pin_resolver(&pru_mcu.get_name())?;
        pin_resolver.reserve_pin(resv1, &config_name)?;
        pin_resolver.reserve_pin(resv2, &config_name)?;
        Ok(Arc::new(ServoPwm { mcu_pwm }))
    }
}

impl PwmPinApi for ServoPwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu_pwm.get_mcu()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.mcu_pwm.setup_max_duration(max_duration);
    }
    /// `setup_cycle_time` always asks the host PWM for hardware PWM
    /// (`replicape.py:150`).
    fn setup_cycle_time(&self, cycle_time: f64, _hardware_pwm: Option<bool>) -> KResult<()> {
        self.mcu_pwm.setup_cycle_time(cycle_time, Some(true))
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        self.mcu_pwm.setup_start_value(start_value, shutdown_value);
    }
    fn next_aligned_print_time(&self, print_time: f64, allow_early: Option<f64>) -> f64 {
        self.mcu_pwm.next_aligned_print_time(print_time, allow_early)
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        self.mcu_pwm.set_pwm(print_time, value)
    }
}

/// `def load_config(config)` (`replicape.py:267`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let replicape = Replicape::new(config)?;
    Ok(Some(replicape as ObjectRef))
}

/// Module factory for the `[replicape]` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
