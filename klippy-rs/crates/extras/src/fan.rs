//! Port of `klippy/extras/fan.py`.
//!
//! `Fan` drives a PWM pin (plus an optional digital enable pin) through
//! `crate::output_pin::GCodeRequestQueue`, with `Fan._apply_speed` as the
//! queue callback. The tachometer delegates to
//! `crate::pulse_counter::FrequencyCounter`.

use indexmap::IndexMap;
use pycompat::PyValue;
use std::sync::Arc;

use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetIntOpts, GetStrOpts};
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts as GcmdGetFloatOpts, RegisterCommandOpts};
use klippy_api::mcu::{DigitalOutPin, Mcu, Pin, PinsApi, PwmPin};
use klippy_api::object::{EventArg, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, GCell, KResult, KlippyError};

use crate::output_pin::{GCodeRequestQueue, GcrqCallback, GcrqCallbackReturn};
use crate::pulse_counter::FrequencyCounter;

fn py_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

fn py_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

// ---------------------------------------------------------------------------
// FanTachometer (klippy/extras/fan.py:83-102, owned by unit
// `extras.fan.FanTachometer`)
// ---------------------------------------------------------------------------

pub struct FanTachometer {
    ppr: i64,
    counter: Option<Arc<FrequencyCounter>>,
}

impl FanTachometer {
    /// `FanTachometer.__init__(config)` (`fan.py:84`).
    pub fn new(printer: &Printer, config: &Config) -> KResult<FanTachometer> {
        let pin = config.get(
            "tachometer_pin",
            GetStrOpts {
                default: Some(None),
                ..GetStrOpts::default()
            },
        )?;
        match pin {
            None => Ok(FanTachometer {
                ppr: 0,
                counter: None,
            }),
            Some(pin_desc) => {
                let ppr = config.getint(
                    "tachometer_ppr",
                    GetIntOpts {
                        default: Some(Some(2)),
                        minval: Some(1),
                        ..GetIntOpts::default()
                    },
                )?.unwrap_or(2);
                let poll_time = config.getfloat(
                    "tachometer_poll_interval",
                    GetFloatOpts {
                        default: Some(Some(0.0015)),
                        above: Some(0.),
                        ..GetFloatOpts::default()
                    },
                )?.unwrap_or(0.0015);
                let counter = FrequencyCounter::new(printer, &pin_desc, 1.0, poll_time)?;
                Ok(FanTachometer {
                    ppr,
                    counter: Some(Arc::new(counter)),
                })
            }
        }
    }

    /// `FanTachometer.get_status(eventtime)` (`fan.py:97`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let rpm = match &self.counter {
            Some(counter) => PyValue::Float(counter.get_frequency() * 30.0 / (self.ppr as f64)),
            None => PyValue::None,
        };
        let mut m = IndexMap::new();
        m.insert(PyValue::Str("rpm".to_string()), rpm);
        PyValue::Dict(m)
    }
}

// ---------------------------------------------------------------------------
// Fan (klippy/extras/fan.py:8-81, owned by unit `extras.fan.ModuleHeader`)
// ---------------------------------------------------------------------------

struct FanState {
    last_fan_value: f64,
    last_req_value: f64,
}

/// The part of `Fan` that `_apply_speed` needs; shared with the request
/// queue callback.
struct FanCore {
    pwm: PwmPin,
    enable: Option<DigitalOutPin>,
    max_power: f64,
    kick_start_time: f64,
    off_below: f64,
    state: GCell<FanState>,
}

impl FanCore {
    /// `Fan._apply_speed(print_time, value)` (`fan.py:49-68`).
    fn apply_speed(&self, print_time: f64, value: f64) -> KResult<GcrqCallbackReturn> {
        let mut value = value;
        if value < self.off_below {
            value = 0.0;
        }
        value = py_max(0.0, py_min(self.max_power, value * self.max_power));
        let last = self.state.borrow().last_fan_value;
        if value == last {
            return Ok(Some(("discard".to_string(), 0.0)));
        }
        if let Some(enable) = &self.enable {
            if value > 0.0 && last == 0.0 {
                enable.set_digital(print_time, true);
            } else if value == 0.0 && last > 0.0 {
                enable.set_digital(print_time, false);
            }
        }
        if value != 0.0
            && self.kick_start_time != 0.0
            && (last == 0.0 || value - last > 0.5)
        {
            // Run fan at full speed for specified kick_start_time.
            {
                let mut st = self.state.borrow_mut();
                st.last_req_value = value;
                st.last_fan_value = self.max_power;
            }
            self.pwm.set_pwm(print_time, self.max_power)?;
            return Ok(Some(("repeat".to_string(), print_time + self.kick_start_time)));
        }
        {
            let mut st = self.state.borrow_mut();
            st.last_fan_value = value;
            st.last_req_value = value;
        }
        self.pwm.set_pwm(print_time, value)?;
        Ok(None)
    }
}

pub struct Fan {
    core: Arc<FanCore>,
    mcu: Mcu,
    gcrq: Arc<GCodeRequestQueue>,
    tachometer: FanTachometer,
}

fn wrong_pin(pin_type: &str, desc: &str) -> KlippyError {
    KlippyError::config(format!(
        "setup_pin('{}') for '{}' returned a pin of another type",
        pin_type, desc
    ))
}

impl Fan {
    /// `Fan.__init__(config, default_shutdown_speed=0.)` (`fan.py:9-45`).
    pub fn new(config: &Config, default_shutdown_speed: f64) -> KResult<Arc<Fan>> {
        let printer = config.get_printer();

        let max_power = config
            .getfloat(
                "max_power",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    above: Some(0.0),
                    maxval: Some(1.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(1.0);
        let kick_start_time = config
            .getfloat(
                "kick_start_time",
                GetFloatOpts {
                    default: Some(Some(0.1)),
                    minval: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.1);
        let off_below = config
            .getfloat(
                "off_below",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    maxval: Some(1.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.0);
        let cycle_time = config
            .getfloat(
                "cycle_time",
                GetFloatOpts {
                    default: Some(Some(0.010)),
                    above: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.010);
        let hardware_pwm = config
            .getboolean(
                "hardware_pwm",
                GetBoolOpts {
                    default: Some(Some(false)),
                    ..GetBoolOpts::default()
                },
            )?
            .unwrap_or(false);
        let shutdown_speed = config
            .getfloat(
                "shutdown_speed",
                GetFloatOpts {
                    default: Some(Some(default_shutdown_speed)),
                    minval: Some(0.0),
                    maxval: Some(1.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(default_shutdown_speed);

        // Setup pwm object (fan.py:23-28).
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_desc = config
            .get("pin", GetStrOpts::default())?
            .expect("GetStrOpts::default means required, which raises first");
        let pwm = match pins.setup_pin("pwm", &pin_desc)? {
            Pin::Pwm(p) => p,
            _ => return Err(wrong_pin("pwm", &pin_desc)),
        };
        pwm.setup_max_duration(0.0);
        pwm.setup_cycle_time(cycle_time, Some(hardware_pwm))?;
        let shutdown_power = py_max(0.0, py_min(max_power, shutdown_speed));
        pwm.setup_start_value(0.0, shutdown_power);

        let enable_pin = config.get(
            "enable_pin",
            GetStrOpts {
                default: Some(None),
                ..GetStrOpts::default()
            },
        )?;
        let enable = match enable_pin {
            None => None,
            Some(desc) => match pins.setup_pin("digital_out", &desc)? {
                Pin::DigitalOut(d) => {
                    d.setup_max_duration(0.0);
                    Some(d)
                }
                _ => return Err(wrong_pin("digital_out", &desc)),
            },
        };

        let mcu = pwm.get_mcu();
        let core = Arc::new(FanCore {
            pwm,
            enable,
            max_power,
            kick_start_time,
            off_below,
            state: GCell::new(FanState {
                last_fan_value: 0.0,
                last_req_value: 0.0,
            }),
        });

        // Create gcode request queue (fan.py:37).
        let callback_core = core.clone();
        let callback: GcrqCallback =
            Arc::new(move |print_time, value| callback_core.apply_speed(print_time, value));
        let gcrq = GCodeRequestQueue::new(config, mcu.clone(), callback)?;

        // Setup tachometer (fan.py:41).
        let tachometer = FanTachometer::new(&printer, config)?;

        let fan = Arc::new(Fan {
            core,
            mcu,
            gcrq,
            tachometer,
        });

        // Register callbacks (fan.py:44-45).
        let restart_fan = fan.clone();
        printer.register_event_handler(
            "gcode:request_restart",
            Box::new(move |args: &[EventArg]| {
                let print_time = match args.first() {
                    Some(EventArg::Value(v)) => v.to_f64().unwrap_or(0.0),
                    _ => 0.0,
                };
                restart_fan.handle_request_restart(print_time)?;
                Ok(PyValue::None)
            }),
        );
        Ok(fan)
    }

    /// `Fan.get_mcu()` (`fan.py:47`).
    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    /// `Fan._apply_speed(print_time, value)` (`fan.py:49`).
    pub fn apply_speed(&self, print_time: f64, value: f64) -> KResult<GcrqCallbackReturn> {
        self.core.apply_speed(print_time, value)
    }

    /// `Fan.set_speed(value, print_time=None)` (`fan.py:69`).
    pub fn set_speed(&self, value: f64, print_time: Option<f64>) -> KResult<()> {
        self.gcrq.send_async_request(value, print_time)
    }

    /// `Fan.set_speed_from_command(value)` (`fan.py:71`).
    pub fn set_speed_from_command(&self, value: f64) -> KResult<()> {
        self.gcrq.queue_gcode_request(value)
    }

    /// `Fan._handle_request_restart(print_time)` (`fan.py:73-74`).
    pub fn handle_request_restart(&self, print_time: f64) -> KResult<()> {
        self.set_speed(0.0, Some(print_time))
    }

    /// `Fan.get_status(eventtime)` (`fan.py:76-81`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        let tach = self.tachometer.get_status(eventtime);
        let rpm = match &tach {
            PyValue::Dict(m) => m
                .get(&PyValue::Str("rpm".to_string()))
                .cloned()
                .unwrap_or(PyValue::None),
            _ => PyValue::None,
        };
        let speed = self.core.state.borrow().last_req_value;
        let mut m = IndexMap::new();
        m.insert(PyValue::Str("speed".to_string()), PyValue::Float(speed));
        m.insert(PyValue::Str("rpm".to_string()), rpm);
        PyValue::Dict(m)
    }

    pub fn last_fan_value(&self) -> f64 {
        self.core.state.borrow().last_fan_value
    }

    pub fn last_req_value(&self) -> f64 {
        self.core.state.borrow().last_req_value
    }
}

// ---------------------------------------------------------------------------
// PrinterFan (klippy/extras/fan.py:104-122)
// ---------------------------------------------------------------------------

pub struct PrinterFan {
    fan: Arc<Fan>,
}

impl PrinterFan {
    /// `PrinterFan.__init__(config)` (`fan.py:105-110`).
    pub fn new(config: &Config) -> KResult<Arc<PrinterFan>> {
        let printer = config.get_printer();
        let fan = Fan::new(config, 0.0)?;
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let fan_m106 = fan.clone();
        gcode.register_command(
            "M106",
            Some(Box::new(move |gcmd: &GCmd| m106(&fan_m106, gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let fan_m107 = fan.clone();
        gcode.register_command(
            "M107",
            Some(Box::new(move |_gcmd: &GCmd| fan_m107.set_speed_from_command(0.0))),
            RegisterCommandOpts::default(),
        )?;
        Ok(Arc::new(PrinterFan { fan }))
    }

    pub fn fan(&self) -> &Arc<Fan> {
        &self.fan
    }

    /// `PrinterFan.get_status(eventtime)` (`fan.py:111-112`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.fan.get_status(eventtime)
    }

    /// `PrinterFan.cmd_M106(gcmd)` (`fan.py:113-116`).
    pub fn cmd_m106(&self, gcmd: &GCmd) -> KResult<()> {
        m106(&self.fan, gcmd)
    }

    /// `PrinterFan.cmd_M107(gcmd)` (`fan.py:117-119`).
    pub fn cmd_m107(&self, _gcmd: &GCmd) -> KResult<()> {
        self.fan.set_speed_from_command(0.0)
    }
}

impl PrinterObject for PrinterFan {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_status(eventtime)))
    }
}

fn m106(fan: &Fan, gcmd: &GCmd) -> KResult<()> {
    let value = gcmd
        .get_float(
            "S",
            GcmdGetFloatOpts {
                default: Some(Some(255.0)),
                minval: Some(0.0),
                ..GcmdGetFloatOpts::default()
            },
        )?
        .unwrap_or(255.0)
        / 255.0;
    fan.set_speed_from_command(value)
}

/// `load_config(config)` (`fan.py:121-122`).
pub fn load_config(config: &Config) -> KResult<Option<klippy_api::object::ObjectRef>> {
    let pf = PrinterFan::new(config)?;
    Ok(Some(pf as klippy_api::object::ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
