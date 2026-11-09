//! Port of `klippy/extras/servo.py`.

use klippy_api::{
    error::{KResult, KlippyError},
    gcell::GCell,
    gcode::{GCmd, GCodeApi},
    load_interface, lookup_interface,
    mcu::{Mcu, Pin, PwmPin, PinsApi},
    object::{EventArg, ObjectRef, PrinterObject},
    printer::{LookupDefault, ModuleFactory, Printer},
    toolhead::{FlushCallback, MotionQueuing, MotionQueuingApi, ToolHead, ToolHeadApi},
};
use pycompat::PyValue;
use std::sync::Arc;

pub const SERVO_SIGNAL_PERIOD: f64 = 0.020;
pub const RESCHEDULE_SLACK: f64 = 0.000500;

pub const CMD_SET_SERVO_HELP: &str = "Set servo angle";

/// Python `min(x, y)`: `y if y < x else x` (NaN-safe, keeps the first arg on NaN).
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python `max(x, y)`: `y if x < y else x` (NaN-safe, keeps the first arg on NaN).
fn py_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

fn get_pwm_from_angle_inner(
    min_width: f64,
    max_angle: f64,
    angle_to_width: f64,
    width_to_value: f64,
    angle: f64,
) -> f64 {
    let clamped = py_max(0., py_min(max_angle, angle));
    let width = min_width + clamped * angle_to_width;
    width * width_to_value
}

fn get_pwm_from_pulse_width_inner(
    min_width: f64,
    max_width: f64,
    width_to_value: f64,
    width: f64,
) -> f64 {
    let w = if width != 0. {
        py_max(min_width, py_min(max_width, width))
    } else {
        width
    };
    w * width_to_value
}

fn config_float_opts(
    default: Option<Option<f64>>,
    minval: Option<f64>,
    maxval: Option<f64>,
    above: Option<f64>,
    below: Option<f64>,
) -> klippy_api::config::GetFloatOpts {
    klippy_api::config::GetFloatOpts {
        default,
        minval,
        maxval,
        above,
        below,
        note_valid: true,
    }
}

struct ServoInner {
    mcu_servo: PwmPin,
    mcu: Mcu,
    motion_queuing: MotionQueuing,
    last_value: GCell<f64>,
    rqueue: GCell<Vec<(f64, f64)>>,
    next_min_flush_time: GCell<f64>,
    toolhead: GCell<Option<ToolHead>>,
}

impl ServoInner {
    fn set_pwm(&self, print_time: f64, value: f64) -> Option<(String, f64)> {
        if value == self.last_value.get() {
            return Some(("discard".to_string(), 0.));
        }
        let aligned_ptime = self
            .mcu_servo
            .next_aligned_print_time(print_time, Some(RESCHEDULE_SLACK));
        if aligned_ptime > print_time + RESCHEDULE_SLACK {
            return Some(("reschedule".to_string(), aligned_ptime));
        }
        self.last_value.set(value);
        self.mcu_servo.set_pwm(aligned_ptime, value);
        None
    }

    fn flush_notification(&self, must_flush_time: f64, _max_step_gen_time: f64) {
        let min_sched_time = self.mcu.min_schedule_time();
        let mut rqueue = self.rqueue.borrow_mut();
        let mut next_min = self.next_min_flush_time.borrow_mut();
        while !rqueue.is_empty() {
            let next_time = py_max(rqueue[0].0, *next_min);
            if next_time > must_flush_time {
                return;
            }
            let mut pos = 0usize;
            while pos + 1 < rqueue.len() && rqueue[pos + 1].0 <= next_time {
                pos += 1;
            }
            let req_val = rqueue[pos].1;
            let ret = self.set_pwm(next_time, req_val);
            if let Some((action, next_min_time)) = ret {
                *next_min = py_max(*next_min, next_min_time);
                if action == "discard" {
                    rqueue.drain(..pos + 1);
                    continue;
                }
                if action == "reschedule" {
                    rqueue.drain(..pos);
                    continue;
                }
                if action == "repeat" {
                    if pos == 0 {
                        // Python: pos=-1, del rqueue[:0] deletes nothing.
                        // Fall through to the note below without draining.
                        *next_min = py_max(*next_min, next_time + min_sched_time);
                        let nmin = *next_min;
                        let mq = self.motion_queuing.clone();
                        drop(rqueue);
                        drop(next_min);
                        mq.note_mcu_movequeue_activity(nmin, false);
                        rqueue = self.rqueue.borrow_mut();
                        next_min = self.next_min_flush_time.borrow_mut();
                        continue;
                    } else {
                        pos -= 1;
                    }
                }
            }
            rqueue.drain(..pos + 1);
            *next_min = py_max(*next_min, next_time + min_sched_time);
            let nmin = *next_min;
            let mq = self.motion_queuing.clone();
            drop(rqueue);
            drop(next_min);
            mq.note_mcu_movequeue_activity(nmin, false);
            // Re-borrow for the next iteration.
            rqueue = self.rqueue.borrow_mut();
            next_min = self.next_min_flush_time.borrow_mut();
        }
    }
}

#[allow(dead_code)]
pub struct PrinterServo {
    printer: Printer,
    min_width: f64,
    max_width: f64,
    max_angle: f64,
    angle_to_width: f64,
    width_to_value: f64,
    inner: Arc<ServoInner>,
}

impl PrinterServo {
    pub fn new(config: &klippy_api::config::Config) -> KResult<Arc<PrinterServo>> {
        let printer = config.get_printer();
        let min_width = config
            .getfloat(
                "minimum_pulse_width",
                config_float_opts(Some(Some(0.001)), None, None, Some(0.), Some(SERVO_SIGNAL_PERIOD)),
            )?
            .unwrap_or(0.001);
        let max_width = config
            .getfloat(
                "maximum_pulse_width",
                config_float_opts(Some(Some(0.002)), None, None, Some(min_width), Some(SERVO_SIGNAL_PERIOD)),
            )?
            .unwrap_or(0.002);
        let max_angle = config
            .getfloat("maximum_servo_angle", config_float_opts(Some(Some(180.)), None, None, None, None))?
            .unwrap_or(180.);
        if max_angle == 0. {
            return Err(KlippyError::Other {
                class_name: "ZeroDivisionError".to_string(),
                message: "float division by zero".to_string(),
            });
        }
        let angle_to_width = (max_width - min_width) / max_angle;
        let width_to_value = 1. / SERVO_SIGNAL_PERIOD;
        let initial_pwm = {
            let iangle = config.getfloat(
                "initial_angle",
                config_float_opts(Some(None), Some(0.), Some(360.), None, None),
            )?;
            match iangle {
                Some(v) => get_pwm_from_angle_inner(min_width, max_angle, angle_to_width, width_to_value, v),
                None => {
                    let iwidth = config
                        .getfloat(
                            "initial_pulse_width",
                            config_float_opts(Some(Some(0.)), Some(0.), Some(max_width), None, None),
                        )?
                        .unwrap_or(0.);
                    get_pwm_from_pulse_width_inner(min_width, max_width, width_to_value, iwidth)
                }
            }
        };
        // Setup mcu_servo pin
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_desc = config
            .get("pin", klippy_api::config::GetStrOpts::default())?
            .ok_or_else(|| KlippyError::config("Option 'pin' in section must be specified"))?;
        let pin = ppins.setup_pin("pwm", &pin_desc)?;
        let mcu_servo = match pin {
            Pin::Pwm(p) => p,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "setup_pin 'pwm' did not return a pwm pin".to_string(),
                })
            }
        };
        mcu_servo.setup_max_duration(0.);
        mcu_servo.setup_cycle_time(SERVO_SIGNAL_PERIOD, Some(false));
        mcu_servo.setup_start_value(initial_pwm, 0.);
        let mcu = mcu_servo.get_mcu();
        // motion_queuing (GCodeRequestQueue equivalent, inlined from output_pin.py)
        let motion_queuing = load_interface::<dyn MotionQueuingApi>(&*printer, config, "motion_queuing", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let inner = Arc::new(ServoInner {
            mcu_servo,
            mcu,
            motion_queuing: motion_queuing.clone(),
            last_value: GCell::new(0.),
            rqueue: GCell::new(Vec::new()),
            next_min_flush_time: GCell::new(0.),
            toolhead: GCell::new(None),
        });
        let servo = Arc::new(PrinterServo {
            printer: printer.clone(),
            min_width,
            max_width,
            max_angle,
            angle_to_width,
            width_to_value,
            inner: inner.clone(),
        });
        // register flush callback (`output_pin.GCodeRequestQueue`
        // equivalent, inlined; `output_pin.py:24`): the inlined closure has
        // no `__qualname__` of its own, so report the same
        // `_flush_notification` Python would resolve on the queue object.
        {
            let inner_cb = inner.clone();
            let fb: FlushCallback = Arc::new(move |must: f64, max_step: f64| {
                inner_cb.flush_notification(must, max_step);
            });
            motion_queuing.register_flush_callback_named(fb, false, Some("_flush_notification"), None);
        }
        // register klippy:connect handler for toolhead
        {
            let inner_cb = inner.clone();
            let printer_cb = printer.clone();
            printer.register_event_handler(
                "klippy:connect",
                Box::new(move |_params: &[EventArg]| {
                    let th = lookup_interface::<dyn ToolHeadApi>(&*printer_cb, "toolhead", LookupDefault::Raise)?
                        .expect("LookupDefault::Raise never yields None");
                    *inner_cb.toolhead.borrow_mut() = Some(th);
                    Ok(PyValue::None)
                }),
            );
        }
        // Register commands
        let servo_name = config
            .get_name()
            .split_whitespace()
            .nth(1)
            .map(|s| s.to_string())
            .ok_or_else(|| KlippyError::Other {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?;
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        {
            let servo_cb = Arc::clone(&servo);
            gcode.register_mux_command(
                "SET_SERVO",
                "SERVO",
                Some(&servo_name),
                Box::new(move |gcmd: &GCmd| servo_cb.cmd_set_servo(gcmd)),
                Some(CMD_SET_SERVO_HELP),
            )?;
        }
        Ok(servo)
    }

    pub fn get_status_value(&self) -> f64 {
        self.inner.last_value.get()
    }

    pub fn get_pwm_from_angle(&self, angle: f64) -> f64 {
        get_pwm_from_angle_inner(self.min_width, self.max_angle, self.angle_to_width, self.width_to_value, angle)
    }

    pub fn get_pwm_from_pulse_width(&self, width: f64) -> f64 {
        get_pwm_from_pulse_width_inner(self.min_width, self.max_width, self.width_to_value, width)
    }

    pub fn queue_gcode_request(&self, value: f64) -> KResult<()> {
        let th = match self.inner.toolhead.cloned() {
            Some(t) => t,
            None => {
                return Err(KlippyError::Other {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'register_lookahead_callback'".to_string(),
                })
            }
        };
        let inner = self.inner.clone();
        th.register_lookahead_callback(Box::new(move |pt: f64| {
            inner.rqueue.borrow_mut().push((pt, value));
            inner.motion_queuing.note_mcu_movequeue_activity(pt, false);
            Ok(())
        }))
    }

    pub fn cmd_set_servo(&self, gcmd: &GCmd) -> KResult<()> {
        let width = gcmd.get_float(
            "WIDTH",
            klippy_api::gcode::GetFloatOpts {
                default: Some(None),
                minval: None,
                maxval: None,
                above: None,
                below: None,
            },
        )?;
        let value = match width {
            Some(w) => self.get_pwm_from_pulse_width(w),
            None => {
                let angle = gcmd
                    .get_float(
                        "ANGLE",
                        klippy_api::gcode::GetFloatOpts {
                            default: None,
                            minval: None,
                            maxval: None,
                            above: None,
                            below: None,
                        },
                    )?
                    .unwrap();
                self.get_pwm_from_angle(angle)
            }
        };
        self.queue_gcode_request(value)
    }
}

impl PrinterObject for PrinterServo {
    klippy_api::printer_object_base!();
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut map = indexmap::IndexMap::new();
        map.insert(
            PyValue::Str("value".to_string()),
            PyValue::Float(self.inner.last_value.get()),
        );
        Some(PyValue::Dict(map))
    }
}

pub fn load_config_prefix(
    config: &klippy_api::config::Config,
) -> KResult<Option<ObjectRef>> {
    let servo = PrinterServo::new(config)?;
    Ok(Some(servo))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
