//! Port of the `led` module (`klippy/extras/led.py`).
//!
//! [`LEDHelper`] is the shared color state used by `PrinterPWMLED` here and by
//! the neopixel driver; [`PrinterPWMLED`] is the `[led <name>]` object behind
//! `SET_LED` / `SET_LED_TEMPLATE`. The template evaluator is the ported
//! `output_pin` object, reached through `output_pin.lookup_template_eval`.

use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandHandler, GCmd, GetFloatOpts as GcmdGetFloatOpts,
    GetIntOpts as GcmdGetIntOpts,
};
use klippy_api::mcu::{McuApi, Pin, PinsApi, PwmPin};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{LookaheadCallback, ToolHeadApi};
use klippy_api::{GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::ReactorMutex;

use crate::output_pin::{
    lookup_template_eval, FlushCallback, PrinterTemplateEvaluator, TemplateCallback,
};

/// `cmd_SET_LED_help` (`led.py:92`).
const CMD_SET_LED_HELP: &str = "Set the color of an LED";
/// `cmd_SET_LED_TEMPLATE_help` (`led.py:113`).
const CMD_SET_LED_TEMPLATE_HELP: &str = "Assign a display_template to an LED";

/// The `update_func(led_state, print_time)` argument of `LEDHelper`.
///
/// `led_state` is the color list as it was when the transmit was requested and
/// `print_time` is `None` for the immediate (`SYNC=0`) path.
pub type LedUpdateFunc = Arc<dyn Fn(&[LedColor], Option<f64>) -> KResult<()> + Send + Sync>;

/// A color tuple `(red, green, blue, white)`. `_template_update` can produce
/// a fifth component (`text.split(',', 4)` yields up to five fields), so the
/// tuple length is not fixed.
pub type LedColor = Vec<f64>;

/// `config.getfloat(option, default, minval=0., maxval=1.)` of the initial color.
fn color_opts(default: f64) -> GetFloatOpts {
    GetFloatOpts {
        default: Some(Some(default)),
        minval: Some(0.),
        maxval: Some(1.),
        ..Default::default()
    }
}

/// `gcmd.get_float(name, 0., minval=0., maxval=1.)` of a `SET_LED` component.
fn gcmd_color_opts(default: f64) -> GcmdGetFloatOpts {
    GcmdGetFloatOpts {
        default: Some(Some(default)),
        minval: Some(0.),
        maxval: Some(1.),
        ..Default::default()
    }
}

/// Python `max(a, b)`: returns `b` only when `b > a`, so a `NaN` first
/// argument stays `NaN` (`f64::max` would return the other argument).
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python `min(a, b)`: returns `b` only when `b < a`.
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `LEDHelper` (`led.py:9`): shared LED color state and the commands that
/// update it.
pub struct LEDHelper {
    printer: Printer,
    mutex: Arc<ReactorMutex>,
    update_func: LedUpdateFunc,
    led_count: usize,
    need_transmit: GCell<bool>,
    led_state: GCell<Vec<LedColor>>,
    template_eval: Arc<PrinterTemplateEvaluator>,
    tcallbacks: Vec<TemplateCallback>,
    /// The bound method `self._check_transmit`, shared by every command so
    /// the evaluator sees one flush callback per helper like in Python.
    flush_callback: FlushCallback,
    me: Weak<LEDHelper>,
}

impl LEDHelper {
    /// `LEDHelper(config, update_func, led_count=1)` (`led.py:10`).
    pub fn new(
        config: &Config,
        update_func: LedUpdateFunc,
        led_count: usize,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let mutex = Arc::new(printer.get_reactor().mutex(false));
        let red = config.getfloat("initial_RED", color_opts(0.))?.unwrap_or(0.);
        let green = config.getfloat("initial_GREEN", color_opts(0.))?.unwrap_or(0.);
        let blue = config.getfloat("initial_BLUE", color_opts(0.))?.unwrap_or(0.);
        let white = config.getfloat("initial_WHITE", color_opts(0.))?.unwrap_or(0.);
        let led_state = GCell::new(vec![vec![red, green, blue, white]; led_count]);
        let need_transmit = GCell::new(false);
        // `self.template_eval = output_pin.lookup_template_eval(config)`
        let template_eval = lookup_template_eval(config)?;
        let printer_for_helper = printer.clone();
        let helper = Arc::new_cyclic(move |me: &Weak<LEDHelper>| {
            // `self.tcallbacks = [(lambda text, s=self, index=i+1: ...)]`
            let tcallbacks: Vec<TemplateCallback> = (0..led_count)
                .map(|i| {
                    let weak = me.clone();
                    let index = i + 1;
                    Arc::new(move |text: &str| {
                        if let Some(helper) = weak.upgrade() {
                            helper.template_update(index, text);
                        }
                    }) as TemplateCallback
                })
                .collect();
            let flush_weak = me.clone();
            let flush_callback: FlushCallback = Arc::new(move || {
                if let Some(helper) = flush_weak.upgrade() {
                    helper.check_transmit(None);
                }
            });
            LEDHelper {
                printer: printer_for_helper,
                mutex,
                update_func,
                led_count,
                need_transmit,
                led_state,
                template_eval,
                tcallbacks,
                flush_callback,
                me: me.clone(),
            }
        });
        // Register commands (`name = config.get_name().split()[-1]`).
        let section = config.get_name();
        let name = section.split_whitespace().last().unwrap_or("").to_string();
        let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        // The registered handlers are bound methods in Python and keep the
        // helper alive on their own.
        let set_led = {
            let helper = helper.clone();
            Box::new(move |gcmd: &GCmd| helper.cmd_set_led(gcmd)) as GCodeCommandHandler
        };
        gcode.register_mux_command("SET_LED", "LED", Some(&name), set_led, Some(CMD_SET_LED_HELP))?;
        let set_led_template = {
            let helper = helper.clone();
            Box::new(move |gcmd: &GCmd| helper.cmd_set_led_template(gcmd)) as GCodeCommandHandler
        };
        gcode.register_mux_command(
            "SET_LED_TEMPLATE",
            "LED",
            Some(&name),
            set_led_template,
            Some(CMD_SET_LED_TEMPLATE_HELP),
        )?;
        Ok(helper)
    }

    /// `led_count` of this helper.
    pub fn led_count(&self) -> usize {
        self.led_count
    }

    /// The color list (`self.led_state`), copied out.
    pub fn color_data(&self) -> Vec<LedColor> {
        self.led_state.cloned()
    }

    /// `get_status(eventtime=None)` (`led.py:39`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let state: Vec<PyValue> = self
            .led_state
            .borrow()
            .iter()
            .map(|c| PyValue::Tuple(c.iter().map(|v| PyValue::Float(*v)).collect()))
            .collect();
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("color_data".to_string()), PyValue::List(state));
        PyValue::Dict(map)
    }

    fn me(&self) -> Arc<LEDHelper> {
        self.me.upgrade().expect("LEDHelper dropped while in use")
    }

    /// `_set_color(index, color)` (`led.py:41`).
    fn set_color(&self, index: Option<usize>, color: LedColor) {
        let new_led_state: Vec<LedColor> = match index {
            None => {
                let new_led_state = vec![color; self.led_count];
                if *self.led_state.borrow() == new_led_state {
                    return;
                }
                new_led_state
            }
            Some(index) => {
                if self.led_state.borrow()[index - 1] == color {
                    return;
                }
                let mut new_led_state = self.led_state.cloned();
                new_led_state[index - 1] = color;
                new_led_state
            }
        };
        self.led_state.replace(new_led_state);
        self.need_transmit.set(true);
    }

    /// `_template_update(index, text)` (`led.py:52`).
    pub fn template_update(&self, index: usize, text: &str) {
        let mut parts: Vec<f64> = Vec::new();
        let mut failed = false;
        for field in text.splitn(5, ',') {
            match pycompat::text::float_from_string(field) {
                Ok(value) => parts.push(py_max(0., py_min(1., value))),
                Err(e) => {
                    klippy_api::log::exception("led template render error", &e.display());
                    failed = true;
                    break;
                }
            }
        }
        if failed {
            parts = Vec::new();
        }
        if parts.len() < 4 {
            parts.resize(4, 0.);
        }
        self.set_color(Some(index), parts);
    }

    /// `_check_transmit(print_time=None)` (`led.py:60`).
    pub fn check_transmit(&self, print_time: Option<f64>) {
        if !self.need_transmit.get() {
            return;
        }
        // Just avoid any race conditions
        let led_state = self.led_state.cloned();
        self.need_transmit.set(false);
        let update_func = self.update_func.clone();
        let mutex = self.mutex.clone();
        let reactor = self.printer.get_reactor();
        let waketime = reactor::NOW;
        reactor.register_callback(
            move |_eventtime: f64| -> KResult<()> {
                mutex.lock();
                let result = update_func(&led_state, print_time);
                mutex.unlock();
                match result {
                    // `except self.printer.command_error`: logged, then swallowed.
                    Err(e @ KlippyError::Command(_)) => {
                        klippy_api::log::exception(
                            "led update transmit error",
                            &e.exception_line(),
                        );
                        Ok(())
                    }
                    // Any other exception escapes the reactor callback.
                    other => other,
                }
            },
            waketime,
        );
    }

    /// `cmd_SET_LED(gcmd)` (`led.py:93`).
    pub fn cmd_set_led(&self, gcmd: &GCmd) -> KResult<()> {
        // Parse parameters
        let red = gcmd.get_float("RED", gcmd_color_opts(0.))?.unwrap_or(0.);
        let green = gcmd.get_float("GREEN", gcmd_color_opts(0.))?.unwrap_or(0.);
        let blue = gcmd.get_float("BLUE", gcmd_color_opts(0.))?.unwrap_or(0.);
        let white = gcmd.get_float("WHITE", gcmd_color_opts(0.))?.unwrap_or(0.);
        let index = gcmd.get_int(
            "INDEX",
            GcmdGetIntOpts {
                default: Some(None),
                minval: Some(1),
                maxval: Some(self.led_count as i64),
            },
        )?;
        let transmit = gcmd.get_int(
            "TRANSMIT",
            GcmdGetIntOpts { default: Some(Some(1)), ..Default::default() },
        )?
        .unwrap_or(1);
        let sync = gcmd.get_int(
            "SYNC",
            GcmdGetIntOpts { default: Some(Some(1)), ..Default::default() },
        )?
        .unwrap_or(1);
        let color = vec![red, green, blue, white];
        let index = index.map(|i| i as usize);
        // Update and transmit data
        if sync != 0 {
            // Sync LED Update with print time and send
            let toolhead = klippy_api::lookup_interface::<dyn ToolHeadApi>(
                &*self.printer,
                "toolhead",
                LookupDefault::Raise,
            )?
            .expect("LookupDefault::Raise never yields None");
            let helper = self.me();
            let lookahead: LookaheadCallback = Box::new(move |print_time: f64| {
                helper.set_color(index, color.clone());
                if transmit != 0 {
                    helper.check_transmit(Some(print_time));
                }
                Ok(())
            });
            toolhead.register_lookahead_callback(lookahead)
        } else {
            // Send update now (so as not to wake toolhead and reset idle_timeout)
            self.set_color(index, color);
            if transmit != 0 {
                self.check_transmit(None);
            }
            Ok(())
        }
    }

    /// `cmd_SET_LED_TEMPLATE(gcmd)` (`led.py:114`).
    pub fn cmd_set_led_template(&self, gcmd: &GCmd) -> KResult<()> {
        let index = gcmd.get_int(
            "INDEX",
            GcmdGetIntOpts {
                default: Some(None),
                minval: Some(1),
                maxval: Some(self.led_count as i64),
            },
        )?;
        let flush_callback = self.flush_callback.clone();
        match index {
            Some(index) => self.template_eval.set_template(
                gcmd,
                self.tcallbacks[index as usize - 1].clone(),
                Some(flush_callback),
            ),
            None => {
                for callback in &self.tcallbacks {
                    self.template_eval.set_template(
                        gcmd,
                        callback.clone(),
                        Some(flush_callback.clone()),
                    )?;
                }
                Ok(())
            }
        }
    }
}

/// `PrinterPWMLED` (`led.py:127`): one `[led <name>]` section driving up to
/// four MCU PWM pins.
pub struct PrinterPWMLED {
    printer: Printer,
    pins: Vec<(usize, PwmPin)>,
    last_print_time: GCell<f64>,
    led_helper: GCell<Option<Arc<LEDHelper>>>,
    prev_color: GCell<LedColor>,
}

impl PrinterPWMLED {
    /// `PrinterPWMLED(config)` (`led.py:128`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        // Configure pwm pins
        let ppins = klippy_api::lookup_interface::<dyn PinsApi>(
            &*printer,
            "pins",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let mcu = klippy_api::lookup_interface::<dyn McuApi>(
            &*printer,
            "mcu",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let max_duration = mcu.max_nominal_duration();
        let cycle_time = config
            .getfloat(
                "cycle_time",
                GetFloatOpts {
                    default: Some(Some(0.010)),
                    above: Some(0.),
                    maxval: Some(max_duration),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.010);
        let hardware_pwm = config
            .getboolean(
                "hardware_pwm",
                GetBoolOpts { default: Some(Some(false)), ..Default::default() },
            )?
            .unwrap_or(false);
        let mut pins: Vec<(usize, PwmPin)> = Vec::new();
        for (i, name) in ["red", "green", "blue", "white"].iter().enumerate() {
            let pin_name = config.get(
                &format!("{}_pin", name),
                GetStrOpts { default: Some(None), ..Default::default() },
            )?;
            let pin_name = match pin_name {
                Some(pin_name) => pin_name,
                None => continue,
            };
            let mcu_pin = match ppins.setup_pin("pwm", &pin_name)? {
                Pin::Pwm(mcu_pin) => mcu_pin,
                _ => {
                    return Err(KlippyError::Other {
                        class_name: "error".to_string(),
                        message: "setup_pin('pwm') returned unexpected pin type".to_string(),
                    });
                }
            };
            mcu_pin.setup_max_duration(0.);
            mcu_pin.setup_cycle_time(cycle_time, Some(hardware_pwm))?;
            pins.push((i, mcu_pin));
        }
        if pins.is_empty() {
            return Err(KlippyError::config(format!(
                "No LED pin definitions found in '{}'",
                config.get_name()
            )));
        }
        let object = Arc::new(PrinterPWMLED {
            printer: printer.clone(),
            pins,
            last_print_time: GCell::new(0.),
            led_helper: GCell::new(None),
            prev_color: GCell::new(vec![0.; 4]),
        });
        // Initialize color data
        // `self.update_leds` is a bound method: it keeps the object alive.
        let update_func: LedUpdateFunc = {
            let object = object.clone();
            Arc::new(move |led_state: &[LedColor], print_time: Option<f64>| {
                object.update_leds(led_state, print_time)
            })
        };
        let led_helper = LEDHelper::new(config, update_func, 1)?;
        let color = led_helper.color_data()[0].clone();
        for (idx, mcu_pin) in &object.pins {
            mcu_pin.setup_start_value(color[*idx], 0.);
        }
        object.prev_color.replace(color);
        object.led_helper.replace(Some(led_helper));
        Ok(object)
    }

    /// `update_leds(led_state, print_time)` (`led.py:148`).
    pub fn update_leds(&self, led_state: &[LedColor], print_time: Option<f64>) -> KResult<()> {
        let mcu = self.pins[0].1.get_mcu();
        let min_sched_time = mcu.min_schedule_time();
        let mut print_time = match print_time {
            Some(print_time) => print_time,
            None => {
                let eventtime = self.printer.get_reactor().monotonic();
                mcu.estimated_print_time(eventtime) + min_sched_time
            }
        };
        print_time = py_max(print_time, self.last_print_time.get() + min_sched_time);
        let color = led_state[0].clone();
        let prev_color = self.prev_color.cloned();
        for (idx, mcu_pin) in &self.pins {
            if prev_color[*idx] != color[*idx] {
                mcu_pin.set_pwm(print_time, color[*idx])?;
                self.last_print_time.set(print_time);
            }
        }
        self.prev_color.replace(color);
        Ok(())
    }

    /// The `LEDHelper` of this object.
    pub fn led_helper(&self) -> Arc<LEDHelper> {
        self.led_helper
            .borrow()
            .clone()
            .expect("led_helper is set before construction returns")
    }

    /// `get_status(eventtime=None)` (`led.py:164`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.led_helper().get_status(eventtime)
    }
}

impl PrinterObject for PrinterPWMLED {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(PrinterPWMLED::get_status(self, eventtime)))
    }
}

fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterPWMLED::new(config)? as ObjectRef))
}
/// Module entry point for `[led <name>]` sections (`led.py:167`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
