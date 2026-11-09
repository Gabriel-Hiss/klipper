//! Port of the `neopixel` module (`klippy/extras/neopixel.py`).
//!
//! Drives a NeoPixel LED chain over the MCU `neopixel` commands. The per-LED
//! colors and G-code interaction come from [`crate::led::LEDHelper`]
//! (`SET_LED` / `SET_LED_TEMPLATE`); this module owns the pin setup, the
//! color-order byte mapping, and the diff-batched MCU transmission.

use crate::led::{LedColor, LEDHelper};
use klippy_api::config::{Config, GetIntOpts, GetListOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueryWrapper, CommandWrapper, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, PinsApi, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{EV_KLIPPY_CONNECT, LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue, PyError};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// `BACKGROUND_PRIORITY_CLOCK` (`klippy/extras/neopixel.py:9`).
pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fffffff00000000;
/// `BIT_MAX_TIME` (`klippy/extras/neopixel.py:11`).
pub const BIT_MAX_TIME: f64 = 0.000004;
/// `RESET_MIN_TIME` (`klippy/extras/neopixel.py:12`).
pub const RESET_MIN_TIME: f64 = 0.000050;
/// `MAX_MCU_SIZE` (`klippy/extras/neopixel.py:14`).
pub const MAX_MCU_SIZE: usize = 500;

fn py_int_error(f: f64) -> Option<KlippyError> {
    if f.is_nan() {
        Some(KlippyError::Other {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        })
    } else if f.is_infinite() {
        Some(KlippyError::Other {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        })
    } else {
        None
    }
}

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

/// `int(v)` for the `bytearray` item assignment in `update_color_data`:
/// truncation toward zero, then the `byte must be in range(0, 256)` check.
fn byte_from_float(v: f64) -> KResult<u8> {
    if let Some(err) = py_int_error(v) {
        return Err(err);
    }
    let n = trunc_float_to_bigint(v);
    let text = n.inner().to_string();
    match text.parse::<i64>() {
        Ok(x) if (0..=255).contains(&x) => Ok(x as u8),
        _ => Err(KlippyError::Other {
            class_name: "ValueError".to_string(),
            message: "byte must be in range(0, 256)".to_string(),
        }),
    }
}

/// Extract `led_helper.get_status()['color_data']` as typed RGBW tuples.
fn color_data_from_status(status: &PyValue) -> KResult<Vec<[f64; 4]>> {
    let PyValue::Dict(map) = status else {
        return Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "'dict' expected".to_string(),
        });
    };
    let Some(v) = map.get(&PyValue::Str("color_data".to_string())) else {
        return Err(KlippyError::Other {
            class_name: "KeyError".to_string(),
            message: "'color_data'".to_string(),
        });
    };
    let PyValue::List(items) = v else {
        return Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "'list' expected".to_string(),
        });
    };
    let mut out = Vec::with_capacity(items.len());
    for item in items {
        let parts: &[PyValue] = match item {
            PyValue::Tuple(parts) | PyValue::List(parts) => parts,
            _ => {
                return Err(KlippyError::Other {
                    class_name: "TypeError".to_string(),
                    message: "'tuple' expected".to_string(),
                });
            }
        };
        if parts.len() != 4 {
            return Err(KlippyError::Other {
                class_name: "TypeError".to_string(),
                message: format!("expected 4 components, got {}", parts.len()),
            });
        }
        let mut rgba = [0_f64; 4];
        for (i, part) in parts.iter().enumerate() {
            rgba[i] = match part {
                PyValue::Float(f) => *f,
                PyValue::Int(n) => n
                    .to_f64_exact()
                    .ok_or_else(|| KlippyError::Other {
                        class_name: "OverflowError".to_string(),
                        message: "int too large to convert to float".to_string(),
                    })?,
                PyValue::Bool(b) => {
                    if *b {
                        1.0
                    } else {
                        0.0
                    }
                }
                _ => {
                    return Err(KlippyError::Other {
                        class_name: "TypeError".to_string(),
                        message: "expected float".to_string(),
                    });
                }
            };
        }
        out.push(rgba);
    }
    Ok(out)
}

/// `printer.get_start_args().get('debugoutput') is not None`
/// (`klippy/extras/neopixel.py:91`).
fn debugoutput_active(printer: &Printer) -> bool {
    let args = printer.get_start_args();
    let PyValue::Dict(map) = args else {
        return false;
    };
    match map.get(&PyValue::Str("debugoutput".to_string())) {
        Some(PyValue::None) | None => false,
        Some(_) => true,
    }
}

fn none_send() -> KlippyError {
    KlippyError::Other {
        class_name: "AttributeError".to_string(),
        message: "'NoneType' object has no attribute 'send'".to_string(),
    }
}

/// Mutable framebuffer and command slots shared with the MCU config
/// callback, the `klippy:connect` handler and the `LEDHelper` callback
/// (all reach the same Python instance through `self`).
struct InnerData {
    color_data: Vec<u8>,
    old_color_data: Vec<u8>,
    update_cmd: Option<CommandWrapper>,
    send_cmd: Option<CommandQueryWrapper>,
}

struct NeopixelInner {
    printer: Printer,
    mcu: Mcu,
    oid: i64,
    pin: String,
    /// `(lidx, cidx)` per `cdidx` (`color_map` in `neopixel.py:39`).
    color_map: Vec<(usize, usize)>,
    data: GCell<InnerData>,
}

impl NeopixelInner {
    /// `PrinterNeoPixel.build_config` (`klippy/extras/neopixel.py:49`).
    fn do_build_config(&self) -> KResult<()> {
        let bit_max = self.mcu.seconds_to_clock(BIT_MAX_TIME);
        let reset_min = self.mcu.seconds_to_clock(RESET_MIN_TIME);
        let data_size = self.data.borrow().color_data.len();
        let cmd = percent_format(
            "config_neopixel oid=%d pin=%s data_size=%d bit_max_ticks=%d reset_min_ticks=%d",
            &[
                PctValue::Int(PyInt::from_i64(self.oid)),
                PctValue::Str(self.pin.clone()),
                PctValue::Int(PyInt::from_i64(data_size as i64)),
                PctValue::Int(PyInt::from_i64(bit_max)),
                PctValue::Int(PyInt::from_i64(reset_min)),
            ],
        )
        .map_err(map_py_error)?;
        self.mcu.add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        let queue = self.mcu.alloc_command_queue();
        let update_cmd = self
            .mcu
            .lookup_command("neopixel_update oid=%c pos=%hu data=%*s", Some(queue))?;
        let send_cmd = self.mcu.lookup_query_command(
            "neopixel_send oid=%c",
            "neopixel_result oid=%c success=%c",
            LookupQueryCommandOpts {
                oid: Some(self.oid),
                cq: Some(queue),
                is_async: false,
            },
        )?;
        let mut data = self.data.borrow_mut();
        data.update_cmd = Some(update_cmd);
        data.send_cmd = Some(send_cmd);
        Ok(())
    }

    fn do_update_color_data(&self, led_state: &[LedColor]) -> KResult<()> {
        let mut data = self.data.borrow_mut();
        for (cdidx, (lidx, cidx)) in self.color_map.iter().enumerate() {
            let Some(rgba) = led_state.get(*lidx) else {
                return Err(KlippyError::Other {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                });
            };
            let Some(&v) = rgba.get(*cidx) else {
                return Err(KlippyError::Other {
                    class_name: "IndexError".to_string(),
                    message: "tuple index out of range".to_string(),
                });
            };
            data.color_data[cdidx] = byte_from_float(v * 255. + 0.5)?;
        }
        Ok(())
    }

    /// `PrinterNeoPixel.send_data` (`klippy/extras/neopixel.py:66`).
    fn do_send_data(&self, print_time: Option<f64>) -> KResult<()> {
        let (old_data, new_data) = {
            let data = self.data.borrow();
            (data.old_color_data.clone(), data.color_data.clone())
        };
        if new_data == old_data {
            return Ok(());
        }
        let mut diffs: Vec<(usize, usize)> = Vec::new();
        for (i, (n, o)) in new_data.iter().zip(old_data.iter()).enumerate() {
            if n != o {
                diffs.push((i, 1));
            }
        }
        // Batch together changes that are close to each other, backwards.
        if diffs.len() >= 2 {
            let mut i = diffs.len() - 2;
            loop {
                let (pos, _count) = diffs[i];
                let (nextpos, nextcount) = diffs[i + 1];
                if pos + 5 >= nextpos && nextcount < 16 {
                    diffs[i].1 = nextcount + (nextpos - pos);
                    diffs.remove(i + 1);
                }
                if i == 0 {
                    break;
                }
                i -= 1;
            }
        }
        let update_cmd = self.data.borrow().update_cmd.clone().ok_or_else(none_send)?;
        for (pos, count) in &diffs {
            update_cmd.send(
                vec![
                    PyValue::Int(PyInt::from_i64(self.oid)),
                    PyValue::Int(PyInt::from_i64(*pos as i64)),
                    PyValue::Bytes(new_data[*pos..*pos + *count].to_vec()),
                ],
                SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK },
            )?;
        }
        self.data.borrow_mut().old_color_data.copy_from_slice(&new_data);
        let minclock = match print_time {
            Some(pt) => self.mcu.print_time_to_clock(pt),
            None => 0,
        };
        let send_cmd = self.data.borrow().send_cmd.clone().ok_or_else(none_send)?;
        if debugoutput_active(&self.printer) {
            return Ok(());
        }
        let mut succeeded = false;
        for _ in 0..8 {
            let params = send_cmd.send(
                vec![PyValue::Int(PyInt::from_i64(self.oid))],
                QuerySendOpts {
                    minclock,
                    reqclock: BACKGROUND_PRIORITY_CLOCK,
                    retry: true,
                },
            )?;
            let success = match &params {
                PyValue::Dict(map) => match map.get(&PyValue::Str("success".to_string())) {
                    Some(v) => v.truthy(),
                    None => {
                        return Err(KlippyError::Other {
                            class_name: "KeyError".to_string(),
                            message: "'success'".to_string(),
                        });
                    }
                },
                _ => {
                    return Err(KlippyError::Other {
                        class_name: "TypeError".to_string(),
                        message: "'dict' expected".to_string(),
                    });
                }
            };
            if success {
                succeeded = true;
                break;
            }
        }
        if !succeeded {
            log::info("Neopixel update did not succeed");
        }
        Ok(())
    }

    fn do_update_leds(&self, led_state: &[LedColor], print_time: Option<f64>) -> KResult<()> {
        self.do_update_color_data(led_state)?;
        self.do_send_data(print_time)
    }
}

/// Port of the `PrinterNeoPixel` class (`klippy/extras/neopixel.py:16`).
pub struct PrinterNeoPixel {
    inner: Arc<NeopixelInner>,
    led_helper: Arc<LEDHelper>,
}

impl PrinterNeoPixel {
    /// `PrinterNeoPixel.__init__(config)` (`klippy/extras/neopixel.py:17`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let Some(pin_desc) = config.get("pin", GetStrOpts::default())? else {
            return Err(KlippyError::config(format!(
                "Option 'pin' in section '{}' must be specified",
                config.get_name()
            )));
        };
        let pin_params = pins.lookup_pin(&pin_desc, LookupPinOpts::default())?;
        let mcu = pin_params.chip.clone();
        let oid = mcu.create_oid()?;
        let pin = pin_params.pin.clone();
        let chain_count = config
            .getint(
                "chain_count",
                GetIntOpts { default: Some(Some(1)), minval: Some(1), ..Default::default() },
            )?
            .unwrap_or(1);
        let mut color_order = config
            .getlist(
                "color_order",
                GetListOpts {
                    default: Some(Some(vec!["GRB".to_string()])),
                    ..Default::default()
                },
            )?
            .unwrap_or_else(|| vec!["GRB".to_string()]);
        if color_order.len() == 1 {
            color_order = vec![color_order[0].clone(); chain_count as usize];
        }
        if color_order.len() as i64 != chain_count {
            return Err(KlippyError::config("color_order does not match chain_count"));
        }
        let mut color_map: Vec<(usize, usize)> = Vec::new();
        for (lidx, co) in color_order.iter().enumerate() {
            let mut chars: Vec<char> = co.chars().collect();
            chars.sort();
            if chars != ['B', 'G', 'R'] && chars != ['B', 'G', 'R', 'W'] {
                return Err(KlippyError::config(
                    percent_format("Invalid color_order '%s'", &[PctValue::Str(co.clone())])
                        .map_err(map_py_error)?,
                ));
            }
            for c in co.chars() {
                let cidx = "RGBW".chars().position(|x| x == c).unwrap_or(0);
                color_map.push((lidx, cidx));
            }
        }
        if color_map.len() > MAX_MCU_SIZE {
            return Err(KlippyError::config("neopixel chain too long"));
        }
        let data_len = color_map.len();
        let inner = Arc::new(NeopixelInner {
            printer: printer.clone(),
            mcu: mcu.clone(),
            oid,
            pin,
            color_map,
            data: GCell::new(InnerData {
                color_data: vec![0u8; data_len],
                old_color_data: vec![0u8; data_len],
                update_cmd: None,
                send_cmd: None,
            }),
        });
        {
            let cfg_inner = inner.clone();
            inner
                .mcu
                .register_config_callback(Box::new(move || cfg_inner.do_build_config()))?;
        }
        // `led.LEDHelper(config, self.update_leds, chain_count)`.
        let led_count = chain_count as usize;
        let cb_inner = inner.clone();
        let update_func: crate::led::LedUpdateFunc = Arc::new(move |state: &[LedColor], pt: Option<f64>| {
            cb_inner.do_update_leds(state, pt)
        });
        let led_helper = LEDHelper::new(config, update_func, led_count)?;
        {
            let status = led_helper.get_status(0.0);
            let led_state = color_data_from_status(&status)?;
            let led_state: Vec<LedColor> = led_state.into_iter().map(|c| c.to_vec()).collect();
            inner.do_update_color_data(&led_state)?;
            let mut data = inner.data.borrow_mut();
            data.old_color_data = data.color_data.iter().map(|d| d ^ 1).collect();
        }
        {
            let ev_inner = inner.clone();
            inner.printer.register_event_handler(
                EV_KLIPPY_CONNECT,
                Box::new(move |_args: &[EventArg]| -> KResult<PyValue> {
                    ev_inner.do_send_data(None)?;
                    Ok(PyValue::None)
                }),
            );
        }
        Ok(PrinterNeoPixel { inner, led_helper })
    }

    pub fn build_config(&self) -> KResult<()> {
        self.inner.do_build_config()
    }

    pub fn update_color_data(&self, led_state: &[[f64; 4]]) -> KResult<()> {
        let led_state: Vec<LedColor> =
            led_state.iter().map(|c| c.to_vec()).collect();
        self.inner.do_update_color_data(&led_state)
    }

    pub fn send_data(&self, print_time: Option<f64>) -> KResult<()> {
        self.inner.do_send_data(print_time)
    }

    pub fn update_leds(&self, led_state: &[[f64; 4]], print_time: Option<f64>) -> KResult<()> {
        let led_state: Vec<LedColor> =
            led_state.iter().map(|c| c.to_vec()).collect();
        self.inner.do_update_leds(&led_state, print_time)
    }

    pub fn oid(&self) -> i64 {
        self.inner.oid
    }

    pub fn pin(&self) -> &str {
        &self.inner.pin
    }

    pub fn color_data(&self) -> Vec<u8> {
        self.inner.data.borrow().color_data.clone()
    }

    pub fn old_color_data(&self) -> Vec<u8> {
        self.inner.data.borrow().old_color_data.clone()
    }
}

impl PrinterObject for PrinterNeoPixel {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    /// `PrinterNeoPixel.get_status` (`klippy/extras/neopixel.py:103`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.led_helper.get_status(eventtime))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/neopixel.py:106`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterNeoPixel::new(config)?) as ObjectRef))
}

/// Module entry point for the `neopixel` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
