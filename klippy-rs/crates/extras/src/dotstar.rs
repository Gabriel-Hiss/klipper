//! Port of the `dotstar` module (`klippy/extras/dotstar.py`).
//!
//! A `[dotstar NAME]` section drives a chain of Dotstar LED chips over a
//! software SPI bus: `data_pin` / `clock_pin` on the same MCU, mode 0,
//! 500 kHz, with `sw_pins = (data, data, clock)`. LED state comes from
//! `led.LEDHelper` ([`crate::led`], which owns the `SET_LED` /
//! `SET_LED_TEMPLATE` commands and the `{color_data: [...]}` status
//! dict); the SPI stream comes from `bus.McuSpi` ([`crate::bus`]).
//!
//! Frame format (`dotstar.py:37-44`): four zero bytes, then one
//! 4-byte group per LED `0xff, B, G, R` with `int(x*255. + .5)`, then
//! four `0xff` bytes. The frame is sent in 20-byte chunks through
//! `spi_send`, with `minclock` derived from `print_time` when one is
//! given and `reqclock` fixed at [`BACKGROUND_PRIORITY_CLOCK`].
//!
//! Construction order follows the reference line by line: look up the
//! `pins` object through the [`PinsApi`] interface, look up both pins,
//! require the same MCU, build the software SPI bus, read `chain_count`
//! (default 1, minimum 1), and build the [`LEDHelper`] with this
//! object's `update_leds` as the update function, ending with the
//! `klippy:connect` registration (`dotstar.py:29`).

use std::sync::Arc;

use klippy_api::config::{Config, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::lookup_interface;
use klippy_api::mcu::{LookupPinOpts, Mcu, PinsApi, SendOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, EV_KLIPPY_CONNECT};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use pycompat::text::trunc_float_to_bigint;
use pycompat::PyValue;

use crate::bus::{McuSpi, McuSpiApi};
use crate::led::{LEDHelper, LedUpdateFunc};

/// `BACKGROUND_PRIORITY_CLOCK` (`dotstar.py:8`), the fixed `reqclock`
/// for every frame chunk: background priority.
pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fff_ffff_0000_0000;

/// One LED color: `(red, green, blue, white)` floats (`dotstar.py:38`).
/// The shared helper stores colors as `Vec<f64>`; this tuple is the
/// shape `update_leds` unpacks in Python.
pub type LedColor = (f64, f64, f64, f64);

/// SPI speed, `500000` (`dotstar.py:22`).
const SPI_SPEED: i64 = 500_000;

/// SPI mode, `0` (`dotstar.py:22`).
const SPI_MODE: i64 = 0;

/// Frame chunk size, `20` (`dotstar.py:49`).
const CHUNK_SIZE: usize = 20;

/// Mutable state shared with the update callback: the SPI stream plus
/// `prev_data` (`dotstar.py:27`). Split out because the `LEDHelper`
/// update function is registered before the printer object exists.
struct Core {
    spi: Arc<McuSpi>,
    prev_data: GCell<Option<Vec<crate::led::LedColor>>>,
}

/// `PrinterDotstar` (`dotstar.py:10`).
pub struct PrinterDotstar {
    /// `chain_count` (`dotstar.py:24`), the `LEDHelper` LED count.
    pub chain_count: i64,
    core: Arc<Core>,
    led_helper: Arc<LEDHelper>,
}

/// `int(float)`: truncate toward zero per CPython `int()` semantics.
///
/// Non-finite values raise the CPython exceptions verbatim:
/// `OverflowError: cannot convert float infinity to integer` and
/// `ValueError: cannot convert float NaN to integer`
/// (`Objects/abstract.c:PyNumber_Long`). Finite values are truncated
/// exactly, clamping at the `i64` extremes the way the `clocksync`
/// port does; LED colors are clamped to 0..1 by the helper so the
/// result is in 0..=255 on every reachable path.
fn py_int(f: f64) -> KResult<i64> {
    if f.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if f.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    let digits = trunc_float_to_bigint(f).0.to_string();
    Ok(digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX }))
}

/// Unpack one helper color `Vec<f64>` into a 4-tuple the way Python
/// `for red, green, blue, white in led_state` does (`dotstar.py:38`).
fn unpack_color(color: &[f64]) -> KResult<LedColor> {
    if color.len() > 4 {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "too many values to unpack (expected 4)".to_string(),
        });
    }
    if color.len() < 4 {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: format!("not enough values to unpack (expected 4, got {})", color.len()),
        });
    }
    Ok((color[0], color[1], color[2], color[3]))
}

/// Convert helper colors (`Vec<f64>` entries) to the unpacked shape.
fn unpack_state(state: &[crate::led::LedColor]) -> KResult<Vec<LedColor>> {
    state.iter().map(|c| unpack_color(c)).collect()
}

/// Transmit one frame (`update_leds`, `dotstar.py:32-51`). Stores the raw
/// helper colors first, then unpacks, mirroring the order of
/// `if led_state == self.prev_data: return; self.prev_data = led_state`
/// followed by the unpack loop at `dotstar.py:33-38`.
fn transmit(core: &Core, led_state: &[crate::led::LedColor], print_time: Option<f64>) -> KResult<()> {
    {
        let prev = core.prev_data.borrow();
        if prev.as_deref() == Some(led_state) {
            return Ok(());
        }
    }
    core.prev_data.replace(Some(led_state.to_vec()));
    let led_state = unpack_state(led_state)?;
    // Build data to send: 4 zero bytes, per LED 0xff,B,G,R (white is
    // unpacked but unused), then 4 bytes of 0xff. The bus takes `&[u8]`,
    // so each int is truncated with `as u8` (modulo 256), matching the
    // byte domain; helper-clamped colors stay in 0..=255 anyway.
    let mut data = vec![0u8; (led_state.len() + 2) * 4];
    for (i, (red, green, blue, _white)) in led_state.iter().enumerate() {
        let idx = (i + 1) * 4;
        data[idx] = 0xff;
        data[idx + 1] = py_int(blue * 255.0 + 0.5)? as u8;
        data[idx + 2] = py_int(green * 255.0 + 0.5)? as u8;
        data[idx + 3] = py_int(red * 255.0 + 0.5)? as u8;
    }
    let len = data.len();
    data[len - 4] = 0xff;
    data[len - 3] = 0xff;
    data[len - 2] = 0xff;
    data[len - 1] = 0xff;
    let minclock = match print_time {
        None => 0,
        Some(print_time) => core.spi.get_mcu().print_time_to_clock(print_time),
    };
    let opts = SendOpts { minclock, reqclock: BACKGROUND_PRIORITY_CLOCK };
    for chunk in data.chunks(CHUNK_SIZE) {
        core.spi.spi_send(chunk, opts)?;
    }
    Ok(())
}

impl PrinterDotstar {
    /// `PrinterDotstar.__init__` (`dotstar.py:11-29`), including the
    /// `klippy:connect` registration at `dotstar.py:29`.
    pub fn new(config: &Config) -> KResult<Arc<PrinterDotstar>> {

        let printer = config.get_printer();
        // Configure a software spi bus.
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let data_pin = config
            .get("data_pin", GetStrOpts::default())?
            .expect("required option yields a value");
        let data_pin_params = ppins.lookup_pin(&data_pin, LookupPinOpts::default())?;
        let clock_pin = config
            .get("clock_pin", GetStrOpts::default())?
            .expect("required option yields a value");
        let clock_pin_params = ppins.lookup_pin(&clock_pin, LookupPinOpts::default())?;
        let mcu: Mcu = data_pin_params.chip.clone();
        if !Arc::ptr_eq(&data_pin_params.chip, &clock_pin_params.chip) {
            return Err(KlippyError::Config(
                "Dotstar pins must be on same mcu".to_string(),
            ));
        }
        let sw_pins = [
            data_pin_params.pin.clone(),
            data_pin_params.pin.clone(),
            clock_pin_params.pin.clone(),
        ];
        let spi = McuSpi::new(mcu, None, None, SPI_MODE, SPI_SPEED, Some(sw_pins), false)?;
        // Initialize color data.
        let chain_count = config
            .getint("chain_count", GetIntOpts {
                default: Some(Some(1)),
                minval: Some(1),
                maxval: None,
                note_valid: true,
            })?
            .expect("defaulted option yields a value");
        let core = Arc::new(Core { spi, prev_data: GCell::new(None) });
        let led_core = Arc::clone(&core);
        let update_func: LedUpdateFunc = Arc::new(move |state: &[crate::led::LedColor], print_time: Option<f64>| {
            transmit(&led_core, state, print_time)
        });
        let led_helper = LEDHelper::new(config, update_func, chain_count as usize)?;
        let obj = Arc::new(PrinterDotstar { chain_count, core, led_helper });
        let obj2 = Arc::clone(&obj);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params: &[EventArg]| {
                obj2.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        Ok(obj)
    }

    /// `handle_connect` (`dotstar.py:30-31`).
    pub fn handle_connect(&self) -> KResult<()> {
        let colors = self.led_helper.color_data();
        transmit(&self.core, &colors, None)
    }

    /// `update_leds` (`dotstar.py:32-51`): transmit `led_state` unless
    /// it is equal to the previously transmitted state.
    pub fn update_leds(&self, led_state: &[LedColor], print_time: Option<f64>) -> KResult<()> {
        let colors: Vec<crate::led::LedColor> = led_state
            .iter()
            .map(|(red, green, blue, white)| vec![*red, *green, *blue, *white])
            .collect();
        transmit(&self.core, &colors, print_time)
    }
}

impl PrinterObject for PrinterDotstar {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    /// `get_status` (`dotstar.py:52-53`): delegate to the LED helper.
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.led_helper.get_status(eventtime))
    }
}

/// `load_config_prefix` (`dotstar.py:55-56`): build the object.
/// Registration happens in [`PrinterDotstar::new`], like `__init__`.
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterDotstar::new(config)? as ObjectRef))
}

/// Module factory for the `[dotstar NAME]` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
