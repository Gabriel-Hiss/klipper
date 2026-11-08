//! Port of the `extras.display.st7920` module (`klippy/extras/display/st7920.py`).
//!
//! The Python module has no `load_config` / `load_config_prefix` /
//! `add_printer_objects`: `extras.display.display.PrinterLCD` builds the
//! chip objects directly from its `LCD_chips` table (`display.py:16`), so
//! [`module_factory`] is empty.
//!
//! The software-SPI helpers (`McuSpi`, `McuBusDigitalOut`) are owned by
//! `crate::bus` (`klippy/extras/bus.py`).

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::lookup_interface;
use klippy_api::mcu::{AddConfigCmdOpts, CommandWrapper, Mcu, Pins, PinsApi, SendOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};

use crate::bus::{McuBusDigitalOut, McuBusDigitalOutApi, McuSpi, McuSpiApi};

pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fff_ffff_0000_0000;
pub const ST7920_CMD_DELAY: f64 = 0.000020;
pub const ST7920_SYNC_DELAY: f64 = 0.000045;

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant st7920 format")
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}

fn lookup_pins(printer: &klippy_api::printer::Printer) -> KResult<Pins> {
    lookup_interface::<dyn PinsApi>(&**printer, "pins", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::config("Unknown config object 'pins'"))
}


// ---------------------------------------------------------------------------
// DisplayBase: framebuffers, glyphs, flush/init.
// ---------------------------------------------------------------------------

/// `CharGlyphs['degrees']` (`font8x14.VGA_FONT[0xf8]`).
const DEGREES_GLYPH: [u8; 16] = crate::display::font8x14::VGA_FONT[0xf8];

/// The text glyph for `TextGlyphs['right_arrow']`.
const RIGHT_ARROW_GLYPH: [u8; 1] = [0x1a];

struct DisplayBaseState {
    text_framebuffer: Vec<u8>,
    glyph_framebuffer: Vec<u8>,
    graphics_framebuffers: Vec<Vec<u8>>,
    text_old: Vec<u8>,
    glyph_old: Vec<u8>,
    graphics_old: Vec<Vec<u8>>,
    cached_glyphs: IndexMap<String, (String, (i64, i64))>,
    icons: IndexMap<String, (Vec<u8>, Vec<u8>)>,
}

impl DisplayBaseState {
    fn new() -> DisplayBaseState {
        DisplayBaseState {
            text_framebuffer: vec![b' '; 64],
            glyph_framebuffer: vec![0u8; 128],
            graphics_framebuffers: vec![vec![0u8; 32]; 32],
            text_old: vec![b'~'; 64],
            glyph_old: vec![b'~'; 128],
            graphics_old: vec![vec![b'~'; 32]; 32],
            cached_glyphs: IndexMap::new(),
            icons: IndexMap::new(),
        }
    }
}

/// Port of `DisplayBase` (`klippy/extras/display/st7920.py:19`).
pub struct DisplayBase {
    state: GCell<DisplayBaseState>,
}

/// Single-index lookup with Python negative-index wrap (`lst[-1]`).
fn py_index(idx: i64, len: usize) -> Option<usize> {
    let mut i = idx;
    if i < 0 {
        i += len as i64;
    }
    if i < 0 || i >= len as i64 {
        None
    } else {
        Some(i as usize)
    }
}

/// Python `bytearray[start:stop] = data` assignment (normalizes and resizes).
fn py_slice_assign(buf: &mut Vec<u8>, start: i64, stop: i64, data: &[u8]) {
    let len = buf.len() as i64;
    let mut start = start;
    let mut stop = stop;
    if start < 0 {
        start += len;
        if start < 0 {
            start = 0;
        }
    }
    if start > len {
        start = len;
    }
    if stop < 0 {
        stop += len;
        if stop < 0 {
            stop = 0;
        }
    }
    if stop > len {
        stop = len;
    }
    if stop < start {
        stop = start;
    }
    buf.splice(start as usize..stop as usize, data.iter().copied());
}

fn pyint_to_u8(n: &PyInt) -> Option<u8> {
    n.inner().to_string().parse::<u8>().ok()
}

fn icon_bytes(v: &PyValue) -> Option<Vec<u8>> {
    let conv = |items: &[PyValue]| -> Option<Vec<u8>> {
        items.iter().map(|e| pyint_to_u8(match e { PyValue::Int(p) => p, _ => return None })).collect()
    };
    match v {
        PyValue::List(items) => conv(items),
        PyValue::Tuple(items) => conv(items),
        PyValue::Bytes(bytes) => Some(bytes.clone()),
        _ => None,
    }
}

/// An `icon16x16` value: a pair of rows `(icon1, icon2)`.
fn parse_icon(v: &PyValue) -> Option<(Vec<u8>, Vec<u8>)> {
    match v {
        PyValue::List(items) if items.len() == 2 => Some((icon_bytes(&items[0])?, icon_bytes(&items[1])?)),
        PyValue::Tuple(items) if items.len() == 2 => Some((icon_bytes(&items[0])?, icon_bytes(&items[1])?)),
        _ => None,
    }
}

impl DisplayBase {
    pub fn new() -> DisplayBase {
        DisplayBase { state: GCell::new(DisplayBaseState::new()) }
    }

    /// `text_framebuffer` (`klippy/extras/display/st7920.py:22`).
    pub fn text_framebuffer(&self) -> Vec<u8> {
        self.state.borrow().text_framebuffer.clone()
    }

    /// `glyph_framebuffer` (`klippy/extras/display/st7920.py:23`).
    pub fn glyph_framebuffer(&self) -> Vec<u8> {
        self.state.borrow().glyph_framebuffer.clone()
    }

    /// `graphics_framebuffers` (`klippy/extras/display/st7920.py:24`).
    pub fn graphics_framebuffers(&self) -> Vec<Vec<u8>> {
        self.state.borrow().graphics_framebuffers.clone()
    }

    /// `cached_glyphs`: name -> (base name, (0, glyph_id * 2)).
    pub fn cached_glyphs(&self) -> IndexMap<String, (String, (i64, i64))> {
        self.state.borrow().cached_glyphs.clone()
    }

    /// `icons`: name -> (icon1, icon2).
    pub fn icons(&self) -> IndexMap<String, (Vec<u8>, Vec<u8>)> {
        self.state.borrow().icons.clone()
    }

    /// `flush()` (`klippy/extras/display/st7920.py:35`). `send` is
    /// `self.send(cmds, is_data, is_extended)` of the subclass.
    pub fn flush(&self, send: &mut dyn FnMut(&[u8], bool, bool) -> KResult<()>) -> KResult<()> {
        for fb_idx in 0..34 {
            let (new_data, old_data, fb_id) = self.fb_snapshot(fb_idx);
            if new_data == old_data {
                continue;
            }
            // Find the position of all changed bytes in this framebuffer.
            let mut diffs: Vec<(usize, usize)> = new_data
                .iter()
                .zip(old_data.iter())
                .enumerate()
                .filter(|(_, (n, o))| n != o)
                .map(|(i, _)| (i, 1))
                .collect();
            // Batch together changes that are close to each other.
            if diffs.len() >= 2 {
                for i in (0..diffs.len() - 1).rev() {
                    let (pos, _count) = diffs[i];
                    let (nextpos, nextcount) = diffs[i + 1];
                    if pos + 5 >= nextpos && nextcount < 16 {
                        diffs[i].1 = nextcount + (nextpos - pos);
                        diffs.remove(i + 1);
                    }
                }
            }
            // Transmit changes.
            for &(pos, mut count) in &diffs {
                count += pos & 0x01;
                count += count & 0x01;
                let pos = pos & !0x01;
                let chip_pos = pos >> 1;
                if fb_id < 0x40 {
                    // Graphics framebuffer update.
                    send(&[0x80 + fb_id, 0x80 + chip_pos as u8], false, true)?;
                } else {
                    send(&[fb_id + chip_pos as u8], false, false)?;
                }
                let end = (pos + count).min(new_data.len());
                send(&new_data[pos..end], true, false)?;
            }
            self.fb_copy(fb_idx);
        }
        Ok(())
    }

    /// `init()` (`klippy/extras/display/st7920.py:63`).
    pub fn init(&self, send: &mut dyn FnMut(&[u8], bool, bool) -> KResult<()>) -> KResult<()> {
        const CMDS: [u8; 8] = [0x24, 0x40, 0x02, 0x26, 0x22, 0x02, 0x06, 0x0c];
        send(&CMDS, false, false)?;
        self.flush(send)
    }

    /// `cache_glyph()` (`klippy/extras/display/st7920.py:74`).
    pub fn cache_glyph(&self, glyph_name: &str, base_glyph_name: &str, glyph_id: i64) {
        self.state.with_mut(|s| {
            let icon = s.icons.get(glyph_name).cloned();
            let base_icon = s.icons.get(base_glyph_name).cloned();
            let (Some(icon), Some(base_icon)) = (icon, base_icon) else {
                return;
            };
            let n = icon.0.len().min(icon.1.len()).min(base_icon.0.len()).min(base_icon.1.len());
            for i in 0..n {
                let x1 = icon.0[i] ^ base_icon.0[i];
                let x2 = icon.1[i] ^ base_icon.1[i];
                let pos = glyph_id * 32 + i as i64 * 2;
                py_slice_assign(&mut s.glyph_framebuffer, pos, pos + 2, &[x1, x2]);
                py_slice_assign(&mut s.glyph_old, pos, pos + 2, &[x1 ^ 1, x2 ^ 1]);
            }
            s.cached_glyphs
                .insert(glyph_name.to_string(), (base_glyph_name.to_string(), (0, glyph_id * 2)));
        });
    }

    /// `set_glyphs()` (`klippy/extras/display/st7920.py:86`).
    pub fn set_glyphs(&self, glyphs: &PyValue) {
        let PyValue::Dict(glyphs) = glyphs else {
            self.cache_glyph("fan2", "fan1", 0);
            self.cache_glyph("bed_heat2", "bed_heat1", 1);
            return;
        };
        self.state.with_mut(|s| {
            for (glyph_name, glyph_data) in glyphs.iter() {
                let PyValue::Str(glyph_name) = glyph_name else {
                    continue;
                };
                let PyValue::Dict(fields) = glyph_data else {
                    continue;
                };
                let Some(icon_value) = fields.get(&PyValue::Str("icon16x16".to_string())) else {
                    continue;
                };
                if matches!(icon_value, PyValue::None) {
                    continue;
                }
                if let Some(icon) = parse_icon(icon_value) {
                    s.icons.insert(glyph_name.clone(), icon);
                }
            }
        });
        // Setup animated glyphs.
        self.cache_glyph("fan2", "fan1", 0);
        self.cache_glyph("bed_heat2", "bed_heat1", 1);
    }

    /// `write_text()` (`klippy/extras/display/st7920.py:94`).
    pub fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        let mut data = data;
        if x + data.len() as i64 > 16 {
            let keep = (16 - x.min(16)).max(0) as usize;
            data = &data[..keep.min(data.len())];
        }
        let base = match py_index(y, 4) {
            Some(idx) => [0i64, 32, 16, 48][idx],
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        let pos = base + x;
        let len = data.len() as i64;
        let data = data.to_vec();
        self.state.with_mut(|s| {
            py_slice_assign(&mut s.text_framebuffer, pos, pos + len, &data);
        });
        Ok(())
    }

    /// `write_graphics()` (`klippy/extras/display/st7920.py:99`).
    pub fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        if x >= 16 || y >= 4 || data.len() != 16 {
            return Ok(());
        }
        let mut gfx_fb = y * 16;
        let mut x = x;
        if gfx_fb >= 32 {
            gfx_fb -= 32;
            x += 16;
        }
        self.state.with_mut(|s| {
            for (i, &bits) in data.iter().enumerate() {
                let fb = match py_index(gfx_fb + i as i64, s.graphics_framebuffers.len()) {
                    Some(v) => v,
                    None => {
                        return Err(KlippyError::Internal {
                            class_name: "IndexError".to_string(),
                            message: "list index out of range".to_string(),
                        })
                    }
                };
                let col = match py_index(x, s.graphics_framebuffers[fb].len()) {
                    Some(v) => v,
                    None => {
                        return Err(KlippyError::Internal {
                            class_name: "IndexError".to_string(),
                            message: "bytearray index out of range".to_string(),
                        })
                    }
                };
                s.graphics_framebuffers[fb][col] = bits;
            }
            Ok(())
        })
    }

    /// `write_glyph()` (`klippy/extras/display/st7920.py:108`).
    pub fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        let cached =
            self.state.borrow().cached_glyphs.get(glyph_name).map(|(b, t)| (b.clone(), *t));
        let resolved = match cached {
            Some((base, (c0, c1))) if x & 1 == 0 => {
                // Render cached icon using character generator.
                let cached_bytes = [c0, c1];
                let bytes = cached_bytes
                    .iter()
                    .map(|c| {
                        if *c < 0 || *c > 255 {
                            Err(KlippyError::Internal {
                                class_name: "ValueError".to_string(),
                                message: "bytes must be in range(0, 256)".to_string(),
                            })
                        } else {
                            Ok(*c as u8)
                        }
                    })
                    .collect::<KResult<Vec<u8>>>()?;
                self.write_text(x, y, &bytes)?;
                base
            }
            _ => glyph_name.to_string(),
        };
        let icon = self.state.borrow().icons.get(&resolved).cloned();
        if let Some((icon0, icon1)) = icon {
            // Draw icon in graphics mode.
            self.write_graphics(x, y, &icon0)?;
            self.write_graphics(x + 1, y, &icon1)?;
            return Ok(2);
        }
        match resolved.as_str() {
            "right_arrow" => {
                // Draw character.
                self.write_text(x, y, &RIGHT_ARROW_GLYPH)?;
                Ok(1)
            }
            "degrees" => {
                // Draw single width character.
                self.write_graphics(x, y, &DEGREES_GLYPH)?;
                Ok(1)
            }
            _ => Ok(0),
        }
    }

    /// `clear()` (`klippy/extras/display/st7920.py:131`).
    pub fn clear(&self) {
        self.state.with_mut(|s| {
            py_slice_assign(&mut s.text_framebuffer, 0, 64, &[b' '; 64]);
            let zeros = vec![0u8; 32];
            for gfb in s.graphics_framebuffers.iter_mut() {
                py_slice_assign(gfb, 0, 32, &zeros);
            }
        });
    }

    /// `get_dimensions()` (`klippy/extras/display/st7920.py:136`).
    pub fn get_dimensions(&self) -> (usize, usize) {
        (16, 4)
    }

    fn fb_snapshot(&self, fb_idx: usize) -> (Vec<u8>, Vec<u8>, u8) {
        let s = self.state.borrow();
        match fb_idx {
            0 => (s.text_framebuffer.clone(), s.text_old.clone(), 0x80),
            1 => (s.glyph_framebuffer.clone(), s.glyph_old.clone(), 0x40),
            _ => {
                let i = fb_idx - 2;
                (s.graphics_framebuffers[i].clone(), s.graphics_old[i].clone(), i as u8)
            }
        }
    }

    fn fb_copy(&self, fb_idx: usize) {
        self.state.with_mut(|s| match fb_idx {
            0 => {
                let new = s.text_framebuffer.clone();
                s.text_old = new;
            }
            1 => {
                let new = s.glyph_framebuffer.clone();
                s.glyph_old = new;
            }
            _ => {
                let i = fb_idx - 2;
                let new = s.graphics_framebuffers[i].clone();
                s.graphics_old[i] = new;
            }
        });
    }
}

// ---------------------------------------------------------------------------
// DisplayChip: the surface `extras.display.display.PrinterLCD` uses.
// ---------------------------------------------------------------------------

/// The surface `PrinterLCD` uses on the chip objects (`display.py:181`).
pub trait DisplayChip: Send + Sync {
    fn set_glyphs(&self, glyphs: &PyValue);
    fn get_dimensions(&self) -> (usize, usize);
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()>;
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()>;
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64>;
    fn clear(&self);
    fn init(&self) -> KResult<()>;
    fn flush(&self) -> KResult<()>;
}

// ---------------------------------------------------------------------------
// ST7920: hardware driver (`klippy/extras/display/st7920.py:140`).
// ---------------------------------------------------------------------------

struct St7920State {
    send_data_cmd: Option<CommandWrapper>,
    send_cmds_cmd: Option<CommandWrapper>,
    is_extended: bool,
}

/// Port of `ST7920` (`klippy/extras/display/st7920.py:140`).
pub struct St7920 {
    base: DisplayBase,
    state: GCell<St7920State>,
    mcu: Mcu,
    oid: i64,
    pins: Vec<String>,
}

impl St7920 {
    /// `ST7920(config)` (`klippy/extras/display/st7920.py:141`).
    pub fn new(config: &Config) -> KResult<Arc<St7920>> {
        let printer = config.get_printer();
        let pins = lookup_pins(&printer)?;
        let mut pin_params = Vec::with_capacity(3);
        for name in ["cs", "sclk", "sid"] {
            let pin_desc = config
                .get(&format!("{name}_pin"), GetStrOpts::default())?
                .ok_or_else(|| {
                    KlippyError::config(format!(
                        "Option '{name}_pin' in section '{}' must be specified",
                        config.get_name()
                    ))
                })?;
            pin_params.push(pins.lookup_pin(&pin_desc, Default::default())?);
        }
        let mut mcu: Option<Mcu> = None;
        for params in &pin_params {
            if let Some(m) = &mcu {
                if !Arc::ptr_eq(&params.chip, m) {
                    return Err(KlippyError::Other {
                        class_name: "error".to_string(),
                        message: "st7920 all pins must be on same mcu".to_string(),
                    });
                }
            } else {
                mcu = Some(Arc::clone(&params.chip));
            }
        }
        let mcu = mcu.expect("at least one pin");
        let pins = pin_params.iter().map(|p| p.pin.clone()).collect();
        let oid = mcu.create_oid()?;
        let st = Arc::new(St7920 {
            base: DisplayBase::new(),
            state: GCell::new(St7920State {
                send_data_cmd: None,
                send_cmds_cmd: None,
                is_extended: false,
            }),
            mcu: Arc::clone(&mcu),
            oid,
            pins,
        });
        let cb_st = Arc::clone(&st);
        mcu.register_config_callback(Box::new(move || cb_st.build_config()))?;
        Ok(st)
    }

    /// `build_config()` (`klippy/extras/display/st7920.py:161`).
    pub fn build_config(&self) -> KResult<()> {
        self.mcu.add_config_cmd(
            &pct_format(
                "config_st7920 oid=%u cs_pin=%s sclk_pin=%s sid_pin=%s sync_delay_ticks=%d cmd_delay_ticks=%d",
                &[
                    pct_int(self.oid),
                    PctValue::Str(self.pins[0].clone()),
                    PctValue::Str(self.pins[1].clone()),
                    PctValue::Str(self.pins[2].clone()),
                    pct_int(self.mcu.seconds_to_clock(ST7920_SYNC_DELAY)),
                    pct_int(self.mcu.seconds_to_clock(ST7920_CMD_DELAY)),
                ],
            ),
            AddConfigCmdOpts::default(),
        )?;
        let cmd_queue = self.mcu.alloc_command_queue();
        let send_cmds_cmd = self
            .mcu
            .lookup_command("st7920_send_cmds oid=%c cmds=%*s", Some(cmd_queue))?;
        let send_data_cmd = self
            .mcu
            .lookup_command("st7920_send_data oid=%c data=%*s", Some(cmd_queue))?;
        self.state.with_mut(|s| {
            s.send_cmds_cmd = Some(send_cmds_cmd);
            s.send_data_cmd = Some(send_data_cmd);
        });
        Ok(())
    }

    /// `send()` (`klippy/extras/display/st7920.py:174`).
    pub fn send(&self, cmds: &[u8], is_data: bool, is_extended: bool) -> KResult<()> {
        let (cmd_opt, final_cmds) = self.state.with_mut(|s| {
            let mut final_cmds = cmds.to_vec();
            let cmd_opt = if is_data {
                s.send_data_cmd.clone()
            } else {
                if s.is_extended != is_extended {
                    let add_cmd = if is_extended { 0x26 } else { 0x22 };
                    final_cmds.insert(0, add_cmd);
                    s.is_extended = is_extended;
                }
                s.send_cmds_cmd.clone()
            };
            (cmd_opt, final_cmds)
        });
        let cmd = cmd_opt.ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "'NoneType' object has no attribute 'send'".to_string(),
        })?;
        let cmds_arg = if is_data {
            PyValue::Bytes(final_cmds)
        } else {
            PyValue::List(
                final_cmds
                    .into_iter()
                    .map(|b| PyValue::Int(PyInt::from_i64(b as i64)))
                    .collect(),
            )
        };
        cmd.send(
            vec![PyValue::Int(PyInt::from_i64(self.oid)), cmds_arg],
            SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK },
        )
    }

    /// `init()` (`DisplayBase.init` through `ST7920.send`).
    pub fn init(&self) -> KResult<()> {
        self.base
            .init(&mut |cmds, is_data, is_extended| self.send(cmds, is_data, is_extended))
    }

    /// `flush()` (`DisplayBase.flush` through `ST7920.send`).
    pub fn flush(&self) -> KResult<()> {
        self.base
            .flush(&mut |cmds, is_data, is_extended| self.send(cmds, is_data, is_extended))
    }
}

impl DisplayChip for St7920 {
    fn set_glyphs(&self, glyphs: &PyValue) {
        self.base.set_glyphs(glyphs);
    }
    fn get_dimensions(&self) -> (usize, usize) {
        self.base.get_dimensions()
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.base.write_text(x, y, data)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.base.write_graphics(x, y, data)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.base.write_glyph(x, y, glyph_name)
    }
    fn clear(&self) {
        self.base.clear();
    }
    fn init(&self) -> KResult<()> {
        St7920::init(self)
    }
    fn flush(&self) -> KResult<()> {
        St7920::flush(self)
    }
}


// ---------------------------------------------------------------------------
// EnableHelper + EmulatedST7920 (`klippy/extras/display/st7920.py:188`).
// ---------------------------------------------------------------------------

/// Port of `EnableHelper` (`klippy/extras/display/st7920.py:188`).
pub struct EnableHelper {
    en_pin: Arc<McuBusDigitalOut>,
}

impl EnableHelper {
    /// `EnableHelper(pin_desc, spi)` (`st7920.py:189`).
    pub fn new(pin_desc: &str, spi: &McuSpi) -> KResult<EnableHelper> {
        Ok(EnableHelper {
            en_pin: McuBusDigitalOut::new(spi.get_mcu(), pin_desc, Some(spi.get_command_queue()), 0)?,
        })
    }

    /// `init()` (`st7920.py:192`).
    pub fn init(&self) -> KResult<()> {
        let mcu = self.en_pin.get_mcu();
        let curtime = mcu.get_printer().get_reactor().monotonic();
        let print_time = mcu.estimated_print_time(curtime);
        let minclock = mcu.print_time_to_clock(print_time + 0.100);
        self.en_pin.update_digital_out(false, SendOpts { minclock, reqclock: 0 })?;
        let minclock = mcu.print_time_to_clock(print_time + 0.200);
        self.en_pin.update_digital_out(true, SendOpts { minclock, reqclock: 0 })?;
        let minclock = mcu.print_time_to_clock(print_time + 0.300);
        self.en_pin.update_digital_out(true, SendOpts { minclock, reqclock: 0 })?;
        Ok(())
    }
}

struct EmulatedState {
    is_extended: bool,
    en_set: bool,
}

/// Port of `EmulatedST7920` (`klippy/extras/display/st7920.py:209`).
pub struct EmulatedSt7920 {
    base: DisplayBase,
    state: GCell<EmulatedState>,
    spi: Arc<McuSpi>,
    en_helper: EnableHelper,
}

impl EmulatedSt7920 {
    /// `EmulatedST7920(config)` (`st7920.py:210`).
    pub fn new(config: &Config) -> KResult<EmulatedSt7920> {
        let printer = config.get_printer();
        let pins = lookup_pins(&printer)?;
        let sw_names = [
            "spi_software_miso_pin",
            "spi_software_mosi_pin",
            "spi_software_sclk_pin",
        ];
        let mut sw_pin_params = Vec::with_capacity(3);
        for name in sw_names {
            let pin_desc = config.get(name, GetStrOpts::default())?.ok_or_else(|| {
                KlippyError::config(format!(
                    "Option '{name}' in section '{}' must be specified",
                    config.get_name()
                ))
            })?;
            sw_pin_params.push(pins.lookup_pin(
                &pin_desc,
                klippy_api::mcu::LookupPinOpts {
                    share_type: Some(name.to_string()),
                    ..Default::default()
                },
            )?);
        }
        let mut mcu: Option<Mcu> = None;
        for params in &sw_pin_params {
            if let Some(m) = &mcu {
                if !Arc::ptr_eq(&params.chip, m) {
                    return Err(KlippyError::Other {
                        class_name: "error".to_string(),
                        message: format!("{}: spi pins must be on same mcu", config.get_name()),
                    });
                }
            } else {
                mcu = Some(Arc::clone(&params.chip));
            }
        }
        let mcu = mcu.expect("at least one pin");
        let sw_pins: [String; 3] = [
            sw_pin_params[0].pin.clone(),
            sw_pin_params[1].pin.clone(),
            sw_pin_params[2].pin.clone(),
        ];
        let speed = config
            .getint(
                "spi_speed",
                GetIntOpts { default: Some(Some(1_000_000)), minval: Some(100_000), ..Default::default() },
            )?
            .ok_or_else(|| {
                KlippyError::config(format!(
                    "Option 'spi_speed' in section '{}' must be specified",
                    config.get_name()
                ))
            })?;
        let spi = McuSpi::new(mcu, None, None, 0, speed, Some(sw_pins), false)?;
        let en_pin_desc = config.get("en_pin", GetStrOpts::default())?.ok_or_else(|| {
            KlippyError::config(format!(
                "Option 'en_pin' in section '{}' must be specified",
                config.get_name()
            ))
        })?;
        Ok(EmulatedSt7920 {
            base: DisplayBase::new(),
            state: GCell::new(EmulatedState { is_extended: false, en_set: false }),
            en_helper: EnableHelper::new(&en_pin_desc, &spi)?,
            spi,
        })
    }

    /// `send()` (`st7920.py:232`).
    pub fn send(&self, cmds: &[u8], is_data: bool, is_extended: bool) -> KResult<()> {
        let final_cmds = self.state.with_mut(|s| {
            let mut final_cmds = cmds.to_vec();
            if !is_data && s.is_extended != is_extended {
                let add_cmd = if is_extended { 0x26 } else { 0x22 };
                final_cmds.insert(0, add_cmd);
                s.is_extended = is_extended;
            }
            final_cmds
        });
        // Copy data to ST7920 data format.
        let sync_byte = if is_data { 0xfa } else { 0xf8 };
        let mut spi_data = vec![0u8; 2 * final_cmds.len() + 1];
        spi_data[0] = sync_byte;
        let mut i = 1;
        for &b in &final_cmds {
            spi_data[i] = b & 0xf0;
            spi_data[i + 1] = (b & 0x0f) << 4;
            i += 2;
        }
        if !self.state.borrow().en_set {
            self.en_helper.init()?;
            self.state.with_mut(|s| {
                s.en_set = true;
            });
        }
        self.spi.spi_send(&spi_data, SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK })
    }

    /// `init()` (`DisplayBase.init` through `EmulatedST7920.send`).
    pub fn init(&self) -> KResult<()> {
        self.base
            .init(&mut |cmds, is_data, is_extended| self.send(cmds, is_data, is_extended))
    }

    /// `flush()` (`DisplayBase.flush` through `EmulatedST7920.send`).
    pub fn flush(&self) -> KResult<()> {
        self.base
            .flush(&mut |cmds, is_data, is_extended| self.send(cmds, is_data, is_extended))
    }
}

impl DisplayChip for EmulatedSt7920 {
    fn set_glyphs(&self, glyphs: &PyValue) {
        self.base.set_glyphs(glyphs);
    }
    fn get_dimensions(&self) -> (usize, usize) {
        self.base.get_dimensions()
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.base.write_text(x, y, data)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.base.write_graphics(x, y, data)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.base.write_glyph(x, y, glyph_name)
    }
    fn clear(&self) {
        self.base.clear();
    }
    fn init(&self) -> KResult<()> {
        EmulatedSt7920::init(self)
    }
    fn flush(&self) -> KResult<()> {
        EmulatedSt7920::flush(self)
    }
}
