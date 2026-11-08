//! Port of `klippy/extras/display/uc1701.py` (UC1701 / SSD1306 / SH1106 LCDs).
//!
//! Python has no `load_config` entry point in this file; `display.py` builds
//! the chips directly (`LCD_chips = {'uc1701': uc1701.UC1701, ...}`), so
//! [`module_factory`] is empty and the `from_config` constructors below are
//! the API. The SPI / I2C buses and the bus-synchronized digital outputs
//! belong to the bus owner; this module imports their traits and
//! constructors from [`crate::bus`](crate::bus) and builds them from the
//! config exactly like the Python chip constructors do.

use std::sync::Arc;

use crate::bus::{
    mcu_i2c_from_config, mcu_spi_from_config, McuBusDigitalOut, McuBusDigitalOutApi, McuI2cApi,
    McuSpiApi,
};
use crate::display::font8x14::VGA_FONT;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::mcu::{CommandQueue, Mcu, SendOpts};
use klippy_api::printer::ModuleFactory;

// ---------------------------------------------------------------------------
// Module header (`uc1701.py:11-13`)
// ---------------------------------------------------------------------------

/// `BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000` (`uc1701.py:11`).
pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fff_ffff_0000_0000;

/// Text glyphs drawn in text mode (`uc1701.py:13`).
pub fn text_glyph(name: &str) -> Option<&'static [u8]> {
    match name {
        "right_arrow" => Some(&[0x1a]),
        "degrees" => Some(&[0xf8]),
        _ => None,
    }
}
/// Glyph table passed to `set_glyphs`: outer map is `glyph_name ->
/// glyph_data`, inner value is the optional `(icon1, icon2)` (`icon16x16`
/// entry); `None` is a glyph dict without that key, as in Python.
pub type GlyphTable = IndexMap<String, Option<(Vec<u8>, Vec<u8>)>>;

// ---------------------------------------------------------------------------
// Bytearray slice assignment with CPython semantics.
// ---------------------------------------------------------------------------

/// `a[start:stop] = vals` for a `bytearray` `a` (`uc1701.py` relies on slice
/// assignment in `write_text` / `write_glyph`).
fn bytearray_slice_assign(buf: &mut Vec<u8>, start: usize, stop: usize, vals: &[u8]) {
    // CPython clamps out-of-range slice indices; a start beyond the end
    // appends, growing the bytearray.
    let len = buf.len();
    let start = start.min(len);
    let stop = stop.min(len);
    if start >= len {
        buf.extend_from_slice(vals);
        return;
    }
    if stop >= start {
        buf.splice(start..stop, vals.iter().copied());
    } else {
        // `a[i:j]` with `j < i` is an empty slice at `i`.
        buf.splice(start..start, vals.iter().copied());
    }
}

// ---------------------------------------------------------------------------
// Transport trait (`io.send`, owned by this module).
// ---------------------------------------------------------------------------

/// Byte transport used by [`DisplayBase`] (`io.send`, `uc1701.py:17`).
pub trait DisplayTransport: Send + Sync {
    /// `send(cmds, is_data=False)` (`uc1701.py:124-127`, `135-142`).
    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()>;
}

// ---------------------------------------------------------------------------
// `DisplayBase` (`uc1701.py:15-115`)
// ---------------------------------------------------------------------------

struct BaseState {
    vram: Vec<Vec<u8>>,
    shadow: Vec<Vec<u8>>,
    icons: IndexMap<String, (Vec<u8>, Vec<u8>)>,
}

/// `DisplayBase` (`uc1701.py:15`).
pub struct DisplayBase {
    io: Arc<dyn DisplayTransport>,
    columns: usize,
    x_offset: usize,
    state: GCell<BaseState>,
    /// Cached [`swizzle_bits`] of every `VGA_FONT` glyph, in display byte order.
    font: Vec<(Vec<u8>, Vec<u8>)>,
}

impl DisplayBase {
    /// `DisplayBase(io, columns=128, x_offset=0)` (`uc1701.py:16`).
    pub fn new(io: Arc<dyn DisplayTransport>, columns: usize, x_offset: usize) -> DisplayBase {
        let vram = vec![vec![0u8; columns]; 8];
        let shadow = vec![vec![b'~'; columns]; 8];
        let font = VGA_FONT.iter().map(|c| swizzle_bits(c)).collect();
        DisplayBase {
            io,
            columns,
            x_offset,
            state: GCell::new(BaseState {
                vram,
                shadow,
                icons: IndexMap::new(),
            }),
            font,
        }
    }

    /// `columns` attribute.
    pub fn columns(&self) -> usize {
        self.columns
    }

    /// `x_offset` attribute.
    pub fn x_offset(&self) -> usize {
        self.x_offset
    }

    /// `vram` snapshot (test access; `bytearray` copies in Python).
    pub fn vram_snapshot(&self) -> Vec<Vec<u8>> {
        self.state.borrow().vram.clone()
    }

    /// The `io.send` transport (chip `init` paths send through it).
    pub fn transport(&self) -> &Arc<dyn DisplayTransport> {
        &self.io
    }

    /// `_swizzle_bits(data)` (`uc1701.py:53`).
    pub fn swizzle_bits(data: &[u8]) -> (Vec<u8>, Vec<u8>) {
        swizzle_bits(data)
    }

    /// `flush` (`uc1701.py:28`).
    pub fn flush(&self) -> KResult<()> {
        // Compute every page's sends first and drop the borrow before any
        // `io.send` call (a borrow must not span a call that may pause the
        // reactor); each page's shadow is updated right after its sends,
        // matching `old_data[:] = new_data` (`uc1701.py:52`) per page.
        for page in 0..8 {
            let (sends, new_data) = {
                let st = self.state.borrow();
                if st.vram[page] == st.shadow[page] {
                    continue;
                }
                let new_data = st.vram[page].clone();
                let old_data = &st.shadow[page];
                let mut diffs: Vec<(usize, usize)> = new_data
                    .iter()
                    .zip(old_data.iter())
                    .enumerate()
                    .filter(|(_, (n, o))| n != o)
                    .map(|(i, _)| (i, 1usize))
                    .collect();
                // Batch together changes that are close to each other.
                let mut i = diffs.len() as i64 - 2;
                while i >= 0 {
                    let (pos, _count) = diffs[i as usize];
                    let (nextpos, nextcount) = diffs[i as usize + 1];
                    if pos + 5 >= nextpos && nextcount < 16 {
                        diffs[i as usize].1 = nextcount + (nextpos - pos);
                        diffs.remove(i as usize + 1);
                    }
                    i -= 1;
                }
                let mut sends: Vec<(Vec<u8>, bool)> = Vec::new();
                for (col_pos, count) in diffs {
                    let ra = 0xb0 | (page & 0x0F);
                    let ca_msb = 0x10 | ((col_pos >> 4) & 0x0F);
                    let ca_lsb = (col_pos & 0x0F) as u8;
                    sends.push((vec![ra as u8, ca_msb as u8, ca_lsb], false));
                    let end = (col_pos + count).min(new_data.len());
                    sends.push((new_data[col_pos..end].to_vec(), true));
                }
                (sends, new_data)
            };
            for (cmds, is_data) in &sends {
                self.io.send(cmds, *is_data)?;
            }
            self.state.borrow_mut().shadow[page] = new_data;
        }
        Ok(())
    }

    /// `set_glyphs(glyphs)` (`uc1701.py:64`).
    ///
    /// `glyphs` maps glyph name -> optional `(icon1, icon2)` (`icon16x16`
    /// entry); a `None` entry is a glyph dict without that key and is
    /// skipped, as in Python.
    pub fn set_glyphs(&self, glyphs: &GlyphTable) {
        let mut st = self.state.borrow_mut();
        for (glyph_name, glyph_data) in glyphs.iter() {
            if let Some(icon) = glyph_data {
                let (top1, bot1) = swizzle_bits(&icon.0);
                let (top2, bot2) = swizzle_bits(&icon.1);
                let mut top = top1;
                top.extend_from_slice(&top2);
                let mut bot = bot1;
                bot.extend_from_slice(&bot2);
                st.icons.insert(glyph_name.clone(), (top, bot));
            }
        }
    }

    /// `write_text(x, y, data)` (`uc1701.py:71`).
    pub fn write_text(&self, x: usize, y: usize, data: &[u8]) {
        let mut data: Vec<u8> = data.to_vec();
        if x + data.len() > 16 {
            let keep = 16usize.saturating_sub(x.min(16));
            data.truncate(keep);
        }
        let mut pix_x = x * 8;
        pix_x += self.x_offset;
        let page_top = y * 2;
        let page_bot = y * 2 + 1;
        let mut st = self.state.borrow_mut();
        for &c in data.iter() {
            let (bits_top, bits_bot) = &self.font[c as usize];
            {
                let page = &mut st.vram[page_top];
                bytearray_slice_assign(page, pix_x, pix_x + 8, bits_top);
            }
            {
                let page = &mut st.vram[page_bot];
                bytearray_slice_assign(page, pix_x, pix_x + 8, bits_bot);
            }
            pix_x += 8;
        }
    }

    /// `write_graphics(x, y, data)` (`uc1701.py:83`).
    pub fn write_graphics(&self, x: usize, y: usize, data: &[u8]) {
        if x >= 16 || y >= 4 || data.len() != 16 {
            return;
        }
        let (bits_top, bits_bot) = swizzle_bits(data);
        let mut pix_x = x * 8;
        pix_x += self.x_offset;
        let page_top = y * 2;
        let page_bot = y * 2 + 1;
        let mut st = self.state.borrow_mut();
        for i in 0..8 {
            // Direct indexing: Python raises `IndexError` past the end of
            // the page (`page_top[pix_x + i] ^= ...`); reproduce it.
            st.vram[page_top][pix_x + i] ^= bits_top[i];
            st.vram[page_bot][pix_x + i] ^= bits_bot[i];
        }
    }

    /// `write_glyph(x, y, glyph_name)` (`uc1701.py:94`).
    pub fn write_glyph(&self, x: usize, y: usize, glyph_name: &str) -> i64 {
        let icon = self.state.borrow().icons.get(glyph_name).cloned();
        if let Some(icon) = icon {
            if x < 15 {
                let mut pix_x = x * 8;
                pix_x += self.x_offset;
                let page_idx = y * 2;
                let mut st = self.state.borrow_mut();
                {
                    let page = &mut st.vram[page_idx];
                    bytearray_slice_assign(page, pix_x, pix_x + 16, &icon.0);
                }
                {
                    let page = &mut st.vram[page_idx + 1];
                    bytearray_slice_assign(page, pix_x, pix_x + 16, &icon.1);
                }
                return 2;
            }
        }
        if let Some(ch) = text_glyph(glyph_name) {
            self.write_text(x, y, ch);
            return 1;
        }
        0
    }

    /// `clear` (`uc1701.py:110`).
    pub fn clear(&self) {
        let zeros = vec![0u8; self.columns];
        for page in self.state.borrow_mut().vram.iter_mut() {
            *page = zeros.clone();
        }
    }

    /// `get_dimensions` (`uc1701.py:114`).
    pub fn get_dimensions(&self) -> (usize, usize) {
        (16, 4)
    }
}

/// `_swizzle_bits` implementation shared by [`DisplayBase`] and `set_glyphs`.
fn swizzle_bits(data: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let mut top: u64 = 0;
    let mut bot: u64 = 0;
    for row in 0..8 {
        // Python ints are unbounded; `& 0x8080...` keeps the low bits, which
        // is exactly a wrapping `u64` multiply here.
        let spaced = (data[row] as u64).wrapping_mul(0x8040201008040201) & 0x8080808080808080;
        top |= spaced >> (7 - row);
        let spaced2 = (data[row + 8] as u64).wrapping_mul(0x8040201008040201) & 0x8080808080808080;
        bot |= spaced2 >> (7 - row);
    }
    let bits_top: Vec<u8> = (0..64usize)
        .step_by(8)
        .map(|s| ((top >> s) & 0xff) as u8)
        .collect();
    let bits_bot: Vec<u8> = (0..64usize)
        .step_by(8)
        .map(|s| ((bot >> s) & 0xff) as u8)
        .collect();
    (bits_top, bits_bot)
}

// ---------------------------------------------------------------------------
// `SPI4wire` (`uc1701.py:118`) and `I2C` (`uc1701.py:130`)
// ---------------------------------------------------------------------------

/// `SPI4wire(config, data_pin_name)` (`uc1701.py:119`).
pub struct Spi4wire {
    spi: Arc<dyn McuSpiApi>,
    dc: Arc<dyn McuBusDigitalOutApi>,
}

impl Spi4wire {
    /// `SPI4wire` with already-built bus objects (test seam).
    pub fn from_parts(spi: Arc<dyn McuSpiApi>, dc: Arc<dyn McuBusDigitalOutApi>) -> Spi4wire {
        Spi4wire { spi, dc }
    }

    /// `SPI4wire.__init__` (`uc1701.py:119-123`): builds the SPI bus from the
    /// config (`pin_option` defaults to `cs_pin` in `bus.py`, `mode` is 0,
    /// `default_speed` is 10000000), then the bus-synchronized data/control
    /// output on `data_pin_name` sharing the SPI command queue.
    pub fn from_config(config: &Config, data_pin_name: &str) -> KResult<Arc<Spi4wire>> {
        let spi: Arc<dyn McuSpiApi> = mcu_spi_from_config(config, 0, "cs_pin", 10000000, None, false)?;
        let dc_pin = config
            .get(data_pin_name, GetStrOpts::default())?
            .expect("required pin option");
        let dc: Arc<dyn McuBusDigitalOutApi> =
            McuBusDigitalOut::new(spi.get_mcu(), &dc_pin, Some(spi.get_command_queue()), 0)?;
        Ok(Arc::new(Spi4wire { spi, dc }))
    }

    /// `spi.get_mcu` (bus surface, used by `ResetHelper`).
    pub fn bus_mcu(&self) -> Mcu {
        self.spi.get_mcu()
    }

    /// `spi.get_command_queue` (bus surface, used by `ResetHelper`).
    pub fn bus_command_queue(&self) -> CommandQueue {
        self.spi.get_command_queue()
    }
}

impl DisplayTransport for Spi4wire {
    /// `send` (`uc1701.py:124`).
    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()> {
        self.dc
            .update_digital_out(is_data, SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK })?;
        self.spi.spi_send(cmds, SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK })
    }
}

/// `I2C(config, default_addr)` (`uc1701.py:131`).
pub struct I2cDisplay {
    i2c: Arc<dyn McuI2cApi>,
}

impl I2cDisplay {
    /// `I2C` with an already-built bus object (test seam).
    pub fn from_parts(i2c: Arc<dyn McuI2cApi>) -> I2cDisplay {
        I2cDisplay { i2c }
    }

    /// `I2C.__init__` (`uc1701.py:131-134`): builds the I2C bus from the
    /// config (`default_speed` is 400000, `async_write_only` is `True`).
    pub fn from_config(config: &Config, default_addr: i64) -> KResult<Arc<I2cDisplay>> {
        let i2c: Arc<dyn McuI2cApi> = mcu_i2c_from_config(config, Some(default_addr), 400000, true)?;
        Ok(Arc::new(I2cDisplay { i2c }))
    }

    /// `i2c.get_mcu` (bus surface, used by `ResetHelper`).
    pub fn bus_mcu(&self) -> Mcu {
        self.i2c.get_mcu()
    }

    /// `i2c.get_command_queue` (bus surface, used by `ResetHelper`).
    pub fn bus_command_queue(&self) -> CommandQueue {
        self.i2c.get_command_queue()
    }
}

impl DisplayTransport for I2cDisplay {
    /// `send` (`uc1701.py:135`).
    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()> {
        let hdr: u8 = if is_data { 0x40 } else { 0x00 };
        let mut buf = vec![hdr];
        buf.extend_from_slice(cmds);
        self.i2c.i2c_write_noack(&buf, SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK })
    }
}

// ---------------------------------------------------------------------------
// `ResetHelper` (`uc1701.py:145`)
// ---------------------------------------------------------------------------

/// `ResetHelper` (`uc1701.py:145`).
pub struct ResetHelper {
    mcu_reset: Option<Arc<dyn McuBusDigitalOutApi>>,
}

impl ResetHelper {
    /// `ResetHelper` with an already-built output (test seam).
    pub fn from_parts(mcu_reset: Option<Arc<dyn McuBusDigitalOutApi>>) -> ResetHelper {
        ResetHelper { mcu_reset }
    }

    /// `ResetHelper.__init__` (`uc1701.py:146-151`): `None` when
    /// `pin_desc` is `None`, otherwise the bus-synchronized digital out for
    /// it on the display bus's MCU and command queue.
    pub fn new(pin_desc: Option<&str>, mcu: &Mcu, cmd_queue: CommandQueue) -> KResult<ResetHelper> {
        let Some(pin_desc) = pin_desc else {
            return Ok(ResetHelper { mcu_reset: None });
        };
        let mcu_reset: Arc<dyn McuBusDigitalOutApi> =
            McuBusDigitalOut::new(mcu.clone(), pin_desc, Some(cmd_queue), 0)?;
        Ok(ResetHelper { mcu_reset: Some(mcu_reset) })
    }

    /// `init` (`uc1701.py:152`).
    pub fn init(&self) -> KResult<()> {
        let Some(mcu_reset) = &self.mcu_reset else {
            return Ok(());
        };
        let mcu = mcu_reset.get_mcu();
        let curtime = mcu.get_printer().get_reactor().monotonic();
        let print_time = mcu.estimated_print_time(curtime);
        let minclock = mcu.print_time_to_clock(print_time + 0.100);
        mcu_reset.update_digital_out(false, SendOpts { minclock, reqclock: 0 })?;
        let minclock = mcu.print_time_to_clock(print_time + 0.200);
        mcu_reset.update_digital_out(true, SendOpts { minclock, reqclock: 0 })?;
        let minclock = mcu.print_time_to_clock(print_time + 0.300);
        mcu_reset.update_digital_out(true, SendOpts { minclock, reqclock: 0 })?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// `UC1701` (`uc1701.py:168`), `SSD1306` (`uc1701.py:199`), `SH1106` (`uc1701.py:238`)
// ---------------------------------------------------------------------------

/// `UC1701` (`uc1701.py:168`).
pub struct Uc1701 {
    base: DisplayBase,
    contrast: i64,
    reset: ResetHelper,
}

impl Uc1701 {
    /// `UC1701` with an already-built transport and reset helper (test seam).
    pub fn from_parts(io: Arc<dyn DisplayTransport>, contrast: i64, reset: ResetHelper) -> Uc1701 {
        Uc1701 {
            base: DisplayBase::new(io, 128, 0),
            contrast,
            reset,
        }
    }

    /// `UC1701.__init__` (`uc1701.py:169-173`): builds the `SPI4wire` bus
    /// (which reads `cs_pin` and `a0_pin`), then reads `contrast`, then
    /// `rst_pin`, in that order.
    pub fn from_config(config: &Config) -> KResult<Uc1701> {
        let io = Spi4wire::from_config(config, "a0_pin")?;
        let mcu = io.bus_mcu();
        let cmd_queue = io.bus_command_queue();
        let base = DisplayBase::new(io, 128, 0);
        let contrast = config
            .getint(
                "contrast",
                GetIntOpts {
                    default: Some(Some(40)),
                    minval: Some(0),
                    maxval: Some(63),
                    ..Default::default()
                },
            )?
            .unwrap();
        let reset = ResetHelper::new(
            config
                .get("rst_pin", GetStrOpts { default: Some(None), note_valid: true })?
                .as_deref(),
            &mcu,
            cmd_queue,
        )?;
        Ok(Uc1701 { base, contrast, reset })
    }

    /// `contrast` attribute.
    pub fn contrast(&self) -> i64 {
        self.contrast
    }

    /// `init` (`uc1701.py:174`).
    pub fn init(&self) -> KResult<()> {
        self.reset.init()?;
        let init_cmds: [u8; 17] = [
            0xE2,
            0x40,
            0xA0,
            0xC8,
            0xA2,
            0x2C,
            0x2E,
            0x2F,
            0xF8,
            0x00,
            0x23,
            0x81,
            (self.contrast & 0xff) as u8,
            0xAC,
            0x00,
            0xA6,
            0xAF,
        ];
        self.base.transport().send(&init_cmds, false)?;
        self.base.transport().send(&[0xA5], false)?;
        self.base.transport().send(&[0xA4], false)?;
        self.base.flush()
    }

    /// `flush` passthrough.
    pub fn flush(&self) -> KResult<()> {
        self.base.flush()
    }

    /// `clear` passthrough.
    pub fn clear(&self) {
        self.base.clear()
    }

    /// `write_text` passthrough.
    pub fn write_text(&self, x: usize, y: usize, data: &[u8]) {
        self.base.write_text(x, y, data)
    }

    /// `write_graphics` passthrough.
    pub fn write_graphics(&self, x: usize, y: usize, data: &[u8]) {
        self.base.write_graphics(x, y, data)
    }

    /// `write_glyph` passthrough.
    pub fn write_glyph(&self, x: usize, y: usize, glyph_name: &str) -> i64 {
        self.base.write_glyph(x, y, glyph_name)
    }

    /// `set_glyphs` passthrough.
    pub fn set_glyphs(&self, glyphs: &GlyphTable) {
        self.base.set_glyphs(glyphs)
    }

    /// `get_dimensions` passthrough.
    pub fn get_dimensions(&self) -> (usize, usize) {
        self.base.get_dimensions()
    }

    /// `vram` snapshot (test access).
    pub fn vram_snapshot(&self) -> Vec<Vec<u8>> {
        self.base.vram_snapshot()
    }
}

/// `SSD1306` (`uc1701.py:199`).
pub struct Ssd1306 {
    base: DisplayBase,
    contrast: i64,
    vcomh: i64,
    invert: bool,
    reset: ResetHelper,
}

impl Ssd1306 {
    /// `SSD1306` with an already-built transport and reset helper (test seam).
    pub fn from_parts(
        io: Arc<dyn DisplayTransport>,
        columns: usize,
        x_offset: usize,
        contrast: i64,
        vcomh: i64,
        invert: bool,
        reset: ResetHelper,
    ) -> Ssd1306 {
        Ssd1306 {
            base: DisplayBase::new(io, columns, x_offset),
            contrast,
            vcomh,
            invert,
            reset,
        }
    }

    /// `SSD1306.__init__` (`uc1701.py:200-212`): reads `cs_pin` first and
    /// chooses the I2C bus (no `cs_pin`) or the `SPI4wire` bus (`dc_pin`),
    /// then reads `reset_pin`, then `contrast`, `vcomh` and `invert`, in
    /// that order.
    pub fn from_config(
        config: &Config,
        columns: usize,
        x_offset: usize,
    ) -> KResult<Ssd1306> {
        let cs_pin = config.get(
            "cs_pin",
            GetStrOpts { default: Some(None), note_valid: true },
        )?;
        let (io, mcu, cmd_queue): (Arc<dyn DisplayTransport>, Mcu, CommandQueue) = match &cs_pin {
            None => {
                let i2c = I2cDisplay::from_config(config, 60)?;
                let (mcu, cmd_queue) = (i2c.bus_mcu(), i2c.bus_command_queue());
                (i2c, mcu, cmd_queue)
            }
            Some(_) => {
                let spi = Spi4wire::from_config(config, "dc_pin")?;
                let (mcu, cmd_queue) = (spi.bus_mcu(), spi.bus_command_queue());
                (spi, mcu, cmd_queue)
            }
        };
        let reset = ResetHelper::new(
            config
                .get("reset_pin", GetStrOpts { default: Some(None), note_valid: true })?
                .as_deref(),
            &mcu,
            cmd_queue,
        )?;
        let base = DisplayBase::new(io, columns, x_offset);
        let contrast = config
            .getint(
                "contrast",
                GetIntOpts {
                    default: Some(Some(239)),
                    minval: Some(0),
                    maxval: Some(255),
                    ..Default::default()
                },
            )?
            .unwrap();
        let vcomh = config
            .getint(
                "vcomh",
                GetIntOpts {
                    default: Some(Some(0)),
                    minval: Some(0),
                    maxval: Some(63),
                    ..Default::default()
                },
            )?
            .unwrap();
        let invert = config
            .getboolean(
                "invert",
                GetBoolOpts {
                    default: Some(Some(false)),
                    ..Default::default()
                },
            )?
            .unwrap();
        Ok(Ssd1306 {
            base,
            contrast,
            vcomh,
            invert,
            reset,
        })
    }

    /// `contrast` attribute.
    pub fn contrast(&self) -> i64 {
        self.contrast
    }

    /// `vcomh` attribute.
    pub fn vcomh(&self) -> i64 {
        self.vcomh
    }

    /// `invert` attribute.
    pub fn invert(&self) -> bool {
        self.invert
    }

    /// `columns` attribute.
    pub fn columns(&self) -> usize {
        self.base.columns()
    }

    /// `x_offset` attribute.
    pub fn x_offset(&self) -> usize {
        self.base.x_offset()
    }

    /// `init` (`uc1701.py:213`).
    pub fn init(&self) -> KResult<()> {
        self.reset.init()?;
        let mut init_cmds: Vec<u8> = vec![
            0xAE,
            0xD5,
            0x80,
            0xA8,
            0x3f,
            0xD3,
            0x00,
            0x40,
            0x8D,
            0x14,
            0x20,
            0x02,
            0xA1,
            0xC8,
            0xDA,
            0x12,
            0x81,
            (self.contrast & 0xff) as u8,
            0xD9,
            0xA1,
            0xDB,
            (self.vcomh & 0xff) as u8,
            0x2E,
            0xA4,
        ];
        init_cmds.push(if self.invert { 0xA7 } else { 0xA6 });
        init_cmds.push(0xAF);
        self.base.transport().send(&init_cmds, false)?;
        self.base.flush()
    }

    /// `flush` passthrough.
    pub fn flush(&self) -> KResult<()> {
        self.base.flush()
    }

    /// `clear` passthrough.
    pub fn clear(&self) {
        self.base.clear()
    }

    /// `write_text` passthrough.
    pub fn write_text(&self, x: usize, y: usize, data: &[u8]) {
        self.base.write_text(x, y, data)
    }

    /// `write_graphics` passthrough.
    pub fn write_graphics(&self, x: usize, y: usize, data: &[u8]) {
        self.base.write_graphics(x, y, data)
    }

    /// `write_glyph` passthrough.
    pub fn write_glyph(&self, x: usize, y: usize, glyph_name: &str) -> i64 {
        self.base.write_glyph(x, y, glyph_name)
    }

    /// `set_glyphs` passthrough.
    pub fn set_glyphs(&self, glyphs: &GlyphTable) {
        self.base.set_glyphs(glyphs)
    }

    /// `get_dimensions` passthrough.
    pub fn get_dimensions(&self) -> (usize, usize) {
        self.base.get_dimensions()
    }

    /// `vram` snapshot (test access).
    pub fn vram_snapshot(&self) -> Vec<Vec<u8>> {
        self.base.vram_snapshot()
    }
}

/// `SH1106` (`uc1701.py:238`) — SSD1306-compatible with 132 columns.
pub struct Sh1106 {
    inner: Ssd1306,
}

impl Sh1106 {
    /// `SH1106` with an already-built transport and reset helper (test seam).
    pub fn from_parts(
        io: Arc<dyn DisplayTransport>,
        x_offset: usize,
        contrast: i64,
        vcomh: i64,
        invert: bool,
        reset: ResetHelper,
    ) -> Sh1106 {
        Sh1106 {
            inner: Ssd1306::from_parts(io, 132, x_offset, contrast, vcomh, invert, reset),
        }
    }

    /// `SH1106.__init__` (`uc1701.py:239-241`): reads `x_offset` first, then
    /// `SSD1306.__init__(config, 132, x_offset)` reads the rest.
    pub fn from_config(config: &Config) -> KResult<Sh1106> {
        let x_offset = config
            .getint(
                "x_offset",
                GetIntOpts {
                    default: Some(Some(0)),
                    minval: Some(0),
                    maxval: Some(3),
                    ..Default::default()
                },
            )?
            .unwrap();
        Ok(Sh1106 {
            inner: Ssd1306::from_config(config, 132, x_offset as usize)?,
        })
    }

    pub fn init(&self) -> KResult<()> {
        self.inner.init()
    }

    /// `contrast` attribute.
    pub fn contrast(&self) -> i64 {
        self.inner.contrast()
    }

    /// `vcomh` attribute.
    pub fn vcomh(&self) -> i64 {
        self.inner.vcomh()
    }

    /// `invert` attribute.
    pub fn invert(&self) -> bool {
        self.inner.invert()
    }

    /// `columns` attribute.
    pub fn columns(&self) -> usize {
        self.inner.columns()
    }

    /// `x_offset` attribute.
    pub fn x_offset(&self) -> usize {
        self.inner.x_offset()
    }

    pub fn flush(&self) -> KResult<()> {
        self.inner.flush()
    }

    pub fn clear(&self) {
        self.inner.clear()
    }

    pub fn write_text(&self, x: usize, y: usize, data: &[u8]) {
        self.inner.write_text(x, y, data)
    }

    pub fn write_graphics(&self, x: usize, y: usize, data: &[u8]) {
        self.inner.write_graphics(x, y, data)
    }

    pub fn write_glyph(&self, x: usize, y: usize, glyph_name: &str) -> i64 {
        self.inner.write_glyph(x, y, glyph_name)
    }

    pub fn set_glyphs(&self, glyphs: &GlyphTable) {
        self.inner.set_glyphs(glyphs)
    }

    pub fn get_dimensions(&self) -> (usize, usize) {
        self.inner.get_dimensions()
    }

    pub fn vram_snapshot(&self) -> Vec<Vec<u8>> {
        self.inner.vram_snapshot()
    }
}

// ---------------------------------------------------------------------------
// Module entry points. `uc1701.py` defines none; `display.py` constructs the
// classes directly, so the factory is empty and the `from_config`
// constructors above are the API.
// ---------------------------------------------------------------------------

/// Frozen factory (empty: the reference module has no entry points).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}

klippy_api::printer_object!(DisplayBase);
klippy_api::printer_object!(Uc1701);
klippy_api::printer_object!(Ssd1306);
klippy_api::printer_object!(Sh1106);
klippy_api::printer_object!(Spi4wire);
klippy_api::printer_object!(I2cDisplay);
klippy_api::printer_object!(ResetHelper);
