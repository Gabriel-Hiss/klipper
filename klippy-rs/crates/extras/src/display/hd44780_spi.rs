//! Port of `klippy/extras/display/hd44780_spi.py`: the SPI variant of the
//! HD44780 (20x4 text) LCD driver. The driver keeps three framebuffers
//! (two text rows and one glyph buffer) and transmits only the changed
//! ranges to the chip over SPI in 4-bit mode.
//!
//! The Python module has no `load_config`-style entry point: `display.py`
//! instantiates the class directly (`display.py:181`), so `module_factory`
//! is all `None`. [`Hd44780Spi::new`] resolves the SPI bus exactly like
//! the Python `__init__` (`bus.MCU_SPI_from_config(config, 0x00,
//! pin_option="latch_pin")` at `hd44780_spi.py:24`) through
//! `crate::bus::mcu_spi_from_config`, so `display.rs` can build the
//! driver with [`Hd44780Spi::new`]. [`Hd44780Spi::from_config`] is the
//! seam that takes an already-built bus as `Arc<dyn McuSpiApi>` (the
//! concrete `crate::bus::McuSpi` coerces to it); `new` uses it, and the
//! oracle tests pass a fake bus through it.
use indexmap::IndexMap;

use crate::bus::McuSpiApi;
use klippy_api::config::{Config, GetBoolOpts, GetChoiceOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{Mcu, SendOpts};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::GCell;
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// Default `line_length` (`hd44780_spi.py:11`).
pub const LINE_LENGTH_DEFAULT: i64 = 20;
/// Config choices for `line_length` (`hd44780_spi.py:12`).
pub const LINE_LENGTH_OPTIONS: [i64; 2] = [16, 20];


/// One `icon5x8` glyph entry as `display.py` builds it in the
/// `get_display_glyphs` dict (`display.py:166`): `{name: {'icon5x8':
/// (slot, bits)}}`. A glyph without `icon5x8` is ignored by
/// [`Hd44780Spi::set_glyphs`], exactly like the Python `.get('icon5x8')`
/// check (`hd44780_spi.py:102`).
#[derive(Clone, Debug, Default)]
pub struct GlyphIcon {
    /// The glyph's `icon5x8` data as `(slot, bits)`, or `None` when the
    /// glyph provides no 5x8 icon for this driver.
    pub icon5x8: Option<Icon5x8>,
}

/// 5x8 icon data: the HD44780 glyph slot and the 8 bitmap bytes
/// (`display.py:164-166`).
#[derive(Clone, Debug)]
pub struct Icon5x8 {
    /// Glyph slot (0..=7 in a real config).
    pub slot: i64,
    /// 8 bytes of bitmap data.
    pub bits: Vec<u8>,
}

/// Bit that marks a data byte (`self.data_mask = (1<<1)`).
const DATA_MASK: u8 = 0x02;
/// Bit that marks a command byte (`self.command_mask = 0`).
const COMMAND_MASK: u8 = 0x00;
/// Active level of the enable bit (`self.enable_mask = (1<<3)`).
const ENABLE_MASK: u8 = 0x08;

/// The LCD chip interface `display.rs` drives (`display.py:193` and
/// below): `init`, `set_glyphs`, `get_dimensions`, `clear`, `write_text`,
/// `write_glyph`, `write_graphics`, `flush`.
pub trait Hd44780SpiApi: Send + Sync + 'static {
    /// `init()` (`hd44780_spi.py:81`): protocol init and a full flush.
    fn init(&self) -> KResult<()>;
    /// `flush()` (`hd44780_spi.py:60`): transmit changed framebuffer
    /// ranges to the chip.
    fn flush(&self) -> KResult<()>;
    /// `write_text(x, y, data)` (`hd44780_spi.py:95`).
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()>;
    /// `set_glyphs(glyphs)` (`hd44780_spi.py:100`).
    fn set_glyphs(&self, glyphs: &IndexMap<String, GlyphIcon>);
    /// `write_glyph(x, y, glyph_name)` (`hd44780_spi.py:105`), returning
    /// 1 when something was drawn and 0 otherwise.
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64>;
    /// `write_graphics(x, y, data)` (`hd44780_spi.py:118`): a no-op for
    /// this chip.
    fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]);
    /// `clear()` (`hd44780_spi.py:120`): blank both text framebuffers.
    fn clear(&self);
    /// `get_dimensions()` (`hd44780_spi.py:124`): `(line_length, 4)`.
    fn get_dimensions(&self) -> (i64, i64);
}

/// `hd44780_spi` (`hd44780_spi.py:18`). Mutable state lives in `GCell`s
/// and no borrow spans a send, per the `GCell` rules.
pub struct Hd44780Spi {
    printer: Printer,
    protocol_init: bool,
    spi: Arc<dyn McuSpiApi>,
    mcu: Mcu,
    line_length: i64,
    text_framebuffers: GCell<[Vec<u8>; 2]>,
    glyph_framebuffer: GCell<Vec<u8>>,
    sent_text_framebuffers: GCell<[Vec<u8>; 2]>,
    sent_glyph_framebuffer: GCell<Vec<u8>>,
    icons: GCell<IndexMap<String, Icon5x8>>,
}

impl Hd44780Spi {
    /// `hd44780_spi.__init__(config)` (`hd44780_spi.py:19`). Resolves the
    /// SPI bus exactly like the Python code (`bus.MCU_SPI_from_config(
    /// config, 0x00, pin_option="latch_pin")` at `hd44780_spi.py:24`, with
    /// the `bus.py:124` defaults `default_speed=100000`, `share_type=None`,
    /// `cs_active_high=False`), then builds through [`from_config`](Self::from_config).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let spi = crate::bus::mcu_spi_from_config(config, 0x00, "latch_pin", 100000, None, false)?;
        Self::from_config(config, spi).map(Arc::new)
    }

    /// `hd44780_spi.__init__(config)` (`hd44780_spi.py:19`) with an
    /// already-built SPI bus: the caller runs `bus.MCU_SPI_from_config(
    /// config, 0x00, pin_option="latch_pin")` (see [`new`](Self::new)) and
    /// passes the result, coerced to `Arc<dyn McuSpiApi>`. The oracle tests
    /// pass a fake bus here. Config reads happen in the Python order:
    /// `hd44780_protocol_init` first, `line_length` last.
    pub fn from_config(config: &Config, spi: Arc<dyn McuSpiApi>) -> KResult<Hd44780Spi> {
        let printer: Printer = config.get_printer();
        let protocol_init = config
            .getboolean(
                "hd44780_protocol_init",
                GetBoolOpts { default: Some(Some(true)), note_valid: true },
            )?
            .unwrap_or(true);
        let mcu: Mcu = spi.get_mcu();
        let choices = PyValue::List(
            LINE_LENGTH_OPTIONS
                .iter()
                .map(|&n| PyValue::Int(PyInt::from_i64(n)))
                .collect(),
        );
        let choice = config.getchoice(
            "line_length",
            &choices,
            GetChoiceOpts {
                default: Some(Some(PyValue::Int(PyInt::from_i64(LINE_LENGTH_DEFAULT)))),
                note_valid: true,
            },
        )?;
        let line_length = match &choice {
            PyValue::Int(n) => n.0.to_string().parse::<i64>().unwrap_or(LINE_LENGTH_DEFAULT),
            other => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "line_length choice must be an int, got {}",
                        pycompat::value::type_name(other)
                    ),
                });
            }
        };
        let fb_len = (2 * line_length) as usize;
        Ok(Hd44780Spi {
            printer,
            protocol_init,
            spi,
            mcu,
            line_length,
            text_framebuffers: GCell::new([
                vec![b' '; fb_len],
                vec![b' '; fb_len],
            ]),
            glyph_framebuffer: GCell::new(vec![0u8; 64]),
            sent_text_framebuffers: GCell::new([
                vec![b'~'; fb_len],
                vec![b'~'; fb_len],
            ]),
            sent_glyph_framebuffer: GCell::new(vec![b'~'; 64]),
            icons: GCell::new(IndexMap::new()),
        })
    }

    /// `self.hd44780_protocol_init` (`hd44780_spi.py:21`).
    pub fn hd44780_protocol_init(&self) -> bool {
        self.protocol_init
    }

    /// `self.line_length` (`hd44780_spi.py:33`).
    pub fn line_length(&self) -> i64 {
        self.line_length
    }

    /// `self.mcu` (`hd44780_spi.py:26`), for `init` timing.
    pub fn mcu(&self) -> &Mcu {
        &self.mcu
    }

    /// `send_4_bits(cmd, is_data, minclock)` (`hd44780_spi.py:48`): one
    /// 4-bit nibble as three SPI bytes (enable pulse between the second
    /// and third). `cmd` is `& 0xF0`-ed inside exactly like the Python
    /// code, so callers may pass `data << 4` for the low nibble.
    fn send_4_bits(&self, cmd: u16, is_data: bool, minclock: i64) -> KResult<()> {
        let mask = if is_data { DATA_MASK } else { COMMAND_MASK };
        let byte = (cmd & 0xF0) as u8 | mask;
        self.spi.spi_send(&[byte], SendOpts { minclock, reqclock: 0 })?;
        self.spi.spi_send(&[byte | ENABLE_MASK], SendOpts { minclock, reqclock: 0 })?;
        self.spi.spi_send(&[byte], SendOpts { minclock, reqclock: 0 })
    }

    /// `send(cmds, is_data=False, minclock=0)` (`hd44780_spi.py:56`):
    /// each byte as high nibble then low nibble.
    fn send(&self, cmds: &[u8], is_data: bool, minclock: i64) -> KResult<()> {
        for &data in cmds {
            self.send_4_bits(data as u16, is_data, minclock)?;
            self.send_4_bits((data as u16) << 4, is_data, minclock)?;
        }
        Ok(())
    }

    /// `flush()` (`hd44780_spi.py:60`). Each framebuffer's current bytes
    /// are snapshotted before any send (the `GCell` rules); the snapshot
    /// of what was already transmitted is only replaced after its
    /// transmission succeeded, matching `old_data[:] = new_data`
    /// (`hd44780_spi.py:80`).
    pub fn flush(&self) -> KResult<()> {
        let (text_new, text_old, glyph_new, glyph_old) = {
            let text = self.text_framebuffers.borrow();
            let sent_text = self.sent_text_framebuffers.borrow();
            let glyph = self.glyph_framebuffer.borrow();
            let sent_glyph = self.sent_glyph_framebuffer.borrow();
            (
                [text[0].clone(), text[1].clone()],
                [sent_text[0].clone(), sent_text[1].clone()],
                glyph.clone(),
                sent_glyph.clone(),
            )
        };
        let framebuffers: [(&[u8], &[u8], u8); 3] = [
            (&text_new[0], &text_old[0], 0x80),
            (&text_new[1], &text_old[1], 0xC0),
            (&glyph_new, &glyph_old, 0x40),
        ];
        for (index, (new_data, old_data, fb_id)) in framebuffers.iter().enumerate() {
            if new_data == old_data {
                continue;
            }
            // Positions of all changed bytes.
            let mut diffs: Vec<(usize, usize)> = (0..new_data.len().min(old_data.len()))
                .filter(|&i| new_data[i] != old_data[i])
                .map(|i| (i, 1))
                .collect();
            // Batch together changes that are close to each other. The
            // `range(len(diffs)-2, -1, -1)` loop walks from len-2 back to 0.
            if diffs.len() >= 2 {
                for i in (0..diffs.len() - 1).rev() {
                    let (pos, _) = diffs[i];
                    let (nextpos, nextcount) = diffs[i + 1];
                    if pos + 4 >= nextpos && nextcount < 16 {
                        diffs[i].1 = nextcount + (nextpos - pos);
                        diffs.remove(i + 1);
                    }
                }
            }
            // Transmit changes.
            for &(pos, count) in &diffs {
                self.send(&[fb_id.wrapping_add(pos as u8)], false, 0)?;
                self.send(&new_data[pos..pos + count], true, 0)?;
            }
            // Reflect exactly what was transmitted.
            match index {
                0 => self.sent_text_framebuffers.borrow_mut()[0] = new_data.to_vec(),
                1 => self.sent_text_framebuffers.borrow_mut()[1] = new_data.to_vec(),
                _ => *self.sent_glyph_framebuffer.borrow_mut() = new_data.to_vec(),
            }
        }
        Ok(())
    }

    /// `init()` (`hd44780_spi.py:81`).
    pub fn init(&self) -> KResult<()> {
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(curtime);
        // Program 4bit / 2-line mode and then issue 0x02 "Home" command.
        let mut init: Vec<&[u8]> = if self.protocol_init {
            vec![&b"\x33"[..], &b"\x33"[..], &b"\x32"[..], &b"\x28\x28\x02"[..]]
        } else {
            vec![&b"\x02"[..]]
        };
        // Reset (set positive direction ; enable display and hide cursor).
        init.push(&b"\x06\x0c"[..]);
        for (i, cmds) in init.iter().enumerate() {
            let minclock = self.mcu.print_time_to_clock(print_time + i as f64 * 0.1);
            self.send(cmds, false, minclock)?;
        }
        self.flush()
    }

    /// `write_text(x, y, data)` (`hd44780_spi.py:95`). A negative `x`
    /// makes Python wrap the bytearray slice assignment; no Klipper
    /// caller can produce it (`display.py` passes `pos >= 0`), so it
    /// fails loudly instead.
    pub fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        let line_length = self.line_length;
        let mut buf = data;
        if x + data.len() as i64 > line_length {
            let keep = line_length - x.min(line_length);
            buf = &data[..keep as usize];
        }
        let pos = x + ((y & 0x02) >> 1) * line_length;
        if pos < 0 {
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: format!("write_text at negative position {pos}"),
            });
        }
        let pos = pos as usize;
        let row = (y & 1) as usize;
        self.text_framebuffers.with_mut(|framebuffers| {
            framebuffers[row][pos..pos + buf.len()].copy_from_slice(buf);
        });
        Ok(())
    }

    /// `set_glyphs(glyphs)` (`hd44780_spi.py:100`).
    pub fn set_glyphs(&self, glyphs: &IndexMap<String, GlyphIcon>) {
        for (glyph_name, glyph_data) in glyphs {
            if let Some(icon) = &glyph_data.icon5x8 {
                self.icons.borrow_mut().insert(glyph_name.clone(), icon.clone());
            }
        }
    }

    /// `write_glyph(x, y, glyph_name)` (`hd44780_spi.py:105`). Returns 1
    /// when the icon was drawn (the icon path or a built-in text glyph),
    /// 0 when the glyph is unknown.
    pub fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        let icon = self.icons.borrow().get(glyph_name).cloned();
        if let Some(icon) = icon {
            if icon.slot < 0 {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: format!("glyph slot {} is negative", icon.slot),
                });
            }
            self.write_text(x, y, &[icon.slot as u8])?;
            // `self.glyph_framebuffer[slot * 8:(slot + 1) * 8] = bits`
            // with Python slice-assignment semantics: an offset beyond
            // the end appends, a tail overlap extends the buffer.
            let start = (icon.slot * 8) as usize;
            self.glyph_framebuffer.with_mut(|framebuffer| {
                let len = framebuffer.len();
                if start >= len {
                    framebuffer.extend(icon.bits.iter().copied());
                } else {
                    framebuffer.splice(
                        start..start + 8.min(len - start),
                        icon.bits.iter().copied(),
                    );
                }
            });
            return Ok(1);
        }
        // `TextGlyphs` (`hd44780_spi.py:14`): there is a single entry.
        if glyph_name == "right_arrow" {
            self.write_text(x, y, b"\x7e")?;
            return Ok(1);
        }
        Ok(0)
    }

    /// `write_graphics(x, y, data)` (`hd44780_spi.py:118`): a no-op.
    pub fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]) {}

    /// `clear()` (`hd44780_spi.py:120`).
    pub fn clear(&self) {
        let spaces = vec![b' '; (2 * self.line_length) as usize];
        self.text_framebuffers.with_mut(|framebuffers| {
            framebuffers[0].clone_from(&spaces);
            framebuffers[1].clone_from(&spaces);
        });
    }

    /// `get_dimensions()` (`hd44780_spi.py:124`).
    pub fn get_dimensions(&self) -> (i64, i64) {
        (self.line_length, 4)
    }
}

impl Hd44780SpiApi for Hd44780Spi {
    fn init(&self) -> KResult<()> {
        Hd44780Spi::init(self)
    }
    fn flush(&self) -> KResult<()> {
        Hd44780Spi::flush(self)
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        Hd44780Spi::write_text(self, x, y, data)
    }
    fn set_glyphs(&self, glyphs: &IndexMap<String, GlyphIcon>) {
        Hd44780Spi::set_glyphs(self, glyphs)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        Hd44780Spi::write_glyph(self, x, y, glyph_name)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) {
        Hd44780Spi::write_graphics(self, x, y, data)
    }
    fn clear(&self) {
        Hd44780Spi::clear(self)
    }
    fn get_dimensions(&self) -> (i64, i64) {
        Hd44780Spi::get_dimensions(self)
    }
}

/// The Python module has no `load_config`-style entry point
/// (`display.py` constructs the class itself), so every factory entry is
/// `None`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
