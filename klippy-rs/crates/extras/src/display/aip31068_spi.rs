//! Port of the `display::aip31068_spi` module (`klippy/extras/display/aip31068_spi.py`).
//!
//! Support for YHCB2004 (20x4 text) LCD displays based on the AiP31068
//! controller, reached over SPI through the `crate::bus` port
//! (`bus.MCU_SPI`). Each command is 9 bits long (1 RS bit + 8 data bits).
//! The transport layer packs 8 commands into 9 bytes before sending, so
//! the MCU only ever sees byte-wide transfers.
//!
use crate::bus::{mcu_spi_from_config, McuSpiApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{Mcu, SendOpts};
use klippy_api::object::ObjectRef;
use klippy_api::printer::Printer;
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// `LINE_LENGTH_DEFAULT` (`aip31068_spi.py:19`).
pub const LINE_LENGTH_DEFAULT: i64 = 20;
/// `LINE_LENGTH_OPTIONS` (`aip31068_spi.py:20`).
pub const LINE_LENGTH_OPTIONS: [(i64, i64); 2] = [(16, 16), (20, 20)];

/// `TextGlyphs` (`aip31068_spi.py:22`).
pub fn text_glyph(name: &str) -> Option<u8> {
    if name == "right_arrow" {
        Some(0x7e)
    } else {
        None
    }
}

/// Command opcodes (`CMND`, `aip31068_spi.py:32-41`).
pub struct CMND;
impl CMND {
    pub const CLR: i64 = 1;
    pub const HOME: i64 = 2;
    pub const ENTERY_MODE: i64 = (2i64).pow(2);
    pub const DISPLAY: i64 = (2i64).pow(3);
    pub const SHIFT: i64 = (2i64).pow(4);
    pub const FUNCTION: i64 = (2i64).pow(5);
    pub const CGRAM: i64 = (2i64).pow(6);
    pub const DDRAM: i64 = (2i64).pow(7);
    pub const WRITE_RAM: i64 = (2i64).pow(8);
}

/// `flg_ENTERY_MODE` (`aip31068_spi.py:44-46`).
#[allow(non_camel_case_types)]
pub struct FLG_ENTERY_MODE;
impl FLG_ENTERY_MODE {
    pub const INC: i64 = (2i64).pow(1);
    pub const SHIFT: i64 = (2i64).pow(0);
}

/// `flg_DISPLAY` (`aip31068_spi.py:48-51`).
#[allow(non_camel_case_types)]
pub struct FLG_DISPLAY;
impl FLG_DISPLAY {
    pub const ON: i64 = (2i64).pow(2);
    pub const CURSOR: i64 = (2i64).pow(1);
    pub const BLINK: i64 = (2i64).pow(0);
}

/// `flg_SHIFT` (`aip31068_spi.py:53-55`).
#[allow(non_camel_case_types)]
pub struct FLG_SHIFT;
impl FLG_SHIFT {
    pub const WHOLE_DISPLAY: i64 = (2i64).pow(3);
    pub const RIGHT: i64 = (2i64).pow(2);
}

/// `flg_FUNCTION` (`aip31068_spi.py:57-59`).
#[allow(non_camel_case_types)]
pub struct FLG_FUNCTION;
impl FLG_FUNCTION {
    pub const TWO_LINES: i64 = (2i64).pow(3);
    pub const FIVE_BY_ELEVEN: i64 = (2i64).pow(2);
}

/// `flg_CGRAM` (`aip31068_spi.py:61-62`).
#[allow(non_camel_case_types)]
pub struct FLG_CGRAM;
impl FLG_CGRAM {
    pub const MASK: i64 = 0b0011_1111;
}

/// `flg_DDRAM` (`aip31068_spi.py:64-65`).
#[allow(non_camel_case_types)]
pub struct FLG_DDRAM;
impl FLG_DDRAM {
    pub const MASK: i64 = 0b0111_1111;
}

/// `flg_WRITE_RAM` (`aip31068_spi.py:67-68`).
#[allow(non_camel_case_types)]
pub struct FLG_WRITE_RAM;
impl FLG_WRITE_RAM {
    pub const MASK: i64 = 0b1111_1111;
}

/// `DISPLAY_INIT_CMNDS` (`aip31068_spi.py:70-77`).
pub const DISPLAY_INIT_CMNDS: [i64; 5] = [
    CMND::HOME,
    CMND::ENTERY_MODE | FLG_ENTERY_MODE::INC,
    CMND::DISPLAY | FLG_DISPLAY::ON,
    CMND::SHIFT | FLG_SHIFT::RIGHT,
    CMND::FUNCTION | FLG_FUNCTION::TWO_LINES,
];

// ---------------------------------------------------------------------------
// SPI transport: `crate::bus::McuSpi` (`bus.MCU_SPI`).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// LCD chip interface (`display.display.PrinterLCD` uses these methods).
// ---------------------------------------------------------------------------

/// Chip interface the printer's LCD layer drives (`display.py:176-274` uses
/// `init`/`flush`/`write_text`/`set_glyphs`/`write_glyph`/`write_graphics`/
/// `clear`/`get_dimensions`, plus `send` for diagnostics).
pub trait Aip31068SpiApi: Send + Sync + 'static {
    /// `aip31068_spi.init` (`aip31068_spi.py:172`).
    fn init(&self) -> KResult<()>;
    /// `aip31068_spi.flush` (`aip31068_spi.py:150`).
    fn flush(&self) -> KResult<()>;
    /// `aip31068_spi.send` (`aip31068_spi.py:138`).
    fn send(&self, data: &[i64], minclock: i64) -> KResult<()>;
    /// `aip31068_spi.write_text` (`aip31068_spi.py:179`).
    fn write_text(&self, x: i64, y: i64, data: &[u8]);
    /// `aip31068_spi.set_glyphs` (`aip31068_spi.py:184`).
    fn set_glyphs(
        &self,
        glyphs: &IndexMap<String, IndexMap<String, (i64, Vec<u8>)>>,
    );
    /// `aip31068_spi.write_glyph` (`aip31068_spi.py:189`).
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64>;
    /// `aip31068_spi.write_graphics` (`aip31068_spi.py:202`).
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]);
    /// `aip31068_spi.clear` (`aip31068_spi.py:204`).
    fn clear(&self);
    /// `aip31068_spi.get_dimensions` (`aip31068_spi.py:208`).
    fn get_dimensions(&self) -> (i64, i64);
}

// ---------------------------------------------------------------------------
// Driver object.
// ---------------------------------------------------------------------------

fn pyvalue_to_i64(v: &PyValue) -> KResult<i64> {
    match v {
        PyValue::Int(n) => n
            .inner()
            .to_string()
            .parse::<i64>()
            .map_err(|_| KlippyError::Internal {
                class_name: "OverflowError".to_string(),
                message: format!("Python int too large to fit in i64: {}", n.inner()),
            }),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "expected int for line_length, got {}",
                pycompat::value::type_name(v)
            ),
        }),
    }
}

fn read_line_length(config: &Config) -> KResult<i64> {
    let mut choices = IndexMap::new();
    for &(k, v) in LINE_LENGTH_OPTIONS.iter() {
        choices.insert(PyValue::Int(PyInt::from_i64(k)), PyValue::Int(PyInt::from_i64(v)));
    }
    let v = config.getchoice(
        "line_length",
        &PyValue::Dict(choices),
        GetChoiceOpts {
            default: Some(Some(PyValue::Int(PyInt::from_i64(LINE_LENGTH_DEFAULT)))),
            note_valid: true,
        },
    )?;
    pyvalue_to_i64(&v)
}

/// CPython `bytearray` simple-slice assignment (`buf[start:stop] = data`).
fn bytearray_slice_assign(buf: &mut Vec<u8>, start: i64, stop: i64, data: &[u8]) {
    let len = buf.len() as i64;
    let norm = |idx: i64| -> i64 {
        let mut x = if idx < 0 { len + idx } else { idx };
        if x < 0 {
            x = 0;
        }
        if x > len {
            x = len;
        }
        x
    };
    let s = norm(start);
    let mut e = norm(stop);
    if e < s {
        e = s;
    }
    buf.splice(s as usize..e as usize, data.iter().copied());
}

/// `klippy/extras/display/aip31068_spi.py:aip31068_spi`.
pub struct Aip31068Spi {
    printer: Printer,
    spi: Arc<dyn McuSpiApi>,
    mcu: Mcu,
    line_length: i64,
    text_new: GCell<Vec<Vec<u8>>>,
    text_old: GCell<Vec<Vec<u8>>>,
    glyph_new: GCell<Vec<u8>>,
    glyph_old: GCell<Vec<u8>>,
    icons: GCell<IndexMap<String, (i64, Vec<u8>)>>,
}

impl Aip31068Spi {
    fn buffer_len(&self) -> usize {
        (2 * self.line_length) as usize
    }

    /// Buffer construction shared by [`from_config_and_spi`](Self::from_config_and_spi).
    pub fn from_parts(printer: Printer, spi: Arc<dyn McuSpiApi>, line_length: i64) -> Self {
        let mcu = spi.get_mcu();
        let n = (2 * line_length) as usize;
        let spaces = vec![b' '; n];
        let tildes = vec![b'~'; n];
        Aip31068Spi {
            printer,
            spi,
            mcu,
            line_length,
            text_new: GCell::new(vec![spaces.clone(), spaces]),
            text_old: GCell::new(vec![tildes.clone(), tildes]),
            glyph_new: GCell::new(vec![0u8; 64]),
            glyph_old: GCell::new(vec![b'~'; 64]),
            icons: GCell::new(IndexMap::new()),
        }
    }

    /// `__init__` with an already-built SPI bus (transport injected).
    pub fn from_config_and_spi(config: &Config, spi: Arc<dyn McuSpiApi>) -> KResult<Self> {
        let printer = config.get_printer();
        let line_length = read_line_length(config)?;
        Ok(Self::from_parts(printer, spi, line_length))
    }

    /// `__init__(config)` (`aip31068_spi.py:80`): SPI first, then config.
    pub fn from_config(config: &Config) -> KResult<Self> {
        let spi: Arc<dyn McuSpiApi> =
            mcu_spi_from_config(config, 0x00, "latch_pin", 100_000, None, false)?;
        Self::from_config_and_spi(config, spi)
    }

    /// `line_length` field.
    pub fn line_length(&self) -> i64 {
        self.line_length
    }

    /// `encode` with the default 9-bit width (`aip31068_spi.py:112`).
    pub fn encode(data: &[i64]) -> KResult<Vec<u8>> {
        Self::encode_in_width(data, 9)
    }

    /// `encode(data, width)` (`aip31068_spi.py:112-137`).
    pub fn encode_in_width(data: &[i64], width: i64) -> KResult<Vec<u8>> {
        let w = u128::try_from(width).map_err(|_| KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "negative shift count".to_string(),
        })?;
        // `1 << width` in Python is unbounded; values are always checked
        // against it (the `>=` test below also covers negative `num`
        // only via full-width comparison, as in CPython).
        let limit: u128 = if w >= 128 { u128::MAX } else { 1u128 << w };
        let mut encoded: Vec<u8> = Vec::new();
        let mut accumulator: u128 = 0;
        let mut acc_bits: u128 = 0;
        for &num in data {
            // Production inputs are controller values 0..=511. A negative
            // `num` never occurs there; reject it rather than pack
            // CPython's infinite two's-complement tail.
            if num < 0 {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "negative value in encode is not representable".to_string(),
                });
            }
            let num_u = num as u128;
            if num_u >= limit {
                return Err(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: format!("Number {num} does not fit in {width} bits"),
                });
            }
            accumulator = (accumulator << w) | num_u;
            acc_bits += w;
            while acc_bits >= 8 {
                acc_bits -= 8;
                let byte = ((accumulator >> acc_bits) & 0xFF) as u8;
                // `accumulator &= (1 << acc_bits) - 1`
                let mask: u128 = if acc_bits == 0 { 0 } else { (1u128 << acc_bits) - 1 };
                accumulator &= mask;
                encoded.push(byte);
            }
        }
        if acc_bits > 0 {
            let last = (accumulator << (8 - acc_bits)) as u8;
            encoded.push(last);
        }
        Ok(encoded)
    }

    /// `send` (`aip31068_spi.py:138-149`).
    pub fn send(&self, data: &[i64], minclock: i64) -> KResult<()> {
        let pad = CMND::ENTERY_MODE | FLG_ENTERY_MODE::INC;
        let mut i: usize = 0;
        while i < data.len() {
            let end = (i + 8).min(data.len());
            let mut group: Vec<i64> = data[i..end].to_vec();
            if group.len() < 8 {
                group.resize(8, pad);
            }
            let encoded = Self::encode_in_width(&group, 9)?;
            self.spi.spi_send(&encoded, SendOpts { minclock, reqclock: 0 })?;
            i = end;
        }
        Ok(())
    }

    fn snapshot_entry(&self, idx: usize) -> (Vec<u8>, Vec<u8>, i64) {
        if idx < 2 {
            let new_data = self.text_new.borrow()[idx].clone();
            let old_data = self.text_old.borrow()[idx].clone();
            let fb = if idx == 0 {
                CMND::DDRAM | (FLG_DDRAM::MASK & 0x00)
            } else {
                CMND::DDRAM | (FLG_DDRAM::MASK & 0x40)
            };
            (new_data, old_data, fb)
        } else {
            (
                self.glyph_new.borrow().clone(),
                self.glyph_old.borrow().clone(),
                CMND::CGRAM | (FLG_CGRAM::MASK & 0x00),
            )
        }
    }

    fn update_old(&self, idx: usize, new_data: Vec<u8>) {
        if idx < 2 {
            self.text_old.borrow_mut()[idx] = new_data;
        } else {
            *self.glyph_old.borrow_mut() = new_data;
        }
    }

    /// `flush` (`aip31068_spi.py:150-171`).
    pub fn flush(&self) -> KResult<()> {
        for idx in 0..3 {
            let (new_data, old_data, fb_cmnd) = self.snapshot_entry(idx);
            if new_data == old_data {
                continue;
            }
            let mut diffs: Vec<(i64, i64)> = new_data
                .iter()
                .zip(old_data.iter())
                .enumerate()
                .filter(|(_, (n, o))| n != o)
                .map(|(i, _)| (i as i64, 1i64))
                .collect();
            let orig_len = diffs.len();
            if orig_len >= 2 {
                for i in (0..=orig_len - 2).rev() {
                    // `diffs[i]` and `diffs[i+1]` read after any merge at i+1,
                    // matching the backward scan of the Python loop.
                    let (pos, _count) = diffs[i];
                    let (nextpos, nextcount) = diffs[i + 1];
                    if pos + 4 >= nextpos && nextcount < 16 {
                        diffs[i].1 = nextcount + (nextpos - pos);
                        diffs.remove(i + 1);
                    }
                }
            }
            for (pos, count) in diffs.iter() {
                let pos = *pos;
                let count = *count;
                self.send(&[fb_cmnd + pos], 0)?;
                let start = pos as usize;
                let end = (pos + count) as usize;
                let mut payload: Vec<i64> =
                    new_data[start..end].iter().map(|&b| CMND::WRITE_RAM | b as i64).collect();
                if payload.is_empty() {
                    // `new_data[pos:pos+count]` empty only when count == 0,
                    // which the backward batching preserves in principle;
                    // send an empty payload to mirror `send([])`.
                    payload = Vec::new();
                }
                self.send(&payload, 0)?;
            }
            self.update_old(idx, new_data);
        }
        Ok(())
    }

    /// `init` (`aip31068_spi.py:172-178`).
    pub fn init(&self) -> KResult<()> {
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(curtime);
        for (i, cmd) in DISPLAY_INIT_CMNDS.iter().enumerate() {
            let minclock = self.mcu.print_time_to_clock(print_time + i as f64 * 0.100);
            self.send(&[*cmd], minclock)?;
        }
        self.flush()
    }

    /// `write_text` (`aip31068_spi.py:179-183`).
    pub fn write_text(&self, x: i64, y: i64, data: &[u8]) {
        let mut owned: Vec<u8> = data.to_vec();
        if x.saturating_add(owned.len() as i64) > self.line_length {
            let n = self.line_length - x.min(self.line_length);
            if (n as usize) < owned.len() {
                owned.truncate(n as usize);
            }
        }
        let pos = x.saturating_add(((y & 2) >> 1).saturating_mul(self.line_length));
        let buf_idx = (y & 1) as usize;
        let len = owned.len() as i64;
        let mut bufs = self.text_new.borrow_mut();
        bytearray_slice_assign(&mut bufs[buf_idx], pos, pos.saturating_add(len), &owned);
    }

    /// `set_glyphs` (`aip31068_spi.py:184-188`).
    pub fn set_glyphs(&self, glyphs: &IndexMap<String, IndexMap<String, (i64, Vec<u8>)>>) {
        let mut icons = self.icons.borrow_mut();
        for (glyph_name, glyph_data) in glyphs.iter() {
            if let Some(entry) = glyph_data.get("icon5x8") {
                icons.insert(glyph_name.clone(), entry.clone());
            }
        }
    }

    /// `write_glyph` (`aip31068_spi.py:189-201`).
    pub fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        let found = self.icons.borrow().get(glyph_name).map(|v| v.clone());
        if let Some((slot, bits)) = found {
            if !(0..=255).contains(&slot) {
                return Err(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: "byte must be in range(0, 256)".to_string(),
                });
            }
            self.write_text(x, y, &[slot as u8]);
            // `glyph_framebuffer[slot*8:(slot+1)*8] = bits`
            let start = slot * 8;
            let stop = (slot + 1) * 8;
            self.glyph_new.with_mut(|buf| {
                bytearray_slice_assign(buf, start, stop, &bits);
            });
            return Ok(1);
        }
        if let Some(ch) = text_glyph(glyph_name) {
            self.write_text(x, y, &[ch]);
            return Ok(1);
        }
        Ok(0)
    }

    /// `write_graphics` (`aip31068_spi.py:202-203`): this display supports
    /// only hardcoded or 8 user defined glyphs.
    pub fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]) {}

    /// `clear` (`aip31068_spi.py:204-207`).
    pub fn clear(&self) {
        let spaces = vec![b' '; self.buffer_len()];
        let mut bufs = self.text_new.borrow_mut();
        bufs[0] = spaces.clone();
        bufs[1] = spaces;
    }

    /// `get_dimensions` (`aip31068_spi.py:208-209`).
    pub fn get_dimensions(&self) -> (i64, i64) {
        (self.line_length, 4)
    }
}

impl Aip31068SpiApi for Aip31068Spi {
    fn init(&self) -> KResult<()> {
        Aip31068Spi::init(self)
    }
    fn flush(&self) -> KResult<()> {
        Aip31068Spi::flush(self)
    }
    fn send(&self, data: &[i64], minclock: i64) -> KResult<()> {
        Aip31068Spi::send(self, data, minclock)
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) {
        Aip31068Spi::write_text(self, x, y, data)
    }
    fn set_glyphs(&self, glyphs: &IndexMap<String, IndexMap<String, (i64, Vec<u8>)>>) {
        Aip31068Spi::set_glyphs(self, glyphs)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        Aip31068Spi::write_glyph(self, x, y, glyph_name)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) {
        Aip31068Spi::write_graphics(self, x, y, data)
    }
    fn clear(&self) {
        Aip31068Spi::clear(self)
    }
    fn get_dimensions(&self) -> (i64, i64) {
        Aip31068Spi::get_dimensions(self)
    }
}

/// The Python file has no `load_config` entry point (the class is
/// instantiated by `display.PrinterLCD` via `LCD_chips`). This factory is
/// provided for the printer module loader and builds the same object as
/// `__init__(config)`; section-based loading does not use it in practice.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(Aip31068Spi::from_config(config)?) as ObjectRef))
}

klippy_api::printer_object!(Aip31068Spi; dyn Aip31068SpiApi);

/// Module entry point for the `aip31068_spi` chip (see [`load_config`]).
pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    klippy_api::printer::ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
