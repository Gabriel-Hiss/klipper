//! Port of the `display::hd44780` module (`klippy/extras/display/hd44780.py`).
//!
//! Low-level 4-bit parallel HD44780 LCD driver. The object is constructed
//! directly by the display manager (`display::PrinterLCD` loads
//! `lcd_type: hd44780` via `hd44780.HD44780(config)`), it is not a
//! printer object of its own.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetChoiceOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{AddConfigCmdOpts, CommandQueue, CommandWrapper, Mcu, PinParams, PinsApi, SendOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{GCell, lookup_interface};
use pycompat::PyValue;
use std::sync::Arc;

pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fffffff00000000i64;
pub const LINE_LENGTH_DEFAULT: i64 = 20;
pub const HD44780_DELAY: f64 = 0.000040;

/// User-defined glyph entry (`icon5x8` data): `(slot, bits)`.
pub type Icon5x8 = (i64, Vec<i64>);

/// Glyph table passed to [`Hd44780Api::set_glyphs`]: outer map is
/// `glyph_name -> glyph_data`, inner map is `format_name -> entry`
/// (only `icon5x8` is read, mirroring `glyph_data.get('icon5x8')`).
pub type GlyphTable = IndexMap<String, IndexMap<String, Icon5x8>>;

/// Chip API used by the display manager (`display::PrinterLCD`).
pub trait Hd44780Api: Send + Sync + 'static {
    fn build_config(&self) -> KResult<()>;
    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()>;
    fn flush(&self) -> KResult<()>;
    fn init(&self) -> KResult<()>;
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()>;
    fn set_glyphs(&self, glyphs: &GlyphTable);
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64>;
    fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]) -> KResult<()>;
    fn clear(&self);
    fn get_dimensions(&self) -> (i64, i64);
    fn line_length(&self) -> i64;
    fn oid(&self) -> i64;
}

struct Framebuffers {
    text: Vec<Vec<u8>>,
    glyph: Vec<u8>,
    old_text: Vec<Vec<u8>>,
    old_glyph: Vec<u8>,
}

/// `klippy/extras/display/hd44780.py:HD44780`.
pub struct HD44780 {
    printer: Printer,
    protocol_init: bool,
    line_length: i64,
    pins: Vec<String>,
    mcu: Mcu,
    oid: i64,
    send_cmds_cmd: GCell<Option<CommandWrapper>>,
    send_data_cmd: GCell<Option<CommandWrapper>>,
    icons: GCell<IndexMap<String, Icon5x8>>,
    fbs: GCell<Framebuffers>,
}

impl HD44780 {
    /// `HD44780.__init__(config)` (`klippy/extras/display/hd44780.py:18`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let mut pin_params_list: Vec<PinParams> = Vec::with_capacity(6);
        for name in ["rs", "e", "d4", "d5", "d6", "d7"] {
            let pin_desc = config
                .get(&format!("{name}_pin"), GetStrOpts::default())?
                .expect("required pin option");
            pin_params_list.push(ppins.lookup_pin(&pin_desc, klippy_api::mcu::LookupPinOpts::default())?);
        }
        let protocol_init = config
            .getboolean(
                "hd44780_protocol_init",
                GetBoolOpts { default: Some(Some(true)), note_valid: true, ..Default::default() },
            )?
            .expect("defaulted boolean");
        let choices = PyValue::List(vec![
            PyValue::Int(pycompat::PyInt::from_i64(16)),
            PyValue::Int(pycompat::PyInt::from_i64(20)),
        ]);
        let line_length_value = config.getchoice(
            "line_length",
            &choices,
            GetChoiceOpts {
                default: Some(Some(PyValue::Int(pycompat::PyInt::from_i64(LINE_LENGTH_DEFAULT)))),
                note_valid: true,
            },
        )?;
        let line_length: i64 = match &line_length_value {
            PyValue::Int(n) => n.0.to_string().parse::<i64>().unwrap_or(LINE_LENGTH_DEFAULT),
            _ => LINE_LENGTH_DEFAULT,
        };
        let mut mcu: Option<Mcu> = None;
        for pin_params in &pin_params_list {
            if let Some(existing) = &mcu {
                if !Arc::ptr_eq(existing, &pin_params.chip) {
                    return Err(KlippyError::Pins(
                        "hd44780 all pins must be on same mcu".to_string(),
                    ));
                }
            }
            mcu = Some(pin_params.chip.clone());
        }
        let mcu = mcu.expect("six pins always yield an mcu");
        let pins: Vec<String> = pin_params_list.iter().map(|p| p.pin.clone()).collect();
        let oid = mcu.create_oid()?;
        let n = (2 * line_length) as usize;
        let this = Arc::new(HD44780 {
            printer,
            protocol_init,
            line_length,
            pins,
            oid,
            mcu: mcu.clone(),
            send_cmds_cmd: GCell::new(None),
            send_data_cmd: GCell::new(None),
            icons: GCell::new(IndexMap::new()),
            fbs: GCell::new(Framebuffers {
                text: vec![vec![b' '; n], vec![b' '; n]],
                glyph: vec![0u8; 64],
                old_text: vec![vec![b'~'; n], vec![b'~'; n]],
                old_glyph: vec![b'~'; 64],
            }),
        });
        let cb_this = Arc::clone(&this);
        mcu.register_config_callback(Box::new(move || cb_this.build_config()))?;
        Ok(this)
    }

    fn send_cmds(&self, cmds: &[u8], minclock: i64, reqclock: i64) -> KResult<()> {
        let cmd = match self.send_cmds_cmd.borrow().clone() {
            Some(c) => c,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'send'".to_string(),
                });
            }
        };
        cmd.send(
            vec![PyValue::Int(pycompat::PyInt::from_i64(self.oid)), PyValue::Bytes(cmds.to_vec())],
            SendOpts { minclock, reqclock },
        )
    }
}

impl Hd44780Api for HD44780 {
    fn build_config(&self) -> KResult<()> {
        let delay_ticks = self.mcu.seconds_to_clock(HD44780_DELAY);
        let cmd = format!(
            "config_hd44780 oid={} rs_pin={} e_pin={} d4_pin={} d5_pin={} d6_pin={} d7_pin={} delay_ticks={}",
            self.oid,
            self.pins[0],
            self.pins[1],
            self.pins[2],
            self.pins[3],
            self.pins[4],
            self.pins[5],
            delay_ticks
        );
        self.mcu.add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        let cq = self.mcu.alloc_command_queue();
        let cmds_cmd = self.mcu.lookup_command("hd44780_send_cmds oid=%c cmds=%*s", Some(cq))?;
        *self.send_cmds_cmd.borrow_mut() = Some(cmds_cmd);
        let data_cmd = self.mcu.lookup_command("hd44780_send_data oid=%c data=%*s", Some(cq))?;
        *self.send_data_cmd.borrow_mut() = Some(data_cmd);
        Ok(())
    }

    fn send(&self, cmds: &[u8], is_data: bool) -> KResult<()> {
        let cmd = if is_data { self.send_data_cmd.borrow().clone() } else { self.send_cmds_cmd.borrow().clone() };
        let cmd = match cmd {
            Some(c) => c,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'send'".to_string(),
                });
            }
        };
        cmd.send(
            vec![PyValue::Int(pycompat::PyInt::from_i64(self.oid)), PyValue::Bytes(cmds.to_vec())],
            SendOpts { minclock: 0, reqclock: BACKGROUND_PRIORITY_CLOCK },
        )
    }

    fn flush(&self) -> KResult<()> {
        let snapshots: Vec<(Vec<u8>, Vec<u8>, i64)> = {
            let fbs = self.fbs.borrow();
            vec![
                (fbs.text[0].clone(), fbs.old_text[0].clone(), 0x80),
                (fbs.text[1].clone(), fbs.old_text[1].clone(), 0xc0),
                (fbs.glyph.clone(), fbs.old_glyph.clone(), 0x40),
            ]
        };
        for (index, (new_data, old_data, fb_id)) in snapshots.iter().enumerate() {
            if new_data == old_data {
                continue;
            }
            let mut diffs: Vec<(i64, i64)> = Vec::new();
            let len = new_data.len().min(old_data.len());
            for i in 0..len {
                if new_data[i] != old_data[i] {
                    diffs.push((i as i64, 1));
                }
            }
            if diffs.len() >= 2 {
                for i in (0..diffs.len() - 1).rev() {
                    let (pos, _count) = diffs[i];
                    let (nextpos, nextcount) = diffs[i + 1];
                    if pos + 4 >= nextpos && nextcount < 16 {
                        diffs[i].1 = nextcount + (nextpos - pos);
                        diffs.remove(i + 1);
                    }
                }
            }
            for &(pos, count) in &diffs {
                let chip_pos = pos;
                self.send(&[(fb_id + chip_pos) as u8], false)?;
                let start = pos as usize;
                let end = (pos + count) as usize;
                let slice = if start >= new_data.len() {
                    Vec::new()
                } else {
                    new_data[start..end.min(new_data.len())].to_vec()
                };
                self.send(&slice, true)?;
            }
            let mut fbs = self.fbs.borrow_mut();
            match index {
                0 => fbs.old_text[0] = new_data.clone(),
                1 => fbs.old_text[1] = new_data.clone(),
                _ => fbs.old_glyph = new_data.clone(),
            }
        }
        Ok(())
    }

    fn init(&self) -> KResult<()> {
        let curtime = self.printer.get_reactor().monotonic();
        let print_time = self.mcu.estimated_print_time(curtime);
        let mut init: Vec<Vec<u8>> = if self.protocol_init {
            vec![vec![0x33], vec![0x33], vec![0x32], vec![0x28, 0x28, 0x02]]
        } else {
            vec![vec![0x02]]
        };
        init.push(vec![0x06, 0x0c]);
        for (i, cmds) in init.iter().enumerate() {
            let minclock = self.mcu.print_time_to_clock(print_time + i as f64 * 0.100);
            self.send_cmds(cmds, minclock, 0)?;
        }
        self.flush()
    }

    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        let mut truncated: &[u8] = data;
        if x + truncated.len() as i64 > self.line_length {
            let keep = self.line_length - x.min(self.line_length);
            let keep_usize = if keep < 0 { 0usize } else { keep as usize };
            truncated = &truncated[..keep_usize.min(truncated.len())];
        }
        let pos = x + ((y & 0x02) >> 1) * self.line_length;
        let buf_index = (y & 1) as usize;
        let mut fbs = self.fbs.borrow_mut();
        py_slice_assign(&mut fbs.text[buf_index], pos, pos + truncated.len() as i64, truncated);
        Ok(())
    }

    fn set_glyphs(&self, glyphs: &GlyphTable) {
        let mut icons = self.icons.borrow_mut();
        for (glyph_name, glyph_data) in glyphs.iter() {
            if let Some(data) = glyph_data.get("icon5x8") {
                icons.insert(glyph_name.clone(), data.clone());
            }
        }
    }

    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        let icon = self.icons.borrow().get(glyph_name).cloned();
        if let Some((slot, bits)) = icon {
            let slot_bytes = bytes_from_ints(&[slot])?;
            self.write_text(x, y, &slot_bytes)?;
            let bits_bytes = bytes_from_ints(&bits)?;
            let mut fbs = self.fbs.borrow_mut();
            let start = slot * 8;
            py_slice_assign(&mut fbs.glyph, start, (slot + 1) * 8, &bits_bytes);
            return Ok(1);
        }
        if glyph_name == "right_arrow" {
            self.write_text(x, y, &[0x7e])?;
            return Ok(1);
        }
        Ok(0)
    }

    fn write_graphics(&self, _x: i64, _y: i64, _data: &[u8]) -> KResult<()> {
        Ok(())
    }

    fn clear(&self) {
        let spaces = vec![b' '; (2 * self.line_length) as usize];
        let mut fbs = self.fbs.borrow_mut();
        fbs.text[0] = spaces.clone();
        fbs.text[1] = spaces;
    }

    fn get_dimensions(&self) -> (i64, i64) {
        (self.line_length, 4)
    }

    fn oid(&self) -> i64 {
        self.oid
    }

    fn line_length(&self) -> i64 {
        self.line_length
    }
}

impl HD44780 {
    /// Framebuffer snapshot helpers (read-only state for tests).
    pub fn text_framebuffers(&self) -> Vec<Vec<u8>> {
        let fbs = self.fbs.borrow();
        fbs.text.clone()
    }

    pub fn glyph_framebuffer(&self) -> Vec<u8> {
        let fbs = self.fbs.borrow();
        fbs.glyph.clone()
    }

    pub fn old_framebuffers(&self) -> (Vec<Vec<u8>>, Vec<u8>) {
        let fbs = self.fbs.borrow();
        (fbs.old_text.clone(), fbs.old_glyph.clone())
    }

    pub fn icons(&self) -> IndexMap<String, Icon5x8> {
        self.icons.borrow().clone()
    }

    pub fn hd44780_protocol_init(&self) -> bool {
        self.protocol_init
    }

    pub fn pins(&self) -> Vec<String> {
        self.pins.clone()
    }
}

klippy_api::printer_object!(HD44780; dyn Hd44780Api);

fn bytes_from_ints(vals: &[i64]) -> KResult<Vec<u8>> {
    let mut out = Vec::with_capacity(vals.len());
    for &v in vals {
        if v < 0 || v > 255 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "byte must be in range(0, 256)".to_string(),
            });
        }
        out.push(v as u8);
    }
    Ok(out)
}

/// CPython `bytearray[start:stop] = data` slice assignment.
fn py_slice_assign(buf: &mut Vec<u8>, start: i64, stop: i64, data: &[u8]) {
    let n = buf.len() as i64;
    let mut a = start;
    let mut b = stop;
    if a < 0 {
        a += n;
        if a < 0 {
            a = 0;
        }
    } else if a > n {
        a = n;
    }
    if b < 0 {
        b += n;
        if b < 0 {
            b = 0;
        }
    } else if b > n {
        b = n;
    }
    if b < a {
        b = a;
    }
    let new_len = (n - (b - a)) as usize + data.len();
    let prefix = buf[..a as usize].to_vec();
    let suffix = buf[b as usize..].to_vec();
    buf.clear();
    buf.extend_from_slice(&prefix);
    buf.extend_from_slice(data);
    buf.extend_from_slice(&suffix);
    debug_assert_eq!(buf.len(), new_len);
}

/// Legacy command queue type alias kept for call-site compatibility.
pub type CmdQueue = CommandQueue;

pub type Hd44780 = HD44780;

/// Module factory: `hd44780.py` defines no `load_config`/`load_config_prefix`
/// entry point (the class is constructed directly by `display.PrinterLCD`),
/// so every factory slot is `None`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
