//! Port of `klippy/extras/display/display.py`.
//!
//! The `[display]` section drives an LCD chip (`PrinterLCD`), renders the
//! `display_data` groups through `display_template` sections and the Jinja
//! templates of `gcode_macro`, and caches templates, groups and glyphs in the
//! shared `display_template` printer object (`PrinterDisplayTemplate`).
//!
//! Cross-module interfaces are declared by the display objects that own them.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use extras_macros_templates::gcode_macro::{parse_literal, TemplateError, Value};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetIntOpts, GetStrOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GetOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_READY};
use klippy_api::{load_interface, lookup_interface, GCell, ObjectRef};
use pycompat::text::{int_from_string, trunc_float_to_bigint};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};

use crate::display::aip31068_spi::{Aip31068Spi, Aip31068SpiApi};
use crate::display::hd44780::{self, Hd44780Api, HD44780};
use crate::display::hd44780_spi::{self, Hd44780Spi, Hd44780SpiApi};
use crate::display::menu::MenuManager;
use crate::display::st7920::{DisplayChip, EmulatedSt7920, St7920};
use crate::display::uc1701::{self, Sh1106, Ssd1306, Uc1701};
use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};

/// Normal time between each screen redraw (`display.py:12`).
pub const REDRAW_TIME: f64 = 0.500;
/// Minimum time between screen redraws (`display.py:14`).
pub const REDRAW_MIN_TIME: f64 = 0.100;

/// `cmd_SET_DISPLAY_GROUP_help` (`display.py:265`).
pub const SET_DISPLAY_GROUP_HELP: &str = "Set the active display group";

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `str(int)` of a Python int as `i64`; `None` when it does not fit.
fn pyint_to_i64(n: &PyInt) -> Option<i64> {
    n.0.to_string().parse::<i64>().ok()
}

fn python_whitespace(c: char) -> bool {
    c.is_whitespace() || matches!(c, '\u{1c}'..='\u{1f}')
}

/// `config.get(option)` with no default (a required option).
fn required_str(config: &Config, option: &str) -> KResult<String> {
    match config.get(option, GetStrOpts::default())? {
        Some(v) => Ok(v),
        None => Err(KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option,
            config.get_name()
        ))),
    }
}

// ---------------------------------------------------------------------------
// Glyphs
// ---------------------------------------------------------------------------

/// One entry of the glyph dict `PrinterDisplayTemplate.display_glyphs`
/// (`display.py:161,166`): `{'icon16x16': (icon1, icon2), 'icon5x8': (slot, idata)}`
/// with each key present only when the glyph section defined it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GlyphData {
    /// `'icon16x16'`: the high and the low byte of each of the 16 rows.
    pub icon16x16: Option<(Vec<i64>, Vec<i64>)>,
    /// `'icon5x8'`: the HD44780 character slot and its 8 rows.
    pub icon5x8: Option<(i64, Vec<i64>)>,
}

/// `PrinterDisplayTemplate.display_glyphs`: glyph name to its icons.
pub type GlyphTable = IndexMap<String, GlyphData>;

/// `PrinterDisplayTemplate._parse_glyph` (`display.py:103-114`).
fn parse_glyph(glyph_name: &str, data: &str, width: usize, height: usize) -> KResult<Vec<i64>> {
    let mut glyph_data = Vec::new();
    for line in data.split('\n') {
        let line = line.trim_matches(python_whitespace).replace('.', "0").replace('*', "1");
        if line.is_empty() {
            continue;
        }
        if line.chars().count() != width || !line.replace(['0', '1'], "").is_empty() {
            return Err(KlippyError::config(format!(
                "Invalid glyph line in {glyph_name}"
            )));
        }
        // `int(line, 2)`: only 0 and 1 remain and width <= 16.
        let bits = i64::from_str_radix(&line, 2).map_err(|_| {
            KlippyError::config(format!("Invalid glyph line in {glyph_name}"))
        })?;
        glyph_data.push(bits);
    }
    if glyph_data.len() != height {
        return Err(KlippyError::config(format!(
            "Glyph {glyph_name} incorrect lines"
        )));
    }
    Ok(glyph_data)
}

// ---------------------------------------------------------------------------
// LCD chips (`LCD_chips`, `display.py:16-22`)
// ---------------------------------------------------------------------------

/// The surface `PrinterLCD` uses on `self.lcd_chip` (`display.py:181-263`).
pub trait LcdChipApi: Send + Sync + 'static {
    /// `set_glyphs(glyphs)` with `PrinterDisplayTemplate.get_display_glyphs()`.
    fn set_glyphs(&self, glyphs: &GlyphTable);
    /// `get_dimensions()`: `(columns, rows)`.
    fn get_dimensions(&self) -> (i64, i64);
    /// `init()`.
    fn init(&self) -> KResult<()>;
    /// `clear()`.
    fn clear(&self);
    /// `flush()`.
    fn flush(&self) -> KResult<()>;
    /// `write_text(x, y, data)`.
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()>;
    /// `write_glyph(x, y, glyph_name)`: the number of columns drawn.
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64>;
    /// `write_graphics(x, y, data)`.
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()>;
}

/// A `LCD_chips` constructor: `LCD_chips[name](config)`.
pub type ChipCtor = Arc<dyn Fn(&Config) -> KResult<Arc<dyn LcdChipApi>> + Send + Sync>;

fn chip_ctor<F>(f: F) -> ChipCtor
where
    F: Fn(&Config) -> KResult<Arc<dyn LcdChipApi>> + Send + Sync + 'static,
{
    Arc::new(f)
}

fn py_int_list(values: &[i64]) -> PyValue {
    PyValue::List(
        values
            .iter()
            .map(|v| PyValue::Int(PyInt::from_i64(*v)))
            .collect(),
    )
}

/// The glyph dict as the Python chips receive it.
fn py_glyph_table(glyphs: &GlyphTable) -> PyValue {
    let mut table: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (name, data) in glyphs {
        let mut entry: IndexMap<PyValue, PyValue> = IndexMap::new();
        if let Some((icon1, icon2)) = &data.icon16x16 {
            entry.insert(
                PyValue::Str("icon16x16".to_string()),
                PyValue::Tuple(vec![py_int_list(icon1), py_int_list(icon2)]),
            );
        }
        if let Some((slot, idata)) = &data.icon5x8 {
            entry.insert(
                PyValue::Str("icon5x8".to_string()),
                PyValue::Tuple(vec![
                    PyValue::Int(PyInt::from_i64(*slot)),
                    py_int_list(idata),
                ]),
            );
        }
        table.insert(PyValue::Str(name.clone()), PyValue::Dict(entry));
    }
    PyValue::Dict(table)
}

fn icon_bytes(values: &[i64]) -> Vec<u8> {
    values.iter().map(|v| (*v & 0xff) as u8).collect()
}

/// `st7920.ST7920` and `st7920.EmulatedST7920`.
struct DisplayChipAdapter(Arc<dyn DisplayChip>);

impl LcdChipApi for DisplayChipAdapter {
    fn set_glyphs(&self, glyphs: &GlyphTable) {
        self.0.set_glyphs(&py_glyph_table(glyphs));
    }
    fn get_dimensions(&self) -> (i64, i64) {
        let (columns, rows) = self.0.get_dimensions();
        (columns as i64, rows as i64)
    }
    fn init(&self) -> KResult<()> {
        self.0.init()
    }
    fn clear(&self) {
        self.0.clear();
    }
    fn flush(&self) -> KResult<()> {
        self.0.flush()
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_text(x, y, data)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.0.write_glyph(x, y, glyph_name)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_graphics(x, y, data)
    }
}

/// `hd44780.HD44780`.
struct Hd44780Chip(Arc<dyn Hd44780Api>);

impl LcdChipApi for Hd44780Chip {
    fn set_glyphs(&self, glyphs: &GlyphTable) {
        let mut table = hd44780::GlyphTable::new();
        for (name, data) in glyphs {
            if let Some((slot, idata)) = &data.icon5x8 {
                let mut entry = IndexMap::new();
                entry.insert("icon5x8".to_string(), (*slot, idata.clone()));
                table.insert(name.clone(), entry);
            }
        }
        self.0.set_glyphs(&table);
    }
    fn get_dimensions(&self) -> (i64, i64) {
        self.0.get_dimensions()
    }
    fn init(&self) -> KResult<()> {
        self.0.init()
    }
    fn clear(&self) {
        self.0.clear();
    }
    fn flush(&self) -> KResult<()> {
        self.0.flush()
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_text(x, y, data)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.0.write_glyph(x, y, glyph_name)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_graphics(x, y, data)
    }
}

/// `hd44780_spi.hd44780_spi`.
struct Hd44780SpiChip(Arc<dyn Hd44780SpiApi>);

impl LcdChipApi for Hd44780SpiChip {
    fn set_glyphs(&self, glyphs: &GlyphTable) {
        let mut table: IndexMap<String, hd44780_spi::GlyphIcon> = IndexMap::new();
        for (name, data) in glyphs {
            if let Some((slot, idata)) = &data.icon5x8 {
                table.insert(
                    name.clone(),
                    hd44780_spi::GlyphIcon {
                        icon5x8: Some(hd44780_spi::Icon5x8 {
                            slot: *slot,
                            bits: icon_bytes(idata),
                        }),
                    },
                );
            }
        }
        self.0.set_glyphs(&table);
    }
    fn get_dimensions(&self) -> (i64, i64) {
        self.0.get_dimensions()
    }
    fn init(&self) -> KResult<()> {
        self.0.init()
    }
    fn clear(&self) {
        self.0.clear();
    }
    fn flush(&self) -> KResult<()> {
        self.0.flush()
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_text(x, y, data)
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.0.write_glyph(x, y, glyph_name)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_graphics(x, y, data);
        Ok(())
    }
}

/// `aip31068_spi.aip31068_spi`.
struct Aip31068SpiChip(Arc<dyn Aip31068SpiApi>);

impl LcdChipApi for Aip31068SpiChip {
    fn set_glyphs(&self, glyphs: &GlyphTable) {
        let mut table: IndexMap<String, IndexMap<String, (i64, Vec<u8>)>> = IndexMap::new();
        for (name, data) in glyphs {
            if let Some((slot, idata)) = &data.icon5x8 {
                let mut entry = IndexMap::new();
                entry.insert("icon5x8".to_string(), (*slot, icon_bytes(idata)));
                table.insert(name.clone(), entry);
            }
        }
        self.0.set_glyphs(&table);
    }
    fn get_dimensions(&self) -> (i64, i64) {
        self.0.get_dimensions()
    }
    fn init(&self) -> KResult<()> {
        self.0.init()
    }
    fn clear(&self) {
        self.0.clear();
    }
    fn flush(&self) -> KResult<()> {
        self.0.flush()
    }
    fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_text(x, y, data);
        Ok(())
    }
    fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
        self.0.write_glyph(x, y, glyph_name)
    }
    fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
        self.0.write_graphics(x, y, data);
        Ok(())
    }
}

/// Coordinates of the `uc1701` chips are `usize` and index four text rows.
/// Python raises `IndexError` for a row past the last page (`self.vram[y * 2]`),
/// which the chip port would turn into a panic, and negative coordinates
/// (Python's from-the-end indexing) cannot be expressed; both raise the same
/// `IndexError`.
fn uc1701_xy(x: i64, y: i64) -> KResult<(usize, usize)> {
    match (usize::try_from(x), usize::try_from(y)) {
        (Ok(x), Ok(y)) if y < 4 => Ok((x, y)),
        _ => Err(index_error()),
    }
}

/// `uc1701.UC1701`, `uc1701.SSD1306` and `uc1701.SH1106` share one surface.
macro_rules! uc1701_family_chip {
    ($wrapper:ident, $chip:ty) => {
        struct $wrapper {
            chip: $chip,
            /// Glyph names with an `icon16x16`, to tell when `write_glyph` draws.
            icons: GCell<IndexMap<String, ()>>,
        }

        impl $wrapper {
            fn new(chip: $chip) -> $wrapper {
                $wrapper {
                    chip,
                    icons: GCell::new(IndexMap::new()),
                }
            }
        }

        impl LcdChipApi for $wrapper {
            fn set_glyphs(&self, glyphs: &GlyphTable) {
                let mut table = uc1701::GlyphTable::new();
                for (name, data) in glyphs {
                    if let Some((icon1, icon2)) = &data.icon16x16 {
                        table.insert(
                            name.clone(),
                            Some((icon_bytes(icon1), icon_bytes(icon2))),
                        );
                    }
                }
                self.chip.set_glyphs(&table);
                self.icons.with_mut(|icons| {
                    for name in table.keys() {
                        icons.insert(name.clone(), ());
                    }
                });
            }
            fn get_dimensions(&self) -> (i64, i64) {
                let (columns, rows) = self.chip.get_dimensions();
                (columns as i64, rows as i64)
            }
            fn init(&self) -> KResult<()> {
                self.chip.init()
            }
            fn clear(&self) {
                self.chip.clear();
            }
            fn flush(&self) -> KResult<()> {
                self.chip.flush()
            }
            fn write_text(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
                let (x, y) = uc1701_xy(x, y)?;
                self.chip.write_text(x, y, data);
                Ok(())
            }
            fn write_glyph(&self, x: i64, y: i64, glyph_name: &str) -> KResult<i64> {
                let has_icon = x < 15 && self.icons.borrow().contains_key(glyph_name);
                if !has_icon && uc1701::text_glyph(glyph_name).is_none() {
                    return Ok(0);
                }
                let (x, y) = uc1701_xy(x, y)?;
                Ok(self.chip.write_glyph(x, y, glyph_name))
            }
            fn write_graphics(&self, x: i64, y: i64, data: &[u8]) -> KResult<()> {
                if x >= 16 || y >= 4 || data.len() != 16 {
                    return Ok(());
                }
                let (x, y) = uc1701_xy(x, y)?;
                self.chip.write_graphics(x, y, data);
                Ok(())
            }
        }
    };
}

uc1701_family_chip!(Uc1701Chip, Uc1701);
uc1701_family_chip!(Ssd1306Chip, Ssd1306);
uc1701_family_chip!(Sh1106Chip, Sh1106);

/// `LCD_chips` in the Python dict order (`display.py:16-22`).
pub fn lcd_chips() -> Vec<(String, ChipCtor)> {
    vec![
        (
            "st7920".to_string(),
            chip_ctor(|config| {
                let chip: Arc<dyn DisplayChip> = St7920::new(config)?;
                Ok(Arc::new(DisplayChipAdapter(chip)) as Arc<dyn LcdChipApi>)
            }),
        ),
        (
            "emulated_st7920".to_string(),
            chip_ctor(|config| {
                let chip: Arc<dyn DisplayChip> = Arc::new(EmulatedSt7920::new(config)?);
                Ok(Arc::new(DisplayChipAdapter(chip)) as Arc<dyn LcdChipApi>)
            }),
        ),
        (
            "hd44780".to_string(),
            chip_ctor(|config| {
                let chip: Arc<dyn Hd44780Api> = HD44780::new(config)?;
                Ok(Arc::new(Hd44780Chip(chip)) as Arc<dyn LcdChipApi>)
            }),
        ),
        (
            "uc1701".to_string(),
            chip_ctor(|config| {
                Ok(Arc::new(Uc1701Chip::new(Uc1701::from_config(config)?)) as Arc<dyn LcdChipApi>)
            }),
        ),
        (
            "ssd1306".to_string(),
            chip_ctor(|config| {
                Ok(
                    Arc::new(Ssd1306Chip::new(Ssd1306::from_config(config, 128, 0)?))
                        as Arc<dyn LcdChipApi>,
                )
            }),
        ),
        (
            "sh1106".to_string(),
            chip_ctor(|config| {
                Ok(Arc::new(Sh1106Chip::new(Sh1106::from_config(config)?)) as Arc<dyn LcdChipApi>)
            }),
        ),
        (
            "hd44780_spi".to_string(),
            chip_ctor(|config| {
                let chip: Arc<dyn Hd44780SpiApi> = Hd44780Spi::new(config)?;
                Ok(Arc::new(Hd44780SpiChip(chip)) as Arc<dyn LcdChipApi>)
            }),
        ),
        (
            "aip31068_spi".to_string(),
            chip_ctor(|config| {
                let chip: Arc<dyn Aip31068SpiApi> = Arc::new(Aip31068Spi::from_config(config)?);
                Ok(Arc::new(Aip31068SpiChip(chip)) as Arc<dyn LcdChipApi>)
            }),
        ),
    ]
}

// ---------------------------------------------------------------------------
// Template helpers shared by `DisplayTemplate` and `DisplayGroup`
// ---------------------------------------------------------------------------

fn template_error(e: &KlippyError, lineno: usize) -> TemplateError {
    TemplateError::new(e.class_name(), e.message().to_string(), Some(lineno))
}

fn type_error(message: String, lineno: usize) -> TemplateError {
    TemplateError::new("TypeError", message, Some(lineno))
}

fn value_error(message: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: message.to_string(),
    }
}

/// Bind the arguments of a Python call to `def fname(*params)` the way CPython
/// 3.8 does, with its `TypeError` texts. `self_args` is 1 for a bound method,
/// whose `self` CPython counts in "takes N positional arguments".
fn bind_arguments(
    fname: &str,
    params: &[&str],
    self_args: usize,
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<Vec<Value>, TemplateError> {
    if args.len() > params.len() {
        let takes = params.len() + self_args;
        let given = args.len() + self_args;
        return Err(type_error(
            format!(
                "{fname}() takes {takes} positional argument{} but {given} {} given",
                if takes == 1 { "" } else { "s" },
                if given == 1 { "was" } else { "were" }
            ),
            lineno,
        ));
    }
    let mut slots: Vec<Option<Value>> = vec![None; params.len()];
    for (slot, arg) in slots.iter_mut().zip(args) {
        *slot = Some(arg.clone());
    }
    for (key, value) in kwargs {
        match params.iter().position(|p| p == key) {
            None => {
                return Err(type_error(
                    format!("{fname}() got an unexpected keyword argument '{key}'"),
                    lineno,
                ))
            }
            Some(i) if slots[i].is_some() => {
                return Err(type_error(
                    format!("{fname}() got multiple values for argument '{key}'"),
                    lineno,
                ))
            }
            Some(i) => slots[i] = Some(value.clone()),
        }
    }
    let missing: Vec<String> = params
        .iter()
        .zip(&slots)
        .filter(|(_, slot)| slot.is_none())
        .map(|(name, _)| format!("'{name}'"))
        .collect();
    if !missing.is_empty() {
        let names = match missing.len() {
            1 => missing[0].clone(),
            2 => format!("{} and {}", missing[0], missing[1]),
            n => format!("{}, and {}", missing[..n - 1].join(", "), missing[n - 1]),
        };
        return Err(type_error(
            format!(
                "{fname}() missing {} required positional argument{}: {names}",
                missing.len(),
                if missing.len() == 1 { "" } else { "s" }
            ),
            lineno,
        ));
    }
    Ok(slots.into_iter().flatten().collect())
}

/// A template argument used as a Python `int` (`bool` is an `int`).
fn int_argument(value: &Value, lineno: usize) -> Result<i64, TemplateError> {
    match value {
        Value::Py(PyValue::Int(n)) => pyint_to_i64(n).ok_or_else(|| {
            TemplateError::new(
                "OverflowError",
                "Python int too large to convert to C long".to_string(),
                Some(lineno),
            )
        }),
        Value::Py(PyValue::Bool(b)) => Ok(i64::from(*b)),
        other => Err(type_error(
            format!(
                "'{}' object cannot be interpreted as an integer",
                other.type_name()
            ),
            lineno,
        )),
    }
}

/// A template argument used as a Python number in float arithmetic.
fn float_argument(value: &Value, lineno: usize) -> Result<f64, TemplateError> {
    match value {
        Value::Py(PyValue::Float(f)) => Ok(*f),
        Value::Py(PyValue::Int(n)) => n.to_f64_exact().ok_or_else(|| {
            TemplateError::new(
                "OverflowError",
                "int too large to convert to float".to_string(),
                Some(lineno),
            )
        }),
        Value::Py(PyValue::Bool(b)) => Ok(if *b { 1. } else { 0. }),
        other => Err(type_error(
            format!(
                "unsupported operand type(s) for -: 'float' and '{}'",
                other.type_name()
            ),
            lineno,
        )),
    }
}

// ---------------------------------------------------------------------------
// DisplayTemplate
// ---------------------------------------------------------------------------

/// The display template methods used by `output_pin`.
pub trait DisplayTemplateApi: Send + Sync + 'static {
    fn get_params(&self) -> IndexMap<String, PyValue>;
    fn render(
        &self,
        context: &IndexMap<String, Value>,
        kwargs: &IndexMap<String, PyValue>,
    ) -> KResult<String>;
}

/// Storage of `[display_template my_template]` config sections (`display.py:25`).
pub struct DisplayTemplate {
    name: String,
    params: IndexMap<String, PyValue>,
    template: TemplateWrapper,
}

impl DisplayTemplate {
    /// `DisplayTemplate.__init__(config)` (`display.py:26-42`).
    pub fn new(config: &Config) -> KResult<DisplayTemplate> {
        let printer = config.get_printer();
        let full_name = config.get_name();
        let name_parts: Vec<&str> = full_name.split(python_whitespace).filter(|s| !s.is_empty()).collect();
        if name_parts.len() != 2 {
            return Err(KlippyError::config(format!(
                "Section name '{full_name}' is not valid"
            )));
        }
        let name = name_parts[1].to_string();
        let mut params: IndexMap<String, PyValue> = IndexMap::new();
        for option in config.get_prefix_options("param_") {
            let raw = required_str(config, &option)?;
            match parse_literal(&raw) {
                Ok(value) => {
                    params.insert(option, value);
                }
                // Only `ValueError` is caught (`display.py:37`); every other
                // `ast.literal_eval` failure propagates.
                Err(e) if e.kind == "ValueError" => {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{full_name}' is not a valid literal"
                    )));
                }
                Err(e) => {
                    return Err(KlippyError::Internal {
                        class_name: e.kind.to_string(),
                        message: e.msg,
                    });
                }
            }
        }
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let template = gcode_macro.load_template(config, "text", None)?;
        Ok(DisplayTemplate {
            name,
            params,
            template,
        })
    }

    /// `self.name`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `get_params()` (`display.py:43`).
    pub fn get_params(&self) -> IndexMap<String, PyValue> {
        self.params.clone()
    }

    /// `render(context, **kwargs)` (`display.py:45-53`) with template values as
    /// keyword arguments, which may be jinja `Undefined` or callables.
    pub fn render_values(
        &self,
        context: &IndexMap<String, Value>,
        kwargs: IndexMap<String, Value>,
    ) -> KResult<String> {
        let mut params: IndexMap<String, Value> = self
            .params
            .iter()
            .map(|(k, v)| (k.clone(), Value::py(v.clone())))
            .collect();
        for (key, value) in kwargs {
            params.insert(key, value);
        }
        if params.len() != self.params.len() {
            return Err(KlippyError::command(format!(
                "Invalid parameter to display_template {}",
                self.name
            )));
        }
        let mut context = context.clone();
        for (key, value) in params {
            context.insert(key, value);
        }
        self.template.render(Some(context))
    }

    /// `render(context, **kwargs)` (`display.py:45-53`).
    pub fn render(
        &self,
        context: &IndexMap<String, Value>,
        kwargs: &IndexMap<String, PyValue>,
    ) -> KResult<String> {
        let kwargs = kwargs
            .iter()
            .map(|(k, v)| (k.clone(), Value::py(v.clone())))
            .collect();
        self.render_values(context, kwargs)
    }
}

impl DisplayTemplateApi for DisplayTemplate {
    fn get_params(&self) -> IndexMap<String, PyValue> {
        DisplayTemplate::get_params(self)
    }
    fn render(
        &self,
        context: &IndexMap<String, Value>,
        kwargs: &IndexMap<String, PyValue>,
    ) -> KResult<String> {
        DisplayTemplate::render(self, context, kwargs)
    }
}

// ---------------------------------------------------------------------------
// DisplayGroup
// ---------------------------------------------------------------------------

/// Storage of `[display_data my_group my_item]` sections (`display.py:56`),
/// one instance per group name.
pub struct DisplayGroup {
    data_items: Vec<(i64, i64, TemplateWrapper)>,
}

impl DisplayGroup {
    /// `DisplayGroup.__init__(config, name, data_configs)` (`display.py:57-77`).
    pub fn new(config: &Config, _name: &str, data_configs: &[Config]) -> KResult<DisplayGroup> {
        // Load and parse the position of display_data items.
        let mut items: Vec<(i64, i64, String)> = Vec::new();
        for c in data_configs {
            let pos = required_str(c, "position")?;
            let parsed = (|| -> Option<(i64, i64)> {
                let values: Vec<i64> = pos
                    .split(',')
                    .map(|v| {
                        int_from_string(v.trim_matches(python_whitespace), 10)
                            .ok()
                            .and_then(|n| pyint_to_i64(&n))
                    })
                    .collect::<Option<Vec<i64>>>()?;
                match values.as_slice() {
                    [row, col] => Some((*row, *col)),
                    _ => None,
                }
            })();
            let Some((row, col)) = parsed else {
                return Err(KlippyError::config(format!(
                    "Unable to parse 'position' in section '{}'",
                    c.get_name()
                )));
            };
            items.push((row, col, c.get_name()));
        }
        // Load all templates and store sorted by display position.
        let mut configs_by_name: IndexMap<String, &Config> = IndexMap::new();
        for c in data_configs {
            configs_by_name.insert(c.get_name(), c);
        }
        let printer = config.get_printer();
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        items.sort();
        let mut data_items = Vec::new();
        for (row, col, name) in items {
            let c = configs_by_name[&name];
            if !required_str(c, "text")?.is_empty() {
                let template = gcode_macro.load_template(c, "text", None)?;
                data_items.push((row, col, template));
            }
        }
        Ok(DisplayGroup { data_items })
    }

    /// `(row, col)` of every `data_items` entry, in draw order.
    pub fn item_positions(&self) -> Vec<(i64, i64)> {
        self.data_items.iter().map(|(r, c, _)| (*r, *c)).collect()
    }

    /// `DisplayGroup.show(display, templates, eventtime)` (`display.py:78-87`).
    pub fn show(
        &self,
        display: &Arc<PrinterLCD>,
        templates: &IndexMap<String, Arc<DisplayTemplate>>,
        eventtime: f64,
    ) -> KResult<()> {
        let first = self.data_items.first().ok_or_else(index_error)?;
        let mut context = first.2.create_template_context(Some(eventtime));

        let bar_display = Arc::clone(display);
        context.insert(
            "draw_progress_bar".to_string(),
            Value::Func(Rc::new(move |args: &[Value], kwargs: &[(String, Value)], lineno: usize| -> Result<Value, TemplateError> {
                let bound = bind_arguments(
                    "draw_progress_bar",
                    &["row", "col", "width", "value"],
                    1,
                    args,
                    kwargs,
                    lineno,
                )?;
                let value = float_argument(&bound[3], lineno)?;
                let width = int_argument(&bound[2], lineno)?;
                let col = int_argument(&bound[1], lineno)?;
                let row = int_argument(&bound[0], lineno)?;
                bar_display
                    .draw_progress_bar(row, col, width, value)
                    .map(|text| Value::py(PyValue::Str(text)))
                    .map_err(|e| template_error(&e, lineno))
            })),
        );

        // `render` sees the context that holds it, like the Python closure.
        let shared: Rc<RefCell<Option<IndexMap<String, Value>>>> = Rc::new(RefCell::new(None));
        let render_shared = Rc::clone(&shared);
        let render_templates = templates.clone();
        context.insert(
            "render".to_string(),
            Value::Func(Rc::new(move |args: &[Value], kwargs: &[(String, Value)], lineno: usize| -> Result<Value, TemplateError> {
                if args.len() > 1 {
                    return Err(type_error(
                        format!(
                            "render() takes 1 positional argument but {} were given",
                            args.len()
                        ),
                        lineno,
                    ));
                }
                let mut name = args.first().cloned();
                let mut template_kwargs: IndexMap<String, Value> = IndexMap::new();
                for (key, value) in kwargs {
                    if key == "name" {
                        if name.is_some() {
                            return Err(type_error(
                                "render() got multiple values for argument 'name'".to_string(),
                                lineno,
                            ));
                        }
                        name = Some(value.clone());
                    } else {
                        template_kwargs.insert(key.clone(), value.clone());
                    }
                }
                let Some(name) = name else {
                    return Err(type_error(
                        "render() missing 1 required positional argument: 'name'".to_string(),
                        lineno,
                    ));
                };
                // `templates[name]`.
                let template = match &name {
                    Value::Py(PyValue::Str(s)) => render_templates.get(s).cloned().ok_or_else(|| {
                        TemplateError::new("KeyError", pycompat::value::repr_key(s), Some(lineno))
                    })?,
                    Value::Py(v @ (PyValue::List(_) | PyValue::Dict(_))) => {
                        return Err(type_error(
                            format!("unhashable type: '{}'", pycompat::value::type_name(v)),
                            lineno,
                        ))
                    }
                    Value::Py(v) => {
                        return Err(TemplateError::new("KeyError", v.repr(), Some(lineno)))
                    }
                    other => {
                        return Err(TemplateError::new(
                            "KeyError",
                            other.type_name().to_string(),
                            Some(lineno),
                        ))
                    }
                };
                let context = render_shared
                    .borrow()
                    .as_ref()
                    .cloned()
                    .unwrap_or_default();
                template
                    .render_values(&context, template_kwargs)
                    .map(|text| Value::py(PyValue::Str(text)))
                    .map_err(|e| template_error(&e, lineno))
            })),
        );
        *shared.borrow_mut() = Some(context.clone());

        let result = (|| -> KResult<()> {
            for (row, col, template) in &self.data_items {
                let text = template.render(Some(context.clone()))?;
                display.draw_text(*row, *col, &text.replace('\n', ""), eventtime)?;
            }
            Ok(())
        })();
        // `context.clear()`: drop the cycle between the context and `render`.
        shared.borrow_mut().take();
        result
    }
}

// ---------------------------------------------------------------------------
// PrinterDisplayTemplate
// ---------------------------------------------------------------------------

/// `os.path.join(os.path.dirname(__file__), 'display.cfg')`.
fn default_display_config() -> String {
    let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir.ancestors().nth(3).unwrap_or(manifest_dir);
    repo_root
        .join("klippy/extras/display/display.cfg")
        .to_string_lossy()
        .into_owned()
}

/// Sections of `prefix` in `config`, then those of `dconfig` the main config does
/// not define (`display.py:125-128`).
fn prefix_sections_with_defaults(config: &Config, dconfig: &Config, prefix: &str) -> Vec<Config> {
    let mut main = config.get_prefix_sections(prefix);
    let main_names: Vec<String> = main.iter().map(|c| c.get_name()).collect();
    let defaults: Vec<Config> = dconfig
        .get_prefix_sections(prefix)
        .into_iter()
        .filter(|c| !main_names.contains(&c.get_name()))
        .collect();
    main.extend(defaults);
    main
}

/// The shared `display_template` printer object.
pub trait PrinterDisplayTemplateApi: Send + Sync + 'static {
    fn get_display_templates(&self) -> IndexMap<String, Arc<DisplayTemplate>>;
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>>;
    fn get_display_glyphs(&self) -> GlyphTable;
}

/// Global cache of `DisplayTemplate`, `DisplayGroup`, and glyphs (`display.py:90`).
pub struct PrinterDisplayTemplate {
    display_templates: IndexMap<String, Arc<DisplayTemplate>>,
    display_data_groups: IndexMap<String, Arc<DisplayGroup>>,
    display_glyphs: GlyphTable,
}

impl PrinterDisplayTemplate {
    /// `PrinterDisplayTemplate.__init__(config)` (`display.py:91-96`).
    pub fn new(config: &Config) -> KResult<PrinterDisplayTemplate> {
        let printer = config.get_printer();
        Self::load_config(&printer, config)
    }

    /// `PrinterDisplayTemplate.load_config(config)` (`display.py:115-166`).
    fn load_config(printer: &Printer, config: &Config) -> KResult<PrinterDisplayTemplate> {
        // Load default display config file.
        let pconfig =
            lookup_interface::<dyn PrinterConfigApi>(&**printer, "configfile", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let filename = default_display_config();
        let dconfig = match pconfig.read_config(&filename) {
            Ok(dconfig) => dconfig,
            Err(_) => {
                return Err(KlippyError::config(format!(
                    "Cannot load config '{filename}'"
                )))
            }
        };

        // Load display_template sections.
        let mut display_templates: IndexMap<String, Arc<DisplayTemplate>> = IndexMap::new();
        for c in prefix_sections_with_defaults(config, &dconfig, "display_template ") {
            let dt = DisplayTemplate::new(&c)?;
            display_templates.insert(dt.name().to_string(), Arc::new(dt));
        }

        // Load display_data sections.
        let mut groups: IndexMap<String, Vec<Config>> = IndexMap::new();
        for c in prefix_sections_with_defaults(config, &dconfig, "display_data ") {
            let full_name = c.get_name();
            let name_parts: Vec<&str> = full_name.split(python_whitespace).filter(|s| !s.is_empty()).collect();
            if name_parts.len() != 3 {
                return Err(KlippyError::config(format!(
                    "Section name '{full_name}' is not valid"
                )));
            }
            groups.entry(name_parts[1].to_string()).or_default().push(c);
        }
        let mut display_data_groups: IndexMap<String, Arc<DisplayGroup>> = IndexMap::new();
        for (group_name, data_configs) in &groups {
            let dg = DisplayGroup::new(config, group_name, data_configs)?;
            display_data_groups.insert(group_name.clone(), Arc::new(dg));
        }

        // Load display glyphs.
        let dg_prefix = "display_glyph ";
        let mut icons: GlyphTable = IndexMap::new();
        for dg in prefix_sections_with_defaults(config, &dconfig, dg_prefix) {
            let glyph_name = dg.get_name()[dg_prefix.len()..].to_string();
            let data = dg.get(
                "data",
                GetStrOpts {
                    default: Some(None),
                    note_valid: true,
                },
            )?;
            if let Some(data) = data {
                let idata = parse_glyph(&glyph_name, &data, 16, 16)?;
                let icon1 = idata.iter().map(|bits| (bits >> 8) & 0xff).collect();
                let icon2 = idata.iter().map(|bits| bits & 0xff).collect();
                icons.entry(glyph_name.clone()).or_default().icon16x16 = Some((icon1, icon2));
            }
            let data = dg.get(
                "hd44780_data",
                GetStrOpts {
                    default: Some(None),
                    note_valid: true,
                },
            )?;
            if let Some(data) = data {
                let slot = dg
                    .getint(
                        "hd44780_slot",
                        GetIntOpts {
                            default: None,
                            minval: Some(0),
                            maxval: Some(7),
                            note_valid: true,
                        },
                    )?
                    .ok_or_else(|| {
                        KlippyError::config(format!(
                            "Option 'hd44780_slot' in section '{}' must be specified",
                            dg.get_name()
                        ))
                    })?;
                let idata = parse_glyph(&glyph_name, &data, 5, 8)?;
                icons.entry(glyph_name.clone()).or_default().icon5x8 = Some((slot, idata));
            }
        }
        Ok(PrinterDisplayTemplate {
            display_templates,
            display_data_groups,
            display_glyphs: icons,
        })
    }

    /// `get_display_templates()` (`display.py:97`).
    pub fn get_display_templates(&self) -> IndexMap<String, Arc<DisplayTemplate>> {
        self.display_templates.clone()
    }

    /// `get_display_data_groups()` (`display.py:99`).
    pub fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        self.display_data_groups.clone()
    }

    /// `get_display_glyphs()` (`display.py:101`).
    pub fn get_display_glyphs(&self) -> GlyphTable {
        self.display_glyphs.clone()
    }
}

impl PrinterDisplayTemplateApi for PrinterDisplayTemplate {
    fn get_display_templates(&self) -> IndexMap<String, Arc<DisplayTemplate>> {
        PrinterDisplayTemplate::get_display_templates(self)
    }
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        PrinterDisplayTemplate::get_display_data_groups(self)
    }
    fn get_display_glyphs(&self) -> GlyphTable {
        PrinterDisplayTemplate::get_display_glyphs(self)
    }
}

klippy_api::printer_object!(PrinterDisplayTemplate; dyn PrinterDisplayTemplateApi);

/// `lookup_display_templates(config)` (`display.py:168-174`).
pub fn lookup_display_templates(config: &Config) -> KResult<Arc<dyn PrinterDisplayTemplateApi>> {
    let printer = config.get_printer();
    if let Some(templates) = lookup_interface::<dyn PrinterDisplayTemplateApi>(
        &*printer,
        "display_template",
        LookupDefault::ReturnNone,
    )? {
        return Ok(templates);
    }
    let dt = Arc::new(PrinterDisplayTemplate::new(config)?);
    printer.add_object("display_template", dt.clone() as ObjectRef)?;
    Ok(dt)
}

// ---------------------------------------------------------------------------
// PrinterLCD
// ---------------------------------------------------------------------------

/// What `menu.MenuManager` calls on the `display` object it is given
/// (`menu.py:641-655,702,1043`).
pub trait PrinterLCDApi: Send + Sync + 'static {
    /// `get_dimensions()`: `(columns, rows)`.
    fn get_dimensions(&self) -> (i64, i64);
    /// `request_redraw()`.
    fn request_redraw(&self);
    /// `draw_text(row, col, mixed_text, eventtime)`: the column after the text.
    fn draw_text(&self, row: i64, col: i64, mixed_text: &str, eventtime: f64) -> KResult<i64>;
}

struct LcdState {
    redraw_request_pending: bool,
    redraw_time: f64,
}

/// The `[display]` printer object (`display.py:176`).
pub struct PrinterLCD {
    printer: Printer,
    reactor: Reactor,
    lcd_chip: Arc<dyn LcdChipApi>,
    menu: GCell<Option<Arc<MenuManager>>>,
    display_templates: GCell<IndexMap<String, Arc<DisplayTemplate>>>,
    display_data_groups: GCell<IndexMap<String, Arc<DisplayGroup>>>,
    show_data_group: GCell<Option<Arc<DisplayGroup>>>,
    screen_update_timer: GCell<Option<Timer>>,
    state: GCell<LcdState>,
}

impl PrinterLCD {
    /// `PrinterLCD.__init__(config)` (`display.py:177-214`) with the `LCD_chips`
    /// table and the `menu.MenuManager` constructor of the reference.
    pub fn new(config: &Config) -> KResult<Arc<PrinterLCD>> {
        Self::with_chips(config, &lcd_chips())
    }

    /// Construct with a chip table, corresponding to the Python `LCD_chips` global.
    pub fn with_chips(
        config: &Config,
        chips: &[(String, ChipCtor)],
    ) -> KResult<Arc<PrinterLCD>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        // Load low-level lcd handler.
        let choices = PyValue::Dict(
            chips
                .iter()
                .map(|(name, _)| (PyValue::Str(name.clone()), PyValue::Str(name.clone())))
                .collect(),
        );
        let chosen = config.getchoice("lcd_type", &choices, GetChoiceOpts::default())?;
        let chip_ctor = match &chosen {
            PyValue::Str(chosen) => chips.iter().find(|(name, _)| name == chosen).map(|c| &c.1),
            _ => None,
        };
        let Some(chip_ctor) = chip_ctor else {
            return Err(KlippyError::config(format!(
                "Choice '{}' for option 'lcd_type' in section '{}' is not a valid choice",
                chosen.str(),
                config.get_name()
            )));
        };
        let lcd_chip = chip_ctor(config)?;
        let lcd = Arc::new(PrinterLCD {
            printer: printer.clone(),
            reactor: reactor.clone(),
            lcd_chip,
            menu: GCell::new(None),
            display_templates: GCell::new(IndexMap::new()),
            display_data_groups: GCell::new(IndexMap::new()),
            show_data_group: GCell::new(None),
            screen_update_timer: GCell::new(None),
            state: GCell::new(LcdState {
                redraw_request_pending: false,
                redraw_time: 0.,
            }),
        });
        // Load menu and display_status.
        let name = config.get_name();
        if name == "display" {
            // Only load menu for the primary display.
            let menu = MenuManager::new(config, lcd.clone() as Arc<dyn PrinterLCDApi>)?;
            lcd.menu.replace(Some(menu));
        }
        printer.load_object(config, "display_status", LookupDefault::Raise)?;
        // Configurable display.
        let templates = lookup_display_templates(config)?;
        lcd.display_templates.replace(templates.get_display_templates());
        lcd.display_data_groups
            .replace(templates.get_display_data_groups());
        lcd.lcd_chip.set_glyphs(&templates.get_display_glyphs());
        let mut dgroup = "_default_16x4".to_string();
        if lcd.lcd_chip.get_dimensions().0 == 20 {
            dgroup = "_default_20x4".to_string();
        }
        let dgroup = config
            .get(
                "display_group",
                GetStrOpts {
                    default: Some(Some(dgroup)),
                    note_valid: true,
                },
            )?
            .expect("display_group has a default");
        let group = lcd.display_data_groups.borrow().get(&dgroup).cloned();
        let Some(group) = group else {
            return Err(KlippyError::config(format!(
                "Unknown display_data group '{dgroup}'"
            )));
        };
        lcd.show_data_group.replace(Some(group));
        // Screen updating.
        let ready_lcd = Arc::clone(&lcd);
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params| {
                ready_lcd.handle_ready()?;
                Ok(PyValue::None)
            }),
        );
        let timer_lcd = Arc::clone(&lcd);
        let timer = reactor.register_timer(
            move |eventtime| match timer_lcd.screen_update_event(eventtime) {
                Ok(waketime) => waketime,
                Err(e) => {
                    // An exception out of a reactor timer ends `reactor.run()`;
                    // `klippy.run` logs it and shuts the printer down.
                    let msg = "Unhandled exception during run";
                    klippy_api::log::exception(
                        msg,
                        &format!("{}: {}", e.class_name(), e.message()),
                    );
                    timer_lcd
                        .printer
                        .invoke_async_shutdown(msg, PyValue::Dict(IndexMap::new()));
                    NEVER
                }
            },
            NEVER,
        );
        lcd.screen_update_timer.replace(Some(timer));
        // Register g-code commands.
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let cmd_lcd = Arc::clone(&lcd);
        gcode.register_mux_command(
            "SET_DISPLAY_GROUP",
            "DISPLAY",
            Some(&name),
            Box::new(move |gcmd: &GCmd| cmd_lcd.cmd_set_display_group(gcmd)),
            Some(SET_DISPLAY_GROUP_HELP),
        )?;
        if name == "display" {
            let cmd_lcd = Arc::clone(&lcd);
            gcode.register_mux_command(
                "SET_DISPLAY_GROUP",
                "DISPLAY",
                None,
                Box::new(move |gcmd: &GCmd| cmd_lcd.cmd_set_display_group(gcmd)),
                None,
            )?;
        }
        Ok(lcd)
    }

    /// `get_dimensions()` (`display.py:215`).
    pub fn get_dimensions(&self) -> (i64, i64) {
        self.lcd_chip.get_dimensions()
    }

    /// `handle_ready()` (`display.py:217-220`).
    pub fn handle_ready(&self) -> KResult<()> {
        self.lcd_chip.init()?;
        // Start screen update timer.
        if let Some(timer) = self.screen_update_timer.borrow().as_ref() {
            self.reactor.update_timer(timer, NOW);
        }
        Ok(())
    }

    /// `screen_update_event(eventtime)` (`display.py:222-241`): the next wake time.
    pub fn screen_update_event(self: &Arc<Self>, eventtime: f64) -> KResult<f64> {
        self.state.with_mut(|st| {
            if st.redraw_request_pending {
                st.redraw_request_pending = false;
                st.redraw_time = eventtime + REDRAW_MIN_TIME;
            }
        });
        self.lcd_chip.clear();
        // Update menu component.
        let menu: Option<Arc<MenuManager>> = self.menu.cloned();
        if let Some(menu) = menu {
            let ret: bool = MenuManager::screen_update_event(&menu, eventtime)?;
            if ret {
                self.lcd_chip.flush()?;
                return Ok(eventtime + REDRAW_TIME);
            }
        }
        // Update normal display.
        let group = self.show_data_group.cloned().ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "'PrinterLCD' object has no attribute 'show_data_group'".to_string(),
        })?;
        let templates = self.display_templates.cloned();
        if let Err(e) = group.show(self, &templates, eventtime) {
            klippy_api::log::exception(
                "Error during display screen update",
                &format!("{}: {}", e.class_name(), e.message()),
            );
        }
        self.lcd_chip.flush()?;
        // A redraw may have been requested while the screen was drawn.
        let st = self.state.borrow();
        if st.redraw_request_pending {
            return Ok(st.redraw_time);
        }
        Ok(eventtime + REDRAW_TIME)
    }

    /// `request_redraw()` (`display.py:242-246`).
    pub fn request_redraw(&self) {
        let redraw_time = {
            let mut st = self.state.borrow_mut();
            if st.redraw_request_pending {
                return;
            }
            st.redraw_request_pending = true;
            st.redraw_time
        };
        if let Some(timer) = self.screen_update_timer.borrow().as_ref() {
            self.reactor.update_timer(timer, redraw_time);
        }
    }

    /// `draw_text(row, col, mixed_text, eventtime)` (`display.py:247-257`).
    pub fn draw_text(
        &self,
        row: i64,
        col: i64,
        mixed_text: &str,
        _eventtime: f64,
    ) -> KResult<i64> {
        let mut pos = col;
        for (i, text) in mixed_text.split('~').enumerate() {
            if i & 1 == 0 {
                // Write text.
                self.lcd_chip.write_text(pos, row, text.as_bytes())?;
                pos = pos.wrapping_add(text.chars().count() as i64);
            } else {
                // Write glyph.
                pos = pos.wrapping_add(self.lcd_chip.write_glyph(pos, row, text)?);
            }
        }
        Ok(pos)
    }

    /// `draw_progress_bar(row, col, width, value)` (`display.py:258-264`).
    pub fn draw_progress_bar(
        &self,
        row: i64,
        col: i64,
        width: i64,
        value: f64,
    ) -> KResult<String> {
        // `pixels = -1 << int(width * 8 * (1. - value) + .5)`
        let bits = i128::from(width) * 8;
        let shift = trunc_checked(bits as f64 * (1. - value) + 0.5)?;
        if shift < 0 {
            return Err(value_error("negative shift count"));
        }
        // `pixels |= (1 << (width * 8 - 1)) | 1`
        let top = bits - 1;
        if top < 0 {
            return Err(value_error("negative shift count"));
        }
        for i in 0..width {
            // Bit `k` of `pixels` is set at and above the shift, at the top
            // bit and at bit 0 (two's complement of `-1 << shift`).
            let mut byte = 0u8;
            for b in 0..8i128 {
                let k = i128::from(i) * 8 + b;
                if k >= shift || k == top || k == 0 {
                    byte |= 1 << b;
                }
            }
            let mut data = [byte; 16];
            data[0] = 0xff;
            data[15] = 0xff;
            self.lcd_chip.write_graphics(
                col.wrapping_add(width).wrapping_sub(1).wrapping_sub(i),
                row,
                &data,
            )?;
        }
        Ok(String::new())
    }

    /// `cmd_SET_DISPLAY_GROUP(gcmd)` (`display.py:266-271`).
    pub fn cmd_set_display_group(&self, gcmd: &GCmd) -> KResult<()> {
        let group = match gcmd.get("GROUP", GetOpts::default())? {
            PyValue::Str(group) => group,
            other => other.str(),
        };
        let new_dg = self.display_data_groups.borrow().get(&group).cloned();
        let Some(new_dg) = new_dg else {
            return Err(KlippyError::command(format!(
                "Unknown display_data group '{group}'"
            )));
        };
        self.show_data_group.replace(Some(new_dg));
        Ok(())
    }

    /// `self.menu`, present only on the primary display.
    pub fn menu(&self) -> Option<Arc<MenuManager>> {
        self.menu.cloned()
    }

    /// `self.show_data_group`.
    pub fn show_data_group(&self) -> Option<Arc<DisplayGroup>> {
        self.show_data_group.cloned()
    }

    /// `self.display_data_groups`.
    pub fn display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        self.display_data_groups.cloned()
    }

    /// `self.display_templates`.
    pub fn display_templates(&self) -> IndexMap<String, Arc<DisplayTemplate>> {
        self.display_templates.cloned()
    }

    /// `self.redraw_request_pending`.
    pub fn redraw_request_pending(&self) -> bool {
        self.state.borrow().redraw_request_pending
    }

    /// `self.redraw_time`.
    pub fn redraw_time(&self) -> f64 {
        self.state.borrow().redraw_time
    }
}

/// `int(x)` of a float as a wide integer: `NaN` and infinities raise, integers
/// beyond `i128` saturate (they exceed every shift the display can express).
fn trunc_checked(x: f64) -> KResult<i128> {
    if x.is_nan() {
        return Err(value_error("cannot convert float NaN to integer"));
    }
    if x.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    let n = trunc_float_to_bigint(x);
    Ok(n.0
        .to_string()
        .parse::<i128>()
        .unwrap_or(if n.sign() < 0 { i128::MIN } else { i128::MAX }))
}

impl PrinterLCDApi for PrinterLCD {
    fn get_dimensions(&self) -> (i64, i64) {
        PrinterLCD::get_dimensions(self)
    }
    fn request_redraw(&self) {
        PrinterLCD::request_redraw(self);
    }
    fn draw_text(&self, row: i64, col: i64, mixed_text: &str, eventtime: f64) -> KResult<i64> {
        PrinterLCD::draw_text(self, row, col, mixed_text, eventtime)
    }
}

klippy_api::printer_object!(PrinterLCD; dyn PrinterLCDApi);

/// `load_config(config)` (`display.py:273-274`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterLCD::new(config)? as ObjectRef))
}

/// Module entry point for the `display` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
