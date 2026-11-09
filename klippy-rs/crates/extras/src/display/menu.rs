//! Port of the `display::menu` module (`klippy/extras/display/menu.py`).
//!
//! Scriptable LCD menu support: [`MenuManager`] owns the menu item tree,
//! the navigation stack and the G-code queue; [`MenuElement`] is the
//! scriptable element base with subclasses for containers, commands and
//! inputs. The Python module has no `load_config` entry point: the
//! manager is constructed directly by the `display` module
//! (`menu.MenuManager(config, self)`), so this file exposes the same
//! constructor and an empty [`module_factory`].
//!
//! Collaborators come from the modules that own the Python classes:
//! `PrinterLCD` (`crate::display::display::PrinterLCDApi`, the `display`
//! argument of `MenuManager(config, display)`), `PrinterGCodeMacro` and
//! `TemplateWrapper` (`crate::gcode_macro`), `VirtualSD`
//! (`crate::virtual_sdcard::VirtualSDApi`) and `MenuKeys`
//! (`crate::display::menu_keys`). Template contexts are the dicts of the
//! Python side: `IndexMap<String, Value>` with the `menu` entry holding a
//! `PyValue` dict.
//!
//! The template engine currently cannot call nested `PyValue::Host` values,
//! so the `menu.back()` and `menu.exit()` template actions require engine
//! support. Direct navigation methods and the list's back callback do not.

use crate::display::display::PrinterLCDApi;
use crate::display::menu_keys::{MenuKeyCallback, MenuKeys};
use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use crate::virtual_sdcard::VirtualSDApi;
use extras_macros_templates::gcode_macro::{parse_literal, Value};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetBoolOpts, GetFloatOpts, GetIntOpts, GetStrOpts, PrinterConfig, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeApi};
use klippy_api::printer::{LookupDefault, Printer};
use klippy_api::{lookup_interface, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::text::{bytes_repr, float_from_string, float_str, str_repr, PyError};
use pycompat::{PyInt, PyValue};
use pytext::pyre;
use std::sync::{Arc, Weak};

/// `TIMER_DELAY` (menu.py:685).
pub const TIMER_DELAY: f64 = 1.0;

/// The Jinja context dict (`manager.context` and the copies handed to
/// templates).
pub type TemplateContext = IndexMap<String, Value>;

/// `menu_items` repr used by the choice error (menu.py:676-683). The
/// classes are `<class 'extras.display.menu.X'>`: the package imports the
/// module as `extras.display.menu`.
const MENU_ITEMS_REPR: &str = "{'disabled': <class 'extras.display.menu.MenuDisabled'>, 'command': <class 'extras.display.menu.MenuCommand'>, 'input': <class 'extras.display.menu.MenuInput'>, 'list': <class 'extras.display.menu.MenuList'>, 'vsdlist': <class 'extras.display.menu.MenuVSDList'>}";

/// Host id reserved for the `menu.back` action in the default context.
pub const HOST_BACK: u64 = 1;
/// Host id reserved for the `menu.exit` action in the default context.
pub const HOST_EXIT: u64 = 2;

/// `class error(Exception)` (menu.py:16).
fn menu_error(message: String) -> KlippyError {
    py_exception("error", message)
}

/// A Python exception the callers do not catch by class.
fn py_exception(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: message.into(),
    }
}

/// `list.insert(index, value)`: negative and out-of-range indexes clamp.
fn list_insert<T>(list: &mut Vec<T>, index: i64, value: T) {
    let len = list.len() as i64;
    let at = if index < 0 {
        (len + index).max(0)
    } else {
        index.min(len)
    };
    list.insert(at as usize, value);
}

// ---------------------------------------------------------------------------
// Literals.
// ---------------------------------------------------------------------------

/// A Python exception from `ast.literal_eval` or `float()` as the error the
/// callers see: the parser's `str(e)`, except that a bare name reports the
/// `_ast.Name` node.
fn literal_error(error: &PyError, source: &str) -> KlippyError {
    let message = if error.kind == "ValueError" && error.msg.starts_with("malformed node or string")
    {
        let t = source.trim();
        let bare_name = !t.is_empty()
            && !matches!(t, "True" | "False" | "None")
            && t.chars().next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
            && t.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
        if bare_name {
            "malformed node or string: <_ast.Name object at 0x0>".to_string()
        } else {
            error.msg.clone()
        }
    } else {
        error.msg.clone()
    };
    py_exception(error.kind, message)
}

/// `ast.literal_eval(text)`.
fn literal_eval(text: &str) -> KResult<PyValue> {
    parse_literal(text).map_err(|e| literal_error(&e, text))
}

/// `float(value)` (`PyNumber_Float`).
fn float_of(value: &PyValue) -> KResult<f64> {
    match value {
        PyValue::Float(f) => Ok(*f),
        PyValue::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        PyValue::Int(n) => match n.to_f64_exact() {
            Some(f) if f.is_finite() => Ok(f),
            _ => Err(py_exception(
                "OverflowError",
                "int too large to convert to float",
            )),
        },
        PyValue::Str(s) => float_from_string(s).map_err(|e| literal_error(&e, s)),
        PyValue::Bytes(b) => {
            let text = String::from_utf8_lossy(b).into_owned();
            float_from_string(&text).map_err(|_| {
                py_exception(
                    "ValueError",
                    format!("could not convert string to float: {}", bytes_repr(b)),
                )
            })
        }
        other => Err(py_exception(
            "TypeError",
            format!(
                "float() argument must be a string or a number, not '{}'",
                pycompat::value::type_name(other)
            ),
        )),
    }
}

/// `float(ast.literal_eval(rendered))` for input values (menu.py:501-525):
/// a `ValueError` is logged and yields `None`; every other exception
/// propagates, like the Python `except ValueError`.
fn literal_float(rendered: &str, log_message: &str) -> KResult<Option<f64>> {
    match literal_eval(rendered).and_then(|value| float_of(&value)) {
        Ok(value) => Ok(Some(value)),
        Err(e) if e.class_name() == "ValueError" => {
            klippy_api::log::exception(
                log_message,
                &format!("{}: {}", e.class_name(), e.message()),
            );
            Ok(None)
        }
        Err(e) => Err(e),
    }
}

/// `float(None)`: the bound or value was never a number.
fn float_of_option(value: Option<f64>) -> KResult<f64> {
    value.ok_or_else(|| {
        py_exception(
            "TypeError",
            "float() argument must be a string or a number, not 'NoneType'",
        )
    })
}

/// Type name in the operand errors of `None` versus `float` arithmetic.
fn float_or_none_type(value: Option<f64>) -> &'static str {
    if value.is_some() {
        "float"
    } else {
        "NoneType"
    }
}

/// `max(a, b)` of two floats where either may be `None`: Python keeps `a`
/// unless `b > a`, and compares with `>` (`None` raises `TypeError`).
fn py_max_float(a: Option<f64>, b: Option<f64>) -> KResult<f64> {
    match (a, b) {
        (Some(a), Some(b)) => Ok(if b > a { b } else { a }),
        _ => Err(py_exception(
            "TypeError",
            format!(
                "'>' not supported between instances of '{}' and '{}'",
                float_or_none_type(b),
                float_or_none_type(a)
            ),
        )),
    }
}

/// `min(a, b)`: Python keeps `a` unless `b < a`.
fn py_min_float(a: Option<f64>, b: f64) -> KResult<f64> {
    match a {
        Some(a) => Ok(if b < a { b } else { a }),
        None => Err(py_exception(
            "TypeError",
            "'<' not supported between instances of 'float' and 'NoneType'",
        )),
    }
}

// ---------------------------------------------------------------------------
// Scripts and element kinds.
// ---------------------------------------------------------------------------

/// Callable script `(element, context) -> str | None` (`gcode=_cb` of
/// `menuitem_from`).
pub type ScriptCallback =
    Arc<dyn Fn(&Arc<MenuElement>, &TemplateContext) -> KResult<Option<String>> + Send + Sync>;

/// `populate` keyword callback (`_populate_cb`).
pub type PopulateCallback = Arc<dyn Fn(&Arc<MenuElement>) -> KResult<()> + Send + Sync>;

/// A value stored in `self._scripts` (menu.py:56): a static string, a
/// template, a callable, or `None`.
#[derive(Clone)]
pub enum ScriptValue {
    /// Static string (or `None` from a dict config value).
    Static(Option<String>),
    /// Loaded Jinja template.
    Template(Arc<TemplateWrapper>),
    /// Callable script.
    Callback(ScriptCallback),
}

/// Element class (`type(self)` checks and `isinstance` calls in Python).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ElementKind {
    /// `MenuDisabled`.
    Disabled,
    /// `MenuCommand`.
    Command,
    /// `MenuInput`.
    Input,
    /// `MenuList`.
    List,
    /// `MenuVSDList`.
    VsdList,
}

impl ElementKind {
    /// `isinstance(item, MenuContainer)`.
    fn is_container(self) -> bool {
        matches!(self, ElementKind::List | ElementKind::VsdList)
    }
    /// `isinstance(item, MenuList)`.
    fn is_list(self) -> bool {
        self.is_container()
    }
    /// `isinstance(item, MenuCommand)` (`MenuInput` is a subclass).
    fn is_command(self) -> bool {
        matches!(self, ElementKind::Command | ElementKind::Input)
    }
}

/// `menu_items[type]` (menu.py:676-683).
fn kind_of(type_name: &str) -> KResult<ElementKind> {
    match type_name {
        "disabled" => Ok(ElementKind::Disabled),
        "command" => Ok(ElementKind::Command),
        "input" => Ok(ElementKind::Input),
        "list" => Ok(ElementKind::List),
        "vsdlist" => Ok(ElementKind::VsdList),
        _ => Err(menu_error(format!(
            "Choice '{type_name}' for option '{MENU_ITEMS_REPR}' is not a valid choice"
        ))),
    }
}

/// `menuitem_from(type, **kwargs)` keyword arguments (menu.py:961-965).
#[derive(Clone, Default)]
pub struct MenuItemKwargs {
    /// `index`.
    pub index: Option<i64>,
    /// `enable`.
    pub enable: Option<bool>,
    /// `name`.
    pub name: Option<String>,
    /// `ns`.
    pub ns: Option<String>,
    /// `populate` callback (`_populate_cb`).
    pub populate: Option<PopulateCallback>,
    /// `input`.
    pub input: Option<f64>,
    /// `input_min`.
    pub input_min: Option<f64>,
    /// `input_max`.
    pub input_max: Option<f64>,
    /// `input_step`.
    pub input_step: Option<f64>,
    /// `realtime`.
    pub realtime: Option<bool>,
    /// `gcode` script.
    pub gcode: Option<ScriptValue>,
}

/// `run_script(name, **kwargs)` keyword arguments (menu.py:201-220).
#[derive(Clone, Default)]
pub struct RunScriptOpts {
    /// `event`.
    pub event: Option<String>,
    /// `context` overlay.
    pub context: Option<TemplateContext>,
    /// `render_only`.
    pub render_only: bool,
}

// ---------------------------------------------------------------------------
// Element state.
// ---------------------------------------------------------------------------

/// Shared per-element state (`_index`, `_enable`, ... in Python; the
/// name-mangled `__selected`/`__scroll_*` land here too).
struct ElementInner {
    cursor: char,
    index: Option<i64>,
    enable: bool,
    enable_tpl: Option<Arc<TemplateWrapper>>,
    name: Option<String>,
    name_tpl: Option<Arc<TemplateWrapper>>,
    ns: String,
    last_heartbeat: Option<f64>,
    scripts: IndexMap<String, ScriptValue>,
    scroll_pos: Option<usize>,
    scroll_pending: Option<bool>,
    scroll_next: f64,
    // MenuInput state.
    input: Option<f64>,
    input_min: Option<f64>,
    input_max: Option<f64>,
    input_step: f64,
    realtime: bool,
    input_tpl: Option<Arc<TemplateWrapper>>,
    input_min_tpl: Option<Arc<TemplateWrapper>>,
    input_max_tpl: Option<Arc<TemplateWrapper>>,
    input_value: Option<f64>,
    input_dirty: bool,
    input_last_change: Option<f64>,
    // MenuContainer state.
    populate_cb: Option<PopulateCallback>,
    selected: Option<i64>,
    allitems: Vec<(Arc<MenuElement>, String)>,
    items: Vec<Arc<MenuElement>>,
    names: Vec<String>,
    parents: Vec<Arc<MenuElement>>,
    viewport_top: i64,
    back_item: Option<Arc<MenuElement>>,
}

impl Default for ElementInner {
    fn default() -> Self {
        ElementInner {
            cursor: '>',
            index: None,
            enable: true,
            enable_tpl: None,
            name: None,
            name_tpl: None,
            ns: String::new(),
            last_heartbeat: None,
            scripts: IndexMap::new(),
            scroll_pos: None,
            scroll_pending: Some(false),
            scroll_next: 0.0,
            input: None,
            input_min: Some(-999999.0),
            input_max: Some(999999.0),
            input_step: 1.0,
            realtime: false,
            input_tpl: None,
            input_min_tpl: None,
            input_max_tpl: None,
            input_value: None,
            input_dirty: false,
            input_last_change: None,
            populate_cb: None,
            selected: None,
            allitems: Vec::new(),
            items: Vec::new(),
            names: Vec::new(),
            parents: Vec::new(),
            viewport_top: 0,
            back_item: None,
        }
    }
}

/// A menu element (`MenuElement` and subclasses, menu.py:21-683).
pub struct MenuElement {
    manager: Weak<MenuManager>,
    self_weak: Weak<MenuElement>,
    kind: ElementKind,
    inner: GCell<ElementInner>,
}

/// Reference for `lookup_item` / `index_of` (menu.py:270-301): a name
/// or a live element.
#[derive(Clone)]
pub enum LookupRef {
    /// A menu path string.
    Named(String),
    /// A live element.
    Element(Arc<MenuElement>),
}

/// Which input bound a template evaluation is for.
#[derive(Clone, Copy)]
enum InputBound {
    Min,
    Max,
}

impl MenuElement {
    fn manager(&self) -> Arc<MenuManager> {
        self.manager
            .upgrade()
            .expect("menu manager is alive while elements exist")
    }

    /// Handle to this element as `Arc<MenuElement>` (`self` in Python,
    /// for identity checks and event payloads).
    fn element_handle(&self) -> Arc<MenuElement> {
        self.self_weak
            .upgrade()
            .expect("menu element is alive while its methods run")
    }

    /// `cursor` property (menu.py:222-224).
    pub fn cursor(&self) -> String {
        self.inner.borrow().cursor.to_string()
    }

    /// `index` property (menu.py:230-232).
    pub fn index(&self) -> Option<i64> {
        self.inner.borrow().index
    }

    /// `selected` property (menu.py:412-414).
    pub fn selected(&self) -> Option<i64> {
        self.inner.borrow().selected
    }

    /// `isinstance(item, MenuContainer)`.
    pub fn is_container(&self) -> bool {
        self.kind.is_container()
    }

    /// `isinstance(item, MenuList)`.
    pub fn is_list(&self) -> bool {
        self.kind.is_list()
    }

    /// `isinstance(item, MenuInput)`.
    pub fn is_input(&self) -> bool {
        self.kind == ElementKind::Input
    }

    /// `isinstance(item, MenuCommand)`.
    pub fn is_command(&self) -> bool {
        self.kind.is_command()
    }

    /// `init()` hook (menu.py:60-62, overridden by `MenuContainer` and
    /// `MenuInput`): resets the recursive guard and the editing state.
    pub fn init(&self) {
        let mut inner = self.inner.borrow_mut();
        if self.kind.is_container() {
            inner.parents.clear();
        }
        if self.kind == ElementKind::Input {
            inner.input_dirty = false;
            inner.input_last_change = None;
            inner.input_value = None;
        }
    }

    /// `_render_name` (menu.py:64-68).
    fn render_name_raw(&self) -> KResult<String> {
        let (name_tpl, name) = {
            let inner = self.inner.borrow();
            (inner.name_tpl.clone(), inner.name.clone())
        };
        if let Some(tpl) = name_tpl {
            let context = self.get_context(None)?;
            let rendered = template_render(&tpl, &context)?;
            Ok(MenuManager::asflat(&rendered))
        } else {
            // `asflat(None)`: `aslatin` falls through to the undefined
            // `unicode` name, which is a NameError on Python 3.
            match name {
                Some(name) => Ok(MenuManager::asflat(&name)),
                None => Err(py_exception(
                    "NameError",
                    "name 'unicode' is not defined",
                )),
            }
        }
    }

    /// `is_editing` (menu.py:83-85; overridden by `MenuContainer` and
    /// `MenuInput`).
    pub fn is_editing(&self) -> bool {
        if self.kind == ElementKind::Input {
            return self.inner.borrow().input_value.is_some();
        }
        if self.kind.is_container() {
            let items = self.inner.borrow().items.clone();
            // The list comprehension asks every item (menu.py:263).
            let mut editing = false;
            for item in &items {
                editing = item.is_editing() || editing;
            }
            return editing;
        }
        false
    }

    /// `is_scrollable` (menu.py:87-90, overridden by `MenuInput`).
    pub fn is_scrollable(&self) -> bool {
        self.kind != ElementKind::Input
    }

    /// `is_enabled` (menu.py:91-94, overridden by `MenuDisabled`; the
    /// `MenuInput` override only skips the input evaluation).
    pub fn is_enabled(&self) -> KResult<bool> {
        if self.kind == ElementKind::Disabled {
            return Ok(false);
        }
        let context = self.element_context(None)?;
        self.eval_enable(&context)
    }

    /// `start_editing` (menu.py:96-98; `MenuInput` override).
    pub fn start_editing(&self) -> KResult<()> {
        if self.kind == ElementKind::Input {
            if self.is_editing() {
                return Ok(());
            }
            return self.init_value();
        }
        Ok(())
    }

    /// `stop_editing` (menu.py:100-102; `MenuContainer` and `MenuInput`
    /// overrides).
    pub fn stop_editing(&self) -> KResult<()> {
        if self.kind == ElementKind::Input {
            if !self.is_editing() {
                return Ok(());
            }
            self.reset_value();
            return Ok(());
        }
        if self.kind.is_container() {
            let items = self.inner.borrow().items.clone();
            for item in &items {
                if item.is_editing() {
                    item.stop_editing()?;
                }
            }
        }
        Ok(())
    }

    /// `MenuElement.get_context` (menu.py:104-111): the manager context
    /// with this element's `ns`.
    fn element_context(&self, cxt: Option<&TemplateContext>) -> KResult<TemplateContext> {
        let manager = self.manager();
        let mut context = manager.get_context(cxt);
        manager.update_menu(
            &mut context,
            cxt,
            &[("ns", PyValue::Str(self.get_ns(".")))],
        )?;
        Ok(context)
    }

    /// `get_context` (menu.py:104-111; `MenuInput` override adds `input`,
    /// menu.py:486-493).
    pub fn get_context(&self, cxt: Option<&TemplateContext>) -> KResult<TemplateContext> {
        let mut context = self.element_context(cxt)?;
        if self.kind == ElementKind::Input {
            let stored = self.inner.borrow().input_value;
            let value = match stored {
                None => self.eval_value(&context)?,
                stored => stored,
            };
            self.manager().update_menu(
                &mut context,
                cxt,
                &[("input", value.map(PyValue::Float).unwrap_or(PyValue::None))],
            )?;
        }
        Ok(context)
    }

    /// `eval_enable` (menu.py:113-116).
    pub fn eval_enable(&self, context: &TemplateContext) -> KResult<bool> {
        let (tpl, enable) = {
            let inner = self.inner.borrow();
            (inner.enable_tpl.clone(), inner.enable)
        };
        match tpl {
            Some(tpl) => {
                let rendered = template_render(&tpl, context)?;
                Ok(literal_eval(&rendered)?.truthy())
            }
            None => Ok(enable),
        }
    }

    /// `select` (menu.py:118-120).
    pub fn select(&self) {
        let mut inner = self.inner.borrow_mut();
        inner.scroll_pos = None;
        inner.scroll_pending = Some(false);
    }

    /// `heartbeat` (menu.py:122-127; `MenuInput` override at 475-484).
    pub fn heartbeat(&self, eventtime: f64) -> KResult<()> {
        let tick = {
            let mut inner = self.inner.borrow_mut();
            inner.last_heartbeat = Some(eventtime);
            if eventtime >= inner.scroll_next {
                inner.scroll_next = eventtime + 0.5;
                true
            } else {
                false
            }
        };
        if tick && !self.is_editing() {
            self.update_scroller();
        }
        if self.kind == ElementKind::Input {
            let (dirty, last_change, has_value) = {
                let inner = self.inner.borrow();
                (
                    inner.input_dirty,
                    inner.input_last_change,
                    inner.input_value.is_some(),
                )
            };
            if let (true, Some(last_change), true) = (dirty, last_change, has_value) {
                if (eventtime - last_change) > 0.250 {
                    if self.inner.borrow().realtime {
                        self.run_script(
                            "gcode",
                            &RunScriptOpts {
                                event: Some("change".to_string()),
                                ..Default::default()
                            },
                        )?;
                        self.run_script("change", &RunScriptOpts::default())?;
                    }
                    self.inner.borrow_mut().input_dirty = false;
                }
            }
        }
        Ok(())
    }

    /// `__update_scroller` (menu.py:129-138).
    fn update_scroller(&self) {
        let mut inner = self.inner.borrow_mut();
        match (inner.scroll_pos, inner.scroll_pending) {
            (None, Some(true)) => inner.scroll_pos = Some(0),
            (Some(pos), Some(true)) => {
                inner.scroll_pos = Some(pos + 1);
                inner.scroll_pending = Some(false);
            }
            (_, Some(false)) => {}
            (_, None) => {
                inner.scroll_pos = None;
                inner.scroll_pending = Some(false);
            }
        }
    }

    /// `need_scroller` (menu.py:144-152).
    pub fn need_scroller(&self, value: Option<bool>) {
        self.inner.borrow_mut().scroll_pending = value;
    }

    /// `__slice_name` (menu.py:154-161).
    pub fn slice_name(&self, name: &str, index: usize) -> String {
        let pattern = pyre::compile("(\\~.*?\\~)").expect("tilde pattern is shipped");
        let parts = pattern.split(name);
        let mut chunks: Vec<String> = Vec::new();
        for (i, text) in parts.iter().enumerate() {
            if i % 2 == 0 {
                for ch in text.chars() {
                    chunks.push(ch.to_string());
                }
            } else {
                chunks.push(text.clone());
            }
        }
        let skip = index.min(chunks.len());
        chunks[skip..].join("")
    }

    /// `render_name` (menu.py:163-169).
    pub fn render_name(&self, selected: bool) -> KResult<String> {
        let name = self.render_name_raw()?;
        let scroll_pos = self.inner.borrow().scroll_pos;
        match scroll_pos {
            Some(pos) if selected => Ok(self.slice_name(&name, pos)),
            _ => {
                let mut inner = self.inner.borrow_mut();
                inner.scroll_pos = None;
                inner.scroll_pending = Some(false);
                Ok(name)
            }
        }
    }

    /// `get_ns` (menu.py:171-178).
    pub fn get_ns(&self, name: &str) -> String {
        let raw = name.trim().to_string();
        let ns = self.inner.borrow().ns.clone();
        let resolved = if let Some(rest) = raw.strip_prefix("..") {
            let mut parts: Vec<&str> = ns.split(' ').collect();
            parts.pop();
            [parts.join(" "), rest.to_string()].join(" ")
        } else if let Some(rest) = raw.strip_prefix('.') {
            [ns.clone(), rest.to_string()].join(" ")
        } else {
            raw.clone()
        };
        resolved.trim().to_string()
    }

    /// `send_event` (menu.py:180-182).
    pub fn send_event(&self, event: &str, args: &[EventArg]) -> KResult<Vec<PyValue>> {
        let full = format!("{}:{}", self.get_ns("."), event);
        self.manager().send_event(&full, args)
    }

    /// `get_script` (menu.py:184-187). Returns `None` for unknown names.
    pub fn get_script(&self, name: &str) -> Option<ScriptValue> {
        self.inner.borrow().scripts.get(name).cloned()
    }

    /// `_load_script` for a config (menu.py:70-81, `config` branch).
    fn load_script(&self, config: &Config, name: &str, option: Option<&str>) -> KResult<()> {
        // `option = option or name`.
        let option = match option {
            Some(option) if !option.is_empty() => option,
            _ => name,
        };
        self.note_script_declaration(name);
        let tpl = self
            .manager()
            .gcode_macro
            .load_template(config, option, Some(""))?;
        self.inner
            .borrow_mut()
            .scripts
            .insert(name.to_string(), ScriptValue::Template(Arc::new(tpl)));
        Ok(())
    }

    /// `_load_script` for a kwargs dict (menu.py:70-78, `dict` branch).
    fn load_script_value(&self, name: &str, script: ScriptValue) {
        self.note_script_declaration(name);
        self.inner.borrow_mut().scripts.insert(name.to_string(), script);
    }

    /// The `logging.info` of `_load_script` when a script is redeclared.
    fn note_script_declaration(&self, name: &str) {
        if self.inner.borrow().scripts.contains_key(name) {
            klippy_api::log::info(&format!(
                "Declaration of '{name}' hides previous script declaration"
            ));
        }
    }

    /// `_run_script` (menu.py:189-199).
    fn run_one_script(&self, name: &str, context: &TemplateContext) -> KResult<Option<String>> {
        let script = match self.inner.borrow().scripts.get(name).cloned() {
            Some(s) => s,
            None => return Ok(None),
        };
        match script {
            ScriptValue::Template(tpl) => template_render(&tpl, context).map(Some),
            ScriptValue::Callback(cb) => cb(&self.element_handle(), context),
            ScriptValue::Static(text) => Ok(text),
        }
    }

    /// `run_script` (menu.py:201-220). The result is `None` when a callable
    /// script returns `None` or a script entry holds `None`.
    pub fn run_script(&self, name: &str, opts: &RunScriptOpts) -> KResult<Option<String>> {
        let mut result = Some(String::new());
        let known = self.inner.borrow().scripts.contains_key(name);
        if known {
            let mut context = self.get_context(opts.context.as_ref())?;
            // `event or name`: an empty event falls back to the script name.
            let event = match opts.event.as_deref() {
                Some(event) if !event.is_empty() => event.to_string(),
                _ => name.to_string(),
            };
            self.manager().update_menu(
                &mut context,
                opts.context.as_ref(),
                &[("event", PyValue::Str(event))],
            )?;
            result = self.run_one_script(name, &context)?;
        }
        if !opts.render_only {
            self.manager().queue_gcode(result.as_deref().unwrap_or(""));
            // `handle_script_click` exists on `MenuInput` only (menu.py:574-578).
            if self.kind == ElementKind::Input && name == "click" {
                if !self.is_editing() {
                    self.start_editing()?;
                } else {
                    self.stop_editing()?;
                }
            }
        }
        Ok(result)
    }

    // -- MenuInput (menu.py:431-579) --------------------------------------

    /// `_eval_value` (menu.py:517-524): logs and yields `None` on `ValueError`.
    pub fn eval_value(&self, context: &TemplateContext) -> KResult<Option<f64>> {
        let (tpl, raw) = {
            let inner = self.inner.borrow();
            (inner.input_tpl.clone(), inner.input)
        };
        match tpl {
            Some(tpl) => {
                let rendered = template_render(&tpl, context)?;
                literal_float(&rendered, "Input value evaluation error")
            }
            None => float_of_option(raw).map(Some),
        }
    }

    /// `_eval_min` / `_eval_max` (menu.py:499-515).
    fn eval_input_bound(
        &self,
        context: &TemplateContext,
        bound: InputBound,
    ) -> KResult<Option<f64>> {
        let (tpl, fallback) = {
            let inner = self.inner.borrow();
            match bound {
                InputBound::Min => (inner.input_min_tpl.clone(), inner.input_min),
                InputBound::Max => (inner.input_max_tpl.clone(), inner.input_max),
            }
        };
        match tpl {
            Some(tpl) => {
                let rendered = template_render(&tpl, context)?;
                let message = match bound {
                    InputBound::Min => "Input min value evaluation error",
                    InputBound::Max => "Input max value evaluation error",
                };
                literal_float(&rendered, message)
            }
            None => float_of_option(fallback).map(Some),
        }
    }

    /// `_value_changed` (menu.py:526-528).
    fn value_changed(&self) {
        let mut inner = self.inner.borrow_mut();
        inner.input_last_change = inner.last_heartbeat;
        inner.input_dirty = true;
    }

    /// `_init_value` (menu.py:530-537). The bounds are stored as they are
    /// evaluated, so a failure leaves them set (possibly `None`).
    fn init_value(&self) -> KResult<()> {
        let context = self.element_context(None)?;
        self.inner.borrow_mut().input_value = None;
        let input_min = self.eval_input_bound(&context, InputBound::Min)?;
        self.inner.borrow_mut().input_min = input_min;
        let input_max = self.eval_input_bound(&context, InputBound::Max)?;
        self.inner.borrow_mut().input_max = input_max;
        let value = self.eval_value(&context)?;
        let lower = py_max_float(input_min, value)?;
        let clamped = py_min_float(input_max, lower)?;
        self.inner.borrow_mut().input_value = Some(clamped);
        self.value_changed();
        Ok(())
    }

    /// `_reset_value` (menu.py:539-540).
    fn reset_value(&self) {
        self.inner.borrow_mut().input_value = None;
    }

    /// `_get_input_step` (menu.py:542-545). The span test only runs for a
    /// fast rate, like Python's short-circuiting `and`.
    fn get_input_step(&self, fast_rate: bool) -> KResult<f64> {
        let (step, min, max) = {
            let inner = self.inner.borrow();
            (inner.input_step, inner.input_min, inner.input_max)
        };
        if !fast_rate {
            return Ok(step);
        }
        let span = match (max, min) {
            (Some(max), Some(min)) => max - min,
            (max, min) => {
                return Err(py_exception(
                    "TypeError",
                    format!(
                        "unsupported operand type(s) for -: '{}' and '{}'",
                        float_or_none_type(max),
                        float_or_none_type(min)
                    ),
                ))
            }
        };
        if step == 0.0 {
            return Err(py_exception("ZeroDivisionError", "float division by zero"));
        }
        Ok(if span / step > 100.0 { 10.0 * step } else { step })
    }

    /// `inc_value` (menu.py:547-558).
    pub fn inc_value(&self, fast_rate: bool) -> KResult<()> {
        self.step_value(fast_rate, true)
    }

    /// `dec_value` (menu.py:560-571).
    pub fn dec_value(&self, fast_rate: bool) -> KResult<()> {
        self.step_value(fast_rate, false)
    }

    /// Shared body of `inc_value` / `dec_value`. The stepped value is stored
    /// before the clamp runs, so a failing clamp keeps it.
    fn step_value(&self, fast_rate: bool, increase: bool) -> KResult<()> {
        let last_value = self.inner.borrow().input_value;
        let Some(current) = last_value else {
            return Ok(());
        };
        let step = self.get_input_step(fast_rate)?.abs();
        let moved = if increase { current + step } else { current - step };
        self.inner.borrow_mut().input_value = Some(moved);
        let (min, max) = {
            let inner = self.inner.borrow();
            (inner.input_min, inner.input_max)
        };
        let lower = py_max_float(min, Some(moved))?;
        let clamped = py_min_float(max, lower)?;
        self.inner.borrow_mut().input_value = Some(clamped);
        if last_value != Some(clamped) {
            self.value_changed();
        }
        Ok(())
    }

    // -- MenuContainer (menu.py:235-415) -----------------------------------

    /// `_names_aslist` (menu.py:254-256; `MenuList` override at 592-593).
    fn names_as_list(&self) -> Vec<String> {
        if self.kind.is_list() {
            self.manager().lookup_children(&self.get_ns("."))
        } else {
            Vec::new()
        }
    }

    /// `lookup_item` (menu.py:270-277).
    pub fn lookup_item(&self, item: &LookupRef) -> KResult<Option<(Arc<MenuElement>, String)>> {
        match item {
            LookupRef::Named(name) => {
                let trimmed = name.trim().to_string();
                let ns = self.get_ns(&trimmed);
                let found = self.manager().lookup_menuitem(Some(&ns))?;
                Ok(found.map(|item| (item, trimmed)))
            }
            LookupRef::Element(element) => Ok(Some((element.clone(), element.get_ns(".")))),
        }
    }

    /// `_index_of` (menu.py:283-293).
    pub fn index_of_item(&self, item: &LookupRef) -> Option<i64> {
        let inner = self.inner.borrow();
        match item {
            LookupRef::Named(name) => {
                let trimmed = name.trim();
                inner
                    .names
                    .iter()
                    .position(|n| n == trimmed)
                    .map(|i| i as i64)
            }
            LookupRef::Element(element) => inner
                .items
                .iter()
                .position(|e| Arc::ptr_eq(e, element))
                .map(|i| i as i64),
        }
    }

    /// `index_of` (menu.py:295-301). The `and con._index_of(item)`
    /// truthiness bug is preserved: index `0` never matches inside, and the
    /// loop has no `break`, so the last matching container wins.
    pub fn index_of(&self, item: &LookupRef, look_inside: bool) -> Option<i64> {
        let mut index = self.index_of_item(item);
        if index.is_none() && look_inside {
            let children = self.inner.borrow().items.clone();
            for con in &children {
                if con.is_container() && con.index_of_item(item).is_some_and(|found| found != 0) {
                    index = self.index_of_item(&LookupRef::Element(con.clone()));
                }
            }
        }
        index
    }

    /// `add_parents` (menu.py:303-307).
    pub fn add_parents(&self, parents: &[Arc<MenuElement>]) {
        self.inner.borrow_mut().parents.extend(parents.iter().cloned());
    }

    /// `assert_recursive_relation` (menu.py:309-311).
    pub fn assert_recursive_relation(&self) -> KResult<()> {
        let parents = self.inner.borrow().parents.clone();
        let self_handle = self.element_handle();
        if parents.iter().any(|p| Arc::ptr_eq(p, &self_handle)) {
            return Err(py_exception(
                "AssertionError",
                format!("Recursive relation of '{}' container", self.get_ns(".")),
            ));
        }
        Ok(())
    }

    /// `insert_item` / `_insert_item` (menu.py:313-330).
    pub fn insert_item(&self, item: &LookupRef, index: Option<i64>) -> KResult<()> {
        let looked_up = self.lookup_item(item)?;
        if let Some((item, name)) = looked_up {
            // `is_accepted` is true for every element (menu.py:259-260).
            item.init();
            if item.is_container() {
                let own_parents = self.inner.borrow().parents.clone();
                item.add_parents(&own_parents);
                item.add_parents(&[self.element_handle()]);
                item.assert_recursive_relation()?;
            }
            let mut inner = self.inner.borrow_mut();
            match index {
                None => inner.allitems.push((item, name)),
                Some(at) => list_insert(&mut inner.allitems, at, (item, name)),
            }
        }
        Ok(())
    }

    /// `_populate` of `MenuList` and `MenuVSDList` (menu.py:595-599, 666-673).
    fn populate_successors(&self) -> KResult<()> {
        if !self.kind.is_list() {
            return Ok(());
        }
        self.inner.borrow_mut().viewport_top = 0;
        // Add the back item as the first item.
        let back = self.inner.borrow().back_item.clone();
        if let Some(back) = back {
            self.insert_item(&LookupRef::Element(back), Some(0))?;
        }
        if self.kind == ElementKind::VsdList {
            let manager = self.manager();
            let sdcard = lookup_interface::<dyn VirtualSDApi>(
                &*manager.printer,
                "virtual_sdcard",
                LookupDefault::ReturnNone,
            )?;
            if let Some(sdcard) = sdcard {
                for (fname, _fsize) in sdcard.get_file_list(false)? {
                    let kwargs = MenuItemKwargs {
                        name: Some(str_repr(&fname)),
                        gcode: Some(ScriptValue::Static(Some(format!("M23 /{fname}")))),
                        ..Default::default()
                    };
                    let item = manager.menuitem_from("command", &kwargs)?;
                    self.insert_item(&LookupRef::Element(item), None)?;
                }
            }
        }
        Ok(())
    }

    /// `populate` (menu.py:336-346).
    pub fn populate(&self) -> KResult<()> {
        self.inner.borrow_mut().allitems.clear();
        for name in self.names_as_list() {
            self.insert_item(&LookupRef::Named(name), None)?;
        }
        // Populate successor items.
        self.populate_successors()?;
        // Run the populate callback.
        let cb = self.inner.borrow().populate_cb.clone();
        if let Some(cb) = cb {
            cb(&self.element_handle())?;
        }
        // Send the populate event.
        self.send_event("populate", &[EventArg::handle(self.element_handle())])?;
        Ok(())
    }

    /// `update_items` (menu.py:348-351).
    pub fn update_items(&self) -> KResult<()> {
        let all = self.inner.borrow().allitems.clone();
        let mut items = Vec::new();
        let mut names = Vec::new();
        for (item, name) in all {
            if item.is_enabled()? {
                items.push(item);
                names.push(name);
            }
        }
        if items.is_empty() {
            // `zip(*[])` is an empty (truthy) iterator, so Python's
            // `or ([], [])` fallback never applies and the unpacking fails.
            return Err(py_exception(
                "ValueError",
                "not enough values to unpack (expected 2, got 0)",
            ));
        }
        let mut inner = self.inner.borrow_mut();
        inner.items = items;
        inner.names = names;
        Ok(())
    }

    /// `init_selection` (menu.py:354-355); returns `None` like the reference.
    pub fn init_selection(&self) {
        self.select_at(Some(0));
    }

    /// `select_at` (menu.py:357-363).
    pub fn select_at(&self, index: Option<i64>) -> Option<Arc<MenuElement>> {
        self.inner.borrow_mut().selected = index;
        let item = self.selected_item();
        if let Some(item) = &item {
            item.select();
        }
        item
    }

    /// `select_item` (menu.py:365-373).
    pub fn select_item(&self, needle: &Arc<MenuElement>) -> Option<i64> {
        let current = self.selected_item();
        if !matches!(&current, Some(cur) if Arc::ptr_eq(cur, needle)) {
            if let Some(index) = self.index_of(&LookupRef::Element(needle.clone()), false) {
                self.select_at(Some(index));
            }
        }
        self.selected()
    }

    /// `selected_item` (menu.py:375-379).
    pub fn selected_item(&self) -> Option<Arc<MenuElement>> {
        let inner = self.inner.borrow();
        match inner.selected {
            Some(index) if 0 <= index && index < inner.items.len() as i64 => {
                inner.items.get(index as usize).cloned()
            }
            _ => None,
        }
    }

    /// `select_next` (menu.py:381-388).
    pub fn select_next(&self) -> Option<Arc<MenuElement>> {
        let len = self.len() as i64;
        let index = match self.selected() {
            None => (len > 0).then_some(0),
            Some(cur) if 0 <= cur && cur < len - 1 => Some(cur + 1),
            Some(cur) => Some(cur),
        };
        self.select_at(index)
    }

    /// `select_prev` (menu.py:390-397).
    pub fn select_prev(&self) -> Option<Arc<MenuElement>> {
        let len = self.len() as i64;
        let index = match self.selected() {
            None => (len > 0).then_some(0),
            Some(cur) if 0 < cur && cur < len => Some(cur - 1),
            Some(cur) => Some(cur),
        };
        self.select_at(index)
    }

    /// `__len__` (menu.py:406-407).
    pub fn len(&self) -> usize {
        self.inner.borrow().items.len()
    }

    /// `__len__ == 0`.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// `__iter__` snapshot (menu.py:403-404).
    pub fn iter_items(&self) -> Vec<Arc<MenuElement>> {
        self.inner.borrow().items.clone()
    }

    /// `draw_container` for lists (menu.py:399-401, 601-659). Errors inside
    /// the draw loop are logged as "List drawing error" exactly like Python.
    pub fn draw_container(&self, nrows: i64, eventtime: f64) {
        if !self.kind.is_list() {
            return;
        }
        let mut viewport_top = self.inner.borrow().viewport_top;
        let selected_row = self.selected();
        match selected_row {
            Some(row) => {
                if row >= viewport_top + nrows {
                    viewport_top = (row - nrows) + 1;
                }
                if row < viewport_top {
                    viewport_top = row;
                }
            }
            None => viewport_top = 0,
        }
        // Clamp the viewport: `max(0, min(viewport_top, len(self) - nrows))`.
        let len = self.len() as i64;
        viewport_top = viewport_top.min(len - nrows).max(0);
        self.inner.borrow_mut().viewport_top = viewport_top;
        if let Err(e) = self.draw_rows(nrows, eventtime, viewport_top, selected_row) {
            klippy_api::log::exception(
                "List drawing error",
                &format!("{}: {}", e.class_name(), e.message()),
            );
        }
    }

    fn draw_rows(
        &self,
        nrows: i64,
        eventtime: f64,
        viewport_top: i64,
        selected_row: Option<i64>,
    ) -> KResult<()> {
        let manager = self.manager();
        let cols = manager.cols();
        // `selected` and `current` keep their value from the previous row
        // when `row >= len(self)` and are unbound before the first item
        // (menu.py:620-651).
        let mut bound: Option<(bool, Arc<MenuElement>)> = None;
        for (y, row) in (0_i64..).zip(viewport_top..(viewport_top + nrows)) {
            let mut text = String::new();
            let mut prefix = String::new();
            let mut suffix = String::new();
            let current = self.inner.borrow().items.get(row as usize).cloned();
            if let (true, Some(cur)) = (row >= 0, current) {
                let selected = Some(row) == selected_row;
                bound = Some((selected, cur.clone()));
                if selected {
                    cur.heartbeat(eventtime)?;
                }
                text = cur.render_name(selected)?;
                // Add the prefix (selection indicator).
                if selected && !cur.is_editing() {
                    prefix = cur.cursor();
                } else if selected && cur.is_editing() {
                    prefix = "*".to_string();
                } else {
                    prefix = " ".to_string();
                }
                // Add the suffix (folder indicator).
                if cur.is_list() {
                    suffix.push('>');
                }
            }
            let plen = prefix.chars().count() as i64;
            let slen = suffix.chars().count() as i64;
            let width = cols - plen - slen;
            // Draw the item prefix (cursor), then the item name.
            let ppos = manager.display.draw_text(y, 0, &prefix, eventtime)?;
            let tpos = manager.display.draw_text(y, ppos, &ljust(&text, width), eventtime)?;
            // Check the scroller.
            let Some((selected, current)) = &bound else {
                return Err(py_exception(
                    "UnboundLocalError",
                    "local variable 'selected' referenced before assignment",
                ));
            };
            if *selected && tpos > cols && current.is_scrollable() {
                current.need_scroller(Some(true));
            } else {
                current.need_scroller(None);
            }
            // Draw the item suffix.
            if !suffix.is_empty() {
                manager.display.draw_text(y, cols - slen, &suffix, eventtime)?;
            }
        }
        Ok(())
    }
}

/// `text.ljust(width)`.
fn ljust(text: &str, width: i64) -> String {
    let len = text.chars().count() as i64;
    if width <= len {
        return text.to_string();
    }
    let mut out = text.to_string();
    for _ in 0..(width - len) {
        out.push(' ');
    }
    out
}

/// `TemplateWrapper.render(context)` (gcode_macro.py:66).
fn template_render(tpl: &TemplateWrapper, context: &TemplateContext) -> KResult<String> {
    tpl.render(Some(context.clone()))
}

/// `printer.lookup_object(name)` with the `configfile.sentinel` default,
/// followed by the interface the Python code duck-types against.
fn lookup_required<T>(printer: &Printer, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    lookup_interface::<T>(&**printer, name, LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::config(format!("Unknown config object '{name}'")))
}

/// `context['menu']` as the dict `update` works on (`KeyError` when
/// `update_context` has not run, exactly like Python).
fn context_menu(context: &TemplateContext) -> KResult<IndexMap<PyValue, PyValue>> {
    match context.get("menu") {
        Some(Value::Py(PyValue::Dict(menu))) => Ok(menu.clone()),
        Some(other) => Err(py_exception(
            "AttributeError",
            format!("'{}' object has no attribute 'update'", other.type_name()),
        )),
        None => Err(py_exception("KeyError", "'menu'")),
    }
}

// ---------------------------------------------------------------------------
// Manager.
// ---------------------------------------------------------------------------

/// The template context kept between redraws (`MenuManager.context`).
struct StoredContext(TemplateContext);

// SAFETY: `Value::Func` closures hold `Rc`s, so `Value` is not `Send`. The
// context is only created, cloned and dropped on reactor greenlets, which
// run one at a time with a memory-ordering baton hand-off (see
// `klippy_api::gcell`), so the non-atomic reference counts are never
// touched concurrently.
unsafe impl Send for StoredContext {}

/// Mutable manager state (`MenuManager.__init__` fields).
struct ManagerState {
    running: bool,
    menuitems: IndexMap<String, Arc<MenuElement>>,
    menustack: Vec<Arc<MenuElement>>,
    children: IndexMap<String, Vec<String>>,
    gcode_queue: Vec<String>,
    context: StoredContext,
    root: Option<Arc<MenuElement>>,
    root_name: String,
    cols: i64,
    rows: i64,
    timeout: i64,
    timer: i64,
    reverse_navigation: bool,
}

/// `MenuManager` (menu.py:688-1067). Construct with [`MenuManager::new`],
/// like the Python `menu.MenuManager(config, self)` call in the display
/// module.
pub struct MenuManager {
    state: GCell<ManagerState>,
    weak_self: Weak<MenuManager>,
    printer: Printer,
    pconfig: PrinterConfig,
    gcode: GCode,
    gcode_macro: Arc<dyn PrinterGCodeMacroApi>,
    display: Arc<dyn PrinterLCDApi>,
}

impl MenuManager {
    /// `MenuManager.__init__(config, display)` (menu.py:689-722).
    /// Raises on bad options or an unknown root, like Python.
    pub fn new(config: &Config, display: Arc<dyn PrinterLCDApi>) -> KResult<Arc<MenuManager>> {
        let printer = config.get_printer();
        let pconfig = lookup_required::<dyn PrinterConfigApi>(&printer, "configfile")?;
        let gcode = lookup_required::<dyn GCodeApi>(&printer, "gcode")?;
        let root_name = config
            .get(
                "menu_root",
                GetStrOpts {
                    default: Some(Some("__main".to_string())),
                    ..Default::default()
                },
            )?
            .unwrap_or_else(|| "__main".to_string());
        let (cols, rows) = display.get_dimensions();
        let timeout = config
            .getint(
                "menu_timeout",
                GetIntOpts {
                    default: Some(Some(0)),
                    ..Default::default()
                },
            )?
            .unwrap_or(0);
        // Reverse container navigation.
        let reverse_navigation = config
            .getboolean(
                "menu_reverse_navigation",
                GetBoolOpts {
                    default: Some(Some(false)),
                    ..Default::default()
                },
            )?
            .unwrap_or(false);
        // Load printer objects.
        let gcode_macro = klippy_api::load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .ok_or_else(|| KlippyError::config("Unknown config object 'gcode_macro'"))?;
        let manager = Arc::new_cyclic(|weak| MenuManager {
            state: GCell::new(ManagerState {
                running: false,
                menuitems: IndexMap::new(),
                menustack: Vec::new(),
                children: IndexMap::new(),
                gcode_queue: Vec::new(),
                context: StoredContext(IndexMap::new()),
                root: None,
                root_name,
                cols,
                rows,
                timeout,
                timer: 0,
                reverse_navigation,
            }),
            weak_self: weak.clone(),
            printer,
            pconfig,
            gcode,
            gcode_macro,
            display,
        });
        // Register itself for printer callbacks.
        manager
            .printer
            .add_object("menu", manager.clone() as ObjectRef)?;
        {
            let ready_manager = manager.clone();
            manager.printer.register_event_handler(
                "klippy:ready",
                Box::new(move |_args| {
                    ready_manager.handle_ready();
                    Ok(PyValue::None)
                }),
            );
        }
        // Register for key events.
        {
            let key_manager = manager.clone();
            let callback: MenuKeyCallback = Arc::new(move |key: &str, eventtime: f64| {
                key_manager.key_event(key, eventtime)
                    .unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message()));
            });
            MenuKeys::new(config, callback)?;
        }
        // Load the local config file in the same directory as this module,
        // then the items of the main config.
        manager.load_config(&bundled_menu_config())?;
        manager.load_menuitems(config)?;
        // Load the menu root and send the init event.
        let root = manager.lookup_menuitem(Some(&manager.root_name()))?;
        manager.state.borrow_mut().root = root;
        manager.send_event("init", &[EventArg::Object(manager.self_object())])?;
        Ok(manager)
    }

    /// `self._root` (menu.py:701).
    pub fn root_name(&self) -> String {
        self.state.borrow().root_name.clone()
    }

    /// `self.cols` (menu.py:702).
    pub fn cols(&self) -> i64 {
        self.state.borrow().cols
    }

    /// `self.rows` (menu.py:702).
    pub fn rows(&self) -> i64 {
        self.state.borrow().rows
    }

    /// `self` (the manager handle Python code passes around).
    fn manager_handle(&self) -> Arc<MenuManager> {
        self.weak_self
            .upgrade()
            .expect("menu manager is alive while its methods run")
    }

    /// `self` as a printer object (for event payloads).
    fn self_object(&self) -> ObjectRef {
        self.manager_handle() as ObjectRef
    }


    /// `handle_ready` (menu.py:724-727).
    pub fn handle_ready(&self) {
        let reactor = self.printer.get_reactor();
        let manager = self.manager_handle();
        reactor.register_timer(
            move |eventtime| manager.timer_event(eventtime)
                .unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message())),
            reactor::NOW,
        );
    }

    /// `timer_event` (menu.py:729-731).
    pub fn timer_event(&self, eventtime: f64) -> KResult<f64> {
        self.timeout_check(eventtime)?;
        Ok(eventtime + TIMER_DELAY)
    }

    /// `timeout_check` (menu.py:733-741).
    pub fn timeout_check(&self, _eventtime: f64) -> KResult<()> {
        let (running, timeout, timer, root_is_container) = {
            let state = self.state.borrow();
            (
                state.running,
                state.timeout,
                state.timer,
                state.root.as_ref().is_some_and(|r| r.is_container()),
            )
        };
        if running && timeout > 0 && root_is_container {
            if timer >= timeout {
                self.exit(false)?;
            } else {
                self.state.borrow_mut().timer += 1;
            }
        } else {
            self.state.borrow_mut().timer = 0;
        }
        Ok(())
    }

    /// `send_event` (menu.py:743-744).
    pub fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.printer.send_event(&format!("menu:{event}"), params)
    }

    /// `is_running` (menu.py:746-747).
    pub fn is_running(&self) -> bool {
        self.state.borrow().running
    }

    /// `begin` (menu.py:749-763).
    pub fn begin(&self, eventtime: f64) -> KResult<()> {
        {
            let mut state = self.state.borrow_mut();
            state.menustack.clear();
            state.timer = 0;
        }
        let root = self.state.borrow().root.clone();
        match root {
            Some(root) if root.is_container() => {
                self.send_event("begin", &[EventArg::Object(self.self_object())])?;
                self.update_context(eventtime)?;
                root.init_selection();
                self.stack_push(&root)?;
                self.state.borrow_mut().running = true;
                return Ok(());
            }
            Some(_) => klippy_api::log::error("Invalid root, menu stopped!"),
            None => {}
        }
        self.state.borrow_mut().running = false;
        Ok(())
    }

    /// `get_status` (menu.py:765-771).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let state = self.state.borrow();
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("timeout".to_string()),
            PyValue::Int(PyInt::from_i64(state.timeout)),
        );
        map.insert(
            PyValue::Str("running".to_string()),
            PyValue::Bool(state.running),
        );
        map.insert(
            PyValue::Str("rows".to_string()),
            PyValue::Int(PyInt::from_i64(state.rows)),
        );
        map.insert(
            PyValue::Str("cols".to_string()),
            PyValue::Int(PyInt::from_i64(state.cols)),
        );
        PyValue::Dict(map)
    }

    /// `_action_back` (menu.py:773-775): the `menu.back()` template action.
    pub fn action_back(&self, force: bool, update: bool) -> KResult<String> {
        self.back(force, update)?;
        Ok(String::new())
    }

    /// `_action_exit` (menu.py:777-779): the `menu.exit()` template action.
    pub fn action_exit(&self, force: bool) -> KResult<String> {
        self.exit(force)?;
        Ok(String::new())
    }

    /// `get_context` (menu.py:781-785).
    pub fn get_context(&self, cxt: Option<&TemplateContext>) -> TemplateContext {
        let mut context = self.state.borrow().context.0.clone();
        if let Some(extra) = cxt {
            for (k, v) in extra {
                context.insert(k.clone(), v.clone());
            }
        }
        context
    }

    /// `context['menu'].update({...})` on a context from `get_context`.
    /// The Python context is a shallow copy, so unless `cxt` brings its own
    /// `menu` dict the update also lands in the manager's dict, and later
    /// contexts keep the keys (`ns`, `event`, `input`) until
    /// `update_context` replaces it.
    fn update_menu(
        &self,
        context: &mut TemplateContext,
        cxt: Option<&TemplateContext>,
        updates: &[(&str, PyValue)],
    ) -> KResult<()> {
        let mut menu = context_menu(context)?;
        for (key, value) in updates {
            menu.insert(PyValue::Str((*key).to_string()), value.clone());
        }
        let own_menu = cxt.is_some_and(|extra| extra.contains_key("menu"));
        if !own_menu {
            self.state.borrow_mut().context.0.insert(
                "menu".to_string(),
                Value::py(PyValue::Dict(menu.clone())),
            );
        }
        context.insert("menu".to_string(), Value::py(PyValue::Dict(menu)));
        Ok(())
    }

    /// `update_context` (menu.py:787-794).
    pub fn update_context(&self, eventtime: f64) -> KResult<()> {
        let mut context = self.gcode_macro.create_template_context(Some(eventtime));
        let mut menu = IndexMap::new();
        menu.insert(
            PyValue::Str("eventtime".to_string()),
            PyValue::Float(eventtime),
        );
        menu.insert(PyValue::Str("back".to_string()), PyValue::Host(HOST_BACK));
        menu.insert(PyValue::Str("exit".to_string()), PyValue::Host(HOST_EXIT));
        context.insert("menu".to_string(), Value::py(PyValue::Dict(menu)));
        self.state.borrow_mut().context = StoredContext(context);
        Ok(())
    }

    /// `stack_push` (menu.py:796-809).
    pub fn stack_push(&self, container: &Arc<MenuElement>) -> KResult<()> {
        if !container.is_container() {
            return Err(menu_error("Wrong type, expected MenuContainer".to_string()));
        }
        container.populate()?;
        if let Some(top) = self.stack_peek(0) {
            if top.is_list() {
                top.run_script("leave", &RunScriptOpts::default())?;
            }
        }
        if container.is_list() {
            container.run_script("enter", &RunScriptOpts::default())?;
        }
        if !container.is_editing() {
            container.update_items()?;
            container.init_selection();
        }
        self.state.borrow_mut().menustack.push(container.clone());
        Ok(())
    }

    /// `stack_pop` (menu.py:811-831).
    pub fn stack_pop(&self, update: bool) -> KResult<Option<Arc<MenuElement>>> {
        if self.stack_size() == 0 {
            return Ok(None);
        }
        let container = self.state.borrow_mut().menustack.pop();
        if let Some(container) = &container {
            if !container.is_container() {
                return Err(menu_error("Wrong type, expected MenuContainer".to_string()));
            }
            match self.stack_peek(0) {
                Some(top) => {
                    if !top.is_editing() && update {
                        top.update_items()?;
                        top.init_selection();
                    }
                    if container.is_list() {
                        container.run_script("leave", &RunScriptOpts::default())?;
                    }
                    if top.is_list() {
                        top.run_script("enter", &RunScriptOpts::default())?;
                    }
                }
                None => {
                    if container.is_list() {
                        container.run_script("leave", &RunScriptOpts::default())?;
                    }
                }
            }
        }
        Ok(container)
    }

    /// `stack_size` (menu.py:833-834).
    pub fn stack_size(&self) -> usize {
        self.state.borrow().menustack.len()
    }

    /// `stack_peek` (menu.py:836-840).
    pub fn stack_peek(&self, lvl: usize) -> Option<Arc<MenuElement>> {
        let state = self.state.borrow();
        if state.menustack.len() > lvl {
            state.menustack.get(state.menustack.len() - lvl - 1).cloned()
        } else {
            None
        }
    }

    /// `screen_update_event` (menu.py:842-852).
    pub fn screen_update_event(&self, eventtime: f64) -> KResult<bool> {
        // Screen update.
        if !self.is_running() {
            return Ok(false);
        }
        // Draw the menu.
        self.update_context(eventtime)?;
        let container = self.stack_peek(0);
        if self.is_running() {
            if let Some(container) = container.filter(|c| c.is_container()) {
                container.heartbeat(eventtime)?;
                container.draw_container(self.rows(), eventtime);
            }
        }
        Ok(true)
    }

    /// The running container, when the menu runs on one (the shared guard
    /// of `up`, `down`, `back`, `exit`, `push_container` and `press`).
    fn running_container(&self) -> Option<Arc<MenuElement>> {
        let container = self.stack_peek(0);
        if self.is_running() {
            container.filter(|c| c.is_container())
        } else {
            None
        }
    }

    /// `up` (menu.py:854-865).
    pub fn up(&self, fast_rate: bool) -> KResult<()> {
        let Some(container) = self.running_container() else {
            return Ok(());
        };
        self.state.borrow_mut().timer = 0;
        let current = container.selected_item();
        match current {
            Some(current) if current.is_input() && current.is_editing() => {
                current.dec_value(fast_rate)?;
            }
            _ => {
                if self.state.borrow().reverse_navigation {
                    container.select_next(); // reverse
                } else {
                    container.select_prev(); // normal
                }
            }
        }
        Ok(())
    }

    /// `down` (menu.py:867-878).
    pub fn down(&self, fast_rate: bool) -> KResult<()> {
        let Some(container) = self.running_container() else {
            return Ok(());
        };
        self.state.borrow_mut().timer = 0;
        let current = container.selected_item();
        match current {
            Some(current) if current.is_input() && current.is_editing() => {
                current.inc_value(fast_rate)?;
            }
            _ => {
                if self.state.borrow().reverse_navigation {
                    container.select_prev(); // reverse
                } else {
                    container.select_next(); // normal
                }
            }
        }
        Ok(())
    }

    /// `back` (menu.py:880-901).
    pub fn back(&self, force: bool, update: bool) -> KResult<()> {
        let Some(container) = self.running_container() else {
            return Ok(());
        };
        self.state.borrow_mut().timer = 0;
        if let Some(current) = container.selected_item() {
            if current.is_input() && current.is_editing() {
                if force {
                    current.stop_editing()?;
                } else {
                    return Ok(());
                }
            }
        }
        match self.stack_peek(1).filter(|p| p.is_container()) {
            Some(parent) => {
                self.stack_pop(update)?;
                let index = parent.index_of(&LookupRef::Element(container.clone()), true);
                if let Some(index) = index {
                    parent.select_at(Some(index));
                } else if parent.selected_item().is_none() {
                    parent.init_selection();
                }
            }
            None => {
                self.stack_pop(true)?;
                self.state.borrow_mut().running = false;
            }
        }
        Ok(())
    }

    /// `exit` (menu.py:903-914).
    pub fn exit(&self, force: bool) -> KResult<()> {
        let Some(container) = self.running_container() else {
            return Ok(());
        };
        self.state.borrow_mut().timer = 0;
        if let Some(current) = container.selected_item() {
            if !force && current.is_input() && current.is_editing() {
                return Ok(());
            }
        }
        if container.is_list() {
            container.run_script("leave", &RunScriptOpts::default())?;
        }
        self.send_event("exit", &[EventArg::Object(self.self_object())])?;
        self.state.borrow_mut().running = false;
        Ok(())
    }

    /// `push_container` (menu.py:916-924).
    pub fn push_container(&self, menu: &Arc<MenuElement>) -> KResult<bool> {
        if let Some(container) = self.running_container() {
            if menu.is_container() && !container.is_editing() && !Arc::ptr_eq(menu, &container) {
                self.stack_push(menu)?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// `press` (menu.py:926-942).
    pub fn press(&self, event: &str) -> KResult<()> {
        let Some(container) = self.running_container() else {
            return Ok(());
        };
        self.state.borrow_mut().timer = 0;
        let event_opts = RunScriptOpts {
            event: Some(event.to_string()),
            ..Default::default()
        };
        let no_opts = RunScriptOpts::default();
        match container.selected_item() {
            Some(current) if current.is_container() => self.stack_push(&current)?,
            Some(current) if current.is_input() => {
                if current.is_editing() {
                    current.run_script("gcode", &event_opts)?;
                }
                current.run_script(event, &no_opts)?;
            }
            Some(current) if current.is_command() => {
                current.run_script("gcode", &event_opts)?;
                current.run_script(event, &no_opts)?;
            }
            // `current` is `None` (no selection): pass through to the container.
            _ => {
                container.run_script(event, &no_opts)?;
            }
        }
        Ok(())
    }

    /// Snapshot of `self.gcode_queue` (menu.py:699).
    pub fn gcode_queue(&self) -> Vec<String> {
        self.state.borrow().gcode_queue.clone()
    }

    /// `queue_gcode` (menu.py:944-950).
    pub fn queue_gcode(&self, script: &str) {
        if script.is_empty() {
            return;
        }
        let needs_callback = self.state.borrow().gcode_queue.is_empty();
        if needs_callback {
            let reactor = self.printer.get_reactor();
            let manager = self.manager_handle();
            let _ = reactor.register_callback(
                move |_eventtime| {
                    manager.dispatch_gcode();
                },
                reactor::NOW,
            );
        }
        self.state.borrow_mut().gcode_queue.push(script.to_string());
    }

    /// `dispatch_gcode` (menu.py:952-959).
    pub fn dispatch_gcode(&self) {
        loop {
            let script = self.state.borrow().gcode_queue.first().cloned();
            let Some(script) = script else {
                break;
            };
            if let Err(e) = self.gcode.run_script(&script) {
                klippy_api::log::exception(
                    "Script running error",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
            }
            self.state.borrow_mut().gcode_queue.remove(0);
        }
    }

    /// `menuitem_from` (menu.py:961-965).
    pub fn menuitem_from(
        &self,
        type_name: &str,
        kwargs: &MenuItemKwargs,
    ) -> KResult<Arc<MenuElement>> {
        let kind = kind_of(type_name)?;
        MenuElement::build(self.manager_handle(), kind, None, kwargs.clone())
    }

    /// The `..` back item of a list (`menuitem_from('command', name='..',
    /// gcode=_cb)` where `_cb` calls `el.manager.back()`, menu.py:586-590).
    fn back_item(&self) -> KResult<Arc<MenuElement>> {
        let kwargs = MenuItemKwargs {
            name: Some("..".to_string()),
            gcode: Some(ScriptValue::Callback(Arc::new(|element, _context| {
                element.manager().back(false, true)?;
                Ok(None)
            }))),
            ..Default::default()
        };
        self.menuitem_from("command", &kwargs)
    }

    /// `add_menuitem` (menu.py:967-983).
    pub fn add_menuitem(&self, name: &str, item: Arc<MenuElement>) {
        let existing_item = self.state.borrow().menuitems.contains_key(name);
        if existing_item {
            klippy_api::log::info(&format!(
                "Declaration of '{name}' hides previous menuitem declaration"
            ));
        }
        self.state
            .borrow_mut()
            .menuitems
            .insert(name.to_string(), item.clone());
        let parent = item.get_ns("..");
        if !parent.is_empty() && !existing_item {
            let ns = item.get_ns(".");
            let mut state = self.state.borrow_mut();
            let entry = state.children.entry(parent).or_default();
            match item.index() {
                Some(index) => list_insert(entry, index, ns),
                None => entry.push(ns),
            }
        }
    }

    /// `lookup_menuitem(name)` with the `sentinel` default (menu.py:985-993).
    pub fn lookup_menuitem(&self, name: Option<&str>) -> KResult<Option<Arc<MenuElement>>> {
        let Some(name) = name else {
            return Ok(None);
        };
        match self.state.borrow().menuitems.get(name).cloned() {
            Some(item) => Ok(Some(item)),
            None => Err(KlippyError::config(format!("Unknown menuitem '{name}'"))),
        }
    }

    /// `lookup_children` (menu.py:995-998).
    pub fn lookup_children(&self, ns: &str) -> Vec<String> {
        self.state
            .borrow()
            .children
            .get(ns)
            .cloned()
            .unwrap_or_default()
    }

    /// `load_config` (menu.py:1000-1010).
    pub fn load_config(&self, filename: &str) -> KResult<Config> {
        let cfg = match self.pconfig.read_config(filename) {
            Ok(cfg) => cfg,
            Err(_) => {
                return Err(KlippyError::config(format!(
                    "Cannot load config '{filename}'"
                )))
            }
        };
        self.load_menuitems(&cfg)?;
        Ok(cfg)
    }

    /// `load_menuitems` (menu.py:1012-1019).
    pub fn load_menuitems(&self, config: &Config) -> KResult<()> {
        for cfg in config.get_prefix_sections("menu ") {
            let type_name = cfg.get("type", GetStrOpts::default())?.unwrap_or_default();
            let kind = kind_of(&type_name)?;
            let item = MenuElement::build(
                self.manager_handle(),
                kind,
                Some(&cfg),
                MenuItemKwargs::default(),
            )?;
            self.add_menuitem(&item.get_ns("."), item);
        }
        Ok(())
    }

    /// `_click_callback` (menu.py:1021-1026).
    pub fn click_callback(&self, eventtime: f64, event: &str) -> KResult<()> {
        if self.is_running() {
            self.press(event)
        } else {
            // Let's start and populate the menu items.
            self.begin(eventtime)
        }
    }

    /// `key_event` (menu.py:1028-1043): the callback `MenuKeys` takes.
    pub fn key_event(&self, key: &str, eventtime: f64) -> KResult<()> {
        match key {
            "click" | "long_click" => self.click_callback(eventtime, key)?,
            "up" => self.up(false)?,
            "fast_up" => self.up(true)?,
            "down" => self.down(false)?,
            "fast_down" => self.down(true)?,
            "back" => self.back(false, true)?,
            _ => {}
        }
        self.display.request_redraw();
        Ok(())
    }

    /// `stripliterals` (menu.py:1047-1054).
    pub fn stripliterals(s: &str) -> String {
        let chars: Vec<char> = s.chars().collect();
        let quoted = (chars.first() == Some(&'"') && chars.last() == Some(&'"'))
            || (chars.first() == Some(&'\'') && chars.last() == Some(&'\''));
        if !quoted {
            return s.to_string();
        }
        if chars.len() <= 2 {
            return String::new();
        }
        chars[1..chars.len() - 1].iter().collect()
    }

    /// `aslatin` (menu.py:1056-1063). `str` values pass through; the
    /// `unicode` branch cannot run on Python 3.
    pub fn aslatin(s: &str) -> String {
        s.to_string()
    }

    /// `asflat` (menu.py:1065-1067).
    pub fn asflat(s: &str) -> String {
        Self::stripliterals(&splitlines_joined(&Self::aslatin(s)))
    }
}

/// `''.join(s.splitlines())` with the CPython line boundaries.
fn splitlines_joined(s: &str) -> String {
    let mut out = String::new();
    let mut chars = s.chars().peekable();
    while let Some(ch) = chars.next() {
        match ch {
            '\r' => {
                if chars.peek() == Some(&'\n') {
                    chars.next();
                }
            }
            '\n' | '\u{b}' | '\u{c}' | '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{85}' | '\u{2028}'
            | '\u{2029}' => {}
            _ => out.push(ch),
        }
    }
    out
}

/// `os.path.join(os.path.dirname(__file__), 'menu.cfg')` (menu.py:716).
fn bundled_menu_config() -> String {
    klippy_api::paths::klippy_path(&["extras", "display", "menu.cfg"])
}

// ---------------------------------------------------------------------------
// Element construction (`MenuElement.__init__` and subclass `__init__`).
// ---------------------------------------------------------------------------

impl MenuElement {
    /// `MenuElement.__init__` followed by the `__init__` of each subclass
    /// in the order Python runs them (menu.py:22-58, 237-247, 426-428,
    /// 432-451, 582-590), so option reads and errors keep their order.
    /// `config` is the section for `menu_items[type](manager, cfg)`, `None`
    /// for `menu_items[type](manager, None, **kwargs)`.
    fn build(
        manager: Arc<MenuManager>,
        kind: ElementKind,
        config: Option<&Config>,
        kwargs: MenuItemKwargs,
    ) -> KResult<Arc<MenuElement>> {
        // `MenuDisabled.__init__` passes only `name=''` to its base (menu.py:418-419).
        let kwargs = if kind == ElementKind::Disabled {
            MenuItemKwargs {
                name: Some(String::new()),
                ..Default::default()
            }
        } else {
            kwargs
        };
        let mut inner = ElementInner {
            index: kwargs.index,
            enable: kwargs.enable.unwrap_or(true),
            name: kwargs.name.clone(),
            input: kwargs.input,
            input_min: Some(kwargs.input_min.unwrap_or(-999999.0)),
            input_max: Some(kwargs.input_max.unwrap_or(999999.0)),
            input_step: kwargs.input_step.unwrap_or(1.0),
            realtime: kwargs.realtime.unwrap_or(false),
            populate_cb: kwargs.populate.clone(),
            ..Default::default()
        };
        if let Some(config) = config {
            // Overwrite the class attributes from the config (menu.py:33-44).
            inner.index = config.getint(
                "index",
                GetIntOpts {
                    default: Some(inner.index),
                    ..Default::default()
                },
            )?;
            inner.name_tpl = Some(Arc::new(manager.gcode_macro.load_template(
                config,
                "name",
                inner.name.as_deref(),
            )?));
            match config.getboolean(
                "enable",
                GetBoolOpts {
                    default: Some(Some(inner.enable)),
                    ..Default::default()
                },
            ) {
                Ok(value) => inner.enable = value.unwrap_or(inner.enable),
                Err(KlippyError::Config(_)) => {
                    inner.enable_tpl = Some(Arc::new(
                        manager.gcode_macro.load_template(config, "enable", None)?,
                    ));
                }
                Err(e) => return Err(e),
            }
            // Item namespace, used in relative paths:
            // `str(" ".join(config.get_name().split(' ')[1:])).strip()`.
            let section = config.get_name();
            let rest: Vec<&str> = section.split(' ').skip(1).collect();
            inner.ns = rest.join(" ").trim().to_string();
        } else {
            // Generated namespace (menu.py:45-50).
            let id_text = format!("__menu_{:x}", generated_id());
            let raw = format!(
                "menu {}",
                kwargs.ns.clone().unwrap_or_else(|| id_text.clone())
            );
            inner.ns = safe_substitute_id(&raw, &id_text);
        }
        // The element exists before `_load_script` runs (it logs and reads
        // `self._scripts`); `init()` only resets state that is already default.
        let element = Arc::new_cyclic(|weak| MenuElement {
            manager: Arc::downgrade(&manager),
            self_weak: weak.clone(),
            kind,
            inner: GCell::new(inner),
        });
        element.init();
        // `MenuCommand.__init__`: `_load_script(config or kwargs, 'gcode')`.
        if kind.is_command() {
            match config {
                Some(config) => element.load_script(config, "gcode", None)?,
                None => element.load_script_value(
                    "gcode",
                    kwargs.gcode.clone().unwrap_or(ScriptValue::Static(None)),
                ),
            }
        }
        // `MenuInput.__init__` (menu.py:441-451).
        if kind == ElementKind::Input {
            if let Some(config) = config {
                let (realtime, input_min, input_max, step) = {
                    let inner = element.inner.borrow();
                    (inner.realtime, kwargs.input_min, kwargs.input_max, inner.input_step)
                };
                let realtime = config
                    .getboolean(
                        "realtime",
                        GetBoolOpts {
                            default: Some(Some(realtime)),
                            ..Default::default()
                        },
                    )?
                    .unwrap_or(realtime);
                element.inner.borrow_mut().realtime = realtime;
                let input_tpl = manager.gcode_macro.load_template(config, "input", None)?;
                let min_tpl = manager.gcode_macro.load_template(
                    config,
                    "input_min",
                    Some(&float_str(input_min.unwrap_or(-999999.0))),
                )?;
                let max_tpl = manager.gcode_macro.load_template(
                    config,
                    "input_max",
                    Some(&float_str(input_max.unwrap_or(999999.0))),
                )?;
                let step = config
                    .getfloat(
                        "input_step",
                        GetFloatOpts {
                            default: Some(Some(step)),
                            above: Some(0.0),
                            ..Default::default()
                        },
                    )?
                    .unwrap_or(step);
                let mut inner = element.inner.borrow_mut();
                inner.input_tpl = Some(Arc::new(input_tpl));
                inner.input_min_tpl = Some(Arc::new(min_tpl));
                inner.input_max_tpl = Some(Arc::new(max_tpl));
                inner.input_step = step;
            }
        }
        if kind.is_list() {
            // `MenuList.__init__` creates the back item (menu.py:582-590).
            let back = manager.back_item()?;
            element.inner.borrow_mut().back_item = Some(back);
        }
        Ok(element)
    }
}

/// Monotonic element namespace ids (the Python `id(self)` equivalent;
/// hex digits only, so the `.lstrip("0x").rstrip("L")` steps are no-ops).
fn generated_id() -> usize {
    static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0x1000);
    NEXT.fetch_add(0x10, std::sync::atomic::Ordering::SeqCst)
}

/// `Template(text).safe_substitute(__id=id_text)` (menu.py:49-50): `$$`
/// becomes `$`, `$__id` and `${__id}` are replaced, every other `$name`,
/// `${name}` and lone `$` stays as written.
fn safe_substitute_id(template: &str, id_text: &str) -> String {
    let is_start = |c: char| c == '_' || c.is_ascii_alphabetic();
    let is_part = |c: char| c == '_' || c.is_ascii_alphanumeric();
    let chars: Vec<char> = template.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i] != '$' {
            out.push(chars[i]);
            i += 1;
            continue;
        }
        match chars.get(i + 1).copied() {
            Some('$') => {
                out.push('$');
                i += 2;
            }
            Some(c) if is_start(c) => {
                let mut end = i + 1;
                while end < chars.len() && is_part(chars[end]) {
                    end += 1;
                }
                let name: String = chars[i + 1..end].iter().collect();
                if name == "__id" {
                    out.push_str(id_text);
                } else {
                    out.push('$');
                    out.push_str(&name);
                }
                i = end;
            }
            Some('{') => {
                let mut end = i + 2;
                if end < chars.len() && is_start(chars[end]) {
                    while end < chars.len() && is_part(chars[end]) {
                        end += 1;
                    }
                    if end < chars.len() && chars[end] == '}' {
                        let name: String = chars[i + 2..end].iter().collect();
                        if name == "__id" {
                            out.push_str(id_text);
                        } else {
                            out.push_str("${");
                            out.push_str(&name);
                            out.push('}');
                        }
                        i = end + 1;
                        continue;
                    }
                }
                out.push('$');
                i += 1;
            }
            _ => {
                out.push('$');
                i += 1;
            }
        }
    }
    out
}

impl PrinterObject for MenuManager {
    klippy_api::printer_object_base!();
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(MenuManager::get_status(self, eventtime))
    }
}

/// Module entry points. The Python module has none (`MenuManager` is
/// constructed by the `display` module), so all entries are `None`.
pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    klippy_api::printer::ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
