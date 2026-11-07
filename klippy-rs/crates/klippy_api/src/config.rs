//! Contract of the `config` service.
//!
//! Ports the public surface of `klippy/configfile.py`: [`ConfigWrapper`]
//! (exposed as [`ConfigApi`]) and [`PrinterConfig`] (exposed as
//! [`PrinterConfigApi`], printer object `configfile`). The ported
//! `core::configfile` module implements these traits.
//!
//! The Python `sentinel` default (`configfile.py:15`, "required" vs
//! `default=None` "optional") is modelled as `Option<Option<T>>` in each
//! `XxxOpts` struct: outer `None` is required (missing option raises
//! `KlippyError::Config`), `Some(None)` returns `None` when absent, and
//! `Some(Some(v))` returns `v` when absent. All getters are synchronous
//! with no reactor pause.
//!
//! `ConfigWrapper.error` (`configfile.py:18`) is the `configparser.Error`
//! class; in Rust it is [`KlippyError::Config`] and needs no method.

use std::sync::Arc;

use pycompat::PyValue;

use crate::error::KResult;
use crate::mcu::Mcu;
use crate::printer::Printer;

/// Options for [`ConfigApi::get`] (`configfile.py:61`).
#[derive(Clone, Debug)]
pub struct GetStrOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<String>>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetStrOpts {
    fn default() -> Self {
        GetStrOpts { default: None, note_valid: true }
    }
}

/// Options for [`ConfigApi::getint`] (`configfile.py:64`).
#[derive(Clone, Debug)]
pub struct GetIntOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<i64>>,
    /// `minval`.
    pub minval: Option<i64>,
    /// `maxval`.
    pub maxval: Option<i64>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetIntOpts {
    fn default() -> Self {
        GetIntOpts { default: None, minval: None, maxval: None, note_valid: true }
    }
}

/// Options for [`ConfigApi::getfloat`] (`configfile.py:68`).
#[derive(Clone, Debug)]
pub struct GetFloatOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<f64>>,
    /// `minval`.
    pub minval: Option<f64>,
    /// `maxval`.
    pub maxval: Option<f64>,
    /// `above` (exclusive lower bound, `v <= above` raises).
    pub above: Option<f64>,
    /// `below` (exclusive upper bound, `v >= below` raises).
    pub below: Option<f64>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetFloatOpts {
    fn default() -> Self {
        GetFloatOpts {
            default: None,
            minval: None,
            maxval: None,
            above: None,
            below: None,
            note_valid: true,
        }
    }
}

/// Options for [`ConfigApi::getboolean`] (`configfile.py:73`).
#[derive(Clone, Debug)]
pub struct GetBoolOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<bool>>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetBoolOpts {
    fn default() -> Self {
        GetBoolOpts { default: None, note_valid: true }
    }
}

/// Options for [`ConfigApi::getchoice`] (`configfile.py:76`).
#[derive(Clone, Debug)]
pub struct GetChoiceOpts {
    /// `default` key (`None` = required, see module docs); the key, not
    /// the mapped value. `Some(None)` returns the outcome of looking up
    /// `None` in the choices (which raises unless `None` is a key).
    pub default: Option<Option<PyValue>>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetChoiceOpts {
    fn default() -> Self {
        GetChoiceOpts { default: None, note_valid: true }
    }
}

/// Element parser for [`ConfigApi::getlists`] (`configfile.py:87`).
///
/// The Python `parser=str|int|float` parameter only ever takes these
/// three callables in the tree.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ListParser {
    /// `str`.
    #[default]
    Str,
    /// `int`.
    Int,
    /// `float`.
    Float,
}

/// Options for [`ConfigApi::getlists`] (`configfile.py:87`).
#[derive(Clone, Debug)]
pub struct GetListsOpts {
    /// `default` (`None` = required, see module docs); an already-parsed
    /// tuple value as [`PyValue::Tuple`].
    pub default: Option<Option<PyValue>>,
    /// `seps` (default `[","]`).
    pub seps: Vec<String>,
    /// `count`.
    pub count: Option<usize>,
    /// `parser`.
    pub parser: ListParser,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetListsOpts {
    fn default() -> Self {
        GetListsOpts {
            default: None,
            seps: vec![",".to_string()],
            count: None,
            parser: ListParser::Str,
            note_valid: true,
        }
    }
}

/// Options for [`ConfigApi::getlist`] (`configfile.py:107`).
#[derive(Clone, Debug)]
pub struct GetListOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<Vec<String>>>,
    /// `sep` (default `","`).
    pub sep: String,
    /// `count`.
    pub count: Option<usize>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetListOpts {
    fn default() -> Self {
        GetListOpts {
            default: None,
            sep: ",".to_string(),
            count: None,
            note_valid: true,
        }
    }
}

/// Options for [`ConfigApi::getintlist`] (`configfile.py:111`).
#[derive(Clone, Debug)]
pub struct GetIntListOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<Vec<i64>>>,
    /// `sep` (default `","`).
    pub sep: String,
    /// `count`.
    pub count: Option<usize>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetIntListOpts {
    fn default() -> Self {
        GetIntListOpts {
            default: None,
            sep: ",".to_string(),
            count: None,
            note_valid: true,
        }
    }
}

/// Options for [`ConfigApi::getfloatlist`] (`configfile.py:115`).
#[derive(Clone, Debug)]
pub struct GetFloatListOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<Vec<f64>>>,
    /// `sep` (default `","`).
    pub sep: String,
    /// `count`.
    pub count: Option<usize>,
    /// `note_valid` (default `true`).
    pub note_valid: bool,
}

impl Default for GetFloatListOpts {
    fn default() -> Self {
        GetFloatListOpts {
            default: None,
            sep: ",".to_string(),
            count: None,
            note_valid: true,
        }
    }
}

/// Options for [`PrinterConfigApi::deprecate_gcode`] (`configfile.py:518`).
#[derive(Clone, Debug, Default)]
pub struct DeprecateGcodeOpts {
    /// `param`.
    pub param: Option<String>,
    /// `value` (any Python value).
    pub value: Option<PyValue>,
    /// `msg` (default message when `None`).
    pub msg: Option<String>,
}

/// The `ConfigWrapper` service (`klippy/configfile.py:18`).
pub trait ConfigApi: Send + Sync + 'static {
    /// `configfile.py:25`.
    fn get_printer(&self) -> Printer;
    /// `configfile.py:27`.
    fn get_name(&self) -> String;
    /// `configfile.py:61`.
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>>;
    /// `configfile.py:64`.
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>>;
    /// `configfile.py:68`.
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>>;
    /// `configfile.py:73`.
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>>;
    /// `configfile.py:76`.
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue>;
    /// `configfile.py:87`.
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue>;
    /// `configfile.py:107`.
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>>;
    /// `configfile.py:111`.
    fn getintlist(&self, option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>>;
    /// `configfile.py:115`.
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>>;
    /// `configfile.py:119`.
    fn getsection(&self, section: &str) -> Config;
    /// `configfile.py:122`.
    fn has_section(&self, section: &str) -> bool;
    /// `configfile.py:124`.
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config>;
    /// `configfile.py:127`.
    fn get_prefix_options(&self, prefix: &str) -> Vec<String>;
    /// `configfile.py:130`.
    fn deprecate(&self, option: &str, value: Option<PyValue>);
}

/// A `ConfigWrapper` view (`ConfigApi` object).
pub type Config = Arc<dyn ConfigApi>;

/// The `configfile` printer object service (`klippy/configfile.py:459`).
pub trait PrinterConfigApi: Send + Sync + 'static {
    /// `configfile.py:467`.
    fn get_printer(&self) -> Printer;
    /// `configfile.py:469`.
    fn read_config(&self, filename: &str) -> KResult<Config>;
    /// `configfile.py:474`.
    fn read_main_config(&self) -> KResult<Config>;
    /// `configfile.py:482`.
    fn log_config(&self, config: &Config);
    /// `configfile.py:488`.
    fn check_unused_options(&self, config: &Config) -> KResult<()>;
    /// `configfile.py:498`.
    fn runtime_warning(&self, msg: &str);
    /// `configfile.py:503`.
    fn deprecate(&self, section: &str, option: &str, value: Option<PyValue>, msg: Option<String>);
    /// `configfile.py:518`.
    fn deprecate_gcode(&self, cmd: &str, opts: DeprecateGcodeOpts);
    /// `configfile.py:532`.
    fn deprecate_mcu_code(&self, mcu: &Mcu, feature: &str, msg: Option<String>);
    /// `configfile.py:550`.
    fn get_status(&self, eventtime: Option<f64>) -> PyValue;
    /// `configfile.py:557`.
    fn set(&self, section: &str, option: &str, value: &str);
    /// `configfile.py:559`.
    fn remove_section(&self, section: &str);
}

/// A `PrinterConfig` object (`PrinterConfigApi` object).
pub type PrinterConfig = Arc<dyn PrinterConfigApi>;
