//! Port of the `save_variables` module (`klippy/extras/save_variables.py`).
//!
//! Persists arbitrary G-code command parameters to an ini-style file so
//! values survive restarts. The file is a plain `configparser.ConfigParser`
//! with one `[Variables]` section; each variable is written as
//! `name = repr(value)` (option names are lowercased by `optionxform`) in
//! name order, and read back through `ast.literal_eval` (the shared
//! [`extras_macros_templates::gcode_macro::parse_literal`] port).

use extras_macros_templates::gcode_macro::parse_literal;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GetOpts, GetParser, GCodeApi, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::text::str_repr;
use pycompat::PyValue;
use pytext::configparser::{ConfigParser, Error as ConfigParserError};
use std::io::Write;
use std::path::Path;
use std::sync::Arc;

/// `SaveVariables.cmd_SAVE_VARIABLE_help`
/// (`klippy/extras/save_variables.py:36`).
pub const CMD_SAVE_VARIABLE_HELP: &str = "Save arbitrary variables to disk";

/// `klippy/extras/save_variables.py:SaveVariables`.
#[allow(dead_code)]
pub struct SaveVariables {
    printer: Printer,
    filename: String,
    all_variables: GCell<IndexMap<String, PyValue>>,
}

impl SaveVariables {
    /// `SaveVariables.__init__(config)` (`klippy/extras/save_variables.py:10`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let filename_raw = config.get("filename", GetStrOpts::default())?.unwrap_or_default();
        let filename = expand_user(&filename_raw);
        let obj = SaveVariables {
            printer,
            filename,
            all_variables: GCell::new(IndexMap::new()),
        };
        // `try:` block of `__init__` (`klippy/extras/save_variables.py:14`).
        // `open(self.filename, "w").close()` runs only when the file does not
        // exist; an `OSError` there is not a `command_error`, so it
        // propagates untouched.
        if !Path::new(&obj.filename).exists() {
            create_empty_file(&obj.filename)?;
        }
        // `except self.printer.command_error as e: raise config.error(str(e))`.
        obj.load_variables().map_err(|e| match e {
            KlippyError::Command(msg) => KlippyError::Config(msg),
            other => other,
        })?;
        Ok(obj)
    }

    /// `SaveVariables.loadVariables()`
    /// (`klippy/extras/save_variables.py:23`). Every failure is a
    /// `command_error("Unable to parse existing variable file")`.
    fn load_variables(&self) -> KResult<()> {
        const PARSE_MSG: &str = "Unable to parse existing variable file";
        let mut varfile = ConfigParser::new_interpolating();
        // `varfile.read()` silently skips a file that cannot be opened.
        let result = (|| -> Result<IndexMap<String, PyValue>, String> {
            varfile.read(&self.filename).map_err(|e| e.to_string())?;
            let mut allvars = IndexMap::new();
            if varfile.has_section("Variables") {
                for (name, val) in varfile.items("Variables").map_err(|e| e.to_string())? {
                    // A valueless option cannot occur (`allow_no_value` is
                    // false), but `literal_eval(None)` would fail the same way.
                    let val = val.ok_or_else(|| "None".to_string())?;
                    // `ast.literal_eval(val)`: any `ValueError` or
                    // `SyntaxError` lands in the bare `except:` below.
                    let v = parse_literal(&val).map_err(|e| e.display())?;
                    allvars.insert(name, v);
                }
            }
            Ok(allvars)
        })();
        match result {
            Ok(allvars) => {
                self.all_variables.replace(allvars);
                Ok(())
            }
            Err(msg) => {
                log::exception(PARSE_MSG, &msg);
                Err(KlippyError::command(PARSE_MSG))
            }
        }
    }

    /// `SaveVariables.cmd_SAVE_VARIABLE(gcmd)`
    /// (`klippy/extras/save_variables.py:37`).
    pub fn cmd_save_variable(&self, gcmd: &GCmd) -> KResult<()> {
        let var_name = gcmd
            .get(
                "VARIABLE",
                GetOpts { default: None, parser: GetParser::Str, ..Default::default() },
            )?
            .str();
        if var_name.to_lowercase() != var_name {
            return Err(KlippyError::command("VARIABLE must not contain upper case"));
        }
        let value_text = gcmd
            .get(
                "VALUE",
                GetOpts { default: None, parser: GetParser::Str, ..Default::default() },
            )?
            .str();
        // `except ValueError as e: raise gcmd.error(...)`; other literal
        // errors (e.g. `SyntaxError`) propagate as internal errors.
        let value = match parse_literal(&value_text) {
            Ok(v) => v,
            Err(e) if e.kind == "ValueError" => {
                return Err(KlippyError::command(format!(
                    "Unable to parse '{value_text}' as a literal"
                )));
            }
            Err(e) => {
                return Err(KlippyError::Internal {
                    class_name: e.kind.to_string(),
                    message: e.msg,
                });
            }
        };
        let mut newvars = self.all_variables.cloned();
        newvars.insert(var_name, value);
        // Write file: fresh parser, entries in `sorted(newvars.items())`
        // order, i.e. name order (names in a dict are unique).
        let mut varfile = ConfigParser::new_interpolating();
        varfile.add_section("Variables").map_err(|e| config_parser_error(&e))?;
        // `set()` with interpolation raises `InterpolationSyntaxError` on bad
        // `%` syntax; like Python it propagates as an internal error.
        let mut names: Vec<String> = newvars.keys().cloned().collect();
        names.sort();
        for name in &names {
            varfile.set("Variables", name, Some(newvars[name].repr()))
                .map_err(|e| config_parser_error(&e))?;
        }
        let text = varfile.write();
        let write_file = (|| -> std::io::Result<()> {
            let mut f = std::fs::File::create(&self.filename)?;
            f.write_all(text.as_bytes())?;
            Ok(())
        })();
        if let Err(e) = write_file {
            const SAVE_MSG: &str = "Unable to save variable";
            log::exception(SAVE_MSG, &e.to_string());
            return Err(KlippyError::command(SAVE_MSG));
        }
        self.load_variables()
    }
}

impl PrinterObject for SaveVariables {
    klippy_api::printer_object_base!();

    /// `get_status(eventtime)` (`klippy/extras/save_variables.py:62`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let variables = PyValue::Dict(
            self.all_variables.borrow()
                .iter()
                .map(|(k, v)| (PyValue::Str(k.clone()), v.clone()))
                .collect(),
        );
        Some(PyValue::Dict(IndexMap::from_iter([(PyValue::Str("variables".to_string()), variables)])))
    }
}

/// `load_config(config)` (`klippy/extras/save_variables.py:65`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = Arc::new(SaveVariables::new(config)?);
    let gcode = lookup_interface::<dyn GCodeApi>(&*obj.printer, "gcode", LookupDefault::Raise)?
        .expect("Raise never yields None");
    let handler_obj = Arc::clone(&obj);
    let _ = gcode.register_command(
        "SAVE_VARIABLE",
        Some(Box::new(move |gcmd: &GCmd| handler_obj.cmd_save_variable(gcmd))),
        RegisterCommandOpts {
            when_not_ready: false,
            desc: Some(CMD_SAVE_VARIABLE_HELP.to_string()),
        },
    )?;
    Ok(Some(obj))
}

/// Module entry point for the `save_variables` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}

/// A `configparser` error as the Python exception would surface out of
/// `cmd_SAVE_VARIABLE` (an internal error; `str(exc)` is the message).
fn config_parser_error(e: &ConfigParserError) -> KlippyError {
    use ConfigParserError as E;
    let class_name = match e {
        E::Error(_) => "Error",
        E::DuplicateSectionError { .. } => "DuplicateSectionError",
        E::DuplicateOptionError { .. } => "DuplicateOptionError",
        E::NoSectionError { .. } => "NoSectionError",
        E::NoOptionError { .. } => "NoOptionError",
        E::ParsingError { .. } => "ParsingError",
        E::MissingSectionHeaderError { .. } => "MissingSectionHeaderError",
        E::InterpolationMissingOptionError { .. } => "InterpolationMissingOptionError",
        E::InterpolationSyntaxError { .. } => "InterpolationSyntaxError",
        E::InterpolationDepthError { .. } => "InterpolationDepthError",
        E::ValueError(_) => "ValueError",
    };
    KlippyError::Internal { class_name: class_name.to_string(), message: e.to_string() }
}

/// `open(filename, "w").close()` with the Python `OSError` shape
/// (`str(exc)` is `[Errno N] <strerror>: '<filename>'`).
fn create_empty_file(filename: &str) -> KResult<()> {
    std::fs::File::create(filename).map_err(|e| python_os_error(filename, &e))?;
    Ok(())
}

fn python_os_error(filename: &str, e: &std::io::Error) -> KlippyError {
    let Some(errno) = e.raw_os_error() else {
        // `open()` with an embedded NUL raises `ValueError` in Python.
        return KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "embedded null byte".to_string(),
        };
    };
    let class_name = match errno {
        1 | 13 => "PermissionError",
        2 => "FileNotFoundError",
        17 => "FileExistsError",
        20 => "NotADirectoryError",
        21 => "IsADirectoryError",
        _ => "OSError",
    };
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: format!("[Errno {errno}] {}: {}", kos::fd::strerror(errno), str_repr(filename)),
    }
}

/// `os.path.expanduser(path)` (`posixpath.py`, CPython 3.8).
fn expand_user(path: &str) -> String {
    if !path.starts_with('~') {
        return path.to_string();
    }
    let i = path[1..].find('/').map(|j| 1 + j).unwrap_or(path.len());
    if i == 1 {
        // `~` or `~/...`: `$HOME`, else the current user's pw_dir. An empty
        // HOME (`'HOME' in os.environ` but blank) leaves the path unchanged.
        let home = match std::env::var_os("HOME") {
            Some(v) => {
                let s = v.into_string().unwrap_or_default();
                if s.is_empty() {
                    None
                } else {
                    Some(s)
                }
            }
            None => pwdir_current(),
        };
        match home {
            Some(h) => format!("{h}{}", &path[i..]),
            None => path.to_string(),
        }
    } else {
        // `~user` / `~user/...`: unknown users leave the path unchanged.
        let user = &path[1..i];
        match pwdir_of(user) {
            Some(h) => format!("{h}{}", &path[i..]),
            None => path.to_string(),
        }
    }
}

#[cfg(unix)]
fn pwdir_of(user: &str) -> Option<String> {
    use std::ffi::CString;
    let name = CString::new(user).ok()?;
    unsafe {
        let pw = getpwnam(name.as_ptr());
        pw_dir(pw)
    }
}

#[cfg(not(unix))]
fn pwdir_of(_user: &str) -> Option<String> {
    None
}

#[cfg(unix)]
fn pwdir_current() -> Option<String> {
    unsafe {
        let pw = getpwuid(getuid());
        pw_dir(pw)
    }
}

#[cfg(not(unix))]
fn pwdir_current() -> Option<String> {
    None
}

#[cfg(unix)]
#[repr(C)]
struct Passwd {
    pw_name: *const std::os::raw::c_char,
    pw_passwd: *const std::os::raw::c_char,
    pw_uid: u32,
    pw_gid: u32,
    pw_gecos: *const std::os::raw::c_char,
    pw_dir: *const std::os::raw::c_char,
    pw_shell: *const std::os::raw::c_char,
}

/// Read `pw_dir` from a `struct passwd*` (null → None).
#[cfg(unix)]
unsafe fn pw_dir(pw: *mut Passwd) -> Option<String> {
    if pw.is_null() {
        return None;
    }
    let dir = unsafe { (*pw).pw_dir };
    if dir.is_null() {
        return None;
    }
    let cstr = unsafe { std::ffi::CStr::from_ptr(dir) };
    Some(cstr.to_string_lossy().into_owned())
}

#[cfg(unix)]
unsafe extern "C" {
    fn getpwnam(name: *const std::os::raw::c_char) -> *mut Passwd;
    fn getpwuid(uid: u32) -> *mut Passwd;
    fn getuid() -> u32;
}
