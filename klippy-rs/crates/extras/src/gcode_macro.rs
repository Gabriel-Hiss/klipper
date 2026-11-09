//! Port of `klippy/extras/gcode_macro.py`.
//!
//! The Jinja2 template engine lives in `extras_macros_templates`
//! (`TemplateWrapper`, `GCodeMacroVars`, `parse_literal`). This module
//! owns the printer-facing objects: `PrinterGCodeMacro`
//! (`load_template`, `create_template_context`), the printer-bound
//! `TemplateWrapper` (`render`, `run_gcode_from_command`), and
//! `GCodeMacro` (`[gcode_macro NAME]` sections, `SET_GCODE_VARIABLE`).

use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandHandler, GetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::printer::{EV_KLIPPY_CONNECT, LookupDefault, ModuleFactory, Printer};
use klippy_api::webhooks::WebHooksApi;
use klippy_api::{GCell, ObjectRef, PrinterObject, load_interface, lookup_interface};
use pycompat::PyValue;
use std::sync::Arc;

use extras_macros_templates::gcode_macro as tmpl;
use extras_macros_templates::gcode_macro::{TemplateError, Value};
use pycompat::text::PyError;

/// Help text of the `SET_GCODE_VARIABLE` mux command
/// (`klippy/extras/gcode_macro.py:174`).
pub const SET_GCODE_VARIABLE_HELP: &str = "Set the value of a G-Code macro variable";

/// Python `str(e)` of a caught literal error (`klippy/extras/gcode_macro.py:159`
/// and `:183`): the parser's message, except that a bare-name
/// `ValueError` matches CPython 3.8 `ast.literal_eval`, which reports the
/// unconverted node as
/// `malformed node or string: <_ast.Name object at 0x...>`; the address is
/// allocator-specific, so it is rendered as `0x0` (tests normalize it).
fn literal_py_str(error: &PyError, src: &str) -> String {
    if error.kind == "ValueError" && error.msg.starts_with("malformed node or string") {
        let t = src.trim();
        let bare_name = !t.is_empty()
            && !matches!(t, "True" | "False" | "None")
            && t.chars().next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
            && t.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
        if bare_name {
            return "malformed node or string: <_ast.Name object at 0x0>".to_string();
        }
    }
    error.msg.clone()
}

/// Rebuild an `extras_macros_templates` error string in the form the
/// reference produces with `traceback.format_exception_only(type(e), e)[-1]`
/// (`klippy/extras/gcode_macro.py:61` and `:72`): the exception class is
/// module-qualified as Python renders it (`gcode.CommandError` for
/// `printer.command_error`/`gcode.error`, `jinja2.exceptions.*` for engine
/// errors, builtins unqualified) and the line ends with a newline.
fn exception_only_tail(msg: &str) -> String {
    let Some(sep) = msg.find("': ") else {
        return format!("{msg}\n");
    };
    let prefix = &msg[..sep + 1];
    let rest = &msg[sep + 3..];
    let Some(colon) = rest.find(": ") else {
        return format!("{msg}\n");
    };
    let class = &rest[..colon];
    let detail = &rest[colon + 2..];
    let qualified = match class {
        "CommandError" => "gcode.CommandError",
        "UndefinedError" => "jinja2.exceptions.UndefinedError",
        "TemplateSyntaxError" => "jinja2.exceptions.TemplateSyntaxError",
        other => other,
    };
    format!("{prefix}: {qualified}: {detail}\n")
}

// ---------------------------------------------------------------------------
// Template context (`PrinterGCodeMacro.create_template_context`)
// ---------------------------------------------------------------------------

/// `str(value)` of a template value, as `'%s'` would format it.
fn value_to_str(v: &Value) -> String {
    match v {
        Value::Py(p) => p.str(),
        other => other.to_string(),
    }
}

/// A template value as the `PyValue` it wraps; functions and undefined
/// values cross the boundary as strings.
fn value_to_pyvalue(v: &Value) -> PyValue {
    match v {
        Value::Py(p) => p.clone(),
        other => PyValue::Str(other.to_string()),
    }
}

fn type_error(message: String, lineno: usize) -> TemplateError {
    TemplateError::new("TypeError", message, Some(lineno))
}

fn missing_arg(func: &str, name: &str, lineno: usize) -> TemplateError {
    type_error(
        format!("{func}() missing 1 required positional argument: '{name}'"),
        lineno,
    )
}

fn unexpected_kw(func: &str, kw: &str, lineno: usize) -> TemplateError {
    type_error(
        format!("{func}() got an unexpected keyword argument '{kw}'"),
        lineno,
    )
}

/// Human-readable snapshot of the printer statuses for one
/// `create_template_context` call.
///
/// Python resolves statuses lazily per `GetStatusWrapper` access (and
/// caches a deep copy); the template engine consumes an eager dict, so
/// we snapshot every object with `get_status` at the same eventtime.
/// `PyValue` clones are deep copies.
fn printer_status_registry(printer: &Printer, eventtime: Option<f64>) -> IndexMap<String, PyValue> {
    let reactor = printer.get_reactor();
    let effective = eventtime.unwrap_or_else(|| reactor.monotonic());
    let guard = reactor.assert_no_pause();
    let mut registry: IndexMap<String, PyValue> = IndexMap::new();
    for (name, obj) in printer.lookup_objects(None) {
        let _guard = &guard;
        if let Some(sts) = obj.get_status(effective) {
            registry.insert(name, sts);
        }
    }
    drop(guard);
    registry
}

/// `PrinterGCodeMacro.create_template_context`
/// (`klippy/extras/gcode_macro.py:106`).
pub fn create_template_context(
    printer: &Printer,
    eventtime: Option<f64>,
) -> IndexMap<String, Value> {
    let registry = printer_status_registry(printer, eventtime);
    let mut ctx: IndexMap<String, Value> = IndexMap::new();
    ctx.insert(
        "printer".to_string(),
        Value::py(tmpl::make_printer_dict(&registry)),
    );

    {
        let es_printer = printer.clone();
        ctx.insert(
            "action_emergency_stop".to_string(),
            Value::Func(std::rc::Rc::new(
                move |args: &[Value], kwargs: &[(String, Value)], lineno: usize| {
                    if let Some((kw, _)) = kwargs.first() {
                        return Err(unexpected_kw("action_emergency_stop", kw, lineno));
                    }
                    let msg = match args.len() {
                        0 => "action_emergency_stop".to_string(),
                        1 => value_to_str(&args[0]),
                        n => {
                            return Err(type_error(
                                format!(
                                    "action_emergency_stop() takes from 0 to 1 positional arguments but {n} were given"
                                ),
                                lineno,
                            ));
                        }
                    };
                    es_printer.invoke_shutdown(
                        &format!("Shutdown due to {msg}"),
                        PyValue::Dict(IndexMap::new()),
                    );
                    Ok(Value::py(PyValue::Str(String::new())))
                },
            )),
        );
    }

    {
        let ri_printer = printer.clone();
        ctx.insert(
            "action_respond_info".to_string(),
            Value::Func(std::rc::Rc::new(
                move |args: &[Value], kwargs: &[(String, Value)], lineno: usize| {
                    if let Some((kw, _)) = kwargs.first() {
                        return Err(unexpected_kw("action_respond_info", kw, lineno));
                    }
                    let msg = match args.len() {
                        0 => return Err(missing_arg("action_respond_info", "msg", lineno)),
                        1 => value_to_str(&args[0]),
                        n => {
                            return Err(type_error(
                                format!(
                                    "action_respond_info() takes 1 positional argument but {n} were given"
                                ),
                                lineno,
                            ));
                        }
                    };
                    let gcode = match lookup_interface::<dyn GCodeApi>(
                        &*ri_printer,
                        "gcode",
                        LookupDefault::Raise,
                    ) {
                        Ok(Some(g)) => g,
                        Ok(None) => {
                            return Err(TemplateError::new(
                                "Error",
                                "Unknown config object 'gcode'".to_string(),
                                Some(lineno),
                            ));
                        }
                        Err(e) => {
                            return Err(TemplateError::new(
                                e.class_name(),
                                e.message().to_string(),
                                Some(lineno),
                            ));
                        }
                    };
                    gcode.respond_info(&msg, true);
                    Ok(Value::py(PyValue::Str(String::new())))
                },
            )),
        );
    }

    ctx.insert(
        "action_raise_error".to_string(),
        Value::Func(std::rc::Rc::new(
            move |args: &[Value], kwargs: &[(String, Value)], lineno: usize| {
                if let Some((kw, _)) = kwargs.first() {
                    return Err(unexpected_kw("action_raise_error", kw, lineno));
                }
                let msg = match args.len() {
                    0 => return Err(missing_arg("action_raise_error", "msg", lineno)),
                    1 => value_to_str(&args[0]),
                    n => {
                        return Err(type_error(
                            format!(
                                "action_raise_error() takes 1 positional argument but {n} were given"
                            ),
                            lineno,
                        ));
                    }
                };
                Err(TemplateError::new("CommandError", msg, Some(lineno)))
            },
        )),
    );

    {
        let crm_printer = printer.clone();
        ctx.insert(
            "action_call_remote_method".to_string(),
            Value::Func(std::rc::Rc::new(
                move |args: &[Value], kwargs: &[(String, Value)], lineno: usize| {
                    let method = match args.len() {
                        0 => {
                            return Err(missing_arg(
                                "action_call_remote_method",
                                "method",
                                lineno,
                            ));
                        }
                        1 => value_to_str(&args[0]),
                        n => {
                            return Err(type_error(
                                format!(
                                    "action_call_remote_method() takes 1 positional argument but {n} were given"
                                ),
                                lineno,
                            ));
                        }
                    };
                    let mut params: IndexMap<String, PyValue> = IndexMap::new();
                    for (k, v) in kwargs {
                        params.insert(k.clone(), value_to_pyvalue(v));
                    }
                    let webhooks = match lookup_interface::<dyn WebHooksApi>(
                        &*crm_printer,
                        "webhooks",
                        LookupDefault::Raise,
                    ) {
                        Ok(Some(w)) => w,
                        Ok(None) => {
                            return Err(TemplateError::new(
                                "Error",
                                "Unknown config object 'webhooks'".to_string(),
                                Some(lineno),
                            ));
                        }
                        Err(e) => {
                            return Err(TemplateError::new(
                                e.class_name(),
                                e.message().to_string(),
                                Some(lineno),
                            ));
                        }
                    };
                    match webhooks.call_remote_method(&method, &params) {
                        Ok(()) => Ok(Value::py(PyValue::Str(String::new()))),
                        Err(KlippyError::Command(_)) => {
                            klippy_api::log::exception("Remote Call Error", "Remote Call Error");
                            Ok(Value::py(PyValue::Str(String::new())))
                        }
                        Err(e) => Err(TemplateError::new(
                            e.class_name(),
                            e.message().to_string(),
                            Some(lineno),
                        )),
                    }
                },
            )),
        );
    }

    ctx
}

// ---------------------------------------------------------------------------
// `TemplateWrapper` (printer-bound)
// ---------------------------------------------------------------------------

/// Printer-bound wrapper around a Jinja2 template
/// (`klippy/extras/gcode_macro.py:46`).
pub struct TemplateWrapper {
    name: String,
    printer: Printer,
    gcode: GCode,
    inner: tmpl::TemplateWrapper,
}

impl TemplateWrapper {
    /// `TemplateWrapper(printer, env, name, script)`
    /// (`klippy/extras/gcode_macro.py:47`).
    pub fn new(printer: &Printer, name: &str, script: &str) -> KResult<TemplateWrapper> {
        let gcode = match lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)
        {
            Ok(Some(g)) => g,
            Ok(None) => {
                return Err(KlippyError::config("Unknown config object 'gcode'"));
            }
            Err(e) => return Err(e),
        };
        let inner = match tmpl::TemplateWrapper::load(name, script) {
            Ok(w) => w,
            Err(msg) => {
                let is_syntax = tmpl::Template::parse(script)
                    .err()
                    .is_some_and(|e| e.is_syntax_error());
                let msg = if is_syntax { msg } else { exception_only_tail(&msg) };
                klippy_api::log::exception(&msg, &msg);
                if is_syntax {
                    return Err(KlippyError::command(msg));
                }
                return Err(KlippyError::config(msg));
            }
        };
        Ok(TemplateWrapper {
            name: name.to_string(),
            printer: printer.clone(),
            gcode,
            inner,
        })
    }

    /// `TemplateWrapper.name`
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Retained template source, for tests and introspection.
    pub fn source(&self) -> &str {
        self.inner.source()
    }

    /// `TemplateWrapper.create_template_context`
    /// (`klippy/extras/gcode_macro.py:52`).
    pub fn create_template_context(&self, eventtime: Option<f64>) -> IndexMap<String, Value> {
        create_template_context(&self.printer, eventtime)
    }

    /// `TemplateWrapper.render(context=None)`
    /// (`klippy/extras/gcode_macro.py:66`).
    pub fn render(&self, context: Option<IndexMap<String, Value>>) -> KResult<String> {
        let ctx = match context {
            Some(c) => c,
            None => self.create_template_context(None),
        };
        self.inner.render(ctx).map_err(|msg| {
            let msg = exception_only_tail(&msg);
            klippy_api::log::exception(&msg, &msg);
            KlippyError::command(msg)
        })
    }

    /// `TemplateWrapper.run_gcode_from_command(context=None)`
    /// (`klippy/extras/gcode_macro.py:76`).
    /// Pauses the reactor (via `gcode.run_script_from_command`).
    pub fn run_gcode_from_command(&self, context: Option<IndexMap<String, Value>>) -> KResult<()> {
        let script = self.render(context)?;
        self.gcode.run_script_from_command(&script)
    }
}

// ---------------------------------------------------------------------------
// `PrinterGCodeMacro`
// ---------------------------------------------------------------------------

/// The object behind `crate::gcode_macro::PrinterGCodeMacroApi`:
/// `probe`, `homing_override`, `idle_timeout`, `delayed_gcode`,
/// `gcode_button` and the display menus reach the `gcode_macro`
/// printer object through this trait (`klippy/extras/gcode_macro.py:80`).
pub trait PrinterGCodeMacroApi: Send + Sync + 'static {
    /// `PrinterGCodeMacro.load_template(config, option, default=None)`
    /// (`klippy/extras/gcode_macro.py:84`). `default` of `None` makes
    /// the option required, matching the Python sentinel.
    fn load_template(
        &self,
        config: &Config,
        option: &str,
        default: Option<&str>,
    ) -> KResult<TemplateWrapper>;

    /// `PrinterGCodeMacro.create_template_context(eventtime=None)`
    /// (`klippy/extras/gcode_macro.py:106`).
    fn create_template_context(&self, eventtime: Option<f64>) -> IndexMap<String, Value>;
}

/// `PrinterGCodeMacro` (`klippy/extras/gcode_macro.py:80`). Indivisible
/// unit: one shared template environment and `load_template`.
pub struct PrinterGCodeMacro {
    printer: Printer,
}

impl PrinterGCodeMacro {
    /// `PrinterGCodeMacro.__init__(config)`
    /// (`klippy/extras/gcode_macro.py:81`).
    pub fn new(config: &Config) -> PrinterGCodeMacro {
        PrinterGCodeMacro {
            printer: config.get_printer(),
        }
    }
}

impl PrinterGCodeMacroApi for PrinterGCodeMacro {
    fn load_template(
        &self,
        config: &Config,
        option: &str,
        default: Option<&str>,
    ) -> KResult<TemplateWrapper> {
        let name = format!("{}:{}", config.get_name(), option);
        let script = match default {
            Some(d) => {
                let opts = GetStrOpts {
                    default: Some(Some(d.to_string())),
                    note_valid: true,
                };
                match config.get(option, opts)? {
                    Some(v) => v,
                    None => d.to_string(),
                }
            }
            None => {
                let v = config.get(option, GetStrOpts::default())?;
                match v {
                    Some(s) => s,
                    None => {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must be specified",
                            option,
                            config.get_name()
                        )));
                    }
                }
            }
        };
        TemplateWrapper::new(&self.printer, &name, &script)
    }

    fn create_template_context(&self, eventtime: Option<f64>) -> IndexMap<String, Value> {
        create_template_context(&self.printer, eventtime)
    }
}

klippy_api::printer_object!(PrinterGCodeMacro; dyn PrinterGCodeMacroApi);

// ---------------------------------------------------------------------------
// `GCodeMacro`
// ---------------------------------------------------------------------------

fn required_string(config: &Config, option: &str) -> KResult<String> {
    match config.get(option, GetStrOpts::default())? {
        Some(s) => Ok(s),
        None => Err(KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option,
            config.get_name()
        ))),
    }
}

fn gcode_string(gcmd: &GCmd, name: &str) -> KResult<String> {
    let v = gcmd.get(name, GetOpts::default())?;
    Ok(match v {
        PyValue::Str(s) => s,
        other => other.str(),
    })
}

fn cmd_handler(macro_arc: &Arc<GCodeMacro>) -> GCodeCommandHandler {
    let m = macro_arc.clone();
    Box::new(move |gcmd: &GCmd| m.cmd(gcmd))
}

fn set_var_handler(macro_arc: &Arc<GCodeMacro>) -> GCodeCommandHandler {
    let m = macro_arc.clone();
    Box::new(move |gcmd: &GCmd| m.cmd_SET_GCODE_VARIABLE(gcmd))
}

/// `GCodeMacro` (`klippy/extras/gcode_macro.py:123`).
pub struct GCodeMacro {
    alias: String,
    name: String,
    #[allow(dead_code)]
    printer: Printer,
    gcode: GCode,
    template: TemplateWrapper,
    rename_existing: Option<String>,
    cmd_desc: String,
    variables: GCell<tmpl::GCodeMacroVars>,
    in_script: GCell<bool>,
    self_ref: GCell<Option<Arc<GCodeMacro>>>,
}

impl GCodeMacro {
    /// `load_config_prefix` plus `GCodeMacro.__init__`
    /// (`klippy/extras/gcode_macro.py:124`).
    pub fn new(config: &Config) -> KResult<Arc<GCodeMacro>> {
        let full_name = config.get_name();
        let tokens: Vec<&str> = full_name.split_whitespace().collect();
        if tokens.len() > 2 {
            return Err(KlippyError::config(format!(
                "Name of section '{}' contains illegal whitespace",
                full_name
            )));
        }
        let name = match tokens.get(1) {
            Some(n) => n.to_string(),
            None => {
                return Err(KlippyError::Other {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                });
            }
        };
        let alias = name.to_uppercase();
        let printer = config.get_printer();

        let gcode_macro = match load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )? {
            Some(g) => g,
            None => {
                return Err(KlippyError::config("Unknown config object 'gcode_macro'"));
            }
        };
        let template = gcode_macro.load_template(config, "gcode", None)?;
        let gcode = match lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)
        {
            Ok(Some(g)) => g,
            Ok(None) => {
                return Err(KlippyError::config("Unknown config object 'gcode'"));
            }
            Err(e) => return Err(e),
        };

        let rename_existing = config.get(
            "rename_existing",
            GetStrOpts {
                default: Some(None),
                note_valid: true,
            },
        )?;
        let cmd_desc = config.get(
            "description",
            GetStrOpts {
                default: Some(Some("G-Code macro".to_string())),
                note_valid: true,
            },
        )?;
        let cmd_desc = match cmd_desc {
            Some(d) => d,
            None => {
                return Err(KlippyError::config(format!(
                    "Option 'description' in section '{}' must be specified",
                    full_name
                )));
            }
        };

        if let Some(rename) = &rename_existing {
            if gcode.is_traditional_gcode(&alias) != gcode.is_traditional_gcode(rename) {
                return Err(KlippyError::config(format!(
                    "G-Code macro rename of different types ('{}' vs '{}')",
                    alias, rename
                )));
            }
        }

        let macro_arc = Arc::new(GCodeMacro {
            alias: alias.clone(),
            name: name.clone(),
            printer: printer.clone(),
            gcode: gcode.clone(),
            template,
            rename_existing,
            cmd_desc: cmd_desc.clone(),
            variables: GCell::new(tmpl::GCodeMacroVars::new(IndexMap::new())),
            in_script: GCell::new(false),
            self_ref: GCell::new(None),
        });
        macro_arc.self_ref.replace(Some(macro_arc.clone()));

        if macro_arc.rename_existing.is_some() {
            let m = macro_arc.clone();
            printer.register_event_handler(
                EV_KLIPPY_CONNECT,
                Box::new(move |_args| {
                    m.handle_connect()?;
                    Ok(PyValue::None)
                }),
            );
        } else {
            gcode.register_command(
                &alias,
                Some(cmd_handler(&macro_arc)),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(cmd_desc),
                },
            )?;
        }
        gcode.register_mux_command(
            "SET_GCODE_VARIABLE",
            "MACRO",
            Some(&name),
            set_var_handler(&macro_arc),
            Some(SET_GCODE_VARIABLE_HELP),
        )?;

        // `variable_*` options. Registration above happens first, so
        // a failure here leaves the registrations behind, as Python.
        let section = config.get_name();
        let mut vars_map: IndexMap<String, PyValue> = IndexMap::new();
        for option in config.get_prefix_options("variable_") {
            let raw = required_string(config, &option)?;
            let short = option["variable_".len()..].to_string();
            match tmpl::parse_literal(&raw) {
                Ok(v) => match tmpl::validate_literal_json(&v) {
                    Ok(()) => {
                        vars_map.insert(short, v);
                    }
                    Err(e) => {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' is not a valid literal: {}",
                            option,
                            section,
                            literal_py_str(&e, &raw)
                        )));
                    }
                },
                Err(e) => {
                    return Err(KlippyError::config(format!(
                        "Option '{}' in section '{}' is not a valid literal: {}",
                        option,
                        section,
                        literal_py_str(&e, &raw)
                    )));
                }
            }
        }
        macro_arc.variables.replace(tmpl::GCodeMacroVars::new(vars_map));
        Ok(macro_arc)
    }

    /// Upper-cased command name (`self.alias`).
    pub fn alias(&self) -> &str {
        &self.alias
    }

    /// Section macro name as written (`self.name` of the `MACRO=` mux
    /// value; the Python local `name`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.cmd_desc`.
    pub fn cmd_desc(&self) -> &str {
        &self.cmd_desc
    }

    /// `self.in_script` (the Python attribute).
    pub fn in_script(&self) -> bool {
        self.in_script.get()
    }

    /// Sets `self.in_script` (tests mirror `mac.in_script = True`).
    pub fn set_in_script(&self, value: bool) {
        self.in_script.set(value)
    }

    /// `GCodeMacro.handle_connect`
    /// (`klippy/extras/gcode_macro.py:163`).
    pub fn handle_connect(&self) -> KResult<()> {
        let rename = match &self.rename_existing {
            Some(r) => r.clone(),
            None => return Ok(()),
        };
        let prev = self
            .gcode
            .register_command(&self.alias, None, RegisterCommandOpts::default())?;
        let prev = match prev {
            Some(p) => p,
            None => {
                return Err(KlippyError::config(format!(
                    "Existing command '{}' not found in gcode_macro rename",
                    self.alias
                )));
            }
        };
        let pdesc = format!("Renamed builtin of '{}'", self.alias);
        self.gcode.register_command(
            &rename,
            Some(prev),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(pdesc),
            },
        )?;
        let macro_arc = {
            let guard = self.self_ref.borrow();
            match &*guard {
                Some(a) => a.clone(),
                None => {
                    return Err(KlippyError::Other {
                        class_name: "RuntimeError".to_string(),
                        message: "GCodeMacro object has no self reference".to_string(),
                    });
                }
            }
        };
        self.gcode.register_command(
            &self.alias,
            Some(cmd_handler(&macro_arc)),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(self.cmd_desc.clone()),
            },
        )?;
        Ok(())
    }

    /// `GCodeMacro.cmd_SET_GCODE_VARIABLE`
    /// (`klippy/extras/gcode_macro.py:175`).
    pub fn cmd_SET_GCODE_VARIABLE(&self, gcmd: &GCmd) -> KResult<()> {
        let variable = gcode_string(gcmd, "VARIABLE")?;
        let value = gcode_string(gcmd, "VALUE")?;
        let commandline = gcmd.get_commandline();
        {
            let vars = self.variables.borrow();
            if !vars.get_status().contains_key(&variable) {
                return Err(KlippyError::command(format!(
                    "Unknown gcode_macro variable '{}'",
                    variable
                )));
            }
        }
        let literal = match tmpl::parse_literal(&value) {
            Ok(v) => match tmpl::validate_literal_json(&v) {
                Ok(()) => v,
                Err(e) => {
                    return Err(KlippyError::command(format!(
                        "Unable to parse '{}' as a literal: {} in '{}'",
                        value,
                        literal_py_str(&e, &value),
                        commandline
                    )));
                }
            },
            Err(e) => {
                return Err(KlippyError::command(format!(
                    "Unable to parse '{}' as a literal: {} in '{}'",
                    value,
                    literal_py_str(&e, &value),
                    commandline
                )));
            }
        };
        let mut vars = self.variables.borrow_mut();
        let mut map = vars.get_status();
        map.insert(variable.clone(), literal);
        *vars = tmpl::GCodeMacroVars::new(map);
        Ok(())
    }

    /// `GCodeMacro.cmd` (`klippy/extras/gcode_macro.py:189`).
    /// Pauses the reactor (via `gcode.run_script_from_command`).
    pub fn cmd(&self, gcmd: &GCmd) -> KResult<()> {
        if self.in_script.get() {
            return Err(KlippyError::command(format!(
                "Macro {} called recursively",
                self.alias
            )));
        }
        let rendered_ctx = {
            let vars = self.variables.borrow();
            vars.merge_context(
                self.template.create_template_context(None),
                gcmd.get_command_parameters(),
                &gcmd.get_raw_command_parameters(),
            )
        };
        self.in_script.set(true);
        let result = self.template.run_gcode_from_command(Some(rendered_ctx));
        self.in_script.set(false);
        result
    }
}

impl PrinterObject for GCodeMacro {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    /// `GCodeMacro.get_status(eventtime)`
    /// (`klippy/extras/gcode_macro.py:172`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let vars = self.variables.borrow();
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in vars.get_status() {
            d.insert(PyValue::Str(k), v);
        }
        Some(PyValue::Dict(d))
    }
}

/// `load_config(config)` (`klippy/extras/gcode_macro.py:115`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterGCodeMacro::new(config)) as ObjectRef))
}

/// `load_config_prefix(config)` (`klippy/extras/gcode_macro.py:202`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(GCodeMacro::new(config)? as ObjectRef))
}

/// Module entry point for the `gcode_macro` (and `gcode_macro NAME`)
/// sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
