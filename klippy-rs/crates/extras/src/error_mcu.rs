//! Port of `error_mcu` (`klippy/extras/error_mcu.py`).
//!
//! Rewrites shutdown / connect error messages with verbose hints and
//! reports reactor garbage-collection stats after each analyze-shutdown
//! event.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::object::EventArg;
use klippy_api::printer::{EventCallback, EV_KLIPPY_ANALYZE_SHUTDOWN, EV_KLIPPY_NOTIFY_MCU_ERROR, ModuleFactory, Printer};
use pycompat::text;
use pycompat::value::{py_equal, type_name};
use pycompat::PyValue;
use std::sync::Arc;

pub const MESSAGE_SHUTDOWN: &str = "\nOnce the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.\nPrinter is shutdown\n";
pub const MESSAGE_PROTOCOL_ERROR1: &str = "\nThis is frequently caused by running an older version of the\nfirmware on the MCU(s). Fix by recompiling and flashing the\nfirmware.\n";
pub const MESSAGE_PROTOCOL_ERROR2: &str = "\nOnce the underlying issue is corrected, use the \"RESTART\"\ncommand to reload the config and restart the host software.\n";
pub const MESSAGE_MCU_CONNECT_ERROR: &str = "\nOnce the underlying issue is corrected, use the\n\"FIRMWARE_RESTART\" command to reset the firmware, reload the\nconfig, and restart the host software.\nError configuring printer\n";

const COMMON_MCU_ERRORS: &[(&[&str], &str)] = &[
    (
        &["Timer too close"],
        "\nThis often indicates the host computer is overloaded. Check\nfor other processes consuming excessive CPU time, high swap\nusage, disk errors, overheating, unstable voltage, or\nsimilar system problems on the host computer.",
    ),
    (
        &["Missed scheduling of next "],
        "\nThis is generally indicative of an intermittent\ncommunication failure between micro-controller and host.",
    ),
    (
        &["ADC out of range"],
        "\nThis generally occurs when a heater temperature exceeds\nits configured min_temp or max_temp.",
    ),
    (
        &["Rescheduled timer in the past", "Stepper too far in past"],
        "\nThis generally occurs when the micro-controller has been\nrequested to step at a rate higher than it is capable of\nobtaining.",
    ),
    (
        &["Command request"],
        "\nThis generally occurs in response to an M112 G-Code command\nor in response to an internal error in the host software.",
    ),
];

/// `error_hint(msg)` (`error_mcu.py:54`).
pub fn error_hint(msg: &str) -> &'static str {
    for (prefixes, help_msg) in COMMON_MCU_ERRORS {
        for prefix in *prefixes {
            if msg.starts_with(prefix) {
                return help_msg;
            }
        }
    }
    ""
}

fn key_error(key: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: pycompat::value::repr_key(key),
    }
}

fn dict_get(details: &PyValue, key: &str) -> KResult<PyValue> {
    match details {
        PyValue::Dict(d) => d
            .get(&PyValue::Str(key.to_string()))
            .cloned()
            .ok_or_else(|| key_error(key)),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", type_name(details)),
        }),
    }
}

fn err_expected_str_for_startswith(v: &PyValue) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'{}' object has no attribute 'startswith'", type_name(v)),
    }
}

fn err_index() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

pub type ClarifyCallback = Arc<dyn Fn(&str, &PyValue) -> KResult<Option<String>> + Send + Sync>;

/// Narrow interface other extras call on the `PrinterMCUError` object
/// (the Python `add_clarify` duck type, e.g.
/// `klippy/extras/adc_temperature.py:54`).
pub trait PrinterMCUErrorApi: Send + Sync + 'static {
    /// `add_clarify(msg, callback)` (`error_mcu.py:69`).
    fn add_clarify(&self, msg: &str, callback: ClarifyCallback);
}

/// Alias for `PrinterMCUErrorApi` (Rust casing of `MCU`).
pub trait PrinterMcuErrorApi: PrinterMCUErrorApi {}

struct Shared {
    printer: Printer,
    clarify_callbacks: GCell<IndexMap<String, Vec<ClarifyCallback>>>,
}

/// The `PrinterMCUError` object (`error_mcu.py:61`).
pub struct PrinterMCUError {
    shared: Arc<Shared>,
}

/// Alias matching the previous Rust casing.
pub type PrinterMcuError = PrinterMCUError;

klippy_api::printer_object!(PrinterMCUError; dyn PrinterMCUErrorApi, dyn PrinterMcuErrorApi);

fn event_arg_to_value(arg: &EventArg) -> PyValue {
    match arg {
        EventArg::Value(v) => v.clone(),
        EventArg::Object(_) => PyValue::Host(0),
        EventArg::Any(_) => PyValue::Host(0),
        EventArg::List(_) => PyValue::List(Vec::new()),
    }
}


impl PrinterMCUError {
    /// `PrinterMCUError.__init__` (`error_mcu.py:62`).
    pub fn new(config: &Config) -> Self {
        let shared = Arc::new(Shared {
            printer: config.get_printer(),
            clarify_callbacks: GCell::new(IndexMap::new()),
        });
        let this = Arc::new(PrinterMCUError { shared: shared.clone() });
        let analyze_this = this.clone();
        let analyze_cb: EventCallback = Box::new(move |args: &[EventArg]| {
            if args.len() != 2 {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "_handle_analyze_shutdown() takes 3 positional arguments but {} were given",
                        args.len() + 1
                    ),
                });
            }
            let msg = event_arg_to_value(&args[0]);
            let details = event_arg_to_value(&args[1]);
            analyze_this.handle_analyze_shutdown(&msg, &details)?;
            Ok(PyValue::None)
        });
        shared.printer.register_event_handler(EV_KLIPPY_ANALYZE_SHUTDOWN, analyze_cb);
        let notify_this = this.clone();
        let notify_cb: EventCallback = Box::new(move |args: &[EventArg]| {
            if args.len() != 2 {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "_handle_notify_mcu_error() takes 3 positional arguments but {} were given",
                        args.len() + 1
                    ),
                });
            }
            let msg = event_arg_to_value(&args[0]);
            let details = event_arg_to_value(&args[1]);
            notify_this.handle_notify_mcu_error(&msg, &details)?;
            Ok(PyValue::None)
        });
        shared.printer.register_event_handler(EV_KLIPPY_NOTIFY_MCU_ERROR, notify_cb);
        PrinterMCUError { shared }
    }

    /// `add_clarify` (`error_mcu.py:69`).
    pub fn add_clarify(&self, msg: &str, callback: ClarifyCallback) {
        let mut map = self.shared.clarify_callbacks.borrow_mut();
        map.entry(msg.to_string()).or_default().push(callback);
    }

    fn check_mcu_shutdown(&self, msg: &PyValue, details: &PyValue) -> KResult<()> {
        let mcu_name = dict_get(details, "mcu")?;
        let mcu_msg_v = dict_get(details, "reason")?;
        let event_type = dict_get(details, "event_type")?;
        let prefix = if matches!(&event_type, PyValue::Str(s) if s == "is_shutdown") {
            format!("Previous MCU '{}' shutdown: ", mcu_name.str())
        } else {
            format!("MCU '{}' shutdown: ", mcu_name.str())
        };
        let mcu_msg = match &mcu_msg_v {
            PyValue::Str(s) => s.clone(),
            other => return Err(err_expected_str_for_startswith(other)),
        };
        let hint = error_hint(&mcu_msg);
        let cbs: Vec<ClarifyCallback> = {
            let map = self.shared.clarify_callbacks.borrow();
            map.get(&mcu_msg).cloned().unwrap_or_default()
        };
        let msg_str = msg.str();
        let mut clarify: Vec<String> = Vec::new();
        for cb in cbs.iter() {
            if let Some(cm) = cb(&msg_str, details)? {
                clarify.push(cm);
            }
        }
        let clarify_msg = if clarify.is_empty() {
            String::new()
        } else {
            let mut parts: Vec<&str> = vec!["", ""];
            parts.extend(clarify.iter().map(String::as_str));
            parts.push("");
            parts.join("\n")
        };
        let newmsg = format!("{}{}{}{}{}", prefix, mcu_msg, clarify_msg, hint, MESSAGE_SHUTDOWN);
        self.shared.printer.update_error_msg(&msg_str, &newmsg);
        Ok(())
    }

    /// `_handle_analyze_shutdown` (`error_mcu.py:91`); exposed for tests.
    pub fn handle_analyze_shutdown(&self, msg: &PyValue, details: &PyValue) -> KResult<()> {
        if matches!(msg, PyValue::Str(s) if s == "MCU shutdown") {
            self.check_mcu_shutdown(msg, details)?;
        } else {
            let newmsg = format!("{}{}", msg.str(), MESSAGE_SHUTDOWN);
            self.shared.printer.update_error_msg(&msg.str(), &newmsg);
        }
        let stats = self.shared.printer.get_reactor().get_gc_stats();
        let formatted = format!(
            "Reactor garbage collection: ({}, {}, {})",
            text::float_repr(stats[0]),
            text::float_repr(stats[1]),
            text::float_repr(stats[2])
        );
        log::info(&formatted);
        Ok(())
    }

    fn check_protocol_error(&self, msg: &PyValue, details: &PyValue) -> KResult<()> {
        let start_args = self.shared.printer.get_start_args();
        let host_version_v = dict_get(&start_args, "software_version")?;
        let host_version = host_version_v.clone();
        let host_version_str = host_version_v.str();
        let mut msg_update: Vec<String> = Vec::new();
        let mut msg_updated: Vec<String> = Vec::new();
        let objects = self.shared.printer.lookup_objects(Some("mcu"));
        for (mcu_name, mcu) in objects {
            let version_info = match mcu.get_status(0.0) {
                None => {
                    log::exception(
                        "Unable to retrieve mcu_version from mcu",
                        "AttributeError: object has no attribute 'get_status'",
                    );
                    continue;
                }
                Some(v) => v,
            };
            let mcu_version_opt = match &version_info {
                PyValue::Dict(d) => d.get(&PyValue::Str("mcu_version".to_string())).cloned(),
                _ => None,
            };
            let version_info_is_dict = matches!(&version_info, PyValue::Dict(_));
            let mcu_version = match mcu_version_opt {
                Some(v) => v,
                None => {
                    let exc = if version_info_is_dict {
                        format!("KeyError: {}", pycompat::value::repr_key("mcu_version"))
                    } else {
                        format!("TypeError: '{}' object is not subscriptable", type_name(&version_info))
                    };
                    log::exception("Unable to retrieve mcu_version from mcu", &exc);
                    continue;
                }
            };
            let sep = mcu_name.split_whitespace().last().ok_or_else(err_index)?;
            let line = format!("{}: Current version {}", sep, mcu_version.str());
            if py_equal(&mcu_version, &host_version) {
                msg_updated.push(line);
            } else {
                msg_update.push(line);
            }
        }
        if msg_update.is_empty() {
            msg_update.push("<none>".to_string());
        }
        if msg_updated.is_empty() {
            msg_updated.push("<none>".to_string());
        }
        let error_v = dict_get(details, "error")?;
        let error_str = match &error_v {
            PyValue::Str(s) => s.clone(),
            other => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!("sequence item 0: expected str instance, {} found", type_name(other)),
                });
            }
        };
        let mut parts: Vec<String> = vec![
            "MCU Protocol error".to_string(),
            MESSAGE_PROTOCOL_ERROR1.to_string(),
            format!("Your Klipper version is: {}", host_version_str),
            "MCU(s) which should be updated:".to_string(),
        ];
        parts.extend(msg_update);
        parts.push("Up-to-date MCU(s):".to_string());
        parts.extend(msg_updated);
        parts.push(MESSAGE_PROTOCOL_ERROR2.to_string());
        parts.push(error_str);
        let newmsg = parts.join("\n");
        self.shared.printer.update_error_msg(&msg.str(), &newmsg);
        Ok(())
    }

    fn check_mcu_connect_error(&self, msg: &PyValue, details: &PyValue) -> KResult<()> {
        let error_v = dict_get(details, "error")?;
        let newmsg = format!("{}{}", error_v.str(), MESSAGE_MCU_CONNECT_ERROR);
        self.shared.printer.update_error_msg(&msg.str(), &newmsg);
        Ok(())
    }

    /// `_handle_notify_mcu_error` (`error_mcu.py:129`); exposed for tests.
    pub fn handle_notify_mcu_error(&self, msg: &PyValue, details: &PyValue) -> KResult<()> {
        if matches!(msg, PyValue::Str(s) if s == "Protocol error") {
            self.check_protocol_error(msg, details)?;
        } else if matches!(msg, PyValue::Str(s) if s == "MCU error during connect") {
            self.check_mcu_connect_error(msg, details)?;
        }
        Ok(())
    }
}

impl PrinterMCUErrorApi for PrinterMCUError {
    fn add_clarify(&self, msg: &str, callback: ClarifyCallback) {
        PrinterMCUError::add_clarify(self, msg, callback);
    }
}

impl PrinterMcuErrorApi for PrinterMCUError {}

fn load_config(config: &Config) -> KResult<Option<klippy_api::object::ObjectRef>> {
    Ok(Some(Arc::new(PrinterMCUError::new(config))))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        ..Default::default()
    }
}
