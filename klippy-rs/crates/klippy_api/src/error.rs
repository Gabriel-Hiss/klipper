//! Errors that cross module boundaries, mirroring the Python exception classes klippy uses.
//!
//! Klippy decides behaviour by exception class (`config_error` aborts startup with the RESTART
//! hint, `command_error` becomes a G-code `!!` error, anything else is an internal error with a
//! traceback) and shows the message text to users, so both the class and the exact `str(exc)`
//! text are part of the contract.
//!
//! `Other`/`Internal` carry the class as Python's traceback prints it: the bare name for
//! builtins (`KeyError`), `module.Name` otherwise (`stepper.error`,
//! `extras.bed_mesh.BedMeshError`). [`KlippyError::class_name`] is `type(exc).__name__`,
//! [`KlippyError::qualified_class_name`] the traceback spelling.

use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum KlippyError {
    /// `configfile.error` / `printer.config_error` (a `configparser.Error`).
    Config(String),
    /// `gcode.CommandError` / `printer.command_error`.
    Command(String),
    /// `mcu.error`, raised by MCU communication failures.
    Mcu(String),
    /// `pins.error`, distinct from other Python classes also named `error`.
    Pins(String),
    /// `msgproto.error`, caught separately during MCU identification.
    Protocol(String),
    /// `homing.EndstopError`-style or other module-specific exception classes that callers catch
    /// by class: `class_name` is the qualified Python class name (module prefix unless a
    /// builtin), `message` its `str(exc)`.
    Other { class_name: String, message: String },
    /// An exception the Python code does not catch by class (would become an internal error);
    /// `class_name` is qualified like [`KlippyError::Other`].
    Internal { class_name: String, message: String },
}

impl KlippyError {
    pub fn config(msg: impl Into<String>) -> KlippyError {
        KlippyError::Config(msg.into())
    }

    pub fn command(msg: impl Into<String>) -> KlippyError {
        KlippyError::Command(msg.into())
    }

    /// The Python class name (`type(exc).__name__`) of this error.
    pub fn class_name(&self) -> &str {
        let qualified = self.qualified_class_name();
        qualified.rsplit('.').next().unwrap_or(qualified)
    }

    /// The class as `traceback.format_exception_only` prints it: `__module__ + '.' +
    /// __qualname__`, bare for builtins.
    pub fn qualified_class_name(&self) -> &str {
        match self {
            KlippyError::Config(_) => "configparser.Error",
            KlippyError::Command(_) => "gcode.CommandError",
            KlippyError::Mcu(_) => "mcu.error",
            KlippyError::Pins(_) => "pins.error",
            KlippyError::Protocol(_) => "msgproto.error",
            KlippyError::Other { class_name, .. } | KlippyError::Internal { class_name, .. } => {
                class_name
            }
        }
    }

    /// The last line of the traceback `logging.exception` prints for this error
    /// (`traceback.format_exception_only`, without the newline).
    pub fn exception_line(&self) -> String {
        if self.message().is_empty() {
            self.qualified_class_name().to_string()
        } else {
            format!("{}: {}", self.qualified_class_name(), self.message())
        }
    }

    /// Raises this error through the panic path, for callbacks whose contract has no error
    /// channel (reactor timers, serial response handlers). [`panic_error`] turns the payload
    /// back into the error where the panic is caught.
    pub fn raise(&self) -> ! {
        panic!("{}: {}", self.qualified_class_name(), self.message())
    }

    /// `str(exc)`.
    pub fn message(&self) -> &str {
        match self {
            KlippyError::Config(m)
            | KlippyError::Command(m)
            | KlippyError::Mcu(m)
            | KlippyError::Pins(m)
            | KlippyError::Protocol(m) => m,
            KlippyError::Other { message, .. } | KlippyError::Internal { message, .. } => message,
        }
    }
}

impl fmt::Display for KlippyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.message())
    }
}

impl std::error::Error for KlippyError {}

pub type KResult<T> = Result<T, KlippyError>;

/// The error a caught panic stands for: a payload written by [`KlippyError::raise`]
/// (`Class: message`) keeps its class; any other panic is an `Exception`.
pub fn panic_error(payload: Box<dyn std::any::Any + Send>) -> KlippyError {
    let text = if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "panic".to_string()
    };
    let (class_name, message) = text.split_once(": ").unwrap_or(("Exception", &text));
    KlippyError::Internal { class_name: class_name.to_string(), message: message.to_string() }
}
