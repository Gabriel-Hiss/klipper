//! Errors that cross module boundaries, mirroring the Python exception classes klippy uses.
//!
//! Klippy decides behaviour by exception class (`config_error` aborts startup with the RESTART
//! hint, `command_error` becomes a G-code `!!` error, anything else is an internal error with a
//! traceback) and shows the message text to users, so both the class and the exact `str(exc)`
//! text are part of the contract.

use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum KlippyError {
    /// `configfile.error` / `printer.config_error` (a `configparser.Error`).
    Config(String),
    /// `gcode.CommandError` / `printer.command_error`.
    Command(String),
    /// `mcu.error` (`serialhdl.error`), raised by MCU communication failures.
    Mcu(String),
    /// `homing.EndstopError`-style or other module-specific exception classes that callers catch
    /// by class: `class_name` is the Python class name, `message` its `str(exc)`.
    Other { class_name: String, message: String },
    /// An exception the Python code does not catch by class (would become an internal error).
    Internal { class_name: String, message: String },
}

impl KlippyError {
    pub fn config(msg: impl Into<String>) -> KlippyError {
        KlippyError::Config(msg.into())
    }

    pub fn command(msg: impl Into<String>) -> KlippyError {
        KlippyError::Command(msg.into())
    }

    /// The Python class name (`str(type(exc).__name__)`) of this error.
    pub fn class_name(&self) -> &str {
        match self {
            KlippyError::Config(_) => "Error",
            KlippyError::Command(_) => "CommandError",
            KlippyError::Mcu(_) => "error",
            KlippyError::Other { class_name, .. } | KlippyError::Internal { class_name, .. } => {
                class_name
            }
        }
    }

    /// `str(exc)`.
    pub fn message(&self) -> &str {
        match self {
            KlippyError::Config(m) | KlippyError::Command(m) | KlippyError::Mcu(m) => m,
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
