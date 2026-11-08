//! The frozen object-model contract of the port.
//!
//! Python klippy is a graph of objects that find each other by name through the `Printer` and
//! call each other's methods. The port keeps that shape:
//!
//! - Objects are `Arc<dyn PrinterObject>` ([`object`]); their mutable state lives in [`GCell`]s
//!   whose rules (no borrow across a reactor pause) are documented in [`gcell`].
//! - Core services (printer, config, G-code, MCU, toolhead, steppers, webhooks) are traits in
//!   this crate, implemented by the `core` crate, so every module can be compiled and tested
//!   against fakes before the core is assembled.
//! - Errors carry the Python exception class and message ([`error`]).
//!
//! Changing this crate changes the contract of every module: only the integration owner edits it.

pub mod config;
pub mod error;
pub mod gcell;
pub mod gcode;
pub mod log;
pub mod mcu;
pub mod object;
pub mod printer;
pub mod stepper;
pub mod toolhead;
pub mod webhooks;

pub use error::{KResult, KlippyError};
pub use gcell::GCell;
pub use object::{
    downcast, downcast_arc, interface, load_interface, lookup_interface, missing_interface,
    require_interface, EventArg, ObjectRef, PrinterObject,
};
