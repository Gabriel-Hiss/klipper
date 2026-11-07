//! Printer objects, dynamic lookup and event arguments.
//!
//! Every object registered with the printer (`printer.add_object`, `load_object`) is an
//! `Arc<dyn PrinterObject>`. Python code reaches other objects by name and calls their methods
//! directly; Rust code does the same by downcasting to the provider's concrete type
//! ([`downcast`]) when it depends on that provider's crate, or by calling a trait from this
//! crate when the provider is a core service.

use pycompat::PyValue;
use std::any::Any;
use std::sync::Arc;

/// An object registered with the printer.
pub trait PrinterObject: Any + Send + Sync {
    /// Upcast for [`downcast`].
    fn as_any(&self) -> &dyn Any;

    /// Arc upcast for [`downcast_arc`].
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn Any + Send + Sync>;

    /// `get_status(eventtime)`; `None` when the Python class has no `get_status`
    /// (the object then does not appear in `objects/list`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        None
    }
}

/// Implements the boilerplate of [`PrinterObject`] for a concrete type; add `get_status`
/// by implementing the trait by hand instead when the class has one.
#[macro_export]
macro_rules! printer_object {
    ($ty:ty) => {
        impl $crate::PrinterObject for $ty {
            fn as_any(&self) -> &dyn ::std::any::Any {
                self
            }
            fn as_any_arc(
                self: ::std::sync::Arc<Self>,
            ) -> ::std::sync::Arc<dyn ::std::any::Any + Send + Sync> {
                self
            }
        }
    };
}

pub type ObjectRef = Arc<dyn PrinterObject>;

/// `isinstance`-style access to a concrete provider type.
pub fn downcast<T: 'static>(obj: &dyn PrinterObject) -> Option<&T> {
    obj.as_any().downcast_ref::<T>()
}

/// Like [`downcast`], keeping shared ownership.
pub fn downcast_arc<T: Any + Send + Sync>(obj: &ObjectRef) -> Option<Arc<T>> {
    obj.clone().as_any_arc().downcast::<T>().ok()
}

/// A value passed to or returned from an event handler. Python events pass plain values,
/// printer objects (a stepper, a homing move) and occasionally lists of objects.
#[derive(Clone)]
pub enum EventArg {
    Value(PyValue),
    Object(ObjectRef),
    /// A non-printer-object Python object (e.g. a `HomingMove`); consumers downcast it.
    Any(Arc<dyn Any + Send + Sync>),
    List(Vec<EventArg>),
}
