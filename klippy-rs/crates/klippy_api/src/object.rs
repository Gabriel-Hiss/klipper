//! Printer objects, dynamic lookup and event arguments.
//!
//! Every object registered with the printer (`printer.add_object`, `load_object`) is an
//! `Arc<dyn PrinterObject>`. Python code reaches other objects by name and calls their
//! methods directly (duck typing). Rust code asks the object for an interface instead:
//!
//! ```ignore
//! let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
//!     .expect("Raise never yields None");
//! ```
//!
//! An object declares the interfaces it provides with [`printer_object!`] (or
//! [`printer_interfaces!`] inside a hand-written `impl PrinterObject`). Interfaces may be
//! traits of this crate (`dyn PinsApi`, `dyn GCodeApi`) or traits owned by the providing
//! module (`dyn extras::heaters::HeaterApi`); no central list exists. [`downcast`] to the
//! concrete type remains available when the caller already depends on the provider.

use pycompat::PyValue;
use std::any::{Any, TypeId};
use std::sync::Arc;

use crate::config::Config;
use crate::error::{KResult, KlippyError};
use crate::printer::{LookupDefault, PrinterApi};

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

    /// `stats(eventtime)`; `None` when the Python class has no `stats` (the
    /// `hasattr(o, 'stats')` filter of `extras/statistics.py:55`).
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }

    /// Interface query behind [`interface`]: when `want` is `TypeId::of::<Arc<dyn T>>()`
    /// for a trait `T` this object implements, return that `Arc<dyn T>` boxed. Generated
    /// by [`printer_interfaces!`]; the default provides no interface.
    fn query_interface(self: Arc<Self>, _want: TypeId) -> Option<Box<dyn Any + Send + Sync>> {
        None
    }
}

/// Generates `PrinterObject::query_interface` for the listed trait-object types. Use it
/// inside a hand-written `impl PrinterObject` (one that defines `get_status`):
///
/// ```ignore
/// impl PrinterObject for PrinterPins {
///     klippy_api::printer_object_base!();
///     klippy_api::printer_interfaces!(dyn PinsApi);
///     fn get_status(&self, _eventtime: f64) -> Option<PyValue> { ... }
/// }
/// ```
#[macro_export]
macro_rules! printer_interfaces {
    ($($iface:ty),* $(,)?) => {
        fn query_interface(
            self: ::std::sync::Arc<Self>,
            want: ::std::any::TypeId,
        ) -> ::std::option::Option<::std::boxed::Box<dyn ::std::any::Any + Send + Sync>> {
            $(
                if want == ::std::any::TypeId::of::<::std::sync::Arc<$iface>>() {
                    let handle: ::std::sync::Arc<$iface> = self;
                    return ::std::option::Option::Some(::std::boxed::Box::new(handle));
                }
            )*
            let _ = want;
            ::std::option::Option::None
        }
    };
}

/// The `as_any` / `as_any_arc` boilerplate of [`PrinterObject`], for hand-written impls.
#[macro_export]
macro_rules! printer_object_base {
    () => {
        fn as_any(&self) -> &dyn ::std::any::Any {
            self
        }
        fn as_any_arc(
            self: ::std::sync::Arc<Self>,
        ) -> ::std::sync::Arc<dyn ::std::any::Any + Send + Sync> {
            self
        }
    };
}

/// Implements [`PrinterObject`] for a concrete type: `printer_object!(Type)` for a plain
/// object, `printer_object!(Type; dyn PinsApi, dyn ChipApi)` to also declare the
/// interfaces it provides. Write the impl by hand (with [`printer_object_base!`] and
/// [`printer_interfaces!`]) when the class has a `get_status`.
#[macro_export]
macro_rules! printer_object {
    ($ty:ty) => {
        impl $crate::PrinterObject for $ty {
            $crate::printer_object_base!();
        }
    };
    ($ty:ty; $($iface:ty),+ $(,)?) => {
        impl $crate::PrinterObject for $ty {
            $crate::printer_object_base!();
            $crate::printer_interfaces!($($iface),+);
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

/// The `T` interface of `obj` (`T` is a trait-object type such as `dyn PinsApi`), or `None`
/// when the object does not declare it.
pub fn interface<T>(obj: &ObjectRef) -> Option<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    let boxed = obj.clone().query_interface(TypeId::of::<Arc<T>>())?;
    boxed.downcast::<Arc<T>>().ok().map(|handle| *handle)
}

/// Python's `AttributeError` when a looked-up object lacks the interface the caller
/// duck-types against. The message names the printer object and the Rust interface.
pub fn missing_interface<T: ?Sized>(name: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!(
            "printer object '{}' does not provide {}",
            name,
            std::any::type_name::<T>()
        ),
    }
}

/// [`interface`] with the [`missing_interface`] error.
pub fn require_interface<T>(obj: &ObjectRef, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    interface::<T>(obj).ok_or_else(|| missing_interface::<T>(name))
}

/// `printer.lookup_object(name, default)` followed by the `T` interface. A missing object
/// follows `default` exactly like `lookup_object`; a present object without the interface
/// is an `AttributeError`.
pub fn lookup_interface<T>(
    printer: &dyn PrinterApi,
    name: &str,
    default: LookupDefault,
) -> KResult<Option<Arc<T>>>
where
    T: ?Sized + Send + Sync + 'static,
{
    match printer.lookup_object(name, default)? {
        Some(obj) => require_interface::<T>(&obj, name).map(Some),
        None => Ok(None),
    }
}

/// `printer.load_object(config, section, default)` followed by the `T` interface.
pub fn load_interface<T>(
    printer: &dyn PrinterApi,
    config: &Config,
    section: &str,
    default: LookupDefault,
) -> KResult<Option<Arc<T>>>
where
    T: ?Sized + Send + Sync + 'static,
{
    match printer.load_object(config, section, default)? {
        Some(obj) => require_interface::<T>(&obj, section).map(Some),
        None => Ok(None),
    }
}

/// A value passed to or returned from an event handler. Python events pass plain values,
/// printer objects (a stepper, a homing move) and occasionally lists of objects.
#[derive(Clone)]
pub enum EventArg {
    Value(PyValue),
    Object(ObjectRef),
    /// A non-printer-object Python object (e.g. a `HomingMove`). Build it with
    /// [`EventArg::handle`] and read it with [`EventArg::as_handle`].
    Any(Arc<dyn Any + Send + Sync>),
    List(Vec<EventArg>),
}

impl EventArg {
    /// Wraps an interface handle (`Arc<dyn HomingStateApi>`, `Arc<dyn RailApi>`) so a
    /// handler can recover it with [`EventArg::as_handle`]. The payload is the `Arc<T>`
    /// itself, boxed once more because `dyn T` is not `Any`.
    pub fn handle<T>(handle: Arc<T>) -> EventArg
    where
        T: ?Sized + Send + Sync + 'static,
    {
        EventArg::Any(Arc::new(handle))
    }

    /// The handle stored by [`EventArg::handle`], or the `T` interface of an
    /// [`EventArg::Object`].
    pub fn as_handle<T>(&self) -> Option<Arc<T>>
    where
        T: ?Sized + Send + Sync + 'static,
    {
        match self {
            EventArg::Any(payload) => payload.downcast_ref::<Arc<T>>().cloned(),
            EventArg::Object(obj) => interface::<T>(obj),
            _ => None,
        }
    }

    /// The handles of an [`EventArg::List`] (Python passes `rails` as a list).
    pub fn as_handle_list<T>(&self) -> Option<Vec<Arc<T>>>
    where
        T: ?Sized + Send + Sync + 'static,
    {
        match self {
            EventArg::List(items) => items.iter().map(EventArg::as_handle::<T>).collect(),
            _ => None,
        }
    }
}
