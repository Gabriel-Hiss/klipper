//! Interior mutability for printer objects shared between reactor greenlets.
//!
//! Python objects in klippy are mutated freely from any greenlet because only one greenlet runs
//! at a time. The Rust reactor keeps that guarantee (greenlets are OS threads that pass a baton,
//! see the `reactor` crate), so state shared through `Arc` can use a `RefCell`-like cell that is
//! `Sync` without a lock. The rules that keep it sound:
//!
//! 1. Only reactor greenlets (or code they call synchronously) touch a [`GCell`]. Other threads
//!    (the serial reader, step generation) use [`SCell`], `kos` channels or the reactor's
//!    async callbacks. Python's GIL does not protect these direct callbacks in Rust.
//! 2. A borrow never spans a call that can pause the reactor (`pause`, `Completion::wait`,
//!    `ReactorMutex::lock`, MCU queries, `toolhead.wait_moves`, ...) or call back into code that
//!    may borrow the same cell. Copy what you need out, drop the borrow, then call.
//! 3. Borrow conflicts panic with the cell's name, like `RefCell`, so a violation is loud.

use std::cell::{Ref, RefCell, RefMut};

/// A `RefCell` that may be shared between reactor greenlets (see module docs for the rules).
pub struct GCell<T> {
    inner: RefCell<T>,
}

// SAFETY: the reactor runs exactly one greenlet at a time and hands the baton over with
// pthread mutex/condvar operations that order memory, so accesses from different greenlet
// threads never overlap and always observe each other's writes.
unsafe impl<T: Send> Sync for GCell<T> {}
unsafe impl<T: Send> Send for GCell<T> {}

impl<T> GCell<T> {
    pub const fn new(value: T) -> GCell<T> {
        GCell { inner: RefCell::new(value) }
    }

    /// Shared borrow; panics if the value is mutably borrowed.
    #[track_caller]
    pub fn borrow(&self) -> Ref<'_, T> {
        self.inner.borrow()
    }

    /// Exclusive borrow; panics if the value is borrowed.
    #[track_caller]
    pub fn borrow_mut(&self) -> RefMut<'_, T> {
        self.inner.borrow_mut()
    }

    /// Replaces the value and returns the old one.
    #[track_caller]
    pub fn replace(&self, value: T) -> T {
        self.inner.replace(value)
    }

    /// Runs `f` with an exclusive borrow that ends before this returns.
    #[track_caller]
    pub fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        f(&mut self.inner.borrow_mut())
    }
}

impl<T: Copy> GCell<T> {
    #[track_caller]
    pub fn get(&self) -> T {
        *self.inner.borrow()
    }

    #[track_caller]
    pub fn set(&self, value: T) {
        *self.inner.borrow_mut() = value;
    }
}

impl<T: Clone> GCell<T> {
    /// Clone of the current value (the usual way to read without holding a borrow).
    #[track_caller]
    pub fn cloned(&self) -> T {
        self.inner.borrow().clone()
    }
}

impl<T: Default> Default for GCell<T> {
    fn default() -> GCell<T> {
        GCell::new(T::default())
    }
}

/// A mutex-backed cell for state also accessed by serial-reader callbacks.
///
/// Unlike [`GCell`], this cell permits concurrent access from non-reactor
/// threads. Release guards before calling user callbacks, pausing the reactor,
/// or re-entering the same cell.
pub struct SCell<T> {
    inner: kos::sync::Mutex<T>,
}

impl<T> SCell<T> {
    pub fn new(value: T) -> Self {
        Self { inner: kos::sync::Mutex::new(value) }
    }

    pub fn borrow(&self) -> kos::sync::MutexGuard<'_, T> {
        self.inner.lock()
    }

    pub fn borrow_mut(&self) -> kos::sync::MutexGuard<'_, T> {
        self.inner.lock()
    }

    pub fn replace(&self, value: T) -> T {
        std::mem::replace(&mut *self.inner.lock(), value)
    }

    pub fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        f(&mut *self.inner.lock())
    }
}

impl<T: Copy> SCell<T> {
    pub fn get(&self) -> T {
        *self.inner.lock()
    }

    pub fn set(&self, value: T) {
        *self.inner.lock() = value;
    }
}

impl<T: Clone> SCell<T> {
    pub fn cloned(&self) -> T {
        self.inner.lock().clone()
    }
}

impl<T: Default> Default for SCell<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

/// A callback temporarily removed from its slot without holding its mutex.
/// Drop restores it even during unwinding, unless a replacement was installed.
pub struct TakenCallback<'a, T> {
    slot: &'a SCell<Option<T>>,
    value: Option<T>,
}

impl<T> std::ops::Deref for TakenCallback<'_, T> {
    type Target = Option<T>;
    fn deref(&self) -> &Self::Target { &self.value }
}

impl<T> std::ops::DerefMut for TakenCallback<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.value }
}

impl<T> Drop for TakenCallback<'_, T> {
    fn drop(&mut self) {
        let mut slot = self.slot.borrow_mut();
        if slot.is_none() {
            *slot = self.value.take();
        }
    }
}

impl<T> SCell<Option<T>> {
    /// Lease a callback across a potentially panicking or reentrant call.
    pub fn take_callback(&self) -> TakenCallback<'_, T> {
        TakenCallback { slot: self, value: self.borrow_mut().take() }
    }
}
