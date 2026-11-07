//! The frozen `klippy/chelper` C library, linked statically for differential tests.
//!
//! Tests of a ported C unit declare the reference functions they need in their own
//! `extern "C"` block and call both the reference and the port with identical inputs.
//! Depend on this crate only from `[dev-dependencies]`; production code never links it.
//!
//! ```ignore
//! extern "C" {
//!     fn trapq_alloc() -> *mut core::ffi::c_void;
//! }
//! ```

/// Forces the static library to be linked into the test binary.
pub fn link() {}
