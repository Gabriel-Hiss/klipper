//! System libm via `extern "C"` so results match glibc 2.29 bit for bit.
//! Reproduces C `math.h` on the reference target (glibc 2.29, MIPS/x86_64).
//! All wrappers return the glibc result unchanged, including NaN payload
//! handling and errno side effects being ignored (CPython also ignores
//! errno and returns the C result).

#[allow(non_snake_case)]
mod ffi {
    unsafe extern "C" {
        pub fn exp(x: f64) -> f64;
        pub fn log(x: f64) -> f64;
        pub fn log10(x: f64) -> f64;
        pub fn pow(x: f64, y: f64) -> f64;
        pub fn sin(x: f64) -> f64;
        pub fn cos(x: f64) -> f64;
        pub fn sincos(x: f64, sin: *mut f64, cos: *mut f64);
        pub fn tan(x: f64) -> f64;
        pub fn atan(x: f64) -> f64;
        pub fn atan2(y: f64, x: f64) -> f64;
        pub fn asin(x: f64) -> f64;
        pub fn acos(x: f64) -> f64;
        pub fn fabs(x: f64) -> f64;
        pub fn floor(x: f64) -> f64;
        pub fn ceil(x: f64) -> f64;
        pub fn fmod(x: f64, y: f64) -> f64;
        pub fn hypot(x: f64, y: f64) -> f64;
        pub fn sqrt(x: f64) -> f64;
        #[cfg(target_os = "linux")]
        pub fn __errno_location() -> *mut i32;
    }
}

// SAFETY: glibc libm functions are pure in (x) -> result, matching C semantics.

/// Reproduces C `exp` (glibc 2.29); CPython `math.exp` (`Modules/mathmodule.c`) calls it.
pub fn exp(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::exp(x) })
}

/// Reproduces C `log` (glibc 2.29); CPython `math.log` calls it.
pub fn log(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::log(x) })
}

/// Reproduces C `log10` (glibc 2.29); CPython `math.log10` calls it.
pub fn log10(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::log10(x) })
}

/// Reproduces C `pow` (glibc 2.29). Used by Python `**` on floats.
/// CPython 3.8 `float.__pow__` calls C `pow` and returns its result
/// (with domain checks that produce `OverflowError` for `pow(10, 1000)`
/// as `OverflowError: (10.0 ** 1000.0)` style messages handled by caller).
pub fn pow(x: f64, y: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::pow(x, y) })
}

/// C `pow` plus the `errno` it leaves (`0` when unset). CPython's
/// `float_pow` zeroes `errno`, calls `pow` and turns a surviving `ERANGE`
/// into `OverflowError` (`Objects/floatobject.c`, `_Py_ADJUST_ERANGE1`), so
/// the flag is part of the observable result.
pub fn pow_errno(x: f64, y: f64) -> (f64, i32) {
    #[cfg(target_os = "linux")]
    {
        // SAFETY: `__errno_location` returns the calling thread's errno slot.
        unsafe {
            let errno = ffi::__errno_location();
            *errno = 0;
            let r = std::hint::black_box(ffi::pow(x, y));
            (r, *errno)
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        (pow(x, y), 0)
    }
}

/// Reproduces C `sin` (glibc 2.29); CPython `math.sin` calls it.
pub fn sin(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::sin(x) })
}

/// Reproduces C `cos` (glibc 2.29); CPython `math.cos` calls it.
pub fn cos(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::cos(x) })
}

/// Reproduces glibc `sincos` (a GNU extension): `(sin(x), cos(x))`. GCC
/// folds a `sin`/`cos` pair with the same argument into this one call at
/// `-O1` and above, so C code built by GCC that evaluates both (NumPy's
/// `fftpack.c` twiddles) gets these results, which can differ from separate
/// `sin` and `cos` calls in the last bit.
pub fn sincos(x: f64) -> (f64, f64) {
    let mut s = 0.0;
    let mut c = 0.0;
    // SAFETY: both pointers are valid, writable and distinct for the call.
    unsafe { ffi::sincos(x, &mut s, &mut c) };
    std::hint::black_box((s, c))
}

/// Reproduces C `tan` (glibc 2.29); CPython `math.tan` calls it.
pub fn tan(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::tan(x) })
}

/// Reproduces C `atan` (glibc 2.29); CPython `math.atan` calls it.
pub fn atan(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::atan(x) })
}

/// Reproduces C `atan2` (glibc 2.29); CPython `math.atan2` calls it.
pub fn atan2(y: f64, x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::atan2(y, x) })
}

/// Reproduces C `asin` (glibc 2.29); CPython `math.asin` calls it.
pub fn asin(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::asin(x) })
}

/// Reproduces C `acos` (glibc 2.29); CPython `math.acos` calls it.
pub fn acos(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::acos(x) })
}

/// Reproduces C `sqrt` (glibc 2.29); CPython `math.sqrt` calls it.
pub fn sqrt(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::sqrt(x) })
}

/// Reproduces C `fabs` (glibc 2.29); CPython `math.fabs` calls it.
pub fn fabs(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::fabs(x) })
}

/// Reproduces C `floor` (glibc 2.29); CPython `math.floor` calls it.
pub fn floor(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::floor(x) })
}

/// Reproduces C `ceil` (glibc 2.29); CPython `math.ceil` calls it.
pub fn ceil(x: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::ceil(x) })
}

/// Reproduces C `fmod` (glibc 2.29); CPython `math.fmod` calls it.
pub fn fmod(x: f64, y: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::fmod(x, y) })
}

/// Reproduces C `hypot` (glibc 2.29); CPython `math.hypot` calls it.
pub fn hypot(x: f64, y: f64) -> f64 {
    std::hint::black_box(unsafe { ffi::hypot(x, y) })
}
