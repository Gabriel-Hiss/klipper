//! System libm through `extern "C"`, never Rust intrinsics, so
//! results match glibc 2.29 bit for bit (see ARCHITECTURE.md
//! floating-point rules). `sqrt` may use `f64::sqrt` (correctly
//! rounded) and is still routed through libm here for uniformity.
//!
//! The pycompat crate provides the same declarations (the
//! integration owner dedupes).

// SAFETY: glibc libm; pure functions.
unsafe extern "C" {
    fn raw_exp(x: f64) -> f64;
    fn raw_exp2(x: f64) -> f64;
    fn raw_log(x: f64) -> f64;
    fn raw_log10(x: f64) -> f64;
    fn raw_log2(x: f64) -> f64;
    fn raw_pow(x: f64, y: f64) -> f64;
    fn raw_sin(x: f64) -> f64;
    fn raw_cos(x: f64) -> f64;
    fn raw_tan(x: f64) -> f64;
    fn raw_asin(x: f64) -> f64;
    fn raw_acos(x: f64) -> f64;
    fn raw_atan(x: f64) -> f64;
    fn raw_atan2(y: f64, x: f64) -> f64;
    fn raw_sinh(x: f64) -> f64;
    fn raw_cosh(x: f64) -> f64;
    fn raw_tanh(x: f64) -> f64;
    fn raw_floor(x: f64) -> f64;
    fn raw_ceil(x: f64) -> f64;
    fn raw_trunc(x: f64) -> f64;
    fn raw_fabs(x: f64) -> f64;
    fn raw_fmod(x: f64, y: f64) -> f64;
    fn raw_remainder(x: f64, y: f64) -> f64;
    fn raw_hypot(x: f64, y: f64) -> f64;
    fn raw_sqrt(x: f64) -> f64;
    fn raw_expm1(x: f64) -> f64;
    fn raw_log1p(x: f64) -> f64;
}

/// `exp(x)` in glibc.
pub fn exp(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_exp(x) }
}
/// `exp2(x)` in glibc.
pub fn exp2(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_exp2(x) }
}
/// Natural log in glibc (named `log` to mirror C / Python `math.log`).
pub fn log(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_log(x) }
}
/// Base-10 log in glibc.
pub fn log10(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_log10(x) }
}
/// Base-2 log in glibc.
pub fn log2(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_log2(x) }
}
/// `pow(x, y)` in glibc.
pub fn pow(x: f64, y: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_pow(x, y) }
}
/// `sin(x)` in glibc.
pub fn sin(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_sin(x) }
}
/// `cos(x)` in glibc.
pub fn cos(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_cos(x) }
}
/// `tan(x)` in glibc.
pub fn tan(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_tan(x) }
}
/// `asin(x)` in glibc.
pub fn asin(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_asin(x) }
}
/// `acos(x)` in glibc.
pub fn acos(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_acos(x) }
}
/// `atan(x)` in glibc.
pub fn atan(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_atan(x) }
}
/// `atan2(y, x)` in glibc.
pub fn atan2(y: f64, x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_atan2(y, x) }
}
/// `sinh(x)` in glibc.
pub fn sinh(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_sinh(x) }
}
/// `cosh(x)` in glibc.
pub fn cosh(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_cosh(x) }
}
/// `tanh(x)` in glibc.
pub fn tanh(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_tanh(x) }
}
/// `floor(x)` in glibc.
pub fn floor(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_floor(x) }
}
/// `ceil(x)` in glibc.
pub fn ceil(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_ceil(x) }
}
/// `trunc(x)` in glibc.
pub fn trunc(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_trunc(x) }
}
/// `fabs(x)` in glibc.
pub fn fabs(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_fabs(x) }
}
/// `fmod(x, y)` in glibc.
pub fn fmod(x: f64, y: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_fmod(x, y) }
}
/// `remainder(x, y)` in glibc.
pub fn remainder(x: f64, y: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_remainder(x, y) }
}
/// `hypot(x, y)` in glibc.
pub fn hypot(x: f64, y: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_hypot(x, y) }
}
/// `sqrt(x)` in glibc (correctly rounded).
pub fn sqrt(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_sqrt(x) }
}
/// `expm1(x)` in glibc.
pub fn expm1(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_expm1(x) }
}
/// `log1p(x)` in glibc.
pub fn log1p(x: f64) -> f64 {
    // SAFETY: pure libm call.
    unsafe { raw_log1p(x) }
}

/// True when `x` is NaN (bit test, no libm needed).
pub fn is_nan(x: f64) -> bool {
    x.to_bits() & 0x7ff0_0000_0000_0000 != 0 && x.to_bits() & 0x000f_ffff_ffff_ffff != 0
}

/// True when `x` is infinite.
pub fn is_infinite(x: f64) -> bool {
    x.to_bits() & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_values() {
        assert_eq!(exp(0.), 1.);
        assert_eq!(sin(0.), 0.);
        assert_eq!(floor(1.5), 1.);
        assert_eq!(ceil(1.5), 2.);
        assert_eq!(sqrt(4.), 2.);
        assert!((log(2.7182818284590455) - 1.).abs() < 1e-15);
    }
}
