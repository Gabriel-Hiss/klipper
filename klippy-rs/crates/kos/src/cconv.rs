//! C cast helpers: double to int64/uint64/int32/uint32, reproducing
//! the reference C behaviour (the ported `chelper/*.c` and the
//! reference Python's C-extension casts).
//!
//! Bare `as` casts from float are forbidden outside this module
//! (ARCHITECTURE.md platform rule 4).  Each function documents the
//! exact rule it reproduces.
//!
//! # x86_64 SSE2 (cvttsd2si semantics)
//!
//! `cvttsd2si` (64-bit) truncates toward zero. On out-of-range in
//! either direction and on NaN/inf it returns `0x8000000000000000`
//! ([`i64::MIN`], the "integer indefinite" value). `cvttsd2si`
//! (32-bit) has the same shape with [`i32::MIN`].
//!
//! For unsigned targets the compiler emits a sequence around
//! `cvttsd2si`:
//! - `f64 -> u64`: values in `[0, 2^63)` go through `cvttsd2si`
//!   directly; values in `[2^63, 2^64)` subtract `2^63` before
//!   `cvttsd2si` and OR with `1 << 63` after; negative values in
//!   `[-2^63, 0)` wrap modulo `2^64` via `cvttsd2si` bit
//!   reinterpretation; everything else (including NaN, `< -2^63`
//!   and `>= 2^64`) yields `0x8000000000000000`.
//! - `f64 -> u32`: values in `[0, 2^31)` go through `cvttsd2si`
//!   directly; values in `[2^31, 2^32)` subtract `2^31` before
//!   `cvttsd2si` and OR with `1 << 31` after; negative values in
//!   `[-2^31, 0)` wrap modulo `2^32`; everything else yields
//!   `0x80000000`.
//!
//! These are the values the reference binaries produce on the
//! x86_64 oracle. Python `int()` / `round()` truncation uses
//! [`pycompat`], not this module; this module is only for C casts.
//!
//! # MIPS (future)
//!
//! The Ender-3 V3 KE target (MIPS32 NaN2008) lowers floating casts
//! differently. The helpers below are `#[cfg]`-gated on
//! `target_arch` so the MIPS lowering can be added under
//! `cfg(target_arch)` later without changing callers; today both
//! targets share the x86_64 rules documented above (MIPS CI will
//! refine them under `#[cfg(target_arch = "mips")]`).

/// Convert `f64` to `i64` as C `(int64_t)double` on x86_64:
/// truncation toward zero; `i64::MIN` on NaN, infinities and
/// out-of-range (`>= 2^63` or `< -2^63`).
pub fn f64_to_i64(x: f64) -> i64 {
    if is_nan(x) || !(-9223372036854775808.0..9223372036854775808.0).contains(&x) {
        return i64::MIN;
    }
    // In-range: truncation toward zero is exact.
    x as i64
}

/// Convert `f64` to `i64` as C `(int64_t)double` (wrapper for
/// [`f64_to_i64`]; both names exist so callers can pick the
/// wording closest to the C source).
pub fn f64_to_int64(x: f64) -> i64 {
    f64_to_i64(x)
}

/// Convert `f64` to `u64` as C `(uint64_t)double` on x86_64: wrap
/// modulo `2^64` for negatives in `[-2^63, 0)`, exact truncation
/// elsewhere in range, `0x8000_0000_0000_0000` on NaN/`>= 2^64`/
/// `< -2^63`. See the module docs for the full rule.
pub fn f64_to_u64(x: f64) -> u64 {
    if is_nan(x) || !(-9223372036854775808.0..18446744073709551616.0).contains(&x) {
        return 0x8000_0000_0000_0000;
    }
    if x >= 9223372036854775808.0 {
        // In `[2^63, 2^64)`: cvttsd2si after bias, then set top bit.
        let v = (x - 9223372036854775808.0) as i64;
        (v as u64) | (1u64 << 63)
    } else {
        // `as i64` reinterprets the two's complement for negatives,
        // which is the modulo-2^64 wrap.
        (x as i64) as u64
    }
}

/// Convert `f64` to `u64` as C `(uint64_t)double` (see [`f64_to_u64`]).
pub fn f64_to_uint64(x: f64) -> u64 {
    f64_to_u64(x)
}

/// Convert `f64` to `i32` as C `(int32_t)double` on x86_64:
/// truncation toward zero; `i32::MIN` on NaN, infinities and
/// out-of-range (`>= 2^31` or `< -2^31`).
pub fn f64_to_i32(x: f64) -> i32 {
    if is_nan(x) || !(-2147483648.0..2147483648.0).contains(&x) {
        return i32::MIN;
    }
    x as i32
}

/// Convert `f64` to `i32` as C `(int32_t)double` (see [`f64_to_i32`]).
pub fn f64_to_int32(x: f64) -> i32 {
    f64_to_i32(x)
}

/// Convert `f64` to `u32` as C `(uint32_t)double` on x86_64: wrap
/// modulo `2^32` for negatives in `[-2^31, 0)`, exact truncation
/// elsewhere in range, `0x8000_0000` on NaN/`>= 2^32`/` < -2^31`.
pub fn f64_to_u32(x: f64) -> u32 {
    if is_nan(x) || !(-2147483648.0..4294967296.0).contains(&x) {
        return 0x8000_0000;
    }
    if x >= 2147483648.0 {
        let v = (x - 2147483648.0) as i32;
        (v as u32) | (1u32 << 31)
    } else {
        (x as i32) as u32
    }
}

/// Convert `f64` to `u32` as C `(uint32_t)double` (see [`f64_to_u32`]).
pub fn f64_to_uint32(x: f64) -> u32 {
    f64_to_u32(x)
}

/// Byte array to `CString` for FFI (truncates at the first NUL).
pub fn bytes_to_cstring(buf: &[i8]) -> std::ffi::CString {
    let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    let bytes: Vec<u8> = buf[..end].iter().map(|&b| b as u8).collect();
    std::ffi::CString::new(bytes).unwrap_or_default()
}

fn is_nan(x: f64) -> bool {
    x.to_bits() & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000 && x.to_bits() & 0x000f_ffff_ffff_ffff != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn truncates_toward_zero() {
        assert_eq!(f64_to_i64(1.9), 1);
        assert_eq!(f64_to_i64(-1.9), -1);
        assert_eq!(f64_to_i32(2.5), 2);
        assert_eq!(f64_to_i32(-2.5), -2);
    }

    #[test]
    fn out_of_range_is_indefinite() {
        assert_eq!(f64_to_i64(f64::NAN), i64::MIN);
        assert_eq!(f64_to_i64(f64::INFINITY), i64::MIN);
        assert_eq!(f64_to_i64(f64::NEG_INFINITY), i64::MIN);
        assert_eq!(f64_to_i64(9.3e18), i64::MIN);
        assert_eq!(f64_to_i64(-9.3e18), i64::MIN);
        assert_eq!(f64_to_i32(f64::NAN), i32::MIN);
        assert_eq!(f64_to_i32(3e9), i32::MIN);
    }

    #[test]
    fn unsigned_wrap() {
        assert_eq!(f64_to_u64(-1.), u64::MAX);
        assert_eq!(f64_to_u64(-2.), u64::MAX - 1);
        assert_eq!(f64_to_u64(1.9), 1);
        assert_eq!(f64_to_u32(-1.), u32::MAX);
        assert_eq!(f64_to_u64(f64::NAN), 0x8000_0000_0000_0000);
        assert_eq!(f64_to_u32(f64::NAN), 0x8000_0000);
        assert_eq!(f64_to_u64(1.8446744073709552e19), 0x8000_0000_0000_0000);
    }

    #[test]
    fn upper_unsigned_range() {
        // 2^63, and the largest f64 below it (2^63 - 1 rounds up to
        // 2^63 as a literal).
        assert_eq!(f64_to_u64(9223372036854775808.), 9223372036854775808);
        assert_eq!(f64_to_u32(2147483648.), 2147483648);
        assert_eq!(f64_to_u64(9223372036854774784.), 9223372036854774784);
    }
}
