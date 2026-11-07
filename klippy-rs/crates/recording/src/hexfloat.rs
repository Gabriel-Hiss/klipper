//! Hex-float conversion in CPython spelling.
//!
//! CPython `float.hex()` spells a finite `f64` as `[-]0x<d>[.<13 hex
//! digits>]p<±E>`, with `-0x0.0p+0` for zero and `inf`/`nan` for the
//! specials. The mantissa is the exact binary fraction, so both
//! directions here are exact: no rounding is ever needed.

/// Spell `x` exactly like CPython `float.hex()`.
pub fn f64_to_hex(x: f64) -> String {
    if x.is_nan() {
        return "nan".to_string();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf".to_string() } else { "-inf".to_string() };
    }
    let bits = x.to_bits();
    let sign = if bits >> 63 != 0 { "-" } else { "" };
    let exp = ((bits >> 52) & 0x7ff) as i32;
    let mant = bits & ((1u64 << 52) - 1);
    if exp == 0 && mant == 0 {
        return format!("{sign}0x0.0p+0");
    }
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let hex_frac = |m: u64, digits: usize| -> String {
        let mut out = String::with_capacity(digits);
        for i in 0..digits {
            let shift = 4 * (digits - i - 1) as u64;
            out.push(DIGITS[((m >> shift) & 0xf) as usize] as char);
        }
        out
    };
    if exp == 0 {
        // Subnormal: leading digit 0, exponent -1022.
        let frac = hex_frac(mant, 13);
        let frac = frac.trim_end_matches('0');
        let frac = if frac.is_empty() { "0" } else { frac };
        return format!("{sign}0x0.{frac}p-1022");
    }
    let e = exp - 1023;
    if mant == 0 {
        return format!("{sign}0x1.0p{e:+}");
    }
    let frac = hex_frac(mant, 13);
    let frac = frac.trim_end_matches('0');
    format!("{sign}0x1.{frac}p{e:+}")
}

/// Parse a CPython hex-float spelling back into the exact `f64`.
/// Returns `None` on malformed input.
pub fn f64_from_hex(s: &str) -> Option<f64> {
    let (neg, rest) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let rest = rest.strip_prefix("0x").or_else(|| rest.strip_prefix("0X"))?;
    let (mantissa, exp) = rest.split_once(['p', 'P'])?;
    let exp: i32 = exp.parse().ok()?;
    let (int_part, frac_part) = match mantissa.split_once('.') {
        Some((i, f)) => (i, f),
        None => (mantissa, ""),
    };
    if int_part.is_empty() || !int_part.bytes().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    if !frac_part.bytes().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    // Accumulate the mantissa as an integer (at most 14 hex digits for
    // f64, so it fits a u64 exactly), then scale by a power of two.
    // The u64-to-f64 conversion is exact because the mantissa has at
    // most 53 significant bits (trailing zeros beyond that).
    let mut m: u64 = 0;
    for c in int_part.bytes().chain(frac_part.bytes()) {
        m = m.checked_mul(16)?.checked_add(hex_val(c)? as u64)?;
    }
    let eff = exp - 4 * frac_part.len() as i32;
    // Exact: m is an exact float and 2^eff is an exact power of two,
    // and the product is representable, so no rounding can occur.
    let v = (m as f64) * f64::exp2(eff as f64);
    Some(if neg { -v } else { v })
}

fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}
