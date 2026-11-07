//! Arbitrary-precision integer reproducing CPython 3.8 `int` semantics.
//! CPython ints are arbitrary precision, floor division rounds toward
//! negative infinity (`-7 // 2 == -4`), `%` takes the divisor sign
//! (`-7 % 2 == 1`), and `repr`/`str` are plain decimal.
//! Wraps `num_bigint::BigInt` with Python floor-division semantics.

use num_bigint::BigInt;
use num_traits::Signed;
use std::cmp::Ordering;

/// Arbitrary-precision integer with CPython 3.8 semantics.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct PyInt(pub BigInt);

impl PyInt {
    /// Wrap a `BigInt` (CPython `int` value, `Objects/longobject.c`).
    pub fn new(b: BigInt) -> Self {
        PyInt(b)
    }

    /// Wrap an `i64` (CPython `int(v)` for small ints).
    pub fn from_i64(v: i64) -> Self {
        PyInt(BigInt::from(v))
    }

    /// Wrap a `u64` (CPython `int(v)` for small ints).
    pub fn from_u64(v: u64) -> Self {
        PyInt(BigInt::from(v))
    }

    /// True when the value is zero (CPython `bool(0)` is `False`).
    pub fn is_zero(&self) -> bool {
        self.0.is_zero()
    }

    /// Borrow the inner `BigInt`.
    pub fn inner(&self) -> &BigInt {
        &self.0
    }

    /// Consume and return the inner `BigInt`.
    pub fn into_bigint(self) -> BigInt {
        self.0
    }

    /// CPython `int.__hash__`: `hash(n)` for `n == -1` is `-2`
    /// (CPython 3.8 `Objects/longobject.c:long_hash`).
    /// For other values hash is `n` reduced modulo `2**61 - 1`
    /// with sign handling. Implemented for dict-key collision
    /// compatibility (`1 == 1.0 == True` share the hash).
    pub fn py_hash(&self) -> i64 {
        // CPython: _PyHASH_MODULUS = 2**61 - 1.
        const MOD: u64 = (1u64 << 61) - 1;
        let n = &self.0;
        if n.is_negative() {
            if n == &BigInt::from(-1i64) {
                return -2;
            }
            // hash(n) = -( (-n) mod MOD ), with mod in [0, MOD-1],
            // and hash == -1 mapped to -2.
            let neg = BigInt::from(MOD) - n; // -n
            let m = (neg % BigInt::from(MOD)).to_u64().unwrap_or(0);
            let h = -(m as i64);
            if h == -1 { -2 } else { h }
        } else {
            let m = (n % BigInt::from(MOD)).to_u64().unwrap_or(0);
            let h = m as i64;
            if h == -1 { -2 } else { h }
        }
    }
}

impl From<i64> for PyInt {
    fn from(v: i64) -> Self {
        PyInt::from_i64(v)
    }
}

impl From<u64> for PyInt {
    fn from(v: u64) -> Self {
        PyInt::from_u64(v)
    }
}

impl From<BigInt> for PyInt {
    fn from(v: BigInt) -> Self {
        PyInt(v)
    }
}

impl From<&BigInt> for PyInt {
    fn from(v: &BigInt) -> Self {
        PyInt(v.clone())
    }
}

use num_traits::ToPrimitive;
impl PyInt {
    /// Exact sign: -1, 0, +1.
    pub fn sign(&self) -> i32 {
        match self.0.sign() {
            num_bigint::Sign::Minus => -1,
            num_bigint::Sign::NoSign => 0,
            num_bigint::Sign::Plus => 1,
        }
    }

    /// Magnitude bits for exact mixed int/float comparison helpers.
    pub fn to_f64_exact(&self) -> Option<f64> {
        self.0.to_f64()
    }
}

/// Compare two integers numerically (CPython `int.__lt__` exact).
pub fn int_cmp(a: &PyInt, b: &PyInt) -> Ordering {
    a.0.cmp(&b.0)
}

use num_traits::Zero;
use num_traits::One;

/// Floor division `a // b` for integers, rounding toward negative
/// infinity (CPython `Objects/longobject.c:long_floor_div`).
/// Panics on zero; callers check first and emit the CPython text
/// `integer division or modulo by zero`.
pub fn int_floor_div(a: &PyInt, b: &PyInt) -> PyInt {
    debug_assert!(!b.0.is_zero());
    // num-bigint `/` truncates toward zero; adjust to floor.
    let q = &a.0 / &b.0;
    let r = &a.0 % &b.0; // truncating remainder, sign follows dividend
    let signs_differ = (a.0.sign() != b.0.sign()) && !r.is_zero();
    if signs_differ {
        PyInt(q - BigInt::one())
    } else {
        PyInt(q)
    }
}

/// Modulo `a % b` for integers; result has the divisor sign
/// (CPython `Objects/longobject.c:long_mod`).
/// Panics on zero; callers check zero and use `integer division
/// or modulo by zero`.
pub fn int_mod(a: &PyInt, b: &PyInt) -> PyInt {
    debug_assert!(!b.0.is_zero());
    let r = &a.0 % &b.0;
    if r.is_zero() {
        return PyInt(r);
    }
    let signs_differ = a.0.sign() != b.0.sign();
    if signs_differ {
        PyInt(r + b.0.clone())
    } else {
        PyInt(r)
    }
}

/// `divmod(a, b)` for integers (CPython `divmod`).
pub fn int_divmod(a: &PyInt, b: &PyInt) -> (PyInt, PyInt) {
    (int_floor_div(a, b), int_mod(a, b))
}

/// Exact integer power `base ** exp` for `exp >= 0`
/// (CPython `Objects/longobject.c:long_pow`).
pub fn int_pow(base: &PyInt, exp: u64) -> PyInt {
    PyInt(base.0.clone().pow(u32::try_from(exp).expect("int_pow exponent fits u32")))
}

/// `True if a is odd` used for half-even rounding parity.
pub fn int_is_odd(a: &PyInt) -> bool {
    // Cheap: lowest bit via mod 2.
    let r = &a.0 % &BigInt::from(2i32);
    !r.is_zero()
}
