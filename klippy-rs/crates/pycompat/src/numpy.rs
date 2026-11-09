//! NumPy 1.16.4 float64 kernels, bit for bit.
//!
//! The oracle venv (`/opt/klippy-env`) ships NumPy 1.16.4 built without SIMD
//! math dispatch, so element-wise `exp`/`sin`/`cos`/`sqrt` are the C library
//! calls of [`crate::libm`] and plain IEEE operations. What this module
//! reproduces is the operation order NumPy uses where it differs from a
//! naive loop:
//!
//! - [`pairwise_sum`]: `pairwise_sum_DOUBLE` (`numpy/core/src/umath/loops.c.src`):
//!   below 8 elements a running sum from `0.`, up to 128 elements eight
//!   interleaved accumulators combined as `((r0+r1)+(r2+r3))+((r4+r5)+(r6+r7))`
//!   plus the tail, above that a split at `n/2` rounded down to a multiple of 8.
//! - [`add_reduce`]: `np.add.reduce` over one axis whose inner loop runs along
//!   that axis (`ndarray.sum()`, `np.mean(x, axis=0)` of a column view): the
//!   reduction iterator hands the loop chunks of at most [`BUFSIZE`] elements,
//!   each summed pairwise and added to the result that starts at the identity
//!   `0.`. When the reduced axis is not the innermost one (the `axis=-1` mean
//!   of the transposed `rfft` result in `shaper_calibrate._psd`) NumPy adds the
//!   rows one after the other; that is [`sequential_sum`].
//! - [`arange`]: `PyArray_ArangeObj` (`ctors.c`): length `ceil((stop-start)/step)`,
//!   then `start`, `start+step` and `start + i*delta` with
//!   `delta = (start+step) - start` (`DOUBLE_fill` in `arraytypes.c.src`).
//! - [`i0`]/[`kaiser`]: `numpy/lib/function_base.py` (Cephes Chebyshev
//!   series evaluated with `_chbevl`).
//! - [`interp`]: `arr_interp` (`compiled_base.c`) including
//!   `binary_search_with_guess` and the precomputed slopes.
//! - [`RfftPlan`]/[`FftPlan`]: `numpy.fft` in 1.16 is `fftpack_lite`
//!   (`numpy/fft/fftpack.c`, a C translation of FFTPACK; pocketfft arrived in
//!   1.17). The plans hold the `rffti`/`cffti` twiddles and factors; the
//!   transforms are the forward passes `radf2/3/4/5/radfg` and
//!   `passf2/3/4/5/passf` in the same index and operation order. The
//!   twiddles use glibc `sincos`: GCC folds each `sin`/`cos` pair of
//!   `sincos2pi` into one `sincos` call, which differs from separate `sin`
//!   and `cos` in the last bit for some arguments (e.g. `n = 1001`).
//! - [`rfftfreq`], [`maximum`], [`max_reduce`]/[`min_reduce`] and
//!   the `Complex64` product (`CDOUBLE_multiply`).

use num_traits::ToPrimitive;

use crate::libm;
use crate::text::{trunc_float_to_bigint, PyError};

/// NumPy's default ufunc buffer size (`NPY_BUFSIZE`): reductions over more
/// elements are summed in chunks of this size.
pub const BUFSIZE: usize = 8192;

/// `PW_BLOCKSIZE` of `pairwise_sum_DOUBLE`.
const PW_BLOCKSIZE: usize = 128;

// ---------------------------------------------------------------------------
// Reductions.
// ---------------------------------------------------------------------------

/// `pairwise_sum_DOUBLE(a, n, stride)` over the elements of `a`.
pub fn pairwise_sum(a: &[f64]) -> f64 {
    let n = a.len();
    if n < 8 {
        let mut res = 0.;
        for &v in a {
            res += v;
        }
        res
    } else if n <= PW_BLOCKSIZE {
        let mut r = [a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]];
        let mut i = 8;
        while i < n - (n % 8) {
            for (j, acc) in r.iter_mut().enumerate() {
                *acc += a[i + j];
            }
            i += 8;
        }
        let mut res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        while i < n {
            res += a[i];
            i += 1;
        }
        res
    } else {
        let mut n2 = n / 2;
        n2 -= n2 % 8;
        pairwise_sum(&a[..n2]) + pairwise_sum(&a[n2..])
    }
}

/// `np.add.reduce` along an axis that is the inner loop of the reduction:
/// `ndarray.sum()` of a 1-D array, `np.sum(x, axis=0)` of a column view.
pub fn add_reduce(a: &[f64]) -> f64 {
    let mut res = 0.;
    for chunk in a.chunks(BUFSIZE) {
        res += pairwise_sum(chunk);
    }
    res
}

/// `np.add.reduce` along an axis that is not the inner loop: the result
/// starts at `0.` and adds the elements in order.
pub fn sequential_sum(a: impl IntoIterator<Item = f64>) -> f64 {
    let mut res = 0.;
    for v in a {
        res += v;
    }
    res
}

/// `ndarray.mean()` / `np.mean(x, axis=0)` / `np.average(x)` of a float64
/// array reduced as in [`add_reduce`]: the sum divided by the count
/// (`nan` for an empty array, as NumPy's "Mean of empty slice").
pub fn mean(a: &[f64]) -> f64 {
    add_reduce(a) / a.len() as f64
}

/// `np.maximum(a, b)` for two float64 values (`DOUBLE_maximum`):
/// `a >= b || isnan(a) ? a : b`.
pub fn maximum(a: f64, b: f64) -> f64 {
    if a >= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `np.minimum(a, b)` for two float64 values (`DOUBLE_minimum`):
/// `a <= b || isnan(a) ? a : b`.
pub fn minimum(a: f64, b: f64) -> f64 {
    if a <= b || a.is_nan() {
        a
    } else {
        b
    }
}

/// `ndarray.max()`: `np.maximum.reduce` from the first element.
pub fn max_reduce(a: &[f64]) -> Result<f64, PyError> {
    let (&first, rest) = a.split_first().ok_or_else(|| {
        PyError::new(
            "ValueError",
            "zero-size array to reduction operation maximum which has no identity".to_string(),
        )
    })?;
    Ok(rest.iter().fold(first, |acc, &v| maximum(acc, v)))
}

/// `ndarray.min()`: `np.minimum.reduce` from the first element.
pub fn min_reduce(a: &[f64]) -> Result<f64, PyError> {
    let (&first, rest) = a.split_first().ok_or_else(|| {
        PyError::new(
            "ValueError",
            "zero-size array to reduction operation minimum which has no identity".to_string(),
        )
    })?;
    Ok(rest.iter().fold(first, |acc, &v| minimum(acc, v)))
}

// ---------------------------------------------------------------------------
// Array constructors.
// ---------------------------------------------------------------------------

/// `np.arange(start, stop, step)` with Python float arguments.
///
/// Errors mirror `_calc_length`: `ZeroDivisionError: float division by
/// zero` for `step == 0` (Python float division), `ValueError: arange:
/// cannot compute length` for a NaN length and `ValueError: Maximum allowed
/// size exceeded` when the length overflows `npy_intp`.
pub fn arange(start: f64, stop: f64, step: f64) -> Result<Vec<f64>, PyError> {
    let next = stop - start;
    if step == 0. {
        return Err(PyError::new("ZeroDivisionError", "float division by zero".to_string()));
    }
    let value = next / step;
    let too_large = || PyError::new("ValueError", "Maximum allowed size exceeded".to_string());
    let len: i64 = if value == 0. && next != 0. {
        // Underflow and divide-by-inf check.
        if value.is_sign_negative() {
            0
        } else {
            1
        }
    } else {
        let ivalue = libm::ceil(value);
        if ivalue.is_nan() {
            return Err(PyError::new("ValueError", "arange: cannot compute length".to_string()));
        }
        // Outside `npy_intp` is an OverflowError, which `PyArray_ArangeObj`
        // replaces with this ValueError.
        if ivalue.is_infinite() {
            return Err(too_large());
        }
        trunc_float_to_bigint(ivalue).0.to_i64().ok_or_else(too_large)?
    };
    if len <= 0 {
        return Ok(Vec::new());
    }
    let len = usize::try_from(len).map_err(|_| too_large())?;
    let mut out = Vec::with_capacity(len);
    out.push(start);
    if len == 1 {
        return Ok(out);
    }
    let second = start + step;
    out.push(second);
    let delta = second - start;
    for i in 2..len {
        out.push(start + i as f64 * delta);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Bessel I0 and the Kaiser window (`numpy/lib/function_base.py`).
// ---------------------------------------------------------------------------

/// `_i0A`: Chebyshev coefficients for `exp(-x) I0(x)` on `[0, 8]`.
#[allow(clippy::excessive_precision)]
const I0A: [f64; 30] = [
    -4.41534164647933937950E-18,
    3.33079451882223809783E-17,
    -2.43127984654795469359E-16,
    1.71539128555513303061E-15,
    -1.16853328779934516808E-14,
    7.67618549860493561688E-14,
    -4.85644678311192946090E-13,
    2.95505266312963983461E-12,
    -1.72682629144155570723E-11,
    9.67580903537323691224E-11,
    -5.18979560163526290666E-10,
    2.65982372468238665035E-9,
    -1.30002500998624804212E-8,
    6.04699502254191894932E-8,
    -2.67079385394061173391E-7,
    1.11738753912010371815E-6,
    -4.41673835845875056359E-6,
    1.64484480707288970893E-5,
    -5.75419501008210370398E-5,
    1.88502885095841655729E-4,
    -5.76375574538582365885E-4,
    1.63947561694133579842E-3,
    -4.32430999505057594430E-3,
    1.05464603945949983183E-2,
    -2.37374148058994688156E-2,
    4.93052842396707084878E-2,
    -9.49010970480476444210E-2,
    1.71620901522208775349E-1,
    -3.04682672343198398683E-1,
    6.76795274409476084995E-1,
];

/// `_i0B`: Chebyshev coefficients for `exp(-x) sqrt(x) I0(x)` on `[8, inf]`.
#[allow(clippy::excessive_precision)]
const I0B: [f64; 25] = [
    -7.23318048787475395456E-18,
    -4.83050448594418207126E-18,
    4.46562142029675999901E-17,
    3.46122286769746109310E-17,
    -2.82762398051658348494E-16,
    -3.42548561967721913462E-16,
    1.77256013305652638360E-15,
    3.81168066935262242075E-15,
    -9.55484669882830764870E-15,
    -4.15056934728722208663E-14,
    1.54008621752140982691E-14,
    3.85277838274214270114E-13,
    7.18012445138366623367E-13,
    -1.79417853150680611778E-12,
    -1.32158118404477131188E-11,
    -3.14991652796324136454E-11,
    1.18891471078464383424E-11,
    4.94060238822496958910E-10,
    3.39623202570838634515E-9,
    2.26666899049817806459E-8,
    2.04891858946906374183E-7,
    2.89137052083475648297E-6,
    6.88975834691682398426E-5,
    3.36911647825569408990E-3,
    8.04490411014108831608E-1,
];

/// `_chbevl(x, vals)`.
fn chbevl(x: f64, vals: &[f64]) -> f64 {
    let mut b0 = vals[0];
    let mut b1 = 0.0;
    let mut b2 = 0.0;
    for &v in &vals[1..] {
        b2 = b1;
        b1 = b0;
        b0 = x * b1 - b2 + v;
    }
    0.5 * (b0 - b2)
}

/// `np.i0(x)` for one float64 value.
pub fn i0(x: f64) -> f64 {
    let x = if x < 0. { -x } else { x };
    if x <= 8.0 {
        // `_i0_1`
        libm::exp(x) * chbevl(x / 2.0 - 2., &I0A)
    } else {
        // `_i0_2` (also taken by NaN, which fails `x <= 8.0`)
        libm::exp(x) * chbevl(32.0 / x - 2.0, &I0B) / libm::sqrt(x)
    }
}

/// `np.kaiser(M, beta)`.
pub fn kaiser(m: usize, beta: f64) -> Vec<f64> {
    if m == 1 {
        return vec![1.];
    }
    let alpha = (m as f64 - 1.) / 2.0;
    let denom = i0(beta);
    (0..m)
        .map(|n| {
            let t = (n as f64 - alpha) / alpha;
            i0(beta * libm::sqrt(1. - t * t)) / denom
        })
        .collect()
}

// ---------------------------------------------------------------------------
// `np.interp` (`arr_interp` in `numpy/core/src/multiarray/compiled_base.c`).
// ---------------------------------------------------------------------------

const LIKELY_IN_CACHE_SIZE: usize = 8;

/// `binary_search_with_guess`: `-1` below the range, `len` above it, else
/// the `i` with `arr[i] <= key < arr[i+1]` (`len-1` for `key == arr[len-1]`).
fn binary_search_with_guess(key: f64, arr: &[f64], guess: isize) -> isize {
    let len = arr.len();
    let mut imin = 0usize;
    let mut imax = len;
    if key > arr[len - 1] {
        return len as isize;
    } else if key < arr[0] {
        return -1;
    }
    if len <= 4 {
        let mut i = 1;
        while i < len && key >= arr[i] {
            i += 1;
        }
        return i as isize - 1;
    }
    let mut guess = guess;
    if guess > len as isize - 3 {
        guess = len as isize - 3;
    }
    if guess < 1 {
        guess = 1;
    }
    let guess = guess as usize;
    if key < arr[guess] {
        if key < arr[guess - 1] {
            imax = guess - 1;
            if guess > LIKELY_IN_CACHE_SIZE && key >= arr[guess - LIKELY_IN_CACHE_SIZE] {
                imin = guess - LIKELY_IN_CACHE_SIZE;
            }
        } else {
            return guess as isize - 1;
        }
    } else if key < arr[guess + 1] {
        return guess as isize;
    } else if key < arr[guess + 2] {
        return guess as isize + 1;
    } else {
        imin = guess + 2;
        if (guess as isize) < len as isize - LIKELY_IN_CACHE_SIZE as isize - 1
            && key < arr[guess + LIKELY_IN_CACHE_SIZE]
        {
            imax = guess + LIKELY_IN_CACHE_SIZE;
        }
    }
    while imin < imax {
        let imid = imin + ((imax - imin) >> 1);
        if key >= arr[imid] {
            imin = imid + 1;
        } else {
            imax = imid;
        }
    }
    imin as isize - 1
}

/// `np.interp(x, xp, fp)` with the default `left`/`right` (the end values
/// of `fp`).
pub fn interp(x: &[f64], xp: &[f64], fp: &[f64]) -> Result<Vec<f64>, PyError> {
    let lenxp = xp.len();
    if lenxp == 0 {
        return Err(PyError::new("ValueError", "array of sample points is empty".to_string()));
    }
    if fp.len() != lenxp {
        return Err(PyError::new("ValueError", "fp and xp are not of the same length.".to_string()));
    }
    let lval = fp[0];
    let rval = fp[lenxp - 1];
    if lenxp == 1 {
        let (xp_val, fp_val) = (xp[0], fp[0]);
        return Ok(x
            .iter()
            .map(|&x_val| {
                if x_val < xp_val {
                    lval
                } else if x_val > xp_val {
                    rval
                } else {
                    fp_val
                }
            })
            .collect());
    }
    // Slopes are precomputed only when there are fewer of them than points;
    // both forms evaluate the same expression.
    let slopes: Option<Vec<f64>> = (lenxp <= x.len()).then(|| {
        (0..lenxp - 1).map(|i| (fp[i + 1] - fp[i]) / (xp[i + 1] - xp[i])).collect()
    });
    let mut j: isize = 0;
    let mut out = Vec::with_capacity(x.len());
    for &x_val in x {
        if x_val.is_nan() {
            out.push(x_val);
            continue;
        }
        j = binary_search_with_guess(x_val, xp, j);
        let v = if j == -1 {
            lval
        } else if j == lenxp as isize {
            rval
        } else if j == lenxp as isize - 1 {
            fp[j as usize]
        } else {
            let ju = j as usize;
            if xp[ju] == x_val {
                fp[ju]
            } else {
                let slope = match &slopes {
                    Some(s) => s[ju],
                    None => (fp[ju + 1] - fp[ju]) / (xp[ju + 1] - xp[ju]),
                };
                slope * (x_val - xp[ju]) + fp[ju]
            }
        };
        out.push(v);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Complex values.
// ---------------------------------------------------------------------------

/// A NumPy `complex128` value.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Complex64 {
    /// Real part.
    pub re: f64,
    /// Imaginary part.
    pub im: f64,
}

impl Complex64 {
    /// `complex(re, im)`.
    pub fn new(re: f64, im: f64) -> Self {
        Complex64 { re, im }
    }

    /// `np.conjugate(z)`.
    pub fn conj(self) -> Self {
        Complex64 { re: self.re, im: -self.im }
    }
}

impl std::ops::Mul for Complex64 {
    type Output = Complex64;

    /// `z1 * z2` (`CDOUBLE_multiply`): `(ar*br - ai*bi, ar*bi + ai*br)`.
    /// A real scalar operand is the complex `(s, 0.)`.
    fn mul(self, other: Complex64) -> Complex64 {
        Complex64 {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
}

// ---------------------------------------------------------------------------
// FFT (`numpy/fft/fftpack.c` and `fftpack_litemodule.c`, NumPy 1.16.4).
// ---------------------------------------------------------------------------

/// `TWOPI` of `fftpack.c` (`6.283185307179586476925286766559005768391`,
/// which rounds to `2*pi`).
const TWOPI: f64 = std::f64::consts::TAU;

/// `(sin2pi(m, n), cos2pi(m, n))` of `fftpack.c`. Each `sincos2pi` case
/// evaluates `sin` and `cos` of the same argument, which GCC compiles into
/// one glibc `sincos` call.
fn sincos_2pi(m: usize, n: usize) -> (f64, f64) {
    libm::sincos((TWOPI * m as f64) / n as f64)
}

/// `sincos2pi(m, n)`: `(sin(2pi m/n), cos(2pi m/n))` from the first-octant
/// reduction.
fn sincos2pi(m: usize, n: usize) -> (f64, f64) {
    let n8 = 8 * n;
    let mut m8 = (8 * m) % n8;
    let octant = m8 / n;
    m8 %= n;
    let (co, si) = match octant {
        0 => {
            let (s, c) = sincos_2pi(m8, n8);
            (c, s)
        }
        1 => {
            let (s, c) = sincos_2pi(n - m8, n8);
            (s, c)
        }
        2 => {
            let (s, c) = sincos_2pi(m8, n8);
            (-s, c)
        }
        3 => {
            let (s, c) = sincos_2pi(n - m8, n8);
            (-c, s)
        }
        4 => {
            let (s, c) = sincos_2pi(m8, n8);
            (-c, -s)
        }
        5 => {
            let (s, c) = sincos_2pi(n - m8, n8);
            (-s, -c)
        }
        6 => {
            let (s, c) = sincos_2pi(m8, n8);
            (s, -c)
        }
        _ => {
            let (s, c) = sincos_2pi(n - m8, n8);
            (c, -s)
        }
    };
    (si, co)
}

/// `factorize(n, ifac, ntryh)`: `ifac[0] = n`, `ifac[1]` the factor count,
/// the factors from `ifac[2]`.
fn factorize(n: usize, ntryh: [usize; 4]) -> Vec<usize> {
    let mut ifac = vec![0usize; 2];
    let mut ntry = 3;
    let mut j = 0;
    let mut nf = 0;
    let mut nl = n;
    'start: loop {
        if j < ntryh.len() {
            ntry = ntryh[j];
        } else {
            ntry += 2;
        }
        j += 1;
        loop {
            let nq = nl / ntry;
            let nr = nl - ntry * nq;
            if nr != 0 {
                continue 'start;
            }
            nf += 1;
            ifac.push(ntry);
            nl = nq;
            if ntry == 2 && nf != 1 {
                for i in 2..=nf {
                    let ib = nf - i + 2;
                    ifac[ib + 1] = ifac[ib];
                }
                ifac[2] = 2;
            }
            if nl == 1 {
                break 'start;
            }
        }
    }
    ifac[0] = n;
    ifac[1] = nf;
    ifac
}

/// Twiddles and factors of a real FFT of length `n` (`npy_rffti`).
#[derive(Debug, Clone)]
pub struct RfftPlan {
    n: usize,
    wa: Vec<f64>,
    ifac: Vec<usize>,
}

fn invalid_points(n: usize) -> PyError {
    PyError::new("ValueError", format!("Invalid number of FFT data points ({n}) specified."))
}

impl RfftPlan {
    /// `rffti(n)`; `n < 1` is NumPy's `ValueError`.
    pub fn new(n: usize) -> Result<Self, PyError> {
        if n < 1 {
            return Err(invalid_points(n));
        }
        let mut wa = vec![0.0; n];
        let ifac = if n == 1 { Vec::new() } else { rffti1(n, &mut wa) };
        Ok(RfftPlan { n, wa, ifac })
    }

    /// Transform length.
    pub fn len(&self) -> usize {
        self.n
    }

    /// Always false: a plan has at least one point.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// `np.fft.rfft(x, n)` for one row: `x` is cropped or zero-padded to `n`
    /// and the `n/2 + 1` coefficients are returned.
    pub fn rfft(&self, x: &[f64]) -> Vec<Complex64> {
        let n = self.n;
        let bins = n / 2 + 1;
        // `ret` holds `2*bins` doubles; the input goes to `rptr+1`.
        let mut r = vec![0.0; 2 * bins];
        let take = x.len().min(n);
        r[1..1 + take].copy_from_slice(&x[..take]);
        if n != 1 {
            let mut ch = vec![0.0; n];
            rfftf1(n, &mut r[1..=n], &mut ch, &self.wa, &self.ifac);
        }
        r[0] = r[1];
        r[1] = 0.0;
        r.chunks_exact(2).map(|p| Complex64::new(p[0], p[1])).collect()
    }
}

/// Twiddles and factors of a complex FFT of length `n` (`npy_cffti`).
#[derive(Debug, Clone)]
pub struct FftPlan {
    n: usize,
    wa: Vec<f64>,
    ifac: Vec<usize>,
}

impl FftPlan {
    /// `cffti(n)`; `n < 1` is NumPy's `ValueError`.
    pub fn new(n: usize) -> Result<Self, PyError> {
        if n < 1 {
            return Err(invalid_points(n));
        }
        let mut wa = vec![0.0; 2 * n];
        let ifac = if n == 1 { Vec::new() } else { cffti1(n, &mut wa) };
        Ok(FftPlan { n, wa, ifac })
    }

    /// Transform length.
    pub fn len(&self) -> usize {
        self.n
    }

    /// Always false: a plan has at least one point.
    pub fn is_empty(&self) -> bool {
        false
    }

    /// `np.fft.fft(x, n)` for one row: `x` is cropped or zero-padded to `n`.
    pub fn fft(&self, x: &[Complex64]) -> Vec<Complex64> {
        let n = self.n;
        let mut c = vec![0.0; 2 * n];
        for (slot, v) in c.chunks_exact_mut(2).zip(x.iter()) {
            slot[0] = v.re;
            slot[1] = v.im;
        }
        if n != 1 {
            let mut ch = vec![0.0; 2 * n];
            cfftf1(n, &mut c, &mut ch, &self.wa, &self.ifac, -1.0);
        }
        c.chunks_exact(2).map(|p| Complex64::new(p[0], p[1])).collect()
    }
}

/// `np.fft.rfft(x, n)` of a 1-D float64 array.
pub fn rfft(x: &[f64], n: usize) -> Result<Vec<Complex64>, PyError> {
    Ok(RfftPlan::new(n)?.rfft(x))
}

/// `np.fft.fft(x, n)` of a 1-D complex128 array.
pub fn fft(x: &[Complex64], n: usize) -> Result<Vec<Complex64>, PyError> {
    Ok(FftPlan::new(n)?.fft(x))
}

/// `np.fft.rfftfreq(n, d)`: `k * (1.0/(n*d))` for `k` in `0..=n/2`.
pub fn rfftfreq(n: usize, d: f64) -> Vec<f64> {
    let val = 1.0 / (n as f64 * d);
    (0..n / 2 + 1).map(|k| k as f64 * val).collect()
}

/// `rffti1(n, wa, ifac)`.
fn rffti1(n: usize, wa: &mut [f64]) -> Vec<usize> {
    let ifac = factorize(n, [4, 2, 3, 5]);
    let nf = ifac[1];
    let mut is = 0;
    let nfm1 = nf - 1;
    let mut l1 = 1;
    if nfm1 == 0 {
        return ifac;
    }
    for k1 in 1..=nfm1 {
        let ip = ifac[k1 + 1];
        let mut ld = 0;
        let l2 = l1 * ip;
        let ido = n / l2;
        let ipm = ip - 1;
        for _j in 1..=ipm {
            ld += l1;
            let mut i = is;
            let mut fi = 0;
            let mut ii = 3;
            while ii <= ido {
                i += 2;
                fi += 1;
                let (si, co) = sincos2pi(fi * ld, n);
                wa[i - 1] = si;
                wa[i - 2] = co;
                ii += 2;
            }
            is += ido;
        }
        l1 = l2;
    }
    ifac
}

/// `cffti1(n, wa, ifac)`.
fn cffti1(n: usize, wa: &mut [f64]) -> Vec<usize> {
    let ifac = factorize(n, [3, 4, 2, 5]);
    let nf = ifac[1];
    let mut i = 1;
    let mut l1 = 1;
    for k1 in 1..=nf {
        let ip = ifac[k1 + 1];
        let mut ld = 0;
        let l2 = l1 * ip;
        let ido = n / l2;
        let idot = ido + ido + 2;
        let ipm = ip - 1;
        for _j in 1..=ipm {
            let i1 = i;
            wa[i - 1] = 1.;
            wa[i] = 0.;
            ld += l1;
            let mut fi = 0;
            let mut ii = 4;
            while ii <= idot {
                i += 2;
                fi += 1;
                let (si, co) = sincos2pi(fi * ld, n);
                wa[i] = si;
                wa[i - 1] = co;
                ii += 2;
            }
            if ip > 5 {
                wa[i1 - 1] = wa[i - 1];
                wa[i1] = wa[i];
            }
        }
        l1 = l2;
    }
    ifac
}

/// `rfftf1(n, c, ch, wa, ifac)`: forward real transform in place in `c`.
fn rfftf1(n: usize, c: &mut [f64], ch: &mut [f64], wa: &[f64], ifac: &[usize]) {
    let nf = ifac[1];
    let mut na = true;
    let mut l2 = n;
    let mut iw = n - 1;
    for k1 in 1..=nf {
        let kh = nf - k1;
        let ip = ifac[kh + 2];
        let l1 = l2 / ip;
        let ido = n / l2;
        let idl1 = ido * l1;
        iw -= (ip - 1) * ido;
        na = !na;
        match ip {
            4 => {
                let ix2 = iw + ido;
                let ix3 = ix2 + ido;
                if na {
                    radf4(ido, l1, ch, c, &wa[iw..], &wa[ix2..], &wa[ix3..]);
                } else {
                    radf4(ido, l1, c, ch, &wa[iw..], &wa[ix2..], &wa[ix3..]);
                }
            }
            2 => {
                if na {
                    radf2(ido, l1, ch, c, &wa[iw..]);
                } else {
                    radf2(ido, l1, c, ch, &wa[iw..]);
                }
            }
            3 => {
                let ix2 = iw + ido;
                if na {
                    radf3(ido, l1, ch, c, &wa[iw..], &wa[ix2..]);
                } else {
                    radf3(ido, l1, c, ch, &wa[iw..], &wa[ix2..]);
                }
            }
            5 => {
                let ix2 = iw + ido;
                let ix3 = ix2 + ido;
                let ix4 = ix3 + ido;
                if na {
                    radf5(ido, l1, ch, c, &wa[iw..], &wa[ix2..], &wa[ix3..], &wa[ix4..]);
                } else {
                    radf5(ido, l1, c, ch, &wa[iw..], &wa[ix2..], &wa[ix3..], &wa[ix4..]);
                }
            }
            _ => {
                if ido == 1 {
                    na = !na;
                }
                if !na {
                    radfg(ido, ip, l1, idl1, c, ch, &wa[iw..]);
                    na = true;
                } else {
                    radfg(ido, ip, l1, idl1, ch, c, &wa[iw..]);
                    na = false;
                }
            }
        }
        l2 = l1;
    }
    if na {
        return;
    }
    c[..n].copy_from_slice(&ch[..n]);
}

/// `cfftf1(n, c, ch, wa, ifac, isign)`: complex transform in place in `c`.
fn cfftf1(n: usize, c: &mut [f64], ch: &mut [f64], wa: &[f64], ifac: &[usize], isign: f64) {
    let nf = ifac[1];
    let mut na = false;
    let mut l1 = 1;
    let mut iw = 0;
    for &ip in &ifac[2..=nf + 1] {
        let l2 = ip * l1;
        let ido = n / l2;
        let idot = ido + ido;
        let idl1 = idot * l1;
        match ip {
            4 => {
                let ix2 = iw + idot;
                let ix3 = ix2 + idot;
                if na {
                    passf4(idot, l1, ch, c, &wa[iw..], &wa[ix2..], &wa[ix3..], isign);
                } else {
                    passf4(idot, l1, c, ch, &wa[iw..], &wa[ix2..], &wa[ix3..], isign);
                }
                na = !na;
            }
            2 => {
                if na {
                    passf2(idot, l1, ch, c, &wa[iw..], isign);
                } else {
                    passf2(idot, l1, c, ch, &wa[iw..], isign);
                }
                na = !na;
            }
            3 => {
                let ix2 = iw + idot;
                if na {
                    passf3(idot, l1, ch, c, &wa[iw..], &wa[ix2..], isign);
                } else {
                    passf3(idot, l1, c, ch, &wa[iw..], &wa[ix2..], isign);
                }
                na = !na;
            }
            5 => {
                let ix2 = iw + idot;
                let ix3 = ix2 + idot;
                let ix4 = ix3 + idot;
                if na {
                    passf5(idot, l1, ch, c, &wa[iw..], &wa[ix2..], &wa[ix3..], &wa[ix4..], isign);
                } else {
                    passf5(idot, l1, c, ch, &wa[iw..], &wa[ix2..], &wa[ix3..], &wa[ix4..], isign);
                }
                na = !na;
            }
            _ => {
                let nac = if na {
                    passf(idot, ip, l1, idl1, ch, c, &wa[iw..], isign)
                } else {
                    passf(idot, ip, l1, idl1, c, ch, &wa[iw..], isign)
                };
                if nac {
                    na = !na;
                }
            }
        }
        l1 = l2;
        iw += (ip - 1) * idot;
    }
    if !na {
        return;
    }
    c[..2 * n].copy_from_slice(&ch[..2 * n]);
}

fn passf2(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa1: &[f64], isign: f64) {
    if ido <= 2 {
        for k in 0..l1 {
            let ah = k * ido;
            let ac = 2 * k * ido;
            ch[ah] = cc[ac] + cc[ac + ido];
            ch[ah + ido * l1] = cc[ac] - cc[ac + ido];
            ch[ah + 1] = cc[ac + 1] + cc[ac + ido + 1];
            ch[ah + ido * l1 + 1] = cc[ac + 1] - cc[ac + ido + 1];
        }
    } else {
        for k in 0..l1 {
            let mut i = 0;
            while i + 1 < ido {
                let ah = i + k * ido;
                let ac = i + 2 * k * ido;
                ch[ah] = cc[ac] + cc[ac + ido];
                let tr2 = cc[ac] - cc[ac + ido];
                ch[ah + 1] = cc[ac + 1] + cc[ac + 1 + ido];
                let ti2 = cc[ac + 1] - cc[ac + 1 + ido];
                ch[ah + l1 * ido + 1] = wa1[i] * ti2 + isign * wa1[i + 1] * tr2;
                ch[ah + l1 * ido] = wa1[i] * tr2 - isign * wa1[i + 1] * ti2;
                i += 2;
            }
        }
    }
}

fn passf3(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa1: &[f64], wa2: &[f64], isign: f64) {
    const TAUR: f64 = -0.5;
    #[allow(clippy::excessive_precision)]
    const TAUI: f64 = 0.86602540378443864676;
    if ido == 2 {
        for k in 1..=l1 {
            let ac = (3 * k - 2) * ido;
            let tr2 = cc[ac] + cc[ac + ido];
            let cr2 = cc[ac - ido] + TAUR * tr2;
            let ah = (k - 1) * ido;
            ch[ah] = cc[ac - ido] + tr2;

            let ti2 = cc[ac + 1] + cc[ac + ido + 1];
            let ci2 = cc[ac - ido + 1] + TAUR * ti2;
            ch[ah + 1] = cc[ac - ido + 1] + ti2;

            let cr3 = isign * TAUI * (cc[ac] - cc[ac + ido]);
            let ci3 = isign * TAUI * (cc[ac + 1] - cc[ac + ido + 1]);
            ch[ah + l1 * ido] = cr2 - ci3;
            ch[ah + 2 * l1 * ido] = cr2 + ci3;
            ch[ah + l1 * ido + 1] = ci2 + cr3;
            ch[ah + 2 * l1 * ido + 1] = ci2 - cr3;
        }
    } else {
        for k in 1..=l1 {
            let mut i = 0;
            while i + 1 < ido {
                let ac = i + (3 * k - 2) * ido;
                let tr2 = cc[ac] + cc[ac + ido];
                let cr2 = cc[ac - ido] + TAUR * tr2;
                let ah = i + (k - 1) * ido;
                ch[ah] = cc[ac - ido] + tr2;
                let ti2 = cc[ac + 1] + cc[ac + ido + 1];
                let ci2 = cc[ac - ido + 1] + TAUR * ti2;
                ch[ah + 1] = cc[ac - ido + 1] + ti2;
                let cr3 = isign * TAUI * (cc[ac] - cc[ac + ido]);
                let ci3 = isign * TAUI * (cc[ac + 1] - cc[ac + ido + 1]);
                let dr2 = cr2 - ci3;
                let dr3 = cr2 + ci3;
                let di2 = ci2 + cr3;
                let di3 = ci2 - cr3;
                ch[ah + l1 * ido + 1] = wa1[i] * di2 + isign * wa1[i + 1] * dr2;
                ch[ah + l1 * ido] = wa1[i] * dr2 - isign * wa1[i + 1] * di2;
                ch[ah + 2 * l1 * ido + 1] = wa2[i] * di3 + isign * wa2[i + 1] * dr3;
                ch[ah + 2 * l1 * ido] = wa2[i] * dr3 - isign * wa2[i + 1] * di3;
                i += 2;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn passf4(
    ido: usize,
    l1: usize,
    cc: &[f64],
    ch: &mut [f64],
    wa1: &[f64],
    wa2: &[f64],
    wa3: &[f64],
    isign: f64,
) {
    if ido == 2 {
        for k in 0..l1 {
            let ac = 4 * k * ido + 1;
            let ti1 = cc[ac] - cc[ac + 2 * ido];
            let ti2 = cc[ac] + cc[ac + 2 * ido];
            let tr4 = cc[ac + 3 * ido] - cc[ac + ido];
            let ti3 = cc[ac + ido] + cc[ac + 3 * ido];
            let tr1 = cc[ac - 1] - cc[ac + 2 * ido - 1];
            let tr2 = cc[ac - 1] + cc[ac + 2 * ido - 1];
            let ti4 = cc[ac + ido - 1] - cc[ac + 3 * ido - 1];
            let tr3 = cc[ac + ido - 1] + cc[ac + 3 * ido - 1];
            let ah = k * ido;
            ch[ah] = tr2 + tr3;
            ch[ah + 2 * l1 * ido] = tr2 - tr3;
            ch[ah + 1] = ti2 + ti3;
            ch[ah + 2 * l1 * ido + 1] = ti2 - ti3;
            ch[ah + l1 * ido] = tr1 + isign * tr4;
            ch[ah + 3 * l1 * ido] = tr1 - isign * tr4;
            ch[ah + l1 * ido + 1] = ti1 + isign * ti4;
            ch[ah + 3 * l1 * ido + 1] = ti1 - isign * ti4;
        }
    } else {
        for k in 0..l1 {
            let mut i = 0;
            while i + 1 < ido {
                let ac = i + 1 + 4 * k * ido;
                let ti1 = cc[ac] - cc[ac + 2 * ido];
                let ti2 = cc[ac] + cc[ac + 2 * ido];
                let ti3 = cc[ac + ido] + cc[ac + 3 * ido];
                let tr4 = cc[ac + 3 * ido] - cc[ac + ido];
                let tr1 = cc[ac - 1] - cc[ac + 2 * ido - 1];
                let tr2 = cc[ac - 1] + cc[ac + 2 * ido - 1];
                let ti4 = cc[ac + ido - 1] - cc[ac + 3 * ido - 1];
                let tr3 = cc[ac + ido - 1] + cc[ac + 3 * ido - 1];
                let ah = i + k * ido;
                ch[ah] = tr2 + tr3;
                let cr3 = tr2 - tr3;
                ch[ah + 1] = ti2 + ti3;
                let ci3 = ti2 - ti3;
                let cr2 = tr1 + isign * tr4;
                let cr4 = tr1 - isign * tr4;
                let ci2 = ti1 + isign * ti4;
                let ci4 = ti1 - isign * ti4;
                ch[ah + l1 * ido] = wa1[i] * cr2 - isign * wa1[i + 1] * ci2;
                ch[ah + l1 * ido + 1] = wa1[i] * ci2 + isign * wa1[i + 1] * cr2;
                ch[ah + 2 * l1 * ido] = wa2[i] * cr3 - isign * wa2[i + 1] * ci3;
                ch[ah + 2 * l1 * ido + 1] = wa2[i] * ci3 + isign * wa2[i + 1] * cr3;
                ch[ah + 3 * l1 * ido] = wa3[i] * cr4 - isign * wa3[i + 1] * ci4;
                ch[ah + 3 * l1 * ido + 1] = wa3[i] * ci4 + isign * wa3[i + 1] * cr4;
                i += 2;
            }
        }
    }
}

#[allow(clippy::too_many_arguments, clippy::excessive_precision)]
fn passf5(
    ido: usize,
    l1: usize,
    cc: &[f64],
    ch: &mut [f64],
    wa1: &[f64],
    wa2: &[f64],
    wa3: &[f64],
    wa4: &[f64],
    isign: f64,
) {
    const TR11: f64 = 0.3090169943749474241;
    const TI11: f64 = 0.95105651629515357212;
    const TR12: f64 = -0.8090169943749474241;
    const TI12: f64 = 0.58778525229247312917;
    if ido == 2 {
        for k in 1..=l1 {
            let ac = (5 * k - 4) * ido + 1;
            let ti5 = cc[ac] - cc[ac + 3 * ido];
            let ti2 = cc[ac] + cc[ac + 3 * ido];
            let ti4 = cc[ac + ido] - cc[ac + 2 * ido];
            let ti3 = cc[ac + ido] + cc[ac + 2 * ido];
            let tr5 = cc[ac - 1] - cc[ac + 3 * ido - 1];
            let tr2 = cc[ac - 1] + cc[ac + 3 * ido - 1];
            let tr4 = cc[ac + ido - 1] - cc[ac + 2 * ido - 1];
            let tr3 = cc[ac + ido - 1] + cc[ac + 2 * ido - 1];
            let ah = (k - 1) * ido;
            ch[ah] = cc[ac - ido - 1] + tr2 + tr3;
            ch[ah + 1] = cc[ac - ido] + ti2 + ti3;
            let cr2 = cc[ac - ido - 1] + TR11 * tr2 + TR12 * tr3;
            let ci2 = cc[ac - ido] + TR11 * ti2 + TR12 * ti3;
            let cr3 = cc[ac - ido - 1] + TR12 * tr2 + TR11 * tr3;
            let ci3 = cc[ac - ido] + TR12 * ti2 + TR11 * ti3;
            let cr5 = isign * (TI11 * tr5 + TI12 * tr4);
            let ci5 = isign * (TI11 * ti5 + TI12 * ti4);
            let cr4 = isign * (TI12 * tr5 - TI11 * tr4);
            let ci4 = isign * (TI12 * ti5 - TI11 * ti4);
            ch[ah + l1 * ido] = cr2 - ci5;
            ch[ah + 4 * l1 * ido] = cr2 + ci5;
            ch[ah + l1 * ido + 1] = ci2 + cr5;
            ch[ah + 2 * l1 * ido + 1] = ci3 + cr4;
            ch[ah + 2 * l1 * ido] = cr3 - ci4;
            ch[ah + 3 * l1 * ido] = cr3 + ci4;
            ch[ah + 3 * l1 * ido + 1] = ci3 - cr4;
            ch[ah + 4 * l1 * ido + 1] = ci2 - cr5;
        }
    } else {
        for k in 1..=l1 {
            let mut i = 0;
            while i + 1 < ido {
                let ac = i + 1 + (k * 5 - 4) * ido;
                let ti5 = cc[ac] - cc[ac + 3 * ido];
                let ti2 = cc[ac] + cc[ac + 3 * ido];
                let ti4 = cc[ac + ido] - cc[ac + 2 * ido];
                let ti3 = cc[ac + ido] + cc[ac + 2 * ido];
                let tr5 = cc[ac - 1] - cc[ac + 3 * ido - 1];
                let tr2 = cc[ac - 1] + cc[ac + 3 * ido - 1];
                let tr4 = cc[ac + ido - 1] - cc[ac + 2 * ido - 1];
                let tr3 = cc[ac + ido - 1] + cc[ac + 2 * ido - 1];
                let ah = i + (k - 1) * ido;
                ch[ah] = cc[ac - ido - 1] + tr2 + tr3;
                ch[ah + 1] = cc[ac - ido] + ti2 + ti3;
                let cr2 = cc[ac - ido - 1] + TR11 * tr2 + TR12 * tr3;
                let ci2 = cc[ac - ido] + TR11 * ti2 + TR12 * ti3;
                let cr3 = cc[ac - ido - 1] + TR12 * tr2 + TR11 * tr3;
                let ci3 = cc[ac - ido] + TR12 * ti2 + TR11 * ti3;
                let cr5 = isign * (TI11 * tr5 + TI12 * tr4);
                let ci5 = isign * (TI11 * ti5 + TI12 * ti4);
                let cr4 = isign * (TI12 * tr5 - TI11 * tr4);
                let ci4 = isign * (TI12 * ti5 - TI11 * ti4);
                let dr3 = cr3 - ci4;
                let dr4 = cr3 + ci4;
                let di3 = ci3 + cr4;
                let di4 = ci3 - cr4;
                let dr5 = cr2 + ci5;
                let dr2 = cr2 - ci5;
                let di5 = ci2 - cr5;
                let di2 = ci2 + cr5;
                ch[ah + l1 * ido] = wa1[i] * dr2 - isign * wa1[i + 1] * di2;
                ch[ah + l1 * ido + 1] = wa1[i] * di2 + isign * wa1[i + 1] * dr2;
                ch[ah + 2 * l1 * ido] = wa2[i] * dr3 - isign * wa2[i + 1] * di3;
                ch[ah + 2 * l1 * ido + 1] = wa2[i] * di3 + isign * wa2[i + 1] * dr3;
                ch[ah + 3 * l1 * ido] = wa3[i] * dr4 - isign * wa3[i + 1] * di4;
                ch[ah + 3 * l1 * ido + 1] = wa3[i] * di4 + isign * wa3[i + 1] * dr4;
                ch[ah + 4 * l1 * ido] = wa4[i] * dr5 - isign * wa4[i + 1] * di5;
                ch[ah + 4 * l1 * ido + 1] = wa4[i] * di5 + isign * wa4[i + 1] * dr5;
                i += 2;
            }
        }
    }
}

/// `passf` (generic odd radix). Returns `nac != 0`: whether the result is
/// in `ch` (`ido == 2`) rather than back in `cc`.
#[allow(clippy::too_many_arguments)]
fn passf(
    ido: usize,
    ip: usize,
    l1: usize,
    idl1: usize,
    cc: &mut [f64],
    ch: &mut [f64],
    wa: &[f64],
    isign: f64,
) -> bool {
    let idot = ido / 2;
    let ipph = ip.div_ceil(2);
    let idp = ip * ido;
    if ido >= l1 {
        for j in 1..ipph {
            let jc = ip - j;
            for k in 0..l1 {
                for i in 0..ido {
                    ch[i + (k + j * l1) * ido] =
                        cc[i + (j + k * ip) * ido] + cc[i + (jc + k * ip) * ido];
                    ch[i + (k + jc * l1) * ido] =
                        cc[i + (j + k * ip) * ido] - cc[i + (jc + k * ip) * ido];
                }
            }
        }
        for k in 0..l1 {
            for i in 0..ido {
                ch[i + k * ido] = cc[i + k * ip * ido];
            }
        }
    } else {
        for j in 1..ipph {
            let jc = ip - j;
            for i in 0..ido {
                for k in 0..l1 {
                    ch[i + (k + j * l1) * ido] =
                        cc[i + (j + k * ip) * ido] + cc[i + (jc + k * ip) * ido];
                    ch[i + (k + jc * l1) * ido] =
                        cc[i + (j + k * ip) * ido] - cc[i + (jc + k * ip) * ido];
                }
            }
        }
        for i in 0..ido {
            for k in 0..l1 {
                ch[i + k * ido] = cc[i + k * ip * ido];
            }
        }
    }

    // C: `idl = 2 - ido; inc = 0;` then `idl += ido; inc += ido` per `l`.
    let mut inc = 0;
    for l in 1..ipph {
        let lc = ip - l;
        let idl = 2 + (l - 1) * ido;
        for ik in 0..idl1 {
            cc[ik + l * idl1] = ch[ik] + wa[idl - 2] * ch[ik + idl1];
            cc[ik + lc * idl1] = isign * wa[idl - 1] * ch[ik + (ip - 1) * idl1];
        }
        let mut idlj = idl;
        inc += ido;
        for j in 2..ipph {
            let jc = ip - j;
            idlj += inc;
            if idlj > idp {
                idlj -= idp;
            }
            let war = wa[idlj - 2];
            let wai = wa[idlj - 1];
            for ik in 0..idl1 {
                cc[ik + l * idl1] += war * ch[ik + j * idl1];
                cc[ik + lc * idl1] += isign * wai * ch[ik + jc * idl1];
            }
        }
    }
    for j in 1..ipph {
        for ik in 0..idl1 {
            ch[ik] += ch[ik + j * idl1];
        }
    }
    for j in 1..ipph {
        let jc = ip - j;
        let mut ik = 1;
        while ik < idl1 {
            ch[ik - 1 + j * idl1] = cc[ik - 1 + j * idl1] - cc[ik + jc * idl1];
            ch[ik - 1 + jc * idl1] = cc[ik - 1 + j * idl1] + cc[ik + jc * idl1];
            ch[ik + j * idl1] = cc[ik + j * idl1] + cc[ik - 1 + jc * idl1];
            ch[ik + jc * idl1] = cc[ik + j * idl1] - cc[ik - 1 + jc * idl1];
            ik += 2;
        }
    }
    if ido == 2 {
        return true;
    }
    cc[..idl1].copy_from_slice(&ch[..idl1]);
    for j in 1..ip {
        for k in 0..l1 {
            cc[(k + j * l1) * ido] = ch[(k + j * l1) * ido];
            cc[(k + j * l1) * ido + 1] = ch[(k + j * l1) * ido + 1];
        }
    }
    if idot <= l1 {
        let mut idij = 0;
        for j in 1..ip {
            idij += 2;
            let mut i = 3;
            while i < ido {
                idij += 2;
                for k in 0..l1 {
                    cc[i - 1 + (k + j * l1) * ido] = wa[idij - 2] * ch[i - 1 + (k + j * l1) * ido]
                        - isign * wa[idij - 1] * ch[i + (k + j * l1) * ido];
                    cc[i + (k + j * l1) * ido] = wa[idij - 2] * ch[i + (k + j * l1) * ido]
                        + isign * wa[idij - 1] * ch[i - 1 + (k + j * l1) * ido];
                }
                i += 2;
            }
        }
    } else {
        for j in 1..ip {
            // C: `idj = 2 - ido;` then `idj += ido` per `j`.
            let idj = 2 + (j - 1) * ido;
            for k in 0..l1 {
                let mut idij = idj;
                let mut i = 3;
                while i < ido {
                    idij += 2;
                    cc[i - 1 + (k + j * l1) * ido] = wa[idij - 2] * ch[i - 1 + (k + j * l1) * ido]
                        - isign * wa[idij - 1] * ch[i + (k + j * l1) * ido];
                    cc[i + (k + j * l1) * ido] = wa[idij - 2] * ch[i + (k + j * l1) * ido]
                        + isign * wa[idij - 1] * ch[i - 1 + (k + j * l1) * ido];
                    i += 2;
                }
            }
        }
    }
    false
}

fn radf2(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa1: &[f64]) {
    for k in 0..l1 {
        ch[2 * k * ido] = cc[k * ido] + cc[(k + l1) * ido];
        ch[(2 * k + 1) * ido + ido - 1] = cc[k * ido] - cc[(k + l1) * ido];
    }
    if ido < 2 {
        return;
    }
    if ido != 2 {
        for k in 0..l1 {
            let mut i = 2;
            while i < ido {
                let ic = ido - i;
                let tr2 = wa1[i - 2] * cc[i - 1 + (k + l1) * ido] + wa1[i - 1] * cc[i + (k + l1) * ido];
                let ti2 = wa1[i - 2] * cc[i + (k + l1) * ido] - wa1[i - 1] * cc[i - 1 + (k + l1) * ido];
                ch[i + 2 * k * ido] = cc[i + k * ido] + ti2;
                ch[ic + (2 * k + 1) * ido] = ti2 - cc[i + k * ido];
                ch[i - 1 + 2 * k * ido] = cc[i - 1 + k * ido] + tr2;
                ch[ic - 1 + (2 * k + 1) * ido] = cc[i - 1 + k * ido] - tr2;
                i += 2;
            }
        }
        if ido % 2 == 1 {
            return;
        }
    }
    for k in 0..l1 {
        ch[(2 * k + 1) * ido] = -cc[ido - 1 + (k + l1) * ido];
        ch[ido - 1 + 2 * k * ido] = cc[ido - 1 + k * ido];
    }
}

fn radf3(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa1: &[f64], wa2: &[f64]) {
    const TAUR: f64 = -0.5;
    #[allow(clippy::excessive_precision)]
    const TAUI: f64 = 0.86602540378443864676;
    for k in 0..l1 {
        let cr2 = cc[(k + l1) * ido] + cc[(k + 2 * l1) * ido];
        ch[3 * k * ido] = cc[k * ido] + cr2;
        ch[(3 * k + 2) * ido] = TAUI * (cc[(k + l1 * 2) * ido] - cc[(k + l1) * ido]);
        ch[ido - 1 + (3 * k + 1) * ido] = cc[k * ido] + TAUR * cr2;
    }
    if ido == 1 {
        return;
    }
    for k in 0..l1 {
        let mut i = 2;
        while i < ido {
            let ic = ido - i;
            let dr2 = wa1[i - 2] * cc[i - 1 + (k + l1) * ido] + wa1[i - 1] * cc[i + (k + l1) * ido];
            let di2 = wa1[i - 2] * cc[i + (k + l1) * ido] - wa1[i - 1] * cc[i - 1 + (k + l1) * ido];
            let dr3 =
                wa2[i - 2] * cc[i - 1 + (k + l1 * 2) * ido] + wa2[i - 1] * cc[i + (k + l1 * 2) * ido];
            let di3 =
                wa2[i - 2] * cc[i + (k + l1 * 2) * ido] - wa2[i - 1] * cc[i - 1 + (k + l1 * 2) * ido];
            let cr2 = dr2 + dr3;
            let ci2 = di2 + di3;
            ch[i - 1 + 3 * k * ido] = cc[i - 1 + k * ido] + cr2;
            ch[i + 3 * k * ido] = cc[i + k * ido] + ci2;
            let tr2 = cc[i - 1 + k * ido] + TAUR * cr2;
            let ti2 = cc[i + k * ido] + TAUR * ci2;
            let tr3 = TAUI * (di2 - di3);
            let ti3 = TAUI * (dr3 - dr2);
            ch[i - 1 + (3 * k + 2) * ido] = tr2 + tr3;
            ch[ic - 1 + (3 * k + 1) * ido] = tr2 - tr3;
            ch[i + (3 * k + 2) * ido] = ti2 + ti3;
            ch[ic + (3 * k + 1) * ido] = ti3 - ti2;
            i += 2;
        }
    }
}

fn radf4(ido: usize, l1: usize, cc: &[f64], ch: &mut [f64], wa1: &[f64], wa2: &[f64], wa3: &[f64]) {
    // `hsqt2 = 0.70710678118654752440`, which rounds to `1/sqrt(2)`.
    const HSQT2: f64 = std::f64::consts::FRAC_1_SQRT_2;
    for k in 0..l1 {
        let tr1 = cc[(k + l1) * ido] + cc[(k + 3 * l1) * ido];
        let tr2 = cc[k * ido] + cc[(k + 2 * l1) * ido];
        ch[4 * k * ido] = tr1 + tr2;
        ch[ido - 1 + (4 * k + 3) * ido] = tr2 - tr1;
        ch[ido - 1 + (4 * k + 1) * ido] = cc[k * ido] - cc[(k + 2 * l1) * ido];
        ch[(4 * k + 2) * ido] = cc[(k + 3 * l1) * ido] - cc[(k + l1) * ido];
    }
    if ido < 2 {
        return;
    }
    if ido != 2 {
        for k in 0..l1 {
            let mut i = 2;
            while i < ido {
                let ic = ido - i;
                let cr2 = wa1[i - 2] * cc[i - 1 + (k + l1) * ido] + wa1[i - 1] * cc[i + (k + l1) * ido];
                let ci2 = wa1[i - 2] * cc[i + (k + l1) * ido] - wa1[i - 1] * cc[i - 1 + (k + l1) * ido];
                let cr3 = wa2[i - 2] * cc[i - 1 + (k + 2 * l1) * ido]
                    + wa2[i - 1] * cc[i + (k + 2 * l1) * ido];
                let ci3 = wa2[i - 2] * cc[i + (k + 2 * l1) * ido]
                    - wa2[i - 1] * cc[i - 1 + (k + 2 * l1) * ido];
                let cr4 = wa3[i - 2] * cc[i - 1 + (k + 3 * l1) * ido]
                    + wa3[i - 1] * cc[i + (k + 3 * l1) * ido];
                let ci4 = wa3[i - 2] * cc[i + (k + 3 * l1) * ido]
                    - wa3[i - 1] * cc[i - 1 + (k + 3 * l1) * ido];
                let tr1 = cr2 + cr4;
                let tr4 = cr4 - cr2;
                let ti1 = ci2 + ci4;
                let ti4 = ci2 - ci4;
                let ti2 = cc[i + k * ido] + ci3;
                let ti3 = cc[i + k * ido] - ci3;
                let tr2 = cc[i - 1 + k * ido] + cr3;
                let tr3 = cc[i - 1 + k * ido] - cr3;
                ch[i - 1 + 4 * k * ido] = tr1 + tr2;
                ch[ic - 1 + (4 * k + 3) * ido] = tr2 - tr1;
                ch[i + 4 * k * ido] = ti1 + ti2;
                ch[ic + (4 * k + 3) * ido] = ti1 - ti2;
                ch[i - 1 + (4 * k + 2) * ido] = ti4 + tr3;
                ch[ic - 1 + (4 * k + 1) * ido] = tr3 - ti4;
                ch[i + (4 * k + 2) * ido] = tr4 + ti3;
                ch[ic + (4 * k + 1) * ido] = tr4 - ti3;
                i += 2;
            }
        }
        if ido % 2 == 1 {
            return;
        }
    }
    for k in 0..l1 {
        let ti1 = -HSQT2 * (cc[ido - 1 + (k + l1) * ido] + cc[ido - 1 + (k + 3 * l1) * ido]);
        let tr1 = HSQT2 * (cc[ido - 1 + (k + l1) * ido] - cc[ido - 1 + (k + 3 * l1) * ido]);
        ch[ido - 1 + 4 * k * ido] = tr1 + cc[ido - 1 + k * ido];
        ch[ido - 1 + (4 * k + 2) * ido] = cc[ido - 1 + k * ido] - tr1;
        ch[(4 * k + 1) * ido] = ti1 - cc[ido - 1 + (k + 2 * l1) * ido];
        ch[(4 * k + 3) * ido] = ti1 + cc[ido - 1 + (k + 2 * l1) * ido];
    }
}

#[allow(clippy::too_many_arguments, clippy::excessive_precision)]
fn radf5(
    ido: usize,
    l1: usize,
    cc: &[f64],
    ch: &mut [f64],
    wa1: &[f64],
    wa2: &[f64],
    wa3: &[f64],
    wa4: &[f64],
) {
    const TR11: f64 = 0.3090169943749474241;
    const TI11: f64 = 0.95105651629515357212;
    const TR12: f64 = -0.8090169943749474241;
    const TI12: f64 = 0.58778525229247312917;
    for k in 0..l1 {
        let cr2 = cc[(k + 4 * l1) * ido] + cc[(k + l1) * ido];
        let ci5 = cc[(k + 4 * l1) * ido] - cc[(k + l1) * ido];
        let cr3 = cc[(k + 3 * l1) * ido] + cc[(k + 2 * l1) * ido];
        let ci4 = cc[(k + 3 * l1) * ido] - cc[(k + 2 * l1) * ido];
        ch[5 * k * ido] = cc[k * ido] + cr2 + cr3;
        ch[ido - 1 + (5 * k + 1) * ido] = cc[k * ido] + TR11 * cr2 + TR12 * cr3;
        ch[(5 * k + 2) * ido] = TI11 * ci5 + TI12 * ci4;
        ch[ido - 1 + (5 * k + 3) * ido] = cc[k * ido] + TR12 * cr2 + TR11 * cr3;
        ch[(5 * k + 4) * ido] = TI12 * ci5 - TI11 * ci4;
    }
    if ido == 1 {
        return;
    }
    for k in 0..l1 {
        let mut i = 2;
        while i < ido {
            let ic = ido - i;
            let dr2 = wa1[i - 2] * cc[i - 1 + (k + l1) * ido] + wa1[i - 1] * cc[i + (k + l1) * ido];
            let di2 = wa1[i - 2] * cc[i + (k + l1) * ido] - wa1[i - 1] * cc[i - 1 + (k + l1) * ido];
            let dr3 =
                wa2[i - 2] * cc[i - 1 + (k + 2 * l1) * ido] + wa2[i - 1] * cc[i + (k + 2 * l1) * ido];
            let di3 =
                wa2[i - 2] * cc[i + (k + 2 * l1) * ido] - wa2[i - 1] * cc[i - 1 + (k + 2 * l1) * ido];
            let dr4 =
                wa3[i - 2] * cc[i - 1 + (k + 3 * l1) * ido] + wa3[i - 1] * cc[i + (k + 3 * l1) * ido];
            let di4 =
                wa3[i - 2] * cc[i + (k + 3 * l1) * ido] - wa3[i - 1] * cc[i - 1 + (k + 3 * l1) * ido];
            let dr5 =
                wa4[i - 2] * cc[i - 1 + (k + 4 * l1) * ido] + wa4[i - 1] * cc[i + (k + 4 * l1) * ido];
            let di5 =
                wa4[i - 2] * cc[i + (k + 4 * l1) * ido] - wa4[i - 1] * cc[i - 1 + (k + 4 * l1) * ido];
            let cr2 = dr2 + dr5;
            let ci5 = dr5 - dr2;
            let cr5 = di2 - di5;
            let ci2 = di2 + di5;
            let cr3 = dr3 + dr4;
            let ci4 = dr4 - dr3;
            let cr4 = di3 - di4;
            let ci3 = di3 + di4;
            ch[i - 1 + 5 * k * ido] = cc[i - 1 + k * ido] + cr2 + cr3;
            ch[i + 5 * k * ido] = cc[i + k * ido] + ci2 + ci3;
            let tr2 = cc[i - 1 + k * ido] + TR11 * cr2 + TR12 * cr3;
            let ti2 = cc[i + k * ido] + TR11 * ci2 + TR12 * ci3;
            let tr3 = cc[i - 1 + k * ido] + TR12 * cr2 + TR11 * cr3;
            let ti3 = cc[i + k * ido] + TR12 * ci2 + TR11 * ci3;
            let tr5 = TI11 * cr5 + TI12 * cr4;
            let ti5 = TI11 * ci5 + TI12 * ci4;
            let tr4 = TI12 * cr5 - TI11 * cr4;
            let ti4 = TI12 * ci5 - TI11 * ci4;
            ch[i - 1 + (5 * k + 2) * ido] = tr2 + tr5;
            ch[ic - 1 + (5 * k + 1) * ido] = tr2 - tr5;
            ch[i + (5 * k + 2) * ido] = ti2 + ti5;
            ch[ic + (5 * k + 1) * ido] = ti5 - ti2;
            ch[i - 1 + (5 * k + 4) * ido] = tr3 + tr4;
            ch[ic - 1 + (5 * k + 3) * ido] = tr3 - tr4;
            ch[i + (5 * k + 4) * ido] = ti3 + ti4;
            ch[ic + (5 * k + 3) * ido] = ti4 - ti3;
            i += 2;
        }
    }
}

/// `radfg` (generic odd radix of the real transform).
fn radfg(ido: usize, ip: usize, l1: usize, idl1: usize, cc: &mut [f64], ch: &mut [f64], wa: &[f64]) {
    let (dsp, dcp) = sincos2pi(1, ip);
    let ipph = ip.div_ceil(2);
    let nbd = (ido - 1) / 2;
    if ido != 1 {
        ch[..idl1].copy_from_slice(&cc[..idl1]);
        for j in 1..ip {
            for k in 0..l1 {
                ch[(k + j * l1) * ido] = cc[(k + j * l1) * ido];
            }
        }
        // C: `is = -ido;` then `is += ido` per `j`; `idij = is-1` then
        // `idij += 2` before each use, so the first use is `is + 1`.
        if nbd <= l1 {
            for j in 1..ip {
                let is = (j - 1) * ido;
                let mut idij = is + 1;
                let mut i = 2;
                while i < ido {
                    for k in 0..l1 {
                        ch[i - 1 + (k + j * l1) * ido] = wa[idij - 1] * cc[i - 1 + (k + j * l1) * ido]
                            + wa[idij] * cc[i + (k + j * l1) * ido];
                        ch[i + (k + j * l1) * ido] = wa[idij - 1] * cc[i + (k + j * l1) * ido]
                            - wa[idij] * cc[i - 1 + (k + j * l1) * ido];
                    }
                    idij += 2;
                    i += 2;
                }
            }
        } else {
            for j in 1..ip {
                let is = (j - 1) * ido;
                for k in 0..l1 {
                    let mut idij = is + 1;
                    let mut i = 2;
                    while i < ido {
                        ch[i - 1 + (k + j * l1) * ido] = wa[idij - 1] * cc[i - 1 + (k + j * l1) * ido]
                            + wa[idij] * cc[i + (k + j * l1) * ido];
                        ch[i + (k + j * l1) * ido] = wa[idij - 1] * cc[i + (k + j * l1) * ido]
                            - wa[idij] * cc[i - 1 + (k + j * l1) * ido];
                        idij += 2;
                        i += 2;
                    }
                }
            }
        }
        if nbd >= l1 {
            for j in 1..ipph {
                let jc = ip - j;
                for k in 0..l1 {
                    let mut i = 2;
                    while i < ido {
                        cc[i - 1 + (k + j * l1) * ido] =
                            ch[i - 1 + (k + j * l1) * ido] + ch[i - 1 + (k + jc * l1) * ido];
                        cc[i - 1 + (k + jc * l1) * ido] =
                            ch[i + (k + j * l1) * ido] - ch[i + (k + jc * l1) * ido];
                        cc[i + (k + j * l1) * ido] = ch[i + (k + j * l1) * ido] + ch[i + (k + jc * l1) * ido];
                        cc[i + (k + jc * l1) * ido] =
                            ch[i - 1 + (k + jc * l1) * ido] - ch[i - 1 + (k + j * l1) * ido];
                        i += 2;
                    }
                }
            }
        } else {
            for j in 1..ipph {
                let jc = ip - j;
                let mut i = 2;
                while i < ido {
                    for k in 0..l1 {
                        cc[i - 1 + (k + j * l1) * ido] =
                            ch[i - 1 + (k + j * l1) * ido] + ch[i - 1 + (k + jc * l1) * ido];
                        cc[i - 1 + (k + jc * l1) * ido] =
                            ch[i + (k + j * l1) * ido] - ch[i + (k + jc * l1) * ido];
                        cc[i + (k + j * l1) * ido] = ch[i + (k + j * l1) * ido] + ch[i + (k + jc * l1) * ido];
                        cc[i + (k + jc * l1) * ido] =
                            ch[i - 1 + (k + jc * l1) * ido] - ch[i - 1 + (k + j * l1) * ido];
                    }
                    i += 2;
                }
            }
        }
    } else {
        cc[..idl1].copy_from_slice(&ch[..idl1]);
    }
    for j in 1..ipph {
        let jc = ip - j;
        for k in 0..l1 {
            cc[(k + j * l1) * ido] = ch[(k + j * l1) * ido] + ch[(k + jc * l1) * ido];
            cc[(k + jc * l1) * ido] = ch[(k + jc * l1) * ido] - ch[(k + j * l1) * ido];
        }
    }

    let mut ar1 = 1.0;
    let mut ai1 = 0.0;
    for l in 1..ipph {
        let lc = ip - l;
        let ar1h = dcp * ar1 - dsp * ai1;
        ai1 = dcp * ai1 + dsp * ar1;
        ar1 = ar1h;
        for ik in 0..idl1 {
            ch[ik + l * idl1] = cc[ik] + ar1 * cc[ik + idl1];
            ch[ik + lc * idl1] = ai1 * cc[ik + (ip - 1) * idl1];
        }
        let dc2 = ar1;
        let ds2 = ai1;
        let mut ar2 = ar1;
        let mut ai2 = ai1;
        for j in 2..ipph {
            let jc = ip - j;
            let ar2h = dc2 * ar2 - ds2 * ai2;
            ai2 = dc2 * ai2 + ds2 * ar2;
            ar2 = ar2h;
            for ik in 0..idl1 {
                ch[ik + l * idl1] += ar2 * cc[ik + j * idl1];
                ch[ik + lc * idl1] += ai2 * cc[ik + jc * idl1];
            }
        }
    }

    for j in 1..ipph {
        for ik in 0..idl1 {
            ch[ik] += cc[ik + j * idl1];
        }
    }

    if ido >= l1 {
        for k in 0..l1 {
            for i in 0..ido {
                cc[i + k * ip * ido] = ch[i + k * ido];
            }
        }
    } else {
        for i in 0..ido {
            for k in 0..l1 {
                cc[i + k * ip * ido] = ch[i + k * ido];
            }
        }
    }
    for j in 1..ipph {
        let jc = ip - j;
        let j2 = 2 * j;
        for k in 0..l1 {
            cc[ido - 1 + (j2 - 1 + k * ip) * ido] = ch[(k + j * l1) * ido];
            cc[(j2 + k * ip) * ido] = ch[(k + jc * l1) * ido];
        }
    }
    if ido == 1 {
        return;
    }
    if nbd >= l1 {
        for j in 1..ipph {
            let jc = ip - j;
            let j2 = 2 * j;
            for k in 0..l1 {
                let mut i = 2;
                while i < ido {
                    let ic = ido - i;
                    cc[i - 1 + (j2 + k * ip) * ido] =
                        ch[i - 1 + (k + j * l1) * ido] + ch[i - 1 + (k + jc * l1) * ido];
                    cc[ic - 1 + (j2 - 1 + k * ip) * ido] =
                        ch[i - 1 + (k + j * l1) * ido] - ch[i - 1 + (k + jc * l1) * ido];
                    cc[i + (j2 + k * ip) * ido] = ch[i + (k + j * l1) * ido] + ch[i + (k + jc * l1) * ido];
                    cc[ic + (j2 - 1 + k * ip) * ido] =
                        ch[i + (k + jc * l1) * ido] - ch[i + (k + j * l1) * ido];
                    i += 2;
                }
            }
        }
    } else {
        for j in 1..ipph {
            let jc = ip - j;
            let j2 = 2 * j;
            let mut i = 2;
            while i < ido {
                let ic = ido - i;
                for k in 0..l1 {
                    cc[i - 1 + (j2 + k * ip) * ido] =
                        ch[i - 1 + (k + j * l1) * ido] + ch[i - 1 + (k + jc * l1) * ido];
                    cc[ic - 1 + (j2 - 1 + k * ip) * ido] =
                        ch[i - 1 + (k + j * l1) * ido] - ch[i - 1 + (k + jc * l1) * ido];
                    cc[i + (j2 + k * ip) * ido] = ch[i + (k + j * l1) * ido] + ch[i + (k + jc * l1) * ido];
                    cc[ic + (j2 - 1 + k * ip) * ido] =
                        ch[i + (k + jc * l1) * ido] - ch[i + (k + j * l1) * ido];
                }
                i += 2;
            }
        }
    }
}
