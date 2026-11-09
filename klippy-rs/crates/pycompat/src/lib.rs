//! CPython 3.8 value semantics for the Klipper port.
//!
//! This crate reproduces the exact user-visible behaviour of CPython 3.8.2
//! (the reference runtime on the printer) for the numeric, formatting and
//! dynamic-value operations the ported `klippy/` code performs. Results of
//! every public function match CPython byte for byte on the values klippy
//! renders (temperatures, positions, clock estimates, config errors) and on
//! the differential test corpus (subnormals, powers of ten, halfway cases,
//! huge ints, negative zero).
//!
//! # Modules
//!
//! - [`int`]: arbitrary-precision `PyInt` reproducing CPython `int`
//!   semantics: floor division and modulo with divisor-sign remainder
//!   (`-7 // 2 == -4`, `-7 % 2 == 1`), `divmod`, exact non-negative power,
//!   sign, CPython `hash` (`Objects/longobject.c:long_hash`, including the
//!   `-1 -> -2` sentinel mapping).
//! - [`libm`]: system libm through `extern "C"` (`exp`, `log`, `log10`, `pow`,
//!   `sin`, `cos`, `sincos`, `tan`, `atan`, `atan2`, `asin`, `acos`, `sqrt`,
//!   `fabs`, `floor`, `ceil`, `fmod`, `hypot`) so transcendental results match glibc
//!   2.29 bit for bit. Every wrapper applies `std::hint::black_box` to keep
//!   the C call at the Python evaluation point. `kos` re-exports this
//!   module; `pycompat` also calls it directly for float `**`, `%f`/`%e`
//!   scaling and `round` helpers. `sqrt` uses FFI (it is correctly rounded
//!   in glibc, matching hardware `sqrtd`).
//! - [`text`]: float text, `%` formatting, `str.format`/`format()`
//!   mini-language, rounding and numeric parsing.
//!   - `float_repr`/`float_str`: shortest round-trip decimal with CPython
//!     exponent rules (`1e+16`, `1e-05`, `inf`, `nan`, `-0.0`); `str` and
//!     `repr` share this path in 3.8 (`Objects/floatobject.c`).
//!   - `percent_format`/`percent_format_map`: `str.__mod__` with
//!     `%s %r %d %i %u %f %F %e %E %g %G %x %X %o %c %%`, flags
//!     `# 0 - space +`, width/precision (`*`), mapping keys `%(name)s`.
//!   - `format_value`/`str_format`: `str.format` mini-language
//!     `[[fill]align][sign][#][0][width][grouping][.precision][type]` with
//!     types `s e E f F g G % b c d o x X n`, nested spec fields and
//!     `{}`/`{0}`/`{name}`/`{{`/`}}` templates used by klippy and Jinja.
//!   - `round_py`/`round_py_nd`: correctly rounded half-even
//!     (`round(2.5) == 2`, `float_round` with `ndigits`).
//!   - `trunc_float_to_bigint`, `float_from_string`, `int_from_string`,
//!     `as_float`, `format_f_fixed`: `int(float)` truncation, `float(str)`
//!     and `int(str[, base])` parsing with CPython syntax (whitespace,
//!     underscores, signs, `inf`/`nan` spellings) and exact `ValueError`
//!     texts (`could not convert string to float: 'x'`,
//!     `invalid literal for int() with base 10: 'x'`).
//!   - `float_floor_div`, `float_mod`, `float_true_div`, `float_pow`,
//!     `cmp_bigint_float`, `py_sum_floats`: floor division/modulo with
//!     `-0.0`/`inf`/`nan` rules and CPython `ZeroDivisionError` texts,
//!     exact int/float comparison, and plain left-to-right `sum` of floats
//!     as in 3.8 (Neumaier arrived in 3.12).
//!   - `str_repr`, `bytes_repr`, `float_py_hash`, `PyError`, `PctValue`:
//!     `repr` quoting, `float.__hash__` and the exception carrier
//!     (`kind`/`msg` mirroring `str(e)`).
//! - [`value`]: dynamic `PyValue` (`None`, `bool`, `int`, `float`, `str`,
//!   `bytes`, `list`, `tuple`, insertion-ordered `dict`, opaque `Host`
//!   handle). `repr`/`str` match CPython (string quoting, nested
//!   containers); `py_equal`/`py_hash`/`py_compare`/`py_rich_compare`
//!   implement the numeric tower so `1 == 1.0 == True` are equal and hash
//!   identically (dict-key collision), truthiness follows
//!   `PyObject_IsTrue`, and cross-type ordering raises
//!   `TypeError: '<' not supported between instances of 'X' and 'Y'`.
//! - [`json`]: `json.dumps`/`json.loads` reproducing CPython `json`
//!   (`json/encoder.py`, `json/decoder.py`).
//!   - `dumps`/`dumps_opts`: `separators` (defaults `(', ', ': ')`),
//!     `ensure_ascii` (default `True`), `allow_nan` (default `True`),
//!     float repr, `Infinity`/`-Infinity`/`NaN`, key coercion
//!     (str/int/float/bool/None keys to strings, other keys `TypeError`).
//!   - `loads`: arbitrary-precision ints, float parsing, string escapes
//!     including `\u` with surrogate pairs, `null`/`true`/`false`, and
//!     CPython `JSONDecodeError` texts with `line/col/char`.
//! - [`numpy`]: the float64 kernels of NumPy 1.16.4 (the oracle venv's
//!   version) that klippy's NumPy users call, in NumPy's operation order:
//!   pairwise and buffered `add.reduce`/`mean`, `maximum`/`minimum`,
//!   `arange`, `i0`/`kaiser`, `interp`, `fftpack_lite` `rfft`/`fft` and
//!   `rfftfreq`.
//!
//! The crate never panics on Python error paths; every fallible operation
//! returns `text::PyError` whose `display()` is the CPython exception text.
//! All float-to-int conversions go through the exact helpers here (no bare
//! `as` casts), and no `HashMap`-based ordering is used: dicts are
//! `indexmap::IndexMap` in insertion order.

pub mod int;
pub mod json;
pub mod libm;
pub mod numpy;
pub mod text;
pub mod value;

pub use int::PyInt;
pub use text::{PctValue, PyError};
pub use value::PyValue;
