//! Python 3.8 text primitives as used by `klippy/`.
//!
//! The crate covers three areas. Each module documents the public API and the
//! exact CPython behaviour it reproduces.
//!
//! # `configparser`
//!
//! [`configparser::RawConfigParser`] reproduces the parts of
//! `configparser.RawConfigParser` that `klippy/configfile.py` exercises and
//! [`configparser::ConfigParser`] reproduces the interpolating variant that
//! `klippy/extras/save_variables.py` exercises. Byte-identical section and
//! option regexes, delimiters `:` and `=`, comment prefixes `#` and `;`,
//! inline prefixes `(';' , '#')` under klippy's `strict=False`, continuation
//! lines, empty lines inside values, `optionxform`, `DEFAULTSECT`, the
//! `write()` format and the exception classes and messages live in the
//! [`configparser::Error`] enum whose `Display` equals CPython `str(exc)`.
//!
//! # `shlex`
//!
//! [`shlex::Shlex`] reproduces `shlex.shlex(s, posix=True)` with
//! `whitespace_split=True` and `commenters = '#;'` as set by
//! `klippy/gcode.py`. [`shlex::split`] and [`shlex::quote`] are provided for
//! completeness. The `ValueError` texts `No closing quotation` and
//! `No escaped character` match CPython.
//!
//! # `pyre`
//!
//! [`pyre`] provides the `re` patterns that occur in `klippy/`. The list is
//! fixed and documented in [`pyre::PATTERNS`]. Each exact pattern string is
//! accepted by [`pyre::compile`]; any other pattern is rejected with
//! [`pyre::ReError`]. The supported operations are `sub`, `split`,
//! `match`, `findall` and `search` with Python semantics for the inputs
//! klippy feeds them (`.`, `\w`, `\d`, `\s`, `$`, `^`, greedy and lazy
//! quantifiers, capturing groups, `re.split` keeping delimiters).
//!
//! Ordering follows CPython `dict` insertion order. The parser stores
//! sections and options in `Vec`, so no hashed map is used.

pub mod configparser;
pub mod pyre;
pub mod shlex;
