//! Per-unit recordings: reader plus replay and assertion helpers.
//!
//! Recordings live under `tests/fixtures/recordings/<unit_id>/` as JSONL,
//! one file per (unit, scenario). The format is frozen in
//! `parity/record/FORMAT.md`: one JSON object per line, one call per line,
//! with fields `v`, `unit`, `scenario`, `seq`, `function`, `args`, `ret` or
//! `exc`, `side_effects`, `object_id`, and `kind`.
//!
//! A unit test loads its recordings with [`reader`], drives the ported unit
//! through the same calls, and asserts each actual result with the helpers
//! in [`replay`]. Floats compare bit-for-bit: a recording stores the exact
//! hex float and the Python `repr`, and replay requires both to match.
//! [`hexfloat`] converts between `f64` and CPython hex-float spelling.
//!
//! There are no external dependencies. JSON parsing is a small
//! self-contained recursive-descent parser for exactly the subset the
//! recorder emits, and the value model lives in [`value`].

pub mod hexfloat;
pub mod reader;
pub mod replay;
pub mod value;

pub use hexfloat::{f64_from_hex, f64_to_hex};
pub use reader::{load_file, load_unit, parse_line, record_files};
pub use replay::{assert_equal, Divergence, ReplayCursor};
pub use value::{Exception, Record, RecordKind, SideEffects, Value};
