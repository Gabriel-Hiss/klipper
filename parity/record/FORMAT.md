# Per-unit recording format

Recordings prove a ported unit byte-identical without the whole system.
`parity/harness/run.py --impl python` runs a scenario from the frozen
reference (Python `klippy/` plus the C helper) with the recorders
enabled; one file per unit and scenario lands at:

    tests/fixtures/recordings/<unit_id>/<scenario>.jsonl

`scenario` is the scenario directory basename. One JSON object per line,
UTF-8, one call per line, in call order. The Rust crate
`klippy-rs/crates/recording` reads these files and replays them.

## Record fields

| Field | Type | Meaning |
|---|---|---|
| `v` | int | format version (currently 1) |
| `unit` | string | unit id, e.g. `msgproto` |
| `scenario` | string | scenario name |
| `seq` | int | 1-based per-file counter in call order |
| `function` | string | qualified name, or the C function name |
| `args` | array | arguments, in signature order (`self` first for methods) |
| `ret` | value | return value (absent when the call raised) |
| `exc` | object | `{"type": str, "message": str}` (absent when the call returned) |
| `side_effects` | object | observed side effects (below) |
| `object_id` | string or null | stable id of the receiver for methods on stateful units |
| `kind` | string | `python`, `c_boundary`, or `c_internal` |

`ret` and `exc` are mutually exclusive. `side_effects` always has the
three keys below, each an array (empty when nothing was observed):

| Key | Meaning |
|---|---|
| `mutated_args` | `[[index, value_after], ...]` for arguments the call mutated, such as C out-parameter buffers filled by `*_extract_old` |
| `mcu_messages` | raw bytes of emitted MCU messages, lowercase hex, one message per entry |
| `log_lines` | log records emitted during the call, oldest first |

## Value encoding

Values are encoded recursively. Bare JSON values carry only the types
that cannot be confused:

| Python/C value | Encoding |
|---|---|
| `None` | `null` |
| `bool` | `true` / `false` |
| `int` in `i64` range | JSON number |
| other `int` | `{"$int": "digits"}` |
| `float` | `{"$float": {"hex": "0x1.999999999999ap-4", "repr": "0.4"}}` (lossless: hex float plus repr) |
| `bytes`, `bytearray`, MCU message | `{"$bytes": "deadbeef"}` (lowercase hex) |
| `str` | JSON string |
| `list` | JSON array |
| `tuple` | `{"$tuple": [...]}` |
| `dict` | `{"$dict": [[key, value], ...]}` in insertion order |
| C struct | `{"$struct": {"type": "struct pull_move", "fields": [[name, value], ...]}}` (field map, in declaration order) |
| opaque C pointer | `{"$opaque": {"type": "struct trapq *", "addr": "0x..."}}` |
| host object | `{"$oid": "Thermistor#3"}` (stable object identity) |

Notes:

- Floats are never emitted bare, so a bare JSON number is always an
  integer and a bare JSON array is always a list. Both spellings of a
  float must match at replay time; one alone could hide a one-ULP
  difference. `-0.0` and `0.0` differ by hex float spelling (`-0x0.0p+0`
  vs `0x0.0p+0`) and count as different.
- NaN is encoded `{"$float": {"hex": "nan", "repr": "nan"}}`,
  infinities the same with `inf` / `-inf`.
- Dict keys are encoded with the same rules (a float key is a `$float`
  object, a tuple key is a `$tuple`).
- C structs are field maps: replay compares the type name and every
  field, in order. Padding bytes are not recorded; a struct with
  padding replays equal if the fields agree.
- Stable object ids are assigned on first encounter as `<ClassName>#<n>`
  with `n` counting up from 1 per process. The recorder keeps the
  objects alive until the process exits, so ids are never reused.
  Replay compares ids by string equality.
- The recorder keeps strong references to every object it ids, so an id
  is unique within one recording file.

## Kinds

- `python`: traced with `sys.setprofile` by the Python unit recorder
  (`parity/record/unit_recorder.py`). Only functions configured for the
  unit are traced; see `parity/record/units.py`.
- `c_boundary`: Python-to-C calls, logged by the wrapper around the
  cffi `lib` from `klippy/chelper/__init__.py get_ffi()`
  (`parity/record/cffi_recorder.py`). Every call and return is logged,
  including `*_extract_old` out-parameters snapshotted before and
  after the call.
- `c_internal`: C-to-C calls, captured by the recording `c_helper.so`
  built from the unchanged C sources with `-Wl,--wrap=`
  interposition (`parity/record/c_internal/README.md`). These lines are
  written by the C shim with the same fields and value encoding.

## Replay

A unit test loads its recordings, drives the port through the same
calls, and asserts the first divergence with full context (unit,
scenario, `seq`, function, field path, expected and actual values).
The Rust helpers in `klippy-rs/crates/recording` implement this. A
unit is done when every recording replays byte-identical on x86_64.
