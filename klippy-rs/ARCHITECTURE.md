# klippy-rs architecture contract

This file is the frozen contract every porting and validation agent works against. Changing it requires the integration owner; agents report conflicts instead of working around them.

## Goal

`klippy-rs` is a Rust port of the Klipper host (`klippy/`, including `klippy/chelper/`) at the commit this branch forked from. Parity is byte-for-byte: same config, G-code, simulated MCUs and virtual clock produce identical MCU byte streams, G-code responses, errors, webhooks traffic, written files, exit codes and log text, after the frozen normalizer table in `parity/normalize/` is applied to both sides. Python bugs and C undefined behaviour are reproduced as the reference binaries behave on each target.

## Layout

| Path | Owner | Content |
|---|---|---|
| `klippy-rs/Cargo.toml` | integration | Workspace; members are `crates/*` |
| `klippy-rs/crates/kos` | foundation | OS layer: time, sleep, poll/epoll, fds, ptys, sockets, threads, mutex, condvar, libm |
| `klippy-rs/crates/pycompat` | foundation | Python 3.8 value semantics: numbers, formatting, `repr`, `PyValue`, JSON; NumPy 1.16.4 float64 kernels (`pycompat::numpy`: sums, `kaiser`, `interp`, `arange`, `fftpack_lite` FFT) |
| `klippy-rs/crates/pytext` | foundation | `configparser`, `shlex`, Python `re` subset as used by klippy |
| `klippy-rs/crates/reactor` | foundation | Port of `klippy/reactor.py` with greenlet semantics |
| `klippy-rs/crates/<unit crate>` | horde | One crate per ported area, created from `parity/inventory/units.json` |
| `parity/` | foundation | Oracles, virtual clock, MCU simulator, recorder, harness, scenarios, inventory |
| `tests/fixtures/` | foundation | Dictionaries, pad snapshot, recordings, goldens |

## Platform rules (checked by `klippy-rs/clippy.toml`)

1. Every clock read, sleep, wait, poll, thread start and lock goes through `kos`. `kos` calls glibc symbols (`clock_gettime`, `nanosleep`, `poll`, `epoll_wait`, `pthread_create`, `pthread_mutex_*`, `pthread_cond_*`, `sem_*`, `read`, `write`) so the virtual-clock shim in `parity/vclock` can interpose them. `std::time::{Instant, SystemTime}`, `std::thread`, `std::sync::{Mutex, RwLock, Condvar, Barrier, mpsc}`, `parking_lot`, async runtimes and raw `futex` syscalls are forbidden outside `kos`.
2. `std::collections::{HashMap, HashSet}` are forbidden. Use `indexmap` where Python uses `dict`/`set` insertion order, `BTreeMap` where the Python code sorts.
3. Floating point: `f64` only where Python or C uses double; `f32` only where the C uses `float`. No `mul_add`, no reassociation, evaluate in the exact source order. Transcendentals (`exp`, `log`, `pow`, `sin`, `cos`, `atan2`, ...) call the system libm through `kos::libm`, never Rust intrinsics, so results match glibc 2.29 bit for bit. `sqrt` may use `f64::sqrt` (correctly rounded).
4. Float to integer conversions use the explicit helpers in `pycompat` (Python `int()`, `round()`) or `kos::cconv` (C casts, including the reference behaviour of out-of-range and negative casts per target). Bare `as` casts from float are forbidden.
5. C integer arithmetic that can overflow in the reference uses `wrapping_*` with the reference width.
6. No panics on paths the Python handles as errors: the Python exception text is part of the contract.

## Python semantics

All user-visible text is produced through `pycompat` so that `str(float)`, `repr`, `%` formatting, `str.format`, `round`, `int()`, `float()`, floor division, modulo and JSON encoding match CPython 3.8.2. Dynamic values that cross module boundaries as Python objects (status dicts, event payloads, template contexts, webhooks params) are `pycompat::PyValue`.

## Concurrency model

- `reactor` reproduces `klippy/reactor.py` exactly: one dispatch loop, timers, fd callbacks, `pause`, completions, mutexes, async callbacks via self-pipe. Greenlets are OS threads created by `kos` with strict baton passing: exactly one reactor greenlet runs at a time and switches happen only where the Python switches. Timer order, the copy-on-write timer list, `update_timer` semantics and greenlet caching follow the Python line by line.
- Threads that exist in the reference (serial reader, logging, one step-generation thread per syncemitter, serialqueue background thread) exist in the port with the same blocking points, so the virtual-clock scheduler sees the same thread structure.

## Units and recordings

- `parity/inventory/units.json` lists every unit: id, source file and line span, kind (`py`/`c`), crate and module in the port, dependencies, verification class (`sim`, `ke-only`, `unverifiable`), and whether it is an indivisible block.
- A unit owns exactly its target module files. It must not edit other units' files or this contract.
- Recordings for unit `<id>` live in `tests/fixtures/recordings/<id>/` as JSONL (format in `parity/record/FORMAT.md`). The unit's test target replays them; a unit is done when every recording replays byte-identical on x86_64.
- Integration units are verified by scenarios through `parity/harness` once their layer is assembled.

## Targets

- `x86_64-unknown-linux-gnu`, run inside the x86 oracle container (glibc 2.29).
- MIPS32 little-endian NaN2008 FP64 for the Ender-3 V3 KE (glibc 2.29, `ld-linux-mipsn8.so.1`), built from the pinned nightly with `-Zbuild-std` and the custom target in `klippy-rs/targets/`, run under qemu with the pad snapshot.
