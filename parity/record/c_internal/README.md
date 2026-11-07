# C internal recorder

Captures the C-to-C edges between C units that a Python-side tracer
cannot see. Build with `build.sh`; pass the result with
`record.py --c-helper PATH`.

## Captured edges

| Callee | Function | Caller site |
|---|---|---|
| `stepcompress` | `stepcompress_append` | `itersolve.c itersolve_gen_steps_range` on each generated step |
| `stepcompress` | `stepcompress_commit` | `itersolve.c itersolve_gen_steps_range` (return value ignored upstream) — wrapped via `stepcompress_append` only; see note below |
| `stepcompress` | `stepcompress_flush` | `steppersync.c se_generate_steps` after `itersolve_generate_steps` |
| `stepcompress` | `stepcompress_history_expire` | `steppersync.c se_generate_steps` after the flush |
| `itersolve` | `itersolve_generate_steps` | `steppersync.c se_generate_steps` on the background thread |
| `serialqueue` | `serialqueue_send_batch` | `steppersync.c steppersync_flush` (the serialqueue inputs) |

`stepcompress_commit` is invoked directly by the solver between
`stepcompress_append` calls, but the wrapped set interposes the five
symbols in `build.sh`. A line is written per call with the same field
shape as `FORMAT.md`: the unit is the callee, the arguments are the
exact C arguments (opaque pointers as `$opaque`, `double`s as `$float`
with hex float plus `%.17g` repr), and the return is the exact C
return value.

## How it works

`build.sh` compiles the unchanged C sources from `klippy/chelper/`
plus `recorder.c` with `-Wl,--wrap=<symbol>` for each captured
symbol. Calls from the compiled units are routed to `__wrap_<symbol>`,
which logs one line and then calls `__real_<symbol>` — the original
function — with the same arguments. Nothing in `klippy/` changes, and
the wrapped call itself executes the original code path unmodified.
No input buffer is written by the shims; out-parameters are handled
by the C boundary recorder, which snapshots `*_extract_old` buffers
before and after.

## Why recording is bit-identical to the normal build on x86_64

- Same compiler flags except LTO. The normal build uses
  `-Wall -g -O2 -shared -fPIC -flto -fwhole-program
  -fno-use-linker-plugin -mfpmath=sse -msse2`; the recording build
  drops only `-flto -fwhole-program -fno-use-linker-plugin`. Those
  flags affect how much the linker can inline across translation
  units, not the arithmetic of any function body: each function is
  compiled with the same `-O2` and the same SSE settings in both
  builds. Inlining a call across the wrap boundary never happens in
  the recording build (the wrap interposes exactly there), and in
  the normal build the call to the original function remains in
  either case — `--wrap` does not change its semantics.
- No FMA. Nothing passes `-mfma`, `-march=native`, `-ffast-math`, or
  `-ffp-contract`, so `gcc` cannot emit fused multiply-add or
  reassociate floating-point expressions. Double arithmetic is plain
  SSE2, exactly as the reference build. The recorder's own code
  (`recorder.c`) runs only on the wrapper path and never computes
  step timing.
- Argument passing is unchanged. `--wrap` renames the references to
  the wrapper and the wrapper calls the original with the identical
  argument list, so registers, stack spill layout of the callee, and
  the C ABI are untouched.
- The log write happens after the real call returns, so it cannot
  change any value the callee computed. The only timing difference
  is the extra `fprintf`, which changes no recorded value; replay
  compares recorded arguments and returns, never wall-clock duration.
- Threads are unchanged. `se_generate_steps` still runs on the
  syncemitter background thread; the wrapper adds no locks around
  the call, only a best-effort `fopen`/`fprintf` on the record
  path, so step-compression state transitions are identical.

One limitation: `stepcompress_commit` remains unwrapped (it is not
in the `--wrap` list because the Python boundary already covers
every `stepcompress_commit` return path through `itersolve_check_active`
discard history; adding it changes the wrap set if a port needs it).
