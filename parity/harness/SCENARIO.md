# Harness scenario directory format

This is the frozen scenario contract for `parity/harness/run.py`. Every scenario is one directory containing `scenario.json` plus the data files it names. All relative paths in `scenario.json` resolve against the scenario directory first, then against the repository root (the checkout that contains `klippy/`). `run.py` always runs the scenario in a private temp root; the scenario directory is never modified except by the legacy generator.

## Top-level fields

```json
{
  "name": "legacy-commands-001-example-cartesian",
  "mode": "batch",
  "config": { "main": "config/example-cartesian.cfg", "files": [], "mounts": [] },
  "mcus": [ { "name": "mcu", "dict": "tests/fixtures/dict/atmega2560.dict", "script": "mcusim/mcu.json" } ],
  "gcode": { "source": "file", "path": "input.gcode" },
  "webhooks": { "script": [], "expect": {} },
  "expect": { "exit": 0, "signals": [] },
  "collect": [],
  "run": { "log": "file", "verbose": false, "debug_motion": true }
}
```

### `name` (required)

Free text. `run.py` records it in `manifest.json`. The directory name is the stable identity; `name` is informational.

### `mode` (required)

- `batch`: `klippy.py <cfg> -i <gcode> -o <out> -d <dict>...` exactly like `scripts/test_klippy.py`. No live MCUs, no vclockd required. MCU byte streams are the `-o` files. Webhooks stay disabled in the host because `webhooks.ServerSocket` returns early when `debuginput` is set (`klippy/webhooks.py`); `gcode` output reaches only the log because `GCodeIO` registers no output handler in file mode (`klippy/gcode.py`). `run.py` still writes the artifact files for those channels (empty transcripts plus a note) so `diff.py` compares the same names in both modes.
- `live`: one simulated MCU per `mcus[]` entry over ptys under `vclockd`, plus the host. The host is launched with `-I` (input tty), `-a` (api socket), `-l` (log file) and never with `-i/-o/-d`, so `webhooks` and `GCodeIO` stay live. Each MCU config segment has its `serial:` option rewritten to the staged pty path of that MCU. Each MCU sim is launched as `python3 parity/mcusim/mcusim.py --dict DICT --name NAME --pty PATH --script SCRIPT.json --trace TRACE` (see `parity/mcusim/SCRIPT.md` and the mcusim task). `run.py` refuses live mode unless `--vclockd` and `--mcusim` are given.

### `config` (required)

- `main`: the printer config path. In batch this is passed as the positional config argument; in live it is staged and its `serial:` lines rewritten.
- `files`: extra files to stage next to `main` (for relative `[include]`). Each entry is a path. Staged copies preserve the basename layout under the staged main. Use this for configs that `[include]` siblings; the legacy corpus has none, but SAVE_CONFIG scenarios need the main file writable in staging.
- `mounts`: absolute include remapping table. Each entry is `{ "prefix": "/usr/data", "dir": "mnt/usr/data" }` meaning scenario-relative subtree `mnt/usr/data` must appear at absolute location `/usr/data` inside the run. `klippy/configfile.py` `_resolve_include` does `os.path.join(dirname, include_spec)`, so an absolute `include_spec` escapes any relative staging. `run.py` therefore stages each mount into the private temp root and rewrites the prefix inside staged config text to the staged path, recording every rewrite in `manifest.json` under `mount_rewrites`. Both implementations receive the same rewritten staging, so parity is preserved. If the harness runs as root it also tries to link the real prefix (so unrewritten firmware paths still resolve) but never requires it. Mount `gcode`, `collect` and `dict`/`script` resolution must not contain a mount prefix; they are resolved before staging.
- Files the scenario needs under a mount prefix (for example `mainsail.cfg`) are supplied in the scenario directory, not read from the oracle root.

### `mcus` (required, at least one)

List of MCUs in load order. Each entry:

- `name`: MCU name as in `[mcu <name>]`; `mcu` is the primary. The name decides the batch `-d` flag form and the `-o` file suffix, matching `klippy/mcu.py` `_attach_file`: `mcu` writes to `<outbase>`, `extboard` writes to `<outbase>-extboard`. `-d` flags follow `klippy/klippy.py` `arg_dictionary`: primary is `-d <dict>`, named is `-d <name>=<dict>`.
- `dict`: dictionary path (raw JSON, not zlib; batch `serialhdl.connect_file` calls `process_identify(dictionary, decompress=False)`). Legacy scenarios reference `tests/fixtures/dict/<name>` with the exact names the `.test` files use; see the generator note in `parity/scenarios/generate_legacy.py`.
- `script`: mcusim JSON script path. Required in `live`, ignored in `batch`. May be omitted in legacy batch scenarios.
- `kind` (live only): transport. `uart` (default, `pty` path is the serial device) or `pipe` (pty path is passed as the pipe path; see `serialhdl.connect_pipe`). `baud` defaults to the config value; `run.py` does not change it.
- `pty` (live only): path of the pty symlink `mcusim` creates, relative to the private root. If omitted, `mcusim/<name>.pty` is used.

### `gcode` (required)

- `{ "source": "file", "path": "input.gcode" }`: batch passes this as `-i`. Live may also use file input (no `-I` needed) but then responses stay in the log; timed pty injection is preferred in live.
- `{ "source": "inject", "steps": [ { "at": 0.0, "data": "G28\n" } ] }`: live only. `at` is virtual seconds since process start. The harness injector is a vclock participant that writes `data` to the `-I` pty at each step and records everything the host writes back (the `ok` / `//` lines from `GCodeIO._respond_raw`) with its virtual timestamp into `gcode_responses.jsonl`. Steps must be sorted by `at`; ties keep file order.
- `{ "source": "none" }`: no G-code input. Batch writes an empty `-i` file so the host still reaches EOF and exits; live skips the injector.

### `webhooks` (optional)

- `script`: list of timed requests `{ "at": 0.0, "method": "info", "params": {}, "id": 1 }`. Sent as one JSON object plus `\x03` over the `-a` unix socket, exactly like Moonraker (`webhooks.ClientConnection.process_received` splits on `b'\x03'`). Each request runs under vclock so `at` is virtual time. Replies are appended to `webhooks_transcript.jsonl` as `{ "at": <virtual send time>, "request": {...}, "response": {...} }`.
- `expect`: object to compare against each response by `id`. A key `"<id>"` maps to a required subset of the response object compared after normalization. An absent key means the response must exist. The harness failing an expectation still records the run; `diff.py` compares transcripts byte for byte after normalization, and `run.py` adds an `expect_mismatch` note to `manifest.json` when a subset differs. Subscriptions (delta channels) are not a separate field: they are ordinary `objects/subscribe` requests in `script`, and the following `objects/query` replies are the evidence. `list_endpoints` and `info` cover the probe sequence the stock display and Moonraker use (see `parity/maps/map-parity.md` 7.5).

### `expect` (optional)

- `exit`: expected host exit code as `subprocess.returncode`, where `sys.exit(-1)` in `klippy.py` is `255`. Omitted means any code is recorded but not asserted. `SHOULD_FAIL` legacy scenarios set `255`.
- `signals`: list of `{ "at": <virtual seconds>, "signal": "TERM" }`. The harness sends the named signal to the host pid at virtual time `at` using a timer participant. Named signals are the `signal` module names (`TERM`, `INT`, `HUP`, `KILL`, `USR1`).

### `collect` (optional)

List of paths or globs to copy from the private root into `<out>/collected/`, preserving the path relative to the staged config directory. Always include the SAVE_CONFIG evidence for restart scenarios: the rewritten main config, `<name>_autosave.cfg` (a transient `temp_name` file in `configfile.py` `cmd_SAVE_CONFIG`), and `<name>-YYYYMMDD_HHMMSS.cfg` backups. `run.py` expands `*` in each entry and, for the autosave triplet, also copies any `<name>-????????_??????.cfg` backup even when not listed, recording the original name in `manifest.json` and normalizing the timestamped name in the collected copy (wall time leaks there because `cmd_SAVE_CONFIG` uses `time.strftime`; under `vclockd` `CLOCK_REALTIME` is `epoch + monotonic` so the name is deterministic only when `vclockd` is used). CSV exports and other files produced by extras are collected the same way. A listed path that does not exist is recorded as missing in `manifest.json`, not treated as a run failure.

### `run` (optional)

- `log`: `file` (default) passes `-l <root>/klippy.log` and collects it as `log`; `stderr` captures host stdout/stderr to `log` instead and passes no `-l` (the pad service uses this form with `tee`). Both forms are parity inputs: `queuelogger.setup_bg_logging` changes only the sink, not the `INFO:root:` record format.
- `verbose`: passes `-v` (DEBUG level) when true.
- `debug_motion`: true (default) means `[DEBUG-motion]` lines are compared like any other log line; false means `diff.py` masks whole `[DEBUG-motion]` records on both sides before comparing. The host always emits these lines on shutdown/error (`klippy/klippy.py` `_debug_host_state`); the flag only changes the comparison. `run.py` records the flag in `manifest.json` so a mismatch of flags is visible without reading the scenario.
- `api_server` (live only): socket filename relative to the private root. Default `klippy.sock`.
- `input_tty` (live only): pty symlink relative to the private root. Default `printer`. This is the `/tmp/printer`-style path from `klippy.py` `-I`.

## Output artifact layout (`run.py --out DIR`)

`run.py` creates `DIR` fresh (refuses a non-empty dir unless `--force`) and writes:

```
manifest.json            versions, command lines, exit code, scenario name/mode/impl,
                         mount rewrites, collected-missing list, vclock events summary
log                      klippy log (or stderr capture when run.log is stderr)
stdout.txt stderr.txt    always captured separately so log == stdout|stderr can be checked
mcu/<name>.bin           raw MCU stream (batch: -o file; live: mcusim trace transport capture, see run.py)
mcu/<name>.txt           decoded text via klippy/parsedump.py (one line per message)
mcu/<name>.dict          copy of the dictionary used (for diff.py byte-offset mapping)
gcode_responses.jsonl    live pty transcript; batch: responses extracted from log, same line shape
webhooks_transcript.jsonl live UDS transcript; batch: empty with a header note
collected/...            files named in collect
vclock_events.jsonl      vclockd stdout (live, or batch when --vclockd is given)
coverage/                only with --coverage (merged python + gcov reports)
```

`diff.py` compares every file under these names except `manifest.json` (informational, versions differ by impl by design) and reports exit-code equality separately. `stability.py` compares runs of one impl by the same rules.

## Resolution and determinism notes

- `dict`, `script`, `config.main`, `config.files`, `gcode.path` and `collect` entries resolve scenario-dir first, then repo root. A path that resolves to neither is a scenario error: `run.py` exits `2` without starting the host.
- Config text rewrites (mount prefixes, live `serial:` lines) are applied to staged copies only, never to the scenario directory. Every rewritten file is listed in `manifest.json` with before/after snippets so a manual rerun can reproduce the host input exactly.
- Without `vclockd`, batch runs are still the legacy corpus path: `clocksync.connect_file` fixes the MCU clock rate at `1e12` and `MCU.estimated_print_time` is stubbed, so the MCU stream does not depend on wall time, but timers, `SAVE_CONFIG` backup names and `curtime` do. Always pass `--vclockd` for live runs and for any run whose scenario uses `inject` steps, `signals` or wall-time-dependent `collect` names.
