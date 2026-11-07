# Map of the Klippy core runtime (`klippy/*.py`)

Slice: every Python file directly in `klippy/`. `toolhead.py` and `stepper.py` appear only as interface notes.
Everything below was read from the working tree at `C:/Users/User/CLionProjects/klipper-cp` (branch `ender-3-v3-ke`, HEAD `e47b09436a7e`). Statements marked `[INFERENCE]` were not observed in code or logs.

## 0. Method, caveats, provenance

* No shell and no git CLI were available. Fork commits were identified from `.git/logs/HEAD` (subjects and full hashes) and from the `[DEBUG-motion]` markers inside the code. A commit-to-file mapping needs `git show --stat <hash>`; the mappings below are `[INFERENCE]` unless stated.
* The tree is not upstream master. Upstream master `klippy/reactor.py` has `_recent_callbacks`, `_check_fd_activity` and `_dispatch_fd_events`; this tree has none of them. Treat the checked-out files, not upstream, as the behaviour to reproduce.
* Python is 3.8.2 on the Ender (logs: `python='3.8.2 ... [GCC 7.2.0]'`, MIPS/Ingenic toolchain), 3.14 on the dev smoke run. Python 2 compatibility code in `util.py`, `configfile.py`, `webhooks.py` can be dropped.
* Useful golden data already in the repo: `printer-backups/192.168.0.24/motion-v4-*/klippy-*.log`, `stepcompress-debug/logs/**`, and `config/ender-3-v3-ke/*` (real launch line `klippy.py printer.cfg -a /tmp/klippy_uds`, no `-l`, stderr tee'd to `/usr/data/klippy-mainline.log`).

### 0.1 Fork-specific history (applies to the whole slice)

Fork commits from the reflog (`ender-3-v3-ke`, final surviving sequence). Subjects are Conventional Commits.

| Hash (full) | Subject | Touches this slice? |
|---|---|---|
| `7777d10424b40e51dde26aa15cf43655a4961d6b` | feat(debug): preserve shutdown provenance | `mcu.py` shutdown/timeout capture `[INFERENCE]` |
| `aac1802d46ba93f142e8d6eae48db20b7ca711df` | feat(debug): trace host motion scheduling | `reactor.py` debug rings, `klippy.py` host-state dump / faulthandler, plus `extras/motion_queuing.py`, `extras/virtual_sdcard.py` `[INFERENCE]` |
| `4eec735c3be688fcf561c2ce2a5765b3558eb510` | fix(motion): tolerate host scheduling stalls | not located in core; file list unknown (grep found no obvious functional change in core files) |
| `3bc87895928711f799c4b0a535d3644ea86269cf` | feat(debug): capture step compression state | C: `stepcompress.c`, `itersolve.c` (outside slice) |
| `cf3c4ddb3cac7ae416f86349e1805ea8f193fa97` | feat(debug): trace serial dispatch decisions | C: `serialqueue.c` (outside slice) |
| `7d0860dbbded5d6b0285478478081d82168fa129` | chore(ender3-ke): retain mainline diagnostic logs | scripts/config, not core |
| `e47b09436a7ef87ab02d2212714407ccca2c0f66` | fix(motion): keep producer ahead of step generation | not located in core (`virtual_sdcard.py`/`toolhead.py` likely) |
| `3a63ec4aceff6d4576885eb29eafafef740e467a` | feat(creality): support stock c440x display endpoints | `extras/pause_resume.py` adds webhook endpoints (see 5.6) |
| `d5c37cf900b02227366abc3580ab88706822635c`, `152aa00c78cd76163f74557c27069ac527a705ac`, `4f9c676c999c5b6c35a94ded784695f3a8248ed5` | ender3-ke handover, moonraker gating, calibrated config | config/scripts only |

Other branches exist: `experiment/klippy-rs` and a reflog line `codex/klippy-rs-foundation` (an earlier Rust attempt). Its content was not inspected; check it before starting.

Fork code in this slice, all of it diagnostic and none of it changing the protocol:

* `reactor.py`: constants `DEBUG_RING_SIZE=32`, `DEBUG_STALL_TIME=0.020`, `DEBUG_LATE_TIME=0.020`, `DEBUG_DUMP_ENTRIES=10`; rings `_debug_gc_events/_debug_stalls/_debug_late_timers`; methods `_debug_note_stall`, `_debug_note_late_timer`, `get_timing_debug_report(max_entries)`; instrumentation inside `_check_gc` and `_check_timers`. `get_timing_debug_report` is called from `extras/motion_queuing.py`.
* `klippy.py`: `Printer.debug_host_dumped`, `_debug_shutdown` (registered on `klippy:analyze_shutdown`), `_debug_host_state` (called from `_set_state` for any non-ready state, from `run()` error paths), `setup_fault_handler()` (`faulthandler.enable(file=sys.stderr, all_threads=True)`).
* `mcu.py`: `DEBUG_SHUTDOWN_VERSION="shutdown-v4"`, `DEBUG_SHUTDOWN_MAX_NOTES=16`, `DEBUG_REASON_MAX_LEN=160`; `MCUConnectHelper._debug_*` methods, a first-call hook at the top of `_handle_shutdown`, a hook in `check_timeout`, the `shutdown capture armed` log line in `_mcu_identify`, and `_analyze_shutdown` dumping the frozen records.

Log lines these produce (exact formats are in the code and in the sample logs; Creality-side diagnostics read them):

* `[DEBUG-motion] host pid=%d walltime=%.6f monotonic=%.6f shutdown=%s run_result=%s threads=%d gc_count=%s gc_times=%s`
* `[DEBUG-motion] host runtime python=%r platform=%s`
* `[DEBUG-motion] host resources user=... maxrss=... core_limit=... nofile_limit=...`
* `[DEBUG-motion] host proc_status: ...` (from `/proc/self/status`: VmPeak VmSize VmHWM VmRSS VmSwap Threads FDSize)
* `[DEBUG-motion] shutdown-v4 mcu='<name>' first_report=... clock_known=... late_report=...`
* `[DEBUG-motion] version=host-v4 reactor stalls=...` plus `reactor stall`, `reactor late_timer`, `reactor gc` lines.

A Rust port must decide whether to reproduce them; `config/ender-3-v3-ke/SERVICES.md` documents them as part of the deployed diagnostics, so reproduce at least the shutdown and host-state lines.

---

## 1. Inventory

| File | Lines | Role | Port difficulty |
|---|---:|---|---|
| `klippy.py` | 444 | process entry point, `Printer` object registry, event bus, restart loop | medium |
| `reactor.py` | 522 | greenlet reactor: timers, fds, completions, mutexes | **high** |
| `configfile.py` | 560 | config parser, typed accessors, includes, SAVE_CONFIG, unused-option check | **high** |
| `gcode.py` | 490 | G-code parse and dispatch, mux commands, pty/file input | medium |
| `webhooks.py` | 577 | unix-socket JSON API (Moonraker) | medium |
| `msgproto.py` | 461 | data dictionary, message encode/decode, framing helpers | medium |
| `serialhdl.py` | 400 | serial/CAN/pipe connection, background reader thread, retry commands | medium |
| `clocksync.py` | 217 | MCU clock regression, secondary-MCU sync | medium |
| `mcu.py` | 1393 | MCU object, config/CRC, restart, trsync/endstop/pwm/adc pins, stats | **high** |
| `pins.py` | 138 | pin descriptions, aliases, command pin rewriting | low |
| `util.py` | 236 | pty, git version, cpu/device info, MCU build dump | low |
| `queuelogger.py` | 75 | background-thread rotating file logger | low |
| `mathutil.py` | 220 | coordinate descent (subprocess), trilateration, matrix/gauss solve | medium |
| `console.py` | 286 | standalone MCU debug console (keyboard, `eval`) | low |
| `parsedump.py` | 45 | standalone `.dict` + serial dump decoder | low |
| `toolhead.py` | ~607 | motion coordinator (other slice) | n/a |
| `stepper.py` | ~490 | stepper, rail, MCU stepper wrapper (other slice) | n/a |

Third-party and stdlib imports in this slice:

* Third party: `greenlet` (`reactor.py`), `cffi` via `chelper` (`reactor`, `serialhdl`, `mcu`), `serial`/pyserial (`serialhdl`), `can`/python-can (lazy, `serialhdl.connect_canbus`), `msgspec` (optional, `webhooks`).
* Stdlib worth noting: `configparser`, `logging`, `select`, `socket`, `threading`, `queue`, `multiprocessing`, `shlex`, `json`, `zlib`, `glob`, `struct`, `termios`/`pty`/`fcntl`, `subprocess`, `faulthandler`, `resource`.

Import graph (direct, klippy units only):

```
klippy   -> util reactor queuelogger msgproto gcode configfile pins mcu toolhead webhooks
reactor  -> chelper(get_monotonic) util(set_nonblock)
gcode    -> (stdlib only; calls printer/toolhead by duck typing)
webhooks -> gcode(CommandError)
configfile -> (stdlib only)
pins     -> (stdlib only)
mcu      -> serialhdl msgproto pins chelper clocksync
serialhdl-> msgproto chelper util
clocksync-> (stdlib only)
mathutil -> queuelogger
console  -> util reactor serialhdl msgproto clocksync
parsedump-> msgproto
toolhead -> mcu chelper kinematics.extruder ;  stepper -> chelper
```

Importers of `mathutil` outside this slice: `extras/{bed_tilt,delta_calibrate,probe_eddy_current,shaper_defs,temperature_probe,z_tilt}.py`, `kinematics/{delta,generic_cartesian,rotary_delta,winch}.py`. `parsedump.py` is referenced only by `docs/Debugging.md` and `docs/CANBUS_Troubleshooting.md`.

---

## 2. Per-unit sheets

### 2.1 `klippy.py` (444 lines)

1. **Size**: 444.
2. **Dependencies**: `util, reactor, queuelogger, msgproto, gcode, configfile, pins, mcu, toolhead, webhooks`; stdlib `optparse, gc, importlib, faulthandler, resource, threading`. No third-party package.
3. **External contracts**
   * CLI (optparse): `klippy.py [options] <config>`; `-i/--debuginput FILE`, `-I/--input-tty` (default `/tmp/printer`), `-a/--api-server SOCK`, `-l/--logfile`, `-v`, `-o/--debugoutput FILE`, `-d/--dictionary [MCU=]FILE` (repeatable; key `dictionary` or `dictionary_<mcu>`), `--import-test` (imports every `extras.*` and `kinematics.*` module then exits 0). Exactly one positional arg else `opts.error`.
   * `start_args` dict (read by everything): `config_file, apiserver, start_reason ('startup'|'restart'|'firmware_restart'), debuginput, gcode_fd, debugoutput, dictionary, dictionary_<mcu>, log_file, software_version, cpu_info, device, linux_version`. `start_args.update(options.dictionary)` crashes with `TypeError` if `-o` is given without `-d` (`dictionary` stays `None`). That is an upstream quirk, the tests always pass both.
   * Exit codes: `run()` returns `'exit'`, `'error_exit'`, `'restart'`, `'firmware_restart'`. Loop breaks on `exit`/`error_exit`; process exits `sys.exit(-1)` (255) only for `error_exit`. Every other result sleeps 1 s, `finalize()`s the reactor, builds a new Reactor and Printer, sets `start_args['start_reason']=res`.
   * `Printer` public API used by every extra: `config_error` (= `configparser.Error`), `command_error` (= `gcode.CommandError`), `get_start_args`, `get_reactor`, `get_state_message() -> (msg, 'ready'|'startup'|'shutdown'|'error')`, `is_shutdown`, `update_error_msg(old,new)`, `add_object` (error `Printer object '%s' already created`), `lookup_object(name, default)` (error `Unknown config object '%s'`), `lookup_objects(module=None)` (prefix match is `module + ' '`, plus exact name first), `load_object(config, section, default)`, `register_event_handler`, `send_event` (returns list of handler results, runs synchronously in registration order), `request_exit(result)` (first result wins), `invoke_shutdown(msg, details={})`, `invoke_async_shutdown` (thread-safe via `reactor.register_async_callback`), `set_rollover_info(name, info, log=True)`, `get_start_args`, `run`.
   * State strings: `message_ready="Printer is ready"`, `message_startup` ("Printer is not ready\nThe klippy host software is attempting to connect.  Please\nretry in a few moments.\n"), `message_restart` (the `RESTART` hint). Config errors set the state to `"%s\n%s" % (err, message_restart)`.
   * Events emitted: `klippy:mcu_identify`, `klippy:connect` (handlers called one by one, loop aborts if the state message changed), `klippy:ready` (inside `assert_no_pause`), `klippy:shutdown` and `klippy:analyze_shutdown(msg, details)` (inside `assert_no_pause`, each handler wrapped in try/except with `logging.exception`), `klippy:disconnect`, `klippy:firmware_restart`, `klippy:notify_mcu_error(msg, {"error": str})`. Other slices add `gcode:*`, `toolhead:*`, `stepper:*`, `homing:*`, `idle_timeout:*`, `probe:update_results`, `extruder:activate_extruder`, `dual_carriage:update_kinematics`, `load_cell:*`, `virtual_sdcard:reset_file`, `stepper_enable:motor_off`, `menu:*`.
   * Object load order (`_read_config`): `configfile` object, `pins`, `mcu` (+ every `mcu <name>` section), then every config section in file order via `load_object(config, name, None)`, then `toolhead`, then `check_unused_options`. `load_object` imports `extras.<first word of section>` (file or package with `__init__.py`), calls `load_config(section)` for one-word sections or `load_config_prefix(section)` for multi-word ones; sections without a module are silently skipped here and rejected later by the unused-section check.
   * Files read: the config file(s), `/proc/cpuinfo`, `/proc/device-tree/model` or `/sys/class/dmi/id/product_name`, `/proc/version`, `/proc/self/status`, `klippy/.version`, repo `.config` and `out/klipper.dict` (on MCU error). Files written: pty symlink `-I`, the log file.
   * Log lines: `Starting Klippy...`, `Args: ...`, `Git version: ...`, `CPU:`, `Device:`, `Linux:`, `Python:`, `Start printer at %s (%.1f %.1f)`, `Transition to shutdown state: %s` (ERROR), `Config error` (exception), `Protocol error`, `MCU error during connect`, `Unhandled exception during connect/run/ready callback`, `Restarting printer`, `No log file specified! Severe timing issues may result!`. The `versions` block is stored as log-rollover info.
4. **Runtime traits**
   * Dynamic plugin loading: `importlib.import_module('extras.' + name)`; `getattr(mod, 'load_config'|'load_config_prefix')`. A Rust port needs a static name-to-constructor registry for 143 modules plus prefix-section semantics.
   * `gc.disable()` and explicit collection only from the reactor idle path.
   * `self.reactor.register_callback(self._connect)` runs startup inside a reactor callback so config loading may pause (MCU connect calls `reactor.pause`).
   * `invoke_shutdown` is idempotent and is called from any callback. `invoke_async_shutdown` is the only thread-safe path.
   * Restart loop re-creates all state; the C serialqueue objects are released by cffi GC when the old Printer is dropped.
   * `util.get_git_version()` shells out to `git` (describe, status --porcelain --ignored, branch, config, remote get-url); falls back to `klippy/.version`.
5. **Tests**: every `.test` runs `klippy.py ... -i gcode -o _test_output -v [-l log]` (see `scripts/test_klippy.py`). `commands.test` ends with `RESTART` (with file input RESTART becomes `exit`). `out_of_bounds.test` is `SHOULD_FAIL`. `--import-test` is not run by `test_klippy.py`.
6. **Fork**: `_debug_host_state`, `setup_fault_handler`, `debug_host_dumped`, `_debug_shutdown` (see 0.1). `_set_state` calls `_debug_host_state()` on every transition to a non-ready state (also on config errors and restarts-with-error).
7. **Difficulty: medium**. The logic is small, but the object registry, event bus, dynamic extras and restart loop are the spine every other slice plugs into.

### 2.2 `reactor.py` (522 lines)

1. **Size**: 522.
2. **Dependencies**: `greenlet`, `chelper` (`get_ffi()[1].get_monotonic`), `util.set_nonblock`; stdlib `select, queue, os, gc, math, time, collections`.
3. **External contracts** (API surface used by all extras):
   `monotonic()` (C `get_monotonic`, CLOCK_MONOTONIC), constants `NOW=0.`, `NEVER=9999999999999999.`; `register_timer(cb, waketime=NEVER) -> ReactorTimer`, `update_timer(t, waketime)`, `unregister_timer(t)`; `register_callback(cb, waketime=NOW) -> ReactorCompletion`; `register_async_callback(cb, waketime=NOW)` and `async_complete(completion, result)` (thread-safe); `completion()` (with `test()`, `complete(result)`, `wait(waketime=NEVER, waketime_result=None)`); `pause(waketime) -> eventtime`; `mutex(is_locked=False)` (`test()`, `lock/unlock`, context manager); `assert_no_pause()` context manager and `verify_can_pause()` (raises `ReactorError("Internal error - reactor pause disabled")`); `register_fd(fd, read_cb, write_cb=None)`, `unregister_fd`, `set_fd_wake(handler, readable, writeable)`; `run()`, `end()`, `finalize()`; `get_gc_stats()`; fork: `get_timing_debug_report(max_entries)`.
   `Reactor = PollReactor` when `select.poll` exists (always on Linux). `SelectReactor` (non-Linux) and `EPollReactor` (never selected) share the same logic.
4. **Runtime traits**: the whole concurrency model, described exactly in section 3.
5. **Tests**: exercised by every test. No unit test targets it.
6. **Fork**: `[DEBUG-motion] v4` stall, late-timer and gc rings (see 0.1). They add two float compares to the common path. `_check_timers` ignores lateness when `waketime == 0.` (NOW).
7. **Difficulty: high**. Stackful coroutines with a particular dispatcher hand-off protocol; every module relies on its semantics.

### 2.3 `configfile.py` (560 lines)

1. **Size**: 560.
2. **Dependencies**: stdlib `configparser, glob, io, re, os, time`; `printer.lookup_object("configfile")` and `gcode` by duck typing. No other klippy unit.
3. **Contracts**
   * G-code: `SAVE_CONFIG` (no parameters; help "Overwrite config file and restart").
   * `ConfigWrapper` API used by the extras: `get_printer, get_name, get, getint, getfloat, getboolean, getchoice, getlists, getlist, getintlist, getfloatlist, getsection, has_section, get_prefix_sections, get_prefix_options, deprecate, error`. Keyword rules are in section 4.
   * `PrinterConfig` object `configfile`: `read_config(filename)` (extras reading a second file, no includes), `read_main_config`, `log_config`, `check_unused_options`, `runtime_warning(msg)`, `deprecate(section, option, value=None, msg=None)`, `deprecate_gcode(cmd, param, value, msg)`, `deprecate_mcu_code(mcu, feature, msg)`, `set(section, option, value)`, `remove_section(section)`.
   * `get_status()` fields: `config` (raw strings, section to option to string), `settings` (value actually read, typed Python values, keyed by lowercase section then lowercase option), `warnings` (list of dicts, types `deprecated_option, deprecated_value, deprecated_gcode, deprecated_mcu_code, runtime_warning`), `save_config_pending` (bool), `save_config_pending_items` (section to option to string, `None` meaning "section removed").
   * Files: reads the config and every included file; on `SAVE_CONFIG` writes `<name>_autosave.cfg` then renames the original to `<name>-YYYYMMDD_HHMMSS.cfg` and the temp file to the original name.
   * Log lines: `Can't read autosave from config file - autosave state corrupted`, `... - modifications after header`, `save_config: set [%s] %s = %s`, `SAVE_CONFIG to '%s' (backup in '%s')`, `Unable to open config file %s`.
4. **Runtime traits**: relies entirely on Python `configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';','#'))` semantics; Python `int()`/`float()`/`str(float)` semantics for parsing and messages; `status_save_pending` is replaced (copy-on-write) so readers on other greenlets see consistent dicts; typed `settings` values are stored in a shared `access_tracking` dict across all `ConfigWrapper`s.
5. **Tests**: all configs in `test/klippy/*.cfg` and `config/example-*.cfg` (via `printers.test`). `delta_calibrate.cfg` and `rotary_delta_calibrate.cfg` carry real autosave blocks, so the autosave load path is exercised. No test calls `SAVE_CONFIG`, no test config uses `[include]`.
6. **Fork**: no code change. Fork configs use `[include gcode_macro.cfg]`, `[include /usr/data/mainsail-config/mainsail.cfg]` (absolute path), option names with a space before the colon (`gcode :`) and inline `;` comments (`config/ender-3-v3-ke/printer.cfg`).
7. **Difficulty: high**. Behaviour is defined by the Python `configparser` implementation and by Python number formatting; 143 modules depend on exact accept/reject behaviour and error text.

### 2.4 `gcode.py` (490 lines)

1. **Size**: 490.
2. **Dependencies**: stdlib `re, shlex, collections, operator, os`; `printer` duck-typed (`toolhead` in `request_restart`).
3. **Contracts**
   * Commands registered before config (also in `base_gcode_handlers`): `M110` (no-op), `M112` (shutdown "Shutdown due to M112 command"), `M115` (FIRMWARE_NAME:Klipper FIRMWARE_VERSION:<software_version>), `RESTART`, `FIRMWARE_RESTART`, `ECHO` (echoes full command line via `respond_info`), `STATUS`, `HELP`. Other `when_not_ready` registrations: `M114`, `GET_POSITION` (gcode_move), `M105` (heaters), `M118`, `RESPOND` (respond).
   * Object `gcode` API: `register_command(cmd, func, when_not_ready=False, desc=None)` (func `None` unregisters and returns the old handler), `register_mux_command(cmd, key, value, func, desc=None)`, `register_output_handler(cb)`, `get_command_help()`, `get_status()` -> `{'commands': {name: {'help': str}}}`, `run_script(script)` (takes mutex), `run_script_from_command(script)` (no mutex), `get_mutex()`, `create_gcode_command(command, commandline, params)`, `respond_raw`, `respond_info(msg, log=True)`, `request_restart(result)`, `is_traditional_gcode`, `Coord` class (tuple padded to 4, x/y/z/e properties), `error` (= `CommandError`).
   * `GCodeCommand` API: `get_command, get_commandline, get_command_parameters` (dict, keys upper-case, values raw strings), `get_raw_command_parameters`, `ack(msg=None)`, `respond_info`, `respond_raw`, `get(name, default, parser, minval, maxval, above, below)`, `get_int`, `get_float`, `error`.
   * `gcode_io` object: pty or file input; `stats()` -> `(False, "gcodein=%d")`.
   * Output formats: info lines `"// " + line` per line; errors `"!! first line"` (multi-line errors are preceded by an info block of all lines); acks `ok` or `ok <msg>`; state notices `// Klipper state: Ready|Shutdown|Disconnect`; unknown command `// Unknown command:"X"`.
   * Events: `gcode:command_error` (after an error response), `gcode:request_restart(print_time)`.
   * Files: the pty slave symlink `/tmp/printer` (or `-I`), created by `util.create_pty`; with `-i` the file is opened `rb` and read like a stream.
   * Log lines: `Exiting (print time %.3fs)`, `Internal error on command:"%s"` (exception), `Dumping gcode input %d blocks` / `Read %f: %r` (on analyze_shutdown).
4. **Runtime traits**: reactor mutex and greenlet re-entry in `GCodeIO._process_data` (section 3.4); `lambda`-wrapped handlers for extended commands; `shlex` posix parsing; `getattr(self, 'cmd_' + name)` for the early commands; M112 out-of-order detection at read time.
5. **Tests**: all tests drive G-code through `GCodeIO` in file mode; `commands.test` covers `STATUS`, `HELP`, `M115`, `RESTART`; `macros.test` uses `M112` inside templates as the failure signal.
6. **Fork**: no code change in this file.
7. **Difficulty: medium**. The grammar is simple but several Python-specific behaviours (regex split, `shlex`, `float()` acceptance) leak into user-visible results, and the re-entrant input handler must keep its ordering.

### 2.5 `webhooks.py` (577 lines)

1. **Size**: 577.
2. **Dependencies**: `gcode` (`CommandError`); stdlib `socket, errno, collections, os, sys`; optional `msgspec`, else `json`.
3. **Contracts**: complete API in section 7.
4. **Runtime traits**: reactor fd callbacks with write-readiness switching; each request runs in its own `register_callback` greenlet, so handlers may pause (a `gcode/script` request waits on the G-code mutex); `QueryStatusHelper._do_query` runs `get_status()` of arbitrary objects under `assert_no_pause`; JSON encoder choice (`msgspec` vs `json`) changes number/NaN encoding; objects used as dict keys (`ClientConnection` instances); `id(self)` as client uid.
5. **Tests**: none drive the socket (disabled for file input: `ServerSocket` returns early when `debuginput` is set or no `-a`). The endpoints are registered during every test, so `register_endpoint`/`register_mux_endpoint` conflicts would show up; `get_status` trees are never queried through `objects/query`.
6. **Fork**: no change here; fork endpoints are registered from `extras/pause_resume.py` (section 7.6).
7. **Difficulty: medium**. Protocol is small; the risk is fidelity (framing, ordering of responses relative to pauses, delta-subscription semantics, JSON number encoding).

### 2.6 `msgproto.py` (461 lines)

1. **Size**: 461.
2. **Dependencies**: stdlib `json, zlib, logging`.
3. **Contracts**: protocol described in section 6.2 and 6.3. Public classes: `MessageParser` (`process_identify, parse, dump, check_packet, encode_msgblock, create_command, create_dummy_response, lookup_command, lookup_msgid, get_messages, get_enumerations, get_constants, get_constant[_float|_int], get_version_info, get_raw_data_dictionary, format_params, fill_enumerations`), `MessageFormat`, `OutputFormat`, `UnknownFormat`, `Enumeration`, `enumeration_error`, `error`, `crc16_ccitt`. `parsedump.py` and `console.py` use it directly. Exceptions: `msgproto.error` is caught in `klippy._connect` and becomes the state `Protocol error`.
4. **Runtime traits**: `create_command` parses text with `int(value, 0)` (accepts `0x`, `0o`, `0b`, underscores, rejects leading zeros like `010`) and catches bare `except:`; `convert_msg_format` uses Python `%` formatting for `#output`; the identify blob is zlib+JSON with numeric values stored as strings or ints. No threads inside, but `parse()` runs on the serial background thread.
5. **Tests**: indirectly in every test (file mode loads `atmega2560.dict`, `stm32h723.dict`, `linuxprocess.dict` from `-d`). No dedicated test.
6. **Fork**: none.
7. **Difficulty: medium**. Compact and deterministic, so it can be verified exhaustively against dictionary files, but the text-command parsing and error strings have to match.

### 2.7 `serialhdl.py` (400 lines)

1. **Size**: 400.
2. **Dependencies**: `msgproto, chelper, util`; `serial` (pyserial), `can` (python-can, lazy import in `connect_canbus`); stdlib `threading, os`.
3. **Contracts**
   * `SerialReader` API: `connect_uart(port, baud, rts)`, `connect_canbus(uuid, nodeid, iface)`, `connect_pipe(path)`, `connect_file(outfile, dict_bytes)`, `disconnect`, `send(msg, minclock, reqclock)`, `send_with_response(msg, response)`, `raw_send`, `raw_send_wait_ack`, `register_response(cb, name, oid=None)` (`cb=None` unregisters), `alloc_command_queue`, `set_clock_est`, `stats(eventtime)`, `dump_debug`, `get_msgparser`, `get_serialqueue`, `get_default_command_queue`, `get_reactor`. Also `SerialRetryCommand`, `stk500v2_leave`, `arduino_reset`, `cheetah_reset`, `error`.
   * Log text: `mcu '<name>': Starting serial connect|Starting CAN connect|Starting connect`, `Unable to open serial port: %s`, `Timeout on connect`, `Wait for identify_response` (exception), `Exception in serial callback`, `Unknown message type %d: %s`, `got %s` (default handler, WARNING), `<name>: <msg>` for `#output` messages. The warning prefix is `"mcu '%s': "` when the MCU has a name.
   * C interface: `serialqueue_alloc/exit/free`, `serialqueue_alloc_commandqueue/free_commandqueue`, `serialqueue_send`, `serialqueue_pull`, `serialqueue_set_wire_frequency`, `serialqueue_set_receive_window`, `serialqueue_set_clock_est`, `serialqueue_get_stats`, `serialqueue_extract_old`. Details in 6.5.
4. **Runtime traits**: one Python thread (`serialhdl <name>`, named via `set_thread_name`) blocks in the C `serialqueue_pull`; response handlers run on that thread with `self.lock` held; `async_complete` is the only legal way to wake the reactor from it; `pending_notifications` dict is touched from both threads (GIL-protected); blocking `reactor.pause` loops with 5 s retry and a 90 s give-up; CAN uses a python-can `Bus` object whose `fileno()` goes to C (`serial_fd_type=b'c'`).
5. **Tests**: only `connect_file` (debug output) is run. UART, pipe, CAN paths and the retry logic are untested; `linuxtest.test` uses file mode too.
6. **Fork**: Python file unchanged. The fork's serial tracing is in C (`serialqueue.c`, `[DEBUG-motion] version=motion-v4 serial=...` lines are emitted through the cffi logging callback and appear in Python logs as `ERROR:root:b'...'`).
7. **Difficulty: medium**. Small Python surface; the work is the thread/C boundary and the platform I/O.

### 2.8 `clocksync.py` (217 lines)

1. **Size**: 217.
2. **Dependencies**: stdlib `math, logging` only. Calls `serial.send_with_response`, `serial.raw_send`, `serial.register_response`, `serial.set_clock_est`, `serial.alloc_command_queue`.
3. **Contracts**
   * MCU messages: `get_uptime` -> `uptime high=%u clock=%u`; `get_clock` -> `clock clock=%u` (params `clock`, plus `#sent_time`, `#receive_time`). Constant `CLOCK_FREQ` required.
   * API: `connect, connect_file(serial, pace=False), print_time_to_clock, clock_to_print_time, get_clock(eventtime), estimate_clock_systime, estimated_print_time, clock32_to_clock64, is_active, dump_debug, stats, calibrate_clock(print_time, eventtime)`; public fields `clock_est`, `mcu_freq`, `last_clock`, `main_sync` (secondary).
   * Log lines: `new minimum rtt %.3f: hrtt=%.6f freq=%d` (debug), `Ignoring clock sample ...` (debug), `Resetting prediction variance ...` (info). `stats()` returns `freq=%d` (+ ` adj=%d` for secondary); `mcu.py` parses it.
4. **Runtime traits**: `_get_clock_event` is a reactor timer (period `0.9839 s` to avoid resonance); `_handle_clock` runs on the serial thread and writes `self.clock_est`, `self.last_clock`, `queries_pending` that the main thread reads without locks (relies on single-assignment atomicity); all arithmetic is `f64`; `int()` truncates toward zero. The linear-regression update feeds `serialqueue_set_clock_est` and thus the real send times; it is float-sensitive but not bit-exact-critical (the MCU never sees these numbers).
5. **Tests**: only `connect_file`/`SecondarySync.connect_file` (clock estimate fixed at `freq=1e12`) and the secondary `calibrate_clock` path in dual-MCU tests (`generic_cartesian_iqex.test`, `generic_cartesian_itex.test`).
6. **Fork**: none.
7. **Difficulty: medium**. Short, but the estimation filter has many constants and must reproduce Python float behaviour; golden-sample replay can validate it.

### 2.9 `mcu.py` (1393 lines)

1. **Size**: 1393.
2. **Dependencies**: `serialhdl, msgproto, pins, chelper (trdispatch_*, run_hub_ctrl), clocksync`; `zlib`, `struct`, `math`; `printer.lookup_object('pins' | 'configfile' | 'canbus_ids' | 'canbus_stats' | 'error_mcu' | 'motion_queuing')`.
3. **Contracts**
   * Config options on `[mcu]`/`[mcu name]`: `serial` (required unless `canbus_uuid`), `baud` (default 250000, minval 2400; not read for `/dev/rpmsg_*` or `/tmp/klipper_host_*` paths), `canbus_uuid`, `canbus_interface` (default `can0`), `restart_method` (choices `None, 'arduino', 'cheetah', 'command', 'rpi_usb'`; only read when a baud applies). Config section name prefix `mcu ` is stripped.
   * Printer objects: `mcu` and `mcu <name>`; `get_printer_mcu(printer, name)`. MCU status: `mcu_version`, `mcu_build_versions`, `mcu_constants`, `last_stats` (dict from the `stats` response and serial/clocksync stats, floats if the text has `.` else ints).
   * Public MCU API (used by steppers, heaters, endstops, TMC, LEDs, etc.): `get_name, get_printer, is_fileoutput, setup_pin(pin_type, pin_params)` (types `endstop, digital_out, pwm, adc`), `create_oid, register_config_callback, add_config_cmd(cmd, is_init, on_restart), request_move_queue_slot, get_query_slot, seconds_to_clock, min_schedule_time (0.100), max_nominal_duration (3.0), lookup_command(fmt, cq), lookup_query_command(fmt, respfmt, oid, cq, is_async), try_lookup_command, alloc_command_queue, register_serial_response, check_valid_response, get_enumerations, get_constants, get_constant_float, print_time_to_clock, clock_to_print_time, estimated_print_time, clock32_to_clock64, calibrate_clock, get_status, stats`.
   * Wrappers: `CommandWrapper.send(data, minclock, reqclock)`, `send_wait_ack`, `get_command_tag`; `CommandQueryWrapper.send / send_with_preface`; `AsyncResponseWrapper`; `DummyResponse`; `RetryAsyncCommand`; pin classes `MCU_endstop, MCU_digital_out, MCU_pwm, MCU_adc`; `MCU_trsync`, `TriggerDispatch` (cffi to `trdispatch_*`).
   * MCU commands and responses used (complete list): `identify/identify_response`, `get_uptime/uptime`, `get_clock/clock`, `get_config/config`, `allocate_oids`, `finalize_config`, `emergency_stop`, `reset`, `config_reset`, `shutdown`/`is_shutdown`/`starting` (responses), `stats`, `config_trsync`, `trsync_start`, `trsync_set_timeout`, `trsync_trigger`, `trsync_state`, `stepper_stop_on_trigger`, `config_endstop`, `endstop_home`, `endstop_query_state`/`endstop_state`, `config_digital_out`, `update_digital_out`, `queue_digital_out`, `set_digital_out_pwm_cycle`, `config_pwm_out`, `queue_pwm_out`, `config_analog_in`, `query_analog_in` (two forms), `analog_in_state` (two forms), `get_canbus_id/canbus_id`. Constants read: `CLOCK_FREQ, SERIAL_BAUD, CANBUS_BRIDGE, STATS_SUMSQ_BASE, ADC_MAX, PWM_MAX, RESERVE_PINS_*`.
   * Log lines: `Loaded MCU '%s' %d commands (%s / %s)`, `MCU '%s' config: K=V ...`, `Sending MCU '%s' printer configuration...`, `Configured MCU '%s' (%d moves)`, `Attempting automated MCU '%s' restart: %s`, `Attempting MCU '%s' reset command`, `... config_reset command`, `Timeout with MCU '%s' (eventtime=%f)`, `MCU '%s' shutdown: ...` (analysis dump with clocksync and serial dumps). Error strings include `MCU '%s' CRC does not match config`, `Failed automated reset of MCU '%s'`, `Unable to configure MCU '%s'`, `Too few moves available on MCU '%s'`, `Pin '%s' is not a valid pin name on mcu '%s'`, `Lost communication with MCU '%s'`, `MCU '%s' spontaneous restart`, `MCU shutdown`. `error_mcu.py` pattern-matches some of these (`"MCU shutdown"`, `"Protocol error"`, `"MCU error during connect"`), so keep them verbatim.
   * Config-time pins: warnings through `pconfig.runtime_warning("MCU '%s' configured for %dMhz but running at %dMhz!")`.
4. **Runtime traits**
   * Three code paths run on different threads: `_handle_shutdown`, `_handle_starting`, `_handle_trsync_state`, ADC handlers, `RetryAsyncCommand.handle_callback` run on the serial reader thread; everything else on the reactor.
   * `reactor.pause` loops in restart logic (`pause(+2.0)` after requesting a firmware restart, `+0.015` between reset and disconnect).
   * Config commands are built as text, pin-rewritten by `PinResolver.update_command`, joined with `\n`, hashed with `zlib.crc32 & 0xffffffff` and then re-parsed by `MessageParser.create_command` when sent. The CRC is compared with the value stored in the MCU, so the exact text (`%d` rendering of ints, `%s` of floats) is part of the contract. A mismatch against a running MCU triggers an automated restart (observed in the fork logs as `Attempting automated MCU 'mcu' restart: CRC mismatch`).
   * Float-sensitive values that reach the MCU: `seconds_to_clock = int(time * mcu_freq)`; `MCU_pwm` `int(max(0,min(1,value))*pwm_max+0.5)`; `rest_ticks`, `report_ticks`, `expire_ticks`, `min_extend_ticks` in `MCU_trsync.start`; `MCU_adc` `min_sample/max_sample` integer conversions; `get_query_slot`. All use truncation toward zero or `+0.5` rounding in `f64`.
   * `MCU_adc._handle_analog_in_state` uses `struct.Struct('<H').unpack_from(params['values'])`, which returns a 1-tuple, so a batched report (`batch_num > 1`) yields one sample; the remaining bytes are ignored. Whether this is a repo bug or intended, a port must pick a behaviour on purpose. Batch mode is used only when `batch_num != 1` or the firmware lacks the old command form.
   * `MCU_trsync` handlers call `reactor.async_complete`; `TriggerDispatch.start/wait_end/stop` mix a Python `ReactorCompletion` with C trdispatch structures.
5. **Tests**: every test (file mode; `DummyResponse` stands in for queries; `get_config` returns `{is_config:0, move_count:500, crc:0}`). `generic_cartesian_*` tests use a second MCU `extboard`. `linuxtest.test` uses the Linux-process dictionary. UART/CAN/restart/shutdown paths are untested.
6. **Fork**: shutdown/timeout provenance capture, `shutdown capture armed` log, `_analyze_shutdown` dumps (see 0.1). The deployed printer has two MCUs: `mcu` (gd32f303xe on `/dev/ttyS1` at 230400, `restart_method: command`) and `rpi` (Linux process MCU on pipe `/tmp/klipper_host_mcu`).
7. **Difficulty: high**. Largest file; mixes config-time text generation, startup state machine, restart policies, cffi lifetimes, and the fork's thread-side diagnostics.

### 2.10 `pins.py` (138 lines)

1. **Size**: 138. 2. **Deps**: stdlib `re`.
3. **Contracts**: object `pins` (`PrinterPins`): `parse_pin(desc, can_invert, can_pullup)` returns `{chip, chip_name, pin, invert, pullup(-1|0|1), (share_type)}`; `lookup_pin(desc, can_invert, can_pullup, share_type)`; `setup_pin(pin_type, desc)`; `reset_pin_sharing`; `get_pin_resolver(chip_name)`; `register_chip(name, chip)`; `allow_multi_use_pin(desc)`. Pin description grammar: optional `^` or `~` (pullup, only where allowed), optional `!` (invert, only where allowed), optional `chip:` (default `mcu`), then the pin name (no whitespace or `^~!:`). `PinResolver`: `reserve_pin, alias_pin, update_command`; regex `(?P<prefix>[ _]pin=)(?P<name>[^ ]*)` rewrites every `pin=X` / `_pin=X` argument inside config command text. Error strings: `Unknown pin chip name '%s'`, `Invalid pin description '%s'\nFormat is: %s[chip_name:] pin_name`, `pin %s used multiple times in config`, `Shared pin %s must have same polarity`, `Duplicate chip name '%s'`, `Unknown chip name '%s'`, `Pin %s reserved for %s - can't reserve for %s`, `Alias %s mapped to %s - can't alias to %s`, `Invalid pin alias '%s'\n`, `pin %s is an alias for %s`, `pin %s is reserved for %s`. `pins.error` is caught in `klippy._connect` as a config error. Registered chips include every MCU plus extras that act as pin chips (replicape, pca9685, etc., other slices).
4. **Runtime traits**: none beyond regex text rewriting; chip objects are duck-typed (`setup_pin` method).
5. **Tests**: every config (all use pins). 6. **Fork**: none.
7. **Difficulty: low**.

### 2.11 `util.py` (236 lines)

1. **Size**: 236. 2. **Deps**: stdlib only (`pty, fcntl, termios, signal, subprocess, json`).
3. **Contracts**: `fix_sigint()` runs at import (restores `SIGINT` default so Ctrl-C kills the process); `set_nonblock(fd)`; `clear_hupcl(fd)`; `create_pty(path)` (unlink old symlink, `chmod 0o660` the slave, symlink, non-blocking master, ECHO off, returns master fd); `dump_mcu_build()` (logs `Build file %s(%d): %s`, `========= Last MCU build config =========`, `Last MCU build version/tools/config`); `get_cpu_info()` -> `"%d core %s"`; `get_device_info()`; `get_linux_version()`; `get_version_from_file(dir)`; `get_git_version()` -> `{version, file_status[(code, filename)], branch, remote, url}` (git `describe --always --tags --long --dirty`, `status --porcelain --ignored`); `setup_python2_wrappers()` (drop).
4. **Runtime traits**: subprocess `git`; termios and pty are Linux-only; import-time signal side effect.
5. **Tests**: all tests call `get_git_version`, `get_cpu_info` etc. (output unchecked).
6. **Fork**: none. 7. **Difficulty: low**.

### 2.12 `queuelogger.py` (75 lines)

1. **Size**: 75. 2. **Deps**: stdlib `logging.handlers, threading, queue`.
3. **Contracts**: `setup_bg_logging(filename, level)` returns the `QueueListener` (a `TimedRotatingFileHandler`, `when='midnight'`, `backupCount=5`) which has `set_rollover_info(name, info|None)`, `clear_rollover_info()`, `stop()`. The `QueueHandler` formats each record in the producer thread (so tracebacks are captured), clears `args`/`exc_info`, and enqueues. On rollover the handler writes each stored info block sorted by name, followed by `=============== Log rollover at %s ===============` (`time.asctime()`). `clear_bg_logging()` detaches the handler and sets the root level to WARNING (used in forked subprocesses). Log format is the stdlib default `LEVEL:root:message`; without `-l` the root logger has no handler and the first `logging.info()` call installs `basicConfig` to stderr (which is how the deployed printer logs).
4. **Runtime traits**: one background thread; `queue.Queue` unbounded; rotation timing.
5. **Tests**: used when the test runs without `-v` (`-l _test_.log`). 6. **Fork**: none. 7. **Difficulty: low**.

### 2.13 `mathutil.py` (220 lines)

1. **Size**: 220. 2. **Deps**: `queuelogger` (`clear_bg_logging` in the child process); stdlib `multiprocessing, math, operator, traceback`.
3. **Contracts**: `coordinate_descent(adj_params, params, error_func)` (round limit 10000, threshold 1e-5, step growth 1.1 / shrink 0.9; logs `Coordinate descent initial error: %s`, `... best_err: %s  rounds: %d`); `background_coordinate_descent(printer, adj_params, params, error_func)` (forks a `multiprocessing.Process`, polls with `reactor.pause(eventtime + .1)`, emits `Working on calibration...` through `gcode.respond_info(..., log=False)` every 5 s, raises `Exception("Error in coordinate descent: ...")`); `trilateration`; 3-vector helpers `matrix_cross, matrix_dot, matrix_magsq, matrix_add, matrix_sub, matrix_mul`; NxM helpers `mat_transp, mat_mat_mul, mat_mul_transp, gaussian_solve(a, rhs, allow_underdetermined), pseudo_inverse, solve_linear_equations`.
4. **Runtime traits**: `multiprocessing.Process` relies on `fork` and passes closures without pickling; the child redirects logging; `gaussian_solve` pivots with `abs_col.index(max(abs_col))` (first maximum wins) and divides with `1. / m_i[i]`; evaluation order determines the float result. `shaper_defs.py` feeds `pseudo_inverse` results into input-shaper pulse amplitudes and times that go to C `kin_shaper.c`, so it reaches step timing.
5. **Tests**: `delta_calibrate.test`, `rotary_delta_calibrate.test` (coordinate descent subprocess), `z_tilt.test`, `input_shaper.test`, `eddy.test`, `printers.test` (delta/winch/rotary-delta kinematics) `[INFERENCE: mapped from importer list, not from test bodies]`.
6. **Fork**: none (copyright line shows a 2025-2026 co-author, upstream).
7. **Difficulty: medium**. Simple numerics, but bit-level float ordering and the subprocess calibration need care.

### 2.14 `console.py` (286 lines)

1. **Size**: 286. 2. **Deps**: `util, reactor, serialhdl, msgproto, clocksync`; `optparse`.
3. **Contracts**: standalone tool, `console.py [-v] [-b baud] [-c canbus_iface] [-i canbus_nodeid=64] <serialdevice>`. Reads stdin through the reactor; local commands `SET, DUMP, FILEDUMP, DELAY, FLOOD, SUPPRESS, STATS, LIST, HELP`; `{expr}` expressions evaluated with Python `eval()` over user variables plus `clock` and `freq`. Output lines `%07.3f: %s`. MCU messages used: `debug_read order=%d addr=%d` -> `debug_result`.
4. **Runtime traits**: `eval`, reactor coroutines, nonblocking stdin.
5. **Tests**: none. 6. **Fork**: none.
7. **Difficulty: low**; not needed for runtime parity. Port last, or replace `eval` with a small arithmetic evaluator.

### 2.15 `parsedump.py` (45 lines)

1. **Size**: 45. 2. **Deps**: `msgproto`.
3. **Contracts**: `parsedump.py <dict> <dumpfile>` decodes a raw wire dump or a `-o` debug output file into one line per command (`MessageParser.check_packet` resync loop: on a negative return drops `-l` bytes; prints `Invalid data` to the log). Documented in `docs/Debugging.md`.
4. **Runtime traits**: none. 5. **Tests**: none.
6. **Fork**: none.
7. **Difficulty: low**. Strategic value: it is the oracle for comparing the `-o` output of the Python and Rust implementations (see section 8).

### 2.16 `toolhead.py` and `stepper.py` (interface notes only)

* `toolhead.py` calls into this slice through: `mcu.min_schedule_time/estimated_print_time/is_fileoutput`, `printer.send_event("toolhead:*")`, `reactor.assert_no_pause()` around the move-to-trapq loop, `printer.register_event_handler("klippy:shutdown")`. It exposes `get_last_move_time()`, `dwell()`, `wait_moves()` to `gcode.request_restart`.
* `stepper.py` calls: `mcu.create_oid, register_config_callback, add_config_cmd, lookup_command, alloc_command_queue, get_printer, print_time_to_clock, seconds_to_clock, clock_to_print_time, get_query_slot, get_enumerations`, `printer.register_event_handler('klippy:connect')`, `config.getlists('gear_ratio', (), seps=(':', ','), count=2, parser=float)`. The `MCU_trsync/TriggerDispatch` classes in `mcu.py` call `stepper.get_mcu(), get_oid(), get_name(), note_homing_end()`.
* `mcu.MCUConfigHelper._connect` calls `motion_queuing.setup_mcu_movequeue(mcu, serialqueue, move_count)`, and `mcu.calibrate_clock` is called per flush by `extras/motion_queuing.py`.

---

## 3. The reactor's concurrency model (exact)

### 3.1 Objects

* **One OS thread** runs everything except the serial reader thread, the logging thread and `multiprocessing` children. The "main thread" is the thread that calls `Reactor.run()`.
* `ReactorTimer(callback, waketime)` with `timer_is_running`. A timer callback receives `eventtime` and returns the next `waketime` (`NEVER` to sleep, `NOW`=0 to run on the next pass).
* `ReactorCompletion`: `result` (sentinel until completed), `waiting` list of greenlets. `complete(result)` stores the result, then `update_timer(g.timer, NOW)` for each waiter. `wait(waketime, waketime_result)` appends the current greenlet, `pause(waketime)`, removes it, and returns `waketime_result` if still incomplete. A completion can be completed repeatedly (last result wins) and nothing prevents `complete` before `wait` (then `wait` returns immediately).
* `ReactorCallback`: a one-shot timer. `invoke` unregisters its timer first, then calls `callback(eventtime)`, then `completion.complete(result)` and returns `NEVER`.
* `ReactorMutex`: FIFO of greenlets. `__enter__`: if free, take it; else queue and loop `pause(NEVER)` until `next_pending and queue[0] is g`. `__exit__`: if no waiters, free it; else `next_pending=True` and `update_timer(queue[0].timer, NOW)`. Ownership is handed over directly (the lock never becomes free in between). `test()` reads `is_locked`. `lock`/`unlock` alias `__enter__`/`__exit__`. It is not re-entrant: a greenlet that locks twice deadlocks itself (this is why `run_script_from_command` has no mutex variant and `run_script` is only for entry points).
* `ReactorPreventPause`: counter `_prevent_pause_count`. `pause()` raises `ReactorError("Internal error - reactor pause disabled")` while the counter is non-zero (only when the reactor is running; a pause while not running goes to `_sys_pause`, which sleeps). The counter is reset to 0 at `run()`.
* `register_fd` stores handlers in `_fds`; `Poll` registers `POLLIN|POLLHUP`; `set_fd_wake(h, readable, writeable)` modifies flags (the webhooks client uses `(False, True)` to stop reading while the send buffer is blocked). `_READ = POLLIN|POLLHUP`, `_WRITE = POLLOUT`. Read callback first, then write callback, per event. An fd that is no longer registered maps to a dummy handler.

### 3.2 Dispatch loop

`run()` loops `while self._process`: create a `ReactorGreenlet(run=self._dispatch_loop)`, record it in `_all_greenlets` and `_g_dispatch`, `switch()` into it. Control comes back to `run()` only when the dispatcher finishes (`end()`) or when a dispatcher pauses with no cached greenlet available.

`_dispatch_loop`: `busy=True`; loop: `timeout = _check_timers(eventtime, busy)`; `busy=False`; `poll(ceil(timeout*1000) ms)`; `eventtime = monotonic()`; if fds ready: `busy=True`, `eventtime = _check_fds(...)`.

`_check_timers(eventtime, busy)`:
1. Fork instrumentation (stall detection from the gap between consecutive calls).
2. `if eventtime < _next_timer`: if `busy` return timeout 0; else run `_check_gc(eventtime)` (only if `gc_checking`, `gc.get_count()[0] >= 700`; picks level 0/1/2 from the counts; if it ran, return 0); else return `min(1.0, max(0.001, _next_timer - eventtime))`.
3. Otherwise set `_next_timer = NEVER`, then iterate over `self._timers` (a copy-on-write list, so callbacks may register/unregister timers during iteration): for each timer with `eventtime >= waketime`: set `waketime=NEVER`, `timer_is_running=True`, call the callback, store the returned waketime, `timer_is_running=False`. After each callback, if `_g_dispatch` changed (the callback paused and a new dispatcher took over), fold the returned waketime into `_next_timer`, call `_end_greenlet(old_dispatcher)` and return 0. Otherwise fold `waketime` into `_next_timer`. Timers fire at most once per pass, in registration order.
4. `update_timer(t, w)` is ignored when `t.timer_is_running` (a callback cannot re-arm itself that way; it must return the new waketime). `register_timer` and `update_timer` lower `_next_timer` eagerly; nothing ever raises it except the pass in step 3.

`_check_fds`: for each ready `(fd, event)`: read callback, then write callback; after each call, if the dispatcher changed, `_end_greenlet(old)` and return `monotonic()`.

### 3.3 `pause(waketime)` and greenlet hand-off

* If the reactor is not running (`_g_dispatch is None`): sleep until `waketime` with `time.sleep` and return `monotonic()`. This is the path used before `run()` and after `finalize()`.
* If called from a greenlet that is not the current dispatcher (a callback that already paused once and was resumed): `return self._g_dispatch.switch(waketime)`. The dispatcher's `t.callback(...)` call (which was `g.switch(eventtime)`) returns `waketime`, which becomes the greenlet timer's next wake time. Resumption later returns `eventtime` from `pause`.
* If called from the dispatcher greenlet itself (the normal first pause inside a timer/fd/callback): create a per-greenlet timer `g.timer = register_timer(g.switch, waketime)`, set `_next_timer = NOW`, then either switch into a cached dispatcher greenlet (`g_next.switch()`) or switch to the parent (`run()`), which creates a fresh dispatcher greenlet. The paused callback stays suspended in the middle of its Python stack.
* When a callback that paused finally returns, its original frame (`_check_timers`/`_check_fds`) notices `_g_dispatch` differs, calls `_end_greenlet(g)`: unregister `g.timer`, push `g` on `_cached_dispatch_greenlets`, and `self._g_dispatch.switch(NEVER)` back to the active dispatcher; the `NEVER` return value becomes the waketime of the (now unregistered) greenlet timer. When someone later pauses with `g` cached, `g` resumes inside `_end_greenlet`, sets `_g_dispatch = g` and returns to its loop.
* Consequences a port must reproduce:
  1. Every callback runs inline on the dispatcher; there is no preemption. Two statements without an intervening `pause` (or `mutex.__enter__` that blocks, or `completion.wait`, or `time`-consuming call into C that releases nothing) never interleave with other callbacks.
  2. A paused callback does not block timers or fds. Its resume is itself a timer, so resumption order follows the timer list and the eventtime.
  3. Stacks of paused callbacks are unbounded in depth only by the number of concurrently paused callbacks; each is a full Python stack. The set of `ReactorGreenlet`s only grows until `finalize()`.
  4. `pause()` returns the `eventtime` of the dispatch pass that resumed it (not `monotonic()` at return).
  5. Exceptions escaping a callback propagate to `run()` (all greenlets share the main greenlet as parent) and leave other paused greenlets suspended; `Printer.run` handles that by re-running the reactor once with a shutdown callback.
  6. `end()` just clears `_process`; callbacks in progress finish their current pass (e.g. `GCodeIO._process_data` keeps processing the lines it already read after `request_exit`).

### 3.4 Cross-thread entry points

* `register_async_callback(cb, waketime)` and `async_complete(completion, result)` put work in a `queue.Queue` and write one byte to a non-blocking self-pipe created lazily in `run()`. `_got_pipe_signal` (a registered read fd) drains the pipe, then drains the queue, calling `ReactorCallback(...)` constructors or `completion.complete(result)` on the main thread. A write failure on the pipe is ignored. Users: `Printer.invoke_async_shutdown`, serial reader (`pending_notifications`), `RetryAsyncCommand`, `MCU_trsync._handle_trsync_state`, buttons, replicape, temperature_probe.
* `reactor.completion().complete()` from another thread is unsupported.

### 3.5 Who relies on cooperative scheduling

* `assert_no_pause()` regions (the pause call raises if violated): `klippy:ready` handlers (`klippy.py`), `klippy:shutdown`/`analyze_shutdown` handlers, `toolhead._process_lookahead` trapq append loop, `webhooks._do_query` (consistent `get_status` snapshot), `extras/gcode_macro.py` status cache, `extras/motion_queuing.py` flush callbacks, `extras/statistics.py` stats callbacks.
* G-code mutex (`gcode.get_mutex()`): `GCodeIO._process_data`, `run_script` (webhooks `gcode/script`, restart endpoints), `query_endstops` web handler, `bed_mesh` dump, `tmc` phase offset, `idle_timeout` and `virtual_sdcard` poll `mutex.test()` to avoid starting work while G-code runs; `virtual_sdcard` yields with `pause(+0.050)` when the mutex is taken.
* `GCodeIO._process_data` re-entrancy (section 4): while the first invocation is paused inside a command, the fd fires again; the second invocation sees `is_processing_data`, appends to `self.pending_commands`, runs the out-of-order `M112` check, and returns. The first invocation then picks up `self.pending_commands`. This is relied on to make `M112` work during a long `G4`/heater wait.
* `RetryAsyncCommand`/`SerialRetryCommand` call `reactor.pause` while the serial thread delivers; the handler writes `last_params` and the retry loop re-checks after waking.
* Many modules mutate shared state between `pause` calls assuming exclusivity (heater control, homing, probing, `ToolHead` lookahead). A port cannot make those modules thread-safe incrementally; it must keep a single logical thread with explicit yield points.
* Recommendation: keep "one runnable context at a time" and switch only at `pause`, `completion.wait`, mutex contention and fd/timer return. Stackful coroutines (or OS threads handing off a baton) preserve this without colouring every function `async`. A pure `async/await` port forces every caller of `pause` up the stack (gcode handlers, homing, probing, toolhead `wait_moves`, TMC reads) to become async.

---

## 4. Config-file semantics a port must reproduce

### 4.1 Reading

1. `read_config_file`: open in text mode, `\r\n` to `\n`; failure logs the exception and raises `Unable to open config file <name>`.
2. `ConfigAutoSave.load_main_config`: split at the first `AUTOSAVE_HEADER` (the exact text `\n#*# <---------------------- SAVE_CONFIG ---------------------->\n#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n#*#\n`). Corruption rules (return the whole file as regular data and empty autosave, with a WARNING): a `\n#*# ` line before the header, a second header inside the autosave data, or any autosave line that does not start with `#*#` (or starts with `#*#` followed by something other than a space and has length >= 4). Autosave lines have the first four characters stripped.
3. The regular part goes through `build_fileconfig_with_includes`; the autosave part goes through `_strip_duplicates(autosave, regular_config)` (every autosave field that already exists in the regular config is commented out, so a hand-written value wins at load time) and is parsed without includes into `self.fileconfig`; then it is appended onto the regular config with name `*AUTOSAVE*` (it overrides earlier values for options not stripped).
4. Parser: `RawConfigParser(strict=False, inline_comment_prefixes=(';','#'))`: no interpolation; keys lowercased (`optionxform`); delimiters `=` and `:`; duplicates allowed (later value wins, sections merge); lines starting with whitespace continue the previous value; empty lines inside values are kept; default section `DEFAULT` applies to all sections; `;` and `#` start inline comments only when preceded by whitespace and start full-line comments at column 0. Option names such as `gcode :` are valid (name is trimmed). Section header regex is the stdlib `SECTCRE` (Python 3.8: `\[(?P<header>[^]]+)\]`) `[INFERENCE from memory of the stdlib]`.
5. Pre-processing before configparser: every line is cut at the first `#` anywhere (even mid-value, even without preceding whitespace); this is done in `_parse_config` and again in `append_fileconfig`.
6. Includes (main config only): a line that matches `SECTCRE` at column 0 with header starting `include ` (after the first `#`-strip) is an include; the spec is `header[8:].strip()`. Path is `os.path.join(dirname(including file), spec)` (absolute specs win), expanded with `glob.glob`, sorted, each included file parsed recursively. No match and no glob magic: `Include file '<path>' does not exist`. A wildcard with no match is fine. Recursion detection by `abspath` set: `Recursive include of config file '<name>'`. Lines between includes are buffered and parsed as a unit so later content overrides includes linearly. `read_config(filename)` (used by extras for secondary files) does not process includes.

### 4.2 Typed access (`ConfigWrapper._get_wrapper`)

* Missing option: if a default is given (including `None`), return it; defaults that are not `None` are recorded in `access_tracking[(section.lower(), option.lower())]`; with no default: `Option '<o>' in section '<s>' must be specified`.
* Present: parse with the parser; any `configparser.Error` passes through, anything else becomes `Unable to parse option '<o>' in section '<s>'`. If `note_valid`, store the parsed value in `access_tracking` (this is what the unused check and `settings` use).
* Bounds, in this order: `minval` -> `Option '<o>' in section '<s>' must have minimum of <minval>`, `maxval` -> `... maximum of ...`, `above` -> `... must be above <above>`, `below` -> `... must be below <below>`. The limit is formatted with `%s`, i.e. Python `str()` (floats print as `0.0`, `1e-05`, `1000000.0`; integers as digits).
* `get` returns the raw string; `getint` uses `int(str)` (accepts `+5`, `-3`, `1_000`, surrounding spaces; rejects `1.0`, `0x10`); `getfloat` uses `float(str)` (accepts `inf`, `nan`, `1e3`, `.5`, `5.`, `1_0`); `getboolean` accepts `1 yes true on` and `0 no false off`, case-insensitive, anything else is "Unable to parse".
* `getchoice(option, choices, default)`: `choices` list becomes an identity dict; if the first key is an `int` the option is read with `getint`, otherwise `get`; unknown value: `Choice '<c>' for option '<o>' in section '<s>' is not a valid choice`; returns `choices[c]`.
* `getlists(option, default, seps, count, parser)`: nested separators from the last to the first; whitespace is stripped; empty text gives an empty list; nested levels drop empty parts; the `count` check applies at the innermost level only: `Option '<o>' in section '<s>' must have <n> elements`. Returns tuples. `getlist/getintlist/getfloatlist` wrap it.
* `get_prefix_sections(prefix)` (all sections starting with `prefix`, in file order; `''` lists all), `get_prefix_options(prefix)` (not tracked as used), `getsection(name)`, `has_section`, `deprecate(option, value=None)` (adds a warning only if the option is present).

### 4.3 Unused-option check

`check_unused_options` is a hard error, not a warning: for each section in the final parse (lowercased), it must be an existing printer object name or appear in `access_tracking`, else `Section '<s>' is not a valid config section`; for each option (lowercased) the pair must be in `access_tracking` or in the autosave option set, else `Option '<o>' is not valid in section '<s>'`. These are `configparser.Error`s, so `klippy._connect` reports them as the printer state message with the `RESTART` hint. Notes: defaults that are `None` are never tracked, so an option read only through `get(..., None)` and present in the file is rejected unless some other access tracks it; `note_valid=False` reads do not count; section names with upper case characters only pass if an access recorded them lowercased.
Non-fatal warnings are separate: `PrinterConfig.deprecate*`/`runtime_warning` append dicts to `status_warnings` (deduplicated by the tuple of items) and `runtime_warning` also logs at WARNING.

### 4.4 Autosave block and `SAVE_CONFIG`

* `ConfigAutoSave.set(section, option, value)` stores `str(value)` into the autosave config, updates `status_save_pending` (copy-on-write) and logs `save_config: set [%s] %s = %s`. `remove_section` marks a pending removal as `None`.
* `SAVE_CONFIG` (no-op if there is nothing saved): serialise the autosave config with `RawConfigParser.write` (format `[section]\nkey = value\n\n`, continuation lines prefixed by `\n\t`), prefix every line with `#*# ` and `.strip()` it, put the header first; re-read the current file; strip the old autosave block (`_find_autosave_data`), comment out regular fields that the new autosave defines (`_strip_duplicates`, which also comments out indented continuation lines of those fields); append; re-validate (`Existing config autosave is corrupted. Can't complete SAVE_CONFIG`, `Unable to parse existing config on SAVE_CONFIG`, `Unable to read existing config on SAVE_CONFIG`); reject values that conflict with included files (`SAVE_CONFIG section '<s>' option '<o>' conflicts with included value`); write `<name>_autosave.cfg`, rename the original to `<name>-%Y%m%d_%H%M%S.cfg` (for names ending `.cfg`), rename the temp file into place; failure `Unable to write config file during SAVE_CONFIG`; then `gcode.request_restart('restart')`.
* `_strip_duplicates` details: a line is cut at `[#;]...`, blank lines are skipped, lines starting with whitespace belong to the previous field, a line starting with `[` sets the section to `line[1:-1].strip()`, otherwise the field name is the leading `[A-Za-z0-9_]*` run.

### 4.5 What the test and fork configs exercise

`test/klippy/*.cfg` and `config/example-*.cfg` cover typed accessors, prefix sections (`stepper_*`, `extruder1`, `tmc2130 stepper_x`, `mcu extboard`) and two autosave blocks. The fork's `config/ender-3-v3-ke/printer.cfg` adds includes with absolute paths, inline `;` comments, and `key :` spacing.

---

## 5. G-code parsing, dispatch, errors and batch mode

### 5.1 Line parsing (`_process_commands`)

For each input line: `line = origline = line.strip()`; cut at the first `;`; `parts = re.compile('([A-Z_]+|[A-Z*])').split(line.upper())`. If `parts[0]+parts[1] == 'N'` (a line number) then `cmd = ''.join(parts[3:5]).strip()`, else `cmd = ''.join(parts[:3]).strip()`. `params = {parts[i]: parts[i+1].strip() for i in range(1, len(parts), 2)}`.
Examples: `G1 X10 Y-2` -> `cmd='G1'`, `params={'G':'1','X':'10','Y':'-2'}`; `M117 1 2` -> `cmd='M117 1 2'` (contains a space, handled in `cmd_default`); `set_led led=x` -> `cmd='SET_LED'` and a garbage `params` (the extended-command wrapper re-parses).
The command object receives the original (case preserved) line, so `get_commandline()` and `get_raw_command_parameters()` keep case. `get_raw_command_parameters` strips a leading line number and a trailing `*<digits>` checksum, and one separating space.

### 5.2 Registration and dispatch

* `is_traditional_gcode(cmd)`: first token upper-cased parses as `letter + float`, with `cmd[0].isupper() and cmd[1].isdigit()` (`G1`, `M104`, `T0`).
* Non-traditional names must be upper-case, alphanumeric after replacing `_`, not start with a digit, and not have a digit in position 1; otherwise `Can't register '<cmd>' as it is an invalid name`. Duplicate registration: `gcode command <cmd> already registered`. Non-traditional handlers are wrapped so that `_get_extended_params` runs first: `shlex.shlex(raw, posix=True)` with `whitespace_split=True`, `commenters='#;'`; each token is `key=value` (`split('=', 1)`), key upper-cased, value untouched; any `ValueError` (missing `=`, unmatched quote) raises `Malformed command '<commandline>'`. The params dict of the `GCodeCommand` is cleared and replaced.
* Two handler tables: `ready_gcode_handlers` (everything) and `base_gcode_handlers` (only commands registered with `when_not_ready=True`); `gcode_handlers` points to base until `klippy:ready`, back to base on `klippy:shutdown`. Unknown command -> `cmd_default`.
* Handler call: `try handler(gcmd)`; `except CommandError` -> `_respond_error(str(e))`, `send_event("gcode:command_error")`, re-raise if `need_ack` is False; `except Exception` -> log `Internal error on command:"<cmd>"`, `printer.invoke_shutdown(msg)`, respond error, re-raise if `need_ack` is False. After either branch `gcmd.ack()` still runs, so an error line is followed by `ok` on the pty.
* `cmd_default` order: `M105` -> ack `T:0`; `M21` -> ignore; printer not ready -> raise the printer's state message as a command error; empty command -> debug log of the line; command containing a space and starting `M117`/`M118`/`M23` -> redispatch to the real handler if one exists; `M140`/`M104` with S=0 -> ignore; `M107`, or `M106` with S=0 or in file-input mode -> ignore; otherwise respond `Unknown command:"<cmd>"` (info line, not an error).
* `HELP` output: `Printer is not ready - not all commands available.` (if not ready), `Available extended commands:`, then `%-10s: %s` for each handler that has help, sorted by name.
* `STATUS` when ready: `// Klipper state: Ready`; when not ready: raises the state message plus `\nKlipper state: Not ready`.
* `RESTART`/`FIRMWARE_RESTART` -> `request_restart`: when ready, send `gcode:request_restart(print_time)`, `toolhead.dwell(0.500)`, `toolhead.wait_moves()`; then `printer.request_exit(result)`.

### 5.3 Mux commands

`register_mux_command(cmd, key, value, func, desc)`: the first call registers `cmd` with a dispatcher lambda and stores `(key, {value: func})`; later calls must use the same `key` (`mux command <cmd> <key> <value> may have only one key (<prev_key>)`) and a new `value` (`mux command ... already registered (...)`). The help text comes from the first registrant only. `_cmd_mux`: if `None` is a registered value the key parameter is optional (`gcmd.get(key, None)`), else it is required (`gcmd.get(key)` raises `Error on '<line>': missing <KEY>`). If the value is unknown: error `The value '<v>' is not valid for <KEY>` followed by either `. Did you mean '<value>'?` (when the given text is a non-empty substring of a registered value; the last matching value wins) or `. Options: 'a', 'b'` (registered non-None values in registration order). Used by `SET_PIN`-style commands throughout the extras (`NAME=`, `STEPPER=`, `HEATER=`, `FAN=`...).

### 5.4 `GCodeCommand.get`

Missing: `Error on '<commandline>': missing <name>`. Unparseable: `Error on '<commandline>': unable to parse <value>`. Bound errors: `... <name> must have minimum of <n>`, `... maximum of ...`, `... must be above ...`, `... must be below ...`. `get_int` uses `int()` (so `5.0` fails), `get_float` uses `float()` (accepts `nan`, `inf`; bounds checks pass with `nan`).

### 5.5 Response formatting

* `respond_info(msg, log=True)`: log at INFO if `log`; split `msg.strip()` by line, strip each, emit `// ` + lines joined by `\n// ` in a single output call.
* `respond_raw(msg)`: calls every registered output handler synchronously: the pty writer and the webhooks `gcode/subscribe_output` subscribers.
* `_respond_error(msg)`: WARNING log; if multi-line, `respond_info` of the full text without logging; then `respond_raw('!! ' + first_line.strip())`; in file-input mode also `printer.request_exit('error_exit')`.
* `ack(msg)`: only when the command came through the pty (`need_ack=True`); emits `ok` or `ok <msg>`; returns whether it acked.
* State notices: `// Klipper state: Ready|Shutdown|Disconnect`.

### 5.6 Input paths and batch/file mode

* Interactive: `util.create_pty('/tmp/printer')` (override with `-I`), master fd registered with the reactor; `GCodeIO._process_data` reads up to 4096 bytes, decodes UTF-8 (decode/read errors are logged as `Read g-code` and the chunk is dropped), splits on `\n` keeping the trailing partial line, extends `pending_commands`. If fewer than 20 are pending it scans the newly read lines with `^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)` and calls `cmd_M112` immediately (out-of-order emergency stop). If a processing pass is already running (`is_processing_data`), and 20 or more commands are pending, the fd is unregistered (back-pressure) and re-registered afterwards; the call returns. Otherwise it sets `is_processing_data`, loops `while pending_commands:` swapping in a fresh `self.pending_commands`, running the batch inside `with gcode_mutex: _process_commands(batch)`. Output handler writes `msg + "\n"` to the pty; a write error disables output (`pipe_is_active=False`) until new input arrives.
* Batch/file mode (`-i file`): the output handler is not registered (replies are dropped except to webhooks); the file fd is registered on `klippy:ready`; a regular file is always readable so the loop runs until EOF. At EOF (`os.read` returns empty) `request_restart('exit')` is invoked (unless a pass is in progress), an empty line is appended so the last partial line flushes, the fd is unregistered. Any error response, any `klippy:shutdown` (including `M112`), and any non-ready state set while `debuginput` is set call `request_exit('error_exit')`; `request_exit` keeps the first result. Remaining lines already read in the current chunk still execute after the error. Process exit code is 255 for `error_exit`, else 0.
* `scripts/test_klippy.py` contract: parses `*.test` (`CONFIG`, `DICTIONARY`, `GCODE`, `SHOULD_FAIL`, `#` comments, everything else is G-code, multiple `CONFIG` lines make multiple runs), launches `python klippy/klippy.py <cfg> -i <gcode> -o _test_output -v [-l _test_.log] -d <dict>...`, passes if exit status matches `SHOULD_FAIL`. It never inspects the output file. Parity of the port therefore needs an extra oracle (section 8).

---

## 6. MCU protocol layer

### 6.1 Layers and ownership

```
extras / stepper / heaters
   -> mcu.py  (text commands, wrappers, config/CRC, restart, stats)
      -> msgproto.py  (dictionary, encode/decode)         [Python]
      -> serialhdl.py (connection, bg thread, retries)     [Python + cffi]
         -> chelper serialqueue.c (framing, windowing, retransmit, scheduling)   [C, thread]
            -> fd: tty / pipe / SocketCAN / debug file
      -> clocksync.py (estimates clocks, feeds serialqueue_set_clock_est)
```

The steppersync/stepcompress/trdispatch C code also pushes messages into the same serialqueue (`steppersync_setup_movequeue(ss, sq, move_num)`, `trdispatch_mcu_alloc(td, sq, cq, ...)`), bypassing Python.

### 6.2 Data dictionary and identify

* Boot messages (fixed IDs): `identify_response offset=%u data=%.*s` = 0, `identify offset=%u count=%c` = 1.
* `SerialReader._get_identify_data` loops: send `identify offset=<len(so far)> count=40`, expect `identify_response`; if the returned offset equals the bytes received so far, append `data`; empty data ends. Whole identify is bounded by `completion.wait(monotonic()+5.)` -> `Timeout on connect`, `disconnect()`, retry loop in `connect_*` (the loop gives up 90 s after it started: `Unable to connect`).
* `MessageParser.process_identify(data, decompress=True)`: `zlib.decompress`, `json.loads`, then: `enumerations` (name -> {key: int} or {key: [start, count]}; ranges expand the trailing digits of the key, e.g. `PA0: [0,16]` yields `PA0..PA15`), `commands` {format: id}, `responses` {format: id}, `output` {format: id}, `config` (constants; values may be strings or ints), `version`, `build_versions`. A message is classed `command` if its id is in `commands.values()`, `output` if in `output.values()`, else response. Failures log `process_identify error` and raise `Error during identify: ...`.
* A parameter named `X` or ending `_X`, where `X` is an enumeration name (`pin`, `spi_bus`, `i2c_bus`, `uart_bus`, ...), is wrapped in `Enumeration`: the host sends the string name, the wire carries the integer; unknown names raise `enumeration_error` ("Unknown value '<v>' in enumeration '<e>'"), which `mcu._send_cfg_init_commands` turns into the config error `Pin '<v>' is not a valid pin name on mcu '<name>'` when the enumeration is `pin`. Parsing a value without a name yields `?<n>`.
* Format strings: `name p1=%u p2=%c ...`. Types: `%u %i %hu %hi %c` -> variable-length integer; `%s` (length byte + bytes), `%.*s` (progmem buffer), `%*s` (buffer) -> byte strings. Output (`#output`) formats use Python `%`-style printing after `%u %i %hu %hi %c %.*s %*s` are converted to `%s`.
* `lookup_command(fmt)` requires an exact format match (`Command format mismatch: <a> vs <b>`, `Unknown command: <name>`). `CommandWrapper` uses the message id (`msgtag = id & 0xffffffff`) for the C trdispatch.
* Debug-output mode reads the dictionary from `-d` (uncompressed JSON) via `process_identify(dict, decompress=False)`.

### 6.3 Message encoding

* Integer encoding (`PT_uint32.encode`): signed variable-length base-128, big-endian groups of 7 bits, continuation bit `0x80` on all but the last byte; the group count is chosen from thresholds `v >= 0xc000000 or v < -0x4000000` (5 bytes), `0x180000/-0x80000` (4), `0x3000/-0x1000` (3), `0x60/-0x20` (2), else 1. Decoding sign-extends when the first byte has `(c & 0x60) == 0x60`; unsigned types mask to 32 bits, signed types do not. 16-bit and 8-bit types use the same code (only `max_length` differs: 3 and 2); there is no truncation to 16 or 8 bits on parse.
* Message body: msgid (encoded as an int32) followed by each parameter in order.
* Block framing (`encode_msgblock`/`check_packet`/C `serialqueue.c`): `[len][seq][payload...][crc_hi][crc_lo][0x7e]`; `len` includes header and trailer (min 5, max 64); `seq` = `0x10 | (seq & 0x0f)`; CRC16-CCITT (init `0xffff`, the Klipper bit-twiddle variant in `crc16_ccitt`) over `len, seq, payload`; trailer sync byte `0x7e`. `check_packet` returns 0 (need more), -1 (garbage, caller discards one byte/up to the next `0x7e`), or the block length. One block can carry several commands; `MessageParser.dump(block)` splits them. Payload max is 59 bytes.
* Text commands: `create_command("name a=1 b=0x10 c=PA3 d=0a1b")`: tokens split on whitespace, `key=value` split once on `=`, integers via `int(v, 0)`, dynamic strings are hex digits parsed into bytes, enumeration values stay strings. Errors: `Unknown command: <name>`, `Unable to extract params from: <name>`, `Unable to encode: <name>`.
* `create_dummy_response(name, params)` builds an all-zero (or empty string) response and round-trips it through parse (file-output mode).

### 6.4 Transports

* **UART** (`connect_uart`): pyserial `Serial(baudrate=baud, timeout=0, exclusive=True)`, `rts` set before open (cheetah boards need it deasserted), then `stk500v2_leave` (clear HUPCL, 2400 baud dummy read, 115200, pause 0.1 s, drain, write `1b 01 00 01 0e 11 04`, pause 0.05 s, read, restore baud). `serialqueue_alloc(fd, 'u', 0, name)` then identify. Open errors retry every 5 s.
* **Pipe** (`connect_pipe`): `os.open(path, O_RDWR|O_NOCTTY)`, `os.fdopen(fd,'rb+',0)`; used for the Linux process MCU (`/tmp/klipper_host_*`, `/dev/rpmsg_*`). No baud configuration, `serial_fd_type 'u'`.
* **CAN** (`connect_canbus`): `txid = nodeid*2 + 256`; receive filter id `txid+1`; admin frame id `0x3f0`, data `[0x01, uuid(6 bytes big-endian), nodeid]` (`CMD_SET_NODEID`) sent with python-can on `socketcan`; the python-can bus object is handed to C with type `'c'` and `client_id=txid`. After identify, the host sends `get_canbus_id` and compares `canbus_uuid`; mismatch disconnects and retries. In C, writes split into 8-byte `struct can_frame`s with `can_id = client_id`; reads accept only `can_id == client_id+1`. The wire frequency comes from `CANBUS_FREQUENCY` and the C code derives per-packet bit costs (`CANBUS_PACKET_BITS`, `CANBUS_IFS_BITS`).
* **Debug file** (`connect_file`): type `'f'`; the C queue writes encoded blocks to the output file; ack tracking is disabled (`receive_seq = -1`, `rto = NEVER`), so `send_wait_ack` becomes plain `send` and queries are answered by `DummyResponse`.
* After identify, `SERIAL_BAUD` (or `CANBUS_FREQUENCY` for CAN) goes to `serialqueue_set_wire_frequency` and `RECEIVE_WINDOW` to `serialqueue_set_receive_window` when present.

### 6.5 How commands flow into the C `serialqueue`

* `CommandWrapper.send(data, minclock, reqclock)` encodes to a list of ints and calls `serialqueue_send(sq, cmd_queue, cmd, len, minclock, reqclock, notify_id=0)`; cffi turns the Python list into a temporary `uint8_t[]`. `send_wait_ack` allocates `notify_id`, registers a `ReactorCompletion` in `pending_notifications`, and waits; the reader thread returns an ack as a `pull_queue_message` with that `notify_id` and the sent/receive times, and `async_complete`s the completion. On disconnect all pending completions complete with `None` and the waiter raises `Serial connection closed`.
* `minclock`: do not transmit before the MCU clock estimate reaches this value (order-preserving fence). `reqclock`: requested MCU clock at which the command should reach the MCU; the C scheduler uses it to decide when to flush and to order commands across queues (`BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000` for low-priority traffic). Constants in `serialqueue.c`: `MIN_RTO 0.025`, `MAX_RTO 5.0`, `MAX_PENDING_BLOCKS 12`, `MIN_REQTIME_DELTA 0.100`, `MIN_BACKGROUND_DELTA 0.005`, `IDLE_QUERY_TIME 1.0`. A command queue (`serialqueue_alloc_commandqueue`) is a FIFO; different queues are interleaved by `reqclock`. The default queue is `SerialReader.default_cmd_queue`.
* Reliable delivery is the C code's job: sequence numbers, RTT estimate (`srtt`/`rttvar`, `rto` doubled on retransmit), sync-byte-prefixed retransmit buffers, duplicate-ack NAK handling, `tcflush(TCOFLUSH)` before retransmit on UART.
* Responses: the reader thread loops on `serialqueue_pull`; each message is parsed by `MessageParser.parse`, annotated with `#name, #sent_time, #receive_time`, and dispatched through `handlers[(name, oid)]` under `self.lock` (an unmatched pair falls to `handle_default`, which logs `got <params>` at WARNING). `register_response(None, ...)` deletes the handler. The logs from the Ender show this warning for `analog_in_state` messages that arrive from an already-configured MCU before Klipper registered its handlers.
* C also owns "fast readers" (`serialqueue_add_fastreader`, not used by this slice) and step queues (`steppersync_*`), so the Rust port of `serialhdl` must expose the same C ABI or take over the C code.
* Statistics: `serialqueue_get_stats(buf)` returns `key=value` pairs separated by spaces; `MCUStatsHelper.stats` concatenates `mcu_awake=%.03f mcu_task_avg=%.06f mcu_task_stddev=%.06f`, the serial stats and the clocksync stats, splits on whitespace and `=`, and converts to float if the text contains `.`, else int. Every token must contain `=`. The same string is emitted by `extras/statistics.py` into the log as the periodic `Stats` line.
* Dumps on shutdown: `serialqueue_extract_old(sentq, buf, 1024)` returns recent sent/received blocks; `MessageParser.dump` renders them (`Sent %d %f %f %d: ...`, `Receive: ...`).

### 6.6 Clock synchronisation

* Setup (`ClockSync.connect`): read `CLOCK_FREQ`; `get_uptime` gives `high`/`clock`, so `last_clock=(high<<32)|clock`, `clock_avg=last_clock`, `time_avg=#sent_time`, `clock_est=(time_avg, clock_avg, mcu_freq)`, `prediction_variance=(0.001*mcu_freq)^2`. Then 8 rounds of `pause(+0.050)`, `get_clock`, `_handle_clock`. Afterwards a dedicated command queue is allocated, the `clock` response is registered, and the periodic timer starts (period `0.9839` s).
* `_handle_clock` (serial thread): extend the 32-bit `clock` to 64 bits against `last_clock` (`(clock - last) & 0xffffffff`); `half_rtt = 0.5*(receive_time - sent_time)`; track minimum RTT with ageing `RTT_AGE = 10e-6/3600` per second; outlier filter: reject if the squared error exceeds `25*prediction_variance` and `(0.0005*mcu_freq)^2` (positive errors within 10 s of the last good prediction are ignored, otherwise the variance is reset to `(0.001*mcu_freq)^2`); exponential-decay regression with `DECAY = 1/30` over `time_avg, clock_avg, time_variance, clock_covariance`; `new_freq = covariance/variance`; calls `serialqueue_set_clock_est(new_freq, time_avg + 0.001, int(clock_avg - 3*pred_stddev))`; stores `clock_est = (time_avg + min_half_rtt, clock_avg, new_freq)`.
* Conversions: `print_time_to_clock = int(print_time*mcu_freq)`, `clock_to_print_time = clock/mcu_freq`, `get_clock(t) = int(clock + (t - sample_time)*freq)`, `clock32_to_clock64` uses the signed 32-bit delta from `last_clock`.
* Liveness: `queries_pending` is incremented by each timer-driven `get_clock` and zeroed by each reply; `is_active()` is `queries_pending <= 4`; `MCU.calibrate_clock` (called by `motion_queuing` on every flush) triggers `check_timeout`, which on the first inactive observation records `[DEBUG-motion]` data and invokes `printer.invoke_shutdown("Lost communication with MCU '<name>'")`.
* Secondary MCUs (`SecondarySync`): the `clock_adj=(offset, freq)` pair maps the secondary clock onto the main MCU print time; `connect` seeds it from the main sync's current estimate; `calibrate_clock(print_time, eventtime)` is called from the flush path and solves for a frequency that lands on the main time base `4 s` (min) ahead (`sync2_print_time = max(sync1 + 4., last_sync_time, print_time + 2.5*(print_time - est_print_time))`); results flow to `steppersync_set_time(ss, offset, freq)`.
* File mode: `clock_est = (0, 0, mcu_freq)` and `serialqueue_set_clock_est(1e12 [or mcu_freq if pace], now, 0)`.

### 6.7 MCU lifecycle in `mcu.py`

1. `klippy:mcu_identify` -> `MCUConnectHelper._mcu_identify`: attach (file/uart/pipe/can), log `Loaded MCU...`, `MCU ... config:`, register `shutdown`, `is_shutdown` (both to `_handle_shutdown`) and `starting` handlers, look up `emergency_stop`. `MCUConfigHelper._mcu_identify` applies `RESERVE_PINS_*` constants to the pin resolver and checks `MAX_NOMINAL_DURATION * freq <= MAX_SCHEDULE_TICKS`.
2. `klippy:connect` -> `MCUConfigHelper._connect`: `get_config` query. If not configured: `check_restart_on_send_config` (rpi_usb), finalize config (run callbacks, prepend `allocate_oids count=N`, pin-resolve, CRC, append `finalize_config crc=N`), send `config_cmds + init_cmds`. If configured: refuse when `start_reason == 'firmware_restart'` (`Failed automated reset`), finalize, compare CRC (`check_restart_on_crc_mismatch` or `MCU '<n>' CRC does not match config`), send `restart_cmds + init_cmds`. Query `get_config` again (`Unable to configure MCU`), run post-init callbacks (`AsyncResponseWrapper` registrations), set up the step move queue from `move_count - reserved_move_slots`, set rollover info for the log.
3. Restart policies (`MCURestartHelper`): `command` sends `reset` or `config_reset` (after `force_local_shutdown`), `arduino` toggles DTR, `cheetah` runs the DTR/RTS dance, `rpi_usb` power-cycles the hub with `chelper.run_hub_ctrl` (`sudo hub-ctrl`). Auto-restart mechanism: `printer.request_exit('firmware_restart')`, pause 2 s, raise `Attempt MCU '<n>' restart failed` if the host is still running. CAN bridge MCUs (`CANBUS_BRIDGE`) restart only via the forced path.
4. Shutdown: `shutdown` or `is_shutdown` response on the reader thread -> fork hook, then first-time only: set `_is_shutdown`, remember `static_string_id` and optional `clock`, `printer.invoke_async_shutdown("MCU shutdown", {reason, mcu, event_type, shutdown_clock})`. `klippy:shutdown` makes each MCU send `emergency_stop` (once, unless forced). Spontaneous `starting` after boot -> `MCU '<n>' spontaneous restart`.
5. Stats: `stats count=%u sum=%u sumsq=%u` response; derived `mcu_awake`, `mcu_task_avg`, `mcu_task_stddev` using `STATS_SUMSQ_BASE`.
6. `klippy:ready` frequency sanity check: compare estimated frequency in MHz with `CLOCK_FREQ`; `runtime_warning` if off by more than 1% and the rounded MHz differ.

---

## 7. Webhooks API (unix socket, JSON)

### 7.1 Transport

* Enabled only when `-a <path>` is given and no `-i` file input. The socket file is removed if present (failure to remove an existing file raises), then `AF_UNIX/SOCK_STREAM` bound, `listen(1)`, non-blocking, registered on the reactor. No chmod. The deployed printer uses `/tmp/klippy_uds`.
* Messages are JSON values terminated by a single `0x03` byte. Input is read 4096 bytes at a time; partial frames are buffered. A zero-byte read (or `EBADF`) closes the client. Decode failures are logged (`webhooks: Error decoding Server Request <bytes>`) and skipped without a reply.
* JSON codec: `msgspec.json` if importable, else stdlib `json` with compact separators. Encoding errors (`TypeError`/`ValueError`) shut the printer down (`json encoding error: ...`). The two codecs differ for `nan`/`inf` and for non-string dict keys; the Ender runtime most likely has no `msgspec` `[INFERENCE]`.
* Output is buffered per client; `sock.send` is attempted immediately; a partial write sets `is_blocking`, switches the fd to write-only wake, and sets `blocking_count=5`. The `webhooks` object's `stats()` (called once per second by `statistics`) decrements the counter for blocked clients and closes any that reach below zero (`Closing unresponsive client <uid>`). A write error other than `EAGAIN` closes the client.
* Clients are keyed by `id(client)` (used as `uid`). Per-client log lines: `webhooks client <uid>: New connection | Client info {...} | Disconnected`; rollover info `webhooks client <uid>: <repr(info)>` keyed `webhooks <uid>`; the last 20 raw requests are dumped on `klippy:analyze_shutdown` (`Dumping %d requests for client %d`, `Received %f: %s`).
* Klippy disconnect closes all clients and the listener.

### 7.2 Request and response shapes

* Request: `{"id": <any non-null JSON>, "method": "<path>", "params": {...}}`. `method` must be a string and `params` an object (default `{}`); otherwise the request is dropped. If `id` is missing or `null` the call executes but nothing is returned.
* Success: `{"id": <id>, "result": {...}}` (`{}` if the handler sent nothing). Failure: `{"id": <id>, "error": {"error": "WebRequestError", "message": "<text>"}}`.
* Each request is executed in its own `reactor.register_callback`, so handlers may pause and responses can arrive out of order relative to request order.
* Error mapping: a `printer.command_error` (including `WebRequestError`, a `CommandError` subclass) becomes an error response; any other exception becomes an error response and also `printer.invoke_shutdown("Internal Error on WebRequest: <method>")`. Unknown method: `webhooks: No registered callback for path '<method>'`.
* Parameter helpers: `get(item, default, types)`, `get_str/int/float/dict`. Missing: `Missing Argument [<item>]`; wrong type: `Invalid Argument Type [<item>]` (strict `type()` check: `bool` is not an `int`; `get_float` accepts `int` or `float`). A handler may call `send` once (second call raises `Multiple calls to send not allowed`).

### 7.3 Built-in endpoints (registration order = `list_endpoints` order)

1. `list_endpoints` -> `{"endpoints": [paths...]}` (dict insertion order).
2. `shakehands` -> `{state, state_message, shakehands_id (the request id), curtime (reactor monotonic)}`.
3. `info` (optional `client_info` dict; stored for logs) -> `{state, state_message, hostname, klipper_path, python_path, process_id, user_id, group_id, log_file, config_file, software_version, cpu_info}` (the last four from `start_args`, `null` if absent).
4. `emergency_stop` -> `invoke_shutdown("Shutdown due to webhooks request")`, result `{}`.
5. `register_remote_method` (`response_template` dict, `remote_method` str) -> remembers the template per connection.
6. `gcode/help` -> `{CMD: help}` for every command with help (all registered, not only those valid in the current state).
7. `gcode/script` (`script` str) -> `run_script` under the G-code mutex; a `CommandError` becomes an error response with its message.
8. `gcode/restart`, `gcode/firmware_restart` -> run `restart` / `firmware_restart`.
9. `gcode/subscribe_output` (`response_template` dict, default `{}`) -> registers the connection; every G-code output line (`// ...`, `!! ...`, `ok`, raw responses) is sent as `{**template, "params": {"response": "<line>"}}`. Closed clients are pruned on the next output line. Re-subscribing replaces the template.
10. `objects/list` -> `{"objects": [names of printer objects having get_status]}`.
11. `objects/query` (`objects`: `{name: null | [fields]}`) -> `{"eventtime": t, "status": {name: {field: value}}}`. `null` means all fields. Unknown objects, or objects without `get_status`, appear as `{}`. Argument validation: keys must be strings, values `null` or a list of strings, else `Invalid argument`.
12. `objects/subscribe` (same plus `response_template`) -> replies like `objects/query`, then registers the client. Updates go out at most every `SUBSCRIPTION_REFRESH_TIME = 0.25 s` as `{**template, "params": {"eventtime": t, "status": {...}}}` containing only fields that changed relative to the previous round's snapshot (`last_query`, a single global snapshot shared by all clients; an object not queried in the previous round compares against `{}`). `null` field lists are expanded once on the first pass and frozen. Closed clients are dropped lazily; the shared timer unregisters itself when no object is requested.
13. `register_mux_endpoint(path, key, value, cb)` (used by bulk-sensor and load-cell streaming endpoints): single key per path, error `The value '<v>' is not valid for <key>`; messages: `mux endpoint <path> <key> <value> may have only one key (<k>)` / `... already registered (...)`.
14. Extras register further endpoints (other slices): `bed_mesh/dump_mesh`, `get_mesh`, `query_endstops/status`, `pause_resume/cancel|pause|resume`, bulk/load-cell sensor streams via mux endpoints.

### 7.4 Remote methods

`WebHooks.call_remote_method(method, **kwargs)` (called by the macro action `action_call_remote_method`) sends `{"params": kwargs, **template}` to every registered open connection for that method; if no connection is open it deletes the registration and raises a command error (`Remote method '<m>' not registered` / `No active connections for method '<m>'`), which the macro layer logs and swallows.

### 7.5 `get_status` of `webhooks` and `gcode`

* `webhooks.get_status` -> `{state, state_message}` (`state` in `ready|startup|shutdown|error`).
* `gcode.get_status` -> `{commands: {NAME: {help?}}}`, rebuilt whenever commands register or the ready/shutdown switch occurs.

### 7.6 Fork-specific endpoints

`extras/pause_resume.py` (commit `3a63ec4a`, "support stock c440x display endpoints" `[INFERENCE]` for the attribution) registers `pause_resume/check_continue_print_state` -> `{file_state: false, eeprom_state: false}` (fixed stub) and `getBootLoaderVersion` -> `{software_version: <mcu constant 'software_version' or ''>}`. They belong to the webhooks contract the Creality `c440x` UI expects over `/tmp/klippy_uds`.

---

## 8. Test coverage summary

| Core unit | Exercised by | Notes |
|---|---|---|
| klippy.py, reactor.py, gcode.py (`GCodeIO` file mode), configfile.py (read path), pins.py, mcu.py (file mode), clocksync.py (`connect_file`), msgproto.py, util.py | all 37 `.test` files (`scripts/test_klippy.py`) | assertion is process exit status only |
| gcode `STATUS/HELP/M115/RESTART` | `commands.test` | |
| Error-exit path | `out_of_bounds.test` (`SHOULD_FAIL`), `macros.test` (M112 as failure signal) | |
| Autosave read path | `delta_calibrate.test`, `rotary_delta_calibrate.test` configs | no `SAVE_CONFIG` call in any test |
| Secondary MCU clocks | `generic_cartesian_iqex.test`, `generic_cartesian_itex.test` (`extboard`) | file mode |
| Linux-process dictionary | `linuxtest.test` | |
| mathutil | delta/rotary-delta calibrate, z_tilt, input_shaper, eddy, printers `[INFERENCE]` | |
| webhooks server, UART/CAN/pipe, restart policies, shutdown handling, `queuelogger` rotation, `console.py`, `parsedump.py`, `--import-test` | none | |

Test dictionaries (`atmega2560.dict`, `stm32h723.dict`, `linuxprocess.dict`) come from a firmware build and are passed through `-d`; `scripts/ci-build.sh` runs `scripts/test_klippy.py -d ${DICTDIR} test/klippy/*.test`.

**Parity oracle to add**: the test harness discards `_test_output` (the `-o` debug file). Running the Python tree and the port on the same `.test` inputs and comparing the output files byte for byte (and via `parsedump.py` for readable diffs) checks config CRC text, command encoding, clock conversion, step compression and G-code semantics in one go. Because `connect_file` fixes the clock estimate to `freq=1e12`, the output is deterministic.

---

## 9. Cross-cutting risks and port order

### 9.1 Risks seen in this slice

1. **Reactor semantics are the contract.** Stackful pause inside deep call chains (gcode handler -> homing -> toolhead -> pause), timer ordering, copy-on-write timer list, `update_timer` ignored while a timer runs, mutex hand-off, `assert_no_pause` as an assertion. Any scheduling redesign changes behaviour of all extras.
2. **Python-specific text behaviour reaching users and the MCU.** `configparser`, `int()/float()` acceptance, `str(float)`, `%s` formatting, `shlex`, `re.split`. Error messages are matched by tests, UIs and `error_mcu.py`.
3. **Config command text and CRC.** The text of every `config_*` command is hashed with CRC32 and compared with the MCU's stored value; any formatting drift forces an MCU restart. It also drives the text->binary encoder.
4. **Threads at the C boundary.** The serial reader thread invokes Python response handlers concurrently with the reactor; the safe hand-off points are `async_complete`, `register_async_callback`, and lock-protected handler tables. Shared fields (`clock_est`, `last_clock`, `queries_pending`) are read without locks.
5. **C ABI.** `serialhdl`, `mcu` (`trdispatch_*`) and `reactor` (`get_monotonic`, `set_thread_name`, logging callback) depend on `chelper` symbols. The Rust port needs either an FFI shim identical to the cffi cdefs in `chelper/__init__.py` or a rewrite of `serialqueue.c`, `trdispatch.c`, `pyhelper.c` together with these units. C `errorf` output arrives in logs as `ERROR:root:b'...'`.
6. **Float-sensitive MCU values.** `seconds_to_clock`, PWM and ADC scaling, trsync timing, clock regression, secondary clock calibration, input-shaper coefficients (`gaussian_solve`). Preserve operation order and truncation (`int()` toward zero, `+0.5` before truncation).
7. **Dynamic loading and duck typing.** 143 extras loaded by name, `getattr(self, 'cmd_' + name)`, `hasattr(o, 'get_status')`, `lookup_object` of any name; a Rust port needs a registry and trait objects with optional methods.
8. **Time sources.** Reactor uses C `CLOCK_MONOTONIC`; log lines mix `time.time()`, monotonic and MCU clocks.
9. **Fork diagnostics** are interleaved with core control flow (e.g. `_handle_shutdown` calls the capture first; `_set_state` triggers `_debug_host_state`). Decide what to keep before porting `mcu.py` and `klippy.py`.
10. **Known quirks to decide explicitly**: `MCU_adc` batch unpack reads one sample; `start_args.update(options.dictionary)` crashes without `-d`; unused config options are fatal errors (not warnings); `ClientConnection` stops reading while blocked; the delta-subscription snapshot is global, not per-client; `reactor.EPollReactor` is dead code.
11. **Fork test gap.** No automated test covers webhooks, UART/CAN, restart, shutdown or the `rpi` pipe MCU. Golden logs in `printer-backups/` and `stepcompress-debug/` are the only real-hardware data.
12. **Python 3.8 on MIPS** is the production runtime (`/usr/share/klippy-env/bin/python`, `klipper_mcu` built for MIPS NaN2008). Float behaviour on the target and the 32-bit address space matter if the Rust build targets MIPS.

### 9.2 Suggested port order (leaves first)

1. Pure leaves with no dependencies: `queuelogger` (log format), `pins`, `util` (pty, cpu/device/git info), `mathutil` (float parity tests), `parsedump`.
2. `msgproto` (verify against the three test dictionaries and recorded wire dumps; includes Python-compatible integer text parsing). Then `parsedump` becomes usable as the oracle.
3. `configfile` (needs a `configparser`-compatible reader and a Python-compatible float/int/str formatter; verify by dumping `get_status` of the Python tree for every config in `config/` and `test/klippy/`).
4. `reactor` (decide coroutine strategy; test with a fake clock and a battery of pause/mutex/completion scenarios taken from section 3; needs only `get_monotonic` from C).
5. `gcode` (parser corpus comparison, mux behaviour, file-input mode, response formatting) and `webhooks` (record and replay sessions; depends on `gcode.CommandError` and the reactor's fd API).
6. `clocksync` (replay recorded `uptime`/`clock` samples) and `serialhdl` (needs the `serialqueue` ABI or its Rust port; implement file mode first, then pipe, UART, CAN).
7. `mcu` (pin classes, config/CRC text, restart and shutdown policies; fork diagnostics last; verify with the `-o` oracle on all `.test` inputs).
8. `klippy.py` (registry, event bus, restart loop, CLI) last because it plugs everything together; keep the extras registry stubbed with the real module list from `klippy/extras/` and `klippy/kinematics/`.
9. `console.py` only if wanted.

Dependencies on other slices: `toolhead`/`stepper`/`extras.motion_queuing` (consumers of `mcu`, `clocksync`, `reactor`), `chelper` C code (serialqueue, trdispatch, pyhelper), and the 143 extras (consumers of `configfile`, `gcode`, `webhooks`, `pins`, `printer`). These interfaces are frozen by sections 2 to 7 above.
