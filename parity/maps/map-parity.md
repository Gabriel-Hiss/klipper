# Parity oracle map: test, harness and Ender-3 V3 KE deployment slice

Scope: everything that can act as an oracle for a Python-to-Rust port of the Klipper host (`klippy/` plus `klippy/chelper/`). All paths are relative to `C:/Users/User/CLionProjects/klipper-cp`. Tags: facts come from files I read; anything I could not verify is marked `[INFERENCE]`. I had no shell, so I could not run `git diff`, `git log -- <path>` or any build. Fork attribution below comes from the branch reflog (`.git/logs/refs/heads/ender-3-v3-ke`) plus file content, not from per-file diffs.

## 0. Headline findings

1. The only machine-checked oracle today is "klippy runs a G-code file in batch mode and exits 0". The runner (`scripts/test_klippy.py`) asserts the process exit status and nothing else. The serialized MCU command stream is written to `_test_output` and deleted. No golden output exists anywhere in the repo.
2. A strong, hardware-free comparison point is cheap to build from what exists: the batch-mode output file decoded with `klippy/parsedump.py` gives a text dump of every MCU command. That is the oracle to build on (section 10).
3. Only one MCU data dictionary exists on disk: `out/klipper.dict` (Linux-process MCU, `rpi`, 50 MHz). It is git-ignored (`.gitignore`: `out`). The 28 dictionaries the test suite names must be rebuilt (needs the cross toolchains in `scripts/ci-install.sh`) or downloaded from the CI artifact `data-dict`. The real main MCU of the Ender (GD32F303, stock Creality firmware) has no dictionary in the repo.
4. `stepcompress-debug/harness/`, `stepcompress-debug/patches/`, `tests/fixtures/` and `klippy-protocol/` are empty directories. `stepcompress-debug/` and `printer-backups/` are listed in `.git/info/exclude`, so they are local to this machine and will not reach a swarm that clones the repo. What remains in `stepcompress-debug/` is data (`data/trapq_real.json`, `data/grid_real.json`), four prebuilt MIPS `c_helper.so` variants in `bin/`, logs, and the incident report. The replay scripts that produced the logs live outside the repo (`/home/biel/instrument-build-20260916-191312/` on the dev machine, seen in `logs/host-smoke-output.txt`).
5. klippy cannot run on this Windows host: `klippy/util.py:6` imports `pty`, `fcntl`, `termios`. Oracle runs need Linux or WSL (the logs show the author uses WSL with Python 3.14.4 and GCC 15.2).
6. The fork changes behavior in two constants that affect timing (and possibly batch-mode output): `BGFLUSH_SG_LOW_TIME/HIGH_TIME` (0.450/0.700 upstream to 1.000/1.250) in `klippy/extras/motion_queuing.py:11-12`, and `BUFFER_TIME_HIGH` (1.0 to 2.0) in `klippy/toolhead.py:195`. Evidence: the pad backups `printer-backups/192.168.0.24/20260823-1329/klipper-mainline-live/klippy/...` still carry the old values. The Rust port must pick one set and the oracle must run against the same set.

## 1. Inventory of oracle assets

| Path | Size | State |
|---|---|---|
| `scripts/test_klippy.py` | 6.3 KB, about 175 lines | Test runner (section 2) |
| `test/klippy/*.test` | 37 files, `printers.test` 12.3 KB, others 0.1-1.6 KB | Test scripts |
| `test/klippy/*.cfg` | 31 files, 0.3-12.7 KB | Test printer configs |
| `test/klippy/move.gcode`, `test/klippy/sdcard_loop/big.gcode` | 425 B, 8.2 KB | G-code inputs |
| `test/configs/*.config` | 40 files, 1-3 lines each | Kconfig fragments to build dictionaries |
| `scripts/ci-build.sh`, `scripts/ci-install.sh` | about 82, 79 lines | CI driver |
| `.github/workflows/build-test.yaml` | about 22 lines | Runs the two CI scripts on `ubuntu-22.04`, uploads `ci_build/dict` as artifact `data-dict` |
| `.github/workflows/{klipper3d-deploy,stale-issue-bot,reviewer-needed-label,invalid-label-bot}.yaml` | | Docs deploy and issue hygiene. Irrelevant to parity |
| `tests/fixtures/`, `klippy-protocol/` | empty | Nothing to reuse. Created for the Rust effort but never populated |
| `stepcompress-debug/` | see section 6 | Local only (git-excluded) |
| `printer-backups/192.168.0.24/` | tarballs, logs, old klippy snapshots | Local only (git-excluded). Contains `system-klipper.tar.gz` (stock Creality Klipper), `printer-data.tar.gz`, `S55klipper_service`, `S57klipper_mcu`, `S56moonraker_service` init scripts |
| `config/ender-3-v3-ke/` | README, SERVICES, TODO, printer.cfg, fan.cfg, gcode_macro.cfg, input_shaper.cfg, moonraker.conf, S55klipper_mainline | Deployment (section 7) |
| `scripts/{S99zz_mainline_switch,wait-creality-start-mainline.sh,S55klipper_mainline_direct,S57klipper_mcu_mainline,start-mainline.sh,build-mips-nan2008-klipper.sh,mips_nan2008_start.c,mips_nan2008_compat.c}` | 0.2-2.5 KB each | Deployment (section 7) |
| `pad_sysroot/lib/` | libc-2.29.so, ld-2.29.so, ld-linux-mipsn8.so.1, libgcc_s.so.1, libcrypt-2.29.so | Link-time sysroot copied from the pad |
| `out/klipper.dict`, `out/klipper.elf` | 11.8 KB, 327 KB | Linux-process MCU for the pad. Git-ignored |
| `c_helper.so` (repo root) | 46.9 KB | Stray build artifact, git-ignored (`*.so`) |

Rust code in the repo: none (no `Cargo.toml`, no `*.rs`). Git refs `experiment/klippy-rs` and the reflog entry `codex/klippy-rs-foundation` exist, but `experiment/klippy-rs` points at `7c8d59f1`, the same merge commit as `ender-3-v3-ke` at that time, and the codex branch ref is gone. Treat the port as a blank start.

## 2. How the regression tests run

### 2.1 Unit: `scripts/test_klippy.py` (test runner)

- Size and deps: about 175 lines. Stdlib only (`optparse`, `subprocess`, `logging`). It shells out to `sys.executable ./klippy/klippy.py`, so the cwd must be the repo root.
- Command line: `test_klippy.py [-d dictdir] [-t tempdir] [-k] [-v] <test files>`. CI calls `scripts/test_klippy.py -d ${DICTDIR} test/klippy/*.test` (`scripts/ci-build.sh:77-82`), once with Python 3 and once with Python 2.
- `.test` file mini-language (`parse_test`, lines 27-64). `#` starts a comment. First word `CONFIG <cfg>` (path relative to the .test file), `DICTIONARY <main.dict> [mcuname=file.dict ...]` (relative to the `-d` dir), `GCODE <file>` (relative to the .test file), `SHOULD_FAIL`. Every other non-empty line is G-code and is written to `_test_.gcode`. Using both `GCODE` and inline G-code is an error.
- Multi-config files (`printers.test`): the first `CONFIG` is held; when a second one appears the first is launched, then each later `CONFIG` launches immediately with the dictionary and G-code that are current at that point. About 199 `CONFIG` entries in `printers.test`, spread over about 30 `DICTIONARY` blocks, all running `move.gcode`.
- Invocation per run (`launch_test`, lines 93-99):
  `python ./klippy/klippy.py <cfg> -i <gcode> -o _test_output -v -d <dict> [-d mcu=<dict> ...] [-l _test_.log]`
  `-l _test_.log` is added only when the runner is not in `-v` mode, so quiet runs use the background logger (`klippy/queuelogger.py`) and verbose runs log to stderr.
- Pass/fail rule: `exit code != 0` is failure; `SHOULD_FAIL` inverts it (`out_of_bounds.test` is the only user, expects a move-out-of-range error). On failure the log is dumped to stdout. Runner exits `-1` on first failed file.
- Cleanup quirks: deletes `_test_output*` and `_test_.log`, and the temp gcode. The loop `for fname in os.listdir(self.tempdir): if fname.startswith(...): os.unlink(fname)` unlinks relative to the cwd, not `tempdir`; it only works when `-t` is `.` (default). `-k` keeps files. Multi-MCU output files are named `_test_output-<mcuname>`.

### 2.2 What a "pass" means

klippy in batch mode ends with `sys.exit(-1)` when `Printer.run()` returns `error_exit` (end of `klippy.py:main()`). That happens in these cases, and nothing else is checked:
- Config or pin errors during connect (`Printer._connect`, `klippy.py` lines 127-150), including unknown sections/options (`check_unused_options`).
- Any state other than ready when `debuginput` is set: `Printer._set_state` calls `request_exit('error_exit')` (`klippy.py:63-67`).
- Any G-code command error: `gcode.py:259-262` (`if self.is_fileinput: self.printer.request_exit('error_exit')`).
- Any shutdown, including M112 and "Internal error in stepcompress": `gcode.py:430-431`.
- Python exceptions in reactor callbacks.

Logic assertions exist only as in-config macros that trigger `M112` on a wrong value, for example `test/klippy/macros.cfg` (`TEST_expression`, `TEST_variable_part2`, `TEST_param`, `TEST_in`). `macros.test` runs `TESTIT`. This is the only place computed values are compared with expected values.

### 2.3 Inputs, outputs, stubs in batch mode

Inputs:
- Config file (positional), G-code file `-i` (becomes `start_args['debuginput']` and `gcode_fd`), one dictionary per MCU: `-d file` for `mcu`, `-d name=file` for `mcu name` (`klippy.py:308-315`, `368-370`; keys `dictionary` and `dictionary_<name>`).
- Dictionaries are raw JSON, not zlib (`serialhdl.connect_file` calls `process_identify(dictionary, decompress=False)`, `serialhdl.py:207-209`).
- G-code is read in 4096-byte chunks (`gcode.py` `GCodeIO._process_data`), which matters only for the M112 out-of-order check.

Outputs:
- `-o <file>`: for MCU `mcu` the file itself, for `mcu foo` the file `<out>-foo` (`mcu.py` `_attach_file`, about lines 970-985). Content is the byte stream a serial port would carry: message blocks produced by the C `serialqueue` (`klippy/chelper/serialqueue.c`, debug-file mode `SQT_DEBUGFILE 'f'`, line 181). Block layout from `klippy/chelper/msgblock.h`: `MESSAGE_MAX 64` bytes per block, `MESSAGE_PAYLOAD_MAX` 59, header 2 bytes `[len][0x10 | (seq & 0x0f)]`, trailer 3 bytes `[crc16 hi][crc16 lo][0x7e]`. Payload messages are VLQ-encoded with ids from the dictionary.
- `-l <file>` log (or stderr). Log lines worth knowing: `Loaded MCU '<n>' <k> commands (<version> / <gcc versions>)`, `MCU '<n>' config: ...`, `Configured MCU '<n>' (500 moves)`, `Exiting (print time %.3fs)` (`gcode.py` `request_restart`, lines 352-361; a cheap summary of total move time), plus every `respond_info` text (`gcode.py:250-254` logs unless `log=False`). `GCodeIO` registers no output handler in file mode (`gcode.py:402-403`), so command responses reach only the log.
- Decoder: `python klippy/parsedump.py <dict> <outfile>` prints one text line per message, e.g. `queue_step oid=3 interval=... count=... add=...`. It uses `msgproto.MessageParser.check_packet/dump` and drops the block header line (`msgs[1:]`), so block boundaries and sequence numbers never appear in the text.
- Related tool: `scripts/stepstats.py` (Python 2 `print` syntax, will not run on Python 3) reads parsedump text and totals pulses per oid.

What batch mode replaces or skips (all keyed on `mcu.is_fileoutput()` or `start_args['debugoutput']`):
- No MCU replies at all. `DummyResponse` for queries (`mcu.py:85-87`); `send_wait_ack = send` (`mcu.py:119-121`); `get_config` is faked as `{is_config:0, move_count:500, crc:0}` (`mcu.py:1177-1180`). `finalize_config crc=` is computed for real from the config commands (`zlib.crc32`, `mcu.py` `_finalize_config`), so the CRC is part of the oracle.
- Clock: `ClockSync.connect_file` sets the C clock estimate to `freq = 1e12`, `conv_time = reactor.monotonic()`, `conv_clock = 0` (`clocksync.py:50-57`; `pace` defaults to False). With that rate every `min_clock` gate opens instantly, so output is not paced. `print_time_to_clock` is `int(print_time * mcu_freq)` (`clocksync.py:123-124`). `estimated_print_time` is replaced by a function returning `0.` (`mcu.py:1302-1306`). Result: print_time starts from zero and is independent of wall time.
- `toolhead.can_pause` and `motion_queuing.can_pause` are False (`toolhead.py:221-223`, `motion_queuing.py:99-101`); motion flushing uses `_flush_handler_debug` (`motion_queuing.py:104-105, 447-462`: a synthetic loop with `faux_time = need_flush_time - 1.5`) instead of the wall-clock `_flush_handler`.
- Skipped hardware paths: endstop/probe/trsync waits return canned values (`mcu.py:271-272, 327-329, 395-405`); `stepper._query_mcu_position` returns (`stepper.py:213`); `check_timeout`, `_ready`, `_analyze_shutdown` return early (`mcu.py:1016-1040, 1093`); heaters set `can_extrude` true and `M109/M190/TEMPERATURE_WAIT` return immediately (`heaters.py:37-39, 349-351, 374-375`); `statistics` timer is never started (`statistics.py:62`); sensors with identity checks skip them (`ads1220.py:185`, `ads131m0x.py:296`, `ldc1612.py:147,202`, `bme280.py:156`, `neopixel.py:91`, `tmc_uart.py`, `tmc2130.py:202-225`, `tmc2660.py:204`, `load_cell.py:319,334`, `probe_eddy_current.py:518`, `trigger_analog.py:408`, `temperature_host.py:22`, `temperature_mcu.py:38`, `endstop_phase.py:41,92`, `replicape.py:120,217`, `bus.py:246-292`).
- Because MCU ADC/temperature responses never arrive, the heater PID loop, `verify_heater`, fans driven by temperature and every sensor-derived `get_status()` value run on initial values only.
- Process exit: at EOF `GCodeIO._process_data` calls `gcode.request_restart('exit')` (`gcode.py:448-452`), which dwells 0.5 s of print time and `toolhead.wait_moves()` before `reactor.end()`. A `RESTART` in the script re-enters the `while 1` loop in `klippy.py:main()` after `time.sleep(1.)`, reopens `-o` with `'wb'` (truncating it) and reconnects; that is why `commands.test` (which ends in `RESTART`) leaves almost no stream behind.

### 2.4 Test inventory (37 files)

DICTIONARY use: 33 files use `atmega2560.dict` only; `generic_cartesian_iqex.test` and `generic_cartesian_itex.test` use `stm32h723.dict extboard=stm32h723.dict`; `linuxtest.test` uses `linuxprocess.dict`; `printers.test` uses 28 distinct dictionaries (list in 4.3). Features each file drives:

| Test | Exercises |
|---|---|
| `commands.test` (`config/example-cartesian.cfg`) | GET_POSITION, M114, STATUS, HELP, QUERY_ENDSTOPS, M115, M18, SAVE/RESTORE_GCODE_STATE, G92, offsets, SET_VELOCITY_LIMIT, M204, SET_PRESSURE_ADVANCE, RESTART |
| `printers.test` (about 199 configs + `move.gcode`) | Config parsing and pin setup of every `config/example-*.cfg`, `generic-*.cfg`, `printer-*.cfg`, `kit-*.cfg`; G28, G0/G1 on X/Y/Z/E, G4, M400, G92, M106/M107, relative moves |
| `delta.test`, `rotary_delta_calibrate.test`, `delta_calibrate.test`, `polar.test`, `corexyuv.test`, `dual_carriage.test`, `hybrid_corexy_dual_carriage.test`, `generic_cartesian*.test`, `multi_z.test`, `manual_stepper.test` | Kinematics (delta, rotary delta, polar, corexy-uv, IDEX, generic cartesian, hybrid corexy), STEPPER_BUZZ, SET_DUAL_CARRIAGE, MANUAL_STEPPER, input shaper per carriage, multi-MCU (`iqex`, `itex`) |
| `bed_mesh.test`, `bltouch.test`, `eddy.test`, `load_cell.test`, `smart_effector.test`, `z_virtual_endstop.test`, `z_tilt.test`, `quad_gantry_level.test`, `screws_tilt_adjust.test`, `bed_screws.test` | Probes and leveling with canned probe results |
| `extruders.test`, `pressure_advance.test`, `input_shaper.test` | SET_EXTRUDER_ROTATION_DISTANCE, SYNC_EXTRUDER_MOTION, pressure advance, SET_INPUT_SHAPER |
| `temperature.test`, `tmc.test`, `led.test`, `pwm.test`, `macros.test`, `exclude_object.test`, `gcode_arcs.test`, `sdcard_loop.test`, `linuxtest.test`, `out_of_bounds.test` | Sensor config; DUMP_TMC/INIT_TMC/SET_TMC_CURRENT; LED chips; SET_PIN soft/hard PWM; Jinja macros; M486/EXCLUDE_OBJECT; G2/G3; virtual SD loop; DS18B20 on the Linux MCU; bounds check (expected failure) |

`config/ender-3-v3-ke/printer.cfg` is not run by any test.

### 2.5 CI wiring

`.github/workflows/build-test.yaml` runs `scripts/ci-install.sh` (apt packages `virtualenv python2-dev libffi-dev build-essential gcc-avr avr-libc gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi`, downloads `pru-elf-2024.05` and `or1k-elf-12.0.1`, builds a Python 3 and a Python 2 virtualenv from `scripts/klippy-requirements.txt` plus `scripts/tests-requirements.txt`) and then `scripts/ci-build.sh`. `ci-build.sh` runs: whitespace check; for every `test/configs/*.config` a full MCU build (`make olddefconfig; make V=1`) that copies `out/klipper.dict` to `ci_build/dict/<name>.dict`; `klippy.py --import-test` (imports every module in `extras/` and `kinematics/`, `klippy.py:293-305`) under both Pythons; then the 37 test files under both Pythons. The step "Upload micro-controller data dictionaries" publishes `ci_build/dict` as artifact `data-dict`. Whether CI actually runs on the fork `Gabriel-Hiss/klipper` is unknown.

Python packages touched by tests (`scripts/klippy-requirements.txt`, `scripts/tests-requirements.txt`): greenlet 2.0.2/3.1.1, cffi 1.14.6/1.17.1, Jinja2 2.11.3, markupsafe 1.1.1, pyserial 3.4, python-can 3.3.4, msgspec 0.19.0 (optional, `webhooks.py:9-13`), scipy 1.15.3 (tests only, for `trigger_analog.DigitalFilter`, `klippy/extras/trigger_analog.py:80-110`). numpy is imported lazily by `angle.py`, `load_cell_probe.py` and the resonance code; it is not in the requirements files.

## 3. Unit cards for the oracle units

| Unit | Deps | External contract | Runtime traits | Tests | Fork changes | Difficulty |
|---|---|---|---|---|---|---|
| `scripts/test_klippy.py` | stdlib; spawns klippy | CLI above; `.test` DSL; temp files `_test_.gcode`, `_test_.log`, `_test_output[-mcu]` | Subprocess only | Self-run by CI | None | Low. Re-implement in Rust or keep as Python driver. |
| `test/klippy/*.test`, `*.cfg`, `move.gcode`, `sdcard_loop/big.gcode` | klippy batch mode, 28 dictionaries | Config options and G-code of the features in 2.4 | Jinja macros in `macros.cfg`, `led.cfg`; virtual SD reactor timers in `sdcard_loop.cfg` | They are the tests | None (files dated 5mo/1mo, upstream style) | Low to run, medium to turn into assertions (needs goldens). |
| `test/configs/*.config` | Kconfig | One MCU build each, produces dictionary | Needs cross toolchains | CI | None | Low. |
| `scripts/ci-*.sh`, `build-test.yaml` | apt, wget, Python 2 + 3 | Artifact `data-dict` | Python 2 path is dead weight for a Rust port | n/a | None | Low. |
| `klippy/parsedump.py` (decoder) | `msgproto` | `parsedump.py <dict> <file>` text | None | Not in CI | None | Low. Port `msgproto` first; this tool is a thin consumer. |
| `stepcompress-debug/*` | see section 6 | | | | all fork | see section 6 |
| Deployment pieces | see section 7 | | | | all fork | see section 7 |

Port-difficulty ratings for the harness pieces themselves: golden capture and diff tooling low; reproducing batch-mode stubs in the Rust host medium; replay harness for `trapq_real.json` medium (needs rewriting the lost script); mock-MCU/API harness high (new infrastructure, no prior art in the repo); pad packaging high (unproven MIPS NaN2008 Rust toolchain).

## 4. MCU data dictionaries

### 4.1 Format and generation

`scripts/buildcommands.py` runs at the end of the MCU build (`make`), reads the `.compile_time_request` section of the objects, assigns message ids (`create_message_ids`, `buildcommands.py:290-305`: ids follow the order in which `compile_time_request` lists commands, starting after a fixed table), and writes `out/klipper.dict` as `json.dumps(data, separators=(',', ':'), sort_keys=True)` (`buildcommands.py:577`; the firmware embeds a zlib copy at level 9, the `.dict` file on disk is uncompressed). Top-level keys: `app`, `build_versions`, `commands`, `config` (constants such as `CLOCK_FREQ`, `MCU`, `ADC_MAX`, `PWM_MAX`), `enumerations` (pin names, bus names), `license`, `output`, `responses`, `version`.

Deterministic parts: command ids, formats, constants and enumerations for a fixed Kconfig and source tree. Non-deterministic: `version` (git describe, plus `-YYYYMMDD_HHMMSS-hostname` when the build is dirty or tool versions are unknown, `buildcommands.py:497-510`) and `build_versions`. The host copies them into logs only, never into the MCU stream.

### 4.2 Can they be generated without hardware?

Yes, without any printer, but not without compilers. `make` needs the target toolchain to compile the sources (no flash step). Requirements per family are exactly those installed by `scripts/ci-install.sh`: `gcc-avr` (atmega*, at90usb1286, lgt8f328p), `gcc-arm-none-eabi` (sam*, samd*, stm32*, lpc176x, rp2040/rp2350, hc32f460), `pru-elf` (pru), `or1k-elf` (ar100), host `gcc` (linuxprocess, hostsimulator). A host x86-64 Linux with those packages is enough; no emulator is needed. Alternatives that avoid the toolchains: the artifact `data-dict` from a CI run, or the community tarball `klipper-dict-20??????.tar.gz` referenced in `docs/Debugging.md:17-24`. If the Rust test suite wants stability against toolchain drift, commit the 28 `.dict` files as fixtures; `tests/fixtures/` is the obvious home. The host simulator build (`MACH_SIMU`, `src/simulator/`) is a compile stub only and cannot act as a responding MCU.

### 4.3 Dictionaries the suite needs

atmega2560, atmega1280, atmega1284p, atmega644p, at90usb1286, sam3x8c, sam3x8e, sam4s8c, sam4e8e, samd51p20, same70q20b, lpc176x, stm32f070, stm32f103, stm32f103-serial, stm32f401, stm32f405, stm32f407, stm32f429, stm32f446, stm32h723, stm32h743, stm32g0b1, rp2040, hc32f460-serial-PA3PA2, hc32f460-serial-PA7PA8, pru, linuxprocess (28). Configs that exist in `test/configs/` but no test names: ar100, atmega328, lgt8f328p, rp2350, samd21g18, stm32f031, stm32f042, stm32f765, stm32g431, stm32g474, stm32l412, hostsimulator (build tests only).

### 4.4 What exists for the Ender

- `out/klipper.dict` (11.8 KB, git-ignored): `"MCU":"linux"`, `CLOCK_FREQ=50000000`, `ADC_MAX=4095`, `PWM_MAX=32768`, `PCA9685_MAX=4096`, 145 commands, built with the Ingenic xburst2 GCC 7.2.0 (`build_versions` field) from `.config` (`CONFIG_MACH_LINUX`, `WANT_ADXL345`, `WANT_LIS2DW`, `WANT_HX71X`, `WANT_LDC1612`, `WANT_TRIGGER_ANALOG`, `CONFIG_INLINE_STEPPER_HACK`, etc.). It is the `mcu rpi` of the printer.
- The main MCU is not present. From `stepcompress-debug/logs/capture-20260917-183026Z/klippy-mainline.log` lines 17-18: `Loaded MCU 'mcu' 116 commands (38d96adc-dirty-20231016_135251-longer-virtual-machine / gcc 9.2.1 arm)`, `MCU=gd32f303xe`, `CLOCK_FREQ=120000000`, `SERIAL_BAUD=230400`, `RECEIVE_WINDOW=192`, `RESERVE_PINS_serial=PA2,PA3`, `STEPPER_BOTH_EDGE=1`, `ADC_MAX=4095`, 3802 move slots. Only constants and the command count are logged, not the command table. The full dictionary can only be obtained from the live board (the host fetches it with `identify` messages, `serialhdl._get_identify_data`; `klippy/console.py` or `scripts/dump_mcu.py` can be used) or from the stock firmware sources, which are not in the repo. For offline parity a surrogate build of `test/configs/stm32f103-serial.config` with another clock would differ in message ids and constants, so it checks host behavior but not Ender wire bytes.

## 5. What an output-diff parity harness needs

### 5.1 Shape of the harness

For each scenario `(config, dictionary set, G-code)`:
1. Run Python klippy: `python klippy/klippy.py CFG -i G -o OUT -d DICT [-d name=DICT2] -l LOG` on Linux, with Python 3.8+ and the pinned packages.
2. Run `parsedump.py DICT OUT` per MCU output file. Store text as the golden file.
3. Run the Rust binary with the same arguments (same CLI contract as `klippy.py`: `-i`, `-o`, `-d`, `-l`, `-v`, exit status). Compare its own `OUT` after decoding with the same decoder (or decode both with the Rust `msgproto`, having first validated the Rust decoder against `parsedump.py`).
4. Also compare normalized log lines: `Loaded MCU`, `Configured MCU`, `Exiting (print time ...)`, and every `respond_info` text.

### 5.2 Nondeterminism sources and how to neutralize

| Source | Where | Effect on MCU stream | Neutralize |
|---|---|---|---|
| Block packing and sequence numbers | `serialqueue.c` background thread `command_event` / `build_and_send_command`; a block holds up to 59 payload bytes and is built whenever the thread wakes | Block boundaries, `seq`, crc differ run to run | Compare decoded text (`parsedump`), never raw bytes |
| Cross-queue message order | The thread picks the lowest `req_clock` among messages that are already queued when it wakes (`build_and_send_command`). Python threads race with it | Messages from different `command_queue`s with close `req_clock` may swap `[INFERENCE: I did not run it to see whether swaps occur]` | Measure first: run the Python reference 20 times and diff; then compare per-oid sequences plus a multiset of all lines, or sort by `(oid, order)`. Within one queue order is FIFO and `steppersync_flush` hands each batch to the serial queue atomically (`serialqueue_send_batch`, `steppersync.c:299-340`) |
| Wall clock in logs | `klippy.py` `Printer.run` (`Start printer at <asctime>`), `queuelogger.py:54`, `util.py:67-68` build file times, `[DEBUG-motion]` host lines (`klippy.py` `_debug_host_state`, `walltime=`, `pid=`) | None | Strip timestamp, pid, `Start printer`, `Git version`, `Args`, `Python`, `CPU`, `Linux` lines |
| `reactor.monotonic()` | `chelper.get_monotonic`, used for timers and `set_clock_est(..., conv_time)` | None for the stream (clock is 1e12 Hz in batch mode), real for timers | Do not assert on timer-driven behavior |
| Timer-driven features | `delayed_gcode`, `idle_timeout`, `virtual_sdcard` work timer (`virtual_sdcard.py` work handler, pauses `reactor.pause(NOW)` and `+0.050`), `statistics`, `gcode_button`, LED transmit | Order and presence of side effects depend on whether the run exits before the timer fires | Avoid in goldens or run with a virtual reactor in Rust; `sdcard_loop.test` is timing-sensitive by design |
| `id(self)` in names | `klippy/extras/bed_mesh.py:1327` (`"adaptive-%X" % id(self)`), `klippy/extras/display/menu.py:48` | Appears in profile/log text only | Regex-normalize `adaptive-[0-9A-F]+` and `__menu_...` |
| Random numbers | None in `klippy/` (grep for `random`, `urandom`, `uuid` found nothing) | None | None needed |
| Dict/set ordering | Python 3.7+ preserves insertion order; the only `set()` iteration in `klippy/` is sorted (`kinematics/idex_modes.py:45`). `Printer.objects` is an `OrderedDict` | Order of `klippy:connect` handlers follows config section order | Rust must keep insertion order in config sections and handler lists; Python 2 (still in CI) has random dict order, so only trust Python 3 goldens |
| Git/version strings | `util.get_git_version` runs `git` (called from `klippy.py:main`); `M115` prints `FIRMWARE_VERSION` | Only text responses | Normalize or inject a fixed version |
| Floating point (host) | `toolhead` (accel/jerk lookahead), kinematics in C (`kin_*.c`), `itersolve.c`, `stepcompress.c` (bisect, `(uint64_t)rel_sc`), Python `math` in shaper/pressure advance/thermistors/bed mesh/delta | `queue_step interval/count/add`, `set_next_step_dir`, `trsync` clocks, PWM values, `sos_filter_*` values | Rust `f64` arithmetic is non-fused IEEE, which matches the x86-64 SSE2 build of `c_helper.so` (`chelper/__init__.py:16-17` forces `-mfpmath=sse -msse2` when supported). It need not match the MIPS build (see risk in section 11). Keep operation order identical; do not enable fast-math or FMA contraction |
| libm | `sin`, `cos`, `atan2`, `acos`, `exp`, `log`, `pow`, `sqrt` in `kin_*.c`, `kin_shaper.c`, `extras/input_shaper.py`, `shaper_defs.py`, `thermistor.py` | Last-ULP differences can change an integer clock by one | Pin one libm for goldens (record the glibc version in the golden manifest) and use a bit-exact Rust equivalent or call the system libm; `sqrt` is exact. Add a tolerance policy: strict equality first, then report "off by one clock" separately |
| Float formatting | `%d`, `%f`, `%.6f`, `%g`, `repr()` in log text and in config-command strings built with `%` | Config commands are integers; text in responses uses `%.3f` etc. | Port format strings literally. Rust `{:e}` and `{}` differ from `%g` and `repr`, so write a tested `pyfmt` helper |
| Threads | `serialqueue` pthread, `queuelogger` thread, `serialhdl._bg_thread` (not started in batch mode) | Log ordering only | Compare logs as sets after normalization |
| Environment | `/proc` reads (`util.get_cpu_info`, `get_linux_version`), `os.getuid`, Python version in `webhooks info` | None in batch stream | Ignore |

### 5.3 Preconditions the harness must record in its manifest

Python version, `cffi`/`greenlet` versions, glibc version, compiler flags used for `c_helper.so`, and the exact `klippy` tree hash (because the fork constants in section 0.6 change timing behavior). Without these the goldens are not reproducible.

## 6. `stepcompress-debug/`

Contents (all local, git-excluded):
- `incident_report.md` (3.0 KB): 2026-09-13 report of a stepcompress crash and a second print the agent started without authorization. Not a technical oracle; it records that print #1 crashed with `stepcompress ... Invalid sequence`. Timeline times are UTC-3.
- `data/trapq_real.json` (72.6 KB): one JSON array of objects `{pt, mt, sv, a, sp:[x,y,z], ar:[x,y,z]}`. These are exactly the fields of `struct pull_move` returned by `trapq_extract_old` (`print_time`, `move_t`, `start_v`, `accel`, `start_x/y/z`, `x_r/y_r/z_r`, `chelper/__init__.py` `defs_trapq`). The first entry starts at `pt=2294.612839`, with accel -9000 and +9000 mm/s^2 segments, which matches `max_accel: 9000` in the Ender config. This is a real trapezoidal-move history captured near a crash.
- `data/grid_real.json` (136 B): `{"base":2294.612840058333,"spacing":0.25,"last_step_gen":2295.3628400583334,"step_gen":2295.6128400583334,"flush_offset":-0.05}`. `[INFERENCE]` these are the batch grid parameters of `motion_queuing._advance_flush_time(flush_time, step_gen_time)`: 0.25 s equals `BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME` and -0.05 equals `STEPCOMPRESS_FLUSH_TIME`.
- `bin/c_helper_{original,v2_instrumented,v3_instrumented,v4_instrumented}_mips.so` (198-226 KB): MIPS NaN2008 builds of the C helper, original and with the fork's instrumentation levels (v3 = `motion-v3` stepcompress/itersolve dumps; v4 = `motion-v4` serialqueue ring). They only run on the pad or under a MIPS emulator with the glibc 2.29 sysroot `[INFERENCE]`.
- `logs/`: raw captures. `klippy-crash1-stepper_x.log` (1.4 MB) and `klippy-crash2-stepper_y.log` (1.1 MB) contain `stepcompress o=7 i=-245380 c=1 a=0: Invalid sequence` followed by `[DEBUG-motion] version=motion-v3 ... queue[...]/event[...]/append[...]/conversion[...]` dumps (`%.17g` precision). They also dump the last 20 API requests per client (see 7.5). `capture-*` directories hold `klippy-mainline.log` (4.3 MB in `capture-20260917-183026Z`), `moonraker.log`, `printer.cfg`, `system-snapshot.txt`, `nominal-step-rates.json` (peak 6833 Hz on X) and `evidence-summary.json` (MCU shutdown "Stepper too far in past", `host_stepcompress_invalid_sequence_count: 0`). `verification-v4-*` hold outputs of synthetic smoke tests: `smoke-output.txt` (`PASS bad=0 wrap=0 ret=0`, `PASS bad=1 ... ret=-989898989`, then a forced `i=-500` invalid sequence with a full dump), `shutdown-smoke-output.txt` (8 scenarios, `ALL SHUTDOWN-V4 SMOKE SCENARIOS PASSED`), `scenario_1..3-*-output.txt`, `target-smoke-output.txt`, `serial-smoke-output.txt`.
- `harness/` and `patches/` are empty. The smoke scripts that produced the logs (`host-smoke.py`, `debug_smoke_shutdown_v4`; a compiled `__pycache__/debug_smoke_shutdown_v4.cpython-312.pyc` sits at the repo root) are not recoverable from the repo. The fork instrumentation itself is in the working tree (section 8).

What it captures: step-compression state at failure (queue, last events, 24 recent `append` calls with `step_time`, `clock`, `rel_sc`), solver state in `itersolve.c` (`guess`, `bracket`, three moves), serialqueue decisions, reactor stalls, flush batches, SD chunk timings, MCU shutdown provenance.

Reuse for step-level comparison:
- `trapq_real.json` is a ready input for a differential test of the kinematics plus step generation: load the moves into `trapq_append`, bind `itersolve` with a Cartesian solver (`cartesian_stepper_alloc`) and a `stepcompress` (`stepcompress_fill`, `stepcompress_set_time`), then call `steppersyncmgr_gen_steps` over the grid and compare the `queue_step` sequences from (a) the x86 C library, (b) the MIPS C library, (c) the Rust port. It is a real-world input, so it exercises accel/decel boundaries that hand-made vectors miss. The same input also tests the hypothesis that MIPS and x86 builds diverge `[INFERENCE: not proven by any file here]`.
- The harness that fed it is gone, so it has to be rewritten (a cffi script using the definitions in `klippy/chelper/__init__.py`).
- The `[DEBUG-motion]` C instrumentation (`stepcompress.c:31-35, 78-81, 260-330`, `itersolve.c:37-70`) already prints `%.17g` values of the solver and compressor state; it is usable as a trace oracle on error paths only, because it logs only when a failure occurs (`dbg_reported`).
- For non-failure step comparison use the public API instead: `stepcompress_extract_old` and `trapq_extract_old` (`motion_report.DumpStepper/DumpTrapQ`), or simply compare `queue_step` lines from `parsedump`.

## 7. Ender-3 V3 KE deployment

### 7.1 Platform facts

- Pad (Nebula Pad): MIPS Ingenic xburst2, Linux 4.4.94 SMP PREEMPT (`printer-backups/.../20260823-1329/manifest.txt`), glibc 2.29, o32 NaN2008 ABI (`ld-linux-mipsn8.so.1` in `pad_sysroot/lib/`; README says "MIPS NaN2008"), BusyBox `linuxrc` init with SysV scripts, squashfs `/rom` (100 percent full) with an overlay, 5.9 GB `/usr/data` for user data. Hostname `Ender3V3KE-D07E`.
- Python on the pad: 3.8 (`/usr/lib/python3.8/threading.py` in the stack in `stepcompress-debug/logs/capture-20260917-183026Z/klippy-mainline.log:34131`), virtualenv `/usr/share/klippy-env`. The log shows 9 threads and VmRSS about 25 MB for the mainline klippy.
- Main board MCU: GD32F303xe at 120 MHz, UART `/dev/ttyS1` at 230400 baud, stock Creality firmware built from Klipper `38d96adc` (Oct 2023). `restart_method: command`. A CRC mismatch between the stock firmware config and mainline's config forces an automated MCU reset on every klippy start (`mcu.py:690-691`, `1207-1210`; log line "Attempting automated MCU 'mcu' restart: CRC mismatch"). This is the handshake the Rust host must reproduce exactly: `get_config` returns `is_config=1` with a different `crc`, host sends `reset`/`config_reset`, exits with `firmware_restart`, reconnects and sends the full config.
- Second MCU `mcu rpi`: the Linux-process firmware (`klipper_mcu_new -r`) on the pad, endpoint `/tmp/klipper_host_mcu` (pipe connection, `serialhdl.connect_pipe`). It provides SPI for the ADXL345 (`spi_bus: spidev2.0`, `cs_pin: rpi:None` in `config/ender-3-v3-ke/input_shaper.cfg`), 50 MHz clock.
- Probe: BLTouch on `PC14`/`PC13`. Heaters, TMC2208 UART steppers, filament switch, fans per `config/ender-3-v3-ke/printer.cfg`, `fan.cfg`. Includes `/usr/data/mainsail-config/mainsail.cfg`. `[virtual_sdcard] path: /usr/data/printer_data/gcodes`.
- The README says the pad is "Klipper mainline HEAD 616242d4" (the early baseline); today's tree is `e47b0943` plus uncommitted state.

### 7.2 Build of the native pieces for the pad

- `c_helper.so`: command in `config/ender-3-v3-ke/README.md:33-55`: `mipsel-linux-gnu-gcc -Wall -g -O2 -shared -fPIC -mnan=2008 -D_TIME_BITS=32 -D_FILE_OFFSET_BITS=32 -nostdlib -nostartfiles -Wl,--as-needed -static-libgcc -flto -fwhole-program -fno-use-linker-plugin -o c_helper.so <21 sources> ../../pad_sysroot/lib/libc-2.29.so ../../pad_sysroot/lib/libgcc_s.so.1`. It needs the manual header workaround `stubs-o32_hard_2008.h`. Note it does not pass `-mfp64` (the MCU build below does).
- `klipper_mcu` (`scripts/build-mips-nan2008-klipper.sh`): `-mnan=2008 -mfp64 -flto=auto -fwhole-program`, custom `_start` (`scripts/mips_nan2008_start.c`), compat shims (`scripts/mips_nan2008_compat.c`: `openpty`, `timer_create`/`timer_settime` through `setitimer`, `pthread_create` returning `ENOSYS`), links against the pad's `libc-2.29.so`, dynamic linker `/lib/ld-linux-mipsn8.so.1`. Output copied to `out/klipper.elf` and `out/klipper.dict`.
- `[INFERENCE]` For a Rust binary: target is o32 little-endian MIPS with NaN2008 and FP64, glibc 2.29 dynamic linker `ld-linux-mipsn8.so.1`. The nearest tier-3 Rust triple is `mipsisa32r6el-unknown-linux-gnu` (R6 uses NaN2008 and the `mipsn8` loader); a custom target JSON may be needed. The pad has no recent libc, so link against the `pad_sysroot` libc, or build with musl as a static binary, which removes the glibc 2.29 constraint. Both options need a real cross-test on the device.

### 7.3 Boot, handover and supervision (all fork-added)

Order on a normal boot:
1. Stock services start from `/etc/init.d` (`S55klipper_service`, `S56moonraker_service`, `S57klipper_mcu`). They must stay enabled; disabling them breaks boot and SSH (`SERVICES.md`).
2. `/etc/init.d/S99zz_mainline_switch` (`scripts/S99zz_mainline_switch`, 882 B) starts `/usr/data/wait-creality-start-mainline.sh` in the background (PID file `/var/run/mainline-switch.pid`, log `/usr/data/printer_data/logs/mainline-switch.log`).
3. `scripts/wait-creality-start-mainline.sh` (2.5 KB) polls the stock UDS `/tmp/klippy_uds` with a JSON-RPC `objects/query {"webhooks": null}` (frame terminated by `\x03`) until `webhooks.state == "ready"`; stops Moonraker; then up to 3 attempts: stop mainline klippy, stop mainline MCU, `S57klipper_mcu_mainline start`, `S55klipper_mainline start`, wait up to 75 s for `ready` on the same socket path; on success starts Moonraker; on failure after 3 attempts stops everything and restarts Moonraker.
4. `/usr/data/S57klipper_mcu_mainline` (`scripts/S57klipper_mcu_mainline`, 1.1 KB): stops stock services, runs `screen -dmS klipper_mcu_mainline sh -c "while true; do /usr/data/klipper_mcu_new -r >>/usr/data/klipper_mcu_mainline.log 2>&1; rm -f /tmp/klipper_host_mcu; sleep 1; done"`, waits up to 15 s for `/tmp/klipper_host_mcu`. This loop is the only process supervision in the stack; it respawns the MCU process.
5. `/usr/data/S55klipper_mainline` (source: `config/ender-3-v3-ke/S55klipper_mainline`) is the klippy service. It waits up to 30 s for `/tmp/klipper_host_mcu`, rotates `klippy-mainline.log` through `.1` to `.3`, deletes `/tmp/klippy_uds`, then runs `screen -dmS klipper_mainline sh -c "/usr/share/klippy-env/bin/python /usr/data/klipper-mainline/klippy/klippy.py /usr/data/printer_data/config-mainline/printer.cfg -a /tmp/klippy_uds 2>&1 | tee /usr/data/klippy-mainline.log"`. There is no respawn loop for klippy and no `-l` option, so the log goes through stdout/stderr and `tee` with the `INFO:root:` prefix (the format in all captures), and `klippy.py:main()` logs "No log file specified! Severe timing issues may result!".
6. `scripts/S55klipper_mainline_direct` (a separate wrapper, not the klippy service): a direct-boot variant that removes stale sockets, starts the MCU service, waits up to 30 s for `/tmp/klipper_host_mcu`, starts the klippy service and restarts Moonraker. `scripts/start-mainline.sh` (201 B) is the manual start (stops stock services, starts MCU then klippy, restarts Moonraker). `scripts/klipper-start.sh`, `klipper-mcu.service`, `install-*.sh`, `Dockerfile` are upstream installers and are unused on the pad.

In-process restarts: `RESTART`, `FIRMWARE_RESTART` and the CRC-mismatch reset all end `Printer.run()` with `restart`/`firmware_restart`, and `klippy.py:main()` loops inside the same process (sleep 1 s, `gc.collect`, new `Reactor` and `Printer`). A Rust binary can either copy that loop or exit and rely on a supervisor, but then `S55klipper_mainline` has to become a respawn loop like the MCU one, or the handover script needs to wait for readiness again.

### 7.4 Moonraker and Mainsail

Moonraker 0.10.0 and Mainsail 2.17.0 (port 4409) installed with the Guilouz Creality Helper Script; Moonraker config `/usr/data/printer_data/config/moonraker.conf` (copy in `config/ender-3-v3-ke/moonraker.conf`): `klippy_uds_address: /tmp/klippy_uds`, port 7125, `[machine] provider: supervisord_cli` with `validate_service: False`. Moonraker connects as a client to klippy's UDS. Contract: JSON objects terminated by `\x03`, documented in `docs/API_Server.md`; Moonraker uses `info`, `objects/list|query|subscribe`, `gcode/script`, `gcode/subscribe_output`, `gcode/help`, `gcode/restart`, `gcode/firmware_restart`, `emergency_stop`, `register_remote_method`, `query_endstops/status`, `pause_resume/*`, `bed_mesh/dump_mesh`, and motion-report mux endpoints `[INFERENCE from upstream behavior; confirm in the captured moonraker.log]`. `stepcompress-debug/logs/capture-*/moonraker.log` is available to check which it actually calls.

### 7.5 Creality display (`c440x`) contract

The stock display stack (closed binaries `display-server`, `app-server`, `master-server` running on the pad) connects to `/tmp/klippy_uds` with `client_info {"program":"c440x","version":"V1.1.0.17"}`. The request sequence recorded in `stepcompress-debug/logs/klippy-crash1-stepper_x.log:2803-2833` is:
1. Repeated `shakehands` polls about once per second until klippy answers.
2. `list_endpoints`, `info` with `client_info`, `objects/list`.
3. `register_remote_method` for `shutdown_machine`, `reboot_machine`, `pause_job_queue`, `start_job_queue`, `timelapse_newframe`, `timelapse_saveFrames`, `timelapse_render` (klippy calls them through the macro helper `action_call_remote_method`, `klippy/extras/gcode_macro.py:99-112`).
4. `gcode/subscribe_output` with response template `process_gcode_response`.
5. `get_mesh` (returns `{probed_matrix}`).
6. `objects/subscribe` on a long list: `gcode`, `webhooks`, `configfile`, `mcu`, `mcu rpi`, `fan`, `heaters`, `heater_fan nozzle_fan`, `gcode_move`, `print_stats`, `virtual_sdcard`, `pause_resume`, `display_status`, and many `gcode_macro <NAME>` objects (`G29`, `PRINT_START`, `START_PRINT`, `PRINT_END`, `END_PRINT`, `LOAD_FILAMENT`, `UNLOAD_FILAMENT`, `CANCEL_PRINT`, `PAUSE`, `RESUME`, `SET_PAUSE_NEXT_LAYER`, `SET_PAUSE_AT_LAYER`, `SET_PRINT_STATS_INFO`, `_TOOLHEAD_PARK_PAUSE_CANCEL`, ...). Names come from `config/ender-3-v3-ke/gcode_macro.cfg` and Mainsail's macros.
7. `pause_resume/check_continue_print_state` (answered `{file_state:false, eeprom_state:false}`).
8. Later: `objects/query` for `heaters`, `gcode: [commands]`, `extruder: [can_extrude]`, `configfile: [settings]` / `[config]`, `toolhead`, `print_stats`, `gcode_move`, `virtual_sdcard`, `display_status`, `extruder: [target, temperature]`, plus `gcode/script` with `SDCARD_PRINT_FILE FILENAME="..."` to start a print, and `getBootLoaderVersion` (`pause_resume.py:36-37`).

Files that implement these endpoints (all fork additions, section 8): `klippy/webhooks.py:321,345-352` (`shakehands`: `{state, state_message, shakehands_id: web_request.id, curtime: reactor.monotonic()}`), `klippy/extras/bed_mesh.py:129,135-136` (`get_mesh`), `klippy/extras/pause_resume.py:33-43` (`check_continue_print_state`, `getBootLoaderVersion`, reads MCU constant `software_version`).

### 7.6 What must change to drop in a Rust binary

1. CLI compatibility: positional config path, `-a <uds>`, `-l`, `-v`, `-i`, `-o`, `-d`, `-I`, `--import-test` (only if kept for CI). The pad service passes only `<cfg> -a /tmp/klippy_uds`.
2. Replace `PYTHON` and `KLIPPY` in `config/ender-3-v3-ke/S55klipper_mainline`, and drop the Python venv; the binary must be executable on the pad, with the C helper folded in (no `c_helper.so`, no `gcc` at runtime: upstream's `check_build_c_library` would try to compile on the pad if the library were stale).
3. Keep the same sockets: serial `/dev/ttyS1` 230400 baud, pipe `/tmp/klipper_host_mcu`, UDS `/tmp/klippy_uds` with `\x03` framing, socket permissions as Moonraker expects.
4. Keep the stdout/stderr log format if anything greps it. The handover script does not grep logs; it only probes the socket.
5. Preserve the in-process restart loop or add a supervisor.
6. The `[DEBUG-motion]` log lines are fork-specific and only consumed by humans. Port them only if the same field diagnostics are needed.
7. Memory: klippy uses about 25 MB RSS; the pad has about 98 MB tmpfs and little flash headroom (`/rom` full). A static Rust binary should fit under `/usr/data`.
8. `S55klipper_mainline` and `S57klipper_mcu_mainline` use Entware `screen`; no change needed there.

## 8. Fork-specific changes in this slice

Source of the list: `.git/logs/refs/heads/ender-3-v3-ke` (author Gabriel-Hiss). Hashes are short forms of the reflog entries. The first debug set (`36935fd9`..`e666233a`) was abandoned by `reset` and re-committed as `7777d104`..`e47b0943`. File lists are `[INFERENCE]` from commit messages and from `[DEBUG-motion]` markers found by grep; I could not run `git show --stat`.

| Commit | Message | What it touches (inferred) |
|---|---|---|
| `5900c356` | chore: add ke deps | Pad sysroot, MIPS build helpers |
| `ccc10c37`, `9fa2b8d7`/`9557f2c8` | chore: add tutorial..., add tutorial on how to create auto booting services | `config/ender-3-v3-ke/README.md`, `SERVICES.md` |
| `92a20be0`, `fffb74dd`, `0e3a530b`, `481fc482` | TODO list, redirect README.md, remove start steps, todo and calibration steps | `config/ender-3-v3-ke/TODO.md`, root `README.md` |
| `de6acaac` | chore: update SERVICES.md | `SERVICES.md` |
| `9b3499b2` | chore: i'm not making the load cell work... | load-cell include commented out in `printer.cfg` |
| `78b05289` | feat(InputShaper): Get the input shaper working | `config/ender-3-v3-ke/input_shaper.cfg` |
| `b677987d`, `8b3f3fcb` | chore: update printer.cfg (delayed gcode) | `config/ender-3-v3-ke/printer.cfg` |
| `7c8d59f1` | merge master into ender-3-v3-ke | upstream sync |
| `3a63ec4a` | feat(creality): support stock c440x display endpoints | `webhooks.py` `shakehands`; `bed_mesh.py` `get_mesh`; `pause_resume.py` `check_continue_print_state`, `getBootLoaderVersion` |
| `4f9c676c` | fix(ender3-ke): restore calibrated printer configuration | `printer.cfg` SAVE_CONFIG block (z_offset 2.162, PID, twist compensation) |
| `d5c37cf9` | feat(ender3-ke): add supervised mainline handover | `scripts/S99zz_mainline_switch`, `wait-creality-start-mainline.sh`, `S57klipper_mcu_mainline`, `S55klipper_mainline_direct`, `start-mainline.sh` |
| `152aa00c` | fix(ender3-ke): gate moonraker on mainline readiness | `wait-creality-start-mainline.sh` (stop Moonraker, restart after ready) |
| `7777d104` | feat(debug): preserve shutdown provenance | `klippy/mcu.py:768-1030` (`DEBUG_SHUTDOWN_VERSION`, frozen first shutdown report, timeout capture, extra log lines in `_analyze_shutdown`), `klippy/klippy.py` (`_debug_host_state`, `_debug_shutdown`, `setup_fault_handler` with `faulthandler`) |
| `aac1802d` | feat(debug): trace host motion scheduling | `klippy/reactor.py:13-219` (stall, late-timer, GC rings), `klippy/extras/motion_queuing.py` (batch timing rings, `DebugFlushBatch`, `_flush_handler` wrapper around `_flush_handler_body`), `klippy/extras/virtual_sdcard.py` (chunk timing) |
| `4eec735c` | fix(motion): tolerate host scheduling stalls | `[INFERENCE]` `toolhead.py:195` `BUFFER_TIME_HIGH = 2.0` (was 1.0) |
| `3bc87895` | feat(debug): capture step compression state | `klippy/chelper/stepcompress.c` (rings `dbg_append[24]`, `dbg_event[8]`, `debug_dump`), `klippy/chelper/itersolve.c:37-70` |
| `cf3c4ddb` | feat(debug): trace serial dispatch decisions | `klippy/chelper/serialqueue.c` (`sq_debug_note`, `DEBUG_SERIAL_VERSION`, ring buffers, dump at exit) |
| `7d0860db` | chore(ender3-ke): retain mainline diagnostic logs | `config/ender-3-v3-ke/S55klipper_mainline` log rotation, `SERVICES.md` |
| `e47b0943` (HEAD) | fix(motion): keep producer ahead of step generation | `[INFERENCE]` `motion_queuing.py:11-12` `BGFLUSH_SG_LOW_TIME=1.000`, `BGFLUSH_SG_HIGH_TIME=1.250` (were 0.450/0.700) |

Behavior-changing items the port must decide on: the two constants above; the Creality endpoints; the `_flush_handler` wrapper (instrumentation only, policy lives in `_flush_handler_body`). The instrumentation in `stepcompress.c` records into fixed rings on every `queue_append`; I found no change to the compression arithmetic, but cannot prove it without a diff.

Files in the pad backups (`printer-backups/.../20260823-1329/klipper-mainline-live/`) include `prtouch_ke_native.py` (106.8 KB, Creality PRTouch module, option `creality_touch_correction`), `bltouch.py` and an older full `klippy/chelper/` copy. The module is not in `klippy/extras/` now; the Ender config uses BLTouch. `prtouch_ke_native_backups/` at the repo root holds more copies. `.idea/shelf/*.patch` (five shelves, up to 140 KB) contain earlier diffs.

## 9. Host behaviors with no test (needs new scenario coverage)

Grouped by area. "No test" means no `.test` file or CI step exercises it.

- Serial protocol and MCU responses: `msgproto` encode/decode against real replies, VLQ edge cases (values near 2^28, negative values, `%*s` buffers), `identify` handshake, retransmit/NAK/timeout logic in `serialqueue.c`, `RECEIVE_WINDOW` flow control, CAN and UART connect paths, `clocksync` regression (`ClockSync._handle_clock`, `SecondarySync`, `calibrate_clock` between two MCUs), `trsync` and `trdispatch` timeouts, `MCU_endstop` homing with real trigger times, `check_timeout`, shutdown handling (`_handle_shutdown`, `is_shutdown`, restart methods). Batch mode skips all of them (section 2.3).
- Timing: real-clock flush scheduling (`_flush_handler_body`, BGFLUSH constants), `BUFFER_TIME_*` and lookahead priming, `print_stall`, `idle_timeout`, `delayed_gcode`, `virtual_sdcard` pacing, reactor stalls and GC checking. The incident evidence (`Stepper too far in past`, `Invalid sequence`) comes from exactly this layer.
- Thermal: PID loop, `PID_CALIBRATE`, bang-bang, `verify_heater`, `temperature_fan`, `heater_fan`, sensor math from ADC (thermistors, MAX31855/6675/1xxx, PT100, DS18B20 responses), `M109/M190` waiting, `TEMPERATURE_WAIT`, `HEATER_INTERRUPT`, `homing_heaters`.
- Homing and probing with real timing: trigger-time arithmetic (`homing.py`, `probe.py`, `query_endstops`), `safe_z_home`, `homing_override`, `endstop_phase`, sampling retries, `PROBE_ACCURACY`, `Z_ENDSTOP_CALIBRATE` flows beyond canned dummies. The Ender's BLTouch sequence (stow, touch mode, `probe_with_touch_mode`) is not exercised; `bltouch.test` uses another config.
- Config handling: `SAVE_CONFIG` write-back and regeneration of the `#*#` block (`configfile.py` `cmd_SAVE_CONFIG`), `[include]` globbing and cycle detection, `config/ender-3-v3-ke/printer.cfg` as a whole, `check_unused_options` messages, `deprecate`.
- G-code layer: M112 out-of-order handling with more than 20 pending lines (`gcode.py` `GCodeIO._process_data`), line numbers and checksums (`N`, `*`), `M110`, `M117`/display, `G2/G3` edge cases beyond `gcode_arcs.test`, `G10/G11` retraction, `SET_GCODE_VARIABLE`, templates with `printer.*` status dictionaries (Jinja2 behavior is a large surface for a Rust template engine), `action_*` helpers, `RESPOND`, `SAVE_VARIABLE`.
- API server and webhooks: everything. `webhooks.py` (client connections, subscriptions, `objects/subscribe` batching, `register_remote_method`, `call_remote_method`, `gcode/subscribe_output`, `emergency_stop`), the Creality endpoints, `bed_mesh/dump_mesh`, `motion_report` dumps, `bulk_sensor` streaming. None of it runs in batch mode because `-a` is never passed.
- Status model: `get_status()` of every module is only exercised incidentally through macros (`macros.cfg`) and never compared. There is no test that queries `objects/query` and diffs the result.
- Pause/resume and print control: `PAUSE/RESUME/CANCEL_PRINT`, `pause_resume.py`, `print_stats` transitions, `display_status`, `M600`, `filament_switch_sensor` runout, `filament_motion_sensor`.
- Sensors and signal processing: resonance testing (`TEST_RESONANCES`, `SHAPER_CALIBRATE`, `shaper_calibrate.py` numpy math), accelerometer streaming (`adxl345`, `lis2dw`, `mpu9250`, `icm20948`, `bmi160` bulk helpers), `angle` calibration, `load_cell`/`load_cell_probe` numerics, SOS filter design via scipy (`trigger_analog.py:80-110`; `load_cell.test` only builds with dummy data), `tsl1401cl`/`hall` filament width sensors, `temperature_mcu`, `temperature_combined`.
- Misc modules never mentioned in tests: `skew_correction`, `axis_twist_compensation` (used by the Ender), `z_thermal_adjust`, `firmware_retraction`, `tuning_tower`, `force_move`, `fan_generic`, `controller_fan`, `output_pin`/`pwm_tool` beyond `pwm.test`, `servo` beyond SET_SERVO, `palette2`, `sdcard_loop` timing, `display/*` drivers and menu (only `led.cfg` has a `display_template`), `samd_sercom`, `canbus_stats`, `static_pwm_clock`, `multi_pin`, `replicape`, `adc_scaled`, `sx1509`, `mcp*`, `ad5206`, `dac084S085`, `gcode_button`, `save_variables`, `query_adc`, `statistics`, `garbage_collection`. (Some of these are loaded by the roughly 199 example configs in `printers.test`, so config parsing is covered even when behavior is not.)
- Multi-process and platform behavior: the Linux MCU (`mcu rpi`) pipe connection; SPI via spidev; `restart_method: command`; CRC-mismatch restart path; graceful shutdown on SIGTERM/SIGINT; `util.create_pty` input pipe `/tmp/printer`.
- Fork additions: `shakehands`, `get_mesh`, `check_continue_print_state`, `getBootLoaderVersion`, the [DEBUG-motion] shutdown capture logic (smoke-tested only by the out-of-repo scripts whose outputs are in `stepcompress-debug/logs/verification-v4-*`).

## 10. Strongest parity oracle available today, and its gaps

Description. The best oracle already in the repo is Python klippy in batch mode, run per scenario as:

`python klippy/klippy.py <cfg> -i <gcode> -o <out> -d <dict> [-d mcu=<dict>] -l <log>`

and decoded with `python klippy/parsedump.py <dict> <out>`.

It is hardware-free, runs in seconds per scenario, has the largest breadth in the repo (about 230 scenario runs: 36 per-feature tests plus about 199 printer configs from `printers.test`), and covers the whole outbound path from config parsing through lookahead, kinematics, `itersolve`, `stepcompress`, `steppersync` and `serialqueue` to MCU commands. All C code runs from `klippy/chelper`, so the stream also reflects the C helper arithmetic that a Rust port has to reproduce. Exit status already catches config, pin, G-code and shutdown errors.

To use it as an oracle, add these missing pieces (all small):
1. Capture and commit golden `parsedump` text for each scenario, plus the normalized log lines (`Loaded MCU`, `Configured MCU`, `Exiting (print time ...)`, `respond_info` texts), together with a manifest of Python, cffi, greenlet, glibc versions and the `klippy` tree hash.
2. Commit or fetch the 28 dictionaries (section 4) into `tests/fixtures/`.
3. Add a determinism check: run the Python reference N times and prove the decoded stream is identical, or record which message pairs swap (section 5.2, cross-queue order).
4. Add a Rust-side runner with the same CLI contract and a diff tool that reports the first divergence per `oid`.
5. Extend with two deeper layers that already have data: (a) the `trapq_real.json` + `grid_real.json` replay for kinematics plus step compression, comparing x86 C, MIPS C and Rust `queue_step` output, which is the most failure-prone numeric core and has real captured data; (b) `parsedump` per-oid totals compared with the logic of `scripts/stepstats.py` as a coarse signal.

Gaps that remain even with that:
- No MCU replies or real time (sections 2.3, 9): clock sync, retransmit, trsync/homing, heaters, bulk sensors and the whole API server are untested; batch mode never enters the code paths whose timing bugs motivated the fork changes.
- No oracle for the Ender-specific main MCU dictionary, so wire-level IDs for the real printer cannot be checked offline.
- No oracle for the status model and API: need a second harness that boots both implementations against a scripted fake MCU (a Python mock serial peer or the Linux-process MCU on a pty) and replays the c440x/Moonraker request sequence (section 7.5) with `objects/query`/`objects/subscribe` diffs. The recorded request lines in `stepcompress-debug/logs/klippy-crash1-stepper_x.log:2803-2833` and `moonraker.log` are the seed.
- Floating-point equivalence is only guaranteed against the x86 SSE2 build; the pad runs a MIPS build whose FMA contraction and libm may differ `[INFERENCE]`, so "parity" has to be defined per target.
- Goldens drift with every fork change to `motion_queuing.py`/`toolhead.py` constants, so pin the tree hash and re-baseline deliberately.
- The Windows host cannot run the reference; CI for the swarm needs Linux or WSL.

## 11. Cross-cutting risks and port order (for this slice)

Risks:
1. Local-only artifacts (`stepcompress-debug/`, `printer-backups/`) are git-excluded; copy the needed data (`trapq_real.json`, `grid_real.json`, key captures, `out/klipper.dict`) into tracked fixtures before the swarm starts, and rewrite the missing harness.
2. Message stream order is partly racy because of the `serialqueue` thread; the oracle must be built on decoded, normalized text, and its stability must be measured before any Rust diff is trusted.
3. Fork constants (BGFLUSH_SG, BUFFER_TIME_HIGH) are undocumented; re-baseline after deciding.
4. Float and libm semantics (section 5.2) decide whether "byte-exact" or "within one clock" is the acceptance rule. Decide per layer: strict for `trapq`/`itersolve`/`stepcompress` on x86; documented tolerance elsewhere.
5. Jinja2 macros (`gcode_macro.py`) are a hidden interpreter inside every config (the Ender's `START_PRINT` uses `printer.*` and filters like `|float`, `|default`); parity needs a corpus of macro snippets with expected outputs, which does not exist.
6. scipy-derived SOS filter coefficients (`trigger_analog.py:109-110`) and numpy-based resonance code have no Rust ecosystem equivalent; produce reference vectors from Python once and freeze them.
7. The CRC-mismatch reset handshake on the real printer is outside any test; test it with a mock MCU before trusting the Rust binary on hardware. Remember the incident: hardware tests must run only with explicit per-run permission from the owner.
8. MIPS NaN2008 cross-compilation of Rust is not validated anywhere in the repo.

Suggested order (leaves first):
1. Fixtures and tooling: dictionaries, golden capture script (Python), normalizer and diff tool, determinism measurement of the Python reference.
2. `msgproto` decode/encode and the block framing, validated against `parsedump.py` on the captured streams (this makes every later comparison trustworthy).
3. Replay harness for `trapq_real.json` driving the C library, then the Rust `trapq` + `itersolve` + `stepcompress` port compared against it.
4. Batch-mode skeleton: CLI, config reader, G-code reader, `mcu` file output with the stubs listed in 2.3, run the 37 tests for exit status.
5. Golden diff on `printers.test` configs (broad config/pin coverage), then per-feature tests.
6. Mock-MCU and API-server harness for responses, timing, status model and the c440x sequence.
7. Pad packaging: MIPS target, service-script changes (7.6), handover test on hardware with explicit permission.
