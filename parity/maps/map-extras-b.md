# Klippy port map: `klippy/extras/` files f..o (slice B)

## 0. Scope, method and caveats

**Range.** Listing of `klippy/extras/` plus `klippy/extras/display/` (the only subdirectory). Names starting with `f` through `o`:

- First file: `klippy/extras/fan.py`. Last file: `klippy/extras/output_pin.py`.
- Subdirectory `display/` has six files in range: `font8x14.py`, `hd44780.py`, `hd44780_spi.py`, `menu.py`, `menu_keys.py`, `menu.cfg`. The rest of `display/` (`display.py`, `display.cfg`, `aip31068_spi.py`, `st7920.py`, `uc1701.py`) is outside the range.
- Skipped per instructions (one-line mention only, section 12): `force_move.py` (140 lines), `gcode_move.py`, `homing.py`, `input_shaper.py`, `manual_stepper.py`, `motion_report.py`.
- 36 top-level files + 6 `display/` files = 42 files. About 5,850 lines at top level and about 2,490 in `display/` (of which 276 is a font table and 782 is `menu.cfg`).

**Method.** Every file was read (`menu.py` in part plus a structural grep; `font8x14.py` is a data table). Line counts come from the read tool's "N lines total". No git CLI was available, so fork attribution rests on markers in the code, the reflog (`.git/logs/HEAD`) and file modification order. Claims not proven by the source are tagged `[INFERENCE]`.

**Fork delta summary (this slice).** Only one file in range carries fork changes: `klippy/extras/motion_queuing.py`. Evidence: it is one of two newest-modified files in `extras/` (the other is `virtual_sdcard.py`, outside range); it contains `[DEBUG-motion] v4` instrumentation and non-upstream timing constants (section 4.1). Everything else in range shows no fork markers. Fork-related files outside the range, for context: `klippy.py`, `mcu.py`, `reactor.py`, `toolhead.py`, `extras/virtual_sdcard.py`, `chelper/{stepcompress,serialqueue,itersolve}.c`, `config/ender-3-v3-ke/*`.

Reflog commits that most likely touch `motion_queuing.py` `[INFERENCE from commit messages only; confirm with git log -- klippy/extras/motion_queuing.py]`:

| short hash | message |
|---|---|
| `aac1802` | feat(debug): trace host motion scheduling |
| `4eec735` | fix(motion): tolerate host scheduling stalls |
| `3bc8789` | feat(debug): capture step compression state |
| `7777d10` | feat(debug): preserve shutdown provenance |
| `e47b094` | fix(motion): keep producer ahead of step generation (last entry in the reflog) |

The reflog also shows an earlier series (`36935fd`, `aef8165`, `67136ff`, `f43630a`, `5de84f3`, `e666233`) that was reset away and re-created as the series above. Upstream baseline for a diff: `616242d` (clone point of the fork branch); `git diff 616242d HEAD -- klippy/extras/motion_queuing.py` shows the exact fork delta.

**Fork printer configuration (target usage).** `config/ender-3-v3-ke/printer.cfg` uses these in-range modules: `[fan]`, `[heater_fan nozzle_fan]`, `[heater_bed]`, `[idle_timeout]` (with `gcode:`), `[filament_switch_sensor filament_sensor]` (`pause_on_runout: true`), `[gcode_macro ...]` (G29, PRINT_START, START_PRINT, PRINT_END, END_PRINT, LOAD_FILAMENT, UNLOAD_FILAMENT, plus Mainsail macros via include), `[delayed_gcode fan_off]`, and the auto-loaded `manual_probe`, `motion_queuing`, `garbage_collection`. The `load_cell.cfg` include is commented out (`#[include load_cell.cfg]`); the commit "chore: i'm not making the load cell work it's just annoying" left no source change in `load_cell*.py` that I could find. `[mcu rpi]` (host MCU at `/tmp/klipper_host_mcu`) is configured, so I2C/SPI sensors on the Linux host are the plausible sensor path.

**Test harness fact that matters for parity.** `scripts/test_klippy.py` runs `klippy.py <cfg> -i <gcode> -o _test_output -d <dict>` and only checks the exit status (`SHOULD_FAIL` flips it). It does not compare `_test_output` (the MCU command stream) with a golden file. So "covered by a test" below means "loads and runs without exception", not "output verified". The port needs its own golden captures of the `-o` stream.

**Conventions used below.**
- "Status" means the `get_status()` keys visible to Jinja as `printer["<section name>"]` and to Moonraker.
- Section name = printer object name (for example `led lled`, `output_pin x`); this is how macros and `SENSOR=` lookups find objects.
- Extras are loaded by name with `printer.load_object(config, name)`; `load_config(config)` serves `[name]` and `load_config_prefix(config)` serves `[name suffix]`. Loading is a dynamic import by string, so the port needs a static registry.
- `GCodeRequestQueue` callback protocol: the callback returns `None` (normal) or `(action, next_min_time)` with action in `discard|reschedule|repeat`. Documented under `output_pin` (section 4.2).

---

## 1. Families

| Family | Units |
|---|---|
| Motion queuing core | `motion_queuing`, `output_pin` (hosts `GCodeRequestQueue`), `idle_timeout`, `garbage_collection` |
| Macros, templates, G-code helpers | `gcode_macro`, `gcode_button`, `homing_override`, `gcode_arcs`, `firmware_retraction`, `display/menu*` (templates) |
| Heaters / temperature | `heaters`, `heater_bed`, `heater_generic`, `homing_heaters`; temperature sensors over I2C: `lm75`, `htu21d` |
| Fans | `fan`, `fan_generic`, `heater_fan` |
| Pins, digipots, LEDs | `output_pin`, `multi_pin`, `mcp4018`, `mcp4451`, `mcp4728`, `led`, `neopixel` |
| Filament sensors | `filament_switch_sensor`, `filament_motion_sensor`, `hall_filament_width_sensor` |
| Accelerometers (I2C/SPI bulk sensors) | `icm20948`, `mpu9250`, `lis2dw`, `lis3dh` |
| Load cells / probing / eddy | `hx71x`, `load_cell`, `load_cell_probe`, `ldc1612`, `manual_probe` |
| Displays / UI | `display/hd44780`, `display/hd44780_spi`, `display/font8x14`, `display/menu`, `display/menu_keys`, `display/menu.cfg` |
| TMC drivers | none in range (all `tmc*` start with `t`) |
| Bed leveling / calibration / analysis | none in range except `manual_probe`, `LDC_CALIBRATE_DRIVE_CURRENT` (`ldc1612`) and load-cell calibration (`load_cell`) |

## 2. Numpy / numeric library use (this slice)

| Module | Library | What it computes |
|---|---|---|
| `load_cell_probe.py` | `numpy` (lazy `import numpy as np` inside `LoadCellPrinterProbe.__init__`; config error `"[load_cell_probe] requires the NumPy module"`) | `np.asarray(samples, dtype=float64)` to split time/force columns in `TapAnalysis`; `np.average(np.array(tare_samples)[:, 2].astype(float))` for the tare value sent to the MCU. Nothing else. |
| `load_cell_probe.py` via `trigger_analog.DigitalFilter` (outside range) | `scipy.signal` (lazy; error "DigitalFilter require the SciPy module") | `iirnotch`, `tf2sos`, `butter(..., output='sos')` design SOS filter coefficients that are sent to the MCU (`MCU_SosFilter`). `trigger_analog.GeneratedSOS` has pre-generated tables for common cases; any other filter needs SciPy. |
| `load_cell.py` | none (comment "alternative to numpy's column selection") | pure-Python `select_column`, `avg`. |
| accelerometers, `ldc1612` | none | They only scale and round samples. Their consumers (`resonance_tester`, `shaper_calibrate`, `probe_eddy_current`, other slices) use numpy `[INFERENCE on the consumer side]`. |

No other module in range imports numpy, scipy or any numeric library. `math` is used by `gcode_arcs`, `ldc1612` and `load_cell_probe` only.

## 3. Jinja2 templates and Python expression evaluation (this slice)

- **Jinja2 import:** only `gcode_macro.py` imports `jinja2`. Environment: `jinja2.Environment('{%', '%}', '{', '}')` (block delimiters `{% %}`, variable delimiters `{ }`, default `{# #}` comments, no autoescape, no extensions). Every other consumer calls `gcode_macro.load_template(config, option, default)` and `TemplateWrapper.render(context)`.
- **Template consumers in range:** `gcode_macro` (macro bodies), `homing_override` (`gcode`), `idle_timeout` (`gcode`, default `DEFAULT_IDLE_GCODE` uses `{% if 'heaters' in printer %}`), `filament_switch_sensor` and `filament_motion_sensor` (`runout_gcode`, `insert_gcode`), `gcode_button` (`press_gcode`, `release_gcode`), `output_pin` (`PrinterTemplateEvaluator` renders `display_template` sections every 0.5 s), `fan_generic` and `led` (through that evaluator), `display/menu` (`name`, `enable`, `gcode`, `input`, `input_min`, `input_max` and event scripts), `menu.cfg` (about 100 templates using Python formatting such as `'%3d' % (menu.input*100)` and filters such as `menu.input|int`).
- **`ast.literal_eval`:** `gcode_macro` (`variable_*` values and `SET_GCODE_VARIABLE VALUE=`, then `json.dumps(..., separators=(',', ':'))` as a serializability check), `output_pin` (`PARAM_*` of `SET_PIN`/`SET_FAN_SPEED`/`SET_LED_TEMPLATE`), `display/menu` (`enable`, `input`, `input_min`, `input_max` template results). A Python literal parser is therefore needed (lists, dicts, tuples, sets, signed numbers, bytes, string escapes).
- **No `eval()`, `exec()` or `importlib`** in range. Dynamic behaviour is limited to `getattr`/`hasattr` (`menu.py` `handle_script_<name>`, `motion_queuing.debug_callback_name`).

---

## 4. Per-unit reports: core runtime

### 4.1 `motion_queuing.py` (509 lines)
1. **Size:** 509.
2. **Deps:** `chelper` (cffi `get_ffi()`), `collections`, `logging`. Printer objects: reactor, every `mcu` (`printer.lookup_objects(module='mcu')`), `toolhead` (callers). C functions used: `trapq_alloc/free/append/finalize_moves`, `steppersyncmgr_alloc/free/alloc_steppersync/gen_steps`, `steppersync_alloc_syncemitter/setup_movequeue/set_time`, `syncemitter_get_stepper_kinematics`, `itersolve_get_trapq`, `itersolve_get_gen_steps_pre_active/post_active`. Third party: cffi (through chelper).
3. **Contracts:**
   - No G-code, no config options, no `get_status`.
   - API used by `toolhead`, `stepper`, `force_move`, `output_pin` and others: `allocate_trapq`, `wipe_trapq`, `lookup_trapq_append`, `allocate_syncemitter(mcu, name, alloc_stepcompress)`, `setup_mcu_movequeue`, `stats(eventtime)` (calibrates mcu clocks, sets history expiry), `register_flush_callback(cb, can_add_trapq)` (callbacks with `can_add_trapq` run first), `unregister_flush_callback`, `get_kin_flush_delay`, `check_step_generation_scan_windows`, `flush_all_steps`, `calc_step_gen_restart`, `note_mcu_movequeue_activity(mq_time, is_step_gen=True)`, `drip_update_time(start, end, drip_completion)`, `check_drip_timing`.
   - Events: listens `klippy:shutdown` and `klippy:analyze_shutdown`.
   - Timing constants: `BGFLUSH_LOW_TIME=0.200`, `BGFLUSH_HIGH_TIME=0.400`, `BGFLUSH_SG_LOW_TIME=1.000`, `BGFLUSH_SG_HIGH_TIME=1.250`, `BGFLUSH_EXTRA_TIME=0.250`, `MOVE_HISTORY_EXPIRE=30`, `MIN_KIN_TIME=0.100`, `STEPCOMPRESS_FLUSH_TIME=0.050`, `SDS_CHECK_TIME=0.001`, `DRIP_SEGMENT_TIME=0.050`, `DRIP_TIME=0.100`.
   - Error path: `steppersyncmgr_gen_steps` returning non-zero calls `printer.invoke_shutdown("Internal error in stepcompress", details)` with `queue_name`, flush/step_gen times and `clear_history_time`, then raises `mcu.error`.
   - Log lines read by humans and tools: `[DEBUG-motion] version=host-v4 motion_queuing timing rings batches=%d slow=%d wakes=%d` (info at load); on shutdown `[DEBUG-motion] motion flush stage=... attempted_flush=...`, `[DEBUG-motion] version=host-v4 motion timing ...`, `[DEBUG-motion] motion batch pending|failed|worst|slow|recent ...`, `[DEBUG-motion] motion wake at=...`, plus lines from `reactor.get_timing_debug_report`. `config/ender-3-v3-ke/SERVICES.md` documents the `[DEBUG-motion]` records.
4. **Runtime traits:** reactor timer `flush_timer` runs the background flush; `_await_flush_time` and `drip_update_time` call `reactor.pause` (greenlet yield) and `drip_completion.wait(...)`; flush callbacks run inside `with reactor.assert_no_pause()`; timing depends on `reactor.monotonic()` and `mcu.estimated_print_time`; clock calibration (`mcu.calibrate_clock`) feeds C `steppersync_set_time`; in debug-output mode (`mcu.is_fileoutput()`), `_flush_handler_debug` batches faux-time and `can_pause=False`. Output-file parity depends on exact flush batching (`want_sg_time`, the `next_batch_time` "run-to-run reproducibility" branch). The heavy work is in C (`chelper`), so a Rust port either keeps the C ABI or ports `chelper` first.
5. **Tests:** every test exercises it (loaded by `toolhead.py` and `stepper.py`); the `.test` suite checks no output.
6. **Fork changes: YES.** (a) `BGFLUSH_SG_LOW_TIME`/`BGFLUSH_SG_HIGH_TIME` are 1.000/1.250 `[INFERENCE: upstream is 0.450/0.700 from my memory of upstream; verify with the diff against 616242d]`, which matches the commits "tolerate host scheduling stalls" and "keep producer ahead of step generation"; (b) `[DEBUG-motion] v4` instrumentation: `DebugFlushBatch` class, rings (`DEBUG_RING_SIZE=32`, `DEBUG_SLOW_RING_SIZE=16`, `DEBUG_WAKE_RING_SIZE=48`, thresholds 10 ms and 20 ms), `_flush_handler` wrapper around `_flush_handler_body`, `_debug_shutdown`, `_debug_dump_timing`, `debug_callback_name`; (c) depends on fork-only `reactor.get_timing_debug_report`. The constants change motion timing and must be reproduced; the debug rings are diagnostics but are documented in `SERVICES.md`.
7. **Difficulty: high.** Timing-critical, coupled to the C step generator and the reactor, and carries fork behaviour that is only partly provable from source.

### 4.2 `output_pin.py` (270 lines)
1. **Size:** 270.
2. **Deps:** `.display.display` (importing it pulls in the whole `display/` package: `hd44780`, `st7920`, `uc1701`, `menu`, ...), `motion_queuing`, `gcode_macro` (template context), `pins`, `toolhead`; stdlib `logging`, `ast`.
3. **Contracts:**
   - **`GCodeRequestQueue(config, mcu, callback)`** (shared by `fan`/`heater_fan`/`fan_generic` through `fan.Fan`, and by many modules in other slices): `queue_gcode_request(value)` registers a toolhead lookahead callback that appends `(print_time, value)` and calls `motion_queuing.note_mcu_movequeue_activity(print_time, is_step_gen=False)`; `send_async_request(value, print_time=None)` runs immediately using `mcu.estimated_print_time(now + min_schedule_time)`; the flush callback (registered with `register_flush_callback`) drains the queue up to `must_flush_time`, skipping requests overridden by later ones; callback return protocol `None | (action, next_min_time)` with `discard|reschedule|repeat`; enforces `next_min_flush_time = next_time + mcu.min_schedule_time()`.
   - **`PrinterTemplateEvaluator`** (printer object `template_evaluator`, created by `lookup_template_eval`): every `RENDER_TIME = 0.5` s it renders the active display templates, caches duplicates by `(template, sorted lparams)`, passes a `render()` helper into the context, then calls flush callbacks. Errors are logged as `display template render error`.
   - **`[output_pin name]`** config: `pin`, `pwm` (False), `cycle_time` (0.100, above 0, max = mcu max nominal duration), `hardware_pwm` (False), `scale` (1.0, PWM only), `value` (0), `shutdown_value` (0). Calls `setup_max_duration(0.)` and `setup_start_value(value, shutdown_value)`.
   - **G-code:** `SET_PIN PIN=<name> VALUE=<0..scale>` or `TEMPLATE=<display_template>` with `PARAM_<name>=<literal>`. Errors: `"SET_PIN command must specify VALUE or TEMPLATE"`, `"Invalid pin value"` (non-PWM must be exactly 0 or 1), `"Unknown display_template '%s'"`, `"Invalid display_template parameter: %s"`, `"Unable to parse '%s' as a literal"`.
   - **Status:** `{'value': last_value}`.
   - **MCU:** through `pins` (`pwm` or `digital_out` pins): `set_pwm(print_time, value)` / `set_digital(print_time, value)`. No direct MCU commands.
4. **Runtime traits:** reactor timer (`render_timer`); flush callbacks run inside the `motion_queuing` flush (no pausing allowed); Jinja templates and `ast.literal_eval` (section 3); a `{}.get(uid)` hashability probe decides whether to cache; `context.clear()` breaks reference cycles for the GC.
5. **Tests:** `pwm.test`/`pwm.cfg` (`SET_PIN` on soft and hard PWM pins; `cycle_pwm_pin` is `pwm_cycle_time`, another slice); printers.test loads many `[output_pin ...]` configs (for example `generic-mini-rambo` with `scale: 2.0`, `generic-duet2-duex` with hardware PWM); `led.test` uses `SET_LED_TEMPLATE` through the evaluator.
6. **Fork changes:** none found.
7. **Difficulty: high.** The request-queue semantics are subtle (override skipping, repeat/reschedule, min flush time) and the module is a dependency of fans, LEDs and templates.

### 4.3 `idle_timeout.py` (118 lines)
1. **Size:** 118.
2. **Deps:** `gcode_macro` (template), `toolhead` (`check_busy`, `get_last_move_time`), `gcode` (`run_script`, `get_mutex`), reactor. Auto-loaded by `toolhead.py`, so always present.
3. **Contracts:** config `timeout` (600, above 0), `gcode` (template; default `{% if 'heaters' in printer %} TURN_OFF_HEATERS {% endif %} M84`). G-code `SET_IDLE_TIMEOUT TIMEOUT=<s>` (response `"idle_timeout: Timeout set to %.2f s"`). Status `{state: Idle|Ready|Printing, printing_time, idle_timeout}`. Events sent: `idle_timeout:idle(print_time)`, `idle_timeout:ready(est_print_time+0.1)`, `idle_timeout:printing(...)`. Events received: `klippy:ready`, `toolhead:sync_print_time(curtime, print_time, est_print_time)`. Constants `PIN_MIN_TIME=0.1`, `READY_TIMEOUT=0.5`.
4. **Runtime traits:** reactor timer state machine; runs the idle gcode with `gcode.run_script` inside the timer (greenlet); bare `except:` logs `"idle timeout gcode execution"`; depends on `gcode.get_mutex().test()`.
5. **Tests:** loaded by every test; `printer-anycubic-kossel-*` configs set `[idle_timeout] timeout`.
6. **Fork changes:** none in source (fork config overrides `gcode:`).
7. **Difficulty: medium.** Small, but timer timing and event ordering must match; consumers (`filament_*`, `display_status`) read `get_status()["state"]`.

### 4.4 `garbage_collection.py` (31 lines)
1. **Size:** 31. 2. **Deps:** stdlib `gc`, `logging`. Auto-loaded by `toolhead.py`.
3. **Contracts:** no config, G-code or status. On `klippy:ready` it runs `gc.collect(0..2)` then `gc.freeze()`; on `klippy:disconnect` it calls `gc.unfreeze()`. Debug logs `"Running full garbage collection and freezing"` and `"Unfreezing garbage collection"`.
4. **Runtime traits:** specific to the CPython GC; meaningless in Rust.
5. **Tests:** none specific (loaded everywhere). 6. **Fork:** none.
7. **Difficulty: low.** Keep the module name registered as a no-op so the auto-load list and configs stay valid.

---

## 5. Per-unit reports: macros, templates, G-code helpers

### 5.1 `gcode_macro.py` (203 lines)
1. **Size:** 203.
2. **Deps:** third party **`jinja2`**; stdlib `traceback`, `logging`, `ast`, `copy`, `json`. Printer objects: `gcode` (`register_command`, `register_mux_command`, `is_traditional_gcode`, `run_script_from_command`, `error`), `webhooks.call_remote_method`, `printer.invoke_shutdown`, and every object with `get_status`.
3. **Contracts:**
   - `[gcode_macro NAME]`: `gcode` (required template), `description` ("G-Code macro"), `rename_existing`, `variable_<name>` (Python literal). The section name must have exactly two words (`"Name of section '%s' contains illegal whitespace"`).
   - Registers the command `NAME.upper()`. With `rename_existing`, on `klippy:connect` it takes over the existing command and re-registers the old handler under the new name (`"Existing command '%s' not found in gcode_macro rename"`; description `"Renamed builtin of '%s'"`). Rename between traditional (`G`/`M`) and extended commands is rejected.
   - `SET_GCODE_VARIABLE MACRO=<name> VARIABLE=<v> VALUE=<python literal>`; errors `"Unknown gcode_macro variable '%s'"`, `"Unable to parse '%s' as a literal: %s in '%s'"`. Values must also pass `json.dumps`.
   - Status: the variables dict (visible as `printer["gcode_macro NAME"].var`).
   - Context given to templates: `printer` (`GetStatusWrapper`: lazy, per-render cache, `copy.deepcopy` of each `get_status`, supports `in`, indexing and iteration), `action_emergency_stop(msg)`, `action_respond_info(msg)`, `action_raise_error(msg)`, `action_call_remote_method(method, **kwargs)`, `params` (dict of upper-case command parameters), `rawparams`, plus all macro variables as top-level names.
   - Error messages: `"Macro %s called recursively"`, `"Error loading template '%s'\nline %s: %s # %s"`, `"Error loading template '%s': %s"`, `"Error evaluating '%s': %s"`; exceptions are logged with `logging.exception`.
   - Public API for every other module: `load_template(config, option, default=None)` (returns `TemplateWrapper` with `render(context=None)` and `run_gcode_from_command(context)`) and `create_template_context(eventtime=None)`.
4. **Runtime traits:** **Jinja2** with custom delimiters (filters, tests, `{% set %}`, `{% for %}`, namespaces; filters used in the repo include `float`, `int`, `default`, `round`, `string`, `list`, `min`, `max`, `abs`, `replace`, `join`, `format`); Python object semantics inside templates (`'%3d' % x`, string methods like `.split()`, dict/list/tuple access, namedtuple attribute access such as `gcode_move.gcode_position.x`, `in` against the status wrapper); `ast.literal_eval`; `reactor.assert_no_pause()` while collecting statuses; recursion guard `in_script`; macro bodies run through `gcode.run_script_from_command` (nested G-code execution in the caller's greenlet).
5. **Tests:** `macros.test` + `macros.cfg` (expressions, `variable_*`, `SET_GCODE_VARIABLE`, `params`, `in printer`, unicode in names and comments, `action_respond_info`); macros in `generic_cartesian_*`, `dual_carriage`, `corexyuv`, `hybrid_corexy_dual_carriage`, `exclude_object` (`M486`) configs; `gcode_macro` is also loaded by `idle_timeout`, so every test touches the template path.
6. **Fork changes:** none in the module; fork `gcode_macro.cfg` holds user macros.
7. **Difficulty: high.** Reproducing Jinja2 (including custom delimiters) and Python object semantics is the largest single risk in the slice. Decide early between embedding a Jinja-compatible engine (for example `minijinja` with custom syntax) and a Python-interpreter shim, then validate against a corpus of real macros (`config/*.cfg`, `menu.cfg`, `display.cfg`, `sample-macros.cfg`, Mainsail/Fluidd macros).

### 5.2 `gcode_button.py` (56 lines)
1. **Size:** 56. 2. **Deps:** `buttons` (`register_debounce_button`, `register_debounce_adc_button`), `gcode_macro`.
3. **Contracts:** `[gcode_button NAME]`: `pin`, `analog_range` (2 floats, selects ADC mode), `analog_pullup_resistor` (4700), `press_gcode` (required), `release_gcode` (''), plus `debounce_delay` (read inside `buttons.py`). G-code `QUERY_BUTTON BUTTON=NAME` (response `"NAME: PRESSED|RELEASED"`). Status `{state: PRESSED|RELEASED}`. Template exceptions are swallowed with `logging.exception("Script running error")`.
4. **Runtime traits:** button callbacks run from reactor callbacks; templates run `gcode.run_script`.
5. **Tests:** config load only, via `printer-biqu-bx-2021.cfg` (printers.test).
6. **Fork:** none. 7. **Difficulty: low.**

### 5.3 `homing_override.py` (66 lines)
1. **Size:** 66. 2. **Deps:** `gcode_macro`, `homing` (loaded), `toolhead` (`get_position`, `set_position(pos, homing_axes=...)`), `gcode`.
3. **Contracts:** `[homing_override]`: `gcode` (required), `axes` ('XYZ'), `set_position_x|y|z`. It replaces `G28` by registering a new handler and keeping the previous one (`register_command("G28", None)` returns the previous handler). Behaviour: a recursive `G28` while the script runs calls the original; no axis parameter means override; otherwise override only when one of `axes` is requested; it sets the forced position and a `homing_axes` string before running the template; the context also carries `params` and `rawparams`.
4. **Runtime traits:** command-handler chaining (monkey-patch-like), `in_script` flag, template execution.
5. **Tests:** `sdcard_loop.cfg`/`sdcard_loop.test` (G28 with `homing_override`), `kit-voron2-250mm.cfg` and `kit-zav3d-2019.cfg` (printers.test).
6. **Fork:** none. 7. **Difficulty: low** (given a `gcode` module that returns previous handlers).

### 5.4 `gcode_arcs.py` (183 lines)
1. **Size:** 183. 2. **Deps:** `gcode_move` (`get_status`, direct `cmd_G1`), `gcode.create_gcode_command`; stdlib `math`.
3. **Contracts:** `[gcode_arcs]`: `resolution` (1.0 mm per segment, above 0). G-code `G2`, `G3` (X Y Z I J K E F; `R` rejected), `G17`, `G18`, `G19` (plane select, default XY). Errors: `"G2/G3 does not support relative move mode"`, `"G2/G3 does not support R moves"`, `"G2/G3 requires IJ, IK or JK parameters"`. Output is a series of synthetic `G1` commands with float parameters X, Y, Z, E, F.
4. **Runtime traits:** **float-sensitive**: `atan2`, `hypot`, `cos`, `sin`, `floor`; the generated coordinates go through `gcode_move` to the MCU, so last-ulp libm differences between CPython's libm and Rust's `f64::sin/cos/atan2` can change step counts at boundaries. Source note `# todo: sometimes produces full circles`. The last segment uses `c = targetPos` (exact target).
5. **Tests:** `gcode_arcs.test` + `gcode_arcs.cfg` (XY, XZ, YZ planes, helical, partial IJ/IK/JK forms).
6. **Fork:** none. 7. **Difficulty: medium.** Small and test-covered, but the libm question needs a decision (use the same libm, or verify within a tolerance).

### 5.5 `firmware_retraction.py` (74 lines)
1. **Size:** 74. 2. **Deps:** `gcode` (emits G-code text for `gcode_move`).
3. **Contracts:** `[firmware_retraction]`: `retract_length` (0), `retract_speed` (20, min 1), `unretract_extra_length` (0), `unretract_speed` (10, min 1). G-code `G10`, `G11`, `SET_RETRACTION [RETRACT_LENGTH RETRACT_SPEED UNRETRACT_EXTRA_LENGTH UNRETRACT_SPEED]`, `GET_RETRACTION` (response `RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f`). Status: the four values (not `is_retracted`). `G10`/`G11` emit text: `SAVE_GCODE_STATE NAME=_retract_state`, `G91`, `G1 E-%.5f F%d` (or `E%.5f`), `RESTORE_GCODE_STATE NAME=_retract_state`, with `F = speed*60` formatted `%d` (truncation).
4. **Runtime traits:** string formatting parity only (`%.5f`, `%d` truncates toward zero).
5. **Tests:** config load only (`printer-lulzbot-mini1-2016.cfg`); no `G10`/`G11` command test.
6. **Fork:** none. 7. **Difficulty: low.**

---

## 6. Per-unit reports: heaters, temperature, fans, pins

### 6.1 `heaters.py` (392 lines)
1. **Size:** 392. 2. **Deps:** stdlib `os`, `logging`, `threading`; printer objects `pins` (`pwm`), `gcode`, `toolhead`, `configfile` (`read_config`), `verify_heater <name>` and `pid_calibrate` (loaded by name; other slices), and `temperature_sensors.cfg` in the same directory (another slice owns the file).
3. **Contracts:**
   - **Heater config** (used by `heater_bed`, `heater_generic`, `extruder` in another slice): `sensor_type`, `min_temp` (required), `max_temp` (required), `min_extrude_temp` (170), `max_power` (1.0), `smooth_time` (1.0), `control` (`watermark|pid`), `max_delta` (2.0, watermark), `pid_Kp`, `pid_Ki`, `pid_Kd` (divided by 255), `heater_pin`, `pwm_cycle_time` (0.100, max = sensor report time delta), `gcode_id`. The MCU pin gets `setup_cycle_time` and `setup_max_duration(3.0)`.
   - **G-code:** `M105` (also when not ready), `TURN_OFF_HEATERS`, `TEMPERATURE_WAIT SENSOR=<name> [MINIMUM] [MAXIMUM]` (error `"Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM."`), `SET_HEATER_TEMPERATURE HEATER=<name> TARGET=<t>`. Report format `T0:%.1f /%.1f B:%.1f /%.1f` ordered by `gcode_id` (`"T:0"` when none; empty until `klippy:ready`).
   - **Status (heater):** `{temperature: round(smoothed, 2), target, power}`. **Status (`heaters`):** `{available_heaters, available_sensors, available_monitors}`.
   - **Registry (plugin point):** `add_sensor_factory(sensor_type, factory)` (called by `lm75`, `htu21d`, `thermistor`, `adc_temperature`, `spi_temperature` and others), `setup_heater`, `setup_sensor`, `register_sensor`, `register_monitor`, `lookup_heater`, `get_all_heaters`, `set_temperature(heater, temp, wait)`, `turn_off_all_heaters`.
   - **Stats log line** consumed by `statistics` and log tooling: `"<name>: target=%.0f temp=%.1f pwm=%.3f"`.
   - **Events:** listens `klippy:ready`, `gcode:request_restart` (turns off heaters), `klippy:shutdown`.
   - **Errors:** `"Requested temperature (%.1f) out of range (%.1f:%.1f)"`, `"Unknown heater '%s'"`, `"Heater %s already registered"`, `"G-Code sensor id %s already registered"`, `"Unknown temperature sensor '%s'"`.
4. **Runtime traits:** **threading**: `temperature_callback` runs on the serial background thread (sensor and ADC responses) under a `threading.Lock`; `set_pwm` is called from that thread and sends MCU commands directly; a main-thread watchdog `verify_mainthread_time` (`MAX_MAINTHREAD_TIME = 5 s`) forces PWM 0 if `stats()` stops running; `QUELL_STALE_TIME = 7 s` makes `get_temp` return 0. **Float-sensitive**: PID derivative and integral with `min_deriv_time`, anti-windup, and PWM suppression (`MIN_PWM_CHANGE_RATIO = 0.05`, `next_pwm_time`) all reach the MCU as `set_pwm(pwm_time, value)`. `round(x, 2)` in status (Python round semantics). Greenlets: `_wait_for_temperature` and `cmd_TEMPERATURE_WAIT` loop with `reactor.pause(eventtime + 1.)`, call the side-effecting `toolhead.get_last_move_time()` and emit M105-style lines with `gcode.respond_raw`. The module reads `temperature_sensors.cfg` at runtime to load default thermistor sections.
5. **Tests:** every config with `[extruder]` or `[heater_bed]`; `temperature.test` (`M104 S100`, `M105`, `M140`, `M109`); `control: pid` and `control: watermark` appear in all `test/klippy/*.cfg`.
6. **Fork:** none. 7. **Difficulty: high.** Thread safety, PID numerics, the sensor-factory registry everything plugs into, and the greenlet wait loops.

### 6.2 `heater_bed.py` (28 lines)
1. 28. 2. `heaters`. 3. `[heater_bed]` (heater options above; `gcode_id` fixed to `B`). G-code `M140 S<temp>`, `M190 S<temp>` (wait). `get_status` and `stats` are bound to the `Heater` instance (`self.get_status = self.heater.get_status`). 4. Dynamic attribute binding only. 5. Tests: nearly all `.cfg` files; `temperature.test` (`M140`). 6. Fork: none. 7. **Difficulty: low.**

### 6.3 `heater_generic.py` (9 lines)
1. 9. 2. `heaters`. 3. `[heater_generic NAME]`: heater options; the returned object is the `Heater` itself (status as 6.1), used with `SET_HEATER_TEMPERATURE` and `TEMPERATURE_WAIT`. 4. None. 5. Tests: no test or example config uses it. 6. Fork: none. 7. **Difficulty: low.**

### 6.4 `homing_heaters.py` (64 lines)
1. 64. 2. `heaters`, `toolhead` (kinematics `get_steppers`). 3. `[homing_heaters]`: `heaters` (list, default all), `steppers` (list, default any). Events: `klippy:connect` (validates; errors `"One or more of these heaters are unknown: ..."`, `"One or more of these steppers are unknown: ..."`), `homing:homing_move_begin` and `homing:homing_move_end` (disable, then restore the target with `heater.set_temp`). 4. None special. 5. Tests: `printer-geeetech-301-2019.cfg` (printers.test runs `move.gcode`, which has `G28`). 6. Fork: none. 7. **Difficulty: low.**

### 6.5 `heater_fan.py` (40 lines)
1. 40. 2. `fan` (`Fan` class), `heaters`. 3. `[heater_fan NAME]`: `heater` (list, default `extruder`), `heater_temp` (50.0), `fan_speed` (1.0), all `Fan` options with `shutdown_speed` default 1.0. A 1 s timer polls `heater.get_temp()`; status = fan status. 4. Reactor timer. 5. Tests: many printers.test configs. 6. Fork: none in source (fork `fan.cfg` defines `nozzle_fan`). 7. **Difficulty: low.**

### 6.6 `fan.py` (122 lines)
1. 122. 2. `pulse_counter` (`FrequencyCounter`, another slice), `output_pin` (`GCodeRequestQueue`), `pins`, `gcode`, event `gcode:request_restart`.
3. `[fan]`: `pin`, `enable_pin`, `max_power` (1.0), `kick_start_time` (0.1), `off_below` (0), `cycle_time` (0.010), `hardware_pwm` (False), `shutdown_speed` (0; `heater_fan` passes 1.0), `tachometer_pin`, `tachometer_ppr` (2), `tachometer_poll_interval` (0.0015). G-code `M106 S<0..255>` (default 255, value divided by 255), `M107`. Status `{speed: last_req_value, rpm}` with `rpm = freq*30/ppr`. `_apply_speed`: `off_below` cutoff, `max_power` scale, optional enable pin, and a kick start that returns `("repeat", print_time + kick_start_time)` when starting from 0 or jumping by more than 0.5.
4. Request-queue and flush coupling (section 4.2); no greenlets.
5. Tests: `move.gcode` (`M106 S50`, `M106`, `M106 S90`, `M106 S0`, `M107`) runs under printers.test for all example configs; `generic_cartesian_iqex.cfg` and `generic_cartesian_itex.cfg` define `[fan]`.
6. Fork: none in source (fork `fan.cfg` sets `kick_start_time: 0.3`). 7. **Difficulty: medium.**

### 6.7 `fan_generic.py` (46 lines)
1. 46. 2. `fan`, `output_pin` (template evaluator). 3. `[fan_generic NAME]` (Fan options). `SET_FAN_SPEED FAN=<name> SPEED=<n> | TEMPLATE=<display_template> [PARAM_*]` (`"SET_FAN_SPEED must specify SPEED or TEMPLATE"`); `SPEED` has `minval=0` and no upper limit (positional argument). Status = fan status. 4. Template evaluator (renders every 0.5 s, `float(text)`, errors logged `"fan_generic template render error"`). 5. Tests: config load only (`printer-creality-cr30-2021.cfg`, `printer-creality-sermoonV1-2022.cfg`). 6. Fork: none. 7. **Difficulty: low.**

### 6.8 `multi_pin.py` (56 lines)
1. 56. 2. `pins` (`register_chip('multi_pin', self)`, `ppins.error` swallowed). 3. `[multi_pin NAME]`: `pins` (list). Virtual pin chip with `setup_pin`, `get_mcu`, `setup_max_duration`, `setup_start_value`, `setup_cycle_time`, `set_digital`, `set_pwm`, `next_aligned_print_time`. Errors `"multi_pin %s not configured"`, `"Can't setup multi_pin %s twice"`. 4. Dynamic delegation across pin objects. 5. Tests: config load (`printer-geeetech-301-2019.cfg`). 6. Fork: none. 7. **Difficulty: low.**

### 6.9 `lm75.py` (108 lines) and `htu21d.py` (256 lines): I2C temperature sensors
**`lm75.py`** (108): `bus.MCU_I2C_from_config(default_addr=0x48, default_speed=100000)`; config `lm75_report_time` (0.8, min 0.5); registers sensor type `LM75` with `heaters.add_sensor_factory`; printer object `lm75 NAME`; status `{temperature: round(t, 2)}`. Reads register `TEMP` (2 bytes), `degrees = b0 + (b1 >> 7) * 0.5` (no sign handling, so negative temperatures decode wrongly; keep for parity). Shutdown when outside `[min_temp, max_temp]`: `"LM75 temperature %0.1f outside range of %0.1f:%.01f"`. Reactor timer; the callback `_callback(estimated_print_time(now), temp)` feeds `Heater.temperature_callback` (called from the reactor thread here). Tests: none. Fork: none. **Difficulty: low.**

**`htu21d.py`** (256): sensor types `SI7013|SI7020|SI7021|SHT21|HTU21D` (`sensor_type` also selects the conversion-time table). Config `htu21d_hold_master` (False), `htu21d_resolution` (`TEMP12_HUM08`; choices `TEMP14_HUM12|TEMP13_HUM10|TEMP12_HUM08|TEMP11_HUM11`, error `"Invalid HTU21D Resolution. Valid are ..."`), `htu21d_report_time` (30, min 5), I2C default 0x40 / 100 kHz. Printer object `htu21d NAME`; status `{temperature: round(t, 2), humidity}` (macros read `printer["htu21d NAME"].humidity`). CRC8 polynomial `0x13100`; temperature `0.002681*raw - 46.85`; humidity `0.001907*raw - 6` clamped to 0..100. **Source bugs to replicate for parity:** (a) the humidity temperature-compensation statement `self.humidity = self.humidity + (25.0 - temp) * coef` is split across two lines, so the addition is a no-op and the compensation is never applied; (b) `self.i2c.i2c_write([WRITE], registerData)` passes `registerData` as the `minclock` argument, so the resolution register value is never written. Runtime: `reactor.pause()` inside the sample timer (greenlet yields); `except Exception` logs and returns `reactor.NEVER`. Tests: none (`sample-raspberry-pi.cfg` only documents it). Fork: none. **Difficulty: medium** (pause-based sampling plus the bugs).

---

## 7. Per-unit reports: LEDs and digipots

### 7.1 `led.py` (151 lines)
1. 151. 2. `output_pin` (template evaluator), `pins`, `toolhead` (`register_lookahead_callback`), reactor mutex (`reactor.mutex()`), `mcu` object (`max_nominal_duration`).
3. `[led NAME]` (PWM LED): `red_pin`, `green_pin`, `blue_pin`, `white_pin` (at least one), `cycle_time` (0.010, max = mcu max duration), `hardware_pwm`, `initial_RED|GREEN|BLUE|WHITE` (0..1). `LEDHelper` is shared with `neopixel`, `dotstar`, `pca9533`, `pca9632`: `SET_LED LED=<name> RED GREEN BLUE WHITE [INDEX] [TRANSMIT=1] [SYNC=1]`, `SET_LED_TEMPLATE LED=<name> [INDEX] TEMPLATE=<display_template>`. Status `{color_data: [(r, g, b, w), ...]}` (list of 4-tuples). Template output format `r,g,b,w` floats clamped to 0..1 (up to 4 parts, padded with 0).
4. Runtime: `SYNC=1` goes through the toolhead lookahead queue, `SYNC=0` is immediate; transmits from a `reactor.register_callback` under the mutex; `update_leds` spaces writes by `min_schedule_time`.
5. Tests: `led.test`/`led.cfg` (`lled` and `nled` are in range; `dled`, `p5led`, `p6led` belong to other slices).
6. Fork: none. 7. **Difficulty: medium.**

### 7.2 `neopixel.py` (107 lines)
1. 107. 2. `led` (LEDHelper), `pins` (`lookup_pin`), MCU.
3. `[neopixel NAME]`: `pin`, `chain_count` (1), `color_order` (list, default `GRB`, per-LED allowed), `initial_*`. MCU config command `config_neopixel oid=%d pin=%s data_size=%d bit_max_ticks=%d reset_min_ticks=%d`; commands `neopixel_update oid=%c pos=%hu data=%*s`, query `neopixel_send oid=%c` -> `neopixel_result oid=%c success=%c`. Constants `BIT_MAX_TIME` 4 us, `RESET_MIN_TIME` 50 us, `MAX_MCU_SIZE` 500. Diff batching merges changed bytes when `pos + 5 >= nextpos and nextcount < 16`. Retries `neopixel_send` up to 8 times, then logs `"Neopixel update did not succeed"`. Conversion `int(v*255. + .5)`. All updates use `reqclock=BACKGROUND_PRIORITY_CLOCK (0x7fffffff00000000)`.
4. **Byte-sensitive:** must match the MCU command stream exactly.
5. Tests: `led.test` (`nled`); config load in sidewinder-x2/x3, skr-pico, biqu-bx printers. 6. Fork: none. 7. **Difficulty: medium.**

### 7.3 `mcp4018.py` (36), `mcp4451.py` (30), `mcp4728.py` (23)
- **`mcp4018`**: `[mcp4018 NAME]`, `wiper` (required), `scale` (1.0), I2C default address 0x2f. `SET_DIGIPOT DIGIPOT=<name> WIPER=<v>` (response `"New value for DIGIPOT = %s, wiper = %.2f"`). Value `int(v*127/scale + .5)`. Writes at `klippy:connect`.
- **`mcp4451`**: `[mcp4451 NAME]`, address must be 44..47 (error `"mcp4451 address must be between 44 and 47"`), `wiper_0..3`, `scale`. Writes registers 0x04 and 0x0a (0xff), then wipers `int(v*255/scale + .5)`, at config time. No G-code.
- **`mcp4728`**: `[mcp4728 NAME]`, `channel_a..d`, `scale`, default address 0x60; `int(v*4095/scale + .5)`. No G-code.
- Deps: `bus.MCU_I2C_from_config`. Runtime: I2C writes are queued until the MCU is configured. Tests: `generic-smoothieboard` and `generic-azteeg-x5-mini-v3` (mcp4451), `generic-mightyboard` (mcp4018) in printers.test; `mcp4728` has no coverage. Fork: none. **Difficulty: low** (all three).

---

## 8. Per-unit reports: filament sensors

### 8.1 `filament_switch_sensor.py` (121 lines)
Holds `RunoutHelper` (also used by `filament_motion_sensor` and `hall_filament_width_sensor`) and `SwitchSensor`. Deps: `buttons` (`register_debounce_button`), `gcode_macro`, `pause_resume` (when `pause_on_runout`), `idle_timeout.get_status`.
- Config `[filament_switch_sensor NAME]`: `switch_pin`, `pause_on_runout` (True), `runout_gcode` (template, default empty), `insert_gcode`, `pause_delay` (0.5), `event_delay` (3.0), plus button `debounce_delay`.
- G-code `QUERY_FILAMENT_SENSOR SENSOR=<name>` (`"Filament Sensor %s: filament detected|not detected"`), `SET_FILAMENT_SENSOR SENSOR=<name> ENABLE=<0|1>`. Status `{filament_detected, enabled}`.
- Events: `klippy:ready` arms a 2 s settle time. On runout: `pause_resume.send_pause_command()`, `reactor.pause(eventtime + pause_delay)`, then runs `PAUSE\n` + rendered `runout_gcode` + `\nM400`. Log lines `"Filament Sensor %s: insert event detected, Time %.2f"` and `"... runout event detected, Time %.2f"`.
- Runtime: `reactor.register_callback` creates a greenlet per event with `reactor.pause` inside; templates; swallowed errors logged `"Script running error"`.
- Tests: `extruders.cfg` (`runout_switch`, `runout_switch1`) config load; `generic-prusa-buddy` and `printer-anycubic-*` configs. Fork: the fork printer.cfg uses `[filament_switch_sensor filament_sensor]` with `pause_on_runout: true`; no source change. **Difficulty: medium** (greenlet pause inside the event, state flags).

### 8.2 `filament_motion_sensor.py` (77 lines)
Deps: `filament_switch_sensor.RunoutHelper`, `buttons.register_buttons`, extruder object (`find_past_position(print_time)`), `mcu.estimated_print_time`. Config `switch_pin`, `extruder`, `detection_length` (7.0, above 0) plus the RunoutHelper options. A timer (`CHECK_RUNOUT_TIMEOUT = 0.25`) is armed on `idle_timeout:printing` and stopped on `idle_timeout:ready|idle`. Status delegates to `runout_helper`. Tests: `extruders.cfg` (`runout_encoder` with `extruder = extruder`, `runout_encoder1` with `extruder = extruder_stepper my_extra_stepper`), config load only. Fork: none. **Difficulty: medium.**

### 8.3 `hall_filament_width_sensor.py` (312 lines)
- Deps: `filament_switch_sensor.RunoutHelper`, `pins` (two `adc` pins, report time 0.5, sample time 0.03, count 15), `toolhead.get_position()`, `gcode.run_script("M221 S<n>")`.
- Config (mixed case): `adc1`, `adc2`, `Cal_dia1` (1.5), `Cal_dia2` (2.0), `Raw_dia1` (9500), `Raw_dia2` (10500), `measurement_interval` (10), `default_nominal_filament_diameter` (required, above 1), `measurement_delay` (required), `max_difference` (0.2), `enable` (False), `min_diameter` (1.0), `max_diameter`, `logging` (False), `enable_flow_compensation` (True), `use_current_dia_while_delay` (False). Errors `"hall_filament_width_sensor: raw_dia1 and raw_dia2 must be different"` and a `max_difference` check.
- G-code: `QUERY_FILAMENT_WIDTH`, `RESET_FILAMENT_WIDTH_SENSOR`, `DISABLE_FILAMENT_WIDTH_SENSOR`, `ENABLE_FILAMENT_WIDTH_SENSOR [FLOW_COMPENSATION=0|1]`, `QUERY_RAW_FILAMENT_WIDTH`, `ENABLE_FILAMENT_WIDTH_LOG`, `DISABLE_FILAMENT_WIDTH_LOG`. Status: `filament_detected`, `enabled`, `Diameter`, `Raw`, `is_active`, `flow_compensation_enabled` (capitalized keys kept).
- **Rounding-sensitive:** `round(read*10000)` (ADC samples to int), diameter `round(..., 2)`, smoothing `(5*d + new)/6`, flow factor `round(nominal**2 / width**2 * 100)` sent as `M221 S<percent>`. Python `round()` is round-half-even and `round(x, 2)` is decimal-correct; Rust `f64::round()` rounds half away from zero, so a naive port differs.
- Runtime: ADC callbacks arrive from the serial thread (they write plain attributes); a 1 s reactor timer (`extrude_factor_update_event`); deferred disable/clear flags; runs G-code from the timer.
- Tests: none (only commented out in `config/*.cfg`). Fork: none. **Difficulty: medium.**

---

## 9. Per-unit reports: accelerometers and bulk sensors

All four accelerometers register mux G-code commands through `adxl345.AccelCommandHelper` (another slice): `ACCELEROMETER_MEASURE`, `ACCELEROMETER_QUERY`, `ACCELEROMETER_DEBUG_READ`, `ACCELEROMETER_DEBUG_WRITE`, each with `CHIP=<name>`. Shared helpers from `adxl345`: `read_axes_map(config, sx, sy, sz)` (config `axes_map`, for example `x,z,-y`) and `AccelQueryHelper` (returned by `start_internal_client()` for `resonance_tester`), plus `bulk_sensor.FixedFreqReader` and `BatchBulkHelper` (registers a webhooks mux endpoint `<chip>/dump_<chip>` with header `['time','x_acceleration','y_acceleration','z_acceleration']`; `BulkDataQueue` is filled by the serial background thread under a lock). All use `bus.MCU_I2C_from_config` (`lis2dw` uses `MCU_SPI_from_config` when `cs_pin` is set). Output sample format: `(round(ptime, 6), round(raw*scale, 6), ...)`. No numpy.

| Unit | Lines | Chip specifics | MCU commands | Endpoint |
|---|---|---|---|---|
| `mpu9250.py` | 175 | addr 0x68, 400 kHz, `rate` 4000 only, scale `1/4096 g * 9.80665e3` (8 g), accepted IDs 0x74, 0x73, 0x71, 0x70, 0x68 (+ "defective" 0x75, 0x69), wake/sleep/FIFO register sequence with a 20 ms delayed `REG_SMPLRT_DIV` write | `config_mpu9250 oid=%d i2c_oid=%d`, `query_mpu9250 oid=%d rest_ticks=0` (on_restart), `query_mpu9250 oid=%c rest_ticks=%u`, `query_mpu9250_status oid=%c` | `mpu9250/dump_mpu9250` |
| `icm20948.py` | 173 | addr 0x68, 400 kHz, `rate` 4500 only, scale `1/2048 g` (16 g), ID 0xEA, bank select via `REG_BANK_SEL`, FIFO reset sequence | `config_icm20948 oid=%d i2c_oid=%d`, `query_icm20948 ...`, `query_icm20948_status oid=%c` | `icm20948/dump_icm20948` |
| `lis2dw.py` | 209 | I2C (addr 0x19, 400 kHz) or SPI (mode 3, 5 MHz, selected by `cs_pin`); `LIS2DW` rate 1600, scale `g*1.952/4`; `LIS3DH` rate 1344, scale `g*11.718/16`; `set_reg` writes, **reads back and raises** on mismatch (`"Failed to set LIS2DW register [0x%x] ..."`); IDs 0x44 (LIS2DW), 0x33 (LIS3DH) | `config_lis2dw oid=%d bus_oid=%d bus_oid_type=%s lis_chip_type=%s`, `query_lis2dw ...`, `query_lis2dw_status oid=%c` | `lis2dw/dump_lis2dw` |
| `lis3dh.py` | 12 | thin wrapper: `[lis3dh]` and `[lis3dh NAME]` both build `lis2dw.LIS2DW(config, LIS3DH_TYPE)` | same as `lis2dw` | `lis2dw/dump_lis2dw` |

- Runtime traits (all): `reactor.monotonic()` and `estimated_print_time` for the 20 ms wake delay (`mpu9250`); start and stop through the bulk-client refcount (`BatchBulkHelper`); error text `"Invalid <chip> id (got %x...).\nThis is generally indicative of connection problems..."`; both `load_config` and `load_config_prefix` are registered. Python `round()` caveat: section 13, risk 2.
- Tests: `input_shaper.cfg` loads `[mpu9250 my_mpu]` (`input_shaper.test`); `icm20948`, `lis2dw` and `lis3dh` have no coverage (`sample-duet3-1lc.cfg` shows `[lis3dh]`).
- Fork: none in source (the fork config uses `adxl345` over spidev).
- Difficulty: **medium** each (small chip logic, but relies on the `bulk_sensor` clock regression and the `adxl345` helper API); `lis3dh` is **low**.

### 9.1 `hx71x.py` (178 lines)
- Library only (no `load_config`): `HX711(config)`, `HX717(config)`, `HX71X_SENSOR_TYPES = {'hx711', 'hx717'}`; used by `load_cell` and `load_cell_probe`.
- Config: `dout_pin`, `sclk_pin` (same MCU), `sample_rate` (hx711: 80|10, default 80; hx717: 320|80|20|10, default 320), `gain` (hx711: `A-128`=1, `B-32`=2, `A-64`=3; hx717: `A-128`=1, `B-64`=2, `A-64`=3, `B-8`=4). MCU: `config_hx71x oid=%d gain_channel=%d dout_pin=%s sclk_pin=%s`, `query_hx71x oid=%d rest_ticks=0` (on_restart), `query_hx71x oid=%c rest_ticks=%u`, `query_hx71x_status oid=%c`, `hx71x_attach_trigger_analog oid=%d trigger_analog_oid=%d` (`is_init=True`, for the load-cell probe). Bulk format `<i`, range `(-0x800000, 0x7FFFFF)`.
- Status `{errors, overflows, sample_rate}`. Sample tuple `(round(ptime, 6), val, round(val/2^23, 9))`; error sentinels `-0x80000000` and `0x40000000`; forced sensor restart on an error batch, or after more than 4 consecutive overflow batches.
- Tests: `load_cell.test`/`load_cell.cfg` (`my_hx711`, `my_hx717`). Fork: none. **Difficulty: medium.**

### 9.2 `ldc1612.py` (238 lines)
- Eddy-current sensor driver used by `probe_eddy_current` (other slice) and `trigger_analog`. Deps: `bus` (I2C), `bulk_sensor.FixedFreqReader/BatchBulkHelper`, `mcu` enumerations (`ldc1612_error:`), `configfile`, `toolhead`; stdlib `math`.
- Config (owner section): `frequency` (12 MHz; 2-40 MHz), `max_sensor_hz` (5 MHz; 3-20 MHz), `reg_drive_current` (15; 0-31), `intb_pin` (same MCU), I2C default address 0x2a / 400 kHz. A runtime warning is issued through `configfile.runtime_warning` when calibration frequencies exceed `max_sensor_hz`.
- G-code `LDC_CALIBRATE_DRIVE_CURRENT CHIP=<name>`: reads `REG_DRIVE_CURRENT0`, responds `"%s: reg_drive_current: %d\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer."`, and writes `configfile.set(name, 'reg_drive_current', ...)`.
- Webhook mux endpoint `ldc1612/dump_ldc1612` (header `('time','frequency','z')`). MCU: `config_ldc1612 oid=%d i2c_oid=%d` or `config_ldc1612_with_intb oid=%d i2c_oid=%d intb_pin=%s`, `query_ldc1612 oid=%d rest_ticks=0` (on_restart), `query_ldc1612 oid=%c rest_ticks=%u`, `query_status_ldc1612 oid=%c`, `ldc1612_attach_trigger_analog oid=%d trigger_analog_oid=%d` (`is_init=True`).
- Numerics reaching hardware: `sensor_div = ceil(4*max_hz/clock)`, `freq_conv = clock*div/2^28`, `rcount0 = clock/(16*data_rate)` rounded, settle count `int(0.005*clock/16 + .5)`. Sample decode with error bits; output `(round(ptime, 6), round(freq*mv, 3), 999.9)` where 999.9 is a placeholder replaced by the calibration. Error logging `"%s: %s (%d)"`.
- Under `-o` file output `read_reg` returns 0 and the ID check is skipped.
- Tests: `eddy.test`/`eddy.cfg` (`[probe_eddy_current eddy] sensor_type: ldc1612`). Fork: none. **Difficulty: medium.**

---

## 10. Per-unit reports: load cell and manual probing

### 10.1 `load_cell.py` (542 lines)
- Deps: `hx71x`, `ads1220`, `ads131m0x` (another slice), `bulk_sensor.BatchWebhooksClient`, `webhooks`, `configfile`. A Python 2 shim (`izip`) is still present.
- Config `[load_cell NAME]`: `sensor_type` (`hx711|hx717|ads1220|ads131m02|...`), `reference_tare_counts` (int), `counts_per_gram` (at least 1.0), `sensor_orientation` (`normal|inverted`), plus sensor options.
- G-code (mux `LOAD_CELL=`): `LOAD_CELL_TARE`, `LOAD_CELL_CALIBRATE` (interactive; dynamically registers `ABORT`, `ACCEPT`, `TARE`, `CALIBRATE GRAMS=<50..25000>`; duplicate detection by trying to register `TARE` with `'dummy'`), `LOAD_CELL_READ`, `LOAD_CELL_DIAGNOSTIC` (10 s `reactor.pause`). Many exact response strings (for example `"Load cell tare value: %.2f%% (%i)"`, `"%.1fg (%.2f%%)"`). Calibration finalize writes `configfile.set(name, 'counts_per_gram', '%.5f')` and `'reference_tare_counts', '%i'`.
- Webhook mux endpoint `load_cell/dump_force` (header `["time","force (g)","counts","tare_counts"]`). Events sent: `load_cell:tare`, `load_cell:calibrate`; listens `klippy:ready`.
- Status: `force_g`, `min_force_g`, `max_force_g` (rounded to 1 decimal, only when tared and calibrated), `is_calibrated`, `counts_per_gram`, `reference_tare_counts`, `tare_counts`, plus sensor status (`errors`, `overflows`, `sample_rate`).
- Runtime traits: reactor pause loops in `LoadCellSampleCollector` (`RETRY_DELAY = 0.05`); client-list pattern (a callback returns False to unregister); under file output the collector returns `[(0., 0., 0.)]`.
- Tests: `load_cell.test`/`load_cell.cfg` (four sensor types plus a probe config; the test runs `G28`, `BED_MESH_CALIBRATE`, `PROBE`). Fork: none found (printer.cfg keeps the include commented out). **Difficulty: medium.**

### 10.2 `load_cell_probe.py` (537 lines)
- Deps: `hx71x`, `ads1220`, `ads131m0x`, `probe` (`ProbeParameterHelper`, `ProbeCommandHelper`, `ProbeOffsetsHelper`, `SampleAveragingHelper`, `HomingViaProbeHelper`, `LookupZSteppers`, `lookup_minimum_z`), `manual_probe` (`create_probe_result`), `trigger_analog` (`MCU_trigger_analog`, `MCU_SosFilter`, `DigitalFilter`), `load_cell`, top-level `mcu`, `homing` (`probing_move`), numpy (lazy), scipy (lazy through `DigitalFilter`).
- It registers itself as the printer object `probe`. Config: `sensor_type` (same list as `load_cell`), `tare_time` (4/60 s, 0.01-1.0), `trigger_force` (75 g, 10-250), `force_safety_limit` (2000 g, 100-5000), `drift_filter_cutoff_frequency` (0.1-20), `drift_filter_delay` (1-2, default 2), `buzz_filter_cutoff_frequency`, `buzz_filter_delay` (1-2), `notch_filter_frequencies` (list, up to 2), `notch_filter_quality` (0.5-6, default 2.0). Each can be read from config and overridden per G-code command with the upper-case name (`ParamHelper`), plus the standard probe options.
- G-code: `LOAD_CELL_TEST_TAP [TAPS=1..10] [TIMEOUT=1..120]`, plus the standard probe commands registered through `probe.ProbeCommandHelper` (`PROBE`, `QUERY_PROBE`, `PROBE_ACCURACY`, `PROBE_CALIBRATE`, `Z_OFFSET_APPLY_PROBE`) `[INFERENCE: defined in probe.py, another slice]`. Webhook mux endpoint `load_cell_probe/dump_taps` (`{"tap": {time, force, is_valid}}`).
- Status: probe status + load-cell status + `{last_z_result, is_last_tap_valid}`.
- MCU: `trigger_analog` object with an SOS filter (4 sections, `src/sos_filter.c`), `FRAC_GRAMS_CONV = 32768`, `set_trigger("abs_ge", int(force*32768))`, `set_raw_range`, `set_offset_scale(int(-tare), grams_per_count*32768)`; `OverflowError` if `counts_per_gram >= 2^29`.
- Numpy: section 2. Parameter conversion into the MCU filter and trigger is float-sensitive.
- Tests: `load_cell.test` (`G28`, `BED_MESH_CALIBRATE`, `PROBE` against the file-output MCU). Fork: none. **Difficulty: high** (probe stack coupling, SciPy-designed filters, MCU filter protocol).

### 10.3 `manual_probe.py` (297 lines)
- Auto-loaded by `toolhead.py`. Deps: `gcode_move` (`get_status()['homing_origin'].z`), `toolhead` (`manual_move`, `flush_step_generation`, `get_kinematics`), `configfile.set`. Public helpers used by other slices: `ProbeResult` namedtuple (`bed_x, bed_y, bed_z, test_x, test_y, test_z`), `create_probe_result(test_pos, offsets)`, `lookup_z_endstop_config(config)`, `verify_no_manual_probe(printer)`, `ManualProbeHelper(printer, gcmd, finalize_callback)` (used by `probe`, `axis_twist_compensation`, `safe_z_home` (lookup only), and the leveling modules).
- G-code: `MANUAL_PROBE [SPEED=5]`, `Z_ENDSTOP_CALIBRATE`, `Z_OFFSET_APPLY_ENDSTOP` (cartesian or delta variant), and while active `ACCEPT`/`NEXT`/`ABORT`/`TESTZ Z=<+|++|-|--|float>` (registered and unregistered dynamically). Status `{is_active, z_position, z_position_lower, z_position_upper}`. Output strings: `"Z position: %s --> %.3f <-- %s"`, `"Z position is %.3f"`, `"%s: position_endstop: %.3f\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer."`; `configfile.set(section, 'position_endstop', '%.3f')`.
- Reads `stepper_z` or `carriage z` `position_endstop` and delta `stepper_a|b|c` `position_endstop` with `note_valid=False`.
- Numerics: bisect over previously tested heights, `BISECT_MAX = 0.2`, `Z_BOB_MINIMUM = 0.5`.
- Tests: `multi_z.test` (`MANUAL_PROBE`, `TESTZ`, `ACCEPT`, `ABORT`, `Z_ENDSTOP_CALIBRATE`, `PROBE_CALIBRATE`), plus `z_tilt`, `delta_calibrate`, `bed_screws`, `screws_tilt_adjust`, `z_virtual_endstop` tests (through `ManualProbeHelper`). Fork: none (the copyright line reads 2019-2026, so this tree tracks a recent upstream). **Difficulty: medium.**

---

## 11. Per-unit reports: display subpackage (in range)

### 11.1 `display/hd44780.py` (134 lines)
- LCD chip driver chosen by `lcd_type: hd44780` in `display.py`. Config: `rs_pin`, `e_pin`, `d4_pin`..`d7_pin` (all on one MCU), `hd44780_protocol_init` (True), `line_length` (16|20, default 20). API: `init`, `flush`, `clear`, `write_text`, `write_glyph`, `write_graphics` (no-op), `set_glyphs`, `get_dimensions`. MCU: `config_hd44780 oid=%d rs_pin=%s e_pin=%s d4_pin=%s d5_pin=%s d6_pin=%s d7_pin=%s delay_ticks=%d` (40 us delay), `hd44780_send_cmds oid=%c cmds=%*s`, `hd44780_send_data oid=%c data=%*s`; all sends use `reqclock=BACKGROUND_PRIORITY_CLOCK`. Framebuffer diff batching merges changes when `pos + 4 >= nextpos and nextcount < 16`; init commands are spaced 100 ms apart.
- Tests: printers.test (`generic-gt2560`, `printer-anet-a4-2018`, `printer-anet-a8-2017`, `printer-anycubic-kossel-*`, `printer-bq-hephestos-2014` use `lcd_type: hd44780`). Fork: none. **Difficulty: medium** (byte-exact command stream; Python `bytearray` slicing semantics).

### 11.2 `display/hd44780_spi.py` (125 lines)
- Same logical driver over an SPI shift register (`bus.MCU_SPI_from_config(config, 0x00, pin_option="latch_pin")`): nibble protocol with `data_mask = 1<<1`, `enable_mask = 1<<3`, three SPI sends per nibble. Same config options except pins. Tests: `generic-mightyboard.cfg` (`lcd_type: hd44780_spi`). Fork: none. **Difficulty: medium.** (It duplicates the diff-batching code of `hd44780.py`; `aip31068_spi.py` in another slice copies it again.)

### 11.3 `display/font8x14.py` (276 lines)
- Data only: `VGA_FONT`, 256 byte strings of 16 bytes (8x14 bitmap with padding). Used by `st7920.py` (`CharGlyphs['degrees'] = VGA_FONT[0xf8]`) and `uc1701.py` (bit-swizzled). **Port: embed as a static array generated from the Python file.** Tests: indirect (`st7920`/`uc1701` configs in printers.test). Fork: none. **Difficulty: low.**

### 11.4 `display/menu.py` (1,067 lines), `menu_keys.py` (108), `menu.cfg` (782)
- `MenuManager(config, display)` is created by `display.py` for the primary `[display]` only. Printer object `menu`; status `{timeout, running, rows, cols}`; events sent `menu:init|begin|exit|populate` and item events `<ns>:<event>`; a 0.2 s timer implements `menu_timeout`.
- Config on `[display]`: `menu_root` ('__main'), `menu_timeout` (0), `menu_reverse_navigation` (False). `menu_keys` options: `encoder_pins` ("a,b"), `encoder_steps_per_detent` (2|4, default 4), `encoder_fast_rate` (0.030), `click_pin`, `back_pin`, `up_pin`, `down_pin`, `kill_pin`, `analog_range_<name>`, `analog_pullup_resistor` (4700). Long click at 0.8 s. `kill_pin` calls `printer.invoke_shutdown("Shutdown due to kill button!")`.
- Sections `[menu <path>]`: `type` is `disabled|command|input|list|vsdlist`; options `name`, `enable` (bool or template), `index`, `gcode`, `input`, `input_min`, `input_max`, `input_step`, `realtime`, and event scripts. `menu.cfg` (782 lines; Tune, OctoPrint, SD Card, Control, Temperature, Filament, Setup) is read from the package directory at load with `configfile.read_config`, and user sections override by name.
- Templates: Jinja2 via `gcode_macro.load_template`; results are parsed with `ast.literal_eval` for `enable` and `input*` (section 3); `asflat()` strips quotes and newlines; the VSD list calls `virtual_sdcard.get_file_list()` and emits `M23 /<file>`; queued G-code runs through `reactor.register_callback(dispatch_gcode)` and `gcode.run_script`.
- Python 2 leftover: `aslatin()` references `unicode` (dead path on Python 3). Name-mangled private scroll state (`__scroll_*`).
- Tests: `[display]` configs in printers.test (`st7920`, `hd44780`) instantiate the manager and parse `menu.cfg`; no menu interaction is simulated. Fork: none. **Difficulty: high** (large state machine, plus Jinja and `literal_eval` on every redraw; only feasible after the template engine from 5.1 exists).

---

## 12. Out-of-range neighbours (one-line mentions)

- `force_move.py` (140 lines): `STEPPER_BUZZ`, `FORCE_MOVE`, `SET_KINEMATIC_POSITION`; uses `motion_queuing.allocate_trapq/wipe_trapq/note_mcu_movequeue_activity` and `cartesian_stepper_alloc`.
- `gcode_move.py`: owns `G0/G1`, `gcode_position`, `homing_origin`; called directly by `gcode_arcs` (`cmd_G1`) and `manual_probe` (`get_status`).
- `homing.py`: `probing_move` used by `load_cell_probe`; emits `homing:homing_move_begin/end` used by `homing_heaters`.
- `input_shaper.py`, `manual_stepper.py`, `motion_report.py`: allocate trapqs and step queues from `motion_queuing`; not analyzed here.

---

## 13. Cross-cutting risks seen in this slice

1. **Jinja2 plus Python object semantics** (`gcode_macro`, the `output_pin` evaluator, `menu`, the `idle_timeout` default, every `*_gcode` option). One agreed engine is needed, together with a Python literal parser, Python `%` string formatting, namedtuple attribute access, filters, and `printer` status wrapper semantics (`in`, iteration, deepcopy). Build a regression corpus from `config/*.cfg`, `menu.cfg`, `display.cfg`, `sample-macros.cfg` and the test macros.
2. **Python numeric semantics:** `round(x)` and `round(x, n)` (half-even, decimal-exact) appear in `hall_filament_width_sensor`, the accelerometers, `ldc1612`, `hx71x`, `heaters` status, `lm75`/`htu21d` status and `load_cell` status. `int(x + .5)` truncation in digipots and `neopixel`; `%d` truncation in `firmware_retraction`; `%.5f`, `%.3f`, `%.1f` formatting; libm `sin`/`cos`/`atan2` in `gcode_arcs`.
3. **Threading:** `heaters` callbacks and `set_pwm` run on the serial background thread with a lock and a main-thread watchdog; `hall` ADC callbacks and `bulk_sensor` data queues also cross threads. The Rust architecture for the serial thread must keep the same locking and the 5 s watchdog behaviour.
4. **Greenlets and reactor pause:** pausing code in `htu21d` (inside timers), `heaters` wait loops, `filament_switch_sensor` runout (inside registered callbacks), `load_cell` collectors (10 s diagnostic), `motion_queuing` (`_await_flush_time`, `drip_update_time`), plus nested `gcode.run_script` from timers and callbacks (`idle_timeout`, `gcode_button`, `menu`, `hall`). Choose an equivalent coroutine model before porting these.
5. **Dynamic command (un)registration and chaining:** `homing_override` (G28 chain), `gcode_macro` `rename_existing`, `manual_probe` and `load_cell` (`ACCEPT/ABORT/TESTZ/TARE/CALIBRATE`, with duplicate detection that catches the error from registering `'dummy'`). The gcode module must expose `register_command(name, None)` returning the previous handler.
6. **Config write-back (`SAVE_CONFIG`):** `configfile.set` in `manual_probe`, `load_cell`, `ldc1612`.
7. **Status shapes consumed by macros and Moonraker:** `led.color_data` (list of tuples), heater `power`/`target`, `load_cell` keys, `hall` capitalized keys; int versus float differences are visible in JSON.
8. **Source bugs worth reproducing deliberately:** `htu21d` humidity compensation no-op and `i2c_write` argument misuse, `lm75` unsigned decode, `hall` mixed-case option names.
9. **Weak tests:** exit code only (`scripts/test_klippy.py`); every "covered by a test" claim means "loads and runs". Parity needs golden `-o` stream captures per `.test` file, plus new tests for uncovered modules (`htu21d`, `lm75`, `hall`, `icm20948`, `lis2dw`/`lis3dh`, `mcp4728`, `firmware_retraction` G10/G11, `fan_generic`, `heater_generic`, filament sensor events, menu interaction).
10. **Fork delta is concentrated in `motion_queuing.py`** (plus out-of-range `reactor.py`, `mcu.py`, `toolhead.py`, `klippy.py`, `virtual_sdcard.py`, `chelper`): the tuned flush timings and the `[DEBUG-motion]` log records must be ported, together with `reactor.get_timing_debug_report`.
11. **Import graph oddity:** `output_pin` imports `display.display`, which imports all LCD drivers and `menu`; unit boundaries differ from file boundaries.
12. **Always-on modules:** `toolhead` auto-loads `gcode_move`, `homing`, `idle_timeout`, `statistics`, `manual_probe`, `tuning_tower`, `garbage_collection`; `idle_timeout` loads `gcode_macro`; `motion_queuing` is loaded by `toolhead` and `stepper`.

## 14. Suggested port order (leaves first)

Prerequisites outside this slice: `config`, `gcode`, `pins`, `mcu`/`serialhdl`, `reactor`, `chelper`, `bus`, `buttons`, `bulk_sensor`, the `adxl345` helper.

1. **No-dependency leaves:** `display/font8x14` (static data), `garbage_collection` (no-op), `lis3dh` (wrapper).
2. **Config and I2C/pin glue:** `mcp4018`, `mcp4451`, `mcp4728`, `multi_pin`, `lm75`, `htu21d` (after the `heaters` sensor-factory registry).
3. **Decision gate:** the template engine for `gcode_macro` (with the Python literal parser). Everything template-based waits on it.
4. **Queue core:** `motion_queuing` (with the fork constants and debug rings; blocked by the `chelper` FFI/port decision), then `output_pin` (`GCodeRequestQueue` and the template evaluator).
5. **Heater stack:** `heaters` (threading model), `heater_bed`, `heater_generic`, `homing_heaters`; then `fan`, `heater_fan`, `fan_generic`.
6. **LEDs:** `led`, `neopixel` (after `output_pin`).
7. **Session control:** `idle_timeout`, `homing_override`, `gcode_button`, `firmware_retraction`, `gcode_arcs`, `filament_switch_sensor`, then `filament_motion_sensor` and `hall_filament_width_sensor`.
8. **Bulk sensors:** `hx71x`, `ldc1612`, `mpu9250`, `icm20948`, `lis2dw` (need the `bulk_sensor` clock regression).
9. **Probing:** `manual_probe` (needed earlier by `probe`, `z_tilt` and others in other slices), `load_cell`, `load_cell_probe` (last; needs `probe`, `trigger_analog`, SOS filter design, numpy/scipy replacements).
10. **Display and UI:** `hd44780`, `hd44780_spi`, `menu_keys`, then `menu` and `menu.cfg` (after the template engine and `display.py`).

Difficulty tally (per unit; `menu.py`, `menu_keys.py` and `menu.cfg` are rated together as one unit): low 16 (`garbage_collection`, `gcode_button`, `homing_override`, `firmware_retraction`, `heater_bed`, `heater_generic`, `homing_heaters`, `heater_fan`, `fan_generic`, `multi_pin`, `lm75`, `mcp4018`, `mcp4451`, `mcp4728`, `lis3dh`, `font8x14`); medium 18 (`fan`, `idle_timeout`, `gcode_arcs`, `led`, `neopixel`, `htu21d`, `filament_switch_sensor`, `filament_motion_sensor`, `hall_filament_width_sensor`, `mpu9250`, `icm20948`, `lis2dw`, `hx71x`, `ldc1612`, `load_cell`, `manual_probe`, `hd44780`, `hd44780_spi`); high 6 (`motion_queuing`, `output_pin`, `gcode_macro`, `heaters`, `load_cell_probe`, `menu` group).
