# Port map: klippy/extras, files `p` through `z` (slice C)

## 0. Scope, method and limits

**Range.** The first file is `klippy/extras/palette2.py`. The last is `klippy/extras/z_tilt.py`. The range covers 56 top-level files (55 `.py` plus `temperature_sensors.cfg`) and two files in the `display/` subdirectory whose names start with `s` and `u`: `display/st7920.py` and `display/uc1701.py`. That is 58 files and about 13,400 lines. The other `display/` files (`display.py`, `font8x14.py`, `hd44780*.py`, `menu*.py`, `aip31068_spi.py`) and `__init__.py` start with letters outside `p`-`z`, so they belong to another slice. Whoever owns `display/display.py` should know that `st7920.py` and `uc1701.py` import `font8x14` and `bus`, and that `display.py` imports them by `lcd_type`.

**No `prtouch` module exists in this repo.** A search for `prtouch`/`PRTOUCH` found only unrelated C headers. PRTouch on the KE is meant to run as HX711 (`hx71x.py`) into `load_cell.py` into `load_cell_probe.py`, with `trigger_analog.py` (this slice) as the MCU-side trigger. The README (`config/ender-3-v3-ke/README.md`) says this path needs an MCU reflash with `CONFIG_WANT_HX71X=y`, and `printer.cfg` has `#[include load_cell.cfg]` commented out. The Creality-specific code in this slice is limited to `pause_resume.py` (display endpoints) and `virtual_sdcard.py` (fork debug instrumentation).

**How fork changes were found.** I had no shell and no `git log`. I compared each local file's line count with upstream Klipper at two points, `616242d4` (`origin/master` at clone) and `6d43f8b3` (the master that was merged), and with upstream `master` through `raw.githubusercontent.com`. I read the differing files in detail. A file with an equal line count is presumed unchanged and was not diffed line by line. Commit hashes come from `.git/logs/HEAD` (the reflog), which gives commit messages but not per-file attribution. Where I attribute a file to a commit, that is `[INFERENCE]`.

**Reflog hashes of fork commits** (Conventional Commit style):
- `3a63ec4a` feat(creality): support stock c440x display endpoints
- `4f9c676c` fix(ender3-ke): restore calibrated printer configuration
- `d5c37cf9` feat(ender3-ke): add supervised mainline handover
- `152aa00c` fix(ender3-ke): gate moonraker on mainline readiness
- Final series after the reset to `152aa00c`:
  - `7777d104` feat(debug): preserve shutdown provenance
  - `aac1802d` feat(debug): trace host motion scheduling
  - `4eec735c` fix(motion): tolerate host scheduling stalls
  - `3bc87895` feat(debug): capture step compression state
  - `cf3c4ddb` feat(debug): trace serial dispatch decisions
  - `7d0860db` chore(ender3-ke): retain mainline diagnostic logs
  - `e47b0943` fix(motion): keep producer ahead of step generation
- Older fork commits, config or docs only: `5900c356`, `ccc10c3`, `9fa2b8d7`, `78b05289` feat(InputShaper), `9b3499b2`, `b677987d`, `8b3f3fcb`.

**Fork changes inside this slice:**

| File | Local vs upstream | Nature |
|---|---|---|
| `pause_resume.py` | 111 lines vs 100 (+11) | Two Creality webhook endpoints. Likely `3a63ec4a`. |
| `virtual_sdcard.py` | 487 lines vs 308 (+179) | `[DEBUG-motion] v4` timing instrumentation. Behaviour of `work_handler` is unchanged, I compared the upstream body. Likely `aac1802d`, `4eec735c` and/or `e47b0943` `[INFERENCE]`. |
| `spi_temperature.py` | 357 vs 351 (+6) | Blank-line differences only (read both heads). |
| `sx1509.py` | 191 vs 190 (+1) | One blank line (top-level layout compared). |
| `shaper_calibrate.py` | 493 vs 492 (+1) | One blank line (top-level layout compared). |
| `palette2.py` | 654 vs 652 (+2) | Not inspected, probably whitespace `[INFERENCE]`. |
| `probe.py` | 631 | Equals current upstream master (628 at `6d43f8b3`), so upstream drift, not fork. |
| `probe_eddy_current.py` | 1075 | Between upstream `6d43f8b3` (1070) and master (1087), so upstream drift `[INFERENCE]`. |

Every other file's line count equals upstream (older and newer base).

**How tests work today.** `scripts/test_klippy.py` runs `klippy.py <cfg> -i <gcode> -o <out> -d <dict> -v` and only checks the exit code and the absence of errors. Output is NOT compared against a golden file, except that `SHOULD_FAIL` marks tests that must fail. So parity tests for the port need a new harness that records the `-o` debugoutput (the MCU command stream) from the Python run as golden data. In debugoutput mode the TMC UART/SPI register reads return 0 and writes are skipped, which makes the golden data deterministic.

---

## 1. Module families

| Family | Modules |
|---|---|
| Temperature sensors and heater helpers | `thermistor`, `temperature_sensor`, `temperature_combined`, `temperature_host`, `temperature_mcu`, `temperature_fan`, `temperature_probe`, `spi_temperature`, `sht3x`, `verify_heater`, `pid_calibrate`, `z_thermal_adjust`, `temperature_sensors.cfg` |
| Probes, leveling, compensation | `probe`, `probe_eddy_current`, `trigger_analog`, `smart_effector`, `z_tilt`, `quad_gantry_level`, `screws_tilt_adjust`, `safe_z_home`, `skew_correction`, `tuning_tower` |
| TMC drivers | `tmc`, `tmc_uart`, `tmc2130`, `tmc2208`, `tmc2209`, `tmc2240`, `tmc2660`, `tmc5160` |
| Resonance and analysis | `resonance_tester`, `shaper_calibrate`, `shaper_defs` |
| Print job and host state | `virtual_sdcard`, `sdcard_loop`, `print_stats`, `pause_resume`, `respond`, `save_variables`, `statistics`, `query_adc`, `query_endstops`, `stepper_enable` |
| Pins and MCU peripherals | `pwm_tool`, `pwm_cycle_time`, `static_pwm_clock`, `static_digital_output`, `servo`, `pulse_counter`, `samd_sercom`, `sx1509`, `replicape` |
| LEDs | `pca9533`, `pca9632` |
| Displays (drivers) | `display/st7920`, `display/uc1701` (the SSD1306 and SH1106 drivers live in `uc1701.py`) |
| Misc hardware | `palette2` (MMU over serial), `tsl1401cl_filament_width_sensor` |
| Macros and templates | None in this slice. Templates reach this slice only through `gcode_macro.load_template` (see section 3). |

**Cross-slice names used below.** `heaters`, `pins`, `bus`, `gcode_macro`, `gcode_move`, `homing`, `manual_probe`, `bed_mesh`, `input_shaper`, `fan`, `led`, `ldc1612`, `bulk_sensor`, `adc_temperature`, `motion_queuing`, `force_move`, `toolhead`, `stepper`, `mcu`, `mathutil`, `chelper` and `queuelogger` live in other slices or in the core.

---

## 2. Cross-cutting answers to the extra questions

**Numeric libraries**
- **numpy:** only `shaper_calibrate.py`. It computes:
  - Welch PSD (Kaiser window, `rfft`, `as_strided` windows).
  - `exp`/`sin`/`cos` shaper response matrices.
  - `interp`, `arange` and `maximum` over frequency grids.
  - It also writes the CSV files. `resonance_tester.py` calls `.mean()` on the numpy arrays it receives.
  - numpy is imported lazily with `importlib.import_module('numpy')` and raises a friendly command error if missing. The work runs in a `multiprocessing.Process` so the main process stays responsive.
  - `scripts/calibrate_shaper.py`, `scripts/graph_accelerometer.py` and `scripts/graph_shaper.py` import `extras.shaper_calibrate` and `extras.shaper_defs` directly, so a Rust port must keep an equivalent offline path.
- **scipy:** only `trigger_analog.DigitalFilter` (butter, iirnotch, tf2sos, sosfilt_zi), imported lazily. A pre-generated table `GeneratedSOS` covers the single common case, a 4th-order lowpass at 25 Hz for 400 sps, which is the filter the eddy "tap" mode uses. Any other filter needs SciPy, so a Rust port must either reimplement SciPy's Butterworth design or ship more tables. Note that `DigitalFilter.setup_initial_state` references an undefined name `signal` (an upstream bug), so that path is effectively dead.
- **Pure-Python math (`mathutil`):** least squares, linear solve and pseudo-inverse in `shaper_defs.get_mzv_coeffs`, `temperature_probe.Polynomial2d.fit`, `z_tilt.probe_finalize`, `probe_eddy_current.TapBestFit` and `skew_correction`. No numpy involved.
- **Other third-party:**
  - `pyserial` in `palette2`.
  - `cffi` indirectly in `pwm_tool` via `chelper.get_ffi().syncemitter_queue_msg`.
  - `greenlet` indirectly through every `reactor.pause()`.

**Template or expression evaluation**
- **Jinja2 via `gcode_macro.load_template`:**
  - `virtual_sdcard` (`on_error_gcode`, default `TURN_OFF_HEATERS` guarded by `{% if 'heaters' in printer %}`).
  - `probe.ProbeEndstopWrapper` (`activate_gcode` and `deactivate_gcode`). `smart_effector` reuses that class.
  - There is no direct `import jinja2` in this slice, but the `printer` context object and every `printer.<obj>.<field>` that templates read is an external contract of the `get_status()` fields listed below.
- **Python literal evaluation:** `save_variables.py` uses `ast.literal_eval` on both load and `SAVE_VARIABLE` input and writes values with `repr()`. A port needs a Python-literal parser and a repr-compatible writer (dicts, lists, tuples, strings with quotes, floats, ints, bools, None).
- **Format strings assembled at runtime:** `tuning_tower.py` builds `"%s %s=%%.9f"` or `"%s %s%%.9f"` and feeds the result back as G-code (`%.9f` formatting must match).
- **Command-name introspection:** `palette2.py` registers `O0`..`O32` using `getattr(self, 'cmd_O<i>')`.
- **Regex mini-language:** `shaper_defs.init_shaper` parses shaper names such as `mzv(5,0.6)` or `ei(v_tol=0.05)` with `re` (not eval).

**Creality / Ender-3 V3 KE items**
- **Component 1: pause_resume display endpoints.** `pause_resume.py` adds two webhook endpoints used by Creality's closed-source touchscreen UI. That UI is the `c440x` program, version `V1.1.0.17` in the captured logs, which runs on the "Nebula Pad" (Ingenic xburst2 MIPS, glibc 2.29) and connects to the Klippy Unix socket (`-a /tmp/klippy_uds`). Related fork code outside this slice: `webhooks.py` registers `shakehands` (returns `state`, `state_message`, `shakehands_id`=request id, `curtime`=reactor monotonic) and `bed_mesh.py` registers `get_mesh`.
  - `pause_resume/check_continue_print_state` always replies `{'file_state': False, 'eeprom_state': False}`. It is a stub; it never reports a power-loss-resume state.
  - `getBootLoaderVersion` replies `{'software_version': <mcu constant 'software_version'>}` from the main `mcu` object.
  - The UI's startup traffic has been captured in the repo: `printer-backups/192.168.0.24/*/klippy-*.log` and `stepcompress-debug/logs/*.log` contain lines such as `Received ... "method":"list_endpoints"`, `"method":"info","params":{"client_info":{"program":"c440x","version":"V1.1.0.17"}}`, `objects/list`, and `register_remote_method` for `shutdown_machine`, `reboot_machine` and `pause_job_queue`.
  - Verification without the printer: the JSON-in/JSON-out shape of these endpoints can be checked offline by replaying those captured requests. What the UI does with the answers (whether `file_state:false` is accepted, what it expects from `shakehands`) cannot be verified without the physical pad and display.
- **Component 2: trigger_analog and PRTouch.** The stock Creality MCU firmware lacks the `trigger_analog`/`sos_filter`/`hx71x` commands, so `trigger_analog.py` cannot run on the unmodified KE board. The Python side can be verified offline against the `-o` debugoutput (it emits `config_sos_filter`, `config_trigger_analog`, `sos_filter_set_*` and `trigger_analog_*` commands). The numeric behaviour on real hardware cannot.
- **Component 3: Ender config uses modules from this slice** (`config/ender-3-v3-ke/printer.cfg`):
  - `tmc2208` x3 on UART pins `PB12`/`PB13`/`PB14` (`sense_resistor 0.150`, `run_current 0.75/0.75/0.80`, `stealthchop_threshold 0`; stealthchop is configured by the section options).
  - `[virtual_sdcard] path:/usr/data/printer_data/gcodes`.
  - `[safe_z_home]` (`home_xy_position 110,110`, `speed 120`, `z_hop 10`, `z_hop_speed 20`, `move_to_previous false`).
  - `[verify_heater extruder]` and `[verify_heater heater_bed]` with relaxed values.
  - `[resonance_tester]` (`accel_chip adxl345`, `accel_per_hz 70`, `probe_points 117.5,117.5,100`, `max_freq 90`) in `input_shaper.cfg`.
  - Sensors `EPCOS 100K B57560G104F` for both heaters via `thermistor.py` and `temperature_sensors.cfg`.
  - `temperature_mcu` is commented out because the GD32F303 MCU is unsupported (README).
  - Probe via `bltouch` (another slice) using `probe.py` helpers, `HomingViaProbeHelper` and `endstop_pin: probe:z_virtual_endstop`.
  - `[statistics]` is auto-loaded by `toolhead.py`; the captured logs show its `Stats <t>:` lines.

**Verification limits.** Everything in this slice can be parity-tested without the printer by running the Python and Rust implementations on the same config and G-code and diffing the `-o` MCU stream plus the `get_status()` objects and G-code responses. Exceptions: real serial devices (`palette2`), real sensor values (`sht3x`, `tsl1401cl`), the Creality UI behaviour, and the MCU/physical behaviour of `trigger_analog`.

---

## 3. Module-by-module map

Legend:
- **Deps** means klippy units, then 3p (third-party Python packages).
- **Contract** lists G-code commands with parameters, config options, status fields, webhooks, MCU commands, files and log lines.
- **Traits** are the runtime traits that complicate a port.
- **Tests** are the `test/klippy` files that exercise the module.
- **Fork** is any fork change.
- **Difficulty** is low, medium or high.

### 3.1 Temperature and heaters

**`thermistor.py` (108 lines)**
- Deps: `adc_temperature` (`PrinterADCtoTemperature`), `heaters.add_sensor_factory`. 3p: none.
- Contract: config `[thermistor <name>]` custom definitions with `temperature1/2/3`, `resistance1/2/3` or `beta`, plus per-sensor `pullup_resistor` (default 4700) and `inline_resistor`. Registers the factory under the thermistor name. Steinhart-Hart coefficients come from three points (falls back to beta with a `Using thermistor beta %.3f in heater %s` warning when `c3 <= 0`). `calc_adc(temp)` uses Cardano's closed-form cubic. The ADC bounds clamp to `[.00001,.99999]`.
- Traits: float math reaches the MCU: `heaters` converts `min_temp`/`max_temp` through `calc_adc` into `min_value`/`max_value` for `query_analog_in`, so last-bit float differences can change integer ADC limits.
- Tests: `temperature.test` (custom thermistor), and any config with a built-in sensor name (the example configs loaded by `printers.test`).
- Fork: none (equal line count).
- Difficulty: low. Small, pure `f64` math, but `pow(x-y,1/3)` and `log` must match libm.

**`temperature_sensors.cfg` (111 lines)**
- Deps: read by `heaters.py` via `pconfig.read_config(dirname/temperature_sensors.cfg)`. This file is data and must ship next to the Rust binary (or be embedded).
- Contract: loads modules `adc_temperature`, `bme280`, `ds18b20`, `htu21d`, `sht3x`, `aht10`, `lm75`, `spi_temperature`, `temperature_host`, `temperature_mcu`, `temperature_combined`, and defines 8 default thermistors, six three-point and two beta-only: `ATC Semitec 104GT-2`, `ATC Semitec 104NT-4-R025H42G`, `EPCOS 100K B57560G104F` (25 C/100000, 150 C/1641.9, 250 C/226.15; the Ender sensor), `Generic 3950`, `SliceEngineering 450`, `TDK NTCG104LH104JT1`, `Honeywell 100K 135-104LAG-J01` (beta 3974), `NTC 100K MGB18-104F39050L32` (beta 4100).
- Traits: it is Klipper config syntax and must be parsed by the same config reader. A load failure raises `Cannot load config '<file>'`.
- Tests: all configs with thermistors (everything in `printers.test`).
- Fork: none. Difficulty: low.

**`temperature_sensor.py` (42 lines)**
- Deps: `heaters` (`setup_sensor`, `register_sensor`). 3p: none.
- Contract: `[temperature_sensor <name>]` with `sensor_type`, `min_temp`, `max_temp`; status `temperature`, `measured_min_temp`, `measured_max_temp`; the `stats()` string `"%s: temp=%.1f"` goes into the `Stats` log line.
- Traits: reactor callback from the sensor. Tests: `temperature.test`, `linuxtest.test` (ds18b20). Fork: none. Difficulty: low.

**`temperature_combined.py` (182 lines)**
- Deps: `heaters.add_sensor_factory("temperature_combined")`, `printer.lookup_object(name)` for other sensors. 3p: none.
- Contract: config `sensor_list` (names of other objects with a `temperature` status), `maximum_deviation`, `combination_method` (min/max/mean); status `temperature` plus `humidity`/`pressure`/`gas` when members report them; calls `printer.invoke_shutdown("COMBINED SENSOR maximum deviation exceeded ...")`.
- Traits: reactor timer started 1 s after `klippy:ready` to avoid a race; dynamic dependency on other objects' `get_status`.
- Tests: `temperature.test` (`test_combined`). Fork: none. Difficulty: low.

**`temperature_host.py` (80 lines)**
- Deps: `heaters` factory. 3p: none.
- Contract: config `sensor_path` (default `/sys/class/thermal/thermal_zone0/temp`); status `temperature` (rounded to 2 decimals); timer interval 1.0 s; opens the file at config time and raises `Unable to open temperature file '<path>'` if it cannot.
- Traits: Linux-only file I/O. Tests: none. Fork: none. Difficulty: low.

**`temperature_mcu.py` (178 lines)**
- Deps: `adc_temperature.HelperTemperatureDiagnostics`, `pins` (`ADC_TEMPERATURE` pseudo pin), `mcu` constants. 3p: none.
- Contract: config `sensor_mcu`, `sensor_temperature1/2`, `sensor_adc1/2`; MCU query `debug_read order=%c addr=%u` -> `debug_result val=%u` to read factory calibration words (per-MCU tables: rp2040, sam3/4/e70, samd21/51, stm32f0/f1/f2/f4/g0/h7/h723). Logs `mcu_temperature '%s' nominal base=%.6f slope=%.6f`. Raises `MCU temperature not supported on <mcu>` for unknown MCUs; the KE's GD32F303 falls in that case, hence commented out.
- Traits: a table of hard-coded addresses and slopes; requires a live MCU (cannot be exercised in debugoutput with real calibration words).
- Tests: no config in `test/klippy`; `config/generic-bigtreetech-gtr.cfg` and `generic-bigtreetech-skr-pico-v1.0.cfg` load it via `printers.test`. Fork: none. Difficulty: low.

**`temperature_fan.py` (185 lines)**
- Deps: `fan.Fan` (with `default_shutdown_speed=1.`), `heaters`. 3p: none.
- Contract: `[temperature_fan <name>]`: `min_temp`, `max_temp`, `target_temp`, `max_speed`, `min_speed`, `control` = `watermark` (`max_delta`) or `pid` (`pid_Kp/Ki/Kd`, `pid_deriv_time`); `SET_TEMPERATURE_FAN_TARGET TEMPERATURE_FAN=<n> [TARGET= MIN_SPEED= MAX_SPEED=]`; status = the fan status plus `temperature` and `target`. Constants `PID_PARAM_BASE=255`.
- Traits: PID output feeds `fan` PWM values, so `f64` order of operations matters.
- Tests: `temperature.test` (several `temperature_fan test_*`). Fork: none. Difficulty: low-medium.

**`temperature_probe.py` (715 lines)**
- Deps: `mathutil`, `manual_probe`, `heaters`, probe objects (`probe` / `probe_eddy_current`), `configfile.set`, `toolhead`. 3p: none.
- Contract: `[temperature_probe <name>]` with `speed`, `horizontal_move_z`, `resting_z`, `calibration_position`, `calibration_bed_temp`, `calibration_extruder_temp`, `extruder_heating_z`, `smooth_time`, `min_temp`, `max_temp`, `calibration_temp`, `max_validation_temp`, `drift_calibration_min_temp`, `drift_calibration` (list of quadratic coefficient rows). Commands: `TEMPERATURE_PROBE_CALIBRATE PROBE=<n> [METHOD=manual|tap TARGET= STEP=]`, `TEMPERATURE_PROBE_NEXT`, `TEMPERATURE_PROBE_COMPLETE`, `TEMPERATURE_PROBE_ENABLE PROBE=<n> ENABLE=`, temporary `ABORT`. It writes `calibration_temp`, `drift_calibration`, `drift_calibration_min_temp` into SAVE_CONFIG. Status `temperature`, `measured_min_temp`, `measured_max_temp`, `in_calibration`, `estimated_expansion`, `compensation_enabled`. It registers drift compensation with the eddy probe (`register_drift_compensation`).
- Traits: registers and unregisters G-code commands at runtime (`register_command("ABORT", None)`); a 2D polynomial fit (a, b, c per height, solved with `mathutil.solve_linear_equations`) whose repr is printed into the config (`repr(poly)` format matters for SAVE_CONFIG); a reactor async callback; consumes ldc1612 bulk data.
- Tests: none (`eddy.cfg` has no `[temperature_probe]`). Fork: none. Difficulty: high (calibration state machine, interplay with eddy and bulk data, repr/precision of saved polynomials).

**`spi_temperature.py` (357 lines)**
- Deps: `bus.MCU_SPI_from_config`, `heaters.add_sensor_factory` for `MAX31855`, `MAX31856`, `MAX6675`, `MAX31865`. 3p: none.
- Contract: MCU `config_thermocouple oid spi_oid thermocouple_type`, `query_thermocouple oid clock rest_ticks min_value max_value max_invalid_count`; response `thermocouple_result oid next_clock value fault`. Config: `tc_type`, `tc_use_50Hz_filter`, `tc_averaging_count`, `rtd_nominal_r`, `rtd_reference_r`, `rtd_num_of_wires`, `rtd_use_50Hz_filter`. Faults are logged with `logging.warning`, for example `Max31856: Cold Junction Range Fault`. REPORT_TIME 0.300, MAX_INVALID_COUNT 3.
- Traits: Callendar-Van Dusen quadratic and its inverse for the RTD (float result reaches the MCU as min/max sample values).
- Tests: `temperature.test` (all four chips). Fork: whitespace only (+6 blank lines). Difficulty: low-medium.

**`sht3x.py` (181 lines)**
- Deps: `bus.MCU_I2C_from_config`, `heaters` factory `SHT3X`. 3p: none.
- Contract: config `sht3x_report_time`, `sensor_type`, i2c options; status `temperature`, `humidity`, probably `sensor_type`/`id` extras; I2C commands (BREAK, SOFTRESET, STATUS, periodic 2 Hz high-rep, FETCH 6 bytes, CRC check).
- Traits: uses `reactor.pause()` during init sleeps (1.5 ms), a reactor timer for sampling, CRC8 over sensor words.
- Tests: none. Fork: none. Difficulty: low-medium.

**`verify_heater.py` (93 lines)**
- Deps: `heaters.lookup_heater`, reactor timer, `invoke_shutdown`. 3p: none.
- Contract: `[verify_heater <heater>]`: `hysteresis` (5), `max_error` (120), `heating_gain` (2), `check_gain_time` (20; 60 for `heater_bed`). Log lines: `Starting heater checks for X`, `Heater X within range of T`, `Heater X approaching new target of T`, `Heater X no longer approaching target T`, `Heater X not heating at expected rate`; shutdown message plus the `HINT_THERMAL` hint. This is a safety function, so its exact thresholds matter.
- Traits: timer-driven (1 s) state machine fed by the smoothed temperature.
- Tests: `printers.test` (several example configs set it), none dedicated. Fork: none in the code; the Ender `printer.cfg` relaxes it. Difficulty: low.

**`pid_calibrate.py` (145 lines)**
- Deps: `heaters` (`PID_PARAM_BASE`, `lookup_heater`, `set_control`, `set_temperature`), `configfile.set`. 3p: none.
- Contract: `PID_CALIBRATE HEATER= TARGET= [WRITE_FILE=1]`; writes `/tmp/heattest.txt` (lines `pwm: t v`, then `t temp`); the response string `PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f ...`; stores `control=pid`, `pid_Kp`, `pid_Ki`, `pid_Kd` (3 decimals) for SAVE_CONFIG. Log: `Autotune: final: Kp=%f Ki=%f Kd=%f`, `Autotune: raw=...`. Requires 12 peaks; uses the Astrom-Hagglund and Ziegler-Nichols formulas.
- Traits: it replaces the heater's control object temporarily (duck-typed `temperature_update`, `check_busy`); relies on `reactor.pause` inside `set_temperature`. The Ender `printer.cfg` already has SAVE_CONFIG values `pid_kp 23.535` etc.
- Tests: none. Fork: none. Difficulty: low-medium.

**`z_thermal_adjust.py` (188 lines)**
- Deps: `heaters` (sensor), `gcode_move.set_move_transform(self, force=True)`, `homing:home_rails_end` event. 3p: none. `threading.Lock` only (defensive).
- Contract: `[z_thermal_adjust]`: `temp_coeff`, `z_adjust_off_above`, `max_z_adjustment`, `smooth_time`, `min_temp`, `max_temp` plus sensor options; `SET_Z_THERMAL_ADJUST [ENABLE= TEMP_COEFF= REF_TEMP=]` (multi-line response); status `temperature`, `measured_min_temp`, `measured_max_temp`, `current_z_adjust`, `z_adjust_ref_temperature`, `enabled`.
- Traits: move-transform chain (position/move hooks wrap the next transform), uses `kin.get_steppers()` for the Z step distance.
- Tests: none. Fork: none. Difficulty: medium (transform chain interplay with `gcode_move`).

### 3.2 Probes, leveling, compensation

**`probe.py` (631 lines)**
- Deps: `pins` (`register_chip('probe')`), `manual_probe`, `gcode_macro` templates, `toolhead`, `configfile`, `homing.probing_move` (via helpers). 3p: none.
- Contract:
  - Commands: `QUERY_PROBE`, `PROBE [PROBE_SPEED LIFT_SPEED SAMPLES SAMPLE_RETRACT_DIST SAMPLES_TOLERANCE SAMPLES_TOLERANCE_RETRIES SAMPLES_RESULT METHOD]`, `PROBE_CALIBRATE`, `PROBE_ACCURACY [SAMPLES]`, `Z_OFFSET_APPLY_PROBE`.
  - Config: `x_offset`, `y_offset`, `z_offset`, `speed`, `lift_speed`, `samples`, `sample_retract_dist`, `samples_result`, `samples_tolerance`, `samples_tolerance_retries`, `deactivate_on_each_sample`, `activate_gcode`, `deactivate_gcode`, `pin`.
  - Status: `name`, `last_query`, `last_probe_position`, `last_z_result`.
  - Event: `probe:update_results`, consumed by `axis_twist_compensation`.
  - Output: `probe: at %.3f,%.3f bed will contact at z=%.6f`, which appears in captured fork logs. Results from `PROBE_ACCURACY` include max/min/range/avg/median/sigma.
  - Pseudo-pin `probe:z_virtual_endstop`, used by `stepper_z` in the Ender config.
- Traits: Jinja2 templates; a shared helper layer reused by `bltouch`, `smart_effector`, `probe_eddy_current`, `load_cell_probe`, `bed_mesh`, `z_tilt`, `delta_calibrate` and others, so changes here ripple to many modules; uses a probe-session protocol (`start_probe_session`, `run_probe`, `pull_probed_results`, `end_probe_session`); the `gcode:command_error` event.
- Tests: `z_virtual_endstop.test`, `bed_mesh.test`, `multi_z.test`, `z_tilt.test`, `bltouch.test`, `smart_effector.test`, `eddy.test`, `load_cell.test`, `bed_screws.test`, `delta_calibrate.test`, `screws_tilt_adjust.test`. This is the best-covered module in the slice.
- Fork: none (631 matches upstream master). Difficulty: high (central abstraction, sessions, state across moves, many consumers).

**`probe_eddy_current.py` (1,075 lines)**
- Deps: `ldc1612`, `trigger_analog`, `probe`, `manual_probe`, `mathutil`, `mcu`, `homing`, `toolhead`, `temperature_probe` (drift compensation hook). 3p: none directly (SciPy only through `trigger_analog` for non-table filters).
- Contract:
  - Config: `[probe_eddy_current <name>]`: `sensor_type` (only `ldc1612`), `descend_z` (deprecated `z_offset`), `x_offset`, `y_offset`, `tap_z_offset`, `tap_threshold`, `calibrate` (list `height:frequency`), the probe-parameter options above.
  - Commands:
    - `PROBE_EDDY_CURRENT_CALIBRATE CHIP=<n> [PROBE_SPEED]`
    - `PROBE_EDDY_CURRENT_TAP_CALIBRATE [TAP=...]`
    - `PROBE` with `METHOD=scan|rapid_scan|tap`
    - `Z_OFFSET_APPLY_PROBE`
  - It saves the `calibrate` table to SAVE_CONFIG.
  - It builds a 25 Hz lowpass + derivative SOS filter for the MCU (scales to millihertz). Sample timing checks: error text `Eddy: Gaps in the data`, `Eddy: CLKIN frequency too low`.
  - Tap analysis fits a piecewise quadratic to frequency versus Z (`TapBestFit`, normal equations with an optimized cached-sum path) and enforces `insufficient lift`, `insufficient slope delta`, `invalid free air slope`, `invalid depress distance 0.030..0.250`.
- Traits: heavy float math whose output becomes the probed Z (and the MCU `trigger_value`: `adj_thresh = tap_threshold * probe_speed / sps`, `samp_thresh = int(1000*adj+0.5)`); `bisect` interpolation with `round(zpos, 6)`; reactor pauses inside analysis; `toolhead.register_lookahead_callback` for rapid scan.
- Tests: `eddy.test` (`PROBE METHOD=tap`, `PROBE_EDDY_CURRENT_TAP_CALIBRATE`, plain PROBE) with `eddy.cfg` using dummy calibration data. This only checks that nothing crashes.
- Fork: none behavioural; line count sits between upstream `6d43f8b3` and master `[INFERENCE: upstream drift]`.
- Difficulty: high (largest file, numerical algorithms, three probing modes).

**`trigger_analog.py` (411 lines)**
- Deps: `mcu` (`MCU_trsync.REASON_COMMS_TIMEOUT`, `TriggerDispatch`), sensor objects (`ldc1612`, `hx71x`, ADS drivers via `load_cell_probe`). 3p: scipy (lazy).
- Contract: MCU commands:
  - `config_sos_filter oid max_sections`
  - `sos_filter_set_section`, `sos_filter_set_state`, `sos_filter_set_offset_scale`, `sos_filter_set_active`
  - `config_trigger_analog oid sos_filter_oid`
  - `trigger_analog_query_state` -> `trigger_analog_state oid homing homing_clock`
  - `trigger_analog_set_raw_range`, `trigger_analog_set_trigger`, `trigger_analog_home`
  - Fixed-point helpers: `to_fixed_32` (Q-format, with overflow check), `calc_frac_bits`.
  - Trigger types such as `diff_peak_gt`.
- Traits: floating-to-fixed conversion uses Python `round()`, which rounds half to even, whereas Rust `f64::round` rounds half away from zero, so a naive port can differ by one count on ties. Mirrors C code in `src/trigger_analog.c` and `src/sos_filter.c`. Homing wait/stop flows use `TriggerDispatch` (trsync).
- Tests: `eddy.test` (tap), `load_cell.test`. Fork: none. Difficulty: high (fixed-point parity, scipy dependency, MCU protocol).

**`smart_effector.py` (183 lines)**
- Deps: `probe` (`ProbeEndstopWrapper`, helpers), `gcode`. 3p: none.
- Contract: config `pin`, `control_pin`, `probe_accel`, `recovery_time` + probe options; commands `SET_SMART_EFFECTOR [SENSITIVITY=0..255 ACCEL= RECOVERY_TIME=]`, `RESET_SMART_EFFECTOR`; MCU `config_digital_out`/`queue_digital_out` bit-banged serial to the Duet SmartEffector; status via `ProbeCommandHelper`.
- Traits: timing of the bit stream (`BITS_PER_SECOND`) is computed in print time; `request_move_queue_slot`.
- Tests: `smart_effector.test` (`PROBE`, `QUERY_PROBE`, `SET_SMART_EFFECTOR`). Fork: none. Difficulty: medium.

**`z_tilt.py` (173 lines)**
- Deps: `probe.ProbePointsHelper`, `mathutil.solve_linear_equations(allow_underdetermined=True)`, `force_move`/stepper access in `ZAdjustHelper`. 3p: none.
- Contract: `[z_tilt]`: `z_positions`, `retries`, `retry_tolerance`, `points`, `speed`, `horizontal_move_z`; `Z_TILT_ADJUST [RETRIES= RETRY_TOLERANCE= HORIZONTAL_MOVE_Z=...]`; status `applied` (reset on `stepper_enable:motor_off`). Log: `Calculating bed tilt with: ...`, `Calculated bed tilt parameters: x_adjust=... y_adjust=... z_adjust=...`.
- Traits: moves individual Z steppers by temporarily using `manual_move`; retry loop.
- Tests: `z_tilt.test`, `multi_z.test`, `quad_gantry_level.test` (shared config). Fork: none. Difficulty: medium.

**`quad_gantry_level.py` (128 lines)**
- Deps: `probe`, `z_tilt` (`RetryHelper`, `ZAdjustHelper`). 3p: none.
- Contract: `[quad_gantry_level]`: `gantry_corners`, `max_adjust`, `horizontal_move_z`, `retries`, `retry_tolerance`, four probe `points`; `QUAD_GANTRY_LEVEL`; status `applied`. Logs `quad_gantry_level f1: ..., f2: ...`.
- Traits: plain line fits. Tests: `quad_gantry_level.test` and `z_tilt.test` (shared `z_tilt.cfg`). Fork: none. Difficulty: low-medium.

**`screws_tilt_adjust.py` (130 lines)**
- Deps: `probe.ProbePointsHelper`. 3p: none.
- Contract: `[screws_tilt_adjust]`: `screw1..N` (+ `screwN_name`), `screw_thread` (`CW-M3` ... `CCW-M6`), probe options; `SCREWS_TILT_CALCULATE [MAX_DEVIATION= DIRECTION=CW|CCW]`; status `error`, `max_deviation`, `results`. Output uses sexagesimal-style turn counts (`CW 01:15`).
- Traits: string formatting parity (minutes rounding). Tests: `screws_tilt_adjust.test`. Fork: none. Difficulty: low.

**`safe_z_home.py` (93 lines)**
- Deps: `manual_probe.lookup_z_endstop_config`, `homing`, `gcode` (replaces `G28` and keeps `prev_G28`), `toolhead`. 3p: none.
- Contract: `[safe_z_home]`: `home_xy_position`, `z_hop`, `z_hop_speed`, `speed`, `move_to_previous`. Error `homing_override and safe_z_homing cannot be used simultaneously`. Used by the Ender config.
- Traits: wraps another module's command (`register_command("G28", ...)` returns the previous handler). Tests: none dedicated; configs `printer-anycubic-kobra-*.cfg` are loaded in `printers.test`. Fork: none. Difficulty: low.

**`skew_correction.py` (167 lines)**
- Deps: `gcode_move.set_move_transform(self, force=True)`, `configfile`. 3p: none.
- Contract: `[skew_correction]` plus `[skew_correction <profile>]` (`xy_skew`, `xz_skew`, `yz_skew`); commands `GET_CURRENT_SKEW`, `CALC_MEASURED_SKEW AC= BD= AD=`, `SET_SKEW [XY= XZ= YZ= CLEAR=]`, `SKEW_PROFILE [LOAD|SAVE|REMOVE=<name>]`; status `current_profile_name`.
- Traits: move transform chain; `math.tan/acos` factors. Tests: none. Fork: none. Difficulty: low-medium.

**`tuning_tower.py` (106 lines)**
- Deps: `gcode_move` (`set_move_transform`, auto-loaded by `toolhead.py`). 3p: none.
- Contract: `TUNING_TOWER COMMAND= PARAMETER= START= FACTOR= BAND= STEP_DELTA= STEP_HEIGHT= SKIP=`; emits `COMMAND PARAMETER=%.9f` (or no `=` for traditional G-code names); messages `Starting tuning test (...)`, `Ending tuning test mode`; test ends on a Z drop of more than 2.0 mm (`CANCEL_Z_DELTA`).
- Traits: injects G-code from inside a move transform (`run_script_from_command`). Tests: none. Fork: none. Difficulty: low.

### 3.3 TMC drivers

**`tmc.py` (739 lines)**
- Deps: `stepper.parse_step_distance`, `bulk_sensor`, `force_move.lookup_stepper`, `stepper_enable`, `heaters` (register_monitor), `pins` (virtual endstop chip), `homing` events, toolhead. 3p: none.
- Contract:
  - Commands: `SET_TMC_FIELD STEPPER= FIELD= [VALUE=|VELOCITY=]`, `INIT_TMC STEPPER=`, `SET_TMC_CURRENT STEPPER= [CURRENT= HOLDCURRENT=]`, `DUMP_TMC STEPPER= [REGISTER=]`.
  - Config: `driver_<FIELD>`, `diag_pin`/`diag0_pin`/`diag1_pin`, `stealthchop_threshold`, `coolstep_threshold`, `high_velocity_threshold`, `interpolate`, `driver_mslut*` wave table, plus the stepper section's `microsteps`.
  - Status: `mcu_phase_offset`, `phase_offset_position`, `drv_status`, `temperature`, `run_current`, `hold_current`.
  - Webhook: `tmc/stallguard_dump` (header `time, sg_result, cs_actual`).
  - Log lines: `TMC '<name>' reports ...`, `Pausing toolhead to calculate X phase offset`, `Stepper X phase change (was a now b)`, `Enabling TMC virtual enable for 'X'`.
  - Periodic driver error checks run each second. Sensorless homing uses the `<driver>_stepper_x:virtual_endstop` pin and the `homing:homing_move_begin/end` events.
- Traits: runtime pin chip registration; a register field-bit-packing helper (`FieldHelper`); `stepper:sync_mcu_position` plus `set_dir_inverted` events and phase-offset tracking via MSCNT; `reactor.register_callback` plus mutex; `invoke_shutdown` on driver errors; TSTEP computed from velocity with float math that goes straight into registers (`TMCtstepHelper`).
- Tests: `tmc.test` (DUMP_TMC, INIT_TMC, SET_TMC_CURRENT, SET_TMC_FIELD) with `tmc.cfg` (2130, 5160, 2209, 2208, 2660, 2240); example configs in `printers.test`.
- Fork: none. Difficulty: high (shared core, events, sensorless homing, phase tracking).

**`tmc_uart.py` (267 lines)**
- Deps: `pins` (`share_type=tmc_uart_rx`), `mcu`. 3p: none.
- Contract: MCU `config_tmcuart oid rx_pin pull_up tx_pin bit_time`, query `tmcuart_send oid write read` -> `tmcuart_response oid read`; analog mux uses `config_digital_out` + `update_digital_out`; config `uart_pin`, `tx_pin`, `select_pins`, `uart_address`; baud 40000 bit-bang, CRC8 poly 0x07; read requires `IFCNT` increment check on writes (5 retries). Errors: `Unable to read tmc uart '%s' register %s`, `Shared TMC uarts must use the same pins`.
- Traits: bit-banged serial frames built in Python (start/stop bit insertion, decode); the 10-byte response is validated by re-encoding.
- Tests: via `tmc.test` (tmc2208 `PK5`, tmc2209 `PA5`). The KE uses this path (`uart_pin` on `PB12-14`). Fork: none. Difficulty: medium.

**`tmc2130.py` (353 lines)**
- Deps: `bus.MCU_SPI_from_config`, `tmc`. Also provides `MCU_TMC_SPI`, `MCU_TMC_SPI_chain` and `TMCCurrentHelper`, reused by 2208/2209/2240/5160.
- Contract: config `cs_pin`, `spi_*`, `chain_length`, `chain_position`, `run_current`, `hold_current`, `sense_resistor` (0.110), `driver_*` fields; `TMC_FREQUENCY=13200000`; current formula `cs = int(32*(Rsense+0.020)*I*sqrt(2)/vref + .5) - 1` with vref 0.32 or 0.18 (vsense), clamped 0..31; SPI frames 5 bytes with daisy-chain padding; 5 write retries.
- Traits: float-to-register-bits. Tests: `tmc.test`. Fork: none. Difficulty: medium.

**`tmc2208.py` (230 lines)**
- Deps: `tmc`, `tmc_uart`, `tmc2130` (uses `TMCCurrentHelper`, `FieldFormatters`). 3p: none.
- Contract: `[tmc2208 <stepper>]`: `uart_pin`, `tx_pin`, `select_pins`, `run_current`, `hold_current`, `sense_resistor`, `stealthchop_threshold`, `driver_*` fields (multistep_filt, toff, hstrt, hend, tbl, iholddelay, pwm_* , freewheel, tpowerdown). `TMC_FREQUENCY=12000000`, register set including `IOIN@TMC220x`/`IOIN@TMC222x` translation. This is the KE's driver.
- Tests: `tmc.test` (`stepper_z`), `generic-*` examples. Fork: none. Difficulty: medium.

**`tmc2209.py` (110 lines)** - same family plus coolstep/stallguard4 fields (`semin`, `seup`, ...), `diag_pin`, `stealthchop_threshold`. Depends on `tmc2208`, `tmc2130`, `tmc`, `tmc_uart`. Tests: `tmc.test` (`stepper_y1`). Fork: none. Difficulty: medium.

**`tmc2240.py` (420 lines)** - `rref` (12000..60000), current via `KIFS=[11750,24000,36000,36000]/Rref/sqrt(2)` and a current range 0..3; UART (when `uart_pin` set) or SPI; `TMC_FREQUENCY=12500000`; `tmc_uart.MCU_TMC_uart(config, ..., 7, TMC_FREQUENCY)` and SPI paths. Tests: `tmc.test` (`stepper_z2`). Fork: none. Difficulty: medium.

**`tmc2660.py` (286 lines)** - own `MCU_TMC2660_SPI` (24-bit frames, 4 MHz default), `run_current` (max 2.4 A), `sense_resistor` (required), `idle_current_percent`, vsense choice with vref 0.165/0.310, `idle_timeout:printing` event handler to restore current. Tests: `tmc.test` (`stepper_z1`). Fork: none. Difficulty: medium.

**`tmc5160.py` (389 lines)** - `globalscaler`-based current (`VREF=0.325`, max 10 A, `sense_resistor` 0.075), SPI, `TMC_FREQUENCY=12000000`. Tests: `tmc.test` (`stepper_y`), `generic-duet3-6hc` (chain). Fork: none. Difficulty: medium.

All six chip modules share: 3p none; events: via `tmc.TMCCommandHelper`; status: via `cmdhelper.get_status` monkey-assigned onto the chip object (`self.get_status = cmdhelper.get_status`), which `objects/query` finds with `hasattr(o,'get_status')`.

### 3.4 Resonance and analysis

**`resonance_tester.py` (553 lines)**
- Deps: `shaper_calibrate`, accelerometer chips (`adxl345`, `lis2dw`, `mpu9250`, `icm20948`, `bmi160`, `angle`: anything with `start_internal_client()` / `get_samples()`), `input_shaper` (`disable_shaping`, `enable_shaping`, `apply_params`), `configfile`, `toolhead`, `gcode`. 3p: numpy (indirect).
- Contract:
  - Commands: `MEASURE_AXES_NOISE [MEAS_TIME=]`, `TEST_RESONANCES AXIS= [CHIPS= POINT= OUTPUT=resonances,raw_data NAME= FREQ_START FREQ_END ACCEL_PER_HZ HZ_PER_SEC SWEEPING_ACCEL SWEEPING_PERIOD INPUT_SHAPING]`, `SHAPER_CALIBRATE [AXIS= CHIPS= MAX_SMOOTHING= NAME=]`.
  - Config: `accel_chip` or `accel_chip_x/y/z`, `probe_points`, `min_freq`, `max_freq`, `max_freq_z`, `accel_per_hz`, `accel_per_hz_z`, `hz_per_sec`, `sweeping_accel(_z)`, `sweeping_period`, `move_speed`, `max_smoothing`.
  - Files: `/tmp/resonances_<axis>_<name>.csv`, `/tmp/calibration_data_<axis>_<name>.csv`, raw data `/tmp/raw_data_*.csv`.
  - Responses: `Recommended shaper_type_<axis> = <name>, shaper_freq_<axis> = %.1f Hz`, `Axes noise for ...`.
  - Writes `shaper_type_*`/`shaper_freq_*` to SAVE_CONFIG through `ShaperCalibrate.save_params`.
- Traits: generates the vibration test move sequence (accelerations, sign alternation) from floats that feed straight into the move planner; the test runs toolhead moves while accelerometers record; the heavy math runs in a child process.
- Tests: `input_shaper.cfg` has `[resonance_tester]`, but `input_shaper.test` only runs `SET_INPUT_SHAPER`. The module is loaded but `TEST_RESONANCES` is never run. Fork: none (+0 lines vs upstream 6d43f8b3).
- Difficulty: high (hardware-dependent, numpy-based analysis).

**`shaper_calibrate.py` (493 lines)**
- Deps: `shaper_defs` (via `importlib.import_module('.shaper_defs','extras')`), `queuelogger.clear_bg_logging`, `multiprocessing`. 3p: numpy.
- Contract: class `CalibrationData` (`freq_bins`, `psd_sum`, `psd_x/y/z`, `normalize_to_frequencies`), `ShaperCalibrate` (PSD via Welch with Kaiser window beta 6, `WINDOW_T_SEC=0.5`; `find_best_shaper`, `fit_shaper`, `find_shaper_max_accel`, `save_params`, `apply_params`, `save_calibration_data`), constants `MIN_FREQ=5`, `MAX_FREQ=200`, `MAX_SHAPER_FREQ=150`, `TEST_DAMPING_RATIOS=[0.075,0.1,0.15]`, `AUTOTUNE_SHAPERS=[zv,mzv,ei,2hump_ei,3hump_ei]`. CSV header `freq,psd_x,psd_y,psd_z,psd_xyz` (+ `,shapers:,<name>(<freq>)`), value formats `%.1f`, `%.3e`, `%.3f`.
- Traits: numpy vectorization; a child process with a Pipe for the calculation; scoring uses `_bisect`; the recommended shaper (best score) depends on floating-point details; the CSV is consumed by `scripts/*` tools.
- Tests: none. Fork: one blank line. Difficulty: high (numeric parity of the recommended shaper_freq; FFT implementation must match numpy's `rfft` to the tolerance that flips the best-shaper decision).

**`shaper_defs.py` (197 lines)**
- Deps: `mathutil` (pseudo-inverse, mat mult/transpose). 3p: none. Consumers: `input_shaper.py`, `shaper_calibrate.py`, `scripts/graph_shaper.py`.
- Contract: shaper tables `zv`, `mzv`, `zvd`, `ei`, `2hump_ei`, `3hump_ei` as `(A, T)` pulse lists, each with `min_freq` and `max_damping_ratio`; `DEFAULT_DAMPING_RATIO=0.1`, `SHAPER_VIBRATION_REDUCTION=20`; `init_shaper(name, freq, damping, error=)` parses `name(args)` with positional or named args (`mzv(n,t)` / `mzv(n=,tau=)`); `ShaperError` text `Too small n=...`, `Ill-formed shaper ...`.
- Traits: the (A, T) values are handed to the C `kin_shaper.c` through `input_shaper`, so they directly affect step timing. They need bit-compatible `f64` results from `exp`, `sqrt`, `cos`, `sin`, plus the same pseudo-inverse algorithm for MZV.
- Tests: `input_shaper.test` (`mzv(5,0.6)`, `2hump_ei`) through `input_shaper.py`. Fork: none. Difficulty: medium.

### 3.5 Print job and host state

**`virtual_sdcard.py` (487 lines)**
- Deps: `print_stats`, `gcode_macro` (template `on_error_gcode`), `gcode` (run_script, mutex), reactor, `pause_resume`/`sdcard_loop` (callers), `webhooks`, `klippy:analyze_shutdown` event. 3p: none.
- Contract:
  - Commands: `M20`, `M21`, `M23 <path>`, `M24`, `M25`, `M26 S=`, `M27`; `M28`/`M29`/`M30` raise `SD write not supported`; `SDCARD_RESET_FILE`; `SDCARD_PRINT_FILE FILENAME=`.
  - Config: `path` (required), `on_error_gcode`.
  - Status: `file_path`, `progress`, `is_active`, `file_position`, `file_size`.
  - Behaviour: reads 8192-byte chunks; extensions `gcode`, `g`, `gco`; M20 prints `Begin file list` / `End file list`; `File opened:%s Size:%d`, `File selected`, `Done printing file`; case-insensitive file lookup; subdirectories only for `SDCARD_PRINT_FILE`.
  - Log lines: `Starting SD card print (position %d)`, `Exiting SD card print (position %d)`, `Finished SD card print`, `Virtual sdcard (%d): ... Upcoming (%d): ...` on shutdown, `sd_pos=%d` in the Stats line.
  - Calls `print_stats.note_start/pause/complete/error/cancel/reset/set_current_file` and sends event `virtual_sdcard:reset_file`.
- Traits: runs as a reactor timer coroutine that calls `gcode.run_script` and `reactor.pause` repeatedly (greenlet semantics: mutex test, pause 0.050 s, `reactor.pause(NOW)` after each read); `next_file_position` protocol with `sdcard_loop`; byte-position accounting uses `len(line.encode()) + 1` (UTF-8 bytes, newline = 1 byte).
- Tests: `sdcard_loop.test` (with `test/klippy/sdcard_loop/*.gcode`, running M808 loops). Also loaded in the Ender config.
- **Fork:** `[DEBUG-motion] v4` instrumentation (+179 lines): `DebugSDChunk` rings (24 recent chunks, 16 slow chunks/commands/yields), thresholds 20 ms, and a dump on `klippy:analyze_shutdown`. The dump logs `[DEBUG-motion] version=host-v4 sdcard timing chunks=... cmds=... pending=... pos=a/b ...`, `sdcard chunk pending/slow/recent`, `sdcard cmd at= dur= pos= name= ok=`, `sdcard yield at= dur= kind= pos=` at ERROR level. The G-code line itself is never logged, only the first 24 chars of the command name. Dispatch behaviour is unchanged. The Rust port should decide whether to carry this instrumentation; it is observability only. See commit candidates in section 0.
- Difficulty: medium (coroutine/mutex semantics around the G-code dispatcher; the file I/O is trivial).

**`sdcard_loop.py` (73 lines)**
- Deps: `virtual_sdcard` (`is_cmd_from_sd`, `get_file_position`, `set_file_position`). 3p: none.
- Contract: `SDCARD_LOOP_BEGIN COUNT=`, `SDCARD_LOOP_END`, `SDCARD_LOOP_DESIST`; errors `Only permitted in SD file.` / `Only permitted outside of a SD file.`; log `Desisting existing SD loops`.
- Traits: depends on the exact `next_file_position` skip logic in `virtual_sdcard.work_handler`. Tests: `sdcard_loop.test`. Fork: none. Difficulty: low.

**`print_stats.py` (125 lines)**
- Deps: `gcode_move` (position, extrude factor), reactor time, event `extruder:activate_extruder`. 3p: none.
- Contract: `SET_PRINT_STATS_INFO [TOTAL_LAYER= CURRENT_LAYER=]`; status `filename`, `total_duration`, `print_duration`, `filament_used`, `state` (`standby`, `printing`, `paused`, `complete`, `error`, `cancelled`), `message`, `info.total_layer`, `info.current_layer`. These feed Moonraker, Mainsail and the Creality UI.
- Traits: wall-clock (reactor monotonic) dependent. Tests: loaded by `virtual_sdcard` (`sdcard_loop.test`). Fork: none. Difficulty: low.

**`pause_resume.py` (111 lines)**
- Deps: `virtual_sdcard` (`do_pause`, `do_resume`, `do_cancel`), `gcode` (`SAVE_GCODE_STATE`/`RESTORE_GCODE_STATE`), `webhooks`, `mcu.get_constants()`. 3p: none.
- Contract:
  - Commands: `PAUSE`, `RESUME [VELOCITY=]`, `CLEAR_PAUSE`, `CANCEL_PRINT`.
  - Config: `recover_velocity` (default 50).
  - Status: `is_paused`.
  - Responses: `action:paused`, `action:resumed`, `action:cancel`, `Print already paused`, `Print is not paused, resume aborted`.
  - Webhooks: `pause_resume/cancel`, `pause_resume/pause`, `pause_resume/resume` (upstream) plus the two **Creality endpoints** `pause_resume/check_continue_print_state` and `getBootLoaderVersion` (see section 2).
  - Auto-loaded by `filament_switch_sensor` when `pause_on_runout` is true, as in the Ender config.
- Traits: `RESUME` issues `RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f`. Other modules call `send_pause_command()` directly (`palette2`, `filament_*`).
- Tests: loaded (no commands run) in `extruders.test` through `filament_switch_sensor`. Fork: **yes**, +11 lines (the two endpoint registrations and handlers). Difficulty: low (the logic is tiny; the risk is the unspecified Creality UI contract).

**`respond.py` (55 lines)** - Deps: `gcode` only. Contract: `M118 <text>`, `RESPOND [TYPE=echo|command|error|echo_no_space PREFIX= MSG=]`, config `default_type` (`echo`->`echo:`, `command`->`//`, `error`->`!!`) and `default_prefix`; output format `"%s %s"`. Tests: none. Fork: none. Difficulty: low.

**`save_variables.py` (66 lines)** - Deps: `configparser`, `ast`, `os`, `gcode`. Contract: config `filename` (file is created if missing); `SAVE_VARIABLE VARIABLE= VALUE=` (lowercase names only; values parsed with `ast.literal_eval`); file format is an INI section `[Variables]` with `name = repr(value)`, sorted by name; status `variables` (dict) which macros read as `printer.save_variables.variables.<name>`. Errors: `Unable to parse existing variable file`, `VARIABLE must not contain upper case`, `Unable to parse '%s' as a literal`. Traits: Python-literal parsing and `repr` compatibility, ConfigParser lowercasing and interpolation behaviours. Tests: none. Fork: none. Difficulty: medium (Python-literal semantics).

**`statistics.py` (74 lines)** - Deps: reactor timer; calls every object's `stats(eventtime)` (so each module's `stats()` string is a log contract); `/proc/meminfo`, `os.getloadavg`, `time.process_time`. Contract: logs `Stats %.1f: <parts>` every 1.0 s (only if some part returns `True`), system part `sysload=%.2f cputime=%.3f memavail=%d`; status of object `system_stats`: `sysload`, `cputime`, `memavail`; disabled in debugoutput mode. The fork's capture logs and `scripts/graphstats.py`/`logextract.py` parse these lines. Auto-loaded by `toolhead.py`. Tests: loaded in every test (no assertions). Fork: none. Difficulty: low (Linux-only).

**`query_adc.py` (35 lines)** - Deps: `adc_temperature`, `adc_scaled`, `buttons` register ADCs. Contract: `QUERY_ADC [NAME= PULLUP=]` with response of the last ADC value and timestamp (and the computed resistance when PULLUP is given, clamping to `[.00001,.99999]`); `NAME` omitted lists `Available ADC objects: ...`. Tests: indirectly via temperature sensors. Fork: none. Difficulty: low.

**`query_endstops.py` (45 lines)** - Deps: `webhooks`, `gcode` mutex, `toolhead.wait_moves`, registered endstops from `stepper.py`. Contract: `QUERY_ENDSTOPS` and `M119`; webhook `query_endstops/status`; status `last_query` (dict name -> `open`/`TRIGGERED`). Tests: `commands.test`, `corexyuv.test`, `dual_carriage.test`, `generic_cartesian*.test`, `hybrid_corexy_dual_carriage.test`. Fork: none. Difficulty: low.

**`stepper_enable.py` (147 lines)** - Deps: `pins` (`digital_out` enable pins, shared pins counted), `toolhead.dwell`, `kinematics.clear_homing_state`. Contract: `M18`, `M84`, `SET_STEPPER_ENABLE STEPPER= ENABLE=`; status `steppers` (name -> enabled); event `stepper_enable:motor_off`; callbacks used by `tmc`. Log `<stepper> has been manually enabled/disabled`. Constants `DISABLE_STALL_TIME=0.100`. Traits: toolhead timing (`dwell`, `get_last_move_time`). Tests: `commands.test` (M18), everything that homes. Fork: none. Difficulty: medium.

### 3.6 Pins and MCU peripherals

**`pwm_tool.py` (187 lines)**
- Deps: `chelper` (FFI `syncemitter_queue_msg`), `motion_queuing.allocate_syncemitter`/`register_flush_callback`/`note_mcu_movequeue_activity`, `pins`. 3p: cffi.
- Contract: `[pwm_tool <name>]`: `pin`, `cycle_time`, `hardware_pwm`, `scale`, `value`, `shutdown_value`, `maximum_mcu_duration`; `SET_PIN PIN=<name> VALUE=`; status `value`; MCU commands `config_pwm_out`/`queue_pwm_out`, soft PWM via `config_digital_out`, `set_digital_out_pwm_cycle`, `queue_digital_out`.
- Traits: bypasses the usual command queue by pushing raw message bytes through the C syncemitter with a clock; ties into `motion_queuing` (core, which is fork-modified for debug).
- Tests: `pwm.test` (`pwm_tool test_pwm_tool`). Fork: none in the file (187 lines equals upstream `6d43f8b3`). Difficulty: medium-high (C-helper boundary).

**`pwm_cycle_time.py` (125 lines)** - `[pwm_cycle_time <name>]`: `pin`, `value`, `shutdown_value`, `cycle_time`, `scale`; `SET_PIN PIN= VALUE= [CYCLE_TIME=]`; MCU `config_digital_out`, `set_digital_out_pwm_cycle`, `queue_digital_out`; status `value`. Float to ticks with `+0.5` rounding. Tests: `pwm.test`. Fork: none. Difficulty: low-medium.

**`static_pwm_clock.py` (41 lines)** - `[static_pwm_clock <name>]`: `pin`, `frequency` (greater than 1/0.3, up to 520 MHz); requires `mcu_freq // frequency` to give an exact integer or raises the config error `Frequency output must be without remainder...`; warns `pulse width < 40%` / `> 60%`. Static duty 0.5. Tests: none. Fork: none. Difficulty: low.

**`static_digital_output.py` (19 lines)** - `[static_digital_output <name>]`: `pins` list; emits `set_digital_out pin=%s value=%d` at config time. Tests: none. Fork: none. Difficulty: low.

**`servo.py` (76 lines)** - Deps: `output_pin` (`GCodeRequestQueue`). `[servo <name>]`: `pin`, `maximum_servo_angle`, `minimum_pulse_width`, `maximum_pulse_width`, `initial_angle`, `initial_pulse_width`; `SET_SERVO SERVO= [WIDTH=|ANGLE=]`; status `value`. Fixed 20 ms period. Tests: `corexyuv.test`, `dual_carriage.test`, `generic_cartesian.test` (SET_SERVO). Fork: none. Difficulty: low.

**`pulse_counter.py` (81 lines)** - Provides `MCU_counter` and `FrequencyCounter` for `fan` tachometers and others. MCU `config_counter oid pin pull_up`, `query_counter oid clock poll_ticks sample_ticks`, response `counter_state oid next_clock count count_clock`. Traits: 32-bit counter wraparound handling, frequency computed in floating point. Tests: none directly (used by `fan` tachometer configs). Fork: none. Difficulty: low-medium.

**`samd_sercom.py` (41 lines)** - `[samd_sercom <name>]`: `sercom`, `tx_pin`, `rx_pin`, `clk_pin`; emits `set_sercom_pin bus=%s sercom_pin_type=tx|rx|clk pin=%s`; error `SERCOM pins must be on same mcu`. Tests: none. Fork: none. Difficulty: low.

**`sx1509.py` (191 lines)** - Deps: `bus.MCU_I2C_from_config` (async write only, 400 kHz), `pins.register_chip("sx1509_<name>")`. Provides digital out and PWM (hardware-ish via LED driver registers) pins named `PIN_<n>` through `setup_pin`; I2C register writes with print-time scheduling. Tests: only via `generic-duet2-duex.cfg` in `printers.test`. Fork: one blank line. Difficulty: medium.

**`replicape.py` (268 lines)** - Deps: `bus.MCU_SPI` (shift registers on `spidev1.1` or `spidev2.1`), `mcu.get_printer_mcu`, `pins.register_chip('replicape')`, Linux sysfs `/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/` (BeagleBone). `[replicape]`: `revision B3`, `host_mcu`, `enable_pin`, `stepper_{x,y,z,e,h}_microstep_mode`, `_chopper_*`, `_current` (max 3.84 A), `standstill_power_down`. MCU `config_pca9685 oid bus addr channel cycle_ticks value default_value max_duration`, `queue_pca9685_out`. Traits: board-specific hardware; reads sysfs to find `pwmchipN`; async callbacks from MCU command paths. Tests: only `config/generic-replicape.cfg` via `printers.test`. Fork: none. Difficulty: medium (niche, but a deterministic MCU command mapping).

### 3.7 LEDs

**`pca9533.py` (38 lines)** - Deps: `bus.MCU_I2C_from_config` (default address 98), `led.LEDHelper(count=1)`, event `klippy:connect`. `[pca9533 <name>]`: LED options of `led.LEDHelper` (`initial_RED`, `initial_GREEN`, `initial_BLUE`, `initial_WHITE`), `SET_LED LED=<name> RED GREEN BLUE [TRANSMIT SYNC]`; status = led status. Writes PWM0=85, PWM1=170 at connect. `BACKGROUND_PRIORITY_CLOCK=0x7fffffff00000000`. Tests: `led.test` (`p5led`). Fork: none. Difficulty: low.

**`pca9632.py` (70 lines)** - Same pattern, `color_order` (default `RGBW`), register cache `prev_regs`, 4 channels. Tests: `led.test` (`p6led`, with software I2C pins). Fork: none. Difficulty: low.

### 3.8 Displays

**`display/st7920.py` (257 lines)** - Deps: `bus`, `font8x14`; loaded by `display/display.py`. Two drivers: `ST7920` (hardware via the MCU `st7920` module) and `EmulatedST7920` (software SPI plus the `en_pin` sequencing). Config: `cs_pin`, `sclk_pin`, `sid_pin`, or `spi_software_{miso,mosi,sclk}_pin`, `en_pin`, `spi_speed`. MCU: `config_st7920 oid cs_pin sclk_pin sid_pin sync_delay_ticks cmd_delay_ticks`, `st7920_send_cmds oid cmds`, `st7920_send_data oid data`. Traits: framebuffer diffing and custom glyph caching (animated icons `fan1/fan2`, `bed_heat1/2`). Tests: only through `kit-voron2-250mm.cfg` and Anycubic/Anet configs in `printers.test`. Fork: none. Difficulty: medium.

**`display/uc1701.py` (241 lines)** - Despite the name it holds `UC1701`, `SSD1306` and `SH1106` classes plus IO wrappers (`SPI4wire`, `I2C`) and `ResetHelper`. Config: `a0_pin`, `rst_pin`, `contrast`, `cs_pin`, `reset_pin`, `vcomh`, `invert`, `x_offset`, i2c/spi options. Uses SPI/I2C writes with `BACKGROUND_PRIORITY_CLOCK`; command bytes are hard-coded init sequences. Tests: only via `printers.test` configs that select these LCD types. Fork: none. Difficulty: medium.

### 3.9 Misc hardware

**`palette2.py` (654 lines)**
- Deps: `virtual_sdcard`, `pause_resume`, `gcode_move`, `toolhead`, extruder heater (`can_extrude`), `pyserial`, `queue`. 3p: pyserial.
- Contract: `[palette2]`: `serial`, `baud` (115200), `feedrate_splice` (0.8), `feedrate_normal` (1.0), `auto_load_speed` (2), `auto_cancel_variation`. Commands: `PALETTE_CONNECT`, `PALETTE_DISCONNECT`, `PALETTE_CLEAR`, `PALETTE_CUT`, `PALETTE_SMART_LOAD`, and every `O0`..`O32` code (specific handlers `O1`, `O9`, `O20`, `O21`..`O32`; the rest map to `cmd_OmegaDefault` which forwards to the device). Serial protocol: `O99` heartbeat every 5 s (timeout 11 s), `O50` firmware, `O31` ping, `O51/O52` file list, `O10 D5` cut, `O102 D1` stop smart load; the printer-to-host packets are parsed by `p2cmd_O*` handlers. Status: `ping`, `remaining_load_length`, `is_splicing`.
- Traits: three reactor timers (read, write, heartbeat) plus `queue.Queue` buffers over a non-blocking `serial.Serial(timeout=0)`; `reactor.pause` inside `_wait_for_heartbeat`; dynamic command registration by `getattr`.
- Tests: none. Fork: whitespace only (+2 lines `[INFERENCE]`). Difficulty: medium-high (stateful protocol, needs hardware or a simulator to verify). Not used on the KE.

**`tsl1401cl_filament_width_sensor.py` (154 lines)** - Deps: `pins` ADC, reactor timer, `gcode` (runs `M221 S<pct>`). `[tsl1401cl_filament_width_sensor]`: `pin`, `default_nominal_filament_diameter`, `measurement_delay`, `max_difference`, `min_diameter`, `ADC_*` constants; commands `QUERY_FILAMENT_WIDTH`, `RESET_FILAMENT_WIDTH_SENSOR`, `DISABLE_FILAMENT_WIDTH_SENSOR`, `ENABLE_FILAMENT_WIDTH_SENSOR`; no `get_status` (unlike the hall variant). Behaviour: queues readings by extruder position (`MEASUREMENT_INTERVAL_MM`), then sets the extrusion multiplier through G-code `M221 S%d` using `round(nominal^2/width^2*100)`. Traits: injects G-code at 1 s timer cadence. Tests: none. Fork: none. Difficulty: low.

---

## 4. Test coverage summary (module to test)

| Test | Modules in this slice that it exercises |
|---|---|
| `tmc.test` + `tmc.cfg` | `tmc`, `tmc_uart`, `tmc2130`, `tmc2208`, `tmc2209`, `tmc2240`, `tmc2660`, `tmc5160` (DUMP_TMC, INIT_TMC, SET_TMC_CURRENT, SET_TMC_FIELD) |
| `temperature.test` | `thermistor`, `temperature_sensor`, `temperature_combined`, `temperature_fan`, `spi_temperature` (all 4 chips), `heaters` |
| `eddy.test` | `probe_eddy_current`, `trigger_analog`, `probe`, `ldc1612` |
| `z_virtual_endstop.test`, `multi_z.test`, `bltouch.test`, `bed_mesh.test`, `smart_effector.test`, `load_cell.test` | `probe` (and helpers), `smart_effector`, `trigger_analog` |
| `z_tilt.test`, `quad_gantry_level.test`, `multi_z.test` | `z_tilt`, `quad_gantry_level` |
| `screws_tilt_adjust.test` | `screws_tilt_adjust` |
| `sdcard_loop.test` | `virtual_sdcard`, `sdcard_loop`, `print_stats` |
| `pwm.test` | `pwm_tool`, `pwm_cycle_time` |
| `led.test` | `pca9533`, `pca9632` |
| `corexyuv.test`, `dual_carriage.test`, `generic_cartesian.test` | `servo` |
| `input_shaper.test` | `shaper_defs` (via `input_shaper`); `resonance_tester` is only loaded, not run |
| `commands.test` | `query_endstops`, `stepper_enable` (M18) |
| `extruders.test` | `pause_resume` is loaded (via `filament_switch_sensor`), no command run |
| `printers.test` (about 60+ shipped `config/*.cfg`) | load-only coverage of `st7920`, `uc1701`/SSD1306/SH1106 variants, `replicape`, `sx1509`, `safe_z_home`, `verify_heater`, `temperature_mcu`, tmc variants |
| every test | `statistics`, `stepper_enable`, `query_endstops`, `print_stats` get loaded |

**No test of any kind:** `palette2`, `pid_calibrate`, `pause_resume` commands, `respond`, `save_variables`, `safe_z_home` (dedicated), `skew_correction`, `static_pwm_clock`, `static_digital_output`, `samd_sercom`, `sht3x`, `temperature_host`, `temperature_probe`, `tsl1401cl`, `tuning_tower`, `z_thermal_adjust`, `shaper_calibrate`, `query_adc`, `pulse_counter` and the Creality endpoints. Because the existing harness never compares output, even the "covered" modules are only smoke-tested. The port needs golden `-o` streams captured from Python first.

---

## 5. Port-difficulty summary

| Rating | Modules |
|---|---|
| **High** | `probe` (central abstraction, many consumers), `probe_eddy_current` (1,075 lines of numeric analysis), `trigger_analog` (fixed-point plus SciPy design plus MCU protocol), `tmc` (shared core: events, sensorless homing, phase tracking), `temperature_probe` (calibration state machine), `shaper_calibrate` (numpy/FFT parity), `resonance_tester` (hardware and numpy dependent) |
| **Medium-high** | `pwm_tool` (C syncemitter boundary), `palette2` (stateful serial protocol) |
| **Medium** | `virtual_sdcard`, `stepper_enable`, `z_tilt`, `z_thermal_adjust`, `smart_effector`, `save_variables`, `shaper_defs`, `tmc_uart`, `tmc2130`, `tmc2208`, `tmc2209`, `tmc2240`, `tmc2660`, `tmc5160`, `sx1509`, `replicape`, `display/st7920`, `display/uc1701` |
| **Low-medium** | `temperature_fan`, `pid_calibrate`, `spi_temperature`, `sht3x`, `skew_correction`, `quad_gantry_level`, `pwm_cycle_time`, `pulse_counter` |
| **Low** | `thermistor`, `temperature_sensors.cfg`, `temperature_sensor`, `temperature_combined`, `temperature_host`, `temperature_mcu`, `verify_heater`, `screws_tilt_adjust`, `safe_z_home`, `tuning_tower`, `sdcard_loop`, `print_stats`, `pause_resume`, `respond`, `statistics`, `query_adc`, `query_endstops`, `static_pwm_clock`, `static_digital_output`, `servo`, `samd_sercom`, `pca9533`, `pca9632`, `tsl1401cl_filament_width_sensor` |

---

## 6. Cross-cutting risks seen in this slice

1. **The probe layer is a shared API.** `probe.py` is imported by `bltouch`, `smart_effector`, `probe_eddy_current`, `load_cell_probe`, `bed_mesh`, `z_tilt`, `quad_gantry_level`, `screws_tilt_adjust`, `delta_calibrate`, `axis_twist_compensation`, `temperature_probe`. Its probe-session protocol, `probe:update_results` event and `ProbeResult` objects must be frozen and specified before those consumers are ported in parallel.
2. **Rounding and libm differences.** Python's `round()` (half to even), `int()` truncation toward zero, `%`-formatting of floats (`%.3f`, `%.6f`, `%.9f`, `%.3e`, `repr`) and libm-level `exp`/`pow`/`log`/`cos` results all end up in MCU commands, G-code text, SAVE_CONFIG text or the recommended shaper. This is most acute in `trigger_analog` (`to_fixed_32`), `thermistor`, TMC current and TSTEP math, `shaper_defs`, `shaper_calibrate` and `probe_eddy_current` (`samp_thresh`).
3. **numpy/SciPy replacement.** `shaper_calibrate` needs an `rfft`, a Kaiser window, `np.interp` and a vector-matrix pipeline in Rust with a parity tolerance defined by "best shaper unchanged", not by bitwise equality. SciPy's `butter`/`iirnotch`/`tf2sos` is needed for every digital filter except the one pre-generated table.
4. **Greenlet-style cooperative scheduling.** `virtual_sdcard.work_handler`, `pause_resume.do_pause`, `palette2._wait_for_heartbeat`, `sht3x`, `tmc` enable/disable callbacks and every probe command rely on `reactor.pause()`, G-code mutexes and ordering inside one thread. The Rust runtime must offer an equivalent suspend/resume model (async tasks with an explicit reactor, or stackful coroutines) before these are ported.
5. **Move-transform chain** (`skew_correction`, `z_thermal_adjust`, `tuning_tower`, `bed_mesh`): each wraps the previous transform and `reset_last_position` is called on enable. The order of `set_move_transform(force=True)` calls decides the composition.
6. **Runtime command (un)registration** (`register_command("ABORT", None)`, `G28` override chain in `safe_z_home`, mux commands per section name, `O0..O32` in `palette2`). The Rust G-code layer must support replace, remove and "return previous handler".
7. **`get_status` objects feed Jinja templates.** Fields listed in this report are an external contract with macros, Moonraker, Mainsail and the Creality UI. Missing or renamed fields silently break macros (`printer.<obj>.<field>`).
8. **Webhook surface is partly Creality-private.** `shakehands`, `getBootLoaderVersion`, `pause_resume/check_continue_print_state`, `get_mesh`, and the UI's reliance on `list_endpoints`, `register_remote_method` (`shutdown_machine`, `reboot_machine`, `pause_job_queue`). The captured request logs are the only specification; the stub values (`file_state:false`) are guesses that happen to satisfy the UI.
9. **Log-line contracts.** `Stats %.1f: ...` (with per-module `stats()` strings such as `sd_pos=%d`, `<name>: temp=%.1f`), `[DEBUG-motion] ...` lines, `probe: at X,Y bed will contact at z=...`, `Starting/Exiting SD card print`, `Autotune:` lines. Fork tooling and `scripts/` parse several of them.
10. **Fork debug instrumentation.** `virtual_sdcard.py` and the core `motion_queuing.py`, `reactor.py`, `mcu.py`, `klippy.py`, `toolhead.py`, `chelper/*.c` carry the `[DEBUG-motion] v4` additions. Decide once, for the whole port, whether to port the instrumentation or drop it.
11. **Python literal and repr compatibility.** `save_variables` and `temperature_probe` (polynomial `repr`) write Python-formatted data back to disk and then re-read it.
12. **Test harness gap.** There are no golden outputs; building the golden-stream harness is a prerequisite for claiming parity for any module here.

## 7. Suggested port order (leaves first)

1. **Pure leaves with no klippy dependency beyond config and G-code:** `shaper_defs` (plus `mathutil`), `thermistor` and `temperature_sensors.cfg`, `respond`, `save_variables`, `static_digital_output`, `static_pwm_clock`, `samd_sercom`, `query_adc`, `statistics`, `print_stats`.
2. **Simple pin and peripheral modules:** `servo`, `pwm_cycle_time`, `pulse_counter`, `pca9533`, `pca9632`, `sx1509`, `temperature_host`, `temperature_sensor`, `temperature_combined`, `temperature_mcu`, `spi_temperature`, `sht3x`, `verify_heater`, `pid_calibrate`, `temperature_fan`, `query_endstops`, `stepper_enable`.
3. **SD-card and job control:** `virtual_sdcard`, `sdcard_loop`, then `pause_resume` (including the two Creality endpoints), because the other two depend on the first.
4. **TMC stack:** `tmc` (core), `tmc_uart`, `tmc2130`, then `tmc2208` (the KE driver), `tmc2209`, `tmc5160`, `tmc2240`, `tmc2660`.
5. **Probe layer:** `probe`, then `safe_z_home`, `z_tilt`, `quad_gantry_level`, `screws_tilt_adjust`, `smart_effector`, `skew_correction`, `tuning_tower`, `z_thermal_adjust`.
6. **MCU-trigger analytics:** `trigger_analog`, then `probe_eddy_current`, then `temperature_probe` (depends on the eddy probe).
7. **Offline numerics:** `shaper_calibrate`, then `resonance_tester` (depends on accelerometer chips from another slice).
8. **Hardware-specific and display modules last:** `display/st7920`, `display/uc1701`, `replicape`, `pwm_tool` (needs `motion_queuing`/C syncemitter), `palette2`, `tsl1401cl_filament_width_sensor`.
