# Map of klippy/extras, slice A (`__init__.py` through `extruder_stepper.py`)

## 0. Scope and method

- Directory listing was taken from `klippy/extras/` (plus subdirectory `display/`). Alphabetical (ASCII) order puts `_` before lowercase letters.
- **First file in range:** `klippy/extras/__init__.py`. **Last file in range:** `klippy/extras/extruder_stepper.py`. `__pycache__/` is excluded.
- The range holds 35 top-level `.py` files (8,867 lines) plus the whole `display/` package (12 files, 3,955 lines), 12,822 lines in total.
- No name excluded by the task (input_shaper, gcode_move, homing, force_move, manual_stepper, motion_report) sorts inside this range, because all of them start with `f` to `m`. They only appear as neighbours (section 5).
- Line counts are exact (taken from the read tool's end-of-file message).
- Depth of reading. Read in full: `ad5206`, `adc_scaled`, `adc_temperature`, `ads1x1x`, `adxl345`, `aht10`, `bed_screws`, `bed_tilt`, `bltouch`, `board_pins`, `bulk_sensor`, `bus`, `canbus_*`, `controller_fan`, `dac084S085`, `delayed_gcode`, `display_status`, `dotstar`, `ds18b20`, `duplicate_pin_override`, `endstop_phase`, `error_mcu`, `exclude_object`, `extruder_stepper`, `display/display.py`, `display/menu_keys.py`, `display/__init__.py`. Read through structure grep plus targeted ranges: `ads1220`, `ads131m0x`, `angle`, `axis_twist_compensation`, `bed_mesh` (all commands, config, profile and status code; interpolation bodies not read), `bme280`, `bmi160`, `buttons`, `delta_calibrate`, `display/menu.py`, `display/hd44780.py`, `display/st7920.py`, `display/uc1701.py`, `display/hd44780_spi.py`, `display/aip31068_spi.py`. Not read beyond a few lines: `display/font8x14.py`, `display/display.cfg`, `display/menu.cfg`.
- **Method limit:** no shell is available, so `git log -- <path>` could not be run. Fork status is inferred (section 6).
- The snapshot is a recent upstream (copyrights up to 2026: `bltouch.py` 2018-2026, `ads131m0x.py` 2026). It includes `i2c_transfer`/`i2c_set_sw_bus`/`spi_set_sw_bus`, `canbus_stats`, `mt6816`/`mt6826s` angle chips, bed_mesh `rapid_scan`, `bmi160`, `ads131m0x`. The port must target this snapshot, not older docs.

## 1. Families

| Family | Units |
|---|---|
| Package marker | `__init__.py` |
| Bus and MCU transport helpers (other modules import these) | `bus.py`, `bulk_sensor.py`, `buttons.py`, `board_pins.py`, `duplicate_pin_override.py`, `canbus_ids.py`, `canbus_stats.py`, `error_mcu.py` |
| Digipots and DACs | `ad5206.py`, `dac084S085.py` |
| ADC front ends and temperature sensors | `adc_temperature.py`, `adc_scaled.py`, `ads1x1x.py` (I2C ADC), `ads1220.py`, `ads131m0x.py` (SPI bulk ADCs for load cells), `ds18b20.py`, `aht10.py` (I2C), `bme280.py` (I2C: BME280/BME680/BMP180/BMP388) |
| Accelerometers and angle sensors | `adxl345.py`, `bmi160.py`, `angle.py` |
| Probes, bed leveling, calibration | `bltouch.py`, `bed_mesh.py`, `bed_tilt.py`, `bed_screws.py`, `axis_twist_compensation.py`, `delta_calibrate.py`, `endstop_phase.py` |
| Displays and UI | `display/` package, `display_status.py` |
| LEDs | `dotstar.py` |
| Fans | `controller_fan.py` |
| Macros and templates | `delayed_gcode.py`; Jinja consumers inside `display/` |
| Print-job control | `exclude_object.py` |
| Motion | `extruder_stepper.py` |
| TMC drivers | none in this range (`tmc*.py` sort after `e`). `angle.py` and `endstop_phase.py` look TMC drivers up by name. |
| Other I2C/SPI sensors (`sht3x`, `htu21d`, `lm75`, `lis2dw`, `mpu9250`, `ldc1612`, `mcp4*`, `pca9*`, `spi_temperature`, `hx71x`, `icm20948`) | not in range, but all are built on `bus.py`, `bulk_sensor.py`, `adxl345.py` |

### Numeric-library users (extra question)

- **Only `angle.py` uses numpy.** It imports it inside `AngleCalibration.__init__` (only when the `stepper` option is set; raises `config.error("Angle calibration requires numpy module")` if missing) and again inside `load_calibration`. It builds a `full_steps x 64` interpolation matrix (`numpy.zeros`) and solves it with `numpy.linalg.lstsq(eqs, ans, rcond=None)`, then rounds each solution with `int(s + .5)`. Parity risk: LAPACK `gelsd` minimum-norm least squares on a possibly rank-deficient system. The Rust replacement needs SVD-based minimum-norm lstsq and agreement after rounding.
- No scipy, cffi or greenlet imports in range. Pure-Python numerics: `bulk_sensor.ClockSyncRegression` (exponentially weighted regression), `bed_mesh` (Lagrange and cardinal bicubic), `bed_tilt` and `delta_calibrate` via `klippy/mathutil.py` (`solve_linear_equations`, `coordinate_descent`).

### Template and expression evaluators (extra question)

- Templates are Jinja2, created by `gcode_macro.load_template` (`klippy/extras/gcode_macro.py:83`: `jinja2.Environment('{%', '%}', '{', '}')`). Variable delimiters are single braces and the environment is not sandboxed, so user config can run arbitrary Python expressions. Users in range:
  - `delayed_gcode.py`: option `gcode` is a template, rendered each time the timer fires.
  - `display/display.py`: `[display_template X]` option `text`, and `[display_data group item]` option `text`, rendered with a context holding `printer`, `render()`, `draw_progress_bar()`. `param_*` options go through `ast.literal_eval`.
  - `display/menu.py`: per-item options `name`, `enable`, `gcode`, `input`, `input_min`, `input_max`, plus event scripts are templates. Rendered `enable`/`input*` text is fed to `ast.literal_eval`. `string.Template` is also used for item namespace ids (`$__id`).
  - `display/display.cfg` (461 lines) and `display/menu.cfg` (782 lines) are Jinja-heavy bundled configs, read through `configfile.read_config`.
- No `eval()`/`exec()` in range. `ast.literal_eval` is used only in `display/`.

## 2. Cross-module contracts used by many units

- `config.get*` family (`getfloat`, `getint`, `getboolean`, `getchoice`, `getlist`, `getfloatlist`, `getintlist`, `getlists`, `get_prefix_options`, `get_prefix_sections`, `getsection`, `has_section`, `note_valid=False`). Every unit depends on the config layer's validation messages.
- Pins: `printer.lookup_object('pins')` with `setup_pin('adc'|'pwm'|'endstop', ...)`, `lookup_pin`, `register_chip`, `get_pin_resolver(...).alias_pin/reserve_pin`, `allow_multi_use_pin`, `reset_pin_sharing`, `share_type`.
- MCU object: `create_oid`, `add_config_cmd(cmd, is_init=, on_restart=)`, `register_config_callback`, `lookup_command(fmt, cq=)`, `lookup_query_command`, `try_lookup_command`, `check_valid_response`, `register_serial_response(cb, fmt, oid)`, `alloc_command_queue`, `get_query_slot`, `seconds_to_clock`, `clock_to_print_time`, `print_time_to_clock`, `clock32_to_clock64`, `estimated_print_time`, `get_enumerations`, `get_constants`, `is_fileoutput`.
- Events: `klippy:connect`, `klippy:ready`, `klippy:shutdown`, `klippy:analyze_shutdown`, `klippy:notify_mcu_error`, `stepper:sync_mcu_position`, `homing:home_rails_end`, `probe:update_results`, `virtual_sdcard:reset_file`.
- Move-transform chain: `gcode_move.set_move_transform(obj, force=)` / `reset_last_position()` used by `bed_mesh`, `bed_tilt`, `exclude_object`.
- `configfile.set/remove_section` writes SAVE_CONFIG sections (bed_mesh, bed_tilt, axis_twist_compensation, delta_calibrate, angle, endstop_phase).
- Reactor: `register_timer`, `update_timer`, `register_callback`, `register_async_callback`, `pause`, `mutex()`, `NOW`/`NEVER`.
- Dynamic loading: every module exposes `load_config(config)` and/or `load_config_prefix(config)`; `printer.load_object(config, name)` imports `extras.<name>` by string. `__init__.py` is only the package marker.

## 3. Unit-by-unit

Format per unit: lines; deps; contracts; runtime traits; tests; fork; difficulty.

### 3.1 Package marker

#### `__init__.py` (5 lines)
- Deps: none. Contracts: none. Runtime: package marker only. Tests: implicit. Fork: none seen. Difficulty: **low** (a plugin registry in Rust replaces it).

### 3.2 Transport helpers

#### `bus.py` (370 lines)
- Deps: klippy `mcu` (`mcu.error`, `mcu.get_printer_mcu`), `pins`. Third-party: none.
- Python API: `resolve_bus_name`, `MCU_SPI`, `MCU_SPI_from_config`, `MCU_I2C`, `MCU_I2C_from_config`, `MCU_bus_digital_out`.
- Config options read: SPI `cs_pin` (or custom `pin_option`), `spi_speed` (min 100000), `spi_bus`, `spi_software_{miso,mosi,sclk}_pin`; I2C `i2c_mcu` (default `mcu`), `i2c_speed` (min 100000), `i2c_address` (0..127), `i2c_bus`, `i2c_software_{scl,sda}_pin`.
- MCU config commands (strings must match byte for byte): `config_spi oid=%d pin=%s cs_active_high=%d`, `config_spi_without_cs oid=%d`, `spi_set_bus oid=%d spi_bus=%s mode=%d rate=%d`, `spi_set_software_bus oid=%d miso_pin=%s mosi_pin=%s sclk_pin=%s mode=%d rate=%d`, `spi_set_sw_bus ... pulse_ticks=%d` (chosen when the MCU knows the command, else `configfile.deprecate_mcu_code`), `config_spi_shutdown oid=%d spi_oid=%d shutdown_msg=%s`, `spi_send oid=%d data=%s` (init-time hex form), `config_i2c oid=%d`, `i2c_set_bus oid=%d i2c_bus=%s rate=%d address=%d`, `i2c_set_software_bus`, `i2c_set_sw_bus`, `config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d`, `update_digital_out oid=%c value=%c`.
- Runtime commands: `spi_send oid=%c data=%*s`, `spi_transfer oid=%c data=%*s` answered by `spi_transfer_response oid=%c response=%*s`, `i2c_write oid=%c data=%*s`, `i2c_read oid=%c reg=%*s read_len=%u` answered by `i2c_read_response`, `i2c_transfer oid=%c write=%*s read_len=%u` answered by `i2c_response oid=%c i2c_bus_status=%c response=%*s` (status `SUCCESS`, otherwise shutdown with "MCU '%s' I2C request to addr %i reports error %s"). Both firmware dialects (`i2c_read` older, `i2c_transfer` newer) are handled. `send_with_preface`, `send_wait_ack`, `minclock`, `reqclock` semantics belong to `mcu.py`.
- CS pin `None` means no CS line (`rpi:None`, used by the KE printer for the `spidev` ADXL). Enumerations read: `spi_bus`/`i2c_bus`; constant `BUS_PINS_<bus>` reserves pins.
- Runtime: I2C writes issued before config build are queued in `_to_write` and flushed on `klippy:connect`; `async_write_only` mode registers an async error logger; command objects are chosen after `build_config`.
- Tests: indirect everywhere an SPI/I2C chip appears (`tmc.cfg` SPI drivers, `input_shaper.cfg` adxl345, `load_cell.cfg`, `linuxtest.cfg`).
- Fork: none seen in the file. The KE config `config/ender-3-v3-ke/input_shaper.cfg` uses `cs_pin: rpi:None` and `spi_bus: spidev2.0`, so the `None` pin and the `spidev` enumeration on the Linux host MCU are fork-relevant paths.
- Difficulty: **medium**. Mostly string formatting, but exact message formats and the two I2C dialects must be preserved.

#### `bulk_sensor.py` (299 lines)
- Deps: `threading`, `struct`, `logging`; klippy `webhooks`, mcu object.
- Classes: `BatchBulkHelper` (batch timer, client list, `add_client`, `add_mux_endpoint(path, key, value, start_resp)` registers a webhooks mux endpoint, start/stop callbacks), `BatchWebhooksClient` (reads `response_template` from the web request, sends `{..., 'params': msg}`), `BulkDataQueue` (registers MCU response `sensor_bulk_data oid=%c sequence=%hu data=%*s` into a lock-protected list), `ClockSyncRegression` (decay 1/20, mcu clock to chip clock regression), `FixedFreqReader` (queries `sensor_bulk_status oid=%c clock=%u query_ticks=%u next_sequence=%hu buffered=%u possible_overflows=%hu` through a sensor-specific query command; `MAX_BULK_MSG_SIZE = 51`; unpack formats such as `"BBBBB"`, `"<i"`, `"<hhh"`).
- Runtime: **threading**: `_handle_data` runs on the serial thread, `pull_queue` on the reactor thread; shared state guarded by one `threading.Lock`. Float-sensitive: regression gives sample timestamps (`ptime`, rounded to 6 decimals by callers); not sent to the MCU, but reported over webhooks and used by resonance and load-cell code. Batch interval 0.5 s default, 0.1 s for sensors.
- Tests: `load_cell.test` (ads1220, ads131m02, hx71x); `input_shaper.cfg` (config only).
- Fork: none seen. Difficulty: **medium** (regression parity plus a thread-safe queue).

#### `buttons.py` (345 lines)
- Deps: `logging`; pins, mcu, `query_adc`.
- `PrinterButtons` (section `[buttons]`, auto-loaded by other modules) exposes `register_buttons`, `register_button_push`, `register_adc_button`, `register_adc_button_push`, `register_debounce_button`, `register_debounce_adc_button`, `register_rotary_encoder` (half-step and full-step state machines). Option `debounce_delay` is read from the caller's config.
- MCU: `config_buttons oid=%d button_count=%d`, `buttons_add oid=%d pos=%d pin=%s pull_up=%d` (init), `buttons_query oid=%d clock=%d rest_ticks=%d retransmit_count=%d invert=%d` (init), `buttons_ack oid=%c count=%c`, response `buttons_state oid=%c ack_count=%c state=%*s`. Constants: `QUERY_TIME 0.002`, `RETRANSMIT_COUNT 50`, `ADC_REPORT_TIME 0.010`, `ADC_DEBOUNCE_TIME 0.025`, `ADC_SAMPLE_TIME 0.001`, `ADC_SAMPLE_COUNT 6`.
- Registers ADC pins in `query_adc` as `adc_button:<pin>`. Callbacks run through `reactor.register_async_callback`/`register_callback`. ADC ladder value is `pullup * adc / (1 - adc)`; the 8-bit ack counter is unwrapped.
- Tests: none directly; reached in `printers.test` by example configs with an active `[display]` using encoder or button pins.
- Fork: none seen. Difficulty: **medium** (state tables and ack-count wrap handling).

#### `board_pins.py` (27 lines)
- Config: `mcu` list (default `mcu`), `aliases`, any option prefixed `aliases_`; values of the form `<NAME>` call `reserve_pin`, others call `alias_pin`. Both `[board_pins]` and `[board_pins name]` load. Tests: `printers.test` (many example configs). Fork: none. Difficulty: **low**.

#### `duplicate_pin_override.py` (15 lines)
- Config `pins` list; calls `ppins.allow_multi_use_pin`. Tests: none. Fork: none. Difficulty: **low**.

#### `canbus_ids.py` (26 lines)
- Registry from `canbus_uuid` to node id (first id 4, assigned in registration order, so ids depend on config load order). `add_uuid` raises "Duplicate canbus_uuid"; `get_nodeid` raises config_error "Unknown canbus_uuid %s". The caller is the MCU setup code (not read). Tests: none. Fork: none. Difficulty: **low**.

#### `canbus_stats.py` (83 lines)
- Section `[canbus_stats <mcu>]`. MCU command `get_canbus_status` answered by `canbus_status rx_error=%u tx_error=%u tx_retries=%u canbus_bus_state=%u`; optional response `usb_canbus_state discard=%u` (logs warnings "USB CANBUS bridge '%s' is discarding!" / "... is no longer discarding."). Query timer every 1 s, 32-bit counters unwrapped. Uses `configfile.deprecate_mcu_code` if firmware lacks the command.
- `get_status`: `rx_error`, `tx_error`, `tx_retries`, `bus_state` (set to `'unknown'` after shutdown). `stats()` returns the Stats-line fragment `canstat_<name>: bus_state=%s rx_error=%d tx_error=%d tx_retries=%d` (parsed by log tooling).
- Tests: none. Fork: none seen. Difficulty: **low**.

#### `error_mcu.py` (136 lines)
- Hooks `klippy:analyze_shutdown` and `klippy:notify_mcu_error`; exposes `add_clarify(msg, callback)` (used by `adc_temperature` for "ADC out of range"). Rewrites `printer.update_error_msg` with fixed English hint blocks (for "Timer too close", "Missed scheduling of next ", "ADC out of range", "Rescheduled timer in the past", "Stepper too far in past", "Command request"). Prefixes `MCU '%s' shutdown: ` and `Previous MCU '%s' shutdown: ` depend on `details['event_type'] == 'is_shutdown'`. The protocol-error text compares each MCU's `mcu_version` with `start_args['software_version']`. Logs `Reactor garbage collection: %s` from `reactor.get_gc_stats()`.
- These strings reach the UI error banner and are matched by clients; keep them byte-exact.
- Fork-adjacent: the details dict (`mcu`, `reason`, `event_type`, `shutdown_clock`) is built by `klippy/mcu.py`, which carries the fork's shutdown-provenance work (`7777d10`). `error_mcu.py` itself looks stock. `klippy/klippy.py` registers a second `klippy:analyze_shutdown` handler (`_debug_shutdown`), so handler order is observable in logs.
- Tests: loaded transitively in `temperature.test` through `adc_temperature`; no direct assertions. Difficulty: **low**.

### 3.3 Digipots and DACs

#### `ad5206.py` (22 lines)
- Section `[ad5206 name]`. SPI via `bus.MCU_SPI_from_config(mode 0, pin_option enable_pin, 25 MHz)`. Options `scale` (above 0), `channel_1..6` (0..scale). Sends `[reg, int(val*256/scale + .5)]` once at config time (init-time `spi_send`). Tests: `printers.test` (`generic-rambo.cfg`). Fork: none. Difficulty: **low**.

#### `dac084S085.py` (25 lines)
- Section `[dac084S085 name]`, SPI mode 1, `enable_pin`, 10 MHz. Options `scale`, `channel_A..D`. Bytes: `b1 = (chan<<6)|(1<<4)|((value>>4)&0xf)`, `b2 = (value<<4)&0xf0`, with value `int(val*255/scale)` (truncation, not rounding). Tests: `printers.test` (`generic-alligator-r2/r3`). Fork: none. Difficulty: **low**.

### 3.4 ADC and temperature sensors

#### `adc_temperature.py` (337 lines)
- Deps: `bisect`, `logging`; `heaters` (`add_sensor_factory`), `query_adc`, `error_mcu`, pins.
- Classes: `PrinterADCtoTemperature` (option `sensor_pin`; `setup_pin('adc')`; `setup_adc_sample(0.300, 0.001, 8, minval, maxval, range_check_count=4)`; callback time offset `8 * 0.001`), `HelperTemperatureDiagnostics` (registers the ADC in `query_adc` under the section name; adds clarify text "Sensor '%s' temperature %s not in range %.3f:%.3f"), `LinearInterpolate`, `LinearVoltage`, `LinearResistance`, `CustomLinearVoltage`, `CustomLinearResistance`.
- Registered sensor types: `AD595`, `AD597`, `AD8494`, `AD8495`, `AD8496`, `AD8497`, `PT100 INA826` (voltage tables; `calc_pt100` Callendar-Van Dusen A=3.9083e-3, B=-5.775e-7; INA826 circuit 4400 ohm pullup, gain 10, 5 V), `PT1000` (resistance, pullup default 4700).
- `[adc_temperature name]` with `temperatureN`/`voltageN` pairs (N up to 999), or `resistanceN` (selects resistance mode); options `adc_voltage` (default 5), `voltage_offset`, `pullup_resistor`. `thermistor.py` builds on `PrinterADCtoTemperature`.
- Runtime: float math; `setup_minmax` converts min/max temperature to ADC limits that become integer MCU limits (in `mcu.py`), so interpolation rounding reaches the MCU. Lambda factories capture `params=params`.
- **Latent bug to decide on:** `_clarify_adc_range` has `except e:` (NameError if `calc_temp` raises).
- Tests: `temperature.test` + `temperature.cfg` (AD595, PT100 INA826, PT1000, custom `adc_temperature`, custom resistance). Fork: none seen. Difficulty: **medium** (interpolation and reverse interpolation must match because ADC limits reach the MCU).
- The KE printer reaches it through its `EPCOS 100K B57560G104F` thermistors (`thermistor.py` sits on top).

#### `adc_scaled.py` (80 lines)
- Section `[adc_scaled name]`. Options `vref_pin`, `vssa_pin`, `smooth_time` (default 2, above 0). Registers itself as a pin chip; wrapped ADC pins return `(read - vssa)/(vref - vssa)`. Exponential smoothing in `calc_smooth`. Requires both pins on the same MCU ("vref and vssa must be on same mcu"); registers `name:vref`, `name:vssa` in `query_adc`. Rebinds attributes (`self.setup_adc_sample = self._mcu_adc.setup_adc_sample`). Tests: `printers.test` (duet2-maestro, duet3 configs). Fork: none. Difficulty: **low**.

#### `ads1x1x.py` (398 lines)
- Section `[ads1x1x name]`; I2C ADC chips ADS1013/4/5, ADS1113/4/5. Options: `chip` (choice), `address_pin` (`GND|VCC|SDA|SCL`), `pga` (default `4.096V`), `adc_voltage` (default 3.3), plus `i2c_*`. Registers itself as a pin chip that provides virtual ADC pins `DIFF01, DIFF03, DIFF13, DIFF23, AIN0..AIN3`. One polling timer per pin; single-shot conversions; sample rate chosen from the shortest report time; the reading is rescaled with `pga / adc_voltage`. Calls `printer.invoke_shutdown("ADS1X1X temperature check failed")` after `range_check_count` invalid reads. Sends `i2c_write([0x06, 0, 0])` (general-call reset) on connect. Registers object `ads1x1x <name>`.
- Runtime: reactor `mutex()` and `reactor.pause` loops inside timer greenlets. Tests: none. Fork: none. Difficulty: **medium**.

#### `ads1220.py` (229 lines) and `ads131m0x.py` (409 lines)
- Deps: `bulk_sensor`, `bus`. They are not standalone sections. `load_cell.py` and `load_cell_probe.py` use them through `ADS1220_SENSOR_TYPE` and `ADS131M0X_SENSOR_TYPES` (`ads131m02`, `ads131m04`); `hx71x` follows the same interface (`add_client`, `get_range`, `get_samples_per_second`, `get_status`, `lookup_sensor_error`, `setup_trigger_analog`, `get_mcu`).
- Config (ads1220): `gain` (1..128, default 128), `sample_rate` (normal 20..1000 plus turbo rates; default `'660'`), `input_mux`, `pga_bypass`, `vref`, `data_ready_pin` (same MCU as SPI), SPI via `bus`. Config (ads131m0x): `adc_channel`, `pwm_clock` (reads `frequency` from that PWM section) or `clock_freq` (300 kHz..8.4 MHz), `gain`, `enable_global_chop`, `global_chop_delay`, `sample_rate` (nearest OSR, error if more than 50% off), SPI mode 1 at 4 MHz default.
- MCU: `config_ads1220 oid spi_oid data_ready_pin`, `query_ads1220 oid rest_ticks` (init and `on_restart`), `query_ads1220_status`, `ads1220_attach_trigger_analog oid trigger_analog_oid` (init). Same shape for `ads131m0x` plus `channel`/`num_channels`, and enumeration `ads131m0x_error:` for error text. Sample format `<i`; output tuple `(round(ptime,6), raw, round(raw/2^23, 9))`.
- `get_status`: `errors`, `overflows`, `sample_rate`.
- Runtime: bulk batch every 0.1 s; register writes are verified by read-back and raise `command_error`. Quirk: `ADS1220.lookup_sensor_error` does `"Unknown ads1220 error" % (error_code,)`, which raises `TypeError`.
- Tests: `load_cell.test` with `load_cell.cfg` (`my_ads1220`, `my_ads131m02`, `load_cell_probe`). Fork: no code change seen; commit `9b3499b` ("i'm not making the load cell work it's just annoying") shows the load cell was abandoned on the KE (the include in `printer.cfg` is commented out). Difficulty: **medium** each.

#### `ds18b20.py` (81 lines)
- Sensor type `DS18B20` registered with `heaters` (used by `temperature_sensor` and heater sections). Options `serial_no`, `sensor_mcu`, `ds18_report_time` (default 3, min 1). MCU: `config_ds18b20 oid=%d serial=%s max_error_count=%d`, `query_ds18b20 oid=%d clock=%u rest_ticks=%u min_value=%d max_value=%d` (init; values are `temp*1000` formatted with `%d`, which truncates toward zero), response `ds18b20_result oid=%c next_clock=%u value=%i fault=%u`. `get_status` returns `temperature` from `self.temp`, which is never updated (always 0.0; the real value goes to the heater callback). Tests: `linuxtest.cfg` (`my_ds18b20`). Fork: none. Difficulty: **low**.

#### `aht10.py` (203 lines)
- Sensor types `AHT10`, `AHT1X`, `AHT2X`, `AHT3X`. Options `aht10_report_time` (default 30, min 5) plus I2C options (address 0x38). Busy loop (max 5 cycles, then soft reset), conversion `temp = raw*200/2^20 - 50`, `humidity = int(raw*100/2^20)` clamped to 0..100; `reactor.pause(0.110)` per attempt. `get_status`: `temperature`, `humidity`. Registers object `aht10 <name>`. Shutdown message `"%s temperature %.1f outside range of %.1f:%.1f"`.
- Runtime: `reactor.pause` inside timer callbacks (greenlet), `invoke_shutdown`. Tests: none. Fork: none. Difficulty: **low**.

#### `bme280.py` (796 lines)
- Sensor type `BME280`; handles chips BME280, BME680, BMP180, BMP388 (chosen from the chip id read at connect). Options `bme280_iir_filter`, `bme280_oversample_temp/hum/pressure`, `bme280_gas_target_temp` (320), `bme280_gas_heat_duration` (150), plus I2C (address 0x76). Per-chip integer and float compensation formulas, calibration reads, gas heater control, `reactor.pause` loops. `get_status`: `temperature`, `pressure`, and `humidity` (BME280, BME680), `gas` (BME680). Shutdown messages name the chip. Does not register the connect handler when `start_args['debugoutput']` is set. Registers object `bme280 <name>`.
- Tests: none. Fork: none. Difficulty: **medium** (four chip variants with many hardware compensation formulas).

### 3.5 Accelerometers and angle sensors

#### `adxl345.py` (317 lines)
- Deps: `bulk_sensor`, `bus`, `multiprocessing`, `os`, `time`, `collections`. Reused by `bmi160`, `lis2dw`, `lis3dh`, `mpu9250`, `icm20948`, `resonance_tester` through `AccelCommandHelper`, `AccelQueryHelper`, `read_axes_map`, `Accel_Measurement`.
- G-code (mux on `CHIP`, plus an unnamed default when the section name has one word): `ACCELEROMETER_MEASURE [CHIP=] [NAME=]`, `ACCELEROMETER_QUERY [CHIP=]`, `ACCELEROMETER_DEBUG_READ REG=`, `ACCELEROMETER_DEBUG_WRITE REG= VAL=`. Output file `/tmp/<base>-<name>.csv` or `/tmp/<base>-<chip>-<name>.csv`, header `#time,accel_x,accel_y,accel_z`, `%.6f` columns. Messages: "accelerometer measurements started", "Writing raw accelerometer data to %s file", "accelerometer values (x, y, z): %.6f, %.6f, %.6f".
- Config: `axes_map` (default `x,y,z`, entries `[-]x|y|z`), `rate` (25..3200, default 3200), SPI (mode 3, 5 MHz default). Webhooks mux endpoint `adxl345/dump_adxl345` (key `sensor`, header `time,x_acceleration,y_acceleration,z_acceleration`). Error text "Invalid adxl345 id (got %x vs %x)".
- MCU: `config_adxl345 oid spi_oid`, `query_adxl345 oid rest_ticks`, `query_adxl345_status`; sample format `BBBBB`; scales `SCALE_XY = 0.003774*9806.65`, `SCALE_Z = 0.003906*9806.65`; registers 0x00, 0x2C, 0x2D, 0x31, 0x38.
- Runtime: `multiprocessing.Process` (daemon, `os.nice(20)`) writes the CSV (fork based; in Rust use a thread or process); toolhead calls (`get_last_move_time`, `wait_moves`, `dwell`); client memory cap of 10,000 messages.
- Tests: `input_shaper.cfg` loads `[adxl345]` (config parse only; `input_shaper.test` runs `SET_INPUT_SHAPER` only). No test runs ACCELEROMETER_* commands.
- Fork: none in the file. The KE printer uses it (`config/ender-3-v3-ke/input_shaper.cfg`: SPI on the `rpi` host MCU, `spidev2.0`, `cs_pin: rpi:None`, 2 MHz) together with `resonance_tester`.
- Difficulty: **medium**.

#### `bmi160.py` (188 lines)
- Same helper API as adxl345. SPI (when `cs_pin` is set, 1 MHz mode 0) or I2C (address 0x69). Fixed rate 1600 Hz, range plus/minus 16 g (`SCALE = 9806.65/2048`), sample format `<hhh`, `axes_map`. MCU: `config_bmi160 oid bus_oid bus_oid_type`, `query_bmi160`, `query_bmi160_status`. Webhooks endpoint `bmi160/dump_bmi160`. Tests: none. Fork: none. Difficulty: **medium**.

#### `angle.py` (754 lines)
- Section `[angle name]`. Deps: `bus`, `bulk_sensor`, `math`, **numpy** (section 1), `force_move` (`lookup_stepper`, `manual_move`), TMC driver modules by name, `configfile.get_status()['settings']`, `toolhead`.
- Config: `sensor_type` (`a1333`, `as5047d`, `tle5012b`, `mt6816`, `mt6826s`), `sample_period` (default 0.0004), `stepper` (enables calibration), `calibrate` (comma list of angles, 8 per line), SPI mode/speed per chip. G-code: `ANGLE_CALIBRATE CHIP=`; `ANGLE_DEBUG_READ`/`ANGLE_DEBUG_WRITE` (tle5012b), `ANGLE_DEBUG_READ` (mt6816, mt6826s), `ANGLE_CHIP_CALIBRATE` (mt6826s). Webhooks endpoint `angle/dump_angle`. `get_status`: `temperature`. Bulk batch dict: `data`, `errors`, `position_offset`.
- MCU: `config_spi_angle oid spi_oid spi_angle_type`, `query_spi_angle oid clock rest_ticks time_shift`, `spi_angle_transfer oid data` answered by `spi_angle_transfer_response oid clock response`. Sample is 3 bytes (`tcode`, 16-bit angle); `TCODE_ERROR = 0xff`; `time_shift` grows until `(0xff << shift)/freq >= 0.002`; tle5012b uses an absolute chip frame counter (`update_clock`, chip frequency tracking).
- Runtime: calibration uses 64-bucket linear interpolation tables (integer math) and the numpy least-squares solve; writes `[angle name] calibrate` through `configfile.set`; listens to `stepper:sync_mcu_position`; long G-code that issues `manual_move` in a loop.
- Tests: none. Fork: none. Difficulty: **high** (numpy lstsq parity, five chip helpers, chip-clock tracking, stepper-phase coupling).

### 3.6 Probes, bed leveling, calibration

#### `bltouch.py` (306 lines)
- Deps: `probe` (`ProbeOffsetsHelper`, `ProbeParameterHelper`, `DescendToEndstopHelper`, `SampleAveragingHelper`, `ProbeCommandHelper`, `HomingViaProbeHelper`), pins (a `pwm` pin for control, an `endstop` pin for the sensor), toolhead.
- Config: `sensor_pin`, `control_pin`, `stow_on_each_sample` (True), `probe_with_touch_mode` (False), `set_output_mode` (`5V|OD|None`), `pin_up_reports_not_triggered` (True), `pin_up_touch_mode_reports_triggered` (True), `pin_move_time` (0.680), plus all `probe.*` options (`x_offset`, `y_offset`, `z_offset`, `speed`, `lift_speed`, `samples`, ...).
- G-code: `BLTOUCH_DEBUG COMMAND=pin_down|touch_mode|pin_up|self_test|reset|set_5V_output_mode|set_OD_output_mode|output_mode_store`, `BLTOUCH_STORE MODE=5V|OD`. Registers itself as object `probe` (`add_object('probe', blt)`); status comes from `probe.ProbeCommandHelper`.
- Pulse table `Commands` (seconds): pin_down 0.000650, touch_mode 0.001165, pin_up 0.001475, self_test 0.001780, reset 0.002190, set_5V_output_mode 0.001988, set_OD_output_mode 0.002091, output_mode_store 0.001884; signal period 0.020 s. PWM duty is `Commands[cmd]/0.020`, followed by `set_pwm(end_time, 0)`.
- Runtime: float timing (`int((duration - MIN_CMD_TIME)/SIGNAL_PERIOD)*SIGNAL_PERIOD`, clock rounding via `print_time_to_clock`), `mcu_pwm.setup_max_duration(0.)`, endstop `home_start` with sample time 15 us, count 4, rest 1 ms; reactor callback plus `finish_home_complete.wait()`; up to 3 retries on failed verification; `next_cmd_time` synchronised with `toolhead.get_last_move_time` and `dwell`.
- Tests: `bltouch.test` (`BLTOUCH_DEBUG`, `BLTOUCH_DEBUG COMMAND=reset`, `BED_MESH_CALIBRATE`), `screws_tilt_adjust.test`.
- Fork: none seen in the file, **but this is the KE printer's probe** (`config/ender-3-v3-ke/printer.cfg`: `[bltouch]` with `probe_with_touch_mode: true`, `stow_on_each_sample: false`, `speed: 700`, `lift_speed: 30`, `y_offset: 27`).
- Difficulty: **high** (exact PWM tick timing and probe session protocol; any drift changes MCU command timestamps).

#### `bed_mesh.py` (1770 lines)
- Deps: `probe` (`ProbePointsHelper`), `gcode_move` (transform), `toolhead`, `configfile`, `webhooks`, `exclude_object` (adaptive mesh), `json`, `math`, `collections`. No numpy.
- G-code: `BED_MESH_CALIBRATE` (params `PROFILE`, `METHOD=automatic|scan|rapid_scan`, `ADAPTIVE`, `ADAPTIVE_MARGIN`, `MESH_MIN`, `MESH_MAX`, `PROBE_COUNT`, `MESH_PPS`, `ALGORITHM`, `MESH_RADIUS`, `MESH_ORIGIN`, `ROUND_PROBE_COUNT`, `SCAN_SPEED`, `HORIZONTAL_MOVE_Z`), `BED_MESH_PROFILE LOAD|SAVE|REMOVE=<name>` (`default` is reserved for SAVE), `BED_MESH_OUTPUT [PGP=1]`, `BED_MESH_MAP` (prints `mesh_map_output {json}` through `respond_raw`), `BED_MESH_CLEAR`, `BED_MESH_OFFSET [X= Y= ZFADE=]`.
- Config `[bed_mesh]`: `speed`, `horizontal_move_z` (5), `fade_start` (1), `fade_end` (0), `fade_target`, `split_delta_z` (0.025), `move_check_distance` (5), `adaptive_margin`, `mesh_radius`, `mesh_origin`, `round_probe_count` (5, odd), `probe_count` (pair, min 3), `mesh_min`, `mesh_max`, `mesh_pps` (pair, default 2,2), `algorithm` (`lagrange|bicubic`), `bicubic_tension` (0.2), `scan_overshoot`, `zero_reference_position`, `faulty_region_N_min/max`. Stored profile sections `[bed_mesh <name>]` with `version=1`, `points` (rows of `%.6f` values separated by `, `, one row per line indented two spaces) and mesh params (`min_x`, `max_x`, `min_y`, `max_y`, `x_count`, `y_count`, `mesh_x_pps`, `mesh_y_pps`, `algo`, `tension`). Incompatible profile versions are queued for removal with a console message.
- `get_status`: `profile_name`, `mesh_min`, `mesh_max`, `probed_matrix`, `mesh_matrix`, `profiles`. Webhooks: `bed_mesh/dump_mesh` (optional `mesh_args` object), `get_mesh`. Console format "bed_mesh: generated points\nIndex |  Tool Adjusted  |   Probe".
- Runtime: acts as a **move transform** (`get_position`, `move`) with fade and `MoveSplitter` (splits long moves by `split_delta_z`); rounds to 6 decimals for output and to 2 decimals for `bed_x`/`bed_y` of probe results; Lagrange and cardinal spline interpolation in plain floats. Mesh Z offsets reach the MCU through toolhead moves, so operation order matters. `rapid_scan` needs a probe whose status name starts with `probe_eddy_current`. Generated mesh names use `id(self)` (`adaptive-%X`).
- Tests: `bed_mesh.test` (`BED_MESH_CALIBRATE`), `bltouch.test`, `eddy.test` (`METHOD=scan`, `METHOD=rapid_scan`), `load_cell.test`, `smart_effector.test`, `z_virtual_endstop.test`; configs in `screws_tilt_adjust.cfg`. No test covers `BED_MESH_PROFILE`, `BED_MESH_OFFSET`, `BED_MESH_MAP`, faulty regions, radius meshes or the webhooks.
- Fork: none seen in the file. The KE printer uses it with `ADAPTIVE=1`, fade 1..10, `probe_count: 5,5` and a `G29` macro (`config/ender-3-v3-ke/gcode_macro.cfg`).
- Difficulty: **high** (1770 lines, transform chain, operation-order sensitivity, profile persistence, adaptive mesh polygons).

#### `bed_tilt.py` (87 lines)
- Deps: `mathutil.solve_linear_equations`, `probe.ProbePointsHelper`. Config `x_adjust`, `y_adjust`, `z_adjust`, `points` (enables calibration, min 3 points). G-code `BED_TILT_CALIBRATE`. Transform: `z += x*x_adjust + y*y_adjust + z_adjust`. Writes `[bed_tilt]` x/y/z_adjust (`%.6f`) through `configfile.set`; calls `set_rollover_info("bed_tilt", ...)`. Tests: `multi_z.test` (`BED_TILT_CALIBRATE`), `z_tilt.cfg`. Fork: none. Difficulty: **low**.

#### `bed_screws.py` (123 lines)
- Config `screwN` (up to 99, `x,y`), `screwN_name`, `screwN_fine_adjust`, `speed`, `probe_speed`, `horizontal_move_z`, `probe_height`; at least 3 screws. G-code `BED_SCREWS_ADJUST`; **dynamically registers and unregisters** `ACCEPT`, `ADJUSTED`, `ABORT` (`gcode.register_command(name, None)` removes a command). `get_status`: `is_active`, `state`, `current_screw`, `accepted_screws`. Tests: `bed_screws.test` (state machine incl. ABORT, ADJUSTED). Fork: none. Difficulty: **low** (the G-code registry must support unregister).

#### `axis_twist_compensation.py` (361 lines)
- Deps: `manual_probe` (`ProbeResult`, `verify_no_manual_probe`, manual probing), `bed_mesh` (`constrain`, `lerp`), `probe`. Config: `horizontal_move_z`, `speed`, `calibrate_start_x`, `calibrate_end_x`, `calibrate_y`, `z_compensations` (list), `compensation_start_x/end_x`, `calibrate_start_y`, `calibrate_end_y`, `calibrate_x`, `zy_compensations`, `compensation_start_y/end_y`. G-code `AXIS_TWIST_COMPENSATION_CALIBRATE [AXIS=X|Y] [SAMPLE_COUNT=3]`. Subscribes to `probe:update_results` and **mutates the probe result list in place** (adds interpolated Z compensation as a new `ProbeResult`). Saves `z_compensations` (`{:.6f}` joined by `, `), `compensation_start_x`, `compensation_end_x` (and the y variants) to the config. Requires `[probe]` at connect.
- Tests: none. Fork: none seen in the file; the KE printer uses it (X axis, saved offsets in `printer.cfg`). Difficulty: **medium** (event contract with `probe.py`, manual-probe interaction).

#### `delta_calibrate.py` (287 lines)
- Deps: `mathutil.background_coordinate_descent` (uses `multiprocessing.Pipe` and a child process), `probe.ProbePointsHelper`, delta kinematics (`kin.get_calibration()`, `calc_stable_position`, `new_calibration`, `coordinate_descent_params`, `get_position_from_stable`).
- Config `radius`, `heightN`/`heightN_pos`, `manual_heightN`/`..._pos`, `distanceN`/`distanceN_pos1/2`. G-code `DELTA_CALIBRATE [METHOD=manual]`, `DELTA_ANALYZE` (`MANUAL_HEIGHT`, `CENTER_DISTS`, `OUTER_DISTS`, `CENTER_PILLAR_WIDTHS`, `OUTER_PILLAR_WIDTHS`, `SCALE`, `CALIBRATE=extended`). Saves delta params and stable positions through `configfile`.
- Runtime: iterative float optimiser in a child process (the error closure is shared by fork), `MEASURE_WEIGHT 0.5`; results are saved with fixed formats, so the exact floating-point operation sequence decides the saved numbers. Raises "Delta calibrate is only for delta printers" at connect.
- Tests: `delta_calibrate.test`, `delta.test`, `rotary_delta_calibrate.test`, and example delta configs in `printers.test`. Fork: none. Difficulty: **medium** (depends on the kinematics slice and `mathutil`).

#### `endstop_phase.py` (231 lines)
- Two entry points. `[endstop_phase]` (class `EndstopPhases`): stats, `ENDSTOP_PHASE_CALIBRATE [STEPPER=]`, `get_status` -> `last_home: {name: {phase, phases, mcu_position}}`. `[endstop_phase <stepper>]` (class `EndstopPhase`): options `trigger_phase` (`p/ps`), `endstop_align_zero`, `endstop_accuracy`. Reads the stepper section (`stepper.parse_step_distance`, `microsteps`) and TMC drivers (`get_phase_offset()`). Handles `homing:home_rails_end` and calls `homing_state.set_stepper_adjustment`. Saves `trigger_phase` as `<n>/<phases>`.
- Tests: `tmc.cfg` contains `[endstop_phase]` (no `ENDSTOP_PHASE_CALIBRATE` in tests). Fork: none. Difficulty: **medium**.

### 3.7 Displays

#### `display/__init__.py` (21 lines)
- `load_config` and `load_config_prefix` for auxiliary `[display name]` sections (error if no primary `[display]`; error for `[display display]`). Tests: `printers.test` indirectly. Fork: none. Difficulty: **low**.

#### `display/display.py` (274 lines)
- Section `[display]`. Options `lcd_type` (choices `st7920`, `emulated_st7920`, `hd44780`, `uc1701`, `ssd1306`, `sh1106`, `hd44780_spi`, `aip31068_spi`), `display_group` (default `_default_16x4`, or `_default_20x4` when the chip is 20 columns wide). Loads templates, groups and glyphs from the bundled `display.cfg` plus user sections (`display_template`, `display_data`, `display_glyph`). G-code `SET_DISPLAY_GROUP [DISPLAY=] GROUP=`. Redraw timer 0.5 s (min 0.1 s). Exports `lookup_display_templates` (registers object `display_template`), used by `led.py` (`SET_LED_TEMPLATE`) and `output_pin.py`. Progress bar math: `-1 << int(width*8*(1-value)+.5)`.
- Glyph config: `data` (16x16 art with `.` and `*`), `hd44780_data` + `hd44780_slot` (5x8).
- Runtime: **Jinja2** rendering; `ast.literal_eval` for `param_*`; the context is cleared after each draw to break reference cycles; text goes through `text.encode()`.
- Tests: `led.cfg` defines `[display_template dtest]` (`led.test` uses `SET_LED_TEMPLATE`); `printers.test` loads active `[display]` sections of many example configs (config parse and chip construction only). Fork: none seen. Difficulty: **high** (Jinja2 semantics and the template context).

#### `display/menu.py` (1067 lines), `display/menu.cfg` (782 lines), `display/menu_keys.py` (108 lines)
- Menu tree (`list`, `vsdlist`, `command`, `input`, `disabled` item types) built from `[menu path]` sections: options `type`, `name`, `index`, `enable`, `gcode`, `input`, `input_min`, `input_max`, `input_step`, `realtime`, plus event scripts. Display options: `menu_root` (`__main`), `menu_timeout`, `menu_reverse_navigation`. Registers object `menu` (status: `timeout`, `running`, `rows`, `cols`). Context exposes `menu.eventtime`, `menu.back()`, `menu.exit()`, `menu.input`. G-code scripts run through a queue and `gcode.run_script` in a reactor callback.
- `menu_keys.py`: options `encoder_pins`, `encoder_steps_per_detent` (2 or 4), `encoder_fast_rate` (0.030), `click_pin`, `back_pin`, `up_pin`, `down_pin`, `kill_pin`, `analog_range_<pin option>`, `analog_pullup_resistor`; long press 0.8 s; `kill_pin` calls `printer.invoke_shutdown("Shutdown due to kill button!")`.
- Runtime: Jinja2 everywhere, `ast.literal_eval` on rendered text, Python 2 leftovers (`unicode`, `aslatin`). `menu.cfg` guards items with `virtual_sdcard` expressions (for example `enable: {('virtual_sdcard' in printer) and printer.virtual_sdcard.file_path and not printer.virtual_sdcard.is_active}`).
- Tests: the menu loads whenever the primary `[display]` exists, so every `printers.test` config with an active display compiles `menu.cfg` templates at config time. Fork: none. Difficulty: **high** (largest UI state machine plus Jinja templates).

#### Low-level LCD drivers and data
- `hd44780.py` (134): 4-bit parallel via MCU, pins `rs_pin e_pin d4_pin d5_pin d6_pin d7_pin` (same MCU), options `hd44780_protocol_init` (True), `line_length` (16|20). MCU: `config_hd44780 oid rs_pin e_pin d4..d7 delay_ticks`, `hd44780_send_cmds oid cmds`, `hd44780_send_data oid data`. Uses `BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000` as `reqclock`. Difficulty **medium**.
- `hd44780_spi.py` (125) and `aip31068_spi.py` (209): over SPI shift-register backpacks (`latch_pin`, `line_length`, `hd44780_protocol_init` for the first); nibbles are sent through `spi_send`. Difficulty **medium**.
- `st7920.py` (257): `ST7920` (pins `cs_pin sclk_pin sid_pin`; MCU `config_st7920 oid cs_pin sclk_pin sid_pin sync_delay_ticks cmd_delay_ticks`, `st7920_send_cmds`, `st7920_send_data`) and `EmulatedST7920` (software SPI pins `spi_software_{miso,mosi,sclk}_pin`, `spi_speed`, an `EnableHelper` for an enable pin). Difficulty **medium**.
- `uc1701.py` (241): `UC1701` (options `a0_pin`, `rst_pin`, `contrast`), `SSD1306` and `SH1106` (SPI 4-wire or I2C address 60 via `i2c_write_noack`). Difficulty **medium**.
- `font8x14.py` (276): static `VGA_FONT` table. Difficulty **low**.
- `display.cfg` (461): default `_default_16x4`, `_multiextruder_16x4`, `_default_20x4` display data groups and templates (`_heater_temperature`, `_fan_speed`, `_printing_time`, `_print_status`). Reads `printer.display_status`, `printer.toolhead`, `printer.fan`, `printer.idle_timeout` and others; text uses glyph markers like `~feedrate~` and `~clock~` between tildes.
- Tests for all of the above: config load only, via `printers.test`. Fork: none seen. (Fork commit `3a63ec4`, "support stock c440x display endpoints", does not touch this package; c440x is the stock Creality UI client of the webhooks socket, see section 6.)

#### `display_status.py` (50 lines)
- G-code `M73 P=` (progress 0..100, held 5 s then falls back to `virtual_sdcard` progress), `M117 <msg>` (raw parameters), `SET_DISPLAY_TEXT [MSG=]`. `get_status`: `progress`, `message`. Tests: `sdcard_loop.cfg` has `[display_status]`; no test sends M73 or M117. Fork: none. Difficulty: **low**.

### 3.8 LED, fan, macro, job control, motion

#### `dotstar.py` (56 lines)
- Software SPI (500 kHz, `data_pin`, `clock_pin` on the same MCU, `bus.MCU_SPI` with `sw_pins`), option `chain_count`. Uses `led.LEDHelper` (SET_LED, SET_LED_TEMPLATE, `get_status` with `color_data`) from `led.py` (out of range). Frame: 4 zero bytes, then per LED `0xff, B, G, R` with `int(x*255 + .5)`, then 4 bytes of `0xff`; sent in 20-byte chunks with `reqclock = BACKGROUND_PRIORITY_CLOCK`. Tests: `led.test` (`SET_LED LED=dled`, `SET_LED_TEMPLATE`). Fork: none. Difficulty: **low**.

#### `controller_fan.py` (69 lines)
- Section `[controller_fan name]`. Options `stepper` (list, default all), `heater` (default `extruder`), `fan_speed`, `idle_speed`, `idle_timeout` (30), plus all `fan.Fan` options (`pin`, `max_power`, `kick_start_time`, ...). Timer at 1 Hz; `get_status` is `fan.get_status`. Depends on `fan.py` (out of range), `stepper_enable`, `heaters`. Tests: `temperature.cfg` (`test_controller_fan`). Fork: none. Difficulty: **low**.

#### `delayed_gcode.py` (54 lines)
- Section `[delayed_gcode name]`. Options `gcode` (Jinja template), `initial_duration`. G-code `UPDATE_DELAYED_GCODE ID= DURATION=`. Reactor timer is created at `klippy:ready`; re-arm rules apply when updated inside the running callback (`inside_timer`, `repeat`). Script errors are logged ("Script running error"), not raised. Tests: none. Fork: none in code; the KE printer defines `[delayed_gcode fan_off]`. Difficulty: **low** (needs the template engine).

#### `exclude_object.py` (304 lines)
- Deps: `gcode_move` transform with `force=True`, `tuning_tower.is_active()`, `json`, events `klippy:connect`, `virtual_sdcard:reset_file`.
- G-code: `EXCLUDE_OBJECT_START NAME=`, `EXCLUDE_OBJECT_END [NAME=]`, `EXCLUDE_OBJECT [NAME= | RESET= | CURRENT=]`, `EXCLUDE_OBJECT_DEFINE [NAME= CENTER= POLYGON= RESET= JSON=]` plus free parameters. `get_status`: `objects`, `excluded_objects`, `current_object`. Object names are upper-cased; `CENTER` is parsed as `json.loads('[%s]')`, `POLYGON` as JSON. Responses: "Excluding object X", "Known objects: ...", "Excluded objects: ...".
- Runtime: a stateful move transform inserted lazily (only after the first exclusion) with extrusion offset tracking; ordering relative to `bed_mesh` and `tuning_tower` in the transform chain is observable; `bed_mesh` adaptive meshes read its status.
- Tests: `exclude_object.test` (M486 macro, EXCLUDE_OBJECT, RESET, CURRENT, move sequences). Fork: none in code; the KE printer enables it for Mainsail/Fluidd. Difficulty: **high** (state machine over float positions and the transform chain).

#### `extruder_stepper.py` (24 lines)
- Section `[extruder_stepper name]`: option `extruder`; wraps `kinematics.extruder.ExtruderStepper(config)`; syncs to the extruder at connect; `find_past_position` and `get_status` pass through. `SYNC_EXTRUDER_MOTION` and pressure-advance commands live in `kinematics/extruder.py` (other slice). Tests: `extruders.test`, `pressure_advance.test`. Fork: none. Difficulty: **low**.

## 4. Test coverage summary (`test/klippy`)

| Test | Range modules exercised |
|---|---|
| `bed_mesh.test` | bed_mesh |
| `bltouch.test` | bltouch, bed_mesh |
| `bed_screws.test` | bed_screws |
| `multi_z.test` | bed_tilt (`BED_TILT_CALIBRATE`) |
| `delta_calibrate.test`, `delta.test`, `rotary_delta_calibrate.test` | delta_calibrate |
| `eddy.test` | bed_mesh scan and rapid_scan |
| `load_cell.test` | ads1220, ads131m0x, bulk_sensor, bus, bed_mesh |
| `smart_effector.test`, `z_virtual_endstop.test`, `screws_tilt_adjust.test` | bed_mesh, bltouch |
| `exclude_object.test` | exclude_object |
| `extruders.test`, `pressure_advance.test` | extruder_stepper |
| `led.test` | dotstar, display templates (`display_template dtest`) |
| `temperature.test` | adc_temperature, controller_fan, error_mcu (loaded), bus (SPI temp chips) |
| `tmc.test` | endstop_phase (section only), bus (SPI/UART drivers) |
| `linuxtest.test` | ds18b20 |
| `input_shaper.test` | adxl345 (config load only) |
| `sdcard_loop.test` | display_status (section only) |
| `printers.test` | config parse and startup of every example config listed in it: board_pins, display package and menu, buttons, adc_scaled, ad5206, dac084S085, controller_fan, bed_screws, delta_calibrate, mcp4*, etc. |

No test covers: `adxl345` commands, `angle`, `aht10`, `bme280`, `bmi160`, `ads1x1x`, `canbus_*`, `axis_twist_compensation`, `delayed_gcode`, `display_status` commands, `BED_MESH_PROFILE/OFFSET/MAP`, `ENDSTOP_PHASE_CALIBRATE`, `duplicate_pin_override`, any display rendering, any webhooks endpoint.

## 5. Out-of-range neighbours (one line each)

- `force_move.py`: used by `angle.py` (`lookup_stepper`, `manual_move`).
- `gcode_move.py`: owner of the move-transform chain used by bed_mesh, bed_tilt, exclude_object.
- `homing.py`: emits `homing:home_rails_end` consumed by `endstop_phase.py`.
- `input_shaper.py`, `resonance_tester.py`: the latter consumes the `AccelQueryHelper` API from `adxl345.py`.
- `motion_report.py`: imports `bulk_sensor`.
- `probe.py`, `manual_probe.py`, `fan.py`, `led.py`, `output_pin.py`, `gcode_macro.py`, `heaters.py`, `query_adc.py`, `thermistor.py`, `tmc*.py`, `load_cell*.py`, `hx71x.py`: direct partners; covered by the other slice.

## 6. Fork-specific changes

- **None detected inside this range.** Evidence is indirect because `git log -- <path>` cannot be run here:
  - `.git/logs/HEAD` lists the fork commits: `3a63ec4` feat(creality): support stock c440x display endpoints; `4f9c676` fix(ender3-ke): restore calibrated printer configuration; `d5c37cf` feat(ender3-ke): add supervised mainline handover; `152aa00` fix(ender3-ke): gate moonraker on mainline readiness; `7777d10` feat(debug): preserve shutdown provenance; `aac1802` feat(debug): trace host motion scheduling; `4eec735` fix(motion): tolerate host scheduling stalls; `3bc8789` feat(debug): capture step compression state; `cf3c4dd` feat(debug): trace serial dispatch decisions; `7d0860d` chore(ender3-ke): retain mainline diagnostic logs; `e47b094` fix(motion): keep producer ahead of step generation. Earlier: `78b0528` feat(InputShaper): Get the input shaper working; `9b3499b` chore: load cell attempt abandoned.
  - Newest-modified files (glob mtime order) in `klippy/` are `toolhead.py`, `reactor.py`, `mcu.py`, `klippy.py`; in `klippy/extras/` only `motion_queuing.py` and `virtual_sdcard.py`. No file of this range appears among them.
  - Grep for `creality`, `c440`, `provenance`, `DEBUG-motion` in range files finds nothing. `[DEBUG-motion]` text exists in `klippy/mcu.py`.
  - `c440x` is the stock Creality UI that talks to klippy over the webhooks socket (`klippy_uds_address /tmp/klippy_uds`, per `config/ender-3-v3-ke/README.md`), so commit `3a63ec4` affects the webhooks/API area, outside this slice.
- Fork-relevant uses of modules in range (config, not code): `config/ender-3-v3-ke/printer.cfg` and siblings use `bltouch`, `bed_mesh` (adaptive, fade 1..10, 5x5), `axis_twist_compensation` (X axis, saved offsets), `exclude_object`, `delayed_gcode fan_off`, `adxl345` on the `rpi` host MCU through `spidev2.0` with `cs_pin: rpi:None`, `adc_temperature` through `EPCOS 100K B57560G104F` thermistors, and two MCUs (`mcu` serial `/dev/ttyS1` at 230400, `mcu rpi` at `/tmp/klipper_host_mcu`). These are the highest-value parity targets.
- Recommendation for the main agent: run `git log --oneline -- klippy/extras/<name>` where git is available to confirm; expect no hits in this range.

## 7. Port-difficulty table

| Unit | Rating | Reason |
|---|---|---|
| `__init__.py` | low | Empty marker. |
| `ad5206`, `dac084S085`, `board_pins`, `duplicate_pin_override`, `canbus_ids`, `canbus_stats`, `controller_fan`, `display_status`, `dotstar`, `ds18b20`, `extruder_stepper`, `error_mcu`, `bed_tilt`, `bed_screws`, `adc_scaled` | low | Short, mostly straight config plus fixed MCU strings. |
| `delayed_gcode` | low | Short; needs only the template engine and timers. |
| `aht10` | low | Single protocol, `reactor.pause` in a timer. |
| `bus` | medium | Exact command strings, two I2C dialects, deferred writes. |
| `bulk_sensor` | medium | Thread-safe queue and float regression. |
| `buttons` | medium | Encoder state tables, ack wrap, ADC ladder. |
| `adc_temperature` | medium | Interpolation parity reaches MCU ADC limits; one latent bug. |
| `ads1x1x` | medium | Mutex plus pauses inside greenlets, pin-chip registration. |
| `ads1220`, `ads131m0x` | medium | SPI bulk ADCs with register verification. |
| `adxl345`, `bmi160` | medium | Bulk data, CSV writer process, helpers shared by six chips. |
| `bme280` | medium | Four chip families and compensation math. |
| `axis_twist_compensation` | medium | In-place mutation of probe results through an event. |
| `delta_calibrate` | medium | Child-process optimiser and operation-order dependence. |
| `endstop_phase` | medium | Stepper phase math and the homing event contract. |
| LCD drivers (`hd44780`, `hd44780_spi`, `aip31068_spi`, `st7920`, `uc1701`) | medium | Framebuffer diffs must emit identical MCU bytes. |
| `font8x14`, `display/__init__` | low | Static table and dispatcher. |
| `angle` | high | numpy lstsq, five chips, chip-clock tracking. |
| `bltouch` | high | Command timing and probe session protocol. |
| `bed_mesh` | high | Transform, interpolation order, profile persistence, adaptive mesh. |
| `exclude_object` | high | Stateful transform and offsets. |
| `display/display.py`, `display/menu.py`, `menu_keys`, `display.cfg`, `menu.cfg` | high | Unsandboxed Jinja2 semantics, literal_eval, large state machine. |

## 8. Cross-cutting risks and suggested port order

### Risks seen in this slice

1. **Jinja2 is Python.** Templates in `display.cfg`, `menu.cfg`, user `[display_template]`, `[delayed_gcode]` and macros use Python expressions (`"%02d:%02d" % (...)`, `"{:^10.0%}".format(x)`, `printer.fan.speed`, `|float`, `in` tests). A Rust Jinja-like engine will not reproduce them. Decide early: embed Python, or write a restricted evaluator and port the bundled `.cfg` files to it. The delimiter overrides (`{%`, `%}`, `{`, `}`) matter.
2. **Move-transform chain order** (`gcode_move`, `bed_mesh`, `bed_tilt`, `exclude_object`, `tuning_tower`) defines results; transforms register with `force=True`, wrap `next_transform`, and are removed dynamically.
3. **SAVE_CONFIG text formats** (`%.6f`, line breaks, `, ` separators, section names) must match so config files stay compatible with a Python klippy.
4. **Message strings are API:** `mesh_map_output {json}`, `canstat_...`, `// ` prefixed responses, the shutdown banner text, ACCELEROMETER file names. Mainsail, Fluidd, Moonraker and the Creality c440x client parse them.
5. **Float parity reaching the MCU:** bltouch PWM durations, ADC min/max limits, `%d` truncation of float arguments (`ds18b20` min/max), bulk-sensor timestamps, bed mesh Z offsets, delta and angle iterative solutions.
6. **Concurrency model:** greenlet-style `reactor.pause` in timer callbacks (aht10, bme280, ads1x1x with a mutex, bltouch waits), a serial thread feeding `BulkDataQueue`, fork-based `multiprocessing` (adxl345 CSV, delta coordinate descent). The Rust port needs an equivalent async/coroutine model first.
7. **Dynamic registration:** objects created by section-name prefix (`load_config` vs `load_config_prefix`), `add_object` aliases (`probe`, `menu`, `display_template`), sensor factory tables (`add_sensor_factory`), `register_command(name, None)` to remove commands, mux commands with an unnamed default.
8. **Latent bugs the tests will not catch:** `adc_temperature._clarify_adc_range` (`except e:`), `ADS1220.lookup_sensor_error` (TypeError), `ds18b20.get_status` stuck at 0.0, `menu.py` Python 2 `unicode` branch. Decide per bug whether to reproduce.
9. **Thin test coverage:** the regression suite only parses most of this range. New golden tests are needed for display rendering, bed_mesh profile/offset/map, adxl345 CSV, angle calibration, ENDSTOP_PHASE_CALIBRATE, axis twist, delayed_gcode.
10. **Hardware-only behavior** (BLTouch, ADS chips, ADXL on spidev, c440x) cannot be verified in `test/klippy` (debugoutput mode skips connect handlers in some modules, for example `bme280`); it needs bench validation on the KE printer.

### Suggested port order (leaves first)

1. Pure data and tiny leaves: `__init__`, `board_pins`, `duplicate_pin_override`, `canbus_ids`, `font8x14`, `ad5206`, `dac084S085`, `extruder_stepper`.
2. Transport: `bus` (SPI/I2C/digital out), then `bulk_sensor`, `buttons`.
3. Diagnostics: `error_mcu`, `canbus_stats`.
4. Temperature path: `adc_temperature`, `adc_scaled`, `ds18b20`, `aht10`, `bme280`, `ads1x1x`.
5. Bulk sensors: `adxl345` (with `AccelQueryHelper`), `bmi160`, `ads1220`, `ads131m0x`, `angle` (last, after choosing the lstsq approach).
6. Fans and LEDs: `controller_fan` (after `fan.py`), `dotstar` (after `led.py`).
7. Calibration and probing (needs `probe.py`, `gcode_move` transforms, `mathutil`): `bed_tilt`, `bed_screws`, `bltouch`, `bed_mesh`, `axis_twist_compensation`, `delta_calibrate`, `endstop_phase`, `exclude_object`.
8. Templates and UI (needs the decision from risk 1): `delayed_gcode`, `display_status`, the `display/` low-level drivers, then `display.py`, `menu_keys.py`, `menu.py`, and the bundled `.cfg` files.
