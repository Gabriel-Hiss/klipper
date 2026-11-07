# Unit inventory

`units.json` lists every porting unit in the Klipper host. `build.py` regenerates it from the source tree; counts printed by the last run are the numbers in the report, not a stable contract. Regenerate with `python3 parity/inventory/build.py`.

## Coverage

Every `klippy/*.py` file, every `klippy/kinematics/*.py` file, every `klippy/extras/**/*.py` file (including `display/`), every `*.cfg` data file under `klippy/extras/` (`display.cfg`, `menu.cfg`, `temperature_sensors.cfg`), and every `klippy/chelper/*.c` and `*.h` file. `__pycache__/` is excluded. Each covered file is partitioned into contiguous, non-overlapping line spans, one per unit.

## Unit fields

- `id`: stable dotted id, for example `extras.heaters.HeaterControl` or `chelper.stepcompress.Stepcompress`. The prefix mirrors the source path (`extras.display.menu.MenuSection`), the last part names the class, function or group that opens the unit. Colliding ids across one file get a `_N` suffix in order.
- `source`: `{"path", "start", "end"}` 1-based inclusive line span in the repo-relative file path.
- `kind`: `py` for Python sources, `c` for `klippy/chelper/` C and header files, `tool` for files that are listed but not ported as code (`klippy/console.py`, `klippy/parsedump.py`, and the `.cfg` data files, which the port embeds as data through `pytext`). `console.py` and `parsedump.py` are the only standalone tools; they are kept listed on purpose.
- `lines`: `end - start + 1`.
- `crate`: the Rust crate that will own the unit. Values: `core`, `msgproto`, `chelper_motion`, `chelper_serial`, `kinematics`, `extras_<family>`.
- `module`: the Rust module path inside that crate, for example `core.gcode` or `extras_tmc_drivers.tmc5160`.
- `deps`: source-module dependencies of the owning file (Python `klippy.*` / `extras.*` / `chelper` imports, or C `#include "..."` headers), mapped to the same dotted convention. Units in one file share the file's `deps`; per-symbol dependency analysis is out of scope.
- `contracts`: `{"gcode", "config", "status", "endpoints", "mcu", "logs"}`, each a sorted-by-first-seen list capped at 40 entries, extracted by regex from the unit's own lines only.
  - `gcode`: `register_command` / `register_mux_command` names and `def cmd_*` functions (uppercased).
  - `config`: option names read through `config.get*`.
  - `status`: dict keys seen near a `get_status` line in the same unit (heuristic, because return dicts are constructed inline).
  - `endpoints`: webhooks `add_endpoint` paths.
  - `mcu`: MCU command/response format strings containing `oid=`, `response=` or `data=`.
  - `logs`: `logging.*` format strings containing `%`.
  The extraction is deliberately conservative: it names the contract, it does not prove completeness. `parity/maps/map-core.md` and `map-extras-*` remain the detailed contract sources.
- `verification`: `{"class", "reason"}`. `sim` means the unit is verifiable with the MCU simulator plus the virtual clock. `ke-only` means real-hardware timing or physical measurement is required (probe contact, LCD waveforms, bed flatness, non-cartesian kinematics); the reason names the physical effect. `unverifiable` means no non-interactive scenario can exercise it (interactive console, offline dump reader, host-process identity, rotating log files).
- `recordable`: `true` if the unit's I/O is serializable, so per-unit JSONL recordings under `tests/fixtures/recordings/<id>/` apply. `false` for the reactor, the threading/reactor-adjacent C (`serialqueue`, `steppersync`, `pollreactor`), and any file that imports `threading`, `greenlet`, `pty`, `socket`, `subprocess`, `faulthandler`, reads `/proc` or calls `os.cpu*`. Timing-driven units are verified at scenario level through the harness instead.
- `difficulty`: `high`, `medium` or `low`. Heuristic from file size plus a fixed high list (`reactor`, `mcu`, `configfile`, `gcode`, `webhooks`, `bed_mesh`, `probe_eddy_current`, `gcode_macro`, the three indivisible C kernels).
- `layer`: integration layer. `0` uses only `pycompat`/`kos`; `1` is `msgproto`; `2` is `core`; `3` is `motion` (kinematics plus the motion C kernels); `4` is `extras`; `5` is `api` (process entry, webhooks, interactive tools). See `layers.md` for what can start in each layer.
- `indivisible` and `reason`: present on the ten units that must not be split further (see below). Splitting them would break a single shared state machine.
- `fields`, `class`, `shared_with`: only on method-group units split from a large class. `fields` lists the `self.*` attributes the class assigns (the fields the groups share), `class` names the source class, and `shared_with` lists the sibling unit ids of the same class.

## Indivisible units

- `chelper.stepcompress.Stepcompress` (whole `stepcompress.c`): compression core.
- `chelper.serialqueue.Serialqueue` (whole `serialqueue.c`): serial queue core.
- `chelper.itersolve.Itersolve` (whole `itersolve.c`): solver.
- `reactor.SelectReactor` (`klippy/reactor.py`): reactor dispatch and greenlets. `PollReactor` and `EPollReactor` stay separate.
- `configfile.ConfigFileReader`, `configfile.ConfigAutoSave`, `configfile.PrinterConfig`: configfile parse and autosave share one mutable config tree, so each of the three classes that own a pass over that tree is kept whole.
- `mcu.MCUConnectHelper`: MCU config/connect state machine.
- `gcode.GCodeDispatch`: G-code dispatch.
- `extras.gcode_macro.PrinterGCodeMacro`: the gcode_macro Jinja environment (`jinja2.Environment('{%', '%}', '{', '}')`, `load_template`).

## Extras families

Family names follow `parity/maps/map-extras-{a,b,c}.md`, merged where the three maps cover overlapping ground.

| Family (crate suffix) | Files |
|---|---|
| `bus_transport` | `bus`, `bulk_sensor`, `buttons`, `board_pins`, `duplicate_pin_override`, `canbus_ids`, `canbus_stats`, `error_mcu`, `query_adc`, `query_endstops` |
| `digipots_dac` | `ad5206`, `dac084S085`, `mcp4018`, `mcp4451`, `mcp4728` |
| `adc_temperature` | `adc_temperature`, `adc_scaled`, `ads1x1x`, `ads1220`, `ads131m0x`, `ds18b20`, `aht10`, `bme280`, `thermistor`, `temperature_sensor`, `temperature_combined`, `temperature_host`, `temperature_mcu`, `temperature_probe`, `temperature_fan`, `spi_temperature`, `sht3x`, `verify_heater`, `pid_calibrate`, `z_thermal_adjust`, `lm75`, `htu21d`, `temperature_sensors.cfg` |
| `accelerometers_angle` | `adxl345`, `bmi160`, `angle`, `icm20948`, `mpu9250`, `lis2dw`, `lis3dh` |
| `probes_leveling` | `bltouch`, `bed_mesh`, `bed_tilt`, `bed_screws`, `axis_twist_compensation`, `delta_calibrate`, `endstop_phase`, `probe`, `probe_eddy_current`, `trigger_analog`, `smart_effector`, `z_tilt`, `quad_gantry_level`, `screws_tilt_adjust`, `safe_z_home`, `skew_correction`, `tuning_tower` |
| `displays_ui` | everything under `display/` (drivers, `menu`, `menu_keys`, `font8x14`, `__init__`, `display.cfg`, `menu.cfg`) plus `display_status` |
| `leds` | `dotstar`, `led`, `neopixel`, `pca9533`, `pca9632` |
| `fans` | `controller_fan`, `fan`, `fan_generic`, `heater_fan` |
| `macros_templates` | `delayed_gcode`, `gcode_macro`, `gcode_button`, `homing_override`, `gcode_arcs`, `firmware_retraction` |
| `print_job` | `exclude_object`, `virtual_sdcard`, `sdcard_loop`, `print_stats`, `pause_resume`, `statistics` |
| `motion_extras` | `extruder_stepper`, `motion_queuing`, `input_shaper`, `shaper_defs`, `gcode_move`, `homing`, `force_move`, `manual_stepper`, `motion_report`, `stepper_enable`, `idle_timeout`, `garbage_collection` |
| `pins_outputs` | `output_pin`, `multi_pin`, `pwm_tool`, `pwm_cycle_time`, `static_pwm_clock`, `static_digital_output`, `servo`, `pulse_counter`, `samd_sercom`, `sx1509`, `replicape` |
| `heaters_temperature` | `heaters`, `heater_bed`, `heater_generic`, `homing_heaters` |
| `filament_sensors` | `filament_switch_sensor`, `filament_motion_sensor`, `hall_filament_width_sensor`, `tsl1401cl_filament_width_sensor` |
| `load_cells_probing` | `hx71x`, `load_cell`, `load_cell_probe`, `ldc1612`, `manual_probe` |
| `tmc_drivers` | `tmc`, `tmc_uart`, `tmc2130`, `tmc2208`, `tmc2209`, `tmc2240`, `tmc2660`, `tmc5160` |
| `resonance_analysis` | `resonance_tester`, `shaper_calibrate` |
| `host_state` | `respond`, `save_variables` |
| `misc` | `palette2`, `__init__.py` |

## Crate split of the C helper

`chelper_serial` owns `msgblock`, `serialqueue`, `pollreactor`, `trdispatch`, `pyhelper`, and the `chelper/__init__.py` cffi loader. `chelper_motion` owns `trapq`, `itersolve`, `stepcompress`, `steppersync`, `kin_cartesian`, `kin_corexy`, `kin_corexz`, `kin_delta`, `kin_deltesian`, `kin_extruder`, `kin_generic`, `kin_idex`, `kin_polar`, `kin_rotary_delta`, `kin_shaper`, `kin_winch`. Headers sit in the same crate as their implementation.

## Generator notes and limits

`build.py` uses `ast` for Python and a brace-depth splitter for C (function definitions at brace depth 0, matched by `^[\\w\\s]+?\\b(name)\\s*\\([^;{]*\\)\\s*\\{$`; three whole files are kept as single units because they are indivisible). Spans are normalized to be contiguous per file, so the inventory has no gaps and no overlaps inside a file. The target unit size is `round(total_lines / 870)`, clamped to 42..60, which lands near 1000 units for the current tree. Unit lines of about 30-80 are the goal; single large methods and the indivisible blocks may exceed it.

Known limits: `contracts.status` is a heuristic (dict keys near `get_status`); C function matching can miss K&R-style or macro-wrapped definitions, in which case whole-file or line-chunk fallback units appear; `verification` for `ke-only` uses filename hints, so a sim-capable unit on the hint list is a conservative over-claim, never an under-claim that blocks KE verification.

## Reproducing the counts

```
python3 -m py_compile parity/inventory/build.py
python3 parity/inventory/build.py
```

The second command prints the unit count, the split target and totals broken down by crate, layer, verification class and recordable flag, plus the indivisible unit ids. Those printed totals are what the report cites.
