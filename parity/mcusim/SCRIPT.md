# MCU simulator script format

`mcusim.py` reads a JSON script (`--script SCRIPT.json`). All sections are
optional. Without a script, ADCs read 0, input pins read their pull-up
level, endstops never trigger, and SPI/I2C/tmcuart replies are zero bytes
(tmcuart falls back to the TMC2208 register model below).

```json
{
  "start_state": {"is_config": 0, "crc": 0},
  "forced_shutdown_at": null,
  "adc": {"PC4": {"value": 3911}, "52": {"table": [[0, 300], [10, 900]]}},
  "thermal": {"52": {"ambient_adc": 300, "span_adc": 2500,
                     "tau_seconds": 20.0, "pwm_oid": 19}},
  "pins": {"PC15": 0},
  "endstop_triggers": {"PA5": {"triggered_level": 0, "travel_steps": 20}},
  "bltouch": {"sensor_pin": "PC14", "control_pin": "PC13",
              "contact_steps": [2000, 1996]},
  "spi_replies": {"02": "0000"},
  "i2c_replies": {"40": "00"},
  "tmcuart_replies": {"ad": "01"},
  "tmc_registers": {"111": 0},
  "sensor_bulk": {"64": {"chunks": [{"data": "0000"}]}},
  "counter": {"71": {"count": 0}},
  "thermocouple": {"60": {"value": 0, "fault": 0}}
}
```

Pin names are the dictionary's `pin` enumeration names (`PA5`, `gpio12`).
The host assigns oids at connect time, so pin names keep a script valid
when the config changes; oid keys (decimal strings) are still accepted
where noted.

## Timing model

The sim mirrors the firmware timers in virtual time. Every queued step
runs at the clock the host scheduled (`reset_step_clock` base, then
`interval`, `interval+add`, ... as in `src/stepper.c`), and
`stepper_position` uses the firmware's uint32 position encoding. Endstop
sampling (`rest_ticks`, then `sample_count` samples `sample_ticks` apart),
trsync reports and timeouts, ADC reports and button reports are events
at absolute clocks. The main loop sleeps until the earliest event, so a
trigger stops its steppers on the exact step the firmware would, and the
`trsync_state` and `endstop_state` clocks the host reads do not depend on
when the host's messages happened to arrive.

## Sections

- `start_state`: `is_config` (0/1) and `crc` returned by `get_config`
  before any `finalize_config`. Set `is_config: 1` with a wrong CRC to
  reproduce the printer's CRC-mismatch reset handshake: the host sees a
  configured MCU whose CRC differs from its computed CRC and sends
  `emergency_stop` + `reset` (via `config_reset` when the dict uses that
  name). With `is_config: 0` the host does the normal identify /
  allocate / config / finalize flow.
- `forced_shutdown_at`: virtual seconds; at that time the simulator
  calls `do_shutdown("Command request")`, producing a `shutdown` message
  the host turns into an emergency stop.
- `adc`: keyed by analog_in oid or pin name. `{"value": N}` (or
  `{"mode": "constant", "value": N}`) or `{"table": [[t0, v0], [t1, v1],
  ...]}`, piecewise linear in virtual seconds since the sim started and
  clamped at the ends. `value` is raw ADC counts (0..4095 for
  `ADC_MAX=4095`); the reported `analog_in_state` value is
  `round(sample * sample_count)` (sum, per `src/adccmds.c`),
  range-checked against the query min/max. A report goes out when the
  last sample of a period has been read.
- `thermal`: first-order heater model, keyed by analog_in oid or pin
  name. `ambient_adc` is the unheated ADC count, `span_adc` the
  additional count at duty 1, `tau_seconds` the time constant, and
  `pwm_oid` the digital_out oid whose `queue_digital_out` duty
  (`on_ticks / cycle_ticks` after `set_digital_out_pwm_cycle`) drives
  the model. Update rule per step of `dt` seconds (exact discretization,
  no randomness, no wall clock): `v += (target - v) * (1 - exp(-dt/tau))`
  with `target = ambient_adc + duty * span_adc`.
- `pins`: static physical input levels (what `gpio_in_read` returns),
  keyed by pin name or decimal pin number. Unlisted input pins read 1
  when configured with a pull-up and 0 otherwise. Buttons
  (`src/buttons.c`) report once when the levels differ from the
  `buttons_query` invert mask, then retransmit every
  `retransmit_count + 1` samples until `buttons_ack`. Endstop pins
  without an `endstop_triggers` or `bltouch` entry also read this level.
- `endstop_triggers`: keyed by endstop pin name.
  `{"triggered_level": L, "travel_steps": N}`: while an `endstop_home` is
  armed, the pin reads `L` once a stepper linked to its trsync (by
  `stepper_stop_on_trigger`) has taken `N` steps after the first sample
  clock, and `1 - L` before that and whenever the endstop is not
  homing. `L` is the physical level, so an inverted config pin (`!PA5`)
  that triggers low uses `"triggered_level": 0`. Steps taken before the
  first sample clock (a retract move still running when `endstop_home`
  arrives) do not count, so the second home after a retract triggers
  after `N` steps of the new move.
- `bltouch`: a BLTouch probe on `sensor_pin` driven by `control_pin`.
  The sim decodes each nonzero `queue_digital_out` pulse on the control
  pin at its scheduled clock to a `klippy/extras/bltouch.py` command by
  its width (`pin_down` 0.650 ms, `touch_mode` 1.165 ms, `pin_up` 1.475
  ms, `reset` 2.190 ms, ...; a width that matches none within 50 us is an
  error). The sensor pin reads 0 while raised, 1 when raised after
  `touch_mode` (the host's connection self test), and while lowered 0
  until the linked Z stepper has travelled the descent's contact
  distance, then 1. Each `endstop_home` armed while the probe is lowered
  is one descent; descent `k` uses `contact_steps[k]` (the list wraps
  around). The trace records `bltouch <command> at clock <c>` and
  `bltouch descent <k> contact after <n> steps`.
- `spi_replies` / `i2c_replies` / `tmcuart_replies`: hex request bytes
  (lowercase, no spaces) to hex reply bytes. A reply may be a single hex
  string, or a list of hex strings cycled per request to the same key.
  Missing keys return zero bytes of the requested length, except for
  tmcuart, which then answers from the register model.
- `tmc_registers`: `{"<reg>": value}` overrides (decimal register
  number) for the TMC2208 register model. Without an override the sim
  answers IFCNT with its write counter (so the host write-verify
  handshake passes), DRV_STATUS with `cs_actual` 8 and no error flags,
  GSTAT 0 and GCONF 0x100.
- `sensor_bulk`: `{"<oid>": {"chunks": [{"data": "hex"}, ...]}}` for
  `adxl345_data` chunks consumed by `query_adxl345`, and generic
  `sensor_bulk_status` elsewhere.
- `counter` / `thermocouple`: direct response fields for
  `query_counter` (`count`) and `query_thermocouple` (`value`, `fault`).

## Trace and capture

`--trace FILE`: one JSON line per received command (decoded with its
parameters) and sent response, plus the BLTouch lines above:
`{"t": <virtual seconds>, "clock": <u32>, "mcu": <name>, "dir":
"rx"|"tx", "msg": <text>}`. Transport-level ack/nak blocks are not
traced. A command handler error is traced as `error <command>` and
written to stderr.

`--capture FILE`: every byte read from the host, unmodified (the live
counterpart of a batch `-o` file; `parsedump.py` decodes it).
