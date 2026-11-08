# MCU simulator script format

`mcusim.py` reads a JSON script (`--script SCRIPT.json`). All sections are
optional; a missing script means constant-zero sensor values, immediate
endstop assertions only via positional triggers, and zero-length SPI/I2C/
tmcuart replies.

```json
{
  "start_state": {"is_config": 0, "crc": 0},
  "forced_shutdown_at": null,
  "adc": {"52": {"mode": "constant", "value": 2048}},
  "thermal": {"52": {"ambient_adc": 300, "span_adc": 2500,
                     "tau_seconds": 20.0, "pwm_oid": 19}},
  "buttons": {"48": {"0": 0, "1": 0}},
  "endstop_pin_values": {"29": 0},
  "endstop_triggers": {"28": {"stepper_oid": 25, "at_steps": 400,
                             "pin_value": 1},
                       "27": {"stepper_oid": 26, "at_steps": 500,
                             "pin_value": 1}},
  "spi_replies": {"02": "0000"},
  "i2c_replies": {"40": "00"},
  "tmcuart_replies": {"ad": "01"},
  "sensor_bulk": {"64": {"chunks": [{"data": "0000"}]}},
  "counter": {"71": {"count": 0}},
  "thermocouple": {"60": {"value": 0, "fault": 0}}
}
```

## Sections

- `start_state`: `is_config` (0/1) and `crc` returned by `get_config`
  before any `finalize_config`. Set `is_config: 1` with a wrong CRC to
  reproduce the printer's CRC-mismatch reset handshake: the host sees a
  configured MCU whose CRC differs from its computed CRC and sends
  `emergency_stop` + `reset` (via `config_reset` when the dict uses that
  name). With `is_config: 0` the host does the normal identify /
  allocate / config / finalize flow.
- `forced_shutdown_at`: virtual seconds; when reached the simulator
  calls `do_shutdown("Command request")`, producing a `shutdown` message
  the host turns into an emergency stop.
- `adc`: `{"<analog_in oid>": {"mode": "constant", "value": N}}` or
  `{"table": [[t0, v0], [t1, v1], ...]}`. The table is piecewise linear in
  virtual time (seconds since `starting`) and clamped at the ends.
  `value` is raw ADC counts (0..4095 for `ADC_MAX=4095`); the reported
  `analog_in_state` value is `round(sample * sample_count)` (sum, per
  `src/adccmds.c`), range-checked against the query min/max.
- `thermal`: first-order heater model per analog_in oid.
  `ambient_adc` is the unheated ADC count, `span_adc` the additional
  count at duty 1, `tau_seconds` the time constant, and `pwm_oid` the
  digital_out oid whose `queue_digital_out` duty (`on_ticks /
  cycle_ticks` after `set_digital_out_pwm_cycle`) drives the model.
  Update rule per step of `dt` seconds (exact discretization, no
  randomness, no wall clock): `v += (target - v) * (1 - exp(-dt/tau))`
  with `target = ambient_adc + duty * span_adc`. Coefficients are the
  concrete numbers in `mcusim.py` `thermal_update`; change them per
  heater (extruder faster, bed slower).
- `buttons`: `{"<buttons oid>": {"<pos>": 0/1}}` state bytes emitted by
  `buttons_state`. Queried via `buttons_query` and repeated every
  `rest_ticks`.
- `endstop_pin_values`: static pin levels by pin number (decimal string).
- `endstop_triggers`: `{"<endstop oid>": {"stepper_oid": N,
  "at_steps": M, "pin_value": 1}}`. The pin reads `pin_value` once the
  linked stepper has advanced at least `at_steps` from the position at
  `endstop_home` (mirrors `src/endstop.c` polling with oversampling).
  Values below `at_steps` read as `1 - pin_value`.
- `spi_replies` / `i2c_replies` / `tmcuart_replies`: hex request bytes
  (lowercase, no spaces) to hex reply bytes. A reply may be a single hex
  string, or a list of hex strings cycled per request to the same key.
  Missing keys return zero bytes of the requested length. This covers
  TMC2208 `tmcuart_send` register traffic and bulk-sensor SPI probes.
- `sensor_bulk`: `{"<oid>": {"chunks": [{"data": "hex"}, ...]}}` for
  `adxl345_data` chunks consumed by `query_adxl345`, and generic
  `sensor_bulk_status` elsewhere.
- `counter` / `thermocouple`: direct response fields for
  `query_counter` (`count`) and `query_thermocouple` (`value`, `fault`).

## Trace

One JSON line per received command and sent response:
`{"t": <virtual seconds>, "clock": <u32>, "mcu": <name>, "dir":
"rx"|"tx", "msg": <text>}`. Transport-level ack/nack blocks are not
traced.
