# Integration order

Units integrate in layers. A unit in layer N can start as soon as every unit in a lower layer it depends on is done. Total is 1027 units (last `python3 parity/inventory/build.py` run), split 111 / 31 / 10 / 11 / 115 / 749 across layers 2 / 5 / 0 / 1 / 3 / 4. The counts shift when the tree changes; regenerate for fresh numbers. Rule: a unit's `layer` field in `units.json` is the source of truth, this file is the plan.

## Layer 0: only pycompat/kos (10 units)

`core.util`, `core.queuelogger`, `core.mathutil`. No `klippy.*` imports, so they can start immediately with only the compat layer done. `mathutil.coordinate_descent` spawns a subprocess in the reference (kept, verified ke-only for that subprocess path). `util` and `queuelogger` are `unverifiable` (host process identity and log rotation), everything else is `sim`.

## Layer 1: msgproto (11 units)

`msgproto` crate (all of `klippy/msgproto.py`). Depends on nothing in the tree. Its only hard consumers are the core crate and the `parsedump` tool. Start immediately after layer 0. `parsedump` (1 unit, `core.parsedump`, kind `tool`) also lives in this layer because it reads `.dict` files with `msgproto`, but it is not part of the build.

## Layer 2: core (111 units)

`core` crate except the layer-0, layer-1 and layer-5 units: `reactor`, `configfile`, `gcode`, `mcu`, `pins`, `clocksync`, `serialhdl`, `toolhead`, `stepper`, `msgproto`'s callers, and `chelper_serial` (`msgblock`, `serialqueue`, `pollreactor`, `trdispatch`, `pyhelper`, the cffi loader in `chelper/__init__.py`). Needs layers 0-1 only. The indivisible units (`reactor.SelectReactor`, the three `configfile` units, `mcu.MCUConnectHelper`, `gcode.GCodeDispatch`) start here; they are the first hard units worth a pilot.

## Layer 3: motion (115 units)


`kinematics` crate (all 15 kinematics files) and `chelper_motion` (the 16 motion C files). Needs layer 2 (`toolhead`, `stepper`, `mcu` plus core). The indivisible C units `chelper.stepcompress.Stepcompress`, `chelper.serialqueue.Serialqueue` (actually layer 2 by crate; it is listed with chelper_serial) and `chelper.itersolve.Itersolve` are the motion proof points. `kin_polar`, `kin_rotary_delta`, `kin_winch` and the hybrid kinematics are `ke-only`; the KE printer itself only needs `cartesian`.

## Layer 4: extras (749 units)

All `extras_<family>` crates: `bus_transport`, `digipots_dac`, `adc_temperature`, `accelerometers_angle`, `probes_leveling`, `displays_ui`, `leds`, `fans`, `macros_templates`, `print_job`, `motion_extras`, `pins_outputs`, `heaters_temperature`, `filament_sensors`, `load_cells_probing`, `tmc_drivers`, `resonance_analysis`, `host_state`, `misc`. Needs layers 0-3 (`configfile`, `gcode`, `pins`, `mcu`, `motion_queuing`, webhooks-independent interfaces). Families that only talk to the MCU object (`tmc_drivers`, `adc_temperature`, `accelerometers_angle`, `bus_transport`, `pins_outputs`, `digipots_dac`) can start once layer 2 is done and do not need motion. `motion_extras` and `probes_leveling` wait for layer 3. Booking order inside the layer: `bus_transport` first (everything else imports it), then `macros_templates` (Jinja env is indivisible), then the heater/fan families, then leveling/probing, then displays.

## Layer 5: api (31 units)

`core.klippy` (process entry and restart loop), `core.webhooks` (Unix-socket JSON API), `core.console` (kind `tool`, interactive debug console, unverifiable). Needs layers 0-4: the API surface is the last thing that integrates, because webhooks and the entry point sit above the printer object registry. Everything here is `sim` except `console`, which is `unverifiable`.
