# Mainline Klipper Auto-Start Services

This guide documents how to create boot services for the mainline Klipper setup
on the Nebula Pad.

The service files are stored under `/usr/data/`:

- `/usr/data/S57klipper_mcu_mainline`
- `/usr/data/S55klipper_mainline`

The services use `screen`, so install it through Entware first.

## Prerequisites

Root the Pad and install Entware with Guilouz Helper Script first.

Then install `screen`:

```bash
opkg update
opkg install screen
```

Required files on the Pad:

- `/usr/data/klipper_mcu_new`
- `/usr/data/klipper-mainline/klippy/klippy.py`
- `/usr/data/printer_data/config-mainline/printer.cfg`
- `/usr/data/klipper-mainline/klippy/chelper/c_helper.so`
- `/usr/data/printer_data/config/moonraker.conf`

Moonraker must point to the mainline Klipper socket:

```ini
klippy_uds_address: /tmp/klippy_uds
```

The stock Klipper processes must be stopped before mainline Klipper is started.
The MCU service below stops both stock services automatically, but when testing
by hand run:

```bash
/etc/init.d/S55klipper_service stop
/etc/init.d/S57klipper_mcu stop
```

Do not run stock Klipper and mainline Klipper at the same time; both try to use
the same MCU-side resources and sockets.

## MCU Service

Create `/usr/data/S57klipper_mcu_mainline`:

```bash
cat >/usr/data/S57klipper_mcu_mainline <<'EOF'
#!/bin/sh

PATH=/usr/data/opt/sbin:/usr/data/opt/bin:/opt/sbin:/opt/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PATH

NAME=klipper_mcu_mainline
SCREEN_NAME=klipper_mcu_mainline
MCU_BIN=/usr/data/klipper_mcu_new
MCU_SOCKET=/tmp/klipper_host_mcu
LOG=/usr/data/klipper_mcu_mainline.log

case "$1" in
  start)
    /etc/init.d/S55klipper_service stop >/dev/null 2>&1
    /etc/init.d/S57klipper_mcu stop >/dev/null 2>&1
    screen -S "$SCREEN_NAME" -X quit >/dev/null 2>&1
    rm -f "$MCU_SOCKET"
    screen -dmS "$SCREEN_NAME" sh -c \
      "while true; do $MCU_BIN -r >>$LOG 2>&1; rm -f $MCU_SOCKET; sleep 1; done"
    elapsed=0
    while [ ! -e "$MCU_SOCKET" ]; do
      if [ "$elapsed" -ge 15 ]; then
        echo "$NAME failed: $MCU_SOCKET was not created"
        exit 1
      fi
      sleep 1
      elapsed=$((elapsed + 1))
    done
    ;;
  stop)
    screen -S "$SCREEN_NAME" -X quit >/dev/null 2>&1
    rm -f "$MCU_SOCKET"
    ;;
  restart)
    "$0" stop
    sleep 1
    "$0" start
    ;;
  status)
    screen -list | grep -q "$SCREEN_NAME"
    ;;
  *)
    echo "Usage: $0 {start|stop|restart|status}"
    exit 1
    ;;
esac
EOF

chmod +x /usr/data/S57klipper_mcu_mainline
```

## klippy Service

Create `/usr/data/S55klipper_mainline`:

```bash
cat >/usr/data/S55klipper_mainline <<'EOF'
#!/bin/sh

PATH=/usr/data/opt/sbin:/usr/data/opt/bin:/opt/sbin:/opt/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PATH

NAME=klipper_mainline
SCREEN_NAME=klipper_mainline
PYTHON=/usr/share/klippy-env/bin/python
KLIPPY=/usr/data/klipper-mainline/klippy/klippy.py
CONFIG=/usr/data/printer_data/config-mainline/printer.cfg
UDS=/tmp/klippy_uds
MCU_SOCKET=/tmp/klipper_host_mcu
LOG=/usr/data/klippy-mainline.log

case "$1" in
  start)
    for i in $(seq 1 30); do
      [ -e "$MCU_SOCKET" ] && break
      sleep 1
    done
    if [ ! -e "$MCU_SOCKET" ]; then
      echo "$NAME failed: $MCU_SOCKET was not created"
      exit 1
    fi

    rm -f "$UDS"
    screen -dmS "$SCREEN_NAME" sh -c "$PYTHON $KLIPPY $CONFIG -a $UDS 2>&1 | tee $LOG"
    ;;
  stop)
    screen -S "$SCREEN_NAME" -X quit >/dev/null 2>&1
    rm -f "$UDS"
    ;;
  restart)
    "$0" stop
    sleep 1
    "$0" start
    ;;
  status)
    screen -list | grep -q "$SCREEN_NAME"
    ;;
  *)
    echo "Usage: $0 {start|stop|restart|status}"
    exit 1
    ;;
esac
EOF

chmod +x /usr/data/S55klipper_mainline
```

## Test Manually

Stop the stock Klipper service first:

```bash
/etc/init.d/S55klipper_service stop
/etc/init.d/S57klipper_mcu stop
sleep 2
```

Run the services by hand first:

```bash
/usr/data/S57klipper_mcu_mainline start
/usr/data/S55klipper_mainline start
```

Check the detached sessions:

```bash
screen -list
```

Attach to a session for debugging:

```bash
screen -r klipper_mcu_mainline
screen -r klipper_mainline
```

Detach from a `screen` session with `Ctrl-A`, then `D`.

## Boot Auto-Start Status

Do not disable the stock Klipper boot services right now.

The printer currently needs the stock services to complete boot reliably. If
`/etc/init.d/S55klipper_service` or `/etc/init.d/S57klipper_mcu` are prevented
from starting during boot, the printer may fail to boot correctly and SSH may
not come up.

Install `scripts/S99zz_mainline_switch` as
`/etc/init.d/S99zz_mainline_switch` and
`scripts/wait-creality-start-mainline.sh` as
`/usr/data/wait-creality-start-mainline.sh`.

The switch service waits for stock Klipper to report `ready`, stops the stock
services through the mainline MCU service, starts mainline on
`/tmp/klippy_uds`, and restarts Moonraker. Keep both stock boot services
enabled. The stock Creality display reconnects to the same socket after the
handover.

## Stop Services

```bash
/usr/data/S55klipper_mainline stop
/usr/data/S57klipper_mcu_mainline stop
```

If symlinked into `/etc/init.d`, the same commands can be run through those
paths:

```bash
/etc/init.d/S55klipper_mainline stop
/etc/init.d/S57klipper_mcu_mainline stop
```
