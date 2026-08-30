#!/bin/sh

PYTHON=/usr/share/klippy-env/bin/python
STOCK_SOCKET=/tmp/klippy_uds
MAINLINE_SOCKET=/tmp/klippy_uds
LOG_PREFIX="mainline-switch"
MOONRAKER=/etc/init.d/S56moonraker_service
MAINLINE_MCU=/usr/data/S57klipper_mcu_mainline
MAINLINE_KLIPPY=/usr/data/S55klipper_mainline
START_TIMEOUT=75
MAX_ATTEMPTS=3


klipper_socket_ready() {
    socket_path=$1
    [ -S "$socket_path" ] || return 1
    "$PYTHON" - "$socket_path" <<'PY'
import json
import socket
import sys

sock = socket.socket(socket.AF_UNIX)
sock.settimeout(3.0)
try:
    sock.connect(sys.argv[1])
    request = {
        "id": 1,
        "method": "objects/query",
        "params": {"objects": {"webhooks": None}},
    }
    sock.sendall((json.dumps(request) + "\x03").encode())
    response = b""
    while b"\x03" not in response:
        chunk = sock.recv(65536)
        if not chunk:
            raise RuntimeError("Klipper closed the API socket")
        response += chunk
    payload = json.loads(response.split(b"\x03", 1)[0])
    state = payload["result"]["status"]["webhooks"]["state"]
    raise SystemExit(0 if state == "ready" else 1)
except Exception:
    raise SystemExit(1)
finally:
    sock.close()
PY
}

wait_until_ready() {
    socket_path=$1
    timeout=$2
    elapsed=0
    while [ "$elapsed" -lt "$timeout" ]; do
        if klipper_socket_ready "$socket_path"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

echo "$LOG_PREFIX: waiting for stock Klipper to become ready"
until klipper_socket_ready "$STOCK_SOCKET"; do
    sleep 1
done

echo "$LOG_PREFIX: stock Klipper is ready; hiding handover from Moonraker"
"$MOONRAKER" stop

attempt=1
while [ "$attempt" -le "$MAX_ATTEMPTS" ]; do
    echo "$LOG_PREFIX: starting mainline attempt $attempt/$MAX_ATTEMPTS"
    "$MAINLINE_KLIPPY" stop
    "$MAINLINE_MCU" stop
    "$MAINLINE_MCU" start
    "$MAINLINE_KLIPPY" start

    if wait_until_ready "$MAINLINE_SOCKET" "$START_TIMEOUT"; then
        echo "$LOG_PREFIX: mainline Klipper is ready; starting Moonraker"
        "$MOONRAKER" start
        echo "$LOG_PREFIX: handover complete; Creality UI remains active"
        exit 0
    fi

    echo "$LOG_PREFIX: attempt $attempt timed out after ${START_TIMEOUT}s" >&2
    attempt=$((attempt + 1))
done

echo "$LOG_PREFIX: mainline failed after $MAX_ATTEMPTS attempts" >&2
"$MAINLINE_KLIPPY" stop
"$MAINLINE_MCU" stop
"$MOONRAKER" start
exit 1
