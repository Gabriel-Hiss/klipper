#!/bin/sh

PYTHON=/usr/share/klippy-env/bin/python
STOCK_SOCKET=/tmp/klippy_uds
MAINLINE_SOCKET=/tmp/klippy_uds
LOG_PREFIX="mainline-switch"


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



echo "$LOG_PREFIX: waiting for stock Klipper to become ready"
until klipper_socket_ready "$STOCK_SOCKET"; do
    sleep 1
done

echo "$LOG_PREFIX: stock Klipper is ready; starting mainline"
/usr/data/S57klipper_mcu_mainline start
/usr/data/S55klipper_mainline start
/etc/init.d/S56moonraker_service restart

echo "$LOG_PREFIX: waiting for mainline Klipper to become ready"
until klipper_socket_ready "$MAINLINE_SOCKET"; do
    sleep 1
done

echo "$LOG_PREFIX: mainline Klipper is ready; leaving Creality UI active"

while [ -S "$MAINLINE_SOCKET" ]; do
    sleep 60
done

echo "$LOG_PREFIX: mainline socket disappeared" >&2
exit 1
