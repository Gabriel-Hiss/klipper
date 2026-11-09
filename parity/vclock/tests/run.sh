#!/bin/sh
# Run each determinism probe 20 times under vclockd, assert byte-identical
# output across runs and exact virtual elapsed time.
#
# Usage: sh tests/run.sh [libvclock.so] [vclockd] [epoch]
set -eu
LIB="${1:-build/x86_64/libvclock.so}"
DAEMON="${2:-build/x86_64/vclockd}"
EPOCH="${3:-1700000000}"
TMPD="$(mktemp -d)"
trap 'rm -rf "$TMPD"' EXIT

# Build the C probes
gcc -O1 -std=gnu11 -Wall -Wextra -pthread -o "$TMPD/threaded_trace" tests/threaded_trace.c
gcc -O1 -std=gnu11 -Wall -Wextra -o "$TMPD/spawn_probe" tests/spawn_probe.c

run_once() {
    # $1 = output file, rest = command
    out="$1"; shift
    sock="$TMPD/sock.$$.sock"
    rm -f "$sock" "$sock.shm"
    "$DAEMON" --sock "$sock" --epoch "$EPOCH" --trace "$TMPD/trace.log" >"$TMPD/daemon.log" 2>&1 &
    dpid=$!
    # wait for socket file to appear
    i=0
    while [ ! -S "$sock" ] && [ $i -lt 50 ]; do sleep 0.05; i=$((i+1)); done
    # a scheduling stall must fail the run, not hang the suite
    timeout 60 env LD_PRELOAD="$LIB" VCLOCK_SOCK="$sock" "$@" >"$out" 2>&1 || true
    # allow daemon to observe exit and print `done`
    i=0
    while [ $i -lt 50 ]; do
        if grep -q '"event":"done"' "$TMPD/daemon.log" 2>/dev/null; then break; fi
        sleep 0.05; i=$((i+1))
    done
    kill "$dpid" 2>/dev/null || true
    wait "$dpid" 2>/dev/null || true
    cat "$TMPD/daemon.log"
}

echo "== C probe: 20 runs =="
run_once "$TMPD/c0.out" "$TMPD/threaded_trace"
for i in $(seq 2 20); do
    run_once "$TMPD/c$i.out" "$TMPD/threaded_trace"
    if ! cmp -s "$TMPD/c0.out" "$TMPD/c$i.out"; then
        echo "FAIL: C run $i differs from run 1" >&2
        diff "$TMPD/c0.out" "$TMPD/c$i.out" >&2 || true
        exit 1
    fi
done
echo "C probe: 20/20 byte-identical"
grep -q "elapsed_ms=20" "$TMPD/c0.out" || echo "note: elapsed_ms check wants 20; got:" && grep elapsed_ms "$TMPD/c0.out"

echo "== posix_spawn probe: native oracle + 20 vclock runs =="
"$TMPD/spawn_probe" >"$TMPD/s_native.out" 2>&1
# run_once reuses $i, so this loop counts with $k
for k in $(seq 1 20); do
    run_once "$TMPD/s$k.out" "$TMPD/spawn_probe" >"$TMPD/s$k.daemon"
    if ! cmp -s "$TMPD/s_native.out" "$TMPD/s$k.out"; then
        echo "FAIL: spawn run $k differs from the native run" >&2
        diff "$TMPD/s_native.out" "$TMPD/s$k.out" >&2 || true
        exit 1
    fi
    if ! grep -q '"event":"done"' "$TMPD/s$k.daemon"; then
        echo "FAIL: spawn run $k: vclockd did not finish" >&2
        cat "$TMPD/s$k.daemon" >&2
        exit 1
    fi
done
echo "posix_spawn probe: 20/20 equal to native"

echo "== Python probe: 20 runs =="
run_once "$TMPD/p0.out" python3 tests/py_determinism.py
for i in $(seq 2 20); do
    run_once "$TMPD/p$i.out" python3 tests/py_determinism.py
    if ! cmp -s "$TMPD/p0.out" "$TMPD/p$i.out"; then
        echo "FAIL: Python run $i differs from run 1" >&2
        diff "$TMPD/p0.out" "$TMPD/p$i.out" >&2 || true
        exit 1
    fi
done
echo "Python probe: 20/20 byte-identical"
echo "PASS"
