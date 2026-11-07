#!/bin/bash
# Usage: build-dicts SRC_REPO OUT_DIR
# Builds every test/configs/*.config from a private copy of SRC_REPO and writes OUT_DIR/<name>.dict.
set -eu
SRC=$1
OUT=$2
mkdir -p "$OUT"
mkdir -p /work/tree
for d in Makefile src lib scripts test klippy; do cp -a "$SRC/$d" /work/tree/; done
cd /work/tree
grep -rIl $'\r' . | xargs -r sed -i 's/\r$//'
for TARGET in test/configs/*.config; do
    NAME=$(basename "$TARGET" .config)
    make clean >/dev/null 2>&1 || true
    make distclean >/dev/null 2>&1 || true
    cp "$TARGET" .config
    make olddefconfig >/dev/null
    if make -j"$(nproc)" >"/work/$NAME.log" 2>&1; then
        cp out/klipper.dict "$OUT/$NAME.dict"
        echo "ok $NAME"
    else
        echo "FAIL $NAME (see /work/$NAME.log)"
        tail -20 "/work/$NAME.log"
        exit 1
    fi
done
