#!/bin/sh
# Build build/mips/libvclock.so with the same NaN2008 FP64 flags as
# scripts/build-mips-nan2008-klipper.sh (repo root).
set -eu
CROSS_PREFIX="${CROSS_PREFIX:-mipsel-linux-gnu-}"
SYSROOT_LIB="${SYSROOT_LIB:-pad_sysroot/lib}"
CC="${CROSS_PREFIX}gcc"
mkdir -p build/mips
"$CC" -std=gnu11 -O2 -Wall -Wextra -fPIC -shared -mnan=2008 -mfp64 \
  -D_TIME_BITS=32 -D_FILE_OFFSET_BITS=32 \
  -o build/mips/libvclock.so src/libvclock.c -ldl -lpthread
echo "built build/mips/libvclock.so (CROSS_PREFIX=$CROSS_PREFIX SYSROOT_LIB=$SYSROOT_LIB)"
