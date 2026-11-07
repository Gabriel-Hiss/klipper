#!/bin/sh
# Build klippy/chelper/c_helper.so for the printer exactly as config/ender-3-v3-ke/README.md does.
# Usage: build-chelper.sh CHELPER_DIR SYSROOT_LIB
set -eu
cd "$1"
SYSROOT_LIB=$2
mipsel-linux-gnu-gcc -Wall -g -O2 -shared -fPIC \
  -mnan=2008 \
  -D_TIME_BITS=32 -D_FILE_OFFSET_BITS=32 \
  -nostdlib -nostartfiles \
  -Wl,--as-needed \
  -static-libgcc \
  -flto -fwhole-program -fno-use-linker-plugin \
  -o _temp_c_helper.so \
  pyhelper.c serialqueue.c stepcompress.c steppersync.c itersolve.c \
  trapq.c pollreactor.c msgblock.c trdispatch.c \
  kin_cartesian.c kin_corexy.c kin_corexz.c kin_delta.c kin_deltesian.c \
  kin_polar.c kin_rotary_delta.c kin_winch.c kin_extruder.c kin_shaper.c \
  kin_idex.c kin_generic.c \
  "$SYSROOT_LIB/libc-2.29.so" \
  "$SYSROOT_LIB/libgcc_s.so.1"
mv _temp_c_helper.so c_helper.so
