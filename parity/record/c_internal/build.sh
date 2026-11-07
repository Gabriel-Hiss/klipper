#!/bin/sh
# Build the recording c_helper.so from the unchanged C sources.
#
# Usage: parity/record/c_internal/build.sh [output.so]
#
# The build differs from klippy/chelper/__init__.py check_build_c_library():
#   - no LTO (-flto -fwhole-program removed), so -Wl,--wrap= can see
#     individual symbols;
#   - SSE flags kept identical (-mfpmath=sse -msse2), and no other
#     optimization change, so arithmetic is bit-identical (see
#     README.md for the argument);
#   - recorder.c (the __wrap_* shims) is compiled in and the five
#     wrapped symbols are interposed.
#
# The output defaults to parity/record/build/c_helper_rec.so. Pass it
# to the driver as --c-helper, which sets KLIPPY_RECORD_C_HELPER, or
# point KLIPPY_RECORD_C_HELPER at it directly. The driver then wraps
# the lib returned by get_ffi() and loads this recording library via
# the cffi_recorder.activate() build-check patch; the frozen klippy/
# tree is never edited.
set -eu

HERE=$(dirname "$0")
REPO_ROOT=$(cd "$HERE/../../.." && pwd)
CHELPER="$REPO_ROOT/klippy/chelper"

OUT=${1:-"$REPO_ROOT/parity/record/build/c_helper_rec.so"}
mkdir -p "$(dirname "$OUT")"

SOURCES="pyhelper.c serialqueue.c stepcompress.c steppersync.c
itersolve.c trapq.c pollreactor.c msgblock.c trdispatch.c
kin_cartesian.c kin_corexy.c kin_corexz.c kin_delta.c
kin_deltesian.c kin_polar.c kin_rotary_delta.c kin_winch.c
kin_extruder.c kin_shaper.c kin_idex.c kin_generic.c"
SRC=""
for f in $SOURCES; do
    SRC="$SRC $CHELPER/$f"
done
SRC="$SRC $HERE/recorder.c"

gcc -Wall -g -O2 -shared -fPIC -mfpmath=sse -msse2 \
    -o "$OUT" $SRC \
    -Wl,--wrap=stepcompress_append \
    -Wl,--wrap=stepcompress_flush \
    -Wl,--wrap=stepcompress_history_expire \
    -Wl,--wrap=itersolve_generate_steps \
    -Wl,--wrap=serialqueue_send_batch

echo "built $OUT"
