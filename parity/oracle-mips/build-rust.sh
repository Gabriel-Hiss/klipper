#!/bin/bash
# Cross-builds a klippy-rs binary for the Ender-3 V3 KE (MIPS32 LE, NaN2008, FP64, glibc 2.29),
# linking against the printer's own libraries from the pad snapshot.
# Usage: build-rust.sh KLIPPY_RS_DIR PACKAGE
set -eu
SRC=$1
PKG=$2
PAD=${PAD:-/pad}
LINK=/tmp/padlink
REPO=$(cd "$SRC/.." && pwd)

rustup toolchain install 1.93.1 --profile minimal --component rust-src >/dev/null 2>&1 || true

mkdir -p "$LINK"
for spec in c:libc.so.6 m:libm.so.6 pthread:libpthread.so.0 dl:libdl.so.2 rt:librt.so.1 \
            util:libutil.so.1 gcc_s:libgcc_s.so.1; do
    ln -sf "$PAD/lib/${spec#*:}" "$LINK/lib${spec%%:*}.so"
done
CFLAGS="-O2 -mnan=2008 -mfp64 -fno-stack-protector -D_TIME_BITS=32 -D_FILE_OFFSET_BITS=32"
mipsel-linux-gnu-gcc $CFLAGS -c "$REPO/scripts/mips_nan2008_start.c" -o "$LINK/start.o"
mipsel-linux-gnu-gcc $CFLAGS -c "$REPO/parity/oracle-mips/rust_compat.c" -o "$LINK/rust_compat.o"

export RUSTC_BOOTSTRAP=1
export CARGO_TARGET_DIR=${CARGO_TARGET_DIR:-/target-mips}
export RUSTFLAGS="-C linker=$REPO/parity/oracle-mips/link.sh -C linker-flavor=gcc \
 -C link-arg=-nostartfiles -C link-arg=-mnan=2008 -C link-arg=-mfp64 \
 -C link-arg=-no-pie -C link-arg=-Wl,-e,_start \
 -C link-arg=$LINK/start.o -C link-arg=$LINK/rust_compat.o \
 -C link-arg=-L$LINK -C link-arg=-Wl,-rpath-link,$PAD/lib \
 -C link-arg=$PAD/lib/ld-linux-mipsn8.so.1 \
 -C link-arg=-Wl,--dynamic-linker=/lib/ld-linux-mipsn8.so.1"
cd "$SRC"
cargo +1.93.1 build --release -p "$PKG" \
    -Zbuild-std=std,panic_unwind --target targets/mipsel-ke-linux-gnu.json
