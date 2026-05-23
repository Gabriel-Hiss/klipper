#!/bin/sh
set -eu

OUTDIR="${OUTDIR:-out_mips_nan2008_nossp}"
SYSROOT_LIB="${SYSROOT_LIB:-pad_sysroot/lib}"
CROSS_PREFIX="${CROSS_PREFIX:-mipsel-linux-gnu-}"
CC="${CROSS_PREFIX}gcc"

CFLAGS_COMMON="-iquote ${OUTDIR}/ -iquote src -iquote ${OUTDIR}/board-generic/ \
-std=gnu11 -O2 -MD -Wall -Wold-style-definition -Wtype-limits \
-ffunction-sections -fdata-sections -fno-delete-null-pointer-checks \
-flto=auto -fwhole-program -fno-use-linker-plugin -ggdb3 \
-mnan=2008 -mfp64 -D_TIME_BITS=32 -D_FILE_OFFSET_BITS=32 \
-fno-stack-protector"

mkdir -p "${OUTDIR}"

"${CC}" ${CFLAGS_COMMON} -c scripts/mips_nan2008_start.c \
    -o "${OUTDIR}/mips_nan2008_start.o"
"${CC}" ${CFLAGS_COMMON} -c scripts/mips_nan2008_compat.c \
    -o "${OUTDIR}/mips_nan2008_compat.o"

LDFLAGS_COMMON="${CFLAGS_COMMON} -Wl,--gc-sections -Wl,-e,_start \
-Wl,--dynamic-linker=/lib/ld-linux-mipsn8.so.1 \
-Wl,--allow-shlib-undefined -Wl,-rpath-link,${SYSROOT_LIB} \
-nostdlib ${OUTDIR}/mips_nan2008_start.o ${OUTDIR}/mips_nan2008_compat.o \
${SYSROOT_LIB}/libc-2.29.so -Wl,--no-as-needed ${SYSROOT_LIB}/libgcc_s.so.1"

if [ -e "${OUTDIR}/compile_time_request.o" ]; then
    FORCE_RELINK="-W ${OUTDIR}/compile_time_request.o"
else
    FORCE_RELINK=""
fi

make V=1 OUT="${OUTDIR}/" CROSS_PREFIX="${CROSS_PREFIX}" ${FORCE_RELINK} \
    CFLAGS="${CFLAGS_COMMON}" CFLAGS_klipper.elf="${LDFLAGS_COMMON}"

mkdir -p out
cp "${OUTDIR}/klipper.elf" out/klipper.elf
cp "${OUTDIR}/klipper.dict" out/klipper.dict
