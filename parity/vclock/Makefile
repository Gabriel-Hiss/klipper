# vclock build: x86_64 daemon + preload, MIPS preload for qemu-user guests.
# x86_64:
#   make x86_64
# MIPS (NaN2008 FP64 like scripts/build-mips-nan2008-klipper.sh):
#   make mips CROSS_PREFIX=mipsel-linux-gnu- SYSROOT_LIB=../../pad_sysroot/lib
CC ?= gcc
CFLAGS := -std=gnu11 -O2 -Wall -Wextra -fPIC -fno-omit-frame-pointer
SRC := src
BUILD_X := build/x86_64
BUILD_M := build/mips

.PHONY: all x86_64 mips clean
all: x86_64

x86_64:
	mkdir -p $(BUILD_X)
	$(CC) $(CFLAGS) -o $(BUILD_X)/vclockd $(SRC)/vclockd.c
	$(CC) $(CFLAGS) -shared -o $(BUILD_X)/libvclock.so $(SRC)/libvclock.c -ldl -lpthread

mips:
	mkdir -p $(BUILD_M)
	$(CURDIR)/scripts/build-mips.sh

clean:
	rm -rf build
