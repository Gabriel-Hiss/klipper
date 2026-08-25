# Ender-3 V3 KE Mainline Setup on Nebula Pad

## Building Mainline Klipper for MIPS NaN2008

### Root the Pad First

Enable root access before installing the helper script or copying any mainline
files to the Pad.

Follow the rooted firmware/root access flow from the Guilouz wiki:
<https://guilouz.github.io/Creality-Helper-Script-Wiki/firmwares/install-and-update-rooted-firmware-ender3/>

After root access is enabled, connect over SSH as `root`.

### Ubuntu Build Prerequisites

Install the MIPS little-endian cross-compiler on Ubuntu (I used Ubuntu 24.04 WSL):

```bash
sudo apt install gcc-mipsel-linux-gnu
```

The Ubuntu cross toolchain may not ship the NaN2008 hard-float stub header.
Create the missing header before building:

```bash
sudo cp /usr/mipsel-linux-gnu/include/gnu/stubs-o32_hard.h \
        /usr/mipsel-linux-gnu/include/gnu/stubs-o32_hard_2008.h
```

### Build C Helper

Run this from `klippy/chelper/`:

```bash
cd <repo>/klippy/chelper

mipsel-linux-gnu-gcc -Wall -g -O2 -shared -fPIC \
  -mnan=2008 \
  -D_TIME_BITS=32 -D_FILE_OFFSET_BITS=32 \
  -nostdlib -nostartfiles \
  -Wl,--as-needed \
  -static-libgcc \
  -flto -fwhole-program -fno-use-linker-plugin \
  -o c_helper.so \
  pyhelper.c serialqueue.c stepcompress.c steppersync.c itersolve.c \
  trapq.c pollreactor.c msgblock.c trdispatch.c \
  kin_cartesian.c kin_corexy.c kin_corexz.c kin_delta.c kin_deltesian.c \
  kin_polar.c kin_rotary_delta.c kin_winch.c kin_extruder.c kin_shaper.c \
  kin_idex.c kin_generic.c \
  ../../pad_sysroot/lib/libc-2.29.so \
  ../../pad_sysroot/lib/libgcc_s.so.1
```

### Configure klipper_mcu

Before building `klipper_mcu`, run menuconfig and select `Linux process`:

```bash
cd <repo>
make menuconfig
```

### Build klipper_mcu

```bash
bash scripts/build-mips-nan2008-klipper.sh
```

The script writes the final MCU binary to `out/klipper.elf`.

### Copy Mainline Klipper to the Pad

Create the destination directory on the Pad first:

```bash
mkdir -p /usr/data/klipper-mainline/klippy
```

Copy the local `klippy/` tree from Ubuntu to the Pad:

```bash
scp -r klippy/ root@<IP>:/usr/data/klipper-mainline/klippy/
```

Fix the duplicated directory level on the Pad:

```bash
mv /usr/data/klipper-mainline/klippy/klippy/* /usr/data/klipper-mainline/klippy/
rmdir /usr/data/klipper-mainline/klippy/klippy
```

### Copy Built Binaries to the Pad

Copy `c_helper.so` and the new `klipper_mcu` binary:

```bash
scp klippy/chelper/c_helper.so root@<IP>:/usr/data/klipper-mainline/klippy/chelper/c_helper.so
scp out/klipper.elf root@<IP>:/usr/data/klipper_mcu_new
```

Make the MCU binary executable on the Pad:

```bash
chmod +x /usr/data/klipper_mcu_new
```

## Complete Setup

- Klipper mainline HEAD `616242d4` running on the Nebula Pad.
- Pad hardware: MIPS Ingenic xburst2, glibc 2.29, o32 NaN2008 ABI.
- Entware installed before using Guilouz Helper Script.
- Moonraker v0.10.0 and Mainsail v2.17.0 installed with Guilouz Helper Script.
- Mainsail is available at `http://<IP>:4409`.
- Moonraker config path on the Pad: `/usr/data/printer_data/config/moonraker.conf`.
- `klippy_uds_address` points to `/tmp/klippy_uds`, the binding used by Moonraker and the Creality `c440x` UI.

## Moonraker Configuration

Edit `/usr/data/printer_data/config/moonraker.conf` on the Pad and point
Moonraker at the mainline Klipper UDS socket:

```ini
klippy_uds_address: /tmp/klippy_uds
```

The mainline fork implements the Creality private webhook endpoints required by
the stock display and takes over this socket only after stock Klipper stops.

## Starting Mainline Klipper

Use 2 separate SSH sessions.

### Session 1 - stop stock Klipper and start the new klipper_mcu

```bash
/etc/init.d/S55klipper_service stop
sleep 2
/usr/data/klipper_mcu_new -r 2>&1 | tee /dev/null &
sleep 3
ls /tmp/klipper_host_mcu  # confirms that the socket was created
```

### Session 1 continued - start mainline klippy (stays in foreground)

Keep this process in the foreground with `tee`.

```bash
/usr/share/klippy-env/bin/python \
  /usr/data/klipper-mainline/klippy/klippy.py \
  /usr/data/printer_data/config-mainline/printer.cfg \
  -a /tmp/klippy_uds 2>&1 | tee /usr/data/klippy-mainline.log
```

### Session 2 - restart Moonraker

When the bed_mesh points appear in session 1, restart Moonraker:

```bash
/etc/init.d/S56moonraker_service restart
```

## Files on the Pad

- `/usr/data/klipper-mainline/klippy/` - mainline klippy tree.
- `/usr/data/klipper-mainline/klippy/chelper/c_helper.so` - built for MIPS NaN2008.
- `/usr/data/klipper_mcu_new` - `klipper_mcu` built for MIPS NaN2008.
- `/usr/data/printer_data/config-mainline/` - mainline configuration files.
- `/usr/data/mainsail-config/` - `mainsail-config` cloned from GitHub.

## Commented Sections in printer.cfg

- `#[include load_cell.cfg]` - PRTouch through HX711 requires reflashing the MCU with `CONFIG_WANT_HX71X=y`.
- `#[include input_shaper.cfg]` - requires an ADXL345 connected to the `mcu rpi`.
- `#[temperature_sensor mcu_temp]` - the GD32F303 does not expose MCU temperature with the stock firmware.

## Known Limitations

- PRTouch, including Z-offset through the load cell, does not work without reflashing the MCU through SWD/OpenOCD.
- Auto-start requires the delayed handover service documented in `SERVICES.md`; never disable the stock boot services.
- `klipper_mcu_new` must not replace `/usr/bin/klipper_mcu` directly, because that causes SSH boot failure.
- There is a CRC mismatch between the stock MCU firmware and mainline Klipper; this is currently worked around without reflashing.

## Guilouz Helper Script

The Pad setup used Guilouz Helper Script after installing Entware:

```bash
git clone --depth 1 https://github.com/Guilouz/Creality-Helper-Script.git /usr/data/helper-script
sh /usr/data/helper-script/helper.sh
```

Menu flow used:

In the Install Menu:

- Option `4`: install Entware first.
- Option `1`: install Moonraker and Nginx.
- Option `3`: install Mainsail on port `4409`.

### Others
Read the SERVICE.md to set up services for klipper
Read the CALIBRATE.md to calibrate the PRTouch