# Printer userland snapshot

`rootfs.tar.gz` is a read-only copy taken over SSH from the Ender-3 V3 KE (Nebula pad, `192.168.0.24`) on 2026-11-06. It contains `lib`, `usr/lib` (glibc 2.29, Python 3.8.2 and the system site-packages with greenlet 0.4.15, cffi 1.13.1, numpy 1.16.4, pyserial 3.4), `usr/bin/python3*`, `usr/share/klippy-env` (Jinja2 2.11.3, MarkupSafe 1.1.1), the deployed `usr/data/klipper-mainline/klippy/chelper/c_helper.so` (md5 `f5bd3e54d3c8dbb2b9af87f0c87da2fe`) and `usr/data/klipper_mcu_new` (md5 `aef386c333fddd72dd87377a948aa055`), and the configuration trees `usr/data/printer_data/config`, `usr/data/printer_data/config-mainline` and `usr/data/mainsail-config`.

The MIPS oracle (`parity/oracle-mips`) runs this userland under qemu. The x86 oracle mounts `usr/data` from it so absolute include paths in the printer configuration resolve.
