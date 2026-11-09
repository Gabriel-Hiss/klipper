#!/bin/sh
# Linker wrapper for the KE target. rustc writes its generated symbols.o
# (exported-symbol references, no code) without the EF_MIPS_NAN2008 ELF
# flag, and as the first input it makes ld reject every NaN2008 object that
# follows. Mark it NaN2008 like the rest, then link with the cross gcc.
for arg in "$@"; do
    case "$arg" in
        @*) files=$(tr '\n' ' ' < "${arg#@}") ;;
        *) files=$arg ;;
    esac
    for f in $files; do
        f=${f#\"}; f=${f%\"}
        case "$f" in
            */symbols.o)
                python3 - "$f" <<'EOF'
import struct, sys
path = sys.argv[1]
with open(path, 'r+b') as fh:
    hdr = fh.read(0x28)
    assert hdr[:4] == b'\x7fELF' and hdr[4] == 1 and hdr[5] == 1, path
    flags = struct.unpack_from('<I', hdr, 0x24)[0]
    fh.seek(0x24)
    fh.write(struct.pack('<I', flags | 0x400))  # EF_MIPS_NAN2008
EOF
                ;;
        esac
    done
done
exec mipsel-linux-gnu-gcc "$@"
