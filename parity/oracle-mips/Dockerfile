# MIPS parity oracle: the printer's own userland (glibc 2.29, NaN2008, FP64) under qemu-user.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=UTC
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      qemu-user ca-certificates curl git make python3 \
      gcc-mipsel-linux-gnu binutils-mipsel-linux-gnu libc6-dev-mipsel-cross \
      build-essential \
 && rm -rf /var/lib/apt/lists/*

# Printer userland snapshot (see tests/fixtures/pad/README.md for provenance).
COPY rootfs.tar.gz /tmp/rootfs.tar.gz
RUN mkdir -p /pad && tar xzf /tmp/rootfs.tar.gz -C /pad && rm /tmp/rootfs.tar.gz \
 && ln -s /pad/usr/lib/python3.8 /usr/lib/python3.8 \
 && mkdir -p /usr/share && ln -s /pad/usr/share/klippy-env /usr/share/klippy-env \
 && cp /usr/mipsel-linux-gnu/include/gnu/stubs-o32_hard.h /usr/mipsel-linux-gnu/include/gnu/stubs-o32_hard_2008.h

ENV QEMU_LD_PREFIX=/pad PAD_PYTHON=/usr/share/klippy-env/bin/python3 KLIPPY_PYTHON=/usr/local/bin/pad-python \
    RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:$PATH
RUN curl -fsSL https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain none \
 && chmod -R a+rwX /opt/rustup /opt/cargo

COPY pad-python /usr/local/bin/pad-python
RUN chmod +x /usr/local/bin/pad-python \
 && pad-python -c "import sys, greenlet, cffi, jinja2, numpy, serial; print(sys.version.split()[0], greenlet.__version__, cffi.__version__, jinja2.__version__, numpy.__version__)"

WORKDIR /work
