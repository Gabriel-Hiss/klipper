# x86_64 parity oracle: glibc 2.29 (Ubuntu 19.04) with the printer's Python stack.
FROM ubuntu:19.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=UTC
RUN sed -i -e 's|http://archive.ubuntu.com|http://old-releases.ubuntu.com|g' \
           -e 's|http://security.ubuntu.com|http://old-releases.ubuntu.com|g' /etc/apt/sources.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential ca-certificates curl wget git pkg-config \
      libffi-dev zlib1g-dev libssl-dev libbz2-dev libreadline-dev libsqlite3-dev \
      liblzma-dev libncurses5-dev libgdbm-dev uuid-dev \
      gfortran libopenblas-dev liblapack-dev \
      lcov gcovr \
 && rm -rf /var/lib/apt/lists/*

# CPython 3.8.2, the printer's interpreter version.
ARG PY_VERSION=3.8.2
RUN curl -fsSL https://www.python.org/ftp/python/${PY_VERSION}/Python-${PY_VERSION}.tgz | tar xz -C /tmp \
 && cd /tmp/Python-${PY_VERSION} \
 && ./configure --prefix=/opt/py38 --enable-shared LDFLAGS=-Wl,-rpath,/opt/py38/lib \
 && make -j"$(nproc)" && make install \
 && rm -rf /tmp/Python-${PY_VERSION}

# Printer package set (klippy-env on the pad).
RUN /opt/py38/bin/python3.8 -m venv /opt/klippy-env \
 && /opt/klippy-env/bin/pip install --no-cache-dir 'pip==20.0.2' 'setuptools==45.2.0' 'wheel==0.34.2' \
 && /opt/klippy-env/bin/pip install --no-cache-dir --no-binary greenlet,cffi \
      'greenlet==0.4.15' 'cffi==1.13.1' 'Jinja2==2.11.3' 'MarkupSafe==1.1.1' 'pyserial==3.4' \
 && /opt/klippy-env/bin/pip install --no-cache-dir --no-binary numpy 'Cython==0.29.14' 'numpy==1.16.4' \
 && /opt/klippy-env/bin/pip install --no-cache-dir 'coverage==5.5'

# Full profile: adds the optional packages the printer lacks, for code paths that need them.
RUN /opt/py38/bin/python3.8 -m venv --system-site-packages /opt/klippy-env-full \
 && cp -a /opt/klippy-env/lib/python3.8/site-packages/. /opt/klippy-env-full/lib/python3.8/site-packages/ \
 && /opt/klippy-env-full/bin/pip install --no-cache-dir 'scipy==1.3.3' 'python-can==3.3.4' 'msgspec==0.18.6' || \
    /opt/klippy-env-full/bin/pip install --no-cache-dir 'scipy==1.3.3' 'python-can==3.3.4'

# Rust toolchain; the pinned channel comes from klippy-rs/rust-toolchain.toml at build time.
ENV RUSTUP_HOME=/opt/rustup CARGO_HOME=/opt/cargo PATH=/opt/cargo/bin:/opt/klippy-env/bin:$PATH
RUN curl -fsSL https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain none \
 && chmod -R a+rwX /opt/rustup /opt/cargo

RUN /opt/klippy-env/bin/python -c "import sys, greenlet, cffi, jinja2, markupsafe, serial, numpy; \
print(sys.version.split()[0], greenlet.__version__, cffi.__version__, jinja2.__version__, markupsafe.__version__, serial.VERSION, numpy.__version__)" \
 && ldd --version | head -1

WORKDIR /work
