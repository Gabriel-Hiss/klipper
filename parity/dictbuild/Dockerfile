# Builds the MCU data dictionaries named by test/configs/*.config (same toolchains as scripts/ci-install.sh).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential python3 git ca-certificates wget xz-utils libffi-dev \
      gcc-avr avr-libc libnewlib-arm-none-eabi gcc-arm-none-eabi binutils-arm-none-eabi \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/tc && cd /opt/tc \
 && wget -q https://github.com/dinuxbg/gnupru/releases/download/2024.05/pru-elf-2024.05.amd64.tar.xz \
 && tar xJf pru-elf-2024.05.amd64.tar.xz && rm pru-elf-2024.05.amd64.tar.xz \
 && wget -q https://github.com/openrisc/or1k-gcc/releases/download/or1k-12.0.1-20220210-20220304/or1k-elf-12.0.1-20220210-20220304.tar.xz \
 && tar xJf or1k-elf-12.0.1-20220210-20220304.tar.xz && rm or1k-elf-12.0.1-20220210-20220304.tar.xz
ENV PATH=/opt/tc/pru-elf/bin:/opt/tc/or1k-elf/bin:$PATH

COPY build.sh /usr/local/bin/build-dicts
RUN chmod +x /usr/local/bin/build-dicts
WORKDIR /work
