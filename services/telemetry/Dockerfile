# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Platform telemetry: CPU/GPU/NPU load, and package power from Intel PCM.
#
# Two tools are built from source here, both pinned, and neither is available
# any other way on this board:
#
#   pcm     Debian's `pcm` package (202502-1) REFUSES this CPU outright --
#           "Error: unsupported processor ... CPU family 6 model number 204
#           Brand: Intel(R) Core(TM) Ultra X7 358H". It exits rc=1 before
#           touching an MSR, so no amount of capability granting helps.
#           Upstream added Panther Lake as PTL = PCM_CPU_FAMILY_MODEL(6, 204)
#           (src/cpucounters.h), so it is built from the pinned commit below.
#           This is the whole reason for a source build: the packaged binary
#           cannot measure this machine at all.
#
#   qmassa  Not packaged anywhere -- not in apt, not on crates.io, only the
#           upstream Rust repository. Used for per-engine GPU busy, which is
#           the max across render/blitter/video/compute engines rather than
#           the complement of the GT idle residency that sysfs offers.
#
# Both are pinned to a commit, like SUITE_COMMIT for the ROS bricks and for the
# same reason: a telemetry panel that changes what it reports because an
# upstream default moved is worse than one that does not build.
#
# The runtime image stays small -- the two builders contribute one binary each
# and none of their toolchains.

FROM debian:trixie-slim AS pcm-build
ARG PCM_COMMIT=4246ea47f27c471bcc8484c065886f31f449933e
RUN apt-get update && apt-get install -y --no-install-recommends \
        git cmake g++ make ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/intel/pcm.git /src/pcm \
    && git -C /src/pcm checkout -q ${PCM_COMMIT} \
    && git -C /src/pcm submodule update --init --recursive --depth 1
RUN cmake -S /src/pcm -B /src/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /src/build -j"$(nproc)" --target pcm

FROM rust:slim AS qmassa-build
ARG QMASSA_COMMIT=9ebccca310c52236920fd30bc17235b5d93a9d09
RUN apt-get update && apt-get install -y --no-install-recommends \
        git pkg-config libdrm-dev libudev-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/ulissesf/qmassa.git /src/qmassa \
    && git -C /src/qmassa checkout -q ${QMASSA_COMMIT}
# The repository root is a VIRTUAL manifest (a cargo workspace), so
# `cargo install --path .` refuses it: "found a virtual manifest ... instead of
# a package manifest". Build the workspace and take the binary out of target/.
RUN cargo build --release --locked --manifest-path /src/qmassa/Cargo.toml

FROM python:3.12-slim

# libcap2-bin only for setcap at build time; kept because `getcap` is the one
# way to check from inside a running container that the caps actually stuck.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libcap2-bin libdrm2 libudev1 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir pyzmq msgpack

COPY --from=pcm-build /src/build/bin/pcm /usr/local/sbin/pcm
COPY --from=qmassa-build /src/qmassa/target/release/qmassa /usr/local/bin/qmassa

# File capabilities, as the reference does. These only take effect if the
# container's BOUNDING set already contains them -- a file capability can grant
# what the bounding set permits and nothing more -- so docker-compose.yml adds
# cap_add: SYS_RAWIO, SYS_ADMIN, DAC_OVERRIDE alongside. Without the cap_add,
# execve of a +ep binary fails outright with "Operation not permitted" and the
# error names neither capabilities nor the bounding set. Measured that way once.
RUN setcap cap_sys_rawio,cap_sys_admin,cap_dac_override+ep /usr/local/sbin/pcm \
    && getcap /usr/local/sbin/pcm

COPY common /opt/edgebot
COPY services/telemetry/ /app/
WORKDIR /app
ENV PYTHONPATH=/opt/edgebot
ENTRYPOINT ["python3", "/app/collector.py"]
