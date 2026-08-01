# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# Ubuntu 24.04 with the Intel GPU AND NPU compute runtimes, so the RL locomotion
# policy can run on either via OpenVINO (OV_DEVICE=GPU or NPU). The NPU part
# mirrors services/perception/Dockerfile: without the driver installed here,
# OV_DEVICE=NPU compiled nothing and fell back to the CPU, which is where the
# policy had quietly been running all along.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG NPU_DRIVER_VERSION=1.35.0
ARG NPU_DRIVER_BUILD=v1.35.0.20260722-29947505341-ubuntu2404

RUN apt-get update && apt-get install -y --no-install-recommends \
      wget curl ca-certificates gnupg software-properties-common \
      python3 python3-pip \
      libgomp1 libtbb12 ocl-icd-libopencl1 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Intel graphics PPA: OpenCL runtime. libze1 is NOT taken from here: it is
# pinned below to the exact version the NPU driver expects, and a mismatched
# loader breaks the NPU while leaving the GPU working, which is hard to spot.
RUN add-apt-repository -y ppa:kobuk-team/intel-graphics \
    && apt-get update && apt-get install -y --no-install-recommends --allow-unauthenticated \
      libze-intel-gpu1 libze-dev \
      intel-opencl-icd intel-ocloc clinfo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Level Zero loader v1.28.2, the version validated with NPU driver 1.35.0.
RUN wget -q -O /tmp/libze1.deb \
      https://snapshot.ppa.launchpadcontent.net/kobuk-team/intel-graphics/ubuntu/20260606T100000Z/pool/main/l/level-zero-loader/libze1_1.28.2-1~24.04~ppa1_amd64.deb \
    && (dpkg -i /tmp/libze1.deb || apt-get update && apt-get install -y -f --no-install-recommends) \
    && rm -f /tmp/libze1.deb

# Intel NPU driver: compiler + firmware + Level Zero NPU backend.
RUN wget -q -O /tmp/npu.tgz \
      https://github.com/intel/linux-npu-driver/releases/download/v${NPU_DRIVER_VERSION}/linux-npu-driver-${NPU_DRIVER_BUILD}.tar.gz \
    && mkdir -p /tmp/npu && tar -xf /tmp/npu.tgz -C /tmp/npu \
    && dpkg -i /tmp/npu/intel-driver-compiler-npu*.deb \
               /tmp/npu/intel-fw-npu*.deb \
               /tmp/npu/intel-level-zero-npu*.deb \
    && rm -rf /tmp/npu /tmp/npu.tgz

RUN ldconfig -p | grep -q libze_loader || { echo "ERROR: libze_loader missing"; exit 1; } \
    && ldconfig -p | grep -q libOpenCL  || { echo "ERROR: libOpenCL missing";  exit 1; } \
    && echo "[OK] GPU + NPU + Level Zero runtimes installed"

COPY services/sim/requirements.txt /tmp/
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip3 install --break-system-packages --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/sim/ .

CMD ["python3", "sim.py"]
