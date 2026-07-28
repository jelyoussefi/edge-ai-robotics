# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# Ubuntu 24.04 with the Intel GPU compute runtime, so the RL locomotion policy
# runs on the integrated GPU via OpenVINO (OV_DEVICE=GPU). Follows Intel's recipe.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      wget curl ca-certificates gnupg software-properties-common \
      python3 python3-pip \
      libgomp1 libtbb12 ocl-icd-libopencl1 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Intel graphics PPA: Level Zero + OpenCL for the OpenVINO GPU plugin.
RUN add-apt-repository -y ppa:kobuk-team/intel-graphics \
    && apt-get update && apt-get install -y --no-install-recommends --allow-unauthenticated \
      libze-intel-gpu1 libze1 libze-dev \
      intel-opencl-icd intel-ocloc clinfo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN ldconfig -p | grep -q libze_loader || { echo "ERROR: libze_loader missing"; exit 1; } \
    && ldconfig -p | grep -q libOpenCL  || { echo "ERROR: libOpenCL missing";  exit 1; } \
    && echo "[OK] GPU + Level Zero runtimes installed"

COPY services/sim/requirements.txt /tmp/
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip3 install --break-system-packages --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/sim/ .

CMD ["python3", "sim.py"]
