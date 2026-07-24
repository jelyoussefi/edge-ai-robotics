# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.12-slim

# Mesa and GLFW for on-screen rendering through the host X socket, plus the
# Intel media driver so the iGPU is used rather than llvmpipe.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglfw3 \
        libglx-mesa0 \
        libegl1 \
        libxinerama1 \
        libxcursor1 \
        libxi6 \
        libxrandr2 \
        mesa-utils \
        intel-media-va-driver-non-free \
    && rm -rf /var/lib/apt/lists/*

COPY services/viewer/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/viewer/ .

CMD ["python", "viewer.py"]
