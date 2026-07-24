# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.12-slim

# Mesa and GLFW for on-screen rendering through the host X socket, plus the
# Intel media driver so the iGPU is used rather than llvmpipe. The media driver
# lives in the non-free component and was renamed between Debian releases, so
# enable non-free and try both names, tolerating either being absent. The
# viewer still works without it; VA-API encode just falls back.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglfw3 \
        libglx-mesa0 \
        libegl1 \
        libxinerama1 \
        libxcursor1 \
        libxi6 \
        libxrandr2 \
        libgl1-mesa-dri \
        mesa-utils \
    && ( echo 'deb http://deb.debian.org/debian trixie non-free non-free-firmware' \
            > /etc/apt/sources.list.d/non-free.list && apt-get update ) \
    && ( apt-get install -y --no-install-recommends intel-media-va-driver \
         || apt-get install -y --no-install-recommends intel-media-va-driver-non-free \
         || echo "media driver unavailable, continuing without VA-API" ) \
    && rm -rf /var/lib/apt/lists/*

COPY services/viewer/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/viewer/ .

CMD ["python", "viewer.py"]
