# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.11-slim

# libusb for the RealSense, plus the X libraries OpenCV's Qt platform plugin
# links against at load time. The plugin is only used by `make calibrate`, which
# runs in this container because this is the one that owns the camera, but the
# non-headless OpenCV wheel needs libGL even to encode a JPEG, so these are not
# optional once the wheel is the full one.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libusb-1.0-0 \
        libgl1 \
        libglib2.0-0 \
        libxext6 \
        libxrender1 \
        libsm6 \
        libice6 \
        libx11-xcb1 \
        libxcb1 \
        libxcb-glx0 \
        libxcb-keysyms1 \
        libxcb-image0 \
        libxcb-shm0 \
        libxcb-icccm4 \
        libxcb-sync1 \
        libxcb-xfixes0 \
        libxcb-shape0 \
        libxcb-randr0 \
        libxcb-render-util0 \
        libxcb-render0 \
        libxcb-util1 \
        libxcb-xinerama0 \
        libxcb-cursor0 \
        libxcb-xkb1 \
        libxkbcommon0 \
        libxkbcommon-x11-0 \
        libfontconfig1 \
        libdbus-1-3 \
    && rm -rf /var/lib/apt/lists/*

COPY services/source/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/source/ .

CMD ["python", "source.py"]
