# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.12-slim

# Mesa and GLFW for offscreen rendering, plus the X libraries that OpenCV's Qt
# platform plugin (used by cv2.imshow) needs to load. The wheel bundles its own
# Qt, but the xcb plugin links against these system libraries at load time, and
# a missing one makes cv2.imshow crash the process (SIGSEGV, "could not load the
# Qt platform plugin xcb"). This is the full documented set for the plugin.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        libusb-1.0-0 \
        libglfw3 \
        libglx-mesa0 \
        libegl1 \
        libglvnd0 \
        libopengl0 \
        libgles2 \
        libegl-mesa0 \
        libgl1-mesa-dri \
        libglu1-mesa \
        mesa-utils \
        libxinerama1 \
        libxcursor1 \
        libxi6 \
        libxrandr2 \
        libxrender1 \
        libxext6 \
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
    && ( echo 'deb http://deb.debian.org/debian trixie non-free non-free-firmware' \
            > /etc/apt/sources.list.d/non-free.list && apt-get update ) \
    && ( apt-get install -y --no-install-recommends intel-media-va-driver \
         || apt-get install -y --no-install-recommends intel-media-va-driver-non-free \
         || echo "media driver unavailable, continuing without VA-API" ) \
    && rm -rf /var/lib/apt/lists/*

COPY services/compositor/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/compositor/ .

# VIEWER_MODE=backdrop runs the compositing viewer (robot over camera),
# anything else runs the default interactive viewer.
CMD ["python", "compositor.py"]
