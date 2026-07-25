# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.12-slim

# libgl and libglib are needed by OpenCV even in headless builds for codecs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY services/perception/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/perception/ .

CMD ["python", "perception.py"]
