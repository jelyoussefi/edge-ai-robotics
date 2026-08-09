# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# The web console. Deliberately the smallest image in the project: it holds no
# model, no GL, no ROS and no OpenCV -- it forwards JPEGs the compositor has
# already encoded and reads a few numbers off the bus. Anything heavier here
# would be a second place where frames are produced.
FROM python:3.12-slim

RUN pip install --no-cache-dir aiohttp==3.10.5 pyzmq msgpack

COPY common /opt/edgebot
COPY services/web/ /app/
WORKDIR /app
ENV PYTHONPATH=/opt/edgebot
ENTRYPOINT ["python3", "/app/app.py"]
