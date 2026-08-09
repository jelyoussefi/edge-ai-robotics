# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Platform telemetry. Like the web console, deliberately tiny: it opens files
# under /proc and /sys and puts numbers on the bus. No qmassa, no
# intel_gpu_top, no libdrm -- those tools read the very same sysfs attributes
# this reads, so depending on one would add a parser and a thing that can be
# missing without adding a measurement.
#
# It runs as root, which it needs: RAPL's energy_uj is mode 0400 root:root
# (tightened after CVE-2020-8694, the power side-channel), so package power is
# unreadable to anyone else. Everything it touches is mounted read-only and it
# publishes to the bus like any other service.
FROM python:3.12-slim

RUN pip install --no-cache-dir pyzmq msgpack

COPY common /opt/edgebot
COPY services/telemetry/ /app/
WORKDIR /app
ENV PYTHONPATH=/opt/edgebot
ENTRYPOINT ["python3", "/app/collector.py"]
