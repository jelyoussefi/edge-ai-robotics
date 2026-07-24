# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.12-slim

COPY services/teleop/requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

COPY common /opt/edgebot
RUN pip install --no-cache-dir -e /opt/edgebot

WORKDIR /app
COPY services/teleop/ .

CMD ["python", "teleop.py"]
