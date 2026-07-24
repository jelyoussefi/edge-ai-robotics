# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
FROM python:3.12-slim

RUN pip install --no-cache-dir pyzmq==26.2.0

WORKDIR /app
COPY services/bus/broker.py .

CMD ["python", "broker.py"]
