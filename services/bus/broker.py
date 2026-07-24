# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""XSUB/XPUB forwarder. The only service that binds."""

import logging

import zmq

logging.basicConfig(level=logging.INFO, format="%(asctime)s bus %(levelname)s %(message)s")

ctx = zmq.Context()
frontend = ctx.socket(zmq.XSUB)
frontend.bind("tcp://*:5555")
backend = ctx.socket(zmq.XPUB)
backend.setsockopt(zmq.XPUB_VERBOSE, 1)
backend.bind("tcp://*:5556")

logging.info("broker up: publishers on 5555, subscribers on 5556")
try:
    zmq.proxy(frontend, backend)
except KeyboardInterrupt:
    pass
finally:
    frontend.close()
    backend.close()
    ctx.term()
