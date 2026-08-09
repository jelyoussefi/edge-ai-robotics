# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Thin ZeroMQ wrapper shared by every service.

Publishers connect to the broker's XSUB socket, subscribers to its XPUB socket.
Nothing binds except the broker itself, so services can start in any order and
new ones can join without touching the existing wiring.
"""

from __future__ import annotations

import os
from typing import Any, Iterable

import msgpack
import zmq

BUS_PUB = os.environ.get("BUS_PUB", "tcp://bus:5555")
BUS_SUB = os.environ.get("BUS_SUB", "tcp://bus:5556")


class Publisher:
    """Sends messages to the broker."""

    def __init__(self, addr: str = BUS_PUB) -> None:
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        # A send buffer of only 16 messages is far too small once large camera
        # frames (tens of KB, 15/s) share the socket with small telemetry: the
        # buffer fills and NOBLOCK then silently drops every frame after the
        # first, so a subscriber sees one frozen image. Give it real headroom.
        self._sock.setsockopt(zmq.SNDHWM, 256)
        self._sock.connect(addr)

    def send(self, topic: str, payload: dict[str, Any]) -> None:
        # Doubles, NOT single floats. `use_single_float=True` used to be set
        # here to save bytes, and it silently destroyed every timestamp on the
        # bus: a float32 has 24 bits of mantissa, so around a Unix epoch of
        # 1.79e9 its ULP is 2^31 / 2^24 = 128 SECONDS. Every `time.time()` on
        # every topic was being snapped to a 128 s grid.
        #
        # What that broke, all from the one line: the web console's MJPEG
        # stream sent a client exactly one frame and then went quiet, because
        # it only writes frames whose stamp differs from the last one it sent
        # and the stamp did not change for two minutes at a time; frame ages
        # sawtoothed between -64 s and +64 s; and no end-to-end latency could
        # be measured at all. The saving it bought was nothing measurable --
        # payloads here are dominated by JPEG and by raw byte blobs (SUITE_CLOUD
        # packs its points itself precisely so they do not go through msgpack
        # as floats), so the float64s are a handful of bytes per message.
        self._sock.send_multipart(
            [topic.encode(), msgpack.packb(payload)],
            flags=zmq.NOBLOCK,
        )

    def close(self) -> None:
        self._sock.close(linger=0)


class Subscriber:
    """Receives messages from the broker.

    `recv` is non-blocking by default so a caller running a fixed-rate loop can
    poll it without ever stalling the loop.
    """

    def __init__(self, topics: Iterable[str], addr: str = BUS_SUB,
                 rcvhwm: int = 256) -> None:
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        # A low receive high-water mark makes ZeroMQ drop the OLDEST queued
        # messages when the buffer is full, so a slow consumer of a high-rate
        # stream (camera frames) naturally keeps the freshest frames instead of
        # falling behind on a growing backlog. Small messages use the default.
        self._sock.setsockopt(zmq.RCVHWM, rcvhwm)
        self._sock.setsockopt(zmq.CONFLATE, 0)
        for topic in topics:
            self._sock.setsockopt(zmq.SUBSCRIBE, topic.encode())
        self._sock.connect(addr)

    def recv(self, timeout_ms: int = 0) -> tuple[str, dict[str, Any]] | None:
        """Return the next message, or None if nothing arrived within timeout."""
        if not self._sock.poll(timeout_ms):
            return None
        topic, body = self._sock.recv_multipart()
        return topic.decode(), msgpack.unpackb(body, raw=False)

    def drain(self) -> tuple[str, dict[str, Any]] | None:
        """Return only the most recent message, discarding any backlog.

        Control loops want the freshest command, not a queue of stale ones.
        """
        latest = None
        while (msg := self.recv(0)) is not None:
            latest = msg
        return latest

    def close(self) -> None:
        self._sock.close(linger=0)
