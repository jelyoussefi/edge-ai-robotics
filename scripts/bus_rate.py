#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Measure what the bus actually carries, per topic, in bytes and messages.

Written for the 480p -> 720p move: raising the source resolution raises the
JPEG on CAMERA_RGB, the Z16 blob on CAMERA_DEPTH and the composited JPEG on
COMPOSITED_FRAME all at once, and "it feels heavier" is not a number.

This goes UNDER edgebot.bus on purpose. Subscriber.recv() unpacks msgpack and
hands back a dict, which is exactly what a byte meter must not do: the question
is how many bytes crossed the wire, not how large the decoded object is. So it
opens its own SUB socket, subscribes to everything with an empty prefix, and
weighs the frames before touching them.

The rate reported is per SUBSCRIBER. Every subscriber gets its own copy from the
broker's XPUB, so a topic with three consumers costs three times this on the
loopback -- the number here is what one consumer pays.
"""
from __future__ import annotations

import argparse
import collections
import os
import time

import zmq

BUS_SUB = os.environ.get("BUS_SUB", "tcp://bus:5556")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--addr", default=BUS_SUB)
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    # Big HWM: this is a meter, and a meter that drops what it is measuring
    # reports a rate lower than the truth and gives no sign it did so.
    sock.setsockopt(zmq.RCVHWM, 20000)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(args.addr)

    n = collections.Counter()
    b = collections.Counter()
    # Discard the first moment: connecting a new subscriber makes the broker
    # start forwarding mid-frame, and the partial first second reads as a dip.
    time.sleep(0.5)
    while sock.poll(0):
        sock.recv_multipart()

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < args.seconds:
        if not sock.poll(200):
            continue
        parts = sock.recv_multipart()
        topic = parts[0].decode()
        n[topic] += 1
        b[topic] += sum(len(p) for p in parts)
    dt = time.perf_counter() - t0

    print(f"over {dt:.1f} s, per subscriber:")
    print(f"{'topic':<24}{'msg/s':>9}{'MB/s':>9}{'kB/msg':>9}")
    for topic in sorted(b, key=lambda k: -b[k]):
        print(f"{topic:<24}{n[topic] / dt:>9.1f}{b[topic] / dt / 1e6:>9.3f}"
              f"{b[topic] / n[topic] / 1e3:>9.1f}")
    print(f"{'TOTAL':<24}{sum(n.values()) / dt:>9.1f}"
          f"{sum(b.values()) / dt / 1e6:>9.3f}")
    sock.close(linger=0)


if __name__ == "__main__":
    main()
