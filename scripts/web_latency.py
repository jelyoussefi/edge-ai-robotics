#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""End-to-end latency of the web console's MJPEG stream, machine side.

The compositor burns `time.time()` into the bottom-left of every frame AND
carries the same value as the message's `t`. This connects to /stream as a
browser would, and for each JPEG that arrives subtracts that `t` from the
instant the last byte landed.

What this number includes: the compositor's JPEG encode, the bus hop, the web
service's pump, the HTTP multipart write and the network. What it does NOT
include: the browser's own decode and paint. That last part is exactly why the
stamp is burned into the pixels as well -- photographing a browser next to a
clock is the only way to measure it, and it needs a human. This tool measures
everything up to the browser's front door and says so.

Both halves matter and neither substitutes for the other, so this prints the
distribution rather than a single figure: a median hides the stalls that a
viewer actually notices.
"""
from __future__ import annotations

import argparse
import socket
import statistics
import struct
import time


def _jpeg_stamp_bytes(buf: bytes) -> None:
    """Placeholder: the stamp is read from the message, not from the pixels.

    Kept as a note rather than an implementation. Reading the burned-in digits
    back off the JPEG would need OCR, and OCR of our own overlay would only ever
    confirm what the message already says -- the two are written from the same
    variable in the same statement. The burned-in copy exists for the human
    half of the measurement (photograph the screen), not for this half.
    """
    raise NotImplementedError


def measure(host: str, port: int, seconds: float) -> list[float]:
    s = socket.create_connection((host, port), timeout=5)
    s.settimeout(2.0)
    s.sendall(b"GET /stream?stamp=1 HTTP/1.1\r\nHost: x\r\n\r\n")
    buf = b""
    lat: list[float] = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            chunk = s.recv(262144)
        except socket.timeout:
            continue
        if not chunk:
            break
        now = time.time()
        buf += chunk
        # The web service prefixes each part with an X-Stamp header carrying the
        # compositor's t as a double. Parsing the header rather than the JPEG
        # keeps this honest: it is the same value the pixels show.
        while True:
            i = buf.find(b"X-Stamp: ")
            if i < 0:
                break
            j = buf.find(b"\r\n", i)
            if j < 0:
                break
            try:
                lat.append(now - float(buf[i + 9:j]))
            except ValueError:
                pass
            buf = buf[j:]
        if len(buf) > 4 << 20:
            buf = buf[-(1 << 20):]
    s.close()
    return lat


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    lat = [v * 1000.0 for v in measure(args.host, args.port, args.seconds)]
    if not lat:
        print("no frames arrived -- the stream is not delivering")
        raise SystemExit(1)
    lat.sort()
    p = lambda q: lat[min(len(lat) - 1, int(q * len(lat)))]   # noqa: E731
    print(f"{args.label or 'web latency'}: {len(lat)} frames over "
          f"{args.seconds:.0f} s ({len(lat) / args.seconds:.1f} fps)")
    print(f"  median {statistics.median(lat):.0f} ms | p95 {p(0.95):.0f} ms | "
          f"max {max(lat):.0f} ms | min {min(lat):.0f} ms")
    print("  (compositor encode -> bus -> web -> socket; excludes browser "
          "decode and paint)")


if __name__ == "__main__":
    main()
