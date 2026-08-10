#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""The compositor's own fps and frame-work figures, over a stated window.

Runs on the HOST, not in a container: it reads `docker compose logs`, because
the numbers it wants are the ones the compositor already prints every 30 s and
a second measurement path would just be a second thing to be wrong.

Two reasons this exists rather than a grep:

  A single 30 s line is noise. On this board consecutive lines span 12.4 to
  14.6 fps with the scene untouched, so quoting one is quoting whichever the
  scheduler handed you. This takes the median over several and prints the
  spread beside it, so a change smaller than the spread cannot be claimed.

  Every measurement in this project now carries its lap range. A window can
  straddle a stall or a replan, and two windows with the same fps over
  different laps are not the same measurement. The lap range comes from the
  navigator's own log lines over the SAME window, matched by timestamp.
"""
from __future__ import annotations

import argparse
import re
import statistics
import subprocess
import sys

FPS_RE = re.compile(
    r"composited ([\d.]+) fps, frame work median ([\d.]+) ms \(p95 ([\d.]+)\)")
# Separate, and not a trailing optional group on FPS_RE: an optional group after
# a lazy .* matches the empty string every time and silently reports no encode.
ENC_RE = re.compile(r"jpeg encode median ([\d.]+) ms")
LAP_RE = re.compile(r"\blap (\d+)\b")


def logs(service: str, since: str) -> list[str]:
    out = subprocess.run(
        ["docker", "compose", "logs", "--since", since, service],
        capture_output=True, text=True)
    return out.stdout.splitlines()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="12m",
                    help="docker logs window; 12m gives about 24 samples")
    ap.add_argument("--samples", type=int, default=20,
                    help="how many of the most recent 30 s lines to keep")
    ap.add_argument("--label", default="fps")
    args = ap.parse_args()

    fps, work, p95, enc = [], [], [], []
    for line in logs("compositor", args.since):
        m = FPS_RE.search(line)
        if not m:
            continue
        fps.append(float(m.group(1)))
        work.append(float(m.group(2)))
        p95.append(float(m.group(3)))
        e = ENC_RE.search(line)
        if e:
            enc.append(float(e.group(1)))
    if not fps:
        print("no 'composited ... fps' lines in the window; is the stack up?")
        return 1
    fps, work, p95 = (v[-args.samples:] for v in (fps, work, p95))
    enc = enc[-args.samples:]

    laps = [int(m.group(1)) for line in logs("sim", args.since)
            for m in [LAP_RE.search(line)] if m]

    print(f"=== {args.label}: {len(fps)} windows of 30 s "
          f"({30 * len(fps) / 60.0:.0f} min) ===")
    if laps:
        print(f"laps covered: {min(laps)} to {max(laps)}")
    else:
        print("laps covered: unmeasured (no lap line from the sim)")

    def row(name, v, unit):
        print(f"{name:<26} median {statistics.median(v):>7.1f} {unit}"
              f"   ({min(v):.1f} to {max(v):.1f})")

    row("composited fps", fps, "fps")
    row("frame work, median", work, "ms")
    row("frame work, p95", p95, "ms")
    if enc:
        row("jpeg encode", enc, "ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
