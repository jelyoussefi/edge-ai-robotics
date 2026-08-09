#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Measure what the navigator actually has to walk in, and whether it scrapes.

Written for the footprint-merge problem in a small room: obstacle rectangles
that are merged when the gap between them is too narrow for the robot turn two
pieces of furniture into one barrier, and in a tight lounge that can close every
route and collapse the walkable floor to a band.

Reports, over a window:

  walkable width   the lateral extent of the PATROL_ROI polygon, sampled at
                   several forward distances. This is the number that says
                   whether there is anywhere to go, and a single area figure
                   hides a band that is wide at one end and shut at the other.
  footprints       how many blocked rectangles the navigator sees, and how big
                   the largest is -- a merge shows up here as a count that
                   falls while the largest span grows.
  clearance        for every robot pose, the distance from the robot centre to
                   the nearest footprint edge. Negative means inside one.
  scrapes          poses closer than ROBOT_HALF_WIDTH to a footprint, i.e. the
                   robot's body overlapping something it was told to avoid.
  travel           lateral spread of the path, which is how "did it route
                   around the table" shows up as a number rather than a memory.

Clearance is measured against the footprints the navigator was given, not
against the furniture, because that is what it steers on. A footprint that is
wrong is a perception problem and this cannot see it.
"""
from __future__ import annotations

import argparse
import math
import os
import statistics
import time

from edgebot import topics
from edgebot.bus import Subscriber

HALF_WIDTH = float(os.environ.get("ROBOT_HALF_WIDTH", "0.22"))


def rect_distance(x: float, y: float, r) -> float:
    """Distance from a point to a rectangle [x0, x1, y0, y1]; <0 when inside."""
    x0, x1, y0, y1 = r
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx == 0.0 and dy == 0.0:
        # Inside: the negative distance to the nearest edge.
        return -min(x - x0, x1 - x, y - y0, y1 - y)
    return math.hypot(dx, dy)


def poly_width_at(poly, x: float, tol: float = 0.25):
    """Lateral extent of the polygon near a forward distance x.

    Sampled from the vertices rather than by clipping: the polygon comes from a
    depth contour and has enough vertices that a band around x captures its
    width, and a clipper would add a dependency for no extra truth.
    """
    ys = [p[1] for p in poly if abs(p[0] - x) <= tol]
    return (max(ys) - min(ys)) if len(ys) >= 2 else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI, topics.ROBOT_STATE])
    poses: list[tuple[float, float]] = []
    clearances: list[float] = []
    scrapes: list[tuple[float, float, float]] = []
    widths: dict[float, list[float]] = {}
    counts: list[int] = []
    spans: list[float] = []
    blocked: list = []
    roi = None

    t0 = time.time()
    while time.time() - t0 < args.seconds:
        msg = sub.recv(500)
        if msg is None:
            continue
        topic, p = msg
        if topic == topics.PATROL_ROI:
            roi = [(float(a), float(b)) for a, b in (p.get("roi") or [])]
            blocked = [[float(v) for v in r] for r in (p.get("blocked") or [])]
            counts.append(len(blocked))
            if blocked:
                spans.append(max(max(r[1] - r[0], r[3] - r[2]) for r in blocked))
            for x in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
                w = poly_width_at(roi, x)
                if w is not None:
                    widths.setdefault(x, []).append(w)
        elif topic == topics.ROBOT_STATE:
            q = p.get("qpos") or []
            if len(q) < 2:
                continue
            x, y = float(q[0]), float(q[1])
            poses.append((x, y))
            if blocked:
                d = min(rect_distance(x, y, r) for r in blocked)
                clearances.append(d)
                if d < HALF_WIDTH:
                    scrapes.append((x, y, d))
    sub.close()

    lab = args.label or "nav"
    print(f"=== {lab}: {args.seconds:.0f} s, {len(poses)} poses, "
          f"{len(counts)} ROI updates ===")

    print("walkable floor width (lateral extent of the ROI polygon):")
    if widths:
        for x in sorted(widths):
            v = widths[x]
            print(f"   at x={x:.1f} m : median {statistics.median(v):.2f} m "
                  f"(min {min(v):.2f}, max {max(v):.2f}, n={len(v)})")
    else:
        print("   the ROI polygon never reached these distances")

    if roi:
        xs = [p[0] for p in roi]
        ys = [p[1] for p in roi]
        print(f"   ROI bounding box: x {min(xs):.2f}..{max(xs):.2f} m, "
              f"y {min(ys):.2f}..{max(ys):.2f} m")

    if counts:
        print(f"footprints: median {statistics.median(counts):.0f} "
              f"(min {min(counts)}, max {max(counts)}); largest single span "
              f"median {statistics.median(spans):.2f} m" if spans else "")

    if clearances:
        cl = sorted(clearances)
        print(f"clearance to the nearest footprint: min {cl[0]:+.3f} m, "
              f"p05 {cl[max(0, int(0.05 * len(cl)))]:+.3f}, "
              f"median {statistics.median(cl):+.3f}")
        print(f"scrapes (closer than ROBOT_HALF_WIDTH={HALF_WIDTH:.2f} m): "
              f"{len(scrapes)} of {len(clearances)} poses "
              f"({100.0 * len(scrapes) / len(clearances):.1f} %)")
        if scrapes:
            worst = min(scrapes, key=lambda s: s[2])
            print(f"   worst at ({worst[0]:.2f}, {worst[1]:.2f}) "
                  f"clearance {worst[2]:+.3f} m")
    else:
        print("clearance: no footprints were published while the robot moved")

    if poses:
        xs = [p[0] for p in poses]
        ys = [p[1] for p in poses]
        print(f"travel: x {min(xs):.2f}..{max(xs):.2f} m "
              f"(span {max(xs) - min(xs):.2f}), y {min(ys):.2f}..{max(ys):.2f} m "
              f"(lateral span {max(ys) - min(ys):.2f})")


if __name__ == "__main__":
    main()
