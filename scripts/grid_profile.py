#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Where the compositor's frame time goes, function by function.

The frame budget went from 13-23 ms to 54-57 ms when the occupancy-grid path
arrived, a factor of three, and every figure in the simplification plan is
compared against that. Before deciding what to cut, the cost has to be
attributed rather than guessed at: the plan names points_to_grid, the
morphological close and dilate on the grid, findContours and shrink, and those
four run on very different array sizes.

Runs on one real depth frame and one real silhouette taken off the bus, so the
sizes and the sparsity are the ones the compositor actually sees. Each stage is
timed on its own, repeatedly, and the median reported: a single call is
dominated by whatever the allocator was doing at the time.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import time

import cv2
import numpy as np

from edgebot import topics
from edgebot.bus import Subscriber
from edgebot.floor import (GRID_BOUNDS, GRID_CELL, points_to_grid,
                           polygon_from_mask, shrink)

CALIB = "/config/camera_calibration.json"


def timeit(label, fn, repeats):
    """Median milliseconds over `repeats` calls, plus the result of the last."""
    out = None
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        out = fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    return label, statistics.median(times), out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=15)
    ap.add_argument("--seconds", type=float, default=25.0)
    args = ap.parse_args()

    sub = Subscriber([topics.CAMERA_DEPTH, topics.OBSTACLE_MASK])
    depth = mask = None
    t0 = time.time()
    while time.time() - t0 < args.seconds and (depth is None or mask is None):
        m = sub.recv(1000)
        if m is None:
            continue
        t, p = m
        if t == topics.CAMERA_DEPTH and depth is None:
            depth = (np.frombuffer(p["depth"], np.uint16)
                     .reshape(p["h"], p["w"]).astype(np.float32)
                     * p.get("scale", 0.001))
        elif t == topics.OBSTACLE_MASK and mask is None:
            w, h = int(p["w"]), int(p["h"])
            mask = (np.unpackbits(np.frombuffer(p["bits"], np.uint8))[:w * h]
                    .reshape(h, w).astype(bool))
    sub.close()
    if depth is None:
        raise SystemExit("no depth on the bus; is the stack running?")

    with open(CALIB) as fh:
        cal = json.load(fh)
    i = cal["intrinsics"]
    H = float(cal["camera_height_m"])
    pit = math.radians(abs(float(cal["pitch_deg"])))
    dh, dw = depth.shape
    fx = i["fx"] * dw / i["width"]
    fy = i["fy"] * dh / i["height"]
    ppx = i["ppx"] * dw / i["width"]
    ppy = i["ppy"] * dh / i["height"]
    print(f"depth {dw}x{dh}, mask "
          f"{'none' if mask is None else f'{mask.shape[1]}x{mask.shape[0]}'}, "
          f"grid {int((GRID_BOUNDS[1] - GRID_BOUNDS[0]) / GRID_CELL)}"
          f"x{int((GRID_BOUNDS[3] - GRID_BOUNDS[2]) / GRID_CELL)} "
          f"cell {GRID_CELL} m, {args.repeats} repeats\n")

    rows = []

    def deproject():
        uu, vv = np.meshgrid(np.arange(dw), np.arange(dh))
        xn = (uu - ppx) / fx
        yn = (vv - ppy) / fy
        cp, sp = math.cos(pit), math.sin(pit)
        return depth * (cp - yn * sp), -xn * depth, H - depth * (sp + yn * cp)

    rows.append(timeit("deproject the whole depth frame", deproject,
                       args.repeats))
    f, l, up = rows[-1][2]
    valid = (depth > 0.3) & (depth < 12.0)
    sm = (cv2.resize(mask.astype(np.uint8), (dw, dh),
                     interpolation=cv2.INTER_NEAREST).astype(bool)
          if mask is not None else np.zeros_like(valid))
    obj = sm & valid & (up > 0.06)
    flo = valid & ~sm

    rows.append(timeit("points_to_grid, objects (with margin)",
                       lambda: points_to_grid(f, l, obj, GRID_CELL,
                                              GRID_BOUNDS, margin=0.12),
                       args.repeats))
    occ = rows[-1][2]
    rows.append(timeit("points_to_grid, floor (no margin)",
                       lambda: points_to_grid(f, l, flo, GRID_CELL,
                                              GRID_BOUNDS),
                       args.repeats))

    k3 = np.ones((3, 3), np.uint8)
    rows.append(timeit("morphology close on the grid",
                       lambda: cv2.morphologyEx(occ.astype(np.uint8),
                                                cv2.MORPH_CLOSE, k3),
                       args.repeats))
    rows.append(timeit("morphology dilate on the grid",
                       lambda: cv2.dilate(occ.astype(np.uint8), k3),
                       args.repeats))

    fm = flo.astype(np.uint8)
    rows.append(timeit("findContours on the floor mask",
                       lambda: cv2.findContours(fm, cv2.RETR_CCOMP,
                                                cv2.CHAIN_APPROX_SIMPLE),
                       args.repeats))

    def to_world(u, v):
        zc = (v - ppy) / fy
        wdz = -zc * math.cos(pit) - math.sin(pit)
        if wdz >= -1e-6:
            return None
        z = -H / wdz
        return (z * (math.cos(pit) - zc * math.sin(pit)),
                -((u - ppx) / fx) * z)

    poly = polygon_from_mask(flo, to_world)
    rows.append(timeit("polygon_from_mask on the floor mask",
                       lambda: polygon_from_mask(flo, to_world), args.repeats))
    if poly:
        rows.append(timeit("shrink the polygon by 0.10 m",
                           lambda: shrink(poly, 0.10), args.repeats))

    total = sum(r[1] for r in rows)
    print(f"{'stage':<40}{'ms':>9}{'share':>9}")
    for label, ms, _ in sorted(rows, key=lambda r: -r[1]):
        print(f"{label:<40}{ms:>9.2f}{100.0 * ms / total:>8.0f}%")
    print(f"{'TOTAL of the stages timed':<40}{total:>9.2f}")


if __name__ == "__main__":
    main()
