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
    ap.add_argument("--strides", default="2,4,8",
                    help="floor subsampling factors to compare against 1")
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
    # The GEOMETRIC floor mask, the one _ground_grids is handed: pixels within
    # a tolerance of the ground plane. An earlier version of this profile used
    # `valid & ~sm` -- every valid pixel that is not an object -- which is a
    # different and much larger set, and it reported a floor cost the code does
    # not pay. Measuring the wrong call is the mistake this repository keeps
    # buying, so the two floor rows below now mirror the source line for line.
    flo = valid & (np.abs(up) < 0.08)

    rows.append(timeit("points_to_grid, objects (with margin)",
                       lambda: points_to_grid(f, l, obj, GRID_CELL,
                                              GRID_BOUNDS, margin=0.12),
                       args.repeats))
    occ = rows[-1][2]

    # The floor path is NOT the object path with a different mask. It goes
    # through the ground plane, on the pixel indices only:
    #     ys, xs = np.nonzero(floor_mask)
    #     ff, ll = project_many(xs, ys)
    #     points_to_grid(ff, ll, isfinite(ff) & isfinite(ll))
    # so its cost scales with the number of floor PIXELS, not with the frame.
    def to_world_many(u, v):
        xw = (u - ppx) / fx
        yw = (v - ppy) / fy
        cp, sp = math.cos(pit), math.sin(pit)
        den = yw * cp + sp
        with np.errstate(divide="ignore", invalid="ignore"):
            ze = np.where(den > 1e-6, H / den, np.nan)
        ze = np.where((ze > 0.2) & (ze < 25.0), ze, np.nan)
        return ze * (cp - yw * sp), -xw * ze

    rows.append(timeit("nonzero on the floor mask",
                       lambda: np.nonzero(flo), args.repeats))
    fys, fxs = rows[-1][2]
    rows.append(timeit("project_many over the floor pixels",
                       lambda: to_world_many(fxs.astype(np.float64),
                                             fys.astype(np.float64)),
                       args.repeats))
    ff, ll = rows[-1][2]
    fsel = np.isfinite(ff) & np.isfinite(ll)
    rows.append(timeit("points_to_grid, floor (1-D points)",
                       lambda: points_to_grid(ff, ll, fsel, GRID_CELL,
                                              GRID_BOUNDS),
                       args.repeats))
    flr = rows[-1][2]
    ncell = max(1, int(flr.sum()))
    print(f"floor pixels {int(flo.sum())}, of which {int(fsel.sum())} project "
          f"to a finite ground point, landing on {ncell} cells "
          f"= {fsel.sum() / ncell:.1f} points per occupied cell\n")

    # Subsampling: does it give back the same grid, and how much does it save?
    # "Faster" alone is not an argument -- rasterising nothing at all is
    # fastest. The grid is the deliverable, so the grid is what is compared.
    for stride in (int(s) for s in args.strides.split(",")):
        sub = flo[::stride, ::stride]
        sys_, sxs = np.nonzero(sub)
        sfy, sfx = (sys_ * stride).astype(np.float64), (sxs * stride).astype(np.float64)

        def one(sfx=sfx, sfy=sfy):
            gf, gl = to_world_many(sfx, sfy)
            return points_to_grid(gf, gl, np.isfinite(gf) & np.isfinite(gl),
                                  GRID_CELL, GRID_BOUNDS)

        _, ms, g = timeit(f"stride {stride}", one, args.repeats)
        lost = int((flr & ~g).sum())
        gained = int((g & ~flr).sum())
        print(f"stride {stride}: {int(sfx.size)} points "
              f"({100.0 * sfx.size / max(1, fsel.sum()):.1f} %), "
              f"{ms:.2f} ms for project+rasterise, {int(g.sum())} cells "
              f"vs {ncell}, lost {lost}, gained {gained}, "
              f"symmetric difference {lost + gained} "
              f"({100.0 * (lost + gained) / ncell:.1f} % of the full grid)")
    print()

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
