#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Dilate the cells, or inflate the query? Same grid, same instant, both answers.

The two ways to spend CLEARANCE demand the same corridor of real floor, so the
only honest way to choose is to make them disagree and look at where.

  dilate  points_to_grid grows every occupied cell by CLEARANCE with an ELLIPSE
          structuring element, and the corridor test then asks for the robot's
          bare half-width.
  query   the grid stays raw and the corridor test asks for half-width plus
          CLEARANCE.

Dilation is a Minkowski sum with a disc: it is what a CIRCULAR robot could not
enter, and it rounds corners. The query test grows the swept rectangle: what a
RECTANGULAR robot could not enter. Past a corner, diagonally, the disc is the
more permissive of the two, and that is exactly the geometry of rounding
furniture. Whether the difference matters in this room is a measurement.

Run OFFLINE, against one PATROL_ROI message, so both forms see the identical
grid at the identical instant. Two live runs cannot do this: the scene drifts
between them by more than the effect, as the camera-to-camera spread of 2624 /
2656 / 2744 occupied cells shows.

REQUIRES A RAW GRID. In dilate mode the published cells are already grown and
the raw layer is gone -- dilation destroys the information this comparison
needs. That is itself one of the findings, and the probe says so and exits
rather than comparing a grid against itself.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from edgebot import topics
from edgebot.bus import Subscriber
from edgebot.floor import (GRID_BOUNDS, GRID_CELL, corridor_blocked,
                           free_lane, grid_extent, min_corridor, query_pad,
                           unpack_grid)

HALF_WIDTH = float(os.environ.get("ROBOT_HALF_WIDTH", "0.22"))


def dilate(occ, metres: float, cell: float):
    """The cells as points_to_grid would have grown them: ellipse, both sides."""
    import cv2
    if metres <= 0:
        return occ.copy()
    k = 2 * int(round(metres / cell)) + 1
    el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    return cv2.dilate(occ.astype(np.uint8), el).astype(bool)


def lane_map(occ, half, xs, ys, look, cell, bounds, pad=0.0):
    """For each (x, lane), whether a corridor of `half` is clear ahead."""
    return np.array([[not corridor_blocked(occ, float(x), float(y), 1.0, look,
                                           half, cell, bounds, pad=pad)
                      for y in ys] for x in xs], bool)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--clearance", type=float, default=0.12)
    ap.add_argument("--look", type=float, default=1.5,
                    help="corridor length for the lane comparison")
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI])
    msg = None
    t0 = time.time()
    while time.time() - t0 < args.seconds and msg is None:
        got = sub.recv(500)
        if got and got[1].get("occ"):
            msg = got[1]
    sub.close()
    if msg is None:
        print("no occupancy grid on the bus; is the stack running?")
        return 1

    nx, ny = int(msg["gnx"]), int(msg["gny"])
    cell = float(msg.get("gcell", GRID_CELL))
    bounds = tuple(msg.get("gbounds", GRID_BOUNDS))
    occ = unpack_grid(msg["occ"], nx, ny)
    pub_mode = msg.get("clearance_mode", "unknown")
    pub_clear = msg.get("clearance")

    print(f"grid {nx}x{ny}, {cell} m cell, published in {pub_mode} mode with "
          f"clearance {pub_clear}")
    if pub_mode != "query":
        print("REFUSING to compare: the published cells are already dilated, "
              "so the raw obstacle layer this needs no longer exists. That is "
              "the point of the finding -- dilation is not invertible. Run "
              "the stack with CLEARANCE_MODE=query and try again.")
        return 1

    c = args.clearance
    raw_cells = int(occ.sum())
    dil = dilate(occ, c, cell)
    print(f"occupied cells: raw {raw_cells}, dilated by {c:.2f} m "
          f"{int(dil.sum())} (x{dil.sum() / max(1, raw_cells):.2f})")
    import cv2 as _cv2  # noqa: F401  (dilate() needs it; fail early if absent)
    k = 2 * int(round(c / cell)) + 1
    print(f"both forms demand {min_corridor(HALF_WIDTH, c):.2f} m of real "
          f"floor: dilate asks the grown grid for {HALF_WIDTH:.2f} m, query "
          f"asks the raw grid for {HALF_WIDTH + c:.2f} m with a "
          f"{c:.2f} m longitudinal pad")
    print(f"BUT dilate cannot deliver {c:.2f} m on a {cell:.2f} m grid: the "
          f"kernel is {k}x{k}, so it grows {(k // 2) * cell:.2f} m, "
          f"{100.0 * ((k // 2) * cell - c) / c:+.0f} % of what was asked. "
          f"Query mode is exact -- it never quantises the margin.")
    print()

    # --- where the two disagree, lane by lane -----------------------------
    xs = np.arange(1.5, 5.51, 0.10)
    ys = np.arange(-2.6, 2.61, 0.05)
    a = lane_map(dil, HALF_WIDTH, xs, ys, args.look, cell, bounds)   # dilate
    b = lane_map(occ, HALF_WIDTH + c, xs, ys, args.look, cell, bounds,
                 pad=query_pad(c, "query"))                        # query
    tot = a.size
    same = int((a == b).sum())
    only_a = int((a & ~b).sum())     # dilate says clear, query says blocked
    only_b = int((b & ~a).sum())
    print(f"lane decisions over {len(xs)} distances x {len(ys)} lanes = {tot}:")
    print(f"  agree                      {same:6d}  ({100.0 * same / tot:.2f} %)")
    print(f"  clear for DILATE only      {only_a:6d}  "
          f"({100.0 * only_a / tot:.2f} %)  <- the rounded corner")
    print(f"  clear for QUERY only       {only_b:6d}  "
          f"({100.0 * only_b / tot:.2f} %)")
    if only_a or only_b:
        d = np.argwhere(a != b)
        print(f"  disagreements span x {xs[d[:, 0].min()]:.2f}"
              f"..{xs[d[:, 0].max()]:.2f} m, y {ys[d[:, 1].min()]:+.2f}"
              f"..{ys[d[:, 1].max()]:+.2f} m")

    # --- can the raw grid still answer another question? ------------------
    print()
    print("asking the SAME grid for other corridor widths (the thing dilation "
          "cannot do):")
    for w in (0.44, 0.56, 0.68, 0.80, 0.90, 1.00):
        half = w / 2.0
        n_ok = sum(1 for y in ys
                   if not corridor_blocked(occ, 2.0, float(y), 1.0, args.look,
                                           half, cell, bounds))
        print(f"  a {w:.2f} m corridor from x=2.0 m is clear on "
              f"{n_ok:3d} of {len(ys)} lanes")

    # --- the narrowest passage each form will accept ----------------------
    print()
    print("narrowest FREE passage found at each forward distance, real floor:")
    for x in np.arange(2.0, 5.01, 0.5):
        row = occ[max(0, int((float(x) - bounds[0]) / cell)), :]
        runs, start, out = [], None, []
        for j in range(len(row) + 1):
            free = j < len(row) and not row[j]
            if free and start is None:
                start = j
            elif not free and start is not None:
                out.append((j - start) * cell)
                start = None
        runs = [r for r in out if r >= 0.30]
        if runs:
            print(f"  x {x:.1f} m: {len(runs)} passage(s), narrowest "
                  f"{min(runs):.2f} m, widest {max(runs):.2f} m"
                  f"   {'PASSABLE' if min(runs) >= min_corridor(HALF_WIDTH, c) else 'narrowest is BELOW the budget'}")
        else:
            print(f"  x {x:.1f} m: no passage above 0.30 m")

    ext = grid_extent(occ, cell, bounds)
    if ext:
        print(f"\noccupied extent {ext[0]:.1f}-{ext[1]:.1f} x "
              f"{ext[2]:.1f}-{ext[3]:.1f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
