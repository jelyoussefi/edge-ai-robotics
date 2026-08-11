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
    ap.add_argument("--frames", type=int, default=10,
                    help="grids to aggregate; one frame is noise")
    ap.add_argument("--corridor", type=float, nargs=4,
                    metavar=("X0", "X1", "YLO", "YHI"),
                    default=[2.0, 5.0, -4.0, 4.0],
                    help="x range and y band to analyse slice by slice")
    ap.add_argument("--look", type=float, default=1.5,
                    help="corridor length for the lane comparison")
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI])
    msgs = []
    t0 = time.time()
    while time.time() - t0 < args.seconds and len(msgs) < args.frames:
        got = sub.recv(500)
        if got and got[1].get("occ"):
            msgs.append(got[1])
    sub.close()
    if not msgs:
        print("no occupancy grid on the bus; is the stack running?")
        return 1
    msg = msgs[-1]

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

    # --- the check that actually decides: width AND axis drift -----------
    #
    # free_lane only ever tests a STRAIGHT corridor parallel to the x axis, so
    # a passage that is wide everywhere but whose centre line slides sideways
    # is not passable however wide each individual slice is. Measured on
    # 2026-08-10: the corridor held 0.50-0.70 m at every slice while its centre
    # drifted from -0.95 to -1.08 m between x=2.5 and 4.25, leaving 0.35 m of
    # COMMON width for a 0.44 m robot. Width per slice said "fine"; the robot
    # was right and the number was answering the wrong question.
    #
    # The quantity that matters is the intersection over slices: for each lane
    # y, the largest half-width free at that y in EVERY slice at once.
    x0, x1 = args.corridor[0], args.corridor[1]
    ylo, yhi = args.corridor[2], args.corridor[3]
    i0 = max(0, int((x0 - bounds[0]) / cell))
    i1 = min(occ.shape[0], int((x1 - bounds[0]) / cell) + 1)
    j_lo = max(0, int((ylo - bounds[2]) / cell))
    j_hi = min(occ.shape[1], int((yhi - bounds[2]) / cell) + 1)
    print()
    print(f"corridor analysis over x {x0:.2f}..{x1:.2f} m, y {ylo:+.2f}"
          f"..{yhi:+.2f} m, slice by slice:")
    print(f"{'x':>7}{'free width':>12}{'centre y':>11}   run")
    rows = []
    for i in range(i0, i1):
        row = occ[i, :]
        runs, st = [], None
        for j in range(j_lo, j_hi + 1):
            free = j < j_hi and not row[j]
            if free and st is None:
                st = j
            elif not free and st is not None:
                runs.append((st, j))
                st = None
        if not runs:
            print(f"{bounds[0] + i * cell:7.2f}{'BLOCKED':>12}")
            rows.append(None)
            continue
        st, en = max(runs, key=lambda r: r[1] - r[0])
        w = (en - st) * cell
        a, b_ = bounds[2] + st * cell, bounds[2] + en * cell
        c_y = (a + b_) / 2.0
        rows.append((a, b_))
        print(f"{bounds[0] + i * cell:7.2f}{w:12.2f}{c_y:+11.2f}   "
              f"{a:+.2f}..{b_:+.2f}")

    # The same analysis on EVERY frame collected, because one is noise. Two
    # consecutive single-frame runs of this gave 0.50 m and 0.40 m of common
    # width -- 0.10 m apart on the quantity the verdict turns on. The per-slice
    # table above is the last frame, for shape; these are the numbers to quote.
    agg = []
    for mm in msgs:
        o = unpack_grid(mm["occ"], nx, ny)
        if o is None:
            continue
        rr = []
        for i in range(i0, i1):
            row = o[i, :]
            rs_, st = [], None
            for j in range(j_lo, j_hi + 1):
                free = j < j_hi and not row[j]
                if free and st is None:
                    st = j
                elif not free and st is not None:
                    rs_.append((st, j))
                    st = None
            if not rs_:
                rr = None
                break
            a2, b2 = max(rs_, key=lambda r: r[1] - r[0])
            rr.append((bounds[2] + a2 * cell, bounds[2] + b2 * cell))
        if rr is None:
            agg.append((0.0, 0.0, 0.0))
            continue
        lo2 = max(r[0] for r in rr)
        hi2 = min(r[1] for r in rr)
        agg.append((max(0.0, hi2 - lo2),
                    min(r[1] - r[0] for r in rr),
                    max(r[1] - r[0] for r in rr)))
    if agg:
        com = sorted(a[0] for a in agg)
        nar = sorted(a[1] for a in agg)
        wid = sorted(a[2] for a in agg)
        med = lambda v: v[len(v) // 2]
        print()
        print(f"over {len(agg)} frames, not one:")
        print(f"  common width   median {med(com):.2f} m  "
              f"({com[0]:.2f} to {com[-1]:.2f})")
        print(f"  narrowest      median {med(nar):.2f} m  "
              f"({nar[0]:.2f} to {nar[-1]:.2f})")
        print(f"  widest         median {med(wid):.2f} m  "
              f"({wid[0]:.2f} to {wid[-1]:.2f})")
        print(f"  budget         {min_corridor(HALF_WIDTH, c):.2f} m, "
              f"cleared in {sum(1 for v in com if v >= min_corridor(HALF_WIDTH, c))}"
              f" of {len(com)} frames")

    print()
    if any(r is None for r in rows):
        print("INTERSECTION: EMPTY -- at least one slice is fully blocked in "
              "this band. No straight corridor exists here at any width.")
    else:
        lo = max(r[0] for r in rows)
        hi = min(r[1] for r in rows)
        common = max(0.0, hi - lo)
        budget = min_corridor(HALF_WIDTH, c)
        span = max(r[1] - r[0] for r in rows)
        narrow = min(r[1] - r[0] for r in rows)
        print(f"widest single slice   {span:.2f} m")
        print(f"narrowest slice       {narrow:.2f} m")
        print(f"COMMON to all slices  {common:.2f} m over y {lo:+.2f}..{hi:+.2f}")
        print(f"budget                {budget:.2f} m")
        drift = max(abs((r[0] + r[1]) / 2.0 - (rows[0][0] + rows[0][1]) / 2.0)
                    for r in rows)
        print(f"axis drift            {drift:.2f} m from the first slice")
        if common >= budget:
            print(f"\nPASSABLE in a straight line: {common:.2f} m of common "
                  f"width against {budget:.2f} m demanded, centred "
                  f"{(lo + hi) / 2.0:+.2f} m.")
        else:
            # WHY it fails matters more than THAT it fails, and the two causes
            # want opposite responses. If the common width is close to the
            # narrowest slice, the corridor is essentially straight and simply
            # too narrow -- widen the gap, or accept the robot cannot fit. If
            # the common width is far below the narrowest slice, every slice is
            # wide enough and the corridor still fails, which is drift: no gap
            # widening helps and it needs a planner that can follow a curve.
            #
            # An earlier version printed "not straight" for both, which was
            # simply wrong the first time the drift came out at 0.08 m.
            lost = narrow - common
            if lost < 0.10:
                print(f"\nTOO NARROW. The narrowest slice is {narrow:.2f} m "
                      f"and {common:.2f} m survives the intersection, so the "
                      f"corridor is effectively straight ({drift:.2f} m of "
                      f"drift) and simply narrower than the {budget:.2f} m "
                      f"demanded. Lowering CLEARANCE to "
                      f"{max(0.0, common / 2.0 - HALF_WIDTH):.2f} m would let "
                      f"the robot in with nothing to spare; whether that is "
                      f"wise is a different question from whether it fits.")
            else:
                print(f"\nNOT PASSABLE IN A STRAIGHT LINE, and not because it "
                      f"is narrow. Every slice is at least {narrow:.2f} m wide "
                      f"but only {common:.2f} m is common to all of them -- "
                      f"{lost:.2f} m lost to an axis that drifts {drift:.2f} m. "
                      f"No value of CLEARANCE fixes this: free_lane tests "
                      f"straight corridors and this one is not straight. It "
                      f"needs a planner that can follow a curve: etape 4, "
                      f"Nav2 and ITS.")

    ext = grid_extent(occ, cell, bounds)
    if ext:
        print(f"\noccupied extent {ext[0]:.1f}-{ext[1]:.1f} x "
              f"{ext[2]:.1f}-{ext[3]:.1f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
