#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Why does the grid read a passage 0.15 m narrower than a tape measure?

Established in docs/ETAPE-2-RESULTS.md §7.1 and §7.3bis: over 15 frames the
occupancy grid reports 0.10 to 0.20 m less free width than the tape, on both
sides of the same table, and that shortfall is what refuses a 0.70 m passage a
0.44 m robot fits through. A perception defect deciding a navigation verdict.

Three candidates, and the point of this script is to price them SEPARATELY on
ONE frozen frame, because correcting them together would leave nobody able to
say which one mattered.

  CELL       points_to_grid floors each point into a 0.05 m cell, so an object
             grows outward by up to one cell on each side and the GAP between
             two objects shrinks by up to two. Isolated by re-rasterising the
             identical points at 0.01 m and 0.005 m: same points, same
             deprojection, same mask, only the cell changes.

  MASK       the YOLO silhouette may be wider than the object. Isolated by
             building the same occupancy from a purely GEOMETRIC criterion --
             height above the floor plane -- which needs no model at all. If
             the mask-derived obstacle is wider than the height-derived one,
             the mask is the difference.

  DEPTH      the RealSense spatial filter and hole filling do their worst
             exactly at a depth discontinuity, which is precisely an object's
             edge. Not isolated here: it needs the sensor reconfigured with
             DEPTH_FILTERS=0 and a second run of this script. The two runs are
             compared by hand, and the script prints the row to fill in.

Everything is measured on the FREE GAP, not on the object, because the free gap
is what the navigator tests and what the tape measured.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

from edgebot import topics
from edgebot.bus import Subscriber
from edgebot.floor import points_to_grid

CALIB = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
OBJECT_Z_MIN = float(os.environ.get("OBJECT_Z_MIN", "0.08"))
OBSTACLE_X_MIN = float(os.environ.get("OBSTACLE_X_MIN", "1.4"))
FOOTPRINT_X_MAX = float(os.environ.get("FOOTPRINT_X_MAX", "6.5"))


def widest_free_run(occ, x: float, cell: float, bounds, y_lo: float,
                    y_hi: float) -> float:
    """Widest run of free cells at forward distance x, inside [y_lo, y_hi].

    The GAP, not the object. Quantisation moves both of its edges inward and
    that is the quantity the tape and the navigator both care about.
    """
    i = int((x - bounds[0]) / cell)
    if not (0 <= i < occ.shape[0]):
        return 0.0
    j0 = max(0, int((y_lo - bounds[2]) / cell))
    j1 = min(occ.shape[1], int((y_hi - bounds[2]) / cell) + 1)
    row = occ[i, j0:j1]
    best = run = 0
    for v in row:
        run = 0 if v else run + 1
        best = max(best, run)
    return best * cell


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--x", type=float, default=3.5,
                    help="forward distance at which to measure the gap")
    ap.add_argument("--band", type=float, nargs=2, default=[-1.6, -0.1],
                    metavar=("Y_LO", "Y_HI"),
                    help="lateral band containing the passage")
    ap.add_argument("--tape", type=float, default=None,
                    help="the tape measurement of that gap, for the shortfall")
    args = ap.parse_args()

    import cv2

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
    if depth is None or mask is None:
        print("need both a depth frame and a silhouette; is the stack up?")
        return 1

    with open(CALIB) as fh:
        cal = json.load(fh)
    i_ = cal["intrinsics"]
    H = float(cal["camera_height_m"])
    pit = math.radians(abs(float(cal["pitch_deg"])))
    dh, dw = depth.shape
    fx = i_["fx"] * dw / i_["width"]
    fy = i_["fy"] * dh / i_["height"]
    ppx = i_["ppx"] * dw / i_["width"]
    ppy = i_["ppy"] * dh / i_["height"]

    uu, vv = np.meshgrid(np.arange(dw), np.arange(dh))
    xn = (uu - ppx) / fx
    yn = (vv - ppy) / fy
    cp, sp = math.cos(pit), math.sin(pit)
    fwd = depth * (cp - yn * sp)
    lat = -xn * depth
    up = H - depth * (sp + yn * cp)

    sm = cv2.resize(mask.astype(np.uint8), (dw, dh),
                    interpolation=cv2.INTER_NEAREST).astype(bool)
    valid = ((depth > 0.3) & (depth < 12.0) & np.isfinite(fwd) & np.isfinite(lat)
             & (fwd > OBSTACLE_X_MIN) & (fwd < FOOTPRINT_X_MAX))
    tall = valid & (up > OBJECT_Z_MIN)

    sel_mask = sm & tall            # what the stack uses today
    sel_geom = tall                 # geometry alone, no model

    y_lo, y_hi = args.band
    print(f"depth {dw}x{dh}, gap measured at x = {args.x:.2f} m, "
          f"band y {y_lo:+.2f}..{y_hi:+.2f} m")
    print(f"silhouette {int(sm.sum())} px, above {OBJECT_Z_MIN:.2f} m "
          f"{int(sel_mask.sum())} px; geometry alone {int(sel_geom.sum())} px")
    print()

    bounds = (0.0, 8.0, -4.0, 4.0)

    print("CELL -- the same points, only the cell changes")
    print(f"{'cell':>8}{'gap (mask)':>13}{'gap (geometry)':>17}")
    base = {}
    for cell in (0.05, 0.02, 0.01, 0.005):
        gm = widest_free_run(points_to_grid(fwd, lat, sel_mask, cell, bounds),
                             args.x, cell, bounds, y_lo, y_hi)
        gg = widest_free_run(points_to_grid(fwd, lat, sel_geom, cell, bounds),
                             args.x, cell, bounds, y_lo, y_hi)
        base[cell] = (gm, gg)
        print(f"{cell:>8.3f}{gm:>13.3f}{gg:>17.3f}")
    q_mask = base[0.005][0] - base[0.05][0]
    q_geom = base[0.005][1] - base[0.05][1]
    print(f"\nquantisation costs {q_mask:+.3f} m of gap on the mask path, "
          f"{q_geom:+.3f} m on the geometric one")
    print("  (0.005 m is the reference: 100x finer than the object's own edge "
          "noise, so what it adds back is the cell and nothing else)")

    print()
    print("MASK -- silhouette against pure geometry, at the SHIPPED 0.05 m cell")
    gm, gg = base[0.05]
    print(f"  gap from the YOLO silhouette   {gm:.3f} m")
    print(f"  gap from height alone          {gg:.3f} m")
    print(f"  the mask costs                 {gg - gm:+.3f} m")
    if gg > gm:
        print("  -> the silhouette is WIDER than the thing that stands there")
    elif gg < gm:
        print("  -> the silhouette is NARROWER; geometry sees more obstacle, "
              "probably floor noise or a wall the model does not label")

    print()
    print("DEPTH -- not isolated here")
    print(f"  this run: DEPTH_FILTERS={os.environ.get('PROBE_FILTERS', '?')}, "
          f"gap {gm:.3f} m at the shipped cell")
    print("  rerun the source with DEPTH_FILTERS=0 and compare this one number")

    if args.tape:
        print()
        print(f"SHORTFALL against the tape ({args.tape:.2f} m)")
        print(f"  grid as shipped        {gm:.3f} m   "
              f"short by {args.tape - gm:+.3f} m")
        print(f"  cell removed           {base[0.005][0]:.3f} m   "
              f"short by {args.tape - base[0.005][0]:+.3f} m")
        print(f"  cell and mask removed  {base[0.005][1]:.3f} m   "
              f"short by {args.tape - base[0.005][1]:+.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
