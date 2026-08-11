#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""How many 60 s windows does a navigation criterion need before it means anything?

Forced by docs/SURPROJECTION-RESULTS.md §15bis: at a fixed scene and a fixed
configuration, the body-overlap rate over 60 s came out 12.7 %, 19.2 % and
41.5 %. Every comparison built on a single window in the earlier documents --
mine included -- was carrying a difference smaller than the metric's own spread.

Two questions, and neither is answerable by argument:

  HOW MANY WINDOWS before the median settles? If the answer is five, an honest
  comparison costs five minutes and the plan has to say so before a criterion is
  written against it.

  WHICH QUANTITY is the better criterion? The rate tripled across passes where
  the minimum clearance moved from -0.024 to -0.010 m. That is a hint, not a
  result: -0.024 and -0.010 are both "the robot is inside a cell", and a
  quantity can look stable simply by being saturated.

Method. One long run, split into consecutive windows, then the median's own
variability estimated by bootstrap at each n. No modelling: resample the windows
with replacement, take the median, repeat, and report the width of the 90 %
interval. That width IS the resolution of a comparison made with n windows -- a
difference smaller than it cannot be claimed, whatever the plan says.

The metric is reported for BOTH quantities, on the SAME windows, so the choice
between them is made on one run rather than two.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

import numpy as np

from edgebot import topics
from edgebot.bus import Subscriber
from edgebot.floor import (assert_same_corridor, min_corridor, query_half,
                           query_pad, unpack_grid)

HALF_WIDTH = float(os.environ.get("ROBOT_HALF_WIDTH", "0.22"))
CLEARANCE = float(os.environ.get("CLEARANCE", "0.12"))
CLEARANCE_MODE = os.environ.get("CLEARANCE_MODE", "query")


def grid_distance(x, y, occ, cell, bounds):
    """Distance to the nearest occupied cell, negative inside. As nav_probe."""
    if occ is None or not occ.any():
        return None
    x0, _x1, y0, _y1 = bounds
    ix = np.nonzero(occ)
    cx = x0 + (ix[0] + 0.5) * cell
    cy = y0 + (ix[1] + 0.5) * cell
    d = np.hypot(cx - x, cy - y).min() - 0.5 * cell
    gi, gj = int((x - x0) / cell), int((y - y0) / cell)
    if 0 <= gi < occ.shape[0] and 0 <= gj < occ.shape[1] and occ[gi, gj]:
        return -abs(d) if d != 0 else -0.5 * cell
    return float(d)


def boot_median_width(vals, n, draws=2000, seed=0):
    """Width of the 90 % interval of the median computed from n windows."""
    if len(vals) < 2:
        return float("nan")
    rng = np.random.default_rng(seed)
    a = np.asarray(vals, float)
    meds = np.median(rng.choice(a, size=(draws, n), replace=True), axis=1)
    return float(np.percentile(meds, 95) - np.percentile(meds, 5))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=60.0)
    ap.add_argument("--windows", type=int, default=12)
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI, topics.ROBOT_STATE,
                      topics.SIM_TELEMETRY])
    qh = query_half(HALF_WIDTH, CLEARANCE, CLEARANCE_MODE)
    occ = None
    cell, bounds = 0.02, (0.0, 8.0, -4.0, 4.0)
    checked = False
    rows = []

    print(f"{args.windows} windows of {args.window:.0f} s "
          f"= {args.windows * args.window / 60:.0f} minutes")
    print(f"{'win':>4}{'laps':>12}{'overlap %':>11}{'scrape %':>10}"
          f"{'min clr':>10}{'p05 clr':>10}{'med clr':>10}")

    for w in range(args.windows):
        clear, over, scr, laps = [], 0, 0, []
        t0 = time.time()
        while time.time() - t0 < args.window:
            m = sub.recv(500)
            if m is None:
                continue
            t, p = m
            if t == topics.SIM_TELEMETRY:
                nav = (p or {}).get("nav") or {}
                if nav.get("lap") is not None:
                    laps.append(int(nav["lap"]))
                if not checked and nav.get("MIN_CORRIDOR") is not None:
                    checked = True
                    assert_same_corridor(
                        {"GRID_HALF": qh,
                         "GRID_PAD": query_pad(CLEARANCE, CLEARANCE_MODE),
                         "MIN_CORRIDOR": min_corridor(HALF_WIDTH, CLEARANCE)},
                        {k: nav.get(k) for k in
                         ("GRID_HALF", "GRID_PAD", "MIN_CORRIDOR")},
                        "criterion_probe")
            elif t == topics.PATROL_ROI and p.get("occ"):
                occ = unpack_grid(p["occ"], int(p["gnx"]), int(p["gny"]))
                cell = float(p.get("gcell", cell))
                bounds = tuple(p.get("gbounds", bounds))
            elif t == topics.ROBOT_STATE and occ is not None:
                q = p.get("qpos") or []
                if len(q) < 2:
                    continue
                d = grid_distance(float(q[0]), float(q[1]), occ, cell, bounds)
                if d is None:
                    continue
                clear.append(d)
                if d < qh:
                    scr += 1
                if d < HALF_WIDTH:
                    over += 1
        if not clear:
            print(f"{w:>4}   no poses")
            continue
        cl = sorted(clear)
        rows.append({
            "overlap": 100.0 * over / len(clear),
            "scrape": 100.0 * scr / len(clear),
            "min": cl[0],
            "p05": cl[max(0, int(0.05 * len(cl)))],
            "med": statistics.median(cl),
        })
        lp = f"{min(laps)}-{max(laps)}" if laps else "?"
        print(f"{w:>4}{lp:>12}{rows[-1]['overlap']:>11.1f}"
              f"{rows[-1]['scrape']:>10.1f}{rows[-1]['min']:>10.3f}"
              f"{rows[-1]['p05']:>10.3f}{rows[-1]['med']:>10.3f}")
    sub.close()
    if len(rows) < 3:
        print("not enough windows to say anything")
        return 1

    print()
    print("spread of the RAW windows")
    for key, unit in (("overlap", "%"), ("scrape", "%"), ("min", "m"),
                      ("p05", "m"), ("med", "m")):
        v = sorted(r[key] for r in rows)
        rel = (100.0 * (v[-1] - v[0]) / abs(statistics.median(v))
               if statistics.median(v) else float("nan"))
        print(f"  {key:<9} median {statistics.median(v):>8.3f} {unit}   "
              f"range {v[0]:.3f} to {v[-1]:.3f}   "
              f"spread {v[-1] - v[0]:.3f} = {rel:.0f} % of the median")

    print()
    print("RESOLUTION of a comparison made with n windows")
    print("  (width of the 90 % bootstrap interval of the median; a difference")
    print("   smaller than this cannot be claimed, whatever the plan says)")
    ns = [n for n in (1, 2, 3, 5, 8, 12, 20) if n <= 4 * len(rows)]
    print(f"{'n':>4}" + "".join(f"{k:>14}" for k in
                                ("overlap %", "scrape %", "min clr m",
                                 "p05 clr m")))
    for n in ns:
        cells = "".join(
            f"{boot_median_width([r[k] for r in rows], n):>14.3f}"
            for k in ("overlap", "scrape", "min", "p05"))
        print(f"{n:>4}{cells}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
