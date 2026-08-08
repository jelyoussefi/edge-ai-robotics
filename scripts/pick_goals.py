#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Pick goals that are actually reachable from one another, from the map.

Etape E2 failed its first acceptance run because the goals were chosen one at a
time. Each had clearance, each was plannable from the START, and goal 2 was not
plannable from where goal 1 left the robot: the right-hand corridor does not
admit a 0.22 m robot with 0.35 m of inflation. **Reachability here is pairwise,
so the goals have to be chosen as a mutually reachable SET.**

The test is connectivity of the traversable free space, which is what a costmap
reduces to: a cell is traversable if its distance to the nearest occupied cell
is at least the clearance the robot needs, and two cells are mutually plannable
if they lie in the same connected component of that set. That is a necessary
condition rather than a proof -- the planner's roadmap is probabilistic and may
still fail to find a route it could have found -- but the converse is exact:
different components means no path exists, whatever the planner does.

Reporting the component structure ACROSS thresholds is the other half of the
job, and the more interesting one. It says whether the robot fits this room.

    make pick-goals
    make pick-goals ARGS="--clearance 0.30 --n 3"
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, "common")
sys.path.insert(0, "/opt/edgebot")

from edgebot import topics
from edgebot.bus import Subscriber

ARENA = (1.5, 6.5, -2.6, 1.5)
# Where the robot enters the mapped floor, and so the cell every goal has to be
# reachable FROM. Not a free choice: goal mode bootstraps straight out from
# under the camera and joins the floor at about RETURN_TO on the axis.
ENTRY = (1.9, 0.0)


def main() -> int:
    import cv2

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--clearance", type=float, default=0.30,
                    help="metres a goal must have to the nearest occupied cell")
    ap.add_argument("--n", type=int, default=3)
    args = ap.parse_args()

    sub = Subscriber([topics.SUITE_MAP])
    m = None
    deadline = time.time() + args.seconds
    while time.time() < deadline and m is None:
        msg = sub.recv(300)
        if msg is not None:
            m = msg[1]
    if m is None:
        print("  rien recu sur suite.map. `make fastmapping` tourne-t-il ?")
        return 1

    w, h = int(m["w"]), int(m["h"])
    grid = np.frombuffer(m["grid"], np.int8).reshape(h, w)
    res, x0, y0 = float(m["res"]), float(m["x0"]), float(m["y0"])
    xs = x0 + np.arange(w) * res
    ys = y0 + np.arange(h) * res
    X, Y = np.meshgrid(xs, ys)
    ax0, ax1, ay0, ay1 = ARENA
    in_arena = (X >= ax0) & (X <= ax1) & (Y >= ay0) & (Y <= ay1)

    # Unknown counts as blocked. track_unknown_space is true in the costmap, and
    # a goal in never-observed space is exactly the mistake E1 made with
    # (5.8, -1.8): plausible on a picture, refused by the planner.
    free = (grid == 0)
    blocked = (~free).astype(np.uint8)
    # Distance to the nearest blocked cell, in metres.
    dist = cv2.distanceTransform((1 - blocked).astype(np.uint8),
                                 cv2.DIST_L2, 5) * res

    print(f"\n  === LE ROBOT TIENT-IL DANS CETTE PIECE ? ===\n")
    print(f"    grille {w}x{h} a {res:.3f} m, arene x {ax0}-{ax1}, "
          f"y {ay0} a {ay1}")
    print(f"\n    {'degagement exige':>18s} {'surface libre':>14s} "
          f"{'composantes':>12s} {'plus grande':>12s}")
    entry_rc = (int(round((ENTRY[1] - y0) / res)),
                int(round((ENTRY[0] - x0) / res)))
    comps = {}
    for thr in (0.22, 0.25, 0.30, 0.35, 0.40):
        ok = (dist >= thr) & in_arena
        n, lab = cv2.connectedComponents(ok.astype(np.uint8), connectivity=8)
        sizes = [(int((lab == i).sum()), i) for i in range(1, n)]
        sizes.sort(reverse=True)
        big = sizes[0] if sizes else (0, 0)
        comps[thr] = (lab, big[1])
        print(f"    {thr:15.2f} m {ok.sum() * res * res:11.2f} m2 "
              f"{max(0, n - 1):12d} {big[0] * res * res:9.2f} m2")

    thr = args.clearance
    lab, big_id = comps.get(thr, (None, 0))
    if lab is None:
        ok = (dist >= thr) & in_arena
        _, lab = cv2.connectedComponents(ok.astype(np.uint8), connectivity=8)
        big_id = 1
    entry_lab = int(lab[entry_rc])
    print(f"\n    entree du robot ({ENTRY[0]}, {ENTRY[1]}) : composante "
          f"{entry_lab}" + ("" if entry_lab else "  -- HORS de l'espace "
                            "traversable, aucun but ne sera atteignable"))
    if entry_lab == 0:
        # Fall back to the largest component and say so: the entry cell itself
        # may be marginal while the corridor beyond it is fine.
        entry_lab = big_id
        print(f"    repli sur la plus grande composante ({entry_lab})")

    reach = (lab == entry_lab)
    cand = np.argwhere(reach)
    if len(cand) < args.n:
        print("    pas assez de cellules atteignables pour choisir des buts")
        return 1
    pts = np.stack([x0 + cand[:, 1] * res, y0 + cand[:, 0] * res], axis=1)
    clr = dist[cand[:, 0], cand[:, 1]]

    # Greedy farthest-point sampling, seeded from the entry: maximise spread
    # while every pick stays in the one component, so the whole sequence
    # start -> g1 -> g2 -> g3 is plannable by construction.
    chosen = []
    ref = np.array(ENTRY)
    dmin = np.hypot(pts[:, 0] - ref[0], pts[:, 1] - ref[1])
    for _ in range(args.n):
        i = int(np.argmax(dmin))
        chosen.append((float(pts[i, 0]), float(pts[i, 1]), float(clr[i])))
        d = np.hypot(pts[:, 0] - pts[i, 0], pts[:, 1] - pts[i, 1])
        dmin = np.minimum(dmin, d)

    print(f"\n  === {args.n} BUTS, MEME COMPOSANTE, DEGAGEMENT >= {thr} m ===\n")
    seq = [ENTRY] + [(gx, gy) for gx, gy, _ in chosen]
    for k, (gx, gy, c) in enumerate(chosen, 1):
        print(f"    but {k} : ({gx:5.2f}, {gy:5.2f})  degagement {c:.3f} m  "
              f"saut precedent {np.hypot(gx - seq[k - 1][0], gy - seq[k - 1][1]):.2f} m")
    print("\n    ITS_GOALS=" + ";".join(f"{gx:.2f},{gy:.2f}"
                                        for gx, gy, _ in chosen))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
