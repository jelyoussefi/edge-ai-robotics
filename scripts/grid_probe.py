#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Measure the grid navigation against the rectangle one, off the hardware.

Runs both representations over the same synthetic lounge and the same 60 s of
patrol, so the change can be judged before the stack is rebuilt and without
waiting for the camera, the policy and the renderer to agree.

It is a MODEL, and it says so: the kinematics are a unicycle, not the RL
policy, so the distances are not the ones the real robot walks. What it does
measure honestly is the decision -- whether a lane exists, which one is taken,
and whether the line taken crosses an obstacle's real outline. That is the part
that was broken, and it is the part that is testable here.

The room is built to the measurements of the lounge the demo runs in: an
L-shaped couch along the far wall and down one side, a coffee table in front of
it, 0.9 m of free floor each side of the table and 1.1 m behind it.

  python3 scripts/grid_probe.py
"""
import logging
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "common"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "services",
                                "sim"))

from edgebot.floor import (GRID_BOUNDS, GRID_CELL, corridor_blocked,
                           grid_shape, pack_grid, points_to_grid)

SECONDS = float(os.environ.get("PROBE_SECONDS", "60"))
HZ = 50.0


def room():
    """Occupancy of the lounge, with and without the obstacle margin."""
    rng = np.random.default_rng(0)

    def block(x0, x1, y0, y1, n=12000):
        return rng.uniform(x0, x1, n), rng.uniform(y0, y1, n)

    # Both sides bounded. An earlier version of this room had no wall on the
    # left, so every lane pushed that way was clear out to the edge of the
    # grid and the corridor width being asked for was never tested against
    # anything. It reported a working detour while the real room, bounded on
    # both sides, had no lane wide enough at all. A test room open on one side
    # is not a room.
    parts = [(2.8, 3.8, -0.5, 0.5),      # coffee table, 1.0 x 1.0 m
             (4.9, 5.8, -2.3, 1.4),      # couch, back arm, 0.9 m deep
             (2.6, 5.8, 1.4, 2.3),       # couch, side arm, 0.9 m wide
             (1.4, 5.8, -2.4, -1.4),     # TV unit and pillar, left side
             (1.4, 5.8, 2.3, 2.5)]       # right wall

    # Free floor either side of the table: 1.4 - 0.5 = 0.90 m, both sides, the
    # measurement this whole exercise turns on.
    xs, ys = zip(*[block(*p) for p in parts])
    fwd, lat = np.concatenate(xs), np.concatenate(ys)
    sel = np.ones(fwd.shape, bool)
    occ = points_to_grid(fwd, lat, sel, GRID_CELL, GRID_BOUNDS,
                         margin=0.12, passable=0.44)
    # The real outline, no margin: what a scrape is measured against. Scoring
    # against the margined grid would score the margin, not the collision.
    raw = points_to_grid(fwd, lat, sel, GRID_CELL, GRID_BOUNDS)
    return occ, raw, fwd, lat


def run(rep, occ, raw, fwd, lat):
    for mod in ("navigator",):
        sys.modules.pop(mod, None)
    import navigator as N
    N.Navigator.REP = rep
    nav = N.Navigator()
    nx, ny = grid_shape()
    nav.set_grid({"occ": pack_grid(occ), "flr": b"", "gnx": nx, "gny": ny,
                  "gcell": GRID_CELL, "gbounds": list(GRID_BOUNDS)})
    if rep == "rects":
        # One bounding box over the same object, which is what the rectangle
        # path produces for a connected L: the control this exists to beat.
        nav.set_floor([], [(float(fwd.min()) - 0.12, float(fwd.max()) + 0.12,
                            float(lat.min()) - 0.12, float(lat.max()) + 0.12)],
                      [0])
    ix, iy = np.nonzero(raw)
    px = GRID_BOUNDS[0] + ix * GRID_CELL
    py = GRID_BOUNDS[2] + iy * GRID_CELL
    x, y, yaw, dt = 1.9, -0.39, 0.0, 1.0 / HZ
    dist, hits, clr, xs = 0.0, 0, [], []
    for k in range(int(SECONDS * HZ)):
        pose = N.Pose(lead=(x + 0.25 * np.cos(yaw), y + 0.25 * np.sin(yaw)),
                      centre=(x, y), yaw=yaw)
        cmd = nav.step(pose, dt)
        vx, wz = float(cmd[0]), float(cmd[2])
        yaw += wz * dt
        x += vx * np.cos(yaw) * dt
        y += vx * np.sin(yaw) * dt
        dist += abs(vx) * dt
        xs.append(x)
        if corridor_blocked(raw, x, y, 1.0, 0.0, 0.22, behind=0.0):
            hits += 1
        if k % 25 == 0:
            clr.append(float(np.hypot(px - x, py - y).min()) - 0.22)
    return dist, 100.0 * hits / (SECONDS * HZ), min(clr), min(xs), max(xs)


def main():
    logging.basicConfig(level=logging.CRITICAL)
    occ, raw, fwd, lat = room()
    aabb = ((fwd.max() - fwd.min()) * (lat.max() - lat.min()))
    cells = float(occ.sum()) * GRID_CELL ** 2
    print("obstacle area: %.2f m2 as one bounding box, %.2f m2 as cells"
          % (aabb, cells))
    print("  the box claims %.2f m2 of floor the objects do not occupy" %
          (aabb - cells))
    print()
    print("%-6s %10s %10s %14s %14s" %
          ("rep", "travelled", "scraping", "worst clear", "x range"))
    for rep in ("rects", "grid"):
        d, s, c, lo, hi = run(rep, occ, raw, fwd, lat)
        print("%-6s %8.2f m %8.1f %% %11.3f m %8.2f-%.2f m"
              % (rep, d, s, c, lo, hi))


if __name__ == "__main__":
    main()
