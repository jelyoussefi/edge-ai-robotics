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


# ---------------------------------------------------------------------------
# Etape 6: the bounds guard, on BOTH published representations.
#
# The recurring defect of this repository is a guard applied to one of two
# paths. Three documented occurrences, and this case pins the two that are
# testable off the hardware:
#
#   - clip_footprints protected the RECTANGLES with FOOTPRINT_X_MAX while
#     nothing protected the GRID, which is what the navigator steers on.
#     Measured "ground grid 1.2-7.8 m" in a room whose far wall is at 6.2 m.
#   - the mirror of it: the grid gained a near guard, OBSTACLE_X_MIN, and
#     clip_footprints never did, so a rectangle could be published at 1.0 m in
#     front of a camera that cannot see the floor closer than 1.4 m.
#
# One mask, points at 1.0 m and 7.5 m, and BOTH outputs checked. A test that
# checked one of them would have passed throughout the entire history of both
# bugs, which is the whole point.
# ---------------------------------------------------------------------------

X_MIN, X_MAX = 1.4, 6.5


def bounds_case() -> int:
    """Points outside the arena must reach neither the grid nor the rectangles."""
    from edgebot.floor import clip_footprints, grid_extent, in_band

    # Three clusters: one too near, one legitimate, one past the far wall.
    fwd = np.concatenate([np.full(400, 1.0), np.full(400, 3.5),
                          np.full(400, 7.5)])
    lat = np.concatenate([np.linspace(-0.3, 0.3, 400)] * 3)
    sel = in_band(fwd, X_MIN, X_MAX)

    occ = points_to_grid(fwd, lat, sel, GRID_CELL, GRID_BOUNDS)
    ext = grid_extent(occ, GRID_CELL, GRID_BOUNDS)

    # The rectangles the same points would produce, unguarded, then guarded by
    # the SAME two numbers.
    raw = [(0.9, 1.1, -0.3, 0.3), (3.4, 3.6, -0.3, 0.3), (7.4, 7.6, -0.3, 0.3)]
    boxes = clip_footprints(raw, X_MAX, 0.0, x_min=X_MIN)

    bad = []
    print(f"arena {X_MIN}-{X_MAX} m; points at 1.0, 3.5 and 7.5 m")
    print(f"  grid extent      {ext}")
    if ext is None:
        bad.append("the grid is empty: the legitimate cluster was dropped too")
    else:
        if ext[0] < X_MIN:
            bad.append(f"grid starts at {ext[0]:.2f} m, inside {X_MIN} m")
        if ext[1] > X_MAX:
            bad.append(f"grid reaches {ext[1]:.2f} m, past {X_MAX} m")
    print(f"  rectangles       {[tuple(round(v, 2) for v in b) for b in boxes]}")
    if not boxes:
        bad.append("every rectangle was dropped, including the legitimate one")
    for b in boxes:
        if b[0] < X_MIN - 1e-9:
            bad.append(f"a rectangle starts at {b[0]:.2f} m, inside {X_MIN} m")
        if b[1] > X_MAX + 1e-9:
            bad.append(f"a rectangle reaches {b[1]:.2f} m, past {X_MAX} m")
    # The legitimate obstacle must SURVIVE. A guard that drops everything
    # passes every bounds check ever written and blinds the robot.
    if not any(abs(b[0] - 3.4) < 1e-6 for b in boxes):
        bad.append("the legitimate rectangle at 3.5 m did not survive")
    if ext is not None and not (ext[0] <= 3.5 <= ext[1]):
        bad.append("the legitimate cluster is missing from the grid")

    for m in bad:
        print(f"  FAIL: {m}")
    print("  bounds case: " + ("FAILED" if bad else "passed"))
    return 1 if bad else 0


if __name__ == "__main__" and os.environ.get("PROBE_CASE") == "bounds":
    raise SystemExit(bounds_case())
