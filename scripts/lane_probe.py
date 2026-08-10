#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Where, in THIS room, can the robot actually walk?

nav_probe answers "did the run scrape", which needs a run. This answers the
question that comes before it: is there a patrol leg at all, and where. It
reads one PATROL_ROI message, takes the occupancy grid the navigator steers
on, and reports:

  top view        the occupancy and the seen floor, as text, so the couch and
                  the corridors are visible without a display
  lane reach      for every candidate lane, how far forward a corridor of the
                  robot's own width stays clear. This is exactly what
                  free_lane asks, at the same width, on the same grid.
  free width      the widest lateral gap at several forward distances, which
                  is what "the corridor is 0.9 m" has to survive
  verdict         RETURN_TO / STOP_AT / DETOUR_MAX that the measurement
                  supports, so the patrol is not ordered across furniture

Nothing here steers anything. It reads the bus and prints numbers.
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
                           grid_extent, unpack_grid)

HALF_WIDTH = float(os.environ.get("ROBOT_HALF_WIDTH", "0.22"))
LANE_SLACK = float(os.environ.get("LANE_SLACK", "0.08"))
DETOUR_MAX = float(os.environ.get("DETOUR_MAX", "1.8"))
# The detour is a shift from the PATROL LANE, not from the world axis. On the
# outbound leg the navigator holds -LANE, so the band it can ask for is
# -LANE +/- DETOUR_MAX. Measuring against |y| <= DETOUR_MAX instead reports a
# lane reachable that the navigator can never request, which is how a corridor
# 39 cm outside the budget looked available.
LANE = float(os.environ.get("LANE", "0.39"))
LANE_BASE = -LANE                     # outbound; the inbound leg mirrors it
RETURN_TO = float(os.environ.get("RETURN_TO", "1.9"))
STOP_AT = float(os.environ.get("STOP_AT", "6.0"))


def lane_reach(occ, x0: float, y: float, look: float, half: float,
               cell: float, bounds, step: float = 0.10) -> float:
    """How far forward from x0 a corridor of half-width `half` stays clear."""
    reach = 0.0
    while reach < look:
        if corridor_blocked(occ, x0, y, 1.0, reach + step, half, cell, bounds):
            return reach
        reach += step
    return look


def widest_gap(occ, x: float, cell: float, bounds, depth: float = 0.20):
    """Widest run of unoccupied lateral cells in the band [x, x+depth].

    Returns (width_m, centre_y). A band and not a single row: one row of a
    0.05 m grid is thinner than the depth noise it was built from.
    """
    nx, ny = occ.shape
    x0, _, y0, _ = bounds
    i0 = max(0, int((x - x0) / cell))
    i1 = min(nx, max(i0 + 1, int((x + depth - x0) / cell)))
    if i0 >= i1:
        return 0.0, 0.0
    col = occ[i0:i1, :].any(axis=0)
    best, best_j, run, start = 0, 0, 0, 0
    for j in range(ny):
        if not col[j]:
            if run == 0:
                start = j
            run += 1
            if run > best:
                best, best_j = run, start
        else:
            run = 0
    if best == 0:
        return 0.0, 0.0
    centre = y0 + (best_j + best / 2.0) * cell
    return best * cell, centre


def free_runs(occ, x: float, cell: float, bounds, depth: float = 0.20,
              min_width: float = 0.40):
    """EVERY free lateral run in the band [x, x+depth], not just the widest.

    The widest run is the open half of the room and says nothing about the
    passages: a lounge can show a 5 m gap on one side while the 0.9 m corridor
    beside the coffee table is the only way through. Returns a list of
    (y_lo, y_hi, width) for runs at least `min_width` wide.
    """
    nx, ny = occ.shape
    x0, _, y0, _ = bounds
    i0 = max(0, int((x - x0) / cell))
    i1 = min(nx, max(i0 + 1, int((x + depth - x0) / cell)))
    if i0 >= i1:
        return []
    col = occ[i0:i1, :].any(axis=0)
    runs, start = [], None
    for j in range(ny + 1):
        free = j < ny and not col[j]
        if free and start is None:
            start = j
        elif not free and start is not None:
            w = (j - start) * cell
            if w >= min_width:
                runs.append((y0 + start * cell, y0 + j * cell, w))
            start = None
    return runs


def top_view(occ, flr, cell: float, bounds, x_max: float = 6.5,
             y_abs: float = 2.8, step: float = 0.20) -> str:
    """Text map. '#' occupied, '.' floor seen and free, ' ' never seen."""
    x0, _, y0, _ = bounds
    rows = []
    ys = np.arange(-y_abs, y_abs + 1e-6, step)
    rows.append("        y  " + "".join(
        "|" if abs(y) < step / 2 else " " for y in ys))
    for x in np.arange(x_max, -1e-6, -step):
        line = []
        for y in ys:
            i = int((x - x0) / cell)
            j = int((y - y0) / cell)
            i2 = min(occ.shape[0], i + int(step / cell))
            j2 = min(occ.shape[1], j + int(step / cell))
            if i < 0 or j < 0 or i >= occ.shape[0] or j >= occ.shape[1]:
                line.append(" ")
            elif occ[i:i2, j:j2].any():
                line.append("#")
            elif flr is not None and flr[i:i2, j:j2].any():
                line.append(".")
            else:
                line.append(" ")
        rows.append("  x=%4.1f  %s" % (x, "".join(line)))
    ticks = "".join("+" if min(abs(y % 1.0), abs(1.0 - y % 1.0)) < step / 2
                    else " " for y in ys)
    rows.append("           " + ticks)
    rows.append("           lateral y from %+.1f m (left of this map) to "
                "%+.1f m, ticks every 1 m" % (-y_abs, y_abs))
    return "\n".join(rows)


KNOB_SOURCE = "from THIS container (env defaults)"


def adopt_live_knobs(sub, timeout_s: float = 4.0) -> None:
    """Replace the env defaults with the values the sim is running.

    The sim puts its navigator's live values on SIM_TELEMETRY once a second.
    Reading them is the difference between reporting on the patrol that IS
    steering and reporting on the one this container happens to default to --
    which is exactly what this probe did when it announced LANE=0.39
    DETOUR_MAX=1.80 STOP_AT=6.00 against a demo running 0 / 2.4 / 4.0, and then
    drew conclusions about a patrol that was not running.
    """
    global LANE, DETOUR_MAX, RETURN_TO, STOP_AT, HALF_WIDTH, KNOB_SOURCE
    end = time.time() + timeout_s
    while time.time() < end:
        m = sub.recv(500)
        if m is None or m[0] != topics.SIM_TELEMETRY:
            continue
        nav = (m[1] or {}).get("nav")
        if not nav:
            continue
        LANE = float(nav.get("LANE", LANE))
        DETOUR_MAX = float(nav.get("DETOUR_MAX", DETOUR_MAX))
        RETURN_TO = float(nav.get("RETURN_TO", RETURN_TO))
        STOP_AT = float(nav.get("STOP_AT", STOP_AT))
        HALF_WIDTH = float(nav.get("ROBOT_HALF_WIDTH", HALF_WIDTH))
        KNOB_SOURCE = "LIVE from the sim"
        return


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0,
                    help="how long to wait for a grid message")
    ap.add_argument("--from-x", type=float, default=0.40,
                    help="forward distance the lane scan starts from")
    ap.add_argument("--look", type=float, default=6.0,
                    help="how far forward the lane scan goes")
    ap.add_argument("--lane-step", type=float, default=0.10)
    ap.add_argument("--gaps-min", type=float, default=0.40,
                    help="narrowest free run worth listing, metres")
    args = ap.parse_args()

    # Liveness topics alongside the one we need, so a silent bus names the
    # service that is quiet instead of reporting "no grid" for every cause.
    # Small messages only: subscribing to the frame topics to prove the
    # compositor is alive would pull tens of KB per frame through a probe.
    watch = {topics.PATROL_ROI: "compositor",
             topics.DETECTIONS: "perception",
             topics.OBSTACLE_MASK: "perception mask",
             topics.SIM_TELEMETRY: "sim",
             topics.ROBOT_STATE: "sim pose"}
    sub = Subscriber(list(watch))
    # Before anything is reported, take the configuration from the sim itself.
    adopt_live_knobs(sub)
    seen = {k: 0 for k in watch}
    deadline = time.time() + args.seconds
    msg = None
    fallback = None
    while time.time() < deadline:
        got = sub.recv(500)
        if not got:
            continue
        topic, body = got
        if topic in seen:
            seen[topic] += 1
        if topic == topics.PATROL_ROI:
            fallback = body
            if body.get("occ"):
                msg = body
                break
    if msg is None:
        print("no occupancy grid on %s in %.0f s." % (topics.PATROL_ROI,
                                                      args.seconds))
        print("what the bus carried meanwhile:")
        for t, name in watch.items():
            print("  %-22s %-16s %d message(s)" % (t, name, seen[t]))
        if not any(seen.values()):
            print("nothing at all. The stack is not running: this probe reads "
                  "a LIVE bus.")
            print("  start the demo in another terminal first:  make")
            print("  then, here:                                make lane-probe")
        elif seen[topics.PATROL_ROI] == 0:
            print("the compositor is not publishing the walkable floor. Check "
                  "its log for 'could not build the walkable floor'.")
        elif seen[topics.DETECTIONS] == 0:
            print("the compositor publishes, but perception sends no "
                  "detections, so no occupancy grid is built. Check the "
                  "perception container.")
        else:
            print("the floor is published WITHOUT a grid: the segmentation "
                  "mask was stale when the polygon was last rebuilt. Move in "
                  "front of the camera to force a rebuild, then rerun.")
        if fallback:
            print()
            print("last floor message, without the grid:")
            _r = fallback.get("roi") or []
            if _r:
                _x = [p[0] for p in _r]
                _y = [p[1] for p in _r]
                print("  roi %d vertices, %.2f-%.2f m ahead, %.2f-%.2f m "
                      "across" % (len(_r), min(_x), max(_x), min(_y), max(_y)))
            print("  %d rectangle(s)" % len(fallback.get("blocked") or []))
        sub.close()
        return 1

    cell = float(msg.get("gcell", GRID_CELL))
    bounds = tuple(float(v) for v in msg.get("gbounds", GRID_BOUNDS))
    nx, ny = int(msg["gnx"]), int(msg["gny"])
    occ = unpack_grid(msg["occ"], nx, ny)
    flr = unpack_grid(msg["flr"], nx, ny) if msg.get("flr") else None
    if occ is None:
        print("the occupancy grid did not unpack")
        return 1

    half = HALF_WIDTH + LANE_SLACK
    roi = msg.get("roi") or []
    raw = msg.get("raw") or []
    blocked = msg.get("blocked") or []

    print("=" * 72)
    # Printed because this ran once against the defaults while the sim was
    # running with other values, and the verdict contradicted the stack with
    # nothing on screen to show why.
    print("knobs %s: LANE=%.2f DETOUR_MAX=%.2f RETURN_TO=%.2f "
          "STOP_AT=%.2f ROBOT_HALF_WIDTH=%.2f LANE_SLACK=%.2f"
          % (KNOB_SOURCE, LANE, DETOUR_MAX, RETURN_TO, STOP_AT, HALF_WIDTH,
             LANE_SLACK))
    if KNOB_SOURCE.startswith("from THIS"):
        print("  WARNING: the sim published no configuration, so these are "
              "this container's defaults and may not be what is steering. "
              "Everything below about the patrol is then about a patrol that "
              "is not running.")
    print("grid %dx%d, %.2f m cell, bounds %s" % (nx, ny, cell, bounds))
    oe, fe = grid_extent(occ, cell, bounds), (
        grid_extent(flr, cell, bounds) if flr is not None else None)
    print("occupied %d cell(s) = %.2f m2 over %s"
          % (int(occ.sum()), occ.sum() * cell * cell,
             "%.1f-%.1f x %.1f-%.1f m" % oe if oe else "none"))
    print("floor    %d cell(s) = %.2f m2 over %s"
          % (int(flr.sum()) if flr is not None else 0,
             (flr.sum() * cell * cell) if flr is not None else 0.0,
             "%.1f-%.1f x %.1f-%.1f m" % fe if fe else "none"))
    for name, poly in (("roi", roi), ("raw", raw)):
        if poly:
            px = [p[0] for p in poly]
            py = [p[1] for p in poly]
            print("%s polygon: %d vertices, %.2f-%.2f m ahead, "
                  "%.2f-%.2f m across" % (name, len(poly), min(px), max(px),
                                          min(py), max(py)))
        else:
            print("%s polygon: empty" % name)
    print("published rectangles: %d" % len(blocked))
    print()
    print(top_view(occ, flr, cell, bounds))
    print()

    # Lane scan, at exactly the width free_lane asks for.
    print("lane reach from x=%.2f m, corridor %.2f m wide "
          "(%.2f half + %.2f slack), look %.1f m:"
          % (args.from_x, 2 * half, HALF_WIDTH, LANE_SLACK, args.look))
    lanes = np.arange(-2.6, 2.6 + 1e-6, args.lane_step)
    reaches = []
    for y in lanes:
        r = lane_reach(occ, args.from_x, float(y), args.look, half, cell,
                       bounds)
        reaches.append(r)
        bar = "#" * int(round(r / 0.1))
        mark = ("  <- outside DETOUR_MAX"
                if abs(y - LANE_BASE) > DETOUR_MAX + 1e-9 else "")
        print("  y %+5.2f m  reach %4.2f m  %s%s" % (y, r, bar, mark))
    reaches = np.array(reaches)

    best_i = int(np.argmax(reaches))
    best_y, best_r = float(lanes[best_i]), float(reaches[best_i])
    inside = np.abs(lanes - LANE_BASE) <= DETOUR_MAX + 1e-9
    bi = int(np.argmax(np.where(inside, reaches, -1.0)))
    in_y, in_r = float(lanes[bi]), float(reaches[bi])

    print()
    print("every free passage wider than %.2f m, per forward distance."
          % args.gaps_min)
    print("a real corridor of W metres appears here as W - 2 x "
          "OBSTACLE_MARGIN, the grid already carries the margin.")
    for x in np.arange(1.5, 5.51, 0.25):
        runs = free_runs(occ, float(x), cell, bounds,
                         min_width=args.gaps_min)
        if not runs:
            print("  x %.2f m  nothing wider than %.2f m" % (x, args.gaps_min))
            continue
        parts = []
        for lo, hi, w in runs:
            ok = "ok" if w >= 2 * half else "narrow"
            parts.append("y %+.2f..%+.2f = %.2f m %s" % (lo, hi, w, ok))
        print("  x %.2f m  %s" % (x, " | ".join(parts)))

    print()
    print("free width at each forward distance (widest lateral gap):")
    for x in np.arange(1.0, 6.01, 0.5):
        w, c = widest_gap(occ, float(x), cell, bounds)
        fits = "fits" if w >= 2 * half else "TOO NARROW"
        print("  x %.1f m  gap %.2f m centred y %+.2f m  (%s for %.2f m)"
              % (x, w, c, fits, 2 * half))

    print()
    print("-" * 72)
    print("best lane anywhere : y %+.2f m, clear to x %.2f m"
          % (best_y, args.from_x + best_r))
    print("best lane reachable: y %+.2f m, clear to x %.2f m"
          % (in_y, args.from_x + in_r))
    print("  reachable band: lane %+.2f m (LANE %.2f) +/- DETOUR_MAX %.2f "
          "= %+.2f to %+.2f m"
          % (LANE_BASE, LANE, DETOUR_MAX, LANE_BASE - DETOUR_MAX,
             LANE_BASE + DETOUR_MAX))
    print("patrol as configured: RETURN_TO %.2f -> STOP_AT %.2f"
          % (RETURN_TO, STOP_AT))
    if oe and RETURN_TO >= oe[0]:
        print("  WARNING: RETURN_TO %.2f m is already inside the occupied "
              "band, which starts at %.2f m." % (RETURN_TO, oe[0]))
    far = args.from_x + in_r
    if far < STOP_AT:
        print("  the reachable lane ends %.2f m short of STOP_AT."
              % (STOP_AT - far))
    if abs(best_y - LANE_BASE) > DETOUR_MAX and best_r > in_r + 0.3:
        print("  the best lane is OUTSIDE the band: DETOUR_MAX %.2f m would "
              "open %.2f m more." % (abs(best_y - LANE_BASE) + 0.2,
                                     best_r - in_r))
    sug_stop = max(1.0, round((args.from_x + in_r - 0.6) * 20) / 20)
    sug_ret = max(0.6, round(min(RETURN_TO, sug_stop - 1.0) * 20) / 20)
    print("suggested for a patrol that walks: RETURN_TO=%.2f STOP_AT=%.2f"
          % (sug_ret, sug_stop))
    print("-" * 72)
    sub.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
