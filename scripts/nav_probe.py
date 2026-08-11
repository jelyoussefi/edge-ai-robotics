#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Measure what the navigator actually has to walk in, and whether it scrapes.

Written for the footprint-merge problem in a small room: obstacle rectangles
that are merged when the gap between them is too narrow for the robot turn two
pieces of furniture into one barrier, and in a tight lounge that can close every
route and collapse the walkable floor to a band.

Reports, over a window:

  walkable width   the lateral extent of the PATROL_ROI polygon, sampled at
                   several forward distances. This is the number that says
                   whether there is anywhere to go, and a single area figure
                   hides a band that is wide at one end and shut at the other.
  footprints       how many blocked rectangles the navigator sees, and how big
                   the largest is -- a merge shows up here as a count that
                   falls while the largest span grows.
  clearance        for every robot pose, the distance from the robot centre to
                   the nearest footprint edge. Negative means inside one.
  scrapes          poses closer than ROBOT_HALF_WIDTH to a footprint, i.e. the
                   robot's body overlapping something it was told to avoid.
  travel           lateral spread of the path, which is how "did it route
                   around the table" shows up as a number rather than a memory.

Clearance is measured against the OCCUPANCY GRID when one is published, and
against the rectangles only when it is not. That distinction is the whole
point: with OBSTACLE_REP=grid the navigator steers on cells, so measuring the
robot against rectangles reports the clearance of a representation nobody uses.
This repository has already paid for that mistake once, when the footprint was
computed twice and the floor mask and the navigator disagreed about the couch
by three quarters of a metre while both looked fine on their own.

Against the furniture is a different question again, and this cannot see it: a
footprint or a cell that is wrong is a perception problem.
"""
from __future__ import annotations

import argparse
import math
import os
import statistics

import numpy as np
import time

from edgebot import topics
from edgebot.bus import Subscriber
from edgebot.floor import (assert_same_corridor, min_corridor, query_half,
                           query_pad, unpack_grid)

HALF_WIDTH = float(os.environ.get("ROBOT_HALF_WIDTH", "0.22"))
CLEARANCE = float(os.environ.get("CLEARANCE", "0.12"))
CLEARANCE_MODE = os.environ.get("CLEARANCE_MODE", "query")


def grid_distance(x: float, y: float, occ, cell: float, bounds) -> float | None:
    """Distance from a point to the nearest occupied cell, negative if inside.

    Brute force over the occupied cells. There are a few thousand of them and
    this runs at the robot's publish rate on a measurement machine, so an
    exact answer is worth more than a distance transform here.

    Returns None when nothing is occupied: no obstacles is not zero clearance,
    and reporting 0.0 would put a false scrape in the table.
    """
    if occ is None or not occ.any():
        return None
    x0, _x1, y0, _y1 = bounds
    ix = np.nonzero(occ)
    # Cell centres.
    cx = x0 + (ix[0] + 0.5) * cell
    cy = y0 + (ix[1] + 0.5) * cell
    d = np.hypot(cx - x, cy - y).min() - 0.5 * cell
    # Inside an occupied cell: report how deep, so the sign matches the
    # rectangle path and a scrape is comparable between the two.
    gi = int((x - x0) / cell)
    gj = int((y - y0) / cell)
    if 0 <= gi < occ.shape[0] and 0 <= gj < occ.shape[1] and occ[gi, gj]:
        return -abs(d) if d != 0 else -0.5 * cell
    return float(d)


def rect_distance(x: float, y: float, r) -> float:
    """Distance from a point to a rectangle [x0, x1, y0, y1]; <0 when inside."""
    x0, x1, y0, y1 = r
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx == 0.0 and dy == 0.0:
        # Inside: the negative distance to the nearest edge.
        return -min(x - x0, x1 - x, y - y0, y1 - y)
    return math.hypot(dx, dy)


def poly_width_at(poly, x: float, tol: float = 0.25):
    """Lateral extent of the polygon near a forward distance x.

    Sampled from the vertices rather than by clipping: the polygon comes from a
    depth contour and has enough vertices that a band around x captures its
    width, and a clipper would add a dependency for no extra truth.
    """
    ys = [p[1] for p in poly if abs(p[0] - x) <= tol]
    return (max(ys) - min(ys)) if len(ys) >= 2 else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    global HALF_WIDTH, CLEARANCE, CLEARANCE_MODE

    sub = Subscriber([topics.PATROL_ROI, topics.ROBOT_STATE,
                      topics.SIM_TELEMETRY])
    laps: list = []
    # Filled from the first telemetry message, then checked. Until it is, no
    # scrape verdict is printed: a scrape threshold is a width, and reporting
    # one measured at a width the robot does not use is how this probe already
    # reported 21 % of poses scraping against a representation nobody steers on.
    nav_corridor = None
    checked = False
    trace: list = []
    stalled_seen = 0
    occ = None
    gcell = 0.05
    gbounds = (0.0, 8.0, -4.0, 4.0)
    measured_against = "nothing yet"
    poses: list[tuple[float, float]] = []
    clearances: list[float] = []
    scrapes: list[tuple[float, float, float]] = []
    contacts: list[tuple[float, float, float]] = []
    widths: dict[float, list[float]] = {}
    counts: list[int] = []
    spans: list[float] = []
    blocked: list = []
    roi = None

    t0 = time.time()
    while time.time() - t0 < args.seconds:
        msg = sub.recv(500)
        if msg is None:
            continue
        topic, p = msg
        if topic == topics.SIM_TELEMETRY:
            nav = (p or {}).get("nav") or {}
            if nav:
                trace.append(("nav", time.time() - t0,
                              nav.get("lane"), nav.get("last_lane"),
                              bool(nav.get("stalled")), bool(nav.get("blocked")),
                              nav.get("lap")))
            if not checked and nav.get("MIN_CORRIDOR") is not None:
                checked = True
                HALF_WIDTH = float(nav.get("ROBOT_HALF_WIDTH", HALF_WIDTH))
                CLEARANCE = float(nav.get("CLEARANCE", CLEARANCE))
                CLEARANCE_MODE = nav.get("CLEARANCE_MODE", CLEARANCE_MODE)
                nav_corridor = float(nav["MIN_CORRIDOR"])
                assert_same_corridor(
                    {"GRID_HALF": query_half(HALF_WIDTH, CLEARANCE,
                                             CLEARANCE_MODE),
                     "GRID_PAD": query_pad(CLEARANCE, CLEARANCE_MODE),
                     "MIN_CORRIDOR": min_corridor(HALF_WIDTH, CLEARANCE)},
                    {k: nav.get(k) for k in
                     ("GRID_HALF", "GRID_PAD", "MIN_CORRIDOR")}, "nav_probe")
            if "lap" in nav:
                laps.append(int(nav["lap"]))
            if nav.get("stalled"):
                stalled_seen += 1
            continue
        if topic == topics.PATROL_ROI:
            # Per SAMPLE, not once at the end. "The grid emptied" and "the
            # polygon collapsed" look identical in a summary and are different
            # defects: one is the map the robot steers on, the other is a
            # picture. Recording both every time is the only way to tell which
            # moved, and when relative to the robot going somewhere it should
            # not have.
            _roi_pts = [(float(a), float(b)) for a, b in (p.get("roi") or [])]
            _cells = None
            if p.get("occ") and p.get("gnx"):
                _g = unpack_grid(p["occ"], int(p["gnx"]), int(p["gny"]))
                _cells = int(_g.sum()) if _g is not None else None
            trace.append(("roi", time.time() - t0, _cells,
                          (min(q[0] for q in _roi_pts),
                           max(q[0] for q in _roi_pts),
                           min(q[1] for q in _roi_pts),
                           max(q[1] for q in _roi_pts)) if _roi_pts else None,
                          None, None, None))
            if p.get("occ") and p.get("gnx"):
                occ = unpack_grid(p["occ"], int(p["gnx"]), int(p["gny"]))
                gcell = float(p.get("gcell", gcell))
                gbounds = tuple(p.get("gbounds", gbounds))
                measured_against = "the occupancy grid"
            roi = [(float(a), float(b)) for a, b in (p.get("roi") or [])]
            blocked = [[float(v) for v in r] for r in (p.get("blocked") or [])]
            counts.append(len(blocked))
            if blocked:
                spans.append(max(max(r[1] - r[0], r[3] - r[2]) for r in blocked))
            for x in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
                w = poly_width_at(roi, x)
                if w is not None:
                    widths.setdefault(x, []).append(w)
        elif topic == topics.ROBOT_STATE:
            q = p.get("qpos") or []
            if len(q) < 2:
                continue
            x, y = float(q[0]), float(q[1])
            poses.append((x, y))
            d = None
            if occ is not None:
                d = grid_distance(x, y, occ, gcell, gbounds)
            elif blocked:
                d = min(rect_distance(x, y, r) for r in blocked)
                measured_against = "the published rectangles (NO GRID ON THE "
                measured_against += "BUS -- this is not what the navigator uses)"
            if d is not None:
                clearances.append(d)
                # The SAME width the navigator asked the grid for. In dilate
                # mode the cells already hold the clearance, so the threshold
                # is the bare body; in query mode they do not, so it is body
                # plus clearance. Getting this from query_half rather than
                # rewriting it here is the point: the two used to be written
                # out separately and drifted.
                if d < query_half(HALF_WIDTH, CLEARANCE, CLEARANCE_MODE):
                    scrapes.append((x, y, d))
                # A SECOND, harder threshold. Eating into the clearance and
                # actually overlapping the furniture are different events and
                # only one of them is a collision: at CLEARANCE 0.12 the first
                # starts 0.12 m before the second. Reporting one number made a
                # pass through a 0.90 m corridor read as 41.9 % scraping when
                # the body never touched anything -- true, alarming, and the
                # wrong alarm.
                if d < HALF_WIDTH:
                    contacts.append((x, y, d))
    sub.close()

    lab = args.label or "nav"
    print(f"=== {lab}: {args.seconds:.0f} s, {len(poses)} poses, "
          f"{len(counts)} ROI updates ===")
    print(f"clearance measured against: {measured_against}")
    if laps:
        print(f"laps covered: {min(laps)} to {max(laps)}"
              + (f"  STALLED reported in {stalled_seen} telemetry sample(s)"
                 if stalled_seen else ""))
    else:
        print("laps covered: unmeasured (no sim telemetry seen)")

    print("walkable floor width (lateral extent of the ROI polygon):")
    if widths:
        for x in sorted(widths):
            v = widths[x]
            print(f"   at x={x:.1f} m : median {statistics.median(v):.2f} m "
                  f"(min {min(v):.2f}, max {max(v):.2f}, n={len(v)})")
    else:
        print("   the ROI polygon never reached these distances")

    if roi:
        xs = [p[0] for p in roi]
        ys = [p[1] for p in roi]
        print(f"   ROI bounding box: x {min(xs):.2f}..{max(xs):.2f} m, "
              f"y {min(ys):.2f}..{max(ys):.2f} m")

    if counts:
        print(f"footprints: median {statistics.median(counts):.0f} "
              f"(min {min(counts)}, max {max(counts)}); largest single span "
              f"median {statistics.median(spans):.2f} m" if spans else "")

    if clearances:
        cl = sorted(clearances)
        print(f"clearance to the nearest footprint: min {cl[0]:+.3f} m, "
              f"p05 {cl[max(0, int(0.05 * len(cl)))]:+.3f}, "
              f"median {statistics.median(cl):+.3f}")
        _qh = query_half(HALF_WIDTH, CLEARANCE, CLEARANCE_MODE)
        print(f"corridor demanded: {min_corridor(HALF_WIDTH, CLEARANCE):.2f} m "
              f"of real floor, grid queried at {_qh:.2f} m "
              f"(CLEARANCE {CLEARANCE:.2f} in {CLEARANCE_MODE} mode"
              + (", CHECKED against the navigator)" if checked
                 else ", NOT checked -- no sim telemetry seen)"))
        print(f"scrapes (closer than {_qh:.2f} m): "
              f"{len(scrapes)} of {len(clearances)} poses "
              f"({100.0 * len(scrapes) / len(clearances):.1f} %)")
        print(f"body overlap (closer than ROBOT_HALF_WIDTH="
              f"{HALF_WIDTH:.2f} m, i.e. the robot IS in the furniture): "
              f"{len(contacts)} of {len(clearances)} poses "
              f"({100.0 * len(contacts) / len(clearances):.1f} %)")
        if scrapes:
            worst = min(scrapes, key=lambda s: s[2])
            print(f"   worst at ({worst[0]:.2f}, {worst[1]:.2f}) "
                  f"clearance {worst[2]:+.3f} m")
    else:
        print("clearance: no footprints were published while the robot moved")

    # ---- what moved, and when -------------------------------------------
    rois = [t for t in trace if t[0] == "roi"]
    navs = [t for t in trace if t[0] == "nav"]
    if rois:
        cells = [t[2] for t in rois if t[2] is not None]
        boxes = [t[3] for t in rois if t[3] is not None]
        print()
        if cells:
            cs = sorted(cells)
            print(f"occupied cells per ROI message: median {cs[len(cs) // 2]}, "
                  f"range {cs[0]}-{cs[-1]} over {len(cs)} samples")
        if boxes:
            depth = sorted(b[1] - b[0] for b in boxes)
            far = sorted(b[1] for b in boxes)
            print(f"ROI polygon depth: median {depth[len(depth) // 2]:.2f} m, "
                  f"range {depth[0]:.2f}-{depth[-1]:.2f}; far edge median "
                  f"{far[len(far) // 2]:.2f} m, range {far[0]:.2f}-{far[-1]:.2f}")
            collapsed = sum(1 for b in boxes if b[1] - b[0] < 1.0)
            print(f"ROI collapsed (< 1.0 m deep) in {collapsed} of "
                  f"{len(boxes)} samples ({100.0 * collapsed / len(boxes):.0f} %)")
    if navs:
        lanes = [t[3] for t in navs if t[3] is not None]
        if lanes:
            ls = sorted(lanes)
            print(f"lane held by the navigator: median {ls[len(ls) // 2]:+.2f} m,"
                  f" range {ls[0]:+.2f} to {ls[-1]:+.2f}")
        st = [t for t in navs if t[4]]
        print(f"STALLED in {len(st)} of {len(navs)} telemetry samples")

    if poses:
        xs = [p[0] for p in poses]
        ys = [p[1] for p in poses]
        print(f"travel: x {min(xs):.2f}..{max(xs):.2f} m "
              f"(span {max(xs) - min(xs):.2f}), y {min(ys):.2f}..{max(ys):.2f} m "
              f"(lateral span {max(ys) - min(ys):.2f})")


if __name__ == "__main__":
    main()
