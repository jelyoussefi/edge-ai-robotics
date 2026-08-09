#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Height above the floor, cell by cell, over the ground plane.

For the question "why is that object not an obstacle": the floor test here is a
HEIGHT gate -- a pixel is floor when |height above the ground plane| is under
floor_h_tol -- so anything shorter than the tolerance is floor by definition,
and an obstacle that reads as floor is the same limit seen from the other side.

Deprojects every depth pixel to world (forward, lateral, up) using exactly the
calibration the compositor uses, bins it onto the ground plane, and prints the
height of each cell. An object shows up as a block of cells standing above its
neighbours; the floor shows up as a field near zero. Nothing has to be told
where the furniture is.

Also answers, for a chosen point: is it inside the published walkable polygon
(so the robot believes it may walk there), and inside the raw floor polygon
(so the geometry believes it IS floor).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time

import numpy as np

from edgebot import topics
from edgebot.bus import Subscriber

CALIB = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")


def deproject(depth_m, calib):
    """Depth image -> (forward, lateral, up) per pixel, world frame.

    The inverse of the compositor's _cloud_to_pixels, same angles, same signs:
    world X forward, Y left, Z up, camera at height H tilted down by pitch.
    """
    i = calib["intrinsics"]
    H = float(calib["camera_height_m"])
    p = math.radians(abs(float(calib["pitch_deg"])))
    cp, sp = math.cos(p), math.sin(p)
    dh, dw = depth_m.shape
    # Intrinsics scale to whatever raster the depth arrived at.
    fx = float(i["fx"]) * dw / float(i["width"])
    fy = float(i["fy"]) * dh / float(i["height"])
    ppx = float(i["ppx"]) * dw / float(i["width"])
    ppy = float(i["ppy"]) * dh / float(i["height"])
    uu, vv = np.meshgrid(np.arange(dw), np.arange(dh))
    xn = (uu - ppx) / fx
    yn = (vv - ppy) / fy
    z = depth_m
    fwd = z * (cp - yn * sp)
    lat = -xn * z
    up = H - z * (sp + yn * cp)
    return fwd, lat, up


def point_in_poly(x, y, poly) -> bool:
    inside = False
    n = len(poly)
    for k in range(n):
        x0, y0 = poly[k]
        x1, y1 = poly[(k + 1) % n]
        if (y0 > y) != (y1 > y):
            xin = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < xin:
                inside = not inside
    return inside


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--x0", type=float, default=1.5)
    ap.add_argument("--x1", type=float, default=5.0)
    ap.add_argument("--y0", type=float, default=-2.0)
    ap.add_argument("--y1", type=float, default=2.0)
    ap.add_argument("--cell", type=float, default=0.25)
    ap.add_argument("--at", default="", help="x,y to interrogate in detail")
    args = ap.parse_args()

    with open(CALIB) as fh:
        calib = json.load(fh)
    h_tol = float(calib.get("floor_h_tol_m", 0.08))
    print(f"calibration: height {calib['camera_height_m']} m, "
          f"pitch {calib['pitch_deg']} deg, floor_h_tol {h_tol} m, "
          f"intrinsics {calib['intrinsics']['width']}x"
          f"{calib['intrinsics']['height']}")

    sub = Subscriber([topics.CAMERA_DEPTH, topics.PATROL_ROI, topics.DETECTIONS])
    nx = int(round((args.x1 - args.x0) / args.cell))
    ny = int(round((args.y1 - args.y0) / args.cell))
    hi = np.full((ny, nx), -np.inf, np.float32)   # 90th pct height per cell
    cnt = np.zeros((ny, nx), np.int64)
    acc: dict[tuple[int, int], list] = {}
    roi = raw = None
    blocked: list = []
    dets: list = []

    t0 = time.time()
    frames = 0
    while time.time() - t0 < args.seconds:
        msg = sub.recv(500)
        if msg is None:
            continue
        topic, pay = msg
        if topic == topics.PATROL_ROI:
            roi = [(float(a), float(b)) for a, b in (pay.get("roi") or [])]
            raw = [(float(a), float(b)) for a, b in (pay.get("raw") or [])]
            blocked = [[float(v) for v in r] for r in (pay.get("blocked") or [])]
        elif topic == topics.DETECTIONS:
            dets = pay.get("detections") or []
        elif topic == topics.CAMERA_DEPTH:
            frames += 1
            if frames % 3:           # every third depth frame is plenty
                continue
            d = np.frombuffer(pay["depth"], np.uint16).reshape(
                pay["h"], pay["w"]).astype(np.float32) * pay.get("scale", 0.001)
            fwd, lat, up = deproject(d, calib)
            ok = (d > 0.3) & (d < 8.0)
            ix = ((fwd - args.x0) / args.cell).astype(np.int32)
            iy = ((lat - args.y0) / args.cell).astype(np.int32)
            ok &= (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            for a, b, c in zip(iy[ok], ix[ok], up[ok]):
                acc.setdefault((int(a), int(b)), []).append(float(c))
    sub.close()

    for (a, b), vals in acc.items():
        cnt[a, b] = len(vals)
        hi[a, b] = float(np.percentile(vals, 90))

    print(f"\nheight above the floor, 90th percentile per {args.cell:.2f} m cell "
          f"(cm; '.' = no depth, lateral y across, forward x down)")
    print("        " + "".join(f"{args.y0 + (j + 0.5) * args.cell:>6.1f}"
                               for j in range(ny)))
    for b in range(nx):
        x = args.x0 + (b + 0.5) * args.cell
        row = ""
        for a in range(ny):
            row += ("     ." if cnt[a, b] < 20
                    else f"{100.0 * hi[a, b]:>6.0f}")
        print(f"x={x:4.2f} " + row)

    print(f"\nfloor gate: a cell is 'floor' when |height| < {100 * h_tol:.0f} cm")

    if args.at:
        qx, qy = (float(v) for v in args.at.split(","))
        a = int((qy - args.y0) / args.cell)
        b = int((qx - args.x0) / args.cell)
        print(f"\n--- at ({qx:.2f}, {qy:.2f}) ---")
        if 0 <= a < ny and 0 <= b < nx and (a, b) in acc:
            v = np.array(acc[(a, b)])
            qs = np.percentile(v, [5, 25, 50, 75, 90, 99])
            print(f"  {len(v)} depth points; height above floor (cm): "
                  f"p05 {100*qs[0]:.1f}  p25 {100*qs[1]:.1f}  "
                  f"med {100*qs[2]:.1f}  p75 {100*qs[3]:.1f}  "
                  f"p90 {100*qs[4]:.1f}  p99 {100*qs[5]:.1f}  "
                  f"max {100*v.max():.1f}")
            frac = float((np.abs(v) < h_tol).mean())
            print(f"  {100*frac:.1f} % of those points pass the floor gate "
                  f"(|height| < {100*h_tol:.0f} cm)")
        else:
            print("  no depth binned into that cell")
        if roi:
            print(f"  inside the WALKABLE polygon (roi): "
                  f"{point_in_poly(qx, qy, roi)}")
        if raw:
            print(f"  inside the RAW floor polygon:      "
                  f"{point_in_poly(qx, qy, raw)}")
        if blocked:
            ins = [r for r in blocked
                   if r[0] <= qx <= r[1] and r[2] <= qy <= r[3]]
            print(f"  covered by an obstacle footprint:  {bool(ins)}"
                  + (f" {ins}" if ins else ""))

    print(f"\nfootprints currently published: {len(blocked)}")
    for r in blocked:
        print(f"   x {r[0]:6.2f}..{r[1]:6.2f}  y {r[2]:6.2f}..{r[3]:6.2f}")
    print(f"detections in the last message: {len(dets)}")
    for d in dets[:12]:
        print(f"   class {d.get('class_id')} score {d.get('score', 0):.2f} "
              f"at cx={d.get('cx', 0):.2f} cy={d.get('cy', 0):.2f} "
              f"w={d.get('w', 0):.2f} h={d.get('h', 0):.2f}")


if __name__ == "__main__":
    main()
