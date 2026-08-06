#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Compare this project's obstacle footprints with Intel's segmentation.

Step B asks one question: does the suite's brick see the same floor we do, and
what does the round trip cost. This listens to both sides of the running demo
and answers it, rather than either being replaced by the other on faith.

    make suite-compare              30 seconds, then a report
    make suite-compare ARGS="--seconds 120"

Both services must be running:

    make                            in one terminal
    make groundfloor                in another
"""
from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "common")
sys.path.insert(0, "/opt/edgebot")

from edgebot import topics
from edgebot.bus import Subscriber


def overlap(a, b) -> float:
    """Intersection over union of two ground rectangles."""
    ix = max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[2], b[2]))
    inter = ix * iy
    union = ((a[1] - a[0]) * (a[3] - a[2])
             + (b[1] - b[0]) * (b[3] - b[2]) - inter)
    return inter / union if union > 1e-9 else 0.0


def match(ours, theirs, threshold: float = 0.3):
    """Pair footprints by overlap. Returns (pairs, ours_only, theirs_only)."""
    pairs, used = [], set()
    for i, a in enumerate(ours):
        best, score = None, threshold
        for j, b in enumerate(theirs):
            if j in used:
                continue
            o = overlap(a, b)
            if o > score:
                best, score = j, o
        if best is not None:
            used.add(best)
            pairs.append((i, best, score))
    ours_only = [i for i in range(len(ours))
                 if i not in {p[0] for p in pairs}]
    theirs_only = [j for j in range(len(theirs)) if j not in used]
    return pairs, ours_only, theirs_only


def floor_compare(poly_a, poly_b, cell: float = 0.05, pad: int = 4):
    """IoU and boundary distance between two ground polygons.

    Rasterised onto one grid rather than compared analytically, for the reason
    shrink() gives: polygon geometry is unstable at reflex corners, and both of
    these outlines have them. At 5 cm the grid is well below the accuracy either
    floor detection actually has.

    Boundary distance is symmetric and reported as a distribution, not a mean.
    A mean hides the failure that matters here: two floors agreeing over most of
    their outline while one of them runs metres past the other at the far wall
    would still average small.

    Returns None when either polygon is degenerate, else a dict.
    """
    import cv2
    import numpy as np

    if not poly_a or not poly_b or len(poly_a) < 3 or len(poly_b) < 3:
        return None
    a = np.asarray(poly_a, np.float64)
    b = np.asarray(poly_b, np.float64)
    lo = np.minimum(a.min(axis=0), b.min(axis=0))
    hi = np.maximum(a.max(axis=0), b.max(axis=0))
    nx = int((hi[0] - lo[0]) / cell) + 2 * pad + 2
    ny = int((hi[1] - lo[1]) / cell) + 2 * pad + 2
    if nx * ny > 16_000_000 or nx < 3 or ny < 3:
        return None

    def raster(poly):
        img = np.zeros((ny, nx), np.uint8)
        ij = np.empty((len(poly), 2), np.int32)
        ij[:, 0] = ((poly[:, 0] - lo[0]) / cell).astype(np.int32) + pad
        ij[:, 1] = ((poly[:, 1] - lo[1]) / cell).astype(np.int32) + pad
        cv2.fillPoly(img, [ij], 1)
        return img

    ra, rb = raster(a), raster(b)
    inter = int(np.count_nonzero(ra & rb))
    union = int(np.count_nonzero(ra | rb))
    if union == 0:
        return None

    def edge(img):
        er = cv2.erode(img, np.ones((3, 3), np.uint8), iterations=1)
        return (img - er).astype(np.uint8)

    ea, eb = edge(ra), edge(rb)
    if not ea.any() or not eb.any():
        return None
    # distanceTransform measures distance to the nearest ZERO pixel, so invert
    # the boundary to get "distance to this outline" everywhere.
    da = cv2.distanceTransform((1 - ea).astype(np.uint8), cv2.DIST_L2, 3) * cell
    db = cv2.distanceTransform((1 - eb).astype(np.uint8), cv2.DIST_L2, 3) * cell
    d_ba = da[eb > 0]          # their boundary -> our boundary
    d_ab = db[ea > 0]          # our boundary -> their boundary
    both = np.concatenate([d_ab, d_ba])
    return {
        "iou": inter / union,
        "area_a": float(np.count_nonzero(ra)) * cell * cell,
        "area_b": float(np.count_nonzero(rb)) * cell * cell,
        "bd_median": float(np.median(both)),
        "bd_p95": float(np.percentile(both, 95)),
        "bd_max": float(both.max()),
        "bd_ours_to_theirs": float(np.median(d_ab)),
        "bd_theirs_to_ours": float(np.median(d_ba)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI, topics.GROUNDFLOOR_OBSTACLES,
                      topics.GROUNDFLOOR_FLOOR])
    ours, theirs = None, None
    our_floor, their_floor, our_raw = None, None, None
    samples, ious, counts = [], [], []
    floors, raws = [], []
    theirs_seen, floor_seen = 0, 0
    deadline = time.time() + args.seconds
    print(f"  ecoute pendant {args.seconds:.0f} s ...")

    while time.time() < deadline:
        msg = sub.recv(200)
        if msg is None:
            continue
        topic, payload = msg
        if topic == topics.PATROL_ROI:
            ours = [tuple(map(float, b)) for b in (payload.get("blocked") or [])]
            our_floor = [tuple(map(float, v)) for v in (payload.get("roi") or [])]
            our_raw = [tuple(map(float, v)) for v in (payload.get("raw") or [])]
        elif topic == topics.GROUNDFLOOR_OBSTACLES:
            theirs = [tuple(map(float, b)) for b in (payload.get("blocked") or [])]
            theirs_seen += 1
        elif topic == topics.GROUNDFLOOR_FLOOR:
            their_floor = [tuple(map(float, v)) for v in (payload.get("poly") or [])]
            floor_seen += 1
        if our_floor and their_floor:
            fc = floor_compare(our_floor, their_floor)
            if fc is not None:
                floors.append(fc)
        if our_raw and their_floor:
            rc = floor_compare(our_raw, their_floor)
            if rc is not None:
                raws.append(rc)
        if ours is None or theirs is None:
            continue
        pairs, only_a, only_b = match(ours, theirs)
        samples.append((len(ours), len(theirs), len(pairs)))
        counts.append((len(only_a), len(only_b)))
        ious.extend(p[2] for p in pairs)

    if theirs_seen == 0 and floor_seen == 0:
        print("  rien recu de la segmentation. Le service groundfloor "
              "tourne-t-il ?  make groundfloor")
        return 1
    if not samples and not floors:
        print("  rien recu du compositor. La demo tourne-t-elle ?  make")
        return 1

    avg = lambda xs: sum(xs) / len(xs) if xs else 0.0

    # --- Sol contre sol. C'est le critere de l'etape B: la question est de
    # savoir si leur segmentation voit le meme sol que la notre. Les empreintes
    # en dessous sont un produit derive, gardees pour information.
    print(f"\n  === SOL CONTRE SOL ({len(floors)} comparaisons, "
          f"{floor_seen} messages de sol) ===\n")
    if not floors:
        print("    aucune comparaison de sol possible : "
              f"{'notre roi est vide' if not our_floor else ''}"
              f"{' et ' if not our_floor and not their_floor else ''}"
              f"{'leur polygone est vide' if not their_floor else ''}"
              f"{'polygone degenere' if our_floor and their_floor else ''}")
    else:
        med = lambda k: sorted(f[k] for f in floors)[len(floors) // 2]
        print(f"    IoU du sol              : mediane {med('iou'):.3f}, "
              f"min {min(f['iou'] for f in floors):.3f}, "
              f"max {max(f['iou'] for f in floors):.3f}")
        print(f"    surface, ce projet      : {med('area_a'):.2f} m2")
        print(f"    surface, la suite       : {med('area_b'):.2f} m2")
        print(f"    distance des frontieres : mediane {med('bd_median'):.3f} m, "
              f"p95 {med('bd_p95'):.3f} m, max {med('bd_max'):.3f} m")
        print(f"      dont nous -> eux      : {med('bd_ours_to_theirs'):.3f} m")
        print(f"      dont eux -> nous      : {med('bd_theirs_to_ours'):.3f} m")

    # Definitions neutralisees. `roi` est une POLITIQUE (ou le robot a le droit
    # de marcher) : la marge et la soustraction des silhouettes y sont deja
    # appliquees, donc la comparer a leur segmentation mesure autant nos choix
    # que leur detecteur. `raw` est la PERCEPTION seule.
    print(f"\n  === SOL BRUT CONTRE SOL, DEFINITIONS NEUTRALISEES "
          f"({len(raws)} comparaisons) ===\n")
    if not raws:
        print("    aucun polygone brut recu (compositor trop ancien ?)")
    else:
        rmed = lambda k: sorted(f[k] for f in raws)[len(raws) // 2]
        print(f"    IoU du sol brut         : mediane {rmed('iou'):.3f}, "
              f"min {min(f['iou'] for f in raws):.3f}, "
              f"max {max(f['iou'] for f in raws):.3f}")
        print(f"    surface, brut           : {rmed('area_a'):.2f} m2")
        print(f"    surface, la suite       : {rmed('area_b'):.2f} m2")
        print(f"    distance des frontieres : mediane {rmed('bd_median'):.3f} m, "
              f"p95 {rmed('bd_p95'):.3f} m, max {rmed('bd_max'):.3f} m")
        print(f"      dont nous -> eux      : {rmed('bd_ours_to_theirs'):.3f} m")
        print(f"      dont eux -> nous      : {rmed('bd_theirs_to_ours'):.3f} m")

    if not samples:
        print("\n  (pas d'empreintes recues, comparaison d'obstacles ignoree)")
        return 0

    n = len(samples)
    print(f"\n  === EMPREINTES (produit derive) ===")
    print(f"\n  {n} comparaisons, {theirs_seen} messages de segmentation\n")
    print(f"    empreintes, ce projet   : {avg([s[0] for s in samples]):.1f}")
    print(f"    empreintes, la suite    : {avg([s[1] for s in samples]):.1f}")
    print(f"    appariees               : {avg([s[2] for s in samples]):.1f}")
    print(f"    vues par nous seulement : {avg([c[0] for c in counts]):.1f}")
    print(f"    vues par eux seulement  : {avg([c[1] for c in counts]):.1f}")
    if ious:
        ious.sort()
        print(f"\n    recouvrement des paires : mediane "
              f"{ious[len(ious)//2]:.2f}, min {ious[0]:.2f}, max {ious[-1]:.2f}")
    print("\n  Le critere de l'etape B est l'IoU du sol, pas les empreintes.")
    print("  Un IoU au-dessus de 0.5 avec une frontiere a moins de 0.20 m")
    print("  signifie que les deux detections voient le meme sol. Un ecart")
    print("  systematique en profondeur pointe la calibration, un ecart en")
    print("  largeur pointe la marge ou le seuil de hauteur. Les empreintes")
    print("  ci-dessus sont un produit derive et leur IoU est dilue des que")
    print("  l'un des deux cotes fusionne : lire le sol d'abord.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
