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


def union_iou(a, boxes) -> float:
    """IoU between one rectangle and the UNION of several others.

    Exact, by coordinate compression rather than rasterisation: the union of
    overlapping rectangles is not a rectangle, and summing their areas would
    double-count the overlaps and depress the score exactly where a detector
    fragments an object into pieces that touch. Counts are single digits per
    frame, so the (2n)^2 cells cost nothing.
    """
    if not boxes:
        return 0.0
    xs = sorted({a[0], a[1], *(v for b in boxes for v in (b[0], b[1]))})
    ys = sorted({a[2], a[3], *(v for b in boxes for v in (b[2], b[3]))})
    inter = union = 0.0
    for i in range(len(xs) - 1):
        x0, x1 = xs[i], xs[i + 1]
        w = x1 - x0
        if w <= 1e-12:
            continue
        cx = 0.5 * (x0 + x1)
        in_a_x = a[0] <= cx <= a[1]
        for j in range(len(ys) - 1):
            y0, y1 = ys[j], ys[j + 1]
            h = y1 - y0
            if h <= 1e-12:
                continue
            cy = 0.5 * (y0 + y1)
            in_a = in_a_x and a[2] <= cy <= a[3]
            in_b = any(b[0] <= cx <= b[1] and b[2] <= cy <= b[3] for b in boxes)
            if in_a and in_b:
                inter += w * h
            if in_a or in_b:
                union += w * h
    return inter / union if union > 1e-9 else 0.0


def group_match(ours, theirs, cover: float = 0.5, threshold: float = 0.3):
    """Many-to-one matching: every box of `theirs` inside one of `ours` joins it.

    Answers a different question from match(). match() asks "did they draw the
    same rectangle we did", which a detector that splits one object into three
    fails by construction even when it is looking at exactly the same thing.
    This asks "did they cover the same ground", which is what the navigator
    would actually consume, since it merges footprints itself.

    Assignment is by coverage of THEIR box rather than by IoU: a fragment one
    tenth the size of our footprint sits entirely inside it and should join it,
    yet scores 0.1 IoU against it and would never pair. `cover` is the fraction
    of their box that has to fall inside ours; each box joins at most one group,
    the one covering it most.

    **This has to be a relaxation of match(), never a different rule**, or the
    comparison between the two blocks says nothing. Two things enforce that, and
    both were found by measurement rather than by reasoning:

    1. The groups are SEEDED with match()'s own pairs, then grown. Assigning
       purely by coverage let a box join the footprint that covered it most
       while match() had paired it with a different one, and the second
       footprint lost its pair -- one trial in 3000 random layouts.
    2. A group whose union scores worse than its best single member falls back
       to that member. A fragment can extend well past our footprint and drag
       the union down.

    A coverage rule on its own also rejects wide overlapping boxes that IoU
    accepts, which put the grouped rate at 6% against a per-cluster 19% on the
    first run. Grouped is now an upper bound on per-cluster by construction, and
    the gap between the two is the part of the disagreement that is granularity.

    MEASURED, and the answer is negative: grouping essentially never fires,
    because their rectangles are larger than ours rather than fragments of
    them. The gap is not how the objects are cut up, it is where they are, and
    no merge rule closes that. Both blocks come out of the same run, so the
    comparison is internal and immune to the cross-session floor drift of
    docs/ETAPE-B-RESULTS.md section 7. Numbers in ETAPE-C-RESULTS.md section 7.

    Returns (pairs, ours_only, theirs_ungrouped) where a pair is
    (our index, tuple of their indices, IoU against their union).
    """
    groups = [[] for _ in ours]
    ungrouped = []
    seeded = {}
    for i, j, _ in match(ours, theirs, threshold)[0]:
        groups[i].append(j)
        seeded[j] = i
    for j, b in enumerate(theirs):
        if j in seeded:
            continue
        area_b = (b[1] - b[0]) * (b[3] - b[2])
        best, best_cov = None, cover
        for i, a in enumerate(ours):
            ix = max(0.0, min(a[1], b[1]) - max(a[0], b[0]))
            iy = max(0.0, min(a[3], b[3]) - max(a[2], b[2]))
            c = (ix * iy) / area_b if area_b > 1e-9 else 0.0
            if c >= best_cov:
                best, best_cov = i, c
        if best is None:
            ungrouped.append(j)
        else:
            groups[best].append(j)

    pairs = []
    for i, js in enumerate(groups):
        if not js:
            continue
        boxes = [theirs[j] for j in js]
        score = union_iou(ours[i], boxes)
        keep = tuple(js)
        if len(js) > 1:
            singles = [(overlap(ours[i], b), j) for b, j in zip(boxes, js)]
            best_single, best_j = max(singles)
            if best_single > score:
                score, keep = best_single, (best_j,)
        if score > threshold:
            pairs.append((i, keep, score))
            ungrouped.extend(j for j in js if j not in keep)
        else:
            # Grouped and still not agreeing. Their pieces go back to unmatched
            # rather than counting as a hit: the whole point is to find out
            # whether grouping alone closes the gap.
            ungrouped.extend(js)
    ours_only = [i for i in range(len(ours)) if i not in {p[0] for p in pairs}]
    return pairs, ours_only, ungrouped


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


# The floor the robot actually patrols, and the only region where a footprint
# from either side means anything. Beyond the far wall at 6.5 m and outside the
# lateral extent of the detected floor, a rectangle is a depth artefact whatever
# produced it.
ARENA = (1.5, 6.5, -2.6, 1.5)


def clip_to_arena(boxes, arena=ARENA):
    """Truncate rectangles to the arena. Returns (clipped, n_outside).

    Both pipelines emit a room-spanning blob, and IoU cannot pair one of those
    with anything: our merged barrier sat ENTIRELY inside their 49 m2 rectangle
    and still scored 0.21, under the 0.3 threshold, purely because the union was
    the blob. Clipping both sides to the same region removes the part of the
    disagreement that is about how far each one runs off into unmeasured space,
    and leaves the part that is about where the obstacles are.

    Counted, not silently dropped: a side that puts most of its rectangles
    outside the arena is telling you something, and it must not disappear into
    a better-looking IoU.
    """
    ax0, ax1, ay0, ay1 = arena
    out, outside = [], 0
    for x0, x1, y0, y1 in boxes:
        cx0, cx1 = max(x0, ax0), min(x1, ax1)
        cy0, cy1 = max(y0, ay0), min(y1, ay1)
        if cx1 - cx0 <= 1e-6 or cy1 - cy0 <= 1e-6:
            outside += 1
            continue
        out.append((cx0, cx1, cy0, cy1))
    return out, outside


def spot_map(records, frames: int, radius: float = 0.5):
    """Group unmatched rectangles by where they are. Returns a list of spots.

    The per-frame counts say how MANY rectangles neither side could pair. They
    cannot say whether that is one stable object each side keeps missing or a
    different piece of noise every frame, and those two call for opposite
    responses. This separates them: a spot recurring in 90% of frames is an
    object, a spot at 5% is a flicker.

    Leader clustering on the centre rather than k-means or single-link. k-means
    needs a k we do not know, and single-link chains along a wall -- exactly the
    failure ADBSCAN itself has here, which would be an unfortunate way to
    measure it. `radius` is 0.5 m, under the smallest gap between the real
    objects in this room and over the frame-to-frame jitter of either detector.

    Persistence counts DISTINCT frames, not rectangles: two fragments of one
    object in one frame must not read as twice the persistence.
    """
    spots = []
    for frame, b in records:
        cx, cy = 0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3])
        best, best_d = None, radius
        for s in spots:
            d = ((cx - s["cx"]) ** 2 + (cy - s["cy"]) ** 2) ** 0.5
            if d < best_d:
                best, best_d = s, d
        if best is None:
            best = {"cx": cx, "cy": cy, "n": 0, "frames": set(),
                    "w": [], "h": [], "x0": [], "x1": [], "y0": [], "y1": []}
            spots.append(best)
        best["n"] += 1
        best["frames"].add(frame)
        best["cx"] += (cx - best["cx"]) / best["n"]
        best["cy"] += (cy - best["cy"]) / best["n"]
        best["w"].append(b[1] - b[0])
        best["h"].append(b[3] - b[2])
        for k, v in zip(("x0", "x1", "y0", "y1"), b):
            best[k].append(v)
    md = lambda xs: sorted(xs)[len(xs) // 2]
    for s in spots:
        s["persistence"] = len(s["frames"]) / frames if frames else 0.0
        s["med_w"], s["med_h"] = md(s["w"]), md(s["h"])
        s["span"] = (md(s["x0"]), md(s["x1"]), md(s["y0"]), md(s["y1"]))
    spots.sort(key=lambda s: -s["persistence"])
    return spots


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--unmatched", action="store_true",
                    help="carte des rectangles non apparies, par emplacement")
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI, topics.GROUNDFLOOR_OBSTACLES,
                      topics.GROUNDFLOOR_FLOOR, topics.SUITE_CLUSTERS])
    ours, theirs = None, None
    our_floor, their_floor, our_raw = None, None, None
    samples, ious, counts = [], [], []
    floors, raws = [], []
    clusters = None
    clu_samples, clu_ious, clu_counts, clu_raw = [], [], [], []
    clu_outside = []
    # Groupe : plusieurs-vers-un, dans les deux sens. `t_in_o` groupe leurs
    # clusters dans nos empreintes, `o_in_t` fait l'inverse.
    grp_t_in_o, grp_o_in_t = [], []
    # (numero de trame, rectangle) pour chaque rectangle non apparie, les deux
    # cotes. Le numero de trame est l'index dans clu_samples : c'est le meme
    # denominateur que le taux d'appariement rapporte plus haut.
    un_ours, un_theirs = [], []
    theirs_seen, floor_seen, clu_seen = 0, 0, 0
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
        elif topic == topics.SUITE_CLUSTERS:
            clusters = [tuple(map(float, b)) for b in (payload.get("blocked") or [])]
            clu_raw.append(int(payload.get("clusters", len(clusters))))
            clu_seen += 1
        if our_floor and their_floor:
            fc = floor_compare(our_floor, their_floor)
            if fc is not None:
                floors.append(fc)
        if our_raw and their_floor:
            rc = floor_compare(our_raw, their_floor)
            if rc is not None:
                raws.append(rc)
        # Clusters against our footprints. Same match() and same 0.3 threshold
        # as the groundfloor footprints, so the two sections are readable
        # against each other rather than against two different rulers.
        if ours is not None and clusters is not None:
            a_ours, out_ours = clip_to_arena(ours)
            a_theirs, out_theirs = clip_to_arena(clusters)
            cp, c_only_a, c_only_b = match(a_ours, a_theirs)
            frame = len(clu_samples)
            un_ours.extend((frame, a_ours[i]) for i in c_only_a)
            un_theirs.extend((frame, a_theirs[j]) for j in c_only_b)
            clu_samples.append((len(a_ours), len(a_theirs), len(cp)))
            clu_counts.append((len(c_only_a), len(c_only_b)))
            clu_outside.append((out_ours, out_theirs))
            clu_ious.extend(p[2] for p in cp)
            # Meme trame, meme rognage, meme seuil de 0.3 : la seule difference
            # est le groupage, donc l'ecart entre les deux blocs est exactement
            # la part du desaccord qui est de la granularite.
            gp, g_only, g_loose = group_match(a_ours, a_theirs)
            grp_t_in_o.append((len(a_ours), len(gp), len(g_only), len(g_loose),
                               [p[2] for p in gp], [len(p[1]) for p in gp]))
            rp, r_only, r_loose = group_match(a_theirs, a_ours)
            grp_o_in_t.append((len(a_theirs), len(rp), len(r_only), len(r_loose),
                               [p[2] for p in rp], [len(p[1]) for p in rp]))
        if ours is None or theirs is None:
            continue
        pairs, only_a, only_b = match(ours, theirs)
        samples.append((len(ours), len(theirs), len(pairs)))
        counts.append((len(only_a), len(only_b)))
        ious.extend(p[2] for p in pairs)

    # Les deux briques sont independantes et se lancent separement, donc une
    # seule qui parle suffit a produire un rapport. Ne rien recevoir des DEUX
    # est la seule condition d'echec.
    if theirs_seen == 0 and floor_seen == 0 and clu_seen == 0:
        print("  rien recu de la suite. Un des deux services tourne-t-il ?"
              "  make groundfloor / make adbscan")
        return 1
    if not samples and not floors and not clu_samples:
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
        print("    pas de comparaison : "
              + ("groundfloor ne tourne pas" if not their_floor else
                 "le compositor ne publie pas `raw` (image trop ancienne ?)"))
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

    # ADBSCAN. Contrairement au sol contre empreintes de l'etape B, les deux
    # cotes repondent ici a la meme question : "quelque chose est la, gros comme
    # ca". C'est la comparaison d'empreintes que l'etape B aurait du faire.
    ax0, ax1, ay0, ay1 = ARENA
    print(f"\n  === CLUSTERS ADBSCAN CONTRE NOS EMPREINTES "
          f"({len(clu_samples)} comparaisons, {clu_seen} messages) ===")
    print(f"      arene x {ax0}-{ax1} m, y {ay0} a {ay1} m ; les deux jeux y "
          f"sont rognes avant appariement\n")
    if not clu_samples:
        print("    rien recu d'adbscan. Le service tourne-t-il ?  make adbscan")
    else:
        cavg = lambda xs: sum(xs) / len(xs) if xs else 0.0
        print("    DANS L'ARENE")
        print(f"      empreintes, ce projet : "
              f"{cavg([s[0] for s in clu_samples]):.1f}")
        print(f"      clusters, adbscan     : "
              f"{cavg([s[1] for s in clu_samples]):.1f} "
              f"(avant filtrage du pont : {cavg(clu_raw):.1f})")
        print(f"      apparies              : "
              f"{cavg([s[2] for s in clu_samples]):.1f}")
        print(f"      vus par nous seulement: "
              f"{cavg([c[0] for c in clu_counts]):.1f}")
        print(f"      vus par eux seulement : "
              f"{cavg([c[1] for c in clu_counts]):.1f}")
        if clu_ious:
            ci = sorted(clu_ious)
            print(f"      recouvrement des paires : mediane "
                  f"{ci[len(ci) // 2]:.2f}, min {ci[0]:.2f}, max {ci[-1]:.2f}")
            denom = max(1e-9, cavg([s[0] for s in clu_samples]))
            print(f"      taux d'appariement      : "
                  f"{cavg([s[2] for s in clu_samples]) / denom:.0%} de nos "
                  f"empreintes ont une contrepartie")
        else:
            print("      aucune paire au-dessus du seuil de 0.3 d'IoU")
        # Plusieurs-vers-un. La question : le desaccord qui reste est-il un
        # desaccord sur OU sont les obstacles, ou seulement sur leur decoupage ?
        # Si le recouvrement groupe monte nettement au-dessus du recouvrement
        # par cluster, c'est de la granularite, et le navigateur -- qui fusionne
        # deja les empreintes -- ne la verrait pas.
        def _grp(rows, label, unit_a, unit_b):
            n_a = cavg([r[0] for r in rows])
            n_pair = cavg([r[1] for r in rows])
            scores = sorted(s for r in rows for s in r[4])
            sizes = [s for r in rows for s in r[5]]
            print(f"\n      {label}")
            print(f"        {unit_a} apparies    : {n_pair:.1f} sur "
                  f"{n_a:.1f}  ({n_pair / max(1e-9, n_a):.0%})")
            if scores:
                print(f"        recouvrement groupe : mediane "
                      f"{scores[len(scores) // 2]:.2f}, min {scores[0]:.2f}, "
                      f"max {scores[-1]:.2f}")
                print(f"        {unit_b} par groupe : "
                      f"{sum(sizes) / len(sizes):.2f} "
                      f"(max {max(sizes)})")
            else:
                print("        aucun groupe au-dessus du seuil de 0.3")
            print(f"        sans groupe         : {cavg([r[2] for r in rows]):.1f} "
                  f"/ {cavg([r[3] for r in rows]):.1f} de l'autre cote")

        if grp_t_in_o:
            print("\n    APPARIEMENT GROUPE (plusieurs-vers-un, seuil 0.3, "
                  "rattachement a 50% de couverture)")
            _grp(grp_t_in_o, "leurs clusters groupes dans nos empreintes",
                 "nos empreintes", "clusters")
            _grp(grp_o_in_t, "nos empreintes groupees dans leurs clusters",
                 "leurs clusters", "empreintes")

        # Nommer le desaccord. Les compteurs disent combien de rectangles
        # personne n'apparie ; ils ne disent pas si c'est toujours le meme
        # objet ou un bruit different a chaque trame, et les deux appellent des
        # reponses opposees.
        #
        # Ce que cette carte a donne sur cette piece, avant et apres le filtre
        # de residu de sol, avec la lecture physique de chaque emplacement :
        # docs/ETAPE-C-RESULTS.md section 7. Les chiffres vivent la-bas et pas
        # ici, parce qu'ils decrivent une piece et une session, pas ce code.
        if args.unmatched:
            def _spots(records, label):
                spots = spot_map(records, len(clu_samples))
                print(f"\n      {label}")
                if not spots:
                    print("        aucun")
                    return
                print("        persist.  centre (x, y)   taille       "
                      "etendue x / y")
                for s in spots:
                    if s["persistence"] < 0.05:
                        continue
                    x0, x1, y0, y1 = s["span"]
                    print(f"        {s['persistence']:6.0%}   "
                          f"{s['cx']:5.2f}, {s['cy']:5.2f}   "
                          f"{s['med_w']:4.2f} x {s['med_h']:4.2f}   "
                          f"{x0:4.2f}-{x1:4.2f} / {y0:5.2f}-{y1:5.2f}")
                weak = sum(1 for s in spots if s["persistence"] < 0.05)
                if weak:
                    print(f"        (+ {weak} emplacements sous 5%, du "
                          f"scintillement)")

            print("\n    CARTE DES NON APPARIES (regroupes a 0.5 m, "
                  f"{len(clu_samples)} trames)")
            _spots(un_ours, "vus par nous seulement")
            _spots(un_theirs, "vus par eux seulement")

        print("\n    HORS ARENE (rectangles entierement dehors, ecartes)")
        print(f"      ce projet : {cavg([c[0] for c in clu_outside]):.1f} "
              f"par trame")
        print(f"      adbscan   : {cavg([c[1] for c in clu_outside]):.1f} "
              f"par trame")

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
