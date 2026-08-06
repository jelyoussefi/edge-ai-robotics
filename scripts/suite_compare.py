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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    sub = Subscriber([topics.PATROL_ROI, topics.GROUNDFLOOR_OBSTACLES])
    ours, theirs = None, None
    samples, ious, counts = [], [], []
    theirs_seen = 0
    deadline = time.time() + args.seconds
    print(f"  ecoute pendant {args.seconds:.0f} s ...")

    while time.time() < deadline:
        msg = sub.recv(200)
        if msg is None:
            continue
        topic, payload = msg
        if topic == topics.PATROL_ROI:
            ours = [tuple(map(float, b)) for b in (payload.get("blocked") or [])]
        elif topic == topics.GROUNDFLOOR_OBSTACLES:
            theirs = [tuple(map(float, b)) for b in (payload.get("blocked") or [])]
            theirs_seen += 1
        if ours is None or theirs is None:
            continue
        pairs, only_a, only_b = match(ours, theirs)
        samples.append((len(ours), len(theirs), len(pairs)))
        counts.append((len(only_a), len(only_b)))
        ious.extend(p[2] for p in pairs)

    if theirs_seen == 0:
        print("  rien recu de la segmentation. Le service groundfloor "
              "tourne-t-il ?  make groundfloor")
        return 1
    if not samples:
        print("  rien recu du compositor. La demo tourne-t-elle ?  make")
        return 1

    n = len(samples)
    avg = lambda xs: sum(xs) / len(xs) if xs else 0.0
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
    print("\n  Un recouvrement median au-dessus de 0.5 et peu d'empreintes")
    print("  vues d'un seul cote signifient que les deux voient la meme piece.")
    print("  Un ecart systematique en profondeur pointe la calibration, un")
    print("  ecart en largeur pointe la marge ou le seuil de hauteur.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
