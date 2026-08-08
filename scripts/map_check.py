#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Read FastMapping's accumulated grid off the bus and say what is in it.

The acceptance question for the third brick is not a rate or a latency, it is
whether a map built over minutes contains the room: the barrier the navigator
avoids, the walls, and a free area consistent with the floor we detect. This
answers those three, against our own outputs on the same bus rather than
against a memory of the room.

    make map-check                    # 20 s, then a report
    make map-check ARGS="--seconds 60"
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

# The arena the navigator plans in, the same bounds suite_compare.py scores on.
# Outside it a cell is not wrong, it is simply not what either side is for.
ARENA = (1.5, 6.5, -2.6, 1.5)


def cell_centres(m):
    """(X, Y) world coordinates of every cell centre, as two 2-D arrays."""
    xs = m["x0"] + np.arange(m["w"]) * m["res"]
    ys = m["y0"] + np.arange(m["h"]) * m["res"]
    return np.meshgrid(xs, ys)


def unpack(payload):
    w, h = int(payload["w"]), int(payload["h"])
    grid = np.frombuffer(payload["grid"], dtype=np.int8).reshape(h, w)
    return {"grid": grid, "w": w, "h": h, "res": float(payload["res"]),
            "x0": float(payload["x0"]), "y0": float(payload["y0"])}


def poly_mask(poly, X, Y):
    """Point-in-polygon for a whole grid, by ray casting. No OpenCV needed."""
    inside = np.zeros(X.shape, bool)
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        # Edges that straddle the horizontal line through each cell.
        straddles = ((y0 > Y) != (y1 > Y))
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x1 - x0) * (Y - y0) / np.where(y1 == y0, np.nan, y1 - y0) + x0
        inside ^= straddles & (X < xint)
    return inside


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    sub = Subscriber([topics.SUITE_MAP, topics.PATROL_ROI,
                      topics.SUITE_CLUSTERS])
    m = None
    our_raw, our_blocked, their_clusters = None, None, None
    deadline = time.time() + args.seconds
    print(f"  ecoute pendant {args.seconds:.0f} s ...")
    while time.time() < deadline:
        msg = sub.recv(200)
        if msg is None:
            continue
        topic, payload = msg
        if topic == topics.SUITE_MAP:
            m = unpack(payload)
        elif topic == topics.PATROL_ROI:
            our_raw = [tuple(map(float, v)) for v in (payload.get("raw") or [])]
            our_blocked = [tuple(map(float, b))
                           for b in (payload.get("blocked") or [])]
        elif topic == topics.SUITE_CLUSTERS:
            their_clusters = [tuple(map(float, b))
                              for b in (payload.get("blocked") or [])]

    if m is None:
        print("  rien recu sur suite.map. `make fastmapping` tourne-t-il ?")
        return 1

    grid, res = m["grid"], m["res"]
    X, Y = cell_centres(m)
    unknown, free, occ = grid < 0, grid == 0, grid >= 50
    cell_area = res * res

    print(f"\n  === LA GRILLE ({m['w']}x{m['h']} a {res:.3f} m) ===\n")
    print(f"    etendue      : x {m['x0']:.2f} a {m['x0'] + m['w'] * res:.2f} m,"
          f" y {m['y0']:.2f} a {m['y0'] + m['h'] * res:.2f} m")
    total = grid.size
    print(f"    inconnu      : {int(unknown.sum()):7d} "
          f"({100.0 * unknown.mean():5.1f} %)")
    print(f"    libre        : {int(free.sum()):7d} "
          f"({100.0 * free.mean():5.1f} %, {free.sum() * cell_area:6.2f} m2)")
    print(f"    occupe       : {int(occ.sum()):7d} "
          f"({100.0 * occ.mean():5.1f} %, {occ.sum() * cell_area:6.2f} m2)")

    ax0, ax1, ay0, ay1 = ARENA
    in_arena = (X >= ax0) & (X <= ax1) & (Y >= ay0) & (Y <= ay1)
    n_arena = max(1, int(in_arena.sum()))
    print(f"\n    dans l'arene x {ax0}-{ax1}, y {ay0} a {ay1} "
          f"({n_arena} cellules)")
    for name, sel in (("inconnu", unknown), ("libre", free), ("occupe", occ)):
        k = int((sel & in_arena).sum())
        print(f"      {name:10s} : {k:6d} ({100.0 * k / n_arena:5.1f} %, "
              f"{k * cell_area:6.2f} m2)")

    # --- Les murs. Un mur est une ligne d'occupation au bord de la piece, pas
    # un amas quelque part : on regarde donc l'occupation PAR BANDE.
    print("\n  === LES MURS ===\n")
    print("    occupation par bande de x (toute la largeur y de l'arene)")
    band = (Y >= ay0) & (Y <= ay1)
    for lo in np.arange(1.0, 7.5, 0.5):
        sel = band & (X >= lo) & (X < lo + 0.5)
        n = max(1, int(sel.sum()))
        k = int((occ & sel).sum())
        bar = "#" * int(40 * k / n)
        print(f"      x {lo:4.1f}-{lo + 0.5:4.1f} : {k:5d}/{n:5d} "
              f"{100.0 * k / n:5.1f}% {bar}")

    # Le mur du fond est a 6,2 m. Une bande nettement plus occupee que ses
    # voisines la-bas est le mur ; une occupation plate partout ne l'est pas.
    far = (band & (X >= 6.0) & (X < 6.6))
    mid = (band & (X >= 3.0) & (X < 5.0))
    r_far = (occ & far).sum() / max(1, far.sum())
    r_mid = (occ & mid).sum() / max(1, mid.sum())
    print(f"\n    mur du fond (x 6.0-6.6) : {100 * r_far:.1f} % occupe")
    print(f"    milieu de piece (3-5 m) : {100 * r_mid:.1f} % occupe")
    print(f"    rapport                 : {r_far / max(1e-9, r_mid):.2f}x "
          + ("-> le mur ressort" if r_far > 2 * r_mid else
             "-> PAS de mur distinct"))

    print("\n    occupation par bande de y (x 1.5-6.5)")
    bandx = (X >= ax0) & (X <= ax1)
    for lo in np.arange(-3.0, 2.0, 0.5):
        sel = bandx & (Y >= lo) & (Y < lo + 0.5)
        n = max(1, int(sel.sum()))
        k = int((occ & sel).sum())
        bar = "#" * int(40 * k / n)
        print(f"      y {lo:5.1f}-{lo + 0.5:5.1f} : {k:5d}/{n:5d} "
              f"{100.0 * k / n:5.1f}% {bar}")

    # --- La barriere. Nos empreintes sont ce que le navigateur evite : la
    # question est si la carte est occupee la ou elles sont.
    print("\n  === LA BARRIERE (nos empreintes contre la carte) ===\n")
    if not our_blocked:
        print("    rien recu sur patrol.roi")
    else:
        for x0, x1, y0, y1 in sorted(our_blocked, key=lambda b: -(b[1] - b[0])):
            sel = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
            n = max(1, int(sel.sum()))
            k = int((occ & sel).sum())
            u = int((unknown & sel).sum())
            print(f"    empreinte {x1 - x0:4.2f} x {y1 - y0:4.2f} m a "
                  f"({(x0 + x1) / 2:5.2f},{(y0 + y1) / 2:5.2f}) : "
                  f"{100.0 * k / n:5.1f} % occupe, "
                  f"{100.0 * u / n:5.1f} % inconnu")

    if their_clusters:
        print("\n    et leurs propres clusters ADBSCAN, meme question :")
        for x0, x1, y0, y1 in their_clusters:
            sel = (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
            n = max(1, int(sel.sum()))
            k = int((occ & sel).sum())
            print(f"    cluster   {x1 - x0:4.2f} x {y1 - y0:4.2f} m a "
                  f"({(x0 + x1) / 2:5.2f},{(y0 + y1) / 2:5.2f}) : "
                  f"{100.0 * k / n:5.1f} % occupe")

    # --- Le sol. Leur "libre" contre notre polygone brut, dans l'arene.
    print("\n  === LE SOL (leur libre contre notre sol brut) ===\n")
    if not our_raw or len(our_raw) < 3:
        print("    pas de polygone `raw` sur patrol.roi")
    else:
        ours = poly_mask(our_raw, X, Y) & in_arena
        theirs = free & in_arena
        inter = int((ours & theirs).sum())
        union = int((ours | theirs).sum())
        print(f"    notre sol brut : {int(ours.sum()) * cell_area:6.2f} m2")
        print(f"    leur libre     : {int(theirs.sum()) * cell_area:6.2f} m2")
        print(f"    intersection   : {inter * cell_area:6.2f} m2")
        print(f"    IoU            : {inter / max(1, union):.3f}")
        only_ours = int((ours & ~theirs).sum())
        only_th = int((theirs & ~ours).sum())
        unk_in_ours = int((ours & unknown).sum())
        print(f"    nous seulement : {only_ours * cell_area:6.2f} m2 "
              f"(dont {unk_in_ours * cell_area:5.2f} m2 encore inconnus chez eux)")
        print(f"    eux seulement  : {only_th * cell_area:6.2f} m2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
