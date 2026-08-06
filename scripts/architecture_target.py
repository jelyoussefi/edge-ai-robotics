#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Draw the TARGET architecture: maximum reuse of the Robotics AI Suite.

Distinct from architecture_diagram.py, which shows what runs today. This one
shows where the project is heading once the suite's bricks replace the parts
written by hand, and is the picture to discuss with the suite team.

    python3 scripts/architecture_target.py [output.png]
"""
from __future__ import annotations

import sys

import cairosvg

W, H = 1800, 1120

INK = "#12233A"
MUTED = "#5A6B7D"
LINE = "#B9C6D4"
BG = "#FFFFFF"
PANEL = "#F4F7FA"

THEIRS = "#0068B5"      # Robotics AI Suite
MINE = "#00875A"        # this project
CLOUD = "#C2410C"       # the point cloud artery
HW = "#7A8899"

F = "DejaVu Sans"


def box(x, y, w, h, title, lines, accent, tag=None, cloud=False):
    """One component. `cloud` marks it as a consumer of the point cloud."""
    out = [
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" fill="white" '
        f'stroke="{CLOUD if cloud else LINE}" '
        f'stroke-width="{2.5 if cloud else 1.5}"/>',
        f'<rect x="{x}" y="{y}" width="5" height="{h}" rx="2.5" fill="{accent}"/>',
        f'<text x="{x + 18}" y="{y + 28}" font-family="{F}" font-size="18" '
        f'font-weight="bold" fill="{INK}">{title}</text>',
    ]
    for i, line in enumerate(lines):
        out.append(f'<text x="{x + 18}" y="{y + 52 + i * 19}" font-family="{F}" '
                   f'font-size="13.5" fill="{MUTED}">{line}</text>')
    if tag:
        tw = 7 * len(tag) + 18
        out.append(f'<rect x="{x + w - tw - 12}" y="{y + 12}" width="{tw}" '
                   f'height="21" rx="10.5" fill="{accent}" opacity="0.14"/>')
        out.append(f'<text x="{x + w - tw / 2 - 12}" y="{y + 27}" '
                   f'font-family="{F}" font-size="12" font-weight="bold" '
                   f'fill="{accent}" text-anchor="middle">{tag}</text>')
    return "\n".join(out)


def arrow(x1, y1, x2, y2, colour=THEIRS, width=2, dash=False, marker="a"):
    d = ' stroke-dasharray="6 5"' if dash else ""
    return (f'<path d="M {x1} {y1} L {x2} {y2}" stroke="{colour}" '
            f'stroke-width="{width}"{d} fill="none" marker-end="url(#{marker})"/>')


def label(x, y, text, colour=MUTED, size=13, anchor="middle", bold=False):
    w = ' font-weight="bold"' if bold else ""
    return (f'<text x="{x}" y="{y}" font-family="{F}" font-size="{size}" '
            f'fill="{colour}" text-anchor="{anchor}"{w}>{text}</text>')


def build() -> str:
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}">',
         '<defs>',
         f'<marker id="a" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
         f'markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{THEIRS}"/></marker>',
         f'<marker id="c" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" '
         f'markerHeight="6" orient="auto-start-reverse">'
         f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{CLOUD}"/></marker>',
         f'<marker id="g" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
         f'markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{MINE}"/></marker>',
         '</defs>',
         f'<rect width="{W}" height="{H}" fill="{BG}"/>']

    s.append(label(60, 58, "Edge AI Robotics, architecture cible", INK, 31,
                   "start", True))
    s.append(label(60, 88, "Maximum de briques Robotics AI Suite, avec un "
                   "chemin de bascule du robot simule vers un G1 reel",
                   MUTED, 16, "start"))
    s.append(f'<line x1="60" y1="106" x2="{W - 60}" y2="106" stroke="{LINE}" '
             f'stroke-width="1.5"/>')

    # Rows shared by the two panels, so the flows stay horizontal.
    R = [228, 368, 508, 648]
    PT, PB = 186, 780

    # ---- sensor column ---------------------------------------------------
    s.append(box(60, R[0], 250, 92, "RealSense D455",
                 ["couleur + profondeur alignee", "640x480 a 30 fps"], HW))
    s.append(box(60, R[1], 250, 106, "realsense_ros2",
                 ["publie le nuage de points", "et les images"],
                 THEIRS, "SUITE", cloud=True))
    s.append(arrow(185, R[0] + 92, 185, R[1] - 2))

    # ---- perception panel ------------------------------------------------
    s.append(f'<rect x="376" y="{PT}" width="546" height="{PB - PT}" rx="14" '
             f'fill="{PANEL}"/>')
    s.append(label(400, PT + 26, "PERCEPTION", MUTED, 14, "start", True))

    s.append(box(400, R[0], 498, 116, "Groundfloor segmentation",
                 ["segmente le plan de sol dans le nuage de points",
                  "remplace la detection de sol ecrite a la main"],
                 THEIRS, "SUITE", cloud=True))
    s.append(box(400, R[1], 498, 116, "ADBSCAN",
                 ["regroupe les points hors sol en obstacles",
                  "version Intel, remplace la fusion d'empreintes"],
                 THEIRS, "SUITE", cloud=True))
    s.append(box(400, R[2], 498, 116, "YOLO-seg sur OpenVINO",
                 ["silhouettes des objets, pour la semantique",
                  "et pour l'occlusion du robot compose"],
                 THEIRS, "SUITE", "NPU"))
    s.append(box(400, R[3], 498, 116, "Calibration camera vers sol",
                 ["pose de la camera, validee a 1 % sur la taille rendue",
                  "ancre le monde simule sur la piece reelle"],
                 MINE, "PROJET"))

    # ---- map and navigation panel ---------------------------------------
    s.append(f'<rect x="978" y="{PT}" width="486" height="{PB - PT}" rx="14" '
             f'fill="{PANEL}"/>')
    s.append(label(1002, PT + 26, "CARTE ET NAVIGATION", MUTED, 14, "start", True))

    s.append(box(1002, R[0], 438, 116, "FastMapping",
                 ["carte d'occupation 3D persistante",
                  "au lieu d'un polygone recalcule"],
                 THEIRS, "SUITE", cloud=True))
    s.append(box(1002, R[1], 438, 116, "ITS Path Planner",
                 ["greffon de planification ROS2 Nav2",
                  "remplace le contournement local fait main"],
                 THEIRS, "SUITE"))
    s.append(box(1002, R[2], 438, 116, "Mission et patrouille",
                 ["ce que le robot doit accomplir",
                  "navigator.py, sans dependance au simulateur"],
                 MINE, "PROJET"))
    s.append(box(1002, R[3], 438, 116, "Adaptateur d'incarnation",
                 ["une Pose en entree, une vitesse en sortie",
                  "MuJoCo aujourd'hui, G1 reel demain"],
                 MINE, "PROJET"))

    # ---- right column ----------------------------------------------------
    s.append(box(1520, R[0], 240, 116, "MuJoCo + ROS2",
                 ["simulation du G1", "greffon de la suite"],
                 THEIRS, "SUITE"))
    s.append(box(1520, R[1], 240, 116, "G1 reel",
                 ["odometrie en entree", "vitesses en sortie"],
                 HW, "PLUS TARD"))
    s.append(box(1520, R[2], 240, 116, "Compositeur GPU",
                 ["occlusion par la profondeur", "mesuree, apport du projet"],
                 MINE, "GPU"))
    s.append(box(1520, R[3], 240, 116, "Affichage et KPI",
                 ["video composee a 30 fps",
                  "Benchtool pour les mesures"], HW))

    # ---- the point cloud, in its own channel -----------------------------
    CH = 352                      # vertical channel, clear of both panels
    TOP = 150
    s.append(f'<path d="M 185 {R[1] + 106} L 185 {R[1] + 130} L {CH} '
             f'{R[1] + 130} L {CH} {TOP} L 1221 {TOP} L 1221 {R[0] - 2}" '
             f'stroke="{CLOUD}" stroke-width="3.5" fill="none" '
             f'marker-end="url(#c)"/>')
    for row in (R[0], R[1]):
        s.append(f'<path d="M {CH} {row + 58} L 398 {row + 58}" '
                 f'stroke="{CLOUD}" stroke-width="3.5" fill="none" '
                 f'marker-end="url(#c)"/>')
    s.append(label(1000, TOP - 10, "nuage de points 3D", CLOUD, 14, "middle", True))

    # ---- other flows -----------------------------------------------------
    s.append(f'<path d="M 185 {R[1] + 106} L 185 {R[2] + 58} L 398 {R[2] + 58}" '
             f'stroke="{THEIRS}" stroke-width="2" fill="none" '
             f'marker-end="url(#a)"/>')
    s.append(label(250, R[2] + 48, "images", MUTED, 13, "middle"))

    s.append(arrow(898, R[1] + 58, 1000, R[0] + 90))
    s.append(label(950, R[1] + 30, "obstacles", MUTED, 12, "middle"))
    s.append(arrow(898, R[0] + 58, 1000, R[0] + 40))
    s.append(label(950, R[0] + 34, "plan de sol", MUTED, 12, "middle"))
    s.append(arrow(1221, R[0] + 116, 1221, R[1] - 2))
    s.append(label(1268, R[1] - 12, "carte", MUTED, 12, "start"))
    s.append(arrow(1221, R[1] + 116, 1221, R[2] - 2))
    s.append(label(1268, R[2] - 12, "chemin", MUTED, 12, "start"))
    s.append(arrow(1221, R[2] + 116, 1221, R[3] - 2, MINE, marker="g"))
    s.append(label(1268, R[3] - 12, "consigne", MINE, 12, "start"))

    # Calibration feeds the floor segmentation and the compositor.
    s.append(f'<path d="M 898 {R[3] + 40} L 912 {R[3] + 40} L 912 '
             f'{R[0] + 90} L 900 {R[0] + 90}" stroke="{MINE}" '
             f'stroke-width="1.8" stroke-dasharray="6 5" fill="none" '
             f'marker-end="url(#g)"/>')
    s.append(f'<text x="906" y="{R[2] + 30}" font-family="{F}" font-size="12" '
             f'fill="{MINE}" text-anchor="middle" '
             f'transform="rotate(-90 906 {R[2] + 30})">pose de la camera</text>')

    # Embodiment to the robot, simulated or real.
    s.append(f'<path d="M 1440 {R[3] + 40} L 1480 {R[3] + 40} L 1480 '
             f'{R[0] + 58} L 1518 {R[0] + 58}" stroke="{MINE}" '
             f'stroke-width="2" fill="none" marker-end="url(#g)"/>')
    s.append(f'<path d="M 1480 {R[3] + 40} L 1480 {R[1] + 58} L 1518 '
             f'{R[1] + 58}" stroke="{MINE}" stroke-width="2" fill="none" '
             f'marker-end="url(#g)"/>')
    s.append(f'<text x="1490" y="{R[3] + 66}" font-family="{F}" font-size="12" '
             f'fill="{MINE}" text-anchor="middle" '
             f'transform="rotate(-90 1490 {R[3] + 66})">vitesses</text>')
    s.append(arrow(1640, R[0] + 116, 1640, R[1] - 2, MUTED))
    s.append(label(1672, R[1] - 12, "ou bien", MUTED, 12, "start"))
    s.append(arrow(1640, R[1] + 116, 1640, R[2] - 2, MINE, marker="g"))
    s.append(label(1672, R[2] - 12, "pose", MINE, 12, "start"))
    s.append(arrow(1640, R[2] + 116, 1640, R[3] - 2, MINE, marker="g"))

    # Silhouettes to the compositor, routed under both panels.
    s.append(f'<path d="M 649 {R[2] + 116} L 649 {PB + 14} L 1640 {PB + 14} '
             f'L 1640 {R[3] + 118}" stroke="{THEIRS}" stroke-width="1.8" '
             f'stroke-dasharray="6 5" fill="none" marker-end="url(#a)"/>')
    s.append(label(1120, PB + 34, "silhouettes, pour l'occlusion du robot",
                   MUTED, 13, "middle"))

    # ---- note ------------------------------------------------------------
    s.append(f'<rect x="60" y="880" width="1400" height="128" rx="12" '
             f'fill="{PANEL}"/>')
    s.append(label(84, 914, "Ce que la bascule vers un robot reel demande",
                   INK, 17, "start", True))
    s.append(label(84, 944, "L'adaptateur d'incarnation est le seul point de "
                   "contact avec le robot : il recoit une pose et rend une "
                   "vitesse. Passer du G1 simule au G1 reel consiste a en",
                   MUTED, 14, "start"))
    s.append(label(84, 968, "ecrire une seconde version lisant l'odometrie, "
                   "sans toucher a la perception, a la carte, a la "
                   "planification ni a la mission.", MUTED, 14, "start"))
    s.append(label(84, 994, "Le compositeur reste utile avec un robot reel : "
                   "il sert alors a superposer ce que le robot croit voir sur "
                   "ce que la camera voit.", MUTED, 14, "start"))

    # ---- legend ----------------------------------------------------------
    for i, (name, colour) in enumerate((
            ("Robotics AI Suite", THEIRS), ("ce projet", MINE),
            ("materiel", HW))):
        x = 60 + i * 210
        s.append(f'<rect x="{x}" y="1052" width="16" height="16" rx="4" '
                 f'fill="{colour}"/>')
        s.append(label(x + 24, 1065, name, MUTED, 14, "start"))
    s.append(f'<line x1="700" y1="1060" x2="780" y2="1060" stroke="{CLOUD}" '
             f'stroke-width="3.5"/>')
    s.append(label(790, 1065, "nuage de points 3D", CLOUD, 14, "start", True))
    s.append(f'<rect x="1010" y="1052" width="16" height="16" rx="4" '
             f'fill="none" stroke="{CLOUD}" stroke-width="2.5"/>')
    s.append(label(1034, 1065, "brique qui consomme le nuage de points",
                   MUTED, 14, "start"))

    s.append('</svg>')
    return "\n".join(s)


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "architecture_target.png"
    cairosvg.svg2png(bytestring=build().encode(), write_to=out,
                     output_width=W * 2, output_height=H * 2)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
