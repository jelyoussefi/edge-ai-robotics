#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Draw the edge-ai-robotics architecture as a PNG.

A script rather than a drawing tool so the figure can be regenerated when the
design moves; a diagram that has to be redrawn by hand goes stale within a week.

    python3 scripts/architecture_diagram.py [output.png]
"""
from __future__ import annotations

import sys

import cairosvg

W, H = 1600, 1000

INTEL = "#0068B5"
INK = "#12233A"
MUTED = "#5A6B7D"
LINE = "#B9C6D4"
NPU = "#00A3A1"
GPU = "#7B4FBF"
BG = "#FFFFFF"
PANEL = "#F4F7FA"

F = "DejaVu Sans"


def box(x, y, w, h, title, lines, accent=INTEL, chip=None, chip_colour=INTEL):
    """One service card: title, a few lines of detail, optional hardware chip."""
    out = [
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" '
        f'fill="white" stroke="{LINE}" stroke-width="1.5"/>',
        f'<rect x="{x}" y="{y}" width="5" height="{h}" rx="2.5" fill="{accent}"/>',
        f'<text x="{x + 20}" y="{y + 30}" font-family="{F}" font-size="20" '
        f'font-weight="bold" fill="{INK}">{title}</text>',
    ]
    for i, line in enumerate(lines):
        out.append(
            f'<text x="{x + 20}" y="{y + 56 + i * 21}" font-family="{F}" '
            f'font-size="14.5" fill="{MUTED}">{line}</text>')
    if chip:
        cw = 8 * len(chip) + 22
        out.append(
            f'<rect x="{x + w - cw - 14}" y="{y + 14}" width="{cw}" height="24" '
            f'rx="12" fill="{chip_colour}" opacity="0.14"/>')
        out.append(
            f'<text x="{x + w - cw / 2 - 14}" y="{y + 31}" font-family="{F}" '
            f'font-size="13" font-weight="bold" fill="{chip_colour}" '
            f'text-anchor="middle">{chip}</text>')
    return "\n".join(out)


def arrow(x1, y1, x2, y2, label="", dash=False, colour=INTEL, off=-9,
          anchor="middle"):
    d = ' stroke-dasharray="6 5"' if dash else ""
    parts = [f'<path d="M {x1} {y1} L {x2} {y2}" stroke="{colour}" '
             f'stroke-width="2"{d} fill="none" marker-end="url(#a)"/>']
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        parts.append(
            f'<text x="{mx}" y="{my + off}" font-family="{F}" font-size="13" '
            f'fill="{MUTED}" text-anchor="{anchor}">{label}</text>')
    return "\n".join(parts)


def build() -> str:
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}">',
         f'<defs><marker id="a" viewBox="0 0 10 10" refX="9" refY="5" '
         f'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         f'<path d="M 0 0 L 10 5 L 0 10 z" fill="{INTEL}"/></marker></defs>',
         f'<rect width="{W}" height="{H}" fill="{BG}"/>']

    # Title
    s.append(f'<text x="60" y="62" font-family="{F}" font-size="32" '
             f'font-weight="bold" fill="{INK}">Edge AI Robotics</text>')
    s.append(f'<text x="60" y="92" font-family="{F}" font-size="17" '
             f'fill="{MUTED}">A simulated Unitree G1 composited into a live '
             f'camera feed, in real time, on one Intel Core Ultra NUC</text>')
    s.append(f'<line x1="60" y1="112" x2="{W - 60}" y2="112" stroke="{LINE}" '
             f'stroke-width="1.5"/>')

    # Physical inputs and output
    s.append(box(60, 150, 250, 96, "RealSense D455", [
        "640x480 at 30 fps", "colour + aligned depth"], accent=MUTED))
    s.append(box(60, 700, 250, 96, "Display", [
        "composited video", "30 fps"], accent=MUTED))

    # Pipeline lane
    s.append(f'<rect x="360" y="140" width="800" height="670" rx="14" '
             f'fill="{PANEL}"/>')
    s.append(f'<text x="384" y="170" font-family="{F}" font-size="15" '
             f'font-weight="bold" fill="{MUTED}">DOCKER COMPOSE  '
             f'&#183;  ZeroMQ message bus</text>')

    s.append(box(390, 190, 330, 108, "source", [
        "camera capture, depth filtering", "publishes colour and depth"],
        chip="CPU", chip_colour=MUTED))

    s.append(box(390, 340, 330, 130, "perception", [
        "YOLOv11m-seg, 27 ms per frame", "object masks, not just boxes",
        "silhouettes to the compositor"], chip="NPU", chip_colour=NPU))

    s.append(box(800, 340, 330, 130, "sim", [
        "MuJoCo physics at 1 kHz", "RL locomotion policy via OpenVINO",
        "walks, turns, avoids obstacles"], chip="NPU", chip_colour=NPU))

    s.append(box(390, 530, 740, 150, "compositor", [
        "merges the calibrated floor plane with the detected silhouettes",
        "renders the robot off-screen and composites it into the camera frame",
        "depth-accurate occlusion: the robot passes behind real furniture",
        "publishes the walkable floor and the obstacle footprints"],
        chip="GPU", chip_colour=GPU))

    # Right-hand column: what makes it correct
    s.append(box(1200, 190, 340, 170, "Calibration", [
        "camera height and tilt, once",
        "floor plane fitted to the scene",
        "verified: rendered robot within",
        "1 % of its predicted size",
        "at every distance"], accent=NPU))

    s.append(box(1200, 400, 340, 190, "Why it looks real", [
        "the robot is scaled and placed by",
        "the same geometry as the room",
        "it is hidden by objects in front",
        "of it and hides what is behind",
        "it keeps clear of real furniture",
        "detected in the live feed"], accent=GPU))

    s.append(box(1200, 630, 340, 180, "Runs on one machine", [
        "Intel Core Ultra, Panther Lake",
        "NPU: vision and locomotion",
        "GPU: rendering and compositing",
        "CPU: capture and control",
        "no cloud, no discrete GPU"], accent=INTEL))

    # Flows
    s.append(arrow(312, 210, 388, 235, "frames", off=-8))
    s.append(arrow(555, 298, 555, 338, "colour"))
    s.append(arrow(722, 262, 852, 338, "colour + depth", off=-10))
    # The two arrows between sim and compositor are separated horizontally and
    # their labels put on opposite sides: stacked at the same midpoint they
    # overprinted each other.
    s.append(arrow(900, 470, 900, 528, ""))
    s.append(f'<text x="890" y="503" font-family="{F}" font-size="13" '
             f'fill="{MUTED}" text-anchor="end">robot pose</text>')
    s.append(arrow(1010, 528, 1010, 472, "", dash=True))
    s.append(f'<text x="1022" y="496" font-family="{F}" font-size="13" '
             f'fill="{MUTED}">walkable floor</text>')
    s.append(f'<text x="1022" y="512" font-family="{F}" font-size="13" '
             f'fill="{MUTED}">+ obstacles</text>')
    s.append(arrow(388, 660, 312, 730, "", colour=INTEL))
    s.append(f'<text x="352" y="754" font-family="{F}" font-size="13" '
             f'fill="{MUTED}" text-anchor="end">composited frame</text>')

    s.append(box(60, 300, 250, 340, "What it does", [
        "walks the robot up and",
        "down the room",
        "",
        "goes around real chairs",
        "and tables as they move",
        "",
        "stops when there is no",
        "way through, rather than",
        "walking into furniture",
        "",
        "one operator key shows",
        "exactly what the system",
        "believes about the floor"], accent=INTEL))

    # Legend
    s.append(f'<text x="60" y="880" font-family="{F}" font-size="14" '
             f'font-weight="bold" fill="{MUTED}">ACCELERATOR</text>')
    for i, (name, colour) in enumerate((("NPU", NPU), ("GPU", GPU),
                                        ("CPU", MUTED))):
        x = 60 + i * 90
        s.append(f'<rect x="{x}" y="898" width="16" height="16" rx="4" '
                 f'fill="{colour}" opacity="0.35"/>')
        s.append(f'<text x="{x + 24}" y="911" font-family="{F}" font-size="14" '
                 f'fill="{MUTED}">{name}</text>')
    s.append(f'<line x1="60" y1="940" x2="330" y2="940" stroke="{INTEL}" '
             f'stroke-width="2" stroke-dasharray="6 5"/>')
    s.append(f'<text x="340" y="945" font-family="{F}" font-size="14" '
             f'fill="{MUTED}">what the robot is allowed to walk on</text>')

    s.append('</svg>')
    return "\n".join(s)


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "architecture.png"
    svg = build()
    # scale=2 gives a file that stays sharp in a slide deck at full width.
    cairosvg.svg2png(bytestring=svg.encode(), write_to=out, scale=2,
                     output_width=W * 2, output_height=H * 2)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
