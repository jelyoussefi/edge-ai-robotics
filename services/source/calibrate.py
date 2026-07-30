#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Calibration de la pose caméra pour placer le robot dans la scène réelle.

Option B, rigoureuse. Produit config/camera_calibration.json que le viewer
charge pour aligner sa caméra virtuelle sur la caméra réelle du D455.

Trois paramètres, deux mesurés automatiquement, un saisi à la main :

  1. FOV vertical (auto, du SDK) : fixe le zoom de la caméra virtuelle MuJoCo.
     Le D455 connaît sa focale exacte, on la lit plutôt que de la deviner.

  2. Inclinaison / pitch (auto, de l'IMU) : au repos l'accéléromètre ne mesure
     que la gravité, dont l'orientation dans le repère caméra donne l'angle.
     Plus précis qu'un rapporteur, à condition que la caméra soit immobile.

  3. Hauteur caméra (saisie) : la seule mesure physique. Distance verticale du
     centre optique au sol, au mètre ou au laser.

Une mesure de référence facultative (objet de hauteur connue à distance connue)
sert à vérifier l'échelle, pas à calibrer.

Usage :
    python3 calibrate_camera.py --height 1.20
    python3 calibrate_camera.py --height 1.20 --serial 220422301817
    python3 calibrate_camera.py --height 1.20 --manual-pitch -12  # sans IMU
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

try:
    import numpy as np
    import pyrealsense2 as rs
except ImportError as exc:
    sys.exit(f"pyrealsense2 et numpy sont requis : {exc}")

try:
    import cv2
except ImportError as exc:
    # Almost always a missing system library rather than a missing wheel, so say
    # which one instead of suggesting a reinstall that will not help.
    sys.exit(f"OpenCV n'a pas pu être chargé : {exc}")

OUT = os.environ.get("CALIB_OUT", "/config/camera_calibration.json")
# The hand-painted floor corrections, read back by the compositor shader.
MASK_OUT = os.environ.get("FLOOR_PAINT", "/config/floor_mask.png")


def read_intrinsics(serial: str | None, width: int, height: int, fps: int) -> dict:
    """Lit les intrinsèques couleur du SDK : FOV, focale, centre optique."""
    cfg = rs.config()
    if serial:
        cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipe = rs.pipeline()
    profile = pipe.start(cfg)
    try:
        stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = stream.get_intrinsics()
        hfov = math.degrees(2 * math.atan2(intr.width / 2.0, intr.fx))
        vfov = math.degrees(2 * math.atan2(intr.height / 2.0, intr.fy))
        return {
            "width": intr.width, "height": intr.height,
            "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
            "hfov_deg": hfov, "vfov_deg": vfov,
        }
    finally:
        pipe.stop()


def read_pitch_from_imu(serial: str | None, samples: int = 200) -> float:
    """Déduit l'inclinaison (pitch, degrés) de l'accéléromètre au repos.

    Repère D455 : X droite, Y bas, Z avant. Au repos la gravité domine, donc
    pitch = atan2(gz, gy). Négatif = caméra inclinée vers le bas. On moyenne
    plusieurs échantillons pour lisser le bruit du capteur.
    """
    cfg = rs.config()
    if serial:
        cfg.enable_device(serial)
    # Le D455 expose l'accéléromètre à des débits précis (100/200/400 Hz).
    # Sans débit, la requête peut ne pas se résoudre, d'où "Couldn't resolve
    # requests". On tente les débits connus, du plus courant au plus rare.
    pipe = rs.pipeline()
    profile = None
    last_err = None
    for rate in (100, 200, 400):
        try:
            cfg_try = rs.config()
            if serial:
                cfg_try.enable_device(serial)
            cfg_try.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, rate)
            profile = pipe.start(cfg_try)
            break
        except RuntimeError as e:
            last_err = e
            continue
    if profile is None:
        # Dernier essai : laisser le SDK choisir le débit par défaut.
        try:
            cfg_any = rs.config()
            if serial:
                cfg_any.enable_device(serial)
            cfg_any.enable_stream(rs.stream.accel)
            profile = pipe.start(cfg_any)
        except RuntimeError:
            raise RuntimeError(
                f"impossible de démarrer l'accéléromètre ({last_err}). "
                "Utilise --manual-pitch pour saisir l'inclinaison à la main."
            )
    try:
        acc = np.zeros(3)
        n = 0
        deadline = time.time() + 5.0
        while n < samples and time.time() < deadline:
            frames = pipe.wait_for_frames()
            f = frames.first_or_default(rs.stream.accel)
            if not f:
                continue
            m = f.as_motion_frame().get_motion_data()
            acc += np.array([m.x, m.y, m.z])
            n += 1
        if n == 0:
            raise RuntimeError("aucune donnée accéléromètre reçue")
        acc /= n
        # Pitch autour de l'axe X. Au repos, l'accéléromètre mesure la gravité.
        # Sur le D455, l'axe Y pointe vers le haut, donc gy est proche de -g au
        # repos horizontal. On calcule l'angle de la gravité par rapport à la
        # verticale (-Y) dans le plan Y-Z: atan2(gz, -gy). Ça donne 0 à
        # l'horizontale et un petit angle négatif quand la caméra plonge.
        # (L'ancienne formule atan2(gz, gy) tombait dans le mauvais quadrant quand
        # gy<0 et renvoyait des valeurs aberrantes proches de -180.)
        pitch = math.degrees(math.atan2(acc[2], -acc[1]))
        # Convention : caméra horizontale -> 0 ; inclinée vers le bas -> négatif.
        # atan2(gz, gy) vaut 0 à l'horizontale (g selon +Y), on garde le signe tel quel.
        return pitch, {"ax": float(acc[0]), "ay": float(acc[1]), "az": float(acc[2]), "samples": n}
    finally:
        pipe.stop()



class FloorGeometry:
    """A pixel is floor when its 3D point sits on the ground plane.

    Back-project the pixel with the intrinsics to get the camera-frame ray
    (x, y, 1); the point RealSense reports at perpendicular depth Z is Z*(x,y,1).
    Only the vertical component of that ray changes height, since without camera
    roll the horizontal axis is parallel to the floor. Height above ground is
    then H + Z*world_dir_z, and the test is a single distance-independent
    threshold, because a height tolerance maps to a depth tolerance that widens
    with distance exactly as stereo error does.
    """

    def __init__(self, height_m, pitch_deg, fx, fy, ppx, ppy):
        self.H = height_m
        self.pitch = math.radians(abs(pitch_deg))
        self.fx, self.fy, self.ppx, self.ppy = fx, fy, ppx, ppy

    def _rays(self, dw, dh):
        uu, vv = np.meshgrid(np.arange(dw), np.arange(dh))
        fx, fy = self.fx * dw / 640.0, self.fy * dh / 480.0
        ppx, ppy = self.ppx * dw / 640.0, self.ppy * dh / 480.0
        return (uu - ppx) / fx, (vv - ppy) / fy

    def height_map(self, depth_m):
        _, y = self._rays(depth_m.shape[1], depth_m.shape[0])
        return self.H + depth_m * (y * (-math.cos(self.pitch)) - math.sin(self.pitch))

    def mask(self, depth_m, tol_h=0.08):
        return (depth_m > 0) & (np.abs(self.height_map(depth_m)) < tol_h)

    def refine(self, mask, depth_m, close_px=9, min_area_frac=0.02):
        """Close the holes a depth sensor leaves in an otherwise flat floor.

        An obstacle returns a SHORTER depth, never a missing one, so a gap with
        no depth, below the horizon and surrounded by floor, is floor. Pixels
        whose valid depth contradicts the plane are removed again, which is what
        keeps furniture and people out.
        """
        m = mask.astype(np.uint8)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        h, w = m.shape
        ff = m.copy()
        cv2.floodFill(ff, np.zeros((h + 2, w + 2), np.uint8), (0, 0), 1)
        m = (m | (1 - ff)).astype(np.uint8)
        m[(depth_m > 0) & (np.abs(self.height_map(depth_m)) >= 0.20)] = 0
        num, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        out = np.zeros_like(m, dtype=bool)
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] >= min_area_frac * h * w:
                out |= lab == i
        return out

    def fit_plane(self, depth_m, inlier_m=0.05, iters=80):
        """Measure the real ground plane by RANSAC instead of trusting the input."""
        x, y = self._rays(depth_m.shape[1], depth_m.shape[0])
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        sel = (depth_m > 0) & ((y * (-cp) - sp) < -1e-3)
        if int(sel.sum()) < 500:
            return None
        Z = depth_m[sel]
        pts = np.stack([x[sel] * Z, y[sel] * Z, Z], axis=1)
        if len(pts) > 40000:
            pts = pts[np.linspace(0, len(pts) - 1, 40000).astype(int)]
        rng = np.random.default_rng(0)
        best = (0, None, 0.0)
        for _ in range(iters):
            i = rng.choice(len(pts), 3, replace=False)
            n = np.cross(pts[i[1]] - pts[i[0]], pts[i[2]] - pts[i[0]])
            nn = np.linalg.norm(n)
            if nn < 1e-9:
                continue
            n = n / nn
            d = float(n @ pts[i[0]])
            hits = int((np.abs(pts @ n - d) < inlier_m).sum())
            if hits > best[0]:
                best = (hits, n, d)
        hits, n, d = best
        if n is None:
            return None
        inl = pts[np.abs(pts @ n - d) < inlier_m]
        c = inl.mean(axis=0)
        n = np.linalg.svd(inl - c)[2][2]
        d = float(n @ c)
        if n[1] > 0:
            n, d = -n, -d
        return abs(d), float(math.degrees(math.atan2(-n[2], -n[1]))), hits / len(pts)


# ---------------------------------------------------------------------------
# Calibration UI. Drawn by hand rather than with cv2 trackbars: the native
# widgets cannot be styled, they sit outside the image in a grey strip, and
# they cannot express a brush tool at all. Everything below is plain OpenCV
# drawing on the frame, so there is no extra dependency and full control.
# ---------------------------------------------------------------------------

BG        = (24, 20, 17)       # panel, BGR (deep slate)
FG        = (238, 240, 244)    # primary text
DIM       = (150, 148, 145)    # secondary text
ACCENT    = (224, 163, 0)      # Intel blue in BGR
WARN      = (90, 120, 245)     # eraser / destructive, warm red in BGR
TRACK     = (62, 56, 50)

UI_W, IMG_H, PANEL_H = 1100, 825, 172
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _rounded(img, x1, y1, x2, y2, colour, r=14, thickness=-1):
    """Rounded rectangle: OpenCV has no primitive for one."""
    if thickness < 0:
        cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), colour, -1)
        cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), colour, -1)
        for cx, cy in ((x1 + r, y1 + r), (x2 - r, y1 + r),
                       (x1 + r, y2 - r), (x2 - r, y2 - r)):
            cv2.circle(img, (cx, cy), r, colour, -1, cv2.LINE_AA)
    else:
        cv2.rectangle(img, (x1, y1), (x2, y2), colour, thickness, cv2.LINE_AA)


def _glass(img, x1, y1, x2, y2, alpha=0.82, colour=BG, r=16):
    """Translucent rounded panel over whatever is underneath."""
    roi = img[y1:y2, x1:x2]
    if roi.size == 0:
        return
    layer = img.copy()
    _rounded(layer, x1, y1, x2, y2, colour, r)
    cv2.addWeighted(layer[y1:y2, x1:x2], alpha, roi, 1 - alpha, 0, dst=roi)


def _text(img, s, x, y, scale=0.52, colour=FG, weight=1, shadow=True):
    if shadow:
        cv2.putText(img, s, (x + 1, y + 1), FONT, scale, (0, 0, 0),
                    weight + 2, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), FONT, scale, colour, weight, cv2.LINE_AA)


class Slider:
    """A labelled horizontal slider with a live value readout."""

    def __init__(self, key, label, unit, x, y, w, lo, hi, value, fmt="{:.2f}",
                 step=0.01, coarse=0.05):
        self.key, self.label, self.unit = key, label, unit
        self.x, self.y, self.w = x, y, w
        self.lo, self.hi, self.value, self.fmt = lo, hi, value, fmt
        # Arrow keys: left/right move by one step, up/down by a coarse one.
        self.step, self.coarse = step, coarse

    def nudge(self, amount):
        self.value = min(self.hi, max(self.lo, self.value + amount))

    @property
    def frac(self):
        return (self.value - self.lo) / (self.hi - self.lo)

    def hit(self, mx, my):
        return self.x - 14 <= mx <= self.x + self.w + 14 and abs(my - self.y) <= 20

    def set_from(self, mx):
        f = min(1.0, max(0.0, (mx - self.x) / float(self.w)))
        self.value = self.lo + f * (self.hi - self.lo)

    def draw(self, img, active, focused=False):
        if focused:
            _rounded(img, self.x - 18, self.y - 38, self.x + self.w + 18,
                     self.y + 20, (44, 39, 34), 12)
            cv2.circle(img, (self.x - 10, self.y - 27), 3, ACCENT, -1, cv2.LINE_AA)
        _text(img, self.label, self.x, self.y - 20, 0.46,
              FG if focused else DIM)
        val = self.fmt.format(self.value) + " " + self.unit
        (tw, _), _ = cv2.getTextSize(val, FONT, 0.56, 2)
        _text(img, val, self.x + self.w - tw, self.y - 20, 0.56, FG, 2)
        cv2.line(img, (self.x, self.y), (self.x + self.w, self.y),
                 TRACK, 5, cv2.LINE_AA)
        cx = int(self.x + self.w * self.frac)
        cv2.line(img, (self.x, self.y), (cx, self.y), ACCENT, 5, cv2.LINE_AA)
        cv2.circle(img, (cx, self.y), 13 if active else 11, (255, 255, 255),
                   -1, cv2.LINE_AA)
        cv2.circle(img, (cx, self.y), 13 if active else 11, ACCENT, 2, cv2.LINE_AA)


def _icon_brush(img, cx, cy, col, bg=BG):
    """Paint brush, tip pointing down-left."""
    handle = np.array([[cx + 4, cy - 15], [cx + 13, cy - 6],
                       [cx - 2, cy + 9], [cx - 11, cy]], np.int32)
    cv2.fillPoly(img, [handle], col, cv2.LINE_AA)
    ferrule = np.array([[cx - 11, cy], [cx - 2, cy + 9],
                        [cx - 7, cy + 14], [cx - 16, cy + 5]], np.int32)
    cv2.fillPoly(img, [ferrule], col, cv2.LINE_AA)
    tip = np.array([[cx - 16, cy + 5], [cx - 7, cy + 14],
                    [cx - 18, cy + 18]], np.int32)
    cv2.fillPoly(img, [tip], col, cv2.LINE_AA)


def _icon_eraser(img, cx, cy, col, bg=BG):
    """Eraser block seen at an angle, with its worn face."""
    body = np.array([[cx - 3, cy - 15], [cx + 15, cy + 2],
                     [cx + 3, cy + 15], [cx - 15, cy - 2]], np.int32)
    cv2.fillPoly(img, [body], col, cv2.LINE_AA)
    face = np.array([[cx - 9, cy + 4], [cx + 3, cy + 15],
                     [cx - 3, cy + 18], [cx - 15, cy + 7]], np.int32)
    cv2.fillPoly(img, [face], bg, cv2.LINE_AA)
    cv2.polylines(img, [body], True, bg, 1, cv2.LINE_AA)


def _icon_target(img, cx, cy, col, bg=BG):
    """Crosshair: measure the plane and snap to it."""
    cv2.circle(img, (cx, cy), 11, col, 2, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 3, col, -1, cv2.LINE_AA)
    for dx, dy in ((0, -17), (0, 17), (-17, 0), (17, 0)):
        cv2.line(img, (cx + dx // 2, cy + dy // 2), (cx + dx, cy + dy),
                 col, 2, cv2.LINE_AA)


def _icon_undo(img, cx, cy, col, bg=BG):
    cv2.ellipse(img, (cx, cy + 2), (12, 12), 0, 30, 320, col, 2, cv2.LINE_AA)
    head = np.array([[cx + 3, cy - 13], [cx + 14, cy - 9],
                     [cx + 5, cy - 2]], np.int32)
    cv2.fillPoly(img, [head], col, cv2.LINE_AA)


class Button:
    """Square icon button with a caption and an on/off state."""

    ICONS = {"brush": _icon_brush, "eraser": _icon_eraser,
             "target": _icon_target, "undo": _icon_undo}

    def __init__(self, key, icon, caption, x, y, size=58, accent=ACCENT):
        self.key, self.icon, self.caption = key, icon, caption
        self.x, self.y, self.size, self.accent = x, y, size, accent

    def hit(self, mx, my):
        return (self.x <= mx <= self.x + self.size
                and self.y <= my <= self.y + self.size)

    def draw(self, img, active, hover=False):
        x2, y2 = self.x + self.size, self.y + self.size
        fill = self.accent if active else (46, 41, 36)
        _rounded(img, self.x, self.y, x2, y2, fill, 13)
        if not active:
            _rounded(img, self.x, self.y, x2, y2,
                     (86, 80, 74) if hover else (62, 56, 50), 13, 1)
        col = (18, 18, 18) if active else FG
        self.ICONS[self.icon](img, (self.x + x2) // 2, (self.y + y2) // 2,
                              col, fill)
        (tw, _), _ = cv2.getTextSize(self.caption, FONT, 0.38, 1)
        _text(img, self.caption, (self.x + x2) // 2 - tw // 2, y2 + 17,
              0.38, FG if active else DIM)


class CalibrationUI:
    """Owns the widget layout, the mouse state and the hand-painted mask."""

    def __init__(self, height_m, tol_m, dw, dh):
        self.sl = {
            "height": Slider("height", "CAMERA HEIGHT", "m", 44, 62, 300,
                             0.60, 3.00, height_m, "{:.2f}", 0.01, 0.05),
            "tol":    Slider("tol", "FLOOR THRESHOLD", "cm", 44, 132, 300,
                             1.0, 40.0, tol_m * 100.0, "{:.0f}", 1.0, 5.0),
            "brush":  Slider("brush", "BRUSH SIZE", "px", 420, 150, 330,
                             4.0, 90.0, 26.0, "{:.0f}", 2.0, 10.0),
        }
        self.order = ["height", "tol", "brush"]
        self.focus = "height"
        self.bt = [
            Button("brush",  "brush",  "FILL",    440, 36, 58, ACCENT),
            Button("eraser", "eraser", "ERASE",   512, 36, 58, WARN),
            Button("fit",    "target", "MEASURE", 584, 36, 58, ACCENT),
            Button("undo",   "undo",   "RESET",   656, 36, 58, WARN),
        ]
        self.tool = None                       # "brush" | "eraser" | None
        self.drag = None                       # slider being dragged
        self.paint = False
        self.hover = None
        self.fit_request = False
        self.add = np.zeros((dh, dw), bool)    # pixels forced to floor
        self.rem = np.zeros((dh, dw), bool)    # pixels forced not-floor
        self.dw, self.dh = dw, dh
        self.scale = 1.0                       # display px -> depth px

    # -- persistence ------------------------------------------------------
    def load(self, path):
        """Restore a mask painted in a previous session.

        Stored as a single greyscale image so the compositor can upload it
        straight to a texture: 255 forces floor, 128 forces not-floor, 0 leaves
        the decision to the geometry.
        """
        if not os.path.exists(path):
            return False
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return False
        if img.shape[:2] != (self.dh, self.dw):
            img = cv2.resize(img, (self.dw, self.dh), interpolation=cv2.INTER_NEAREST)
        self.add = img > 200
        self.rem = (img > 80) & (img <= 200)
        print(f"  mask restored from {path}: "
              f"+{int(self.add.sum())} / -{int(self.rem.sum())} px")
        return True

    def save(self, path):
        """Write the mask, or remove a stale one when nothing is painted."""
        if not (self.add.any() or self.rem.any()):
            if os.path.exists(path):
                os.remove(path)
                print(f"  mask empty, removed {path}")
            return False
        img = np.where(self.add, 255, np.where(self.rem, 128, 0)).astype(np.uint8)
        cv2.imwrite(path, img)
        print(f"  mask written to {path}: "
              f"+{int(self.add.sum())} / -{int(self.rem.sum())} px")
        return True

    # -- geometry ---------------------------------------------------------
    def _panel_xy(self, mx, my):
        return mx, my - IMG_H

    def on_mouse(self, ev, mx, my, flags, _):
        if my >= IMG_H:                        # inside the control panel
            px, py = self._panel_xy(mx, my)
            self.hover = next((b.key for b in self.bt if b.hit(px, py)), None)
            if ev == cv2.EVENT_LBUTTONDOWN:
                for s in self.sl.values():
                    if s.hit(px, py):
                        self.drag = self.focus = s.key
                        s.set_from(px)
                        return
                for b in self.bt:
                    if b.hit(px, py):
                        if b.key in ("brush", "eraser"):
                            self.tool = None if self.tool == b.key else b.key
                        elif b.key == "fit":
                            self.fit_request = True
                        else:
                            self.add[:] = False
                            self.rem[:] = False
                        return
            elif ev == cv2.EVENT_MOUSEMOVE and self.drag:
                self.sl[self.drag].set_from(px)
            elif ev == cv2.EVENT_LBUTTONUP:
                self.drag = None
            return

        self.hover = None
        if self.tool is None:
            return
        if ev == cv2.EVENT_LBUTTONDOWN:
            self.paint = True
        elif ev == cv2.EVENT_LBUTTONUP:
            self.paint = False
        if self.paint and ev in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_MOUSEMOVE):
            self._stroke(mx, my)

    def _stroke(self, mx, my):
        """Paint into the manual masks, in depth-image coordinates."""
        r = max(1, int(self.sl["brush"].value * self.scale))
        c = (int(mx * self.scale), int(my * self.scale))
        tgt, other = ((self.add, self.rem) if self.tool == "brush"
                      else (self.rem, self.add))
        t = tgt.astype(np.uint8)
        o = other.astype(np.uint8)
        cv2.circle(t, c, r, 1, -1)
        cv2.circle(o, c, r, 0, -1)            # the two tools are exclusive
        tgt[:] = t.astype(bool)
        other[:] = o.astype(bool)

    # -- keyboard ---------------------------------------------------------
    UP    = {82, 65362, 2490368}
    DOWN  = {84, 65364, 2621440}
    LEFT  = {81, 65361, 2424832}
    RIGHT = {83, 65363, 2555904}

    def on_key(self, key):
        """Arrow keys drive the focused slider; Tab moves the focus."""
        s = self.sl[self.focus]
        if key in self.LEFT:
            s.nudge(-s.step)
        elif key in self.RIGHT:
            s.nudge(+s.step)
        elif key in self.DOWN:
            s.nudge(-s.coarse)
        elif key in self.UP:
            s.nudge(+s.coarse)
        elif (key & 0xFF) == 9:                       # Tab
            i = self.order.index(self.focus)
            self.focus = self.order[(i + 1) % len(self.order)]
        else:
            return False
        return True

    # -- drawing ----------------------------------------------------------
    def draw_panel(self, canvas, stats):
        p = canvas[IMG_H:, :]
        p[:] = BG
        cv2.line(p, (0, 0), (UI_W, 0), (54, 48, 43), 1, cv2.LINE_AA)
        for s in self.sl.values():
            s.draw(p, self.drag == s.key, self.focus == s.key)
        for b in self.bt:
            b.draw(p, b.key == self.tool, self.hover == b.key)

        x = 830
        _text(p, "FLOOR COVERAGE", x, 40, 0.44, DIM)
        _text(p, f"{stats['cover']:.1f} %", x, 76, 1.0, ACCENT, 2)
        _text(p, f"valid depth  {stats['valid']:.0f} %", x, 104, 0.44, DIM)
        _text(p, f"painted  +{stats['add']}  -{stats['rem']}", x, 126, 0.44, DIM)
        _text(p, "ARROWS  adjust     TAB  next", x, 148, 0.42, DIM)
        _text(p, "S  SAVE          Q  QUIT", x, 166, 0.44, FG)

    def draw_cursor(self, canvas, mx, my):
        if self.tool is None or my >= IMG_H:
            return
        r = int(self.sl["brush"].value)
        col = ACCENT if self.tool == "brush" else WARN
        cv2.circle(canvas, (mx, my), r, col, 2, cv2.LINE_AA)
        cv2.circle(canvas, (mx, my), 2, col, -1, cv2.LINE_AA)


def preview_floor(calib: dict, serial, width, height_px, fps) -> dict:
    """Live check of the pose, with the floor tinted red.

    The calibration is otherwise blind: the height is typed in by hand and the
    IMU pitch assumes the camera was still, so nothing tells the operator
    whether the numbers are right until the demo looks wrong. Painting the
    pixels the geometry believes are floor makes an error obvious at a glance.

    The sliders adjust the two quantities that actually matter, and the brush
    and eraser let the operator correct what the sensor cannot see: polished
    tiles reflect the IR pattern away and simply return no depth, so they can be
    filled in by hand rather than left as holes.
    """
    i = calib["intrinsics"]
    h = calib["camera_height_m"]
    pitch = abs(calib["pitch_deg"])
    tol = float(os.environ.get("FLOOR_H_TOL", "0.08"))

    cfg = rs.config()
    if serial:
        cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, width, height_px, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, width, height_px, rs.format.z16, fps)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()

    ui = CalibrationUI(h, tol, width, height_px)
    ui.load(MASK_OUT)
    ui.scale = width / float(UI_W)
    win = "Calibration"
    try:
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    except cv2.error as exc:
        pipe.stop()
        print(f"  No GUI available: {exc}")
        print("  Install opencv-python (not -headless) and export DISPLAY.")
        return calib

    mouse = [0, 0]

    def on_mouse(ev, x, y, flags, param):
        mouse[0], mouse[1] = x, y
        ui.on_mouse(ev, x, y, flags, param)

    cv2.setMouseCallback(win, on_mouse)
    canvas = np.zeros((IMG_H + PANEL_H, UI_W, 3), np.uint8)
    accepted = False

    try:
        while True:
            fr = align.process(pipe.wait_for_frames())
            colour = np.asanyarray(fr.get_color_frame().get_data())
            depth = np.asanyarray(fr.get_depth_frame().get_data()).astype(np.float32) * scale

            h = ui.sl["height"].value
            tol = ui.sl["tol"].value / 100.0
            det = FloorGeometry(h, pitch, i["fx"], i["fy"], i["ppx"], i["ppy"])

            if ui.fit_request:
                ui.fit_request = False
                fit = det.fit_plane(depth)
                if fit is None:
                    print("  not enough floor points in view to fit a plane")
                else:
                    h, pitch = fit[0], abs(fit[1])
                    ui.sl["height"].value = h
                    print(f"  measured plane: height {h:.2f} m, pitch "
                          f"{pitch:.1f} deg ({fit[2]*100:.0f}% inliers)")
                    continue

            auto = det.refine(det.mask(depth, tol_h=tol), depth)
            mask = (auto | ui.add) & ~ui.rem


            # Tint, then upscale once so the overlay stays crisp at UI size.
            tint = colour.astype(np.float32)
            tint[mask] = tint[mask] * 0.62 + np.array([26.0, 26.0, 255.0]) * 0.38
            view = cv2.resize(tint.astype(np.uint8), (UI_W, IMG_H),
                              interpolation=cv2.INTER_LINEAR)
            edges = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
            edges = cv2.resize(edges - mask.astype(np.uint8), (UI_W, IMG_H),
                               interpolation=cv2.INTER_NEAREST).astype(bool)
            view[edges] = (60, 60, 255)

            canvas[:IMG_H] = view
            _glass(canvas, 18, 18, 330, 92, 0.72)
            _text(canvas, f"{h:.2f} m    {pitch:.1f} deg", 36, 52, 0.72, FG, 2)
            _text(canvas, "estimated camera pose", 36, 76, 0.44, DIM)

            ui.draw_panel(canvas, {
                "cover": 100.0 * mask.mean(),
                "valid": 100.0 * (depth > 0).mean(),
                "add": int(ui.add.sum()), "rem": int(ui.rem.sum())})
            ui.draw_cursor(canvas, mouse[0], mouse[1])
            cv2.imshow(win, canvas)

            key = cv2.waitKeyEx(1)
            if key != -1 and ui.on_key(key):
                continue
            k = key & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                accepted = True
                break
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

    if accepted:
        calib["camera_height_m"] = round(float(ui.sl["height"].value), 3)
        calib["pitch_deg"] = round(-abs(pitch), 2)
        calib["floor_h_tol_m"] = round(ui.sl["tol"].value / 100.0, 3)
        calib["verified_visually"] = True
        ui.save(MASK_OUT)
        print(f"  saved: height {calib['camera_height_m']:.2f} m, "
              f"pitch {pitch:.1f} deg, threshold {calib['floor_h_tol_m']*100:.0f} cm")
    else:
        print("  values unchanged")
    return calib


def main() -> None:
    ap = argparse.ArgumentParser(description="Calibration extrinsèque de la caméra (option B).")
    ap.add_argument("--height", type=float, required=True,
                    help="hauteur du centre optique au sol, en mètres (mesurée)")
    ap.add_argument("--serial", default=None, help="numéro de série du D455")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height-px", type=int, default=480, dest="height_px")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--manual-pitch", type=float, default=None,
                    help="inclinaison en degrés si tu ne veux pas lire l'IMU")
    ap.add_argument("--ref-distance", type=float, default=None,
                    help="(vérif) distance au sol d'un objet de référence, en m")
    ap.add_argument("--ref-height", type=float, default=None,
                    help="(vérif) hauteur réelle de cet objet, en m")
    ap.add_argument("--no-preview", action="store_true",
                    help="ne pas afficher la vérification visuelle du sol")
    args = ap.parse_args()

    print("Lecture des intrinsèques (FOV) depuis le SDK ...")
    intr = read_intrinsics(args.serial, args.width, args.height_px, args.fps)
    print(f"  HFOV {intr['hfov_deg']:.1f} deg, VFOV {intr['vfov_deg']:.1f} deg "
          f"({intr['width']}x{intr['height']})")

    if args.manual_pitch is not None:
        pitch = args.manual_pitch
        imu = {"manual": True}
        print(f"Inclinaison (saisie) : {pitch:.1f} deg")
    else:
        print("Lecture de l'IMU (garde la caméra IMMOBILE) ...")
        pitch, imu = read_pitch_from_imu(args.serial)
        print(f"  accel moyen (m/s^2) : ax={imu['ax']:.2f} ay={imu['ay']:.2f} az={imu['az']:.2f}")
        print(f"  inclinaison déduite : {pitch:.1f} deg "
              f"({'vers le bas' if pitch < 0 else 'vers le haut' if pitch > 0 else 'horizontale'})")

    calib = {
        "camera_height_m": args.height,
        "pitch_deg": round(pitch, 2),
        "vfov_deg": round(intr["vfov_deg"], 3),
        "hfov_deg": round(intr["hfov_deg"], 3),
        "intrinsics": intr,
        "imu": imu,
    }

    # Vérification facultative d'échelle.
    if args.ref_distance and args.ref_height:
        # Taille angulaire attendue d'un objet de ref_height à ref_distance,
        # comparée à ce que le FOV implique. Simple contrôle de cohérence.
        ang = math.degrees(2 * math.atan2(args.ref_height / 2.0, args.ref_distance))
        frac = ang / intr["vfov_deg"]
        calib["scale_check"] = {
            "ref_distance_m": args.ref_distance,
            "ref_height_m": args.ref_height,
            "expected_frac_of_frame": round(frac, 3),
        }
        print(f"\nVérif échelle : un objet de {args.ref_height} m à {args.ref_distance} m "
              f"doit occuper ~{frac*100:.0f}% de la hauteur d'image.")
        print("  Compare avec l'image réelle pour valider.")

    if not args.no_preview:
        try:
            calib = preview_floor(calib, args.serial, args.width,
                                  args.height_px, args.fps)
        except Exception as exc:
            print(f"Vérification visuelle indisponible ({exc}), on continue.")

    os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
    with open(OUT, "w") as fh:
        json.dump(calib, fh, indent=2)
    print(f"\nCalibration écrite dans {OUT}")
    print("Le viewer la chargera pour aligner la caméra virtuelle sur la réelle.")


if __name__ == "__main__":
    main()
