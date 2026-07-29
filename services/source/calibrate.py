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


def preview_floor(calib: dict, serial, width, height_px, fps) -> dict:
    """Live check of the pose: floor pixels tinted semi-transparent red.

    The calibration is otherwise blind. The measured height is typed in by hand
    and the IMU pitch assumes the camera was still, so nothing tells the
    operator whether the numbers are right until the demo looks wrong. Painting
    the pixels the geometry believes are floor makes an error obvious in one
    glance: too much red climbing the walls means the height is too small, red
    hugging the horizon only means it is too large.

    Height and pitch stay adjustable here, so the values that get saved are the
    ones that were actually seen to work.
    """

    i = calib["intrinsics"]
    h, pitch = calib["camera_height_m"], abs(calib["pitch_deg"])
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

    print("\nVérification visuelle. Les pixels du sol sont en rouge.")
    print("  flèches haut/bas   hauteur  +/- 1 cm")
    print("  flèches gauche/droite  tangage  +/- 0.5 deg")
    print("  r  ajuster automatiquement sur le plan mesuré (RANSAC)")
    print("  s  accepter et enregistrer     q  quitter sans enregistrer")

    win = "Calibration - sol en rouge"
    gui = True
    try:
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    except cv2.error:
        gui = False
        print("\n  Pas d'interface graphique dans ce cv2 "
              "(paquet opencv-python-headless, ou pas de DISPLAY).")
        print("  Repli: capture de quelques images annotées, sans réglage interactif.")
        print("  Pour l'aperçu interactif :")
        print("    pip uninstall -y opencv-python-headless && pip install opencv-python")
    accepted = False
    refine = os.environ.get("FLOOR_REFINE", "1") == "1"
    shots: list = []
    try:
        while True:
            fr = align.process(pipe.wait_for_frames())
            colour = np.asanyarray(fr.get_color_frame().get_data()).copy()
            depth = np.asanyarray(fr.get_depth_frame().get_data()).astype(np.float32) * scale

            det = FloorGeometry(h, pitch, i["fx"], i["fy"], i["ppx"], i["ppy"])
            raw = det.mask(depth, tol_h=tol)
            mask = det.refine(raw, depth) if refine else raw

            # 35% red over the floor pixels, image untouched elsewhere.
            tint = colour.astype(np.float32)
            tint[mask] = tint[mask] * 0.65 + np.array([0.0, 0.0, 255.0]) * 0.35
            view = tint.astype(np.uint8)

            cover = 100.0 * mask.mean()
            valid = 100.0 * (depth > 0).mean()
            for n, txt in enumerate((
                    f"hauteur {h:.2f} m   tangage {pitch:.1f} deg   tol {tol*100:.0f} cm",
                    f"sol {cover:5.1f}% (brut {100.0 * raw.mean():4.1f}%)   "
                    f"profondeur valide {valid:5.1f}%   "
                    f"comblement {'ON' if refine else 'OFF'} (touche c)",
                    "fleches: ajuster   r: auto   c: comblement   s: enregistrer   q: quitter")):
                cv2.putText(view, txt, (12, 26 + 24 * n), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(view, txt, (12, 26 + 24 * n), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 1, cv2.LINE_AA)
            if not gui:
                # Without a window, sample a few frames, save them and report the
                # coverage so the operator can still judge the pose.
                shots.append(view)
                fit = det.fit_plane(depth)
                print(f"  image {len(shots)}: sol {cover:5.1f}% | profondeur valide "
                      f"{valid:5.1f}%" + ("" if fit is None else
                      f" | plan mesuré: hauteur {fit[0]:.2f} m tangage {abs(fit[1]):.1f} deg"))
                if len(shots) >= 5:
                    for n, img in enumerate(shots):
                        cv2.imwrite(f"/tmp/calib_floor_{n}.png", img)
                    print("  images écrites dans /tmp/calib_floor_*.png")
                    break
                continue

            cv2.imshow(win, view)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                accepted = True
                break
            if k == ord("c"):
                refine = not refine
            if k == ord("r"):
                fit = det.fit_plane(depth)
                if fit is None:
                    print("  pas assez de points de sol visibles pour ajuster")
                else:
                    h, pitch, frac = fit[0], abs(fit[1]), fit[2]
                    print(f"  plan mesuré : hauteur {h:.2f} m, tangage {pitch:.1f} deg "
                          f"({frac*100:.0f}% d'inliers)")
            if k in (82, ord("w")):
                h += 0.01
            if k in (84, ord("x")):
                h -= 0.01
            if k in (81, ord("a")):
                pitch -= 0.5
            if k in (83, ord("d")):
                pitch += 0.5
            if gui and cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        pipe.stop()
        if gui:
            cv2.destroyAllWindows()

    if accepted:
        calib["camera_height_m"] = round(h, 3)
        calib["pitch_deg"] = round(-abs(pitch), 2)
        calib["verified_visually"] = True
        print(f"  retenu : hauteur {h:.2f} m, tangage {pitch:.1f} deg")
    else:
        print("  valeurs inchangées")
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
