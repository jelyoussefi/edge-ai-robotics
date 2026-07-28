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
except ImportError:
    sys.exit(
        "pyrealsense2 et numpy sont requis.\n"
        "  sudo apt install librealsense2-utils python3-pyrealsense2\n"
        "  pip install numpy"
    )

OUT = os.environ.get("CALIB_OUT", "config/camera_calibration.json")


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

    os.makedirs(os.path.dirname(OUT) or ".", exist_ok=True)
    with open(OUT, "w") as fh:
        json.dump(calib, fh, indent=2)
    print(f"\nCalibration écrite dans {OUT}")
    print("Le viewer la chargera pour aligner la caméra virtuelle sur la réelle.")


if __name__ == "__main__":
    main()
