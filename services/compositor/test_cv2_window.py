#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Test minimal : cv2.imshow peut-il ouvrir une fenêtre ?

À lancer DANS le conteneur viewer pour vérifier que la pile OpenCV/Qt/xcb est
correcte, isolément du reste de la démo. Si ça affiche une fenêtre verte une
seconde et se termine proprement, cv2.imshow fonctionne et le viewer compositing
peut tourner. Si ça plante ou échoue, le problème est dans les libs GUI, pas
dans le reste du code.

Usage, depuis l'hôte :
    docker compose run --rm viewer python test_cv2_window.py
"""
import os
import sys

import numpy as np

print(f"DISPLAY = {os.environ.get('DISPLAY', '(non défini)')}")
if not os.environ.get("DISPLAY"):
    sys.exit("Pas de DISPLAY. Le conteneur ne peut pas ouvrir de fenêtre.")

try:
    import cv2
    print(f"OpenCV {cv2.__version__}")
except Exception as e:  # noqa: BLE001
    sys.exit(f"Import cv2 échoué : {e}")

try:
    img = np.zeros((240, 320, 3), np.uint8)
    img[:] = (0, 180, 0)
    cv2.putText(img, "cv2.imshow OK", (30, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.namedWindow("test", cv2.WINDOW_AUTOSIZE)
    for _ in range(60):  # ~1 s
        cv2.imshow("test", img)
        if (cv2.waitKey(16) & 0xFF) in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    print("\nSUCCÈS : cv2.imshow a ouvert une fenêtre. Le viewer compositing peut tourner.")
except Exception as e:  # noqa: BLE001
    print(f"\nÉCHEC : cv2.imshow n'a pas pu ouvrir de fenêtre : {e}")
    print("La pile Qt/xcb du conteneur est incomplète. Vérifie les libs xcb "
          "dans le Dockerfile du viewer.")
    sys.exit(1)
