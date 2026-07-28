#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Isole cv2.imshow : affiche-t-il une image qui CHANGE, sans MuJoCo autour ?

Une barre défile horizontalement sur 5 secondes. Si la barre bouge, imshow
rafraîchit bien et le problème du viewer est le rendu lourd qui affame la boucle
d'événements. Si la barre est figée, imshow lui-même ne rafraîchit pas sur ce
système, et il faut une autre stratégie d'affichage.
"""
import os, sys, time
import numpy as np
try:
    import cv2
except Exception as e:
    sys.exit(f"cv2 import: {e}")

W, H = 1280, 720
cv2.namedWindow("anim", cv2.WINDOW_AUTOSIZE)
print(f"DISPLAY={os.environ.get('DISPLAY')} - une barre doit défiler 5s")
t0 = time.time()
frame = 0
while time.time() - t0 < 5.0:
    img = np.zeros((H, W, 3), np.uint8)
    x = int((time.time() - t0) / 5.0 * W) % W
    cv2.rectangle(img, (x-40, 0), (x+40, H), (0, 200, 0), -1)
    cv2.putText(img, f"frame {frame}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,255,255), 3)
    cv2.imshow("anim", img)
    cv2.waitKey(30)
    frame += 1
cv2.destroyAllWindows()
print(f"{frame} frames affichées. La barre défilait-elle ? Si oui, imshow marche.")
