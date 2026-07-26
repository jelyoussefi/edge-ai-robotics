# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Validate segmentation-keyed compositing in pure numpy.
Given a rendered robot image, a segmentation buffer, and a camera frame,
produce robot-over-camera. This is the load-bearing logic; GL is just what
fills 'rendered' and 'seg' on the real box."""
import numpy as np

def composite(rendered_rgb, seg, camera_bgr):
    """rendered_rgb: HxWx3 uint8 (robot on some bg).
       seg: HxWx2 int32 from MuJoCo segmentation; [...,0] is geom-type/id, -1 = background.
       camera_bgr: HxWx3 uint8 camera frame (already resized to HxW).
       Returns HxWx3 uint8 BGR for cv2.imshow."""
    H, W = rendered_rgb.shape[:2]
    # MuJoCo segmentation: object id in channel 0, -1 where nothing was drawn.
    mask = seg[:, :, 0] >= 0                       # True where robot pixels are
    out = camera_bgr.copy()
    rendered_bgr = rendered_rgb[:, :, ::-1]        # RGB->BGR to match camera
    out[mask] = rendered_bgr[mask]
    return out

# --- synthetic test ---
H, W = 8, 8
# Robot render: white box in the middle rows, black elsewhere.
rendered = np.zeros((H, W, 3), np.uint8)
rendered[3:5, 3:5] = [200, 180, 160]   # RGB-ish robot patch
# Segmentation: object 0 exactly where the robot patch is, -1 elsewhere.
seg = np.full((H, W, 2), -1, np.int32)
seg[3:5, 3:5, 0] = 0
# Camera frame: solid blue (BGR).
cam = np.zeros((H, W, 3), np.uint8); cam[:, :, 0] = 255

out = composite(rendered, seg, cam)

# Background must be the camera (blue), robot area must be the render.
bg_ok = np.all(out[0, 0] == [255, 0, 0])                 # camera blue preserved
robot_px = out[3, 3]
robot_ok = tuple(robot_px) == (160, 180, 200)            # rendered patch, BGR
print("background is camera frame:", bool(bg_ok))
print("robot pixel composited (BGR):", tuple(int(x) for x in robot_px), "expected (160,180,200)")
assert bg_ok, "background not keyed to camera"
assert robot_ok, "robot pixels not composited correctly"

# Edge count: exactly the 2x2 robot patch should differ from camera.
diff = np.any(out != cam, axis=2).sum()
print("composited pixel count:", int(diff), "expected 4")
assert diff == 4, f"expected 4 robot pixels, got {diff}"
print("\nPASS: segmentation keying + BGR compositing correct")
