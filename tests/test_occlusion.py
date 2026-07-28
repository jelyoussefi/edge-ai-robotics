# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Test the depth-occlusion compositing rule without GL or hardware.

The rule: a robot pixel is drawn only if the robot is nearer than the real scene
depth at that pixel. Where a real object is closer, the robot is hidden. Depth 0
(no measurement) is treated as infinitely far so holes never punch through.
"""
import numpy as np


def composite_mask(robot_mask, robot_depth, cam_depth):
    no_reading = cam_depth <= 0.05
    real_depth = np.where(no_reading, np.inf, cam_depth)
    return robot_mask & (robot_depth <= real_depth)


def main() -> int:
    H, W = 100, 100
    robot_mask = np.zeros((H, W), bool)
    robot_mask[30:70, 30:70] = True          # robot occupies a central block
    robot_depth = np.full((H, W), 3.0, np.float32)  # robot at 3 m

    # 1. Real object at 2 m over the left half -> hides robot's left.
    cam = np.full((H, W), 5.0, np.float32)
    cam[:, :50] = 2.0
    vis = composite_mask(robot_mask, robot_depth, cam)
    left = robot_mask[:, :50].sum()
    assert vis[:, :50].sum() == 0, "near object must hide robot's left"
    assert vis[:, 50:].sum() == robot_mask[:, 50:].sum(), "far side must show"
    print(f"near object (2m) over 3m robot: left {vis[:, :50].sum()}/{left} visible  ok")

    # 2. Everything far -> robot fully visible.
    cam = np.full((H, W), 9.0, np.float32)
    vis = composite_mask(robot_mask, robot_depth, cam)
    assert vis.sum() == robot_mask.sum(), "all-far scene must show whole robot"
    print("far scene (9m): whole robot visible  ok")

    # 3. Depth holes (0) must NOT occlude.
    cam = np.zeros((H, W), np.float32)        # no readings anywhere
    vis = composite_mask(robot_mask, robot_depth, cam)
    assert vis.sum() == robot_mask.sum(), "depth holes must not punch through"
    print("depth all holes (0): robot still fully visible  ok")

    # 4. Object exactly at robot depth -> tie, robot draws (<=).
    cam = np.full((H, W), 3.0, np.float32)
    vis = composite_mask(robot_mask, robot_depth, cam)
    assert vis.sum() == robot_mask.sum(), "tie should keep robot"
    print("object at same depth: robot draws (tie)  ok")

    print("\nPASS: occlusion rule correct on all cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
