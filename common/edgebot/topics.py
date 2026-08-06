# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Topic names and message shapes.

Keeping these in one place means a service can be rewritten in another language
without anyone having to reverse-engineer the wire format.
"""

from __future__ import annotations

# teleop -> sim. Velocity command in the robot's own frame.
#   {"vx": float m/s, "vy": float m/s, "wz": float rad/s, "stamp": float}
CMD_VEL = "cmd.vel"
# Command to reset the robot to its start pose (triggered by 'z' in the compositor).
CMD_RESET = "cmd.reset"
# Manual turn nudge from the operator: {"wz": +right / -left} (from ./, keys).
CMD_TURN = "cmd.turn"
# Keep-out zones as world-ground polygons the robot must not enter.
KEEPOUT_ZONES = "keepout.zones"
# Patrol ROI: the polygon the robot must stay inside (replaces distance bounds).
PATROL_ROI = "patrol.roi"

# Silhouettes of everything the segmentation model found, as one packed boolean
# image. Sent as bits rather than a list of per-object masks: the consumer only
# needs to know which pixels are occupied, and packbits makes a 160x120 mask
# 2400 bytes, small enough to publish every frame.
OBSTACLE_MASK = "perception.mask"

# groundfloor -> anyone. Obstacle footprints computed by Intel's
# pointcloud_groundfloor_segmentation, in the same world rectangles as
# PATROL_ROI's `blocked`. Published ALONGSIDE this project's own, not instead
# of them: step B is a comparison, and replacing before comparing would leave
# no way to tell which one is right.
#   {"blocked": [[x0, x1, y0, y1], ...], "points": int, "stamp": float}
GROUNDFLOOR_OBSTACLES = "groundfloor.obstacles"

# groundfloor -> anyone. The GROUND the suite's node labelled, as a world-plane
# outline in the same (forward, lateral) metres as PATROL_ROI's `roi`. This is
# the topic etape B's criterion is actually about: the node's primary product is
# a labelled cloud, and its floor class is what our floor detection should be
# measured against. The obstacle footprints are a derived view and comparing
# those instead measured the wrong thing.
#
# Outer boundary only, exactly like `roi`: a simple polygon cannot express a
# hole, and obstacles are carried separately on GROUNDFLOOR_OBSTACLES.
#   {"poly": [[x, y], ...], "points": int, "stamp": float}
GROUNDFLOOR_FLOOR = "groundfloor.floor"

# sim -> viewer, dashboard. Full configuration of the model.
#   {"t": float, "qpos": list[float], "fallen": bool}
ROBOT_STATE = "robot.state"

# sim -> dashboard. Loop health, published once a second.
#   {"physics_hz": float, "rtf": float, "jitter_p99_us": float, "policy_ms": float}
SIM_TELEMETRY = "sim.telemetry"

# perception -> sim. People are ordered nearest first.
#   {"people": [{"cx": float, "cy": float, "height": float, "score": float,
#                "range_m": float, "camera": int}],
#    "streams": [{"camera": int, "fps": float, "infer_ms": float, "device": str}],
#    "stamp": float}
#
# cx and cy are normalised 0..1 across the frame. range_m is a monocular
# estimate until the D457 depth stream replaces it in milestone 3.
PERCEPTION_PEOPLE = "perception.people"

# perception -> sim. Measured obstacles, nearest first. Supersedes the M2
# people topic; the schema is a superset so consumers can migrate.
#   {"obstacles": [{"cx": float, "cy": float, "height": float, "score": float,
#                   "range_m": float | null, "bearing_deg": float,
#                   "class_id": int, "measured": bool, "camera": int}],
#    "streams": [{"camera": int, "fps": float, "infer_ms": float,
#                 "device": str, "depth": bool}],
#    "stamp": float}
#
# range_m is metres from aligned depth, or null when depth is unavailable.
# bearing_deg is + to the robot's right, from the camera HFOV.
PERCEPTION_OBSTACLES = "perception.obstacles"

# perception -> viewer. The colour frame of one stream, JPEG-encoded, for use
# as a live backdrop behind the robot. Kept separate from the obstacle topic so
# a viewer can subscribe to pixels without the detector payload, and so the
# heavy frame can run at its own rate.
#   {"jpeg": bytes, "w": int, "h": int, "camera": int, "stamp": float}
CAMERA_FRAME = "camera.frame"

# teleop -> sim. Which source of commands the simulator should obey.
#   {"mode": "manual" | "auto"}
CMD_MODE = "cmd.mode"

MODE_MANUAL = "manual"
MODE_AUTO = "auto"

# Velocity limits applied to whatever teleop sends, so a held key cannot ask
# the policy for something it was never trained on.
MAX_VX = 0.8
MAX_VY = 0.4
MAX_WZ = 1.0

# perception -> central processor. Pure detections: bounding box, confidence,
# and class label only. No distance (the central processor has the depth and
# computes distance itself).
#   {"detections": [{"cx": float, "cy": float, "w": float, "h": float,
#                    "score": float, "class_id": int}],
#    "t": float}
# cx, cy, w, h are normalised 0..1 across the frame. "t" echoes the capture
# timestamp of the RGB frame these detections were computed on, so the compositor
# can pair each detection set with the matching depth frame.
DETECTIONS = "perception.detections"

# source -> compositor, perception. RGB colour frame, JPEG-encoded, with the
# capture timestamp so consumers can align it with the matching depth frame and
# detections.
#   {"jpeg": bytes, "w": int, "h": int, "t": float}
CAMERA_RGB = "camera.rgb"

# source -> compositor. Depth frame as raw uint16 millimetres (or the sensor's
# native Z16), with the SAME capture timestamp as the RGB frame it pairs with.
#   {"depth": bytes, "w": int, "h": int, "scale": float, "t": float}
CAMERA_DEPTH = "camera.depth"
