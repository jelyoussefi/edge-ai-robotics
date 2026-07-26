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
