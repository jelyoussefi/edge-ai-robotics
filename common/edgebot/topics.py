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
#   {"roi": [[x, y], ...], "blocked": [[x0, x1, y0, y1], ...],
#    "raw": [[x, y], ...], "stamp": float}
#
# `roi` is where the robot MAY walk: the detected floor minus obstacle
# silhouettes and footprints, then shrunk inward by ROI_MARGIN. That is what
# the navigator follows.
#
# `raw` is where the floor IS, straight from the depth geometry, with none of
# those three applied. Nothing steers on it. It exists so a comparison against
# another floor detection can neutralise our own definitions, which are policy
# and not perception -- comparing a walkable-floor polygon against somebody
# else's ground segmentation measures our margin as much as their detector.
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

# groundfloor -> compositor. A downsampled copy of the suite's labelled cloud,
# for the 'p' display mode. Diagnostic: nothing steers on it.
#
# Raw bytes rather than a list of 4-tuples. At 5000 points a msgpack list of
# lists is around 200 kB and costs a Python loop at both ends; the packed form
# is 5000*3*4 + 5000*2 = 70 kB and unpacks with np.frombuffer. Same reasoning as
# CAMERA_DEPTH, which carries its Z16 the same way.
#
#   {"xyz": bytes,     # n * 3 float32, world frame: x forward, y left, z up
#    "labels": bytes,  # n uint16, the suite's class per point
#    "n": int,         # points in this message, after downsampling
#    "total": int,     # points BEFORE downsampling, so the ratio is visible
#    "stamp": float}
#
# World frame, not camera frame: the bridge already receives them in base_link,
# and converting once here means the compositor needs only its own calibrated
# pose to draw them. These points carry a real z, so the consumer must NOT use
# the floor-plane projection.
SUITE_CLOUD = "suite.cloud"

# adbscan -> anyone. Obstacle clusters from Intel's ADBSCAN (adbscan_ros2), in
# the SAME world rectangles as PATROL_ROI's `blocked` and with the same
# OBSTACLE_MARGIN already applied.
#
# Unlike GROUNDFLOOR_OBSTACLES this is a like-for-like comparison target: both
# sides are obstacle clusters answering "something is here, roughly this big".
# Etape B compared our footprints against a floor segmenter's derived output,
# which measured definitions rather than perception.
#
# `clusters` is the count BEFORE the impossible-return filter, so the two are
# visible separately.
#   {"blocked": [[x0, x1, y0, y1], ...], "clusters": int, "stamp": float}
SUITE_CLUSTERS = "suite.clusters"

# fastmapping -> compositor. Intel's FastMapping planar occupancy grid, the
# `world/map` nav_msgs/OccupancyGrid flattened onto the bus.
#
# The third suite brick and the first PERSISTENT one: groundfloor and adbscan
# both answer "what is in front of the camera right now", and their output is
# recomputed from scratch every frame. This accumulates, so a cell the camera
# saw once stays known after the robot has looked away -- which is the whole
# reason to carry it.
#
# `grid` is row-major, one signed byte per cell, in the ROS convention this
# passes through unchanged: -1 unknown, 0 free, 100 occupied. `x0`/`y0` are the
# world coordinates of the CENTRE of cell (0, 0) and `res` its side in metres,
# so a cell index maps to the world as x = x0 + col * res. Frame is base_link,
# the project's world frame, because the node is given map_frame=base_link --
# see services/fastmapping/entrypoint.sh for why that is the right lie.
#
# `stamp` is when the bridge read it, not when the map was built: the node
# republishes the whole grid on every update and carries no build time.
#   {"grid": bytes, "w": int, "h": int, "res": float,
#    "x0": float, "y0": float, "known": int, "occupied": int, "stamp": float}
SUITE_MAP = "suite.map"

# itsplanner -> compositor. A global path from Intel's ITS Path Planner, as
# world-frame waypoints in the same (forward, lateral) metres as everything
# else here.
#
# Nothing consumes this to DRIVE in E1 -- the robot is not in the loop, the
# navigator does not read it, and the compositor only draws it. That is the
# point of the phase: prove the planner plans on the accumulated map before
# giving it authority over the robot.
#
# `clearance_m` is the minimum distance from any waypoint to an occupied cell
# of the map the path was planned on, computed by the bridge at request time.
# It travels WITH the path because it is only meaningful against that map: a
# path is not safe or unsafe in the abstract, it is safe against the occupancy
# it was planned on, and the map keeps accumulating afterwards.
#   {"path": [[x, y], ...], "length_m": float, "clearance_m": float,
#    "start": [x, y], "goal": [x, y], "planner": str, "stamp": float}
SUITE_PATH = "suite.path"

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
