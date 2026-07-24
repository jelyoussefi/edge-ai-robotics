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

# Milestone 2, perception -> sim.
#   {"people": [{"id": int, "xyz": [float, float, float]}], "stamp": float}
PERCEPTION_PEOPLE = "perception.people"

# Velocity limits applied to whatever teleop sends, so a held key cannot ask
# the policy for something it was never trained on.
MAX_VX = 0.8
MAX_VY = 0.4
MAX_WZ = 1.0
