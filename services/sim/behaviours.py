# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Autonomous behaviour.

Converts detected people into a velocity command. Two behaviours, both cheap
and both legible to someone watching:

  gaze     turn to face the nearest person
  retreat  back off when they come inside the comfort radius

Kept separate from the controller on purpose. This decides *where to go*, the
controller decides *how to move the joints to get there*.
"""

from __future__ import annotations

import numpy as np

# Below this the robot backs away, above it the robot holds position.
COMFORT_M = 1.5
# Full retreat speed is reached this far inside the comfort radius.
PANIC_M = 0.6

YAW_GAIN = 1.6  # rad/s per unit of normalised horizontal offset
RETREAT_GAIN = 0.9  # m/s at full panic
DEADBAND = 0.06  # ignore offsets smaller than this, stops twitching

# Detections older than this are treated as no detection at all, so the robot
# stops rather than chasing a stale target if perception dies.
STALE_S = 1.0


class ReactiveBehaviour:
    def __init__(self) -> None:
        self.people: list[dict] = []
        self.stamp = 0.0
        self.target: dict | None = None

    def observe(self, payload: dict) -> None:
        self.people = payload.get("people", [])
        self.stamp = payload.get("stamp", 0.0)

    def command(self, now: float) -> np.ndarray:
        """Return [vx, vy, wz]. Zero when there is nobody to react to."""
        if not self.people or (now - self.stamp) > STALE_S:
            self.target = None
            return np.zeros(3)

        # People arrive nearest-first from the perception service.
        person = self.people[0]
        self.target = person

        # cx is 0..1 across the frame, so 0.5 is dead ahead. Positive offset
        # means the person is to the right, so the robot yaws negative.
        offset = person["cx"] - 0.5
        wz = 0.0 if abs(offset) < DEADBAND else -YAW_GAIN * offset

        distance = person.get("range_m", 0.0)
        vx = 0.0
        if 0.0 < distance < COMFORT_M:
            urgency = np.clip((COMFORT_M - distance) / (COMFORT_M - PANIC_M), 0.0, 1.0)
            vx = -RETREAT_GAIN * float(urgency)

        return np.array([vx, 0.0, wz])

    def status(self) -> dict:
        if self.target is None:
            return {"tracking": False}
        return {
            "tracking": True,
            "range_m": round(self.target.get("range_m", 0.0), 2),
            "camera": self.target.get("camera", 0),
        }


# ---------------------------------------------------------------------------
# M3: reactive obstacle avoidance with measured depth.
# ---------------------------------------------------------------------------

import math as _math

INFLUENCE_M = 2.5   # obstacles beyond this are ignored
DANGER_M = 0.5      # full repulsion at this range, clamped closer
CRUISE_VX = 0.45    # forward speed on a clear path
REPULSION_GAIN = 1.2
STEER_GAIN = 1.4
FORWARD_BRAKE = 1.0
AVOID_STALE_S = 0.7
UNKNOWN_ASSUMED_M = INFLUENCE_M  # unknown depth -> far but noted


class AvoidBehaviour:
    """Potential-field avoider. Drives forward, steers around measured obstacles.

    No map, no memory: a symmetric wall can trap it in a local minimum. That is
    the accepted limit of a reactive demo and the line real navigation crosses.

    Frame: +x forward, +y left, wz yaw (+ turns left). Obstacle bearing is
    degrees, + to the robot's right.
    """

    def __init__(self) -> None:
        self.obstacles: list[dict] = []
        self.stamp = 0.0
        self._closest = _math.inf

    def observe(self, payload: dict) -> None:
        self.obstacles = payload.get("obstacles", payload.get("people", []))
        self.stamp = payload.get("stamp", 0.0)

    def command(self, now: float) -> np.ndarray:
        if (now - self.stamp) > AVOID_STALE_S:
            self._closest = _math.inf
            return np.zeros(3)

        push = np.zeros(2)
        closest = _math.inf

        for obs in self.obstacles:
            rng = obs.get("range_m", _math.inf)
            if rng is None or not _math.isfinite(rng):
                rng = UNKNOWN_ASSUMED_M
            if rng > INFLUENCE_M:
                continue
            closest = min(closest, rng)

            clamped = max(rng, DANGER_M)
            strength = (INFLUENCE_M - clamped) / (INFLUENCE_M - DANGER_M)
            strength = float(np.clip(strength, 0.0, 1.0)) ** 2

            bearing = _math.radians(obs.get("bearing_deg", 0.0))
            obs_dir = np.array([_math.cos(bearing), -_math.sin(bearing)])
            push -= obs_dir * strength

        self._closest = closest
        if closest == _math.inf:
            return np.array([CRUISE_VX, 0.0, 0.0])

        push *= REPULSION_GAIN
        wz = float(np.clip(STEER_GAIN * push[1], -1.0, 1.0))
        head_on = max(0.0, -push[0])
        vx = CRUISE_VX * float(np.clip(1.0 - FORWARD_BRAKE * head_on, 0.0, 1.0))
        return np.array([vx, 0.0, wz])

    def status(self) -> dict:
        if not _math.isfinite(self._closest):
            return {"avoiding": False}
        return {"avoiding": True, "closest_m": round(self._closest, 2)}
