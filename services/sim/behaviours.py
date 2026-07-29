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

import os

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

INFLUENCE_M = 0.4   # obstacles beyond this are ignored (react only when very close)
DANGER_M = 0.1      # full repulsion at this range, clamped closer
CRUISE_VX = 0.45    # forward speed on a clear path
REPULSION_GAIN = 1.2
STEER_GAIN = 1.4
FORWARD_BRAKE = 1.0
AVOID_STALE_S = 0.7
UNKNOWN_ASSUMED_M = INFLUENCE_M  # unknown depth -> far but noted

# Patrol: walk a fixed distance, turn about-face, walk back, repeat. Distance is
# integrated from the commanded forward speed, so no map is needed. The turn is
# a fixed in-place yaw for a set duration.
PATROL_LEG_M = float(os.environ.get("PATROL_LEG_M", "4.3"))
PATROL_TURN_S = float(os.environ.get("PATROL_TURN_S", "2.0"))
PATROL_TURN_WZ = float(os.environ.get("PATROL_TURN_WZ", "1.6"))

# Distance minimale à la caméra sous laquelle le robot ne descend jamais. Devant
# cette limite se trouve la zone morte (non passante) : trop proche, le robot
# déborderait du cadre et le sol n'y est pas visible. La patrouille est donc
# bornée entre PATROL_MIN_DISTANCE (côté caméra) et PATROL_MIN_DISTANCE +
# PATROL_LEG_M (côté éloigné). Voir la calibration pour choisir cette valeur :
# à hauteur caméra 1.5 m, le sol n'apparaît qu'à ~2.7 m, d'où ce défaut.
PATROL_MIN_DISTANCE = float(os.environ.get("PATROL_MIN_DISTANCE", "2.7"))


class AvoidBehaviour:
    """Autonomous patrol with reactive obstacle avoidance.

    The robot walks forward along a leg, turns about-face at the end, walks back,
    and repeats, patrolling between the near (camera) edge and the far edge. When
    an obstacle comes within influence range it brakes and steers around it with
    a potential field; the patrol resumes once the path is clear.

    No map, no memory: distance is integrated from the commanded speed, so a
    sustained obstacle push can stretch a leg. Accepted limit of a reactive demo.

    Frame: +x forward, +y left, wz yaw (+ turns left). Obstacle bearing is
    degrees, + to the robot's right.
    """

    WALK = "walk"
    TURN = "turn"

    def __init__(self) -> None:
        self.obstacles: list[dict] = []
        self.stamp = 0.0
        self._closest = _math.inf
        self._state = self.WALK
        self._leg_travelled = 0.0
        self._turn_elapsed = 0.0
        self._last_t = None
        # Distance courante à la caméra. Le robot démarre au bord de la zone
        # morte et s'éloigne. Le sens alterne à chaque demi-tour : +1 s'éloigne
        # de la caméra, -1 revient vers elle.
        self._camera_dist = PATROL_MIN_DISTANCE
        self._direction = 1.0
        self._turn_reason = ""  # why the last about-face happened (for logging)

    def observe(self, payload: dict) -> None:
        self.obstacles = payload.get("obstacles", payload.get("people", []))
        self.stamp = payload.get("stamp", 0.0)

    def _avoidance(self, now: float):
        if (now - self.stamp) > AVOID_STALE_S:
            return np.array([CRUISE_VX, 0.0, 0.0]), _math.inf
        push = np.zeros(2)
        closest = _math.inf
        for obs in self.obstacles:
            rng = obs.get("range_m", _math.inf)
            if rng is None or not _math.isfinite(rng):
                # No valid depth: skip it. We now have real deprojected depth for
                # detections, so a missing reading means we can't trust a distance.
                # Estimating from box size caused false positives (a large person
                # box far to the side read as a very close obstacle).
                continue
            if rng > INFLUENCE_M:
                continue
            closest = min(closest, rng)
            clamped = max(rng, DANGER_M)
            strength = (INFLUENCE_M - clamped) / (INFLUENCE_M - DANGER_M)
            strength = float(np.clip(strength, 0.0, 1.0)) ** 2
            bearing = _math.radians(obs.get("bearing_deg", 0.0))
            obs_dir = np.array([_math.cos(bearing), -_math.sin(bearing)])
            push -= obs_dir * strength
        if closest == _math.inf:
            return np.array([CRUISE_VX, 0.0, 0.0]), closest
        push *= REPULSION_GAIN
        wz = float(np.clip(STEER_GAIN * push[1], -1.0, 1.0))
        head_on = max(0.0, -push[0])
        vx = CRUISE_VX * float(np.clip(1.0 - FORWARD_BRAKE * head_on, 0.0, 1.0))
        return np.array([vx, 0.0, wz]), closest

    def command(self, now: float) -> np.ndarray:
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now

        if self._state == self.TURN:
            self._turn_elapsed += dt
            self._closest = _math.inf
            if self._turn_elapsed >= PATROL_TURN_S:
                self._state = self.WALK
                self._leg_travelled = 0.0
                # Un demi-tour a été effectué : le robot repart dans l'autre sens.
                self._direction = -self._direction
                return np.zeros(3)
            return np.array([0.0, 0.0, PATROL_TURN_WZ])

        cmd, closest = self._avoidance(now)
        self._closest = closest
        step = max(0.0, cmd[0]) * dt
        self._leg_travelled += step
        # La distance à la caméra évolue selon le sens de marche. Le robot avance
        # toujours devant lui (cmd[0] >= 0) ; c'est son orientation, encodée par
        # _direction, qui dit s'il s'éloigne ou se rapproche de la caméra.
        self._camera_dist += self._direction * step

        # Distance-based patrol bounds are disabled: the patrol ROI (handled in the
        # sim) now defines where the robot may go. The behaviour just walks and
        # avoids obstacles; the sim turns it back at the ROI edges.
        return cmd

    def status(self) -> dict:
        st = {"state": self._state, "leg_m": round(self._leg_travelled, 2),
              "camera_dist_m": round(self._camera_dist, 2)}
        st["avoiding"] = bool(_math.isfinite(self._closest))
        if st["avoiding"]:
            st["closest_m"] = round(self._closest, 2)
        st["turning"] = self._state == self.TURN
        st["turn_reason"] = getattr(self, "_turn_reason", "")
        return st
