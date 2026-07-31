# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Physics service.

Runs the MuJoCo model at a fixed rate, applies whatever controller was selected,
and publishes the resulting configuration. It does no rendering, so it can be
pinned to isolated cores without fighting the GPU for anything.
"""

from __future__ import annotations

import logging
import os
import random
import signal
import time

import mujoco
import numpy as np
from edgebot import topics
from edgebot.bus import Publisher, Subscriber

from behaviours import AvoidBehaviour
from controllers import make_controller

log = logging.getLogger("sim")

ROBOT = os.environ.get("ROBOT", "g1")
PHYSICS_HZ = float(os.environ.get("PHYSICS_HZ", "1000"))
CONTROL_HZ = float(os.environ.get("CONTROL_HZ", "50"))
PUBLISH_HZ = float(os.environ.get("PUBLISH_HZ", "60"))

SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "h1": "/models/mujoco_menagerie/unitree_h1/scene.xml",
    "t1": "/models/mujoco_menagerie/booster_t1/scene.xml",
    # 29-DoF G1 matching the RL walker policy, fetched by make policy.
    "g1_walker": "/models/g1_walker/scene.xml",
}

# Below this base height the robot has clearly gone over. Used only to report
# state and offer a reset, never to stop the physics.
FALLEN_HEIGHT = 0.4


def _point_in_poly(x: float, y: float, poly: list) -> bool:
    """Ray-casting point-in-polygon test for keep-out zones (world ground)."""
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


class Sim:
    def __init__(self) -> None:
        scene = SCENES.get(ROBOT)
        if scene is None:
            raise SystemExit(f"unknown robot {ROBOT!r}, expected one of {sorted(SCENES)}")
        if not os.path.exists(scene):
            raise SystemExit(
                f"{scene} is missing. Fetch the model first: "
                "run 'make build'")

        log.info("loading %s", scene)
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)

        self.controller = make_controller(self.model, self.data)

        # A controller may require a specific physics rate (the RL policy's
        # gains and armature are tuned for one) and its own standing pose. Honor
        # those before anything steps, falling back to the configured defaults.
        physics_hz = getattr(self.controller, "physics_hz", None) or PHYSICS_HZ
        self.model.opt.timestep = 1.0 / physics_hz
        self.physics_hz = physics_hz

        if hasattr(self.controller, "stand"):
            self.controller.stand(self.data)
        elif self.model.nkey:
            # Menagerie scenes ship a "home" keyframe with the robot standing.
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            mujoco.mj_forward(self.model, self.data)
        else:
            mujoco.mj_forward(self.model, self.data)

        self.behaviour = AvoidBehaviour()
        self.pub = Publisher()
        # Sim only listens to perception; the walk is decided here
        # autonomously and avoids obstacles. No manual/teleop mode.
        self.sub = Subscriber([topics.PERCEPTION_OBSTACLES, topics.CMD_RESET, topics.CMD_TURN,
                               topics.KEEPOUT_ZONES])
        self.cmd = np.zeros(3)
        self._manual_turn = 0.0  # pending operator yaw (radians), applied per frame
        self._keepout: list[list[tuple[float, float]]] = []  # world-ground polygons
        self._avoid_reason = ""  # current avoidance cause, for change-logging
        self._vx = 0.0           # rate-limited commands, for smooth heading changes
        self._wz = 0.0
        self._stopped = False
        self._last_walk_log = 0.0
        self._steer_reason = ""
        # Every position test below uses the front of the feet on the ground, not
        # the free joint. qpos[0:2] is the pelvis, 0.79 m up and roughly 0.2 m
        # behind the toes in mid-stride, so a boundary checked against it is a
        # boundary the feet have already crossed. The G1's four contact spheres
        # per foot sit at x = -0.05 (heel) and x = +0.12 (toe) in the ankle frame,
        # and MuJoCo gives their world position every step.
        self._foot_geoms = [
            g for g in range(self.model.ngeom)
            if "ankle_roll" in (mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY,
                int(self.model.geom_bodyid[g])) or "")]
        log.info("ground reference: %d foot contact geoms", len(self._foot_geoms))
        # True once the robot has reached the walkable floor for the first time.
        # Before that it simply walks straight out from under the camera; after
        # that it is steered back in if it ever leaves.
        self.running = True

        self._steps_per_control = max(1, int(physics_hz / CONTROL_HZ))
        self._steps_per_publish = max(1, int(physics_hz / PUBLISH_HZ))
        log.info("physics %.0f Hz, control %.0f Hz", physics_hz, CONTROL_HZ)

        signal.signal(signal.SIGTERM, self._stop)
        signal.signal(signal.SIGINT, self._stop)

    def _stop(self, *_: object) -> None:
        self.running = False

    def _poll_bus(self) -> None:
        """Consume perception messages, then compute the walk command."""
        while (msg := self.sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.PERCEPTION_OBSTACLES:
                payload = self._obstacles_to_robot_frame(payload)
                self.behaviour.observe(payload)
            elif topic == topics.CMD_RESET:
                # Compositor asked (via 'z') to send the robot back to its start
                # pose. Reset physics and the walk together.
                self._reset()
                self.behaviour = AvoidBehaviour()
            elif topic == topics.CMD_TURN:
                # Operator turn nudge (l/r keys). Accumulate a yaw offset in radians
                # that is applied on top of the walk command over a few frames.
                self._manual_turn += float(payload.get("wz", 0.0))
            elif topic == topics.KEEPOUT_ZONES:
                # Operator-drawn no-go polygons (world ground coords). The robot
                # steers away from these, covering obstacles the detector misses.
                self._keepout = [[tuple(p) for p in poly] for poly in payload.get("zones", [])]

        # Step 1 of the movement work: walk straight along the optical axis and
        # stop at STOP_AT. Everything else (patrol regions, wandering, about-face
        # rules) has been removed; it will be rebuilt one step at a time.
        raw = self._walk_the_axis()

        st = self.behaviour.status()
        if st.get("avoiding") and st.get("closest_m") is not None:
            if self._avoid_reason != "obstacle":
                self._avoid_reason = "obstacle"
                log.info("avoiding: obstacle at %.2fm ahead", st["closest_m"])
        elif self._avoid_reason == "obstacle":
            self._avoid_reason = ""

        raw = self._avoid_keepout(raw)

        self.cmd = np.array(
            [
                np.clip(raw[0], -topics.MAX_VX, topics.MAX_VX),
                np.clip(raw[1], -topics.MAX_VY, topics.MAX_VY),
                np.clip(raw[2], -topics.MAX_WZ, topics.MAX_WZ),
            ]
        )

    def _apply_manual_turn(self, dt: float) -> None:
        """Rotate the robot's heading by any pending operator turn, smoothly.

        Applied a slice per physics step so a 5 deg nudge sweeps over a fraction
        of a second instead of snapping, which keeps the walker balanced.
        """
        if abs(self._manual_turn) < 1e-4:
            return
        # Take a capped slice of the pending turn this step.
        rate = np.radians(120.0)  # deg/s sweep speed
        step = float(np.clip(self._manual_turn, -rate * dt, rate * dt))
        self._manual_turn -= step
        # Rotate the base quaternion about z by 'step'.
        qw, qx, qy, qz = (float(self.data.qpos[3]), float(self.data.qpos[4]),
                          float(self.data.qpos[5]), float(self.data.qpos[6]))
        h = step / 2.0
        cw, sw = np.cos(h), np.sin(h)
        # Quaternion multiply: q_turn (about z) * q_current.
        self.data.qpos[3] = cw * qw - sw * qz
        self.data.qpos[4] = cw * qx - sw * qy
        self.data.qpos[5] = cw * qy + sw * qx
        self.data.qpos[6] = cw * qz + sw * qw

    def _steer(self, reason: str, detail: str = "") -> None:
        """Log why the heading is being changed, once per change of reason."""
        if reason != self._steer_reason:
            self._steer_reason = reason
            log.info("heading change: %s%s", reason, f" ({detail})" if detail else "")

    def _yaw(self) -> float:
        qw, qz = float(self.data.qpos[3]), float(self.data.qpos[6])
        return 2.0 * float(np.arctan2(qz, qw))

    def _ground_ref(self) -> tuple[float, float]:
        """Ground position of the front of the feet: how far the robot has got.

        Among the foot contact geoms, the one furthest along the heading is the
        leading toe. It is the point that reaches a limit first, so distances are
        measured from it.

        It is NOT usable for lateral control: the leading toe alternates between
        the left and the right foot at every step, which injects a square wave of
        about +-0.15 m into any cross-track error built on it. Use _ground_centre
        for that.
        """
        if not self._foot_geoms:
            return float(self.data.qpos[0]), float(self.data.qpos[1])
        pts = self.data.geom_xpos[self._foot_geoms][:, :2]
        yaw = self._yaw()
        fwd = np.array([np.cos(yaw), np.sin(yaw)])
        lead = pts[int(np.argmax(pts @ fwd))]
        return float(lead[0]), float(lead[1])

    def _ground_centre(self) -> tuple[float, float]:
        """Ground position midway between the feet: steady across the gait."""
        if not self._foot_geoms:
            return float(self.data.qpos[0]), float(self.data.qpos[1])
        c = self.data.geom_xpos[self._foot_geoms][:, :2].mean(axis=0)
        return float(c[0]), float(c[1])

    def _obstacles_to_robot_frame(self, payload: dict) -> dict:
        """Re-express camera-measured obstacles relative to the robot.

        Perception reports range and bearing from the CAMERA, which sits at the
        world origin looking along +x. The avoidance behaviour reads them as if
        they were relative to the robot. With the robot several metres down the
        room, a person standing right in front of the lens reads as an obstacle
        a few centimetres ahead of it, and the robot brakes and turns away from
        something that is in fact well behind it.

        The camera sees an obstacle at world (r*cos b, -r*sin b), b positive to
        the right. Subtract the robot's position, rotate into its heading, and
        drop whatever ends up behind it.
        """
        obstacles = payload.get("obstacles") or payload.get("people") or []
        if not obstacles:
            return payload
        rx, ry = self._ground_ref()
        yaw = self._yaw()
        out = []
        for obs in obstacles:
            rng = obs.get("range_m")
            if rng is None or not np.isfinite(rng):
                out.append(obs)
                continue
            b = np.radians(float(obs.get("bearing_deg", 0.0)))
            ox, oy = rng * np.cos(b), -rng * np.sin(b)
            dx, dy = ox - rx, oy - ry
            b_rel = (np.arctan2(dy, dx) - yaw + np.pi) % (2 * np.pi) - np.pi
            if abs(np.degrees(b_rel)) > 100.0:
                continue           # behind the robot: not its problem
            o = dict(obs)
            o["range_m"] = float(np.hypot(dx, dy))
            o["bearing_deg"] = float(-np.degrees(b_rel))
            out.append(o)
        p2 = dict(payload)
        p2["obstacles"] = out
        p2.pop("people", None)
        return p2

    STOP_AT = float(os.environ.get("STOP_AT", "6.0"))   # m from the camera
    CRUISE_VX = 0.45      # m/s
    CROSS_GAIN = 2.0      # how hard to pull back onto the axis
    LOOKAHEAD = 1.0       # m over which the lateral error is nulled
    CROSS_MAX = 0.6       # rad, cap on the heading correction
    TURN_RATE = 0.9       # rad/s cap on the yaw command
    VX_SLEW = 1.2         # m/s^2 limit on the forward command
    WZ_SLEW = 2.5         # rad/s^2 limit on the yaw command

    def _walk_the_axis(self) -> np.ndarray:
        """Walk straight down the optical axis and stop at STOP_AT.

        The world origin is the point of floor directly below the camera and +x
        is the optical axis, so "stay on the axis" means hold y = 0. Heading
        alone is not enough for that: a small yaw bias integrates into a drift
        that never comes back. The lateral error is therefore nulled over
        LOOKAHEAD metres, which is what holds the line.

        Distance is measured from the front of the feet, the point that actually
        reaches STOP_AT first.
        """
        x, _ = self._ground_ref()       # leading toe: how far it has got
        _, y = self._ground_centre()    # midway between the feet: steady lateral
        yaw = self._yaw()
        dt = 1.0 / 50.0

        if x >= self.STOP_AT:
            if not self._stopped:
                self._stopped = True
                log.info("reached STOP_AT: toes at %.2f m on the axis, %+.3f m off it",
                         x, y)
            return self._smooth(0.0, 0.0, dt)

        # Hold y = 0: aim at a point LOOKAHEAD ahead on the axis.
        want = float(np.arctan2(-self.CROSS_GAIN * y, self.LOOKAHEAD))
        want = float(np.clip(want, -self.CROSS_MAX, self.CROSS_MAX))
        err = (want - yaw + np.pi) % (2 * np.pi) - np.pi
        # Walk at cruise all the way, then command zero: the rate limiter turns
        # that into a smooth stop over about a third of a second. Tapering the
        # command instead leaves the policy below the speed at which it actually
        # makes ground, so it shuffles on the spot and never arrives; that is why
        # the robot used to settle short of STOP_AT with a non-zero command.
        vx = self.CRUISE_VX * max(0.25, 1.0 - abs(err) / 1.2)
        if not self._stopped and time.time() - self._last_walk_log > 2.0:
            self._last_walk_log = time.time()
            log.info("walking the axis: toes at %.2f m of %.2f m, %+.3f m off the "
                     "axis, heading %+.1f deg, vx %.2f m/s",
                     x, self.STOP_AT, y, np.degrees(yaw), self._vx)
        return self._smooth(vx, float(np.clip(err * 1.2, -self.TURN_RATE,
                                              self.TURN_RATE)), dt)

    def _smooth(self, vx_des: float, wz_des: float, dt: float) -> np.ndarray:
        """Rate limit the command, so no heading or speed change is a step."""
        self._vx += float(np.clip(vx_des - self._vx, -self.VX_SLEW * dt,
                                  self.VX_SLEW * dt))
        self._wz += float(np.clip(wz_des - self._wz, -self.WZ_SLEW * dt,
                                  self.WZ_SLEW * dt))
        return np.array([self._vx, 0.0, self._wz])

    TURN_AWAY_WZ = 1.6  # rad/s turn when hitting a keep-out zone

    def _avoid_keepout(self, raw: np.ndarray) -> np.ndarray:
        """Stop and turn if the robot is in or approaching a keep-out zone."""
        if not self._keepout:
            return raw
        x, y = self._ground_ref()
        yaw = self._yaw()
        look = 0.6   # m ahead, enough to react before entering a keep-out zone
        ax = x + np.cos(yaw) * look
        ay = y + np.sin(yaw) * look
        for i, poly in enumerate(self._keepout):
            if _point_in_poly(x, y, poly) or _point_in_poly(ax, ay, poly):
                # Inside or about to enter: kill forward speed and turn away hard.
                if self._avoid_reason != f"zone{i}":
                    self._avoid_reason = f"zone{i}"
                    log.info("avoiding: keep-out zone %d at robot (%.1f, %.1f)", i, x, y)
                return np.array([-0.1, 0.0, self.TURN_AWAY_WZ])
        if self._avoid_reason and self._avoid_reason.startswith("zone"):
            self._avoid_reason = ""
        return raw

    def _reset(self) -> None:
        """Return the robot to its start pose without restarting the service."""
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
        # Back to the world origin, which is the floor directly under the camera,
        # facing away from it. The robot then walks out into the scene again.
        self._vx = self._wz = 0.0
        mujoco.mj_forward(self.model, self.data)
        # Reset the controller's internal state in place instead of recreating it,
        # which would recompile the policy (slow, and can stall the reset).
        if hasattr(self.controller, "reset"):
            self.controller.reset(self.data)
        if hasattr(self.controller, "stand"):
            self.controller.stand(self.data)
        log.info("reset to start pose")

    def _fallen(self) -> bool:
        return bool(self.data.qpos[2] < FALLEN_HEIGHT)

    def run(self) -> None:
        dt = self.model.opt.timestep
        step = 0
        next_deadline = time.perf_counter()
        jitter = np.zeros(int(PHYSICS_HZ), dtype=np.float64)
        policy_ms = 0.0
        last_report = time.perf_counter()
        steps_at_report = 0

        log.info("physics %.0f Hz, control %.0f Hz, publish %.0f Hz", PHYSICS_HZ, CONTROL_HZ, PUBLISH_HZ)

        while self.running:
            if step % self._steps_per_control == 0:
                self._poll_bus()
                t0 = time.perf_counter()
                self.controller.update(self.cmd, self.data)
                policy_ms = (time.perf_counter() - t0) * 1e3

            self.controller.apply(self.data)
            self._apply_manual_turn(dt)
            mujoco.mj_step(self.model, self.data)

            if step % self._steps_per_publish == 0:
                self.pub.send(
                    topics.ROBOT_STATE,
                    {
                        "t": float(self.data.time),
                        "qpos": self.data.qpos.tolist(),
                        "fallen": self._fallen(),
                    },
                )

            # Pace the loop against a wall-clock deadline rather than sleeping a
            # fixed amount, so a slow step is absorbed instead of accumulating.
            next_deadline += dt
            slack = next_deadline - time.perf_counter()
            jitter[step % len(jitter)] = -slack * 1e6
            if slack > 0:
                time.sleep(slack)
            else:
                next_deadline = time.perf_counter()

            step += 1

            now = time.perf_counter()
            if now - last_report >= 1.0:
                elapsed = now - last_report
                achieved_hz = (step - steps_at_report) / elapsed
                self.pub.send(
                    topics.SIM_TELEMETRY,
                    {
                        "physics_hz": float(achieved_hz),
                        # Real-time factor: 1.0 means simulated time keeps up
                        # with wall-clock time. Below 1.0 the demo is slow motion.
                        "rtf": float(achieved_hz * dt),
                        "jitter_p99_us": float(np.percentile(jitter[:min(step, len(jitter))], 99)),
                        "policy_ms": policy_ms,
                        **self.behaviour.status(),
                    },
                )
                last_report = now
                steps_at_report = step

        log.info("stopping")
        self.pub.close()
        self.sub.close()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    Sim().run()


if __name__ == "__main__":
    main()
