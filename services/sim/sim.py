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
import signal
import time

import mujoco
import numpy as np
from edgebot import topics
from edgebot.floor import signed_area
from navigator import Navigator, Pose
from edgebot.bus import Publisher, Subscriber

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

        if hasattr(self.controller, "stand"):
            self.controller.stand(self.data)
        elif self.model.nkey:
            # Menagerie scenes ship a "home" keyframe with the robot standing.
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            mujoco.mj_forward(self.model, self.data)
        else:
            mujoco.mj_forward(self.model, self.data)

        self.pub = Publisher()
        # Sim only listens to perception; the walk is decided here
        # autonomously and avoids obstacles. No manual/teleop mode.
        # Obstacles arrive as world footprints on PATROL_ROI, already projected
        # through the calibrated floor plane and grown by the margin. The sim
        # used to also parse raw bearings from PERCEPTION_OBSTACLES and write
        # the same list, so whichever message came last won.
        # SUITE_CLUSTERS is subscribed unconditionally even though the default
        # OBSTACLE_SOURCE ignores it. The alternative is a subscription that
        # depends on an env var read in another module, and a silent topic is
        # free -- the suite profile usually is not running at all.
        self.sub = Subscriber([topics.CMD_RESET, topics.PATROL_ROI,
                               topics.SUITE_CLUSTERS])
        self.cmd = np.zeros(3)
        # Everything about where to go lives here, and knows nothing of MuJoCo.
        # Driving a real G1 means replacing this file, not this object.
        self.nav = Navigator()
        log.info("obstacle source: %s", self.nav.SOURCE)
                                         # compositor, which are the good ones
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

    def _pose(self) -> Pose:
        """Read the simulated robot into the pose the navigator expects.

        This is the whole of the coupling between the two. A real G1 would fill
        the same four fields from its odometry, and nothing else would change.
        """
        qw, qz = float(self.data.qpos[3]), float(self.data.qpos[6])
        yaw = 2.0 * float(np.arctan2(qz, qw))
        return Pose(lead=self._ground_ref(), centre=self._ground_centre(),
                    yaw=yaw, yaw_rate=float(self.data.qvel[5]))

    def _poll_bus(self) -> None:
        """Consume perception messages, then ask the navigator for a command."""
        while (msg := self.sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.PATROL_ROI:
                # Footprints arrive already projected onto the calibrated floor
                # plane and already grown by the margin. Preferred over the raw
                # detections: the compositor derives them from where each object
                # meets the floor, which the plane locates exactly, while a
                # depth reading is poor on thin legs and shiny surfaces.
                roi = [tuple(map(float, q)) for q in (payload.get("roi") or [])]
                if roi and signed_area(roi) < 0:
                    # Counter-clockwise, so the interior is on the left of the
                    # direction of travel and the boundary on the right.
                    roi = roi[::-1]
                self.nav.set_floor(roi, payload.get("blocked"))
                continue

            if topic == topics.SUITE_CLUSTERS:
                # Same world rectangles as PATROL_ROI's `blocked` and with the
                # same margin already applied, so they need no conversion. The
                # navigator drops them unless OBSTACLE_SOURCE asks for them.
                self.nav.set_suite(payload.get("blocked"))
                continue

            if topic == topics.CMD_RESET:
                self._reset()

        raw = self.nav.step(self._pose(), 1.0 / CONTROL_HZ)

        self.cmd = np.array(
            [
                np.clip(raw[0], -topics.MAX_VX, topics.MAX_VX),
                np.clip(raw[1], -topics.MAX_VY, topics.MAX_VY),
                np.clip(raw[2], -topics.MAX_WZ, topics.MAX_WZ),
            ]
        )



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






    # About-face. Measured on this policy: at 0.89 rad/s it delivers 0.83, but
    # at 1.20 the actual yaw collapses to 0.04 and even reverses. That is not
    # saturation, it is falling off the end of the training distribution, and
    # asking for more made the turn three times SLOWER (10.5 s against 3.5 s).
    # So the command is capped at what the policy can actually track. Forward
    # speed helps because a biped turns by stepping, but it must stay small or
    # the about-face sweeps the robot metres off the line.
    # TURN_VX matters more than TURN_WZ. Measured on this policy, yaw collapses
    # from 0.74 to 0.03 rad/s at a CONSTANT 0.9 command as the robot slows: it
    # turns by stepping, and with almost no forward speed it plants both feet and
    # stops rotating. Walking a proper arc keeps it in the regime it was trained
    # for. The cost is that the about-face sweeps some ground sideways.
    # The arc radius is TURN_VX divided by the yaw actually delivered, so the
    # only safe way to tighten it is less speed OR more yaw. Less speed stalls
    # the policy, so TURN_VX stays at the value where the feet keep stepping and
    # the yaw command is what gets raised. 0.9 is the most this policy tracks:
    # beyond that it collapses, which the yaw-tracking line will report.
    # Distance over which the lateral error is nulled. This is the OUTER loop,
    # and it has to be slow compared with the inner heading loop or the two
    # fight each other. At 1.0 m and 0.6 m/s the outer loop was under two
    # seconds, barely slower than the policy's own response, and the pair
    # limit-cycled: 3.6 deg of heading swing holding a lane, 7 deg recovering
    # from a turn. At 2.0 m those become 0.8 and 2.5 deg.
    # Cap on the heading used to rejoin the axis. At 0.6 rad the robot walks
    # back at 34 degrees, which looks like it has lost its way rather than like
    # a correction. Lower, it takes longer to converge but always looks like it
    # is walking down the line.
    # Half the turn diameter: the lane offset that makes each about-face land on
    # the opposite lane. TURN_VX / delivered yaw, and this policy delivers about
    # 0.66 rad/s of the 0.9 asked.
    # A wide obstacle needs a wide detour: r=0.6 with 0.45 m of clearance and
    # the 1.25 safety factor already asks for 1.31 m. Capping below that made
    # the robot brush past without saying so.
    # Fraction of the look-ahead by which the shift must be complete: 0.6 means
    # fully clear of the line 60% of the way in, so the robot is already past
    # the obstacle laterally before it gets near it.
    # Time constant of the filter on the heading and lateral position used for
    # control. Must exceed one stride or the controller chases the gait's own
    # sway; too long and it is slow to notice a real drift.











    def _reset(self) -> None:
        """Return the robot to its start pose without restarting the service."""
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
        # Back to the world origin, which is the floor directly under the camera,
        # facing away from it. The robot then walks out into the scene again.
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
