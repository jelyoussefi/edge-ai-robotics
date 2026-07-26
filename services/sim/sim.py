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


class Sim:
    def __init__(self) -> None:
        scene = SCENES.get(ROBOT)
        if scene is None:
            raise SystemExit(f"unknown robot {ROBOT!r}, expected one of {sorted(SCENES)}")

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
        self.sub = Subscriber([topics.CMD_VEL, topics.CMD_MODE, topics.PERCEPTION_OBSTACLES])
        self.teleop_cmd = np.zeros(3)
        self.cmd = np.zeros(3)
        self.mode = os.environ.get("START_MODE", topics.MODE_AUTO)
        self.running = True

        self._steps_per_control = max(1, int(physics_hz / CONTROL_HZ))
        self._steps_per_publish = max(1, int(physics_hz / PUBLISH_HZ))
        log.info("physics %.0f Hz, control %.0f Hz", physics_hz, CONTROL_HZ)

        signal.signal(signal.SIGTERM, self._stop)
        signal.signal(signal.SIGINT, self._stop)

    def _stop(self, *_: object) -> None:
        self.running = False

    def _poll_bus(self) -> None:
        """Consume every pending message, then decide which command to obey.

        All three topics share one subscriber, so this drains rather than
        conflates: a mode change must not be dropped just because a velocity
        command arrived after it.
        """
        while (msg := self.sub.recv(0)) is not None:
            topic, payload = msg
            if topic == topics.CMD_VEL:
                self.teleop_cmd = np.array(
                    [payload.get("vx", 0.0), payload.get("vy", 0.0), payload.get("wz", 0.0)]
                )
                if payload.get("reset"):
                    self._reset()
            elif topic == topics.CMD_MODE:
                new_mode = payload.get("mode", topics.MODE_MANUAL)
                if new_mode != self.mode:
                    log.info("mode -> %s", new_mode)
                    self.mode = new_mode
            elif topic == topics.PERCEPTION_OBSTACLES:
                self.behaviour.observe(payload)

        raw = self.behaviour.command(time.time()) if self.mode == topics.MODE_AUTO else self.teleop_cmd
        self.cmd = np.array(
            [
                np.clip(raw[0], -topics.MAX_VX, topics.MAX_VX),
                np.clip(raw[1], -topics.MAX_VY, topics.MAX_VY),
                np.clip(raw[2], -topics.MAX_WZ, topics.MAX_WZ),
            ]
        )

    def _reset(self) -> None:
        """Return the robot to its start pose without restarting the service."""
        if self.model.nkey:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.controller = make_controller(self.model, self.data)
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
                        "mode": self.mode,
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
