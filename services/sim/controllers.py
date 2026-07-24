# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Controller backends.

Two are provided on purpose.

`kinematic` drives the base pose directly and plays a gait cycle on the joints.
It cannot fall over and needs no trained weights, so the demo has something that
always works while the real policy is being brought up.

`rl` runs a trained velocity-tracking policy through OpenVINO and lets the
physics decide what happens. This is the one worth showing.
"""

from __future__ import annotations

import logging
import os

import mujoco
import numpy as np

log = logging.getLogger("controllers")

POLICY = os.environ.get("POLICY", "kinematic")
POLICY_PATH = os.environ.get("POLICY_PATH", "")
OV_DEVICE = os.environ.get("OV_DEVICE", "NPU")


class KinematicController:
    """Animates a gait and moves the base directly. Always stable.

    The legs swing in antiphase with an amplitude proportional to commanded
    speed, so standing still looks like standing still rather than marching.
    """

    STRIDE_HZ = 1.6

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.nu = model.nu
        self.home = data.qpos.copy()
        self.phase = 0.0
        self.cmd = np.zeros(3)
        self.yaw = 0.0
        self.joint_ids = self._swing_joints(model)

    @staticmethod
    def _swing_joints(model: mujoco.MjModel) -> dict[str, int]:
        """Locate hip pitch and knee joints by name, tolerating naming variation."""
        found: dict[str, int] = {}
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
            lowered = name.lower()
            for key, needle in (
                ("l_hip", "left_hip_pitch"),
                ("r_hip", "right_hip_pitch"),
                ("l_knee", "left_knee"),
                ("r_knee", "right_knee"),
            ):
                if needle in lowered and key not in found:
                    found[key] = model.jnt_qposadr[i]
        missing = {"l_hip", "r_hip", "l_knee", "r_knee"} - found.keys()
        if missing:
            log.warning("kinematic gait: joints not found %s, legs will not swing", sorted(missing))
        return found

    def update(self, cmd: np.ndarray, data: mujoco.MjData) -> None:
        self.cmd = cmd

    def apply(self, data: mujoco.MjData) -> None:
        dt = self.model.opt.timestep
        speed = float(np.linalg.norm(self.cmd[:2]))
        self.phase += 2 * np.pi * self.STRIDE_HZ * dt * min(1.0, speed / 0.5)

        # Integrate the base pose from the command.
        self.yaw += self.cmd[2] * dt
        heading = np.array([np.cos(self.yaw), np.sin(self.yaw)])
        lateral = np.array([-np.sin(self.yaw), np.cos(self.yaw)])
        data.qpos[0:2] += (heading * self.cmd[0] + lateral * self.cmd[1]) * dt
        data.qpos[2] = self.home[2] + 0.012 * np.sin(2 * self.phase) * min(1.0, speed / 0.5)
        data.qpos[3:7] = [np.cos(self.yaw / 2), 0.0, 0.0, np.sin(self.yaw / 2)]

        amp = 0.45 * min(1.0, speed / 0.5)
        swing = amp * np.sin(self.phase)
        for key, sign in (("l_hip", 1.0), ("r_hip", -1.0)):
            if key in self.joint_ids:
                data.qpos[self.joint_ids[key]] = self.home[self.joint_ids[key]] + sign * swing
        for key, sign in (("l_knee", 1.0), ("r_knee", -1.0)):
            if key in self.joint_ids:
                bend = amp * max(0.0, sign * np.sin(self.phase - 0.6))
                data.qpos[self.joint_ids[key]] = self.home[self.joint_ids[key]] - bend

        data.qvel[:] = 0.0


class RLController:
    """Velocity-tracking locomotion policy executed with OpenVINO.

    The observation layout below must match the one the policy was trained
    with, field for field and scale for scale. This is the single most common
    reason a downloaded checkpoint produces a robot that falls over instantly.
    Check it against the training config before blaming the physics.
    """

    ANG_VEL_SCALE = 0.25
    DOF_POS_SCALE = 1.0
    DOF_VEL_SCALE = 0.05
    ACTION_SCALE = 0.25

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        import openvino as ov

        if not POLICY_PATH or not os.path.exists(POLICY_PATH):
            raise SystemExit(
                f"POLICY=rl but no checkpoint at {POLICY_PATH!r}. "
                "Export one to policies/ or start with POLICY=kinematic."
            )

        self.model = model
        self.nu = model.nu
        self.default_q = data.qpos[7:].copy()
        self.last_action = np.zeros(self.nu, dtype=np.float32)
        self.target = self.default_q.copy()

        core = ov.Core()
        log.info("compiling %s for %s", POLICY_PATH, OV_DEVICE)
        try:
            self.net = core.compile_model(POLICY_PATH, OV_DEVICE)
        except Exception:
            log.warning("%s unavailable, falling back to CPU", OV_DEVICE)
            self.net = core.compile_model(POLICY_PATH, "CPU")
        self.out = self.net.output(0)

    def _observation(self, cmd: np.ndarray, data: mujoco.MjData) -> np.ndarray:
        quat = data.qpos[3:7]
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, quat)
        rot = rot.reshape(3, 3)

        base_ang_vel = rot.T @ data.qvel[3:6]
        projected_gravity = rot.T @ np.array([0.0, 0.0, -1.0])

        obs = np.concatenate(
            [
                base_ang_vel * self.ANG_VEL_SCALE,
                projected_gravity,
                cmd,
                (data.qpos[7:] - self.default_q) * self.DOF_POS_SCALE,
                data.qvel[6:] * self.DOF_VEL_SCALE,
                self.last_action,
            ]
        )
        return obs.astype(np.float32)[None, :]

    def update(self, cmd: np.ndarray, data: mujoco.MjData) -> None:
        action = self.net(self._observation(cmd, data))[self.out].reshape(-1)
        self.last_action = action
        self.target = self.default_q + action * self.ACTION_SCALE

    def apply(self, data: mujoco.MjData) -> None:
        data.ctrl[:] = self.target[: self.nu]


def make_controller(model: mujoco.MjModel, data: mujoco.MjData):
    if POLICY == "kinematic":
        log.info("controller: kinematic gait (no physics on the joints)")
        return KinematicController(model, data)
    if POLICY == "rl":
        log.info("controller: RL locomotion policy")
        return RLController(model, data)
    raise SystemExit(f"unknown POLICY {POLICY!r}, expected 'kinematic' or 'rl'")
