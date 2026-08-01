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
OV_DEVICE = os.environ.get("OV_DEVICE", "GPU")


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
        # Initial heading in radians. Set START_YAW to face the robot away
        # from the camera (its back to us) at launch. pi = 180 deg.
        self.yaw = float(__import__('os').environ.get('START_YAW', '0.0'))
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

    def reset(self, data: mujoco.MjData) -> None:
        """Clear accumulated heading and gait phase for a clean restart."""
        self.phase = 0.0
        self.yaw = float(__import__("os").environ.get("START_YAW", "0.0"))
        self.cmd = np.zeros(3)

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
    """Velocity-tracking locomotion policy through OpenVINO.

    Wraps a policy exported from the LuckyRobots G1 challenge (see
    policies/g1_walker/PROVENANCE.md). The observation layout, joint order and
    action scaling below are taken from that policy's own runner, not guessed.
    Three things in here are load-bearing and each one, if wrong, produces a
    robot that falls over on the first step with no useful error:

      1. Observation field order and framing must match exactly. The policy
         wants base-frame linear and angular velocity, base-frame projected
         gravity, then joint pos/vel/last-action/command.
      2. Joint order is the policy's own and differs from the loaded model's,
         so everything is mapped by joint NAME, never by index.
      3. Observations are fed raw. Normalization is baked into the ONNX graph.
    """

    # The policy's PD gains and armature are tuned for 200 Hz physics. Running
    # the loop faster makes the same gains too stiff and the robot falls.
    physics_hz = 200.0

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        import json

        import openvino as ov

        meta_path = os.path.join(os.path.dirname(POLICY_PATH), "walker_meta.json")
        if not os.path.exists(POLICY_PATH) or not os.path.exists(meta_path):
            raise SystemExit(
                f"POLICY=rl needs {POLICY_PATH} and walker_meta.json beside it. "
                "Run 'make policy' to fetch them, or start with POLICY=kinematic."
            )

        meta = json.load(open(meta_path))
        self.joint_names: list[str] = meta["joint_names"]
        self.default_pos = np.asarray(meta["default_joint_pos"], dtype=np.float32)
        self.action_scales = np.asarray(meta["action_scales"], dtype=np.float32)
        self.n = len(self.joint_names)

        # Map the policy's joint order onto the loaded model, by name. If the
        # model is missing a joint the policy drives, that is fatal and worth
        # saying loudly rather than silently misaligning the vector.
        self.qpos_adr = np.empty(self.n, dtype=np.int32)
        self.qvel_adr = np.empty(self.n, dtype=np.int32)
        self.act_id = np.empty(self.n, dtype=np.int32)
        missing = []
        for i, name in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if jid < 0 or aid < 0:
                missing.append(name)
                continue
            self.qpos_adr[i] = model.jnt_qposadr[jid]
            self.qvel_adr[i] = model.jnt_dofadr[jid]
            self.act_id[i] = aid
        if missing:
            raise SystemExit(
                f"model is missing {len(missing)} joint(s) the policy drives: "
                f"{missing[:5]}{'...' if len(missing) > 5 else ''}. "
                "This policy targets a 29-DoF G1; check ROBOT matches."
            )

        self.model = model
        self.last_action = np.zeros(self.n, dtype=np.float32)
        self.target = self.default_pos.copy()

        driven = set(int(a) for a in self.act_id)
        undriven = [a for a in range(model.nu) if a not in driven]
        self._undriven = np.asarray(undriven, dtype=np.int32) if undriven else None

        self._apply_armature(model)

        core = ov.Core()
        log.info("compiling %s for %s", POLICY_PATH, OV_DEVICE)
        model_ov = core.read_model(POLICY_PATH)
        # Pin the batch dimension before compiling. The ONNX declares [?, 99],
        # and the NPU does not accept dynamic shapes: it fails to compile and the
        # fallback below quietly moves the policy to the CPU. Exactly one
        # observation is fed per control step, so the shape is [1, 99] and always
        # was; declaring it is what makes the NPU usable at all.
        try:
            shape = list(model_ov.input(0).partial_shape)
            if any(d.is_dynamic for d in shape):
                fixed = [1 if d.is_dynamic else d.get_length() for d in shape]
                model_ov.reshape({model_ov.input(0): ov.PartialShape(fixed)})
                log.info("input reshaped to %s so the NPU can take it", fixed)
        except Exception as exc:
            log.warning("could not fix the input shape (%s)", exc)

        try:
            compiled = core.compile_model(model_ov, OV_DEVICE)
        except Exception as exc:
            # Say WHY, rather than only that it happened: the difference between
            # a missing driver and an unsupported model matters here.
            # The useful part of an OpenVINO error is at the END of the chain:
            # the first line is only "Exception from core.cpp:117".
            detail = [ln.strip() for ln in str(exc).splitlines() if ln.strip()]
            log.warning("%s unavailable, falling back to CPU: %s", OV_DEVICE,
                        " | ".join(detail[-3:])[:300])
            try:
                log.warning("  devices OpenVINO can see here: %s",
                            ", ".join(core.available_devices) or "none")
            except Exception:
                pass
            compiled = core.compile_model(model_ov, "CPU")
        else:
            log.info("policy running on %s", OV_DEVICE)
        self.net = compiled
        self.out_port = compiled.output(0)

    # Rotor inertia the policy was trained with. Without it the joints respond
    # differently and the robot falls even holding a correct pose. Keyed by
    # joint name because the model's DoF order is not the policy's order.
    _ARMATURE = {
        "5020": 0.00360972,
        "7520_14": 0.01017752,
        "7520_22": 0.02510192,
        "4010": 0.00425000,
        "2x5020": 0.00721945,
    }

    def _apply_armature(self, model: mujoco.MjModel) -> None:
        for name in self.joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            dof = model.jnt_dofadr[jid]
            if "elbow" in name or "shoulder" in name or "wrist_roll" in name:
                val = self._ARMATURE["5020"]
            elif "hip_pitch" in name or "hip_yaw" in name or name == "waist_yaw_joint":
                val = self._ARMATURE["7520_14"]
            elif "hip_roll" in name or "knee" in name:
                val = self._ARMATURE["7520_22"]
            elif "wrist_pitch" in name or "wrist_yaw" in name:
                val = self._ARMATURE["4010"]
            elif "ankle" in name or name in ("waist_pitch_joint", "waist_roll_joint"):
                val = self._ARMATURE["2x5020"]
            else:
                val = self._ARMATURE["5020"]
            model.dof_armature[dof] = val

    def stand(self, data: mujoco.MjData) -> None:
        """Place the robot at its default standing pose. Call before stepping."""
        data.qpos[self.qpos_adr] = self.default_pos
        data.qpos[0:3] = [0.0, 0.0, 0.76]
        data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[:] = 0.0
        self.last_action[:] = 0.0
        self.target = self.default_pos.copy()
        mujoco.mj_forward(self.model, data)

    @staticmethod
    def _quat_rotate_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
        w = quat[0]
        xyz = quat[1:]
        t = np.cross(xyz, vec) * 2.0
        return vec - w * t + np.cross(xyz, t)

    def _observation(self, cmd: np.ndarray, data: mujoco.MjData) -> np.ndarray:
        quat = data.qpos[3:7].astype(np.float32)

        lin_vel = self._quat_rotate_inverse(quat, data.qvel[0:3].astype(np.float32))
        ang_vel = data.qvel[3:6].astype(np.float32)
        proj_gravity = self._quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0], np.float32))

        joint_pos = data.qpos[self.qpos_adr].astype(np.float32) - self.default_pos
        joint_vel = data.qvel[self.qvel_adr].astype(np.float32)

        obs = np.concatenate(
            [lin_vel, ang_vel, proj_gravity, joint_pos, joint_vel, self.last_action, cmd.astype(np.float32)]
        )
        return obs[None, :].astype(np.float32)

    def update(self, cmd: np.ndarray, data: mujoco.MjData) -> None:
        action = self.net(self._observation(cmd, data))[self.out_port].reshape(-1)
        self.last_action = action.astype(np.float32)
        self.target = self.default_pos + action * self.action_scales

    def apply(self, data: mujoco.MjData) -> None:
        data.ctrl[self.act_id] = self.target
        # Actuators the policy does not drive (hands) are held at zero so they
        # hang neutrally instead of flopping and perturbing balance.
        if self._undriven is not None:
            data.ctrl[self._undriven] = 0.0


def make_controller(model: mujoco.MjModel, data: mujoco.MjData):
    if POLICY == "kinematic":
        log.info("controller: kinematic gait (no physics on the joints)")
        return KinematicController(model, data)
    if POLICY == "rl":
        log.info("controller: RL locomotion policy")
        return RLController(model, data)
    raise SystemExit(f"unknown POLICY {POLICY!r}, expected 'kinematic' or 'rl'")
