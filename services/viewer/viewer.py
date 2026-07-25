# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Viewer service.

Loads the same model as the simulator but runs no physics. It only applies the
configuration arriving on the bus and draws it, which keeps rendering hiccups
out of the control loop entirely.
"""

from __future__ import annotations

import logging
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
from edgebot import topics
from edgebot.bus import Subscriber

log = logging.getLogger("viewer")

ROBOT = os.environ.get("ROBOT", "g1")
SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "h1": "/models/mujoco_menagerie/unitree_h1/scene.xml",
    "t1": "/models/mujoco_menagerie/booster_t1/scene.xml",
    # 29-DoF G1 matching the RL walker policy, fetched by make policy.
    "g1_walker": "/models/g1_walker/scene.xml",
}

# Camera follows the base with a fixed offset rather than tracking rigidly,
# which reads far better on a large screen.
CAM_DISTANCE = 3.5
CAM_ELEVATION = -12.0
CAM_SMOOTHING = 0.08


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    scene = SCENES[ROBOT]
    log.info("loading %s", scene)
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)

    sub = Subscriber([topics.ROBOT_STATE])
    look_at = np.zeros(3)
    stale_since: float | None = None

    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.distance = CAM_DISTANCE
        viewer.cam.elevation = CAM_ELEVATION
        viewer.cam.azimuth = 135.0

        log.info("viewer up on DISPLAY=%s", os.environ.get("DISPLAY"))

        while viewer.is_running():
            frame_start = time.perf_counter()

            msg = sub.drain()
            if msg is not None:
                _, payload = msg
                qpos = np.asarray(payload["qpos"], dtype=np.float64)
                if qpos.shape[0] == model.nq:
                    data.qpos[:] = qpos
                    mujoco.mj_forward(model, data)
                    stale_since = None
                else:
                    log.warning("qpos size %d != model nq %d, ignoring", qpos.shape[0], model.nq)
            elif stale_since is None:
                stale_since = frame_start
            elif frame_start - stale_since > 3.0:
                log.warning("no state from sim for 3 s")
                stale_since = frame_start

            look_at += (data.qpos[:3] - look_at) * CAM_SMOOTHING
            viewer.cam.lookat[:] = look_at

            viewer.sync()

            # Render at roughly 60 Hz. The simulator is free to run faster.
            time.sleep(max(0.0, (1 / 60) - (time.perf_counter() - frame_start)))

    log.info("viewer closed")
    sub.close()


if __name__ == "__main__":
    main()
