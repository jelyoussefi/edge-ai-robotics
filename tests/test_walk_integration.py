# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Full physics check: the G1 stands, then walks forward under command.

The real proof that M1.5 works. Skips if MuJoCo/OpenVINO/model are absent.
Run `make policy` first.
"""
import os, sys
import numpy as np

def main() -> int:
    try:
        import mujoco, openvino  # noqa: F401
    except ImportError:
        print("SKIP: mujoco or openvino not installed")
        return 0
    scene = "models/g1_walker/scene.xml"
    if not os.path.exists(scene) or not os.path.exists("policies/g1_walker/walker.onnx"):
        print("SKIP: walker model/policy not fetched (run 'make policy')")
        return 0

    import mujoco
    sys.path.insert(0, "services/sim")
    os.environ.update(POLICY="rl",
                      POLICY_PATH=os.path.abspath("policies/g1_walker/walker.onnx"),
                      OV_DEVICE="CPU")
    import controllers

    model = mujoco.MjModel.from_xml_path(scene)
    model.opt.timestep = 1.0 / controllers.RLController.physics_hz
    data = mujoco.MjData(model)
    ctrl = controllers.RLController(model, data)
    ctrl.stand(data)

    spc = 4  # 200 Hz physics / 50 Hz control
    def run(cmd, secs):
        for step in range(int(secs * controllers.RLController.physics_hz)):
            if step % spc == 0:
                ctrl.update(np.asarray(cmd, np.float32), data)
            ctrl.apply(data)
            mujoco.mj_step(model, data)

    run([0, 0, 0], 1.0)                       # settle
    assert data.qpos[2] > 0.5, f"fell while standing: h={data.qpos[2]:.2f}"
    x0 = data.qpos[0]
    run([0.4, 0, 0], 2.0)                      # walk
    h, moved = data.qpos[2], data.qpos[0] - x0
    assert h > 0.5, f"fell while walking: h={h:.2f}"
    assert moved > 0.1, f"did not walk: moved={moved:.2f}m"
    print(f"PASS: stood (h>0.5), walked {moved:.2f}m forward, still up (h={h:.2f})")
    return 0

if __name__ == "__main__":
    sys.exit(main())
