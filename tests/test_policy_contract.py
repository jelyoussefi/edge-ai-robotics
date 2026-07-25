# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Verify the walker policy loads and honors its observation contract.

Skips cleanly if OpenVINO or the fetched policy are absent, so it is safe to run
anywhere. Run `make policy` first for the full check.
"""
import json, os, sys
import numpy as np

POLICY = "policies/g1_walker/walker.onnx"
META = "policies/g1_walker/walker_meta.json"

def main() -> int:
    try:
        import openvino as ov
    except ImportError:
        print("SKIP: openvino not installed")
        return 0
    if not (os.path.exists(POLICY) and os.path.exists(META)):
        print("SKIP: policy not fetched (run 'make policy')")
        return 0

    meta = json.load(open(META))
    n = len(meta["joint_names"])
    assert n == 29, f"expected 29 joints, got {n}"
    default = np.asarray(meta["default_joint_pos"], np.float32)
    scales = np.asarray(meta["action_scales"], np.float32)

    core = ov.Core()
    net = core.compile_model(POLICY, "CPU")
    out = net.output(0)

    # Standing observation: everything zero, gravity down in base frame.
    obs = np.concatenate([
        np.zeros(3), np.zeros(3), np.array([0, 0, -1]),
        np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(3),
    ])[None].astype(np.float32)
    assert obs.shape[1] == 99, obs.shape

    a_stand = net(obs)[out].reshape(-1)
    assert a_stand.shape[0] == 29
    assert np.all(np.isfinite(a_stand)), "non-finite action"

    obs_walk = obs.copy(); obs_walk[0, -3:] = [0.5, 0, 0]
    a_walk = net(obs_walk)[out].reshape(-1)
    assert np.abs(a_walk - a_stand).max() > 1e-4, "command had no effect"

    target = default + a_stand * scales
    assert np.all(np.abs(target) < np.pi), "targets outside plausible joint range"
    print(f"PASS: 99->29, finite, command-sensitive, targets sane (n={n})")
    return 0

if __name__ == "__main__":
    sys.exit(main())
