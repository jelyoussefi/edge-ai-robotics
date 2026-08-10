#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""The speed at which this policy STARTS walking, which is not where it stops.

TURN_VX=0.26 is documented as "the speed at which the policy still walks", and
that is true in steady state and false from standstill. The two are different
thresholds and the gap between them is a hysteresis the navigator does not know
about: it slows to a floor that keeps a walking robot walking, the robot stops
anyway, and the command it then holds is too small to start it again. (0.26, 0)
becomes a stable fixed point and the patrol never resumes. Observed live at
lap 18: "walking back: toes at 1.21 m (limit 1.20), vx 0.26 m/s" repeated
forever with the robot stationary.

Measured against the POLICY, not against the running demo: this builds the same
model and the same controller and ramps the commanded speed itself, so the
answer does not depend on where the furniture is or on what the navigator
decided that second.

Two sweeps, because one only tells half of it:
  up    from a standstill, in steps, until the robot actually advances -> START
  down  from a walk, in steps, until it stops advancing               -> STOP
"""
from __future__ import annotations

import argparse
import os

import mujoco
import numpy as np

import controllers

SCENES = {
    "g1": "/models/mujoco_menagerie/unitree_g1/scene.xml",
    "g1_walker": "/models/g1_walker/scene.xml",
}
MOVING = 0.02       # metres advanced during the hold that counts as "walking"


def build():
    scene = SCENES.get(os.environ.get("ROBOT", "g1_walker"))
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)
    ctrl = controllers.make_controller(model, data)
    hz = getattr(ctrl, "physics_hz", None) or float(
        os.environ.get("PHYSICS_HZ", "200"))
    model.opt.timestep = 1.0 / hz
    return model, data, ctrl, hz


def settle(model, data, ctrl, hz, seconds=2.0):
    """Let the robot stand still before anything is asked of it."""
    cmd = np.zeros(3)
    for k in range(int(seconds * hz)):
        if k % max(1, int(hz / 50)) == 0:
            ctrl.update(cmd, data)
        ctrl.apply(data)
        mujoco.mj_step(model, data)


def hold(model, data, ctrl, hz, vx, seconds):
    """Command vx for `seconds` and return the metres actually advanced."""
    cmd = np.array([vx, 0.0, 0.0])
    x0, y0 = float(data.qpos[0]), float(data.qpos[1])
    steps = int(seconds * hz)
    every = max(1, int(hz / 50))
    for k in range(steps):
        if k % every == 0:
            ctrl.update(cmd, data)
        ctrl.apply(data)
        mujoco.mj_step(model, data)
        if data.qpos[2] < 0.3:                 # fallen: the answer is not here
            return float("nan")
    return float(np.hypot(data.qpos[0] - x0, data.qpos[1] - y0))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=0.02)
    ap.add_argument("--max", type=float, default=0.50)
    ap.add_argument("--hold", type=float, default=2.0,
                    help="seconds per rung; the policy needs time to commit")
    args = ap.parse_args()

    model, data, ctrl, hz = build()
    print(f"policy {os.environ.get('POLICY')}, physics {hz:.0f} Hz, "
          f"{args.hold:.1f} s per rung, {args.step:.02f} m/s steps, "
          f"a rung counts as walking above {MOVING:.03f} m advanced\n")

    settle(model, data, ctrl, hz)
    print("UP, from a standstill")
    print(f"{'vx cmd':>8}{'advanced':>11}{'mean speed':>12}   verdict")
    start = None
    vx = 0.0
    while vx <= args.max + 1e-9:
        d = hold(model, data, ctrl, hz, vx, args.hold)
        if d != d:                                    # NaN: it fell over
            print(f"{vx:>8.02f}{'fell':>11}")
            break
        walking = d > MOVING
        print(f"{vx:>8.02f}{d:>11.03f}{d / args.hold:>12.03f}   "
              f"{'walks' if walking else 'stationary'}")
        if walking and start is None:
            start = vx
            break
        vx += args.step

    print("\nDOWN, from a walk")
    stop = None
    if start is not None:
        vx = min(args.max, start + 0.20)
        hold(model, data, ctrl, hz, vx, args.hold)    # get it going properly
        print(f"{'vx cmd':>8}{'advanced':>11}{'mean speed':>12}   verdict")
        while vx >= 0.0:
            d = hold(model, data, ctrl, hz, vx, args.hold)
            if d != d:
                print(f"{vx:>8.02f}{'fell':>11}")
                break
            walking = d > MOVING
            print(f"{vx:>8.02f}{d:>11.03f}{d / args.hold:>12.03f}   "
                  f"{'walks' if walking else 'stationary'}")
            if not walking:
                stop = round(vx + args.step, 3)
                break
            vx -= args.step

    print()
    print(f"START threshold : {'unmeasured' if start is None else f'{start:.02f} m/s'}")
    print(f"STOP  threshold : {'unmeasured' if stop is None else f'{stop:.02f} m/s'}")
    if start is not None and stop is not None:
        print(f"hysteresis      : {start - stop:+.02f} m/s")
        print(f"\nA speed floor has to sit above the START threshold, not the "
              f"STOP one.\nAnything in [{stop:.02f}, {start:.02f}) keeps a "
              f"walking robot walking and\ncannot get a stopped one going "
              f"again.")


if __name__ == "__main__":
    main()
