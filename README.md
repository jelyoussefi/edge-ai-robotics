<!--
Copyright (C) 2026 Intel Corporation
SPDX-License-Identifier: Apache-2.0
-->

# Edge AI Robotics

A single-board robotics demonstrator running on Intel Core Ultra Mobile Processor
(Series 3). One board carries perception, policy, physics and rendering at the
same time, with the workload split across CPU, GPU and NPU.

Milestone 1, in this repository, is a simulated humanoid driven from the
keyboard. Cameras and language control arrive in later milestones.

## Requirements

- Panther Lake board (Robinson Bay reference design or equivalent)
- Ubuntu 24.04, Docker Engine with the compose plugin
- A display for the viewer
- Intel GPU driver on the host (the viewer reaches `/dev/dri` directly)

## Quick start

```bash
git clone <this-repo> edge-ai-robotics
cd edge-ai-robotics
make build       # fetch robot models, build the four images
make run         # bring the stack up, humanoid appears on the display
```

Then, in a second terminal:

```bash
make teleop
```

| Key     | Action                 |
|---------|------------------------|
| `w` `s` | walk forward, backward |
| `a` `d` | turn left, right       |
| `q` `e` | strafe left, right     |
| space   | stop                   |
| `r`     | reset to start pose    |

`make help` lists everything else.

## Configuration

Override on the command line or in `.env`:

```bash
make run ROBOT=h1              # unitree_h1 instead of unitree_g1
make run POLICY=rl             # trained policy instead of the kinematic gait
make run SIM_CPUS=8-11         # pin physics to isolated cores
```

## Architecture

Four containers, one per concern, connected through a ZeroMQ broker. Adding
perception later means adding a publisher, not rewiring anything.

```
teleop  ──cmd.vel──▶  bus  ──cmd.vel──▶  sim  ──robot.state──▶  bus  ──▶  viewer
                                          │
                                          └── physics 1 kHz, control 50 Hz
```

| Service  | Does                                  | Uses            |
|----------|---------------------------------------|-----------------|
| `bus`    | XSUB/XPUB broker, the only binder     | nothing         |
| `sim`    | MuJoCo physics and the controller     | CPU, NPU for RL |
| `viewer` | draws the scene on the display        | iGPU via X11    |
| `teleop` | keyboard to velocity command          | a terminal      |

The simulator never renders and the viewer never steps physics. That separation
is what lets the physics loop be pinned to isolated cores without a frame drop
ever perturbing it.

## Controllers

`POLICY=kinematic` (default) drives the base pose directly and plays a gait
cycle on the legs. It cannot fall over and needs no trained weights. Use it to
get the stack running and to have something that always works on demo day.

`POLICY=rl` runs a trained velocity-tracking policy through OpenVINO on the NPU
and lets the physics decide the outcome. This is the version worth showing, and
the one that takes real work.

To use it, export a policy to `policies/g1_locomotion.xml` (OpenVINO IR) or
`.onnx`. The observation layout in `services/sim/controllers.py` must match the
one the policy was trained with, field for field and scale for scale. A mismatch
produces a robot that falls over immediately with no useful error message, and
it is by far the most common cause of a bad first run.

## Robot models

Models are fetched by `make models`, not committed. They are large, and each
one in MuJoCo Menagerie carries its own licence. Review the per-model `LICENSE`
before publishing renders or redistributing anything, particularly for a
vendor's robot appearing in Intel material.

## Roadmap

| Milestone | Scope                                                    |
|-----------|----------------------------------------------------------|
| M1        | Humanoid in simulation, keyboard control                 |
| M1.5      | RL locomotion policy on the NPU, real balance            |
| M2        | D457 perception, person detection, robot reacts to a human |
| M3        | Four cameras, fused point cloud rendered into the scene  |
| M4        | Language commands grounded against the live scene        |
| M5        | Telemetry overlay showing CPU, GPU and NPU concurrently  |

## Troubleshooting

**Viewer window does not appear.** The X display is not reachable. Run
`xhost +local:root` and check `DISPLAY` matches your session. On Wayland, start
the session under Xwayland or switch the viewer to headless streaming.

**Viewer is slow or renders in software.** The container is falling back to
llvmpipe. Check that `/dev/dri` exists on the host and that `RENDER_GID` in the
Makefile resolved to a real group id.

**Robot stands still.** Confirm teleop is publishing with
`make logs S=sim`, and that both containers resolve `bus`.

**Physics slower than real time.** Check `rtf` in the telemetry. Below 1.0 means
the box is not keeping up. Reduce `PHYSICS_HZ` or pin `SIM_CPUS`.

## Licence

Apache-2.0. Robot models and trained policies are covered by their own licences.
