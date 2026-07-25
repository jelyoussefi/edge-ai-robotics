<!--
Copyright (C) 2026 Intel Corporation
SPDX-License-Identifier: Apache-2.0
-->

# Edge AI Robotics

A single-board robotics demonstrator running on Intel Core Ultra Mobile Processor
(Series 3). One board carries perception, policy, physics and rendering at the
same time, with the workload split across CPU, GPU and NPU.

Milestone 3, in this repository, is a simulated humanoid that walks forward and
steers around real obstacles measured by a D457 depth camera. Distance comes
from the camera's aligned depth stream, not an estimate. It still runs on a
sample video when no camera is attached, with distance reported as unknown.

## Requirements

- Panther Lake board (Robinson Bay reference design or equivalent)
- Ubuntu 24.04, Docker Engine with the compose plugin
- A display for the viewer
- Intel GPU driver on the host (the viewer reaches `/dev/dri` directly)

## Quick start

```bash
git clone <this-repo> edge-ai-robotics
cd edge-ai-robotics
make build       # fetch models and assets, build the five images
make run         # humanoid appears on the display, perception starts on video
```

Then, in a second terminal:

```bash
make teleop
```

Press `m` to hand control to perception. The humanoid then cruises forward and
steers around whatever the camera sees, pushing away from the nearest obstacles
and slowing when something is dead ahead. Press `m` again to take the keyboard
back.

| Key     | Action                 |
|---------|------------------------|
| `w` `s` | walk forward, backward |
| `a` `d` | turn left, right       |
| `q` `e` | strafe left, right     |
| space   | stop                   |
| `r`     | reset to start pose    |
| `m`     | toggle manual / auto   |

`make help` lists everything else.

## Configuration

Override on the command line or in `.env`:

```bash
make run ROBOT=h1              # unitree_h1 instead of unitree_g1
make run POLICY=rl             # trained policy instead of the kinematic gait
make run SIM_CPUS=8-11         # pin physics to isolated cores
make run STREAMS=single        # one detector instead of four, for bring-up
make run STREAMS=d457          # four real cameras instead of the sample video
make run POLICY=rl ROBOT=g1_walker  # real balancing and walking
make perception STREAMS=video  # perception alone, no physics, for tuning
```

### Perception streams

`config/streams.*.json` selects the source. A RealSense entry opens the D457
over pyrealsense2 with aligned depth:

```json
{ "type": "realsense", "serial": null, "width": 848, "height": 480, "fps": 30,
  "model": "/assets/models/yolo11n/FP16/yolo11n.xml", "device": "NPU" }
```

A video entry opens a file with no depth, for development:

```json
{ "source": "/assets/videos/How_People_Walk.mp4", "loop": true,
  "model": "/assets/models/yolo11n/FP16/yolo11n.xml", "device": "CPU" }
```

`serial` null takes the first camera found. `device` goes straight to OpenVINO
(NPU, GPU or CPU); a missing device logs a warning and falls back to CPU. The
detector reads a median depth from the centre of each box, so distance is a real
measurement when depth is present and reported as unknown otherwise.

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
| `bus`        | XSUB/XPUB broker, the only binder | nothing            |
| `sim`        | MuJoCo physics, controller, behaviour | CPU, NPU for RL |
| `viewer`     | draws the scene on the display    | iGPU via X11       |
| `teleop`     | keyboard to velocity command      | a terminal         |
| `perception` | detects and ranges obstacles      | NPU/GPU/CPU, D457  |

The simulator never renders and the viewer never steps physics. That separation
is what lets the physics loop be pinned to isolated cores without a frame drop
ever perturbing it.

## Controllers

`POLICY=kinematic` (default) drives the base pose directly and plays a gait
cycle on the legs. It cannot fall over and needs no trained weights. Use it to
get the stack running and to have something that always works on demo day.

`POLICY=rl` runs a trained velocity-tracking policy through OpenVINO and lets
the physics decide the outcome. The robot genuinely balances: bad commands make
it stumble, and it falls over if it loses its footing. This is the version worth
showing.

```bash
make policy                        # fetch the walker policy and its G1 model
make run POLICY=rl ROBOT=g1_walker # walk for real
make teleop                        # W/S/A/D to drive it
```

The policy is the G1 walker from the LuckyRobots challenge (see
`policies/g1_walker/PROVENANCE.md`). Its observation layout, joint order, action
scaling, armature and 200 Hz control rate are all taken from that policy's own
runner and verified, not guessed. `RLController` maps the policy's joints onto
the loaded model by name, so it stays correct even though the model's DoF order
differs from the policy's.

Two things in that path are load-bearing, and each one silently drops the robot
if wrong: the physics runs at 200 Hz because the PD gains are tuned for it, and
per-joint armature (rotor inertia) is applied because the policy was trained
with it. Both are handled automatically when `POLICY=rl`. If you bring your own
policy, `tests/test_policy_contract.py` and `tests/test_walk_integration.py`
check the contract and that the robot actually walks.

## Robot models

Models are fetched by `make models`, not committed. They are large, and each
one in MuJoCo Menagerie carries its own licence. Review the per-model `LICENSE`
before publishing renders or redistributing anything, particularly for a
vendor's robot appearing in Intel material.

## Roadmap

| Milestone | Scope                                                    |
|-----------|----------------------------------------------------------|
| M1        | Humanoid in simulation, keyboard control (done)          |
| M1.5      | RL locomotion policy, real balance and walking (done)    |
| M2        | Person detection on N streams, robot reacts (done)       |
| M3        | Real depth from D457, fused point cloud in the scene     |
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

**Robot ignores people in auto mode.** Check `make logs S=perception` shows a
non-zero person count. If detections are arriving but the robot does not move,
the mode never switched: `sim` logs `mode -> auto` when it does.

**Perception falls back to CPU.** The log line names the device it got. For NPU
check the driver is installed and `/dev/accel` exists on the host; for GPU check
`/dev/dri` and that `RENDER_GID` resolved.

**Distances look wrong.** `range_m` is now the median depth over the centre of
each box, in metres, from the D457 aligned depth stream. If it reads unknown for
everything, depth is not arriving: check the camera is in a mode that streams
depth and that the container sees it (`make logs S=perception` shows `/D` per
stream when depth is present). On a video source distance is always unknown by
design.

**Robot drives into a symmetric obstacle.** Expected. The avoider is a
potential field with no map, so a symmetric wall cancels the sideways push and
traps it in a local minimum. It brakes rather than colliding hard. Real path
planning around this is a later milestone.

**Physics slower than real time.** Check `rtf` in the telemetry. Below 1.0 means
the box is not keeping up. Reduce `PHYSICS_HZ` or pin `SIM_CPUS`.

## Tests

```bash
python3 tests/test_postprocess.py   # YOLO decode, multi-class NMS, bearing, depth
python3 tests/test_depth.py         # depth sampling: central patch, median, invalids
python3 tests/test_avoid.py         # avoidance: steer, brake, saturate, staleness
python3 tests/test_behaviour.py     # M2 reactive behaviour (still present)
```

All four stub their dependencies, so they run without OpenVINO, MuJoCo, a camera
or a display.

## Licence

Apache-2.0. Robot models and trained policies are covered by their own licences.
The sample video is Apache-2.0 from the Intel Robotics AI Suite multicam-demo
component, fetched by `make assets` rather than vendored.
