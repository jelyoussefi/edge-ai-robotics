<!--
Copyright (C) 2026 Intel Corporation
SPDX-License-Identifier: Apache-2.0
-->

# Edge AI Robotics

A single-board robotics demonstrator running on Intel Core Ultra Mobile Processor
(Series 3). One board carries perception, policy, physics and rendering at the
same time, with the workload split across CPU, GPU and NPU.

Milestone 2, in this repository, is a simulated humanoid that reacts to people
seen by real camera streams. It runs against a sample video out of the box, so
nothing waits on camera bring-up.

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

Press `m` to hand control to perception. The humanoid then turns to face the
nearest detected person and backs away if they come within 1.5 m. Press `m`
again to take the keyboard back.

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
make perception STREAMS=video  # perception alone, no physics, for tuning
```

### Perception streams

`config/streams.*.json` uses the same shape as the Robotics AI Suite
multicam-demo config, one entry per stream:

```json
{ "source": "/assets/videos/How_People_Walk.mp4",
  "model":  "/assets/models/yolov8n/FP16/yolov8n.xml",
  "device": "NPU", "confidence": 0.4, "vfov_deg": 50.0 }
```

`source` is anything OpenCV can open, so moving from the sample video to four
D457 cameras means changing four strings. `device` is passed straight to
OpenVINO, so the four streams can be spread across NPU, GPU and CPU to show
all three engines working at once. If a device is missing the detector logs a
warning and falls back to CPU rather than failing.

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
| `perception` | detects people on N streams       | NPU, GPU, CPU      |

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
| M1        | Humanoid in simulation, keyboard control (done)          |
| M2        | Person detection on N streams, robot reacts (done)       |
| M1.5      | RL locomotion policy on the NPU, real balance            |
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

**Range estimates look wrong.** `range_m` is monocular, derived from apparent
person height assuming someone standing and fully in frame. It is an estimate,
not a measurement. Set `vfov_deg` per stream to match the real lens, and expect
it to be wrong for seated or partly occluded people until M3 brings real depth.

**Physics slower than real time.** Check `rtf` in the telemetry. Below 1.0 means
the box is not keeping up. Reduce `PHYSICS_HZ` or pin `SIM_CPUS`.

## Tests

```bash
python3 tests/test_postprocess.py   # letterbox round trip, NMS, range estimate
python3 tests/test_behaviour.py     # gaze symmetry, retreat saturation, staleness
```

Both stub their dependencies, so they run without OpenVINO, MuJoCo or hardware.

## Licence

Apache-2.0. Robot models and trained policies are covered by their own licences.
The sample video is Apache-2.0 from the Intel Robotics AI Suite multicam-demo
component, fetched by `make assets` rather than vendored.
