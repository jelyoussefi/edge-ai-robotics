# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A simulated Unitree G1 humanoid (MuJoCo) composited in real time onto a live Intel RealSense D455 feed, so the robot appears to walk in the filmed room. One Panther Lake board, workload split across CPU (physics), iGPU (GLSL compositing) and NPU (YOLO11m-seg + the RL locomotion policy, both via OpenVINO). Deployment is Docker Compose, one container per concern.

`docs/reprise-edge-ai-robotics.md` and `docs/ROADMAP.md` are the living design docs (in French). Read them before any non-trivial change.

## Architecture

Transport is **ZeroMQ**, not ROS 2 — msgpack payloads, topic name as the first multipart frame.

- `services/bus/broker.py` is the **only** process that binds (XSUB `:5555` / XPUB `:5556`). Everyone else connects, so **startup order never matters**.
- `common/edgebot/topics.py` is the single source of truth for topic names and message schemas. Add a topic there before publishing it.
- `common/edgebot/bus.py` — `Publisher` / `Subscriber`. Use `drain()` ("latest wins") in control loops, `recv(timeout_ms)` otherwise. `SNDHWM`/`RCVHWM` are 256 deliberately; a small HWM silently froze camera frames.

Flow: `source → camera.rgb → perception → perception.{detections,mask} → compositor`; `source → camera.depth → compositor`; `compositor → patrol.roi → sim → robot.state → compositor`.

`services/groundfloor/` is the optional ROS 2 Jazzy bridge (compose profile `suite`) to Intel's Robotics AI Suite — reached only via `make groundfloor`.

## Commands

The Makefile is the entire developer interface. Do not invent `docker compose` invocations that bypass it.

```bash
make                      # == make run
make run                  # down → build → xhost +local:root → up -d → follow logs
make down / restart / build / ps
make logs S=sim           # follow one service
make shell S=perception   # exec a shell (defaults to compositor)
make calibrate HEIGHT=1.56    # HEIGHT is REQUIRED; errors out without it
make seg-test SEG_ARGS="--image /data/shot.png"   # exits 1 when nothing detected
make groundfloor              # builds/runs the ROS 2 suite profile
make suite-compare ARGS="--seconds 120"           # our footprints vs Intel's, IoU
make clean / distclean
```

- `run` depends on `down` on purpose: a leftover container still holds the RealSense and the next start fails on a busy V4L2 ioctl.
- `.make/` holds build stamps; `.env` is auto-created from `.env.example`.

## Verification — run all three before delivering

There is no test suite and no CI (the old `tests/` was deleted in `b19723a`). Validation is these three checkers plus actually running the demo:

```bash
python3 -m py_compile services/*/*.py common/edgebot/*.py
python3 scripts/check_compose.py docker-compose.yml   # duplicate YAML keys PyYAML accepts but Docker rejects
python3 scripts/check_names.py services/*/*.py common/edgebot/*.py   # names used but never defined
```

`make build` runs the last two; `py_compile` is not wired in — run it yourself. `/verify-stack` does all three.

## Code style

No ruff/black/mypy config exists; style is by convention, applied consistently:

- SPDX header on every file: `# Copyright (C) 2026 Intel Corporation` / `# SPDX-License-Identifier: Apache-2.0`.
- `from __future__ import annotations` at the top of every module.
- Line length ~88–90 (not 79), double quotes, hanging indent aligned to the open paren.
- **Comments and docstrings in English and they explain *why*, including what was tried and rejected.** The Dockerfiles and `navigator.py` read as engineering logs with measured numbers — match that.
- `logging.getLogger("<service>")`; `basicConfig(format="%(asctime)s %(name)s %(levelname)s %(message)s")` in `main()`.
- Services install SIGTERM/SIGINT handlers that flip a `running` flag, then close bus sockets on exit.
- `cv2` is imported *inside* functions in `common/edgebot/floor.py` — deliberate, the `sim` image has no OpenCV.

## Tuning constants

**`docker-compose.yml` is authoritative.** Every knob is `${NAME:-default}` there and read with `os.environ.get(NAME, default)` in the service. The Python fallback is a safety net only; when they disagree, compose wins. Update compose when changing a value, and keep the Python default in step rather than letting it drift further.

Load-bearing relationships:

- `OBSTACLE_STALE` (sim, 3.0) must exceed `ROI_PERIOD` (compositor, 1.0), or footprints expire between messages and the robot ping-pongs.
- `OBSTACLE_MARGIN` must be identical in `compositor` and `groundfloor` or the two footprint sets aren't comparable.
- `POLICY=rl` forces 200 Hz physics and per-joint armature (`controller.physics_hz` overrides `PHYSICS_HZ`); both silently drop the robot if wrong.

## Hardware and environment gotchas

- **Only `source` may open the RealSense.** It runs `privileged: true` with `/dev` mounted. Anything else needing frames reads them off the bus — that is why `make seg-test` goes through the bus and `make calibrate` depends on `down` and runs inside the source container.
- **The NPU needs `/dev/accel` mounted, not `/dev/dri`.** Without it `OV_DEVICE=NPU` silently falls back to CPU.
- `/dev/dri` plus `group_add: [RENDER_GID, VIDEO_GID]` for `compositor`, `sim`, `perception`; the GIDs are resolved on the host by the Makefile.
- Driver versions are pinned hard: NPU driver 1.35.0 with Level Zero loader exactly v1.28.2. A mismatched `libze1` breaks the NPU while leaving the GPU working — hard to diagnose, do not bump casually.
- Compositor pins `MUJOCO_GL=glx` (EGL fails with a gladLoadGL error here). X11 only; `make run` calls `xhost +local:root`.
- MuJoCo's depth buffer is inverted on this driver (background = 0.0), detected at startup.
- With GLFW, CPU-side annotation must go through `gpu.present_image()`.
- World frame: origin is the ground point under the camera, `+x` forward along the optical axis, `+y` left, `+z` up.

## Calibration

`config/camera_calibration.json` and `config/floor_mask.png` are gitignored **on purpose** — they encode one room and one camera position, and inheriting someone else's looks exactly like a code bug. Regenerate with `make calibrate HEIGHT=<metres>`. Do not press `MEASURE` in the calibration UI: RANSAC is unreliable in this room (19–25 % inliers).

## Working agreement

- **Commit before any local intervention.** Deliveries arrive as `edge-ai-robotics-<etape>.tar.gz` archives that are extracted over the tree and **overwrite without warning**; an uncommitted edit is simply lost.
- **Never edit the same file from the chat side and from Claude Code in parallel** — same reason.
- `models/`, `policies/`, `assets/`, `data/` are gitignored and fetched by `make` (several GB). `models/mujoco_menagerie/` is a vendored DeepMind checkout: its `pyproject.toml`, `.pre-commit-config.yaml` and `Makefile` are **not** project config.

## Known-stale docs

Do not trust these; the code is authoritative:

- `README.md` describes `viewer`/`teleop` services, `make teleop|policy|models|perception`, and a `tests/` suite — none of which exist.
- `docs/CALIBRATION.md` references `scripts/calibrate_camera.py`; the real script is `services/source/calibrate.py`, reached via `make calibrate`.
- `.env.example` defaults (`ROBOT=g1`, `POLICY=kinematic`) are overridden unconditionally by compose (`g1_walker`, `rl`); `SIM_CPUS` and `STREAMS` are no longer read anywhere.
