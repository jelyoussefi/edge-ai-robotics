# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A simulated Unitree G1 humanoid (MuJoCo) composited in real time onto a live Intel RealSense D455 feed, so the robot appears to walk in the filmed room. One Panther Lake board, workload split across CPU (physics), iGPU (GLSL compositing) and NPU (YOLO11m-seg + the RL locomotion policy, both via OpenVINO). Deployment is Docker Compose, one container per concern.

`docs/reprise-edge-ai-robotics.md` and `docs/ROADMAP.md` are the living design docs (in French). Read them before any non-trivial change.

## Where the étapes stand

- **A — camera tilt: closed.** 14.1°, floor from 1.5 m, +28 % usable band.
- **B — ROS 2 bridge and `groundfloor`: closed.** Criterion met with definitions neutralised: raw floor IoU 0.530, median boundary 0.164 m. Write-up `docs/ETAPE-B-RESULTS.md`.
- **C — ADBSCAN and FastMapping: closed, frozen.** Write-up `docs/ETAPE-C-RESULTS.md`. Substitution was **rejected by measurement**; the architecture is the **union** of our footprints and their clusters. Do not reopen the tuning without a reason — the residual cost and its untried leads are recorded in §8.
- **D — developer guide: written.** `docs/DEVELOPER-GUIDE.md`, in English, the one document meant for people outside this project. No white paper exists in the tree; if one is claimed as done, it is not here.
- **E — Nav2: deliberately deferred.** Not an addition, an architecture change.

**Three suite bricks are integrated**, all `profiles: ["suite"]`, all built from source at the same pinned `SUITE_COMMIT`: `groundfloor`, `adbscan`, `fastmapping`. `groundfloor` **owns the camera path** — depth, `camera_info`, TF — and the other two consume it and publish none of it.

`docs/reprise-edge-ai-robotics.md` lags the code on several points (it still describes B as in progress and lists resolved open items). **The code is authoritative**; see the known-stale list at the end.

## Architecture

Transport is **ZeroMQ**, not ROS 2 — msgpack payloads, topic name as the first multipart frame.

- `services/bus/broker.py` is the **only** process that binds (XSUB `:5555` / XPUB `:5556`). Everyone else connects, so **startup order never matters**.
- `common/edgebot/topics.py` is the single source of truth for topic names and message schemas. Add a topic there before publishing it.
- `common/edgebot/bus.py` — `Publisher` / `Subscriber`. Use `drain()` ("latest wins") in control loops, `recv(timeout_ms)` otherwise. `SNDHWM`/`RCVHWM` are 256 deliberately; a small HWM silently froze camera frames.

Flow: `source → camera.rgb → perception → perception.{detections,mask} → compositor`; `source → camera.depth → compositor`; `compositor → patrol.roi → sim → robot.state → compositor`.

`services/groundfloor/` is the optional ROS 2 Jazzy bridge (compose profile `suite`) to Intel's Robotics AI Suite — reached only via `make groundfloor`. `services/adbscan/` is the second brick, chained behind it. `services/fastmapping/` is the third, and the first **persistent** one: the other two recompute from scratch every frame, this one accumulates, which is why its bus payload is never aged out.

### QoS: three bricks, three different answers

Check the publisher and the subscriber separately, per topic, per direction. "The suite uses best-effort" is not a property of the suite, and the reflex learned on one brick is wrong for the next:

- **`groundfloor`** subscribes RELIABLE by default. Our depth publisher is BEST_EFFORT, as a camera driver would be, so DDS refused to match: *"no messages will be sent"*, then 30 s of silence. Fixed by passing `use_best_effort_qos:=True` at launch (`fed42a8`), not by weakening the bridge.
- **`adbscan`** needs nothing. `adbscan_sub` subscribes with `rclcpp::SensorDataQoS()` unconditionally, already BEST_EFFORT, already matching. The care went the *other* way: its `ObstacleArray` publisher is a plain `rclcpp::QoS(1)`, i.e. RELIABLE, so the bridge's subscription must be RELIABLE too.
- **`fastmapping`** cannot be told anything. It subscribes with a bare `rclcpp::QoS(10)` (`Subscribers.cpp:19`), RELIABLE, and exposes **no parameter**. Fixed on our side with `GF_DEPTH_RELIABLE=1`, because compatibility is one-directional: a RELIABLE publisher serves a BEST_EFFORT subscriber, so one reliable depth publisher feeds both bricks. Off by default; `make fastmapping` sets it. Verified by falsification — with it off, FastMapping receives **zero** images while `groundfloor` keeps running on the same topic.
- Its map publisher is `KeepLast(1) + transient_local + reliable`. A VOLATILE subscriber against a TRANSIENT_LOCAL publisher is the same silent non-match, one policy over.

### The shared ROS base

`docker/ros-base/` builds **`edge-ai-robotics-ros-base:jazzy`** — Ubuntu 24.04, ROS Jazzy `ros-base`, PCL/Eigen, the colcon toolchain, and `pyzmq`/`msgpack`. Both suite bricks are `FROM` it; **FastMapping will be the third consumer**, which is why it exists.

- One container per brick is still the rule. The base is shared *layers*, not a shared service — it has no `ENTRYPOINT` and is never run.
- Each brick's Dockerfile is only sparse checkout + `colcon build` + its own entrypoint. Anything shared belongs in the base; anything version-pinned to a component (`SUITE_COMMIT`) does not.
- **`COPY common /opt/edgebot` stays in the bricks, never in the base.** In the base it would mean one line changed in `common/edgebot/` invalidates both multi-minute colcon builds.
- `docker-compose.yml` maps each brick's `FROM` to `service:ros-base` via `build.additional_contexts`, so compose rebuilds a stale base *before* the brick. **Build through compose or `make`, never a bare `docker build`** — that would silently use whatever local image carries the tag.
- Entrypoints call `source /opt/ros-env.sh` from the base. It relaxes `set -u` around ROS's not-nounset-clean setup scripts and sources `/ws/install` when present; see the file for why the guard is an `if` and not `&&`.
- Measured gain: the two bricks went from ~4.8 GB apiece to 4.812 GB shared plus 5.8 MB / 17.9 MB unique — total image store 42.2 → 37.8 GB *while adding a third image*.

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
make ros-base                 # shared ROS Jazzy base image for the suite bricks
make groundfloor              # builds/runs the ROS 2 suite profile
make adbscan                  # ADBSCAN; brings groundfloor up with it, it is the input
make fastmapping              # FastMapping; same prerequisite, sets GF_DEPTH_RELIABLE=1
make suite-compare ARGS="--seconds 120"           # our footprints vs Intel's, IoU
make suite-compare ARGS="--seconds 60 --unmatched"   # + the unmatched map by location
make map-check ARGS="--seconds 20"                # what the accumulated map contains
make clean / distclean
```

- `run` depends on `down` on purpose: a leftover container still holds the RealSense and the next start fails on a busy V4L2 ioctl.
- `.make/` holds build stamps; `.env` is auto-created from `.env.example`.
- **`docker compose up -d <service>` does not rebuild.** It restarts the container with the existing image, so a source change you intend to *measure* needs `up -d --build`. This cost two measurement runs against code that no longer existed in the tree; the tell is `docker compose exec -T <svc> grep -c "<a new comment>" /app/<file>.py`.
- `make map-check` mounts `common/` as well as the script, because adding a bus topic edits `common/edgebot/topics.py`, which every service image bakes in, and that target rebuilds nothing.
- The `msg` macro expands into a shell single-quoted string: **no apostrophes** in a target's message, or the recipe line becomes an unterminated quote.

## Compositor overlays

Keys: `f` floor, `s` detections, `h` scale, `r` reset, `q` quit, `p` suite point cloud (off / over the video / cloud only), **`m` FastMapping's map on the floor plane (off / occupied / occupied plus free)**. Each has a matching env knob so a capture can be scripted with no keyboard: `SHOW_FLOOR`, `SHOW_CLOUD`, **`SHOW_MAP`**, plus `DIAG_FRAMES` (a **count** of annotated frames to write to `/data`, 0 by default).

`m` is its own key and not a fourth `p` state on purpose: `p` means "right now", the map means "everything seen so far", and the useful question is what one holds that the other does not — one cycle would make them mutually exclusive.

**The overlay rule: draw what the consumer sees, not the raw topic.** An overlay exists to explain what the robot acted on, so anything drawn goes through the same filtering the consumer applies:

- ADBSCAN's clusters are drawn in orange **only** when `OBSTACLE_SOURCE=union`, and only after the same arena clip and the same `SUITE_MAX_SPAN` rejection the navigator applies. Drawing the raw topic would put a rectangle over half the room that the robot never reacts to. A test asserts the compositor's filter returns exactly what the navigator's does on the same input — a picture that asserts an input the robot ignored is a worse failure than no picture.
- The map is bounded to x < 8 m, |y| < 4 m before projection. The grid is 20 m square and the room is 7; unbounded, the floor-plane projection piled 14 000 distant cells into a solid band across the frame.
- Everything ground-level goes through the **same** `_world_to_pixel` / `_cloud_to_pixels` at z = 0, never a second projection path, so a visible gap between two overlays is a real disagreement and not two ways of drawing a polygon.
- When an overlay is requested, `DIAG_FRAMES` waits for it. The floor overlay annotates from ~13 s after startup and the first occupancy grid arrives at ~15 s, so a scripted capture used to write three frames with every layer *except* the one asked for.

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
- `OBSTACLE_SOURCE` (`ours` default / `suite` / `union`) must be the same for `sim` and `compositor`, or the overlay claims an input the robot is ignoring. `SUITE_MAX_SPAN` and the `SUITE_*` arena bounds are read by both for the same reason.
- `GF_DEPTH_RELIABLE` belongs to the `fastmapping` profile, not to the demo. It is set by `make fastmapping` and is off everywhere else.

## Hardware and environment gotchas

- **Only `source` may open the RealSense.** It runs `privileged: true` with `/dev` mounted. Anything else needing frames reads them off the bus — that is why `make seg-test` goes through the bus and `make calibrate` depends on `down` and runs inside the source container.
- **The NPU needs `/dev/accel` mounted, not `/dev/dri`.** Without it `OV_DEVICE=NPU` silently falls back to CPU.
- **Model placement is a measurement, not a policy.** Timed on this board (Core Ultra X7 358H, Arc B390 iGPU, AI Boost NPU, OpenVINO 2026.2.0), inference only, warm:

  | | NPU | GPU | CPU |
  |---|---|---|---|
  | policy `walker.onnx` `[1,99]` | 0.135 ms (max **19.9**) | 0.135 ms | **0.034 ms** |
  | detector `yolo11m-seg` FP16 | **13.6 ms** | 9.7 ms | 306 ms |

  The **policy is 4× faster on the CPU** and its NPU worst case is 34× its own median — for a 99-input MLP the dispatch overhead has no arithmetic to amortise against. Everything still fits the 200 Hz control budget, so this is not a defect, but `OV_DEVICE=NPU` for the policy buys nothing measurable. The detector is the opposite case and earns its placement. **Inference is not the frame cost**: the service reports 29–48 ms/frame against 13.6 ms of inference, and `perception` holds 1356–1401 % CPU — offloading to the NPU does not make the service cheap.
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
- **`git push` after every commit.** `origin` is a private GitHub repo over SSH (`git@github.com:jelyoussefi/edge-ai-robotics.git`). A commit that only exists locally is still one archive extraction away from being lost, which is the same reason the rule above exists.
- **Never edit the same file from the chat side and from Claude Code in parallel** — same reason.
- `models/`, `policies/`, `assets/`, `data/` are gitignored and fetched by `make` (several GB). `models/mujoco_menagerie/` is a vendored DeepMind checkout: its `pyproject.toml`, `.pre-commit-config.yaml` and `Makefile` are **not** project config.

## Known-stale docs

Do not trust these; the code is authoritative:

- `README.md` describes `viewer`/`teleop` services, `make teleop|policy|models|perception`, and a `tests/` suite — none of which exist.
- `docs/CALIBRATION.md` references `scripts/calibrate_camera.py`; the real script is `services/source/calibrate.py`, reached via `make calibrate`.
- `.env.example` defaults (`ROBOT=g1`, `POLICY=kinematic`) are overridden unconditionally by compose (`g1_walker`, `rl`); `SIM_CPUS` and `STREAMS` are no longer read anywhere.
- **`docs/reprise-edge-ai-robotics.md` is behind the code**, despite `45fd989` being titled "reprise finale, A/B/C closés". That commit added six lines and changed one; it did not rewrite the document. Specifically:
  - "Étape B en cours" and the roadmap entry "B ... en cours" — B closed in `824f80f`, C in `a204d1e`.
  - It lists six services plus `groundfloor`; there are also `adbscan` and `fastmapping`.
  - Open items 3 (`RETURN_TO`) and 4 (the misplaced `navigator.py` comment) were **resolved and removed** in `6abffa6`, and `45fd989` put them back.
  - `STOP_AT=5.2` in Réglages. It is **6.0** in `docker-compose.yml` and `navigator.py`; `6abffa6` had already corrected this and `45fd989` reverted it.
  - NPU: "latence politique 0,48 ms, détection ~28 ms". Measured here: policy 0.135 ms on NPU / 0.034 ms on CPU, detector 13.6 ms of inference inside a 31–33 ms frame.
  - Keys: it lists `f s h r` only; `p` and `m` are missing.
  - It mentions a livre blanc as a project direction. No white paper exists in this tree.

  It is a chat-side artefact and the working agreement forbids editing it from both sides, so it is recorded here rather than fixed here.
