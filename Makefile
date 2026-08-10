# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# The demo has exactly one configuration: a RealSense D455, a Unitree G1 driven
# by the RL walking policy, and GPU composition onto the live camera. That is
# stated in docker-compose.yml rather than passed on the command line, so
# `make run` is the whole interface.

SHELL   := /bin/bash
COMPOSE ?= docker compose
DISPLAY ?= :0

# The render group id differs per distribution; the compositor needs it to reach
# /dev/dri on the host. Resolved here so compose does not have to guess.
RENDER_GID := $(shell getent group render 2>/dev/null | cut -d: -f3)
VIDEO_GID  := $(shell getent group video  2>/dev/null | cut -d: -f3)
export DISPLAY RENDER_GID VIDEO_GID

# Stamp files: fetch and build re-run only when their inputs change, never just
# because make was invoked again.
STAMP_DIR    := .make
IMAGES_STAMP := $(STAMP_DIR)/images
ASSETS_STAMP := $(STAMP_DIR)/assets
POLICY_STAMP := $(STAMP_DIR)/policy
ROSBASE_STAMP := $(STAMP_DIR)/ros-base

DOCKERFILES := $(shell find services -name Dockerfile 2>/dev/null)
SRC_FILES   := $(shell find services common -type f -name '*.py' 2>/dev/null)
REQ_FILES   := $(shell find services -type f -name 'requirements.txt' 2>/dev/null)

.PHONY: default help build run full down restart calibrate seg-test ros-base groundfloor adbscan fastmapping itsplanner grid-probe suite-compare map-check lane-probe pick-goals bus-rate web-latency logs ps shell clean distclean
default: run

help:
	@$(call msg, Edge AI Robotics)
	@echo "  make run        Stop anything running, build if needed, start the demo"
	@echo "  make down       Stop the stack"
	@echo "  make restart    Same as run"
	@echo "  make calibrate  Camera pose, with the floor shown in red   HEIGHT=1.50"
	@echo "                  SHOW_OBJECTS=1 make  draws labels, boxes and masks"
	@echo "                  CALIB_YOLO_MASK=0 for the plain height threshold"
	@echo "                  CALIB_YOLO_CONF=0.10 CALIB_YOLO_FRAMES=15 tune the assist"
	@echo "  make seg-test   What the segmentation model sees, as an image"
	@echo "  make ros-base       Shared ROS Jazzy base image for the suite bricks"
	@echo "  make groundfloor    Intel Robotics AI Suite floor segmentation"
	@echo "  make adbscan        Intel Robotics AI Suite ADBSCAN clustering"
	@echo "  make fastmapping    Intel Robotics AI Suite persistent occupancy map"
	@echo "  make itsplanner     Intel Robotics AI Suite ITS global path planner"
	@echo "  make full           The demo AND all four suite bricks, union source"
	@echo "  make suite-compare  Their floor against ours, side by side"
	@echo "  make lane-probe     Where the robot can walk in this room, as a map"
	@echo "                      Needs the demo already running in another terminal"
	@echo "  make bus-rate       What the bus carries, per topic     [ARGS=--seconds 25]"
	@echo "  make web-latency    Console stream latency, compositor to socket"
	@echo "  make logs       Follow the logs                            [S=compositor]"
	@echo "  make ps         Container status"
	@echo "  make shell      Shell inside a service                     [S=sim]"
	@echo "  make clean      Remove containers and build stamps"
	@echo "  make distclean  Also remove images and fetched assets"
	@echo ""
	@echo "  In the compositor window:  f  floor   s  detections   h  scale   r  reset   q  quit"
	@echo "                             f also draws the suite's ground in cyan when groundfloor runs"
	@echo "                             p  suite point cloud: off / over the video / cloud only"
	@echo "                             m  FastMapping map on the floor: off / occupied / plus free"

$(ASSETS_STAMP): scripts/fetch_assets.sh
	@$(call msg, Fetching perception assets ...)
	@mkdir -p $(STAMP_DIR)
	@bash scripts/fetch_assets.sh
	@touch $@

$(POLICY_STAMP): scripts/fetch_policy.sh services/sim/g1_walker_scene.xml
	@$(call msg, Fetching the G1 walker model and policy ...)
	@mkdir -p $(STAMP_DIR)
	@bash scripts/fetch_policy.sh
	@touch $@

.env:
	@cp .env.example .env

# Shared ROS Jazzy base for the suite bricks, tagged edge-ai-robotics-ros-base:
# jazzy. Building it by hand is rarely necessary -- compose names it as a build
# context for both bricks, so `make groundfloor` and `make adbscan` rebuild it
# first when it is stale. The target exists to build or inspect it on its own,
# and to make the dependency visible in the interface rather than only in YAML.
$(ROSBASE_STAMP): docker/ros-base/Dockerfile docker/ros-base/ros-env.sh
	@$(call msg, Building the shared ROS Jazzy base image ...)
	@mkdir -p $(STAMP_DIR)
	@$(COMPOSE) --profile suite build ros-base
	@touch $@

ros-base: $(ROSBASE_STAMP)

adbscan:
	@$(call msg, Starting the Robotics AI Suite ADBSCAN clustering ...)
	@# Brings groundfloor up with it, and not for tidiness: ADBSCAN clusters
	@# /segmentation/obstacle_points, which is the groundfloor node's output.
	@# Its own ground removal is a single height threshold and cannot handle a
	@# pitched camera. compose's depends_on enforces the order.
	@$(COMPOSE) --profile suite up --build adbscan groundfloor

fastmapping:
	@$(call msg, Starting the Robotics AI Suite FastMapping ...)
	@# Same prerequisite as adbscan, and a stronger one: since the etape C
	@# rewire the groundfloor container OWNS the camera path -- it publishes
	@# the depth image, the camera_info and the base_link TF -- and this brick
	@# consumes all three while publishing none of them.
	@# GF_DEPTH_RELIABLE=1 is not optional here. FastMapping subscribes with a
	@# bare rclcpp::QoS(10), i.e. RELIABLE, and exposes no parameter to change
	@# it; against our best-effort depth publisher DDS simply never matches and
	@# it receives nothing, silently. A reliable publisher feeds both bricks.
	@GF_DEPTH_RELIABLE=1 $(COMPOSE) --profile suite up --build fastmapping groundfloor

itsplanner:
	@$(call msg, Starting the Robotics AI Suite ITS path planner ...)
	@# Needs fastmapping, which needs groundfloor. ITS is a pluginlib plugin,
	@# not a node: planner_server hosts it and its global costmap's static
	@# layer is subscribed to /world/map, so with no map there is no costmap
	@# and every request is refused.
	@GF_DEPTH_RELIABLE=1 $(COMPOSE) --profile suite up --build itsplanner fastmapping groundfloor

groundfloor:
	@$(call msg, Starting the Robotics AI Suite floor segmentation ...)
	@# Own profile, so the demo runs without it and this can be added or
	@# removed without touching the rest of the stack.
	@$(COMPOSE) --profile suite up --build groundfloor

full: $(ROSBASE_STAMP)
	@$(call msg, Starting the demo AND the Robotics AI Suite bricks ...)
	@# Everything at once: the core services plus groundfloor, adbscan,
	@# fastmapping and itsplanner. `make` alone starts the default profile
	@# only, and the four suite bricks sit behind the `suite` profile, so a
	@# plain `make` leaves them out entirely -- which is why OBSTACLE_SOURCE
	@# stayed on `ours` no matter what was set.
	@#
	@# GF_DEPTH_RELIABLE=1 is not optional and not a preference. FastMapping
	@# subscribes with a bare rclcpp::QoS(10), i.e. RELIABLE, and exposes no
	@# parameter to change it; against a best-effort depth publisher DDS never
	@# matches and it receives nothing, SILENTLY. Forced here so the full
	@# pipeline cannot be started in the one configuration that fails without
	@# an error message.
	@#
	@# OBSTACLE_SOURCE defaults to `union` here and not to `ours`: starting
	@# the bricks and then not steering on them is the failure mode this
	@# target exists to prevent. Override on the command line to compare.
	@xhost +local:root > /dev/null 2>&1 || true
	@GF_DEPTH_RELIABLE=1 OBSTACLE_SOURCE=$${OBSTACLE_SOURCE:-union} \
	 $(COMPOSE) --profile suite up -d --build
	@echo ""
	@$(call msg, Bricks: groundfloor -> adbscan + fastmapping -> itsplanner)
	@echo "  suite topics take ~20 s to appear. Watch for:"
	@echo "    groundfloor  'ground' and 'obstacle' point counts"
	@echo "    adbscan      SUITE_CLUSTERS rectangles"
	@echo "    fastmapping  SUITE_MAP"
	@echo "    itsplanner   SUITE_PATH (needs a map first)"
	@echo "  If a brick logs nothing at all, suspect QoS before anything else."
	@echo ""
	@trap '$(COMPOSE) --profile suite down --remove-orphans >/dev/null 2>&1; \
	       printf "\n  demo stopped\n"; exit 0' INT TERM; \
	 $(COMPOSE) --profile suite logs -f sim compositor groundfloor adbscan \
	   fastmapping itsplanner || true; \
	 $(COMPOSE) --profile suite down --remove-orphans >/dev/null 2>&1 || true

grid-probe:
	@$(call msg, Grid navigation against the rectangle one, off the hardware ...)
	@# No camera, no policy, no renderer: a synthetic room built to this
	@# lounge's measurements and 60 s of patrol under both representations.
	@# It measures the DECISION -- whether a lane exists and whether the line
	@# taken crosses an obstacle's real outline -- which is the part that was
	@# broken. The distances are a unicycle model and are not the ones the
	@# real robot walks.
	@# common/ and services/sim/ mounted too, not just the script. Without
	@# them the container imports edgebot and navigator from the IMAGE, which
	@# is whatever was baked at the last build, and the probe measures code
	@# that is not the code in the repository. Same trap as the footprint
	@# computed twice, in another place.
	@$(COMPOSE) run --rm --no-deps --entrypoint python3 \
		-v $(PWD)/scripts:/scripts:ro \
		-v $(PWD)/common:/common:ro \
		-v $(PWD)/services/sim:/navsrc:ro \
		-e PYTHONPATH=/common:/navsrc \
		sim /scripts/grid_probe.py $(ARGS)

suite-compare:
	@$(call msg, Comparing the two floor pipelines ...)
	@# Runs inside a container so it reaches the bus by name, and mounts the
	@# script rather than baking it into an image: it is a measurement, and
	@# it will change every time we learn something from what it reports.
	@$(COMPOSE) run --rm --no-deps --entrypoint python3 \
		-v $(PWD)/scripts:/scripts:ro \
		perception /scripts/suite_compare.py $(ARGS)

map-check:
	@# No apostrophe in this message: `msg` expands into a shell single-quoted
	@# string, so one turns the whole recipe line into an unterminated quote.
	@$(call msg, Reading the accumulated FastMapping grid ...)
	@# Same arrangement as suite-compare: inside a container to reach the bus
	@# by name, script mounted rather than baked, because it is a measurement.
	@# common/ is mounted too, not just the script. Adding a bus topic edits
	@# common/edgebot/topics.py, which every service image has BAKED IN, and
	@# this target rebuilds nothing -- so a fresh topic fails here with
	@# AttributeError until an unrelated image is rebuilt. Mounting it makes the
	@# measurement read the tree, which is what a measurement should do.
	@$(COMPOSE) run --rm --no-deps --entrypoint python3 \
		-v $(PWD)/scripts:/scripts:ro \
		-v $(PWD)/common:/opt/edgebot:ro \
		perception /scripts/map_check.py $(ARGS)

lane-probe:
	@$(call msg, Measuring where the robot can actually walk in this room ...)
	@# Same arrangement as map-check: in a container to reach the bus by
	@# name, script and common/ mounted rather than baked, because it is a
	@# measurement and it must read the tree.
	@# Reads the SAME occupancy grid the navigator steers on and asks the
	@# SAME question free_lane asks, at the same corridor width, so a lane it
	@# reports clear is a lane the navigator will accept.
	@# The navigator knobs are forwarded EXPLICITLY. This runs in the
	@# perception service, whose environment block declares none of them, and
	@# `docker compose run` passes the compose environment, not the shell's.
	@# Without this the probe silently measured against DETOUR_MAX=1.8 while
	@# the sim was running with 2.4, and its verdict contradicted the stack.
	@# Same defaults as navigator.py, so a bare `make lane-probe` still
	@# describes the shipped configuration.
	@$(COMPOSE) run --rm --no-deps --entrypoint python3 \
		-v $(PWD)/scripts:/scripts:ro \
		-v $(PWD)/common:/opt/edgebot:ro \
		-e LANE=$${LANE:-0.39} \
		-e DETOUR_MAX=$${DETOUR_MAX:-1.8} \
		-e RETURN_TO=$${RETURN_TO:-1.9} \
		-e STOP_AT=$${STOP_AT:-6.0} \
		-e ROBOT_HALF_WIDTH=$${ROBOT_HALF_WIDTH:-0.22} \
		-e LANE_SLACK=$${LANE_SLACK:-0.08} \
		perception /scripts/lane_probe.py $(ARGS)

bus-rate:
	@$(call msg, Measuring what the bus carries, per topic ...)
	@# Runs in a container to reach the broker by name. Mounted, not baked: it
	@# is a measurement, and it must read the tree rather than an older image.
	@$(COMPOSE) run --rm --no-deps --entrypoint python3 \
		-v $(PWD)/scripts:/scripts:ro \
		-v $(PWD)/common:/opt/edgebot:ro \
		perception /scripts/bus_rate.py $(ARGS)

web-latency:
	@$(call msg, Timing the console stream from the compositor to the socket ...)
	@# From the HOST, not a container: the point is to measure what a viewer on
	@# the LAN sees, and a container on the compose network would skip the
	@# published port. Pass ARGS="--host <lan-ip>" to measure from elsewhere.
	@python3 scripts/web_latency.py $(ARGS)

pick-goals:
	@$(call msg, Choosing mutually reachable goals from the map ...)
	@$(COMPOSE) run --rm --no-deps --entrypoint python3 \
		-v $(PWD)/scripts:/scripts:ro \
		-v $(PWD)/common:/opt/edgebot:ro \
		perception /scripts/pick_goals.py $(ARGS)

seg-test: $(IMAGES_STAMP)
	@$(call msg, Running the segmentation model on the live camera feed ...)
	@# Frames come from the running source service over the bus, not from the
	@# camera directly: a RealSense has one client, so opening it here would
	@# fail whenever the demo holds it, which is exactly when you want to look.
	@$(COMPOSE) run --rm --entrypoint python3 \
		perception /app/seg_test.py $(SEG_ARGS)

$(IMAGES_STAMP): .env $(DOCKERFILES) $(REQ_FILES) $(SRC_FILES)
	@# A repeated key parses fine in Python and is rejected by Docker, so it
	@# only shows up after the long asset fetch. Check it first.
	@python3 scripts/check_compose.py docker-compose.yml >/dev/null
	@# py_compile accepts a function that reads a name defined nowhere; the
	@# error only surfaces when that line runs. Catch it before the build.
	@python3 scripts/check_names.py services/*/*.py common/edgebot/*.py >/dev/null
	@$(call msg, Building container images ...)
	@mkdir -p $(STAMP_DIR)
	@$(COMPOSE) build
	@touch $@

build: $(ASSETS_STAMP) $(POLICY_STAMP) $(IMAGES_STAMP)

# Always stop first: a container left over from a previous run holds the
# RealSense device, and the next start then fails on a busy V4L2 ioctl.
run: down build
	@xhost +local:root > /dev/null 2>&1 || true
	@$(call msg, Starting the demo ...)
	@$(COMPOSE) up -d
	@# Ctrl-C interrupts `logs -f`, which used to leave every container running
	@# in the background. Trap it and bring the stack down, so one Ctrl-C stops
	@# the demo as anyone would expect.
	@trap '$(COMPOSE) down --remove-orphans >/dev/null 2>&1; \
	       printf "\n  demo stopped\n"; exit 0' INT TERM; \
	 $(COMPOSE) logs -f source sim compositor perception recorder || true; \
	 $(COMPOSE) down --remove-orphans >/dev/null 2>&1 || true

down:
	@$(COMPOSE) down --remove-orphans 2>/dev/null || true

restart: run

# Runs inside the source container, the one that owns the camera, so the same
# RealSense settings and filters apply as during the demo. Floor pixels are
# tinted red so the pose can be checked before it is written.
calibrate: $(IMAGES_STAMP) down
	@test -n "$(HEIGHT)" || (echo "Usage: make calibrate HEIGHT=1.50   (camera height in metres)"; exit 1)
	@xhost +local:root > /dev/null 2>&1 || true
ifneq ($(CALIB_YOLO_MASK),0)
	@# Combined mode, the default: a pixel is floor when it passes the height
	@# threshold AND is outside every detected object. Geometry proposes and
	@# catches what has no COCO class (thin stool legs, the mast); the model
	@# subtracts what it recognises by its real SHAPE. It can only ever remove
	@# floor -- there is no floor class -- so a false positive costs a manual
	@# FILL while a false negative leaves furniture declared walkable. That
	@# asymmetry is why CALIB_YOLO_CONF is far below the runtime threshold.
	@#
	@# Three steps because no single image can do all three jobs: only `source`
	@# may open the camera, only `perception` carries OpenVINO and the model,
	@# and the calibration UI belongs with the camera. The detector is the SAME
	@# one perception runs -- seg_test.py builds the same Detector on the same
	@# weights, it is not a second inference path.
	@$(call msg, Grabbing $(or $(CALIB_YOLO_FRAMES),15) frames for the detector ...)
	@$(COMPOSE) run --rm --no-deps source python3 /app/calibrate.py \
		--height $(HEIGHT) --dump-frame /data/calib-frame.png \
		--dump-count $(or $(CALIB_YOLO_FRAMES),15)
	@$(call msg, Running YOLO11m-seg over them and unioning the contours ...)
	@$(COMPOSE) run --rm --no-deps perception python3 /app/seg_test.py \
		--images '/data/calib-frame-*.png' --out /data/calib-seg \
		--mask-out /data/calib-furniture.png \
		--mask-classes-out /data/calib-furniture \
		--manifest /data/calib-furniture.json \
		--conf $(or $(CALIB_YOLO_CONF),0.10) $(SEG_ARGS)
	@$(call msg, Camera calibration - furniture already removed from the red ...)
	@$(COMPOSE) run --rm --no-deps source python3 /app/calibrate.py \
		--height $(HEIGHT) --furniture-mask /data/calib-furniture.png
else
	@$(call msg, Camera calibration - floor shown in red ...)
	@$(COMPOSE) run --rm --no-deps source python3 /app/calibrate.py --height $(HEIGHT)
endif

logs:
	@$(COMPOSE) logs -f $(S)

ps:
	@$(COMPOSE) ps

shell:
	@$(COMPOSE) exec $(or $(S),compositor) /bin/bash

clean: down
	@rm -rf $(STAMP_DIR)

distclean: clean
	@$(COMPOSE) down --rmi local --volumes 2>/dev/null || true
	@rm -rf assets models/g1_walker policies/g1_walker/walker.onnx \
	        policies/g1_walker/walker.onnx.data policies/g1_walker/walker_meta.json

define msg
	tput setaf 2 && \
	for i in $(shell seq 1 100); do echo -n "-"; done; echo "" && \
	echo "         "$1 && \
	for i in $(shell seq 1 100); do echo -n "-"; done; echo "" && \
	tput sgr0
endef
