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

DOCKERFILES := $(shell find services -name Dockerfile 2>/dev/null)
SRC_FILES   := $(shell find services common -type f -name '*.py' 2>/dev/null)
REQ_FILES   := $(shell find services -type f -name 'requirements.txt' 2>/dev/null)

.PHONY: default help build run down restart calibrate seg-test logs ps shell clean distclean
default: run

help:
	@$(call msg, Edge AI Robotics)
	@echo "  make run        Stop anything running, build if needed, start the demo"
	@echo "  make down       Stop the stack"
	@echo "  make restart    Same as run"
	@echo "  make calibrate  Camera pose, with the floor shown in red   HEIGHT=1.50"
	@echo "  make seg-test   What the segmentation model sees, as an image"
	@echo "  make logs       Follow the logs                            [S=compositor]"
	@echo "  make ps         Container status"
	@echo "  make shell      Shell inside a service                     [S=sim]"
	@echo "  make clean      Remove containers and build stamps"
	@echo "  make distclean  Also remove images and fetched assets"
	@echo ""
	@echo "  In the compositor window:  f  floor   s  detections   h  scale   r  reset   q  quit"

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
	@$(call msg, Camera calibration - floor shown in red ...)
	@$(COMPOSE) run --rm --no-deps source python3 /app/calibrate.py --height $(HEIGHT)

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
