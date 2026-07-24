# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

#----------------------------------------------------------------------------------------------------------------------
# Flags
#----------------------------------------------------------------------------------------------------------------------
SHELL := /bin/bash
CURRENT_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# Configuration — override on the command line, e.g. `make run ROBOT=h1 POLICY=rl`.
ROBOT    ?= g1
POLICY   ?= kinematic
COMPOSE  ?= docker compose
DISPLAY  ?= :0

# The render group id differs per distribution; the viewer needs it to reach
# /dev/dri on the host. Resolved here so compose does not have to guess.
RENDER_GID := $(shell getent group render 2>/dev/null | cut -d: -f3)
VIDEO_GID  := $(shell getent group video  2>/dev/null | cut -d: -f3)

# CPU set for the physics loop. Leave empty to let the scheduler decide, or set
# to isolated cores, e.g. `make run SIM_CPUS=8-11`.
SIM_CPUS ?=

export ROBOT POLICY DISPLAY RENDER_GID VIDEO_GID SIM_CPUS

# Stamp files: build/fetch re-run only when their inputs actually change,
# never just because make was invoked again.
STAMP_DIR    := .make
IMAGES_STAMP := $(STAMP_DIR)/images
MODELS_STAMP := $(STAMP_DIR)/models
XHOST_STAMP  := $(STAMP_DIR)/xhost

MENAGERIE    := models/mujoco_menagerie
DOCKERFILES  := $(shell find services -name Dockerfile 2>/dev/null)
SRC_FILES    := $(shell find services common -type f -name '*.py' 2>/dev/null)
REQ_FILES    := $(shell find services -type f -name 'requirements.txt' 2>/dev/null)

#----------------------------------------------------------------------------------------------------------------------
# Targets
#----------------------------------------------------------------------------------------------------------------------
default: run

.PHONY: help build models run up down restart teleop logs ps shell check lint test format clean distclean

help:
	@$(call msg, Edge AI Robotics - available commands)
	@echo "  make build      Fetch robot models and build every container image"
	@echo "  make run        Bring the stack up and open the viewer (default)"
	@echo "  make teleop     Attach the keyboard controller in this terminal"
	@echo "  make down       Stop the stack"
	@echo "  make restart    Restart the stack"
	@echo "  make logs       Follow the logs of every service   [S=sim]"
	@echo "  make ps         Show container status"
	@echo "  make shell      Open a shell inside a service      [S=sim]"
	@echo "  make models     Fetch the MuJoCo Menagerie robot models"
	@echo "  make check      Run lint, type-check and tests"
	@echo "  make format     Auto-format the source"
	@echo "  make clean      Remove build stamps and containers"
	@echo "  make distclean  Also remove images and downloaded models"
	@echo ""
	@echo "  Configuration:  ROBOT=$(ROBOT)  POLICY=$(POLICY)  SIM_CPUS=$(SIM_CPUS)"
	@echo "  Robots:         g1 h1 t1        Policies: kinematic rl"

# Robot meshes are large and separately licensed, so they are fetched rather
# than vendored. The stamp file (not the directory) is the target, so a shallow
# pull does not re-trigger the clone.
$(MODELS_STAMP): scripts/fetch_models.sh
	@$(call msg, Fetching robot models ...)
	@mkdir -p $(STAMP_DIR)
	@bash scripts/fetch_models.sh
	@touch $@

models: $(MODELS_STAMP)

# Generate .env once, on first use. It is a real file target, so it is created
# only when missing — never on every run.
.env:
	@$(call msg, Creating .env from .env.example ...)
	@cp .env.example .env
	@echo "  Created .env — review it before the first run."

$(IMAGES_STAMP): .env $(DOCKERFILES) $(REQ_FILES) $(SRC_FILES)
	@$(call msg, Building container images ...)
	@mkdir -p $(STAMP_DIR)
	@$(COMPOSE) build
	@touch $@

build: $(MODELS_STAMP) $(IMAGES_STAMP)

# The viewer draws on the host X display, so the local user has to be allowed
# through once per login session.
$(XHOST_STAMP):
	@$(call msg, Granting the viewer access to the X display ...)
	@mkdir -p $(STAMP_DIR)
	@xhost +local:root > /dev/null 2>&1 || echo "  xhost not available — skipping (Wayland or headless?)"
	@touch $@

run: build $(XHOST_STAMP)
	@$(call msg, Starting the stack - robot $(ROBOT), policy $(POLICY) ...)
	@echo "  The humanoid opens on $(DISPLAY). Run 'make teleop' in a second terminal to drive it."
	@$(COMPOSE) up -d bus sim viewer
	@$(COMPOSE) logs -f sim viewer

up: run

down:
	@$(call msg, Stopping the stack ...)
	@$(COMPOSE) down

restart: down run

# Interactive, so it runs in the foreground with a tty attached rather than as
# a compose service.
teleop: $(IMAGES_STAMP)
	@$(call msg, Keyboard control - W/S walk, A/D turn, Q/E strafe, space stop, Ctrl-C quit)
	@$(COMPOSE) run --rm teleop

logs:
	@$(COMPOSE) logs -f $(S)

ps:
	@$(COMPOSE) ps

shell:
	@$(COMPOSE) exec $(or $(S),sim) /bin/bash

check: $(IMAGES_STAMP)
	@$(call msg, Running lint, type-check and tests ...)
	@$(COMPOSE) run --rm --no-deps sim python -m ruff check /opt/edgebot /app
	@$(COMPOSE) run --rm --no-deps sim python -m mypy --ignore-missing-imports /app
	@$(COMPOSE) run --rm --no-deps sim python -m pytest -q /app

lint: $(IMAGES_STAMP)
	@$(COMPOSE) run --rm --no-deps sim python -m ruff check /opt/edgebot /app

test: $(IMAGES_STAMP)
	@$(COMPOSE) run --rm --no-deps sim python -m pytest -q /app

format: $(IMAGES_STAMP)
	@$(COMPOSE) run --rm --no-deps sim python -m ruff format /opt/edgebot /app

clean:
	@$(call msg, Removing containers and build stamps ...)
	@$(COMPOSE) down --remove-orphans 2>/dev/null || true
	@rm -rf $(STAMP_DIR)

distclean: clean
	@$(call msg, Removing images and downloaded models ...)
	@$(COMPOSE) down --rmi local --volumes 2>/dev/null || true
	@rm -rf $(MENAGERIE)
	@echo "  Note: .env and anything under policies/ is yours and was left untouched."

#----------------------------------------------------------------------------------------------------------------------
# Helper Functions
#----------------------------------------------------------------------------------------------------------------------
define msg
	tput setaf 2 && \
	for i in $(shell seq 1 118); do echo -n "-"; done; echo "" && \
	echo "         "$1 && \
	for i in $(shell seq 1 118); do echo -n "-"; done; echo "" && \
	tput sgr0
endef
