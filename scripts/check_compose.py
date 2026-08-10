#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Reject a compose file that repeats a key, or declares a knob nobody reads.

Two checks, both for defects this repository has actually shipped.

DUPLICATE KEYS. PyYAML keeps the last of two identical keys without
complaining, so a plain safe_load says the file is fine while Docker refuses it
outright. This walks the node tree instead of constructing it, which also
sidesteps the merge tags the compose file uses for its anchors.

DEAD KNOBS. A variable declared in compose and read by nothing is worse than
clutter: it reads as a control. OBSTACLE_CONF cost hours of tuning before
anyone noticed that the detector threshold lives in
config/streams.d455.json and that nothing anywhere reads OBSTACLE_CONF. Two
more, PATROL_MODE and CORNER_R, advertised behaviour that does not exist --
PATROL_MODE=perimeter was never implemented.

A knob counts as read only if it appears in something that EXECUTES: Python,
Dockerfiles, entrypoints and other shell, the Makefile, ROS parameter
templates. Documentation deliberately does not count -- all three dead knobs
are named in docs, including in the plan that asks for their removal, so
counting prose would have reported the tree clean. Variables consumed by
third-party runtimes cannot be found by grep at all and are listed in
READ_ELSEWHERE, each with the reason.
"""
import os
import re
import subprocess
import sys

import yaml


def check(path):
    bad = []

    def walk(node, where=""):
        if isinstance(node, yaml.MappingNode):
            seen = {}
            for k, v in node.value:
                key = getattr(k, "value", None)
                if key in seen:
                    bad.append(f"{where}{key} (lines {seen[key]} and "
                               f"{k.start_mark.line + 1})")
                else:
                    seen[key] = k.start_mark.line + 1
                walk(v, f"{where}{key}.")
        elif isinstance(node, yaml.SequenceNode):
            for i, v in enumerate(node.value):
                walk(v, f"{where}{i}.")

    with open(path) as fh:
        walk(yaml.compose(fh))
    return bad


# Read by something grep cannot see. Every entry carries who reads it, because
# an unexplained exemption is how a dead knob comes back.
READ_ELSEWHERE = {
    "PYTHONUNBUFFERED": "the Python runtime itself",
    "MUJOCO_GL": "mujoco picks its GL backend from this",
    "LIBVA_DRIVER_NAME": "libva",
    "INTEL_FORCE_PROBE": "the Mesa driver",
    "XDG_RUNTIME_DIR": "the X/Wayland client libraries",
    "ROS_DISTRO": "the ROS setup scripts",
    "ROS_DOMAIN_ID": "rclcpp/rclpy DDS discovery",
    "DISPLAY": "the X client libraries",
    "QT_X11_NO_MITSHM": "Qt",
    "OPENCV_LOG_LEVEL": "OpenCV",
    "NO_AT_BRIDGE": "GTK",
}


def declared_env(path):
    """Variable names declared under any environment: block, with line numbers."""
    out = {}
    with open(path) as fh:
        for n, line in enumerate(fh, 1):
            m = re.match(r"\s+([A-Z][A-Z0-9_]*):\s", line)
            if m:
                out.setdefault(m.group(1), n)
    return out


def unread(path):
    """Names declared in compose that appear nowhere else in the tree."""
    root = os.path.dirname(os.path.abspath(path)) or "."
    dead = []
    for name, line in sorted(declared_env(path).items()):
        if name in READ_ELSEWHERE:
            continue
        # git grep so the search follows what is tracked rather than whatever
        # build output happens to be lying around.
        hit = subprocess.run(
            ["git", "-C", root, "grep", "-lF", "--", name,
             "--", "*.py", "*.sh", "*.yaml", "*.yml", "*.in", "*.json",
             "Makefile", "*/Dockerfile", "Dockerfile", "*.bash", "*.cpp",
             "*.xml"],
            capture_output=True, text=True)
        # This file names the dead knobs in its own docstring as examples, and
        # a checker that counts itself as a consumer never fires again.
        files = [f for f in hit.stdout.split()
                 if f != os.path.basename(path)
                 and not f.startswith("docs/")
                 and not f.endswith("check_compose.py")]
        if not files:
            dead.append((name, line))
    return dead


if __name__ == "__main__":
    problems = []
    for p in sys.argv[1:] or ["docker-compose.yml"]:
        for b in check(p):
            problems.append(f"{p}: duplicate key {b}")
        for name, line in unread(p):
            problems.append(f"{p}:{line}: {name} is declared here and read "
                            f"nowhere in the tree")
    for line in problems:
        print(line)
    print("no duplicate keys, no dead knobs" if not problems else "FIX THESE")
    sys.exit(1 if problems else 0)

