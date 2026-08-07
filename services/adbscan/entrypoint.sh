#!/bin/bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Two processes, stopped together when either dies. Run separately, a crashed
# clusterer would leave the bridge waiting on a topic nobody publishes, which
# reads as "working" in the logs -- the failure mode that cost the most time on
# the groundfloor service.
#
#   bridge.py     ROS obstacle_array -> bus footprints
#   adbscan_sub   the clusterer
#
# REQUIRES services/groundfloor TO BE RUNNING. ADBSCAN is fed
# /segmentation/obstacle_points, the groundfloor node's output, rather than a
# cloud built here: see params.yaml.in for why a single height threshold cannot
# remove a floor seen by a pitched camera. That container owns the depth path --
# it publishes the depth image, the camera_info and the TF -- so this one
# publishes none of them, and there is exactly one producer of each.
#
# compose enforces the order with depends_on; `make adbscan` brings both up.
set -euo pipefail

# ROS setup scripts are not nounset-clean: setup.bash reads
# AMENT_TRACE_SETUP_FILES before assigning it, which under `set -u` kills the
# container on line one. Same trap as the groundfloor entrypoint.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /ws/install/setup.bash
set -u

# The ground is at z = 0 now that the input is in base_link, so this is a plain
# height above the floor rather than a value derived from the mounting. Kept as
# a template substitution anyway: it is the one number that has to move if the
# floor tolerance changes, and rendering it keeps that in one place.
PARAMS=/tmp/adbscan_params.yaml
python3 - "$PARAMS" <<'PY'
import os, sys

tol = float(os.environ.get("ADBSCAN_Z_TOL", "0.08"))
with open("/app/params.yaml.in") as fh:
    text = fh.read().replace("@Z_FILTER@", f"{tol:.4f}")
with open(sys.argv[1], "w") as fh:
    fh.write(text)
print(f"adbscan z_filter = {tol:.4f} m above the floor (backstop only; the "
      f"floor itself is removed upstream by groundfloor)", file=sys.stderr)
PY

python3 /app/bridge.py &
BRIDGE=$!

# No use_best_effort_qos to pass, and none exists: adbscan_sub subscribes with
# rclcpp::SensorDataQoS(), already BEST_EFFORT, which is what the groundfloor
# node publishes its obstacle cloud with.
ros2 run adbscan_ros2 adbscan_sub --ros-args --params-file "$PARAMS" &
SEG=$!

trap 'kill $BRIDGE $SEG 2>/dev/null || true' INT TERM
wait -n $BRIDGE $SEG
echo "one of the two processes exited, stopping the other" >&2
kill $BRIDGE $SEG 2>/dev/null || true
wait || true
