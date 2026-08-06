#!/bin/bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Start the bridge and Intel's segmentation node together, and stop both when
# either one dies. Run separately, a crashed segmentation would leave the bridge
# happily forwarding depth frames to nobody, which reads as "working" in the
# logs and is the failure mode hardest to notice.
set -euo pipefail

# ROS setup scripts are not nounset-clean: setup.bash reads
# AMENT_TRACE_SETUP_FILES before assigning it, which under `set -u` kills the
# container on line one. Seen on the very first start. Relax -u for the two
# sources only, the rest of the script keeps the strict mode.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# The brick is built from source into /ws, not installed under /opt/ros.
source /ws/install/setup.bash
set -u

python3 /app/bridge.py &
BRIDGE=$!

# standalone:=False because the camera is the bridge, not a RealSense node.
# use_best_effort_qos:=True because the bridge publishes depth the way a real
# camera driver does, BEST_EFFORT (bridge.py). Their node subscribes RELIABLE by
# default, and DDS refuses to match the two: the first run logged "incompatible
# QoS ... no messages will be sent" on image_rect_raw and camera_info from both
# ends, then sat silent for 30 s. The flag moves their side, not ours.
# node_params_file points at a mounted copy rather than the one baked into the
# image, so a parameter can be changed and the node restarted without a 4.8 GB
# rebuild. Falls back to their installed file if the mount is missing.
GF_PARAMS="${GF_PARAMS:-/params/groundfloor_segmentation_params.yaml}"
if [ ! -f "$GF_PARAMS" ]; then
    echo "no params at $GF_PARAMS, using the node's built-in defaults" >&2
    GF_PARAMS="/ws/install/pointcloud_groundfloor_segmentation/share/pointcloud_groundfloor_segmentation/params/groundfloor_segmentation_params.yaml"
fi
ros2 launch pointcloud_groundfloor_segmentation \
    realsense_groundfloor_segmentation_launch.py \
    standalone:=False with_rviz:=False use_best_effort_qos:=True \
    node_params_file:="$GF_PARAMS" &
SEG=$!

trap 'kill $BRIDGE $SEG 2>/dev/null || true' INT TERM
wait -n $BRIDGE $SEG
echo "one of the two processes exited, stopping the other" >&2
kill $BRIDGE $SEG 2>/dev/null || true
wait || true
