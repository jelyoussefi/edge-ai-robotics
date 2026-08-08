#!/bin/bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Two processes, stopped together when either dies -- same arrangement as the
# adbscan brick, and for the same reason: a crashed node would otherwise leave
# the bridge waiting on a topic nobody publishes, which reads as "working".
#
#   bridge.py       world/map (OccupancyGrid) -> the bus, as SUITE_MAP
#   fast_mapping    the mapper
#
# REQUIRES services/groundfloor TO BE RUNNING. Since the etape C rewire that
# container OWNS the camera path -- it publishes /camera/depth/image_rect_raw,
# /camera/depth/camera_info and the base_link -> camera_depth_optical_frame TF.
# This brick publishes none of them and consumes all three, so there is exactly
# one producer of each. compose enforces it with depends_on; `make fastmapping`
# brings both up.
set -euo pipefail

source /opt/ros-env.sh

# WHY map_frame IS base_link AND NOT "map".
#
# Their default is "map", the SLAM convention: FastMapping ships beside RTAB-Map,
# which publishes map -> odom -> base_link. There is no SLAM here and there is no
# odometry -- the camera is on a tripod and does not move -- so a "map" frame
# would be an empty node in the TF tree and every lookup would fail with
# "Failed to get transform from camera_depth_optical_frame to map".
#
# base_link is already this project's world frame by construction (origin on the
# ground under the camera, x forward, y left, z up), and groundfloor publishes
# base_link -> camera_depth_optical_frame. Naming it as the map frame is
# therefore not a workaround: for a fixed camera the map frame and the world
# frame ARE the same thing, and this makes the grid come back in the coordinates
# everything else here already uses.
MAP_FRAME="${FM_MAP_FRAME:-base_link}"

# max_depth_range: their default is 3.0 m, sized for an AMR that maps the
# corridor it is driving down. This room's far wall is at 6.2 m and the whole
# point of the acceptance test is whether the walls appear, so 3.0 would decide
# the answer before the measurement. 7.0 clears the wall with margin.
MAX_RANGE="${FM_MAX_DEPTH_RANGE:-7.0}"

# projection_min_z / max_z: the height band flattened into the 2D grid, in the
# map frame -- which is base_link, so these are plain heights above the floor.
# Their launch file uses 0.2 to 0.5, an AMR's bumper height. Here the band has
# to hold what the navigator calls an obstacle: above the floor-fit residue at
# the bottom (see GF_Z_LOW, 0.12) and up over a table top at the other end.
MIN_Z="${FM_PROJECTION_MIN_Z:-0.15}"
MAX_Z="${FM_PROJECTION_MAX_Z:-1.20}"

# voxel_size: 0.04 is their default and is kept. Our own floor grid is 0.10 and
# the comparison rasterises at 0.05, so a finer map costs nothing to compare
# against and can only be coarsened later.
VOXEL="${FM_VOXEL_SIZE:-0.04}"
NOISE="${FM_NOISE_FACTOR:-0.02}"

echo "fastmapping: map_frame=${MAP_FRAME} range=${MAX_RANGE} m " \
     "band=${MIN_Z}..${MAX_Z} m voxel=${VOXEL} m" >&2

python3 /app/bridge.py &
BRIDGE=$!

# depth_cameras 1: one D455. The node also supports 2, 3 and 4 and synchronises
# them with message_filters ApproximateTime; with one camera the sync is between
# the depth image and its camera_info, which our bridge publishes with the same
# header, so it matches on every frame.
# fast_mapping_node, not fast_mapping: the package is fast_mapping
# (package.xml) but CMakeLists.txt:74 names the target fast_mapping_node, and
# `ros2 run` wants the executable. Getting it wrong exits with the unhelpful
# "No executable found" and no hint that the package itself was located.
ros2 run fast_mapping fast_mapping_node --ros-args \
    -p map_frame:="${MAP_FRAME}" \
    -p depth_cameras:=1 \
    -p depth_topic_1:="/${GF_SENSOR_NAME:-camera}/depth/image_rect_raw" \
    -p depth_info_topic:="/${GF_SENSOR_NAME:-camera}/depth/camera_info" \
    -p max_depth_range:="${MAX_RANGE}" \
    -p projection_min_z:="${MIN_Z}" \
    -p projection_max_z:="${MAX_Z}" \
    -p voxel_size:="${VOXEL}" \
    -p noise_factor:="${NOISE}" &
MAPPER=$!

trap 'kill $BRIDGE $MAPPER 2>/dev/null || true' INT TERM
wait -n $BRIDGE $MAPPER
echo "one of the two processes exited, stopping the other" >&2
kill $BRIDGE $MAPPER 2>/dev/null || true
wait || true
