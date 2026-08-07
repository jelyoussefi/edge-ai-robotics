#!/bin/bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Three processes, stopped together when any one dies. Run separately, a
# crashed clusterer would leave the bridge forwarding depth to nobody, which
# reads as "working" in the logs -- the failure mode that cost the most time on
# the groundfloor service.
#
#   bridge.py             bus depth -> ROS depth image + camera_info + TF,
#                         and ROS obstacle_array -> bus footprints
#   point_cloud_xyz_node  depth image -> PointCloud2, because ADBSCAN consumes
#                         a cloud and our bus carries an image
#   adbscan_sub           the clusterer itself
set -euo pipefail

# ROS setup scripts are not nounset-clean: setup.bash reads
# AMENT_TRACE_SETUP_FILES before assigning it, which under `set -u` kills the
# container on line one. Same trap as the groundfloor entrypoint.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /ws/install/setup.bash
set -u

# z_filter cannot be a constant: it is the ground plane's height in a
# CAMERA-CENTRED frame, so it depends on how high this camera is mounted. See
# params.yaml.in. Rendered here rather than baked so `make calibrate` stays the
# single source of truth for the mounting.
PARAMS=/tmp/adbscan_params.yaml
python3 - "$PARAMS" <<'PY'
import json, os, sys

calib = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
tol = float(os.environ.get("ADBSCAN_Z_TOL", "0.08"))
try:
    with open(calib) as fh:
        height = float(json.load(fh).get("camera_height_m", 1.56))
except Exception as exc:                                        # noqa: BLE001
    print(f"no calibration at {calib} ({exc}); assuming 1.56 m", file=sys.stderr)
    height = 1.56
z = -(height - tol)
with open("/app/params.yaml.in") as fh:
    text = fh.read().replace("@Z_FILTER@", f"{z:.4f}")
with open(sys.argv[1], "w") as fh:
    fh.write(text)
print(f"adbscan z_filter = {z:.4f} (camera {height:.2f} m, tolerance {tol:.2f} m)",
      file=sys.stderr)
PY

python3 /app/bridge.py &
BRIDGE=$!

# ADBSCAN wants a cloud in the OPTICAL frame: it applies its own RS rotation
# (z,-y,-x -> x,y,z) internally, so handing it an already-rotated cloud would
# rotate it twice. point_cloud_xyz_node emits exactly that, in the frame the
# depth image declares.
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
    -r image_rect:="/${GF_SENSOR_NAME:-camera}/depth/image_rect_raw" \
    -r camera_info:="/${GF_SENSOR_NAME:-camera}/depth/camera_info" \
    -r points:=/camera/depth/color/points &
CLOUD=$!

# No use_best_effort_qos to pass: unlike the groundfloor node, this one
# subscribes with rclcpp::SensorDataQoS() unconditionally, which is already
# BEST_EFFORT and therefore already matches what the bridge publishes.
ros2 run adbscan_ros2 adbscan_sub --ros-args --params-file "$PARAMS" &
SEG=$!

trap 'kill $BRIDGE $CLOUD $SEG 2>/dev/null || true' INT TERM
wait -n $BRIDGE $CLOUD $SEG
echo "one of the three processes exited, stopping the others" >&2
kill $BRIDGE $CLOUD $SEG 2>/dev/null || true
wait || true
