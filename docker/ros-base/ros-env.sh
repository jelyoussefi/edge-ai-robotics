#!/bin/bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# SOURCED, never executed:  source /opt/ros-env.sh
#
# Puts ROS and the brick's own colcon workspace on the path. Three lines that
# cannot just be written inline in each entrypoint, for three separate reasons,
# each of which cost a debugging session once already.
#
# 1. ROS setup scripts are not nounset-clean. setup.bash reads
#    AMENT_TRACE_SETUP_FILES before assigning it, so under `set -u` -- which
#    every entrypoint here sets -- the container dies on line one, before any of
#    its own code runs. Relaxed for the sourcing only; strict mode is restored
#    at the end, so the caller keeps the checking it asked for.
#
# 2. /ws/install may legitimately not exist. It is created by the brick's colcon
#    build, so it is present in a finished image but absent if this file is ever
#    sourced in the base itself or in a half-built layer. Guarded with `if`
#    rather than `[ -f ... ] && source ...`: a false test as the LAST command of
#    a sourced file makes the `source` itself return non-zero, and the callers
#    run under `set -e`, so the plain && form turns a missing workspace into a
#    silent exit 1 with no message.
#
# 3. Order matters. /opt/ros first, then the overlay, or the workspace's own
#    packages lose to the installed ones.
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
set -u
