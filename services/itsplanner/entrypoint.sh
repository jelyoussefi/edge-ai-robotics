#!/bin/bash
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Three processes, stopped together when any dies. One more than the other
# bricks, because ITS is a plugin and not a node: something has to host it and
# something has to drive that host through its lifecycle.
#
#   planner_server     hosts its_planner::ITSPlanner, owns the global costmap
#   lifecycle_manager  configure + activate, without which planner_server sits
#                      in the UNCONFIGURED state and answers nothing
#   bridge.py          the static map -> base_link TF, the path request, and
#                      the path onto the bus
#
# REQUIRES services/fastmapping (and so services/groundfloor) TO BE RUNNING.
# The costmap's static layer subscribes to /world/map, which only FastMapping
# publishes. Without it the costmap stays entirely unknown and the planner
# refuses every request -- correctly, and with an error that does not name the
# missing map.
set -euo pipefail

source /opt/ros-env.sh

# min_samples is the roadmap's sample count. Rendered rather than fixed because
# it is read in configure() and the roadmap is built once, so a sweep has to
# restart the node -- see docs, and scripts/its_sweep.py which measures it.
MIN_SAMPLES="${ITS_MIN_SAMPLES:-250}"
PARAMS=/tmp/its_params.yaml
sed "s/@MIN_SAMPLES@/${MIN_SAMPLES}/" /app/params.yaml.in > "$PARAMS"
echo "itsplanner: planner_server + global costmap on /world/map, " \
     "min_samples=${MIN_SAMPLES}" >&2

python3 /app/bridge.py &
BRIDGE=$!

ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS" &
PLANNER=$!

# autostart drives UNCONFIGURED -> INACTIVE -> ACTIVE. bond is disabled because
# it is a liveness contract with the rest of a Nav2 stack that is not running
# here: with bond on, the manager tears the planner down again a few seconds
# after activating it, which reads as the planner crashing on its own.
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
    -p use_sim_time:=false \
    -p autostart:=true \
    -p bond_timeout:=0.0 \
    -p node_names:="['planner_server']" &
LIFECYCLE=$!

trap 'kill $BRIDGE $PLANNER $LIFECYCLE 2>/dev/null || true' INT TERM
wait -n $BRIDGE $PLANNER $LIFECYCLE
echo "one of the three processes exited, stopping the others" >&2
kill $BRIDGE $PLANNER $LIFECYCLE 2>/dev/null || true
wait || true
