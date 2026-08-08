# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Intel's ITS Path Planner, plus a bridge that drives it and puts the resulting
# path on this project's bus. FOURTH consumer of docker/ros-base.
#
# THIS BRICK IS NOT LIKE THE OTHER THREE. groundfloor, adbscan and fastmapping
# are standalone ROS nodes: you run them and they publish. ITS is a **pluginlib
# plugin**, its_planner::ITSPlanner with base_class_type nav2_core::GlobalPlanner
# (global_planner_plugin.xml), so it does not run at all on its own. It is
# loaded by Nav2's planner_server, which is a LIFECYCLE node, and it reads its
# input from a nav2_costmap_2d::Costmap2DROS handed to it in configure() rather
# than from any topic of its own.
#
# So the dependency the ros-base header says must never be pulled in -- the Nav2
# stack -- is partly unavoidable here, and this is the deliberate exception:
#
#   nav2_planner            planner_server, the lifecycle node that loads it
#   nav2_costmap_2d         the costmap it plans on, and its static layer
#   nav2_lifecycle_manager  drives both through configure/activate
#   nav2_msgs               the ComputePathToPose action the bridge calls
#   nav2_core, pluginlib    the plugin interface itself
#
# nav2_bringup is STILL not installed and still must not be: it brings the
# behaviour tree, the controller, the smoother, the behaviour server and the
# waypoint follower, none of which E1 uses. What is above is the planning half
# of Nav2 and nothing else.
FROM edge-ai-robotics-ros-base:jazzy

ARG SUITE_REPO=https://github.com/open-edge-platform/edge-ai-suites.git
# Same commit as the other three bricks. Four bricks at one SHA is what makes
# them the same suite.
ARG SUITE_COMMIT=d35ad014d42e270630cd7866f38e679b7bd8ea4a

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-nav2-planner \
      ros-${ROS_DISTRO}-nav2-costmap-2d \
      ros-${ROS_DISTRO}-nav2-lifecycle-manager \
      ros-${ROS_DISTRO}-nav2-msgs \
      ros-${ROS_DISTRO}-nav2-core \
      ros-${ROS_DISTRO}-nav2-util \
      ros-${ROS_DISTRO}-pluginlib \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# The component root holds several packages (its_planner, send_localization,
# collab_slam, relocalization); only its_planner is wanted. The others pull in
# collaborative SLAM and a localisation sender that this project has no use for.
RUN mkdir -p /ws/src && cd /tmp \
    && git init suite && cd suite \
    && git remote add origin ${SUITE_REPO} \
    && git config core.sparseCheckout true \
    && echo "robotics-ai-suite/components/its-planner/its_planner/" \
         > .git/info/sparse-checkout \
    && git fetch --depth 1 --filter=blob:none origin ${SUITE_COMMIT} \
    && git checkout FETCH_HEAD \
    && cp -r robotics-ai-suite/components/its-planner/its_planner /ws/src/ \
    && cd / && rm -rf /tmp/suite

RUN cd /ws && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

COPY common /opt/edgebot
COPY services/itsplanner/ /app/
WORKDIR /app

COPY services/itsplanner/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
