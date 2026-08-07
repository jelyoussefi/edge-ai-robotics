# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Intel's ADBSCAN clustering (adbscan_ros2), plus a bridge to this project's
# bus. Same shape as services/groundfloor: everything shared sits in
# docker/ros-base, and only this component's own steps are below. Its own image
# for the same reason groundfloor has one -- one container per brick.
#
# One thing genuinely differs from groundfloor and it costs a build step:
# adbscan_ros2 depends on nav2_dynamic_msgs, which has no Jazzy binary package,
# so it is built from source alongside. (The message packages it needs at the
# apt level, visualization_msgs and unique_identifier_msgs, are in the base.)
#
# The depth path is NOT here. ADBSCAN is chained behind the groundfloor node and
# clusters /segmentation/obstacle_points; the depth_image_proc conversion this
# image used to run was removed with it. See params.yaml.in for why a single
# height threshold cannot remove a floor seen by a pitched camera.
#
# BUILD IT THROUGH COMPOSE, not with a bare `docker build`. docker-compose.yml
# maps this FROM to `service:ros-base` as a named build context, so the base is
# rebuilt first if it is stale.
FROM edge-ai-robotics-ros-base:jazzy

# ROS_DISTRO comes from the base as an ENV, so it is not redeclared here.

ARG SUITE_REPO=https://github.com/open-edge-platform/edge-ai-suites.git
# Same commit as the groundfloor image. Both bricks are the etape B/C
# measurement baseline; bump deliberately, never by rebuilding.
ARG SUITE_COMMIT=d35ad014d42e270630cd7866f38e679b7bd8ea4a
# nav2_dynamic_msgs is a hard dependency of adbscan_ros2 and ships no Jazzy
# binary, so it is built from source alongside. Message-only package: no code of
# theirs runs, and pinning it keeps the wire format fixed.
ARG N2D_REPO=https://github.com/ros-navigation/navigation2_dynamic.git
ARG N2D_COMMIT=e1b0d920fa309c7eb072524b00de7ee12e21076e

RUN mkdir -p /ws/src && cd /tmp \
    && git init suite && cd suite \
    && git remote add origin ${SUITE_REPO} \
    && git config core.sparseCheckout true \
    && echo "robotics-ai-suite/components/adbscan/ROS2_node/" > .git/info/sparse-checkout \
    && git fetch --depth 1 --filter=blob:none origin ${SUITE_COMMIT} \
    && git checkout FETCH_HEAD \
    && cp -r robotics-ai-suite/components/adbscan/ROS2_node /ws/src/adbscan_ros2 \
    && cd / && rm -rf /tmp/suite

# Only the msgs package, not the tracker: adbscan needs the message definitions
# and nothing else, and kf_hungarian_tracker would pull in more of Nav2.
RUN cd /tmp && git init n2d && cd n2d \
    && git remote add origin ${N2D_REPO} \
    && git config core.sparseCheckout true \
    && echo "nav2_dynamic_msgs/" > .git/info/sparse-checkout \
    && git fetch --depth 1 --filter=blob:none origin ${N2D_COMMIT} \
    && git checkout FETCH_HEAD \
    && cp -r nav2_dynamic_msgs /ws/src/ \
    && cd / && rm -rf /tmp/n2d

RUN cd /ws && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Late, and after the colcon build, on purpose: editing a bridge or one line of
# common/edgebot rebuilds these thin layers only, never the workspace above.
COPY common /opt/edgebot
COPY services/adbscan/ /app/
WORKDIR /app

COPY services/adbscan/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
