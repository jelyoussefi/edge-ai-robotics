# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Intel's FastMapping, plus a bridge that puts its occupancy grid on this
# project's bus. The THIRD consumer of docker/ros-base, which is why that image
# exists: Ubuntu 24.04, ROS Jazzy, PCL, the colcon toolchain and pyzmq/msgpack
# are all shared, and only what is specific to this component is below.
#
# BUILD IT THROUGH COMPOSE, not with a bare `docker build`. docker-compose.yml
# maps this FROM to `service:ros-base` as a named build context, so the base is
# rebuilt first if it is stale. `make fastmapping` does the right thing.
FROM edge-ai-robotics-ros-base:jazzy

# ROS_DISTRO comes from the base as an ENV, so it is not redeclared here.

ARG SUITE_REPO=https://github.com/open-edge-platform/edge-ai-suites.git
# The same commit the other two bricks are pinned to. Bumping one brick and not
# the others would mean the three are no longer the same suite, and the etape B
# and C measurements would stop being comparable with anything measured here.
ARG SUITE_COMMIT=d35ad014d42e270630cd7866f38e679b7bd8ea4a

# Single ament_cmake package at the component root -- no nested package
# directory, unlike groundfloor -- so the component directory IS the package and
# is copied straight into the workspace.
RUN mkdir -p /ws/src && cd /tmp \
    && git init suite && cd suite \
    && git remote add origin ${SUITE_REPO} \
    && git config core.sparseCheckout true \
    && echo "robotics-ai-suite/components/fast-mapping/" > .git/info/sparse-checkout \
    && git fetch --depth 1 --filter=blob:none origin ${SUITE_COMMIT} \
    && git checkout FETCH_HEAD \
    && cp -r robotics-ai-suite/components/fast-mapping /ws/src/fast_mapping \
    && cd / && rm -rf /tmp/suite

# cv_bridge and message_filters are the two build deps the other bricks do not
# pull in: FastMapping takes depth as an Image (not a cloud) and synchronises it
# with camera_info through message_filters. Installed here rather than in the
# base because no other brick needs them -- the base is for what is shared.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-visualization-msgs \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN cd /ws && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Late, and after the colcon build, on purpose: editing the bridge or one line
# of common/edgebot rebuilds these thin layers only, never the workspace above.
COPY common /opt/edgebot
COPY services/fastmapping/ /app/
WORKDIR /app

COPY services/fastmapping/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
