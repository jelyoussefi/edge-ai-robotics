# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Intel's pointcloud groundfloor segmentation, plus a bridge to this project's
# bus. Everything shared with the other suite bricks -- Ubuntu 24.04, ROS Jazzy,
# PCL, the colcon toolchain, pyzmq/msgpack -- lives in docker/ros-base. What is
# left below is only what is specific to this component.
#
# Kept as its own image rather than folded into `perception`: ROS pulls in a
# large dependency tree and its own DDS middleware, and none of that belongs in
# a service whose job is to run one model. One container per brick.
#
# BUILD IT THROUGH COMPOSE, not with a bare `docker build`. docker-compose.yml
# maps this FROM to `service:ros-base` as a named build context, so the base is
# rebuilt first if it is stale. A bare `docker build` would silently use
# whatever edge-ai-robotics-ros-base:jazzy happens to be lying around, or fail
# to find it. `make groundfloor` and `make build` do the right thing.
FROM edge-ai-robotics-ros-base:jazzy

# ROS_DISTRO comes from the base as an ENV, so it is not redeclared here.

ARG SUITE_REPO=https://github.com/open-edge-platform/edge-ai-suites.git
# Pinned to the commit reviewed on 2026-08-06. Bump deliberately, not by
# rebuilding: main moves and this brick is the measurement baseline for step B.
ARG SUITE_COMMIT=d35ad014d42e270630cd7866f38e679b7bd8ea4a

# Sparse checkout of the one component, fetched by SHA so the build is
# reproducible even after main moves. GitHub serves reachable SHAs directly.
RUN mkdir -p /ws/src && cd /tmp \
    && git init suite && cd suite \
    && git remote add origin ${SUITE_REPO} \
    && git config core.sparseCheckout true \
    && echo "robotics-ai-suite/components/groundfloor/" > .git/info/sparse-checkout \
    && git fetch --depth 1 --filter=blob:none origin ${SUITE_COMMIT} \
    && git checkout FETCH_HEAD \
    && cp -r robotics-ai-suite/components/groundfloor/pointcloud_groundfloor_segmentation \
             /ws/src/ \
    && cd / && rm -rf /tmp/suite

RUN cd /ws && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Late, and after the colcon build, on purpose: editing a bridge or one line of
# common/edgebot rebuilds these thin layers only, never the workspace above.
COPY common /opt/edgebot
COPY services/groundfloor/ /app/
WORKDIR /app

# Launch their node and the bridge together. Their launch file expects a camera
# publishing on /<sensor>/depth; the bridge is that camera, fed from the bus.
COPY services/groundfloor/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
