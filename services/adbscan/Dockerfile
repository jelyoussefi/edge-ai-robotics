# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Intel's ADBSCAN clustering (adbscan_ros2), plus a bridge to this project's
# bus. Same recipe as services/groundfloor: Ubuntu 24.04, ROS 2 Jazzy, the
# component built from source at a pinned SUITE_COMMIT. Its own image for the
# same reason groundfloor has one -- ROS drags in a large dependency tree and
# its own DDS, and none of it belongs in a service that runs one model.
#
# Two things differ from groundfloor and both cost a build step:
#
#   1. ADBSCAN consumes a PointCloud2, not a depth image. Their default topic
#      is a RealSense's /camera/depth/color/points. Our bus carries a depth
#      IMAGE, so depth_image_proc's point_cloud_xyz_node sits between the two.
#      That keeps the same depth path groundfloor consumes -- one bridge
#      publishing image_rect_raw plus camera_info -- rather than inventing a
#      second producer that could disagree about the intrinsics.
#   2. It depends on nav2_dynamic_msgs, which has no Jazzy binary package. Built
#      from navigation2_dynamic, also pinned.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release software-properties-common wget \
      python3 python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-visualization-msgs \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-unique-identifier-msgs \
      python3-numpy \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake \
      python3-colcon-common-extensions \
      libpcl-dev libeigen3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

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

RUN pip3 install --break-system-packages --no-cache-dir pyzmq msgpack

COPY common /opt/edgebot
COPY services/adbscan/ /app/
WORKDIR /app

COPY services/adbscan/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
