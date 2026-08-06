# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Intel's pointcloud groundfloor segmentation, plus a bridge to this project's
# bus. Ubuntu 24.04 and ROS 2 Jazzy, which is what their package supports and,
# conveniently, the same base as the other services here: the version clash
# feared at the outset does not exist, since the brick ships for Jazzy as well
# as Humble.
#
# Kept as its own image rather than added to `perception`. ROS pulls in a large
# dependency tree and its own DDS middleware, and none of that belongs in a
# service whose job is to run one model.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release software-properties-common wget \
      python3 python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy from the official repository.
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-sensor-msgs \
      python3-numpy python3-opencv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# The brick is BUILT FROM SOURCE rather than installed from Intel's AMR APT
# repository. Tried and abandoned, twice, with the binary route:
#
#   1. The AMR .deb links against Intel's oneAPI build of PCL, whose runtime
#      lives in apt.repos.intel.com, absent -> every dependency "not
#      installable".
#   2. With that repository added, its 2025.x runtimes have been rotated out:
#      only 2026.1.1 remains, and the AMR package fences at < 2026.0.0. Binary
#      route dead until Intel rebuilds one side or the other.
#
# The source tells a different story: the suite is public, Apache-2.0, at
# github.com/open-edge-platform/edge-ai-suites, and its CMakeLists wants plain
# `find_package(PCL)` from Ubuntu's libpcl-dev. The oneAPI linkage exists ONLY
# behind FUZZTEST_FUZZING_MODE, so a release build never touches it. Intel's
# own Dockerfile for this component does exactly this: ros base + libpcl-dev +
# colcon. The launch file imports nav2_common.launch, hence ros-jazzy-nav2-
# common; nav2_bringup in package.xml is exec_depend for the AAEON demo launch
# only, and would drag the whole Nav2 stack into the image for nothing.
ARG SUITE_REPO=https://github.com/open-edge-platform/edge-ai-suites.git
# Pinned to the commit reviewed on 2026-08-06. Bump deliberately, not by
# rebuilding: main moves and this brick is the measurement baseline for step B.
ARG SUITE_COMMIT=d35ad014d42e270630cd7866f38e679b7bd8ea4a

RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake \
      python3-colcon-common-extensions \
      libpcl-dev libeigen3-dev \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-tf2-eigen \
      ros-${ROS_DISTRO}-nav2-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

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

RUN pip3 install --break-system-packages --no-cache-dir pyzmq msgpack

COPY common /opt/edgebot
COPY services/groundfloor/ /app/
WORKDIR /app

# Launch their node and the bridge together. Their launch file expects a camera
# publishing on /<sensor>/depth; the bridge is that camera, fed from the bus.
COPY services/groundfloor/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
