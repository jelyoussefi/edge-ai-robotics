# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Shared base for the Intel Robotics AI Suite bricks. One container per brick
# stays the rule -- this is not a merge of the services, it is the ~4.6 GB of
# Ubuntu, ROS and PCL that every brick needs, built once instead of once per
# brick.
#
# Consumers today: services/groundfloor, services/adbscan. FastMapping will be
# the third, which is why this exists now rather than after the second copy of
# the same apt block was noticed.
#
# WHAT BELONGS HERE, AND WHAT DOES NOT
#
# Here: anything a suite brick needs to COMPILE and RUN a ROS component from
# source -- the distribution, the ROS runtime, the message packages the suite
# components pull in, the build toolchain, and the two Python wheels the bridges
# need to reach this project's bus.
#
# Not here: `COPY common /opt/edgebot`, deliberately. Putting it in the base
# would mean that editing one line of common/edgebot/bus.py invalidates the base
# image, and with it every layer of both bricks INCLUDING the multi-minute colcon
# build. Copied late in each brick instead, where it costs one thin layer.
#
# Also not here: anything version-pinned to a component (SUITE_COMMIT and
# friends). A base that changed when a brick's pin changed would defeat the
# point.
FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=jazzy
# As ENV, not just ARG: the bricks' colcon step and every entrypoint read it, and
# an ENV here means neither has to redeclare it. compose sets the same value at
# runtime, which is redundant and harmless.
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release software-properties-common wget \
      python3 python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS 2 Jazzy from the official repository. Jazzy and not Humble because the
# suite components ship for both and Jazzy is the Ubuntu 24.04 pairing, which is
# the same distribution the rest of this project's services use -- the version
# clash feared at the outset does not exist.
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-eigen \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-depth-image-proc \
      `# nav2: the groundfloor launch file imports nav2_common.launch, and` \
      `# adbscan's nav2_dynamic_msgs needs these two message packages. Both are` \
      `# small; nav2_bringup is NOT here and must not be -- it drags the whole` \
      `# Nav2 stack in for a launch file we do not use.` \
      ros-${ROS_DISTRO}-nav2-common \
      ros-${ROS_DISTRO}-visualization-msgs \
      ros-${ROS_DISTRO}-unique-identifier-msgs \
      python3-numpy python3-opencv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Build toolchain. The suite bricks are built FROM SOURCE rather than installed
# from Intel's AMR APT repository. Tried and abandoned, twice, with the binary
# route:
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
# behind FUZZTEST_FUZZING_MODE, so a release build never touches it. Intel's own
# Dockerfiles for these components do exactly this: ros base + libpcl-dev +
# colcon.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake \
      python3-colcon-common-extensions \
      libpcl-dev libeigen3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# The bus client. Every brick ships a bridge that publishes to this project's
# ZeroMQ bus, so these two belong to the base rather than to any one brick.
RUN pip3 install --break-system-packages --no-cache-dir pyzmq msgpack

# Sourced by each brick's entrypoint. See the file for why it cannot simply be
# two `source` lines in each entrypoint.
COPY docker/ros-base/ros-env.sh /opt/ros-env.sh
RUN chmod +x /opt/ros-env.sh

# No ENTRYPOINT and no CMD on purpose. This image is never run; naming a default
# command would invite exactly that.
