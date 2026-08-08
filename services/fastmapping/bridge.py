#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Put Intel's FastMapping occupancy grid on this project's bus.

One job, and deliberately no second one. Unlike the other two suite bridges this
publishes nothing into ROS: since the etape C rewire the groundfloor container
owns the camera path -- depth, camera_info and the TF -- and this brick is a
pure consumer of it. If depth stops arriving here, the fault is upstream.

WHAT MAKES THIS BRICK DIFFERENT. groundfloor and adbscan both answer "what is in
front of the camera right now" and recompute from scratch every frame; a bridge
for either can drop a message with no consequence. FastMapping ACCUMULATES, so
its output is the integral of everything it has seen. Two things follow:

  - Its `world/map` publisher is transient_local, i.e. latched, so a subscriber
    that connects late still receives the current map. This subscription has to
    declare the same durability or it matches nothing -- see _map_qos below.
  - A late or dropped map message is not a lost frame, it is a lost minute of
    accumulation. The bus publication is therefore unconditional: every grid the
    node emits goes out, and the consumer decides what to keep.
"""
from __future__ import annotations

import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher                             # noqa: E402

MAP_TOPIC = os.environ.get("FM_MAP_TOPIC", "/world/map")
# How often the grid is forwarded to the bus, at most. The node republishes the
# WHOLE grid on every update, and at 0.04 m a 20 m square is 250 k cells --
# 250 kB per message. At the node's update rate that would be tens of MB/s on a
# bus sized for camera frames, for a map that changes by a few cells at a time.
# The consumer is an overlay refreshed by hand, so 1 Hz is already generous.
PERIOD = float(os.environ.get("FM_PUBLISH_PERIOD", "1.0"))


class FastMappingBridge(Node):
    """ROS in, bus out. Nothing goes the other way."""

    def __init__(self) -> None:
        super().__init__("edgebot_fastmapping_bridge")

        # Their publisher is KeepLast(1) + transient_local + reliable
        # (MapManager.cpp:142). All three have to be matched here. Durability is
        # the one that actually bites: a VOLATILE subscriber against a
        # TRANSIENT_LOCAL publisher is an incompatible pair in DDS, so the
        # subscription would sit silent with no error -- the same failure shape
        # as the etape B reliability mismatch, one policy over.
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, map_qos)

        self.bus_pub = Publisher()
        self._got = 0
        self._sent = 0
        self._last_pub = 0.0
        self._last_report = time.time()
        self.get_logger().info(
            f"fastmapping bridge up, listening on {MAP_TOPIC} "
            f"(reliable, transient_local, depth 1), forwarding to the bus as "
            f"{topics.SUITE_MAP} at most every {PERIOD:.1f} s")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._got += 1
        now = time.time()
        if now - self._last_pub < PERIOD:
            return
        self._last_pub = now

        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution)
        if w <= 0 or h <= 0 or res <= 0.0:
            return

        # int8 on the wire, exactly as ROS defines it: -1 unknown, 0 free,
        # 100 occupied. Not remapped to anything friendlier here -- a consumer
        # that wants three colours can threshold, and a consumer that wants to
        # know how much is unknown needs the -1 to survive.
        grid = np.asarray(msg.data, dtype=np.int8)
        known = int(np.count_nonzero(grid >= 0))
        occupied = int(np.count_nonzero(grid >= 50))

        # The origin is the CORNER of cell (0, 0) in ROS; the bus carries the
        # CENTRE, so a consumer indexes cells without having to remember which
        # convention this producer used. Half a cell is 2 cm here and would be
        # invisible in an overlay, which is exactly why it would never be found
        # if it were wrong.
        self.bus_pub.send(topics.SUITE_MAP, {
            "grid": grid.tobytes(),
            "w": w, "h": h, "res": res,
            "x0": float(msg.info.origin.position.x) + 0.5 * res,
            "y0": float(msg.info.origin.position.y) + 0.5 * res,
            "known": known, "occupied": occupied,
            "stamp": now,
        })
        self._sent += 1

        if now - self._last_report > 10.0:
            self._last_report = now
            self.get_logger().info(
                f"{self._got} map(s) in, {self._sent} out | {w}x{h} at "
                f"{res:.3f} m, origin ({msg.info.origin.position.x:.2f}, "
                f"{msg.info.origin.position.y:.2f}) in {msg.header.frame_id} | "
                f"{known} cell(s) known, {occupied} occupied "
                f"({100.0 * known / max(1, w * h):.1f}% of the grid seen)")


def main() -> None:
    rclpy.init()
    node = FastMappingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus_pub.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
