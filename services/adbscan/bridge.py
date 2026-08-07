#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Put Intel's ADBSCAN clusters on this project's bus as obstacle footprints.

Chained behind the groundfloor node rather than fed a cloud of its own. ADBSCAN
removes the ground with one height threshold, which cannot work for a camera
pitched 14 degrees down: the floor ramps 1.22 m of z across the arena, so the
cut that clears the near floor keeps the far floor, and the far floor comes back
as a room-spanning cluster. groundfloor already removes it with a tilt-aware
plane fit. Composing the two bricks is the suite's own answer, and it is why
this bridge no longer publishes depth: services/groundfloor owns that path, and
one producer per topic is the point.

What is left is small on purpose -- read their clusters, drop the impossible
ones, put the rest on the bus in the same rectangle format everything else here
uses.

Why the comparison target is right. ADBSCAN produces obstacle clusters and so do
we; both answer "something is here, roughly this big". Etape B compared our
footprints against a floor segmenter's derived output, which measured
definitions rather than perception.
"""
from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from nav2_dynamic_msgs.msg import ObstacleArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher                             # noqa: E402

# Identical to the compositor's and the groundfloor bridge's, or the three
# footprint sets are not comparable.
MARGIN = float(os.environ.get("OBSTACLE_MARGIN", "0.20"))
# Same impossible-return guards as the groundfloor bridge, applied to cluster
# centres here rather than to points.
Z_MIN = float(os.environ.get("GF_Z_MIN", "-0.3"))
X_MAX = float(os.environ.get("GF_X_MAX", "8.0"))
CLUSTER_TOPIC = os.environ.get("ADBSCAN_TOPIC", "/obstacle_array")


class AdbscanBridge(Node):
    """ROS in, bus out."""

    def __init__(self) -> None:
        super().__init__("edgebot_adbscan_bridge")

        # Their publisher is a plain rclcpp::QoS(1), i.e. RELIABLE, so this
        # subscription has to be reliable too. Note this is the opposite of the
        # groundfloor depth path, where both ends are BEST_EFFORT: the mismatch
        # that cost 30 s of silence in etape B was on the sensor topics, not
        # here.
        self.create_subscription(
            ObstacleArray, CLUSTER_TOPIC, self._on_obstacles,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

        self.bus_pub = Publisher()
        self._got = 0
        self._clusters = 0
        self._dropped = 0
        self._last_report = time.time()
        self.get_logger().info(
            f"adbscan bridge up, listening on {CLUSTER_TOPIC}. Input is "
            f"groundfloor's /segmentation/obstacle_points, so positions arrive "
            f"in base_link and need no transform.")

    def _on_obstacles(self, msg: ObstacleArray) -> None:
        """Turn their clusters into footprints and put them on the bus.

        No frame conversion. With Lidar_type 3D the node clusters the cloud as
        given, and groundfloor publishes in base_link, whose origin is the
        ground point under the camera -- this project's world origin by
        construction. The earlier RS path needed a pitch rotation and a height
        lift precisely because it re-derived its own frame from the optical one.
        """
        boxes = []
        dropped = 0
        for ob in msg.obstacles:
            cx, cy, cz = ob.position.x, ob.position.y, ob.position.z
            hx, hy = abs(ob.size.x) / 2.0, abs(ob.size.y) / 2.0
            if not math.isfinite(cx + cy + cz + hx + hy):
                dropped += 1
                continue
            # A cluster whose centre is below the floor or past the far wall is
            # a depth artefact, and it would stretch a rectangle across the room
            # exactly as the raw returns did.
            if cz < Z_MIN or cx > X_MAX:
                dropped += 1
                continue
            boxes.append((round(cx - hx - MARGIN, 3), round(cx + hx + MARGIN, 3),
                          round(cy - hy - MARGIN, 3), round(cy + hy + MARGIN, 3)))

        self._got += 1
        self._clusters = len(boxes)
        self._dropped += dropped
        self.bus_pub.send(topics.SUITE_CLUSTERS, {
            "blocked": boxes, "clusters": len(msg.obstacles),
            "stamp": time.time()})

        if time.time() - self._last_report > 5.0:
            self._last_report = time.time()
            self.get_logger().info(
                f"{self._got} cluster sets out, {len(msg.obstacles)} cluster(s) "
                f"this frame, {self._clusters} footprint(s) after filtering, "
                f"{self._dropped} dropped as impossible "
                f"(z < {Z_MIN} or x > {X_MAX})")


def main() -> None:
    rclpy.init()
    node = AdbscanBridge()
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
