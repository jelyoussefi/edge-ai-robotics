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

Two jobs, both small on purpose. Read their clusters, drop the impossible ones,
put the rest on the bus in the same rectangle format everything else here uses.
And, on the way in, clip the cloud to the arena interior before their node sees
it -- see ARENA_* below for why a wall is worse than an obstacle here.

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
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher                             # noqa: E402
from edgebot.pointcloud import clip_xy                        # noqa: E402

# Identical to the compositor's and the groundfloor bridge's, or the three
# footprint sets are not comparable.
MARGIN = float(os.environ.get("OBSTACLE_MARGIN", "0.20"))
# Same impossible-return guards as the groundfloor bridge, applied to cluster
# centres here rather than to points.
Z_MIN = float(os.environ.get("GF_Z_MIN", "-0.3"))
X_MAX = float(os.environ.get("GF_X_MAX", "8.0"))
CLUSTER_TOPIC = os.environ.get("ADBSCAN_TOPIC", "/obstacle_array")

# The arena interior. groundfloor removes the floor, not the walls, so the far
# wall and the side walls survive into obstacle_points as large connected
# sheets -- and DBSCAN chains through whatever touches. A chair against the wall
# is not a chair any more, it is one mass with the wall and half the room. The
# navigator never plans outside these bounds, so nothing is lost by cutting
# them before their node runs rather than discarding the clusters afterwards:
# by then the damage is done, the chair is already inside the merged cluster.
#
# Slightly WIDER than suite_compare.py's arena (x 1.5-6.5, y -2.6 to 1.5), on
# purpose. Clipping exactly at the comparison boundary would truncate the very
# clusters the comparison scores, and their extent would then be an artefact of
# this filter rather than of their detector.
ARENA_X_MIN = float(os.environ.get("ARENA_X_MIN", "1.2"))
ARENA_X_MAX = float(os.environ.get("ARENA_X_MAX", "6.3"))
ARENA_Y_MIN = float(os.environ.get("ARENA_Y_MIN", "-2.8"))
ARENA_Y_MAX = float(os.environ.get("ARENA_Y_MAX", "1.7"))
CLOUD_IN = os.environ.get("ADBSCAN_CLOUD_IN", "/segmentation/obstacle_points")
CLOUD_OUT = os.environ.get("ADBSCAN_CLOUD_OUT", "/segmentation/arena_points")

# The residue of the true floor. groundfloor classifies point by point against
# an estimated plane, and what it leaves behind is a thin scatter lying ON the
# floor rather than a clean cut. Measured on 40 clouds of this room, after the
# arena clip and the same 1-in-75 subsample their node applies:
#
#   z band        pts/cloud   cells of 0.1 m   on open floor
#   0.08-0.12          45           38             71 %
#   0.08-0.20         111           68             61 %
#   > 0.30            845          309              -
#
# "on open floor" is the load-bearing column: cells the low band occupies with
# nothing above 0.30 m over them. Furniture keeps its mass high, so a low return
# with no body above it is not the base of an object -- it is floor the plane fit
# failed to claim. DBSCAN chains through contact, and a scatter across open
# floor is exactly the medium a chain needs, which is the hypothesis this filter
# tests.
#
# GF_Z_LOW is the top of the band; 0 is the bottom, and points BELOW the floor
# are deliberately kept (see clip_xy). Note their node already drops z < 0.08
# itself, through z_filter with Z_based_ground_removal -- verified in
# doDBSCAN.cpp:304, the 3D branch -- so the slice this actually removes is
# 0.08 to GF_Z_LOW, 45 of the 1011 points per cloud that reach the clusterer.
# Set GF_Z_LOW to 0 to disable.
Z_LOW = float(os.environ.get("GF_Z_LOW", "0.12"))


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

        # The cloud path. Both ends BEST_EFFORT: groundfloor publishes the
        # obstacle cloud that way and adbscan_sub subscribes with
        # rclcpp::SensorDataQoS(), which is BEST_EFFORT too, so this hop keeps
        # the sensor semantics of the topic it sits in the middle of. A dropped
        # cloud is better than a stalled one.
        cloud_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_pub = self.create_publisher(PointCloud2, CLOUD_OUT, cloud_qos)
        self.create_subscription(PointCloud2, CLOUD_IN, self._on_cloud, cloud_qos)

        self.bus_pub = Publisher()
        self._got = 0
        self._clusters = 0
        self._dropped = 0
        self._clouds = 0
        self._pts_in = 0
        self._pts_out = 0
        self._unreadable = 0
        self._last_report = time.time()
        self.get_logger().info(
            f"adbscan bridge up, listening on {CLUSTER_TOPIC}. Input is "
            f"groundfloor's /segmentation/obstacle_points, so positions arrive "
            f"in base_link and need no transform.")
        self.get_logger().info(
            f"clipping {CLOUD_IN} -> {CLOUD_OUT} at x {ARENA_X_MIN}..."
            f"{ARENA_X_MAX}, y {ARENA_Y_MIN}...{ARENA_Y_MAX}"
            + (f", and dropping the 0 to {Z_LOW} m floor residue"
               if Z_LOW > 0 else ", no height band dropped")
            + f". The uncut cloud stays on {CLOUD_IN}, untouched.")

    def _on_cloud(self, msg: PointCloud2) -> None:
        """Republish the cloud with everything outside the arena removed.

        A separate topic rather than filtering in place: the input is the
        groundfloor node's own output and stays exactly as it publishes it, so
        the uncut cloud remains available to rviz, `ros2 topic echo` and the
        compositor's point-cloud overlay without this node having to copy it.
        """
        clipped = clip_xy(msg.fields, msg.point_step, msg.row_step, msg.width,
                          msg.height, msg.data, msg.is_bigendian,
                          ARENA_X_MIN, ARENA_X_MAX, ARENA_Y_MIN, ARENA_Y_MAX,
                          0.0, Z_LOW)
        self._clouds += 1
        self._pts_in += msg.width * msg.height
        if clipped is None:
            # No x/y columns to test. Pass it through rather than publish an
            # empty cloud: "unreadable" must not reach their node as "clear".
            self._unreadable += 1
            self.cloud_pub.publish(msg)
            return

        data, count = clipped
        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.point_step = msg.point_step
        out.is_bigendian = msg.is_bigendian
        # Unordered: the surviving points are scattered through the original
        # image grid, so there is no height/width left to preserve. Their node
        # reads width * height points and does not care about the layout.
        out.height = 1
        out.width = count
        out.row_step = msg.point_step * count
        out.data = data
        out.is_dense = True
        self.cloud_pub.publish(out)
        self._pts_out += count

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
            kept = (100.0 * self._pts_out / self._pts_in) if self._pts_in else 0.0
            # The kept fraction is the one number that says whether the arena
            # bounds are sane: near 100 % means the clip is doing nothing, and
            # very low means it is eating the room rather than its edges.
            self.get_logger().info(
                f"{self._clouds} cloud(s) clipped, {kept:.0f}% of points kept "
                f"inside the arena"
                + (f", {self._unreadable} passed through unclipped"
                   if self._unreadable else ""))


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
