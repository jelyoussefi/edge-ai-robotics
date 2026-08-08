#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Drive Intel's ITS Path Planner and put the path it returns on the bus.

Three jobs, and the first one is a naming decision worth reading before the
code.

THE FRAME NAMING, DELIBERATE AND TEMPORARY
------------------------------------------
This project's `base_link` is the WORLD: the origin is the ground point under
the camera, x forward, y left, z up, and it never moves because the camera is on
a tripod. That was the right call for etapes B and C -- it made ADBSCAN's
clusters come back needing no transform at all.

Nav2 means something else by `base_link`. There it is the ROBOT BODY, and the
convention is map -> odom -> base_link with base_link moving. A global costmap
is built in `map` and asks TF where `base_link` is in order to know where the
robot stands.

For E1 the two meanings are reconciled by making them numerically identical:
FastMapping is switched to publish its grid in `map` (FM_MAP_FRAME), and this
node publishes a STATIC IDENTITY map -> base_link. Nothing about the geometry
changes -- the map origin was already the world origin -- only the label. Nav2
then sees a perfectly stationary robot sitting at the world origin, which is
exactly true: there is no robot in the loop yet.

WHAT E2 MUST DO. This identity is a placeholder and must not survive contact
with a moving robot. In E2 `base_link` has to become the robot body, which means
this static transform is REPLACED by map -> odom -> base_link driven from the
sim pose, and the world frame keeps its meaning under the name `map`. Everything
downstream of that rename is already written in terms of `map`, so the change is
confined to who publishes base_link.
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher                             # noqa: E402

MAP_FRAME = os.environ.get("ITS_MAP_FRAME", "map")
BASE_FRAME = os.environ.get("ITS_BASE_FRAME", "base_link")
MAP_TOPIC = os.environ.get("ITS_MAP_TOPIC", "/world/map")
# The E1 acceptance request: across the barrier side of the arena, so a straight
# line between the two is not a valid answer.
START = (float(os.environ.get("ITS_START_X", "1.8")),
         float(os.environ.get("ITS_START_Y", "0.0")))
GOAL = (float(os.environ.get("ITS_GOAL_X", "5.8")),
        float(os.environ.get("ITS_GOAL_Y", "-1.8")))
PERIOD = float(os.environ.get("ITS_REQUEST_PERIOD", "5.0"))


class ItsPlannerBridge(Node):
    """Static TF in, path request out, path onto the bus."""

    def __init__(self) -> None:
        super().__init__("edgebot_itsplanner_bridge")

        # Published before anything else. planner_server's costmap needs
        # map -> base_link to exist the moment it activates, and a transform
        # that shows up late leaves it stuck reporting that it cannot find the
        # robot -- which reads like a broken costmap rather than a missing TF.
        self._static = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = BASE_FRAME
        t.transform.rotation.w = 1.0          # identity, see the module docstring
        self._static.sendTransform(t)
        self.get_logger().info(
            f"static TF {MAP_FRAME} -> {BASE_FRAME} = identity. E1 only: "
            f"base_link is this project's world frame and there is no robot in "
            f"the loop. E2 replaces this with map -> odom -> base_link from the "
            f"sim pose.")

        # The map is kept here too, to measure clearance against the same
        # occupancy the planner used rather than against a later one.
        self._map = None
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._client = ActionClient(self, ComputePathToPose,
                                    "compute_path_to_pose")
        self.bus_pub = Publisher()
        self._asked = 0
        self._got = 0
        self._pending = False
        self.create_timer(PERIOD, self._request)
        self.get_logger().info(
            f"will ask for a path from {START} to {GOAL} every {PERIOD:.0f} s")

    def _on_map(self, msg: OccupancyGrid) -> None:
        w, h = int(msg.info.width), int(msg.info.height)
        if w <= 0 or h <= 0:
            return
        self._map = {
            "grid": np.asarray(msg.data, np.int8).reshape(h, w),
            "res": float(msg.info.resolution),
            "x0": float(msg.info.origin.position.x),
            "y0": float(msg.info.origin.position.y),
        }

    def _pose(self, xy) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = MAP_FRAME
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(xy[0]), float(xy[1])
        p.pose.orientation.w = 1.0
        return p

    def _request(self) -> None:
        if self._pending:
            return
        if not self._client.server_is_ready():
            self._client.wait_for_server(timeout_sec=0.0)
            if not self._client.server_is_ready():
                self.get_logger().info(
                    "compute_path_to_pose not up yet (planner_server "
                    "activating?)", throttle_duration_sec=10.0)
                return
        goal = ComputePathToPose.Goal()
        goal.start = self._pose(START)
        goal.goal = self._pose(GOAL)
        # use_start true: plan from the requested start, NOT from where TF says
        # the robot is. In E1 the robot is at the origin by construction, so
        # without this every path would begin at (0, 0) -- outside the mapped
        # floor -- and be rejected.
        goal.use_start = True
        self._pending = True
        self._asked += 1
        self._client.send_goal_async(goal).add_done_callback(self._accepted)

    def _accepted(self, fut) -> None:
        handle = fut.result()
        if not handle.accepted:
            self._pending = False
            self.get_logger().warning("planner rejected the goal")
            return
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, fut) -> None:
        self._pending = False
        try:
            path = fut.result().result.path
        except Exception as exc:
            self.get_logger().warning(f"no path: {exc}")
            return
        pts = [(float(p.pose.position.x), float(p.pose.position.y))
               for p in path.poses]
        if len(pts) < 2:
            self.get_logger().warning(
                f"path has {len(pts)} pose(s), nothing to publish")
            return
        self._got += 1
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        clearance = self._clearance(pts)
        self.bus_pub.send(topics.SUITE_PATH, {
            "path": [[round(x, 3), round(y, 3)] for x, y in pts],
            "length_m": round(length, 3),
            "clearance_m": round(clearance, 3) if clearance is not None else -1.0,
            "start": list(START), "goal": list(GOAL),
            "planner": "its_planner::ITSPlanner",
            "stamp": time.time(),
        })
        self.get_logger().info(
            f"path {self._got}/{self._asked}: {len(pts)} waypoint(s), "
            f"{length:.2f} m, min clearance "
            + (f"{clearance:.3f} m" if clearance is not None
               else "unknown (no map)")
            + f", straight line would be {math.dist(START, GOAL):.2f} m")

    def _clearance(self, pts):
        """Smallest distance from any waypoint to an occupied cell.

        Measured against the map, not against the costmap. The costmap has the
        inflation layer baked in, so a path that merely respects inflation would
        score as clear by construction -- it would be marking its own homework.
        The question the acceptance criterion asks is a geometric one about the
        occupancy the map actually holds.
        """
        m = self._map
        if m is None:
            return None
        occ = m["grid"] >= 50
        if not occ.any():
            return None
        rows, cols = np.nonzero(occ)
        ox = m["x0"] + (cols + 0.5) * m["res"]
        oy = m["y0"] + (rows + 0.5) * m["res"]
        best = float("inf")
        for x, y in pts:
            d = np.min(np.hypot(ox - x, oy - y))
            best = min(best, float(d))
        return best


def main() -> None:
    rclpy.init()
    node = ItsPlannerBridge()
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
