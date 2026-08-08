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

WHAT E2 DID, and it is not what E1 predicted. E1 assumed base_link would have to
BECOME the robot body. It does not, and renaming it would have been the wrong
call: every footprint, arena bound and calibration figure written since etape B
reads world coordinates out of base_link, and silently moving that frame would
have moved all of them at once, with nothing failing loudly.

So E2 gives Nav2 what it actually needs under names of its own -- a moving
`robot_base` for the body and an `odom` between it and the map -- and leaves
base_link alone as the world. Three transforms now:

    map -> base_link    static identity, the world, unchanged since E1
    map -> odom         static identity, because the sim pose is ground truth;
                        odom exists to hold accumulated drift and a simulator
                        has none. On a real G1 this becomes the localisation
                        correction and nothing else here changes.
    odom -> robot_base  dynamic, from the sim pose at 20 Hz

The costmap's robot_base_frame is robot_base, so Nav2 tracks the robot while the
world keeps its name.
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
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher, Subscriber                  # noqa: E402

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
# E2. The robot's own frame, and NOT base_link: base_link keeps its meaning as
# this project's world, static, floor under the camera. Everything written since
# etape B reads world coordinates out of it and renaming it would silently move
# every footprint, every arena bound and every calibration figure.
#
# So Nav2 gets what it actually needs -- a moving frame for the robot body and
# an odom frame between it and the map -- under names of its own, and the world
# keeps its name. map -> odom is a static identity because the sim pose is
# ground truth: odom exists to hold accumulated drift, and a simulator has none.
# On a real G1 that identity becomes the localisation correction and nothing
# else about this file changes.
ROBOT_FRAME = os.environ.get("ITS_ROBOT_FRAME", "robot_base")
ODOM_FRAME = os.environ.get("ITS_ODOM_FRAME", "odom")
# Three distinct goals, cycled as each is reached. Semicolon-separated pairs.
GOALS = [tuple(float(v) for v in g.split(","))
         for g in os.environ.get(
             "ITS_GOALS", "2.1,-1.9;4.6,-1.8;2.6,0.9").split(";") if g.strip()]
GOAL_TOL = float(os.environ.get("GOAL_TOL", "0.45"))
# Clearance the final approach segment must keep from occupied cells. The same
# 0.22 m the costmap and the navigator use for the robot's half width.
APPROACH_CLEAR = float(os.environ.get("ITS_APPROACH_CLEAR", "0.22"))


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
        t2 = TransformStamped()
        t2.header.stamp = t.header.stamp
        t2.header.frame_id = MAP_FRAME
        t2.child_frame_id = ODOM_FRAME
        t2.transform.rotation.w = 1.0
        self._static.sendTransform([t, t2])
        self.get_logger().info(
            f"static TF {MAP_FRAME} -> {BASE_FRAME} (the world, unchanged) and "
            f"{MAP_FRAME} -> {ODOM_FRAME}, both identity; "
            f"{ODOM_FRAME} -> {ROBOT_FRAME} follows the sim pose")

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
        self.bus_sub = Subscriber([topics.ROBOT_STATE])
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf = TransformBroadcaster(self)
        self._robot = None          # (x, y, yaw) from the sim, world frame
        self._goal_i = 0
        self._reached = [False] * len(GOALS)
        self._t_goal = time.time()
        self._asked = 0
        self._got = 0
        self._pending = False
        # Pose forwarding runs far faster than planning: the costmap and any
        # future controller want a live transform, the planner does not need a
        # fresh route every frame.
        self.create_timer(0.05, self._pump_pose)
        self.create_timer(PERIOD, self._request)
        self.get_logger().info(
            f"E2: following the sim pose, goals {GOALS}, tolerance "
            f"{GOAL_TOL} m, replanning every {PERIOD:.0f} s")

    def _pump_pose(self) -> None:
        """Drain the bus for the sim pose, then publish TF and /odom."""
        while (msg := self.bus_sub.recv(0)) is not None:
            topic, payload = msg
            if topic != topics.ROBOT_STATE:
                continue
            q = payload.get("qpos") or []
            if len(q) < 7:
                continue
            # qpos is [x, y, z, qw, qx, qy, qz, ...]; only the yaw matters here.
            yaw = 2.0 * math.atan2(float(q[6]), float(q[3]))
            self._robot = (float(q[0]), float(q[1]), yaw)
        if self._robot is None:
            return
        x, y, yaw = self._robot
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = ODOM_FRAME
        t.child_frame_id = ROBOT_FRAME
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf.sendTransform(t)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = ODOM_FRAME
        od.child_frame_id = ROBOT_FRAME
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = t.transform.rotation.z
        od.pose.pose.orientation.w = t.transform.rotation.w
        self.odom_pub.publish(od)

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
        if self._pending or self._robot is None:
            return
        # Advance the goal when the robot has arrived, so the acceptance run is
        # three goals and not one repeated. Reported here as well as in the
        # navigator: this side knows the planning time, that side knows the
        # tracking error, and neither is the whole story.
        gx, gy = GOALS[self._goal_i]
        d = math.dist(self._robot[:2], (gx, gy))
        if d <= GOAL_TOL:
            if not self._reached[self._goal_i]:
                self._reached[self._goal_i] = True
                self.get_logger().info(
                    f"goal {self._goal_i + 1}/{len(GOALS)} ({gx}, {gy}) "
                    f"reached: {d:.3f} m, {time.time() - self._t_goal:.1f} s")
            if self._goal_i + 1 < len(GOALS):
                self._goal_i += 1
                self._t_goal = time.time()
            else:
                return
        if not self._client.server_is_ready():
            self._client.wait_for_server(timeout_sec=0.0)
            if not self._client.server_is_ready():
                self.get_logger().info(
                    "compute_path_to_pose not up yet (planner_server "
                    "activating?)", throttle_duration_sec=10.0)
                return
        goal = ComputePathToPose.Goal()
        # Start from WHERE THE ROBOT IS, which is the whole difference between
        # E1 and E2: in E1 the start was a constant and the robot did not exist.
        goal.start = self._pose(self._robot[:2])
        goal.goal = self._pose(GOALS[self._goal_i])
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
        goal = GOALS[self._goal_i]
        # A one-pose path is the planner saying start and goal are the same
        # node. Seed the approach from the ROBOT instead, so the last leg is
        # still built and checked rather than the request being dropped.
        if len(pts) < 2 and self._robot is not None:
            pts = [self._robot[:2]]
        pts, note = self._approach(pts, goal)
        if len(pts) < 2:
            self.get_logger().warning(
                f"path has {len(pts)} pose(s) after approach ({note}), "
                f"nothing to publish")
            return
        if note:
            self.get_logger().info("%s", note)
        self._got += 1
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        clearance = self._clearance(pts)
        self.bus_pub.send(topics.SUITE_PATH, {
            "path": [[round(x, 3), round(y, 3)] for x, y in pts],
            "length_m": round(length, 3),
            "clearance_m": round(clearance, 3) if clearance is not None else -1.0,
            "start": list(self._robot[:2]) if self._robot else list(START),
            "goal": list(GOALS[self._goal_i]),
            "planner": "its_planner::ITSPlanner",
            "stamp": time.time(),
        })
        self.get_logger().info(
            f"path {self._got}/{self._asked}: {len(pts)} waypoint(s), "
            f"{length:.2f} m, min clearance "
            + (f"{clearance:.3f} m" if clearance is not None
               else "unknown (no map)")
            + f", to goal {self._goal_i + 1}/{len(GOALS)} "
            + f"{GOALS[self._goal_i]}")

    def _segment_clear(self, a, b, need):
        """Whether a straight segment keeps `need` metres from occupancy.

        Sampled every half cell. Coarser would step over a single occupied cell,
        which at 0.04 m is a table leg -- the exact thing this must not miss.
        """
        m = self._map
        if m is None:
            return False
        occ = m["grid"] >= 50
        if not occ.any():
            return True
        rows, cols = np.nonzero(occ)
        ox = m["x0"] + cols * m["res"]
        oy = m["y0"] + rows * m["res"]
        n = max(2, int(math.dist(a, b) / (0.5 * m["res"])))
        for t in np.linspace(0.0, 1.0, n):
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            if float(np.min(np.hypot(ox - x, oy - y))) < need:
                return False
        return True

    def _approach(self, pts, goal):
        """Extend a plan to the EXACT goal with a checked straight segment.

        Why this exists. The roadmap places its nodes where it samples, so a
        path ends at the node nearest the goal and starts at the node nearest
        the robot -- 0.30 to 0.45 m off at each end, measured. That is tolerable
        at the start and fatal at the end: goal 3 of the E2 acceptance failed
        with the planner returning a path of ONE pose, its way of saying start
        and goal fall on the same node and there is nothing to route between.
        The tolerance and the node spacing are therefore not independent, and no
        amount of sampling fixes it -- 2000 samples changed nothing.

        So the last leg is ours, not theirs: a straight line from where their
        plan ends to where the goal actually is, and it is CHECKED. If the map
        says that line is not clear the plan is left alone and the robot stops
        short, which is the honest outcome -- a planner that cannot reach a goal
        should say so rather than have us draw the last metre for it.
        """
        if not pts:
            return pts, "no plan"
        gap = math.dist(pts[-1], goal)
        if gap < 0.05:
            return pts, ""
        if not self._segment_clear(pts[-1], goal, APPROACH_CLEAR):
            return pts, f"final {gap:.2f} m to the goal is not clear, stopping short"
        n = max(2, int(gap / 0.05))
        tail = [(pts[-1][0] + (goal[0] - pts[-1][0]) * t,
                 pts[-1][1] + (goal[1] - pts[-1][1]) * t)
                for t in np.linspace(0.0, 1.0, n)[1:]]
        return pts + [(float(a), float(b)) for a, b in tail], \
            f"final approach {gap:.2f} m appended"

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
