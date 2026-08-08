#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Fire N planning requests at one leg and report success rate and time.

Run INSIDE the itsplanner container: it is an action client of planner_server,
and the point is to measure the planner as configured, not to re-implement it.

    ros2 run ... is not involved; this is plain rclpy.
    python3 /scripts/its_sweep.py --start 6.10,0.42 --goal 4.14,-1.34 -n 10
"""
from __future__ import annotations

import argparse
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def pose(node, xy, frame):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = xy
    p.pose.orientation.w = 1.0
    return p


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default="6.10,0.42")
    ap.add_argument("--goal", default="4.14,-1.34")
    ap.add_argument("--frame", default="map")
    ap.add_argument("-n", type=int, default=10)
    a = ap.parse_args()
    st = tuple(float(v) for v in a.start.split(","))
    go = tuple(float(v) for v in a.goal.split(","))

    rclpy.init()
    node = Node("its_sweep")
    cli = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not cli.wait_for_server(timeout_sec=60.0):
        print("planner_server never came up")
        return 1

    ok, times, lengths = 0, [], []
    for i in range(a.n):
        g = ComputePathToPose.Goal()
        g.start = pose(node, st, a.frame)
        g.goal = pose(node, go, a.frame)
        g.use_start = True
        t0 = time.perf_counter()
        fut = cli.send_goal_async(g)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
        handle = fut.result()
        n_pts = 0
        if handle is not None and handle.accepted:
            rf = handle.get_result_async()
            rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
            res = rf.result()
            if res is not None:
                n_pts = len(res.result.path.poses)
        dt = (time.perf_counter() - t0) * 1e3
        if n_pts >= 2:
            ok += 1
            times.append(dt)
            pts = [(p.pose.position.x, p.pose.position.y)
                   for p in res.result.path.poses]
            lengths.append(sum(((b[0] - c[0]) ** 2 + (b[1] - c[1]) ** 2) ** 0.5
                               for b, c in zip(pts, pts[1:])))
        print(f"  attempt {i + 1:2d}: "
              + (f"{n_pts:3d} poses, {dt:6.0f} ms" if n_pts >= 2
                 else f"FAILED         {dt:6.0f} ms"))
    print(f"\nsuccess {ok}/{a.n}"
          + (f" | plan time median {statistics.median(times):.0f} ms, "
             f"max {max(times):.0f} ms | length median "
             f"{statistics.median(lengths):.2f} m" if times else ""))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
