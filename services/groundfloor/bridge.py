#!/usr/bin/env python3
# Copyright (C) 2026 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
"""Bridge between this project's ZeroMQ bus and ROS 2.

Feeds Intel's `pointcloud_groundfloor_segmentation` node with the depth frames
the `source` service already publishes, and returns its obstacle footprints to
the bus so they can be compared against the ones this project computes.

Why a bridge rather than a rewrite. The suite's bricks are ROS 2 nodes and a
real G1 is driven over ROS 2, so ROS has to enter the picture eventually. Doing
it as one extra service leaves the demo running throughout and lets the bricks
be adopted one at a time, which a rewrite would not.

Why the depth image rather than a point cloud. Their node accepts
`<sensor>/depth/image_rect_raw` plus a `camera_info`, which is exactly what the
bus already carries. Producing a PointCloud2 here would add a conversion, and a
second place for the intrinsics to disagree.

The node runs BESIDE the existing floor detection rather than replacing it.
Step B is a measurement: are their footprints the same as ours, and what does
the round trip cost.
"""
from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, "/opt/edgebot")
from edgebot import topics                                    # noqa: E402
from edgebot.bus import Publisher, Subscriber                 # noqa: E402
from edgebot.pointcloud import (footprints, floor_polygon,   # noqa: E402
                                read_xyz, read_xyz_label)

CALIB = os.environ.get("CAMERA_CALIBRATION", "/config/camera_calibration.json")
SENSOR = os.environ.get("GF_SENSOR_NAME", "camera")
MARGIN = float(os.environ.get("OBSTACLE_MARGIN", "0.20"))
# Which class in their labelled cloud is the ground. Measured rather than read
# out of a config: over 1.5 M points the label-3 class sits at z median 0.060 m
# with a 0.094 m spread, while every other class sits at 0.77 m or above.
GROUND_LABEL = int(os.environ.get("GF_GROUND_LABEL", "3"))
FLOOR_CELL = float(os.environ.get("GF_FLOOR_CELL", "0.10"))
# Physically impossible returns, dropped before anything is built from them.
# Measured on the live cloud: z reached -6.096 m, six metres below a floor the
# camera is standing on, and x reached 14.378 m in a room whose far wall is at
# 6.2 m. Both are depth noise, and both inflate a bounding box for free.
Z_MIN = float(os.environ.get("GF_Z_MIN", "-0.3"))
X_MAX = float(os.environ.get("GF_X_MAX", "8.0"))


def _quaternion_from_pitch(pitch_rad: float):
    """Rotation about the body Y axis, nose down for a positive pitch."""
    half = pitch_rad / 2.0
    return (0.0, math.sin(half), 0.0, math.cos(half))


class GroundfloorBridge(Node):
    """Bus in, ROS out, ROS in, bus out."""

    def __init__(self) -> None:
        super().__init__("edgebot_groundfloor_bridge")

        import json
        try:
            with open(CALIB) as fh:
                calib = json.load(fh)
        except Exception as exc:
            self.get_logger().error(
                f"no calibration at {CALIB} ({exc}). The segmentation needs a "
                f"complete transform between base_link and the camera, so it "
                f"cannot run without one. Run 'make calibrate' first.")
            raise SystemExit(1)

        self.height = float(calib.get("camera_height_m", 1.56))
        self.pitch = math.radians(abs(float(calib.get("pitch_deg", 0.0))))
        self.fx = float(calib.get("fx", 386.4))
        self.fy = float(calib.get("fy", 386.5))
        self.ppx = float(calib.get("ppx", 325.6))
        self.ppy = float(calib.get("ppy", 239.6))

        # Best effort matches what a camera driver would use, and their node
        # takes `use_best_effort_qos` for the same reason: a dropped depth frame
        # is better than a stalled pipeline.
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.depth_pub = self.create_publisher(
            Image, f"/{SENSOR}/depth/image_rect_raw", qos)
        self.info_pub = self.create_publisher(
            CameraInfo, f"/{SENSOR}/depth/camera_info", qos)
        self.create_subscription(
            PointCloud2, "/segmentation/obstacle_points", self._on_obstacles, qos)
        # Their primary product. obstacle_points is a filtered view of this, so
        # the ground class exists only here.
        self.create_subscription(
            PointCloud2, "/segmentation/labeled_points", self._on_labeled, qos)

        self._publish_static_tf()

        self.bus_pub = Publisher()
        self.bus_sub = Subscriber([topics.CAMERA_DEPTH])
        self.create_timer(0.01, self._pump_bus)

        self._sent = 0
        self._got = 0
        self._floor = 0
        self._floor_pts = 0
        self._floor_vertices = 0
        self._floor_dropped = 0
        self._obs_dropped = 0
        self._stamps: dict = {}
        self._last_report = time.time()
        self.get_logger().info(
            f"bridge up: camera at {self.height:.2f} m, pitch "
            f"{math.degrees(self.pitch):.1f} deg, feeding /{SENSOR}/depth")

    def _publish_static_tf(self) -> None:
        """base_link -> camera frame, from the calibration.

        Their node requires a complete transform between the sensor frame and
        `base_frame`, and computes heights in that frame. Since this camera is
        fixed and looks at the floor from a tripod, base_link is placed on the
        floor directly below it, which is also this project's world origin. The
        two coordinate systems then agree by construction rather than by a
        conversion someone has to remember.
        """
        self._tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = f"{SENSOR}_depth_optical_frame"
        t.transform.translation.z = self.height
        # Optical frames are x right, y down, z forward, while body frames are
        # x forward, y left, z up. The fixed part of this rotation is that
        # change of axes; the pitch is added on top.
        qx, qy, qz, qw = self._optical_quaternion(self.pitch)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf.sendTransform(t)

    @staticmethod
    def _optical_quaternion(pitch: float):
        """Body-to-optical rotation, with the camera pitched nose down."""
        # R = Rz(-90) * Rx(-90) composed with a pitch about the optical x axis.
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        # Base optical rotation as a quaternion (x, y, z, w).
        bx, by, bz, bw = -0.5, 0.5, -0.5, 0.5
        # Rotation about the optical X axis, which points to the camera's
        # right. A camera tilted nose DOWN needs a NEGATIVE rotation here:
        # with a positive one the forward axis came back as (+0.97, 0, +0.24),
        # pointing at the ceiling, and the segmentation would have measured
        # every height against a plane above the camera.
        px, py, pz, pw = -sp, 0.0, 0.0, cp
        return (bw * px + bx * pw + by * pz - bz * py,
                bw * py - bx * pz + by * pw + bz * px,
                bw * pz + bx * py - by * px + bz * pw,
                bw * pw - bx * px - by * py - bz * pz)

    def _pump_bus(self) -> None:
        """Forward every depth frame from the bus into ROS."""
        for _ in range(4):
            msg = self.bus_sub.recv(0)
            if msg is None:
                return
            topic, payload = msg
            if topic != topics.CAMERA_DEPTH:
                continue
            w, h = int(payload["w"]), int(payload["h"])
            scale = float(payload.get("scale", 0.001))
            now = self.get_clock().now().to_msg()

            img = Image()
            img.header.stamp = now
            img.header.frame_id = f"{SENSOR}_depth_optical_frame"
            img.height, img.width = h, w
            # 16UC1 in millimetres is what a RealSense publishes and what their
            # node expects. The bus carries the sensor's native Z16 with a
            # scale, so convert only if that scale is not already millimetres.
            depth = np.frombuffer(payload["depth"], np.uint16).reshape(h, w)
            if abs(scale - 0.001) > 1e-6:
                depth = (depth.astype(np.float32) * scale * 1000.0).astype(np.uint16)
            img.encoding = "16UC1"
            img.is_bigendian = 0
            img.step = w * 2
            img.data = depth.tobytes()
            self.depth_pub.publish(img)

            info = CameraInfo()
            info.header = img.header
            info.height, info.width = h, w
            sx, sy = w / 640.0, h / 480.0
            info.k = [self.fx * sx, 0.0, self.ppx * sx,
                      0.0, self.fy * sy, self.ppy * sy,
                      0.0, 0.0, 1.0]
            info.p = [self.fx * sx, 0.0, self.ppx * sx, 0.0,
                      0.0, self.fy * sy, self.ppy * sy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            self.info_pub.publish(info)

            self._sent += 1
            self._stamps[(now.sec, now.nanosec)] = time.perf_counter()
            if len(self._stamps) > 64:
                self._stamps.pop(next(iter(self._stamps)))

    @staticmethod
    def _plausible(pts):
        """Drop returns the room cannot contain. Returns (kept, n_dropped).

        A depth sensor emits some points that no geometry explains: measured on
        this cloud, z went down to -6.096 m and x out to 14.378 m, against a
        floor at 0 and a far wall at 6.2 m. They are a small fraction of the
        cloud and they do not move a median, but a bounding box is a max, so a
        single one of them stretches a footprint across the room for free.

        Only the two impossible directions are cut. Points ABOVE the robot are
        left alone: a low ceiling or a shelf is a real obstacle to a walking
        machine, and deciding what is too high is the navigator's business.
        """
        if pts.size == 0:
            return pts, 0
        ok = np.isfinite(pts).all(axis=1)
        ok &= pts[:, 2] >= Z_MIN
        ok &= pts[:, 0] <= X_MAX
        return pts[ok], int((~ok).sum())

    def _on_labeled(self, msg: PointCloud2) -> None:
        """Turn the ground class of their labelled cloud into a floor outline."""
        pts, labels = read_xyz_label(msg.fields, msg.point_step, msg.row_step,
                                     msg.width, msg.height, bytes(msg.data),
                                     bool(msg.is_bigendian))
        if pts.shape[0] == 0:
            return
        ground, dropped = self._plausible(pts[labels == GROUND_LABEL])
        poly = floor_polygon(ground, cell=FLOOR_CELL)
        self._floor_dropped += dropped
        self._floor_pts = int(ground.shape[0])
        self._floor_vertices = len(poly)
        self._floor += 1
        self.bus_pub.send(topics.GROUNDFLOOR_FLOOR, {
            "poly": poly, "points": int(ground.shape[0]),
            "stamp": time.time()})

    def _on_obstacles(self, msg: PointCloud2) -> None:
        """Turn their obstacle cloud into footprints and put it on the bus."""
        pts = read_xyz(msg.fields, msg.point_step, msg.row_step,
                       msg.width, msg.height, bytes(msg.data),
                       bool(msg.is_bigendian))
        pts, dropped = self._plausible(pts)
        self._obs_dropped += dropped
        boxes = footprints(pts, margin=MARGIN)
        self.bus_pub.send(topics.GROUNDFLOOR_OBSTACLES, {
            "blocked": boxes, "points": int(pts.shape[0]),
            "stamp": time.time()})

        self._got += 1
        sent_at = self._stamps.pop(
            (msg.header.stamp.sec, msg.header.stamp.nanosec), None)
        if sent_at is not None:
            self._latency = (time.perf_counter() - sent_at) * 1000.0
        if time.time() - self._last_report > 5.0:
            self._last_report = time.time()
            lat = getattr(self, "_latency", float("nan"))
            self.get_logger().info(
                f"{self._sent} depth frames in, {self._got} segmentations out, "
                f"{pts.shape[0]} obstacle points, {len(boxes)} footprint(s), "
                f"round trip {lat:.0f} ms | floor: {self._floor} msg(s), "
                f"{self._floor_pts} ground points, {self._floor_vertices} "
                f"vertices | dropped as impossible: {self._obs_dropped} "
                f"obstacle, {self._floor_dropped} ground "
                f"(z < {Z_MIN} or x > {X_MAX})")


def main() -> None:
    rclpy.init()
    node = GroundfloorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
